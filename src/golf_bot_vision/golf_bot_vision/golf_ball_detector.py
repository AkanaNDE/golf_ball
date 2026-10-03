"""Golf ball detector: camera -> YOLO -> nearest ball position.

Publishes:
  /golf_ball/target  geometry_msgs/PointStamped  nearest ball (only when one is seen)
                       x = horizontal offset, -1 (left edge) .. +1 (right edge)
                       y = bottom edge of the box, 0 (top of image) .. 1 (bottom = close)
                       z = detection confidence
  /golf_ball/image   sensor_msgs/Image (bgr8)    annotated image for debugging (rqt_image_view)

"Nearest" = the box whose bottom edge is lowest in the image (closest to the robot
for a forward-looking camera on flat ground).

Model (parameter `model`):
  "auto"  (default) use models/golf_ball_yolo11n_ncnn_model if it exists (export it on the
          Raspberry Pi, ~4x faster on ARM), otherwise models/golf_ball_yolo11n.pt
  a path  to a .pt file or an NCNN model folder
`imgsz` 0 = auto: 320 for NCNN (the size it was exported with), 640 for .pt
"""

import glob
import os
import re
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class GolfBallDetector(Node):

    def __init__(self):
        super().__init__('golf_ball_detector')

        self.declare_parameters('', [
            ('model', 'auto'),
            ('camera', 'auto'),         # "auto", an index ("0") or a path ("/dev/video8")
            ('width', 640),
            ('height', 480),
            ('flip', False),            # rotate 180 deg if the camera is mounted upside down
            ('imgsz', 0),               # YOLO input size, 0 = auto (320 NCNN / 640 .pt)
            ('confidence', 0.4),
            ('max_rate', 15.0),         # Hz, upper limit of the detection loop
            ('publish_image', True),
            ('image_every_n', 3),       # publish every Nth annotated frame
            ('show_window', False),     # cv2 window (desktop only)
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.imgsz = p('imgsz')
        self.conf = p('confidence')
        self.flip = p('flip')
        self.publish_image = p('publish_image')
        self.image_every_n = max(1, p('image_every_n'))
        self.show_window = p('show_window')

        # Import here so the node starts with a clear error if ultralytics is missing
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().fatal('ultralytics not installed: pip install ultralytics')
            raise
        model_path = self.resolve_model(p('model'))
        if self.imgsz <= 0:
            self.imgsz = 320 if model_path.endswith('_ncnn_model') else 640
        self.get_logger().info(f'Loading model {model_path} (imgsz {self.imgsz})')
        self.model = YOLO(model_path, task='detect')

        cam, self.cap = self.open_camera(str(p('camera')), p('width'), p('height'))

        self.target_pub = self.create_publisher(PointStamped, 'golf_ball/target', 10)
        self.image_pub = self.create_publisher(Image, 'golf_ball/image', 2)
        self.create_timer(1.0 / p('max_rate'), self.on_timer)

        self.frame_count = 0
        self.fps = 0.0
        self.last_time = time.monotonic()
        self.get_logger().info(f'Detecting golf balls from camera {cam}')

    def open_camera(self, camera, width, height):
        """Open the camera; "auto" tries every /dev/video* until one returns a frame.

        Raspberry Pi 5 exposes many /dev/video* nodes that are not cameras (decoder, ISP),
        so a USB webcam is usually not /dev/video0 there.
        """
        if camera == 'auto':
            candidates = sorted(glob.glob('/dev/video*'), key=lambda d: int(d[10:] or 0))
        else:
            candidates = [int(camera) if camera.isdigit() else camera]
        for dev in candidates:
            # Open by index: OpenCV 5's V4L2 backend can no longer open by path
            index = dev
            if isinstance(dev, str) and re.fullmatch(r'/dev/video\d+', dev):
                index = int(dev[len('/dev/video'):])
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always process the newest frame
                ok, _ = cap.read()
                if ok:
                    return dev, cap
            cap.release()
        raise RuntimeError(
            f'No working camera found (tried {", ".join(map(str, candidates)) or "none"}). '
            'Check the USB webcam with: v4l2-ctl --list-devices')

    @staticmethod
    def resolve_model(model):
        if model != 'auto':
            return model
        pt = os.path.join(
            get_package_share_directory('golf_bot_vision'), 'models', 'golf_ball_yolo11n.pt')
        # With --symlink-install the installed .pt links back to src/.../models, where the
        # NCNN folder is exported, so look next to the real file.
        ncnn = os.path.join(os.path.dirname(os.path.realpath(pt)), 'golf_ball_yolo11n_ncnn_model')
        return ncnn if os.path.isdir(ncnn) else pt

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Camera read failed', throttle_duration_sec=2.0)
            return
        if self.flip:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        h, w = frame.shape[:2]

        result = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]
        boxes = result.boxes.xyxy.cpu().numpy() if result.boxes is not None else []
        confs = result.boxes.conf.cpu().numpy() if result.boxes is not None else []

        # Nearest ball = lowest bottom edge
        target = None
        if len(boxes):
            i = int(boxes[:, 3].argmax())
            target = (boxes[i], float(confs[i]))
            x1, y1, x2, y2 = target[0]
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.point.x = float(((x1 + x2) / 2.0 - w / 2.0) / (w / 2.0))
            msg.point.y = float(y2 / h)
            msg.point.z = target[1]
            self.target_pub.publish(msg)

        # FPS (smoothed)
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

        self.frame_count += 1
        want_image = self.publish_image and self.frame_count % self.image_every_n == 0
        if want_image or self.show_window:
            img = self.annotate(frame, boxes, confs, target)
            if want_image:
                self.image_pub.publish(self.to_image_msg(img))
            if self.show_window:
                cv2.imshow('golf_ball_detector', img)
                cv2.waitKey(1)

    def to_image_msg(self, img):
        # Built by hand instead of cv_bridge: ROS Jazzy's cv_bridge is compiled against
        # NumPy 1.x and breaks next to the NumPy 2.x that ultralytics installs.
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = img.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg

    def annotate(self, frame, boxes, confs, target):
        img = frame.copy()
        h, w = img.shape[:2]
        cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        for (x1, y1, x2, y2), c in zip(boxes, confs):
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 255), 2)
            cv2.putText(img, f'{c:.2f}', (int(x1), int(y1) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        if target is not None:
            x1, y1, x2, y2 = target[0]
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
            cv2.line(img, (w // 2, h), (int((x1 + x2) / 2), int(y2)), (0, 255, 0), 2)
        cv2.putText(img, f'{len(boxes)} ball(s)  {self.fps:.1f} FPS', (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return img

    def destroy_node(self):
        self.cap.release()
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GolfBallDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
