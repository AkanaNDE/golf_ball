"""Measure the camera focal length in pixels (needed for distance-from-size).

Put one tag straight in front of the camera at a measured distance (camera lens to tag),
then run on the robot:

  ros2 run golf_bot_tags tag_focal_calibration --distance 1.00 --size 0.15

It measures continuously:  focal_px = distance * tag_height_px / tag_size
prints the running median every second, and publishes the annotated camera image on
/tag_calibration/image/compressed (view it on the laptop: ros2 run golf_bot_vision image_viewer).
Press Ctrl+C when the value is stable; put focal_px and focal_width in tag_mission.yaml.
Repeat at 2-3 distances to check it.
"""

import argparse
import math
import sys
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from golf_bot_tags.tag_vision import (TagDetector, encode_jpeg, open_camera, preview_frame,
                                      set_exposure)


class FocalCalibration(Node):

    def __init__(self, a):
        super().__init__('tag_focal_calibration')
        self.a = a
        dev, self.cap = open_camera(a.camera, a.width, a.height)
        set_exposure(self.cap, a.exposure)
        self.det = TagDetector(a.family, a.size, focal_px=1.0)  # focal unused: only h_px
        self.pub = self.create_publisher(CompressedImage, 'tag_calibration/image/compressed', 2)
        self.values = []
        self.last_print = time.monotonic()
        self.last_image = 0.0
        self.width = a.width
        self.get_logger().info(
            f'Camera {dev}: tag {a.family} {a.size * 100:.1f} cm at {a.distance:.2f} m. '
            'Image: /tag_calibration/image/compressed. Ctrl+C when stable.')
        self.create_timer(0.03, self.on_timer)

    def median(self):
        if not self.values:
            return None
        v = sorted(self.values[-300:])  # last ~10 s
        return v[len(v) // 2]

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        h, w = frame.shape[:2]
        self.width = w
        obs = self.det.detect(frame, time.monotonic())
        for o in obs:
            self.values.append(self.a.distance * o.h_px / self.a.size)
        f = self.median()

        now = time.monotonic()
        if now - self.last_print >= 1.0:
            self.last_print = now
            if f is None:
                self.get_logger().warn('No tag detected (check family, light, distance)')
            else:
                hfov = math.degrees(2 * math.atan(w / 2 / f))
                self.get_logger().info(f'focal_px: {f:.1f}  (HFOV {hfov:.1f} deg, '
                                       f'{len(self.values)} samples)')

        if self.pub.get_subscription_count() == 0 or now - self.last_image < 0.12:
            return  # nobody watching, or keep it ~8 FPS over WiFi
        self.last_image = now
        img, scale = preview_frame(frame)
        w, h = img.shape[1], img.shape[0]
        cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        for o in obs:
            pts = (o.corners * scale).astype(int).reshape(-1, 1, 2)
            centered = abs(o.u) < 0.15
            color = (0, 255, 0) if centered else (0, 165, 255)
            cv2.polylines(img, [pts], True, color, 2)
            x, y = pts[0][0]
            cv2.putText(img, f'#{o.tag_id}  h={o.h_px:.1f}px', (int(x), int(y) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            if not centered:
                cv2.putText(img, 'move the tag to the centre line', (8, h - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        text = (f'focal_px = {f:.1f}  ({len(self.values)} samples)' if f
                else 'no tag detected')
        cv2.putText(img, text, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(img, f'set distance {self.a.distance:.2f} m, tag {self.a.size * 100:.1f} cm',
                    (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        data = encode_jpeg(img)
        if data:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format, msg.data = 'jpeg', data
            self.pub.publish(msg)

    def report(self):
        f = self.median()
        if f is None:
            print('No tag was detected - no focal length measured.')
            return
        hfov = math.degrees(2 * math.atan(self.width / 2 / f))
        print(f'\nfocal_px: {f:.1f}    (horizontal FOV {hfov:.1f} deg, {len(self.values)} samples)')
        print('Put these in src/golf_bot_bringup/config/tag_mission.yaml:')
        print(f'    focal_px: {f:.1f}')
        print(f'    focal_width: {self.width}')


def main(args=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--distance', type=float, required=True, help='camera-to-tag distance (m)')
    ap.add_argument('--size', type=float, default=0.15, help='black square side length (m)')
    ap.add_argument('--family', default='36h11')
    ap.add_argument('--camera', default='UGREEN',
                    help='device name (part of it), auto, index or /dev/videoN')
    ap.add_argument('--width', type=int, default=2560, help='use the same size as tag_mission')
    ap.add_argument('--height', type=int, default=1440)
    ap.add_argument('--exposure', type=float, default=0.0, help='ms, 0 = auto exposure')
    a, _ = ap.parse_known_args(sys.argv[1:] if args is None else args)

    rclpy.init(args=args)
    node = FocalCalibration(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.report()
        node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
