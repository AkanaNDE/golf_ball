"""Show the robot camera (with golf ball detections) on another computer.

  ros2 run golf_bot_vision image_viewer

Subscribes:
  /golf_ball/image/compressed  sensor_msgs/CompressedImage  JPEG from golf_ball_detector
  /ball_chaser/state           std_msgs/String              shown as an overlay
  /auto_mode                   std_msgs/Bool                shown as an overlay
Press q or Esc in the window to quit.
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

WINDOW = 'golf ball camera'


class ImageViewer(Node):

    def __init__(self):
        super().__init__('golf_ball_image_viewer')
        self.declare_parameter('topic', 'golf_ball/image/compressed')
        self.declare_parameter('scale', 1.0)
        self.scale = float(self.get_parameter('scale').value)

        topic = self.get_parameter('topic').value
        # Best effort: drop late frames instead of queueing them over slow WiFi
        self.create_subscription(CompressedImage, topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, 'ball_chaser/state', self.on_state, 10)
        self.create_subscription(Bool, 'auto_mode', self.on_auto, 10)
        self.create_timer(0.03, self.on_timer)

        self.frame = None
        self.state = '-'
        self.auto = None
        self.fps = 0.0
        self.last_rx = None
        self.kbytes = 0.0
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        self.get_logger().info(f'Waiting for images on /{topic} (q / Esc to quit)')

    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        now = time.monotonic()
        if self.last_rx is not None and now > self.last_rx:
            self.fps = 0.8 * self.fps + 0.2 / (now - self.last_rx)
        self.last_rx = now
        self.kbytes = len(msg.data) / 1024.0
        self.frame = img

    def on_state(self, msg):
        self.state = msg.data

    def on_auto(self, msg):
        self.auto = msg.data

    def on_timer(self):
        if self.frame is None:
            img = np.zeros((240, 480, 3), np.uint8)
            cv2.putText(img, 'Waiting for robot camera...', (20, 125),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            img = self.frame.copy()
            if self.scale != 1.0:
                img = cv2.resize(img, None, fx=self.scale, fy=self.scale)
            h = img.shape[0]
            stale = self.last_rx is not None and time.monotonic() - self.last_rx > 2.0
            auto = 'AUTO' if self.auto else ('MANUAL' if self.auto is not None else '?')
            text = f'{auto}  {self.state}  {self.fps:.1f} FPS  {self.kbytes:.0f} KB'
            color = (0, 0, 255) if stale else (0, 255, 255)
            if stale:
                text += '  (no new frames)'
            cv2.putText(img, text, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        cv2.imshow(WINDOW, img)
        key = cv2.waitKey(1) & 0xFF
        closed = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
        if key in (ord('q'), 27) or closed:
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
