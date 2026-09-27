"""Wheel odometry for the 4-wheel skid-steer (diff-drive) golf ball collector.

Subscribes:
  /wheel_vel  geometry_msgs/Vector3  measured wheel speed from the ESP32 (rad/s)
                                     x = left side, y = right side
Publishes:
  /odom       nav_msgs/Odometry
  TF          odom -> base_link (if publish_tf is true)

wheel_radius and track_width must match the values in the firmware config.h.
For skid steer, track_width is the *effective* track: calibrate it by commanding
a 360 degree turn in place and scaling until odometry reports exactly one turn.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameters('', [
            ('wheel_radius', 0.100),
            ('track_width', 0.510),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_link'),
            ('publish_tf', True),
            ('max_dt', 0.5),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.wheel_radius = p('wheel_radius')
        self.track_width = p('track_width')
        self.odom_frame = p('odom_frame')
        self.base_frame = p('base_frame')
        self.publish_tf = p('publish_tf')
        self.max_dt = p('max_dt')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Vector3, 'wheel_vel', self.on_wheel_vel, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None

    def on_wheel_vel(self, msg):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > self.max_dt:
            # Skip gaps (e.g. ESP32 reconnect) instead of integrating a huge step
            return

        # 4-wheel diff drive forward kinematics (both wheels of a side share one speed):
        #   v = r * (w_R + w_L) / 2,   w = r * (w_R - w_L) / W
        v_left = msg.x * self.wheel_radius
        v_right = msg.y * self.wheel_radius
        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.track_width

        # Midpoint integration
        mid_yaw = self.yaw + w * dt / 2.0
        self.x += v * math.cos(mid_yaw) * dt
        self.y += v * math.sin(mid_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Skid steer slips a lot when turning: larger yaw uncertainty
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.1
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
