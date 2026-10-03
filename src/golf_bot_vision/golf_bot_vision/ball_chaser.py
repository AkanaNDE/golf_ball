"""Ball chaser: drive the robot to the nearest golf ball (visual servoing).

Subscribes:
  /golf_ball/target   geometry_msgs/PointStamped  from golf_ball_detector
Publishes:
  /cmd_vel_auto       geometry_msgs/Twist         used by xbox_teleop when AUTO mode is on (Y)
  /ball_chaser/state  std_msgs/String             SEARCH / TRACK / COLLECT / WAIT

States:
  WAIT     ball just lost: stop for search_delay seconds (it may come back)
  SEARCH   no ball: turn in place toward the side the ball was last seen
  TRACK    ball seen: turn to center it (P control), drive forward when roughly centered,
           slow down as it gets close (its bottom edge moves down the image)
  COLLECT  ball reached the bottom of the image (or vanished there): drive straight
           for collect_time seconds to run over / pick it up, then SEARCH again
"""

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def clamp(value, low, high):
    return max(low, min(high, value))


class BallChaser(Node):

    def __init__(self):
        super().__init__('ball_chaser')

        self.declare_parameters('', [
            ('rate', 20.0),
            ('target_timeout', 0.5),    # s without detection = ball lost
            ('kp_angular', 1.5),        # rad/s per unit of horizontal offset
            ('max_angular', 1.0),       # rad/s
            ('max_linear', 0.3),        # m/s when the ball is centered and far
            ('min_linear', 0.08),       # m/s when the ball is about to be reached
            ('align_threshold', 0.3),   # |offset| above this: turn in place only
            ('slow_down_y', 0.6),       # start slowing when the ball's bottom passes this
            ('arrive_y', 0.92),         # ball bottom this low = reached
            ('collect_speed', 0.15),    # m/s straight drive to pick the ball up
            ('collect_time', 1.5),      # s
            ('search_delay', 1.0),      # s to wait after losing the ball before searching
            ('search_angular', 0.5),    # rad/s turning in place while searching
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.target_timeout = p('target_timeout')
        self.kp_angular = p('kp_angular')
        self.max_angular = p('max_angular')
        self.max_linear = p('max_linear')
        self.min_linear = p('min_linear')
        self.align_threshold = p('align_threshold')
        self.slow_down_y = p('slow_down_y')
        self.arrive_y = p('arrive_y')
        self.collect_speed = p('collect_speed')
        self.collect_time = p('collect_time')
        self.search_delay = p('search_delay')
        self.search_angular = p('search_angular')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.state_pub = self.create_publisher(String, 'ball_chaser/state', 10)
        self.create_subscription(PointStamped, 'golf_ball/target', self.on_target, 10)
        self.create_timer(1.0 / p('rate'), self.on_timer)

        self.target = None            # (x_offset, y_bottom)
        self.target_time = None
        self.last_side = 1.0          # +1 = last seen on the left (turn left), -1 = right
        self.last_y = 0.0
        self.state = 'SEARCH'
        self.state_since = self.now()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
            self.state = state
            self.state_since = self.now()

    def on_target(self, msg):
        self.target = (msg.point.x, msg.point.y)
        self.target_time = self.now()
        # Image x to the right = robot must turn right (negative yaw)
        self.last_side = -1.0 if msg.point.x > 0 else 1.0
        self.last_y = msg.point.y

    def on_timer(self):
        now = self.now()
        seen = self.target_time is not None and now - self.target_time < self.target_timeout
        in_state = now - self.state_since
        twist = Twist()

        if self.state == 'COLLECT':
            if in_state < self.collect_time:
                twist.linear.x = self.collect_speed
            else:
                self.target_time = None  # forget the collected ball
                self.set_state('SEARCH')
        elif seen:
            x, y = self.target
            if y >= self.arrive_y:
                self.set_state('COLLECT')
                twist.linear.x = self.collect_speed
            else:
                self.set_state('TRACK')
                twist.angular.z = clamp(-self.kp_angular * x, -self.max_angular, self.max_angular)
                if abs(x) < self.align_threshold:
                    # Faster when centered, slower when close
                    align = 1.0 - abs(x) / self.align_threshold
                    if y > self.slow_down_y:
                        near = (y - self.slow_down_y) / (self.arrive_y - self.slow_down_y)
                        speed = self.max_linear + (self.min_linear - self.max_linear) * near
                    else:
                        speed = self.max_linear
                    twist.linear.x = max(self.min_linear, speed * (0.5 + 0.5 * align))
        else:
            if self.state == 'TRACK':
                # Lost while very close: it went under the camera -> collect it
                if self.last_y > self.arrive_y - 0.1:
                    self.set_state('COLLECT')
                    twist.linear.x = self.collect_speed
                else:
                    self.set_state('WAIT')
            elif self.state == 'WAIT' and in_state >= self.search_delay:
                self.set_state('SEARCH')
            if self.state == 'SEARCH':
                twist.angular.z = self.search_angular * self.last_side

        self.cmd_pub.publish(twist)
        self.state_pub.publish(String(data=self.state))


def main(args=None):
    rclpy.init(args=args)
    node = BallChaser()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
