"""Xbox controller teleop for the golf ball collector robot (drive only).

Subscribes to /joy and publishes:
  /cmd_vel  geometry_msgs/Twist  drive command (only while the deadman button is held)

Default mapping (joy_node, Xbox controller over USB / xpad driver):
  LB (hold)        deadman: the robot only moves while this is held
  RB (hold)        turbo speed
  Left stick Y     forward / backward
  Right stick X    turn left / right
  Y                toggle AUTO mode: drive with /cmd_vel_auto (ball_chaser)
                   holding LB in AUTO takes over manually; joystick lost -> stop
  B                emergency stop (latched, stops immediately, also leaves AUTO)
  Start            release emergency stop

Speed changes are ramped (accel_* / decel_* parameters), so the robot speeds up
and slows down smoothly. Releasing LB or losing the joystick ramps down to zero.

Button/axis indices are parameters, so a different mapping only needs a YAML change.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class XboxTeleop(Node):

    def __init__(self):
        super().__init__('xbox_teleop')

        self.declare_parameters('', [
            ('axis_linear', 1),
            ('axis_angular', 3),
            ('scale_linear', 0.4),
            ('scale_linear_turbo', 0.8),
            ('scale_angular', 1.2),
            ('scale_angular_turbo', 2.0),
            ('button_deadman', 4),
            ('button_turbo', 5),
            ('button_estop', 1),
            ('button_estop_release', 7),
            ('button_auto', 3),
            ('auto_timeout', 0.5),
            ('accel_linear', 0.5),
            ('decel_linear', 1.0),
            ('accel_angular', 2.0),
            ('decel_angular', 4.0),
            ('joy_timeout', 0.5),
            ('publish_rate', 20.0),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.axis_linear = p('axis_linear')
        self.axis_angular = p('axis_angular')
        self.scale_linear = p('scale_linear')
        self.scale_linear_turbo = p('scale_linear_turbo')
        self.scale_angular = p('scale_angular')
        self.scale_angular_turbo = p('scale_angular_turbo')
        self.btn_deadman = p('button_deadman')
        self.btn_turbo = p('button_turbo')
        self.btn_estop = p('button_estop')
        self.btn_estop_release = p('button_estop_release')
        self.btn_auto = p('button_auto')
        self.auto_timeout = p('auto_timeout')
        self.joy_timeout = p('joy_timeout')
        self.accel_linear = p('accel_linear')
        self.decel_linear = p('decel_linear')
        self.accel_angular = p('accel_angular')
        self.decel_angular = p('decel_angular')
        self.dt = 1.0 / p('publish_rate')
        self.cur_linear = 0.0
        self.cur_angular = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.auto_pub = self.create_publisher(Bool, 'auto_mode', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_subscription(Twist, 'cmd_vel_auto', self.on_cmd_auto, 10)
        self.create_timer(1.0 / p('publish_rate'), self.on_timer)

        self.last_joy = None
        self.last_joy_time = None
        self.prev_buttons = []
        self.estop = False
        self.auto = False
        self.auto_cmd = Twist()
        self.auto_cmd_time = None
        self.was_driving = False
        self.joy_lost_reported = False

        self.get_logger().info(
            'Xbox teleop ready: hold LB to drive, RB turbo, Y auto, B e-stop, Start release')

    # --- helpers -----------------------------------------------------------
    @staticmethod
    def _get(values, index, default=0):
        return values[index] if 0 <= index < len(values) else default

    def _pressed(self, joy, index):
        """Rising edge: button went from released to pressed."""
        return self._get(joy.buttons, index) == 1 and self._get(self.prev_buttons, index) == 0

    def _joy_alive(self):
        if self.last_joy_time is None:
            return False
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        return age < self.joy_timeout

    # --- callbacks ---------------------------------------------------------
    def on_joy(self, joy):
        if self._pressed(joy, self.btn_estop):
            self.estop = True
            self.auto = False
            self.get_logger().warn('EMERGENCY STOP (press Start to release)')
        if self._pressed(joy, self.btn_estop_release) and self.estop:
            self.estop = False
            self.get_logger().info('Emergency stop released')
        if self._pressed(joy, self.btn_auto) and not self.estop:
            self.auto = not self.auto
            self.get_logger().info(f'AUTO mode {"ON" if self.auto else "OFF"}')

        self.prev_buttons = list(joy.buttons)
        self.last_joy = joy
        self.last_joy_time = self.get_clock().now()
        self.joy_lost_reported = False

    def on_cmd_auto(self, msg):
        self.auto_cmd = msg
        self.auto_cmd_time = self.get_clock().now()

    def _auto_cmd_fresh(self):
        if self.auto_cmd_time is None:
            return False
        age = (self.get_clock().now() - self.auto_cmd_time).nanoseconds * 1e-9
        return age < self.auto_timeout

    def on_timer(self):
        alive = self._joy_alive()
        if not alive and self.last_joy is not None and not self.joy_lost_reported:
            self.get_logger().warn('Joystick lost: stopping robot (AUTO off)')
            self.joy_lost_reported = True
            self.auto = False
        self.auto_pub.publish(Bool(data=self.auto))

        joy = self.last_joy
        deadman = alive and joy is not None and self._get(joy.buttons, self.btn_deadman) == 1

        if self.estop:
            # Emergency stop: no ramp, stop immediately
            self.cur_linear = 0.0
            self.cur_angular = 0.0
            if self.was_driving:
                self.cmd_pub.publish(Twist())
                self.was_driving = False
            return

        # Target from the sticks (zero when LB released or joystick lost -> smooth stop)
        target_linear = 0.0
        target_angular = 0.0
        if deadman:
            turbo = self._get(joy.buttons, self.btn_turbo) == 1
            lin = self.scale_linear_turbo if turbo else self.scale_linear
            ang = self.scale_angular_turbo if turbo else self.scale_angular
            target_linear = lin * float(self._get(joy.axes, self.axis_linear, 0.0))
            target_angular = ang * float(self._get(joy.axes, self.axis_angular, 0.0))
        elif self.auto and alive and self._auto_cmd_fresh():
            # AUTO: follow ball_chaser (still ramped). The joystick must stay connected.
            target_linear = self.auto_cmd.linear.x
            target_angular = self.auto_cmd.angular.z

        self.cur_linear = self._ramp(self.cur_linear, target_linear,
                                     self.accel_linear, self.decel_linear)
        self.cur_angular = self._ramp(self.cur_angular, target_angular,
                                      self.accel_angular, self.decel_angular)

        if self.cur_linear != 0.0 or self.cur_angular != 0.0 or deadman or self.auto:
            twist = Twist()
            twist.linear.x = self.cur_linear
            twist.angular.z = self.cur_angular
            self.cmd_pub.publish(twist)
            self.was_driving = True
        elif self.was_driving:
            # Ramp finished at zero: send one last zero, then go quiet so other
            # cmd_vel sources (e.g. Nav2) are not blocked
            self.cmd_pub.publish(Twist())
            self.was_driving = False

    def _ramp(self, current, target, accel, decel):
        """Move current toward target, limited by accel (speeding up) or decel (slowing down)."""
        # Slowing down = moving toward zero, or crossing zero (reversing direction)
        slowing = abs(target) < abs(current) or current * target < 0.0
        step = (decel if slowing else accel) * self.dt
        if target > current:
            current = min(current + step, target)
        else:
            current = max(current - step, target)
        return current


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Best effort stop on exit
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
