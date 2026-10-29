"""AprilTag tour mission: scan 360 deg, map every tag by its apparent size, then visit all of
them along the fastest route.

Algorithm "Scan - Calibrate - Plan - Visit - Relocalize - Replan":

  SCAN      turn in place a bit more than one revolution, recording every tag observation
            with the robot pose at frame time.
  CALIBRATE visual loop closure: a tag crosses the image centre exactly once per real
            revolution, which gives the true/odometry yaw ratio. Wheels of a skid-steer
            robot slip when turning, so this corrects the map and all later dead reckoning.
  MAP       fuse all observations per tag (inverse-variance weights: far / image-edge
            observations count less) -> one (x, y) per tag in the start frame.
  PLAN      exact time-optimal open tour (Held-Karp over (visited, previous, last) because
            turning time depends on the arrival heading); heuristic above ~10 tags.
  VISIT     turn toward the next tag and drive: dead reckoning while the tag is not visible,
            visual servoing on the tag's size/position as soon as it is. If the tag is not
            seen where it should be: local search sweep, then skip it.
  RELOCALIZE on arrival the measured offset to a tag with a known map position corrects the
            robot's (x, y) - drift does not accumulate over the tour.
  REPLAN    plan the remaining tags again from the corrected pose (milliseconds).

Safety, with joystick (tag_mission.launch.py): the node publishes /cmd_vel_auto and
xbox_teleop forwards it to /cmd_vel while AUTO (Y) is on; LB overrides, B stops. AUTO off
pauses the mission, AUTO on resumes it (or starts a new one after DONE).
Without joystick (tag_mission_auto.launch.py, require_auto: false): /cmd_vel_auto is remapped
to /cmd_vel, the mission starts after `start_delay` seconds, and
  ros2 topic pub --once /tag_mission/stop std_msgs/msg/Bool "{data: true}"   pauses (false resumes).
Commands are ramped (accel_linear / accel_angular) because no teleop smooths them, and the
ESP32 stops the wheels by itself if this node dies (no /cmd_vel for 0.5 s).
"""

import math
import time

import cv2
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from golf_bot_tags.tag_map import PoseHistory, ScanCalibrator, TagMap
from golf_bot_tags.tag_vision import (CameraThread, RotationEstimator, SimulatedCamera,
                                      TagDetector, encode_jpeg, focal_from_hfov, preview_frame,
                                      open_camera, set_exposure)
from golf_bot_tags.tour_planner import MotionModel, plan_tour, wrap


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TagMission(Node):

    def __init__(self):
        super().__init__('tag_mission')
        self.declare_parameters('', [
            # camera / tags
            ('camera', 'auto'), ('width', 640), ('height', 480), ('flip', False),
            ('tag_family', '36h11'), ('tag_size', 0.15),
            ('focal_px', 0.0), ('hfov_deg', 70.0),
            ('focal_width', 0),              # frame width focal_px was measured at (0 = actual width)
            ('camera_offset', 0.15), ('camera_latency', 0.08),
            ('exposure_ms', 0.0),            # manual exposure: short = sharp while turning (0 = auto)
            ('gain', -1.0),                  # camera gain to brighten short exposures (-1 = keep)
            ('brightness', 0.0),             # digital gain lifts dark frames to this mean (0 = off)
            ('max_blur_px', 0.0),            # turn no faster than this much motion blur (0 = off)
            ('auto_exposure_guess_ms', 33.0),  # exposure assumed for max_blur_px in auto exposure
            ('camera_controls', ''),         # extra v4l2-ctl -c controls, if OpenCV cannot set them
            # pose estimate
            ('pose_source', 'cmd'),          # forward speed from 'cmd' (/cmd_vel) or 'odom' (encoders)
            ('heading_source', 'auto'),      # 'auto': IMU (/imu) when it publishes, else command
            ('imu_timeout', 0.5),
            ('yaw_scale', 1.0), ('linear_scale', 1.0),
            # scan
            ('scan_mode', 'step'),           # step: turn-stop-look (sharp images) | continuous
            ('scan_speed', 0.4), ('scan_extra_deg', 45.0), ('scan_max_deg', 540.0),
            ('scan_step_deg', 30.0),         # turn this much between stops (< camera FOV)
            ('scan_step_time', 2.5),         # s: turn for a fixed time per step (0 = by angle)
            ('scan_settle_time', 0.5),       # s to wait after stopping (robot stops shaking)
            ('scan_look_time', 0.7),         # s of sharp frames collected at each stop
            ('scan_visual_yaw', True),       # measure the real turn between stops from the image
            ('scan_speed_max', 1.2),         # rad/s, limit when boosting a stalled turn
            ('min_observations', 3),
            # planner
            ('v_max', 0.25), ('w_max', 0.8), ('standoff', 0.5), ('leg_overhead', 0.5),
            ('exact_limit', 10),
            # control
            ('rate', 20.0), ('k_heading', 1.5), ('k_distance', 0.8), ('v_min', 0.06),
            ('w_min', 0.25), ('turn_in_place_deg', 25.0), ('align_tolerance_deg', 10.0),
            ('align_step_deg', 45.0), ('approach_step', 0.4), ('approach_look_time', 1.0),
            ('approach_turn_deg', 20.0), ('approach_blind_steps', 6),
            ('approach_max_corrections', 1),  # in-place turns toward a seen tag before driving anyway
            ('drive_visual_steer', True),    # steer toward the tag seen in the image while driving
            ('visual_steer_gain', 0.5),      # 0..1: how much each sighting moves the heading goal
            ('confirm_distance', 0.5),       # m before the standoff point: stop once to look
            ('turn_track', False),           # stop turns on the angle measured from the images
            ('turn_track_period', 0.15),     # s between image measurements while turning
            ('turn_stop_lag', 0.15),         # s: stop this early (robot keeps turning a bit)
            ('arrive_tolerance', 0.08),
            ('lost_timeout', 0.6), ('local_search_deg', 70.0), ('dwell_time', 1.0),
            ('leg_timeout_factor', 3.0), ('relocalize_gain', 0.7),
            ('require_auto', True),
            ('start_delay', 5.0),            # s countdown before moving (require_auto: false)
            ('accel_linear', 0.5),           # m/s^2 output ramp
            ('accel_angular', 2.0),          # rad/s^2 output ramp
            # simulation (dry run without camera/robot)
            ('simulate', False), ('sim_tags', [0.0]), ('sim_yaw_scale', 0.85),
            ('sim_noise', 0.03),
            ('sim_blur_speed', -1.0),        # rad/s: blurred above this turn rate (-1: from exposure)
            ('sim_auto_exposure_ms', 33.0),  # simulated exposure when exposure_ms is 0 (auto)
            ('sim_min_turn_rate', 0.0),      # rad/s: commanded turns below this do not move
            ('sim_yaw_scale_fast', -1.0),    # true/commanded yaw for |w| > 0.5 (-1 = same)
            ('sim_visual_noise_deg', 0.7),   # noise of the simulated visual yaw per stop
            ('sim_render', False),           # render tag images + run the real detector
            ('sim_imu', False),              # simulated IMU (true yaw + slow drift) on /imu path
            ('sim_blur_linear', 0.0),        # m/s: frames blurred when driving faster than this
            ('sim_turn_lag', 0.0),           # s: simulated turn rate follows the command this slowly
                                             # (robot keeps turning after a stop -> overshoot)
        ])
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p = {n: g(n) for n in [
            'camera_offset', 'camera_latency', 'pose_source', 'yaw_scale', 'linear_scale',
            'exposure_ms', 'heading_source', 'imu_timeout',
            'scan_speed', 'scan_extra_deg', 'scan_max_deg', 'scan_mode', 'scan_step_deg',
            'scan_settle_time', 'scan_look_time', 'scan_visual_yaw', 'scan_speed_max',
            'scan_step_time',
            'sim_min_turn_rate', 'sim_visual_noise_deg', 'sim_yaw_scale_fast', 'k_heading', 'k_distance', 'v_min',
            'w_min', 'turn_in_place_deg', 'align_tolerance_deg', 'align_step_deg',
            'approach_step', 'approach_look_time', 'approach_turn_deg', 'approach_blind_steps',
            'approach_max_corrections', 'drive_visual_steer', 'visual_steer_gain',
            'confirm_distance', 'turn_track', 'turn_track_period', 'turn_stop_lag', 'sim_imu',
            'sim_blur_linear', 'sim_turn_lag',
            'arrive_tolerance', 'lost_timeout',
            'local_search_deg', 'dwell_time', 'leg_timeout_factor', 'relocalize_gain',
            'require_auto', 'exact_limit', 'simulate', 'sim_yaw_scale', 'start_delay',
            'accel_linear', 'accel_angular']}
        self.model = MotionModel(g('v_max'), g('w_max'), g('standoff'), g('leg_overhead'))
        self.dt = 1.0 / g('rate')
        self.yaw_scale = float(g('yaw_scale'))
        self.tag_map = TagMap(self.p['camera_offset'], min_observations=g('min_observations'))
        self.history = PoseHistory()

        # pose estimate (map frame = robot start pose) and simulated truth
        self.pose = [0.0, 0.0, 0.0]
        self.truth = [0.0, 0.0, 0.0]
        self.truth_turned = 0.0        # simulated true yaw, unwrapped
        self.truth_speed = 0.0
        self.sim_t0 = time.monotonic()
        self.latest_frame = None       # newest camera frame (for visual yaw)
        self.rotation = RotationEstimator()
        self.vel = (0.0, 0.0)
        self.vel_time = 0.0
        self.travelled = 0.0
        self.yaw_odom_total = 0.0
        self.turn_deadband = 0.0       # rad/s: turning in place below this does not move (learned)
        self.w_turn_min = self.p['w_min']  # rad/s: smallest turn-in-place command that works

        # camera
        if self.p['simulate']:
            flat = list(g('sim_tags'))
            tags = {i: (flat[2 * i], flat[2 * i + 1]) for i in range(len(flat) // 2)}
            self.truth_rate = 0.0
            self.camera = SimulatedCamera(tags, lambda: tuple(self.truth), g('hfov_deg'),
                                          noise=g('sim_noise'), camera_offset=self.p['camera_offset'],
                                          truth_rate=lambda: self.truth_rate,
                                          blur_speed=g('sim_blur_speed') if g('sim_blur_speed') > 0
                                          else 2.0 / ((g('exposure_ms') if g('exposure_ms') > 0
                                                       else g('sim_auto_exposure_ms')) / 1000.0
                                                      * (g('focal_px') or 670.0)),
                                          detector=TagDetector(g('tag_family'), g('tag_size'),
                                                               g('focal_px'), g('hfov_deg'),
                                                               focal_width=g('focal_width'))
                                          if g('sim_render') else None,
                                          width=g('width'), height=g('height'))
            self.camera.truth_speed = lambda: self.truth_speed
            self.camera.blur_linear = g('sim_blur_linear')
            self.get_logger().info(f'SIMULATION: {len(tags)} tags {tags}, true yaw scale '
                                   f'{self.p["sim_yaw_scale"]}')
        else:
            detector = TagDetector(g('tag_family'), g('tag_size'), g('focal_px'), g('hfov_deg'),
                                   focal_width=g('focal_width'))
            self.detector = detector
            cam, cap = open_camera(g('camera'), g('width'), g('height'))
            real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if (real_w, real_h) != (g('width'), g('height')):
                self.get_logger().warn(f'Camera gives {real_w}x{real_h}, not {g("width")}x'
                                       f'{g("height")} (focal is computed for the real size)')
            gain_before = cap.get(cv2.CAP_PROP_GAIN)
            exposure = set_exposure(cap, g('exposure_ms'), g('gain'), cam, g('camera_controls'))
            self.camera = CameraThread(cap, detector, g('flip'), brightness=g('brightness'))
            self.camera.restore_auto_exposure = g('exposure_ms') > 0
            if g('gain') >= 0:
                self.camera.restore_gain = gain_before
            self.get_logger().info(
                f'Camera {cam}, tag {g("tag_family")} {g("tag_size") * 100:.1f} cm, '
                f'{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}, '
                f'focal {detector.focal(int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))):.0f} px, {exposure}'
                + (f', digital brightness {g("brightness"):.0f}' if g('brightness') > 0 else ''))
        self.last_frame = -1
        self.last_image_sent = 0.0     # time the last JPEG was published

        # Motion blur (px) = turn rate (rad/s) x exposure (s) x focal (px): limit the turn rate
        self.w_blur = float('inf')
        if g('max_blur_px') > 0:
            exposure_s = (g('exposure_ms') if g('exposure_ms') > 0 else g('auto_exposure_guess_ms')) / 1000.0
            focal = (g('focal_px') * g('width') / g('focal_width')
                     if g('focal_px') > 0 and g('focal_width') > 0 else g('focal_px')) \
                or focal_from_hfov(g('width'), g('hfov_deg'))
            self.w_blur = g('max_blur_px') / (exposure_s * focal)
            msg = (f'Turn rate limited to {self.w_blur:.2f} rad/s ({math.degrees(self.w_blur):.0f} deg/s) '
                   f'for <= {g("max_blur_px"):.1f} px blur at {exposure_s * 1000:.0f} ms exposure')
            if self.w_blur < self.p['w_min']:
                self.get_logger().warn(msg + f', below w_min {self.p["w_min"]:.2f}: the robot may not '
                                       'turn that slowly -> using w_min (shorter exposure_ms helps)')
            else:
                self.get_logger().info(msg)

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.status_pub = self.create_publisher(String, 'tag_mission/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'tag_mission/markers', 2)
        self.image_pub = self.create_publisher(CompressedImage, 'tag_mission/image/compressed', 2)
        self.create_subscription(Bool, 'auto_mode', self.on_auto, 10)
        self.create_subscription(Bool, 'tag_mission/stop', self.on_stop, 10)
        if self.p['pose_source'] == 'odom':
            self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.imu_total = None          # unwrapped IMU yaw
        self.imu_time = 0.0
        self.imu_prev = None
        self.imu_active = False
        self.imu_announced = False
        self.create_timer(self.dt, self.on_timer)
        self.create_timer(0.5, self.publish_markers)

        self.auto = not self.p['require_auto']
        self.stopped = False
        self.out = (0.0, 0.0)
        self.state = 'IDLE'
        self.state_time = time.monotonic()
        self.reset_mission()
        if self.p['require_auto']:
            self.get_logger().info('Tag mission ready: press Y (AUTO) to start')
        else:
            self.start_at = time.monotonic() + self.p['start_delay']
            self.get_logger().warn(
                f'NO JOYSTICK MODE: robot starts moving in {self.p["start_delay"]:.0f} s. '
                'Stop: ros2 topic pub --once /tag_mission/stop std_msgs/msg/Bool "{data: true}" '
                'or Ctrl+C / emergency switch')

    # ------------------------------------------------------------------ setup
    def reset_mission(self):
        self.pose = [0.0, 0.0, 0.0]
        self.tag_map.clear()
        self.calib = ScanCalibrator()
        self.calib.yaw0 = 0.0          # the scan starts at the map origin, facing +x
        self.scan_turned = 0.0         # unwrapped yaw turned during the scan
        self.scan_history = PoseHistory()
        self.scan_phase = 'settle'     # stop-and-go: (settle -> look -> turn) repeated
        self.scan_stop = 0             # index of the current stop
        self.scan_target = 0.0         # unwrapped yaw where the next stop is
        self.scan_phase_time = 0.0
        self.scan_look_frames = 0
        self.scan_true = 0.0           # real yaw turned (visual), unwrapped
        self.scan_stop_yaw = None      # real yaw of the current stop, once measured
        self.scan_kf = None            # previous stop's keyframe
        self.scan_odom_at_kf = 0.0
        self.scan_truth_at_kf = self.truth_turned if hasattr(self, 'truth_turned') else 0.0
        self.scan_vis_true = 0.0       # sums over visually measured steps
        self.scan_vis_odom = 0.0
        self.scan_ratio = 1.0          # real / odometry turn, learned during the scan
        self.scan_cmd_speed = None     # turn rate command (boosted if the robot stalls)
        self.scan_visual_ok = 0
        self.scan_pending = []         # detections while turning, waiting for the stop's yaw
        now = time.monotonic()         # turn tracker defaults (see start_turn_track)
        self.tt_goal, self.tt_dir, self.tt_odom0 = 0.0, 1.0, 0.0
        self.tt_t0 = self.tt_boost_time = now
        self.tt_value, self.tt_time, self.tt_last_try, self.tt_frame, self.tt_ref = None, 0.0, 0.0, -1, None
        self.tt_stop_meas, self.tt_stop_w = None, 0.0
        if not hasattr(self, 'turn_lag'):
            self.turn_lag = self.p['turn_stop_lag']   # learned: how long the robot keeps turning
        self.ids, self.positions = [], []
        self.tour = None
        self.visited, self.missed = [], []
        self.unconfirmed = []          # reached by the remembered map position, tag not seen
        self.target = None
        self.leg_start = 0.0
        self.leg_budget = 0.0
        self.last_seen = 0.0
        self.mission_start = None
        self.tour_start = None
        self.travelled = 0.0
        self.planned_time = 0.0

    def set_state(self, state, note=''):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state} {note}'.rstrip())
            self.state = state
            self.state_time = time.monotonic()
            self.publish_status(note)

    def publish_status(self, note=''):
        tgt = f' target={self.target}' if self.target is not None else ''
        text = (f'{self.state}{tgt} visited={self.visited} missed={self.missed} '
                f'map={len(self.tag_map.tags())} tags {note}').strip()
        self.status_pub.publish(String(data=text))

    # ------------------------------------------------------------------ inputs
    def on_auto(self, msg):
        if not self.p['require_auto']:
            return
        if msg.data and not self.auto and self.state in ('IDLE', 'DONE'):
            self.reset_mission()
            self.set_state('SCAN', '(AUTO on: new mission)')
        elif not msg.data and self.auto and self.state not in ('IDLE', 'DONE'):
            self.get_logger().info('AUTO off: mission paused')
        elif msg.data and not self.auto and self.state not in ('IDLE', 'DONE'):
            self.get_logger().info('AUTO on: mission resumed')
            self.leg_start = time.monotonic()  # do not count the pause against the leg
        self.auto = msg.data

    def on_stop(self, msg):
        if msg.data != self.stopped:
            self.get_logger().warn('STOP requested: mission paused' if msg.data
                                   else 'Stop released: mission resumed')
            if not msg.data:
                self.leg_start = time.monotonic()
        self.stopped = msg.data

    def on_imu(self, msg):
        q = msg.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        if self.imu_total is None:
            self.imu_total = yaw
        else:
            self.imu_total += wrap(yaw - self.imu_total)
        self.imu_time = time.monotonic()

    def on_cmd(self, msg):
        self.vel = (msg.linear.x, msg.angular.z)
        self.vel_time = time.monotonic()

    def on_odom(self, msg):
        self.vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.vel_time = time.monotonic()

    def integrate(self):
        now = time.monotonic()
        v, w = self.vel if now - self.vel_time < 0.3 else (0.0, 0.0)
        if abs(v) < 0.02 and abs(w) <= self.turn_deadband:
            w = 0.0  # learned: too weak to turn the robot in place
        if self.p['simulate']:  # the "real" robot under-rotates by sim_yaw_scale
            tw = w * self.p['sim_yaw_scale']
            if self.p['sim_yaw_scale_fast'] >= 0 and abs(w) > 0.5:  # non-linear motor response
                tw = w * self.p['sim_yaw_scale_fast']
            if abs(v) < 0.02 and abs(w) < self.p['sim_min_turn_rate']:
                tw = 0.0                   # too weak to overcome skid-steer friction
            if self.p['sim_turn_lag'] > 0:  # inertia: the real turn rate lags the command
                self.sim_rate = getattr(self, 'sim_rate', 0.0)
                self.sim_rate += (tw - self.sim_rate) * min(1.0, self.dt / self.p['sim_turn_lag'])
                tw = self.sim_rate
            self.truth_rate = tw
            self.truth_speed = v
            self.truth_turned += tw * self.dt
            self.truth[0] += v * math.cos(self.truth[2] + tw * self.dt / 2) * self.dt
            self.truth[1] += v * math.sin(self.truth[2] + tw * self.dt / 2) * self.dt
            self.truth[2] = wrap(self.truth[2] + tw * self.dt)
            if self.p['sim_imu']:   # like a BNO055: true yaw plus a slow drift (0.5 deg/min)
                self.imu_total = self.truth_turned + math.radians(0.5 / 60.0) * (now - self.sim_t0)
                self.imu_time = now
        v *= self.p['linear_scale']
        w *= self.yaw_scale
        # Heading from the IMU when available: measures the real turn (slip, stall, coasting)
        self.imu_active = (self.p['heading_source'] != 'cmd' and self.imu_total is not None
                           and now - self.imu_time < self.p['imu_timeout'])
        if self.imu_active:
            if not self.imu_announced:
                self.get_logger().info('Heading from the IMU (/imu)')
                self.imu_announced = True
            w = 0.0 if self.imu_prev is None else (self.imu_total - self.imu_prev) / self.dt
            self.imu_prev = self.imu_total
        else:
            if self.imu_announced:
                self.get_logger().warn('IMU silent: heading from the command again')
                self.imu_announced = False
            self.imu_prev = None
        x, y, yaw = self.pose
        self.pose = [x + v * math.cos(yaw + w * self.dt / 2) * self.dt,
                     y + v * math.sin(yaw + w * self.dt / 2) * self.dt,
                     wrap(yaw + w * self.dt)]
        if self.state == 'SCAN':
            self.scan_turned += w * self.dt
            self.scan_history.push(now, (self.pose[0], self.pose[1], self.scan_turned))
        self.travelled += abs(v) * self.dt
        self.yaw_odom_total += w * self.dt   # unwrapped dead-reckoned yaw
        self.history.push(now, tuple(self.pose))

    def new_observations(self):
        """Observations from a frame not processed yet, with the pose at frame time."""
        frame, obs, stamp, frames = self.camera.latest()
        if frames == self.last_frame:
            return [], None, None      # nothing new (the image is not re-sent either)
        self.last_frame = frames
        self.frame_stamp = stamp - self.p['camera_latency']
        pose = self.history.at(self.frame_stamp) or tuple(self.pose)
        return obs, pose, frame

    # ------------------------------------------------------------------ main loop
    def on_timer(self):
        self.integrate()
        obs, pose, frame = self.new_observations()
        if frame is not None:
            self.latest_frame = frame
        if obs:
            self.latest_obs = (time.monotonic(), {o.tag_id: o for o in obs})
        if self.state == 'IDLE' and not self.p['require_auto'] and not self.stopped:
            left = self.start_at - time.monotonic()
            if left <= 0:
                self.set_state('SCAN', '(no joystick: starting)')
            elif int(left + self.dt) != int(left + 2 * self.dt):
                self.get_logger().info(f'Starting in {int(left) + 1} s ...')
        cmd = (0.0, 0.0)
        active = self.auto and not self.stopped and self.state not in ('IDLE', 'DONE')
        if active:
            handler = getattr(self, 'step_' + self.state.lower())
            cmd = handler(obs, pose)
            # sharp images: no faster than the blur limit, but never below a turn rate that
            # was found necessary to move at all (w_turn_min grows when turns stall)
            # (a scan step boosted after a stall must get through too)
            w_lim = max(self.w_blur, self.w_turn_min, self.p['w_min'],
                        (self.scan_cmd_speed or 0.0) if self.state == 'SCAN' else 0.0)
            cmd = (cmd[0], max(-w_lim, min(w_lim, cmd[1])))
        if active:
            self.out = (self.ramp(self.out[0], cmd[0], self.p['accel_linear']),
                        self.ramp(self.out[1], cmd[1], self.p['accel_angular']))
        else:
            self.out = (0.0, 0.0)  # paused / stopped / done: stop at once
        twist = Twist()
        twist.linear.x, twist.angular.z = float(self.out[0]), float(self.out[1])
        # standing still -> sharp frames: slow, thorough detection (finds small / far tags);
        # moving -> blurred frames: fast detection keeps the frame rate up
        self.camera.thorough = abs(self.out[0]) < 1e-3   # not driving forward
        self.cmd_pub.publish(twist)
        if (frame is not None and time.monotonic() - self.last_image_sent > 0.12
                and self.image_pub.get_subscription_count() > 0):
            self.last_image_sent = time.monotonic()  # ~8 FPS max over WiFi
            self.publish_image(frame, obs)

    def ramp(self, current, target, accel):
        # brake twice as hard as we accelerate, so stops are not overshot
        slowing = abs(target) < abs(current) or current * target < 0
        step = accel * self.dt * (2.0 if slowing else 1.0)
        return clamp(target, current - step, current + step)

    def visible(self, tag_id):
        if not hasattr(self, 'latest_obs'):
            return None
        t, obs = self.latest_obs
        if time.monotonic() - t > self.p['lost_timeout']:
            return None
        return obs.get(tag_id)

    # SCAN ------------------------------------------------------------------
    def step_scan(self, obs, pose):
        if self.mission_start is None:
            self.mission_start = time.monotonic()
        if self.p['scan_mode'] == 'step':
            return self.step_scan_stop_and_go(obs, pose)
        if obs:  # unwrapped yaw at frame time, so the scan can exceed one revolution
            scan_pose = self.scan_history.at(self.frame_stamp) or (
                self.pose[0], self.pose[1], self.scan_turned)
            for o in obs:
                self.calib.record(o, scan_pose)
        turned = abs(self.scan_turned)
        direction = 1.0 if self.p['scan_speed'] >= 0 else -1.0
        scale, n = self.calib.yaw_scale(direction)
        done = turned >= math.radians(360 + self.p['scan_extra_deg']) and scale is not None
        if done or turned >= math.radians(self.p['scan_max_deg']):
            self.finish_scan(scale, n)
            return (0.0, 0.0)
        return (0.0, self.p['scan_speed'])

    def step_scan_stop_and_go(self, obs, pose):
        """Turn ~scan_step_deg, stop, wait until still, look, repeat until 360 deg REAL.

        The real turn between stops is measured from the images (RotationEstimator), so the
        scan covers a full circle even if the wheels slip or stall (odometry may claim 500 deg
        when the robot turned 225). The learned real/odometry ratio sizes the next step and
        later dead reckoning; a step that did not move boosts the turn command.
        Only frames captured after the settle time are used (sharp, exact pose).
        """
        now = time.monotonic()
        direction = 1.0 if self.p['scan_speed'] >= 0 else -1.0
        step = math.radians(self.p['scan_step_deg'])
        if self.scan_cmd_speed is None:
            self.scan_cmd_speed = abs(self.p['scan_speed'])

        if self.scan_phase == 'turn':
            if obs:  # sharp enough with a short exposure: keep, yaw is filled in at the next stop
                self.scan_pending.append((obs, self.scan_history.at(self.frame_stamp) or
                                          (self.pose[0], self.pose[1], self.scan_turned)))
            if self.p['scan_step_time'] > 0:   # open loop: fixed turning time per step
                done = now - self.scan_phase_time >= self.p['scan_step_time']
            else:
                done = self.turn_finished(self.scan_cmd_speed,
                                          direction * (self.scan_turned - self.scan_target) >= 0)
            if done == 'stalled':
                self.scan_cmd_speed = self.boost_turn(self.scan_cmd_speed)
            elif done:
                self.scan_phase, self.scan_phase_time = 'settle', now
                return (0.0, 0.0)
            return (0.0, direction * self.scan_cmd_speed)

        if self.scan_phase == 'settle':
            if now - self.scan_phase_time >= self.p['scan_settle_time']:
                self.scan_phase, self.scan_phase_time = 'look', now
                self.scan_look_frames = 0
                self.scan_stop_yaw = None
            return (0.0, 0.0)

        # look: only frames captured after the robot came to rest
        sharp = pose is not None and self.frame_stamp >= self.scan_phase_time - self.p['camera_latency']
        if sharp and self.scan_stop_yaw is None:
            self.measure_stop_yaw(direction)
        if sharp and self.scan_stop_yaw is not None:
            self.scan_look_frames += 1
            for o in obs:
                self.calib.record(o, (self.pose[0], self.pose[1], self.scan_stop_yaw),
                                  group=self.scan_stop)
        if now - self.scan_phase_time < self.p['scan_look_time'] or self.scan_stop_yaw is None:
            return (0.0, 0.0)

        seen = sorted({o.tag_id for o, _ in self.calib.records[-60:]})
        self.get_logger().info(
            f'Scan stop {self.scan_stop}: real {math.degrees(abs(self.scan_true)):.0f} deg '
            f'(odometry {math.degrees(abs(self.scan_turned)):.0f}), {self.scan_look_frames} frames, '
            f'tags so far {seen}')
        if abs(self.scan_true) >= math.radians(360 + self.p['scan_extra_deg']) or \
                abs(self.scan_true) > math.radians(self.p['scan_max_deg']) or self.scan_stop >= 60:
            self.finish_scan_visual(direction)
            return (0.0, 0.0)
        self.scan_stop += 1
        # odometry turn that should give a real turn of `step`
        self.scan_target = self.scan_turned + direction * step / max(self.scan_ratio, 0.2)
        self.scan_phase, self.scan_phase_time = 'turn', now
        self.start_turn_track(direction * step)
        return (0.0, 0.0)

    def measure_stop_yaw(self, direction):
        """Real yaw of this stop = previous stop + turn measured from the images."""
        odom_step = self.scan_turned - self.scan_odom_at_kf
        kf, turn, how = None, None, 'start'
        if self.p['simulate']:
            if self.scan_kf is not None:
                turn = (self.truth_turned - self.scan_truth_at_kf
                        + math.radians(self.p['sim_visual_noise_deg']) * (2 * (time.monotonic() % 1) - 1))
                how = 'visual(sim)'
            self.scan_kf = True
            self.scan_truth_at_kf = self.truth_turned
        elif self.latest_frame is not None:
            kf = self.rotation.keyframe(self.latest_frame)
            if self.scan_kf is not None and self.p['scan_visual_yaw']:
                f = self.detector.focal(self.latest_frame.shape[1])
                turn, info = self.rotation.turn(self.scan_kf, kf, f)
                how = f'visual: {info}' if turn is not None else f'odometry (image: {info})'
            self.scan_kf = kf
        else:
            return  # no image yet
        if self.imu_active and self.scan_stop > 0:
            cam = f', camera {math.degrees(turn):+.1f}' if turn is not None else ''
            turn, how = odom_step, f'IMU{cam}'     # scan_turned already follows the IMU
        if self.scan_stop == 0:
            step_true = 0.0
        elif turn is not None:
            step_true = turn
            self.scan_visual_ok += 1
            if abs(odom_step) > math.radians(3) and abs(step_true) >= math.radians(3):
                # only steps that really moved: a stalled step says nothing about slip
                self.scan_vis_true += step_true
                self.scan_vis_odom += odom_step
                self.scan_ratio = clamp(self.scan_vis_true / self.scan_vis_odom, 0.1, 3.0)
        else:
            step_true = odom_step * self.scan_ratio
            if how == 'start':
                how = 'odometry'
        if self.scan_stop > 0 and abs(step_true) < math.radians(3):
            boosted = min(self.scan_cmd_speed * 1.5, self.p['scan_speed_max'])
            self.get_logger().warn(f'Robot did not turn ({math.degrees(step_true):.1f} deg): '
                                   f'turn command {self.scan_cmd_speed:.2f} -> {boosted:.2f} rad/s')
            self.turn_deadband = max(self.turn_deadband, self.scan_cmd_speed)
            self.scan_cmd_speed = boosted
        # detections taken while turning: real yaw interpolated along this step
        odom_prev = self.scan_odom_at_kf
        for frame_obs, (px, py, odom_yaw) in self.scan_pending:
            frac = clamp((odom_yaw - odom_prev) / odom_step, 0.0, 1.0) if abs(odom_step) > 1e-6 else 1.0
            for o in frame_obs:
                self.calib.record(o, (px, py, self.scan_true + frac * step_true))
        if self.scan_pending:
            n = sum(len(f) for f, _ in self.scan_pending)
            self.get_logger().info(f'  {n} tag detection(s) while turning')
        self.scan_pending = []
        self.scan_true += step_true
        self.scan_stop_yaw = self.scan_true
        self.scan_odom_at_kf = self.scan_turned
        if self.scan_stop > 0 and turn is not None:
            self.learn_turn_lag(step_true)
        if self.scan_stop > 0:
            self.get_logger().info(f'  turned {math.degrees(step_true):+.1f} deg real '
                                   f'(odometry {math.degrees(odom_step):+.1f}) by {how}, '
                                   f'real/odometry {self.scan_ratio:.2f}')

    def finish_scan_visual(self, direction):
        """Map from real (visual) yaws; tags seen again after a full turn fine-tune them."""
        corr, n = self.calib.yaw_scale_revisit(self.p['camera_offset'], direction)
        if corr is not None and 0.9 <= corr <= 1.1:
            note = f'visual yaw, fine-tuned x{corr:.3f} by {n} revisit(s)'
        else:
            corr = 1.0
            note = 'visual yaw (no revisit correction)'
        if self.scan_visual_ok == 0:
            note = 'NO visual yaw (too little texture?): odometry only'
        true_total = self.scan_true * corr
        if self.scan_vis_odom != 0.0:
            self.yaw_scale *= self.scan_ratio * corr   # real/commanded while turning, for later
        if self.turn_deadband > 0:
            self.w_turn_min = max(self.p['w_min'], self.scan_cmd_speed)
            note += (f'; turning in place needs >= {self.w_turn_min:.2f} rad/s '
                     f'({self.turn_deadband:.2f} stalled)')
        self.pose[2] = wrap(true_total)
        self.tag_map.clear()
        self.calib.build_map(self.tag_map, corr)
        tags = self.tag_map.tags()
        self.get_logger().info(
            f'Scan done: {note}; real turn {math.degrees(true_total):.0f} deg vs odometry '
            f'{math.degrees(self.scan_turned):.0f} -> yaw scale now {self.yaw_scale:.2f}; '
            f'{len(tags)} tag(s): ' + ', '.join(
                f'#{i} ({x:+.2f}, {y:+.2f}) {math.hypot(x, y):.2f} m'
                for i, (x, y) in sorted(tags.items())))
        self.replan('SCAN')

    def finish_scan(self, scale, n):
        if scale is None:
            scale = 1.0
            note = 'no tag seen twice: yaw scale unchanged'
        else:
            note = f'yaw scale x{scale:.3f} from {n} tag(s) (loop closure)'
        self.yaw_scale *= scale
        self.pose[2] = wrap(self.scan_turned * scale)
        self.tag_map.clear()
        self.calib.build_map(self.tag_map, scale)
        tags = self.tag_map.tags()
        self.get_logger().info(f'Scan done: {note}; {len(tags)} tag(s): ' + ', '.join(
            f'#{i} ({x:+.2f}, {y:+.2f}) {math.hypot(x, y):.2f} m' for i, (x, y) in sorted(tags.items())))
        self.replan('SCAN')

    # PLAN / REPLAN -----------------------------------------------------------
    def replan(self, reason):
        tags = {i: p for i, p in self.tag_map.tags().items()
                if i not in self.visited and i not in self.missed}
        if not tags:
            self.finish()
            return
        self.ids = list(tags.keys())
        self.positions = [tags[i] for i in self.ids]
        t0 = time.monotonic()
        self.tour = plan_tour((self.pose[0], self.pose[1]), self.pose[2], self.positions,
                              self.model, self.p['exact_limit'])
        order = [self.ids[k] for k in self.tour.order]
        if reason == 'SCAN':
            self.planned_time = self.tour.time
            self.tour_start = time.monotonic()
        self.get_logger().info(
            f'Plan ({self.tour.method}, {1000 * (time.monotonic() - t0):.0f} ms): {order} '
            f'-> {self.tour.distance:.2f} m, {self.tour.time:.1f} s')
        self.start_leg()

    def start_leg(self):
        leg = self.tour.legs[0]
        self.target = self.ids[leg.tag]
        self.leg_start = time.monotonic()
        self.leg_budget = self.p['leg_timeout_factor'] * leg.time + 10.0
        self.search_turned = 0.0
        self.start_align(f'-> tag {self.target}')

    # ALIGN -------------------------------------------------------------------
    def start_align(self, note=''):
        """Turn toward the target in steps whose real angle is measured from the images.

        Dead-reckoned turns are unreliable (skid steer slips differently at each turn rate,
        and the motor response is not linear), so a wrong heading would send the robot the
        wrong way. Each step: stand still -> keyframe -> turn -> stand still -> measure the
        real turn visually -> correct the heading. Repeat until facing the target or seeing it.
        """
        self.align_phase = 'settle'
        self.align_phase_time = time.monotonic()
        self.align_iter = 0
        self.align_kf = None
        self.set_state('ALIGN', note)

    # TURN TRACKING -------------------------------------------------------------
    def start_turn_track(self, goal):
        """Start a turn of `goal` rad (real, left +) that is measured from the images.

        Dead reckoning cannot tell how far a skid-steer robot really turned (slip, stall,
        non-linear motor response), so while turning the current frame is compared with the
        frame taken when the turn started (same visual method as between scan stops, on a
        640 px copy so it stays fast) and the turn stops when the measured angle is reached.
        """
        now = time.monotonic()
        self.tt_goal, self.tt_dir = goal, (1.0 if goal >= 0 else -1.0)
        self.tt_odom0, self.tt_t0, self.tt_boost_time = self.yaw_odom_total, now, now
        self.tt_value, self.tt_time, self.tt_last_try = None, 0.0, 0.0
        self.tt_stop_meas = None
        self.tt_yaw0 = self.pose[2]
        self.tt_frame = getattr(self, 'last_frame', -1)
        self.tt_ref = self.truth_turned if self.p['simulate'] else self._small_keyframe()

    def _small_keyframe(self):
        if self.latest_frame is None or not hasattr(self, 'detector'):
            return None
        img, w = self.latest_frame, self.latest_frame.shape[1]
        scale = 640.0 / w if w > 640 else 1.0
        if scale < 1.0:
            img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        return {'kf': self.rotation.keyframe(img), 'f': self.detector.focal(w) * scale}

    def turn_progress(self):
        """Real turn since start_turn_track (rad, signed), or None if not measurable now."""
        now = time.monotonic()
        if self.p['simulate']:
            if self.tt_ref is None:
                self.tt_ref = self.truth_turned
            self.tt_value, self.tt_time = self.truth_turned - self.tt_ref, now
            return self.tt_value
        if self.tt_ref is None:
            self.tt_ref = self._small_keyframe()
            return None
        if (now - self.tt_last_try >= self.p['turn_track_period'] and self.last_frame != self.tt_frame
                and self.latest_frame is not None):
            self.tt_last_try, self.tt_frame = now, self.last_frame
            cur = self._small_keyframe()
            turn, _ = self.rotation.turn(self.tt_ref['kf'], cur['kf'], cur['f'])
            if turn is not None:
                self.tt_value, self.tt_time = turn, now
        return self.tt_value if now - self.tt_time < 0.6 else None

    def turn_finished(self, w_cmd, odom_done):
        """True when the measured turn reached the goal, 'stalled' if the robot is not turning,
        else False. Falls back to dead reckoning (odom_done) when the image cannot be matched."""
        now = time.monotonic()
        goal = abs(self.tt_goal)
        odom = self.tt_dir * (self.yaw_odom_total - self.tt_odom0)
        if odom > 2.0 * goal + 0.35:
            return True                      # safety: never spin on, whatever the images say
        meas = self.turn_progress() if self.p['turn_track'] else None
        if meas is None:
            if self.imu_active and goal > math.radians(3):
                # IMU = real turn: stop early by the learned coasting (rate x lag), else the
                # robot keeps rolling past the goal and the next correction turns back
                if odom >= goal - abs(w_cmd) * self.turn_lag:
                    self.tt_stop_meas, self.tt_stop_w = odom, abs(w_cmd)
                    return True
                return False
            return odom_done
        if self.tt_dir * meas >= goal - abs(w_cmd) * self.turn_lag:
            self.tt_stop_meas, self.tt_stop_w = meas, abs(w_cmd)   # to learn the coasting
            return True
        if now - self.tt_t0 > 1.5 and abs(meas) < math.radians(1.5) and now - self.tt_boost_time > 1.0:
            self.tt_boost_time = now
            return 'stalled'
        return False

    def finish_turn_track(self):
        """Robot is still again: set the heading from the measured turn (not dead reckoning)."""
        self.tt_last_try = 0.0                     # measure now, on the newest (sharp) frame
        if self.imu_active and not self.p['turn_track']:
            # robot is still: the IMU gives the full real turn incl. coasting -> learn the lag
            final = self.yaw_odom_total - self.tt_odom0
            if self.tt_stop_meas is not None:
                before = self.turn_lag
                self.learn_turn_lag(final)
                self.get_logger().info(
                    f'Turn {math.degrees(final):+.0f} deg (goal {math.degrees(self.tt_goal):+.0f}): '
                    f'stop lag {before:.2f} -> {self.turn_lag:.2f} s')
            return
        meas = self.turn_progress()
        if meas is None or self.imu_active:      # the IMU already tracks the heading
            return
        self.pose[2] = wrap(self.tt_yaw0 + meas)
        self.learn_turn_lag(meas)

    def learn_turn_lag(self, final_turn):
        """After a tracked turn: compare the final (still) measurement with the goal and adjust
        how early to stop, so the next turns land on the requested angle."""
        if self.tt_stop_meas is None or self.tt_stop_w < 0.05 or abs(self.tt_goal) < math.radians(5):
            return
        error = self.tt_dir * (final_turn - self.tt_goal)       # + overshoot, - undershoot
        self.turn_lag = clamp(self.turn_lag + 0.5 * error / self.tt_stop_w, 0.0, 0.6)
        self.tt_stop_meas = None

    def boost_turn(self, current):
        boosted = min(current * 1.3, self.p['scan_speed_max'])
        self.turn_deadband = max(self.turn_deadband, current)
        self.w_turn_min = max(self.w_turn_min, boosted)
        self.get_logger().warn(f'Not turning (camera): turn command {current:.2f} -> {boosted:.2f} rad/s')
        return boosted

    def capture_keyframe(self):
        if self.p['simulate']:
            return {'sim': self.truth_turned}
        if self.latest_frame is None:
            return None
        return {'kf': self.rotation.keyframe(self.latest_frame), 'w': self.latest_frame.shape[1]}

    def visual_turn(self, kf_a, kf_b):
        """Real turn (rad, left +) between two keyframes, or None if not measurable."""
        if kf_a is None or kf_b is None:
            return None
        if 'sim' in kf_a:
            return kf_b['sim'] - kf_a['sim']
        turn, _ = self.rotation.turn(kf_a['kf'], kf_b['kf'], self.detector.focal(kf_b['w']))
        return turn

    def step_align(self, obs, pose):
        now = time.monotonic()
        sharp = pose is not None and self.frame_stamp >= self.align_phase_time - self.p['camera_latency']
        if self.align_phase == 'settle':          # come to rest before looking
            if now - self.align_phase_time >= self.p['scan_settle_time']:
                self.align_phase, self.align_phase_time = 'look', now
            return (0.0, 0.0)

        if self.align_phase == 'look':
            if not sharp:
                return (0.0, 0.0)
            kf = self.capture_keyframe()
            if self.align_kf is not None:          # finish the previous step: measure it
                real = self.visual_turn(self.align_kf, kf)
                est = self.yaw_odom_total - self.align_odom0
                if real is not None and not self.imu_active:
                    self.pose[2] = wrap(self.align_yaw0 + real)
                    self.learn_turn_lag(real)
                    if abs(est) > math.radians(5) and abs(real) > math.radians(2):
                        ratio = clamp(real / est, 0.3, 3.0)
                        self.yaw_scale *= 1.0 + 0.5 * (ratio - 1.0)   # learn, gently
                    if abs(real) < math.radians(2):
                        self.w_turn_min = min(self.w_turn_min * 1.3, self.p['scan_speed_max'])
                        self.get_logger().warn(f'Align: robot did not turn, turn command now '
                                               f'>= {self.w_turn_min:.2f} rad/s')
                    self.get_logger().info(
                        f'Align step: turned {math.degrees(real):+.1f} deg real '
                        f'(estimate {math.degrees(est):+.1f}), heading corrected')
            self.align_kf = kf
            self.align_yaw0 = self.pose[2]
            self.align_odom0 = self.yaw_odom_total
            if self.visible(self.target) is not None:
                self.start_navigate(f'tag {self.target} in view')
                return (0.0, 0.0)
            dist, bearing, _ = self.relative_target()
            self.align_iter += 1
            tx, ty = self.target_position()
            self.get_logger().info(
                f'Align: tag {self.target} at map ({tx:+.2f}, {ty:+.2f}), robot ({self.pose[0]:+.2f}, '
                f'{self.pose[1]:+.2f}) heading {math.degrees(self.pose[2]):+.0f} deg -> turn '
                f'{math.degrees(bearing):+.0f} deg ({"left" if bearing > 0 else "right"})')
            if abs(bearing) < math.radians(self.p['align_tolerance_deg']) or self.align_iter > 8:
                self.start_navigate(f'facing tag {self.target}')
                return (0.0, 0.0)
            step = clamp(bearing, -math.radians(self.p['align_step_deg']),
                         math.radians(self.p['align_step_deg']))
            self.align_goal = self.yaw_odom_total + step
            self.align_dir = 1.0 if step > 0 else -1.0
            self.align_phase = 'turn'
            self.start_turn_track(step)
            return (0.0, 0.0)

        # turn: until dead reckoning says the step is done, then settle and measure;
        # stop early if the target appears (sharp frames while turning with short exposure)
        remaining = self.align_dir * (self.align_goal - self.yaw_odom_total)
        meas = self.tt_value if self.tt_value is not None else None
        if meas is not None:                      # slow down on the measured remaining angle
            remaining_real = abs(self.tt_goal) - self.tt_dir * meas
        else:
            remaining_real = remaining
        w = max(self.w_turn_min, min(self.model.w_max, self.p['k_heading'] * max(remaining_real, 0)))
        done = self.turn_finished(w, remaining <= 0)
        if any(o.tag_id == self.target for o in obs):
            self.get_logger().info(f'Align: tag {self.target} seen while turning, stopping')
            done = True
        if done == 'stalled':
            self.boost_turn(w)
        elif done:
            self.align_phase, self.align_phase_time = 'settle', now
            return (0.0, 0.0)
        return (0.0, self.align_dir * w)

    def target_position(self):
        return self.tag_map.tags().get(self.target, self.positions[self.ids.index(self.target)])

    # NAVIGATE ----------------------------------------------------------------
    def relative_target(self):
        """(distance, bearing, seen) to the target from the robot centre."""
        o = self.visible(self.target)
        if o is not None:
            fwd, left = self.p['camera_offset'] + o.z, -o.x
            self.last_seen = time.monotonic()
            return math.hypot(fwd, left), math.atan2(left, fwd), True
        tx, ty = self.target_position()
        dx, dy = tx - self.pose[0], ty - self.pose[1]
        return math.hypot(dx, dy), wrap(math.atan2(dy, dx) - self.pose[2]), False

    def start_navigate(self, note=''):
        self.nav_phase = 'settle'
        self.nav_phase_time = time.monotonic()
        self.nav_blind = 0
        self.nav_seen = []
        self.nav_turned = False
        self.nav_corrections = 0
        self.set_state('NAVIGATE', note)

    def step_navigate(self, obs, pose):
        """Stop-and-go approach: look (still, sharp) -> turn or drive <= approach_step -> look.

        Frames taken while driving are motion-blurred and small tags are lost, so the target
        is only measured while standing still (thorough detection). A short dead-reckoned
        move between looks cannot go far wrong.
        """
        if time.monotonic() - self.leg_start > self.leg_budget:
            return self.skip_target('leg timeout')
        now = time.monotonic()
        sharp = pose is not None and self.frame_stamp >= self.nav_phase_time - self.p['camera_latency']

        if self.nav_phase == 'settle':
            if now - self.nav_phase_time >= self.p['scan_settle_time']:
                if self.nav_turned:
                    self.finish_turn_track()
                    self.nav_turned = False
                self.nav_phase, self.nav_phase_time, self.nav_seen = 'look', now, []
            return (0.0, 0.0)

        if self.nav_phase == 'look':
            if sharp:
                for o in obs:
                    self.tag_map.add(o, pose, weight=0.3)
                    if o.tag_id == self.target:
                        self.nav_seen.append(o)
            enough = len(self.nav_seen) >= 2
            if not enough and now - self.nav_phase_time < self.p['approach_look_time']:
                return (0.0, 0.0)
            return self.plan_approach_step()

        if self.nav_phase == 'turn':
            remaining = self.nav_dir * (self.nav_goal - self.yaw_odom_total)
            w = max(self.w_turn_min, min(self.model.w_max, self.p['k_heading'] * max(remaining, 0)))
            done = self.turn_finished(w, remaining <= 0)
            if done == 'stalled':
                self.boost_turn(w)
            elif done:
                self.nav_phase, self.nav_phase_time = 'settle', now
                self.nav_turned = True
                return (0.0, 0.0)
            return (0.0, self.nav_dir * w)

        # drive: straight, holding the heading, for the planned segment
        done = self.travelled - self.nav_start_travel
        remaining = self.nav_segment - done
        if remaining <= 0:
            self.nav_phase, self.nav_phase_time = 'settle', now
            return (0.0, 0.0)
        if self.p['drive_visual_steer'] and pose is not None:
            o = next((o for o in obs if o.tag_id == self.target), None)
            if o is not None:   # heading to the tag as seen in this frame (pose at frame time)
                seen_heading = wrap(pose[2] + math.atan2(-o.x, self.p['camera_offset'] + o.z))
                self.nav_heading = wrap(self.nav_heading + self.p['visual_steer_gain']
                                        * wrap(seen_heading - self.nav_heading))
        v = clamp(self.p['k_distance'] * remaining + self.p['v_min'], self.p['v_min'],
                  self.model.v_max)
        w = clamp(self.p['k_heading'] * wrap(self.nav_heading - self.pose[2]),
                  -self.model.w_max, self.model.w_max)
        return (v, w)

    def plan_approach_step(self):
        """After a look: use the camera if the tag is seen, else the remembered map position.

        Far away: one straight drive (aimed by the camera if the tag is seen, else by the
        remembered position) to `confirm_distance` before the standoff point, then look.
        Close and seen: short camera-guided steps (<= approach_step) to the exact standoff,
        then relocalize.
        Never sweeps around searching: if the tag is still not seen at its remembered
        position, that position counts as reached.
        """
        now = time.monotonic()
        if self.nav_seen:   # measured from sharp frames: average them
            fwd = sum(self.p['camera_offset'] + o.z for o in self.nav_seen) / len(self.nav_seen)
            left = sum(-o.x for o in self.nav_seen) / len(self.nav_seen)
            dist, bearing, seen = math.hypot(fwd, left), math.atan2(left, fwd), True
            self.latest_obs = (now, {self.target: self.nav_seen[-1]})  # for relocalize()
        else:
            dist, bearing, seen = self.relative_target()
        remaining = dist - self.model.standoff
        self.get_logger().info(
            f'Approach tag {self.target}: {dist:.2f} m, {math.degrees(bearing):+.0f} deg '
            f'({"camera" if seen else "remembered position"})')

        if remaining <= self.p['arrive_tolerance']:
            if seen:
                self.relocalize()
                self.set_state('ARRIVED', f'tag {self.target} at {dist:.2f} m')
            else:
                self.unconfirmed.append(self.target)
                self.set_state('ARRIVED', f'tag {self.target} by remembered position (not seen)')
            return (0.0, 0.0)
        if not seen and abs(bearing) > math.radians(self.p['turn_in_place_deg']):
            self.start_align(f'tag {self.target} is {math.degrees(bearing):+.0f} deg off')
            return (0.0, 0.0)
        # a seen tag a little off-centre needs no turn in place: the drive steers toward it.
        # After `approach_max_corrections` turns, drive anyway (no turning back and forth)
        turn_needed = abs(bearing) > math.radians(self.p['approach_turn_deg'])
        if (turn_needed and seen and self.nav_corrections >= self.p['approach_max_corrections']
                and abs(bearing) <= math.radians(self.p['turn_in_place_deg']) + 0.2):
            turn_needed = False
        if turn_needed:
            self.nav_corrections += 1
            self.nav_goal = self.yaw_odom_total + bearing
            self.nav_dir = 1.0 if bearing > 0 else -1.0
            self.nav_phase, self.nav_phase_time = 'turn', now
            self.start_turn_track(bearing)
            return (0.0, 0.0)
        if remaining > self.p['confirm_distance'] + 0.1:
            segment = remaining - self.p['confirm_distance']   # one long drive, then look once
        elif seen:
            segment = min(remaining, self.p['approach_step'])  # camera-guided finish
        else:
            segment = remaining                                 # close, not seen: finish by the map
        self.nav_corrections = 0
        self.nav_segment = segment
        self.nav_start_travel = self.travelled
        self.nav_heading = wrap(self.pose[2] + bearing)
        self.nav_phase, self.nav_phase_time = 'drive', now
        return (0.0, 0.0)

    def relocalize(self):
        o = self.visible(self.target)
        if o is None:
            return
        tx, ty = self.target_position()
        fwd, left = self.p['camera_offset'] + o.z, -o.x
        yaw = self.pose[2]
        mx = tx - (fwd * math.cos(yaw) - left * math.sin(yaw))
        my = ty - (fwd * math.sin(yaw) + left * math.cos(yaw))
        g = self.p['relocalize_gain']
        err = math.hypot(mx - self.pose[0], my - self.pose[1])
        self.pose[0] += g * (mx - self.pose[0])
        self.pose[1] += g * (my - self.pose[1])
        self.get_logger().info(f'Relocalized on tag {self.target}: corrected {err:.2f} m of drift')

    # SEARCH ------------------------------------------------------------------
    def step_search(self, obs, pose):
        """Sweep left by local_search_deg, then right by 2x, looking for the target."""
        if self.visible(self.target) is not None:
            self.start_navigate(f'found tag {self.target}')
            return (0.0, 0.0)
        limit = math.radians(self.p['local_search_deg'])
        speed = max(self.w_turn_min, 0.5 * self.model.w_max)
        self.search_turned += speed * self.dt   # total angle swept so far
        phase = self.search_turned
        if phase < limit:
            return (0.0, speed)
        if phase < 3 * limit:
            return (0.0, -speed)
        return self.skip_target('not found by local search')

    def skip_target(self, why):
        self.get_logger().warn(f'Skipping tag {self.target}: {why}')
        self.missed.append(self.target)
        self.target = None
        self.replan('skip')
        return (0.0, 0.0)

    # ARRIVED / DONE ------------------------------------------------------------
    def step_arrived(self, obs, pose):
        if time.monotonic() - self.state_time < self.p['dwell_time']:
            return (0.0, 0.0)
        self.visited.append(self.target)
        self.get_logger().info(f'Visited tag {self.target} ({len(self.visited)} done)')
        self.target = None
        self.replan('arrived')
        return (0.0, 0.0)

    def finish(self):
        now = time.monotonic()
        scan = (self.tour_start or now) - self.mission_start if self.mission_start else 0.0
        tour = now - self.tour_start if self.tour_start else 0.0
        dwell = self.p['dwell_time'] * len(self.visited)
        self.target = None
        unconf = f' (not seen on arrival: {self.unconfirmed})' if self.unconfirmed else ''
        self.set_state('DONE', f'visited {self.visited}{unconf}, missed {self.missed}; scan {scan:.1f} s, '
                               f'tour {tour:.1f} s incl. {dwell:.0f} s dwell '
                               f'(planned {self.planned_time:.1f} s), {self.travelled:.2f} m driven')

    # ------------------------------------------------------------------ outputs
    def publish_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def marker(mid, mtype, ns):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'tag_map', stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        for i, (x, y) in self.tag_map.tags().items():
            color = (0.1, 0.9, 0.1) if i in self.visited else (
                (0.9, 0.1, 0.1) if i in self.missed else (1.0, 0.6, 0.0))
            m = marker(i, Marker.CYLINDER, 'tags')
            x, y = float(x), float(y)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.1
            m.scale.x = m.scale.y = 0.15
            m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.0
            t = marker(i, Marker.TEXT_VIEW_FACING, 'ids')
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, 0.35
            t.scale.z = 0.2
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f'#{i}'
            arr.markers += [m, t]
        path = marker(0, Marker.LINE_STRIP, 'plan')
        path.scale.x = 0.03
        path.color.g, path.color.b, path.color.a = 0.6, 1.0, 1.0
        path.points.append(Point(x=float(self.pose[0]), y=float(self.pose[1])))
        if self.tour and self.state in ('ALIGN', 'NAVIGATE', 'SEARCH', 'ARRIVED'):
            for leg in self.tour.legs:
                path.points.append(Point(x=float(leg.stop[0]), y=float(leg.stop[1])))
        arr.markers.append(path)
        robot = marker(0, Marker.ARROW, 'robot')
        robot.pose.position.x, robot.pose.position.y = float(self.pose[0]), float(self.pose[1])
        robot.pose.orientation.z = float(math.sin(self.pose[2] / 2))
        robot.pose.orientation.w = float(math.cos(self.pose[2] / 2))
        robot.scale.x, robot.scale.y, robot.scale.z = 0.4, 0.12, 0.12
        robot.color.r, robot.color.b, robot.color.a = 0.8, 0.8, 1.0
        arr.markers.append(robot)
        self.marker_pub.publish(arr)
        self.publish_status()

    def publish_image(self, frame, obs):
        img, scale = preview_frame(frame)
        for o in obs:
            if o.corners is None:
                continue
            pts = (o.corners * scale).astype(int).reshape(-1, 1, 2)
            color = (0, 255, 0) if o.tag_id == self.target else (0, 200, 255)
            cv2.polylines(img, [pts], True, color, 2)
            x, y = pts[0][0]
            cv2.putText(img, f'#{o.tag_id} {o.distance:.2f}m', (int(x), int(y) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        cv2.putText(img, f'{self.state} target={self.target} visited={len(self.visited)}',
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        data = encode_jpeg(img)
        if data:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format, msg.data = 'jpeg', data
            self.image_pub.publish(msg)

    def destroy_node(self):
        self.camera.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TagMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
            except Exception:  # context already shut down by Ctrl+C
                pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
