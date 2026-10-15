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

Safety: the node only publishes /cmd_vel_auto. xbox_teleop forwards it to /cmd_vel while
AUTO mode (Y) is on and the joystick is alive; LB overrides, B stops. When AUTO is switched
off the mission pauses and resumes where it was; switching AUTO on after DONE starts again.
"""

import math
import time

import cv2
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from golf_bot_tags.tag_map import PoseHistory, ScanCalibrator, TagMap
from golf_bot_tags.tag_vision import CameraThread, SimulatedCamera, TagDetector
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
            ('camera_offset', 0.15), ('camera_latency', 0.08),
            # pose estimate
            ('pose_source', 'cmd'),          # 'cmd' (/cmd_vel) or 'odom' (/odom twist)
            ('yaw_scale', 1.0), ('linear_scale', 1.0),
            # scan
            ('scan_speed', 0.4), ('scan_extra_deg', 45.0), ('scan_max_deg', 540.0),
            ('min_observations', 3),
            # planner
            ('v_max', 0.25), ('w_max', 0.8), ('standoff', 0.5), ('leg_overhead', 0.5),
            ('exact_limit', 10),
            # control
            ('rate', 20.0), ('k_heading', 1.5), ('k_distance', 0.8), ('v_min', 0.06),
            ('w_min', 0.25), ('turn_in_place_deg', 25.0), ('arrive_tolerance', 0.08),
            ('lost_timeout', 0.6), ('local_search_deg', 70.0), ('dwell_time', 1.0),
            ('leg_timeout_factor', 3.0), ('relocalize_gain', 0.7),
            ('require_auto', True),
            # simulation (dry run without camera/robot)
            ('simulate', False), ('sim_tags', [0.0]), ('sim_yaw_scale', 0.85),
            ('sim_noise', 0.03),
        ])
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p = {n: g(n) for n in [
            'camera_offset', 'camera_latency', 'pose_source', 'yaw_scale', 'linear_scale',
            'scan_speed', 'scan_extra_deg', 'scan_max_deg', 'k_heading', 'k_distance', 'v_min',
            'w_min', 'turn_in_place_deg', 'arrive_tolerance', 'lost_timeout',
            'local_search_deg', 'dwell_time', 'leg_timeout_factor', 'relocalize_gain',
            'require_auto', 'exact_limit', 'simulate', 'sim_yaw_scale']}
        self.model = MotionModel(g('v_max'), g('w_max'), g('standoff'), g('leg_overhead'))
        self.dt = 1.0 / g('rate')
        self.yaw_scale = float(g('yaw_scale'))
        self.tag_map = TagMap(self.p['camera_offset'], min_observations=g('min_observations'))
        self.history = PoseHistory()

        # pose estimate (map frame = robot start pose) and simulated truth
        self.pose = [0.0, 0.0, 0.0]
        self.truth = [0.0, 0.0, 0.0]
        self.vel = (0.0, 0.0)
        self.vel_time = 0.0
        self.travelled = 0.0

        # camera
        if self.p['simulate']:
            flat = list(g('sim_tags'))
            tags = {i: (flat[2 * i], flat[2 * i + 1]) for i in range(len(flat) // 2)}
            self.camera = SimulatedCamera(tags, lambda: tuple(self.truth), g('hfov_deg'),
                                          noise=g('sim_noise'), camera_offset=self.p['camera_offset'])
            self.get_logger().info(f'SIMULATION: {len(tags)} tags {tags}, true yaw scale '
                                   f'{self.p["sim_yaw_scale"]}')
        else:
            detector = TagDetector(g('tag_family'), g('tag_size'), g('focal_px'), g('hfov_deg'))
            cam, cap = self.open_camera(str(g('camera')), g('width'), g('height'))
            self.camera = CameraThread(cap, detector, g('flip'))
            self.get_logger().info(
                f'Camera {cam}, tag {g("tag_family")} {g("tag_size") * 100:.1f} cm, '
                f'focal {detector.focal(g("width")):.0f} px')
        self.last_frame = -1

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.status_pub = self.create_publisher(String, 'tag_mission/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'tag_mission/markers', 2)
        self.image_pub = self.create_publisher(CompressedImage, 'tag_mission/image/compressed', 2)
        self.create_subscription(Bool, 'auto_mode', self.on_auto, 10)
        if self.p['pose_source'] == 'odom':
            self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_timer(self.dt, self.on_timer)
        self.create_timer(0.5, self.publish_markers)

        self.auto = not self.p['require_auto']
        self.state = 'IDLE'
        self.state_time = time.monotonic()
        self.reset_mission()
        if not self.p['require_auto']:
            self.set_state('SCAN', '(require_auto = false)')
        self.get_logger().info('Tag mission ready: press Y (AUTO) to start' if self.p['require_auto']
                               else 'Tag mission starting (require_auto = false)')

    # ------------------------------------------------------------------ setup
    def open_camera(self, camera, width, height):
        import glob
        import re
        devs = sorted(glob.glob('/dev/video*'), key=lambda d: int(d[10:] or 0)) \
            if camera == 'auto' else [camera]
        for dev in devs:
            index = int(dev[10:]) if re.fullmatch(r'/dev/video\d+', dev) else (
                int(dev) if dev.isdigit() else dev)
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if cap.read()[0]:
                    return dev, cap
            cap.release()
        raise RuntimeError(f'No working camera found in {devs}')

    def reset_mission(self):
        self.pose = [0.0, 0.0, 0.0]
        self.tag_map.clear()
        self.calib = ScanCalibrator()
        self.calib.yaw0 = 0.0          # the scan starts at the map origin, facing +x
        self.scan_turned = 0.0         # unwrapped yaw turned during the scan
        self.scan_history = PoseHistory()
        self.ids, self.positions = [], []
        self.tour = None
        self.visited, self.missed = [], []
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

    def on_cmd(self, msg):
        self.vel = (msg.linear.x, msg.angular.z)
        self.vel_time = time.monotonic()

    def on_odom(self, msg):
        self.vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.vel_time = time.monotonic()

    def integrate(self):
        now = time.monotonic()
        v, w = self.vel if now - self.vel_time < 0.3 else (0.0, 0.0)
        if self.p['simulate']:  # the "real" robot under-rotates by sim_yaw_scale
            tw = w * self.p['sim_yaw_scale']
            self.truth[0] += v * math.cos(self.truth[2] + tw * self.dt / 2) * self.dt
            self.truth[1] += v * math.sin(self.truth[2] + tw * self.dt / 2) * self.dt
            self.truth[2] = wrap(self.truth[2] + tw * self.dt)
        v *= self.p['linear_scale']
        w *= self.yaw_scale
        x, y, yaw = self.pose
        self.pose = [x + v * math.cos(yaw + w * self.dt / 2) * self.dt,
                     y + v * math.sin(yaw + w * self.dt / 2) * self.dt,
                     wrap(yaw + w * self.dt)]
        if self.state == 'SCAN':
            self.scan_turned += w * self.dt
            self.scan_history.push(now, (self.pose[0], self.pose[1], self.scan_turned))
        self.travelled += abs(v) * self.dt
        self.history.push(now, tuple(self.pose))

    def new_observations(self):
        """Observations from a frame not processed yet, with the pose at frame time."""
        frame, obs, stamp, frames = self.camera.latest()
        if frames == self.last_frame:
            return [], None, frame
        self.last_frame = frames
        self.frame_stamp = stamp - self.p['camera_latency']
        pose = self.history.at(self.frame_stamp) or tuple(self.pose)
        return obs, pose, frame

    # ------------------------------------------------------------------ main loop
    def on_timer(self):
        self.integrate()
        obs, pose, frame = self.new_observations()
        if obs:
            self.latest_obs = (time.monotonic(), {o.tag_id: o for o in obs})
        cmd = (0.0, 0.0)
        active = self.auto and self.state not in ('IDLE', 'DONE')
        if active:
            handler = getattr(self, 'step_' + self.state.lower())
            cmd = handler(obs, pose)
        twist = Twist()
        twist.linear.x, twist.angular.z = cmd
        self.cmd_pub.publish(twist)
        if frame is not None and self.image_pub.get_subscription_count() > 0:
            self.publish_image(frame, obs)

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
        self.set_state('NAVIGATE', f'-> tag {self.target}')

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

    def drive_to(self, dist, bearing):
        remaining = dist - self.model.standoff
        w = clamp(self.p['k_heading'] * bearing, -self.model.w_max, self.model.w_max)
        if abs(bearing) > math.radians(self.p['turn_in_place_deg']):
            if abs(w) < self.p['w_min']:
                w = math.copysign(self.p['w_min'], w)
            return (0.0, w)
        v = clamp(self.p['k_distance'] * remaining, self.p['v_min'], self.model.v_max)
        return (v * math.cos(bearing), w)

    def step_navigate(self, obs, pose):
        for o in obs:  # keep refining the map with everything in view (lower weight)
            self.tag_map.add(o, pose, weight=0.3)
        if time.monotonic() - self.leg_start > self.leg_budget:
            return self.skip_target('leg timeout')
        dist, bearing, seen = self.relative_target()
        if dist - self.model.standoff <= self.p['arrive_tolerance']:
            if seen:
                self.relocalize()
                self.set_state('ARRIVED', f'tag {self.target} at {dist:.2f} m')
                return (0.0, 0.0)
            self.search_turned = 0.0
            self.set_state('SEARCH', f'tag {self.target} not visible at its map position')
            return (0.0, 0.0)
        return self.drive_to(dist, bearing)

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
            self.set_state('NAVIGATE', f'found tag {self.target}')
            return (0.0, 0.0)
        limit = math.radians(self.p['local_search_deg'])
        speed = max(self.p['w_min'], 0.5 * self.model.w_max)
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
        self.set_state('DONE', f'visited {self.visited}, missed {self.missed}; scan {scan:.1f} s, '
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
        path.points.append(Point(x=self.pose[0], y=self.pose[1]))
        if self.tour and self.state in ('NAVIGATE', 'SEARCH', 'ARRIVED'):
            for leg in self.tour.legs:
                path.points.append(Point(x=leg.stop[0], y=leg.stop[1]))
        arr.markers.append(path)
        robot = marker(0, Marker.ARROW, 'robot')
        robot.pose.position.x, robot.pose.position.y = self.pose[0], self.pose[1]
        robot.pose.orientation.z = math.sin(self.pose[2] / 2)
        robot.pose.orientation.w = math.cos(self.pose[2] / 2)
        robot.scale.x, robot.scale.y, robot.scale.z = 0.4, 0.12, 0.12
        robot.color.r, robot.color.b, robot.color.a = 0.8, 0.8, 1.0
        arr.markers.append(robot)
        self.marker_pub.publish(arr)
        self.publish_status()

    def publish_image(self, frame, obs):
        img = frame.copy()
        for o in obs:
            if o.corners is None:
                continue
            pts = o.corners.astype(int).reshape(-1, 1, 2)
            color = (0, 255, 0) if o.tag_id == self.target else (0, 200, 255)
            cv2.polylines(img, [pts], True, color, 2)
            x, y = pts[0][0]
            cv2.putText(img, f'#{o.tag_id} {o.distance:.2f}m', (int(x), int(y) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        cv2.putText(img, f'{self.state} target={self.target} visited={len(self.visited)}',
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format, msg.data = 'jpeg', jpg.tobytes()
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
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
