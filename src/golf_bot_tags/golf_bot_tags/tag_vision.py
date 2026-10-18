"""AprilTag detection and distance-from-size geometry (no ROS).

Distance from apparent size (pinhole camera):
    Z = f * tag_size / h_px

where h_px is the mean length of the tag's two *vertical* edges in pixels and f is the focal
length in pixels. Vertical edges are used because a tag that is turned left/right keeps its
vertical edge length (only its horizontal edges shrink), so Z stays correct for tags viewed
at an angle; the horizontal edges only shrink with cos(angle).

Camera frame used here: Z forward, X to the right (metres).
    X = (u_center - cx) * Z / f
"""

import math
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

FAMILIES = {
    '36h11': cv2.aruco.DICT_APRILTAG_36h11,
    '25h9': cv2.aruco.DICT_APRILTAG_25h9,
    '16h5': cv2.aruco.DICT_APRILTAG_16h5,
    '36h10': cv2.aruco.DICT_APRILTAG_36h10,
}


@dataclass
class TagObservation:
    tag_id: int
    z: float            # m, forward distance from the camera
    x: float            # m, to the right of the optical axis
    u: float            # horizontal image position, -1 (left edge) .. +1 (right edge)
    h_px: float         # apparent tag height in pixels
    stamp: float        # time.monotonic() of the frame
    corners: object = None  # 4x2 pixel corners (for drawing)

    @property
    def distance(self):
        return math.hypot(self.z, self.x)

    @property
    def bearing(self):
        """Angle to the tag, positive = to the left (ROS convention)."""
        return math.atan2(-self.x, self.z)


def set_exposure(cap, exposure_ms=0.0, gain=-1.0, dev=None, controls=''):
    """Short manual exposure so frames stay sharp while the robot turns.

    Motion blur in pixels = turn rate (rad/s) x exposure (s) x focal (px). Indoors auto
    exposure is often 20-33 ms: at 0.4 rad/s and f = 670 px that is 5-9 px of blur, and
    AprilTags are lost above ~2 px. 5 ms gives ~1.3 px. Raise gain (or add light) to
    compensate the darker image. exposure_ms <= 0 keeps auto exposure.
    V4L2 units: exposure is in 100 us steps; AUTO_EXPOSURE 1 = manual, 3 = auto.
    `controls` (e.g. 'auto_exposure=1,exposure_time_absolute=50') is passed to v4l2-ctl
    for cameras whose controls OpenCV cannot set. Returns a short description.
    """
    notes = []
    if exposure_ms > 0:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure_ms * 10.0)
        notes.append(f'exposure {cap.get(cv2.CAP_PROP_EXPOSURE) / 10.0:.1f} ms '
                     f'(asked {exposure_ms:.1f}, auto={cap.get(cv2.CAP_PROP_AUTO_EXPOSURE):.0f})')
    if gain >= 0:
        cap.set(cv2.CAP_PROP_GAIN, gain)
        notes.append(f'gain {cap.get(cv2.CAP_PROP_GAIN):.0f}')
    if controls and dev:
        import subprocess
        r = subprocess.run(['v4l2-ctl', '-d', str(dev), '-c', controls],
                           capture_output=True, text=True)
        notes.append(f'v4l2-ctl {controls}: ' + ('ok' if r.returncode == 0 else r.stderr.strip()))
    return ', '.join(notes) if notes else 'auto exposure'


def open_camera(camera='auto', width=640, height=480, fourcc='MJPG'):
    """Open a V4L2 camera; 'auto' tries every /dev/video* until one returns a frame.

    Opens by index (OpenCV 5 cannot open V4L2 devices by path), and skips the many
    non-camera /dev/video* nodes of a Raspberry Pi 5.
    """
    import glob
    import re
    camera = str(camera)
    devs = (sorted(glob.glob('/dev/video*'), key=lambda d: int(d[10:] or 0))
            if camera == 'auto' else [camera])
    for dev in devs:
        if re.fullmatch(r'/dev/video\d+', dev):
            index = int(dev[len('/dev/video'):])
        else:
            index = int(dev) if dev.isdigit() else dev
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if cap.isOpened():
            if fourcc:  # MJPG: USB webcams only give full frame rate at 720p+ in MJPG
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if cap.read()[0]:
                return dev, cap
        cap.release()
    raise RuntimeError(f'No working camera found (tried {devs}). Check: v4l2-ctl --list-devices')


def encode_jpeg(img, quality=60, max_width=640):
    if img.shape[1] > max_width:  # preview only: keep WiFi traffic low at high resolutions
        scale = max_width / img.shape[1]
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return jpg.tobytes() if ok else None


def focal_from_hfov(width, hfov_deg):
    return (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


class TagDetector:
    """Two detection modes (measured on 1280x720 with 4 cm tags):
      fast      SUBPIX corners, ~4 ms/frame, tags up to ~1.0 m   -> while the robot moves
      thorough  AprilTag quad detection, ~130 ms/frame (laptop), up to ~1.5 m -> when still
    Moving frames are blurred anyway, so the slow mode is only worth it when standing still.
    """

    def __init__(self, family='36h11', tag_size=0.15, focal_px=0.0, hfov_deg=70.0,
                 min_side_px=12.0):
        self.dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
        fast = cv2.aruco.DetectorParameters()
        fast.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        thorough = cv2.aruco.DetectorParameters()
        thorough.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.fast = cv2.aruco.ArucoDetector(self.dictionary, fast)
        self.thorough = cv2.aruco.ArucoDetector(self.dictionary, thorough)
        self.detector = self.thorough
        self.tag_size = tag_size
        self.focal_px = focal_px
        self.hfov_deg = hfov_deg
        self.min_side_px = min_side_px

    def focal(self, width):
        return self.focal_px if self.focal_px > 0 else focal_from_hfov(width, self.hfov_deg)

    def detect(self, frame, stamp, thorough=True):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        h, w = gray.shape[:2]
        f = self.focal(w)
        corners, ids, _ = (self.thorough if thorough else self.fast).detectMarkers(gray)
        observations = []
        if ids is None:
            return observations
        for c, tag_id in zip(corners, ids.ravel()):
            c = c.reshape(4, 2)  # TL, TR, BR, BL
            left = np.linalg.norm(c[3] - c[0])
            right = np.linalg.norm(c[2] - c[1])
            h_px = 0.5 * (left + right)
            if h_px < self.min_side_px:
                continue  # too small to trust the distance
            cx_px = float(c[:, 0].mean())
            z = f * self.tag_size / h_px
            x = (cx_px - w / 2.0) * z / f
            u = (cx_px - w / 2.0) / (w / 2.0)
            # plain Python floats: numpy.float32 values crash ROS message fields later
            observations.append(TagObservation(int(tag_id), float(z), float(x), float(u),
                                               float(h_px), stamp, c))
        return observations


class RotationEstimator:
    """Measure how far the robot really turned between two still images (visual yaw).

    For a camera turning about a vertical axis, a scene point at image column u has the
    bearing atan((u - cx) / f); after a turn of d (left positive) it appears at bearing + d.

    1. ORB features: match descriptors, the median bearing change is the turn
       (precise, needs some texture; low FAST threshold for plain indoor scenes).
    2. Fallback, cylindrical phase correlation: both images are unrolled onto a cylinder
       (column = bearing, f pixels per radian) where a pure turn is a pure horizontal
       shift, found with cv2.phaseCorrelate over the whole image - works on blurry or
       low-texture images where ORB finds too few corners.
    Independent of wheel slip or stalled wheels.
    """

    def __init__(self, features=1500, ratio=0.8, max_dy_px=25.0, min_matches=12,
                 inlier_deg=2.0, min_inlier_frac=0.35, fast_threshold=7, min_response=0.08):
        self.orb = cv2.ORB_create(features, fastThreshold=fast_threshold)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.ratio = ratio
        self.max_dy = max_dy_px
        self.min_matches = min_matches
        self.inlier = math.radians(inlier_deg)
        self.min_inlier_frac = min_inlier_frac
        self.min_response = min_response
        self._maps = {}

    def keyframe(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        gray = cv2.equalizeHist(gray)   # more corners in dull / dark scenes
        kp, des = self.orb.detectAndCompute(gray, None)
        return {'kp': kp, 'des': des, 'w': gray.shape[1], 'gray': gray}

    # --- 1. features -------------------------------------------------------------
    def _turn_features(self, kf1, kf2, focal):
        if kf1['des'] is None or kf2['des'] is None or len(kf1['kp']) < 2 or len(kf2['kp']) < 2:
            return None, 0
        cx = kf1['w'] / 2.0
        diffs = []
        for pair in self.matcher.knnMatch(kf1['des'], kf2['des'], k=2):
            if len(pair) < 2 or pair[0].distance > self.ratio * pair[1].distance:
                continue
            p1 = kf1['kp'][pair[0].queryIdx].pt
            p2 = kf2['kp'][pair[0].trainIdx].pt
            if abs(p1[1] - p2[1]) > self.max_dy:   # pure rotation: rows stay (almost) the same
                continue
            # turning left moves the scene to the right in the image
            diffs.append(math.atan((p2[0] - cx) / focal) - math.atan((p1[0] - cx) / focal))
        if len(diffs) < self.min_matches:
            return None, len(diffs)
        diffs.sort()
        med = diffs[len(diffs) // 2]
        inliers = [d for d in diffs if abs(d - med) < self.inlier]
        if len(inliers) < self.min_inlier_frac * len(diffs) or len(inliers) < self.min_matches:
            return None, len(inliers)
        return sum(inliers) / len(inliers), len(inliers)

    # --- 2. cylindrical phase correlation -----------------------------------------
    def _cylinder(self, gray, focal):
        h, w = gray.shape
        key = (w, h, round(focal, 1))
        if key not in self._maps:
            half = math.atan((w / 2.0) / focal)
            cols = int(2 * half * focal)              # f pixels per radian
            theta = (np.arange(cols) - cols / 2.0) / focal
            rows = np.arange(h) - h / 2.0
            th, yy = np.meshgrid(theta, rows)
            map_x = (w / 2.0 + focal * np.tan(th)).astype(np.float32)
            map_y = (h / 2.0 + yy / np.cos(th)).astype(np.float32)
            window = cv2.createHanningWindow((cols, h), cv2.CV_32F)
            self._maps[key] = (map_x, map_y, window)
        map_x, map_y, window = self._maps[key]
        cyl = cv2.remap(gray, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return cyl.astype(np.float32), window

    def _turn_phase(self, kf1, kf2, focal):
        # a featureless image (plain wall, lens cap) has nothing to correlate
        if min(float(kf1['gray'].std()), float(kf2['gray'].std())) < 4.0:
            return None, 0.0
        c1, window = self._cylinder(kf1['gray'], focal)
        c2, _ = self._cylinder(kf2['gray'], focal)
        (dx, dy), response = cv2.phaseCorrelate(c1, c2, window)
        if response < self.min_response or abs(dy) > self.max_dy:
            return None, response
        return dx / focal, response

    def turn(self, kf1, kf2, focal):
        """(turn_rad, method_info) from keyframe 1 to keyframe 2, or (None, info)."""
        turn, n = self._turn_features(kf1, kf2, focal)
        if turn is not None:
            return turn, f'{n} feature matches'
        turn_p, resp = self._turn_phase(kf1, kf2, focal)
        if turn_p is not None:
            return turn_p, f'phase correlation {resp:.2f} ({n} matches)'
        return None, f'{n} matches, phase {resp:.2f}'


class CameraThread:
    """Reads the camera and runs detection in the background; keeps only the newest result."""

    def __init__(self, cap, detector, flip=False):
        self.cap = cap
        self.detector = detector
        self.flip = flip
        self.thorough = True    # set False while the robot moves (fast detection)
        self.restore_auto_exposure = False
        self.lock = threading.Lock()
        self.frame = None
        self.observations = []
        self.stamp = 0.0
        self.frames = 0
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            ok, frame = self.cap.read()
            stamp = time.monotonic()
            if not ok:
                time.sleep(0.05)
                continue
            if self.flip:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            obs = self.detector.detect(frame, stamp, self.thorough)
            with self.lock:
                self.frame, self.observations, self.stamp = frame, obs, stamp
                self.frames += 1

    def latest(self):
        with self.lock:
            return self.frame, list(self.observations), self.stamp, self.frames

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        if self.restore_auto_exposure:  # manual exposure would stay on the camera otherwise
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        self.cap.release()


class SimulatedCamera:
    """Fake camera for dry runs without hardware.

    `truth_pose()` returns the *true* robot pose (x, y, yaw); tags are at known world
    positions. Produces noisy observations of every tag inside the field of view.
    Motion blur: if `truth_rate()` (true yaw rate, rad/s) is above `blur_speed`, the frame
    is too blurred and no tag is detected (like the real camera while turning).
    """

    def __init__(self, tags, truth_pose, hfov_deg=70.0, max_range=6.0, noise=0.03,
                 camera_offset=0.0, rate=15.0, seed=0, truth_rate=None, blur_speed=0.0,
                 detector=None, width=640, height=480):
        self.tags = tags  # {id: (x, y)}
        # With a detector the simulator renders real tag images and runs the real
        # detection on them (same code path and data types as on the robot).
        self.detector = detector
        self.width, self.height = width, height
        self.frame = None
        self.thorough = True
        self.truth_speed = None      # true forward speed (m/s), for forward-motion blur
        self.blur_linear = 0.0
        self.truth_pose = truth_pose
        self.truth_rate = truth_rate
        self.blur_speed = blur_speed
        self.half_fov = math.radians(hfov_deg) / 2.0
        self.max_range = max_range
        self.noise = noise
        self.offset = camera_offset
        self.period = 1.0 / rate
        self.rng = np.random.default_rng(seed)
        self.lock = threading.Lock()
        self.observations = []
        self.stamp = 0.0
        self.frames = 0
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            x, y, yaw = self.truth_pose()
            cx, cy = x + self.offset * math.cos(yaw), y + self.offset * math.sin(yaw)
            obs = []
            blurred = (self.blur_speed > 0 and self.truth_rate is not None
                       and abs(self.truth_rate()) > self.blur_speed)
            blurred = blurred or (self.blur_linear > 0 and self.truth_speed is not None
                                  and abs(self.truth_speed()) > self.blur_linear)
            for tag_id, (tx, ty) in ({} if blurred else self.tags).items():
                dx, dy = tx - cx, ty - cy
                fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
                left = -dx * math.sin(yaw) + dy * math.cos(yaw)
                if fwd < 0.2 or math.hypot(fwd, left) > self.max_range:
                    continue
                if abs(math.atan2(left, fwd)) > self.half_fov:
                    continue
                z = fwd * (1.0 + self.rng.normal(0.0, self.noise))
                x_right = -left + self.rng.normal(0.0, 0.01)
                u = (x_right / z) / math.tan(self.half_fov)
                obs.append(TagObservation(int(tag_id), float(z), float(x_right), float(u),
                                          float(100.0 / z), time.monotonic()))
            frame = None
            if self.detector is not None:
                frame = self._render(obs)
                obs = self.detector.detect(frame, time.monotonic(), self.thorough)
            with self.lock:
                self.observations, self.stamp, self.frame = obs, time.monotonic(), frame
                self.frames += 1
            time.sleep(self.period)

    def _render(self, obs):
        """Grey image with each visible tag drawn at its projected position and size."""
        w, h = self.width, self.height
        f = self.detector.focal(w)
        img = np.full((h, w), 170, np.uint8)
        for o in sorted(obs, key=lambda o: -o.z):           # far tags first
            side = int(round(f * self.detector.tag_size / o.z))
            if side < 8:
                continue
            marker = cv2.aruco.generateImageMarker(self.detector.dictionary, o.tag_id, side)
            pad = max(2, side // 8)                          # white quiet zone around the tag
            marker = cv2.copyMakeBorder(marker, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
            cx = int(round(w / 2 + f * o.x / o.z))
            x0, y0 = cx - marker.shape[1] // 2, h // 2 - marker.shape[0] // 2
            xs, ys = max(0, x0), max(0, y0)
            xe, ye = min(w, x0 + marker.shape[1]), min(h, y0 + marker.shape[0])
            if xe > xs and ye > ys:
                img[ys:ye, xs:xe] = marker[ys - y0:ye - y0, xs - x0:xe - x0]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    def latest(self):
        with self.lock:
            return self.frame, list(self.observations), self.stamp, self.frames

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
