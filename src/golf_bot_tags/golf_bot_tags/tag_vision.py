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


def open_camera(camera='auto', width=640, height=480):
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
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if cap.read()[0]:
                return dev, cap
        cap.release()
    raise RuntimeError(f'No working camera found (tried {devs}). Check: v4l2-ctl --list-devices')


def encode_jpeg(img, quality=60):
    ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return jpg.tobytes() if ok else None


def focal_from_hfov(width, hfov_deg):
    return (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


class TagDetector:
    def __init__(self, family='36h11', tag_size=0.15, focal_px=0.0, hfov_deg=70.0,
                 min_side_px=12.0):
        self.dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        self.tag_size = tag_size
        self.focal_px = focal_px
        self.hfov_deg = hfov_deg
        self.min_side_px = min_side_px

    def focal(self, width):
        return self.focal_px if self.focal_px > 0 else focal_from_hfov(width, self.hfov_deg)

    def detect(self, frame, stamp):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        h, w = gray.shape[:2]
        f = self.focal(w)
        corners, ids, _ = self.detector.detectMarkers(gray)
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
            observations.append(TagObservation(int(tag_id), z, x, u, h_px, stamp, c))
        return observations


class RotationEstimator:
    """Measure how far the robot really turned between two still images (visual yaw).

    For a camera turning about a vertical axis, a scene point at image column u has the
    bearing atan((u - cx) / f); after a turn of d (left positive) the same point appears
    at bearing + d. ORB features are matched between the two images and the median
    bearing change is the true turn - independent of wheel slip or stalled wheels.
    """

    def __init__(self, features=1500, ratio=0.75, max_dy_px=25.0, min_matches=15,
                 inlier_deg=2.0, min_inlier_frac=0.4):
        self.orb = cv2.ORB_create(features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.ratio = ratio
        self.max_dy = max_dy_px
        self.min_matches = min_matches
        self.inlier = math.radians(inlier_deg)
        self.min_inlier_frac = min_inlier_frac

    def keyframe(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        kp, des = self.orb.detectAndCompute(gray, None)
        return {'kp': kp, 'des': des, 'w': gray.shape[1]}

    def turn(self, kf1, kf2, focal):
        """(turn_rad, n_inliers) from keyframe 1 to keyframe 2, or (None, n) if unreliable."""
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


class CameraThread:
    """Reads the camera and runs detection in the background; keeps only the newest result."""

    def __init__(self, cap, detector, flip=False):
        self.cap = cap
        self.detector = detector
        self.flip = flip
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
            obs = self.detector.detect(frame, stamp)
            with self.lock:
                self.frame, self.observations, self.stamp = frame, obs, stamp
                self.frames += 1

    def latest(self):
        with self.lock:
            return self.frame, list(self.observations), self.stamp, self.frames

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()


class SimulatedCamera:
    """Fake camera for dry runs without hardware.

    `truth_pose()` returns the *true* robot pose (x, y, yaw); tags are at known world
    positions. Produces noisy observations of every tag inside the field of view.
    Motion blur: if `truth_rate()` (true yaw rate, rad/s) is above `blur_speed`, the frame
    is too blurred and no tag is detected (like the real camera while turning).
    """

    def __init__(self, tags, truth_pose, hfov_deg=70.0, max_range=6.0, noise=0.03,
                 camera_offset=0.0, rate=15.0, seed=0, truth_rate=None, blur_speed=0.0):
        self.tags = tags  # {id: (x, y)}
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
                obs.append(TagObservation(int(tag_id), z, x_right, u, 100.0 / z, time.monotonic()))
            with self.lock:
                self.observations, self.stamp = obs, time.monotonic()
                self.frames += 1
            time.sleep(self.period)

    def latest(self):
        with self.lock:
            return None, list(self.observations), self.stamp, self.frames

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
