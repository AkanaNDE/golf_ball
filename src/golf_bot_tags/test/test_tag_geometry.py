import math

import cv2
import numpy as np

from golf_bot_tags.tag_map import PoseHistory, ScanCalibrator, TagMap
from golf_bot_tags.tag_vision import TagDetector, TagObservation


def render_tag(tag_id, z, x, f=600.0, size=0.15, w=640, h=480):
    """Synthetic camera image of a tag facing the camera at depth z, x metres right."""
    img = np.full((h, w), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    side_px = int(round(f * size / z))
    # The black square is 8 of the 10 cells of the marker image (1-cell white border)
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, side_px)
    cx = int(round(w / 2 + f * x / z))
    top, left = h // 2 - side_px // 2, cx - side_px // 2
    img[top:top + side_px, left:left + side_px] = marker
    return img


def test_distance_and_offset_from_size():
    det = TagDetector('36h11', tag_size=0.15, focal_px=600.0)
    for z, x in [(0.6, 0.0), (1.0, 0.2), (2.0, -0.3), (3.0, 0.5)]:
        obs = det.detect(render_tag(5, z, x), 0.0)
        assert len(obs) == 1 and obs[0].tag_id == 5
        assert abs(obs[0].z - z) / z < 0.03, (z, obs[0].z)
        assert abs(obs[0].x - x) < 0.03 + 0.02 * z, (x, obs[0].x)


def simulate_scan(true_tags, yaw_scale_true, odom_turn=2 * math.pi * 1.25, steps=900):
    """Robot turns in place; odometry believes it turned odom_turn, truth = x yaw_scale_true."""
    cal = ScanCalibrator()
    cal.yaw0 = 0.0
    half_fov = math.radians(35)
    for k in range(steps):
        odom = odom_turn * k / steps
        true = odom * yaw_scale_true
        for tid, (tx, ty) in true_tags.items():
            fwd = tx * math.cos(true) + ty * math.sin(true)
            left = -tx * math.sin(true) + ty * math.cos(true)
            if fwd > 0.2 and abs(math.atan2(left, fwd)) < half_fov:
                u = (-left / fwd) / math.tan(half_fov)
                cal.record(TagObservation(tid, fwd, -left, u, 600 * 0.15 / fwd, 0.0), (0, 0, odom))
    return cal


def test_scan_loop_closure_recovers_yaw_scale_and_map():
    tags = {0: (2.0, 1.0), 1: (-1.5, 2.0), 2: (1.0, -2.5), 3: (-2.0, -1.0)}
    cal = simulate_scan(tags, yaw_scale_true=0.85, odom_turn=2 * math.pi / 0.85 * 1.15)
    scale, n = cal.yaw_scale(direction=1.0)
    assert n >= 1
    assert abs(scale - 0.85) < 0.01
    m = TagMap(camera_offset=0.0, min_observations=3)
    cal.build_map(m, scale)
    for tid, (x, y) in tags.items():
        mx, my = m.tags()[tid]
        assert math.hypot(mx - x, my - y) < 0.05


def test_pose_history_interpolates():
    h = PoseHistory()
    h.push(0.0, (0.0, 0.0, 0.0))
    h.push(1.0, (1.0, 2.0, 1.0))
    x, y, yaw = h.at(0.5)
    assert abs(x - 0.5) < 1e-9 and abs(y - 1.0) < 1e-9 and abs(yaw - 0.5) < 1e-9


def test_far_observations_weigh_less():
    m = TagMap(min_observations=1)
    near = TagObservation(1, 1.0, 0.0, 0.0, 90.0, 0.0)
    far = TagObservation(1, 4.0, 0.0, 0.0, 22.5, 0.0)
    m.add(near, (0.0, 0.0, 0.0))
    m.add(TagObservation(1, 4.4, 0.0, 0.0, 22.5, 0.0), (-3.0, 0.0, 0.0))  # says x = 1.4
    x, _ = m.tags()[1]
    assert abs(x - 1.0) < 0.05  # dominated by the near observation
    assert m.variance(far) > m.variance(near)


def test_stop_and_go_revisit_recovers_yaw_scale_and_map():
    """Turn 30 deg (odometry), stop, look; true rotation is only 0.85 x odometry."""
    import random
    rng = random.Random(4)
    tags = {0: (2.0, 1.0), 1: (-1.5, 2.0), 2: (1.0, -2.5), 3: (-2.0, -1.0)}
    true_scale, offset, half_fov = 0.85, 0.15, math.radians(35)
    cal = ScanCalibrator()
    cal.yaw0 = 0.0
    stop, odom = 0, 0.0
    while odom <= math.radians(480):
        true = odom * true_scale
        cx, cy = offset * math.cos(true), offset * math.sin(true)  # camera ahead of centre
        for tid, (tx, ty) in tags.items():
            dx, dy = tx - cx, ty - cy
            fwd = dx * math.cos(true) + dy * math.sin(true)
            left = -dx * math.sin(true) + dy * math.cos(true)
            if fwd > 0.2 and abs(math.atan2(left, fwd)) < half_fov:
                for _ in range(4):  # a few sharp frames per stop
                    z = fwd * (1 + rng.gauss(0, 0.02))
                    x = -left + rng.gauss(0, 0.005)
                    u = (x / z) / math.tan(half_fov)
                    cal.record(TagObservation(tid, z, x, u, 600 * 0.15 / z, 0.0), (0, 0, odom),
                               group=stop)
        stop += 1
        odom += math.radians(30)
    scale, n = cal.yaw_scale_revisit(camera_offset=offset)
    assert n >= 3
    assert abs(scale - true_scale) < 0.01, scale
    m = TagMap(camera_offset=offset, min_observations=3)
    cal.build_map(m, scale)
    for tid, (x, y) in tags.items():
        mx, my = m.tags()[tid]
        assert math.hypot(mx - x, my - y) < 0.08, (tid, mx, my)


def textured_scene(seed=0, w=640, h=480):
    rng = np.random.default_rng(seed)
    img = np.full((h, w, 3), 128, np.uint8)
    for _ in range(400):
        color = tuple(int(c) for c in rng.integers(0, 255, 3))
        x, y = int(rng.integers(0, w)), int(rng.integers(0, h))
        if rng.random() < 0.5:
            cv2.circle(img, (x, y), int(rng.integers(3, 25)), color, -1)
        else:
            cv2.rectangle(img, (x, y), (x + int(rng.integers(5, 40)), y + int(rng.integers(5, 40))),
                          color, -1)
    return img


def rotated_view(img, turn_rad, f):
    """What the camera sees after turning left by turn_rad (pure rotation homography)."""
    h, w = img.shape[:2]
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])
    c, s = math.cos(turn_rad), math.sin(turn_rad)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    H = K @ R @ np.linalg.inv(K)
    return cv2.warpPerspective(img, H, (w, h))


def test_visual_rotation_estimate():
    from golf_bot_tags.tag_vision import RotationEstimator
    est = RotationEstimator()
    f = 457.0
    scene = textured_scene()
    kf0 = est.keyframe(scene)
    for deg in (5, 13, 30, -20):
        turn, n = est.turn(kf0, est.keyframe(rotated_view(scene, math.radians(deg), f)), f)
        assert turn is not None and n >= 15, (deg, n)
        assert abs(math.degrees(turn) - deg) < 1.0, (deg, math.degrees(turn))
    # no texture -> no estimate instead of a wrong one
    blank = np.full((480, 640, 3), 128, np.uint8)
    assert est.turn(est.keyframe(blank), est.keyframe(blank), f)[0] is None
