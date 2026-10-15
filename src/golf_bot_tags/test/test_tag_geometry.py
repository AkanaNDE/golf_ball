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
