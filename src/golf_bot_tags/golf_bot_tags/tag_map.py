"""Tag map: fuse many noisy observations into one position per tag (no ROS).

1. Observation weighting
   Distance from size has a pixel error of ~1 px on the tag height, so the relative depth
   error is sigma_px / h_px: far (small) tags are trusted less. Tags near the image edge get
   extra variance for lens distortion. Every observation is projected into the map frame
   with the robot pose *at the time the frame was captured* and averaged with weight 1/var.

2. Scan self-calibration (visual loop closure)
   The robot turns in place for a bit more than one revolution. Every tag crosses the image
   centre once per real revolution, so the yaw the odometry *believes* it turned between
   two centre crossings of the same tag corresponds to exactly 2*pi. That gives a yaw scale
   factor (skid-steer wheels slip when turning, so commanded/encoder yaw is usually wrong)
   that corrects all scan observations and the rest of the mission's dead reckoning.

3. Pose history
   Poses are stored with timestamps so an observation can use the pose at frame time
   (minus camera latency) instead of the pose when it is processed - this matters while
   turning at 0.4 rad/s.
"""

import bisect
import math
from collections import defaultdict, deque


def _project(pose, obs, camera_offset):
    x, y, yaw = pose
    fwd = camera_offset + obs.z
    left = -obs.x
    return (x + fwd * math.cos(yaw) - left * math.sin(yaw),
            y + fwd * math.sin(yaw) + left * math.cos(yaw))


class TagMap:
    def __init__(self, camera_offset=0.0, pixel_sigma=1.0, min_observations=3):
        self.camera_offset = camera_offset
        self.pixel_sigma = pixel_sigma
        self.min_observations = min_observations
        self.clear()

    def clear(self):
        self.acc = defaultdict(lambda: [0.0, 0.0, 0.0, 0])  # sum_w, sum_wx, sum_wy, count

    def variance(self, obs):
        rel = self.pixel_sigma / max(obs.h_px, 1.0)
        sigma_depth = obs.z * rel + 0.02
        return sigma_depth ** 2 * (1.0 + 2.0 * obs.u ** 2)

    def add(self, obs, pose, weight=1.0):
        px, py = _project(pose, obs, self.camera_offset)
        w = weight / self.variance(obs)
        a = self.acc[obs.tag_id]
        a[0] += w
        a[1] += w * px
        a[2] += w * py
        a[3] += 1

    def tags(self):
        """{tag_id: (x, y)} for tags seen at least min_observations times."""
        return {tid: (a[1] / a[0], a[2] / a[0]) for tid, a in self.acc.items()
                if a[3] >= self.min_observations and a[0] > 0}

    def count(self, tag_id):
        return self.acc[tag_id][3] if tag_id in self.acc else 0


class ScanCalibrator:
    """Records scan observations with the odometry yaw, then estimates the true yaw scale."""

    def __init__(self, min_scale=0.6, max_scale=1.6):
        self.records = []  # (obs, pose)
        self.min_scale = min_scale
        self.max_scale = max_scale
        self.yaw0 = None

    def record(self, obs, pose):
        if self.yaw0 is None:
            self.yaw0 = pose[2]
        self.records.append((obs, pose))

    def _unwrapped(self):
        # Odometry yaw is wrapped to (-pi, pi]; unwrap it in time order
        out, prev, offset = [], None, 0.0
        for obs, pose in self.records:
            yaw = pose[2]
            if prev is not None:
                d = yaw - prev
                if d < -math.pi:
                    offset += 2 * math.pi
                elif d > math.pi:
                    offset -= 2 * math.pi
            prev = yaw
            out.append((obs, pose, yaw + offset))
        return out

    def crossings(self, direction=1.0):
        """{tag_id: [unwrapped yaw at each centre crossing]}"""
        per_tag = defaultdict(list)
        for obs, pose, yaw in self._unwrapped():
            per_tag[obs.tag_id].append((yaw, obs.u))
        result = {}
        for tag_id, seq in per_tag.items():
            seq.sort()
            found = []
            for (y1, u1), (y2, u2) in zip(seq, seq[1:]):
                # Turning left (direction > 0): tags move left -> right through the centre
                if direction * u1 < 0 <= direction * u2 and y2 - y1 < 0.3:
                    found.append(y1 + (y2 - y1) * (-u1) / (u2 - u1))
            # Keep crossings at least half a turn apart (ignore jitter around the centre)
            clean = []
            for c in found:
                if not clean or abs(c - clean[-1]) > math.pi:
                    clean.append(c)
            result[tag_id] = clean
        return result

    def turned(self):
        """Unwrapped odometry yaw turned so far."""
        u = self._unwrapped()
        return abs(u[-1][2] - u[0][2]) if len(u) > 1 else 0.0

    def yaw_scale(self, direction=1.0):
        """(scale, n_tags_used); scale = true / odometry yaw. None if not measurable."""
        ratios = []
        for crossings in self.crossings(direction).values():
            if len(crossings) >= 2:
                turned = abs(crossings[1] - crossings[0])
                ratios.append(2 * math.pi / turned)
        ratios = [r for r in ratios if self.min_scale <= r <= self.max_scale]
        if not ratios:
            return None, 0
        ratios.sort()
        return ratios[len(ratios) // 2], len(ratios)

    def build_map(self, tag_map, scale):
        """Fill tag_map from the scan, with odometry yaw corrected by `scale`."""
        for obs, pose, yaw in self._unwrapped():
            corrected = self.yaw0 + (yaw - self.yaw0) * scale
            tag_map.add(obs, (pose[0], pose[1], corrected))


class PoseHistory:
    """Dead-reckoned pose (x, y, yaw) with a short time history for frame-time lookup."""

    def __init__(self, max_age=3.0):
        self.max_age = max_age
        self.times = deque()
        self.poses = deque()

    def push(self, t, pose):
        self.times.append(t)
        self.poses.append(pose)
        while self.times and t - self.times[0] > self.max_age:
            self.times.popleft()
            self.poses.popleft()

    def at(self, t):
        if not self.times:
            return None
        times = list(self.times)
        i = bisect.bisect_left(times, t)
        if i <= 0:
            return self.poses[0]
        if i >= len(times):
            return self.poses[-1]
        t0, t1 = times[i - 1], times[i]
        p0, p1 = self.poses[i - 1], self.poses[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        dyaw = math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))
        return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]), p0[2] + a * dyaw)
