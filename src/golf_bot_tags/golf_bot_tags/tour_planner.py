"""Time-optimal tour through all AprilTags for a skid-steer robot (pure Python, no ROS).

Why not a plain shortest-path TSP: a skid-steer robot spends real time turning in place,
so the cost of going from tag i to tag k depends on the heading it arrived at i with,
i.e. on the tag p it came from. Each leg is modelled as "turn in place, then drive straight":

    A(p, i)  approach point: `standoff` metres before tag i on the line p -> i
    H(p, i)  heading while driving p -> i
    t(p, i, k) = |H(i,k)' - H(p,i)| / w_max         turn at A(p,i) toward A(i,k)
               + |A(p,i) - A(i,k)| / v_max           drive
               + leg_overhead                        accelerate / brake / settle

The cost depends only on (p, i, k), so Held-Karp dynamic programming over
(visited set, previous tag, last tag) gives the exact optimum of this model.
The robot does not return to the start (open tour). Objective = total time; total distance
is reported too (and is what the tour minimises when w_max is very large).

Solver:
  n <= exact_limit  Held-Karp, O(2^n * n^3) -> exact (n = 9: well under a second)
  n >  exact_limit  cheapest insertion + 2-opt / or-opt local search on the same cost
"""

import itertools
import math
from dataclasses import dataclass, field


@dataclass
class MotionModel:
    v_max: float = 0.3          # m/s cruise speed
    w_max: float = 1.0          # rad/s turn-in-place speed
    standoff: float = 0.4       # m, stop this far in front of each tag
    leg_overhead: float = 0.5   # s per leg: accelerate, decelerate, settle


@dataclass
class Leg:
    tag: int                    # index into the tag list
    start: tuple                # (x, y) where the leg starts
    stop: tuple                 # (x, y) approach point in front of the tag
    heading: float              # rad, heading while driving this leg
    turn: float                 # rad, signed turn in place before driving
    distance: float             # m driven
    time: float                 # s


@dataclass
class Tour:
    order: list = field(default_factory=list)   # tag indices in visiting order
    legs: list = field(default_factory=list)
    time: float = 0.0
    distance: float = 0.0
    method: str = ''


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class _Geometry:
    """Approach points / headings between nodes. Node n is the start pose."""

    def __init__(self, start, start_heading, tags, model):
        self.pos = [tuple(t) for t in tags] + [tuple(start)]
        self.n = len(tags)
        self.h0 = start_heading
        self.m = model

    def approach(self, p, i):
        (px, py), (ix, iy) = self.pos[p], self.pos[i]
        dx, dy = ix - px, iy - py
        d = math.hypot(dx, dy)
        heading = math.atan2(dy, dx) if d > 1e-9 else (self.h0 if p == self.n else 0.0)
        if d <= self.m.standoff:
            return (px, py), heading          # already close enough: stay
        s = self.m.standoff / d
        return (ix - dx * s, iy - dy * s), heading

    def leg(self, p, i, k):
        """Leg to tag k, standing at A(p, i) facing H(p, i). p = i = n means 'at the start'."""
        if i == self.n:
            here, heading = self.pos[self.n], self.h0
        else:
            here, heading = self.approach(p, i)
        stop, _ = self.approach(i, k)
        dx, dy = stop[0] - here[0], stop[1] - here[1]
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) if dist > 1e-9 else heading
        turn = wrap(bearing - heading)
        time = abs(turn) / self.m.w_max + dist / self.m.v_max + self.m.leg_overhead
        return Leg(k, here, stop, bearing, turn, dist, time)


def evaluate(order, start, start_heading, tags, model):
    """Cost of a visiting order under the planner's model."""
    g = _Geometry(start, start_heading, tags, model)
    tour = Tour(order=list(order))
    p, i = g.n, g.n
    for k in order:
        leg = g.leg(p, i, k)
        tour.legs.append(leg)
        tour.time += leg.time
        tour.distance += leg.distance
        p, i = i, k
    return tour


def _held_karp(g):
    n, S = g.n, g.n
    INF = float('inf')
    cost = {}

    def c(p, i, k):
        key = (p, i, k)
        if key not in cost:
            cost[key] = g.leg(p, i, k).time
        return cost[key]

    dp, parent = {}, {}
    for j in range(n):
        dp[(1 << j, S, j)] = c(S, S, j)
    for size in range(1, n):
        for subset in itertools.combinations(range(n), size):
            mask = sum(1 << b for b in subset)
            for i in subset:
                prevs = [S] if size == 1 else [p for p in subset if p != i]
                for p in prevs:
                    base = dp.get((mask, p, i), INF)
                    if base == INF:
                        continue
                    for k in range(n):
                        if mask >> k & 1:
                            continue
                        key = (mask | 1 << k, i, k)
                        value = base + c(p, i, k)
                        if value < dp.get(key, INF):
                            dp[key] = value
                            parent[key] = p
    full = (1 << n) - 1
    key = min((k for k in dp if k[0] == full), key=dp.get)
    order = []
    while True:
        mask, p, i = key
        order.append(i)
        if p == S:
            break
        key = (mask & ~(1 << i), parent[key], p)
    return order[::-1]


def _local_search(start, start_heading, tags, model):
    n = len(tags)

    def total(order):
        return evaluate(order, start, start_heading, tags, model).time

    order, remaining = [], list(range(n))
    while remaining:  # cheapest insertion
        best = min(((total(order[:pos] + [k] + order[pos:]), pos, k)
                    for k in remaining for pos in range(len(order) + 1)))
        order.insert(best[1], best[2])
        remaining.remove(best[2])

    best_t, improved = total(order), True
    while improved:
        improved = False
        for i in range(n - 1):                      # 2-opt: reverse a segment
            for j in range(i + 1, n):
                cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                t = total(cand)
                if t < best_t - 1e-9:
                    order, best_t, improved = cand, t, True
        for i in range(n):                          # or-opt: move one tag
            for j in range(n):
                if i != j:
                    cand = order[:i] + order[i + 1:]
                    cand.insert(j, order[i])
                    t = total(cand)
                    if t < best_t - 1e-9:
                        order, best_t, improved = cand, t, True
    return order


def plan_tour(start, start_heading, tags, model=None, exact_limit=9):
    """Best visiting order for `tags` (list of (x, y)) from `start` facing `start_heading`."""
    model = model or MotionModel()
    if not tags:
        return Tour(method='empty')
    if len(tags) <= exact_limit:
        order = _held_karp(_Geometry(start, start_heading, tags, model))
        method = 'held-karp (exact)'
    else:
        order = _local_search(start, start_heading, tags, model)
        method = 'insertion + 2-opt/or-opt'
    tour = evaluate(order, start, start_heading, tags, model)
    tour.method = method
    return tour
