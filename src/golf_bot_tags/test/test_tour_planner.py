import itertools
import math
import random
import time

from golf_bot_tags.tour_planner import MotionModel, evaluate, plan_tour


def brute_force(start, heading, tags, model):
    return min((evaluate(p, start, heading, tags, model) for p in
                itertools.permutations(range(len(tags)))), key=lambda t: t.time)


def random_tags(rng, n):
    return [(rng.uniform(-4, 4), rng.uniform(-4, 4)) for _ in range(n)]


def test_exact_matches_brute_force():
    rng = random.Random(1)
    model = MotionModel(v_max=0.3, w_max=0.8, standoff=0.4, leg_overhead=0.5)
    for n in range(1, 7):
        for _ in range(15):
            tags = random_tags(rng, n)
            heading = rng.uniform(-math.pi, math.pi)
            tour = plan_tour((0.0, 0.0), heading, tags, model)
            best = brute_force((0.0, 0.0), heading, tags, model)
            assert sorted(tour.order) == list(range(n))
            assert abs(tour.time - best.time) < 1e-9


def test_turning_cost_changes_the_order():
    # Two tags at the same distance: one straight ahead, one behind.
    # Ignoring turns both orders are equal; with slow turning, ahead-first must win.
    tags = [(-2.0, 0.0), (2.0, 0.0)]
    model = MotionModel(v_max=0.3, w_max=0.3, standoff=0.0, leg_overhead=0.0)
    tour = plan_tour((0.0, 0.0), 0.0, tags, model)
    assert tour.order == [1, 0]


def test_heuristic_close_to_optimal():
    rng = random.Random(7)
    model = MotionModel()
    worst = 0.0
    for _ in range(10):
        tags = random_tags(rng, 8)
        exact = plan_tour((0, 0), 0.0, tags, model, exact_limit=9)
        heur = plan_tour((0, 0), 0.0, tags, model, exact_limit=0)
        assert heur.method.startswith('insertion')
        worst = max(worst, heur.time / exact.time - 1.0)
    assert worst < 0.05  # within 5 % of the optimum


def test_nine_tags_fast_enough():
    tags = random_tags(random.Random(3), 9)
    t0 = time.monotonic()
    tour = plan_tour((0, 0), 0.0, tags, MotionModel())
    assert tour.method.startswith('held-karp')
    assert time.monotonic() - t0 < 5.0


def test_standoff_and_empty():
    model = MotionModel(standoff=0.5, leg_overhead=0.0)
    tour = plan_tour((0, 0), 0.0, [(2.0, 0.0)], model)
    assert abs(tour.distance - 1.5) < 1e-9
    assert abs(tour.legs[0].stop[0] - 1.5) < 1e-9
    assert plan_tour((0, 0), 0.0, [], model).order == []
