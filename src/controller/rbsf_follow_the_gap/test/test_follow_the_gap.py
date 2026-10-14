import math
import time

import numpy as np
import pytest
from rbsf_follow_the_gap.follow_the_gap import FollowTheGap, Inflation, largest_gap

ANGLE_MIN = -math.radians(135.0)
INCREMENT = math.radians(0.25)
N = 1081
RANGE_MAX = 30.0
CONE_RADIUS = 0.1
ANGLES = ANGLE_MIN + INCREMENT * np.arange(N)
DX, DY = np.cos(ANGLES), np.sin(ANGLES)


def cast(walls=(), cones=()):
    """
    Simulate the LaserScan ranges (inf: no return) of wall segments and cones.

    Walls are ((x0, y0), (x1, y1)) segments, cones (x, y) circles of CONE_RADIUS. Rays start at
    the origin, the car's LiDAR.
    """
    ranges = np.full(N, np.inf)
    for (x0, y0), (x1, y1) in walls:
        ex, ey = x1 - x0, y1 - y0
        denom = DX * ey - DY * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (x0 * ey - y0 * ex) / denom        # along the ray
            s = (x0 * DY - y0 * DX) / denom        # along the segment, 0..1
        hit = (np.abs(denom) > 1e-12) & (t > 0.0) & (s >= 0.0) & (s <= 1.0)
        ranges = np.where(hit, np.minimum(ranges, t), ranges)
    for cx, cy in cones:
        b = DX * cx + DY * cy
        disc = b * b - (cx * cx + cy * cy - CONE_RADIUS ** 2)
        t = b - np.sqrt(np.maximum(disc, 0.0))
        hit = (disc >= 0.0) & (t > 0.0)
        ranges = np.where(hit, np.minimum(ranges, t), ranges)
    return np.where(ranges < RANGE_MAX, ranges, np.inf).tolist()


def corridor(left, right, angle=0.0, length=20.0):
    """Walls `left` and `right` m beside the car, rotated `angle` rad, from 2 m behind it."""
    ux, uy = math.cos(angle), math.sin(angle)
    walls = []
    for d in (left, -right):
        walls.append(((-2.0 * ux - d * uy, -2.0 * uy + d * ux),
                      (length * ux - d * uy, length * uy + d * ux)))
    return walls


def box(x0, x1, y0, y1):
    """Return the four walls of an axis-aligned box."""
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)),
            ((x0, y1), (x0, y0))]


def cone_row(y, gap, x_range=(-2.0, 20.0)):
    """Cones along x at lateral position y (centres), `gap` m between neighbours."""
    return [(x, y) for x in np.arange(x_range[0], x_range[1], gap + 2 * CONE_RADIUS)]


def ftg(**kwargs):
    return FollowTheGap(**kwargs)


def drive(controller, ranges, scans=1):
    for _ in range(scans):
        cmd = controller.update(ranges, ANGLE_MIN, INCREMENT, RANGE_MAX)
    return cmd


def brute_force_inflate(ranges, increment, radius, max_range):
    """Inflate one point at a time: each blocks ceil(asin(radius / r) / increment) beams a side."""
    out = ranges.copy()
    full = math.ceil(0.5 * math.pi / increment)
    for i, r in enumerate(ranges):
        if r >= max_range:
            continue
        k = min(full, math.ceil(math.asin(radius / r) / increment)) if r > radius else full
        lo, hi = max(0, i - k), min(len(ranges), i + k + 1)
        out[lo:hi] = np.minimum(out[lo:hi], r)
    return out


def test_largest_gap():
    f = np.array([0, 1, 1, 0, 1, 1, 1, 0], dtype=bool)
    assert largest_gap(f) == (4, 7)
    assert largest_gap(np.zeros(5, dtype=bool)) is None
    assert largest_gap(np.ones(5, dtype=bool)) == (0, 5)
    assert largest_gap(np.array([1, 1, 0, 0, 0, 1, 1], dtype=bool), heading=6) == (5, 7)


def test_largest_gap_tie_prefers_the_heading():
    f = np.array([1, 1, 0, 1, 1, 0, 0, 0, 1, 1], dtype=bool)
    assert largest_gap(f, heading=4) == (3, 5)
    assert largest_gap(f, heading=9) == (8, 10)
    assert largest_gap(f, heading=0) == (0, 2)


@pytest.mark.parametrize('seed', range(10))
def test_inflation_matches_brute_force(seed):
    rng = np.random.default_rng(seed)
    max_range, radius, size = 10.0, 0.25, 961
    ranges = rng.uniform(0.05, 12.0, size).astype(np.float32)
    ranges[rng.random(size) < 0.3] = max_range
    ranges = np.minimum(ranges, max_range)
    inflated = Inflation(size, INCREMENT, radius)(ranges, max_range)
    assert np.array_equal(inflated, brute_force_inflate(ranges, INCREMENT, radius, max_range))


def test_inflation_half_widths():
    inflation = Inflation(100, INCREMENT, 0.25)
    full = math.ceil(0.5 * math.pi / INCREMENT)
    k = inflation.half_widths(np.array([0.1, 0.25, 0.6, 1.0, 1e6]))
    assert k[0] == k[1] == full
    assert k[2] == math.ceil(math.asin(0.25 / 0.6) / INCREMENT)
    assert k[3] == math.ceil(math.asin(0.25) / INCREMENT)
    assert k[4] == 1


def test_inflation_without_obstacles_is_a_copy():
    ranges = np.full(50, 10.0, dtype=np.float32)
    assert np.array_equal(Inflation(50, INCREMENT, 0.25)(ranges, 10.0), ranges)


def test_centred_corridor_goes_straight_at_full_speed():
    cmd = drive(ftg(), cast(corridor(1.0, 1.0)))
    assert abs(cmd.steering) < 0.01
    assert cmd.speed == pytest.approx(2.0, abs=0.05)


@pytest.mark.parametrize('left, right', [(0.5, 1.5), (1.5, 0.5)])
def test_off_centre_steers_back_to_the_centre(left, right):
    cmd = drive(ftg(), cast(corridor(left, right)))
    assert np.sign(cmd.steering) == np.sign(left - right)


@pytest.mark.parametrize('angle', [0.3, -0.3])
def test_corridor_turning_steers_into_it_and_slows_down(angle):
    cmd = drive(ftg(), cast(corridor(1.0, 1.0, angle)))
    assert np.sign(cmd.steering) == np.sign(angle)
    assert cmd.speed < 2.0


@pytest.mark.parametrize('side', [1.0, -1.0])
def test_static_obstacle_ahead_is_avoided(side):
    # A box covering the centre and the `side` half of a 3 m corridor, 1.2 m ahead.
    y0, y1 = sorted((0.0, side * 0.6))
    walls = corridor(1.5, 1.5) + box(1.2, 1.5, y0, y1)
    cmd = drive(ftg(), cast(walls))
    assert np.sign(cmd.steering) == -side
    assert np.sign(cmd.target_angle) == -side


def test_obstacle_beyond_the_threshold_is_ignored_until_closer():
    # At standstill the threshold is base_gap_threshold: a box 3 m ahead leaves the centre free.
    walls = corridor(1.5, 1.5) + box(3.0, 3.3, -0.1, 0.4)
    cmd = drive(ftg(speed_gain=0.0), cast(walls))
    assert abs(cmd.steering) < 0.05
    # Driving fast raises the threshold above 3 m, so the same box is avoided.
    cmd = drive(ftg(speed_gain=1.0), cast(walls), scans=3)
    assert cmd.threshold > 3.0 and cmd.steering < -0.05


def test_cone_rows_read_as_walls():
    # Cone boundaries with 0.4 m gaps: no beam leaks through a gap, so the car stays centred.
    cones = cone_row(1.0 + CONE_RADIUS, 0.4) + cone_row(-1.0 - CONE_RADIUS, 0.4)
    controller = ftg()
    cmd = drive(controller, cast(cones=cones))
    assert abs(cmd.steering) < 0.02
    side = np.abs(controller.layout.angles) > math.radians(45.0)
    assert np.all(controller.inflated[side] < 2.0)


def test_cone_row_across_the_path_blocks_it():
    # A row of cones 0.8 m ahead, across the corridor: no gap fits the car, so it stops.
    cones = [(0.8, y) for y in np.arange(-1.0, 1.0, 0.4 + 2 * CONE_RADIUS)]
    cmd = drive(ftg(), cast(walls=corridor(1.0, 1.0), cones=cones))
    assert cmd.speed == 0.0 and cmd.target_angle is None


def test_boxed_in_stops():
    cmd = drive(ftg(), cast(box(-0.7, 0.7, -0.7, 0.7)))
    assert cmd.speed == 0.0 and cmd.target_angle is None


def test_speed_never_below_min_speed_while_moving():
    # A narrow corridor turning left: the steering saturates, the speed stays at min_speed.
    cmd = drive(ftg(min_speed=0.8, max_speed=1.2, lookahead=0.8), cast(corridor(0.8, 0.8, 0.6)))
    assert cmd.target_angle is not None and cmd.steering == pytest.approx(0.4189)
    assert cmd.speed == pytest.approx(0.8)


def test_invalid_returns_are_open_space():
    ranges = cast(corridor(1.0, 1.0))
    ranges[540:560] = [float('nan')] * 10 + [0.0] * 10
    controller = ftg()
    drive(controller, ranges)
    assert np.all(np.isfinite(controller.inflated))
    assert controller.inflated.max() == pytest.approx(controller.max_range)


def test_layout_rebuilds_when_the_scan_changes():
    controller = ftg()
    drive(controller, cast(corridor(1.0, 1.0)))
    first = controller.layout
    half = controller.update([5.0] * 541, ANGLE_MIN, 2 * INCREMENT, RANGE_MAX)
    assert controller.layout is not first and controller.layout.key[0] == 541
    assert abs(half.steering) < 0.01


def test_steering_low_pass():
    walls = cast(corridor(1.0, 1.0, 0.3))
    raw = drive(ftg(), walls).steering
    filtered = drive(ftg(steering_alpha=0.5), walls).steering
    assert filtered == pytest.approx(0.5 * raw)


def test_invalid_parameters_raise():
    with pytest.raises(ValueError):
        ftg(target_mode='widest')
    with pytest.raises(ValueError):
        ftg(min_speed=3.0, max_speed=2.0)
    with pytest.raises(ValueError):
        ftg(base_gap_threshold=5.0, max_gap_threshold=4.0)


def test_deepest_target_mode_aims_further_into_a_corner():
    walls = cast(corridor(1.0, 1.0, 0.3))
    middle = drive(ftg(), walls)
    deepest = drive(ftg(target_mode='deepest'), walls)
    assert deepest.target_distance >= middle.target_distance - 1e-6


def test_update_is_fast():
    walls = corridor(1.5, 1.5, 0.2) + box(1.5, 1.8, 0.0, 0.5)
    ranges = np.asarray(cast(walls), dtype=np.float32)
    controller = ftg()
    drive(controller, ranges, scans=20)
    runs = 500
    start = time.perf_counter()
    drive(controller, ranges, scans=runs)
    mean = (time.perf_counter() - start) / runs
    print(f'FollowTheGap.update: {1e6 * mean:.0f} us per {N}-beam scan')
    assert mean < 1e-3
