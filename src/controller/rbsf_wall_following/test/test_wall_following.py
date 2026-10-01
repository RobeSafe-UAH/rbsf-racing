import math

import pytest
from rbsf_wall_following.wall_following import PI, ray_range, wall_errors, WallFollower

ANGLE_MIN = -math.radians(135.0)
INCREMENT = math.radians(0.25)
N = 1081
RANGE_MAX = 30.0


def wall_scan(distance, wall_angle=0.0, side='left'):
    """
    Scan of one straight wall `distance` m to the side, rotated `wall_angle` rad from the heading.

    The wall passes through (0, +-distance); a positive angle makes it diverge (left wall) ahead.
    """
    sign = 1.0 if side == 'left' else -1.0
    px, py = 0.0, sign * distance
    dx, dy = math.cos(sign * wall_angle), math.sin(sign * wall_angle)
    ranges = []
    for i in range(N):
        a = ANGLE_MIN + i * INCREMENT
        rx, ry = math.cos(a), math.sin(a)
        denom = rx * dy - ry * dx
        r = (px * dy - py * dx) / denom if abs(denom) > 1e-9 else -1.0
        ranges.append(r if 0.0 < r < RANGE_MAX else math.inf)
    return ranges


def follower(side='left', kp=1.0):
    return WallFollower(side, reference=0.7, lateral_angle=math.pi / 2,
                        forward_angle=math.pi / 3, window=math.radians(2.0),
                        distance_pi=PI(kp, 0.0, 1.0), orientation_pi=PI(kp, 0.0, 1.0),
                        max_steering=0.4189)


def test_parallel_wall_errors():
    scan = wall_scan(1.0)
    lat = ray_range(scan, ANGLE_MIN, INCREMENT, math.pi / 2, 0.0, RANGE_MAX)
    fwd = ray_range(scan, ANGLE_MIN, INCREMENT, math.pi / 3, 0.0, RANGE_MAX)
    de, oe = wall_errors(lat, fwd, math.pi / 2, math.pi / 3, 0.7)
    assert de == pytest.approx(0.3, abs=1e-6)
    assert oe == pytest.approx(0.0, abs=1e-6)


def test_wall_angle_sign():
    # Wall converging ahead (car heading into it): negative orientation error, steer right.
    scan = wall_scan(0.7, wall_angle=-0.2)
    lat = ray_range(scan, ANGLE_MIN, INCREMENT, math.pi / 2, 0.0, RANGE_MAX)
    fwd = ray_range(scan, ANGLE_MIN, INCREMENT, math.pi / 3, 0.0, RANGE_MAX)
    _, oe = wall_errors(lat, fwd, math.pi / 2, math.pi / 3, 0.7)
    assert oe == pytest.approx(-0.2, abs=1e-6)
    assert follower().steering(scan, ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025) < 0.0


def test_too_far_steers_towards_wall():
    assert follower('left').steering(
        wall_scan(1.0), ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025) > 0.0
    assert follower('right').steering(
        wall_scan(1.0, side='right'), ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025) < 0.0


@pytest.mark.parametrize('distance, angle', [(0.5, 0.1), (0.9, -0.15), (0.7, 0.0)])
def test_left_right_mirror(distance, angle):
    left = follower('left').steering(
        wall_scan(distance, angle, 'left'), ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025)
    right = follower('right').steering(
        wall_scan(distance, angle, 'right'), ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025)
    assert right == pytest.approx(-left, abs=1e-9)


def test_steering_is_clipped():
    assert follower(kp=100.0).steering(
        wall_scan(3.0), ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025) == pytest.approx(0.4189)


def test_invalid_rays_count_as_range_max():
    ranges = [math.nan, math.inf, 0.0, 1.0, 1.0]
    # Window covers all five rays: three invalid (range_max) and two at 1 m -> median range_max.
    assert ray_range(ranges, 0.0, 0.1, 0.2, 0.2, RANGE_MAX) == RANGE_MAX
    # A single noisy ray is filtered out by the median.
    assert ray_range([1.0, 1.0, math.nan, 1.0, 1.0], 0.0, 0.1, 0.2, 0.2, RANGE_MAX) == 1.0


def test_angle_outside_scan_raises():
    with pytest.raises(ValueError):
        ray_range([1.0] * 10, 0.0, 0.1, 2.0, 0.0, RANGE_MAX)


def test_pi_anti_windup_and_invalid_dt():
    pi = PI(kp=0.0, ki=1.0, integral_limit=0.5)
    for _ in range(100):
        pi.update(1.0, 0.1)
    assert pi.integral == pytest.approx(0.5)
    before = pi.integral
    for dt in (0.0, -0.1, math.nan, math.inf):
        pi.update(1.0, dt)
    assert pi.integral == before


def gap_follower(side='left'):
    return WallFollower(side, reference=0.7, lateral_angle=math.pi / 2,
                        forward_angle=math.pi / 3, window=math.radians(2.0),
                        distance_pi=PI(1.0, 1.0, 1.0), orientation_pi=PI(1.0, 1.0, 1.0),
                        max_steering=0.4189, max_wall_distance=1.5, corner_distance=1.5,
                        corner_min_distance=0.7)


def with_rays(ranges, value, lo_deg, hi_deg):
    """Copy of `ranges` with the rays between lo_deg and hi_deg set to `value`."""
    out = list(ranges)
    for i in range(N):
        if lo_deg <= math.degrees(ANGLE_MIN + i * INCREMENT) <= hi_deg:
            out[i] = value
    return out


@pytest.mark.parametrize('side', ['left', 'right'])
def test_side_opening_keeps_straight(side):
    # Wall at 0.7 m with an opening on the wall side; the road ahead is clear.
    sign = 1 if side == 'left' else -1
    lo, hi = sorted((sign * 40.0, sign * 120.0))
    scan = with_rays(wall_scan(0.7, side=side), math.inf, lo, hi)
    f = gap_follower(side)
    assert f.steering(scan, ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025) == 0.0
    assert f.distance_pi.integral == 0.0 and f.orientation_pi.integral == 0.0


@pytest.mark.parametrize('side', ['left', 'right'])
def test_wall_lost_with_front_blocked_turns_towards_wall_side(side):
    sign = 1 if side == 'left' else -1
    lo, hi = sorted((sign * 40.0, sign * 120.0))
    scan = with_rays(with_rays(wall_scan(0.7, side=side), math.inf, lo, hi), 0.6, -10.0, 10.0)
    assert gap_follower(side).steering(
        scan, ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025) == pytest.approx(sign * 0.4189)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_wall_ahead_turns_away_from_wall_side(side):
    # Corner turning away from the followed wall: the wall is there, the front gets blocked.
    sign = 1 if side == 'left' else -1
    straight = wall_scan(0.7, side=side)
    f = gap_follower(side)
    assert f.steering(straight, ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025) == \
        pytest.approx(0.0, abs=1e-3)
    halfway = f.steering(with_rays(straight, 1.1, -10.0, 10.0),
                         ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025)
    assert halfway == pytest.approx(-sign * 0.5 * 0.4189, abs=1e-3)
    assert gap_follower(side).steering(with_rays(straight, 0.5, -10.0, 10.0),
                                       ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025) == \
        pytest.approx(-sign * 0.4189)


def test_opening_ahead_drops_orientation_term():
    # Lateral ray still on the wall at the reference, forward ray into an opening.
    scan = with_rays(wall_scan(0.7), math.inf, 50.0, 70.0)
    assert gap_follower().steering(
        scan, ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025) == pytest.approx(0.0, abs=1e-3)
    # Without gap handling (legacy behaviour) the same scan steers hard into the opening.
    assert follower().steering(scan, ANGLE_MIN, INCREMENT, RANGE_MAX, 0.025) > 0.3
