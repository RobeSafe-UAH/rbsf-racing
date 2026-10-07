import math

import numpy as np
import pytest
from rbsf_wall_following.wall_following import nearest, PI, scan_points, WallFollower

ANGLE_MIN = -math.radians(135.0)
INCREMENT = math.radians(0.25)
N = 1081
RANGE_MAX = 30.0
DT = 0.025
MAX_STEERING = 0.4189
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


def boundary(side, distance, angle=0.0, gap=None, phase=0.0, x_range=(-20.0, 20.0)):
    """
    One side's boundary `distance` m from the car (inner surface), rotated `angle` rad.

    side +1 (left) or -1 (right). gap=None: a wall; else a cone row with `gap` m between
    neighbouring cones, shifted `phase` m along the row. Returns (walls, cones) for cast().
    """
    ux, uy = math.cos(angle), math.sin(angle)
    if gap is None:
        (a, b) = x_range
        p0 = (a * ux - side * distance * uy, a * uy + side * distance * ux)
        p1 = (b * ux - side * distance * uy, b * uy + side * distance * ux)
        return [(p0, p1)], []
    offset = side * (distance + CONE_RADIUS)
    spacing = gap + 2 * CONE_RADIUS
    s = np.arange(x_range[0], x_range[1], spacing) + phase
    return [], [(si * ux - offset * uy, si * uy + offset * ux) for si in s]


def corridor(left, right, angle=0.0, gap=None, phase=0.0):
    """Build the walls and cones of a straight corridor; None: no boundary on that side."""
    walls, cones = [], []
    for side, d, ph in ((1, left, phase), (-1, right, phase + 0.17)):
        if d is not None:
            w, c = boundary(side, d, angle, gap, ph)
            walls += w
            cones += c
    return walls, cones


def scan(left, right, angle=0.0, gap=None, phase=0.0):
    return cast(*corridor(left, right, angle, gap, phase))


def follower(kp=1.0, ki=0.0, width_memory=2.0):
    return WallFollower(distance_pi=PI(kp, ki, 1.0), orientation_pi=PI(kp, ki, 1.0),
                        max_steering=MAX_STEERING, max_wall_distance=1.0,
                        fallback_distance=0.4, width_memory=width_memory,
                        corner_distance=1.5, corner_min_distance=0.7)


def steer(f, ranges, dt=DT):
    return f.steering(ranges, ANGLE_MIN, INCREMENT, RANGE_MAX, dt)


# Boundaries: a wall, and cone rows with tight (5 cm) and RoboRacer-style (40 cm, 50 cm) gaps.
GAPS = [None, 0.05, 0.4, 0.5]


def test_scan_points_drop_invalid_returns():
    x, y = scan_points([1.0, math.nan, math.inf, 0.0, -1.0, 40.0], 0.0, 0.1, RANGE_MAX)
    assert len(x) == 1 and x[0] == pytest.approx(1.0) and y[0] == pytest.approx(0.0)


def test_nearest_ignores_stray_returns():
    assert nearest(np.array([0.1, 0.9, 1.0, 1.0, 1.1]), 3) == 1.0
    assert nearest(np.array([0.1, 0.2]), 3) == math.inf


@pytest.mark.parametrize('gap', GAPS)
def test_centred_goes_straight(gap):
    for phase in (0.0, 0.13, 0.31):
        f = follower()
        assert steer(f, scan(0.7, 0.7, gap=gap, phase=phase)) == pytest.approx(0.0, abs=0.02)
        assert f.width == pytest.approx(1.4, abs=0.02)


@pytest.mark.parametrize('gap', GAPS)
def test_off_centre_steers_to_the_centre(gap):
    assert steer(follower(), scan(0.5, 0.9, gap=gap)) < -0.1
    assert steer(follower(), scan(0.9, 0.5, gap=gap)) > 0.1


@pytest.mark.parametrize('gap', GAPS)
def test_corridor_turning_left_steers_left(gap):
    assert steer(follower(), scan(0.7, 0.7, angle=0.1, gap=gap)) > 0.05
    assert steer(follower(), scan(0.7, 0.7, angle=-0.1, gap=gap)) < -0.05


@pytest.mark.parametrize('left, right, angle',
                         [(0.5, 0.9, 0.1), (0.8, 0.6, -0.15), (0.7, None, 0.05),
                          (None, 0.6, 0.0)])
def test_left_right_mirror(left, right, angle):
    a = steer(follower(), scan(left, right, angle))
    b = steer(follower(), scan(right, left, -angle))
    assert b == pytest.approx(-a, abs=1e-6)


@pytest.mark.parametrize('gap', GAPS)
def test_lone_boundary_without_history_holds_fallback(gap):
    assert steer(follower(), scan(None, 0.4, gap=gap)) == pytest.approx(0.0, abs=0.02)
    assert steer(follower(), scan(0.4, None, gap=gap)) == pytest.approx(0.0, abs=0.02)


def test_lone_wall_too_far_without_history_steers_towards_it():
    assert steer(follower(), scan(None, 0.7)) < 0.0
    assert steer(follower(), scan(0.7, None)) > 0.0


def test_lone_wall_with_history_holds_half_the_width():
    f = follower(width_memory=2.0)
    steer(f, scan(0.7, 0.7))
    # The right wall disappears: the car keeps 0.7 m to the left one.
    lone = scan(0.7, None)
    for _ in range(int(2.0 / DT) - 1):
        assert steer(f, lone) == pytest.approx(0.0, abs=1e-3)
    # The remembered width expires: back to 0.4 m, steer towards the left wall.
    for _ in range(3):
        steering = steer(f, lone)
    assert steering > 0.0


def test_history_refreshes_with_both_walls():
    f = follower(width_memory=0.1)
    for _ in range(10):
        steer(f, scan(0.7, None))
    steer(f, scan(0.6, 0.6))
    assert f.width == pytest.approx(1.2, abs=1e-3) and f.width_age == 0.0
    assert steer(f, scan(0.6, None)) == pytest.approx(0.0, abs=1e-3)


def test_no_walls_holds_heading_and_freezes_integrals():
    f = follower(ki=1.0)
    for _ in range(10):
        assert steer(f, scan(None, None)) == 0.0
    assert f.distance_pi.integral == 0.0 and f.orientation_pi.integral == 0.0


def test_opening_ahead_drops_that_sides_orientation():
    # Left wall only beside the car (it ends at x = 0.5), right wall all along.
    walls = boundary(1, 0.7, x_range=(-20.0, 0.5))[0] + boundary(-1, 0.7)[0]
    assert steer(follower(), cast(walls)) == pytest.approx(0.0, abs=1e-3)


def cross_wall(x, half_width=3.0):
    return [((x, -half_width), (x, half_width))]


@pytest.mark.parametrize('side', [1, -1])
def test_front_blocked_turns_towards_the_open_side(side):
    # Corner turning towards `side`: only the opposite boundary, a wall ahead.
    walls = boundary(-side, 0.4)[0]
    assert steer(follower(), cast(walls + cross_wall(0.6))) == \
        pytest.approx(side * MAX_STEERING)
    # Halfway through the corner band: half lock.
    assert steer(follower(), cast(walls + cross_wall(1.1))) == \
        pytest.approx(side * 0.5 * MAX_STEERING, abs=1e-3)


@pytest.mark.parametrize('gap', [0.05, 0.4, 0.5])
@pytest.mark.parametrize('phase', [0.0, 0.2])
def test_cone_row_ahead_is_an_obstacle_even_through_a_gap(gap, phase):
    # Levine-like corner: the left boundary ends beside the car, the right one continues, and a
    # cone row crosses the path. phase 0 centres a gap on the heading, so the rays straight
    # ahead see nothing.
    spacing = gap + 2 * CONE_RADIUS
    row_x = 0.6 + CONE_RADIUS
    cones = [(row_x, y) for y in np.arange(-3.0, 3.0, spacing) + spacing / 2 + phase]
    walls = boundary(1, 0.7, x_range=(-20.0, -0.7))[0] + boundary(-1, 0.7)[0]
    f = follower()
    assert f.front_distance(*scan_points(cast(walls, cones), ANGLE_MIN, INCREMENT, RANGE_MAX)) \
        == pytest.approx(0.6, abs=0.06)
    assert steer(f, cast(walls, cones)) == pytest.approx(MAX_STEERING)


def test_steering_is_clipped():
    assert steer(follower(kp=100.0), scan(0.2, 0.9)) == pytest.approx(-MAX_STEERING)


def test_invalid_parameters_raise():
    pi = PI(1.0, 0.0, 1.0)
    for kwargs in ({'fallback_distance': 0.0}, {'width_memory': -1.0}, {'bin_length': 0.0},
                   {'min_points': 0}, {'corner_distance': 1.0, 'corner_min_distance': 1.0}):
        with pytest.raises(ValueError):
            WallFollower(pi, pi, 0.4, **kwargs)


def test_pi_anti_windup_and_invalid_dt():
    pi = PI(kp=0.0, ki=1.0, integral_limit=0.5)
    for _ in range(100):
        pi.update(1.0, 0.1)
    assert pi.integral == pytest.approx(0.5)
    before = pi.integral
    for dt in (0.0, -0.1, math.nan, math.inf):
        pi.update(1.0, dt)
    assert pi.integral == before
