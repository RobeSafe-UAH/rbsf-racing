import math

import numpy as np
import pytest
from rbsf_path_tracking.speed_profile import curvature, speed_colors, speed_profile


def circle(radius, n=200):
    angles = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return radius * np.column_stack((np.cos(angles), np.sin(angles)))


def stadium(radius=2.0, straight=20.0, step=0.05):
    """Return a closed counter-clockwise track: two straights joined by half circles."""
    line = np.arange(0.0, straight, step)
    arc = np.arange(0.0, math.pi, step / radius)
    bottom = np.column_stack((line, np.full_like(line, -radius)))
    right = np.column_stack((straight + radius * np.sin(arc), -radius * np.cos(arc)))
    top = np.column_stack((straight - line, np.full_like(line, radius)))
    left = np.column_stack((-radius * np.sin(arc), radius * np.cos(arc)))
    return np.vstack((bottom, right, top, left))


def profile(xy, **overrides):
    kwargs = {'max_speed': 6.0, 'min_speed': 0.5, 'max_lateral_acceleration': 4.0,
              'max_acceleration': 3.0, 'max_deceleration': 5.0, 'curvature_spacing': 0.5}
    kwargs.update(overrides)
    return speed_profile(xy, **kwargs)


def test_curvature_of_a_circle_is_one_over_its_radius():
    assert curvature(circle(2.0), 0.5) == pytest.approx(0.5, rel=1e-3)


def test_curvature_of_a_straight_is_zero():
    track = stadium()
    assert curvature(track, 0.5)[100:300] == pytest.approx(0.0, abs=1e-9)


def test_constant_corner_speed_on_a_circle():
    # v = sqrt(a / k) = sqrt(4 * 2) on a 2 m circle.
    assert profile(circle(2.0)) == pytest.approx(math.sqrt(8.0), rel=1e-3)


def test_top_speed_on_straights_and_corner_speed_in_corners():
    track = stadium()
    v = profile(track)
    k = curvature(track, 0.5)
    middle_of_straight = len(np.arange(0.0, 20.0, 0.05)) // 2
    assert v[middle_of_straight] == pytest.approx(6.0)
    assert v[k > 0.4999].max() <= math.sqrt(8.0) + 1e-6


def test_acceleration_and_braking_limits_hold_across_the_wrap():
    track = stadium()
    v = profile(track)
    d = np.linalg.norm(np.roll(track, -1, axis=0) - track, axis=1)
    following = np.roll(v, -1)
    assert np.all(following ** 2 <= v ** 2 + 2.0 * 3.0 * d + 1e-9)
    assert np.all(v ** 2 <= following ** 2 + 2.0 * 5.0 * d + 1e-9)


def test_braking_starts_before_the_corner():
    track = stadium()
    v = profile(track)
    corner_start = len(np.arange(0.0, 20.0, 0.05))
    # (36 - 8) / (2 * 5) = 2.8 m of braking from 6 m/s down to the corner speed.
    assert v[corner_start - 20] < 6.0  # 1 m before the corner
    assert v[corner_start - 60] == pytest.approx(6.0)  # 3 m before


def test_planner_target_speeds_cap_the_profile_and_nan_means_none():
    xy = circle(2.0)
    targets = np.full(len(xy), np.nan)
    assert profile(xy, target_speeds=targets) == pytest.approx(math.sqrt(8.0), rel=1e-3)
    targets[:] = 1.5
    assert profile(xy, target_speeds=targets) == pytest.approx(1.5)


def test_min_speed_floors_tight_corners():
    # A 0.1 m circle needs sqrt(0.4) = 0.63 m/s; the floor lifts it.
    assert profile(circle(0.1), min_speed=0.8) == pytest.approx(0.8)


def test_max_speed_wins_over_a_higher_min_speed():
    assert profile(circle(0.1), min_speed=0.8, max_speed=0.5) == pytest.approx(0.5)


def test_speed_colors_scale_from_red_to_green():
    colors = speed_colors([0.5, 1.0, 2.0, 3.0, 4.0], slow=1.0, fast=3.0)
    np.testing.assert_allclose(colors, [[1, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 1, 0]])
