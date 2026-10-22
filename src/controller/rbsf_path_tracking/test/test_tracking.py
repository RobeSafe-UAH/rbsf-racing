import math

import numpy as np
import pytest
from rbsf_path_tracking.tracking import (
    arc_curvature, lookahead_distance, lookahead_index, map_steering, nearest_index,
    speed_colors, steering_angle)


def circle(radius=5.0, n=200):
    t = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return np.stack([radius * np.cos(t), radius * np.sin(t)], axis=1)


def test_lookahead_walks_forward_and_wraps():
    xy = circle()
    start = len(xy) - 2
    target = lookahead_index(xy, start, xy[start], 1.0)
    assert target < 10  # wrapped past the end
    assert np.linalg.norm(xy[target] - xy[start]) >= 1.0
    assert nearest_index(xy, xy[42] + 0.01) == 42


def test_lookahead_distance_grows_with_speed_up_to_max():
    assert lookahead_distance(0.0, 0.5, 0.2, 3.0) == pytest.approx(0.5)
    assert lookahead_distance(5.0, 0.5, 0.2, 3.0) == pytest.approx(1.5)
    assert lookahead_distance(50.0, 0.5, 0.2, 3.0) == pytest.approx(3.0)
    assert lookahead_distance(-1.0, 0.5, 0.2, 3.0) == pytest.approx(0.5)


@pytest.mark.parametrize('law', ['pure_pursuit', 'map'])
def test_steering_matches_circle_curvature(law):
    # Driving counter-clockwise on a circle, any point ahead on it has curvature 1 / R, which
    # needs atan(wheelbase / R) kinematically (MAP with no understeer is the same).
    radius, wheelbase = 5.0, 0.33
    xy = circle(radius)
    x, y = xy[0]
    assert arc_curvature(x, y, math.pi / 2, xy[20]) == pytest.approx(1.0 / radius, rel=1e-6)
    angle = steering_angle(law, x, y, math.pi / 2, xy[20], 3.0, wheelbase, max_steering=1.0)
    assert angle == pytest.approx(math.atan(wheelbase / radius), rel=1e-6)
    # Mirror target to the right: negative.
    assert steering_angle(law, 0.0, 0.0, 0.0, (1.0, -1.0), 3.0, wheelbase, 1.0) < 0.0
    # Clipped.
    assert steering_angle(law, 0.0, 0.0, 0.0, (0.0, 0.1), 3.0, wheelbase, 0.4) == \
        pytest.approx(0.4)


def test_map_adds_understeer_proportional_to_lateral_acceleration():
    wheelbase, gradient, curvature = 0.33, 0.01, 0.2
    kinematic = math.atan(wheelbase * curvature)
    for speed in (1.0, 3.0, 6.0):
        expected = kinematic + gradient * speed ** 2 * curvature
        assert map_steering(curvature, speed, wheelbase, gradient) == pytest.approx(expected)
    # Standing still it still steers like Pure Pursuit (no lateral acceleration to correct).
    assert map_steering(curvature, 0.0, wheelbase, gradient) == pytest.approx(kinematic)
    # Symmetric left/right.
    assert map_steering(-curvature, 3.0, wheelbase, gradient) == \
        pytest.approx(-map_steering(curvature, 3.0, wheelbase, gradient))


def test_unknown_law_is_rejected():
    with pytest.raises(ValueError, match='Valid options'):
        steering_angle('stanley', 0.0, 0.0, 0.0, (1.0, 0.0), 1.0, 0.33, 0.4)


def test_speed_colors_scale_from_red_to_green():
    colors = speed_colors([0.5, 1.0, 2.0, 3.0, 4.0], slow=1.0, fast=3.0)
    np.testing.assert_allclose(colors, [[1, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 1, 0]])
