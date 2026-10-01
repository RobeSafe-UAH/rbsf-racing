import math

import numpy as np
import pytest
from rbsf_pure_pursuit.pure_pursuit import lookahead_index, nearest_index, steering_angle


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


def test_steering_matches_circle_curvature():
    # Driving counter-clockwise on a circle, any point ahead on it needs atan(wheelbase / R).
    radius, wheelbase = 5.0, 0.33
    xy = circle(radius)
    x, y = xy[0]
    angle = steering_angle(x, y, math.pi / 2, xy[20], wheelbase, max_steering=1.0)
    assert angle == pytest.approx(math.atan(wheelbase / radius), rel=1e-6)
    # Mirror target to the right: same magnitude, negative.
    assert steering_angle(0.0, 0.0, 0.0, (1.0, -1.0), wheelbase, 1.0) < 0.0
    # Clipped.
    assert steering_angle(0.0, 0.0, 0.0, (0.0, 0.1), wheelbase, 0.4) == pytest.approx(0.4)
