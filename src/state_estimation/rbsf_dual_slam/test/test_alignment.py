import math

import numpy as np
import pytest
from rbsf_dual_slam.alignment import fit_rigid_2d, residual_rms


def transform(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + (x, y)


def test_recovers_a_known_transform():
    rng = np.random.default_rng(0)
    source = rng.uniform(-20, 20, size=(50, 2))
    target = transform(source, -1.5, 3.0, -2.88)
    fit = fit_rigid_2d(source, target)
    assert fit == pytest.approx((-1.5, 3.0, -2.88))
    assert residual_rms(source, target, fit) == pytest.approx(0.0, abs=1e-9)


def test_noisy_points_give_a_least_squares_fit():
    rng = np.random.default_rng(1)
    source = rng.uniform(-20, 20, size=(200, 2))
    target = transform(source, 2.0, -1.0, 0.3) + rng.normal(0, 0.05, size=source.shape)
    x, y, yaw = fit_rigid_2d(source, target)
    assert (x, y, yaw) == pytest.approx((2.0, -1.0, 0.3), abs=0.02)
