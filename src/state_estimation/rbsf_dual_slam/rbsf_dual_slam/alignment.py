"""Least-squares 2D rigid alignment of two trajectories (no ROS imports, unit-tested)."""

import math

import numpy as np


def fit_rigid_2d(source, target):
    """
    Rotation + translation (x, y, yaw) that best maps `source` points onto `target` (Kabsch).

    Both are (N, 2) arrays of corresponding points, N >= 2 and not all coincident.
    """
    source, target = np.asarray(source, float), np.asarray(target, float)
    src_mean, tgt_mean = source.mean(axis=0), target.mean(axis=0)
    cov = (source - src_mean).T @ (target - tgt_mean)
    yaw = math.atan2(cov[0, 1] - cov[1, 0], cov[0, 0] + cov[1, 1])
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = tgt_mean - np.array([[c, -s], [s, c]]) @ src_mean
    return float(x), float(y), yaw


def residual_rms(source, target, fit):
    """RMS distance (m) between `target` and `source` transformed by `fit`."""
    x, y, yaw = fit
    c, s = math.cos(yaw), math.sin(yaw)
    moved = np.asarray(source, float) @ np.array([[c, s], [-s, c]]) + (x, y)
    return float(np.sqrt(np.mean(np.sum((moved - np.asarray(target, float)) ** 2, axis=1))))
