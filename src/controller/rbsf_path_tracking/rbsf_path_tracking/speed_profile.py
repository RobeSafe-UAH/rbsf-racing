"""
Curvature-based speed profile on a closed waypoint path (no ROS dependencies).

The speed at each waypoint is the fastest the car can take it, under three limits:

1. Lateral grip: v = sqrt(max_lateral_acceleration / |k|), with k the path curvature there.
   Curvature comes from the circle through the points `curvature_spacing` metres before and
   after the waypoint (arc length, wrapping around), which smooths the noise of closely spaced
   waypoints. The planner's target speed (NaN = none) and [min_speed, max_speed] clip it.
2. Acceleration: a forward pass, v[i+1] <= sqrt(v[i]^2 + 2 max_acceleration d[i]).
3. Braking: a backward pass, v[i] <= sqrt(v[i+1]^2 + 2 max_deceleration d[i]), so the car slows
   down before a corner instead of in it.

The path is closed, so each pass runs twice round the loop: the second lap carries the limits
across the wrap from the last waypoint to the first.
"""

import math

import numpy as np


def segment_lengths(xy):
    """Distance from each waypoint to the next one, wrapping around."""
    return np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)


def points_at(xy, s):
    """Points at arc lengths `s` (any real values, taken modulo the length) on the closed path."""
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths(xy))))
    loop = np.vstack((xy, xy[:1]))
    s = np.mod(s, cumulative[-1])
    return np.column_stack((np.interp(s, cumulative, loop[:, 0]),
                            np.interp(s, cumulative, loop[:, 1])))


def curvature(xy, spacing):
    """Return |k| (1/m) at each waypoint, from the points `spacing` m before and after it."""
    s = np.concatenate(([0.0], np.cumsum(segment_lengths(xy)[:-1])))
    a, b, c = points_at(xy, s - spacing), xy, points_at(xy, s + spacing)
    ab, bc, ac = b - a, c - b, c - a
    cross = ab[:, 0] * ac[:, 1] - ab[:, 1] * ac[:, 0]
    sides = np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1) * np.linalg.norm(ac, axis=1)
    return np.divide(2.0 * np.abs(cross), sides, out=np.zeros(len(xy)), where=sides > 1e-12)


def speed_profile(xy, max_speed, min_speed, max_lateral_acceleration, max_acceleration,
                  max_deceleration, curvature_spacing, target_speeds=None):
    """Return the speed (m/s) at each waypoint of the closed path `xy` (module docstring)."""
    k = curvature(xy, curvature_spacing)
    with np.errstate(divide='ignore'):
        v = np.sqrt(max_lateral_acceleration / k)
    if target_speeds is not None:
        v = np.fmin(v, target_speeds)  # fmin ignores NaN targets
    v = np.clip(v, min(min_speed, max_speed), max_speed)

    d = segment_lengths(xy)
    n = len(v)
    for step in range(2 * n):
        i, j = step % n, (step + 1) % n
        v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2.0 * max_acceleration * d[i]))
    for step in range(2 * n, 0, -1):
        i, j = step % n, (step + 1) % n
        v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2.0 * max_deceleration * d[i]))
    return v


def speed_colors(speeds, slow, fast):
    """Return an RGB row in [0, 1] per speed: red <= `slow`, yellow halfway, green >= `fast`."""
    t = np.clip((np.asarray(speeds, dtype=float) - slow) / (fast - slow), 0.0, 1.0)
    return np.column_stack((np.minimum(1.0, 2.0 * (1.0 - t)), np.minimum(1.0, 2.0 * t),
                            np.zeros_like(t)))
