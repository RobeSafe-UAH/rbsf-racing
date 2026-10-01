"""
Closed reference paths (centerline/raceline) of a track: loading and arc-length geometry.

No ROS dependencies, so controllers, simulators and offline tools can all use it.
"""

import math
import re

import numpy as np


def load_waypoints(path):
    """
    Load a closed path from a gym centerline or raceline CSV.

    Centerline: `x_m, y_m, w_tr_right_m, w_tr_left_m`. Raceline:
    `s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`. Lines starting with '#' are skipped.
    Returns (xy, speeds): an (N, 2) array and an (N,) array, or None without a speed profile.
    """
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                rows.append([float(v) for v in re.split(r'[;,]', line) if v.strip()])
    data = np.array(rows)
    if data.ndim != 2 or len(data) < 3:
        raise ValueError(f'{path}: expected at least 3 waypoints')
    if data.shape[1] == 7:
        return data[:, 1:3].copy(), data[:, 5].copy()
    return data[:, 0:2].copy(), None


def arc_lengths(xy):
    """Cumulative distance of each waypoint from the first, and the closed path length."""
    segments = np.linalg.norm(np.diff(xy, axis=0, append=xy[:1]), axis=1)
    return np.concatenate(([0.0], np.cumsum(segments[:-1]))), float(segments.sum())


def pose_at(xy, s):
    """Interpolate (x, y, yaw) at distance `s` along the closed path."""
    cumulative, length = arc_lengths(xy)
    s %= length
    i = int(np.searchsorted(cumulative, s, side='right') - 1)
    p0, p1 = xy[i], xy[(i + 1) % len(xy)]
    segment = np.linalg.norm(p1 - p0)
    t = (s - cumulative[i]) / segment if segment > 0 else 0.0
    x, y = p0 + t * (p1 - p0)
    return float(x), float(y), math.atan2(p1[1] - p0[1], p1[0] - p0[0])


def spawn_poses(xy, num_cars, spacing):
    """
    Start poses (x, y, yaw) for `num_cars` cars `spacing` metres apart along the path.

    The path is driven towards increasing arc length: car k starts at s = k * spacing, so car 0
    (the ego) is last and every other car is k gaps ahead of it.
    """
    _, length = arc_lengths(xy)
    if num_cars * spacing > length:
        raise ValueError(f'{num_cars} cars {spacing} m apart do not fit on a {length:.1f} m path.')
    return [pose_at(xy, k * spacing) for k in range(num_cars)]
