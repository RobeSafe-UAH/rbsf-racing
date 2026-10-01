"""
Pure Pursuit geometry on a closed waypoint path (no ROS dependencies).

Waypoint loading and arc-length helpers live in rbsf_worlds.paths.

Poses refer to the rear axle (base_link), where the Pure Pursuit bicycle model is defined:
for a target point at distance L and lateral offset y in the car frame, the arc through it has
curvature k = 2 y / L^2 and the steering angle is atan(wheelbase * k).
"""

import math

import numpy as np


def nearest_index(xy, position):
    """Index of the waypoint closest to `position`."""
    return int(np.argmin(np.linalg.norm(xy - position, axis=1)))


def lookahead_index(xy, start, position, distance):
    """First waypoint after `start` (wrapping around) at least `distance` away from `position`."""
    order = (start + np.arange(len(xy))) % len(xy)
    far = np.nonzero(np.linalg.norm(xy[order] - position, axis=1) >= distance)[0]
    return int(order[far[0]]) if len(far) else int(order[-1])


def steering_angle(x, y, yaw, target, wheelbase, max_steering):
    """Pure Pursuit steering angle (rad, left positive) to reach `target` from pose (x, y, yaw)."""
    dx, dy = target[0] - x, target[1] - y
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    distance_sq = dx * dx + dy * dy
    if distance_sq < 1e-9:
        return 0.0
    angle = math.atan(wheelbase * 2.0 * lateral / distance_sq)
    return max(-max_steering, min(max_steering, angle))
