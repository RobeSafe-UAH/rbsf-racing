"""
Path tracking geometry on a closed waypoint path (no ROS dependencies).

Poses refer to the rear axle (base_link), where the bicycle model is defined. Both steering laws
aim at a lookahead point at distance L and lateral offset y in the car frame:

- Pure Pursuit steers along the arc through it: curvature k = 2 y / L^2, delta = atan(wheelbase k).
- MAP (Model- and Acceleration-based Pursuit, Becker et al., ForzaETH 2023) asks for the lateral
  acceleration of that arc at the measured speed v, a = 2 v^2 y / L^2, and turns it into a
  steering angle with the steady-state single-track model: delta = atan(wheelbase k) +
  understeer_gradient * a. With understeer_gradient = 0 it is Pure Pursuit; a positive gradient
  steers more as the tyres need a larger slip angle. Ported from the linear-understeer variant
  in RobeSafe-UAH/tfg-ruben-lopez (`map_controller.py`), with the kinematic term computed from
  the curvature so the car still steers at low speed.
"""

import math

import numpy as np

STEERING_LAWS = ('pure_pursuit', 'map')


def nearest_index(xy, position):
    """Index of the waypoint closest to `position`."""
    return int(np.argmin(np.linalg.norm(xy - position, axis=1)))


def lookahead_index(xy, start, position, distance):
    """First waypoint after `start` (wrapping around) at least `distance` away from `position`."""
    order = (start + np.arange(len(xy))) % len(xy)
    far = np.nonzero(np.linalg.norm(xy[order] - position, axis=1) >= distance)[0]
    return int(order[far[0]]) if len(far) else int(order[-1])


def lookahead_distance(speed, minimum, gain, maximum):
    """Lookahead L = minimum + gain * speed, at most `maximum`."""
    return min(minimum + gain * max(speed, 0.0), maximum)


def arc_curvature(x, y, yaw, target):
    """Curvature (1/m, left positive) of the arc from pose (x, y, yaw) through `target`."""
    dx, dy = target[0] - x, target[1] - y
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    distance_sq = dx * dx + dy * dy
    if distance_sq < 1e-9:
        return 0.0
    return 2.0 * lateral / distance_sq


def pure_pursuit_steering(curvature, wheelbase):
    """Kinematic steering angle (rad) for `curvature`."""
    return math.atan(wheelbase * curvature)


def map_steering(curvature, speed, wheelbase, understeer_gradient):
    """MAP steering angle (rad): kinematic term plus understeer for a = speed^2 * curvature."""
    lateral_acceleration = max(speed, 0.0) ** 2 * curvature
    return pure_pursuit_steering(curvature, wheelbase) + understeer_gradient * lateral_acceleration


def steering_angle(law, x, y, yaw, target, speed, wheelbase, max_steering,
                   understeer_gradient=0.0):
    """Steering angle (rad, left positive, clipped to max_steering) of `law` towards `target`."""
    curvature = arc_curvature(x, y, yaw, target)
    if law == 'pure_pursuit':
        angle = pure_pursuit_steering(curvature, wheelbase)
    elif law == 'map':
        angle = map_steering(curvature, speed, wheelbase, understeer_gradient)
    else:
        raise ValueError(
            f"Unknown steering law '{law}'. Valid options: {', '.join(STEERING_LAWS)}")
    return max(-max_steering, min(max_steering, angle))
