"""
Wall-following geometry and PI control (no ROS dependencies).

The car keeps a reference distance to one wall using two LiDAR rays on that side: a lateral ray
(90 deg) and a forward one (60 deg). With both hits as points in the car frame (x forward,
y left), the lateral one gives the distance to the wall and the line through both gives the wall
orientation relative to the car's heading.

Everything is written for a wall on the left. A wall on the right is handled by mirroring: read
the rays at the negated angles, run the same maths, and negate the steering.
"""

import math


def ray_range(ranges, angle_min, angle_increment, angle, window, range_max):
    """
    Median range of the rays within `window` rad of `angle` (rad, car frame).

    Readings that are not finite or not positive (no return, invalid) count as `range_max`: there
    is no wall closer than that along the ray. Raises ValueError if `angle` is outside the scan.
    """
    n = len(ranges)
    centre = round((angle - angle_min) / angle_increment)
    if not 0 <= centre < n:
        raise ValueError(
            f'angle {math.degrees(angle):.1f} deg is outside the scan '
            f'[{math.degrees(angle_min):.1f}, '
            f'{math.degrees(angle_min + (n - 1) * angle_increment):.1f}] deg')
    half = int(abs(window) / angle_increment)
    rays = [r if math.isfinite(r) and r > 0.0 else range_max
            for r in ranges[max(0, centre - half):min(n, centre + half + 1)]]
    rays.sort()
    mid = len(rays) // 2
    return rays[mid] if len(rays) % 2 else 0.5 * (rays[mid - 1] + rays[mid])


def wall_errors(lateral_range, forward_range, lateral_angle, forward_angle, reference):
    """
    Distance and orientation errors to a wall on the left.

    Returns (distance_error, orientation_error): the lateral distance minus `reference` (positive
    when too far from the wall) and the wall's angle relative to the heading (positive when the
    wall diverges ahead, i.e. the car heads away from it). Both call for steering left (positive)
    when positive.
    """
    x_lat = lateral_range * math.cos(lateral_angle)
    y_lat = lateral_range * math.sin(lateral_angle)
    x_fwd = forward_range * math.cos(forward_angle)
    y_fwd = forward_range * math.sin(forward_angle)
    return abs(y_lat) - reference, math.atan2(y_fwd - y_lat, x_fwd - x_lat)


class PI:
    """PI controller with a clamped integral (anti-windup)."""

    def __init__(self, kp, ki, integral_limit):
        self.kp = kp
        self.ki = ki
        self.integral_limit = abs(integral_limit)
        self.integral = 0.0

    def update(self, error, dt):
        """Return kp * e + ki * integral(e). A non-finite or non-positive `dt` is skipped."""
        if math.isfinite(dt) and dt > 0.0:
            self.integral = min(self.integral_limit,
                                max(-self.integral_limit, self.integral + error * dt))
        return self.kp * error + self.ki * self.integral

    def reset(self):
        self.integral = 0.0


class WallFollower:
    """
    Steering from one scan: distance PI + orientation PI, mirrored for a right wall.

    Two situations the plain PI (legacy) does not handle, both common in corridors and tracks:
      - Gaps in the wall (doorways, side openings): the far reading looks like a wall that is too
        far away, so the PI steers into them. A wall-side hit farther than max_wall_distance
        means the wall is lost: the PI is skipped (integrals frozen) and the car holds its
        heading. A forward hit that far only drops the orientation term.
      - Corners: the side rays see a wall ahead too late to turn. A front term takes over as the
        front range drops from corner_distance to corner_min_distance (linear blend, full lock at
        the minimum): away from the wall side while the wall is there (corner turning away from
        it), towards it when the wall is lost (corner turning towards it).
    """

    def __init__(self, wall_side, reference, lateral_angle, forward_angle, window,
                 distance_pi, orientation_pi, max_steering, max_wall_distance=math.inf,
                 corner_distance=0.0, corner_min_distance=0.0):
        if wall_side not in ('left', 'right'):
            raise ValueError(f"wall_side must be 'left' or 'right', not '{wall_side}'")
        if corner_distance > 0.0 and not 0.0 <= corner_min_distance < corner_distance:
            raise ValueError('corner_min_distance must be in [0, corner_distance)')
        self.sign = 1.0 if wall_side == 'left' else -1.0
        self.reference = reference
        self.lateral_angle = lateral_angle
        self.forward_angle = forward_angle
        self.window = window
        self.distance_pi = distance_pi
        self.orientation_pi = orientation_pi
        self.max_steering = abs(max_steering)
        self.max_wall_distance = max_wall_distance
        self.corner_distance = corner_distance
        self.corner_min_distance = corner_min_distance

    def corner_weight(self, front):
        """0 with the front clear (>= corner_distance), 1 at corner_min_distance or closer."""
        if front >= self.corner_distance:
            return 0.0
        span = self.corner_distance - self.corner_min_distance
        return min(1.0, (self.corner_distance - front) / span)

    def steering(self, ranges, angle_min, angle_increment, range_max, dt):
        """Steering angle (rad, left positive), clipped to max_steering."""
        def read(angle):
            return ray_range(ranges, angle_min, angle_increment, self.sign * angle,
                             self.window, range_max)

        # Everything below is for a wall on the left (positive = towards the wall); the sign
        # mirrors it for a right wall.
        lateral, forward = read(self.lateral_angle), read(self.forward_angle)
        wall_lost = lateral * math.sin(self.lateral_angle) > self.max_wall_distance
        if wall_lost:
            follow, corner_turn = 0.0, self.max_steering
        else:
            distance_error, orientation_error = wall_errors(
                lateral, forward, self.lateral_angle, self.forward_angle, self.reference)
            follow = self.distance_pi.update(distance_error, dt)
            if forward * math.sin(self.forward_angle) <= self.max_wall_distance:
                follow += self.orientation_pi.update(orientation_error, dt)
            corner_turn = -self.max_steering
        weight = self.corner_weight(read(0.0))
        steering = self.sign * ((1.0 - weight) * follow + weight * corner_turn)
        return max(-self.max_steering, min(self.max_steering, steering))
