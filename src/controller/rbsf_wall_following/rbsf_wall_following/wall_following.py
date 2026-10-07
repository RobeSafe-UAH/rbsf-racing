"""
Corridor-centring geometry and PI control (no ROS dependencies).

The track boundary on each side may be a wall or a row of cones with gaps between them, so
nothing is read from single rays (a ray through a gap sees whatever lies behind the row).
The scan becomes points in the car frame (x forward, y left), and every measurement is the
nearest boundary inside a strip of the plane:

    near bin    |x| <= bin_length / 2                      distance to each side's boundary
    far bin     |x - lookahead| <= bin_length / 2          the boundary's orientation
    path        0 < x, |y| <= path_half_width              free distance ahead (front)

A strip longer than the gaps of a cone row always holds at least one cone, so the row reads
like a wall. "Nearest" is the min_points-th smallest value, which ignores one or two stray
returns and needs a few hits before it believes in an obstacle.
"""

import math

import numpy as np


def scan_points(ranges, angle_min, angle_increment, range_max):
    """(x, y) arrays of the valid returns (finite, positive, below range_max) of a scan."""
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(len(r))
    valid = np.isfinite(r) & (r > 0.0) & (r < range_max)
    r, angles = r[valid], angles[valid]
    return r * np.cos(angles), r * np.sin(angles)


def nearest(values, k):
    """k-th smallest of `values` (math.inf if there are fewer than k)."""
    if len(values) < k:
        return math.inf
    return float(np.partition(values, k - 1)[k - 1])


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
    Steering from one scan: keep the car centred between the left and right boundaries.

    A side's boundary is visible while its near-bin distance is within max_wall_distance. Then:
      - Both visible: distance error (d_left - d_right) / 2, and the corridor width
        d_left + d_right is remembered.
      - One visible (doorway, side opening, the inner side of a corner): hold half the
        remembered width to it while that width is younger than width_memory s, else
        fallback_distance. It is the same error as with both, with the lost one assumed where
        it was.
      - None visible: the PIs are skipped (integrals frozen) and the car holds its heading.
    Orientation error: mean over the visible sides of the boundary's angle between the near and
    far bins (a far bin without a boundary within max_wall_distance drops that side's term).
    Steering = distance PI + orientation PI.

    Corners: a front term takes over as the free distance in the car's path drops from
    corner_distance to corner_min_distance (linear blend, full lock at the minimum), towards the
    more open side just before the obstacle (bin x in [front - bin_length, front - front_margin],
    never behind the near bin: the LiDAR barely sees behind the car). Points of the obstacle
    itself (x >= front - front_margin) never count as a side boundary. A boundary closer than
    path_half_width is in the car's path: the car turns away from it at full lock.
    """

    def __init__(self, distance_pi, orientation_pi, max_steering, bin_length=1.0,
                 lookahead=1.2, path_half_width=0.3, front_margin=0.2, min_points=3,
                 max_wall_distance=math.inf, fallback_distance=0.4, width_memory=0.0,
                 corner_distance=0.0, corner_min_distance=0.0):
        if fallback_distance <= 0.0:
            raise ValueError('fallback_distance must be positive')
        if width_memory < 0.0:
            raise ValueError('width_memory must be non-negative')
        if bin_length <= 0.0 or lookahead <= 0.0 or path_half_width <= 0.0:
            raise ValueError('bin_length, lookahead and path_half_width must be positive')
        if min_points < 1:
            raise ValueError('min_points must be at least 1')
        if corner_distance > 0.0 and not 0.0 <= corner_min_distance < corner_distance:
            raise ValueError('corner_min_distance must be in [0, corner_distance)')
        self.distance_pi = distance_pi
        self.orientation_pi = orientation_pi
        self.max_steering = abs(max_steering)
        self.bin_length = bin_length
        self.lookahead = lookahead
        self.path_half_width = path_half_width
        self.front_margin = front_margin
        self.min_points = int(min_points)
        self.max_wall_distance = max_wall_distance
        self.fallback_distance = fallback_distance
        self.width_memory = width_memory
        self.corner_distance = corner_distance
        self.corner_min_distance = corner_min_distance
        self.width = None
        self.width_age = 0.0

    def corner_weight(self, front):
        """0 with the front clear (>= corner_distance), 1 at corner_min_distance or closer."""
        if front >= self.corner_distance:
            return 0.0
        span = self.corner_distance - self.corner_min_distance
        return min(1.0, (self.corner_distance - front) / span)

    def target_distance(self):
        """Distance to hold to a lone boundary: half the remembered width, or the fallback."""
        if self.width is not None and self.width_age <= self.width_memory:
            return 0.5 * self.width
        return self.fallback_distance

    def front_distance(self, x, y):
        """Free distance ahead: nearest x of the points in the car's path."""
        return nearest(x[(x > 0.0) & (np.abs(y) <= self.path_half_width)], self.min_points)

    def boundary(self, x, lateral, x_lo, x_hi):
        """Distance to one side's boundary in the bin x_lo <= x <= x_hi (`lateral` = side * y)."""
        return nearest(lateral[(lateral > 0.0) & (x >= x_lo) & (x <= x_hi)], self.min_points)

    def steering(self, ranges, angle_min, angle_increment, range_max, dt):
        """Steering angle (rad, left positive), clipped to max_steering."""
        x, y = scan_points(ranges, angle_min, angle_increment, range_max)
        front = self.front_distance(x, y)
        cutoff = front - self.front_margin
        half = 0.5 * self.bin_length

        # Per side: near distance, orientation error (left positive), visibility of the near and
        # far bins, and the openness just before the obstacle ahead.
        sides = {}
        for name, sign in (('left', 1.0), ('right', -1.0)):
            lateral = sign * y
            near = self.boundary(x, lateral, -half, max(0.0, min(half, cutoff)))
            far = self.boundary(x, lateral, self.lookahead - half,
                                min(self.lookahead + half, cutoff))
            # The boundary diverging ahead (far > near) means the car heads away from it.
            orientation = sign * math.atan2(far - near, self.lookahead) \
                if math.isfinite(far) and math.isfinite(near) else 0.0
            sides[name] = (near, orientation, near <= self.max_wall_distance,
                           far <= self.max_wall_distance,
                           self.boundary(x, lateral, max(-half, cutoff - self.bin_length),
                                         max(half, cutoff)))
        d_left, oe_left, left, left_oe, open_left = sides['left']
        d_right, oe_right, right, right_oe, open_right = sides['right']

        if left and right:
            distance_error = 0.5 * (d_left - d_right)
            self.width, self.width_age = d_left + d_right, 0.0
        else:
            if math.isfinite(dt) and dt > 0.0:
                self.width_age += dt
            if left:
                distance_error = d_left - self.target_distance()
            elif right:
                distance_error = self.target_distance() - d_right
            else:
                distance_error = None

        follow = 0.0
        if distance_error is not None:
            follow = self.distance_pi.update(distance_error, dt)
            orientations = [oe for oe, ok in ((oe_left, left and left_oe),
                                              (oe_right, right and right_oe)) if ok]
            if orientations:
                follow += self.orientation_pi.update(sum(orientations) / len(orientations), dt)

        corner_turn = self.max_steering if open_left >= open_right else -self.max_steering
        weight = self.corner_weight(front)
        steering = (1.0 - weight) * follow + weight * corner_turn
        return max(-self.max_steering, min(self.max_steering, steering))
