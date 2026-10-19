"""
Follow-the-Gap on a LiDAR scan (no ROS dependencies).

Adapted from the ForzaETH race stack (controller/ftg/ftg.py), which uses it to map an unknown
track. Every step is vectorized: the only Python loop runs over the log2(n) levels of the
inflation's sparse table, never over beams, and everything that depends only on the scan layout
is computed once (Layout).

    1. Clean: the beams inside the field of view, plus INFLATION_MARGIN on each side (obstacles
       just outside it still block its edge beams); no return (inf, NaN, 0, >= range_max) is
       open space at max_range, and every range is clipped to max_range (the planning horizon).
    2. Smooth: moving mean over smoothing_window beams, never above the raw range (a lone ray
       through a gap is shortened, an obstacle never moves away).
    3. Inflate: a point at range r blocks every beam within asin(inflation_radius / r) of it
       (the disc around it), so a gap must fit the car, and the gaps of a cone row (narrower
       than 2 inflation_radius) close. See Inflation. A point closer than the radius would
       block 90 degrees on each side, the beams pointing away from it too, so the radius
       shrinks to NEAR_FACTOR times the nearest range: next to a wall, the car can still
       steer away from it.
    4. Gap: the beams of the field of view whose inflated range reaches the gap threshold
       (base_gap_threshold + speed_gain * last speed, at most max_gap_threshold) are free. The
       threshold must exceed half the track width, or the walls beside the car count as free
       and the gap spans the whole field of view. The largest run of free beams is the gap
       (ties: the one nearest the heading); the run holding the last target counts
       gap_hysteresis times its length, so the car does not switch sides in front of a
       centred obstacle. Without a gap at that threshold, the base threshold is tried, then
       fallback_ratio times the deepest inflated range (at least min_gap_threshold) among the
       beams the car can turn to and stop before (range >= 2 turning radius |sin angle| +
       stop_distance): in a tight bend, or stopped in front of an obstacle, the car takes the
       deepest of them instead of stopping for good. Without a gap there either, it stops.
    5. Target: the beam at target_fraction of the gap, from its right edge (0) to its left edge
       (1), 0.5 being the middle (or, with target_mode 'deepest', the farthest beam within
       deepest_fraction of the gap around it), at distance min(range, lookahead). Above 0.5 the
       car keeps to the inner side of a counter-clockwise lap, away from dead-end bays on the
       outside of the bends.
    6. Steering: pure pursuit to the target, atan(2 wheelbase sin(alpha) / distance), low-passed
       with steering_alpha and clipped to max_steering.
    7. Speed: linear from max_speed (straight) to min_speed (full lock), capped so the car can
       stop before the target, sqrt(2 max_decel (distance - stop_distance)), and never below
       min_speed while moving.
"""

from collections import namedtuple
import math

import numpy as np
from scipy.ndimage import uniform_filter1d

# Beams this far (rad) outside the field of view are inflated too, so that obstacles just beyond
# its edges (the next cone of a row) still block the edge beams. Narrow enough to leave out the
# rear beams that may hit the car itself.
INFLATION_MARGIN = math.pi / 6
# A point closer than the inflation radius shrinks it to this fraction of its range.
NEAR_FACTOR = 0.9

Command = namedtuple('Command', [
    'speed', 'steering',
    'target_angle', 'target_distance',   # None when no gap was found
    'gap_start_angle', 'gap_end_angle',  # None when no gap was found
    'threshold',
])


class Layout:
    """Everything that depends only on the scan geometry, the field of view and the radius."""

    def __init__(self, size, angle_min, angle_increment, fov, inflation_radius):
        if size < 1 or angle_increment == 0.0:
            raise ValueError('The scan needs beams and a non-zero angle_increment')
        angles = angle_min + angle_increment * np.arange(size)
        inside = np.flatnonzero(np.abs(angles) <= 0.5 * fov)
        if inside.size == 0:
            raise ValueError('No beam inside the field of view')
        processed = np.flatnonzero(np.abs(angles) <= 0.5 * fov + INFLATION_MARGIN)
        self.key = (size, angle_min, angle_increment)
        # Beams cleaned and inflated (scan indices), and the field of view within them.
        self.start, self.stop = int(processed[0]), int(processed[-1]) + 1
        self.fov_start = int(inside[0]) - self.start
        self.fov_stop = int(inside[-1]) + 1 - self.start
        self.angles = angles[inside[0]:inside[-1] + 1]
        self.heading = int(np.argmin(np.abs(self.angles)))
        self.inflate = Inflation(self.stop - self.start, abs(angle_increment), inflation_radius)


class Inflation:
    """
    Inflate every obstacle over the beams the car would hit (exact, no loop over beams).

    A disc of the inflation radius around a point at range r covers asin(radius / r) on each
    side: k = ceil(asin(radius / r) / increment) beams, or 90 degrees once r <= radius. Each
    point therefore sets min(range) over the interval [i - k, i + k] of beams. All the intervals
    are applied at once with a sparse table (offline range-min updates): an interval of length
    L is covered by the two blocks of length 2^p (p = floor(log2 L)) at its ends, scattered with
    np.minimum.at, and the levels are pushed down to single beams in log2(n) vectorized steps.
    """

    def __init__(self, size, increment, radius):
        self.size = size
        self.full = max(1, math.ceil(0.5 * math.pi / increment))
        # k(r) is the smallest m with r >= radius / sin(m increment). Those ranges ascend with
        # decreasing m: k(r) = 1 + #{m : radius / sin(m increment) > r}, a sorted lookup.
        m = np.arange(self.full, 0, -1)
        self.ranges_for_width = radius / np.sin(np.minimum(m * increment, 0.5 * math.pi))
        # floor(log2(L)) for every interval length L <= size.
        self.log2 = np.zeros(size + 1, dtype=np.intp)
        self.log2[1:] = np.floor(np.log2(np.arange(1, size + 1))).astype(np.intp)
        self.levels = int(self.log2[size]) + 1
        self.table = np.empty((self.levels, size), dtype=np.float32)

    def half_widths(self, ranges, scale=1.0):
        """Beams blocked on each side of a point at each range (at least 1, at most 90 deg)."""
        k = 1 + self.full - np.searchsorted(self.ranges_for_width, ranges / scale, side='right')
        return np.minimum(k, self.full, out=k)

    def __call__(self, ranges, max_range, scale=1.0):
        """
        Inflated copy of `ranges` (float32, size beams). Ranges >= max_range are open space.

        `scale` shrinks the radius (scale * radius) without rebuilding the lookup table.
        """
        n = self.size
        inflated = ranges.copy()
        points = np.flatnonzero(ranges < max_range)
        if points.size == 0:
            return inflated
        values = ranges[points]
        k = self.half_widths(values, scale)
        lo = np.maximum(points - k, 0)
        hi = np.minimum(points + k, n - 1)
        p = self.log2[hi - lo + 1]
        levels = int(p.max()) + 1
        table = self.table[:levels]
        table.fill(np.inf)
        flat = table.reshape(-1)
        np.minimum.at(flat, p * n + lo, values)
        np.minimum.at(flat, p * n + hi + 1 - (1 << p), values)
        # A block of 2^q beams at i holds the blocks of 2^(q-1) at i and i + 2^(q-1).
        for q in range(levels - 1, 0, -1):
            half = 1 << (q - 1)
            below, above = table[q - 1], table[q]
            np.minimum(below, above, out=below)
            np.minimum(below[half:], above[:-half], out=below[half:])
        return np.minimum(inflated, table[0], out=inflated)


def largest_gap(free, heading=0, keep=None, hysteresis=1.0):
    """
    (start, stop) of the longest run of True in `free` (ties: nearest `heading`), or None.

    The run containing beam `keep` (the last target) counts `hysteresis` times its length, so the
    car does not switch between two gaps of about the same size (around a centred obstacle).
    """
    edges = np.diff(np.concatenate(([0], free.view(np.int8), [0])))
    starts = np.flatnonzero(edges == 1)
    if starts.size == 0:
        return None
    stops = np.flatnonzero(edges == -1)
    lengths = (stops - starts).astype(float)
    if keep is not None:
        lengths[(starts <= keep) & (keep < stops)] *= hysteresis
    # Lengths are integers: a tie-break below 1 never beats a longer run.
    distance = np.abs(starts + stops - 1 - 2 * heading) / (2.0 * free.size + 2.0)
    best = int(np.argmax(lengths - distance))
    return int(starts[best]), int(stops[best])


class FollowTheGap:
    """Speed and steering from one scan: drive into the largest gap that fits the car."""

    def __init__(self, fov=math.pi, max_range=10.0, smoothing_window=3, inflation_radius=0.25,
                 base_gap_threshold=2.0, speed_gain=0.5, max_gap_threshold=4.0,
                 target_mode='middle', deepest_fraction=0.5, lookahead=2.0, wheelbase=0.33,
                 max_steering=0.4189, steering_alpha=1.0, max_speed=2.0, min_speed=0.5,
                 max_decel=3.0, stop_distance=0.3, min_gap_threshold=1.0,
                 fallback_ratio=0.8, target_fraction=0.5, gap_hysteresis=1.0):
        if fov <= 0.0 or max_range <= 0.0 or inflation_radius <= 0.0:
            raise ValueError('fov, max_range and inflation_radius must be positive')
        if not 0.0 < base_gap_threshold <= max_gap_threshold:
            raise ValueError('base_gap_threshold must be in (0, max_gap_threshold]')
        if target_mode not in ('middle', 'deepest'):
            raise ValueError("target_mode must be 'middle' or 'deepest'")
        if not 0.0 < deepest_fraction <= 1.0 or not 0.0 < steering_alpha <= 1.0:
            raise ValueError('deepest_fraction and steering_alpha must be in (0, 1]')
        if not 0.0 <= min_speed <= max_speed or max_decel <= 0.0:
            raise ValueError('Need 0 <= min_speed <= max_speed and a positive max_decel')
        if lookahead <= 0.0 or wheelbase <= 0.0 or max_steering <= 0.0:
            raise ValueError('lookahead, wheelbase and max_steering must be positive')
        if min_gap_threshold <= 0.0 or not 0.0 < fallback_ratio <= 1.0:
            raise ValueError('min_gap_threshold must be positive and fallback_ratio in (0, 1]')
        if not 0.0 <= target_fraction <= 1.0 or gap_hysteresis < 1.0:
            raise ValueError('target_fraction must be in [0, 1] and gap_hysteresis at least 1')
        self.fov = fov
        self.max_range = max_range
        self.smoothing_window = int(smoothing_window)
        self.inflation_radius = inflation_radius
        self.base_gap_threshold = base_gap_threshold
        self.speed_gain = speed_gain
        self.max_gap_threshold = max_gap_threshold
        self.target_mode = target_mode
        self.deepest_fraction = deepest_fraction
        self.lookahead = lookahead
        self.wheelbase = wheelbase
        self.max_steering = abs(max_steering)
        self.steering_alpha = steering_alpha
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_decel = max_decel
        self.stop_distance = stop_distance
        self.min_gap_threshold = min_gap_threshold
        self.fallback_ratio = fallback_ratio
        self.target_fraction = target_fraction
        self.gap_hysteresis = gap_hysteresis
        self.target = None  # beam of the last target in the field of view, None after a stop
        self.layout = None
        self.speed = 0.0
        self.steering = 0.0
        self.inflated = None

    def processed_ranges(self, ranges, angle_min, angle_increment, range_max):
        """Clean, smooth and inflate the ranges, and return those of the field of view."""
        ranges = np.asarray(ranges, dtype=np.float32)
        layout = self.layout
        if layout is None or layout.key != (ranges.size, angle_min, angle_increment):
            layout = self.layout = Layout(ranges.size, angle_min, angle_increment, self.fov,
                                          self.inflation_radius)
            turn_radius = self.wheelbase / math.tan(self.max_steering)
            layout.reach = 2.0 * turn_radius * np.abs(np.sin(layout.angles)) + self.stop_distance
        r = ranges[layout.start:layout.stop].copy()
        # NaN compares False everywhere, so `~(r > 0)` catches it with 0 and negatives.
        r[~(r > 0.0) | (r >= range_max)] = self.max_range
        np.minimum(r, self.max_range, out=r)
        if self.smoothing_window > 1:
            np.minimum(r, uniform_filter1d(r, self.smoothing_window, mode='nearest'), out=r)
        # Closer to a point than the radius, its disc would cover 90 degrees on each side, beams
        # pointing away from it too: shrink the radius below the nearest range instead.
        nearest = float(r.min())
        scale = min(1.0, NEAR_FACTOR * nearest / self.inflation_radius)
        return layout.inflate(r, self.max_range, scale)[layout.fov_start:layout.fov_stop]

    def find_gap(self, threshold, ranges=None):
        """Largest gap of `ranges` (the inflated scan) at `threshold`, near the last target."""
        ranges = self.inflated if ranges is None else ranges
        return largest_gap(ranges >= threshold, self.layout.heading, self.target,
                           self.gap_hysteresis)

    def update(self, ranges, angle_min, angle_increment, range_max):
        """Process one scan and return the Command (speed in m/s, steering in rad, left +)."""
        inflated = self.inflated = self.processed_ranges(
            ranges, angle_min, angle_increment, range_max)
        layout = self.layout

        threshold = min(self.max_gap_threshold,
                        self.base_gap_threshold + self.speed_gain * self.speed)
        gap = self.find_gap(threshold)
        if gap is None and threshold > self.base_gap_threshold:
            threshold = self.base_gap_threshold
            gap = self.find_gap(threshold)
        if gap is None:
            # A tight bend, or stopped in front of an obstacle: the deepest beams the car can
            # turn to and still stop before (an arc to range r at angle a needs r >= 2 turning
            # radius |sin a|, plus stop_distance).
            reachable = np.where(inflated >= layout.reach, inflated, 0.0)
            threshold = max(self.min_gap_threshold, self.fallback_ratio * float(reachable.max()))
            gap = self.find_gap(threshold, reachable)
        if gap is None:
            # Boxed in: stop (drive_gate holds it), wheels towards the most open beam.
            self.speed = 0.0
            self.target = None
            self.steering = math.copysign(
                self.max_steering, float(layout.angles[int(np.argmax(inflated))]))
            return Command(0.0, self.steering, None, None, None, None, threshold)

        start, stop = gap
        centre = start + int(round(self.target_fraction * (stop - start - 1)))
        if self.target_mode == 'middle':
            target = centre
        else:
            half = max(1, int(0.5 * self.deepest_fraction * (stop - start)))
            lo, hi = max(start, centre - half), min(stop, centre + half + 1)
            target = lo + int(np.argmax(inflated[lo:hi]))
        self.target = target
        alpha = float(layout.angles[target])
        distance = min(float(inflated[target]), self.lookahead)

        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), distance)
        steering = self.steering_alpha * steering + (1.0 - self.steering_alpha) * self.steering
        self.steering = max(-self.max_steering, min(self.max_steering, steering))

        speed = self.max_speed - (self.max_speed - self.min_speed) * \
            abs(self.steering) / self.max_steering
        brake = 2.0 * self.max_decel * max(0.0, float(inflated[target]) - self.stop_distance)
        self.speed = max(self.min_speed, min(speed, math.sqrt(brake)))
        return Command(self.speed, self.steering, alpha, distance,
                       float(layout.angles[start]), float(layout.angles[stop - 1]), threshold)
