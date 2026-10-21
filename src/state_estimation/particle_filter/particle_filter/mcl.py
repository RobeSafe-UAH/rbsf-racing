# MIT License
#
# Copyright (c) 2020 Hongrui Zheng, Corey Walsh, Chinmay Samak, Tanmay Samak
# Copyright (c) 2026 RobeSafe (Universidad de Alcala)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the 'Software'), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Monte Carlo Localization math, without ROS: 2D poses are numpy arrays (x, y, yaw).

Upstream's (f1tenth/particle_filter) motion model, sensor-model table and pose estimate, pulled
out of the node so they can be unit-tested. Particles are poses of the car's base frame in the
map frame; the sensor model ray-casts from the laser pose (laser_poses).
"""

import numpy as np


def wrap_angle(angle):
    """Angle(s) wrapped to [-pi, pi)."""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def yaw_from_quaternion(x, y, z, w):
    """Yaw (rad) of a unit quaternion."""
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def quaternion_from_yaw(yaw):
    """(x, y, z, w) of a rotation by `yaw` about z."""
    return 0.0, 0.0, float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0))


def compose(a, b):
    """Pose b expressed in a's parent frame: a (A -> B) followed by b (B -> C) gives A -> C."""
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
                     wrap_angle(a[2] + b[2])])


def inverse(a):
    """Inverse of pose a (A -> B), i.e. B -> A."""
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap_angle(-a[2])])


def odom_delta(last, now):
    """Motion from odometry pose `last` to `now`, in `last`'s frame (dx, dy, dyaw)."""
    return compose(inverse(last), now)


def map_to_odom(map_base, odom_base):
    """Return map -> odom from the localized pose (map -> base) and odometry (odom -> base)."""
    return compose(map_base, inverse(odom_base))


def motion_model(particles, delta, dispersion, rng):
    """
    Move each particle by the odometry `delta` (in its own frame), plus Gaussian noise, in place.

    `dispersion` is the noise standard deviation (x, y, yaw), in the map frame as upstream does.
    """
    c, s = np.cos(particles[:, 2]), np.sin(particles[:, 2])
    particles[:, 0] += c * delta[0] - s * delta[1]
    particles[:, 1] += s * delta[0] + c * delta[1]
    particles[:, 2] += delta[2]
    particles += rng.normal(0.0, dispersion, size=particles.shape)
    particles[:, 2] = wrap_angle(particles[:, 2])


def laser_poses(particles, offset, out=None):
    """Laser pose of each particle (base pose) given the base -> laser `offset` (x, y, yaw)."""
    out = np.empty(particles.shape, dtype=np.float32) if out is None else out
    c, s = np.cos(particles[:, 2]), np.sin(particles[:, 2])
    out[:, 0] = particles[:, 0] + c * offset[0] - s * offset[1]
    out[:, 1] = particles[:, 1] + s * offset[0] + c * offset[1]
    out[:, 2] = particles[:, 2] + offset[2]
    return out


def expected_pose(particles, weights):
    """Weighted mean pose; yaw is a circular mean, so particles across +-pi average right."""
    x, y = weights @ particles[:, 0], weights @ particles[:, 1]
    yaw = np.arctan2(weights @ np.sin(particles[:, 2]), weights @ np.cos(particles[:, 2]))
    return np.array([x, y, yaw])


def covariance(particles, weights):
    """Weighted 3x3 covariance of the particles (yaw residuals wrapped around the mean)."""
    residual = particles - expected_pose(particles, weights)
    residual[:, 2] = wrap_angle(residual[:, 2])
    return (residual * weights[:, None]).T @ residual


def sensor_model_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_hit):
    """
    Beam model table P(measured r | expected d), in map pixels, indexed [r, d].

    Upstream's mixture: a Gaussian hit around d, short readings below d, a max-range spike and a
    uniform random term. Each column (expected range d) is normalized.
    """
    width = int(max_range_px) + 1
    r, d = np.meshgrid(np.arange(width, dtype=float), np.arange(width, dtype=float),
                       indexing='ij')
    z = r - d
    table = z_hit * np.exp(-(z * z) / (2.0 * sigma_hit * sigma_hit)) / (
        sigma_hit * np.sqrt(2.0 * np.pi))
    with np.errstate(divide='ignore', invalid='ignore'):
        table += np.where(r < d, 2.0 * z_short * (d - r) / d, 0.0)
    table[r == int(max_range_px)] += z_max
    table[r < int(max_range_px)] += z_rand / float(max_range_px)
    return table / table.sum(axis=0, keepdims=True)


def clean_ranges(ranges, max_range):
    """Scan ranges as float32 with NaN, inf and out-of-range readings set to `max_range`."""
    ranges = np.asarray(ranges, dtype=np.float32).copy()
    ranges[~np.isfinite(ranges) | (ranges > max_range)] = max_range
    return ranges


def global_poses(free_rows, free_cols, resolution, origin, count, rng):
    """`count` poses spread uniformly over the free map cells, with random yaw (global init)."""
    pick = rng.integers(0, len(free_rows), size=count)
    poses = np.empty((count, 3))
    # Cell centres; origin is (x, y, yaw) of cell (0, 0)'s corner, yaw ignored (as map_server).
    poses[:, 0] = origin[0] + (free_cols[pick] + 0.5) * resolution
    poses[:, 1] = origin[1] + (free_rows[pick] + 0.5) * resolution
    poses[:, 2] = rng.uniform(-np.pi, np.pi, size=count)
    return poses


def gaussian_poses(pose, std, count, rng):
    """`count` poses around `pose`, with standard deviation `std` (x, y, yaw)."""
    poses = np.asarray(pose, dtype=float) + rng.normal(0.0, std, size=(count, 3))
    poses[:, 2] = wrap_angle(poses[:, 2])
    return poses
