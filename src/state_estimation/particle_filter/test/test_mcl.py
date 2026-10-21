import math

import numpy as np
from particle_filter import mcl
import pytest


def test_wrap_angle():
    np.testing.assert_allclose(mcl.wrap_angle([0.0, 3 * math.pi / 2, -3 * math.pi / 2, 7.0]),
                               [0.0, -math.pi / 2, math.pi / 2, 7.0 - 2 * math.pi])


@pytest.mark.parametrize('yaw', [0.0, 1.0, -2.5, math.pi - 1e-6])
def test_quaternion_round_trip(yaw):
    assert mcl.yaw_from_quaternion(*mcl.quaternion_from_yaw(yaw)) == pytest.approx(yaw)


def test_compose_inverse():
    a, b = np.array([1.0, 2.0, 0.5]), np.array([-0.3, 0.7, 2.9])
    np.testing.assert_allclose(mcl.compose(mcl.compose(a, b), mcl.inverse(b)), a, atol=1e-12)
    np.testing.assert_allclose(mcl.compose(a, mcl.inverse(a)), 0.0, atol=1e-12)


def test_odom_delta_is_in_the_car_frame_and_wraps_yaw():
    # Facing +y and crossing yaw = +-pi: 1 m forward is +x in the car frame, small left turn.
    last = np.array([5.0, 5.0, math.pi - 0.05])
    now = np.array([5.0 + math.cos(last[2]), 5.0 + math.sin(last[2]), -math.pi + 0.05])
    np.testing.assert_allclose(mcl.odom_delta(last, now), [1.0, 0.0, 0.1], atol=1e-9)


def test_map_to_odom_places_odometry_on_the_estimate():
    map_base, odom_base = np.array([3.0, -1.0, 2.0]), np.array([0.4, 0.2, -0.7])
    map_odom = mcl.map_to_odom(map_base, odom_base)
    np.testing.assert_allclose(mcl.compose(map_odom, odom_base), map_base, atol=1e-12)


def test_motion_model_moves_each_particle_in_its_own_frame():
    particles = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, math.pi / 2], [0.0, 0.0, math.pi]])
    mcl.motion_model(particles, np.array([1.0, 0.0, 0.2]), np.zeros(3),
                     np.random.default_rng(0))
    np.testing.assert_allclose(
        particles, [[1.0, 0.0, 0.2], [1.0, 2.0, math.pi / 2 + 0.2], [-1.0, 0.0, -math.pi + 0.2]],
        atol=1e-12)


def test_laser_poses_apply_the_mount_offset():
    particles = np.array([[1.0, 2.0, math.pi / 2]])
    np.testing.assert_allclose(mcl.laser_poses(particles, (0.27, 0.0, 0.0)),
                               [[1.0, 2.27, math.pi / 2]], atol=1e-6)


def test_expected_pose_averages_yaw_across_pi():
    particles = np.array([[0.0, 0.0, math.pi - 0.1], [2.0, 4.0, -math.pi + 0.1]])
    pose = mcl.expected_pose(particles, np.array([0.5, 0.5]))
    assert pose[:2] == pytest.approx([1.0, 2.0])
    assert abs(mcl.wrap_angle(pose[2] - math.pi)) < 1e-9
    assert mcl.covariance(particles, np.array([0.5, 0.5]))[2, 2] == pytest.approx(0.01)


def test_sensor_model_table_columns_are_distributions_peaked_at_the_expected_range():
    table = mcl.sensor_model_table(200, 0.75, 0.01, 0.07, 0.12, 8.0)
    assert table.shape == (201, 201)
    np.testing.assert_allclose(table.sum(axis=0), 1.0)
    assert np.argmax(table[:200, 100]) == 100
    assert table[200, 100] > table[199, 100]   # the max-range spike


def test_clean_ranges():
    ranges = mcl.clean_ranges([1.0, float('inf'), float('nan'), 12.0, 0.5], 10.0)
    assert ranges.dtype == np.float32
    np.testing.assert_allclose(ranges, [1.0, 10.0, 10.0, 10.0, 0.5])


def test_global_poses_land_on_free_cells():
    rows, cols = np.array([2, 3]), np.array([5, 7])
    poses = mcl.global_poses(rows, cols, 0.1, (-1.0, 0.5), 100, np.random.default_rng(1))
    centres = {(-1.0 + 0.55, 0.5 + 0.25), (-1.0 + 0.75, 0.5 + 0.35)}
    assert {(round(x, 6), round(y, 6)) for x, y, _ in poses} == {
        (round(x, 6), round(y, 6)) for x, y in centres}
    assert np.all(np.abs(poses[:, 2]) <= math.pi)
