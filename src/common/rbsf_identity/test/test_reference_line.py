import math

from builtin_interfaces.msg import Time
import numpy as np
import pytest
from rbsf_identity.reference_line import path_msg, trajectory_msg


def test_path_heads_along_the_line_in_the_map_frame():
    square = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    path = path_msg(square, Time(sec=7))
    assert path.header.frame_id == 'map'
    assert all(p.header.frame_id == 'map' and p.header.stamp.sec == 7 for p in path.poses)
    assert [[p.pose.position.x, p.pose.position.y] for p in path.poses] == square.tolist()
    yaws = [2.0 * math.atan2(p.pose.orientation.z, p.pose.orientation.w) for p in path.poses]
    assert yaws == pytest.approx([0.0, math.pi / 2, math.pi, -math.pi / 2])


def test_no_line_gives_an_empty_path():
    assert path_msg([], Time()).poses == []


def test_trajectory_carries_the_raceline_speeds_in_the_map_frame():
    square = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    trajectory = trajectory_msg(square, np.array([1.0, 2.0, 3.0, 4.0]), Time(sec=7))
    assert trajectory.header.frame_id == 'map' and trajectory.header.stamp.sec == 7
    assert [[w.point.x, w.point.y] for w in trajectory.waypoints] == square.tolist()
    assert [w.speed for w in trajectory.waypoints] == [1.0, 2.0, 3.0, 4.0]


def test_a_line_without_speeds_leaves_them_unset():
    trajectory = trajectory_msg(np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]), None, Time())
    assert all(math.isnan(w.speed) for w in trajectory.waypoints)
    assert trajectory_msg([], None, Time()).waypoints == []
