import math

import numpy as np
import pytest
from rbsf_worlds.paths import arc_lengths, headings, load_waypoints, pose_at, spawn_poses


def test_load_centerline_and_raceline(tmp_path):
    centerline = tmp_path / 'c.csv'
    centerline.write_text('# x_m, y_m, w_tr_right_m, w_tr_left_m\n0,0,1,1\n1,0,1,1\n1,1,1,1\n')
    xy, speeds = load_waypoints(centerline)
    assert xy.shape == (3, 2) and speeds is None

    raceline = tmp_path / 'r.csv'
    raceline.write_text('# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n'
                        '0;0;0;0;0;5;0\n1;1;0;0;0;6;0\n2;1;1;0;0;7;0\n')
    xy, speeds = load_waypoints(raceline)
    assert xy[1].tolist() == [1.0, 0.0] and speeds.tolist() == [5.0, 6.0, 7.0]


def test_arc_length_and_pose_on_closed_path():
    square = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]])
    cumulative, length = arc_lengths(square)
    assert length == pytest.approx(8.0)
    assert cumulative.tolist() == [0.0, 2.0, 4.0, 6.0]
    x, y, yaw = pose_at(square, 3.0)
    assert (x, y, yaw) == pytest.approx((2.0, 1.0, math.pi / 2))
    assert pose_at(square, 9.0)[:2] == pytest.approx((1.0, 0.0))  # wraps around


def test_spawn_poses_are_spaced_along_the_path_with_ego_last():
    square = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
    poses = spawn_poses(square, 3, 4.0)
    assert [p[:2] for p in poses] == [pytest.approx((0.0, 0.0)), pytest.approx((4.0, 0.0)),
                                      pytest.approx((8.0, 0.0))]
    assert all(p[2] == pytest.approx(0.0) for p in poses)
    assert spawn_poses(square, 2, 12.0)[1] == pytest.approx((10.0, 2.0, math.pi / 2))
    with pytest.raises(ValueError):
        spawn_poses(square, 5, 10.0)


def test_headings_point_to_the_next_waypoint_and_wrap():
    square = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    assert headings(square) == pytest.approx([0.0, math.pi / 2, math.pi, -math.pi / 2])
