import math

import pytest
from rbsf_dual_slam.map_saver_node import GroundTruthAlignment, parse_maps, Trajectory


def test_parse_maps():
    assert parse_maps(['slam_toolbox:/map', 'cartographer:/r0/mapping/cartographer/map']) == {
        'slam_toolbox': '/map', 'cartographer': '/r0/mapping/cartographer/map'}


@pytest.mark.parametrize('entry', ['/map', ':/map', 'slam_toolbox:'])
def test_parse_maps_rejects_bad_entries(entry):
    with pytest.raises(ValueError):
        parse_maps([entry])


def test_trajectory_keeps_poses_every_spacing(tmp_path):
    trajectory = Trajectory(0.5)
    for i in range(11):
        trajectory.add(i * 0.1, i * 0.1, 0.0, 0.0)  # 1 m in 0.1 m steps
    assert [p[1] for p in trajectory.poses] == pytest.approx([0.0, 0.5, 1.0])
    trajectory.save(tmp_path / 'trajectory.csv', 'map')
    lines = (tmp_path / 'trajectory.csv').read_text().splitlines()
    assert lines[0] == '# t_s, x_m, y_m, yaw_rad (frame map)'
    assert len(lines) == 4


def test_ground_truth_alignment_recovers_the_world_pose_in_map():
    # map is the car's start pose (2, 1, 30 deg) in the world: the world frame's pose in map is
    # the inverse of it.
    x0, y0, yaw0 = 2.0, 1.0, math.radians(30.0)
    c, s = math.cos(yaw0), math.sin(yaw0)
    alignment = GroundTruthAlignment()
    assert alignment.fit() is None
    for i in range(100):
        t = 0.1 * i
        wx, wy = 3.0 * math.cos(0.1 * i), 2.0 * math.sin(0.1 * i)  # world positions
        alignment.add_ground_truth(t, wx, wy)
        dx, dy = wx - x0, wy - y0
        alignment.add_map(t + 0.01, c * dx + s * dy, -s * dx + c * dy)
    (x, y, yaw), rms = alignment.fit()
    assert yaw == pytest.approx(-yaw0)
    assert (x, y) == pytest.approx((-(c * x0 + s * y0), s * x0 - c * y0))
    assert rms == pytest.approx(0.0, abs=1e-9)


def test_ground_truth_alignment_skips_unmatched_stamps():
    alignment = GroundTruthAlignment()
    alignment.add_ground_truth(0.0, 0.0, 0.0)
    alignment.add_map(1.0, 0.0, 0.0)  # 1 s away from any ground truth
    assert alignment.map == []
