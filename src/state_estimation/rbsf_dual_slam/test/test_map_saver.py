import pytest
from rbsf_dual_slam.map_saver_node import parse_maps, Trajectory


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
