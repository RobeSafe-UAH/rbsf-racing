import pytest
from rbsf_dual_slam.map_saver_node import parse_maps


def test_parse_maps():
    assert parse_maps(['slam_toolbox:/map', 'cartographer:/r0/mapping/cartographer/map']) == {
        'slam_toolbox': '/map', 'cartographer': '/r0/mapping/cartographer/map'}


@pytest.mark.parametrize('entry', ['/map', ':/map', 'slam_toolbox:'])
def test_parse_maps_rejects_bad_entries(entry):
    with pytest.raises(ValueError):
        parse_maps([entry])
