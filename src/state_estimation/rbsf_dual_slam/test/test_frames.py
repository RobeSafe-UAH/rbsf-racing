import pytest
from rbsf_dual_slam.frames import cartographer_config, config_variant, slam_names


def test_primary_owns_map_and_tf():
    names = slam_names('/r0', 'slam_toolbox', primary=True)
    assert (names.map_frame, names.map_topic, names.publish_tf) == ('map', '/map', True)
    assert (names.odom_frame, names.base_frame) == ('r0/odom', 'r0/base_link')


def test_secondary_has_its_own_frame_and_topic():
    names = slam_names('r1', 'cartographer', primary=False)
    assert names.map_frame == 'r1/cartographer/map'
    assert names.map_topic == '/r1/mapping/cartographer/map'
    assert not names.publish_tf


def test_unknown_slam_is_rejected():
    with pytest.raises(ValueError):
        slam_names('r0', 'gmapping', primary=True)


def test_config_variant():
    assert [config_variant(p) for p in ('real', 'mvsim', 'f1tenth')] == ['real', 'sim', 'sim']


def test_cartographer_config_sets_frames_and_tf():
    lua = cartographer_config('options = {}\n', slam_names('r0', 'cartographer', primary=False))
    assert lua.startswith('options = {}\n')
    assert 'options.map_frame = "r0/cartographer/map"' in lua
    assert 'options.tracking_frame = "r0/base_link"' in lua
    assert 'options.published_frame = "r0/odom"' in lua
    assert 'options.publish_to_tf = false' in lua
    assert 'options.publish_tracked_pose = true' in lua
    assert lua.rstrip().endswith('return options')
