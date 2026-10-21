import pytest
from stack_launcher.registry import (
    DEFAULT_TRACK, IMPLEMENTATIONS, PLATFORMS, resolve_track, SLOTS, USE_SIM_TIME, validate)

VALID = {slot: 'identity' for slot in SLOTS}


def test_every_slot_has_an_identity():
    assert set(IMPLEMENTATIONS) == set(SLOTS)
    for slot in SLOTS:
        assert 'identity' in IMPLEMENTATIONS[slot]
    assert set(USE_SIM_TIME) == set(PLATFORMS)


@pytest.mark.parametrize('controller', ['wall_following', 'follow_the_gap', 'pure_pursuit', 'map'])
def test_valid_selection(controller):
    validate('mvsim', 'local', {**VALID, 'controller': controller})
    validate('real', 'distributed', VALID)


@pytest.mark.parametrize('mapping', ['slam_toolbox', 'cartographer', 'dual_slam'])
def test_slam_mappers_are_registered(mapping):
    assert IMPLEMENTATIONS['mapping'][mapping][0] == 'rbsf_dual_slam'
    validate('real', 'distributed', {**VALID, 'mapping': mapping, 'controller': 'wall_following'})


@pytest.mark.parametrize('platform, deployment, slots, message', [
    ('foo', 'local', VALID, "platform 'foo'"),
    ('real', 'remote', VALID, "deployment 'remote'"),
    ('real', 'local', {**VALID, 'controller': 'foo'}, "controller implementation 'foo'"),
    ('real', 'local', {s: 'identity' for s in SLOTS[:-1]}, 'Missing slot(s): controller'),
    ('real', 'local', {**VALID, 'foo': 'identity'}, "slot 'foo'"),
])
def test_invalid_selection_lists_valid_options(platform, deployment, slots, message):
    with pytest.raises(ValueError) as error:
        validate(platform, deployment, slots)
    assert message in str(error.value)
    assert 'Valid options' in str(error.value) or 'Missing' in str(error.value)


def test_track_defaults_per_platform():
    assert set(DEFAULT_TRACK) == set(PLATFORMS)
    assert resolve_track('mvsim', '') == 'levine'
    assert resolve_track('f1tenth', '') == 'levine'
    # The real car drives an unknown track: no simulator track (and its wall side) leaks in.
    assert resolve_track('real', '') == ''
    assert resolve_track('real', '/maps/x/track/t.yaml') == '/maps/x/track/t.yaml'
    assert resolve_track('mvsim', 'iros2026') == 'iros2026'


KNOWN_MAP = {**VALID, 'mapping': 'map_server', 'localization': 'particle_filter'}


def test_known_map_implementations_are_registered():
    assert IMPLEMENTATIONS['mapping']['map_server'][0] == 'rbsf_map_manager'
    assert IMPLEMENTATIONS['localization']['particle_filter'][0] == 'particle_filter'


@pytest.mark.parametrize('platform', PLATFORMS)
@pytest.mark.parametrize('controller', ['pure_pursuit', 'map', 'wall_following'])
def test_known_map_with_particle_filter_drives_anywhere(platform, controller):
    validate(platform, 'local', {**KNOWN_MAP, 'controller': controller})


@pytest.mark.parametrize('mapping, localization', [
    ('map_server', 'identity'),         # nobody owns map -> odom
    ('identity', 'particle_filter'),    # two owners
    ('dual_slam', 'particle_filter'),
])
def test_map_to_odom_has_exactly_one_owner(mapping, localization):
    with pytest.raises(ValueError, match='known-map mapping'):
        validate('mvsim', 'local', {**VALID, 'mapping': mapping, 'localization': localization})


@pytest.mark.parametrize('controller', ['pure_pursuit', 'map'])
def test_real_map_frame_controller_needs_a_real_map_pose(controller):
    with pytest.raises(ValueError, match='cannot place it on the track'):
        validate('real', 'local', {**VALID, 'controller': controller})
    # Ground truth makes identity fine on the simulators, and SLAM gives a map pose on the car.
    validate('mvsim', 'local', {**VALID, 'controller': controller})
    validate('real', 'local', {**VALID, 'mapping': 'dual_slam', 'controller': controller})
