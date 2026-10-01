import pytest
from stack_launcher.registry import (
    DEFAULT_TRACK, IMPLEMENTATIONS, PLATFORMS, resolve_track, SLOTS, USE_SIM_TIME, validate)

VALID = {slot: 'identity' for slot in SLOTS}


def test_every_slot_has_an_identity():
    assert set(IMPLEMENTATIONS) == set(SLOTS)
    for slot in SLOTS:
        assert 'identity' in IMPLEMENTATIONS[slot]
    assert set(USE_SIM_TIME) == set(PLATFORMS)


def test_valid_selection():
    validate('mvsim', 'local', {**VALID, 'controller': 'wall_following'})
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
