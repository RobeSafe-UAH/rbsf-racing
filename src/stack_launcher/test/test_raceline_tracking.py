import importlib.util
import pathlib

import pytest
from stack_launcher.registry import PLATFORMS, SLOTS, validate

LAUNCH = pathlib.Path(__file__).parents[1] / 'launch' / 'raceline_tracking.launch.py'


@pytest.fixture(scope='module')
def behaviour():
    spec = importlib.util.spec_from_file_location('raceline_tracking', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_real_car_localizes_on_the_known_map(behaviour):
    assert behaviour.default_slot('real', 'mapping') == 'map_server'
    assert behaviour.default_slot('real', 'localization') == 'particle_filter'
    for platform in ('mvsim', 'f1tenth'):
        assert behaviour.default_slot(platform, 'mapping') == 'identity'
        assert behaviour.default_slot(platform, 'localization') == 'identity'


@pytest.mark.parametrize('platform', PLATFORMS)
def test_defaults_pass_validation(behaviour, platform):
    validate(platform, 'local', {slot: behaviour.default_slot(platform, slot) for slot in SLOTS})
