import importlib.util
import pathlib

import pytest

LAUNCH = pathlib.Path(__file__).parents[1] / 'launch' / 'particle_filter.launch.py'


@pytest.fixture(scope='module')
def launch():
    spec = importlib.util.spec_from_file_location('particle_filter_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_explicit_pose(launch):
    assert launch.initial_pose('1, 2.5, -0.3', 'eps_circuit') == [1.0, 2.5, -0.3]
    with pytest.raises(ValueError, match="'x,y,yaw'"):
        launch.initial_pose('1,2', 'eps_circuit')


def test_global(launch):
    assert launch.initial_pose('global', 'eps_circuit') == []
    assert launch.initial_pose('', '') == []


def test_default_is_the_start_of_the_reference_line(launch):
    # eps_circuit's centerline starts at (0.05, -0.044), heading along its first segment.
    x, y, yaw = launch.initial_pose('', 'eps_circuit')
    assert (x, y) == pytest.approx((0.05, -0.0443))
    assert -3.15 < yaw < 3.15
