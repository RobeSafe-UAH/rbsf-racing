import pathlib

import pytest
from rbsf_map_manager.known_map import known_map_yaml
from rbsf_worlds.tracks import resolve_map_yaml
import yaml


def test_track_without_cones_is_served_as_is(tmp_path):
    assert known_map_yaml('eps_circuit', tmp_path) == str(resolve_map_yaml('eps_circuit'))


def test_cones_are_drawn_into_a_copy(tmp_path):
    served = pathlib.Path(known_map_yaml('levine', tmp_path))
    assert served.parent == tmp_path
    spec = yaml.safe_load(served.read_text())
    assert 'cones' not in spec and (tmp_path / spec['image']).exists()


def test_a_track_is_required(tmp_path):
    with pytest.raises(ValueError, match='track:='):
        known_map_yaml('', tmp_path)
