from rbsf_worlds.paths import arc_lengths, load_waypoints
from rbsf_worlds.tracks import find_waypoints, resolve_map_yaml, track_names, track_yaml
import yaml


def test_both_tracks_are_installed_with_their_lines():
    assert {'levine', 'spielberg'} <= set(track_names())
    levine = track_yaml('levine')
    assert find_waypoints(levine).name == 'levine_centerline.csv'  # no raceline: falls back
    spielberg = track_yaml('spielberg')
    assert find_waypoints(spielberg).name == 'spielberg_raceline.csv'
    assert find_waypoints(spielberg, 'centerline').name == 'spielberg_centerline.csv'


def test_map_images_exist_and_lines_are_closed_loops():
    for name, length in (('levine', 61.8), ('spielberg', 338.1)):
        spec = yaml.safe_load(track_yaml(name).read_text())
        assert (track_yaml(name).parent / spec['image']).exists()
        _, total = arc_lengths(load_waypoints(find_waypoints(track_yaml(name)))[0])
        assert abs(total - length) < 1.0


def test_resolve_accepts_track_names_and_absolute_paths():
    levine = track_yaml('levine')
    assert resolve_map_yaml('levine') == levine
    assert resolve_map_yaml(str(levine)) == levine
    assert resolve_map_yaml(str(levine.with_suffix(''))) == levine
