import numpy as np
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


def test_iros2026_centerline_is_a_closed_loop_inside_the_corridor():
    iros = track_yaml('iros2026')
    assert 'iros2026' in track_names()
    assert find_waypoints(iros).name == 'iros2026_centerline.csv'
    xy, speeds = load_waypoints(find_waypoints(iros))
    assert speeds is None
    _, total = arc_lengths(xy)
    assert abs(total - 209.4) < 1.0
    # Closed (first point not repeated), evenly sampled, every point at least 0.8 m from a wall.
    gaps = np.linalg.norm(np.diff(xy, axis=0, append=xy[:1]), axis=1)
    assert gaps.max() < 0.15
    widths = np.loadtxt(find_waypoints(iros), delimiter=',', comments='#')[:, 2:]
    assert widths.min() > 0.8


def test_icra2026_centerlines_are_closed_loops_inside_the_corridor():
    for name, length in (('icra2026_classic', 80.8), ('icra2026_master', 152.5)):
        path = find_waypoints(track_yaml(name))
        assert path.name == f'{name}_centerline.csv'
        xy, _ = load_waypoints(path)
        _, total = arc_lengths(xy)
        assert abs(total - length) < 1.0
        gaps = np.linalg.norm(np.diff(xy, axis=0, append=xy[:1]), axis=1)
        assert gaps.max() < 0.15
        widths = np.loadtxt(path, delimiter=',', comments='#')[:, 2:]
        assert widths.min() > 0.5
