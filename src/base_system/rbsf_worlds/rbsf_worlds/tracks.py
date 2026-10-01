"""
Track lookup shared by the simulators: map yamls and their reference line CSVs.

A track is a directory in this package's share/tracks/ (see tracks/README.md). Launch files accept
either a track name or a map path; resolve_map_yaml() turns both into the map yaml.
"""

import pathlib
import subprocess

from ament_index_python.packages import get_package_share_directory
import yaml


def tracks_dir():
    """Directory holding the installed tracks."""
    return pathlib.Path(get_package_share_directory('rbsf_worlds')) / 'tracks'


def track_names():
    """Names of the installed tracks."""
    return sorted(p.name for p in tracks_dir().iterdir() if (p / f'{p.name}.yaml').exists())


def track_yaml(name):
    """Map yaml of track `name`."""
    path = tracks_dir() / name / f'{name}.yaml'
    if not path.exists():
        raise FileNotFoundError(f"Unknown track '{name}'. Available: {', '.join(track_names())}")
    return path


def _yaml_candidates(path):
    if path.suffix in ('.yaml', '.yml'):
        return [path]
    return [path.with_suffix('.yaml'), path.with_suffix('.yml'),
            path.parent / f'{path.stem}_map.yaml']


def resolve_map_yaml(map_path, gym_python=None):
    """
    Find the map yaml for `map_path`.

    Tried in order: an absolute path (with or without extension), a track name of this package, a
    path relative to the f1tenth_gym_ros share directory (maps/levine), and, when `gym_python`
    (the f1tenth_gym interpreter) is given, an f1tenth_gym track name, downloaded on first use.
    """
    path = pathlib.Path(map_path)
    if path.is_absolute():
        candidates = _yaml_candidates(path)
    else:
        candidates = [tracks_dir() / map_path / f'{map_path}.yaml']
        try:
            gym_ros_share = pathlib.Path(get_package_share_directory('f1tenth_gym_ros'))
            candidates += _yaml_candidates(gym_ros_share / path)
        except LookupError:
            pass
    for candidate in candidates:
        if candidate.exists():
            return candidate
    if gym_python and not path.is_absolute():
        track_dir = pathlib.Path(subprocess.run(
            [gym_python, '-c', 'import sys; from f1tenth_gym.envs.track.utils import '
                               'find_track_dir; print(find_track_dir(sys.argv[1]))', map_path],
            check=True, capture_output=True, text=True).stdout.strip().splitlines()[-1])
        for candidate in (track_dir / f'{track_dir.stem}_map.yaml',
                          track_dir / f'{track_dir.stem}.yaml'):
            if candidate.exists():
                return candidate
    raise FileNotFoundError(
        f"No map yaml for '{map_path}'. Tracks: {', '.join(track_names())}")


def find_waypoints(map_yaml, kind='auto'):
    """
    Return the reference line CSV of a map: kind 'raceline', 'centerline' or 'auto'.

    Looks for <map>_<kind>.csv next to the yaml, then a `<kind>:` key in the yaml (a path
    relative to it). 'auto' prefers the raceline, which carries a speed profile.
    """
    map_yaml = pathlib.Path(map_yaml)
    with open(map_yaml) as f:
        spec = yaml.safe_load(f) or {}
    for k in (('raceline', 'centerline') if kind == 'auto' else (kind,)):
        for candidate in (map_yaml.parent / f'{map_yaml.stem}_{k}.csv',
                          map_yaml.parent / spec[k] if spec.get(k) else None):
            if candidate is not None and candidate.exists():
                return candidate
    raise FileNotFoundError(f'{map_yaml}: no {kind} waypoint file')
