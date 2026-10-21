"""The map yaml the known-map mapping serves for a track (no ROS imports, unit-tested)."""

from rbsf_worlds.cones import with_cones
from rbsf_worlds.tracks import resolve_map_yaml


def known_map_yaml(track, out_dir):
    """
    Map yaml for `track` (an rbsf_worlds track name or a map yaml path), cones drawn in.

    It is the same map the simulators load, so the particle filter ray-casts the walls the LiDAR
    sees. Raises ValueError without a track: a known-map mapping needs one.
    """
    if not track:
        raise ValueError('mapping:=map_server needs track:=<rbsf_worlds track or map yaml>')
    return str(with_cones(resolve_map_yaml(track), out_dir))
