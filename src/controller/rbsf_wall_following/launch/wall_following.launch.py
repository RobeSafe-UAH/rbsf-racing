"""
Controller slot: reactive wall following (see rbsf_wall_following/wall_follower_node.py).

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2): takes robot, platform,
track and use_sim_time, and runs the node in /<robot>. Parameters come from
config/wall_following_real.yaml on the real car and wall_following_sim.yaml on the simulators.

wall_side:=auto (default) follows the track's outer wall when `track` has a reference line
(right on a counter-clockwise track, left on a clockwise one); otherwise the YAML's wall_side.

    Subscribes  /<robot>/scan
    Publishes   /<robot>/drive_gate/auto_cmd
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import numpy as np
from rbsf_worlds.paths import load_waypoints
from rbsf_worlds.tracks import find_waypoints, resolve_map_yaml

PLATFORMS = ('real', 'mvsim', 'f1tenth')


def outer_wall(track):
    """'right' on a counter-clockwise reference line, 'left' on a clockwise one; None if none."""
    if not track:
        return None
    try:
        xy, _ = load_waypoints(find_waypoints(resolve_map_yaml(track)))
    except FileNotFoundError:
        return None
    x, y = xy[:, 0], xy[:, 1]
    area = 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))
    return 'right' if area > 0.0 else 'left'


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    platform = arg('platform')
    if platform not in PLATFORMS:
        raise RuntimeError(f"Unknown platform '{platform}'. Valid options: {list(PLATFORMS)}")
    config = arg('params_file') or os.path.join(
        get_package_share_directory('rbsf_wall_following'), 'config',
        f"wall_following_{'real' if platform == 'real' else 'sim'}.yaml")
    overrides = {'use_sim_time': arg('use_sim_time').lower() == 'true'}
    wall_side = arg('wall_side')
    if wall_side == 'auto':
        wall_side = outer_wall(arg('track'))
    elif wall_side not in ('left', 'right'):
        raise RuntimeError(f"wall_side must be auto, left or right, not '{wall_side}'")
    actions = []
    if wall_side:
        overrides['wall_side'] = wall_side
        actions.append(LogInfo(msg=f'Wall following: {wall_side} wall'))
    return actions + [Node(
        package='rbsf_wall_following',
        executable='wall_follower',
        namespace=arg('robot').strip('/'),
        parameters=[config, overrides],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'platform', default_value='mvsim', description='real, mvsim or f1tenth.'),
        DeclareLaunchArgument(
            'track', default_value='', description='rbsf_worlds track (for wall_side:=auto).'),
        DeclareLaunchArgument(
            'wall_side', default_value='auto',
            description="auto (the track's outer wall, else the YAML), left or right."),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Parameter YAML; default picks _real/_sim from platform.'),
        OpaqueFunction(function=launch_setup),
    ])
