"""
Controller-slot launch description shared by pure_pursuit.launch.py and map.launch.py.

Both run path_tracker (path_tracker_node.py) in /<robot>, named after the steering law, with
config/<law>_real.yaml on the real car or <law>_sim.yaml on the simulators, then the track's tuning
config/tracks/<track>_<real|sim>.yaml when it exists (both laws share it). `params_file` replaces
both.
"""

import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PLATFORMS = ('real', 'mvsim', 'f1tenth')
NODE_NAMES = {'pure_pursuit': 'pure_pursuit', 'map': 'map_controller'}


def config_files(law, platform, track, config_dir):
    """Parameter files for `law`: the platform config, then the track's tuning if it exists."""
    kind = 'real' if platform == 'real' else 'sim'
    files = [os.path.join(config_dir, f'{law}_{kind}.yaml')]
    # A track is a name or a map yaml path; its tuning is keyed by the name (the yaml's stem).
    tuning = os.path.join(config_dir, 'tracks', f'{pathlib.Path(track).stem}_{kind}.yaml')
    if track and os.path.exists(tuning):
        files.append(tuning)
    return files


def tracker_node(law, namespace, platform, track='', params_file='', overrides=None,
                 remappings=()):
    """Return the path_tracker Node for `law`: its configs (or `params_file`) plus `overrides`."""
    if platform not in PLATFORMS:
        raise RuntimeError(f"Unknown platform '{platform}'. Valid options: {list(PLATFORMS)}")
    configs = [params_file] if params_file else config_files(
        law, platform, track,
        os.path.join(get_package_share_directory('rbsf_path_tracking'), 'config'))
    return Node(
        package='rbsf_path_tracking',
        executable='path_tracker',
        name=NODE_NAMES[law],
        namespace=namespace,
        parameters=[*configs, {'steering_law': law, **(overrides or {})}],
        remappings=list(remappings),
        output='screen',
    )


def launch_description(law):
    """Component launch contract (robot, platform, track, use_sim_time) plus tracker options."""
    def launch_setup(context):
        arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
        overrides = {'use_sim_time': arg('use_sim_time').lower() == 'true'}
        if arg('speed'):
            overrides['speed'] = float(arg('speed'))
        return [tracker_node(law, arg('robot').strip('/'), arg('platform'), arg('track'),
                             arg('params_file'), overrides)]

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'platform', default_value='mvsim', description='real, mvsim or f1tenth.'),
        DeclareLaunchArgument(
            'track', default_value='',
            description='rbsf_worlds track or map yaml: selects its tuning in config/tracks/ '
                        '(the line itself comes from planning/trajectory).'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Parameter YAML, replacing the platform and track configs.'),
        DeclareLaunchArgument(
            'speed', default_value='', description='Constant speed (m/s); default from config.'),
        OpaqueFunction(function=launch_setup),
    ])
