"""
Controller-slot launch description shared by pure_pursuit.launch.py and map.launch.py.

Both run path_tracker (path_tracker_node.py) in /<robot>, named after the steering law, with
config/<law>_real.yaml on the real car or <law>_sim.yaml on the simulators (or `params_file`).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PLATFORMS = ('real', 'mvsim', 'f1tenth')
NODE_NAMES = {'pure_pursuit': 'pure_pursuit', 'map': 'map_controller'}


def tracker_node(law, namespace, platform, params_file='', overrides=None, remappings=()):
    """Return the path_tracker Node for `law`: its platform config plus `overrides`."""
    if platform not in PLATFORMS:
        raise RuntimeError(f"Unknown platform '{platform}'. Valid options: {list(PLATFORMS)}")
    config = params_file or os.path.join(
        get_package_share_directory('rbsf_path_tracking'), 'config',
        f"{law}_{'real' if platform == 'real' else 'sim'}.yaml")
    return Node(
        package='rbsf_path_tracking',
        executable='path_tracker',
        name=NODE_NAMES[law],
        namespace=namespace,
        parameters=[config, {'steering_law': law, **(overrides or {})}],
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
        return [tracker_node(law, arg('robot').strip('/'), arg('platform'),
                             arg('params_file'), overrides)]

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'platform', default_value='mvsim', description='real, mvsim or f1tenth.'),
        DeclareLaunchArgument(
            'track', default_value='',
            description='rbsf_worlds track (unused: the line comes from planning/trajectory).'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Parameter YAML; default picks _real/_sim from platform.'),
        DeclareLaunchArgument(
            'speed', default_value='', description='Constant speed (m/s); default from config.'),
        OpaqueFunction(function=launch_setup),
    ])
