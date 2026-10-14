"""
Controller slot: reactive Follow-the-Gap (see rbsf_follow_the_gap/follow_the_gap_node.py).

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2): takes robot, platform,
track and use_sim_time, and runs the node in /<robot>. Parameters come from
config/follow_the_gap_real.yaml on the real car and follow_the_gap_sim.yaml on the simulators.
The car only reacts to the scan, so `track` is accepted but unused.

    Subscribes  /<robot>/scan
    Publishes   /<robot>/drive_gate/auto_cmd
                /<robot>/follow_the_gap/{scan,markers}  (publish_debug: true only)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PLATFORMS = ('real', 'mvsim', 'f1tenth')


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    platform = arg('platform')
    if platform not in PLATFORMS:
        raise RuntimeError(f"Unknown platform '{platform}'. Valid options: {list(PLATFORMS)}")
    config = arg('params_file') or os.path.join(
        get_package_share_directory('rbsf_follow_the_gap'), 'config',
        f"follow_the_gap_{'real' if platform == 'real' else 'sim'}.yaml")
    return [Node(
        package='rbsf_follow_the_gap',
        executable='follow_the_gap',
        namespace=arg('robot').strip('/'),
        parameters=[config, {'use_sim_time': arg('use_sim_time').lower() == 'true'}],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'platform', default_value='mvsim', description='real, mvsim or f1tenth.'),
        DeclareLaunchArgument(
            'track', default_value='', description='rbsf_worlds track (unused).'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Parameter YAML; default picks _real/_sim from platform.'),
        OpaqueFunction(function=launch_setup),
    ])
