"""
Identity planning: the track's reference line on /<robot>/planning/trajectory (reference_line).

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2). The line is the track's
raceline when it has one, else its centerline (`line` picks one explicitly). With no track (the
real car's default) the trajectory is empty.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rbsf_identity.contract import contract_arguments
from rbsf_worlds.tracks import find_waypoints, resolve_map_yaml


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    track = arg('track')
    waypoints = str(find_waypoints(resolve_map_yaml(track), arg('line'))) if track else ''
    return [Node(
        package='rbsf_identity',
        executable='reference_line',
        namespace=arg('robot').strip('/'),
        parameters=[{'waypoints_file': waypoints,
                     'use_sim_time': arg('use_sim_time').lower() == 'true'}],
    )]


def generate_launch_description():
    return LaunchDescription([
        *contract_arguments(),
        DeclareLaunchArgument(
            'line', default_value='auto',
            description="Reference line: 'raceline', 'centerline' or 'auto' (raceline if any)."),
        OpaqueFunction(function=launch_setup),
    ])
