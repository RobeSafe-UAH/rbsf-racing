"""
Known-map mapping: the track's map on /map, from nav2's map_server.

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2). Unlike SLAM and the
identity mapping, it publishes no map -> <robot>/odom: on a known map the localization (the
particle filter) owns it (§3.2).

    /<robot>/map_server  lifecycle node, autostarted
        Publishes  /map  nav_msgs/OccupancyGrid (frame map, latched)

`track` is an rbsf_worlds track name or a map yaml path (e.g. a car-mapped session's
maps/<stamp>/track/<name>.yaml). Synthetic cones are drawn in, as the simulators do.
"""

import tempfile

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rbsf_identity.contract import contract_arguments
from rbsf_map_manager.known_map import known_map_yaml


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context).strip('/')
    track = LaunchConfiguration('track').perform(context)
    map_yaml = known_map_yaml(track, tempfile.mkdtemp(prefix='rbsf_map_server_'))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [
        LogInfo(msg=f'Known map: {map_yaml}'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             namespace=robot, output='screen',
             parameters=[{'yaml_filename': map_yaml, 'topic_name': '/map', 'frame_id': 'map',
                          'autostart_node': True, 'use_sim_time': use_sim_time}]),
    ]


def generate_launch_description():
    return LaunchDescription([*contract_arguments(), OpaqueFunction(function=launch_setup)])
