"""
Mapping slot: cartographer SLAM alone (see dual_slam.launch.py to run it next to the other SLAM).

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2), plus primary, map_saver
and output_dir. Config: config/cartographer_{real,sim}.lua, picked from platform.

    Subscribes  /<robot>/scan, /tf (<robot>/odom -> <robot>/base_link from the platform)
    Publishes   /map + TF map -> <robot>/odom              (primary:=true, the default)
                /<robot>/mapping/cartographer/map, no TF  (primary:=false)
    Services    /<robot>/mapping/save_maps                 (map_saver:=true, see map_saver_node.py)
"""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from rbsf_dual_slam import launch_nodes
from rbsf_dual_slam.contract import check_platform, slam_arguments
from rbsf_dual_slam.frames import slam_names

IMPLEMENTATION = 'cartographer'


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    platform = arg('platform')
    check_platform(platform)
    robot = arg('robot').strip('/')
    primary = arg('primary').lower() == 'true'
    names = slam_names(robot, IMPLEMENTATION, primary)
    use_sim_time = arg('use_sim_time').lower() == 'true'
    actions = getattr(launch_nodes, IMPLEMENTATION)(robot, platform, names, use_sim_time)
    if not primary:
        actions.append(launch_nodes.overlay_tf(robot, IMPLEMENTATION, names))
    if arg('map_saver').lower() == 'true':
        actions.append(launch_nodes.map_saver(
            robot, platform, arg('track'), {IMPLEMENTATION: names.map_topic},
            IMPLEMENTATION if primary else '', arg('output_dir')))
    return actions


def generate_launch_description():
    return LaunchDescription([*slam_arguments(), OpaqueFunction(function=launch_setup)])
