"""
Mapping slot: DualSLAM. Cartographer and SLAM Toolbox map side by side from the same scans.

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2), plus output_dir.
Cartographer is the primary: it alone owns /map and map -> <robot>/odom (it builds the better
maps in sim and on the car). SLAM Toolbox runs without TF on /<robot>/mapping/slam_toolbox/map,
in the frame <robot>/slam_toolbox/map, which map_overlay hangs below map, fitted to SLAM
Toolbox's poses of the car, so the maps overlay. One map_saver saves both (joystick button Y, or
`ros2 service call /<robot>/mapping/save_maps std_srvs/srv/Trigger`) into
<output_dir>/<YYYYMMDD_HHMMSS>/{cartographer,slam_toolbox}/.

    Subscribes  /<robot>/scan, /tf (<robot>/odom -> <robot>/base_link from the platform)
    Publishes   /map, TF map -> <robot>/odom (cartographer)
                /<robot>/mapping/slam_toolbox/map, /tf_static map -> <robot>/slam_toolbox/map
    Services    /<robot>/mapping/save_maps
"""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from rbsf_dual_slam import launch_nodes
from rbsf_dual_slam.contract import check_platform, slam_arguments
from rbsf_dual_slam.frames import slam_names

PRIMARY, SECONDARY = 'cartographer', 'slam_toolbox'


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    platform = arg('platform')
    check_platform(platform)
    robot = arg('robot').strip('/')
    use_sim_time = arg('use_sim_time').lower() == 'true'
    primary = slam_names(robot, PRIMARY, primary=True)
    secondary = slam_names(robot, SECONDARY, primary=False)
    return [
        *getattr(launch_nodes, PRIMARY)(robot, platform, primary, use_sim_time),
        *getattr(launch_nodes, SECONDARY)(robot, platform, secondary, use_sim_time),
        launch_nodes.overlay_tf(robot, SECONDARY, secondary),
        launch_nodes.map_saver(
            robot, platform, arg('track'),
            {PRIMARY: primary.map_topic, SECONDARY: secondary.map_topic}, PRIMARY,
            arg('output_dir')),
    ]


def generate_launch_description():
    # primary/map_saver are declared (shared contract) but fixed here.
    return LaunchDescription([*slam_arguments(), OpaqueFunction(function=launch_setup)])
