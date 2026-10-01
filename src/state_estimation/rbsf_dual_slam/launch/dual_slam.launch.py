"""
Mapping slot: DualSLAM. SLAM Toolbox and Cartographer map side by side from the same scans.

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2), plus output_dir.
SLAM Toolbox is the primary: it alone owns /map and map -> <robot>/odom. Cartographer runs
without TF on /<robot>/mapping/cartographer/map, in the frame <robot>/cartographer/map, which
map_overlay hangs below map once, from Cartographer's first tracked_pose, so the maps overlay. One
map_saver saves both (joystick button Y, or `ros2 service call /<robot>/mapping/save_maps
std_srvs/srv/Trigger`) into <output_dir>/<YYYYMMDD_HHMMSS>/{slam_toolbox,cartographer}/.

    Subscribes  /<robot>/scan, /tf (<robot>/odom -> <robot>/base_link from the platform)
    Publishes   /map, TF map -> <robot>/odom (slam_toolbox)
                /<robot>/mapping/cartographer/map, /tf_static map -> <robot>/cartographer/map
    Services    /<robot>/mapping/save_maps
"""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from rbsf_dual_slam import launch_nodes
from rbsf_dual_slam.contract import check_platform, slam_arguments
from rbsf_dual_slam.frames import slam_names

PRIMARY, SECONDARY = 'slam_toolbox', 'cartographer'


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    platform = arg('platform')
    check_platform(platform)
    robot = arg('robot').strip('/')
    use_sim_time = arg('use_sim_time').lower() == 'true'
    primary = slam_names(robot, PRIMARY, primary=True)
    secondary = slam_names(robot, SECONDARY, primary=False)
    return [
        *launch_nodes.slam_toolbox(robot, platform, primary, use_sim_time),
        *launch_nodes.cartographer(robot, platform, secondary, use_sim_time),
        launch_nodes.overlay_tf(robot, SECONDARY, secondary),
        launch_nodes.map_saver(
            robot, platform, arg('track'),
            {PRIMARY: primary.map_topic, SECONDARY: secondary.map_topic}, PRIMARY,
            arg('output_dir')),
    ]


def generate_launch_description():
    # primary/map_saver are declared (shared contract) but fixed here.
    return LaunchDescription([*slam_arguments(), OpaqueFunction(function=launch_setup)])
