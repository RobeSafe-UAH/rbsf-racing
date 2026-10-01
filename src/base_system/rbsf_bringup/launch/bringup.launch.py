"""
Real-car bringup: VESC, Hokuyo LiDAR, RF2O laser odometry, joystick and drive_gate, in /<robot>.

This is the platform side of platform:=real (docs/DESIGN_PHILOSOPHY.md §3.1). It always includes
rbsf_safety's drive_gate: there is no way to start the motor drivers without the gate.

    Subscribes  /<robot>/drive                 ackermann_msgs/AckermannDriveStamped
                                               (ackermann_to_vesc; only drive_gate publishes it)
    Publishes   /<robot>/scan                  sensor_msgs/LaserScan (frame <robot>/laser)
                /<robot>/odom                  nav_msgs/Odometry (RF2O laser odometry, frame
                                               <robot>/odom, child <robot>/base_link)
                /<robot>/vesc/odom             nav_msgs/Odometry (VESC wheel odometry,
                                               diagnostics only: no TF)
                /<robot>/joy                   sensor_msgs/Joy
                /<robot>/drive_gate/state      std_msgs/String
                /tf                            <robot>/odom -> <robot>/base_link (RF2O)
                /tf_static                     <robot>/base_link -> <robot>/laser
    Takes       /<robot>/drive_gate/auto_cmd   (from the controller slot, local or remote)

Devices: the VESC on /dev/sensors/vesc (udev rule on the host), the Hokuyo on Ethernet
(192.168.0.10) and the joystick on /dev/input/js0. Parameters: config/vesc_real.yaml,
config/urg_real.yaml, config/rf2o_real.yaml, rbsf_safety/config/drive_gate_real.yaml.

Odometry is LiDAR-first: `odom` and its TF come from RF2O (rf2o_laser_odometry, vendored in
src/base_system). The VESC's odometry (wheel speed + servo command) is too noisy to drive TF or
SLAM, so it is kept on vesc/odom for diagnostics only.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# base_link (rear axle) -> laser, as on the legacy car (x, y, z in m).
LASER_POSE = ('0.27', '0.0', '0.11')


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    # Checked here too (not only in drive_gate.launch.py) so no driver starts on a bad config.
    if arg('require_deadman').lower() != 'true':
        raise RuntimeError(
            'require_deadman:=false is not allowed on the real platform: autonomous motion on '
            'the real car always requires the deadman (docs/DESIGN_PHILOSOPHY.md §6.1)')
    robot = arg('robot').strip('/')
    base_frame, odom_frame, laser_frame = (f'{robot}/{f}' for f in ('base_link', 'odom', 'laser'))
    share = get_package_share_directory('rbsf_bringup')
    vesc_config = os.path.join(share, 'config', 'vesc_real.yaml')
    urg_config = os.path.join(share, 'config', 'urg_real.yaml')
    rf2o_config = os.path.join(share, 'config', 'rf2o_real.yaml')

    def node(package, executable, **kwargs):
        return Node(package=package, executable=executable, namespace=robot, output='screen',
                    **kwargs)

    return [
        node('vesc_driver', 'vesc_driver_node', parameters=[vesc_config, {
            'speed_max': float(arg('speed_max_erpm')),
            'speed_min': float(arg('speed_min_erpm')),
        }]),
        node('vesc_ackermann', 'ackermann_to_vesc_node', parameters=[vesc_config],
             remappings=[('ackermann_cmd', 'drive')]),
        node('vesc_ackermann', 'vesc_to_odom_node', parameters=[vesc_config, {
            'odom_frame': odom_frame, 'base_frame': base_frame, 'publish_tf': False}],
             remappings=[('odom', 'vesc/odom')]),
        node('rf2o_laser_odometry', 'rf2o_laser_odometry_node', name='rf2o_laser_odometry',
             parameters=[rf2o_config, {
                 'laser_scan_topic': 'scan', 'odom_topic': 'odom', 'publish_tf': True,
                 'odom_frame_id': odom_frame, 'base_frame_id': base_frame}]),
        node('urg_node', 'urg_node_driver', parameters=[urg_config, {
            'laser_frame_id': laser_frame}]),
        node('tf2_ros', 'static_transform_publisher', arguments=[
            '--x', LASER_POSE[0], '--y', LASER_POSE[1], '--z', LASER_POSE[2],
            '--frame-id', base_frame, '--child-frame-id', laser_frame]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rbsf_safety'), 'launch', 'drive_gate.launch.py')),
            launch_arguments={'robot': robot, 'platform': 'real',
                              'require_deadman': arg('require_deadman')}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'speed_max_erpm', default_value='23250.0', description='VESC speed limit (ERPM).'),
        DeclareLaunchArgument(
            'speed_min_erpm', default_value='-23250.0', description='VESC speed limit (ERPM).'),
        DeclareLaunchArgument(
            'require_deadman', default_value='true',
            description='Must be true on the real car (false is an error).'),
        OpaqueFunction(function=launch_setup),
    ])
