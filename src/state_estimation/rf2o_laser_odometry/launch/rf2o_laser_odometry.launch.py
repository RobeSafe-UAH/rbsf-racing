"""Standalone RF2O laser odometry (scan -> odom + TF odom -> base_link), for testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'laser_scan_topic': 'scan',
                'odom_topic': 'odom',
                'publish_tf': True,
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'init_pose_from_topic': '',
            }],
        ),
    ])
