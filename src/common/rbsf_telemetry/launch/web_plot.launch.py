"""
Live web plot of one car's message fields (see rbsf_telemetry/web_plot_node.py).

Runs web_plot in /<robot> with config/<config>.yaml (default: speed, the controller command, the
command drive_gate sends, the VESC measurement and odometry). Open the printed URL in a browser.

    ros2 launch rbsf_telemetry web_plot.launch.py
    ros2 launch rbsf_telemetry web_plot.launch.py robot:=r1 port:=8091
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    config = os.path.join(
        get_package_share_directory('rbsf_telemetry'), 'config', arg('config') + '.yaml')
    if not os.path.isfile(config):
        raise RuntimeError(f"No web_plot config '{arg('config')}' ({config})")
    overrides = {name: int(arg(name)) if name == 'port' else arg(name)
                 for name in ('host', 'port') if arg(name)}
    return [Node(
        package='rbsf_telemetry',
        executable='web_plot',
        namespace=arg('robot').strip('/'),
        parameters=[config, overrides],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'config', default_value='speed', description='Config name in rbsf_telemetry/config.'),
        DeclareLaunchArgument('host', default_value='', description='Override the bind address.'),
        DeclareLaunchArgument('port', default_value='', description='Override the port.'),
        OpaqueFunction(function=launch_setup),
    ])
