"""
Car-side safety: joy_node and drive_gate in /<robot> (see rbsf_safety/drive_gate_node.py).

Included by every platform launch (rbsf_bringup on the real car, stack_launcher's
platform.launch.py on the simulators), so the gate always runs with the platform.

    Subscribes  /<robot>/drive_gate/auto_cmd   ackermann_msgs/AckermannDriveStamped
    Publishes   /<robot>/drive                 ackermann_msgs/AckermannDriveStamped
                /<robot>/drive_gate/state      std_msgs/String
                /<robot>/joy                   sensor_msgs/Joy (joy_node)

Parameters come from config/drive_gate_real.yaml on the real car and drive_gate_sim.yaml on the
simulators. If drive_gate exits (Ctrl-C, or a violated invariant), the whole launch shuts down.

require_deadman: on the simulators it may be true or false (default false: no joystick needed).
On the real car it MUST be true: the launch raises otherwise (and the node refuses it too).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PLATFORMS = ('real', 'mvsim', 'f1tenth')


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    platform = arg('platform')
    if platform not in PLATFORMS:
        raise RuntimeError(f"Unknown platform '{platform}'. Valid options: {list(PLATFORMS)}")
    robot = arg('robot').strip('/')
    config = arg('params_file') or os.path.join(
        get_package_share_directory('rbsf_safety'), 'config',
        f"drive_gate_{'real' if platform == 'real' else 'sim'}.yaml")
    require_deadman = arg('require_deadman').lower() or ('true' if platform == 'real' else 'false')
    if require_deadman not in ('true', 'false'):
        raise RuntimeError(f"require_deadman must be 'true' or 'false', not '{require_deadman}'")
    if platform == 'real' and require_deadman != 'true':
        raise RuntimeError(
            'require_deadman:=false is not allowed on the real platform: autonomous motion on '
            'the real car always requires the deadman (docs/DESIGN_PHILOSOPHY.md §6.1)')
    use_sim_time = {'use_sim_time': arg('use_sim_time').lower() == 'true'}
    actions = [Node(
        package='rbsf_safety',
        executable='drive_gate',
        namespace=robot,
        parameters=[config, {'platform': platform, 'require_deadman': require_deadman == 'true'},
                    use_sim_time],
        output='screen',
        on_exit=[Shutdown(reason='drive_gate exited')],
    )]
    if arg('joy').lower() == 'true':
        actions.append(Node(
            package='joy',
            executable='joy_node',
            namespace=robot,
            parameters=[config, use_sim_time],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'platform', default_value='real', description='real, mvsim or f1tenth.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('joy', default_value='true', description='Start joy_node.'),
        DeclareLaunchArgument(
            'require_deadman', default_value='',
            description='true/false. Default: false on simulators, true on real (where false '
                        'is an error).'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Parameter YAML; default picks _real/_sim from platform.'),
        OpaqueFunction(function=launch_setup),
    ])
