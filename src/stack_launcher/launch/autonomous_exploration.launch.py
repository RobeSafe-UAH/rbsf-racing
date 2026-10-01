"""
Behaviour: autonomous exploration.

The car drives on its own by following a wall while the mapping slot builds the map (legacy
02_autonomous_slam). By default the mapper is DualSLAM: SLAM Toolbox (owns /map and
map -> <robot>/odom) and Cartographer (on /<robot>/mapping/cartographer/map) side by side.

Slots (defaults, each overridable with <slot>:=<implementation>):
    mapping       dual_slam       (slam_toolbox, cartographer or identity also work)
    localization  identity
    perception    identity
    planning      identity
    controller    wall_following  (scan -> drive_gate/auto_cmd)

Quick start:
    ros2 launch stack_launcher autonomous_exploration.launch.py platform:=mvsim track:=levine
    ros2 launch stack_launcher autonomous_exploration.launch.py platform:=f1tenth
    ros2 launch stack_launcher autonomous_exploration.launch.py platform:=real
Save both maps (PGM + YAML for the offline map tools, plus each SLAM's own state) into
maps/<YYYYMMDD_HHMMSS>/ with joystick button Y, or:
    ros2 service call /r0/mapping/save_maps std_srvs/srv/Trigger
Distributed (same ROS_DOMAIN_ID on both machines):
    car:      ros2 launch stack_launcher platform.launch.py platform:=real
    compute:  ros2 launch stack_launcher autonomous_exploration.launch.py platform:=real \
                  deployment:=distributed

On the real car the wheels only turn while drive_gate allows it (docs/DESIGN_PHILOSOPHY.md §6):
arm with START, then hold RB (autonomous) or LB (manual, left stick + right stick).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from stack_launcher.compose import include, include_slots
from stack_launcher.registry import (
    DEPLOYMENTS, IMPLEMENTATIONS, PLATFORMS, SLOTS, USE_SIM_TIME, validate)

DEFAULT_SLOTS = {
    'mapping': 'dual_slam',
    'localization': 'identity',
    'perception': 'identity',
    'planning': 'identity',
    'controller': 'wall_following',
}


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    platform, deployment = arg('platform'), arg('deployment')
    slots = {slot: arg(slot) for slot in SLOTS}
    validate(platform, deployment, slots)
    robot = arg('robot').strip('/')
    track = arg('track')

    actions = [LogInfo(msg=f'Autonomous exploration on {platform} ({deployment}), /{robot}: '
                           + ', '.join(f'{slot}={name}' for slot, name in slots.items()))]
    if deployment == 'local':
        actions.append(include('stack_launcher', 'platform.launch.py', {
            'platform': platform, 'robot': robot, 'track': track, 'headless': arg('headless'),
            'rviz': arg('rviz'), 'foxglove': arg('foxglove'), 'joy': arg('joy'),
            'require_deadman': arg('require_deadman')}))
    actions += include_slots(slots, robot, platform, track, USE_SIM_TIME[platform])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'platform', default_value='mvsim', description=f"One of: {', '.join(PLATFORMS)}."),
        DeclareLaunchArgument(
            'deployment', default_value='local',
            description=f"One of: {', '.join(DEPLOYMENTS)}. distributed skips the platform "
                        'side (it runs on the car: platform.launch.py).'),
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'track', default_value='levine',
            description='rbsf_worlds track (simulators only).'),
        *[DeclareLaunchArgument(
            slot, default_value=DEFAULT_SLOTS[slot],
            description=f"{slot} implementation: {', '.join(IMPLEMENTATIONS[slot])}.")
          for slot in SLOTS],
        DeclareLaunchArgument(
            'headless', default_value='false', description='MVSim without its window.'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz (f1tenth).'),
        DeclareLaunchArgument(
            'foxglove', default_value='true', description='Foxglove bridge (simulators).'),
        DeclareLaunchArgument(
            'joy', default_value='true',
            description='Start joy_node on the simulators (always on for real).'),
        DeclareLaunchArgument(
            'require_deadman', default_value='',
            description='drive_gate deadman: default false on simulators, true on real (where '
                        'false is an error).'),
        OpaqueFunction(function=launch_setup),
    ])
