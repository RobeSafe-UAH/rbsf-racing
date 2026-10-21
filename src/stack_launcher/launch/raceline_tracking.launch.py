"""
Behaviour: raceline tracking.

The car follows the track's reference line (legacy 03_centerline_tracking): planning publishes it
on /<robot>/planning/trajectory and the controller tracks it from the map-frame pose on
/<robot>/localization/odom.

Slots (defaults, each overridable with <slot>:=<implementation>):
    mapping       identity      (simulators: /ground_truth/map, identity map -> <robot>/odom)
    localization  identity      (simulators: ground truth in the map frame)
    perception    identity
    planning      identity      (the track's raceline, else its centerline; line:= picks one)
    controller    pure_pursuit  (map: MAP, Model- and Acceleration-based Pursuit)

Quick start (simulators only for now):
    ros2 launch stack_launcher raceline_tracking.launch.py track:=iros2026
    ros2 launch stack_launcher raceline_tracking.launch.py track:=iros2026 controller:=map
    ros2 launch stack_launcher raceline_tracking.launch.py platform:=f1tenth track:=spielberg
Options: speed:=<m/s> (constant speed), params_file:=<controller yaml>, line:=centerline.

platform:=real is refused: on the car the pose must be localized on the track's map, which needs
a known-map mapping (map_server) and a particle-filter localization (docs/DESIGN_PHILOSOPHY.md
§3.2). Neither exists yet.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from stack_launcher.compose import include, include_slots
from stack_launcher.registry import (
    DEPLOYMENTS, IMPLEMENTATIONS, PLATFORMS, resolve_track, SLOTS, USE_SIM_TIME, validate)

DEFAULT_SLOTS = {
    'mapping': 'identity',
    'localization': 'identity',
    'perception': 'identity',
    'planning': 'identity',
    'controller': 'pure_pursuit',
}


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    platform, deployment = arg('platform'), arg('deployment')
    slots = {slot: arg(slot) for slot in SLOTS}
    validate(platform, deployment, slots)
    if platform == 'real':
        raise ValueError(
            'raceline_tracking does not run on the real car yet: it needs a known-map mapping '
            '(map_server) and a particle-filter localization to place the car on the track.')
    robot = arg('robot').strip('/')
    track = resolve_track(platform, arg('track'))

    actions = [LogInfo(msg=f'Raceline tracking on {platform} ({deployment}), /{robot}: '
                           + ', '.join(f'{slot}={name}' for slot, name in slots.items())
                           + f', track={track}')]
    if deployment == 'local':
        actions.append(include('stack_launcher', 'platform.launch.py', {
            'platform': platform, 'robot': robot, 'track': track, 'headless': arg('headless'),
            'rviz': arg('rviz'), 'foxglove': arg('foxglove'), 'joy': arg('joy'),
            'require_deadman': arg('require_deadman')}))
    extra = {'planning': {'line': arg('line')},
             'controller': {'speed': arg('speed'), 'params_file': arg('params_file')}}
    actions += include_slots(slots, robot, platform, track, USE_SIM_TIME[platform], extra)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'platform', default_value='mvsim',
            description=f"One of: {', '.join(PLATFORMS)} (real is refused for now)."),
        DeclareLaunchArgument(
            'deployment', default_value='local',
            description=f"One of: {', '.join(DEPLOYMENTS)}. distributed skips the platform "
                        'side (it runs on the car: platform.launch.py).'),
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'track', default_value='',
            description='rbsf_worlds track or map yaml. Default: levine on the simulators.'),
        *[DeclareLaunchArgument(
            slot, default_value=DEFAULT_SLOTS[slot],
            description=f"{slot} implementation: {', '.join(IMPLEMENTATIONS[slot])}.")
          for slot in SLOTS],
        DeclareLaunchArgument(
            'line', default_value='auto',
            description="Planning reference line: 'raceline', 'centerline' or 'auto' (raceline "
                        'if the track has one).'),
        DeclareLaunchArgument(
            'speed', default_value='',
            description='Controller constant speed (m/s). Default: from its config.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Controller parameter YAML. Default: its _sim/_real config.'),
        DeclareLaunchArgument(
            'headless', default_value='false', description='MVSim without its window.'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz (f1tenth).'),
        DeclareLaunchArgument(
            'foxglove', default_value='true', description='Foxglove bridge (simulators).'),
        DeclareLaunchArgument(
            'joy', default_value='true', description='Start joy_node on the simulators.'),
        DeclareLaunchArgument(
            'require_deadman', default_value='',
            description='drive_gate deadman: default false on simulators.'),
        OpaqueFunction(function=launch_setup),
    ])
