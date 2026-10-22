"""
Behaviour: raceline tracking.

The car follows the track's reference line (legacy 03_centerline_tracking): planning publishes it
on /<robot>/planning/trajectory and the controller tracks it from the map-frame pose on
/<robot>/localization/odom.

Slots (defaults, each overridable with <slot>:=<implementation>):
    mapping       simulators: identity (/ground_truth/map, identity map -> <robot>/odom)
                  real:       map_server (the track's map on /map, no TF)
    localization  simulators: identity (ground truth in the map frame)
                  real:       particle_filter (RF2O odometry + LiDAR on the map; owns
                              map -> <robot>/odom)
    perception    identity
    planning      identity      (the track's raceline, else its centerline; line:= picks one)
    controller    pure_pursuit  (map: MAP, Model- and Acceleration-based Pursuit)

Quick start:
    ros2 launch stack_launcher raceline_tracking.launch.py track:=iros2026
    ros2 launch stack_launcher raceline_tracking.launch.py track:=iros2026 controller:=map
    ros2 launch stack_launcher raceline_tracking.launch.py platform:=f1tenth track:=spielberg
    # The real car's chain in simulation: known map + particle filter instead of ground truth.
    ros2 launch stack_launcher raceline_tracking.launch.py track:=eps_circuit \
        mapping:=map_server localization:=particle_filter
    # On the car (track:= is required: a track name or maps/<stamp>/track/<name>.yaml).
    ros2 launch stack_launcher raceline_tracking.launch.py platform:=real \
        deployment:=distributed track:=eps_circuit
Options: speed:=<m/s> (top speed of the profile), params_file:=<controller yaml>, line:=centerline,
initial_pose:=x,y,yaw|global (particle filter start; default the reference line's start, where the
car is placed; RViz's 2D Pose Estimate re-seeds it).
View it with rviz_slam (docker/ros_env.sh): reference line, lookahead target, driven path, car,
scan and the particles.
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
# The real car has no ground truth: it localizes on the track's known map.
PLATFORM_SLOTS = {'real': {'mapping': 'map_server', 'localization': 'particle_filter'}}


def default_slot(platform, slot):
    """Return the implementation of `slot` when the behaviour gets none on `platform`."""
    return PLATFORM_SLOTS.get(platform, {}).get(slot, DEFAULT_SLOTS[slot])


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    platform, deployment = arg('platform'), arg('deployment')
    slots = {slot: arg(slot) or default_slot(platform, slot) for slot in SLOTS}
    validate(platform, deployment, slots)
    robot = arg('robot').strip('/')
    track = resolve_track(platform, arg('track'))
    if not track:
        raise ValueError('raceline_tracking needs track:= on the real car: an rbsf_worlds track '
                         'or a session map, maps/<stamp>/track/<name>.yaml')

    actions = [LogInfo(msg=f'Raceline tracking on {platform} ({deployment}), /{robot}: '
                           + ', '.join(f'{slot}={name}' for slot, name in slots.items())
                           + f', track={track}')]
    if deployment == 'local':
        actions.append(include('stack_launcher', 'platform.launch.py', {
            'platform': platform, 'robot': robot, 'track': track, 'headless': arg('headless'),
            'rviz': 'false', 'foxglove': arg('foxglove'), 'joy': arg('joy'),
            'require_deadman': arg('require_deadman')}))
    extra = {'planning': {'line': arg('line')},
             'localization': {'initial_pose': arg('initial_pose')},
             'controller': {'speed': arg('speed'), 'params_file': arg('params_file')}}
    actions += include_slots(slots, robot, platform, track, USE_SIM_TIME[platform], extra)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'platform', default_value='mvsim',
            description=f"One of: {', '.join(PLATFORMS)}."),
        DeclareLaunchArgument(
            'deployment', default_value='local',
            description=f"One of: {', '.join(DEPLOYMENTS)}. distributed skips the platform "
                        'side (it runs on the car: platform.launch.py).'),
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'track', default_value='',
            description='rbsf_worlds track or map yaml. Default: levine on the simulators.'),
        *[DeclareLaunchArgument(
            slot, default_value='',
            description=f"{slot} implementation: {', '.join(IMPLEMENTATIONS[slot])}. Default: "
                        f'{DEFAULT_SLOTS[slot]}'
                        + ''.join(f', {impl} on {platform}' for platform, slots in
                                  PLATFORM_SLOTS.items() for s, impl in slots.items() if s == slot)
                        + '.')
          for slot in SLOTS],
        DeclareLaunchArgument(
            'line', default_value='auto',
            description="Planning reference line: 'raceline', 'centerline' or 'auto' (raceline "
                        'if the track has one).'),
        DeclareLaunchArgument(
            'initial_pose', default_value='',
            description="Particle filter start pose 'x,y,yaw' (map frame) or 'global'. Default: "
                        "the start of the track's reference line. RViz 2D Pose Estimate "
                        're-seeds.'),
        DeclareLaunchArgument(
            'speed', default_value='',
            description='Controller top speed (m/s): the speed profile slows down for corners. '
                        'Default: from its config.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Controller parameter YAML. Default: its _sim/_real config.'),
        DeclareLaunchArgument(
            'headless', default_value='false', description='MVSim without its window.'),
        DeclareLaunchArgument(
            'foxglove', default_value='true', description='Foxglove bridge (simulators).'),
        DeclareLaunchArgument(
            'joy', default_value='true', description='Start joy_node on the simulators.'),
        DeclareLaunchArgument(
            'require_deadman', default_value='',
            description='drive_gate deadman: default false on simulators.'),
        OpaqueFunction(function=launch_setup),
    ])
