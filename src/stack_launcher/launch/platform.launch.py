"""
Platform side of one car: drivers or simulator, joystick and drive_gate.

On the car in deployment:=distributed this is all that runs; behaviours include it themselves
in deployment:=local. Either way, the platform side is the same (docs/DESIGN_PHILOSOPHY.md §5.2).

    platform:=real     rbsf_bringup/bringup.launch.py (VESC, Hokuyo, joystick, drive_gate)
    platform:=mvsim    rbsf_mvsim/mvsim.launch.py + rbsf_safety/drive_gate.launch.py
    platform:=f1tenth  rbsf_f1tenth_gym/f1tenth_gym.launch.py + rbsf_safety/drive_gate.launch.py

Every platform gives /<robot>/{scan,odom} and takes /<robot>/drive, which only drive_gate
publishes; commands go to /<robot>/drive_gate/auto_cmd. `odom` is always in the <robot>/odom frame
(TF <robot>/odom -> <robot>/base_link), but its producer depends on the platform: RF2O laser
odometry on real (LiDAR-first; the VESC's odometry is too noisy), relabelled ground truth on the
simulators (RF2O does poorly in their feature-poor worlds), which also publish
/<robot>/ground_truth/odom and /ground_truth/map. map -> <robot>/odom belongs to the mapping
slot. The simulators are looked up at runtime (they are not dependencies: the robot image does
not ship them).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from stack_launcher.compose import include
from stack_launcher.registry import check_choice, PLATFORMS, USE_SIM_TIME


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    platform = arg('platform')
    check_choice('platform', platform, PLATFORMS)
    robot = arg('robot').strip('/')
    track = arg('track')

    if platform == 'real':
        return [include('rbsf_bringup', 'bringup.launch.py', {
            'robot': robot, 'require_deadman': arg('require_deadman')})]

    if platform == 'mvsim':
        sim = include('rbsf_mvsim', 'mvsim.launch.py', {
            'robot': robot, 'track': track,
            'headless': arg('headless'), 'foxglove': arg('foxglove')})
    else:
        sim = include('rbsf_f1tenth_gym', 'f1tenth_gym.launch.py', {
            'robot': robot, 'track': track, 'num_agents': 1,
            'rviz': arg('rviz'), 'foxglove': arg('foxglove')})
    gate = include('rbsf_safety', 'drive_gate.launch.py', {
        'robot': robot, 'platform': platform,
        'use_sim_time': str(USE_SIM_TIME[platform]).lower(), 'joy': arg('joy'),
        'require_deadman': arg('require_deadman')})
    return [sim, gate]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'platform', default_value='mvsim', description=f"One of: {', '.join(PLATFORMS)}."),
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'track', default_value='levine',
            description='rbsf_worlds track (simulators only).'),
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
