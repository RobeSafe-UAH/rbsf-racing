"""
Localization slot: particle filter (MCL) on the known map.

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2). Pair it with a known-map
mapping (mapping:=map_server): the filter, not the mapping, owns map -> <robot>/odom (§3.2).

    /<robot>/particle_filter  scan + odom + /map -> /tf map -> <robot>/odom, pf/pose, pf/viz/*
    /<robot>/localization     rbsf_identity odom_relay: odom composed with that TF, published on
                              /<robot>/localization/odom at the odometry rate

Parameters come from config/particle_filter_real.yaml on the real car (RF2O odometry) and
config/particle_filter_sim.yaml on the simulators. The particles start around initial_pose:
    ''              the start of the track's reference line (its raceline, else centerline):
                    where the simulators spawn the car, and the finish line on a car-mapped track
    x,y,yaw         that pose (map frame)
    global          spread over the whole map. It can lock onto the wrong part of a track with
                    look-alike corridors; RViz's 2D Pose Estimate (initialpose) re-seeds anytime.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rbsf_identity.contract import contract_arguments
from rbsf_worlds.paths import load_waypoints, pose_at
from rbsf_worlds.tracks import find_waypoints, resolve_map_yaml

PLATFORMS = ('real', 'mvsim', 'f1tenth')


def initial_pose(text, track):
    """
    Start pose [x, y, yaw] for the initial_pose argument, or [] for global initialization.

    Empty: the start of `track`'s reference line (global without a track or line).
    """
    text = text.replace(' ', '')
    if text == 'global' or not (text or track):
        return []
    if not text:
        try:
            xy = load_waypoints(find_waypoints(resolve_map_yaml(track)))[0]
        except FileNotFoundError:
            return []
        return list(pose_at(xy, 0.0))
    values = [float(v) for v in text.split(',')]
    if len(values) != 3:
        raise ValueError(f"initial_pose must be 'x,y,yaw', 'global' or empty, got '{text}'")
    return values


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    robot, platform = arg('robot').strip('/'), arg('platform')
    if platform not in PLATFORMS:
        raise RuntimeError(f"Unknown platform '{platform}'. Valid options: {list(PLATFORMS)}")
    kind = 'real' if platform == 'real' else 'sim'
    config = os.path.join(get_package_share_directory('particle_filter'), 'config',
                          f'particle_filter_{kind}.yaml')
    use_sim_time = arg('use_sim_time').lower() == 'true'
    overrides = {'use_sim_time': use_sim_time, 'base_frame': f'{robot}/base_link',
                 'odom_frame': f'{robot}/odom'}
    pose = initial_pose(arg('initial_pose'), arg('track'))
    if pose:
        overrides['initial_pose'] = pose
    return [
        Node(package='particle_filter', executable='particle_filter', name='particle_filter',
             namespace=robot, parameters=[config, overrides], output='screen'),
        Node(package='rbsf_identity', executable='odom_relay', namespace=robot,
             parameters=[{'use_sim_time': use_sim_time}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        *contract_arguments(),
        DeclareLaunchArgument(
            'initial_pose', default_value='',
            description="Start pose 'x,y,yaw' (map frame), 'global', or empty: the start of the "
                        "track's reference line."),
        OpaqueFunction(function=launch_setup),
    ])
