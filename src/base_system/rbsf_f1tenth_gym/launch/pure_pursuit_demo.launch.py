"""
Demo: every car follows the map's raceline/centerline with Pure Pursuit.

Spawns the ego (/<robot>) and `num_opponents` opponents (/opponents/or<i>) on the reference
line, `spacing` metres apart along it (ego last, opponents ahead of it), each heading along the
line. Then starts one rbsf_pure_pursuit node per car in its namespace (odom -> drive), on top of
f1tenth_gym.launch.py (see it for the full topic list).

The reference line is <map>_raceline.csv when the map has one (speed profile x speed_scale), else
its centerline (constant `speed`). With ego_driver:=false the ego is left to you
(keyboard teleop or your own stack on /<robot>/drive); opponents will lap it if it stops.
"""

import os
import pathlib
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rbsf_f1tenth_gym.naming import opponent_namespace
from rbsf_worlds.paths import arc_lengths, load_waypoints, spawn_poses
from rbsf_worlds.tracks import find_waypoints, resolve_map_yaml
import yaml


def launch_setup(context):
    share = get_package_share_directory('rbsf_f1tenth_gym')
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    robot = arg('robot')
    num_opponents = int(arg('num_opponents'))
    spacing = float(arg('spacing'))

    base_config = os.path.join(share, 'config', 'f1tenth_gym.yaml')
    with open(base_config) as f:
        config = yaml.safe_load(f)
    params = config['f1tenth_gym_bridge']['ros__parameters']
    map_yaml = resolve_map_yaml(
        arg('track') or params['map_path'], os.environ.get('RBSF_GYM_PYTHON'))
    params['map_path'] = str(map_yaml)
    waypoints = arg('waypoints') or str(find_waypoints(map_yaml, arg('line')))

    # Car k (0 = ego, last) starts k gaps ahead along the line; the bridge's pose k is sx<k>.
    xy, _ = load_waypoints(waypoints)
    num_cars = num_opponents + 1
    _, length = arc_lengths(xy)
    params['num_agents'] = num_cars
    for k, (x, y, yaw) in enumerate(spawn_poses(xy, num_cars, spacing)):
        suffix = '' if k == 0 else str(k)
        params[f'sx{suffix}'], params[f'sy{suffix}'], params[f'stheta{suffix}'] = x, y, yaw
    demo_config = pathlib.Path(tempfile.mkdtemp(prefix='rbsf_pp_demo_')) / 'f1tenth_gym.yaml'
    with open(demo_config, 'w') as f:
        yaml.safe_dump(config, f)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'f1tenth_gym.launch.py')),
        launch_arguments={
            'config': str(demo_config),
            'spawn_on_line': 'false',  # poses already set above, on the chosen line
            'robot': robot,
            'rviz': LaunchConfiguration('rviz'),
            'foxglove': LaunchConfiguration('foxglove'),
        }.items(),
    )
    pure_pursuit_config = os.path.join(
        get_package_share_directory('rbsf_pure_pursuit'), 'config', 'pure_pursuit_sim.yaml')
    overrides = {
        'waypoints_file': waypoints,
        'speed': float(arg('speed')),
        'speed_scale': float(arg('speed_scale')),
        # Same wall-clock start for every car: one that sets off before the car ahead of it
        # closes the gap (and rear-ends it).
        'start_at': time.time() + float(arg('start_delay')),
    }
    namespaces = [opponent_namespace(i) for i in range(num_opponents)]
    if arg('ego_driver').lower() == 'true':
        namespaces.insert(0, robot)
    drivers = [
        Node(
            package='rbsf_pure_pursuit',
            executable='pure_pursuit',
            namespace=namespace,
            parameters=[pure_pursuit_config, overrides],
        )
        for namespace in namespaces
    ]
    return [
        LogInfo(msg=f'Pure Pursuit demo: {num_cars} cars, {spacing} m apart on {waypoints} '
                    f'({length:.1f} m)'),
        sim,
        *drivers,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='r0', description='Ego namespace.'),
        DeclareLaunchArgument(
            'num_opponents', default_value='3', description='Number of opponents.'),
        DeclareLaunchArgument(
            'spacing', default_value='5.0', description='Gap between cars along the line (m).'),
        DeclareLaunchArgument(
            'track', default_value='',
            description='Track or map (see f1tenth_gym.launch.py). Empty takes it from the '
                        'config.'),
        DeclareLaunchArgument(
            'line', default_value='auto',
            description="Reference line: 'raceline', 'centerline' or 'auto' (raceline if any)."),
        DeclareLaunchArgument(
            'waypoints', default_value='', description='Waypoint CSV; overrides map and line.'),
        DeclareLaunchArgument(
            'speed', default_value='2.0', description='Constant speed on a centerline (m/s).'),
        DeclareLaunchArgument(
            'speed_scale', default_value='0.6', description='Scale of the raceline speeds.'),
        DeclareLaunchArgument(
            'start_delay', default_value='5.0',
            description='Seconds after launch before the cars start driving (all at once).'),
        DeclareLaunchArgument(
            'ego_driver', default_value='true',
            description='Drive the ego with Pure Pursuit too.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument(
            'foxglove', default_value='true', description='Start the Foxglove bridge.'),
        OpaqueFunction(function=launch_setup),
    ])
