"""
Demo: every car follows the track's raceline/centerline with Pure Pursuit (or MAP), on MVSim.

Same demo as rbsf_f1tenth_gym's pure_pursuit_demo.launch.py, with MVSim as the simulator: spawns
the ego (/<robot>) and `num_opponents` opponents (/opponents/or<i>) on the reference line,
`spacing` metres apart along it (ego last), and starts the same line follower per car in its
namespace (ground_truth/odom -> drive). See mvsim.launch.py for the topics.

The reference line is <track>_raceline.csv when the track has one, else its centerline; the cars
drive it at the constant `speed`. With ego_driver:=false the ego is left to you.
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
from rbsf_mvsim.scene import scenery
from rbsf_mvsim.world import opponent_namespace, outline_map, world_xml
from rbsf_path_tracking.launch_common import tracker_node
from rbsf_worlds.paths import arc_lengths, load_waypoints, spawn_poses
from rbsf_worlds.tracks import find_waypoints, resolve_map_yaml


def demo_driver(law, namespace, waypoints, overrides, track):
    """
    One car's line follower outside drive_gate: identity planning plus the path tracker.

    The tracker reads the ground truth (the line is in the map frame) and drives `drive` directly.
    """
    return [
        Node(package='rbsf_identity', executable='reference_line', namespace=namespace,
             parameters=[{'waypoints_file': waypoints}]),
        tracker_node(law, namespace, 'mvsim', track, overrides=overrides,
                     remappings=[('localization/odom', 'ground_truth/odom'),
                                 ('drive_gate/auto_cmd', 'drive')]),
    ]


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    robot = arg('robot').strip('/')
    num_opponents = int(arg('num_opponents'))
    spacing = float(arg('spacing'))

    map_yaml = resolve_map_yaml(arg('track'))
    waypoints = arg('waypoints') or str(find_waypoints(map_yaml, arg('line')))
    xy, _ = load_waypoints(waypoints)
    _, length = arc_lengths(xy)
    names = [robot] + [opponent_namespace(i) for i in range(num_opponents)]
    cars = [(name, x, y, yaw, 'ego' if k == 0 else 'opponent')
            for k, (name, (x, y, yaw)) in enumerate(
                zip(names, spawn_poses(xy, len(names), spacing)))]
    out_dir = pathlib.Path(tempfile.mkdtemp(prefix='rbsf_mvsim_pp_demo_'))
    world_file = out_dir / 'world.xml'
    grid = outline_map(map_yaml, out_dir)
    world_file.write_text(world_xml(grid, cars, follow=robot, work_dir=out_dir,
                                    scenery=scenery(grid, xy, out_dir)))

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rbsf_mvsim'), 'launch', 'mvsim.launch.py')),
        launch_arguments={
            'world_file': str(world_file),
            'robot': robot,
            'num_opponents': str(num_opponents),
            'headless': LaunchConfiguration('headless'),
            'foxglove': LaunchConfiguration('foxglove'),
        }.items(),
    )
    # Identity mapping for the ego: /map from the ground truth, map -> <robot>/odom identity.
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rbsf_identity'), 'launch', 'mapping.launch.py')),
        launch_arguments={'robot': robot, 'platform': 'mvsim'}.items(),
    )
    overrides = {
        'speed': float(arg('speed')),
        # Same wall-clock start for every car: one that sets off before the car ahead of it
        # closes the gap (and rear-ends it).
        'start_at': time.time() + float(arg('start_delay')),
    }
    drivers = []
    for name in (names if arg('ego_driver').lower() == 'true' else names[1:]):
        drivers += demo_driver(arg('controller'), name, waypoints, overrides, str(map_yaml))
    return [
        LogInfo(msg=f'Pure Pursuit demo (MVSim): {len(names)} cars, {spacing} m apart on '
                    f'{waypoints} ({length:.1f} m)'),
        sim,
        mapping,
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
            'track', default_value='levine',
            description='rbsf_worlds track (levine, spielberg, ...) or map yaml path.'),
        DeclareLaunchArgument(
            'line', default_value='auto',
            description="Reference line: 'raceline', 'centerline' or 'auto' (raceline if any)."),
        DeclareLaunchArgument(
            'waypoints', default_value='', description='Waypoint CSV; overrides track and line.'),
        DeclareLaunchArgument(
            'speed', default_value='2.0', description='Constant speed (m/s).'),
        DeclareLaunchArgument(
            'controller', default_value='pure_pursuit',
            description="Steering law: 'pure_pursuit' or 'map'."),
        DeclareLaunchArgument(
            'ego_driver', default_value='true',
            description='Drive the ego with the controller too.'),
        DeclareLaunchArgument(
            'start_delay', default_value='5.0',
            description='Seconds after launch before the cars start driving (all at once).'),
        DeclareLaunchArgument(
            'headless', default_value='false', description='Run MVSim without its window.'),
        DeclareLaunchArgument(
            'foxglove', default_value='true', description='Start the Foxglove bridge.'),
        OpaqueFunction(function=launch_setup),
    ])
