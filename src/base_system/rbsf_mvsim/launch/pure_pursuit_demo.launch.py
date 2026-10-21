"""
Demo: every car follows the track's raceline/centerline with Pure Pursuit, on MVSim.

Same demo as rbsf_f1tenth_gym's pure_pursuit_demo.launch.py, with MVSim as the simulator: spawns
the ego (/<robot>) and `num_opponents` opponents (/opponents/or<i>) on the reference line,
`spacing` metres apart along it (ego last), and starts one rbsf_pure_pursuit node per car in its
namespace (ground_truth/odom -> drive). See mvsim.launch.py for the topics.

The reference line is <track>_raceline.csv when the track has one (speed profile x speed_scale),
else its centerline (constant `speed`). With ego_driver:=false the ego is left to you.
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
from rbsf_worlds.paths import arc_lengths, load_waypoints, spawn_poses
from rbsf_worlds.tracks import find_waypoints, resolve_map_yaml


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
    drivers = [
        Node(package='rbsf_pure_pursuit', executable='pure_pursuit', namespace=name,
             parameters=[pure_pursuit_config, overrides],
             remappings=[('odom', 'ground_truth/odom')])  # the line is in the map frame
        for name in (names if arg('ego_driver').lower() == 'true' else names[1:])
    ]
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
            'speed', default_value='2.0', description='Constant speed on a centerline (m/s).'),
        DeclareLaunchArgument(
            'speed_scale', default_value='0.6', description='Scale of the raceline speeds.'),
        DeclareLaunchArgument(
            'ego_driver', default_value='true',
            description='Drive the ego with Pure Pursuit too.'),
        DeclareLaunchArgument(
            'start_delay', default_value='5.0',
            description='Seconds after launch before the cars start driving (all at once).'),
        DeclareLaunchArgument(
            'headless', default_value='false', description='Run MVSim without its window.'),
        DeclareLaunchArgument(
            'foxglove', default_value='true', description='Start the Foxglove bridge.'),
        OpaqueFunction(function=launch_setup),
    ])
