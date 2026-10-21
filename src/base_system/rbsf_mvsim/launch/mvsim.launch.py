"""
Launch MVSim on an RBSF track with the same car interface as f1tenth_gym.launch.py.

Per car (<ns> = <robot> for the ego, opponents/or<i> for opponent i):
    Subscribes  /<ns>/drive                    ackermann_msgs/AckermannDriveStamped
                /<ns>/cmd_vel                  geometry_msgs/Twist (direct MVSim input, teleop)
    Publishes   /<ns>/scan                     sensor_msgs/LaserScan (frame <ns>/laser)
                /<ns>/odom                     nav_msgs/Odometry (<ns>/odom frame, child
                                               <ns>/base_link)
                /<ns>/ground_truth/odom        nav_msgs/Odometry (map frame, child <ns>/base_link)
                /<ns>/collision                std_msgs/Bool
                /<ns>/base_pose_ground_truth   nav_msgs/Odometry (MVSim)
                /<ns>/robot_description    std_msgs/String (f1tenth_gym car model, RViz/Foxglove)
Global:
    /ground_truth/map (nav_msgs/OccupancyGrid, the grid MVSim simulates; the mapping slot owns
    /map), /tf (<ns>/odom -> <ns>/base_link -> <ns>/laser), /tf_static (map ->
    opponents/or<i>/odom, identity: opponents are part of the world), /clock (MVSim).

mvsim_node runs every vehicle; its own odom/scan/tf topics are remapped to /<ns>/mvsim/* and one
rbsf_mvsim mvsim_adapter per car turns them into the topics above (see mvsim_adapter.py).

Without world_file, the world is generated from `track` with the ego alone, at the start of the
track's reference line. The track's cones (rbsf_worlds/cones.py) are drawn into the grid. With
scenery:=true (and a reference line) the world also gets the race-circuit look of
rbsf_mvsim/scene.py: grass, asphalt, red/white curbs on the walls and a finish line.
"""

import os
import pathlib
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rbsf_mvsim.scene import scenery
from rbsf_mvsim.world import opponent_namespace, outline_map, world_xml
from rbsf_worlds.cones import with_cones
from rbsf_worlds.paths import load_waypoints, pose_at
from rbsf_worlds.tracks import find_waypoints, resolve_map_yaml


def mvsim_remappings(namespaces):
    """Move each vehicle's MVSim odom/scan/tf under /<ns>/mvsim/; the grid is /ground_truth/map."""
    remappings = [('/simul_map', '/ground_truth/map'),
                  ('/simul_map_metadata', '/ground_truth/map_metadata')]
    for ns in namespaces:
        remappings += [(f'/{ns}/{topic}', f'/{ns}/mvsim/{topic}')
                       for topic in ('odom', 'scan', 'tf', 'tf_static')]
    return remappings


EGO_ACCENT_COLOR = '0.3 0.57 1.0 1.0'
OPP_ACCENT_COLOR = '1.0 0.57 0.1 1.0'


def robot_state_publisher(namespace, accent_color):
    """Publish the f1tenth_gym car model (same URDF and colours) for one car."""
    xacro_file = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'), 'urdf', 'racecar_mesh.xacro')
    # value_type=str: the URDF must not be yaml-parsed. Quoted: the colour has spaces.
    description = ParameterValue(Command([
        'xacro ', xacro_file, ' car_name:=', namespace, ' accent_color:="', accent_color, '"']),
        value_type=str)
    return Node(package='robot_state_publisher', executable='robot_state_publisher',
                namespace=namespace, parameters=[{'robot_description': description}])


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    robot = arg('robot').strip('/')
    namespaces = [robot] + [opponent_namespace(i) for i in range(int(arg('num_opponents')))]

    world_file = arg('world_file')
    if not world_file:
        map_yaml = resolve_map_yaml(arg('track'))
        try:
            xy = load_waypoints(find_waypoints(map_yaml))[0]
            x, y, yaw = pose_at(xy, 0.0)
        except FileNotFoundError:
            xy = None
            x = y = yaw = 0.0
        out_dir = pathlib.Path(tempfile.mkdtemp(prefix='rbsf_mvsim_'))
        world_file = str(out_dir / 'world.xml')
        grid = outline_map(with_cones(map_yaml, out_dir), out_dir)
        scene = (scenery(grid, xy, out_dir)
                 if xy is not None and arg('scenery').lower() == 'true' else [])
        with open(world_file, 'w') as f:
            f.write(world_xml(grid, [(robot, x, y, yaw, 'ego')],
                              follow=robot, work_dir=out_dir, scenery=scene))

    mvsim = Node(
        package='mvsim',
        executable='mvsim_node',
        parameters=[{
            'world_file': world_file,
            'headless': arg('headless').lower() == 'true',
            'force_publish_vehicle_namespace': True,
            # The adapters publish <ns>/odom -> <ns>/base_link; MVSim's per-vehicle tf is unused.
            'do_fake_localization': False,
            'publish_tf_odom2baselink': False,
        }],
        remappings=mvsim_remappings(namespaces),
        output='screen',
        on_exit=[Shutdown(reason='mvsim exited')],
    )
    adapter_config = os.path.join(
        get_package_share_directory('rbsf_mvsim'), 'config', 'mvsim_sim.yaml')
    adapters = [
        Node(package='rbsf_mvsim', executable='mvsim_adapter', namespace=ns,
             parameters=[adapter_config])
        for ns in namespaces
    ]
    # The ego's map -> <ns>/odom belongs to its mapping slot; opponents' odom is ground truth.
    opponent_odom_tfs = [
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
             namespace=ns, arguments=['--frame-id', 'map', '--child-frame-id', f'{ns}/odom'])
        for ns in namespaces[1:]
    ]
    models = [robot_state_publisher(ns, EGO_ACCENT_COLOR if ns == robot else OPP_ACCENT_COLOR)
              for ns in namespaces]
    port = arg('foxglove_port')
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'port': int(port)}],
        condition=IfCondition(LaunchConfiguration('foxglove')),
    )
    return [
        LogInfo(msg=f'MVSim world: {world_file}'),
        mvsim, *adapters, *models, *opponent_odom_tfs, foxglove,
        LogInfo(msg=f'Foxglove: https://app.foxglove.dev/?ds=foxglove-websocket'
                    f'&ds.url=ws://localhost:{port}',
                condition=IfCondition(LaunchConfiguration('foxglove'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'track', default_value='levine',
            description='rbsf_worlds track (levine, spielberg, ...) or map yaml path.'),
        DeclareLaunchArgument(
            'robot', default_value='r0', description='Namespace of the ego car (r0, r1, ...).'),
        DeclareLaunchArgument(
            'world_file', default_value='',
            description='MVSim world to load instead of generating one from the track.'),
        DeclareLaunchArgument(
            'num_opponents', default_value='0',
            description='Opponents in world_file (named opponents/or0, ...); one adapter each.'),
        DeclareLaunchArgument(
            'headless', default_value='false', description='Run MVSim without its window.'),
        DeclareLaunchArgument(
            'scenery', default_value='true',
            description='Race-circuit look (grass, curbs, finish line) for generated worlds.'),
        DeclareLaunchArgument(
            'foxglove', default_value='true', description='Start the Foxglove bridge.'),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765', description='Foxglove bridge port.'),
        OpaqueFunction(function=launch_setup),
    ])
