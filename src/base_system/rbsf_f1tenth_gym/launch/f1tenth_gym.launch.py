"""
Launch the f1tenth_gym simulator: bridge, map server, robot models, Foxglove bridge and RViz.

Replaces f1tenth_gym_ros/gym_bridge_launch.py to apply the RBSF naming
(rbsf_f1tenth_gym/naming.py): the ego car lives under /<robot> (default r0) and opponent i under
/opponents/or<i>.

Ego (<robot> = r0):
    Subscribes  /r0/drive                        ackermann_msgs/AckermannDriveStamped
                /r0/initialpose                  geometry_msgs/PoseWithCovarianceStamped (reset)
                /r0/cmd_vel                      geometry_msgs/Twist (keyboard teleop)
    Publishes   /r0/scan                         sensor_msgs/LaserScan
                /r0/odom                         nav_msgs/Odometry
                /r0/collision                    std_msgs/Bool
                /r0/lap_count, /r0/lap_time      std_msgs/Int32, std_msgs/Float32
                /r0/opponents/or<i>/odom         nav_msgs/Odometry (opponent i, for the ego)
                /r0/robot_description            std_msgs/String
Opponent i (<ns> = opponents/or<i>):
    Subscribes  /<ns>/drive, /<ns>/goal_pose (geometry_msgs/PoseStamped, reset)
    Publishes   /<ns>/scan, /<ns>/odom, /<ns>/ego_odom, /<ns>/collision, /<ns>/lap_count,
                /<ns>/lap_time, /<ns>/robot_description
Global:
    /ground_truth/map (nav_msgs/OccupancyGrid; the mapping slot owns /map), /tf
    (<ns>/odom -> <ns>/base_link -> <ns>/laser), /tf_static (map -> opponents/or<i>/odom,
    identity: opponents are part of the world), /pause_sim (std_msgs/Bool), /clock (if
    use_sim_time_bridge).
Every car also publishes /<ns>/ground_truth/odom (frame map). Its odom is the same pose in the
<ns>/odom frame, whose origin is the map origin (see gym_bridge.py).

f1tenth_gym needs NumPy 2, so the image installs it in its own venv (RBSF_GYM_PYTHON). The bridge
is the only process started with that interpreter; everything else runs on the system one.
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
from rbsf_f1tenth_gym.naming import bridge_parameters, bridge_remappings, opponent_namespace
from rbsf_worlds.paths import load_waypoints, spawn_poses
from rbsf_worlds.tracks import find_waypoints, resolve_map_yaml
import yaml

EGO_ACCENT_COLOR = '0.3 0.57 1.0 1.0'
OPP_ACCENT_COLOR = '1.0 0.57 0.1 1.0'


def scaled_map_yaml(map_yaml, scale):
    """Write a copy of `map_yaml` scaled like the bridge scales the track, for map_server."""
    with open(map_yaml) as f:
        spec = yaml.safe_load(f)
    spec['resolution'] *= scale
    spec['origin'] = [spec['origin'][0] * scale, spec['origin'][1] * scale, spec['origin'][2]]
    spec['image'] = str((map_yaml.parent / spec['image']).resolve())
    out = pathlib.Path(tempfile.mkdtemp(prefix='rbsf_f1tenth_gym_')) / 'map.yaml'
    with open(out, 'w') as f:
        yaml.safe_dump(spec, f)
    return str(out)


def spawn_on_line(map_yaml, num_agents, spacing):
    """
    Bridge start poses (sx, sy, stheta, sx1, ...) along the track's reference line.

    Car 0 (the ego) at the start of the line, car k k gaps ahead. Empty when the map has no
    reference line: the config's poses are used then.
    """
    try:
        xy, _ = load_waypoints(find_waypoints(map_yaml))
    except FileNotFoundError:
        return {}
    poses = {}
    for k, (x, y, yaw) in enumerate(spawn_poses(xy, num_agents, spacing)):
        suffix = '' if k == 0 else str(k)
        poses.update({f'sx{suffix}': x, f'sy{suffix}': y, f'stheta{suffix}': yaw})
    return poses


def robot_state_publisher(namespace, accent_color, vehicle, use_sim_time):
    xacro_file = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'), 'urdf', 'racecar_mesh.xacro')
    # value_type=str: the URDF must not be yaml-parsed. Quoted: the colour has spaces.
    description = ParameterValue(Command([
        'xacro ', xacro_file, ' car_name:=', namespace, ' accent_color:="', accent_color, '"',
        ' vehicle:=', vehicle]), value_type=str)
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}],
    )


def opponent_odom_tf(namespace):
    """Identity map -> <ns>/odom for an opponent: its odometry is ground truth."""
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        namespace=namespace,
        arguments=['--frame-id', 'map', '--child-frame-id', f'{namespace}/odom'],
    )


def launch_setup(context):
    gym_python = os.environ.get('RBSF_GYM_PYTHON', '')
    if not os.access(gym_python, os.X_OK):
        raise RuntimeError(
            f"RBSF_GYM_PYTHON='{gym_python}' is not an executable interpreter. f1tenth_gym is "
            'only installed in the dev image (make build FLAVOR=dev).')

    share = get_package_share_directory('rbsf_f1tenth_gym')
    config = LaunchConfiguration('config').perform(context)
    with open(config) as f:
        params = yaml.safe_load(f)['f1tenth_gym_bridge']['ros__parameters']
    robot = LaunchConfiguration('robot').perform(context).strip('/')
    num_agents = int(LaunchConfiguration('num_agents').perform(context) or params['num_agents'])
    if num_agents < 1:
        raise RuntimeError(f'num_agents must be at least 1, got {num_agents}.')
    track = LaunchConfiguration('track').perform(context) or params['map_path']
    map_yaml = resolve_map_yaml(track, gym_python)
    use_sim_time = params.get('use_sim_time_bridge', False)
    vehicle = params.get('vehicle_params', 'f1tenth')
    start_poses = spawn_on_line(
        map_yaml, num_agents, float(LaunchConfiguration('spacing').perform(context))
    ) if LaunchConfiguration('spawn_on_line').perform(context).lower() == 'true' else {}

    bridge = Node(
        package='rbsf_f1tenth_gym',
        executable='gym_bridge',
        name='f1tenth_gym_bridge',
        prefix=[gym_python],
        parameters=[config, bridge_parameters(robot), {
            'num_agents': num_agents,
            'map_path': str(map_yaml),
            'use_sim_time': False,  # the bridge itself always runs on wall time
            **start_poses,
        }],
        remappings=bridge_remappings(robot, num_agents - 1),
        output='screen',
        on_exit=[Shutdown(reason='f1tenth_gym bridge exited')],
    )
    scaled_yaml = scaled_map_yaml(map_yaml, params.get('scale', 1.0))
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': scaled_yaml, 'topic_name': '/ground_truth/map',
                     'frame_id': 'map',
                     'use_sim_time': use_sim_time}],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{'autostart': True, 'node_names': ['map_server'],
                     'use_sim_time': use_sim_time}],
    )
    models = [robot_state_publisher(robot, EGO_ACCENT_COLOR, vehicle, use_sim_time)] + [
        robot_state_publisher(opponent_namespace(i), OPP_ACCENT_COLOR, vehicle, use_sim_time)
        for i in range(num_agents - 1)]

    port = LaunchConfiguration('foxglove_port').perform(context)
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'port': int(port)}],
        condition=IfCondition(LaunchConfiguration('foxglove')),
    )
    foxglove_url = ('https://app.foxglove.dev/?ds=foxglove-websocket'
                    f'&ds.url=ws://localhost:{port}')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(share, 'config', 'rviz', 'f1tenth_gym.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )
    opponent_odom_tfs = [opponent_odom_tf(opponent_namespace(i)) for i in range(num_agents - 1)]
    return [
        bridge, map_server, lifecycle_manager, *models, *opponent_odom_tfs, foxglove, rviz,
        LogInfo(msg=f'Foxglove: {foxglove_url} (layout: rbsf_f1tenth_gym/config/foxglove/)',
                condition=IfCondition(LaunchConfiguration('foxglove'))),
    ]


def generate_launch_description():
    share = get_package_share_directory('rbsf_f1tenth_gym')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=os.path.join(share, 'config', 'f1tenth_gym.yaml'),
            description='Bridge parameter file.'),
        DeclareLaunchArgument(
            'robot', default_value='r0', description='Namespace of the ego car (r0, r1, ...).'),
        DeclareLaunchArgument(
            'track', default_value='',
            description='rbsf_worlds track (levine, ...), map yaml path, f1tenth_gym_ros '
                        'path (maps/levine) or gym track name. Empty takes it from the config.'),
        DeclareLaunchArgument(
            'num_agents', default_value='',
            description='Number of cars (ego + opponents). Empty takes it from the config.'),
        DeclareLaunchArgument(
            'spawn_on_line', default_value='true',
            description="Place the cars on the track's reference line, `spacing` m apart (ego "
                        'last). false (or a map without a line) uses the config poses.'),
        DeclareLaunchArgument(
            'spacing', default_value='5.0', description='Gap between cars on the line (m).'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument(
            'foxglove', default_value='true', description='Start the Foxglove bridge.'),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765', description='Foxglove bridge port.'),
        OpaqueFunction(function=launch_setup),
    ])
