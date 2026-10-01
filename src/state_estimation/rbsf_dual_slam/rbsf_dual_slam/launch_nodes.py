"""Launch actions shared by the rbsf_dual_slam launch files."""

import os
import pathlib
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from rbsf_dual_slam.frames import cartographer_config, config_variant

SHARE = get_package_share_directory('rbsf_dual_slam')


def overlay_tf(robot, implementation, names):
    """
    Hang a secondary map frame below map, so the two maps overlay.

    SLAM Toolbox anchors its map at the odometry origin: identity. Cartographer anchors it at
    the car's start pose: map_overlay measures it from the first tracked_pose.
    """
    if implementation == 'cartographer':
        return Node(
            package='rbsf_dual_slam', executable='map_overlay', namespace=robot,
            parameters=[{'base_frame': names.base_frame}])
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name=f'{implementation}_map_overlay', namespace=robot,
        arguments=['--frame-id', 'map', '--child-frame-id', names.map_frame])


def slam_toolbox(robot, platform, names, use_sim_time):
    """async_slam_toolbox_node (a lifecycle node, configured and activated at startup)."""
    config = os.path.join(SHARE, 'config', f'slam_toolbox_{config_variant(platform)}.yaml')
    overrides = {'use_sim_time': use_sim_time, 'map_frame': names.map_frame,
                 'odom_frame': names.odom_frame, 'base_frame': names.base_frame,
                 'use_lifecycle_manager': False}
    if not names.publish_tf:
        overrides['transform_publish_period'] = 0.0
    node = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        namespace=robot, parameters=[config, overrides], output='screen',
        remappings=[('map', names.map_topic), ('map_metadata', f'{names.map_topic}_metadata')])
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    return [node, configure, activate]


def cartographer(robot, platform, names, use_sim_time):
    """cartographer_node + cartographer_occupancy_grid_node with a generated per-car config."""
    base = pathlib.Path(SHARE, 'config', f'cartographer_{config_variant(platform)}.lua')
    out_dir = pathlib.Path(tempfile.mkdtemp(prefix='rbsf_cartographer_'))
    (out_dir / 'cartographer.lua').write_text(cartographer_config(base.read_text(), names))
    return [
        LogInfo(msg=f'Cartographer config: {out_dir / "cartographer.lua"}'),
        Node(
            package='cartographer_ros', executable='cartographer_node', name='cartographer',
            namespace=robot, output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', str(out_dir),
                       '-configuration_basename', 'cartographer.lua']),
        Node(
            package='cartographer_ros', executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid', namespace=robot, output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            remappings=[('map', names.map_topic)]),
    ]


def map_saver(robot, platform, track, maps, primary, output_dir):
    """map_saver for {name: topic} maps."""
    return Node(
        package='rbsf_dual_slam', executable='map_saver', namespace=robot, output='screen',
        parameters=[{
            'output_dir': output_dir, 'maps': [f'{n}:{t}' for n, t in maps.items()],
            'platform': platform, 'track': track, 'primary': primary}])
