"""
Identity mapping: the map frame is the odometry frame.

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2). Without a SLAM mapper,
mapping still owns /map and map -> <robot>/odom (§3.2):

    /tf_static  map -> <robot>/odom (identity). On the simulators odom is ground truth whose
                origin is the map origin, so this is exact. On the real car it means "map = odom".
    /map        relayed from the simulator's /ground_truth/map (map_relay). Not on real: there is
                no map to give.
"""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rbsf_identity.contract import contract_arguments


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context).strip('/')
    platform = LaunchConfiguration('platform').perform(context)
    actions = [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        namespace=robot,
        arguments=['--frame-id', 'map', '--child-frame-id', f'{robot}/odom'],
    )]
    if platform != 'real':
        actions.append(Node(package='rbsf_identity', executable='map_relay', namespace=robot))
    return actions


def generate_launch_description():
    return LaunchDescription([*contract_arguments(), OpaqueFunction(function=launch_setup)])
