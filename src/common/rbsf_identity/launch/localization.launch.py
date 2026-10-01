"""
Identity localization: /<robot>/odom placed in the map frame by TF (odom_relay).

Publishes /<robot>/localization/odom.

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2).
"""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rbsf_identity.contract import contract_arguments


def generate_launch_description():
    return LaunchDescription([
        *contract_arguments(),
        Node(
            package='rbsf_identity',
            executable='odom_relay',
            namespace=LaunchConfiguration('robot'),
            parameters=[{'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)}],
        ),
    ])
