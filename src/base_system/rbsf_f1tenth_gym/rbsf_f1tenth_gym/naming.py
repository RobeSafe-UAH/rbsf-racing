"""
Topic and namespace layout of the f1tenth_gym simulation.

The ego car lives under /<robot> (r0, r1, ... as the real cars) and opponent i under
/opponents/or<i>, numbered from 0. TF frames follow the namespaces (r0/base_link,
opponents/or0/laser, ...), and odometry is in each car's <ns>/odom frame.

f1tenth_gym_ros builds part of each opponent's names from a suffix (see gym_bridge.py in this
package, which numbers it from 0); the rest are global topics that bridge_remappings() moves into
the opponent's namespace.
"""

OPP_NAMESPACE_PREFIX = 'opponents/or'


def opponent_namespace(index):
    """Namespace (without leading slash) of opponent `index`, counted from 0."""
    return f'{OPP_NAMESPACE_PREFIX}{index}'


def odom_frame(namespace):
    """Odometry frame of the car in `namespace` (`<ns>/odom`)."""
    return f'{namespace}/odom'


def ground_truth_odom_topic(namespace):
    """Absolute ground-truth odometry topic of the car in `namespace`."""
    return f'/{namespace}/ground_truth/odom'


def bridge_parameters(robot):
    """Naming parameters of the f1tenth_gym_ros bridge for ego `robot`."""
    return {
        'ego_namespace': robot,
        'ego_scan_topic': f'{robot}/scan',
        'ego_drive_topic': f'{robot}/drive',
        'ego_odom_topic': 'odom',
        'ego_opp_odom_topic': 'opp_odom',
        'opp_namespace': OPP_NAMESPACE_PREFIX,
        'opp_scan_topic': 'opp_scan',
        'opp_drive_topic': 'opp_drive',
        'opp_odom_topic': 'odom',
        'opp_ego_odom_topic': 'ego_odom',
    }


def bridge_remappings(robot, num_opponents):
    """Remappings that move the bridge's global topics under the ego and opponent namespaces."""
    remappings = [
        ('/initialpose', f'/{robot}/initialpose'),
        ('/cmd_vel', f'/{robot}/cmd_vel'),
    ]
    for i in range(num_opponents):
        namespace = opponent_namespace(i)
        remappings += [
            (f'/opp_scan{i}', f'/{namespace}/scan'),
            (f'/opp_drive{i}', f'/{namespace}/drive'),
            (f'/goal_pose{i}', f'/{namespace}/goal_pose'),
            (f'/{robot}/opp_odom{i}', f'/{robot}/{namespace}/odom'),
        ]
    return remappings
