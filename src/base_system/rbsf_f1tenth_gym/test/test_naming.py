from rbsf_f1tenth_gym.gym_bridge import opp_suffix
from rbsf_f1tenth_gym.naming import bridge_parameters, bridge_remappings, opponent_namespace


def test_opponents_are_numbered_from_zero():
    # Upstream counts opponents from 1.
    assert [opp_suffix(i) for i in (1, 2, 3)] == ['0', '1', '2']
    assert opponent_namespace(0) == 'opponents/or0'
    params = bridge_parameters('r0')
    assert params['opp_namespace'] + opp_suffix(1) == opponent_namespace(0)


def test_ego_names_follow_robot():
    params = bridge_parameters('r1')
    assert params['ego_namespace'] == 'r1'
    assert params['ego_scan_topic'] == 'r1/scan'
    assert params['ego_drive_topic'] == 'r1/drive'


def test_remappings_move_global_topics_into_namespaces():
    remappings = dict(bridge_remappings('r0', 2))
    assert remappings['/initialpose'] == '/r0/initialpose'
    assert remappings['/cmd_vel'] == '/r0/cmd_vel'
    assert remappings['/opp_scan1'] == '/opponents/or1/scan'
    assert remappings['/opp_drive0'] == '/opponents/or0/drive'
    assert remappings['/goal_pose1'] == '/opponents/or1/goal_pose'
    assert remappings['/r0/opp_odom0'] == '/r0/opponents/or0/odom'
    assert not any('or2' in target for target in remappings.values())
