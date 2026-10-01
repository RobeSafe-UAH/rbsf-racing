from stack_launcher.compose import slot_arguments


def test_contract_arguments_reach_every_slot():
    args = slot_arguments('mapping', 'r0', 'real', '', False)
    assert args == {'robot': 'r0', 'platform': 'real', 'track': '', 'use_sim_time': 'false'}


def test_extra_arguments_reach_only_their_slot():
    extra = {'controller': {'wall_side': 'right'}}
    assert slot_arguments('controller', 'r0', 'real', '', False, extra)['wall_side'] == 'right'
    assert 'wall_side' not in slot_arguments('mapping', 'r0', 'real', '', False, extra)
