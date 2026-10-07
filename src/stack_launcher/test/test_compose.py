from stack_launcher.compose import slot_arguments


def test_contract_arguments_reach_every_slot():
    args = slot_arguments('mapping', 'r0', 'real', '', False)
    assert args == {'robot': 'r0', 'platform': 'real', 'track': '', 'use_sim_time': 'false'}


def test_extra_arguments_reach_only_their_slot():
    extra = {'controller': {'params_file': 'x.yaml'}}
    assert slot_arguments('controller', 'r0', 'real', '', False, extra)['params_file'] == 'x.yaml'
    assert 'params_file' not in slot_arguments('mapping', 'r0', 'real', '', False, extra)
