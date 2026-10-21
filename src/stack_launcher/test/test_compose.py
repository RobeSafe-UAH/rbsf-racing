from stack_launcher.compose import slot_arguments


def test_contract_arguments_reach_every_slot():
    args = slot_arguments('mapping', 'r0', 'real', '', False)
    assert args == {'robot': 'r0', 'platform': 'real', 'track': '', 'use_sim_time': 'false'}


def test_extra_arguments_reach_only_their_slot():
    extra = {'controller': {'params_file': 'x.yaml'}}
    assert slot_arguments('controller', 'r0', 'real', '', False, extra)['params_file'] == 'x.yaml'
    assert 'params_file' not in slot_arguments('mapping', 'r0', 'real', '', False, extra)


def test_includes_see_only_their_own_arguments(tmp_path, monkeypatch):
    # A behaviour-level params_file must not reach an included file that declares one too.
    from launch import LaunchDescription, LaunchService
    from launch.actions import SetLaunchConfiguration
    from stack_launcher import compose

    seen = tmp_path / 'seen.txt'
    child = tmp_path / 'child.launch.py'
    child.write_text(
        'from launch import LaunchDescription\n'
        'from launch.actions import DeclareLaunchArgument, OpaqueFunction\n'
        'from launch.substitutions import LaunchConfiguration\n'
        'def record(context):\n'
        "    args = [LaunchConfiguration(n).perform(context) for n in ('robot', 'params_file')]\n"
        f"    open({str(seen)!r}, 'w').write(' '.join(args) + '|')\n"
        '    return []\n'
        'def generate_launch_description():\n'
        "    return LaunchDescription([DeclareLaunchArgument('robot'),\n"
        "                              DeclareLaunchArgument('params_file', default_value=''),\n"
        '                              OpaqueFunction(function=record)])\n')
    monkeypatch.setattr(compose, 'launch_file', lambda package, name: str(child))

    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        SetLaunchConfiguration('params_file', 'controller.yaml'),
        compose.include('pkg', 'child.launch.py', {'robot': 'r0'}),
    ]))
    assert service.run() == 0
    assert seen.read_text() == 'r0 |'
