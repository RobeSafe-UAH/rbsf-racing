from rbsf_path_tracking.launch_common import config_files


def test_track_tuning_is_loaded_on_top_when_it_exists(tmp_path):
    (tmp_path / 'tracks').mkdir()
    (tmp_path / 'tracks' / 'eps_sim.yaml').write_text('')
    base = str(tmp_path / 'map_sim.yaml')
    tuning = str(tmp_path / 'tracks' / 'eps_sim.yaml')
    assert config_files('map', 'mvsim', 'eps', str(tmp_path)) == [base, tuning]
    # A map yaml path selects the tuning by its name.
    assert config_files('map', 'f1tenth', '/maps/x/eps.yaml', str(tmp_path)) == [base, tuning]


def test_no_tuning_for_other_tracks_platforms_or_no_track(tmp_path):
    (tmp_path / 'tracks').mkdir()
    (tmp_path / 'tracks' / 'eps_sim.yaml').write_text('')
    assert config_files('pure_pursuit', 'mvsim', 'levine', str(tmp_path)) == \
        [str(tmp_path / 'pure_pursuit_sim.yaml')]
    assert config_files('pure_pursuit', 'real', 'eps', str(tmp_path)) == \
        [str(tmp_path / 'pure_pursuit_real.yaml')]
    assert config_files('pure_pursuit', 'mvsim', '', str(tmp_path)) == \
        [str(tmp_path / 'pure_pursuit_sim.yaml')]
