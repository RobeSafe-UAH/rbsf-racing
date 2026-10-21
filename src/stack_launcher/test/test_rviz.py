import numpy as np
import pytest
from stack_launcher.rviz import adapt_config, CENTRE, VIEW

TEMPLATE = ('Value: /r0/planning/trajectory\nValue: /r0/pure_pursuit/target\n'
            + VIEW + CENTRE)


def test_namespaces_and_replacements():
    text = adapt_config(TEMPLATE, 'r1', {'/pure_pursuit/target': '/map_controller/target'})
    assert 'Value: /r1/planning/trajectory' in text
    assert 'Value: /r1/map_controller/target' in text
    assert '/r0/' not in text


def test_view_frames_the_waypoints():
    xy = np.array([[10.0, -2.0], [30.0, -2.0], [30.0, 8.0], [10.0, 8.0]])
    text = adapt_config(TEMPLATE, 'r0', xy=xy, window=(1300, 900))
    assert '      X: 20.00\n      Y: 3.00\n' in text
    assert f'      Scale: {0.85 * min(1000 / 20, 900 / 10):.1f}\n' in text


def test_view_missing_raises():
    with pytest.raises(ValueError):
        adapt_config('Value: /r0/scan\n', 'r0', xy=np.zeros((3, 2)))
