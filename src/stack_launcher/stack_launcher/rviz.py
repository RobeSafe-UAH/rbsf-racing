"""RViz configs for behaviours: a template written for /r0, adapted to the car and its track."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import numpy as np

# The template's view block (rviz_default_plugins/TopDownOrtho), rewritten to frame the track.
VIEW = '      Scale: 25\n      Target Frame: <Fixed Frame>\n'
CENTRE = '      X: 0\n      Y: 0\n'


def adapt_config(text, robot, replacements=None, xy=None, window=(1300, 900)):
    """
    Adapt template `text`: each `replacements` pair is applied, then /r0/ becomes /<robot>/.

    With waypoints `xy` (N x 2, map frame) the top-down view is centred on them and zoomed to fit.
    """
    for old, new in (replacements or {}).items():
        text = text.replace(old, new)
    text = text.replace('/r0/', f'/{robot}/')
    if xy is not None and len(xy):
        low, high = xy.min(axis=0), xy.max(axis=0)
        centre = (low + high) / 2.0
        # Pixels per metre; the displays panel takes about 300 px of the width.
        scale = 0.85 * min((window[0] - 300) / max(high[0] - low[0], 1.0),
                           window[1] / max(high[1] - low[1], 1.0))
        for old, new in ((VIEW, VIEW.replace('25', f'{scale:.1f}')),
                         (CENTRE, f'      X: {centre[0]:.2f}\n      Y: {centre[1]:.2f}\n')):
            if text.count(old) != 1:
                raise ValueError(f'RViz template has no single view entry {old!r}')
            text = text.replace(old, new)
    return text


def rviz_node(config, robot, replacements=None, xy=None, use_sim_time=False):
    """Return an RViz Node on config/rviz/`config`, adapted (adapt_config) into a temp file."""
    share = get_package_share_directory('stack_launcher')
    template = os.path.join(share, 'config', 'rviz', config)
    with open(template) as f:
        text = adapt_config(f.read(), robot, replacements, None if xy is None else np.asarray(xy))
    fd, path = tempfile.mkstemp(prefix=f'rbsf_{robot}_', suffix='.rviz')
    with os.fdopen(fd, 'w') as f:
        f.write(text)
    return Node(package='rviz2', executable='rviz2', arguments=['-d', path],
                parameters=[{'use_sim_time': use_sim_time}], output='log')
