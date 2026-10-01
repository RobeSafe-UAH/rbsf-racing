"""
Canary for the f1tenth_gym_ros internals that rbsf_f1tenth_gym relies on.

gym_bridge.py replaces upstream's opp_suffix() and re-frames its odometry, and naming.py
remaps the global topics upstream builds with the suffix. If upstream renames or restructures
any of this, these checks fail before the simulation silently publishes under the wrong names.
"""

import importlib.util
import pathlib

import pytest


@pytest.fixture(scope='module')
def upstream_source():
    # find_spec does not execute the module, which needs f1tenth_gym.
    spec = importlib.util.find_spec('f1tenth_gym_ros.gym_bridge')
    assert spec is not None, 'f1tenth_gym_ros is not installed'
    return pathlib.Path(spec.origin).read_text()


@pytest.mark.parametrize('snippet', [
    'def opp_suffix(opp_index):',
    'for i in range(1, num_agents):',
    'suffix = opp_suffix(i)',
    'namespace = opp_namespace + suffix',
    'scan_topic=opp_scan_topic + suffix',
    'drive_topic=opp_drive_topic + suffix',
    "odom_in_ego_topic=self.ego_namespace + '/' + ego_opp_odom_topic + suffix",
    "'/goal_pose' + opp_suffix(i + 1)",
    "'/initialpose'",
    "'/cmd_vel'",
])
def test_upstream_naming_is_unchanged(upstream_source, snippet):
    assert snippet in upstream_source


# gym_bridge.patch_odom_frames() wraps these methods and relies on the map-frame values they set.
@pytest.mark.parametrize('snippet', [
    'def _make_odom_msg(self, ts, namespace, pose, speed):',
    "odom.header.frame_id = 'map'",
    'def _publish_odom(self, ts):',
    'def _publish_transforms(self, ts):',
    "ts_msg.header.frame_id = 'map'",
    "ts_msg.child_frame_id = namespace + '/base_link'",
    'self.br.sendTransform(ts_msg)',
    'self.ego_speed = ',
    'self.speed = ',
])
def test_upstream_odometry_is_unchanged(upstream_source, snippet):
    assert snippet in upstream_source
