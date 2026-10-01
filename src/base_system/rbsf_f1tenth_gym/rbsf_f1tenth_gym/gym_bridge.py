"""
f1tenth_gym_ros bridge with opponents numbered from 0 and odometry in the `<ns>/odom` frame.

Runs the upstream bridge (f1tenth_gym_ros.gym_bridge) with three patches:

- Opponent suffix: upstream names opponents <opp_namespace>, <opp_namespace>2, ...; here they
  are <opp_namespace>0, <opp_namespace>1, ... See naming.py for the full topic layout.
- Odometry frame: upstream publishes `odom` and TF `map -> <ns>/base_link`. Here both use the
  `<ns>/odom` frame (TF `<ns>/odom -> <ns>/base_link`), so that `map -> <ns>/odom` belongs to
  the mapping slot (docs/DESIGN_PHILOSOPHY.md §3.1). The values do not change: the gym's
  odometry is ground truth with the odom origin at the map origin.
- Ground truth: every car also gets `/<ns>/ground_truth/odom` (frame map).

Needs the f1tenth_gym interpreter (RBSF_GYM_PYTHON); the launch file starts it with it.
"""

from rbsf_f1tenth_gym.naming import ground_truth_odom_topic, odom_frame


def opp_suffix(opp_index):
    """Return the suffix of opponent `opp_index` (upstream counts from 1) counting from 0."""
    return str(opp_index - 1)


class OdomParentBroadcaster:
    """TF broadcaster proxy that re-parents `map -> <ns>/base_link` to `<ns>/odom`."""

    def __init__(self, broadcaster):
        self.broadcaster = broadcaster

    def sendTransform(self, transform):  # noqa: N802 (tf2_ros API)
        if transform.header.frame_id == 'map':
            namespace = transform.child_frame_id.rpartition('/')[0]
            transform.header.frame_id = odom_frame(namespace)
        self.broadcaster.sendTransform(transform)


def patch_odom_frames(bridge_class):
    """Patch GymBridge: odom and its TF in `<ns>/odom`, plus ground_truth/odom per car."""
    from nav_msgs.msg import Odometry

    make_odom_msg = bridge_class._make_odom_msg
    publish_odom = bridge_class._publish_odom
    publish_transforms = bridge_class._publish_transforms

    def _make_odom_msg(self, ts, namespace, pose, speed):
        odom = make_odom_msg(self, ts, namespace, pose, speed)
        odom.header.frame_id = odom_frame(namespace)
        return odom

    def _publish_odom(self, ts):
        publish_odom(self, ts)
        if not hasattr(self, 'ground_truth_pubs'):
            self.ground_truth_pubs = {}
        cars = [(self.ego_namespace, self.ego_pose, self.ego_speed)] + [
            (opp.namespace, opp.pose, opp.speed) for opp in self.opps]
        for namespace, pose, speed in cars:
            if namespace not in self.ground_truth_pubs:
                self.ground_truth_pubs[namespace] = self.create_publisher(
                    Odometry, ground_truth_odom_topic(namespace), 10)
            odom = make_odom_msg(self, ts, namespace, pose, speed)  # upstream: frame map
            self.ground_truth_pubs[namespace].publish(odom)

    def _publish_transforms(self, ts):
        broadcaster = self.br
        self.br = OdomParentBroadcaster(broadcaster)
        try:
            publish_transforms(self, ts)
        finally:
            self.br = broadcaster

    bridge_class._make_odom_msg = _make_odom_msg
    bridge_class._publish_odom = _publish_odom
    bridge_class._publish_transforms = _publish_transforms


def main(args=None):
    # Imported here: it pulls in f1tenth_gym, which only the gym interpreter has.
    from f1tenth_gym_ros import gym_bridge as upstream

    upstream.opp_suffix = opp_suffix
    patch_odom_frames(upstream.GymBridge)
    upstream.main(args)


if __name__ == '__main__':
    main()
