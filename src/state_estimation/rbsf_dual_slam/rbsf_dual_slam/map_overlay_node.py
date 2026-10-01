"""
Align a secondary Cartographer map with the primary map, so the two overlay.

Cartographer anchors its map frame at the car's start pose, while SLAM Toolbox anchors at the
odometry origin. This node pairs the car's position in both maps at the same instants (every
`spacing` m of travel) and publishes the static map -> <secondary map frame> that best fits all
pairs (least squares, alignment.py), refreshed every `refit_every` new pairs. Residual drift
between the two SLAMs stays visible as local misalignment (logged as the fit's RMS). It does not
touch map -> <robot>/odom.

Subscribes  tracked_pose  geometry_msgs/PoseStamped (Cartographer, frame = secondary map frame)
            /tf           map -> <base_frame> (from the primary SLAM and the platform odometry)
Publishes   /tf_static    map -> <secondary map frame>

Parameters: base_frame (<robot>/base_link), spacing (0.5 m), refit_every (10).
"""

from collections import deque
import math

from geometry_msgs.msg import PoseStamped, TransformStamped
from rbsf_dual_slam.alignment import fit_rigid_2d, residual_rms
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

MIN_PAIRS = 5


class MapOverlay(Node):

    def __init__(self):
        super().__init__('map_overlay')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.spacing = self.declare_parameter('spacing', 0.5).value
        self.refit_every = self.declare_parameter('refit_every', 10).value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.pending = deque(maxlen=200)  # tracked poses TF does not cover yet
        self.source, self.target = [], []  # secondary-map and map positions
        self.since_fit = 0
        self.create_subscription(PoseStamped, 'tracked_pose', self.callback, 10)

    def callback(self, msg):
        # Cartographer extrapolates tracked_pose slightly ahead of TF: pair the newest pose
        # that TF already covers, and drop it with everything older.
        self.pending.append(msg)
        for i in range(len(self.pending) - 1, -1, -1):
            pose = self.pending[i]
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', self.base_frame, Time.from_msg(pose.header.stamp),
                    timeout=Duration(seconds=0.0))
            except TransformException:
                continue
            for _ in range(i + 1):
                self.pending.popleft()
            self.add_pair(pose, tf)
            return

    def add_pair(self, pose, tf):
        source = (pose.pose.position.x, pose.pose.position.y)
        target = (tf.transform.translation.x, tf.transform.translation.y)
        if self.source and math.dist(self.source[-1], source) < self.spacing:
            return
        self.source.append(source)
        self.target.append(target)
        self.since_fit += 1
        if len(self.source) >= MIN_PAIRS and (
                len(self.source) == MIN_PAIRS or self.since_fit >= self.refit_every):
            self.publish(pose.header)

    def publish(self, header):
        self.since_fit = 0
        x, y, yaw = fit_rigid_2d(self.source, self.target)
        out = TransformStamped()
        out.header.stamp = header.stamp
        out.header.frame_id = 'map'
        out.child_frame_id = header.frame_id
        out.transform.translation.x, out.transform.translation.y = x, y
        out.transform.rotation.z, out.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.broadcaster.sendTransform(out)
        rms = residual_rms(self.source, self.target, (x, y, yaw))
        self.get_logger().info(
            f'map -> {header.frame_id}: ({x:.2f}, {y:.2f}) m, {math.degrees(yaw):.1f} deg '
            f'from {len(self.source)} poses, RMS {rms:.2f} m', throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = MapOverlay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
