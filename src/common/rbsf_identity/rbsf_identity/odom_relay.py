"""
Identity localization: the platform odometry, expressed in the map frame.

Runs inside a car's namespace (/r0, ...). `odom` is in `<ns>/odom`, and whoever owns
`map -> <ns>/odom` (a SLAM mapper, or the identity mapping's static identity) places it in the
map: localization/odom = (map -> <ns>/odom) * odom. With a SLAM mapper this is a real map-frame
pose; with identity mapping it is the odometry itself (ground truth on the simulators).

Subscribes  odom               nav_msgs/Odometry (frame <ns>/odom)
            /tf, /tf_static    map -> <ns>/odom
Publishes   localization/odom  nav_msgs/Odometry (frame map, child unchanged; twist unchanged,
                               it is expressed in the child frame)

Messages that arrive before `map -> <ns>/odom` exists are dropped with a throttled warning.
"""

from nav_msgs.msg import Odometry
from rbsf_identity.pose_math import compose
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

MAP_FRAME = 'map'


class OdomRelay(Node):

    def __init__(self):
        super().__init__('localization')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Odometry, 'localization/odom', 10)
        self.create_subscription(Odometry, 'odom', self.callback, 10)

    def callback(self, msg):
        if msg.header.frame_id != MAP_FRAME:
            try:
                tf = self.tf_buffer.lookup_transform(MAP_FRAME, msg.header.frame_id, Time())
            except TransformException as error:
                self.get_logger().warn(
                    f'No {MAP_FRAME} -> {msg.header.frame_id} yet, dropping odom: {error}',
                    throttle_duration_sec=5.0)
                return
            t, r = tf.transform.translation, tf.transform.rotation
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            (x, y, z), (qx, qy, qz, qw) = compose(
                ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w)), ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
            p.x, p.y, p.z = x, y, z
            q.x, q.y, q.z, q.w = qx, qy, qz, qw
            msg.header.frame_id = MAP_FRAME
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
