"""
Identity localization: republish the platform odometry as the localization output.

Runs inside a car's namespace (/r0, ...). On the simulators `odom` is ground truth in the `map`
frame, so this is a valid localizer. On the real car `odom` is wheel odometry in `<ns>/odom`;
the relay still runs (reactive controllers do not need a map-frame pose) but warns about it.

Subscribes  odom               nav_msgs/Odometry
Publishes   localization/odom  nav_msgs/Odometry (same message, unchanged)
"""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class OdomRelay(Node):

    def __init__(self):
        super().__init__('localization')
        self.warned = False
        self.pub = self.create_publisher(Odometry, 'localization/odom', 10)
        self.create_subscription(Odometry, 'odom', self.callback, 10)

    def callback(self, msg):
        if not self.warned and msg.header.frame_id != 'map':
            self.get_logger().warn(
                f"odom is in '{msg.header.frame_id}', not 'map': localization/odom is not a "
                'map-frame pose (identity localization without a map)')
            self.warned = True
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
