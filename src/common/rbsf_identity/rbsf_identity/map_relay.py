"""
Identity mapping on the simulators: the simulator's ground-truth map becomes /map.

The mapping slot owns /map (docs/DESIGN_PHILOSOPHY.md §3.2). Simulators publish their map on
/ground_truth/map; without a mapper, that map is the map. Both topics are latched.

Subscribes  /ground_truth/map  nav_msgs/OccupancyGrid (transient local)
Publishes   /map               nav_msgs/OccupancyGrid (transient local, same message)
"""

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

LATCHED = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class MapRelay(Node):

    def __init__(self):
        super().__init__('map_relay')
        self.pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.create_subscription(OccupancyGrid, '/ground_truth/map', self.pub.publish, LATCHED)


def main(args=None):
    rclpy.init(args=args)
    node = MapRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
