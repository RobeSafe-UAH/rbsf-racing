"""
Identity planning: the track's reference line is the trajectory.

Runs inside a car's namespace (/r0, ...). The launch file resolves `waypoints_file` from the
track (rbsf_worlds: its raceline, else its centerline, unless `line` says which). The line is
already a closed loop in the driving direction, in the map frame of the track's map.

Publishes   planning/trajectory  nav_msgs/Path (frame map, transient local; each pose heads to
                                 the next waypoint)

Without a waypoints file (no track, e.g. the real car) it publishes an empty Path: "no
trajectory", which controllers treat as "do not drive" (docs/DESIGN_PHILOSOPHY.md §3.3). The
raceline's speed profile is dropped: nav_msgs/Path cannot carry it (rbsf_msgs/Trajectory, §4.1).
"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rbsf_worlds.paths import headings, load_waypoints
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

LATCHED = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
MAP_FRAME = 'map'


def path_msg(xy, stamp):
    """nav_msgs/Path in the map frame through the (N, 2) waypoints `xy`."""
    path = Path()
    path.header.stamp = stamp
    path.header.frame_id = MAP_FRAME
    for (x, y), yaw in zip(xy, headings(xy) if len(xy) else []):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(pose)
    return path


class ReferenceLine(Node):

    def __init__(self):
        super().__init__('planning')
        self.declare_parameter('waypoints_file', '')
        waypoints_file = self.get_parameter('waypoints_file').value
        if waypoints_file:
            xy, _ = load_waypoints(waypoints_file)
            self.get_logger().info(f'Trajectory: {len(xy)} waypoints from {waypoints_file}')
        else:
            xy = []
            self.get_logger().warn('No reference line (no track): publishing an empty trajectory')
        self.pub = self.create_publisher(Path, 'planning/trajectory', LATCHED)
        self.pub.publish(path_msg(xy, self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = ReferenceLine()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
