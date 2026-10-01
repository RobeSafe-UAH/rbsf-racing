"""
Pure Pursuit tracking of a closed centerline/raceline.

Runs inside a car's namespace (/r0, /opponents/or0, ...), so its topics are relative.

Subscribes  odom                 nav_msgs/Odometry (pose of base_link, the rear axle, in map)
Publishes   drive                ackermann_msgs/AckermannDriveStamped
            pure_pursuit/path    nav_msgs/Path (the waypoints, transient local)
            pure_pursuit/target  geometry_msgs/PointStamped (current lookahead point)

Algorithm, at `rate` Hz once odometry has arrived (and, if `start_at` is set, once that wall-clock
time has passed, so several cars can set off together):
    1. Find the waypoint nearest to the car.
    2. Walk forward along the path to the first waypoint at least L away, where
       L = clip(lookahead_min + lookahead_gain * speed, lookahead_max).
    3. Steer along the arc through it: delta = atan(wheelbase * 2 y / L^2), with y its lateral
       offset in the car frame, clipped to max_steering.
    4. Speed: the raceline profile at the nearest waypoint times speed_scale, or the constant
       `speed` when the file has no profile.
"""

import time

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
from rbsf_pure_pursuit.pure_pursuit import lookahead_index, nearest_index, steering_angle
from rbsf_worlds.paths import load_waypoints
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('lookahead_min', 0.8)
        self.declare_parameter('lookahead_gain', 0.25)
        self.declare_parameter('lookahead_max', 3.0)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering', 0.4189)
        self.declare_parameter('rate', 50.0)
        # Wall-clock time (s since the epoch) before which the car stays still; 0 = start now.
        self.declare_parameter('start_at', 0.0)

        waypoints_file = self.get_parameter('waypoints_file').value
        if not waypoints_file:
            raise ValueError('waypoints_file is not set')
        self.xy, self.speeds = load_waypoints(waypoints_file)
        self.get_logger().info(
            f'{len(self.xy)} waypoints from {waypoints_file} '
            f"({'raceline speed profile' if self.speeds is not None else 'constant speed'})")

        self.pose = None  # x, y, yaw
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.target_pub = self.create_publisher(PointStamped, 'pure_pursuit/target', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_publisher(Path, 'pure_pursuit/path', latched).publish(self.path_msg())
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.control)

    def path_msg(self):
        path = Path()
        path.header.frame_id = 'map'
        for x, y in self.xy:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, float(yaw))

    def control(self):
        if self.pose is None or time.time() < self.get_parameter('start_at').value:
            return
        x, y, yaw = self.pose
        position = np.array([x, y])
        nearest = nearest_index(self.xy, position)
        if self.speeds is not None:
            speed = float(self.speeds[nearest]) * self.get_parameter('speed_scale').value
        else:
            speed = self.get_parameter('speed').value
        lookahead = min(
            self.get_parameter('lookahead_min').value
            + self.get_parameter('lookahead_gain').value * speed,
            self.get_parameter('lookahead_max').value)
        target = self.xy[lookahead_index(self.xy, nearest, position, lookahead)]

        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.drive.speed = speed
        drive.drive.steering_angle = steering_angle(
            x, y, yaw, target, self.get_parameter('wheelbase').value,
            self.get_parameter('max_steering').value)
        self.drive_pub.publish(drive)

        point = PointStamped()
        point.header.stamp = drive.header.stamp
        point.header.frame_id = 'map'
        point.point.x, point.point.y = float(target[0]), float(target[1])
        self.target_pub.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
