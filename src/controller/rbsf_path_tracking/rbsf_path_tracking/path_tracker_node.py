"""
Controller slot: track the planned closed trajectory with Pure Pursuit or MAP.

Runs inside a car's namespace (/r0, /opponents/or0, ...), so its topics are relative. The launch
file names the node after its steering law (pure_pursuit, map_controller), which prefixes the
debug topic.

Subscribes  localization/odom      nav_msgs/Odometry (pose of base_link, the rear axle, in map;
                                   twist in base_link)
            planning/trajectory    nav_msgs/Path (closed loop in the driving direction, map frame,
                                   transient local)
Publishes   drive_gate/auto_cmd    ackermann_msgs/AckermannDriveStamped
            ~/target               geometry_msgs/PointStamped (current lookahead point)

Algorithm, at `rate` Hz once a trajectory and fresh odometry (< odom_timeout s old) have arrived
(and, if `start_at` is set, once that wall-clock time has passed, so several cars can set off
together). Without them it publishes nothing, and drive_gate stops the car:
    1. Find the waypoint nearest to the car.
    2. Walk forward along the path to the first waypoint at least L away, where
       L = min(lookahead_min + lookahead_gain * speed, lookahead_max) at the commanded speed.
    3. Steer towards it with `steering_law` (tracking.py), clipped to max_steering:
       pure_pursuit  delta = atan(wheelbase * k), k = 2 y / L^2 the arc curvature
       map           delta = atan(wheelbase * k) + understeer_gradient * v^2 k, at the measured
                     speed v (Model- and Acceleration-based Pursuit)
    4. Speed: the constant `speed`. nav_msgs/Path carries no speed profile; it comes with
       rbsf_msgs/Trajectory (docs/DESIGN_PHILOSOPHY.md §4.1).
"""

import time

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
from rbsf_path_tracking.tracking import (
    lookahead_distance, lookahead_index, nearest_index, steering_angle, STEERING_LAWS)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class PathTrackerNode(Node):

    def __init__(self):
        super().__init__('path_tracker')
        self.declare_parameter('steering_law', 'pure_pursuit')
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('lookahead_min', 0.8)
        self.declare_parameter('lookahead_gain', 0.25)
        self.declare_parameter('lookahead_max', 3.0)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering', 0.4189)
        self.declare_parameter('understeer_gradient', 0.0)
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('odom_timeout', 0.5)
        # Wall-clock time (s since the epoch) before which the car stays still; 0 = start now.
        self.declare_parameter('start_at', 0.0)

        self.law = self.get_parameter('steering_law').value
        if self.law not in STEERING_LAWS:
            raise ValueError(
                f"Unknown steering_law '{self.law}'. Valid options: {', '.join(STEERING_LAWS)}")

        self.xy = None
        self.frame = None
        self.pose = None  # x, y, yaw, measured speed
        self.odom_received = 0.0  # monotonic receive time
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive_gate/auto_cmd', 10)
        self.target_pub = self.create_publisher(PointStamped, '~/target', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, 'planning/trajectory', self.path_callback, latched)
        self.create_subscription(Odometry, 'localization/odom', self.odom_callback, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.control)
        self.get_logger().info(f'Steering law: {self.law}')

    def path_callback(self, msg):
        if len(msg.poses) < 3:
            self.xy = None
            self.get_logger().warn(f'Trajectory has {len(msg.poses)} poses: not driving')
            return
        self.xy = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        self.frame = msg.header.frame_id
        self.get_logger().info(f'Trajectory: {len(self.xy)} poses in {self.frame}')

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, float(yaw),
                     msg.twist.twist.linear.x)
        self.odom_received = time.monotonic()
        if self.frame is not None and msg.header.frame_id != self.frame:
            self.get_logger().error(
                f'Odometry in {msg.header.frame_id}, trajectory in {self.frame}: not driving',
                throttle_duration_sec=5.0)
            self.pose = None

    def control(self):
        if self.xy is None or self.pose is None:
            return
        if time.monotonic() - self.odom_received > self.get_parameter('odom_timeout').value:
            self.get_logger().warn('Odometry is stale: not driving', throttle_duration_sec=5.0)
            return
        if time.time() < self.get_parameter('start_at').value:
            return
        param = lambda name: self.get_parameter(name).value  # noqa: E731
        x, y, yaw, measured_speed = self.pose
        position = np.array([x, y])
        speed = param('speed')
        nearest = nearest_index(self.xy, position)
        lookahead = lookahead_distance(
            speed, param('lookahead_min'), param('lookahead_gain'), param('lookahead_max'))
        target = self.xy[lookahead_index(self.xy, nearest, position, lookahead)]

        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.drive.speed = speed
        drive.drive.steering_angle = steering_angle(
            self.law, x, y, yaw, target, measured_speed, param('wheelbase'),
            param('max_steering'), param('understeer_gradient'))
        self.drive_pub.publish(drive)

        point = PointStamped()
        point.header.stamp = drive.header.stamp
        point.header.frame_id = self.frame
        point.point.x, point.point.y = float(target[0]), float(target[1])
        self.target_pub.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = PathTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
