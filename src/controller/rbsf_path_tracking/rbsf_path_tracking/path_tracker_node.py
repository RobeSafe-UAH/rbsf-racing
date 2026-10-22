"""
Controller slot: track the planned closed trajectory with Pure Pursuit or MAP.

Runs inside a car's namespace (/r0, /opponents/or0, ...), so its topics are relative. The launch
file names the node after its steering law (pure_pursuit, map_controller), which prefixes the
debug topic.

Subscribes  localization/odom      nav_msgs/Odometry (pose of base_link, the rear axle, in map;
                                   twist in base_link)
            planning/trajectory    rbsf_msgs/Trajectory (closed loop in the driving direction,
                                   map frame, transient local; NaN speeds = no target)
Publishes   drive_gate/auto_cmd    ackermann_msgs/AckermannDriveStamped
            ~/target               geometry_msgs/PointStamped (current lookahead point)
            ~/trajectory           rbsf_msgs/Trajectory (the tracked line with the speed profile
                                   it drives, transient local)
            ~/speed_profile        visualization_msgs/Marker (that line as a closed strip coloured
                                   by speed, red at speed_color_min to green at speed_color_max,
                                   for RViz, transient local)

Algorithm, at `rate` Hz once a trajectory and fresh odometry (< odom_timeout s old) have arrived
(and, if `start_at` is set, once that wall-clock time has passed, so several cars can set off
together). Without them it publishes nothing, and drive_gate stops the car:
    0. On a new trajectory or new profile parameters, compute the speed profile
       (speed_profile.py): the curvature limit sqrt(max_lateral_acceleration / |k|), clipped by the
       planner's target speeds and [min_speed, speed], then acceleration and braking passes.
    1. Find the waypoint nearest to the car.
    2. Speed: the profile at the waypoint `speed_preview` seconds ahead at the measured speed
       (covers the drivetrain's lag when braking into a corner).
    3. Walk forward along the path to the first waypoint at least L away, where
       L = min(lookahead_min + lookahead_gain * speed, lookahead_max) at the commanded speed.
    4. Steer towards it with `steering_law` (tracking.py), clipped to max_steering:
       pure_pursuit  delta = atan(wheelbase * k), k = 2 y / L^2 the arc curvature
       map           delta = atan(wheelbase * k) + understeer_gradient * v^2 k, at the measured
                     speed v (Model- and Acceleration-based Pursuit)
"""

import time

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
import numpy as np
from rbsf_msgs.msg import Trajectory, Waypoint
from rbsf_path_tracking.speed_profile import speed_colors, speed_profile
from rbsf_path_tracking.tracking import (
    lookahead_distance, lookahead_index, nearest_index, steering_angle, STEERING_LAWS)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

PROFILE_PARAMETERS = ('speed', 'min_speed', 'max_lateral_acceleration', 'max_acceleration',
                      'max_deceleration', 'curvature_spacing')


class PathTrackerNode(Node):

    def __init__(self):
        super().__init__('path_tracker')
        self.declare_parameter('steering_law', 'pure_pursuit')
        # Speed profile: top speed (straights), floor, and the car's acceleration limits.
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_lateral_acceleration', 4.0)
        self.declare_parameter('max_acceleration', 3.0)
        self.declare_parameter('max_deceleration', 4.0)
        self.declare_parameter('curvature_spacing', 0.5)
        self.declare_parameter('speed_preview', 0.2)
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
        # RViz colour scale of ~/speed_profile (m/s): red at or below min, green at or above max.
        self.declare_parameter('speed_color_min', 1.0)
        self.declare_parameter('speed_color_max', 3.0)

        self.law = self.get_parameter('steering_law').value
        if self.law not in STEERING_LAWS:
            raise ValueError(
                f"Unknown steering_law '{self.law}'. Valid options: {', '.join(STEERING_LAWS)}")

        self.xy = None
        self.target_speeds = None
        self.frame = None
        self.profile = None
        self.profile_key = None
        self.pose = None  # x, y, yaw, measured speed
        self.odom_received = 0.0  # monotonic receive time
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive_gate/auto_cmd', 10)
        self.target_pub = self.create_publisher(PointStamped, '~/target', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.profile_pub = self.create_publisher(Trajectory, '~/trajectory', latched)
        self.marker_pub = self.create_publisher(Marker, '~/speed_profile', latched)
        self.create_subscription(
            Trajectory, 'planning/trajectory', self.trajectory_callback, latched)
        self.create_subscription(Odometry, 'localization/odom', self.odom_callback, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.control)
        self.get_logger().info(f'Steering law: {self.law}')

    def trajectory_callback(self, msg):
        if len(msg.waypoints) < 3:
            self.xy = None
            self.get_logger().warn(f'Trajectory has {len(msg.waypoints)} waypoints: not driving')
            return
        self.xy = np.array([(w.point.x, w.point.y) for w in msg.waypoints])
        self.target_speeds = np.array([w.speed for w in msg.waypoints])
        self.frame = msg.header.frame_id
        self.profile_key = None
        targets = np.count_nonzero(~np.isnan(self.target_speeds))
        self.get_logger().info(
            f'Trajectory: {len(self.xy)} waypoints in {self.frame}, {targets} with target speeds')

    def update_profile(self):
        """Recompute the speed profile if the trajectory or its parameters changed."""
        key = tuple(self.get_parameter(name).value for name in PROFILE_PARAMETERS)
        if key == self.profile_key:
            return self.profile is not None
        self.profile_key = key
        top, floor, lateral, acceleration, deceleration, spacing = key
        if min(top, floor, lateral, acceleration, deceleration, spacing) <= 0.0:
            self.profile = None
            self.get_logger().error(
                f'Speed profile parameters must be positive ({dict(zip(PROFILE_PARAMETERS, key))})'
                ': not driving')
            return False
        self.profile = speed_profile(self.xy, top, floor, lateral, acceleration, deceleration,
                                     spacing, self.target_speeds)
        self.get_logger().info(
            f'Speed profile: {self.profile.min():.2f} to {self.profile.max():.2f} m/s')

        trajectory = Trajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = self.frame
        for (x, y), speed in zip(self.xy, self.profile):
            waypoint = Waypoint(speed=float(speed))
            waypoint.point.x, waypoint.point.y = float(x), float(y)
            trajectory.waypoints.append(waypoint)
        self.profile_pub.publish(trajectory)
        self.publish_marker(trajectory.header)
        return True

    def publish_marker(self, header):
        """Publish the profile as a closed line strip coloured by speed."""
        slow = self.get_parameter('speed_color_min').value
        fast = self.get_parameter('speed_color_max').value
        if fast <= slow:
            self.get_logger().warn(
                f'speed_color_max ({fast}) must exceed speed_color_min ({slow}): no RViz colours')
            return
        marker = Marker(header=header, ns='speed_profile', type=Marker.LINE_STRIP)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.color.a = 1.0
        loop = np.append(np.arange(len(self.xy)), 0)  # back to the first waypoint
        for i, (r, g, b) in zip(loop, speed_colors(self.profile[loop], slow, fast)):
            point = Point(x=float(self.xy[i, 0]), y=float(self.xy[i, 1]), z=0.02)
            marker.points.append(point)
            marker.colors.append(ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0))
        self.marker_pub.publish(marker)

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
        if not self.update_profile():
            return
        param = lambda name: self.get_parameter(name).value  # noqa: E731
        x, y, yaw, measured_speed = self.pose
        position = np.array([x, y])
        nearest = nearest_index(self.xy, position)
        preview = max(measured_speed, 0.0) * param('speed_preview')
        speed = float(self.profile[lookahead_index(self.xy, nearest, position, preview)])
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
