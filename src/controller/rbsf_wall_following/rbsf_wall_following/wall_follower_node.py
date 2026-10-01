"""
Reactive wall following: keep a reference distance to the left (or right) wall with a PI loop.

Runs inside a car's namespace (/r0, ...), so its topics are relative. It is the controller slot of
the stack (docs/DESIGN_PHILOSOPHY.md): its commands go to drive_gate, never straight to `drive`.

Subscribes  scan                  sensor_msgs/LaserScan
Publishes   drive_gate/auto_cmd   ackermann_msgs/AckermannDriveStamped (constant speed)

Algorithm, on every scan (see wall_following.py):
    1. dt from the scan stamps (scan_time on the first one).
    2. Median range around the lateral ray (90 deg) and the forward ray (60 deg) on the wall side.
    3. Distance error = lateral distance - reference_distance; orientation error = angle of the
       wall (line through both hits) relative to the heading.
    4. Steering = PI(distance error) + PI(orientation error), mirrored for a right wall and
       clipped to max_steering. Speed is the constant `speed`.
    5. Gaps: a wall-side hit farther than max_wall_distance means the wall is lost (doorway,
       side opening): skip the PI and hold the heading.
    6. Corners: as the front range drops from corner_distance to corner_min_distance, blend
       linearly into full lock, away from the wall side (wall present) or towards it (wall
       lost).
"""

import math

from ackermann_msgs.msg import AckermannDriveStamped
from rbsf_wall_following.wall_following import PI, WallFollower
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class WallFollowerNode(Node):

    def __init__(self):
        super().__init__('wall_follower')
        self.declare_parameter('wall_side', 'left')
        self.declare_parameter('reference_distance', 0.7)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('distance_kp', 1.0)
        self.declare_parameter('distance_ki', 0.0)
        self.declare_parameter('distance_integral_limit', 1.0)
        self.declare_parameter('orientation_kp', 1.0)
        self.declare_parameter('orientation_ki', 0.0)
        self.declare_parameter('orientation_integral_limit', 1.0)
        self.declare_parameter('lateral_angle', 90.0)
        self.declare_parameter('forward_angle', 60.0)
        self.declare_parameter('ray_window', 2.0)
        self.declare_parameter('max_steering', 0.4189)
        self.declare_parameter('max_wall_distance', 1.5)
        self.declare_parameter('corner_distance', 1.5)
        self.declare_parameter('corner_min_distance', 0.7)

        param = lambda name: self.get_parameter(name).value  # noqa: E731
        self.speed = float(param('speed'))
        self.follower = WallFollower(
            wall_side=param('wall_side'),
            reference=float(param('reference_distance')),
            lateral_angle=math.radians(param('lateral_angle')),
            forward_angle=math.radians(param('forward_angle')),
            window=math.radians(param('ray_window')),
            distance_pi=PI(param('distance_kp'), param('distance_ki'),
                           param('distance_integral_limit')),
            orientation_pi=PI(param('orientation_kp'), param('orientation_ki'),
                              param('orientation_integral_limit')),
            max_steering=float(param('max_steering')),
            max_wall_distance=float(param('max_wall_distance')),
            corner_distance=float(param('corner_distance')),
            corner_min_distance=float(param('corner_min_distance')),
        )
        self.get_logger().info(
            f"Following the {param('wall_side')} wall at {param('reference_distance')} m, "
            f'{self.speed} m/s')

        self.previous_stamp = None
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive_gate/auto_cmd', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

    def scan_callback(self, scan):
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        dt = scan.scan_time if self.previous_stamp is None else stamp - self.previous_stamp
        self.previous_stamp = stamp

        steering = self.follower.steering(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_max, dt)

        msg = AckermannDriveStamped()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = scan.header.frame_id
        msg.drive.speed = self.speed
        msg.drive.steering_angle = steering
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
