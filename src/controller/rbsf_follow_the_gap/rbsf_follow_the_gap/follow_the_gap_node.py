"""
Reactive Follow-the-Gap: drive into the largest free gap of the scan, around static obstacles.

Runs inside a car's namespace (/r0, ...), so its topics are relative. It is the controller slot of
the stack (docs/DESIGN_PHILOSOPHY.md): its commands go to drive_gate, never straight to `drive`.
It needs no map or pose, so it can drive an unknown track while the mapping slot builds the map
(ForzaETH maps with it too).

Subscribes  scan                       sensor_msgs/LaserScan
Publishes   drive_gate/auto_cmd        ackermann_msgs/AckermannDriveStamped
            follow_the_gap/scan        sensor_msgs/LaserScan (inflated ranges, publish_debug only)
            follow_the_gap/markers     visualization_msgs/MarkerArray (gap arc and target,
                                       publish_debug only)
Debug topics are filled only while someone subscribes.

Algorithm, on every scan (see follow_the_gap.py):
    1. Beams inside the field of view; no return is open space at max_range (the horizon).
    2. Moving-mean smoothing, never above the raw range.
    3. Inflate every point by asin(inflation_radius / r) on each side: gaps must fit the car,
       and cone rows close into walls. Next to a wall the radius shrinks below its distance.
    4. Free beams reach the gap threshold, which grows with the speed. The largest run of free
       beams is the gap, the one of the last target preferred (gap_hysteresis); with none, the
       deepest beams; with none of those either, the car stops.
    5. Target target_fraction of the way across the gap (0.5: the middle, more: towards the
       inner side of a counter-clockwise lap); pure-pursuit steering to it, low-passed and
       clipped.
    6. Speed from max_speed (straight) down to min_speed (full lock), capped to stop before the
       target, and never below min_speed while moving.
"""

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
import numpy as np
from rbsf_follow_the_gap.follow_the_gap import FollowTheGap
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

PARAMETERS = {
    'fov': 3.1416,
    'max_range': 10.0,
    'smoothing_window': 3,
    'inflation_radius': 0.25,
    'base_gap_threshold': 2.0,
    'speed_gain': 0.5,
    'max_gap_threshold': 4.0,
    'target_mode': 'middle',
    'deepest_fraction': 0.5,
    'lookahead': 2.0,
    'wheelbase': 0.33,
    'max_steering': 0.4189,
    'steering_alpha': 1.0,
    'max_speed': 2.0,
    'min_speed': 0.5,
    'max_decel': 3.0,
    'stop_distance': 0.3,
    'min_gap_threshold': 1.0,
    'fallback_ratio': 0.8,
    'target_fraction': 0.5,
    'gap_hysteresis': 1.0,
}

# Points of the debug gap arc (the arc is subsampled, not one point per beam).
ARC_POINTS = 32


class FollowTheGapNode(Node):

    def __init__(self):
        super().__init__('follow_the_gap')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('publish_debug', False)
        params = {name: self.get_parameter(name).value for name in PARAMETERS}
        self.ftg = FollowTheGap(**params)
        self.get_logger().info(
            f"Following the gap, {params['min_speed']}-{params['max_speed']} m/s, cars "
            f"{2 * params['inflation_radius']:.2f} m wide")

        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive_gate/auto_cmd', 10)
        self.scan_pub = self.marker_pub = None
        if self.get_parameter('publish_debug').value:
            self.scan_pub = self.create_publisher(LaserScan, 'follow_the_gap/scan', 1)
            self.marker_pub = self.create_publisher(MarkerArray, 'follow_the_gap/markers', 1)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

    def scan_callback(self, scan):
        cmd = self.ftg.update(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_max)

        msg = AckermannDriveStamped()
        msg.header = scan.header
        msg.drive.speed = cmd.speed
        msg.drive.steering_angle = cmd.steering
        self.drive_pub.publish(msg)

        if self.scan_pub is not None and self.scan_pub.get_subscription_count() > 0:
            self.publish_scan(scan)
        if self.marker_pub is not None and self.marker_pub.get_subscription_count() > 0:
            self.publish_markers(scan.header, cmd)

    def publish_scan(self, scan):
        layout = self.ftg.layout
        out = LaserScan()
        out.header = scan.header
        out.angle_min = float(layout.angles[0])
        out.angle_max = float(layout.angles[-1])
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = self.ftg.max_range
        out.ranges = self.ftg.inflated.tolist()
        self.scan_pub.publish(out)

    def publish_markers(self, header, cmd):
        arc = Marker(header=header, ns='gap', id=0, type=Marker.LINE_STRIP)
        target = Marker(header=header, ns='target', id=0, type=Marker.SPHERE)
        if cmd.target_angle is None:
            arc.action = target.action = Marker.DELETE
        else:
            angles = np.linspace(cmd.gap_start_angle, cmd.gap_end_angle, ARC_POINTS)
            arc.points = [Point(x=float(cmd.threshold * np.cos(a)),
                                y=float(cmd.threshold * np.sin(a))) for a in angles]
            arc.scale.x = 0.03
            arc.color.g = arc.color.a = 1.0
            target.pose.position.x = cmd.target_distance * float(np.cos(cmd.target_angle))
            target.pose.position.y = cmd.target_distance * float(np.sin(cmd.target_angle))
            target.pose.orientation.w = 1.0
            target.scale.x = target.scale.y = target.scale.z = 0.2
            target.color.b = target.color.g = target.color.a = 1.0
        self.marker_pub.publish(MarkerArray(markers=[arc, target]))


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
