"""
Reactive wall following: keep the car centred between the left and right walls with a PI loop.

Runs inside a car's namespace (/r0, ...), so its topics are relative. It is the controller slot of
the stack (docs/DESIGN_PHILOSOPHY.md): its commands go to drive_gate, never straight to `drive`.

Subscribes  scan                  sensor_msgs/LaserScan
Publishes   drive_gate/auto_cmd   ackermann_msgs/AckermannDriveStamped (constant speed)

Boundaries may be walls or rows of cones with gaps, so nothing comes from single rays: the
scan becomes points, and each measurement is the nearest point inside a strip (a strip longer
than the gaps always holds a cone).

Algorithm, on every scan (see wall_following.py):
    1. dt from the scan stamps (scan_time on the first one).
    2. Front: nearest point in the car's path (|y| <= path_half_width). Per side: nearest
       boundary in a bin_length strip beside the car (near) and lookahead m ahead (far). A side
       is visible while its near distance is within max_wall_distance.
    3. Both sides visible: distance error = (d_left - d_right) / 2, and the corridor width is
       remembered. One side visible: hold half the remembered width (younger than width_memory)
       to it, else fallback_distance. None: skip the PI and hold the heading.
    4. Orientation error = mean angle of the visible boundaries (near to far) relative to the
       heading. Steering = PI(distance error) + PI(orientation error), clipped to max_steering.
       Speed is the constant `speed`.
    5. Corners: as the front distance drops from corner_distance to corner_min_distance, blend
       linearly into full lock towards the side that is more open just before the obstacle.
With follow_side left/right, step 3 follows that side (the inner boundary of the lap) instead:
hold min(side_distance, half the corridor width) to it, and turn towards it once it is lost.
"""

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
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('distance_kp', 1.0)
        self.declare_parameter('distance_ki', 0.0)
        self.declare_parameter('distance_integral_limit', 1.0)
        self.declare_parameter('orientation_kp', 1.0)
        self.declare_parameter('orientation_ki', 0.0)
        self.declare_parameter('orientation_integral_limit', 1.0)
        self.declare_parameter('bin_length', 1.0)
        self.declare_parameter('lookahead', 1.2)
        self.declare_parameter('path_half_width', 0.3)
        self.declare_parameter('front_margin', 0.2)
        self.declare_parameter('min_points', 3)
        self.declare_parameter('max_steering', 0.4189)
        self.declare_parameter('max_wall_distance', 1.0)
        self.declare_parameter('fallback_distance', 0.4)
        self.declare_parameter('width_memory', 2.0)
        self.declare_parameter('corner_distance', 1.5)
        self.declare_parameter('corner_min_distance', 0.7)
        self.declare_parameter('follow_side', 'centre')
        self.declare_parameter('side_distance', 0.8)
        self.declare_parameter('side_capture_distance', 1.2)
        self.declare_parameter('wall_end_memory', 0.5)
        self.declare_parameter('wheelbase', 0.3302)

        param = lambda name: self.get_parameter(name).value  # noqa: E731
        self.speed = float(param('speed'))
        self.follower = WallFollower(
            distance_pi=PI(param('distance_kp'), param('distance_ki'),
                           param('distance_integral_limit')),
            orientation_pi=PI(param('orientation_kp'), param('orientation_ki'),
                              param('orientation_integral_limit')),
            max_steering=float(param('max_steering')),
            bin_length=float(param('bin_length')),
            lookahead=float(param('lookahead')),
            path_half_width=float(param('path_half_width')),
            front_margin=float(param('front_margin')),
            min_points=int(param('min_points')),
            max_wall_distance=float(param('max_wall_distance')),
            fallback_distance=float(param('fallback_distance')),
            width_memory=float(param('width_memory')),
            corner_distance=float(param('corner_distance')),
            corner_min_distance=float(param('corner_min_distance')),
            follow_side=str(param('follow_side')),
            side_distance=float(param('side_distance')),
            side_capture_distance=float(param('side_capture_distance')),
            wall_end_memory=float(param('wall_end_memory')),
            wheelbase=float(param('wheelbase')),
        )
        if self.follower.follow_side == 'centre':
            mode = f"Centring between the walls (fallback {param('fallback_distance')} m to a " \
                'lone wall)'
        else:
            mode = f"Following the {param('follow_side')} wall at up to " \
                f"{param('side_distance')} m"
        self.get_logger().info(f'{mode}, {self.speed} m/s')

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
