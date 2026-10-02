"""
Adapter between one MVSim vehicle and the RBSF car interface (same topics as f1tenth_gym).

Runs inside the car's namespace (/r0, /opponents/or0, ...). mvsim.launch.py remaps MVSim's own
odom/scan/tf for the vehicle to mvsim/*, and this node republishes them in the stack convention:
TF frames prefixed with the namespace, and `odom` in the `<ns>/odom` frame. The simulator's
odometry is ground truth whose `<ns>/odom` origin is the map origin, so `map -> <ns>/odom` is
the identity unless a SLAM mapper estimates it (docs/DESIGN_PHILOSOPHY.md §3.1).

Subscribes  drive                    ackermann_msgs/AckermannDriveStamped
            base_pose_ground_truth   nav_msgs/Odometry (MVSim, pose in world coordinates)
            mvsim/scan               sensor_msgs/LaserScan (MVSim, frame 'scan')
Publishes   cmd_vel                  geometry_msgs/Twist (MVSim's vehicle controller input)
            odom                     nav_msgs/Odometry (frame <ns>/odom, child <ns>/base_link)
            ground_truth/odom        nav_msgs/Odometry (frame map, child <ns>/base_link)
            scan                     sensor_msgs/LaserScan (frame <ns>/laser)
            /tf                      <ns>/odom -> <ns>/base_link
            /tf_static               <ns>/base_link -> <ns>/laser

Also animates the car model (urdf/racecar_mesh.xacro of f1tenth_gym_ros, shown by the launch's
robot_state_publisher): /tf <ns>/<hinge> -> <ns>/<wheel>, as the f1tenth_gym bridge does. The
front wheels steer by the last commanded angle and every wheel rolls at the no-slip rate v / r.

Ackermann to Twist: MVSim's front-steer controller tracks (v, w), and the bicycle model with
wheelbase L gives w = v * tan(steering_angle) / L.
"""

import math

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


WHEEL_RADIUS = 0.051  # wheel meshes of the car model (f1tenth size)
WHEEL_JOINTS = (
    ('front_left_hinge', 'front_left_wheel', True),
    ('front_right_hinge', 'front_right_wheel', True),
    ('back_left_hinge', 'back_left_wheel', False),
    ('back_right_hinge', 'back_right_wheel', False),
)


def wheel_quaternion(steering, spin):
    """Return (x, y, z, w) of Rz(steering) @ Ry(spin): a front wheel steers, then rolls."""
    cz, sz = math.cos(steering / 2), math.sin(steering / 2)
    cy, sy = math.cos(spin / 2), math.sin(spin / 2)
    return (-sz * sy, cz * sy, sz * cy, cz * cy)


def ackermann_to_twist(speed, steering_angle, wheelbase):
    """Return (linear, angular) velocity of the bicycle model for an Ackermann command."""
    return speed, speed * math.tan(steering_angle) / wheelbase


class MvsimAdapter(Node):

    def __init__(self):
        super().__init__('mvsim_adapter')
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('laser_x', 0.27)
        self.declare_parameter('max_steering', 0.4189)  # f1tenth.vehicle.xml, 24 deg
        prefix = self.get_namespace().strip('/')
        self.base_frame = f'{prefix}/base_link' if prefix else 'base_link'
        self.laser_frame = f'{prefix}/laser' if prefix else 'laser'
        self.odom_frame = f'{prefix}/odom' if prefix else 'odom'

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.ground_truth_pub = self.create_publisher(Odometry, 'ground_truth/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_laser_tf()
        self.steering = 0.0
        self.wheel_angle = 0.0
        self.last_stamp = None

        self.create_subscription(AckermannDriveStamped, 'drive', self.drive_callback, 10)
        self.create_subscription(Odometry, 'base_pose_ground_truth', self.pose_callback, 10)
        self.create_subscription(
            LaserScan, 'mvsim/scan', self.scan_callback, qos_profile_sensor_data)

    def publish_laser_tf(self):
        self.static_broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.laser_frame
        tf.transform.translation.x = self.get_parameter('laser_x').value
        tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf)

    def drive_callback(self, msg):
        twist = Twist()
        twist.linear.x, twist.angular.z = ackermann_to_twist(
            msg.drive.speed, msg.drive.steering_angle, self.get_parameter('wheelbase').value)
        self.cmd_pub.publish(twist)
        limit = self.get_parameter('max_steering').value
        self.steering = max(-limit, min(limit, msg.drive.steering_angle))

    def pose_callback(self, msg):
        msg.header.frame_id = 'map'
        msg.child_frame_id = self.base_frame
        self.ground_truth_pub.publish(msg)
        msg.header.frame_id = self.odom_frame
        self.odom_pub.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)
        self.publish_wheel_tf(msg)

    def publish_wheel_tf(self, odom):
        stamp = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and stamp > self.last_stamp:
            speed = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
            self.wheel_angle += speed / WHEEL_RADIUS * (stamp - self.last_stamp)
        self.last_stamp = stamp
        prefix = self.base_frame.rsplit('/', 1)[0]
        transforms = []
        for hinge, wheel, steered in WHEEL_JOINTS:
            tf = TransformStamped()
            tf.header.stamp = odom.header.stamp
            tf.header.frame_id = f'{prefix}/{hinge}'
            tf.child_frame_id = f'{prefix}/{wheel}'
            q = wheel_quaternion(self.steering if steered else 0.0, self.wheel_angle)
            tf.transform.rotation.x, tf.transform.rotation.y = q[0], q[1]
            tf.transform.rotation.z, tf.transform.rotation.w = q[2], q[3]
            transforms.append(tf)
        self.tf_broadcaster.sendTransform(transforms)

    def scan_callback(self, msg):
        msg.header.frame_id = self.laser_frame
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MvsimAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
