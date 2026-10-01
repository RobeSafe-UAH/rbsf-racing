"""
Adapter between one MVSim vehicle and the RBSF car interface (same topics as f1tenth_gym).

Runs inside the car's namespace (/r0, /opponents/or0, ...). mvsim.launch.py remaps MVSim's own
odom/scan/tf for the vehicle to mvsim/*, and this node republishes them in the stack convention:
poses in the map frame, TF frames prefixed with the namespace.

Subscribes  drive                    ackermann_msgs/AckermannDriveStamped
            base_pose_ground_truth   nav_msgs/Odometry (MVSim, pose in world coordinates)
            mvsim/scan               sensor_msgs/LaserScan (MVSim, frame 'scan')
Publishes   cmd_vel                  geometry_msgs/Twist (MVSim's vehicle controller input)
            odom                     nav_msgs/Odometry (frame map, child <ns>/base_link)
            scan                     sensor_msgs/LaserScan (frame <ns>/laser)
            /tf                      map -> <ns>/base_link
            /tf_static               <ns>/base_link -> <ns>/laser

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


def ackermann_to_twist(speed, steering_angle, wheelbase):
    """Return (linear, angular) velocity of the bicycle model for an Ackermann command."""
    return speed, speed * math.tan(steering_angle) / wheelbase


class MvsimAdapter(Node):

    def __init__(self):
        super().__init__('mvsim_adapter')
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('laser_x', 0.27)
        prefix = self.get_namespace().strip('/')
        self.base_frame = f'{prefix}/base_link' if prefix else 'base_link'
        self.laser_frame = f'{prefix}/laser' if prefix else 'laser'

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_laser_tf()

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

    def pose_callback(self, msg):
        msg.header.frame_id = 'map'
        msg.child_frame_id = self.base_frame
        self.odom_pub.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

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
