"""
Car model for RViz on the real car: the f1tenth_gym URDF (meshes) in the car's frames.

The simulators publish this model themselves (rbsf_f1tenth_gym, rbsf_mvsim), with spinning
wheels. The real car does not, and cannot: the model lives in f1tenth_gym_ros, a simulator
package that is not in the robot image. So this runs on the visualisation side (the laptop),
and rviz_slam (docker/ros_env.sh) starts it only when nobody publishes the model yet.
f1tenth_gym_ros is looked up at runtime, never declared as a dependency.

    Publishes   /<robot>/robot_description   std_msgs/String (transient local)
                /<robot>/joint_states        sensor_msgs/JointState (wheel joints held at 0)
                /tf_static                   <robot>/base_link -> laser_model and wheel hinges
                /tf                          <robot>/<hinge> -> <robot>/<wheel> (still wheels)

Every frame hangs off <robot>/base_link, which the car's odometry already provides. The LiDAR
mesh is <robot>/laser_model, so the car's own <robot>/base_link -> <robot>/laser is untouched.

    ros2 launch stack_launcher robot_model.launch.py robot:=r0
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Same accent as the ego car in the simulators.
EGO_ACCENT_COLOR = '0.3 0.57 1.0 1.0'


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context).strip('/')
    try:
        share = get_package_share_directory('f1tenth_gym_ros')
    except PackageNotFoundError:
        raise RuntimeError('The car model needs f1tenth_gym_ros (dev image only)') from None
    # value_type=str: the URDF must not be yaml-parsed. Quoted: the colour has spaces.
    description = ParameterValue(Command([
        'xacro ', os.path.join(share, 'urdf', 'racecar_mesh.xacro'), ' car_name:=', robot,
        ' accent_color:="', EGO_ACCENT_COLOR, '"']), value_type=str)
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=robot, parameters=[{'robot_description': description}]),
        # Without joint states the continuous wheel joints have no TF and RViz flags them.
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             namespace=robot),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        OpaqueFunction(function=launch_setup),
    ])
