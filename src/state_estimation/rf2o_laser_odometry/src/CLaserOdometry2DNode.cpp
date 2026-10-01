/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es
*
* Modifications: Jeremie Deray & (see contributons on github)
* ROS 2 Jazzy port (RobeSafe, see PORTING.md): event-driven processing, relative topic
* defaults, laser TF retried until available, identity initial pose.
*
* Subscribes  laser_scan_topic (default "scan")        sensor_msgs/LaserScan
*             init_pose_from_topic (default disabled)  nav_msgs/Odometry (first message only)
* Publishes   odom_topic (default "odom")              nav_msgs/Odometry (odom_frame_id -> base_frame_id)
*             /tf                                      odom_frame_id -> base_frame_id (publish_tf)
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"

namespace rf2o {

CLaserOdometry2DNode::CLaserOdometry2DNode()
: Node("rf2o_laser_odometry"),
  rf2o_ref(get_logger().get_child("estimator"))
{
  RCLCPP_INFO(get_logger(), "Initializing RF2O node...");

  // Read Parameters. Topics are relative so the node works inside a namespace.
  //----------------
  laser_scan_topic = declare_parameter<std::string>("laser_scan_topic", "scan");
  odom_topic = declare_parameter<std::string>("odom_topic", "odom");
  base_frame_id = declare_parameter<std::string>("base_frame_id", "base_link");
  odom_frame_id = declare_parameter<std::string>("odom_frame_id", "odom");
  publish_tf = declare_parameter<bool>("publish_tf", true);
  init_pose_from_topic = declare_parameter<std::string>("init_pose_from_topic", "");

  // Init Publishers and Subscribers
  //---------------------------------
  buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  odom_pub = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
  laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(
    laser_scan_topic, rclcpp::SensorDataQoS(),
    std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));

  // Initialize pose
  initial_robot_pose.pose.pose.orientation.w = 1.0;  // identity (all-zero is not a rotation)
  if (!init_pose_from_topic.empty()) {
    initPose_sub = create_subscription<nav_msgs::msg::Odometry>(
      init_pose_from_topic, rclcpp::SensorDataQoS(),
      std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));
    GT_pose_initialized = false;
  } else {
    GT_pose_initialized = true;
  }

  // Init variables
  rf2o_ref.module_initialized = false;
  rf2o_ref.first_laser_scan = true;
}


/**
 * Estimates the odometry on every scan. On the first scan (once the laser -> base TF is
 * available) the estimator is initialized instead.
*/
void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (!GT_pose_initialized) {
    return;
  }
  rf2o_ref.current_scan_time = new_scan->header.stamp;

  if (rf2o_ref.first_laser_scan) {
    // Initialize module on first scan (from laser params). Retry until the TF exists.
    if (!setLaserPoseFromTf(new_scan->header.frame_id)) {
      return;
    }
    rf2o_ref.init(*new_scan, initial_robot_pose.pose.pose);
    rf2o_ref.first_laser_scan = false;
    return;
  }

  if (new_scan->ranges.size() != rf2o_ref.width) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Scan size changed (%zu, expected %u): skipping",
      new_scan->ranges.size(), rf2o_ref.width);
    return;
  }

  if (rf2o_ref.is_initialized() && rf2o_ref.odometryCalculation(*new_scan)) {
    publish();
  }
}


/**
 * Gets the laser pose with respect the base_link (through TF)
 * This allow estimation of the odometry with respect to the robot base reference system.
 */
bool CLaserOdometry2DNode::setLaserPoseFromTf(const std::string & laser_frame_id)
{
  geometry_msgs::msg::TransformStamped tf_laser;
  try {
    tf_laser = buffer_->lookupTransform(base_frame_id, laser_frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Waiting for TF %s -> %s: %s",
      base_frame_id.c_str(), laser_frame_id.c_str(), ex.what());
    return false;
  }

  // Keep this transform as Eigen Matrix3d
  tf2::Transform transform;
  tf2::convert(tf_laser.transform, transform);
  const tf2::Matrix3x3 & basis = transform.getBasis();
  Eigen::Matrix3d R;

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      R(r, c) = basis[r][c];
    }
  }

  Pose3d laser_tf(R);

  const tf2::Vector3 & t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  // Sets this transform in rf2o
  rf2o_ref.setLaserPose(laser_tf);
  return true;
}


/**
 * This function is used to initialize the robot pose before estimating its odometry.
 * By default the odometry will start from pose_0, but when comparing different methods
 * it may be necessary to start from a different pose.
*/
void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  // Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized) {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}


/**
 * Publish current odometry estimation over ROS
 * According to the node parameters it will publish over tf and/or especified topic
*/
void CLaserOdometry2DNode::publish()
{
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);

  // compose odom msg
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
  odom.header.frame_id = odom_frame_id;
  odom.child_frame_id = base_frame_id;
  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed;
  odom_pub->publish(odom);

  // Only one node may publish this transform.
  if (publish_tf) {
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header = odom.header;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quaternion;
    odom_broadcaster->sendTransform(odom_trans);
  }
}

}  // namespace rf2o


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rf2o::CLaserOdometry2DNode>());
  rclcpp::shutdown();
  return 0;
}
