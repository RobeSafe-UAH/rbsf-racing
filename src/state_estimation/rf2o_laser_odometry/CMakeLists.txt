cmake_minimum_required(VERSION 3.8)
project(rf2o_laser_odometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # The estimator is numeric-heavy; an unoptimized build cannot keep up with the LiDAR.
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(rf2o_laser_odometry_node
  src/CLaserOdometry2DNode.cpp
  src/CLaserOdometry2D.cpp)
target_include_directories(rf2o_laser_odometry_node PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(rf2o_laser_odometry_node
  geometry_msgs nav_msgs rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS rf2o_laser_odometry_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
