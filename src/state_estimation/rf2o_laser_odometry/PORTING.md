# ROS 2 Jazzy port

Vendored from [MAPIRlab/rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry),
branch `ros2`, commit `b38c68e`. The algorithm (`CLaserOdometry2D`) is unchanged. License stays
GPL-3.0 (see `LICENSE`); this package is separate from the Apache-2.0 rest of the repository.

Changes:

- `package.xml` uses format 3 and has the right dependencies. Upstream's `cmake_modules` key does
  not exist on Jazzy, which broke `rosdep install`, and `nav_msgs` was missing.
- CMake: C++17, Release by default, no unused Boost or library target.
- `CLaserOdometry2D` is no longer an `rclcpp::Node`. It logs through the owning node's logger, so a
  remapped `__node`/`__ns` no longer gives two nodes the same name. Run time is measured with
  `std::chrono::steady_clock`.
- Bugs: `laser_oldpose_` self-assignment in `init()` (now `laser_pose_`); speeds divided by zero on
  repeated stamps; all-zero initial quaternion when `init_pose_from_topic` is empty (now identity).
- Node: event-driven (odometry on every scan instead of a fixed-rate `spin_some` loop, so the `freq`
  parameter is gone); laser -> base TF retried until available instead of initializing with an
  empty transform; scans with a changed size are skipped; relative topic defaults (`scan`, `odom`)
  for namespaced use; sensor-data QoS; per-scan logs moved to DEBUG.
