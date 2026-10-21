# ROS environment for the RBSF Racing container.
# Sourced by the entrypoint and by every interactive shell (/etc/bash.bashrc).

# The image's copy (/etc/rbsf) hands over to the mounted workspace's copy, so edits to this file
# (helpers like rviz_slam) apply without `make build`. It is still used without a mounted repo.
if [ "${BASH_SOURCE[0]}" = /etc/rbsf/ros_env.sh ] && [ -f /ros2_ws/docker/ros_env.sh ]; then
  source /ros2_ws/docker/ros_env.sh
  return
fi

RBSF_WS=/ros2_ws

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "$RBSF_WS/install/local_setup.bash" ]; then
  source "$RBSF_WS/install/local_setup.bash"
fi

if [[ $- == *i* ]]; then
  if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
  fi
fi

# Build the workspace (extra args go to colcon) and re-source the overlay.
cb() {
  (cd "$RBSF_WS" && colcon build "$@") && source "$RBSF_WS/install/local_setup.bash"
}

# RViz with both DualSLAM maps (SLAM Toolbox on /map, Cartographer overlay), the scan and the
# car model of r0, plus the plot_speed web plot of r0. The simulators publish the car model; on
# the real car nobody does, so it starts stack_launcher's robot_model.launch.py here. Everything
# it starts stops with RViz.
rviz_slam() {
  local plot= model=
  # Reuse a plot that is already serving (the port of plot_speed's speed.yaml).
  if (exec 3<>/dev/tcp/127.0.0.1/8090) 2>/dev/null; then
    echo "Speed plot already running on http://localhost:8090"
  else
    plot_speed &
    plot=$!
  fi
  if ! ros2 topic info /r0/robot_description 2>/dev/null | grep -q '^Publisher count: [1-9]'; then
    ros2 launch stack_launcher robot_model.launch.py robot:=r0 >/dev/null 2>&1 &
    model=$!
  fi
  rviz2 -d "$(ros2 pkg prefix rbsf_dual_slam)/share/rbsf_dual_slam/config/rviz/dual_slam.rviz" "$@"
  # $plot is the subshell running plot_speed; web_plot is its child. SIGINT is ignored by
  # background jobs in shells without job control, hence the SIGKILL fallback.
  if [ -n "$plot" ]; then
    pkill -INT -P "$plot" && sleep 1 && pkill -KILL -P "$plot"
  fi
  if [ -n "$model" ]; then
    # $model is ros2 launch itself: SIGINT makes it stop its nodes. If it is still up after 3 s
    # (SIGINT ignored, see above), kill its nodes and then it.
    kill -INT "$model" 2>/dev/null
    for _ in 1 2 3; do kill -0 "$model" 2>/dev/null && sleep 1; done
    if kill -0 "$model" 2>/dev/null; then
      pkill -KILL -P "$model"; kill -KILL "$model" 2>/dev/null
    fi
  fi
  return 0
}

# Live web plot of one car's speed (default r0) on http://localhost:8090 (rbsf_telemetry, config
# speed.yaml): controller command, what drive_gate sends, VESC measurement and odometry.
# Extra arguments go to the node, e.g. plot_speed r1 -p port:=8091
plot_speed() {
  local robot="${1:-r0}"
  shift
  # The executable itself, not `ros2 run`, so rviz_slam can stop it by PID.
  "$(ros2 pkg prefix rbsf_telemetry)/lib/rbsf_telemetry/web_plot" --ros-args -r "__ns:=/$robot" \
    --params-file "$(ros2 pkg prefix rbsf_telemetry)/share/rbsf_telemetry/config/speed.yaml" "$@"
}
