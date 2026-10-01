# ROS environment for the RBSF Racing container.
# Sourced by the entrypoint and by every interactive shell (/etc/bash.bashrc).

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
