#!/usr/bin/env bash
# Container entrypoint.
#
# Phase 1 (root, the container's default user):
#   - Remap the image user `roboracer` to HOST_UID/HOST_GID so files written to the
#     mounted workspace belong to the host user. Doing this at runtime instead of
#     at build time keeps every image layer host-independent (and cacheable).
#   - Give `roboracer` access to mounted hardware (VESC, LiDAR, joystick) whose group
#     IDs come from the host and may not exist in the image.
#   - Re-exec this script as `roboracer`.
# Phase 2 (roboracer):
#   - Source ROS and the workspace overlay, warn if the image is stale, build the
#     workspace when AUTO_BUILD=1, then exec the requested command.

set -eo pipefail
trap 'echo "❌ rbsf-entrypoint failed at line $LINENO: $BASH_COMMAND" >&2' ERR

WS=/ros2_ws
RBSF_USER=roboracer

if [ "$(id -u)" -eq 0 ]; then
  host_uid="${HOST_UID:-$(id -u "$RBSF_USER")}"
  host_gid="${HOST_GID:-$(id -g "$RBSF_USER")}"

  if [ "$(id -g "$RBSF_USER")" != "$host_gid" ]; then
    groupmod -o -g "$host_gid" "$RBSF_USER"
    chown -R "$RBSF_USER:$RBSF_USER" "/home/$RBSF_USER"
  fi
  if [ "$(id -u "$RBSF_USER")" != "$host_uid" ]; then
    # usermod also chowns the files in the home directory.
    usermod -o -u "$host_uid" "$RBSF_USER"
  fi

  for dev in /dev/ttyACM* /dev/ttyUSB* /dev/input/js* /dev/input/event* /dev/sensors/*; do
    [ -e "$dev" ] || continue
    gid="$(stat -L -c %g "$dev")"
    [ "$gid" -eq 0 ] && continue
    # getent exits 2 when the host group has no counterpart in the image.
    group="$(getent group "$gid" | cut -d: -f1 || true)"
    if [ -z "$group" ]; then
      group="hostdev$gid"
      groupadd -g "$gid" "$group"
    fi
    usermod -aG "$group" "$RBSF_USER"
  done

  export HOME="/home/$RBSF_USER" USER="$RBSF_USER"
  exec setpriv --reuid="$RBSF_USER" --regid="$RBSF_USER" --init-groups "$0" "$@"
fi

# shellcheck source=/dev/null
source /etc/rbsf/ros_env.sh

# The image bakes in rosdep and Python dependencies; warn if the mounted
# workspace declares different ones.
if [ -d "$WS/src" ]; then
  if ! diff -q <(cd "$WS" && find src -name package.xml -print0 | sort -z | xargs -0r sha256sum) \
               /etc/rbsf/manifests.sha256 > /dev/null; then
    echo "⚠️  package.xml files changed since the image was built. Run 'make build' to update dependencies."
  fi
fi
if [ -f "$WS/pyproject.toml" ] && ! (cd "$WS" && sha256sum --quiet -c /etc/rbsf/python.sha256 > /dev/null 2>&1); then
  echo "⚠️  pyproject.toml/uv.lock changed since the image was built. Run 'make build' to update dependencies."
fi

if [ "${AUTO_BUILD:-0}" = "1" ] && [ -n "$(find "$WS/src" -name package.xml -print -quit 2>/dev/null)" ]; then
  echo "🔨 Building workspace..."
  if (cd "$WS" && colcon build); then
    source /etc/rbsf/ros_env.sh
  else
    echo "❌ colcon build failed. Fix the errors and rebuild with 'cb'."
  fi
fi

echo "✅ RBSF Racing [${RBSF_FLAVOR}] · ROS ${ROS_DISTRO} · ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} · RMW=${RMW_IMPLEMENTATION:-default}"

exec "$@"
