# syntax=docker/dockerfile:1
#
# RBSF Racing images: ROS 2 Jazzy on Ubuntu 24.04.
#
# Two flavors share every layer up to the ROS stack dependencies:
#   dev    workstation: ROS desktop, RViz, MVSim and f1tenth_gym simulators
#   robot  on-board computer: ros-base + drivers, no GUI
#
# Caching rules followed here (keep them when editing):
#   - Nothing depends on the host user. The image ships a fixed user `roboracer` and the entrypoint
#     remaps its UID/GID at runtime, so any machine (or a registry) can reuse the same layers.
#   - Layers go from most stable to most volatile. Source code never reaches the image; only
#     package.xml files (rosdep) and pyproject.toml/uv.lock (Python) do, so editing code
#     never invalidates a layer.
#   - External inputs are pinned (ROS apt source, uv), and ARGs are declared right before the
#     layer that uses them.
#   - apt and uv downloads live in BuildKit cache mounts, so even a cache miss re-downloads little.

ARG BASE_IMAGE=ubuntu:24.04
ARG UV_VERSION=0.12.21
ARG FLAVOR=dev

FROM ghcr.io/astral-sh/uv:${UV_VERSION} AS uv

# ---------------------------------------------------------------------------
# base: OS, user, ROS core and build tools
# ---------------------------------------------------------------------------
FROM ${BASE_IMAGE} AS base

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TZ=Europe/Madrid \
    ROS_DISTRO=jazzy

# Keep downloaded .debs in the cache mount instead of deleting them after each install.
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
 && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        bash-completion \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        gnupg \
        less \
        nano \
        python3-dev \
        python3-pip \
        sudo \
        tzdata \
        vim \
 && ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Static, host-independent user. Ubuntu 24.04 images ship `ubuntu` (1000:1000); rename it
# instead of adding a second user that would collide on UID/GID 1000.
RUN usermod -l roboracer -d /home/roboracer -m -c roboracer ubuntu \
 && groupmod -n roboracer ubuntu \
 && groupadd -f input && groupadd -f plugdev \
 && usermod -aG dialout,video,input,plugdev roboracer \
 && echo 'roboracer ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/roboracer \
 && chmod 0440 /etc/sudoers.d/roboracer

ARG ROS_APT_SOURCE_VERSION=1.3.0
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    . /etc/os-release \
 && curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb" \
 && apt-get install -y /tmp/ros2-apt-source.deb \
 && rm /tmp/ros2-apt-source.deb \
 && apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        ros-dev-tools \
 && rosdep init

# ---------------------------------------------------------------------------
# stack: ROS packages the racing stack needs on every platform.
# Pre-installed here (rather than left to rosdep) so the heavy, well-known
# dependencies are cached independently of package.xml edits.
# ---------------------------------------------------------------------------
FROM base AS stack

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ackermann-msgs \
        ros-${ROS_DISTRO}-asio-cmake-module \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-laser-proc \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-serial-driver \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-urg-node \
        ros-${ROS_DISTRO}-xacro

# Flavor-specific system layers. FLAVOR selects one of them below.
FROM stack AS robot-sys

FROM stack AS dev-sys
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        gdb \
        mesa-utils \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-mvsim

# f1tenth_gym (JAX physics) for the f1tenth_gym_ros bridge. It needs NumPy 2, so it lives in
# its own venv instead of the system interpreter (which stays on numpy<2 for the Jazzy binaries).
# The venv sees the system ROS packages; only the bridge runs on it (see rbsf_f1tenth_gym).
# The install is editable because the gym downloads tracks into its own maps/ directory, which is
# world-writable since the runtime UID is only known when the container starts.
# Qt/GL libraries are only needed by the gym's own renderer (examples), not by the bridge.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        libegl1 libgl1 libgl1-mesa-dri libopengl0 libxcb-cursor0 libxcb-icccm4 libxcb-keysyms1 \
        libxcb-randr0 libxcb-render-util0 libxcb-shape0 libxcb-xinerama0 libxkbcommon-x11-0

ARG F1TENTH_GYM_REF=ec3b1a4693ed20317e42e57a9454d94b824cec81
# Freezes the resolution of the gym's (unpinned) transitive dependencies.
ARG F1TENTH_GYM_EXCLUDE_NEWER=2026-10-01T00:00:00Z
ENV RBSF_GYM_PYTHON=/opt/f1tenth_gym/venv/bin/python3 \
    JAX_PLATFORMS=cpu
RUN --mount=type=cache,target=/root/.cache/uv \
    --mount=type=bind,from=uv,source=/uv,target=/usr/local/bin/uv \
    git init -q /opt/f1tenth_gym/src \
 && git -C /opt/f1tenth_gym/src fetch -q --depth 1 https://github.com/f1tenth/f1tenth_gym.git "$F1TENTH_GYM_REF" \
 && git -C /opt/f1tenth_gym/src -c advice.detachedHead=false checkout -q FETCH_HEAD \
 && uv venv -q --system-site-packages --python /usr/bin/python3 /opt/f1tenth_gym/venv \
 && UV_LINK_MODE=copy uv pip install -q --python /opt/f1tenth_gym/venv/bin/python3 \
        --exclude-newer "$F1TENTH_GYM_EXCLUDE_NEWER" \
        -e /opt/f1tenth_gym/src "numpy>=2" \
 && chmod 1777 /opt/f1tenth_gym/src/maps

# ---------------------------------------------------------------------------
# manifests: strip src/ down to package.xml files (and colcon ignore markers).
# This stage re-runs on any source change, but its output only changes when a
# manifest does, so the rosdep layer below stays cached.
# ---------------------------------------------------------------------------
FROM base AS manifests
COPY src /tmp/ws/src
RUN find /tmp/ws/src -mindepth 1 -type f \
        ! -name package.xml ! -name COLCON_IGNORE ! -name AMENT_IGNORE -delete \
 && find /tmp/ws/src -mindepth 1 -type d -empty -delete
# The robot image carries no simulators: hide their packages from rosdep (it honors COLCON_IGNORE).
# Simulator packages sit next to the hardware ones in base_system/ as drop-in replacements.
ARG FLAVOR
ARG SIM_PACKAGES="f1tenth_gym_ros rbsf_f1tenth_gym rbsf_mvsim"
RUN if [ "$FLAVOR" = robot ]; then \
        for pkg in $SIM_PACKAGES; do \
            find /tmp/ws/src -type d -name "$pkg" -exec touch {}/COLCON_IGNORE \; ; \
        done; \
    fi

# ---------------------------------------------------------------------------
# final: workspace dependencies + runtime glue
# ---------------------------------------------------------------------------
FROM ${FLAVOR}-sys AS final

ARG DEBIAN_FRONTEND=noninteractive

# Workspace dependencies declared in package.xml files. The checksums let the
# entrypoint warn when the mounted workspace has drifted from the image.
COPY --from=manifests /tmp/ws /tmp/ws
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && rosdep update --rosdistro "$ROS_DISTRO" \
 && rosdep install --from-paths /tmp/ws/src --ignore-src -y --rosdistro "$ROS_DISTRO" \
 && mkdir -p /etc/rbsf \
 && (cd /tmp/ws && find src -name package.xml -print0 | sort -z | xargs -0r sha256sum) > /etc/rbsf/manifests.sha256 \
 && rm -rf /tmp/ws

# Python dependencies not available through apt/rosdep, installed into the
# system interpreter so ROS nodes see them without PYTHONPATH tweaks.
COPY --from=uv /uv /uvx /usr/local/bin/
RUN --mount=type=cache,target=/root/.cache/uv \
    --mount=type=bind,source=pyproject.toml,target=/tmp/py/pyproject.toml \
    --mount=type=bind,source=uv.lock,target=/tmp/py/uv.lock \
    uv export --project /tmp/py --frozen --no-dev --no-emit-project --no-hashes -o /tmp/requirements.txt \
 && if grep -qv '^#' /tmp/requirements.txt; then \
        UV_LINK_MODE=copy uv pip install --system --break-system-packages -r /tmp/requirements.txt; \
    fi \
 && rm /tmp/requirements.txt \
 && (cd /tmp/py && sha256sum pyproject.toml uv.lock) > /etc/rbsf/python.sha256

COPY docker/colcon-defaults.yaml docker/ros_env.sh docker/prompt.sh /etc/rbsf/
COPY docker/entrypoint.sh /usr/local/bin/rbsf-entrypoint
RUN echo '. /etc/rbsf/ros_env.sh' >> /etc/bash.bashrc \
 && for rc in /root/.bashrc /home/roboracer/.bashrc; do echo '. /etc/rbsf/prompt.sh' >> "$rc"; done

ARG FLAVOR
ENV RBSF_FLAVOR=${FLAVOR} \
    COLCON_DEFAULTS_FILE=/etc/rbsf/colcon-defaults.yaml

WORKDIR /ros2_ws
ENTRYPOINT ["/usr/local/bin/rbsf-entrypoint"]
CMD ["bash"]
