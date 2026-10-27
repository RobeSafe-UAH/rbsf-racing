# Third-party licenses

`rbsf-racing` is licensed under Apache-2.0 (see [`LICENSE`](LICENSE)). This repository also
carries or fetches code from other projects, which keep their own licenses. Each vendored
directory has its own `LICENSE`, and `PORTING.md` describes what was changed from upstream.

## In this repository

| Path | Upstream | License | Notes |
|---|---|---|---|
| `src/state_estimation/rf2o_laser_odometry` | [MAPIRlab/rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry) (`ros2` branch) | **GPL-3.0-only** | Jazzy port, see its `PORTING.md`. The real car's odometry source. |
| `src/state_estimation/particle_filter` | [f1tenth/particle_filter](https://github.com/f1tenth/particle_filter) (`humble-devel`) | MIT | Port, see its `PORTING.md`. |
| `src/state_estimation/range_libc` | [kctess5/range_libc](https://github.com/kctess5/range_libc) | Apache-2.0 | `ament_cmake` build, CPU methods only, see its `PORTING.md`. |
| `src/controller/rbsf_path_tracking` (MAP steering law) | [RobeSafe-UAH/tfg-ruben-lopez](https://github.com/RobeSafe-UAH/tfg-ruben-lopez) | see upstream | Ported from the RobeSafe group's own repository. |

## Git submodules

Not copied into the history: `git clone --recurse-submodules` downloads them.

| Path | Upstream | License |
|---|---|---|
| `src/base_system/f1tenth_gym_ros` | [f1tenth/f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) (`dev-jazzy`) | MIT |
| `src/base_system/vesc` | [f1tenth/vesc](https://github.com/f1tenth/vesc) (`jazzy`) | BSD |

## Fetched when the image is built

Installed in the Docker images, not stored in the repository: ROS 2 Jazzy packages (mostly
Apache-2.0 and BSD), [MVSim](https://github.com/MRPT/mvsim) (`ros-jazzy-mvsim`),
[f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) (pinned by `F1TENTH_GYM_REF`) and the
Python packages in `uv.lock`. Check their own licenses before redistributing an image.

## GPL-3.0 note

RF2O is GPL-3.0 and the rest of the stack is Apache-2.0. RF2O runs as its own ROS node and talks
to the rest of the stack only through topics, so the stack does not link against it. The GPL-3.0
terms still apply to RF2O's directory, and to any copy of this repository that includes it.
Remove that directory to get a distribution without GPL code (the real car then has no LiDAR
odometry). This note is not legal advice: confirm the choice with the group before the release.
