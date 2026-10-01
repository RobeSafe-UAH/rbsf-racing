# rbsf-racing
RobeSafe Racing Stack for RoboRacer Autonomous Racing

## Installation

Requires git and Docker (with BuildKit, Docker 23+). Add the NVIDIA Container Toolkit for GPU
access. Nothing else is installed on the host.

Third-party ROS packages (such as `f1tenth_gym_ros`) are git submodules under `src/`, so clone
with `--recurse-submodules`:

```bash
git clone --recurse-submodules https://github.com/RobeSafe-UAH/rbsf-racing.git
cd rbsf-racing
git switch dev    # day-to-day work; main holds stable releases
```

If you already cloned without it, or after pulling changes that add or move a submodule:

```bash
git submodule update --init --recursive
```

To have `git pull` keep submodules in sync automatically, run
`git config submodule.recurse true` once in the repo.

Then build the image and start the container:

```bash
make build        # dev image on x86_64, robot image on aarch64 (override with FLAVOR=dev|robot)
make run          # start the container; the repo is mounted at /ros2_ws and built on start
make attach       # open another shell in the running container
make help         # list all targets and variables
```

Images are based on ROS 2 Jazzy / Ubuntu 24.04 and contain nothing specific to the host,
so rebuilding (on a workstation or on the car) reuses cached layers. When a `package.xml`
or `pyproject.toml` changes, the container warns you to run `make build` again.

## Naming

Every car lives in its own namespace, in simulation and on the real cars:

| Namespace | What |
|---|---|
| `/r0`, `/r1` | Our cars (real or simulated ego). TF frames: `r0/base_link`, `r0/laser`, ... |
| `/opponents/or0`, `/opponents/or1`, ... | Simulated opponents, numbered from 0 |

Inside each namespace: `scan` (`sensor_msgs/LaserScan`), `odom` (`nav_msgs/Odometry`) and
`drive` (`ackermann_msgs/AckermannDriveStamped`). `/map` and `/tf` are global.

## Tracks

`src/base_system/rbsf_worlds` holds the tracks every simulator (and later the car) uses: a ROS map
(yaml + image) plus the reference lines to drive on. Both simulators take `track:=<name>`.

| Track | Length | Lines |
|---|---|---|
| `levine` | 61.8 m | centerline |
| `spielberg` | 338 m | centerline, raceline (with speed profile) |

See `src/base_system/rbsf_worlds/tracks/README.md` for the file formats and sources.

## Simulators

The dev image ships two simulators. Their packages live in `src/base_system/` next to the
hardware bringup, so they act as drop-in replacements for the real car: both give every car the
same `scan`, `odom` (map frame), `drive` and `collision` topics and TF frames.


### f1tenth_gym

[`f1tenth_gym_ros`](https://github.com/f1tenth/f1tenth_gym_ros) (branch `dev-jazzy`, submodule)
bridges the [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) simulator to ROS. Launch it
through `rbsf_f1tenth_gym`, which applies the naming above:

```bash
ros2 launch rbsf_f1tenth_gym f1tenth_gym.launch.py
#   track:=<rbsf_worlds track>  robot:=r0  num_agents:=3  spacing:=5.0  rviz:=false
#   foxglove:=false  spawn_on_line:=false (use the config's start poses)
```

- Drive the ego with `/r0/drive`, or from the keyboard:
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/r0/cmd_vel`.
- Opponents only move when something publishes on their `/opponents/orX/drive`.
- `track:=` also accepts a map yaml path, an f1tenth_gym_ros map (`maps/levine_obs`) or a gym
  track name (downloaded on first use).
- Parameters (LiDAR model, start poses, ...) are in
  `src/base_system/rbsf_f1tenth_gym/config/f1tenth_gym.yaml`.
- **Foxglove**: open
  <https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://localhost:8765> and import the
  layout `src/base_system/rbsf_f1tenth_gym/config/foxglove/f1tenth_gym.json`.
- The gym needs NumPy 2, so the image installs it in its own venv (`/opt/f1tenth_gym/venv`), and
  only the bridge runs on that interpreter. The rest of the stack stays on NumPy 1.x, which the
  Jazzy binaries (e.g. `cv_bridge`) need.

### MVSim

[MVSim](https://github.com/MRPT/mvsim) (`ros-jazzy-mvsim`) through `rbsf_mvsim`:

```bash
ros2 launch rbsf_mvsim mvsim.launch.py
#   track:=<rbsf_worlds track>  robot:=r0  headless:=true  foxglove:=false  world_file:=<xml>
```

- The world is generated at launch: the track's map as an occupancy grid, and F1TENTH cars
  (`definitions/f1tenth.vehicle.xml`, same wheelbase, steering limit and Hokuyo UST-10LX as the
  gym) in blue (ego) and orange (opponents).
- MVSim cars take `geometry_msgs/Twist`, so a per-car `mvsim_adapter` converts `/<ns>/drive`
  (Ackermann) to `/<ns>/cmd_vel`, and republishes MVSim's pose, scan and TF in the stack's
  convention. Keyboard teleop works on `/r0/cmd_vel` directly.
- `/map` is the grid MVSim simulates. Foxglove (same URL and layout as above) shows the map,
  scans, paths and plots, but not the car models, which come from f1tenth_gym_ros's URDF; MVSim's
  own window shows the cars.
