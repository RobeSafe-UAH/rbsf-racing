# rbsf-racing
RobeSafe Racing Stack for RoboRacer Autonomous Racing

## Installation

Requires git and Docker (with BuildKit, Docker 23+). Add the NVIDIA Container Toolkit for GPU
access. Nothing else is installed on the host.

Third-party ROS packages (`f1tenth_gym_ros`, and `vesc` for the real car's motor controller) are
git submodules under `src/`, so clone with `--recurse-submodules`:

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

Inside each namespace: `scan` (`sensor_msgs/LaserScan`), `odom` (`nav_msgs/Odometry`, frame
`<ns>/odom`) and `drive` (`ackermann_msgs/AckermannDriveStamped`). `/map` and `/tf` are global.
The TF tree is `map -> <ns>/odom -> <ns>/base_link -> <ns>/laser`, and `map -> <ns>/odom` belongs
to the mapping slot (a SLAM, or the identity when there is none).

## Tracks

`src/base_system/rbsf_worlds` holds the tracks every simulator (and later the car) uses: a ROS map
(yaml + image) plus the reference lines to drive on. Both simulators take `track:=<name>`.

| Track | Length (centerline) | Lines |
|---|---|---|
| `levine` | 61.8 m | centerline |
| `spielberg` | 343.3 m | centerline, raceline (338.1 m, with speed profile) |
| `iros2026` | 209.4 m | centerline |

See `src/base_system/rbsf_worlds/tracks/README.md` for the file formats and sources.

## Apps

The stack is split into five interchangeable functional slots (mapping, localization,
perception, planning, controller) that behaviours in `stack_launcher` compose into apps, on any
platform (`real`, `mvsim`, `f1tenth`) and deployment (`local`, `distributed`). See
[`docs/DESIGN_PHILOSOPHY.md`](docs/DESIGN_PHILOSOPHY.md).

Every command to the motors goes through `drive_gate` (`rbsf_safety`), the only publisher on
`/<ns>/drive`. It fails closed: any lost signal stops the car, and a second publisher on `drive`
shuts the whole stack down. Its state is on `/<ns>/drive_gate/state`.

### Autonomous exploration

The car follows a wall (`rbsf_wall_following`, PI on two LiDAR rays, with gap and corner
handling) while the mapping slot builds the map. The default mapper is DualSLAM
(`rbsf_dual_slam`): SLAM Toolbox and Cartographer map side by side from the same scans.

```bash
ros2 launch stack_launcher autonomous_exploration.launch.py platform:=mvsim track:=spielberg
ros2 launch stack_launcher autonomous_exploration.launch.py platform:=f1tenth track:=spielberg
#   robot:=r0  headless:=true  foxglove:=false  rviz:=false  joy:=false  require_deadman:=true
#   wall_side:=auto|left|right
#   mapping:=dual_slam|slam_toolbox|cartographer|identity ... controller:=wall_following

# Save both maps (gamepad Y does the same):
ros2 service call /r0/mapping/save_maps std_srvs/srv/Trigger
```

- SLAM Toolbox is the primary mapper: it publishes `/map` and owns `map -> r0/odom`.
  Cartographer runs without TF on `/r0/mapping/cartographer/map`, in the frame
  `r0/cartographer/map`. `map_overlay` fits that frame to `map` from both trajectories, so the
  two maps overlay in Foxglove.
- A save writes `maps/<YYYYMMDD_HHMMSS>/{slam_toolbox,cartographer}/map.{pgm,yaml}` (nav2
  map_server format, the input of the offline map cleaning and centerline tools), each SLAM's
  own state (`posegraph.*`, `map.pbstream`), each SLAM's own trajectory, ground truth on the
  simulators, and `session.yaml` (platform, track, and each map's frame and pose in `map`). Save
  after driving a little past the start line, so the end of the lap is in both maps.

#### From a mapping session to a track

Neither SLAM is reliably better (it depends on the track), so the map a track is built from is
chosen per session, after inspecting both. In the dev container, with the workspace sourced:

```bash
python3 scripts/maps/session_track.py inspect maps/<stamp>
#   -> maps/<stamp>/inspection.png (look at it) + inspection.yaml, and a recommended source
python3 scripts/maps/session_track.py centerline maps/<stamp> --source cartographer
#   -> maps/<stamp>/track/<track>_<stamp>.{png,yaml}, _centerline.csv, centerline.png
ros2 launch rbsf_mvsim pure_pursuit_demo.launch.py track:=/ros2_ws/maps/<stamp>/track/<name>.yaml
```

- `inspect` checks every map: the cleaned free space must be one closed corridor (an unclosed
  lap, a doubled wall from a bad loop closure or a large unknown hole fails it), and at least 95%
  of the SLAM's own trajectory must lie inside its own corridor (a drifted or tangled map fails
  it). On the simulators it also scores each map and trajectory against ground truth.
- `centerline` refuses a map that failed inspection unless you pass `--force`. It writes a
  cleaned map (corridor free, rest occupied) and the centerline in the rbsf_worlds format. The
  line is ordered in the driving direction and starts where the car started. Both come from the
  recorded trajectory, and `--ccw/--cw` and `--start X Y` override them. Coordinates are in the
  source map's frame, so the result is a self-contained track for both simulators and the planners.
- The algorithm (`rbsf_worlds/centerline.py`) is shared with `scripts/tracks/drawn_track.py`.

- With `wall_side:=auto` (default), on a track with a reference line, the car follows its outer
  wall (right on counter-clockwise tracks, left on clockwise ones). Otherwise it follows the
  YAML's `wall_side`. `track` defaults to `levine` on the simulators and to none on the real
  car, so pass `wall_side:=left|right` there. Gains and distances are in
  `src/controller/rbsf_wall_following/config/wall_following_{sim,real}.yaml`.
- `levine` is a building floor with junctions that lead off the loop: the car explores them.
  Use `iros2026` or `spielberg` for a closed lap.
- In simulation the car drives without a joystick (`require_deadman` defaults to `false`). With
  a gamepad plugged in, LB + sticks takes over at any time.

### Real car

```bash
# Everything on the car (follow the outer wall: wall_side:=left|right)
ros2 launch stack_launcher autonomous_exploration.launch.py platform:=real wall_side:=left
# Distributed: same ROS_DOMAIN_ID on both machines (make run ROS_DOMAIN_ID=<n>)
ros2 launch stack_launcher platform.launch.py platform:=real                                    # car
ros2 launch stack_launcher autonomous_exploration.launch.py platform:=real deployment:=distributed wall_side:=left  # laptop
```

`rbsf_bringup` starts the VESC (`/dev/sensors/vesc`), the Hokuyo (Ethernet, 192.168.0.10), RF2O
laser odometry, the joystick (`/dev/input/js0`, pair a Bluetooth gamepad on the host first) and
`drive_gate`. Odometry is LiDAR-first: `odom` and its TF come from RF2O
(`src/state_estimation/rf2o_laser_odometry`, a Jazzy port of MAPIRlab's package, GPL-3.0). The VESC's
wheel odometry is too noisy to drive TF or SLAM, so it is only on `vesc/odom` for diagnostics.
`require_deadman` is always `true` on the real car: launching with `false` is an error.

| Gamepad | Action |
|---|---|
| START (7) | Arm autonomy (only while RB is released) |
| BACK (6) | Disarm |
| RB (5), held | Autonomous driving. Release = stop |
| LB (4), held | Manual driving: left stick speed, right stick steering. Always wins |

Indices are in `src/base_system/rbsf_safety/config/drive_gate_real.yaml`. Check them for your
gamepad with `ros2 topic echo /r0/joy`. A joystick timeout (0.5 s) disarms, and a stale command
from the controller (0.2 s, e.g. network loss in distributed mode) stops the car.

First run of a new build or config (docs/DESIGN_PHILOSOPHY.md §6.4), **wheels off the ground**:
1. Start the platform side. `drive_gate/state` must say `STOPPED` / `NO_JOY`.
2. Connect the gamepad: `DISARMED`. Drive manually with LB.
3. Start the app, arm with START, hold RB: the wheels turn. Release RB: they stop at once.
4. In distributed mode, cut the network while holding RB: the car must stop
   (`AUTO_CMD_TIMEOUT`).

## Simulators

The dev image ships two simulators. Their packages live in `src/base_system/` next to the
hardware bringup, so they act as drop-in replacements for the real car: both give every car the
same `scan`, `odom` (frame `<ns>/odom`), `drive` and `collision` topics and TF frames. Their `odom`
is ground truth whose origin is the map origin. They also publish `/<ns>/ground_truth/odom` (frame
`map`) and the track's map as `/ground_truth/map`. RF2O is real-only: it does poorly in the
feature-poor synthetic worlds.

### Pure Pursuit demo

Every car follows the track's raceline (or its centerline when there is none) with
`rbsf_pure_pursuit`, spawned `spacing` metres apart along it with the ego last. All cars set off
together `start_delay` seconds after launch. The same command works on both simulators:

```bash
ros2 launch rbsf_mvsim pure_pursuit_demo.launch.py                       # Levine, centerline, 2 m/s
ros2 launch rbsf_f1tenth_gym pure_pursuit_demo.launch.py track:=spielberg  # raceline x 0.6
#   num_opponents:=3  spacing:=5.0  speed:=2.0  speed_scale:=0.6  line:=auto|raceline|centerline
#   ego_driver:=false  start_delay:=5.0
```

With `ego_driver:=false` the ego is yours to drive (opponents will lap it if it stops). Pure
Pursuit has no obstacle avoidance, so cars only keep their gaps at equal speeds. On MVSim, Spielberg's
raceline (which passes 0.25 m from the walls at its apexes) makes the cars graze a few walls; use
`line:=centerline speed:=3.0` there for a contact-free run. Its gains are in
`src/controller/rbsf_pure_pursuit/config/pure_pursuit_sim.yaml`.

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

- The world is generated at launch: the track's map as an occupancy grid (outline copy: MVSim's
  LiDAR misses walls of maps filled solid outside the track), and F1TENTH cars
  (`definitions/f1tenth.vehicle.xml`, same wheelbase, steering limit and Hokuyo UST-10LX as the
  gym) in blue (ego) and orange (opponents).
- MVSim cars take `geometry_msgs/Twist`, so a per-car `mvsim_adapter` converts `/<ns>/drive`
  (Ackermann) to `/<ns>/cmd_vel`, and republishes MVSim's pose, scan and TF in the stack's
  convention. Keyboard teleop works on `/r0/cmd_vel` directly.
- `/ground_truth/map` is the grid MVSim simulates (the mapping slot republishes it, or its own
  map, as `/map`). Foxglove (same URL and layout as above) shows the map,
  scans, paths and plots, but not the car models, which come from f1tenth_gym_ros's URDF; MVSim's
  own window shows the cars.
