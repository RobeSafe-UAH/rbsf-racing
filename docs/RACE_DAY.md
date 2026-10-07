# Race day protocol

From an unknown track to a reference line the stack can use: map the track autonomously,
inspect the two maps, and build the centerline from the better one. This covers only what is
implemented today. Racing on the generated track is not (see the end).

> **The real car has not run this stack yet.** Everything below is verified in MVSim and
> f1tenth_gym. On the car, do the first-run checks of step 1 before anything else, and expect to
> tune. Untested parts are marked *(untested on the car)*.

| Step | Where | Output |
|---|---|---|
| 0. Prepare | car + laptop | images built, workspace built |
| 1. Bring up and check the car | car | `drive_gate` STOPPED → DISARMED, odometry moving |
| 2. Autonomous exploration | car (+ laptop) | `maps/<stamp>/` session with both maps |
| 3. Inspection | laptop | `inspection.png`, `inspection.yaml`, recommended map |
| 4. Centerline generation | laptop | `maps/<stamp>/track/` |
| 5. Check the track in simulation | laptop | the track drives |

## 0. Prepare (the day before)

- **Images.** On the car: `make build` (`FLAVOR=robot` by default on the Jetson). On the laptop:
  `make build` (`FLAVOR=dev`). Rebuild after any `package.xml` change: Cartographer and the
  other dependencies come from rosdep. The entrypoint warns when the image is out of date.
- **Workspace.** In each container: `cb`. On the car this also compiles RF2O
  (`src/state_estimation/rf2o_laser_odometry`, C++): give it a few minutes.
- **Laptop.** Inspection and centerline generation need matplotlib and SciPy, which are only in
  the dev image. Live map views (Foxglove, RViz) are only there too. Bring the laptop.
- **Network.** Both machines on the same network, both containers started with the same
  `ROS_DOMAIN_ID` (and `RMW_IMPLEMENTATION` if you set one):
  `make run ROS_DOMAIN_ID=<n>`.
- **Gamepad.** Pair it on the car's host. Buttons (Xbox layout, indices in
  `src/base_system/rbsf_safety/config/drive_gate_real.yaml`):

  | Button | Action |
  |---|---|
  | START (7) | Arm autonomy (only while RB is released) |
  | BACK (6) | Disarm |
  | RB (5), held | Autonomous driving. Release = stop |
  | LB (4), held | Manual driving: left stick speed, right stick steering. Always wins |
  | Y (3) | Save the maps (step 2) |

- **Wall following.** The car centres itself between both walls. A wall farther than 1.0 m
  (`max_wall_distance`) is lost: the car holds half the last measured corridor width (for up to
  `width_memory` = 2 s), or 0.4 m (`fallback_distance`), to the other wall. With both walls lost
  it holds its heading, so a section wider than 2 m needs a larger `max_wall_distance`. Rows of
  cones count as walls as long as their gaps are shorter than `bin_length` (1.0 m) and narrower
  than twice `path_half_width` (0.3 m), so up to ~0.5 m; check the gaps on the track. These and
  the speed (0.8 m/s) are in
  `src/controller/rbsf_wall_following/config/wall_following_real.yaml`.

## 1. Bring up and check the car

Use `deployment:=distributed`: the car runs the platform side (drivers, RF2O odometry, joystick,
`drive_gate`), and the laptop runs the components and saves the maps where you inspect them.

1. **Wheels off the ground** for the first run of a new build or config.
2. Car: `ros2 launch stack_launcher platform.launch.py platform:=real`
3. Check the gate: `ros2 topic echo /r0/drive_gate/state`. It must say `STOPPED` / `NO_JOY`.
   Connect the gamepad: `DISARMED`. Drive manually with LB.
4. **Check the odometry** *(untested on the car)*. Odometry is LiDAR-first: `odom` comes from
   RF2O, never from the VESC. Push the car by hand along a wall and watch
   `ros2 topic echo /r0/odom --field pose.pose.position`: x must grow forwards, and stop when
   the car stops. The VESC's own estimate is on `/r0/vesc/odom` for comparison only.
5. Continue with the rest of the design doc §6.4 checklist (arm, deadman, network cut) before the
   car goes on the ground.

## 2. Autonomous exploration

On the laptop, with the car's platform side running:

```bash
ros2 launch stack_launcher autonomous_exploration.launch.py \
    platform:=real deployment:=distributed
```

- The launch log confirms the controller (`Centring between the walls (fallback 0.4 m to a lone
  wall), 0.8 m/s`) and shows `track=none`: on the car there is no track by default, so nothing from the simulator tracks
  steers the run.
- The mapper is DualSLAM: SLAM Toolbox owns `/map` and the `map -> r0/odom` transform, and
  Cartographer maps in parallel on `/r0/mapping/cartographer/map`. On the car Cartographer is
  LiDAR-only *(untested on the car)*.
- **Watch it** (optional): on the laptop run `ros2 run foxglove_bridge foxglove_bridge` and open
  https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://localhost:8765 (or `rviz2`). Show
  `/map`, `/r0/mapping/cartographer/map` and `/r0/scan` in the `map` frame. The two maps should
  overlay; Cartographer's alignment is fitted as the lap progresses.

Drive the lap:

1. Put the car on the track, pointing in the driving direction, at the start/finish line. **Where
   the car starts becomes the start of the centerline**, and its direction the driving direction.
2. Arm (START), then hold RB. Stay ready to release it: release = stop, LB = take over.
3. Drive one full lap **plus a few metres past the start line**, so the end of the lap is in both
   maps (Cartographer adds scans to its map with a short delay).
4. **Save**: press Y, or on the laptop
   `ros2 service call /r0/mapping/save_maps std_srvs/srv/Trigger`. The reply names the session
   directory. The saver logs `slam_toolbox native state saved` and `cartographer native state
   saved`.
5. Release RB, disarm (BACK), stop the launch (Ctrl-C). You can save several times during one
   run: each save is a new session.

**Where the maps are.** On the machine that runs the behaviour (the laptop in distributed mode),
in the repository's `maps/` folder (`/ros2_ws/maps/` in the container, gitignored):

```
maps/<YYYYMMDD_HHMMSS>/
  session.yaml              platform, track, primary SLAM, each map's frame and pose in `map`
  slam_toolbox/             map.pgm + map.yaml, trajectory.csv, posegraph.{posegraph,data}
  cartographer/             map.pgm + map.yaml, trajectory.csv, map.pbstream
  ground_truth.csv          simulators only
```

Each `trajectory.csv` is the car's path as *that* SLAM saw it, in its own map's frame. The next
steps take the driving direction and the start line from it.

## 3. Inspection: which map is better?

Neither SLAM wins everywhere. In simulation SLAM Toolbox was better on spielberg, while
Cartographer was far better on iros2026, where SLAM Toolbox produced a tangled map. So check both
on every session. In the laptop's dev container (workspace sourced):

```bash
python3 scripts/maps/session_track.py inspect maps/<stamp>
```

It prints a verdict per map and a recommendation, and writes `maps/<stamp>/inspection.yaml` and
`maps/<stamp>/inspection.png`. **Open the PNG.** Each map has two automated checks:

| Check | Passes when | Catches |
|---|---|---|
| `closed_loop` | The cleaned free space is one corridor around one infield | An unclosed lap, a doubled wall from a bad loop closure, a large unknown hole |
| `trajectory_in_corridor` | ≥ 95 % of the SLAM's own path lies inside its own corridor | A drifted or tangled map: it no longer contains the path that built it |

It also reports the length and width range of the centerline each map would give. On the car
there is no ground truth, so the bottom-right panel asks you to judge by eye.

**Look at the figure.** Top row: each map in its own frame, with its cleaned corridor edge
(orange), its own trajectory and start (dot), and the centerline it would give (green).
Bottom-left: both maps overlaid in the `map` frame. Judge each map by:

- **Walls**: one clean line per wall. Doubled or smeared walls mean a bad loop closure.
- **Closure**: the start/finish area joins up, with no seam or step.
- **Trajectory**: inside the corridor all the way round, never through a wall.
- **Shape**: straights straight and corners where you saw them. Compare with the organisers'
  layout if you have it.
- **Size**: centerline length and widths plausible (official lap length, measured corridor width).

| Result | Do |
|---|---|
| One map passes | Use it (step 4) after a look at the figure |
| Both pass | Use the recommended one, unless the figure shows it is worse |
| Both fail | Drive another lap (new session), slower. `--force` only after checking the map by eye |

## 4. Centerline generation

```bash
python3 scripts/maps/session_track.py centerline maps/<stamp> --source <slam_toolbox|cartographer> \
    --name <track_name>
```

- It refuses a map that failed inspection (with the reasons). `--force` overrides that.
- `--name` names the track files. Without it the name is `track_<stamp>` on the car (no track
  given at launch) and `<track>_<stamp>` in simulation.
- The driving direction and start/finish come from the source map's trajectory. Override them
  with `--ccw`/`--cw` and `--start X Y` (metres, source map frame) if the car did not start on
  the line. Optional: `--step` (point spacing, 0.1 m) and `--smooth` (0.25 m).

Output, in the source map's own frame:

```
maps/<stamp>/track/
  <name>.png, <name>.yaml      cleaned map: the corridor free, everything else occupied
  <name>_centerline.csv        x_m, y_m, w_tr_right_m, w_tr_left_m: closed loop in the driving
                               direction from the start/finish line (rbsf_worlds format)
  centerline.png               preview
```

Open `centerline.png`: the green line follows the middle of the corridor all the way round, and
the arrow at the red start dot points in the driving direction. The script prints the length
and width range. Check them against the track.

The track is self-contained: launch files take it as `track:=<path to <name>.yaml>`.

## 5. Check the track in simulation

Before trusting a generated track, drive it in MVSim (laptop, dev image):

```bash
ros2 launch rbsf_mvsim pure_pursuit_demo.launch.py \
    track:=/ros2_ws/maps/<stamp>/track/<name>.yaml num_opponents:=0
```

MVSim builds its world from the cleaned map, and Pure Pursuit follows the centerline. The car
must lap without touching a wall (`/r0/collision` stays false). In the iros2026 rehearsal the
generated track ran 129 m in 70 s without a collision.

**This demo is simulation only.** `rbsf_pure_pursuit` still publishes `drive` directly, bypassing
`drive_gate`. Never run it on the car.

## Not implemented yet

Stop here on race day. These pieces are still missing before the car can race the generated
track:

- **Known-map mapping and localization on the car**: mapping slot `map_server` (the track's
  map) and a particle-filter localization slot that owns `map -> r0/odom`.
- **Planning**: a planning slot that publishes the track's line (`planning/trajectory`), and a
  raceline optimised from the centerline.
- **Controller migration**: `rbsf_pure_pursuit` reading `planning/trajectory` and
  `localization/odom`, and publishing `drive_gate/auto_cmd`.
- **A `raceline_tracking` behaviour** that ties them together (design doc §5.1).

## Troubleshooting

| Symptom | Likely cause | Do |
|---|---|---|
| Gate stays `NO_JOY` | Gamepad not seen | Pair it on the car's host; check `ros2 topic echo /r0/joy` |
| Gate shows `AUTO_CMD_TIMEOUT` while RB is held | Components not running, or network/`ROS_DOMAIN_ID` mismatch | Check the laptop launch; `ros2 topic hz /r0/drive_gate/auto_cmd` on the car |
| Car drifts into an opening | Width memory expired before the gap ended | Raise `width_memory`, or take over with LB through that section |
| Car goes straight where the track is wide | Both walls farther than `max_wall_distance` | Raise `max_wall_distance` above half the track width |
| `save_maps`: "No map received yet" | The SLAMs are not running or have no scans | Check `/r0/scan` and the launch output |
| `closed_loop` fails on both maps | Lap not closed before saving | Drive further past the start line, save again |
| `trajectory_in_corridor` fails | That SLAM drifted | Use the other map, or remap more slowly |
| `centerline` says "no trajectory" | Session saved before trajectories were recorded | Pass `--ccw`/`--cw` and `--start X Y` |
