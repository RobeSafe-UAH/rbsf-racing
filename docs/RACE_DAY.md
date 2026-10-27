# Race day protocol

From an unknown track to a reference line the stack can use: map the track autonomously,
inspect the two maps, build the centerline and the raceline from the better one, and drive them on
the car, aiming at the best lap time. This covers only what is implemented today (see the end for
what is missing).

> **Validated on the car:** bringup, autonomous exploration and mapping, and centerline tracking
> (steps 1 to 4 and the centerline part of 6). **Not validated on the car:** the raceline optimiser's
> output and following it at speed (marked *(untested on the car)*). Do the first-run checks of step 1 after any new build or config, and expect to tune per
> track.

| Step | Where | Output |
|---|---|---|
| 0. Prepare | car + laptop | images built, workspace built |
| 1. Bring up and check the car | car | `drive_gate` STOPPED → DISARMED, odometry moving |
| 2. Autonomous exploration | car (+ laptop) | `maps/<stamp>/` session with both maps |
| 3. Inspection | laptop | `inspection.png`, `inspection.yaml`, recommended map |
| 4. Centerline and raceline generation | laptop | `maps/<stamp>/track/` with both lines |
| 5. Check both lines in simulation | laptop | both lines lap, with their sim lap times |
| 6. Best lap on the car | car + laptop | lap times: centerline baseline, then raceline |

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

- **Follow-the-Gap.** The exploration controller. It drives into the largest gap of the scan, so
  it needs no wall to follow and it also goes around static obstacles. Every LiDAR point is
  inflated by `inflation_radius` (0.25 m, half the car's width plus a margin), which closes the
  gaps of cone rows narrower than twice that (0.5 m): check the gaps on the track against it. A
  beam is free when its inflated range reaches `base_gap_threshold` (1.5 m): keep that above half
  the track width, or the walls beside the car count as free and it stops centring, and below
  the depth of dead-end bays, or it drives into them. The speed is 1.2 m/s at full lock to
  1.6 m/s on a straight. All of it is in
  `src/controller/rbsf_follow_the_gap/config/follow_the_gap_real.yaml`.

## 1. Bring up and check the car

Use `deployment:=distributed`: the car runs the platform side (drivers, RF2O odometry, joystick,
`drive_gate`), and the laptop runs the components and saves the maps where you inspect them.

1. **Wheels off the ground** for the first run of a new build or config.
2. Car: `ros2 launch stack_launcher platform.launch.py platform:=real`
3. Check the gate: `ros2 topic echo /r0/drive_gate/state`. It must say `STOPPED` / `NO_JOY`.
   Connect the gamepad: `DISARMED`. Drive manually with LB.
4. **Check the odometry.** Odometry is LiDAR-first: `odom` comes from
   RF2O, never from the VESC. Push the car by hand along a wall and watch
   `ros2 topic echo /r0/odom --field pose.pose.position`: x must grow forwards, and stop when
   the car stops. The VESC's own estimate is on `/r0/vesc/odom` for comparison only.
5. Continue with the rest of the design doc §6.4 checklist (arm, deadman, network cut) before the
   car goes on the ground.

## 2. Autonomous exploration

On the laptop, with the car's platform side running:

```bash
ros2 launch stack_launcher autonomous_exploration.launch.py \
    platform:=real deployment:=distributed controller:=follow_the_gap
```

- The launch log confirms the controller (`Following the gap, 1.2-1.6 m/s, cars 0.50 m wide`)
  and shows `track=none`: on the car there is no track by default, so nothing from the simulator tracks
  steers the run.
- The mapper is DualSLAM: Cartographer owns `/map` and the `map -> r0/odom` transform, and
  SLAM Toolbox maps in parallel on `/r0/mapping/slam_toolbox/map`. On the car Cartographer is
  LiDAR-only.
- **Watch it** (optional): on the laptop run `ros2 run foxglove_bridge foxglove_bridge` and open
  https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://localhost:8765 (or `rviz2`). Show
  `/map`, `/r0/mapping/slam_toolbox/map` and `/r0/scan` in the `map` frame. The two maps should
  overlay; SLAM Toolbox's alignment is fitted as the lap progresses.

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

## 4. Centerline and raceline generation

### Centerline


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

### Raceline

The raceline is the fastest line through the corridor, with a speed for every point, optimised
from the centerline and the corridor widths. It goes next to the track's yaml, in the
f1tenth_gym format that `rbsf_worlds` already reads (`line:=raceline` or `auto` picks it up, and
`auto` prefers it over the centerline):

```
maps/<stamp>/track/
  <name>_raceline.csv          s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2: closed loop
                               in the driving direction, in the same frame as the centerline
```

> The optimiser is not in this checkout, so its command is not documented here. Run it on
> `<name>_centerline.csv` and write its output to `<name>_raceline.csv`. Add the command here.

Check it before it leaves the laptop:

- The line is a closed loop in the same direction as the centerline, starting at the same
  point (the car is placed at the start of the reference line).
- It stays inside the corridor with margin: a raceline cuts the apexes, and a map that is a few
  centimetres off becomes a wall strike. Compare it with `centerline.png`, and keep the margin to
  the walls above the particle filter's error (about 6 cm in MVSim) plus the car's half-width.
- The speeds (`vx_mps`) are plausible for the car: the controllers take them as an upper bound
  and drive slower where their own curvature and acceleration limits say so, never faster.
- The curvature and speed profiles are smooth. A jump is a point where the car will brake hard.

## 5. Check both lines in simulation

Before trusting a generated track, drive it in MVSim (laptop, dev image). First the centerline:

```bash
ros2 launch stack_launcher raceline_tracking.launch.py \
    track:=/ros2_ws/maps/<stamp>/track/<name>.yaml
```

MVSim builds its world from the cleaned map, and Pure Pursuit follows the centerline
(`controller:=map` for MAP, `speed:=<m/s>` to go faster). The car
must lap without touching a wall (`/r0/collision` stays false). In the iros2026 rehearsal the
generated track ran 129 m in 70 s without a collision.

Then the raceline, with the same track and controller (`line:=raceline`, or leave `line` out once
the `_raceline.csv` exists: `auto` prefers it):

```bash
ros2 launch stack_launcher raceline_tracking.launch.py \
    track:=/ros2_ws/maps/<stamp>/track/<name>.yaml line:=raceline
```

It must also lap with `/r0/collision` false, and it should be faster than the centerline at the
same `speed:=`. If it is not, or it grazes a wall at an apex, fix the raceline (step 4) before
the car sees it. Write down both lap times: they are the reference for step 6, where the car
will be slower than MVSim.

To also check the car's localization chain (known map + particle filter instead of ground truth),
add `mapping:=map_server localization:=particle_filter`.

## 6. Best lap on the car

The goal is the best lap time on the real track, reached in steps: a safe lap on the centerline
as the baseline, then the raceline, then speed. Lap times are the only judge, so measure every
lap the same way (the start-line crossings, from a stopwatch or from `/r0/localization/odom` in a
bag) and keep a table.

On the car the behaviour localizes on the track's map: `map_server` publishes the cleaned map on
`/map`, and the particle filter (RF2O odometry + LiDAR) owns `map -> r0/odom`. Same
`ROS_DOMAIN_ID` on both machines.

```bash
# Car
ros2 launch stack_launcher platform.launch.py platform:=real

# Laptop (e.g. the eps track mapped on 2026-10-22)
ros2 launch stack_launcher raceline_tracking.launch.py platform:=real deployment:=distributed \
    line:=centerline track:=/ros2_ws/maps/20261022_094722/track/eps_261022.yaml
```

Before every run:

1. Put the car at the start of the reference line (the red dot in `centerline.png`), pointing in
   the driving direction: the particle filter starts there. If it starts elsewhere, pass
   `initial_pose:=x,y,yaw` or re-seed it with RViz's 2D Pose Estimate.
2. Check the localization before moving: in RViz (`rviz_slam`) or Foxglove the scan must overlay
   the map's walls. Push the car a metre by hand: the particles must follow it.
3. Arm (START), then hold RB. Release = stop, LB = take over.

### 6.1 Baseline: the centerline

Run it as above (`line:=centerline`) at a speed the car holds cleanly, a few laps. This is the
validated part. Record the lap time: every later run must beat it.

### 6.2 The raceline *(untested on the car)*

- **Raceline on the car**: `scripts/tracks/raceline.py maps/<stamp>/track/<name>.yaml` builds a
  checked minimum-curvature raceline for the mapped track (see the tracks README), and
  `line:=raceline` drives it. It is verified in MVSim only. Measure the car first (chassis
  overhangs and width at the wheels) and pass them as `--rear --front --width`.

Same command with `line:=raceline` (the track folder must hold `<name>_raceline.csv`), starting
at the **same top speed as the baseline**. Watch the first lap from the side: the raceline passes
closer to the walls, so any localization error shows up at the apexes. If the car touches a wall
or loses its localization, go back to the centerline and fix the cause (map, particle filter, or
the raceline's margin) before adding speed.

### 6.3 Push the lap time

From a raceline lap that is clean, change **one thing at a time** and keep the change only if the
lap time improves and the next three laps stay clean:

| Change | Where |
|---|---|
| Top speed | `speed:=<m/s>` (never below 0.8 m/s on the car). The raceline's own speeds cap it |
| Corner speed | `max_lateral_acceleration` in `src/controller/rbsf_path_tracking/config/pure_pursuit_real.yaml`, or a per-track `config/tracks/<track>_real.yaml` loaded on top. It is a conservative 3.0 until identified on the car |
| Acceleration and braking | `max_acceleration`, `max_deceleration` in the same file |
| Lookahead | `lookahead_min`, `lookahead_gain`: shorter cuts corners less, longer is smoother at speed |
| Controller | `controller:=map` instead of Pure Pursuit, which adds steering for the tyres' slip in fast corners. Its `understeer_gradient` is 0 in sim and has not been identified on the car |
| Speed limit | `max_auto_speed` in `src/base_system/rbsf_safety/config/drive_gate_real.yaml` (3.0 m/s). `drive_gate` clips every command to it, so `speed:=` above it does nothing. Raise it deliberately, with the wheels off the ground for the first check |

Watch the speed live with `plot_speed`: the controller's command, what `drive_gate` sends, and the
measured speed. A gap between the command and the measurement at the corners means the car is
not holding the profile. Stop raising speed when the car leaves the line, touches a wall, or the
particle filter loses it, and keep the last clean configuration as the race setup.

## Not validated yet

The car follows the centerline, but has not raced the generated track:

- **Raceline following.** The raceline optimiser is built, but its output has not been driven on
  the car (step 6.2 and 6.3). Until it is, the centerline lap is the validated fallback, and the
  controllers drive it with their curvature-based speed profile.
- **Real-car tuning values.** `max_lateral_acceleration` (3.0) and MAP's `understeer_gradient`
  are not identified on the car.

## Troubleshooting

| Symptom | Likely cause | Do |
|---|---|---|
| Gate stays `NO_JOY` | Gamepad not seen | Pair it on the car's host; check `ros2 topic echo /r0/joy` |
| Gate shows `AUTO_CMD_TIMEOUT` while RB is held | Components not running, or network/`ROS_DOMAIN_ID` mismatch | Check the laptop launch; `ros2 topic hz /r0/drive_gate/auto_cmd` on the car |
| Car drives into a side bay or an opening | `base_gap_threshold` is deeper than the bay, so the bay looks like a gap | Lower `base_gap_threshold` (keep it above half the track width), or take over with LB through that section |
| Car stops centring and runs along one wall | `base_gap_threshold` is below half the track width, so the walls beside the car count as free | Raise `base_gap_threshold` above half the track width |
| Car aims between the cones of a boundary | A gap in a cone row is wider than twice `inflation_radius` (0.5 m) | Raise `inflation_radius`, or close the gap |
| Car stops with no obstacle in front | No gap fits the car (the speed is 0 only then) | Check `inflation_radius` against the corridor width; `publish_debug: true` shows the inflated scan on `/r0/follow_the_gap/scan` |
| `save_maps`: "No map received yet" | The SLAMs are not running or have no scans | Check `/r0/scan` and the launch output |
| `closed_loop` fails on both maps | Lap not closed before saving | Drive further past the start line, save again |
| `trajectory_in_corridor` fails | That SLAM drifted | Use the other map, or remap more slowly |
| `centerline` says "no trajectory" | Session saved before trajectories were recorded | Pass `--ccw`/`--cw` and `--start X Y` |
