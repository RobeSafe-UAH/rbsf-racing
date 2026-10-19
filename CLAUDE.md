# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project

`rbsf-racing` is the RobeSafe (Universidad de Alcalá) autonomous racing stack for
RoboRacer (formerly F1Tenth) competitions. It runs on a 1/10-scale Ackermann car and in
simulation.

This repo is a **full rewrite and port** of the previous stack,
[`rbsf-racing-irobocity`](../rbsf-racing-irobocity), which was built for teaching at the
iRoboCity Summer School. The legacy repo is the functional reference, not a template to copy:

- **Keep**: the overall behaviors (teleop SLAM, wall following, autonomous SLAM, centerline
  extraction and tracking), the `real` / `mvsim` / `distributed` deployment modes, and the
  Docker + `uv` development workflow.
- **Drop**: teaching scaffolding. That means `*_student.py` files, hardcoded "students fill
  this in" parameters, warnings about `ROS_DOMAIN_ID` seat numbers, and course-specific names
  like `curso_uah_irobocity` or `iRoboCity`.
- **Improve**: move parameters into ROS 2 params / YAML configs instead of hardcoding them
  in nodes, add proper package metadata (maintainer, license), and add tests.

When porting a component, read its legacy implementation and the matching `docs/*.md` in the
legacy repo first. Then rewrite it to this repo's conventions.

## Architecture

Read `docs/DESIGN_PHILOSOPHY.md` before adding a package or launch file. In short: apps are
behaviours in `stack_launcher`, defined by `platform:=real|mvsim|f1tenth`,
`deployment:=local|distributed` and one implementation per functional slot (mapping,
localization, perception, planning, controller), with identity implementations for unused slots.
Every command to the actuators goes through `drive_gate` (`rbsf_safety`).

## Legacy architecture (reference)

ROS 2 Humble (Ubuntu 22.04) colcon workspace. All packages are `ament_python`.

```
src/
  base_system/
    f1tenth_system/            # git submodule (RobeSafe-UAH fork, humble-devel): VESC, ackermann_mux, teleop_tools
    rbsf_bringup/              # hardware bringup launch + RViz config
    rbsf_mvsim/                # MVSim worlds (.world.xml), vehicle/sensor definitions, world generator
  controller/
    rbsf_wall_following/       # PI corridor centring between walls or cone rows, LaserScan -> drive_gate
    rbsf_centerline_tracking/  # path tracker, /centerline (nav_msgs/Path) + /odom -> /drive
  localization/
    rbsf_slam_toolbox/         # slam_toolbox config (sim/real), map_saver node, centerline_publisher node
  stack_launcher/              # top-level 0X_*.launch.py files plus helper nodes (keyboard teleop, ackermann->twist, odom)
scripts/
  centerline/                  # offline centerline extraction from saved maps (CSV output)
  f1tenth_racetracks/          # git submodule: f1tenth/f1tenth_racetracks
```

Key interfaces (legacy):
- Control output is `/drive` (`ackermann_msgs/AckermannDriveStamped`), multiplexed through `ackermann_mux`.
- The LiDAR topic is `/scan` on the real car and `/laser1` in MVSim.
- The stack launchers take `mode:=real|mvsim|distributed`. `distributed` skips hardware
  bringup on the compute node and expects `00_distributed_bringup.launch.py` to run on the
  embedded device, with the same `ROS_DOMAIN_ID` on both.
- Maps are saved to `maps/map_YYYYMMDD_HHMMSS/`.

## Development environment

Everything runs in Docker (ROS 2 Jazzy, Ubuntu 24.04). Nothing is installed on the host.
Files: `docker/Dockerfile`, `docker/entrypoint.sh`, `docker/ros_env.sh`,
`docker/colcon-defaults.yaml`, `Makefile`, `.dockerignore`.

- `make build` builds `rbsf-racing:jazzy-<FLAVOR>`. `FLAVOR=dev` (the default on x86_64)
  adds the ROS desktop, RViz and MVSim. `FLAVOR=robot` (the default on aarch64) is ros-base
  plus drivers, with no GUI.
- `make run` starts the container with the repo mounted at `/ros2_ws`. `make attach` opens
  another shell in it (as user `roboracer`). `make help` lists every target and variable
  (`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `GPU`, `AUTO_BUILD`, `CMD`, `BASE_IMAGE`).
- Inside the container, `cb [colcon args]` builds the workspace and re-sources the overlay.
  Symlink install is on by default. Run lint and tests with
  `colcon test --packages-select <pkg> && colcon test-result --verbose`.

**Image caching rules.** `make build` on the car must reuse cached layers, so keep these when
editing the Dockerfile:
- No build step may depend on the host user. The image ships a fixed user `roboracer`, and
  `entrypoint.sh` (which starts as root) remaps it to `HOST_UID`/`HOST_GID`, adds it to the
  groups of the mounted `/dev` nodes, then drops privileges with `setpriv`.
- Source code never enters the image. Only `package.xml` files (extracted in the `manifests`
  stage, then fed to `rosdep install`) and `pyproject.toml`/`uv.lock` do. Editing code
  re-runs only the small manifest stage. Editing a `package.xml` re-runs rosdep onward.
- Order layers from stable to volatile, pin external inputs (`ROS_APT_SOURCE_VERSION`,
  `UV_VERSION`), and declare each `ARG` right before the layer that uses it.
- New ROS dependencies go in the package's `package.xml` (rosdep). Add a dependency to the
  `stack` layer only if it is heavy and needed by the whole stack.
- f1tenth_gym needs NumPy 2, so it lives in its own venv (`/opt/f1tenth_gym/venv`,
  `--system-site-packages`, pinned by `F1TENTH_GYM_REF`). `rbsf_f1tenth_gym`'s launch file runs
  only its `gym_bridge` node with that interpreter (`Node(prefix=...)`). Never put
  the venv on `PATH`/`PYTHONPATH`.
- Simulator-only packages are listed in the Dockerfile's `SIM_PACKAGES` so the robot image skips
  their rosdep keys. Add new simulator packages there (`rbsf_worlds` is not simulator-only).
  Never declare them as dependencies of non-simulator packages (`stack_launcher` looks them up
  at runtime), or rosdep fails on the robot image.
- MVSim only takes `Twist` on `cmd_vel`, publishes `odom` relative to the start pose and a
  per-vehicle `/<ns>/tf` with unprefixed frames. `rbsf_mvsim/launch/mvsim.launch.py` remaps those
  to `/<ns>/mvsim/*` and its `mvsim_adapter` node restores the stack convention. Worlds are
  generated at launch (`rbsf_mvsim/world.py`) because MVSim defines vehicles inside the world.
- Python packages go in `pyproject.toml` (then run `uv lock`) only if apt/rosdep cannot provide
  them. They are installed into the system interpreter, and `numpy<2` is enforced to stay ABI-compatible with
  the Jazzy binaries.
- The entrypoint warns when the mounted workspace's `package.xml` or Python lock differ from
  what the image was built with. Then you need `make build`.

## Conventions

- Every car has its own namespace: our cars `/r0`, `/r1` (TF frames `r0/base_link`, ...), simulated
  opponents `/opponents/or0`, `/opponents/or1`, ... (numbered from 0). `rbsf_f1tenth_gym` applies
  this to the unmodified upstream bridge by patching `opp_suffix` and remapping its global topics;
  its `test_upstream_bridge.py` fails if upstream changes those internals.
- Package names use the `rbsf_` prefix. Group packages under `src/<block>/`, following the
  ForzaETH blocks on top of the platform: `base_system` (drivers, simulators, worlds, safety),
  `state_estimation` (mapping and localization slots, odometry estimators), `perception`,
  `planning`, `controller`; plus `common` and `stack_launcher`. Blocks group code, slots are
  launch contracts (`docs/DESIGN_PHILOSOPHY.md` §7).
- Each node module starts with a docstring that lists its subscribed and published topics
  with message types, followed by a short outline of the algorithm. Keep this practice.
- Launch files are Python (`*.launch.py`) and use `OpaqueFunction` + `launch_setup(context)`
  when they branch on `mode`. Validate `mode` and raise on unknown values.
- Keep separate `*_sim.yaml` and `*_real.yaml` configs whenever parameters differ between
  simulation and the car.
- The license is Apache-2.0 (see `LICENSE`). Legacy code was MIT. Use Apache-2.0 in
  `package.xml` / `setup.py`.
- Commit messages use Conventional Commits (`feat:`, `fix:`, `chore:`, `docs:`, `refactor:`).
- Day-to-day work happens on `dev`. `main` holds stable releases.

## Roadmap

**Milestone 1 — autonomous exploration (sim + real).** Reproduce the legacy autonomous
exploration behaviour (reactive wall following + simultaneous SLAM mapping, legacy
`02_autonomous_slam`) on both MVSim and the real car. Status:
- [x] Docker images, Makefile, entrypoint (Jazzy / Ubuntu 24.04)
- [x] Platform drivers for Jazzy (`f1tenth/vesc` jazzy submodule; `drive_gate` replaces
  ackermann_mux and joy_teleop)
- [x] Hardware bringup (`rbsf_bringup`) and MVSim simulation packages. The real car is not
  tested yet.
- [x] Wall-following controller (`rbsf_wall_following`), verified in MVSim and f1tenth_gym.
  Default `follow_side: left` explores eps_oval_single_wall in both sims with DualSLAM (maps
  pass inspection, 2026-10-19). Known gaps: crashes on levine's inner-wall recess, and on eps
  with 80 ms of extra steering lag (passes up to 60 ms)
- [x] Follow-the-Gap controller (`rbsf_follow_the_gap`, `controller:=follow_the_gap`), the ForzaETH
  mapping controller with static obstacle avoidance. Verified in f1tenth_gym (spielberg with
  boxes on the line) and MVSim (iros2026 lap, Cartographer map passes inspection) (2026-10-07).
  Gap fallback, near-wall inflation, gap hysteresis and `target_fraction` (inner-side lean) added
  so it explores eps_oval_single_wall in both sims with DualSLAM (2026-10-19)
- [x] Safety gate (`rbsf_safety` / `drive_gate`), verified in sim
- [x] SLAM integration and map saving (mapping slot): DualSLAM (`rbsf_dual_slam`), verified in
  MVSim and f1tenth_gym. Cartographer does not close the start/finish loop on spielberg yet
  (`min_score` trade-off, see `config/cartographer_sim.lua`)
- [x] Top-level launchers (`stack_launcher`) with `platform:=real|mvsim|f1tenth`,
  `deployment:=local|distributed`
- [ ] Real-car validation (design doc §6.4 checklist, then wall-following gains)

## Decisions

- ROS 2 Jazzy on Ubuntu 24.04 (Python 3.12). Decided 2026-10-01.
- Simulators: MVSim (`rbsf_mvsim`) and f1tenth_gym (`f1tenth_gym_ros` `dev-jazzy` submodule, pure
  upstream, wrapped by `rbsf_f1tenth_gym`), both dev-image only. Simulator packages live in
  `src/base_system/` next to the hardware packages as drop-in replacements: every simulator must
  give each car `/<ns>/{scan,odom,drive,collision}` and TF frames `<ns>/base_link`, `<ns>/laser`
  on the global `/tf`. Decided 2026-10-01.
- Odometry contract, same on every platform: `odom` in the `<ns>/odom` frame plus TF
  `<ns>/odom -> <ns>/base_link`. No platform publishes `/map` or `map -> <ns>/odom`: the mapping
  slot owns both (identity mapping = identity TF + `/ground_truth/map` relay). Simulators also
  publish `/<ns>/ground_truth/odom` (frame `map`) and `/ground_truth/map`. Decided 2026-10-01.
- The odom *producer* is platform-dependent. The real car is LiDAR-first: RF2O
  (`src/state_estimation/rf2o_laser_odometry`, vendored Jazzy port of MAPIRlab's `ros2` branch, GPL-3.0,
  see its `PORTING.md`). VESC odom/IMU never feeds TF or SLAM: it was too noisy in the legacy
  stack, and other teams reported the same. It stays on `vesc/odom` for diagnostics. Simulators
  use ground truth relabelled to `<ns>/odom`: RF2O degrades in feature-poor synthetic worlds.
  Decided 2026-10-01.
- Mapping: DualSLAM (`rbsf_dual_slam`). SLAM Toolbox is primary (owns `/map` + TF), and
  Cartographer is secondary (`publish_to_tf = false`, `/<ns>/mapping/cartographer/map`, aligned by
  `map_overlay`). `map_saver` writes nav2-format PGM/YAML per SLAM for the offline map tools.
  Cartographer uses odometry in sim and runs LiDAR-only on the real car (RF2O would count the
  scans twice). Decided 2026-10-01.
- Offline track generation: the source map is chosen per session, after inspection, because
  neither SLAM wins on every track (SLAM Toolbox was better on spielberg, Cartographer far better
  on iros2026). `scripts/maps/session_track.py inspect` runs automated checks (closed corridor,
  own trajectory inside its own corridor, ground truth in sim) and writes a figure for a human
  look; `centerline --source <map>` builds the track in `maps/<stamp>/track/` and refuses failed
  maps without `--force`. It ports the legacy `scripts/centerline` tools: the algorithm lives in
  `rbsf_worlds/centerline.py`, shared with `scripts/tracks/drawn_track.py`. Decided 2026-10-01.
- Tracks (maps + reference lines) and world/map tools live in the shared `rbsf_worlds` package;
  simulators and controllers take `track:=<name>`. Reference lines are closed loops ordered in the
  driving direction. Decided 2026-10-01.

- Architecture (see `docs/DESIGN_PHILOSOPHY.md`). Decided 2026-10-01:
  - Five functional slots, swapped at launch level (no plugin framework). Every app fills all
    five slots, using identity implementations for the ones it does not need.
  - Behaviours live in `stack_launcher` and take `platform:=real|mvsim|f1tenth` and
    `deployment:=local|distributed`. These replace the legacy `mode`.
  - Mapping owns SLAM. Exactly one component publishes `map -> <ns>/odom`.
  - Safety is `drive_gate` in its own `rbsf_safety` package, on the car side. It fails closed,
    and autonomous motion on `real` always requires the deadman.
  - Directories follow the ForzaETH race-stack blocks (`state_estimation`, `perception`,
    `planning`, `controller`) on top of `base_system`. Mapping and localization are two slots in
    the `state_estimation` block. Odometry estimators (RF2O) live there too, but they are not a
    slot: the platform launches them. Decided 2026-10-01.

## Open decisions

Confirm these with the user before assuming:
- Whether performance-critical nodes move to C++ (`ament_cmake`).
- The rest of `docs/DESIGN_PHILOSOPHY.md` §9 (`rbsf_msgs` types, runtime orchestrator, VESC brake).

Move each item to **Decisions** once it is settled.
