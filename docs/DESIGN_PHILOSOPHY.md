# Design philosophy

How `rbsf-racing` is put together: which pieces exist, how they talk to each other, how an app
is composed from them, and which safety rules no app may break. Read this before adding a
package or a launch file.

> Status: this is the target architecture. Some of it is not implemented yet (see §9). Names
> marked *proposed* may change while the first implementations land. Update this document
> when they do.

## 1. Purpose and scope

The stack is split into interchangeable **functional components** (localization, mapping,
perception, planning, controller), and **behaviours** compose them into the apps we deploy. The
same wall follower or Pure Pursuit then runs unchanged on the real car, in MVSim and in
f1tenth_gym, on one machine or split across two. Replacing a component (one controller with
another, a ground-truth pose with a particle filter) is a launch argument, not an edit to the
app.

The idea comes from [EasyNav](https://easynavigation.github.io/), which splits navigation into
fixed, pluggable stages. We adapt it to autonomous racing:

- **Several cars, one namespace each.** Every component runs per car under `/<ns>`.
- **High-rate control on a small car.** Contracts are plain ROS 2 topics with explicit rates and
  timeouts, so components can sit in different processes or on different machines.
- **A real vehicle with real risk.** One gate between every command source and the motors
  (`drive_gate`, §6) enforces safety. The gate does not depend on any component behaving well.

## 2. Vocabulary

| Term | Meaning |
|---|---|
| **Platform** | What turns `drive` commands into motion and produces sensor data: `real` (VESC, LiDAR, joystick), `mvsim`, or `f1tenth` (f1tenth_gym). Chosen with `platform:=`. |
| **Deployment** | Where the processes run: `local` (one machine) or `distributed` (platform side on the car, components on a compute node). Chosen with `deployment:=`. |
| **Functional component** | One of the five fixed slots: localization, mapping, perception, planning, controller. Each slot has a topic contract (§4). |
| **Implementation** | A concrete fill for a slot, e.g. `pure_pursuit` or `wall_following` for the controller slot, `slam_toolbox` for mapping. |
| **Identity** | The no-op implementation of a slot. It honours the slot's contract with neutral output (§3.3). |
| **Behaviour (app)** | An entrypoint in `stack_launcher` that picks one implementation per slot, plus the platform and deployment, and launches them together. Examples: autonomous exploration, raceline tracking. |
| **Car namespace** | `/r0`, `/r1`, ... for our cars. `/opponents/or0`, `/opponents/or1`, ... for simulated opponents. TF frames are prefixed the same way (`r0/base_link`). |

## 3. Layers

```
  behaviour (stack_launcher)        e.g. autonomous_exploration.launch.py
     │  selects one implementation per slot + platform + deployment
     ▼
  ┌────────────── functional components (per car, /<ns>) ──────────────┐
  │ mapping → localization → perception → planning → controller        │
  └─────────────────────────────────────────────────────────┬──────────┘
                                                            │ drive_gate/auto_cmd
  ┌──────────────────── platform side (per car) ────────────▼──────────┐
  │ joystick ──────────────────────────────────────────► drive_gate    │
  │                                                         │ drive    │
  │ platform: real drivers | MVSim + adapter | f1tenth_gym bridge      │
  └────────────────────────────────────────────────────────────────────┘
```

### 3.1 Platform layer

The platform is not a slot. It is chosen by `platform:=` and is always present. Every platform
gives each car the same interface. This is the existing simulator contract from `CLAUDE.md`:

- `/<ns>/scan` (`sensor_msgs/LaserScan`, frame `<ns>/laser`)
- `/<ns>/odom` (`nav_msgs/Odometry`, child frame `<ns>/base_link`)
- `/<ns>/drive` (`ackermann_msgs/AckermannDriveStamped`), written **only** by `drive_gate`
- `/<ns>/collision` (`std_msgs/Bool`), simulators only
- TF `<ns>/base_link` and `<ns>/laser` on the global `/tf`

The platform side also always runs the **joystick** and **`drive_gate`** (§6), on every
platform and in every deployment.

| Platform | Package(s) | `odom` |
|---|---|---|
| `real` | `rbsf_bringup` + `vesc` submodule (VESC), `urg_node` (Hokuyo), `joy` | Wheel odometry in `<ns>/odom` |
| `mvsim` | `rbsf_mvsim` (`mvsim_adapter`) | Ground truth in `map` |
| `f1tenth` | `rbsf_f1tenth_gym` (`gym_bridge`) | Ground truth in `map` |

The simulators' `odom` is ground truth in the `map` frame, and the real car's is not. That
difference is why localization is a separate slot. The identity localizer is only valid where
`odom` is already in `map`.

### 3.2 Functional components

Every app instantiates **all five** slots. A slot the app does not need gets its identity
implementation. Slots are never missing. Every topic in the contract therefore always exists,
consumers never need "is there a perception node?" logic, and you can see the shape of every app
at a glance.

| Slot | Responsibility | Implementations (current / planned) | Identity |
|---|---|---|---|
| **mapping** | Provide `/map`. If it does SLAM, it also owns `map -> <ns>/odom`. | `slam_toolbox` (SLAM), `map_server` (known track from `rbsf_worlds`) | Publishes nothing new. The platform's `/map` (simulators) is used as is. |
| **localization** | Estimate the car's pose in `map`. | particle filter / AMCL on a known map | Relays the platform's `odom` (valid only where `odom` is in `map`: simulators, or alongside a SLAM mapper). |
| **perception** | Detect opponents and obstacles. | LiDAR opponent detection | Publishes an empty opponent list at a fixed rate. |
| **planning** | Produce the trajectory to follow: global line, overtaking, avoidance. | raceline / overtaking planner | Republishes the track's reference line (`rbsf_worlds`, `track:=`) as the trajectory. |
| **controller** | Turn trajectory + pose (or raw scan, for reactive controllers) into an Ackermann command. | `pure_pursuit`, `wall_following` | Publishes zero commands (speed 0, steering 0). The car stays still. |

**Mapping owns SLAM.** Exactly one component publishes `map -> <ns>/odom`:

- **SLAM** (exploration, teleop mapping). Mapping is `slam_toolbox` and publishes `/map` and
  `map -> <ns>/odom`. Localization is identity and composes that TF with `odom`.
- **Known map** (racing). Mapping is `map_server` (static `/map` from `rbsf_worlds`).
  Localization is a particle filter and owns `map -> <ns>/odom`.
- **Simulators with ground truth.** Mapping and localization are both identity.

### 3.3 Identity components

An identity implementation honours its slot's contract with **neutral values, not silence**.
An empty opponent list says "no opponents". A missing message could equally mean "perception
crashed". Because identity components publish, consumers can apply the same timeouts whichever
implementation is plugged in, and a stale input is always an error. The one exception is
mapping. Its identity publishes nothing, because `/map` (where a platform provides it) is
latched and already exists.

### 3.4 Behaviour layer

Behaviours live in the `stack_launcher` package (`src/stack_launcher/`, as in the legacy repo).
The package name deliberately has no `rbsf_` prefix: it is the stack's single entrypoint, not a
component. A behaviour does not do any of the components' work. It decides which
implementations run, with which configuration, on which platform and deployment (§5).

## 4. Interfaces

All topics are relative to the car namespace `/<ns>` unless they start with `/`. These contracts
are what make implementations interchangeable. An implementation may use extra internal topics
under its own prefix (e.g. `pure_pursuit/target`). It may not depend on another implementation's
internals.

### 4.1 Topic contracts

**Mapping**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `scan`, `odom` | LaserScan, Odometry | SLAM implementations only |
| out | `/map` | `nav_msgs/OccupancyGrid` | Transient local (latched) |
| out | TF `map -> <ns>/odom` | | SLAM implementations only |

**Localization**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `scan`, `odom`, `/map`, `/tf` | | |
| out | `localization/odom` | `nav_msgs/Odometry` | Frame `map`, child `<ns>/base_link`. At least the platform `odom` rate. |
| out | TF `map -> <ns>/odom` | | Only when mapping does not own it |

**Perception**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `scan`, `localization/odom` | | |
| out | `perception/opponents` | *proposed* `rbsf_msgs/OpponentArray` | Frame `map`. Empty array = none seen. Fixed rate. |

**Planning**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/map`, `localization/odom`, `perception/opponents` | | Plus the track's reference line from `rbsf_worlds` (`track:=`) |
| out | `planning/trajectory` | *proposed* `rbsf_msgs/Trajectory` (poses + target speeds) | Frame `map`. A closed loop in the driving direction for global lines. |

**Controller**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `planning/trajectory`, `localization/odom` | | Reactive controllers may use `scan` instead |
| out | `drive_gate/auto_cmd` | `ackermann_msgs/AckermannDriveStamped` | **Never `drive`.** At the control rate. `drive_gate` stops the car if it goes stale (§6). |

`nav_msgs/Path` cannot carry speeds, and no standard message describes opponents, so those two
types go in a future `rbsf_msgs` package (§9). Until it exists, implementations may use
`nav_msgs/Path` and a fixed speed parameter.

### 4.2 Rules for every implementation

- **Launch contract.** Each implementation exposes `launch/<impl>.launch.py` with the arguments
  `robot` (namespace), `platform`, `track` and `use_sim_time`, and puts its nodes in `/<robot>`.
  Behaviours include only that file.
- **Configuration.** Parameters live in YAML. Implementations keep separate `*_sim.yaml` and
  `*_real.yaml` files when values differ. The launch file picks one from `platform` (`mvsim` and
  `f1tenth` both map to `sim`).
- **Time.** `use_sim_time` is `true` on simulators and `false` on `real`. The behaviour sets it.
- **No hidden inputs.** A component reads only its slot's contract, its parameters and
  `rbsf_worlds` data. No hardcoded topics or paths.
- **Docstrings.** Every node module starts with its subscribed/published topics and an algorithm
  outline (see `CLAUDE.md`).

## 5. Behaviours and apps

An **app** = behaviour + `platform` + `deployment` + one implementation per slot.

```bash
ros2 launch stack_launcher autonomous_exploration.launch.py platform:=mvsim
ros2 launch stack_launcher autonomous_exploration.launch.py platform:=real deployment:=distributed
ros2 launch stack_launcher raceline_tracking.launch.py platform:=f1tenth track:=spielberg controller:=pure_pursuit
```

### 5.1 Behaviour launch files

- One `<behaviour>.launch.py` per behaviour in `stack_launcher/launch/`. Each declares
  `platform`, `deployment`, `robot`, `track` and one argument per slot (`mapping`,
  `localization`, `perception`, `planning`, `controller`). The behaviour sets the slot
  defaults, and the user can override any of them.
- A single **registry** in `stack_launcher` maps `slot -> implementation name -> (package, launch
  file)`. Adding an implementation means adding one registry entry, and every behaviour can then
  use it.
- Launch files use `OpaqueFunction` + `launch_setup(context)`. They validate `platform`,
  `deployment` and every slot value against the registry, and raise on unknown values. A
  combination where a consumer needs something no provider gives (e.g. a controller that needs a
  `map`-frame pose with `localization:=identity` on `platform:=real` and no SLAM mapper) should
  also be rejected. This is not checked yet, because no such consumer exists: wall following
  only reads `scan`.

Planned behaviours (legacy equivalents in parentheses):

| Behaviour | mapping | localization | perception | planning | controller |
|---|---|---|---|---|---|
| `teleop_mapping` (`00_teleop_slam`) | slam_toolbox | identity | identity | identity | identity |
| `wall_following` (`01_wall_following`) | identity | identity | identity | identity | wall_following |
| `autonomous_exploration` (`02_autonomous_slam`) | slam_toolbox | identity | identity | identity | wall_following |
| `raceline_tracking` (`03_centerline_tracking`) | map_server | particle filter (identity in sim) | identity | identity | pure_pursuit |

### 5.2 Deployment

| `deployment` | Car / simulator machine | Compute machine |
|---|---|---|
| `local` | `ros2 launch stack_launcher <behaviour>.launch.py platform:=<p>` runs everything | |
| `distributed` | `ros2 launch stack_launcher platform.launch.py platform:=real` | `ros2 launch stack_launcher <behaviour>.launch.py platform:=real deployment:=distributed` |

- `platform.launch.py` brings up only the platform side: drivers (or simulator), joystick and
  `drive_gate`. With `deployment:=local`, behaviours include this same file, so the platform side
  is identical in both deployments.
- With `deployment:=distributed`, the behaviour skips the platform side and launches only the
  five components. It still needs `platform` to choose configs and `use_sim_time`.
- Both machines must use the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` (see `make help`).
- **Startup order on the car:** start the platform side first and check that `drive_gate`
  reports STOPPED. Then start the compute side. Then arm (§6.4).

## 6. Safety: drive_gate (package rbsf_safety)

`drive_gate` is the only path from any command source to the actuators. It lives in its own
package, `rbsf_safety` (`src/base_system/rbsf_safety/`). The platform launch files include its
launch file, so it starts with every platform.

### 6.1 Invariants

- `drive_gate` is the only publisher on the actuator topics. Any extra publisher -> fatal error,
  stack shutdown (checked at runtime via `count_publishers()`). Controllers never publish on the
  platform topics; they are remapped to `drive_gate/auto_cmd`.
- **Fail-closed.** AUTONOMOUS requires positive, live evidence: joystick alive + armed + deadman
  held. Any missing signal is treated as "deadman released" -> STOPPED. If a joystick has never
  been received, the gate stays in STOPPED indefinitely. A priority mux alone is fail-open
  (lower-priority inputs pass when higher ones are silent) and is not acceptable as the safety
  mechanism (see §8).
- Manual always has priority over autonomous.
- STOPPED = speed 0, brake active, steering centered.
- Autonomous motion on the real platform always requires the deadman. This cannot be disabled.
- No input path bypasses `drive_gate`, in any platform or mode. This includes sim-only teleop
  configs that publish Twist directly; zero-speed steering in sim is handled by the sim adapter.
- `drive_gate` and the joystick run on the car side. With `deployment:=distributed`, losing the
  network produces AUTO_CMD_TIMEOUT -> STOPPED.

### 6.2 States

```
              ┌──────────── any stop reason ─────────────┐
              ▼                                          │
  start ─► STOPPED ── manual input + deadman ─► MANUAL ──┤
              │  ▲                                       │
              │  └──── manual input released ────────────┤
              └── armed + deadman held + fresh auto_cmd ─► AUTONOMOUS
```

- **STOPPED.** Initial state and the fallback from every other state. Publishes the STOPPED
  command (§6.1) at a fixed rate.
- **MANUAL.** The joystick drives. It pre-empts AUTONOMOUS at any time (manual priority).
- **AUTONOMOUS.** Forwards `drive_gate/auto_cmd` to `drive`, clipped to the configured speed and
  steering limits, while every condition holds.

Stop reasons, published in `drive_gate/state` so the operator always knows why the car is still:

| Reason | Trigger |
|---|---|
| `NO_JOY` | No joystick message ever received |
| `JOY_TIMEOUT` | Joystick messages stopped arriving |
| `DISARMED` | Autonomy not armed |
| `DEADMAN_RELEASED` | Deadman button not held |
| `AUTO_CMD_TIMEOUT` | `drive_gate/auto_cmd` stale (crashed controller, network loss) |
| `EXTRA_PUBLISHER` | Another publisher on an actuator topic (fatal, shuts the stack down) |

### 6.3 Interface (per car)

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `joy` | `sensor_msgs/Joy` | Manual axes, deadman, arm/disarm buttons |
| in | `drive_gate/auto_cmd` | `ackermann_msgs/AckermannDriveStamped` | From the controller slot |
| out | `drive` | `ackermann_msgs/AckermannDriveStamped` | Only publisher. On `real`, remapped to the VESC/Ackermann driver input. |
| out | `drive_gate/state` | *proposed* `rbsf_msgs/GateState` | State, stop reason, armed flag. Latched and periodic. |

Timeouts, limits and button mappings are parameters in `rbsf_safety/config/drive_gate_{real,sim}.yaml`.

### 6.4 Bringing up the real car

1. **First run of a new build or config: wheels off the ground.**
2. Start the platform side (`platform.launch.py platform:=real`). Check that `drive_gate/state`
   is STOPPED with `NO_JOY`.
3. Connect the joystick and check that the reason changes to `DISARMED`. Test manual driving.
4. Start the components (locally or on the compute node). Check that `drive_gate/auto_cmd`
   arrives at the expected rate.
5. Arm and hold the deadman: the car moves. Release it: the car must stop at once.
6. In `distributed`, unplug or block the network once while armed: the car must stop with
   `AUTO_CMD_TIMEOUT`.

Only after steps 5 and 6 pass may the car run on the ground.

### 6.5 Simulation

- `drive_gate` runs on simulators too, so apps behave identically on every platform.
- The deadman requirement is the launch argument `require_deadman`
  (`rbsf_safety/launch/drive_gate.launch.py`, passed through `platform.launch.py` and the
  behaviours). On simulators it may be `true` or `false` and defaults to `false`, so autonomous
  runs need no joystick. On `platform:=real` it must be `true`. `false` raises in the launch
  file before any driver starts (in `rbsf_bringup` too), and `drive_gate` itself refuses to
  start with it (§6.1).
- Keyboard and other sim-only teleop also go through the gate (as manual input), never straight
  to `cmd_vel` or `drive`.
- Simulated **opponents** are part of the simulated world, not of the stack under test. Their
  drivers (e.g. the Pure Pursuit demo cars under `/opponents/or<i>`) publish their `drive`
  directly. This exemption applies only to `/opponents/*` namespaces on simulators.

## 7. Repository layout and conventions

```
src/
  base_system/       # platforms and shared infrastructure
    vesc/                # submodule: f1tenth/vesc (jazzy), VESC driver + Ackermann/odom
    rbsf_bringup/        # real-car drivers launch
    rbsf_mvsim/          # MVSim platform
    rbsf_f1tenth_gym/    # f1tenth_gym platform (+ f1tenth_gym_ros submodule)
    rbsf_worlds/         # tracks: maps + reference lines
    rbsf_safety/         # drive_gate
  mapping/           # rbsf_slam_toolbox, ...
  localization/      # particle filter, ...
  perception/        # opponent detection, ...
  planning/          # planners, ...
  controller/        # rbsf_pure_pursuit, rbsf_wall_following, ...
  common/
    rbsf_identity/       # identity implementation of every slot
    rbsf_msgs/           # (planned) stack messages
  stack_launcher/    # behaviours, platform.launch.py, implementation registry
```

- Package names use the `rbsf_` prefix (except `stack_launcher`, §3.4). Licence Apache-2.0.
- One implementation per package (or a few closely related ones). The package depends only on
  its slot contract, `rbsf_worlds` and `rbsf_msgs`.
- Tests: pure algorithm code is kept separate from the ROS node (as in
  `rbsf_pure_pursuit/pure_pursuit.py`) and unit-tested. `rbsf_safety` gets exhaustive state
  machine tests.

## 8. Rationale and rejected alternatives

- **A priority mux (`ackermann_mux`) as the safety mechanism.** Rejected. A mux forwards the
  highest-priority *active* input. When teleop goes silent (joystick unplugged, out of range),
  the lower-priority autonomous input passes through. That is fail-open. The legacy stack worked
  this way, and its "autonomous deadman" published to `/dev/null`, so it enforced nothing.
  `drive_gate` inverts the logic: motion needs live, positive evidence (§6.1). A mux may still
  be used upstream of the gate for convenience, never instead of it.
- **EasyNav-style in-process plugins.** Rejected for now. Swapping at launch level keeps every
  implementation an ordinary, separately testable ROS node, works in Python today, and lets
  components be distributed across machines. Revisit together with the C++ decision (§9).
- **`drive_gate` inside `rbsf_bringup`.** Rejected. The gate must run on every platform
  (§6.1), but `rbsf_bringup` is real-hardware only and pulls driver dependencies the simulators
  do not need. A small dependency-free `rbsf_safety` package is easy to review, test and protect,
  and it ships in both the robot and dev images.
- **A fused "SLAM" slot.** Rejected. Keeping mapping and localization separate, with the rule
  that exactly one of them owns `map -> <ns>/odom` (§3.2), covers SLAM, known-map localization
  and ground-truth simulation with the same five slots.
- **Optional slots (an app omits what it does not need).** Rejected in favour of identity
  components (§3.3). Every contract topic always exists, and staleness is always an error.
- **Running `drive_gate` on the compute node.** Rejected. Safety must not depend on the network.
  Losing the link must stop the car (§6.1).

## 9. Open decisions

- Whether performance-critical nodes move to C++ (`ament_cmake`). This would also reopen
  in-process plugins (§8).
- The `rbsf_msgs` definitions: `Trajectory` (poses + speeds), `OpponentArray`, `GateState`.
- Whether behaviours need a runtime orchestrator (lifecycle management, or a state machine such
  as "explore one lap, save the map, switch to raceline tracking") beyond launch composition.
- How STOPPED's "brake active" maps to the VESC (brake current vs. zero speed). Today STOPPED
  sends speed 0, which the VESC's speed loop holds.
- Neutral outputs of the identity components (§3.3). Today the identities launch nothing,
  except localization (`odom_relay`, `odom` -> `localization/odom`). The empty opponent list,
  the reference-line trajectory and the zero command come with `rbsf_msgs` and their first
  consumer.
- Migration of what exists today:
  - `rbsf_pure_pursuit` loads its waypoint CSV itself and publishes `drive`. It should read
    `planning/trajectory` and `localization/odom` and publish `drive_gate/auto_cmd`.
  - The `pure_pursuit_demo.launch.py` files should become `stack_launcher` behaviours.
  - The simulators' teleop path (`cmd_vel` straight into MVSim / the gym bridge) should go
    through `drive_gate`.
