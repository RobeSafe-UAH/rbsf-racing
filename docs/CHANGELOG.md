# Changelog

Development ran from 2026-10-01 to 2026-10-27. Items marked *not validated* have no real-car run
yet.

## [1.0.0] - 2026-10-27

Full rewrite and port of `rbsf-racing-irobocity` to ROS 2 Jazzy, with two behaviours:
`autonomous_exploration` and `raceline_tracking`.

### Behaviours

- **`autonomous_exploration`**: reactive wall following (or Follow-the-Gap) while the mapping slot
  builds the map. Runs on `platform:=real|mvsim|f1tenth` and `deployment:=local|distributed`.
  Validated on the real car.
- **`raceline_tracking`**: follows the track's reference line with Pure Pursuit or MAP, on a
  curvature-based speed profile. On the car it localizes on the track's known map (RF2O odometry
  + particle filter). Centerline following is validated on the car. Raceline following is
  *not validated*.

### Added

- **Platform and safety**
  - Docker workflow (`make build|run|attach`) for ROS 2 Jazzy, `dev` and `robot` flavors, with
    cached layers and a host-independent user.
  - Real-car bringup (`rbsf_bringup`): VESC, Hokuyo, joystick, RF2O laser odometry (vendored
    Jazzy port, GPL-3.0).
  - `drive_gate` (`rbsf_safety`): the only publisher to the actuators. Deadman, arming and
    fail-closed timeouts.
- **Simulators**: MVSim (`rbsf_mvsim`) and f1tenth_gym (`rbsf_f1tenth_gym`) as drop-in
  replacements for the car, with multi-opponent Pure Pursuit demos and a 3D bridge for the ICRA 2026
  tracks.
- **Slots**
  - Mapping: DualSLAM (`rbsf_dual_slam`), Cartographer primary and SLAM Toolbox secondary, with
    map saving, and `map_server` for known maps (`rbsf_map_manager`).
  - Localization: particle filter on RangeLibc (ports of f1tenth/particle_filter and RangeLibc).
  - Identity implementations for every slot.
- **Controllers**: wall following (centring, one-boundary following, hairpin rounding),
  Follow-the-Gap with dead-end bay avoidance, and `rbsf_path_tracking` (Pure Pursuit and MAP, with
  a curvature-based speed profile and per-track tuning).
- **Tracks and tools** (`rbsf_worlds`)
  - Tracks: Levine, Spielberg, EPS circuit and oval, ICRA 2025, ICRA 2026 classic and master,
    IROS 2026, IEEE IV 2026.
  - Session-to-track pipeline (`scripts/maps/session_track.py`): inspection, then centerline.
  - Minimum-curvature raceline optimiser with a swept-footprint and steering-limit check.
  - Generated map catalog (`docs/MAP_CATALOG.md`).
- **Tooling**: `rbsf_telemetry` live web plots, `rviz_slam` view with the reference line coloured
  by speed.
- **Docs**: `docs/DESIGN_PHILOSOPHY.md`, `docs/RACE_DAY.md`.

### Known limitations

- Raceline following on the real car is *not validated*; no raceline is committed (the car's
  footprint is still to be measured).
- `max_lateral_acceleration` (3.0) and MAP's `understeer_gradient` are not identified on the car.
- Wall following crashes on levine's inner-wall recess and on eps with 80 ms of extra steering
  lag (it passes up to 60 ms).


