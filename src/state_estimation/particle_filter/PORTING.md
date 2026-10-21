# Port to the RBSF Racing localization slot

Vendored from [f1tenth/particle_filter](https://github.com/f1tenth/particle_filter), branch
`humble-devel`, commit `672ea43`. The license stays MIT (`LICENSE`); this package is separate
from the Apache-2.0 rest of the repository. Ray casting uses the vendored `range_libc`.

Kept: the MCL algorithm. That is multinomial resampling, upstream's motion model (odometry delta
in each particle's frame plus Gaussian dispersion), the precomputed beam-model table (`z_hit`,
`z_short`, `z_max`, `z_rand`, `sigma_hit`) evaluated in RangeLibc, the squash factor, and the
parameter names and defaults.

Changes:

- **Frames and TF.** Particles are `<ns>/base_link` poses. The sensor model casts from each
  particle's laser pose, using the static `base_link -> laser` offset (upstream treated
  particles as the laser). The node publishes `map -> <ns>/odom = estimate * odom^-1`,
  post-dated by `transform_tolerance` as AMCL does. Upstream published `map -> laser`, which
  gives the laser a second parent in a tree where `base_link` already owns it.
- **Contracts.** Relative topics (`scan`, `odom`, `initialpose`, `pf/...`) for namespaced cars.
  The map comes from the latched `/map` topic, whichever mapping publishes it. Upstream made a
  blocking `GetMap` call to `/map_server/map` in `__init__`. `localization/odom` is not
  published here: the launch file adds `rbsf_identity`'s `odom_relay`, which gives it at the
  odometry rate.
- **Update timing.** One filter update per scan, with the odometry change since the previous
  update. Upstream updated on every odometry message.
- **Initialization.** `initial_pose: [x, y, yaw]` seeds the particles. The launch defaults it to
  the start of the track's reference line. Without it, particles spread over the free cells (as
  upstream). `initialpose` re-seeds at any time. `/clicked_point` global re-initialization is
  gone.
- **Fixes.** The odometry yaw delta and the particle yaw are wrapped to [-pi, pi). The pose
  estimate uses a circular mean for yaw (a linear mean breaks for particles on both sides of
  +-pi). NaN and inf ranges are set to `max_range`. Weights that underflow to zero reset to
  uniform instead of producing NaN. Parameters have typed defaults.
- **Dependencies.** `tf_transformations`/`transforms3d` and the nav2 map server launch are gone.
  Math lives in `particle_filter/mcl.py` (no ROS imports, unit-tested in `test/test_mcl.py`).
  Upstream's `utils.py`, the rviz config and its maps were dropped.
- Only RangeLibc sensor-model variants 2 (`calc_range_repeat_angles` + `eval_sensor_model`,
  every range method) and 3 (one call) are kept. Variants 0, 1 and 4 were demonstrations or
  CDDT-only.
- **Configs.** `config/particle_filter_{sim,real}.yaml`. `range_method: cddt` (CPU: there is no
  `rmgpu` build yet), 2000 particles, every 18th of the Hokuyo's 1081 beams. The real config
  keeps upstream's motion dispersion for RF2O drift. The sim config uses less, because MVSim's
  odometry is ground truth.
