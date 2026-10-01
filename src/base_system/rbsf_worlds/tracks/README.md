# Tracks

One directory per track: `<name>/<name>.yaml` + image (ROS map_server format) and the reference
lines the stack drives on, in the f1tenth_gym CSV formats:

- `<name>_centerline.csv`: `x_m, y_m, w_tr_right_m, w_tr_left_m`
- `<name>_raceline.csv`: `s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`

A yaml may instead name its centerline with a `centerline:` key (path relative to the yaml).

| Track | Length | Lines | Source |
|---|---|---|---|
| `levine` | 61.8 m (centerline) | centerline | f1tenth_gym_ros `maps/levine*` |
| `spielberg` | 343.3 m (centerline), 338.1 m (raceline) | centerline, raceline | f1tenth_gym_ros `maps/Spielberg*` |
| `iros2026` | 209.4 m (centerline) | centerline | IROS 2026 RoboRacer competition map |

`levine` and `spielberg` come from [f1tenth/f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) (branch
`dev-jazzy`, commit `f9ad174`), MIT License, Copyright (c) 2020 Hongrui Zheng. Spielberg files
were renamed to lowercase.

`iros2026` is the IROS 2026 competition map. The organisers' drawing (thin wall lines on white)
is kept as `iros2026_original.{png,yaml}`; `iros2026.png`, its yaml and the centerline are
generated from it by `scripts/tracks/drawn_track.py` (rerun it if the organisers update the map):

    python3 scripts/tracks/drawn_track.py \
        src/base_system/rbsf_worlds/tracks/iros2026/iros2026_original.yaml \
        --name iros2026 --start 32.5 8.3 --ccw

Tracks mapped by the car itself are not stored here: `scripts/maps/session_track.py` builds them
inside the mapping session (`maps/<stamp>/track/`, same file formats) from the SLAM map you pick
after inspection, and launch files take them as `track:=<path to the yaml>`. Both scripts share
the corridor and centerline code in `rbsf_worlds/centerline.py`.

The generated map keeps the corridor and fills everything else, with anti-aliased edges: the
LiDAR sees the same walls, but f1tenth_gym extracts 801 wall segments instead of 5992 and scans
about 6x faster. The centerline is the zero set of (distance to outer wall - distance to inner
wall), driven counter-clockwise from the bottom straight; the corridor is 1.7-3.5 m wide.
