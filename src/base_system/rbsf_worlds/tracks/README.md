# Tracks

One directory per track: `<name>/<name>.yaml` + image (ROS map_server format) and the reference
lines the stack drives on, in the f1tenth_gym CSV formats:

- `<name>_centerline.csv`: `x_m, y_m, w_tr_right_m, w_tr_left_m`
- `<name>_raceline.csv`: `s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`

A yaml may instead name its centerline with a `centerline:` key (path relative to the yaml).

| Track | Length | Lines | Source |
|---|---|---|---|
| `levine` | 61.8 m (centerline) | centerline | f1tenth_gym_ros `maps/levine*` |
| `spielberg` | 338 m (raceline) | centerline, raceline | f1tenth_gym_ros `maps/Spielberg*` |

Both come from [f1tenth/f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) (branch
`dev-jazzy`, commit `f9ad174`), MIT License, Copyright (c) 2020 Hongrui Zheng. Spielberg files
were renamed to lowercase.
