# Tracks

One directory per track: `<name>/<name>.yaml` + image (ROS map_server format) and the reference
lines the stack drives on, in the f1tenth_gym CSV formats:

- `<name>_centerline.csv`: `x_m, y_m, w_tr_right_m, w_tr_left_m`
- `<name>_raceline.csv`: `s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`

A yaml may instead name its centerline with a `centerline:` key (path relative to the yaml).

A yaml may also list synthetic cones under `cones:`, in map coordinates: `[x, y]` for one cone,
`[x0, y0, x1, y1]` for a row of them (every 0.25 m, 20 cm wide, so no car fits through). Both
simulators draw them into the map they load (`rbsf_worlds/cones.py`); the track's own image is
left untouched. `levine` uses four rows to close the corridors that lead off its loop, so it is a
closed lap.

A yaml may also declare elevated roads under `bridges:` (`start`/`end` feet of the ramps on the
road axis, parallel to x or y, `deck: [s0, s1]` along it, `width`, `height`, and optional
`under_cones` rows `[x0, y0, x1, y1]` on the ground under the deck). The image stays 2D; MVSim
adds the ramps, the deck, its side rails and the cones under it in 3D, so cars drive over or
under it (`rbsf_mvsim/bridges.py`). The cones stop only cars at ground level, which is why they
are not `cones:` (those are walls in the map, at every height). f1tenth_gym has no 3D and drives
the bridge as a flat crossing, without the cones under it.

[`docs/MAP_CATALOG.md`](../../../../docs/MAP_CATALOG.md) lists every track with a preview, its
lengths, widths and driving direction. It is generated from these files: after adding or
regenerating a track, add its source to `scripts/tracks/catalog.py` and rerun it (dev container,
workspace sourced):

    python3 scripts/tracks/catalog.py

`levine` and `spielberg` come from [f1tenth/f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) (branch
`dev-jazzy`, commit `f9ad174`), MIT License, Copyright (c) 2020 Hongrui Zheng. Spielberg files
were renamed to lowercase.

`iros2026` is the IROS 2026 competition map. The organisers' drawing (thin wall lines on white)
is kept as `iros2026_original.{png,yaml}`; `iros2026.png`, its yaml and the centerline are
generated from it by `scripts/tracks/drawn_track.py` (rerun it if the organisers update the map):

    python3 scripts/tracks/drawn_track.py \
        src/base_system/rbsf_worlds/tracks/iros2026/iros2026_original.yaml \
        --name iros2026 --start 32.5 8.3 --ccw

`icra2026_classic` and `icra2026_master` are the ICRA 2026 competition maps, kept as
`<name>_original.{png,yaml}` (the organisers' PGM, converted losslessly). Unlike iros2026 they
are not a single loop, so `drawn_track.py` gets two centerline-only options; the generated map
keeps every wall, gap and branch:

- Dashed walls are the competition's *open track borders*: the LiDAR sees through the gaps but
  no car fits (gaps under 0.4 m). `--close-gaps 0.4` closes them for the centerline.
- `--block` segments cut off the branches the reference line does not take. Classic: the left
  side splits into two lanes and the line takes the inner one (outer lane blocked), and the
  music-note island is tied to the top wall.

Both are driven counter-clockwise from the bottom straight (corridor 1.4-3.5 m wide on Classic,
1.0-3.8 m on Master):

    python3 scripts/tracks/drawn_track.py \
        src/base_system/rbsf_worlds/tracks/icra2026_classic/icra2026_classic_original.yaml \
        --name icra2026_classic --start -4.87 -6.55 --ccw --close-gaps 0.4 \
        --block -16.65 -2.84 -14.41 -2.84 --block -4.68 5.90 -4.68 4.76

The Master map has a 0.7 m bridge (the real one is 60-80 cm) over a five-way crossing near its
centre: the west-east straight climbs onto it, the south-north road passes under it at ground
level. Its `bridges:` entry (in `_original` too, so regenerating keeps it) has ramps of 4.6 m
(west, 8.7 deg) and 5.3 m (east, 7.5 deg), as long as the straight allows, and a 3.4 m deck over
the crossing. Three cone rows stand under the deck: two across the straight (x = 5.20 and 7.95)
close the road below where the straight's walls stop, so the south-north road is fenced off
from the space under the ramps, and one along x = 6.69 continues the dashed lane divider
between the two diamonds, where the bridge road cuts it in the map. Cars at ground level go north
on its west side, round the northern diamond and come back south on its east side.

The line passes the crossing twice under the deck (north on the divider's west side, back south
on its east side) and once on it: at the right-hand junction it takes the left exit, the straight
over the bridge, rather than the long flat loop to the north. A self-crossing line is not a ring,
so it takes two steps. `drawn_track.py` gives the flat route (straight blocked on both sides of
the crossing, divider closed); `scripts/tracks/detour_track.py` then replaces, in place, its part
from the junction with the cheapest centred path over the bridge:

    T=src/base_system/rbsf_worlds/tracks/icra2026_master
    python3 scripts/tracks/drawn_track.py $T/icra2026_master_original.yaml \
        --name icra2026_master --start -5.35 0.03 --ccw --close-gaps 0.4 \
        --block 5.10 7.25 5.10 5.16 --block 8.14 7.25 8.14 5.16 \
        --block 6.72 7.63 6.72 4.21 --block 6.72 1.26 6.72 0.69
    python3 scripts/tracks/detour_track.py $T/icra2026_master.yaml \
        --name icra2026_master --out-dir $T \
        --leave 15.93 2.88 --rejoin -1.45 10.48 --close-gaps 0.4 \
        --block 16.31 7.63 18.78 7.63 --block 0.92 10.48 3.39 10.48 \
        --block 4.82 6.86 8.43 6.86 --block 4.82 5.72 8.43 5.72

`icra2025` is the ICRA 2025 competition map (Atlanta), from
[Noce99/AuRoRus](https://github.com/Noce99/AuRoRus) `maps/Atlanta_2025` (commit `5175016`),
MIT or Apache-2.0 at your option, Copyright (c) 2026 Enrico Mannocci. Their map is a bi-level
TIFF already filled like a SLAM map (corridor white, everything else black), kept losslessly as
`icra2025_original.{png,yaml}`. Their `info.json` uses a y-down frame anchored at the top-left
pixel corner, so the ROS origin is the bottom-left corner (`[-9.15, -12.85]`, the same numbers
since the map is centred) and their y coordinates change sign. The start is the midpoint of
their start/finish line, driven the way their `start_pose()` heads (in ROS coordinates `b` is on
the left; their own docs say `a`, in the mirrored frame): clockwise. The corridor is 0.9-2.4 m
wide:

    python3 scripts/tracks/drawn_track.py \
        src/base_system/rbsf_worlds/tracks/icra2025/icra2025_original.yaml \
        --name icra2025 --start 4.973 2.266 --cw

`eps_circuit` is the exception to the rule below: a car-mapped track kept so both simulators
can drive it by name. It is `session_track.py centerline --source cartographer` output, copied
from `maps/20261021_122444/track/`; the corridor is 1.0-1.9 m wide, driven counter-clockwise.

Tracks mapped by the car itself are not stored here: `scripts/maps/session_track.py` builds them
inside the mapping session (`maps/<stamp>/track/`, same file formats) from the SLAM map you pick
after inspection, and launch files take them as `track:=<path to the yaml>`. Both scripts share
the corridor and centerline code in `rbsf_worlds/centerline.py`.

The generated map keeps the corridor and fills everything else, with anti-aliased edges: the
LiDAR sees the same walls, but f1tenth_gym extracts 801 wall segments instead of 5992 and scans
about 6x faster. The centerline is the zero set of (distance to outer wall - distance to inner
wall), driven counter-clockwise from the bottom straight; the corridor is 1.7-3.5 m wide.
