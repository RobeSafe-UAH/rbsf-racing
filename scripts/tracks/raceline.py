#!/usr/bin/env python3
r"""
Optimise a raceline for a track's lap time and check that the car fits along it.

Input: a track, by name (src/base_system/rbsf_worlds/tracks/<name>) or by map yaml (e.g. a
car-mapped maps/<stamp>/track/<name>.yaml), with its centerline. The walls come from the map image
and the cones its yaml declares, not from the widths in the centerline CSV.

The line (rbsf_worlds/raceline.py) keeps the car's half width plus --margin from both walls and
minimises the squared curvature over the lap plus w times its length, with a penalty above the
steering limit tan(--max-steering) / --wheelbase. Pure minimum curvature (w = 0) straightens
bends the car takes flat out anyway and makes the lap longer; the shortest path (large w) makes
the corners slow. The tool tries every --length-weights value and keeps the line with the
shortest lap time under the speed profile (below) that passes the check. The check sweeps the
car's footprint (--rear / --front of the rear axle, --width) along it every 2 cm: every pose
must keep --margin from the walls, and the curvature must stay within the steering limit.
Stretches that come too close are pushed away from that wall and the line is optimised again.

Output, next to the yaml (or in --out-dir):
    <name>_raceline.csv   `s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2` (psi is the ROS
                          yaw), closed, starting next to the centerline's first point. The
                          planner picks it over the centerline (line:=auto), and vx caps the
                          controller's own speed profile. Only written when both checks pass,
                          unless --force.
    raceline.png          preview: the map, the centerline, the raceline coloured by vx, the
                          footprint every metre, and the poses that fail the check in red

vx is the curvature-based speed profile (rbsf_worlds/speed_profile.py, the one the path tracker
uses) with the --speed / --min-speed / --lateral / --accel / --decel limits; the defaults are
the real car's (pure_pursuit_real.yaml). --speed is capped at 3.0 m/s, the car's max_auto_speed
in drive_gate_real.yaml. The vehicle defaults are the real chassis.

Run it in the dev container with the workspace sourced:
    python3 scripts/tracks/raceline.py eps_circuit
    python3 scripts/tracks/raceline.py maps/<stamp>/track/<name>.yaml --margin 0.2
    python3 scripts/tracks/raceline.py spielberg --check-only \\
        src/base_system/rbsf_worlds/tracks/spielberg/spielberg_raceline.csv
Exit status: 0 when the line passes both checks (or --force wrote it anyway), 1 otherwise.
"""

import argparse
import pathlib
import sys

from matplotlib.collections import LineCollection
import matplotlib.pyplot as plt
import numpy as np
from rbsf_worlds.centerline import TrackError
from rbsf_worlds.paths import load_waypoints
from rbsf_worlds.raceline import (
    check_fit, Footprint, lap_time, optimise, raceline_rows, TrackMap, write_raceline)
from rbsf_worlds.speed_profile import speed_profile
from rbsf_worlds.tracks import find_waypoints

TRACKS = pathlib.Path(__file__).resolve().parents[2] / 'src/base_system/rbsf_worlds/tracks'
# Highest --speed accepted: drive_gate's max_auto_speed on the car (drive_gate_real.yaml).
SPEED_CAP = 3.0


def map_yaml_of(track):
    """Map yaml of a built-in track name (source tree, so outputs land in the repo) or a path."""
    built_in = TRACKS / track / f'{track}.yaml'
    if built_in.exists():
        return built_in
    path = pathlib.Path(track)
    if path.suffix in ('.yaml', '.yml') and path.exists():
        return path.resolve()
    names = sorted(p.name for p in TRACKS.iterdir() if (p / f'{p.name}.yaml').exists())
    sys.exit(f"No track '{track}': give a map yaml or one of {', '.join(names)}")


def print_report(title, report):
    """Print a FitReport under a title."""
    print(f'{title}:')
    for line in report.summary():
        print(f'  {line}')


def plot(path, track_map, centerline, xy, speeds, report, footprint, title):
    """Preview figure (module docstring)."""
    fig, ax = plt.subplots(figsize=(11, 8))
    ax.imshow(track_map.drivable, cmap='gray', extent=track_map.extent(), interpolation='nearest')
    loop = np.vstack([centerline, centerline[:1]])
    ax.plot(loop[:, 0], loop[:, 1], c='0.6', lw=1, label='centerline')
    closed = np.vstack([xy, xy[:1]])
    segments = np.stack([closed[:-1], closed[1:]], axis=1)
    line = LineCollection(segments, cmap='RdYlGn', linewidths=2.0)
    line.set_array(speeds)
    ax.add_collection(line)
    fig.colorbar(line, ax=ax, fraction=0.04, label='raceline vx (m/s)')

    corners = np.vstack([footprint.corners(), footprint.corners()[:1]])
    every = max(1, int(round(1.0 / np.mean(np.diff(report.pose_s)))))
    bad = report.pose_clearance < report.margin
    for k in sorted(set(range(0, len(report.poses), every)) | set(np.flatnonzero(bad)[::5])):
        x, y, yaw = report.poses[k]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        outline = corners @ rot.T + (x, y)
        ax.plot(outline[:, 0], outline[:, 1], lw=0.6,
                c='#d1495b' if bad[k] else '#3080ff', alpha=0.9 if bad[k] else 0.5)
    ax.plot(*report.max_curvature_xy, 'o', c='m', ms=6,
            label=f'max curvature {report.max_curvature:.2f} 1/m')
    ax.annotate('', xy=xy[5], xytext=xy[0],
                arrowprops={'arrowstyle': '->', 'color': '#d1495b', 'lw': 2})
    pad = 1.0
    ax.set_xlim(xy[:, 0].min() - pad, xy[:, 0].max() + pad)
    ax.set_ylim(xy[:, 1].min() - pad, xy[:, 1].max() + pad)
    ax.set_aspect('equal')
    ax.set_title(title)
    ax.legend(loc='lower right')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    """Optimise and check a raceline from the command line."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('track', help='Track name or map yaml (with a centerline).')
    parser.add_argument('--out-dir', type=pathlib.Path, help='Default: next to the yaml.')
    parser.add_argument('--check-only', type=pathlib.Path, metavar='CSV',
                        help='Check this raceline (or centerline) CSV instead of optimising.')
    parser.add_argument('--force', action='store_true',
                        help='Write the raceline even if it fails the check.')
    vehicle = parser.add_argument_group('vehicle (default: the real chassis)')
    vehicle.add_argument('--rear', type=float, default=0.05,
                         help='Chassis behind the rear axle (m).')
    vehicle.add_argument('--front', type=float, default=0.43,
                         help='Chassis ahead of the rear axle (m).')
    vehicle.add_argument('--width', type=float, default=0.31, help='Chassis width (m).')
    vehicle.add_argument('--margin', type=float, default=0.15,
                         help='Clearance kept between the chassis and the walls (m).')
    vehicle.add_argument('--wheelbase', type=float, default=0.33, help='Wheelbase (m).')
    vehicle.add_argument('--max-steering', type=float, default=0.4189,
                         help='Steering limit (rad).')
    profile = parser.add_argument_group('speed profile (vx; default: the real car)')
    profile.add_argument('--speed', type=float, default=SPEED_CAP,
                         help=f'Top speed (m/s), at most {SPEED_CAP}.')
    profile.add_argument('--min-speed', type=float, default=0.8, help='Floor (m/s).')
    profile.add_argument('--lateral', type=float, default=3.0,
                         help='Max lateral acceleration (m/s^2).')
    profile.add_argument('--accel', type=float, default=2.0, help='Max acceleration (m/s^2).')
    profile.add_argument('--decel', type=float, default=3.0, help='Max deceleration (m/s^2).')
    profile.add_argument('--curvature-spacing', type=float, default=0.5,
                         help='Span of the profile curvature estimate (m).')
    parser.add_argument('--step', type=float, default=0.1, help='Raceline point spacing (m).')
    parser.add_argument('--length-weights', type=float, nargs='+',
                        default=[0.0, 0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0], metavar='W',
                        help='Length weights to try (1/m^2): 0 is minimum curvature, large '
                             'values approach the shortest path. One value skips the sweep.')
    args = parser.parse_args()
    if not 0.0 < args.speed <= SPEED_CAP:
        parser.error(f'--speed must be in (0, {SPEED_CAP}] m/s (the car\'s max_auto_speed), '
                     f'got {args.speed}')

    map_yaml = map_yaml_of(args.track)
    name = map_yaml.stem
    out_dir = args.out_dir or map_yaml.parent
    track_map = TrackMap.load(map_yaml)
    centerline, _ = load_waypoints(find_waypoints(map_yaml, 'centerline'))
    footprint = Footprint(args.rear, args.front, args.width)

    def check(xy):
        return check_fit(track_map, xy, footprint, args.margin, args.wheelbase, args.max_steering)

    print(f'{name}: footprint {args.rear} m behind / {args.front} m ahead of the rear axle, '
          f'{args.width} m wide, margin {args.margin} m')
    print_report('centerline', check(centerline))
    if args.check_only:
        xy, _ = load_waypoints(args.check_only)
        report = check(xy)
        print_report(args.check_only.name, report)
        sys.exit(0 if report.ok else 1)

    def profile(xy):
        return speed_profile(xy, args.speed, args.min_speed, args.lateral, args.accel,
                             args.decel, args.curvature_spacing)

    print(f'centerline lap {lap_time(centerline, profile(centerline)):.2f} s (at the profile)')
    candidates = []  # (passes, lap time, weight, xy, report)
    for weight in args.length_weights:
        try:
            xy, report = optimise(track_map, centerline, footprint, args.margin, args.wheelbase,
                                  args.max_steering, args.step, length_weight=weight,
                                  log=lambda line: None)
        except TrackError as error:
            sys.exit(f'{name}: {error}. Lower --margin, or drive the centerline.')
        lap = lap_time(xy, profile(xy))
        candidates.append((report.ok, -lap, weight, xy, report))
        print(f'  length weight {weight:<5g}: {report.length:7.2f} m, {lap:6.2f} s, max curvature '
              f'{report.max_curvature:.3f} 1/m, clearance {report.min_clearance:.3f} m'
              + ('' if report.ok else ', FAILS the check'))
    # The fastest line that passes; if none passes, the fastest one (reported as failing).
    _, minus_lap, weight, xy, report = max(candidates, key=lambda c: (c[0], c[1]))
    lap = -minus_lap
    print_report(f'raceline (length weight {weight:g})', report)

    speeds = profile(xy)
    verdict = 'passes the check' if report.ok else 'FAILS the check'
    print(f'speed profile {speeds.min():.2f}-{speeds.max():.2f} m/s, {lap:.2f} s per lap '
          f'(at the profile)')

    out_dir.mkdir(parents=True, exist_ok=True)
    figure = out_dir / 'raceline.png'
    plot(figure, track_map, centerline, xy, speeds, report, footprint,
         f'{name}: raceline {report.length:.1f} m, {lap:.2f} s (centerline '
         f'{check(centerline).length:.1f} m, {lap_time(centerline, profile(centerline)):.2f} s), '
         f'{verdict}')
    print(f'preview: {figure}')
    if not report.ok and not args.force:
        sys.exit(f'{name}: the raceline fails the check, not written (--force writes it).')

    csv = out_dir / f'{name}_raceline.csv'
    write_raceline(
        csv, raceline_rows(xy, speeds, args.curvature_spacing),
        f'{name} raceline, {len(xy)} points, {report.length:.2f} m, {lap:.2f} s at vx; minimum '
        f'curvature + {weight:g} x length from the centerline (the fastest of length weights '
        f'{", ".join(f"{w:g}" for w in args.length_weights)}); {verdict} '
        f'(footprint {args.rear}/{args.front} m x {args.width} m, margin '
        f'{args.margin} m: min clearance {report.min_clearance:.3f} m; max curvature '
        f'{report.max_curvature:.3f} 1/m, steering limit {report.curvature_limit:.3f} 1/m). '
        f'vx: top {args.speed} m/s, min {args.min_speed} m/s, lateral {args.lateral} m/s^2, '
        f'accel {args.accel} / decel {args.decel} m/s^2. Generated by scripts/tracks/raceline.py.')
    print(f'raceline: {csv}')


if __name__ == '__main__':
    main()
