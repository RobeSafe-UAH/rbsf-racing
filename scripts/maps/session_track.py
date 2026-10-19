#!/usr/bin/env python3
"""
Turn a mapping session (rbsf_dual_slam map_saver) into an rbsf_worlds track, after inspection.

A session (maps/<YYYYMMDD_HHMMSS>/) holds one map per SLAM. No SLAM is reliably better than the
other (it depends on the track), so the map to build the track from is chosen per session, after
looking at both:

1. inspect <session>
   Checks every map and writes <session>/inspection.png (look at it) and inspection.yaml:
   - closed_loop: the cleaned free space is one corridor around one infield. Fails on an
     unclosed lap, a doubled wall (bad loop closure) or a large unknown hole in the corridor.
   - trajectory_in_corridor: fraction of the SLAM's own trajectory inside its own corridor. A
     tangled or drifted map does not contain the path that built it. Needs >= 0.95.
   - centerline: length and width range of the reference line it would give.
   - On the simulators, with the session's track in rbsf_worlds: error of the walls and of the
     trajectory against ground truth (informative; the pass/fail checks never need it).
   It recommends the passing map closest to ground truth, else the one with the most of its
   trajectory inside its corridor.

2. centerline <session> --source <map>
   Builds the track from the chosen map, which must have passed inspection (--force overrides
   it). Writes <session>/track/:
       <name>.png, <name>.yaml        cleaned map: corridor free, everything else occupied
       <name>_centerline.csv          x_m, y_m, w_tr_right_m, w_tr_left_m: closed loop in the
                                      driving direction from the start/finish line
       centerline.png                 preview
   The driving direction and the start/finish line come from the map's trajectory (where the
   car started), unless --ccw/--cw and --start are given. Frames are the source map's own.
   Use the result as a track: track:=<session>/track/<name>.yaml.

Run in the dev container with the workspace sourced (rbsf_worlds, numpy, scipy, opencv,
matplotlib):
    python3 scripts/maps/session_track.py inspect maps/20261001_183417
    python3 scripts/maps/session_track.py centerline maps/20261001_183417 --source cartographer
"""

import argparse
import math
import pathlib
import sys
import textwrap

import cv2
import matplotlib.pyplot as plt
import numpy as np
from rbsf_worlds.centerline import (
    centerline, corridor_from_slam_map, corridor_image, driving_direction, loop_length,
    pixel_frame, split_ring, TrackError, write_centerline)
from scipy.ndimage import binary_erosion, distance_transform_edt
import yaml

plt.switch_backend('Agg')  # headless: figures are only saved

MIN_IN_CORRIDOR = 0.95
COLORS = {'slam_toolbox': '#d1495b', 'cartographer': '#2e6fbd'}
FALLBACK_COLORS = ['#8a5cc2', '#e08a1e', '#3a9a8a']


# Loading ------------------------------------------------------------------------------------

class SessionMap:
    """One SLAM map of a session: image, metadata, pose in `map` and its own trajectory."""

    def __init__(self, session_dir, name, entry):
        self.name = name
        self.dir = session_dir / name
        self.spec = yaml.safe_load((self.dir / 'map.yaml').read_text())
        self.img = cv2.imread(str(self.dir / self.spec['image']), cv2.IMREAD_GRAYSCALE)
        self.resolution = self.spec['resolution']
        self.origin = self.spec['origin']
        self.pose_in_map = entry.get('pose_in_map')
        self.frame = entry.get('frame', '')
        path = self.dir / 'trajectory.csv'
        self.trajectory = (np.loadtxt(path, delimiter=',', comments='#', ndmin=2)
                           if path.exists() else np.empty((0, 4)))

    def to_pixels(self, xy):
        scale, offset = pixel_frame(self.img.shape[0], self.resolution, self.origin)
        return np.round((np.asarray(xy) - offset) / scale).astype(int)  # (col, row)

    def wall_points(self):
        """Occupied cells, in this map's frame."""
        rows, cols = np.nonzero(self.img < 100)
        scale, offset = pixel_frame(self.img.shape[0], self.resolution, self.origin)
        return np.c_[cols, rows] * scale + offset


def load_session(session_dir):
    session = yaml.safe_load((session_dir / 'session.yaml').read_text())
    maps = {name: SessionMap(session_dir, name, entry)
            for name, entry in session['maps'].items()}
    return session, maps


def from_map_frame(xy, pose):
    """Inverse of to_map_frame: points of `map` expressed in the frame whose pose is `pose`."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = np.asarray(xy, float)[:, 0] - x, np.asarray(xy, float)[:, 1] - y
    return np.c_[c * dx + s * dy, -s * dx + c * dy]


def to_map_frame(xy, pose):
    """Points of a map frame expressed in `map`, given that frame's pose (x, y, yaw) in map."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    xy = np.asarray(xy, float)
    return np.c_[x + c * xy[:, 0] - s * xy[:, 1], y + s * xy[:, 0] + c * xy[:, 1]]


def color(name, index):
    return COLORS.get(name, FALLBACK_COLORS[index % len(FALLBACK_COLORS)])


# Ground truth (simulators) -------------------------------------------------------------------

class GroundTruth:
    """
    The session track's map from rbsf_worlds and the recorded ground-truth path.

    Both are in the simulator's world frame, whose pose in `map` is `pose_in_map` (the session's
    ground_truth_pose_in_map; identity for older sessions, whose primary SLAM anchored `map` at
    the world origin). Every method takes and returns `map` coordinates.
    """

    def __init__(self, map_yaml, path_csv, pose_in_map=(0.0, 0.0, 0.0)):
        self.pose_in_map = tuple(pose_in_map)
        spec = yaml.safe_load(map_yaml.read_text())
        img = cv2.imread(str(map_yaml.parent / spec['image']), cv2.IMREAD_GRAYSCALE)
        occupied = (255 - img.astype(float)) / 255.0 > spec['occupied_thresh']
        # Wall surface only: maps filled solid outside the track would otherwise put every
        # point inside the fill at 0 m from a wall.
        self.walls = occupied & ~binary_erosion(occupied, border_value=1)
        self.resolution, self.origin, self.shape = spec['resolution'], spec['origin'], img.shape
        self.distance = distance_transform_edt(~self.walls) * self.resolution
        self.path = (np.loadtxt(path_csv, delimiter=',', comments='#', ndmin=2)
                     if path_csv.exists() else None)

    def wall_points(self):
        rows, cols = np.nonzero(self.walls)
        scale, offset = pixel_frame(self.shape[0], self.resolution, self.origin)
        return to_map_frame(np.c_[cols, rows] * scale + offset, self.pose_in_map)

    def wall_error(self, points):
        """Distance (m) of map-frame points to the nearest ground-truth wall."""
        points = from_map_frame(points, self.pose_in_map)
        scale, offset = pixel_frame(self.shape[0], self.resolution, self.origin)
        cols, rows = np.round((points - offset) / scale).astype(int).T
        ok = (rows >= 0) & (rows < self.shape[0]) & (cols >= 0) & (cols < self.shape[1])
        return self.distance[rows[ok], cols[ok]]

    def path_error(self, trajectory_map):
        """(t, error) of a map-frame trajectory (t, x, y) against the ground-truth path."""
        gt = self.path[np.argsort(self.path[:, 0])]
        t = trajectory_map[:, 0]
        keep = (t >= gt[0, 0]) & (t <= gt[-1, 0])
        t, xy = t[keep], from_map_frame(trajectory_map[keep, 1:3], self.pose_in_map)
        ref = np.c_[np.interp(t, gt[:, 0], gt[:, 1]), np.interp(t, gt[:, 0], gt[:, 2])]
        return t - gt[0, 0], np.hypot(*(xy - ref).T)


def find_ground_truth(session, session_dir):
    if session.get('platform') in (None, '', 'real') or not session.get('track'):
        return None
    try:
        from rbsf_worlds.tracks import resolve_map_yaml
        map_yaml = resolve_map_yaml(session['track'])
    except (ImportError, FileNotFoundError, LookupError):
        return None
    return GroundTruth(map_yaml, session_dir / session.get('ground_truth', 'ground_truth.csv'),
                       session.get('ground_truth_pose_in_map', (0.0, 0.0, 0.0)))


# Centerline from one map ----------------------------------------------------------------------

def extract_track(m, step=0.1, smooth=0.25, ccw=None, start=None):
    """Corridor and centerline of a session map; direction/start from its trajectory by default."""
    corridor = corridor_from_slam_map(m.img, m.resolution)
    infield, outside = split_ring(corridor)
    if ccw is None:
        if len(m.trajectory) < 10:
            raise TrackError('no trajectory to take the driving direction from: pass --ccw/--cw')
        ccw = driving_direction(m.trajectory[:, 1:3])
    if start is None:
        if not len(m.trajectory):
            raise TrackError('no trajectory to take the start line from: pass --start X Y')
        start = m.trajectory[0, 1:3]
    line = centerline(corridor, infield, outside, m.resolution, m.origin, step, smooth,
                      np.asarray(start, float), ccw)
    return corridor, line, ccw


# inspect -------------------------------------------------------------------------------------

def inspect_map(m, ground_truth):
    report = {'checks': {}, 'reasons': []}
    corridor = line = None
    try:
        corridor = corridor_from_slam_map(m.img, m.resolution)
        split_ring(corridor)
        report['checks']['closed_loop'] = True
    except TrackError as error:
        report['checks']['closed_loop'] = False
        report['reasons'].append(f'closed_loop: {error}')

    if corridor is not None and len(m.trajectory):
        # 0.2 m of tolerance: the car drives close to the wall it follows.
        tolerance = max(1, int(round(0.2 / m.resolution)))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * tolerance + 1,) * 2)
        inside = cv2.dilate(corridor.astype(np.uint8), kernel).astype(bool)
        cols, rows = m.to_pixels(m.trajectory[:, 1:3]).T
        ok = (rows >= 0) & (rows < inside.shape[0]) & (cols >= 0) & (cols < inside.shape[1])
        fraction = float(np.mean(np.where(ok, inside[rows.clip(0, inside.shape[0] - 1),
                                                     cols.clip(0, inside.shape[1] - 1)], False)))
        report['checks']['trajectory_in_corridor'] = round(fraction, 3)
        if fraction < MIN_IN_CORRIDOR:
            report['reasons'].append(
                f"trajectory_in_corridor: {fraction:.0%} of the SLAM's own path is inside its "
                f'corridor (needs {MIN_IN_CORRIDOR:.0%}): the map does not match its trajectory')
    elif not len(m.trajectory):
        report['checks']['trajectory_in_corridor'] = None  # older session: not recorded

    if report['checks']['closed_loop'] and len(m.trajectory) >= 10:
        _, line, ccw = extract_track(m)
        widths = line[:, 2] + line[:, 3]
        report['centerline'] = {
            'length_m': round(loop_length(line), 2), 'points': len(line),
            'direction': 'counter-clockwise' if ccw else 'clockwise',
            'width_min_m': round(float(widths.min()), 2),
            'width_max_m': round(float(widths.max()), 2)}

    if ground_truth is not None and m.pose_in_map is not None:
        error = ground_truth.wall_error(to_map_frame(m.wall_points(), m.pose_in_map))
        gt = {'wall_error_mean_m': round(float(error.mean()), 3),
              'wall_error_p95_m': round(float(np.percentile(error, 95)), 3),
              'walls_within_20cm': round(float(np.mean(error < 0.2)), 3)}
        if ground_truth.path is not None and len(m.trajectory):
            traj = np.c_[m.trajectory[:, 0], to_map_frame(m.trajectory[:, 1:3], m.pose_in_map)]
            _, e = ground_truth.path_error(traj)
            if len(e):
                gt['trajectory_rmse_m'] = round(float(np.sqrt(np.mean(e ** 2))), 3)
                gt['trajectory_max_m'] = round(float(e.max()), 3)
        report['ground_truth'] = gt

    report['passed'] = not report['reasons']
    return report, corridor, line


def recommend(reports):
    passed = [n for n, r in reports.items() if r['passed']]
    if not passed:
        return None
    if all('ground_truth' in reports[n] for n in passed):
        return min(passed, key=lambda n: reports[n]['ground_truth']['wall_error_mean_m'])
    return max(passed, key=lambda n: reports[n]['checks'].get('trajectory_in_corridor') or 0)


def plot_inspection(path, session, maps, reports, corridors, lines, ground_truth):
    names = list(maps)
    has_errors = ground_truth is not None and ground_truth.path is not None
    rows = 2
    fig, axes = plt.subplots(rows, len(names), figsize=(8 * len(names), 6.5 * rows),
                             squeeze=False)
    for i, name in enumerate(names):
        m, r, ax = maps[name], reports[name], axes[0, i]
        h, w = m.img.shape
        x0, y0 = m.origin[0], m.origin[1]
        ax.imshow(m.img, cmap='gray', vmin=0, vmax=255,
                  extent=(x0, x0 + w * m.resolution, y0, y0 + h * m.resolution))
        if corridors.get(name) is not None:
            ax.contour(np.flipud(corridors[name]).astype(float), levels=[0.5], colors='#f4a261',
                       linewidths=0.8, extent=(x0, x0 + w * m.resolution, y0,
                                               y0 + h * m.resolution))
        if len(m.trajectory):
            ax.plot(m.trajectory[:, 1], m.trajectory[:, 2], c=color(name, i), lw=0.8,
                    label='own trajectory')
            ax.plot(*m.trajectory[0, 1:3], 'o', c=color(name, i), ms=6, label='start')
        if lines.get(name) is not None:
            loop = np.vstack([lines[name][:, :2], lines[name][:1, :2]])
            ax.plot(loop[:, 0], loop[:, 1], c='#2a9d55', lw=1.2, label='centerline')
        verdict = 'PASS' if r['passed'] else 'FAIL'
        detail = '\n'.join(textwrap.fill(reason, 90) for reason in r['reasons']) or ', '.join(
            f'{k}: {v}' for k, v in r['checks'].items())
        ax.set_title(f'{name} ({m.frame}): {verdict}\n{detail}', fontsize=9,
                     color='#1b7a3e' if r['passed'] else '#b3261e')
        ax.legend(loc='lower right', fontsize=8)
        ax.set_xlabel('x (m, own frame)')
        ax.set_ylabel('y (m)')
        cols, rows_ = np.nonzero(m.img.T < 250)  # zoom on the mapped area
        if len(cols):
            pad = 2.0
            ax.set_xlim(x0 + cols.min() * m.resolution - pad, x0 + cols.max() * m.resolution + pad)
            ax.set_ylim(y0 + (h - rows_.max()) * m.resolution - pad,
                        y0 + (h - rows_.min()) * m.resolution + pad)

    ax = axes[1, 0]
    if ground_truth is not None:
        gt_pts = ground_truth.wall_points()
        ax.scatter(gt_pts[:, 0], gt_pts[:, 1], s=0.2, c='#b9b9b9', label='ground truth walls')
    for i, name in enumerate(names):
        m = maps[name]
        if m.pose_in_map is not None:
            pts = to_map_frame(m.wall_points(), m.pose_in_map)
            ax.scatter(pts[:, 0], pts[:, 1], s=0.2, c=color(name, i), label=f'{name} walls')
    ax.set_title('All maps in the map frame' + (' over ground truth' if ground_truth else ''))
    ax.set_aspect('equal')
    ax.legend(loc='lower right', markerscale=20, fontsize=8)

    for ax in axes[1, 1:]:
        if has_errors:
            for i, name in enumerate(names):
                m = maps[name]
                if len(m.trajectory) and m.pose_in_map is not None:
                    traj = np.c_[m.trajectory[:, 0],
                                 to_map_frame(m.trajectory[:, 1:3], m.pose_in_map)]
                    t, e = ground_truth.path_error(traj)
                    ax.plot(t, e, c=color(name, i), label=name)
            ax.set_title('Trajectory error vs ground truth')
            ax.set_xlabel('time (s)')
            ax.set_ylabel('error (m)')
            ax.grid(alpha=0.3)
            ax.legend()
        else:
            ax.axis('off')
            ax.text(0.5, 0.5, 'No ground truth (real car): judge the maps above by eye.',
                    ha='center', va='center')
    for ax in axes[0]:
        ax.set_aspect('equal')
    fig.suptitle(f"Session {session.get('stamp', '')}: {session.get('platform', '')} "
                 f"{session.get('track', '')}", fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def cmd_inspect(args):
    session, maps = load_session(args.session)
    ground_truth = find_ground_truth(session, args.session)
    reports, corridors, lines = {}, {}, {}
    for name, m in maps.items():
        reports[name], corridors[name], lines[name] = inspect_map(m, ground_truth)
    best = recommend(reports)
    result = {'session': session.get('stamp'), 'maps': reports, 'recommended': best,
              'ground_truth': ground_truth is not None}
    (args.session / 'inspection.yaml').write_text(yaml.safe_dump(result, sort_keys=False))
    plot_inspection(args.session / 'inspection.png', session, maps, reports, corridors, lines,
                    ground_truth)

    for name, r in reports.items():
        print(f"{name}: {'PASS' if r['passed'] else 'FAIL'}  {r['checks']}")
        for reason in r['reasons']:
            print(f'    {reason}')
        if 'centerline' in r:
            print(f"    centerline: {r['centerline']}")
        if 'ground_truth' in r:
            print(f"    ground truth: {r['ground_truth']}")
    print(f"Look at {args.session / 'inspection.png'}.")
    if best:
        print(f'Recommended: {best}. Next:\n    python3 {sys.argv[0]} centerline {args.session} '
              f'--source {best}')
    else:
        print('No map passed: drive another lap or fix the SLAM, or --force a source after '
              'checking it by eye.')
    return 0


# centerline ----------------------------------------------------------------------------------

def cmd_centerline(args):
    session, maps = load_session(args.session)
    if args.source not in maps:
        sys.exit(f"Unknown source '{args.source}'. Maps in this session: {', '.join(maps)}")
    inspection_path = args.session / 'inspection.yaml'
    inspection = (yaml.safe_load(inspection_path.read_text())
                  if inspection_path.exists() else None)
    if not args.force:
        if inspection is None:
            sys.exit(f'Inspect the session first: python3 {sys.argv[0]} inspect {args.session} '
                     '(or --force)')
        if not inspection['maps'][args.source]['passed']:
            reasons = '; '.join(inspection['maps'][args.source]['reasons'])
            sys.exit(f'{args.source} failed inspection ({reasons}). Pick another source, or '
                     '--force after checking it by eye.')

    m = maps[args.source]
    ccw = True if args.ccw else False if args.cw else None
    try:
        corridor, line, ccw = extract_track(m, args.step, args.smooth, ccw, args.start)
    except TrackError as error:
        sys.exit(f'{args.source}: {error}')

    name = args.name or f"{session.get('track') or 'track'}_{session.get('stamp', 'session')}"
    out = args.session / 'track'
    out.mkdir(exist_ok=True)
    cv2.imwrite(str(out / f'{name}.png'), corridor_image(corridor))
    spec = {'image': f'{name}.png', 'resolution': m.resolution, 'origin': list(m.origin),
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196,
            'centerline': f'{name}_centerline.csv'}
    (out / f'{name}.yaml').write_text(yaml.safe_dump(spec, sort_keys=False,
                                                     default_flow_style=None))
    length = loop_length(line)
    verdict = ('forced' if args.force else 'passed inspection')
    write_centerline(
        out / f'{name}_centerline.csv', line,
        f'{name} centerline, {len(line)} points, {length:.2f} m, '
        f"{'counter-clockwise' if ccw else 'clockwise'}; finish line = first point. "
        f"Generated by scripts/maps/session_track.py from session {session.get('stamp')}, "
        f'source {args.source} (frame {m.frame}, {verdict}).')

    fig, ax = plt.subplots(figsize=(10, 7))
    ax.imshow(corridor_image(corridor), cmap='gray', extent=(
        m.origin[0], m.origin[0] + m.img.shape[1] * m.resolution,
        m.origin[1], m.origin[1] + m.img.shape[0] * m.resolution))
    loop = np.vstack([line[:, :2], line[:1, :2]])
    ax.plot(loop[:, 0], loop[:, 1], c='#2a9d55', lw=1.5, label='centerline')
    ax.annotate('', xy=line[5, :2], xytext=line[0, :2],
                arrowprops={'arrowstyle': '->', 'color': '#d1495b', 'lw': 2})
    ax.plot(*line[0, :2], 'o', c='#d1495b', label='start/finish, driving direction')
    widths = line[:, 2] + line[:, 3]
    ax.set_title(f'{name}: {length:.1f} m, width {widths.min():.2f}-{widths.max():.2f} m '
                 f'(source {args.source})')
    ax.legend(loc='lower right')
    ys, xs = np.nonzero(corridor)
    pad = 2.0
    ax.set_xlim(m.origin[0] + xs.min() * m.resolution - pad,
                m.origin[0] + xs.max() * m.resolution + pad)
    ax.set_ylim(m.origin[1] + (m.img.shape[0] - ys.max()) * m.resolution - pad,
                m.origin[1] + (m.img.shape[0] - ys.min()) * m.resolution + pad)
    fig.tight_layout()
    fig.savefig(out / 'centerline.png', dpi=110)
    plt.close(fig)

    print(f'{name}: {len(line)} points, {length:.2f} m, width {widths.min():.2f}-'
          f'{widths.max():.2f} m, {"counter-clockwise" if ccw else "clockwise"} '
          f'(source {args.source}, {verdict})')
    print(f'Track: {(out / f"{name}.yaml").resolve()}  (preview: {out / "centerline.png"})')
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    inspect = sub.add_parser('inspect', help='Check every map of a session (step 1).')
    inspect.add_argument('session', type=pathlib.Path, help='Session directory (maps/<stamp>).')
    inspect.set_defaults(func=cmd_inspect)

    build = sub.add_parser('centerline', help='Build the track from one map (step 2).')
    build.add_argument('session', type=pathlib.Path, help='Session directory (maps/<stamp>).')
    build.add_argument('--source', required=True,
                       help='Map to build the track from (slam_toolbox, cartographer, ...).')
    build.add_argument('--name', help='Track name. Default: <track>_<session stamp>.')
    build.add_argument('--force', action='store_true',
                       help='Use the source even if it failed (or skipped) inspection.')
    direction = build.add_mutually_exclusive_group()
    direction.add_argument('--ccw', action='store_true', help='Override: counter-clockwise.')
    direction.add_argument('--cw', action='store_true', help='Override: clockwise.')
    build.add_argument('--start', type=float, nargs=2, metavar=('X', 'Y'),
                       help='Override the start/finish point (m, source map frame).')
    build.add_argument('--step', type=float, default=0.1, help='Centerline spacing (m).')
    build.add_argument('--smooth', type=float, default=0.25,
                       help='Centerline smoothing sigma (m).')
    build.set_defaults(func=cmd_centerline)

    args = parser.parse_args()
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
