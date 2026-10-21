#!/usr/bin/env python3
"""
Make a track variant whose reference line takes a detour: another route between two points.

For lines the ring algorithm of drawn_track.py cannot give, e.g. a route that crosses itself on a
bridge (icra2026_master: drawn_track.py gives the flat route, this script takes the bridge
instead). The variant shares the base track's map image and copies its other keys (bridges,
cones). With the base track's name and directory, it replaces the base line in place.

Input: the base track yaml (with its reference line), the points where the detour leaves and
rejoins the base line, and --block segments that shape the detour corridor (cut off the base
route and anything the detour must not cut through, as in drawn_track.py).

Output, in --out-dir (default: tracks/<name>/ next to the base track):
    <name>.yaml                the base yaml, image pointing at the base track's, `centerline:` set
    <name>_centerline.csv      the base line from the start to --leave, the detour, then the base
                               line from --rejoin back to the start (same finish line)

Detour: the cheapest path in the detour corridor, each step costing its length over the squared
distance to the nearest wall (so it keeps to the middle), then the joined line is smoothed and
resampled like drawn_track.py's. Widths: the base line's where it is kept, measured across the
detour corridor elsewhere.

Run it in the dev container with the workspace sourced:
    python3 scripts/tracks/detour_track.py base.yaml --name base_variant \\
        --leave X Y --rejoin X Y --block X0 Y0 X1 Y1 ...
"""

import argparse
import os
import pathlib

import cv2
import numpy as np
from rbsf_worlds.centerline import (
    corridor_from_drawing, distance_to, loop_length, pixel_frame, route_corridor,
    write_centerline)
from rbsf_worlds.tracks import find_waypoints
from scipy.ndimage import gaussian_filter1d
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
import yaml


def cheapest_path(region, clearance, src, dst):
    """Pixels (row, col) of the cheapest 8-connected path from src to dst inside `region`."""
    rows, cols = np.nonzero(region)
    index = -np.ones(region.shape, np.int64)
    index[rows, cols] = np.arange(len(rows))
    cost = 1.0 / np.maximum(clearance[rows, cols], 1.0) ** 2
    heads, tails, weights = [], [], []
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
        r2, c2 = rows + dr, cols + dc
        ok = (r2 < region.shape[0]) & (c2 >= 0) & (c2 < region.shape[1])
        ok[ok] = region[r2[ok], c2[ok]]
        a, b = index[rows[ok], cols[ok]], index[r2[ok], c2[ok]]
        heads.append(a)
        tails.append(b)
        weights.append(np.hypot(dr, dc) * 0.5 * (cost[a] + cost[b]))
    n = len(rows)
    graph = coo_matrix((np.concatenate(weights), (np.concatenate(heads), np.concatenate(tails))),
                       shape=(n, n)).tocsr()
    start, goal = index[src], index[dst]
    if start < 0 or goal < 0:
        raise ValueError('--leave and --rejoin must lie inside the detour corridor')
    _, predecessors = dijkstra(graph, directed=False, indices=start, return_predecessors=True)
    if predecessors[goal] < 0:
        raise ValueError('--rejoin is not reachable from --leave in the detour corridor')
    path = [goal]
    while path[-1] != start:
        path.append(predecessors[path[-1]])
    path = np.array(path[::-1])
    return rows[path], cols[path]


def widths_across(xy, region, scale, offset, max_m=5.0):
    """(right, left) free distance (m) from each point of a line to the edge of `region`."""
    tangent = np.gradient(xy, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    left = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)
    step = abs(scale[0]) / 2
    out = np.zeros((len(xy), 2))
    for k, side in enumerate((-1.0, 1.0)):
        reached = np.zeros(len(xy))
        active = np.ones(len(xy), bool)
        for d in np.arange(step, max_m, step):
            col, row = np.round((xy + side * d * left - offset) / scale).astype(int).T
            inside = ((row >= 0) & (row < region.shape[0]) & (col >= 0) & (col < region.shape[1]))
            free = np.zeros(len(xy), bool)
            free[inside] = region[row[inside], col[inside]]
            active &= free
            reached[active] = d
        out[:, k] = reached
    return out


def main():
    """Generate the variant track from the command line."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('base_yaml', type=pathlib.Path, help='Base track yaml.')
    parser.add_argument('--name', required=True, help='Variant track name.')
    parser.add_argument('--leave', type=float, nargs=2, required=True, metavar=('X', 'Y'),
                        help='Where the detour leaves the base line (map coordinates, m).')
    parser.add_argument('--rejoin', type=float, nargs=2, required=True, metavar=('X', 'Y'),
                        help='Where it rejoins the base line, further along it.')
    parser.add_argument('--block', type=float, nargs=4, action='append', default=[],
                        metavar=('X0', 'Y0', 'X1', 'Y1'),
                        help='Virtual wall segment of the detour corridor (repeatable).')
    parser.add_argument('--close-gaps', type=float, default=0.0, metavar='M',
                        help='Close wall gaps narrower than M metres.')
    parser.add_argument('--out-dir', type=pathlib.Path,
                        help='Default: <name>/ next to the base track directory.')
    parser.add_argument('--step', type=float, default=0.1, help='Centerline spacing (m).')
    parser.add_argument('--smooth', type=float, default=0.25,
                        help='Centerline smoothing sigma (m).')
    args = parser.parse_args()

    base_dir = args.base_yaml.parent
    spec = yaml.safe_load(args.base_yaml.read_text())
    resolution, origin = spec['resolution'], spec['origin']
    img = cv2.imread(str(base_dir / spec['image']), cv2.IMREAD_GRAYSCALE)
    scale, offset = pixel_frame(img.shape[0], resolution, origin)
    base = np.loadtxt(find_waypoints(args.base_yaml, 'centerline'), delimiter=',', comments='#')

    region = route_corridor(corridor_from_drawing(img, spec['occupied_thresh']), resolution,
                            origin, args.close_gaps, args.block, args.leave)
    leave = int(np.argmin(np.hypot(*(base[:, :2] - args.leave).T)))
    rejoin = int(np.argmin(np.hypot(*(base[:, :2] - args.rejoin).T)))
    if not 0 < leave < rejoin:
        raise ValueError('--leave must come before --rejoin along the base line, after its start')

    def to_cell(xy):
        col, row = np.round((np.asarray(xy) - offset) / scale).astype(int)
        return row, col

    rows, cols = cheapest_path(region, distance_to(~region), to_cell(base[leave, :2]),
                               to_cell(base[rejoin, :2]))
    detour = np.stack([cols, rows], axis=1) * scale + offset

    # Join, flag the detour points (their widths are measured), smooth and resample.
    xy = np.vstack([base[:leave, :2], detour, base[rejoin + 1:, :2]])
    kept = np.concatenate([np.ones(leave, bool), np.zeros(len(detour), bool),
                           np.ones(len(base) - rejoin - 1, bool)])
    widths = np.vstack([base[:leave, 2:], np.zeros((len(detour), 2)), base[rejoin + 1:, 2:]])
    closed = np.vstack([xy, xy[:1]])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
    n = int(round(s[-1] / args.step))
    t = np.arange(n) * s[-1] / n
    line = np.stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])], axis=1)
    line = gaussian_filter1d(line, args.smooth / args.step, axis=0, mode='wrap')
    kept = np.interp(t, s, np.append(kept, kept[0]).astype(float)) > 0.999
    widths = np.stack([np.interp(t, s, np.append(widths[:, k], widths[0, k])) for k in (0, 1)],
                      axis=1)
    widths[~kept] = widths_across(line, region, scale, offset)[~kept]
    line = np.column_stack([line, widths])

    out_dir = args.out_dir or base_dir.parent / args.name
    out_dir.mkdir(parents=True, exist_ok=True)
    length = loop_length(line)
    write_centerline(
        out_dir / f'{args.name}_centerline.csv', line,
        f'{args.name} centerline, {len(line)} points, {length:.2f} m; finish line = first point. '
        f'Generated by scripts/tracks/detour_track.py from {args.base_yaml.name}.')
    image = os.path.relpath(base_dir / spec['image'], out_dir)
    spec.update(image=image, centerline=f'{args.name}_centerline.csv')
    spec.pop('raceline', None)
    with open(out_dir / f'{args.name}.yaml', 'w') as f:
        yaml.safe_dump(spec, f, sort_keys=False, default_flow_style=None)
    print(f'{args.name}: {len(line)} points, {length:.2f} m (base {loop_length(base):.2f} m), '
          f'width {line[:, 2:].sum(axis=1).min():.2f}-{line[:, 2:].sum(axis=1).max():.2f} m')


if __name__ == '__main__':
    main()
