#!/usr/bin/env python3
"""
Turn a drawn track map (thin wall lines on white) into an rbsf_worlds track.

Input: a ROS map yaml + image where the walls are drawn as thin closed lines and everything else
(inside the infield, outside the track) is free. The drivable corridor is the largest free region
that does not touch the image border, and must be an annulus (one loop, one infield).

Output, next to the input yaml (or in --out-dir):
    <name>.png                 simulator map: corridor free, everything else occupied, edges
                               anti-aliased so f1tenth_gym extracts few, smooth wall segments
                               (its LiDAR cost grows with the segment count)
    <name>.yaml                map yaml pointing at it, with `centerline:` set
    <name>_centerline.csv      `x_m, y_m, w_tr_right_m, w_tr_left_m`, a closed loop ordered in the
                               driving direction, starting at the point closest to --start

Centerline: the zero set of (distance to the outer wall - distance to the inner wall), smoothed
and resampled at a fixed step. Widths are distances to the nearest wall along the corridor sides.

Needs numpy, scipy, opencv and PyYAML (all in the dev image):
    python3 scripts/tracks/drawn_track.py original.yaml --name iros2026 --start 32.5 8.3 --ccw
"""

import argparse
import pathlib

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d, map_coordinates
import yaml


def corridor_mask(img, occupied_thresh):
    """Return the largest free region not touching the border, and its infield and outside."""
    occupied = (255 - img.astype(np.float64)) / 255.0 > occupied_thresh
    n, labels, stats, _ = cv2.connectedComponentsWithStats(
        (~occupied).astype(np.uint8), connectivity=8)
    border = set(np.unique(np.concatenate(
        [labels[0], labels[-1], labels[:, 0], labels[:, -1]])))
    inner = [i for i in range(1, n) if i not in border]
    if not inner:
        raise ValueError('no free region enclosed by walls')
    corridor = labels == max(inner, key=lambda i: stats[i, cv2.CC_STAT_AREA])
    contours, _ = cv2.findContours(
        corridor.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 2:
        raise ValueError(f'the corridor is not a single loop ({len(contours)} boundaries)')
    outer = np.zeros(img.shape, np.uint8)
    cv2.drawContours(outer, contours, int(np.argmax([cv2.contourArea(c) for c in contours])),
                     1, -1)
    infield = outer.astype(bool) & ~corridor
    return corridor, infield, ~outer.astype(bool)


def sim_image(corridor, sigma_px):
    """Corridor white, rest black, with a blurred-and-sharpened edge ramp (same 0.5 level set)."""
    blurred = cv2.GaussianBlur(corridor.astype(np.float64), (0, 0), sigma_px)
    return np.round(np.clip((blurred - 0.5) * 2.0 + 0.5, 0.0, 1.0) * 255).astype(np.uint8)


def distance_to(region):
    """Per-pixel distance (px) to the nearest pixel of `region`."""
    return cv2.distanceTransform((~region).astype(np.uint8), cv2.DIST_L2, 5)


def centerline(corridor, infield, outside, resolution, origin, step, sigma_m, start, ccw):
    """Return the closed (N, 4) line x, y, w_right, w_left in map coordinates, driving order."""
    height = corridor.shape[0]
    d_in, d_out = distance_to(infield), distance_to(outside)
    near_inner = ((d_out > d_in) & corridor) | infield
    contours, _ = cv2.findContours(
        near_inner.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    px = max(contours, key=cv2.contourArea)[:, 0, :].astype(np.float64)  # (col, row)
    to_map = np.array([1.0, -1.0])
    offset = np.array([origin[0] + 0.5 * resolution, origin[1] + (height - 0.5) * resolution])
    xy = gaussian_filter1d(px * resolution * to_map + offset, sigma_m / resolution,
                           axis=0, mode='wrap')

    area = 0.5 * np.sum(xy[:, 0] * np.roll(xy[:, 1], -1) - np.roll(xy[:, 0], -1) * xy[:, 1])
    if (area > 0) != ccw:
        xy = xy[::-1]
    closed = np.vstack([xy, xy[:1]])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
    t = np.arange(int(round(s[-1] / step))) * s[-1] / int(round(s[-1] / step))
    xy = np.stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])], axis=1)
    xy = np.roll(xy, -int(np.argmin(np.hypot(*(xy - start).T))), axis=0)

    pixel = (xy - offset) / resolution * to_map  # (col, row)
    sample = [pixel[:, 1], pixel[:, 0]]
    w_in = map_coordinates(d_in, sample, order=1) * resolution
    w_out = map_coordinates(d_out, sample, order=1) * resolution
    # Driving counter-clockwise the infield is on the left.
    right, left = (w_out, w_in) if ccw else (w_in, w_out)
    return np.column_stack([xy, right, left])


def main():
    """Generate the track files from the command line."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('map_yaml', type=pathlib.Path, help='Drawn map yaml.')
    parser.add_argument('--name', required=True, help='Track name (output file prefix).')
    parser.add_argument('--start', type=float, nargs=2, required=True, metavar=('X', 'Y'),
                        help='Start/finish point in map coordinates (m).')
    direction = parser.add_mutually_exclusive_group(required=True)
    direction.add_argument('--ccw', action='store_true', help='Drive counter-clockwise.')
    direction.add_argument('--cw', action='store_true', help='Drive clockwise.')
    parser.add_argument('--out-dir', type=pathlib.Path, help='Default: next to the input.')
    parser.add_argument('--step', type=float, default=0.1, help='Centerline spacing (m).')
    parser.add_argument('--smooth', type=float, default=0.25,
                        help='Centerline smoothing sigma (m).')
    parser.add_argument('--edge-sigma', type=float, default=1.5,
                        help='Wall edge anti-aliasing sigma (px).')
    args = parser.parse_args()

    spec = yaml.safe_load(args.map_yaml.read_text())
    if spec.get('negate', 0):
        raise ValueError('negate: 1 maps are not supported')
    img = cv2.imread(str(args.map_yaml.parent / spec['image']), cv2.IMREAD_GRAYSCALE)
    out_dir = args.out_dir or args.map_yaml.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    corridor, infield, outside = corridor_mask(img, spec['occupied_thresh'])
    cv2.imwrite(str(out_dir / f'{args.name}.png'), sim_image(corridor, args.edge_sigma))

    line = centerline(corridor, infield, outside, spec['resolution'], spec['origin'],
                      args.step, args.smooth, np.array(args.start), args.ccw)
    length = np.linalg.norm(np.diff(line[:, :2], axis=0, append=line[:1, :2]), axis=1).sum()
    widths = line[:, 2] + line[:, 3]
    with open(out_dir / f'{args.name}_centerline.csv', 'w') as f:
        f.write('# x_m, y_m, w_tr_right_m, w_tr_left_m\n')
        f.write(f'# {args.name} centerline, {len(line)} points, {length:.2f} m, '
                f'{"counter-clockwise" if args.ccw else "clockwise"}; finish line = first '
                f'point. Generated by scripts/tracks/drawn_track.py from '
                f'{args.map_yaml.name}.\n')
        np.savetxt(f, line, fmt='%.4f', delimiter=', ')

    spec.update(image=f'{args.name}.png', centerline=f'{args.name}_centerline.csv')
    with open(out_dir / f'{args.name}.yaml', 'w') as f:
        yaml.safe_dump(spec, f, sort_keys=False, default_flow_style=None)
    print(f'{args.name}: {len(line)} points, {length:.2f} m, width '
          f'{widths.min():.2f}-{widths.max():.2f} m')


if __name__ == '__main__':
    main()
