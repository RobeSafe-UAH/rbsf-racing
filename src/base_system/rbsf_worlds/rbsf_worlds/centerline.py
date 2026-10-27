"""
Centerline extraction from a track map: the corridor, its walls and the reference line.

Shared by scripts/tracks/drawn_track.py (organisers' drawings) and scripts/maps/session_track.py
(SLAM maps saved by rbsf_dual_slam). No ROS imports.

A track map gives a *corridor*: one free loop around one *infield*, everything else *outside*.
The centerline is the zero set of (distance to the outer wall - distance to the inner wall),
smoothed and resampled at a fixed step. It is a closed loop ordered in the driving direction,
starting at the point closest to the start/finish, with the free width on each side.

Drawn maps already have a clean corridor (corridor_from_drawing). SLAM maps need cleaning first
(corridor_from_slam_map): unknown specks inside the corridor, rays leaking through gaps in the
walls, and stray free cells outside.
"""

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d, map_coordinates

FREE_LEVEL = 250  # trinary SLAM maps (nav2 map_saver): free 254, unknown 205, occupied 0


class TrackError(ValueError):
    """The map does not contain a single closed corridor (no usable track)."""


def split_ring(corridor):
    """
    Return (infield, outside) of a corridor mask that must be a single loop.

    Raises TrackError naming the number of boundaries otherwise (an unclosed loop has 1, a
    doubled wall or an unfilled hole gives more than 2).
    """
    contours, _ = cv2.findContours(
        corridor.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 2:
        raise TrackError(f'the corridor is not a single loop ({len(contours)} boundaries, '
                         'expected 2: outer and inner wall)')
    outer = np.zeros(corridor.shape, np.uint8)
    cv2.drawContours(outer, contours, int(np.argmax([cv2.contourArea(c) for c in contours])),
                     1, -1)
    infield = outer.astype(bool) & ~corridor
    return infield, ~outer.astype(bool)


def corridor_from_drawing(img, occupied_thresh, thin_walls=False):
    """
    Corridor of a drawn map: the largest free region not touching the image border.

    thin_walls: the walls are 1 px lines drawn 8-connected, which free pixels leak through
    diagonally, so free space is 4-connected.
    """
    occupied = (255 - img.astype(np.float64)) / 255.0 > occupied_thresh
    n, labels, stats, _ = cv2.connectedComponentsWithStats(
        (~occupied).astype(np.uint8), connectivity=4 if thin_walls else 8)
    border = set(np.unique(np.concatenate(
        [labels[0], labels[-1], labels[:, 0], labels[:, -1]])))
    inner = [i for i in range(1, n) if i not in border]
    if not inner:
        raise TrackError('no free region enclosed by walls')
    return labels == max(inner, key=lambda i: stats[i, cv2.CC_STAT_AREA])


def route_corridor(corridor, resolution, origin, close_gaps_m, blocks, start):
    """
    Corridor a reference line runs in: a sub-corridor of `corridor` (drawn maps), in map units.

    close_gaps_m: wall gaps narrower than this are closed (open track borders: the LiDAR sees
    through them, no car fits). blocks: (x0, y0, x1, y1) virtual wall segments, cutting off
    branches the line does not take. Returns the part (4-connected) holding `start`.
    """
    walls = (~corridor).astype(np.uint8)
    if close_gaps_m > 0:
        size = int(round(close_gaps_m / resolution)) | 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        walls = cv2.morphologyEx(walls, cv2.MORPH_CLOSE, kernel)
    scale, offset = pixel_frame(corridor.shape[0], resolution, origin)

    def to_px(x, y):
        return tuple(int(round(v)) for v in (np.array([x, y]) - offset) / scale)

    for x0, y0, x1, y1 in blocks:
        cv2.line(walls, to_px(x0, y0), to_px(x1, y1), 1, thickness=3)
    _, labels = cv2.connectedComponents(1 - walls, connectivity=4)
    col, row = to_px(*start)
    if not (0 <= row < labels.shape[0] and 0 <= col < labels.shape[1]) or not labels[row, col]:
        raise TrackError(f'--start {start} is not inside the corridor')
    return labels == labels[row, col]


def fill_small_holes(mask, max_area_px):
    """Set to True every hole (False region enclosed by `mask`) smaller than `max_area_px`."""
    n, labels, stats, _ = cv2.connectedComponentsWithStats(
        (~mask).astype(np.uint8), connectivity=4)
    border = set(np.unique(np.concatenate(
        [labels[0], labels[-1], labels[:, 0], labels[:, -1]])))
    out = mask.copy()
    for i in range(1, n):
        if i not in border and stats[i, cv2.CC_STAT_AREA] < max_area_px:
            out[labels == i] = True
    return out


def corridor_from_slam_map(img, resolution, open_m=0.15, hole_m2=0.25):
    """
    Return the cleaned corridor of a SLAM occupancy map (trinary image, free = 254).

    1. Free cells, with unknown holes smaller than `hole_m2` filled (sensor shadows, specks).
       Kept small: a ghost wall from a bad loop closure (two lanes) must survive as a hole.
    2. Morphological opening with a disc of diameter `open_m`: removes thin free leaks through
       gaps in the walls and isolated free cells. Narrower than any drivable corridor.
    3. The largest connected free region.
    """
    free = img >= FREE_LEVEL
    free = fill_small_holes(free, hole_m2 / resolution ** 2)
    size = max(3, int(round(open_m / resolution)) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    free = cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_OPEN, kernel).astype(bool)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(free.astype(np.uint8), connectivity=8)
    if n < 2:
        raise TrackError('no free space in the map')
    return labels == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))


def corridor_image(corridor, sigma_px=1.5):
    """
    Map image of a corridor: white inside, black elsewhere, with an anti-aliased edge.

    Blurred and sharpened so the 0.5 level set stays on the wall: f1tenth_gym then extracts few,
    smooth wall segments (its LiDAR cost grows with the segment count).
    """
    blurred = cv2.GaussianBlur(corridor.astype(np.float64), (0, 0), sigma_px)
    return np.round(np.clip((blurred - 0.5) * 2.0 + 0.5, 0.0, 1.0) * 255).astype(np.uint8)


def distance_to(region):
    """Per-pixel distance (px) to the nearest pixel of `region`."""
    return cv2.distanceTransform((~region).astype(np.uint8), cv2.DIST_L2, 5)


def pixel_frame(height, resolution, origin):
    """(scale, offset) mapping pixel (col, row) to map (x, y): xy = px * scale + offset."""
    scale = np.array([resolution, -resolution])
    offset = np.array([origin[0] + 0.5 * resolution, origin[1] + (height - 0.5) * resolution])
    return scale, offset


def signed_area(xy):
    """Shoelace area of a closed polygon: positive when counter-clockwise."""
    return 0.5 * float(np.sum(xy[:, 0] * np.roll(xy[:, 1], -1) - np.roll(xy[:, 0], -1) * xy[:, 1]))


def driving_direction(path):
    """
    Return True (counter-clockwise) or False from a path driven around the track (N x 2).

    The path need not be closed or a single lap: the sign of the swept angle around the path's
    centroid decides it.
    """
    rel = np.asarray(path, float) - np.mean(path, axis=0)
    angles = np.unwrap(np.arctan2(rel[:, 1], rel[:, 0]))
    return bool(angles[-1] - angles[0] > 0)


def centerline(corridor, infield, outside, resolution, origin, step, sigma_m, start, ccw):
    """Return the closed (N, 4) line x, y, w_right, w_left in map coordinates, driving order."""
    height = corridor.shape[0]
    d_in, d_out = distance_to(infield), distance_to(outside)
    near_inner = ((d_out > d_in) & corridor) | infield
    contours, _ = cv2.findContours(
        near_inner.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    px = max(contours, key=cv2.contourArea)[:, 0, :].astype(np.float64)  # (col, row)
    scale, offset = pixel_frame(height, resolution, origin)
    xy = gaussian_filter1d(px * scale + offset, sigma_m / resolution, axis=0, mode='wrap')

    if (signed_area(xy) > 0) != ccw:
        xy = xy[::-1]
    closed = np.vstack([xy, xy[:1]])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
    t = np.arange(int(round(s[-1] / step))) * s[-1] / int(round(s[-1] / step))
    xy = np.stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])], axis=1)
    xy = np.roll(xy, -int(np.argmin(np.hypot(*(xy - start).T))), axis=0)

    pixel = (xy - offset) / scale  # (col, row)
    sample = [pixel[:, 1], pixel[:, 0]]
    w_in = map_coordinates(d_in, sample, order=1) * resolution
    w_out = map_coordinates(d_out, sample, order=1) * resolution
    # Driving counter-clockwise the infield is on the left.
    right, left = (w_out, w_in) if ccw else (w_in, w_out)
    return np.column_stack([xy, right, left])


def loop_length(line):
    """Length (m) of a closed line (first point not repeated)."""
    return float(np.linalg.norm(np.diff(line[:, :2], axis=0, append=line[:1, :2]), axis=1).sum())


def write_centerline(path, line, comment):
    """Write the rbsf_worlds centerline CSV (x_m, y_m, w_tr_right_m, w_tr_left_m)."""
    with open(path, 'w') as f:
        f.write('# x_m, y_m, w_tr_right_m, w_tr_left_m\n')
        f.write(f'# {comment}\n')
        np.savetxt(f, line, fmt='%.4f', delimiter=', ')
