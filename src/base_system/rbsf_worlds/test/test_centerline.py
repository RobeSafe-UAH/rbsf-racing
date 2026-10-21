import math

import cv2
import numpy as np
import pytest
from rbsf_worlds.centerline import (
    centerline, corridor_from_slam_map, driving_direction, loop_length, route_corridor,
    split_ring, TrackError)

RES = 0.05
FREE, UNKNOWN, OCC = 254, 205, 0


def ring_map(r_out=6.0, r_in=4.0, size=320):
    """SLAM-style trinary map: a circular corridor (free) with walls, unknown elsewhere."""
    img = np.full((size, size), UNKNOWN, np.uint8)
    c = (size // 2, size // 2)
    cv2.circle(img, c, int(r_out / RES) + 2, OCC, -1)
    cv2.circle(img, c, int(r_out / RES), FREE, -1)
    cv2.circle(img, c, int(r_in / RES), OCC, -1)
    cv2.circle(img, c, int(r_in / RES) - 2, UNKNOWN, -1)
    origin = (-size * RES / 2, -size * RES / 2)  # map origin at the ring centre
    return img, origin


def extract(img, origin, ccw=True, start=(5.0, 0.0)):
    corridor = corridor_from_slam_map(img, RES)
    infield, outside = split_ring(corridor)
    return centerline(corridor, infield, outside, RES, origin, 0.1, 0.25, np.array(start), ccw)


def test_clean_ring_gives_the_mid_circle():
    img, origin = ring_map()
    line = extract(img, origin)
    radius = np.hypot(line[:, 0], line[:, 1])
    assert radius.mean() == pytest.approx(5.0, abs=0.05)
    assert loop_length(line) == pytest.approx(2 * math.pi * 5.0, rel=0.02)
    assert line[:, 2:].mean() == pytest.approx(1.0, abs=0.08)  # 1 m to each wall
    assert np.hypot(*(line[0, :2] - (5.0, 0.0))) < 0.15  # starts at the finish line


def test_direction_orders_the_line_and_swaps_sides():
    img, origin = ring_map(r_out=6.5)  # outer wall 1.5 m away, inner 1.0 m... from 5.25
    ccw = extract(img, origin, ccw=True)
    cw = extract(img, origin, ccw=False)
    heading_ccw = np.cross(ccw[0, :2], ccw[1, :2] - ccw[0, :2])
    heading_cw = np.cross(cw[0, :2], cw[1, :2] - cw[0, :2])
    assert heading_ccw > 0 > heading_cw
    # Counter-clockwise the infield is on the left: left width is the inner-wall distance.
    assert ccw[:, 3].mean() == pytest.approx(cw[:, 2].mean(), abs=0.05)


def test_slam_noise_is_cleaned():
    img, origin = ring_map()
    rng = np.random.default_rng(0)
    corridor = img == FREE
    specks = corridor & (rng.random(img.shape) < 0.01)
    img[specks] = UNKNOWN  # sensor shadows inside the corridor
    img[150:152, 30:60] = FREE  # a thin ray leaking out through the outer wall
    img[10:13, 10:13] = FREE  # stray free cells outside
    line = extract(img, origin)
    assert np.hypot(line[:, 0], line[:, 1]).mean() == pytest.approx(5.0, abs=0.05)


def test_unclosed_loop_is_rejected():
    img, origin = ring_map()
    band = img[:160, 156:164]  # the lap was never completed: no free cells across the top
    band[band == FREE] = UNKNOWN
    corridor = corridor_from_slam_map(img, RES)
    with pytest.raises(TrackError, match='not a single loop'):
        split_ring(corridor)


def test_doubled_wall_is_rejected():
    img, origin = ring_map()
    # A ghost wall along the corridor (bad loop closure) splits it into two lanes over a stretch.
    cv2.ellipse(img, (160, 160), (100, 100), 0, 0, 120, OCC, 2)
    corridor = corridor_from_slam_map(img, RES)
    with pytest.raises(TrackError):
        split_ring(corridor)


def test_driving_direction_from_a_partial_path():
    t = np.linspace(0, 1.3 * 2 * math.pi, 200)
    path = np.c_[5 * np.cos(t), 5 * np.sin(t)]
    assert driving_direction(path) is True
    assert driving_direction(path[::-1]) is False


def test_route_corridor_closes_open_borders_and_blocks_branches():
    img, origin = ring_map()
    # An open border (dashed wall, 0.17 m gaps) down the middle: two lanes, r 4-5 m and 5-6 m.
    for start_deg in range(0, 360, 6):
        cv2.ellipse(img, (160, 160), (100, 100), 0, start_deg, start_deg + 4, OCC, 2)
    corridor = img == FREE
    with pytest.raises(TrackError):
        split_ring(corridor)  # the dashes are islands in one wide corridor
    outer = route_corridor(corridor, RES, origin, 0.3, [], (5.5, 0.0))
    split_ring(outer)  # gaps closed: the outer lane alone is a loop
    assert outer.sum() < 0.6 * corridor.sum()
    with pytest.raises(TrackError, match='1 boundaries'):
        split_ring(route_corridor(corridor, RES, origin, 0.3, [(0.0, 4.9, 0.0, 6.2)],
                                  (5.5, 0.0)))  # blocked across: no longer a loop
    with pytest.raises(TrackError, match='not inside'):
        route_corridor(corridor, RES, origin, 0.3, [], (0.0, 0.0))  # infield
