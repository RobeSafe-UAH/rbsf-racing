import math

import cv2
import numpy as np
import pytest
from rbsf_worlds.centerline import TrackError
from rbsf_worlds.cones import draw_cones
from rbsf_worlds.paths import load_waypoints
from rbsf_worlds.raceline import (
    _jacobian, _residuals, box_qp, check_fit, corridor_widths, Footprint, menger, normals,
    optimise, raceline_rows, resample, TrackMap, write_raceline)

RES = 0.05
CAR = Footprint(rear=0.05, front=0.43, width=0.31)
MARGIN, WHEELBASE, MAX_STEERING = 0.15, 0.33, 0.4189  # steering limit R = 0.74 m


def blank(size_m):
    size = int(size_m / RES)
    return np.zeros((size, size), np.uint8), (-size_m / 2, -size_m / 2)


def to_pixels(xy, image, origin):
    """Map points to integer pixel (col, row) for cv2 drawing."""
    col = (xy[:, 0] - origin[0]) / RES - 0.5
    row = image.shape[0] - 0.5 - (xy[:, 1] - origin[1]) / RES
    return np.round(np.column_stack([col, row])).astype(np.int32)


def ring(radius=5.0, width=2.0, n=300):
    """Circular corridor (free) of `width` around a centerline circle, driven counter-clockwise."""
    image, origin = blank(2 * (radius + width) + 2)
    angles = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    line = radius * np.column_stack([np.cos(angles), np.sin(angles)])
    cv2.polylines(image, [to_pixels(line, image, origin)], True, 255, int(width / RES))
    return image, origin, line


def stadium(radius=2.0, straight=6.0, width=2.0, step=0.05):
    """Two straights joined by half circles, corridor `width` wide, counter-clockwise."""
    s = np.arange(0.0, straight, step)
    arc = np.arange(0.0, math.pi, step / radius)
    line = np.vstack([
        np.column_stack([s - straight / 2, np.full_like(s, -radius)]),
        np.column_stack([straight / 2 + radius * np.sin(arc), -radius * np.cos(arc)]),
        np.column_stack([straight / 2 - s, np.full_like(s, radius)]),
        np.column_stack([-straight / 2 - radius * np.sin(arc), radius * np.cos(arc)])])
    image, origin = blank(straight + 2 * (radius + width) + 2)
    cv2.polylines(image, [to_pixels(line, image, origin)], True, 255, int(width / RES))
    return image, origin, line


def track_map(image, origin):
    return TrackMap.from_image(image, RES, origin, free_thresh=0.196)


def fit(m, xy):
    return check_fit(m, xy, CAR, MARGIN, WHEELBASE, MAX_STEERING)


def run(m, line):
    return optimise(m, line, CAR, MARGIN, WHEELBASE, MAX_STEERING, log=lambda _: None)


def test_corridor_widths_on_a_ring():
    image, origin, line = ring(width=2.0)
    xy = resample(line, 0.1)
    w_right, w_left = corridor_widths(track_map(image, origin), xy, normals(xy))
    # The polyline is drawn with a whole-pixel thickness: allow a pixel either way.
    assert w_right == pytest.approx(1.0, abs=RES)
    assert w_left == pytest.approx(1.0, abs=RES)


def test_a_ring_stays_a_circle():
    image, origin, line = ring()
    xy, report = run(track_map(image, origin), line)
    radius = np.linalg.norm(xy, axis=1)
    assert report.ok
    assert np.ptp(radius) < 0.05  # still a circle, wherever it settles across the corridor


def squared_curvature(xy):
    k, ds = menger(xy)
    return float(np.sum(k ** 2 * ds))


def test_the_raceline_cuts_the_corners_of_a_stadium():
    image, origin, line = stadium()
    m = track_map(image, origin)
    xy, report = run(m, line)
    centerline = fit(m, line)
    assert report.ok
    assert report.length < centerline.length - 0.5
    # The objective, not the peak: on short straights the line bends all the way round, so its
    # apex can be slightly sharper than the 2 m half circle while the lap's sum of k^2 drops.
    assert squared_curvature(xy) < 0.8 * squared_curvature(line)
    # At the apex of each half circle the car is close to the inner wall, at about the margin.
    assert report.min_clearance == pytest.approx(MARGIN, abs=0.03)


def test_a_length_weight_shortens_the_line():
    image, origin, line = stadium()
    m = track_map(image, origin)
    _, curvy = run(m, line)
    _, short = optimise(m, line, CAR, MARGIN, WHEELBASE, MAX_STEERING, length_weight=1.0,
                        log=lambda _: None)
    assert short.ok
    assert short.length < curvy.length - 0.3


def test_a_corridor_narrower_than_the_car_raises():
    image, origin, line = ring(width=0.5)  # 0.31 m car + 2 x 0.15 m margin needs 0.61 m
    with pytest.raises(TrackError, match='does not fit'):
        run(track_map(image, origin), line)


def test_a_cone_beside_the_centerline_fails_the_check_and_the_raceline_avoids_it():
    image, origin, line = ring(width=2.0)
    # 0.3 m outside the centerline: the 0.31 m wide car clears it by 0.05 m, under the margin.
    image = draw_cones(image, [(5.3, 0.0)], RES, origin)
    m = track_map(image, origin)
    centerline = fit(m, line)
    assert not centerline.fits
    x, y = centerline.worst_xy
    assert math.hypot(x - 5.3, y) < 0.6  # the footprint comes too close around there
    xy, report = run(m, line)
    assert report.fits
    assert np.linalg.norm(xy - (5.3, 0.0), axis=1).min() > 0.1 + 0.155 + MARGIN - 0.02


def test_a_corner_tighter_than_the_steering_limit_fails():
    image, origin, line = ring(radius=0.6, width=1.0, n=120)
    report = fit(track_map(image, origin), line)
    assert not report.steerable
    assert report.max_curvature == pytest.approx(1 / 0.6, rel=0.02)


@pytest.mark.parametrize('size', [30, 31, 32])  # every loop length modulo the 3 colours
def test_coloured_jacobian_matches_dense_finite_differences(size):
    angles = np.linspace(0.0, 2.0 * math.pi, size, endpoint=False)
    c = np.column_stack([3.0 * np.cos(angles), 2.0 * np.sin(angles)])
    n = normals(c)
    alpha = np.random.default_rng(0).normal(0.0, 0.1, size)
    jac = _jacobian(c, n, alpha, 0.5, 100.0).toarray()
    dense = np.zeros_like(jac)
    for j in range(size):
        e = np.zeros(size)
        e[j] = 1e-6
        dense[:, j] = (_residuals(c, n, alpha + e, 0.5, 100.0)
                       - _residuals(c, n, alpha - e, 0.5, 100.0)) / 2e-6
    np.testing.assert_allclose(jac, dense, atol=1e-6)


def test_box_qp_solves_a_bounded_problem():
    h = np.eye(3)
    g = np.array([2.0, -2.0, 0.5])  # unconstrained minimum at g
    x = box_qp(h, g, np.full(3, -1.0), np.full(3, 1.0), np.zeros(3))
    np.testing.assert_allclose(x, [1.0, -1.0, 0.5], atol=1e-9)


def test_raceline_file_round_trip(tmp_path):
    _, _, line = ring()
    xy = resample(line, 0.1)
    speeds = np.linspace(1.0, 3.0, len(xy))
    rows = raceline_rows(xy, speeds)
    path = tmp_path / 'ring_raceline.csv'
    write_raceline(path, rows, 'test')
    loaded_xy, loaded_speeds = load_waypoints(path)
    np.testing.assert_allclose(loaded_xy, xy, atol=1e-6)
    np.testing.assert_allclose(loaded_speeds, speeds, atol=1e-6)
    # Counter-clockwise circle of radius 5: curvature +0.2, yaw along the tangent.
    assert rows[:, 4] == pytest.approx(0.2, rel=0.01)
    assert rows[0, 3] == pytest.approx(math.pi / 2, abs=0.05)
