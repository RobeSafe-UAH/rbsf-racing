"""
Offline minimum-curvature raceline and the check that the car fits along it (no ROS imports).

Used by scripts/tracks/raceline.py, on the built-in tracks and on car-mapped ones alike.

Geometry comes from the track map, not from the widths stored in a centerline CSV, so the
optimiser and the check see the same walls (and the cones the yaml declares):

- TrackMap: the drivable cells of the map image (occupancy below free_thresh; unknown is not
  drivable) and their clearance, the distance (m) to the nearest blocked cell.
- Corridor: the centerline, resampled uniformly, is the fixed base c with left normals n. The line
  is p = c + alpha n. Along each normal the free width to either wall is measured on the map
  (sub-pixel, then a running minimum so wall pixels do not make the bounds jagged), and alpha
  keeps a lateral clearance from both walls. On the inside of a corner alpha also stays below
  0.9 times the base's radius, so neighbouring normals never cross and the line cannot fold.
- min_curvature: minimise F = sum (k^2 + mu max(|k| - k_max, 0)^2) ds over the bounds, with k the
  exact curvature of the circle through each point and its neighbours (independent of their
  spacing) and k_max the steering limit. F is a sum of squared residuals, each depending on three
  neighbouring offsets, so its Jacobian is cyclic tridiagonal and finite differences give it
  exactly with a handful of evaluations (offsets three apart perturbed together). Each
  Levenberg-Marquardt step is then a box-constrained sparse quadratic program (projected Newton,
  box_qp) whose model matches F to first order, so it converges to the constrained minimum: the
  line runs out to the edges where that lowers the curvature. This is the minimum-curvature
  problem of Heilmeier et al. (TUM, 2019), solved on the exact curvature instead of re-basing.
  Minimum curvature is only a proxy for lap time: it also straightens bends taken flat out, which
  makes the line longer. A length term w sum l^2 / h (about w times the lap length) pulls it back
  towards the shortest path; scripts/tracks/raceline.py sweeps w and keeps the line with the
  shortest lap time under the speed profile.
- check_fit: sweep the car's footprint (a rectangle about the rear axle, base_link) along the
  line every 2 cm, sampled as a grid of body points so obstacles smaller than the car count too.
  Every pose must keep `margin` of clearance, and the curvature must stay within the steering
  limit tan(max_steering) / wheelbase.
- optimise: min_curvature with the car's half width + margin as lateral clearance, then
  check_fit. Where the swept footprint comes too close (the front overhang swings wide in tight
  corners), the bound on that side of that stretch is tightened by the deficit and the line is
  optimised again, up to `fit_rounds` times.
"""

from dataclasses import dataclass, field

import cv2
import numpy as np
from rbsf_worlds.centerline import distance_to, pixel_frame, TrackError
from rbsf_worlds.cones import draw_cones, load_cones
from rbsf_worlds.paths import arc_lengths, headings
from rbsf_worlds.speed_profile import curvature
from scipy import sparse
from scipy.ndimage import map_coordinates, maximum_filter1d, minimum_filter1d
from scipy.sparse.linalg import spsolve
from scipy.spatial import cKDTree
import yaml

POSE_STEP = 0.02  # m between the poses check_fit sweeps the footprint through
BODY_STEP = 0.025  # m between the footprint's body points
RAY_STEP = 0.01  # m, corridor width resolution along the normals
WIDTH_WINDOW = 0.2  # m each side: running minimum of the widths along the line
FOLD = 0.9  # inner offsets stay below this fraction of the base's radius
STEERING_PENALTY = 100.0  # mu: weight of the curvature above the steering limit
SMOOTHING = 1e-6  # weight of alpha^2: keeps the solution unique where the line is straight


@dataclass
class Footprint:
    """Car outline about the rear axle (base_link): metres behind it, ahead of it, and width."""

    rear: float
    front: float
    width: float

    def body_points(self, step=BODY_STEP):
        """(M, 2) grid of points covering the rectangle, edges included (car frame)."""
        xs = np.linspace(-self.rear, self.front, int(np.ceil((self.front + self.rear) / step)) + 1)
        ys = np.linspace(-self.width / 2, self.width / 2, int(np.ceil(self.width / step)) + 1)
        return np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2)

    def corners(self):
        """Return the four corners of the rectangle (car frame), in order."""
        w = self.width / 2
        return np.array([[-self.rear, -w], [self.front, -w], [self.front, w], [-self.rear, w]])


@dataclass
class TrackMap:
    """Drivable cells of a track map and their clearance (m) to the nearest blocked cell."""

    drivable: np.ndarray
    clearance: np.ndarray
    resolution: float
    origin: tuple

    @classmethod
    def load(cls, map_yaml):
        """Read a ROS map yaml + image (negate: 0), with the cones its yaml declares drawn in."""
        with open(map_yaml) as f:
            spec = yaml.safe_load(f)
        if spec.get('negate', 0):
            raise ValueError(f'{map_yaml}: negate: 1 maps are not supported')
        image = cv2.imread(str(map_yaml.parent / spec['image']), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f"{map_yaml}: cannot read image {spec['image']}")
        image = draw_cones(image, load_cones(map_yaml), spec['resolution'], spec['origin'])
        return cls.from_image(image, spec['resolution'], spec['origin'], spec['free_thresh'])

    @classmethod
    def from_image(cls, image, resolution, origin, free_thresh):
        """Map from a uint8 image (row 0 at the top): occupancy (255 - v) / 255 < free_thresh."""
        drivable = (255.0 - image) / 255.0 < free_thresh
        # Distances run between cell centres; the wall surface is half a cell closer.
        clearance = np.maximum(distance_to(~drivable) - 0.5, 0.0) * resolution
        return cls(drivable, clearance, float(resolution), tuple(origin[:2]))

    def extent(self):
        """(left, right, bottom, top) of the image in map coordinates, for imshow."""
        height, width = self.drivable.shape
        return (self.origin[0], self.origin[0] + width * self.resolution,
                self.origin[1], self.origin[1] + height * self.resolution)

    def _sample(self, grid, xy, order):
        scale, offset = pixel_frame(grid.shape[0], self.resolution, self.origin)
        pixel = (np.asarray(xy, float) - offset) / scale  # (col, row)
        return map_coordinates(grid, [pixel[..., 1], pixel[..., 0]], order=order,
                               mode='constant', cval=0)

    def clearance_at(self, xy):
        """Clearance (m) at points (..., 2); 0 outside the map."""
        return self._sample(self.clearance, xy, order=1)

    def drivable_at(self, xy):
        """Whether points (..., 2) are drivable, with walls placed between cells (bilinear)."""
        return self._sample(self.drivable.astype(np.float64), xy, order=1) >= 0.5


@dataclass
class FitReport:
    """check_fit's result. Arc lengths s (m) are along the checked line from its first point."""

    length: float
    min_clearance: float
    worst_s: float
    worst_xy: np.ndarray
    violations: list  # (s_start, s_end) stretches with clearance below the margin
    max_curvature: float
    max_curvature_s: float
    max_curvature_xy: np.ndarray
    curvature_limit: float
    margin: float
    poses: np.ndarray = field(repr=False)  # (P, 3) x, y, yaw swept every POSE_STEP
    pose_s: np.ndarray = field(repr=False)
    pose_clearance: np.ndarray = field(repr=False)
    pose_side: np.ndarray = field(repr=False)  # side of the car nearest a wall: -1 right, 1 left

    @property
    def fits(self):
        """Whether the footprint keeps the margin everywhere."""
        return not self.violations

    @property
    def steerable(self):
        """Whether the curvature stays within the steering limit."""
        return self.max_curvature <= self.curvature_limit

    @property
    def ok(self):
        """Whether both checks pass."""
        return self.fits and self.steerable

    def summary(self):
        """One line per check, for the command line."""
        x, y = self.worst_xy
        kx, ky = self.max_curvature_xy
        return [
            f'length {self.length:.2f} m',
            f'footprint clearance min {self.min_clearance:.3f} m at s = {self.worst_s:.2f} m '
            f'({x:.2f}, {y:.2f}), margin {self.margin:.2f} m: '
            + ('OK' if self.fits else f'FAILS on {len(self.violations)} stretches, s = '
               + ', '.join(f'{s0:.2f}-{s1:.2f} m' for s0, s1 in self.violations[:5])
               + (', ...' if len(self.violations) > 5 else '')),
            f'curvature max {self.max_curvature:.3f} 1/m '
            f'(R {1.0 / max(self.max_curvature, 1e-9):.2f} m) at s = '
            f'{self.max_curvature_s:.2f} m ({kx:.2f}, {ky:.2f}), steering limit '
            f'{self.curvature_limit:.3f} 1/m (R {1.0 / self.curvature_limit:.2f} m): '
            + ('OK' if self.steerable else 'FAILS')]


def resample(xy, step, start=None):
    """Resample a closed line at a uniform spacing near `step`, from the point nearest `start`."""
    closed = np.vstack([xy, xy[:1]])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
    n = max(3, int(round(s[-1] / step)))
    t = np.arange(n) * s[-1] / n
    out = np.column_stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])])
    if start is not None:
        out = np.roll(out, -int(np.argmin(np.linalg.norm(out - start, axis=1))), axis=0)
    return out


def normals(xy):
    """Return the left unit normals of a closed line, from central-difference tangents."""
    tangent = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    return np.column_stack([-tangent[:, 1], tangent[:, 0]])


def corridor_widths(track_map, xy, n, max_width=10.0):
    """
    Free distance (m) from each point to the walls along its normal: (w_right, w_left).

    Walks the normal in RAY_STEP steps to the first blocked point, then takes the running minimum
    over WIDTH_WINDOW each side (conservative, and smooth enough to optimise against).
    """
    d = np.arange(RAY_STEP, max_width, RAY_STEP)
    spacing = np.mean(np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1))
    window = 2 * int(round(WIDTH_WINDOW / spacing)) + 1
    widths = []
    for sign in (-1.0, 1.0):  # right, left
        rays = xy[:, None, :] + sign * d[None, :, None] * n[:, None, :]
        blocked = ~track_map.drivable_at(rays)
        first = np.where(blocked.any(axis=1), blocked.argmax(axis=1), len(d) - 1)
        widths.append(minimum_filter1d(d[first] - 0.5 * RAY_STEP, size=window, mode='wrap'))
    return widths[0], widths[1]


def offset_bounds(c, w_right, w_left, clear_right, clear_left):
    """
    Return the bounds (lower, upper) on alpha.

    They keep `clear_*` (scalars or per point) from each wall, and the inner side below FOLD
    times the base's radius. Raises TrackError where the car does not fit.
    """
    lower = -(w_right - clear_right)
    upper = w_left - clear_left
    k = curvature(c, 0.25, signed=True)
    with np.errstate(divide='ignore'):
        upper = np.where(k > 0, np.minimum(upper, FOLD / k), upper)  # left turn: infield left
        lower = np.where(k < 0, np.maximum(lower, FOLD / k), lower)
    tight = np.nonzero(lower >= upper)[0]
    if len(tight):
        s, _ = arc_lengths(c)
        i = tight[0]
        need = np.broadcast_to(np.asarray(clear_right) + clear_left, lower.shape)[i]
        raise TrackError(
            f'the car does not fit: {len(tight)} points where the corridor is narrower than the '
            f'{need:.2f} m it needs, first at s = {s[i]:.2f} m ({c[i, 0]:.2f}, {c[i, 1]:.2f}): '
            f'{w_right[i]:.2f} m right + {w_left[i]:.2f} m left')
    return lower, upper


def menger(p):
    """
    Return the curvature through each point and its neighbours, and the length it stands for.

    The curvature (1/m, left positive) is that of the circle through the three points; the length
    ds is half of each neighbouring segment.
    """
    before, after = np.roll(p, 1, axis=0) - p, np.roll(p, -1, axis=0) - p
    la, lb = np.linalg.norm(before, axis=1), np.linalg.norm(after, axis=1)
    lc = np.linalg.norm(np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0), axis=1)
    cross = after[:, 0] * before[:, 1] - after[:, 1] * before[:, 0]
    return 2.0 * cross / (la * lb * lc), 0.5 * (la + lb)


def _residuals(c, n, alpha, limit, mu, length_weight=0.0):
    """
    Return the residuals whose squares sum to the objective (SMOOTHING aside).

    sqrt(ds) k for every point, then sqrt(mu ds) times its curvature above the steering limit,
    then (with a length weight w) sqrt(w / h) times each segment's x and y extent: their squares
    sum to w sum l^2 / h, about w times the lap length at the base spacing h.
    """
    p = c + alpha[:, None] * n
    k, ds = menger(p)
    root = np.sqrt(ds)
    blocks = [root * k, np.sqrt(mu) * root * np.maximum(np.abs(k) - limit, 0.0)]
    if length_weight:
        h = float(np.mean(np.linalg.norm(np.roll(c, -1, axis=0) - c, axis=1)))
        segments = np.sqrt(length_weight / h) * (np.roll(p, -1, axis=0) - p)
        blocks += [segments[:, 0], segments[:, 1]]
    return np.concatenate(blocks)


def _jacobian(c, n, alpha, limit, mu, length_weight=0.0, eps=1e-6):
    """
    Sparse Jacobian of _residuals with respect to alpha.

    Residual i of every block depends on alpha[i-1], alpha[i] and alpha[i+1] only (a segment's,
    on alpha[i] and alpha[i+1]).
    Offsets of one colour, three apart, are perturbed together: each residual then sees at most
    one of them, so one central difference per colour gives every entry. When the loop length is
    not a multiple of three, its last offsets get colours of their own.
    """
    size = len(alpha)
    tail = size % 3
    colour = np.arange(size) % 3
    if tail:
        colour[size - tail:] = 3 + np.arange(tail)
    neighbours = (np.arange(size)[:, None] + np.array([-1, 0, 1])) % size  # (size, 3)
    rows, cols, values = [], [], []
    for group in range(colour.max() + 1):
        mask = colour == group
        delta = np.where(mask, eps, 0.0)
        change = (_residuals(c, n, alpha + delta, limit, mu, length_weight)
                  - _residuals(c, n, alpha - delta, limit, mu, length_weight)) / (2.0 * eps)
        hit = mask[neighbours]  # which neighbour of residual i is in this group
        i, which = np.nonzero(hit)
        for block in range(0, len(change), size):
            rows.append(block + i)
            cols.append(neighbours[i, which])
            values.append(change[block + i])
    entries = (np.concatenate(values), (np.concatenate(rows), np.concatenate(cols)))
    return sparse.csr_matrix(entries, shape=(len(change), size))


def box_qp(h, g, lower, upper, x0, max_iterations=100, tolerance=1e-9):
    """
    Minimise 0.5 x'Hx - g'x over lower <= x <= upper, H sparse and positive definite.

    Projected Newton (Bertsekas, 1982): Newton steps on the variables not held at a bound by
    their gradient, an Armijo search along the projection of that step onto the box.
    """
    x = np.clip(x0, lower, upper)
    h = sparse.csr_matrix(h)
    cost = lambda v: 0.5 * v @ (h @ v) - g @ v  # noqa: E731
    f = cost(x)
    for _ in range(max_iterations):
        grad = h @ x - g
        held = ((x <= lower + 1e-12) & (grad > 0)) | ((x >= upper - 1e-12) & (grad < 0))
        free = np.flatnonzero(~held)
        if not len(free) or np.max(np.abs(grad[free])) < tolerance:
            break
        step = np.zeros_like(x)
        step[free] = spsolve(h[free][:, free].tocsc(), -grad[free])
        t = 1.0
        while True:
            candidate = np.clip(x + t * step, lower, upper)
            f_candidate = cost(candidate)
            if f_candidate <= f + 1e-4 * grad @ (candidate - x) or t < 1e-10:
                break
            t /= 2.0
        if f - f_candidate < tolerance * max(abs(f), 1.0):
            if f_candidate < f:
                x = candidate
            break
        x, f = candidate, f_candidate
    return x


def min_curvature(c, n, lower, upper, curvature_limit, alpha=None, mu=STEERING_PENALTY,
                  length_weight=0.0, max_steps=500, tolerance=1e-7):
    """
    Offsets alpha (m, left positive) of the minimum-curvature line within [lower, upper].

    With `length_weight` w (1/m^2) it minimises curvature + w x lap length instead: 0 is pure
    minimum curvature, larger weights pull towards the shortest path. `c` must be uniformly
    spaced. Levenberg-Marquardt from `alpha` (default: the centerline,
    clipped to the bounds); stops when three steps in a row lower the objective by less than
    `tolerance` (relative), or when the damping grows so large that no step helps.
    """
    alpha = np.clip(np.zeros(len(c)) if alpha is None else alpha, lower, upper)
    r = _residuals(c, n, alpha, curvature_limit, mu, length_weight)
    f = float(r @ r + SMOOTHING * alpha @ alpha)
    identity = sparse.identity(len(c), format='csr')
    damping, small = None, 0
    for _ in range(max_steps):
        jac = _jacobian(c, n, alpha, curvature_limit, mu, length_weight)
        gauss_newton = (jac.T @ jac).tocsr() + SMOOTHING * identity
        gradient = jac.T @ r + SMOOTHING * alpha  # half the gradient of F
        if damping is None:
            damping = 1e-3 * float(gauss_newton.diagonal().max())
        # Model: F(alpha + d) ~ f + 2 gradient.d + d'(J'J + S I)d, d within the bounds.
        step = box_qp(gauss_newton + damping * identity, -gradient,
                      lower - alpha, upper - alpha, np.zeros(len(c)))
        predicted = -(2.0 * gradient @ step + step @ (gauss_newton @ step))
        candidate = alpha + step
        r_candidate = _residuals(c, n, candidate, curvature_limit, mu, length_weight)
        f_candidate = float(r_candidate @ r_candidate + SMOOTHING * candidate @ candidate)
        ratio = (f - f_candidate) / predicted if predicted > 0 else -1.0
        if ratio > 0:
            small = small + 1 if f - f_candidate < tolerance * f else 0
            alpha, r, f = candidate, r_candidate, f_candidate
            if small >= 3:
                break
        damping *= 1.0 / 3.0 if ratio > 0.75 else (2.0 if ratio < 0.25 else 1.0)
        if damping > 1e12:
            break
    return alpha


def check_fit(track_map, xy, footprint, margin, wheelbase, max_steering, curvature_spacing=0.25):
    """Sweep the footprint along the closed line `xy` and check the steering limit (FitReport)."""
    poses_xy = resample(xy, POSE_STEP)
    yaw = headings(poses_xy)
    body = footprint.body_points()
    c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    world = np.stack([poses_xy[:, None, 0] + c * body[:, 0] - s * body[:, 1],
                      poses_xy[:, None, 1] + s * body[:, 0] + c * body[:, 1]], axis=-1)
    clearances = track_map.clearance_at(world)
    nearest = np.argmin(clearances, axis=1)
    pose_clearance = clearances[np.arange(len(nearest)), nearest]
    pose_side = np.where(body[nearest, 1] >= 0.0, 1, -1)
    pose_s, length = arc_lengths(poses_xy)
    worst = int(np.argmin(pose_clearance))

    violations = []
    bad = pose_clearance < margin
    if bad.all():
        violations = [(0.0, length)]
    elif bad.any():
        # Stretches of consecutive failing poses, starting after a passing one (wraps).
        shift = int(np.argmin(bad))
        rolled = np.roll(bad, -shift).astype(int)
        edges = np.flatnonzero(np.diff(np.concatenate([[0], rolled, [0]])))
        for i0, i1 in zip(edges[::2], edges[1::2]):
            a, b = (i0 + shift) % len(bad), (i1 - 1 + shift) % len(bad)
            violations.append((float(pose_s[a]), float(pose_s[b])))

    k = curvature(xy, curvature_spacing)
    line_s, _ = arc_lengths(xy)
    sharpest = int(np.argmax(k))
    return FitReport(
        length=length, min_clearance=float(pose_clearance[worst]), worst_s=float(pose_s[worst]),
        worst_xy=poses_xy[worst], violations=violations, max_curvature=float(k[sharpest]),
        max_curvature_s=float(line_s[sharpest]), max_curvature_xy=xy[sharpest],
        curvature_limit=float(np.tan(max_steering) / wheelbase), margin=margin,
        poses=np.column_stack([poses_xy, yaw]), pose_s=pose_s, pose_clearance=pose_clearance,
        pose_side=pose_side)


def optimise(track_map, centerline_xy, footprint, margin, wheelbase, max_steering, step=0.1,
             length_weight=0.0, fit_rounds=5, log=print):
    """
    Minimum-curvature raceline from a centerline that the car fits along (module docstring).

    `length_weight` trades curvature for lap length (min_curvature).

    Returns (xy, report): the uniformly spaced closed line, starting next to the centerline's
    first point, and its FitReport. The caller decides what to do when report.ok is False.
    """
    c = resample(centerline_xy, step, centerline_xy[0])
    n = normals(c)
    w_right, w_left = corridor_widths(track_map, c, n)
    # The objective aims 5% under the steering limit: check_fit measures the curvature over a
    # different span (curvature_spacing) than the optimiser's neighbouring points.
    limit = 0.95 * float(np.tan(max_steering) / wheelbase)
    extra = {-1: np.zeros(len(c)), 1: np.zeros(len(c))}  # added clearance, right and left
    alpha = None
    for round_ in range(1, fit_rounds + 1):
        lateral = footprint.width / 2 + margin
        lower, upper = offset_bounds(c, w_right, w_left, lateral + extra[-1], lateral + extra[1])
        alpha = min_curvature(c, n, lower, upper, limit, alpha, length_weight=length_weight)
        p = c + alpha[:, None] * n
        report = check_fit(track_map, p, footprint, margin, wheelbase, max_steering)
        log(f'round {round_}: length {report.length:.2f} m, max curvature '
            f'{report.max_curvature:.3f} 1/m, footprint clearance {report.min_clearance:.3f} m'
            + ('' if report.fits else f', {len(report.violations)} stretches too close'))
        if report.fits:
            break
        # Tighten that side's bound on the base points the failing poses are nearest to, by the
        # deficit plus a centimetre so the next round does not land just short again.
        bad = report.pose_clearance < margin
        nearest = cKDTree(p).query(report.poses[bad, :2])[1]
        deficit = margin - report.pose_clearance[bad] + 0.01
        spread = 2 * int(round(WIDTH_WINDOW / step)) + 1
        for side in (-1, 1):
            on_side = report.pose_side[bad] == side
            needed = np.zeros(len(c))
            np.maximum.at(needed, nearest[on_side], deficit[on_side])
            extra[side] += maximum_filter1d(needed, size=spread, mode='wrap')
    return resample(p, step, centerline_xy[0]), report


def lap_time(xy, speeds):
    """Time (s) to drive the closed line `xy` at `speeds`, linear in speed along each segment."""
    d = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)
    return float(np.sum(2.0 * d / (speeds + np.roll(speeds, -1))))


def raceline_rows(xy, speeds, curvature_spacing=0.5):
    """Rows s, x, y, psi, kappa, vx, ax of the TUM raceline format (psi is the ROS yaw)."""
    s, _ = arc_lengths(xy)
    d = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)
    ax = (np.roll(speeds, -1) ** 2 - speeds ** 2) / (2.0 * d)
    return np.column_stack([s, xy, headings(xy), curvature(xy, curvature_spacing, signed=True),
                            speeds, ax])


def write_raceline(path, rows, comment):
    """Write a raceline CSV (`s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`)."""
    with open(path, 'w') as f:
        f.write(f'# {comment}\n')
        f.write('# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n')
        np.savetxt(f, rows, fmt='%.7f', delimiter='; ')
