"""
Race-circuit scenery for the MVSim worlds: ground texture, curbs and lights. Visual only.

Adapted from RobeSafe-UAH/sci_practica_final, src/sci_mvsim/sci_mvsim/scene.py (Apache-2.0),
which draws a parametric rounded-rectangle road. Here everything comes from the occupancy grid
MVSim simulates (outline_map) and the track's reference line, so it fits any track:

- the corridor is the free cells connected (4-neighbours) to the reference line;
- the curbs are the wall cells next to the corridor, red and white every STRIPE metres of the
  reference line. They are extruded from the same cells the LiDAR and the collisions use, so the
  3D view matches the physics;
- the ground texture has grass, asphalt on the corridor, a white edge line, the curbs and a
  chequered finish line across the corridor at the reference line's first point.

The curbs are an intangible block and the ground a horizontal plane above the grid's own flat
rendering, so neither changes the simulation.
"""

import math
import pathlib
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image
from rbsf_mvsim.world import occupied_cells
from rbsf_worlds.paths import arc_lengths
from scipy.spatial import cKDTree
import yaml

STRIPE = 0.8  # m of curb per colour
CURB_HEIGHT = 0.25
CURB_WIDTH = 0.2  # matches outline_map's default wall thickness
EDGE_LINE = (0.1, 0.18)  # white line, distance range from the wall (m)
CHECKER = 0.2  # finish-line square side (m), two rows
GRASS, ASPHALT = (62, 91, 62), (54, 59, 65)
WHITE, RED, LINE = (212, 222, 224), (186, 46, 43), (232, 237, 242)
CHECKER_COLORS = ((52, 57, 64), (237, 240, 237))
MATERIALS = {1: ('white', '0.83 0.87 0.88'), 2: ('red', '0.73 0.18 0.17')}
LIGHTS = (('enable_shadows', 'true'), ('shadow_map_size', 1024),
          ('light_azimuth_deg', 45), ('light_elevation_deg', 65),
          ('light_ambient', 0.45), ('light_diffuse', 0.75), ('light_specular', 0.25))


class Grid:
    """A ROS map's occupied mask and its pixel <-> world transforms (row 0 is the top, y max)."""

    def __init__(self, map_yaml):
        map_yaml = pathlib.Path(map_yaml)
        with open(map_yaml) as f:
            spec = yaml.safe_load(f)
        image = np.array(Image.open(map_yaml.parent / spec['image']).convert('L'))
        self.occupied = occupied_cells(image, int(spec.get('negate', 0)), spec['occupied_thresh'])
        self.resolution = float(spec['resolution'])
        self.origin = np.array(spec['origin'][:2], dtype=float)
        self.height, self.width = self.occupied.shape

    def to_pixel(self, xy):
        """(row, col) integer cells of world points (N, 2)."""
        col = np.floor((xy[:, 0] - self.origin[0]) / self.resolution).astype(int)
        row = self.height - 1 - np.floor((xy[:, 1] - self.origin[1]) / self.resolution).astype(int)
        return row, col

    def centers(self, rows, cols):
        """World (N, 2) centres of cells."""
        return np.stack([self.origin[0] + (cols + 0.5) * self.resolution,
                         self.origin[1] + (self.height - rows - 0.5) * self.resolution], axis=1)


def corridor(grid, xy):
    """Free cells connected to the reference line `xy`."""
    free = (~grid.occupied).astype(np.uint8)
    _, labels = cv2.connectedComponents(free, connectivity=4)
    rows, cols = grid.to_pixel(xy)
    inside = (rows >= 0) & (rows < grid.height) & (cols >= 0) & (cols < grid.width)
    seeds = labels[rows[inside], cols[inside]]
    seeds = np.unique(seeds[free[rows[inside], cols[inside]] > 0])
    if not len(seeds):
        raise ValueError('the reference line does not cross any free cell of the map')
    return np.isin(labels, seeds) & (free > 0)


def curb_stripes(grid, road, xy, thickness=CURB_WIDTH):
    """Label image: 0 no curb, 1 white, 2 red. Colour by the arc length of the nearest point."""
    steps = max(1, math.ceil(thickness / grid.resolution))
    cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    near = cv2.dilate(road.astype(np.uint8), cross, iterations=steps) > 0
    rows, cols = np.nonzero(grid.occupied & near)
    cumulative, _ = arc_lengths(xy)
    _, nearest = cKDTree(xy).query(grid.centers(rows, cols))
    labels = np.zeros(road.shape, np.uint8)
    labels[rows, cols] = 1 + (cumulative[nearest] // STRIPE).astype(int) % 2
    return labels


def cross_section(grid, road, xy):
    """Extent (right, left) in m of the corridor across the reference line's first point."""
    heading = xy[1] - xy[0]
    normal = np.array([-heading[1], heading[0]]) / np.linalg.norm(heading)  # to the left
    extent = []
    for side in (-1.0, 1.0):
        d = 0.0
        while d < 10.0:
            rows, cols = grid.to_pixel((xy[0] + side * (d + grid.resolution / 2) * normal)[None])
            if not (0 <= rows[0] < grid.height and 0 <= cols[0] < grid.width
                    and road[rows[0], cols[0]]):
                break
            d += grid.resolution / 2
        extent.append(d)
    return extent


def ground_texture(grid, road, stripes, xy, seed=2026):
    """RGB texture (row 0 at y max, like the map) of grass, asphalt, lines, curbs, finish."""
    noise = np.random.default_rng(seed).integers(-128, 128, road.shape)[..., None]

    def material(color, scale):
        return np.clip(np.array(color) + noise // scale, 0, 255)

    image = np.where(road[..., None], material(ASPHALT, 24), material(GRASS, 12))
    distance = cv2.distanceTransform(road.astype(np.uint8), cv2.DIST_L2, 3) * grid.resolution
    image[road & (distance > EDGE_LINE[0]) & (distance <= EDGE_LINE[1])] = LINE
    image[stripes == 1] = WHITE
    image[stripes == 2] = RED

    # Finish line: CHECKER squares, two rows ahead of the first point, across the corridor.
    right, left = cross_section(grid, road, xy)
    heading = (xy[1] - xy[0]) / np.linalg.norm(xy[1] - xy[0])
    rows, cols = np.nonzero(road)
    offset = grid.centers(rows, cols) - xy[0]
    u = offset @ heading
    v = offset @ np.array([-heading[1], heading[0]])
    on_line = (u >= 0) & (u < 2 * CHECKER) & (v >= -right) & (v <= left)
    square = (np.floor(u / CHECKER) + np.floor((v + right) / CHECKER)).astype(int) % 2
    for k, color in enumerate(CHECKER_COLORS):
        image[rows[on_line & (square == k)], cols[on_line & (square == k)]] = color
    return image.astype(np.uint8)


def _runs(values):
    """(start, end, value) of the runs of equal nonzero values in a 1D array, end exclusive."""
    edges = np.flatnonzero(np.diff(np.concatenate(([0], values, [0]))) != 0)
    return [(a, b, values[a]) for a, b in zip(edges[:-1], edges[1:]) if values[a]]


def curb_quads(stripes, resolution, origin, height=CURB_HEIGHT):
    """
    {label: [(4 vertices, normal)]} of the curb cells extruded to `height`.

    Cells are merged into runs (tops along rows, sides along the exposed edge) to keep the mesh
    small. Only sides facing a non-curb cell are emitted. Vertices are counter-clockwise seen
    from outside.
    """
    rows, cols = stripes.shape
    x = origin[0] + np.arange(cols + 1) * resolution  # cell edges
    y = origin[1] + (rows - np.arange(rows + 1)) * resolution  # row r spans y[r + 1]..y[r]
    padded = np.pad(stripes, 1)
    quads = {label: [] for label in MATERIALS}
    h = height
    for r in range(rows):
        for a, b, label in _runs(stripes[r]):
            quads[label].append((((x[a], y[r + 1], h), (x[b], y[r + 1], h), (x[b], y[r], h),
                                  (x[a], y[r], h)), (0, 0, 1)))
        for dr, normal in ((-1, (0, 1, 0)), (1, (0, -1, 0))):  # north (row above), south
            exposed = np.where(padded[r + 1 + dr, 1:-1] == 0, stripes[r], 0)
            ye = y[r] if dr < 0 else y[r + 1]
            for a, b, label in _runs(exposed):
                xa, xb = (x[b], x[a]) if dr < 0 else (x[a], x[b])
                quads[label].append((((xa, ye, 0), (xb, ye, 0), (xb, ye, h), (xa, ye, h)),
                                     normal))
    for c in range(cols):
        for dc, normal in ((1, (1, 0, 0)), (-1, (-1, 0, 0))):  # east, west
            exposed = np.where(padded[1:-1, c + 1 + dc] == 0, stripes[:, c], 0)
            xe = x[c + 1] if dc > 0 else x[c]
            for a, b, label in _runs(exposed):
                ya, yb = (y[b], y[a]) if dc > 0 else (y[a], y[b])
                quads[label].append((((xe, ya, 0), (xe, yb, 0), (xe, yb, h), (xe, ya, h)),
                                     normal))
    return quads


def write_obj(quads, out_dir):
    """Write curbs.obj + curbs.mtl to `out_dir`; return the obj path."""
    out_dir = pathlib.Path(out_dir)
    (out_dir / 'curbs.mtl').write_text(''.join(
        f'newmtl {name}\nKd {kd}\nKs 0.08 0.08 0.08\nNs 16\n' for name, kd in MATERIALS.values()))
    lines, count = ['mtllib curbs.mtl'], 0
    for label, faces in quads.items():
        lines.append(f'usemtl {MATERIALS[label][0]}')
        for vertices, normal in faces:
            lines.extend(f'v {vx:.4f} {vy:.4f} {vz:.4f}' for vx, vy, vz in vertices)
            count += 1
            lines.append('vn {} {} {}'.format(*normal))
            lines.append('f ' + ' '.join(f'{4 * (count - 1) + i + 1}//{count}' for i in range(4)))
    path = out_dir / 'curbs.obj'
    path.write_text('\n'.join(lines) + '\n')
    return path


def _tag(parent, name, value):
    ET.SubElement(parent, name).text = str(value)


def scenery(map_yaml, xy, out_dir):
    """
    Write the scenery assets to `out_dir` and return the MVSim world elements that use them.

    map_yaml: the grid MVSim simulates (outline_map output). xy: the track's reference line
    (N, 2), closed and ordered in the driving direction; its first point is the finish line.
    """
    out_dir = pathlib.Path(out_dir)
    grid = Grid(map_yaml)
    road = corridor(grid, xy)
    stripes = curb_stripes(grid, road, xy)
    texture = ground_texture(grid, road, stripes, xy)
    # MVSim maps the texture's first row to y_min; ROS maps put y_max there.
    Image.fromarray(texture[::-1]).save(out_dir / 'circuit_surface.png')
    mesh = write_obj(curb_quads(stripes, grid.resolution, grid.origin), out_dir)

    lights = ET.Element('lights')
    for name, value in LIGHTS:
        _tag(lights, name, value)
    size = (grid.width * grid.resolution, grid.height * grid.resolution)
    ground = ET.Element('element', {'class': 'horizontal_plane'})
    for name, value in (('x_min', grid.origin[0]), ('y_min', grid.origin[1]),
                        ('x_max', grid.origin[0] + size[0]), ('y_max', grid.origin[1] + size[1]),
                        ('z', 0.002), ('texture', out_dir / 'circuit_surface.png'),
                        ('texture_size_x', size[0]), ('texture_size_y', size[1])):
        _tag(ground, name, value)
    curbs = ET.Element('block', name='circuit_scenery')
    for name, value in (('static', 'true'), ('intangible', 'true'),
                        ('zmin', 0.0), ('zmax', 0.001), ('init_pose', '0 0 0')):
        _tag(curbs, name, value)
    _tag(ET.SubElement(curbs, 'visual'), 'model_uri', mesh)
    return [lights, ground, curbs]
