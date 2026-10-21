"""
Bridges for the MVSim worlds: elevated roads a track yaml declares on top of its 2D map.

A map yaml may list bridges under a `bridges:` key, in map coordinates (m):

    bridges:
    - start: [x, y]   # foot of the first ramp, on the road's axis
      end: [x, y]     # foot of the second ramp; the axis must be parallel to x or y
      deck: [s0, s1]  # the flat deck spans these distances along the axis from `start`
      width: 1.14     # road width
      height: 0.7     # deck height
      under_cones:    # optional: cone rows on the ground under the deck, [x0, y0, x1, y1] each
      - [x0, y0, x1, y1]

The map image stays 2D: its walls bound the ramps, and the crossing road under the deck is free
in it. MVSim adds the third dimension (bridge_elements()):

- the road surface (both ramps and the deck) is one elevation map exactly as wide as the road.
  A car whose wheels are on it climbs (MVSim takes the highest surface below each wheel's axle),
  a car below the deck ignores it. One continuous surface on purpose: with a block per ramp and
  deck, MVSim 1.5 dropped a climbing car to the ground whenever its front wheels reached the
  next block;
- two rails along the deck, RAIL_HEIGHT above it, bound the deck where the map has no walls:
  MVSim's terrain collision stops a car on the deck at any surface more than half a wheel above
  its wheels, while a car below looks no higher than its own chassis and passes under;
- cone rows under the deck bound the roads below it (where they meet the space under the ramps,
  or lane dividers the deck's road cuts in the map), for cars at ground level only (a car on the
  deck passes over them). The same rule, mirrored: the cones (CONE_HEIGHT, taller than a
  chassis) are too high to count, so a continuous base
  BASE_HEIGHT high under each row is what stops a car, and it also closes the gaps between
  cones. They cannot be `cones:` of the map (rbsf_worlds/cones.py): the map's walls stop cars at
  every height, the one on the deck included.

Limits: the LiDAR stays 2D (MVSim's 2D raytracing sees the map and tangible blocks only), so a
car on the deck sees the walls below through the crossing and a car below sees neither the ramps
nor the cones under the deck.
Two cars at different levels still collide (MVSim's Box2D physics is 2D). f1tenth_gym has no 3D:
there the bridge is a flat crossing, without the cones under it.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np
from rbsf_worlds.cones import cone_positions, CONE_RADIUS
import yaml

RESOLUTION = 0.04  # elevation map cell (m)
RAIL_HEIGHT = 0.12  # below the car's chassis top (0.18 m): MVSim only collides with lower tops
RAIL_WIDTH = 0.05
MAX_PIECE = 4.0  # MVSim finds blocks by the 4 m cells holding their corners
CONE_HEIGHT = 0.3
BASE_HEIGHT = 0.1  # above half a wheel (0.055 m) and below the chassis top (0.18 m)
BASE_WIDTH = 0.1
DECK_COLOR, RAIL_COLOR, CONE_COLOR = '#8a8d91', '#d8dadc', '#ff6a13'


def load_bridges(map_yaml):
    """`bridges:` entries of a map yaml, validated (empty if it has none)."""
    with open(map_yaml) as f:
        spec = yaml.safe_load(f) or {}
    bridges = spec.get('bridges') or []
    for bridge in bridges:
        missing = {'start', 'end', 'deck', 'width', 'height'} - set(bridge)
        if missing:
            raise ValueError(f'{map_yaml}: bridge {bridge} lacks {sorted(missing)}')
        (x0, y0), (x1, y1) = bridge['start'], bridge['end']
        if x0 != x1 and y0 != y1:
            raise ValueError(f'{map_yaml}: bridge {bridge}: its axis must be parallel to x or y')
        length = math.dist(bridge['start'], bridge['end'])
        s0, s1 = bridge['deck']
        if not 0 < s0 < s1 < length:
            raise ValueError(f'{map_yaml}: bridge deck {bridge["deck"]} must lie strictly '
                             f'inside the ramps (0, {length:.2f})')
        for row in bridge.get('under_cones') or []:
            if len(row) != 4:
                raise ValueError(f'{map_yaml}: bridge cone row {row}: expected [x0, y0, x1, y1]')
    return bridges


def profile(s, length, s0, s1, height):
    """Road height at distances `s` along the axis: ramp up, deck, ramp down."""
    s = np.asarray(s, float)
    return height * np.clip(np.minimum(s / s0, (length - s) / (length - s1)), 0.0, 1.0)


def surface(bridge, resolution=RESOLUTION):
    """(corner_min_x, corner_min_y, z) of the road's elevation map: z[i, j] at x_i, y_j."""
    start, end = np.array(bridge['start'], float), np.array(bridge['end'], float)
    length = float(np.linalg.norm(end - start))
    along_x = start[1] == end[1]
    axis = 0 if along_x else 1
    lo = min(start[axis], end[axis])
    # Grid nodes along the road (from its lower coordinate) and across it (centred on the axis).
    n_along = int(math.ceil(length / resolution)) + 1
    n_across = int(round(float(bridge['width']) / resolution)) + 1
    s = np.abs(lo + np.arange(n_along) * resolution - start[axis])  # distance from `start`
    z_along = profile(s, length, *bridge['deck'], float(bridge['height']))
    across0 = start[1 - axis] - (n_across - 1) * resolution / 2
    z = np.repeat(z_along[:, None], n_across, axis=1)
    if along_x:
        return lo, across0, z
    return across0, lo, z.T


def _elevation_map(corner_x, corner_y, z, resolution=RESOLUTION):
    element = ET.Element('element', {'class': 'elevation_map'})
    rows = '; '.join(' '.join(f'{v:.4f}' for v in row) for row in z)
    for tag, value in (('resolution', f'{resolution}'), ('corner_min_x', f'{corner_x:.4f}'),
                       ('corner_min_y', f'{corner_y:.4f}'), ('mesh_color', DECK_COLOR),
                       ('elevation_data_matrix', f'[{rows}]')):
        ET.SubElement(element, tag).text = value
    return element


def _pieces(a, b):
    """Split [a, b] into equal intervals no longer than MAX_PIECE."""
    n = max(1, math.ceil((b - a) / MAX_PIECE - 1e-9))
    edges = np.linspace(a, b, n + 1)
    return list(zip(edges[:-1], edges[1:]))


def _prism(name, polygon, height, x, y, z, yaw, color):
    """
    Make a static intangible MVSim block: `polygon` (local x, y) extruded to `height`, posed in 3D.

    A plain shaped block: MVSim 1.5 crashes on intangible blocks with a box geometry.
    """
    block = ET.Element('block', name=name)
    for tag, value in (('static', 'true'), ('intangible', 'true'), ('color', color),
                       ('zmin', '0'), ('zmax', f'{height:.4f}')):
        ET.SubElement(block, tag).text = value
    shape = ET.SubElement(block, 'shape')
    for px, py in polygon:
        ET.SubElement(shape, 'pt').text = f'{px:.4f} {py:.4f}'
    ET.SubElement(block, 'init_pose3d').text = (
        f'{x:.4f} {y:.4f} {z:.4f} {math.degrees(yaw):.3f} 0 0')
    # Otherwise MVSim drops the block onto the highest surface below it, ignoring z.
    ET.SubElement(block, 'skip_elevation_adjust')
    return block


def _box(name, size, x, y, z, yaw, color):
    """Make a box block spanning local (0, 0, 0)-(lx, ly, lz), posed by that corner."""
    lx, ly, lz = size
    return _prism(name, ((0, 0), (lx, 0), (lx, ly), (0, ly)), lz, x, y, z, yaw, color)


def cone_row(name, row):
    """Blocks of one cone row on the ground: its continuous base, then one block per cone."""
    p0, p1 = np.array(row[:2], float), np.array(row[2:], float)
    length = float(np.linalg.norm(p1 - p0))
    u = (p1 - p0) / length
    n = np.array([-u[1], u[0]])
    corner = p0 - u * CONE_RADIUS - n * BASE_WIDTH / 2
    blocks = [_box(f'{name}_base{j}', (b - a, BASE_WIDTH, BASE_HEIGHT),
                   *(corner + a * u), 0.0, math.atan2(u[1], u[0]), CONE_COLOR)
              for j, (a, b) in enumerate(_pieces(0.0, length + 2 * CONE_RADIUS))]
    octagon = [(CONE_RADIUS * math.cos(a), CONE_RADIUS * math.sin(a))
               for a in np.arange(8) * math.pi / 4]
    blocks += [_prism(f'{name}_cone{j}', octagon, CONE_HEIGHT, x, y, 0.0, 0.0, CONE_COLOR)
               for j, (x, y) in enumerate(cone_positions([row]))]
    return blocks


def bridge_elements(bridges):
    """Return the MVSim elements (road surfaces, rails, cones) of a yaml's `bridges:`."""
    elements = []
    for k, bridge in enumerate(bridges):
        elements.append(_elevation_map(*surface(bridge)))
        start, end = np.array(bridge['start'], float), np.array(bridge['end'], float)
        u = (end - start) / np.linalg.norm(end - start)  # along the road
        n = np.array([-u[1], u[0]])  # to its left
        yaw = math.atan2(u[1], u[0])
        width, height = float(bridge['width']), float(bridge['height'])
        for j, (a, b) in enumerate(_pieces(*bridge['deck'])):
            for side, offset in (('right', -width / 2 - RAIL_WIDTH), ('left', width / 2)):
                x, y = start + a * u + n * offset
                elements.append(_box(f'bridge{k}_rail_{side}{j}', (b - a, RAIL_WIDTH, RAIL_HEIGHT),
                                     x, y, height, yaw, RAIL_COLOR))
        for j, row in enumerate(bridge.get('under_cones') or []):
            elements += cone_row(f'bridge{k}_cones{j}', row)
    return elements
