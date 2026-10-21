import math

import numpy as np
from PIL import Image
import pytest
from rbsf_mvsim.scene import corridor, curb_quads, curb_stripes, Grid, scenery, STRIPE
import yaml

RESOLUTION = 0.05


@pytest.fixture
def ring_map(tmp_path):
    """Write a 1 m wide square ring corridor inside walls, plus a free pocket outside it."""
    image = np.zeros((100, 100), dtype=np.uint8)  # 5 x 5 m, all occupied
    image[10:90, 10:90] = 255
    image[30:70, 30:70] = 0  # infield
    image[0:5, 0:5] = 255  # unconnected free pocket
    Image.fromarray(image).save(tmp_path / 'ring.png')
    with open(tmp_path / 'ring.yaml', 'w') as f:
        yaml.safe_dump({'image': 'ring.png', 'resolution': RESOLUTION, 'origin': [0.0, 0.0, 0.0],
                        'negate': 0, 'occupied_thresh': 0.45, 'free_thresh': 0.196}, f)
    # Reference line through the middle of the corridor (x, y in 1..4 m), counter-clockwise.
    t = np.linspace(0.0, 1.0, 80, endpoint=False)
    xy = np.concatenate([np.stack([1.0 + 3.0 * t, np.full_like(t, 1.0)], 1),
                         np.stack([np.full_like(t, 4.0), 1.0 + 3.0 * t], 1),
                         np.stack([4.0 - 3.0 * t, np.full_like(t, 4.0)], 1),
                         np.stack([np.full_like(t, 1.0), 4.0 - 3.0 * t], 1)])
    return tmp_path / 'ring.yaml', xy


def test_corridor_is_the_free_space_around_the_reference_line(ring_map):
    grid = Grid(ring_map[0])
    road = corridor(grid, ring_map[1])
    assert road.sum() == 80 * 80 - 40 * 40
    assert not road[0, 0]  # the pocket is free but not connected
    rows, cols = grid.to_pixel(np.array([[1.0, 1.0]]))
    assert (rows[0], cols[0]) == (79, 20) and road[79, 20]


def test_curbs_alternate_colour_along_the_line(ring_map):
    grid = Grid(ring_map[0])
    road = corridor(grid, ring_map[1])
    stripes = curb_stripes(grid, road, ring_map[1], thickness=0.1)
    assert not (stripes[road]).any()
    # The outer wall's bottom row (y in 0.40..0.45 m), along x: STRIPE m per colour.
    row = stripes[91, 20:80]
    assert set(row) == {1, 2}
    changes = np.flatnonzero(np.diff(row.astype(int)) != 0)
    assert np.diff(changes).min() >= round(STRIPE / RESOLUTION) - 1


def test_curb_mesh_is_a_closed_extrusion():
    stripes = np.zeros((4, 5), dtype=np.uint8)
    stripes[1:3, 1:4] = 1  # one 3 x 2 cell block
    stripes[1, 4] = 2  # and a red cell touching it
    quads = curb_quads(stripes, 1.0, (0.0, 0.0), height=0.5)
    white_tops = [q for q, n in quads[1] if n == (0, 0, 1)]
    assert len(white_tops) == 2  # one run per row
    for vertices, normal in quads[1] + quads[2]:
        a, b, c = (np.array(v) for v in vertices[:3])
        assert np.cross(b - a, c - a) / np.linalg.norm(np.cross(b - a, c - a)) == pytest.approx(
            np.array(normal, dtype=float))
    # No side face between the touching white and red cells.
    assert not [v for v, n in quads[1] if n == (1, 0, 0) and v[0][0] == 4.0
                and v[0][1] in (2.0, 3.0) and v[1][1] in (2.0, 3.0)]
    assert math.fsum(abs(v[2][0] - v[0][0]) + abs(v[2][1] - v[0][1])
                     for v, n in quads[2] if n == (0, 0, 1)) == pytest.approx(2.0)


def test_scenery_writes_assets_and_world_elements(ring_map, tmp_path):
    out = tmp_path / 'out'
    out.mkdir()
    lights, ground, curbs = scenery(*ring_map, out)
    assert lights.tag == 'lights' and ground.get('class') == 'horizontal_plane'
    assert float(ground.find('x_max').text) == pytest.approx(5.0)
    assert curbs.find('intangible').text == 'true'
    assert curbs.find('visual/model_uri').text == str(out / 'curbs.obj')
    texture = np.array(Image.open(out / 'circuit_surface.png'))
    assert texture.shape == (100, 100, 3)
    # Flipped for MVSim: row 0 is y_min, so the free pocket (top-left in the map) is at the end.
    assert (texture[-1, 0] != texture[50, 20]).any()
