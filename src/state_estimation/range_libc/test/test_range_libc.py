import math

from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import range_libc

RESOLUTION = 0.05
WIDTH, HEIGHT = 120, 80        # cells: 6 m x 4 m, not square, so a transposed grid shows up
ORIGIN = (-1.0, -2.0)          # map coordinates of cell (0, 0)
MAX_RANGE_PX = 200


def box_map():
    """OccupancyGrid with a one-cell wall on its border: free inside x in [-0.95, 4.95)."""
    data = np.zeros((HEIGHT, WIDTH), dtype=np.int8)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
    msg = OccupancyGrid()
    msg.info.resolution = RESOLUTION
    msg.info.width, msg.info.height = WIDTH, HEIGHT
    msg.info.origin.position.x, msg.info.origin.position.y = ORIGIN
    msg.info.origin.orientation.w = 1.0
    msg.data = data.flatten().tolist()
    return msg


def expected(x, y, heading):
    """Distance (m) from (x, y) to the inner face of the border wall along `heading`."""
    x_min, x_max = ORIGIN[0] + RESOLUTION, ORIGIN[0] + (WIDTH - 1) * RESOLUTION
    y_min, y_max = ORIGIN[1] + RESOLUTION, ORIGIN[1] + (HEIGHT - 1) * RESOLUTION
    return {0.0: x_max - x, math.pi / 2: y_max - y,
            math.pi: x - x_min, -math.pi / 2: y - y_min}[heading]


METHODS = {
    'bl': lambda m: range_libc.PyBresenhamsLine(m, MAX_RANGE_PX),
    'rm': lambda m: range_libc.PyRayMarching(m, MAX_RANGE_PX),
    'cddt': lambda m: range_libc.PyCDDTCast(m, MAX_RANGE_PX, 112),
    'glt': lambda m: range_libc.PyGiantLUTCast(m, MAX_RANGE_PX, 112),
}


@pytest.mark.parametrize('method', METHODS)
def test_ranges_in_map_coordinates(method):
    caster = METHODS[method](range_libc.PyOMap(box_map()))
    poses = [(0.5, 0.0, h) for h in (0.0, math.pi / 2, math.pi, -math.pi / 2)] + [(3.0, -1.0, 0.0)]
    ins = np.array(poses, dtype=np.float32)
    outs = np.zeros(len(ins), dtype=np.float32)
    caster.calc_range_many(ins, outs)
    # calc_range_many returns metres when the map came from an OccupancyGrid.
    want = [expected(*pose) for pose in poses]
    np.testing.assert_allclose(outs, want, atol=2 * RESOLUTION)


def test_repeat_angles_matches_single_casts():
    caster = range_libc.PyCDDTCast(range_libc.PyOMap(box_map()), MAX_RANGE_PX, 112)
    poses = np.array([[0.5, 0.0, 0.0], [2.0, 1.0, math.pi / 2]], dtype=np.float32)
    angles = np.array([0.0, math.pi / 2], dtype=np.float32)
    outs = np.zeros(len(poses) * len(angles), dtype=np.float32)
    caster.calc_range_repeat_angles(poses, angles, outs)
    want = [expected(0.5, 0.0, 0.0), expected(0.5, 0.0, math.pi / 2),
            expected(2.0, 1.0, math.pi / 2), expected(2.0, 1.0, math.pi)]
    np.testing.assert_allclose(outs, want, atol=2 * RESOLUTION)
