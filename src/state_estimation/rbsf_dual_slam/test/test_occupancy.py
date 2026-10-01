import math

import numpy as np
import pytest
from rbsf_dual_slam.occupancy import (
    FREE, grid_to_image, map_yaml, OCCUPIED, save_map, UNKNOWN, yaw_from_quaternion)
import yaml


def test_trinary_thresholds():
    image = grid_to_image([-1, 0, 25, 26, 64, 65, 100], 7, 1)
    assert image.tolist() == [[UNKNOWN, FREE, FREE, UNKNOWN, UNKNOWN, OCCUPIED, OCCUPIED]]


def test_rows_are_flipped():
    # Row 0 of the grid (at the origin) is the bottom row of the image.
    image = grid_to_image([100, 100, 0, 0], 2, 2)
    assert image.tolist() == [[FREE, FREE], [OCCUPIED, OCCUPIED]]


def test_yaml_matches_map_server():
    spec = map_yaml('map.pgm', 0.05, (-1.0, 2.0, 0.0))
    assert spec == {'image': 'map.pgm', 'mode': 'trinary', 'resolution': 0.05,
                    'origin': [-1.0, 2.0, 0.0], 'negate': 0, 'occupied_thresh': 0.65,
                    'free_thresh': 0.25}


def test_yaw_from_quaternion():
    assert yaw_from_quaternion(0.0, 0.0, math.sin(0.4), math.cos(0.4)) == pytest.approx(0.8)


def test_save_map_round_trip(tmp_path):
    data = [0, 100, -1, 0, 0, 100]
    yaml_path = save_map(tmp_path / 'slam_toolbox', data, 3, 2, 0.1, (0.5, -0.5, 0.0))
    with open(yaml_path) as f:
        spec = yaml.safe_load(f)
    assert spec['image'] == 'map.pgm'
    raw = (tmp_path / 'slam_toolbox' / 'map.pgm').read_bytes()
    header = b'P5\n3 2\n255\n'
    assert raw.startswith(header)
    pixels = np.frombuffer(raw[len(header):], dtype=np.uint8).reshape(2, 3)
    assert pixels.tolist() == [[FREE, FREE, OCCUPIED], [FREE, OCCUPIED, UNKNOWN]]
