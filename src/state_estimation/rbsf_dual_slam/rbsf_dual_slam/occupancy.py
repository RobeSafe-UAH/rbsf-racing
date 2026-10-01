"""
Occupancy grid -> PGM + map YAML, in the format of nav2's map_saver (no ROS imports).

Trinary mode, as nav2_map_server writes it: free (0..free_thresh) -> 254, occupied
(>= occupied_thresh) -> 0, unknown (-1 or in between) -> 205. The grid's row 0 is at the origin
(bottom of the image), so rows are flipped. This is the input of the offline map cleaning and
centerline tools (image + yaml).
"""

import math
import pathlib

import numpy as np
import yaml

FREE, OCCUPIED, UNKNOWN = 254, 0, 205
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.25


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def grid_to_image(data, width, height):
    """uint8 image (height x width, top row first) from row-major occupancy values (-1..100)."""
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    image = np.full(grid.shape, UNKNOWN, dtype=np.uint8)
    image[(grid >= 0) & (grid <= FREE_THRESH * 100)] = FREE
    image[grid >= OCCUPIED_THRESH * 100] = OCCUPIED
    return np.flipud(image)


def map_yaml(image_name, resolution, origin):
    """map_server YAML dict; origin is (x, y, yaw) of the bottom-left cell."""
    return {
        'image': image_name,
        'mode': 'trinary',
        'resolution': float(resolution),
        'origin': [float(v) for v in origin],
        'negate': 0,
        'occupied_thresh': OCCUPIED_THRESH,
        'free_thresh': FREE_THRESH,
    }


def write_pgm(path, image):
    """Binary (P5) PGM."""
    height, width = image.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(np.ascontiguousarray(image, dtype=np.uint8).tobytes())


def save_map(directory, data, width, height, resolution, origin, name='map'):
    """Write <directory>/<name>.pgm and <name>.yaml. Returns the YAML path."""
    directory = pathlib.Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    write_pgm(directory / f'{name}.pgm', grid_to_image(data, width, height))
    yaml_path = directory / f'{name}.yaml'
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(map_yaml(f'{name}.pgm', resolution, origin), f, sort_keys=False)
    return yaml_path
