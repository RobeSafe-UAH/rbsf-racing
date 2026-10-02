import math
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest
from rbsf_mvsim.mvsim_adapter import ackermann_to_twist, wheel_quaternion
from rbsf_mvsim.world import occupied_cells, opponent_namespace, outline, outline_map, world_xml
import yaml


def test_world_has_grid_classes_and_one_vehicle_per_car():
    cars = [('r0', 1.0, 2.0, 0.0, 'ego'),
            (opponent_namespace(0), 3.0, 4.0, math.pi / 2, 'opponent')]
    root = ET.fromstring(world_xml('/tracks/levine/levine.yaml', cars, follow='r0',
                                   definitions_dir='/defs'))
    assert root.find("element[@class='occupancy_grid']/file").text == '/tracks/levine/levine.yaml'
    includes = root.findall('include')
    assert {i.get('class_name') for i in includes} == {'f1tenth_ego', 'f1tenth_opp'}
    assert all(i.get('file') == '/defs/f1tenth.vehicle.xml' for i in includes)
    vehicles = root.findall('vehicle')
    assert [(v.get('name'), v.get('class')) for v in vehicles] == [
        ('r0', 'f1tenth_ego'), ('opponents/or0', 'f1tenth_opp')]
    x, y, yaw_deg = map(float, vehicles[1].find('init_pose').text.split())
    assert (x, y, yaw_deg) == pytest.approx((3.0, 4.0, 90.0))
    assert root.find('gui/follow_vehicle').text == 'r0'


def test_occupied_cells_follow_map_server_thresholds():
    image = np.array([[0, 100, 200, 255]], dtype=np.uint8)
    assert occupied_cells(image, 0, 0.45).tolist() == [[True, True, False, False]]
    assert occupied_cells(image, 1, 0.45).tolist() == [[False, False, True, True]]


def test_outline_keeps_walls_next_to_free_space():
    occupied = np.ones((9, 9), dtype=bool)
    occupied[3:6, 3:6] = False  # a free room inside solid fill
    walls = outline(occupied, 1)
    assert walls.sum() == 12  # the ring around the room (4-connected), corners excluded
    assert not walls[0, 0] and walls[2, 4] and not walls[3, 3]
    thin = np.zeros((5, 5), dtype=bool)
    thin[2, :] = True  # a thin wall stays as it is
    assert (outline(thin, 2) == thin).all()


def test_outline_map_writes_a_map_next_to_the_world(tmp_path):
    image = np.zeros((10, 10), dtype=np.uint8)
    image[4:6, 4:6] = 255
    Image.fromarray(image).save(tmp_path / 'track.png')
    with open(tmp_path / 'track.yaml', 'w') as f:
        yaml.safe_dump({'image': 'track.png', 'resolution': 0.05, 'origin': [1.0, 2.0, 0.0],
                        'negate': 0, 'occupied_thresh': 0.45, 'free_thresh': 0.196}, f)
    out = tmp_path / 'out'
    out.mkdir()
    path = outline_map(tmp_path / 'track.yaml', out, thickness=0.05)
    with open(path) as f:
        spec = yaml.safe_load(f)
    assert spec['origin'] == [1.0, 2.0, 0.0] and spec['resolution'] == 0.05
    walls = np.array(Image.open(out / spec['image'])) == 0
    assert walls.sum() == 8 and not walls[0, 0]


def test_ackermann_to_twist_is_the_bicycle_model():
    assert ackermann_to_twist(2.0, 0.0, 0.33) == (2.0, 0.0)
    v, w = ackermann_to_twist(2.0, 0.3, 0.33)
    # Turning radius R = L / tan(delta), and w = v / R.
    assert w == pytest.approx(2.0 / (0.33 / math.tan(0.3)))
    assert ackermann_to_twist(-1.0, 0.3, 0.33)[1] < 0.0  # reversing turns the other way


def test_wheel_quaternion_is_a_unit_steer_then_roll():
    assert wheel_quaternion(0.0, 0.0) == (0.0, 0.0, 0.0, 1.0)
    x, y, z, w = wheel_quaternion(0.4, 1.0)
    assert x * x + y * y + z * z + w * w == pytest.approx(1.0)
    assert wheel_quaternion(0.4, 0.0)[2] == pytest.approx(math.sin(0.2))  # pure steer: about z
    assert wheel_quaternion(0.0, 1.0)[1] == pytest.approx(math.sin(0.5))  # pure roll: about y
