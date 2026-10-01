import math
import xml.etree.ElementTree as ET

import pytest
from rbsf_mvsim.mvsim_adapter import ackermann_to_twist
from rbsf_mvsim.world import opponent_namespace, world_xml


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


def test_ackermann_to_twist_is_the_bicycle_model():
    assert ackermann_to_twist(2.0, 0.0, 0.33) == (2.0, 0.0)
    v, w = ackermann_to_twist(2.0, 0.3, 0.33)
    # Turning radius R = L / tan(delta), and w = v / R.
    assert w == pytest.approx(2.0 / (0.33 / math.tan(0.3)))
    assert ackermann_to_twist(-1.0, 0.3, 0.33)[1] < 0.0  # reversing turns the other way
