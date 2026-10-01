"""
MVSim world generation for the RBSF tracks.

MVSim defines the vehicles and their start poses inside the world file, so launch files build it
on the fly: the track's ROS map yaml as an occupancy grid (walls for the LiDAR and collisions) and
one F1TENTH car per entry in `cars`, blue for the ego and orange for opponents.
"""

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

OPP_NAMESPACE_PREFIX = 'opponents/or'
VEHICLE_CLASSES = {'ego': ('f1tenth_ego', '#3080ff'), 'opponent': ('f1tenth_opp', '#ff9020')}


def opponent_namespace(index):
    """Vehicle name and ROS namespace of opponent `index`, counted from 0 (as in the gym)."""
    return f'{OPP_NAMESPACE_PREFIX}{index}'


def world_xml(map_yaml, cars, follow=None, definitions_dir=None):
    """
    Return an MVSim world XML string.

    map_yaml: absolute path of a ROS map yaml. cars: (name, x, y, yaw_rad, kind) tuples with
    kind 'ego' or 'opponent'; the name is also the ROS namespace (r0, opponents/or0, ...).
    follow: vehicle the GUI camera follows.
    """
    if definitions_dir is None:
        definitions_dir = os.path.join(get_package_share_directory('rbsf_mvsim'), 'definitions')
    world = ET.Element('mvsim_world', version='1.0')
    ET.SubElement(world, 'simul_timestep').text = '0'
    gui = ET.SubElement(world, 'gui')
    ET.SubElement(gui, 'ortho').text = 'false'
    ET.SubElement(gui, 'cam_distance').text = '8'
    ET.SubElement(gui, 'show_forces').text = 'false'
    if follow:
        ET.SubElement(gui, 'follow_vehicle').text = follow

    grid = ET.SubElement(world, 'element', {'class': 'occupancy_grid'})
    ET.SubElement(grid, 'file').text = str(map_yaml)

    vehicle_file = os.path.join(definitions_dir, 'f1tenth.vehicle.xml')
    for class_name, color in VEHICLE_CLASSES.values():
        ET.SubElement(world, 'include', file=vehicle_file, class_name=class_name,
                      chassis_color=color)
    for name, x, y, yaw, kind in cars:
        vehicle = ET.SubElement(world, 'vehicle', name=name, **{'class': VEHICLE_CLASSES[kind][0]})
        ET.SubElement(vehicle, 'init_pose').text = f'{x:.4f} {y:.4f} {math.degrees(yaw):.2f}'

    ET.indent(world)
    body = ET.tostring(world, encoding='unicode')
    return '<?xml version="1.0" encoding="utf-8"?>\n' + body + '\n'
