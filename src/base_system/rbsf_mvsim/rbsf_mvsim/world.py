"""
MVSim world generation for the RBSF tracks.

MVSim defines the vehicles and their start poses inside the world file, so launch files build it
on the fly: the track's ROS map yaml as an occupancy grid (walls for the LiDAR and collisions) and
one F1TENTH car per entry in `cars`, blue for the ego and orange for opponents, with the car
model RViz shows for f1tenth_gym.

MVSim's 2D LiDAR misses most walls of maps whose whole outside is occupied (e.g. iros2026, filled
for f1tenth_gym): rays return max range and the car only finds the wall by hitting it. Launch
files therefore give MVSim outline_map(), which keeps only the occupied cells next to free space.
scene.py adds the race-circuit look (ground texture, curbs, lights) on top, from that same grid.
"""

import math
import os
import pathlib
import shutil
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
from PIL import Image
import yaml

OPP_NAMESPACE_PREFIX = 'opponents/or'
VEHICLE_CLASSES = {'ego': ('f1tenth_ego', '#3080ff'), 'opponent': ('f1tenth_opp', '#ff9020')}


def opponent_namespace(index):
    """Vehicle name and ROS namespace of opponent `index`, counted from 0 (as in the gym)."""
    return f'{OPP_NAMESPACE_PREFIX}{index}'


def occupied_cells(image, negate, occupied_thresh):
    """Occupied mask of a ROS map image (uint8), with map_server's trinary interpretation."""
    occupancy = image / 255.0 if negate else (255 - image.astype(np.float64)) / 255.0
    return occupancy > occupied_thresh


def outline(occupied, thickness_px):
    """Occupied cells within `thickness_px` cells (4-connected) of a free cell."""
    near_free = ~occupied
    for _ in range(thickness_px):
        grown = near_free.copy()
        grown[1:] |= near_free[:-1]
        grown[:-1] |= near_free[1:]
        grown[:, 1:] |= near_free[:, :-1]
        grown[:, :-1] |= near_free[:, 1:]
        near_free = grown
    return occupied & near_free


def outline_map(map_yaml, out_dir, thickness=0.2):
    """
    Write an outline copy of a ROS map (walls `thickness` m thick, rest free) to `out_dir`.

    Returns the path of the new map yaml. Maps drawn with thin walls come out the same.
    """
    map_yaml = pathlib.Path(map_yaml)
    with open(map_yaml) as f:
        spec = yaml.safe_load(f)
    image = np.array(Image.open(map_yaml.parent / spec['image']).convert('L'))
    occupied = occupied_cells(image, int(spec.get('negate', 0)), spec['occupied_thresh'])
    walls = outline(occupied, max(1, math.ceil(thickness / spec['resolution'])))
    out_dir = pathlib.Path(out_dir)
    stem = f'{map_yaml.stem}_outline'
    Image.fromarray(np.where(walls, 0, 255).astype(np.uint8)).save(out_dir / f'{stem}.png')
    spec.update(image=f'{stem}.png', negate=0)
    with open(out_dir / f'{stem}.yaml', 'w') as f:
        yaml.safe_dump(spec, f)
    return str(out_dir / f'{stem}.yaml')


def world_xml(map_yaml, cars, follow=None, definitions_dir=None, meshes_dir=None,
              work_dir=None, scenery=()):
    """
    Return an MVSim world XML string.

    map_yaml: absolute path of a ROS map yaml. cars: (name, x, y, yaw_rad, kind) tuples with
    kind 'ego' or 'opponent'; the name is also the ROS namespace (r0, opponents/or0, ...).
    follow: vehicle the GUI camera follows. meshes_dir: the car model's meshes (default: the
    ones of f1tenth_gym_ros, which RViz shows with f1tenth_gym). work_dir: where the per-class
    copies of the accent mesh go (MVSim tints a mesh once per file, not once per vehicle).
    scenery: extra world elements, e.g. from rbsf_mvsim.scene.scenery().
    """
    if meshes_dir is None:
        meshes_dir = os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'meshes')
    work_dir = pathlib.Path(work_dir or tempfile.mkdtemp(prefix='rbsf_mvsim_'))
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
    world.extend(scenery)

    grid = ET.SubElement(world, 'element', {'class': 'occupancy_grid'})
    ET.SubElement(grid, 'file').text = str(map_yaml)

    vehicle_file = os.path.join(definitions_dir, 'f1tenth.vehicle.xml')
    for class_name, color in VEHICLE_CLASSES.values():
        accent = work_dir / f'{class_name}_accent.stl'
        shutil.copy(os.path.join(meshes_dir, 'roboracer_accent.stl'), accent)
        ET.SubElement(world, 'include', file=vehicle_file, class_name=class_name,
                      chassis_color=color, meshes_dir=str(meshes_dir), accent_mesh=str(accent))
    for name, x, y, yaw, kind in cars:
        vehicle = ET.SubElement(world, 'vehicle', name=name, **{'class': VEHICLE_CLASSES[kind][0]})
        ET.SubElement(vehicle, 'init_pose').text = f'{x:.4f} {y:.4f} {math.degrees(yaw):.2f}'

    ET.indent(world)
    body = ET.tostring(world, encoding='unicode')
    return '<?xml version="1.0" encoding="utf-8"?>\n' + body + '\n'
