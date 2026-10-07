"""
Synthetic cones on a track: obstacles the track yaml declares on top of its map image.

A map yaml may list cones under a `cones:` key, in map coordinates (m). Each entry is either a
single cone `[x, y]` or a row of cones `[x0, y0, x1, y1]`, placed every `spacing` m from one
end to the other (both ends included). Rows close openings that lead off the lap (side
corridors, dead ends): with the default size and spacing the gaps are narrower than a car, and
the LiDAR sees the row as a wall.

The simulators load with_cones(), a copy of the map with every cone drawn as an occupied disc,
so both see the same obstacles in their collision checks, their LiDAR and /ground_truth/map.
No ROS dependencies.
"""

import math
import pathlib

import cv2
import yaml

CONE_RADIUS = 0.1   # m, a 20 cm cone base
CONE_SPACING = 0.25  # m between cone centres in a row: 5 cm gaps


def cone_positions(entries, spacing=CONE_SPACING):
    """(x, y) of every cone in the `cones:` entries of a map yaml."""
    cones = []
    for entry in entries or []:
        if len(entry) == 2:
            cones.append((float(entry[0]), float(entry[1])))
        elif len(entry) == 4:
            x0, y0, x1, y1 = (float(v) for v in entry)
            n = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / spacing))
            cones += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n + 1)]
        else:
            raise ValueError(f'cone entry {entry}: expected [x, y] or [x0, y0, x1, y1]')
    return cones


def load_cones(map_yaml, spacing=CONE_SPACING):
    """Cone positions declared by a map yaml (empty if it has no `cones:` key)."""
    with open(map_yaml) as f:
        spec = yaml.safe_load(f) or {}
    return cone_positions(spec.get('cones'), spacing)


def draw_cones(image, cones, resolution, origin, negate=0, radius=CONE_RADIUS):
    """Copy of a ROS map image (uint8, row 0 at the top) with each cone as an occupied disc."""
    out = image.copy()
    occupied = 255 if negate else 0
    height = image.shape[0]
    radius_px = max(1, round(radius / resolution))
    for x, y in cones:
        col = round((x - origin[0]) / resolution)
        row = height - 1 - round((y - origin[1]) / resolution)
        cv2.circle(out, (col, row), radius_px, occupied, thickness=-1)
    return out


def with_cones(map_yaml, out_dir, radius=CONE_RADIUS, spacing=CONE_SPACING):
    """
    Map yaml for the simulators: `map_yaml` itself without cones, else a copy with them drawn.

    The copy (`<stem>.yaml` + `<stem>_cones.png` in `out_dir`) keeps every other key; the
    reference line paths become absolute so they still resolve, and `cones` is dropped (it is
    already in the image, and f1tenth_gym warns about keys it does not know).
    """
    map_yaml = pathlib.Path(map_yaml)
    with open(map_yaml) as f:
        spec = yaml.safe_load(f)
    cones = cone_positions(spec.pop('cones', None), spacing)
    if not cones:
        return map_yaml
    image = cv2.imread(str(map_yaml.parent / spec['image']), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"{map_yaml}: cannot read image {spec['image']}")
    image = draw_cones(image, cones, spec['resolution'], spec['origin'],
                       int(spec.get('negate', 0)), radius)
    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_dir / f'{map_yaml.stem}_cones.png'), image)
    spec['image'] = f'{map_yaml.stem}_cones.png'
    for key in ('centerline', 'raceline'):
        if spec.get(key) and not pathlib.Path(spec[key]).is_absolute():
            spec[key] = str((map_yaml.parent / spec[key]).resolve())
    out = out_dir / map_yaml.name
    with open(out, 'w') as f:
        yaml.safe_dump(spec, f)
    return out
