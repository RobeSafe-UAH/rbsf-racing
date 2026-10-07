import math

import cv2
import numpy as np
import pytest
from rbsf_worlds.cones import cone_positions, draw_cones, load_cones, with_cones
from rbsf_worlds.tracks import find_waypoints, track_yaml
import yaml


def test_single_cones_and_rows():
    assert cone_positions([[1.0, 2.0]]) == [(1.0, 2.0)]
    row = cone_positions([[0.0, 0.0, 1.0, 0.0]], spacing=0.25)
    assert len(row) == 5 and row[0] == (0.0, 0.0) and row[-1] == pytest.approx((1.0, 0.0))
    gaps = [math.dist(a, b) for a, b in zip(row, row[1:])]
    assert max(gaps) <= 0.25 + 1e-9
    assert cone_positions(None) == []
    with pytest.raises(ValueError):
        cone_positions([[1.0, 2.0, 3.0]])


def test_cones_are_drawn_where_they_are():
    image = np.full((100, 200), 255, np.uint8)  # 10 x 5 m at 5 cm, origin (-1, -1)
    out = draw_cones(image, [(2.0, 1.0)], 0.05, [-1.0, -1.0, 0.0], radius=0.1)
    col, row = round(3.0 / 0.05), 99 - round(2.0 / 0.05)
    assert out[row, col] == 0 and out[row, col + 3] == 255
    assert image.min() == 255  # input untouched
    assert draw_cones(image, [(2.0, 1.0)], 0.05, [-1.0, -1.0, 0.0], negate=1)[row, col] == 255


def test_map_without_cones_is_used_as_is(tmp_path):
    spielberg = track_yaml('spielberg')
    assert load_cones(spielberg) == []
    assert with_cones(spielberg, tmp_path) == spielberg
    assert not any(tmp_path.iterdir())


def test_levine_copy_closes_its_openings(tmp_path):
    levine = track_yaml('levine')
    assert len(load_cones(levine)) > 20
    copy = with_cones(levine, tmp_path)
    spec = yaml.safe_load(copy.read_text())
    assert 'cones' not in spec and copy.parent == tmp_path
    # The reference line still resolves from the copy.
    assert find_waypoints(copy) == (levine.parent / 'levine_centerline.csv').resolve()
    image = cv2.imread(str(tmp_path / spec['image']), cv2.IMREAD_GRAYSCALE)
    res, (ox, oy, _) = spec['resolution'], spec['origin']

    def pixel(x, y):
        return image[image.shape[0] - 1 - round((y - oy) / res), round((x - ox) / res)]
    # Across the annex with the open corridor, and in the original image: free there.
    original = cv2.imread(str(levine.parent / 'levine.png'), cv2.IMREAD_GRAYSCALE)
    for x, y in ((10.6, 0.5), (9.75, -1.05), (-13.6, 9.5), (-13.6, -1.0)):
        assert pixel(x, y) == 0
        assert original[image.shape[0] - 1 - round((y - oy) / res), round((x - ox) / res)] > 200
