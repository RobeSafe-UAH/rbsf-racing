import numpy as np
import pytest
from rbsf_mvsim.bridges import (
    BASE_HEIGHT, bridge_elements, CONE_HEIGHT, load_bridges, profile, RAIL_HEIGHT, surface)
from rbsf_worlds.tracks import track_yaml
import yaml

BRIDGE = {'start': [0.0, 2.0], 'end': [10.0, 2.0], 'deck': [4.0, 6.0], 'width': 1.0,
          'height': 0.8}


def test_profile_ramps_up_holds_the_deck_and_ramps_down():
    s = np.array([-1.0, 0.0, 2.0, 4.0, 5.0, 6.0, 8.0, 10.0, 11.0])
    z = profile(s, 10.0, 4.0, 6.0, 0.8)
    assert z == pytest.approx([0.0, 0.0, 0.4, 0.8, 0.8, 0.8, 0.4, 0.0, 0.0])


@pytest.mark.parametrize('flip', [False, True])
def test_surface_spans_the_road_exactly(flip):
    bridge = dict(BRIDGE)
    if flip:  # same bridge, declared from the other end
        bridge.update(start=BRIDGE['end'], end=BRIDGE['start'], deck=[4.0, 6.0])
    corner_x, corner_y, z = surface(bridge, resolution=0.1)
    assert (corner_x, corner_y) == pytest.approx((0.0, 1.5))
    assert z.shape == (101, 11)  # rows along x, columns along y (MVSim's order)
    assert z[0] == pytest.approx(0.0) and z[-1] == pytest.approx(0.0)
    assert z[50] == pytest.approx(0.8)
    assert z[20, 0] == pytest.approx(0.4) and z[80, 10] == pytest.approx(0.4)


def test_surface_of_a_bridge_along_y_is_transposed():
    bridge = dict(BRIDGE, start=[2.0, 0.0], end=[2.0, 10.0])
    corner_x, corner_y, z = surface(bridge, resolution=0.1)
    assert (corner_x, corner_y) == pytest.approx((1.5, 0.0))
    assert z.shape == (11, 101)
    assert z[:, 50] == pytest.approx(0.8)


def test_elements_are_the_road_surface_and_the_deck_rails():
    elements = bridge_elements([BRIDGE])
    assert elements[0].get('class') == 'elevation_map'
    rails = elements[1:]
    assert len(rails) == 2  # a 2 m deck: one piece per side
    for rail in rails:
        assert rail.find('intangible').text == 'true'
        assert rail.find('skip_elevation_adjust') is not None
        x, y, z = (float(v) for v in rail.find('init_pose3d').text.split()[:3])
        assert x == pytest.approx(4.0) and z == pytest.approx(0.8)
        assert float(rail.find('zmax').text) == pytest.approx(RAIL_HEIGHT)
    assert sorted(float(r.find('init_pose3d').text.split()[1]) for r in rails) == \
        pytest.approx([1.45, 2.5])  # right rail outside y = 1.5, left rail from y = 2.5


def test_load_rejects_skewed_axes_and_decks_off_the_bridge(tmp_path):
    def load(**changes):
        path = tmp_path / 'map.yaml'
        path.write_text(yaml.safe_dump({'bridges': [dict(BRIDGE, **changes)]}))
        return load_bridges(path)

    assert len(load()) == 1
    with pytest.raises(ValueError, match='parallel'):
        load(end=[10.0, 3.0])
    with pytest.raises(ValueError, match='inside the ramps'):
        load(deck=[0.0, 6.0])
    with pytest.raises(ValueError, match='cone row'):
        load(under_cones=[[1.0, 2.0]])
    with pytest.raises(ValueError, match='lacks'):
        path = tmp_path / 'map.yaml'
        path.write_text(yaml.safe_dump({'bridges': [{'start': [0, 0]}]}))
        load_bridges(path)


def test_icra2026_master_has_its_bridge():
    (bridge,) = load_bridges(track_yaml('icra2026_master'))
    assert 0.6 <= bridge['height'] <= 0.8
    assert load_bridges(track_yaml('levine')) == []


def test_cones_under_the_deck_stand_on_the_ground_on_a_continuous_base():
    bridge = dict(BRIDGE, under_cones=[[4.5, 1.6, 4.5, 2.4]])
    blocks = bridge_elements([bridge])[3:]  # after the surface and the two rails
    kinds = [b.get('name').rsplit('_', 1)[1].rstrip('0123456789') for b in blocks]
    bases = [b for b, kind in zip(blocks, kinds) if kind == 'base']
    cones = [b for b, kind in zip(blocks, kinds) if kind == 'cone']
    assert len(bases) == 1 and len(cones) == 5  # 0.8 m row, cones every <= 0.25 m
    for block in blocks:
        assert block.find('intangible').text == 'true'
        assert float(block.find('init_pose3d').text.split()[2]) == 0.0  # on the ground
    # The base is what a car at ground level hits (lower than its chassis), the cones are not.
    assert float(bases[0].find('zmax').text) == pytest.approx(BASE_HEIGHT)
    assert float(cones[0].find('zmax').text) == pytest.approx(CONE_HEIGHT)
    assert BASE_HEIGHT < RAIL_HEIGHT < CONE_HEIGHT < bridge['height']
