# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Tile-compound flattening tests.

Buildings and roads now contribute a ``body_sdf`` fragment that gets
packed into a single shared `<link>` per tile (huge load-time win on
dense worlds — one link per tile instead of one link per building).
Cartoon trees stay as own-link items because their per-instance yaw
is expressed via the link's <pose>. Verify both kinds coexist in one
valid compound model.
"""

import pytest

pytest.importorskip('shapely')

from terraforge.data_processing.sdf_builder import build_scene_tiles  # noqa: E402


def _building_placement(name, xy, z=0.0):
    x, y = xy
    return {
        'model_name': name,
        'link_name': name,
        'pose_xy': (x, y),
        'pose_z': z,
        # Minimal synthetic body with a recognisable marker per building
        # so we can assert the packed link contains all of them.
        'body_sdf': (
            f"      <visual name='vis_{name}'>\n"
            f'        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>\n'
            f'        <geometry><box><size>1 1 1</size></box></geometry>\n'
            f'      </visual>'
        ),
    }


def _tree_placement(name, xy):
    x, y = xy
    return {
        'model_name': name,
        'link_name': name,
        'pose_xy': (x, y),
        'pose_z': 0.0,
        # Trees keep their per-instance yaw via the link's <pose>, so
        # they contribute a full <link> rather than a body fragment.
        'link_sdf': (
            f"    <link name='{name}'>\n"
            f'      <pose>{x:.3f} {y:.3f} 0 0 0 1.234</pose>\n'
            f"      <visual name='trunk'>\n"
            f'        <geometry><cylinder>'
            f'<radius>0.2</radius><length>3</length></cylinder></geometry>\n'
            f'      </visual>\n'
            f'    </link>'
        ),
    }


def test_bodies_packed_into_single_shared_link():
    # Three buildings, all within a single 200 m tile at the origin.
    buildings = [_building_placement(f'b{i}', (i * 10.0, 0.0)) for i in range(3)]
    records = build_scene_tiles(buildings, trees=None, roads=None,
                                half_extent_m=500.0, tile_size_m=200.0)
    assert len(records) == 1
    sdf = records[0]['model_sdf']
    # Exactly one <link> for the tile — the shared bodies link.
    assert sdf.count('<link name=') == 1
    # Every building's visual is present.
    for i in range(3):
        assert f'vis_b{i}' in sdf
    # No joints (single-link model is trivially valid).
    assert '<joint' not in sdf


def test_cartoon_trees_get_own_links_jointed_to_bodies_link():
    buildings = [_building_placement('b0', (0.0, 0.0))]
    trees = [_tree_placement(f't{i}', (i * 5.0, 10.0)) for i in range(2)]
    records = build_scene_tiles(buildings, trees=trees, roads=None,
                                half_extent_m=500.0, tile_size_m=200.0)
    sdf = records[0]['model_sdf']
    # 1 bodies link + 2 tree links.
    assert sdf.count('<link name=') == 3
    # 2 fixed joints tying each tree link to the shared bodies link.
    assert sdf.count("type='fixed'") == 2
    assert sdf.count('<parent>tile_2_2_bodies</parent>') == 2


def test_tree_only_tile_emits_placeholder_bodies_link():
    trees = [_tree_placement('t0', (0.0, 0.0))]
    records = build_scene_tiles(buildings=None, trees=trees, roads=None,
                                half_extent_m=500.0, tile_size_m=200.0)
    sdf = records[0]['model_sdf']
    # Bodies link is emitted even empty so the tree can joint onto it.
    # Originally added for bullet-featherstone's strict single-tree
    # validation; retained under plain bullet because it keeps the
    # tile-link layout consistent.
    assert "_bodies'></link>" in sdf or "_bodies'>\n" in sdf
    assert sdf.count("type='fixed'") == 1


def test_empty_inputs_produce_no_tile_records():
    records = build_scene_tiles(buildings=None, trees=None, roads=None,
                                half_extent_m=500.0, tile_size_m=200.0)
    assert records == []
