# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Tests for draped, collidable road-mesh generation.

Roads must follow the DEM (not sit on flat averaged slabs) and carry both a
mesh collision with friction and a mesh visual, so a rover can drive on them
over the terrain it collides with.
"""

import json
import os

import pytest

pytest.importorskip('shapely')
pytest.importorskip('pyproj')

from terraforge.data_processing import road_processor  # noqa: E402


def _write_geojson(path, features):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({'type': 'FeatureCollection', 'features': features}, f)


def _road_feature():
    # ~222 m straight E-W road across the origin (0.002 deg lon at equator).
    return {
        'type': 'Feature',
        'properties': {'osmid': 'r1', 'highway': 'residential'},
        'geometry': {
            'type': 'LineString',
            'coordinates': [[-0.001, 0.0], [0.001, 0.0]],
        },
    }


def _all_obj_z(meshes_dir):
    zs = []
    for name in os.listdir(meshes_dir):
        if not name.endswith('.obj'):
            continue
        with open(os.path.join(meshes_dir, name), encoding='utf-8') as fh:
            for line in fh:
                if line.startswith('v '):
                    zs.append(float(line.split()[3]))
    return zs


def test_road_emits_mesh_collision_with_friction(tmp_path):
    path = str(tmp_path / 'roads.geojson')
    _write_geojson(path, [_road_feature()])
    models_dir = str(tmp_path / 'models')

    placements = road_processor.process_osm_roads_to_sdf(
        path, models_dir, origin_wgs84=(0.0, 0.0),
        elevation_sampler=lambda x, y: 0.0,
    )
    assert placements
    sdf = placements[0]['body_sdf']
    assert '<mesh>' in sdf
    assert '<collision' in sdf and '<mu>' in sdf      # collidable + friction
    assert '<visual' in sdf
    objs = [f for f in os.listdir(os.path.join(models_dir, 'road_meshes'))
            if f.endswith('.obj')]
    assert objs


def test_road_drapes_on_terrain(tmp_path):
    path = str(tmp_path / 'roads.geojson')
    _write_geojson(path, [_road_feature()])
    models_dir = str(tmp_path / 'models')

    # Terrain rises 0.1 m per metre of gazebo +x: a ~222 m E-W road should
    # span ~22 m of elevation if it really drapes (the old flat-slab path
    # would have a near-constant z per segment).
    road_processor.process_osm_roads_to_sdf(
        path, models_dir, origin_wgs84=(0.0, 0.0),
        elevation_sampler=lambda x, y: 0.1 * x,
    )
    zs = _all_obj_z(os.path.join(models_dir, 'road_meshes'))
    assert zs
    assert max(zs) - min(zs) > 5.0, (min(zs), max(zs))


def test_road_without_sampler_is_flat(tmp_path):
    path = str(tmp_path / 'roads.geojson')
    _write_geojson(path, [_road_feature()])
    models_dir = str(tmp_path / 'models')

    placements = road_processor.process_osm_roads_to_sdf(
        path, models_dir, origin_wgs84=(0.0, 0.0),
    )
    assert placements
    zs = _all_obj_z(os.path.join(models_dir, 'road_meshes'))
    # Flat: only the top plane and the bottom (top - thickness) appear.
    assert max(zs) - min(zs) < 0.2


def test_missing_roads_file_returns_empty(tmp_path):
    placements = road_processor.process_osm_roads_to_sdf(
        str(tmp_path / 'nope.geojson'), str(tmp_path / 'models'), (0.0, 0.0),
    )
    assert placements == []
