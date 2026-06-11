# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Regression test: image-based tree scatter must run without OSM foliage.

process_osm_trees_to_sdf() used to return [] as soon as the OSM foliage
geojson was missing, skipping the image-based vegetation fill entirely.
Overpass returns no features for the interior of large tagged polygons
(their nodes fall outside the query bbox), so heavily wooded worlds —
the exact case the vegetation fill exists for — came out with zero trees.
"""
import os

import pytest

np = pytest.importorskip('numpy')
Image = pytest.importorskip('PIL.Image')
pytest.importorskip('shapely')

from terraforge.data_processing import tree_processor  # noqa: E402


def test_vegetation_fill_runs_without_osm_foliage_file(tmp_path):
    # Solid-green satellite texture: every pixel passes the legacy EXG
    # vegetation heuristic, so the scatter grid should place trees.
    texture = tmp_path / 'satellite_texture.png'
    Image.new('RGB', (128, 128), (40, 160, 40)).save(texture)

    missing_geojson = tmp_path / 'does_not_exist_foliage.geojson'
    assert not os.path.exists(missing_geojson)

    placements = tree_processor.process_osm_trees_to_sdf(
        str(missing_geojson),
        str(tmp_path / 'models'),
        (37.7694, -122.4862),
        world_half_extent_m=300.0,
        satellite_texture_path=str(texture),
        vegetation_fill=True,
    )
    assert placements, (
        'image-based vegetation fill must scatter trees even when the '
        'OSM foliage layer is missing/empty'
    )


def test_no_texture_and_no_osm_yields_empty(tmp_path):
    # Without a satellite texture there is nothing to scatter from; the
    # stage should still return cleanly instead of raising.
    placements = tree_processor.process_osm_trees_to_sdf(
        str(tmp_path / 'missing.geojson'),
        str(tmp_path / 'models'),
        (37.7694, -122.4862),
        world_half_extent_m=300.0,
        satellite_texture_path=None,
        vegetation_fill=True,
    )
    assert placements == []
