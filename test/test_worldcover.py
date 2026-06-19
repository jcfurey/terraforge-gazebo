# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Network-free tests for the ESA WorldCover tile-selection helpers.

These pin the 3deg lattice math and tile/URL naming so a regression can't
silently point the downloader at the wrong tile. The raster-reading mask
builder is covered separately in test_worldcover_mask.py (it needs GDAL).
"""

from terraforge.data_acquisition.worldcover import (
    _floor_to_tile,
    _tile_name,
    _tiles_for_bbox,
    _vsicurl_url,
    WORLDCOVER_CLASS_NAMES,
)


def test_tile_name_quadrants():
    assert _tile_name(36, -123) == 'N36W123'
    assert _tile_name(0, 0) == 'N00E000'
    assert _tile_name(-3, 18) == 'S03E018'
    assert _tile_name(51, -3) == 'N51W003'


def test_floor_to_tile_snaps_to_3deg_lattice():
    assert _floor_to_tile(37.77) == 36
    assert _floor_to_tile(-122.42) == -123
    assert _floor_to_tile(0.0) == 0
    assert _floor_to_tile(-0.1) == -3
    assert _floor_to_tile(6.0) == 6


def test_tiles_for_bbox_single_tile():
    # Small San Francisco bbox sits inside one tile.
    assert _tiles_for_bbox(-122.43, 37.76, -122.41, 37.78) == [(36, -123)]


def test_tiles_for_bbox_spans_lattice_line():
    tiles = _tiles_for_bbox(-123.5, 37.0, -122.5, 38.0)
    assert (36, -126) in tiles
    assert (36, -123) in tiles
    assert len(tiles) == 2


def test_tiles_for_bbox_edge_not_overincluded():
    # An east/north edge exactly on a lattice line must not add the next tile.
    assert _tiles_for_bbox(-3.0, 0.0, 0.0, 2.999) == [(0, -3)]


def test_vsicurl_url_shape():
    url = _vsicurl_url('N36W123', 'v200', '2021')
    assert url.startswith('/vsicurl/https://')
    assert url.endswith('ESA_WorldCover_10m_2021_v200_N36W123_Map.tif')


def test_class_names_match_published_legend():
    assert WORLDCOVER_CLASS_NAMES[10] == 'tree_cover'
    assert WORLDCOVER_CLASS_NAMES[50] == 'built_up'
    assert WORLDCOVER_CLASS_NAMES[80] == 'permanent_water_bodies'
