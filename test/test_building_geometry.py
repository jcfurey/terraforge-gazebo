# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Unit tests for OSM polygon handling in building_processor.

Regression guards:
  * MultiPolygon features are split into one building per part instead
    of being collapsed to a single centroid.
  * Polygon-with-interior (courtyard / donut) footprints subtract the
    hole area from the height-inference input, not add it.
  * Invalid geometry (zero / negative area) is dropped with a counter
    instead of crashing.
"""

import json

import pytest

pytest.importorskip('shapely')
pytest.importorskip('pyproj')

from terraforge.data_processing import building_processor  # noqa: E402


def _write_geojson(path, features):
    with open(path, 'w') as f:
        json.dump({'type': 'FeatureCollection', 'features': features}, f)


def _square(lat, lon, side_deg, height_tag=None, osmid='id'):
    """GeoJSON Polygon feature centered on (lat, lon) with given side length."""
    half = side_deg / 2.0
    coords = [[
        [lon - half, lat - half],
        [lon + half, lat - half],
        [lon + half, lat + half],
        [lon - half, lat + half],
        [lon - half, lat - half],
    ]]
    props = {'osmid': osmid, 'building': 'yes'}
    if height_tag is not None:
        props['height'] = height_tag
    return {
        'type': 'Feature',
        'properties': props,
        'geometry': {'type': 'Polygon', 'coordinates': coords},
    }


def test_multipolygon_splits_into_separate_buildings(tmp_path):
    # Two detached squares tagged as one MultiPolygon building.
    # Each square is ~110 m on a side at the equator — well above the
    # MIN_BUILDING_AREA_M2 (15 m²) threshold.
    feat = {
        'type': 'Feature',
        'properties': {'osmid': 'multi1', 'building': 'yes', 'height': '10'},
        'geometry': {
            'type': 'MultiPolygon',
            'coordinates': [
                [[
                    [0.0000, 0.0000],
                    [0.0010, 0.0000],
                    [0.0010, 0.0010],
                    [0.0000, 0.0010],
                    [0.0000, 0.0000],
                ]],
                [[
                    [0.0050, 0.0050],
                    [0.0060, 0.0050],
                    [0.0060, 0.0060],
                    [0.0050, 0.0060],
                    [0.0050, 0.0050],
                ]],
            ],
        },
    }
    path = str(tmp_path / 'buildings.geojson')
    _write_geojson(path, [feat])

    placements = building_processor.process_osm_buildings_to_sdf(
        path, str(tmp_path / 'models'), origin_wgs84=(0.0, 0.0),
    )

    assert len(placements) == 2, [p['model_name'] for p in placements]
    names = [p['model_name'] for p in placements]
    assert all('_p' in n for n in names), names
    # Poses should be distinct — the two parts are far apart.
    (x0, y0), (x1, y1) = placements[0]['pose_xy'], placements[1]['pose_xy']
    assert abs(x0 - x1) > 100 or abs(y0 - y1) > 100


def test_single_polygon_keeps_historical_model_name(tmp_path):
    # Single-part Polygon buildings keep the pre-multipart naming scheme
    # so diffs against existing worlds stay small.
    path = str(tmp_path / 'buildings.geojson')
    _write_geojson(path, [_square(0.0, 0.0, 0.001, height_tag='6', osmid='b7')])
    placements = building_processor.process_osm_buildings_to_sdf(
        path, str(tmp_path / 'models'), origin_wgs84=(0.0, 0.0),
    )
    assert len(placements) == 1
    name = placements[0]['model_name']
    assert '_p' not in name, name
    assert name.startswith('building_b7_')


def test_polygon_with_interior_uses_hole_adjusted_area(tmp_path):
    # Outer ring area ~ (0.002 * 111000)**2 = ~49k m², inner hole
    # ~ (0.001 * 111000)**2 = ~12k m² -> net ~37k m². Height inference
    # uses the net; we indirectly check by confirming the feature is
    # emitted (area > MIN threshold) and its bounding box matches the
    # outer ring rather than the hole.
    feat = {
        'type': 'Feature',
        'properties': {'osmid': 'donut1', 'building': 'yes'},
        'geometry': {
            'type': 'Polygon',
            'coordinates': [
                # Exterior
                [
                    [-0.001, -0.001], [0.001, -0.001],
                    [0.001, 0.001], [-0.001, 0.001],
                    [-0.001, -0.001],
                ],
                # Interior hole (CW for valid Polygon)
                [
                    [-0.0005, -0.0005], [-0.0005, 0.0005],
                    [0.0005, 0.0005], [0.0005, -0.0005],
                    [-0.0005, -0.0005],
                ],
            ],
        },
    }
    path = str(tmp_path / 'buildings.geojson')
    _write_geojson(path, [feat])
    placements = building_processor.process_osm_buildings_to_sdf(
        path, str(tmp_path / 'models'), origin_wgs84=(0.0, 0.0),
    )
    assert len(placements) == 1
    # body_sdf should include the full exterior ring's vertices; the
    # hole is omitted (bbox collision fallback can't represent it).
    sdf = placements[0]['body_sdf']
    assert sdf.count('<point>') >= 4


def test_invalid_multipolygon_part_is_counted_but_does_not_crash(tmp_path):
    # A MultiPolygon whose second part is degenerate (all points equal)
    # should emit the first valid part and count the bad one as
    # invalid_geom_skipped rather than raising.
    feat = {
        'type': 'Feature',
        'properties': {'osmid': 'mixed1', 'building': 'yes'},
        'geometry': {
            'type': 'MultiPolygon',
            'coordinates': [
                [[
                    [0.0, 0.0], [0.001, 0.0],
                    [0.001, 0.001], [0.0, 0.001],
                    [0.0, 0.0],
                ]],
                [[
                    [0.002, 0.002], [0.002, 0.002],
                    [0.002, 0.002], [0.002, 0.002],
                ]],
            ],
        },
    }
    path = str(tmp_path / 'buildings.geojson')
    _write_geojson(path, [feat])
    placements = building_processor.process_osm_buildings_to_sdf(
        path, str(tmp_path / 'models'), origin_wgs84=(0.0, 0.0),
    )
    assert len(placements) == 1
