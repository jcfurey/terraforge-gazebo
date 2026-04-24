# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Unit tests for terraforge.utils.coordinates.CoordinateConverter.

Guards the UTM hemisphere selection (regression test for the bug where
every origin was projected as UTM North regardless of latitude) and the
WGS84 <-> UTM <-> local-Gazebo round trip.
"""

import pytest

pyproj = pytest.importorskip('pyproj')  # noqa: F841

from terraforge.utils.coordinates import CoordinateConverter  # noqa: E402


def test_northern_hemisphere_berlin():
    c = CoordinateConverter((52.52, 13.40))
    assert c.utm_zone == 33
    assert c.northern_hemisphere is True
    assert c.utm_crs_string == 'EPSG:32633'


def test_southern_hemisphere_sydney():
    # Regression: previously resolved to EPSG:32656 (UTM North) and
    # shifted assets ~10,000 km north of the terrain.
    c = CoordinateConverter((-33.87, 151.21))
    assert c.utm_zone == 56
    assert c.northern_hemisphere is False
    assert c.utm_crs_string == 'EPSG:32756'


def test_southern_equatorial_quito():
    c = CoordinateConverter((-0.2, -78.5))
    assert c.utm_zone == 17
    assert c.northern_hemisphere is False
    assert c.utm_crs_string == 'EPSG:32717'


def test_equator_exact_treated_as_northern():
    # The UTM standard puts a 0-latitude origin in the northern zone
    # (false northing = 0). Match that convention.
    c = CoordinateConverter((0.0, 0.0))
    assert c.utm_crs_string == 'EPSG:32631'


def test_antimeridian_wrap_around():
    # Longitude +180 is zone 1 (wraps by _determine_utm_zone), hemisphere
    # decided by latitude.
    c = CoordinateConverter((40.0, 180.0))
    assert c.utm_zone == 1
    assert c.utm_crs_string == 'EPSG:32601'


def test_origin_is_local_zero():
    c = CoordinateConverter((52.52, 13.40))
    x, y, z = c.wgs84_to_gazebo((52.52, 13.40))
    assert abs(x) < 1e-3
    assert abs(y) < 1e-3
    assert z == 0.0


def test_round_trip_gazebo_wgs84():
    origin = (-33.87, 151.21)
    c = CoordinateConverter(origin)
    # Point ~1 km north + 500 m east of origin
    offset_gz = (500.0, 1000.0)
    lat, lon = c.gazebo_to_wgs84(offset_gz)
    back_x, back_y, _ = c.wgs84_to_gazebo((lat, lon))
    assert abs(back_x - offset_gz[0]) < 1e-3
    assert abs(back_y - offset_gz[1]) < 1e-3
