# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Unit tests for elevation_processor.open_dem_sampler.

The sampler is called thousands of times per world (once per building
corner, tree, road midpoint). We verify that it reads the DEM once and
returns correct elevations for interior / edge / out-of-bounds queries.
"""

import os

import pytest

pytest.importorskip('osgeo')
pytest.importorskip('numpy')

import numpy as np  # noqa: E402,I100,I202
from osgeo import gdal, osr  # noqa: E402

from terraforge.data_processing.elevation_processor import (  # noqa: E402
    open_dem_sampler,
    sample_dem_elevation_utm,
)


def _write_tiny_dem(path, width=32, height=32, origin_x=500000.0, origin_y=4500000.0,
                    pixel_size=10.0, nodata=-9999.0):
    """Write a 32x32 UTM GeoTIFF where elevation = utm_x / 100 + utm_y / 100.

    Includes a single nodata pixel at (0, 0) to exercise the fallback.
    """
    drv = gdal.GetDriverByName('GTiff')
    ds = drv.Create(path, width, height, 1, gdal.GDT_Float32)
    # Geotransform: origin is top-left, pixel_h is negative (rows go south).
    ds.SetGeoTransform([origin_x, pixel_size, 0, origin_y, 0, -pixel_size])
    srs = osr.SpatialReference()
    srs.ImportFromEPSG(32633)  # some UTM zone — value doesn't matter for sampling
    ds.SetProjection(srs.ExportToWkt())
    arr = np.zeros((height, width), dtype=np.float32)
    for py in range(height):
        for px in range(width):
            utm_x = origin_x + (px + 0.5) * pixel_size
            utm_y = origin_y + (py + 0.5) * -pixel_size
            arr[py, px] = utm_x / 100.0 + utm_y / 100.0
    arr[0, 0] = nodata
    band = ds.GetRasterBand(1)
    band.WriteArray(arr)
    band.SetNoDataValue(nodata)
    ds.FlushCache()
    ds = None


def test_sampler_matches_per_query_impl(tmp_path):
    path = str(tmp_path / 'dem.tif')
    _write_tiny_dem(path)
    sampler = open_dem_sampler(path, nodata_fallback=0.0)
    # Pick a handful of interior points and compare cached vs uncached.
    for utm_x, utm_y in [(500050.0, 4499950.0), (500100.0, 4499900.0), (500310.0, 4499700.0)]:
        assert abs(sampler(utm_x, utm_y) - sample_dem_elevation_utm(path, utm_x, utm_y)) < 1e-3


def test_sampler_clamps_out_of_bounds(tmp_path):
    path = str(tmp_path / 'dem.tif')
    _write_tiny_dem(path)
    sampler = open_dem_sampler(path, nodata_fallback=0.0)
    # 10 km east of the DEM — clamp to east edge column.
    far_east = sampler(500000.0 + 10000.0, 4499950.0)
    edge_east = sampler(500000.0 + 32 * 10 - 5.0, 4499950.0)
    assert abs(far_east - edge_east) < 1e-3


def test_sampler_nodata_fallback(tmp_path):
    path = str(tmp_path / 'dem.tif')
    _write_tiny_dem(path, nodata=-9999.0)
    sampler = open_dem_sampler(path, nodata_fallback=42.0)
    # NE corner of the DEM (origin pixel) is the nodata cell.
    # Origin is top-left at (500000, 4500000), pixel 0,0 covers the top-left 10 m.
    assert sampler(500005.0, 4499995.0) == 42.0


def test_sampler_does_not_reopen_dataset(tmp_path):
    # Light sanity: once the sampler is built, we can delete the file and
    # sampling still works (numpy already holds the band).
    path = str(tmp_path / 'dem.tif')
    _write_tiny_dem(path)
    sampler = open_dem_sampler(path, nodata_fallback=0.0)
    os.unlink(path)
    assert sampler(500050.0, 4499950.0) > 0
