# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""End-to-end tests for the worldcover foliage-mask builder.

Writes a tiny synthetic WorldCover class raster and checks that tree-cover
becomes placeable while built-up / water are carved out — exercising
build_worldcover_mask without any network access. Needs GDAL (osgeo).
"""

import os

import pytest

pytest.importorskip('osgeo')
pytest.importorskip('numpy')
pytest.importorskip('pyproj')
pytest.importorskip('shapely')
pytest.importorskip('PIL')

import numpy as np  # noqa: E402,I100,I202
from osgeo import gdal, osr  # noqa: E402

from terraforge.data_acquisition import worldcover  # noqa: E402
from terraforge.data_processing.foliage_mask import (  # noqa: E402
    build_worldcover_mask,
)
from terraforge.utils.coordinates import CoordinateConverter  # noqa: E402

gdal.UseExceptions()


def _write_worldcover(path, classes):
    """Write a single-band Byte GeoTIFF of WorldCover class codes."""
    h, w = classes.shape
    drv = gdal.GetDriverByName('GTiff')
    ds = drv.Create(path, w, h, 1, gdal.GDT_Byte)
    # Geotransform is irrelevant to the reader (it ReadsAsArray straight) but
    # set a sane north-up one so GDAL doesn't warn.
    ds.SetGeoTransform([500000.0, 10.0, 0, 4500000.0, 0, -10.0])
    band = ds.GetRasterBand(1)
    band.WriteArray(classes)
    band.SetNoDataValue(0)
    ds.FlushCache()
    ds = None


def test_worldcover_mask_tree_positive_builtup_water_negative(tmp_path):
    classes = np.full((8, 8), 30, dtype=np.uint8)  # grassland baseline
    classes[0:4, :] = 10   # tree cover -> positive
    classes[4, :] = 50     # built-up -> negative
    classes[5, :] = 80     # water -> negative
    path = str(tmp_path / 'wc.tif')
    _write_worldcover(path, classes)

    conv = CoordinateConverter((37.0, -122.0))
    bbox = (-122.01, 36.99, -121.99, 37.01)
    mask = build_worldcover_mask(
        path, bbox, world_half_extent_m=100.0, converter=conv,
    )
    assert mask is not None
    arr = mask.sample_grid(8, 8, 100.0)
    assert arr.shape == (8, 8)
    assert arr[0:4, :].all()       # tree rows are placeable
    assert not arr[4, :].any()     # built-up excluded
    assert not arr[5, :].any()     # water excluded
    assert not arr[6:8, :].any()   # grassland is not a positive class


def test_worldcover_positive_class_override(tmp_path):
    classes = np.full((4, 4), 30, dtype=np.uint8)  # all grassland
    path = str(tmp_path / 'wc2.tif')
    _write_worldcover(path, classes)
    conv = CoordinateConverter((37.0, -122.0))
    bbox = (-122.01, 36.99, -121.99, 37.01)

    # Grassland is not a default positive -> nothing placeable.
    default_mask = build_worldcover_mask(
        path, bbox, world_half_extent_m=50.0, converter=conv,
    )
    assert default_mask is not None
    assert default_mask.foliage_fraction == 0.0

    # Opt grassland in as positive -> the whole tile becomes placeable.
    grass_mask = build_worldcover_mask(
        path, bbox, world_half_extent_m=50.0, converter=conv,
        positive_classes=(30,),
    )
    assert grass_mask is not None
    assert grass_mask.foliage_fraction == 1.0


def test_worldcover_mask_missing_file_returns_none(tmp_path):
    conv = CoordinateConverter((37.0, -122.0))
    bbox = (-122.01, 36.99, -121.99, 37.01)
    mask = build_worldcover_mask(
        str(tmp_path / 'does_not_exist.tif'), bbox,
        world_half_extent_m=50.0, converter=conv,
    )
    assert mask is None


def _write_wgs84_tile(path, west, south, east, north, code, size=64):
    """Write a synthetic EPSG:4326 WorldCover-like tile filled with ``code``."""
    drv = gdal.GetDriverByName('GTiff')
    ds = drv.Create(path, size, size, 1, gdal.GDT_Byte)
    px_w = (east - west) / size
    px_h = (north - south) / size
    ds.SetGeoTransform([west, px_w, 0, north, 0, -px_h])
    srs = osr.SpatialReference()
    srs.ImportFromEPSG(4326)
    ds.SetProjection(srs.ExportToWkt())
    band = ds.GetRasterBand(1)
    band.WriteArray(np.full((size, size), code, dtype=np.uint8))
    band.SetNoDataValue(0)
    ds.FlushCache()
    ds = None


def test_download_worldcover_utm_warps_local_tile(tmp_path, monkeypatch):
    # Stand a local EPSG:4326 "tile" (all tree cover) in for the S3 fetch so
    # the gdal.Warp -> UTM grid path is exercised end-to-end offline.
    origin = (37.75, -122.25)
    conv = CoordinateConverter(origin)
    tile_path = str(tmp_path / 'src_4326.tif')
    _write_wgs84_tile(tile_path, -122.5, 37.5, -122.0, 38.0, code=10)

    monkeypatch.setattr(worldcover, '_vsicurl_url',
                        lambda tile, version, year: tile_path)

    out_path = str(tmp_path / 'wc_utm.tif')
    worldcover.download_worldcover_utm(
        origin, 200.0,
        utm_crs=conv.utm_crs_string,
        width=32, height=32,
        output_path=out_path,
    )

    ds = gdal.Open(out_path)
    assert ds is not None
    assert (ds.RasterXSize, ds.RasterYSize) == (32, 32)
    arr = ds.GetRasterBand(1).ReadAsArray()
    ds = None
    # The all-tree source must reproject to tree cover over the UTM box.
    assert (arr == 10).all()

    # Second call hits the size-matched cache and leaves the file untouched.
    mtime = os.path.getmtime(out_path)
    worldcover.download_worldcover_utm(
        origin, 200.0,
        utm_crs=conv.utm_crs_string,
        width=32, height=32,
        output_path=out_path,
    )
    assert os.path.getmtime(out_path) == mtime
