# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""ESA WorldCover 10 m land-cover download + UTM reprojection.

ESA WorldCover is a global, 10 m, 11-class land-cover product derived from
Sentinel-1/2 (CC BY 4.0). It is distributed as 3deg x 3deg Cloud-Optimized
GeoTIFF tiles, named by the SW corner of each tile, e.g.
``ESA_WorldCover_10m_2021_v200_N36W123_Map.tif``.

This module selects the tile(s) covering a WGS84 bbox, reads them straight
from the AWS Open Data bucket with GDAL ``/vsicurl/`` (no whole-tile
download), and warps them onto the same square UTM grid the heightmap and
satellite texture use. Resampling is nearest-neighbour because the pixel
values are categorical class codes, not a continuous field — bilinear would
invent classes that don't exist.

Coordinate contract matches elevation.reproject_dem_to_utm: output is exactly
``width`` x ``height`` pixels spanning (2R x 2R) meters in ``utm_crs``,
centered on the origin, north-up (row 0 = north, col 0 = west).
"""

import math
import os

from pyproj import Transformer

from terraforge.utils.logging import logger
from terraforge.utils.retry import retry_call

# AWS Open Data bucket (eu-central-1), reachable over plain HTTPS — no key.
WORLDCOVER_S3_BASE = 'https://esa-worldcover.s3.eu-central-1.amazonaws.com'
WORLDCOVER_DEFAULT_VERSION = 'v200'   # 2021 map; v100 is the 2020 map
WORLDCOVER_DEFAULT_YEAR = '2021'

# Distribution tiling: 3 degrees on a side, SW corner on a 3 degree lattice.
WORLDCOVER_TILE_DEG = 3

# ESA WorldCover legend (fixed published class codes). Kept here as the
# dataset's home; foliage_mask.py mirrors the handful it consumes so the
# processing layer stays independent of this acquisition module.
WORLDCOVER_CLASS_NAMES = {
    10: 'tree_cover',
    20: 'shrubland',
    30: 'grassland',
    40: 'cropland',
    50: 'built_up',
    60: 'bare_sparse_vegetation',
    70: 'snow_and_ice',
    80: 'permanent_water_bodies',
    90: 'herbaceous_wetland',
    95: 'mangroves',
    100: 'moss_and_lichen',
}


def _gdal():
    """Import GDAL lazily with exceptions on (keeps the import off hot paths).

    Done locally rather than at module import so the tile-selection helpers
    (and their tests) don't drag in GDAL, and so error behaviour doesn't
    depend on whether some other module already called ``UseExceptions``.
    """
    from osgeo import gdal
    gdal.UseExceptions()
    return gdal


def _floor_to_tile(value: float) -> int:
    """Floor ``value`` (degrees) down to the WorldCover 3 degree lattice."""
    return int(math.floor(value / WORLDCOVER_TILE_DEG) * WORLDCOVER_TILE_DEG)


def _tile_name(lat_sw: int, lon_sw: int) -> str:
    """Return the WorldCover tile name for an SW corner, e.g. ``N36W123``.

    Latitude is zero-padded to 2 digits, longitude to 3, each prefixed with
    the hemisphere letter — the exact convention ESA uses in the filenames.
    """
    ns = 'N' if lat_sw >= 0 else 'S'
    ew = 'E' if lon_sw >= 0 else 'W'
    return f'{ns}{abs(lat_sw):02d}{ew}{abs(lon_sw):03d}'


def _tiles_for_bbox(west: float, south: float, east: float,
                    north: float) -> list:
    """Return the (lat_sw, lon_sw) corners of every tile the bbox touches.

    The upper edges are nudged inward by an epsilon so a bbox whose east/north
    edge lands exactly on a lattice line doesn't pull in the next (untouched)
    tile.
    """
    eps = 1e-9
    lon_start = _floor_to_tile(west)
    lon_end = _floor_to_tile(east - eps)
    lat_start = _floor_to_tile(south)
    lat_end = _floor_to_tile(north - eps)
    tiles = []
    lat = lat_start
    while lat <= lat_end:
        lon = lon_start
        while lon <= lon_end:
            tiles.append((lat, lon))
            lon += WORLDCOVER_TILE_DEG
        lat += WORLDCOVER_TILE_DEG
    return tiles


def _vsicurl_url(tile: str, version: str, year: str) -> str:
    """Build the GDAL ``/vsicurl/`` URL for a WorldCover map tile."""
    fname = f'ESA_WorldCover_10m_{year}_{version}_{tile}_Map.tif'
    return f'/vsicurl/{WORLDCOVER_S3_BASE}/{version}/{year}/map/{fname}'


def _bbox_wgs84_from_utm(location: tuple, radius_meters: float,
                         utm_crs: str) -> tuple:
    """Derive the WGS84 (west, south, east, north) of the UTM (2R x 2R) box."""
    lat, lon = location
    to_utm = Transformer.from_crs('EPSG:4326', utm_crs, always_xy=True)
    to_wgs = Transformer.from_crs(utm_crs, 'EPSG:4326', always_xy=True)
    cx, cy = to_utm.transform(lon, lat)
    west, south = to_wgs.transform(cx - radius_meters, cy - radius_meters)
    east, north = to_wgs.transform(cx + radius_meters, cy + radius_meters)
    return (west, south, east, north)


def _cached_raster_ok(path: str, width: int, height: int) -> bool:
    """Return True if ``path`` is a readable raster of the expected size."""
    gdal = _gdal()
    if not path or not os.path.exists(path):
        return False
    try:
        ds = gdal.Open(path)
        if ds is None:
            return False
        ok = ds.RasterXSize == width and ds.RasterYSize == height
        ds = None
        return ok
    except RuntimeError:
        return False


def _existing_sources(sources: list) -> list:
    """Drop tiles that don't resolve (ocean gaps, outside coverage).

    WorldCover ships no tile for all-ocean cells and stops at roughly 60S, so
    a multi-tile bbox near a coast or the coverage edge can reference a tile
    that 404s. Probing here lets a partially-covered world still build from
    the tiles that do exist instead of the whole warp failing.
    """
    gdal = _gdal()
    keep = []
    for src in sources:
        try:
            ds = gdal.Open(src)
            if ds is not None:
                keep.append(src)
                ds = None
        except RuntimeError:
            logger.warning(f'WorldCover tile unavailable, skipping: {src}')
    return keep


def download_worldcover_utm(
    location: tuple,
    radius_meters: float,
    *,
    utm_crs: str,
    width: int,
    height: int,
    output_path: str,
    bbox_wgs84: tuple = None,
    version: str = WORLDCOVER_DEFAULT_VERSION,
    year: str = WORLDCOVER_DEFAULT_YEAR,
) -> str:
    """Warp the WorldCover tiles covering the bbox into a UTM class raster.

    Args:
        location: (latitude, longitude) origin in WGS84.
        radius_meters: Half-width of the output box in true meters.
        utm_crs: Target UTM EPSG string (e.g. ``EPSG:32610``); must match the
            converter used everywhere else in the pipeline.
        width, height: Output raster size in pixels (square grid).
        output_path: Destination GeoTIFF path (also the cache key).
        bbox_wgs84: Optional precomputed (west, south, east, north); derived
            from the UTM box when omitted.
        version, year: WorldCover product version/year (default v200 / 2021).

    Returns:
        ``output_path``.
    """
    gdal = _gdal()
    if _cached_raster_ok(output_path, width, height):
        logger.info(f'Using cached WorldCover raster: {output_path}')
        return output_path

    lat, lon = location
    if bbox_wgs84 is None:
        bbox_wgs84 = _bbox_wgs84_from_utm(location, radius_meters, utm_crs)
    west, south, east, north = bbox_wgs84

    tiles = _tiles_for_bbox(west, south, east, north)
    if not tiles:
        raise RuntimeError(f'No ESA WorldCover tiles cover bbox {bbox_wgs84}')
    sources = [_vsicurl_url(_tile_name(la, lo), version, year)
               for (la, lo) in tiles]
    logger.info(
        f'WorldCover {version}/{year}: {len(sources)} tile(s) cover bbox; '
        f'warping to {width}x{height} {utm_crs}'
    )

    to_utm = Transformer.from_crs('EPSG:4326', utm_crs, always_xy=True)
    cx, cy = to_utm.transform(lon, lat)
    output_bounds = (cx - radius_meters, cy - radius_meters,
                     cx + radius_meters, cy + radius_meters)

    def _warp():
        srcs = sources
        if len(srcs) > 1:
            srcs = _existing_sources(srcs)
            if not srcs:
                raise RuntimeError(
                    'None of the candidate WorldCover tiles are reachable'
                )
        # srcNodata/dstNodata=0: WorldCover uses 0 for no-data/ocean, and 0 is
        # not a valid class, so it stays out of both the positive and the
        # negative class sets downstream.
        result = gdal.Warp(
            destNameOrDestDS=output_path,
            srcDSOrSrcDSTab=srcs,
            dstSRS=utm_crs,
            outputBounds=output_bounds,
            width=width,
            height=height,
            resampleAlg='near',
            format='GTiff',
            srcNodata=0,
            dstNodata=0,
            multithread=True,
        )
        if result is None:
            raise RuntimeError('gdal.Warp returned None for WorldCover')
        result = None

    retry_call(_warp, attempts=3, initial_delay=2.0, label='WorldCover download')
    logger.info(f'WorldCover raster written to {output_path}')
    return output_path
