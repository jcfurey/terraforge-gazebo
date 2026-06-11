# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""DEM download + WGS84 bbox math.

Coordinate-system contract (must stay consistent across the pipeline):

  * Input location:       WGS84 lat/lon (EPSG:4326)
  * Bbox we compute:      WGS84 lat/lon (EPSG:4326) rectangle whose **UTM**
                          projection is exactly (2*radius x 2*radius) meters
                          centered on the origin. We use the local UTM zone
                          (EPSG:326XX/327XX) rather than Web Mercator
                          (EPSG:3857) because Mercator distorts distances by
                          1/cos(lat) — at 32 N that's ~18% stretch, which
                          misaligns the satellite texture and DEM against
                          UTM-projected building positions.
  * DEM fetch:            GeoTIFF in EPSG:4326, clipped to our bbox.
  * DEM reprojection:     EPSG:4326 -> local UTM, clipped to the exact
                          (2R x 2R) meter box, resampled to a square
                          Ogre2-valid grid. This is what the heightmap and
                          per-point elevation sampler consume so they share
                          the same uniform meter grid as building placement.
  * OSM fetch:            GeoJSON in EPSG:4326.
  * Satellite tiles:      Web Mercator (EPSG:3857). We crop the mosaic back
                          to the WGS84 bbox in pixel space so the final
                          texture covers exactly (2R x 2R) in UTM meters.
  * Asset placement:      OSM lat/lon -> UTM via CoordinateConverter ->
                          Gazebo world (meters). Same UTM zone used here, so
                          everything lines up to within pyproj precision.
"""

import elevation
from osgeo import gdal
from pyproj import Transformer

from terraforge.utils.logging import logger

gdal.UseExceptions()


def _utm_crs_for(lat: float, lon: float) -> str:
    """Return the EPSG code for the UTM zone covering (lat, lon)."""
    zone = int((lon + 180.0) / 6.0) + 1
    if zone > 60:
        zone = 1
    hemisphere = 326 if lat >= 0 else 327
    return f'EPSG:{hemisphere}{zone:02d}'


def _calculate_bounds_wgs84(location: tuple, radius_meters: float) -> tuple:
    """Calculate the WGS84 bbox around ``location``.

    Returns a (west, south, east, north) box whose UTM projection is exactly
    (2*radius_meters) x (2*radius_meters) centered on ``location``.

    Args:
        location: (latitude, longitude) in WGS84.
        radius_meters: Half-width of the bounding box, in TRUE METERS on the
            ground (not web-mercator meters).

    Returns:
        Tuple (west, south, east, north) in WGS84 degrees.
    """
    lat, lon = location
    utm_crs = _utm_crs_for(lat, lon)

    to_utm = Transformer.from_crs('EPSG:4326', utm_crs, always_xy=True)
    to_wgs = Transformer.from_crs(utm_crs, 'EPSG:4326', always_xy=True)

    cx_utm, cy_utm = to_utm.transform(lon, lat)
    west_lon, south_lat = to_wgs.transform(cx_utm - radius_meters, cy_utm - radius_meters)
    east_lon, north_lat = to_wgs.transform(cx_utm + radius_meters, cy_utm + radius_meters)

    # Antimeridian crossing: a request near lon=±180° wraps east_lon back to
    # the opposite sign, producing west_lon > east_lon. Downstream consumers
    # (elevation.clip, gdal.Warp, the satellite tile fetcher) all assume an
    # ordered (west < east) bbox and silently produce zero-data output otherwise.
    # Fail loudly here rather than chase the symptom three stages downstream.
    if west_lon > east_lon:
        raise RuntimeError(
            f'Bbox crosses the antimeridian '
            f'(W={west_lon:.4f}°, E={east_lon:.4f}°) for origin '
            f'({lat:.4f}°, {lon:.4f}°) ± {radius_meters} m. Antimeridian-'
            f'crossing worlds are not supported by the SRTM/OSM/tile '
            f'fetchers used here.'
        )

    return (west_lon, south_lat, east_lon, north_lat)


# SRTM coverage: the shuttle flew a 57° inclination orbit, so the dataset
# spans 60°N to 56°S (USGS EROS SRTM mission summary). Outside that band
# there is simply no tile to fetch.
SRTM_LAT_NORTH_LIMIT = 60.0
SRTM_LAT_SOUTH_LIMIT = -56.0


def download_dem(location: tuple, radius_meters: float, output_path: str):
    """Download SRTM3 DEM for ``location`` + ``radius_meters`` as GeoTIFF."""
    from terraforge.utils.retry import retry_call

    logger.info(
        f'Downloading DEM for location {location} with radius {radius_meters}m '
        f'to {output_path}'
    )
    bounds = _calculate_bounds_wgs84(location, radius_meters)
    # Guard here (not in _calculate_bounds_wgs84, which OSM/tile fetchers
    # share and which has no SRTM limitation — a user-supplied dem-file
    # override legitimately enables high-latitude worlds). Without this,
    # an out-of-coverage request dies inside the elevation package with an
    # obscure tile-fetch error that gets retried three times first.
    _, south_lat, _, north_lat = bounds
    if north_lat > SRTM_LAT_NORTH_LIMIT or south_lat < SRTM_LAT_SOUTH_LIMIT:
        raise RuntimeError(
            f'Requested bbox (lat {south_lat:.4f}° to {north_lat:.4f}°) is '
            f'outside SRTM coverage ({SRTM_LAT_SOUTH_LIMIT:.0f}° to '
            f'{SRTM_LAT_NORTH_LIMIT:.0f}°). Supply your own DEM with the '
            f'dem-file CLI option to generate worlds at this latitude.'
        )

    def _clip():
        elevation.clip(bounds=bounds, output=output_path, product='SRTM3')
        elevation.clean()

    try:
        # SRTM3 tiles come from elevation's upstream CDN, which occasionally
        # 5xxs. Retry so a transient outage doesn't kill the pipeline.
        retry_call(
            _clip,
            attempts=3,
            initial_delay=2.0,
            label='DEM download (SRTM3)',
        )
        logger.info(f'DEM data downloaded successfully to {output_path}')
    except Exception as e:
        logger.error(f'Failed to download DEM: {e}')
        raise


def reproject_dem_to_utm(
    src_path: str,
    dst_path: str,
    location: tuple,
    radius_meters: float,
    pixel_count: int,
    utm_crs: str,
) -> None:
    """Warp a WGS84 DEM to a square UTM grid clipped to ±``radius_meters`` around ``location``.

    The output is exactly ``pixel_count`` x ``pixel_count`` pixels spanning
    (2R x 2R) meters in the given UTM zone, i.e. uniform meters-per-pixel on
    both axes. This is what lets the Gazebo heightmap render at real-world
    scale without the anisotropic stretch you get from resizing the native
    EPSG:4326 DEM (whose X pixels are narrower than Y pixels by 1/cos(lat))
    directly onto a square target grid.

    Args:
        src_path:      Input WGS84 DEM GeoTIFF (from :func:`download_dem`).
        dst_path:      Output UTM GeoTIFF path.
        location:      (latitude, longitude) origin in WGS84.
        radius_meters: Half-width of the output in true meters.
        pixel_count:   Output raster is ``pixel_count`` x ``pixel_count``.
                       Pick an Ogre2-valid size (2^n + 1) so the downstream
                       heightmap PNG needs no resize.
        utm_crs:       EPSG string of the target UTM zone, e.g. ``EPSG:32615``.
                       Must match the converter / bbox math used elsewhere
                       in the pipeline.
    """
    lat, lon = location
    to_utm = Transformer.from_crs('EPSG:4326', utm_crs, always_xy=True)
    cx_utm, cy_utm = to_utm.transform(lon, lat)
    output_bounds = (
        cx_utm - radius_meters,
        cy_utm - radius_meters,
        cx_utm + radius_meters,
        cy_utm + radius_meters,
    )

    # Propagate the source nodata value so edge pixels outside WGS84 coverage
    # stay tagged rather than leaking the raw -32768 sentinel into downstream
    # min/max math (which happened before this fix — blew up height_amplitude
    # to tens of kilometers). SRTM3 sets band nodata to -32768; user-supplied
    # DEMs (--dem-file) may use a different value or none at all.
    src_ds = gdal.Open(src_path)
    if src_ds is None:
        raise RuntimeError(f'Failed to open source DEM for nodata probe: {src_path}')
    try:
        src_nodata = src_ds.GetRasterBand(1).GetNoDataValue()
    finally:
        src_ds = None

    if src_nodata is None:
        # No nodata declared on source. The UTM output bbox can extend past
        # the source DEM's coverage (especially for a user-supplied --dem-file
        # that's tightly cropped); without a sentinel, gdal.Warp fills those
        # pixels with bilinear-interpolated edge values that look real to
        # process_dem_to_heightmap and bias the min/max + normalization.
        # Pick a value safely below any real Earth elevation (Marianas Trench
        # ≈ -11 km) so the warp's fill is detectable downstream.
        src_nodata = -99999.0
        logger.warning(
            f'Source DEM {src_path} has no nodata value; using sentinel '
            f'{src_nodata:.0f} m to tag pixels outside source coverage.'
        )

    logger.info(
        f'Reprojecting DEM {src_path} -> {dst_path}: '
        f'{utm_crs}, {pixel_count}x{pixel_count} px over '
        f'({2 * radius_meters:.0f}m x {2 * radius_meters:.0f}m), '
        f'res={2 * radius_meters / pixel_count:.2f} m/px '
        f'(nodata={src_nodata})'
    )
    try:
        result = gdal.Warp(
            destNameOrDestDS=dst_path,
            srcDSOrSrcDSTab=src_path,
            dstSRS=utm_crs,
            outputBounds=output_bounds,
            width=pixel_count,
            height=pixel_count,
            resampleAlg='bilinear',
            format='GTiff',
            multithread=True,
            srcNodata=src_nodata,
            dstNodata=src_nodata,
        )
        if result is None:
            raise RuntimeError(f'gdal.Warp returned None for {src_path}')
        # Explicitly close so the file is flushed before callers open it.
        result = None
    except Exception as e:
        logger.error(f'Failed to reproject DEM to UTM: {e}')
        raise
