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
  * OSM fetch:            GeoJSON in EPSG:4326.
  * Satellite tiles:      Web Mercator (EPSG:3857). We crop the mosaic back
                          to the WGS84 bbox in pixel space so the final
                          texture covers exactly (2R x 2R) in UTM meters.
  * Asset placement:      OSM lat/lon -> UTM via CoordinateConverter ->
                          Gazebo world (meters). Same UTM zone used here, so
                          everything lines up to within pyproj precision.
"""

import elevation
import os
from pyproj import Transformer

from terraforge.utils.logging import logger


def _utm_crs_for(lat: float, lon: float) -> str:
    """Return the EPSG code for the UTM zone covering (lat, lon)."""
    zone = int((lon + 180.0) / 6.0) + 1
    if zone > 60:
        zone = 1
    hemisphere = 326 if lat >= 0 else 327
    return f"EPSG:{hemisphere}{zone:02d}"


def _calculate_bounds_wgs84(location: tuple, radius_meters: float) -> tuple:
    """
    Calculate a (west, south, east, north) WGS84 bbox whose UTM projection is
    exactly (2*radius_meters) x (2*radius_meters) centered on ``location``.

    Args:
        location: (latitude, longitude) in WGS84.
        radius_meters: Half-width of the bounding box, in TRUE METERS on the
            ground (not web-mercator meters).

    Returns:
        Tuple (west, south, east, north) in WGS84 degrees.
    """
    lat, lon = location
    utm_crs = _utm_crs_for(lat, lon)

    to_utm = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True)
    to_wgs = Transformer.from_crs(utm_crs, "EPSG:4326", always_xy=True)

    cx_utm, cy_utm = to_utm.transform(lon, lat)
    west_lon, south_lat = to_wgs.transform(cx_utm - radius_meters, cy_utm - radius_meters)
    east_lon, north_lat = to_wgs.transform(cx_utm + radius_meters, cy_utm + radius_meters)

    return (west_lon, south_lat, east_lon, north_lat)


def download_dem(location: tuple, radius_meters: float, output_path: str):
    """Download SRTM3 DEM for ``location`` + ``radius_meters`` as GeoTIFF."""
    logger.info(
        f"Downloading DEM for location {location} with radius {radius_meters}m "
        f"to {output_path}"
    )
    try:
        bounds = _calculate_bounds_wgs84(location, radius_meters)
        elevation.clip(bounds=bounds, output=output_path, product='SRTM3')
        elevation.clean()
        logger.info(f"DEM data downloaded successfully to {output_path}")
    except Exception as e:
        logger.error(f"Failed to download DEM: {e}")
        raise
