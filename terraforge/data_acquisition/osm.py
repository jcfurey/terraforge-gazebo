

import osmnx as ox
from pathlib import Path
from terraforge.utils.config import Config
from terraforge.utils.logging import logger
from terraforge.data_acquisition.elevation import _calculate_bounds_wgs84


def download_osm_buildings(location: tuple, radius_meters: float, output_path: str) -> Path:
    """Downloads OSM buildings footporints and roads as GeoJSON"""
    logger.info(f"Downloading OSM buildings for location {location} with radius {radius_meters}m to {output_path}")
    try:
        bbox = _calculate_bounds_wgs84(location, radius_meters)

        tags = {"building": True}
        gdf = ox.features_from_bbox(bbox=bbox, tags=tags)
        gdf.to_file(output_path, driver='GeoJSON')
        logger.info(f"OSM building data downloaded successfully to {output_path}")
    except Exception as e:
        logger.error(f"Failed to download OSM buildings: {e}")
        raise

