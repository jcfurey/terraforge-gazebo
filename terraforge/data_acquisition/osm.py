import osmnx as ox

from terraforge.data_acquisition.elevation import _calculate_bounds_wgs84
from terraforge.utils.logging import logger


def download_osm_buildings(location: tuple, radius_meters: float, output_path: str) -> None:
    """Download OSM building footprints as GeoJSON for the given location and radius."""
    logger.info(
        f"Downloading OSM buildings for location {location} with radius "
        f"{radius_meters}m to {output_path}"
    )
    try:
        bbox = _calculate_bounds_wgs84(location, radius_meters)
        tags = {"building": True}
        gdf = ox.features_from_bbox(bbox=bbox, tags=tags)
        gdf.to_file(output_path, driver='GeoJSON')
        logger.info(f"OSM building data downloaded to {output_path}")
    except Exception as e:
        logger.error(f"Failed to download OSM buildings: {e}")
        raise
