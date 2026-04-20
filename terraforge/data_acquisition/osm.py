import osmnx as ox

from terraforge.data_acquisition.elevation import _calculate_bounds_wgs84
from terraforge.utils.logging import logger


def _download(location, radius_meters, tags, output_path, label):
    bbox = _calculate_bounds_wgs84(location, radius_meters)
    try:
        gdf = ox.features_from_bbox(bbox=bbox, tags=tags)
    except Exception as e:
        logger.warning(f"{label}: no features returned ({e})")
        return 0
    if gdf.empty:
        logger.info(f"{label}: 0 features in {radius_meters}m radius")
        return 0
    gdf.to_file(output_path, driver='GeoJSON')
    logger.info(f"{label}: {len(gdf)} features -> {output_path}")
    return len(gdf)


def download_osm_buildings(location: tuple, radius_meters: float, output_path: str) -> int:
    """Download OSM building footprints (polygons)."""
    logger.info(
        f"Downloading OSM buildings for location {location} with radius "
        f"{radius_meters}m to {output_path}"
    )
    try:
        return _download(
            location, radius_meters,
            tags={"building": True},
            output_path=output_path,
            label="OSM buildings",
        )
    except Exception as e:
        logger.error(f"Failed to download OSM buildings: {e}")
        raise


def download_osm_trees(location: tuple, radius_meters: float, output_path: str) -> int:
    """Download individual trees (points) and forested areas (polygons).

    Combines:
      * ``natural=tree`` — individual mapped trees
      * ``natural=wood`` / ``landuse=forest`` — forested polygons we'll scatter
        synthetic trees into during processing.
    """
    logger.info(
        f"Downloading OSM foliage for location {location} with radius "
        f"{radius_meters}m to {output_path}"
    )
    try:
        return _download(
            location, radius_meters,
            tags={"natural": ["tree", "wood"], "landuse": "forest"},
            output_path=output_path,
            label="OSM foliage",
        )
    except Exception as e:
        logger.warning(f"Failed to download OSM foliage (continuing): {e}")
        return 0


def download_osm_roads(location: tuple, radius_meters: float, output_path: str) -> int:
    """Download road centerlines (LineStrings) tagged ``highway=*``."""
    logger.info(
        f"Downloading OSM roads for location {location} with radius "
        f"{radius_meters}m to {output_path}"
    )
    try:
        return _download(
            location, radius_meters,
            tags={"highway": True},
            output_path=output_path,
            label="OSM roads",
        )
    except Exception as e:
        logger.warning(f"Failed to download OSM roads (continuing): {e}")
        return 0
