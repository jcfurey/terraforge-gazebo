# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
import osmnx as ox

from terraforge.data_acquisition.elevation import _calculate_bounds_wgs84
from terraforge.utils.logging import logger
from terraforge.utils.retry import retry_call

try:
    # Not re-exported at package level as of osmnx 2.1; the private module
    # has been its stable home since 1.3.
    from osmnx._errors import InsufficientResponseError
except ImportError:  # future osmnx relocation — degrade to retrying
    InsufficientResponseError = ()


def _download(location, radius_meters, tags, output_path, label):
    bbox = _calculate_bounds_wgs84(location, radius_meters)

    def fetch():
        try:
            return ox.features_from_bbox(bbox=bbox, tags=tags)
        except InsufficientResponseError:
            # "No matching features" is a definitive empty answer from
            # Overpass, not a transient failure — retrying it just burns
            # ~6 s of backoff per empty layer and logs misleading warnings.
            return None

    # Overpass commonly 429s on bursty queries (we issue ~4 in a row).
    # A short backoff absorbs that without the caller seeing a failure.
    gdf = retry_call(
        fetch,
        attempts=3,
        initial_delay=2.0,
        label=label,
    )
    if gdf is None or gdf.empty:
        logger.info(f'{label}: 0 features in {radius_meters}m radius')
        return 0
    gdf.to_file(output_path, driver='GeoJSON')
    logger.info(f'{label}: {len(gdf)} features -> {output_path}')
    return len(gdf)


def download_osm_buildings(location: tuple, radius_meters: float, output_path: str) -> int:
    """Download OSM building footprints (polygons).

    Failure policy is intentionally STRICTER than the other downloaders:
    a buildings fetch failure re-raises and aborts the pipeline, whereas
    trees / roads / parking failures log a warning and return 0. The
    rationale is that a world without buildings is rarely useful (the
    whole point of pulling OSM is the structures), while a world without
    trees, roads, or parking polygons is a degraded but still-functional
    sim. If you want graceful degradation here too, catch the exception
    upstream.
    """
    logger.info(
        f'Downloading OSM buildings for location {location} with radius '
        f'{radius_meters}m to {output_path}'
    )
    try:
        return _download(
            location, radius_meters,
            tags={'building': True},
            output_path=output_path,
            label='OSM buildings',
        )
    except Exception as e:
        logger.error(f'Failed to download OSM buildings: {e}')
        raise


def download_osm_trees(location: tuple, radius_meters: float, output_path: str) -> int:
    """Download individual trees (points) and vegetated areas (polygons).

    Combines:
      * ``natural=tree`` — individual mapped trees (points)
      * ``natural=tree_row`` — hedgerows / tree lines (LineString, handled as scatter)
      * ``natural=wood`` — naturally wooded polygons (dense scatter)
      * ``natural=scrub`` — scrubland / shrubs (sparse small-variant scatter)
      * ``natural=heath`` — heathland (sparse small-variant scatter)
      * ``landuse=forest`` — managed forest polygons (dense scatter)
      * ``landuse=orchard`` — planted orchards (medium scatter, mid-size trees)
      * ``landuse=vineyard`` — vineyards (sparse small-variant scatter)
      * ``leisure=park`` / ``leisure=garden`` — parks with scattered trees
        (very sparse, mid/small variants)

    Previously only ``natural=tree|wood`` + ``landuse=forest`` were queried, so
    rural scrubland and orchards were invisible to the scatterer. Expanding the
    query here is the only place these classes enter the pipeline; the
    tree_processor keys on the same tags to pick appropriate densities.
    """
    logger.info(
        f'Downloading OSM foliage for location {location} with radius '
        f'{radius_meters}m to {output_path}'
    )
    try:
        return _download(
            location, radius_meters,
            tags={
                'natural': ['tree', 'tree_row', 'wood', 'scrub', 'heath'],
                'landuse': ['forest', 'orchard', 'vineyard'],
                'leisure': ['park', 'garden'],
            },
            output_path=output_path,
            label='OSM foliage',
        )
    except Exception as e:
        logger.warning(f'Failed to download OSM foliage (continuing): {e}')
        return 0


def download_osm_roads(location: tuple, radius_meters: float, output_path: str) -> int:
    """Download road centerlines (LineStrings) tagged ``highway=*``."""
    logger.info(
        f'Downloading OSM roads for location {location} with radius '
        f'{radius_meters}m to {output_path}'
    )
    try:
        return _download(
            location, radius_meters,
            tags={'highway': True},
            output_path=output_path,
            label='OSM roads',
        )
    except Exception as e:
        logger.warning(f'Failed to download OSM roads (continuing): {e}')
        return 0


def download_osm_parking(location: tuple, radius_meters: float, output_path: str) -> int:
    """Download parking lot / structure polygons tagged ``amenity=parking``.

    Used by the foliage mask as a negative raster — image-based tree scatter
    should never land in a parking lot, even if the satellite pixel looks
    green (algae, faded paint, patchy grass inside lot dividers). Same
    graceful-degrade behaviour as the road downloader: if Overpass is down
    or returns nothing, the pipeline still generates a world, just without
    parking exclusion.
    """
    logger.info(
        f'Downloading OSM parking for location {location} with radius '
        f'{radius_meters}m to {output_path}'
    )
    try:
        return _download(
            location, radius_meters,
            tags={'amenity': ['parking']},
            output_path=output_path,
            label='OSM parking',
        )
    except Exception as e:
        logger.warning(f'Failed to download OSM parking (continuing): {e}')
        return 0
