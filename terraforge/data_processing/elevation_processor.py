import numpy as np
from osgeo import gdal
from PIL import Image

from terraforge.utils.logging import logger

# Ogre2's heightmap rendering requires the source PNG to have dimensions of
# 2^n + 1 on each side (e.g. 65, 129, 257, 513, 1025). Otherwise the terrain
# geometry fails to build and the world renders with no ground mesh at all
# ("Heightmap final sampling must satisfy 2^n" + "Cannot attach a null
# geometry object"). The DEM is reprojected upstream (in elevation.py) to a
# UTM grid at exactly one of these sizes, so this module just has to verify.
_OGRE2_VALID_SIZES = (65, 129, 257, 513, 1025)


def next_ogre2_size(n: int) -> int:
    """Return the smallest Ogre2-valid heightmap size >= n, capped at 1025."""
    for s in _OGRE2_VALID_SIZES:
        if s >= n:
            return s
    return _OGRE2_VALID_SIZES[-1]


def process_dem_to_heightmap(dem_filepath: str, output_heightmap_path: str) -> dict:
    """Convert a UTM-projected DEM GeoTIFF to a 16-bit PNG heightmap.

    The input DEM is expected to already be square and sized to one of
    ``_OGRE2_VALID_SIZES`` — typically produced by
    :func:`terraforge.data_acquisition.elevation.reproject_dem_to_utm`. This
    function only normalizes elevations into the 16-bit PNG range; the
    spatial grid must already be in true meters so the heightmap renders at
    real-world scale without anisotropic stretch.

    Returns a stats dict::

        {'min': float, 'max': float, 'origin': float}

    where ``min`` / ``max`` are the DEM elevation bounds (meters) and
    ``origin`` is the elevation at the DEM's center pixel (meters). Callers
    use these to (a) size the SDF ``<heightmap>`` vertical range and
    (b) shift the terrain so the sim origin sits at the real ground level.
    """
    logger.info(f"Processing DEM {dem_filepath} to heightmap {output_heightmap_path}")
    dem_dataset = None
    try:
        dem_dataset = gdal.Open(dem_filepath)
        if dem_dataset is None:
            raise Exception(f"Failed to open DEM file: {dem_filepath}")

        band = dem_dataset.GetRasterBand(1)
        if band is None:
            raise Exception("Failed to get raster band from DEM")

        width = dem_dataset.RasterXSize
        height = dem_dataset.RasterYSize
        if width != height or width not in _OGRE2_VALID_SIZES:
            raise ValueError(
                f"DEM must be a square Ogre2-valid raster (one of "
                f"{_OGRE2_VALID_SIZES}); got {width}x{height}. Reproject "
                f"via elevation.reproject_dem_to_utm before calling this."
            )

        raster_array = band.ReadAsArray()
        min_val = float(raster_array.min())
        max_val = float(raster_array.max())

        # Elevation at the DEM's center pixel — by construction this is the
        # sim origin (the UTM bbox is centered on the origin location), so
        # callers offset the heightmap by -origin_val to plant the origin
        # at real ground level.
        cx = width // 2
        cy = height // 2
        origin_val = float(band.ReadAsArray(cx, cy, 1, 1)[0][0])

        if max_val > min_val:
            normalized_array = (
                (raster_array - min_val) / (max_val - min_val) * 65535
            ).astype(np.uint16)
        else:
            normalized_array = (raster_array - min_val).astype(np.uint16)

        # PIL handles uint16 images in mode 'I;16' (single-channel 16-bit).
        Image.fromarray(normalized_array, mode='I;16').save(
            output_heightmap_path, format='PNG'
        )

        logger.info(
            f"DEM processed: min={min_val:.1f}m max={max_val:.1f}m "
            f"origin={origin_val:.1f}m. Heightmap {width}x{height}. "
            f"Saved to {output_heightmap_path}"
        )
        return {'min': min_val, 'max': max_val, 'origin': origin_val}
    except Exception as e:
        logger.error(f"Error processing DEM to heightmap: {e}")
        raise
    finally:
        # Release the GDAL handle whether or not we raised. Without this, an
        # exception path leaves the file open until GC, which can block
        # subsequent runs on Windows / network mounts.
        dem_dataset = None


def sample_dem_elevation_utm(dem_filepath: str, utm_x: float, utm_y: float) -> float:
    """Return the DEM elevation (meters) at the given UTM (x, y).

    ``dem_filepath`` must be a UTM-projected DEM in the same zone used to
    compute ``utm_x`` / ``utm_y`` (see
    :func:`terraforge.data_acquisition.elevation.reproject_dem_to_utm`).
    Points outside the DEM bounds are clamped to the nearest edge pixel.
    """
    ds = gdal.Open(dem_filepath)
    if ds is None:
        raise Exception(f"Failed to open DEM: {dem_filepath}")
    try:
        gt = ds.GetGeoTransform()
        # gt = [origin_x, pixel_w, 0, origin_y, 0, pixel_h (typically negative)]
        px = int((utm_x - gt[0]) / gt[1])
        py = int((utm_y - gt[3]) / gt[5])
        px = max(0, min(px, ds.RasterXSize - 1))
        py = max(0, min(py, ds.RasterYSize - 1))
        band = ds.GetRasterBand(1)
        return float(band.ReadAsArray(px, py, 1, 1)[0][0])
    finally:
        ds = None
