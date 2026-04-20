import os

import numpy as np
from osgeo import gdal
from PIL import Image

from terraforge.utils.logging import logger

# Ogre2's heightmap rendering requires the source PNG to have dimensions of
# 2^n + 1 on each side (e.g. 65, 129, 257, 513, 1025). Otherwise the terrain
# geometry fails to build and the world renders with no ground mesh at all
# ("Heightmap final sampling must satisfy 2^n" + "Cannot attach a null
# geometry object"). We resample the normalized DEM array to the next valid
# size >= max(width, height), capped at 1025 for performance.
_OGRE2_VALID_SIZES = (65, 129, 257, 513, 1025)


def _next_ogre2_size(n: int) -> int:
    for s in _OGRE2_VALID_SIZES:
        if s >= n:
            return s
    return _OGRE2_VALID_SIZES[-1]


def process_dem_to_heightmap(dem_filepath: str, output_heightmap_path: str) -> dict:
    """Convert a DEM GeoTIFF to a 16-bit PNG heightmap.

    Returns a stats dict::

        {'min': float, 'max': float, 'origin': float}

    where ``min`` / ``max`` are the DEM elevation bounds (meters) and
    ``origin`` is the elevation at the DEM's center pixel (meters). Callers
    use these to (a) size the SDF ``<heightmap>`` vertical range and
    (b) shift the terrain so the sim origin sits at the real ground level.
    """
    logger.info(f"Processing DEM {dem_filepath} to heightmap {output_heightmap_path}")
    try:
        dem_dataset = gdal.Open(dem_filepath)
        if dem_dataset is None:
            raise Exception(f"Failed to open DEM file: {dem_filepath}")

        band = dem_dataset.GetRasterBand(1)
        if band is None:
            raise Exception("Failed to get raster band from DEM")

        raster_array = band.ReadAsArray()

        min_val = float(raster_array.min())
        max_val = float(raster_array.max())

        # Elevation at the DEM's center pixel — this is what gets placed at
        # the sim origin, so we later offset the heightmap by -origin_val.
        cx = dem_dataset.RasterXSize // 2
        cy = dem_dataset.RasterYSize // 2
        origin_val = float(band.ReadAsArray(cx, cy, 1, 1)[0][0])

        if max_val > min_val:
            normalized_array = (
                (raster_array - min_val) / (max_val - min_val) * 65535
            ).astype(np.uint16)
        else:
            normalized_array = (raster_array - min_val).astype(np.uint16)

        src_h, src_w = normalized_array.shape
        target = _next_ogre2_size(max(src_w, src_h))

        # Resample to a square Ogre2-valid grid using bicubic interpolation.
        # PIL handles uint16 images in mode 'I;16' (single-channel 16-bit).
        src_img = Image.fromarray(normalized_array, mode='I;16')
        resampled_img = src_img.resize((target, target), resample=Image.BICUBIC)
        resampled_img.save(output_heightmap_path, format='PNG')

        dem_dataset = None

        logger.info(
            f"DEM processed: min={min_val:.1f}m max={max_val:.1f}m origin={origin_val:.1f}m. "
            f"Heightmap resampled {src_w}x{src_h} -> {target}x{target} "
            f"(Ogre2 2^n+1). Saved to {output_heightmap_path}"
        )
        return {'min': min_val, 'max': max_val, 'origin': origin_val}
    except Exception as e:
        logger.error(f"Error processing DEM to heightmap: {e}")
        raise


def sample_dem_elevation(dem_filepath: str, lat: float, lon: float) -> float:
    """Return the DEM elevation (meters) at the given WGS84 lat/lon.

    Points outside the DEM bounds are clamped to the nearest edge pixel.
    """
    ds = gdal.Open(dem_filepath)
    if ds is None:
        raise Exception(f"Failed to open DEM: {dem_filepath}")
    gt = ds.GetGeoTransform()
    # gt = [origin_lon, pixel_w, 0, origin_lat, 0, pixel_h (typically negative)]
    px = int((lon - gt[0]) / gt[1])
    py = int((lat - gt[3]) / gt[5])
    px = max(0, min(px, ds.RasterXSize - 1))
    py = max(0, min(py, ds.RasterYSize - 1))
    band = ds.GetRasterBand(1)
    val = float(band.ReadAsArray(px, py, 1, 1)[0][0])
    ds = None
    return val
