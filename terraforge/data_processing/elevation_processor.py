import numpy as np
from osgeo import gdal
from PIL import Image

from terraforge.utils.logging import logger

gdal.UseExceptions()

# Ogre2's heightmap rendering requires the source PNG to have dimensions of
# 2^n + 1 on each side (e.g. 65, 129, 257, 513, 1025, 2049, 4097). Otherwise
# the terrain geometry fails to build and the world renders with no ground
# mesh at all ("Heightmap final sampling must satisfy 2^n" + "Cannot attach
# a null geometry object"). The DEM is reprojected upstream (in elevation.py)
# to a UTM grid at exactly one of these sizes, so this module just has to
# verify.
#
# 2049 and 4097 are included for high-resolution aerial / LIDAR flyover DEMs
# (sub-meter posts). They quadruple (2049) and 16x (4097) the vertex count
# vs. 1025, which means proportionally higher GPU memory and slower scene
# updates — Gazebo can build them but expect noticeable load/step cost on
# modest hardware. The caller picks the target size; the default heuristic
# in cli._choose_heightmap_size caps at 1025 unless the user raises the
# ceiling via --max-heightmap-size.
_OGRE2_VALID_SIZES = (65, 129, 257, 513, 1025, 2049, 4097)
_DEFAULT_MAX_OGRE2_SIZE = 1025


def next_ogre2_size(n: int, max_size: int = _DEFAULT_MAX_OGRE2_SIZE) -> int:
    """Return the smallest Ogre2-valid heightmap size >= ``n``, capped at
    ``max_size`` (default 1025). To allow 2049 / 4097 for high-res flyovers,
    the caller must pass an explicit larger ``max_size``."""
    for s in _OGRE2_VALID_SIZES:
        if s >= n:
            return min(s, max_size)
    return min(_OGRE2_VALID_SIZES[-1], max_size)


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

        # Exclude nodata pixels from min/max so corner pixels outside the
        # source DEM's coverage (common after a WGS84->UTM warp, since the
        # UTM square doesn't perfectly overlap the source WGS84 rectangle)
        # don't pollute the elevation range. SRTM3 uses -32768 as nodata;
        # including it would inflate height_amplitude by ~1000x and flatten
        # the real terrain against it.
        nodata = band.GetNoDataValue()
        if nodata is not None:
            valid_mask = raster_array != nodata
            if not valid_mask.any():
                raise ValueError(
                    f"DEM {dem_filepath} is entirely nodata ({nodata})."
                )
            valid = raster_array[valid_mask]
            min_val = float(valid.min())
            max_val = float(valid.max())
            invalid_count = int((~valid_mask).sum())
            if invalid_count > 0:
                logger.info(
                    f"DEM has {invalid_count} nodata pixels ({nodata}); "
                    f"excluded from stats and clamped to min={min_val:.1f}m "
                    f"in the output heightmap."
                )
        else:
            valid_mask = None
            min_val = float(raster_array.min())
            max_val = float(raster_array.max())

        # Elevation at the DEM's center pixel — by construction this is the
        # sim origin (the UTM bbox is centered on the origin location), so
        # callers offset the heightmap by -origin_val to plant the origin
        # at real ground level. Fall back to the valid-pixel mean if the
        # center itself is nodata (shouldn't happen for a correctly-sized
        # DEM centered on the origin, but be defensive).
        cx = width // 2
        cy = height // 2
        origin_raw = float(band.ReadAsArray(cx, cy, 1, 1)[0][0])
        if nodata is not None and origin_raw == nodata:
            origin_val = float(np.mean(raster_array[valid_mask]))
            logger.warning(
                f"DEM center pixel is nodata; using valid-pixel mean "
                f"{origin_val:.1f}m as origin elevation."
            )
        else:
            origin_val = origin_raw

        # Clamp nodata pixels to min_val so they render as flat ground at
        # the lowest real elevation rather than plummeting to z=0 (which
        # otherwise shows as huge cliffs at the terrain edges).
        if valid_mask is not None and not valid_mask.all():
            raster_array = np.where(valid_mask, raster_array, min_val)

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


def sample_dem_elevation_utm(
    dem_filepath: str, utm_x: float, utm_y: float, nodata_fallback: float = None
) -> float:
    """Return the DEM elevation (meters) at the given UTM (x, y).

    ``dem_filepath`` must be a UTM-projected DEM in the same zone used to
    compute ``utm_x`` / ``utm_y`` (see
    :func:`terraforge.data_acquisition.elevation.reproject_dem_to_utm`).
    Points outside the DEM bounds are clamped to the nearest edge pixel.
    If the sampled pixel is nodata and ``nodata_fallback`` is given, that
    value is returned instead; otherwise nodata propagates to the caller.
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
        value = float(band.ReadAsArray(px, py, 1, 1)[0][0])
        nodata = band.GetNoDataValue()
        if nodata is not None and value == nodata and nodata_fallback is not None:
            return float(nodata_fallback)
        return value
    finally:
        ds = None
