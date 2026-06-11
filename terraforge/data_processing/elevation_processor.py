# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
import numpy as np
from osgeo import gdal
from PIL import Image

from terraforge.utils.logging import logger

gdal.UseExceptions()

# Ogre2 heightmap size rule, per gz-rendering8 Ogre2Heightmap.cc: the
# native requirement is 2^n samples per side; 2^n + 1 inputs (the classic
# Ogre1 convention used here: 65, 129, ..., 4097) are detected as legacy
# format and accepted with a gzwarn, cropping the last row + column
# (≈0.1% of extent at 1025 — cosmetically negligible). Anything else
# hard-fails and the world renders with no ground mesh at all
# ("Heightmap final sampling must satisfy 2^n" + "Cannot attach a null
# geometry object"). The DEM is reprojected upstream (in elevation.py)
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
# Public alias — cli.py validates --max-heightmap-size against this list,
# and any future caller needs the same set. Underscore version retained so
# internal references in this module don't have to be touched.
OGRE2_VALID_SIZES = _OGRE2_VALID_SIZES
_DEFAULT_MAX_OGRE2_SIZE = 1025


def next_ogre2_size(n: int, max_size: int = _DEFAULT_MAX_OGRE2_SIZE) -> int:
    """Return the smallest Ogre2-valid heightmap size >= ``n``.

    Capped at ``max_size`` (default 1025). To allow 2049 / 4097 for high-res
    flyovers, the caller must pass an explicit larger ``max_size``.
    """
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
    logger.info(f'Processing DEM {dem_filepath} to heightmap {output_heightmap_path}')
    dem_dataset = None
    try:
        dem_dataset = gdal.Open(dem_filepath)
        if dem_dataset is None:
            raise Exception(f'Failed to open DEM file: {dem_filepath}')

        band = dem_dataset.GetRasterBand(1)
        if band is None:
            raise Exception('Failed to get raster band from DEM')

        width = dem_dataset.RasterXSize
        height = dem_dataset.RasterYSize
        if width != height or width not in _OGRE2_VALID_SIZES:
            raise ValueError(
                f'DEM must be a square Ogre2-valid raster (one of '
                f'{_OGRE2_VALID_SIZES}); got {width}x{height}. Reproject '
                f'via elevation.reproject_dem_to_utm before calling this.'
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
                    f'DEM {dem_filepath} is entirely nodata ({nodata}).'
                )
            valid = raster_array[valid_mask]
            min_val = float(valid.min())
            max_val = float(valid.max())
            invalid_count = int((~valid_mask).sum())
            if invalid_count > 0:
                logger.info(
                    f'DEM has {invalid_count} nodata pixels ({nodata}); '
                    f'excluded from stats and clamped to min={min_val:.1f}m '
                    f'in the output heightmap.'
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
        # DEM centered on the origin, but be defensive). Index into the
        # numpy array we already read at line 77 — a second band.ReadAsArray
        # would round-trip through GDAL for nothing.
        cx = width // 2
        cy = height // 2
        origin_raw = float(raster_array[cy, cx])
        if nodata is not None and origin_raw == nodata:
            origin_val = float(np.mean(raster_array[valid_mask]))
            logger.warning(
                f'DEM center pixel is nodata; using valid-pixel mean '
                f'{origin_val:.1f}m as origin elevation.'
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
            f'DEM processed: min={min_val:.1f}m max={max_val:.1f}m '
            f'origin={origin_val:.1f}m. Heightmap {width}x{height}. '
            f'Saved to {output_heightmap_path}'
        )
        return {'min': min_val, 'max': max_val, 'origin': origin_val}
    except Exception as e:
        logger.error(f'Error processing DEM to heightmap: {e}')
        raise
    finally:
        # Release the GDAL handle whether or not we raised. Without this, an
        # exception path leaves the file open until GC, which can block
        # subsequent runs on Windows / network mounts.
        dem_dataset = None


def write_heightmap_normal_map(
    heightmap_path: str, output_normal_path: str,
    extent_meters: float, height_amplitude_m: float,
) -> None:
    """Compute a tangent-space normal map from the 16-bit heightmap PNG.

    The SDF template requires a `<normal>` texture or its parser aborts
    at world load. A flat 4x4 RGB(128,128,255) stand-in satisfies the
    parser but leaves the terrain matte; deriving the normal from the
    heightmap gradient gives proper directional shading for free (pure
    preprocessing, zero runtime cost).

    Args:
        heightmap_path: 16-bit single-channel PNG as written by
            :func:`process_dem_to_heightmap`.
        output_normal_path: Where to write the RGB8 PNG normal map.
        extent_meters: Full side length of the terrain in world metres
            (= ``2 * radius``). Sets meters-per-pixel for the X/Y
            gradient.
        height_amplitude_m: Peak-to-trough Z scale the SDF applies to
            normalized heightmap values. 65535 steps map to
            ``height_amplitude_m`` metres, so the Z gradient per step
            is ``height_amplitude_m / 65535``.
    """
    with Image.open(heightmap_path) as img:
        if img.mode != 'I;16':
            # Fallback: let PIL convert anything else to int16. Worst
            # case is a small precision loss on an 8-bit source.
            img = img.convert('I')
        h_arr = np.asarray(img, dtype=np.float32)

    height, width = h_arr.shape
    # Central differences (np.gradient) give dH/dpx, dH/dpy in heightmap
    # units per pixel. Convert to metres: Z step = amplitude / 65535
    # per heightmap unit; X/Y step = extent / (size - 1) metres per pixel.
    dz_per_unit = height_amplitude_m / 65535.0
    px_size_m = extent_meters / max(width - 1, 1)
    # np.gradient returns (gy, gx) — rows first. Image rows grow SOUTH,
    # so gy < 0 when terrain rises NORTH. World axes: +X = east, +Y =
    # north, +Z = up (Gazebo ENU).
    gy, gx = np.gradient(h_arr)
    # World-space surface gradient in metres/metre (dimensionless slope).
    dzdx = gx * dz_per_unit / px_size_m
    dzdy = gy * dz_per_unit / px_size_m
    # Surface normal of z = h(x, y) is proportional to
    # (-dh/dx, -dh/dy, 1): a slope leans its normal DOWNHILL, never uphill.
    #   east-rising slope  -> normal leans west  -> nx < 0 -> R < 128
    #   north-rising slope -> normal leans south -> ny < 0 -> G < 128
    # dH/d(pixel_col) == gx == dh/dx_world (columns grow east): nx = -dzdx.
    # dH/d(pixel_row) == gy == -dh/dy_world (rows grow south), so
    # dh/dy_world = -dzdy and ny = -(-dzdy) = dzdy.
    # (An earlier revision had both signs inverted — normals leaned
    # uphill, so sun-facing slopes shaded dark: the classic
    # inverted-emboss artifact.)
    nx = -dzdx
    ny = dzdy
    nz = np.ones_like(nx)
    norm = np.sqrt(nx * nx + ny * ny + nz * nz)
    nx /= norm
    ny /= norm
    nz /= norm
    # Remap [-1, 1] -> [0, 255]. Round-to-nearest to avoid a half-pixel
    # darkness bias at fully-flat regions.
    rgb = np.empty((height, width, 3), dtype=np.uint8)
    rgb[..., 0] = np.clip(np.rint((nx + 1.0) * 127.5), 0, 255).astype(np.uint8)
    rgb[..., 1] = np.clip(np.rint((ny + 1.0) * 127.5), 0, 255).astype(np.uint8)
    rgb[..., 2] = np.clip(np.rint((nz + 1.0) * 127.5), 0, 255).astype(np.uint8)

    Image.fromarray(rgb, mode='RGB').save(output_normal_path, format='PNG')
    logger.info(
        f'Normal map derived from heightmap: {width}x{height} -> '
        f'{output_normal_path}'
    )


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

    Each call opens the raster and issues a 1x1 ReadAsArray; prefer
    :func:`open_dem_sampler` when you plan to sample many points (it
    reads the DEM once into numpy and hands back a fast closure).
    """
    ds = gdal.Open(dem_filepath)
    if ds is None:
        raise Exception(f'Failed to open DEM: {dem_filepath}')
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


def open_dem_sampler(dem_filepath: str, nodata_fallback: float = None):
    """Load a UTM DEM once and return a fast ``sampler(utm_x, utm_y)`` closure.

    The building/tree/road processors each ask for hundreds-to-thousands
    of elevation samples. Calling :func:`sample_dem_elevation_utm` per
    sample reopens the GeoTIFF on every query; on a typical 2 km world
    (~1000 buildings × 8 samples, ~2000 trees × 1 sample, ~500 roads ×
    5 samples) that's 10–60 s of GDAL overhead for a few KB of data.

    This helper reads the entire band into a numpy array up front (the
    DEM is at most ~4097² × 4 B ≈ 65 MB, comfortably in RAM) and returns
    a closure that indexes into it directly.
    """
    ds = gdal.Open(dem_filepath)
    if ds is None:
        raise Exception(f'Failed to open DEM: {dem_filepath}')
    try:
        gt = ds.GetGeoTransform()
        width = ds.RasterXSize
        height = ds.RasterYSize
        band = ds.GetRasterBand(1)
        nodata = band.GetNoDataValue()
        # ReadAsArray with no args returns the full band. Keep it float32
        # — plenty of precision for terrain heights, half the memory of
        # float64.
        arr = band.ReadAsArray().astype(np.float32, copy=False)
    finally:
        ds = None

    inv_px_w = 1.0 / gt[1]
    inv_px_h = 1.0 / gt[5]
    ox = gt[0]
    oy = gt[3]

    def sampler(utm_x: float, utm_y: float) -> float:
        px = int((utm_x - ox) * inv_px_w)
        py = int((utm_y - oy) * inv_px_h)
        if px < 0:
            px = 0
        elif px >= width:
            px = width - 1
        if py < 0:
            py = 0
        elif py >= height:
            py = height - 1
        value = float(arr[py, px])
        if nodata is not None and value == nodata and nodata_fallback is not None:
            return float(nodata_fallback)
        return value

    return sampler
