import os
from osgeo import gdal
from terraforge.utils.logging import logger


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
            ).astype('uint16')
        else:
            normalized_array = (raster_array - min_val).astype('uint16')

        # PNG driver doesn't support Create(); use MEM then CreateCopy for 16-bit PNG.
        mem_driver = gdal.GetDriverByName('MEM')
        output_dataset = mem_driver.Create(
            '', dem_dataset.RasterXSize, dem_dataset.RasterYSize, 1, gdal.GDT_UInt16,
        )
        if output_dataset is None:
            raise Exception(f"Failed to create output heightmap file: {output_heightmap_path}")

        output_band = output_dataset.GetRasterBand(1)
        output_band.WriteArray(normalized_array)

        output_dataset.SetGeoTransform(dem_dataset.GetGeoTransform())
        output_dataset.SetProjection(dem_dataset.GetProjection())
        output_band.FlushCache()

        png_driver = gdal.GetDriverByName('PNG')
        png_dataset = png_driver.CreateCopy(output_heightmap_path, output_dataset, strict=0)

        output_dataset = None
        png_dataset = None
        dem_dataset = None

        logger.info(
            f"DEM processed: min={min_val:.1f}m max={max_val:.1f}m origin={origin_val:.1f}m. "
            f"Heightmap saved to {output_heightmap_path}"
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
