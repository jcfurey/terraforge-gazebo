import os
from osgeo import gdal
from terraforge.utils.logging import logger

def process_dem_to_heightmap(dem_filepath: str, output_heightmap_path: str):
    logger.info(f"Processing DEM {dem_filepath} to heightmap {output_heightmap_path}")
    try:
        dem_dataset = gdal.Open(dem_filepath)
        if dem_dataset is None:
            raise Exception(f"Failed to open DEM file: {dem_filepath}")
        
        band = dem_dataset.GetRasterBand(1)
        if band is None:
            raise Exception("Failed to get raster band from DEM")
        
        raster_array = band.ReadAsArray()

        min_val = raster_array.min()
        max_val = raster_array.max()

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

        # copy geotransform and projection from the source DEM
        output_dataset.SetGeoTransform(dem_dataset.GetGeoTransform())
        output_dataset.SetProjection(dem_dataset.GetProjection())
        output_band.FlushCache()

        png_driver = gdal.GetDriverByName('PNG')
        png_dataset = png_driver.CreateCopy(output_heightmap_path, output_dataset, strict=0)

        output_dataset = None
        png_dataset = None
        dem_dataset = None

        logger.info(f"DEM processed and heightmap saved to {output_heightmap_path}")
    except Exception as e:
        logger.error(f"Error processing DEM to heightmap: {e}")
        raise
