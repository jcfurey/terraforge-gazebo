import json
import logging
import os
import shutil

import click
import shapely.geometry

from terraforge.data_acquisition import elevation, osm, textures
from terraforge.data_processing import (
    building_processor,
    elevation_processor,
    sdf_builder,
    texture_processor,
)
from terraforge.utils.config import config
from terraforge.utils.coordinates import CoordinateConverter
from terraforge.utils.logging import setup_logger

logger = setup_logger('terraforge')

TEMPLATE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'data_processing', 'templates'
)


@click.group()
@click.option('--debug', is_flag=True, help='Enable debug logging.')
@click.pass_context
def cli(ctx, debug):
    ctx.ensure_object(dict)
    ctx.obj['DEBUG'] = debug
    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    if debug:
        logger.debug("Debug logging enabled.")


@cli.command('generate-world')
@click.option('--latitude', required=True, type=float, help='Latitude of the location.')
@click.option('--longitude', required=True, type=float, help='Longitude of the location.')
@click.option('--radius', required=True, type=float, help='Radius in meters around the location.')
@click.option('--output-dir', default='generated_world', type=click.Path(),
              help='Output directory for the generated world.')
@click.option('--world-name', default='generated_world', help='Name of the generated Gazebo world.')
@click.pass_context
def generate_world(ctx, latitude, longitude, radius, output_dir, world_name):
    """Generates a Gazebo Harmonic SDF world for a given location and radius."""
    logger.info(
        f"Starting world generation for ({latitude}, {longitude}), radius: {radius}m, "
        f"output: {output_dir}"
    )

    origin_location = (latitude, longitude)
    location_name = f"loc_{latitude:.4f}_{longitude:.4f}"

    dem_output_path = os.path.join(config.DEM_OUTPUT_DIR, f"{location_name}_dem.tif")
    osm_output_path = os.path.join(config.OSM_OUTPUT_DIR, f"{location_name}_buildings.geojson")
    texture_output_dir = os.path.join(config.TEXTURE_OUTPUT_DIR, f"{location_name}_texture")
    os.makedirs(texture_output_dir, exist_ok=True)

    try:
        elevation.download_dem(origin_location, radius, dem_output_path)
        osm.download_osm_buildings(origin_location, radius, osm_output_path)
        textures.download_satellite_texture_tiles(
            origin_location, radius, texture_output_dir,
            mapbox_api_key=config.MAPBOX_API_KEY,
        )
    except Exception as e:
        logger.error(f"Data acquisition failed: {e}")
        if ctx.obj['DEBUG']:
            raise
        return

    heightmap_output_path = os.path.join(config.DEM_OUTPUT_DIR, f"{location_name}_heightmap.png")
    building_sdf_output_dir = os.path.join(
        config.OSM_OUTPUT_DIR, f"{location_name}_building_models_sdf"
    )
    processed_texture_output_dir = os.path.join(config.TEXTURE_OUTPUT_DIR, "processed_textures")
    processed_texture_output_path = os.path.join(
        processed_texture_output_dir, "satellite_texture.png"
    )

    try:
        elevation_processor.process_dem_to_heightmap(dem_output_path, heightmap_output_path)
        building_processor.process_osm_buildings_to_sdf(osm_output_path, building_sdf_output_dir)
        texture_processor.process_satellite_texture(
            texture_output_dir, processed_texture_output_path
        )
    except Exception as e:
        logger.error(f"Data processing failed: {e}")
        if ctx.obj['DEBUG']:
            raise
        return

    converter = CoordinateConverter(origin_location)

    world_builder = sdf_builder.SDFWorldBuilder(TEMPLATE_DIR)

    building_model_paths = (
        [
            os.path.join(building_sdf_output_dir, f)
            for f in os.listdir(building_sdf_output_dir)
            if f.endswith('.sdf')
        ]
        if os.path.exists(building_sdf_output_dir)
        else []
    )

    building_poses_gazebo = []
    if os.path.exists(osm_output_path):
        with open(osm_output_path, 'r') as f:
            osm_data = json.load(f)
        for feature in osm_data['features']:
            if feature['geometry']['type'] in ('Polygon', 'MultiPolygon'):
                polygon = shapely.geometry.shape(feature['geometry'])
                centroid = polygon.centroid
                gazebo_pose = converter.wgs84_to_gazebo((centroid.y, centroid.x))
                building_poses_gazebo.append(gazebo_pose[:2])

    output_sdf_world_path = os.path.join(output_dir, f"{world_name}.world")
    output_textures_dir = os.path.join(output_dir, "media", "materials", "textures")
    os.makedirs(output_textures_dir, exist_ok=True)

    texture_path_for_sdf = None
    output_texture_file_in_media = os.path.join(output_textures_dir, "satellite_texture.png")
    if os.path.exists(processed_texture_output_path):
        shutil.copy2(processed_texture_output_path, output_texture_file_in_media)
        texture_path_for_sdf = os.path.relpath(
            output_texture_file_in_media, os.path.dirname(output_sdf_world_path)
        )

    try:
        sdf_content = world_builder.render_world_template(
            heightmap_path=heightmap_output_path,
            texture_path=texture_path_for_sdf,
            building_model_paths=building_model_paths,
            building_poses=building_poses_gazebo,
        )
        world_builder.save_sdf_world_file(sdf_content, output_sdf_world_path)
        logger.info(f"World generation complete. SDF saved to: {output_sdf_world_path}")
    except Exception as e:
        logger.error(f"SDF world generation failed: {e}")
        if ctx.obj['DEBUG']:
            raise
        return

    logger.info(f"Gazebo world generated in: {output_dir}")


if __name__ == '__main__':
    cli()
