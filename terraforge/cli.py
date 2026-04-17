import logging
import os

import click

from terraforge.data_acquisition import elevation, osm, textures
from terraforge.data_processing import (
    building_processor,
    elevation_processor,
    sdf_builder,
    texture_processor,
)
from terraforge.utils.config import config
from terraforge.utils.logging import setup_logger

logger = setup_logger('terraforge')

TEMPLATE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'data_processing', 'templates'
)


def run_generate_world(
    latitude,
    longitude,
    radius,
    output_dir,
    world_name='generated_world',
    height_amplitude=200.0,
    progress=None,
):
    """Run the full world-generation pipeline.

    Returns the absolute path to the generated `.world` file.

    `progress(msg)` is an optional callable invoked with human-readable status
    strings, so GUIs can surface progress to the user.
    """
    def _log(msg):
        logger.info(msg)
        if progress is not None:
            progress(msg)

    origin_location = (latitude, longitude)
    location_name = f"loc_{latitude:.4f}_{longitude:.4f}"
    output_dir = os.path.abspath(output_dir)

    dem_cache_path = os.path.join(config.DEM_CACHE_DIR, f"{location_name}_dem.tif")
    osm_cache_path = os.path.join(config.OSM_CACHE_DIR, f"{location_name}_buildings.geojson")
    texture_cache_dir = os.path.join(config.TEXTURE_CACHE_DIR, f"{location_name}_texture")
    os.makedirs(texture_cache_dir, exist_ok=True)

    _log("Downloading DEM, OSM, and satellite tiles...")
    elevation.download_dem(origin_location, radius, dem_cache_path)
    osm.download_osm_buildings(origin_location, radius, osm_cache_path)
    textures.download_satellite_texture_tiles(
        origin_location, radius, texture_cache_dir,
        mapbox_api_key=config.MAPBOX_API_KEY,
    )

    output_models_dir = os.path.join(output_dir, 'models')
    output_media_dir = os.path.join(output_dir, 'media')
    output_textures_dir = os.path.join(output_media_dir, 'materials', 'textures')
    os.makedirs(output_models_dir, exist_ok=True)
    os.makedirs(output_textures_dir, exist_ok=True)

    heightmap_output_path = os.path.join(output_media_dir, 'heightmap.png')
    texture_cache_png = os.path.join(texture_cache_dir, 'satellite_texture.png')
    texture_output_path = os.path.join(output_textures_dir, 'satellite_texture.png')

    _log("Processing DEM into heightmap...")
    elevation_processor.process_dem_to_heightmap(dem_cache_path, heightmap_output_path)
    _log("Building Gazebo models from OSM footprints...")
    buildings = building_processor.process_osm_buildings_to_sdf(
        osm_cache_path, output_models_dir, origin_location
    )
    if os.path.exists(texture_cache_png):
        _log("Copying satellite texture...")
        texture_processor.process_satellite_texture(texture_cache_dir, texture_output_path)

    _log("Rendering SDF world...")
    builder = sdf_builder.SDFWorldBuilder(TEMPLATE_DIR)
    extent_meters = 2.0 * radius
    output_sdf_world_path = os.path.join(output_dir, f"{world_name}.world")
    sdf_content = builder.render_world_template(
        heightmap_path=heightmap_output_path if os.path.exists(heightmap_output_path) else None,
        texture_path=texture_output_path if os.path.exists(texture_output_path) else None,
        buildings=buildings,
        extent_meters=extent_meters,
        height_amplitude=height_amplitude,
    )
    builder.save_sdf_world_file(sdf_content, output_sdf_world_path)

    _log(
        f"World saved to {output_sdf_world_path}. Launch with: "
        f"ros2 launch terraforge_gazebo spawn_world.launch.py world:={output_sdf_world_path}"
    )
    return output_sdf_world_path


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
@click.option('--height-amplitude', default=200.0, type=float,
              help='Vertical range (m) mapped to the full heightmap dynamic range.')
@click.pass_context
def generate_world(ctx, latitude, longitude, radius, output_dir, world_name, height_amplitude):
    """Generate a Gazebo Harmonic SDF world for a given location and radius."""
    try:
        run_generate_world(
            latitude=latitude,
            longitude=longitude,
            radius=radius,
            output_dir=output_dir,
            world_name=world_name,
            height_amplitude=height_amplitude,
        )
    except Exception as e:
        logger.error(f"World generation failed: {e}")
        if ctx.obj['DEBUG']:
            raise


if __name__ == '__main__':
    cli()
