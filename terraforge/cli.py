import logging
import os

import click
from osgeo import gdal

from terraforge.data_acquisition import elevation, osm, textures
from terraforge.data_acquisition.elevation import _calculate_bounds_wgs84
from terraforge.data_processing import (
    building_processor,
    cloud_mask as cloud_mask_mod,
    elevation_processor,
    road_processor,
    sdf_builder,
    texture_processor,
    tree_processor,
)
from terraforge.utils.config import config
from terraforge.utils.coordinates import CoordinateConverter
from terraforge.utils.logging import setup_logger

logger = setup_logger('terraforge')

TEMPLATE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'data_processing', 'templates'
)


def _choose_heightmap_size(wgs84_dem_path: str) -> int:
    """Pick an Ogre2-valid heightmap size based on the native DEM resolution.

    We round max(src_w, src_h) up to the next size in
    ``_OGRE2_VALID_SIZES`` (65, 129, 257, 513, 1025). This preserves the
    amount of real detail present in SRTM / user-supplied DEMs without
    over-upsampling (which just smooths between real samples) or dropping
    detail to fit a lower size.
    """
    ds = gdal.Open(wgs84_dem_path)
    if ds is None:
        raise RuntimeError(f"Failed to open DEM for sizing: {wgs84_dem_path}")
    try:
        src_w = ds.RasterXSize
        src_h = ds.RasterYSize
    finally:
        ds = None
    return elevation_processor.next_ogre2_size(max(src_w, src_h))


def run_generate_world(
    latitude,
    longitude,
    radius,
    output_dir,
    world_name='generated_world',
    height_amplitude=None,
    tile_provider=None,
    tile_api_key=None,
    with_roads=False,
    cloud_filter=True,
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
    # Cache key includes radius — the WGS84 bbox depends on it, and a cache
    # file produced at one radius will have the wrong content if reused at
    # another. Bumping the key format (_r{int(radius)}) also sidesteps any
    # pre-existing caches generated with the old Web-Mercator bbox math.
    location_name = f"loc_{latitude:.4f}_{longitude:.4f}_r{int(radius)}"
    output_dir = os.path.abspath(output_dir)

    dem_cache_path = os.path.join(config.DEM_CACHE_DIR, f"{location_name}_dem.tif")
    dem_utm_cache_path = os.path.join(config.DEM_CACHE_DIR, f"{location_name}_dem_utm.tif")
    buildings_cache_path = os.path.join(config.OSM_CACHE_DIR, f"{location_name}_buildings.geojson")
    trees_cache_path = os.path.join(config.OSM_CACHE_DIR, f"{location_name}_foliage.geojson")
    roads_cache_path = os.path.join(config.OSM_CACHE_DIR, f"{location_name}_roads.geojson")
    texture_cache_dir = os.path.join(config.TEXTURE_CACHE_DIR, f"{location_name}_texture")
    os.makedirs(texture_cache_dir, exist_ok=True)

    _log("Downloading DEM, OSM layers, and satellite tiles...")
    elevation.download_dem(origin_location, radius, dem_cache_path)
    osm.download_osm_buildings(origin_location, radius, buildings_cache_path)
    osm.download_osm_trees(origin_location, radius, trees_cache_path)
    if with_roads:
        osm.download_osm_roads(origin_location, radius, roads_cache_path)
    textures.download_satellite_texture_tiles(
        origin_location, radius, texture_cache_dir,
        provider=tile_provider,
        api_key=tile_api_key,
    )

    # Reproject the WGS84 DEM into a true meter-square UTM grid before any
    # heightmap / elevation-sampling happens downstream. The converter is
    # created here (not later) so both the reprojection and the per-point
    # sampler share the same UTM zone.
    converter = CoordinateConverter(origin_location)

    _log("Reprojecting DEM into local UTM grid...")
    target_size = _choose_heightmap_size(dem_cache_path)
    elevation.reproject_dem_to_utm(
        dem_cache_path,
        dem_utm_cache_path,
        origin_location,
        radius,
        pixel_count=target_size,
        utm_crs=converter.utm_crs_string,
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
    dem_stats = elevation_processor.process_dem_to_heightmap(
        dem_utm_cache_path, heightmap_output_path
    )

    # Auto-pick height_amplitude from real DEM relief if caller didn't override.
    dem_range = max(dem_stats['max'] - dem_stats['min'], 0.1)
    if height_amplitude is None or height_amplitude <= 0:
        height_amplitude = dem_range
        _log(
            f"Auto height_amplitude = {height_amplitude:.2f} m "
            f"(DEM relief: {dem_stats['min']:.1f}-{dem_stats['max']:.1f})"
        )

    # Shift the terrain so the DEM elevation at the origin pixel maps to sim z=0.
    # Without this, the SDF heightmap's *minimum* elevation sits at z=0 and the
    # origin ends up buried under (origin - min) * scale meters of terrain.
    z_per_meter = height_amplitude / dem_range
    origin_norm_z = (dem_stats['origin'] - dem_stats['min']) * z_per_meter
    terrain_z_offset = -origin_norm_z

    # Per-asset elevation lookup: convert Gazebo XY -> UTM -> DEM pixel ->
    # elevation -> normalized sim z (shifted by terrain_z_offset so it
    # matches the terrain visual). Staying in UTM all the way through
    # guarantees the sample comes from the same grid Gazebo is rendering.
    def sample_terrain_z(gx, gy):
        utm_x, utm_y = converter.gazebo_to_utm((gx, gy))
        raw_elev = elevation_processor.sample_dem_elevation_utm(
            dem_utm_cache_path, utm_x, utm_y
        )
        return (raw_elev - dem_stats['min']) * z_per_meter + terrain_z_offset

    # Build a cloud mask from the cropped-to-exact-bbox satellite texture so
    # asset processors can skip placements in regions the satellite couldn't
    # verify. Disable via cloud_filter=False if your imagery is cloud-free or
    # you want every OSM feature placed regardless of visual coverage.
    cloud_mask = None
    if cloud_filter and os.path.exists(texture_cache_png):
        texture_bbox = _calculate_bounds_wgs84(origin_location, radius)
        cloud_mask = cloud_mask_mod.build_cloud_mask(texture_cache_png, texture_bbox)

    _log("Building Gazebo models from OSM footprints...")
    buildings = building_processor.process_osm_buildings_to_sdf(
        buildings_cache_path, output_models_dir, origin_location,
        elevation_sampler=sample_terrain_z,
        cloud_mask=cloud_mask,
    )
    _log("Scattering trees from OSM foliage...")
    trees = tree_processor.process_osm_trees_to_sdf(
        trees_cache_path, output_models_dir, origin_location,
        elevation_sampler=sample_terrain_z,
        cloud_mask=cloud_mask,
        world_half_extent_m=radius,
    )
    if with_roads:
        _log("Laying down roads from OSM highways...")
        roads = road_processor.process_osm_roads_to_sdf(
            roads_cache_path, output_models_dir, origin_location,
            elevation_sampler=sample_terrain_z,
        )
    else:
        # Roads are opt-in because flat-per-segment extrusions float above
        # undulating terrain (the heightmap has no collision in dartsim, so
        # "follow the ground" requires per-polyline elevation interpolation
        # that isn't implemented yet). Re-enable via --with-roads.
        roads = []
    if os.path.exists(texture_cache_png):
        _log("Copying satellite texture...")
        texture_processor.process_satellite_texture(texture_cache_dir, texture_output_path)

    # Save the cloud mask alongside the heightmap for debug / visualization.
    if cloud_mask is not None:
        cloud_mask.save_debug_png(os.path.join(output_media_dir, 'cloud_mask.png'))

    _log("Rendering SDF world...")
    builder = sdf_builder.SDFWorldBuilder(TEMPLATE_DIR)
    extent_meters = 2.0 * radius
    output_sdf_world_path = os.path.join(output_dir, f"{world_name}.world")
    sdf_content = builder.render_world_template(
        heightmap_path=heightmap_output_path if os.path.exists(heightmap_output_path) else None,
        texture_path=texture_output_path if os.path.exists(texture_output_path) else None,
        buildings=buildings,
        trees=trees,
        roads=roads,
        extent_meters=extent_meters,
        height_amplitude=height_amplitude,
        terrain_z_offset=terrain_z_offset,
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
@click.option('--height-amplitude', default=None, type=float,
              help='Vertical range (m) mapped to the full heightmap dynamic range. '
                   'Auto-detected from the DEM elevation range if unset.')
@click.option('--tile-provider',
              type=click.Choice(sorted(textures.PROVIDERS.keys()), case_sensitive=False),
              default=None,
              help='Satellite tile source. Defaults to $SATELLITE_TEXTURE_SOURCE or "mapbox".')
@click.option('--tile-api-key', default=None,
              help='API key for the selected tile provider (overrides the provider-specific env var).')
@click.option('--with-roads/--no-roads', default=False,
              help='Emit OSM highway ways as flat road strips. Off by default — '
                   'current implementation is flat-per-segment and floats over undulating terrain.')
@click.option('--cloud-filter/--no-cloud-filter', default=True,
              help='Drop buildings/trees whose satellite pixel looks like cloud '
                   '(high luminance + low saturation). On by default.')
@click.pass_context
def generate_world(ctx, latitude, longitude, radius, output_dir, world_name,
                   height_amplitude, tile_provider, tile_api_key, with_roads,
                   cloud_filter):
    """Generate a Gazebo Harmonic SDF world for a given location and radius."""
    try:
        run_generate_world(
            latitude=latitude,
            longitude=longitude,
            radius=radius,
            output_dir=output_dir,
            world_name=world_name,
            height_amplitude=height_amplitude,
            tile_provider=tile_provider,
            tile_api_key=tile_api_key,
            with_roads=with_roads,
            cloud_filter=cloud_filter,
        )
    except Exception as e:
        logger.error(f"World generation failed: {e}")
        if ctx.obj['DEBUG']:
            raise
        ctx.exit(1)


@cli.command('list-tile-providers')
def list_tile_providers():
    """List available satellite tile providers and their attribution requirements."""
    for name in sorted(textures.PROVIDERS.keys()):
        p = textures.PROVIDERS[name]
        key_tag = "key required" if p.requires_key else "no key"
        click.echo(f"{name:18}  max_zoom={p.max_zoom:<2}  [{key_tag}]  {p.attribution}")


if __name__ == '__main__':
    cli()
