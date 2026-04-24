import logging
import os

import click
from osgeo import gdal
from PIL import Image

from terraforge.data_acquisition import elevation, osm, textures
from terraforge.data_acquisition.elevation import _calculate_bounds_wgs84
from terraforge.data_processing import (
    building_processor,
    cloud_mask as cloud_mask_mod,
    elevation_processor,
    foliage_mask as foliage_mask_mod,
    road_processor,
    sdf_builder,
    tree_processor,
)
from terraforge.utils.config import config
from terraforge.utils.coordinates import CoordinateConverter
from terraforge.utils.logging import setup_logger
from terraforge.utils.naming import safe_identifier

logger = setup_logger('terraforge')

TEMPLATE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'data_processing', 'templates'
)


def _choose_heightmap_size(src_dem_path: str, max_size: int) -> int:
    """Pick an Ogre2-valid heightmap size based on the native DEM resolution.

    Rounds max(src_w, src_h) up to the next valid size (65, 129, 257, 513,
    1025, 2049, 4097), capped at ``max_size``. SRTM3 (~90 m/px) typically
    lands at 65-257 over a few-km world; sub-meter LIDAR / aerial flyovers
    over the same area can fill 2049 or 4097, but require the user to raise
    the cap via --max-heightmap-size to avoid silently downsampling them.
    """
    ds = gdal.Open(src_dem_path)
    if ds is None:
        raise RuntimeError(f"Failed to open DEM for sizing: {src_dem_path}")
    try:
        src_w = ds.RasterXSize
        src_h = ds.RasterYSize
    finally:
        ds = None
    chosen = elevation_processor.next_ogre2_size(max(src_w, src_h), max_size=max_size)
    if max(src_w, src_h) > chosen:
        logger.warning(
            f"Source DEM is {src_w}x{src_h}; capping heightmap to "
            f"{chosen}x{chosen} (raise --max-heightmap-size to preserve "
            f"native resolution at the cost of GPU memory / load time)."
        )
    return chosen


def run_generate_world(
    latitude,
    longitude,
    radius,
    output_dir,
    world_name='generated_world',
    height_amplitude=None,
    tile_provider=None,
    tile_api_key=None,
    tile_zoom=None,
    tile_max_count=None,
    max_texture_px=None,
    with_roads=False,
    cloud_filter=True,
    dem_file=None,
    texture_file=None,
    max_heightmap_size=1025,
    performer_ref='rovermax',
    disable_level_streaming=False,
    foliage_style='cartoon',
    foliage_mask_mode='rgb-osm',
    texture_format='jpeg',
    progress=None,
    progress_percent=None,
    cancel_flag=None,
):
    """Run the full world-generation pipeline.

    Returns the absolute path to the generated `.world` file.

    `progress(msg)` is an optional callable invoked with human-readable status
    strings, so GUIs can surface progress to the user.
    `progress_percent(int)` is an optional callable invoked with 0-100 values
    at each pipeline stage so GUIs can drive a progress bar.
    `cancel_flag()` is an optional callable polled between stages; if it
    returns truthy, the pipeline raises ``InterruptedError`` and cleanly
    aborts instead of running the rest of the pipeline.
    """
    def _log(msg):
        logger.info(msg)
        if progress is not None:
            progress(msg)

    def _pct(value):
        if progress_percent is not None:
            progress_percent(int(value))

    def _check_cancel():
        if cancel_flag is not None and cancel_flag():
            raise InterruptedError("World generation cancelled by user.")

    # Validate identifiers that flow into filesystem paths and SDF XML
    # before we mkdir anything or hit the network. world_name becomes a
    # directory suffix and a `file://` URI; performer_ref is interpolated
    # directly into SDF <model>/<performer>/<ref> names.
    world_name = safe_identifier(world_name, field='world_name')
    performer_ref = safe_identifier(performer_ref, field='performer_ref')

    if foliage_mask_mode == 'worldcover':
        raise NotImplementedError(
            "foliage_mask_mode='worldcover' is reserved for a future ESA "
            "WorldCover integration; use 'rgb-osm' or 'off'."
        )

    # Fuel mode emits <include><uri>model://tree_fuel_<i></uri></include>.
    # Auto-generate minimal wrappers into <output>/models_fuel/ so the
    # world is self-contained. spawn_world.launch.py adds that dir to
    # GZ_SIM_RESOURCE_PATH; if the operator has a richer wrapper pack
    # earlier on the path (e.g. mesh-backed fuel trees in a bringup
    # workspace), it takes precedence because write_fuel_wrappers only
    # writes files that don't already exist.
    if foliage_style == 'fuel':
        models_fuel_dir = os.path.join(os.path.abspath(output_dir), 'models_fuel')
        written = tree_processor.write_fuel_wrappers(models_fuel_dir)
        if written:
            logger.info(
                f"--foliage-style fuel: wrote {len(written)} wrapper model(s) "
                f"to {models_fuel_dir} ({', '.join(written)}). Launch will "
                f"add this dir to GZ_SIM_RESOURCE_PATH."
            )
        missing = tree_processor.missing_fuel_wrappers(extra_roots=[models_fuel_dir])
        if missing:
            logger.warning(
                f"--foliage-style fuel: still missing {missing} even after "
                f"writing defaults; check filesystem permissions on "
                f"{models_fuel_dir}."
            )

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
    parking_cache_path = os.path.join(config.OSM_CACHE_DIR, f"{location_name}_parking.geojson")
    texture_cache_dir = os.path.join(config.TEXTURE_CACHE_DIR, f"{location_name}_texture")
    os.makedirs(texture_cache_dir, exist_ok=True)

    # User-supplied DEM / orthophoto override the cache-fetch paths. Both must
    # cover the (2R x 2R) meter bbox around the origin (gdal.Warp will clip /
    # reproject to fit). For aerial flyovers, source CRS is auto-detected
    # from the file's metadata — works for any CRS gdal can read (UTM, state
    # plane, EPSG:4326, etc.).
    dem_source_path = dem_cache_path
    if dem_file is not None:
        dem_source_path = os.path.abspath(dem_file)
        if not os.path.isfile(dem_source_path):
            raise click.UsageError(f"--dem-file does not exist: {dem_source_path}")
        _log(f"Using user-supplied DEM: {dem_source_path}")
    else:
        _log("Downloading SRTM3 DEM...")
        _pct(5)
        elevation.download_dem(origin_location, radius, dem_cache_path)
    _check_cancel()

    _log("Downloading OSM layers...")
    _pct(15)
    osm.download_osm_buildings(origin_location, radius, buildings_cache_path)
    osm.download_osm_trees(origin_location, radius, trees_cache_path)
    # Roads + parking are always fetched because the foliage mask consumes
    # them as negative rasters (no trees on asphalt, no trees in parking
    # lots) even when road rendering is disabled. Road emission stays gated
    # on --with-roads below.
    osm.download_osm_roads(origin_location, radius, roads_cache_path)
    osm.download_osm_parking(origin_location, radius, parking_cache_path)

    # Need the UTM CRS before the tile download so we can ask the downloader
    # to reproject the merged mosaic from Web Mercator to UTM. Without this,
    # the PNG's pixel grid doesn't align with the UTM-placed buildings/trees
    # and you get cross-corner drift (NE aligned, SW off).
    converter = CoordinateConverter(origin_location)

    if texture_file is not None:
        texture_user_path = os.path.abspath(texture_file)
        if not os.path.isfile(texture_user_path):
            raise click.UsageError(f"--texture-file does not exist: {texture_user_path}")
        _log(f"Using user-supplied orthophoto: {texture_user_path}")
    else:
        _check_cancel()
        _log("Downloading satellite tiles...")
        _pct(25)
        textures.download_satellite_texture_tiles(
            origin_location, radius, texture_cache_dir,
            provider=tile_provider,
            api_key=tile_api_key,
            zoom=tile_zoom,
            max_tiles=tile_max_count if tile_max_count is not None else textures.MAX_TILES,
            utm_crs=converter.utm_crs_string,
            max_texture_px=(max_texture_px if max_texture_px is not None
                            else textures.DEFAULT_MAX_TEXTURE_PX),
            progress=_log,
        )

    # DEM reprojection uses the same converter created above, so both the
    # texture and DEM share a single UTM zone.
    _check_cancel()
    _log("Reprojecting DEM into local UTM grid...")
    _pct(55)
    target_size = _choose_heightmap_size(dem_source_path, max_size=max_heightmap_size)
    elevation.reproject_dem_to_utm(
        dem_source_path,
        dem_utm_cache_path,
        origin_location,
        radius,
        pixel_count=target_size,
        utm_crs=converter.utm_crs_string,
    )

    output_models_dir = os.path.join(output_dir, 'models')
    # Per-world media subdir: `<output-dir>/media_<world_name>/`. Without the
    # suffix, a second generation at a different lat/lon (or with a different
    # foliage style at the same coords) would overwrite the first world's
    # heightmap / satellite texture / cloud mask, leaving the first world's
    # inline building/tree poses floating relative to a stranger's terrain.
    # Each world file bakes an absolute `file://` path to its media, so
    # parallel dirs coexist cleanly.
    output_media_dir = os.path.join(output_dir, f'media_{world_name}')
    output_textures_dir = os.path.join(output_media_dir, 'materials', 'textures')
    os.makedirs(output_models_dir, exist_ok=True)
    os.makedirs(output_textures_dir, exist_ok=True)

    heightmap_output_path = os.path.join(output_media_dir, 'heightmap.png')
    # texture_source_path is what the cloud-mask + texture-copy steps read.
    # It's either the user-supplied orthophoto or the downloaded-tile mosaic.
    if texture_file is not None:
        texture_source_path = os.path.abspath(texture_file)
    else:
        texture_source_path = os.path.join(texture_cache_dir, 'satellite_texture.png')
    texture_cache_png = texture_source_path  # kept for downstream name parity
    # The mask + copy pipeline reads from the cache PNG directly; only
    # the copy destination picks up the user's chosen texture_format.
    # PNG on disk is lossless but ~10x larger than a q90 JPEG and
    # correspondingly slower for Gazebo to read + decode at world
    # load. JPEG is the default because the satellite texture is a
    # smooth-gradient backdrop where JPEG artefacts are invisible.
    _texfmt = (texture_format or 'jpeg').lower()
    if _texfmt not in ('png', 'jpeg', 'jpg'):
        raise ValueError(f"Unsupported texture_format={texture_format!r}; "
                         f"expected 'png' or 'jpeg'.")
    _texext = 'png' if _texfmt == 'png' else 'jpg'
    texture_output_path = os.path.join(
        output_textures_dir, f'satellite_texture.{_texext}'
    )

    _check_cancel()
    _log("Processing DEM into heightmap...")
    _pct(65)
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
    # If a sample lands on a nodata edge pixel, fall back to dem_stats['min']
    # so the asset sits at ground level rather than at z = -32768 * z_per_meter.
    #
    # Load the DEM once into numpy so we don't pay a gdal.Open + 1x1 read
    # per asset (thousands of buildings/trees/roads -> 10-60 s otherwise).
    dem_sampler = elevation_processor.open_dem_sampler(
        dem_utm_cache_path, nodata_fallback=dem_stats['min']
    )

    def sample_terrain_z(gx, gy):
        utm_x, utm_y = converter.gazebo_to_utm((gx, gy))
        raw_elev = dem_sampler(utm_x, utm_y)
        return (raw_elev - dem_stats['min']) * z_per_meter + terrain_z_offset

    # Build a cloud mask from the cropped-to-exact-bbox satellite texture so
    # asset processors can skip placements in regions the satellite couldn't
    # verify. Disable via cloud_filter=False if your imagery is cloud-free or
    # you want every OSM feature placed regardless of visual coverage.
    cloud_mask = None
    foliage_mask = None
    texture_bbox = _calculate_bounds_wgs84(origin_location, radius)
    # Texture was reprojected to UTM, so pixels are truly meter-spaced.
    # Let the masks scale their morphology kernels by meters-per-pixel so
    # thresholds behave the same at any zoom level.
    texture_meters_per_px = None
    if os.path.exists(texture_cache_png):
        try:
            from PIL import Image as _Img
            with _Img.open(texture_cache_png) as _tex:
                tex_w = _tex.size[0]
            texture_meters_per_px = (2.0 * radius) / tex_w if tex_w > 0 else None
        except Exception:
            texture_meters_per_px = None
    if cloud_filter and os.path.exists(texture_cache_png):
        cloud_mask = cloud_mask_mod.build_cloud_mask(
            texture_cache_png, texture_bbox,
            meters_per_pixel=texture_meters_per_px,
        )

    # Build the foliage mask (image canopy + OSM positives - roads/parking/
    # buildings) BEFORE the tree processor so its image-based scatter path
    # can consult the precomputed mask instead of the legacy bare-EXG
    # heuristic. Mode "off" keeps the old code path in tree_processor.
    if (foliage_mask_mode == 'rgb-osm' and os.path.exists(texture_cache_png)):
        foliage_mask = foliage_mask_mod.build_foliage_mask(
            texture_cache_png, texture_bbox,
            world_half_extent_m=radius,
            converter=converter,
            roads_geojson=roads_cache_path,
            parking_geojson=parking_cache_path,
            positive_osm_geojson=trees_cache_path,
            buildings_geojson=buildings_cache_path,
            meters_per_pixel=texture_meters_per_px,
        )

    _check_cancel()
    _log("Building Gazebo models from OSM footprints...")
    _pct(75)
    buildings = building_processor.process_osm_buildings_to_sdf(
        buildings_cache_path, output_models_dir, origin_location,
        elevation_sampler=sample_terrain_z,
        cloud_mask=cloud_mask,
    )
    _check_cancel()
    _log("Scattering trees from OSM foliage + vegetation mask...")
    _pct(85)
    trees = tree_processor.process_osm_trees_to_sdf(
        trees_cache_path, output_models_dir, origin_location,
        elevation_sampler=sample_terrain_z,
        cloud_mask=cloud_mask,
        world_half_extent_m=radius,
        # Image-based vegetation fill: scatter additional trees on green
        # satellite pixels OSM didn't tag. Needs the UTM-reprojected texture
        # (pixel grid == UTM meter grid) and the buildings geojson (exclude
        # footprints so trunks don't land inside walls).
        satellite_texture_path=texture_cache_png,
        buildings_geojson_path=buildings_cache_path,
        foliage_style=foliage_style,
        # When present, foliage_mask overrides the legacy bare-EXG heuristic
        # inside _scatter_on_vegetation. Roads, parking, buildings, and
        # smooth-grass rejection are all baked into the mask already.
        foliage_mask=foliage_mask,
    )
    if with_roads:
        _check_cancel()
        _log("Laying down roads from OSM highways...")
        _pct(90)
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
    # Historical name — file now carries the heightmap-derived normal
    # map, not a flat 4x4 stand-in. Keeping the filename stable so
    # existing world SDFs + downstream packaging continue to resolve.
    flat_normal_output_path = os.path.join(output_textures_dir, 'flat_normal.png')
    if os.path.exists(texture_source_path):
        _log(f"Copying orthophoto / satellite texture as {_texext.upper()}...")
        os.makedirs(os.path.dirname(texture_output_path), exist_ok=True)
        if _texext == 'png':
            import shutil
            shutil.copy2(texture_source_path, texture_output_path)
        else:
            # JPEG path: re-encode the source PNG/GeoTIFF into a
            # Gazebo-friendly .jpg. Quality 90 is visually lossless on
            # satellite imagery and cuts disk / upload size ~10x vs PNG.
            with Image.open(texture_source_path) as _tex:
                if _tex.mode != 'RGB':
                    _tex = _tex.convert('RGB')
                _tex.save(texture_output_path, format='JPEG',
                          quality=90, optimize=True)
        # Derive a tangent-space normal map from the heightmap gradient
        # instead of emitting a flat 4x4 RGB(128,128,255) stand-in. Pure
        # generation-time compute; Gazebo gets proper directional
        # shading on slopes without any runtime cost. gz-sim's SDF
        # parser still requires the <normal> child to be a real file,
        # so this replaces the placeholder in-place.
        if os.path.exists(heightmap_output_path):
            extent_meters = 2.0 * radius
            elevation_processor.write_heightmap_normal_map(
                heightmap_output_path,
                flat_normal_output_path,
                extent_meters=extent_meters,
                height_amplitude_m=height_amplitude,
            )
        else:
            Image.new('RGB', (4, 4), (128, 128, 255)).save(flat_normal_output_path)

    # Save the cloud mask alongside the heightmap for debug / visualization.
    if cloud_mask is not None:
        cloud_mask.save_debug_png(os.path.join(output_media_dir, 'cloud_mask.png'))
    if foliage_mask is not None:
        foliage_mask.save_debug_png(os.path.join(output_media_dir, 'foliage_mask.png'))

    _check_cancel()
    _log("Rendering SDF world...")
    _pct(95)
    builder = sdf_builder.SDFWorldBuilder(TEMPLATE_DIR)
    extent_meters = 2.0 * radius
    output_sdf_world_path = os.path.join(output_dir, f"{world_name}.world")
    sdf_content = builder.render_world_template(
        heightmap_path=heightmap_output_path if os.path.exists(heightmap_output_path) else None,
        texture_path=texture_output_path if os.path.exists(texture_output_path) else None,
        flat_normal_path=flat_normal_output_path if os.path.exists(flat_normal_output_path) else None,
        buildings=buildings,
        trees=trees,
        roads=roads,
        extent_meters=extent_meters,
        height_amplitude=height_amplitude,
        terrain_z_offset=terrain_z_offset,
        performer_ref=performer_ref,
        enable_level_streaming=not disable_level_streaming,
        foliage_style=foliage_style,
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
@click.option('--side-length', 'side_length', type=float, default=None,
              help='Full side length of the generated square world in meters '
                   '(e.g. --side-length 2000 -> 2000 m x 2000 m terrain).')
@click.option('--radius', type=float, default=None,
              help='DEPRECATED: half-extent. --radius 1000 produces the same '
                   '2000 m x 2000 m world as --side-length 2000. Prefer '
                   '--side-length for new usage.')
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
@click.option('--zoom', 'tile_zoom', type=int, default=None,
              help='Force satellite tile zoom level (clamped to provider.max_zoom). '
                   'Without this, the pipeline picks the highest zoom that fits --max-tiles. '
                   'Higher zoom = finer texture + more tiles + longer download. '
                   'Esri/Bing cap at 19, MapTiler at 20, Mapbox at 22.')
@click.option('--max-tiles', 'tile_max_count', type=int, default=None,
              help='Cap on tile count per world (default 4096). Pipeline steps zoom down '
                   'until the count fits. Raise if you want a 3 km+ world at zoom 19; '
                   'lower if the tile server rate-limits.')
@click.option('--max-texture-size', 'max_texture_px', type=int, default=None,
              help='Cap on the saved satellite texture\'s larger dimension in pixels '
                   f'(default {textures.DEFAULT_MAX_TEXTURE_PX}). Mosaics exceeding this '
                   'are downsampled (LANCZOS) before save. Prevents Gazebo OOM on '
                   '≤4 GB VRAM GPUs and keeps world-load time bounded. Raise to preserve '
                   'native tile detail on big-VRAM workstations; lower for Jetsons/laptops.')
@click.option('--texture-format',
              type=click.Choice(['jpeg', 'png'], case_sensitive=False),
              default='jpeg',
              help='Satellite texture file format written into '
                   'media_<world>/materials/textures/. "jpeg" (default, quality 90) '
                   'is ~10x smaller on disk and decodes correspondingly faster in '
                   'Gazebo — the satellite texture is a smooth-gradient backdrop '
                   'where JPEG artefacts are invisible. "png" keeps the texture '
                   'lossless (relevant only for sharp-edged orthophotos supplied '
                   'via --texture-file).')
@click.option('--with-roads/--no-roads', default=False,
              help='Emit OSM highway ways as flat road strips. Off by default — '
                   'current implementation is flat-per-segment and floats over undulating terrain.')
@click.option('--cloud-filter/--no-cloud-filter', default=True,
              help='Drop buildings/trees whose satellite pixel looks like cloud '
                   '(high luminance + low saturation). On by default.')
@click.option('--dem-file', type=click.Path(), default=None,
              help='User-supplied DEM (any GDAL-readable raster, any CRS). Overrides SRTM3 '
                   'download. Useful for high-resolution LIDAR / aerial flyover DEMs that '
                   'have much better detail than SRTM3 (90 m/px). Must cover the '
                   '(2R x 2R) meter bbox around the origin; will be clipped + reprojected.')
@click.option('--texture-file', type=click.Path(), default=None,
              help='User-supplied orthorectified image (PNG/JPEG/GeoTIFF). Overrides the '
                   'satellite tile download. Dimensions should roughly match the DEM extent; '
                   'Gazebo scales it to the heightmap size regardless.')
@click.option('--max-heightmap-size', type=int, default=1025,
              help='Cap on heightmap PNG dimensions (must be one of 65, 129, 257, 513, '
                   '1025, 2049, 4097 — all 2^n+1). Default 1025. Raise to 2049 or 4097 '
                   'to preserve sub-meter detail from aerial flyovers; each step quadruples '
                   'GPU memory and scene-update cost in Gazebo.')
@click.option('--performer-ref', default='rovermax',
              help='Top-level model name that the level-streaming <performer> follows. '
                   'Set to match the name passed to ros_gz_sim::create when spawning the '
                   'robot. Default "rovermax" matches this workspace\'s ROBOT_NAME.')
@click.option('--disable-level-streaming', is_flag=True, default=False,
              help='Emit tile compound models without <level>/<performer> streaming. '
                   'Result: every tile is loaded at all times (faster to reach a valid '
                   'scene if the performer never spawns; slower full-world loads). Keep '
                   'streaming on for the 3090 + city-scale workflow.')
@click.option('--foliage-style',
              type=click.Choice(['cartoon', 'fuel'], case_sensitive=False),
              default='cartoon',
              help='Tree/foliage rendering. "cartoon" (default): inline primitives '
                   '(trunk cylinder + canopy spheres/cones) baked into each tile '
                   'compound — no external deps, highest per-instance visual diversity '
                   'via per-tree color/size jitter. "fuel": emit each tree as a '
                   'top-level `<include>` of `model://tree_fuel_<variant>` (resolved '
                   'against GZ_SIM_RESOURCE_PATH), so the committed `models_fuel/` '
                   'wrappers can pull real meshes from Gazebo Fuel '
                   '(OpenRobotics/Oak tree + Pine Tree). Fuel downloads land in '
                   '~/.gz/fuel/ (persisted by the gz-cache docker volume on this '
                   'workspace); trees still participate in per-tile level streaming '
                   'via added <ref> entries.')
@click.option('--foliage-mask',
              'foliage_mask_mode',
              type=click.Choice(['off', 'rgb-osm', 'worldcover'], case_sensitive=False),
              default='rgb-osm',
              help='Mask driving the image-based tree-scatter path (independent of '
                   '--foliage-style). "rgb-osm" (default): combine EXG + local '
                   'luminance variance on the satellite texture with an OSM positive '
                   'union (forest/park/scrub/orchard/vineyard/heath/garden) and '
                   'negative subtraction of buildings, road buffers, and parking '
                   'lots. Rejects smooth grass, asphalt, and green rooftops. '
                   '"off": fall back to the legacy bare-EXG heuristic (scatter on '
                   'any green pixel; only buildings excluded). "worldcover" is '
                   'reserved for an ESA WorldCover 10 m tree-cover raster '
                   'integration and currently raises NotImplementedError.')
@click.pass_context
def generate_world(ctx, latitude, longitude, side_length, radius, output_dir,
                   world_name, height_amplitude, tile_provider, tile_api_key,
                   tile_zoom, tile_max_count, max_texture_px, texture_format,
                   with_roads, cloud_filter, dem_file, texture_file,
                   max_heightmap_size, performer_ref, disable_level_streaming,
                   foliage_style, foliage_mask_mode):
    """Generate a Gazebo Harmonic SDF world for a given location.

    Exactly one of ``--side-length`` (full side, preferred) or ``--radius``
    (legacy half-extent) is required. Internally the pipeline works in
    half-extent meters; the conversion is just ``radius = side_length / 2``.
    """
    if side_length is None and radius is None:
        raise click.UsageError("Provide --side-length or --radius.")
    if side_length is not None and radius is not None:
        raise click.UsageError("Use --side-length or --radius, not both.")
    if side_length is not None:
        radius = side_length / 2.0
    if max_heightmap_size not in elevation_processor._OGRE2_VALID_SIZES:
        raise click.UsageError(
            f"--max-heightmap-size must be one of "
            f"{elevation_processor._OGRE2_VALID_SIZES}; got {max_heightmap_size}."
        )
    try:
        world_name = safe_identifier(world_name, field='--world-name')
        performer_ref = safe_identifier(performer_ref, field='--performer-ref')
    except ValueError as e:
        raise click.UsageError(str(e))

    # Validate user-supplied rasters up front so we don't run the full
    # OSM + tile pipeline (minutes of network I/O) before discovering a
    # corrupt or unreadable file.
    if dem_file is not None:
        if not os.path.isfile(dem_file):
            raise click.UsageError(f"--dem-file does not exist: {dem_file}")
        ds = gdal.Open(os.path.abspath(dem_file))
        if ds is None:
            raise click.UsageError(
                f"--dem-file is not a GDAL-readable raster: {dem_file}"
            )
        ds = None
    if texture_file is not None:
        if not os.path.isfile(texture_file):
            raise click.UsageError(f"--texture-file does not exist: {texture_file}")
        try:
            with Image.open(os.path.abspath(texture_file)) as img:
                img.verify()
        except Exception as e:
            raise click.UsageError(
                f"--texture-file is not a readable image ({type(e).__name__}: {e}): "
                f"{texture_file}"
            )

    # --foliage-mask worldcover is reserved but not yet implemented. Fail
    # fast at argument parsing so we don't run the whole DEM+OSM+tile
    # pipeline only to raise from inside run_generate_world.
    if foliage_mask_mode.lower() == 'worldcover':
        raise click.UsageError(
            "--foliage-mask worldcover is reserved for a future ESA WorldCover "
            "10 m tree-cover integration and is not yet implemented. Use "
            "'rgb-osm' (default) or 'off'."
        )

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
            tile_zoom=tile_zoom,
            tile_max_count=tile_max_count,
            max_texture_px=max_texture_px,
            with_roads=with_roads,
            cloud_filter=cloud_filter,
            dem_file=dem_file,
            texture_file=texture_file,
            max_heightmap_size=max_heightmap_size,
            performer_ref=performer_ref,
            disable_level_streaming=disable_level_streaming,
            foliage_style=foliage_style.lower(),
            foliage_mask_mode=foliage_mask_mode.lower(),
            texture_format=texture_format.lower(),
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
