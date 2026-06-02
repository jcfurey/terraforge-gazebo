# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
import concurrent.futures
from dataclasses import dataclass
from io import BytesIO
import math
import os
import threading
from typing import Callable, Optional

from PIL import Image
import requests

from terraforge.data_acquisition.elevation import _calculate_bounds_wgs84
from terraforge.utils.config import config
from terraforge.utils.logging import logger
from terraforge.utils.retry import retry_call

USER_AGENT = 'terraforge_gazebo/0.1 (+https://github.com/r3tr056/terraforge-gazebo)'
# Upper bound on tile count per world. The pipeline steps zoom DOWN from the
# provider's max until the count fits. Raised from 64 so city-scale worlds
# (2-3 km) can still pull at zoom 18-19. At lat 32° with 2 km side length,
# zoom 19 is ~1000 tiles, zoom 18 is ~250; 4096 leaves room for larger worlds.
MAX_TILES = 4096

# Default cap on the saved satellite texture's larger dimension. A 16k x 16k
# RGB mipmap chain costs ~1 GB of GPU memory and OOMs cards with <= 4 GB VRAM
# (common on laptops/Jetsons). 8192 fits comfortably in 1 GB and keeps
# ~0.5 m/px detail on a 4 km world. Override via --max-texture-size.
DEFAULT_MAX_TEXTURE_PX = 8192

# Default concurrency for tile downloads. 8 keeps us polite to most tile
# servers (Mapbox, MapTiler, Bing publish per-IP rate limits but typically
# allow bursts well above this) while still giving a 6-8x wall-clock speedup
# over serial on a 500-tile mosaic. Override via download_satellite_texture_tiles.
DEFAULT_TILE_WORKERS = 8

# Web Mercator (EPSG:3857) is only defined within ±arctan(sinh(π)) ≈ ±85.0511°.
# Past that, _deg2num / _deg2pixel call math.tan(lat) and 1/cos(lat) on values
# that overflow or hit a divide-by-zero at exactly ±90°. Reject bboxes that
# touch the polar regions outright rather than producing a silently corrupt
# mosaic.
MAX_WEBMERCATOR_LAT = 85.05112877980659


@dataclass(frozen=True)
class TileProvider:
    name: str
    requires_key: bool
    max_zoom: int
    attribution: str
    # Exactly one of these is set. `url_template` uses {z}, {x}, {y}, {key}.
    # `url_builder` is for non-XYZ schemes (e.g. Bing's QuadKey).
    url_template: Optional[str] = None
    url_builder: Optional[Callable[[int, int, int, str], str]] = None

    def tile_url(self, z: int, x: int, y: int, api_key: str = '') -> str:
        if self.url_builder is not None:
            return self.url_builder(z, x, y, api_key)
        return self.url_template.format(z=z, x=x, y=y, key=api_key)


def _xy_to_quadkey(x: int, y: int, z: int) -> str:
    # https://learn.microsoft.com/en-us/bingmaps/articles/bing-maps-tile-system
    digits = []
    for i in range(z, 0, -1):
        digit = 0
        mask = 1 << (i - 1)
        if (x & mask) != 0:
            digit += 1
        if (y & mask) != 0:
            digit += 2
        digits.append(str(digit))
    return ''.join(digits)


def _bing_url(z: int, x: int, y: int, api_key: str) -> str:
    return (
        f'https://ecn.t0.tiles.virtualearth.net/tiles/a{_xy_to_quadkey(x, y, z)}.jpeg'
        f'?g=14336&key={api_key}'
    )


PROVIDERS = {
    'mapbox': TileProvider(
        name='mapbox',
        requires_key=True,
        max_zoom=22,
        attribution='(c) Mapbox, (c) OpenStreetMap',
        url_template=(
            'https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/'
            '{z}/{x}/{y}?access_token={key}'
        ),
    ),
    'esri': TileProvider(
        name='esri',
        requires_key=False,
        max_zoom=19,
        attribution='Source: Esri, Maxar, Earthstar Geographics, and the GIS User Community',
        url_template=(
            'https://server.arcgisonline.com/ArcGIS/rest/services/'
            'World_Imagery/MapServer/tile/{z}/{y}/{x}'
        ),
    ),
    'sentinel2': TileProvider(
        name='sentinel2',
        requires_key=False,
        max_zoom=18,
        attribution=(
            'Sentinel-2 cloudless 2023 by EOX (CC BY 4.0). '
            'Contains modified Copernicus Sentinel data.'
        ),
        url_template=(
            'https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless-2023_3857/'
            'default/g/{z}/{y}/{x}.jpg'
        ),
    ),
    'maptiler': TileProvider(
        name='maptiler',
        requires_key=True,
        max_zoom=20,
        attribution='(c) MapTiler (c) OpenStreetMap contributors',
        url_template=(
            'https://api.maptiler.com/tiles/satellite-v2/{z}/{x}/{y}.jpg?key={key}'
        ),
    ),
    'bing': TileProvider(
        name='bing',
        requires_key=True,
        max_zoom=19,
        attribution='(c) Microsoft, Earthstar Geographics',
        url_builder=_bing_url,
    ),
    'usgs_naip': TileProvider(
        name='usgs_naip',
        requires_key=False,
        max_zoom=18,
        attribution='USGS NAIP (public domain). US coverage only.',
        url_template=(
            'https://services.nationalmap.gov/arcgis/rest/services/'
            'USGSNAIPImagery/ImageServer/tile/{z}/{y}/{x}'
        ),
    ),
    'gibs_bluemarble': TileProvider(
        name='gibs_bluemarble',
        requires_key=False,
        max_zoom=8,
        attribution='NASA GIBS BlueMarble_NextGeneration (public domain)',
        url_template=(
            'https://gibs.earthdata.nasa.gov/wmts/epsg3857/best/'
            'BlueMarble_NextGeneration/default/500m/'
            'GoogleMapsCompatible_Level8/{z}/{y}/{x}.jpeg'
        ),
    ),
}

_KEY_ENV_BY_PROVIDER = {
    'mapbox': 'MAPBOX_API_KEY',
    'maptiler': 'MAPTILER_API_KEY',
    'bing': 'BING_MAPS_API_KEY',
}


def _env_var_for(provider: str) -> str:
    return _KEY_ENV_BY_PROVIDER.get(provider, '')


def _default_key_for(provider: str) -> str:
    attr = _KEY_ENV_BY_PROVIDER.get(provider)
    if attr is None:
        return ''
    return getattr(config, attr, '') or ''


def _pick_zoom(bbox_wgs84, max_zoom, max_tiles=MAX_TILES, forced_zoom=None):
    """Pick the zoom level for the bbox.

    If ``forced_zoom`` is set, use it (clamped to the provider's max_zoom)
    and bypass the tile-count safety check — the caller owns the tradeoff.
    Otherwise start from the provider's max_zoom and step DOWN until the
    tile count fits under ``max_tiles``.
    """
    west, south, east, north = bbox_wgs84
    if forced_zoom is not None:
        # Floor at zoom 1: zoom 0 gives one global tile (rarely useful) and
        # negatives blow up _deg2num's 2**zoom math. Ceiling at provider's
        # max_zoom: deeper zooms 404.
        zoom = max(1, min(int(forced_zoom), max_zoom))
        tl = _deg2num(north, west, zoom)
        br = _deg2num(south, east, zoom)
        tiles = (br[0] - tl[0] + 1) * (br[1] - tl[1] + 1)
        return zoom, tiles
    for zoom in range(max_zoom, 0, -1):
        tl = _deg2num(north, west, zoom)
        br = _deg2num(south, east, zoom)
        tiles = (br[0] - tl[0] + 1) * (br[1] - tl[1] + 1)
        if tiles <= max_tiles:
            return zoom, tiles
    return 1, 1


def _deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.log(math.tan(lat_rad) + 1 / math.cos(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)


def _deg2pixel(lat_deg, lon_deg, zoom, tile_size=256):
    """Continuous (sub-tile) global pixel coord for a given lat/lon.

    Used to
    crop the merged tile mosaic to the exact requested bbox in pixel space.
    """
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    x_pixel = (lon_deg + 180.0) / 360.0 * n * tile_size
    y_pixel = (
        (1.0 - math.log(math.tan(lat_rad) + 1 / math.cos(lat_rad)) / math.pi) / 2.0
        * n * tile_size
    )
    return x_pixel, y_pixel


def _reproject_webmercator_to_utm(
    src_png_path: str,
    dst_png_path: str,
    bbox_wgs84: tuple,
    utm_crs: str,
    radius_meters: float,
    output_px: int,
):
    """Reproject a Web-Mercator-cropped PNG onto a true UTM meter-square grid.

    The raw merged+cropped satellite PNG is sampled at Web Mercator pixel spacing —
    its N-S extent per meter is 1/cos(lat) larger than E-W. When Gazebo stretches
    that PNG uniformly over a flat UTM-square terrain, buildings drift the further
    you get from wherever the two projections happened to agree.

    Fix: stamp Web Mercator georef onto the source PNG, warp to UTM with exact
    UTM-square bounds, write PNG. After this, 1 pixel == constant meters everywhere
    and building positions align with the texture.

    Args:
        src_png_path:  Web-Mercator PNG, cropped to the WGS84 bbox derived from
                       UTM corners (cx±r, cy±r).
        dst_png_path:  output PNG in UTM.
        bbox_wgs84:    (west, south, east, north), the source PNG's exact extent.
        utm_crs:       target CRS string, e.g. "EPSG:32615".
        radius_meters: half-extent (m) of the desired output UTM square.
        output_px:     output pixel dimension (square).
    """
    from osgeo import gdal
    from pyproj import Transformer

    west, south, east, north = bbox_wgs84

    # Source PNG bounds in Web Mercator meters.
    wgs_to_merc = Transformer.from_crs('EPSG:4326', 'EPSG:3857', always_xy=True)
    ul_mx, ul_my = wgs_to_merc.transform(west, north)
    lr_mx, lr_my = wgs_to_merc.transform(east, south)

    # Target UTM square: WGS84 bbox was built as UTM_center ± r, so converting
    # the WGS84 corners back to UTM recovers those exact coordinates.
    wgs_to_utm = Transformer.from_crs('EPSG:4326', utm_crs, always_xy=True)
    sw_ux, sw_uy = wgs_to_utm.transform(west, south)
    ne_ux, ne_uy = wgs_to_utm.transform(east, north)

    # VRT lets us stamp georef without touching pixel data or writing to disk.
    vrt_path = '/vsimem/terraforge_wm_source.vrt'
    tmp_tif = '/vsimem/terraforge_utm_warped.tif'
    try:
        vrt = gdal.Translate(
            vrt_path,
            src_png_path,
            format='VRT',
            outputSRS='EPSG:3857',
            outputBounds=[ul_mx, ul_my, lr_mx, lr_my],  # [ulx, uly, lrx, lry]
        )
        if vrt is None:
            raise RuntimeError(f'gdal.Translate failed for {src_png_path}')
        vrt = None

        # gdal.Warp's outputBounds is [minX, minY, maxX, maxY].
        warped = gdal.Warp(
            tmp_tif,
            vrt_path,
            format='GTiff',
            dstSRS=utm_crs,
            outputBounds=[sw_ux, sw_uy, ne_ux, ne_uy],
            width=output_px,
            height=output_px,
            resampleAlg='bilinear',
            multithread=True,
        )
        if warped is None:
            raise RuntimeError('gdal.Warp failed')
        warped = None

        translated = gdal.Translate(dst_png_path, tmp_tif, format='PNG')
        if translated is None:
            raise RuntimeError('gdal.Translate to PNG failed')
        translated = None
    finally:
        for p in (vrt_path, tmp_tif):
            try:
                gdal.Unlink(p)
            except Exception:
                pass

    logger.info(
        f'Reprojected texture: EPSG:3857 -> {utm_crs}, '
        f'bounds=[{sw_ux:.1f}, {sw_uy:.1f}, {ne_ux:.1f}, {ne_uy:.1f}] UTM m '
        f'({output_px}x{output_px} px, {(2*radius_meters)/output_px:.3f} m/px)'
    )


def download_satellite_texture_tiles(
    location: tuple,
    radius_meters: float,
    output_dir: str,
    provider: Optional[str] = None,
    api_key: Optional[str] = None,
    # Backwards-compat: older callers passed `mapbox_api_key=`.
    mapbox_api_key: Optional[str] = None,
    zoom: Optional[int] = None,
    max_tiles: int = MAX_TILES,
    utm_crs: Optional[str] = None,
    max_texture_px: int = DEFAULT_MAX_TEXTURE_PX,
    max_workers: int = DEFAULT_TILE_WORKERS,
    progress: Optional[Callable[[str], None]] = None,
):
    """Download satellite texture tiles for a location/radius from the chosen provider.

    Args:
        location: (latitude, longitude) in WGS84. Both ``location[0]`` and the
            bbox derived from ``radius_meters`` must lie within
            ±``MAX_WEBMERCATOR_LAT`` (~85.0511°); polar requests raise.
        radius_meters: bounding-box half-width around `location`.
        output_dir: where individual tiles and the merged `satellite_texture.png` land.
            **Must be unique per (lat, lon, radius)**: the cached merged-mosaic
            filename and the canonical `satellite_texture.png` are keyed only by
            (provider, zoom, projection), so a shared `output_dir` across
            different locations will silently serve stale data on the second
            call. The CLI satisfies this by nesting the texture cache under
            `<TERRAFORGE_TEXTURE_DIR>/loc_<lat>_<lon>_r<radius>_texture/`;
            programmatic callers should follow the same pattern.
        provider: one of `PROVIDERS`. Defaults to `config.SATELLITE_TEXTURE_SOURCE`.
        api_key: API key for providers that require one. Defaults to the env var
            for the chosen provider (e.g. `MAPBOX_API_KEY` for `mapbox`).
        mapbox_api_key: legacy alias for `api_key` when `provider='mapbox'`.
        utm_crs: if provided (e.g. "EPSG:32615"), the merged mosaic is reprojected
            from Web Mercator to this UTM CRS after cropping, so the output PNG's
            pixel grid matches the UTM terrain grid exactly. Without it the PNG
            is left in Web Mercator and will drift against UTM-placed assets.
    """
    if provider is None:
        provider = (config.SATELLITE_TEXTURE_SOURCE or 'mapbox').lower()
    provider = provider.lower()
    if provider not in PROVIDERS:
        raise ValueError(
            f'Unknown tile provider {provider!r}. Available: {sorted(PROVIDERS)}'
        )
    p = PROVIDERS[provider]

    if api_key is None:
        api_key = mapbox_api_key if mapbox_api_key is not None else _default_key_for(provider)
    if p.requires_key and not api_key:
        env = _env_var_for(provider) or '<unknown>'
        raise RuntimeError(
            f'Provider {provider!r} requires an API key. '
            f'Pass `api_key=` or export {env}.'
        )

    logger.info(
        f'Downloading satellite texture tiles for location {location} '
        f'with radius {radius_meters}m to {output_dir} '
        f'via provider {provider!r}'
    )

    bbox_wgs84 = _calculate_bounds_wgs84(location, radius_meters)
    # Web Mercator breaks beyond ±MAX_WEBMERCATOR_LAT (math.tan + 1/cos blow up
    # at exactly ±90° and produce nonsense earlier). Reject before _deg2num
    # silently corrupts the tile mosaic.
    _west, _south, _east, _north = bbox_wgs84
    if abs(_north) > MAX_WEBMERCATOR_LAT or abs(_south) > MAX_WEBMERCATOR_LAT:
        raise RuntimeError(
            f'bbox latitude exceeds Web Mercator limit (±{MAX_WEBMERCATOR_LAT:.4f}°): '
            f'S={_south:.4f}, N={_north:.4f}. Satellite tile providers used here '
            f'are EPSG:3857 and have no defined imagery at the poles.'
        )
    tile_size = 256

    zoom, tile_count = _pick_zoom(
        bbox_wgs84, max_zoom=p.max_zoom, max_tiles=max_tiles, forced_zoom=zoom,
    )
    logger.info(
        f'Selected zoom {zoom} ({tile_count} tiles) for radius {radius_meters}m '
        f'(provider max_zoom={p.max_zoom}, max_tiles={max_tiles})'
    )
    if tile_count > 512:
        logger.warning(
            f'Pulling {tile_count} tiles at zoom {zoom} — expect a ~{tile_count * 0.1:.0f}s '
            f'download and a {tile_count * tile_size * tile_size * 3 // (1024*1024)} MB '
            f'uncropped mosaic. Drop --zoom if the tile server rate-limits.'
        )

    west, south, east, north = bbox_wgs84
    top_left_tile = _deg2num(north, west, zoom)
    bottom_right_tile = _deg2num(south, east, zoom)
    # Near the antimeridian, rounding quirks, or a degenerate bbox can put
    # top_left's tile indices past bottom_right's. `range()` would silently
    # produce an empty sequence and we'd write a 0x0 (or uninitialized black)
    # mosaic without a single tile request. Fail loudly instead.
    if (top_left_tile[0] > bottom_right_tile[0]
            or top_left_tile[1] > bottom_right_tile[1]):
        raise RuntimeError(
            f'Invalid tile extent at zoom {zoom}: top_left={top_left_tile} '
            f'is not north-west of bottom_right={bottom_right_tile} for bbox '
            f'W={west:.4f} S={south:.4f} E={east:.4f} N={north:.4f}. '
            f'Likely an antimeridian-crossing bbox, which is unsupported.'
        )
    tiles_x = range(top_left_tile[0], bottom_right_tile[0] + 1)
    tiles_y = range(top_left_tile[1], bottom_right_tile[1] + 1)

    # Cached final texture. Keyed by provider + zoom + projection so that
    # toggling UTM reprojection or changing either input invalidates cleanly
    # without clobbering caches from other runs.
    proj_tag = 'utm' if utm_crs else 'wm'
    cached_cropped_path = os.path.join(
        output_dir, f'satellite_texture_{provider}_z{zoom}_{proj_tag}.png'
    )
    output_texture_path = os.path.join(output_dir, 'satellite_texture.png')
    if os.path.exists(cached_cropped_path):
        logger.info(
            f'Cache hit: reusing merged texture from {cached_cropped_path}'
        )
        # Copy into the canonical filename so downstream (cloud_mask,
        # foliage_mask, template render) finds it at the same path
        # regardless of provider/zoom.
        import shutil
        shutil.copy2(cached_cropped_path, output_texture_path)
        logger.info(
            f'Merged satellite texture saved to {output_texture_path} '
            f'(attribution: {p.attribution})'
        )
        return

    merged_image = Image.new(
        'RGB',
        (
            (tiles_x[-1] - tiles_x[0] + 1) * tile_size,
            (tiles_y[-1] - tiles_y[0] + 1) * tile_size,
        ),
    )

    headers = {'User-Agent': USER_AGENT}
    failed_tiles = []
    cache_hits = 0
    fetched = 0
    # Reuse a single Session across the pool so keep-alive works and the
    # adapter's connection pool spans every worker, cutting TLS/handshake
    # overhead on big pulls.
    session = requests.Session()
    session.headers.update(headers)

    # Per-tile filenames include provider + zoom so caches from different
    # runs at the same (lat, lon, radius) don't stomp.
    tile_specs = [
        (x_tile, y_tile, os.path.join(
            output_dir, f'tile_{provider}_z{zoom}_{x_tile}_{y_tile}.png'))
        for x_tile in tiles_x
        for y_tile in tiles_y
    ]
    total_tiles = len(tile_specs)

    def _fetch_one(spec):
        x_tile, y_tile, tile_output_path = spec
        if os.path.exists(tile_output_path) and os.path.getsize(tile_output_path) > 0:
            try:
                return (x_tile, y_tile, Image.open(tile_output_path).convert('RGB'),
                        'cache', None)
            except (OSError, Image.UnidentifiedImageError) as e:
                # Truncated/corrupt cache file (partial write, bad bytes) —
                # re-fetch below. Narrowed from a bare ``Exception`` so a
                # genuinely unexpected error surfaces instead of being
                # masked as a routine cache miss.
                logger.warning(
                    f'tile {x_tile}_{y_tile}: corrupt cache ({e}); re-fetching'
                )
        tile_url = p.tile_url(zoom, x_tile, y_tile, api_key)

        def _do():
            r = session.get(tile_url, stream=True, timeout=30)
            r.raise_for_status()
            return r.content

        # Broad ``except`` is intentional in this worker: it runs inside the
        # ThreadPoolExecutor and must convert ANY failure into a 'failed'
        # result so one bad tile can't kill the pool. The error object is
        # propagated back and logged by the collecting loop.
        try:
            content = retry_call(
                _do,
                attempts=3,
                initial_delay=1.0,
                exceptions=(requests.exceptions.RequestException,),
                label=f'tile {x_tile}_{y_tile}',
            )
        except Exception as e:
            return (x_tile, y_tile, None, 'failed', e)
        try:
            img = Image.open(BytesIO(content)).convert('RGB')
            img.save(tile_output_path)
        except Exception as e:
            return (x_tile, y_tile, None, 'failed', e)
        return (x_tile, y_tile, img, 'fetched', None)

    # Paste onto merged_image from the main loop (PIL.Image.paste is NOT
    # thread-safe); fetches run in parallel, we collect and paste here.
    # Report progress roughly every 5 % so the GUI bar moves during long
    # pulls without spamming the log.
    report_every = max(1, total_tiles // 20)
    paste_lock = threading.Lock()  # kept around for any future parallel-paste
    completed = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, max_workers)) as pool:
        for res in pool.map(_fetch_one, tile_specs):
            x_tile, y_tile, img, status, err = res
            completed += 1
            if status == 'cache':
                cache_hits += 1
            elif status == 'fetched':
                fetched += 1
            else:
                logger.error(f'Error on tile {x_tile}_{y_tile}: {err}')
                failed_tiles.append((x_tile, y_tile))
                continue
            with paste_lock:
                x_offset = (x_tile - top_left_tile[0]) * tile_size
                y_offset = (y_tile - top_left_tile[1]) * tile_size
                merged_image.paste(img, (x_offset, y_offset))
            if progress is not None and (completed % report_every == 0
                                         or completed == total_tiles):
                progress(
                    f'Tiles: {completed}/{total_tiles} '
                    f'({cache_hits} cached, {fetched} downloaded, '
                    f'{len(failed_tiles)} failed)'
                )

    total = cache_hits + fetched + len(failed_tiles)
    logger.info(
        f'Tiles: {cache_hits} cached, {fetched} downloaded, '
        f'{len(failed_tiles)} failed (of {total})'
    )

    # A blank PIL RGB canvas defaults to black, so swallowed failures would
    # leave silent black holes in the cropped texture. Abort instead of
    # writing a corrupt mosaic — operator can re-run (the cache is reused).
    if failed_tiles:
        total = len(tiles_x) * len(tiles_y)
        sample = ', '.join(f'{x}_{y}' for x, y in failed_tiles[:5])
        more = f' (+{len(failed_tiles) - 5} more)' if len(failed_tiles) > 5 else ''
        raise RuntimeError(
            f'{len(failed_tiles)} of {total} satellite tiles failed to download '
            f'from provider {provider!r}; aborting to avoid a corrupt mosaic. '
            f'Failed tiles: {sample}{more}. Re-run to retry.'
        )

    # Crop the merged mosaic to the EXACT requested bbox in pixel space.
    # Tile indices snap to the tile grid (always >= the bbox), so the raw
    # mosaic covers slightly more geography than requested. Without this crop
    # Gazebo's <heightmap><texture><size> stretches the larger image onto the
    # smaller terrain extent, visually shifting the texture relative to
    # correctly-placed building/tree poses (which are computed from WGS84->UTM
    # with zero slack).
    tile_origin_px = (tiles_x[0] * tile_size, tiles_y[0] * tile_size)
    tl_px = _deg2pixel(north, west, zoom, tile_size)
    br_px = _deg2pixel(south, east, zoom, tile_size)
    crop_box = (
        max(int(round(tl_px[0] - tile_origin_px[0])), 0),
        max(int(round(tl_px[1] - tile_origin_px[1])), 0),
        min(int(round(br_px[0] - tile_origin_px[0])), merged_image.width),
        min(int(round(br_px[1] - tile_origin_px[1])), merged_image.height),
    )
    # Guard against a degenerate (sub-pixel) bbox at very coarse zoom: a
    # zero-width/height crop makes PIL raise "cannot write empty image" on
    # save. Clamp to at least 1px so the mosaic is always writable.
    _l, _t, _r, _b = crop_box
    crop_box = (_l, _t, max(_r, _l + 1), max(_b, _t + 1))
    cropped = merged_image.crop(crop_box)
    logger.info(
        f'Cropped mosaic to user bbox: '
        f'mosaic={merged_image.size} -> crop={cropped.size} '
        f'({crop_box[2] - crop_box[0]}x{crop_box[3] - crop_box[1]} px)'
    )
    # Cap the texture so it fits in a reasonable GPU memory budget.
    # Gazebo loads the whole PNG as a single Ogre2 texture at world
    # load; a 16k x 16k RGB mipmap chain is ~1 GB of VRAM. Resize down
    # with LANCZOS (sharpest of Pillow's high-quality filters) before
    # the optional UTM warp so both paths share the cap.
    cw, ch = cropped.size
    if max(cw, ch) > max_texture_px:
        scale = max_texture_px / float(max(cw, ch))
        new_w = max(1, int(round(cw * scale)))
        new_h = max(1, int(round(ch * scale)))
        logger.warning(
            f'Satellite texture {cw}x{ch} exceeds max_texture_px '
            f'({max_texture_px}); downsampling to {new_w}x{new_h} before '
            f'save (GPU memory / Gazebo load-time safety). Pass '
            f'--max-texture-size to raise or lower the cap.'
        )
        cropped = cropped.resize((new_w, new_h), Image.LANCZOS)

    if utm_crs:
        # Write the Web-Mercator-cropped mosaic to a staging file, then warp
        # into a true UTM meter-square PNG. Side length matches the cropped
        # input dimensions' geometric mean, so we neither over- nor under-
        # sample the web-mercator grid significantly.
        import math as _math
        wm_staging_path = os.path.join(
            output_dir, f'satellite_texture_{provider}_z{zoom}_wm_staging.png')
        cropped.save(wm_staging_path)
        output_px = int(round(_math.sqrt(cropped.size[0] * cropped.size[1])))
        _reproject_webmercator_to_utm(
            src_png_path=wm_staging_path,
            dst_png_path=output_texture_path,
            bbox_wgs84=bbox_wgs84,
            utm_crs=utm_crs,
            radius_meters=radius_meters,
            output_px=output_px,
        )
        # Cache the reprojected PNG; drop the Web-Mercator staging file.
        import shutil
        shutil.copy2(output_texture_path, cached_cropped_path)
        try:
            os.remove(wm_staging_path)
        except OSError:
            pass
    else:
        # No UTM CRS passed — save the Web-Mercator mosaic as-is. This path
        # keeps legacy behaviour; callers who want aligned texture must pass
        # utm_crs. The PNG will still look correct visually, just drift from
        # building placements the further you are from the origin.
        cropped.save(output_texture_path)
        cropped.save(cached_cropped_path)
    logger.info(
        f'Merged satellite texture saved to {output_texture_path} '
        f'(attribution: {p.attribution})'
    )
