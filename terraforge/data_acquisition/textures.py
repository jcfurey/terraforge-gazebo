import math
import os
from dataclasses import dataclass
from io import BytesIO
from typing import Callable, Optional

import requests
from PIL import Image

from terraforge.utils.config import config
from terraforge.utils.logging import logger
from terraforge.data_acquisition.elevation import _calculate_bounds_wgs84

USER_AGENT = "terraforge_gazebo/0.1 (+https://github.com/r3tr056/terraforge-gazebo)"
MAX_TILES = 64
DEFAULT_ZOOM = 15


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

    def tile_url(self, z: int, x: int, y: int, api_key: str = "") -> str:
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
    return "".join(digits)


def _bing_url(z: int, x: int, y: int, api_key: str) -> str:
    return (
        f"https://ecn.t0.tiles.virtualearth.net/tiles/a{_xy_to_quadkey(x, y, z)}.jpeg"
        f"?g=14336&key={api_key}"
    )


PROVIDERS = {
    "mapbox": TileProvider(
        name="mapbox",
        requires_key=True,
        max_zoom=22,
        attribution="(c) Mapbox, (c) OpenStreetMap",
        url_template=(
            "https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/"
            "{z}/{x}/{y}?access_token={key}"
        ),
    ),
    "esri": TileProvider(
        name="esri",
        requires_key=False,
        max_zoom=19,
        attribution="Source: Esri, Maxar, Earthstar Geographics, and the GIS User Community",
        url_template=(
            "https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}"
        ),
    ),
    "sentinel2": TileProvider(
        name="sentinel2",
        requires_key=False,
        max_zoom=18,
        attribution=(
            "Sentinel-2 cloudless 2023 by EOX (CC BY 4.0). "
            "Contains modified Copernicus Sentinel data."
        ),
        url_template=(
            "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless-2023_3857/"
            "default/g/{z}/{y}/{x}.jpg"
        ),
    ),
    "maptiler": TileProvider(
        name="maptiler",
        requires_key=True,
        max_zoom=20,
        attribution="(c) MapTiler (c) OpenStreetMap contributors",
        url_template=(
            "https://api.maptiler.com/tiles/satellite-v2/{z}/{x}/{y}.jpg?key={key}"
        ),
    ),
    "bing": TileProvider(
        name="bing",
        requires_key=True,
        max_zoom=19,
        attribution="(c) Microsoft, Earthstar Geographics",
        url_builder=_bing_url,
    ),
    "usgs_naip": TileProvider(
        name="usgs_naip",
        requires_key=False,
        max_zoom=18,
        attribution="USGS NAIP (public domain). US coverage only.",
        url_template=(
            "https://services.nationalmap.gov/arcgis/rest/services/"
            "USGSNAIPImagery/ImageServer/tile/{z}/{y}/{x}"
        ),
    ),
    "gibs_bluemarble": TileProvider(
        name="gibs_bluemarble",
        requires_key=False,
        max_zoom=8,
        attribution="NASA GIBS BlueMarble_NextGeneration (public domain)",
        url_template=(
            "https://gibs.earthdata.nasa.gov/wmts/epsg3857/best/"
            "BlueMarble_NextGeneration/default/500m/"
            "GoogleMapsCompatible_Level8/{z}/{y}/{x}.jpeg"
        ),
    ),
}

_KEY_ENV_BY_PROVIDER = {
    "mapbox": "MAPBOX_API_KEY",
    "maptiler": "MAPTILER_API_KEY",
    "bing": "BING_MAPS_API_KEY",
}


def _env_var_for(provider: str) -> str:
    return _KEY_ENV_BY_PROVIDER.get(provider, "")


def _default_key_for(provider: str) -> str:
    attr = {
        "mapbox": "MAPBOX_API_KEY",
        "maptiler": "MAPTILER_API_KEY",
        "bing": "BING_MAPS_API_KEY",
    }.get(provider)
    if attr is None:
        return ""
    return getattr(config, attr, "") or ""


def _pick_zoom(bbox_wgs84, max_zoom, max_tiles=MAX_TILES):
    """Choose the highest zoom whose tile count for the bbox stays under max_tiles."""
    west, south, east, north = bbox_wgs84
    cap = min(max_zoom, DEFAULT_ZOOM)
    for zoom in range(cap, 0, -1):
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


def download_satellite_texture_tiles(
    location: tuple,
    radius_meters: float,
    output_dir: str,
    provider: Optional[str] = None,
    api_key: Optional[str] = None,
    # Backwards-compat: older callers passed `mapbox_api_key=`.
    mapbox_api_key: Optional[str] = None,
):
    """Download satellite texture tiles for a location/radius from the chosen provider.

    Args:
        location: (latitude, longitude) in WGS84.
        radius_meters: bounding-box half-width around `location`.
        output_dir: where individual tiles and the merged `satellite_texture.png` land.
        provider: one of `PROVIDERS`. Defaults to `config.SATELLITE_TEXTURE_SOURCE`.
        api_key: API key for providers that require one. Defaults to the env var
            for the chosen provider (e.g. `MAPBOX_API_KEY` for `mapbox`).
        mapbox_api_key: legacy alias for `api_key` when `provider='mapbox'`.
    """
    if provider is None:
        provider = (config.SATELLITE_TEXTURE_SOURCE or "mapbox").lower()
    provider = provider.lower()
    if provider not in PROVIDERS:
        raise ValueError(
            f"Unknown tile provider {provider!r}. Available: {sorted(PROVIDERS)}"
        )
    p = PROVIDERS[provider]

    if api_key is None:
        api_key = mapbox_api_key if mapbox_api_key is not None else _default_key_for(provider)
    if p.requires_key and not api_key:
        env = _env_var_for(provider) or "<unknown>"
        raise RuntimeError(
            f"Provider {provider!r} requires an API key. "
            f"Pass `api_key=` or export {env}."
        )

    logger.info(
        f"Downloading satellite texture tiles for location {location} "
        f"with radius {radius_meters}m to {output_dir} "
        f"via provider {provider!r}"
    )

    bbox_wgs84 = _calculate_bounds_wgs84(location, radius_meters)
    tile_size = 256

    zoom, tile_count = _pick_zoom(bbox_wgs84, max_zoom=p.max_zoom)
    logger.info(
        f"Selected zoom {zoom} ({tile_count} tiles) for radius {radius_meters}m "
        f"(provider max_zoom={p.max_zoom})"
    )

    west, south, east, north = bbox_wgs84
    top_left_tile = _deg2num(north, west, zoom)
    bottom_right_tile = _deg2num(south, east, zoom)
    tiles_x = range(top_left_tile[0], bottom_right_tile[0] + 1)
    tiles_y = range(top_left_tile[1], bottom_right_tile[1] + 1)

    merged_image = Image.new(
        'RGB',
        (
            (tiles_x[-1] - tiles_x[0] + 1) * tile_size,
            (tiles_y[-1] - tiles_y[0] + 1) * tile_size,
        ),
    )

    headers = {"User-Agent": USER_AGENT}
    for x_tile in tiles_x:
        for y_tile in tiles_y:
            tile_url = p.tile_url(zoom, x_tile, y_tile, api_key)
            try:
                response = requests.get(tile_url, headers=headers, stream=True, timeout=30)
                response.raise_for_status()

                tile_image = Image.open(BytesIO(response.content)).convert("RGB")
                x_offset = (x_tile - top_left_tile[0]) * tile_size
                y_offset = (y_tile - top_left_tile[1]) * tile_size
                merged_image.paste(tile_image, (x_offset, y_offset))

                tile_filename = f"tile_{x_tile}_{y_tile}.png"
                tile_output_path = os.path.join(output_dir, tile_filename)
                tile_image.save(tile_output_path)
                logger.debug(f"Downloaded tile {x_tile}_{y_tile} to {tile_output_path}")
            except requests.exceptions.RequestException as e:
                logger.error(f"Error downloading tile {x_tile}_{y_tile}: {e}")
            except Exception as e:
                logger.error(f"Error processing tile {x_tile}_{y_tile}: {e}")

    output_texture_path = os.path.join(output_dir, "satellite_texture.png")
    merged_image.save(output_texture_path)
    logger.info(
        f"Merged satellite texture saved to {output_texture_path} "
        f"(attribution: {p.attribution})"
    )
