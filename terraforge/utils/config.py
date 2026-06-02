# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.

import os
import tempfile


def _default_cache_root():
    xdg = os.environ.get('XDG_CACHE_HOME')
    if xdg:
        return os.path.join(xdg, 'terraforge')
    home = os.path.expanduser('~')
    if home and home != '~':
        return os.path.join(home, '.cache', 'terraforge')
    return os.path.join(tempfile.gettempdir(), 'terraforge')


class Config:
    ELEVATION_DATA_SOURCE = os.getenv("ELEVATION_DATA_SOURCE", "earthexplorer")
    OSM_DATA_SOURCE = os.getenv("OSM_DATA_SOURCE", "overpass")
    SATELLITE_TEXTURE_SOURCE = os.getenv("SATELLITE_TEXTURE_SOURCE", "mapbox")

    EARTH_EXPLORER_API_KEY = os.getenv("EARTH_EXPLORER_API_KEY", "")
    MAPBOX_API_KEY = os.getenv("MAPBOX_API_KEY", "")
    MAPTILER_API_KEY = os.getenv("MAPTILER_API_KEY", "")
    BING_MAPS_API_KEY = os.getenv("BING_MAPS_API_KEY", "")
    SENTINEL_HUB_API_KEY = os.getenv("SENTINEL_HUB_API_KEY", "")

    def __init__(self):
        cache_root = os.environ.get('TERRAFORGE_CACHE_DIR', _default_cache_root())
        self.DEM_CACHE_DIR = os.environ.get('TERRAFORGE_DEM_DIR', os.path.join(cache_root, 'dem'))
        self.OSM_CACHE_DIR = os.environ.get('TERRAFORGE_OSM_DIR', os.path.join(cache_root, 'osm'))
        self.TEXTURE_CACHE_DIR = os.environ.get(
            'TERRAFORGE_TEXTURE_DIR', os.path.join(cache_root, 'textures')
        )
        os.makedirs(self.DEM_CACHE_DIR, exist_ok=True)
        os.makedirs(self.OSM_CACHE_DIR, exist_ok=True)
        os.makedirs(self.TEXTURE_CACHE_DIR, exist_ok=True)


config = Config()
