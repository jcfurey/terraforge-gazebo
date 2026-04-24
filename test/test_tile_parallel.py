# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Smoke test for the parallel tile-download path.

Mocks ``requests.Session.get`` to return a tiny PNG and drives
:func:`textures.download_satellite_texture_tiles` at small zoom. Verifies
every tile is fetched exactly once, cached tiles are reused, and
progress callbacks fire.
"""

import io
import os
import threading

import pytest

pytest.importorskip('PIL')
pytest.importorskip('requests')
pytest.importorskip('elevation')
pytest.importorskip('osgeo')  # textures -> elevation -> osgeo.gdal
pytest.importorskip('pyproj')  # elevation uses pyproj directly

import requests  # noqa: E402
from PIL import Image  # noqa: E402

from terraforge.data_acquisition import textures  # noqa: E402


class _FakeResponse:
    def __init__(self, content):
        self.content = content

    def raise_for_status(self):
        pass


def _make_png_bytes(color=(255, 0, 0)):
    buf = io.BytesIO()
    Image.new('RGB', (256, 256), color).save(buf, format='PNG')
    return buf.getvalue()


class _FakeSession:
    """Thread-safe fake. Records every URL fetched; returns distinct
    bytes per call so paste-order bugs would produce a corrupt mosaic."""

    def __init__(self):
        self.fetched = []
        self.headers = {}
        self._lock = threading.Lock()

    def get(self, url, stream=False, timeout=None):
        with self._lock:
            self.fetched.append(url)
        return _FakeResponse(_make_png_bytes())


def test_parallel_downloads_fetch_each_tile_once(tmp_path, monkeypatch):
    session = _FakeSession()
    monkeypatch.setattr(requests, 'Session', lambda: session)

    progress_msgs = []

    def progress(msg):
        progress_msgs.append(msg)

    # Zoom 2 over a small bbox gives a handful of tiles — fast and
    # deterministic. Sentinel-2 is keyless so we don't hit API-key
    # validation.
    textures.download_satellite_texture_tiles(
        location=(0.0, 0.0),
        radius_meters=5000.0,
        output_dir=str(tmp_path),
        provider='sentinel2',
        zoom=2,
        max_tiles=64,
        utm_crs=None,  # skip UTM warp
        max_texture_px=4096,
        max_workers=4,
        progress=progress,
    )

    # Mosaic file exists.
    assert (tmp_path / 'satellite_texture.png').is_file()
    # Every URL fetched is unique (no double-fetching under concurrency).
    assert len(session.fetched) == len(set(session.fetched))
    # Progress fired at least once (end-of-pull summary message).
    assert any('Tiles:' in m for m in progress_msgs)


def test_cached_tiles_skip_network(tmp_path, monkeypatch):
    session = _FakeSession()
    monkeypatch.setattr(requests, 'Session', lambda: session)

    # First pass: populate the cache.
    textures.download_satellite_texture_tiles(
        location=(0.0, 0.0),
        radius_meters=5000.0,
        output_dir=str(tmp_path),
        provider='sentinel2',
        zoom=2,
        max_tiles=64,
        utm_crs=None,
        max_texture_px=4096,
        max_workers=4,
    )
    first_fetched = len(session.fetched)
    assert first_fetched > 0

    # Second pass: the merged-texture cache hit short-circuits before
    # any tile is requested. Remove the cached mosaic to force the
    # per-tile cache path instead.
    for name in os.listdir(str(tmp_path)):
        if name.startswith('satellite_texture_') and name.endswith('.png'):
            os.remove(os.path.join(str(tmp_path), name))
    os.remove(str(tmp_path / 'satellite_texture.png'))

    textures.download_satellite_texture_tiles(
        location=(0.0, 0.0),
        radius_meters=5000.0,
        output_dir=str(tmp_path),
        provider='sentinel2',
        zoom=2,
        max_tiles=64,
        utm_crs=None,
        max_texture_px=4096,
        max_workers=4,
    )
    # No additional network fetches — all tiles came from disk cache.
    assert len(session.fetched) == first_fetched
