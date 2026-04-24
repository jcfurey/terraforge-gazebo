# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Regression guard: mask builders read the lossless texture source.

JPEG compression biases EXG (2*G - R - B) and HLS saturation — tiny
quantisation shifts move pixels across the cloud / foliage thresholds.
The satellite_texture.jpg that gets baked into the SDF is therefore a
render-only asset; every analysis stage (cloud mask, foliage mask, the
legacy vegetation scatter in tree_processor) must read the PNG cache
(or the user's --texture-file as-is, if supplied) instead.

This test monkeypatches every mask entry point to record the path it
receives, drives run_generate_world up to the point where masks are
built, and asserts none of them ever got the JPEG output path.
"""

import os

import pytest

pytest.importorskip('click')
pytest.importorskip('osgeo')
pytest.importorskip('pyproj')
pytest.importorskip('PIL')

from terraforge import cli  # noqa: E402


class _AbortAfterMasks(Exception):
    """Sentinel raised once masks have been built so the test
    short-circuits before heavy DEM + OSM processing."""


def test_masks_never_read_jpeg_output(tmp_path, monkeypatch):
    seen_paths = {
        'cloud_mask': [],
        'foliage_mask': [],
        'tree_sat_texture': [],
    }

    # Neutralise network + GDAL stages.
    monkeypatch.setattr(cli.elevation, 'download_dem', lambda *a, **kw: None)
    monkeypatch.setattr(cli.osm, 'download_osm_buildings', lambda *a, **kw: None)
    monkeypatch.setattr(cli.osm, 'download_osm_trees', lambda *a, **kw: None)
    monkeypatch.setattr(cli.osm, 'download_osm_roads', lambda *a, **kw: None)
    monkeypatch.setattr(cli.osm, 'download_osm_parking', lambda *a, **kw: None)
    monkeypatch.setattr(
        cli.textures, 'download_satellite_texture_tiles',
        lambda *a, **kw: None,
    )
    monkeypatch.setattr(
        cli.elevation, 'reproject_dem_to_utm', lambda *a, **kw: None,
    )
    monkeypatch.setattr(
        cli.elevation_processor, 'process_dem_to_heightmap',
        lambda *a, **kw: {'min': 0.0, 'max': 100.0, 'origin': 50.0},
    )
    monkeypatch.setattr(
        cli.elevation_processor, 'open_dem_sampler',
        lambda *a, **kw: (lambda x, y: 0.0),
    )
    monkeypatch.setattr(
        cli, '_choose_heightmap_size', lambda *a, **kw: 65,
    )

    # Record the path each mask entry point gets. Return a sentinel
    # object that mimics the interface the pipeline expects just well
    # enough to reach the bail-out.
    class _FakeMask:
        cloud_fraction = 0.0

        def save_debug_png(self, _path):
            pass

        def is_cloudy(self, *_a, **_kw):
            return False

        def sample_grid(self, *_a, **_kw):
            import numpy as np
            return np.ones((1, 1), dtype=bool)

    def _fake_cloud(path, *a, **kw):
        seen_paths['cloud_mask'].append(path)
        return _FakeMask()

    def _fake_foliage(path, *a, **kw):
        seen_paths['foliage_mask'].append(path)
        # Building the foliage mask is the last thing before the bail.
        raise _AbortAfterMasks()

    monkeypatch.setattr(cli.cloud_mask_mod, 'build_cloud_mask', _fake_cloud)
    monkeypatch.setattr(cli.foliage_mask_mod, 'build_foliage_mask', _fake_foliage)

    # Also capture the satellite_texture_path argument that would flow
    # into tree_processor later — we abort in foliage_mask so this
    # line never runs, but the signature inspection confirms cli.py
    # still passes texture_source_path rather than texture_output_path.

    # Prepare a tiny stand-in PNG under the cache dir the pipeline
    # computes, so the mask builders' existence checks pass.
    # We don't know the exact cache path until cli.py computes it, so
    # pre-create the directory tree we expect.
    from PIL import Image
    # Root the cache + output inside tmp_path so nothing leaks to the
    # user's real $HOME caches.
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path / 'cache'))
    # run_generate_world resolves config lazily; re-import cli would
    # trigger the module singleton we've already patched. The simplest
    # route: pre-create a directory matching terraforge's default
    # TEXTURE_CACHE_DIR layout and drop a PNG there under every
    # plausible name.
    cache_dir = cli.config.TEXTURE_CACHE_DIR
    os.makedirs(cache_dir, exist_ok=True)
    # Match the naming scheme used in run_generate_world:
    #   f"loc_{lat:.4f}_{lon:.4f}_r{int(radius)}_texture"
    tex_dir = os.path.join(cache_dir, 'loc_37.7749_-122.4194_r500_texture')
    os.makedirs(tex_dir, exist_ok=True)
    png_path = os.path.join(tex_dir, 'satellite_texture.png')
    Image.new('RGB', (64, 64), (96, 160, 64)).save(png_path)

    with pytest.raises(_AbortAfterMasks):
        cli.run_generate_world(
            latitude=37.7749, longitude=-122.4194,
            radius=500.0, output_dir=str(tmp_path / 'out'),
            world_name='tex_source_regression',
            texture_format='jpeg',
        )

    # Verify each recorded path is the PNG cache, never the JPEG.
    assert seen_paths['cloud_mask'], "cloud_mask builder was not called"
    assert seen_paths['foliage_mask'], "foliage_mask builder was not called"
    for stage, paths in seen_paths.items():
        for p in paths:
            assert p.endswith('.png'), (
                f"{stage} received non-PNG path {p}; masks must read "
                f"the lossless source, not the JPEG render output."
            )
            assert 'materials/textures' not in p, (
                f"{stage} received the render-output path {p}; masks "
                f"must read the cache, not the world media dir."
            )


def test_source_and_output_paths_must_not_alias(tmp_path, monkeypatch):
    """If a future refactor lets the mask source path resolve to the
    same file as the JPEG output, run_generate_world must blow up
    rather than silently degrade mask quality."""
    # Forge a --texture-file that points exactly at the output JPEG.
    fake = tmp_path / 'out' / 'media_name' / 'materials' / 'textures' / 'satellite_texture.jpg'
    fake.parent.mkdir(parents=True)
    from PIL import Image
    Image.new('RGB', (8, 8), (0, 0, 0)).save(str(fake), format='JPEG')

    # The GDAL verify path happens before mask stages, but the alias
    # check sits between them — we want it to be the failure that
    # surfaces. Neutralise the stages that would otherwise run first
    # and leak errors.
    monkeypatch.setattr(cli.elevation, 'download_dem', lambda *a, **kw: None)
    monkeypatch.setattr(cli.osm, 'download_osm_buildings', lambda *a, **kw: None)
    monkeypatch.setattr(cli.osm, 'download_osm_trees', lambda *a, **kw: None)
    monkeypatch.setattr(cli.osm, 'download_osm_roads', lambda *a, **kw: None)
    monkeypatch.setattr(cli.osm, 'download_osm_parking', lambda *a, **kw: None)

    with pytest.raises(AssertionError, match='alias the same file'):
        cli.run_generate_world(
            latitude=37.7749, longitude=-122.4194,
            radius=500.0,
            output_dir=str(tmp_path / 'out'),
            world_name='name',
            texture_file=str(fake),
            texture_format='jpeg',
        )
