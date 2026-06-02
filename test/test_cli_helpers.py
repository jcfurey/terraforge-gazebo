# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Unit tests for the helpers extracted from terraforge.cli.

`_resolve_texture_paths` decides the lossless mask *source* vs the
render-only *output* texture paths (and guards against the two ever
aliasing). `_setup_fuel_wrappers` bootstraps the self-contained Gazebo
Fuel wrapper models for ``--foliage-style fuel``.

terraforge.cli imports the full geospatial stack (gdal, PIL, osmnx,
numpy, shapely, pyproj, jinja2) at module load, so skip this whole
module if any of those is unavailable rather than erroring at collection.
"""

import os

import pytest

try:
    from terraforge.cli import _resolve_texture_paths, _setup_fuel_wrappers
except Exception as exc:  # pragma: no cover - depends on optional geo deps
    pytest.skip(
        f'terraforge.cli unavailable ({exc})', allow_module_level=True
    )


def test_default_format_is_jpeg_output_png_source(tmp_path):
    textures_dir = str(tmp_path / 'textures')
    cache_dir = str(tmp_path / 'cache')
    source, output, ext = _resolve_texture_paths(
        textures_dir, None, cache_dir, 'jpeg'
    )
    assert ext == 'jpg'
    assert source == os.path.join(cache_dir, 'satellite_texture.png')
    assert output == os.path.join(textures_dir, 'satellite_texture.jpg')


def test_png_format_keeps_png_extension(tmp_path):
    textures_dir = str(tmp_path / 'textures')
    cache_dir = str(tmp_path / 'cache')
    _, output, ext = _resolve_texture_paths(
        textures_dir, None, cache_dir, 'png'
    )
    assert ext == 'png'
    assert output.endswith('satellite_texture.png')


def test_user_texture_file_becomes_source(tmp_path):
    user_tex = tmp_path / 'ortho.tif'
    user_tex.write_bytes(b'')
    source, _, _ = _resolve_texture_paths(
        str(tmp_path / 'textures'), str(user_tex), str(tmp_path / 'cache'),
        'jpeg',
    )
    assert source == os.path.abspath(str(user_tex))


def test_invalid_format_raises(tmp_path):
    with pytest.raises(ValueError):
        _resolve_texture_paths(
            str(tmp_path / 'textures'), None, str(tmp_path / 'cache'), 'gif'
        )


def test_source_output_alias_raises(tmp_path):
    # A user-supplied texture that resolves to the render output path must
    # trip the alias safety rail so masks never read the compressed output.
    textures_dir = tmp_path / 'textures'
    alias = textures_dir / 'satellite_texture.png'
    with pytest.raises(AssertionError):
        _resolve_texture_paths(
            str(textures_dir), str(alias), str(tmp_path / 'cache'), 'png'
        )


def test_setup_fuel_wrappers_writes_and_is_idempotent(tmp_path):
    out = str(tmp_path)
    _setup_fuel_wrappers(out)
    models_fuel = os.path.join(out, 'models_fuel')
    assert os.path.isdir(models_fuel)
    wrappers = [
        name for name in os.listdir(models_fuel)
        if os.path.isfile(os.path.join(models_fuel, name, 'model.sdf'))
    ]
    assert wrappers, 'expected at least one tree_fuel_<i> wrapper model'
    # Second call must not raise (write_fuel_wrappers skips existing files).
    _setup_fuel_wrappers(out)
