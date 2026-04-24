# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Unit tests for elevation_processor.write_heightmap_normal_map.

The normal map is generated at preprocessing time to give the heightmap
proper directional shading without runtime cost. Verify that flat
terrain maps to (128, 128, 255) (tangent-space up) and that a slope
shifts the corresponding channel in the expected direction.
"""

import pytest

pytest.importorskip('osgeo')
pytest.importorskip('numpy')
pytest.importorskip('PIL')

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from terraforge.data_processing.elevation_processor import (  # noqa: E402
    write_heightmap_normal_map,
)


def _write_heightmap(path, arr_uint16):
    Image.fromarray(arr_uint16.astype(np.uint16), mode='I;16').save(path, format='PNG')


def test_flat_terrain_produces_flat_normals(tmp_path):
    hm = tmp_path / 'hm.png'
    nm = tmp_path / 'nm.png'
    flat = np.full((32, 32), 32768, dtype=np.uint16)
    _write_heightmap(str(hm), flat)
    write_heightmap_normal_map(
        str(hm), str(nm), extent_meters=1000.0, height_amplitude_m=200.0,
    )
    with Image.open(str(nm)) as img:
        arr = np.asarray(img)
    # Interior pixels (avoid the 1-pixel border where np.gradient uses
    # forward/backward differences) should be exactly (128, 128, 255).
    interior = arr[1:-1, 1:-1]
    assert np.all(interior[..., 0] == 128), interior[..., 0]
    assert np.all(interior[..., 1] == 128), interior[..., 1]
    assert np.all(interior[..., 2] == 255), interior[..., 2]


def test_east_sloping_terrain_shifts_nx_channel(tmp_path):
    hm = tmp_path / 'hm.png'
    nm = tmp_path / 'nm.png'
    # Ramp climbs east (world +X). OpenGL normal-map convention: east-
    # rising -> R > 128.
    h = 65
    w = 65
    ramp = np.tile(np.linspace(0, 65535, w, dtype=np.float32), (h, 1))
    _write_heightmap(str(hm), ramp.astype(np.uint16))
    write_heightmap_normal_map(
        str(hm), str(nm), extent_meters=100.0, height_amplitude_m=100.0,
    )
    with Image.open(str(nm)) as img:
        arr = np.asarray(img)
    interior = arr[1:-1, 1:-1]
    assert interior[..., 0].mean() > 136, interior[..., 0].mean()
    assert abs(interior[..., 1].mean() - 128) < 3
    assert interior[..., 2].mean() < 255


def test_north_sloping_terrain_shifts_ny_channel(tmp_path):
    hm = tmp_path / 'hm.png'
    nm = tmp_path / 'nm.png'
    h = 65
    w = 65
    # Rows grow south in PNG coords. Terrain rising NORTH means row 0
    # (top of image) is the high point. Convention: north-rising -> G > 128.
    ramp = np.tile(np.linspace(65535, 0, h, dtype=np.float32)[:, None], (1, w))
    _write_heightmap(str(hm), ramp.astype(np.uint16))
    write_heightmap_normal_map(
        str(hm), str(nm), extent_meters=100.0, height_amplitude_m=100.0,
    )
    with Image.open(str(nm)) as img:
        arr = np.asarray(img)
    interior = arr[1:-1, 1:-1]
    assert interior[..., 1].mean() > 136, interior[..., 1].mean()
    assert abs(interior[..., 0].mean() - 128) < 3


def test_output_is_rgb_png_same_size_as_input(tmp_path):
    hm = tmp_path / 'hm.png'
    nm = tmp_path / 'nm.png'
    _write_heightmap(str(hm), np.full((129, 129), 1000, dtype=np.uint16))
    write_heightmap_normal_map(
        str(hm), str(nm), extent_meters=500.0, height_amplitude_m=80.0,
    )
    with Image.open(str(nm)) as img:
        assert img.mode == 'RGB'
        assert img.size == (129, 129)
