# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Regression tests for the numpy box-mean inside the foliage mask.

The original implementation routed through PIL's BoxBlur on a mode-'F'
image, which raises "image has wrong mode" on Pillow < 11 — the version
ROS 2 Jazzy / Ubuntu 24.04 ships as python3-pil. That exception was
swallowed by build_foliage_mask(), silently degrading the default
rgb-osm mask to the legacy bare-EXG path on the package's primary
target platform. These tests pin the numpy replacement's numerics so a
future swap back to a library filter can't reintroduce the regression.
"""
import numpy as np
import pytest

pytest.importorskip('PIL')

from terraforge.data_processing.foliage_mask import (  # noqa: E402
    _box_mean,
    _local_luminance_sigma,
)


def _naive_box_mean(arr, radius):
    """Compute the reference O(n * w^2) box mean with replicate edges."""
    padded = np.pad(arr.astype(np.float64), radius, mode='edge')
    h, w = arr.shape
    out = np.empty((h, w), dtype=np.float64)
    win = 2 * radius + 1
    for y in range(h):
        for x in range(w):
            out[y, x] = padded[y:y + win, x:x + win].mean()
    return out


def test_box_mean_matches_naive_reference():
    rng = np.random.RandomState(42)
    arr = rng.rand(23, 31).astype(np.float32)
    for window in (3, 5, 9):
        radius = (window - 1) // 2
        fast = _box_mean(arr, window)
        ref = _naive_box_mean(arr, radius)
        assert fast.shape == arr.shape
        assert fast.dtype == np.float32
        np.testing.assert_allclose(fast, ref, rtol=0, atol=1e-5)


def test_box_mean_constant_image_is_identity():
    arr = np.full((16, 16), 0.37, dtype=np.float32)
    out = _box_mean(arr, 7)
    np.testing.assert_allclose(out, arr, atol=1e-6)


def test_box_mean_accepts_float32_input():
    # The Pillow-10 failure mode was an exception on mode-'F' input;
    # guard that plain float32 arrays of any (h, w) just work.
    arr = np.linspace(0.0, 1.0, 12 * 17, dtype=np.float32).reshape(12, 17)
    out = _box_mean(arr, 5)
    assert out.shape == arr.shape
    assert np.isfinite(out).all()


def test_local_luminance_sigma_flat_vs_textured():
    rng = np.random.RandomState(7)
    flat = np.full((32, 32), 0.5, dtype=np.float32)
    textured = (0.5 + 0.25 * rng.randn(32, 32)).astype(np.float32)
    sigma_flat = _local_luminance_sigma(flat, 9)
    sigma_tex = _local_luminance_sigma(textured, 9)
    # Uniform regions must clamp to ~0 (no negative variance from
    # round-off); dappled regions must register clearly above it.
    assert (sigma_flat >= 0).all()
    assert sigma_flat.max() < 1e-4
    assert sigma_tex.mean() > 0.05
