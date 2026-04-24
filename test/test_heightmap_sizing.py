# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Unit tests for elevation_processor.next_ogre2_size.

Ogre2's heightmap terrain requires dimensions of the form 2^n + 1. The
helper rounds a source-DEM resolution up to the nearest valid size,
capped by the user's --max-heightmap-size. Regression-test the cap and
the rounding behavior so a silent DEM downsampling can't sneak back in.
"""

import pytest

pytest.importorskip('osgeo')
pytest.importorskip('numpy')

from terraforge.data_processing.elevation_processor import (  # noqa: E402
    _OGRE2_VALID_SIZES,
    next_ogre2_size,
)


@pytest.mark.parametrize('n,expected', [
    (1, 65),
    (65, 65),
    (66, 129),
    (128, 129),
    (129, 129),
    (130, 257),
    (256, 257),
    (257, 257),
    (513, 513),
    (1000, 1025),
    (1025, 1025),
    (1026, 2049),
])
def test_rounds_up_to_valid_size(n, expected):
    assert next_ogre2_size(n, max_size=4097) == expected


def test_caps_at_max_size():
    # A 3000-px DEM with a 1025 cap should clamp at 1025, even though the
    # next valid size (2049) would fit the DEM losslessly.
    assert next_ogre2_size(3000, max_size=1025) == 1025


def test_caps_at_max_size_for_huge_input():
    # Beyond the largest valid size, fall back to the ceiling (clamped).
    assert next_ogre2_size(10000, max_size=4097) == 4097
    assert next_ogre2_size(10000, max_size=1025) == 1025


def test_returned_size_is_always_ogre2_valid():
    for n in (1, 65, 129, 257, 513, 1025, 2049, 4097, 8000):
        for cap in (_OGRE2_VALID_SIZES):
            assert next_ogre2_size(n, max_size=cap) in _OGRE2_VALID_SIZES
