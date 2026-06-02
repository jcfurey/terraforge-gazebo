# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Boolean-mask morphology helpers shared between CloudMask and FoliageMask.

Both masks run a strict-seed → opening → loose-envelope geodesic dilation
pipeline; these two helpers (``open_u8`` and ``geodesic_dilate``) are the
primitives used by every stage. Kept in ``utils/`` so the mask modules
don't duplicate them or import from each other.
"""

import numpy as np
from PIL import Image, ImageFilter

# Geodesic dilation iteration cap. Each iteration grows the seed by 2 px
# (5x5 max filter), so 50 iters reaches ~100 px from any seed — plenty for
# a feature spanning hundreds of pixels in a 500x500 mosaic.
GEODESIC_MAX_ITERS = 50
GEODESIC_FILTER_PX = 5  # MaxFilter window size; must be odd.


def open_u8(arr_u8: np.ndarray, r: int) -> np.ndarray:
    """Morphological opening (erode then dilate) by radius ``r`` pixels.

    Returns ``arr_u8`` unchanged when ``r <= 0`` so callers can pass a
    config value directly without guarding zero.
    """
    if r <= 0:
        return arr_u8
    img = Image.fromarray(arr_u8, mode='L')
    img = img.filter(ImageFilter.MinFilter(2 * r + 1))
    img = img.filter(ImageFilter.MaxFilter(2 * r + 1))
    return np.asarray(img)


def geodesic_dilate(seed_u8: np.ndarray, envelope_u8: np.ndarray):
    """Grow ``seed_u8`` iteratively, intersecting each step with
    ``envelope_u8``. Converges when no pixel is added.

    Returns ``(final_mask_u8, iters_used)``. The iter count is exposed for
    diagnostic logging — callers can tell whether they hit the
    GEODESIC_MAX_ITERS cap (rare, only on enormous connected regions).
    """
    current = seed_u8.copy()
    iters = 0
    for iters in range(1, GEODESIC_MAX_ITERS + 1):
        dilated = np.asarray(
            Image.fromarray(current, mode='L')
                 .filter(ImageFilter.MaxFilter(GEODESIC_FILTER_PX))
        )
        new = np.minimum(dilated, envelope_u8)
        if np.array_equal(new, current):
            break
        current = new
    return current, iters
