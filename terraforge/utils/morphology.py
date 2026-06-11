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

# Reconstruction-by-dilation per Vincent (1993), "Morphological grayscale
# reconstruction in image analysis", IEEE TIP 2(2): the elementary geodesic
# dilation is a UNIT (3x3) dilation intersected with the envelope, iterated
# to stability. The structuring element must stay 3x3: a larger per-step
# dilation (an earlier revision used 5x5) jumps 1-px-wide background
# corridors in the envelope before the intersection can stop it, connecting
# regions that are not geodesically connected — at the 5 m/px mask
# resolution that meant leaking across any sub-10 m road or stream gap.
GEODESIC_FILTER_PX = 3  # MaxFilter window size; must stay 3 (see above).
# Iteration cap: 3x3 grows 1 px per iteration, so 200 iters reaches 200 px
# (~1 km at 5 m/px) from the nearest seed before truncating — beyond any
# single connected canopy/cloud region a radius-capped world can contain.
# Callers log the iteration count, so hitting the cap is observable.
GEODESIC_MAX_ITERS = 200


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
    """Grow ``seed_u8`` iteratively within ``envelope_u8`` until stable.

    Intersects each step with ``envelope_u8``; converges when no pixel is
    added.

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
