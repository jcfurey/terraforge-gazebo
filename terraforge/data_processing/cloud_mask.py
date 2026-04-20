"""Heuristic cloud detection on a satellite tile mosaic.

Clouds in visible-light imagery have two reliable signatures:

  * high luminance (L > ~0.82 in HLS)
  * low saturation (S < ~0.18 in HLS)

A pixel that is both bright *and* desaturated is almost certainly cloud — or
snow, which we treat the same. We build a boolean mask from the cropped
satellite texture and dilate a few pixels to absorb cloud-edge halos, then
expose an ``is_cloudy(lat, lon)`` lookup so asset processors can skip
placements in those zones.

Known false-positive classes (material looks cloud-like):
  * Bright concrete rooftops / parking lots
  * Sand / dry lake beds
  * Building cores in very new satellite scenes with high glint

If you see legitimate buildings being dropped, loosen ``l_min`` or raise
``s_max`` in the :class:`CloudMask.from_image` call. Default values chosen to
err on the side of keeping assets (fewer skips, accept some visible clouds).
"""

from typing import Optional

import numpy as np
from PIL import Image, ImageFilter

from terraforge.utils.logging import logger

DEFAULT_L_MIN = 0.85
DEFAULT_S_MAX = 0.15
# Morphological opening radius: erode N pixels before re-dilating. Kills
# building-sized false-positive blobs (bright concrete rooftops typically
# span 5-10 px at zoom 15) while preserving real cloud regions (usually 30+ px).
DEFAULT_OPENING_PX = 5
DEFAULT_DILATION_PX = 3


class CloudMask:
    def __init__(self, mask_array: np.ndarray, bbox_wgs84: tuple):
        """
        Args:
            mask_array: 2D boolean numpy array, True = cloud, indexed as
                ``mask[row, col]`` with row 0 at the NORTH edge of the bbox.
            bbox_wgs84: ``(west, south, east, north)``.
        """
        if mask_array.ndim != 2:
            raise ValueError(f"mask must be 2D, got shape {mask_array.shape}")
        self._mask = mask_array.astype(bool)
        self._bbox = bbox_wgs84
        self.height, self.width = mask_array.shape
        self.cloud_fraction = float(self._mask.mean())

    @classmethod
    def from_image(
        cls,
        image_path: str,
        bbox_wgs84: tuple,
        l_min: float = DEFAULT_L_MIN,
        s_max: float = DEFAULT_S_MAX,
        opening_px: int = DEFAULT_OPENING_PX,
        dilation_px: int = DEFAULT_DILATION_PX,
    ):
        """Build a mask from a satellite PNG that has been precisely cropped
        to ``bbox_wgs84`` (north at image top, west at image left)."""
        img = Image.open(image_path).convert("RGB")
        rgb = np.asarray(img, dtype=np.float32) / 255.0
        maxc = np.max(rgb, axis=-1)
        minc = np.min(rgb, axis=-1)

        # HLS luminance and saturation. Guard the divide so numpy doesn't
        # emit a RuntimeWarning for pixels where denom==0 (pure black/white).
        luminance = (maxc + minc) / 2.0
        delta = maxc - minc
        denom = np.where(luminance < 0.5, (maxc + minc), (2.0 - maxc - minc))
        saturation = np.zeros_like(delta)
        np.divide(delta, denom, out=saturation, where=(denom > 1e-6))

        raw_mask = (luminance >= l_min) & (saturation <= s_max)
        raw_cloud_fraction = float(raw_mask.mean())

        # Pipeline: opening (erode→dilate by opening_px) removes small false
        # positives like bright rooftops; then an extra dilation of dilation_px
        # absorbs the soft halo around real cloud edges.
        mask_img = Image.fromarray((raw_mask * 255).astype(np.uint8), mode='L')
        if opening_px > 0:
            mask_img = mask_img.filter(ImageFilter.MinFilter(2 * opening_px + 1))
            mask_img = mask_img.filter(ImageFilter.MaxFilter(2 * opening_px + 1))
        if dilation_px > 0:
            mask_img = mask_img.filter(ImageFilter.MaxFilter(2 * dilation_px + 1))
        mask = np.asarray(mask_img) > 127

        instance = cls(mask, bbox_wgs84)
        logger.info(
            f"Cloud mask built from {image_path}: "
            f"{raw_cloud_fraction:.1%} raw -> {instance.cloud_fraction:.1%} after "
            f"opening/dilation (L>{l_min}, S<{s_max}, open={opening_px}px, dilate={dilation_px}px)"
        )
        return instance

    def is_cloudy(self, lat: float, lon: float) -> bool:
        west, south, east, north = self._bbox
        if lon < west or lon > east or lat < south or lat > north:
            # Outside the bbox we have no data — default to "not cloudy" so
            # features at the very edge aren't spuriously dropped.
            return False
        x_frac = (lon - west) / (east - west)
        y_frac = (north - lat) / (north - south)
        x_px = int(x_frac * self.width)
        y_px = int(y_frac * self.height)
        x_px = max(0, min(x_px, self.width - 1))
        y_px = max(0, min(y_px, self.height - 1))
        return bool(self._mask[y_px, x_px])

    def save_debug_png(self, path: str) -> None:
        """Save the mask as a visual debug image (white = cloud)."""
        Image.fromarray((self._mask * 255).astype(np.uint8), mode='L').save(path)


def build_cloud_mask(
    image_path: str, bbox_wgs84: tuple, **kwargs
) -> Optional[CloudMask]:
    """Convenience wrapper — returns None if the image can't be loaded so
    callers can gracefully continue without cloud filtering."""
    try:
        return CloudMask.from_image(image_path, bbox_wgs84, **kwargs)
    except Exception as e:
        logger.warning(f"Cloud mask unavailable ({e}); continuing without filtering")
        return None
