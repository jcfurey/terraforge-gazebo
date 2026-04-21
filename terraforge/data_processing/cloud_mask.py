"""Heuristic cloud detection on a satellite tile mosaic.

Clouds in visible-light imagery have two reliable signatures:

  * high luminance (L above ~0.75 in HLS, higher for crisp clouds)
  * low saturation (S below ~0.20 in HLS)

A pixel that is both bright *and* desaturated is almost certainly cloud — or
snow, which we treat the same. We build a boolean mask from the cropped
satellite texture and dilate a few pixels to absorb cloud-edge halos, then
expose an ``is_cloudy(lat, lon)`` lookup so asset processors can skip
placements in those zones.

Threshold tuning notes:
  * Esri / Sentinel-2 composites tone-map clouds grayer than raw imagery,
    so most cloud pixels land at L ≈ 0.70-0.85 rather than near 1.0.
  * We keep the morphological opening (default 5 px) to filter out
    individual bright concrete rooftops (~5-10 px at zoom 15) while
    preserving real cloud regions (usually 30+ px contiguous).
  * If you see legitimate buildings being dropped, raise ``l_min`` (e.g.
    0.82). If large clouds are still missed, lower ``l_min`` toward 0.70
    and/or raise ``s_max`` toward 0.25.

The constructor logs L/S percentiles and the mask fraction at each stage so
tuning is evidence-based rather than a guessing game.

Known false-positive classes (material looks cloud-like):
  * Bright concrete rooftops / parking lots
  * Sand / dry lake beds
  * Building cores in very new satellite scenes with high glint
"""

from typing import Optional

import numpy as np
from PIL import Image, ImageFilter

from terraforge.utils.logging import logger

# Two-stage thresholds. The strict pair detects "definitely cloud" pixels
# (bright cloud cores) — we use these as seeds. The loose pair defines
# "could be cloud" — pixels we'll accept *only if* connected to a strict
# seed via geodesic dilation. This catches the dim halo around a real cloud
# without sweeping in unrelated bright ground (parking lots, sand) that has
# no nearby strict seed.
DEFAULT_L_MIN = 0.75      # strict: definitely-cloud luminance (HLS)
DEFAULT_S_MAX = 0.20      # strict: definitely-cloud saturation
DEFAULT_L_LOOSE = 0.55    # loose: could-be-cloud luminance (halo / haze)
DEFAULT_S_LOOSE = 0.30    # loose: could-be-cloud saturation
# Morphological opening radius applied to the STRICT seeds before dilation.
# Kills building-sized false-positive seeds (bright concrete rooftops, ~5-10
# px at the mask's working resolution) so they can't seed an unwanted geodesic
# expansion.
DEFAULT_OPENING_PX = 5
# Bridge-severing opening applied AFTER the first geodesic reconstruction.
DEFAULT_BRIDGE_OPENING_PX = 2
# Final dilation applied AFTER both geodesic passes — captures the soft
# outer halo of the cloud that's slightly outside the loose threshold.
DEFAULT_DILATION_PX = 3
# Target ground sampling distance (meters-per-pixel) for the cloud-mask
# morphology pass. Pillow's MaxFilter / MinFilter are O(kernel² × pixels),
# which blows up at z19 native resolution (0.3 m/px, 64 MP). A cloud is
# hundreds of meters across, so ~5 m/px is plenty to detect one, and the
# small pixel kernels above stay comparable across all zoom levels. If the
# caller supplies a finer meters_per_pixel, we downsample the image to this
# target before morphology; if it's already coarser, we leave it alone.
DEFAULT_TARGET_MPP = 5.0
# Cap on geodesic-dilation iterations. Each iteration grows the seed by
# 2 px (5x5 max filter), so 50 iters reaches ~100 px from any seed —
# plenty for a cloud spanning hundreds of pixels in a 500x500 mosaic.
_GEODESIC_MAX_ITERS = 50
_GEODESIC_FILTER_PX = 5  # MaxFilter window size; must be odd.


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
        l_loose: float = DEFAULT_L_LOOSE,
        s_loose: float = DEFAULT_S_LOOSE,
        opening_px: int = DEFAULT_OPENING_PX,
        bridge_opening_px: int = DEFAULT_BRIDGE_OPENING_PX,
        dilation_px: int = DEFAULT_DILATION_PX,
        meters_per_pixel: Optional[float] = None,
        target_mpp: float = DEFAULT_TARGET_MPP,
    ):
        """Build a mask from a satellite PNG that has been precisely cropped
        to ``bbox_wgs84`` (north at image top, west at image left).

        Two-stage geodesic detection with bridge severing:
          1. STRICT mask = (L >= l_min) & (S <= s_max) → cloud cores
          2. Opening on strict seeds removes building-sized false positives
          3. LOOSE mask = (L >= l_loose) & (S <= s_loose) → cloud envelope
          4. First geodesic dilation grows strict seeds into the loose
             envelope. This can leak across narrow bridges (a few pixels
             wide) into adjacent bright regions like concrete roofs.
          5. Opening on the reconstructed region SEVERS those narrow
             bridges (and trims thin halo tendrils).
          6. Second geodesic dilation from the strict seeds into the
             opened reconstruction — keeps only components still connected
             to a strict seed, so disconnected rooftops drop out.
          7. Final dilation captures the soft outer halo just outside the
             loose threshold.
        """
        img = Image.open(image_path).convert("RGB")

        # Downsample to target_mpp before morphology. At z19 native (0.25 m/px
        # over a 2 km world = 8 k × 8 k = 64 MP), a 5 px Pillow opening kernel
        # still takes several minutes. Running at 5 m/px (400 × 400) finishes
        # in <1 s. A cloud spans hundreds of metres; 5 m detection cells are
        # plenty, and the mask's only job is a binary yes/no per building.
        if meters_per_pixel is not None and meters_per_pixel > 0 and target_mpp > meters_per_pixel:
            factor = target_mpp / meters_per_pixel
            new_w = max(2, int(round(img.size[0] / factor)))
            new_h = max(2, int(round(img.size[1] / factor)))
            logger.info(
                f"Cloud mask: downsampling {img.size[0]}x{img.size[1]} "
                f"({meters_per_pixel:.3f} m/px) -> {new_w}x{new_h} "
                f"(~{target_mpp:.1f} m/px) for morphology"
            )
            img = img.resize((new_w, new_h), Image.BILINEAR)

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

        # Diagnostic percentiles (logged below so tuning is evidence-based).
        l_p50, l_p90, l_p99 = np.percentile(luminance, [50, 90, 99])
        s_p50, s_p90, s_p99 = np.percentile(saturation, [50, 90, 99])

        strict_mask = (luminance >= l_min) & (saturation <= s_max)
        loose_mask = (luminance >= l_loose) & (saturation <= s_loose)
        strict_fraction = float(strict_mask.mean())
        loose_fraction = float(loose_mask.mean())

        def _open_u8(arr_u8, r):
            """Morphological opening (erode then dilate) by radius r."""
            if r <= 0:
                return arr_u8
            img = Image.fromarray(arr_u8, mode='L')
            img = img.filter(ImageFilter.MinFilter(2 * r + 1))
            img = img.filter(ImageFilter.MaxFilter(2 * r + 1))
            return np.asarray(img)

        def _geodesic(seed_u8, envelope_u8):
            """Grow ``seed_u8`` iteratively, intersecting each step with
            ``envelope_u8``. Converges when no pixel is added. Returns the
            final uint8 mask and the iteration count for diagnostics."""
            current = seed_u8.copy()
            iters = 0
            for iters in range(1, _GEODESIC_MAX_ITERS + 1):
                dilated = np.asarray(
                    Image.fromarray(current, mode='L')
                         .filter(ImageFilter.MaxFilter(_GEODESIC_FILTER_PX))
                )
                new = np.minimum(dilated, envelope_u8)
                if np.array_equal(new, current):
                    break
                current = new
            return current, iters

        strict_u8 = (strict_mask * 255).astype(np.uint8)
        loose_u8 = (loose_mask * 255).astype(np.uint8)

        # (2) Opening on strict seeds → seeds
        seeds = _open_u8(strict_u8, opening_px)
        seed_fraction = float((seeds > 127).mean())

        # (4) First geodesic growth: seeds → loose envelope
        recon1, iters1 = _geodesic(seeds, loose_u8)
        recon1_fraction = float((recon1 > 127).mean())

        # (5) Bridge-severing opening on the reconstruction
        recon1_open = _open_u8(recon1, bridge_opening_px)
        bridge_cut_fraction = float((recon1_open > 127).mean())

        # (6) Second geodesic growth: re-grow from strict seeds into the
        # bridge-severed envelope. Disconnected components (the false-
        # positive roofs that used to be reached via narrow bridges) are
        # dropped here because no strict seed can reach them.
        recon2, iters2 = _geodesic(seeds, recon1_open)
        recon2_fraction = float((recon2 > 127).mean())

        # (7) Final dilation captures the slightly-outside-loose halo so
        # cloud edges aren't sharp.
        if dilation_px > 0:
            recon2 = np.asarray(
                Image.fromarray(recon2, mode='L')
                     .filter(ImageFilter.MaxFilter(2 * dilation_px + 1))
            )
        mask = recon2 > 127

        instance = cls(mask, bbox_wgs84)
        logger.info(
            f"Cloud mask built from {image_path}: "
            f"L p50/p90/p99 = {l_p50:.2f}/{l_p90:.2f}/{l_p99:.2f}, "
            f"S p50/p90/p99 = {s_p50:.2f}/{s_p90:.2f}/{s_p99:.2f}. "
            f"Pipeline: strict={strict_fraction:.1%} "
            f"(L>={l_min},S<={s_max}) -> "
            f"{seed_fraction:.1%} after seed-open({opening_px}px) -> "
            f"{recon1_fraction:.1%} after geodesic-1 "
            f"(into loose {loose_fraction:.1%}, {iters1} iters) -> "
            f"{bridge_cut_fraction:.1%} after bridge-open({bridge_opening_px}px) -> "
            f"{recon2_fraction:.1%} after geodesic-2 ({iters2} iters) -> "
            f"{instance.cloud_fraction:.1%} after dilate({dilation_px}px)."
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
