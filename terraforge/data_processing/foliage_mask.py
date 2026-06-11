# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Raster foliage mask for the image-based tree-scatter path.

This is the FoliageMask analogue of cloud_mask.CloudMask. Where CloudMask
identifies pixels the satellite can't "see through", FoliageMask identifies
pixels where it makes sense to place a scatter tree:

  * Green **and textured** (canopy dapples shadow; smooth lawns don't),
    with a morphological seed-and-grow pass to pull shadow pockets inside
    woods into the mask without dragging roads / grass in with them.
  * Unioned with OSM-tagged foliage polygons (wood / forest / scrub /
    orchard / park / heath / vineyard / garden) so tagged areas scatter
    even when seasonal or imagery-quality issues zero out the RGB signal.
  * Subtracted against a negative raster of buildings + road buffers +
    parking polygons so no image-scatter tree lands in a car lot, in
    the middle of a road, or inside a building wall.

Rationale for an RGB-only canopy detector (no NIR / NDVI):
  * Every tile provider in terraforge's textures.py returns RGB only.
    Sentinel-2's NIR isn't accessible through the tile APIs we use.
  * OSM is authoritative for most of the world's parks, so the positive
    OSM union catches a lot of the canopy we care about even when RGB
    thresholds miss. The canopy detector's job is to catch what OSM
    doesn't tag (untagged suburban trees, informal wooded lots).
  * A plain EXG greenness threshold (the previous vegetation heuristic)
    fired on grass, green roofs, and algae. Adding a local luminance
    standard deviation (sigma) check distinguishes the dappling pattern
    of canopy from flat grass — canopies run sigma(L) ≈ 0.06-0.15 at
    native 0.3-1 m/px, lawns sit at sigma(L) ≈ 0.01-0.02.

Threshold tuning notes mirror cloud_mask.py: the constructor logs EXG
and sigma percentiles at each pipeline stage so operators can tune
against real imagery instead of guessing.

Known false-positive / false-negative classes:
  * False positive: bare-earth cornfields at harvest when the stubble
    is strongly textured — falls off once the ground goes uniform.
  * False negative: dense winter canopy with no leaves — mitigated by
    the OSM positive union.
  * False negative: solar panels (dark + textured, low green).
"""

import json
import os
from typing import Optional

import numpy as np
from PIL import Image, ImageDraw, ImageFilter
import shapely.geometry
import shapely.ops

from terraforge.utils.logging import logger
from terraforge.utils.morphology import geodesic_dilate, open_u8
from terraforge.utils.osm_roads import (
    half_width_for_props,
    NARROW_ROAD_CLASSES,
)

# Two-stage canopy thresholds. The strict trio (EXG + sigma + L cap) picks
# "definitely canopy" seeds; the loose pair defines "plausibly vegetation"
# and is only accepted where a strict seed can geodesic-reach it. This keeps
# smooth grass out of the mask even when it's visually green.
DEFAULT_EXG_STRICT = 0.10       # canopy-core Excess-Green Index
DEFAULT_EXG_LOOSE = 0.03        # plausibly-vegetation envelope
DEFAULT_L_MAX_STRICT = 0.55     # canopy is dim; reject bright pavement/sand
DEFAULT_L_MAX_LOOSE = 0.70      # envelope cap; reject white concrete / roofs
DEFAULT_SIGMA_STRICT = 0.035    # local luminance std; grass ≈ 0.015, canopy ≥ 0.05
DEFAULT_SIGMA_WINDOW_PX = 5     # box-blur window for sigma at native resolution

# Morphological opening radius on the strict seeds — kills isolated
# speckle (one-pixel bright greens) before geodesic growth.
DEFAULT_OPENING_PX = 2
# Final dilation on the canopy mask to soften edges so tiles placed right
# at the seam between mask and OSM polygon blend cleanly.
DEFAULT_DILATION_PX = 1

# Road buffer margins (metres). Added to the per-highway class half-width
# from road_processor. tracks/paths pass through woods legitimately so they
# get a smaller margin — the mask still carves a narrow corridor along the
# trail but not a full road-width exclusion zone.
DEFAULT_ROAD_MARGIN_M = 2.0
DEFAULT_TRACK_MARGIN_M = 1.0
DEFAULT_PARKING_MARGIN_M = 1.0
DEFAULT_BUILDING_MARGIN_M = 6.0   # same 6 m buffer tree_processor used to apply

# Target ground sampling for the mask. Same as cloud_mask: a 5 m/px grid
# over a 2 km world is 400x400 = 160 k pixels — box filters finish in <1s
# and cells are far smaller than any canopy clump, road, or parking lot.
DEFAULT_TARGET_MPP = 5.0

# OSM polygon tags treated as authoritative "scatter trees here" signal
# for the positive union. Matches tree_processor._POLYGON_CLASSES keys so
# the mask stays consistent with where OSM scatter actually fires.
_OSM_POSITIVE_TAGS = {
    ('natural', 'wood'),
    ('natural', 'scrub'),
    ('natural', 'heath'),
    ('landuse', 'forest'),
    ('landuse', 'orchard'),
    ('landuse', 'vineyard'),
    ('leisure', 'park'),
    ('leisure', 'garden'),
}


def _box_mean(arr_f32: np.ndarray, window_px: int) -> np.ndarray:
    """Mean of a WxW window around each pixel, replicate edges.

    Implemented with a numpy summed-area table instead of PIL's BoxBlur:
    Pillow only learned to filter 32-bit float ('F') images with BoxBlur in
    11.0, and ROS 2 Jazzy / Ubuntu 24.04 ships python3-pil 10.2 — there the
    BoxBlur call raised "image has wrong mode", which silently degraded the
    whole rgb-osm foliage mask to the legacy bare-EXG path on the package's
    primary target platform. Window semantics match BoxBlur:
    radius = (window-1)//2, so the box spans 2*radius+1 pixels.
    """
    radius = max(1, (window_px - 1) // 2)
    win = 2 * radius + 1
    h, w = arr_f32.shape
    # float64 accumulation: a float32 cumsum over megapixel images loses
    # enough precision that E[L^2] - E[L]^2 goes visibly wrong.
    padded = np.pad(arr_f32.astype(np.float64), radius, mode='edge')
    sat = np.zeros((padded.shape[0] + 1, padded.shape[1] + 1), dtype=np.float64)
    np.cumsum(padded, axis=0, out=padded)
    np.cumsum(padded, axis=1, out=padded)
    sat[1:, 1:] = padded
    sums = (sat[win:win + h, win:win + w] - sat[:h, win:win + w]
            - sat[win:win + h, :w] + sat[:h, :w])
    return (sums / float(win * win)).astype(np.float32)


def _local_luminance_sigma(luminance: np.ndarray, window_px: int) -> np.ndarray:
    """Local standard deviation of luminance in an N-px window.

    sigma = sqrt(E[L^2] - E[L]^2), with both means computed via BoxBlur.
    Clamped at 0 because float round-off can yield tiny negatives inside
    perfectly-uniform regions.
    """
    mean_l = _box_mean(luminance, window_px)
    mean_l2 = _box_mean(luminance * luminance, window_px)
    var = np.maximum(mean_l2 - mean_l * mean_l, 0.0)
    return np.sqrt(var)


def _rasterize_polygons(polys_gazebo, image_size_px, world_half_extent_m,
                        dilate_m: float = 0.0) -> np.ndarray:
    """Rasterize Gazebo-frame polygons into a boolean mask at image resolution.

    Generalization of tree_processor._rasterize_building_mask: takes any
    iterable of shapely Polygon / MultiPolygon in Gazebo metres and stamps
    them into a boolean image at the same UTM-square pixel grid the
    satellite texture uses (image x=0 = world -R, image y=0 = world +R).
    """
    w, h = image_size_px
    img = Image.new('L', (w, h), 0)
    draw = ImageDraw.Draw(img)
    px_per_m = w / (2.0 * world_half_extent_m)

    def g2px(x, y):
        return ((x + world_half_extent_m) * px_per_m,
                (world_half_extent_m - y) * px_per_m)

    for poly in polys_gazebo:
        if poly is None or poly.is_empty:
            continue
        if dilate_m > 0:
            try:
                poly = poly.buffer(dilate_m, resolution=2)
            except Exception:
                pass
        geoms = ([poly] if isinstance(poly, shapely.geometry.Polygon)
                 else list(getattr(poly, 'geoms', [poly])))
        for g in geoms:
            if not isinstance(g, shapely.geometry.Polygon) or g.is_empty:
                continue
            try:
                ext = [g2px(x, y) for x, y in g.exterior.coords]
                draw.polygon(ext, fill=1)
                for interior in g.interiors:
                    draw.polygon([g2px(x, y) for x, y in interior.coords], fill=0)
            except Exception:
                continue
    return np.asarray(img, dtype=bool)


def _load_polygons_gazebo(geojson_path: str, converter, world_box,
                          tag_filter=None) -> list:
    """Load GeoJSON Polygon/MultiPolygon features into Gazebo-frame shapely.

    ``tag_filter`` (optional) is a set of ``(key, value)`` pairs; only features
    whose properties satisfy one of those pairs are returned. ``None`` accepts
    all polygons.
    """
    if not geojson_path or not os.path.exists(geojson_path):
        return []

    def project(lon, lat, z=None):
        gx, gy, _z = converter.wgs84_to_gazebo((lat, lon))
        return (gx, gy) if z is None else (gx, gy, z)

    try:
        with open(geojson_path, encoding='utf-8') as f:
            data = json.load(f)
    except Exception as e:
        logger.warning(f'Foliage mask: could not load {geojson_path}: {e}')
        return []

    polys = []
    for feat in data.get('features', []) or []:
        g = feat.get('geometry')
        if not g or g.get('type') not in ('Polygon', 'MultiPolygon'):
            continue
        if tag_filter is not None:
            props = feat.get('properties', {}) or {}
            if not any(props.get(k) == v for (k, v) in tag_filter):
                continue
        try:
            shape_wgs = shapely.geometry.shape(g)
            shape_local = shapely.ops.transform(project, shape_wgs)
            if not shape_local.is_valid:
                shape_local = shape_local.buffer(0)
            if world_box is not None:
                shape_local = shape_local.intersection(world_box)
            if shape_local.is_empty:
                continue
            if isinstance(shape_local, shapely.geometry.Polygon):
                polys.append(shape_local)
            elif isinstance(shape_local, shapely.geometry.MultiPolygon):
                polys.extend(shape_local.geoms)
        except Exception:
            continue
    return polys


def _rasterize_road_buffers(roads_geojson_path: str, converter, world_box,
                            image_size_px, world_half_extent_m,
                            road_margin_m: float,
                            track_margin_m: float) -> np.ndarray:
    """Buffer OSM road LineStrings by per-class half-width + margin, rasterize.

    Uses per-highway half-widths local to this module (mirrored from
    road_processor so the two files stay independent). LineStrings (one road
    way per feature) are buffered with flat ends (``cap_style=2``) so the
    buffer width along the path matches the road width, not a half-disk at
    each endpoint.
    """
    if not roads_geojson_path or not os.path.exists(roads_geojson_path):
        return np.zeros((image_size_px[1], image_size_px[0]), dtype=bool)

    def project(lon, lat, z=None):
        gx, gy, _z = converter.wgs84_to_gazebo((lat, lon))
        return (gx, gy) if z is None else (gx, gy, z)

    try:
        with open(roads_geojson_path, encoding='utf-8') as f:
            data = json.load(f)
    except Exception as e:
        logger.warning(f'Foliage mask: could not load roads {roads_geojson_path}: {e}')
        return np.zeros((image_size_px[1], image_size_px[0]), dtype=bool)

    buffered = []
    for feat in data.get('features', []) or []:
        g = feat.get('geometry')
        if not g or g.get('type') not in ('LineString', 'MultiLineString'):
            continue
        props = feat.get('properties', {}) or {}
        highway = str(props.get('highway', '')).lower()
        # half_width_for_props handles the OSM <width> override + clamping —
        # same rule road_processor uses, so the mask's road exclusion stays
        # in lockstep with the rendered road widths.
        hw = half_width_for_props(props)
        margin = track_margin_m if highway in NARROW_ROAD_CLASSES else road_margin_m
        total_buffer_m = hw + margin
        try:
            line_wgs = shapely.geometry.shape(g)
            line_local = shapely.ops.transform(project, line_wgs)
            if world_box is not None:
                line_local = line_local.intersection(world_box)
            if line_local.is_empty:
                continue
            buf = line_local.buffer(total_buffer_m, cap_style=2)
            if buf.is_empty:
                continue
            if isinstance(buf, shapely.geometry.Polygon):
                buffered.append(buf)
            elif isinstance(buf, shapely.geometry.MultiPolygon):
                buffered.extend(buf.geoms)
        except Exception:
            continue

    return _rasterize_polygons(buffered, image_size_px, world_half_extent_m)


class FoliageMask:
    """Boolean raster of "OK to image-scatter trees here" cells.

    ``mask[row, col]`` with row 0 at the NORTH edge of the bbox, column 0 at
    the WEST edge — same indexing convention as CloudMask.
    """

    def __init__(self, mask_array: np.ndarray, bbox_wgs84: tuple,
                 world_half_extent_m: float):
        if mask_array.ndim != 2:
            raise ValueError(f'mask must be 2D, got shape {mask_array.shape}')
        self._mask = mask_array.astype(bool)
        self._bbox = bbox_wgs84
        self._half_extent_m = float(world_half_extent_m)
        self.height, self.width = mask_array.shape
        self.foliage_fraction = float(self._mask.mean()) if self._mask.size else 0.0

    @classmethod
    def from_image_and_vectors(
        cls,
        image_path: str,
        bbox_wgs84: tuple,
        *,
        world_half_extent_m: float,
        converter,
        roads_geojson: Optional[str] = None,
        parking_geojson: Optional[str] = None,
        positive_osm_geojson: Optional[str] = None,
        buildings_geojson: Optional[str] = None,
        exg_strict: float = DEFAULT_EXG_STRICT,
        exg_loose: float = DEFAULT_EXG_LOOSE,
        l_max_strict: float = DEFAULT_L_MAX_STRICT,
        l_max_loose: float = DEFAULT_L_MAX_LOOSE,
        sigma_strict: float = DEFAULT_SIGMA_STRICT,
        sigma_window_px: int = DEFAULT_SIGMA_WINDOW_PX,
        opening_px: int = DEFAULT_OPENING_PX,
        dilation_px: int = DEFAULT_DILATION_PX,
        road_margin_m: float = DEFAULT_ROAD_MARGIN_M,
        track_margin_m: float = DEFAULT_TRACK_MARGIN_M,
        parking_margin_m: float = DEFAULT_PARKING_MARGIN_M,
        building_margin_m: float = DEFAULT_BUILDING_MARGIN_M,
        meters_per_pixel: Optional[float] = None,
        target_mpp: float = DEFAULT_TARGET_MPP,
    ):
        """Build a FoliageMask from a UTM-reprojected RGB texture + OSM vectors.

        The texture is downsampled to ``target_mpp`` (default 5 m/px) before
        morphology so the same thresholds behave consistently across tile
        providers and zoom levels. Sigma is computed at NATIVE resolution
        first then MAX-pooled during downsample so canopy dappling survives;
        an average-pool downsample would wash it out against smooth grass.

        ``meters_per_pixel`` is required (not Optional) — the morphology
        + sigma cost is O(window² × pixels) and silently running at native
        resolution is hostile to memory. cli derives it from the cropped
        texture's 2 R / max_dim.
        """
        if meters_per_pixel is None or meters_per_pixel <= 0:
            raise ValueError(
                'meters_per_pixel is required and must be positive; '
                'running foliage-mask sigma + morphology at native '
                'resolution would blow up memory on z18+ inputs.'
            )

        img = Image.open(image_path).convert('RGB')
        native_w, native_h = img.size

        rgb_native = np.asarray(img, dtype=np.float32) / 255.0
        R_n = rgb_native[..., 0]
        G_n = rgb_native[..., 1]
        B_n = rgb_native[..., 2]
        luminance_native = (R_n + G_n + B_n) / 3.0

        # Local sigma at native resolution — catches canopy dappling that
        # would be averaged away if computed post-downsample.
        sigma_native = _local_luminance_sigma(luminance_native, sigma_window_px)

        # Downsample everything to target mpp. EXG / L / envelope use BILINEAR
        # so edges stay smooth; sigma uses MAX so the dappled peaks survive.
        if (meters_per_pixel is not None and meters_per_pixel > 0
                and target_mpp > meters_per_pixel):
            factor = target_mpp / meters_per_pixel
            new_w = max(2, int(round(native_w / factor)))
            new_h = max(2, int(round(native_h / factor)))
            logger.info(
                f'Foliage mask: downsampling {native_w}x{native_h} '
                f'({meters_per_pixel:.3f} m/px) -> {new_w}x{new_h} '
                f'(~{target_mpp:.1f} m/px) for morphology'
            )
            img = img.resize((new_w, new_h), Image.BILINEAR)
            # Max-pool sigma by taking true block-maxes via np.maximum.reduceat.
            # The previous PIL approximation (MaxFilter(~factor) + NEAREST
            # resize) was O(pixels * factor²): ~16 s for a 2545² texture at
            # factor 21, minutes for km-scale worlds. reduceat covers every
            # native pixel exactly once (no window gaps/overlaps) and runs in
            # milliseconds.
            row_starts = np.arange(new_h) * native_h // new_h
            col_starts = np.arange(new_w) * native_w // new_w
            sigma = np.maximum.reduceat(
                np.maximum.reduceat(sigma_native, row_starts, axis=0),
                col_starts, axis=1,
            ).astype(np.float32)
        else:
            sigma = sigma_native
            new_w, new_h = native_w, native_h

        rgb = np.asarray(img, dtype=np.float32) / 255.0
        R = rgb[..., 0]
        G = rgb[..., 1]
        B = rgb[..., 2]
        exg = 2.0 * G - R - B
        luminance = (R + G + B) / 3.0

        # Diagnostic percentiles (skip reprojection-padding black pixels so
        # stats reflect the actual imagery).
        nonblack = luminance > 0.01
        if nonblack.any():
            e50, e90, e99 = np.percentile(exg[nonblack], [50, 90, 99])
            l50, l90, l99 = np.percentile(luminance[nonblack], [50, 90, 99])
            s50, s90, s99 = np.percentile(sigma[nonblack], [50, 90, 99])
        else:
            e50 = e90 = e99 = l50 = l90 = l99 = s50 = s90 = s99 = 0.0

        # STRICT canopy seeds: green AND textured AND not too bright.
        strict_mask = (
            (exg >= exg_strict)
            & (sigma >= sigma_strict)
            & (luminance <= l_max_strict)
        )
        # LOOSE envelope: any plausibly-green pixel (no texture requirement)
        # so canopy-adjacent shadow gets swept in during geodesic growth.
        loose_mask = (exg >= exg_loose) & (luminance <= l_max_loose)

        strict_fraction = float(strict_mask.mean())
        loose_fraction = float(loose_mask.mean())

        strict_u8 = (strict_mask * 255).astype(np.uint8)
        loose_u8 = (loose_mask * 255).astype(np.uint8)

        seeds = open_u8(strict_u8, opening_px)
        seed_fraction = float((seeds > 127).mean())

        grown, iters = geodesic_dilate(seeds, loose_u8)
        grown_fraction = float((grown > 127).mean())

        if dilation_px > 0:
            grown = np.asarray(
                Image.fromarray(grown, mode='L')
                     .filter(ImageFilter.MaxFilter(2 * dilation_px + 1))
            )
        canopy_mask = grown > 127

        # World box in Gazebo metres for clipping OSM features.
        world_box = shapely.geometry.box(
            -world_half_extent_m, -world_half_extent_m,
            world_half_extent_m, world_half_extent_m,
        )
        image_size_px = (new_w, new_h)

        # Positive OSM union: tagged foliage polygons are scatter-authoritative.
        positive_osm_polys = _load_polygons_gazebo(
            positive_osm_geojson, converter, world_box,
            tag_filter=_OSM_POSITIVE_TAGS,
        )
        positive_osm_mask = _rasterize_polygons(
            positive_osm_polys, image_size_px, world_half_extent_m,
        )
        positive_osm_fraction = float(positive_osm_mask.mean()) if positive_osm_mask.size else 0.0

        # Negative mask: buildings + parking polygons + buffered roads.
        building_polys = _load_polygons_gazebo(
            buildings_geojson, converter, world_box,
        )
        building_mask = _rasterize_polygons(
            building_polys, image_size_px, world_half_extent_m,
            dilate_m=building_margin_m,
        )
        parking_polys = _load_polygons_gazebo(
            parking_geojson, converter, world_box,
        )
        parking_mask = _rasterize_polygons(
            parking_polys, image_size_px, world_half_extent_m,
            dilate_m=parking_margin_m,
        )
        road_mask = _rasterize_road_buffers(
            roads_geojson, converter, world_box,
            image_size_px, world_half_extent_m,
            road_margin_m=road_margin_m,
            track_margin_m=track_margin_m,
        )
        negative_mask = building_mask | parking_mask | road_mask
        negative_fraction = float(negative_mask.mean()) if negative_mask.size else 0.0

        # Fuse: (canopy OR positive OSM) AND NOT negative.
        combined = (canopy_mask | positive_osm_mask) & (~negative_mask)

        instance = cls(combined, bbox_wgs84, world_half_extent_m)
        logger.info(
            f'Foliage mask built from {image_path}: '
            f'EXG p50/p90/p99 = {e50:.3f}/{e90:.3f}/{e99:.3f}, '
            f'L p50/p90/p99 = {l50:.3f}/{l90:.3f}/{l99:.3f}, '
            f'sigma p50/p90/p99 = {s50:.3f}/{s90:.3f}/{s99:.3f}. '
            f'Pipeline: strict={strict_fraction:.1%} (EXG>={exg_strict}, '
            f'sigma>={sigma_strict}, L<={l_max_strict}) -> '
            f'{seed_fraction:.1%} after seed-open({opening_px}px) -> '
            f'{grown_fraction:.1%} after geodesic '
            f'(into loose {loose_fraction:.1%}, {iters} iters) + '
            f'{positive_osm_fraction:.1%} OSM positive - '
            f'{negative_fraction:.1%} negative (roads/parking/buildings) -> '
            f'{instance.foliage_fraction:.1%} placeable.'
        )
        return instance

    def is_foliage(self, lat: float, lon: float) -> bool:
        """Lookup by WGS84 — returns True iff the cell at (lat, lon) is placeable."""
        west, south, east, north = self._bbox
        if lon < west or lon > east or lat < south or lat > north:
            return False
        x_frac = (lon - west) / (east - west)
        y_frac = (north - lat) / (north - south)
        x_px = int(x_frac * self.width)
        y_px = int(y_frac * self.height)
        x_px = max(0, min(x_px, self.width - 1))
        y_px = max(0, min(y_px, self.height - 1))
        return bool(self._mask[y_px, x_px])

    def sample_grid(self, target_w: int, target_h: int,
                    world_half_extent_m: float) -> np.ndarray:
        """Resample the mask to a different pixel grid of the same UTM square.

        Used by the tree scatter path to get a boolean mask at the texture's
        native resolution without reimplementing the (lat, lon) lookup per
        pixel. Nearest-neighbor because the mask is boolean.

        Asserts the target grid covers the same world half-extent as the
        one baked in — the mask's indexing logic assumes that.
        """
        if abs(world_half_extent_m - self._half_extent_m) > 1e-3:
            raise ValueError(
                f'FoliageMask.sample_grid extent mismatch: mask built for '
                f'{self._half_extent_m} m, caller asked for {world_half_extent_m} m'
            )
        if target_w == self.width and target_h == self.height:
            return self._mask.copy()
        img = Image.fromarray(self._mask.astype(np.uint8) * 255, mode='L')
        img = img.resize((target_w, target_h), Image.NEAREST)
        return np.asarray(img, dtype=bool)

    def save_debug_png(self, path: str) -> None:
        """Save the mask as a visual debug image (white = placeable)."""
        Image.fromarray((self._mask * 255).astype(np.uint8), mode='L').save(path)


def build_foliage_mask(
    image_path: str, bbox_wgs84: tuple, **kwargs
) -> Optional[FoliageMask]:
    """Build a FoliageMask, returning None if the mask can't be built.

    Lets callers gracefully continue without foliage filtering.
    """
    try:
        return FoliageMask.from_image_and_vectors(image_path, bbox_wgs84, **kwargs)
    except Exception as e:
        logger.warning(f'Foliage mask unavailable ({e}); continuing without filtering')
        return None
