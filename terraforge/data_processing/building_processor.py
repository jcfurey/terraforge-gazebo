# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.

import json
import math as _math
import os
import random as _random

import shapely.affinity
import shapely.geometry
import shapely.ops

from terraforge.utils.coordinates import CoordinateConverter
from terraforge.utils.logging import logger
from terraforge.utils.seeding import stable_seed

DEFAULT_BUILDING_HEIGHT = 6.0  # single-story fallback when no signal exists
LEVEL_HEIGHT_M = 3.0  # floor-to-floor height used when OSM gives building:levels

# Type-specific height defaults (meters). Used when OSM doesn't tag explicit
# height/levels. Values chosen to look plausible without overshooting — a
# detached house is ~6 m, an apartment block ~9-12 m, a warehouse ~8 m.
_TYPE_HEIGHT_DEFAULTS = {
    'house':        6.0,
    'detached':     6.0,
    'residential':  6.0,
    'bungalow':     4.5,
    'cabin':        4.5,
    'terrace':      7.0,
    'semidetached_house': 7.0,
    'apartments':   12.0,
    'dormitory':    12.0,
    'hotel':        15.0,
    'commercial':   9.0,
    'retail':       7.0,
    'supermarket':  8.0,
    'office':       15.0,
    'industrial':   9.0,
    'warehouse':    8.0,
    'factory':      10.0,
    'church':       12.0,
    'cathedral':    20.0,
    'chapel':       9.0,
    'school':       9.0,
    'university':   12.0,
    'kindergarten': 6.0,
    'hospital':     12.0,
    'civic':        10.0,
    'government':   12.0,
    'public':       10.0,
    'garage':       3.0,
    'garages':      3.0,
    'shed':         3.0,
    'carport':      3.0,
    'roof':         3.0,
    'greenhouse':   4.0,
    'stable':       5.0,
    'barn':         7.0,
    'silo':         10.0,
    'tower':        20.0,
}

# Random variation on the extrapolated height so a row of same-type
# buildings isn't suspiciously flat-roofed at identical Y. ±15% is enough
# to read as natural without producing wildly wrong heights.
_HEIGHT_JITTER = 0.15


def _sanitize(name):
    return str(name).replace(':', '_').replace('/', '_').replace(' ', '_')


def _area_based_default_height(area_m2: float) -> float:
    """Coarse fallback height from footprint area when OSM has no type tag.

    Larger footprints tend to be taller on average (detached house ~100 m²
    → ~6 m; big-box retail ~5000 m² → ~10 m). Bounded so pathological areas
    don't produce absurd values.
    """
    if area_m2 <= 0:
        return DEFAULT_BUILDING_HEIGHT
    # Log interpolation: area 50 m² → 4 m, 500 → 7 m, 5000 → 12 m.
    h = 4.0 + 2.0 * _math.log10(max(area_m2, 50.0) / 50.0)
    return max(3.0, min(h, 20.0))


def _infer_height(props: dict, area_m2: float = 0.0,
                  rng: _random.Random = None) -> float:
    """Derive a plausible building height from OSM tags, with fallbacks.

    Priority:
      1. ``height`` (explicit meters, may include 'm')
      2. ``building:levels`` * LEVEL_HEIGHT_M (+0.5 m parapet)
      3. ``building`` type default (residential, apartments, warehouse, ...)
      4. Area-based extrapolation (bigger → taller on average)

    Levels 2-4 apply a small random jitter so rows of same-type buildings
    don't all come out at identical elevation.
    """
    # (1) explicit height
    if 'height' in props:
        try:
            return float(str(props['height']).rstrip(' m'))
        except (TypeError, ValueError):
            pass
    # (2) building:levels
    if 'building:levels' in props:
        try:
            levels = max(float(props['building:levels']), 1.0)
            h = levels * LEVEL_HEIGHT_M + 0.5  # include a parapet / roof band
            return _jitter(h, rng)
        except (TypeError, ValueError):
            pass
    # (3) type-specific default
    btype = str(props.get('building', '')).lower()
    if btype in _TYPE_HEIGHT_DEFAULTS:
        return _jitter(_TYPE_HEIGHT_DEFAULTS[btype], rng)
    # (4) area-based
    return _jitter(_area_based_default_height(area_m2), rng)


def _jitter(h: float, rng: _random.Random) -> float:
    """Apply up to ±_HEIGHT_JITTER relative jitter.

    Deterministic given rng.
    """
    if rng is None:
        return h
    return h * (1.0 + rng.uniform(-_HEIGHT_JITTER, _HEIGHT_JITTER))


def _polygon_to_polyline_body_sdf(polygon, name_prefix: str, pose_xyz: tuple,
                                  height: float, color=(0.7, 0.7, 0.7)) -> str:
    """Return `<collision>` + `<visual>` SDF for one building.

    Poses are baked into each child element and element names made unique
    via ``name_prefix``.

    Designed to be dropped into a shared `<link>` that holds every static
    body in a tile — collapses hundreds of per-building links (+ fixed
    joints) down to a single link per tile, which is the single biggest
    world-load speedup available without mesh baking. See
    ``sdf_builder.build_scene_tiles`` for the assembly.
    """
    if not polygon.is_valid or polygon.geom_type != 'Polygon':
        return _polygon_to_box_body_sdf(polygon, name_prefix, pose_xyz, height, color)

    if not polygon.exterior.is_ccw:
        polygon = shapely.geometry.polygon.orient(polygon, sign=1.0)

    coords = list(polygon.exterior.coords)
    if coords[0] == coords[-1]:
        coords = coords[:-1]
    pts = '\n            '.join(f'<point>{x:.3f} {y:.3f}</point>' for x, y in coords)

    minx, miny, maxx, maxy = polygon.bounds
    box_x = max(maxx - minx, 0.1)
    box_y = max(maxy - miny, 0.1)
    box_cx = (minx + maxx) / 2.0
    box_cy = (miny + maxy) / 2.0

    r, g, b = color
    px, py, pz = pose_xyz
    # Collision: axis-aligned bbox expressed in absolute world metres,
    # centered on the building's bbox-center + half-height. Visual:
    # polyline coords are relative to the building's pose, so we apply
    # the pose via <visual><pose>.
    return (
        f"      <collision name='col_{name_prefix}'>\n"
        f'        <pose>{px + box_cx:.3f} {py + box_cy:.3f} '
        f'{pz + height / 2.0:.3f} 0 0 0</pose>\n'
        f'        <geometry><box><size>{box_x:.3f} {box_y:.3f} '
        f'{height:.3f}</size></box></geometry>\n'
        f'      </collision>\n'
        f"      <visual name='vis_{name_prefix}'>\n"
        f'        <pose>{px:.3f} {py:.3f} {pz:.3f} 0 0 0</pose>\n'
        f'        <geometry>\n'
        f'          <polyline>\n'
        f'            {pts}\n'
        f'            <height>{height:.3f}</height>\n'
        f'          </polyline>\n'
        f'        </geometry>\n'
        f'        <material>\n'
        f'          <ambient>{r} {g} {b} 1</ambient>\n'
        f'          <diffuse>{r} {g} {b} 1</diffuse>\n'
        f'          <specular>0.1 0.1 0.1 1</specular>\n'
        f'        </material>\n'
        f'      </visual>'
    )


def _polygon_to_box_body_sdf(polygon, name_prefix: str, pose_xyz: tuple,
                             height: float, color=(0.7, 0.7, 0.7)) -> str:
    """Bounding-box fallback body fragment.

    Used when polyline extrusion
    isn't viable (multi-polygon exterior, self-intersecting, etc.).
    """
    minx, miny, maxx, maxy = polygon.bounds
    size_x = max(maxx - minx, 0.1)
    size_y = max(maxy - miny, 0.1)
    # The polygon was centred on its centroid by the caller. For asymmetric
    # footprints (L-shape, U-shape, …) the centroid is NOT the bbox centre,
    # so the box has to be offset by (box_cx, box_cy) relative to pose_xyz
    # to actually enclose the original polygon. _polygon_to_polyline_body_sdf
    # already does this — keep the two paths consistent.
    box_cx = (minx + maxx) / 2.0
    box_cy = (miny + maxy) / 2.0
    r, g, b = color
    px, py, pz = pose_xyz
    cx_world = px + box_cx
    cy_world = py + box_cy
    cz = pz + height / 2.0
    return (
        f"      <collision name='col_{name_prefix}'>\n"
        f'        <pose>{cx_world:.3f} {cy_world:.3f} {cz:.3f} 0 0 0</pose>\n'
        f'        <geometry><box><size>{size_x:.3f} {size_y:.3f} '
        f'{height:.3f}</size></box></geometry>\n'
        f'      </collision>\n'
        f"      <visual name='vis_{name_prefix}'>\n"
        f'        <pose>{cx_world:.3f} {cy_world:.3f} {cz:.3f} 0 0 0</pose>\n'
        f'        <geometry><box><size>{size_x:.3f} {size_y:.3f} '
        f'{height:.3f}</size></box></geometry>\n'
        f'        <material>\n'
        f'          <ambient>{r} {g} {b} 1</ambient>\n'
        f'          <diffuse>{r} {g} {b} 1</diffuse>\n'
        f'          <specular>0.1 0.1 0.1 1</specular>\n'
        f'        </material>\n'
        f'      </visual>'
    )


_BUILDING_COLORS = {
    'residential': (0.75, 0.70, 0.60),
    'apartments':  (0.70, 0.65, 0.55),
    'commercial':  (0.65, 0.65, 0.75),
    'industrial':  (0.55, 0.55, 0.55),
    'warehouse':   (0.60, 0.55, 0.50),
    'church':      (0.85, 0.80, 0.70),
    'school':      (0.80, 0.75, 0.65),
    'garage':      (0.50, 0.50, 0.50),
}


def _building_color(props: dict) -> tuple:
    b = str(props.get('building', '')).lower()
    return _BUILDING_COLORS.get(b, (0.72, 0.72, 0.72))


def process_osm_buildings_to_sdf(osm_filepath: str, models_dir: str, origin_wgs84: tuple,
                                 elevation_sampler=None, cloud_mask=None):
    """Convert OSM building footprints into per-building Gazebo SDF models.

    Reads ``osm_filepath`` (GeoJSON) and writes one model per building under
    ``models_dir``. The footprint polygon is preserved via SDF ``<polyline>``
    when possible; self-intersecting or multipart polygons fall back to an
    axis-aligned bounding box.

    ``elevation_sampler(gx, gy) -> z_meters`` optionally provides terrain Z at
    the building footprint so the model sits on the ground instead of floating
    at z=0.

    Returns ``[{'model_name': str, 'pose_xy': (x, y), 'pose_z': z}, ...]``.
    """
    logger.info(f'Processing OSM buildings from {osm_filepath} to models in {models_dir}')
    os.makedirs(models_dir, exist_ok=True)

    converter = CoordinateConverter(origin_wgs84)
    # Deterministic RNG seeded by origin coords, so two runs over the same
    # location produce the same height-jitter pattern and diffs stay clean.
    # stable_seed (sha1-based) replaces hash() because Python's hash of
    # tuples-containing-floats is randomized per process — the previous code
    # silently broke the reproducibility this comment promises.
    rng = _random.Random(stable_seed(*origin_wgs84))

    def project(x_lon, y_lat, z=None):
        gx, gy, _ = converter.wgs84_to_gazebo((y_lat, x_lon))
        return (gx, gy) if z is None else (gx, gy, z)

    buildings = []
    # Minimum building footprint to emit. Filters out sheds, outhouses,
    # carport roofs, and bus-shelter tags that OSM counts as "building" but
    # add a lot of collision polylines without mattering for navigation.
    # Raise to cull more aggressively (e.g. 50 m² keeps only houses/larger).
    MIN_BUILDING_AREA_M2 = 15.0
    if not os.path.exists(osm_filepath):
        logger.info('No OSM buildings file; skipping buildings stage.')
        return buildings
    try:
        with open(osm_filepath, 'r', encoding='utf-8') as f:
            osm_data = json.load(f)
        polyline_count = 0
        bbox_fallback = 0
        tiny_skipped = 0

        cloud_skipped = 0
        multipart_split = 0
        invalid_geom_skipped = 0
        feature_errors = 0
        for feature_idx, feature in enumerate(osm_data.get('features', []) or []):
            try:
                geom = feature.get('geometry') or {}
                geom_type = geom.get('type')
                if geom_type not in ('Polygon', 'MultiPolygon'):
                    continue

                props = feature.get('properties') or {}
                building_id = props.get('osmid', f'{feature_idx}')
                color = _building_color(props)

                polygon_wgs84 = shapely.geometry.shape(geom)
                # Skip buildings whose centroid falls under a cloud in the
                # satellite mosaic — we can't visually verify the footprint.
                if cloud_mask is not None:
                    c = polygon_wgs84.centroid
                    if cloud_mask.is_cloudy(c.y, c.x):
                        cloud_skipped += 1
                        continue

                # MultiPolygon buildings are common in OSM (courtyard + wings
                # tagged as one way, industrial compounds, detached garages
                # grouped under a single building=*). Emit each part as its
                # own link so the visuals aren't collapsed to one bounding
                # box. shapely.Polygon.area already subtracts interior holes
                # for us, so a donut building gets the right footprint.
                if polygon_wgs84.geom_type == 'MultiPolygon':
                    parts_wgs84 = list(polygon_wgs84.geoms)
                    if len(parts_wgs84) > 1:
                        multipart_split += 1
                else:
                    parts_wgs84 = [polygon_wgs84]
            except Exception as e:
                # Per-feature defence: a single malformed OSM feature (null
                # geometry, unexpected property type, exotic GeoJSON variant)
                # used to abort the whole loop via the outer except. Skip
                # the bad feature and keep going.
                feature_errors += 1
                logger.debug(f'Skipping building feature {feature_idx}: {e}')
                continue

            for part_idx, part_wgs84 in enumerate(parts_wgs84):
                try:
                    polygon_local = shapely.ops.transform(project, part_wgs84)

                    # Skip sheds / outhouses / bus shelters that OSM tags as
                    # "building". These inflate model count without affecting
                    # navigation; removing them shaves scene-load time
                    # noticeably. Also guards against invalid geometry where
                    # shapely returns a non-positive area (self-intersecting,
                    # CW exterior, etc.) — those would otherwise get a
                    # nonsense area-based height.
                    try:
                        footprint_area = polygon_local.area
                    except Exception:
                        footprint_area = 0.0
                    if footprint_area <= 0.0:
                        invalid_geom_skipped += 1
                        continue
                    if footprint_area < MIN_BUILDING_AREA_M2:
                        tiny_skipped += 1
                        continue

                    centroid_local = polygon_local.centroid
                    pose_xy = (centroid_local.x, centroid_local.y)

                    # Re-center the polygon on the model's own origin so the
                    # <include> pose places it correctly in the world.
                    polygon_centered = shapely.affinity.translate(
                        polygon_local, xoff=-pose_xy[0], yoff=-pose_xy[1]
                    )

                    # Place the building's base at the LOWEST terrain point
                    # under its footprint — stops buildings on slopes from
                    # floating on one side.
                    if elevation_sampler is not None:
                        samples = [
                            elevation_sampler(x + pose_xy[0], y + pose_xy[1])
                            for x, y in list(polygon_centered.exterior.coords)[:8]
                        ]
                        pose_z = min(samples) if samples else 0.0
                    else:
                        pose_z = 0.0

                    # For single-part buildings keep the stable historical
                    # name; for multipart splits add a _pN suffix so link
                    # names stay unique inside a tile compound.
                    if len(parts_wgs84) == 1:
                        model_name = f'building_{_sanitize(building_id)}_{feature_idx}'
                    else:
                        model_name = (
                            f'building_{_sanitize(building_id)}_'
                            f'{feature_idx}_p{part_idx}'
                        )

                    # Height inference has access to the metric footprint
                    # area — lets the area-based extrapolation kick in when
                    # OSM gave no type.
                    height = _infer_height(props, area_m2=footprint_area, rng=rng)

                    body_sdf = _polygon_to_polyline_body_sdf(
                        polygon_centered, name_prefix=model_name,
                        pose_xyz=(pose_xy[0], pose_xy[1], pose_z),
                        height=height, color=color,
                    )
                    if '<polyline>' in body_sdf:
                        polyline_count += 1
                    else:
                        bbox_fallback += 1

                    buildings.append({
                        'model_name': model_name,
                        'link_name': model_name,
                        'pose_xy': pose_xy,
                        'pose_z': pose_z,
                        'body_sdf': body_sdf,
                    })
                except Exception as e:
                    # Per-part defence: a failure mid-part (transform error,
                    # degenerate centroid, sampler hiccup) shouldn't kill the
                    # surrounding feature's other parts or any later features.
                    feature_errors += 1
                    logger.debug(
                        f'Skipping building feature {feature_idx} '
                        f'part {part_idx}: {e}'
                    )
                    continue

        logger.info(
            f'OSM buildings processed: {len(buildings)} models '
            f'({polyline_count} polyline, {bbox_fallback} bbox-fallback, '
            f'{cloud_skipped} cloud-masked, '
            f'{tiny_skipped} below {MIN_BUILDING_AREA_M2:.0f} m², '
            f'{invalid_geom_skipped} invalid geometry, '
            f'{multipart_split} multipart features split, '
            f'{feature_errors} skipped on feature/part errors)'
        )
        return buildings
    except Exception as e:
        logger.error(f'Error processing OSM buildings to SDF models: {e}')
        raise
