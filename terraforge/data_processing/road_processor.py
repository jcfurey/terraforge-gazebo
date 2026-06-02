# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Turn OSM ``highway=*`` LineStrings into Gazebo road models.

Each OSM way becomes one static model. The way's polyline is buffered by its
per-class width (motorway wider than residential) and emitted as an extruded
SDF ``<polyline>`` polygon with a small thickness so it sits just above the
terrain. Elevations are sampled along the way and averaged to keep the road
roughly flush with the ground.
"""

import json
import math
import os

import shapely.affinity
import shapely.geometry
import shapely.ops

from terraforge.utils.coordinates import CoordinateConverter
from terraforge.utils.logging import logger
from terraforge.utils.osm_roads import half_width_for_props

ROAD_THICKNESS = 0.08  # meters above terrain
ROAD_COLOR = (0.20, 0.20, 0.22)  # dark asphalt

# Max length (meters) of a single road segment before the emitter breaks
# the OSM way into multiple sub-segments. Each sub-segment gets its own
# elevation sample and its own flat slab, so the chain of slabs follows
# the DEM at segment granularity instead of one giant horizontal plank.
# 20 m is short enough to stay close to the terrain even on steep
# hillsides (~5 deg per 20 m = ~1.8 m drop, below ROAD_THICKNESS +
# visible z-fight margin) without multiplying output SDF size too much.
_ROAD_SEGMENT_MAX_LEN_M = 20.0


def _sanitize(name):
    return str(name).replace(':', '_').replace('/', '_').replace(' ', '_')


def _polygon_to_body_sdf(polygon, name_prefix, pose_xyz, thickness, color=ROAD_COLOR) -> str:
    """Return a `<visual>`-only fragment for a road segment.

    The segment pose is baked in and the element name uniquified via
    ``name_prefix``. Designed to live inside the shared per-tile
    ``<link>`` alongside buildings — one link per tile instead of one
    link per road segment slashes entity count at load.

    Roads don't emit a `<collision>` — the rover drives on the flat
    ground plane (no gz-sim physics backend implements heightmap
    collision) and the road surface is purely visual.
    """
    if polygon.geom_type != 'Polygon' or not polygon.is_valid or not polygon.is_simple:
        return None
    if not polygon.exterior.is_ccw:
        polygon = shapely.geometry.polygon.orient(polygon, sign=1.0)
    coords = list(polygon.exterior.coords)
    if coords[0] == coords[-1]:
        coords = coords[:-1]
    pts = '\n            '.join(f'<point>{x:.3f} {y:.3f}</point>' for x, y in coords)
    r, g, b = color
    px, py, pz = pose_xyz
    return (
        f"      <visual name='vis_{name_prefix}'>\n"
        f'        <pose>{px:.3f} {py:.3f} {pz:.3f} 0 0 0</pose>\n'
        f'        <geometry>\n'
        f'          <polyline>\n'
        f'            {pts}\n'
        f'            <height>{thickness:.3f}</height>\n'
        f'          </polyline>\n'
        f'        </geometry>\n'
        f'        <material>\n'
        f'          <ambient>{r} {g} {b} 1</ambient>\n'
        f'          <diffuse>{r} {g} {b} 1</diffuse>\n'
        f'          <specular>0.05 0.05 0.05 1</specular>\n'
        f'        </material>\n'
        f'      </visual>'
    )


def process_osm_roads_to_sdf(osm_filepath: str, models_dir: str, origin_wgs84: tuple,
                             elevation_sampler=None) -> list:
    """Return a list of ``{model_name, pose_xy, pose_z}`` road placements.

    Each LineString becomes one buffered polygon-extrusion model. Non-line
    geometries in the GeoJSON (e.g. crossing markers) are skipped.
    """
    if not os.path.exists(osm_filepath):
        logger.info('No OSM roads file; skipping roads stage.')
        return []

    logger.info(f'Processing OSM roads from {osm_filepath} to models in {models_dir}')
    os.makedirs(models_dir, exist_ok=True)

    converter = CoordinateConverter(origin_wgs84)

    def project(x_lon, y_lat, z=None):
        gx, gy, _ = converter.wgs84_to_gazebo((y_lat, x_lon))
        return (gx, gy) if z is None else (gx, gy, z)

    placements = []
    skipped = 0
    feature_errors = 0
    with open(osm_filepath, encoding='utf-8') as f:
        osm_data = json.load(f)

    for feature_idx, feature in enumerate(osm_data.get('features', []) or []):
        try:
            geom = feature.get('geometry') or {}
            gtype = geom.get('type')
            props = feature.get('properties') or {}
            if gtype not in ('LineString', 'MultiLineString'):
                skipped += 1
                continue

            line_wgs84 = shapely.geometry.shape(geom)
            line_local = shapely.ops.transform(project, line_wgs84)
            if line_local.is_empty or line_local.length < 0.5:
                skipped += 1
                continue

            half_w = half_width_for_props(props)
            road_id = props.get('osmid', f'{feature_idx}')
            base_name = f'road_{_sanitize(road_id)}_{feature_idx}'

            # MultiLineString: iterate component lines; each becomes its own
            # segment chain below.
            if line_local.geom_type == 'MultiLineString':
                component_lines = list(line_local.geoms)
            else:
                component_lines = [line_local]

            for comp_idx, comp_line in enumerate(component_lines):
                comp_placements = _emit_road_segments(
                    comp_line, half_w, base_name, comp_idx,
                    elevation_sampler=elevation_sampler,
                )
                if not comp_placements:
                    skipped += 1
                else:
                    placements.extend(comp_placements)
        except Exception as e:
            # Per-feature defence: a single corrupt OSM way shouldn't kill
            # the rest of the road processing pass.
            feature_errors += 1
            logger.debug(f'Skipping road feature {feature_idx}: {e}')
            continue

    logger.info(
        f'OSM roads processed: {len(placements)} road models, '
        f'{skipped} skipped, {feature_errors} on feature errors'
    )
    return placements


def _emit_road_segments(line_local, half_w, base_name, comp_idx,
                        elevation_sampler=None):
    """Break ``line_local`` into short, DEM-following slab segments.

    Each chunk is <= _ROAD_SEGMENT_MAX_LEN_M and becomes one inline-link
    placement, sampling elevation at each
    chunk's midpoint so the sequence of slabs follows the DEM instead of
    producing one long horizontal plank. Placements carry ``link_sdf`` so
    ``sdf_builder.build_scene_tiles`` groups them into per-tile compound
    models alongside buildings and cartoon trees — no per-segment
    model.sdf is written (nothing references it via model://).

    Returns a list of placement dicts (possibly empty). Returning the list
    directly avoids the previous module-level ``_segment_placements`` global
    that doubled as a return channel — that pattern was reentrancy-fragile
    and surprising to readers tracing data flow.
    """
    placements = []
    length = line_local.length
    if length < 0.5:
        return placements

    n_segments = max(1, int(math.ceil(length / _ROAD_SEGMENT_MAX_LEN_M)))
    for seg_idx in range(n_segments):
        t0 = seg_idx / n_segments
        t1 = (seg_idx + 1) / n_segments
        seg_pts = _subline_points(line_local, t0, t1)
        if len(seg_pts) < 2:
            continue
        seg_line = shapely.geometry.LineString(seg_pts)
        try:
            poly_local = seg_line.buffer(half_w, cap_style=2, join_style=2)
        except Exception:
            continue
        if poly_local.is_empty or poly_local.geom_type != 'Polygon':
            continue

        centroid = poly_local.centroid
        pose_xy = (centroid.x, centroid.y)
        poly_centered = shapely.affinity.translate(
            poly_local, xoff=-pose_xy[0], yoff=-pose_xy[1]
        )

        if elevation_sampler is not None:
            mid = seg_line.interpolate(0.5, normalized=True)
            start = seg_pts[0]
            end = seg_pts[-1]
            samples = [
                elevation_sampler(start[0], start[1]),
                elevation_sampler(mid.x, mid.y),
                elevation_sampler(end[0], end[1]),
            ]
            pose_z = sum(samples) / len(samples) + ROAD_THICKNESS / 2.0
        else:
            pose_z = ROAD_THICKNESS / 2.0

        link_name = f'{base_name}_c{comp_idx}_s{seg_idx}'
        body_sdf = _polygon_to_body_sdf(
            poly_centered, link_name, (pose_xy[0], pose_xy[1], pose_z),
            ROAD_THICKNESS,
        )
        if body_sdf is None:
            continue

        placements.append({
            'model_name': link_name,
            'link_name': link_name,
            'pose_xy': pose_xy,
            'pose_z': pose_z,
            'body_sdf': body_sdf,
        })
    return placements


def _subline_points(line, t0, t1):
    """Return the vertex sequence of ``line`` over a parametric sub-range.

    Restricted to the range [t0, t1] (normalized), keeping any original OSM
    vertices that fall strictly inside so the sub-line preserves curvature.
    """
    pts = [line.interpolate(t0, normalized=True)]
    total = line.length
    if total <= 0:
        return [(pts[0].x, pts[0].y)]
    target_lo = t0 * total
    target_hi = t1 * total
    # Walk the original vertices; distance-along-line is monotonic.
    coords = list(line.coords)
    if len(coords) < 2:
        return [(coords[0][0], coords[0][1])]
    accumulated = 0.0
    for i in range(1, len(coords)):
        x0, y0 = coords[i - 1][0], coords[i - 1][1]
        x1, y1 = coords[i][0], coords[i][1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        next_acc = accumulated + seg_len
        if next_acc > target_lo and accumulated < target_hi:
            # This original segment overlaps [target_lo, target_hi].
            if accumulated >= target_lo and accumulated <= target_hi:
                pts.append(shapely.geometry.Point(x0, y0))
        accumulated = next_acc
        if accumulated >= target_hi:
            break
    pts.append(line.interpolate(t1, normalized=True))
    return [(p.x, p.y) for p in pts]
