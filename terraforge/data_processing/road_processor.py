# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Turn OSM ``highway=*`` LineStrings into draped, collidable road meshes.

Each OSM way is split into short chunks (so per-tile level streaming still
culls distant road), and each chunk is emitted as a thin triangulated ribbon
slab baked to OBJ. The ribbon's cross-sections are draped onto the DEM — every
left/right edge vertex samples the terrain elevation — so the road follows the
relief smoothly instead of stepping as flat horizontal planks. The mesh is
referenced for BOTH ``<visual>`` and ``<collision>`` (with asphalt friction),
so with dartsim's heightmap collision the rover drives on the road surface
sitting just above the terrain it collides with.
"""

import json
import math
import os
from urllib.parse import quote

import shapely.geometry
import shapely.ops

from terraforge.data_processing import mesh_builder
from terraforge.utils.coordinates import CoordinateConverter
from terraforge.utils.logging import logger
from terraforge.utils.osm_roads import half_width_for_props

ROAD_THICKNESS = 0.08            # slab thickness (m)
ROAD_SURFACE_CLEARANCE = 0.06    # how far the road top sits above the DEM (m)
ROAD_COLOR = (0.20, 0.20, 0.22)  # dark asphalt
# Asphalt friction. Matches the terrain/ground-plane surface so a rover gets
# the same (deliberately high) traction on-road as off-road rather than
# sliding where the two surfaces meet.
ROAD_MU = 100
ROAD_MU2 = 50

# Max length (m) of a chunk before the way is broken up. Each chunk is one
# placement so sdf_builder.build_scene_tiles buckets it into the right tile
# and level streaming culls distant road with its tile.
_ROAD_SEGMENT_MAX_LEN_M = 20.0
# Spacing (m) of draped cross-sections within a chunk. Finer than the chunk
# length so the ribbon hugs the DEM mid-chunk, not just at chunk ends.
_ROAD_VERTEX_STEP_M = 4.0


def _sanitize(name):
    return str(name).replace(':', '_').replace('/', '_').replace(' ', '_')


def _mesh_file_uri(path: str) -> str:
    """Absolute path -> percent-encoded ``file://`` URI for an SDF mesh uri."""
    return 'file://' + quote(os.path.abspath(path), safe='/')


def _densify(points, step):
    """Insert points so consecutive spacing is <= ``step`` (for DEM draping)."""
    if len(points) < 2:
        return list(points)
    out = [points[0]]
    for i in range(1, len(points)):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        dist = math.hypot(x1 - x0, y1 - y0)
        if dist <= step:
            out.append((x1, y1))
            continue
        n = int(math.ceil(dist / step))
        for k in range(1, n + 1):
            t = k / n
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    return out


def _cross_sections(points, half_w):
    """Return (left, right) edge points offset +/- half_w along the polyline.

    The offset direction at each vertex uses the central-difference tangent so
    the ribbon width stays consistent through bends.
    """
    n = len(points)
    left, right = [], []
    for i in range(n):
        if i == 0:
            tx, ty = points[1][0] - points[0][0], points[1][1] - points[0][1]
        elif i == n - 1:
            tx = points[i][0] - points[i - 1][0]
            ty = points[i][1] - points[i - 1][1]
        else:
            tx = points[i + 1][0] - points[i - 1][0]
            ty = points[i + 1][1] - points[i - 1][1]
        length = math.hypot(tx, ty) or 1.0
        nx, ny = -ty / length, tx / length      # unit left normal
        x, y = points[i]
        left.append((x + half_w * nx, y + half_w * ny))
        right.append((x - half_w * nx, y - half_w * ny))
    return left, right


def _road_mesh_body_sdf(name_prefix, pose_xyz, mesh_uri, color=ROAD_COLOR):
    """`<collision>` (asphalt friction) + `<visual>` referencing a road mesh.

    Lives inside the shared per-tile ``<link>`` alongside buildings — one link
    per tile instead of one link per road chunk keeps the entity count low.
    """
    r, g, b = color
    px, py, pz = pose_xyz
    return (
        f"      <collision name='col_{name_prefix}'>\n"
        f'        <pose>{px:.3f} {py:.3f} {pz:.3f} 0 0 0</pose>\n'
        f'        <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>\n'
        f'        <surface>\n'
        f'          <friction>\n'
        f'            <ode>\n'
        f'              <mu>{ROAD_MU}</mu>\n'
        f'              <mu2>{ROAD_MU2}</mu2>\n'
        f'            </ode>\n'
        f'          </friction>\n'
        f'        </surface>\n'
        f'      </collision>\n'
        f"      <visual name='vis_{name_prefix}'>\n"
        f'        <pose>{px:.3f} {py:.3f} {pz:.3f} 0 0 0</pose>\n'
        f'        <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>\n'
        f'        <material>\n'
        f'          <ambient>{r} {g} {b} 1</ambient>\n'
        f'          <diffuse>{r} {g} {b} 1</diffuse>\n'
        f'          <specular>0.05 0.05 0.05 1</specular>\n'
        f'        </material>\n'
        f'      </visual>'
    )


def process_osm_roads_to_sdf(osm_filepath: str, models_dir: str, origin_wgs84: tuple,
                             elevation_sampler=None) -> list:
    """Return a list of ``{model_name, pose_xy, pose_z, body_sdf}`` placements.

    Each LineString becomes a chain of draped ribbon-slab chunks. Non-line
    geometries in the GeoJSON (e.g. crossing markers) are skipped.
    """
    if not os.path.exists(osm_filepath):
        logger.info('No OSM roads file; skipping roads stage.')
        return []

    logger.info(f'Processing OSM roads from {osm_filepath} to models in {models_dir}')
    os.makedirs(models_dir, exist_ok=True)
    meshes_dir = os.path.join(models_dir, 'road_meshes')
    os.makedirs(meshes_dir, exist_ok=True)

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

            if line_local.geom_type == 'MultiLineString':
                component_lines = list(line_local.geoms)
            else:
                component_lines = [line_local]

            for comp_idx, comp_line in enumerate(component_lines):
                comp_placements = _emit_road_segments(
                    comp_line, half_w, base_name, comp_idx, meshes_dir,
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
        f'OSM roads processed: {len(placements)} road mesh chunks, '
        f'{skipped} skipped, {feature_errors} on feature errors'
    )
    return placements


def _emit_road_segments(line_local, half_w, base_name, comp_idx, meshes_dir,
                        elevation_sampler=None):
    """Break ``line_local`` into draped ribbon-slab chunks.

    Each chunk is <= _ROAD_SEGMENT_MAX_LEN_M long, densified to
    _ROAD_VERTEX_STEP_M, offset to left/right edges, draped on the DEM, and
    baked to an OBJ referenced for visual + collision. Placements carry
    ``body_sdf`` so ``sdf_builder.build_scene_tiles`` merges them into per-tile
    compound models alongside buildings. Returns a list of placement dicts.
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

        dense = _densify(seg_pts, _ROAD_VERTEX_STEP_M)
        left, right = _cross_sections(dense, half_w)
        if len(left) < 2:
            continue

        if elevation_sampler is not None:
            z_left = [elevation_sampler(x, y) + ROAD_SURFACE_CLEARANCE
                      for x, y in left]
            z_right = [elevation_sampler(x, y) + ROAD_SURFACE_CLEARANCE
                       for x, y in right]
        else:
            z_left = [ROAD_SURFACE_CLEARANCE] * len(left)
            z_right = [ROAD_SURFACE_CLEARANCE] * len(right)

        all_x = [p[0] for p in left] + [p[0] for p in right]
        all_y = [p[1] for p in left] + [p[1] for p in right]
        cx = sum(all_x) / len(all_x)
        cy = sum(all_y) / len(all_y)
        top_left = [(x - cx, y - cy, z) for (x, y), z in zip(left, z_left)]
        top_right = [(x - cx, y - cy, z) for (x, y), z in zip(right, z_right)]

        link_name = f'{base_name}_c{comp_idx}_s{seg_idx}'
        mesh_path = os.path.join(meshes_dir, f'{link_name}.obj')
        if not mesh_builder.generate_ribbon_obj(
                mesh_path, top_left, top_right, ROAD_THICKNESS):
            continue

        body_sdf = _road_mesh_body_sdf(
            link_name, (cx, cy, 0.0), _mesh_file_uri(mesh_path),
        )
        placements.append({
            'model_name': link_name,
            'link_name': link_name,
            'pose_xy': (cx, cy),
            'pose_z': 0.0,
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
