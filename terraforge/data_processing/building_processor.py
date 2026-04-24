
import json
import os

import shapely.affinity
import shapely.geometry
import shapely.ops

from terraforge.utils.coordinates import CoordinateConverter
from terraforge.utils.logging import logger

import random as _random

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
    import math as _m
    h = 4.0 + 2.0 * _m.log10(max(area_m2, 50.0) / 50.0)
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
    """Apply up to ±_HEIGHT_JITTER relative jitter. Deterministic given rng."""
    if rng is None:
        return h
    return h * (1.0 + rng.uniform(-_HEIGHT_JITTER, _HEIGHT_JITTER))


def _polygon_to_polyline_link_sdf(polygon, link_name: str, pose_xyz: tuple,
                                   height: float, color=(0.7, 0.7, 0.7)) -> str:
    """Render a shapely Polygon as an inline <link> SDF fragment.

    Used when many buildings share a single compound <model>, so each
    building is one <link> with its own pose inside the tile model.
    Returns just the <link>...</link> string (no enclosing <model>).
    """
    if not polygon.is_valid or polygon.geom_type != 'Polygon':
        return _polygon_to_box_link_sdf(polygon, link_name, pose_xyz, height, color)

    if not polygon.exterior.is_ccw:
        polygon = shapely.geometry.polygon.orient(polygon, sign=1.0)

    coords = list(polygon.exterior.coords)
    if coords[0] == coords[-1]:
        coords = coords[:-1]
    pts = "\n            ".join(f"<point>{x:.3f} {y:.3f}</point>" for x, y in coords)

    minx, miny, maxx, maxy = polygon.bounds
    box_x = max(maxx - minx, 0.1)
    box_y = max(maxy - miny, 0.1)
    box_cx = (minx + maxx) / 2.0
    box_cy = (miny + maxy) / 2.0

    r, g, b = color
    px, py, pz = pose_xyz
    return f"""    <link name='{link_name}'>
      <pose>{px:.3f} {py:.3f} {pz:.3f} 0 0 0</pose>
      <collision name='collision'>
        <pose>{box_cx:.3f} {box_cy:.3f} {height / 2.0:.3f} 0 0 0</pose>
        <geometry><box><size>{box_x:.3f} {box_y:.3f} {height:.3f}</size></box></geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <polyline>
            {pts}
            <height>{height:.3f}</height>
          </polyline>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>"""


def _polygon_to_box_link_sdf(polygon, link_name: str, pose_xyz: tuple,
                              height: float, color=(0.7, 0.7, 0.7)) -> str:
    """Bounding-box fallback as a <link>. Used when polyline extrusion
    isn't viable (multi-polygon, self-intersecting exterior, etc.)."""
    minx, miny, maxx, maxy = polygon.bounds
    size_x = max(maxx - minx, 0.1)
    size_y = max(maxy - miny, 0.1)
    r, g, b = color
    px, py, pz = pose_xyz
    return f"""    <link name='{link_name}'>
      <pose>{px:.3f} {py:.3f} {pz + height / 2.0:.3f} 0 0 0</pose>
      <collision name='collision'>
        <geometry><box><size>{size_x:.3f} {size_y:.3f} {height:.3f}</size></box></geometry>
      </collision>
      <visual name='visual'>
        <geometry><box><size>{size_x:.3f} {size_y:.3f} {height:.3f}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>"""


def _polygon_to_polyline_sdf(polygon, height: float, color=(0.7, 0.7, 0.7)) -> str:
    """Render a shapely Polygon as an SDF visual polyline extrusion + bbox collision.

    dartsim does not support <polyline> as a collision geometry (only box,
    sphere, cylinder, capsule, mesh, plane). We therefore keep the polyline as
    a *visual* for accurate footprint rendering and use an axis-aligned bbox
    for collision — the rover collides with a conservative hull of the
    building, but no per-run silent collision-build failures.
    """
    # If it's not a simple Polygon we can express with <polyline>, fall back to bbox-only.
    if polygon.geom_type != 'Polygon' or not polygon.is_valid or not polygon.is_simple:
        return _polygon_to_box_sdf(polygon, height, color)

    # SDF <polyline> winding must be CCW for the extrusion normal to point up.
    if not polygon.exterior.is_ccw:
        polygon = shapely.geometry.polygon.orient(polygon, sign=1.0)

    coords = list(polygon.exterior.coords)
    if coords[0] == coords[-1]:
        coords = coords[:-1]
    pts = "\n            ".join(f"<point>{x:.3f} {y:.3f}</point>" for x, y in coords)

    minx, miny, maxx, maxy = polygon.bounds
    box_x = max(maxx - minx, 0.1)
    box_y = max(maxy - miny, 0.1)
    box_cx = (minx + maxx) / 2.0
    box_cy = (miny + maxy) / 2.0

    r, g, b = color
    return f"""    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <pose>{box_cx:.3f} {box_cy:.3f} {height / 2.0:.3f} 0 0 0</pose>
        <geometry><box><size>{box_x:.3f} {box_y:.3f} {height:.3f}</size></box></geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <polyline>
            {pts}
            <height>{height:.3f}</height>
          </polyline>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>"""


def _polygon_to_box_sdf(polygon, height: float, color=(0.7, 0.7, 0.7)) -> str:
    """Axis-aligned bounding-box fallback for tricky footprints."""
    minx, miny, maxx, maxy = polygon.bounds
    size_x = max(maxx - minx, 0.1)
    size_y = max(maxy - miny, 0.1)
    r, g, b = color
    return f"""    <static>true</static>
    <pose>0 0 {height / 2.0} 0 0 0</pose>
    <link name='link'>
      <collision name='collision'>
        <geometry><box><size>{size_x} {size_y} {height}</size></box></geometry>
      </collision>
      <visual name='visual'>
        <geometry><box><size>{size_x} {size_y} {height}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>"""


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
    """
    Process OSM building footprints from a GeoJSON file and write one Gazebo
    model per building under ``models_dir``. The footprint polygon is preserved
    via SDF ``<polyline>`` when possible; self-intersecting or multipart
    polygons fall back to an axis-aligned bounding box.

    ``elevation_sampler(gx, gy) -> z_meters`` optionally provides terrain Z at
    the building footprint so the model sits on the ground instead of floating
    at z=0.

    Returns ``[{'model_name': str, 'pose_xy': (x, y), 'pose_z': z}, ...]``.
    """
    logger.info(f"Processing OSM buildings from {osm_filepath} to models in {models_dir}")
    os.makedirs(models_dir, exist_ok=True)

    converter = CoordinateConverter(origin_wgs84)
    # Deterministic RNG seeded by origin coords, so two runs over the same
    # location produce the same height-jitter pattern and diffs stay clean.
    rng = _random.Random(hash(origin_wgs84) & 0xFFFFFFFF)

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
        logger.info("No OSM buildings file; skipping buildings stage.")
        return buildings
    try:
        with open(osm_filepath, 'r') as f:
            osm_data = json.load(f)
        polyline_count = 0
        bbox_fallback = 0
        tiny_skipped = 0

        cloud_skipped = 0
        for feature_idx, feature in enumerate(osm_data['features']):
            geom_type = feature['geometry']['type']
            if geom_type not in ('Polygon', 'MultiPolygon'):
                continue

            props = feature['properties']
            building_id = props.get('osmid', f"{feature_idx}")
            color = _building_color(props)
            # Height inference needs the footprint area, computed below.
            # We'll set it just before emitting the link SDF. For now,
            # placeholder; finalised after polygon_local is computed.

            polygon_wgs84 = shapely.geometry.shape(feature['geometry'])
            # Skip buildings whose centroid falls under a cloud in the
            # satellite mosaic — we can't visually verify the footprint.
            if cloud_mask is not None:
                c = polygon_wgs84.centroid
                if cloud_mask.is_cloudy(c.y, c.x):
                    cloud_skipped += 1
                    continue
            polygon_local = shapely.ops.transform(project, polygon_wgs84)

            # Skip sheds / outhouses / bus shelters that OSM tags as
            # "building". These inflate model count without affecting
            # navigation; removing them shaves scene-load time noticeably.
            try:
                footprint_area = polygon_local.area
            except Exception:
                footprint_area = 0.0
            if footprint_area < MIN_BUILDING_AREA_M2:
                tiny_skipped += 1
                continue

            centroid_local = polygon_local.centroid
            pose_xy = (centroid_local.x, centroid_local.y)

            # Re-center the polygon on the model's own origin so the <include>
            # pose places it correctly in the world.
            polygon_centered = shapely.affinity.translate(
                polygon_local, xoff=-pose_xy[0], yoff=-pose_xy[1]
            )

            # Place the building's base at the LOWEST terrain point under its
            # footprint — stops buildings on slopes from floating on one side.
            if elevation_sampler is not None:
                samples = [
                    elevation_sampler(x + pose_xy[0], y + pose_xy[1])
                    for x, y in list(polygon_centered.exterior.coords)[:8]
                ]
                pose_z = min(samples) if samples else 0.0
            else:
                pose_z = 0.0

            model_name = f"building_{_sanitize(building_id)}_{feature_idx}"

            # Height inference now has access to the metric footprint area —
            # lets the area-based extrapolation kick in when OSM gave no type.
            height = _infer_height(props, area_m2=footprint_area, rng=rng)

            link_sdf = _polygon_to_polyline_link_sdf(
                polygon_centered, link_name=model_name,
                pose_xyz=(pose_xy[0], pose_xy[1], pose_z),
                height=height, color=color,
            )
            if '<polyline>' in link_sdf:
                polyline_count += 1
            else:
                bbox_fallback += 1

            buildings.append({
                'model_name': model_name,
                'link_name': model_name,   # link inside the tile compound model
                'pose_xy': pose_xy,
                'pose_z': pose_z,
                'link_sdf': link_sdf,
            })

        logger.info(
            f"OSM buildings processed: {len(buildings)} models "
            f"({polyline_count} polyline, {bbox_fallback} bbox-fallback, "
            f"{cloud_skipped} cloud-masked, "
            f"{tiny_skipped} below {MIN_BUILDING_AREA_M2:.0f} m²) in {models_dir}"
        )
        return buildings
    except Exception as e:
        logger.error(f"Error processing OSM buildings to SDF models: {e}")
        raise
