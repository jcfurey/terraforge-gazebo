# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Turn OSM vegetation features into Gazebo tree instances.

Handles ``natural=tree`` points and ``natural=wood`` / ``landuse=forest``
polygons.

Each tree is a single static model (trunk cylinder + canopy sphere). We emit
one reusable ``tree_generic_<variant>`` model and place many ``<include>``
instances in the world — avoiding thousands of per-instance SDF files.
"""

import json
import math
import os
import random

import numpy as np
from PIL import Image, ImageDraw
import shapely
import shapely.affinity
import shapely.geometry
import shapely.ops

from terraforge.utils.coordinates import CoordinateConverter
from terraforge.utils.logging import logger
from terraforge.utils.seeding import stable_seed

# Number of reusable tree variants (different heights / canopy radii) that
# will be generated once and referenced by each placement. Variants are sorted
# smallest -> largest in _generate_tree_variants; named subsets below pick
# index ranges so scrub uses small variants and forests use the full mix.
TREE_VARIANTS = 5
_VARIANT_SHRUB = (0, 1)        # small variants (~3-5 m) for scrub / heath / hedge
_VARIANT_MID = (1, 2, 3)       # mid variants (~5-9 m) for orchards / parks
_VARIANT_FOREST = (0, 1, 2, 3, 4)  # full mix for dense woods

# Per-class scatter density (trees per square meter) for polygon features.
# Each tree is a full SDF model with collision; gz-sim + gz-rendering starts
# struggling beyond a few hundred instances at scale. Densities here are
# deliberately sparse — the scene still looks forested, but initial scene
# load stays under ~30 s on an RTX 3090 and the GUI doesn't flake out.
_POLYGON_CLASSES = {
    ('natural', 'wood'):     {'density': 1 / 900.0,  'variants': _VARIANT_FOREST},
    ('landuse', 'forest'):   {'density': 1 / 900.0,  'variants': _VARIANT_FOREST},
    ('natural', 'scrub'):    {'density': 1 / 1200.0, 'variants': _VARIANT_SHRUB},
    ('natural', 'heath'):    {'density': 1 / 1500.0, 'variants': _VARIANT_SHRUB},
    ('landuse', 'orchard'):  {'density': 1 / 600.0,  'variants': _VARIANT_MID},
    ('landuse', 'vineyard'): {'density': 1 / 1500.0, 'variants': _VARIANT_SHRUB},
    ('leisure', 'park'):     {'density': 1 / 2500.0, 'variants': _VARIANT_MID},
    ('leisure', 'garden'):   {'density': 1 / 800.0,  'variants': _VARIANT_MID},
}

# Per-meter density for LineString features like natural=tree_row / hedge.
# A dense hedgerow is ~one shrub every 2 m.
_LINESTRING_DENSITY = 1 / 2.0

# Hard cap on scattered trees per world. Now that level streaming (see
# sdf_builder) only keeps ~9 tiles active around the rover, the relevant
# number is trees-per-tile rather than total-trees. 3000 on a 2 km world
# with 97 tiles = ~30 trees/tile = ~270 trees loaded at any moment —
# within gz-rendering's comfortable range even on modest GPUs.
MAX_FOREST_TREES = 3000

# Image-based vegetation scatter — picks up tree cover OSM doesn't tag.
#   VEGETATION_DENSITY: trees per m² inside the vegetation mask.
#     1/150 ≈ 1 tree per 12 × 12 m patch, typical of temperate deciduous
#     forest canopy. Safe to crank because streaming bounds active count.
#   VEGETATION_EXG_MIN: Excess-Green threshold; lowered to 0.05 so we
#     capture the fainter green of Vicksburg's mixed residential cover,
#     not only dark dense canopy.
#   VEGETATION_L_MAX:   luminance cap; unchanged.
#   VEGETATION_BUILDING_BUFFER_M: exclude placements this close to walls.
VEGETATION_EXG_MIN = 0.05
VEGETATION_L_MAX = 0.65
VEGETATION_DENSITY = 1 / 150.0
# Largest tree-canopy radius is 4 m (biggest variant). 2 m was letting
# canopy spheres poke visibly through building walls; 6 m gives a clean
# 2 m gap between any tree trunk and the nearest facade.
VEGETATION_BUILDING_BUFFER_M = 6.0


# Tree "species" mapped from the existing variant index. Variants 0-4 keep
# the smallest-to-largest size progression that the variant_pool constants
# below already sort on, but each variant now gets a distinct canopy shape
# so a forest reads as a mix rather than identical ball-on-stick copies.
#   shape: shrub       = squat, multi-blob canopy, minimal trunk
#          broadleaf   = multi-sphere canopy (oak / maple silhouette)
#          conifer     = elongated spire (pine / spruce)
#   (trunk_h, trunk_r, canopy_half_h, canopy_r, shape)
_TREE_VARIANT_CONFIGS = [
    (1.0,  0.08, 1.2, 1.5, 'shrub'),       # low brush / shrub
    (3.5,  0.12, 2.2, 2.0, 'broadleaf'),   # small tree
    (6.0,  0.18, 3.0, 2.8, 'broadleaf'),   # medium broadleaf
    (9.0,  0.25, 4.5, 3.3, 'conifer'),     # tall conifer
    (12.0, 0.32, 5.0, 4.0, 'broadleaf'),   # mature broadleaf
]

# Canopy color palettes. Each tree picks a shade from the list matching its
# shape class so forests get a natural mottled look instead of one flat green.
_CANOPY_COLORS = {
    'broadleaf': [
        (0.10, 0.38, 0.12),   # forest
        (0.18, 0.46, 0.16),   # leaf mid
        (0.26, 0.52, 0.20),   # bright spring
        (0.32, 0.42, 0.16),   # olive
    ],
    'conifer': [
        (0.09, 0.30, 0.18),   # blue-green spruce
        (0.12, 0.32, 0.14),   # pine
        (0.15, 0.38, 0.20),   # fir
    ],
    'shrub': [
        (0.20, 0.45, 0.18),
        (0.28, 0.50, 0.22),
        (0.25, 0.40, 0.18),
    ],
}

_TRUNK_COLORS = [
    (0.25, 0.15, 0.08),   # dark bark
    (0.32, 0.20, 0.10),   # mid brown
    (0.35, 0.28, 0.20),   # weathered grey-brown
    (0.42, 0.30, 0.18),   # light brown
]

# Per-instance canopy size jitter (fractional). ±0.2 means canopies vary
# from 80% to 120% of nominal — enough to break the stamped-copy look.
_CANOPY_JITTER = 0.2
# Per-instance trunk length jitter — just enough that tree crowns sit at
# slightly different heights even within the same size tier.
_TRUNK_H_JITTER = 0.15


def _rand_color(palette, jitter_rng) -> tuple:
    r, g, b = palette[jitter_rng.randrange(len(palette))]

    # Small hue noise so even within a palette entry, adjacent trees vary.
    def d():
        return jitter_rng.uniform(-0.04, 0.04)

    return (max(0.0, min(1.0, r + d())),
            max(0.0, min(1.0, g + d())),
            max(0.0, min(1.0, b + d())))


def _broadleaf_canopy(base_z: float, cr: float, rng) -> str:
    """Multi-sphere blob canopy — oak/maple look.

    Spheres overlap and offset
    horizontally so the silhouette isn't a perfect circle from any angle.
    """
    r_c, g_c, b_c = _rand_color(_CANOPY_COLORS['broadleaf'], rng)
    mat = (f'<material><ambient>{r_c:.3f} {g_c:.3f} {b_c:.3f} 1</ambient>'
           f'<diffuse>{r_c:.3f} {g_c:.3f} {b_c:.3f} 1</diffuse>'
           f'<specular>0.02 0.02 0.02 1</specular></material>')
    # Three overlapping spheres around the trunk top.
    offsets = [
        (0.0, 0.0, 0.0, 1.00),
        (cr * 0.45 * rng.choice((-1, 1)), cr * 0.20 * rng.choice((-1, 1)), cr * 0.30, 0.85),
        (cr * 0.25 * rng.choice((-1, 1)), cr * 0.45 * rng.choice((-1, 1)), cr * 0.15, 0.80),
    ]
    parts = []
    for i, (dx, dy, dz, scale) in enumerate(offsets):
        parts.append(
            f"      <visual name='canopy_{i}'>\n"
            f'        <pose>{dx:.3f} {dy:.3f} {base_z + dz:.3f} 0 0 0</pose>\n'
            f'        <geometry><sphere><radius>{cr * scale:.3f}</radius></sphere></geometry>\n'
            f'        {mat}\n'
            f'      </visual>'
        )
    return '\n'.join(parts)


def _conifer_canopy(base_z: float, ch_half: float, cr: float, rng) -> str:
    """Spire canopy with a pine/spruce silhouette.

    Stacked cylinders with decreasing radius.

    Uses cylinder primitives since gz-sim primitive cone support
    is uneven across render backends.
    """
    r_c, g_c, b_c = _rand_color(_CANOPY_COLORS['conifer'], rng)
    mat = (f'<material><ambient>{r_c:.3f} {g_c:.3f} {b_c:.3f} 1</ambient>'
           f'<diffuse>{r_c:.3f} {g_c:.3f} {b_c:.3f} 1</diffuse>'
           f'<specular>0.02 0.02 0.02 1</specular></material>')
    # Three stacked cylinders: wide base, mid, narrow top.
    total_h = ch_half * 2.0
    sections = [
        (0.0,              total_h * 0.45, cr * 0.90),
        (total_h * 0.40,   total_h * 0.35, cr * 0.60),
        (total_h * 0.75,   total_h * 0.30, cr * 0.30),
    ]
    parts = []
    for i, (z_off, seg_h, seg_r) in enumerate(sections):
        cz = base_z - ch_half + z_off + seg_h / 2.0
        parts.append(
            f"      <visual name='canopy_{i}'>\n"
            f'        <pose>0 0 {cz:.3f} 0 0 0</pose>\n'
            f'        <geometry><cylinder><radius>{seg_r:.3f}</radius>'
            f'<length>{seg_h:.3f}</length></cylinder></geometry>\n'
            f'        {mat}\n'
            f'      </visual>'
        )
    return '\n'.join(parts)


def _shrub_canopy(base_z: float, cr: float, rng) -> str:
    """Low wide mound canopy for brush/hedgerow feel.

    A flattened sphere, optionally with a second small bump.
    """
    r_c, g_c, b_c = _rand_color(_CANOPY_COLORS['shrub'], rng)
    mat = (f'<material><ambient>{r_c:.3f} {g_c:.3f} {b_c:.3f} 1</ambient>'
           f'<diffuse>{r_c:.3f} {g_c:.3f} {b_c:.3f} 1</diffuse>'
           f'<specular>0.02 0.02 0.02 1</specular></material>')
    # Ellipsoid via sphere scaled by pose is not SDF-legal; use a short fat
    # cylinder for the main mound and a small sphere cap on top.
    mound_h = cr * 0.9
    parts = [
        f"      <visual name='canopy_mound'>\n"
        f'        <pose>0 0 {base_z:.3f} 0 0 0</pose>\n'
        f'        <geometry><cylinder><radius>{cr:.3f}</radius>'
        f'<length>{mound_h:.3f}</length></cylinder></geometry>\n'
        f'        {mat}\n'
        f'      </visual>',
        f"      <visual name='canopy_cap'>\n"
        f'        <pose>{cr*0.2*rng.choice((-1, 1)):.3f} '
        f'{cr*0.2*rng.choice((-1, 1)):.3f} '
        f'{base_z + mound_h * 0.4:.3f} 0 0 0</pose>\n'
        f'        <geometry><sphere><radius>{cr*0.55:.3f}</radius></sphere></geometry>\n'
        f'        {mat}\n'
        f'      </visual>',
    ]
    return '\n'.join(parts)


def _tree_link_sdf(link_name: str, pose_xyz: tuple, variant_idx: int) -> str:
    """Emit a single inline <link> for a tree at the given world pose.

    Shape varies by variant — shrubs, broadleaves, and conifers all generate
    different canopy geometry. Per-instance color and size jitter make any
    two trees distinguishable even within the same variant.
    """
    trunk_h, trunk_r, canopy_half_h, canopy_r, shape = _TREE_VARIANT_CONFIGS[variant_idx]
    px, py, pz = pose_xyz
    # Deterministic per-tree RNG keyed on (link_name, variant_idx). Keeps
    # a given regen reproducible while giving each tree a unique seed —
    # see stable_seed for why we cannot use Python's randomized hash().
    rng = random.Random(stable_seed(link_name, variant_idx))

    # Apply per-instance jitter.
    trunk_h_i = trunk_h * (1.0 + rng.uniform(-_TRUNK_H_JITTER, _TRUNK_H_JITTER))
    canopy_r_i = canopy_r * (1.0 + rng.uniform(-_CANOPY_JITTER, _CANOPY_JITTER))
    canopy_half_h_i = canopy_half_h * (1.0 + rng.uniform(-_CANOPY_JITTER, _CANOPY_JITTER))

    trunk_z = trunk_h_i / 2.0
    # Canopy center: top of trunk plus a fraction of the canopy height so the
    # crown sits just above the trunk tip.
    canopy_base_z = trunk_h_i + canopy_half_h_i * (0.2 if shape != 'shrub' else 0.0)

    r_t, g_t, b_t = _rand_color(_TRUNK_COLORS, rng)
    yaw = rng.uniform(0.0, 6.2832)   # random heading — mostly matters for asymmetric canopies

    if shape == 'broadleaf':
        canopy_sdf = _broadleaf_canopy(canopy_base_z, canopy_r_i, rng)
    elif shape == 'conifer':
        canopy_sdf = _conifer_canopy(canopy_base_z, canopy_half_h_i, canopy_r_i, rng)
    else:  # shrub
        canopy_sdf = _shrub_canopy(canopy_base_z, canopy_r_i, rng)

    return f"""    <link name='{link_name}'>
      <pose>{px:.3f} {py:.3f} {pz:.3f} 0 0 {yaw:.3f}</pose>
      <visual name='trunk_visual'>
        <pose>0 0 {trunk_z:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{trunk_r:.3f}</radius><length>{trunk_h_i:.3f}</length></cylinder></geometry>
        <material>
          <ambient>{r_t:.3f} {g_t:.3f} {b_t:.3f} 1</ambient>
          <diffuse>{r_t:.3f} {g_t:.3f} {b_t:.3f} 1</diffuse>
        </material>
      </visual>
      <collision name='trunk_collision'>
        <pose>0 0 {trunk_z:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{trunk_r:.3f}</radius><length>{trunk_h_i:.3f}</length></cylinder></geometry>
      </collision>
{canopy_sdf}
    </link>"""


def _variant_label(variant_idx: int) -> str:
    return f'tree_generic_{variant_idx}'


def fuel_wrapper_model_names() -> list:
    """Names of the `tree_fuel_<i>` wrapper models required by Fuel mode.

    Kept in sync with ``TREE_VARIANTS``; the include URI
    ``model://tree_fuel_<i>`` is resolved against ``GZ_SIM_RESOURCE_PATH``
    (or a path the generator bundles into the output world) at load time.
    """
    return [f'tree_fuel_{i}' for i in range(TREE_VARIANTS)]


def _fuel_wrapper_sdf(name: str, variant_idx: int) -> str:
    """Return a minimal, self-contained wrapper SDF for one fuel variant.

    The wrapper is intentionally a small set of primitive shapes that
    roughly matches the variant's cartoon counterpart. gz-sim parses
    the URI once and GPU-instances every `<include>model://...</include>`
    that references it, which is the point of fuel mode over cartoon —
    2000 tree instances cost the renderer 5 unique meshes, not 2000
    unique inline links. Each wrapper is static and collision-free.
    """
    trunk_h, trunk_r, canopy_half_h, canopy_r, shape = _TREE_VARIANT_CONFIGS[variant_idx]
    # Pick a representative color per shape class. Fuel wrappers are
    # variant-uniform by design — per-instance variety is cartoon's job.
    palette = _CANOPY_COLORS.get(shape, _CANOPY_COLORS['broadleaf'])
    r, g, b = palette[0]
    tr, tg, tb = _TRUNK_COLORS[0]
    half_trunk = trunk_h / 2.0
    canopy_z = trunk_h + canopy_half_h
    if shape == 'conifer':
        canopy_visual = (
            f'        <geometry><cone>'
            f'<radius>{canopy_r:.3f}</radius>'
            f'<length>{2 * canopy_half_h:.3f}</length>'
            f'</cone></geometry>'
        )
    else:
        canopy_visual = (
            f'        <geometry><sphere>'
            f'<radius>{canopy_r:.3f}</radius>'
            f'</sphere></geometry>'
        )
    return f"""<?xml version='1.0'?>
<sdf version='1.10'>
  <model name='{name}'>
    <static>true</static>
    <link name='body'>
      <visual name='trunk'>
        <pose>0 0 {half_trunk:.3f} 0 0 0</pose>
        <geometry><cylinder>
          <radius>{trunk_r:.3f}</radius>
          <length>{trunk_h:.3f}</length>
        </cylinder></geometry>
        <material>
          <ambient>{tr} {tg} {tb} 1</ambient>
          <diffuse>{tr} {tg} {tb} 1</diffuse>
        </material>
      </visual>
      <visual name='canopy'>
        <pose>0 0 {canopy_z:.3f} 0 0 0</pose>
{canopy_visual}
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _fuel_wrapper_config(name: str) -> str:
    return f"""<?xml version='1.0'?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version='1.10'>model.sdf</sdf>
  <description>TerraForge-generated fuel wrapper for {name}.</description>
</model>
"""


def write_fuel_wrappers(dest_dir: str) -> list:
    """Write minimal wrapper model dirs for every fuel variant.

    Writes one ``tree_fuel_<i>/model.sdf`` + ``model.config`` per variant
    under ``dest_dir``. Returns the list of wrapper names written.

    Idempotent — existing wrappers are left alone so a user-supplied
    pack that appears earlier on ``GZ_SIM_RESOURCE_PATH`` continues to
    take precedence. The output world's launch file adds ``dest_dir``
    to that path automatically (see ``launch/spawn_world.launch.py``).
    """
    os.makedirs(dest_dir, exist_ok=True)
    written = []
    for variant_idx in range(TREE_VARIANTS):
        name = f'tree_fuel_{variant_idx}'
        model_dir = os.path.join(dest_dir, name)
        sdf_path = os.path.join(model_dir, 'model.sdf')
        cfg_path = os.path.join(model_dir, 'model.config')
        if os.path.isfile(sdf_path):
            continue
        os.makedirs(model_dir, exist_ok=True)
        with open(sdf_path, 'w', encoding='utf-8') as f:
            f.write(_fuel_wrapper_sdf(name, variant_idx))
        with open(cfg_path, 'w', encoding='utf-8') as f:
            f.write(_fuel_wrapper_config(name))
        written.append(name)
    return written


def missing_fuel_wrappers(extra_roots: list = None) -> list:
    """Return the subset of fuel wrapper model names not found on disk.

    Searches every path in ``$GZ_SIM_RESOURCE_PATH`` plus any ``extra_roots``
    the caller passes (e.g. the current output directory's ``models_fuel/``
    subdir). A wrapper is "present" iff ``<root>/tree_fuel_<i>/model.sdf``
    exists. If the returned list is non-empty, ``--foliage-style fuel`` will
    produce SDF with unresolvable URIs and trees will silently disappear in
    Gazebo — callers should surface this to the user up front.
    """
    roots = []
    env = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if env:
        # GZ_SIM_RESOURCE_PATH is colon-separated on Linux, semicolon on Windows.
        sep = ';' if os.name == 'nt' else ':'
        roots.extend(p for p in env.split(sep) if p)
    if extra_roots:
        roots.extend(extra_roots)
    missing = []
    for name in fuel_wrapper_model_names():
        found = any(
            os.path.isfile(os.path.join(root, name, 'model.sdf'))
            for root in roots
        )
        if not found:
            missing.append(name)
    return missing


def _tree_fuel_include_sdf(unique_name: str, pose_xyz: tuple, variant_idx: int) -> str:
    """Emit a top-level `<include>` referencing a `tree_fuel_<variant>` wrapper.

    The wrapper model is resolved via `GZ_SIM_RESOURCE_PATH` — it lives under
    `<world-dir>/models_fuel/tree_fuel_<variant>/`, where its `model.sdf`
    `<include>`s the real Gazebo Fuel URI (Oak tree / Pine Tree) with the
    appropriate `<scale>` for this variant's size class.

    Yaw is randomized per-tree via the same name-hashed RNG used by the cartoon
    path so a regen produces reproducibly-oriented trees. Fuel models already
    carry their own trunk color + canopy shape, so no per-instance jitter is
    needed here — visual variation comes from the 5 wrapper scales and yaw.
    """
    px, py, pz = pose_xyz
    rng = random.Random(stable_seed(unique_name, variant_idx))
    yaw = rng.uniform(0.0, 6.2832)
    return (
        f'<include>\n'
        f'  <name>{unique_name}</name>\n'
        f'  <pose>{px:.3f} {py:.3f} {pz:.3f} 0 0 {yaw:.3f}</pose>\n'
        f'  <uri>model://tree_fuel_{variant_idx}</uri>\n'
        f'</include>'
    )


def _scatter_in_polygon(polygon, density, max_count, rng):
    """Poisson-ish point scatter inside a polygon at ``density`` points/m²."""
    minx, miny, maxx, maxy = polygon.bounds
    area = polygon.area
    target = min(max(int(area * density), 1), max_count)
    # Batch the rejection sample: shapely.contains_xy vectorizes over
    # numpy arrays at GEOS speed, ~50x faster than looping with
    # polygon.contains(Point(...)). On a 1000-vertex forest with target
    # 2000, the scatter goes from ~300 ms to ~6 ms per polygon. We fill
    # the budget in at most two rounds for very sparse polygons (small
    # intersection area relative to bbox).
    need = target
    out_x = []
    out_y = []
    # Draw 6x the remaining target per round so the bbox:polygon area
    # ratio doesn't starve the result. Seeded via Python's rng so the
    # stream is deterministic for a given (world, origin, radius).
    for _round in range(4):
        if need <= 0:
            break
        n = max(need * 6, 16)
        # random.Random -> numpy uses a 32-bit-safe seed per draw.
        seed = rng.randrange(0, 2**32)
        rs = np.random.RandomState(seed)
        xs = rs.uniform(minx, maxx, n)
        ys = rs.uniform(miny, maxy, n)
        inside = shapely.contains_xy(polygon, xs, ys)
        hit_x = xs[inside]
        hit_y = ys[inside]
        take = min(need, hit_x.size)
        out_x.extend(hit_x[:take].tolist())
        out_y.extend(hit_y[:take].tolist())
        need -= take
    return list(zip(out_x, out_y))


def _load_building_polygons_gazebo(buildings_geojson_path: str, converter, world_box):
    """Return a list of building footprint polygons in Gazebo-frame meters.

    Used to exclude tree placements from building footprints in the
    image-based vegetation scatter. Silently returns [] if the file is
    missing or unreadable — tree placement just gains no building exclusion.
    """
    if not buildings_geojson_path or not os.path.exists(buildings_geojson_path):
        return []

    def project(lon, lat, *_):
        gx, gy, _z = converter.wgs84_to_gazebo((lat, lon))
        return gx, gy

    try:
        with open(buildings_geojson_path, encoding='utf-8') as f:
            data = json.load(f)
    except Exception as e:
        logger.warning(f'Could not load buildings for tree exclusion: {e}')
        return []

    polys = []
    for feat in data.get('features', []):
        g = feat.get('geometry')
        if not g:
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
        except Exception as e:
            # Defensive per-feature skip (tree placement just loses one
            # exclusion polygon), but log it so malformed OSM extracts are
            # diagnosable instead of silently shrinking the mask.
            logger.debug(f'Skipping building polygon for tree exclusion: {e}')
            continue
    return polys


def _build_vegetation_mask(texture_path: str):
    """Detect likely-vegetation pixels in the satellite texture (legacy path).

    Only used when ``foliage_mask`` is ``None`` (CLI flag ``--foliage-mask off``).
    The default code path routes through :class:`FoliageMask` which layers
    texture-variance canopy detection + OSM positive union + road/parking
    exclusion on top of this — see ``data_processing/foliage_mask.py``.

    Uses the Excess-Green index (2G - R - B) combined with a luminance cap
    so the mask picks up canopy and foliage but rejects gray concrete /
    black pavement. The input PNG is expected to be in a true UTM-meter-
    square projection (see textures._reproject_webmercator_to_utm) so the
    mask can be indexed linearly against Gazebo-frame coordinates.

    Logs EXG and luminance percentiles so operators can tune
    VEGETATION_EXG_MIN / VEGETATION_L_MAX for their imagery — different
    providers and seasons push the distribution around (winter vs summer,
    JPEG compression artefacts, etc.). Returns the boolean mask (shape
    H x W) and the PIL image size.
    """
    img = Image.open(texture_path).convert('RGB')
    w, h = img.size
    arr = np.asarray(img, dtype=np.float32) / 255.0
    R, G, B = arr[..., 0], arr[..., 1], arr[..., 2]
    exg = 2.0 * G - R - B
    luminance = (R + G + B) / 3.0
    # Drop exact-black pixels from percentiles so reprojection padding
    # (if any) doesn't skew the stats toward zero.
    nonblack = luminance > 0.01
    if nonblack.any():
        e50, e90, e99 = np.percentile(exg[nonblack], [50, 90, 99])
        l50, l90, l99 = np.percentile(luminance[nonblack], [50, 90, 99])
    else:
        e50 = e90 = e99 = l50 = l90 = l99 = 0.0
    logger.info(
        f'Vegetation detector: EXG p50/p90/p99 = {e50:.3f}/{e90:.3f}/{e99:.3f}, '
        f'L p50/p90/p99 = {l50:.3f}/{l90:.3f}/{l99:.3f} '
        f'(thresholds: EXG>{VEGETATION_EXG_MIN}, L<{VEGETATION_L_MAX})'
    )
    mask = (exg > VEGETATION_EXG_MIN) & (luminance < VEGETATION_L_MAX)
    return mask, (w, h)


def _rasterize_building_mask(building_polys_gazebo, image_size_px, world_half_extent_m,
                             dilate_m: float = 0.0):
    """Rasterize building footprints into a boolean mask at image resolution.

    ``dilate_m`` buffers each polygon outward by this many meters before
    rasterization, so trees can't be placed *right against* building walls.
    """
    w, h = image_size_px
    img = Image.new('L', (w, h), 0)
    draw = ImageDraw.Draw(img)
    # Image (px=0, py=0) == world (-r, +r); (px=w-1, py=h-1) == (+r, -r).
    # Resolution is the same on both axes since textures are UTM-square.
    px_per_m = w / (2.0 * world_half_extent_m)

    def g2px(x, y):
        return ((x + world_half_extent_m) * px_per_m,
                (world_half_extent_m - y) * px_per_m)

    for poly in building_polys_gazebo:
        if dilate_m > 0:
            try:
                poly = poly.buffer(dilate_m, resolution=2)
            except Exception:
                pass
        geoms = [poly] if isinstance(poly, shapely.geometry.Polygon) else list(poly.geoms)
        for g in geoms:
            ext = [g2px(x, y) for x, y in g.exterior.coords]
            draw.polygon(ext, fill=1)
            for interior in g.interiors:
                draw.polygon([g2px(x, y) for x, y in interior.coords], fill=0)
    return np.asarray(img, dtype=bool)


def _scatter_on_vegetation(
    texture_path: str,
    building_polys_gazebo: list,
    world_half_extent_m: float,
    budget: int,
    rng: random.Random,
    add_tree,
    placements_list: list,
    variant_pool,
    foliage_mask=None,
):
    """Grid-scatter trees onto vegetation pixels, avoiding building footprints.

    Returns the number of trees actually appended to ``placements_list``.

    When ``foliage_mask`` is a :class:`~terraforge.data_processing.foliage_mask.FoliageMask`,
    it takes full responsibility for the placeable raster — canopy texture,
    OSM positive union, and road/parking/building exclusion are all baked in.
    When it's ``None``, fall back to the legacy bare-EXG heuristic so callers
    that predate the foliage-mask CLI flag (``--foliage-mask off``) still work.
    """
    if not texture_path or not os.path.exists(texture_path):
        logger.info('No satellite texture — skipping vegetation-based tree fill.')
        return 0
    # Figure out the reference pixel grid. We always use the satellite
    # texture's native grid so the sampled Gazebo-frame (cx, cy) → pixel
    # conversion below is an exact identity with the texture, regardless
    # of how the mask itself was constructed (at ~5 m/px internally).
    with Image.open(texture_path) as _tex_img:
        w, h = _tex_img.size

    if foliage_mask is not None:
        placeable = foliage_mask.sample_grid(w, h, world_half_extent_m)
        placeable_frac = float(placeable.mean()) if placeable.size else 0.0
        logger.info(
            f'Vegetation scatter using FoliageMask '
            f'(mask native {foliage_mask.width}x{foliage_mask.height}, '
            f'placeable {placeable_frac:.1%} after resample to {w}x{h})'
        )
    else:
        veg_mask, _ = _build_vegetation_mask(texture_path)
        bldg_mask = _rasterize_building_mask(
            building_polys_gazebo, (w, h), world_half_extent_m,
            dilate_m=VEGETATION_BUILDING_BUFFER_M,
        )
        placeable = veg_mask & (~bldg_mask)
        veg_frac = float(veg_mask.mean()) if veg_mask.size else 0.0
        bldg_frac = float(bldg_mask.mean()) if bldg_mask.size else 0.0
        placeable_frac = float(placeable.mean()) if placeable.size else 0.0
        logger.info(
            f'Vegetation mask (legacy EXG path): {veg_frac:.1%} green '
            f'(EXG>{VEGETATION_EXG_MIN}, L<{VEGETATION_L_MAX}), '
            f'{bldg_frac:.1%} buildings — {placeable_frac:.1%} placeable'
        )

    pitch_m = 1.0 / math.sqrt(VEGETATION_DENSITY)
    jitter = pitch_m * 0.35  # random offset within cell to break the grid pattern
    px_per_m = w / (2.0 * world_half_extent_m)

    grid_n = int((2.0 * world_half_extent_m) / pitch_m)

    # Build the full list of (ix, iy) cells, then shuffle so when the budget
    # caps placements, the kept trees are uniformly sampled across the map
    # instead of clustering at whichever edge row-major iteration started on.
    cells = [(ix, iy) for ix in range(grid_n) for iy in range(grid_n)]
    rng.shuffle(cells)

    before = len(placements_list)
    for ix, iy in cells:
        if (len(placements_list) - before) >= budget:
            return len(placements_list) - before
        cx = -world_half_extent_m + (ix + 0.5) * pitch_m + rng.uniform(-jitter, jitter)
        cy = -world_half_extent_m + (iy + 0.5) * pitch_m + rng.uniform(-jitter, jitter)
        px = int((cx + world_half_extent_m) * px_per_m)
        py = int((world_half_extent_m - cy) * px_per_m)
        if px < 0 or px >= w or py < 0 or py >= h:
            continue
        if not placeable[py, px]:
            continue
        # add_tree rejects out-of-world and cloud-masked points itself;
        # we diff placements_list length to count real adds.
        add_tree(cx, cy, variant_pool=variant_pool)
    return len(placements_list) - before


def process_osm_trees_to_sdf(osm_filepath: str, models_dir: str, origin_wgs84: tuple,
                             elevation_sampler=None, cloud_mask=None,
                             world_half_extent_m: float = None,
                             seed: int = 1337,
                             satellite_texture_path: str = None,
                             buildings_geojson_path: str = None,
                             vegetation_fill: bool = True,
                             foliage_style: str = 'cartoon',
                             foliage_mask=None) -> list:
    """Return a list of tree placements with mode-appropriate SDF fragments.

    Handles both point features (``natural=tree``) and polygon features
    (``natural=wood`` / ``landuse=forest``), scattering synthetic trees inside
    each polygon at the per-class density declared in ``_POLYGON_CLASSES``,
    capped globally at ``MAX_FOREST_TREES``.

    ``foliage_style`` selects the emission mode:

    - ``'cartoon'`` (default): each placement carries ``link_sdf`` — an inline
      trunk+canopy `<link>` with per-instance color/size jitter. The SDF
      builder packs these into tile compound models so the scene renders with
      no external dependencies.
    - ``'fuel'``: each placement carries ``fuel_include_sdf`` — a top-level
      `<include>` of ``model://tree_fuel_<variant>`` (resolved via
      ``GZ_SIM_RESOURCE_PATH`` to a wrapper model that pulls a real Fuel
      mesh). The SDF builder keeps these OUT of tile compounds (SDF doesn't
      allow `<include>` inside `<model>`) and instead emits them at world
      scope, with a per-tile `<ref>` so level streaming still works.
    """
    # A missing OSM foliage file is NOT a reason to skip the whole stage:
    # the image-based vegetation fill below exists precisely for areas where
    # OSM has no foliage features (Overpass only returns features whose
    # nodes fall inside the bbox, so e.g. the middle of a large tagged park
    # comes back empty). Returning early here used to produce zero trees on
    # heavily wooded worlds whenever the OSM layer was empty.
    have_osm_foliage = os.path.exists(osm_filepath)
    if not have_osm_foliage:
        logger.info(
            'No OSM foliage file; scattering from satellite imagery alone.'
        )
    else:
        logger.info(f'Processing OSM foliage from {osm_filepath}')
    os.makedirs(models_dir, exist_ok=True)
    rng = random.Random(seed)

    converter = CoordinateConverter(origin_wgs84)

    def project(x_lon, y_lat, z=None):
        gx, gy, _ = converter.wgs84_to_gazebo((y_lat, x_lon))
        return (gx, gy) if z is None else (gx, gy, z)

    placements = []
    forest_tree_budget = MAX_FOREST_TREES
    point_count = 0
    poly_count = 0
    cloud_skipped = 0
    out_of_world_skipped = 0

    # Clip forest polygons to this square so trees from a large natural=wood
    # feature don't scatter far outside the terrain's extent. Point trees
    # (natural=tree) get a separate per-point bounds check.
    world_box = None
    if world_half_extent_m is not None and world_half_extent_m > 0:
        world_box = shapely.geometry.box(
            -world_half_extent_m, -world_half_extent_m,
            world_half_extent_m, world_half_extent_m,
        )

    if have_osm_foliage:
        with open(osm_filepath, encoding='utf-8') as f:
            osm_data = json.load(f)
    else:
        osm_data = {'features': []}

    def in_world(x, y):
        if world_half_extent_m is None:
            return True
        return (-world_half_extent_m <= x <= world_half_extent_m and
                -world_half_extent_m <= y <= world_half_extent_m)

    def tree_is_cloudy(gx, gy):
        if cloud_mask is None:
            return False
        lat, lon = converter.gazebo_to_wgs84((gx, gy))
        return cloud_mask.is_cloudy(lat, lon)

    def add_tree(x, y, variant_pool=None):
        nonlocal cloud_skipped, out_of_world_skipped
        if not in_world(x, y):
            out_of_world_skipped += 1
            return
        if tree_is_cloudy(x, y):
            cloud_skipped += 1
            return
        pool = variant_pool if variant_pool is not None else tuple(range(TREE_VARIANTS))
        variant_idx = rng.choice(pool)
        z = float(elevation_sampler(x, y)) if elevation_sampler is not None else 0.0
        link_name = f'tree_{len(placements)}'
        placement = {
            'model_name': _variant_label(variant_idx),
            'link_name': link_name,   # unique per-instance link id inside tile model
            'variant_idx': variant_idx,
            'pose_xy': (x, y),
            'pose_z': z,
        }
        if foliage_style == 'fuel':
            # Top-level `<include>` — sdf_builder will NOT bucket this into a
            # tile compound (SDF doesn't allow `<include>` inside `<model>`);
            # it emits it at world scope and registers a per-tile <ref> so
            # level streaming still culls it when the rover is far away.
            fuel_name = f'tree_fuel_{len(placements)}'
            placement['fuel_include_name'] = fuel_name
            placement['fuel_include_sdf'] = _tree_fuel_include_sdf(
                fuel_name, (x, y, z), variant_idx,
            )
        else:
            # Cartoon (default): inline trunk+canopy `<link>`, color- and
            # size-jittered per-instance. Packed into tile compound models by
            # sdf_builder for fast scene parse on >1 km worlds.
            placement['link_sdf'] = _tree_link_sdf(link_name, (x, y, z), variant_idx)
        placements.append(placement)

    def _classify_polygon(props):
        """Return the _POLYGON_CLASSES entry matching this feature's tags.

        Returns None if the polygon isn't a recognized foliage class.
        """
        for (key, value), cfg in _POLYGON_CLASSES.items():
            if props.get(key) == value:
                return cfg
        return None

    line_count = 0
    for feature in osm_data['features']:
        geom = feature['geometry']
        gtype = geom['type']
        props = feature.get('properties', {}) or {}

        if gtype == 'Point':
            lon, lat = geom['coordinates'][:2]
            gx, gy, _ = converter.wgs84_to_gazebo((lat, lon))
            # Individual OSM trees are usually medium-sized; skip the
            # largest-forest and smallest-shrub variants to keep them
            # looking like single mature trees rather than a 12 m monster.
            add_tree(gx, gy, variant_pool=_VARIANT_MID)
            point_count += 1

        elif gtype == 'LineString':
            # Hedgerows / tree_row / tree lines. Scatter shrub-sized variants
            # along the line at _LINESTRING_DENSITY. Treat MultiLineString via
            # iteration over parts if we ever see them.
            if forest_tree_budget <= 0:
                continue
            line_wgs84 = shapely.geometry.shape(geom)
            line_local = shapely.ops.transform(project, line_wgs84)
            if world_box is not None:
                line_local = line_local.intersection(world_box)
                if line_local.is_empty:
                    continue
            length = line_local.length
            target = min(int(length * _LINESTRING_DENSITY),
                         forest_tree_budget)
            # Decrement the budget by trees actually placed (not by `target`),
            # since add_tree rejects cloud-masked / out-of-world candidates.
            # Decrementing optimistically would silently exhaust the budget on
            # cloudy or edge-of-world polygons before reaching the cap.
            before = len(placements)
            for i in range(target):
                t = (i + 0.5) / max(target, 1)
                pt = line_local.interpolate(t, normalized=True)
                add_tree(pt.x, pt.y, variant_pool=_VARIANT_SHRUB)
            forest_tree_budget -= (len(placements) - before)
            line_count += 1

        elif gtype in ('Polygon', 'MultiPolygon'):
            cfg = _classify_polygon(props)
            if cfg is None or forest_tree_budget <= 0:
                continue
            polygon_wgs84 = shapely.geometry.shape(geom)
            polygon_local = shapely.ops.transform(project, polygon_wgs84)
            if not polygon_local.is_valid or polygon_local.area <= 0:
                continue
            # OSM returns the feature's full geometry (which can span km beyond
            # the user's bbox). Clip to the world square so scattered trees
            # can't land outside the terrain mesh.
            if world_box is not None:
                polygon_local = polygon_local.intersection(world_box)
                if polygon_local.is_empty or polygon_local.area <= 0:
                    continue
            pts = _scatter_in_polygon(
                polygon_local, cfg['density'], forest_tree_budget, rng
            )
            # Decrement the budget by trees actually placed (not by len(pts)),
            # since add_tree rejects cloud-masked / out-of-world candidates.
            before = len(placements)
            for x, y in pts:
                add_tree(x, y, variant_pool=cfg['variants'])
            forest_tree_budget -= (len(placements) - before)
            poly_count += 1

    osm_count = len(placements)

    # Image-based vegetation fill: scatter additional trees on satellite
    # pixels that look like canopy but weren't covered by OSM tags. Vicksburg
    # is a good example — OSM has 3 foliage features for 2 km², the actual
    # ground is heavily wooded. Runs only if caller gave us a UTM-reprojected
    # satellite texture (without that, pixel-to-Gazebo mapping is off).
    veg_count = 0
    if vegetation_fill and satellite_texture_path:
        remaining = max(0, MAX_FOREST_TREES - osm_count)
        if remaining == 0:
            logger.info('Tree budget exhausted by OSM; skipping vegetation fill.')
        else:
            # Only the legacy-EXG fallback path needs building polygons — the
            # FoliageMask has already rasterized them at construction time.
            if foliage_mask is None:
                building_polys = _load_building_polygons_gazebo(
                    buildings_geojson_path, converter, world_box,
                )
            else:
                building_polys = []
            veg_count = _scatter_on_vegetation(
                texture_path=satellite_texture_path,
                building_polys_gazebo=building_polys,
                world_half_extent_m=world_half_extent_m,
                budget=remaining,
                rng=rng,
                add_tree=add_tree,
                placements_list=placements,
                variant_pool=_VARIANT_FOREST,
                foliage_mask=foliage_mask,
            )

    style_tag = 'fuel-include' if foliage_style == 'fuel' else 'inline'
    logger.info(
        f'OSM foliage processed: {len(placements)} trees '
        f'({point_count} mapped points, {line_count} hedgerows, '
        f'{poly_count} vegetated polygons, {veg_count} image-vegetation, '
        f'{cloud_skipped} cloud-masked, {out_of_world_skipped} outside-world) '
        f'across {TREE_VARIANTS} {style_tag} variants'
    )
    return placements
