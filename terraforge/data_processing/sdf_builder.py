import math

from jinja2 import Environment, FileSystemLoader

from terraforge.utils.logging import logger
from terraforge.utils.naming import safe_identifier


# Tile size for the compound static scene. Buildings and trees are grouped
# into one <model name='tile_X_Y'> per tile so gz-sim parses a small number
# of top-level entities instead of hundreds of per-object <include>s — the
# main lever for world-load time on >1 km scenes.
DEFAULT_TILE_SIZE_M = 200.0

# Max distance from the performer (metres) at which a tile should stay
# loaded. Tiles farther than this get culled by gz-sim's level manager
# (which requires `gz sim --levels` — see the <plugin filename="dummy">
# block in world_template.sdf.j2 for the mechanics). Each per-level
# <buffer> actually emitted to SDF is derived as max(0, active_radius -
# tile_size/2) so a rover within active_radius of any tile's centre
# lands inside that level's extended AABB. Tune against fog end in
# <scene> to hide the activation boundary.
DEFAULT_LEVEL_ACTIVE_RADIUS_M = 300.0

# Default performer model name that the level manager tracks. ros_gz_sim's
# `create` spawns the rover with name=$ROBOT_NAME, which is "rovermax" in
# this workspace. Override via CLI if your model has a different name.
DEFAULT_PERFORMER_REF = "rovermax"


def _tile_index(pose_xy, half_extent_m, tile_size_m):
    """Return (tx, ty) tile indices for a placement at Gazebo (x, y).

    Tiles are numbered from the SW corner: (0, 0) is at world (-r, -r).
    Values outside [0, n-1] indicate a placement outside the world box.
    """
    n = max(1, int(math.ceil((2.0 * half_extent_m) / tile_size_m)))
    x, y = pose_xy
    tx = int((x + half_extent_m) / tile_size_m)
    ty = int((y + half_extent_m) / tile_size_m)
    tx = max(0, min(tx, n - 1))
    ty = max(0, min(ty, n - 1))
    return tx, ty, n


def _group_placements_into_tiles(placements, half_extent_m, tile_size_m):
    """Bucket placements into a dict keyed by (tx, ty); each value is a list
    of {'link_sdf': ...} items. Placements lacking 'link_sdf' are skipped
    (legacy callers that didn't emit inline SDF)."""
    tiles = {}
    for p in placements or []:
        if 'link_sdf' not in p or p['link_sdf'] is None:
            continue
        tx, ty, _n = _tile_index(p['pose_xy'], half_extent_m, tile_size_m)
        tiles.setdefault((tx, ty), []).append(p)
    return tiles


def _fuel_tree_tile_map(trees, half_extent_m, tile_size_m):
    """Return {(tx, ty): [fuel_include_name, ...]} for fuel-mode tree
    placements. Non-fuel placements (those without ``fuel_include_sdf``) are
    skipped. The returned mapping lets `render_world_template` emit per-tile
    `<ref>` entries so the gz-sim level manager unloads distant Fuel trees
    alongside their cartoon-scene tile.
    """
    tile_map = {}
    for p in trees or []:
        if not p.get('fuel_include_sdf'):
            continue
        tx, ty, _n = _tile_index(p['pose_xy'], half_extent_m, tile_size_m)
        tile_map.setdefault((tx, ty), []).append(p['fuel_include_name'])
    return tile_map


def build_scene_tiles(buildings, trees, roads, half_extent_m,
                       tile_size_m=DEFAULT_TILE_SIZE_M):
    """Group every static placement into tile buckets and render each as a
    compound <model name='tile_X_Y'> with all its links inline.

    Returns a list of dicts with model SDF, tile indices, world-frame
    center (for level bbox placement), and link count. Fuel-mode tree
    placements (those emitted as top-level `<include>` instead of inline
    `<link>`) skip the compound entirely — see ``_fuel_tree_tile_map`` /
    ``render_world_template`` for how they're attached to per-tile
    `<level>` entries so streaming still culls them with their tile.
    """
    all_placements = list(buildings or []) + list(trees or []) + list(roads or [])
    tiles_map = _group_placements_into_tiles(all_placements, half_extent_m, tile_size_m)
    # Fuel-mode tree names, bucketed by tile, so each tile's <level> can
    # <ref> its trees. Computed up-front so even tiles that happen to have
    # zero buildings/roads (trees only) still emit a level block below.
    fuel_tree_tile_map = _fuel_tree_tile_map(trees, half_extent_m, tile_size_m)
    all_tile_keys = sorted(set(tiles_map.keys()) | set(fuel_tree_tile_map.keys()))

    tile_records = []
    for (tx, ty) in all_tile_keys:
        tile_id = f"tile_{tx}_{ty}"
        items = tiles_map.get((tx, ty), [])
        fuel_tree_names = fuel_tree_tile_map.get((tx, ty), [])
        # bullet-featherstone requires every link in a model to be connected
        # into a single kinematic tree via joints. A static compound tile
        # with N disconnected links fails validation ("Multiple sub-trees /
        # floating links detected") and the engine silently drops all but
        # one link — breaking collision for every other building/tree.
        # Fix: emit an empty root link and a fixed joint tying each content
        # link to it. Zero DOF, zero dynamics cost, passes validation.
        # Note: SDFormat reserves names with leading/trailing double
        # underscores; use plain names so the parser accepts them.
        root_name = f"{tile_id}_root"
        root_link = f"    <link name='{root_name}'></link>"
        if items:
            links_sdf = "\n".join(item['link_sdf'] for item in items)
            joints_sdf = "\n".join(
                f"    <joint name='j_{item['link_name']}' type='fixed'>\n"
                f"      <parent>{root_name}</parent>\n"
                f"      <child>{item['link_name']}</child>\n"
                f"    </joint>"
                for item in items
            )
            # <static>true</static> mandatory — compound tile is a terrain prop.
            # Single pose at origin; each link carries its own world-relative pose.
            model_sdf = (
                f"  <model name='{tile_id}'>\n"
                f"    <static>true</static>\n"
                f"    <pose>0 0 0 0 0 0</pose>\n"
                f"{root_link}\n"
                f"{links_sdf}\n"
                f"{joints_sdf}\n"
                f"  </model>"
            )
        else:
            # Fuel-only tile — no inline links, so no compound model needed.
            # We still emit a record so the level block below gets a <ref> for
            # the tile's fuel trees.
            model_sdf = ""
        # Centre of the tile in world metres. Used by <level> geometry/pose
        # below so the tile's activation bbox sits over its contents.
        cx = -half_extent_m + (tx + 0.5) * tile_size_m
        cy = -half_extent_m + (ty + 0.5) * tile_size_m
        tile_records.append({
            'tile_id': tile_id,
            'tx': tx,
            'ty': ty,
            'count': len(items),
            'fuel_tree_count': len(fuel_tree_names),
            'fuel_tree_names': fuel_tree_names,
            'has_compound_model': bool(items),
            'model_sdf': model_sdf,
            'center_x': cx,
            'center_y': cy,
            'size_xy': tile_size_m,
        })
    if tile_records:
        avg = sum(r['count'] for r in tile_records) / len(tile_records)
        total_fuel = sum(r['fuel_tree_count'] for r in tile_records)
        fuel_tile_count = sum(1 for r in tile_records if r['fuel_tree_count'])
        logger.info(
            f"Scene tiled: {len(tile_records)} tile(s), "
            f"{sum(r['count'] for r in tile_records)} compound links total, "
            f"~{avg:.0f} links/tile (tile_size={tile_size_m:.0f} m)"
            + (f"; {total_fuel} fuel-mode trees across {fuel_tile_count} tile(s)"
               if total_fuel else "")
        )
    return tile_records


def collect_fuel_tree_includes(trees):
    """Return the concatenated top-level `<include>` SDF for every fuel-mode
    tree placement, or empty string if none. Placements are emitted in the
    order tree_processor produced them; the level manager takes care of
    activation, so ordering is cosmetic."""
    fragments = [
        p['fuel_include_sdf']
        for p in (trees or [])
        if p.get('fuel_include_sdf')
    ]
    if not fragments:
        return ""
    return "\n".join(fragments)


class SDFWorldBuilder:
    def __init__(self, template_dir):
        self.template_env = Environment(loader=FileSystemLoader(template_dir))
        logger.info(f"SDF World Builder initialized with template directory: {template_dir}")

    def render_world_template(self, *, heightmap_path=None, texture_path=None,
                              flat_normal_path=None,
                              buildings=None, trees=None, roads=None,
                              extent_meters=1000.0, height_amplitude=200.0,
                              terrain_z_offset=0.0,
                              tile_size_m=DEFAULT_TILE_SIZE_M,
                              level_active_radius_m=DEFAULT_LEVEL_ACTIVE_RADIUS_M,
                              performer_ref=DEFAULT_PERFORMER_REF,
                              enable_level_streaming=True,
                              foliage_style='cartoon'):
        # performer_ref is interpolated into <model name>, <performer name>,
        # and <ref> elements in world_template.sdf.j2 without XML escaping.
        # Reject anything that would break SDF parsing.
        performer_ref = safe_identifier(performer_ref, field='performer_ref')
        half_extent_m = extent_meters / 2.0
        scene_tiles = build_scene_tiles(
            buildings, trees, roads, half_extent_m, tile_size_m=tile_size_m,
        )
        # Fuel-mode tree placements are top-level includes — collect their
        # raw SDF fragment so the template can splice them in at world scope,
        # separately from the tile compounds. Each tile already carries the
        # names of its fuel trees for <level><ref> registration.
        fuel_tree_includes_sdf = collect_fuel_tree_includes(trees)
        # Native gz-sim <level> uses an AABB + <buffer> for hysteresis.
        # Keep the "active radius" arg as the caller-facing knob (max
        # distance from tile centre at which it should load) and derive
        # the per-level buffer from it.
        level_buffer_m = max(0.0, level_active_radius_m - tile_size_m / 2.0)
        if enable_level_streaming and scene_tiles:
            logger.info(
                f"Native level streaming enabled: performer '{performer_ref}', "
                f"active radius {level_active_radius_m:.0f} m "
                f"(buffer {level_buffer_m:.0f} m around "
                f"{tile_size_m:.0f} m tiles), {len(scene_tiles)} tile(s). "
                f"Launch with `gz sim --levels`."
            )
        template = self.template_env.get_template('world_template.sdf.j2')
        rendered_sdf = template.render(
            heightmap_path=heightmap_path,
            texture_path=texture_path,
            flat_normal_path=flat_normal_path,
            scene_tiles=scene_tiles,
            fuel_tree_includes_sdf=fuel_tree_includes_sdf,
            extent_meters=extent_meters,
            height_amplitude=height_amplitude,
            terrain_z_offset=terrain_z_offset,
            enable_level_streaming=enable_level_streaming,
            level_active_radius_m=level_active_radius_m,
            level_buffer_m=level_buffer_m,
            performer_ref=performer_ref,
            foliage_style=foliage_style,
        )
        logger.info("SDF world template rendered.")
        return rendered_sdf

    def save_sdf_world_file(self, sdf_content, output_path):
        with open(output_path, 'w') as sdf_file:
            sdf_file.write(sdf_content)
        logger.info(f"SDF world file saved to {output_path}")
