import math

from jinja2 import Environment, FileSystemLoader

from terraforge.utils.logging import logger


# Tile size for the compound static scene. Buildings and trees are grouped
# into one <model name='tile_X_Y'> per tile so gz-sim parses a small number
# of top-level entities instead of hundreds of per-object <include>s — the
# main lever for world-load time on >1 km scenes.
DEFAULT_TILE_SIZE_M = 200.0

# Active radius (metres) the tile streamer keeps tiles loaded within. Tiles
# whose centres are farther than this from the performer are teleported to
# DEFAULT_LEVEL_HIDDEN_Z so their render + physics cost disappears. Fog in
# <scene> is tuned to hide the boundary; keep active_radius ≳ fog end.
DEFAULT_LEVEL_ACTIVE_RADIUS_M = 300.0

# Z depth (metres) to park hidden tiles at. Must be well below any
# possible driving surface so the teleported tiles don't occlude the
# rover. -10000 m is far past any reasonable world.
DEFAULT_LEVEL_HIDDEN_Z = -10000.0

# Default performer model name that the streamer tracks. ros_gz_sim::create
# spawns the rover with name=$ROBOT_NAME, which is "rovermax" in this
# workspace. Override via CLI if your model has a different name.
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


def build_scene_tiles(buildings, trees, roads, half_extent_m,
                       tile_size_m=DEFAULT_TILE_SIZE_M):
    """Group every static placement into tile buckets and render each as a
    compound <model name='tile_X_Y'> with all its links inline.

    Returns a list of dicts with model SDF, tile indices, world-frame
    center (for level bbox placement), and link count.
    """
    all_placements = list(buildings or []) + list(trees or []) + list(roads or [])
    tiles_map = _group_placements_into_tiles(all_placements, half_extent_m, tile_size_m)

    tile_records = []
    for (tx, ty), items in sorted(tiles_map.items()):
        tile_id = f"tile_{tx}_{ty}"
        links_sdf = "\n".join(item['link_sdf'] for item in items)
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
        joints_sdf = "\n".join(
            f"    <joint name='j_{item['link_name']}' type='fixed'>\n"
            f"      <parent>{root_name}</parent>\n"
            f"      <child>{item['link_name']}</child>\n"
            f"    </joint>"
            for item in items
        )
        # <static>true</static> is mandatory — compound tile is a terrain prop.
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
        # Centre of the tile in world metres. Used by <level> geometry/pose
        # below so the tile's activation bbox sits over its contents.
        cx = -half_extent_m + (tx + 0.5) * tile_size_m
        cy = -half_extent_m + (ty + 0.5) * tile_size_m
        tile_records.append({
            'tile_id': tile_id,
            'tx': tx,
            'ty': ty,
            'count': len(items),
            'model_sdf': model_sdf,
            'center_x': cx,
            'center_y': cy,
            'size_xy': tile_size_m,
        })
    if tile_records:
        avg = sum(r['count'] for r in tile_records) / len(tile_records)
        logger.info(
            f"Scene tiled: {len(tile_records)} tile model(s), "
            f"{sum(r['count'] for r in tile_records)} links total, "
            f"~{avg:.0f} links/tile (tile_size={tile_size_m:.0f} m)"
        )
    return tile_records


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
                              level_hidden_z=DEFAULT_LEVEL_HIDDEN_Z,
                              performer_ref=DEFAULT_PERFORMER_REF,
                              enable_level_streaming=True):
        half_extent_m = extent_meters / 2.0
        scene_tiles = build_scene_tiles(
            buildings, trees, roads, half_extent_m, tile_size_m=tile_size_m,
        )
        if enable_level_streaming and scene_tiles:
            logger.info(
                f"Tile streamer plugin enabled: performer '{performer_ref}', "
                f"active radius {level_active_radius_m:.0f} m, "
                f"{len(scene_tiles)} tile(s)"
            )
        template = self.template_env.get_template('world_template.sdf.j2')
        rendered_sdf = template.render(
            heightmap_path=heightmap_path,
            texture_path=texture_path,
            flat_normal_path=flat_normal_path,
            scene_tiles=scene_tiles,
            extent_meters=extent_meters,
            height_amplitude=height_amplitude,
            terrain_z_offset=terrain_z_offset,
            enable_level_streaming=enable_level_streaming,
            level_active_radius_m=level_active_radius_m,
            level_hidden_z=level_hidden_z,
            performer_ref=performer_ref,
        )
        logger.info("SDF world template rendered.")
        return rendered_sdf

    def save_sdf_world_file(self, sdf_content, output_path):
        with open(output_path, 'w') as sdf_file:
            sdf_file.write(sdf_content)
        logger.info(f"SDF world file saved to {output_path}")
