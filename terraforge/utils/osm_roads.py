"""OSM ``highway=*`` width tables and parser, shared between road_processor
and foliage_mask.

Both modules need to know the half-width of an OSM road by class (and how
to interpret an explicit ``width`` tag) — road_processor for SDF emission,
foliage_mask for the negative road-buffer that excludes scatter trees from
asphalt. Keeping the table in one place avoids the two files drifting
when a new highway class is added.
"""


# Per-highway-class half-width (metres). Road total width = 2 * entry.
ROAD_HALF_WIDTH = {
    'motorway':    7.5,
    'trunk':       6.0,
    'primary':     5.0,
    'secondary':   4.0,
    'tertiary':    3.5,
    'residential': 3.0,
    'service':     2.5,
    'unclassified': 3.0,
    'track':       2.0,
    'path':        1.0,
    'footway':     0.8,
    'cycleway':    1.0,
}
DEFAULT_ROAD_HALF_WIDTH = 2.5

# Tracks/paths/footways/cycleways legitimately thread through canopy,
# so the foliage mask uses a smaller exclusion margin around them than
# around real roads. Centralised here so a new narrow class only needs
# to be declared once.
NARROW_ROAD_CLASSES = {'track', 'path', 'footway', 'cycleway'}


def half_width_for_props(props: dict) -> float:
    """Return the road's half-width in metres given OSM feature properties.

    Priority:
      1. Explicit ``width`` tag (in metres, optional trailing 'm'); halved.
      2. Per-class default from :data:`ROAD_HALF_WIDTH`.
      3. :data:`DEFAULT_ROAD_HALF_WIDTH` for unrecognised classes.

    Robust to missing tags and unparseable widths (e.g. ``width="auto"``).
    Always returns at least 0.5 m to avoid degenerate buffers.
    """
    if 'width' in props:
        try:
            return max(float(str(props['width']).rstrip(' m')) / 2.0, 0.5)
        except (TypeError, ValueError):
            pass
    highway = str(props.get('highway', '')).lower()
    return ROAD_HALF_WIDTH.get(highway, DEFAULT_ROAD_HALF_WIDTH)
