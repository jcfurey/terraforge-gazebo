# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Trees must carry a trunk-cylinder collision (canopy stays visual-only).

With dartsim heightmap collision, trees sit on collidable terrain; a trunk
collision is what stops a rover driving through the trunk. Cartoon and Fuel
paths both need it — the canopy must NOT collide so the rover can pass under
the crown.
"""

import xml.etree.ElementTree as ET

import pytest

pytest.importorskip('numpy')   # tree_processor imports numpy at module load
pytest.importorskip('shapely')

from terraforge.data_processing.tree_processor import (  # noqa: E402
    _fuel_wrapper_sdf,
    _tree_link_sdf,
    TREE_VARIANTS,
)


def test_cartoon_tree_has_exactly_one_trunk_collision():
    for variant in range(TREE_VARIANTS):
        link = ET.fromstring(_tree_link_sdf('tree_x', (1.0, 2.0, 3.0), variant))
        collisions = link.findall('collision')
        # Exactly one collision (the trunk); canopy visuals carry none.
        assert len(collisions) == 1, variant
        assert collisions[0].find('geometry/cylinder') is not None, variant


def test_fuel_wrapper_has_trunk_collision_only():
    for variant in range(TREE_VARIANTS):
        wrapper = ET.fromstring(_fuel_wrapper_sdf(f'tree_fuel_{variant}', variant))
        link = wrapper.find('model/link')
        assert link is not None, variant
        collisions = link.findall('collision')
        assert len(collisions) == 1, variant
        assert collisions[0].find('geometry/cylinder') is not None, variant
