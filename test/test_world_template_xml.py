# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Regression tests: rendered worlds must be strict well-formed XML.

Two real-world failures pinned here:

* The template's <physics> element shipped without the `type` attribute,
  which the SDF spec marks required — libsdformat rejected every
  generated world ("Required attribute[type] in element[physics] is not
  specified" via `gz sdf --check`).
* Template comments contained double hyphens (CLI flags like `levels`
  spelled with their leading dashes), which the XML spec forbids inside
  comments — strict parsers (ElementTree, lxml, most ROS tooling)
  refused the file even though tinyxml2 happens to tolerate it.
"""
import xml.etree.ElementTree as ET

import pytest

pytest.importorskip('jinja2')
pytest.importorskip('shapely')

from terraforge.data_processing import sdf_builder  # noqa: E402


def _render(**overrides):
    import os
    template_dir = os.path.join(
        os.path.dirname(sdf_builder.__file__), 'templates',
    )
    builder = sdf_builder.SDFWorldBuilder(template_dir)
    kwargs = {
        'heightmap_path': '/tmp/x/heightmap.png',
        'texture_path': '/tmp/x/satellite_texture.jpg',
        'flat_normal_path': '/tmp/x/flat_normal.png',
        'buildings': [], 'trees': [], 'roads': [],
        'extent_meters': 600.0,
        'height_amplitude': 42.0,
        'terrain_z_offset': -3.0,
        'origin_lat': 37.7749, 'origin_lon': -122.4194, 'origin_elev_m': 12.0,
    }
    kwargs.update(overrides)
    return builder.render_world_template(**kwargs)


def test_rendered_world_is_strict_well_formed_xml():
    root = ET.fromstring(_render())
    assert root.tag == 'sdf'


def test_physics_element_declares_required_type_attribute():
    root = ET.fromstring(_render())
    physics = root.find('./world/physics')
    assert physics is not None
    assert physics.get('type'), (
        '<physics> must carry the SDF-required type attribute or '
        'libsdformat rejects the world'
    )


def test_no_double_hyphen_inside_comments():
    # ET.fromstring already fails on `--` inside comments, but spell the
    # invariant out so the failure message names the actual rule.
    rendered = _render()
    import re
    for m in re.finditer(r'<!--(.*?)-->', rendered, re.S):
        assert '--' not in m.group(1), (
            'XML forbids double hyphens inside comments; found one in: '
            f'{m.group(1)[:80]!r}'
        )


def test_streaming_disabled_render_is_also_well_formed():
    root = ET.fromstring(_render(enable_level_streaming=False))
    assert root.tag == 'sdf'


def test_physics_engine_is_dartsim():
    # dartsim is mandatory: it's the only gz-physics backend that supports
    # both heightmap and mesh collision (and skid-steer via fdir1).
    root = ET.fromstring(_render())
    physics = root.find('./world/physics')
    assert physics.get('type') == 'dartsim'
    engine_files = [
        e.text for e in
        (p.find('engine/filename') for p in root.findall('./world/plugin'))
        if e is not None
    ]
    assert 'gz-physics-dartsim-plugin' in engine_files, engine_files


def test_heightmap_world_has_terrain_collision_and_no_flat_plane():
    # With a DEM, the terrain heightmap is the driving surface and the flat
    # ground plane is omitted to avoid a conflicting second floor.
    root = ET.fromstring(_render())  # default kwargs include a heightmap
    models = {m.get('name'): m for m in root.findall('./world/model')}
    assert 'terrain' in models
    assert models['terrain'].find('./link/collision/geometry/heightmap') is not None
    assert 'ground_plane' not in models


def test_flat_world_keeps_ground_plane_collision():
    # Without a DEM, fall back to the flat collision+visual ground plane.
    root = ET.fromstring(_render(heightmap_path=None))
    models = {m.get('name'): m for m in root.findall('./world/model')}
    assert 'ground_plane' in models
    assert models['ground_plane'].find('./link/collision/geometry/plane') is not None
    assert 'terrain' not in models
