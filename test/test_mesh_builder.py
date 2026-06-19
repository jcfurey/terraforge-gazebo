# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Tests for the watertight building-mesh generator.

Pins the extrusion + roof geometry: vertex/height invariants, that pitched
roofs raise a ridge above the eave, that non-rectangular footprints fall back
to a flat roof, and that every emitted face winds outward (so gz-sim's
backface culling shows the building from outside, not inside).
"""

import math

import pytest

pytest.importorskip('shapely')

import shapely.geometry  # noqa: E402,I100,I202

from terraforge.data_processing import mesh_builder  # noqa: E402


def _read_obj(path):
    verts, faces = [], []
    with open(path, encoding='utf-8') as f:
        for line in f:
            if line.startswith('v '):
                _, x, y, z = line.split()[:4]
                verts.append((float(x), float(y), float(z)))
            elif line.startswith('f '):
                idx = [int(tok.split('//')[0]) for tok in line.split()[1:]]
                faces.append(tuple(i - 1 for i in idx))
    return verts, faces


def _normal(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return (nx / length, ny / length, nz / length)


def _assert_faces_point_outward(verts, faces):
    """For a convex solid, every face normal points away from the centroid."""
    cx = sum(v[0] for v in verts) / len(verts)
    cy = sum(v[1] for v in verts) / len(verts)
    cz = sum(v[2] for v in verts) / len(verts)
    for i, j, k in faces:
        a, b, c = verts[i], verts[j], verts[k]
        nx, ny, nz = _normal(a, b, c)
        fx = (a[0] + b[0] + c[0]) / 3.0 - cx
        fy = (a[1] + b[1] + c[1]) / 3.0 - cy
        fz = (a[2] + b[2] + c[2]) / 3.0 - cz
        assert nx * fx + ny * fy + nz * fz >= -1e-6, (i, j, k)


def _square(side=10.0):
    h = side / 2.0
    return shapely.geometry.Polygon(
        [(-h, -h), (h, -h), (h, h), (-h, h)]
    )


def test_flat_roof_box_vertices_and_height(tmp_path):
    path = str(tmp_path / 'b.obj')
    assert mesh_builder.generate_building_obj(path, _square(10.0), 6.0)
    verts, faces = _read_obj(path)
    assert len(verts) == 8                         # 4 base + 4 roof
    assert sorted({round(z, 3) for _, _, z in verts}) == [0.0, 6.0]
    for f in faces:
        assert all(0 <= i < len(verts) for i in f)
    _assert_faces_point_outward(verts, faces)


def test_gabled_roof_raises_ridge_above_eave(tmp_path):
    path = str(tmp_path / 'g.obj')
    poly = shapely.geometry.Polygon(
        [(-6, -3), (6, -3), (6, 3), (-6, 3)]
    )
    assert mesh_builder.generate_building_obj(
        path, poly, 6.0, roof_shape='gabled', roof_height=2.0
    )
    verts, faces = _read_obj(path)
    assert abs(max(z for _, _, z in verts) - 6.0) < 1e-6     # ridge
    assert any(abs(z - 4.0) < 1e-6 for _, _, z in verts)     # eave
    ridge = [v for v in verts if abs(v[2] - 6.0) < 1e-6]
    assert len(ridge) == 2
    _assert_faces_point_outward(verts, faces)


def test_pyramidal_roof_has_single_apex(tmp_path):
    path = str(tmp_path / 'p.obj')
    assert mesh_builder.generate_building_obj(
        path, _square(8.0), 5.0, roof_shape='pyramidal', roof_height=2.0
    )
    verts, _ = _read_obj(path)
    apex = [v for v in verts if abs(v[2] - 5.0) < 1e-6]
    assert len(apex) == 1


def test_non_rectangular_pitched_falls_back_to_flat(tmp_path):
    # L-shaped footprint: area is ~half its OBB, so a gabled request degrades
    # to a flat roof on the true footprint (no ridge, top at full height).
    poly = shapely.geometry.Polygon(
        [(0, 0), (10, 0), (10, 3), (3, 3), (3, 10), (0, 10)]
    )
    path = str(tmp_path / 'l.obj')
    assert mesh_builder.generate_building_obj(
        path, poly, 6.0, roof_shape='gabled', roof_height=2.0
    )
    verts, _ = _read_obj(path)
    assert abs(max(z for _, _, z in verts) - 6.0) < 1e-6
    top = [v for v in verts if abs(v[2] - 6.0) < 1e-6]
    bot = [v for v in verts if abs(v[2] - 0.0) < 1e-6]
    assert len(top) == len(bot) == 6


def test_unknown_roof_shape_is_flat(tmp_path):
    path = str(tmp_path / 'm.obj')
    assert mesh_builder.generate_building_obj(
        path, _square(10.0), 6.0, roof_shape='mansard'
    )
    verts, _ = _read_obj(path)
    assert sorted({round(z, 3) for _, _, z in verts}) == [0.0, 6.0]


def test_degenerate_polygon_returns_false(tmp_path):
    poly = shapely.geometry.Polygon([(0, 0), (0, 0), (0, 0)])
    assert mesh_builder.generate_building_obj(
        str(tmp_path / 'x.obj'), poly, 6.0
    ) is False


def test_ear_clip_square_is_two_triangles():
    tris = mesh_builder._ear_clip([(0, 0), (1, 0), (1, 1), (0, 1)])
    assert len(tris) == 2
    for t in tris:
        assert len(set(t)) == 3


def test_file_uri_is_percent_encoded():
    uri = mesh_builder.file_uri('/tmp/a b/c.obj')
    assert uri.startswith('file:///')
    assert ' ' not in uri
