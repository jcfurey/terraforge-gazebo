# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Watertight building-mesh generation (extruded footprint + optional roof).

Replaces the old ``<polyline>`` visual + axis-aligned-bbox collision with a
single OBJ mesh per building, used for BOTH the SDF ``<visual>`` and
``<collision>`` so a robot collides with the real footprint, not its bounding
box. The mesh is:

  * walls: the footprint exterior ring extruded from z=0 to the eave,
  * a base cap (so the prism is closed / watertight),
  * a roof: flat (default) or a pitched gable / hip / pyramid.

Pitched roofs are only built when the footprint is close to rectangular
(area within ``RECT_THRESHOLD`` of its minimum rotated rectangle); the roof is
constructed on that rectangle so walls and roof stay watertight without a
straight-skeleton solver. Non-rectangular footprints fall back to a flat roof
on the true footprint. Everything is dependency-free (a small ear-clipping
triangulator + OBJ writer); no trimesh / earcut import.

Frame: vertices are in the building's local frame — XY relative to the model
origin (the caller centres the footprint on its centroid), Z from 0 at the
base upward — so the SDF ``<pose>`` places the mesh on the terrain.
"""

import math
import os
from urllib.parse import quote

# A footprint counts as "rectangular enough" for a pitched roof when its area
# fills at least this fraction of its minimum rotated rectangle. Below it, the
# OBB roof would leave gaps over the footprint, so we use a flat roof instead.
RECT_THRESHOLD = 0.9
# Keep at least this much vertical wall under a pitched roof so a tall roof
# height doesn't eat the whole building.
MIN_WALL_HEIGHT_M = 2.0

PITCHED_SHAPES = ('gabled', 'hipped', 'pyramidal')


def file_uri(path: str) -> str:
    """Return a percent-encoded ``file://`` URI for an absolute mesh path."""
    return 'file://' + quote(os.path.abspath(path), safe='/')


def _signed_area(ring) -> float:
    """Shoelace signed area of a 2D ring (CCW positive)."""
    s = 0.0
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        s += x0 * y1 - x1 * y0
    return 0.5 * s


def _ensure_ccw_open(ring):
    """Return the ring as an open (no repeated last point), CCW vertex list."""
    pts = [(float(x), float(y)) for x, y in ring]
    if len(pts) >= 2 and pts[0] == pts[-1]:
        pts = pts[:-1]
    # Drop consecutive duplicates that would create zero-length wall quads.
    dedup = []
    for p in pts:
        if not dedup or (abs(p[0] - dedup[-1][0]) > 1e-9
                         or abs(p[1] - dedup[-1][1]) > 1e-9):
            dedup.append(p)
    if len(dedup) >= 2 and dedup[0] == dedup[-1]:
        dedup = dedup[:-1]
    if _signed_area(dedup) < 0:
        dedup = dedup[::-1]
    return dedup


def _cross_z(a, b, c) -> float:
    """Z component of (b-a) x (c-a) — >0 for a CCW (left) turn."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _point_in_tri(p, a, b, c) -> bool:
    """True if ``p`` lies inside triangle ``a,b,c`` (CCW), edges inclusive."""
    d1 = _cross_z(a, b, p)
    d2 = _cross_z(b, c, p)
    d3 = _cross_z(c, a, p)
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
    return not (has_neg and has_pos)


def _ear_clip(ring):
    """Triangulate a simple CCW polygon (no holes) into index triples.

    Plain O(n^2) ear clipping — building footprints have a handful of
    vertices, so this is more than fast enough and avoids a triangulation
    dependency. Returns indices into ``ring``.
    """
    n = len(ring)
    if n < 3:
        return []
    idx = list(range(n))
    tris = []
    guard = 0
    while len(idx) > 3 and guard < 5 * n + 10:
        guard += 1
        m = len(idx)
        ear = False
        for i in range(m):
            i0, i1, i2 = idx[(i - 1) % m], idx[i], idx[(i + 1) % m]
            a, b, c = ring[i0], ring[i1], ring[i2]
            if _cross_z(a, b, c) <= 0:
                continue  # reflex or collinear, not an ear tip
            if any(j not in (i0, i1, i2) and _point_in_tri(ring[j], a, b, c)
                   for j in idx):
                continue
            tris.append((i0, i1, i2))
            idx.pop(i)
            ear = True
            break
        if not ear:
            break  # numerically degenerate; stop with what we have
    if len(idx) == 3:
        tris.append((idx[0], idx[1], idx[2]))
    return tris


class _Mesh:
    """Tiny vertex/triangle accumulator with positional vertex dedup."""

    def __init__(self):
        self.verts = []
        self.faces = []
        self._index = {}

    def vertex(self, x, y, z) -> int:
        key = (round(x, 4), round(y, 4), round(z, 4))
        i = self._index.get(key)
        if i is None:
            i = len(self.verts)
            self.verts.append((float(x), float(y), float(z)))
            self._index[key] = i
        return i

    def tri(self, a, b, c):
        if a != b and b != c and a != c:
            self.faces.append((a, b, c))


def _add_walls(mesh, ring_xy, z0, z1):
    """Extrude ring edges into outward-facing wall quads (z0 -> z1)."""
    ring = _ensure_ccw_open(ring_xy)
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        b0 = mesh.vertex(x0, y0, z0)
        b1 = mesh.vertex(x1, y1, z0)
        t0 = mesh.vertex(x0, y0, z1)
        t1 = mesh.vertex(x1, y1, z1)
        # CCW-from-outside winding -> outward normals for a CCW ring.
        mesh.tri(b0, b1, t1)
        mesh.tri(b0, t1, t0)


def _add_cap(mesh, ring_xy, z, facing_up):
    """Triangulate the ring at height ``z`` as a horizontal cap."""
    ring = _ensure_ccw_open(ring_xy)
    tris = _ear_clip(ring)
    vid = [mesh.vertex(x, y, z) for x, y in ring]
    for a, b, c in tris:
        if facing_up:
            mesh.tri(vid[a], vid[b], vid[c])      # +Z normal
        else:
            mesh.tri(vid[a], vid[c], vid[b])      # -Z normal (base)


def _build_flat(mesh, ring_xy, height):
    """Closed prism: walls + base cap (down) + roof cap (up)."""
    _add_walls(mesh, ring_xy, 0.0, height)
    _add_cap(mesh, ring_xy, 0.0, facing_up=False)
    _add_cap(mesh, ring_xy, height, facing_up=True)


def _obb_axes(polygon):
    """Return (center, u, v, L, W) of the footprint's min rotated rectangle.

    ``u`` is the unit long-axis, ``v`` the unit short-axis, L>=W the side
    lengths. Returns None if the OBB is degenerate.
    """
    obb = polygon.minimum_rotated_rectangle
    if obb.geom_type != 'Polygon':
        return None
    pts = list(obb.exterior.coords)[:4]
    if len(pts) < 4:
        return None
    cx = sum(p[0] for p in pts) / 4.0
    cy = sum(p[1] for p in pts) / 4.0
    e1 = (pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
    e2 = (pts[2][0] - pts[1][0], pts[2][1] - pts[1][1])
    len1 = math.hypot(*e1)
    len2 = math.hypot(*e2)
    long_vec, length_l, length_w = ((e1, len1, len2) if len1 >= len2
                                    else (e2, len2, len1))
    if length_l <= 1e-6 or length_w <= 1e-6:
        return None
    u = (long_vec[0] / length_l, long_vec[1] / length_l)
    v = (-u[1], u[0])
    return (cx, cy), u, v, length_l, length_w


def _default_roof_height(short_side_m: float) -> float:
    """Pick a believable roof rise (~25-30 deg pitch) from the short side."""
    return max(1.5, min(0.35 * short_side_m, 5.0))


def _build_pitched(mesh, center, u, v, length_l, length_w, eave, ridge_h,
                   shape):
    """Build walls (0..eave) + a gable/hip/pyramid roof on the OBB rectangle."""
    cx, cy = center
    hu, hv = length_l / 2.0, length_w / 2.0
    zr = eave + ridge_h

    def corner(a, b):
        return (cx + a * u[0] + b * v[0], cy + a * u[1] + b * v[1])

    ax, bx = corner(-hu, -hv), corner(hu, -hv)
    cc, dx = corner(hu, hv), corner(-hu, hv)
    rect = [ax, bx, cc, dx]  # CCW

    _add_walls(mesh, rect, 0.0, eave)
    _add_cap(mesh, rect, 0.0, facing_up=False)

    av = mesh.vertex(ax[0], ax[1], eave)
    bv = mesh.vertex(bx[0], bx[1], eave)
    cv = mesh.vertex(cc[0], cc[1], eave)
    dv = mesh.vertex(dx[0], dx[1], eave)

    if shape == 'pyramidal' or length_l <= length_w * 1.05:
        apex = mesh.vertex(cx, cy, zr)
        for p, q in ((av, bv), (bv, cv), (cv, dv), (dv, av)):
            mesh.tri(p, q, apex)
        return

    inset = min(hv if shape == 'hipped' else 0.0, hu * 0.95)
    r0 = mesh.vertex(cx - (hu - inset) * u[0], cy - (hu - inset) * u[1], zr)
    r1 = mesh.vertex(cx + (hu - inset) * u[0], cy + (hu - inset) * u[1], zr)
    # Long slopes.
    mesh.tri(av, bv, r1)
    mesh.tri(av, r1, r0)
    mesh.tri(dv, r0, r1)
    mesh.tri(dv, r1, cv)
    # End faces: gable triangles when inset==0, hip slopes when inset>0.
    mesh.tri(bv, cv, r1)
    mesh.tri(dv, av, r0)


def _face_normal(a, b, c):
    """Unit normal of triangle a,b,c; falls back to +Z if degenerate."""
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length < 1e-12:
        return (0.0, 0.0, 1.0)
    return (nx / length, ny / length, nz / length)


def write_obj(path: str, verts, faces) -> None:
    """Write a Wavefront OBJ with per-face normals (one ``vn`` per triangle)."""
    lines = ['# Generated by TerraForge',
             f'# verts {len(verts)} tris {len(faces)}']
    for x, y, z in verts:
        lines.append(f'v {x:.4f} {y:.4f} {z:.4f}')
    for i, j, k in faces:
        nx, ny, nz = _face_normal(verts[i], verts[j], verts[k])
        lines.append(f'vn {nx:.4f} {ny:.4f} {nz:.4f}')
    for n, (i, j, k) in enumerate(faces, start=1):
        lines.append(f'f {i + 1}//{n} {j + 1}//{n} {k + 1}//{n}')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def generate_building_obj(path: str, polygon, height: float, *,
                          roof_shape: str = 'flat',
                          roof_height: float = None) -> bool:
    """Write a watertight building OBJ to ``path``; return True on success.

    ``polygon`` is a shapely Polygon in the building's local frame (centred on
    its centroid). ``height`` is the total ridge height in meters. A pitched
    ``roof_shape`` (gabled / hipped / pyramidal) is honoured only for
    rectangular-ish footprints; otherwise the roof is flat. Any failure returns
    False so the caller can fall back to a bbox body.
    """
    try:
        if polygon is None or polygon.is_empty or polygon.geom_type != 'Polygon':
            return False
        poly = polygon if polygon.is_valid else polygon.buffer(0)
        if poly.is_empty or poly.geom_type != 'Polygon':
            return False
        height = float(height)
        if height <= 0:
            return False
        ring = _ensure_ccw_open(list(poly.exterior.coords))
        if len(ring) < 3:
            return False

        mesh = _Mesh()
        shape = roof_shape if roof_shape in PITCHED_SHAPES else 'flat'
        built_pitched = False
        if shape in PITCHED_SHAPES:
            axes = _obb_axes(poly)
            if axes is not None:
                center, u, v, length_l, length_w = axes
                obb_area = length_l * length_w
                if obb_area > 0 and (poly.area / obb_area) >= RECT_THRESHOLD:
                    rh = (roof_height if (roof_height and roof_height > 0)
                          else _default_roof_height(length_w))
                    rh = min(rh, height * 0.6)
                    eave = height - rh
                    if eave >= MIN_WALL_HEIGHT_M:
                        _build_pitched(mesh, center, u, v, length_l, length_w,
                                       eave, rh, shape)
                        built_pitched = True

        if not built_pitched:
            _build_flat(mesh, ring, height)

        if not mesh.faces:
            return False
        write_obj(path, mesh.verts, mesh.faces)
        return True
    except Exception:
        return False
