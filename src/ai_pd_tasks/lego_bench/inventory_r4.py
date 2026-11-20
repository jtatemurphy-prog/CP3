# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Round 4a of the inventory part library (2026-10-14): knob and bevel-half gears, the large tread sprocket, the
3-axle hub, the bent pin connector, the cam, the changeover catches, the worm gearbox and the steering arm.

Bench geometry of the parts is in :data:`.parts.INVENTORY_GEOMETRY` (round 4a section); this module holds what is
specific to them: the bench frames of the parts whose converted (LDraw) frame does not fit the bench convention, and
the mesh rules of the new gears, all derived from the LDraw geometry.

Bench frames
------------
Design parts only yaw (about the world z) and spin (about their local y), so a part's turning / main hole axis must be
its local y (the gear / liftarm convention). Four parts are rotated into that frame (:data:`BENCH_FRAMES`,
``v_bench = M v_usd``; :func:`write_usds` writes ``assets/data/lego_bench_parts/<id>.usda`` with the Visual /
Collision children, the centre of mass and the principal axes rotated, like :func:`.liftarms.write_usds`):

* 57585 hub: its cross hole (LDraw / USD z) -> bench y; its three 1L axles lie in the bench x-z plane (one along +z);
* 55615 bent pin connector: its corner and end pin holes (USD x) -> bench y; the L lies in the x-z plane (arms along
  +x and +z);
* 6641 changeover catch: its through cross hole (USD x) -> bench y; the lever runs along -x;
* 57515 steering arm: its fork holes (USD x) -> bench y; the arm runs along -x (fork at x = 0..-16, the tow ball
  socket at x = -32..-44).

The other seven keep the converted frame (gears, sprocket, cam, rotary catch: axis local y; gearbox: studs down).

Mesh rules (derived from the LDraw geometry)
--------------------------------------------
Method (``scripts/inventory_r4_mesh_scan.py``): each gear's LDraw mesh is voxelised at 0.1 mm (surface + flood
fill; volumes within 10-20 % of the BrickLink mass / 1.05 g/cm3). Two copies are posed at a candidate distance and
turned together at the gear ratio over a tooth pitch (24-90 relative phases x 8-30 steps); a pose *meshes* when, at
the best phase, no point of one gear is more than 0.15 mm inside the other (the voxel resolution), and *engages* when
no fixed phase of one gear lets the other turn a
full revolution untouched (the teeth would skip). Distances are measured from each gear's centre (its origin, the
middle of its bore) to the other gear's axis; right-angle pairs need intersecting axes (<= 1 mm apart, like the
single bevels).

* **Knob wheel 32072** (4 knobs at r 8.8 mm; knob discs r 3.2 in the 1.6 mm mid plate, pegs r 1.6 to +-2.8 mm):
  - right angles, 1 : 1, with another 32072 (:func:`knob_right_angle_problem`). With dA, dB the two distances (m =
    min, M = max): clear when m >= 8.8, or m >= 7.8 and M >= 11.8 (one wheel on the stud grid, the other half a stud
    further out); skips when m > 11.2, or M > 13.2 unless m < 8.5 (then M <= 14.2). Typical: (8, 12), (10, 10),
    (9, 13). Both on the 1-stud grid (8, 8) is rejected: the knob pegs dig 0.48 mm into the other wheel's knob discs
    in the LDraw model.
  - parallel, 1 : 1 (opposite senses): centre distance 16.8..20.0 mm (16 = 2 studs overlaps 0.32 mm; skips from
    ~20.5); the 2 x 1 stud diagonal (17.9 mm) is on the grid.
* **Bevel halves 69762 (14T, thick, cross hole) and 69761 (22T, round pin hole)**: one-sided bevels, teeth facing
  local -y (tooth tip / root lines: 14T ~31 deg pitch cone, apex ~17 mm on -y; 22T ~59 deg, apex ~8.7 mm on -y:
  31 + 59 = 90, the pair is made for each other).
  - 14T / 22T at right angles, ratio 14 : 22: the 14T's centre 16 mm (2 studs) from the 22T's axis and the 22T's
    centre 8 mm (1 stud) from the 14T's axis, both on their toothed side of the intersection. Accepted for
    d14 >= 15.8, d22 >= 7.8, d14 + d22 <= 27 (skips beyond). The grid position (16, 8) overlaps 0.18 mm in the LDraw
    model (fine phase scan), within its modelling accuracy; it is clear from (16.5, 8) or (16.2, 8.2).
  - 14T / 14T at right angles, 1 : 1: each centre 12 mm (1.5 studs) from the other axis, window 11.5..12.5 (11 mm:
    0.53 mm interference, 13 mm: skips): the cones (2 x 31 deg) do not match, the teeth only touch near their tips;
    a shallow mesh.
  - 22T / 22T: no right-angle mesh (>= 0.2 mm interference up to 15.5 mm, skips from 16 mm): rejected.
  - with any other gear (the 12T / 20T single bevels, double bevels, spurs): rejected; the 69761 / 69762 never mesh
    with parallel axes.
  - the 22T has a round bore: it turns freely on its axle (``free_bore``, a hinge like the 18946), so it is an idler
    between two 14T (e.g. two coaxial 14T on either side: the output turns the other way, 1 : 1).
* Simulation: every mesh of these gears (parallel knob meshes too) is a PhysX gear joint at the tooth ratio
  (``env.gear_joint_specs``: the convex hulls cannot carry knobs or bevel teeth); the two gears do not collide with
  each other.
"""

from __future__ import annotations

import os

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD_DIR = os.path.join(HERE, "assets", "data", "lego_bench_parts")

M_HUB = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
"""57585: bench (x, y, z) = (x, -z, y) of the USD frame: the cross hole (USD z) along y, the arms in the x-z plane."""
M_X_TO_Y = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
"""55615 / 6641 / 57515: bench (x, y, z) = (y, -x, z) of the USD frame: the USD x holes along y, z kept (up)."""

BENCH_FRAMES: dict[str, np.ndarray] = {"57585": M_HUB, "55615": M_X_TO_Y, "6641": M_X_TO_Y, "57515": M_X_TO_Y}
"""Parts rotated into the bench frame (``v_bench = M v_usd``), see the module docstring."""

KNOB = "32072"
BEVEL_14, BEVEL_22 = "69762", "69761"
R4_MESH_PARTS = frozenset((KNOB, BEVEL_14, BEVEL_22))
"""Gears whose meshes follow this module's rules (:func:`mesh_problems`) and are always PhysX gear joints."""
BEVEL_FACE = {BEVEL_14: -1.0, BEVEL_22: -1.0}
"""Side (sign of the local y) the teeth of the one-sided bevel halves face: the axes' intersection lies on it."""
BEVEL_PAIRS: dict[tuple[str, str], dict] = {
    (BEVEL_14, BEVEL_22): dict(nominal=(16.0, 8.0), lo=(15.8, 7.8), sum_max=27.0, ratio="14 : 22"),
    (BEVEL_14, BEVEL_14): dict(nominal=(12.0, 12.0), lo=(11.5, 11.5), hi=(12.5, 12.5), ratio="1 : 1"),
}
"""Right-angle bevel-half pairs (a, b): nominal distances of a's and b's centres from the other gear's axis [mm] and
the window where the LDraw teeth clear and engage (``lo``, ``hi`` per gear, ``sum_max`` on the sum)."""
KNOB_PARALLEL_MM = (16.8, 20.0)
"""Centre distance window of two parallel knob wheels [mm] (clear from 16.8, skip beyond ~20.5)."""
HOUSINGS = frozenset(("6588",))
"""Gearbox housings: collisions between the housing and the parts on the shafts that turn in it are filtered in
simulation (``env.housing_pairs``)."""
AXES_TOL_MM = 1.0
"""Right-angle meshes: largest distance between the two axis lines (they must intersect)."""


def usd_path(pid: str) -> str:
    return os.path.join(USD_DIR, f"{pid}.usda")


def write_usds(only=None) -> list[str]:
    """Copy each converted USD of :data:`BENCH_FRAMES` into the bench frame (``only``: these ids)."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: PLC0415

    from ai_pd_tasks.lego.catalog import PART_TYPES  # noqa: PLC0415

    os.makedirs(USD_DIR, exist_ok=True)
    out = []
    for pid, M in BENCH_FRAMES.items():
        if only and pid not in only:
            continue
        m = Gf.Matrix3d(*M.T.flatten().tolist())  # Gf is row-vector: v' = v * m  <=>  v' = M v
        q_rot = m.ExtractRotation().GetQuat()
        layer = Usd.Stage.Open(PART_TYPES[pid].usd_path).Flatten()
        dst = usd_path(pid)
        layer.Export(dst)
        st = Usd.Stage.Open(dst)
        part = st.GetPrimAtPath("/Part")
        for child in ("Visual", "Collision"):
            x = UsdGeom.Xformable(st.GetPrimAtPath(f"/Part/{child}"))
            x.ClearXformOpOrder()
            x.AddOrientOp().Set(Gf.Quatf(q_rot))
        mass = UsdPhysics.MassAPI(part)
        com = mass.GetCenterOfMassAttr().Get()
        if com is not None:
            mass.GetCenterOfMassAttr().Set(Gf.Vec3f(*(M @ np.array([com[0], com[1], com[2]])).tolist()))
        pa = mass.GetPrincipalAxesAttr().Get()
        if pa is not None:
            mass.GetPrincipalAxesAttr().Set(Gf.Quatf(q_rot * Gf.Quatd(pa)))
        st.GetRootLayer().Save()
        out.append(dst)
    return out


def knob_right_angle_problem(d_a: float, d_b: float) -> str:
    """'' when two knob wheels at right angles, their centres ``d_a`` / ``d_b`` mm from the other wheel's axis, clear
    and engage (LDraw sweep, see the module docstring); else what is wrong."""
    m, mx = min(d_a, d_b), max(d_a, d_b)
    if not (m >= 8.8 or (m >= 7.8 and mx >= 11.8)):
        return "the knobs jam (too close: use e.g. 8 / 12, 10 / 10 or 9 / 13 mm)"
    if m > 11.2 or (mx > 13.2 and not (m < 8.5 and mx <= 14.2)):
        return "the knobs skip (too far apart: use e.g. 8 / 12, 10 / 10 or 9 / 13 mm)"
    return ""


def _intersecting(tag: str, a, b) -> tuple[list[str], float, float]:
    """Right-angle pair: problems with the axes, and each gear's signed distance (along its own axis, from its centre)
    to the axes' intersection."""
    da, db = a.axis(), b.axis()
    n = np.cross(da, db)
    gap = abs(float(np.dot(b.p - a.p, n / np.linalg.norm(n))))
    errs = [f"{tag}: the gear axes do not intersect ({gap:.2f} mm apart)"] if gap > AXES_TOL_MM else []
    return errs, float(np.dot(b.p - a.p, da)), float(np.dot(a.p - b.p, db))


def mesh_problems(tag: str, a, b) -> list[str] | None:
    """Validator rules of a gear mesh involving a round-4a gear (:data:`R4_MESH_PARTS`); ``a`` / ``b`` are
    ``design.Placed``. None when neither gear is one (the earlier rules apply)."""
    pa, pb = a.inst.part, b.inst.part
    if pa not in R4_MESH_PARTS and pb not in R4_MESH_PARTS:
        return None
    perpendicular = abs(float(np.dot(a.axis(), b.axis()))) < 1e-3
    if KNOB in (pa, pb):
        if pa != pb:
            return [f"{tag}: a knob wheel (32072) meshes only with another knob wheel"]
        if perpendicular:
            errs, ta, tb = _intersecting(tag, a, b)
            p = knob_right_angle_problem(abs(ta), abs(tb))
            if p:
                errs.append(f"{tag}: knob wheels {abs(ta):.2f} / {abs(tb):.2f} mm from the other's axis: {p}")
            return errs
        if abs(float(np.dot(a.axis(), b.axis()))) < 0.999:
            return [f"{tag}: knob wheels mesh with parallel or perpendicular axes only"]
        d = b.p - a.p
        axial = float(np.dot(d, a.axis()))
        centre = float(np.linalg.norm(d - axial * a.axis()))
        errs = []
        if abs(axial) > 1.0:
            errs.append(f"{tag}: knob wheels are {abs(axial):.2f} mm apart along the axis (not in one plane)")
        lo, hi = KNOB_PARALLEL_MM
        if not lo <= centre <= hi:
            errs.append(f"{tag}: parallel knob wheels {centre:.2f} mm apart (they mesh at {lo}..{hi} mm, e.g. the "
                        f"2 x 1 stud diagonal 17.9; 16 mm jams)")
        return errs
    if not perpendicular:
        return [f"{tag}: the bevel halves 69761 / 69762 mesh only at right angles"]
    key = (pa, pb) if (pa, pb) in BEVEL_PAIRS else (pb, pa)
    if key not in BEVEL_PAIRS:
        return [f"{tag}: {pa} / {pb} do not mesh (right-angle pairs of the bevel halves: 14T 69762 with 22T 69761 "
                f"(14 : 22) or with another 14T (1 : 1))"]
    g = BEVEL_PAIRS[key]
    x, y = (a, b) if (pa, pb) == key else (b, a)
    errs, t1, t2 = _intersecting(tag, x, y)
    for gear, t in ((x, t1), (y, t2)):
        if t * BEVEL_FACE[gear.inst.part] < 0:
            errs.append(f"{tag}: {gear.inst.name}'s teeth face away from the other gear (its toothed face, local "
                        f"{'-' if BEVEL_FACE[gear.inst.part] < 0 else '+'}y, must face the axes' intersection)")
    d1, d2 = abs(t1), abs(t2)
    lo, hi = g["lo"], g.get("hi", (np.inf, np.inf))
    if d1 < lo[0] or d2 < lo[1] or d1 > hi[0] or d2 > hi[1] or d1 + d2 > g.get("sum_max", np.inf):
        errs.append(f"{tag}: gear centres {d1:.2f} / {d2:.2f} mm from the other's axis ({key[0]} / {key[1]}: nominal "
                    f"{g['nominal'][0]:g} / {g['nominal'][1]:g} mm, ratio {g['ratio']})")
    return errs


def joint_mesh(part_a: str, part_b: str) -> bool:
    """A mesh the simulator always models as a PhysX gear joint (also with parallel axes: knob wheels)."""
    return part_a in R4_MESH_PARTS or part_b in R4_MESH_PARTS


if __name__ == "__main__":
    import sys

    for p in write_usds(sys.argv[1:] or None):
        print(p)
