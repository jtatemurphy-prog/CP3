# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Technic liftarms for the round-3 mechanisms (wheel-legs, deployable booms).

The parts are real catalog entries (:data:`ai_pd_tasks.lego.catalog.LIFTARM_PART_TYPES`: BrickLink masses, LDraw
geometry converted by ``scripts/convert_lego_parts.py``). The converted USDs keep the LDraw frame, in which a
liftarm lies along local y with its holes along local z. The benchmark uses the Technic-brick convention instead:
**long axis local x, holes along local y** (so a liftarm turns about its holes with ``spin_deg``, and its holes line
up with bricks, axles and pins like a Technic brick's). :func:`write_usds` writes a copy of each converted USD into
``assets/data/lego_bench_parts/`` with its Visual / Collision children rotated into that frame, and the centre of
mass and principal axes rotated to match (``python -m ai_pd_tasks.lego_bench.liftarms``).

Bench frames (millimetres):

* straight thick liftarm 1 x n (32524 1x7, 40490 1x9, 32278 1x15): centred, holes at x = (k - (n - 1) / 2) * 8,
  z = 0, along y; 7.2 mm tall (z), 8 mm thick (y); kind ``technic_brick`` without studs (an axle can turn in a
  hole, a pin can join two holes);
* 60483 liftarm 1 x 2 with axle hole and pin hole: **origin on the axle (cross) hole**, the pin hole at x = +8 mm;
  kind ``axle_brick``: a ``cross`` fit locks it to an axle, and a Technic pin in its pin hole locks a liftarm beside
  it that the same axle passes through (the standard way to fix a beam to a turning axle).

Joint strength used in the mechanism checks: a Technic pin loaded in shear (ABS/POM, 4.8 mm)
carries hundreds of newtons, a cross fit several N m before the hole deforms; the weak joint of a LEGO structure is
the stud clutch (about 1 N per stud).
"""

from __future__ import annotations

import os

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD_DIR = os.path.join(HERE, "assets", "data", "lego_bench_parts")

M_USD_TO_BENCH = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
"""v_bench = M v_usd: the LDraw liftarm's long axis (usd y) becomes x, its hole axis (usd z) becomes y."""

LIFTARMS: dict[str, dict] = {
    "60483": dict(n=2, origin_hole=0, axle_holes=(0,), title="Technic liftarm thick 1x2 with axle hole and pin hole"),
    "32524": dict(n=7, title="Technic liftarm thick 1x7"),
    "40490": dict(n=9, title="Technic liftarm thick 1x9"),
    "32278": dict(n=15, title="Technic liftarm thick 1x15"),
    "7229": dict(n=3, origin_hole=1, axle_holes=(1,), title="Technic liftarm thick 1x3 with centre axle hole"),
    # -- inventory part set (2026-10-06): straight thick liftarms --
    "32523": dict(n=3, title="Technic liftarm thick 1x3"),
    "32316": dict(n=5, title="Technic liftarm thick 1x5"),
    "32525": dict(n=11, title="Technic liftarm thick 1x11"),
    "41239": dict(n=13, title="Technic liftarm thick 1x13"),
    # -- inventory part set: bent / shaped liftarms. ``holes``: hole centres (bench x, z) [mm], all along bench y, from
    # the LDraw hole primitives (beamhole / connhole / peghole / axlehol*); ``cross``: indices of the axle (cross)
    # holes; ``segs``: centre-line segments of the beam (collision envelope: 3.6 mm round the line, +-4 mm in y);
    # origin = the LDraw origin (the first hole) --
    "32140": dict(holes=((0, 0), (8, 0), (16, 0), (24, 0), (24, 8)), cross=(0,),
                  segs=(((0, 0), (24, 0)), ((24, 0), (24, 8))), title="Technic liftarm L 2x4 thick (90 deg)"),
    "32526": dict(holes=((0, 0), (8, 0), (16, 0), (24, 0), (32, 0), (32, 8), (32, 16)), cross=(),
                  segs=(((0, 0), (32, 0)), ((32, 0), (32, 16))), title="Technic liftarm L 3x5 thick (90 deg)"),
    "32348": dict(holes=((0, 0), (8, 0), (16, 0), (24, 0), (28.8, 6.4), (33.6, 12.8), (38.4, 19.2)), cross=(0, 6),
                  segs=(((0, 0), (24, 0)), ((24, 0), (38.4, 19.2))), title="Technic liftarm bent 1x7 (4-4, 53.13 deg)"),
    "32271": dict(holes=tuple((8.0 * k, 0) for k in range(7)) + ((52.8, 6.4), (57.6, 12.8)), cross=(0, 8),
                  segs=(((0, 0), (48, 0)), ((48, 0), (57.6, 12.8))), title="Technic liftarm bent 1x9 (7-3, 53.13 deg)"),
    "6629": dict(holes=tuple((8.0 * k, 0) for k in range(6)) + ((44.8, 6.4), (49.6, 12.8), (54.4, 19.2)), cross=(0, 8),
                 segs=(((0, 0), (40, 0)), ((40, 0), (54.4, 19.2))), title="Technic liftarm bent 1x9 (6-4, 53.13 deg)"),
    "60484": dict(holes=((0, 0), (8, 0), (16, -8), (16, 0), (16, 8)), cross=(),
                  segs=(((0, 0), (16, 0)), ((16, -8), (16, 8))), title="Technic liftarm T 3x3 thick"),
    "32056": dict(holes=((0, 0), (0, 8), (0, 16), (8, 0), (16, 0)), cross=(0, 2, 4), thin=True,
                  segs=(((0, 0), (0, 16)), ((0, 0), (16, 0))), title="Technic liftarm L 3x3 thin (4 mm)"),
    "32009": dict(holes=tuple((-64.0 + 8.0 * k, 0) for k in range(7)) + ((0, 16), (0, 24), (0, 32)), cross=(0, 9),
                  segs=(((-64, 0), (-16, 0)), ((-16, 0), (0, 16)), ((0, 16), (0, 32))),
                  title="Technic liftarm double bent 1x11.5 thick (45 deg)"),
    # -- round 2 (2026-10-06): thick frames with an open centre. Bench frame: long axis x, width z, thickness y (the
    # frame stands in the x-z plane like the shaped liftarms). ``holes``: through holes along y; ``inplane``: pin holes
    # in the plane of the frame (x, z, axis 'x' | 'z'), 8 mm long through one bar. All from the LDraw hole primitives
    # (peghole / connhole / connhol2 and their axes); no axle (cross) holes in either part --
    "64179": dict(holes=tuple((sx * x, sz * 16.0) for x in (8.0, 24.0) for sx in (-1, 1) for sz in (-1, 1)),
                  inplane=tuple((sx * x, sz * 16.0, "z") for x, sx in ((0.0, 1),) + tuple((16.0, s) for s in (-1, 1))
                                for sz in (-1, 1))
                  + tuple((sx * 24.0, z, "x") for sx in (-1, 1) for z in (-8.0, 0.0, 8.0)),
                  segs=(((-24, -16), (24, -16)), ((-24, 16), (24, 16)), ((-24, -16), (-24, 16)), ((24, -16), (24, 16))),
                  radius=4.0, title="Technic liftarm frame thick 5x7 open centre"),
    "64178": dict(holes=tuple((sx * x, sz * 16.0) for x in (8.0, 24.0) for sx in (-1, 1) for sz in (-1, 1)),
                  inplane=tuple((sx * x, sz * 16.0, "z") for x, sx in ((0.0, 1),) + tuple(
                      (x, s) for x in (16.0, 32.0, 40.0) for s in (-1, 1)) for sz in (-1, 1))
                  + tuple((sx * 24.0, z, "x") for sx in (-1, 1) for z in (-8.0, 0.0, 8.0)),
                  segs=(((-40, -16), (40, -16)), ((-40, 16), (40, 16)), ((-24, -16), (-24, 16)), ((24, -16), (24, 16))),
                  radius=4.0, title="Technic liftarm frame thick 5x11 open centre (cross bars 2 holes in from the ends)"),
}
FRAMES = ("64179", "64178")
"""Open-centre frames (round 2): their in-plane holes are ``pin_holes`` of the bench part (axes x / z)."""


def hole_xs(pid: str) -> tuple[float, ...]:
    """Hole centres along the bench x axis."""
    g = LIFTARMS[pid]
    if "holes" in g:
        return tuple(float(h[0]) for h in g["holes"])
    n = g["n"]
    if "origin_hole" in g:
        return tuple(8.0 * (k - g["origin_hole"]) for k in range(n))
    return tuple(8.0 * (k - (n - 1) / 2) for k in range(n))


def geometry(pid: str) -> dict:
    """BenchPart keyword arguments (without mass / usd) of a liftarm."""
    g = LIFTARMS[pid]
    if "holes" in g:
        return _shaped_geometry(pid, g)
    xs = hole_xs(pid)
    lo, hi = (min(xs) - 3.6, -4.0, -3.6), (max(xs) + 3.6, 4.0, 3.6)
    if "axle_holes" in g:  # 60483: axle hole at the origin, pin hole(s) elsewhere
        pins = tuple((x, 0.0, 0.0, "y") for i, x in enumerate(xs) if i not in g["axle_holes"])
        return dict(
            kind="axle_brick",
            title=g["title"],
            shape=_box(lo, hi),
            width=8.0,
            hub_mm=(-4.0, 4.0),
            pin_holes=pins,
            notes="cross hole along y at the origin, pin hole(s) at " + ", ".join(f"x = {p[0]:+.0f}" for p in pins),
        )
    return dict(
        kind="technic_brick",
        title=g["title"],
        shape=_box(lo, hi),
        holes=tuple((x, 0.0) for x in xs),
        notes="studless liftarm: holes along y at z = 0 (not 4 mm below the top like a brick)",
    )


def _shaped_geometry(pid: str, g: dict) -> dict:
    """Bent / L / T liftarms: pin holes in ``holes`` (x, z), the axle (cross) holes in ``axle_holes`` (an axle in one
    locks the liftarm to it: a ``cross`` connection with the liftarm as the child), collision envelope = the beam's
    centre-line segments (:class:`.parts.Segs`)."""
    from .parts import Segs  # noqa: PLC0415

    hw = 2.0 if g.get("thin") else 4.0
    rad = float(g.get("radius", 3.6))
    holes = [(float(x), float(z)) for x, z in g["holes"]]
    cross = set(g.get("cross", ()))
    segs = tuple((float(a[0]), float(a[1]), float(b[0]), float(b[1])) for a, b in g["segs"])
    xs = [h[0] for h in holes] + [s[i] for s in segs for i in (0, 2)]
    zs = [h[1] for h in holes] + [s[i] for s in segs for i in (1, 3)]
    lo, hi = (min(xs) - rad, -hw, min(zs) - rad), (max(xs) + rad, hw, max(zs) + rad)
    inplane = tuple((float(x), 0.0, float(z), a) for x, z, a in g.get("inplane", ()))
    extra = {"pin_holes": inplane} if inplane else {}
    return dict(
        kind="technic_brick",
        title=g["title"],
        shape=Segs(tuple(lo), tuple(hi), segs=segs, radius=rad, half_w=hw),
        holes=tuple(h for i, h in enumerate(holes) if i not in cross),
        axle_holes=tuple((h[0], 0.0, h[1], "y") for i, h in enumerate(holes) if i in cross),
        hole_half=hw,
        **extra,
        notes=f"{'thin ' if g.get('thin') else ''}shaped liftarm: pin holes "
        + ", ".join(f"({x:g}, {z:g})" for i, (x, z) in enumerate(holes) if i not in cross)
        + (", axle holes " + ", ".join(f"({x:g}, {z:g})" for i, (x, z) in enumerate(holes) if i in cross) if cross else "")
        + " (x, z mm, all along y)"
        + ("; in-plane pin holes " + ", ".join(f"({x:g}, {z:g}) along {a}" for x, _, z, a in inplane) if inplane else ""),
    )


def _box(lo, hi):
    from .parts import Box  # noqa: PLC0415

    return Box(tuple(lo), tuple(hi))


def usd_path(pid: str) -> str:
    return os.path.join(USD_DIR, f"{pid}.usda")


def source_usd(pid: str) -> str:
    from ai_pd_tasks.lego.catalog import PART_TYPES  # noqa: PLC0415

    return PART_TYPES[pid].usd_path


def write_usds(only=None) -> list[str]:
    """Copy each converted liftarm USD into the bench frame (see the module docstring); ``only``: these ids."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: PLC0415

    os.makedirs(USD_DIR, exist_ok=True)
    m = Gf.Matrix3d(*M_USD_TO_BENCH.T.flatten().tolist())  # Gf is row-vector: v' = v * m  <=>  v' = M v
    q_rot = m.ExtractRotation().GetQuat()
    out = []
    for pid in LIFTARMS:
        if only and pid not in only:
            continue
        src = source_usd(pid)
        stage = Usd.Stage.Open(src)
        layer = stage.Flatten()
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
            c = M_USD_TO_BENCH @ np.array([com[0], com[1], com[2]])
            mass.GetCenterOfMassAttr().Set(Gf.Vec3f(*c.tolist()))
        pa = mass.GetPrincipalAxesAttr().Get()
        if pa is not None:
            mass.GetPrincipalAxesAttr().Set(Gf.Quatf(q_rot * Gf.Quatd(pa)))
        st.GetRootLayer().Save()
        out.append(dst)
    return out


if __name__ == "__main__":
    import sys

    for p in write_usds(sys.argv[1:] or None):
        print(p)
