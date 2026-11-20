"""Motor-mount model (2026-10-21, calibration from the real E0 trials): a PF motor held only by studs lifts off the
bricks under it when it pulls hard, and its pinion loses the mesh (seen on the bench and heard: the motor whines
free while the wheels stop). The rigid simulation keeps every mesh, so this is checked from the motor's torque:

* offline, per motor, the structural model (``clutch.Structure``) at the design pose is loaded with 1 N m of motor
  torque: the reaction on the motor body about its output axis plus the mesh force on its pinion (tangential T / r and
  separating T / r tan 20 deg, r = teeth / 2 mm) -> ``c`` = the largest pull-off force on any stud of the motor's
  own stud joints per N m (both senses);
* during the run, the motor's torque is estimated from its drive (tau = D (|v_target| - |v|), capped at the stall
  torque) and smoothed over 0.1 s; when ``c`` |tau| exceeds the per-stud pull-off strength
  (``LEGO_BENCH_MOUNT_PULL_N``) the run fails: "motor mount lifted".

A motor fixed with any Technic pin (L / XL motors, servo, or an M motor pinned through its holes) is not checked:
pins hold it."""
from __future__ import annotations

import math
import os

import numpy as np

MOUNT_PULL_N = float(os.environ.get("LEGO_BENCH_MOUNT_PULL_N", "0") or 0.0)
"""Per-stud pull-off strength of a motor's stud mount [N]; 0 = model off (the E0 simulator)."""
PRESSURE = math.radians(20.0)


def mount_coefficients(design, frames=None) -> dict[str, dict]:
    """{motor: {"c": N of stud pull-off per N m, "joint": label}} for the stud-mounted PF motors that drive a gear."""
    from .clutch import Structure, design_frames  # noqa: PLC0415
    from .design import shafts  # noqa: PLC0415
    from .parts import bench_parts  # noqa: PLC0415

    cat = bench_parts()
    frames = frames or design_frames(design)
    parts = {p.name: p for p in design.parts}
    st = Structure(design, frames, design.chassis)
    sh = shafts(design)
    out = {}
    for mc in design.connections:
        if mc.kind != "motor" or mc.parent not in parts or cat[parts[mc.parent].part].kind != "motor":
            continue
        m = mc.parent
        if cat[parts[m].part].servo:
            continue
        if any(c.kind == "pin" and m in (c.parent, c.child) for c in design.connections):
            continue
        if not any(c.kind == "stud" and m in (c.parent, c.child) for c in design.connections):
            continue
        child_kind = cat[parts[mc.child].part].kind
        ax = frames[mc.child][1][:, 0] if child_kind == "axle" else frames[m][1][:, 1]
        ax = ax / np.linalg.norm(ax)
        sid = sh.get(mc.child, mc.child)
        on = {n for n in parts if sh.get(n) == sid}
        mesh = next(((g.gear_a, g.gear_b) if g.gear_a in on else (g.gear_b, g.gear_a)
                     for g in (design.gear_meshes or []) if g.gear_a in on or g.gear_b in on), None)
        best = (0.0, None)
        for sgn in (1.0, -1.0):
            f = np.zeros((len(st.names), 6))
            f[st.idx[m], 3:] += -sgn * ax
            if mesh and mesh[0] in st.idx and mesh[1] in st.idx:
                pin, gear = mesh
                r = max(cat[parts[pin].part].teeth, 8) / 2 * 1e-3
                cp, cg = np.asarray(frames[pin][0]), np.asarray(frames[gear][0])
                radial = (cg - cp) - np.dot(cg - cp, ax) * ax
                radial = ax.copy() if np.linalg.norm(radial) < 1e-6 else radial / np.linalg.norm(radial)
                tang = np.cross(ax, radial) * sgn
                F = -(1.0 / r) * tang - (1.0 / r) * math.tan(PRESSURE) * radial
                cpt = cp + radial * r
                f[st.idx[pin], :3] += F
                f[st.idx[pin], 3:] += np.cross(cpt - st.pos[st.idx[pin]], F)
                f[st.idx[gear], :3] -= F
                f[st.idx[gear], 3:] += np.cross(cpt - st.pos[st.idx[gear]], -F)
            # direct drive (no gear on the motor shaft): only the reaction on the motor body; the wheel puts the drive
            # torque into the ground, which the support (the chassis) takes
            q = np.zeros(6 * len(st.names))
            q[st.free] = st.lu.solve(f.reshape(-1)[st.free])
            dd = (st.D @ q).reshape(len(st.springs), 6)
            for si, (ci, a, b, p, E, kl, kr) in enumerate(st.springs):
                c = design.connections[ci]
                if c.kind != "stud" or m not in (a, b):
                    continue
                fz = st.kdiag[6 * si + 2] * dd[si, 2]
                up = 1.0 if np.dot(np.asarray(frames[b][0]) - np.asarray(frames[a][0]), E[:, 2]) > 0 else -1.0
                if fz * up > best[0]:
                    best = (float(fz * up), f"{c.child}->{c.parent}")
        if best[0] > 0:
            out[m] = {"c": best[0], "joint": best[1]}
    return out
