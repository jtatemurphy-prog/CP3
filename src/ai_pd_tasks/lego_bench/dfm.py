# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Manufacturability / assemblability review of a brick-vehicle design (design-quality constraints, not rules).

``review(design)`` inspects what a design leaves unconstrained, the way a design review or a DFM report would, and
says what each finding does to the real build. It complements ``design.validate`` (which checks that every declared
connection is geometrically possible): a design can be valid and still come apart on the bench.

Checks (thresholds are the module constants below):

1. **Axial retention** of everything that turns (``falls_out``, ``loses_mesh``, ``axial_play``, ``friction_held``).
   A rotating cluster (an axle with the gears / wheels / bushes cross-fitted on it, a free-turning gear, a pivot pin)
   turns in its holes by design; along its axis it is held only where one of its members bears on a face of the rest
   of the vehicle. For each cluster the free travel along the axis in each direction is measured (swept oriented
   boxes) and compared with what that travel does: the shaft leaves its holes / the motor socket (falls out), a gear
   or chain sprocket runs out of mesh (loses mesh), or just play.
   CRITICAL when nothing but zero-friction round holes hold it; a cluster gripped by friction somewhere (axle in a
   motor's cross socket, a ridged pin) only gets a WARNING (it moves if pushed, it does not drift by itself).
2. **Separable sub-assemblies** (``separable``, ``weak_stud_joint``, ``single_row_studs``, ``plate_seam``). For each
   direction the parts are grouped into blocks that cannot move relative to each other along it (studs interlock
   sideways, pins and axles interlock across their axis); a group of blocks that can be lifted off the rest along
   the direction (every joint across the cut pulls apart that way: stud joints all on the same face, pins / axles
   along it, and a swept-box test shows nothing else is in the way) is a separable sub-assembly. Its joint strength
   (pull-off and tipping about the joint edges, per stud / per pin) is compared with design loads: its weight
   (vehicle lifted by the chassis, safety factor ``SF_HANG``), a hard landing on one of its wheel axles with
   traction (``IMPACT_G``, ``TRACTION_MU``, with the vehicle's inertia relief), and a motor's stall-torque reaction.
3. **Pivoting members** (``pivoting``, ``linkage``). Bodies are parts joined rigidly (two or more studs, cross fits,
   two or more pins, a locked axle-pin); a single pin, an axle turning in a hole, a single stud or a motor output is a
   hinge. A group of bodies whose only attachment to the chassis lies on ONE hinge line swings about it: CRITICAL
   when it carries load (wheel, motor, battery / receiver, a gear meshing outside it, >= 10 % of the mass). Pivot
   pins (frictionless 3673 / 65249) and shocks are deliberate and skipped; an axle-only pivot (rocker) is a WARNING.
   Members that can still move after that (a pinned parallelogram racks) are a ``linkage`` WARNING.
4. **Assembly access** (``insertion``): a pin or axle both of whose ends are blocked along its axis by parts of the
   structure it sits in (within its own length) cannot be pushed in once that structure is built: WARNING.
5. **Metrics**: parts, part types, stud joints / studs, pins, fraction of part-to-part joints that are stud-only,
   longest span between ground supports, overhangs, mass, mechanism degrees of freedom.

Never raises: a check that hits unknown geometry is skipped with a note in ``metrics['notes']``.

    python -m ai_pd_tasks.lego_bench.dfm design.json [...]       review of each design
    python -m ai_pd_tasks.lego_bench.dfm --all <runs dir>        every */submission.json below it, tabulated
"""

from __future__ import annotations

import glob
import json
import math
import os
import sys
from collections import Counter, defaultdict

import numpy as np

from .design import (
    BEVEL_FAMILIES,
    BEVEL_RIGHT_ANGLE,
    Design,
    Placed,
    axis_vec,
    part_holes,
    pin_fit,
    pin_sections,
    pivot_connections,
    place,
)
from .parts import Box, Boxes, Cyl, Segs

# ----------------------------------------------------------------------------------------------------------------
# thresholds
# ----------------------------------------------------------------------------------------------------------------

G = 9.81
SHRINK = 0.2  # mm: collision boxes are shrunk by this, so parts that only touch do not count as overlapping
COAX_TOL = 0.6  # mm: a hole is on a shaft's line within this
ENGAGE_OUT_MM = 0.5  # a shaft with less than this inside a hole has left it
MAX_TRAVEL_MM = 60.0  # free travel is reported up to this
PLAY_OK_MM = 1.0  # axial play above this (in total, both directions) is a WARNING
MESH_MIN_OVERLAP_MM = 1.0  # two parallel gears with less face overlap than this are out of mesh
RIGHT_ANGLE_SLIP_MM = 1.5  # a bevel gear moved this far away from its partner's axis is out of mesh
CHAIN_SLIP_MM = 2.0  # a chain / tread sprocket this far off the plane of its partner throws the chain
BEVEL_IDS = frozenset(BEVEL_RIGHT_ANGLE) | frozenset().union(*BEVEL_FAMILIES)
"""Bevel teeth (double bevels 12 / 20 / 36T, single bevels) push the gears apart along their axles whenever they
carry torque, also when meshed as spur gears: a free cluster with such a gear walks out of mesh by itself (the E0
ramp / steps trials that lost their drive), so that is CRITICAL. Straight spur teeth and plain chains do not push;
a free spur cluster can still be walked out by side loads on the wheels: WARNING."""
MOTOR_SOCKET_MM = 8.0  # depth of a motor's output socket

STUD_PULL_N = 2.5  # pull-off force of one stud, plate / brick on plate / brick
STUD_PULL_HOUSING_N = 1.2  # one stud of a motor / battery / receiver / hub housing (shallow, smooth anti-studs)
HOUSING_KINDS = ("motor", "battery", "receiver", "hub", "sensor")
PIN_PULL_N = 5.0  # a ridged friction pin pulling out of one hole (laps.PIN_PULLOUT_LIMIT_N)
PIVOT_PIN_PULL_N = 1.0  # a frictionless pin pulling out of one hole
CROSS_PULL_N = 1.0  # an axle sliding in a cross hole / motor socket (friction of the cross fit)
PIN_TILT_NM = 0.2  # a pin / axle in an 8 mm hole resists tilting (it bears on the hole walls; ~50 N x 4 mm)
STUD_EDGE_MM = 4.0  # a stud joint tips about the edge of the contact face, half a stud pitch beyond the last stud
SEP_TRAVEL_MM = 3.0  # a sub-assembly that can move this far (studs are 1.7 mm tall) is separable

SF_HANG = 3.0  # safety factor on a sub-assembly's own weight (vehicle picked up by the chassis)
IMPACT_G = 3.0  # hard landing on one axle (off a 2-3 cm step / bump onto rubber tyres peaks near 3 g): that axle takes 3 x the vehicle weight
TRACTION_MU = 0.8  # with a horizontal (traction / braking) ground force of this times the vertical one
SF_DRIVE = 1.0  # safety factor on the landing and motor-torque loads (they are already extreme cases)
SIGNIFICANT_MASS_FRAC = 0.10  # a sub-assembly heavier than this fraction of the vehicle matters on its own
FEW_STUDS = 4  # a stud-only joint of this many studs or fewer between significant groups is a WARNING
SINGLE_ROW_MOMENT_FRAC = 0.25  # a single-row stud joint loaded to this fraction of its tipping strength: WARNING

STRUCTURAL_EXCLUDE = ("pin", "axle", "gear", "wheel", "bush")  # kinds that are not frame members
LOAD_KINDS = ("motor", "battery", "receiver", "hub")
INSERT_MARGIN_MM = 1.0


# ----------------------------------------------------------------------------------------------------------------
# geometry: oriented boxes, swept separating-axis test
# ----------------------------------------------------------------------------------------------------------------


def _part_obbs(pl: Placed) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Oriented boxes (centre, axes as columns, half extents) of a placed part, design frame, mm, shrunk SHRINK."""
    s = pl.bp.shape
    out = []
    boxes = ()
    if isinstance(s, Boxes) and s.boxes:
        boxes = s.boxes
    elif isinstance(s, Segs) and s.segs:
        for x0, z0, x1, z1 in s.segs:
            a, b = np.array([x0, 0, z0], float), np.array([x1, 0, z1], float)
            d = b - a
            L = float(np.linalg.norm(d)) or 1e-6
            u = d / L
            w = np.cross(u, [0, 1.0, 0])
            out.append(((a + b) / 2, np.column_stack([u, [0, 1.0, 0], w]), np.array([L / 2 + s.radius, s.half_w, s.radius])))
    elif isinstance(s, Box):
        boxes = ((s.lo, s.hi),)
    for lo, hi in boxes:
        lo, hi = np.array(lo, float), np.array(hi, float)
        out.append(((lo + hi) / 2, np.eye(3), (hi - lo) / 2))
    if isinstance(s, Cyl):
        h = np.full(3, s.radius)
        h["xyz".index(s.axis)] = s.half_len
        out.append((np.array(s.centre, float), np.eye(3), h))
    return [(pl.p + pl.R @ c, pl.R @ A, np.maximum(h - SHRINK, 0.05)) for c, A, h in out]


class _Geo:
    """All parts' boxes as arrays, the rest-overlap matrix and swept contact queries."""

    def __init__(self, P: dict[str, Placed]):
        self.names = list(P)
        self.idx = {n: i for i, n in enumerate(self.names)}
        C, A, H, own = [], [], [], []
        for i, n in enumerate(self.names):
            try:
                bs = _part_obbs(P[n])
            except Exception:  # noqa: BLE001
                bs = []
            for c, a, h in bs:
                C.append(c), A.append(a), H.append(h), own.append(i)
        self.C = np.array(C, float).reshape(-1, 3)
        self.A = np.array(A, float).reshape(-1, 3, 3)
        self.H = np.array(H, float).reshape(-1, 3)
        self.own = np.array(own, int)
        self.rad = np.linalg.norm(self.H, axis=1)
        n = len(self.names)
        self.boxes_of = [np.nonzero(self.own == i)[0] for i in range(n)]
        # rest overlap, part level
        self.ov = np.zeros((n, n), bool)
        nb = len(self.C)
        if nb:
            ia, ib = np.triu_indices(nb, 1)
            keep = (self.own[ia] != self.own[ib]) & (
                np.linalg.norm(self.C[ia] - self.C[ib], axis=1) <= self.rad[ia] + self.rad[ib])
            ia, ib = ia[keep], ib[keep]
            if len(ia):
                lo, hi = self._intervals(ia, ib, np.zeros(3))
                hit = (lo <= 0.0) & (hi >= 0.0) & (lo <= hi)
                self.ov[self.own[ia[hit]], self.own[ib[hit]]] = True
                self.ov[self.own[ib[hit]], self.own[ia[hit]]] = True

    def _intervals(self, ia, ib, u):
        """t-interval during which box ia moved by t u overlaps box ib (separating-axis test, 15 axes)."""
        ca, Aa, ha = self.C[ia], self.A[ia], self.H[ia]
        cb, Ab, hb = self.C[ib], self.A[ib], self.H[ib]
        axes = [Aa[:, :, i] for i in range(3)] + [Ab[:, :, j] for j in range(3)]
        for i in range(3):
            for j in range(3):
                v = np.cross(Aa[:, :, i], Ab[:, :, j])
                nv = np.linalg.norm(v, axis=1, keepdims=True)
                axes.append(np.where(nv > 1e-6, v / np.maximum(nv, 1e-12), 0.0))
        L = np.stack(axes, axis=1)  # (K, 15, 3)
        ra = (np.abs(np.einsum("kdi,kld->kli", Aa, L)) * ha[:, None, :]).sum(-1)
        rb = (np.abs(np.einsum("kdi,kld->kli", Ab, L)) * hb[:, None, :]).sum(-1)
        r = ra + rb + 1e-9
        d = np.einsum("kd,kld->kl", cb - ca, L)
        du = L @ np.asarray(u, float)
        small = np.abs(du) < 1e-9
        dus = np.where(small, 1.0, du)
        t1, t2 = (d - r) / dus, (d + r) / dus
        lo = np.where(small, np.where(np.abs(d) <= r, -np.inf, np.inf), np.minimum(t1, t2))
        hi = np.where(small, np.where(np.abs(d) <= r, np.inf, -np.inf), np.maximum(t1, t2))
        return lo.max(1), hi.min(1)

    def first_contact(self, moving: list[str], others: list[str], u, tmax: float) -> tuple[float, tuple | None]:
        """Travel of the ``moving`` parts along unit u until a box of theirs first touches one of ``others`` (pairs
        that overlap where they sit excluded); (tmax, None) if nothing within tmax."""
        mi = [self.idx[n] for n in moving if n in self.idx]
        oi = [self.idx[n] for n in others if n in self.idx]
        if not mi or not oi:
            return tmax, None
        mb = np.concatenate([self.boxes_of[i] for i in mi]) if mi else np.zeros(0, int)
        ob = np.concatenate([self.boxes_of[i] for i in oi]) if oi else np.zeros(0, int)
        if not len(mb) or not len(ob):
            return tmax, None
        ga, gb = np.meshgrid(mb, ob, indexing="ij")
        ba, bb = ga.ravel(), gb.ravel()
        keep = ~self.ov[self.own[ba], self.own[bb]]
        ba, bb = ba[keep], bb[keep]
        if not len(ba):
            return tmax, None
        # broad phase: the box must come within reach along the sweep
        rel = self.C[bb] - self.C[ba]
        along = rel @ np.asarray(u, float)
        perp = np.linalg.norm(rel - np.outer(along, u), axis=1)
        reach = self.rad[ba] + self.rad[bb]
        keep = (perp <= reach) & (along >= -reach) & (along <= tmax + reach)
        ba, bb = ba[keep], bb[keep]
        if not len(ba):
            return tmax, None
        lo, hi = self._intervals(ba, bb, u)
        valid = (lo <= hi) & (hi > 0) & (lo < tmax)
        if not valid.any():
            return tmax, None
        t = np.where(valid, np.maximum(lo, 0.0), np.inf)
        k = int(np.argmin(t))
        return float(t[k]), (self.names[self.own[ba[k]]], self.names[self.own[bb[k]]])


# ----------------------------------------------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------------------------------------------


class _UF:
    def __init__(self, items):
        self.p = {x: x for x in items}

    def find(self, a):
        p = self.p
        while p[a] != a:
            p[a] = p[p[a]]
            a = p[a]
        return a

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[rb] = ra

    def groups(self):
        out = defaultdict(list)
        for x in self.p:
            out[self.find(x)].append(x)
        return out


def _finding(check: str, parts, message: str, value=None, limit=None) -> dict:
    def r(x):
        return round(float(x), 2) if isinstance(x, (int, float, np.floating)) and not isinstance(x, bool) else x
    return {"check": check, "parts": sorted(set(parts)), "message": message, "value": r(value), "limit": r(limit)}


def _com(pl: Placed) -> np.ndarray:
    try:
        return pl.to_world(pl.bp.com_mm)
    except Exception:  # noqa: BLE001
        return pl.p


def _structural(pl: Placed) -> bool:
    return pl.bp.kind not in STRUCTURAL_EXCLUDE


def _perp_basis(v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    a = np.array([1.0, 0, 0]) if abs(v[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = np.cross(v, a)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(v, e1)


def _show(P, names, k: int = 5) -> str:
    """The heaviest frame parts of a group first (pins, axles, gears follow), at most k named."""
    ns = sorted(names, key=lambda n: (not _structural(P[n]), -float(P[n].bp.mass_g), n))
    return ", ".join(ns[:k]) + (f" and {len(ns) - k} more parts" if len(ns) > k else "")


def _fmt_dir(v) -> str:
    v = np.asarray(v, float)
    k = int(np.argmax(np.abs(v)))
    if abs(v[k]) > 0.999:
        return ("+" if v[k] > 0 else "-") + "xyz"[k]
    return "(" + ", ".join(f"{x:.2f}" for x in v) + ")"


# ----------------------------------------------------------------------------------------------------------------
# joints
# ----------------------------------------------------------------------------------------------------------------

STUD_Z_TOL = 0.3
STUD_XY_TOL = 0.5


def _stud_points(upper: Placed, lower: Placed) -> tuple[list[np.ndarray], float]:
    """World positions of the studs of ``lower`` engaged in ``upper``'s underside, and the face gap."""
    if not upper.bp.studs_bottom or not lower.bp.studs_top:
        return [], float("inf")
    if float(np.dot(upper.R[:, 2], lower.R[:, 2])) < 0.999:
        return [], float("inf")
    a = np.array([lower.to_local(upper.to_world((x, y, upper.bp.bottom_z))) for x, y in upper.bp.studs_bottom])
    b = np.array([(x, y, lower.bp.top_z) for x, y in lower.bp.studs_top])
    gap = abs(float(np.mean(a[:, 2])) - lower.bp.top_z)
    dist = np.linalg.norm(b[:, None, :2] - a[None, :, :2], axis=2).min(axis=1)
    return [lower.to_world(p) for p in b[dist < STUD_XY_TOL]], gap


def _joints(design: Design, P: dict[str, Placed], notes: list[str]) -> list[dict]:
    """Every declared connection as a joint: kind, the two parts, its line (pins / axles), stud points and normal,
    pull-off per element, whether it allows rotation (hinge) and whether it is a deliberate pivot."""
    try:
        piv = pivot_connections(design)
    except Exception:  # noqa: BLE001
        piv = set()
        notes.append("pivot pins could not be resolved")
    out = []
    for i, c in enumerate(design.connections):
        if c.child not in P or c.parent not in P or c.child == c.parent:
            continue
        ch, pa = P[c.child], P[c.parent]
        j = dict(i=i, kind=c.kind, a=c.child, b=c.parent, line=None, rot=False, f=0.0, intended=i in piv,
                 pts=[], n=None, upper=None)
        try:
            if c.kind == "stud":
                p1, g1 = _stud_points(ch, pa)
                p2, g2 = _stud_points(pa, ch)
                if g1 <= g2:
                    pts, gap, up, lo = p1, g1, ch, pa
                else:
                    pts, gap, up, lo = p2, g2, pa, ch
                if not pts or gap > 1.0:
                    notes.append(f"stud joint {c.child}->{c.parent}: no engaged studs found (skipped)")
                    continue
                j.update(pts=pts, n=lo.R[:, 2].copy(), upper=up.inst.name,
                         f=STUD_PULL_HOUSING_N if (ch.bp.kind in HOUSING_KINDS or pa.bp.kind in HOUSING_KINDS)
                         else STUD_PULL_N)
                if len(pts) == 1:
                    j.update(rot=True, line=(pts[0], lo.R[:, 2].copy()))
            elif c.kind == "cross":
                ax = pa.axis() if pa.bp.kind in ("axle", "pin") else ch.axis()
                j.update(line=(pa.p.copy() if pa.bp.kind in ("axle", "pin") else ch.p.copy(), ax), f=CROSS_PULL_N)
            elif c.kind == "hinge":
                j.update(line=(ch.p.copy(), ch.axis()), rot=True, f=0.0)
            elif c.kind == "motor":
                j.update(line=(pa.p.copy(), pa.R[:, 1].copy()), rot=True, f=CROSS_PULL_N, motor=c.parent)
            elif c.kind == "pin":
                fits = pin_fit(ch, pa) if ch.bp.kind == "pin" or ch.bp.male_pins or ch.bp.male_axles else []
                locked = any(isinstance(sec, tuple) and sec[-1] == "axle" and hk == "axle" for sec, _, hk in fits)
                if ch.bp.kind == "pin":
                    line = (ch.p.copy(), ch.axis())
                    f = PIVOT_PIN_PULL_N if ch.bp.pivot else PIN_PULL_N
                    if fits and all(sec[-1] == "axle" for sec, _, _ in fits):
                        f = CROSS_PULL_N
                elif fits and len(fits[0][0]) == 4:  # a connector's integral pin (x, y, z, axis)
                    x, y, z, a = fits[0][0]
                    line = (ch.to_world((x, y, z)), ch.R @ axis_vec(a))
                    f = PIN_PULL_N
                    lines = {tuple(np.round(ch.to_world(s[:3]), 1)) for s, _, _ in fits if len(s) == 4}
                    if len(lines) > 1:
                        locked = True  # two integral pins in the same part: rigid
                else:
                    line, f = (pa.p.copy(), pa.R[:, 1].copy()), PIN_PULL_N
                    if not fits:
                        notes.append(f"pin joint {c.child}->{c.parent}: no pin section found in a hole (line guessed)")
                j.update(line=line, rot=not locked, f=f)
                if ch.bp.pivot or pa.bp.kind == "shock" or ch.bp.kind == "shock":
                    j["intended"] = True
            else:
                notes.append(f"connection kind '{c.kind}' {c.child}->{c.parent} not reviewed")
                continue
        except Exception as e:  # noqa: BLE001
            notes.append(f"connection {c.child}->{c.parent}: {e!r} (skipped)")
            continue
        out.append(j)
    return out


def _line_key(lines: list, p, a) -> int:
    """Index of the line (point, unit axis) in ``lines`` that (p, a) lies on; appended if new."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    for k, (q, b) in enumerate(lines):
        if abs(float(np.dot(a, b))) > 0.999:
            d = np.asarray(p, float) - q
            if np.linalg.norm(d - np.dot(d, b) * b) < COAX_TOL:
                return k
    lines.append((np.asarray(p, float), a))
    return len(lines) - 1


# ----------------------------------------------------------------------------------------------------------------
# check 1: axial retention
# ----------------------------------------------------------------------------------------------------------------


def _hole_intervals(o: Placed, p0: np.ndarray, ax: np.ndarray) -> list[tuple[float, float]]:
    """Axial intervals (coordinate s = x . ax) of the holes of ``o`` on the line (p0, ax)."""
    out = []
    for c, a, hh, _k in part_holes(o.bp):
        aw = o.R @ a
        if abs(float(np.dot(aw, ax))) < 0.999:
            continue
        cw = o.to_world(c)
        d = cw - p0
        if np.linalg.norm(d - np.dot(d, ax) * ax) > COAX_TOL:
            continue
        s = float(np.dot(cw, ax))
        out.append((s - hh, s + hh))
    if not out and o.bp.free_bore:
        s = float(np.dot(o.p, ax))
        k = float(np.dot(o.axis(), ax))
        h0, h1 = sorted((s + k * o.bp.hub[0], s + k * o.bp.hub[1]))
        out.append((h0, h1))
    return out


def _overlap(iv, ivs, t):
    return sum(max(0.0, min(iv[1] + t, h1) - max(iv[0] + t, h0)) for h0, h1 in ivs)


def _t_out(iv, ivs, sgn: float, tmax: float) -> float | None:
    """Travel (in direction sgn) at which the member interval ``iv`` has left the hole intervals; None if not within
    tmax or if it is not engaged now."""
    if _overlap(iv, ivs, 0.0) < ENGAGE_OUT_MM:
        return None
    t = 0.0
    while t <= tmax + 1e-9:
        if _overlap(iv, ivs, sgn * t) < ENGAGE_OUT_MM:
            return t
        t += 0.1
    return None


def _shaft_interval(pl: Placed, ax: np.ndarray) -> list[tuple[float, float]]:
    s = float(np.dot(pl.p, ax))
    k = float(np.dot(pl.axis(), ax))
    if pl.bp.kind == "axle":
        h = pl.bp.axle_half_len
        return [(s - h, s + h)]
    if pl.bp.kind == "pin":
        return [tuple(sorted((s + k * (cx - h), s + k * (cx + h)))) for cx, h, _ in pin_sections(pl)]
    if pl.bp.free_bore or pl.bp.kind in ("gear", "wheel", "bush"):
        return [tuple(sorted((s + k * pl.bp.hub[0], s + k * pl.bp.hub[1])))]
    return []


def _mesh_pairs(design: Design) -> list[tuple[str, str, str]]:
    out = [(m.gear_a, m.gear_b, "mesh") for m in design.gear_meshes]
    for ch in getattr(design, "chains", []) or []:
        out.append((ch.gears[0], ch.gears[1], "chain"))
    for t in getattr(design, "treads", []) or []:
        out.append((t.sprockets[0], t.sprockets[1], "tread"))
    return out


def _check_axial(design, P, geo: _Geo, joints, crit, warn, notes) -> list[dict]:
    names = list(P)
    uf = _UF(names)
    for j in joints:
        if j["kind"] == "cross":
            uf.union(j["a"], j["b"])
    clusters = uf.groups()
    meshes = _mesh_pairs(design)
    report = []
    for members in clusters.values():
        mset = set(members)
        shafts_ = [m for m in members if P[m].bp.kind in ("axle", "pin")]
        free_bore = [m for m in members if P[m].bp.free_bore]
        if any(_structural(P[m]) for m in members) and not any(
                j["kind"] in ("hinge", "motor") and (j["a"] in mset) != (j["b"] in mset) for j in joints):
            continue  # static structure (an axle locked into the frame)
        if not shafts_ and not free_bore:
            continue
        sh = shafts_[0] if shafts_ else free_bore[0]
        ax = P[sh].axis()
        p0 = P[sh].p
        # engagements: (member, other, kind, member interval(s), hole intervals, friction)
        eng = []
        for j in joints:
            if j["kind"] not in ("hinge", "motor", "pin"):
                continue
            if (j["a"] in mset) == (j["b"] in mset):
                continue
            m, o = (j["a"], j["b"]) if j["a"] in mset else (j["b"], j["a"])
            if j["line"] is None or abs(float(np.dot(j["line"][1], ax))) < 0.999:
                continue
            try:
                if j["kind"] == "motor":
                    mo = P[o]
                    d = mo.R[:, 1]
                    s0 = float(np.dot(mo.p, ax))
                    sd = 1.0 if float(np.dot(d, ax)) > 0 else -1.0
                    if mo.bp.through_bore:
                        hiv = [(s0 - mo.bp.through_bore, s0 + mo.bp.through_bore)]
                        bottom = None
                    else:
                        hiv = [tuple(sorted((s0, s0 + sd * MOTOR_SOCKET_MM)))]
                        bottom = (s0 + sd * MOTOR_SOCKET_MM, sd)
                    eng.append(dict(m=m, o=o, kind="motor", miv=_shaft_interval(P[m], ax), hiv=hiv, friction=True,
                                    bottom=bottom))
                elif j["kind"] == "hinge":
                    if P[m].bp.kind == "axle":
                        hiv = _hole_intervals(P[o], p0, ax)
                        miv = _shaft_interval(P[m], ax)
                    else:  # the free-turning gear is ours, the axle the other part
                        hiv = _shaft_interval(P[o], ax)
                        miv = _shaft_interval(P[m], ax)
                    if not hiv or not miv:
                        notes.append(f"{m} in {o}: hole geometry not found (axial retention skipped for it)")
                        continue
                    eng.append(dict(m=m, o=o, kind="hinge", miv=miv, hiv=hiv, friction=False, bottom=None))
                else:  # pin
                    if P[m].bp.kind != "pin":
                        continue
                    hiv = _hole_intervals(P[o], p0, ax)
                    if not hiv:
                        continue
                    eng.append(dict(m=m, o=o, kind="pin", miv=_shaft_interval(P[m], ax), hiv=hiv,
                                    friction=not P[m].bp.pivot, bottom=None))
            except Exception as e:  # noqa: BLE001
                notes.append(f"axial retention {m} in {o}: {e!r}")
        if not eng:
            continue
        is_pin = P[sh].bp.kind == "pin"
        if is_pin and all(e["friction"] for e in eng):
            continue  # ridged friction pins are held by design
        others = [n for n in names if n not in mset]
        free, stop = {}, {}
        for sgn in (1.0, -1.0):
            t, hit = geo.first_contact(members, others, sgn * ax, MAX_TRAVEL_MM)
            t = max(t - 2 * SHRINK, 0.0) if hit else MAX_TRAVEL_MM
            for e in eng:  # a motor socket is blind: the shaft cannot go deeper than its bottom
                if e["bottom"] is not None and e["bottom"][1] == sgn:
                    end = max(sgn * x for iv in e["miv"] for x in iv)
                    room = max(sgn * e["bottom"][0] - end, 0.0)
                    if room < t:
                        t, hit = room, (e["m"], f"{e['o']} (socket bottom)")
            free[sgn], stop[sgn] = round(t, 2), hit
        friction = any(e["friction"] for e in eng)
        issues = []
        for sgn in (1.0, -1.0):
            T = free[sgn]
            dname = ("+" if sgn > 0 else "-") + "axis " + _fmt_dir(sgn * ax)
            stop_txt = (f"until {stop[sgn][0]} touches {stop[sgn][1]}" if stop[sgn] else "with nothing in the way")
            # leaving holes
            lost = []
            for e in eng:
                outs = [_t_out(iv, e["hiv"], sgn, T) for iv in e["miv"]]
                outs = [x for x in outs if x is not None]
                engaged_now = [iv for iv in e["miv"] if _overlap(iv, e["hiv"], 0.0) >= ENGAGE_OUT_MM]
                if engaged_now and len(outs) == len(engaged_now):
                    lost.append((max(outs), e))
            bearing_parts = {e["o"] for e in eng}
            lost_parts = {e["o"] for _, e in lost}
            for t_o, e in lost:
                whole = lost_parts >= bearing_parts or e["kind"] == "motor"
                if is_pin:
                    msg = (f"pivot pin {sh} can slide {T:.1f} mm ({dname}) {stop_txt} and leaves its hole in {e['o']} "
                           f"after {t_o:.1f} mm: a frictionless pin has no ridges to hold it; the joint comes apart")
                    issues.append(("WARNING", _finding("falls_out", [sh, e["o"]], msg, t_o, T)))
                    continue
                if whole:
                    what = (f"comes out of the motor {e['o']}'s output (the drive is lost)" if e["kind"] == "motor"
                            else f"leaves its last hole in {e['o']} (it falls out)")
                    sev = "WARNING" if friction else "CRITICAL"
                    chk = "friction_held" if friction else "falls_out"
                    held = (" Only the friction of the cross fit holds it there." if friction else
                            " Nothing holds it but the parts it would have to touch.")
                    msg = (f"{sh} (with {', '.join(m for m in members if m != sh) or 'nothing on it'}) can slide "
                           f"{T:.1f} mm ({dname}) {stop_txt}; after {t_o:.1f} mm it {what}.{held} Add a bush or "
                           f"stop so it cannot travel that far.")
                    issues.append((sev, _finding(chk, [sh, e["o"]], msg, T, t_o)))
                else:
                    msg = (f"{sh} can slide {T:.1f} mm ({dname}) {stop_txt} and then leaves its hole in {e['o']} "
                           f"(after {t_o:.1f} mm): it runs in fewer bearings and wobbles")
                    issues.append(("WARNING", _finding("falls_out", [sh, e["o"]], msg, T, t_o)))
            # meshes
            for ga, gb, mk in meshes:
                if (ga in mset) == (gb in mset) or ga not in P or gb not in P:
                    continue
                g, h = (ga, gb) if ga in mset else (gb, ga)
                pg, ph = P[g], P[h]
                ah = ph.axis()
                sg, sh_ = float(np.dot(pg.p, ax)), float(np.dot(ph.p, ax))
                wg, wh = (pg.bp.width or 8.0), (ph.bp.width or 8.0)
                t_lost = None
                if abs(float(np.dot(ah, ax))) > 0.99:
                    if mk == "mesh":
                        g0, g1, h0, h1 = sg - wg / 2, sg + wg / 2, sh_ - wh / 2, sh_ + wh / 2
                        if min(g1, h1) - max(g0, h0) < MESH_MIN_OVERLAP_MM:
                            continue
                        t_lost = (h1 - g0 - MESH_MIN_OVERLAP_MM) if sgn > 0 else (g1 - h0 - MESH_MIN_OVERLAP_MM)
                    else:
                        off0 = sg - sh_
                        if abs(off0) > CHAIN_SLIP_MM:
                            continue
                        t_lost = CHAIN_SLIP_MM - sgn * off0
                elif abs(float(np.dot(ah, ax))) < 0.1 and mk == "mesh":
                    if pg.bp.teeth == 1:
                        continue  # a worm slides along its own threads
                    if ph.bp.teeth == 1:
                        t_lost = wg / 2
                    else:
                        d = float(np.dot(ph.p - pg.p, ax))
                        if sgn * d >= 0:
                            continue  # towards the partner: the teeth jam, they do not separate
                        t_lost = RIGHT_ANGLE_SLIP_MM
                if t_lost is None or t_lost > T:
                    continue
                t_lost = max(t_lost, 0.0)
                what = {"mesh": f"out of mesh with {h}", "chain": f"off the line of the chain to {h} (it throws the "
                        "chain)", "tread": f"off the line of the tread to {h} (it throws the tread)"}[mk]
                thrust = (mk != "mesh" or pg.bp.part_id in BEVEL_IDS or ph.bp.part_id in BEVEL_IDS)
                sev = "CRITICAL" if (thrust and not friction) else "WARNING"
                chk = "friction_held" if friction else "loses_mesh"
                if friction:
                    held = " Only the friction of the cross fit holds it."
                elif mk != "mesh":
                    held = " Nothing holds it there, and a chain throws at a small offset."
                elif thrust:
                    held = (" Nothing holds it there, and bevelled teeth push the gears apart along the axle "
                            "whenever they carry torque, so it walks out by itself.")
                else:
                    held = (" Nothing holds it there; straight teeth do not push it, but side loads on the wheels can "
                            "walk it along the axle.")
                msg = (f"{g} (on {sh}) can slide {T:.1f} mm ({dname}) {stop_txt}; after {t_lost:.1f} mm it is {what} "
                       f"and the drive stops.{held} Close the gap with a bush or move a part so it stops the cluster.")
                issues.append((sev, _finding(chk, [g, h, sh], msg, T, t_lost)))
        play = free[1.0] + free[-1.0]
        if not issues and not friction and not is_pin and play > PLAY_OK_MM:
            msg = (f"{sh} cluster has {min(play, 2 * MAX_TRAVEL_MM):.1f} mm of axial play (free +{free[1.0]:.1f} / "
                   f"-{free[-1.0]:.1f} mm): gears and wheels on it wander and rub")
            issues.append(("WARNING", _finding("axial_play", [sh], msg, play, PLAY_OK_MM)))
        for sev, f in issues:
            (crit if sev == "CRITICAL" else warn).append(f)
        report.append(dict(shaft=sh, members=sorted(members), axis=[round(float(x), 3) for x in ax],
                           bearings=sorted({e["o"] for e in eng}), friction=friction,
                           free_plus_mm=free[1.0], free_minus_mm=free[-1.0],
                           stop_plus=list(stop[1.0]) if stop[1.0] else None,
                           stop_minus=list(stop[-1.0]) if stop[-1.0] else None,
                           findings=[f["check"] for _, f in issues]))
    return report


# ----------------------------------------------------------------------------------------------------------------
# context: masses, wheels, chassis, motors
# ----------------------------------------------------------------------------------------------------------------


class _Ctx:
    def __init__(self, design: Design, P: dict[str, Placed], joints: list[dict]):
        self.design, self.P, self.joints = design, P, joints
        self.notes: list[str] = []
        self.mass = {n: max(float(pl.bp.mass_g), 0.0) / 1000.0 for n, pl in P.items()}  # kg
        self.com = {n: _com(pl) for n, pl in P.items()}
        self.M = sum(self.mass.values()) or 1e-3
        self.r_com = sum(self.mass[n] * self.com[n] for n in P) / self.M
        # inertia tensor of point masses about the COM (+ a small isotropic term per part)
        I = np.zeros((3, 3))
        for n in P:
            r = self.com[n] - self.r_com
            I += self.mass[n] * ((r @ r) * np.eye(3) - np.outer(r, r)) * 1e-6
            I += self.mass[n] * 1e-4 * np.eye(3) * 1e-2
        self.I_inv = np.linalg.inv(I + 1e-9 * np.eye(3))
        # chassis
        self.chassis = design.chassis if design.chassis in P else max(P, key=lambda n: self.mass[n])
        # connectivity over every joint
        adj = defaultdict(set)
        for j in joints:
            adj[j["a"]].add(j["b"])
            adj[j["b"]].add(j["a"])
        self.adj = adj
        seen, stack = {self.chassis}, [self.chassis]
        while stack:
            x = stack.pop()
            for y in adj[x]:
                if y not in seen:
                    seen.add(y)
                    stack.append(y)
        self.connected = seen
        # cross clusters (shafts) and wheels
        uf = _UF(list(P))
        for j in joints:
            if j["kind"] == "cross":
                uf.union(j["a"], j["b"])
        self.cluster_of = {n: uf.find(n) for n in P}
        self.clusters = uf.groups()
        sprockets = {s for t in (getattr(design, "treads", []) or []) for s in t.sprockets}
        self.wheels = [n for n, pl in P.items() if pl.bp.kind == "wheel" or n in sprockets]
        # ground contact of each wheel (lowest point of its rim)
        self.contact = {}
        for w in self.wheels:
            pl = self.P[w]
            r = pl.bp.wheel_radius or (pl.bp.shape.radius if isinstance(pl.bp.shape, Cyl) else 0.0)
            self.contact[w] = pl.p - np.array([0, 0, r])
        # wheel axles: cross clusters with wheels
        self.axles = defaultdict(list)
        for w in self.wheels:
            self.axles[self.cluster_of[w]].append(w)
        self.motors = [n for n, pl in P.items() if pl.bp.kind == "motor"]

    def drive_torque(self, m: str) -> tuple[float, str]:
        """Largest reaction torque the motor's housing takes [N m]: its stall torque, or less when it drives wheels,
        whose grip (TRACTION_MU x their share of the vehicle weight at the tyre radius, through the gear ratio)
        limits it."""
        st = self.stall(m)
        try:
            from .design import _propagate, shafts  # noqa: PLC0415
            outs = [j["a"] for j in self.joints if j["kind"] == "motor" and j["b"] == m]
            if not outs:
                return st, "stall"
            sh = shafts(self.design)
            spin, _ = _propagate(self.design, self.P, sh, outs[0], self.P[outs[0]].axis())
            # its wheels carry their share of the weight (wheel axles it drives / all wheel axles)
            driven = {self.cluster_of[w] for w in self.wheels if sh.get(w) in spin}
            share = len(driven) / max(len(self.axles), 1)
            best = 0.0
            for w in self.wheels:
                if sh.get(w) in spin:
                    r = (self.P[w].bp.wheel_radius or 15.0) * 1e-3
                    best = max(best, TRACTION_MU * share * self.M * G * r * float(np.linalg.norm(spin[sh[w]])))
            if 0.0 < best < st:
                return best, "wheel grip"
        except Exception:  # noqa: BLE001
            pass
        return st, "stall"

    def stall(self, m: str) -> float:
        bp = self.P[m].bp
        if bp.motor_stall_torque_nm:
            return float(bp.motor_stall_torque_nm)
        try:
            from .design import motor_params  # noqa: PLC0415
            return float(motor_params(bp.part_id)["stall_torque_nm"])
        except Exception:  # noqa: BLE001
            return 0.15

    def significant(self, B: set[str]) -> bool:
        mB = sum(self.mass[n] for n in B)
        return (mB >= SIGNIFICANT_MASS_FRAC * self.M or any(n in B for n in self.wheels)
                or any(self.P[n].bp.kind in ("motor",) for n in B))

    def landing_loads(self, B: set[str], axle_root: str, sx: float) -> tuple[list, list]:
        """Forces on B when the vehicle lands on one wheel axle: IMPACT_G x weight up at its wheels' contacts with a
        traction / braking force (sx = +-1 along x), plus the inertia of B's parts (linear and pitch / roll)."""
        ws = self.axles[axle_root]
        Rtot = IMPACT_G * self.M * G * np.array([sx * TRACTION_MU, 0.0, 1.0])
        ext = [(self.contact[w], Rtot / len(ws)) for w in ws]
        a_lin = Rtot / self.M  # gravity cancels: every part's weight is part of its inertia relief
        tau = sum((np.cross((q - self.r_com) * 1e-3, F) for q, F in ext), np.zeros(3))
        alpha = self.I_inv @ tau
        forces = [(q, F) for (q, F), w in zip(ext, ws) if w in B]
        Bl = [n for n in B]
        m = np.array([self.mass[n] for n in Bl])
        c = np.array([self.com[n] for n in Bl]).reshape(-1, 3)
        a_i = a_lin[None, :] + np.cross(alpha[None, :], (c - self.r_com) * 1e-3)
        forces += list(zip(c, -m[:, None] * a_i))
        return forces, []


# ----------------------------------------------------------------------------------------------------------------
# check 2: separable sub-assemblies
# ----------------------------------------------------------------------------------------------------------------


def _joint_capacity(elems, v, wrench, n_shafts: int = 0) -> tuple[float, str]:
    """Worst ratio load / strength of a joint (elements: (point, pull N)) separating along v under a wrench (forces
    [(point mm, F N)], moments [T N m]): pure pull-off and tipping about the edge of the joint in 8 directions (pins
    and axles across the joint, ``n_shafts``, must tilt in their holes to let it tip: PIN_TILT_NM each)."""
    forces, moments = wrench
    Q = np.array([q for q, _ in forces], float).reshape(-1, 3) * 1e-3
    Fm = np.array([f for _, f in forces], float).reshape(-1, 3)
    F = Fm.sum(0)
    T0 = np.cross(Q, Fm).sum(0) + (np.sum(moments, axis=0) if moments else 0.0)
    cap = sum(f for _, f in elems)
    worst, mode = 0.0, "pull-off"
    Fv = float(np.dot(F, v))
    if Fv > 1e-9:
        r = Fv / cap if cap > 1e-9 else math.inf
        if r > worst:
            worst, mode = r, "pull-off"
    pts = np.array([p for p, _ in elems], float)
    fs = np.array([f for _, f in elems], float)
    zc = float(np.mean(pts @ v))
    e1, e2 = _perp_basis(v)
    for k in range(8):
        th = k * math.pi / 4
        e = math.cos(th) * e1 + math.sin(th) * e2
        s = pts @ e
        s_edge = float(s.max()) + STUD_EDGE_MM
        c = (s_edge * e + zc * v) * 1e-3
        w = np.cross(v, e)
        M = float(np.dot(T0 - np.cross(c, F), w))
        if M <= 1e-9:
            continue
        Mres = float(np.sum(fs * (s_edge - s))) * 1e-3 + n_shafts * PIN_TILT_NM
        r = M / Mres if Mres > 1e-12 else math.inf
        if r > worst:
            worst, mode = r, f"tipping about its edge towards {_fmt_dir(e)}"
    return worst, mode


def _collinear(pts, tol=0.5) -> bool:
    pts = np.array(pts, float)
    if len(pts) < 2:
        return True
    d = pts - pts[0]
    far = d[np.argmax(np.linalg.norm(d, axis=1))]
    L = np.linalg.norm(far)
    if L < 1e-6:
        return True
    u = far / L
    return bool(np.all(np.linalg.norm(d - np.outer(d @ u, u), axis=1) < tol))


def _directions(joints) -> list[np.ndarray]:
    dirs = [np.array(v, float) for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    for j in joints:
        for v in ([j["n"]] if j["n"] is not None else []) + ([j["line"][1]] if j["line"] is not None else []):
            v = np.asarray(v, float) / np.linalg.norm(v)
            if not any(abs(float(np.dot(v, d))) > 0.999 for d in dirs):
                dirs.append(v)
    return [s * d for d in dirs for s in (1.0, -1.0)]


def _check_separable(ctx: _Ctx, geo: _Geo, crit, warn, notes) -> int:
    P, joints = ctx.P, ctx.joints
    parts = [n for n in P if n in ctx.connected]
    pset = set(parts)
    js = [j for j in joints if j["a"] in pset and j["b"] in pset]
    seen = set()
    n_cand = 0
    results = []
    for v in _directions(js):
        uf = _UF(parts)
        push = defaultdict(set)  # part -> parts it pushes when moved +v
        for j in js:
            if j["kind"] == "stud":
                if abs(float(np.dot(j["n"], v))) < 0.999:
                    uf.union(j["a"], j["b"])
                else:
                    lower = j["b"] if j["upper"] == j["a"] else j["a"]
                    if float(np.dot(j["n"], v)) > 0:
                        push[lower].add(j["upper"])
                    else:
                        push[j["upper"]].add(lower)
            else:
                if j["line"] is None or abs(float(np.dot(j["line"][1], v))) < 0.999:
                    uf.union(j["a"], j["b"])
        groups = uf.groups()
        root = {n: uf.find(n) for n in parts}
        bpush = defaultdict(set)
        for a, bs in push.items():
            for b in bs:
                if root[a] != root[b]:
                    bpush[root[a]].add(root[b])
        for X in groups:
            S, stack = {X}, [X]
            while stack:
                y = stack.pop()
                for z in bpush[y]:
                    if z not in S:
                        S.add(z)
                        stack.append(z)
            Sparts = {n for r in S for n in groups[r]}
            if ctx.chassis in Sparts:
                B, vB = pset - Sparts, -v
            else:
                B, vB = Sparts, v
            if not B or len(B) == len(pset):
                continue
            key = (frozenset(B), tuple(np.round(vB, 3)))
            if key in seen:
                continue
            seen.add(key)
            if not any(_structural(P[n]) for n in B):
                continue  # a rotating cluster or a pin: check 1
            n_cand += 1
            try:
                r = _review_cut(ctx, geo, B, vB, js)
                if r is not None:
                    results.append(r)
            except Exception as e:  # noqa: BLE001
                notes.append(f"separable check {sorted(B)[:3]}...: {e!r}")
    # one finding per joint: cuts that differ only by the parts at the joint are the same seam
    order = {"CRITICAL": 0, "WARNING": 1}
    results.sort(key=lambda r: (order[r["sev"]], -r["worst"], len(r["B"])))
    kept = []
    for r in results:
        same = False
        for k in kept:
            near = k["other"] | r["other"] | k["touch"] | r["touch"]
            diff = {n for n in (r["B"] ^ k["B"]) if _structural(P[n])}
            if diff <= near or (r["check"] == k["check"] == "plate_seam" and r["other"] == k["other"]):
                same = True
                break
        if not same:
            kept.append(r)
            (crit if r["sev"] == "CRITICAL" else warn).append(r["finding"])
    return n_cand


def _review_cut(ctx: _Ctx, geo: _Geo, B: set[str], v: np.ndarray, js) -> dict | None:
    """Strength of the joint of a separable group B (moving along v) under the design loads; a result dict when it
    is a finding."""
    P = ctx.P
    rest = [n for n in P if n not in B and n in ctx.connected]
    cross = [j for j in js if (j["a"] in B) != (j["b"] in B)]
    if not cross:
        return None
    # an undeclared axle / pin through a hole across the cut blocks sideways motion
    for b in B:
        for o in rest:
            if geo.ov[geo.idx[b], geo.idx[o]]:
                for x in (b, o):
                    if P[x].bp.kind in ("axle", "pin") and abs(float(np.dot(P[x].axis(), v))) < 0.999:
                        return None
    t, hit = geo.first_contact(sorted(B), rest, v, SEP_TRAVEL_MM + 2 * SHRINK)
    if hit is not None and t - 2 * SHRINK < SEP_TRAVEL_MM:
        return None
    elems, n_studs, kinds = [], 0, Counter()
    stud_pts = []
    for j in cross:
        kinds[j["kind"]] += 1
        if j["kind"] == "stud":
            for p in j["pts"]:
                elems.append((p, j["f"]))
                stud_pts.append(p)
            n_studs += len(j["pts"])
        else:
            elems.append((j["line"][0], j["f"]))
    stud_only = set(kinds) == {"stud"}
    sig = ctx.significant(B)
    mB = sum(ctx.mass[n] for n in B)
    cap = sum(f for _, f in elems)
    show = _show(P, B)
    via = (f"{n_studs} stud{'s' if n_studs != 1 else ''}" if stud_only else
           ", ".join(f"{k} x{c}" for k, c in sorted(kinds.items())) + (f" ({n_studs} studs)" if n_studs else ""))
    other = sorted({j["b"] if j["a"] in B else j["a"] for j in cross})
    touch = {j["a"] if j["a"] in B else j["b"] for j in cross}

    def res(sev, finding):
        return dict(sev=sev, finding=finding, B=frozenset(B), other=set(other), touch=touch, worst=worst,
                    check=finding["check"])
    # loads
    cases = []
    comB = sum(ctx.mass[n] * ctx.com[n] for n in B) / mB if mB > 0 else None
    if comB is not None:
        cases.append(("its own weight with the vehicle picked up by the chassis (x3)", SF_HANG,
                      ([(comB, np.array([0.0, 0.0, -mB * G]))], [])))
    for ax_root, ws in ctx.axles.items():
        if not any(w in B for w in ws):
            continue
        for sx in (1.0, -1.0):
            cases.append((f"a hard landing on the wheels {', '.join(sorted(ws))} ({IMPACT_G:g} x the vehicle weight, "
                          f"traction {TRACTION_MU:g})", SF_DRIVE, ctx.landing_loads(B, ax_root, sx)))
    for m in ctx.motors:
        if m not in B:
            continue
        # does the motor's drive leave B? (its output cluster, or what that cluster meshes with / turns in)
        outs = {ctx.cluster_of[j["a"]] for j in ctx.joints if j["kind"] == "motor" and j["b"] == m}
        drive_parts = {n for r in outs for n in ctx.clusters.get(r, [])}
        partners = set()
        for ga, gb, _ in _mesh_pairs(ctx.design):
            if ga in drive_parts:
                partners.add(gb)
            if gb in drive_parts:
                partners.add(ga)
        bear = {j["b"] if j["a"] in drive_parts else j["a"] for j in ctx.joints
                if j["kind"] in ("hinge",) and ((j["a"] in drive_parts) != (j["b"] in drive_parts))}
        if (drive_parts | partners | bear) - B:
            tq, why = ctx.drive_torque(m)
            T = tq * P[m].R[:, 1]
            lab = (f"the stall torque of {m} ({tq:.2f} N m) reacting on its mount" if why == "stall" else
                   f"the drive torque of {m} up to wheel slip ({tq:.3f} N m) reacting on its mount")
            for s in (1.0, -1.0):
                cases.append((lab, SF_DRIVE, ([], [s * T])))
    worst, wcase, wmode = 0.0, None, None
    for label, sf, wrench in cases:
        r, mode = _joint_capacity(elems, v, wrench, n_shafts=sum(1 for j in cross if j["kind"] != "stud"))
        r *= sf
        if r > worst:
            worst, wcase, wmode = r, label, mode
    dir_txt = _fmt_dir(v)
    if sig and cap < 1e-9:
        msg = (f"{show} is held to the rest only by axles it can slide along ({via}); nothing stops it moving "
               f"{dir_txt} off {', '.join(other)}. Lock it with a pin across the axis or a stop.")
        return res("CRITICAL", _finding("separable", list(B) + other, msg, 0.0, None))
    if sig and worst > 1.0:
        msg = (f"{show} ({mB * 1000:.0f} g) can be pulled off {', '.join(other)} {dir_txt}: the only things holding "
               f"it are {via}, all pulling apart the same way, with nothing interlocking across the joint. Under "
               f"{wcase} the joint is loaded to {worst:.1f} x its strength ({wmode}, {cap:.1f} N total pull-off), so "
               f"it comes apart. Clamp it from the other side, pin through both groups, or spread the joint.")
        return res("CRITICAL", _finding("separable", list(B) + other, msg, worst, 1.0))
    if not sig:
        return None
    if stud_only and n_studs <= FEW_STUDS:
        msg = (f"{show} ({mB * 1000:.0f} g) is held to {', '.join(other)} by only {n_studs} stud"
               f"{'s' if n_studs != 1 else ''}, all pulling off {dir_txt}; a knock pops it off "
               f"(worst design load {worst:.2f} x strength). Add studs or a pin across the joint.")
        return res("WARNING", _finding("weak_stud_joint", list(B) + other, msg, n_studs, FEW_STUDS))
    if stud_only and _collinear(stud_pts) and worst > SINGLE_ROW_MOMENT_FRAC:
        msg = (f"{show} is held to {', '.join(other)} by a single row of {n_studs} studs, which works like a hinge: "
               f"bending about the row peels the studs one by one (worst design load {worst:.2f} x strength). Use a "
               f"wider joint (two rows) or brace it.")
        return res("WARNING", _finding("single_row_studs", list(B) + other, msg, worst, 1.0))
    if (stud_only and len(other) == 1 and P[other[0]].bp.kind == "plate" and any(w in B for w in ctx.wheels)
            and any(w in rest for w in ctx.wheels) and len(touch) >= 2):
        msg = (f"{show} carries wheels and is joined to the rest only through the studs of the plate {other[0]}: a "
               f"single plate across a joint flexes, so the frame bends at that seam under load (worst design load "
               f"{worst:.2f} x stud strength). Overlap the frame members or add a second layer across the seam.")
        return res("WARNING", _finding("plate_seam", list(B) + other, msg, worst, 1.0))
    return None


# ----------------------------------------------------------------------------------------------------------------
# check 3: pivoting members
# ----------------------------------------------------------------------------------------------------------------


def _check_pivots(ctx: _Ctx, crit, warn, notes) -> dict:
    P, joints = ctx.P, ctx.joints
    parts = [n for n in P if n in ctx.connected]
    pset = set(parts)
    js = [j for j in joints if j["a"] in pset and j["b"] in pset]
    uf = _UF(parts)
    for j in js:
        if not j["rot"]:
            uf.union(j["a"], j["b"])
    body = {n: uf.find(n) for n in parts}
    members = defaultdict(list)
    for n in parts:
        members[body[n]].append(n)
    # rotating bodies: a cross cluster that turns in hinges / on a motor output
    turning = set()
    for b, ms in members.items():
        mset = set(ms)
        turns = False
        for j in js:
            if j["kind"] not in ("hinge", "motor") or (j["a"] in mset) == (j["b"] in mset):
                continue
            # the turning side of a hinge / motor joint is the axle (child), or a free-turning gear (parent)
            if j["a"] in mset or (j["b"] in mset and P[j["b"]].bp.free_bore):
                turns = True
        if turns and all(ctx.cluster_of[m] == ctx.cluster_of[ms[0]] for m in ms):
            turning.add(b)
    structural = {b for b, ms in members.items() if b not in turning and any(_structural(P[m]) for m in ms)}
    # hinge edges between bodies, by line
    lines: list = []
    edges = []  # (body a, body b, line id, joint)
    for j in js:
        if not j["rot"]:
            continue
        ba, bb = body[j["a"]], body[j["b"]]
        if ba == bb:
            continue
        k = _line_key(lines, j["line"][0], j["line"][1])
        edges.append((ba, bb, k, j))
    ch = body[ctx.chassis]
    bodies = list(members)

    def reach(skip_line):
        adj = defaultdict(set)
        for a, b, k, _ in edges:
            if k != skip_line:
                adj[a].add(b)
                adj[b].add(a)
        seen, stack = {ch}, [ch]
        while stack:
            x = stack.pop()
            for y in adj[x]:
                if y not in seen:
                    seen.add(y)
                    stack.append(y)
        return seen, adj

    full, _ = reach(None)
    flagged_bodies = set()
    by_line = defaultdict(list)
    for a, b, k, j in edges:
        by_line[k].append(j)
    for k, jl in by_line.items():
        if any(j["intended"] for j in jl):
            continue
        seen, adj = reach(k)
        lost = [b for b in bodies if b in full and b not in seen]
        if not lost:
            continue
        # components of the detached bodies
        comp_seen = set()
        for b0 in lost:
            if b0 in comp_seen:
                continue
            comp, stack = {b0}, [b0]
            while stack:
                x = stack.pop()
                for y in adj[x]:
                    if y not in comp and y not in seen:
                        comp.add(y)
                        stack.append(y)
            comp_seen |= comp
            if not (comp & structural):
                continue
            B = {n for b in comp for n in members[b]}
            allp = {n for b in full for n in members[b]}
            if sum(ctx.mass[n] for n in B) > 0.5 * sum(ctx.mass[n] for n in allp):
                B = allp - B  # the declared chassis is the small side: it is what swings
            kinds = {j["kind"] for j in jl}
            motors_on_line = {j.get("motor") for j in jl if j["kind"] == "motor"}
            spins_on_own_shaft = [m for m in motors_on_line if m in B]
            if "motor" in kinds and not spins_on_own_shaft:
                continue  # the driven cluster turning on its motor: intended
            q, a = lines[k]
            hinge_names = sorted({(f"{j['a']}/{j['b']}" if j["kind"] == "stud" else
                                   j["a"] if P[j["a"]].bp.kind in ("pin", "axle") else j["b"]) for j in jl})
            load = []
            if any(n in ctx.wheels for n in B):
                load.append("wheels")
            if any(P[n].bp.kind in LOAD_KINDS for n in B):
                load.append(", ".join(sorted(n for n in B if P[n].bp.kind in LOAD_KINDS)))
            for ga, gb, _ in _mesh_pairs(ctx.design):
                if (ga in B) != (gb in B):
                    load.append(f"gear {ga if ga in B else gb} meshing outside it")
                    break
            mB = sum(ctx.mass[n] for n in B)
            if mB >= SIGNIFICANT_MASS_FRAC * ctx.M and not load:
                load.append(f"{mB * 1000:.0f} g")
            Bs = sorted(n for n in B if _structural(P[n]))
            show = _show(P, B)
            what = (" and ".join(sorted(kinds)) if kinds else "hinge")
            if spins_on_own_shaft:
                msg = (f"motor {', '.join(spins_on_own_shaft)} is held only on the line of its own output shaft "
                       f"({', '.join(hinge_names)}): under torque the motor housing turns instead of the shaft. Fix "
                       f"the housing with pins or studs off that line.")
                crit.append(_finding("pivoting", list(B), msg, 1, 0))
            elif kinds <= {"hinge"}:
                msg = (f"{show} hangs on the axle {', '.join(hinge_names)} alone (axis {_fmt_dir(a)}): it is free to "
                       f"rock about it{(' while carrying ' + '; '.join(load)) if load else ''}. Fine for a deliberate "
                       f"rocker or bogie; otherwise fix it with a second pin or a stud joint off that axis.")
                warn.append(_finding("pivoting", list(B), msg, len(jl), 2))
            elif load:
                why = ("a single stud is a pivot" if kinds == {"stud"} else
                       "a single pin is a hinge; pin friction does not hold a loaded member")
                msg = (f"{show} is attached to the rest only along one line ({what}: {', '.join(hinge_names)}, axis "
                       f"{_fmt_dir(a)}) and carries {'; '.join(load)}: it swings about that line under load ({why}). "
                       f"Add a second pin off that line, a brace, or a stud joint of two or more studs.")
                crit.append(_finding("pivoting", list(B), msg, 1, 2))
            else:
                msg = (f"{show} is attached only along one line ({what}: {', '.join(hinge_names)}, axis "
                       f"{_fmt_dir(a)}) and can flop about it. Add a second pin or a stud joint off that line.")
                warn.append(_finding("pivoting", list(B), msg, 1, 2))
            flagged_bodies |= comp | {body[n] for n in B if n in body}
    # remaining mobility: kinematic null space with deliberate pivots and motor outputs locked
    dof = 0
    try:
        dof, moving = _mechanism_modes(ctx, members, body, edges, ch, structural)
        rest_moving = [b for b in moving if b not in flagged_bodies]
        if rest_moving:
            # group them by adjacency (through pins and other non-frame bodies)
            adj = defaultdict(set)
            for a, b, _, j in edges:
                adj[a].add(b)
                adj[b].add(a)
            left = set(rest_moving)
            passable = set(rest_moving) | {b for b in members if b not in structural and b != ch}
            while left:
                b0 = left.pop()
                comp, stack, vis = {b0}, [b0], {b0}
                while stack:
                    x = stack.pop()
                    for y in adj[x]:
                        if y in passable and y not in vis:
                            vis.add(y)
                            stack.append(y)
                            if y in left:
                                left.discard(y)
                                comp.add(y)
                B = {n for b in comp for n in members[b]}
                load = sorted(n for n in B if P[n].bp.kind in LOAD_KINDS) + (["wheels"] if any(
                    w for w in ctx.wheels if any(ctx.cluster_of[w] == ctx.cluster_of[n] for n in B)) else [])
                Bs = sorted(n for n in B if _structural(P[n]))
                if not Bs:
                    continue
                show = _show(P, B)
                msg = (f"{show} form a linkage of single-pin joints that can rack (e.g. a pinned parallelogram)"
                       f"{(' carrying ' + ', '.join(load)) if load else ''}: only pin friction keeps the shape. Add a "
                       f"diagonal brace or a second pin in one of the joints.")
                warn.append(_finding("linkage", Bs, msg, len(comp), 0))
    except Exception as e:  # noqa: BLE001
        notes.append(f"mechanism analysis skipped: {e!r}")
    return {"bodies": len(members), "hinge_lines": len(lines), "mechanism_dof": dof}


def _mechanism_modes(ctx, members, body, edges, ch, structural):
    """Instantaneous mobility of the structural bodies (twists of the null space of the joint constraints) with
    deliberate pivots and motor outputs held fixed. Returns (dof among structural bodies, moving bodies). Solved per
    group of bodies hanging off the chassis (they are independent once the chassis is fixed)."""
    uf = _UF(list(members))
    hinges = []
    for a, b, k, j in edges:
        if j["intended"] or j["kind"] == "motor":
            uf.union(a, b)
        else:
            hinges.append((a, b, j))
    # a pin body on one line is just a hinge between its two neighbours; two hinges on different lines between the
    # same two bodies lock them together: merge until nothing changes (keeps the systems small)
    lines: list = []
    hl = [(a, b, _line_key(lines, j["line"][0], j["line"][1]), j) for a, b, j in hinges]
    pin_bodies = {b for b, ms in members.items() if all(ctx.P[m].bp.kind == "pin" for m in ms)}
    changed = True
    while changed:
        changed = False
        between = defaultdict(set)
        for a, b, k, _ in hl:
            ra, rb = uf.find(a), uf.find(b)
            if ra != rb:
                between[frozenset((ra, rb))].add(k)
        for pair, ks in between.items():
            if len(ks) >= 2:
                x, y = tuple(pair)
                uf.union(x, y)
                changed = True
        if changed:
            continue
        # contract pin bodies whose hinges all lie on one line (2 neighbours)
        inc = defaultdict(list)
        for i, (a, b, k, j) in enumerate(hl):
            ra, rb = uf.find(a), uf.find(b)
            if ra != rb:
                inc[ra].append((i, rb, k))
                inc[rb].append((i, ra, k))
        for pb in list(pin_bodies):
            r = uf.find(pb)
            if r != pb or pb == uf.find(ch):
                continue
            es = inc.get(pb, [])
            nbrs = {o for _, o, _ in es}
            if len(es) >= 2 and len({k for _, _, k in es}) == 1 and len(nbrs) == 2:
                o1, o2 = sorted(nbrs)
                k = es[0][2]
                hl = [h for i, h in enumerate(hl) if i not in {e[0] for e in es}]
                hl.append((o1, o2, k, {"line": lines[k], "intended": False, "kind": "pin"}))
                pin_bodies.discard(pb)
                changed = True
                break
    root_ch = uf.find(ch)
    groups = uf.groups()  # super body -> bodies
    rows = []
    for a, b, k, j in hl:
        ra, rb = uf.find(a), uf.find(b)
        if ra == rb:
            continue  # a hinge inside a locked group: closed loop, ignored
        p, ax = j["line"]
        p = np.asarray(p) / 100.0  # 100 mm units keep the matrix balanced
        sc = np.concatenate([ax, np.cross(p, ax)])
        sc /= np.linalg.norm(sc)
        rows.append((ra, rb, np.eye(6) - np.outer(sc, sc)))
    # components of the hinge graph without the chassis
    adj = defaultdict(set)
    for ra, rb, _ in rows:
        adj[ra].add(rb)
        adj[rb].add(ra)
    seen = {root_ch}
    dof, moving = 0, []
    for start in list(adj):
        if start in seen:
            continue
        comp, stack = {start}, [start]
        seen.add(start)
        while stack:
            x = stack.pop()
            for y in adj[x]:
                if y not in seen:
                    seen.add(y)
                    comp.add(y)
                    stack.append(y)
        bl = sorted(comp)
        if len(bl) > 150:
            ctx.notes.append(f"mechanism analysis skipped for a group of {len(bl)} hinged bodies (too large)")
            continue
        col = {x: i for i, x in enumerate(bl)}
        crow = [(ra, rb, Pm) for ra, rb, Pm in rows if ra in col or rb in col]
        A = np.zeros((6 * len(crow), 6 * len(bl)))
        for r, (ra, rb, Pm) in enumerate(crow):
            if ra in col:
                A[6 * r:6 * r + 6, 6 * col[ra]:6 * col[ra] + 6] += Pm
            if rb in col:
                A[6 * r:6 * r + 6, 6 * col[rb]:6 * col[rb] + 6] -= Pm
        w, V = np.linalg.eigh(A.T @ A)
        N = V[:, w < 1e-9 * max(float(w.max()), 1.0)]
        if N.shape[1] == 0:
            continue
        srows = [x for x in bl if any(bb in structural for bb in groups[x])]
        if not srows:
            continue
        idx = np.concatenate([np.arange(6 * col[x], 6 * col[x] + 6) for x in srows])
        Ns = N[idx]
        dof += int((np.linalg.svd(Ns, compute_uv=False) > 1e-6).sum())
        for i, x in enumerate(srows):
            if np.linalg.norm(Ns[6 * i:6 * i + 6]) > 1e-6:
                moving += [bb for bb in groups[x] if bb in structural]
    return dof, moving


# ----------------------------------------------------------------------------------------------------------------
# check 4: assembly access
# ----------------------------------------------------------------------------------------------------------------


def _check_insertion(ctx: _Ctx, geo: _Geo, warn, notes) -> None:
    P = ctx.P
    # rigid bodies (studs, cross fits, pins) to know what belongs to the structure a shaft sits in
    uf = _UF(list(P))
    for j in ctx.joints:
        if j["kind"] in ("stud",) or (j["kind"] == "pin" and P[j["a"]].bp.kind != "pin"):
            uf.union(j["a"], j["b"])
    pin_par = defaultdict(set)
    for j in ctx.joints:
        if j["kind"] == "pin":
            pin_par[j["a"]].add(j["b"])
    for p_, ps in pin_par.items():  # parts pinned together by two or more pins are one frame
        pass
    pairs = Counter()
    for p_, ps in pin_par.items():
        ps = sorted(ps)
        for i in range(len(ps)):
            for k in range(i + 1, len(ps)):
                pairs[(ps[i], ps[k])] += 1
    for (a, b), c in pairs.items():
        if c >= 2:
            uf.union(a, b)
    for n, pl in P.items():
        if pl.bp.kind not in ("axle", "pin") or n not in ctx.connected:
            continue
        try:
            ax = pl.axis()
            mine = set(ctx.clusters.get(ctx.cluster_of[n], [n]))
            bearings = {j["b"] if j["a"] == n else j["a"] for j in ctx.joints
                        if n in (j["a"], j["b"]) and j["kind"] in ("hinge", "pin", "motor")}
            if not bearings:
                continue
            frame = {uf.find(b) for b in bearings}
            others = [o for o in P if o not in mine and o not in bearings and uf.find(o) in frame]
            if not others:
                continue
            L = 2 * (pl.bp.axle_half_len or 8.0) + INSERT_MARGIN_MM
            blocked = []
            for sgn in (1.0, -1.0):
                t, hit = geo.first_contact([n], others, sgn * ax, L)
                if hit is None:
                    break
                blocked.append(hit[1])
            if len(blocked) == 2:
                msg = (f"{n} cannot be pushed in: along its axis both ends are closed by {blocked[0]} and "
                       f"{blocked[1]}, which belong to the same rigid frame as the parts it runs in. Build it in "
                       f"before closing the frame (and check that order is possible) or leave one end open.")
                warn.append(_finding("insertion", [n] + blocked, msg, None, None))
        except Exception as e:  # noqa: BLE001
            notes.append(f"insertion check {n}: {e!r}")


# ----------------------------------------------------------------------------------------------------------------
# metrics
# ----------------------------------------------------------------------------------------------------------------


def _metrics(ctx: _Ctx, design: Design) -> dict:
    P = ctx.P
    studs = [j for j in ctx.joints if j["kind"] == "stud"]
    pins = [n for n, pl in P.items() if pl.bp.kind == "pin"]
    pair_kinds = defaultdict(set)
    for j in ctx.joints:
        if j["kind"] == "stud":
            pair_kinds[frozenset((j["a"], j["b"]))].add("stud")
    by_pin = defaultdict(set)
    for j in ctx.joints:
        if j["kind"] == "pin" and P[j["a"]].bp.kind == "pin":
            by_pin[j["a"]].add(j["b"])
        elif j["kind"] == "pin":
            pair_kinds[frozenset((j["a"], j["b"]))].add("pin")
    for pn, ps in by_pin.items():
        ps = sorted(ps)
        for i in range(len(ps)):
            for k in range(i + 1, len(ps)):
                pair_kinds[frozenset((ps[i], ps[k]))].add("pin")
    for j in ctx.joints:
        if j["kind"] in ("hinge", "cross") and _structural(P[j["a"]]) and _structural(P[j["b"]]):
            pair_kinds[frozenset((j["a"], j["b"]))].add("axle")
    n_pairs = len(pair_kinds)
    stud_only = sum(1 for k in pair_kinds.values() if k == {"stud"})
    # ground supports along x
    xs = sorted(float(ctx.contact[w][0]) for w in ctx.wheels)
    sup = []
    for x in xs:
        if not sup or x - sup[-1] > 5.0:
            sup.append(x)
    span = max((b - a for a, b in zip(sup, sup[1:])), default=0.0)
    lo = min((geo_lo for geo_lo in (pl.aabb()[0][0] for pl in P.values())), default=0.0)
    hi = max((pl.aabb()[1][0] for pl in P.values()), default=0.0)
    return {
        "parts": len(design.parts),
        "part_types": len({p.part for p in design.parts}),
        "mass_g": round(ctx.M * 1000, 1),
        "stud_joints": len(studs),
        "studs": sum(len(j["pts"]) for j in studs),
        "pins": len(pins),
        "joined_pairs": n_pairs,
        "stud_only_fraction": round(stud_only / n_pairs, 3) if n_pairs else 0.0,
        "supports_x_mm": [round(x, 1) for x in sup],
        "longest_span_mm": round(span, 1),
        "overhang_front_mm": round(hi - sup[-1], 1) if sup else None,
        "overhang_rear_mm": round(sup[0] - lo, 1) if sup else None,
    }


# ----------------------------------------------------------------------------------------------------------------
# entry points
# ----------------------------------------------------------------------------------------------------------------


def review(design: Design) -> dict:
    """Design-quality review: {"critical": [...], "warnings": [...], "metrics": {...}, "clusters": [...]}; each
    finding is {"check", "parts", "message", "value", "limit"}. A CRITICAL finding is a defect that makes the real
    build come apart or stop driving; a WARNING is a weakness worth fixing."""
    crit, warn, notes = [], [], []
    metrics: dict = {"notes": notes}
    clusters: list = []
    try:
        if isinstance(design, dict):
            design = Design.from_dict(design)
        P = place(design)
        missing = [p.name for p in design.parts if p.name not in P]
        if missing:
            notes.append(f"parts with unknown geometry skipped: {', '.join(missing[:8])}")
        if not P:
            notes.append("no parts with known geometry")
            return {"critical": crit, "warnings": warn, "metrics": metrics, "clusters": clusters}
        geo = _Geo(P)
        joints = _joints(design, P, notes)
        ctx = _Ctx(design, P, joints)
        ctx.notes = notes
        unconnected = [n for n in P if n not in ctx.connected]
        if unconnected:
            notes.append(f"parts not connected to the chassis (not reviewed): {', '.join(sorted(unconnected)[:8])}"
                         + (" ..." if len(unconnected) > 8 else ""))
        for name, fn in (
            ("axial", lambda: clusters.extend(_check_axial(design, P, geo, joints, crit, warn, notes))),
            ("separable", lambda: metrics.__setitem__("separable_candidates",
                                                       _check_separable(ctx, geo, crit, warn, notes))),
            ("pivots", lambda: metrics.update(_check_pivots(ctx, crit, warn, notes))),
            ("insertion", lambda: _check_insertion(ctx, geo, warn, notes)),
            ("metrics", lambda: metrics.update(_metrics(ctx, design))),
        ):
            try:
                fn()
            except Exception as e:  # noqa: BLE001
                notes.append(f"{name} check failed: {e!r}")
    except Exception as e:  # noqa: BLE001
        notes.append(f"review failed: {e!r}")
    _dedupe(crit)
    _dedupe(warn)
    return {"critical": crit, "warnings": warn, "metrics": metrics, "clusters": clusters}


def _dedupe(fs: list[dict]) -> None:
    seen = set()
    keep = []
    for f in fs:
        k = (f["check"], f["message"])
        if k not in seen:
            seen.add(k)
            keep.append(f)
    fs[:] = keep


def summary(r: dict) -> str:
    lines = []
    for sev, key in (("CRITICAL", "critical"), ("WARNING", "warnings")):
        for f in r[key]:
            lines.append(f"  {sev} [{f['check']}] {f['message']}")
    m = r["metrics"]
    keys = ("parts", "part_types", "mass_g", "stud_joints", "studs", "pins", "stud_only_fraction", "longest_span_mm",
            "mechanism_dof")
    lines.append("  metrics: " + ", ".join(f"{k}={m.get(k)}" for k in keys if k in m))
    for n in m.get("notes", [])[:6]:
        lines.append(f"  note: {n}")
    return "\n".join(lines)


def _main(argv: list[str]) -> int:
    import time  # noqa: PLC0415

    out_json = None
    if "--json" in argv:
        k = argv.index("--json")
        out_json = argv[k + 1]
        argv = argv[:k] + argv[k + 2:]
    if argv and argv[0] == "--all":
        root = argv[1]
        files = sorted(glob.glob(os.path.join(root, "*", "*_L*", "submission.json")))  # <agent>/<bench>_L<n>/
        if not files:
            files = sorted(glob.glob(os.path.join(root, "**", "submission.json"), recursive=True))
        files = [f for f in files if "/all_L" not in f and "/workspace/" not in f
                 and not any(p.startswith("_") for p in os.path.relpath(f, root).split(os.sep))]
        rows, results = [], {}
        per_agent = defaultdict(lambda: Counter())
        common = Counter()
        for f in files:
            rel = os.path.relpath(f, root)
            parts_ = rel.split(os.sep)
            agent = parts_[-3] if len(parts_) >= 3 else "?"
            t0 = time.time()
            try:
                with open(f) as fh:
                    design = Design.from_dict(json.load(fh))
            except Exception as e:  # noqa: BLE001
                print(f"{rel:70s} not a single design ({e!r:.60s}), skipped")
                continue
            r = review(design)
            dt = time.time() - t0
            results[rel] = r
            nc, nw = len(r["critical"]), len(r["warnings"])
            per_agent[agent]["designs"] += 1
            per_agent[agent]["with_critical"] += nc > 0
            per_agent[agent]["with_warning"] += nw > 0
            for f_ in r["critical"]:
                common["CRITICAL " + f_["check"]] += 1
            for f_ in r["warnings"]:
                common["WARNING " + f_["check"]] += 1
            crit_kinds = Counter(x["check"] for x in r["critical"])
            rows.append((rel, nc, nw, dt, dict(crit_kinds)))
            print(f"{rel:70s} crit {nc:2d} warn {nw:2d} {dt:5.2f}s {dict(crit_kinds) if crit_kinds else ''}",
                  flush=True)
        print("\nper agent: designs / with >= 1 CRITICAL / with >= 1 WARNING")
        for a, c in sorted(per_agent.items()):
            print(f"  {a:45s} {c['designs']:4d} {c['with_critical']:4d} {c['with_warning']:4d}")
        print("\nfindings (count over all designs):")
        for k, n in common.most_common():
            print(f"  {k:35s} {n}")
        if out_json:
            with open(out_json, "w") as fh:
                json.dump(results, fh, indent=1, default=str)
        return 0
    for f in argv:
        t0 = time.time()
        r = review(Design.from_dict(json.load(open(f))))
        print(f"{f}: {len(r['critical'])} critical, {len(r['warnings'])} warnings ({time.time() - t0:.2f} s)")
        print(summary(r))
        if out_json:
            with open(out_json, "w") as fh:
                json.dump(r, fh, indent=1, default=str)
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
