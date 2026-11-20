# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Rigid pinned laps (``--laps rigid``, opt-in, round 3g).

In the reference model every Technic pin is its own rigid body on two fixed joints. A lap of two liftarms joined by
several pins is a chain of sub-gram bodies that the solver lets sag at long levers (gap L24). In real LEGO a lap
joined by two or more pins -- or by one pin plus an axle through both parts that is locked (cross fit) to one of
them -- cannot turn: it is one rigid part, and what can fail is a pin, in shear or by pulling out.

This model therefore

* merges every such lap (the two parts and their pins, transitively) into one rigid body (the parts become
  colliders of a representative member, with the cluster's true mass and inertia);
* keeps single-pin joints (without a locking axle) as they are;
* checks the pins of every lap that sits on a *mechanism* (a cluster whose only link to the rest of the vehicle is one
  member, e.g. an arm on its 60483 lock): each step, the wrench that the part beyond the lap receives (contact
  forces on its colliders from the contact report, plus its weight) is carried to the lap and shared by its pins:
  in-plane force and the bending moment about the pin axis as shear (a couple over the pin positions), out-of-plane
  force and moments as axial pull-out. An attempt fails "broke (lap overloaded: ...)" when a pin's smoothed load
  exceeds :data:`PIN_SHEAR_LIMIT_N` or :data:`PIN_PULLOUT_LIMIT_N`.

Limits (conservative): a Technic friction pin pulls out of a hole at roughly 5-10 N (friction
ridges, measured by hobbyists and in LEGO clutch tests); its shear strength (4.8 mm ABS/POM shank) is several hundred
newtons, capped here at 50 N per pin.
"""

from __future__ import annotations

import math

import numpy as np

PIN_SHEAR_LIMIT_N = 50.0
PIN_PULLOUT_LIMIT_N = 5.0
LAP_SMOOTH_S = 0.1


def _uf(names):
    parent = {n: n for n in names}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    return parent, find


def rigid_laps(design):
    """Returns (merged, laps).

    ``merged``: {part: representative body} for every member of a rigid lap cluster except the representative.
    ``laps``: [{"parts": (a, b), "pins": [...], "cluster": body}] for every pair of parts joined rigidly by pins."""
    from .design import axle_through_holes, place  # noqa: PLC0415
    from .parts import bench_parts  # noqa: PLC0415

    cat = bench_parts()
    P = place(design)
    held: dict[str, list[str]] = {}
    for c in design.connections:
        if c.kind == "pin":
            held.setdefault(c.child, []).append(c.parent)
    pairs: dict[frozenset, list[str]] = {}
    shocks = {p.name for p in design.parts if p.part in cat and cat[p.part].kind == "shock"}  # round 4b: they turn
    for pin, pars in held.items():
        if len(pars) == 2 and pars[0] != pars[1] and not shocks & set(pars):
            pairs.setdefault(frozenset(pars), []).append(pin)
    laps = []
    for pair, pins in pairs.items():
        a, b = sorted(pair)
        rigid = len(pins) >= 2
        if not rigid:  # one pin plus an axle, cross-fitted to one part, through a hole of the other
            for x, y in ((a, b), (b, a)):
                axles = [c.parent for c in design.connections if c.child == x and c.kind == "cross"]
                if any(ax in P and y in P and axle_through_holes(P[ax], P[y]) for ax in axles):
                    rigid = True
        if rigid:
            laps.append({"parts": (a, b), "pins": sorted(pins)})
    names = [p.name for p in design.parts]
    parent, find = _uf(names)
    for lap in laps:
        for n in (lap["parts"][1], *lap["pins"]):
            parent[find(n)] = find(lap["parts"][0])
    clusters: dict[str, list[str]] = {}
    for lap in laps:
        for n in (*lap["parts"], *lap["pins"]):
            clusters.setdefault(find(n), [])
            if n not in clusters[find(n)]:
                clusters[find(n)].append(n)
    merged = {}
    for root, members in clusters.items():
        if design.chassis in members:
            body = design.chassis
        else:
            body = max((m for m in members if cat[design.part(m).part].kind != "pin"),
                       key=lambda m: cat[design.part(m).part].mass_g)  # fmt: skip
        for m in members:
            if m != body:
                merged[m] = body
        for lap in laps:
            if lap["parts"][0] in members:
                lap["cluster"] = body
    return merged, laps


def cluster_mass(design, body: str, members: list[str]) -> dict:
    """Mass, centre of mass (body frame, m) and principal inertia (diagonal + principal-axes quaternion wxyz, body
    frame) of a rigid cluster: each part a solid box (its collision box) or cylinder, at its design pose."""
    from .design import rot_matrix  # noqa: PLC0415
    from .parts import Box, bench_parts  # noqa: PLC0415

    cat = bench_parts()
    parts = {p.name: p for p in design.parts}
    items = []
    for n in [body] + [m for m in members if m != body]:
        p, bp = parts[n], cat[parts[n].part]
        R = rot_matrix(p.yaw_deg, p.spin_deg)
        m = bp.mass_g * 1e-3
        s = bp.shape
        if isinstance(s, Box):
            lo, hi = np.array(s.lo) * 1e-3, np.array(s.hi) * 1e-3
            c_loc, ext = (lo + hi) / 2, hi - lo
            I_loc = m / 12.0 * np.diag([ext[1] ** 2 + ext[2] ** 2, ext[0] ** 2 + ext[2] ** 2, ext[0] ** 2 + ext[1] ** 2])
        else:
            r, h = s.radius * 1e-3, 2 * s.half_len * 1e-3
            c_loc = np.array(s.centre) * 1e-3
            ia, it = 0.5 * m * r * r, m * (3 * r * r + h * h) / 12.0
            I_loc = np.diag([ia if s.axis == "x" else it, ia if s.axis == "y" else it, it])
        c = np.array(p.pos_mm) * 1e-3 + R @ c_loc
        items.append((m, c, R @ I_loc @ R.T))
    M = sum(i[0] for i in items)
    C = sum(m * c for m, c, _ in items) / M
    I = np.zeros((3, 3))
    for m, c, Ic in items:
        d = c - C
        I += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    pb = parts[body]
    Rb = rot_matrix(pb.yaw_deg, pb.spin_deg)
    com_b = Rb.T @ (C - np.array(pb.pos_mm) * 1e-3)
    Ib = Rb.T @ I @ Rb
    w, V = np.linalg.eigh(Ib)
    if np.linalg.det(V) < 0:
        V[:, 0] = -V[:, 0]
    q = _mat_to_quat_wxyz(V)
    return {"mass_kg": M, "com_m": tuple(float(v) for v in com_b), "inertia": tuple(float(v) for v in w), "axes_wxyz": q}


def _mat_to_quat_wxyz(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)


def mechanism_laps(design, laps: list[dict]) -> list[dict]:
    """Laps whose far side can be isolated: in a cluster that touches the rest of the vehicle through one member
    only (the anchor: an arm's 60483 on its axle). For each such lap: the anchor-side part, the far-side members
    (the parts beyond the lap, and the pins among them), the pins of the lap, and the far-side parts borne on an
    axle that runs in a near-side hole ({part: axle})."""
    from .design import axle_through_holes, place  # noqa: PLC0415

    P = place(design)
    members_of: dict[str, set] = {}
    for lap in laps:
        members_of.setdefault(lap["cluster"], set()).update((*lap["parts"], *lap["pins"]))
    out = []
    for body, mem in members_of.items():
        # parts hanging on the cluster alone (a tip wheel on its axle): not a link to the vehicle, but load on the
        # member they hang on
        rest = {c: set() for c in [q.name for q in design.parts] if c not in mem}
        for c in design.connections:
            if c.child in rest and c.parent in rest:
                rest[c.child].add(c.parent), rest[c.parent].add(c.child)
        linked = _reach(rest, design.chassis, set()) if design.chassis in rest else set()
        payload: dict[str, set] = {}
        for c in design.connections:
            for m, o in ((c.child, c.parent), (c.parent, c.child)):
                if m in mem and o not in mem and o not in linked:
                    payload.setdefault(m, set()).update(_reach(rest, o, set()))
        ext = {c.child if c.parent in mem else c.parent
               for c in design.connections if (c.child in mem) != (c.parent in mem)} & linked  # fmt: skip
        anchors = {n for n in mem if any((c.child == n and c.parent in ext) or (c.parent == n and c.child in ext)
                                         for c in design.connections)}  # fmt: skip
        if len(anchors) != 1 or design.chassis in mem:
            continue
        anchor = next(iter(anchors))
        adj: dict[str, set] = {n: set() for n in mem}
        for lap in laps:
            if lap["cluster"] != body:
                continue
            a, b = lap["parts"]
            for p in lap["pins"]:
                adj[a].add(p), adj[p].add(a), adj[b].add(p), adj[p].add(b)
        for lap in laps:
            if lap["cluster"] != body:
                continue
            a, b = lap["parts"]
            # far side: parts reachable from the anchor-far part without crossing this lap's pins
            blocked = set(lap["pins"])
            near = _reach(adj, anchor, blocked)
            far_part = b if a in near else a
            near_part = a if far_part == b else b
            far = _reach(adj, far_part, blocked | {near_part})
            far |= set().union(*[payload.get(n, set()) for n in far])
            # a far-side axle that also runs through a hole of a near-side member (a motor's output shaft through
            # the rail it is pinned to) is borne there: its radial loads go to that hole, only its torque about its
            # own axis reaches the far side (the motor's reaction)
            borne = {}
            nearset = mem - far
            for n in far:
                if n in P and P[n].bp.kind == "axle" and any(axle_through_holes(P[n], P[m]) for m in nearset if m in P):
                    borne[n] = n
            for c in design.connections:
                if c.kind == "cross" and c.parent in borne and c.child in far:
                    borne[c.child] = c.parent
            out.append({"lap": (near_part, far_part), "pins": lap["pins"], "far": sorted(far), "anchor": anchor,
                        "borne": borne})  # fmt: skip
    return out


def _reach(adj, start, blocked):
    seen, todo = {start}, [start]
    while todo:
        n = todo.pop()
        for m in adj.get(n, ()):
            if m not in seen and m not in blocked:
                seen.add(m)
                todo.append(m)
    return seen


def pin_loads(far_force, far_moment, pins_world, pin_axis) -> tuple[float, float]:
    """Worst pin (shear, pull-out) [N] of a lap.

    ``far_force`` / ``far_moment``: the wrench on the far side (about the world origin, SI); ``pins_world``: the
    bearing points of the lap (its pins, and for a pin-plus-axle lap the locking axle's hole); ``pin_axis``: their
    common axis. The wrench is moved to the bearings' centroid and shared:

    * shear: the in-plane force / n + the moment about the pin axis as a couple (M d_max / sum d^2);
    * pull-out: the axial force / n + the moment about the in-plane axis normal to the pin line as a couple, + the
      moment about the pin line (the lap trying to peel open) over a 4 mm half-width, shared by the n pins."""
    P = np.array(pins_world, dtype=float)
    n = max(len(P), 1)
    c = P.mean(axis=0)
    F = np.array(far_force, dtype=float)
    M = np.array(far_moment, dtype=float) - np.cross(c, F)
    ax = np.array(pin_axis, dtype=float)
    ax /= np.linalg.norm(ax)
    r = P - c
    r_perp = r - np.outer(r @ ax, ax)
    d = np.linalg.norm(r_perp, axis=1)
    s2 = float(np.sum(d * d))
    if s2 > 1e-12:
        u = r_perp[int(np.argmax(d))] / d.max()  # the pin line
    else:
        u = np.cross(ax, [1.0, 0.0, 0.0])
        u = u / np.linalg.norm(u) if np.linalg.norm(u) > 1e-9 else np.array([0.0, 0.0, 1.0])
    e = np.cross(ax, u)
    F_ax = abs(float(F @ ax))
    F_pl = float(np.linalg.norm(F - (F @ ax) * ax))
    M_ax, M_u, M_e = abs(float(M @ ax)), abs(float(M @ u)), abs(float(M @ e))
    couple = (lambda m: m * float(d.max()) / s2) if s2 > 1e-12 else (lambda m: m / 0.004)
    shear = F_pl / n + couple(M_ax)
    pull = F_ax / n + couple(M_e) + M_u / 0.004 / n
    return shear, pull
