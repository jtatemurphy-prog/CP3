# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Rigid stud clutch with a structural clutch-load check (``LEGO_BENCH_CLUTCH=rigid``, opt-in, 2026-10-06).

Why. In the reference model every stud and pin connection is a PhysX fixed joint between two rigid bodies (maximal
coordinates, TGS, 32 position iterations). A long chain of such joints is solved iteratively, and the residual joint
error grows with the number of bodies in the load path and with the mass ratios along it (1-6 g bricks, 0.2 g pins,
a 100+ g battery box). The structure then sags by centimetres where real ABS sags by about a millimetre.
More iterations do not fix it (tested up to 255).

What this model does.

* **Merge.** Parts joined by studs, and rigid pinned laps (two or more pins, or a pin plus a locking axle; the same
  rule as :mod:`.laps`), become one rigid body per connected cluster: the parts are colliders of a representative
  member, with the cluster's true mass, centre of mass and inertia. Axles, gears, wheels, bushes and single-pin joints
  stay as they are (their own bodies on joints).
* **Clutch check (keeps it honest).** A merged connection can no longer break in PhysX, so the harness computes its
  load. Every 10 ms a linear structural model of the whole vehicle in its spawn configuration is loaded with
  * each part's weight and the contact forces on its colliders (PhysX contact report),
  * inertia relief: the net load accelerates the vehicle as a rigid body, so -m_i (a + alpha x r_i) per part
    (skipped when the chassis is held, ``LEGO_BENCH_HOLD``, where the holder takes the net load).

  Each part is a node (6 dof); each connection is a spring: a stud joint is one spring per engaged stud
  (pull-off :data:`K_STUD_AXIAL`, shear :data:`K_STUD_SHEAR`), a pin one spring at its hole, a cross fit stiff, a
  hinge / motor joint stiff except about its axle. The chassis node is the support. The solve gives every
  connection's force. As in the reference model (PhysX break force on the joint force magnitude), a merged stud
  connection fails at ``STUD_CLUTCH_N_PER_STUD`` x studs (10 N per stud, ``LEGO_STUD_SHEAR_N``) and a merged lap pin
  at ``PIN_BREAK_FORCE_N`` (50 N); loads are smoothed over 0.1 s and checked from 0.2 s after the start. The attempt
  then fails ``broke (clutch overload: ...)``.
* **Stiffness-aware deformation.** The same solve gives each part's elastic displacement relative to the chassis
  (joint compliance; the ABS bending of the bricks themselves is not included, it is smaller, see the study).
  The judge's 15 mm 'broke' limit is applied to it as well: ``broke (elastic deformation: ...)``.

Not modelled: the reaction torque of motors and the torques of PhysX gear joints / worm locks on their bearings
(mechanism directions of the structural model); the structural model keeps the spawn geometry, so a mechanism far
from its spawn pose is checked at its spawn pose.
"""

from __future__ import annotations

import os

import numpy as np

K_STUD_AXIAL = float(os.environ.get("LEGO_BENCH_K_STUD_AXIAL", "1.0e5"))
"""Stud stiffness along the stud axis (pull-off) [N/m per stud]: ~1-2 N pull-off reached at 10-20 um of micro-slip
(an estimate; env.STUD_AXIAL_STIFFNESS_N_M = 4e4 is the softer value used by ``--joints compliant``)."""
K_STUD_SHEAR = float(os.environ.get("LEGO_BENCH_K_STUD_SHEAR", "5.0e5"))
"""Stud stiffness across the stud axis [N/m per stud]: the stud bears on the tube / the 1.5 mm side wall (normal
contact, no slip); an estimate between the wall-bending (~2e5) and contact (~1e6+) values."""
K_PIN_RADIAL = 5.0e5
K_PIN_AXIAL = 5.0e4
K_AXLE = 1.0e6
"""Cross fits and axle bearings, radial [N/m]."""
K_FREE = 1.0e-2
"""Rotation about a hinge / motor / pivot axis [N m/rad]: a mechanism direction, kept non-singular."""
SMOOTH_S = 0.1
START_S = 0.2
ELASTIC_BREAK_MM = float(os.environ.get("LEGO_BENCH_CLUTCH_ELASTIC_MM", "15.0"))
NO_MERGE_KINDS = ("axle", "gear", "wheel", "bush", "shock", "differential")  # round 4b: a shock / diff case moves


def _uf(names):
    parent = {n: n for n in names}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    return parent, find


def rigid_clusters(design):
    """Returns (merged, clusters, laps): ``merged`` {member: body} for every non-representative member,
    ``clusters`` {body: [members incl. body]} (clusters of >= 2 parts), ``laps`` (from :func:`.laps.rigid_laps`)."""
    from .laps import rigid_laps  # noqa: PLC0415
    from .parts import bench_parts  # noqa: PLC0415

    cat = bench_parts()
    kinds = {p.name: cat[p.part].kind for p in design.parts}
    names = [p.name for p in design.parts]
    parent, find = _uf(names)
    for c in design.connections:
        if c.kind == "stud" and kinds.get(c.child) not in NO_MERGE_KINDS and kinds.get(c.parent) not in NO_MERGE_KINDS:
            parent[find(c.child)] = find(c.parent)
    _, laps = rigid_laps(design)
    for lap in laps:
        members = [n for n in (*lap["parts"], *lap["pins"]) if kinds.get(n) not in NO_MERGE_KINDS]
        for n in members[1:]:
            parent[find(n)] = find(members[0])
    groups: dict[str, list[str]] = {}
    for n in names:
        groups.setdefault(find(n), []).append(n)
    merged, clusters = {}, {}
    for members in groups.values():
        if len(members) < 2:
            continue
        if design.chassis in members:
            body = design.chassis
        else:
            body = max((m for m in members if kinds[m] != "pin"), key=lambda m: cat[design.part(m).part].mass_g,
                       default=members[0])  # fmt: skip
        clusters[body] = members
        for m in members:
            if m != body:
                merged[m] = body
    return merged, clusters, laps


# ------------------------------------------------------------------------------------------------ structural model
def _skew(r):
    return np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])


def _T(r):
    """6x6: generalized displacement (u, theta) of a node -> (displacement at a point offset r, rotation)."""
    T = np.eye(6)
    T[:3, 3:] = -_skew(r)
    return T


class Structure:
    """Linear spring model of the vehicle at a reference configuration.

    ``frames``: {part: (position [m] (3,), rotation matrix (3, 3))} of every part at the reference configuration
    (the spawn pose in the world). ``fixed``: the support node (the chassis)."""

    def __init__(self, design, frames: dict, fixed: str, monitored: set[str] | None = None, supports=()):
        import scipy.sparse as sp  # noqa: PLC0415
        from scipy.sparse.linalg import splu  # noqa: PLC0415

        from ai_pd_tasks.lego import car_design as cd  # noqa: PLC0415

        from .design import pivot_connections  # noqa: PLC0415
        from .parts import Box, bench_parts  # noqa: PLC0415

        cat = bench_parts()
        self.names = [p.name for p in design.parts]
        self.idx = {n: i for i, n in enumerate(self.names)}
        self.frames = frames
        self.fixed = fixed
        parts = {p.name: p for p in design.parts}
        self.mass = np.array([cat[parts[n].part].mass_g * 1e-3 for n in self.names])
        # centre of mass of each part (shape centre) at the reference configuration
        com = []
        for n in self.names:
            s = cat[parts[n].part].shape
            c_loc = (np.array(s.lo) + np.array(s.hi)) / 2 * 1e-3 if isinstance(s, Box) else np.array(s.centre) * 1e-3
            pos, R = frames[n]
            com.append(np.asarray(pos) + R @ c_loc)
        self.com = np.array(com)
        self.pos = np.array([frames[n][0] for n in self.names])
        pivots = pivot_connections(design)
        springs = []  # (conn index, node a, node b, point, R_local (3x3 columns = axes), k_lin (3,), k_rot (3,))
        self.conn_limit = {}  # conn index -> (label, force limit [N])
        for ci, c in enumerate(design.connections):
            a, b = c.parent, c.child
            if a not in self.idx or b not in self.idx:
                continue
            pa, Ra = frames[a]
            pb, Rb = frames[b]
            if c.kind == "stud":
                pts, Rz = self._stud_points(cat, parts[a], parts[b], frames)
                for p in pts:
                    springs.append((ci, a, b, p, Rz, (K_STUD_SHEAR, K_STUD_SHEAR, K_STUD_AXIAL),
                                    (K_STUD_AXIAL * 2.4e-3**2, K_STUD_AXIAL * 2.4e-3**2, K_STUD_SHEAR * 2.4e-3**2)))  # fmt: skip
                if monitored is None or (a in monitored and b in monitored):
                    self.conn_limit[ci] = (f"{b}->{a} ({c.studs or len(pts)} studs)",
                                           cd.STUD_CLUTCH_N_PER_STUD * max(c.studs or 1, 1))  # fmt: skip
            elif c.kind == "pin":
                ax = Rb[:, 0] if cat[parts[b].part].kind == "pin" else Ra[:, 0]
                t = float(np.dot(np.asarray(pa) - np.asarray(pb), ax))
                p = np.asarray(pb) + ax * np.sign(t) * min(abs(t), 4e-3)
                E = _frame_from_axis(ax)
                k_ax_rot = K_FREE if ci in pivots else K_PIN_RADIAL * 2.4e-3**2
                springs.append((ci, a, b, p, E, (K_PIN_AXIAL, K_PIN_RADIAL, K_PIN_RADIAL),
                                (k_ax_rot, K_PIN_RADIAL * 4e-3**2, K_PIN_RADIAL * 4e-3**2)))  # fmt: skip
                if monitored is None or (a in monitored and b in monitored):
                    from .env import PIN_BREAK_FORCE_N  # noqa: PLC0415

                    self.conn_limit[ci] = (f"pin {b}->{a}", PIN_BREAK_FORCE_N)
            else:  # cross (fixed), hinge / motor (free about the axle)
                axle = b if cat[parts[b].part].kind == "axle" else a
                pax, Rax = frames[axle]
                ax = Rax[:, 0] if cat[parts[axle].part].kind == "axle" else frames[b][1][:, 1]
                other = a if axle == b else b
                p = np.asarray(pax) + ax * float(np.dot(np.asarray(frames[other][0]) - np.asarray(pax), ax))
                if c.kind == "cross":
                    p = np.asarray(frames[b][0])
                E = _frame_from_axis(ax)
                k_ax_rot = K_AXLE * 4e-3**2 if c.kind == "cross" else K_FREE
                springs.append((ci, a, b, p, E, (K_AXLE * 0.1, K_AXLE, K_AXLE),
                                (k_ax_rot, K_AXLE * 4e-3**2, K_AXLE * 4e-3**2)))  # fmt: skip
        if getattr(design, "racks", None):  # rack slides: the mesh and the guides (lego_bench.racks), not monitored
            from .racks import structure_springs  # noqa: PLC0415

            springs += [sp for sp in structure_springs(design, frames, K_AXLE, K_FREE, K_PIN_RADIAL * 4e-3**2, K_FREE)
                        if sp[1] in self.idx and sp[2] in self.idx]
        self.springs = springs
        n = len(self.names)
        rows, cols, vals = [], [], []
        # relative generalized displacement of each spring: D q (6 per spring); K = D^T k D
        drows, dcols, dvals = [], [], []
        kdiag = []
        for si, (ci, a, b, p, E, kl, kr) in enumerate(springs):
            ia, ib = self.idx[a], self.idx[b]
            Ta, Tb = _T(p - self.pos[ia]), _T(p - self.pos[ib])
            Eb = np.zeros((6, 6))
            Eb[:3, :3] = E.T
            Eb[3:, 3:] = E.T
            Ba, Bb = -Eb @ Ta, Eb @ Tb
            for blk, node in ((Ba, ia), (Bb, ib)):
                for r in range(6):
                    for col in range(6):
                        if blk[r, col] != 0.0:
                            drows.append(6 * si + r)
                            dcols.append(6 * node + col)
                            dvals.append(blk[r, col])
            kdiag.extend(kl)
            kdiag.extend(kr)
        D = sp.csr_matrix((dvals, (drows, dcols)), shape=(6 * len(springs), 6 * n))
        self.D = D
        self.kdiag = np.array(kdiag)
        K = (D.T @ sp.diags(self.kdiag) @ D).tocsc()
        free = np.ones(6 * n, dtype=bool)
        for nd in {fixed, *supports}:  # the chassis, plus held parts (a clamp, LEGO_BENCH_HOLD_PARTS)
            free[6 * self.idx[nd] : 6 * self.idx[nd] + 6] = False
        self.free = np.nonzero(free)[0]
        Kf = K[self.free][:, self.free]
        # a tiny mass-proportional ground spring keeps parts that hang on mechanism directions only non-singular
        reg = np.repeat(np.maximum(self.mass, 1e-4), 6)[self.free] * 1e-3
        self.lu = splu((Kf + sp.diags(reg)).tocsc())
        # spring -> connection summation (linear force components only)
        conn_ids = sorted(self.conn_limit)
        self.conn_ids = conn_ids
        pos_of = {ci: k for k, ci in enumerate(conn_ids)}
        mr = [(pos_of[sprg[0]], si) for si, sprg in enumerate(springs) if sprg[0] in pos_of]
        self.Msum = sp.csr_matrix((np.ones(len(mr)), ([r for r, _ in mr], [c for _, c in mr])),
                                  shape=(len(conn_ids), len(springs)))  # fmt: skip
        self._spring_E = np.array([s[4] for s in springs]) if springs else np.zeros((0, 3, 3))
        self.limits = np.array([self.conn_limit[ci][1] for ci in conn_ids])

    @staticmethod
    def _stud_points(cat, pa, pb, frames):
        """World points [m] of the engaged studs between two parts (whichever is on top), and the lower part's
        rotation (stud axis = its local z)."""
        out = []
        best = None
        for up, lo in ((pb, pa), (pa, pb)):
            bu, bl = cat[up.part], cat[lo.part]
            if not bu.studs_bottom or not bl.studs_top:
                continue
            pu, Ru = frames[up.name]
            pl, Rl = frames[lo.name]
            a = np.array([np.asarray(pu) + Ru @ (np.array([x, y, bu.bottom_z]) * 1e-3) for x, y in bu.studs_bottom])
            b = np.array([np.asarray(pl) + Rl @ (np.array([x, y, bl.top_z]) * 1e-3) for x, y in bl.studs_top])
            d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
            gap = float(np.abs(d.min(axis=1)).min())
            pts = a[d.min(axis=1) < 0.6e-3]
            if best is None or len(pts) > len(best[0]) or (len(pts) == len(best[0]) and gap < best[2]):
                best = (pts, Rl, gap)
        if best is None or len(best[0]) == 0:  # fall back: one stud at the midpoint
            p = (np.asarray(frames[pa.name][0]) + np.asarray(frames[pb.name][0])) / 2
            return [p], frames[pa.name][1]
        out = list(best[0])
        return out, best[1]

    def solve(self, f: np.ndarray):
        """``f``: (n, 6) node loads (force, moment about the node origin) at the reference configuration.
        Returns (u (n, 6), connection forces (m,) [N])."""
        q = np.zeros(6 * len(self.names))
        q[self.free] = self.lu.solve(f.reshape(-1)[self.free])
        d = self.D @ q  # (6 S)
        S = len(self.springs)
        fl = (self.kdiag * d).reshape(S, 6)[:, :3]  # spring force in its local axes
        fw = np.einsum("sij,sj->si", self._spring_E, fl)  # in the reference frame
        F = self.Msum @ fw
        return q.reshape(-1, 6), np.linalg.norm(F, axis=1) if len(self.conn_ids) else np.zeros(0)

    def gravity(self, g_vec) -> np.ndarray:
        f = np.zeros((len(self.names), 6))
        F = self.mass[:, None] * np.asarray(g_vec)[None, :]
        f[:, :3] = F
        f[:, 3:] = np.cross(self.com - self.pos, F)
        return f

    def relieve(self, f: np.ndarray) -> np.ndarray:
        """Inertia relief: subtract the rigid-body inertia loads that balance the net load."""
        m = self.mass
        M_tot = m.sum()
        c = (m[:, None] * self.com).sum(axis=0) / M_tot
        r = self.com - c
        F = f[:, :3].sum(axis=0)
        Mo = (f[:, 3:] + np.cross(self.pos - c, f[:, :3])).sum(axis=0)
        J = sum(mi * (np.dot(ri, ri) * np.eye(3) - np.outer(ri, ri)) for mi, ri in zip(m, r, strict=True))
        J = J + np.eye(3) * 1e-9
        a = F / M_tot
        alpha = np.linalg.solve(J, Mo)
        acc = a[None, :] + np.cross(alpha[None, :], r)
        Fi = -m[:, None] * acc
        out = f.copy()
        out[:, :3] += Fi
        out[:, 3:] += np.cross(self.com - self.pos, Fi)
        return out

    def label(self, k: int) -> str:
        return self.conn_limit[self.conn_ids[k]][0]


def _frame_from_axis(ax) -> np.ndarray:
    """Rotation whose first column is ``ax``."""
    ax = np.asarray(ax, dtype=float)
    ax = ax / np.linalg.norm(ax)
    t = np.array([0.0, 0.0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    y = np.cross(t, ax)
    y /= np.linalg.norm(y)
    z = np.cross(ax, y)
    return np.stack([ax, y, z], axis=1)


def design_frames(design) -> dict:
    """{part: (position [m], rotation)} at the design pose (design frame)."""
    from .design import place  # noqa: PLC0415

    P = place(design)
    return {n: (pl.p * 1e-3, pl.R) for n, pl in P.items()}


def static_check(design, held: list[str], g=(0.0, 0.0, -9.81)) -> dict:
    """Offline: the design at its design pose under its own weight, ``held`` parts (and the chassis) clamped
    (stiff ground springs). Returns the worst clutch load ratio and the largest elastic displacement."""
    frames = design_frames(design)
    merged, clusters, _ = rigid_clusters(design)
    members = set(merged) | set(clusters)
    st = Structure(design, frames, design.chassis, monitored=members, supports=held)
    f = st.gravity(g)
    u, F = st.solve(f)
    ratio = F / st.limits if len(F) else np.zeros(0)
    k = int(np.argmax(ratio)) if len(ratio) else None
    disp = np.linalg.norm(u[:, :3], axis=1) * 1e3
    j = int(np.argmax(disp))
    return {
        "max_ratio": float(ratio[k]) if k is not None else 0.0,
        "worst": st.label(k) if k is not None else None,
        "worst_force_n": float(F[k]) if k is not None else 0.0,
        "max_disp_mm": float(disp[j]),
        "max_disp_part": st.names[j],
        "disp_z_mm": {n: float(u[i, 2] * 1e3) for i, n in enumerate(st.names)},
        "n_clusters": len(clusters),
        "n_merged": len(merged),
    }
