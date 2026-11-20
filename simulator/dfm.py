"""Design-quality inspection (not design rules): measures what a design leaves unconstrained, the way a design
review or a DFM report would, and reports the consequence of each finding.

First check: axial retention of everything that turns. A rotating cluster (an axle with its gears, wheels and bushes,
or a pin) is free to rotate in its holes by design; along its axis it is only held where one of its members bears on a
face of the structure. For each cluster: the free travel along its axis in each direction (until a member touches a
part outside the cluster), and what that travel allows:
  * the axle (or pin) slides out of one of its holes      -> finding "falls out"
  * a gear of the cluster slides out of mesh              -> finding "loses mesh"
  * a wheel's tyre runs into the structure               (caught as blocking contact)
  * otherwise, axial play larger than PLAY_OK_MM           -> finding "play"
Geometry: the catalog's part shapes as oriented boxes, shrunk 0.2 mm; parts that overlap where they sit (an axle through
a brick's hole, a gear meshing) do not block each other.

    python dfm.py <design.json> [...]          python dfm.py --e0     (every E0 design + CAL1, summary table)
"""
from __future__ import annotations

import glob
import json
import os
import sys

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "src"))
from ai_pd_tasks.lego_bench.design import Design, place  # noqa: E402
from ai_pd_tasks.lego_bench.parts import Box, Boxes, Cyl, Segs  # noqa: E402

SHRINK = 0.2
STEP = 0.25
MAX_T = 60.0  # (an axle free to slide this far has left its holes)
PLAY_OK_MM = 1.0
HOLE_HALF = 4.0  # a Technic hole is 8 mm long


def obbs(pl):
    """oriented boxes (centre, axes (3x3, columns), half extents) of a placed part, design frame, mm"""
    s = pl.bp.shape
    out = []
    boxes = []
    if isinstance(s, Boxes) and s.boxes:
        boxes = s.boxes
    elif isinstance(s, Segs) and s.segs:
        for x0, z0, x1, z1 in s.segs:
            a, b = np.array([x0, 0, z0]), np.array([x1, 0, z1])
            d = b - a
            L = float(np.linalg.norm(d)) or 1e-6
            u = d / L
            w = np.cross(u, [0, 1.0, 0])
            ax = np.column_stack([u, [0, 1.0, 0], w])
            out.append(((a + b) / 2, ax, np.array([L / 2 + s.radius, s.half_w, s.radius])))
    elif isinstance(s, Box):
        boxes = ((s.lo, s.hi),)
    for lo, hi in boxes:
        lo, hi = np.array(lo, float), np.array(hi, float)
        out.append(((lo + hi) / 2, np.eye(3), (hi - lo) / 2))
    if isinstance(s, Cyl):
        h = np.full(3, s.radius)
        h["xyz".index(s.axis)] = s.half_len
        out.append((np.array(s.centre, float), np.eye(3), h))
    res = []
    for c, A, h in out:
        res.append((pl.p + pl.R @ c, pl.R @ A, np.maximum(h - SHRINK, 0.05)))
    return res


def overlap(a, b):
    """separating-axis test for two oriented boxes"""
    ca, Aa, ha = a
    cb, Ab, hb = b
    t = cb - ca
    axes = [Aa[:, i] for i in range(3)] + [Ab[:, i] for i in range(3)]
    for i in range(3):
        for j in range(3):
            v = np.cross(Aa[:, i], Ab[:, j])
            n = np.linalg.norm(v)
            if n > 1e-6:
                axes.append(v / n)
    for L in axes:
        ra = sum(ha[k] * abs(float(np.dot(Aa[:, k], L))) for k in range(3))
        rb = sum(hb[k] * abs(float(np.dot(Ab[:, k], L))) for k in range(3))
        if abs(float(np.dot(t, L))) > ra + rb:
            return False
    return True


def inspect(path: str) -> dict:
    d = Design.from_dict(json.load(open(path)))
    P = place(d)
    names = list(P)
    B = {n: obbs(P[n]) for n in names}
    # rotating clusters: cross fits
    par = {n: n for n in names}

    def find(x):
        while par[x] != x:
            par[x] = par[par[x]]
            x = par[x]
        return x
    for c in d.connections:
        if c.kind == "cross" and c.parent in par and c.child in par:
            par[find(c.child)] = find(c.parent)
    clusters: dict[str, list[str]] = {}
    for n in names:
        clusters.setdefault(find(n), []).append(n)
    kind = {n: P[n].bp.kind for n in names}
    # pairs that overlap where they sit do not block each other (through holes, meshes, hubs)
    def parts_overlap(a, b):
        return any(overlap(x, y) for x in B[a] for y in B[b])
    findings, report = [], []
    for root, members in clusters.items():
        shafts = [m for m in members if kind[m] in ("axle", "pin")]
        if not shafts:
            continue
        sh = shafts[0]
        bearings = [(c, c.parent if c.child == sh else c.child) for c in d.connections
                    if c.kind in ("hinge", "motor", "pin") and sh in (c.parent, c.child)]
        bearings = [(c, o) for c, o in bearings if o not in members]
        if not bearings:
            continue
        ax = P[sh].R[:, 0]
        others = [n for n in names if n not in members]
        cand, skip = [], set()
        for o in others:
            # only parts near the cluster can block it
            if min(np.linalg.norm(P[o].p - P[m].p) for m in members) > 120:
                continue
            cand.append(o)  # only the member that overlaps o (the axle in its own hole) ignores it; a bush or gear on the
            skip |= {(m, o) for m in members if parts_overlap(m, o)}  # same axle still stops against that part
        free = {}
        for sgn in (1.0, -1.0):
            t = 0.0
            hit = None
            while t < MAX_T and hit is None:
                t += STEP
                off = sgn * t * ax
                for m in members:
                    for bm in B[m]:
                        moved = (bm[0] + off, bm[1], bm[2])
                        for o in cand:
                            if (m, o) in skip:
                                continue
                            if any(overlap(moved, bo) for bo in B[o]):
                                hit = (m, o)
                                break
                        if hit:
                            break
                    if hit:
                        break
            # a motor output is a blind socket: the shaft cannot go further into the motor
            for c, o in bearings:
                if c.kind == "motor":
                    into = float(np.dot(P[o].R[:, 1], ax))  # motor output faces -its local y? (socket depth)
                    if (into > 0) == (sgn > 0):
                        t, hit = min(t, 0.0), (sh, o + " (socket bottom)")
            free[sgn] = (round(max(t - STEP, 0.0) if hit else MAX_T, 2), hit)
        # consequences
        s_ax = float(np.dot(P[sh].p, ax))
        half = P[sh].bp.shape.half_len
        lo_end, hi_end = s_ax - half, s_ax + half
        issues = []
        for sgn, (tf, hit) in free.items():
            for c, o in bearings:
                if c.kind == "motor":
                    continue
                s_h = float(np.dot(P[o].p, ax))
                # hole centre along the axis: the bearing part's position projected (its hole is 8 mm long)
                t_exit = (s_h + HOLE_HALF - lo_end) if sgn > 0 else (hi_end - (s_h - HOLE_HALF))
                if tf >= t_exit and kind[sh] == "pin":  # a friction pin is held in its holes by its ridges
                    issues.append(("friction only", f"{sh} is held in {o} by its friction ridges alone (nothing stops it "
                                   f"{'+' if sgn > 0 else '-'}axis)"))
                elif tf >= t_exit:
                    issues.append(("falls out", f"{sh} can slide {tf:.1f} mm {'+' if sgn > 0 else '-'}axis and leave its "
                                   f"hole in {o} (out at {t_exit:.1f} mm)"))
            for gm in d.gear_meshes:
                g = gm.gear_a if gm.gear_a in members else (gm.gear_b if gm.gear_b in members else None)
                if g is None:
                    continue
                h = gm.gear_b if g == gm.gear_a else gm.gear_a
                wg, wh = (P[g].bp.width or 8.0), (P[h].bp.width or 8.0)
                off = float(np.dot(P[h].p - P[g].p, ax)) * sgn
                ov = (wg + wh) / 2 - abs(off)
                lose = ov - 1.0 if off <= 0 else (wg + wh) / 2 + off - 1.0
                if tf >= max(lose, 0.0):
                    issues.append(("loses mesh", f"{g} can slide {tf:.1f} mm {'+' if sgn > 0 else '-'}axis, out of mesh "
                                   f"with {h} (at {max(lose, 0):.1f} mm)"))
        play = free[1.0][0] + free[-1.0][0]
        if not issues and play > PLAY_OK_MM:
            issues.append(("play", f"{sh} cluster has {play:.1f} mm of axial play"))
        report.append(dict(cluster=members, shaft=sh, free_plus=free[1.0][0], free_minus=free[-1.0][0],
                           stop_plus=free[1.0][1], stop_minus=free[-1.0][1], issues=issues))
        findings += issues
    return dict(design=d.name, path=path, clusters=report, findings=findings,
                counts={k: sum(1 for f in findings if f[0] == k) for k in ("falls out", "loses mesh", "play", "friction only")})


if __name__ == "__main__":
    if "--e0" in sys.argv:
        files = sorted(glob.glob(os.path.join(ROOT, "bench_eval/runs/e0/*__*/*_L*/submission.json")))
        files = [f for f in files if "antigravity" not in f and "/all_L" not in f]
        files += sorted(glob.glob(os.path.join(ROOT, "bench_eval/runs/calib1/reference__crossbot/*/submission.json")))
        out = {}
        for f in files:
            try:
                r = inspect(f)
            except Exception as e:  # noqa: BLE001
                r = {"error": repr(e)}
            out[f.split("runs/")[1]] = r
            print(f.split("runs/")[1], r.get("counts", r.get("error")), flush=True)
        json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "dfm_axial.json"), "w"),
                  indent=1, default=str)
    else:
        for f in sys.argv[1:]:
            r = inspect(f)
            print(r["design"], r["counts"])
            for c in r["clusters"]:
                print(f"  {c['shaft']:12s} free +{c['free_plus']} / -{c['free_minus']} mm  stops {c['stop_plus']} / {c['stop_minus']}")
                for k, msg in c["issues"]:
                    print(f"     [{k}] {msg}")
