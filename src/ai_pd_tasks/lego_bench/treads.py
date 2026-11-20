# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Technic tread (3873 links on 57520 sprockets): a reduced, physically honest model (round 2, 2026-10-06).

The real part
-------------
* 3873 "Technic chain tread 2.5 wide" (LDraw): a 3711 chain link with a pad. Pitch 6.4 mm (hinge to hinge), 20.8 mm
  wide, 0.19 g, hard ABS. Its body is +-2 mm about the hinge line; on the outer face a grouser bar 3.2 mm long (along
  the tread), 1.2 mm tall, across the full width, once per link. The links fit the standard Technic gear pitch (two
  teeth per link).
* 57520 "Technic sprocket wheel 25.4": 6 lobes (r 12.8 mm), 8 mm hub with a cross hole. 12 links wrap one turn
  (two per lobe), so the hinge (pitch) line runs on a 12-gon of side 6.4: pitch radius 6.4 / (2 sin 15 deg) = 12.36 mm.
  Grouser tips at 12.36 + 2.0 + 1.2 = 15.56 mm from the axle.
* A loop round two sprockets at centre distance D needs N links with N * 6.4 >= 2 D + 2 pi 12.36 (12 links per
  sprocket-turn of wrap). The user owns 50 links (two loops of 25: D <= 41.6 mm, e.g. 40 mm = 5 holes, 2.3 mm slack).

Why a reduced model
-------------------
A literal tread is 20-25 jointed bodies of 0.19 g per side against a 300-500 g vehicle (mass ratio > 1000), closed in
a loop. PhysX maximal-coordinate joints do not converge on that (the same failure as the 3711 chain); a closed loop
also cannot be one articulation. So the tread is simulated as its *contact envelope*:

* **Rollers.** n + 1 coaxial-parallel rollers (``ROLLER_SPACING_MM`` apart, n = ceil(D / 6.4)) along the line
  between the two sprocket axles, each a rigid body on a revolute joint carried by the part, rigidly joined to the
  first sprocket axle's bearing, that this axle passes through nearest the tread (``carrier``). Their union is the tread's stadium-shaped envelope: radius = grouser-tip radius 15.56 mm, width
  20.8 mm, centred on the sprocket plane. Between two rollers the envelope dips 0.4 mm (a flat tread does not).
* **Belt kinematics.** The first roller is coupled to the first sprocket's shaft by a PhysX gear joint, ratio
  r_pitch / r_tip = 0.794, each next roller 1:1 to the previous one, so the envelope surface moves at the belt speed
  v = omega_sprocket * 12.36 mm (not the grouser-tip radius). The second sprocket's shaft is coupled 1:1 to the first (the belt turns both). A tread is
  driven when either sprocket's shaft is driven (motor, gears).
* **Grousers.** The two end rollers (the wraps round the sprockets) carry 15 grouser bars (3.2 x 1.2 mm, full width)
  on a 14.36 mm core: with the roller turning at 0.794 of the sprocket, 15 grousers pass the contact per 12 links,
  the real rate. The middle rollers (the straight runs) are smooth cylinders at the grouser-tip radius: the run rests
  on its grouser tips as the real one, but its grousers cannot hook an edge (the real ones can): conservative.
* **Material.** ABS (static 0.40, dynamic 0.35), the same material as every other hard LEGO part: no rubber.
* **Mass.** The links' mass (N x 0.19 g) is shared by the rollers; the sprockets keep their own.
* **Collisions.** Rollers do not collide with each other, with the two sprockets, with the parts on the two sprocket
  shafts, or with the rigid cluster that carries them (their union is a tread that wraps round those parts; the
  validator checks that no other part sits inside the tread's shell, r 10.36..15.56 mm, +-10.4 mm wide). Every other
  part and every course shape collides with them.

Limits (the model is never stronger or grippier than the real tread)
--------------------------------------------------------------------
* Rigid envelope: the real tread is a slack chain that can sag over an edge and wrap a little round it. The model
  is a rigid stadium (no sag, no conformity): it bridges an edge without the extra wrap contact.
* Straight-run grousers are smooth in the model (no edge hooking by the bottom run); the wrap grousers are modelled.
* No tooth skipping and no link pull-apart: the gear-joint coupling transmits any torque. On flat ground and steps the
  belt tension is at most the traction mu * N (a 400 g car: < 2 N), far below an estimated skip force of ~20 N
  (``TREAD_SKIP_FORCE_N``, an estimate, not a measurement); the validator warns when the drivetrain's stall force at the
  tread exceeds it (a jammed tread would really skip: the simulation would hold).
* **Transmission creep (weaker than real).** The chain of gear joints is solved iteratively and gives under load:
  with the friction raised to mu = 2 (diagnostic) the tread car climbed the 30 deg ramp at ~0.09-0.11 m/s instead of
  0.17 (about 40 % creep at ~0.9 N of traction); on the flat it runs at 0.167 m/s (predicted 0.173). The roller solver
  iterations are raised (``TREAD_POS_ITERS`` 128 / ``TREAD_VEL_ITERS`` 16, the whole vehicle island) to keep this down.
* The gear joints couple velocities about the hinge axes in the world frame, like the other gear joints of the bench.
* Visual: static boxes along the loop (they do not animate); the contact is the rollers'.
* Only two equal sprockets per loop (no third sprocket, no road wheels).

Chain track (round 4b follow-up): ``"link": "3711"`` on two equal spur gears (24T 3648: 12 links a turn, pitch
r 12.36, envelope 14.36; 40T 3649: 20, r 20.46, 22.46; 16T 94925 / 4019: 8, 8.36, 10.36), 7.8 mm wide, smooth hard ABS,
the same roller model. Use 24T or 40T (a 16T envelope is only 20.7 mm across). The coupling-only ``chains`` entry
(``inventory_r4b``) stays for chains that do not touch the ground.

Wide tread (round 4b, 2026-10-14): 57518 links on 57519 sprockets
-------------------------------------------------------------------
* 57518 "Technic link tread wide with 2 pin holes" (LDraw): pitch 12 mm (hinge pins at y = 0 and -12), 38.4 mm wide,
  8.4 mm thick: the body runs 2.8 mm inside the hinge line (towards the sprocket) and 5.6 mm outside it (the flat
  pad, with two pin holes); 1.4 g, hard ABS, no grouser bars.
* 57519 sprocket: 10 lobes, one link per lobe: pitch radius 12 / (2 sin 18 deg) = 19.42 mm (pitch diameter 38.8; the
  hinge pins r 1.6 sit in its valleys at r 17.8). Envelope: r 19.42 + 5.6 = 25.0 mm (LDraw, the link's outer face; not
  pitch radius + full thickness, 27.8: 2.8 mm of the link is inside the pitch line), inner face r 16.6.
* Loop: N x 12 >= 2 D + pi x 38.84 with at most one link of slack. The 52 owned links make two loops of 25 at
  D = 88 mm (11 studs; 2.0 mm slack) or of 26 at D = 92 mm (6.0 mm slack).
* The same reduced model (rollers of r 25.0, 38.4 mm wide, the links' mass on them, gear-joint belt kinematics at
  19.42 / 25.0); smooth envelope (no grousers). The 3873 / 57520 model is unchanged.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np

TREAD_LINKS: dict[str, dict] = {
    "3873": dict(title="Technic link tread 2.5 wide (3873)", pitch=6.4, mass_g=0.19, width=20.8, inner=2.0, pad=2.0,
                 grouser_h=1.2, grouser_w=3.2, tyre_mu=0.23),
    # round 4b: the wide tread (LDraw 57518: hinges 12 mm apart, z -2.8..5.6 about the hinge line, 38.4 wide; 1.4 g)
    "57518": dict(title="Technic link tread wide with 2 pin holes (57518)", pitch=12.0, mass_g=1.4, width=38.4,
                  inner=2.8, pad=5.6, grouser_h=0.0, grouser_w=0.0, tyre_mu=0.25),
    # round 4b follow-up: the 3711 chain as a track on spur gears (LDraw 3711: 6.4 pitch, +-2 mm about the pitch
    # line, 7.8 mm wide; 0.09 g)
    "3711": dict(title="Technic link chain (3711) as a track", pitch=6.4, mass_g=0.09, width=7.8, inner=2.0, pad=2.0,
                 grouser_h=0.0, grouser_w=0.0, tyre_mu=0.23),
}
"""Tread links: pitch (hinge to hinge), width, the body's extent inside / outside the hinge line (inner / pad), the
grouser bar (height above the pad, length along the tread) [mm]; mass [g] (BrickLink); tyre_mu: grip as the tyres'
(Brick Experiment Channel "33 Lego Wheels/Tracks on Wood": tan of the steepest climb on pine; 3873 13 deg, 57518 14 deg;
the 3711 chain is not measured: hard ABS like the 3873)."""

SPROCKETS: dict[str, int] = {"57520": 12}
"""Tread sprockets -> tread links per turn (pitch circle) of the 3873 tread."""
LINK_SPROCKETS: dict[str, dict[str, int]] = {"3873": {"57520": 12}, "57518": {"57519": 10},
                                             "3711": {"3648": 12, "3649": 20, "94925": 8, "4019": 8}}
"""Tread link -> {its sprocket: links per turn} (round 4b: the wide 57518 on the 10-lobe 57519)."""

MAX_SLACK_PITCH = 1.0
"""Most slack a tread loop may have, in links (more and the real tread sags onto the ground or jumps the lobes)."""
ROLLER_SPACING_MM = float(os.environ.get("LEGO_BENCH_TREAD_SPACING_MM", "6.4"))
END_GROUSERS = 15
TREAD_SKIP_FORCE_N = 20.0
"""Estimated belt tension at which a 3873 tread jumps a 57520 sprocket (an estimate: no published test). Only used for
a validator warning; the simulation does not model skipping."""
TREAD_POS_ITERS = int(os.environ.get("LEGO_BENCH_TREAD_POS_ITERS", "128"))
TREAD_VEL_ITERS = int(os.environ.get("LEGO_BENCH_TREAD_VEL_ITERS", "16"))
"""Solver iterations of the tread rollers (PhysX uses the largest count of an island, so a tread vehicle is solved
with these). The chain of gear joints from the sprocket to the rollers converges slowly: with the bench's 32 position
iterations the tread crept (slipped against its sprocket) under a climbing load; 128 holds it (smoke runs)."""
ABS_MATERIAL = (0.40, 0.35)  # design.ABS_FRICTION (repeated: no import cycle)
ROLLER_CARRIER_RATIO = float(os.environ.get("LEGO_BENCH_ROLLER_RATIO", "1.0"))
"""Most link mass the rollers of one 57518 / 3711 tread carry, relative to their carrier part's mass (the rest goes on
the carrier). At 2 the wide-tread tank's rails exploded at spawn (35 g of rollers on a 5.9 g rail); 1 is stable. The
3873 tread keeps its round-2 model (its cars are at ~1.7)."""


@dataclass
class DesignTread:
    """A tread loop in a design: ``links`` tread links (``link`` part id) round two sprockets (part names)."""

    name: str
    sprockets: tuple[str, str]
    links: int
    link: str = "3873"

    def to_dict(self) -> dict:
        d = {"name": self.name, "sprockets": list(self.sprockets), "links": int(self.links)}
        if self.link != "3873":
            d["link"] = self.link
        return d

    @classmethod
    def from_dict(cls, d: dict) -> DesignTread:
        unknown = set(d) - {"name", "sprockets", "links", "link"}
        if unknown:
            raise ValueError(f"unknown tread fields: {sorted(unknown)}")
        sp = tuple(str(s) for s in d["sprockets"])
        return cls(str(d["name"]), sp, int(d["links"]), str(d.get("link", "3873")))


def pitch_radius(sprocket: str, link: str = "3873") -> float:
    return TREAD_LINKS[link]["pitch"] / (2.0 * math.sin(math.pi / LINK_SPROCKETS[link][sprocket]))


def tip_radius(sprocket: str, link: str = "3873") -> float:
    L = TREAD_LINKS[link]
    return pitch_radius(sprocket, link) + L["pad"] + L["grouser_h"]


def inner_radius(sprocket: str, link: str = "3873") -> float:
    return pitch_radius(sprocket, link) - TREAD_LINKS[link]["inner"]


@dataclass
class TreadGeom:
    """A resolved tread loop (design frame, mm)."""

    tread: DesignTread
    a: object  # design.Placed (first sprocket)
    b: object
    axis: np.ndarray
    ca: np.ndarray
    cb: np.ndarray
    e: np.ndarray  # unit vector a -> b in the sprocket plane
    n: np.ndarray  # axis x e
    D: float
    r_pitch: float
    r_tip: float
    r_in: float
    half_w: float
    loop_mm: float
    slack_mm: float
    axle_a: str = ""
    axle_b: str = ""
    bearing_a: object = None  # DesignConnection
    bearing_b: object = None
    errors: list[str] = field(default_factory=list)

    def rollers(self) -> list[np.ndarray]:
        """Roller centres along a -> b (both ends included)."""
        k = max(1, math.ceil(self.D / ROLLER_SPACING_MM - 1e-9))
        return [self.ca + self.e * self.D * i / k for i in range(k + 1)]

    def loop(self, r: float, step: float = 1.5) -> list[tuple[np.ndarray, np.ndarray]]:
        """Points (and unit tangents) round the stadium at distance r from the line a-b, in the sprocket plane."""
        out = []
        m = max(2, int(self.D / step) + 1)
        for sgn in (1.0, -1.0):  # straight runs (+n: top when the axis is +y, -n: bottom)
            for t in np.linspace(0.0, self.D, m):
                out.append((self.ca + self.e * t + sgn * r * self.n, sgn * self.e))
        k = max(4, int(math.pi * r / step) + 1)
        for c, s0 in ((self.cb, 1.0), (self.ca, -1.0)):  # half circles round b (on +e) and a (on -e)
            for ang in np.linspace(0.0, math.pi, k):
                rad = s0 * (math.sin(ang) * self.e + math.cos(ang) * self.n)
                tan = np.cross(self.axis, rad)
                out.append((c + r * rad, tan / max(np.linalg.norm(tan), 1e-12)))
        return out

    def shell_points(self, step: float = 1.5) -> np.ndarray:
        """Sample points filling the tread's shell (r_in..r_tip from the a-b line, +-half_w across)."""
        pts = []
        for r in np.linspace(self.r_in, self.r_tip, 4):
            for p, _ in self.loop(r, step):
                for u in np.linspace(-self.half_w, self.half_w, 6):
                    pts.append(p + u * self.axis)
        return np.array(pts)

    def bbox(self) -> tuple[np.ndarray, np.ndarray]:
        pts = np.array([p for p, _ in self.loop(self.r_tip, 4.0)])
        pts = np.concatenate([pts + self.half_w * self.axis, pts - self.half_w * self.axis])
        return pts.min(axis=0), pts.max(axis=0)


def tread_geoms(design, P: dict | None = None) -> list[TreadGeom]:
    """Resolve every tread of ``design`` (errors of each in ``TreadGeom.errors``; unresolvable treads are skipped
    with their errors reported by :func:`check_treads`)."""
    return [g for g in (_resolve(design, t, P) for t in getattr(design, "treads", []) or []) if isinstance(g, TreadGeom)]


def _resolve(design, t: DesignTread, P=None):
    from .design import place  # noqa: PLC0415

    P = P if P is not None else place(design)
    errs = []
    if t.link not in TREAD_LINKS:
        return [f"tread {t.name}: '{t.link}' is not a tread link ({sorted(TREAD_LINKS)})"]
    if len(t.sprockets) != 2 or t.sprockets[0] == t.sprockets[1]:
        return [f"tread {t.name}: needs two different sprockets"]
    for s in t.sprockets:
        if s not in P:
            return [f"tread {t.name}: unknown part '{s}'"]
        if P[s].inst.part not in LINK_SPROCKETS[t.link]:
            if t.link == "3873":
                return [f"tread {t.name}: '{s}' ({P[s].inst.part}) is not a tread sprocket ({sorted(SPROCKETS)})"]
            return [f"tread {t.name}: '{s}' ({P[s].inst.part}) is not a sprocket of the {t.link} tread "
                    f"({sorted(LINK_SPROCKETS[t.link])})"]
    a, b = P[t.sprockets[0]], P[t.sprockets[1]]
    if a.inst.part != b.inst.part:
        return [f"tread {t.name}: both sprockets must be the same part (got {a.inst.part} / {b.inst.part})"]
    ax = a.axis() / np.linalg.norm(a.axis())
    if abs(float(np.dot(ax, b.axis()))) < 0.999:
        return [f"tread {t.name}: the sprocket axes are not parallel"]
    d = b.p - a.p
    axial = float(np.dot(d, ax))
    if abs(axial) > 1.0:
        errs.append(f"tread {t.name}: the sprockets are {abs(axial):.2f} mm apart along their axles (not in one plane)")
    dp = d - axial * ax
    D = float(np.linalg.norm(dp))
    if D < 1e-6:
        return [f"tread {t.name}: the sprockets are coaxial"]
    e = dp / D
    L = TREAD_LINKS[t.link]
    rp, rt, ri = pitch_radius(a.inst.part, t.link), tip_radius(a.inst.part, t.link), inner_radius(a.inst.part, t.link)
    loop = 2 * D + 2 * math.pi * rp
    g = TreadGeom(t, a, b, ax, a.p.copy(), a.p + dp, e, np.cross(ax, e), D, rp, rt, ri, 0.5 * L["width"], loop,
                  t.links * L["pitch"] - loop, errors=errs)
    for side, s in (("a", t.sprockets[0]), ("b", t.sprockets[1])):
        fit = [c for c in design.connections if c.child == s and c.kind == "cross"]
        if len(fit) != 1:
            continue  # reported by the generic one-cross-fit check
        axle = fit[0].parent
        brg = next((c for c in design.connections if c.child == axle and c.kind in ("hinge", "motor")), None)
        setattr(g, f"axle_{side}", axle)
        setattr(g, f"bearing_{side}", brg)
    return g


def carrier(g: TreadGeom, design=None, P=None) -> str:
    """The part that carries the rollers: of the parts rigidly joined to the first sprocket axle's bearing that this
    axle passes through (its bearing brick, the other rails it crosses), the one nearest the tread along the axle,
    so the tread's load enters the structure where the real axle carries it (the first smoke run hung the right
    tread on the left rail: the whole car's load twisted that one rail off its studs). Without a design: the bearing."""
    if g.bearing_a is None:
        return ""
    if design is None or P is None or os.environ.get("LEGO_BENCH_TREAD_CARRIER") == "bearing":  # (diagnostic)
        return g.bearing_a.parent
    from .design import _components, axle_through_holes, rigid_edges  # noqa: PLC0415

    rigid = _components([p.name for p in design.parts], rigid_edges(design))
    comp = next((c for c in rigid if g.bearing_a.parent in c), {g.bearing_a.parent})
    ax = P.get(g.axle_a)
    cands = [n for n in comp if n in P and ax is not None and (n == g.bearing_a.parent or axle_through_holes(ax, P[n]))]
    return min(cands, key=lambda n: (abs(float(np.dot(P[n].p - g.ca, g.axis))), n != g.bearing_a.parent))


def check_treads(design, P) -> tuple[list[str], list[str]]:
    """Validator checks of the design's treads (see the module docstring)."""
    from .design import OVERLAP_TOL, _components, _depth_in_shape, rigid_edges  # noqa: PLC0415

    E, W = [], []
    treads = getattr(design, "treads", []) or []
    names = [t.name for t in treads]
    for n in {n for n in names if names.count(n) > 1}:
        E.append(f"duplicate tread name '{n}'")
    used = [s for t in treads for s in t.sprockets]
    for s in {s for s in used if used.count(s) > 1}:
        E.append(f"sprocket '{s}' carries more than one tread")
    rigid = _components([p.name for p in design.parts], rigid_edges(design))
    comp = {n: i for i, c in enumerate(rigid) for n in c}
    geoms = []
    for t in treads:
        g = _resolve(design, t, P)
        if not isinstance(g, TreadGeom):
            E.extend(g)
            continue
        E.extend(g.errors)
        tag = f"tread {t.name}"
        if t.links < 1:
            E.append(f"{tag}: needs links")
            continue
        r_spr = max(x.bp.wheel_radius or getattr(x.bp.shape, "radius", 0.0) for x in (g.a, g.b))  # (gears: tips)
        if g.D < 2 * r_spr + 1.0:
            E.append(f"{tag}: sprocket centres {g.D:.1f} mm apart: the sprockets touch (>= {2 * r_spr + 1.0:.1f} mm)")
        pitch = TREAD_LINKS[t.link]["pitch"]
        if g.slack_mm < -0.5:
            need = math.ceil(g.loop_mm / pitch - 1e-9)
            E.append(f"{tag}: {t.links} links ({t.links * pitch:.1f} mm) are shorter than the loop round the sprockets "
                     f"({g.loop_mm:.1f} mm = 2 x {g.D:.1f} + 2 pi x {g.r_pitch:.2f}): needs >= {need} links")
        elif g.slack_mm > MAX_SLACK_PITCH * pitch + 1e-6:
            E.append(f"{tag}: {g.slack_mm:.1f} mm of slack (> {MAX_SLACK_PITCH:g} link): the real tread sags and jumps "
                     f"the sprocket; use {math.floor((g.loop_mm + MAX_SLACK_PITCH * pitch) / pitch)} links or move the "
                     f"sprockets {g.slack_mm / 2:.1f} mm apart")
        if g.slack_mm > 0.5 * pitch:
            W.append(f"{tag}: {g.slack_mm:.1f} mm of slack (the simulated tread is taut)")
        for side in ("a", "b"):
            axle, brg = getattr(g, f"axle_{side}"), getattr(g, f"bearing_{side}")
            if axle and brg is None:
                E.append(f"{tag}: the axle '{axle}' of sprocket {t.sprockets[0 if side == 'a' else 1]} needs its own "
                         f"hinge (Technic hole) or motor bearing")
        if g.axle_a and g.axle_a == g.axle_b:
            E.append(f"{tag}: both sprockets are on the same axle")
        if g.bearing_a is not None and g.bearing_b is not None:
            ca, cb = comp.get(g.bearing_a.parent), comp.get(g.bearing_b.parent)
            if ca is None or ca != cb:
                E.append(f"{tag}: the two sprocket axles must turn in one rigid (stud / pin) structure "
                         f"({g.bearing_a.parent} and {g.bearing_b.parent} are not rigidly joined)")
        # nothing else may sit inside the tread's shell (the links pass there)
        pts = g.shell_points()
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        for nm, q in P.items():
            if nm in t.sprockets:
                continue
            qlo, qhi = q.aabb()
            if np.any(qlo > hi) or np.any(lo > qhi):
                continue
            depth = float(_depth_in_shape(q.bp.shape, (q.R.T @ (pts - q.p).T).T).max())
            if depth > OVERLAP_TOL:
                E.append(f"{tag}: {nm} is inside the tread (the links pass {depth:.1f} mm through it; the tread is "
                         f"{2 * g.half_w:.1f} mm wide, r {g.r_in:.2f}..{g.r_tip:.2f} mm round the sprocket line)")
        geoms.append(g)
    for i, g in enumerate(geoms):  # two treads through each other
        for h in geoms[i + 1 :]:
            (la, ha), (lb, hb) = g.bbox(), h.bbox()
            if np.all(la < hb) and np.all(lb < ha):
                E.append(f"treads {g.tread.name} and {h.tread.name} overlap")
    return E, W


def tread_wheel_radii(design, P) -> dict[str, tuple[float, float, float]]:
    """{sprocket name: (ground radius = grouser tips, drive radius = pitch line, tread width)} of the treads' sprockets
    (``compute_stats``: ground contact, speed)."""
    out = {}
    for g in tread_geoms(design, P):
        for s in g.tread.sprockets:
            out[s] = (g.r_tip, g.r_pitch, 2 * g.half_w)
    return out


def tread_pieces(design) -> int:
    return sum(int(t.links) for t in (getattr(design, "treads", []) or []))


def tread_mass_g(design) -> float:
    return sum(int(t.links) * TREAD_LINKS.get(t.link, TREAD_LINKS["3873"])["mass_g"]
               for t in (getattr(design, "treads", []) or []))


def tread_ratio(design, sa: str, sb: str) -> float:
    """Speed ratio w_b / w_a of sprockets ``sa`` -> ``sb`` of one tread (same sense)."""
    from .parts import bench_parts  # noqa: PLC0415

    cat = bench_parts()
    pa = next(p.part for p in design.parts if p.name == sa)
    pb = next(p.part for p in design.parts if p.name == sb)
    if cat[pa].kind != "wheel" or pa not in SPROCKETS or pb not in SPROCKETS:
        return 1.0
    return SPROCKETS[pa] / SPROCKETS[pb]


# ----------------------------------------------------------------------------------------------------------------
# simulation (Isaac Sim / PhysX): called from env.run_episode after the design joints
# ----------------------------------------------------------------------------------------------------------------


def _quat_from_matrix(R: np.ndarray) -> tuple[float, float, float, float]:
    """xyzw quaternion of a rotation matrix."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return (x, y, z, w)


def _qmat(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def author_treads(stage, root: str, design, pose, merged: dict | None = None) -> list[str]:
    """Author every tread's rollers (rigid bodies, revolute joints on the carrier, gear joints to the sprocket shaft),
    the 1:1 coupling of the two sprocket shafts, the collision filters and static visual links. Returns the roller
    prim paths."""
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade  # noqa: PLC0415

    from ai_pd_tasks.lego import car_design as cd  # noqa: PLC0415

    from .design import _components, place, rigid_edges, shafts  # noqa: PLC0415
    from .env import JOINT_SCOPE, MAX_ANGULAR_VEL_DEG_S, PART_POS_ITERS, PART_VEL_ITERS, joint_path  # noqa: PLC0415

    merged = merged or {}
    P = place(design)
    geoms = tread_geoms(design, P)
    if not geoms or "norollers" in os.environ.get("LEGO_BENCH_TREAD_DEBUG", ""):
        return []
    gs = float(os.environ.get("LEGO_BENCH_GEAR_SIGN", "1"))
    grousers = os.environ.get("LEGO_BENCH_TREAD_GROUSERS", "1") != "0"
    dbg = set(os.environ.get("LEGO_BENCH_TREAD_DEBUG", "").split(","))  # diagnostics only: nogear, norollgear, nospr, nocoll, novis, norollers
    body = lambda n: merged.get(n, n)  # noqa: E731
    bpath = lambda n: f"{root}/{body(n)}"  # noqa: E731
    sh = shafts(design)
    rigid = _components([p.name for p in design.parts], rigid_edges(design))

    mat_path = f"{root}/TreadMaterial"
    mat = UsdShade.Material.Define(stage, mat_path)
    mapi = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    mu_dbg = float(os.environ.get("LEGO_BENCH_TREAD_DEBUG_MU", "0"))  # diagnostics only (traction vs transmission)
    mapi.CreateStaticFrictionAttr(mu_dbg or ABS_MATERIAL[0])
    mapi.CreateDynamicFrictionAttr(mu_dbg or ABS_MATERIAL[1])
    mapi.CreateRestitutionAttr(0.0)

    def collider(prim):
        UsdPhysics.CollisionAPI.Apply(prim)
        pc = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        pc.CreateContactOffsetAttr().Set(0.0005)
        pc.CreateRestOffsetAttr().Set(0.0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        UsdGeom.Imageable(prim).CreatePurposeAttr("guide")

    out = []
    uses: dict[str, int] = {}  # gear joints per hinge (diagnostics)

    def gear(name, b0, h0, b1, h1, ratio):
        gj = PhysxSchema.PhysxPhysicsGearJoint.Define(stage, f"{root}/{JOINT_SCOPE}/{name}")
        gj.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
        gj.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        gj.CreateHinge0Rel().SetTargets([Sdf.Path(h0)])
        gj.CreateHinge1Rel().SetTargets([Sdf.Path(h1)])
        gj.CreateGearRatioAttr().Set(-gs * ratio)
        uses[h0] = uses.get(h0, 0) + 1
        uses[h1] = uses.get(h1, 0) + 1

    for g in geoms:
        t = g.tread
        L = TREAD_LINKS[t.link]
        car = carrier(g, design, P)
        if not car or g.bearing_a is None or g.bearing_b is None:
            continue
        cbody = body(car)
        # world pose of the carrier body (spawn) and of the design frame
        pc_w, qc_w = pose.parts[cbody]
        Rc_w = _qmat(qc_w)
        Rc_d = P[cbody].R
        axle_a = P[g.axle_a]
        Rr_d = axle_a.R  # roller frame = the first sprocket's axle frame (local x = axle axis)
        q_rel = _quat_from_matrix(Rc_d.T @ Rr_d)
        Rr_w = Rc_w @ Rc_d.T @ Rr_d
        qr_w = _quat_from_matrix(Rr_w)
        k = g.r_pitch / g.r_tip
        centres = g.rollers()
        m_kg = t.links * L["mass_g"] * 1e-3 / len(centres)
        # round 4b: heavy links (57518: 35 g a loop) on a light carrier (a 6 g rail) made the roller joints explode at
        # spawn; above ROLLER_CARRIER_RATIO x the carrier's mass, the rollers keep that much and the rest of the links'
        # mass goes onto the carrier (the vehicle's total is unchanged). Not for the 3873 tread (unchanged).
        m_car = P[car].bp.mass_g * 1e-3
        if t.link != "3873" and m_kg * len(centres) > ROLLER_CARRIER_RATIO * m_car:
            extra = m_kg * len(centres) - ROLLER_CARRIER_RATIO * m_car
            m_kg = ROLLER_CARRIER_RATIO * m_car / len(centres)
            ma_c = UsdPhysics.MassAPI(stage.GetPrimAtPath(f"{root}/{cbody}"))
            if ma_c.GetMassAttr().Get():
                ma_c.GetMassAttr().Set(float(ma_c.GetMassAttr().Get()) + extra)
        i_spin = m_kg * (g.r_pitch * 1e-3) ** 2 * float(os.environ.get("LEGO_BENCH_TREAD_INERTIA_SCALE", "1"))
        i_tr = 0.5 * i_spin + m_kg * (2 * g.half_w * 1e-3) ** 2 / 12.0
        rollers = []
        for i, c in enumerate(centres):
            path = f"{root}/tread_{t.name}_r{i}"
            x = UsdGeom.Xform.Define(stage, path)
            lp = Rc_d.T @ (c - P[cbody].p) * 1e-3
            pw = np.array(pc_w) + Rc_w @ lp
            x.AddTranslateOp().Set(Gf.Vec3d(*pw.tolist()))
            x.AddOrientOp().Set(Gf.Quatf(float(qr_w[3]), Gf.Vec3f(*(float(v) for v in qr_w[:3]))))
            prim = x.GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim)
            ma = UsdPhysics.MassAPI.Apply(prim)
            ma.CreateMassAttr().Set(m_kg)
            ma.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            ma.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(i_spin, i_tr, i_tr))
            ma.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0))
            rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rb.CreateSolverPositionIterationCountAttr().Set(max(PART_POS_ITERS, TREAD_POS_ITERS))
            rb.CreateSolverVelocityIterationCountAttr().Set(max(PART_VEL_ITERS, TREAD_VEL_ITERS))
            rb.CreateMaxDepenetrationVelocityAttr().Set(0.2)
            rb.CreateMaxAngularVelocityAttr().Set(MAX_ANGULAR_VEL_DEG_S)
            rb.CreateSleepThresholdAttr().Set(0.0)
            end = i in (0, len(centres) - 1) and L["grouser_h"] > 0  # (the wide 57518 has no grousers)
            r_core = (g.r_tip - L["grouser_h"]) if (end and grousers) else g.r_tip
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/core")
            cyl.CreateRadiusAttr(r_core * 1e-3)
            cyl.CreateHeightAttr(2 * g.half_w * 1e-3)
            cyl.CreateAxisAttr("X")
            if "nocoll" not in dbg:
                collider(cyl.GetPrim())
            if end and grousers and "nocoll" not in dbg:
                hh = L["grouser_h"] + 0.4  # 0.4 mm sunk into the core (no gap)
                rc = (g.r_tip - 0.5 * hh) * 1e-3
                for j in range(END_GROUSERS):
                    phi = 2 * math.pi * j / END_GROUSERS
                    cube = UsdGeom.Cube.Define(stage, f"{path}/grouser_{j}")
                    cube.CreateSizeAttr(1.0)
                    cube.AddTranslateOp().Set(Gf.Vec3d(0.0, -rc * math.sin(phi), rc * math.cos(phi)))
                    cube.AddOrientOp().Set(Gf.Quatf(math.cos(phi / 2), Gf.Vec3f(math.sin(phi / 2), 0.0, 0.0)))
                    cube.AddScaleOp().Set(Gf.Vec3f(2 * g.half_w * 1e-3, L["grouser_w"] * 1e-3, hh * 1e-3))
                    collider(cube.GetPrim())
            # revolute joint on the carrier (about the roller's local x = the axle direction)
            j = UsdPhysics.RevoluteJoint.Define(stage, f"{root}/{JOINT_SCOPE}/tread_{t.name}_r{i}")
            j.CreateAxisAttr("X")
            j.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{cbody}")])
            j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
            j.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in lp)))
            j.CreateLocalRot0Attr().Set(Gf.Quatf(float(q_rel[3]), Gf.Vec3f(*(float(v) for v in q_rel[:3]))))
            j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
            j.CreateCollisionEnabledAttr(False)
            d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(0.0)
            d.CreateDampingAttr(cd.HINGE_DAMPING_NM_S_PER_RAD * math.pi / 180.0)
            d.CreateMaxForceAttr(0.01)
            d.CreateTargetVelocityAttr(0.0)
            PhysxSchema.PhysxJointAPI.Apply(j.GetPrim())
            # belt kinematics: roller surface (grouser tips) at the belt speed of the first sprocket's pitch line
            rollers.append(path)
        # belt kinematics: PhysX gear joints. The first roller is geared to sprocket a's shaft (ratio r_pitch / r_tip),
        # each next roller 1:1 to the previous one, and sprocket b's shaft 1:1 to a's. Tried and rejected in the smoke
        # runs: every roller geared to the sprocket bearing (several gear
        # joints on one hinge: the vehicle exploded at spawn); chains from both ends (unstable, veered); larger roller
        # inertia (exploded). The chain converges slowly: hence TREAD_POS_ITERS / TREAD_VEL_ITERS.
        xa, xb = P[g.axle_a].R[:, 0], P[g.axle_b].R[:, 0]
        s_ab = 1.0 if float(np.dot(xa, xb)) > 0 else -1.0
        sb_ratio = s_ab * tread_ratio(design, t.sprockets[0], t.sprockets[1])  # w_b / w_a about their own axes
        hinge = lambda i: f"{root}/{JOINT_SCOPE}/tread_{t.name}_r{i}"  # noqa: E731
        if not dbg & {"nogear", "nospr"}:
            gear(f"tread_{t.name}_sprockets", bpath(g.axle_a), joint_path(root, g.bearing_a), bpath(g.axle_b),
                 joint_path(root, g.bearing_b), sb_ratio)
        for i in range(len(rollers)):
            if dbg & {"nogear", "norollgear"}:
                break
            if i == 0:
                gear(f"tread_{t.name}_gear_r0", bpath(g.axle_a), joint_path(root, g.bearing_a), rollers[0], hinge(0), k)
            else:
                gear(f"tread_{t.name}_gear_r{i}", rollers[i - 1], hinge(i - 1), rollers[i], hinge(i), 1.0)
        # collision filters: rollers <-> each other, the sprockets, their shafts and the carrier's rigid cluster
        excl = set(next((c for c in rigid if car in c), {car}))
        for s, ax in zip(t.sprockets, (g.axle_a, g.axle_b), strict=True):
            excl |= {s, ax}
            excl |= {n for n, r in sh.items() if r == sh.get(ax)}
        targets = sorted({bpath(n) for n in excl if n in P})
        for rp in rollers:
            api = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(rp))
            for tp in targets + [q for q in rollers if q != rp]:
                api.CreateFilteredPairsRel().AddTarget(Sdf.Path(tp))
        # static visual links round the loop (children of the carrier body: no collision, no mass)
        if "novis" in dbg:
            out += rollers
            continue
        vis = f"{root}/{cbody}/TreadVisual_{t.name}"
        UsdGeom.Xform.Define(stage, vis)
        r_vis = g.r_pitch + (0.5 * L["pad"] if t.link == "3873" else 0.5 * (L["pad"] - L["inner"]))  # body mid-line
        loop = g.loop(r_vis, 0.5)
        lengths = np.cumsum([0.0] + [float(np.linalg.norm(loop[i + 1][0] - loop[i][0])) for i in range(len(loop) - 1)])
        for li in range(t.links):
            s_ = li * lengths[-1] / t.links
            idx = int(np.searchsorted(lengths, s_))
            p_, tan = loop[min(idx, len(loop) - 1)]
            rad = np.cross(tan, g.axis)
            Rl = np.stack([g.axis, tan, rad], axis=1)  # cube x = width, y = along, z = radial
            Rl_c = Rc_d.T @ Rl
            if np.linalg.det(Rl_c) < 0:
                Rl_c[:, 2] *= -1
            ql = _quat_from_matrix(Rl_c)
            lp = Rc_d.T @ (p_ - P[cbody].p) * 1e-3
            cube = UsdGeom.Cube.Define(stage, f"{vis}/link_{li}")
            cube.CreateSizeAttr(1.0)
            cube.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.36, 0.38)])
            cube.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in lp)))
            cube.AddOrientOp().Set(Gf.Quatf(float(ql[3]), Gf.Vec3f(*(float(v) for v in ql[:3]))))
            cube.AddScaleOp().Set(Gf.Vec3f(2 * g.half_w * 1e-3, 0.875 * L["pitch"] * 1e-3, (L["pad"] + L["inner"]) * 1e-3))
        out += rollers
    return out
