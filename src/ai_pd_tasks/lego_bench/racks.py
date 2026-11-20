# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Technic gear racks on a sliding guide (round 3, 2026-10-07): 3743 (1x4), 6630 (1x8 with holes), 87761 (1x7 with
axle and pin holes).

The real parts (LDraw, bench frame = the converted USD frame: rack along local x, teeth on top (+z), width y)
-------------------------------------------------------------------------------------------------------------
* Tooth pitch 3.2 mm (one rack tooth per 3.2 mm: 2.5 teeth per stud), the LEGO module-1 gears' pitch (pi mm) rounded
  to the stud grid. Tooth roots at local x = multiples of 3.2 mm on all three racks (so two racks laid end to end on
  the stud grid keep the tooth phase).
* 3743 rack 1 x 4: 32 x 8 mm, a plate-thick body (z -3.2..0.4) with 4 anti-studs underneath, 10 teeth over the full
  length (z 0.4..2.4). Mounts on studs.
* 6630 rack 1 x 8 with holes: 64 x 8 mm, 4 mm thick (z -4..0), 14 teeth between its round ends (x -22.4..22.4,
  z 0..1.6), a vertical pin hole (along local z, 4 mm long) in each round end (x = +-28). Mounts with pins.
* 87761 rack 1 x 7: 56 x 8 x 8 mm, pin holes along local z at x = +-24, cross (axle) holes along local y at x = +-16,
  8 teeth in the middle (x -12.8..12.8, z 1..3) between raised end blocks (z up to 4): the end blocks stop a pinion.
* Pitch line: tooth tip - 1.0 mm (module 1 addendum): z = 1.4 (3743), 0.6 (6630), 2.0 (87761). A spur gear with z
  teeth meshes with its centre z/2 mm from the pitch line (same jam / skip tolerances as two spur gears). Tooth for
  tooth, a pinion turn moves the rack z x 3.2 mm (effective pitch radius 0.509 z mm).

Model
-----
A design's ``racks`` field lists slides. A slide is one straight toothed bar made of one or more rack parts (joined end
to end, each held by studs / pins on a carrier), one pinion (a spur gear cross-fitted on an axle) and the guide parts
it slides in::

    "racks": [{"name": "lift", "racks": ["r1", "r2"], "pinion": "p16", "guides": ["g_back", "g_left", "g_right"]}]

* **Two sides.** The *rack side* is every part attached (connections or gear meshes) to the racks; the *guide side*
  everything attached to the guides and to the pinion's bearing. The slide is the only link between them: the
  validator rejects any other tie (a stud, pin, axle or gear mesh across). Either side may be the chassis side (a lift
  or a telescoping leg: rack side moves; a carriage on a fixed rack: guide side moves). The side without the chassis is
  a moving mechanism for the judge (its parts are checked against their own anchor, not the chassis).
* **Sim.** One PhysX prismatic joint between a guide body and a rack-side body, axis along the rack, limits at the
  travel ends (below): the guide. The pinion drives the rack by **real tooth contact**, like every spur mesh of the
  bench: the racks collide as a body box plus one straight-flank prism per tooth (20 deg flanks, 0.05 mm backlash per
  flank, cooked in mm like the gear teeth), and the pinion is turned at spawn so that one of its teeth sits in the
  rack's tooth gap at the contact (:func:`pinion_spins`; the spur gears meshing with it turn with it). Tooth for tooth,
  a pinion turn moves the rack z x 3.2 mm. Collisions are filtered between the rack side and the guide frame (the
  guides' rigid cluster, minus the parts the travel sweep found as stops): the joint is the guide, and a frame part
  flush with the slide rubs on it like a guide (PhysX contact friction on such exact contacts locked the slide in the
  rig). Every other pair collides: the pinion's teeth, the travel stops, and every mechanism part.
  (``LEGO_BENCH_RACK_MODEL=joint`` replaces the teeth by a PhysX rack-and-pinion joint: lossless, opt-in.)
* **Self-locking.** The mesh is reversible: a load on the rack back-drives the pinion, its gear train and the motor
  (a floating PF motor lets a lifted load drop). Through a worm (4716) it holds: the worm lock of the bench
  (``env.worm_lock_specs``) brakes the worm shaft while its motor is off.
* **Travel.** ``s`` = displacement of the rack side relative to the guide side along the rack's +x [mm], 0 at the
  design pose. Limits: the mesh range (the pinion's contact point stays >= half a tooth pitch inside the teeth), and the
  first contact of the rack side with any other part of the vehicle in each direction (swept at 1 mm, refined to
  0.1 mm; guides and the pinion excepted). A mesh end that is not a physical stop is where the real rack runs off its
  pinion: reaching it fails the run ``broke (rack ... ran off its pinion)``. The 87761 end blocks are physical stops.

Validator rules (``check_racks``)
---------------------------------
* every rack part belongs to exactly one slide; racks connect only by studs / pins (no cross / hinge fits);
* the racks of a slide are parallel, teeth on the same side, on one pitch line (<= 0.3 mm), contiguous (no toothless
  gap > 0.3 mm), in tooth phase (offset a multiple of 3.2 mm, +-0.3); every two neighbouring racks share a carrier: a
  non-rack part held directly to both by studs or by pins (real LEGO joins racks on a common beam or brick);
* the pinion is a spur gear (8T 10928, 12T 69778, 16T 94925 / 4019, 24T 3648, 40T 3649, double bevels 32270 /
  32269 / 32498), axis along the rack's width, mid-plane <= 2 mm off the rack's centre plane, centre z/2 mm from
  the pitch line (-0.15 / +0.6 mm), its contact inside the teeth at s = 0; its axle turns in a bearing (hinge or
  motor) rigidly joined to the guides;
* the guides are one rigid cluster on the other side; the rack side's cross-section is touched (gap <= 0.75 mm) by
  guide parts on its back face (away from the pinion: the tooth load pushes the rack there) and on both side faces,
  over >= 16 mm of its length each, at both travel ends and at s = 0; no guide part penetrates the rack side anywhere
  on the travel (it would jam); the travel is > 0.

Limits (stated plainly)
-----------------------
* The guide is ideal: no friction, no play, no wear (a real LEGO slide has friction and 0.1-0.2 mm of play).
* Tooth contact has the bench's spur-mesh limits (backlash, solver compliance; the guide is rigid, so the separating
  tooth force cannot push the rack away and make it skip as a soft real guide can under a few tens of N).
* The pinion must have yaw 0 (the bench's gear phasing works in the x-z plane), so racks run in the x-z plane.
* Only one pinion per slide.
* The clutch check (``LEGO_BENCH_CLUTCH=rigid``) models the mesh and the guides as springs at the design pose (rack
  side at s = 0).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np

RACK_PITCH_MM = 3.2
"""Rack tooth pitch [mm] (2.5 teeth per stud); a pinion turn moves the rack teeth x this."""

RACKS: dict[str, dict] = {
    "3743": dict(title="Technic gear rack 1x4", teeth=(-16.0, 16.0), n_teeth=10, pitch_z=1.4, tip_z=2.4,
                 end_stops=False),
    "6630": dict(title="Technic gear rack 1x8 with holes", teeth=(-22.4, 22.4), n_teeth=14, pitch_z=0.6, tip_z=1.6,
                 end_stops=False),
    "87761": dict(title="Technic gear rack 1x7 with axle and pin holes", teeth=(-12.8, 12.8), n_teeth=8, pitch_z=2.0,
                  tip_z=3.0, end_stops=True),
}
"""Rack parts: toothed span along local x [mm], tooth count, pitch-line and tip height (local z) [mm]; ``end_stops``:
raised ends that stop the pinion (87761)."""

MESH_MARGIN_MM = 0.5 * RACK_PITCH_MM
"""The pinion's contact point stays this far inside the toothed span (half a tooth: one tooth still engaged)."""
CENTRE_JAM_MM = 0.15
CENTRE_SKIP_MM = 0.6
LATERAL_TOL_MM = 2.0
GUIDE_MIN_MM = 16.0
GUIDE_GAP_MM = 0.75
JOIN_TOL_MM = 0.3
RAN_OUT_MM = 0.3
"""The run fails when the slide comes this close to a mesh end that is not a physical stop."""
SWEEP_STEP_MM = 1.0
RACK_POS_ITERS = int(os.environ.get("LEGO_BENCH_RACK_ITERS", "128"))
"""Solver position iterations of a design with a rack slide (PhysX solves an island at its largest count): the
rack-and-pinion joint couples a light pinion shaft to a heavy rack side (a 200 g lift: m r^2 ~ 1e-5 kg m^2 against
~1e-8 for the shaft)."""
RACK_MODEL = os.environ.get("LEGO_BENCH_RACK_MODEL", "teeth")
"""How the pinion drives the rack in simulation. ``teeth`` (default): real tooth contact (the racks' straight-flank tooth
hulls against the pinion's tooth hulls, like every spur mesh of the bench; the pinion is phased into the rack's teeth
at spawn). ``joint`` (opt-in): a PhysX rack-and-pinion joint (lossless, exact ratio; the pinion's collisions with the
racks filtered). Both give the tooth-for-tooth ratio on the rack rig."""
RACK_SIGN = float(os.environ.get("LEGO_BENCH_RACK_SIGN", "1"))
"""Diagnostics only: -1 flips the PhysX rack-and-pinion ratio (the default +1 is the sign calibrated on the rack rig:
the rack moves the way the teeth push it)."""
RACK_RATIO_UNIT = os.environ.get("LEGO_BENCH_RACK_RATIO_UNIT", "deg")
"""Unit of the USD ``physics:ratio`` (``deg`` per metre, the omni.physx convention, or ``rad`` per metre)."""


@dataclass
class DesignRack:
    """A slide in a design: rack parts (one straight toothed bar), the pinion meshing with it, the guide parts."""

    name: str
    racks: tuple[str, ...]
    pinion: str
    guides: tuple[str, ...]

    def to_dict(self) -> dict:
        return {"name": self.name, "racks": list(self.racks), "pinion": self.pinion, "guides": list(self.guides)}

    @classmethod
    def from_dict(cls, d: dict) -> DesignRack:
        unknown = set(d) - {"name", "racks", "pinion", "guides"}
        if unknown:
            raise ValueError(f"unknown rack slide fields: {sorted(unknown)}")
        return cls(str(d["name"]), tuple(str(r) for r in d["racks"]), str(d["pinion"]),
                   tuple(str(g) for g in d.get("guides", [])))


def pitch_radius_eff(teeth: int) -> float:
    """Rack travel per radian of the pinion [mm] (tooth for tooth: z x 3.2 mm per turn)."""
    return teeth * RACK_PITCH_MM / (2.0 * math.pi)


@dataclass
class RackGeom:
    """A resolved slide (design frame, mm)."""

    slide: DesignRack
    racks: list  # design.Placed, sorted along e
    e: np.ndarray  # travel axis (the first rack's local x)
    n: np.ndarray  # teeth normal (towards the pinion)
    v: np.ndarray  # width axis
    o: np.ndarray  # origin on the pitch line (first rack's centre + n pitch_z)
    spans: list  # [(rack name, lo, hi)] toothed spans along e (relative to o), sorted
    lo: float = 0.0  # toothed span of the whole bar
    hi: float = 0.0
    pinion: object = None
    x_c: float = 0.0  # pinion contact along e
    d_n: float = 0.0  # pinion centre from the pitch line
    d_v: float = 0.0
    r_eff: float = 0.0
    axle: str = ""
    bearing: object = None  # DesignConnection (hinge / motor) of the pinion's axle
    rack_side: set = field(default_factory=set)
    rack_rigid: set = field(default_factory=set)
    guide_rigid: set = field(default_factory=set)
    errors: list = field(default_factory=list)

    @property
    def mesh_range(self) -> tuple[float, float]:
        """Slide displacements [mm] for which the pinion stays in mesh."""
        return (self.x_c - self.hi + MESH_MARGIN_MM, self.x_c - self.lo - MESH_MARGIN_MM)

    def contact_rack(self, s: float = 0.0) -> str | None:
        """The rack the pinion meshes with at displacement s."""
        x = self.x_c - s
        for name, a, b in self.spans:
            if a - 1e-6 <= x <= b + 1e-6:
                return name
        return None

    def frame(self) -> np.ndarray:
        """Joint frame (columns e, n, e x n) in the design frame."""
        return np.stack([self.e, self.n, np.cross(self.e, self.n)], axis=1)

    def ends_physical(self) -> tuple[bool, bool]:
        """Whether the bar's first / last rack has end blocks (a physical stop at that mesh end)."""
        first = next(r for r in self.racks if r.inst.name == self.spans[0][0])
        last = next(r for r in self.racks if r.inst.name == self.spans[-1][0])
        return RACKS[first.inst.part]["end_stops"], RACKS[last.inst.part]["end_stops"]


def _graph_components(design, extra_edges=()):
    from .design import _components  # noqa: PLC0415

    names = [p.name for p in design.parts]
    edges = [(c.child, c.parent) for c in design.connections]
    edges += [(m.gear_a, m.gear_b) for m in design.gear_meshes]
    edges += list(extra_edges)
    return _components(names, edges)


def rack_parts(design, P=None) -> list[str]:
    """Names of the design's rack parts."""
    return [p.name for p in design.parts if p.part in RACKS]


def resolve(design, slide: DesignRack, P=None):
    """A :class:`RackGeom` (with its errors in ``errors``), or a list of errors when it cannot be resolved."""
    from .design import _components, place, rigid_edges  # noqa: PLC0415

    P = P if P is not None else place(design)
    tag = f"rack slide {slide.name}"
    if not slide.racks:
        return [f"{tag}: needs at least one rack part"]
    for r in slide.racks:
        if r not in P:
            return [f"{tag}: unknown part '{r}'"]
        if P[r].inst.part not in RACKS:
            return [f"{tag}: '{r}' ({P[r].inst.part}) is not a gear rack ({sorted(RACKS)})"]
    if slide.pinion not in P:
        return [f"{tag}: unknown pinion '{slide.pinion}'"]
    if not slide.guides:
        return [f"{tag}: needs guide parts (a rack must slide in a guide)"]
    for g in slide.guides:
        if g not in P:
            return [f"{tag}: unknown guide part '{g}'"]
        if g in slide.racks:
            return [f"{tag}: rack '{g}' cannot be its own guide"]
    errs: list[str] = []
    r0 = P[slide.racks[0]]
    e, n, v = r0.R[:, 0].copy(), r0.R[:, 2].copy(), r0.R[:, 1].copy()
    o = r0.p + n * RACKS[r0.inst.part]["pitch_z"]
    spans = []
    for name in slide.racks:
        r = P[name]
        spec = RACKS[r.inst.part]
        if abs(float(np.dot(r.R[:, 0], e))) < 0.999:
            errs.append(f"{tag}: rack '{name}' is not parallel to '{slide.racks[0]}'")
            continue
        if float(np.dot(r.R[:, 2], n)) < 0.999:
            errs.append(f"{tag}: rack '{name}' has its teeth on another side than '{slide.racks[0]}'")
            continue
        pt = r.p + n * spec["pitch_z"] - o
        off_n, off_v = float(np.dot(pt, n)), float(np.dot(pt, v))
        if abs(off_n) > JOIN_TOL_MM or abs(off_v) > JOIN_TOL_MM:
            errs.append(f"{tag}: rack '{name}' is off the pitch line of '{slide.racks[0]}' ({off_n:+.2f} mm across, "
                        f"{off_v:+.2f} mm sideways)")
            continue
        t = float(np.dot(r.p - r0.p, e))
        sg = 1.0 if float(np.dot(r.R[:, 0], e)) > 0 else -1.0
        a, b = sorted((t + sg * spec["teeth"][0], t + sg * spec["teeth"][1]))
        spans.append((name, a, b))
        ph = (t % RACK_PITCH_MM)
        if min(ph, RACK_PITCH_MM - ph) > JOIN_TOL_MM:
            errs.append(f"{tag}: rack '{name}' is {min(ph, RACK_PITCH_MM - ph):.2f} mm out of tooth phase with "
                        f"'{slide.racks[0]}' (offset along the rack must be a multiple of {RACK_PITCH_MM} mm)")
    if errs:
        return errs
    spans.sort(key=lambda s: s[1])
    g = RackGeom(slide, sorted((P[r] for r in slide.racks), key=lambda r: float(np.dot(r.p - r0.p, e))), e, n, v, o,
                 spans, spans[0][1], spans[-1][2])
    for (na, _, b), (nb, a, _) in zip(spans[:-1], spans[1:], strict=True):
        if a - b > JOIN_TOL_MM:
            g.errors.append(f"{tag}: {a - b:.1f} mm without teeth between racks '{na}' and '{nb}' (the pinion loses "
                            f"mesh there; lay the racks end to end)")
        elif a - b < -JOIN_TOL_MM:
            g.errors.append(f"{tag}: racks '{na}' and '{nb}' overlap by {b - a:.1f} mm")
    # pinion
    pn = P[slide.pinion]
    g.pinion = pn
    if pn.bp.kind != "gear" or pn.bp.teeth < 6 or pn.bp.bevel_only:
        g.errors.append(f"{tag}: pinion '{slide.pinion}' ({pn.inst.part}) must be a spur gear (8T 10928, 16T 94925, "
                        f"24T 3648, 40T 3649, ...): not a worm or single bevel")
        return g
    if abs(float(np.dot(pn.axis(), v))) < 0.999:
        g.errors.append(f"{tag}: the pinion's axis must run across the rack (along its width)")
        return g
    if abs(pn.inst.yaw_deg % 360.0) > 1e-6:
        g.errors.append(f"{tag}: the pinion must have yaw 0 (gear axes along the design y axis, as every spur mesh)")
    d = pn.p - o
    g.x_c, g.d_n, g.d_v = float(np.dot(d, e)), float(np.dot(d, n)), float(np.dot(d, v))
    g.r_eff = pitch_radius_eff(pn.bp.teeth)
    nominal = pn.bp.teeth / 2.0
    if g.d_n < 0:
        g.errors.append(f"{tag}: the pinion is on the back of the rack (teeth face the other way)")
    elif g.d_n < nominal - CENTRE_JAM_MM:
        g.errors.append(f"{tag}: pinion centre {g.d_n:.2f} mm from the pitch line < nominal {nominal:.1f} mm (teeth jam)")
    elif g.d_n > nominal + CENTRE_SKIP_MM:
        g.errors.append(f"{tag}: pinion centre {g.d_n:.2f} mm from the pitch line > nominal {nominal:.1f} mm + "
                        f"{CENTRE_SKIP_MM} (teeth skip)")
    if abs(g.d_v) > LATERAL_TOL_MM:
        g.errors.append(f"{tag}: the pinion's mid-plane is {abs(g.d_v):.2f} mm off the rack's centre plane "
                        f"(> {LATERAL_TOL_MM} mm: less than half its face on the teeth)")
    lo, hi = g.mesh_range
    if not lo <= 0.0 <= hi:
        g.errors.append(f"{tag}: the pinion is off the teeth at the design pose (contact {g.x_c:.1f} mm along the bar, "
                        f"teeth {g.lo:.1f}..{g.hi:.1f})")
    fit = [c for c in design.connections if c.child == slide.pinion and c.kind == "cross"]
    if len(fit) == 1:
        g.axle = fit[0].parent
        g.bearing = next((c for c in design.connections if c.child == g.axle and c.kind in ("hinge", "motor")), None)
        if g.bearing is None:
            g.errors.append(f"{tag}: the pinion's axle '{g.axle}' has no hinge / motor bearing")
    elif not fit:
        g.errors.append(f"{tag}: the pinion must be cross-fitted on an axle")
    # sides
    rigid = _components([p.name for p in design.parts], rigid_edges(design))
    comp = {nm: i for i, c in enumerate(rigid) for nm in c}
    g.rack_rigid = set(rigid[comp[slide.racks[0]]])
    g.guide_rigid = set(rigid[comp[slide.guides[0]]])
    for r in slide.racks[1:]:
        if r not in g.rack_rigid:
            g.errors.append(f"{tag}: rack '{r}' is not rigidly (stud / pin) joined to '{slide.racks[0]}' (the racks of a "
                            f"slide are one bar on a common carrier)")
    for gd in slide.guides[1:]:
        if gd not in g.guide_rigid:
            g.errors.append(f"{tag}: guide '{gd}' is not rigidly joined to '{slide.guides[0]}' (the guides are one frame)")
    if g.rack_rigid & g.guide_rigid:
        g.errors.append(f"{tag}: the racks are rigidly joined to their guides (nothing can slide)")
    if g.bearing is not None and g.bearing.parent not in g.guide_rigid:
        g.errors.append(f"{tag}: the pinion's axle must turn in a part of the guide frame ('{g.bearing.parent}' is not "
                        f"rigidly joined to '{slide.guides[0]}')")
    comps = _graph_components(design)
    side = next(c for c in comps if slide.racks[0] in c)
    g.rack_side = set(side)
    if slide.guides[0] in side:
        g.errors.append(f"{tag}: the rack side is tied to the guide side by another connection or gear mesh (the slide "
                        f"would be locked); only the slide may join them")
    return g


def carriers(design, rack: str) -> set[str]:
    """Non-rack parts held directly to ``rack`` by studs or through a pin."""
    out = set()
    pins: dict[str, set[str]] = {}
    for c in design.connections:
        if c.kind == "stud" and rack in (c.child, c.parent):
            out.add(c.parent if c.child == rack else c.child)
        if c.kind == "pin":
            pins.setdefault(c.child, set()).add(c.parent)
    for _, held in pins.items():
        if rack in held:
            out |= held - {rack}
    parts = {p.name: p.part for p in design.parts}
    return {n for n in out if parts.get(n) not in RACKS}


def _samples_world(pl, step: float = 1.5) -> np.ndarray:
    from .design import _samples  # noqa: PLC0415

    return (pl.R @ _samples(pl.bp.shape, step).T).T + pl.p


def _depth(pl, pts_w: np.ndarray) -> np.ndarray:
    from .design import _depth_in_shape  # noqa: PLC0415

    return _depth_in_shape(pl.bp.shape, (pl.R.T @ (pts_w - pl.p).T).T)


class _Sweep:
    """Penetration of the rack side moved by s along e against every other part (cached samples)."""

    def __init__(self, g: RackGeom, P: dict, movers: set[str], others: set[str], skip: set):
        self.g, self.P = g, P
        self.movers = sorted(movers & set(P))
        self.others = sorted(others & set(P))
        self.skip = skip
        self.pts = {}
        self.box = {nm: P[nm].aabb() for nm in set(self.movers) | set(self.others)}

    def _pts(self, nm):
        if nm not in self.pts:
            self.pts[nm] = _samples_world(self.P[nm])
        return self.pts[nm]

    def worst(self, s: float, only: set | None = None) -> tuple[float, str, str]:
        from .design import OVERLAP_TOL  # noqa: PLC0415

        sh = self.g.e * s
        best = (0.0, "", "")
        for m in self.movers:
            lo_m, hi_m = self.box[m]
            lo_m, hi_m = lo_m + sh, hi_m + sh
            for o in self.others:
                if (m, o) in self.skip or (only is not None and o not in only):
                    continue
                lo_o, hi_o = self.box[o]
                if np.any(lo_m > hi_o + OVERLAP_TOL) or np.any(lo_o > hi_m + OVERLAP_TOL):
                    continue
                d1 = float(_depth(self.P[o], self._pts(m) + sh).max())
                d2 = float(_depth(self.P[m], self._pts(o) - sh).max())
                d = max(d1, d2)
                if d > best[0]:
                    best = (d, m, o)
        return best

    def first_hit(self, direction: float, limit: float, only: set | None = None) -> tuple[float, str | None, str | None]:
        """Largest |s| <= |limit| in ``direction`` that stays free (penetration <= OVERLAP_TOL), and the parts that
        touch there (None if the limit is free)."""
        from .design import OVERLAP_TOL  # noqa: PLC0415

        if abs(limit) < 1e-9:
            return 0.0, None, None
        n = max(1, int(math.ceil(abs(limit) / SWEEP_STEP_MM)))
        prev = 0.0
        for k in range(1, n + 1):
            s = direction * min(k * SWEEP_STEP_MM, abs(limit))
            d, m, o = self.worst(s, only)
            if d > OVERLAP_TOL:
                a, b = prev, s
                for _ in range(4):  # refine to ~0.06 mm
                    mid = 0.5 * (a + b)
                    if self.worst(mid, only)[0] > OVERLAP_TOL:
                        b = mid
                    else:
                        a = mid
                return a, m, o
            prev = s
        return direction * abs(limit), None, None


def _guided_length(g: RackGeom, P: dict, s: float, guides: list) -> dict[str, float]:
    """Guided length [mm] of the rack side's back (-n) and side (+-v) faces at displacement s."""
    bins = 2.0
    pts = np.concatenate([_samples_world(P[nm]) for nm in sorted(g.rack_rigid) if nm in P])
    rel = pts - g.o
    pe, pn, pv = rel @ g.e, rel @ g.n, rel @ g.v
    k = np.floor(pe / bins).astype(int)
    out = {"back": 0.0, "side+": 0.0, "side-": 0.0}
    for kb in np.unique(k):
        sel = k == kb
        n_min, v_min, v_max = float(pn[sel].min()), float(pv[sel].min()), float(pv[sel].max())
        n_max = float(pn[sel].max())
        e_c = (kb + 0.5) * bins + s
        base = g.o + g.e * e_c
        probes = {
            "back": [base + g.n * (n_min - dl) + g.v * vv for vv in np.linspace(v_min + 0.5, v_max - 0.5, 4)
                     for dl in (0.25, GUIDE_GAP_MM)],
            "side+": [base + g.v * (v_max + dl) + g.n * nn for nn in np.linspace(n_min + 0.5, n_max - 0.5, 4)
                      for dl in (0.25, GUIDE_GAP_MM)],
            "side-": [base + g.v * (v_min - dl) + g.n * nn for nn in np.linspace(n_min + 0.5, n_max - 0.5, 4)
                      for dl in (0.25, GUIDE_GAP_MM)],
        }
        for face, pp in probes.items():
            arr = np.array(pp)
            if any(float(_depth(P[gd], arr).max()) > 0.05 for gd in guides):
                out[face] += bins
    return out


@dataclass
class Travel:
    lo: float
    hi: float
    lo_kind: str  # 'mesh_end' (the rack runs off its pinion) | 'rack_end_stop' (87761 end blocks) | 'stop:<part>'
    hi_kind: str


def travel(design, g: RackGeom, P: dict) -> tuple[Travel, list[str], list[str]]:
    """Travel limits of a resolved slide (mesh range, stops) and the guide / jam errors and warnings."""
    tag = f"rack slide {g.slide.name}"
    E, W = [], []
    names = set(P)
    movers = set(g.rack_side)
    others = names - movers
    guides = [gd for gd in g.slide.guides if gd in P]
    skip = {(r, g.slide.pinion) for r in g.slide.racks}
    skip |= {(m, gd) for m in movers for gd in guides}
    sw = _Sweep(g, P, movers, others, skip)
    m_lo, m_hi = g.mesh_range
    s_hi, a_hi, b_hi = sw.first_hit(1.0, max(m_hi, 0.0))
    s_lo, a_lo, b_lo = sw.first_hit(-1.0, min(m_lo, 0.0))
    phys_lo, phys_hi = g.ends_physical()
    # s > 0 moves the contact (x_c - s) towards the bar's low end: the upper travel limit is the bar's first rack
    hi_kind = f"stop:{b_hi}" if b_hi else ("rack_end_stop" if phys_lo else "mesh_end")
    lo_kind = f"stop:{b_lo}" if b_lo else ("rack_end_stop" if phys_hi else "mesh_end")
    tr = Travel(round(s_lo, 2), round(s_hi, 2), lo_kind, hi_kind)
    if tr.hi - tr.lo < 1.0:
        W.append(f"{tag}: travel {tr.lo:+.1f}..{tr.hi:+.1f} mm (the slide hardly moves: blocked by "
                 f"{b_lo or b_hi or 'the mesh range'})")
    # guides: no jam anywhere on the travel; the cross-section held on its back and both sides at the ends and at 0
    gw = _Sweep(g, P, movers, set(guides), {(r, g.slide.pinion) for r in g.slide.racks})
    span = tr.hi - tr.lo
    for s in sorted({tr.lo, 0.0, tr.hi} | {tr.lo + span * k / 8.0 for k in range(1, 8)}):
        d, m, o = gw.worst(s)
        from .design import OVERLAP_TOL  # noqa: PLC0415

        if d > OVERLAP_TOL:
            E.append(f"{tag}: '{m}' runs {d:.2f} mm into guide '{o}' at s = {s:+.1f} mm (the slide jams)")
            break
    for s in sorted({tr.lo, 0.0, tr.hi}):
        lens = _guided_length(g, P, s, guides)
        short = {f: L for f, L in lens.items() if L < GUIDE_MIN_MM}
        if short:
            E.append(f"{tag}: at s = {s:+.1f} mm the rack side is guided over only "
                     + ", ".join(f"{L:.0f} mm on its {f} face" for f, L in short.items())
                     + f" (needs >= {GUIDE_MIN_MM:.0f} mm on the back face, away from the pinion, and on both side "
                     f"faces, gap <= {GUIDE_GAP_MM} mm)")
            break
    return tr, E, W


def rack_geoms(design, P=None) -> list[RackGeom]:
    """Every resolvable slide of ``design`` (errors in each ``errors``)."""
    from .design import place  # noqa: PLC0415

    P = P if P is not None else place(design)
    return [g for g in (resolve(design, s, P) for s in getattr(design, "racks", []) or []) if isinstance(g, RackGeom)]


def check_racks(design, P) -> tuple[list[str], list[str], dict]:
    """Validator checks of the design's racks (module docstring); returns errors, warnings and stats."""
    E, W = [], []
    stats: dict = {}
    slides = getattr(design, "racks", []) or []
    names = [s.name for s in slides]
    for nm in {nm for nm in names if names.count(nm) > 1}:
        E.append(f"duplicate rack slide name '{nm}'")
    in_slides = [r for s in slides for r in s.racks]
    for r in {r for r in in_slides if in_slides.count(r) > 1}:
        E.append(f"rack '{r}' is in more than one slide")
    for r in rack_parts(design):
        if r not in in_slides:
            E.append(f"rack '{r}' is in no slide (a rack must slide in a guide and mesh with a pinion: add a 'racks' "
                     f"entry)")
    pin_used = [s.pinion for s in slides]
    for p in {p for p in pin_used if pin_used.count(p) > 1}:
        E.append(f"pinion '{p}' drives more than one slide")
    rk = set(rack_parts(design))
    for c in design.connections:
        if (c.child in rk or c.parent in rk) and c.kind not in ("stud", "pin"):
            E.append(f"{c.kind} {c.child}->{c.parent}: a rack connects to its carrier by studs or pins only")
    for s in slides:
        g = resolve(design, s, P)
        if not isinstance(g, RackGeom):
            E.extend(g)
            continue
        E.extend(g.errors)
        tag = f"rack slide {s.name}"
        order = [nm for nm, _, _ in g.spans]
        for a, b in zip(order[:-1], order[1:], strict=True):
            if not carriers(design, a) & carriers(design, b):
                E.append(f"{tag}: racks '{a}' and '{b}' share no carrier (join racks end to end on a common beam or "
                         f"brick: a part held by studs / pins to both)")
        from .design import STUD_Z_TOL, _count_studs  # noqa: PLC0415

        locked = []
        for m in sorted(g.rack_rigid):
            for gd in s.guides:
                for up, lowp in ((m, gd), (gd, m)):
                    k, gap = _count_studs(P[up], P[lowp])
                    if k and gap <= STUD_Z_TOL:
                        locked.append(f"{up} on {lowp} ({k} studs)")
        if locked:
            E.append(f"{tag}: the slide rides on studs: {', '.join(locked[:3])} (engaged studs lock a slider: let it "
                     f"ride on smooth faces, e.g. brick sides, a liftarm's edge, or a brick's underside on a smooth top)")
        if g.errors or locked:
            continue
        tr, e2, w2 = travel(design, g, P)
        E.extend(e2)
        W.extend(w2)
        if any(k == "mesh_end" for k in (tr.lo_kind, tr.hi_kind)):
            W.append(f"{tag}: travel {tr.lo:+.1f}..{tr.hi:+.1f} mm ends where the rack runs off its pinion "
                     f"({tr.lo_kind} / {tr.hi_kind}); a run that reaches a mesh end fails (add an end stop or keep "
                     f"the program inside)")
        stats[s.name] = {
            "travel_mm": [tr.lo, tr.hi],
            "travel_ends": [tr.lo_kind, tr.hi_kind],
            "pinion_teeth": int(g.pinion.bp.teeth),
            "mm_per_pinion_turn": round(g.pinion.bp.teeth * RACK_PITCH_MM, 2),
            "contact_rack": g.contact_rack(0.0),
            "moving_side": "racks" if design.chassis not in g.rack_side else "guides",
            "axis": [round(float(x), 4) for x in g.e],
        }
    return E, W, stats


def drive_stats(design, P, spins: dict, w_ref: float, stall_nm: float) -> dict:
    """Per slide: rack speed [mm/s] at the motors' no-load speeds (relative to the guide side, + = along the rack's
    +x), lossless stall force [N], and whether the drive is self-locking (a worm in the train)."""
    from .design import shafts  # noqa: PLC0415
    from .env import worm_lock_specs  # noqa: PLC0415

    sh = shafts(design)
    out = {}
    worm_gear_shafts = {sh.get(s["gear"]) for s in worm_lock_specs(design)}
    for g in rack_geoms(design, P):
        if g.errors or g.pinion is None:
            continue
        key = sh.get(g.slide.pinion)
        w = spins.get(key)
        ent = {}
        if w is not None:
            # contact point on the pitch line: -n r_eff from the pinion centre; rack velocity = (w x r) . e
            k = float(np.dot(np.cross(w, -g.n), g.e)) * g.r_eff  # mm per s per (rad/s of the primary motor)
            ent["speed_mm_s"] = round(k * w_ref, 2)
            ratio = abs(float(np.dot(w, g.pinion.axis())))
            if ratio > 1e-9:
                ent["stall_force_n"] = round(stall_nm / ratio / (g.r_eff * 1e-3), 1)
        # self-locking: the pinion's shaft is reached from a worm's gear through spur meshes only
        reach, todo = set(), [key]
        while todo:
            x = todo.pop()
            if x in reach or x is None:
                continue
            reach.add(x)
            for m in design.gear_meshes:
                a, b = sh.get(m.gear_a), sh.get(m.gear_b)
                if P[m.gear_a].bp.teeth == 1 or P[m.gear_b].bp.teeth == 1:
                    continue
                if a == x:
                    todo.append(b)
                elif b == x:
                    todo.append(a)
        ent["self_locking"] = bool(reach & worm_gear_shafts)
        out[g.slide.name] = ent
    return out


def moving_groups(design) -> list[tuple[str, set[str]]]:
    """(anchor, members) of each slide's moving side (the side without the chassis) for the judge."""
    out = []
    for g in rack_geoms(design):
        if not g.rack_side:
            continue
        if design.chassis not in g.rack_side:
            out.append((g.slide.racks[0], set(g.rack_side)))
        else:
            rest = {p.name for p in design.parts} - set(g.rack_side)
            if rest:
                out.append((g.slide.guides[0], rest))
    return out


def slide_edges(design) -> list[tuple[str, str]]:
    """(rack, guide) of each slide: the slide joins the two sides (assembly connectivity)."""
    return [(s.racks[0], s.guides[0]) for s in (getattr(design, "racks", []) or []) if s.racks and s.guides]


def structure_springs(design, frames: dict, k_lin: float, k_free_lin: float, k_rot: float, k_free_rot: float) -> list:
    """Springs of the clutch check's structural model (``clutch.Structure``): the mesh (pinion <-> the rack in contact,
    along the rack and across the pitch line) and each guide part (guide <-> nearest part of the rack's rigid
    cluster, across the rack, free along it). ``frames``: {part: (position [m], rotation)} at the design pose."""
    out = []
    k = 0
    for g in rack_geoms(design):
        if g.errors:
            continue
        rk = g.contact_rack(0.0)
        if rk is None:
            continue
        Rd0 = next(r.R for r in g.racks if r.inst.name == g.slide.racks[0])
        M = np.asarray(frames[g.slide.racks[0]][1]) @ Rd0.T  # design frame -> world rotation
        e, n = M @ g.e, M @ g.n
        E = np.stack([e, n, np.cross(e, n)], axis=1)
        # contact point: the contact rack's frame origin + (contact - rack centre), in world
        r_des = next(r for r in g.racks if r.inst.name == rk)
        c_des = g.o + g.e * g.x_c
        pc = np.asarray(frames[rk][0]) + M @ ((c_des - r_des.p) * 1e-3)
        k -= 1
        out.append((k, g.slide.pinion, rk, pc, E, (k_lin, k_lin, k_free_lin), (k_free_rot, k_free_rot, k_free_rot)))
        for gd in g.slide.guides:
            if gd not in frames:
                continue
            pg = np.asarray(frames[gd][0])
            near = min((nm for nm in g.rack_rigid if nm in frames),
                       key=lambda nm: float(np.linalg.norm(np.asarray(frames[nm][0]) - pg)))
            pn_ = np.asarray(frames[near][0])
            k -= 1
            out.append((k, gd, near, 0.5 * (pg + pn_), E, (k_free_lin, k_lin, k_lin), (k_rot, k_rot, k_rot)))
    return out


def pinion_spins(design, spins: dict[str, float], tooth_phase_deg: dict[str, float] | None = None) -> dict[str, float]:
    """Spin [deg] of each slide's pinion so that a pinion tooth sits in the rack's tooth gap at the contact (the
    convention of ``design.gear_spins``: a tooth at ``phase`` in the gear frame is at ``phase - spin`` in the x-z
    plane), applied on top of ``spins`` (the gear trains' phases); the spur gears meshing with the pinion (and theirs)
    turn with it so their phases are kept."""
    from .design import part_metadata, place  # noqa: PLC0415

    if tooth_phase_deg is None:
        tooth_phase_deg = {pid: m["tooth_phase_deg"] for pid, m in part_metadata().items() if "tooth_phase_deg" in m}
    out = dict(spins)
    P = place(design)
    for g in rack_geoms(design, P):
        pn = g.pinion
        if pn is None or g.errors:
            continue
        z = pn.bp.teeth
        th = math.degrees(math.atan2(-g.n[2], -g.n[0]))
        delta = (g.x_c + 0.5 * RACK_PITCH_MM) % RACK_PITCH_MM - 0.5 * RACK_PITCH_MM
        tang = np.array([-math.sin(math.radians(th)), 0.0, math.cos(math.radians(th))])
        sg = 1.0 if float(np.dot(tang, g.e)) > 0 else -1.0
        phi = th - delta * sg * 360.0 / (RACK_PITCH_MM * z)
        want = (tooth_phase_deg.get(pn.inst.part, 0.0) - phi) % 360.0
        shift = {pn.inst.name: want - out.get(pn.inst.name, pn.inst.spin_deg)}
        todo = [pn.inst.name]
        while todo:  # spur meshes turn with the pinion: delta_b = -delta_a z_a / z_b
            a = todo.pop()
            for m in design.gear_meshes:
                if a not in (m.gear_a, m.gear_b):
                    continue
                b = m.gear_b if m.gear_a == a else m.gear_a
                if b in shift or P[a].bp.teeth == 1 or P[b].bp.teeth == 1:
                    continue
                if abs(float(np.dot(P[a].axis(), P[b].axis()))) < 0.999:
                    continue
                shift[b] = -shift[a] * P[a].bp.teeth / P[b].bp.teeth
                todo.append(b)
        for nm, dlt in shift.items():
            out[nm] = (out.get(nm, P[nm].inst.spin_deg) + dlt) % 360.0
    return out


# ----------------------------------------------------------------------------------------------------------------
# simulation (Isaac Sim / PhysX): called from env.run_episode after the design joints
# ----------------------------------------------------------------------------------------------------------------


def _quat(R: np.ndarray) -> tuple[float, float, float, float]:
    from .treads import _quat_from_matrix  # noqa: PLC0415

    return _quat_from_matrix(R)


def _colliders(stage, path: str) -> list[str]:
    from pxr import Usd, UsdPhysics  # noqa: PLC0415

    prim = stage.GetPrimAtPath(path)
    if not prim:
        return []
    return [str(p.GetPath()) for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]


def author_racks(stage, root: str, design, merged: dict | None = None, log=print) -> list[dict]:
    """Author every slide: the prismatic joint (guide body -> rack-side body, limits at the travel ends), the
    rack-and-pinion joint (pinion shaft bearing <-> slide) and the collider filters. Returns the monitor specs."""
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics  # noqa: PLC0415

    from .design import place  # noqa: PLC0415
    from .env import JOINT_SCOPE, joint_path  # noqa: PLC0415

    merged = merged or {}
    P = place(design)
    body = lambda nm: merged.get(nm, nm)  # noqa: E731
    ppath = lambda nm: f"{root}/{merged[nm]}/{nm}" if nm in merged else f"{root}/{nm}"  # noqa: E731
    out = []
    for g in rack_geoms(design, P):
        if g.errors or g.bearing is None:
            log(f"[RACK] slide {g.slide.name} skipped: {g.errors or 'no pinion bearing'}")
            continue
        tr, _, _ = travel(design, g, P)
        gb, rb = body(g.slide.guides[0]), body(g.slide.racks[0])
        # geometry: ds/dtheta [mm/rad] along e for the pinion axle turning about its +x (its bearing's hinge axis)
        R_ax = P[g.axle].R
        k_geom = float(np.dot(np.cross(R_ax[:, 0], -g.n), g.e)) * g.r_eff
        # convention of the joint model (validated on the rack rig, 2026-10-07: 8.149 mm/rad in the physical sense):
        # the prismatic axis u = +-e is the one along which ds_u / dtheta > 0, and the ratio is positive (PhysX picks
        # the velocity sense from the joint frames' geometry). Other sign choices were not validated: the early rig
        # runs were confounded by contact-friction locks (see the collider filters below)
        u_sign = 1.0 if k_geom > 0 else -1.0
        u = g.e * u_sign
        Rj = np.stack([u, g.n, np.cross(u, g.n)], axis=1)
        ref = g.o + g.e * g.x_c  # joint origin: the contact point at the design pose
        lo_u, hi_u = sorted((u_sign * tr.lo, u_sign * tr.hi))

        def local(bname, R_j=Rj, x=ref):
            pl = P[bname]
            return pl.R.T @ (x - pl.p) * 1e-3, _quat(pl.R.T @ R_j)

        sp = f"{root}/{JOINT_SCOPE}/rack_{g.slide.name}_slide"
        j = UsdPhysics.PrismaticJoint.Define(stage, sp)
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{gb}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{rb}")])
        for idx, bn in ((0, gb), (1, rb)):
            lp, lq = local(bn)
            getattr(j, f"CreateLocalPos{idx}Attr")().Set(Gf.Vec3f(*(float(v) for v in lp)))
            getattr(j, f"CreateLocalRot{idx}Attr")().Set(Gf.Quatf(float(lq[3]), Gf.Vec3f(*(float(v) for v in lq[:3]))))
        dbg = set(os.environ.get("LEGO_BENCH_RACK_DEBUG", "").split(","))  # diagnostics: nolimit, nocoll, nopinion
        if "nolimit" not in dbg:
            j.CreateLowerLimitAttr(float(lo_u) * 1e-3)
            j.CreateUpperLimitAttr(float(hi_u) * 1e-3)
        j.CreateCollisionEnabledAttr(True)  # the guide is filtered per collider below: end stops keep colliding
        PhysxSchema.PhysxJointAPI.Apply(j.GetPrim())
        ax = g.axle
        ab = body(ax)
        use_joint = RACK_MODEL == "joint" and "nopinion" not in dbg
        ratio = 0.0
        if use_joint:  # opt-in model: PhysX rack-and-pinion joint (pinion shaft <-> rack side)
            rj = PhysxSchema.PhysxPhysicsRackAndPinionJoint.Define(stage, f"{root}/{JOINT_SCOPE}/rack_{g.slide.name}_pinion")
            rj.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{ab}")])
            rj.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{rb}")])
            rj.CreateHingeRel().SetTargets([Sdf.Path(joint_path(root, g.bearing))])
            rj.CreatePrismaticRel().SetTargets([Sdf.Path(sp)])
            lp0, lq0 = local(ab, R_j=R_ax, x=P[ax].p)  # axle frame: local x = hinge axis
            rj.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in lp0)))
            rj.CreateLocalRot0Attr().Set(Gf.Quatf(float(lq0[3]), Gf.Vec3f(*(float(v) for v in lq0[:3]))))
            lp1, lq1 = local(rb)
            rj.CreateLocalPos1Attr().Set(Gf.Vec3f(*(float(v) for v in lp1)))
            rj.CreateLocalRot1Attr().Set(Gf.Quatf(float(lq1[3]), Gf.Vec3f(*(float(v) for v in lq1[:3]))))
            unit = (180.0 / math.pi) if RACK_RATIO_UNIT == "deg" else 1.0
            ratio = RACK_SIGN * unit / (abs(k_geom) * 1e-3)  # positive, with ds_u / dtheta > 0
            rj.CreateRatioAttr().Set(float(ratio))
            rj.CreateCollisionEnabledAttr(False)
        # collider filters: guides <-> rack side (the joint is the guide), pinion <-> racks (the joint is the mesh). A
        # body whose parts are all in a set is filtered as a body; a merged body (LEGO_BENCH_CLUTCH=rigid) that also holds
        # other parts (the chassis cluster with its end stops) is filtered per collider of the listed parts only
        members: dict[str, set] = {}
        for nm in P:
            members.setdefault(body(nm), set()).add(nm)

        def prims(names: set) -> list[str]:
            out_ = []
            for b_, mem in sorted(members.items()):
                sel = mem & names
                if not sel:
                    continue
                if sel == mem:
                    out_.append(f"{root}/{b_}")
                else:
                    out_ += [c for nm in sorted(sel) for c in _colliders(stage, ppath(nm))]
            return out_

        movers = {nm for nm in g.rack_side if nm in P}
        # the rack side slides along every part of the guide frame (the guides' rigid cluster) except its travel stops:
        # a frame part that touches the slide face to face rubs on it like a guide (rack rig 2026-10-07: 2x4 plates flush
        # with the column's sides locked it through PhysX contact friction); the travel sweep shows the slide meets no
        # other frame part inside its limits, and a stop keeps colliding
        stops = {k[5:] for k in (tr.lo_kind, tr.hi_kind) if k.startswith("stop:")}
        frame = (set(g.guide_rigid) | set(g.slide.guides)) - stops - movers
        groups = [(movers, frame)] + ([(set(g.slide.racks), {g.slide.pinion})] if use_joint else [])
        # the pinion turns between parts flush with its faces (zero gap across its axis, e.g. the bearing bricks):
        # those PhysX edge contacts jammed the drive on the rig (the same effect as env.crank_bearing_pairs); filtered
        flush = flush_parts(P, g.slide.pinion) - set(g.slide.racks)
        if flush:
            groups.append(({g.slide.pinion}, flush))
        if "nocoll" in dbg:
            groups = [(movers, set(P) - movers)]
        extra = {x.split("=", 1)[1] for x in dbg if x.startswith("filter=")}  # diagnostics: filter=<part>[,filter=..]
        if extra:
            groups.append((movers, extra & set(P)))
        n_t = 0
        for src_names, dst_names in groups:
            dst = [Sdf.Path(x) for x in prims(dst_names)]
            for src in prims(src_names):
                rel = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(src)).CreateFilteredPairsRel()
                for x in dst:
                    rel.AddTarget(x)
                n_t += len(dst)
        if "contacts" in dbg:  # diagnostics: log the contacts of the rack side's bodies with anything
            _debug_contacts(stage, root, sorted({body(m) for m in movers}), log)
        log(f"[RACK] slide {g.slide.name}: guide body {gb}, rack body {rb}, pinion shaft {ab}, travel "
            f"{tr.lo:+.1f}..{tr.hi:+.1f} mm ({tr.lo_kind} / {tr.hi_kind}), "
            f"mesh {f'joint, ratio {ratio:.1f}' if use_joint else 'teeth'} "
            f"(ds/dtheta {k_geom:+.3f} mm/rad), {n_t} filtered pairs")
        out.append({"name": g.slide.name, "rack": g.slide.racks[0], "guide": g.slide.guides[0], "axle": ax,
                    "e_guide": P[g.slide.guides[0]].R.T @ g.e,
                    "rel0": P[g.slide.guides[0]].R.T @ (P[g.slide.racks[0]].p - P[g.slide.guides[0]].p) * 1e-3,
                    "lo": tr.lo, "hi": tr.hi, "lo_kind": tr.lo_kind, "hi_kind": tr.hi_kind, "k_geom": k_geom})
    return out


def flush_parts(P: dict, name: str, gap_mm: float = 0.3) -> set[str]:
    """Parts whose faces lie flush (gap < ``gap_mm``) against a gear's two faces across its axis and that overlap it
    in the other two directions (they can only slide on its faces)."""
    pl = P[name]
    d = pl.axis()
    if np.max(np.abs(d)) < 0.999:
        return set()
    k = int(np.argmax(np.abs(d)))
    lo, hi = pl.aabb()
    others = [i for i in range(3) if i != k]
    out = set()
    for n, q in P.items():
        if n == name:
            continue
        qlo, qhi = q.aabb()
        if np.any(qlo[others] > hi[others]) or np.any(lo[others] > qhi[others]):
            continue
        if abs(qlo[k] - hi[k]) < gap_mm or abs(lo[k] - qhi[k]) < gap_mm:
            out.add(n)
    return out


_CONTACT_SUBS: list = []


def _debug_contacts(stage, root: str, bodies: list[str], log) -> None:
    """Diagnostics (LEGO_BENCH_RACK_DEBUG=contacts): every 100 contact reports, log the summed impulse per collider pair
    that involves a rack-side body."""
    from omni.physx import get_physx_simulation_interface  # noqa: PLC0415
    from pxr import PhysicsSchemaTools, PhysxSchema  # noqa: PLC0415

    from isaaclab.app.settings_manager import get_settings_manager  # noqa: PLC0415

    get_settings_manager().set_bool("/physics/disableContactProcessing", False)
    paths = [f"{root}/{b}" for b in bodies]
    for pth in paths:
        api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(pth))
        api.CreateThresholdAttr().Set(0.0)
    acc: dict = {}
    n = [0]

    def cb(headers, data):
        n[0] += 1
        for h in headers:
            a = str(PhysicsSchemaTools.intToSdfPath(h.collider0))
            b = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
            if not any(a.startswith(p + "/") or b.startswith(p + "/") for p in paths):
                continue
            imp = 0.0
            for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                im = data[i].impulse
                imp += float(np.linalg.norm([im[0], im[1], im[2]]))
            key = (a.replace(root + "/", ""), b.replace(root + "/", ""))
            acc[key] = acc.get(key, 0.0) + imp
        if n[0] % 100 == 0 and acc:
            top = sorted(acc.items(), key=lambda kv: -kv[1])[:6]
            log("[RACK] contacts (impulse N s over 100 steps): " + "; ".join(f"{k[0]} | {k[1]}: {v:.4f}" for k, v in top))
            acc.clear()

    _CONTACT_SUBS.append(get_physx_simulation_interface().subscribe_contact_report_events(cb))


def _qmat(q) -> np.ndarray:
    from .treads import _qmat as qm  # noqa: PLC0415

    return qm(q)


class RackMonitor:
    """Slide positions during a run (from the part poses) and the ran-off-the-pinion failure."""

    def __init__(self, specs: list[dict]):
        self.specs = specs
        self.trace = {s["name"]: {"min": None, "max": None, "now": None} for s in specs}

    def position(self, spec: dict, parts_now: dict) -> float:
        pg, qg = parts_now[spec["guide"]]
        pr, _ = parts_now[spec["rack"]]
        rel = _qmat(qg).T @ (np.asarray(pr) - np.asarray(pg))
        return float(np.dot(rel - spec["rel0"], spec["e_guide"])) * 1e3

    def update(self, parts_now: dict) -> str | None:
        fail = "" if "noranout" in os.environ.get("LEGO_BENCH_RACK_DEBUG", "") else None  # diagnostics: no failure
        for s in self.specs:
            x = self.position(s, parts_now)
            tr = self.trace[s["name"]]
            tr["now"] = round(x, 2)
            tr["min"] = round(x, 2) if tr["min"] is None else min(tr["min"], round(x, 2))
            tr["max"] = round(x, 2) if tr["max"] is None else max(tr["max"], round(x, 2))
            if fail is None:
                if s["hi_kind"] == "mesh_end" and x >= s["hi"] - RAN_OUT_MM or s["lo_kind"] == "mesh_end" and x <= s["lo"] + RAN_OUT_MM:
                    fail = f"broke (rack {s['name']} ran off its pinion at s = {x:+.1f} mm)"
        return fail or None

    def result(self) -> dict:
        return {s["name"]: dict(self.trace[s["name"]], travel_mm=[s["lo"], s["hi"]], ends=[s["lo_kind"], s["hi_kind"]])
                for s in self.specs}
