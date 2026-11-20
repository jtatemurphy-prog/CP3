# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Round 4b of the inventory part library (2026-10-14): shock absorbers, the CV joint pair, the chain, the wide tread
(in :mod:`.treads`), the differentials, the pullback motor and the PF train motor.

Every number below comes from the LDraw model of the part (hole / axle primitives, bounds) unless it says
"estimate"; masses are the inventory sheet's BrickLink weights. Bench geometry of the parts is in
:data:`.parts.INVENTORY_GEOMETRY` (round 4b section); this module holds their mechanisms: the validator rules, the
spin propagation and the simulator model.

Shock absorbers (731c04 6.5L hard, 2909c03 9.5L soft) -- one design part, two simulated bodies
-------------------------------------------------------------------------------------------------
* Frames (converted LDraw frame, the shock's axis = local z, the rod towards -z, end holes along y):
  - 731c04 (LDraw 76537 = 4255 cylinder + 4254 rod + 22977 extra-stiff spring): a pin hole at each end, the cylinder's
    at the origin, the rod's at z = -44 (extended; 36 compressed: stroke 8 mm).
  - 2909c03 (LDraw 74741-f1 = 2909 + 2910 + u9409 soft spring): the cylinder end has a transverse cross hole along y
    offset 8 mm from the axis (x = -8, z = -4) and an axial cross hole in its cap (z = -8..0); the rod end a pin hole
    along y at z = -68 (extended; 52 compressed, 74741-f2: stroke 16 mm).
* Design: the shock is ONE part, drawn extended. Its ends connect like any part's holes: a pin in an end hole
  (``pin``, the pin the child), an axle turning in it (``hinge``), an axle through the 9.5L's cross hole (``cross``,
  the shock the child). An end hole is a round bearing: a pin in it is a revolute joint about the pin
  (``design.pivot_connections``), so the shock swings at both ends like the real one.
* Simulation: the design part is the cylinder body (LDraw composite ``<id>_cyl``: cylinder + spring); a second body,
  the rod (``<id>_rod``, origin at the rod-end hole), sits at the extended rod-end position. The connections into the
  rod-end hole are authored on the rod body. A prismatic joint along the shock axis joins them: travel q = compression
  0..stroke (hard limits), a linear spring drive F = -(F0 + k q) (preload F0 against the extended stop), light
  damping. Rod / cylinder collisions are filtered.
  - 731c04: k = 1.0 N/mm, F0 = 4.7 N (12.7 N at full compression; the user's figure, split preload / rate: an
    estimate); 2909c03: k = 0.4 N/mm, F0 = 1.5 N (7.9 N at full compression; estimate).
  - Damping (``damping_n_s_per_m``): 731c04 0.5 N s/m, 2909c03 1.5 N s/m (no hydraulic damper; the rod's friction in
    the cylinder makes a bounce die out; see SHOCK_DAMPING_N_S_PER_M for the sweep).
  - Rod mass: the shock's weight x the rod's share of the LDraw hull volume (0.33 / 0.30), the cylinder body the
    rest.
* Judge: the shock and the parts behind it are a mechanism (the pins / hinges at its ends turn, the rod side moves):
  a compressing shock is not "broke". The rod body is not a design part.

CV joint (32494 ball + 92906 cup)
---------------------------------
* 32494 (axis local y): the ball centre at the origin, a tube with a cross hole y = 8..20 (``hub_mm``): it is
  cross-fitted on an axle (kind ``bush``). 92906 (bench frame, kind ``axle``): its integral axle along local x
  (x = -6..6, LDraw y = -20..-8), the cup at x < -6, the ball seat (LDraw origin) at x = -14.
* Design field ``cv_joints``: ``[{"name", "ball": <32494>, "cup": <92906>}]``. Validator: the ball centre within
  0.5 mm of the cup's seat, the bend (between the two shafts' drive lines) <= 35 deg, two different shafts, each turning
  in a bearing.
* Kinematics / simulation: the two shafts turn 1 : 1 about their own drive lines (a PhysX gear joint between the two
  bearings, ratio 1 with the axis signs) plus a spherical joint at the ball centre; ball / cup collisions filtered.

Chain (3711, 100 links)
-----------------------
* Design field ``chains``: ``[{"name", "gears": [a, b], "links": N}]``: two coplanar spur gears on parallel axles
  (16T 94925 / 4019, 24T 3648, 40T 3649, 14T 69762; not the 8T, not the round-bore clutch gears).
* Pitch 6.4 mm = 2 teeth: a z-tooth gear wraps z / 2 links, the chain's pitch circle r = 6.4 / (2 sin(2 pi / z))
  (8.36 / 12.36 / 20.46 / 7.38 mm). Loop: the open-belt length round the two pitch circles at centre distance C,
  L = 2 sqrt(C^2 - (ra - rb)^2) + pi (ra + rb) + 2 (ra - rb) asin((ra - rb) / C); N x 6.4 >= L with at most half a link
  of slack. The gears must not mesh: C >= (za + zb) / 2 + 2 mm. Both gears cross-fitted, their bearings in one rigid
  structure; nothing else in the chain's path (r_pitch +-2 mm, +-3.9 mm across the gear plane).
* Simulation: no chain bodies: a gear joint between the two gears' shafts, ratio za : zb, same sense; the links' mass
  (0.09 g each) is split onto the two gears' bodies. Counts 3711 pieces. The links are drawn: visual-only boxes along
  the loop, advanced at the chain speed (:class:`LoopVisuals`, also for every tread loop).
* Chain as a TRACK (touching the ground, carrying the vehicle): a ``treads`` entry with ``"link": "3711"`` round two
  equal spur gears (24T or 40T; :mod:`.treads`): the tread roller model with the chain's envelope (pitch r + 2 mm).

Wide tread (57518 on 57519): :mod:`.treads` (``"link": "57518"``).

Differentials
-------------
* 62821b (old one-piece differential, LDraw 62821; axis local y): case r 10 over y = -11..7, the 28T bevel ring
  at y = 8..12 (tips r 14.4; its pitch plane taken at y = +10), round pin holes along y at both ends (y = +-8). Inside,
  two 12T bevels (6589) on the output axles and one or two 12T satellites on the case's inner tabs.
* Design field ``differentials``: ``[{"name", "case": <62821b>, "outputs": [<6589>, <6589>],
  "satellites": [<6589>, ...]}]``. The outputs are cross-fitted on two axles that run in the case's end holes (``hinge``,
  the case the parent) and in the chassis; the satellites (1-2) are parts with no connections (they sit on the inner
  tabs). Validator: the output gears at 6 mm (+-1) each side of the case centre on its axis, the satellites at 6 mm
  (+-1) from the centre across it, axes radial; the axles coaxial with the case.
* The ring meshes (a ``gear_meshes`` entry with the case): like a 28T double bevel: parallel with a 12T / 20T / 36T
  double bevel (32270 / 32269 / 32498) at (28 + z) / 2 mm (-0.15 / +0.6), the gear's mid-plane within 2.5 mm of the
  ring's pitch plane; at right angles with a 12T / 20T bevel (6589, 32198, 32270, 32269), axes intersecting
  (<= 1 mm), the pinion's centre 14 mm from the intersection and the ring plane z / 2 mm from it (+-1 mm) towards the
  case centre (a 20T crosses the case axis at its centre).
* Kinematics: w_L + w_R = 2 w_case about the case axis (validator / stats: both outputs at the case speed).
* Simulation: the case turns on a hidden revolute in the bearing of an output axle (the hinge of the ring's gear
  joint); the ring meshes are gear joints; the satellites are spawned fixed to the case. The 3-way constraint is a
  per-step ANGLE coupling on the two axle-in-case revolutes: each output's turn relative to the case is tracked from
  the poses (th_L, th_R, exact), the spider angle phi = (th_R - th_L) / 2, and the two drives pull their joints to
  -phi / +phi with stiffness K = 2 N m/rad (and a light damping 0.005 N m s/rad at -/+ w_s): the torques are
  tau_L = tau_R = -K (th_L + th_R) / 2, i.e. equal, reacted by the case, zero when w_L + w_R = 2 w_case, while the
  difference (phi) follows freely (one physics step of lag: ~0.004 N m s/rad of drag on the wheel-speed difference).
  A first version coupled velocities (targets -/+ w_s); its one-step lag acted as a large flywheel on the difference
  and, with a kinematic body in the loop (held rigs, the rigid-clutch base), it bound the outputs at 0.76 x the case.
  PhysX gear joints are two-body rows on absolute angular velocities with axes fixed in one body; the satellite's mesh
  axis is fixed in no single body, so no joint arrangement gives the 3-way relation exactly.
* 65414 (new-style housing; its ring 65413 is not owned): a plain part (round pin hole along y at the closed end),
  not a working differential.

Pullback motor 12787c01 (LDraw 12799)
-------------------------------------
* Bench frame (output along y): body x = -43.6..6.4, y = -12..12, z = -11.6..27.6; the cross output bore through the
  whole width at the origin (LDraw u9029: 11.6 mm above the bottom, 6.4 mm from the end); pin holes along y (LDraw
  12787s03) at x = -40..-8 (z = -8) and x = 0, -8 (z = 24), usable from both faces.
* A spring motor wound by hand before the run and released at t = 0 (after the settle; until then it holds like a
  braked motor). It is a ``motor`` connection (an axle through its bore) but not a PF motor: no IR, no lead, no
  battery, not counted by ``design.wheel_motors`` (the "exactly one wheel-driving PF motor" rule), ignores the motor
  program (a command for it is a validator error; ``"all"`` skips it).
* Design field ``pullback`` (optional): ``[{"motor": <name>, "wind_turns": 0..10, "sense": 1 | -1}]``: how far it is
  wound (output turns; default 10, a full wind) and the sense it drives about its output axis (default: the one that
  rolls its wheels forward, else +1). It is a stored-energy source for any mechanism on its output shaft. Until the
  release its output is held (ratchet / hand: up to 1 N m).
* Torque: tau = tau_s (1 - theta / theta_max) (1 - w / w0) for 0 <= w < w0 (tau_s 0.05 N m, theta_max 10 output turns,
  w0 600 rpm: ``catalog.INVENTORY_MOTOR_LINES``, estimates); it freewheels above w0 and when spent; turned backwards it
  winds up again (theta decreases, capped at a full wind). It drives the sense that rolls its wheels forward (or
  +y without wheels). Simulated per step on the motor joint's drive.

PF train motor 87574c01
-----------------------
* Bench frame (axles along y): body x = -43..43, y = -16..16, z = -23.6..5.6; two integral 32 mm axles along y at
  x = +-24, z = -16.8 (``male_axles``), both driven through internal gears; a 2 x 2 pivot plate on top (studs at
  z = 5.6; the bogie's turning is not modelled); its lead: the M-motor's LDraw cable (224 mm).
* Line at the wheel axles: 1900 rpm, 0.036 N m stall (Philo), 57.1 g. Wheels 55423c01 cross-fit on the axle ends (the
  motor the parent); on a flat floor they ride on their ABS flanges.
* A PF motor: IR-controlled, plugs into a receiver / box, counts as a wheel-driving PF motor when it carries wheels.
  In a design it has no ``motor`` connection: its integral axles are its outputs (``design.motor_connections`` adds a
  virtual one).
* Simulation: two hidden axle bodies (2 g each, taken from the body) on revolutes in the motor, each with half the
  motor line as its drive, coupled 1 : 1 by a gear joint; the parts on an axle end are fixed to its axle body.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD_DIR = os.path.join(HERE, "assets", "data", "lego_bench_parts")

# ----------------------------------------------------------------------------------------------------------------
# parts
# ----------------------------------------------------------------------------------------------------------------
SHOCKS: dict[str, dict] = {
    "731c04": dict(rod_hole=(0.0, 0.0, -44.0), stroke_mm=8.0, rate_n_per_mm=1.0, preload_n=4.7, rod="731c04_rod",
                   damping_n_s_per_m=0.5, title="6.5L hard spring"),
    "2909c03": dict(rod_hole=(0.0, 0.0, -68.0), stroke_mm=16.0, rate_n_per_mm=0.4, preload_n=1.5, rod="2909c03_rod",
                    damping_n_s_per_m=1.5, title="9.5L soft spring"),
}
"""Shock absorbers: the rod-end hole (extended, shock frame), stroke, spring rate and preload (see the docstring)."""
SHOCK_DAMPING_N_S_PER_M = os.environ.get("LEGO_BENCH_SHOCK_DAMPING")  # override of the per-shock damping [N s/m]
"""2026-10-14 (user): a real shock has no hydraulic damper, but its rod rubs in the cylinder and the spring in its
seats, so a bounce dies out. Viscous damping on the spring drive: 6.5L 0.5 N s/m, 9.5L 1.5 N s/m. Swept on the lever
rigs under the realistic flags (0.05 / 0.3 / 0.5 / 1 / 1.5 / 2 / 4): at 0.05 (the first value) the bounce lost ~3 % per
0.5 s; more damping settles faster but drags the resting compression away from the static balance (the rod body is
0.7-1.2 g, so a large drive damping stiffens the solve): 2 N s/m is 0.75 mm (6.5L) / 0.8 mm (9.5L) off, 4 N s/m sticks.
The chosen values keep the rest within the rigs' 0.3 mm / 15 % check; the 9.5L's bounce halves every ~0.8 s, the
6.5L's every ~3 s (its lever rig rings in the pivot too)."""
SHOCK_AXIS = np.array([0.0, 0.0, 1.0])
"""Compression direction of the rod in the shock frame (the rod moves +z towards the cylinder)."""

CV_BALL, CV_CUP = "32494", "92906"
CV_SEAT = (-14.0, 0.0, 0.0)
"""The ball seat of the cup (92906 bench frame): the LDraw origin of both parts (the ball centre)."""
CV_MAX_ANGLE_DEG = 35.0
CV_SEAT_TOL_MM = 0.5

CHAIN_LINK = "3711"
CHAIN_PITCH_MM = 6.4
CHAIN_LINK_MASS_G = 0.09
CHAIN_GEARS = {"94925": 16, "4019": 16, "3648": 24, "3649": 40, "69762": 14}
"""Gears a 3711 chain runs on (the user's list: 16T, 24T, 40T, 14T; not the 8T, not the round-bore clutch gears)."""
CHAIN_CLEARANCE_MM = 2.0
CHAIN_TIGHT_MM = 2.0
"""(user 2026-10-28, real build) how much shorter than the ideal loop a chain may be: the links' play takes it up. The
real Sol bridge L1 car ran perfectly at -1.8 mm (motor on a Technic brick) and jammed at +4.5 mm (two 4x4 plates)."""
CHAIN_HALF_W, CHAIN_HALF_T = 3.9, 2.0
"""The chain's path for the clearance check: +-3.9 mm across the gear plane, +-2 mm about the pitch line (LDraw)."""

DIFF_CASE = "62821b"
DIFF_GEAR = "6589"
DIFF_RING_TEETH = 28
DIFF_RING_Y = 10.0
"""Pitch plane of the 28T ring in the case frame (LDraw: ring teeth at y = 8..12, tips r 14.4)."""
DIFF_GEAR_OFFSET_MM = 6.0
DIFF_TOL_MM = 1.0
DIFF_RING_PARALLEL = {"32270": 12, "32269": 20, "32498": 36}
DIFF_RING_RIGHT = {"6589": 12, "32198": 20, "32270": 12, "32269": 20}
DIFF_RING_PLANE_TOL_MM = 2.5
DIFF_COUPLING_NM_PER_RAD = float(os.environ.get("LEGO_BENCH_DIFF_K", "2.0"))
"""Stiffness of the differential's angle coupling (common-mode lag tau / K: 0.05 rad at 0.1 N m)."""
DIFF_COUPLING_NMS_PER_RAD = float(os.environ.get("LEGO_BENCH_DIFF_D", "0.005"))
"""Its light velocity damping (numerical; also the drag on the outputs' difference: 0.1 N m at 20 rad/s)."""
DIFF_MAX_TORQUE_NM = 5.0
DIFF_HOUSING = "65414"

PULLBACK = "12787c01"
PULLBACK_HOLD_NM = 1.0
PULLBACK_HOLD_CREEP_DEG_S = 0.5
"""Before its release the wound pullback's output is held (its ratchet / the owner's hand): up to 1 N m, creeping
0.5 deg/s at that torque."""
TRAIN_MOTOR = "87574c01"
TRAIN_AXLE_MASS_G = 2.0
"""Hidden axle bodies of the train motor (estimate: axle + its internal gear), taken from the motor body's mass."""

M_X_TO_Y = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
"""bench (x, y, z) = (y, -x, z) of the USD frame: the USD x axis (an output / axle along x) -> bench -y."""
M_CUP = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
"""92906: bench x = -USD y (its integral axle, LDraw y = -20..-8, along +x), bench y = USD x."""
BENCH_FRAMES: dict[str, tuple[np.ndarray, tuple[float, float, float]]] = {
    TRAIN_MOTOR: (M_X_TO_Y, (0.0, 0.0, 0.0)),
    PULLBACK: (M_X_TO_Y, (0.0, 0.0, 0.0)),
    CV_CUP: (M_CUP, (-14.0, 0.0, 0.0)),
}
"""Round-4b parts rotated (and shifted) into the bench frame: ``v_bench = M v_usd + t`` (mm)."""


def usd_path(pid: str) -> str:
    return os.path.join(USD_DIR, f"{pid}.usda")


def write_usds(only=None) -> list[str]:
    """Copy each converted USD of :data:`BENCH_FRAMES` into the bench frame (like ``inventory_r4.write_usds``, plus a
    translation)."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: PLC0415

    from ai_pd_tasks.lego.catalog import PART_TYPES  # noqa: PLC0415

    os.makedirs(USD_DIR, exist_ok=True)
    out = []
    for pid, (M, t) in BENCH_FRAMES.items():
        if only and pid not in only:
            continue
        m = Gf.Matrix3d(*M.T.flatten().tolist())
        q_rot = m.ExtractRotation().GetQuat()
        layer = Usd.Stage.Open(PART_TYPES[pid].usd_path).Flatten()
        dst = usd_path(pid)
        layer.Export(dst)
        st = Usd.Stage.Open(dst)
        part = st.GetPrimAtPath("/Part")
        for child in ("Visual", "Collision"):
            x = UsdGeom.Xformable(st.GetPrimAtPath(f"/Part/{child}"))
            x.ClearXformOpOrder()
            x.AddTranslateOp().Set(Gf.Vec3d(*(v * 1e-3 for v in t)))
            x.AddOrientOp().Set(Gf.Quatf(q_rot))
        mass = UsdPhysics.MassAPI(part)
        com = mass.GetCenterOfMassAttr().Get()
        if com is not None:
            c = M @ np.array([com[0], com[1], com[2]]) + np.array(t) * 1e-3
            mass.GetCenterOfMassAttr().Set(Gf.Vec3f(*c.tolist()))
        pa = mass.GetPrincipalAxesAttr().Get()
        if pa is not None:
            mass.GetPrincipalAxesAttr().Set(Gf.Quatf(q_rot * Gf.Quatd(pa)))
        st.GetRootLayer().Save()
        out.append(dst)
    return out


def is_shock(pid: str) -> bool:
    return pid in SHOCKS


def is_pullback(pid: str) -> bool:
    return pid == PULLBACK


def is_train_motor(pid: str) -> bool:
    return pid == TRAIN_MOTOR


# ----------------------------------------------------------------------------------------------------------------
# design format
# ----------------------------------------------------------------------------------------------------------------


def _strict(d: dict, keys: set, what: str) -> None:
    unknown = set(d) - keys
    if unknown:
        raise ValueError(f"unknown {what} fields: {sorted(unknown)}")


@dataclass
class DesignChain:
    """A 3711 chain loop of ``links`` links round two gears (part names)."""

    name: str
    gears: tuple[str, str]
    links: int

    def to_dict(self) -> dict:
        return {"name": self.name, "gears": list(self.gears), "links": int(self.links)}

    @classmethod
    def from_dict(cls, d: dict) -> DesignChain:
        _strict(d, {"name", "gears", "links"}, "chain")
        return cls(str(d["name"]), tuple(str(g) for g in d["gears"]), int(d["links"]))


@dataclass
class DesignCV:
    """A CV joint: the ball (32494) seated in the cup (92906)."""

    name: str
    ball: str
    cup: str

    def to_dict(self) -> dict:
        return {"name": self.name, "ball": self.ball, "cup": self.cup}

    @classmethod
    def from_dict(cls, d: dict) -> DesignCV:
        _strict(d, {"name", "ball", "cup"}, "cv joint")
        return cls(str(d["name"]), str(d["ball"]), str(d["cup"]))


@dataclass
class DesignDiff:
    """A differential: the case (62821b), its two output 12T bevels and its 1-2 satellite 12T bevels (part names)."""

    name: str
    case: str
    outputs: tuple[str, str]
    satellites: tuple[str, ...] = field(default_factory=tuple)

    def to_dict(self) -> dict:
        return {"name": self.name, "case": self.case, "outputs": list(self.outputs), "satellites": list(self.satellites)}

    @classmethod
    def from_dict(cls, d: dict) -> DesignDiff:
        _strict(d, {"name", "case", "outputs", "satellites"}, "differential")
        return cls(str(d["name"]), str(d["case"]), tuple(str(g) for g in d["outputs"]),
                   tuple(str(s) for s in d.get("satellites", [])))


@dataclass
class DesignPullback:
    """How far a pullback motor (12787c01) is wound before the run (output turns, 0..10; default: fully) and, optionally,
    the sense it drives: +1 / -1 about the motor's output axis (its local y); default: the sense that rolls its wheels
    forward, else +1."""

    motor: str
    wind_turns: float = 10.0
    sense: int = 0

    def to_dict(self) -> dict:
        d = {"motor": self.motor, "wind_turns": float(self.wind_turns)}
        if self.sense:
            d["sense"] = int(self.sense)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> DesignPullback:
        _strict(d, {"motor", "wind_turns", "sense"}, "pullback")
        return cls(str(d["motor"]), float(d.get("wind_turns", 10.0)), int(d.get("sense", 0)))


def check_pullbacks(design, P) -> tuple[list[str], list[str], dict]:
    from ai_pd_tasks.lego.catalog import INVENTORY_MOTOR_LINES  # noqa: PLC0415

    E, W, stats = [], [], {}
    entries = getattr(design, "pullback", None) or []
    seen = set()
    for e in entries:
        tag = f"pullback {e.motor}"
        if e.motor not in P or not is_pullback(P[e.motor].inst.part):
            E.append(f"{tag}: not a pullback motor (12787c01) of the design")
            continue
        if e.motor in seen:
            E.append(f"{tag}: listed twice")
        seen.add(e.motor)
        cap = INVENTORY_MOTOR_LINES[PULLBACK]["stored_turns"]
        if not 0.0 <= e.wind_turns <= cap:
            E.append(f"{tag}: wind_turns {e.wind_turns:g} outside 0..{cap:g} (output turns of a full wind)")
        if e.sense not in (0, 1, -1):
            E.append(f"{tag}: sense must be 1 or -1 (about the motor's output axis)")
    for n, pl in P.items():
        if is_pullback(pl.inst.part):
            e = next((x for x in entries if x.motor == n), None)
            stats[n] = {"wind_turns": e.wind_turns if e else INVENTORY_MOTOR_LINES[PULLBACK]["stored_turns"],
                        "sense": (e.sense if e and e.sense else "default"),
                        "driven": any(c.kind == "motor" and c.parent == n for c in design.connections)}
    return E, W, stats


def chain_pieces(design) -> int:
    return sum(int(c.links) for c in (getattr(design, "chains", None) or []))


def chain_mass_g(design) -> float:
    return chain_pieces(design) * CHAIN_LINK_MASS_G


def internal_parts(design) -> set[str]:
    """Parts attached through a round-4b entry rather than a connection: the differentials' satellites."""
    return {s for d in (getattr(design, "differentials", None) or []) for s in d.satellites}


def extra_edges(design) -> list[tuple[str, str]]:
    """Connectivity edges of the round-4b entries (chains, CV joints, differentials) for the 'every part attached'
    check (not rigid)."""
    out = []
    for c in getattr(design, "chains", None) or []:
        if len(c.gears) == 2:
            out.append(tuple(c.gears))
    for cv in getattr(design, "cv_joints", None) or []:
        out.append((cv.ball, cv.cup))
    for d in getattr(design, "differentials", None) or []:
        out += [(d.case, g) for g in (*d.outputs, *d.satellites)]
    return out


def overlap_exempt(design) -> set[frozenset]:
    """Pairs whose shapes may overlap: a CV ball in its cup, a differential's case, outputs, satellites and output
    axles (the case is hollow)."""
    out: set[frozenset] = set()
    for cv in getattr(design, "cv_joints", None) or []:
        out.add(frozenset((cv.ball, cv.cup)))
    for d in getattr(design, "differentials", None) or []:
        axles = [c.parent for c in design.connections if c.kind == "cross" and c.child in d.outputs]
        group = [d.case, *d.outputs, *d.satellites, *axles]
        out |= {frozenset((a, b)) for i, a in enumerate(group) for b in group[i + 1:] if a != b}
    return out


# ----------------------------------------------------------------------------------------------------------------
# shocks
# ----------------------------------------------------------------------------------------------------------------


def _rod_hole_z(spec: dict) -> float:
    return spec["rod_hole"][2]


def is_rod_hole(spec: dict, centre) -> bool:
    """A hole centre (shock frame) on the rod end (anywhere along the stroke)."""
    return float(centre[2]) <= _rod_hole_z(spec) + spec["stroke_mm"] + 1.0


def shock_compression(design, P) -> dict[str, float]:
    """{shock: compression [mm] it is drawn at}: the rod end's partner (a pin or axle along the end hole's axis, the
    shock the parent) may sit anywhere on the stroke; 0 (extended) when none is found there."""
    from .design import COAXIAL_TOL  # noqa: PLC0415

    out = {}
    for name, pl in P.items():
        spec = SHOCKS.get(pl.inst.part)
        if spec is None:
            continue
        q = 0.0
        for c in design.connections:
            if c.parent != name or c.kind not in ("pin", "hinge") or c.child not in P:
                continue
            ch = P[c.child]
            if ch.bp.kind not in ("pin", "axle") or abs(float(np.dot(ch.axis(), pl.R[:, 1]))) < 0.999:
                continue
            loc = pl.to_local(ch.p)
            z = float(loc[2]) - _rod_hole_z(spec)
            if abs(float(loc[0]) - spec["rod_hole"][0]) < COAXIAL_TOL and -COAXIAL_TOL <= z <= spec["stroke_mm"] + COAXIAL_TOL:
                q = min(max(z, 0.0), spec["stroke_mm"])
                break
        out[name] = q
    return out


def place_shocks(design, P) -> None:
    """``design.place`` hook: each shock's rod-end hole (and shape) at the compression it is drawn at."""
    import dataclasses  # noqa: PLC0415

    from .parts import Box, Boxes  # noqa: PLC0415

    for name, q in shock_compression(design, P).items():
        if q <= 0.0:
            continue
        pl = P[name]
        bp = pl.bp
        holes = tuple((x, y, z + q if is_rod_hole(SHOCKS[pl.inst.part], (x, y, z)) else z, a) for x, y, z, a in bp.pin_holes)
        sh = bp.shape
        if isinstance(sh, Boxes):
            (lo, hi), rest = sh.boxes[0], sh.boxes[1:]
            boxes = (((lo[0], lo[1], lo[2] + q), hi), *rest)
            shape = Boxes((sh.lo[0], sh.lo[1], min(b[0][2] for b in boxes)), sh.hi, boxes=boxes)
        elif isinstance(sh, Box):
            shape = Box((sh.lo[0], sh.lo[1], sh.lo[2] + q), sh.hi)
        else:
            shape = sh
        P[name] = dataclasses.replace(pl, bp=dataclasses.replace(bp, pin_holes=holes, shape=shape))


def shock_sides(design, P=None) -> dict[int, tuple[str, str]]:
    """{connection index: (shock name, 'rod' | 'cyl')} of every connection into a shock's end hole (pin, hinge: the
    shock the parent; cross: the shock the child, its cylinder-end cross hole). Unresolved connections are left out
    (the validator reports them)."""
    from .design import part_holes, pin_fit, place, segment_in_holes  # noqa: PLC0415

    P = P if P is not None else place(design)
    out = {}
    for i, c in enumerate(design.connections):
        for shock, other, role in ((c.parent, c.child, "parent"), (c.child, c.parent, "child")):
            if shock not in P or other not in P or not is_shock(P[shock].inst.part):
                continue
            sp, op = P[shock], P[other]
            holes = part_holes(sp.bp)
            idx = None
            if c.kind == "pin" and role == "parent":
                fit = pin_fit(op, sp)
                idx = fit[0][1] if fit else None
            elif c.kind == "hinge" and role == "parent" and op.bp.kind == "axle":
                hits = segment_in_holes(op.p, op.axis(), 0.0, op.bp.axle_half_len, sp, min_engage=2.0)
                idx = hits[0][0] if hits else None
            elif c.kind == "cross" and role == "child" and op.bp.kind == "axle":
                hits = segment_in_holes(op.p, op.axis(), 0.0, op.bp.axle_half_len, sp, which=("axle",), min_engage=2.0)
                idx = hits[0][0] if hits else None
            if idx is None:
                continue
            out[i] = (shock, "rod" if is_rod_hole(SHOCKS[sp.inst.part], holes[idx][0]) else "cyl")
    return out


def shock_pivot_connections(design) -> set[int]:
    """Pin connections into a shock's end hole that turn (revolute about the pin): every pin connection with a shock
    as the parent, unless the pin is a frictionless pivot (3673 / 65249: ``design.pivot_connections`` decides which of
    its connections turn) or every connection of the pin goes to a shock (one stays fixed)."""
    from .parts import bench_parts  # noqa: PLC0415

    cat = bench_parts()
    parts = {p.name: p.part for p in design.parts}
    out: set[int] = set()
    by_pin: dict[str, list[int]] = {}
    for i, c in enumerate(design.connections):
        if c.kind == "pin" and c.child in parts and c.parent in parts:
            by_pin.setdefault(c.child, []).append(i)
    for pin, idx in by_pin.items():
        bp = cat.get(parts[pin])
        if bp is None or bp.pivot:
            continue
        sh = [i for i in idx if is_shock(parts[design.connections[i].parent])]
        if sh and len(sh) == len(idx):
            sh = sh[1:]
        out |= set(sh)
    return out


def check_shocks(design, P) -> tuple[list[str], list[str], dict]:
    from .design import _components, rigid_edges  # noqa: PLC0415

    E, W, stats = [], [], {}
    shocks = [n for n, pl in P.items() if is_shock(pl.inst.part)]
    if not shocks:
        return E, W, stats
    sides = shock_sides(design, P)
    rigid = _components([p.name for p in design.parts], rigid_edges(design))
    comp = {n: k for k, c in enumerate(rigid) for n in c}
    for name in shocks:
        spec = SHOCKS[P[name].inst.part]
        conns = [(i, c) for i, c in enumerate(design.connections) if name in (c.parent, c.child)]
        ends: dict[str, list[str]] = {"cyl": [], "rod": []}
        for i, c in conns:
            other = c.child if c.parent == name else c.parent
            if i not in sides:
                E.append(f"shock {name}: {c.kind} {c.child}->{c.parent} is not in one of its end holes (a pin / axle in "
                         f"an end pin hole, the shock the parent; an axle through the 9.5L's cross hole, the shock the "
                         f"child)")
                continue
            if c.kind == "stud":
                E.append(f"shock {name}: it has no studs")
                continue
            ends[sides[i][1]].append(other)
        if not ends["cyl"] or not ends["rod"]:
            W.append(f"shock {name}: its {'rod' if not ends['rod'] else 'cylinder'} end is not connected (the spring "
                     "acts on nothing)")
        else:
            ca = {comp.get(n) for n in ends["cyl"]}
            cb = {comp.get(n) for n in ends["rod"]}
            if ca & cb - {None}:
                W.append(f"shock {name}: both ends are joined to one rigid structure: it cannot compress")
        q0 = shock_compression(design, P).get(name, 0.0)
        stats[name] = {"part": P[name].inst.part, "drawn_compression_mm": round(q0, 2),
                       "force_drawn_n": round(spec["preload_n"] + spec["rate_n_per_mm"] * q0, 2),
                       "stroke_mm": spec["stroke_mm"], "rate_n_per_mm": spec["rate_n_per_mm"],
                       "preload_n": spec["preload_n"],
                       "force_full_n": round(spec["preload_n"] + spec["rate_n_per_mm"] * spec["stroke_mm"], 2),
                       "rod_end": ends["rod"], "cylinder_end": ends["cyl"]}
    return E, W, stats


# ----------------------------------------------------------------------------------------------------------------
# CV joints
# ----------------------------------------------------------------------------------------------------------------


def cv_lines(P, cv: DesignCV) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """(ball centre, cup seat, drive line of the ball's shaft (towards the cup), drive line of the cup's shaft (away
    from the ball)), world."""
    b, c = P[cv.ball], P[cv.cup]
    return b.p.copy(), c.to_world(CV_SEAT), -b.R[:, 1], c.R[:, 0]


def check_cvs(design, P, sh) -> tuple[list[str], list[str], dict]:
    E, W, stats = [], [], {}
    seen: dict[str, str] = {}
    for cv in getattr(design, "cv_joints", None) or []:
        tag = f"cv joint {cv.name}"
        if cv.ball not in P or cv.cup not in P:
            E.append(f"{tag}: unknown part")
            continue
        if P[cv.ball].inst.part != CV_BALL or P[cv.cup].inst.part != CV_CUP:
            E.append(f"{tag}: the ball must be a 32494 and the cup a 92906")
            continue
        for n in (cv.ball, cv.cup):
            if n in seen:
                E.append(f"{tag}: {n} is already in cv joint {seen[n]}")
            seen[n] = cv.name
        pb, pc, ua, ub = cv_lines(P, cv)
        gap = float(np.linalg.norm(pb - pc))
        if gap > CV_SEAT_TOL_MM:
            E.append(f"{tag}: the ball is not seated in the cup ({gap:.2f} mm from the cup's ball seat, cup-local "
                     f"{CV_SEAT}; <= {CV_SEAT_TOL_MM} mm)")
        ang = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(ua, ub))))))
        if ang > CV_MAX_ANGLE_DEG + 1e-6:
            E.append(f"{tag}: bent {ang:.1f} deg (at most {CV_MAX_ANGLE_DEG:g} deg)")
        sa, sb = sh.get(cv.ball), sh.get(cv.cup)
        if sa is None:
            E.append(f"{tag}: the ball {cv.ball} must be cross-fitted on an axle")
        elif sa == sb:
            E.append(f"{tag}: the ball and the cup are on the same shaft")
        stats[cv.name] = {"angle_deg": round(ang, 2), "seat_gap_mm": round(gap, 3), "shafts": [sa, sb]}
    return E, W, stats


# ----------------------------------------------------------------------------------------------------------------
# chains
# ----------------------------------------------------------------------------------------------------------------


def chain_radius(teeth: int) -> float:
    """Pitch radius of the 3711 chain on a gear of ``teeth`` teeth (z / 2 links per turn)."""
    return CHAIN_PITCH_MM / (2.0 * math.sin(2.0 * math.pi / teeth))


def belt_length(ra: float, rb: float, c: float) -> float:
    """Open-belt length round two pulleys of pitch radii ra, rb at centre distance c."""
    d = ra - rb
    return 2.0 * math.sqrt(max(c * c - d * d, 0.0)) + math.pi * (ra + rb) + 2.0 * d * math.asin(max(-1.0, min(1.0, d / c)))


@dataclass
class ChainGeom:
    chain: DesignChain
    a: object
    b: object
    axis: np.ndarray
    ca: np.ndarray
    cb: np.ndarray
    D: float
    ra: float
    rb: float
    loop_mm: float
    slack_mm: float
    errors: list[str] = field(default_factory=list)

    def path_points(self, step: float = 1.5) -> np.ndarray:
        """Points filling the chain's path (both straight runs and both wraps)."""
        e = (self.cb - self.ca) / self.D
        n = np.cross(self.axis, e)
        d = self.ra - self.rb
        beta = math.asin(max(-1.0, min(1.0, d / self.D)))
        pts = []
        for r_off in (-CHAIN_HALF_T, 0.0, CHAIN_HALF_T):
            for s in (1.0, -1.0):  # the two straight runs: tangent points
                u = math.cos(beta) * n * s + math.sin(beta) * e
                pa, pb = self.ca + (self.ra + r_off) * u, self.cb + (self.rb + r_off) * u
                k = max(2, int(np.linalg.norm(pb - pa) / step) + 1)
                pts += [pa + (pb - pa) * t for t in np.linspace(0.0, 1.0, k)]
            for c, r in ((self.ca, self.ra), (self.cb, self.rb)):  # wraps (whole circles: conservative)
                k = max(8, int(2 * math.pi * r / step))
                for ang in np.linspace(0.0, 2 * math.pi, k, endpoint=False):
                    pts.append(c + (r + r_off) * (math.cos(ang) * e + math.sin(ang) * n))
        pts = np.array(pts)
        return np.concatenate([pts + w * self.axis for w in np.linspace(-CHAIN_HALF_W, CHAIN_HALF_W, 4)])


def chain_geoms(design, P) -> list:
    out = []
    for ch in getattr(design, "chains", None) or []:
        tag = f"chain {ch.name}"
        if len(ch.gears) != 2 or ch.gears[0] == ch.gears[1]:
            out.append([f"{tag}: needs two different gears"])
            continue
        if any(g not in P for g in ch.gears):
            out.append([f"{tag}: unknown gear"])
            continue
        a, b = P[ch.gears[0]], P[ch.gears[1]]
        bad = [g.inst.name for g in (a, b) if g.inst.part not in CHAIN_GEARS]
        if bad:
            out.append([f"{tag}: {bad[0]} ({P[bad[0]].inst.part}) does not take the 3711 chain (16T 94925 / 4019, 24T "
                        f"3648, 40T 3649, 14T 69762)"])
            continue
        ax = a.axis() / np.linalg.norm(a.axis())
        if abs(float(np.dot(ax, b.axis()))) < 0.999:
            out.append([f"{tag}: the gear axes are not parallel"])
            continue
        d = b.p - a.p
        axial = float(np.dot(d, ax))
        dp = d - axial * ax
        D = float(np.linalg.norm(dp))
        errs = []
        if abs(axial) > 1.0:
            errs.append(f"{tag}: the gears are {abs(axial):.2f} mm apart along their axles (not in one plane)")
        if D < 1e-6:
            out.append([f"{tag}: the gears are coaxial"])
            continue
        za, zb = CHAIN_GEARS[a.inst.part], CHAIN_GEARS[b.inst.part]
        ra, rb = chain_radius(za), chain_radius(zb)
        loop = belt_length(ra, rb, D)
        out.append(ChainGeom(ch, a, b, ax, a.p.copy(), a.p + dp, D, ra, rb, loop, ch.links * CHAIN_PITCH_MM - loop,
                             errors=errs))
    return out


def check_chains(design, P, sh) -> tuple[list[str], list[str], dict]:
    from .design import OVERLAP_TOL, _components, _depth_in_shape, rigid_edges  # noqa: PLC0415

    E, W, stats = [], [], {}
    chains = getattr(design, "chains", None) or []
    # (user 2026-11-04) LEGO_MAX_CHAINS: at most this many 3711 chain loops per design (chain drives + chain tracks)
    mx = os.environ.get("LEGO_MAX_CHAINS")
    if mx:
        n_tr = sum(1 for t in (getattr(design, "treads", None) or []) if getattr(t, "link", "") == "3711")
        if len(chains) + n_tr > int(mx):
            E.append(f"{len(chains) + n_tr} chain loops (3711 chain drives + chain tracks): at most {int(mx)} per design")
    if not chains:
        return E, W, stats
    names = [c.name for c in chains]
    for n in {n for n in names if names.count(n) > 1}:
        E.append(f"duplicate chain name '{n}'")
    used = [g for c in chains for g in c.gears]
    for g in {g for g in used if used.count(g) > 1}:
        E.append(f"gear '{g}' carries more than one chain")
    rigid = _components([p.name for p in design.parts], rigid_edges(design))
    comp = {n: i for i, c in enumerate(rigid) for n in c}
    for g in chain_geoms(design, P):
        if not isinstance(g, ChainGeom):
            E.extend(g)
            continue
        ch = g.chain
        tag = f"chain {ch.name}"
        E.extend(g.errors)
        za, zb = CHAIN_GEARS[g.a.inst.part], CHAIN_GEARS[g.b.inst.part]
        if g.D < (za + zb) / 2.0 + CHAIN_CLEARANCE_MM - 1e-6:
            E.append(f"{tag}: gear centres {g.D:.1f} mm apart: the gears touch (>= {(za + zb) / 2 + CHAIN_CLEARANCE_MM:.1f} "
                     "mm; mesh them directly instead)")
        if ch.links < 1 or g.slack_mm < -CHAIN_TIGHT_MM - 1e-6:
            need = math.ceil((g.loop_mm - CHAIN_TIGHT_MM) / CHAIN_PITCH_MM - 1e-9)
            E.append(f"{tag}: {ch.links} links ({ch.links * CHAIN_PITCH_MM:.1f} mm) are shorter than the loop round the "
                     f"gears ({g.loop_mm:.1f} mm at {g.D:.1f} mm, chain pitch radii {g.ra:.2f} / {g.rb:.2f}) by more than 2 mm: needs >= "
                     f"{need} links")
        elif g.slack_mm > CHAIN_PITCH_MM / 2 + 1e-6:  # (user 2026-10-28) half a link
            E.append(f"{tag}: {g.slack_mm:.1f} mm of slack (> half a link, 3.2 mm): the chain jumps the teeth; use "
                     f"{math.ceil(g.loop_mm / CHAIN_PITCH_MM - 1e-6)} links and move the gears apart (or closer) until the slack is <= 3.2 mm")
        sa, sb = sh.get(ch.gears[0]), sh.get(ch.gears[1])
        if sa is None or sb is None:
            E.append(f"{tag}: both gears must be cross-fitted on axles")
        elif sa == sb:
            E.append(f"{tag}: both gears are on the same shaft")
        brg = []
        for s in (sa, sb):
            b = next((c for c in design.connections if c.child == s and c.kind in ("hinge", "motor")), None)
            if s is not None and b is None:
                E.append(f"{tag}: the axle '{s}' needs a hinge or motor bearing")
            brg.append(b)
        if all(b is not None for b in brg) and comp.get(brg[0].parent) != comp.get(brg[1].parent):
            E.append(f"{tag}: the two gear axles must turn in one rigid (stud / pin) structure ({brg[0].parent} and "
                     f"{brg[1].parent} are not rigidly joined: the centre distance would change)")
        pts = g.path_points()
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        skip = {n for n, s in sh.items() if s in (sa, sb)} | set(ch.gears)
        for nm, q in P.items():
            if nm in skip:
                continue
            qlo, qhi = q.aabb()
            if np.any(qlo > hi) or np.any(lo > qhi):
                continue
            depth = float(_depth_in_shape(q.bp.shape, (q.R.T @ (pts - q.p).T).T).max())
            if depth > OVERLAP_TOL:
                E.append(f"{tag}: {nm} is in the chain's path ({depth:.1f} mm)")
        stats[ch.name] = {"gears": list(ch.gears), "teeth": [za, zb], "ratio_b_per_a": round(za / zb, 4),
                          "centre_mm": round(g.D, 2), "loop_mm": round(g.loop_mm, 2), "slack_mm": round(g.slack_mm, 2),
                          "links": ch.links}
    return E, W, stats


# ----------------------------------------------------------------------------------------------------------------
# differentials
# ----------------------------------------------------------------------------------------------------------------


def diff_axles(design, d: DesignDiff) -> list[str | None]:
    """The axles the two output gears are cross-fitted on."""
    return [next((c.parent for c in design.connections if c.kind == "cross" and c.child == g), None) for g in d.outputs]


def check_diffs(design, P, sh) -> tuple[list[str], list[str], dict]:
    from .design import COAXIAL_TOL, _line_point_distance  # noqa: PLC0415

    E, W, stats = [], [], {}
    for d in getattr(design, "differentials", None) or []:
        tag = f"differential {d.name}"
        if d.case not in P or P[d.case].inst.part != DIFF_CASE:
            E.append(f"{tag}: the case must be a 62821b")
            continue
        case = P[d.case]
        a = case.axis()
        if len(d.outputs) != 2 or len(set(d.outputs)) != 2 or any(g not in P for g in d.outputs):
            E.append(f"{tag}: needs two output gears (6589)")
            continue
        if not 1 <= len(d.satellites) <= 2:
            E.append(f"{tag}: needs one or two satellite gears (6589)")
        for g in (*d.outputs, *d.satellites):
            if g in P and P[g].inst.part != DIFF_GEAR:
                E.append(f"{tag}: {g} must be a 12T bevel 6589")
        axles = diff_axles(design, d)
        if any(x is None for x in axles):
            E.append(f"{tag}: each output gear must be cross-fitted on an axle")
            continue
        if sh.get(axles[0]) == sh.get(axles[1]):
            E.append(f"{tag}: the two outputs are on the same shaft")
        offs = []
        for g, x in zip(d.outputs, axles, strict=True):
            if abs(float(np.dot(P[x].axis(), a))) < 0.999 or _line_point_distance(case.p, a, P[x].p)[0] > COAXIAL_TOL:
                E.append(f"{tag}: the output axle {x} is not on the case's axis")
            if not any(c.kind == "hinge" and c.child == x and c.parent == d.case for c in design.connections):
                E.append(f"{tag}: the output axle {x} must turn in one of the case's end holes (a hinge {x}->{d.case})")
            t = float(np.dot(P[g].p - case.p, a))
            offs.append(t)
            if abs(abs(t) - DIFF_GEAR_OFFSET_MM) > DIFF_TOL_MM or _line_point_distance(case.p, a, P[g].p)[0] > DIFF_TOL_MM:
                E.append(f"{tag}: the output gear {g} must sit on the case axis {DIFF_GEAR_OFFSET_MM:g} mm from the "
                         f"centre (+-{DIFF_TOL_MM:g}); it is at {t:+.2f} mm")
        if len(offs) == 2 and offs[0] * offs[1] >= 0:
            E.append(f"{tag}: the two output gears must sit on opposite sides of the case centre")
        for s in d.satellites:
            if s not in P:
                E.append(f"{tag}: unknown satellite '{s}'")
                continue
            r = P[s].p - case.p
            along = float(np.dot(r, a))
            rad = r - along * a
            if abs(along) > DIFF_TOL_MM or abs(float(np.linalg.norm(rad)) - DIFF_GEAR_OFFSET_MM) > DIFF_TOL_MM:
                E.append(f"{tag}: the satellite {s} must sit {DIFF_GEAR_OFFSET_MM:g} mm from the case centre, across "
                         "its axis (+-1 mm)")
            elif abs(float(np.dot(P[s].axis(), rad / np.linalg.norm(rad)))) < 0.999:
                E.append(f"{tag}: the satellite {s}'s axis must point at the case's axis")
            if any(s in (c.child, c.parent) for c in design.connections):
                E.append(f"{tag}: the satellite {s} sits on the case's inner tab: it takes no connections")
        rings = [m for m in design.gear_meshes if d.case in (m.gear_a, m.gear_b)]
        if not rings:
            W.append(f"{tag}: nothing meshes with the 28T ring (the case does not turn)")
        stats[d.name] = {"case": d.case, "axles": axles, "outputs": list(d.outputs), "satellites": list(d.satellites)}
    for n, pl in P.items():
        if pl.inst.part == DIFF_CASE and not any(d.case == n for d in getattr(design, "differentials", None) or []):
            E.append(f"{n}: a 62821b differential case must be declared in 'differentials' (case, outputs, satellites)")
    return E, W, stats


def ring_mesh_problems(tag: str, case, other) -> list[str]:
    """Validator rules of a mesh between a differential case's 28T ring and another gear (see the docstring)."""
    from .design import _line_point_distance  # noqa: PLC0415

    a, b = case.axis(), other.axis()
    pid = other.inst.part
    zr = DIFF_RING_TEETH
    ring_c = case.to_world((0.0, DIFF_RING_Y, 0.0))
    if abs(float(np.dot(a, b))) >= 0.999:
        if pid not in DIFF_RING_PARALLEL:
            return [f"{tag}: with parallel axes the 28T ring meshes with a double bevel 12T / 20T / 36T (32270 / "
                    "32269 / 32498)"]
        z = DIFF_RING_PARALLEL[pid]
        d = other.p - case.p
        axial = float(np.dot(other.p - ring_c, a))
        centre = float(np.linalg.norm(d - float(np.dot(d, a)) * a))
        errs = []
        if abs(axial) > DIFF_RING_PLANE_TOL_MM:
            errs.append(f"{tag}: {other.inst.name}'s mid-plane is {axial:+.2f} mm from the ring's pitch plane "
                        f"(<= {DIFF_RING_PLANE_TOL_MM:g}; the ring is at case-local y = {DIFF_RING_Y:g})")
        nominal = (zr + z) / 2.0
        if centre < nominal - 0.15:
            errs.append(f"{tag}: centre distance {centre:.2f} mm < nominal {nominal:.1f} mm (teeth jam)")
        elif centre > nominal + 0.6:
            errs.append(f"{tag}: centre distance {centre:.2f} mm > nominal {nominal:.1f} mm + 0.6 (teeth skip)")
        return errs
    if abs(float(np.dot(a, b))) > 1e-3:
        return [f"{tag}: the 28T ring meshes with parallel or perpendicular axes only"]
    if pid not in DIFF_RING_RIGHT:
        return [f"{tag}: at right angles the 28T ring meshes with a 12T / 20T bevel (6589, 32198, 32270, 32269)"]
    z = DIFF_RING_RIGHT[pid]
    n = np.cross(a, b)
    gap = abs(float(np.dot(other.p - case.p, n / np.linalg.norm(n))))
    errs = []
    if gap > DIFF_TOL_MM:
        errs.append(f"{tag}: the axes do not intersect ({gap:.2f} mm apart)")
    t_int = float(np.dot(other.p - case.p, a))  # the intersection on the case axis (other's centre projected)
    inter = case.p + t_int * a
    ring_t = float(np.dot(ring_c - case.p, a))
    d_ring = ring_t - t_int  # ring plane relative to the intersection (towards the ring)
    d_pin = _line_point_distance(case.p, a, other.p)[0]
    if abs(abs(d_ring) - z / 2.0) > DIFF_TOL_MM or abs(ring_t) < abs(t_int):
        errs.append(f"{tag}: the pinion's axis crosses the case axis {abs(d_ring):.2f} mm from the ring plane (nominal "
                    f"{z / 2:g} mm, towards the case centre: case-local y = {DIFF_RING_Y - z / 2:g})")
    if abs(d_pin - zr / 2.0) > DIFF_TOL_MM:
        errs.append(f"{tag}: the pinion's centre is {d_pin:.2f} mm from the case axis (nominal {zr / 2:g})")
    del inter
    return errs


# ----------------------------------------------------------------------------------------------------------------
# kinematics (design._propagate hook) and motors
# ----------------------------------------------------------------------------------------------------------------


def propagate(design, P, sh, spin: dict) -> tuple[bool, list[str]]:
    """One pass of the round-4b couplings over the shaft spins (chains, CV joints, differentials); returns (changed,
    problems)."""
    changed, problems = False, []

    def put(s, w, what):
        nonlocal changed
        if s not in spin:
            spin[s] = w
            changed = True
        elif np.linalg.norm(spin[s] - w) > 1e-6:
            problems.append(f"gear train locked: shaft {s} is driven at two different speeds ({what})")

    for ch in getattr(design, "chains", None) or []:
        if len(ch.gears) != 2 or any(g not in sh or g not in P or P[g].inst.part not in CHAIN_GEARS for g in ch.gears):
            continue
        a, b = ch.gears
        za, zb = CHAIN_GEARS[P[a].inst.part], CHAIN_GEARS[P[b].inst.part]
        for x, y, zx, zy in ((a, b, za, zb), (b, a, zb, za)):
            if sh[x] in spin and sh[x] != sh[y]:
                put(sh[y], float(np.dot(spin[sh[x]], P[x].axis())) * zx / zy * P[y].axis() * float(
                    np.sign(np.dot(P[x].axis(), P[y].axis())) or 1.0), f"chain {ch.name}")
    for cv in getattr(design, "cv_joints", None) or []:
        if cv.ball not in sh or cv.cup not in sh or cv.ball not in P or cv.cup not in P:
            continue
        _, _, ua, ub = cv_lines(P, cv)
        sa, sb = sh[cv.ball], sh[cv.cup]
        if sa == sb:
            continue
        if sa in spin:
            put(sb, float(np.dot(spin[sa], ua)) * ub, f"cv joint {cv.name}")
        if sb in spin:
            put(sa, float(np.dot(spin[sb], ub)) * ua, f"cv joint {cv.name}")
    for d in getattr(design, "differentials", None) or []:
        if d.case not in P or d.case not in sh:
            continue
        a = P[d.case].axis()
        outs = [sh.get(x) for x in diff_axles(design, d)]
        if any(o is None for o in outs) or outs[0] == outs[1]:
            continue
        sc = sh[d.case]
        if sc in spin:  # straight running: both outputs at the case speed (w_L + w_R = 2 w_case)
            wc = float(np.dot(spin[sc], a))
            for o in outs:
                if o not in spin:
                    put(o, wc * a, "")
            wl, wr = (float(np.dot(spin[o], a)) for o in outs)
            if abs(wl + wr - 2 * wc) > 1e-6:
                problems.append(f"gear train locked: differential {d.name} (w_L + w_R != 2 w_case)")
        elif all(o in spin for o in outs):
            put(sc, 0.5 * float(np.dot(spin[outs[0]] + spin[outs[1]], a)) * a, "")
    return changed, problems


def virtual_motor_connections(design) -> list:
    """Train motors (87574c01): a synthetic ``motor`` connection per motor that carries parts on its integral axles
    (child = the first part cross-fitted on it: the spin seed; ``design.motor_connections``)."""
    from .design import DesignConnection  # noqa: PLC0415

    parts = {p.name: p.part for p in design.parts}
    out = []
    for name, pid in parts.items():
        if not is_train_motor(pid):
            continue
        kid = next((c.child for c in design.connections if c.kind == "cross" and c.parent == name), None)
        if kid is not None:
            out.append(DesignConnection(kid, name, "motor"))
    return out


def train_axle_of(design, P, motor: str, part: str) -> int:
    """Index of the train motor's axle end the part is cross-fitted on."""
    from .design import _male_axle_line, male_axle_lines  # noqa: PLC0415

    p0, _, _ = _male_axle_line(P[motor], P[part])
    lines = male_axle_lines(P[motor])
    return int(np.argmin([np.linalg.norm(p0 - c) for c, _, _ in lines]))


def drive_sense(design, motor: str) -> float:
    """+1 / -1: the sense (about the motor joint's axis, the shaft's local x) a pullback drives: the one that rolls its
    wheels forward (+1 without wheels)."""
    from .design import _propagate, place, shafts  # noqa: PLC0415

    P = place(design)
    sh = shafts(design)
    conn = next((c for c in design.connections if c.kind == "motor" and c.parent == motor), None)
    if conn is None or conn.child not in P:
        return 1.0
    spin, _ = _propagate(design, P, sh, conn.child, P[conn.child].axis())
    wheels = [n for n, pl in P.items() if pl.bp.kind == "wheel" and abs(pl.axis()[2]) < 0.01 and sh.get(n) in spin]
    if not wheels:
        return 1.0
    s = float(spin[sh[wheels[0]]][1])
    return 1.0 if s >= 0 else -1.0


# ----------------------------------------------------------------------------------------------------------------
# validator entry (design.validate)
# ----------------------------------------------------------------------------------------------------------------


def check(design, P, sh) -> tuple[list[str], list[str], dict]:
    """All round-4b validator rules; (errors, warnings, stats)."""
    E, W, stats = [], [], {}
    for fn, key in ((check_shocks, "shocks"), (check_cvs, "cv_joints"), (check_chains, "chains"),
                    (check_diffs, "differentials"), (check_pullbacks, "pullbacks")):
        e, w, s = fn(design, P) if fn in (check_shocks, check_pullbacks) else fn(design, P, sh)
        E += e
        W += w
        if s:
            stats[key] = s
    for n, pl in P.items():
        if pl.inst.part == DIFF_HOUSING:
            W.append(f"{n}: the 65414 differential housing has no ring gear (65413 is not owned): a plain part")
    return E, W, stats


# ----------------------------------------------------------------------------------------------------------------
# simulation (Isaac Sim / PhysX): called from env.run_episode
# ----------------------------------------------------------------------------------------------------------------


def _quat_from_matrix(R):
    from .treads import _quat_from_matrix as q  # noqa: PLC0415

    return q(R)


def _qmat(q):
    from .treads import _qmat as m  # noqa: PLC0415

    return m(q)


def needs_sim(design) -> bool:
    """The design has a round-4b mechanism the simulator models (every earlier design: False, nothing changes)."""
    pids = {p.part for p in design.parts}
    return bool(pids & (set(SHOCKS) | {PULLBACK, TRAIN_MOTOR}) or getattr(design, "chains", None)
                or getattr(design, "cv_joints", None) or getattr(design, "differentials", None))


class R4bSim:
    """Round-4b simulation state of one episode: hidden bodies, extra joints, per-step drives."""

    def __init__(self, design, pose, merged: dict, root: str, log=print):
        from .design import DesignPart, place, shafts  # noqa: PLC0415

        self.design, self.pose, self.merged, self.root, self.log = design, pose, merged or {}, root, log
        self.P = place(design)
        self.sh = shafts(design)
        self.parts = {p.name: p for p in design.parts}
        self.aux: dict[str, DesignPart] = {}  # hidden body name -> its design-frame pose (a DesignPart)
        self.ext: dict[str, DesignPart] = {}  # shock rod -> its extended pose (the prismatic joint's zero)
        self.aux_usd: dict[str, str] = {}
        self.aux_mass: dict[str, float] = {}  # kg
        self.remap: dict[int, dict] = {}  # connection index -> {"parent" | "child": hidden body}
        self.shocks: dict[str, dict] = {}
        self.trains: dict[str, dict] = {}
        self.pullbacks: dict[str, dict] = {}
        self.diffs: list[dict] = []
        self.released = False
        self.aux_objs: dict = {}
        self.trace: dict[str, list] = {}
        sides = shock_sides(design, self.P)
        for name, pl in self.P.items():
            pid = pl.inst.part
            if is_shock(pid):
                spec = SHOCKS[pid]
                rod = f"{name}__rod"
                q0 = shock_compression(design, self.P).get(name, 0.0)
                hole = np.array(spec["rod_hole"]) + q0 * SHOCK_AXIS
                self.aux[rod] = DesignPart(rod, spec["rod"], tuple(pl.to_world(hole)), pl.inst.yaw_deg, pl.inst.spin_deg)
                self.ext[rod] = DesignPart(rod, spec["rod"], tuple(pl.to_world(spec["rod_hole"])), pl.inst.yaw_deg,
                                           pl.inst.spin_deg)
                from ai_pd_tasks.lego.catalog import PART_TYPES  # noqa: PLC0415

                self.aux_usd[rod] = PART_TYPES[spec["rod"]].usd_path
                self.aux_mass[rod] = PART_TYPES[spec["rod"]].mass_g * 1e-3
                self.shocks[name] = {"rod": rod, "spec": spec, "q_min": 0.0, "q_max": 0.0}
                for i, (s, side) in sides.items():
                    if s == name and side == "rod":
                        c = design.connections[i]
                        self.remap[i] = {"parent" if c.parent == name else "child": rod}
            elif is_train_motor(pid):
                axles = []
                for k, (x, y, z, _a, _h) in enumerate(pl.bp.male_axles):
                    ax = f"{name}__ax{k}"
                    self.aux[ax] = DesignPart(ax, pid, tuple(pl.to_world((x, y, z))), pl.inst.yaw_deg, pl.inst.spin_deg)
                    self.aux_mass[ax] = TRAIN_AXLE_MASS_G * 1e-3
                    axles.append(ax)
                seed = next((c.child for c in virtual_motor_connections(design) if c.parent == name), None)
                sgn = float(np.sign(np.dot(self.P[seed].axis(), pl.R[:, 1]))) if seed in self.P else 1.0
                self.trains[name] = {"axles": axles, "seed_sign": sgn or 1.0}
                for i, c in enumerate(design.connections):
                    if c.kind == "cross" and c.parent == name and c.child in self.P:
                        self.remap[i] = {"parent": axles[train_axle_of(design, self.P, name, c.child)]}
            elif is_pullback(pid):
                conn = next((c for c in design.connections if c.kind == "motor" and c.parent == name), None)
                if conn is not None:
                    from .design import motor_params  # noqa: PLC0415

                    prm = motor_params(pid)
                    from ai_pd_tasks.lego.catalog import INVENTORY_MOTOR_LINES  # noqa: PLC0415

                    turns = INVENTORY_MOTOR_LINES[pid]["stored_turns"]
                    entry = next((e for e in getattr(design, "pullback", None) or [] if e.motor == name), None)
                    wind = turns if entry is None else min(max(entry.wind_turns, 0.0), turns)
                    sense = drive_sense(design, name)
                    if entry is not None and entry.sense:  # about the motor's output axis (its local y)
                        sense = float(entry.sense) * float(np.sign(np.dot(self.P[conn.child].axis(), pl.R[:, 1])) or 1.0)
                    self.pullbacks[name] = {"conn": conn, "sense": sense, "theta": 2 * math.pi * (turns - wind),
                                            "w0": prm["no_load_rad_s"], "tau": prm["stall_torque_nm"],
                                            "theta_max": 2 * math.pi * turns, "wind_turns": wind,
                                            "regime": None, "w_max": 0.0}

    # -- helpers --
    def body(self, n: str) -> str:
        return self.merged.get(n, n)

    def dpart(self, n: str):
        return self.aux.get(n) or self.parts[n]

    def world_pose(self, n: str):
        """Spawn world pose (m, xyzw) of a design part or hidden body."""
        if n in self.pose.parts:
            return self.pose.parts[n]
        from .env import rel_pose  # noqa: PLC0415

        host = next(h for h in (*self.shocks, *self.trains) if n.startswith(h + "__"))
        lp, lq = rel_pose(self.parts[host], self.aux[n])
        hp, hq = self.pose.parts[host]
        from .design import qmul  # noqa: PLC0415

        return tuple(float(v) for v in np.add(hp, _qmat(hq) @ np.array(lp))), tuple(float(v) for v in qmul(hq, lq))

    def frame(self, body: str, of: str):
        """(pos, quat) of design part / hidden body ``of`` in body ``body``'s frame (for joint frames)."""
        from .env import rel_pose  # noqa: PLC0415

        return rel_pose(self.dpart(body), self.dpart(of))

    # -- spawn (before env.author_design_joints) --
    def spawn(self, stage, spawn_usd) -> None:
        """Hidden bodies: shock rods (their USD, through ``spawn_usd(path, usd, pos, rot)`` -> a RigidObject) and the
        train motors' axle bodies (bare rigid bodies, no collider)."""
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: PLC0415

        from .env import MAX_ANGULAR_VEL_DEG_S, PART_POS_ITERS, PART_VEL_ITERS  # noqa: PLC0415

        for n in self.aux:
            pos, rot = self.world_pose(n)
            path = f"{self.root}/{n}"
            if n in self.aux_usd:
                self.aux_objs[n] = spawn_usd(path, self.aux_usd[n], pos, rot)
                continue
            x = UsdGeom.Xform.Define(stage, path)
            x.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in pos)))
            x.AddOrientOp().Set(Gf.Quatf(float(rot[3]), Gf.Vec3f(*(float(v) for v in rot[:3]))))
            prim = x.GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim)
            ma = UsdPhysics.MassAPI.Apply(prim)
            m = self.aux_mass[n]
            ma.CreateMassAttr().Set(m)
            ma.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            i_sp = 0.5 * m * 3.2e-3 ** 2
            i_tr = m * (32e-3) ** 2 / 12.0
            ma.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(i_tr, i_sp, i_tr))  # axle along local y
            ma.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0))
            rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rb.CreateSolverPositionIterationCountAttr().Set(PART_POS_ITERS)
            rb.CreateSolverVelocityIterationCountAttr().Set(PART_VEL_ITERS)
            rb.CreateMaxAngularVelocityAttr().Set(MAX_ANGULAR_VEL_DEG_S)
            rb.CreateSleepThresholdAttr().Set(0.0)

    def joint_parts(self) -> dict:
        """Design parts + hidden bodies for env.author_design_joints' joint frames."""
        return dict(self.aux)

    # -- joints (after env.author_design_joints) --
    def author(self, stage) -> None:
        from pxr import Gf, PhysxSchema, Sdf, UsdPhysics  # noqa: PLC0415

        from .design import motor_params  # noqa: PLC0415
        from .env import JOINT_SCOPE, joint_path  # noqa: PLC0415

        root = self.root
        gs = float(os.environ.get("LEGO_BENCH_GEAR_SIGN", "1"))
        filt: set[tuple[str, str]] = set()

        def fixed_or_rev(kind, path, b0, b1, of, axis=None, of0=None):
            j = (UsdPhysics.RevoluteJoint if kind == "rev" else UsdPhysics.PrismaticJoint if kind == "pri"
                 else UsdPhysics.SphericalJoint if kind == "sph" else UsdPhysics.FixedJoint).Define(stage, path)
            if axis is not None:
                j.CreateAxisAttr(axis)
            if of0 is not None:  # body0's frame at another pose (the shock's extended rod: the prismatic zero)
                from .env import rel_pose  # noqa: PLC0415

                lp0, lq0 = rel_pose(self.dpart(b0), of0)
            else:
                lp0, lq0 = self.frame(b0, of)
            lp1, lq1 = self.frame(b1, of)
            j.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{b0}")])
            j.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{b1}")])
            j.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in lp0)))
            j.CreateLocalRot0Attr().Set(Gf.Quatf(float(lq0[3]), Gf.Vec3f(*(float(v) for v in lq0[:3]))))
            j.CreateLocalPos1Attr().Set(Gf.Vec3f(*(float(v) for v in lp1)))
            j.CreateLocalRot1Attr().Set(Gf.Quatf(float(lq1[3]), Gf.Vec3f(*(float(v) for v in lq1[:3]))))
            j.CreateCollisionEnabledAttr(False)
            PhysxSchema.PhysxJointAPI.Apply(j.GetPrim())
            return j

        def gear(name, b0, h0, b1, h1, ratio):
            gj = PhysxSchema.PhysxPhysicsGearJoint.Define(stage, f"{root}/{JOINT_SCOPE}/{name}")
            gj.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{b0}")])
            gj.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{b1}")])
            gj.CreateHinge0Rel().SetTargets([Sdf.Path(h0)])
            gj.CreateHinge1Rel().SetTargets([Sdf.Path(h1)])
            gj.CreateGearRatioAttr().Set(-gs * ratio)  # PhysX: w1 = -gearRatio w0 about the hinge axes

        def bearing(axle):
            return next((c for c in self.design.connections if c.child == axle and c.kind in ("hinge", "motor")
                         and not (c.parent in self.P and self.P[c.parent].bp.free_bore)), None)

        # shocks: prismatic spring between the cylinder body and the rod
        for name, s in self.shocks.items():
            spec = s["spec"]
            k = spec["rate_n_per_mm"] * 1e3
            path = f"{root}/{JOINT_SCOPE}/shock_{name}"
            j = fixed_or_rev("pri", path, self.body(name), s["rod"], s["rod"], axis="Z", of0=self.ext[s["rod"]])
            j.CreateLowerLimitAttr(0.0)
            j.CreateUpperLimitAttr(spec["stroke_mm"] * 1e-3)
            d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(k)
            d.CreateDampingAttr(float(SHOCK_DAMPING_N_S_PER_M or spec["damping_n_s_per_m"]))
            d.CreateTargetPositionAttr(-spec["preload_n"] / k)
            d.CreateMaxForceAttr(1e3)
            s["joint"] = path
            filt.add((self.body(name), s["rod"]))
            # each end turns on its pin / axle against the parts beside it (face to face, like pinned parts): the
            # end's body does not collide with the pin / axle nor with the parts that pin / axle joins
            for i, (shock, side) in shock_sides(self.design, self.P).items():
                if shock != name:
                    continue
                c = self.design.connections[i]
                link = c.child if c.parent == name else c.parent
                end = s["rod"] if side == "rod" else self.body(name)
                near = {link} | {x for cc in self.design.connections if link in (cc.child, cc.parent)
                                 for x in (cc.child, cc.parent)}
                filt |= {(end, self.body(n)) for n in near if n != name and self.body(n) != end}
            mass = UsdPhysics.MassAPI(stage.GetPrimAtPath(f"{root}/{name}"))
            if name not in self.merged and mass.GetMassAttr().Get():
                mass.GetMassAttr().Set(float(mass.GetMassAttr().Get()) - self.aux_mass[s["rod"]])
        # train motors: two axle revolutes with half the motor line each, coupled 1 : 1
        for name, t in self.trains.items():
            drive = motor_params(self.parts[name].part)["drive"]
            paths = []
            for ax in t["axles"]:
                path = f"{root}/{JOINT_SCOPE}/train_{ax}"
                j = fixed_or_rev("rev", path, self.body(name), ax, ax, axis="Y")
                d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
                d.CreateTypeAttr("force")
                d.CreateStiffnessAttr(0.0)
                d.CreateDampingAttr(0.5 * drive["damping_nm_s_per_deg"])
                d.CreateMaxForceAttr(0.5 * drive["max_force_nm"])
                d.CreateTargetVelocityAttr(0.0)
                paths.append(path)
                filt.add((self.body(name), ax))
            t["joints"] = paths
            gear(f"train_{name}_axles", t["axles"][0], paths[0], t["axles"][1], paths[1], 1.0)
            for i, rm in self.remap.items():
                if rm.get("parent") in t["axles"]:
                    filt.add((self.body(name), self.body(self.design.connections[i].child)))
            mass = UsdPhysics.MassAPI(stage.GetPrimAtPath(f"{root}/{name}"))
            if name not in self.merged and mass.GetMassAttr().Get():
                mass.GetMassAttr().Set(float(mass.GetMassAttr().Get()) - TRAIN_AXLE_MASS_G * 1e-3 * len(t["axles"]))
        # CV joints: 1 : 1 gear joint between the two shafts' bearings + a spherical joint at the ball
        for cv in getattr(self.design, "cv_joints", None) or []:
            sa, sb = self.sh.get(cv.ball), self.sh.get(cv.cup)
            ba, bb = bearing(sa), bearing(sb)
            _, _, ua, ub = cv_lines(self.P, cv)
            if ba is not None and bb is not None:
                ha, hb = self.P[sa].axis(), self.P[sb].axis()
                ratio = float(np.sign(np.dot(ha, ua)) * np.sign(np.dot(hb, ub)))
                gear(f"cv_{cv.name}", self.body(sa), joint_path(root, ba), self.body(sb), joint_path(root, bb), ratio)
            if os.environ.get("LEGO_BENCH_CV_SPHERICAL", "1") != "0":
                fixed_or_rev("sph", f"{root}/{JOINT_SCOPE}/cv_{cv.name}_ball", self.body(cv.ball), self.body(cv.cup),
                             cv.ball)
            filt.add((self.body(cv.ball), self.body(cv.cup)))
        # chains: gear joint za : zb (same sense) between the two gears' shafts
        for g in chain_geoms(self.design, self.P):
            if not isinstance(g, ChainGeom):
                continue
            a, b = g.chain.gears
            sa, sb = self.sh.get(a), self.sh.get(b)
            ba, bb = bearing(sa), bearing(sb)
            if ba is None or bb is None:
                continue
            za, zb = CHAIN_GEARS[self.parts[a].part], CHAIN_GEARS[self.parts[b].part]
            sgn_a = float(np.sign(np.dot(self.P[a].axis(), self.P[sa].axis())))
            sgn_b = float(np.sign(np.dot(self.P[b].axis(), self.P[sb].axis())))
            same = float(np.sign(np.dot(self.P[a].axis(), self.P[b].axis())))
            gear(f"chain_{g.chain.name}", self.body(sa), joint_path(root, ba), self.body(sb), joint_path(root, bb),
                 za / zb * sgn_a * sgn_b * same)
        # differentials: case revolute in an output axle's bearing, ring gear joints, satellites fixed to the case
        for d in getattr(self.design, "differentials", None) or []:
            axles = diff_axles(self.design, d)
            hinges = [next((c for c in self.design.connections if c.kind == "hinge" and c.child == x and c.parent == d.case),
                           None) for x in axles]
            brg = next((c for x in axles for c in self.design.connections if c.kind in ("hinge", "motor")
                        and c.child == x and c.parent != d.case), None)
            if brg is None or any(h is None for h in hinges):
                self.log(f"[R4B] differential {d.name}: no bearing / hinges: not simulated")
                continue
            cpath = f"{root}/{JOINT_SCOPE}/diff_{d.name}_case"
            fixed_or_rev("rev", cpath, self.body(brg.parent), self.body(d.case), d.case, axis="Y")
            for m in self.design.gear_meshes:
                if d.case not in (m.gear_a, m.gear_b):
                    continue
                other = m.gear_b if m.gear_a == d.case else m.gear_a
                so = self.sh.get(other)
                bo = bearing(so)
                if bo is None:
                    continue
                zo = self.P[other].bp.teeth
                ca, go = self.P[d.case].axis(), self.P[other].axis()
                perp = abs(float(np.dot(ca, go))) < 1e-3
                k = (DIFF_RING_TEETH / zo) if perp else (-DIFF_RING_TEETH / zo)
                k *= float(np.sign(np.dot(go, self.P[so].axis())))
                gear(f"diff_{d.name}_ring_{other}", self.body(d.case), cpath, self.body(so), joint_path(root, bo), k)
                filt.add((self.body(d.case), self.body(other)))
            for s in d.satellites:
                fixed_or_rev("fix", f"{root}/{JOINT_SCOPE}/diff_{d.name}_sat_{s}", self.body(d.case), self.body(s), s)
            group = [d.case, *d.outputs, *d.satellites, *axles]
            filt |= {(self.body(x), self.body(y)) for x in group for y in group if self.body(x) != self.body(y)}
            a = self.P[d.case].axis()
            hj = []
            for x, h in zip(axles, hinges, strict=True):
                dr = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(joint_path(root, h)), "angular")
                dr.GetStiffnessAttr().Set(DIFF_COUPLING_NM_PER_RAD * math.pi / 180.0)
                dr.GetDampingAttr().Set(DIFF_COUPLING_NMS_PER_RAD * math.pi / 180.0)
                dr.GetTargetPositionAttr().Set(0.0)
                dr.GetMaxForceAttr().Set(DIFF_MAX_TORQUE_NM)
                hj.append({"path": joint_path(root, h), "axle": x,
                           "sign": float(np.sign(np.dot(self.P[x].axis(), a)))})
            self.diffs.append({"diff": d, "hinges": hj, "axis_local": (0.0, 1.0, 0.0), "log": []})
        for m, pb in self.pullbacks.items():  # wound and held (ratchet / the owner's hand) until the release at t = 0
            dr = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(joint_path(root, pb["conn"])), "angular")
            dr.GetDampingAttr().Set(PULLBACK_HOLD_NM / PULLBACK_HOLD_CREEP_DEG_S)
            dr.GetMaxForceAttr().Set(PULLBACK_HOLD_NM)
            dr.GetTargetVelocityAttr().Set(0.0)
        for a, b in sorted(filt):
            if a == b:
                continue
            api = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{root}/{a}"))
            api.CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{root}/{b}"))

    def add_masses(self, stage) -> None:
        """After the shaft conditioning: the chain links' mass onto the two gears' bodies."""
        from pxr import UsdPhysics  # noqa: PLC0415

        for ch in getattr(self.design, "chains", None) or []:
            for g in ch.gears:
                if g not in self.parts:
                    continue
                ma = UsdPhysics.MassAPI(stage.GetPrimAtPath(f"{self.root}/{self.body(g)}"))
                m = ma.GetMassAttr().Get()
                if m:
                    ma.GetMassAttr().Set(float(m) + 0.5 * ch.links * CHAIN_LINK_MASS_G * 1e-3)

    # -- motors --
    def set_train_level(self, stage, motor: str, target_deg_s: float, coast: bool, damping: float) -> bool:
        """Train motor command (env.set_motor_level): both axle drives. False when ``motor`` is not a train motor."""
        from pxr import UsdPhysics  # noqa: PLC0415

        t = self.trains.get(motor)
        if t is None:
            return False
        for path in t.get("joints", ()):
            d = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(path), "angular")
            d.GetDampingAttr().Set(0.0 if coast else 0.5 * damping)
            d.GetTargetVelocityAttr().Set(float(target_deg_s))
        return True

    def release(self) -> None:
        """t = 0: the pullback motors are let go."""
        self.released = True

    def step(self, stage, dt: float, body_frame) -> None:
        """Every physics step: the differentials' coupling, the pullback motors' spring torque, the shocks' travel."""
        from pxr import UsdPhysics  # noqa: PLC0415

        from .env import joint_path  # noqa: PLC0415

        for st in self.diffs:
            d = st["diff"]
            wc_vec, qc = body_frame(d.case)
            a = _qmat(qc) @ np.array(st["axis_local"])
            wc = float(np.dot(wc_vec, a))
            w = [float(np.dot(body_frame(h["axle"])[0], a)) for h in st["hinges"]]
            ws = 0.5 * (w[1] - w[0])
            # the outputs' turns relative to the case (accumulated) give the spider angle phi = (th_R - th_L) / 2; each
            # axle-in-case drive pulls its joint to -/+ phi (stiffness K, light damping at -/+ w_s): equal torques
            # K (w_case lag), the reaction on the case, the difference (phi) free up to one step of lag
            # each output's turn relative to the case about its joint axis (the axle's x), from the poses: exact, no
            # drift (an integrated velocity drifted and loaded the drive)
            Rc = _qmat(qc)
            th = st.setdefault("theta", [0.0, 0.0])
            prev = st.setdefault("rrel", [None, None])
            for k, h in enumerate(st["hinges"]):
                Rr = Rc.T @ _qmat(body_frame(h["axle"])[1])
                if prev[k] is not None:
                    dR = prev[k].T @ Rr
                    th[k] += h["sign"] * math.atan2(dR[2, 1], dR[1, 1])
                prev[k] = Rr
            phi = 0.5 * (th[1] - th[0])
            for h, tgt, v in zip(st["hinges"], (-phi, phi), (-ws, ws), strict=True):
                dr = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(h["path"]), "angular")
                ang = (math.degrees(h["sign"] * tgt) + 180.0) % 360.0 - 180.0  # the joint angle wraps
                dr.GetTargetPositionAttr().Set(float(ang))
                dr.GetTargetVelocityAttr().Set(float(math.degrees(h["sign"] * v)))
            st["last"] = (wc, w[0], w[1])
            st["n"] = st.get("n", 0) + 1
            if os.environ.get("LEGO_BENCH_R4B_DEBUG") and st["n"] % 500 == 0:
                self.log(f"[R4B] diff {d.name}: w_case {wc:+.3f} w_l {w[0]:+.3f} w_r {w[1]:+.3f} rad/s, targets "
                         f"{[h['path'].rsplit('/', 1)[-1] for h in st['hinges']]} {-ws:+.3f} / {ws:+.3f}")

        if self.released:
            for m, s in self.pullbacks.items():
                c = s["conn"]
                w_s, q_s = body_frame(c.child)
                w_m, _ = body_frame(c.parent)
                ax = _qmat(q_s) @ np.array([1.0, 0.0, 0.0])
                w = s["sense"] * float(np.dot(w_s - w_m, ax))
                s["theta"] = min(max(s["theta"] + w * dt, 0.0), s["theta_max"])  # turned back: it winds up again
                s["w_max"] = max(s["w_max"], w)
                frac = max(0.0, 1.0 - s["theta"] / s["theta_max"])
                tau = s["tau"] * frac
                regime = "free" if (tau <= 1e-6 or w >= s["w0"]) else "drive"
                key = (regime, round(tau, 5))
                if key == s["regime"]:
                    continue
                s["regime"] = key
                d = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(joint_path(self.root, c)), "angular")
                w0_deg = math.degrees(s["w0"])
                if regime == "free":
                    d.GetDampingAttr().Set(0.0)
                    d.GetMaxForceAttr().Set(0.0)
                else:
                    d.GetDampingAttr().Set(tau / w0_deg)
                    d.GetMaxForceAttr().Set(tau)
                    d.GetTargetVelocityAttr().Set(s["sense"] * w0_deg)

    def sample(self, t: float, parts_now: dict) -> None:
        """On the judge's check loop: the shocks' compression (from the rod's pose)."""
        for name, s in self.shocks.items():
            q = self.compression(name, parts_now)
            if q is None:
                continue
            s["q_min"], s["q_max"] = min(s["q_min"], q), max(s["q_max"], q)
            s["q_last"] = q
            tr = self.trace.setdefault(name, [])
            if not tr or t - tr[-1][0] >= 0.05 - 1e-9:
                tr.append((round(t, 3), round(q, 3)))

    def compression(self, name: str, parts_now: dict):
        """Compression [mm] of a shock (0 = extended) from the current poses of its two bodies."""
        s = self.shocks[name]
        o = self.aux_objs.get(s["rod"])
        if o is None or name not in parts_now:
            return None
        pc, qc = parts_now[name]
        pr = np.array(o.data.root_pos_w.torch[0].cpu().tolist())
        R = _qmat(qc)
        hole = np.array(pc) + R @ (np.array(s["spec"]["rod_hole"]) * 1e-3)
        return float(np.dot(pr - hole, R @ SHOCK_AXIS)) * 1e3

    def result(self) -> dict:
        out = {}
        if self.shocks:
            out["shocks"] = {n: {"min_mm": round(s["q_min"], 3), "max_mm": round(s["q_max"], 3),
                                 "final_mm": round(s.get("q_last", 0.0), 3), "stroke_mm": s["spec"]["stroke_mm"],
                                 "trace": self.trace.get(n, [])[:400]} for n, s in self.shocks.items()}
        if self.pullbacks:
            out["pullbacks"] = {m: {"wind_turns": s["wind_turns"],
                                    "turns_unwound": round(s["theta"] / (2 * math.pi) - (s["theta_max"] / (2 * math.pi)
                                                                                        - s["wind_turns"]), 3),
                                    "w_max_rad_s": round(s["w_max"], 3)} for m, s in self.pullbacks.items()}
        if self.diffs:
            out["differentials"] = {st["diff"].name: {"w_case_l_r_rad_s": [round(v, 3) for v in st.get("last", ())]}
                                    for st in self.diffs}
        return out


if __name__ == "__main__":
    import sys

    for p in write_usds(sys.argv[1:] or None):
        print(p)


# ----------------------------------------------------------------------------------------------------------------
# visual chain / tread links (2026-10-14, round 4b follow-up): moving link boxes, visual only
# ----------------------------------------------------------------------------------------------------------------


def loop_path(ca, cb, ra: float, rb: float, axis, step: float = 0.5) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Ordered closed path of a belt round two pitch circles (design frame, mm), in the direction a point of gear a's
    pitch circle moves when gear a turns +1 about ``axis``: (points, unit tangents, cumulative length)."""
    ca, cb, g = np.asarray(ca, float), np.asarray(cb, float), np.asarray(axis, float)
    g = g / np.linalg.norm(g)
    d = cb - ca
    d = d - float(np.dot(d, g)) * g
    D = float(np.linalg.norm(d))
    e = d / D
    n = np.cross(g, e)
    beta = math.asin(max(-1.0, min(1.0, (ra - rb) / D)))
    u1 = math.cos(beta) * n + math.sin(beta) * e  # upper tangent normal
    u2 = -math.cos(beta) * n + math.sin(beta) * e
    pts = []

    def arc(c, r, u_from, u_to, through):
        a0 = math.atan2(float(np.dot(u_from, n)), float(np.dot(u_from, e)))
        a1 = math.atan2(float(np.dot(u_to, n)), float(np.dot(u_to, e)))
        at = math.atan2(float(np.dot(through, n)), float(np.dot(through, e)))
        # go from a0 to a1 passing at
        for sgn in (1.0, -1.0):
            span = (sgn * (a1 - a0)) % (2 * math.pi)
            if (sgn * (at - a0)) % (2 * math.pi) <= span:
                break
        k = max(4, int(span * r / step))
        for t in np.linspace(0.0, span, k, endpoint=False):
            a = a0 + sgn * t
            pts.append(c + r * (math.cos(a) * e + math.sin(a) * n))

    def line(p, q):
        k = max(2, int(float(np.linalg.norm(q - p)) / step))
        for t in np.linspace(0.0, 1.0, k, endpoint=False):
            pts.append(p + (q - p) * t)

    arc(ca, ra, u1, u2, -e)
    line(ca + ra * u2, cb + rb * u2)
    arc(cb, rb, u2, u1, e)
    line(cb + rb * u1, ca + ra * u1)
    P = np.array(pts)
    seg = np.roll(P, -1, axis=0) - P
    L = np.linalg.norm(seg, axis=1)
    tang = seg / np.maximum(L, 1e-12)[:, None]
    return P, tang, np.concatenate([[0.0], np.cumsum(L)])


class LoopVisuals:
    """Moving visual links of the design's chains (coupling loops) and treads (incl. 3711 chain tracks): the real LDraw
    link meshes (``<PART_USD_DIR>/links/<id>.usda``, scripts/convert_link_visuals.py; a box if the file is missing),
    children of the loop's carrier body (no collision, no mass). Each link's hinge sits on the loop's pitch line (the
    hinge line round the sprockets / gears), the link points along the chord to the next hinge, its outer face out of
    the loop; the links are advanced at the belt speed (the reference gear / sprocket's turn relative to the carrier x
    its pitch radius). Updated on the judge's check loop."""

    def __init__(self, design, pose, merged: dict, root: str):
        from .design import place  # noqa: PLC0415

        self.design, self.root, self.merged = design, root, merged or {}
        self.P = place(design)
        self.parts = {p.name: p for p in design.parts}
        self.loops: list[dict] = []
        for g in chain_geoms(design, self.P):
            if not isinstance(g, ChainGeom):
                continue
            brg = self._bearing(g.chain.gears[0])
            if brg is None:
                continue
            self.loops.append(dict(name=f"chain_{g.chain.name}", ref=g.chain.gears[0], carrier=self.body(brg.parent),
                                   ca=g.ca, cb=g.cb, ra=g.ra, rb=g.rb, axis=self.P[g.chain.gears[0]].axis(),
                                   n=g.chain.links, dims=(2 * CHAIN_HALF_W, 0.85 * CHAIN_PITCH_MM, 2 * CHAIN_HALF_T),
                                   link="3711", existing=None))
        from .treads import TREAD_LINKS, carrier, tread_geoms  # noqa: PLC0415

        for g in tread_geoms(design, self.P):
            car = carrier(g, design, self.P)
            if not car:
                continue
            L = TREAD_LINKS[g.tread.link]
            self.loops.append(dict(name=g.tread.name, ref=g.tread.sprockets[0], carrier=self.body(car), ca=g.ca,
                                   cb=g.cb, ra=g.r_pitch, rb=g.r_pitch, axis=self.P[g.tread.sprockets[0]].axis(),
                                   n=g.tread.links, dims=(2 * g.half_w, 0.875 * L["pitch"], L["pad"] + L["inner"]),
                                   link=g.tread.link, existing=f"TreadVisual_{g.tread.name}"))
        for lp in self.loops:
            lp["path"] = loop_path(lp["ca"], lp["cb"], lp["ra"], lp["rb"], lp["axis"])
            lp["s"] = 0.0
            lp["prims"] = []

    def body(self, n):
        return self.merged.get(n, n)

    def _bearing(self, gear):
        from .design import shafts  # noqa: PLC0415

        sh = shafts(self.design).get(gear)
        return next((c for c in self.design.connections if c.child == sh and c.kind in ("hinge", "motor")), None)

    def author(self, stage) -> None:
        from pxr import Gf, UsdGeom  # noqa: PLC0415

        from ai_pd_tasks.lego.catalog import PART_USD_DIR  # noqa: PLC0415

        for lp in self.loops:
            base = f"{self.root}/{lp['carrier']}/{lp['existing'] or 'ChainVisual_' + lp['name']}"
            if not stage.GetPrimAtPath(base).IsValid():
                UsdGeom.Xform.Define(stage, base)
            usd = os.path.abspath(os.path.join(PART_USD_DIR, "links", f"{lp['link']}.usda"))
            mesh = os.path.exists(usd)
            lp["mesh"] = mesh
            for k in range(int(lp["n"])):
                path = f"{base}/link_{k}"
                if stage.GetPrimAtPath(path).IsValid():  # the tread model's static boxes
                    stage.RemovePrim(path)
                if mesh:
                    x = UsdGeom.Xform.Define(stage, path)
                    x.GetPrim().GetReferences().AddReference(usd)
                    x.AddTranslateOp()
                    x.AddOrientOp()
                    prim = x.GetPrim()
                else:
                    cube = UsdGeom.Cube.Define(stage, path)
                    cube.CreateSizeAttr(1.0)
                    cube.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.08, 0.09)])
                    cube.AddTranslateOp()
                    cube.AddOrientOp()
                    cube.AddScaleOp().Set(Gf.Vec3f(*(v * 1e-3 for v in lp["dims"])))
                    prim = cube.GetPrim()
                ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
                lp["prims"].append((ops[0], ops[1]))
            self.place(lp)

    @staticmethod
    def _at(lp, s: float) -> np.ndarray:
        P, T, cum = lp["path"]
        s %= cum[-1]
        i = min(int(np.searchsorted(cum, s, side="right")) - 1, len(P) - 1)
        return P[i] + T[i] * (s - cum[i])

    def place(self, lp) -> None:
        from pxr import Gf  # noqa: PLC0415

        from .design import rot_matrix  # noqa: PLC0415
        from .treads import _quat_from_matrix  # noqa: PLC0415

        P, _T, cum = lp["path"]
        Ltot = cum[-1]
        cp = self.parts[lp["carrier"]]
        Rc = rot_matrix(cp.yaw_deg, cp.spin_deg)
        pc = np.array(cp.pos_mm)
        g = np.asarray(lp["axis"], float)
        centre = 0.5 * (np.asarray(lp["ca"], float) + np.asarray(lp["cb"], float))
        n = int(lp["n"])
        step = Ltot / n
        for k, (op_t, op_r) in enumerate(lp["prims"]):
            s = (lp["s"] + k * step) % Ltot
            p, q = self._at(lp, s), self._at(lp, s + step)  # this hinge and the next one
            y = q - p
            y /= np.linalg.norm(y)
            z = np.cross(y, g)
            if np.dot(z, 0.5 * (p + q) - centre) < 0:
                z = -z
            if lp.get("mesh"):  # the link's hinge on the pitch line, its body along the chord, the outer face out
                Rl, at = np.stack([np.cross(y, z), y, z], axis=1), p
            else:  # box centred on the chord
                Rl, at = np.stack([np.cross(y, z), y, z], axis=1), 0.5 * (p + q)
            Rl = Rc.T @ Rl
            qq = _quat_from_matrix(Rl)
            op_t.Set(Gf.Vec3d(*(float(v) * 1e-3 for v in Rc.T @ (at - pc))))
            op_r.Set(Gf.Quatf(float(qq[3]), Gf.Vec3f(*(float(v) for v in qq[:3]))))

    def step(self, dt: float, body_frame) -> None:
        """Advance each loop by the reference gear's turn relative to its carrier (every physics step)."""
        for lp in self.loops:
            w_g, q_g = body_frame(lp["ref"])
            w_c, _ = body_frame(lp["carrier"])
            ax = _qmat(q_g) @ np.array([0.0, 1.0, 0.0])  # gears / sprockets / wheels: axis local y
            lp["s"] += float(np.dot(w_g - w_c, ax)) * lp["ra"] * dt

    def update(self) -> None:
        for lp in self.loops:
            self.place(lp)
