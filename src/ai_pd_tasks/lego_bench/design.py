# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Design spec for LEGO assemblies, a static validator and computed stats (no simulator).

Design frame (same as :mod:`ai_pd_tasks.lego.car_design`): x forward (direction of travel), y left, z up, the
lowest wheel bottoms at z = 0 for a ground vehicle. Lengths in the JSON are millimetres, angles in degrees.
A part pose is the pose of its LDraw origin (see :mod:`.parts`): position, then ``yaw_deg`` about z and
``spin_deg`` about the part's own y axis (quaternion = yaw * spin, xyzw), exactly as ``car_design.PartInstance``.

Connections (as in :mod:`ai_pd_tasks.lego.joints`):

* ``stud``  child pressed on/under parent (fixed joint, break force 10 N per stud);
* ``cross`` gear / wheel / bush on an axle (fixed joint);
* ``hinge`` axle turning in a Technic brick hole (revolute joint about the axle, light viscous damping);
* ``motor`` axle in a motor's output (revolute joint with the DC-motor velocity drive);
* ``pin``   a Technic pin (child) in a pin hole of the parent (Technic brick hole, motor pin hole): fixed joint. A
            pin needs two pin connections (one per joined part).

Gear meshes are real tooth contact in simulation; the validator checks the centre distance (module 1:
``(z1 + z2) / 2`` mm). JSON example::

    {
        "name": "car_9686",
        "chassis": "rail_l",
        "motor": "motor",
        "battery": "battery",
        "parts": [{"name": "rail_l", "part": "3895", "pos_mm": [0, 12, 16]}, ...],
        "connections": [{"child": "deck", "parent": "rail_l", "kind": "stud", "studs": 6}, ...],
        "gear_meshes": [{"gear_a": "pinion8", "gear_b": "jack_gear24"}, ...],
    }
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field
from itertools import combinations

import numpy as np

from .parts import (
    GEAR_MODULE,
    INVENTORY_IDS,
    BenchPart,
    Box,
    Boxes,
    Segs,
    bench_parts,
    bom_items,
    default_inventory,
    part_metadata,
    part_set_inventory,
    pin_hole_frames,
)
from .inventory_r4 import mesh_problems
from . import inventory_r4b as r4b
from .inventory_r4b import DesignChain, DesignCV, DesignDiff, DesignPullback
from .racks import DesignRack, slide_edges
from .treads import DesignTread, check_treads, tread_mass_g, tread_pieces, tread_ratio, tread_wheel_radii

CONNECTION_KINDS = ("stud", "cross", "hinge", "motor", "pin")
RIGID_KINDS = ("stud", "pin")  # connections that tie two parts rigidly (a chassis cluster)
PIN_MIN_ENGAGE = 4.0  # mm of a pin inside each of its holes
SKID_HEIGHT_MM = 5.0  # a non-wheel part this close to the ground is a skid (gap spanning)
G = 9.81

# friction (static, dynamic) of the materials; PhysX combines two materials by averaging (default combine mode)
ABS_FRICTION = (0.40, 0.35)
SURFACES: dict[str, tuple[float, float]] = {
    "laminate": (0.35, 0.30),  # table / course boards (same as the LEGO cell table)
    "wood": (0.45, 0.40),  # sanded plank
    "rubber_mat": (1.00, 0.90),  # rubber mat (high grip)
    "waxed": (0.12, 0.10),  # waxed board / PTFE sheet (low friction)
    "cord": (0.30, 0.25),  # braided nylon cord (rope)
    "abs": ABS_FRICTION,  # LEGO / ABS load
    "steel": (0.30, 0.25),
}


TYRE_REFERENCE_SURFACE = "wood"
"""Surface standing in for the plywood board of the tyre measurements (BEC)."""


def _apply_calibration() -> None:
    """Simulator calibration overrides (2026-10-12; the benchmark's sim parameters, bench_eval/spec/sim_params.json):
    ``LEGO_BENCH_ABS_MU="s,d"`` (every hard LEGO part, and the LEGO-plate course surface 'abs'),
    ``LEGO_BENCH_SURFACE_MU="laminate=s,d;cord=s,d"`` (course surfaces). Unset: the values above."""
    global ABS_FRICTION
    v = os.environ.get("LEGO_BENCH_ABS_MU")
    if v:
        ABS_FRICTION = tuple(float(x) for x in v.split(","))[:2]
        SURFACES["abs"] = ABS_FRICTION
    for item in filter(None, (os.environ.get("LEGO_BENCH_SURFACE_MU") or "").split(";")):
        name, val = item.split("=")
        SURFACES[name.strip()] = tuple(float(x) for x in val.split(","))[:2]


_apply_calibration()
TYRE_MU_SCALE = float(os.environ.get("LEGO_BENCH_TYRE_MU_SCALE", "1") or 1.0)
"""Calibration: scales every rubber tyre's measured friction (``part_friction``)."""


def part_friction(part_id: str) -> tuple[float, float]:
    """(static, dynamic) friction of a part's simulation material: ABS, or for a rubber-tyred wheel a rubber material
    chosen so that PhysX's average combine reproduces the measured tyre friction on the reference surface (plywood
    ~ ``wood``): mu_mat = 2 mu_measured - mu_wood (static = dynamic; the measurement is sliding friction)."""
    bp = bench_parts().get(part_id)
    if bp is None or not bp.tyre_mu:
        return ABS_FRICTION
    mu = 2.0 * bp.tyre_mu * TYRE_MU_SCALE - SURFACES[TYRE_REFERENCE_SURFACE][0]
    return (round(mu, 4), round(mu, 4))


def contact_friction(surface: str, part_id: str | None = None) -> tuple[float, float]:
    """Effective (static, dynamic) friction of ABS LEGO parts (or of ``part_id``'s material) on ``surface`` (PhysX
    average combine)."""
    s, d = SURFACES[surface]
    ps, pd = part_friction(part_id) if part_id else ABS_FRICTION
    return (0.5 * (s + ps), 0.5 * (d + pd))


# ----------------------------------------------------------------------------------------------------------------
# format
# ----------------------------------------------------------------------------------------------------------------


@dataclass
class DesignPart:
    name: str
    part: str
    pos_mm: tuple[float, float, float]
    yaw_deg: float = 0.0
    spin_deg: float = 0.0

    def __post_init__(self):
        self.pos_mm = tuple(float(v) for v in self.pos_mm)


@dataclass
class DesignConnection:
    child: str
    parent: str
    kind: str
    studs: int = 0


@dataclass
class DesignGearMesh:
    gear_a: str
    gear_b: str
    teeth_a: int = 0  # optional; checked against the catalog when given
    teeth_b: int = 0


@dataclass
class Design:
    name: str
    parts: list[DesignPart]
    connections: list[DesignConnection]
    gear_meshes: list[DesignGearMesh] = field(default_factory=list)
    chassis: str = ""  # reference body for progress / flip / break-apart checks
    motor: str = ""  # M-motor part name ("" = unpowered)
    battery: str = ""
    description: str = ""
    tow_point_mm: tuple[float, float, float] | None = None  # hook for the 'tug' obstacle (on the chassis)
    intended: list[str] = field(default_factory=list)  # obstacles the designer targets (informative)
    part_set: str = "9686"  # part set the design draws from ('9686' default, 'extended', 'open'; see parts.PART_SETS)
    rope_wheels: list[str] = field(default_factory=list)  # wheels that run on a rope (rope obstacle; optional)
    motor_program: list[dict] = field(default_factory=list)  # timed / position-triggered motor commands (optional)
    hub: str = ""  # SPIKE Prime Large Hub part (programmable controller, 6 ports) (optional)
    ports: dict[str, str] = field(default_factory=dict)  # hub port letter -> motor / sensor part name
    cargo_mm: tuple[float, float, float] | None = None  # where a tier payload is set on the vehicle (top surface)
    treads: list[DesignTread] = field(default_factory=list)  # tread loops round two sprockets (lego_bench.treads)
    racks: list[DesignRack] = field(default_factory=list)  # rack slides: racks + pinion + guides (lego_bench.racks)
    # round 4b (lego_bench.inventory_r4b): 3711 chain loops, CV joints, differentials (62821b)
    chains: list[DesignChain] = field(default_factory=list)
    cv_joints: list[DesignCV] = field(default_factory=list)
    differentials: list[DesignDiff] = field(default_factory=list)
    pullback: list[DesignPullback] = field(default_factory=list)  # how far each pullback motor is wound (optional)
    start_gap_mm: float | None = None  # front of the vehicle to the obstacle's start line (None: the bench default)

    # -- access --
    def part(self, name: str) -> DesignPart:
        for p in self.parts:
            if p.name == name:
                return p
        raise KeyError(name)

    @property
    def part_names(self) -> list[str]:
        return [p.name for p in self.parts]

    def bom(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for p in self.parts:
            out[p.part] = out.get(p.part, 0) + 1
        for t in self.treads:  # tread links are pieces of the bill of materials
            out[t.link] = out.get(t.link, 0) + int(t.links)
        for c in self.chains:  # round 4b: 3711 chain links too
            out["3711"] = out.get("3711", 0) + int(c.links)
        return out

    # -- JSON --
    def to_dict(self) -> dict:
        d = {
            "name": self.name,
            "description": self.description,
            "chassis": self.chassis,
            "motor": self.motor,
            "battery": self.battery,
            "intended": list(self.intended),
            "parts": [
                {
                    k: v
                    for k, v in (
                        ("name", p.name),
                        ("part", p.part),
                        ("pos_mm", [round(c, 4) for c in p.pos_mm]),
                        ("yaw_deg", p.yaw_deg),
                        ("spin_deg", p.spin_deg),
                    )
                    if not (k == "spin_deg" and v == 0.0)
                }
                for p in self.parts
            ],
            "connections": [
                {k: v for k, v in asdict(c).items() if not (k == "studs" and c.kind != "stud")}
                for c in self.connections
            ],
            "gear_meshes": [
                {k: v for k, v in asdict(m).items() if v or k in ("gear_a", "gear_b")} for m in self.gear_meshes
            ],
        }
        if self.tow_point_mm is not None:
            d["tow_point_mm"] = [round(c, 4) for c in self.tow_point_mm]
        if self.part_set != "9686":
            d["part_set"] = self.part_set
        if self.rope_wheels:
            d["rope_wheels"] = list(self.rope_wheels)
        if self.motor_program:
            d["motor_program"] = [dict(c) for c in self.motor_program]
        if self.start_gap_mm is not None:
            d["start_gap_mm"] = round(float(self.start_gap_mm), 2)
        if self.hub:
            d["hub"] = self.hub
            d["ports"] = dict(self.ports)
        if self.cargo_mm is not None:
            d["cargo_mm"] = [round(c, 4) for c in self.cargo_mm]
        if self.treads:
            d["treads"] = [t.to_dict() for t in self.treads]
        if self.racks:
            d["racks"] = [r.to_dict() for r in self.racks]
        for key in ("chains", "cv_joints", "differentials", "pullback"):  # round 4b
            if getattr(self, key):
                d[key] = [x.to_dict() for x in getattr(self, key)]
        return d

    @classmethod
    def from_dict(cls, d: dict) -> Design:
        unknown = set(d) - {
            "name",
            "description",
            "chassis",
            "motor",
            "battery",
            "intended",
            "parts",
            "connections",
            "gear_meshes",
            "tow_point_mm",
            "part_set",
            "rope_wheels",
            "motor_program",
            "hub",
            "ports",
            "cargo_mm",
            "treads",
            "racks",
            "chains",
            "cv_joints",
            "differentials",
            "pullback",
            "start_gap_mm",
        }
        if unknown:
            raise ValueError(f"unknown design fields: {sorted(unknown)}")
        return cls(
            name=d["name"],
            description=d.get("description", ""),
            chassis=d.get("chassis", ""),
            motor=d.get("motor", ""),
            battery=d.get("battery", ""),
            intended=list(d.get("intended", [])),
            parts=[
                DesignPart(
                    p["name"],
                    str(p["part"]),
                    tuple(p["pos_mm"]),
                    float(p.get("yaw_deg", 0.0)),
                    float(p.get("spin_deg", 0.0)),
                )
                for p in d["parts"]
            ],
            connections=[
                DesignConnection(c["child"], c["parent"], c["kind"], int(c.get("studs", 0)))
                for c in d.get("connections", [])
            ],
            gear_meshes=[
                DesignGearMesh(m["gear_a"], m["gear_b"], int(m.get("teeth_a", 0)), int(m.get("teeth_b", 0)))
                for m in d.get("gear_meshes", [])
            ],
            tow_point_mm=tuple(d["tow_point_mm"]) if d.get("tow_point_mm") is not None else None,
            part_set=str(d.get("part_set", "9686")),
            rope_wheels=[str(n) for n in d.get("rope_wheels", [])],
            motor_program=[dict(c) for c in d.get("motor_program", [])],
            hub=str(d.get("hub", "")),
            ports={str(k): str(v) for k, v in d.get("ports", {}).items()},
            cargo_mm=tuple(d["cargo_mm"]) if d.get("cargo_mm") is not None else None,
            treads=[DesignTread.from_dict(t) for t in d.get("treads", [])],
            racks=[DesignRack.from_dict(r) for r in d.get("racks", [])],
            chains=[DesignChain.from_dict(x) for x in d.get("chains", [])],
            cv_joints=[DesignCV.from_dict(x) for x in d.get("cv_joints", [])],
            differentials=[DesignDiff.from_dict(x) for x in d.get("differentials", [])],
            pullback=[DesignPullback.from_dict(x) for x in d.get("pullback", [])],
            start_gap_mm=float(d["start_gap_mm"]) if d.get("start_gap_mm") is not None else None,
        )

    def to_json(self, indent: int | None = 1) -> str:
        return json.dumps(self.to_dict(), indent=indent)

    @classmethod
    def from_json(cls, text: str) -> Design:
        return cls.from_dict(json.loads(text))

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            f.write(self.to_json() + "\n")

    @classmethod
    def load(cls, path: str) -> Design:
        with open(path) as f:
            return cls.from_json(f.read())


def from_car_design() -> Design:
    """The existing 9686 car (:mod:`ai_pd_tasks.lego.car_design`) in the design format."""
    from ai_pd_tasks.lego import car_design as cd
    from ai_pd_tasks.lego.catalog import SUBPART_OF

    # parts the build simulates separately but that belong to another inventory item (the motor's cable plug) are
    # not benchmark parts: the benchmark's motor mass already includes its cable
    sub = {p.name for p in cd.PARTS if p.part in SUBPART_OF}
    return Design(
        name="car_9686",
        description="The LEGO Education 9686 car of the bimanual build task: 2 x 1x12 Technic rails, M-motor -> 8T:24T "
        "-> 16T:16T (3:1) to the rear axle, 4 wedge-belt wheels, AAA battery box on a 4x6 deck.",
        chassis="rail_l",
        motor="motor",
        battery="battery",
        intended=["ramp", "gap", "bridge", "steps", "tunnel", "speed"],
        parts=[DesignPart(p.name, p.part, tuple(round(v / cd.MM, 4) for v in p.pos), p.yaw_deg, 0.0) for p in cd.PARTS
               if p.name not in sub],
        connections=[DesignConnection(c.child, c.parent, c.kind, c.studs) for c in cd.CONNECTIONS
                     if c.child not in sub and c.parent not in sub],
        gear_meshes=[DesignGearMesh(m.gear_a, m.gear_b, m.teeth_a, m.teeth_b) for m in cd.GEAR_MESHES],
        tow_point_mm=(-48.0, 0.0, 12.0),
    )


# ----------------------------------------------------------------------------------------------------------------
# geometry helpers
# ----------------------------------------------------------------------------------------------------------------


def rot_matrix(yaw_deg: float, spin_deg: float = 0.0) -> np.ndarray:
    """Rz(yaw) @ Ry(spin): the orientation used by car_design.part_quat_xyzw."""
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    cs, ss = math.cos(math.radians(spin_deg)), math.sin(math.radians(spin_deg))
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cs, 0.0, ss], [0.0, 1.0, 0.0], [-ss, 0.0, cs]])
    return rz @ ry


def quat_xyzw(yaw_deg: float, spin_deg: float = 0.0) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) of :func:`rot_matrix` (Isaac Lab 3.0 convention)."""
    hy, hs = math.radians(yaw_deg) / 2, math.radians(spin_deg) / 2
    a = (0.0, 0.0, math.sin(hy), math.cos(hy))
    b = (0.0, math.sin(hs), 0.0, math.cos(hs))
    return qmul(a, b)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


@dataclass
class Placed:
    """A part instance with its catalog data and pose (design frame, mm)."""

    inst: DesignPart
    bp: BenchPart
    R: np.ndarray
    p: np.ndarray

    def to_world(self, v) -> np.ndarray:
        return self.p + self.R @ np.asarray(v, dtype=float)

    def to_local(self, w) -> np.ndarray:
        return self.R.T @ (np.asarray(w, dtype=float) - self.p)

    def axis(self) -> np.ndarray:
        """Rotation axis in the design frame: local x for axles, local y for gears / wheels / bushes / motor output."""
        return self.R[:, 0] if self.bp.kind in ("axle", "pin") else self.R[:, 1]

    def aabb(self) -> tuple[np.ndarray, np.ndarray]:
        s = self.bp.shape
        if isinstance(s, Box):
            lo, hi = np.array(s.lo), np.array(s.hi)
        else:
            c = np.array(s.centre)
            ext = np.full(3, s.radius)
            ext["xyz".index(s.axis)] = s.half_len
            lo, hi = c - ext, c + ext
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        w = (self.R @ corners.T).T + self.p
        return w.min(axis=0), w.max(axis=0)


def place(design: Design) -> dict[str, Placed]:
    parts = bench_parts()
    P = {
        p.name: Placed(p, parts[p.part], rot_matrix(p.yaw_deg, p.spin_deg), np.array(p.pos_mm, dtype=float))
        for p in design.parts
        if p.part in parts
    }
    if any(r4b.is_shock(p.part) for p in design.parts):  # round 4b: a shock drawn compressed (its rod end's partner)
        r4b.place_shocks(design, P)
    return P


def _seg_dist(shape: Segs, xz: np.ndarray) -> np.ndarray:
    """Distance of points (N, 2) in the x-z plane from a shaped liftarm's centre line."""
    best = np.full(len(xz), np.inf)
    for x0, z0, x1, z1 in shape.segs:
        a, b = np.array([x0, z0]), np.array([x1, z1])
        d = b - a
        t = np.clip(((xz - a) @ d) / max(float(d @ d), 1e-12), 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(xz - (a + t[:, None] * d), axis=1))
    return best


def _depth_in_shape(shape, pts_local: np.ndarray) -> np.ndarray:
    """Signed penetration depth of local points into a shape (> 0 inside)."""
    if isinstance(shape, Boxes):  # union of boxes (round 4a: gearbox, bent connector, steering arm)
        return np.max([_depth_in_shape(Box(lo, hi), pts_local) for lo, hi in shape.boxes], axis=0)
    if isinstance(shape, Segs):
        return np.minimum(shape.radius - _seg_dist(shape, pts_local[:, [0, 2]]), shape.half_w - np.abs(pts_local[:, 1]))
    if isinstance(shape, Box):
        lo, hi = np.array(shape.lo), np.array(shape.hi)
        return np.minimum(pts_local - lo, hi - pts_local).min(axis=1)
    a = "xyz".index(shape.axis)
    q = pts_local - np.array(shape.centre)
    axial = np.abs(q[:, a])
    radial = np.linalg.norm(np.delete(q, a, axis=1), axis=1)
    return np.minimum(shape.half_len - axial, shape.radius - radial)


def _samples(shape, step: float = 1.0) -> np.ndarray:
    """Points filling a shape (surface + interior), local frame."""
    if isinstance(shape, Boxes):
        return np.concatenate([_samples(Box(lo, hi), step) for lo, hi in shape.boxes])
    if isinstance(shape, Segs):  # discs along each segment, swept over y
        pts = []
        ys = np.linspace(-shape.half_w, shape.half_w, max(2, int(2 * shape.half_w / step) + 1))
        ang = np.linspace(0.0, 2 * math.pi, 16, endpoint=False)
        for x0, z0, x1, z1 in shape.segs:
            n = max(2, int(math.hypot(x1 - x0, z1 - z0) / step) + 1)
            for t in np.linspace(0.0, 1.0, n):
                cx, cz = x0 + t * (x1 - x0), z0 + t * (z1 - z0)
                for r in (0.0, 0.5 * shape.radius, shape.radius):
                    for a in ang if r > 0 else (0.0,):
                        for y in ys:
                            pts.append((cx + r * math.cos(a), y, cz + r * math.sin(a)))
        return np.array(pts)
    if isinstance(shape, Box):
        lo, hi = np.array(shape.lo), np.array(shape.hi)
        axes = [
            np.linspace(lo[i], hi[i], max(2, min(60, int(math.ceil((hi[i] - lo[i]) / step)) + 1))) for i in range(3)
        ]
        g = np.meshgrid(*axes, indexing="ij")
        return np.stack([x.ravel() for x in g], axis=1)
    a = "xyz".index(shape.axis)
    n_ang = 48
    radii = np.linspace(0.0, shape.radius, max(2, min(16, int(shape.radius / step) + 1)))
    axial = np.linspace(-shape.half_len, shape.half_len, max(2, min(60, int(2 * shape.half_len / step) + 1)))
    ang = np.linspace(0.0, 2 * math.pi, n_ang, endpoint=False)
    rr, aa, xx = np.meshgrid(radii, ang, axial, indexing="ij")
    u, v = rr * np.cos(aa), rr * np.sin(aa)
    other = [i for i in range(3) if i != a]
    pts = np.zeros((rr.size, 3))
    pts[:, a] = xx.ravel()
    pts[:, other[0]] = u.ravel()
    pts[:, other[1]] = v.ravel()
    return pts + np.array(shape.centre)


def penetration(a: Placed, b: Placed) -> float:
    """Largest penetration depth [mm] between the collision shapes of two parts (0 if separate)."""
    lo_a, hi_a = a.aabb()
    lo_b, hi_b = b.aabb()
    if np.any(lo_a > hi_b) or np.any(lo_b > hi_a):
        return 0.0
    best = 0.0
    for x, y in ((a, b), (b, a)):
        pts_w = (x.R @ _samples(x.bp.shape).T).T + x.p
        d = _depth_in_shape(y.bp.shape, (y.R.T @ (pts_w - y.p).T).T)
        best = max(best, float(d.max()))
    return max(best, 0.0)


STUD_R, STUD_H = 2.4, 1.7  # mm: a LEGO stud (diameter 4.8, height 1.7) on top of a part


def _stud_points(bp) -> np.ndarray:
    """Points inside every top stud of a part (local frame): centre and a ring at r = 2 mm, at 0.4 and 1.4 mm above
    the top face. Studs are visual only in the collision shapes, so the overlap check alone does not see them."""
    if not bp.studs_top:
        return np.zeros((0, 3))
    ring = [(0.0, 0.0)] + [(2.0 * math.cos(a), 2.0 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 6, endpoint=False)]
    return np.array([(x + dx, y + dy, bp.top_z + h) for x, y in bp.studs_top for dx, dy in ring for h in (0.4, 1.4)])


def stud_interference(P: dict, design) -> list[str]:
    """Parts that sit inside the studs of another part without being stud-connected to it: a body resting on (or
    pushed into) studs must either take them in a stud connection (its underside has the anti-studs) or sit at least
    1.7 mm above the top face."""
    stud_conn = {frozenset((c.child, c.parent)) for c in design.connections if c.kind == "stud"}
    errs = []
    for sname, s in P.items():
        pts_l = _stud_points(s.bp)
        if not len(pts_l):
            continue
        pts_w = (s.R @ pts_l.T).T + s.p
        lo, hi = pts_w.min(axis=0), pts_w.max(axis=0)
        for qname, q in P.items():
            if qname == sname:
                continue
            qlo, qhi = q.aabb()
            if np.any(qlo > hi) or np.any(lo > qhi):
                continue
            ql = (q.R.T @ (pts_w - q.p).T).T
            d = _depth_in_shape(q.bp.shape, ql)
            ux = getattr(q.bp, "underside_half_x", 0.0)
            if ux:  # rounded underside: below its flat face's level + a stud, the body only spans |x| <= ux
                d = np.where((np.abs(ql[:, 0]) > ux) & (ql[:, 2] < q.bp.bottom_z + STUD_H + 0.2), -1.0, d)
            d = d.reshape(len(s.bp.studs_top), -1)
            hit = np.nonzero(d.max(axis=1) > OVERLAP_TOL)[0]
            if len(hit) and frozenset((sname, qname)) in stud_conn:
                # stud-connected: every stud under q must sit in one of q's anti-stud positions
                if not q.bp.studs_bottom:
                    continue  # q's collision shape has no anti-stud grid to compare with (e.g. a printed part)
                anti = np.array([q.to_world((x, y, q.bp.bottom_z)) for x, y in q.bp.studs_bottom])
                centres = np.array([s.to_world((x, y, s.bp.top_z)) for x, y in s.bp.studs_top])[hit]
                dmin = np.linalg.norm(centres[:, None, :2] - anti[None, :, :2], axis=2).min(axis=1)
                off = int((dmin > STUD_XY_TOL).sum())
                if off:
                    errs.append(f"studs: {qname} is stud-connected to {sname} but {off} of {sname}'s studs under it do not "
                                f"land in {qname}'s anti-stud positions (off the 8 mm grid by up to {float(dmin.max()):.1f} mm): "
                                "align it on the stud grid.")
                continue
            if len(hit):
                errs.append(f"studs: {qname} sits inside {len(hit)} stud(s) of {sname} (up to {float(d.max()):.1f} mm deep). "
                            f"Either stud-connect {qname} onto {sname} (its underside must take studs) or move it at least "
                            f"{STUD_H} mm above {sname}'s top face; parts may not overlap the studs.")
    return errs


def _line_point_distance(p0: np.ndarray, d: np.ndarray, q: np.ndarray) -> tuple[float, float]:
    """Distance of point q from the line p0 + t d (unit d), and t."""
    t = float(np.dot(q - p0, d))
    return float(np.linalg.norm(q - (p0 + t * d))), t


def axis_vec(axis) -> np.ndarray:
    """Local axis of a hole / pin: 'x' | 'y' | 'z' or a unit vector."""
    if isinstance(axis, str):
        v = np.zeros(3)
        v["xyz".index(axis)] = 1.0
        return v
    v = np.asarray(axis, dtype=float)
    return v / np.linalg.norm(v)


def part_holes(bp: BenchPart) -> list[tuple[np.ndarray, np.ndarray, float, str]]:
    """Every hole of a part: (centre, local axis, half length, 'pin' | 'axle'): Technic brick / liftarm holes (along
    y), pin holes (``pin_holes``) and cross holes (``axle_holes``)."""
    out = []
    if bp.kind == "technic_brick":
        out += [(np.array([hx, 0.0, hz]), axis_vec("y"), bp.hole_half, "pin") for hx, hz in bp.holes]
    out += [(np.array([x, y, z]), axis_vec(a), bp.hole_half if bp.kind in ("technic_brick", "rack") else 4.0, "pin")
            for x, y, z, a in bp.pin_holes]
    out += [(np.array(h[:3], dtype=float), axis_vec(h[3]), h[4] if len(h) > 4 else 4.0, "axle") for h in bp.axle_holes]
    return out


def segment_in_holes(p0: np.ndarray, d: np.ndarray, centre: float, half: float, part: Placed, which=("pin", "axle"),
                     tol: float = 0.35, min_engage: float = 4.0):
    """Holes of ``part`` that a straight segment (pin section / axle) sits in: (index, kind, engaged mm). The segment
    runs along the unit vector d from p0 + (centre - half) d to p0 + (centre + half) d."""
    out = []
    for i, (c, a, hh, kind) in enumerate(part_holes(part.bp)):
        if kind not in which:
            continue
        aw = part.R @ a
        if abs(float(np.dot(d, aw))) < 0.999:
            continue
        cw = part.to_world(c)
        dist, t = _line_point_distance(p0, d, cw)
        engage = min(centre + half, t + hh) - max(centre - half, t - hh)
        if dist < tol and engage >= min(min_engage, 2 * hh) - 1e-6:
            out.append((i, kind, engage))
    return out


def axle_through_holes(axle: Placed, brick: Placed, tol: float = 0.35, min_engage: float = 0.1) -> list[int]:
    """Indices of the brick's Technic holes the axle runs in (axis parallel and on the hole centre line, at least
    ``min_engage`` mm of the axle inside the 8 mm long hole). Inventory parts (2026-10-06): every hole of the part
    (``part_holes``: liftarm / connector pin holes and cross holes)."""
    if axle.bp.kind != "axle":
        return []
    if brick.bp.inventory and (brick.bp.kind != "technic_brick" or brick.bp.axle_holes or brick.bp.pin_holes):
        hits = segment_in_holes(axle.p, axle.axis(), 0.0, axle.bp.axle_half_len, brick, tol=tol, min_engage=min_engage)
        return [i for i, _, _ in hits]
    if brick.bp.kind != "technic_brick":
        return []
    d = axle.axis()
    if abs(float(np.dot(d, brick.R[:, 1]))) < 0.999:
        return []
    half = axle.bp.axle_half_len
    out = []
    for i, (hx, hz) in enumerate(brick.bp.holes):
        c = brick.to_world((hx, 0.0, hz))
        dist, t = _line_point_distance(axle.p, d, c)
        engage = min(half, t + 4.0) - max(-half, t - 4.0)
        if dist < tol and engage >= min_engage:
            out.append(i)
    return out


# ----------------------------------------------------------------------------------------------------------------
# validation
# ----------------------------------------------------------------------------------------------------------------


def _owned_receivers() -> int:
    try:
        from ai_pd_tasks.lego_bench.parts import owned_inventory  # noqa: PLC0415

        return max(1, int(owned_inventory().get("58123c01", 1)))
    except Exception:
        return 1


IR_RECEIVERS = _owned_receivers()
"""PF IR receivers (8884 / 58123c01) the user owns (inventory + planned purchase: 2 since 2026-10-07): each has two
outputs (red A, blue B) on its own channel (1-4) (``Budget.ir_receivers``)."""
IR_OUTPUTS_PER_RECEIVER = 2
IR_LATENCY_S = float(os.environ.get("LEGO_BENCH_IR_LATENCY_S", "0.035") or 0.035)
"""Real command timing of the IR link [s]: one PF IR message lasts ~16 ms and the bridge sends each command several
times (the PF protocol's 5 repeats; the receiver acts on the first good copy), so a level change takes effect 20-50 ms
after the laptop issues it. The simulator delays every IR (PF level) command by this much when the real open-loop
rules are on (``env.MotorProgram``)."""


def bom_item_counts(design: Design) -> dict[str, int]:
    """Bill of materials in inventory items (a simulated composite such as 4185t counts one 4185 and one 2815)."""
    out: dict[str, int] = {}
    for pid, n in design.bom().items():
        for item in bom_items(pid):
            out[item] = out.get(item, 0) + n
    return out


@dataclass
class Budget:
    """What the designer may use: part counts (default: 9686 inventory parts with USD assets) and optional limits."""

    inventory: dict[str, int] = field(default_factory=lambda: dict(default_inventory()))
    max_parts: int | None = None
    max_mass_g: float | None = None
    # every design must be self-powered by a real LEGO motor on board: a motor part, a battery box, and a gear
    # train from the motor's output shaft to at least one wheel
    require_powered: bool = True
    # real-world, open-loop runs (the ``inventory`` part set): the laptop sends the motor program over IR (Uno IR
    # bridge -> PF receivers) with no feedback, so only timed triggers, PF steps and the owned IR receivers / leads
    # (``check_real_electrics``)
    real_open_loop: bool = False
    ir_receivers: int = 1

    @classmethod
    def for_part_set(cls, name: str = "9686", **kw) -> Budget:
        """Budget with a named part set ('9686': the default kit parts; 'extended': see parts.extended_inventory).
        The ``inventory`` set also turns on the real open-loop rules (``real_open_loop``, ``IR_RECEIVERS``)."""
        if name == "inventory":
            kw.setdefault("real_open_loop", True)
            kw.setdefault("ir_receivers", IR_RECEIVERS)
        return cls(inventory=part_set_inventory(name), **kw)


@dataclass
class ValidationReport:
    design: str
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    stats: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict:
        return {
            "design": self.design,
            "ok": self.ok,
            "errors": self.errors,
            "warnings": self.warnings,
            "stats": self.stats,
        }

    def summary(self) -> str:
        lines = [f"design '{self.design}': {'VALID' if self.ok else 'INVALID'}"]
        lines += [f"  error: {e}" for e in self.errors]
        lines += [f"  warning: {w}" for w in self.warnings]
        return "\n".join(lines)


STUD_Z_TOL = 0.3  # mm between mating faces
STUD_XY_TOL = 0.5
OVERLAP_TOL = 0.3  # mm of collision-shape interpenetration tolerated between unconnected parts
COAXIAL_TOL = 0.35


def _count_studs(upper: Placed, lower: Placed) -> tuple[int, float]:
    """Studs of ``lower``'s top face inside anti-stud positions of ``upper``'s bottom face; face gap [mm]."""
    if not upper.bp.studs_bottom or not lower.bp.studs_top:
        return 0, float("inf")
    # both parts must share their "up" axis (local z); parts tilted together (e.g. a boom turned about an axle, all
    # its bricks with the same spin) are measured in the lower part's frame
    if float(np.dot(upper.R[:, 2], lower.R[:, 2])) < 0.999:
        return 0, float("inf")
    a = np.array([lower.to_local(upper.to_world((x, y, upper.bp.bottom_z))) for x, y in upper.bp.studs_bottom])
    b = np.array([(x, y, lower.bp.top_z) for x, y in lower.bp.studs_top])
    gap = float(np.mean(a[:, 2])) - lower.bp.top_z
    dmin = np.linalg.norm(a[:, None, :2] - b[None, :, :2], axis=2).min(axis=1)
    return int((dmin < STUD_XY_TOL).sum()), abs(gap)


def _check_connection(c: DesignConnection, P: dict[str, Placed], errors: list[str], warnings: list[str]) -> None:
    if c.kind not in CONNECTION_KINDS:
        errors.append(f"connection {c.child}->{c.parent}: unknown kind '{c.kind}' (one of {CONNECTION_KINDS})")
        return
    if c.child not in P or c.parent not in P:
        errors.append(f"connection {c.child}->{c.parent}: unknown part")
        return
    ch, pa = P[c.child], P[c.parent]
    tag = f"{c.kind} {c.child}->{c.parent}"
    if c.kind == "stud":
        n1, g1 = _count_studs(ch, pa)  # child on top of parent
        n2, g2 = _count_studs(pa, ch)  # child under parent
        n, gap = (n1, g1) if g1 <= g2 else (n2, g2)
        if gap > STUD_Z_TOL:
            errors.append(f"{tag}: faces do not touch (gap {gap:.2f} mm; child must sit on top of or under the parent)")
        elif n == 0:
            errors.append(f"{tag}: no studs line up (check the 8 mm stud grid)")
        elif c.studs and c.studs > n:
            errors.append(f"{tag}: declares {c.studs} studs but only {n} engage")
        elif c.studs and c.studs < n:
            warnings.append(f"{tag}: declares {c.studs} studs, geometry engages {n} (break force uses {c.studs})")
    elif c.kind in ("cross", "hinge") and (ch.bp.inventory or pa.bp.inventory):
        _check_inventory_axle_fit(c, ch, pa, tag, errors, warnings)
    elif c.kind in ("cross", "hinge"):
        if pa.bp.kind != "axle" and c.kind == "cross":
            errors.append(f"{tag}: parent must be an axle (cross fit), got {pa.bp.kind}")
            return
        if c.kind == "hinge":
            if ch.bp.kind != "axle" or pa.bp.kind != "technic_brick":
                errors.append(f"{tag}: a hinge is an axle (child) in a Technic brick hole (parent)")
                return
            if not axle_through_holes(ch, pa, min_engage=4.0):
                errors.append(
                    f"{tag}: the axle does not pass through a hole of {c.parent} (>= 4 mm engaged; "
                    f"holes along its local y, 4 mm below its top, x = multiples of 8 mm between the studs)"
                )
            return
        if ch.bp.kind not in ("gear", "wheel", "bush", "axle_brick"):
            errors.append(f"{tag}: cross child must be a gear, wheel, bush or axle brick (32064), got {ch.bp.kind}")
            return
        d = pa.axis()
        if abs(float(np.dot(d, ch.axis()))) < 0.999:
            errors.append(f"{tag}: {c.child}'s axis is not parallel to the axle")
            return
        dist, t = _line_point_distance(pa.p, d, ch.p)
        if dist > COAXIAL_TOL:
            errors.append(f"{tag}: {c.child} is {dist:.2f} mm off the axle centre line")
        # the cross bore (hub) of the child, measured along the axle from the axle centre
        s = float(np.dot(ch.axis(), d))
        ends = [t + s * h for h in ch.bp.hub]
        overhang = max(abs(e) for e in ends) - pa.bp.axle_half_len
        if overhang > 1.0:
            errors.append(f"{tag}: {c.child} hangs {overhang:.2f} mm off the end of the axle")
        elif overhang > 0.0:
            warnings.append(f"{tag}: {c.child} overhangs the axle end by {overhang:.2f} mm")
    elif c.kind == "pin" and (ch.bp.inventory or pa.bp.inventory):
        if not pin_fit(ch, pa):
            errors.append(
                f"{tag}: no pin section of {c.child} sits in a hole of {c.parent} (a pin / integral pin in a pin hole, "
                f"an axle-pin's axle end in an axle or pin hole; coaxial within {COAXIAL_TOL} mm, >= "
                f"{PIN_MIN_ENGAGE:.0f} mm inside the hole, 2 mm for a thin liftarm)"
            )
    elif c.kind == "pin":
        if ch.bp.kind != "pin":
            errors.append(f"{tag}: a pin connection is a Technic pin (child) in a pin hole of the parent")
            return
        holes = pin_hole_frames(pa.bp)
        if not holes:
            errors.append(f"{tag}: {c.parent} ({pa.bp.kind}) has no pin holes")
            return
        if not pin_in_holes(ch, pa):
            errors.append(
                f"{tag}: the pin is not in a pin hole of {c.parent} (coaxial within {COAXIAL_TOL} mm, >= "
                f"{PIN_MIN_ENGAGE:.0f} mm inside the 8 mm hole)"
            )
    elif c.kind == "motor":
        if pa.bp.kind != "motor" or ch.bp.kind != "axle":
            errors.append(f"{tag}: a motor connection is an axle (child) in the motor output (parent)")
            return
        d = pa.R[:, 1]
        if abs(float(np.dot(ch.axis(), d))) < 0.999:
            errors.append(f"{tag}: the shaft is not along the motor output axis")
            return
        if pa.bp.male_axles:  # round 4b: the train motor's outputs are its integral axles
            errors.append(f"{tag}: {c.parent}'s outputs are its integral axles: cross-fit wheels / gears on them "
                          f"(a 'cross' connection with {c.parent} as the parent), no motor connection")
            return
        # shaft segment along the motor axis, relative to the output face (t = 0, body at t > 0)
        dist, t = _line_point_distance(pa.p, d, ch.p)
        if dist > COAXIAL_TOL:
            errors.append(f"{tag}: the shaft is {dist:.2f} mm off the motor output axis")
        inside = min(t + ch.bp.axle_half_len, 8.0) - max(t - ch.bp.axle_half_len, 0.0)
        outside = max(0.0, -(t - ch.bp.axle_half_len))
        if pa.bp.through_bore:  # round 4b (pullback): a bore through the body, y = -B..B; the shaft sticks out a side
            B, h = pa.bp.through_bore, ch.bp.axle_half_len
            inside = min(t + h, B) - max(t - h, -B)
            outside = max(t + h - B, -B - (t - h), 0.0)
        if inside < 3.0:
            errors.append(f"{tag}: the shaft enters the motor output by {inside:.1f} mm (needs >= 3 mm)")
        if outside < 3.0:
            errors.append(f"{tag}: the shaft sticks out of the motor by only {outside:.1f} mm")


def _check_inventory_axle_fit(c, ch: Placed, pa: Placed, tag: str, errors: list[str], warnings: list[str]) -> None:
    """Cross / hinge connections that involve an inventory part (2026-10-06): liftarms with cross holes, connectors,
    free-turning gears (round bore), axle-pins (axle end as the cross parent), the new Technic bricks and wheels."""
    if c.kind == "hinge":
        if ch.bp.kind != "axle":
            errors.append(f"{tag}: a hinge is an axle (child) turning in a hole of the parent")
            return
        if pa.bp.free_bore:  # a gear turning freely on the axle
            d = ch.axis()
            if abs(float(np.dot(d, pa.axis()))) < 0.999:
                errors.append(f"{tag}: {c.parent}'s axis is not parallel to the axle")
                return
            dist, t = _line_point_distance(ch.p, d, pa.p)
            if dist > COAXIAL_TOL:
                errors.append(f"{tag}: {c.parent} is {dist:.2f} mm off the axle centre line")
            elif abs(t) + 0.5 * pa.bp.width - ch.bp.axle_half_len > 1.0:
                errors.append(f"{tag}: {c.parent} hangs off the end of the axle")
            return
        if pa.bp.kind == "technic_brick" and not pa.bp.axle_holes and not pa.bp.pin_holes:
            if not axle_through_holes(ch, pa, min_engage=min(4.0, 2 * pa.bp.hole_half)):
                errors.append(f"{tag}: the axle does not pass through a hole of {c.parent} (>= 4 mm engaged)")
            return
        hits = segment_in_holes(ch.p, ch.axis(), 0.0, ch.bp.axle_half_len, pa, min_engage=4.0)
        if not hits:
            errors.append(f"{tag}: the axle does not pass through a pin hole of {c.parent} (coaxial within "
                          f"{COAXIAL_TOL} mm, >= 4 mm engaged; 2 mm in a thin liftarm)")
        elif all(k == "axle" for _, k, _ in hits):
            errors.append(f"{tag}: the axle sits in an axle (cross) hole of {c.parent}: it cannot turn there (use a "
                          f"'cross' connection with {c.parent} as the child)")
        return
    # cross: parent = an axle (or the axle end of an axle-pin); child = gear / wheel / bush / axle brick, or a part
    # with cross holes (liftarm, connector)
    p0 = pa.p
    if pa.bp.kind == "axle":
        spans = [(0.0, pa.bp.axle_half_len)]
    elif pa.bp.kind == "pin" and pa.bp.axle_segments:
        spans = list(pa.bp.axle_segments)
    elif pa.bp.male_axles:  # round 4a: an integral axle end of a hub (57585): the one the child sits on
        p0, d, spans = _male_axle_line(pa, ch)
    else:
        errors.append(f"{tag}: parent must be an axle or an axle-pin's axle end (cross fit), got {pa.bp.kind}")
        return
    if ch.bp.free_bore:
        errors.append(f"{tag}: {c.child} has a round bore and turns freely on an axle (use a 'hinge' connection with "
                      f"the axle as the child and {c.child} as the parent)")
        return
    if not pa.bp.male_axles:
        d = pa.axis()
    if ch.bp.kind in ("gear", "wheel", "bush", "axle_brick"):
        if abs(float(np.dot(d, ch.axis()))) < 0.999:
            errors.append(f"{tag}: {c.child}'s axis is not parallel to the axle")
            return
        dist, t = _line_point_distance(p0, d, ch.p)
        if dist > COAXIAL_TOL:
            errors.append(f"{tag}: {c.child} is {dist:.2f} mm off the axle centre line")
        s_ = float(np.dot(ch.axis(), d))
        ends = sorted(t + s_ * h for h in ch.bp.hub)
        over = min(max(ends[1] - (cx + h), (cx - h) - ends[0]) for cx, h in spans)
        if over > 1.0:
            errors.append(f"{tag}: {c.child} hangs {over:.2f} mm off the end of the axle")
        elif over > 0.0:
            warnings.append(f"{tag}: {c.child} overhangs the axle end by {over:.2f} mm")
        return
    if not ch.bp.axle_holes:
        errors.append(f"{tag}: {c.child} ({ch.bp.kind}) has no axle (cross) hole")
        return
    if not any(segment_in_holes(p0, d, cx, h, ch, which=("axle",), min_engage=2.0) for cx, h in spans):
        errors.append(f"{tag}: the axle does not pass through a cross hole of {c.child} (coaxial within "
                      f"{COAXIAL_TOL} mm, >= 2 mm engaged)")


def male_axle_lines(part: Placed) -> list[tuple[np.ndarray, np.ndarray, float]]:
    """World (middle, unit direction, half length) of a part's integral axle ends (``male_axles``, round 4a: 57585)."""
    return [(part.to_world((x, y, z)), part.R @ axis_vec(a), float(h)) for x, y, z, a, h in part.bp.male_axles]


def _male_axle_line(pa: Placed, ch: Placed):
    """The integral axle end of ``pa`` that ``ch`` is cross-fitted on: (point, direction, spans) of the best match (a
    parallel axis through the child's centre, or one of the child's cross holes on it); the first one otherwise."""
    best, best_d = None, np.inf
    for p0, d, h in male_axle_lines(pa):
        if ch.bp.kind in ("gear", "wheel", "bush", "axle_brick"):
            dist = _line_point_distance(p0, d, ch.p)[0] if abs(float(np.dot(d, ch.axis()))) >= 0.999 else np.inf
        else:
            dist = 0.0 if segment_in_holes(p0, d, 0.0, h, ch, which=("axle",), min_engage=2.0) else np.inf
        if best is None or dist < best_d:
            best, best_d = (p0, d, [(0.0, h)]), dist
    return best


def pin_sections(pin: Placed) -> list[tuple[float, float, str]]:
    """(centre, half length, 'pin' | 'axle') of a pin's sections along its local x."""
    bp = pin.bp
    segs = [(cx, h, "pin") for cx, h in (bp.pin_segments or ((0.0, bp.axle_half_len),))]
    return segs + [(cx, h, "axle") for cx, h in bp.axle_segments]


def pin_fit(ch: Placed, pa: Placed) -> list[tuple]:
    """How a pin-like child sits in the parent's holes: [(section, hole index, hole kind)]. A pin section goes in a
    pin hole; an axle-pin's axle end in a cross hole (locked) or a pin hole; a connector's integral pins
    (``male_pins``) in pin holes."""
    out = []
    if ch.bp.kind == "pin":
        d = ch.axis()
        for cx, h, kind in pin_sections(ch):
            for i, hk, _ in segment_in_holes(ch.p, d, cx, h, pa, which=("pin", "axle") if kind == "axle" else ("pin",),
                                             min_engage=PIN_MIN_ENGAGE):
                out.append(((cx, h, kind), i, hk))
    for x, y, z, a, h in ch.bp.male_pins:
        d = ch.R @ axis_vec(a)
        c = ch.to_world((x, y, z))
        for i, hk, _ in segment_in_holes(c, d, 0.0, h, pa, which=("pin",), min_engage=PIN_MIN_ENGAGE):
            out.append(((x, y, z, a), i, hk))
    for c, d, h in male_axle_lines(ch):  # round 4a: a hub's axle ends, in a cross hole (locked) or a pin hole
        for i, hk, _ in segment_in_holes(c, d, 0.0, h, pa, which=("pin", "axle"), min_engage=PIN_MIN_ENGAGE):
            out.append(((*c, "axle"), i, hk))
    return out


def pin_in_holes(pin: Placed, part: Placed, tol: float = COAXIAL_TOL, min_engage: float = PIN_MIN_ENGAGE) -> list[int]:
    """Indices of ``part``'s pin holes (``parts.pin_hole_frames``) that the pin sits in."""
    d = pin.axis()
    half = pin.bp.axle_half_len
    out = []
    for i, (centre, axis) in enumerate(pin_hole_frames(part.bp)):
        a = part.R[:, "xyz".index(axis)]
        if abs(float(np.dot(d, a))) < 0.999:
            continue
        c = part.to_world(centre)
        dist, t = _line_point_distance(pin.p, d, c)
        engage = min(half, t + 4.0) - max(-half, t - 4.0)
        if dist < tol and engage >= min_engage:
            out.append(i)
    return out


def _components(nodes: list[str], edges: list[tuple[str, str]]) -> list[set[str]]:
    parent = {n: n for n in nodes}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b in edges:
        if a in parent and b in parent:
            parent[find(a)] = find(b)
    comps: dict[str, set[str]] = {}
    for n in nodes:
        comps.setdefault(find(n), set()).add(n)
    return list(comps.values())


def shafts(design: Design) -> dict[str, str]:
    """Part name -> shaft (axle) name for every axle and every part cross-fitted to one. Inventory parts: an axle-pin
    (its axle end) and a free-turning gear (round bore) are shafts of their own; a connector cross-fitted on two axles
    (32034, 6536 on an axle stub) joins them into one shaft (named after the first axle)."""
    cat = bench_parts()
    kinds = {p.name: cat[p.part] for p in design.parts if p.part in cat}
    out = {n: n for n, bp in kinds.items() if bp.kind == "axle"}
    roots = dict(out)
    roots.update({n: n for n, bp in kinds.items() if (bp.kind == "pin" and bp.axle_segments) or bp.free_bore
                  or bp.male_axles or bp.kind == "differential"})
    if all(not bp.inventory for bp in kinds.values()):  # the earlier part sets: unchanged
        for c in design.connections:
            if c.kind == "cross" and c.parent in out:
                out[c.child] = out[c.parent]
        return out
    parent = {n: n for n in kinds}
    order = {p.name: i for i, p in enumerate(design.parts)}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for c in design.connections:
        if c.kind == "cross" and c.parent in roots and c.child in parent:
            ra, rb = find(c.child), find(c.parent)
            if ra != rb:
                parent[max(ra, rb, key=lambda n: order[n])] = min(ra, rb, key=lambda n: order[n])
    groups: dict[str, list[str]] = {}
    for n in kinds:
        groups.setdefault(find(n), []).append(n)
    out = {}
    for members in groups.values():
        rs = [n for n in members if n in roots]
        if not rs:
            continue
        rep_ = min(rs, key=lambda n: (kinds[n].kind != "axle", order[n]))
        out.update({n: rep_ for n in members})
    return out


def pivot_connections(design: Design) -> set[int]:
    """Indices of the pin connections that are revolute joints: a frictionless pin (3673, 65249) turns in the holes of
    the parts it joins. Each pivot pin keeps one connection fixed (its axle end's, else its first) so it stays a body
    of the assembly; its other pin connections turn about the pin's axis."""
    cat = bench_parts()
    parts = {p.name: p for p in design.parts}
    out: set[int] = set()
    for name, p in parts.items():
        bp = cat.get(p.part)
        if bp is None or not bp.pivot:
            continue
        idx = [i for i, c in enumerate(design.connections) if c.kind == "pin" and c.child == name]
        if not idx:
            continue
        P = place(Design(design.name, [parts[name]] + [parts[design.connections[i].parent] for i in idx
                                                       if design.connections[i].parent in parts], []))
        fixed = idx[0]
        for i in idx:
            pa = design.connections[i].parent
            if pa in P and any(sec[2] == "axle" for sec, _, _ in pin_fit(P[name], P[pa])):
                fixed = i
                break
        out |= {i for i in idx if i != fixed}
    return out | r4b.shock_pivot_connections(design)  # round 4b: a shock's end turns on its pin


def rigid_edges(design: Design) -> list[tuple[str, str]]:
    """(child, parent) of the connections that tie two parts rigidly (stud, pin; not a pivot pin's turning ones)."""
    piv = pivot_connections(design)
    return [(c.child, c.parent) for i, c in enumerate(design.connections) if c.kind in RIGID_KINDS and i not in piv]


MOTORS_PER_BATTERY = 3
"""Most motors one PF battery box may power. LEGO's own models run up to three or four PF motors from one box (e.g.
9398 4x4 Crawler: 2 XL + 1 L + servo on one AA box); the benchmark allows three per box (wiring and voltage sag
are not simulated)."""


PF_SPEED_STEPS = 7
"""Speed levels of the motor program: -7..7, i.e. the 7 forward / 7 reverse PWM steps of the LEGO Power Functions IR
speed remote (8879) and receiver (8884); 0 stops the motor (the drive holds it: brake). A Powered Up hub has finer
steps; the benchmark uses the coarser PF set."""
MOTOR_PROGRAM_MIN_DT = 0.1
"""At most one command per 0.1 s (a real IR remote / hub cannot switch faster; position-triggered commands that fire
closer together are delayed)."""
MOTOR_PROGRAM_MAX_COMMANDS = 200  # 40 until 2026-10-13 (user: programs may be complex)


HUB_PORTS = ("A", "B", "C", "D", "E", "F")
HUB_TICK_S = 0.01
"""SPIKE hub program loop / sensor sample period (100 Hz sensors and motor encoders)."""
SENSOR_OPS = ("lt", "gt", "eq")


def device_system(bp) -> str:
    """'pf' (Power Functions: battery box, IR receiver, no sensors) or 'lpf2' (SPIKE / Powered Up hub devices)."""
    return getattr(bp, "system", "pf") or "pf"


def check_motor_program(design: Design, real: bool = False) -> list[str]:
    """Errors of a design's ``motor_program`` (and hub wiring).

    A command fires once, the first time all its triggers hold:

    * ``"t": s`` -- at least s seconds after the start signal;
    * ``"x": m`` -- the chassis origin has passed x = m in the course frame (an operator watching, or odometry);
    * ``"sensor": name, "lt" | "gt" | "eq": value`` -- a hub sensor reading (distance [mm] or None when out of range,
      force sensor ``touch`` 0/1, color sensor LEGO colour name, hub ``"tilt_pitch"`` / ``"tilt_roll"`` [deg]);
    * ``"angle": motor, "lt" | "gt": deg`` -- the encoder of an LPF2 motor (degrees turned since the start);
    * ``"after": i`` (+ optional ``"delay": s``) -- command i has fired (s seconds ago); with ``"since_after": true`` an
      angle trigger counts from the encoder angle when command i fired (odometry: run N degrees).

    Action: ``"motor"`` (a motor with an output shaft, or ``"all"``) and ``"level"`` (Power Functions speed step -7..7)
    or ``"speed_pct"`` (LPF2 motor, -100..100). 0 stops the motor: a PF motor brakes, an LPF2 motor holds its angle.
    For a motor that drives wheels the sign is relative to forward, for any other motor it is the motor's own sense.
    Sensor and angle triggers need a hub (``design.hub``) with the device on a port. Commands for PF motors come
    from an IR remote: at least 0.1 s apart."""
    prog = design.motor_program
    errs: list[str] = []
    parts = {p.name: p for p in design.parts}
    cat = bench_parts()
    kind = {n: cat[p.part].kind for n, p in parts.items() if p.part in cat}
    system = {n: device_system(cat[p.part]) for n, p in parts.items() if p.part in cat}
    motors = {c.parent for c in motor_connections(design)}
    pull = pullback_motors(design)
    # -- hub wiring --
    if design.hub:
        if design.hub not in parts or kind.get(design.hub) != "hub":
            errs.append(f"hub '{design.hub}' is not a hub part (SPIKE Prime Large Hub 53444c01)")
        for port, dev in design.ports.items():
            if port not in HUB_PORTS:
                errs.append(f"hub port '{port}' does not exist (A-F)")
            if dev not in parts or kind.get(dev) not in ("motor", "sensor") or system.get(dev) != "lpf2":
                errs.append(f"hub port {port}: '{dev}' is not an LPF2 motor or sensor of the design")
        devs = list(design.ports.values())
        for d in {d for d in devs if devs.count(d) > 1}:
            errs.append(f"'{d}' is on more than one hub port")
    for n in parts:
        if system.get(n) == "lpf2" and kind.get(n) in ("motor", "sensor") and n not in design.ports.values():
            errs.append(f"'{n}' (LPF2 device) must be plugged into a hub port (design.ports)")
    for n in motors:
        if system.get(n) == "pf" and not any(kind.get(q) == "battery" for q in parts):
            errs.append(f"PF motor '{n}' needs a PF battery box (87513c01 AAA or 59510c01 AA)")
    if not prog:
        return errs
    if real:  # open loop over IR: timed commands only, PF steps (REAL_TRIGGERS, ir_mapping)
        for i, c in enumerate(prog):
            bad = sorted(k for k in ("x", "sensor", "angle", "since_after", "speed_pct") if k in c)
            if bad:
                errs.append(f"motor program command {i}: real runs are open loop over IR -- only 't' or 'after' "
                            f"(+ 'delay') triggers and PF 'level' steps; remove {bad}")
    if len(prog) > MOTOR_PROGRAM_MAX_COMMANDS:
        errs.append(f"motor program: {len(prog)} commands (at most {MOTOR_PROGRAM_MAX_COMMANDS})")
    last_pf_t = None
    for i, c in enumerate(prog):
        tag = f"motor program command {i}"
        extra = set(c) - {"t", "x", "sensor", "angle", "after", "delay", "lt", "gt", "eq", "motor", "level",
                          "speed_pct", "note", "since_after"}  # fmt: skip
        if c.get("since_after") and not ("angle" in c and "after" in c):
            errs.append(f"{tag}: 'since_after' measures an angle trigger from the 'after' command's firing")
        if extra:
            errs.append(f"{tag}: unknown fields {sorted(extra)}")
        if not any(k in c for k in ("t", "x", "sensor", "angle", "after")):
            errs.append(f"{tag}: needs a trigger ('t', 'x', 'sensor', 'angle' or 'after')")
        if ("sensor" in c or "angle" in c) and sum(k in c for k in SENSOR_OPS) != 1:
            errs.append(f"{tag}: a sensor / angle trigger needs exactly one of 'lt', 'gt', 'eq'")
        if "sensor" in c:
            sn = c["sensor"]
            if sn not in ("tilt_pitch", "tilt_roll") and (kind.get(sn) != "sensor" or sn not in design.ports.values()):
                errs.append(f"{tag}: sensor '{sn}' is not a sensor on a hub port")
            if sn in ("tilt_pitch", "tilt_roll") and not design.hub:
                errs.append(f"{tag}: tilt needs the hub's IMU (design.hub)")
        if "angle" in c and (c["angle"] not in motors or system.get(c["angle"]) != "lpf2" or c["angle"] not in design.ports.values()):
            errs.append(f"{tag}: angle trigger needs an LPF2 motor on a hub port (encoder), got '{c['angle']}'")
        if "after" in c and not (isinstance(c["after"], int) and 0 <= c["after"] < len(prog) and c["after"] != i):
            errs.append(f"{tag}: 'after' must be the index of another command")
        m = c.get("motor")
        targets = sorted(motors - pull) if m == "all" else [m]
        if m in pull:
            errs.append(f"{tag}: '{m}' is a pullback motor: it is wound before the run and runs from its release, it "
                        "takes no commands")
        elif m != "all" and m not in motors:
            errs.append(f"{tag}: motor '{m}' is not a motor with an output shaft (or 'all')")
        if ("level" in c) == ("speed_pct" in c):
            errs.append(f"{tag}: give exactly one of 'level' (PF) or 'speed_pct' (LPF2)")
        if "level" in c:
            lv = c["level"]
            if real and lv == "brake":
                lv = 0
            if not isinstance(lv, int) or isinstance(lv, bool) or abs(lv) > PF_SPEED_STEPS:
                errs.append(f"{tag}: level must be an integer in -{PF_SPEED_STEPS}..{PF_SPEED_STEPS} (PF speed steps)")
            if any(system.get(t) == "lpf2" for t in targets):
                errs.append(f"{tag}: 'level' is for PF motors; use 'speed_pct' for LPF2 motors")
        if "speed_pct" in c:
            sp = c["speed_pct"]
            if not isinstance(sp, int) or isinstance(sp, bool) or abs(sp) > 100:
                errs.append(f"{tag}: speed_pct must be an integer in -100..100")
            if any(system.get(t) == "pf" for t in targets):
                errs.append(f"{tag}: 'speed_pct' is for LPF2 motors; use 'level' for PF motors")
            if not design.hub:
                errs.append(f"{tag}: LPF2 motors are driven by a hub program (design.hub)")
        if "t" in c:
            t = c["t"]
            if not isinstance(t, (int, float)) or t < 0:
                errs.append(f"{tag}: t must be >= 0")
            elif "level" in c and not any(k in c for k in ("sensor", "angle", "after", "x")):
                if last_pf_t is not None and t - last_pf_t < MOTOR_PROGRAM_MIN_DT - 1e-9:
                    errs.append(f"{tag}: less than {MOTOR_PROGRAM_MIN_DT} s after the previous command")
                else:
                    last_pf_t = t
        if "x" in c and not isinstance(c["x"], (int, float)):
            errs.append(f"{tag}: x must be a number (m)")
        if ("sensor" in c or "angle" in c) and "level" in c:
            errs.append(f"{tag}: sensor / angle triggers run on the hub and drive LPF2 motors (speed_pct)")
    return errs


def motor_schedule(design: Design) -> dict[str, dict]:
    """How each PF motor is commanded: {motor: {"constant": level or None, "commands": [indices]}}. A motor is
    constant when it runs at one level from t = 0 to the end: no motor program (every motor at full speed from the
    launch), or only commands at t = 0 without other triggers, all at the same level."""
    pull = pullback_motors(design)  # round 4b: a pullback motor is not wired or commanded
    motors = [c.parent for c in motor_connections(design) if c.parent not in pull]
    prog = design.motor_program
    out = {}
    for m in motors:
        idx = [i for i, c in enumerate(prog) if c.get("motor") in (m, "all")]
        if not prog:
            out[m] = {"constant": PF_SPEED_STEPS, "commands": []}
            continue
        levels = {prog[i].get("level") for i in idx}
        at_start = all(set(prog[i]) & {"t", "x", "sensor", "angle", "after"} == {"t"} and prog[i]["t"] == 0 for i in idx)
        const = next(iter(levels)) if idx and at_start and len(levels) == 1 else (0 if not idx else None)
        out[m] = {"constant": const, "commands": idx}
    return out


def _device_point(pl: Placed) -> np.ndarray:
    """Where a lead attaches, approximated by the part's shape centre (world, mm)."""
    s = pl.bp.shape
    c = 0.5 * (np.array(s.lo) + np.array(s.hi)) if isinstance(s, Box) else np.array(s.centre)
    return pl.to_world(c)


LEAD_BEND_MM = 20.0
"""A lead leaves its part and its plug at right angles: the run between them is the straight distance + this."""
PF_PLUG_H, PF_PLUG_HALF, PF_PLUG_BOOT = 6.4, 8.0, 6.0
"""A PF plug (LDraw 58124): 16 x 16 mm, 6.4 mm per plug in a stack (+ studs), its cable boot 4-6 mm out on one side."""


def _plug_room(P: dict, box: str, n: int) -> list[str]:
    """Room for a stack of ``n`` PF plugs on a battery box's pad: nothing else within the plug's footprint (+ 0.5 mm)
    up to the stack's top + 3 mm, and one side of the stack free for the cable boot."""
    import dataclasses  # noqa: PLC0415

    bx = P[box]
    if not bx.bp.pf_pads:
        return []
    px, py, pz = bx.bp.pf_pads[0]
    h = PF_PLUG_H * n
    hw = PF_PLUG_HALF + 0.5

    def vol(lo, hi):
        return Placed(bx.inst, dataclasses.replace(bx.bp, shape=Box(tuple(lo), tuple(hi))), bx.R, bx.p)

    others = [q for name, q in P.items() if name != box]
    core = vol((px - hw, py - hw, pz + 0.2), (px + hw, py + hw, pz + h + 3.0))
    hit = [q.inst.name for q in others if penetration(core, q) > 0.2]
    if hit:
        return [f"battery box '{box}': its PF connector is blocked by {', '.join(sorted(hit)[:4])}: {n} plug(s) need a "
                f"clear {2 * PF_PLUG_HALF:.0f} x {2 * PF_PLUG_HALF:.0f} mm space {h + 3:.0f} mm tall on top of it "
                f"(box-local x {px - PF_PLUG_HALF:.0f}..{px + PF_PLUG_HALF:.0f}, y {py - PF_PLUG_HALF:.0f}.."
                f"{py + PF_PLUG_HALF:.0f})"]
    sides = [((px + hw, py - PF_PLUG_HALF, pz + 0.2), (px + hw + PF_PLUG_BOOT, py + PF_PLUG_HALF, pz + h)),
             ((px - hw - PF_PLUG_BOOT, py - PF_PLUG_HALF, pz + 0.2), (px - hw, py + PF_PLUG_HALF, pz + h)),
             ((px - PF_PLUG_HALF, py + hw, pz + 0.2), (px + PF_PLUG_HALF, py + hw + PF_PLUG_BOOT, pz + h)),
             ((px - PF_PLUG_HALF, py - hw - PF_PLUG_BOOT, pz + 0.2), (px + PF_PLUG_HALF, py - hw, pz + h))]
    if not any(all(penetration(vol(lo, hi), q) <= 0.2 for q in others) for lo, hi in sides):
        return [f"battery box '{box}': no side of its PF connector is free for the plug's cable ({PF_PLUG_BOOT:.0f} mm "
                "beside the plug on at least one side)"]
    return []


def ir_mapping(design: Design, ir_receivers: int | None = None) -> dict:
    """Deterministic wiring of a design's PF motors for a real open-loop run (used by the real-world runner and the
    build guides):

    * motors in order: wheel motors first, then the others (design part order);
    * every motor whose level changes during the run ("sequenced") needs an IR receiver output: receiver k (design
      part order) is set to channel k + 1, its outputs red A then blue B;
    * free outputs go to the remaining motors in the same order (the runner can stop them), and a motor that runs
      at full speed (|level| 7) from t = 0 to the end may instead plug straight into a battery box (it starts when
      the box is switched on at launch; a box has no speed control, so only |level| 7 can be box-direct);
    * each receiver and box-direct motor plugs into the nearest battery box.

    Returns {"outputs": {motor: {"receiver", "channel", "output", "level_sign"}}, "box_direct": {motor: box},
    "receivers": {receiver: {"channel", "box"}}, "idle": [motors never switched on], "errors": [...]}."""
    from ai_pd_tasks.lego.catalog import MOTOR_LEAD_MM, RECEIVER_LEAD_MM  # noqa: PLC0415

    n_rx = IR_RECEIVERS if ir_receivers is None else ir_receivers
    P = place(design)
    sched = motor_schedule(design)
    wheels = wheel_motors(design) if sched else set()
    order = {p.name: i for i, p in enumerate(design.parts)}
    motors = sorted(sched, key=lambda m: (m not in wheels, order.get(m, 0)))
    receivers = sorted((n for n, pl in P.items() if pl.bp.kind == "receiver"), key=lambda n: order[n])
    boxes = [n for n, pl in P.items() if pl.bp.kind == "battery"]
    errors: list[str] = []
    if len(receivers) > n_rx:
        errors.append(f"{len(receivers)} IR receivers on board, {n_rx} owned (IR_RECEIVERS)")
    receivers = receivers[:n_rx]
    slots = [(rx, k + 1, out) for k, rx in enumerate(receivers) for out in ("A", "B")]
    idle = [m for m in motors if sched[m]["constant"] == 0]
    active = [m for m in motors if m not in idle]
    seq = [m for m in active if sched[m]["constant"] is None]
    const = [m for m in active if m not in seq]
    # PF plugs stack: motors share one receiver output only if they get the same voltage the whole run, i.e. their
    # commands are identical once each is turned into the motor's own turning sense (seen from its output face). A
    # stacked plug cannot be put on turned round (2026-10-20, checked on the bench), so two motors that must turn
    # opposite ways (e.g. mirror-image wheel motors) need two outputs. Group key = trigger fields + signed polarity.
    prog = design.motor_program
    _, _rel, _ = motor_trains(design, P) if sched else ({}, {}, [])
    _dir = (compute_stats(design, P).get("motor_direction", 1) or 1) if sched else 1
    _wheels = wheel_motors(design) if sched else set()
    _shaft = {c.parent: c.child for c in motor_connections(design)}

    def _face_sign(m):  # +1: a +level turns the motor counter-clockwise seen from its output face
        sh = _shaft.get(m)
        if sh is None or sh not in P:
            return 1
        out_dir = -P[m].R[:, 1]
        c = 1 if float(np.dot(P[sh].axis(), out_dir)) >= 0 else -1
        return int((_dir if m in _wheels else 1) * _rel.get(m, 1) * c)

    def _key(m):
        cmds = [prog[i] for i in sched[m]["commands"]]
        trig = [tuple((k, c[k]) for k in sorted(c) if k not in ("motor", "level", "note")) for c in cmds]
        sg = _face_sign(m)
        lv = [c.get("level") for c in cmds]
        return (tuple(trig), tuple(v * sg if isinstance(v, (int, float)) else v for v in lv)), 1

    groups: dict = {}
    for m in seq:
        key, sg = _key(m)
        groups.setdefault(key, []).append((m, sg))
    seq_groups = list(groups.values())
    if len(seq_groups) > len(slots):
        errors.append(
            f"{len(seq_groups)} different motor programs change level during the run ({', '.join(seq)}) but the design "
            f"has {len(slots)} IR receiver outputs ({len(receivers)} receiver(s) x 2; {n_rx} owned): each program needs "
            "its own receiver output (motors can share one by stacking their plugs only if they must turn the same "
            "way at the same speed the whole run, seen from each motor's output face: a stacked plug cannot be "
            "turned round, so mirror-image motors need two outputs; a motor plugged into a battery box only runs at "
            "full speed from the launch)"
        )
    n_free = max(len(slots) - len(seq_groups), 0)
    _, rel, _ = motor_trains(design, P) if sched else ({}, {}, [])
    outputs, box_direct = {}, {}
    k = 0
    seq_set = set(seq)
    const_ir = set(const[:n_free])
    for m in motors:  # design order (wheel motors first), as before stacking existed
        if m in outputs or k >= len(slots):
            continue
        if m in seq_set:
            grp = next(g for g in seq_groups if any(x == m for x, _ in g))
            rx, ch, out = slots[k]
            for x, sg in grp:
                outputs[x] = {"receiver": rx, "channel": ch, "output": out, "level_sign": int(rel.get(x, 1))}
                if len(grp) > 1:
                    outputs[x].update(stacked_with=[y for y, _ in grp if y != x], plug_flipped=sg < 0)
            k += 1
        elif m in const_ir:
            rx, ch, out = slots[k]
            outputs[m] = {"receiver": rx, "channel": ch, "output": out, "level_sign": int(rel.get(m, 1))}
            k += 1
    for m in const:
        if m in outputs:
            continue
        lv = sched[m]["constant"]
        if lv is None or abs(lv) != PF_SPEED_STEPS:
            errors.append(f"motor '{m}' runs at level {lv} from t = 0 but has no IR receiver output: plugged straight "
                          f"into a battery box it can only run at full speed (|level| {PF_SPEED_STEPS})")
        if not boxes:
            errors.append(f"motor '{m}': no battery box to plug into")
            continue
        box_direct[m] = min(boxes, key=lambda b: float(np.linalg.norm(_device_point(P[m]) - _device_point(P[b]))))
    rx_box = {}
    for rx in receivers:
        if boxes:
            rx_box[rx] = min(boxes, key=lambda b: float(np.linalg.norm(_device_point(P[rx]) - _device_point(P[b]))))
    if outputs and not receivers:
        errors.append("motors on IR outputs but no IR receiver (58123c01) on board")
    # cables and plugs (2026-10-14): a lead runs from where it leaves its part (pf_cable) to the PF pad it plugs into
    # (a receiver output, or the battery box's pad), and every box pad needs room for its stack of plugs
    warnings = []
    plug_to: list[tuple[str, str, tuple]] = []  # (device with a lead, part it plugs into, local pad)
    for m, o in outputs.items():
        pads = P[o["receiver"]].bp.pf_pads
        plug_to.append((m, o["receiver"], pads[0 if o["output"] == "A" else -1] if pads else None))
    for m, box in box_direct.items():
        plug_to.append((m, box, (P[box].bp.pf_pads or (None,))[0]))
    for rx, box in rx_box.items():
        plug_to.append((rx, box, (P[box].bp.pf_pads or (None,))[0]))
    for dev, tgt, pad in plug_to:
        lead = RECEIVER_LEAD_MM if P[dev].bp.kind == "receiver" else MOTOR_LEAD_MM.get(P[dev].inst.part, 200.0)
        a = P[dev].to_world(P[dev].bp.pf_cable) if P[dev].bp.pf_cable is not None else _device_point(P[dev])
        b = P[tgt].to_world(pad) if pad is not None else _device_point(P[tgt])
        d = float(np.linalg.norm(a - b)) + LEAD_BEND_MM
        what = "IR receiver" if P[dev].bp.kind == "receiver" else "motor"
        if d > lead:
            errors.append(f"{what} '{dev}': its lead must run {d:.0f} mm (lead exit to the plug on '{tgt}', + "
                          f"{LEAD_BEND_MM:.0f} mm for the bends) but it is {lead:.0f} mm long (no extension cables owned)")
        elif d > 0.85 * lead:
            warnings.append(f"{what} '{dev}': its lead needs {d:.0f} of its {lead:.0f} mm to reach '{tgt}': little slack")
    stacks: dict[str, int] = {}
    for dev, tgt, pad in plug_to:
        if P[tgt].bp.kind == "battery":
            stacks[tgt] = stacks.get(tgt, 0) + 1
    for box, n in stacks.items():
        errors += _plug_room(P, box, n)
    return {"outputs": outputs, "box_direct": box_direct,
            "receivers": {rx: {"channel": i + 1, "box": rx_box.get(rx)} for i, rx in enumerate(receivers)},
            "idle": idle, "errors": errors, "warnings": warnings}


def check_real_electrics(design: Design, ir_receivers: int = IR_RECEIVERS) -> tuple[list[str], list[str]]:
    """Real-world electrics of an open-loop run (``Budget.real_open_loop``, the ``inventory`` part set): every
    motor the program sequences has an IR receiver output (``ir_receivers`` receivers x 2 outputs), box-direct motors
    run at full speed from the launch, motor and receiver leads reach their box / receiver. Battery boxes per motor:
    the general ``MOTORS_PER_BATTERY`` rule. Returns (errors, warnings)."""
    m = ir_mapping(design, ir_receivers)
    return [f"real electrics: {e}" for e in m["errors"]], [f"real electrics: {w}" for w in m["warnings"]]


def wheel_motors(design: Design) -> set[str]:
    """Motors whose gear train reaches a wheel (driving motors); every other motor runs an attachment."""
    P = place(design)
    sh = shafts(design)
    wheels = {n for n, pl in P.items() if pl.bp.kind == "wheel"} | {s for t in design.treads for s in t.sprockets}
    out = set()
    for c in motor_connections(design):  # round 4b: train motors too; a pullback is not a (PF) wheel motor
        if c.child not in P or (c.parent in P and r4b.is_pullback(P[c.parent].inst.part)):
            continue
        spin, _ = _propagate(design, P, sh, c.child, P[c.child].axis())
        if any(sh.get(w) in spin for w in wheels):
            out.add(c.parent)
    return out


def motor_connections(design: Design) -> list[DesignConnection]:
    """Motor connections, the primary motor's (``design.motor``) first. Round 4b: a PF train motor (integral driven
    axles) has a virtual one (``inventory_r4b.virtual_motor_connections``)."""
    mc = [c for c in design.connections if c.kind == "motor"] + r4b.virtual_motor_connections(design)
    return sorted(mc, key=lambda c: c.parent != design.motor)


def pullback_motors(design: Design) -> set[str]:
    """Round 4b: the design's pullback (spring) motors: motors, but not PF motors (no IR, not commanded)."""
    return {p.name for p in design.parts if r4b.is_pullback(p.part)}


WORM_PITCH_RADIUS = 4.0
"""Pitch radius of the single-start worm (4716, 10.2 mm across the threads): it meshes with a spur gear like an 8T,
at (z / 2 + 4) mm from the gear's axis (16 mm for a 24T, 24 mm for a 40T), axes at right angles; ratio 1 : z."""
BEVEL_RIGHT_ANGLE = {"32270", "32269", "32498"}
BEVEL_FAMILIES = ({"32270", "6589"}, {"32269", "32198"})
"""Inventory additions: a single bevel (12T 6589, 20T 32198) meshes at right angles with a single or double bevel of the
same tooth count (1 : 1). The apex rule is the double bevels' (each gear's origin z / 2 mm from the axes'
intersection) with 1 mm tolerance: a single bevel's teeth sit 0.8 mm off its hole's mid-plane (LDraw)."""
"""Double-bevel gears (12T, 20T, 36T; 45 deg teeth): two equal ones mesh at right angles, axes intersecting, each gear
centred at the other's pitch radius (z / 2 mm) from the intersection; ratio 1 : 1."""


SINGLE_BEVEL_FACE = {"6589": 1.0, "32198": -1.0}
"""Side (sign of the local y) a single bevel's teeth face (LDraw / results-UI mesh: the toothed rim is the 0.8 mm
offset of its shape, parts.py): the axes' intersection must lie on that side, or the two gears touch back to back."""


def single_bevel_facing_problems(tag: str, a: Placed, b: Placed) -> list[str]:
    """A single bevel (6589 / 32198) in a right-angle mesh must face the axes' intersection."""
    errs = []
    for x, y in ((a, b), (b, a)):
        face = SINGLE_BEVEL_FACE.get(x.inst.part)
        if face is None:
            continue
        t = float(np.dot(y.p - x.p, x.axis()))  # the intersection lies at x.p + t * axis
        if t * face <= 0:
            errs.append(f"{tag}: {x.inst.name}'s teeth face away from the other gear (a single bevel's toothed face, "
                        f"local {'+' if face > 0 else '-'}y, must face the axes' intersection: turn it 180 deg, "
                        f"yaw + 180, or move it to the other side of the other gear's axle)")
    return errs


def crossed_mesh_sense(pa, na, za: int, pb, nb, zb: int) -> float:
    """Real sense s of a right-angle mesh: w_b . n_b = s (z_a / z_b) w_a . n_a, with n = each gear's own axis and p
    its centre (any length unit). Axes intersecting at O (bevels, 14T / 22T, knob wheels): the pitch point moves
    alike on both gears, so s = -sign(t_a t_b), t = the gear centre's position along its own axis from O (the sense
    flips when one gear moves to the other side of the other's axle). Worm (4716, right-hand thread, LDraw mesh): the
    thread drives the gear's teeth at the contact along -n_w per +w_w, so s = -sign((n_g x d) . n_w), d = gear centre
    to the worm's axis in the gear's plane (the sense flips with the side of the gear the worm is on)."""
    pa, na, pb, nb = (np.asarray(v, dtype=float) for v in (pa, na, pb, nb))
    if za == 1 or zb == 1:
        (pw, nw), (pg, ng) = ((pa, na), (pb, nb)) if za == 1 else ((pb, nb), (pa, na))
        dv = pw - pg
        dv = dv - ng * float(np.dot(dv, ng))
        k = float(np.dot(np.cross(ng, dv), nw))
        return -1.0 if k > 0 else 1.0
    ta = float(np.dot(pb - pa, na))  # O - pa along n_a (pb - O is normal to n_a)
    tb = float(np.dot(pa - pb, nb))
    return -1.0 if ta * tb > 0 else 1.0


def right_angle_mesh(ba: BenchPart, bb: BenchPart) -> bool:
    """A worm with a spur gear, or two equal double-bevel gears (they may also mesh as spurs, axes parallel)."""
    return ba.teeth == 1 or bb.teeth == 1


def check_right_angle_mesh(tag: str, a: Placed, b: Placed) -> list[str]:
    """Geometry of a right-angle mesh (worm + spur gear, or two equal double bevels)."""
    errs = []
    da, db = a.axis(), b.axis()
    if abs(float(np.dot(da, db))) > 1e-3:
        return [f"{tag}: a worm / right-angle bevel mesh needs perpendicular axes"]
    if a.bp.teeth == 1 or b.bp.teeth == 1:
        worm, gear = (a, b) if a.bp.teeth == 1 else (b, a)
        if gear.bp.teeth == 1:
            return [f"{tag}: two worms do not mesh"]
        dw = worm.axis()
        dist, t = _line_point_distance(worm.p, dw, gear.p)  # gear centre from the worm's axis line
        nominal = GEAR_MODULE * gear.bp.teeth / 2.0 + WORM_PITCH_RADIUS
        if abs(dist - nominal) > 0.4:
            errs.append(f"{tag}: worm axis {dist:.2f} mm from the gear centre (nominal {nominal:.1f})")
        if abs(t) > 4.0:
            errs.append(f"{tag}: the gear is {t:.1f} mm along the worm from its middle (> 4 mm)")
        off = float(np.dot(worm.p - gear.p, gear.axis()))
        if abs(off) > 0.6:
            errs.append(f"{tag}: the worm is {off:.2f} mm off the gear's mid-plane")
        return errs
    family = next((f for f in BEVEL_FAMILIES if a.inst.part in f and b.inst.part in f), None)
    if family is None and (a.inst.part not in BEVEL_RIGHT_ANGLE or b.inst.part != a.inst.part):
        return [f"{tag}: gears at right angles must be a worm with a spur gear, two equal double bevels or a single "
                f"bevel with a bevel of the same tooth count"]
    tol = 1.0 if (a.bp.bevel_only or b.bp.bevel_only) else 0.4
    # the two axis lines intersect; each gear centre at the other's pitch radius from the intersection
    n = np.cross(da, db)
    gap = abs(float(np.dot(b.p - a.p, n / np.linalg.norm(n))))
    if gap > tol:
        errs.append(f"{tag}: bevel axes do not intersect ({gap:.2f} mm apart)")
    r = GEAR_MODULE * a.bp.teeth / 2.0
    ta = float(np.dot(b.p - a.p, da))  # the intersection lies at a.p + ta * da (b.p sits on the line through it)
    tb = float(np.dot(a.p - b.p, db))
    if abs(abs(ta) - r) > tol or abs(abs(tb) - r) > tol:
        errs.append(f"{tag}: bevel gear centres {abs(ta):.2f} / {abs(tb):.2f} mm from the intersection (nominal {r:.1f})")
    errs.extend(single_bevel_facing_problems(tag, a, b))
    return errs


def _propagate(design: Design, P, sh, seed: str, vec: np.ndarray) -> tuple[dict[str, np.ndarray], list[str]]:
    spin: dict[str, np.ndarray] = {sh.get(seed, seed): vec}
    problems: list[str] = []
    changed = True
    while changed:
        changed = False
        for mesh in design.gear_meshes:
            a, b = mesh.gear_a, mesh.gear_b
            if a not in sh or b not in sh:
                continue
            za, zb = P[a].bp.teeth, P[b].bp.teeth
            perpendicular = abs(float(np.dot(P[a].axis(), P[b].axis()))) < 1e-3
            for x, y, zx, zy in ((a, b, za, zb), (b, a, zb, za)):
                sx, sy = sh[x], sh[y]
                if sx in spin:
                    # right-angle mesh: the speed scales by z_x / z_y about the other gear's axis, in the sense the
                    # geometry gives (which side of the other's axle each gear sits on; crossed_mesh_sense)
                    w = (
                        crossed_mesh_sense(P[x].p, P[x].axis(), zx, P[y].p, P[y].axis(), zy)
                        * float(np.dot(spin[sx], P[x].axis())) * zx / zy * P[y].axis()
                        if perpendicular
                        else -spin[sx] * zx / zy
                    )
                    if sy not in spin:
                        spin[sy] = w
                        changed = True
                    elif np.linalg.norm(spin[sy] - w) > 1e-6:
                        problems.append(f"gear train locked: shaft {sy} is driven at two different speeds")
        for tr in design.treads:  # a tread turns its two sprockets together (same sense, 1:1 for equal sprockets)
            if len(tr.sprockets) != 2 or any(s not in sh or s not in P for s in tr.sprockets):
                continue
            for x, y in (tuple(tr.sprockets), tuple(tr.sprockets)[::-1]):
                sx, sy = sh[x], sh[y]
                if sx in spin:
                    w = spin[sx] * tread_ratio(design, x, y)
                    if sy not in spin:
                        spin[sy] = w
                        changed = True
                    elif np.linalg.norm(spin[sy] - w) > 1e-6:
                        problems.append(f"gear train locked: shaft {sy} is driven at two different speeds (tread {tr.name})")
        if design.chains or design.cv_joints or design.differentials:  # round 4b couplings
            ch2, pr2 = r4b.propagate(design, P, sh, spin)
            changed = changed or ch2
            problems += pr2
    return spin, problems


def motor_trains(design: Design, P: dict[str, Placed] | None = None):
    """Spin of every driven shaft with all motors on (rad/s, the primary motor's shaft at +1 rad/s about its axis),
    the relative direction of every motor (``{motor part name: +-1}``, the primary +1; each other motor turns the way
    that makes its wheels turn like the primary's) and problems (locked trains, motors fighting)."""
    P = P or place(design)
    sh = shafts(design)
    mcs = motor_connections(design)
    if not mcs:
        return {}, {}, []
    pull = pullback_motors(design)
    if pull:  # round 4b: the electric motors' speeds first; a pullback only adds torque on shafts they turn
        mcs = sorted(mcs, key=lambda c: c.parent in pull and c.parent != design.motor)
    w_ref = motor_params(P[mcs[0].parent].inst.part)["no_load_rad_s"] if mcs[0].parent in P else 1.0
    wheels = ({n for n, pl in P.items() if pl.bp.kind == "wheel"} | {s for t in design.treads for s in t.sprockets if s in P}) or {
        n for n, pl in P.items() if pl.bp.kind == "axle_brick"}
    total: dict[str, np.ndarray] = {}
    rel: dict[str, int] = {}
    problems: list[str] = []
    ref_sign = None
    for c in mcs:
        if c.child not in P or c.parent not in P:
            continue
        pull = r4b.is_pullback(P[c.parent].inst.part)  # round 4b: a spring motor adds torque, it freewheels when faster
        w = motor_params(P[c.parent].inst.part)["no_load_rad_s"] / w_ref
        spin, prob = _propagate(design, P, sh, c.child, P[c.child].axis() * w)
        problems += prob
        wspins = [float(spin[sh[n]][1]) for n in sorted(wheels) if sh.get(n) in spin]
        d = 1
        if wspins:
            if ref_sign is None:
                ref_sign = 1.0 if wspins[0] > 0 else -1.0
            else:
                d = 1 if wspins[0] * ref_sign > 0 else -1
        rel[c.parent] = d
        for k, v in spin.items():
            v = v * d
            if k in total and np.linalg.norm(total[k] - v) > 1e-6 and not pull:
                problems.append(f"motors fight: shaft {k} is driven at two different speeds by two motors")
            total.setdefault(k, v)
    return total, rel, sorted(set(problems))


def shaft_spins(design: Design, P: dict[str, Placed] | None = None) -> tuple[dict[str, np.ndarray], list[str]]:
    """Angular velocity vector of every shaft for the primary motor shaft turning at +1 rad/s (relative to the
    chassis); every other motor turns at its own no-load speed relative to the primary's (:func:`motor_trains`).

    The motor joint turns the shaft about its local x (joints.py: revolute axis "X" in the child frame). An external
    spur mesh reverses the spin and scales it by ``z_a / z_b``.
    """
    spin, _, problems = motor_trains(design, P)
    return spin, problems


WITHDRAWN_PARTS = {"87574c01": "the PF train motor is not part of the offered inventory"}
"""(sim v13, 2026-10-27, user) parts the validator rejects although the catalogue still describes them."""


def validate(design: Design, budget: Budget | None = None) -> ValidationReport:
    """Static checks (catalog, budget, connection geometry, gear centre distances, overlaps) and stats."""
    budget = budget or Budget.for_part_set(design.part_set)
    rep = ValidationReport(design.name)
    E, W = rep.errors, rep.warnings
    catalog = bench_parts()

    # -- parts, names, catalog, assets, budget --
    names = [p.name for p in design.parts]
    for n in {n for n in names if names.count(n) > 1}:
        E.append(f"duplicate part name '{n}'")
    for p in design.parts:
        if p.part not in catalog:
            E.append(f"part '{p.name}': '{p.part}' is not in the catalog ({', '.join(sorted(catalog))})")
        elif not catalog[p.part].has_usd:
            E.append(f"part '{p.name}': '{p.part}' has no USD asset (needs scripts/convert_lego_parts.py)")
        if p.part in WITHDRAWN_PARTS:  # (sim v13) offered no more
            E.append(f"part '{p.name}': {p.part} is not available ({WITHDRAWN_PARTS[p.part]})")
    for pid, n in bom_item_counts(design).items():
        have = budget.inventory.get(pid, 0)
        if n > have:
            E.append(f"budget: {n} x {pid} used, the part set has {have}")
    n_pieces = len(design.parts) + tread_pieces(design) + r4b.chain_pieces(design)
    if budget.max_parts is not None and n_pieces > budget.max_parts:
        E.append(f"budget: {n_pieces} parts, limit {budget.max_parts}")
    P = place(design)
    if E:
        rep.stats = {}
        return rep

    # -- roles --
    for role, want in (("chassis", None), ("motor", "motor"), ("battery", "battery")):
        name = getattr(design, role)
        if not name:
            if role != "chassis" and any(p.bp.kind == want for p in P.values()):
                W.append(f"'{role}' not set although the design contains one")
            if role == "chassis":
                E.append("'chassis' (reference part) must be set")
            continue
        if name not in P:
            E.append(f"{role} '{name}' is not a part of the design")
        elif want and P[name].bp.kind != want and not (role == "battery" and P[name].bp.kind == "hub"):
            E.append(f"{role} '{name}' is a {P[name].bp.kind}, not a {want}")
    if design.motor and not design.battery and not (design.motor in P and r4b.is_pullback(P[design.motor].inst.part)):
        E.append("a motor needs a battery box on board")

    # -- connections --
    for c in design.connections:
        _check_connection(c, P, E, W)
    pairs = [(c.child, c.parent) for c in design.connections]
    for pr in {pr for pr in pairs if pairs.count(pr) > 1}:
        E.append(f"connection {pr[0]}->{pr[1]} declared twice")
    comps = _components(names, pairs + slide_edges(design) + r4b.extra_edges(design))  # a rack slide joins its two sides
    # (round 4b: a chain, a CV joint and a differential join their parts too)
    if len(comps) > 1:
        main = max(comps, key=len)
        loose = sorted(set(names) - main)
        E.append(f"parts not attached to the assembly: {loose}")
    rigid = _components(names, rigid_edges(design))
    chassis_set = next((s for s in rigid if design.chassis in s), set())
    for role in ("motor", "battery"):
        name = getattr(design, role)
        if name and name in P and name not in chassis_set:
            E.append(f"{role} '{name}' is not stud/pin-connected to the chassis cluster of '{design.chassis}'")
    for c in design.connections:
        if c.kind == "hinge" and c.parent in P and c.parent not in chassis_set and not P[c.parent].bp.free_bore:
            W.append(f"hinge {c.child}->{c.parent}: the bearing brick is not part of the chassis cluster")
    sh = shafts(design)
    internal = r4b.internal_parts(design)  # round 4b: a differential's satellites sit on its inner tabs
    for name, pl in P.items():
        if pl.bp.free_bore:
            runs = [c for c in design.connections if c.parent == name and c.kind == "hinge"]
            if len(runs) != 1:
                E.append(f"{name}: a free-turning (round bore) gear needs exactly one axle hinged in it (has {len(runs)})")
            elif pl.bp.bevel_only:  # round 4a: the 22T bevel half idles on a bare axle (the sim turns the two together)
                ax = runs[0].child
                busy = [c.child for c in design.connections if c.parent == ax and c.kind == "cross"
                        and c.child in P and P[c.child].bp.kind != "bush"]
                busy += [c.parent for c in design.connections if c.child == ax and c.kind == "motor"]
                if busy:
                    E.append(f"{name}: the 22T bevel half (round bore) must idle on a bare axle, but {ax} also carries "
                             f"{', '.join(busy)} (bushes only; use the 14T 69762 to drive or take off a shaft)")
        elif pl.bp.kind in ("gear", "wheel", "bush", "axle_brick") and name not in internal:
            fits = [c for c in design.connections if c.child == name and c.kind == "cross"]
            if len(fits) != 1:
                E.append(f"{name}: a {pl.bp.kind} needs exactly one cross fit on an axle (has {len(fits)})")
        if pl.bp.kind == "axle":
            bearings = [c for c in design.connections if c.child == name and c.kind in ("hinge", "motor")
                        and not (c.parent in P and P[c.parent].bp.free_bore)]
            if not bearings:
                E.append(f"axle {name}: no hinge (Technic hole) or motor bearing")
            if len([c for c in bearings if c.kind == "motor"]) and len(bearings) > 1:
                E.append(f"axle {name}: the motor shaft cannot also turn in a brick hole")
        if pl.bp.kind == "pin" and pl.bp.inventory:
            held = {c.parent for c in design.connections if c.child == name and c.kind == "pin"}
            need = 1 if pl.bp.pin_holes else 2  # a pin with its own pin hole may hold one part (another pin in it)
            if len(held) < need:
                E.append(f"pin {name}: needs pin connections to at least {need} part(s), has {len(held)}")
            slots = sum(round(2 * h / 8.0) for _, h, _ in pin_sections(pl))
            if len(held) > slots:
                E.append(f"pin {name}: holds {len(held)} parts but has {slots} module(s) of pin / axle")
        elif pl.bp.kind == "pin":
            held = {c.parent for c in design.connections if c.child == name and c.kind == "pin"}
            if len(held) != 2:
                E.append(f"pin {name}: a pin joins two parts (needs pin connections to 2 parts, has {len(held)})")
    pinned: dict[frozenset, int] = {}
    for p in P.values():
        if p.bp.kind == "pin":
            held = frozenset(c.parent for c in design.connections if c.child == p.inst.name and c.kind == "pin")
            if len(held) == 2:
                pinned[held] = pinned.get(held, 0) + 1
    for pair, n in pinned.items():
        linked = any(c.kind == "stud" and {c.child, c.parent} == set(pair) for c in design.connections)
        # a beam pinned to an axle-locked part (60483 / 32064) that the same axle also passes through is locked:
        # the axle in one hole and the pin in the next one (the standard way to fix a liftarm to a turning axle)
        for a, b in (tuple(pair), tuple(pair)[::-1]):
            axles = [c.parent for c in design.connections if c.child == a and c.kind == "cross"]
            if any(ax in P and b in P and axle_through_holes(P[ax], P[b]) for ax in axles):
                linked = True
        if any(r4b.is_shock(P[q].inst.part) for q in pair if q in P):
            linked = True  # round 4b: a shock's end turns on its pin by design
        if n == 1 and not linked and not any(
            P[p].bp.pivot for p in P if P[p].bp.kind == "pin"
            and frozenset(c.parent for c in design.connections if c.child == p and c.kind == "pin") == pair
        ):
            W.append(f"{' and '.join(sorted(pair))} are joined by a single pin (free to turn about it in reality)")
    motor_parts = [c.parent for c in design.connections if c.kind == "motor"]
    for n in {n for n in motor_parts if motor_parts.count(n) > 1}:
        E.append(f"motor '{n}' has two output shafts")
    motor_parts += [c.parent for c in r4b.virtual_motor_connections(design)]  # round 4b: train motors
    for n in motor_parts:
        if n != design.motor and n in P and P[n].bp.kind == "motor" and n not in chassis_set:
            E.append(f"motor '{n}' is not stud/pin-connected to the chassis cluster of '{design.chassis}'")
    n_bat = sum(1 for pl in P.values() if pl.bp.kind == "battery")
    # PF motors run from PF battery boxes (3 per box); SPIKE / Powered Up motors run from the hub's ports (6, checked
    # with the hub wiring). Until 2026-10-01 every motor counted against the PF boxes, which wrongly required a PF box
    # for a 4th motor of a hub-powered design.
    pf_motors = {m for m in set(motor_parts) if m in P and device_system(P[m].bp) == "pf"}
    if len(pf_motors) > MOTORS_PER_BATTERY * max(n_bat, 1):
        E.append(f"{len(pf_motors)} PF motors need at least {math.ceil(len(pf_motors) / MOTORS_PER_BATTERY)} "
                 f"battery boxes (at most {MOTORS_PER_BATTERY} PF motors per box)")
    for n, pl in P.items():
        if pl.bp.kind == "motor" and n not in motor_parts:
            W.append(f"motor part '{n}' drives nothing (no shaft in its output)")
    for n in design.rope_wheels:
        if n not in P or P[n].bp.kind != "wheel":
            E.append(f"rope wheel '{n}' is not a wheel of the design")
    if design.treads:
        e2, w2 = check_treads(design, P)
        E.extend(e2)
        W.extend(w2)
    rack_stats = {}
    if design.racks or any(pl.bp.kind == "rack" for pl in P.values()):
        from .racks import check_racks  # noqa: PLC0415

        e2, w2, rack_stats = check_racks(design, P)
        E.extend(e2)
        W.extend(w2)
    E.extend(check_motor_program(design, real=budget.real_open_loop))
    if budget.real_open_loop:
        e2, w2 = check_real_electrics(design, budget.ir_receivers)
        E.extend(e2)
        W.extend(w2)
    if design.motor and not any(c.parent == design.motor for c in motor_connections(design)):
        E.append(f"motor '{design.motor}' has no motor connection (a shaft in its output)")
    r4b_stats = {}
    if any(r4b.is_shock(pl.inst.part) or pl.inst.part in (r4b.DIFF_CASE, r4b.DIFF_HOUSING, r4b.PULLBACK)
           for pl in P.values()) or (design.chains or design.cv_joints or design.differentials or design.pullback):
        e2, w2, r4b_stats = r4b.check(design, P, sh)
        E.extend(e2)
        W.extend(w2)

    # -- gear meshes --
    mesh_pairs = set()
    for m in design.gear_meshes:
        if m.gear_a not in P or m.gear_b not in P:
            E.append(f"gear mesh {m.gear_a}/{m.gear_b}: unknown part")
            continue
        a, b = P[m.gear_a], P[m.gear_b]
        tag = f"gear mesh {m.gear_a}/{m.gear_b}"
        if "differential" in (a.bp.kind, b.bp.kind):  # round 4b: the 28T ring of a 62821b differential
            case, other = (a, b) if a.bp.kind == "differential" else (b, a)
            if other.bp.kind != "gear":
                E.append(f"{tag}: the differential's ring meshes with a gear")
                continue
            mesh_pairs.add(frozenset((m.gear_a, m.gear_b)))
            E.extend(r4b.ring_mesh_problems(tag, case, other))
            continue
        if a.bp.kind != "gear" or b.bp.kind != "gear":
            E.append(f"{tag}: both parts must be gears")
            continue
        mesh_pairs.add(frozenset((m.gear_a, m.gear_b)))
        for nm, z_decl, pl in ((m.gear_a, m.teeth_a, a), (m.gear_b, m.teeth_b, b)):
            if z_decl and z_decl != pl.bp.teeth:
                E.append(f"{tag}: {nm} declared with {z_decl} teeth, the part has {pl.bp.teeth}")
        hubbed = [nm for nm in (m.gear_a, m.gear_b) if any(c.child == nm and c.kind == "cross" and c.parent in P
                                                            and P[c.parent].bp.male_axles for c in design.connections)]
        if hubbed:  # round 4a: a gear on a 57585 arm turns about the hub's shaft, not about its own axis
            E.append(f"{tag}: {hubbed[0]} sits on an axle end of a 3-axle hub (57585): gears on the hub's arms do not "
                     "mesh (not modelled)")
            continue
        r4 = mesh_problems(tag, a, b)  # round 4a: knob wheels, bevel halves 69761 / 69762 (lego_bench.inventory_r4)
        if r4 is not None:
            E.extend(r4)
            if m.gear_a in sh and m.gear_b in sh and sh[m.gear_a] == sh[m.gear_b]:
                E.append(f"{tag}: both gears are on the same shaft")
            continue
        if (a.bp.bevel_only or b.bp.bevel_only) and abs(float(np.dot(a.axis(), b.axis()))) >= 1e-3:
            E.append(f"{tag}: a single bevel gear (6589 / 32198) meshes only at right angles")
            continue
        if right_angle_mesh(a.bp, b.bp) or abs(float(np.dot(a.axis(), b.axis()))) < 1e-3:
            E.extend(check_right_angle_mesh(tag, a, b))
            if m.gear_a in sh and m.gear_b in sh and sh[m.gear_a] == sh[m.gear_b]:
                E.append(f"{tag}: both gears are on the same shaft")
            continue
        for nm, pl in ((m.gear_a, a), (m.gear_b, b)):
            if abs(pl.inst.yaw_deg % 360.0) > 1e-6:
                E.append(f"{tag}: {nm} must have yaw 0 (gear axes along the design y axis)")
        ax = a.axis()
        if abs(float(np.dot(ax, b.axis()))) < 0.999:
            E.append(f"{tag}: axes not parallel")
            continue
        d = b.p - a.p
        axial = float(np.dot(d, ax))
        centre = float(np.linalg.norm(d - axial * ax))
        nominal = GEAR_MODULE * (a.bp.teeth + b.bp.teeth) / 2.0
        if abs(axial) > 1.0:
            E.append(f"{tag}: gears are {abs(axial):.2f} mm apart along the axis (not in one plane)")
        mixed = (a.bp.part_id in BEVEL_RIGHT_ANGLE) != (b.bp.part_id in BEVEL_RIGHT_ANGLE)
        if centre < nominal - 0.15:
            E.append(f"{tag}: centre distance {centre:.2f} mm < nominal {nominal:.1f} mm (teeth jam)")
        elif mixed and centre > nominal + 0.15:  # (user 2026-10-28) a double bevel on a spur gear: only its rounded
            # tooth tips reach the spur's teeth, so it holds only within 0.15 mm of the nominal distance
            E.append(f"{tag}: double bevel on a spur gear at {centre:.2f} mm > nominal {nominal:.1f} mm + 0.15 "
                     f"(the bevelled teeth barely touch: they skip)")
        elif centre > nominal + 0.6:
            E.append(f"{tag}: centre distance {centre:.2f} mm > nominal {nominal:.1f} mm + 0.6 (teeth skip)")
        elif centre > nominal + 0.3:
            W.append(
                f"{tag}: centre distance {centre:.2f} mm (nominal {nominal:.1f}): {centre - nominal:.2f} mm backlash"
            )
        if m.gear_a in sh and m.gear_b in sh and sh[m.gear_a] == sh[m.gear_b]:
            E.append(f"{tag}: both gears are on the same shaft")
    _, spin_problems = shaft_spins(design, P)
    E.extend(spin_problems)

    # -- overlaps --
    connected = {frozenset((c.child, c.parent)) for c in design.connections}
    connected |= set(pinned)  # parts pinned together sit face to face
    connected |= {frozenset((r, s.pinion)) for s in design.racks for r in s.racks}  # the pinion's teeth in the rack's
    connected |= r4b.overlap_exempt(design)  # round 4b: a CV ball in its cup, a differential's insides
    for a, b in combinations(P, 2):
        key = frozenset((a, b))
        if key in connected or key in mesh_pairs:
            continue
        pa, pb = P[a], P[b]
        if pa.bp.kind == "axle" and axle_through_holes(pa, pb) or pb.bp.kind == "axle" and axle_through_holes(pb, pa):
            continue
        depth = penetration(pa, pb)
        if depth > OVERLAP_TOL:
            E.append(f"overlap: {a} and {b} interpenetrate by {depth:.2f} mm")
    E.extend(stud_interference(P, design))  # studs are not in the collision shapes: checked separately

    rep.stats = compute_stats(design, P)
    rep.stats.update(r4b_stats)  # round 4b: shocks, chains, CV joints, differentials
    if rack_stats:  # rack slides: travel, speed at the motors' no-load speed, stall force, self-locking
        from .racks import drive_stats  # noqa: PLC0415

        spins, _ = shaft_spins(design, P)
        mp = design_motor_params(design) if design.motor else motor_params(None)
        for k, v in drive_stats(design, P, spins, mp["no_load_rad_s"], mp["stall_torque_nm"]).items():
            rack_stats.setdefault(k, {}).update(v)
        rep.stats["racks"] = rack_stats
    if design.treads and rep.stats.get("driven_wheels"):
        from .treads import TREAD_SKIP_FORCE_N  # noqa: PLC0415

        n_tr = sum(1 for t in design.treads if any(s in rep.stats["driven_wheels"] for s in t.sprockets))
        f = rep.stats.get("stall_tractive_force_n", 0.0)
        if n_tr and f > TREAD_SKIP_FORCE_N * n_tr:
            W.append(f"treads: the drive can pull {f:.0f} N at the treads (stall), above the estimated ~"
                     f"{TREAD_SKIP_FORCE_N:.0f} N per tread at which a real tread jumps its sprocket (not simulated: a "
                     f"jammed simulated tread holds)")
    if budget.max_mass_g is not None and rep.stats["mass_g"] > budget.max_mass_g:
        E.append(f"budget: mass {rep.stats['mass_g']:.1f} g > {budget.max_mass_g} g")
    if rep.stats.get("drive_problem"):
        E.append(rep.stats["drive_problem"])
    if budget.require_powered:
        if not design.motor or design.motor not in P or P[design.motor].bp.kind != "motor":
            E.append("powered: the design needs a real LEGO motor part (e.g. PF M-motor 58120c01) set as 'motor'")
        if not design.battery:
            E.append("powered: the design needs a battery box on board")
        driven_racks = any(v.get("speed_mm_s") for v in rep.stats.get("racks", {}).values())  # a motor-driven slide
        if os.environ.get("LEGO_BENCH_POWERED", "wheels") == "any":
            # 2026-10-15 (user, benchmark v4): no wheels needed; a motor must drive something (any output shaft)
            if not any(c.kind == "motor" for c in design.connections):
                E.append("powered: no motor drives anything (connect an axle to a motor's output)")
        elif not rep.stats.get("driven_wheels") and not rep.stats.get("driven_legs") and not driven_racks:
            E.append(
                "powered: no wheel is driven by the motor (connect the motor shaft to a wheel through axles/gears)"
            )
    return rep


# ----------------------------------------------------------------------------------------------------------------
# stats
# ----------------------------------------------------------------------------------------------------------------


def motor_params(part_id: str | None = None) -> dict[str, float]:
    """Motor line (no-load speed [rad/s], stall torque [N m]) and USD drive of a motor part: the PF M-motor line from
    car_design by default, the part's own measured line for the PF L / XL motors (``catalog.MOTOR_LINES``)."""
    from ai_pd_tasks.lego import car_design as cd

    bp = bench_parts().get(part_id) if part_id else None
    if bp is None or not bp.motor_no_load_rpm:
        out = {
            "no_load_rad_s": cd.MOTOR_NO_LOAD_RPM * 2 * math.pi / 60.0,
            "stall_torque_nm": cd.MOTOR_STALL_TORQUE,
            "drive": cd.motor_drive_params(),
        }
    else:
        w0_deg = bp.motor_no_load_rpm * 360.0 / 60.0
        out = {
            "no_load_rad_s": bp.motor_no_load_rpm * 2 * math.pi / 60.0,
            "stall_torque_nm": bp.motor_stall_torque_nm,
            # same USD angular drive as car_design.motor_drive_params: tau = tau_stall (1 - w / w0)
            "drive": {
                "target_velocity_deg_s": w0_deg,
                "damping_nm_s_per_deg": bp.motor_stall_torque_nm / w0_deg,
                "max_force_nm": bp.motor_stall_torque_nm,
            },
        }
    # calibration from the real E0 trials (2026-10-21): LEGO_BENCH_M_STALL sets the PF M-motor's stall torque [N m]
    # (BEC 0.15; Philo measured ~0.07-0.08 at 9 V), LEGO_BENCH_TORQUE_SCALE scales every motor's stall torque
    # (battery sag under load). The drive keeps tau = tau_stall (1 - w / w0).
    m_st = os.environ.get("LEGO_BENCH_M_STALL")
    ts = float(os.environ.get("LEGO_BENCH_TORQUE_SCALE", "1") or 1.0)
    if (m_st and (bp is None or not bp.motor_no_load_rpm)) or ts != 1.0:
        tau = (float(m_st) if (m_st and (bp is None or not bp.motor_no_load_rpm)) else out["stall_torque_nm"]) * ts
        out = dict(out, stall_torque_nm=tau, drive=dict(out["drive"]))
        out["drive"]["max_force_nm"] = tau
        out["drive"]["damping_nm_s_per_deg"] = tau / out["drive"]["target_velocity_deg_s"]
    # open-loop robustness audit (2026-10-06): LEGO_BENCH_MOTOR_SPEED_SCALE s scales every motor's no-load speed
    # (battery voltage / motor spread); the stall torque is kept, so the motor line is tau_stall (1 - w / (s w0))
    s = float(os.environ.get("LEGO_BENCH_MOTOR_SPEED_SCALE", "1") or 1.0)
    if s != 1.0:
        out = dict(out, no_load_rad_s=out["no_load_rad_s"] * s, drive=dict(out["drive"]))
        out["drive"]["target_velocity_deg_s"] *= s
        out["drive"]["damping_nm_s_per_deg"] /= s
    # calibration: LEGO_BENCH_MOTOR_TORQUE_SCALE scales every motor's stall torque (the no-load speed is kept)
    k = float(os.environ.get("LEGO_BENCH_MOTOR_TORQUE_SCALE", "1") or 1.0)
    if k != 1.0:
        out = dict(out, stall_torque_nm=out["stall_torque_nm"] * k, drive=dict(out["drive"]))
        out["drive"]["damping_nm_s_per_deg"] *= k
        out["drive"]["max_force_nm"] *= k
    return out


def design_motor_params(design: Design) -> dict[str, float]:
    """:func:`motor_params` of the design's motor part."""
    part = next((p.part for p in design.parts if p.name == design.motor), None)
    return motor_params(part)


def _step_height(r: float, tan_phi: float) -> float:
    phi = math.atan(max(tan_phi, 0.0))
    return r * (1.0 - math.cos(min(phi, math.pi / 2)))


def compute_stats(design: Design, P: dict[str, Placed] | None = None, surface: str = "laminate") -> dict:
    """Mass properties, footprint, drivetrain and quasi-static capability estimates (mm, g, N, m/s)."""
    P = P or place(design)
    st: dict = {}
    mass = sum(pl.bp.mass_g for pl in P.values())
    com = sum((pl.to_world(pl.bp.com_mm) * pl.bp.mass_g for pl in P.values()), np.zeros(3)) / max(mass, 1e-9)
    lo = np.min([pl.aabb()[0] for pl in P.values()], axis=0)
    hi = np.max([pl.aabb()[1] for pl in P.values()], axis=0)
    treads = tread_wheel_radii(design, P) if design.treads else {}  # sprocket -> (ground r, drive r, width)
    if design.treads:  # tread links: mass at the loop centres, envelope in the bounding box
        from .treads import tread_geoms  # noqa: PLC0415

        tg = tread_geoms(design, P)
        m_t = tread_mass_g(design)
        if tg and m_t > 0:
            c_t = np.mean([0.5 * (g.ca + g.cb) for g in tg], axis=0)
            com = (com * mass + c_t * m_t) / (mass + m_t)
            lo = np.min([lo] + [g.bbox()[0] for g in tg], axis=0)
            hi = np.max([hi] + [g.bbox()[1] for g in tg], axis=0)
        mass += m_t
    if design.chains:  # round 4b: the 3711 links' mass, half on each gear
        m_c = 0.0
        for ch in design.chains:
            if all(g in P for g in ch.gears):
                m_l = int(ch.links) * r4b.CHAIN_LINK_MASS_G
                com = (com * (mass + m_c) + 0.5 * (P[ch.gears[0]].p + P[ch.gears[1]].p) * m_l) / (mass + m_c + m_l)
                m_c += m_l
        mass += m_c
    st.update(
        mass_g=round(mass, 2),
        com_mm=[round(float(v), 2) for v in com],
        n_parts=len(P) + tread_pieces(design) + r4b.chain_pieces(design),
        bom=design.bom(),
        bbox_min_mm=[round(float(v), 2) for v in lo],
        bbox_max_mm=[round(float(v), 2) for v in hi],
        length_mm=round(float(hi[0] - lo[0]), 2),
        width_mm=round(float(hi[1] - lo[1]), 2),
    )

    # -- wheels on the ground --
    wheels = [pl for pl in P.values() if (pl.bp.kind == "wheel" or pl.inst.name in treads) and abs(pl.axis()[2]) < 0.01]
    rope_set = set(design.rope_wheels)
    if rope_set and any(pl.inst.name not in rope_set for pl in wheels):  # rope wheels are not ground wheels
        wheels = [pl for pl in wheels if pl.inst.name not in rope_set]
    def r_ground(pl):  # a tread's sprocket stands on the tread's grouser tips
        return treads[pl.inst.name][0] if pl.inst.name in treads else pl.bp.wheel_radius

    ground_z = min((pl.p[2] - r_ground(pl) for pl in wheels), default=float(lo[2]))
    ground = [pl for pl in wheels if pl.p[2] - r_ground(pl) < ground_z + 0.5]
    st["ground_z_mm"] = round(float(ground_z), 3)
    st["height_mm"] = round(float(hi[2] - ground_z), 2)
    st["com_height_mm"] = round(float(com[2] - ground_z), 2)
    st["wheels"] = [pl.inst.name for pl in ground]
    axles: list[list[Placed]] = []
    for pl in sorted(ground, key=lambda q: q.p[0]):
        if axles and abs(axles[-1][0].p[0] - pl.p[0]) < 1.0:
            axles[-1].append(pl)
        else:
            axles.append([pl])
    ax_x = [float(np.mean([w.p[0] for w in a])) for a in axles]
    ys = [float(w.to_world(w.bp.shape.centre)[1]) for w in ground]  # tyre centre lines
    st["axle_x_mm"] = [round(x, 2) for x in ax_x]
    st["wheelbase_mm"] = round(ax_x[-1] - ax_x[0], 2) if len(ax_x) > 1 else 0.0
    st["track_mm"] = round(max(ys) - min(ys), 2) if ys else 0.0
    st["track_center_y_mm"] = round(0.5 * (max(ys) + min(ys)), 2) if ys else 0.0
    st["wheel_radius_mm"] = round(max((r_ground(w) for w in ground), default=0.0), 2)
    st["wheel_width_mm"] = round(max((treads[w.inst.name][2] if w.inst.name in treads else w.bp.width for w in ground),
                                     default=0.0), 2)
    if treads:
        st["treads"] = {t.name: list(t.sprockets) for t in design.treads}
    st["tyres"] = sorted({w.inst.part for w in ground if w.bp.tyre_mu})
    non_wheel_low = min((pl.aabb()[0][2] for pl in P.values() if pl.bp.kind != "wheel"), default=ground_z)
    st["ground_clearance_mm"] = round(float(non_wheel_low - ground_z), 2)
    st["front_x_mm"], st["rear_x_mm"] = round(float(hi[0]), 2), round(float(lo[0]), 2)
    inline = bool(ys) and max(ys) - min(ys) < 1.0
    st["inline_wheels"] = inline

    # -- drivetrain --
    mp = design_motor_params(design)
    st["motor_part"] = next((p.part for p in design.parts if p.name == design.motor), None)
    st["motor_no_load_rpm"] = round(mp["no_load_rad_s"] * 60.0 / (2 * math.pi), 1)
    st["motor_stall_torque_nm"] = mp["stall_torque_nm"]
    spin, rel_dirs, _ = motor_trains(design, P)
    st["n_motors"] = len(rel_dirs)
    sh = shafts(design)
    driven, speeds = [], []
    for w in ground:
        s = sh.get(w.inst.name)
        if s in spin:
            driven.append(w.inst.name)
            speeds.append(float(np.dot(spin[s], np.array([0.0, 1.0, 0.0]))))
    st["driven_wheels"] = driven
    # wheel-legs (whegs, crank legs): axle bricks (32064) on a motor-driven shaft that reach below the axle
    st["driven_legs"] = [
        n for n, pl in P.items() if pl.bp.kind == "axle_brick" and sh.get(n) in spin
    ]
    if not driven and st["driven_legs"]:  # a legged vehicle: forward = the legs' spin sense of a wheel rolling forward
        s_leg = float(np.dot(spin[sh[st["driven_legs"][0]]], np.array([0.0, 1.0, 0.0])))
        st["motor_direction"] = 1 if s_leg > 0 else -1
        st["motor_directions"] = {m: st["motor_direction"] * d for m, d in rel_dirs.items()}
    st["drive_problem"] = ""
    if driven:
        if max(speeds) * min(speeds) < 0 or max(abs(v) for v in speeds) - min(abs(v) for v in speeds) > 1e-6:
            st["drive_problem"] = (
                f"driven wheels turn at different speeds/directions {dict(zip(driven, speeds, strict=True))}"
            )
        s = speeds[0]
        ratio = 1.0 / abs(s)
        r_m = st["wheel_radius_mm"] * 1e-3
        if treads and all(n in treads for n in driven):  # a tread moves at its pitch line's speed
            r_m = max(treads[n][1] for n in driven) * 1e-3
        st["gear_ratio"] = round(ratio, 4)
        st["motor_direction"] = 1 if s > 0 else -1
        st["motor_directions"] = {m: st["motor_direction"] * d for m, d in rel_dirs.items()}
        st["wheel_no_load_rad_s"] = round(mp["no_load_rad_s"] / ratio, 3)
        st["expected_speed_m_s"] = round(mp["no_load_rad_s"] / ratio * r_m, 4)
        st["wheel_stall_torque_nm"] = round(mp["stall_torque_nm"] * ratio, 4)
        st["stall_tractive_force_n"] = round(mp["stall_torque_nm"] * ratio / r_m, 3)
        st["expected_speed_note"] = "no-load motor speed through the gear ratio (ideal gears, no slip)"
    else:
        st.update(gear_ratio=None, motor_direction=0, expected_speed_m_s=0.0, stall_tractive_force_n=0.0)

    # -- quasi-static capability estimates (two-axle model) --
    mu_s, mu_d = contact_friction(surface)
    st["surface"] = surface
    st["mu_static_effective"] = round(mu_s, 3)
    weight = mass * 1e-3 * G
    st["weight_n"] = round(weight, 4)
    cap: dict = {}
    if len(axles) >= 2 and driven:
        xr, xf = ax_x[0], ax_x[-1]
        L = xf - xr
        h = float(com[2] - ground_z)
        d_f, d_r = xf - com[0], com[0] - xr
        n_f, n_r = weight * d_r / L, weight * d_f / L
        rear_driven = any(w.inst.name in driven for w in axles[0])
        front_driven = any(w.inst.name in driven for w in axles[-1])
        drive = "4WD" if rear_driven and front_driven else "RWD" if rear_driven else "FWD" if front_driven else "mid"
        st["drive"] = drive
        st["load_front_frac"] = round(n_f / weight, 3)
        f_stall = st["stall_tractive_force_n"]

        def axle_mu(ax, surf):  # static friction of an axle's wheels on a surface (hard ABS or rubber tyres)
            return min(contact_friction(surf, w.inst.part)[0] for w in ax)

        mu_r, mu_f = axle_mu(axles[0], surface), axle_mu(axles[-1], surface)
        if st["tyres"]:
            st["mu_static_rear_front"] = [round(mu_r, 3), round(mu_f, 3)]
        f_trac = min(mu_r * n_r * rear_driven + mu_f * n_f * front_driven, f_stall)
        cap["flat_traction_n"] = round(float(f_trac), 4)

        def max_slope(surf):
            mr, mf = axle_mu(axles[0], surf), axle_mu(axles[-1], surf)
            if drive == "4WD":
                t = mr if mr == mf else (mr * d_f + mf * d_r) / (L - h * (mr - mf))
            elif drive == "RWD":
                mu = mr
                t = mu * d_f / (L - mu * h) if L - mu * h > 0 else 10.0
            else:
                mu = mf
                t = mu * d_r / (L + mu * h)
            t = min(t, d_r / h if h > 0 else 10.0)  # tipping backwards
            ang = math.degrees(math.atan(max(t, 0.0)))
            if f_stall < weight:
                ang = min(ang, math.degrees(math.asin(f_stall / weight)))
            return round(ang, 2)

        cap["max_slope_deg"] = {s: max_slope(s) for s in ("laminate", "wood", "rubber_mat", "waxed")}
        r = st["wheel_radius_mm"]
        # a wheel line at a step edge climbs if tan(phi) <= (push of the other driven axles + own edge friction) / load
        tf = (mu_r * n_r * rear_driven + mu_f * n_f * front_driven) / n_f
        tr = (mu_f * n_f * front_driven + mu_r * n_r * rear_driven) / n_r
        h_step = min(_step_height(r, tf), _step_height(r, tr))
        cap["max_step_mm"] = round(h_step, 2)
        cap["max_gap_roll_mm"] = round(2 * math.sqrt(max(r * r - (r - h_step) ** 2, 0.0)), 2)
        # spanning a gap: while an axle is over it the car must rest on the other axle and on a skid (a non-wheel part
        # within SKID_HEIGHT_MM of the ground) beyond the gap, and the axle still on the ground must be driven
        skids = [
            pl
            for pl in P.values()
            if pl.bp.kind not in ("wheel", "gear", "axle", "bush") and pl.aabb()[0][2] - ground_z < SKID_HEIGHT_MM
        ]
        if skids and front_driven and rear_driven:
            front_reach = max(pl.aabb()[1][0] for pl in skids) - xf
            rear_reach = xr - min(pl.aabb()[0][0] for pl in skids)
            cap["max_gap_span_mm"] = round(float(max(min(L, front_reach, rear_reach), 0.0)), 1)
        else:
            cap["max_gap_span_mm"] = 0.0
        sled_mu = 0.5 * (SURFACES["laminate"][1] + ABS_FRICTION[1])  # ABS sled sliding on laminate
        cap["tug_max_load_g"] = round(float(f_trac / (sled_mu * G) * 1e3), 1)
        # both wheel lines wholly on the plank (track + one wheel width; 4 mm for the 9686 wedge-belt wheels)
        cap["min_plank_width_mm"] = round(st["track_mm"] + st["wheel_width_mm"], 1)
        cap["tip_lateral_deg"] = round(math.degrees(math.atan2(st["track_mm"] / 2, h)), 1) if h > 0 else 90.0
        cap["note"] = (
            "quasi-static rigid-body estimates (no momentum, no compliance), effective friction "
            "= average of the wheel material (ABS or tyre rubber) and the surface; simulation decides"
        )
    elif inline and driven:
        st["drive"] = "inline (rope)"
    else:
        st["drive"] = "none"
    # rope wheels declared by the design (a car that drives onto a rope rail on its rope wheels)
    rope_ws = [P[n] for n in design.rope_wheels if n in P]
    if rope_ws:
        y_line = float(np.mean([w.to_world(w.bp.shape.centre)[1] for w in rope_ws]))
        bottom = min(w.p[2] - (w.bp.groove_radius or w.bp.wheel_radius) for w in rope_ws)
        depth = float(bottom - com[2])
        st["rope_line_y_mm"] = round(y_line, 2)
        cap["rope_wheel_line_y_mm"] = round(y_line, 2)
        cap["rope_wheel_bottom_above_ground_mm"] = round(float(bottom - ground_z), 2)
        cap["rope_hang_depth_mm"] = round(depth, 2)
        cap["rope_roll_deg"] = round(math.degrees(math.atan2(float(com[1]) - y_line, depth)), 1) if depth > 0 else None
        cap["rope_clearance_mm"] = round(rope_clearance(design, P, y_line, bottom - 0.45, 2.0), 2)
        rx = sorted({round(float(w.p[0]), 1) for w in rope_ws})
        drv = [w for w in rope_ws if sh.get(w.inst.name) in spin]
        if drv and len(rx) >= 2:
            f = (com[0] - rx[0]) / (rx[-1] - rx[0])
            share = (1 - f) * any(abs(w.p[0] - rx[0]) < 0.5 for w in drv) + f * any(
                abs(w.p[0] - rx[-1]) < 0.5 for w in drv
            )
            mu_r = min(contact_friction("cord", w.inst.part)[0] for w in rope_ws)
            cap["rope_max_incline_deg"] = round(math.degrees(math.atan(mu_r * float(share))), 1)
    # rope: wheels in one line, centre of mass below the rope contact -> hangs stably
    elif inline and ground:
        y_line = float(np.mean(ys))
        depth = float(ground_z - com[2])
        cap["rope_wheel_line_y_mm"] = round(y_line, 2)
        cap["rope_hang_depth_mm"] = round(depth, 2)
        cap["rope_roll_deg"] = round(math.degrees(math.atan2(float(com[1]) - y_line, depth)), 1) if depth > 0 else None
        cap["rope_clearance_mm"] = round(rope_clearance(design, P, y_line, ground_z, 2.0), 2)
        if driven and len(axles) >= 2:
            xr, xf = ax_x[0], ax_x[-1]
            n_f_frac = (com[0] - xr) / (xf - xr)
            share = (1 - n_f_frac) * any(w.inst.name in driven for w in axles[0]) + n_f_frac * any(
                w.inst.name in driven for w in axles[-1]
            )
            mu_r = min(contact_friction("cord", w.inst.part)[0] for w in ground)
            cap["rope_max_incline_deg"] = round(math.degrees(math.atan(mu_r * float(share))), 1)
    st["capability"] = cap
    return st


def rope_clearance(design: Design, P: dict[str, Placed], y_line: float, ground_z: float, r_rope: float) -> float:
    """Free space [mm] between a rope of radius r_rope under the wheel line and all non-wheel parts."""
    groove_depth = 0.45  # the rope settles this far above the wheel bottoms (groove)
    z_axis = ground_z + groove_depth - r_rope
    xs = np.arange(min(pl.aabb()[0][0] for pl in P.values()) - 5, max(pl.aabb()[1][0] for pl in P.values()) + 5, 0.5)
    best = float("inf")
    for pl in P.values():
        if pl.bp.kind == "wheel":
            continue
        # sample the rope surface ring around the axis and measure penetration into the part
        ang = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        for rr in (r_rope, 0.0):
            pts = np.stack(
                [
                    np.repeat(xs, len(ang)),
                    np.tile(y_line + rr * np.cos(ang), len(xs)),
                    np.tile(z_axis + rr * np.sin(ang), len(xs)),
                ],
                axis=1,
            )
            d = _depth_in_shape(pl.bp.shape, (pl.R.T @ (pts - pl.p).T).T)
            best = min(best, float(-d.max()) - (r_rope if rr == 0.0 else 0.0))
    return best


# ----------------------------------------------------------------------------------------------------------------
# solver conditioning (for spawning)
# ----------------------------------------------------------------------------------------------------------------

SHAFT_MASS_RATIO_MAX = 4.0
"""Largest tolerated ratio between the mass cross-fitted on an axle and the axle's own mass. A 1.2 g axle carrying two
33 g, 62 mm wheels (57:1) is a light body in a chain of fixed joints and contacts that carries the car's weight and
the drive torque: at 1 ms / 32 iterations the chain goes soft, the axle turns inside its wheels and the gear train
locks. The 9686 car's shafts are at <= 2.5:1."""
SHAFT_MASS_RATIO_TARGET = 3.0
PIN_MASS_RATIO = 10.0
MOTOR_SHAFT_MASS_RATIO = 8.0
MOTOR_PINION_RATIO = 4.0
KID_MIN_RATIO = 4.0
CRANK_RIDERS = os.environ.get("LEGO_BENCH_CRANK_RIDERS", "0") != "0"
"""Crank riders (opt-in, ``LEGO_BENCH_CRANK_RIDERS=1``; gear-torque study 2026-10-06): a rigid cluster pinned to an inventory crank (a 32140 / 32348 / 32271 / 6629 / 32056
liftarm or a 6536 / 32184 / 42003 connector cross-fitted on the axle) rides the shaft like one on a 60483 / 7229 /
32064 and counts towards the shaft's conditioning. In the default model such a 13-50 g lever or leg hangs on a bare
1.2-1.7 g axle (spin inertia ~5e-9 kg m2) that a PhysX gear joint (worm, bevel) drives: the worm spins at no-load speed
while the stalled lever presses with ~2 % of the worm's stall torque, and gravity swings a worm-held leg through the
"locked" worm. With the rule a crank lever reaches 58 % of the lossless stall torque at 32 iterations and 96 % with
``LEGO_BENCH_CRANK_ITERS=128`` (``env.CRANK_POS_ITERS``). Not the default: at 32 iterations the worm-leg designs
i5d_wt3 / i5d_n9r shake apart at the start, and at 128 the spur pinions of several claimed cars slip on their shafts."""


def crank_rider_axles(design: Design) -> set[str]:
    """Axles that carry a rigid cluster through an inventory crank (the clusters :data:`CRANK_RIDERS` adds to the
    shaft conditioning); empty when the rule is off or the design has none (every pre-inventory design)."""
    if not CRANK_RIDERS:
        return set()
    catalog = bench_parts()
    parts = {p.name: p for p in design.parts}
    rigid = _components([q.name for q in design.parts], [(c.child, c.parent) for c in design.connections if c.kind in RIGID_KINDS])
    cluster_of = {n: comp for comp in rigid for n in comp}
    out = set()
    for c in design.connections:
        if c.kind != "cross" or c.parent not in parts or c.child not in parts:
            continue
        pb, kb = catalog.get(parts[c.parent].part), catalog.get(parts[c.child].part)
        if pb is None or kb is None or pb.kind != "axle" or kb.kind == "axle_brick" or not _locks_cluster(kb):
            continue
        comp = cluster_of.get(c.child, {c.child})
        if design.chassis not in comp and len(comp) > 1:
            out.add(c.parent)
    return out


def _locks_cluster(bp: BenchPart) -> bool:
    """A cross-fitted part that can carry a pinned / studded cluster round with its axle (an axle-hole lock)."""
    if bp.kind == "axle_brick":
        return True
    return CRANK_RIDERS and bool(bp.inventory) and bp.kind in ("technic_brick", "connector")


def _spin_inertia(bp: BenchPart, mass_g: float | None = None) -> float:
    """Approximate inertia [kg m2] about the part's rotation axis (solid cylinder)."""
    m = (bp.mass_g if mass_g is None else mass_g) * 1e-3
    if bp.kind in ("axle", "pin"):
        r = 2.4e-3
    elif bp.kind == "wheel":
        r = bp.wheel_radius * 1e-3
    elif isinstance(bp.shape, Box):
        return 0.0
    else:
        r = bp.shape.radius * 1e-3
    return 0.5 * m * r * r


def shaft_mass_overrides(design: Design, skip_axles=(), rider_log: dict | None = None) -> dict[str, dict]:
    """Solver conditioning for shafts whose cross-fitted parts are much heavier than the axle (ratio >
    :data:`SHAFT_MASS_RATIO_MAX`): mass is moved from the parts on the axle to the axle until the ratio is
    :data:`SHAFT_MASS_RATIO_TARGET`. The shaft's total mass is unchanged, its centre of mass moves by < 1 mm along the
    axle (the parts keep their relative mass), and the spin inertia taken from the parts is given to the axle
    (isotropic, at least the axle's transverse inertia). Returns {part name: {"mass_kg": .., "inertia_kgm2": ..}}
    (inertia only for the axle). Designs of the 9686 part set are not affected."""
    parts = {p.name: p for p in design.parts}
    catalog = bench_parts()
    out: dict[str, dict] = {}
    # round 3: a rigid cluster built on an axle-locked part (a liftarm arm or leg pinned to a 60483 / 7229, bricks on
    # a 32064) turns with the axle like a cross-fitted part: it counts towards the shaft's load, and its parts are
    # scaled with the other kids (the 9686 / extended designs have no such clusters)
    names = [q.name for q in design.parts]
    rigid = _components(names, [(c.child, c.parent) for c in design.connections if c.kind in RIGID_KINDS])
    cluster_of = {n: comp for comp in rigid for n in comp}
    for p in design.parts:
        bp = catalog.get(p.part)
        if bp is None or bp.kind != "axle" or p.name in skip_axles:  # skip_axles: compound bodies (true masses)
            continue
        kids = [c.child for c in design.connections if c.kind == "cross" and c.parent == p.name]
        riders = []
        for kid in kids:
            if _locks_cluster(catalog[parts[kid].part]) and design.chassis not in cluster_of.get(kid, {design.chassis}):
                riders += [n for n in sorted(cluster_of[kid]) if n != kid and n not in kids and n not in riders]
        m_kids = sum(catalog[parts[k].part].mass_g for k in kids + riders)
        if not kids or m_kids / bp.mass_g <= SHAFT_MASS_RATIO_MAX:
            continue
        total = m_kids + bp.mass_g
        m_axle = total / (1.0 + SHAFT_MASS_RATIO_TARGET)
        k = (total - m_axle) / m_kids
        for r in riders:
            out[r] = {"mass_kg": catalog[parts[r].part].mass_g * k * 1e-3}
        if rider_log is not None:  # mass moved from the riders to the shaft [g] (mechanism shafts add it to the body)
            rider_log[p.name] = sum(catalog[parts[r].part].mass_g for r in riders) * (1.0 - k)
        m_kids -= sum(catalog[parts[r].part].mass_g for r in riders)
        new_m = {kid: catalog[parts[kid].part].mass_g * k for kid in kids}
        # a light part among heavy ones (a 1.2 g gear between two 33 g wheels) would now be ~1/20 of the axle: its
        # fixed joint let it spin around the axle under load. Every part keeps at least 1/KID_MIN_RATIO of the axle,
        # taken from the heavier parts of the same shaft (the shaft total is unchanged).
        floor = m_axle / KID_MIN_RATIO
        light = [kid for kid in kids if new_m[kid] < floor]
        if light and len(light) < len(kids):
            need = sum(floor - new_m[kid] for kid in light)
            heavy_m = sum(new_m[kid] for kid in kids if kid not in light)
            for kid in kids:
                new_m[kid] = floor if kid in light else new_m[kid] * (1.0 - need / heavy_m)
        moved = 0.0
        for kid in kids:
            kb = catalog[parts[kid].part]
            out[kid] = {"mass_kg": new_m[kid] * 1e-3}
            moved += _spin_inertia(kb) - _spin_inertia(kb, new_m[kid])
        length = 2 * bp.axle_half_len * 1e-3
        transverse = m_axle * 1e-3 * length * length / 12.0
        out[p.name] = {"mass_kg": m_axle * 1e-3, "inertia_kgm2": max(moved, transverse)}
    # Technic pins between a heavy part (a 45-72 g motor) and a brick: the 0.2 g pin is the light link of the chain;
    # give each pin 1/PIN_MASS_RATIO of the heavy part's mass, taken from that part (pins sit on its face)
    held: dict[str, list[str]] = {}
    for c in design.connections:
        if c.kind == "pin":
            held.setdefault(c.child, []).append(c.parent)
    take: dict[str, float] = {}
    for pin, pars in held.items():
        heavy = max(pars, key=lambda n: catalog[parts[n].part].mass_g)
        m_heavy = catalog[parts[heavy].part].mass_g
        m_pin = out[pin]["mass_kg"] * 1e3 if pin in out else catalog[parts[pin].part].mass_g
        target = m_heavy / PIN_MASS_RATIO
        if target > m_pin:
            out[pin] = {"mass_kg": target * 1e-3, "inertia_kgm2": 0.5 * target * 1e-3 * (8e-3) ** 2}
            take[heavy] = take.get(heavy, 0.0) + target - m_pin
    # the motor's output shaft (0.3-0.9 g) carries the full stall torque between a 33-72 g motor and its gear: give
    # it 1/MOTOR_SHAFT_MASS_RATIO of the motor's mass, taken from the motor
    for c in design.connections:
        if c.kind != "motor" or c.child not in parts or c.parent not in parts:
            continue
        m_motor = catalog[parts[c.parent].part].mass_g - take.get(c.parent, 0.0)
        m_shaft = out.get(c.child, {}).get("mass_kg", catalog[parts[c.child].part].mass_g * 1e-3) * 1e3
        target = m_motor / MOTOR_SHAFT_MASS_RATIO
        if design.part_set != "9686" and target > m_shaft:  # the validated 9686 designs keep their masses
            half = catalog[parts[c.child].part].axle_half_len * 1e-3
            inertia = max(out.get(c.child, {}).get("inertia_kgm2", 0.0), target * 1e-3 * (2 * half) ** 2 / 12.0)
            out[c.child] = {"mass_kg": target * 1e-3, "inertia_kgm2": inertia}
            take[c.parent] = take.get(c.parent, 0.0) + target - m_shaft
    # the pinion on the motor shaft (an 8T is 0.22 g) sits between the heavy conditioned shaft and the gear teeth
    # that carry the full motor torque: under high load (a 55 deg climb, a rope car pulling itself) its fixed joint
    # did not hold it and it spun freely around its shaft. Every part cross-fitted on the motor shaft gets at least
    # 1/MOTOR_PINION_RATIO of the (conditioned) shaft mass, taken from the motor (total mass unchanged).
    for c in design.connections:
        if c.kind != "motor" or design.part_set == "9686" or c.child not in out:
            continue
        m_shaft = out[c.child]["mass_kg"] * 1e3
        for k in (cc.child for cc in design.connections if cc.kind == "cross" and cc.parent == c.child):
            kb = catalog[parts[k].part]
            target = m_shaft / MOTOR_PINION_RATIO
            if kb.mass_g < target and k not in out:
                out[k] = {"mass_kg": target * 1e-3, "inertia_kgm2": max(_spin_inertia(kb, target), 1e-9)}
                take[c.parent] = take.get(c.parent, 0.0) + target - kb.mass_g
    for heavy, dm in take.items():
        m = out[heavy]["mass_kg"] * 1e3 if heavy in out else catalog[parts[heavy].part].mass_g
        out[heavy] = {**out.get(heavy, {}), "mass_kg": (m - dm) * 1e-3}
    return out


# ----------------------------------------------------------------------------------------------------------------
# gear phases (for spawning)
# ----------------------------------------------------------------------------------------------------------------


def gear_spins(design: Design, tooth_phase_deg: dict[str, float] | None = None) -> dict[str, float]:
    """Spin [deg] of every meshing gear so that its teeth interleave with its partners'.

    Gears have yaw 0 (axis along y); a tooth centre at angle ``phase`` = atan2(z, x) in the gear frame is at
    ``phase - spin`` in the design x-z plane. The first gear of a chain points a tooth at its partner; every
    partner gets a tooth gap on the line of centres, shifted by the rolling relation when the driving gear's
    nearest tooth is off that line.
    """
    if tooth_phase_deg is None:
        tooth_phase_deg = {pid: m["tooth_phase_deg"] for pid, m in part_metadata().items() if "tooth_phase_deg" in m}
    P = place(design)
    spins: dict[str, float] = {}

    def wrap(a, period):
        return (a + period / 2) % period - period / 2

    for m in design.gear_meshes:
        a, b = P[m.gear_a], P[m.gear_b]
        za, zb = a.bp.teeth, b.bp.teeth
        if m.gear_b in spins and m.gear_a not in spins:
            a, b, za, zb = b, a, zb, za
        th_ab = math.degrees(math.atan2(b.p[2] - a.p[2], b.p[0] - a.p[0]))
        th_ba = th_ab + 180.0
        pa = tooth_phase_deg.get(a.inst.part, 0.0)
        pb = tooth_phase_deg.get(b.inst.part, 0.0)
        if a.inst.name not in spins:
            spins[a.inst.name] = pa - th_ab
        delta = wrap(pa - spins[a.inst.name] - th_ab, 360.0 / za)
        if b.inst.name not in spins:
            spins[b.inst.name] = pb + 180.0 / zb - th_ba + delta * za / zb
    return {k: v % 360.0 for k, v in spins.items()}
