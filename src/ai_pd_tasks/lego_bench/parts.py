# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Connection geometry of the catalog parts, in each part's own frame (millimetres).

Frames are those of the converted USD parts (LDraw origin, Z up; see :mod:`ai_pd_tasks.lego.ldraw`):

* bricks / plates / battery box: origin at the top-centre of the body (studs stick up 1.6 mm above z = 0);
  Technic brick holes run along local y, 5.6 mm above the underside (z = -4 mm), between the studs;
* axles: centred, along local x;
* gears, wheels, bushes: centred, rotation axis local y;
* PF M-motor: origin on the output axis at the output face; body along +y (0..48 mm), 3 x 6 studs underneath;
* PF L / XL motors (studless): same output frame (body along +y, 0..56 / 0..48 mm); mounted with Technic pins in
  the pin holes of their faces (:attr:`BenchPart.pin_holes`);
* Technic pins: centred, along local x (8 mm each side of the collar);
* wheels with rubber tyres: rotation axis local y through the origin; the tyre and the axle bore are not always
  centred on the origin (:attr:`BenchPart.hub_mm`, ``shape.centre``).

Values come from ``assets/data/lego/parts/metadata.json`` (bounds, centre of mass, gear teeth written by
``scripts/convert_lego_parts.py``) and :mod:`ai_pd_tasks.lego.catalog` (measured masses); no Isaac imports.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, replace
from functools import lru_cache

from ai_pd_tasks.lego.catalog import (
    INVENTORY_MOTOR_LINES,
    MOTOR_LINES,
    PART_TYPES,
    PART_USD_DIR,
    TYRE_FRICTION,
    read_inventory,
)

STUD = 8.0  # stud pitch [mm]
PLATE_H = 3.2
BRICK_H = 9.6
HOLE_BELOW_TOP = 4.0  # Technic hole centre below a Technic brick's top face
AXLE_COLLISION_RADIUS = 2.2  # analytic axle collider radius (convert_lego_parts.AXLE_COLLISION_RADIUS)
PIN_HOLE_RADIUS = 2.45
GEAR_MODULE = 1.0  # LEGO Technic gears: module 1 -> centre distance (z1 + z2) / 2 mm


@dataclass(frozen=True)
class Box:
    lo: tuple[float, float, float]
    hi: tuple[float, float, float]


@dataclass(frozen=True)
class Segs(Box):
    """A bent / shaped liftarm: centre-line segments (x0, z0, x1, z1) in the part's x-z plane, swept with ``radius``
    in x-z and +-``half_w`` in y. ``lo`` / ``hi`` are its bounding box (every Box consumer keeps working)."""

    segs: tuple[tuple[float, float, float, float], ...] = ()
    radius: float = 3.6
    half_w: float = 4.0


@dataclass(frozen=True)
class Boxes(Box):
    """A part made of several boxes (a hollow housing, an L): ``boxes`` = ((lo, hi), ...) in the part frame; ``lo`` /
    ``hi`` are their bounding box (every Box consumer keeps working). Round 4a (2026-10-14)."""

    boxes: tuple = ()


@dataclass(frozen=True)
class Cyl:
    axis: str  # "x" or "y" (local)
    radius: float
    half_len: float
    centre: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class BenchPart:
    """Everything the validator needs to know about one catalog part type."""

    part_id: str  # BrickLink id (key into ai_pd_tasks.lego.catalog.PART_TYPES)
    kind: str  # technic_brick | plate | axle | gear | wheel | bush | motor | battery | pin
    title: str
    mass_g: float
    com_mm: tuple[float, float, float]
    shape: Box | Cyl
    studs_top: tuple[tuple[float, float], ...] = ()  # stud centres (x, y) on the top face (z = top_z)
    top_z: float = 0.0
    studs_bottom: tuple[tuple[float, float], ...] = ()  # anti-stud (tube) centres on the underside (z = bottom_z)
    underside_half_x: float = 0.0  # > 0: the flat underside only spans |x| <= this (the body curves up beyond it)
    bottom_z: float = 0.0
    holes: tuple[tuple[float, float], ...] = ()  # Technic hole centres (x, z); hole axis = local y, length 8 mm
    teeth: int = 0
    axle_half_len: float = 0.0
    wheel_radius: float = 0.0
    groove_radius: float = 0.0  # wedge-belt wheel: radius at the bottom of the groove
    width: float = 0.0  # extent along the rotation axis (gears, wheels, bushes)
    usd_path: str = ""
    has_usd: bool = False
    notes: str = ""
    # -- extended part set --
    hub_mm: tuple[float, float] | None = None  # axial range (local y) of the cross bore; None = +-width / 2
    tyre_mu: float = 0.0  # measured sliding friction of a rubber tyre (0 = hard ABS part)
    tyre_source: str = ""
    pin_holes: tuple[tuple[float, float, float, str], ...] = ()  # (x, y, z of the 8 mm hole's middle, axis 'x'|'y')
    motor_no_load_rpm: float = 0.0  # 0 = the PF M-motor line of ai_pd_tasks.lego.car_design
    motor_stall_torque_nm: float = 0.0
    motor_source: str = ""
    # -- round 3 parts (lego_bench.electronics) --
    sensor: str = ""  # distance | force | color
    ports: int = 0  # hub: number of device ports
    system: str = "pf"  # pf (Power Functions) | lpf2 (SPIKE / Powered Up)
    # -- inventory part set (2026-10-06) --
    axle_holes: tuple = ()  # (x, y, z, axis[, half length = 4]) cross holes; axis 'x' | 'y' | 'z' or a unit vector
    pin_segments: tuple[tuple[float, float], ...] = ()  # pins: (centre, half length) of each pin section on local x
    axle_segments: tuple[tuple[float, float], ...] = ()  # axle-pins: (centre, half length) of the axle end on local x
    male_pins: tuple = ()  # connectors with integral pins: (x, y, z, axis, half length)
    pivot: bool = False  # frictionless pin (3673, 65249): the parts it joins turn about it (revolute joint)
    free_bore: bool = False  # gear with a round bore (18946, 6542b): turns freely on its axle (hinge, not cross)
    bevel_only: bool = False  # single bevel gear: meshes only at right angles (6589, 32198)
    servo: bool = False  # PF servo: position output, PF level k -> k / 7 * 90 deg
    hole_half: float = 4.0  # half length of the part's pin holes along their axis (thin liftarm: 2 mm)
    inventory: bool = False  # added with the inventory part set (2026-10-06)
    # -- PF cables and plugs (2026-10-14, from the LDraw models) --
    pf_pads: tuple = ()  # PF connector pads (x, y, z of the pad's top centre, local; plugs stack on it along +z)
    pf_cable: tuple | None = None  # where the part's own lead leaves its body (local), or None (no lead)
    # -- round 4a (2026-10-14) --
    male_axles: tuple = ()  # integral axle ends (57585 hub): (x, y, z, axis, half length); in cross / pin holes (pin),
    # and gears / wheels / bushes cross-fit on them (cross, the part as the parent)
    # -- round 4b (2026-10-14) --
    through_bore: float = 0.0  # > 0: the motor's output is a bore through the body, local y = -this..this (pullback)

    @property
    def hub(self) -> tuple[float, float]:
        """Axial range (local y, mm) of the part's cross bore (gears, wheels, bushes)."""
        return self.hub_mm if self.hub_mm is not None else (-self.width / 2, self.width / 2)


def _grid(nx: int, ny: int, ox: float = 0.0, oy: float = 0.0) -> tuple[tuple[float, float], ...]:
    return tuple(((i - (nx - 1) / 2) * STUD + ox, (j - (ny - 1) / 2) * STUD + oy) for i in range(nx) for j in range(ny))


@lru_cache(maxsize=1)
def part_metadata() -> dict:
    path = os.path.join(PART_USD_DIR, "parts", "metadata.json")
    with open(path) as f:
        return json.load(f)


def _technic_brick(pid: str, n: int, title: str) -> dict:
    return dict(
        kind="technic_brick",
        title=title,
        shape=Box((-4 * n, -4.0, -BRICK_H), (4 * n, 4.0, 0.0)),
        studs_top=_grid(n, 1),
        studs_bottom=_grid(n, 1),
        bottom_z=-BRICK_H,
        holes=tuple(((k - 0.5 * (n - 2)) * STUD, -HOLE_BELOW_TOP) for k in range(n - 1)),
    )


def _plate(nx: int, ny: int, title: str) -> dict:
    return dict(
        kind="plate",
        title=title,
        shape=Box((-4 * nx, -4 * ny, -PLATE_H), (4 * nx, 4 * ny, 0.0)),
        studs_top=_grid(nx, ny),
        studs_bottom=_grid(nx, ny),
        bottom_z=-PLATE_H,
    )


def _axle(half: float, title: str) -> dict:
    return dict(kind="axle", title=title, shape=Cyl("x", AXLE_COLLISION_RADIUS, half), axle_half_len=half)


def _gear(teeth: int, r_out: float, half_w: float, title: str) -> dict:
    return dict(kind="gear", title=title, shape=Cyl("y", r_out, half_w), teeth=teeth, width=2 * half_w)


_GEOMETRY: dict[str, dict] = {
    "3895": _technic_brick("3895", 12, "Technic brick 1x12 with holes"),
    "3701": _technic_brick("3701", 4, "Technic brick 1x4 with holes"),
    "3710": _plate(4, 1, "Plate 1x4"),
    "3032": _plate(6, 4, "Plate 4x6"),
    "44294": _axle(27.8, "Technic axle 7"),
    "3705": _axle(15.8, "Technic axle 4"),
    "32062": _axle(7.8, "Technic axle 2 notched"),
    "94925": _gear(16, 8.66, 4.0, "Gear 16 tooth (reinforced)"),
    "3648": _gear(24, 12.96, 3.85, "Gear 24 tooth"),
    "10928": _gear(8, 4.97, 4.0, "Gear 8 tooth"),
    "4185": dict(
        kind="wheel",
        title="Technic wedge-belt wheel (24 mm)",
        shape=Cyl("y", 12.0, 2.0),
        wheel_radius=12.0,
        groove_radius=11.4,
        width=4.0,
        notes="hard ABS pulley: 0.6 mm deep, ~1 mm wide groove at the rim (collision hull)",
    ),
    "3713": dict(kind="bush", title="Technic bush", shape=Cyl("y", 3.6, 4.0), width=8.0),
    "58120c01": dict(
        kind="motor",
        title="Power Functions M-motor",
        shape=Box((-11.6, 0.0, -12.0), (11.6, 48.0, 11.6)),
        # the underside takes studs on a 2 x 6 grid half a stud off the 3-wide top (LDraw 59143: anti-stud bars at
        # x = +-4 mm); was _grid(3, 6) until 2026-10-13, which put the studs into the solid underside
        studs_bottom=_grid(2, 6, 0.0, 24.0),
        underside_half_x=8.6,  # rounded sides: flat only within |x| <= 8.6 mm, 7 mm higher at the edges (LDraw mesh)
        bottom_z=-12.0,
        notes="output axis = local y through the origin; 400 rpm no load, 0.15 N m stall",
    ),
    "87513c01": dict(
        kind="battery",
        title="Power Functions AAA battery box",
        shape=Box((-32.0, -16.0, -38.4), (32.0, 16.0, 0.0)),
        # the real top (LDraw 87513): two rows of 8 studs along the long edges and 2 at the -x end; the middle strip
        # holds the switch (x -11..0) and the PF connector pad (x 8..24, y -8..8), where nothing may be built
        studs_top=tuple((x, y) for x in (-28.0, -20.0, -12.0, -4.0, 4.0, 12.0, 20.0, 28.0) for y in (-12.0, 12.0))
        + ((-28.0, -4.0), (-28.0, 4.0)),
        studs_bottom=_grid(8, 4),
        bottom_z=-38.4,
        notes="6 AAA cells included in the mass",
    ),
}


def _pin_holes_y(*xz: tuple[float, float], y: float) -> tuple[tuple[float, float, float, str], ...]:
    return tuple((x, y, z, "y") for x, z in xz)


def _pin_holes_x(*yz: tuple[float, float], x: float) -> tuple[tuple[float, float, float, str], ...]:
    return tuple((sx * x, y, z, "x") for sx in (-1.0, 1.0) for y, z in yz)


def _tyred_wheel(pid: str, r: float, y0: float, y1: float, hub: tuple[float, float], title: str, notes: str) -> dict:
    from ai_pd_tasks.lego.catalog import INVENTORY_TYRE_FRICTION  # noqa: PLC0415

    t = TYRE_FRICTION[pid] if pid in TYRE_FRICTION else INVENTORY_TYRE_FRICTION[pid]
    return dict(
        kind="wheel",
        title=title,
        shape=Cyl("y", r, 0.5 * (y1 - y0), (0.0, 0.5 * (y0 + y1), 0.0)),
        wheel_radius=r,
        groove_radius=r,
        width=y1 - y0,
        hub_mm=hub,
        tyre_mu=t["mu"],
        tyre_source=f"{t['source']}, {t['surface']}",
        notes=notes,
    )


def _motor(pid: str, box: Box, holes, title: str, notes: str) -> dict:
    line = MOTOR_LINES[pid]
    return dict(
        kind="motor",
        title=title,
        shape=box,
        pin_holes=holes,
        motor_no_load_rpm=line["no_load_rpm"],
        motor_stall_torque_nm=line["stall_torque_nm"],
        motor_source=line["source"],
        notes=notes,
    )


EXTENDED_GEOMETRY: dict[str, dict] = {
    "3894": _technic_brick("3894", 6, "Technic brick 1x6 with holes"),
    "3702": _technic_brick("3702", 8, "Technic brick 1x8 with holes"),
    "3703": _technic_brick("3703", 16, "Technic brick 1x16 with holes"),
    "32073": _axle(19.8, "Technic axle 5"),
    "3706": _axle(23.8, "Technic axle 6"),
    "3707": _axle(31.8, "Technic axle 8"),
    "3737": _axle(39.8, "Technic axle 10"),
    "3708": _axle(47.8, "Technic axle 12"),
    "3649": _gear(40, 20.94, 4.0, "Gear 40 tooth"),
    "4716": _gear(1, 5.08, 8.0, "Worm gear (single start, meshes at right angles with a spur gear: 1 : z)"),
    "32270": _gear(12, 6.8, 4.0, "Gear 12 tooth double bevel (meshes as a spur gear)"),
    "32269": _gear(20, 10.8, 4.0, "Gear 20 tooth double bevel (meshes as a spur gear)"),
    "32498": _gear(36, 18.8, 4.0, "Gear 36 tooth double bevel (meshes as a spur gear)"),
    "2780": dict(
        kind="pin",
        title="Technic pin with friction ridges",
        shape=Cyl("x", AXLE_COLLISION_RADIUS, 8.0),
        axle_half_len=8.0,
        notes="joins two coaxial pin holes (8 mm of the pin in each); a part held by one pin can turn about it",
    ),
    "3482c01": _tyred_wheel(
        "3482c01",
        12.4,
        -4.0,
        4.0,
        (-4.0, 4.0),
        "Wheel 24 x 8 mm with rubber tyre (3482 / 3483)",
        "rubber tyre 12.4 mm radius, 7.2 mm tread; rim 8 mm wide",
    ),
    "56904c01": _tyred_wheel(
        "56904c01",
        21.42,
        -8.0,
        7.2,
        (-8.0, 0.0),
        "Wheel 43.2 x 14 mm with rubber tyre (56904 / 56898)",
        "rubber tyre 21.4 mm radius, 14.4 mm tread; the axle bore is on the -y half of the hub (y = -8..0)",
    ),
    "32020c01": _tyred_wheel(
        "32020c01",
        31.32,
        -13.8,
        6.2,
        (-8.0, 4.0),
        "Wheel 62.4 x 20 mm with rubber tyre (32020 / 32019)",
        "rubber tyre 31.3 mm radius, 20 mm wide, centred at y = -3.8; axle bore y = -8..4",
    ),
    "99499c01": _motor(
        "99499c01",
        Box((-12.0, 0.0, -15.8), (12.0, 56.0, 15.8)),
        _pin_holes_y((-8.0, 0.0), (8.0, 0.0), (0.0, -8.0), (0.0, 8.0), y=4.0)
        + _pin_holes_y((-8.0, 0.0), (8.0, 0.0), y=52.0)
        + _pin_holes_x((4.0, -8.0), (4.0, 8.0), (52.0, -8.0), (52.0, 8.0), x=8.0),
        "Power Functions L-motor",
        "studless: pin it on with Technic pins (front face pin holes at x = +-8 / z = +-8 mm around the output); "
        "390 rpm no load, 0.18 N m stall (Philo)",
    ),
    "58121c01": _motor(
        "58121c01",
        Box((-20.0, 0.0, -19.74), (20.0, 48.0, 19.74)),
        _pin_holes_y((-8.0, 0.0), (8.0, 0.0), (0.0, -8.0), (0.0, 8.0), (0.0, -16.0), (0.0, 16.0), y=4.0)
        + _pin_holes_x((4.0, 0.0), (44.0, 0.0), x=16.0),
        "Power Functions XL-motor",
        "studless: pin it on with Technic pins (front face pin holes at x = +-8 / z = +-8, +-16 mm); "
        "220 rpm no load, 0.40 N m stall (Philo)",
    ),
}
"""Geometry of the extended part set (``catalog.EXTENDED_PART_TYPES``)."""
_GEOMETRY.update(EXTENDED_GEOMETRY)


# ----------------------------------------------------------------------------------------------------------------
# Inventory part set (2026-10-06): the user's photographed inventory (catalog.INVENTORY_PART_TYPES). Frames are the
# converted USDs' (LDraw origin, Z up); hole positions come from the LDraw hole primitives (peghole / connhole /
# beamhole / axlehol* / bush0), checked against the part bounds. Liftarms are in :mod:`.liftarms` (bench frame).
# ----------------------------------------------------------------------------------------------------------------


def _brick(nx: int, ny: int, title: str) -> dict:
    """A plain brick (9.6 mm tall, studs on top, anti-studs below)."""
    d = _plate(nx, ny, title)
    d.update(shape=Box((-4 * nx, -4 * ny, -BRICK_H), (4 * nx, 4 * ny, 0.0)), bottom_z=-BRICK_H)
    return d


def _pin(title: str, half: float, segs=None, axle_segs=(), centre: float = 0.0, notes: str = "", **kw) -> dict:
    return dict(
        kind="pin",
        title=title,
        shape=Cyl("x", AXLE_COLLISION_RADIUS, half, (centre, 0.0, 0.0)),
        axle_half_len=half,
        pin_segments=tuple(segs) if segs is not None else ((centre, half),),
        axle_segments=tuple(axle_segs),
        notes=notes,
        **kw,
    )


def _conn(title: str, lo, hi, pin_holes=(), axle_holes=(), male_pins=(), notes: str = "") -> dict:
    return dict(kind="connector", title=title, shape=Box(lo, hi), pin_holes=tuple(pin_holes),
                axle_holes=tuple(axle_holes), male_pins=tuple(male_pins), notes=notes)


INVENTORY_GEOMETRY: dict[str, dict] = {
    # -- electrics --
    "59510c01": dict(
        kind="battery",
        title="Power Functions AA battery box (8881)",
        shape=Box((-44.0, -16.0, -28.0), (44.0, 16.0, 28.0)),
        pin_holes=tuple((sx * 40.0, sy * 12.0, z, "y") for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for z in (-8.0, 0.0, 8.0)),
        notes="studless 88 x 32 x 56 mm body (origin at its centre); mount it with Technic pins in the 3 + 3 pin holes "
        "of each end cover (x = +-40, z = -8 / 0 / 8, both faces y = +-12); 6 AA cells in the mass; PF output on top",
    ),
    "58123c01": dict(
        kind="receiver",
        title="Power Functions IR receiver (8884)",
        shape=Box((-16.0, -16.0, -9.6), (16.0, 16.0, 22.4)),
        studs_bottom=_grid(4, 4),
        bottom_z=-9.6,
        notes="4 x 4 studs underneath; two outputs (red A / blue B) on its channel 1-4; plugs onto a battery box; "
        "needs line of sight to the IR transmitter",
    ),
    "99498c01": dict(
        kind="motor",
        title="Power Functions servo motor (88004)",
        shape=Box((-12.0, 0.0, -11.8), (12.0, 55.6, 27.8)),
        pin_holes=_pin_holes_y((-8.0, 8.0), (0.0, 8.0), (8.0, 8.0), (0.0, 16.0), (0.0, 24.0), (0.0, -8.0), y=4.0)
        + _pin_holes_y((-8.0, 8.0), (0.0, 8.0), (8.0, 8.0), (0.0, 16.0), (0.0, 24.0), (0.0, -8.0), y=51.6),
        motor_no_load_rpm=INVENTORY_MOTOR_LINES["99498c01"]["no_load_rpm"],
        motor_stall_torque_nm=INVENTORY_MOTOR_LINES["99498c01"]["stall_torque_nm"],
        motor_source=INVENTORY_MOTOR_LINES["99498c01"]["source"],
        servo=True,
        notes="position servo: PF level k turns the output to k / 7 * 90 deg (7 steps each way, 0 = centre); output "
        "on local y through the origin (front face), pin holes in the front and back faces",
    ),
    # -- wheels --
    "56145c01": _tyred_wheel("56145c01", 21.6, -10.8, 10.8, (-10.0, 10.0),
                             "Wheel 30.4 x 20 rim with tyre 43.2 x 22 ZR (56145 / 44309)",
                             "rubber tyre 21.6 mm radius, 21.6 mm wide, centred on the origin"),
    "55982c01": _tyred_wheel("55982c01", 15.2, -6.8, 8.0, (-6.8, 8.0),
                             "Wheel 18 x 14 rim with tyre 30.4 x 14 offset tread (55982 / 30391)",
                             "rubber tyre 15.2 mm radius, 13.6 mm tread (y = -6.8..6.8); rim to y = 8"),
    "4185t": _tyred_wheel("4185t", 15.2, -2.0, 2.0, (-2.0, 2.0),
                          "Wedge-belt wheel 4185 with rubber tyre 2815",
                          "thin rubber tyre (3.2 mm wide) 15.2 mm radius on the 24 mm pulley; uses one 4185 and one 2815"),
    "55423c01": dict(
        kind="wheel",
        title="RC train wheel with rubber friction band (55423)",
        shape=Cyl("y", 12.0, 2.5, (0.0, 1.5, 0.0)),
        wheel_radius=12.0,
        groove_radius=9.15,
        width=5.0,
        hub_mm=(-1.0, 4.0),
        notes="on a flat floor it rides on its hard ABS flange (r = 12.0 mm, 1.2 mm wide), not on the rubber band "
        "(r = 9.15 mm, which only touches a rail): simulated as an ABS wheel",
    ),
    # -- Technic bricks --
    "2730": _technic_brick("2730", 10, "Technic brick 1x10 with holes"),
    "32018": _technic_brick("32018", 14, "Technic brick 1x14 with holes"),
    "3700": dict(_technic_brick("3700", 2, "Technic brick 1x2 with hole"), holes=((0.0, -HOLE_BELOW_TOP),)),
    "32000": dict(_technic_brick("32000", 2, "Technic brick 1x2 with 2 holes (under the studs)"),
                  holes=((-4.0, -HOLE_BELOW_TOP), (4.0, -HOLE_BELOW_TOP))),
    "6541": dict(_technic_brick("6541", 1, "Technic brick 1x1 with hole"), holes=((0.0, -HOLE_BELOW_TOP),)),
    # -- pins (local x; pin sections ``pin_segments``) --
    "3673": _pin("Technic pin without friction ridges (pivot)", 8.0, pivot=True,
                 notes="frictionless: the parts it joins turn about it (one fixed joint, the others revolute)"),
    "6558": _pin("Technic pin 3L with friction ridges", 12.0, notes="joins two or three parts"),
    "32054": _pin("Technic pin 3L with friction ridges and stop bush", 12.0, segs=((4.0, 8.0),),
                  notes="2L friction pin x = -4..12 mm, stop bush x = -12..-4 (the bush end does not go in a hole)"),
    "43093": _pin("Technic axle-pin 1L/1L with friction", 8.0, segs=((4.0, 4.0),), axle_segs=((-4.0, 4.0),),
                  notes="pin x = 0..8 mm (in a pin hole), axle x = -8..0 (in an axle hole: locked, or a pin hole)"),
    "65249": _pin("Technic axle-pin 1L pin (no friction) + 2L axle", 12.0, segs=((-8.0, 4.0),), axle_segs=((4.0, 8.0),),
                  pivot=True, notes="frictionless pin x = -12..-4 mm (turns in its hole), 2L axle x = -4..12"),
    "15100": _pin("Technic pin 1L with friction and perpendicular pin hole", 7.8, segs=((8.0, 4.0),), centre=4.2,
                  pin_holes=((0.0, 0.0, 0.0, "y"),),
                  notes="1L friction pin x = 4..12 mm, a pin hole along y through its body at the origin"),
    "87082": _pin("Technic pin 3L with centre pin hole", 12.0, segs=((-8.0, 4.0), (8.0, 4.0)),
                  pin_holes=((0.0, 0.0, 0.0, "y"),), notes="two 1L friction pins (x = +-4..12), centre pin hole along y"),
    # -- axles --
    "4519": _axle(11.8, "Technic axle 3"),
    "60485": _axle(35.8, "Technic axle 9"),
    "23948": _axle(43.8, "Technic axle 11"),
    "50451": _axle(63.8, "Technic axle 16"),
    "24316": dict(_axle(11.9, "Technic axle 3 with stop"), notes="stop at +x (the stop is visual in collision)"),
    "15462": dict(_axle(19.9, "Technic axle 5 with stop"), notes="stop at +x"),
    "87083": dict(_axle(15.9, "Technic axle 4 with stop"), notes="stop at +x"),
    "6587": dict(_axle(11.8, "Technic axle 3 with stud"), notes="stud end at +x (visual)"),
    "4265c": dict(kind="bush", title="Technic bush 1/2 smooth", shape=Cyl("y", 3.6, 2.0), width=4.0),
    # -- gears --
    "69778": _gear(12, 7.08, 4.0, "Gear 12 tooth (spur, 4 mm wide teeth)"),
    "4019": _gear(16, 8.66, 4.0, "Gear 16 tooth (axle hole, open sides)"),
    "18946": dict(_gear(16, 8.71, 4.0, "Gear 16 tooth with clutch on both sides (round bore: turns freely)"),
                  free_bore=True),
    "6542b": dict(_gear(16, 8.74, 4.0, "Gear 16 tooth with clutch, smooth centre (round bore: turns freely)"),
                  free_bore=True),
    "6589": dict(kind="gear", title="Gear 12 tooth bevel (meshes at right angles only)",
                 shape=Cyl("y", 6.4, 2.0, (0.0, 0.8, 0.0)), teeth=12, width=4.0, hub_mm=(-1.2, 2.8), bevel_only=True),
    "32198": dict(kind="gear", title="Gear 20 tooth bevel (meshes at right angles only)",
                  shape=Cyl("y", 10.4, 2.0, (0.0, -0.8, 0.0)), teeth=20, width=4.0, hub_mm=(-2.8, 1.2), bevel_only=True),
    # -- connectors (axle holes lock them to an axle with a cross connection) --
    "6536": _conn("Technic axle and pin connector perpendicular", (-4.0, -4.0, -11.6), (4.0, 4.0, 3.6),
                  pin_holes=((0.0, 0.0, -8.0, "y"),), axle_holes=((0.0, 0.0, 0.0, "x"),)),
    "32184": _conn("Technic axle and pin connector perpendicular 3L", (-4.0, -4.0, -11.6), (4.0, 4.0, 11.6),
                   pin_holes=((0.0, 0.0, 0.0, "y"),), axle_holes=((0.0, 0.0, -8.0, "x"), (0.0, 0.0, 8.0, "x"))),
    "42003": _conn("Technic axle and pin connector perpendicular 3L with 2 pin holes", (-4.0, -4.0, -11.6),
                   (4.0, 4.0, 11.6), pin_holes=((0.0, 0.0, 0.0, "y"), (0.0, 0.0, -8.0, "y")),
                   axle_holes=((0.0, 0.0, 8.0, "x"),)),
    "32013": _conn("Technic angle connector #1", (-4.0, -12.0, -3.6), (4.0, 3.6, 3.6),
                   pin_holes=((0.0, 0.0, 0.0, "x"),), axle_holes=((0.0, -8.0, 0.0, "y"),)),
    "32015": _conn("Technic angle connector #5 (112.5 deg)", (-4.0, -12.0, -3.6), (4.0, 7.82, 12.42),
                   pin_holes=((0.0, 0.0, 0.0, "x"),),
                   axle_holes=((0.0, -8.0, 0.0, "y"), (0.0, 3.06, 7.39, (0.0, -0.3827, -0.9239)))),
    "32034": _conn("Technic angle connector #2 (180 deg)", (-4.0, -12.0, -3.6), (4.0, 12.0, 3.6),
                   pin_holes=((0.0, 0.0, 0.0, "x"),), axle_holes=((0.0, -8.0, 0.0, "y"), (0.0, 8.0, 0.0, "y")),
                   notes="joins two axles end to end (one cross connection per axle: they become one shaft)"),
    "48989": _conn("Technic pin connector perpendicular 3L with 4 pins", (-4.0, -12.0, -11.6), (4.0, 12.0, 11.6),
                   pin_holes=((0.0, 0.0, 8.0, "x"), (0.0, 0.0, -8.0, "x"), (0.0, 0.0, 0.0, "y")),
                   male_pins=tuple((0.0, sy * 8.0, sz * 8.0, "y", 4.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)),
                   notes="4 integral 1L friction pins along y (it is the pin of a 'pin' connection)"),
    # -- plates and bricks --
    "3001": _brick(4, 2, "Brick 2x4"),
    "3020": _plate(4, 2, "Plate 2x4"),
    "3021": _plate(3, 2, "Plate 2x3"),
    "3022": _plate(2, 2, "Plate 2x2"),
    "3023": _plate(2, 1, "Plate 1x2"),
    "3623": _plate(3, 1, "Plate 1x3"),
    "3795": _plate(6, 2, "Plate 2x6"),
    "3034": _plate(8, 2, "Plate 2x8"),
    "4477": _plate(10, 1, "Plate 1x10"),
    "3028": _plate(12, 6, "Plate 6x12"),
    "3030": _plate(10, 4, "Plate 4x10"),
    "3031": _plate(4, 4, "Plate 4x4"),
    "3033": _plate(10, 6, "Plate 6x10"),
    "3035": _plate(8, 4, "Plate 4x8"),
    "3036": _plate(8, 6, "Plate 6x8"),
    "3958": _plate(6, 6, "Plate 6x6"),
    "41539": _plate(8, 8, "Plate 8x8"),
    "92438": _plate(16, 8, "Plate 8x16"),
    # -- round 2 (2026-10-06): tread sprocket (lego_bench.treads: a 3873 tread loop around two of them) --
    "57520": dict(kind="wheel", title="Technic tread sprocket wheel small (25.4 mm, 6 lobes, 12 tread links per turn)",
                  shape=Cyl("y", 12.8, 4.0), wheel_radius=12.8, groove_radius=12.8, width=8.0, hub_mm=(-4.0, 4.0),
                  notes="cross hole; alone it is a hard ABS 25.6 mm lobed wheel; in a design 'treads' entry it carries a "
                  "3873 tread loop (ground contact at the tread's grouser tips, 15.56 mm from the axle)"),
    # -- round 3 (2026-10-07): gear racks (lego_bench.racks: a 'racks' slide = racks + pinion + guides). Bench frame =
    # LDraw: rack along x, teeth on top (+z), width y; tooth pitch 3.2 mm, roots at x = multiples of 3.2 --
    "3743": dict(kind="rack", title="Technic gear rack 1x4 (10 teeth, studs underneath)",
                 shape=Box((-16.0, -4.0, -3.2), (16.0, 4.0, 2.4)), studs_bottom=_grid(4, 1), bottom_z=-3.2,
                 notes="plate-thick body (z -3.2..0.4), 10 teeth over the full length (tips z 2.4, pitch line z 1.4); "
                 "4 anti-studs underneath; only in a 'racks' slide"),
    "6630": dict(kind="rack", title="Technic gear rack 1x8 with holes (14 teeth)",
                 shape=Box((-32.0, -4.0, -4.0), (32.0, 4.0, 1.6)), pin_holes=((-28.0, 0.0, -2.0, "z"), (28.0, 0.0, -2.0, "z")),
                 hole_half=2.0,
                 notes="4 mm thick (z -4..0), 14 teeth between the round ends (x -22.4..22.4, tips z 1.6, pitch line "
                 "z 0.6), a 4 mm pin hole along z in each end (x = +-28); no studs; only in a 'racks' slide"),
    "87761": dict(kind="rack", title="Technic gear rack 1x7 with axle and pin holes (8 teeth, end stops)",
                  shape=Box((-28.0, -4.0, -4.0), (28.0, 4.0, 4.0)), pin_holes=((-24.0, 0.0, 0.0, "z"), (24.0, 0.0, 0.0, "z")),
                  axle_holes=((-16.0, 0.0, 0.0, "y"), (16.0, 0.0, 0.0, "y")),
                  notes="8 x 8 mm section, 8 teeth in the middle (x -12.8..12.8, tips z 3, pitch line z 2) between raised "
                  "end blocks (they stop the pinion); pin holes along z at x = +-24, cross holes along y at x = +-16; "
                  "only in a 'racks' slide"),
}
"""Geometry of the inventory part set's additions (``catalog.INVENTORY_PART_TYPES``; liftarms in :mod:`.liftarms`)."""


def _boxes(*bs) -> Boxes:
    lo = tuple(min(b[0][i] for b in bs) for i in range(3))
    hi = tuple(max(b[1][i] for b in bs) for i in range(3))
    return Boxes(lo, hi, boxes=tuple(bs))


_R3 = 0.5 * 3**0.5
# round 4a (2026-10-14): from the LDraw primitives (peghole / connhole / connhol3 / connect7 / axlehol* / axleend20),
# checked against the part bounds. 57585, 55615, 6641 and 57515 are in the bench frame of :mod:`.inventory_r4` (their
# main holes along y), the others in the converted frame. Mesh rules of the knob / bevel-half gears: inventory_r4.
INVENTORY_GEOMETRY.update({
    "57519": dict(kind="wheel", title="Technic tread sprocket wheel large (40.4 mm lobes, cross hole, 6 pin holes)",
                  shape=Cyl("y", 20.4, 7.28), wheel_radius=20.4, groove_radius=20.4, width=14.56, hub_mm=(-4.0, 4.0),
                  pin_holes=tuple((8.0 * math.sin(math.radians(a)), 0.0, 8.0 * math.cos(math.radians(a)), "y")
                                  for a in range(0, 360, 60)),
                  notes="hard ABS lobed wheel (lobe tips r 20.4 mm in the mid plane, valleys r 17.8; body r 15.3 at "
                  "y = +-7.3); cross hole y = -4..4, 6 pin holes along y on a r 8 mm circle; no tread model (57520 has "
                  "one)"),
    "32072": dict(_gear(4, 12.0, 4.0, "Technic knob wheel (4 knobs; meshes with another knob wheel, 1 : 1)"),
                  notes="4 knobs at r 8.8 mm; right angles: see inventory_r4 (e.g. 8 / 12 mm from the other's axis), "
                  "parallel: 16.8..20 mm apart (the 2 x 1 stud diagonal); simulated as a gear joint"),
    "69762": dict(_gear(14, 10.8, 4.0, "Gear 14 tooth bevel thick (half bevel, teeth on the -y face)"), bevel_only=True,
                  notes="right angles only, teeth facing local -y: with the 22T 69761 (14 : 22; 14T centre 16 mm, 22T "
                  "centre 8 mm from the other's axis) or another 14T (12 / 12 mm); simulated as a gear joint"),
    "69761": dict(_gear(22, 16.45, 4.0, "Gear 22 tooth bevel half with pin hole (round bore: turns freely)"),
                  bevel_only=True, free_bore=True,
                  notes="round bore (y = 0..4): an idler hinged on its axle; right angles with the 14T 69762 only (22T "
                  "centre 8 mm from the 14T's axis, teeth facing local -y)"),
    "57585": dict(kind="connector", title="Technic axle connector hub with 3 axles",
                  shape=Segs((-10.79, -4.0, -7.5), (10.79, 4.0, 12.0),
                             segs=((0.0, 0.0, 0.0, 9.0), (0.0, 0.0, 9.0 * _R3, -4.5), (0.0, 0.0, -9.0 * _R3, -4.5)),
                             radius=3.0, half_w=4.0),
                  axle_holes=((0.0, 0.0, 0.0, "y"),),
                  male_axles=((0.0, 0.0, 8.0, (0.0, 0.0, 1.0), 4.0), (8.0 * _R3, 0.0, -4.0, (_R3, 0.0, -0.5), 4.0),
                              (-8.0 * _R3, 0.0, -4.0, (-_R3, 0.0, -0.5), 4.0)),
                  notes="bench frame: cross hole along y through the hub (r 3.6, y = -4..4), three 1L axles in the x-z "
                  "plane (r 4..12 mm; +z and +-120 deg): each goes in a cross hole (locked) or a pin hole ('pin', the "
                  "hub as the child), and a gear / wheel / bush cross-fits on it ('cross', the hub as the parent)"),
    "55615": dict(_conn("Technic pin connector perpendicular 3x3 bent with 4 pins", (0, 0, 0), (0, 0, 0),
                        pin_holes=((0.0, 0.0, 0.0, "y"), (16.0, 0.0, 0.0, "y"), (0.0, 0.0, 16.0, "y"),
                                   (8.0, 0.0, 0.0, "z"), (0.0, 0.0, 8.0, "x")),
                        male_pins=((-8.0, 0.0, 0.0, "x", 4.0), (-8.0, 0.0, 16.0, "x", 4.0), (0.0, 0.0, -8.0, "z", 4.0),
                                   (16.0, 0.0, -8.0, "z", 4.0)),
                        notes="bench frame: an L in the x-z plane (arms along +x and +z, 3 holes each, the corner "
                        "shared); pin holes along y at the corner and both arm ends, one across each arm's middle "
                        "(along z at x = 8, along x at z = 8); 4 integral 1L friction pins pointing -x (corner, z-arm "
                        "end) and -z (corner, x-arm end)"),
                  shape=_boxes(((-3.6, -4.0, -3.6), (19.6, 4.0, 3.6)), ((-3.6, -4.0, -3.6), (3.6, 4.0, 19.6)),
                               ((-12.0, -2.4, -2.4), (-3.6, 2.4, 2.4)), ((-12.0, -2.4, 13.6), (-3.6, 2.4, 18.4)),
                               ((-2.4, -2.4, -12.0), (2.4, 2.4, -3.6)), ((13.6, -2.4, -12.0), (18.4, 2.4, -3.6)))),
    "6575": dict(kind="connector", title="Technic cam (4 mm thick, 4 cross holes in a line)",
                 shape=Segs((-7.6, -2.0, -11.6), (7.6, 2.0, 11.6), segs=((0.0, -4.0, 0.0, 4.0),), radius=7.6,
                            half_w=2.0),
                 axle_holes=((0.0, 0.0, -8.0, "y", 2.0), (0.0, 0.0, 0.0, "y", 2.0), (0.0, 0.2, 4.0, "y", 1.8),
                             (0.0, 0.0, 8.0, "y", 2.0)),
                 notes="egg-shaped cam in the x-z plane, 4 mm thick (y = -2..2); cross holes along y at z = -8 (narrow "
                 "slit), 0, +4 (recessed, 3.6 mm deep) and +8: cross-fit it on an axle (a crank / eccentric)"),
    "6641": dict(kind="connector", title="Technic changeover catch (lever with a cross hole and an axle socket)",
                 shape=Box((-18.4, -4.0, -3.6), (2.4, 4.0, 3.6)),
                 axle_holes=((-8.0, 0.0, 0.0, "y"), (-1.0, 0.0, 0.0, "x", 3.4)),
                 notes="bench frame: a cross hole along y at x = -8 (it turns with its axle there), a blind cross "
                 "socket along x at the hub end (x = -4.4..2.4: an axle end goes in), the lever tip at x = -18.4; the "
                 "shifting function is not modelled"),
    "35188": dict(kind="connector", title="Technic changeover rotary catch (round, cross hole)",
                  shape=Cyl("y", 11.04, 5.91), axle_holes=((0.0, 0.0, 0.0, "y"),),
                  notes="round shifter body (r 11.04, y = -5.91..5.91) with a cross hole along y (y = -4..4); "
                  "cross-fit it on an axle; the shifting function is not modelled"),
    "6588": dict(kind="technic_brick", title="Technic gearbox 2x4x3 1/3 (worm + 24T housing)",
                 shape=_boxes(((-16.0, -8.0, -16.0), (16.0, 8.0, -12.3)),  # floor
                              ((9.0, -5.0, -12.3), (12.4, 5.0, -2.0)),  # worm bearing blocks
                              ((-12.4, -5.0, -12.3), (-9.0, 5.0, -2.0)),
                              ((-12.4, 4.0, -2.0), (12.4, 8.0, 16.0)),  # side walls
                              ((-12.4, -8.0, -2.0), (12.4, -4.0, 16.0)),
                              *(((min(sx * 12.4, sx * 16.0), min(sy * 5.0, sy * 8.0), -12.3),
                                 (max(sx * 12.4, sx * 16.0), max(sy * 5.0, sy * 8.0), 16.0))
                                for sx in (-1, 1) for sy in (-1, 1))),  # corner posts
                 studs_bottom=_grid(4, 2), bottom_z=-16.0, hole_half=2.0,
                 pin_holes=tuple((x, sy * 6.0, 8.8, "y") for x in (-8.0, 0.0, 8.0) for sy in (-1.0, 1.0))
                 + tuple((sx * 10.2, 0.0, -7.2, "x") for sx in (-1.0, 1.0))
                 + tuple((sx * 14.2, 0.0, 8.8, "x") for sx in (-1.0, 1.0)),
                 notes="32 x 16 x 32 mm housing, 2 x 4 anti-studs underneath (z = -16), open top and middle; worm "
                 "axle along x at z = -7.2 through the end blocks (pin holes x = +-10.2), 24T axle along y at x = 0, "
                 "z = 8.8 through the side walls (pin holes y = +-6; also at x = +-8), pin holes along x in the end "
                 "posts at z = 8.8; a worm (4716) on its axle at x = 0 meshes a 24T (3648) at x = 0, y = 0 (16 mm "
                 "apart); 4 mm walls: every hole is 4 mm long"),
    "57515": dict(kind="technic_brick", title="Technic steering arm 6x2 with tow ball socket (fork + socket)",
                  shape=_boxes(((-24.0, 4.0, -3.6), (3.6, 8.0, 3.6)), ((-24.0, -8.0, -3.6), (3.6, -4.0, 3.6)),
                               ((-44.0, -5.0, -3.6), (-18.0, 5.0, 3.6))),
                  hole_half=2.0,
                  pin_holes=tuple((x, sy * 6.0, 0.0, "y") for x in (0.0, -8.0) for sy in (-1.0, 1.0))
                  + ((-32.0, -2.0, 0.0, "y"), (-32.0, 2.0, 0.0, "y")),
                  axle_holes=((-16.0, -6.0, 0.0, "y", 2.0), (-16.0, 6.0, 0.0, "y", 2.0)),
                  notes="bench frame: a fork of two 4 mm plates (y = +-4..8, an 8 mm gap between them) along -x from "
                  "the origin to x = -24, then the tow ball socket to x = -44 (the ball socket is not modelled); pin "
                  "holes along y through both plates at x = 0 and -8, cross holes at x = -16, a pin hole through the "
                  "socket body at x = -32 (two 4 mm halves)"),
})
# round 4b (2026-10-14): from the LDraw models (hole / axle primitives, bounds); mechanisms in :mod:`.inventory_r4b`
# (its docstring is the model). 87574c01, 12787c01 and 92906 are in the bench frame of inventory_r4b.BENCH_FRAMES.
INVENTORY_GEOMETRY.update({
    "731c04": dict(kind="shock", title="Technic shock absorber 6.5L, hard spring (a sprung slide between its end holes)",
                   shape=Box((-4.0, -4.0, -48.0), (4.0, 4.0, 4.0)),
                   pin_holes=((0.0, 0.0, 0.0, "y"), (0.0, 0.0, -44.0, "y")),
                   notes="axis along local z, drawn extended: a pin hole along y at each end, the cylinder's at the "
                   "origin, the rod's at z = -44 (stroke 8 mm: 36 compressed); 1.0 N/mm, 4.7 N preload (12.7 N at full "
                   "compression); an end turns on its pin (a revolute joint)"),
    "2909c03": dict(kind="shock", title="Technic shock absorber 9.5L, soft spring (a sprung slide between its end holes)",
                    # LDraw profile: the rod end r 4 (z -72..-57), the cylinder with its spring r 5.2 (z -57..-13), the
                    # cap r 4 (z -13..0), the cross-hole lug x -12..-4 (z -8..0); the first box moves with the rod
                    shape=_boxes(((-4.0, -4.0, -72.0), (4.0, 4.0, -57.0)), ((-5.2, -5.2, -57.0), (5.2, 5.2, -13.0)),
                                 ((-4.0, -4.0, -13.0), (4.0, 4.0, 0.0)), ((-12.0, -4.0, -8.0), (-4.0, 4.0, 0.0))),
                    pin_holes=((0.0, 0.0, -68.0, "y"),),
                    axle_holes=((-8.0, 0.0, -4.0, "y"), (0.0, 0.0, -4.0, "z")),
                    notes="axis along local z, drawn extended: the cylinder end has a cross hole along y offset 8 mm "
                    "(x = -8, z = -4) and an axial cross hole in its cap (z = -8..0); the rod end a pin hole along y at "
                    "z = -68 (stroke 16 mm: 52 compressed); 0.4 N/mm, 1.5 N preload"),
    "32494": dict(kind="bush", title="Technic CV joint ball (axle driver; cross-fit on an axle)",
                  shape=Cyl("y", 3.6, 12.0, (0.0, 8.0, 0.0)), width=24.0, hub_mm=(8.0, 20.0),
                  notes="the ball (r 2.4 with two driving pegs) at the origin, a tube with a cross hole y = 8..20 (the "
                  "axle goes in from +y); seat it in a 92906 cup (design 'cv_joints')"),
    "92906": dict(kind="axle", title="Technic CV joint cup with integral axle (turns in a hole, takes a wheel / gear)",
                  shape=_boxes(((-6.0, -2.2, -2.2), (6.0, 2.2, 2.2)), ((-16.4, -4.0, -4.0), (-6.0, 4.0, 4.0))),
                  axle_half_len=6.0,
                  notes="bench frame: its integral axle along local x, x = -6..6 (an axle: hinge it in a hole, "
                  "cross-fit a wheel / gear on it), the cup at x = -16.4..-6 with the ball seat at x = -14"),
    "62821b": dict(kind="differential", title="Technic differential (one-piece, 28T bevel ring)",
                   shape=Cyl("y", 14.4, 12.0), teeth=28, width=24.0,
                   pin_holes=((0.0, -8.0, 0.0, "y"), (0.0, 8.0, 0.0, "y")),
                   notes="axis along y: case r 10 over y = -11..7, the 28T ring at y = 8..12 (pitch plane y = 10), "
                   "round end holes y = +-(4..12) for the output axles; inside: two 12T output bevels and 1-2 "
                   "satellites (design 'differentials')"),
    "65414": dict(kind="connector", title="Technic differential housing, new style (no ring gear: a plain part)",
                  shape=Cyl("y", 10.5, 11.4, (0.0, 7.4, 0.0)), pin_holes=((0.0, 0.0, 0.0, "y"),),
                  notes="cup along +y (y = -4..18.8, r 10.5), a round pin hole along y in its closed end (y = -4..4); "
                  "its ring (65413) is not owned, so it is not a working differential"),
    "12787c01": dict(kind="motor", system="mech", title="Pullback motor 6 x 5 x 3 (spring motor, not IR controlled)",
                     shape=Box((-43.6, -12.0, -11.6), (6.4, 12.0, 27.6)),
                     pin_holes=tuple((x, sy * 8.0, z, "y") for x, z in ((-40.0, -8.0), (-32.0, -8.0), (-24.0, -8.0),
                                                                        (-16.0, -8.0), (-8.0, -8.0), (0.0, 24.0),
                                                                        (-8.0, 24.0)) for sy in (-1.0, 1.0)),
                     through_bore=12.0, motor_no_load_rpm=INVENTORY_MOTOR_LINES["12787c01"]["no_load_rpm"],
                     motor_stall_torque_nm=INVENTORY_MOTOR_LINES["12787c01"]["stall_torque_nm"],
                     motor_source=INVENTORY_MOTOR_LINES["12787c01"]["source"],
                     notes="bench frame: the cross output bore along y through the whole width at the origin (11.6 mm "
                     "above the bottom, 6.4 mm from the +x end), body x = -43.6..6.4; pin holes along y at x = -40..-8 "
                     "(z = -8) and x = 0 / -8 (z = 24), from both faces; wound before the run, released at t = 0, "
                     "ignores the motor program"),
    "87574c01": dict(kind="motor", title="Power Functions train motor (two integral driven axles)",
                     shape=Box((-43.0, -16.0, -23.6), (43.0, 16.0, 5.6)),
                     male_axles=((-24.0, 0.0, -16.8, "y", 16.0), (24.0, 0.0, -16.8, "y", 16.0)),
                     studs_top=_grid(2, 2), top_z=5.6,
                     motor_no_load_rpm=INVENTORY_MOTOR_LINES["87574c01"]["no_load_rpm"],
                     motor_stall_torque_nm=INVENTORY_MOTOR_LINES["87574c01"]["stall_torque_nm"],
                     motor_source=INVENTORY_MOTOR_LINES["87574c01"]["source"],
                     notes="bench frame: body x = -43..43, y = -16..16; two 32 mm axles along y at x = +-24, z = -16.8 "
                     "(48 mm wheelbase), both driven: cross-fit 55423c01 wheels on their ends (the motor the parent); "
                     "a 2 x 2 pivot plate on top (studs at z = 5.6); IR-controlled PF motor with its own lead"),
})
_GEOMETRY.update(INVENTORY_GEOMETRY)
INVENTORY_IDS = frozenset(INVENTORY_GEOMETRY) | frozenset(
    ("32523", "32316", "32525", "41239", "32140", "32526", "32348", "32271", "6629", "60484", "32056", "32009",
     "64179", "64178"))  # (3743 / 6630 / 87761 racks: in INVENTORY_GEOMETRY)
"""Every part id added with the inventory part set (zero in every earlier part set)."""


def _extra_parts() -> dict[str, BenchPart]:
    """Round-3 parts registered in :mod:`.electronics` (not in the build-task catalog; USDs generated there)."""
    from .electronics import EXTRA_PARTS, usd_path  # noqa: PLC0415

    out = {}
    for pid, g in EXTRA_PARTS.items():
        lo, hi = g["lo"], g["hi"]
        com = tuple((a + b) / 2 for a, b in zip(lo, hi, strict=True))
        kw = dict(
            part_id=pid,
            kind=g["kind"],
            title=g["title"],
            mass_g=g["mass_g"],
            com_mm=com,
            shape=Box(lo, hi),
            studs_top=g.get("studs_top", ()),
            top_z=g.get("top_z", 0.0),
            studs_bottom=g.get("studs_bottom", ()),
            bottom_z=g.get("bottom_z", 0.0),
            width=g.get("width", 0.0),
            usd_path=usd_path(pid),
            has_usd=True,
            notes=g.get("notes", ""),
            pin_holes=g.get("pin_holes", ()),
            motor_no_load_rpm=g.get("motor_no_load_rpm", 0.0),
            motor_stall_torque_nm=g.get("motor_stall_torque_nm", 0.0),
            motor_source=g.get("motor_source", ""),
            sensor=g.get("sensor", ""),
            ports=g.get("ports", 0),
            system=g.get("system", "pf"),
        )
        if g["kind"] == "axle_brick":
            kw["hub_mm"] = (-4.0, 4.0)
        out[pid] = BenchPart(**kw)
    out.update(_liftarm_parts())
    return out


def _liftarm_parts() -> dict[str, BenchPart]:
    """Round-3 liftarms (catalog entries, bench-frame USDs from :mod:`.liftarms`)."""
    import numpy as np  # noqa: PLC0415

    from . import liftarms as la  # noqa: PLC0415

    meta = part_metadata()
    out = {}
    for pid in la.LIFTARMS:
        if pid not in PART_TYPES:
            continue
        com_usd = np.array(meta.get(pid, {}).get("com_mm", (0.0, 0.0, 0.0)), dtype=float)
        com = tuple(float(v) for v in la.M_USD_TO_BENCH @ com_usd)
        out[pid] = BenchPart(
            part_id=pid,
            mass_g=PART_TYPES[pid].mass_g,
            com_mm=com,
            usd_path=la.usd_path(pid),
            has_usd=True,
            **la.geometry(pid),
        )
    return out


@lru_cache(maxsize=1)
def bench_parts() -> dict[str, BenchPart]:
    """All parts with connection geometry (every entry of :data:`ai_pd_tasks.lego.catalog.PART_TYPES`, plus the
    round-3 parts of :mod:`.electronics`)."""
    meta = part_metadata()
    out: dict[str, BenchPart] = dict(_extra_parts())
    for pid, geo in _GEOMETRY.items():
        pt = PART_TYPES[pid]
        m = meta.get(pid, {})
        com = tuple(float(v) for v in m.get("com_mm", (0.0, 0.0, 0.0)))
        if geo.get("teeth") and m.get("teeth") and int(m["teeth"]) != geo["teeth"]:
            raise ValueError(f"part {pid}: teeth mismatch with metadata")
        out[pid] = BenchPart(
            part_id=pid, mass_g=pt.mass_g, com_mm=com, usd_path=pt.usd_path, has_usd=True, **geo
        )
    for pid in INVENTORY_IDS & set(out):
        out[pid] = replace(out[pid], inventory=True)
    import numpy as np  # noqa: PLC0415

    from .inventory_r4 import BENCH_FRAMES, usd_path  # noqa: PLC0415  (round 4a: parts rotated into the bench frame)

    for pid, M in BENCH_FRAMES.items():
        if pid in out:
            com = tuple(float(v) for v in M @ np.asarray(out[pid].com_mm, dtype=float))
            out[pid] = replace(out[pid], usd_path=usd_path(pid), has_usd=True, com_mm=com)
    from .inventory_r4b import BENCH_FRAMES as R4B_FRAMES  # noqa: PLC0415  (round 4b: rotated and shifted)
    from .inventory_r4b import usd_path as r4b_usd_path  # noqa: PLC0415

    for pid, (M, t) in R4B_FRAMES.items():
        if pid in out:
            com = tuple(float(v) for v in M @ np.asarray(out[pid].com_mm, dtype=float) + np.asarray(t))
            out[pid] = replace(out[pid], usd_path=r4b_usd_path(pid), has_usd=True, com_mm=com)
    for pid, kw in PF_ELECTRICS.items():
        if pid in out:
            out[pid] = replace(out[pid], **kw)
    return out


PF_ELECTRICS: dict[str, dict] = {
    # motors: the lead leaves the back of the body (LDraw u9190 cable origin, converted to the bench frame)
    "58120c01": dict(pf_cable=(0.0, 46.4, 9.4)),
    "58121c01": dict(pf_cable=(0.0, 46.4, 9.4)),
    "99499c01": dict(pf_cable=(0.0, 52.0, 0.0)),
    "99498c01": dict(pf_cable=(0.0, 58.0, 18.0)),
    # round 4b: the train motor's lead (LDraw 87574c01: u9190c01 at the -y end of the top, bench frame)
    "87574c01": dict(pf_cable=(24.16, 0.0, -2.2)),
    # IR receiver: outputs A / B are the two 2x2 pads on its low front step (LDraw 58147); its lead leaves the front
    "58123c01": dict(pf_pads=((-8.0, 8.0, 0.0), (8.0, 8.0, 0.0)), pf_cable=(0.0, 16.2, -4.0)),
    # battery boxes: the output pad on top. AAA: LDraw 87513. AA 8881: the user's CAD model (2026-10-14,
    # third_party/cad/59510c01 - Battery_Box.STL): pad x 8..24 mm from the centre, centred across, on the face with the
    # switch knob
    "87513c01": dict(pf_pads=((16.0, 0.0, 0.0),)),
    "59510c01": dict(pf_pads=((16.0, 0.0, 28.0),)),
}
"""PF electrics from the LDraw models (2026-10-14): connector pads (where PF plugs stack) and lead exits."""


def get_part(pid: str) -> BenchPart:
    parts = bench_parts()
    if pid not in parts:
        raise KeyError(f"unknown part '{pid}' (catalog: {sorted(parts)})")
    return parts[pid]


DEFAULT_PART_IDS = frozenset(
    ("3895", "3710", "3701", "3032", "44294", "3705", "94925", "3648", "10928", "32062", "4185", "3713")
    + ("58120c01", "87513c01")
)
"""The original '9686' part set (the parts converted for the build task)."""


@lru_cache(maxsize=1)
def default_inventory() -> dict[str, int]:
    """The default part set given to the designer: the LEGO Education 9686 inventory counts of the original 14 part
    types (:data:`DEFAULT_PART_IDS`). Every other catalog part is listed with 0 (use the ``extended`` part set)."""
    inv = read_inventory()
    out = {}
    for pid, pt in PART_TYPES.items():
        if pid in _GEOMETRY and True:
            out[pid] = inv.get((pid, pt.bricklink_colour), 0) if pid in DEFAULT_PART_IDS else 0
    out.update({pid: 0 for pid in _extra_parts()})  # round-3 parts: open set only
    out.update({pid: 0 for pid in INVENTORY_IDS if pid in bench_parts()})  # inventory additions: inventory set only
    return out


EXTENDED_SUPPLEMENT: dict[str, int] = {
    "3707": 2,  # the kit has one axle 8; a long wide car needs two
    "3703": 14,  # the kit has five 1x16 Technic bricks; the long gap spanner needs fourteen
    "3894": 6,  # 1x6 Technic bricks: the long spanner's skids (kit 2)
    "3649": 4,  # 40T gears: the long spanner's drive (kit 3)
    "32269": 4,  # the kit has one 20T double bevel
    "32498": 2,
    "3482c01": 4,
    "56904c01": 4,
    "32020c01": 4,
    "99499c01": 1,
    "58121c01": 1,
}
"""Supplementary pack of the ``extended`` part set: minimum counts of parts the 9686 kit lacks or has too few of."""


@lru_cache(maxsize=1)
def extended_inventory() -> dict[str, int]:
    """The ``extended`` part set: every converted catalog part at its 9686 kit count (1x6/1x8/1x16 Technic bricks,
    axles 5/6/8, 40T and double-bevel gears, pins included), raised to :data:`EXTENDED_SUPPLEMENT` (rubber-tyred
    wheels, PF L and XL motors, one more axle 8, more 20T double bevels)."""
    inv = read_inventory()
    out = {}
    for pid, pt in PART_TYPES.items():
        if pid in _GEOMETRY and True:
            out[pid] = max(inv.get((pid, pt.bricklink_colour), 0), EXTENDED_SUPPLEMENT.get(pid, 0))
    out.update({pid: 0 for pid in _extra_parts()})  # round-3 parts: open set only
    out.update({pid: 0 for pid in INVENTORY_IDS if pid in bench_parts()})  # inventory additions: inventory set only
    return out


OPEN_SUPPLEMENT: dict[str, int] = {
    "3703": 320,  # 1x16 Technic bricks: metre-scale gap spanners with deep (up to 8-layer) walls
    "3702": 24,
    "3894": 24,
    "3895": 12,
    "3701": 64,
    "3710": 80,
    "3032": 6,
    "3707": 16,
    "3706": 12,
    "32073": 12,
    "44294": 12,
    "3705": 24,
    "32062": 24,
    "3649": 12,
    "3648": 16,
    "94925": 32,
    "10928": 12,
    "32269": 8,
    "3713": 40,
    "2780": 80,
    "4185": 12,
    "3482c01": 12,
    "56904c01": 12,
    "32020c01": 16,
    "58120c01": 4,
    "99499c01": 6,
    "58121c01": 2,
    "87513c01": 2,
}
"""The ``open`` part set: every converted part, in the counts of a hobbyist's BrickLink order (all common, in-production
or widely available parts). Up to four PF motors and two AAA battery boxes (:data:`design.MOTORS_PER_BATTERY`)."""


OPEN_EXTRA: dict[str, int] = {
    "60483": 16,  # liftarm 1x2 with axle + pin hole: locks a beam to a turning axle
    "4716": 4,  # worm gear
    "7229": 16,  # liftarm 1x3 with centre axle hole
    "3737": 4,
    "3708": 4,
    "32524": 8,
    "40490": 8,
    "32278": 8,
    "32064": 24,  # Technic brick 1x2 with axle hole
    "53444c01": 1,  # SPIKE Prime Large Hub (+ battery)
    "54675c01": 4,  # Large Angular Motor
    "54696c01": 4,  # Medium Angular Motor
    "37316c01": 2,  # Distance Sensor
    "37312c01": 2,  # Force Sensor
    "37308c01": 2,  # Color Sensor
}
"""Round-3 additions to the ``open`` set (a SPIKE Prime core set plus add-on motors and sensors)."""


@lru_cache(maxsize=1)
def open_inventory() -> dict[str, int]:
    """The ``open`` part set: :func:`extended_inventory` raised to :data:`OPEN_SUPPLEMENT` (stretched levels: metre
    gaps, long ropes, steep ramps), plus the round-3 parts (:data:`OPEN_EXTRA`)."""
    out = {pid: max(n, OPEN_SUPPLEMENT.get(pid, 0)) for pid, n in extended_inventory().items()}
    out.update(OPEN_EXTRA)
    return out


CAPPED_LIMITS: dict[str, int] = {
    # structural parts: what one builder's box of Technic bricks, plates and beams holds (the open set's 320 1x16s
    # made metre-long laminated spanners the answer to everything)
    "3703": 8, "3702": 8, "3894": 8, "3895": 6, "3701": 16,  # Technic bricks 1x16 / 1x8 / 1x6 / 1x12 / 1x4
    "3710": 20, "3032": 2,  # plates 1x4, 4x6
    "32278": 4, "40490": 4, "32524": 4,  # thick liftarms 1x15 / 1x9 / 1x7
    "60483": 8, "7229": 8, "32064": 8,  # axle locks (liftarm 1x2, liftarm 1x3, brick 1x2 with axle hole)
    "2780": 40,  # friction pins
    "32020c01": 6, "56904c01": 6, "3482c01": 8, "4185": 8,  # wheels
    "3713": 20,  # bushes
}
"""Upper limits of the ``capped`` part set (every part type of the ``open`` set; these structural parts, wheels, pins and
bushes capped). Motors, battery boxes, hub, sensors, gears and axles keep their ``open`` counts."""


@lru_cache(maxsize=1)
def capped_inventory() -> dict[str, int]:
    """The ``capped`` part set: :func:`open_inventory` with :data:`CAPPED_LIMITS` applied (the constrained redo)."""
    return {pid: min(n, CAPPED_LIMITS.get(pid, n)) for pid, n in open_inventory().items()}


SPIKE_IDS = ("53444c01", "54675c01", "54696c01", "37316c01", "37312c01", "37308c01")
"""The SPIKE Prime hub, Large / Medium Angular motors and sensors (excluded from ``capped_pf``)."""


@lru_cache(maxsize=1)
def capped_pf_inventory() -> dict[str, int]:
    """The ``capped_pf`` part set: :func:`capped_inventory` without the SPIKE electronics (:data:`SPIKE_IDS`): Power
    Functions motors (M / L / XL) and PF AAA battery boxes only (final campaign, user constraint)."""
    return {pid: n for pid, n in capped_inventory().items() if pid not in SPIKE_IDS}


FINAL_PF_RAISE: dict[str, int] = {
    # what the final campaign's mechanisms (long beams, booms, hook arms, flippers, pier legs) need beyond the open
    # set: a second box of liftarms / long bricks, axles and pins, a third battery box and two more PF motors
    "32278": 16, "40490": 16, "32524": 16,  # thick liftarms 1x15 / 1x9 / 1x7
    "3702": 48, "3895": 24, "3894": 32,  # Technic bricks 1x8 / 1x12 / 1x6
    "60483": 24, "3032": 12,  # liftarm 1x2 axle lock, plate 4x6
    "2780": 160, "3713": 80,  # pins, bushes
    "3708": 16, "3737": 12, "3707": 24, "44294": 24,  # axles 12 / 10 / 8 / 7
    "4716": 8, "3649": 16, "32270": 8, "32498": 8,  # worms, 40T, 12T bevel, 36T
    "3648": 24,  # 24T gears (a second gear pack): wall L5 stilt walker's 9-roller drive chain + 4 hip worm gears
    "58120c01": 6, "58121c01": 4, "87513c01": 3,  # PF M / XL motors, PF AAA battery boxes
}
"""Raises of the ``final_pf`` part set over the ``open`` counts (real-world amounts: about two Technic boxes)."""


@lru_cache(maxsize=1)
def final_pf_inventory() -> dict[str, int]:
    """The ``final_pf`` part set (final5 campaign default since 2026-10-04): the ``open`` set without the SPIKE
    electronics (:data:`SPIKE_IDS`: hub, angular motors, sensors), every structural part, wheel, pin and bush at its open
    count (no ``CAPPED_LIMITS``), raised to :data:`FINAL_PF_RAISE`. Power Functions only; the one-wheel-motor rule and
    the trigger rules are unchanged (they are campaign constraints, not part-set ones)."""
    out = {pid: n for pid, n in open_inventory().items() if pid not in SPIKE_IDS}
    for pid, n in FINAL_PF_RAISE.items():
        if pid in out:
            out[pid] = max(out[pid], n)
    return out


INVENTORY_DATE = "2026-10-06"
INVENTORY_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", f"inventory_{INVENTORY_DATE}.json")
PURCHASE_JSON = os.path.join(os.path.dirname(INVENTORY_JSON), "inventory_purchase_2026-10-07.json")
# overlays added to the owned counts: the planned purchase, then the benchmark library's +10 Technic bricks 1xN
PURCHASE_JSONS = (PURCHASE_JSON, os.path.join(os.path.dirname(INVENTORY_JSON), "inventory_addition_2026-10-11.json"),
                  os.path.join(os.path.dirname(INVENTORY_JSON), "inventory_addition_2026-10-14.json"))


@lru_cache(maxsize=1)
def owned_inventory() -> dict[str, int]:
    """The user's physical inventory (``data/inventory_2026-10-06.json``, from ``bricklink_inventory.xlsx`` via
    ``scripts/import_inventory.py``): BrickLink id -> pieces (all colours summed), every id, simulated or not."""
    with open(INVENTORY_JSON) as f:
        out = {pid: int(e["count"]) for pid, e in json.load(f)["parts"].items()}
    # 2026-10-14 (user): the inventory is the Excel sheet + the 2026-10-11 addition (10 more of every Technic brick 1xN
    # with holes) + the 2026-10-14 addition (a 2nd IR receiver); the 2026-10-07 planned purchase (more bricks, a 2nd
    # receiver) counts only with LEGO_BENCH_PURCHASE=1
    paths = PURCHASE_JSONS if os.environ.get("LEGO_BENCH_PURCHASE", "0") == "1" else PURCHASE_JSONS[1:]
    for path in paths if os.environ.get("LEGO_BENCH_NO_PURCHASE", "0") != "1" else ():
        if os.path.exists(path):  # planned purchase (Technic bricks, a 2nd IR receiver), the +10 Technic bricks 1xN
            with open(path) as f:
                for pid, n in json.load(f)["parts"].items():
                    out[pid] = out.get(pid, 0) + int(n)
    return out


def bom_items(pid: str) -> tuple[str, ...]:
    """Inventory items a simulated part uses (``catalog.BOM_COMPONENTS``: 4185t = 4185 + 2815)."""
    from ai_pd_tasks.lego.catalog import BOM_COMPONENTS  # noqa: PLC0415

    return BOM_COMPONENTS.get(pid, (pid,))


@lru_cache(maxsize=1)
def inventory_part_set() -> dict[str, int]:
    """The ``inventory`` part set: exactly the owned counts (:func:`owned_inventory`, summed over colours) of every
    part type the simulator has, and 0 for every other simulated part. Composite simulated parts (4185t) are listed
    with the count of their scarcest item; the validator counts their items (``bom_items``) against the inventory."""
    owned = owned_inventory()
    out = {}
    for pid in bench_parts():
        items = bom_items(pid)
        out[pid] = min(owned.get(i, 0) for i in items)
    for pid, n in owned.items():  # items only simulated inside a composite (2815 tyre): budget accounting only
        out.setdefault(pid, 0)
        if pid not in bench_parts():
            out[pid] = n
    out["87574c01"] = 0  # (sim v13, user 2026-10-27) the PF train motor is not offered (design.WITHDRAWN_PARTS)
    return out


PART_SETS = {"9686": default_inventory, "extended": extended_inventory, "open": open_inventory, "capped": capped_inventory,
             "capped_pf": capped_pf_inventory, "final_pf": final_pf_inventory, "inventory": inventory_part_set}
"""Named part sets (a design's ``part_set`` field, ``Budget.for_part_set``, ``run_lego_bench.py --part_set``)."""


def part_set_inventory(name: str) -> dict[str, int]:
    if name not in PART_SETS:
        raise KeyError(f"unknown part set '{name}' (one of {sorted(PART_SETS)})")
    return dict(PART_SETS[name]())


def pin_hole_frames(bp: BenchPart) -> list[tuple[tuple[float, float, float], str]]:
    """Every pin hole of a part (middle of the 8 mm hole, local axis 'x' / 'y'): Technic brick holes and the motors'
    pin holes."""
    out = [((hx, 0.0, hz), "y") for hx, hz in bp.holes] if bp.kind == "technic_brick" else []
    out += [((x, y, z), a) for x, y, z, a in bp.pin_holes]
    return out


CANDIDATE_PARTS: dict[str, str] = {
    # 9686 inventory parts without USD assets yet, most useful for the benchmark (id -> why)
    "4519": "Technic axle 3 (x3): stub axles, idlers",
    "3711": "Technic chain link (x40): chain drive to the front axle (4WD at any wheelbase), winch cable (skipped: "
    "needs a chain/sprocket model and ~0.1 g jointed links)",
    "32012": "Technic reel 2.5x2 (x1): winch drum for rope climbing / hoisting",
    "3736": "Technic pulley wheel 5D (x2): larger rope pulleys",
    "4265c": "Technic bush 1/2 (x28): axial spacers",
    "2780": "Technic pin with friction (x37): liftarm structures",
    "32525": "Technic beam 11 (x4): liftarm frames",
    "4716/32905": "Worm gear (x1): self-locking winch, 24:1 reductions",
}
"""Parts in the inventory that would need ``scripts/convert_lego_parts.py`` (and a ``catalog.PART_TYPES`` entry)."""
