# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""The LEGO car: exact assembled pose of every part and the connection graph.

Car frame: x forward, y left, z up, ground at z = 0. Lengths are derived from the LDraw geometry
(1 stud = 8 mm, brick body 9.6 mm, plate 3.2 mm, Technic hole centre 5.6 mm above a brick's underside,
wedge-belt wheel radius 12 mm). Part poses are the pose of each part's LDraw origin (top-centre of the body
for bricks/plates, centre for axles/gears/wheels, output axis for the motor).

Drivetrain (all parts from the inventory)::

    M-motor --axle 2--> 8T pinion  =(3:1)=>  24T on jackshaft (axle 4)  --16T =(1:1)=> 16T on rear axle (axle 7)

The jackshaft is carried 16 mm above the rear axle by 1x4 Technic bricks standing on two 1x4 plates on each
rail ("1 brick + 2 plates = 2 studs" spacing). The motor sits on the left gear-frame brick; its axis is 4 mm
behind the jackshaft (stud-grid constraint), giving 16.5 mm centre distance for the 8T/24T pair (nominal
16 mm: 0.5 mm backlash).

Motor data (Brick Experiment Channel, *Characteristics of LEGO parts*, 2023): PF M-motor 400 rpm no-load,
0.15 N m stall. Axle deformation 0.5 N m, 8T gear slip 0.4-0.6 N m: stall torque at the pinion (0.15 N m)
and at the rear axle (0.45 N m) stay below both.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

MM = 0.001

WHEEL_RADIUS = 12.0 * MM
AXLE_Z = WHEEL_RADIUS  # rail holes carry the wheel axles
RAIL_TOP = AXLE_Z + 4.0 * MM  # hole centre is 4 mm below the rail's top face
PLATE_H = 3.2 * MM
BRICK_H = 9.6 * MM
RAIL_Y = 12.0 * MM
REAR_X = -32.0 * MM
FRONT_X = 32.0 * MM
JACK_Z = AXLE_Z + 16.0 * MM
MOTOR_X = -28.0 * MM
MOTOR_Z = JACK_Z + 16.0 * MM
MOTOR_OUTPUT_Y = -8.0 * MM
WHEEL_Y = 19.4 * MM  # 1.4-1.5 mm off the rails and the deck edge: a wheel leans ~3 deg in the grip (rim +0.9 mm)
FRONT_WHEEL_Y = 20.4 * MM  # front wheels 1 mm further out on the 7L axle: the 32 mm battery box overhangs them; at
# 19.4 its sides rubbed their inner faces on the way down, turned the wheels and drove the car off its jig
BUSH_Y = 24.25 * MM
AXIAL_CLEARANCE = 0.1 * MM
"""Axial gap left between rotating faces and their neighbours (gears, wheels, bushes): real LEGO parts do not sit
perfectly flush, and zero-gap faces rub under solver penetration."""

# PF M-motor (BEC measurements) and the resulting joint drive
MOTOR_NO_LOAD_RPM = 400.0
MOTOR_STALL_TORQUE = 0.15  # N m
GEAR_RATIO_TOTAL = 3.0  # 8T -> 24T, then 16T -> 16T
WHEEL_TYRE_FRICTION = 0.30
"""Hard-ABS wedge-belt wheel on a laminate table (BEC wheel tests: 0.23 for hard plastic up to 1.38 for rubber)."""
STUD_PULL_OFF_N_PER_STUD = 1.0
"""Measured stud pull-off (clutch) force [N per stud] (about 1 N/stud; 3-5 N for a whole brick)."""
STUD_SHEAR_N_PER_STUD = float(__import__("os").environ.get("LEGO_STUD_SHEAR_N", "10.0"))
"""Break force used for stud joints [N per stud]. PhysX breaks a joint on the magnitude of its constraint force in
any direction; stud joints in this car are loaded mainly in shear/moment, which interlocking studs resist far
better than pull-off, so the break force is set from shear capacity (pull-off is not modelled separately)."""
STUD_CLUTCH_N_PER_STUD = STUD_SHEAR_N_PER_STUD


@dataclass(frozen=True)
class PartInstance:
    name: str
    part: str  # BrickLink id (key into catalog.PART_TYPES)
    pos: tuple[float, float, float]  # assembled position of the part origin [m], car frame
    yaw_deg: float = 0.0  # rotation about z
    arm: str = "left"  # which arm installs it
    spin_deg: float = 0.0  # rotation about the part's own y axis (gears: tooth phase for meshing)


@dataclass(frozen=True)
class Connection:
    child: str
    parent: str
    kind: str  # "stud" (fixed, press-fit) | "cross" (fixed, cross-axle fit) | "hinge" (axle in round hole) | "motor"
    studs: int = 0
    axis: str = "Y"  # hinge / motor axis in the parent frame


@dataclass(frozen=True)
class GearMesh:
    gear_a: str
    gear_b: str
    teeth_a: int
    teeth_b: int
    shaft_a: str
    shaft_b: str


def _p(x_mm: float, y_mm: float, z_m: float) -> tuple[float, float, float]:
    return (x_mm * MM, y_mm * MM, z_m)


PARTS: list[PartInstance] = [
    # chassis
    PartInstance("rail_l", "3895", _p(0, 12, RAIL_TOP), arm="left"),
    PartInstance("rail_r", "3895", _p(0, -12, RAIL_TOP), arm="right"),
    PartInstance("deck", "3032", _p(24, 0, RAIL_TOP + PLATE_H), arm="right"),
    # gear frame: 2 plates + Technic brick per side over the rear axle
    PartInstance("plate1_l", "3710", _p(-32, 12, RAIL_TOP + PLATE_H), arm="left"),
    PartInstance("plate1_r", "3710", _p(-32, -12, RAIL_TOP + PLATE_H), arm="right"),
    PartInstance("plate2_l", "3710", _p(-32, 12, RAIL_TOP + 2 * PLATE_H), arm="left"),
    PartInstance("plate2_r", "3710", _p(-32, -12, RAIL_TOP + 2 * PLATE_H), arm="right"),
    PartInstance("frame_l", "3701", _p(-32, 12, RAIL_TOP + 2 * PLATE_H + BRICK_H), arm="left"),
    PartInstance("frame_r", "3701", _p(-32, -12, RAIL_TOP + 2 * PLATE_H + BRICK_H), arm="right"),
    # rear axle with its 16T gear and wheels (held by their cross-axle friction fit; no bushes -- see README_lego)
    PartInstance("rear_axle", "44294", _p(-32, 0, AXLE_Z), yaw_deg=90.0, arm="left"),
    PartInstance("rear_gear16", "94925", _p(-32, 3.9, AXLE_Z), arm="right"),
    PartInstance("wheel_rl", "4185", _p(-32, WHEEL_Y / MM, AXLE_Z), arm="left"),
    PartInstance("wheel_rr", "4185", _p(-32, -WHEEL_Y / MM, AXLE_Z), arm="right"),
    # jackshaft with 24T and 16T
    PartInstance("jackshaft", "3705", _p(-32, 0, JACK_Z), yaw_deg=90.0, arm="right"),
    PartInstance("jack_gear24", "3648", _p(-32, -4.05, JACK_Z), arm="left"),
    PartInstance("jack_gear16", "94925", _p(-32, 3.9, JACK_Z), arm="left"),
    # front axle and wheels
    PartInstance("front_axle", "44294", _p(32, 0, AXLE_Z), yaw_deg=90.0, arm="right"),
    PartInstance("wheel_fl", "4185", _p(32, FRONT_WHEEL_Y / MM, AXLE_Z), arm="left"),
    PartInstance("wheel_fr", "4185", _p(32, -FRONT_WHEEL_Y / MM, AXLE_Z), arm="right"),
    # power
    # the M-motor's output is at its LDraw origin end (local y = 0; the body runs along +y): turned 180 deg so the
    # output faces the car's centre line and the body runs outboard over the right frame
    PartInstance("motor", "58120c01", _p(-28, -8, MOTOR_Z), yaw_deg=180.0, arm="right"),
    PartInstance("motor_shaft", "32062", _p(-28, -8, MOTOR_Z), yaw_deg=90.0, arm="right"),
    PartInstance("pinion8", "10928", _p(-28, -3.85, MOTOR_Z), arm="right"),
    PartInstance("battery", "87513c01", _p(24, 0, RAIL_TOP + PLATE_H + 38.4 * MM), arm="left"),
    # the motor cable's 2x2 PF plug on the battery box's socket (the box's rear 16 mm, 24 mm behind its centre),
    # cable towards the motor; origin = the plug's top, 6.4 mm above the box top
    PartInstance("motor_plug", "58124c01", _p(0, 0, RAIL_TOP + PLATE_H + 38.4 * MM + 6.4 * MM), yaw_deg=180.0,
                 arm="right"),
]
PART_BY_NAME = {p.name: p for p in PARTS}

CONNECTIONS: list[Connection] = [
    Connection("deck", "rail_l", "stud", studs=6),
    Connection("deck", "rail_r", "stud", studs=6),
    Connection("plate1_l", "rail_l", "stud", studs=4),
    Connection("plate1_r", "rail_r", "stud", studs=4),
    Connection("plate2_l", "plate1_l", "stud", studs=4),
    Connection("plate2_r", "plate1_r", "stud", studs=4),
    Connection("frame_l", "plate2_l", "stud", studs=4),
    Connection("frame_r", "plate2_r", "stud", studs=4),
    Connection("battery", "deck", "stud", studs=24),
    Connection("motor_plug", "battery", "stud", studs=4),
    Connection("motor", "frame_r", "stud", studs=3),
    Connection("rear_axle", "rail_l", "hinge"),
    Connection("front_axle", "rail_l", "hinge"),
    Connection("jackshaft", "frame_l", "hinge"),
    Connection("rear_gear16", "rear_axle", "cross"),
    Connection("wheel_rl", "rear_axle", "cross"),
    Connection("wheel_rr", "rear_axle", "cross"),
    Connection("jack_gear24", "jackshaft", "cross"),
    Connection("jack_gear16", "jackshaft", "cross"),
    Connection("wheel_fl", "front_axle", "cross"),
    Connection("wheel_fr", "front_axle", "cross"),
    Connection("motor_shaft", "motor", "motor"),
    Connection("pinion8", "motor_shaft", "cross"),
]

GEAR_MESHES: list[GearMesh] = [
    GearMesh("pinion8", "jack_gear24", 8, 24, "motor_shaft", "jackshaft"),
    GearMesh("jack_gear16", "rear_gear16", 16, 16, "jackshaft", "rear_axle"),
]

# rigid clusters after assembly (parts welded by stud / cross joints)
CHASSIS_PARTS = ["rail_l", "rail_r", "deck", "plate1_l", "plate1_r", "plate2_l", "plate2_r", "frame_l", "frame_r",
                 "battery", "motor"]


def bill_of_materials() -> dict[str, int]:
    from .catalog import SUBPART_OF  # noqa: PLC0415

    bom: dict[str, int] = {}
    for p in PARTS:
        if p.part in SUBPART_OF:  # e.g. the motor's cable plug: part of the motor's inventory item
            continue
        bom[p.part] = bom.get(p.part, 0) + 1
    return bom


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def part_quat_xyzw(inst: PartInstance) -> tuple[float, float, float, float]:
    """Orientation (x, y, z, w) of a part in the car frame: yaw about z, then spin about the part's y axis."""
    hs = math.radians(inst.spin_deg) * 0.5
    return _qmul(yaw_quat_xyzw(inst.yaw_deg), (0.0, math.sin(hs), 0.0, math.cos(hs)))


def gear_spins(tooth_phase_deg: dict[str, float]) -> dict[str, float]:
    """Spin of every gear so that meshing teeth interleave.

    ``tooth_phase_deg`` maps a gear part id to the angle atan2(z, x) of a tooth centre in its own frame
    (from the converter metadata). A spin beta about +y moves that tooth to world angle ``phase - beta``.
    For each mesh the driving gear points a tooth at the driven gear, which presents a gap (half a pitch off).
    """
    spins: dict[str, float] = {}
    for m in GEAR_MESHES:
        a, b = PART_BY_NAME[m.gear_a], PART_BY_NAME[m.gear_b]
        theta = math.degrees(math.atan2(b.pos[2] - a.pos[2], b.pos[0] - a.pos[0]))
        if m.gear_a not in spins:
            spins[m.gear_a] = tooth_phase_deg[a.part] - theta
        # where gear a's tooth actually points, then put a gap of gear b there
        pa = tooth_phase_deg[a.part] - spins[m.gear_a]
        spins[m.gear_b] = tooth_phase_deg[b.part] - (pa + 180.0 + 180.0 / m.teeth_b)
    return spins


def apply_gear_spins(tooth_phase_deg: dict[str, float]) -> None:
    """Rewrite :data:`PARTS` with meshing spins (call once, before spawning)."""
    spins = gear_spins(tooth_phase_deg)
    for i, p in enumerate(PARTS):
        if p.name in spins:
            PARTS[i] = PartInstance(p.name, p.part, p.pos, p.yaw_deg, p.arm, spins[p.name] % 360.0)
    PART_BY_NAME.update({p.name: p for p in PARTS})


def load_gear_phases() -> dict[str, float]:
    import json
    import os

    from .catalog import PART_USD_DIR

    meta = json.load(open(os.path.join(PART_USD_DIR, "parts", "metadata.json")))
    return {pid: m["tooth_phase_deg"] for pid, m in meta.items() if "tooth_phase_deg" in m}


def yaw_quat_xyzw(yaw_deg: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) of a rotation about z (Isaac Lab 3.0 convention)."""
    h = math.radians(yaw_deg) * 0.5
    return (0.0, 0.0, math.sin(h), math.cos(h))


# An axle turning in a Technic hole has a little friction; a frictionless hinge lets a wheel on a free axle
# swing like a pendulum (its centre of mass is a fraction of a millimetre off the axle), which destabilised the
# jointed car in the simulator. Viscous damping of the hinge: 1e-5 N m s/rad costs < 0.3 % of the motor's
# torque-speed slope (0.15 N m / 42 rad/s) and damps a wheel's swing within milliseconds.
HINGE_DAMPING_NM_S_PER_RAD = float(__import__("os").environ.get("LEGO_HINGE_DAMPING", "1.0e-5"))


def motor_drive_params() -> dict[str, float]:
    """USD angular drive reproducing the DC motor line tau = tau_stall * (1 - w / w0) (USD units: degrees)."""
    w0_deg = MOTOR_NO_LOAD_RPM * 360.0 / 60.0
    return {
        "target_velocity_deg_s": w0_deg,
        "damping_nm_s_per_deg": MOTOR_STALL_TORQUE / w0_deg,
        "max_force_nm": MOTOR_STALL_TORQUE,
        "expected_wheel_speed_rad_s": MOTOR_NO_LOAD_RPM * 2 * math.pi / 60.0 / GEAR_RATIO_TOTAL,
        "expected_car_speed_m_s": MOTOR_NO_LOAD_RPM * 2 * math.pi / 60.0 / GEAR_RATIO_TOTAL * WHEEL_RADIUS,
    }
