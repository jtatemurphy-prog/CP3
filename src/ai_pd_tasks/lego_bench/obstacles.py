# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Parametric obstacles with difficulty levels, vehicle placement and the success / failure judge (pure Python).

Env frame: x = direction of travel, z up, metres. Every course starts on a surface at z = 0 unless stated
(the rope hangs higher). Geometry is static boxes / cylinders / capsules; the seesaw plank and the tug sled are
dynamic bodies. Each obstacle has a start line, a goal region, failure conditions and a time limit:

* success   -- the reference points (all part origins for ``goal.ref == "rear"``, the front-most part origin for
               ``"front"``, the towed load for ``"load"``) are inside the goal region, the vehicle is intact;
* fell      -- the chassis origin drops below ``fail_z``;
* flipped   -- the vehicle's up axis tilts more than 75 deg for 0.5 s;
* left_lane -- the chassis leaves ``|y| <= lane_half_width`` (with a ``centreline``: its distance from the centreline);
* broke     -- a part moved more than 15 mm from its design place relative to the chassis (a joint broke);
* stuck     -- progress improved less than 5 mm in ``stuck_window_s``;
* timeout   -- ``time_limit_s`` of driving elapsed.

``generate(kind, level, ladder)`` builds one obstacle (``ladder="redo"``: the harder geometric ladder; ``"final"``: the
5-level LEGO-buildable final ladder; ``"final6"``: the previous 6-level final ladder); ``OBSTACLE_KINDS`` lists them with their level tables.

A curved course (``turns``) sets ``Obstacle.centreline``: progress, the goal and the lane are then measured along it
(arc length, distance from it) instead of along x and |y|.
"""

from __future__ import annotations

import dataclasses
import math
import os
import random
from dataclasses import asdict, dataclass, field

import numpy as np

from .design import SURFACES, Design, place, quat_xyzw, rot_matrix  # noqa: F401  (SURFACES re-exported)

INF = 1.0e9
PAYLOAD_DROP_Z = 0.004  # a payload lower than this (m above z = 0) has fallen off the vehicle


@dataclass
class Shape:
    name: str
    kind: str  # box | cylinder | capsule
    size: tuple  # box: (sx, sy, sz); cylinder / capsule: (radius, length) along ``axis``
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # xyzw
    axis: str = "X"
    surface: str = "laminate"
    color: tuple[float, float, float] = (0.55, 0.45, 0.34)
    dynamic: bool = False
    mass_kg: float = 0.0


@dataclass
class JointSpec:
    name: str
    kind: str  # revolute (to the world) | distance (tow line)
    body0: str  # "" = world, a shape name, or "vehicle:tow" (design chassis at its tow point)
    body1: str
    anchor: tuple[float, float, float]  # world anchor on body0 [m]
    anchor1: tuple[float, float, float] = (0.0, 0.0, 0.0)  # world anchor on body1 [m]
    axis: str = "Y"
    lower_deg: float = -INF
    upper_deg: float = INF
    max_dist: float = 0.0


@dataclass
class StartSpec:
    front_x: float | None = 0.0  # the vehicle's front-most extent is placed here ...
    rear_x: float | None = None  # ... or its rear-most extent here
    y: float = 0.0  # track centre (or rope wheel line) goes here
    surface_z: float = 0.0  # surface height at x = surface_x (the vehicle's wheels are set on this line)
    surface_x: float = 0.0
    pitch_deg: float = 0.0  # surface slope at the start (nose up > 0)
    align: str = "track"  # track | rope | rope_track (on the ground wheels, rope wheel line at y)


@dataclass
class GoalSpec:
    x_min: float
    x_max: float = INF
    y_half: float = 0.2
    z_min: float = -0.02
    z_max: float = 1.0
    ref: str = "rear"  # rear (all part origins) | front (front-most part origin) | load (the towed sled)
    #   | half (the midpoint of the part origins' x extent: half the vehicle past x_min; y / z checked on all parts)
    hold_s: float = 0.0  # > 0: the vehicle must stay inside the goal region, at rest (< 5 mm of travel), this long


@dataclass
class Obstacle:
    kind: str
    level: int
    params: dict
    description: str
    shapes: list[Shape]
    start: StartSpec
    goal: GoalSpec
    fail_z: float
    time_limit_s: float
    floor_z: float = 0.0
    floor_surface: str = "laminate"
    joints: list[JointSpec] = field(default_factory=list)
    lane_half_width: float = 0.2
    stuck_window_s: float = 4.0
    stuck_min_progress_m: float = 0.005
    tow_line_m: float = 0.0  # tug: tow line length; the sled is placed behind the vehicle by :func:`finalize`
    sled_gap_m: float = 0.0
    constraints: dict = field(default_factory=dict)  # tier constraints on the design (``check_constraints``)
    payload: dict = field(default_factory=dict)  # {mass_g, size_mm, surface}: a loose load carried on design.cargo_mm
    # optional course centreline, a polyline [(x, y), ...] [m]. When set, progress is the arc length along it (from its
    # first point) instead of x, ``lane_half_width`` bounds the chassis' distance from it instead of |y|, and the goal's
    # x_min / x_max are arc lengths and y_half a distance from it (see ``Judge``). Empty: the straight x-course.
    centreline: list = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)


# ----------------------------------------------------------------------------------------------------------------
# geometry helpers
# ----------------------------------------------------------------------------------------------------------------

GREY = (0.45, 0.46, 0.50)
WOOD = (0.62, 0.48, 0.30)
BLUE = (0.20, 0.35, 0.65)
RED = (0.70, 0.20, 0.15)
WAX = (0.85, 0.85, 0.92)
RUBBER = (0.12, 0.12, 0.12)
SURF_COLOR = {
    "laminate": (0.55, 0.45, 0.34),
    "wood": WOOD,
    "rubber_mat": RUBBER,
    "waxed": WAX,
    "cord": (0.9, 0.85, 0.6),
}


def pitch_quat(angle_deg: float) -> tuple[float, float, float, float]:
    """Rotation raising the +x end by ``angle_deg`` (rotation about y by -angle)."""
    h = -math.radians(angle_deg) / 2
    return (0.0, math.sin(h), 0.0, math.cos(h))


def box(name, x0, x1, y_half, z_top, z_bot, surface="laminate", color=None, y0=0.0) -> Shape:
    return Shape(
        name,
        "box",
        (x1 - x0, 2 * y_half, z_top - z_bot),
        (0.5 * (x0 + x1), y0, 0.5 * (z_top + z_bot)),
        surface=surface,
        color=color or SURF_COLOR.get(surface, GREY),
    )


def slab(name, a, b, y_half, t=0.02, surface="laminate", color=None) -> Shape:
    """A plank whose top face runs from point a = (x, z) to b = (x, z)."""
    (xa, za), (xb, zb) = a, b
    ang = math.atan2(zb - za, xb - xa)
    length = math.hypot(xb - xa, zb - za)
    nx, nz = -math.sin(ang), math.cos(ang)
    c = (0.5 * (xa + xb) - nx * t / 2, 0.0, 0.5 * (za + zb) - nz * t / 2)
    return Shape(
        name,
        "box",
        (length, 2 * y_half, t),
        c,
        pitch_quat(math.degrees(ang)),
        surface=surface,
        color=color or SURF_COLOR.get(surface, GREY),
    )


def profile_slabs(prefix, pts, y_half, surface="laminate", t=0.02) -> list[Shape]:
    return [slab(f"{prefix}{i}", pts[i], pts[i + 1], y_half, t, surface) for i in range(len(pts) - 1)]


# ----------------------------------------------------------------------------------------------------------------
# obstacle generators
# ----------------------------------------------------------------------------------------------------------------

COURSE_HALF_WIDTH = 0.15

RAMP_LEVELS = [  # slope deg, surface, rise m
    (5, "laminate", 0.030),
    (8, "laminate", 0.030),
    (11, "laminate", 0.030),
    (14, "laminate", 0.030),
    (17, "laminate", 0.030),
    (20, "laminate", 0.030),
    (10, "waxed", 0.030),
    (25, "rubber_mat", 0.030),
    # steep climbs on the rubber mat, with a taller rise (the whole car has to be on the slope)
    (30, "rubber_mat", 0.060),
    (35, "rubber_mat", 0.080),
    (40, "rubber_mat", 0.100),
    (45, "rubber_mat", 0.150),
    (50, "rubber_mat", 0.150),
    (55, "rubber_mat", 0.200),
    (60, "rubber_mat", 0.200),
]
"""Levels 1-8 as before (30 mm rise). Levels 9-15 climb 30-60 deg on the rubber mat to plateaus 60-200 mm up. The
best measured tyre (32020c01, mu 1.38 on plywood) has mu ~1.66 on the rubber mat: atan(1.66) = 59 deg, so level 15
(60 deg, tan = 1.73) is beyond what friction alone can climb."""


RAMP_C_LEVELS = [(45, "rubber_mat", 0.150), (50, "rubber_mat", 0.150), (55, "rubber_mat", 0.200)]
RAMP_C_CONSTRAINTS = {"max_mass_g": 250.0, "max_drive_motors": 1}
"""Tier C: the steep climbs with at most 250 g and one drive motor (the 55 deg winner weighs 318 g): the 62 mm tyres
alone are 132 g, so lighter wheels, a lighter power source and a low centre of mass are needed."""


def ramp_transition(slope: float) -> tuple[int, float]:
    """(number of intermediate segments, segment length [m]) of the foot and crest transitions: one 40 mm half-slope
    segment up to 25 deg (as before); steeper ramps step in 15 deg or smaller increments of 25 mm segments."""
    if slope <= 25:
        return 1, 0.040
    return int(math.ceil(slope / 15.0)) - 1, 0.025


def ramp(level: int, spec=None) -> Obstacle:
    """Climb a ramp to a plateau. Slope grows with the level; level 7 is waxed (low friction), levels 8-15 steep
    rubber-mat ramps (25-60 deg, plateaus 30-200 mm up). Transitions at the foot and the crest (break-over): one
    40 mm half-slope segment up to 25 deg, from 30 deg on 25 mm segments stepping up by <= 15 deg."""
    tier = spec is None and level > len(RAMP_LEVELS)
    slope, surface, rise = spec or (RAMP_C_LEVELS[level - len(RAMP_LEVELS) - 1] if tier else RAMP_LEVELS[level - 1])
    k, lt = ramp_transition(slope)
    angles = [slope * i / (k + 1) for i in range(1, k + 1)]
    rise_t = sum(lt * math.sin(math.radians(a)) for a in angles)
    main = (rise - 2 * rise_t) / math.sin(math.radians(slope))
    assert main > 0.02, (level, main)
    pts = [(0.0, 0.0)]
    for a in angles:
        pts.append((pts[-1][0] + lt * math.cos(math.radians(a)), pts[-1][1] + lt * math.sin(math.radians(a))))
    pts.append((pts[-1][0] + main * math.cos(math.radians(slope)), pts[-1][1] + main * math.sin(math.radians(slope))))
    for a in reversed(angles):
        pts.append((pts[-1][0] + lt * math.cos(math.radians(a)), pts[-1][1] + lt * math.sin(math.radians(a))))
    pts[-1] = (pts[-1][0], rise)
    x_top = pts[-1][0]
    shapes = profile_slabs("ramp", pts, COURSE_HALF_WIDTH, surface)
    shapes.append(box("plateau", x_top, x_top + 0.35, COURSE_HALF_WIDTH, rise, 0.0, surface))
    return Obstacle(
        "ramp",
        level,
        {
            "slope_deg": slope,
            "surface": surface,
            "rise_m": rise,
            "ramp_length_m": round(x_top, 4),
            "slope_length_m": round(main, 4),
            **({"tier": "C", **RAMP_C_CONSTRAINTS} if tier else {}),
        },
        f"ramp {slope} deg ({surface}) up to a plateau {rise * 1000:.0f} mm high"
        + ("; tier C: <= 250 g, one drive motor" if tier else ""),
        shapes,
        StartSpec(front_x=-0.02),
        GoalSpec(x_min=x_top + 0.01, x_max=x_top + 0.35, z_min=rise - 0.004),
        fail_z=-0.05,
        time_limit_s=30.0 if spec is None else round(30.0 + 50.0 * max(0.0, rise - 0.1), 1),
        lane_half_width=COURSE_HALF_WIDTH,
        constraints=dict(RAMP_C_CONSTRAINTS) if tier else {},
    )


GAP_LEVELS = [5, 8, 11, 14, 18, 24, 32, 45, 60, 80, 100, 130, 160, 200, 250, 320, 400, 500, 630, 800, 1000]  # mm
"""Levels 1-7 fit wheel-dip crossings; 8-13 (45-160 mm) need a chassis long enough to span the gap, or momentum;
14-21 (200 mm to 1 m) need a vehicle more than twice as long as the gap (supports on both sides of the centre of mass
while the gap is under any part of it)."""
GAP_DEEP_FROM_MM = 45  # from here on the gap is a 150 mm deep chasm (a car that drops in cannot climb out)
GAP_CHASM_FROM_MM = 200  # from here on a 400 mm deep chasm


GAP_C_LEVELS = [100, 130, 160, 200]  # mm
GAP_C_CONSTRAINTS = {"max_length_mm": 250.0, "max_mass_g": 800.0}
"""Tier C: the vehicle must be at most 250 mm long at the start (folded), 800 g: the skid spanners (304-544 mm) are
ruled out; booms / skids must deploy on the way."""


def gap(level: int, spec=None) -> Obstacle:
    """Cross a gap between two platforms (50 mm drop; 150 mm from 45 mm wide, 400 mm from 200 mm wide). Width grows
    with the level, up to 1 m. Levels after the 21 base levels are tier C (compact vehicle)."""
    tier = spec is None and level > len(GAP_LEVELS)
    plat_final = None
    if isinstance(spec, (tuple, list)):  # final ladder: (gap mm, platform length m)
        spec, plat_final = spec
    gmm = spec or (GAP_C_LEVELS[level - len(GAP_LEVELS) - 1] if tier else GAP_LEVELS[level - 1])
    w = gmm * 1e-3
    drop = 0.40 if gmm >= GAP_CHASM_FROM_MM else 0.15 if gmm >= GAP_DEEP_FROM_MM else 0.05
    # platforms long enough for a car that spans the gap (wheelbase > gap, skids reaching a gap beyond each axle:
    # ~3x the gap long); 0.40 m (goal) / 0.45 m (start) up to 100 mm gaps, as before
    plat = max(0.40, 3.5 * w + 0.10) if plat_final is None else plat_final
    time_limit = 20.0 if gmm <= 160 else round(20.0 + 25.0 * w, 1)
    shapes = [
        box("platform_start", -(plat + 0.05), 0.0, COURSE_HALF_WIDTH, 0.0, -drop),
        box("platform_goal", w, w + plat, COURSE_HALF_WIDTH, 0.0, -drop),
    ]
    return Obstacle(
        "gap",
        level,
        {"gap_mm": gmm, "drop_m": drop, **({"tier": "C", **GAP_C_CONSTRAINTS} if tier else {}),
         **({"platform_m": plat, "platform_rule": "max(0.45, 1.6 x gap + 0.15) m"} if plat_final is not None else {})},
        f"gap {gmm} mm wide between two platforms, {drop * 1000:.0f} mm drop"
        + ("; tier C: vehicle <= 250 mm long at the start, <= 800 g" if tier else ""),
        shapes,
        StartSpec(front_x=-0.03),
        GoalSpec(x_min=w + 0.005, x_max=w + plat, z_min=-0.004),
        fail_z=-0.02,
        time_limit_s=30.0 if tier else time_limit,
        floor_z=-drop,
        lane_half_width=COURSE_HALF_WIDTH,
        constraints=dict(GAP_C_CONSTRAINTS) if tier else {},
    )


BRIDGE_LEVELS = [(120, 0.25), (80, 0.30), (60, 0.30), (50, 0.35), (45, 0.35), (40, 0.40), (34, 0.40), (28, 0.40), (24, 0.45)]


def bridge(level: int, spec=None) -> Obstacle:
    """Drive along a plank over a 100 mm drop. The plank narrows (and lengthens) with the level."""
    wmm, length = spec or BRIDGE_LEVELS[level - 1]
    w = wmm * 1e-3
    drop = 0.10
    shapes = [
        box("platform_start", -0.45, 0.0, COURSE_HALF_WIDTH, 0.0, -drop),
        box("plank", 0.0, length, w / 2, 0.0, -0.012, "wood"),
        box("platform_goal", length, length + 0.40, COURSE_HALF_WIDTH, 0.0, -drop),
    ]
    return Obstacle(
        "bridge",
        level,
        {"plank_width_mm": wmm, "plank_length_m": length, "drop_m": drop},
        f"plank {wmm} mm wide, {length * 1000:.0f} mm long over a {drop * 1000:.0f} mm drop",
        shapes,
        StartSpec(front_x=-0.03),
        GoalSpec(x_min=length + 0.005, x_max=length + 0.40, z_min=-0.004),
        fail_z=-0.03,
        time_limit_s=max(30.0, round(15.0 + 25.0 * length, 1)),
        floor_z=-drop,
        lane_half_width=COURSE_HALF_WIDTH,
    )


ROPE_LEVELS = [  # free span m, incline deg, sag mm, rope radius mm
    (0.30, 0.0, 0.0, 2.0),
    (0.45, 0.0, 4.0, 2.0),
    (0.60, 0.0, 8.0, 2.0),
    (0.80, 2.0, 12.0, 2.0),
    (1.00, 3.0, 18.0, 1.5),
    (1.25, 4.0, 25.0, 1.5),
    (1.50, 5.0, 35.0, 1.5),
    (1.75, 6.0, 45.0, 1.25),
    (2.00, 8.0, 60.0, 1.0),
]
ROPE_HEIGHT = 0.0  # the start deck top
ROPE_PLATFORM_LEN = 0.70  # start and goal platforms (solid)
ROPE_EDGE_TOP = 0.080  # rope top above the deck at the platform edges (it runs over a saddle post there)
ROPE_ANCHOR_BACK = 0.40  # the rope is anchored lying on the deck 0.40 m behind the start edge / beyond the goal edge
ROPE_START_FRONT_X = -0.41  # the vehicle's front starts just behind the start anchor
ROPE_CHASM = 0.40  # depth of the chasm under the rope (below the start deck)
ROPE_SADDLE_LEN = 0.010  # saddle posts under the rope at the edges: 10 mm long, as wide as the rope


def rope_profile(span, incline_deg, sag, n=None, z0=None):
    """Rope axis points (x, z) of the free span: straight incline minus a parabolic sag (max ``sag`` at mid-span),
    starting at height ``z0`` (default: a 4 mm rope over the start saddle)."""
    n = n or max(2, int(math.ceil(span / 0.025)))
    z0 = ROPE_HEIGHT + ROPE_EDGE_TOP - 0.002 if z0 is None else z0
    t = math.tan(math.radians(incline_deg))
    return [(span * i / n, z0 + span * i / n * t - 4 * sag * (i / n) * (1 - i / n)) for i in range(n + 1)]


def rope_path(level: int, spec=None) -> tuple[list[tuple[float, float]], int, int]:
    """Rope axis polyline (x, z) of a level: lying on the start deck from its anchor, rising to the start saddle, the
    free span, the goal saddle, down to its anchor lying on the goal deck. Returns (points, index of the start edge,
    index of the goal edge)."""
    span, inc, sag_mm, r_mm, _goal_len, goal_anchor = _rope_spec(level, spec)
    r = r_mm * 1e-3
    z_edge = ROPE_HEIGHT + ROPE_EDGE_TOP - r
    pts = [(-ROPE_ANCHOR_BACK, ROPE_HEIGHT + r)]
    i0 = len(pts)
    pts += rope_profile(span, inc, sag_mm * 1e-3, z0=z_edge)
    i1 = len(pts) - 1
    goal_top = pts[i1][1] + r - ROPE_EDGE_TOP
    pts.append((span + goal_anchor, goal_top + r))
    return pts, i0, i1


def rope(level: int, spec=None) -> Obstacle:
    """Cross a chasm on a rope, from a solid start platform to a solid goal platform.

    Both platforms are solid (no slot). A braided nylon cord (a chain of static capsules) is anchored lying on the
    start deck 0.40 m behind the edge, rises in a straight line to a saddle post at the edge (rope top 80 mm above
    the deck), spans the chasm (incline and parabolic sag), runs over a saddle post at the goal edge (80 mm above the
    goal deck) and down to its anchor lying on the goal deck 0.40 m beyond the edge. The vehicle starts standing on
    the start deck with its front 10 mm behind the rope's anchor (the rope lies on the deck in front of it, along the
    course centre line): it has to drive onto the rope, be carried by it up to the edge and across the chasm, set down
    on the goal deck and end with every part over the goal platform. Span, incline and sag grow and the rope gets
    thinner with the level."""
    span, inc, sag_mm, r_mm, goal_len, _anchor = _rope_spec(level, spec)
    tier = spec is None and _rope_tier(level)
    r = r_mm * 1e-3
    pts, i0, i1 = rope_path(level, spec)
    shapes = []
    for i in range(len(pts) - 1):
        (xa, za), (xb, zb) = pts[i], pts[i + 1]
        ang = math.degrees(math.atan2(zb - za, xb - xa))
        prefix = "rope_s" if i < i0 else "rope" if i < i1 else "rope_g"
        shapes.append(
            Shape(
                f"{prefix}{i}",
                "capsule",
                (r, math.hypot(xb - xa, zb - za)),
                (0.5 * (xa + xb), 0.0, 0.5 * (za + zb)),
                pitch_quat(ang),
                axis="X",
                surface="cord",
                color=SURF_COLOR["cord"],
            )
        )
    goal_top = pts[i1][1] + r - ROPE_EDGE_TOP
    L = ROPE_PLATFORM_LEN
    shapes += [
        box("platform_start", -L, 0.0, COURSE_HALF_WIDTH, 0.0, -ROPE_CHASM, "wood"),
        box("platform_goal", span, span + goal_len, COURSE_HALF_WIDTH, goal_top, -ROPE_CHASM, "wood"),
        # saddle posts under the rope at the edges (as wide as the rope)
        box("saddle_start", -ROPE_SADDLE_LEN, 0.0, r, ROPE_EDGE_TOP - 2 * r, 0.0, "wood", GREY),
        box("saddle_goal", span, span + ROPE_SADDLE_LEN, r, goal_top + ROPE_EDGE_TOP - 2 * r, goal_top, "wood", GREY),
    ]
    lowest = min(p[1] for p in pts[i0 : i1 + 1])
    params = {
        "span_m": span,
        "incline_deg": inc,
        "sag_mm": sag_mm,
        "rope_radius_mm": r_mm,
        "edge_rope_top_mm": ROPE_EDGE_TOP * 1000,
        "anchor_back_m": ROPE_ANCHOR_BACK,
        "goal_deck_z_m": round(goal_top, 4),
        "goal_platform_m": goal_len,
    }
    desc = (
        f"rope {span * 1000:.0f} mm span between two solid platforms, {inc} deg incline, {sag_mm} mm sag, "
        f"{2 * r_mm} mm diameter"
    )
    if tier:
        params.update(tier="C", **ROPE_C_CONSTRAINTS)
        desc += f"; tier C: stop on the {goal_len * 1000:.0f} mm goal platform (it ends in a drop)"
    return Obstacle(
        "rope",
        level,
        params,
        desc,
        shapes,
        StartSpec(front_x=ROPE_START_FRONT_X, y=0.0, surface_z=ROPE_HEIGHT, align="rope_track"),
        GoalSpec(
            x_min=span + 0.005,
            x_max=span + goal_len,
            z_min=lowest - 0.15,
            ref="rear",
            y_half=0.1,
            hold_s=TIER_HOLD_S if tier else 0.0,
            # the redo L5 goal deck is 1.17 m up: keep the region's ceiling 0.5 m above the deck (1 m default below)
            z_max=max(1.0, goal_top + 0.5),
        ),
        fail_z=lowest - 0.15,
        time_limit_s=round(30.0 + 12.0 * span, 1) + (10.0 if tier else 0.0),
        floor_z=-ROPE_CHASM,
        lane_half_width=0.1,
        constraints=dict(ROPE_C_CONSTRAINTS) if tier else {},
    )


# ------------------------------------------------------------------------------------------------ tier C (rope)
ROPE_C_LEVELS = [  # span m, incline deg, sag mm, rope radius mm
    (0.60, 0.0, 8.0, 2.0),
    (1.00, 8.0, 18.0, 1.5),
    (1.00, 20.0, 10.0, 1.5),
]
ROPE_C_GOAL = (0.40, 0.25)  # goal platform length, rope anchor beyond the goal edge [m]
ROPE_C_CONSTRAINTS = {"max_mass_g": 450.0, "sensors_only": True}
TIER_HOLD_S = 1.0


def _rope_tier(level: int) -> bool:
    return level > len(ROPE_LEVELS)


def _rope_spec(level: int, spec=None):
    if spec is not None:
        return (*spec, ROPE_PLATFORM_LEN, ROPE_ANCHOR_BACK)
    if _rope_tier(level):
        return (*ROPE_C_LEVELS[level - len(ROPE_LEVELS) - 1], *ROPE_C_GOAL)
    return (*ROPE_LEVELS[level - 1], ROPE_PLATFORM_LEN, ROPE_ANCHOR_BACK)


STEP_LEVELS = [(0.5, 1), (1.0, 1), (1.5, 1), (2.0, 1), (3.0, 1), (4.0, 1), (2.0, 3), (3.0, 3)]  # mm, count
STEP_LEVELS += [(6.0, 1), (9.0, 1), (12.0, 1), (16.0, 1), (20.0, 1), (6.0, 3), (10.0, 3)]  # curbs and stairs for big wheels


STEP_C_LEVELS = [(16.0, 1), (24.0, 1), (32.0, 1), (16.0, 3)]  # mm, count (80 mm treads)
STEP_C_CONSTRAINTS = {"max_wheel_diameter_mm": 25.0}
"""Tier C: wheels at most 25 mm across (24 mm tyres, wedge-belt wheels): a plain wheel climbs only a few mm, the curbs
are 16-32 mm: legs, a lifting arm or a jack is needed."""


def steps(level: int, spec=None) -> Obstacle:
    """Climb a curb (levels 1-6, height grows) or a flight of three stairs (levels 7-8, 60 mm treads). Tier C (after
    the base levels): 16-32 mm curbs and 3 x 16 mm stairs (80 mm treads) with small wheels only."""
    tier = spec is None and level > len(STEP_LEVELS)
    h_mm, n = spec or (STEP_C_LEVELS[level - len(STEP_LEVELS) - 1] if tier else STEP_LEVELS[level - 1])
    h = h_mm * 1e-3
    # redo stairs: treads 2.4 x the riser (>= 80 mm), so a climber can stand on each step
    tread = round(max(0.08, 2.4 * h), 3) if spec is not None else (0.08 if tier else 0.06)
    shapes = []
    for i in range(n):
        x0 = i * tread
        x1 = x0 + tread if i < n - 1 else x0 + 0.35
        shapes.append(box(f"step{i}", x0, x1, COURSE_HALF_WIDTH, (i + 1) * h, 0.0))
    top = n * h
    x_goal = (n - 1) * tread + 0.01
    return Obstacle(
        "steps",
        level,
        {"step_mm": h_mm, "count": n, "tread_m": tread, **({"tier": "C", **STEP_C_CONSTRAINTS} if tier else {})},
        f"{n} step(s) of {h_mm} mm" + (f", {tread * 1000:.0f} mm treads" if n > 1 else ""),
        shapes,
        StartSpec(front_x=-0.03),
        GoalSpec(x_min=x_goal, x_max=x_goal + 0.34, z_min=top - 0.003),
        fail_z=-0.05,
        time_limit_s=40.0 if tier else 30.0 if spec is None else round(30.0 + 10.0 * n + 150.0 * h, 1),
        lane_half_width=COURSE_HALF_WIDTH,
        constraints=dict(STEP_C_CONSTRAINTS) if tier else {},
    )


BUMP_LEVELS = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0]  # mm


def bumps(level: int, seed: int = 7, spec=None) -> Obstacle:
    """Rough terrain: 12 round rods (radius 10 mm, partly sunk into the floor) across a 0.6 m course, random
    positions, lengths and lateral offsets (one wheel line at a time rocks the vehicle). Height grows with level.
    Redo ladder: rods up to 68 mm high; their radius grows with the height (0.6 x, >= 10 mm) so they stay on the
    floor (boulders rather than floating bars)."""
    n_rods, final = 12, isinstance(spec, (tuple, list))  # final ladder: spec = (max height mm, rod count)
    if final:
        spec, n_rods = spec
    hmm = spec if spec is not None else BUMP_LEVELS[level - 1]
    hmax = hmm * 1e-3
    rng = random.Random(seed * 100 + level + (2000 if final else 1000 if spec is not None else 0))
    R = max(0.010, 0.6 * hmax)
    shapes = []
    xs = sorted(rng.uniform(0.02, 0.58) for _ in range(n_rods))
    for i, x in enumerate(xs):
        hgt = hmax * rng.uniform(0.6, 1.0)
        length = rng.uniform(0.04, 0.22)
        y = rng.uniform(-0.08, 0.08)
        shapes.append(
            Shape(f"bump{i}", "cylinder", (R, length), (x, y, hgt - R), axis="Y", surface="laminate", color=GREY)
        )
    return Obstacle(
        "bumps",
        level,
        {"max_bump_mm": hmm, "rods": len(xs), "seed": seed, **({"rod_radius_mm": round(R * 1000, 1)} if spec else {})},
        f"rough terrain: {len(xs)} rods up to {hmm} mm high over 0.6 m",
        shapes,
        StartSpec(front_x=-0.01),
        GoalSpec(x_min=0.62, x_max=1.2, z_min=-0.004),
        fail_z=-0.05,
        time_limit_s=30.0,
        lane_half_width=COURSE_HALF_WIDTH,
    )


TUNNEL_LEVELS = [100, 75, 66, 62, 58, 50, 46]  # mm clear height


TUNNEL_C_LEVELS = [75, 66, 58]  # mm clear height, with the payload
TUNNEL_C_PAYLOAD = {"mass_g": 50.0, "size_mm": (32.0, 32.0, 6.0), "surface": "steel"}
"""Tier C: carry a loose 50 g steel plate (32 x 32 x 6 mm, 7.8 g/cm3) through the tunnel: it is set on the design's
``cargo_mm`` point at the start and must still be on the vehicle, inside the goal region, at the end."""


def tunnel(level: int, spec=None) -> Obstacle:
    """Drive through a 200 mm long tunnel whose ceiling is lowered with the level (height limit). Tier C (after the
    base levels): the same with a loose payload to carry."""
    tier = spec is None and level > len(TUNNEL_LEVELS)
    hmm, length = spec or (TUNNEL_C_LEVELS[level - len(TUNNEL_LEVELS) - 1] if tier else TUNNEL_LEVELS[level - 1], 0.20)
    h = hmm * 1e-3
    x0, half = 0.05, 0.10
    x1 = x0 + length
    shapes = [
        box("ceiling", x0, x1, half + 0.01, h + 0.01, h, "laminate", BLUE),
        box("wall_l", x0, x1, 0.005, h, 0.0, "laminate", BLUE, y0=half + 0.005),
        box("wall_r", x0, x1, 0.005, h, 0.0, "laminate", BLUE, y0=-half - 0.005),
    ]
    return Obstacle(
        "tunnel",
        level,
        {"clear_height_mm": hmm, "length_m": x1 - x0, "width_m": 2 * half,
         **({"tier": "C", "payload_g": TUNNEL_C_PAYLOAD["mass_g"]} if tier else {})},  # fmt: skip
        f"tunnel {hmm} mm high, {2 * half * 1000:.0f} mm wide, {(x1 - x0) * 1000:.0f} mm long"
        + ("; tier C: carry a loose 50 g steel plate through it" if tier else ""),
        shapes,
        StartSpec(front_x=0.0),
        GoalSpec(x_min=x1 + 0.01, x_max=x1 + 0.6, z_min=-0.004),
        fail_z=-0.05,
        time_limit_s=round(16.0 + 20.0 * length, 1),
        lane_half_width=COURSE_HALF_WIDTH,
        payload=dict(TUNNEL_C_PAYLOAD) if tier else {},
        constraints={"payload": True} if tier else {},
    )


TUG_LEVELS = [20, 50, 80, 120, 160, 220, 350, 500, 750, 1000]  # g


TUG_C_LEVELS = [(1000, 300.0), (1000, 250.0), (1000, 200.0)]  # sled g, vehicle mass cap g
"""Tier C: a 1 kg sled with a light vehicle (sled / vehicle mass 3.3, 4, 5). Pure traction pulls at most
mu_tyre / mu_sled = 1.33 / 0.325 = 4.1 times the vehicle's mass on laminate."""


def tug(level: int, spec=None) -> Obstacle:
    """Tow an ABS sled (40 x 40 x 15 mm, mass grows with the level) 0.4 m on laminate with a 50 mm tow line
    (distance joint) hooked to the design's tow point (``tow_point_mm``, default: rear centre of the chassis).
    Tier C (after the base levels): a 1 kg sled with a vehicle mass cap."""
    tier = spec is None and level > len(TUG_LEVELS)
    sled_g, cap = TUG_C_LEVELS[level - len(TUG_LEVELS) - 1] if tier else (spec or TUG_LEVELS[level - 1], None)
    m = sled_g * 1e-3
    sled = Shape(
        "sled", "box", (0.04, 0.04, 0.015), (0.0, 0.0, 0.0075), surface="abs", color=RED, dynamic=True, mass_kg=m
    )
    ob = Obstacle(
        "tug",
        level,
        {"sled_g": sled_g, "tow_line_m": 0.05, "distance_m": 0.40, **({"tier": "C", "max_mass_g": cap} if tier else {})},
        f"tow a {sled_g} g sled 0.4 m (sliding ABS on laminate)" + (f"; tier C: vehicle <= {cap:.0f} g" if tier else ""),
        [sled],
        StartSpec(front_x=0.0),
        GoalSpec(x_min=0.40, x_max=INF, ref="load"),
        fail_z=-0.05,
        time_limit_s=40.0,
        lane_half_width=COURSE_HALF_WIDTH,
        tow_line_m=0.05,
        sled_gap_m=0.04,
        constraints={"max_mass_g": cap} if tier else {},
    )
    ob.joints.append(
        JointSpec("tow_line", "distance", "vehicle:tow", "sled", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), max_dist=0.05)
    )
    return ob


SEESAW_LEVELS = [(10, 100), (15, 80), (20, 60), (25, 50), (30, 45), (35, 40)]  # pivot rise mm, plank width mm


def seesaw(level: int, spec=None) -> Obstacle:
    """Drive over a seesaw: a 300 mm wood plank (30 g) pivoting on a revolute joint between two platforms 30 mm up.
    It starts tilted with its near end flush with the start platform; it tips when the vehicle passes the pivot and
    its far end lands flush with the goal platform. Pivot rise (tilt) grows and the plank narrows with the level."""
    rise_mm, wmm = spec or SEESAW_LEVELS[level - 1]
    L, t, H = 0.30, 0.004, 0.030
    rise = rise_mm * 1e-3
    tilt = math.degrees(math.asin(rise / (L / 2)))
    xc = L / 2 + 0.002
    zc_top = H + rise  # plank top at the pivot
    # plank centre: top face through the pivot line, tilted nose-up by `tilt`
    n = (-math.sin(math.radians(tilt)), math.cos(math.radians(tilt)))
    centre = (xc - n[0] * t / 2, 0.0, zc_top - n[1] * t / 2)
    plank = Shape(
        "plank",
        "box",
        (L, wmm * 1e-3, t),
        centre,
        pitch_quat(tilt),
        surface="wood",
        color=WOOD,
        dynamic=True,
        mass_kg=0.030,
    )
    # end stops: ledges under the near end (start pose) and the far end (tipped pose), plank bottom resting on them
    tan_t, t_eff = math.tan(math.radians(tilt)), t / math.cos(math.radians(tilt))
    x_near, x_far = xc - (L / 2) * math.cos(math.radians(tilt)), xc + (L / 2) * math.cos(math.radians(tilt))
    z_near = H - t_eff + (x_near + 0.0015 - x_near) * tan_t
    z_far = H - t_eff + (x_far - (x_far - 0.0015)) * tan_t
    half_w = wmm * 1e-3 / 2
    # fulcrum under the pivot. Base ladder: top t + 2 mm below the plank top at the pivot (unchanged). At steep tilts
    # that overlaps the tilted plank's underside (0.7 / 2.1 / 4.7 mm at redo L4 / L5 / L6), and PhysX kicks the plank
    # at t = 0. Redo ladder: 1 mm below the plank's underside over the fulcrum's +-6 mm at the start tilt.
    fulcrum_top = zc_top - t - 0.002
    if spec is not None:
        fulcrum_top = centre[2] - (t / 2) / math.cos(math.radians(tilt)) - 0.006 * math.tan(math.radians(tilt)) - 0.001
    shapes = [
        box("platform_start", -0.40, 0.0, COURSE_HALF_WIDTH, H, 0.0),
        box("platform_goal", L + 0.004, L + 0.40, COURSE_HALF_WIDTH, H, 0.0),
        box("ledge_near", x_near + 0.0015, x_near + 0.012, half_w, z_near, 0.0, "wood", GREY),
        box("ledge_far", x_far - 0.012, x_far - 0.0015, half_w, z_far, 0.0, "wood", GREY),
        box("fulcrum", xc - 0.006, xc + 0.006, 0.03, fulcrum_top, 0.0, "wood", GREY),
        plank,
    ]
    ob = Obstacle(
        "seesaw",
        level,
        {
            "pivot_rise_mm": rise_mm,
            "tilt_deg": round(tilt, 2),
            "plank_width_mm": wmm,
            "plank_length_m": L,
            "plank_g": 30,
        },
        f"seesaw plank {wmm} mm wide tilted {tilt:.1f} deg, pivot {rise_mm} mm above the platforms",
        shapes,
        StartSpec(front_x=-0.005, surface_z=H),
        GoalSpec(x_min=L + 0.01, x_max=L + 0.40, z_min=H - 0.004),
        fail_z=-0.02,
        time_limit_s=30.0,
        lane_half_width=COURSE_HALF_WIDTH,
    )
    ob.joints.append(JointSpec("pivot", "revolute", "", "plank", centre, axis="Y"))
    return ob


SPEED_LEVELS = [15.0, 10.0, 7.0, 5.0, 3.5, 2.5, 2.0]  # s


SPEED_C_LEVELS = [0.30, 0.20, 0.15]  # stop-zone length [m], starting at x = 1 m
SPEED_C_CONSTRAINTS = {"sensors_only": True}
"""Tier C: drive 1 m and come to rest (1 s) with the whole vehicle inside a blue floor zone; no position triggers
(the program must use sensors or time)."""


def speed(level: int, spec=None) -> Obstacle:
    """Time trial: the whole vehicle past the 1 m line within the level's time limit, staying in a 300 mm lane. Tier C
    (after the base levels): stop inside a blue zone 1.0 .. 1.0 + L m."""
    if spec is None and level > len(SPEED_LEVELS):
        zl = SPEED_C_LEVELS[level - len(SPEED_LEVELS) - 1]
        return Obstacle(
            "speed",
            level,
            {"distance_m": 1.0, "zone_m": zl, "tier": "C", **SPEED_C_CONSTRAINTS},
            f"tier C: drive 1 m and stop inside a {zl * 1000:.0f} mm blue zone (1 s at rest)",
            [box("stop_zone", 1.0, 1.0 + zl, COURSE_HALF_WIDTH, 0.0005, -0.001, "laminate", BLUE)],
            StartSpec(front_x=0.0),
            GoalSpec(x_min=1.0, x_max=1.0 + zl, z_min=-0.004, hold_s=TIER_HOLD_S),
            fail_z=-0.05,
            time_limit_s=20.0,
            lane_half_width=COURSE_HALF_WIDTH,
            stuck_window_s=3.0,
            constraints=dict(SPEED_C_CONSTRAINTS),
        )
    tl = spec or SPEED_LEVELS[level - 1]
    return Obstacle(
        "speed",
        level,
        {"distance_m": 1.0, "time_limit_s": tl},
        f"drive 1 m within {tl} s",
        [box("finish_line", 1.0, 1.004, COURSE_HALF_WIDTH, 0.0002, -0.001, "laminate", RED)],
        StartSpec(front_x=0.0),
        GoalSpec(x_min=1.0, x_max=INF, z_min=-0.004),
        fail_z=-0.05,
        time_limit_s=tl,
        lane_half_width=COURSE_HALF_WIDTH,
        stuck_window_s=3.0,
    )


TURN_LEVELS = [  # turns, turn angle deg, centreline radius mm, lane width mm (wall to wall), straight between turns mm
    (1, 45, 400, 300, 150),
    (2, 45, 300, 280, 150),
    (2, 90, 225, 260, 150),
    (3, 90, 170, 240, 120),
    (4, 90, 125, 220, 100),
    (4, 135, 115, 210, 100),
    (4, 180, 110, 200, 100),
    (6, 180, 100, 180, 100),
    # levels 9-16: the same alternating turns on a raised plywood deck with drop-off edges and no walls (6th field)
    (1, 45, 400, 360, 150, "deck"),
    (2, 45, 300, 320, 150, "deck"),
    (2, 90, 250, 300, 150, "deck"),
    (3, 90, 210, 280, 150, "deck"),
    (4, 90, 180, 260, 150, "deck"),
    (4, 135, 170, 240, 150, "deck"),
    (4, 180, 160, 220, 150, "deck"),
    (6, 180, 150, 200, 120, "deck"),
    # levels 17-24: the same turns on the flat floor, the lane only painted (border lines, colour marks; 6th field)
    (1, 45, 400, 400, 150, "paint"),
    (2, 45, 300, 360, 150, "paint"),
    (2, 90, 250, 320, 150, "paint"),
    (3, 90, 210, 290, 150, "paint"),
    (4, 90, 180, 260, 150, "paint"),
    (4, 135, 170, 240, 150, "paint"),
    (4, 180, 160, 220, 150, "paint"),
    (6, 180, 150, 200, 120, "paint"),
]
"""Consecutive sharp turns in a walled channel, alternating left / right (the first turn is to the left). Total turning
45, 90, 180, 270, 360, 540, 720, 1080 deg (x1.5-2 per level); centreline radius 400 -> 100 mm (below a car's length
from L5 on: the 175 mm car_l4wd); lane 300 -> 180 mm wide. L7-L8 are serpentines of 180 deg hairpins: neighbouring
lanes share a 20 mm divider wall whose end is a 10 mm-radius post. Every rotation axis of the design format is
horizontal (yaw x spin about the part's y axis), so a steering pivot about a vertical axis cannot be built: a turn
comes from the walls (a sliding bumper and a yaw-free chassis) or from free motors (liftarm whegs on each side
skid-steering, a paddle on a longitudinal axle). A pivot-in-place vehicle whose diagonal is under the lane width can
drive every level (the inscribed circle of the channel is the lane width everywhere).

Levels 9-16 (``"deck"``) repeat the progression on an 80 mm high plywood deck with no walls: its edges drop off to the
floor, so a wall follower has nothing to follow and the vehicle must steer itself. Total turning 45 -> 1080 deg, radius
400 -> 150 mm (about a car's length), deck 360 -> 200 mm wide; serpentine lanes keep a >= 100 mm gap (a car that cuts
across is judged ``left_lane``). Colour marks are painted across the deck (LEGO colours, for the SPIKE colour sensor):
green 30 mm before a left turn, red before a right turn, yellow where a turn ends.

Levels 17-24 (``"paint"``) repeat it on the flat floor with no walls and no edges: the lane is only painted (white
border lines, the same colour marks, a blue finish band), all flush with the floor. Leaving the lane fails: every part
origin must stay within w/2 + 10 mm of the centreline. Lane 400 -> 200 mm wide."""
TURN_PAINT_T = 0.001  # painted boards: 1 mm thick, tops flush at z = 0 (the room floor is lowered to -1 mm)
TURN_PAINT_BORDER = 0.018  # white border lines (18 mm tape) inside the lane edge [m]
TURN_PAINT_APRON = 0.04  # plain board beyond the border lines [m] (its outer 1 mm step is 40 mm outside the lane)
TURN_PAINT_PART_MARGIN = 0.010  # every part origin within w/2 + 10 mm of the centreline
TURN_DECK_H = 0.080  # deck top above the floor [m]
TURN_DECK_GAP = 0.10  # minimum gap between deck parts that are far apart along the course [m]
TURN_DECK_STEP_DEG = 6.0  # deck boxes per arc: one per 6 deg
TURN_MARK_LEN = 0.015  # colour marks: 15 mm along the course, the full deck width, 0.2 mm proud of the deck
TURN_MARK_COLORS = {k: tuple(round(c / 255.0, 4) for c in rgb) for k, rgb in
                    (("green", (0, 133, 43)), ("red", (180, 0, 0)), ("yellow", (250, 200, 10)))}  # fmt: skip
TURN_WALL_H = 0.050  # wall height [m]: above a 62 mm tyre's axle (a wheel cannot climb it)
TURN_WALL_T = 0.010  # wall thickness [m] (plywood strips)
TURN_BACK = 0.80  # the channel starts (back wall) 0.80 m behind the start line
TURN_LEAD = 0.15  # straight from the start line to the first turn [m]
TURN_FINAL = 0.40  # straight after the last turn, ending at an end wall [m]; the goal is its last 0.38 m
TURN_ARC_STEP_DEG = 3.0  # centreline polyline resolution on the arcs
TURN_WALL_STEP_DEG = 6.0  # outer-wall boxes per arc: one per 6 deg


def _yaw_quat(h: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(h / 2), math.cos(h / 2))


def turns_centreline(n: int, ang_deg: float, R: float, d: float) -> tuple[list, list, float, float]:
    """Centreline of a turns course: (polyline points (x, y), segments, arc length of the start line, arc length of
    the end of the last turn). Segments: ("S", x0, y0, heading, length) and ("A", cx, cy, h0, signed angle, R)."""
    x, y, h = -TURN_BACK, 0.0, 0.0
    pts, segs = [(x, y)], []
    s, s_last = 0.0, 0.0

    def straight(length):
        nonlocal x, y, s
        segs.append(("S", x, y, h, length))
        x, y = x + length * math.cos(h), y + length * math.sin(h)
        s += length
        pts.append((x, y))

    straight(TURN_BACK + TURN_LEAD)
    for i in range(n):
        sgn = 1.0 if i % 2 == 0 else -1.0
        a = math.radians(ang_deg) * sgn
        cx, cy = x - sgn * R * math.sin(h), y + sgn * R * math.cos(h)
        segs.append(("A", cx, cy, h, a, R))
        k = max(2, int(math.ceil(ang_deg / TURN_ARC_STEP_DEG)))
        for j in range(1, k + 1):
            hj = h + a * j / k
            pts.append((cx + sgn * R * math.sin(hj), cy - sgn * R * math.cos(hj)))
        h += a
        x, y = pts[-1]
        s += R * abs(a)
        s_last = s
        if i < n - 1:
            straight(d)
    straight(TURN_FINAL)
    return [(round(px, 6), round(py, 6)) for px, py in pts], segs, TURN_BACK, s_last


def _footprint_dist(sh: Shape, p) -> float:
    """Horizontal distance from point p = (x, y) to a wall's footprint (box or vertical cylinder)."""
    if sh.kind == "cylinder":
        return max(0.0, math.hypot(p[0] - sh.pos[0], p[1] - sh.pos[1]) - sh.size[0])
    h = 2 * math.atan2(sh.quat[2], sh.quat[3])
    dx, dy = p[0] - sh.pos[0], p[1] - sh.pos[1]
    u, v = dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h)
    return math.hypot(max(0.0, abs(u) - sh.size[0] / 2), max(0.0, abs(v) - sh.size[1] / 2))


def turns(level: int, spec=None) -> Obstacle:
    """Drive a walled channel with consecutive sharp turns (alternating left / right). The floor is the laminate room
    floor; the walls are 50 mm high, 10 mm thick plywood strips (outer side of each turn: boxes every 6 deg; inner
    side: a round post of radius R - w/2). The vehicle starts heading +x with its front on the start line, the channel
    closed by a back wall 0.80 m behind it and an end wall after the last turn. Touching and sliding along the walls is
    allowed; the chassis leaving the lane (climbing a wall) fails. Progress and the goal are measured along the
    centreline (``Obstacle.centreline``: arc length from its first point): every part past the end of the last turn
    (+ 20 mm) and before the end wall."""
    if len(spec or TURN_LEVELS[level - 1]) == 6:
        sp = spec or TURN_LEVELS[level - 1]
        return turns_paint(level, sp) if sp[5] == "paint" else turns_deck(level, sp)
    n, ang, r_mm, w_mm, d_mm = spec or TURN_LEVELS[level - 1]
    R, hw, d, t, H = r_mm * 1e-3, w_mm * 1e-3 / 2, d_mm * 1e-3, TURN_WALL_T, TURN_WALL_H
    assert R - hw >= 0.005, (level, "the inner post needs a radius of >= 5 mm")
    pts, segs, s_start, s_last = turns_centreline(n, ang, R, d)
    wood = SURF_COLOR["wood"]
    shapes: list[Shape] = []
    for side in (1.0, -1.0):  # left, right (relative to the direction of travel)
        tag = "l" if side > 0 else "r"
        for k, sg in enumerate(segs):
            if sg[0] == "S":
                _, x0, y0, h, length = sg
                off = side * (hw + t / 2)
                cx = x0 + 0.5 * length * math.cos(h) - off * math.sin(h)
                cy = y0 + 0.5 * length * math.sin(h) + off * math.cos(h)
                shapes.append(Shape(f"wall_{tag}{k}", "box", (length, t, H), (cx, cy, H / 2), _yaw_quat(h),
                                    surface="wood", color=wood))  # fmt: skip
                continue
            _, cx, cy, h0, a, _r = sg
            sgn = 1.0 if a > 0 else -1.0
            if side == sgn:  # inner side of the turn: a round post filling the inside of the curve
                shapes.append(Shape(f"post_{tag}{k}", "cylinder", (R - hw, H), (cx, cy, H / 2), axis="Z",
                                    surface="wood", color=GREY))  # fmt: skip
                continue
            m = max(1, int(math.ceil(abs(math.degrees(a)) / TURN_WALL_STEP_DEG)))
            da = abs(a) / m
            ro = R + hw  # wall face radius; each box's face is tangent to it at the box middle
            for j in range(m):
                hm = h0 + a * (j + 0.5) / m  # heading at the box middle
                rc = ro + t / 2
                px, py = cx + sgn * rc * math.sin(hm), cy - sgn * rc * math.cos(hm)
                length = 2 * (ro + t) * math.tan(da / 2)  # outer corners of neighbouring boxes meet
                shapes.append(Shape(f"wall_{tag}{k}_{j}", "box", (length, t, H), (px, py, H / 2), _yaw_quat(hm),
                                    surface="wood", color=wood))  # fmt: skip
    # the channel must be clear: every wall at least half a lane from every centreline point (no lane overlaps)
    samples = []
    for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:], strict=True):
        m = max(1, int(math.ceil(math.hypot(xb - xa, yb - ya) / 0.005)))
        samples += [(xa + (xb - xa) * i / m, ya + (yb - ya) * i / m) for i in range(m)]
    samples.append(pts[-1])
    clear = min(_footprint_dist(sh, p) for sh in shapes for p in samples)
    assert clear >= hw - 1e-3, (level, f"walls intrude into the lane: {clear * 1000:.1f} mm < {hw * 1000:.0f} mm")
    # back and end walls
    for name, (xa, ya), (xb, yb) in (("wall_back", pts[0], pts[1]), ("wall_end", pts[-1], pts[-2])):
        h = math.atan2(ya - yb, xa - xb)  # pointing out of the channel
        cx, cy = xa + (t / 2) * math.cos(h), ya + (t / 2) * math.sin(h)
        shapes.append(Shape(name, "box", (t, 2 * (hw + t), H), (cx, cy, H / 2), _yaw_quat(h), surface="wood",
                            color=RED if name == "wall_end" else wood))  # fmt: skip
    s_total = sum(math.hypot(xb - xa, yb - ya) for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:], strict=True))
    goal_s0 = s_last + 0.02
    path = goal_s0 - s_start
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    params = {
        "turns": n,
        "turn_deg": ang,
        "radius_mm": r_mm,
        "lane_width_mm": w_mm,
        "straight_mm": d_mm,
        "total_turn_deg": n * ang,
        "path_m": round(path, 4),
        "wall_mm": H * 1000,
        "extent_m": [round(max(xs) - min(xs), 3), round(max(ys) - min(ys), 3)],
    }
    return Obstacle(
        "turns",
        level,
        params,
        f"{n} consecutive {ang} deg turn(s) (alternating, radius {r_mm} mm) in a {w_mm} mm walled lane, "
        f"{path:.2f} m to the goal",
        shapes,
        StartSpec(front_x=0.0),
        GoalSpec(x_min=round(goal_s0, 6), x_max=round(s_total, 6), y_half=hw + 0.01, z_min=-0.02),
        fail_z=-0.05,
        time_limit_s=round(20.0 + 30.0 * path),
        lane_half_width=hw + 0.01,
        stuck_window_s=6.0,
        centreline=pts,
    )


def turns_deck(level: int, spec) -> Obstacle:
    """Turns levels 9+: the same alternating turns on a raised plywood deck (``TURN_DECK_H`` high, solid down to the
    floor) without walls; its edges drop off. The deck starts 0.80 m behind the start line and ends after the last
    turn's 0.40 m straight at a red end wall (a stop). Straights are single boxes; each arc is a fan of boxes (one per
    6 deg) spanning R - w/2 .. R + w/2 (the edges are true to < 1 mm). Fail: the chassis origin 30 mm below the deck
    top (``fell``) or more than w/2 + 20 mm from the centreline (``left_lane``: off the deck, or cutting across to
    another lane). Colour marks across the deck: green before a left turn, red before a right turn, yellow at the end
    of each turn."""
    n, ang, r_mm, w_mm, d_mm, _ = spec
    R, hw, d, H, t = r_mm * 1e-3, w_mm * 1e-3 / 2, d_mm * 1e-3, TURN_DECK_H, TURN_WALL_T
    assert R - hw >= 0.0, (level, "the deck's inner edge needs a non-negative radius")
    pts, segs, s_start, s_last = turns_centreline(n, ang, R, d)
    wood = SURF_COLOR["wood"]
    shapes: list[Shape] = []
    marks = []  # (name, x, y, heading, colour)
    for k, sg in enumerate(segs):
        if sg[0] == "S":
            _, x0, y0, h, length = sg
            cx, cy = x0 + 0.5 * length * math.cos(h), y0 + 0.5 * length * math.sin(h)
            shapes.append(Shape(f"deck{k}", "box", (length, 2 * hw, H), (cx, cy, H / 2), _yaw_quat(h),
                                surface="wood", color=wood))  # fmt: skip
            continue
        _, cx, cy, h0, a, _r = sg
        sgn = 1.0 if a > 0 else -1.0
        m = max(1, int(math.ceil(abs(math.degrees(a)) / TURN_DECK_STEP_DEG)))
        da = abs(a) / m
        for j in range(m):
            hm = h0 + a * (j + 0.5) / m
            px, py = cx + sgn * R * math.sin(hm), cy - sgn * R * math.cos(hm)
            width = 2 * (R + hw) * math.tan(da / 2)  # tangential: neighbouring boxes meet at the outer edge
            shapes.append(Shape(f"deck{k}_{j}", "box", (width, 2 * hw, H), (px, py, H / 2), _yaw_quat(hm),
                                surface="wood", color=wood))  # fmt: skip
        # marks: before the turn (on the previous straight) and at its end (on the next straight)
        bx, by = cx + sgn * R * math.sin(h0), cy - sgn * R * math.cos(h0)
        back = 0.03 + TURN_MARK_LEN / 2
        marks.append((f"mark_in{k}", bx - back * math.cos(h0), by - back * math.sin(h0), h0,
                      "green" if sgn > 0 else "red"))  # fmt: skip
        h1 = h0 + a
        ex, ey = cx + sgn * R * math.sin(h1), cy - sgn * R * math.cos(h1)
        fwd = TURN_MARK_LEN / 2
        marks.append((f"mark_out{k}", ex + fwd * math.cos(h1), ey + fwd * math.sin(h1), h1, "yellow"))
    for name, mx, my, h, col in marks:
        shapes.append(Shape(name, "box", (TURN_MARK_LEN, 2 * hw, 0.0012), (mx, my, H - 0.0004), _yaw_quat(h),
                            surface="wood", color=TURN_MARK_COLORS[col]))  # fmt: skip
    # deck parts far apart along the course must be >= TURN_DECK_GAP apart (no shortcut across a gap a car spans)
    samples, ss = [], []
    s_acc = 0.0
    for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:], strict=True):
        L = math.hypot(xb - xa, yb - ya)
        m = max(1, int(math.ceil(L / 0.01)))
        for i in range(m):
            samples.append((xa + (xb - xa) * i / m, ya + (yb - ya) * i / m))
            ss.append(s_acc + L * i / m)
        s_acc += L
    P, S = np.array(samples), np.array(ss)
    need = 2 * hw + TURN_DECK_GAP
    far = np.abs(S[:, None] - S[None, :]) >= 2 * need
    dmin = float(np.min(np.where(far, np.linalg.norm(P[:, None] - P[None, :], axis=2), INF)))
    assert dmin >= need - 1e-3, (level, f"deck lanes {dmin * 1000:.0f} mm apart < {need * 1000:.0f} mm")
    # end wall (a stop at the end of the deck)
    (xa, ya), (xb, yb) = pts[-1], pts[-2]
    h = math.atan2(ya - yb, xa - xb)
    shapes.append(Shape("wall_end", "box", (t, 2 * hw, TURN_WALL_H), (xa - (t / 2) * math.cos(h),
                        ya - (t / 2) * math.sin(h), H + TURN_WALL_H / 2), _yaw_quat(h), surface="wood",
                        color=RED))  # fmt: skip
    s_total = s_acc
    goal_s0 = s_last + 0.02
    path = goal_s0 - s_start
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    params = {
        "turns": n,
        "turn_deg": ang,
        "radius_mm": r_mm,
        "lane_width_mm": w_mm,
        "straight_mm": d_mm,
        "total_turn_deg": n * ang,
        "path_m": round(path, 4),
        "deck_mm": H * 1000,
        "marks": True,
        "extent_m": [round(max(xs) - min(xs), 3), round(max(ys) - min(ys), 3)],
    }
    return Obstacle(
        "turns",
        level,
        params,
        f"{n} consecutive {ang} deg turn(s) (alternating, radius {r_mm} mm) on a {w_mm} mm deck {H * 1000:.0f} mm "
        f"up with drop-off edges (no walls), {path:.2f} m to the goal",
        shapes,
        StartSpec(front_x=0.0, surface_z=H),
        GoalSpec(x_min=round(goal_s0, 6), x_max=round(s_total - t, 6), y_half=hw + 0.02, z_min=H - 0.01,
                 z_max=H + 1.0),  # fmt: skip
        fail_z=H - 0.03,
        time_limit_s=round(20.0 + 40.0 * path),
        lane_half_width=hw + 0.02,
        stuck_window_s=6.0,
        centreline=pts,
    )


def _far_lane_clearance(pts, need: float) -> float:
    """Smallest distance between centreline points that are >= 2 x ``need`` apart along the course."""
    samples, ss, s_acc = [], [], 0.0
    for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:], strict=True):
        L = math.hypot(xb - xa, yb - ya)
        m = max(1, int(math.ceil(L / 0.01)))
        for i in range(m):
            samples.append((xa + (xb - xa) * i / m, ya + (yb - ya) * i / m))
            ss.append(s_acc + L * i / m)
        s_acc += L
    P, S = np.array(samples), np.array(ss)
    far = np.abs(S[:, None] - S[None, :]) >= 2 * need
    return float(np.min(np.where(far, np.linalg.norm(P[:, None] - P[None, :], axis=2), INF)))


def turns_paint(level: int, spec) -> Obstacle:
    """Turns levels 17+: the same alternating turns on the flat laminate floor, no walls and no edges: the lane is a
    painted board, 1 mm thick with its top flush with the floor (the room floor plane is lowered by 1 mm under it).
    In cross-section: a 40 mm plain apron, an 18 mm white border line, the plain lane, the other line, the other apron.
    The colour marks (green before a left turn, red before a right turn, yellow after each turn, a blue finish band at
    the end) are pieces of the lane board between the lines, not raised strips, so nothing on the floor stands proud.
    Straights are cut into boxes at the marks; arcs are fans of boxes, one per 6 deg, per band. Fail: any part origin
    more than w/2 + 10 mm from the centreline (``left_lane``)."""
    n, ang, r_mm, w_mm, d_mm, _ = spec
    R, hw, d, T = r_mm * 1e-3, w_mm * 1e-3 / 2, d_mm * 1e-3, TURN_PAINT_T
    B, A = TURN_PAINT_BORDER, TURN_PAINT_APRON
    assert R - hw - A >= 0.0, (level, "the inner apron needs a non-negative radius")
    pts, segs, s_start, s_last = turns_centreline(n, ang, R, d)
    lam = GREY  # the board (grey laminate: white lines stand out; the colour sensor reads it as "none")
    white = (round(244 / 255, 4),) * 3
    blue = (round(30 / 255, 4), round(90 / 255, 4), round(168 / 255, 4))
    bands = [("apron_l", hw, hw + A, lam), ("line_l", hw - B, hw, white), ("lane", -(hw - B), hw - B, lam),
             ("line_r", -hw, -(hw - B), white), ("apron_r", -(hw + A), -hw, lam)]  # fmt: skip
    shapes: list[Shape] = []

    def board(name, cx, cy, h, length, v0, v1, color):
        vc = 0.5 * (v0 + v1)
        shapes.append(Shape(name, "box", (length, v1 - v0, T), (cx - vc * math.sin(h), cy + vc * math.cos(h), -T / 2),
                            _yaw_quat(h), surface="laminate", color=color))  # fmt: skip

    # marks on the straights, as (segment index, start, end along it, colour)
    marks: dict[int, list] = {}
    for k, sg in enumerate(segs):
        if sg[0] != "A":
            continue
        marks.setdefault(k - 1, []).append((segs[k - 1][4] - 0.03 - TURN_MARK_LEN, segs[k - 1][4] - 0.03,
                                            "green" if sg[4] > 0 else "red"))  # fmt: skip
        marks.setdefault(k + 1, []).append((0.0, TURN_MARK_LEN, "yellow"))
    last = len(segs) - 1
    marks.setdefault(last, []).append((segs[last][4] - TURN_MARK_LEN, segs[last][4], "blue"))
    colors = {**TURN_MARK_COLORS, "blue": blue}
    for k, sg in enumerate(segs):
        if sg[0] == "S":
            _, x0, y0, h, length = sg
            c, si = math.cos(h), math.sin(h)
            for name, v0, v1, col in bands:
                if name != "lane":
                    board(f"{name}{k}", x0 + 0.5 * length * c, y0 + 0.5 * length * si, h, length, v0, v1, col)
                    continue
                cuts, u = [], 0.0
                for m0, m1, mc in sorted(marks.get(k, [])):
                    if m0 > u + 1e-9:
                        cuts.append((u, m0, lam, "lane"))
                    cuts.append((m0, m1, colors[mc], f"mark_{mc}_"))
                    u = m1
                if u < length - 1e-9:
                    cuts.append((u, length, lam, "lane"))
                for j, (u0, u1, col2, nm) in enumerate(cuts):
                    um = 0.5 * (u0 + u1)
                    board(f"{nm}{k}_{j}", x0 + um * c, y0 + um * si, h, u1 - u0, v0, v1, col2)
            continue
        _, cx, cy, h0, a, _r = sg
        sgn = 1.0 if a > 0 else -1.0
        m = max(1, int(math.ceil(abs(math.degrees(a)) / TURN_DECK_STEP_DEG)))
        da = abs(a) / m
        for j in range(m):
            hm = h0 + a * (j + 0.5) / m
            px, py = cx + sgn * R * math.sin(hm), cy - sgn * R * math.cos(hm)
            for name, v0, v1, col in bands:
                r_out = R - v0 if sgn > 0 else R + v1  # the band's outer radius (v > 0 is left)
                board(f"{name}{k}_{j}", px, py, hm, 2 * r_out * math.tan(da / 2), v0, v1, col)
    need = 2 * (hw + A) + 0.02  # aprons of far-apart lane parts never overlap (>= 20 mm of bare floor between)
    dmin = _far_lane_clearance(pts, need)
    assert dmin >= need - 1e-3, (level, f"painted lanes {dmin * 1000:.0f} mm apart < {need * 1000:.0f} mm")
    s_total = sum(math.hypot(xb - xa, yb - ya) for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:], strict=True))
    goal_s0 = s_last + 0.02
    path = goal_s0 - s_start
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    params = {
        "turns": n,
        "turn_deg": ang,
        "radius_mm": r_mm,
        "lane_width_mm": w_mm,
        "straight_mm": d_mm,
        "total_turn_deg": n * ang,
        "path_m": round(path, 4),
        "painted": True,
        "border_mm": B * 1000,
        "lane_parts_half_width_m": round(hw + TURN_PAINT_PART_MARGIN, 4),
        "extent_m": [round(max(xs) - min(xs), 3), round(max(ys) - min(ys), 3)],
    }
    return Obstacle(
        "turns",
        level,
        params,
        f"{n} consecutive {ang} deg turn(s) (alternating, radius {r_mm} mm) in a {w_mm} mm painted lane on the flat "
        f"floor (no walls, no edges), {path:.2f} m to the goal",
        shapes,
        StartSpec(front_x=0.0),
        GoalSpec(x_min=round(goal_s0, 6), x_max=round(s_total, 6), y_half=hw + TURN_PAINT_PART_MARGIN, z_min=-0.02),
        fail_z=-0.05,
        time_limit_s=round(20.0 + 40.0 * path),
        floor_z=-T,
        lane_half_width=hw + TURN_PAINT_PART_MARGIN,
        stuck_window_s=6.0,
        centreline=pts,
    )


# ------------------------------------------------------------------------------------------------ gapdrop
GAPDROP_H_MM = 20.0  # drop of level 1 [mm]; every level drops 1.5 x more
GAPDROP_GAP_M = 0.150  # fixed horizontal gap between the start deck and the landing deck [m]
GAPDROP_N = 8
GAPDROP_LEVELS = [round(GAPDROP_H_MM * 1.5**i, 1) for i in range(GAPDROP_N)]  # drop mm: 20, 30, 45, 67.5 ... 341.7
GAPDROP_CHASM = 0.25  # chasm floor below the landing deck [m]
"""Gap-drop: a fixed 150 mm gap to a landing deck that is lower by h x 1.5^(L-1) (20 -> 342 mm). The small drops are
a gap crossing with a step down (a car longer than the gap rolls across); the big ones need the vehicle to survive
or control the descent (a long wheelbase, a nose leg / boom lowered onto the landing by a free motor, a cushioned
landing): a free fall of 342 mm hits at 2.6 m/s."""


def gapdrop(level: int, spec=None) -> Obstacle:
    """Cross a fixed gap (``GAPDROP_GAP_M``) from a start deck (top at z = 0) to a landing deck lower by the level's
    drop, over a chasm (``GAPDROP_CHASM`` below the landing). Fail: the chassis origin 50 mm below the landing deck
    (fell into the chasm); flipped / broke as usual (a hard landing that breaks the car fails). Goal: every part on the
    landing deck."""
    dmm = spec if spec is not None else GAPDROP_LEVELS[level - 1]
    drop, w = dmm * 1e-3, GAPDROP_GAP_M
    floor = round(-(drop + GAPDROP_CHASM), 5)
    plat = 0.50
    shapes = [
        box("platform_start", -0.60, 0.0, COURSE_HALF_WIDTH, 0.0, floor),
        box("platform_landing", w, w + plat, COURSE_HALF_WIDTH, -drop, floor, "wood"),
    ]
    return Obstacle(
        "gapdrop",
        level,
        {"drop_mm": dmm, "gap_mm": round(w * 1000, 1), "h_mm": GAPDROP_H_MM, "chasm_m": GAPDROP_CHASM},
        f"gap {w * 1000:.0f} mm to a landing deck {dmm} mm lower, over a chasm",
        shapes,
        StartSpec(front_x=-0.03),
        GoalSpec(x_min=w + 0.005, x_max=w + plat, z_min=-drop - 0.004, z_max=0.5),
        fail_z=round(-drop - 0.05, 5),
        time_limit_s=30.0,
        floor_z=floor,
        lane_half_width=COURSE_HALF_WIDTH,
    )


# ------------------------------------------------------------------------------------------------ wall
WALL_BRICK_MM = 9.6  # one LEGO brick
WALL_LEVELS = [1, 2, 4, 8, 16, 32, 64]  # wall height in bricks (9.6 mm to 614.4 mm)
WALL_THICK_M = 4 * 0.008  # wall thickness: 4 brick widths (4 x 8 mm = 32 mm), as the user specified
WALL_LANDING_M = 0.60  # floor beyond the wall where the vehicle must end
"""Wall: a thin vertical plywood wall (4 brick widths = 32 mm thick) across the lane, 1 -> 64 bricks high (doubling).
The vehicle must climb OVER it and come down on the floor beyond, upright and in one piece (a 32 mm top cannot carry
a car). Up to 4 bricks (38.4 mm) tyres and a lifting nose do it; 8-16 bricks (77-154 mm) need a lifting arm / jack /
tail on a free motor; 32 and 64 bricks (307 / 614 mm) need a hook arm over the top edge plus a controlled descent on
the far side (see the docs)."""


def wall(level: int, spec=None) -> Obstacle:
    """Climb over a thin vertical wall (plywood, ``WALL_THICK_M`` = 32 mm thick, across the 300 mm lane) and come down
    on the floor beyond. Height = bricks x 9.6 mm. Fail: off the lane sides (left_lane), flipped / broke; the goal is
    every part on the floor beyond the wall (x from the wall's far face + 10 mm to + ``WALL_LANDING_M``)."""
    n = spec if spec is not None else WALL_LEVELS[level - 1]
    h = round(n * WALL_BRICK_MM * 1e-3, 5)
    t = WALL_THICK_M
    shapes = [box("wall", 0.0, t, COURSE_HALF_WIDTH, h, 0.0, "wood")]
    return Obstacle(
        "wall",
        level,
        {"bricks": n, "height_mm": round(n * WALL_BRICK_MM, 1), "thickness_mm": round(t * 1000, 1),
         "landing_m": WALL_LANDING_M},  # fmt: skip
        f"climb over a vertical wall {n} brick(s) = {n * WALL_BRICK_MM:.1f} mm high, {t * 1000:.0f} mm thick",
        shapes,
        StartSpec(front_x=-0.03),
        GoalSpec(x_min=round(t + 0.01, 4), x_max=round(t + WALL_LANDING_M, 4), z_min=-0.004),
        fail_z=-0.05,
        time_limit_s=round(30.0 + 100.0 * h),
        lane_half_width=COURSE_HALF_WIDTH,
    )


# ------------------------------------------------------------------------------------------------ uturn
UTURN_LANE_W = 0.20  # width of each lane plate [m]
UTURN_GAP = 0.10  # gap between the two lanes' long sides [m]
UTURN_LANE_LEN = 0.55  # lane length from the start line to the tilted plate [m] (plus 0.45 m behind the start line)
UTURN_BACK = 0.45
UTURN_LEVELS = [0, 5, 10, 15, 20, 25, 30, 35]  # tilt of the connector plate [deg], sloping UP away from the lanes
UTURN_PLATE_T = 0.020
"""U-turn: two parallel lane plates at the same height (z = 0) side by side with a gap between their long sides, joined
at their far end by a connector plate hinged along the lanes' end line and tilted UP away from them (pitch about the
y axis): the vehicle climbs onto it, makes its 180 deg turn on the slope (side-hill: lateral grip and roll-over
margin), and comes back down onto lane B. Tilt 0 -> 35 deg."""


def uturn(level: int, spec=None) -> Obstacle:
    """Drive lane A (+x), U-turn on the tilted connector plate, drive back along lane B (-x, to the left of A) to its
    near end. Lanes: ``UTURN_LANE_W`` wide, ``UTURN_GAP`` apart, ``UTURN_LANE_LEN`` m from the start line to the
    plate. The plate spans both lanes and the gap and reaches 2R + 50 mm beyond the hinge (R = (lane + gap) / 2, the
    turn radius of the centreline), sloping UP away from the lanes at the level's tilt (the vehicle climbs onto the
    slope, makes its 180 deg turn on it and comes back down); wood. Below: a chasm. Judge on the centreline (lane A, a
    half circle of radius R on the plate, lane B): progress = arc length; fail: the chassis origin falls 50 mm below
    the lane tops (off a plate / into the gap), or more than R from the centreline (it crossed the gap between the
    lanes). Goal: every part on lane B's last 0.55 m (from x = 0.10 m back to its end)."""
    tilt = spec if spec is not None else UTURN_LEVELS[level - 1]
    hw, gap, L, back = UTURN_LANE_W / 2, UTURN_GAP, UTURN_LANE_LEN, UTURN_BACK
    if isinstance(tilt, (tuple, list)):  # final ladder: (tilt deg, lane width mm, gap mm)
        tilt, lane_mm, gap_mm = tilt
        hw, gap = lane_mm * 1e-3 / 2, gap_mm * 1e-3
    R = hw + gap / 2
    C = 2 * R + 0.05
    th = math.radians(tilt)
    high = C * math.sin(th)  # the plate's far edge, above the lane tops
    fail = -0.05
    floor = fail - 0.15
    yb = 2 * R
    shapes = [
        box("lane_a", -back, L, hw, 0.0, floor, "wood"),
        box("lane_b", -back, L, hw, 0.0, floor, "wood", y0=yb),
    ]
    pl = slab("plate", (L, 0.0), (L + C * math.cos(th), high), R + hw, UTURN_PLATE_T, "wood")
    pl.pos = (pl.pos[0], R, pl.pos[2])
    pl.color = BLUE
    shapes.append(pl)
    # centreline: lane A, the half circle on the plate (horizontal projection), lane B
    pts = [(-back, 0.0)]
    k = 60
    for j in range(k + 1):
        a = math.pi * j / k
        pts.append((round(L + R * math.sin(a), 6), round(R - R * math.cos(a), 6)))
    pts.append((-back, yb))
    seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:], strict=True)]
    s_arc_end = sum(seg[:-1])
    goal_s0 = s_arc_end + (L - 0.10)
    s_total = sum(seg)
    path = goal_s0 - back
    return Obstacle(
        "uturn",
        level,
        {"tilt_deg": tilt, "lane_width_mm": round(2 * hw * 1000), "gap_mm": round(gap * 1000), "lane_m": L,
         "turn_radius_mm": round(R * 1000), "plate_m": round(C, 3), "plate_rise_mm": round(high * 1000, 1),
         "path_m": round(path, 4)},  # fmt: skip
        f"U-turn on a plate tilted {tilt} deg up (turn radius {R * 1000:.0f} mm) between two {2 * hw * 1000:.0f} mm "
        f"lanes {gap * 1000:.0f} mm apart",
        shapes,
        StartSpec(front_x=0.0),
        GoalSpec(x_min=round(goal_s0, 6), x_max=round(s_total, 6), y_half=hw + 0.02, z_min=-0.004),
        fail_z=round(fail, 5),
        time_limit_s=round(30.0 + 25.0 * path),
        floor_z=round(floor, 5),
        lane_half_width=R,
        stuck_window_s=6.0,
        centreline=pts,
    )


OBSTACLE_KINDS = {
    "ramp": (ramp, len(RAMP_LEVELS) + len(RAMP_C_LEVELS)),
    "gap": (gap, len(GAP_LEVELS) + len(GAP_C_LEVELS)),
    "bridge": (bridge, len(BRIDGE_LEVELS)),
    "rope": (rope, len(ROPE_LEVELS) + len(ROPE_C_LEVELS)),
    "steps": (steps, len(STEP_LEVELS) + len(STEP_C_LEVELS)),
    "bumps": (bumps, len(BUMP_LEVELS)),
    "tunnel": (tunnel, len(TUNNEL_LEVELS) + len(TUNNEL_C_LEVELS)),
    "tug": (tug, len(TUG_LEVELS) + len(TUG_C_LEVELS)),
    "seesaw": (seesaw, len(SEESAW_LEVELS)),
    "speed": (speed, len(SPEED_LEVELS) + len(SPEED_C_LEVELS)),
    "turns": (turns, len(TURN_LEVELS)),
    "gapdrop": (gapdrop, len(GAPDROP_LEVELS)),
    "wall": (wall, len(WALL_LEVELS)),
    "uturn": (uturn, len(UTURN_LEVELS)),
}


TIER_C_FIRST = {"ramp": 16, "gap": 22, "rope": 10, "steps": 16, "tunnel": 8, "tug": 11, "speed": 8}
"""First level of each obstacle's constrained tier C (the base levels are unchanged)."""


# ------------------------------------------------------------------------------------------------ redo ladder
REDO_LEVELS: dict[str, list] = {
    # slope deg, surface, rise m: the rise grows x1.5 per level. Tyre friction on the rubber mat (PhysX average
    # combine): 62 mm tyre 1.66 static / 1.61 sliding (atan 58 deg), 43 mm tyre 1.43 (55 deg): 57 deg needs the 62 mm
    # tyres, every one of them driven (one motor: a driveshaft / bevel transfer) and a CoM low and forward.
    "ramp": [(20, "laminate", 0.040), (28, "laminate", 0.060), (36, "rubber_mat", 0.090), (44, "rubber_mat", 0.135),
             (50, "rubber_mat", 0.200), (54, "rubber_mat", 0.300), (57, "rubber_mat", 0.450)],
    # gap mm, x1.5 per level: from 150 mm a plain chassis must be > 2 x the gap; with a piece cap a counterweight /
    # deploying boom (cantilever) is needed
    "gap": [30, 45, 68, 100, 150, 225, 340, 500, 750],
    # plank width mm (x0.7 per level), plank length m: narrow planks need inline wheels with balance (gyro, keel,
    # pendulum) or wheels gripping the plank
    "bridge": [(60, 0.40), (42, 0.50), (30, 0.60), (21, 0.75), (15, 0.90), (10, 1.10)],
    # span m (x1.5), incline deg, sag mm, rope radius mm: steep inclines need a grip that does not rely on weight
    # (pinch rollers)
    "rope": [(0.50, 0.0, 5.0, 2.0), (0.75, 5.0, 12.0, 2.0), (1.10, 10.0, 25.0, 1.5), (1.70, 15.0, 40.0, 1.5),
             (2.50, 25.0, 50.0, 1.0)],
    # riser mm (x1.5 per level), count: curbs 10-110 mm (the biggest tyre is 62 mm across), then 3-step stairs with
    # 2.4 x riser treads: climbing mechanisms (lifting arm, jack, tail, whegs)
    "steps": [(10.0, 1), (15.0, 1), (22.0, 1), (34.0, 1), (50.0, 1), (75.0, 1), (110.0, 1), (34.0, 3), (50.0, 3)],
    # max rod height mm (x1.5 per level); rod radius 0.6 x the height (boulders)
    "bumps": [6.0, 9.0, 13.0, 20.0, 30.0, 45.0, 68.0],
    # clear height mm (x0.8 per level), tunnel length m (x1.35): the battery box is 32 mm across lying on its side,
    # the Large Hub 32 mm flat; 35 mm leaves 3 mm, only 24 mm wheels fit
    "tunnel": [(80, 0.20), (64, 0.30), (52, 0.40), (44, 0.55), (38, 0.75), (35, 1.00)],
    # sled g, x2 per level: pure traction pulls ~4 x the mass on the driven wheels; beyond, weight transfer (lift the
    # sled's weight onto the driven wheels)
    "tug": [100, 200, 400, 800, 1600, 3200],
    # pivot rise mm, plank width mm: tilt 6-42 deg on a narrowing 300 mm plank (climb, then a controlled descent)
    "seesaw": [(15, 80), (25, 60), (38, 45), (55, 35), (75, 28), (100, 22)],
    # time limit s for 1 m (/1.5 per level): 0.7 s from rest needs ~2 m/s top speed (gear up, light)
    "speed": [5.0, 3.3, 2.2, 1.5, 1.0, 0.7],
    # turns, angle deg, radius mm, lane mm, straight mm: the base ladder (already geometric: total turning x1.5-2 per
    # level, radius 400 -> 100 mm, lane 300 -> 180 mm); the top levels need a free-motor steering mechanism
    "turns": list(TURN_LEVELS),
    # drop mm (x1.5 per level) over the fixed 150 mm gap; wall height in bricks (x2); U-turn plate tilt deg (+5):
    # the base ladders, already geometric / steep enough
    "gapdrop": list(GAPDROP_LEVELS),
    "wall": list(WALL_LEVELS),
    "uturn": list(UTURN_LEVELS),
}
"""The redo ladder (``ladder="redo"``): geometric progressions (gaps and rises x1.5, sleds x2, planks x0.7, tunnels
x0.8, time /1.5) so that each obstacle's upper levels need a different mechanism. No tier C levels (the redo profile's
constraints apply to every level: one wheel-driving motor, a piece cap)."""

FINAL6_LEVELS: dict[str, list] = {
    # slope deg, surface, rise m: rubber mat throughout, 52 -> 57 deg (above a plain one-motor 4WD's ~50.5 deg;
    # 57 deg needs every 62 mm tyre driven and a low, forward CoM)
    "ramp": [(52, "rubber_mat", 0.18), (53, "rubber_mat", 0.21), (54, "rubber_mat", 0.24), (55, "rubber_mat", 0.27),
             (56, "rubber_mat", 0.30), (57, "rubber_mat", 0.35)],
    # gap mm, platform length m: max(0.45, 1.6 x gap + 0.15) on each side (buildable; the base rule's 3.5 x gap + 0.1
    # made L6 6.3 m long); chasm 150 mm deep, 400 mm from 200 mm
    "gap": [(g, round(max(0.45, 1.6 * g * 1e-3 + 0.15), 3)) for g in (100, 150, 225, 340, 500, 750)],
    "bridge": [(30, 0.5), (21, 0.6), (15, 0.7), (12, 0.8), (10, 0.9), (8, 1.0)],  # plank width mm, length m
    "rope": [(0.75, 5.0, 12.0, 2.0), (1.10, 10.0, 25.0, 1.5), (1.40, 15.0, 30.0, 1.5), (1.70, 15.0, 40.0, 1.25),
             (2.10, 20.0, 45.0, 1.0), (2.50, 25.0, 50.0, 1.0)],  # span m, incline deg, sag mm, rope radius mm
    "steps": [(34.0, 1), (50.0, 1), (75.0, 1), (110.0, 1), (50.0, 3), (75.0, 3)],  # riser mm, count (treads 2.4 x riser)
    "bumps": [(45.0, 14), (52.0, 14), (57.0, 16), (63.0, 16), (70.0, 18), (80.0, 18)],  # max rod mm, rod count
    "seesaw": [(55, 35), (65, 30), (75, 28), (88, 25), (100, 22), (110, 20)],  # pivot rise mm, plank mm (21.5 -> 47 deg)
    "gapdrop": GAPDROP_LEVELS[2:],  # drop mm over the fixed 150 mm gap: 45 -> 342
    "wall": [4, 8, 16, 32, 48, 64],  # bricks (32 mm thick wall: climb over, land beyond)
    "uturn": [(10, 200, 100), (15, 190, 110), (20, 180, 120), (25, 170, 130), (30, 160, 140), (35, 150, 150)],
}
"""The previous final ladder (``ladder="final6"``, the ``final`` ladder until 2026-10-04): 10 benches x 6 levels, the
easy starting levels removed (every level needs a real mechanism), run with the redo profile (one wheel-driving motor,
capped parts, piece caps). Tunnel, tug and speed are not part of it, nor turns (dropped with the hub, angular motors and
sensors: built with Power Functions only, part set ``capped_pf``). The final-campaign results logged before
2026-10-04 (``results/lego_bench/campaigns/final``, ladder ``final`` in the log) were run on this ladder."""


# ------------------------------------------------------------------------------------------------ final ladder (LEGO)
# The final ladder (``ladder="final"``): 10 benches x 5 levels, each level exactly what a simple real build produces:
# LEGO 10 in baseplates (32 x 32 studs, 256 mm), LEGO columns (2x2 bricks) and decks (two crossed plate layers + tiles)
# and very few 3D-printed parts. Heights are whole plates (3.2 mm) / bricks (9.6 mm), widths and positions are on the
# 8 mm stud grid. Floor benches (ramp, steps, bumps, seesaw, wall): baseplates at z = -3.2 mm, 3.2 mm LEGO floor tiles
# make z = 0. Raised benches (gap, bridge, gapdrop, uturn): LEGO decks 99.2 mm (31 plates) above the baseplate sheet,
# which is the chasm floor. Printed parts (real_world_kit/print_final): bridge plank segments, rope saddles and
# anchors, bump plates, the seesaw plank, U-turn wedge adapters; their geometry is read from these generators.
LEGO_STUD = 0.008
LEGO_PLATE = 0.0032
LEGO_BRICK = 0.0096
LEGO_SURF = "abs"  # LEGO tiles / bricks and the printed (PLA) parts: ABS-like friction
LEGO_GREY = (0.62, 0.64, 0.68)
PRINT_COLOR = (0.95, 0.55, 0.15)
F_HALF = 0.152  # decks, steps, walls and plateaus: 38 studs (304 mm) across
F_DECK_H = 0.0992  # raised decks: top 31 plates above the baseplate sheet (the chasm floor)
TECHNIC_HOLE_Z = 0.0058  # a Technic brick's hole axis above the brick's bottom


def _r6(v: float) -> float:
    return round(v, 6)


def _bisect(f, lo: float, hi: float, n: int = 100) -> float:
    """Root of f on [lo, hi] (f(lo), f(hi) of opposite signs)."""
    flo = f(lo)
    for _ in range(n):
        mid = 0.5 * (lo + hi)
        if (f(mid) > 0) == (flo > 0):
            lo, flo = mid, f(mid)
        else:
            hi = mid
    return 0.5 * (lo + hi)


# ---- ramp: one sharp LEGO slope panel (rubber mat) from the floor to a LEGO plateau
F_RAMP_PANEL_L = 0.460  # panel length along the slope (constant for every level; real build: MDF + rubber mat)
F_RAMP_PANEL_T = 0.0048  # a 3/16 in (4.8 mm) foam-core board, as built (was 8 mm: plate layers + rubber mat)
F_RAMP_STOP_L = 0.016  # foot stop: a 2-stud row of plates + tiles, top 3.2 mm above the floor
F_RAMP_PLATEAU_L = 0.352  # 44 studs
F_RAMP_SEAT_DZ = (0.003, 0.0096)  # the seat touches the panel's underside 3-9.6 mm (vertical) below its upper end
F_RAMP_PANEL_KG = 0.15  # (2026-10-24, user: the board swings) 460 x 304 x 4.8 mm foam-core board (~0.67 l at
# ~0.2 kg/l, paper faces included) with its surface: ~0.15 kg
F_RAMP_HINGED = True  # the board hangs from the plateau edge on a hinge along its top edge (tape on the real course);
# its foot just rests on the floor (board/floor friction), so a car pushing on its lower part can lift / swing it


def final_ramp_geometry(theta0: float) -> dict:
    """Real ramp build for a nominal slope ``theta0`` [deg]. The panel (256 mm along the slope, 8 mm thick) rests with its
    lower bottom edge on the floor (x = 0) against a 2-stud stop (plate + tile, 3.2 mm high: its top corner touches the
    panel's end face) and with its underside on the front edge of the plateau's seat strip (a LEGO height h_s just
    below the panel's upper end). Stop and seat sit on the stud grid, so h_s / tan(a) + 3.2 tan(a) is a whole number
    of studs: that fixes the actual angle a (the closest to theta0). The plateau deck behind the seat strip is the LEGO
    height nearest the panel's top corner (crest step <= 1.6 mm)."""
    P, T, pl, st = F_RAMP_PANEL_L, F_RAMP_PANEL_T, LEGO_PLATE, LEGO_STUD
    th0 = math.radians(theta0)
    best = None
    j_top = int(P * math.sin(th0) / pl)
    for j in range(j_top - 5, j_top + 1):
        hs = j * pl

        def f(th, hs=hs):
            return hs / math.tan(th) + pl * math.tan(th)

        k0 = round(f(th0) / st)
        for k in range(k0 - 4, k0 + 5):
            lo, hi = th0 - math.radians(8), th0 + math.radians(8)
            if not (f(hi) - k * st) < 0 < (f(lo) - k * st):
                continue
            th = _bisect(lambda a, k=k: f(a) - k * st, lo, hi)
            dz = P * math.sin(th) - hs
            if not F_RAMP_SEAT_DZ[0] <= dz <= F_RAMP_SEAT_DZ[1]:
                continue
            if best is None or abs(th - th0) < abs(best[0] - th0):
                best = (th, hs, k)
    th, hs, k = best
    s, c = math.sin(th), math.cos(th)
    xs = -pl * s / c  # the stop's front face
    xc = hs * c / s  # the seat strip's front edge (it touches the panel's underside)
    ex, ez = P * c, P * s  # panel: upper bottom corner
    ux, uz = ex - T * s, ez + T * c  # panel: upper top corner
    H = round(uz / pl) * pl  # plateau deck top (LEGO height nearest the panel's top corner)
    m = max(1, math.ceil((ex + 0.0005 - xc) / st - 1e-9))
    xd = xc + m * st  # plateau deck front face
    return {
        "slope_deg": round(math.degrees(th), 2), "slope_rad": th, "rise_m": _r6(H), "seat_m": _r6(hs),
        "stop_x_m": _r6(xs), "seat_x_m": _r6(xc), "plateau_x_m": _r6(xd), "stop_to_seat_studs": k,
        "seat_studs": m, "crest_step_mm": round((H - uz) * 1000, 2), "crest_notch_mm": round((xd - ux) * 1000, 1),
        "foot_lip_mm": round(T * c * 1000, 2), "panel_top_corner_m": (_r6(ux), _r6(uz)),
    }


def final_ramp(level: int, spec) -> Obstacle:
    """Climb a sharp rubber-mat LEGO slope panel to a LEGO plateau (``final_ramp_geometry``)."""
    theta0, surface = spec
    g = final_ramp_geometry(theta0)
    th, H, xs, xc, xd = g["slope_rad"], g["rise_m"], g["stop_x_m"], g["seat_x_m"], g["plateau_x_m"]
    P, T = F_RAMP_PANEL_L, F_RAMP_PANEL_T
    s, c = math.sin(th), math.cos(th)
    panel = Shape("ramp", "box", (P, 2 * F_HALF, T), (_r6(P / 2 * c - T / 2 * s), 0.0, _r6(P / 2 * s + T / 2 * c)),
                  pitch_quat(math.degrees(th)), surface=surface, color=SURF_COLOR.get(surface, GREY),
                  dynamic=F_RAMP_HINGED, mass_kg=F_RAMP_PANEL_KG if F_RAMP_HINGED else 0.0)
    shapes = [  # (no stop strip at the foot: the real board just rests on the floor, user 2026-10-22)
        panel,
        box("seat", xc, xd, F_HALF, g["seat_m"], 0.0, LEGO_SURF, LEGO_GREY),
        box("plateau", xd, xd + F_RAMP_PLATEAU_L, F_HALF, H, 0.0, LEGO_SURF, LEGO_GREY),
    ]
    params = {k: v for k, v in g.items() if k != "slope_rad"}
    params.update(slope_nominal_deg=theta0, surface=surface, panel_mm=(P * 1000, 2 * F_HALF * 1000, T * 1000),
                  ramp_length_m=_r6(xd), slope_length_m=P)
    ob = Obstacle(
        "ramp",
        level,
        params,
        f"sharp ramp {g['slope_deg']} deg (a {P * 1000:.0f} mm foam-core board"
        + (" hinged at the plateau edge, its foot resting on the floor" if F_RAMP_HINGED else " resting on the floor")
        + ") up to a plateau "
        f"{H * 1000:.1f} mm high",
        shapes,
        StartSpec(front_x=_r6(xs - F_RAMP_STOP_L - 0.02)),
        GoalSpec(x_min=_r6(xd + 0.01), x_max=_r6(xd + F_RAMP_PLATEAU_L), z_min=_r6(H - 0.004)),
        fail_z=-0.05,
        time_limit_s=round(30.0 + 50.0 * max(0.0, H - 0.1), 1),
        floor_surface=LEGO_SURF,
        lane_half_width=F_HALF,
    )
    if F_RAMP_HINGED:  # hinge along the board's top edge (mid-thickness of its upper end), across the course
        params["panel_hinge_m"] = (_r6(P * c - T / 2 * s), _r6(P * s + T / 2 * c))
        params["panel_kg"] = F_RAMP_PANEL_KG
        hx, hz = params["panel_hinge_m"]
        ob.joints.append(JointSpec("panel_hinge", "revolute", "", "ramp", (hx, 0.0, hz), axis="Y"))
    return ob


# ---- gap: two LEGO decks
def final_gap(level: int, spec) -> Obstacle:
    """Cross a gap (a whole number of studs) between two LEGO decks 99.2 mm above the baseplate floor. Platforms
    max(0.45, 1.6 x gap + 0.15) m long rounded up to whole studs (the start one 6 studs longer)."""
    gmm = spec
    w = gmm * 1e-3
    plat = _r6(math.ceil(max(0.45, 1.6 * w + 0.15) / LEGO_STUD - 1e-9) * LEGO_STUD)
    shapes = [
        box("platform_start", -(plat + 0.048), 0.0, F_HALF, 0.0, -F_DECK_H, LEGO_SURF, LEGO_GREY),
        box("platform_goal", w, w + plat, F_HALF, 0.0, -F_DECK_H, LEGO_SURF, LEGO_GREY),
    ]
    return Obstacle(
        "gap",
        level,
        {"gap_mm": gmm, "gap_studs": round(gmm / 8), "drop_m": F_DECK_H, "platform_m": plat},
        f"gap {gmm} mm ({round(gmm / 8)} studs) between two LEGO decks, {F_DECK_H * 1000:.1f} mm deep",
        shapes,
        StartSpec(front_x=-0.03),
        GoalSpec(x_min=_r6(w + 0.005), x_max=_r6(w + plat), z_min=-0.004),
        fail_z=-0.02,
        time_limit_s=20.0 if gmm <= 160 else round(20.0 + 25.0 * w, 1),
        floor_z=-F_DECK_H,
        lane_half_width=F_HALF,
    )


# ---- bridge: printed plank segments on hidden LEGO columns
F_BRIDGE_L = 0.640  # 80 studs
F_BRIDGE_SEG = 0.160  # 4 identical printed segments per width
F_BRIDGE_T = 0.0096  # plank thickness (one brick): top flush with the decks
F_BRIDGE_START, F_BRIDGE_GOAL = 0.456, 0.400


def final_bridge_supports() -> list[tuple[float, float]]:
    """x ranges [m] of the hidden support columns under the plank: a 1-stud column at each end (one jumper plate, its
    stud in the end socket) and a 2-stud column at each joint (two 1x2 jumper plates: one stud per segment end)."""
    out = [(0.0, LEGO_STUD)]
    n = round(F_BRIDGE_L / F_BRIDGE_SEG)
    out += [(_r6(i * F_BRIDGE_SEG - LEGO_STUD), _r6(i * F_BRIDGE_SEG + LEGO_STUD)) for i in range(1, n)]
    out.append((_r6(F_BRIDGE_L - LEGO_STUD), F_BRIDGE_L))
    return out


def final_bridge(level: int, spec) -> Obstacle:
    """Drive along a printed plank (``spec`` mm wide, 640 mm long, 9.6 mm thick, top flush with the decks) over a
    99.2 mm drop. Four 160 mm segments, each gripping one stud of the hidden LEGO support columns (16 mm wide, top
    9.6 mm below the plank top) at both ends."""
    wmm = spec
    w = wmm * 1e-3
    shapes = [
        box("platform_start", -F_BRIDGE_START, 0.0, F_HALF, 0.0, -F_DECK_H, LEGO_SURF, LEGO_GREY),
        box("plank", 0.0, F_BRIDGE_L, w / 2, 0.0, -F_BRIDGE_T, LEGO_SURF, PRINT_COLOR),
        box("platform_goal", F_BRIDGE_L, F_BRIDGE_L + F_BRIDGE_GOAL, F_HALF, 0.0, -F_DECK_H, LEGO_SURF, LEGO_GREY),
    ]
    for i, (x0, x1) in enumerate(final_bridge_supports()):
        shapes.append(box(f"support{i}", x0, x1, LEGO_STUD, -F_BRIDGE_T, -F_DECK_H, LEGO_SURF, LEGO_GREY))
    return Obstacle(
        "bridge",
        level,
        {"plank_width_mm": wmm, "plank_length_m": F_BRIDGE_L, "plank_thickness_mm": F_BRIDGE_T * 1000,
         "segments": round(F_BRIDGE_L / F_BRIDGE_SEG), "segment_mm": F_BRIDGE_SEG * 1000, "drop_m": F_DECK_H},
        f"printed plank {wmm} mm wide, {F_BRIDGE_L * 1000:.0f} mm long over a {F_DECK_H * 1000:.1f} mm drop",
        shapes,
        StartSpec(front_x=-0.03),
        GoalSpec(x_min=_r6(F_BRIDGE_L + 0.005), x_max=_r6(F_BRIDGE_L + F_BRIDGE_GOAL), z_min=-0.004),
        fail_z=-0.03,
        time_limit_s=max(30.0, round(15.0 + 25.0 * F_BRIDGE_L, 1)),
        floor_z=-F_DECK_H,
        lane_half_width=F_HALF,
    )


# ---- rope: LEGO decks, printed saddles (pressed onto the deck-edge studs) and table-clamped anchor posts
F_ROPE_TOWER = 0.192  # start deck top above the table (20 bricks); the goal tower is the same on a rigid riser
F_ROPE_PLATFORM = 0.704  # 88 studs
F_ROPE_SADDLE = (0.010, 0.008)  # printed saddle post: 10 mm along the cord, 8 mm across, top at the cord axis
# Two-table course (2026-10-07, user request; LEGO_BENCH_ROPE_EDGE=0 restores the anchor-ramp course): a fixed table
# and a standing desk, one printed edge device each (print_final/rope_devices): the cord runs only between the two
# saddle posts; each device puts a low clamp arm on the deck edge. The car starts on the deck and climbs onto the
# cord over the post.
F_ROPE_EDGE = os.environ.get("LEGO_BENCH_ROPE_EDGE", "1") == "1"
F_ROPE_DEVICE_ARM = (-0.032, 0.0185, 0.024, 0.008)  # x0, x1 (from the deck edge), width, height of the clamp's top arm
F_ROPE_DEVICE_POST = (-0.0025, 0.0175)  # the saddle post along the cord, over the clamp's spine (the cord goes down a
# hole in its top at x = 6 mm; on the gap side it lies in a groove over the post's rounded nose)
F_ROPE_DEVICE_POST_W = 0.010  # the saddle post across the cord (printed: on the clamp's bed side, cord in its middle)
F_ROPE_EDGE_START = -0.06  # car front 60 mm before the deck edge


F_ROPE_POST_NOSE_R = 0.006  # the printed saddle post's gap-side top edge is rounded (make_rope_devices.py NOSE_R 6 mm)


def _rope_post(name, x0, x1, z_bot, z_top, gap_dir):
    """a printed saddle post: a box with its gap-side top edge rounded to F_ROPE_POST_NOSE_R (gap_dir +1: the gap is at
    x1, -1: at x0), as printed, so a wheel or pulley arriving from the gap rolls up over the nose"""
    r = F_ROPE_POST_NOSE_R
    w = F_ROPE_DEVICE_POST_W / 2
    if gap_dir > 0:
        xa, xb, xc = x0, x1 - r, x1 - r  # full-height block, nose strip, nose centre
        strip = box(name + "_nose_base", x1 - r, x1, w, z_top - r, z_bot, LEGO_SURF, PRINT_COLOR)
    else:
        xa, xb, xc = x0 + r, x1, x0 + r
        strip = box(name + "_nose_base", x0, x0 + r, w, z_top - r, z_bot, LEGO_SURF, PRINT_COLOR)
    return [box(name, xa, xb, w, z_top, z_bot, LEGO_SURF, PRINT_COLOR), strip,
            Shape(name + "_nose", "cylinder", (r, 2 * w), (xc, 0.0, z_top - r), axis="Y", surface=LEGO_SURF,
                  color=PRINT_COLOR)]


def final_rope_rise(span: float, incline_deg: float) -> float:
    """Goal deck height above the start deck [m]: span x tan(incline), rounded to whole plates."""
    return _r6(round(span * math.tan(math.radians(incline_deg)) / LEGO_PLATE) * LEGO_PLATE)


def final_rope(level: int, spec) -> Obstacle:
    """Rope over a chasm (``rope``) with the span fixed and the goal deck raised by a whole number of plates (the actual
    incline is atan(rise / span)). LEGO decks 704 mm long; saddle posts 10 x 8 mm up to the cord axis (the cord lies in
    a groove in their top); the cord ends go over the flush tops of two printed anchor posts 400 mm behind / beyond
    the saddles. The chasm floor is the table, 192 mm below the start deck."""
    span, inc0, sag_mm, r_mm = spec[:4]
    # optional (2026-10-23, filmed CAL3 courses): spec[4] = goal device raised by this many mm (a brick under its clamp
    # arm: arm, post and cord end go up, the deck does not); spec[5] = the goal deck's measured rise in mm (instead of
    # span x tan(incline) rounded to whole plates). A plain 4-tuple builds exactly what it did before.
    raise_g = float(spec[4]) * 1e-3 if len(spec) > 4 and spec[4] else 0.0
    dz = _r6(float(spec[5]) * 1e-3) if len(spec) > 5 and spec[5] is not None else final_rope_rise(span, inc0)
    inc = math.degrees(math.atan2(dz + raise_g, span))  # the cord's incline, saddle to saddle
    ob = rope(level, spec=(span, inc, sag_mm, r_mm))
    r = r_mm * 1e-3
    top = ROPE_EDGE_TOP - r  # saddle top = cord axis
    sx, sy = F_ROPE_SADDLE
    if F_ROPE_EDGE:  # two tables, one printed edge device each: the cord ends at the saddle posts (no cord on the decks)
        caps = [sh for sh in ob.shapes if sh.kind == "capsule" and 0.0 < sh.pos[0] < span]
        a0, a1, aw, ah = F_ROPE_DEVICE_ARM
        p0, p1 = F_ROPE_DEVICE_POST
        ob.shapes = caps + [
            box("platform_start", -F_ROPE_PLATFORM, 0.0, F_HALF, 0.0, -F_ROPE_TOWER, LEGO_SURF, LEGO_GREY),
            box("platform_goal", span, span + F_ROPE_PLATFORM, F_HALF, dz, -F_ROPE_TOWER, LEGO_SURF, LEGO_GREY),
            box("device_arm_start", a0, a1, aw / 2, ah, 0.0, LEGO_SURF, PRINT_COLOR),
            box("device_arm_goal", span - a1, span - a0, aw / 2, dz + raise_g + ah, dz, LEGO_SURF, PRINT_COLOR),
        ] + _rope_post("saddle_start", p0, p1, 0.0, top, +1) + _rope_post("saddle_goal", span - p1, span - p0, dz,
                                                                           dz + raise_g + top, -1)
        ob.start.front_x = F_ROPE_EDGE_START
    else:
        ob.shapes = [sh for sh in ob.shapes if sh.kind == "capsule"] + [
            box("platform_start", -F_ROPE_PLATFORM, 0.0, F_HALF, 0.0, -F_ROPE_TOWER, LEGO_SURF, LEGO_GREY),
            box("platform_goal", span, span + F_ROPE_PLATFORM, F_HALF, dz, -F_ROPE_TOWER, LEGO_SURF, LEGO_GREY),
            box("saddle_start", -0.0001 - sx, -0.0001, sy / 2, top, 0.0, LEGO_SURF, PRINT_COLOR),
            box("saddle_goal", span + 0.0001, span + 0.0001 + sx, sy / 2, dz + top, dz, LEGO_SURF, PRINT_COLOR),
        ]
    if raise_g:
        ob.params["goal_device_raise_mm"] = round(raise_g * 1000, 2)
    ob.params.update(incline_deg=round(inc, 3), incline_nominal_deg=inc0, goal_deck_z_m=dz, goal_rise_mm=round(dz * 1000, 1),
                     goal_platform_m=F_ROPE_PLATFORM, start_platform_m=F_ROPE_PLATFORM, cord_d_mm=2 * r_mm,
                     start_tower_mm=F_ROPE_TOWER * 1000, saddle_mm=(sx * 1000, sy * 1000, round(top * 1000, 2)))
    ob.description = (f"rope {span * 1000:.0f} mm span between two LEGO decks, {inc:.2f} deg incline (goal deck "
                      f"{dz * 1000:.1f} mm higher), {sag_mm} mm sag, {2 * r_mm} mm cord")
    ob.goal.x_max = _r6(span + F_ROPE_PLATFORM)
    ob.floor_z = -F_ROPE_TOWER
    ob.lane_half_width = 0.1
    return ob


# ---- steps: three LEGO steps (bricks + tiles)
def final_steps(level: int, spec) -> Obstacle:
    """Climb ``count`` LEGO steps of ``riser`` mm (whole plates), treads 2.4 x riser rounded to whole studs."""
    h_mm, n = spec
    h = h_mm * 1e-3
    tread = _r6(round(2.4 * h / LEGO_STUD) * LEGO_STUD)
    shapes = []
    for i in range(n):
        x0 = i * tread
        x1 = x0 + tread if i < n - 1 else x0 + 0.352
        shapes.append(box(f"step{i}", _r6(x0), _r6(x1), F_HALF, _r6((i + 1) * h), 0.0, LEGO_SURF, LEGO_GREY))
    top = n * h
    x_goal = _r6((n - 1) * tread + 0.01)
    return Obstacle(
        "steps",
        level,
        {"step_mm": h_mm, "count": n, "tread_m": tread, "riser_plates": round(h / LEGO_PLATE)},
        f"{n} LEGO steps of {h_mm} mm, {tread * 1000:.0f} mm treads",
        shapes,
        StartSpec(front_x=-0.03),
        GoalSpec(x_min=x_goal, x_max=_r6(x_goal + 0.34), z_min=_r6(top - 0.003)),
        fail_z=-0.05,
        time_limit_s=round(30.0 + 10.0 * n + 150.0 * h, 1),
        floor_surface=LEGO_SURF,
        lane_half_width=F_HALF,
    )


# ---- bumps: rods printed on two bump plates (32 x 32 studs, one per 10 in baseplate) that press onto the baseplates
F_BUMP_PITCH = 0.256  # one 10 in baseplate (32 studs); the printed outline is 32 x 8 - 0.2 = 255.8 mm
F_BUMP_N = 2  # plates per level: the field is 512 mm long
F_BUMP_PLATES = tuple((_r6(k * F_BUMP_PITCH), _r6((k + 1) * F_BUMP_PITCH)) for k in range(F_BUMP_N))  # x ranges [m]
F_BUMP_FIELD = F_BUMP_PLATES[-1][1]
F_BUMP_PART_MARGIN = 0.02  # (user 2026-10-25) part origins may step this far outside the plates
F_BUMP_WHEEL_TOL = 0.005  # (v12) a wheel's origin over the bump field stays within the rods' y span +- this [m]: a
# wide car may not put its wheels in the rod-free strips along the plate edges (Judge: "a wheel went around the rods")
F_BUMP_PLATE_Y = 0.128  # y -0.128 .. 0.128 (32 studs, the central 256 mm of the 304 mm lane; 24 mm tile strips beside)
F_BUMP_MARGIN = 0.0021  # every rod stays 2 mm inside its plate's printed outline (outline 0.1 mm inside the pitch)


F_BUMP_AS_PRINTED = True
"""Rods as on the printed plates (real_world_kit/print_final/bumps/L*/bump_plate_{1,2}.stl): 40-220 mm long, off-centre
in y. Restored 2026-10-22 (user: use the bumps that were printed); False gives the full-span variant."""


def final_bump_rods(level: int, hmm: float, n_rods: int, seed: int = 7) -> list[dict]:
    """Seeded rod layout: rod radius 0.6 x the max height, heights 0.6-1 x the max. Every rod spans its bump plate
    across the lane (y -125.9 .. 125.9 mm, 2 mm inside the 255.8 mm outline; 40-220 mm long, off-centre until
    2026-10-13) and lies on one plate (its whole circle within the plate's x range)."""
    rng = random.Random(seed * 100 + level + 4000)
    hmax = hmm * 1e-3
    R = 0.6 * hmax
    rods = []
    while len(rods) < n_rods:
        x = rng.uniform(0.0, F_BUMP_FIELD)
        k = min(int(x / F_BUMP_PITCH), F_BUMP_N - 1)
        x0, x1 = F_BUMP_PLATES[k]
        if not x0 + R + F_BUMP_MARGIN <= x <= x1 - R - F_BUMP_MARGIN:
            continue
        hgt = hmax * rng.uniform(0.6, 1.0)
        length = rng.uniform(0.04, 0.22)
        ylim = min(0.08, F_BUMP_PLATE_Y - F_BUMP_MARGIN - length / 2)
        y = rng.uniform(-ylim, ylim)
        if not F_BUMP_AS_PRINTED:  # full-span variant (2026-10-13 .. 10-22; never printed)
            length, y = 2 * (F_BUMP_PLATE_Y - F_BUMP_MARGIN), 0.0
        rods.append({"x": round(x, 5), "y": round(y, 5), "top": round(hgt, 5), "len": round(length, 5), "plate": k})
    rods.sort(key=lambda d: d["x"])
    return rods


def final_bumps(level: int, spec, seed: int = 7) -> Obstacle:
    """Rough terrain on two printed bump plates (32 x 32 studs = one 10 in baseplate each, 255.8 x 255.8 x 3.2 mm, top
    flush with the floor tiles): ``n`` rods (radius 0.6 x the max height, axis along y, partly sunk into the plate) over
    0.512 m, each entirely on one plate."""
    hmm, n_rods = spec
    R = 0.6 * hmm * 1e-3
    rods = final_bump_rods(level, hmm, n_rods, seed)
    shapes = [Shape(f"bump{i}", "cylinder", (_r6(R), d["len"]), (d["x"], d["y"], _r6(d["top"] - R)), axis="Y",
                    surface=LEGO_SURF, color=PRINT_COLOR) for i, d in enumerate(rods)]
    return Obstacle(
        "bumps",
        level,
        {"max_bump_mm": hmm, "rods": len(rods), "seed": seed, "rod_radius_mm": round(R * 1000, 2),
         "plates_mm": [[round(x0 * 1000, 3), round(x1 * 1000, 3), -F_BUMP_PLATE_Y * 1000, F_BUMP_PLATE_Y * 1000]
                       for x0, x1 in F_BUMP_PLATES],
         "field_m": F_BUMP_FIELD, "rod_plate": [d["plate"] for d in rods],
         # (v12) the rods' lateral span [m]: wheels over the field stay within it (+ F_BUMP_WHEEL_TOL; Judge)
         "rods_y_span_m": [round(min(d["y"] - d["len"] / 2 for d in rods), 5),
                           round(max(d["y"] + d["len"] / 2 for d in rods), 5)]},
        f"rough terrain: {len(rods)} rods up to {hmm} mm high over {F_BUMP_FIELD:g} m on {F_BUMP_N} printed bump plates "
        f"(32 x 32 studs)",
        shapes,
        StartSpec(front_x=-0.01),
        # (user 2026-10-25) crossed when half the car is past the end of the bump line
        # (v12) no lateral bound past the field either (y_half was 0.2 m: a car driving on and veering during the 5 s
        # hold failed "left the goal after crossing")
        GoalSpec(x_min=_r6(F_BUMP_FIELD), x_max=INF, y_half=INF, z_min=-0.004, ref="half"),  # driving on past the field is fine
        fail_z=-0.05,
        time_limit_s=30.0,
        floor_surface=LEGO_SURF,
        lane_half_width=F_BUMP_PLATE_Y,  # the plates' width: with FINAL_OVER_KINDS every part must stay on them
    )


# ---- seesaw: a printed plank on a LEGO axle through Technic bricks on LEGO columns
F_SEESAW_L, F_SEESAW_W = 0.240, 0.064  # plank (8 studs wide)
F_SEESAW_T = 0.008  # plank top to the rib bottoms (1.6 mm skin + ribs)
F_SEESAW_AXLE_D = 0.010  # cross-axle hole centre below the plank top
F_SEESAW_GAP = 0.256  # constant gap between the platforms (32 studs); the axle is at its centre (a stud-grid line)
F_SEESAW_H = 0.0288  # platform tops above the floor (3 bricks + 1 tile above the baseplate)
F_SEESAW_LEDGE = F_SEESAW_H - 2 * LEGO_PLATE  # end ledges: 2 plates lower than the platforms
F_SEESAW_PLANK_G = 54.0  # print_final/seesaw/seesaw_plank.stl (51.8 g at 20 % infill) + axle 12 + 2 bushes (~2 g)
F_SEESAW_PLAT_L = 0.400


def final_seesaw_geometry(tilt0: float) -> dict:
    """Axle column height (whole plates under a Technic brick: axle at c + 2.6 mm above the floor) and the actual tilt:
    in the start pose the near end's rib bottoms rest on the near ledge (2 plates below the platform tops); choose the
    column giving the tilt closest to ``tilt0``."""
    half, zr = F_SEESAW_L / 2, F_SEESAW_AXLE_D - F_SEESAW_T  # rib bottoms 2 mm above the axle
    best = None
    for j in range(4, 80):
        za = j * LEGO_PLATE - LEGO_PLATE + TECHNIC_HOLE_Z
        dz = za - F_SEESAW_LEDGE
        if not 0 < dz < half * 0.95:
            continue
        th = _bisect(lambda a, dz=dz: half * math.sin(a) - zr * math.cos(a) - dz, 0.0, math.radians(80))
        if best is None or abs(th - math.radians(tilt0)) < abs(best[0] - math.radians(tilt0)):
            best = (th, j, za)
    th, j, za = best
    s, c = math.sin(th), math.cos(th)
    xp = F_SEESAW_GAP / 2
    top_near = za - half * s + F_SEESAW_AXLE_D * c
    x_contact = xp - half * c - zr * s
    a = round((x_contact - LEGO_STUD) / LEGO_STUD)
    return {"tilt_deg": round(math.degrees(th), 2), "tilt_rad": th, "axle_column_plates": j, "axle_z_m": _r6(za),
            "axle_x_m": xp, "ledge_x_m": (_r6(a * LEGO_STUD), _r6(a * LEGO_STUD + 2 * LEGO_STUD)),
            "end_step_mm": round((top_near - F_SEESAW_H) * 1000, 2),
            "end_gap_mm": round((xp - half * c - F_SEESAW_AXLE_D * s) * 1000, 1)}


def final_seesaw(level: int, spec) -> Obstacle:
    """Drive over a printed seesaw plank (240 x 64 mm, ribbed, ``F_SEESAW_PLANK_G`` g) on a LEGO cross axle through
    Technic bricks on 2x2 LEGO columns beside it, between two LEGO platforms 28.8 mm high and 256 mm apart. It starts
    tilted with its near end on a LEGO ledge (top flush with the platform within ``end_step_mm``), tips when the car
    passes the axle and lands on the far ledge."""
    # spec < 0 (2026-10-16, E0 v4 extension): the same seesaw tipped the other way: it starts with its far end down on the far
    # ledge and its near end raised above the start platform (the plank mirrored about the axle)
    flip = spec < 0
    g = final_seesaw_geometry(abs(spec))
    th, za, xp = g["tilt_rad"], g["axle_z_m"], g["axle_x_m"]
    s, c = math.sin(th), math.cos(th)
    off = F_SEESAW_AXLE_D - F_SEESAW_T / 2  # plank box centre above the axle (plank frame)
    centre = (_r6(xp + off * s if flip else xp - off * s), 0.0, _r6(za + off * c))
    plank = Shape("plank", "box", (F_SEESAW_L, F_SEESAW_W, F_SEESAW_T), centre,
                  pitch_quat(-math.degrees(th) if flip else math.degrees(th)),
                  surface=LEGO_SURF, color=PRINT_COLOR, dynamic=True, mass_kg=F_SEESAW_PLANK_G * 1e-3)
    near_top = za + (F_SEESAW_L / 2) * s + F_SEESAW_AXLE_D * c  # the raised near end's top edge (flipped)
    l0, l1 = g["ledge_x_m"]
    hw = F_SEESAW_W / 2
    col_top = _r6(za + LEGO_BRICK - TECHNIC_HOLE_Z)  # Technic brick top (its studs 1.7 mm higher)
    shapes = [
        box("platform_start", -F_SEESAW_PLAT_L, 0.0, F_HALF, F_SEESAW_H, 0.0, LEGO_SURF, LEGO_GREY),
        box("platform_goal", F_SEESAW_GAP, F_SEESAW_GAP + F_SEESAW_PLAT_L, F_HALF, F_SEESAW_H, 0.0, LEGO_SURF, LEGO_GREY),
        box("ledge_near", l0, l1, hw, F_SEESAW_LEDGE, 0.0, LEGO_SURF, LEGO_GREY),
        box("ledge_far", _r6(F_SEESAW_GAP - l1), _r6(F_SEESAW_GAP - l0), hw, F_SEESAW_LEDGE, 0.0, LEGO_SURF, LEGO_GREY),
        box("pivot_left", xp - LEGO_STUD, xp + LEGO_STUD, LEGO_STUD, col_top, 0.0, LEGO_SURF, LEGO_GREY, y0=0.048),
        box("pivot_right", xp - LEGO_STUD, xp + LEGO_STUD, LEGO_STUD, col_top, 0.0, LEGO_SURF, LEGO_GREY, y0=-0.048),
        plank,
    ]
    ob = Obstacle(
        "seesaw",
        level,
        {"tilt_deg": g["tilt_deg"], "tilt_nominal_deg": spec, "plank_width_mm": F_SEESAW_W * 1000,
         "plank_length_m": F_SEESAW_L, "plank_g": F_SEESAW_PLANK_G, "platform_gap_mm": F_SEESAW_GAP * 1000,
         "platform_mm": F_SEESAW_H * 1000, "axle_column_plates": g["axle_column_plates"],
         "axle_z_mm": round(za * 1000, 2), "ledge_mm": round(F_SEESAW_LEDGE * 1000, 1),
         "ledge_x_mm": (round(l0 * 1000, 1), round(l1 * 1000, 1)), "end_step_mm": g["end_step_mm"],
         "end_gap_mm": g["end_gap_mm"],
         **({"start_pose": "far end down on the far ledge, near end raised",
             "near_end_top_mm": round(near_top * 1000, 1),
             "near_end_above_platform_mm": round((near_top - F_SEESAW_H) * 1000, 1)} if flip else
            {"start_pose": "near end down on the near ledge"})},
        (f"printed seesaw plank 64 x 240 mm on a LEGO axle {za * 1000:.1f} mm above the floor, tilted {g['tilt_deg']} "
         f"deg with its far end down: its near end is raised {(near_top - F_SEESAW_H) * 1000:.0f} mm above the start "
         "platform" if flip else
         f"printed seesaw plank 64 x 240 mm tilted {g['tilt_deg']} deg on a LEGO axle {za * 1000:.1f} mm above the floor"),
        shapes,
        StartSpec(front_x=-0.005, surface_z=F_SEESAW_H),
        GoalSpec(x_min=_r6(F_SEESAW_GAP + 0.01), x_max=_r6(F_SEESAW_GAP + F_SEESAW_PLAT_L), z_min=_r6(F_SEESAW_H - 0.004)),
        fail_z=-0.02,
        time_limit_s=30.0,
        floor_surface=LEGO_SURF,
        lane_half_width=F_HALF,
    )
    ob.joints.append(JointSpec("pivot", "revolute", "", "plank", (xp, 0.0, _r6(za)), axis="Y"))
    return ob


# ---- gapdrop: LEGO decks
F_GAPDROP_GAP = 0.152  # 19 studs
F_GAPDROP_LANDING = 0.504


def final_gapdrop(level: int, spec) -> Obstacle:
    """Cross a 152 mm (19-stud) gap to a LEGO landing deck ``spec`` mm (whole plates) lower; the landing deck is 99.2 mm
    above the baseplate floor."""
    # spec: drop mm, or (drop mm, gap mm) for a measured course (2026-10-23); a plain number is unchanged
    dmm, gmm = (spec[0], spec[1]) if isinstance(spec, (tuple, list)) else (spec, None)
    drop, w = dmm * 1e-3, (gmm * 1e-3 if gmm is not None else F_GAPDROP_GAP)
    floor = _r6(-(drop + F_DECK_H))
    shapes = [
        box("platform_start", -0.60, 0.0, F_HALF, 0.0, floor, LEGO_SURF, LEGO_GREY),
        box("platform_landing", w, w + F_GAPDROP_LANDING, F_HALF, -drop, floor, LEGO_SURF, LEGO_GREY),
    ]
    return Obstacle(
        "gapdrop",
        level,
        {"drop_mm": dmm, "gap_mm": round(w * 1000, 1), "drop_plates": round(drop / LEGO_PLATE),
         "chasm_m": F_DECK_H},
        f"gap {w * 1000:.0f} mm to a LEGO landing deck {dmm} mm lower",
        shapes,
        StartSpec(front_x=-0.03),
        GoalSpec(x_min=_r6(w + 0.005), x_max=_r6(w + F_GAPDROP_LANDING), z_min=_r6(-drop - 0.004), z_max=0.5),
        fail_z=_r6(-drop - 0.05),
        time_limit_s=30.0,
        floor_z=floor,
        lane_half_width=F_HALF,
    )


# ---- wall: LEGO bricks (+ a tile layer that makes the top smooth and lifts it by the floor tiles' 3.2 mm)
F_WALL_THICK = 2 * LEGO_STUD  # one layer of 2x4 bricks laid along the wall: 2 studs (16 mm) thick
F_WALL_HALF = 0.128  # 8 bricks of 2x4 end to end: 32 studs (256 mm, one 10 in baseplate) across; the lane is as wide


def final_wall(level: int, spec) -> Obstacle:
    """Climb over a LEGO wall ``spec`` bricks high above the floor: one layer of 2x4 bricks (2 studs = 16 mm thick),
    8 bricks (32 studs = 256 mm) across, the lane exactly as wide, and come down on the floor beyond."""
    n = spec
    h = _r6(n * LEGO_BRICK)
    t = F_WALL_THICK
    return Obstacle(
        "wall",
        level,
        {"bricks": n, "height_mm": round(n * 9.6, 1), "thickness_mm": round(t * 1000, 1), "width_mm": round(2 * F_WALL_HALF * 1000, 1),
         "landing_m": WALL_LANDING_M},
        f"climb over a LEGO wall {n} bricks = {n * 9.6:.1f} mm high, one 2x4 brick ({t * 1000:.0f} mm) thick, "
        f"{2 * F_WALL_HALF * 1000:.0f} mm wide",
        [box("wall", 0.0, t, F_WALL_HALF, h, 0.0, LEGO_SURF, LEGO_GREY)],
        StartSpec(front_x=-0.03),
        GoalSpec(x_min=round(t + 0.01, 4), x_max=round(t + WALL_LANDING_M, 4), z_min=-0.004),
        fail_z=-0.05,
        time_limit_s=round(30.0 + 100.0 * h),
        floor_surface=LEGO_SURF,
        lane_half_width=F_WALL_HALF,
    )


# ---- uturn: LEGO lanes, a tilted plate of four 10 in baseplates + tiles on printed wedge adapters
F_UTURN_LANE_LEN = 0.552  # lane length from the start line (car front) to the plate's hinge
# lane length behind the start line: 0.200 since 2026-10-07 (lanes 752 mm = 94 studs, user request; was 0.448 = 1.0 m
# lanes). LEGO_BENCH_UTURN_BACK=0.448 restores the old course.
F_UTURN_BACK = float(os.environ.get("LEGO_BENCH_UTURN_BACK", "0.200"))
F_UTURN_PLATE = 0.512  # 2 x 2 baseplates (64 x 64 studs)
F_UTURN_PLATE_T = 0.0047  # baseplate sheet 1.5 mm + tiles 3.2 mm
F_UTURN_LIP_MIN = 0.0016  # wedge lip thickness along the face (per angle: the plate's top edge clears the lane end)
F_UTURN_LIP_H = 0.003  # lip height above the wedge face (below the plate top)
F_UTURN_WEDGE_H = (0.004, 0.0072)  # wedge face height above the column top at the lip foot (chosen per angle)
F_UTURN_SUPPORT_Y = (-0.096, 0.0, 0.096, 0.192, 0.288, 0.384)  # wedge / column centres across the plate (y, m)


def final_uturn_geometry(tilt: float) -> dict:
    """Plate pose and supports. Row 1: printed wedges (2x2 LEGO-socket underside, face at ``tilt``, lip at the downhill
    end) on 2x2 columns right at the lane ends; the plate's lower edge rests against the lips. The wedge face height
    at the lip foot is chosen per angle so that the plate's top lower corner is level with the lane tops (z = 0); the
    plate starts a hinge gap (< 8 mm) beyond the lane ends. Rows 2-3: plain 2x2 LEGO columns with a 2x2 tile whose
    downhill top edge touches the plate's underside, placed at the grid position / plate count with the smallest
    height residual near 1/2 and near the top of the plate."""
    th = math.radians(tilt)
    s, c = math.sin(th), math.cos(th)
    t, L = F_UTURN_PLATE_T, F_UTURN_LANE_LEN
    lip = max(F_UTURN_LIP_MIN, (0.0004 + t * s) / c)  # the plate's top lower corner ends up ~0.5 mm past the lanes
    xd = L + 0.0001  # wedge downhill end (wedges on 2x2 columns right at the lane ends)
    bx = xd + lip * c  # plate's lower bottom corner (against the lip)
    tx = bx - t * s  # plate's lower top corner
    bz = -t * c
    j = math.floor((bz + F_DECK_H - F_UTURN_WEDGE_H[0]) / LEGO_PLATE)
    zc = -F_DECK_H + j * LEGO_PLATE  # wedge column top
    h_lip = bz - zc
    rows = [{"kind": "wedge", "x0": _r6(xd), "top_m": _r6(zc), "plates": j, "residual_mm": 0.0}]
    for s_target in (0.5 * F_UTURN_PLATE, F_UTURN_PLATE - 0.040):
        xt = bx + s_target * c
        best = None
        for m in range(-10, 11):
            x0 = L + 0.0001 + (round((xt - L) / LEGO_STUD) + m) * LEGO_STUD
            if x0 + 2 * LEGO_STUD > bx + (F_UTURN_PLATE - 0.004) * c or x0 < bx + 0.03:
                continue
            zu = bz + (x0 - bx) * s / c  # plate underside above the column's downhill top edge
            jj = round((zu + F_DECK_H) / LEGO_PLATE)
            res = -F_DECK_H + jj * LEGO_PLATE - zu
            if best is None or abs(res) < abs(best[2]):
                best = (x0, jj, res)
        x0, jj, res = best
        rows.append({"kind": "column", "x0": _r6(x0), "top_m": _r6(-F_DECK_H + jj * LEGO_PLATE), "plates": jj,
                     "residual_mm": round(res * 1000, 2)})
    return {"tilt_rad": th, "hinge_x_m": _r6(tx), "hinge_gap_mm": round((tx - L) * 1000, 2),
            "plate_bottom_corner_m": (_r6(bx), _r6(bz)), "wedge_h_lip_mm": round(h_lip * 1000, 3),
            "wedge_x0_m": _r6(xd), "wedge_lip_mm": round(lip * 1000, 3), "rows": rows}


def final_uturn_plate_dy(lane_mm: float) -> float:
    """Sideways offset of the U-turn plate's centre line from the turn centre y = R (m). The printed pinned angle caps
    (``real_world_kit/print_final/baseplate_adapters``, ``uturn_cap_<a>deg``) pin the plate's underside hole grid to
    the floor's stud grid. The plate is 64 studs wide, so its holes lie 4 mm off its centre line; the floor's studs lie
    on y = 0 mod 8 mm for an odd-stud lane (25 / 21 / 19 studs: L1, L4, L5) and on 4 mod 8 mm for an even one. Odd
    lanes therefore shift the plate by half a stud, +4 mm (centre y = 156 mm), the same way as the support columns
    (``yc + LEGO_STUD / 2`` below): the plate's seam joiners then stay centred in the 80 mm gaps between the columns
    (plate y = -200, -104, +88, +184 mm from its centre line). Even lanes (L2, L3): no shift."""
    return LEGO_STUD / 2 if round(lane_mm / 8) % 2 else 0.0


def final_uturn(level: int, spec) -> Obstacle:
    """U-turn (``uturn``) on LEGO: lanes are LEGO decks 99.2 mm above the baseplate floor, the connector plate is four
    10 in baseplates with tiles (512 x 512 x 4.7 mm) resting on printed wedge adapters (``final_uturn_geometry``)."""
    tilt, lane_mm, gap_mm = spec
    g = final_uturn_geometry(tilt)
    th = g["tilt_rad"]
    hw, gap, L, back = lane_mm * 1e-3 / 2, gap_mm * 1e-3, F_UTURN_LANE_LEN, F_UTURN_BACK
    R = hw + gap / 2
    P = F_UTURN_PLATE
    tx = g["hinge_x_m"]
    high = P * math.sin(th)
    fail = -0.05
    floor = -F_DECK_H
    yb = 2 * R
    shapes = [
        box("lane_a", -back, L, hw, 0.0, floor, LEGO_SURF, LEGO_GREY),
        box("lane_b", -back, L, hw, 0.0, floor, LEGO_SURF, LEGO_GREY, y0=yb),
    ]
    pl = slab("plate", (tx, 0.0), (tx + P * math.cos(th), high), P / 2, F_UTURN_PLATE_T, LEGO_SURF)
    plate_dy = final_uturn_plate_dy(lane_mm)
    pl.pos = (pl.pos[0], _r6(R + plate_dy), pl.pos[2])
    pl.color = BLUE
    shapes.append(pl)
    bx, bz = g["plate_bottom_corner_m"]
    for r_i, row in enumerate(g["rows"]):
        x0 = row["x0"]
        if row["kind"] == "wedge":  # the wedge as a box up to its face height at the downhill end (under the plate)
            ztop = row["top_m"] + (g["wedge_h_lip_mm"] - g["wedge_lip_mm"] * math.sin(th)) * 1e-3
        else:
            ztop = row["top_m"]
        for k, yc in enumerate(F_UTURN_SUPPORT_Y):
            yc = _r6(yc + (LEGO_STUD / 2 if round(lane_mm / 8) % 2 else 0.0))  # 2x2 columns centred on grid lines
            shapes.append(box(f"support{r_i}_{k}", x0, _r6(x0 + 2 * LEGO_STUD - 0.0002), LEGO_STUD - 0.0001,
                              _r6(ztop), floor, LEGO_SURF, PRINT_COLOR if row["kind"] == "wedge" else LEGO_GREY, y0=yc))
    pts = [(-back, 0.0)]
    k = 60
    for j in range(k + 1):
        a = math.pi * j / k
        pts.append((round(L + R * math.sin(a), 6), round(R - R * math.cos(a), 6)))
    pts.append((-back, yb))
    seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:], strict=True)]
    s_arc_end = sum(seg[:-1])
    goal_s0 = s_arc_end + (L - 0.10)
    s_total = sum(seg)
    path = goal_s0 - back
    return Obstacle(
        "uturn",
        level,
        {"tilt_deg": tilt, "lane_width_mm": round(2 * hw * 1000), "gap_mm": round(gap * 1000), "lane_m": L,
         "turn_radius_mm": round(R * 1000), "plate_m": P, "plate_mm": (P * 1000, P * 1000, F_UTURN_PLATE_T * 1000),
         "plate_rise_mm": round(high * 1000, 1), "path_m": round(path, 4), "hinge_gap_mm": g["hinge_gap_mm"],
         "wedge_h_lip_mm": g["wedge_h_lip_mm"], "wedge_lip_mm": g["wedge_lip_mm"], "turn_margin_mm": round((P * math.cos(th) + tx - L - R) * 1000),
         "plate_y_mm": round((R + plate_dy) * 1000, 1),
         "supports": [{k: v for k, v in r.items()} for r in g["rows"]]},
        f"U-turn on a LEGO plate (4 baseplates) tilted {tilt} deg up (turn radius {R * 1000:.0f} mm) between two "
        f"{2 * hw * 1000:.0f} mm LEGO lanes {gap * 1000:.0f} mm apart",
        shapes,
        StartSpec(front_x=0.0),
        GoalSpec(x_min=round(s_arc_end, 6),  # (user 2026-10-25) anywhere on the second lane (was its last 10 cm)
        x_max=round(s_total, 6), y_half=hw + 0.02, z_min=-0.004),
        fail_z=round(fail, 5),
        time_limit_s=round(30.0 + 25.0 * path),
        floor_z=round(floor, 5),
        lane_half_width=R,
        stuck_window_s=6.0,
        centreline=pts,
    )


FINAL_LEVELS: dict[str, list] = {
    "ramp": [(a, "rubber_mat") for a in (30, 38, 46, 54, 62)],  # nominal slope deg (actual: final_ramp_geometry)
    "gap": [152, 224, 336, 504, 752],  # gap mm (19, 28, 42, 63, 94 studs)
    "bridge": [40, 32, 24, 16, 8],  # printed plank width mm (5 .. 1 studs), 640 mm long
    # span m, nominal incline deg, sag mm, r mm; span halved 1.2 -> ~0.6 m and sag 25 -> 12.5 mm (user, 2026-10-12).
    # Exact 0/5/10/15/20 deg (user): the goal deck rises a whole number of plates (the nearest to 600 mm x tan) and the
    # span is that rise / tan(incline), so the tables stand 585-600 mm apart
    "rope": [(0.6, 0.0, 12.5, 1.5), (0.5852, 5.0, 12.5, 1.5), (0.5989, 10.0, 12.5, 1.5), (0.5971, 15.0, 12.5, 1.5),
             (0.5979, 20.0, 12.5, 1.5)],
    "steps": [(28.8, 3), (38.4, 3), (57.6, 3), (76.8, 3), (105.6, 3)],  # riser mm (3, 4, 6, 8, 11 bricks), count
    "bumps": [(35.0, 10), (45.0, 12), (55.0, 14), (65.0, 16), (80.0, 18)],  # max rod mm, rod count
    # nominal tilt deg (actual: final_seesaw_geometry); negative: tipped the other way, near end raised (E0 v4 extension,
    # 2026-10-16, user: "up the other way around"; v4 had [15, 22, 30, 38, 46], near end down)
    "seesaw": [-15.0, -22.0, -30.0, -38.0, -46.0],
    "gapdrop": [48.0, 80.0, 128.0, 208.0, 336.0],  # drop mm (15, 25, 40, 65, 105 plates) over a 152 mm gap
    "wall": [4, 8, 16, 32, 64],  # bricks (32 mm thick LEGO wall: climb over, land beyond)
    # tilt deg, lane mm, gap mm. Tilts since 2026-10-10: the printed stand's foot rails moved to the end of the
    # baseplates under the plate (220 mm leg vertical in the top hole, the original rail's hole spacing); were 10-35.
    "uturn": [(32.45, 200, 104), (33.94, 192, 112), (35.46, 176, 128), (36.9, 168, 136), (37.97, 152, 152)],
}
"""The final ladder (``ladder="final"``): 10 benches x 5 levels, each level exactly what its simple real build
produces (LEGO baseplates, columns and decks; printed only: bridge plank segments, rope saddles and anchors, bump
plates, the seesaw plank, U-turn wedges: ``real_world_kit/print_final``). Run with the redo profile and part set
``final_pf`` (``capped_pf`` before 2026-10-04 14:40). Turns, tunnel, tug and speed are not part of it. ``results/lego_bench/final_ladder.json``
(``scripts/export_lego_final_ladder.py``) exports every level's shapes for CAD."""

FINAL_GENERATORS = {"ramp": final_ramp, "gap": final_gap, "bridge": final_bridge, "rope": final_rope,
                    "steps": final_steps, "bumps": final_bumps, "seesaw": final_seesaw, "gapdrop": final_gapdrop,
                    "wall": final_wall, "uturn": final_uturn}

LADDERS = ("base", "redo", "final6", "final")

FINAL_CHASM_KINDS = ("gap", "bridge", "rope", "gapdrop", "uturn", "seesaw")  # seesaw added 2026-10-24 (user: cross on the plank, never the floor)
FINAL_OVER_KINDS = ("wall", "bumps")
"""Final benches that must be crossed over the top: a run fails ``left_lane`` as soon as any part's origin leaves the
lane (the wall spans the whole lane, so the only way past it is over; on the bumps the car may not skirt the plates)."""
"""Final-ladder benches whose travel surface is raised over a chasm (the ground plane at ``floor_z`` is the chasm floor:
the baseplate sheet under the decks, the table under the rope). On the ``final`` ladder any part of the design touching
it fails the run ("touched chasm floor (<part>)"); ``generate`` marks these levels with ``params["chasm_floor_z"]``.
The base / redo / final6 ladders are unchanged."""


def n_levels(kind: str, ladder: str = "base") -> int:
    if ladder in ("final", "final6"):
        table = FINAL_LEVELS if ladder == "final" else FINAL6_LEVELS
        return len(table[kind]) if kind in table else 0
    return len(REDO_LEVELS[kind]) if ladder == "redo" else OBSTACLE_KINDS[kind][1]


def generate(kind: str, level: int, ladder: str = "base", spec=None) -> Obstacle:
    """``spec`` (final ladder only, optional): build the level from this spec instead of ``FINAL_LEVELS`` (a measured
    real course, e.g. a filmed run); the level number is kept as a label."""
    if kind not in OBSTACLE_KINDS:
        raise KeyError(f"unknown obstacle '{kind}' ({', '.join(OBSTACLE_KINDS)})")
    if ladder not in LADDERS:
        raise KeyError(f"unknown ladder '{ladder}' ({', '.join(LADDERS)})")
    fn, _ = OBSTACLE_KINDS[kind]
    n = n_levels(kind, ladder)
    if ladder in ("final", "final6") and n == 0:
        raise KeyError(f"'{kind}' is not in the {ladder} ladder ({', '.join(FINAL_LEVELS)})")
    if not 1 <= level <= n:
        raise ValueError(f"{kind}: level must be 1..{n} ({ladder} ladder)")
    if ladder == "base":
        return fn(level)
    if ladder == "final":
        ob = FINAL_GENERATORS[kind](level, FINAL_LEVELS[kind][level - 1] if spec is None
                                    else (tuple(spec) if isinstance(spec, list) else spec))
        if spec is not None:
            ob.params["custom_spec"] = list(spec) if isinstance(spec, (list, tuple)) else spec
        if kind in FINAL_CHASM_KINDS:  # any part touching the chasm floor fails (judge: "touched chasm floor")
            ob.params["chasm_floor_z"] = ob.floor_z
        if not ob.centreline:  # (user 2026-10-28) the goal is anywhere past the obstacle: no far end, no side limit
            ob.goal = dataclasses.replace(ob.goal, x_max=INF, y_half=INF)
        if kind in FINAL_OVER_KINDS:  # over, not around (user 2026-10-13): every part origin stays in the lane
            # (user 2026-10-25) on the bumps a part may step slightly outside the plates (a wheel's width, 20 mm)
            ob.params["lane_parts_half_width_m"] = ob.lane_half_width + (F_BUMP_PART_MARGIN if kind == "bumps" else 0.0)
        board = start_board_mm(kind)
        if board is not None:
            ob = extend_start_board(ob, board)
    else:
        ob = fn(level, spec=(FINAL6_LEVELS if ladder == "final6" else REDO_LEVELS)[kind][level - 1])
    ob.params["ladder"] = ladder
    ob.description = f"[{ladder} L{level}] {ob.description}"
    return ob


# ----------------------------------------------------------------------------------------------------------------
# placement
# ----------------------------------------------------------------------------------------------------------------


@dataclass
class VehiclePose:
    origin: tuple[float, float, float]  # world position of the design origin [m]
    quat: tuple[float, float, float, float]  # world orientation of the design frame (xyzw)
    parts: dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]]  # name -> (pos, quat)
    rear_x: float
    front_x: float


def _q_pitch_design(pitch_deg: float):
    return pitch_quat(pitch_deg)


def _rot_pitch(pitch_deg: float) -> np.ndarray:
    return rot_matrix(0.0, -pitch_deg)


def place_vehicle(ob: Obstacle, design: Design, stats: dict, lift: float = 0.0005) -> VehiclePose:
    """World poses of all parts at the start line (design frame -> env frame)."""
    from .design import qmul

    P = place(design)
    ob = with_start_gap(ob, design)
    R = _rot_pitch(ob.start.pitch_deg)
    corners = []
    boxes = [pl.aabb() for pl in P.values()]
    if getattr(design, "treads", None):  # a tread's envelope reaches past its sprockets
        from .treads import tread_geoms  # noqa: PLC0415

        boxes += [g.bbox() for g in tread_geoms(design, P)]
    for lo, hi in boxes:
        corners += [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    c = (R @ (np.array(corners) * 1e-3).T).T
    if ob.start.align in ("rope", "rope_track"):
        y_ref = stats.get("rope_line_y_mm", stats["capability"].get("rope_wheel_line_y_mm", stats["track_center_y_mm"]))
    else:
        y_ref = stats["track_center_y_mm"]
    # wheel contact reference: the rear-most ground wheel bottom
    wheels = [P[n] for n in stats["wheels"]] if stats["wheels"] else list(P.values())
    rw = min(wheels, key=lambda w: w.p[0])
    contact = R @ (np.array([rw.p[0], y_ref, stats["ground_z_mm"]]) * 1e-3)
    ox = (ob.start.front_x - c[:, 0].max()) if ob.start.front_x is not None else (ob.start.rear_x - c[:, 0].min())
    cx = ox + contact[0]
    surf = ob.start.surface_z + (cx - ob.start.surface_x) * math.tan(math.radians(ob.start.pitch_deg))
    oz = surf + lift - contact[2]
    oy = ob.start.y - contact[1]
    origin = np.array([ox, oy, oz])
    q0 = _q_pitch_design(ob.start.pitch_deg)
    parts = {}
    for name, pl in P.items():
        pos = origin + R @ (pl.p * 1e-3)
        parts[name] = (tuple(float(v) for v in pos), qmul(q0, quat_xyzw(pl.inst.yaw_deg, pl.inst.spin_deg)))
    return VehiclePose(tuple(float(v) for v in origin), q0, parts, float(ox + c[:, 0].min()), float(ox + c[:, 0].max()))


APPROACH_BOARD_MM = 500.0
"""Flat board before every obstacle on the real benches (user, 2026-10-22: about 50 cm everywhere): the whole vehicle
must start on it."""


def with_start_gap(ob: Obstacle, design: Design) -> Obstacle:
    """the obstacle with its start line moved to the design's start_gap_mm (front of the vehicle at x = -gap)"""
    gap = getattr(design, "start_gap_mm", None)
    if gap is None or ob.start.front_x is None:
        return ob
    return dataclasses.replace(ob, start=dataclasses.replace(ob.start, front_x=-float(gap) * 1e-3))


START_BOARD_ENV = "LEGO_START_BOARD_MM"
"""(2026-11-03, user: "all could be 500 mm long, for gap could be 1 m") optional start-board length per final bench,
e.g. '{"*": 500, "gap": 1000}' (the all-benches task): the raised start (platform_start / the U-turn's start lane) is
lengthened backwards to it (its front edge stays at the start line) and the vehicle must fit on that board. Unset: the
courses as built (bridge 456 mm, seesaw 400 mm, U-turn 200 mm)."""


def start_board_mm(kind: str) -> float | None:
    raw = os.environ.get(START_BOARD_ENV, "").strip()
    if not raw:
        return None
    import json  # noqa: PLC0415

    v = json.loads(raw)
    v = v if isinstance(v, dict) else {"*": v}
    x = v.get(kind, v.get("*"))
    return None if x is None else float(x)


def extend_start_board(ob: Obstacle, board_mm: float) -> Obstacle:
    """the start surface lengthened backwards to ``board_mm`` behind the start line (never shortened)"""
    L = board_mm / 1000.0
    shapes = []
    for sh in ob.shapes:
        if sh.kind == "box" and (sh.name == "platform_start" or (ob.kind == "uturn" and sh.name == "lane_a")):
            x1 = sh.pos[0] + sh.size[0] / 2
            x0 = min(sh.pos[0] - sh.size[0] / 2, x1 - L) if x1 <= 1e-6 else min(sh.pos[0] - sh.size[0] / 2, -L)
            sh = dataclasses.replace(sh, size=(x1 - x0, *sh.size[1:]), pos=((x0 + x1) / 2, *sh.pos[1:]))
        shapes.append(sh)
    ob = dataclasses.replace(ob, shapes=shapes)
    ob.params["start_board_mm"] = float(board_mm)
    return ob


def approach_mm(ob: Obstacle) -> float:
    """length of flat start surface before the obstacle's start line (x = 0): the 500 mm board, or a shorter start
    table (bridge 456 mm, seesaw 400 mm); a start board set by LEGO_START_BOARD_MM (start_board_mm) wins"""
    if ob.params.get("start_board_mm") is not None:
        return float(ob.params["start_board_mm"])
    for sh in ob.shapes:
        if sh.name == "lane_a" and sh.kind == "box" and ob.kind == "uturn":  # (2026-10-28) the raised start lane
            return min(APPROACH_BOARD_MM, round((sh.size[0] / 2 - sh.pos[0]) * 1000.0, 1))
        if sh.name == "platform_start" and sh.kind == "box":
            return min(APPROACH_BOARD_MM, round((sh.size[0] / 2 - sh.pos[0]) * 1000.0, 1))
    return APPROACH_BOARD_MM


def start_gap_issue(ob: Obstacle, design: Design, length_mm: float) -> str | None:
    """A chosen start gap (front of the vehicle to the obstacle's start line, x = 0) must leave the whole vehicle on
    the approach board: 0 <= gap and gap + length <= approach_mm(ob)."""
    gap = getattr(design, "start_gap_mm", None)
    if gap is None:
        # (sim v13) no start gap chosen: the level's default start. On a raised start (a start table / platform over a
        # chasm: gap, bridge, seesaw, rope, gapdrop, ...) the whole vehicle must still fit on it
        if ob.start.front_x is None or not any(sh.name == "platform_start" for sh in ob.shapes):
            return None
        g0 = max(0.0, -float(ob.start.front_x) * 1000.0)
        room = approach_mm(ob)
        if g0 + length_mm > room:
            return (f"with the default start (front {g0:.0f} mm from the start line) the vehicle's length {length_mm:.0f} mm "
                    f"does not fit on the {room:.0f} mm start board: it is {length_mm + g0 - room:.0f} mm too long")
        return None
    if ob.start.front_x is None:
        return f"start_gap_mm is not available on the {ob.kind} bench"
    if gap < 0:
        return f"start_gap_mm {gap:g} < 0: the vehicle would start on the obstacle"
    room = approach_mm(ob)
    if gap + length_mm > room:
        return (f"start_gap_mm {gap:g} + vehicle length {length_mm:.0f} mm > the {room:.0f} mm approach board: the "
                f"vehicle would not fit on it")
    return None


CONSTRAINT_KEYS = ("max_length_mm", "max_mass_g", "max_drive_motors", "max_wheel_diameter_mm", "sensors_only",
                   "payload", "max_parts")  # fmt: skip


def redo_constraints(max_parts: int | None = None) -> dict:
    """Constraints of the constrained redo campaign, on top of every level's own: exactly one motor may drive wheels
    (motors that only turn legs, arms, masses or tails are free, as ``wheel_motors`` counts them) and an optional cap on
    the total number of pieces (every ``parts`` entry: pins, axles and bushes included)."""
    # LEGO_BENCH_MAX_DRIVE_MOTORS: "none" lifts the one-drive-motor rule (the 2026-10-14 benchmark, v4), a number sets it
    lim = os.environ.get("LEGO_BENCH_MAX_DRIVE_MOTORS", "1")
    out = {} if lim.lower() == "none" else {"max_drive_motors": int(lim)}
    if max_parts is not None:
        out["max_parts"] = int(max_parts)
    return out


def check_constraints(ob: Obstacle, design: Design, stats: dict, extra: dict | None = None) -> list[str]:
    """Violations of a tier's constraints (checked before the run; a violating design is not simulated). ``extra``:
    campaign constraints merged in (the stricter value wins for the numeric caps), e.g. :func:`redo_constraints`."""
    from .design import wheel_motors  # noqa: PLC0415
    from .parts import bench_parts  # noqa: PLC0415

    c, errs = dict(ob.constraints), []
    for k, v in (extra or {}).items():
        c[k] = min(c[k], v) if (k in c and isinstance(v, (int, float)) and not isinstance(v, bool)) else v
    if "max_parts" in c and stats["n_parts"] > c["max_parts"]:
        errs.append(f"constraint: {stats['n_parts']} pieces > {c['max_parts']}")
    if "max_length_mm" in c and stats["length_mm"] > c["max_length_mm"] + 1e-6:
        errs.append(f"constraint: {stats['length_mm']} mm long at the start > {c['max_length_mm']} mm")
    if "max_mass_g" in c and stats["mass_g"] > c["max_mass_g"] + 1e-6:
        errs.append(f"constraint: mass {stats['mass_g']} g > {c['max_mass_g']} g")
    if "max_drive_motors" in c and len(wheel_motors(design)) > c["max_drive_motors"]:
        errs.append(f"constraint: {len(wheel_motors(design))} drive motors > {c['max_drive_motors']}")
    if "max_wheel_diameter_mm" in c:
        cat = bench_parts()
        tr_r = {}
        if getattr(design, "treads", None):  # a tread's sprocket counts with the tread round it
            from .treads import tread_wheel_radii  # noqa: PLC0415

            tr_r = {n: v[0] for n, v in tread_wheel_radii(design, place(design)).items()}
        for p in design.parts:
            bp = cat.get(p.part)
            r = tr_r.get(p.name, bp.wheel_radius if bp is not None else 0.0)
            if bp is not None and (bp.kind == "wheel" or p.name in tr_r) and 2 * r > c["max_wheel_diameter_mm"] + 1e-6:
                errs.append(f"constraint: wheel '{p.name}' is {2 * r:.1f} mm > {c['max_wheel_diameter_mm']} mm")
    if c.get("sensors_only") and any("x" in cmd for cmd in design.motor_program):
        errs.append("constraint: no position ('x') triggers: the program must use sensors or time")
    if c.get("payload") and design.cargo_mm is None:
        errs.append("constraint: the design must declare a cargo point (cargo_mm) for the payload")
    return errs


def finalize(ob: Obstacle, design: Design, stats: dict, pose: VehiclePose) -> Obstacle:
    """Resolve vehicle-dependent geometry: the tug sled behind the vehicle and the tow-line anchors; a tier payload on
    the design's cargo point."""
    if ob.payload and design.cargo_mm is not None:
        sx, sy, sz = (v * 1e-3 for v in ob.payload["size_mm"])
        R = _rot_pitch(ob.start.pitch_deg)
        c = np.array(pose.origin) + R @ (np.array(design.cargo_mm) * 1e-3)
        ob.shapes = [sh for sh in ob.shapes if sh.name != "payload"] + [
            Shape(
                "payload",
                "box",
                (sx, sy, sz),
                (float(c[0]), float(c[1]), float(c[2] + sz / 2 + 0.0005)),
                surface=ob.payload.get("surface", "steel"),
                color=(0.55, 0.57, 0.62),
                dynamic=True,
                mass_kg=ob.payload["mass_g"] * 1e-3,
            )
        ]
        return ob
    if ob.kind != "tug":
        return ob
    sled = next(s for s in ob.shapes if s.name == "sled")
    sx = sled.size[0]
    tow = design.tow_point_mm
    if tow is None:
        tow = (stats["rear_x_mm"], stats["track_center_y_mm"], stats["ground_z_mm"] + 12.0)
    R = _rot_pitch(ob.start.pitch_deg)
    tow_w = np.array(pose.origin) + R @ (np.array(tow) * 1e-3)
    sled.pos = (float(min(tow_w[0], pose.rear_x) - ob.sled_gap_m - sx / 2), float(tow_w[1]), sled.pos[2])
    # a high tow point: the sled starts with the line just short of taut (95 % of its length), not stretched (the
    # distance joint would yank it in at the first step), but at least 5 mm behind the vehicle
    L = ob.tow_line_m or 0.0
    dz = float(tow_w[2] - sled.pos[2])
    dx = float(tow_w[0] - (sled.pos[0] + sx / 2))
    if L and math.hypot(dx, dz) > L and abs(dz) < 0.95 * L:
        front = max(float(tow_w[0]) - math.sqrt((0.95 * L) ** 2 - dz**2), sled.pos[0] + sx / 2)
        front = min(front, float(pose.rear_x) - 0.005)
        sled.pos = (front - sx / 2, sled.pos[1], sled.pos[2])
    j = ob.joints[0]
    j.anchor = tuple(float(v) for v in tow_w)
    j.anchor1 = (sled.pos[0] + sx / 2, sled.pos[1], sled.pos[2])
    return ob


# ----------------------------------------------------------------------------------------------------------------
# judge
# ----------------------------------------------------------------------------------------------------------------


def _qrot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def _qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def mechanism_groups(design: Design) -> list[tuple[str, set[str]]]:
    """Rigid (stud / pin) clusters built on a Technic brick with axle hole (32064) that turns with an axle: arms,
    booms, legs, as (anchor 32064, members). They move relative to the chassis by design, but not relative to their
    own anchor: a member that leaves its anchor has come off. Inventory designs: clusters on a pivot pin, liftarms
    locked on a turning axle, and clusters joined to the rest only through a hinge (their hole turning on an axle,
    :func:`_hinge_anchors`) are mechanisms too."""
    from .design import RIGID_KINDS, _components  # noqa: PLC0415
    from .parts import bench_parts  # noqa: PLC0415

    cat = bench_parts()
    names = [p.name for p in design.parts]
    kinds = {p.name: cat[p.part].kind for p in design.parts if p.part in cat}
    out = []
    inventory = any(cat[p.part].inventory for p in design.parts if p.part in cat)
    if inventory:  # inventory parts: liftarms / connectors locked on an axle (cross) and pivot-pin linkages move too
        from .design import pivot_connections, rigid_edges  # noqa: PLC0415

        piv = pivot_connections(design)
        movers = {c.child for c in design.connections if c.kind == "cross" and kinds.get(c.child) != "axle"}
        movers |= {design.connections[i].parent for i in piv} | {design.connections[i].child for i in piv}
        edges = rigid_edges(design)
    else:
        movers, edges = set(), [(c.child, c.parent) for c in design.connections if c.kind in RIGID_KINDS]
    comps = _components(names, edges)
    anchored: set[str] = set()  # inventory: clusters that are mechanisms of their own (a link on a pivot pin)
    if inventory:
        for comp in comps:
            if design.chassis not in comp and any(
                    kinds.get(n) == "axle_brick" or (n in movers and kinds.get(n) not in ("gear", "wheel", "bush"))
                    for n in comp):
                anchored |= set(comp)
    hinged = _hinge_anchors(design, comps, kinds) if inventory else {}
    for comp in comps:
        if design.chassis not in comp and frozenset(comp) in hinged and not (set(comp) & anchored):
            anchored |= set(comp)
    for comp in comps:
        anchors = sorted(n for n in comp if kinds.get(n) == "axle_brick")
        if inventory and not anchors:
            anchors = sorted(n for n in comp if n in movers and kinds.get(n) not in ("gear", "wheel", "bush"))
        if inventory and not anchors and design.chassis not in comp:
            anchors = hinged.get(frozenset(comp), [])
        if design.chassis not in comp and anchors:
            # parts that hang on the cluster alone (a free wheel on an axle in the arm's tip hole) move with it
            rest = [n for n in names if n not in comp and n not in anchored]
            for sub in _components(rest, [(c.child, c.parent) for c in design.connections
                                          if c.child not in comp and c.parent not in comp]):  # fmt: skip
                touches = any((c.child in sub and c.parent in comp) or (c.parent in sub and c.child in comp)
                               for c in design.connections)  # fmt: skip
                if touches and design.chassis not in sub:
                    comp = set(comp) | set(sub)
            out.append((anchors[0], set(comp)))
    for d in getattr(design, "differentials", None) or []:  # round 4b: the satellites turn with the case
        if d.case in names:
            out.append((d.case, {d.case, *(s for s in d.satellites if s in names)}))
    if getattr(design, "racks", None):  # rack slides (lego_bench.racks): the side without the chassis slides
        from .racks import moving_groups  # noqa: PLC0415

        have = set().union(*[m for _, m in out]) if out else set()
        for anchor, members in moving_groups(design):
            rest = (set(members) - have) | {anchor}
            out.append((anchor, rest))
            have |= rest
    return out


NOT_HINGE_ANCHOR = ("gear", "wheel", "bush")  # free-turning gears / wheels / bushes on an axle stay chassis-checked


def _hinge_anchors(design: Design, comps, kinds) -> dict[frozenset, list[str]]:
    """Inventory designs: rigid clusters (without the chassis) joined to another cluster through a 'hinge' connection
    (an axle turning in a round hole of the cluster) swing by design: {cluster: [its hinge-hole members, sorted]}.
    Free gears / wheels / bushes spinning on an axle are left out (they turn in place)."""
    of = {n: frozenset(c) for c in comps for n in c}
    out: dict[frozenset, set[str]] = {}
    for c in design.connections:
        if c.kind != "hinge" or c.child not in of or c.parent not in of or of[c.child] == of[c.parent]:
            continue
        n = c.parent  # the part with the hole (an axle's own cluster: cross-locked liftarms are movers already)
        if design.chassis not in of[n] and kinds.get(n) not in NOT_HINGE_ANCHOR:
            out.setdefault(of[n], set()).add(n)
    return {k: sorted(v) for k, v in out.items()}


def hinge_links(design: Design) -> list[tuple[str, str]]:
    """(anchor, partner) of every hinge-joined mechanism anchor and the part across its hinge: an axle turning in a
    hole keeps its distance to the hole's part (the axle origin is on the turning axis), so a change of that distance
    beyond the break limit means the hinge came apart."""
    anchors = {a for a, _ in mechanism_groups(design)}
    out = []
    for c in design.connections:
        if c.kind == "hinge":
            if c.child in anchors:
                out.append((c.child, c.parent))
            elif c.parent in anchors:
                out.append((c.parent, c.child))
    return out


def mechanism_parts(design: Design) -> set[str]:
    """All parts of :func:`mechanism_groups` clusters."""
    return set().union(*[m for _, m in mechanism_groups(design)]) if mechanism_groups(design) else set()


FINAL_SETTLE_S = 5.0  # final ladder: seconds the vehicle must stay in the goal, upright, after reaching it


class Judge:
    """Decides success / failure from part poses (call :meth:`update` every few milliseconds while driving)."""

    FLIP_DEG = 75.0
    FLIP_HOLD_S = 0.5
    BREAK_MM = 15.0
    # final ladder: after the vehicle first reaches the goal it must stay there, upright (chassis within UPRIGHT_DEG
    # of its start attitude, a longer tilt than UPRIGHT_HOLD_S fails) and unbroken, for settle_s seconds; the run's
    # time is the moment it reached the goal (the settle is not counted). LEGO_BENCH_SETTLE_S overrides (0 = off).
    UPRIGHT_DEG = 30.0
    UPRIGHT_HOLD_S = 0.3

    def __init__(self, ob: Obstacle, design: Design, pose: VehiclePose):
        self.ob = ob
        self.chassis = design.chassis
        from .design import qmul

        self._qmul = qmul
        c_pos, c_q = pose.parts[self.chassis]
        self.q_chassis0_inv = _qconj(c_q)
        self.q_vehicle0 = pose.quat
        # every part's start offset in the chassis frame (break detection); parts of a moving mechanism (a rigid
        # cluster built on a Technic brick with axle hole: arms, booms, legs) are exempt: they move by design
        self.rel0 = {n: np.array(_qrot(_qconj(c_q), np.subtract(p, c_pos))) for n, (p, _) in pose.parts.items()}
        self.mechanism = mechanism_parts(design)
        self.mech_rel0 = {}  # member -> (anchor, start offset in the anchor frame)
        for a, members in mechanism_groups(design):
            a_pos, a_q = pose.parts[a]
            for n in members:
                if n != a and n in pose.parts:
                    self.mech_rel0[n] = (a, np.array(_qrot(_qconj(a_q), np.subtract(pose.parts[n][0], a_pos))))
        self.hinge0 = [(a, b, float(np.linalg.norm(np.subtract(pose.parts[a][0], pose.parts[b][0]))))
                       for a, b in hinge_links(design) if a in pose.parts and b in pose.parts]
        self.cl = None  # curved course: (segment starts, segment vectors), their lengths and start arc lengths
        if ob.centreline:
            P = np.asarray(ob.centreline, dtype=float)
            self.cl = (P[:-1], P[1:] - P[:-1])
            self.cl_len = np.linalg.norm(self.cl[1], axis=1)
            self.cl_s = np.concatenate([[0.0], np.cumsum(self.cl_len)[:-1]])
            self.s_chassis = float(self._project([c_pos])[0][0])
        self.start_ref = self._ref(pose.parts, None, init=True)
        self.goal_dist = max(ob.goal.x_min - self.start_ref, 1e-6)
        self.best = self.start_ref
        self.anchor, self.anchor_t = self.start_ref, 0.0  # stuck detection: last 5 mm progress mark
        self.flip_since = None
        self.t = 0.0
        self.result = None
        self.hold_hist: list = []
        self.chasm_touch = None  # final ladder, raised benches: name of the first part that touched the chasm floor
        env_settle = os.environ.get("LEGO_BENCH_SETTLE_S")
        self.settle_s = float(env_settle) if env_settle not in (None, "") else (
            FINAL_SETTLE_S if ob.params.get("ladder") == "final" else 0.0)
        self.cross_t = None  # when the vehicle first reached the goal (settle phase running)
        self.tilt_since = None
        # (v12) set by the simulator: "<upper>/<lower>" of the first stud joint released while driving -> "broke"
        self.joint_released = None
        # (sim v13) set by the simulator: a mechanism that came apart while driving (a rack ran off its pinion) -> "broke"
        self.mech_broke = None
        # (v12) bumps: the wheels (tyres, tread sprockets), whose origins over the field must stay within the rods' span
        self.wheels = set()
        if ob.kind == "bumps" and ob.params.get("rods_y_span_m") is not None:
            from .parts import bench_parts  # noqa: PLC0415

            cat = bench_parts()
            self.wheels = {p.name for p in design.parts if p.part in cat and cat[p.part].kind == "wheel"}
            self.wheels |= {s_ for t_ in (getattr(design, "treads", None) or []) for s_ in t_.sprockets}
            self.wheels &= set(pose.parts)

    # windows of the centreline searched around the chassis' last arc length: the chassis itself (it moves < 30 mm per
    # check) and the other parts (a vehicle is < 0.6 m long). A part that climbed over a divider into the next lane
    # is then far from the local centreline (left_lane), not progress along the next lane.
    CL_CHASSIS_WIN = 0.15
    CL_PARTS_WIN = 0.6

    def _project(self, pts, lo: float = -INF, hi: float = INF) -> tuple[np.ndarray, np.ndarray]:
        """Arc length along the centreline and horizontal distance from it of points (x, y, ...), searching the
        segments that overlap the arc-length window [lo, hi]."""
        A, D = self.cl
        sel = np.nonzero((self.cl_s + self.cl_len >= lo) & (self.cl_s <= hi))[0]
        if not len(sel):
            sel = np.arange(len(A))
        p = np.asarray([q[:2] for q in pts], dtype=float)[:, None, :]  # (k, 1, 2)
        a, dv, ln = A[sel][None], D[sel][None], self.cl_len[sel][None]
        u = np.clip(np.sum((p - a) * dv, axis=2) / np.maximum(ln**2, 1e-12), 0.0, 1.0)  # (k, m)
        dist = np.linalg.norm(p - (a + u[..., None] * dv), axis=2)
        i = np.argmin(dist, axis=1)
        k = np.arange(len(p))
        return self.cl_s[sel][i] + u[k, i] * self.cl_len[sel][i], dist[k, i]

    def _ref(self, parts, load_pos, init=False) -> float:
        g = self.ob.goal
        if self.cl is not None:
            if g.ref not in ("rear", "front"):
                raise ValueError("a centreline course supports goal.ref 'rear' / 'front' only")
            s, _ = self._project([p for p, _ in parts.values()], self.s_chassis - self.CL_PARTS_WIN,
                                 self.s_chassis + self.CL_PARTS_WIN)  # fmt: skip
            return float(s.min() if g.ref == "rear" else s.max())
        if g.ref == "load":
            if load_pos is None:
                return next(s.pos[0] for s in self.ob.shapes if s.name == "sled")
            return float(load_pos[0])
        xs = [p[0] for p, _ in parts.values()]
        if g.ref == "half":
            return 0.5 * (min(xs) + max(xs))
        return min(xs) if g.ref == "rear" else max(xs)

    def progress(self) -> float:
        return (self.best - self.start_ref) / self.goal_dist

    def update(self, t: float, parts: dict, load_pos=None) -> dict | None:
        """``parts``: name -> (pos xyz, quat xyzw) in the env frame; ``load_pos``: towed sled position."""
        if self.result is not None:
            return self.result
        self.t = t
        ob, g = self.ob, self.ob.goal
        c_pos, c_q = parts[self.chassis]
        lateral = abs(c_pos[1])
        if self.cl is not None:
            s_c, d_c = self._project([c_pos], self.s_chassis - self.CL_CHASSIS_WIN, self.s_chassis + self.CL_CHASSIS_WIN)
            self.s_chassis, lateral = float(s_c[0]), float(d_c[0])
        ref = self._ref(parts, load_pos)
        self.best = max(self.best, ref)
        if ref > self.anchor + ob.stuck_min_progress_m:
            self.anchor, self.anchor_t = ref, t
        # vehicle up axis: rotation relative to the start pose applied to the start up axis
        q_rel = self._qmul(c_q, self.q_chassis0_inv)
        up = _qrot(self._qmul(q_rel, self.q_vehicle0), (0.0, 0.0, 1.0))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
        if tilt > self.FLIP_DEG:
            self.flip_since = t if self.flip_since is None else self.flip_since
        else:
            self.flip_since = None
        self.tilt_now = tilt
        broken = []
        for n, (p, _) in parts.items():
            if n in self.mechanism:
                continue
            d = np.array(_qrot(_qconj(c_q), np.subtract(p, c_pos)))
            if np.linalg.norm(d - self.rel0[n]) * 1e3 > self.BREAK_MM:
                broken.append(n)
        for n, (a, r0) in self.mech_rel0.items():  # a leg / arm member that came off its 32064
            a_pos, a_q = parts[a]
            d = np.array(_qrot(_qconj(a_q), np.subtract(parts[n][0], a_pos)))
            if np.linalg.norm(d - r0) * 1e3 > self.BREAK_MM:
                broken.append(n)
        for a, b, d0 in self.hinge0:  # a hinge-joined mechanism that came off its axle / hole
            if abs(float(np.linalg.norm(np.subtract(parts[a][0], parts[b][0]))) - d0) * 1e3 > self.BREAK_MM:
                broken.append(a)
        reason = None
        if c_pos[2] < ob.fail_z:
            reason = "fell"
        elif self.chasm_touch is not None and ob.params.get("chasm_floor_z") is not None:
            reason = f"touched chasm floor ({self.chasm_touch})"
        elif self.flip_since is not None and t - self.flip_since >= self.FLIP_HOLD_S:
            reason = "flipped"
        elif self.joint_released is not None:
            reason = f"broke (stud joint released: {self.joint_released})"
        elif getattr(self, "mech_broke", None) is not None:
            reason = f"broke ({self.mech_broke})"
        elif broken:
            reason = f"broke ({', '.join(sorted(broken)[:4])})"
        elif lateral > ob.lane_half_width and (ob.kind != "bumps" or (self.cross_t is None and c_pos[0] <= ob.params.get(
                "field_m", INF))):  # (v12) bumps: the chassis over the plates until it crossed / is past the field
            reason = "left_lane"
        elif (self.cl is None and ob.params.get("lane_parts_half_width_m") is not None
              and max((abs(p[1]) for p, _ in self._lane_parts(parts)), default=0.0) > ob.params["lane_parts_half_width_m"]):
            reason = "left_lane (a part went around the obstacle)"
        elif self._wheel_outside(parts):
            reason = "left_lane (a wheel went around the rods)"
        elif self.cl is not None:  # curved course: the goal region in arc length / distance from the centreline
            pl = [p for p, _ in parts.values()]
            s, d = self._project(pl, self.s_chassis - self.CL_PARTS_WIN, self.s_chassis + self.CL_PARTS_WIN)
            part_hw = ob.params.get("lane_parts_half_width_m")  # painted lanes: every part origin must stay inside
            if part_hw is not None and float(np.max(d)) > part_hw:
                return self._finish(False, "left_lane")
            if g.ref == "front":
                i = int(np.argmax(s))
                pl, s, d = [pl[i]], s[i : i + 1], d[i : i + 1]
            inside = all(g.x_min <= si <= g.x_max and di <= g.y_half and g.z_min <= p[2] <= g.z_max
                         for p, si, di in zip(pl, s, d, strict=True))  # fmt: skip
            if inside:
                return self._reached(t)
            if self.cross_t is not None:
                # (user 2026-10-28) reaching the goal is the pass, even if the vehicle leaves the goal area afterwards
                return self._finish(True, "crossed (left the goal area afterwards)", self.cross_t)
            if t - self.anchor_t > float(os.environ.get("LEGO_BENCH_STUCK_S") or ob.stuck_window_s):  # env: diagnostics
                reason = "stuck"
            elif t >= ob.time_limit_s and self.cross_t is None:
                reason = "timeout"
        else:
            pts = (
                [p for p, _ in parts.values()]
                if g.ref == "rear"
                else ([max((p for p, _ in parts.values()), key=lambda q: q[0])] if g.ref == "front" else [load_pos])
            )
            if g.ref == "half":  # half the vehicle (the midpoint of its x extent) past x_min, every part in y / z
                pts = [p for p, _ in parts.values()]
                xm = 0.5 * (min(p[0] for p in pts) + max(p[0] for p in pts))
                inside = g.x_min <= xm <= g.x_max and all(abs(p[1]) <= g.y_half and g.z_min <= p[2] <= g.z_max for p in pts)
            else:
                inside = all(
                    p is not None and g.x_min <= p[0] <= g.x_max and abs(p[1]) <= g.y_half and g.z_min <= p[2] <= g.z_max
                    for p in pts
                )
            if ob.payload:
                pl = load_pos
                if pl is None or pl[2] < PAYLOAD_DROP_Z or pl[0] < min(p[0] for p, _ in parts.values()) - 0.05:
                    return self._finish(False, "dropped payload")
                inside = inside and g.x_min <= pl[0] <= g.x_max
            if inside and g.hold_s > 0:
                self.hold_hist.append((t, float(c_pos[0])))
                self.hold_hist = [h for h in self.hold_hist if t - h[0] <= g.hold_s + 1e-9]
                xs_h = [h[1] for h in self.hold_hist]
                if t - self.hold_hist[0][0] >= g.hold_s - 1e-9 and max(xs_h) - min(xs_h) < 0.005:
                    return self._finish(True, "stopped in the goal")
                inside = False
                self.anchor_t = t  # waiting in the goal is not "stuck"
            elif g.hold_s > 0:
                self.hold_hist = []
            if inside:
                return self._reached(t)
            if self.cross_t is not None:
                # (user 2026-10-28) reaching the goal is the pass, even if the vehicle leaves the goal area afterwards
                return self._finish(True, "crossed (left the goal area afterwards)", self.cross_t)
            if t - self.anchor_t > float(os.environ.get("LEGO_BENCH_STUCK_S") or ob.stuck_window_s):  # env: diagnostics
                reason = "stuck"
            elif t >= ob.time_limit_s and self.cross_t is None:
                reason = "timeout"
        if reason:
            return self._finish(False, reason if self.cross_t is None else f"{reason} after crossing", self.cross_t)
        return None

    def _lane_parts(self, parts: dict):
        """the parts the per-part lane rule applies to: all on the wall; on the bumps (v12) only parts not yet past
        the end of the bump field, and none once the vehicle has reached the goal (driving on past the field is fine)"""
        if self.ob.kind != "bumps":
            return list(parts.values())
        if self.cross_t is not None:
            return []
        x_end = self.ob.params.get("field_m", INF)
        return [(p, q) for p, q in parts.values() if p[0] <= x_end]

    def _wheel_outside(self, parts: dict) -> bool:
        """(v12) bumps: a wheel origin over the bump field outside the rods' y span (+ F_BUMP_WHEEL_TOL), before the
        vehicle reached the goal"""
        span = self.ob.params.get("rods_y_span_m")
        if not self.wheels or span is None or self.cross_t is not None:
            return False
        x_end = self.ob.params.get("field_m", INF)
        for n in self.wheels:
            p = parts.get(n)
            if p is not None and 0.0 <= p[0][0] <= x_end and not (
                    span[0] - F_BUMP_WHEEL_TOL <= p[0][1] <= span[1] + F_BUMP_WHEEL_TOL):
                return True
        return False

    def _reached(self, t: float) -> dict | None:
        """inside the goal: success at once, or (final ladder) after settle_s seconds there, upright"""
        if self.settle_s <= 0:
            return self._finish(True, "crossed")
        if self.cross_t is None:
            self.cross_t = t
        self.anchor_t = t  # resting in the goal is not "stuck"
        if self.tilt_now > self.UPRIGHT_DEG:
            self.tilt_since = t if self.tilt_since is None else self.tilt_since
            if t - self.tilt_since >= self.UPRIGHT_HOLD_S:
                return self._finish(False, f"tipped over after crossing ({self.tilt_now:.0f} deg)", self.cross_t)
        else:
            self.tilt_since = None
        if t - self.cross_t >= self.settle_s - 1e-9:
            return self._finish(True, f"crossed and stayed upright {self.settle_s:g} s", self.cross_t)
        return None

    def _finish(self, ok: bool, reason: str, t_report: float | None = None) -> dict:
        self.result = {
            "obstacle": self.ob.kind,
            "level": self.ob.level,
            "ladder": self.ob.params.get("ladder", "base"),
            "crossed": ok,
            "time_s": round(self.t if t_report is None else t_report, 3),
            "reached_goal_s": None if self.cross_t is None else round(self.cross_t, 3),
            "settle_s": self.settle_s,
            "max_progress": round(min(max(self.progress(), 0.0), 1.0), 4),
            "max_progress_m": round(self.best - self.start_ref, 4),
            "failure": None if ok else reason,
            "params": self.ob.params,
        }
        return self.result

    def timeout_result(self) -> dict:
        return self._finish(False, "timeout")


# ----------------------------------------------------------------------------------------------------------------
# static prediction from the validator stats
# ----------------------------------------------------------------------------------------------------------------


def predict(ob: Obstacle, stats: dict) -> tuple[str, str]:
    """Rough verdict ('likely' / 'unlikely' / 'unknown') from the quasi-static capability estimates (tier C levels:
    their constraints and mechanisms are not modelled: 'unknown' when the estimate does not apply)."""
    try:
        return _predict(ob, stats)
    except (KeyError, TypeError, ZeroDivisionError) as e:
        return "unknown", f"no static estimate ({e})"


def _predict(ob: Obstacle, stats: dict) -> tuple[str, str]:
    cap = stats.get("capability", {})
    p = ob.params
    rope_ready = "rope_hang_depth_mm" in cap and cap.get("rope_hang_depth_mm", -1) > 0
    if ob.kind != "rope" and (
        "max_slope_deg" not in cap or stats.get("inline_wheels") or stats["ground_clearance_mm"] < 0
    ):
        return (
            "unlikely",
            "not a ground vehicle (needs two wheel lines, driven wheels and nothing below the wheel bottoms)",
        )
    if ob.kind == "ramp":
        lim = cap["max_slope_deg"][p["surface"]]
        return (
            "likely" if lim >= p["slope_deg"] else "unlikely"
        ), f"traction-limited slope {lim} deg vs {p['slope_deg']} deg"
    if ob.kind in ("steps",):
        lim = cap["max_step_mm"]
        clr = stats["ground_clearance_mm"]
        ok = lim >= p["step_mm"] and clr > p["step_mm"]
        return (
            "likely" if ok else "unlikely"
        ), f"quasi-static step {lim} mm, clearance {clr} mm vs step {p['step_mm']} mm"
    if ob.kind == "bumps":
        lim = cap["max_step_mm"] * 1.5  # round bumps: shallower contact angle than a square edge
        return ("likely" if lim >= p["max_bump_mm"] else "unlikely"), f"~{lim:.2f} mm bump vs {p['max_bump_mm']} mm"
    if ob.kind == "gap":
        lim = max(cap["max_gap_roll_mm"], cap.get("max_gap_span_mm", 0.0))
        how = "wheel dip/climb-out" if lim == cap["max_gap_roll_mm"] else "span (wheelbase / skid reach)"
        return ("likely" if lim >= p["gap_mm"] else "unlikely"), f"{how} limit {lim} mm vs gap {p['gap_mm']} mm"
    if ob.kind == "bridge":
        lim = cap["min_plank_width_mm"]
        return (
            "likely" if lim <= p["plank_width_mm"] - 2 else "unlikely"
        ), f"needs >= {lim} mm (+ drift) vs plank {p['plank_width_mm']} mm"
    if ob.kind == "tunnel":
        h = stats["height_mm"]
        return (
            "likely" if h < p["clear_height_mm"] - 1 else "unlikely"
        ), f"vehicle height {h} mm vs {p['clear_height_mm']} mm"
    if ob.kind == "tug":
        lim = cap["tug_max_load_g"]
        return (
            "likely" if lim >= p["sled_g"] * 1.1 else "unlikely"
        ), f"traction pulls ~{lim} g vs sled {p['sled_g']} g"
    if ob.kind == "seesaw":
        slope = cap["max_slope_deg"]["wood"]
        ok = slope > p["tilt_deg"] and stats["track_mm"] + 4 <= p["plank_width_mm"] - 2
        return (
            "likely" if ok else "unlikely"
        ), f"slope {slope} deg vs tilt {p['tilt_deg']} deg, track {stats['track_mm']} mm vs {p['plank_width_mm']} mm"
    if ob.kind == "speed":
        v = stats.get("expected_speed_m_s", 0.0) or 0.0
        t = (1.0 + stats["length_mm"] * 1e-3) / (0.9 * v) if v > 0 else INF
        return (
            "likely" if t < p["time_limit_s"] else "unlikely"
        ), f"~{t:.1f} s at 90% of no-load speed vs {p['time_limit_s']} s"
    if ob.kind == "rope":
        if not rope_ready:
            return "unlikely", "wheels not in one line / centre of mass not below the rope"
        need = p["incline_deg"] + math.degrees(math.atan(4 * p["sag_mm"] * 1e-3 / p["span_m"]))
        lim = cap.get("rope_max_incline_deg", 0.0)
        clr = cap.get("rope_clearance_mm", 0.0)
        ok = lim >= need and clr >= 0.5
        return (
            "likely" if ok else "unlikely"
        ), f"rope traction {lim} deg vs steepest {need:.1f} deg, rope clearance {clr} mm"
    if ob.kind == "turns":  # steering is not modelled statically: only rule out a vehicle that cannot fit the lane
        w, length = stats["width_mm"], stats["length_mm"]
        if w >= p["lane_width_mm"]:
            return "unlikely", f"vehicle {w} mm wide vs lane {p['lane_width_mm']} mm"
        return "unknown", (f"no static steering model; vehicle {length} x {w} mm (diagonal "
                           f"{math.hypot(length, w):.0f} mm) vs lane {p['lane_width_mm']} mm, radius {p['radius_mm']} mm")
    if ob.kind == "wall":
        lim, clr = cap["max_step_mm"], stats["ground_clearance_mm"]
        ok = lim >= p["height_mm"] and clr > p["height_mm"]
        return ("likely" if ok else "unlikely"), f"quasi-static step {lim} mm vs wall {p['height_mm']} mm (no mechanisms modelled)"
    if ob.kind == "uturn":
        w = stats["width_mm"]
        if w >= p["lane_width_mm"]:
            return "unlikely", f"vehicle {w} mm wide vs lane {p['lane_width_mm']} mm"
        return "unknown", f"no static steering model; turn radius {p['turn_radius_mm']} mm on a {p['tilt_deg']} deg plate"
    return "unknown", ""
