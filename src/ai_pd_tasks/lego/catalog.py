# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Part catalog for the LEGO car: which real parts it uses, their colours and measured masses.

Every part comes from the user's inventory ``mechanical_principles_partlist.xml`` (BrickLink format; it is
LEGO Education 9686 *Simple & Powered Machines*). Masses are BrickLink catalogue weights; the battery box
adds six AAA cells. Geometry comes from the LDraw library (see :mod:`ai_pd_tasks.lego.ldraw`).
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from .ldraw import BRICKLINK_TO_LDRAW_COLOUR

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
LDRAW_ROOT = os.environ.get("LDRAW_ROOT", os.path.abspath(os.path.join(PROJECT_ROOT, "..", "third_party", "ldraw")))
PART_USD_DIR = os.path.join(HERE, "..", "assets", "data", "lego")
PARTLIST_XML = os.path.join(PART_USD_DIR, "mechanical_principles_partlist.xml")

AAA_CELL_MASS_G = 11.5
"""Typical alkaline AAA cell [g]."""


@dataclass(frozen=True)
class PartType:
    """One LEGO element type used in the car."""

    bricklink_id: str
    ldraw_id: str
    bricklink_colour: int  # colour id in the inventory
    mass_g: float  # measured (BrickLink catalogue) [g]
    mass_source: str
    role: str

    @property
    def ldraw_colour(self) -> int:
        return BRICKLINK_TO_LDRAW_COLOUR[self.bricklink_colour]

    @property
    def usd_name(self) -> str:
        return f"{self.ldraw_id}_c{self.ldraw_colour}"

    @property
    def usd_path(self) -> str:
        return os.path.abspath(os.path.join(PART_USD_DIR, "parts", f"{self.usd_name}.usda"))


PART_TYPES: dict[str, PartType] = {
    p.bricklink_id: p
    for p in (
        PartType("3895", "3895", 11, 4.2, "BrickLink", "chassis rail: Technic brick 1x12 with holes"),
        PartType("3710", "3710", 11, 0.71, "BrickLink", "spacer plate 1x4"),
        PartType("3701", "3701", 11, 1.46, "BrickLink", "gear-frame: Technic brick 1x4 with holes"),
        PartType("3032", "3032", 11, 3.3, "BrickLink", "deck: plate 4x6"),
        PartType("44294", "44294", 86, 1.05, "BrickLink", "wheel axle: Technic axle 7"),
        PartType("3705", "3705", 11, 0.59, "BrickLink", "jackshaft: Technic axle 4"),
        PartType("94925", "94925", 86, 0.70, "BrickLink", "gear 16 tooth"),
        PartType("3648", "3648b", 85, 1.17, "BrickLink", "gear 24 tooth"),
        PartType("10928", "10928", 85, 0.22, "BrickLink", "motor pinion: gear 8 tooth"),
        PartType("32062", "32062", 5, 0.26, "BrickLink", "motor shaft: Technic axle 2 notched"),
        PartType("4185", "4185a", 86, 0.97, "BrickLink", "wheel: Technic wedge belt wheel"),
        PartType("3713", "3713", 86, 0.14, "BrickLink", "bush"),
        PartType("58120c01", "58120", 86, 33.5, "BrickLink (incl. cable)", "Power Functions M-motor"),
        PartType("87513c01", "64228", 86, 40.0 + 6 * AAA_CELL_MASS_G, "BrickLink 40 g + 6 AAA", "PF AAA battery box"),
    )
}


def read_inventory(path: str = PARTLIST_XML) -> dict[tuple[str, int], int]:
    """BrickLink inventory -> {(part id, colour id): quantity}."""
    inv: dict[tuple[str, int], int] = {}
    for item in ET.parse(path).getroot().findall("ITEM"):
        key = (item.findtext("ITEMID"), int(item.findtext("COLOR")))
        inv[key] = inv.get(key, 0) + int(item.findtext("MINQTY"))
    return inv


def check_against_inventory(bom: dict[str, int], path: str = PARTLIST_XML) -> list[str]:
    """Return problems if the bill of materials (bricklink id -> count) is not covered by the inventory."""
    inv = read_inventory(path)
    problems = []
    for pid, count in bom.items():
        colour = PART_TYPES[pid].bricklink_colour
        have = inv.get((pid, colour), 0)
        if have < count:
            problems.append(f"{pid} colour {colour}: need {count}, inventory has {have}")
    return problems


GEAR_PARTS = {"94925", "3648", "10928"}
CONTACT_OFFSET_M = {"gear": 0.0001, "default": 0.0005}
"""PhysX contact offsets: gears need a small offset (a 0.5 mm offset makes teeth effectively 0.5 mm fatter and jams)."""


def contact_offset(bricklink_id: str) -> float:
    return CONTACT_OFFSET_M["gear" if bricklink_id in GEAR_PARTS else "default"]


# ----------------------------------------------------------------------------------------------------------------
# Extended part set (additive): more real parts for the obstacle benchmark (``ai_pd_tasks.lego_bench``).
# Masses are BrickLink catalogue weights, taken from ``assets/data/lego/online_catalog.json``; geometry is the LDraw
# model named by ``ldraw_id`` (wheel + tyre pairs are the LDraw composite ``c01`` files: rim and rubber tyre).
# ----------------------------------------------------------------------------------------------------------------

EXTENDED_PART_TYPES: dict[str, PartType] = {
    p.bricklink_id: p
    for p in (
        PartType("3894", "3894", 11, 2.2, "BrickLink", "frame: Technic brick 1x6 with holes"),
        PartType("3702", "3702", 11, 2.85, "BrickLink", "frame: Technic brick 1x8 with holes"),
        PartType("3703", "3703", 11, 5.87, "BrickLink", "long chassis rail: Technic brick 1x16 with holes"),
        PartType("32073", "32073", 86, 0.66, "BrickLink", "axle: Technic axle 5"),
        PartType("3706", "3706", 11, 0.86, "BrickLink", "axle: Technic axle 6"),
        PartType("3707", "3707", 11, 1.18, "BrickLink", "axle: Technic axle 8"),
        PartType("3649", "3649", 86, 3.76, "BrickLink", "gear 40 tooth"),
        PartType("32270", "32270", 86, 0.58, "BrickLink", "gear 12 tooth double bevel"),
        PartType("32269", "32269", 2, 1.4, "BrickLink", "gear 20 tooth double bevel"),
        PartType("32498", "32498", 11, 3.5, "BrickLink", "gear 36 tooth double bevel"),
        PartType("2780", "61332", 11, 0.2, "BrickLink", "Technic pin with friction ridges"),
        PartType("3482c01", "3482c01", 86, 2.35, "BrickLink", "wheel 24 mm with rubber tyre (3482 / 3483)"),
        PartType("56904c01", "56904c01", 86, 11.68, "BrickLink", "wheel 43.2 x 14 mm with rubber tyre (56904 / 56898)"),
        PartType("32020c01", "32020c01", 86, 33.0, "BrickLink", "wheel 62.4 x 20 mm with rubber tyre (32020 / 32019)"),
        PartType("99499c01", "99499", 86, 45.0, "BrickLink (incl. cable)", "Power Functions L-motor"),
        PartType("58121c01", "58121", 86, 72.0, "BrickLink (incl. cable)", "Power Functions XL-motor"),
    )
}
"""New part types (none of them replaces a :data:`PART_TYPES` entry); merged into :data:`PART_TYPES` below."""

PART_TYPES.update({k: v for k, v in EXTENDED_PART_TYPES.items() if k not in PART_TYPES})

GEAR_PARTS.update({"3649", "32270", "32269", "32498"})  # tooth collision: 0.1 mm contact offset

TYRE_FRICTION: dict[str, dict] = {
    "3482c01": {"mu": 0.65, "surface": "plywood", "source": "Brick Experiment Channel wheel grip test (3482c01)"},
    "56904c01": {"mu": 1.15, "surface": "plywood", "source": "Brick Experiment Channel wheel grip test (56904c01)"},
    "32020c01": {"mu": 1.38, "surface": "plywood", "source": "Brick Experiment Channel wheel grip test (32020c01)"},
}
"""Measured sliding friction of rubber-tyred wheels (BEC: driven 4-wheel car on a tilting plywood board, wheels
slipping, mu = tan(angle); ``online_catalog.json``). Hard ABS parts (every other part) keep the ABS material."""

MOTOR_LINES: dict[str, dict] = {
    "99499c01": {
        "no_load_rpm": 390.0,
        "stall_torque_nm": 0.18,
        "source": "Philo motor comparison, 9 V (PF Large motor); BEC with a PF battery box: 375 rpm, 0.22 N m",
    },
    "58121c01": {
        "no_load_rpm": 220.0,
        "stall_torque_nm": 0.40,
        "source": "Philo motor comparison, 9 V (PF XL motor); BEC with a PF battery box: 220 rpm, 0.40 N m",
    },
}
"""Linear DC-motor lines (tau = tau_stall (1 - w / w0)) of the extra motors. The PF M-motor keeps the line in
:mod:`ai_pd_tasks.lego.car_design` (BEC: 400 rpm, 0.15 N m)."""

# ----------------------------------------------------------------------------------------------------------------
# Round-3 liftarms (additive, append-only): pinned Technic liftarm structures for the obstacle benchmark's
# mechanisms (wheel-legs, deployable booms). Masses are BrickLink catalogue weights (``online_catalog.json``);
# geometry is the LDraw model named by ``ldraw_id``.
# ----------------------------------------------------------------------------------------------------------------

LIFTARM_PART_TYPES: dict[str, PartType] = {
    p.bricklink_id: p
    for p in (
        PartType("60483", "60483", 11, 0.54, "BrickLink", "Technic liftarm thick 1x2 with axle hole and pin hole"),
        PartType("32524", "16615", 11, 1.74, "BrickLink", "Technic liftarm thick 1x7"),
        PartType("40490", "40490", 11, 2.59, "BrickLink", "Technic liftarm thick 1x9"),
        PartType("32278", "32278", 11, 4.0, "BrickLink", "Technic liftarm thick 1x15"),
        PartType("7229", "7229", 11, 0.69, "BrickLink", "Technic liftarm thick 1x3 with centre axle hole"),
        PartType("3737", "3737", 11, 1.49, "BrickLink", "axle: Technic axle 10"),
        PartType("3708", "3708", 11, 1.82, "BrickLink", "axle: Technic axle 12"),
        PartType("4716", "4716", 86, 0.6, "BrickLink", "Technic worm gear (2L, single start)"),
    )
}
"""New part types for round 3 (none replaces an existing entry); merged into :data:`PART_TYPES` below."""

PART_TYPES.update({k: v for k, v in LIFTARM_PART_TYPES.items() if k not in PART_TYPES})

# ----------------------------------------------------------------------------------------------------------------
# Motor cable and plug (the car's electrical connection). The PF M-motor (58120c01) comes with a fixed 4-wire
# cable ending in a 2x2 PF connector plug; the plug is pressed onto the 2x2 socket on the battery box top. The
# plug is simulated as its own part (LDraw 58124c01), the cable as a chain of capsules (lego/cable.py). They are
# part of the motor's inventory item, so the bill of materials counts them with the motor (SUBPART_OF). Masses
# are estimates: BrickLink's 33.5 g for the motor includes cable and plug; the motor body is spawned lighter by
# PLUG + CABLE so the total stays 33.5 g.
# ----------------------------------------------------------------------------------------------------------------
PF_PLUG_MASS_G = 2.5  # estimate (a 2x2 PF connector, ~1.5 cm^3 of ABS plus contacts)
PF_CABLE_MASS_G = 4.5  # estimate (~20 g/m for the flat 4-wire PF cable)
PF_M_MOTOR_CABLE_LENGTH_M = 0.224  # LDraw u9190c01 path length (the real M-motor cable is ~22 cm)
PART_TYPES.setdefault(
    "58124c01", PartType("58124c01", "58124c01", 86, PF_PLUG_MASS_G, "estimate (inside the motor's 33.5 g)",
                         "PF connector plug of the M-motor cable"))
SUBPART_OF: dict[str, str] = {"58124c01": "58120c01"}
"""Parts simulated separately that belong to another inventory item (not counted in the bill of materials)."""

# ----------------------------------------------------------------------------------------------------------------
# Inventory part set (2026-10-06, additive, append-only): the parts of the user's photographed LEGO inventory
# (``lego_bench/data/inventory_2026-10-06.json``, from ``bricklink_inventory.xlsx`` via scripts/import_inventory.py)
# that the simulator did not have yet. Masses are the BrickLink catalogue unit weights of that sheet; colours are the
# inventory colour with the most pieces; geometry is the LDraw model named by ``ldraw_id`` (``4185t`` / ``55982c01``
# are project composites in ``assets/data/lego/ldraw_extra``). Bench geometry: ``lego_bench.inventory_parts``.
# ----------------------------------------------------------------------------------------------------------------
AA_CELL_MASS_G = 23.0
"""Typical alkaline AA cell [g]."""

INVENTORY_PART_TYPES: dict[str, PartType] = {
    p.bricklink_id: p
    for p in (
        # -- tier 1: electrics --
        PartType("59510c01", "58119", 86, 53.5 + 6 * AA_CELL_MASS_G, "BrickLink 53.5 g + 6 AA",
                 "PF AA battery box 8881 (4 x 11 x 7)"),
        PartType("58123c01", "58123", 86, 20.6, "BrickLink (incl. cable)", "PF IR receiver 8884"),
        # -- tier 1: wheels --
        PartType("56145c01", "56145c02", 11, 15.1, "BrickLink", "wheel 30.4 x 20 rim with 43.2 x 22 ZR tyre"),
        PartType("55982c01", "55982c01", 86, 8.37, "BrickLink", "wheel 18 x 14 rim with 30.4 x 14 tyre"),
        PartType("55423c01", "55423", 11, 1.1, "BrickLink", "RC train wheel with rubber friction band"),
        PartType("4185t", "4185t", 3, 0.97 + 0.75, "BrickLink 4185 + 2815",
                 "wedge-belt wheel 4185 with its rubber tyre 2815 (counts one of each)"),
        # -- tier 1: Technic bricks --
        PartType("2730", "2730", 85, 3.67, "BrickLink", "Technic brick 1x10 with holes"),
        PartType("32018", "32018", 86, 4.92, "BrickLink", "Technic brick 1x14 with holes"),
        PartType("3700", "3700", 86, 0.82, "BrickLink", "Technic brick 1x2 with hole"),
        PartType("32000", "32000", 86, 0.73, "BrickLink", "Technic brick 1x2 with 2 holes"),
        PartType("6541", "6541", 86, 0.38, "BrickLink", "Technic brick 1x1 with hole"),
        # -- tier 1: liftarms --
        PartType("32523", "32523", 86, 0.7, "BrickLink", "Technic liftarm thick 1x3"),
        PartType("32316", "32316", 86, 1.25, "BrickLink", "Technic liftarm thick 1x5"),
        PartType("32525", "32525", 86, 2.8, "BrickLink", "Technic liftarm thick 1x11"),
        PartType("41239", "41239", 86, 3.3, "BrickLink", "Technic liftarm thick 1x13"),
        # -- tier 1: pins --
        PartType("3673", "3673", 86, 0.16, "BrickLink", "Technic pin without friction (pivot)"),
        PartType("6558", "6558", 7, 0.37, "BrickLink", "Technic pin 3L with friction"),
        PartType("32054", "32054", 86, 0.33, "BrickLink", "Technic pin 3L with friction and stop bush"),
        PartType("43093", "43093", 7, 0.26, "BrickLink", "Technic axle-pin 1L+1L with friction"),
        PartType("65249", "65249", 86, 0.3, "BrickLink", "Technic axle-pin 1L pin (no friction) + 2L axle"),
        PartType("15100", "15100", 153, 0.4, "BrickLink", "Technic pin 1L with friction and perpendicular pin hole"),
        PartType("87082", "87082", 86, 0.5, "BrickLink", "Technic pin 3L with centre pin hole"),
        # -- tier 1: axles --
        PartType("4519", "4519", 86, 0.43, "BrickLink", "Technic axle 3"),
        PartType("60485", "60485", 86, 1.3, "BrickLink", "Technic axle 9"),
        PartType("23948", "23948", 86, 1.65, "BrickLink", "Technic axle 11"),
        PartType("50451", "50451", 11, 2.37, "BrickLink", "Technic axle 16"),
        PartType("24316", "24316", 85, 0.45, "BrickLink", "Technic axle 3 with stop"),
        PartType("15462", "15462", 85, 0.73, "BrickLink", "Technic axle 5 with stop"),
        PartType("87083", "87083", 85, 0.59, "BrickLink", "Technic axle 4 with stop"),
        PartType("6587", "6587", 85, 0.48, "BrickLink", "Technic axle 3 with stud"),
        PartType("4265c", "32123a", 1, 0.1, "BrickLink", "Technic bush 1/2 smooth"),
        # -- tier 1: gears --
        PartType("6589", "6589", 86, 0.29, "BrickLink", "gear 12 tooth bevel"),
        PartType("32198", "32198a", 86, 0.75, "BrickLink", "gear 20 tooth bevel"),
        PartType("69778", "69778", 86, 0.5, "BrickLink", "gear 12 tooth (spur, narrow teeth)"),
        PartType("18946", "18946", 86, 0.84, "BrickLink", "gear 16 tooth with clutch on both sides (round bore)"),
        PartType("6542b", "6542b", 86, 0.7, "BrickLink", "gear 16 tooth with clutch, smooth centre (round bore)"),
        PartType("4019", "4019", 9, 0.7, "BrickLink", "gear 16 tooth (axle hole, open sides)"),
        # -- tier 2: bent / shaped liftarms --
        PartType("32140", "32140", 86, 1.43, "BrickLink", "Technic liftarm L 2x4 thick"),
        PartType("32526", "32526", 86, 1.77, "BrickLink", "Technic liftarm L 3x5 thick"),
        PartType("32348", "32348", 86, 2.0, "BrickLink", "Technic liftarm bent 1x7 (4-4) thick"),
        PartType("32271", "32271", 86, 2.63, "BrickLink", "Technic liftarm bent 1x9 (7-3) thick"),
        PartType("6629", "6629", 86, 2.6, "BrickLink", "Technic liftarm bent 1x9 (6-4) thick"),
        PartType("60484", "60484", 86, 1.19, "BrickLink", "Technic liftarm T 3x3 thick"),
        PartType("32056", "32056", 86, 0.73, "BrickLink", "Technic liftarm L 3x3 thin"),
        PartType("32009", "32009", 86, 3.46, "BrickLink", "Technic liftarm double bent 1x11.5 thick"),
        # -- tier 2: connectors --
        PartType("6536", "6536", 86, 0.39, "BrickLink", "Technic axle and pin connector perpendicular"),
        PartType("32184", "32184", 86, 0.64, "BrickLink", "Technic axle and pin connector perpendicular 3L"),
        PartType("42003", "42003", 86, 0.77, "BrickLink", "Technic axle and pin connector perpendicular 3L, 2 pin holes"),
        PartType("32013", "32013", 86, 0.46, "BrickLink", "Technic angle connector #1"),
        PartType("32015", "32015", 86, 0.69, "BrickLink", "Technic angle connector #5 (112.5 deg)"),
        PartType("32034", "32034", 86, 0.68, "BrickLink", "Technic angle connector #2 (180 deg)"),
        PartType("48989", "48989", 86, 1.22, "BrickLink", "Technic pin connector perpendicular 3L with 4 pins"),
        # -- tier 2: plates and bricks --
        PartType("3001", "3001", 11, 2.32, "BrickLink", "brick 2x4"),
        PartType("3020", "3020", 86, 1.2, "BrickLink", "plate 2x4"),
        PartType("3021", "3021", 86, 0.93, "BrickLink", "plate 2x3"),
        PartType("3022", "3022", 86, 0.64, "BrickLink", "plate 2x2"),
        PartType("3023", "3023b", 86, 0.36, "BrickLink", "plate 1x2"),
        PartType("3623", "3623", 86, 0.53, "BrickLink", "plate 1x3"),
        PartType("3795", "3795", 86, 1.74, "BrickLink", "plate 2x6"),
        PartType("3034", "3034", 86, 2.27, "BrickLink", "plate 2x8"),
        PartType("4477", "4477", 86, 1.61, "BrickLink", "plate 1x10"),
        PartType("3028", "3028", 86, 9.47, "BrickLink", "plate 6x12"),
        PartType("3030", "3030", 86, 5.55, "BrickLink", "plate 4x10"),
        PartType("3031", "3031", 86, 2.22, "BrickLink", "plate 4x4"),
        PartType("3033", "3033", 86, 7.95, "BrickLink", "plate 6x10"),
        PartType("3035", "3035", 86, 4.7, "BrickLink", "plate 4x8"),
        PartType("3036", "3036", 86, 6.4, "BrickLink", "plate 6x8"),
        PartType("3958", "3958", 86, 4.71, "BrickLink", "plate 6x6"),
        PartType("41539", "41539", 86, 9.4, "BrickLink", "plate 8x8"),
        PartType("92438", "92438", 86, 17.0, "BrickLink", "plate 8x16"),
        # -- tier 2: PF servo --
        PartType("99498c01", "99498", 86, 44.0, "BrickLink (incl. cable)", "Power Functions servo motor 88004"),
        # -- round 2 (2026-10-06, agent_library_r2): frames and the tread sprocket --
        PartType("64179", "64179", 86, 5.3, "BrickLink", "Technic liftarm frame thick 5x7 open centre"),
        PartType("64178", "64178", 86, 7.0, "BrickLink", "Technic liftarm frame thick 5x11 open centre"),
        PartType("57520", "57520", 66, 1.63, "BrickLink", "Technic tread sprocket wheel small (25.4)"),
        # -- round 3 (2026-10-07, agent_library_r3): gear racks (lego_bench.racks) --
        PartType("3743", "3743", 11, 0.79, "BrickLink", "Technic gear rack 1x4"),
        PartType("6630", "6630", 9, 2.0, "BrickLink", "Technic gear rack 1x8 with holes"),
        PartType("87761", "87761", 11, 1.8, "BrickLink", "Technic gear rack 1x7 with axle and pin holes"),
        # -- round 4a (2026-10-14, library agent r4a): knob / bevel gears, sprocket, connectors, cam, catches,
        # gearbox, steering arm (bench geometry: lego_bench.parts / lego_bench.inventory_r4) --
        PartType("57519", "57519", 3, 5.1, "BrickLink", "Technic tread sprocket wheel large (40.4)"),
        PartType("32072", "32072", 3, 0.87, "BrickLink", "Technic gear knob cog wheel (4 knobs, axle hole)"),
        PartType("69761", "69761", 3, 2.2, "BrickLink", "Technic gear 22 tooth bevel (half, pin hole)"),
        PartType("69762", "69762", 3, 1.2, "BrickLink", "Technic gear 14 tooth bevel thick (half)"),
        PartType("57585", "57585", 86, 0.65, "BrickLink", "Technic axle connector hub with 3 axles"),
        PartType("55615", "55615", 86, 1.9, "BrickLink", "Technic pin connector perpendicular 3x3 bent with 4 pins"),
        PartType("6575", "6575a", 3, 0.7, "BrickLink", "Technic cam"),
        PartType("6641", "6641", 5, 0.5, "BrickLink", "Technic changeover catch"),
        PartType("35188", "35188", 4, 1.08, "BrickLink", "Technic changeover rotary catch"),
        PartType("6588", "6588", 9, 4.5, "BrickLink", "Technic gearbox 2x4x3 1/3 (worm gearbox housing)"),
        PartType("57515", "57515", 85, 1.52, "BrickLink", "Technic steering arm 6x2 with tow ball socket"),
        # -- round 4b (2026-10-14, library agent r4b): shock absorbers (split into a cylinder and a rod body in the
        # simulator: the ``*_cyl`` / ``*_rod`` project composites), CV joint pair, differential and housing, pullback
        # motor, PF train motor (bench geometry: lego_bench.parts / lego_bench.inventory_r4b). Masses: the inventory
        # sheet's BrickLink weights; the rod's share of a shock: inventory_r4b.SHOCKS --
        PartType("731c04", "731c04_cyl", 3, 2.1, "BrickLink",
                 "Technic shock absorber 6.5L, hard spring (LDraw 76537; design part = the whole shock)"),
        PartType("2909c03", "2909c03_cyl", 11, 4.0, "BrickLink",
                 "Technic shock absorber 9.5L, soft spring (LDraw 74741-f1; design part = the whole shock)"),
        PartType("32494", "32494", 86, 0.55, "BrickLink", "Technic steering CV joint ball (axle driver)"),
        PartType("92906", "92906", 85, 0.45, "BrickLink", "Technic steering CV joint cup with 2L axle"),
        PartType("62821b", "62821", 85, 2.8, "BrickLink", "Technic differential, one-piece, 28T bevel ring"),
        PartType("65414", "65414", 86, 2.31, "BrickLink", "Technic differential housing (new style, no ring gear)"),
        PartType("12787c01", "12799", 85, 31.0, "BrickLink", "Pullback motor 6 x 5 x 3 (spring motor)"),
        PartType("87574c01", "87574c01", 11, 57.1, "BrickLink (incl. cable)", "Power Functions train motor 88002"),
    )
}
"""Inventory additions (none replaces an existing entry); merged into :data:`PART_TYPES` below."""

PART_TYPES.update({k: v for k, v in INVENTORY_PART_TYPES.items() if k not in PART_TYPES})
GEAR_PARTS.update({"69778", "18946", "6542b", "4019", "6589", "32198"})  # tooth collision: 0.1 mm contact offset
SHOCK_ROD_PART_TYPES: dict[str, PartType] = {
    "731c04_rod": PartType("731c04_rod", "731c04_rod", 11, 0.7, "estimate: the shock's 2.1 g x the rod's hull-volume share 0.33",
                           "piston rod body of the 6.5L shock 731c04 (simulator only)"),
    "2909c03_rod": PartType("2909c03_rod", "2909c03_rod", 11, 1.2, "estimate: the shock's 4.0 g x the rod's hull-volume share 0.30",
                            "piston rod body of the 9.5L shock 2909c03 (simulator only)"),
}
"""Round 4b: the piston-rod bodies of the shock absorbers (simulated as a second rigid body on a sprung prismatic joint,
``lego_bench.inventory_r4b``): USD assets only, not inventory items (their mass is part of the shock's; the cylinder
body is spawned lighter by it)."""
PART_TYPES.update({k: v for k, v in SHOCK_ROD_PART_TYPES.items() if k not in PART_TYPES})
SUBPART_OF.update({"731c04_rod": "731c04", "2909c03_rod": "2909c03"})

INVENTORY_TYRE_FRICTION: dict[str, dict] = {
    "56145c01": {"mu": 1.19, "surface": "plywood", "source": "Brick Experiment Channel wheel grip test (56145c01)"},
    "55982c01": {"mu": 1.0, "surface": "plywood",
                 "source": "estimate (no published test of tyre 30391): between BEC 3482c01 (0.65, 24 mm) and "
                           "56904c01 (1.15, 43 mm, 14 mm tread)"},
    "4185t": {"mu": 0.37, "surface": "plywood",
              "source": "Philo traction test (wedge-belt wheel 4185 + tyre 2815, pulling force / weight, plywood)"},
}
"""Tyre friction of the inventory wheels (measured where published, else an estimate; same use as TYRE_FRICTION)."""

INVENTORY_MOTOR_LINES: dict[str, dict] = {
    "99498c01": {
        "no_load_rpm": 125.0,
        "stall_torque_nm": 0.13,
        "source": "Philo PF servo page: about 0.12 s per 90 deg of motion (~125 rpm), back-driven above ~12.8 N cm at "
                  "9 V (stall ~0.13 N m); 7 positions each side of neutral (+-90 deg)",
    },
    # round 4b (2026-10-14)
    "87574c01": {
        "no_load_rpm": 1900.0,
        "stall_torque_nm": 0.036,
        "source": "Philo PF train motor 88002 at 9 V, at the wheel axles (both axles driven through internal gears): "
                  "1900 rpm no load, 3.6 N cm stall",
    },
    "12787c01": {
        "no_load_rpm": 600.0,
        "stall_torque_nm": 0.05,
        "stored_turns": 10.0,
        "source": "pullback (spring) motor 12787c01, estimates (no published test): 0.05 N m at full wind, falling "
                  "linearly to 0 over 10 output turns; no-load speed cap 600 rpm. Wound by hand before the run, "
                  "released at t = 0; not IR controlled (lego_bench.inventory_r4b)",
    },
}
"""Motor lines of the inventory additions (the PF servo; its speed / torque are estimates from Philo's servo page; the
round-4b PF train motor (Philo) and pullback motor (estimates; ``stored_turns``: output turns of a full wind))."""

MOTOR_LEAD_MM: dict[str, float] = {
    "58120c01": 224.0, "99499c01": 200.0, "58121c01": 200.0, "99498c01": 200.0,
    "87574c01": 224.0,  # round 4b: the train motor's lead is the same LDraw cable as the M-motor's (u9190c01)
}
"""Fixed PF motor lead length [mm] (motor body to the PF plug; M-motor: LDraw cable path 224 mm; L / XL / servo:
~20 cm). No extension cables are in the inventory (``design.check_real_electrics``)."""
RECEIVER_LEAD_MM = 150.0
"""PF IR receiver lead to the battery box [mm] (estimate; the receiver usually sits on or next to the box)."""

BOM_COMPONENTS: dict[str, tuple[str, ...]] = {"4185t": ("4185", "2815")}
"""Simulated parts that are an assembly of several inventory items (counted per item in the bill of materials)."""
