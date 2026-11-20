# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""LDraw part-library reader: flattens an LDraw part into coloured triangle meshes.

LDraw (https://www.ldraw.org, parts licensed CC BY 2.0) is the open CAD library of official LEGO parts.
Geometry is authored in LDraw units (1 LDU = 0.4 mm) in a right-handed frame with -Y up. This module
resolves sub-files (``parts/``, ``parts/s/``, ``p/``, and the 48-segment high-resolution primitives in
``p/48/``), accumulates transforms, and returns triangles per colour code in metres, Z-up:

    (x, y, z)_usd = 0.0004 * (x, z, -y)_ldraw

No Isaac Lab / USD imports: this module is used by the offline converter (``scripts/convert_lego_parts.py``).
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np

LDU = 0.0004
"""Metres per LDraw unit."""

MAIN_COLOUR = 16
EDGE_COLOUR = 24

# LDraw -> Z-up (proper rotation, det = +1), then scaled to metres
_TO_USD = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]) * LDU

TOP_STUD_FILES = {"stud.dat", "stud2.dat", "stud2a.dat", "stud10.dat", "stud15.dat", "stud6.dat", "stud6a.dat"}
TOP_STUD_PREFIXES = ("stug-", "stug2-", "stug2a-", "stud-logo", "stud2-logo")
"""Sub-files modelling top studs. Underside tubes (stud3/stud4 and their stug3-/stug4- groups) are kept."""

SKIP_SUBFILES = {"u9190c01.dat"}
"""Sub-files never imported: the PF motor's flexible coiled cable (u9190c01) is not simulated."""

# BrickLink colour id -> LDraw colour code for the colours used in the part list
BRICKLINK_TO_LDRAW_COLOUR = {
    1: 15,  # White
    2: 19,  # Tan
    3: 14,  # Yellow
    5: 4,  # Red
    7: 1,  # Blue
    9: 7,  # Light Gray (old)
    11: 0,  # Black
    12: 47,  # Trans-Clear
    69: 28,  # Dark Tan
    85: 72,  # Dark Bluish Gray
    86: 71,  # Light Bluish Gray
    88: 70,  # Reddish Brown
    115: 297,  # Pearl Gold
    # inventory 2026-10-06 (bricklink_inventory.xlsx) colours
    4: 25,  # Orange
    66: 135,  # Pearl Light Gray
    90: 78,  # Light Nougat
    153: 321,  # Dark Azure
    156: 322,  # Medium Azure
}

# BrickLink part numbers that live under a different LDraw file name
BRICKLINK_TO_LDRAW_PART = {
    "87513c01": "64228",  # PF AAA battery box with dark bluish gray bottom
    "x187": "3648b",  # Gear 24 tooth, 3 axle holes -> single axle hole geometry
    "4265c": "32123",  # Technic bush 1/2 smooth
    "6538c": "6538a",
    "32556": "32556a",
    "32198": "32198a",
    "3023": "3023b",
    "60c01": "76019",
    "3648": "3648b",
    "6575": "6575a",
    "4185": "4185a",
    "970c00": "3815c01",
    # inventory 2026-10-06 parts (BrickLink id -> LDraw file; the composites live in EXTRA_PARTS_DIR)
    "56145c01": "56145c02",  # rim 56145 + tyre 43.2 x 22 ZR (44309)
    "59510c01": "58119",  # PF AA (9 V) battery box 8881
    "62821b": "62821",
    "99498c01": "99498",  # PF servo without its coiled cable (cables are not simulated)
    "58123c01": "58123",  # PF IR receiver without its cable
    "55423c01": "55423",  # train wheel with its rubber band (u9242)
}


@dataclass
class Colour:
    code: int
    name: str
    rgb: tuple[float, float, float]
    alpha: float = 1.0
    finish: str = "solid"  # solid | rubber | metal | chrome | pearlescent | transparent


@dataclass
class LDrawMesh:
    """Flattened part: triangles (N, 3, 3) [m, Z-up] grouped by colour code (16 = main colour)."""

    name: str
    title: str
    triangles: dict[int, np.ndarray] = field(default_factory=dict)

    @property
    def all_triangles(self) -> np.ndarray:
        return np.concatenate(list(self.triangles.values()), axis=0) if self.triangles else np.zeros((0, 3, 3))

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        pts = self.all_triangles.reshape(-1, 3)
        return pts.min(0), pts.max(0)


EXTRA_PARTS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "data", "lego",
                               "ldraw_extra")
"""Project LDraw files (highest priority): wheel + tyre composites the LDraw library has no ``c01`` file for
(55982c01 = rim 55982 + tyre 30391; 4185t = wedge-belt wheel 4185a + its tyre 2815)."""


class LDrawLibrary:
    def __init__(self, root: str, hires_primitives: bool = True):
        self.root = root
        self.hires = hires_primitives
        self.colours = self._read_colours(os.path.join(root, "LDConfig.ldr"))
        self._index = self._build_index()

    # -- files -------------------------------------------------------------------------------------------
    def _build_index(self) -> dict[str, str]:
        index: dict[str, str] = {}
        # lowest priority first; later entries override
        for sub in ("p", "parts", "parts/s", "models"):
            base = os.path.join(self.root, sub)
            if not os.path.isdir(base):
                continue
            prefix = "s/" if sub == "parts/s" else ""
            for name in os.listdir(base):
                if name.lower().endswith((".dat", ".ldr")):
                    index[(prefix + name).lower()] = os.path.join(base, name)
        if os.path.isdir(EXTRA_PARTS_DIR):
            for name in os.listdir(EXTRA_PARTS_DIR):
                if name.lower().endswith(".dat"):
                    index[name.lower()] = os.path.join(EXTRA_PARTS_DIR, name)
        if self.hires:
            base = os.path.join(self.root, "p", "48")
            self._hires = {n.lower(): os.path.join(base, n) for n in os.listdir(base)} if os.path.isdir(base) else {}
        else:
            self._hires = {}
        return index

    def resolve(self, name: str) -> str:
        key = name.replace("\\", "/").strip().lower()
        if self.hires and key in self._hires and key in self._index and "/p/" in self._index[key].replace("\\", "/"):
            return self._hires[key]  # 48-segment version of a low-res primitive
        if key in self._index:
            return self._index[key]
        if key.startswith("48/") and key[3:] in self._hires:
            return self._hires[key[3:]]
        if key.startswith("s/") and key in self._index:
            return self._index[key]
        if key.startswith("8/") and key[2:] in self._index:  # low-res (8-segment) primitive: use the normal one
            return self.resolve(key[2:])  # (2026-10-14; before, these faces were dropped, e.g. in 69762 / 6575a)
        raise FileNotFoundError(f"LDraw file not found: {name}")

    def part_path(self, part: str) -> str:
        part = BRICKLINK_TO_LDRAW_PART.get(part, part)
        return self.resolve(part if part.lower().endswith(".dat") else part + ".dat")

    def title(self, part: str) -> str:
        with open(self.part_path(part), errors="ignore") as f:
            return f.readline()[2:].strip()

    # -- colours -----------------------------------------------------------------------------------------
    @staticmethod
    def _read_colours(path: str) -> dict[int, Colour]:
        colours: dict[int, Colour] = {}
        pattern = re.compile(r"0\s+!COLOUR\s+(\S+)\s+CODE\s+(\d+)\s+VALUE\s+#([0-9A-Fa-f]{6})(.*)")
        with open(path, errors="ignore") as f:
            for line in f:
                m = pattern.match(line.strip())
                if not m:
                    continue
                name, code, hexv, rest = m.group(1), int(m.group(2)), m.group(3), m.group(4)
                rgb = tuple(int(hexv[i : i + 2], 16) / 255.0 for i in (0, 2, 4))
                alpha_m = re.search(r"ALPHA\s+(\d+)", rest)
                alpha = int(alpha_m.group(1)) / 255.0 if alpha_m else 1.0
                finish = "solid"
                for key in ("RUBBER", "METAL", "CHROME", "PEARLESCENT"):
                    if key in rest:
                        finish = key.lower()
                if alpha < 1.0:
                    finish = "transparent"
                colours[code] = Colour(code, name, rgb, alpha, finish)
        return colours

    # -- geometry ----------------------------------------------------------------------------------------
    @staticmethod
    def is_top_stud(name: str) -> bool:
        """True for sub-files that model top studs (visual only in collision geometry)."""
        base = name.replace("\\", "/").split("/")[-1].lower()
        return base in TOP_STUD_FILES or base.startswith(TOP_STUD_PREFIXES)

    @lru_cache(maxsize=8192)
    def _flatten_file(self, path: str, skip_top_studs: bool = False) -> dict[int, np.ndarray]:
        """Triangles (N, 3, 3) in the file's own LDraw frame, keyed by colour code (16 = inherited)."""
        out: dict[int, list[np.ndarray]] = {}

        def add(colour: int, tris: np.ndarray) -> None:
            out.setdefault(colour, []).append(tris)

        with open(path, errors="ignore") as f:
            lines = f.readlines()
        for raw in lines:
            tok = raw.split()
            if not tok:
                continue
            kind = tok[0]
            if kind == "1" and len(tok) >= 15:
                colour = int(tok[1])
                x, y, z, a, b, c, d, e, f_, g, h, i = (float(v) for v in tok[2:14])
                sub = " ".join(tok[14:])
                if skip_top_studs and self.is_top_stud(sub):
                    continue
                if sub.lower().replace("\\", "/").split("/")[-1] in SKIP_SUBFILES:
                    continue
                try:
                    sub_path = self.resolve(sub)
                except FileNotFoundError:
                    continue
                m = np.array([[a, b, c], [d, e, f_], [g, h, i]])
                t = np.array([x, y, z])
                for sub_colour, tris in self._flatten_file(sub_path, skip_top_studs).items():
                    new_colour = colour if sub_colour == MAIN_COLOUR else sub_colour
                    add(new_colour, tris @ m.T + t)
            elif kind == "3" and len(tok) >= 11:
                v = np.array([float(x) for x in tok[2:11]]).reshape(1, 3, 3)
                add(int(tok[1]), v)
            elif kind == "4" and len(tok) >= 14:
                q = np.array([float(x) for x in tok[2:14]]).reshape(4, 3)
                add(int(tok[1]), np.stack([q[[0, 1, 2]], q[[0, 2, 3]]]))
        return {k: np.concatenate(v, axis=0) for k, v in out.items()}

    def load(self, part: str, skip_top_studs: bool = False) -> LDrawMesh:
        """Flatten ``part``. With ``skip_top_studs`` the top studs are left out (collision geometry)."""
        path = self.part_path(part)
        tris = self._flatten_file(path, skip_top_studs)
        converted = {code: (t @ _TO_USD.T) for code, t in tris.items() if code != EDGE_COLOUR and len(t)}
        return LDrawMesh(name=part, title=self.title(part), triangles=converted)

    def colour_rgb(self, code: int) -> Colour:
        return self.colours.get(code, Colour(code, f"unknown_{code}", (0.5, 0.5, 0.5)))
