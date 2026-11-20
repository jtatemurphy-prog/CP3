# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""LEGO obstacle benchmark: a designer builds a vehicle or mechanism from catalog parts that must cross an obstacle.

* :mod:`.parts`      geometry of every catalog part (studs, holes, axles, gears) in its LDraw-derived frame;
* :mod:`.design`     JSON design format, static validator and computed stats (no simulator);
* :mod:`.baselines`  reference designs (the 9686 car and variants);
* :mod:`.obstacles`  parametric obstacles with difficulty levels and the success/failure judge;
* :mod:`.env`        Isaac Sim runner (spawns the obstacle and the assembled design; imports Isaac Lab lazily).

Only :mod:`.env` needs Isaac Sim; everything else is pure Python (numpy).
"""
