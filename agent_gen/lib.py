"""Tiny helper layer for writing design JSONs."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))


class D:
    def __init__(self, name, desc=""):
        self.d = {"name": name, "description": desc, "chassis": None, "motor": None, "battery": None,
                  "intended": [], "parts": [], "connections": [], "gear_meshes": [], "part_set": "inventory",
                  "motor_program": []}
        self.names = set()

    def p(self, name, part, pos, yaw=0.0, spin=0.0):
        assert name not in self.names, name
        self.names.add(name)
        self.d["parts"].append({"name": name, "part": part, "pos_mm": [round(float(v), 3) for v in pos],
                                "yaw_deg": float(yaw), "spin_deg": float(spin)})
        return name

    def c(self, child, parent, kind, studs=None):
        e = {"child": child, "parent": parent, "kind": kind}
        if studs:
            e["studs"] = studs
        self.d["connections"].append(e)

    def mesh(self, a, b):
        self.d["gear_meshes"].append({"gear_a": a, "gear_b": b})

    def save(self, path):
        json.dump(self.d, open(path, "w"), indent=1)
        return path


def validate(path, bench="steps"):
    import importlib.machinery
    import types
    H = os.path.join(os.path.dirname(__file__), "..")
    ld = importlib.machinery.SourceFileLoader("brick_mod", os.path.join(H, "brick"))
    m = types.ModuleType("brick_mod")
    m.__file__ = os.path.join(H, "brick")
    ld.exec_module(m)
    return m.validate(path, bench)
