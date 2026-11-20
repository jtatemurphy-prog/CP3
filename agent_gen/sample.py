"""Random population over gN's parameter space: sample, build, validate, keep the valid ones.
Writes population/<id>.json designs + population/params.jsonl (params, valid, errors).  Run: python sample.py N seed"""
import json, os, random, sys, traceback
sys.path.insert(0, os.path.dirname(__file__))
import gN, lib

SPACE = {
    "motor": ["99499c01", "58121c01"],
    "drive": ["ff", "fr"],
    "chain": [True, False],
    "helper": [True, False],
    "rope_mode": ["classic", "pair"],
    "rope_wheel": ["55982c01", "56145c01"],
    "zx_off": [56, 64],
    "xR": [-48, -56, -64],
    "bat_x": [8.0, 16.0, 24.0],
    "bat_y": [8.0, 16.0],
    "rx_y": [-32.0, -40.0],
    "rb_x": [48.0, 56.0],
    "mg": [0, 8],
    "level": [7, 5],
}


def sample(rng):
    p = {k: rng.choice(v) for k, v in SPACE.items()}
    if p["drive"] == "fr":
        p["chain"] = False
    if p["chain"] and p["xR"] != -56:
        p["xR"] = -56          # chain length works out only at wb 112
    if p["rope_mode"] == "pair":
        p["helper"] = True     # rope A rides on the helper post
    if p["rope_mode"] == "classic":
        p["rope_wheel"] = "55982c01"
        p["bat_x"] = 24.0          # deck must clear the rear rope wheel at xR+24
    else:
        p["zx_off"] = 56           # rope gear 24T sits 24 above the helper chain gear
        p["rb_x"] = 56.0 if p["bat_x"] == 8.0 else 72.0   # rear rope wheel clear of the deck and the front post (x 64)
    p["rx_y"] = -32.0 if p["bat_y"] == 16.0 else -40.0
    p["rb_drive"] = p["drive"] == "fr"      # fr: the rear left-right coupling is needed (no gear loop in fr)
    if p["drive"] == "fr" and p["xR"] == -64:
        p["xR"] = -56                       # idler B must sit on the upper beam
    return p


if __name__ == "__main__":
    n, seed = int(sys.argv[1]), int(sys.argv[2])
    rng = random.Random(seed)
    out = os.path.join(os.path.dirname(__file__), "population"); os.makedirs(out, exist_ok=True)
    seen = set()
    with open(os.path.join(out, "params.jsonl"), "a") as f:
        for i in range(n):
            p = sample(rng)
            key = json.dumps(p, sort_keys=True)
            if key in seen:
                continue
            seen.add(key)
            did = f"S{seed}_{i:03d}"
            rec = {"id": did, "params": p}
            try:
                d = gN.build(name=did, **p)
                path = d.save(os.path.join(out, did + ".json"))
                v = lib.validate(path, "steps")
                rec["valid"] = bool(v["ok"] and not v["review"]["critical"])
                rec["errors"] = (v["errors"] + v["review"]["critical"])[:3]
                rec["stats"] = {k: v["stats"].get(k) for k in ("mass_g", "length_mm", "n_parts")}
            except Exception as e:  # noqa: BLE001
                rec["valid"] = False; rec["errors"] = [repr(e)[:200]]
            f.write(json.dumps(rec) + "\n")
            print(did, rec["valid"], rec["errors"][:1])
