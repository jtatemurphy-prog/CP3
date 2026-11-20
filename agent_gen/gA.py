"""Proto A: straddle car, 4 x 32020c01 direct-driven (front L, rear M per side), central tunnel |y|<6 up to the deck.
Params via kwargs."""
import json
import sys

from lib import D, validate

R = 31.32  # wheel radius = axle height


def build(name="A1", wb=80, xf_off=40, deck_n=5, front="99499c01", rear="58120c01", level=7, battery_top=True):
    d = D(name, "straddle proto")
    xf, xr = xf_off, xf_off - wb
    xc = 0.0
    for s, sg in (("L", 1), ("R", -1)):
        yaw_m = 0 if sg > 0 else 180
        yaw_w = 180 if sg > 0 else 0
        ob = d.p(f"ob{s}", "3703", (xc, sg * 87, R + 4))
        # front motor (pin mounted to the outer brick through its back pin holes)
        for pos, mt, tag in ((xf, front, "f"), (xr, rear, "r")):
            m = d.p(f"m{tag}{s}", mt, (pos, sg * 27, R), yaw=yaw_m)
            ax = d.p(f"ax{tag}{s}", "4519", (pos, sg * 19, R), yaw=90)
            w = d.p(f"w{tag}{s}", "32020c01", (pos, sg * 12.2, R), yaw=yaw_w)
            d.c(ax, m, "motor")
            d.c(w, ax, "cross")
            if mt in ("99499c01",):
                for dx in (-8, 8):
                    pn = d.p(f"pin{tag}{s}{'a' if dx < 0 else 'b'}", "2780", (pos + dx, sg * 83, R), yaw=90)
                    d.c(pn, m, "pin")
                    d.c(pn, ob, "pin")
            elif mt == "58120c01":
                pl = d.p(f"mpl{tag}{s}", "3034", (pos, sg * 59, R - 12), yaw=90)
                d.c(m, pl, "stud", 12)
                a = d.p(f"sp1{tag}{s}", "3023", (pos, sg * 87, R - 12 + 3.2))
                b = d.p(f"sp2{tag}{s}", "3023", (pos, sg * 87, R - 12 + 6.4))
                d.c(a, pl, "stud", 2)
                d.c(b, a, "stud", 2)
                d.c(ob, b, "stud", 2)
        # mid plate on the outer brick, between the motors
        mp = d.p(f"mp{s}", "3035", (xc, sg * 59, R + 4 + 3.2), yaw=90)
        d.c(mp, ob, "stud", 4)
        # column of 2x4 bricks on the mid plate (y 31..63)
        prev = mp
        z = R + 4 + 3.2
        for k in range(deck_n):
            z += 9.6
            col = d.p(f"col{s}{k}", "3001", (xc, sg * 47, z), yaw=90)
            d.c(col, prev, "stud", 8)
            prev = col
        d.d.setdefault("_top", {})[s] = (prev, z)
    zt = d.d["_top"]["L"][1]
    deck = d.p("deck", "92438", (xc, 0, zt + 3.2), yaw=90)
    for s in "LR":
        d.c(deck, d.d["_top"][s][0], "stud", 8)
    del d.d["_top"]
    zd = zt + 3.2
    bat = d.p("battery", "87513c01", (xc - 8, 0, zd + 38.4), yaw=180)
    d.c(bat, deck, "stud", 32)
    rx = d.p("receiver", "58123c01", (xc - 8 + 16, 0, zd + 38.4 + 9.6), yaw=0)
    d.c(rx, bat, "stud", 10)
    d.d["chassis"] = "deck"
    d.d["motor"] = "mfL"
    d.d["battery"] = "battery"
    d.d["motor_program"] = [{"t": 0.0, "motor": "all", "level": level}]
    return d


if __name__ == "__main__":
    kw = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    d = build(**kw)
    p = d.save(sys.argv[1])
    v = validate(p, kw.get("bench", "bridge"))
    print(json.dumps(v, indent=1)[:4000])
