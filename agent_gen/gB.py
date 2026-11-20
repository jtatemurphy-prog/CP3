"""Proto B: straddle car with a central tunnel (|y|<6 clear up to the deck).
Per side: 32020c01 wheels at x = +-64 (wb 128) on y 6..26, a 40T gear on each wheel axle (y 32..40), three 40T idlers
at z = R + 32 (x = -40, 0, +40), frame bricks at y = 44, one L motor per side on the centre idler (face y = 48).
Deck (8x16 plate) over both sides at z 80.1..83.3; battery + receiver on the deck."""
import json
import sys

from lib import D, validate

R = 31.32  # axle height (wheel radius)
ZT = R + 32  # idler row


def build(name="B1", level=7, bat_x=0.0, motor="99499c01", deck_extra=0, rx_on_bat=True):
    d = D(name, "straddle proto B")
    for s, sg in (("L", 1), ("R", -1)):
        y = lambda v: sg * v  # noqa: E731
        yaw_w = 180 if sg > 0 else 0
        yaw_m = 0 if sg > 0 else 180
        ub = d.p(f"ub{s}", "3703", (0, y(44), ZT + 4))  # upper frame brick, holes at z = ZT
        for tag, x in (("f", 64), ("r", -64)):
            lb = d.p(f"lb{tag}{s}", "3701", (x, y(44), R + 4))  # 1x4 Technic brick, holes x-8, x, x+8
            ax = d.p(f"ax{tag}{s}", "3706", (x, y(31), R), yaw=90)  # axle 6: y 7.2..54.8
            w = d.p(f"w{tag}{s}", "32020c01", (x, y(12.2), R), yaw=yaw_w)
            g = d.p(f"g{tag}{s}", "3649", (x, y(36), R))
            d.c(w, ax, "cross")
            d.c(g, ax, "cross")
            d.c(ax, lb, "hinge")
            # column lb -> ub : 1x2 technic brick, 1x2 technic brick, 1x2 plate at x = x -+ 8 (inside)
            xs = x - 8 if x > 0 else x + 8
            c1 = d.p(f"c1{tag}{s}", "3700", (xs, y(44), R + 4 + 9.6))
            c2 = d.p(f"c2{tag}{s}", "3700", (xs, y(44), R + 4 + 19.2))
            c3 = d.p(f"c3{tag}{s}", "3023", (xs, y(44), R + 4 + 22.4))
            d.c(c1, lb, "stud", 2)
            d.c(c2, c1, "stud", 2)
            d.c(c3, c2, "stud", 2)
            d.c(ub, c3, "stud", 2)
        idl = {}
        for tag, x in (("a", 40), ("m", 0), ("b", -40)):
            ax = d.p(f"ia{tag}{s}", "4519" if tag != "m" else "3705", (x, y(40 if tag != "m" else 44), ZT), yaw=90)
            g = d.p(f"ig{tag}{s}", "3649", (x, y(36), ZT))
            d.c(g, ax, "cross")
            d.c(ax, ub, "hinge")
            idl[tag] = (ax, g)
        d.mesh(f"gf{s}", idl["a"][1])
        d.mesh(idl["a"][1], idl["m"][1])
        d.mesh(idl["m"][1], idl["b"][1])
        d.mesh(idl["b"][1], f"gr{s}")
        # motor on the centre idler shaft, pinned to ub
        m = d.p(f"mot{s}", motor, (0, y(48), ZT), yaw=yaw_m)
        d.c(idl["m"][0], m, "motor")
        for dx in (-8, 8):
            pn = d.p(f"mp{s}{'a' if dx < 0 else 'b'}", "2780", (dx, y(48), ZT), yaw=90)
            d.c(pn, m, "pin")
            d.c(pn, ub, "pin")
        # deck support column on ub: 1x4 technic brick + 1x4 plate at x = -24 and +24
        for k, x in enumerate((-24, 24)):
            b1 = d.p(f"dc{s}{k}", "3701", (x, y(44), ZT + 4 + 9.6))
            p1 = d.p(f"dp{s}{k}", "3710", (x, y(44), ZT + 4 + 12.8))
            d.c(b1, ub, "stud", 4)
            d.c(p1, b1, "stud", 4)
    zd = ZT + 4 + 12.8 + 3.2
    deck = d.p("deck", "92438", (0, 0, zd), yaw=90)
    for s in "LR":
        for k in (0, 1):
            d.c(deck, f"dp{s}{k}", "stud", 4)
    bat = d.p("battery", "87513c01", (bat_x, 0, zd + 38.4), yaw=180)
    d.c(bat, deck, "stud", 32)
    rx = d.p("receiver", "58123c01", (bat_x + 16, 0, zd + 38.4 + 9.6), yaw=0)
    d.c(rx, bat, "stud", 10)
    d.d["chassis"] = "deck"
    d.d["motor"] = "motL"
    d.d["battery"] = "battery"
    d.d["motor_program"] = [{"t": 0.0, "motor": "all", "level": level}]
    return d


if __name__ == "__main__":
    kw = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    bench = kw.pop("bench", "bridge")
    d = build(**kw)
    p = d.save(sys.argv[1])
    v = validate(p, bench)
    print(json.dumps(v, indent=1)[:5000])
