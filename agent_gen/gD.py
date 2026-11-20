"""Proto D (from C): motors both on the right (idlers A_R, B_R), battery hanging low on the left, receiver on top.
Proto C: straddle car, mechanically coupled 4WD + 2 driven rope wheels, 2 L motors.

y planes (left side, mirrored on the right):
  8..28   32020c01 wheel (y_w = 14.2)
  32..40  40T gears: wheel gears (+-64, R), idlers A/M/B (40, 0, -40 at ZT), 24T rope gears (+-40, ZX)
  40..48  vertical 1x5 posts (pinned)
  48..56  lower segments 1x5 (z = R), upper beam 1x15 (z = ZT), top Technic brick 1x12 (holes at ZX)
  56..112 L motor on the front wheel axle (pinned through the lower front segment with 3L pins)
Rope wheels 55982c01 at y = 0 on axles (x = +-40, z = ZX) spanning both sides; bottom at ZX - 15.2 = 80.1.
"""
import json
import sys

from lib import D, validate

R = 31.32
ZT = R + 32
ZX = ZT + 32


def build(name="D1", ballast=4, bat_plate="3035", level=7, bat_x=0.0, motor="99499c01", chassis="deck", rope_wheel="55982c01",
          wheel_y=14.2, bat_yaw=90, rx_studs=8, rx_off=8):
    d = D(name, "straddle C")
    for s, sg in (("L", 1), ("R", -1)):
        Y = lambda v: sg * v  # noqa: E731
        yaw_w = 180 if sg > 0 else 0
        yaw_m = 0 if sg > 0 else 180
        ub = d.p(f"ub{s}", "32278", (0, Y(52), ZT), yaw=0)          # upper beam, holes -56..56 at ZT
        tb = d.p(f"tb{s}", "3703", (0, Y(52), ZX + 4))              # top technic brick, holes -40..40 at ZX
        for tag, x, sx in (("f", 64, 1), ("r", -64, -1)):
            seg = d.p(f"seg{tag}{s}", "32316", (x, Y(52), R))       # 1x5 liftarm, holes x-16..x+16
            # wheel axle
            if True:
                ax = d.p(f"ax{tag}{s}", "44294", (x, Y(wheel_y - 4.0 + 27.8), R), yaw=90)   # axle 7
                bu = d.p(f"abu{tag}{s}", "4265c", (x, Y(58), R))
                d.c(bu, ax, "cross")
            w = d.p(f"w{tag}{s}", "32020c01", (x, Y(wheel_y), R), yaw=yaw_w)
            g = d.p(f"g{tag}{s}", "3649", (x, Y(36), R))
            d.c(w, ax, "cross")
            d.c(g, ax, "cross")
            d.c(ax, seg, "hinge")
            # two vertical posts (1x5 liftarm standing up, holes along y) at x = sx*48, sx*56 from R to ZT
            for k, xp in enumerate((sx * 48, sx * 56)):
                if abs(xp) == 48:
                    post = d.p(f"post{tag}{s}{k}", "40490", (xp, Y(44), R + 32), yaw=0, spin=90)
                    pz = d.p(f"ppz{tag}{s}{k}", "2780", (xp, Y(48), ZX), yaw=90)
                    d.c(pz, post, "pin"); d.c(pz, tb, "pin")
                else:  # L 3x5: vertical arm R..ZT at xp, foot along x at R (xp, xp+-8, xp+-16)
                    post = d.p(f"post{tag}{s}{k}", "32526", (xp, Y(44), ZT), yaw=0 if sx > 0 else 180, spin=90)
                    pf = d.p(f"ppf{tag}{s}{k}", "2780", (xp + sx * 16, Y(48), R), yaw=90)
                    d.c(pf, post, "pin"); d.c(pf, seg, "pin")
                    d.c(f"ax{tag}{s}", post, "hinge")
                if True:
                    pl = d.p(f"ppl{tag}{s}{k}", "2780", (xp, Y(48), R), yaw=90)
                if sg < 0 and abs(xp) == 48:
                    pu = d.p(f"ppu{tag}{s}{k}", "6558", (xp, Y(52), ZT), yaw=90)
                    d.c(pu, f"mot{'a' if xp > 0 else 'b'}", "pin")
                else:
                    pu = d.p(f"ppu{tag}{s}{k}", "2780", (xp, Y(48), ZT), yaw=90)
                d.c(pl, post, "pin"); d.c(pl, seg, "pin")
                d.c(pu, post, "pin"); d.c(pu, ub, "pin")
        if sg < 0:  # both motors on the right: on idler A (x 40) and idler B (x -40)
            for mt, xm in (("a", 40), ("b", -40)):
                m = d.p(f"mot{mt}", motor, (xm, Y(56), ZT), yaw=yaw_m)
                d.c(f"ia{mt}{s}", m, "motor")
                xo = 32 if xm > 0 else -32
                pn = d.p(f"mpin{mt}", "2780", (xo, Y(56), ZT), yaw=90)
                d.c(pn, m, "pin"); d.c(pn, ub, "pin")
        # idlers A / M / B on axle 3 (y 32.2..55.8), hinged in the upper beam
        for tag, x in (("a", 40), ("m", 0), ("b", -40)):
            motor_shaft = sg < 0 and tag in ("a", "b")
            ax = d.p(f"ia{tag}{s}", "32073" if tag == "m" else "3705", (x, Y(48 if tag == "m" else 44), ZT), yaw=90)
            g = d.p(f"ig{tag}{s}", "3649", (x, Y(36), ZT))
            d.c(g, ax, "cross")
            if not motor_shaft:
                d.c(ax, ub, "hinge")
            if tag == "m":
                bb = d.p(f"ibb{s}", "3713", (x, Y(44), ZT)); d.c(bb, ax, "cross")
                bo = d.p(f"ibo{s}", "4265c", (x, Y(66), ZT)); d.c(bo, ax, "cross")
            ybs = (30,) if motor_shaft else ((30,) if tag == "m" else (30, 58))
            for yb in ybs:
                bu = d.p(f"ibu{tag}{s}{yb}", "4265c", (x, Y(yb), ZT))
                d.c(bu, ax, "cross")
        # side pods on bricks pinned to the upper beam (holes at ZT): left = battery, right = ballast
        long_ = "3894"
        pxs = (-16, -8, 8, 16)
        bi = d.p(f"bi{s}", long_, (0, Y(60), ZT + 4))
        bo_ = d.p(f"bo{s}", long_, (0, Y(84), ZT + 4))
        d.c(f"iam{s}", bi, "hinge")
        for xp in pxs:
            pn = d.p(f"bp{s}{xp}", "2780", (xp, Y(56), ZT), yaw=90)
            d.c(pn, bi, "pin"); d.c(pn, ub, "pin")
        pl = d.p(f"pp{s}", "3032", (0, Y(72), ZT + 7.2))
        n = 6
        d.c(pl, bi, "stud", n); d.c(pl, bo_, "stud", n)
        if sg > 0:
            bat = d.p("battery", "87513c01", (bat_x, Y(72), ZT - 5.6))
            d.c(bat, bi, "stud", 4); d.c(bat, bo_, "stud", 4)
        else:
            prev = pl
            for k in range(ballast):
                b = d.p(f"bal{k}", "3001", (0, Y(72), ZT + 7.2 + 9.6 * (k + 1)), yaw=90)
                d.c(b, prev, "stud", 8)
                prev = b
        d.mesh(f"gf{s}", f"iga{s}"); d.mesh(f"iga{s}", f"igm{s}")
        d.mesh(f"igm{s}", f"igb{s}"); d.mesh(f"igb{s}", f"gr{s}")
    # rope axles across the tunnel
    for tag, x in (("f", 40), ("r", -40)):
        ax = d.p(f"rax{tag}", "50451", (x, 0, ZX), yaw=90)
        rw = d.p(f"rw{tag}", rope_wheel, (x, -0.6, ZX))
        d.c(rw, ax, "cross")
        for s, sg in (("L", 1), ("R", -1)):
            g = d.p(f"rg{tag}{s}", "3648", (x, sg * 36, ZX))
            d.c(g, ax, "cross")
            d.c(ax, f"tb{s}", "hinge")
            bu = d.p(f"rbu{tag}{s}", "4265c", (x, sg * 58, ZX))
            d.c(bu, ax, "cross")
            d.mesh(g, f"ig{'a' if tag == 'f' else 'b'}{s}")
    d.d["rope_wheels"] = ["rwf", "rwr"]
    # deck: 2x4 plates on the top bricks (rows y 44 / 52), 6x12 plate, battery, receiver
    zt = ZX + 4
    for s, sg in (("L", 1), ("R", -1)):
        b = d.p(f"db{s}", "3020", (0, sg * 48, zt + 3.2))
        d.c(b, f"tb{s}", "stud", 4)
    zp = zt + 3.2 + 3.2
    deck = d.p("deck", "3028", (0, 0, zp), yaw=90)
    d.c(deck, "dbL", "stud", 4); d.c(deck, "dbR", "stud", 4)
    rx_ = d.p("receiver", "58123c01", (0, 0, zp + 9.6), yaw=0)
    d.c(rx_, deck, "stud", 16)
    d.d["chassis"] = chassis
    d.d["motor"] = "mota"
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
