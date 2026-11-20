"""Proto H: straddle car, 4WD as two independent axle drives, rope wheels AHEAD of each ground axle.

Per corner (left side shown; right mirrored):
  wheel 32020c01 at (xW, R) on y 8..28, 40T gear on its axle (plane y 32..40),
  idler 40T at (xW+24, ZT), 24T rope gear at (xW+24, ZX) on the rope axle that crosses the tunnel.
  -> the rope wheel (55982c01 between two 40T flanges) sits 24 mm ahead of the ground wheel, so it is on the cord
     before the ground wheel leaves the start deck, and the ground wheel lands on the goal deck soon after the rope
     wheel leaves the goal saddle.
Front axle pair driven by an L motor on the left front wheel axle, rear pair by an L motor on the right rear axle;
left and right are coupled through the rope axles (straight running). Frame: liftarms + pins in planes 40..48
(L-posts, 1x9 posts) and 48..56 (segments, upper beam, top Technic brick). Battery + receiver on a deck on top.
"""
import json
import math
import sys

from lib import D, validate

R = 31.32
ZT = R + 32
ZX = ZT + 32


def build(name="H1", xF=56, xR=-48, level=7, motor="99499c01", chassis="ubL", rope_wheel="55982c01",
          wheel_y=14.2, flanges=True, bat_x=24.0, bat_y=16.0, rx_y=-32.0, rx_on_bat=False, ub_c=24, tb_c=24):
    d = D(name, "straddle H: rope wheels ahead of the ground axles")
    xA, xB = xF + 24, xR + 24
    for s, sg in (("L", 1), ("R", -1)):
        Y = lambda v: sg * v  # noqa: E731
        yaw_w = 180 if sg > 0 else 0
        yaw_m = 0 if sg > 0 else 180
        ub = d.p(f"ub{s}", "32278", (ub_c, Y(52), ZT))           # holes ub_c-56 .. ub_c+56 at ZT
        tb = d.p(f"tb{s}", "3703", (tb_c, Y(52), ZX + 4))        # holes tb_c-56 .. tb_c+56 at ZX
        segf = d.p(f"segf{s}", "32316", (xF, Y(52), R))           # holes xF-16 .. xF+16
        segr = d.p(f"segr{s}", "32524", (xR + 8, Y(52), R))       # holes xR-16 .. xR+32
        # ---- wheels, wheel gears
        drive_f = (sg > 0)   # left front axle carries the front motor
        drive_r = (sg < 0)   # right rear axle carries the rear motor
        for tag, xw, seg, driven in (("f", xF, segf, drive_f), ("r", xR, segr, drive_r)):
            ax = d.p(f"ax{tag}{s}", "44294", (xw, Y(wheel_y - 4.0 + 27.8), R), yaw=90)   # axle 7: hub .. 65.8
            w = d.p(f"w{tag}{s}", "32020c01", (xw, Y(wheel_y), R), yaw=yaw_w)
            g = d.p(f"g{tag}{s}", "3649", (xw, Y(36), R))
            d.c(w, ax, "cross")
            d.c(g, ax, "cross")
            if driven:
                m = d.p(f"mot{tag}", motor, (xw, Y(56), R), yaw=yaw_m)
                d.c(ax, m, "motor")
            else:
                d.c(ax, seg, "hinge")
                bu = d.p(f"abu{tag}{s}", "4265c", (xw, Y(58), R))
                d.c(bu, ax, "cross")
        # ---- front frame: L-post at x = xF-16 (vertical R..ZT, foot along +x at R: xF-16, xF-8, xF), post at xF+8
        xl = xF - 16
        lf = d.p(f"lpf{s}", "32526", (xl, Y(44), ZT), yaw=0, spin=90)
        if not drive_f:
            d.c(f"axf{s}", lf, "hinge")
        p1 = d.p(f"lpf{s}p1", "6558" if drive_f else "2780", (xl, Y(52 if drive_f else 48), R), yaw=90)
        p2 = d.p(f"lpf{s}p2", "6558" if drive_f else "2780", (xl + 8, Y(52 if drive_f else 48), R), yaw=90)
        p3 = d.p(f"lpf{s}p3", "2780", (xl, Y(48), ZT), yaw=90)
        for pn in (p1, p2):
            d.c(pn, lf, "pin"); d.c(pn, segf, "pin")
        d.c(p3, lf, "pin"); d.c(p3, ub, "pin")
        pf = d.p(f"pf{s}", "40490", (xF + 8, Y(44), R + 32), spin=90)   # 1x9 vertical: R, ZT, ZX
        for z_, part_ in ((R, segf), (ZT, ub), (ZX, tb)):
            use3 = drive_f and z_ == R
            pn = d.p(f"pf{s}{int(z_)}", "6558" if use3 else "2780", (xF + 8, Y(52 if use3 else 48), z_), yaw=90)
            d.c(pn, pf, "pin"); d.c(pn, part_, "pin")
            if use3:
                d.c(pn, "motf", "pin")
        if drive_f:  # the motor's other face pin: x = xF - 8 (= L-post foot pin 2, already a 3L pin)
            d.c(p2, "motf", "pin")
        # ---- rear frame: L-post at x = xR+16 (foot along -x: xR+16, xR+8, xR), post at xR+32
        xl = xR + 16
        lr = d.p(f"lpr{s}", "32526", (xl, Y(44), ZT), yaw=180, spin=90)
        if not drive_r:
            d.c(f"axr{s}", lr, "hinge")
        q1 = d.p(f"lpr{s}p1", "2780", (xl, Y(48), R), yaw=90)
        q2 = d.p(f"lpr{s}p2", "6558" if drive_r else "2780", (xl - 8, Y(52 if drive_r else 48), R), yaw=90)
        q3 = d.p(f"lpr{s}p3", "2780", (xl, Y(48), ZT), yaw=90)
        for pn in (q1, q2):
            d.c(pn, lr, "pin"); d.c(pn, segr, "pin")
        d.c(q3, lr, "pin"); d.c(q3, ub, "pin")
        pr = d.p(f"pr{s}", "40490", (xR + 32, Y(44), R + 32), spin=90)
        for z_, part_ in ((R, segr), (ZT, ub), (ZX, tb)):
            pn = d.p(f"pr{s}{int(z_)}", "2780", (xR + 32, Y(48), z_), yaw=90)
            d.c(pn, pr, "pin"); d.c(pn, part_, "pin")
        if drive_r:  # rear motor face pins at xR-8 (new, into segr hole xR-16? no: xR-8) and xR+8 (= q2)
            d.c(q2, "motr", "pin")
            pm = d.p(f"pmr{s}", "2780", (xR - 8, Y(56), R), yaw=90)
            d.c(pm, "motr", "pin"); d.c(pm, segr, "pin")
        # ---- idlers A (xA, ZT), B (xB, ZT)
        for tag, xi in (("a", xA), ("b", xB)):
            ax = d.p(f"ia{tag}{s}", "3705", (xi, Y(44), ZT), yaw=90)   # axle 4: 28.2..59.8
            g = d.p(f"ig{tag}{s}", "3649", (xi, Y(36), ZT))
            d.c(g, ax, "cross")
            d.c(ax, ub, "hinge")
            for yb in (30, 58):
                bu = d.p(f"ibu{tag}{s}{yb}", "4265c", (xi, Y(yb), ZT))
                d.c(bu, ax, "cross")
            bb = d.p(f"ibb{tag}{s}", "3713", (xi, Y(44), ZT))
            d.c(bb, ax, "cross")
        d.mesh(f"gf{s}", f"iga{s}")
        d.mesh(f"gr{s}", f"igb{s}")
    if True:  # front motor's second pin hole at xF + 8 (post pin, 3L) handled above; rear motor at xR+8 (q2), xR-8
        pass
    # ---- rope axles across the tunnel at xA, xB
    for tag, x in (("f", xA), ("r", xB)):
        ax = d.p(f"rax{tag}", "50451", (x, 0, ZX), yaw=90)
        rw = d.p(f"rw{tag}", rope_wheel, (x, -0.6, ZX))
        d.c(rw, ax, "cross")
        if flanges:
            for sgf in (1, -1):
                fl = d.p(f"rfl{tag}{'L' if sgf > 0 else 'R'}", "3649", (x, sgf * 11.4, ZX))
                d.c(fl, ax, "cross")
        for s, sg in (("L", 1), ("R", -1)):
            g = d.p(f"rg{tag}{s}", "3648", (x, sg * 36, ZX))
            d.c(g, ax, "cross")
            d.c(ax, f"tb{s}", "hinge")
            bu = d.p(f"rbu{tag}{s}", "4265c", (x, sg * 58, ZX))
            d.c(bu, ax, "cross")
            d.mesh(g, f"ig{'a' if tag == 'f' else 'b'}{s}")
    d.d["rope_wheels"] = ["rwf", "rwr"]
    # ---- deck on top (6x12 plate, x = bat_x +- 24, between the rope flanges), battery + receiver side by side
    zt = ZX + 4
    for s, sg in (("L", 1), ("R", -1)):
        b = d.p(f"db{s}", "3020", (bat_x, sg * 48, zt + 3.2))
        d.c(b, f"tb{s}", "stud", 4)
    zp = zt + 6.4
    deck = d.p("deck", "3028", (bat_x, 0, zp), yaw=90)
    d.c(deck, "dbL", "stud", 4); d.c(deck, "dbR", "stud", 4)
    bat = d.p("battery", "87513c01", (bat_x, bat_y, zp + 38.4), yaw=90)
    d.c(bat, deck, "stud", 32)
    if rx_on_bat:
        rx_ = d.p("receiver", "58123c01", (bat_x, bat_y + rx_y, zp + 38.4 + 9.6), yaw=90)
        d.c(rx_, bat, "stud", 8)
    else:
        rx_ = d.p("receiver", "58123c01", (bat_x, rx_y, zp + 9.6), yaw=0)
        d.c(rx_, deck, "stud", 16)
    d.d["chassis"] = chassis
    d.d["motor"] = "motf"
    d.d["battery"] = "battery"
    d.d["motor_program"] = [{"t": 0.0, "motor": "all", "level": level}]
    return d


if __name__ == "__main__":
    kw = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    bench = kw.pop("bench", "rope")
    d = build(**kw)
    p = d.save(sys.argv[1])
    v = validate(p, bench)
    print(json.dumps(v, indent=1)[:6000])
