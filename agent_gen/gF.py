"""Proto C: straddle car, mechanically coupled 4WD + 2 driven rope wheels, 2 L motors.

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


def build(name="C1", level=7, bat_x=0.0, motor="99499c01", chassis="deck", rope_wheel="55982c01",
          wheel_y=14.2, bat_yaw=90, rx_studs=8, rx_off=8, flanges=True, flange_part="3649", flange_y=11.4, motor_on="A"):
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
                if not (motor_on == "F" and tag == "f"):
                    bu = d.p(f"abu{tag}{s}", "4265c", (x, Y(58), R))
                    d.c(bu, ax, "cross")
            w = d.p(f"w{tag}{s}", "32020c01", (x, Y(wheel_y), R), yaw=yaw_w)
            g = d.p(f"g{tag}{s}", "3649", (x, Y(36), R))
            d.c(w, ax, "cross")
            d.c(g, ax, "cross")
            if not (motor_on == "F" and tag == "f"):
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
                    if not (motor_on == "F" and tag == "f"):
                        d.c(f"ax{tag}{s}", post, "hinge")
                    if motor_on == "F" and tag == "f":  # the foot pin at x 72 also enters the motor (3L)
                        d.d["parts"][-1]["part"] = "6558"; d.d["parts"][-1]["pos_mm"][1] = Y(52)
                        d.c(pf, f"mot{s}", "pin")
                if motor_on == "F" and tag == "f" and xp == 56:
                    pl = d.p(f"ppl{tag}{s}{k}", "6558", (xp, Y(52), R), yaw=90)
                    d.c(pl, f"mot{s}", "pin")
                else:
                    pl = d.p(f"ppl{tag}{s}{k}", "2780", (xp, Y(48), R), yaw=90)
                if tag == "f" and xp == 48 and motor_on == "A":
                    pu = d.p(f"ppu{tag}{s}{k}", "6558", (xp, Y(52), ZT), yaw=90)
                    d.c(pu, f"mot{s}", "pin")
                else:
                    pu = d.p(f"ppu{tag}{s}{k}", "2780", (xp, Y(48), ZT), yaw=90)
                d.c(pl, post, "pin"); d.c(pl, seg, "pin")
                d.c(pu, post, "pin"); d.c(pu, ub, "pin")
        if motor_on == "A":  # motor on idler A (x = 40, ZT), pinned to the upper beam
            m = d.p(f"mot{s}", motor, (40, Y(56), ZT), yaw=yaw_m)
            d.c(f"iaa{s}", m, "motor")
            pn = d.p(f"mpin{s}", "2780", (32, Y(56), ZT), yaw=90)
            d.c(pn, m, "pin"); d.c(pn, ub, "pin")
        else:  # motor on the front wheel axle, pinned through the front segment (3L pins at x 56 / 72)
            m = d.p(f"mot{s}", motor, (64, Y(56), R), yaw=yaw_m)
            d.c(f"axf{s}", m, "motor")
        # idlers A / M / B on axle 3 (y 32.2..55.8), hinged in the upper beam
        for tag, x in (("a", 40), ("m", 0), ("b", -40)):
            ax = d.p(f"ia{tag}{s}", "3705", (x, Y(44), ZT), yaw=90)
            g = d.p(f"ig{tag}{s}", "3649", (x, Y(36), ZT))
            d.c(g, ax, "cross")
            if tag != "a" or motor_on != "A":
                d.c(ax, ub, "hinge")
            if tag == "m":
                bb = d.p(f"ibb{s}", "3713", (x, Y(44), ZT)); d.c(bb, ax, "cross")
            for yb in ((30, 58) if (tag != "a" or motor_on != "A") else (30,)):
                bu = d.p(f"ibu{tag}{s}{yb}", "4265c", (x, Y(yb), ZT))
                d.c(bu, ax, "cross")
        d.mesh(f"gf{s}", f"iga{s}"); d.mesh(f"iga{s}", f"igm{s}")
        d.mesh(f"igm{s}", f"igb{s}"); d.mesh(f"igb{s}", f"gr{s}")
    # rope axles across the tunnel
    for tag, x in (("f", 40), ("r", -40)):
        ax = d.p(f"rax{tag}", "50451", (x, 0, ZX), yaw=90)
        rw = d.p(f"rw{tag}", rope_wheel, (x, -0.6, ZX))
        d.c(rw, ax, "cross")
        if flanges:  # 40T gears either side of the rope wheel: the cord runs in the groove between them
            for sgf in (1, -1):
                fl = d.p(f"rfl{tag}{'L' if sgf > 0 else 'R'}", flange_part, (x, sgf * flange_y, ZX))
                d.c(fl, ax, "cross")
        for s, sg in (("L", 1), ("R", -1)):
            g = d.p(f"rg{tag}{s}", "3648", (x, sg * 36, ZX))
            d.c(g, ax, "cross")
            d.c(ax, f"tb{s}", "hinge")
            bu = d.p(f"rbu{tag}{s}", "4265c", (x, sg * 58, ZX))
            d.c(bu, ax, "cross")
            d.mesh(g, f"ig{'a' if tag == 'f' else 'b'}{s}")
    d.d["rope_wheels"] = ["rwf", "rwr"]
    # deck: 2x4 plates on the top bricks (rows y 44/52), 2x4 plates (rows 36/44), 4x10 deck plate (x +-16)
    zt = ZX + 4
    for s, sg in (("L", 1), ("R", -1)):
        b = d.p(f"db{s}", "3020", (0, sg * 48, zt + 3.2))
        d.c(b, f"tb{s}", "stud", 4)
        b2 = d.p(f"db2{s}", "3020", (0, sg * 40, zt + 6.4))
        d.c(b2, b, "stud", 4)
    zp = zt + 9.6
    deck = d.p("deck", "3030", (0, 0, zp), yaw=90)
    d.c(deck, "db2L", "stud", 4); d.c(deck, "db2R", "stud", 4)
    bat = d.p("battery", "87513c01", (bat_x, 0, zp + 38.4), yaw=bat_yaw)
    d.c(bat, deck, "stud", 32)
    import math
    ry = math.radians(bat_yaw)
    rx_ = d.p("receiver", "58123c01", (bat_x - rx_off * math.cos(ry), -rx_off * math.sin(ry), zp + 38.4 + 9.6), yaw=bat_yaw)
    d.c(rx_, bat, "stud", rx_studs)
    d.d["chassis"] = chassis
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
