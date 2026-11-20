"""Proto N (= M with every gear/frame plane shifted outward by po=8 mm so a chain plane fits at y 36): both L motors directly on the FRONT wheel stub axles (no gear between motor and front
wheel), each side's rear wheel driven by a 3711 chain 40T-40T (plane y 28; the sim charges ~39% per mesh, chains
count as one mesh), rope wheels lowered to ZX = R+56 with an 8T rope gear on the idlers (rope-wheel bottom 72 mm
above the ground-wheel bottoms: 8 mm margin over the 80 mm cord top at the goal deck). Optional front helper (gK).

Original H notes: straddle car, 4WD as two independent axle drives, rope wheels AHEAD of each ground axle.

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
ZX = ZT + 32   # overridden per build via zx_off


def build(name="N1", po=8, xF=56, xR=-56, zx_off=56, drive="ff", chain=True, chain_y=36.0, spz=24, helper=True, rb_drive=False, rope_mode="classic", rb_x=56.0, rp_gap=2.6, mg=0, longseg=False, rear_ext=False, rb_on_cax=False, rb_high=False, rb_chain=False, rbz=72, hz=8, rb_wheel=None, ra_low=False, rb_sp=None, hy=17.0, ra_z=56, bat_low=None, bat2=True, bat_ly=None, bat_pos="diag", bat_mount=1, fl_y=11.4, fl_part="3649", hchain=None, hx_off=64, rb_side=None, ra_side=None, chain_ceil=False, bat_xo=40, level=7, motor="99499c01", chassis="ubL", rope_wheel="55982c01",
          wheel_y=14.2, flanges=True, bat_x=24.0, bat_y=16.0, rx_y=-32.0, rx_on_bat=False, ub_c=None, tb_c=None):
    global ZX
    ZX = R + zx_off
    if ub_c is None:
        ub_c = xF - 32
    if tb_c is None:
        tb_c = ub_c if not rear_ext else xF - 48
    d = D(name, f"M: front direct drive + chains, rope ZX=R+{zx_off}, helper={helper}")
    xA, xB = xF + 24, xR + 24
    for s, sg in (("L", 1), ("R", -1)):
        Y = lambda v: sg * v  # noqa: E731
        yaw_w = 180 if sg > 0 else 0
        yaw_m = 0 if sg > 0 else 180
        ub = d.p(f"ub{s}", "32278", (ub_c, Y(52 + po), ZT))           # holes ub_c-56 .. ub_c+56 at ZT
        tb = d.p(f"tb{s}", "3703", (tb_c, Y(52 + po), ZX + 4))        # holes tb_c-56 .. tb_c+56 at ZX
        segf = d.p(f"segf{s}", "32316", (xF, Y(52 + po), R))           # holes xF-16 .. xF+16
        segr = d.p(f"segr{s}", "32524", (xR + 8, Y(52 + po), R))       # holes xR-16 .. xR+32
        # ---- wheels, wheel gears
        drive_f = drive == "ff" or (drive == "fr" and sg > 0)
        drive_r = drive == "rr" or (drive == "fr" and sg < 0)
        per_side = drive in ("ff", "rr")
        for tag, xw, seg, driven in (("f", xF, segf, drive_f), ("r", xR, segr, drive_r)):
            drv = (drive_f if tag == "f" else drive_r)
            if mg and drv:   # motor pushed out by mg: axle 9 + a stop bush outside the segment (axle can't walk out)
                ax = d.p(f"ax{tag}{s}", "60485", (xw, Y(wheel_y - 4.0 + 35.8), R), yaw=90)
                sb = d.p(f"asb{tag}{s}", "3713", (xw, Y(60 + po), R)); d.c(sb, ax, "cross")
            else:
                ax = d.p(f"ax{tag}{s}", "3707" if po else "44294", (xw, Y(wheel_y - 4.0 + (31.8 if po else 27.8)), R), yaw=90)   # axle 7: hub .. 65.8
            w = d.p(f"w{tag}{s}", "32020c01", (xw, Y(wheel_y), R), yaw=yaw_w)
            d.c(w, ax, "cross")
            if tag == "f" or rb_drive:
                g = d.p(f"g{tag}{s}", "3649", (xw, Y(36 + po), R))
                d.c(g, ax, "cross")
            else:  # spacer where the gear was: keeps the chain sprocket in line
                g = d.p(f"sp8{tag}{s}", "3713", (xw, Y(36 + po), R))
                d.c(g, ax, "cross")
            if driven:
                m = d.p(f"mot{tag}{s}" if per_side else f"mot{tag}", motor, (xw, Y(56 + po + mg), R), yaw=yaw_m)
                d.c(ax, m, "motor")
            else:
                d.c(ax, seg, "hinge")
                bu = d.p(f"abu{tag}{s}", "4265c", (xw, Y(58 + po), R))
                d.c(bu, ax, "cross")
        # ---- front frame: L-post at x = xF-16 (vertical R..ZT, foot along +x at R: xF-16, xF-8, xF), post at xF+8
        xl = xF - 16
        lf = d.p(f"lpf{s}", "32526", (xl, Y(44 + po), ZT), yaw=0, spin=90)
        if not drive_f:
            d.c(f"axf{s}", lf, "hinge")
        p1_3 = drive_f and motor == "99499c01"
        p1 = d.p(f"lpf{s}p1", "6558" if p1_3 else "2780", (xl, Y((52 + mg / 2 if p1_3 else 48) + po), R), yaw=90)
        p2 = d.p(f"lpf{s}p2", "6558" if drive_f else "2780", (xl + 8, Y((52 + mg / 2 if drive_f else 48) + po), R), yaw=90)
        p3 = d.p(f"lpf{s}p3", "2780", (xl, Y(48 + po), ZT), yaw=90)
        for pn in (p1, p2):
            d.c(pn, lf, "pin"); d.c(pn, segf, "pin")
        d.c(p3, lf, "pin"); d.c(p3, ub, "pin")
        pf = d.p(f"pf{s}", "40490" if zx_off <= 64 else "32525", (xF + 8, Y(44 + po), R + 32 if zx_off <= 64 else R + 40), spin=90)   # 1x9 vertical: R, ZT, ZX
        for z_, part_ in ((R, segf), (ZT, ub), (ZX, tb)):
            use3 = drive_f and z_ == R
            pn = d.p(f"pf{s}{int(z_)}", "6558" if use3 else "2780", (xF + 8, Y((52 + mg / 2 if use3 else 48) + po), z_), yaw=90)
            d.c(pn, pf, "pin"); d.c(pn, part_, "pin")
            if use3:
                d.c(pn, f"motf{s}" if per_side else "motf", "pin")
        if drive_f:  # the motor's other face pin: x = xF - 8 (= L-post foot pin 2, already a 3L pin)
            d.c(p2, f"motf{s}" if per_side else "motf", "pin")
        # ---- rear frame: L-post at x = xR+16 (foot along -x: xR+16, xR+8, xR), post at xR+32
        xl = xR + 16
        lr = d.p(f"lpr{s}", "32526", (xl, Y(44 + po), ZT), yaw=180, spin=90)
        if not drive_r:
            d.c(f"axr{s}", lr, "hinge")
        q1 = d.p(f"lpr{s}p1", "2780", (xl, Y(48 + po), R), yaw=90)
        q2 = d.p(f"lpr{s}p2", "6558" if drive_r else "2780", (xl - 8, Y((52 + mg / 2 if drive_r else 48) + po), R), yaw=90)
        for pn in (q1, q2):
            d.c(pn, lr, "pin"); d.c(pn, segr, "pin")
        if rear_ext:   # upper-beam extension (plane 60+po, ZT) from xR+16 to ub_c-48, two pins into the upper beam
            n_h = int((ub_c - 48 - (xR + 16)) / 8) + 1
            ue = d.p(f"ue{s}", {3: "32523", 5: "32316", 7: "32524", 9: "40490"}[n_h], ((xR + 16 + ub_c - 48) / 2, Y(60 + po), ZT))
            for xq in (ub_c - 56, ub_c - 48):
                pn = d.p(f"uep{s}{xq}", "2780", (xq, Y(56 + po), ZT), yaw=90)
                d.c(pn, ue, "pin"); d.c(pn, ub, "pin")
            q3 = d.p(f"lpr{s}p3", "6558", (xl, Y(52 + po), ZT), yaw=90)
            d.c(q3, lr, "pin"); d.c(q3, ue, "pin")
        elif abs(xl - ub_c) <= 56:
            q3 = d.p(f"lpr{s}p3", "2780", (xl, Y(48 + po), ZT), yaw=90)
            d.c(q3, lr, "pin"); d.c(q3, ub, "pin")
        else:  # brace the rear L-post to the rear 1x9 post with a 1x3 at R+16 (plane y52)
            br = d.p(f"rbr{s}", "32523", (xR + 24, Y(52 + po), R + 16))
            for xq in (xR + 16, xR + 32):
                pn = d.p(f"rbp{s}{xq}", "2780", (xq, Y(48 + po), R + 16), yaw=90)
                d.c(pn, br, "pin"); d.c(pn, lr if xq == xR + 16 else f"pr{s}", "pin")
        pr = d.p(f"pr{s}", "40490" if zx_off <= 64 else "32525", (xR + 32, Y(44 + po), R + 32 if zx_off <= 64 else R + 40), spin=90)
        for z_, part_ in ((R, segr), (ZT, ub), (ZX, tb)):
            if part_ is tb and abs(xR + 32 - tb_c) > 56:
                continue
            if part_ is ub and rear_ext:
                pn = d.p(f"pr{s}{int(z_)}", "6558", (xR + 32, Y(52 + po), z_), yaw=90)
                d.c(pn, pr, "pin"); d.c(pn, f"ue{s}", "pin")
                continue
            pn = d.p(f"pr{s}{int(z_)}", "2780", (xR + 32, Y(48 + po), z_), yaw=90)
            d.c(pn, pr, "pin"); d.c(pn, part_, "pin")
        if drive_r:  # rear motor face pins at xR-8 (new, into segr hole xR-16? no: xR-8) and xR+8 (= q2)
            d.c(q2, f"motr{s}" if per_side else "motr", "pin")
            pm = d.p(f"pmr{s}", "6558" if mg else "2780", (xR - 8, Y(56 + po), R), yaw=90)
            d.c(pm, f"motr{s}" if per_side else "motr", "pin"); d.c(pm, segr, "pin")
        # ---- idlers A (xA, ZT), B (xB, ZT)
        for tag, xi in ((("a", xA), ("b", xB)) if rb_drive else (("a", xA),)):
            ext_b = rear_ext and tag == "b"
            ax = d.p(f"ia{tag}{s}", "32073" if ext_b else "3705", (xi, Y(44 + po + (4 if ext_b else 0)), ZT), yaw=90)
            g = d.p(f"ig{tag}{s}", "3649", (xi, Y(36 + po), ZT))
            d.c(g, ax, "cross")
            d.c(ax, f"ue{s}" if ext_b else ub, "hinge")
            for yb in ((30 + po, 66 + po) if ext_b else (30 + po, 58 + po)):
                bu = d.p(f"ibu{tag}{s}{yb}", "4265c", (xi, Y(yb), ZT))
                d.c(bu, ax, "cross")
            bb = d.p(f"ibb{tag}{s}", "3713", (xi, Y(44 + po), ZT))
            d.c(bb, ax, "cross")
        d.mesh(f"gf{s}", f"iga{s}")
        if rb_drive:
            d.mesh(f"gr{s}", f"igb{s}")
    if True:  # front motor's second pin hole at xF + 8 (post pin, 3L) handled above; rear motor at xR+8 (q2), xR-8
        pass
    # ---- rope axles across the tunnel at xA, xB
    if rope_mode == "pair":
        # rope wheels = 56145c01 PAIRS (r 21.6) whose tyres meet over the cord (a self-centring groove), bottom
        # ZX-21.6 above the ground-wheel bottoms. A at the helper column xF+64, driven by a 24T meshing the helper's
        # 24T chain gear (xF+64, R+32) 24 mm below; B free at rb_x.
        names = []
        yw = 10.8 + rp_gap / 2
        if rb_high:   # free rear rope wheels high (axle R+72 on a 1x9 post at xR+40): they pass over the start post
            rb_x = xR + 48
            for s, sg in (("L", 1), ("R", -1)):
                vp = d.p(f"rbp{s}", "32524", (rb_x, sg * (44 + po), R + 48), spin=90)   # holes R+24 .. R+72
                if rbz != zx_off:
                    pz = d.p(f"rbpz{s}", "2780", (rb_x, sg * (48 + po), ZX), yaw=90)
                    d.c(pz, vp, "pin"); d.c(pz, f"tb{s}", "pin")
                ut = f"uep{s}{int(rb_x)}"
                hit = [p_ for p_ in d.d["parts"] if p_["name"] == ut]
                if hit:   # the ub/ue joint pin at that hole becomes a 3L through the post too
                    hit[0]["part"] = "6558"; hit[0]["pos_mm"][1] = sg * (52 + po)
                    d.c(ut, vp, "pin")
                else:
                    pt = d.p(f"rbpt{s}", "2780", (rb_x, sg * (48 + po), ZT), yaw=90)
                    d.c(pt, vp, "pin"); d.c(pt, f"ub{s}", "pin")
        for tag, x in ((("f", xF + 64), ("r", rb_x)) if not rb_on_cax else (("f", xF + 64),)):
            zr = (R + rbz) if (tag == "r" and rb_high) else (R + ra_z if tag == "f" else ZX)
            ax = d.p(f"rax{tag}", "50451", (x, 0, zr), yaw=90)
            rwp = (rb_wheel or rope_wheel) if tag == "r" else rope_wheel
            if rwp == "55982c01":   # single flanged wheel
                rw = d.p(f"rw{tag}", rwp, (x, -0.6, zr)); d.c(rw, ax, "cross"); names.append(rw)
                for sgf in (1, -1):
                    fl = d.p(f"rfl{tag}{'L' if sgf > 0 else 'R'}", fl_part, (x, sgf * fl_y, zr))
                    d.c(fl, ax, "cross")
            for s, sg in (("L", 1), ("R", -1)):
                if rwp != "55982c01":
                    rw = d.p(f"rw{tag}{s}", "56145c01", (x, sg * yw, zr), yaw=180 if sg > 0 else 0)
                    d.c(rw, ax, "cross"); names.append(rw)
                d.c(ax, (f"rbp{s}" if rb_high else f"tb{s}") if tag == "r" else f"hv{s}", "hinge")
                if tag == "r" and rb_high and rbz == zx_off:
                    d.c(ax, f"tb{s}", "hinge")
                if tag == "f" and ra_low:   # geared down 5:1 from the helper carrier: 8T (hca, y32) -> 40T (rope axle, y32)
                    g8_ = d.p(f"ra8{s}", "10928", (xh_ := xF + 64, sg * 34, R + 32)); d.c(g8_, f"hca{s}", "cross")
                    g = d.p(f"rg{tag}{s}", "3649", (x, sg * 34, ZX)); d.c(g, ax, "cross")
                    d.mesh(g, g8_)
                elif tag == "f":
                    g = d.p(f"rg{tag}{s}", "3648" if ra_z == 56 else "3649", (x, sg * (36 + po), zr))   # ra_z 64: 24T + 40T = 32 mm
                    d.c(g, ax, "cross")
                    d.mesh(g, f"hcg{s}")
                if tag == "r":
                    bu = d.p(f"rbu{tag}{s}", "4265c", (x, sg * ((38 + po) if rb_high else (50 if po else 58)), zr))
                    d.c(bu, ax, "cross")
        d.d["rope_wheels"] = names
        if rb_high and rb_chain and rb_drive:   # drive the high rear rope wheels from the coupling axle by chains (y +-28)
            d.d.setdefault("chains", [])
            C_ = math.hypot(rb_x - xB, (R + rbz) - ZX)
            spn = 24 if C_ >= 26 else 16
            spp = "3648" if spn == 24 else "94925"
            spb = {16: "94925", 24: "3648", 40: "3649"}[rb_sp] if rb_sp else spp
            rpb = 6.4 / (2 * math.sin(2 * math.pi / (rb_sp or spn)))
            rp_ = 6.4 / (2 * math.sin(2 * math.pi / spn))
            nl = int(math.ceil((2 * C_ + math.pi * (rp_ + rpb) + (rpb - rp_) ** 2 / C_) / 6.4 - 2.0 / 6.4))
            for s, sg in (("L", 1), ("R", -1)):
                a_ = d.p(f"rcs{s}", spp, (xB, sg * 28, ZX)); d.c(a_, "cax", "cross")
                b_ = d.p(f"rbs{s}", spb, (rb_x, sg * 28, R + rbz)); d.c(b_, "raxr", "cross")
                d.d["chains"].append({"name": f"rch{s}", "gears": [a_, b_], "links": nl})
        if rb_drive:   # rear left-right coupling axle (no rope wheel) above the cord, driven from idler B by 8T gears
            cx = d.p("cax", "50451", (xB, 0, ZX), yaw=90)
            if rb_on_cax:   # rear rope wheels ride on the driven coupling axle (5x the wheel speed: they push hard)
                for s_, sg_ in (("L", 1), ("R", -1)):
                    rw = d.p(f"rwr{s_}", rope_wheel if rope_wheel == "56145c01" else "56145c01", (xB, sg_ * yw, ZX), yaw=180 if sg_ > 0 else 0)
                    d.c(rw, cx, "cross"); names.append(rw)
            for s, sg in (("L", 1), ("R", -1)):
                g = d.p(f"cg{s}", "10928", (xB, sg * (36 + po), ZX)); d.c(g, cx, "cross")
                d.c(cx, f"tb{s}", "hinge")
                bu = d.p(f"cbu{s}", "4265c", (xB, sg * (50 if po else 58), ZX)); d.c(bu, cx, "cross")
                d.mesh(g, f"igb{s}")
    for tag, x in ((("f", xA), ("r", xB)) if rope_mode == "classic" else ()):
        ax = d.p(f"rax{tag}", "50451", (x, 0, ZX), yaw=90)
        if rope_wheel == "56145c01":   # tyre pair meeting over the cord
            yw_ = 10.8 + rp_gap / 2
            for s_, sg_ in (("L", 1), ("R", -1)):
                rw = d.p(f"rw{tag}{s_}", "56145c01", (x, sg_ * yw_, ZX), yaw=180 if sg_ > 0 else 0)
                d.c(rw, ax, "cross")
        else:
            rw = d.p(f"rw{tag}", rope_wheel, (x, -0.6, ZX))
            d.c(rw, ax, "cross")
        if flanges and rope_wheel != "56145c01":
            for sgf in (1, -1):
                fl = d.p(f"rfl{tag}{'L' if sgf > 0 else 'R'}", fl_part, (x, sgf * fl_y, ZX))
                d.c(fl, ax, "cross")
        for s, sg in (("L", 1), ("R", -1)):
            d.c(ax, f"tb{s}", "hinge")
            if (tag == "r" and (not rb_drive or (rb_side and s != rb_side))) or (tag == "f" and ra_side and s != ra_side):   # rb_side: rope B driven from one side only (no L-R loop)
                bu = d.p(f"rbu{tag}{s}", "4265c", (x, sg * (50 if po else 58), ZX))
                d.c(bu, ax, "cross")
                continue
            g = d.p(f"rg{tag}{s}", "3648" if abs(ZX - ZT - 32) < 0.1 else ("3649" if abs(ZX - ZT - 40) < 0.1 else "10928"), (x, sg * (36 + po), ZX))
            d.c(g, ax, "cross")
            bu = d.p(f"rbu{tag}{s}", "4265c", (x, sg * (50 if po else 58), ZX))
            d.c(bu, ax, "cross")
            d.mesh(g, f"ig{'a' if tag == 'f' else 'b'}{s}")
    if rope_mode == "classic":
        d.d["rope_wheels"] = ["rwf", "rwr"] if rope_wheel != "56145c01" else ["rwfL", "rwfR", "rwrL", "rwrR"]
    if chain:
        wb = xF - xR
        spp = {24: "3648", 40: "3649", 16: "94925"}[spz]
        rp = 6.4 / (2 * math.sin(2 * math.pi / spz))
        links = int(math.ceil((2 * wb + 2 * math.pi * rp) / 6.4)) if chain_ceil else int(math.floor((2 * wb + 2 * math.pi * rp) / 6.4 + 0.3))
        d.d.setdefault("chains", [])
        for s, sg in (("L", 1), ("R", -1)):
            a = d.p(f"spf{s}", spp, (xF, sg * chain_y, R)); d.c(a, f"axf{s}", "cross")
            b = d.p(f"spr{s}", spp, (xR, sg * chain_y, R)); d.c(b, f"axr{s}", "cross")
            d.d["chains"].append({"name": f"ch{s}", "gears": [a, b], "links": links})
    # ---- deck on top (6x12 plate, x = bat_x +- 24, between the rope flanges), battery + receiver side by side
    zt = ZX + 4
    for s, sg in (("L", 1), ("R", -1)):
        if po:
            for k_, dx_ in (("a", -16), ("b", 16), ("c", 0)):
                b = d.p(f"db{s}{k_}", "3020", (bat_x + dx_, sg * (48 + po), zt + 3.2), yaw=90)
                d.c(b, f"tb{s}", "stud", 2)
        else:
            b = d.p(f"db{s}", "3020", (bat_x, sg * (48 + po), zt + 3.2))
            d.c(b, f"tb{s}", "stud", 4)
    zp = zt + 6.4
    deck = d.p("deck", "3028", (bat_x, 0, zp), yaw=90)
    for s in ("L", "R"):
        if po:
            d.c(deck, f"db{s}a", "stud", 2); d.c(deck, f"db{s}b", "stud", 2); d.c(deck, f"db{s}c", "stud", 2)
        else:
            d.c(deck, f"db{s}", "stud", 4)
    if bat_low is not None and bat_mount == 2:   # boxes low at the rear, clear of the wheel axles: 1x4 brick (holes xR+16..xR+32)
        # pinned outside the rear segment, 4x6 plate on it, box on the plate (box bottom 3 mm above the axle bush)
        for s, sg in (("L", 1), ("R", -1)):
            nm = "battery" if s == "L" else "battery2"
            br = d.p(f"bbr{s}", "3701", (xR + 24, sg * (60 + po), R + 4))
            for dx_ in (16, 32):
                hit = [p_ for p_ in d.d["parts"] if p_["part"] in ("2780", "6558") and abs(p_["pos_mm"][0] - (xR + dx_)) < 0.1
                       and abs(p_["pos_mm"][2] - R) < 0.1 and sg * p_["pos_mm"][1] > 0]
                if hit:
                    p_ = hit[0]; p_["part"] = "6558"; p_["pos_mm"][1] = sg * (60 + po - 8); d.c(p_["name"], br, "pin")
                else:
                    pn = d.p(f"bbp{s}{dx_}", "2780", (xR + dx_, sg * (56 + po), R), yaw=90)
                    d.c(pn, f"segr{s}", "pin"); d.c(pn, br, "pin")
            br2 = d.p(f"bbq{s}", "3701", (xR + 24, sg * (68 + po), R + 4))    # second brick outside the first: two stud rows
            pq = d.p(f"bbx{s}", "6558", (xR + 24, sg * (60 + po), R), yaw=90)   # 3L pin: segment + both bricks
            d.c(pq, f"segr{s}", "pin"); d.c(pq, br, "pin"); d.c(pq, br2, "pin")
            ly_ = bat_ly if bat_ly is not None else 80 + po
            pl = d.p(f"bpl{s}", "3032", (xR + 32, sg * (72 + po), R + 4 + 3.2))
            d.c(pl, br, "stud", 3); d.c(pl, br2, "stud", 3)
            b_ = d.p(nm, "87513c01", (xR + 32, sg * ly_, R + 4 + 3.2 + 38.4), yaw=0 if sg > 0 else 180)
            d.c(b_, pl, "stud", 18)
        bat = "battery"
    elif bat_low is not None and bat_mount == 3:   # (needs longseg) two 1x6 bricks side by side outside the long segment,
        # 3L pins through segment + both bricks, 4x8 plate on 12 studs, box on the plate; all clear of the rear axle bush
        xb = xR + bat_xo
        for s, sg in (("L", 1), ("R", -1)):
            nm = "battery" if s == "L" else "battery2"
            br = d.p(f"bbr{s}", "3894", (xb, sg * (60 + po), R + 4))
            br2 = d.p(f"bbq{s}", "3894", (xb, sg * (68 + po), R + 4))
            for dx_ in (-16, 16):
                pq = d.p(f"bbx{s}{dx_ + 16}", "6558", (xb + dx_, sg * (60 + po), R), yaw=90)
                d.c(pq, f"segr{s}", "pin"); d.c(pq, br, "pin"); d.c(pq, br2, "pin")
            ly_ = bat_ly if bat_ly is not None else 80 + po
            pl = d.p(f"bpl{s}", "3035", (xb, sg * (72 + po), R + 4 + 3.2))
            d.c(pl, br, "stud", 6); d.c(pl, br2, "stud", 6)
            b_ = d.p(nm, "87513c01", (xb, sg * ly_, R + 4 + 3.2 + 38.4), yaw=0 if sg > 0 else 180)
            d.c(b_, pl, "stud", 24)
        bat = "battery"
    elif bat_low is not None:   # battery boxes LOW and outboard: 1x6 Technic brick pinned outside a wheel segment, 4x8 plate on it
        # reaching outward, box on the plate (clear of the idler bushes). Left box behind on segr, right box (twin, ballast)
        # in front on segf -> laterally balanced.
        corners = (("L", 1, xR + 8, "segr", "abur"), ("R", -1, xF, "segf", "abuf")) if bat_pos == "diag" else \
            (("L", 1, xR + 8, "segr", "abur"), ("R", -1, xR + 8, "segr", "abur"))
        for s, sg, xs, seg, ab in corners:
            if s == "R" and not bat2:
                continue
            nm = "battery" if s == "L" else "battery2"
            had_bu = any(p_["name"] == f"{ab}{s}" for p_ in d.d["parts"])
            d.d["parts"] = [p_ for p_ in d.d["parts"] if p_["name"] != f"{ab}{s}"]
            d.d["connections"] = [c_ for c_ in d.d["connections"] if f"{ab}{s}" not in (c_["parent"], c_["child"])]
            if had_bu:   # the wheel axle's retaining bush sat where the brick goes: long axle through the brick, bush outside it
                axn = f"ax{'r' if seg == 'segr' else 'f'}{s}"
                for p_ in d.d["parts"]:
                    if p_["name"] == axn:
                        p_["part"] = "60485"; p_["pos_mm"][1] = sg * (wheel_y - 4.0 + 35.8)
                bu2 = d.p(f"{ab}{s}x", "4265c", (xR if seg == "segr" else xF, sg * (74 + po - 8), R))
                d.c(bu2, axn, "cross")
            br = d.p(f"bbr{s}", "3894", (xs, sg * (60 + po), R + 4))     # top centre; holes 4 mm below the top, at R
            axn = f"ax{'r' if seg == 'segr' else 'f'}{s}"
            if abs(xs - (xR if seg == "segr" else xF)) <= 16:
                d.c(axn, br, "hinge")   # the wheel axle turns in the brick's hole (declared: a bearing, not a collision)
            used = 0
            for dx_ in (-16, -8, 0, 8, 16):   # lengthen the segment's existing 2L pins to 3L so they also go through the brick
                for p_ in d.d["parts"]:
                    if p_["part"] == "2780" and abs(p_["pos_mm"][0] - (xs + dx_)) < 0.1 and abs(p_["pos_mm"][1] - sg * (56 + po - 8 + 8 - 8)) < 0.1 \
                            and abs(p_["pos_mm"][2] - R) < 0.1:
                        p_["part"] = "6558"; p_["pos_mm"][1] = sg * (64 + po - 8); d.c(p_["name"], br, "pin"); used += 1
            free = [dx_ for dx_ in (-16, 16, -8, 8) if abs(xs + dx_ - (xF if seg == "segf" else xR)) > 0.1
                    and not any(abs(p_["pos_mm"][0] - (xs + dx_)) < 0.1 and abs(p_["pos_mm"][2] - R) < 0.1 and abs(abs(p_["pos_mm"][1]) - (64 + po - 8)) < 4.1
                                for p_ in d.d["parts"] if p_["part"] in ("2780", "6558"))]
            for k_, dx_ in zip("ab", free[:max(0, 2 - used)]):
                pn = d.p(f"bbp{s}{k_}", "2780", (xs + dx_, sg * (56 + po), R), yaw=90)
                d.c(pn, f"{seg}{s}", "pin"); d.c(pn, br, "pin")
            ly_ = bat_ly if bat_ly is not None else 80 + po
            pl = d.p(f"bpl{s}", "3035", (xs, sg * min(ly_ - 8, 72 + po), R + 4 + 3.2))
            d.c(pl, br, "stud", 6)
            b_ = d.p(nm, "87513c01", (xs, sg * ly_, R + 4 + 3.2 + 38.4), yaw=0 if sg > 0 else 180)
            d.c(b_, pl, "stud", 24 if ly_ - min(ly_ - 8, 72 + po) <= 8 else 16)
        bat = "battery"
    else:
        bat = d.p("battery", "87513c01", (bat_x, bat_y, zp + 38.4), yaw=90)
        d.c(bat, deck, "stud", 32)
    if rx_on_bat:
        rx_ = d.p("receiver", "58123c01", (bat_x, bat_y + rx_y, zp + 38.4 + 9.6), yaw=90)
        d.c(rx_, bat, "stud", 8)
    else:
        rx_ = d.p("receiver", "58123c01", (bat_x, rx_y, zp + 9.6), yaw=0)
        d.c(rx_, deck, "stud", 16 if abs(rx_y) <= 32 else 12)
    d.d["chassis"] = chassis
    d.d["motor"] = {"ff": "motfL", "rr": "motrL"}.get(drive, "motf")
    if longseg and xF - xR == 128:   # 1x15 (front axle .. xF-112) + a 1x7 one plane out carrying the rear axle, 2 shared pins
        for s, sg in (("L", 1), ("R", -1)):
            old = {f"segf{s}", f"segr{s}"}
            d.d["parts"] = [p_ for p_ in d.d["parts"] if p_["name"] not in old]
            d.p(f"seg{s}", "32278", (xF - 56, sg * (52 + po), R))
            d.p(f"segx{s}", "32524", (xR + 8, sg * (60 + po), R))   # holes xR-16 .. xR+32
            pn = d.p(f"sgp{s}", "2780", (xF - 104, sg * (56 + po), R), yaw=90)
            d.c(pn, f"seg{s}", "pin"); d.c(pn, f"segx{s}", "pin")
            conns = []
            for c_ in d.d["connections"]:
                c2 = dict(c_)
                for k_ in ("parent", "child"):
                    if c2[k_] == f"segf{s}":
                        c2[k_] = f"seg{s}"
                    elif c2[k_] == f"segr{s}":
                        c2[k_] = f"segx{s}"
                conns.append(c2)
            conns = [c_ for c_ in conns if not (c_["child"] == f"pf{s}{int(R)}" and c_["parent"] == f"seg{s}")
                     and c_["child"] != f"abur{s}"]
            d.d["connections"] = conns
            d.d["parts"] = [p_ for p_ in d.d["parts"] if p_["name"] != f"abur{s}"]
            for p_ in d.d["parts"]:
                if p_["name"] == f"axr{s}":   # axle 9 so a stop bush fits outside the rear 1x7
                    p_["part"] = "60485"; p_["pos_mm"][1] = sg * (wheel_y - 4.0 + 35.8)
            sb = d.p(f"rsb{s}", "4265c", (xR, sg * (66 + po), R)); d.c(sb, f"axr{s}", "cross")
            for xq, nm in ((xR + 16, f"lpr{s}p1"), (xR + 32, f"pr{s}{int(R)}")):
                d.c(nm, f"seg{s}", "pin")
            # rear pins that used to reach segr (plane 52+po) must now reach segx (60+po): make them 3L at 60+po? they
            # sit at y 48+po (2780, 40..56) -> move to 6558 centred 56+po (44..68)
            for p_ in d.d["parts"]:
                if p_["name"] in (f"lpr{s}p1", f"lpr{s}p2", f"pr{s}{int(R)}"):
                    p_["part"] = "6558"; p_["pos_mm"][1] = sg * (52 + po)
    if longseg and xF - xR == 112:   # one 1x15 at axle height carrying both axles: a rigid wheelbase for the chains
        for s, sg in (("L", 1), ("R", -1)):
            old = {f"segf{s}", f"segr{s}"}
            new = f"seg{s}"
            d.d["parts"] = [p_ for p_ in d.d["parts"] if p_["name"] not in old]
            d.p(new, "32278", ((xF + xR) / 2, sg * (52 + po), R))
            seen = set(); conns = []
            for c_ in d.d["connections"]:
                c2 = dict(c_)
                if c2["parent"] in old: c2["parent"] = new
                if c2["child"] in old: c2["child"] = new
                k_ = (c2["child"], c2["parent"], c2["kind"])
                if k_ in seen: continue
                if c2["parent"] == new and c2["kind"] == "pin":
                    px = next(p_["pos_mm"][0] for p_ in d.d["parts"] if p_["name"] == c2["child"])
                    if not (xF - 112 - 0.1 <= px <= xF + 0.1):
                        continue   # pin beyond the long beam's ends
                seen.add(k_); conns.append(c2)
            d.d["connections"] = conns
            cnt = {}
            for c_ in conns:
                if c_["kind"] == "pin":
                    cnt[c_["child"]] = cnt.get(c_["child"], 0) + 1
            lone = {n for n, k in cnt.items() if k < 2 and n.startswith(("pf", "pmr"))}
            d.d["parts"] = [p_ for p_ in d.d["parts"] if p_["name"] not in lone]
            d.d["connections"] = [c_ for c_ in d.d["connections"] if c_["child"] not in lone]
    if helper:
        import gK  # noqa
        gK.add_helper(d, xF, po=po, tall=(rope_mode == "pair"), hz=hz, long_ca=ra_low, hy=hy, hchain=hchain, hx_off=hx_off)
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
