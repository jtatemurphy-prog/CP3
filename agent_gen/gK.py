"""Proto K: H1 + a driven front HELPER wheel pair for the steps.

Per side, a 56145c01 (r 21.6) wheel at (xF+56, R+8), centre 8 mm above the ground-wheel axles (bottom 17.7 mm above
the ground: it never touches flat ground). Drive: front wheel 40T -> idler A 40T (xF+24, R+32) -> 24T (xF+56, R+32)
-> 24T on the helper stub axle (xF+56, R+8): helper turns 40/24 = 1.67x the ground wheels, rim speed 1.15x.
The helper reaches the riser corner well below its centre (contact angle ~29 deg vs ~5 deg for the 62 mm wheel), so
the ground wheels' push wedges it up and it lifts the nose; then the front wheels meet the riser already lifted.
Mounting: an L 3x5 (plane y 44) whose vertical arm holds both stub axles, its foot pinned to a 1x7 (plane y 52,
z R+40) that is pinned to the front 1x9 post.  Stub axles stay outside |y| < 5 (rope saddle post)."""
import json, sys
from lib import validate
import gH

R = gH.R


def add_helper(d, xF, po=0, tall=False, hz=8, long_ca=False, hy=17.0, hchain=None, hx_off=64):
    xh = xF + hx_off
    tall = tall or hz < 8
    for s, sg in (("L", 1), ("R", -1)):
        Y = lambda v: sg * v  # noqa: E731
        # helper post hv (L 3x5, plane y44): arm x=xh, holes R+8..R+40, foot +x at R+8
        if tall:   # 1x9 vertical: holes R+8 .. R+72 (also carries the front rope axle at R+56)
            hv = d.p(f"hv{s}", "40490", (xh, Y(44 + po), R + 32 + hz), spin=90)
        else:
            hv = d.p(f"hv{s}", "32526", (xh, Y(44 + po), R + 40), yaw=0, spin=90)
        for zz in (R + 40, R + 24):   # two 1x9 links (plane y52) from the front 1x9 post (x=xF+8) to hv
            e = d.p(f"hx{s}{int(zz)}", "40490", (xF + 40, Y(52 + po), zz))
            pn = d.p(f"hpp{s}{int(zz)}", "2780", (xF + 8, Y(48 + po), zz), yaw=90)
            d.c(pn, e, "pin"); d.c(pn, f"pf{s}", "pin")
            pn = d.p(f"hpf{s}{int(zz)}", "2780", (xh, Y(48 + po), zz), yaw=90)
            d.c(pn, e, "pin"); d.c(pn, hv, "pin")
        # gear carrier (plane y60, z R+32): 1x7 holes xF+16 .. xF+64, pinned to the upper beam at xF+16
        gc = d.p(f"hgc{s}", "32524", (xF + 32, Y(60 + po), R + 32))           # holes xF+8 .. xF+56
        pn = d.p(f"hgp{s}", "2780", (xF + 16, Y(56 + po), R + 32), yaw=90)
        d.c(pn, gc, "pin"); d.c(pn, f"ub{s}", "pin")
        pz = f"pf{s}{int(R + 32)}"   # front post's pin at (xF+8, ZT): make it a 3L through pf, ub and gc
        for p_ in d.d["parts"]:
            if p_["name"] == pz:
                p_["part"] = "6558"; p_["pos_mm"][1] = Y(52 + po)
        d.c(pz, gc, "pin")
        drop = {f"ibua{s}{58 + po}"}
        d.d["parts"] = [p_ for p_ in d.d["parts"] if p_["name"] not in drop]
        d.d["connections"] = [c_ for c_ in d.d["connections"] if c_["child"] not in drop]
        if hchain is not None:   # helper driven 1:1 by a chain straight from the front wheel axle (one 'mesh', same direction)
            import math
            ha = d.p(f"hha{s}", "3707" if po else "44294", (xh, Y(7 + (31.8 if po else 27.8)), R + hz), yaw=90)
            hw = d.p(f"hw{s}", "56145c01", (xh, Y(hy), R + hz), yaw=180 if sg > 0 else 0)
            hs = d.p(f"hhs{s}", "3648", (xh, Y(hchain), R + hz)); fs = d.p(f"hfs{s}", "3648", (xF, Y(hchain), R))
            d.c(hw, ha, "cross"); d.c(hs, ha, "cross"); d.c(fs, f"axf{s}", "cross"); d.c(ha, hv, "hinge")
            for yb in (hchain - 6, 46 + po - 8, 58 + po - 8):
                b = d.p(f"hhb{s}{yb}", "4265c", (xh, Y(yb), R + hz)); d.c(b, ha, "cross")
            C_ = math.hypot(64, hz); rp_ = 6.4 / (2 * math.sin(2 * math.pi / 24))
            nl = int(math.ceil((2 * C_ + 2 * math.pi * rp_) / 6.4 - 2.0 / 6.4))
            d.d.setdefault("chains", []).append({"name": f"hch{s}", "gears": [fs, hs], "links": nl})
            continue
        # 8T on idler A: (xF+48, R+32)
        a8 = d.p(f"h8a{s}", "3705", (xF + 48, Y(48 + po), R + 32), yaw=90)     # y 32.2 .. 63.8
        g8 = d.p(f"h8g{s}", "10928", (xF + 48, Y(36 + po), R + 32))
        d.c(g8, a8, "cross"); d.c(a8, gc, "hinge")
        b8 = d.p(f"h8b{s}", "4265c", (xF + 48, Y(42 + po), R + 32)); d.c(b8, a8, "cross")
        d.mesh(g8, f"iga{s}")
        # 24T chain gear (xh, R+32)
        lc = hz == -8 or long_ca
        ca = d.p(f"hca{s}", "3706" if lc else "3705", (xh, Y(44) if lc else Y(44 + po), R + 32), yaw=90)   # 3705: 28.2 .. 59.8 (+po)
        cg = d.p(f"hcg{s}", "3648", (xh, Y(36 + po), R + 32))
        d.c(cg, ca, "cross"); d.c(ca, hv, "hinge")
        if not long_ca or hz == -8:
            b = d.p(f"hcb{s}", "4265c", (xh, Y(30 + po), R + 32)); d.c(b, ca, "cross")
        d.mesh(cg, g8)
        # helper stub axle (xh, R+8): axle 7, y 7 .. 62.6
        ha = d.p(f"hha{s}", "3707" if po else "44294", (xh, Y(7 + (31.8 if po else 27.8)), R + hz), yaw=90)
        hw = d.p(f"hw{s}", "56145c01", (xh, Y(hy), R + hz), yaw=180 if sg > 0 else 0)
        if hz == -8:   # 40T on the carrier axle (inner plane y31) -> 40T on the helper axle (40 mm)
            c2 = d.p(f"hcg2{s}", "3649", (xh, Y(32), R + 32)); d.c(c2, ca, "cross")
            hg = d.p(f"hhg{s}", "3649", (xh, Y(32), R + hz))
            ybs = (40, 58 + po)
        else:
            hg = d.p(f"hhg{s}", "3648" if hz == 8 else "3649", (xh, Y(36 + po), R + hz))   # hz 0: 40T (24T+40T = 32 mm)
            ybs = (max(30, hy + 13), 58 + po)
        d.c(hw, ha, "cross"); d.c(hg, ha, "cross"); d.c(ha, hv, "hinge")
        for yb in ybs:
            b = d.p(f"hhb{s}{yb}", "4265c", (xh, Y(yb), R + hz)); d.c(b, ha, "cross")
        d.mesh(hg, c2 if hz == -8 else cg)
    return d


def build(name="K1", **kw):
    d = gH.build(name=name, **kw)
    d.d["description"] = "H1 + driven front helper wheels (steps)"
    return add_helper(d, kw.get("xF", 56))


if __name__ == "__main__":
    kw = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    bench = kw.pop("bench", "steps")
    d = build(**kw)
    p = d.save(sys.argv[1])
    v = validate(p, bench)
    print("ok", v["ok"]); print(json.dumps({k: v.get(k) for k in ("errors", "warnings", "review", "stats")}, indent=1)[:5000])
