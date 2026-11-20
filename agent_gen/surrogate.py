"""Fit a per-course progress surrogate on (generator params -> max_progress) and rank untested samples.
usage: python surrogate.py <agent_results_dir> [n_pick]
Uses population/params.jsonl (S-designs) plus HAND (hand designs expressed in the same params).
Model: random forest per course on one-hot params; acquisition = mean + kappa * spread across trees, product over courses
(progress capped at 1 = pass)."""
import json, os, sys, collections
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import popdb, campaign
from sklearn.ensemble import RandomForestRegressor

BENCH = ("steps", "rope", "ramp", "bridge")
HAND = {  # hand designs in sampler params (gN-family only)
    "N2": dict(motor="99499c01", drive="ff", chain=True, helper=True, rope_mode="classic", rope_wheel="55982c01", zx_off=56, xR=-56, bat_x=24.0, bat_y=16.0, rx_y=-32.0, rb_x=56.0),
    "N2X": dict(motor="58121c01", drive="ff", chain=True, helper=True, rope_mode="classic", rope_wheel="55982c01", zx_off=56, xR=-56, bat_x=24.0, bat_y=16.0, rx_y=-32.0, rb_x=56.0),
    "P2": dict(motor="99499c01", drive="ff", chain=True, helper=True, rope_mode="pair", rope_wheel="55982c01", zx_off=56, xR=-56, bat_x=8.0, bat_y=8.0, rx_y=-40.0, rb_x=56.0),
    "P2X": dict(motor="58121c01", drive="ff", chain=True, helper=True, rope_mode="pair", rope_wheel="55982c01", zx_off=56, xR=-56, bat_x=8.0, bat_y=8.0, rx_y=-40.0, rb_x=56.0),
    "P1X": dict(motor="58121c01", drive="ff", chain=True, helper=True, rope_mode="pair", rope_wheel="56145c01", zx_off=56, xR=-56, bat_x=8.0, bat_y=8.0, rx_y=-40.0, rb_x=56.0),
}


def encode(params, keys, cats):
    v = []
    for k in keys:
        for c in cats[k]:
            v.append(1.0 if params[k] == c else 0.0)
    return v


def main(res_dir, n_pick=8, kappa=0.5):
    rows = popdb.load(res_dir)
    recs = {r["id"]: r["params"] for r in campaign.records() if r["valid"]}
    allp = dict(recs); allp.update(HAND)
    keys = sorted(next(iter(allp.values())).keys())
    cats = {k: sorted({json.dumps(p[k]) for p in allp.values()}) for k in keys}
    cats = {k: [json.loads(c) for c in v] for k, v in cats.items()}
    X, Y, ids = [], collections.defaultdict(list), []
    for i, p in allp.items():
        rr = rows.get(i, {})
        for b in BENCH:
            r = rr.get((b, "plain"))
            if r and r.get("max_progress") is not None:
                Y[b].append((encode(p, keys, cats), min(1.0, (r.get("max_progress") or 0) if not r.get("crossed") else 1.0)))
    models = {}
    for b in BENCH:
        if len(Y[b]) >= 4:
            Xb = np.array([x for x, _ in Y[b]]); yb = np.array([y for _, y in Y[b]])
            models[b] = RandomForestRegressor(200, min_samples_leaf=1, random_state=0).fit(Xb, yb)
            print(f"{b}: n={len(yb)} mean={yb.mean():.2f} max={yb.max():.2f}")
    tested = {i for i in rows}
    cand = [i for i in allp if i not in tested]
    scores = []
    for i in cand:
        x = np.array([encode(allp[i], keys, cats)])
        s = 1.0
        for b, m in models.items():
            preds = np.array([t.predict(x)[0] for t in m.estimators_])
            s *= max(1e-3, min(1.0, preds.mean() + kappa * preds.std()))
        scores.append((s, i))
    scores.sort(reverse=True)
    # main effects for the report
    for b, m in models.items():
        imp = sorted(zip(m.feature_importances_, [f"{k}={c}" for k in keys for c in cats[k]]), reverse=True)[:5]
        print(b, "top features:", [(f, round(w, 2)) for w, f in imp])
    return scores[:n_pick]


if __name__ == "__main__":
    print(main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 8))
