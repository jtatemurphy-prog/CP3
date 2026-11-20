"""Population campaign helpers.
  pick(n)            greedy max-diversity pick of untested valid samples (Hamming distance on params)
  submit(ids, benches) copy designs to outputs + write runner jobs; returns commit lists
  table(results_dir) population table joined with params (for the surrogate)"""
import json, os, sys
sys.path.insert(0, os.path.dirname(__file__))
import submit as SB
HERE = os.path.dirname(os.path.abspath(__file__))
POP = os.path.join(HERE, "population")


def records():
    return [json.loads(l) for l in open(os.path.join(POP, "params.jsonl"))]


def pick(n, exclude=()):
    recs = [r for r in records() if r["valid"] and r["id"] not in exclude]
    chosen = []
    def dist(a, b):
        return sum(a["params"][k] != b["params"][k] for k in a["params"])
    while recs and len(chosen) < n:
        best = max(recs, key=lambda r: min((dist(r, c) for c in chosen), default=99))
        chosen.append(best); recs.remove(best)
    return [r["id"] for r in chosen]


def submit(ids, benches=("steps", "bridge"), video=True):
    D, J = [], []
    for i in ids:
        dd, jj = SB.jobs(os.path.join(POP, i + ".json"), i, benches=benches, video=video)
        D += dd; J += jj
    return SB.commit_list(D), SB.commit_list(J)


if __name__ == "__main__":
    ids = pick(int(sys.argv[1]))
    d, j = submit(ids)
    json.dump({"ids": ids, "designs": d, "jobs": j}, open("/tmp/batch.json", "w"))
    print(ids)
