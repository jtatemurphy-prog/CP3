"""Population index: scan staged agent_results/*.json and print/write a table of designs x benches.
usage: python popdb.py <results_dir> [out.md]"""
import glob, json, os, sys, collections
BENCH = ("steps", "rope", "ramp", "bridge")


def load(res_dir):
    rows = collections.defaultdict(dict)
    for p in glob.glob(os.path.join(res_dir, "*.json")):
        n = os.path.basename(p)[:-5]
        if n.startswith("_"):
            continue
        try:
            j = json.load(open(p))
        except Exception:
            continue
        r = j.get("result") or {}
        job = j.get("job") or {}
        tag = n
        for b in BENCH:
            k = f"_{b}_"
            if k in n:
                tag, rest = n.split(k, 1)
                bench, nudge = b, rest
                break
        else:
            continue
        rows[tag][(bench, nudge)] = r
    return rows


def cell(r):
    if not r:
        return ""
    if r.get("crossed"):
        return f"PASS {r.get('time_s', 0):.2f}s"
    return f"{r.get('failure', '?')} {r.get('max_progress', 0) or 0:.2f}"


def table(rows):
    out = ["| design | " + " | ".join(BENCH) + " | passes | time |", "|---" * (len(BENCH) + 3) + "|"]
    for tag in sorted(rows):
        rr = rows[tag]
        cells, npass, tt = [], 0, 0.0
        for b in BENCH:
            r = rr.get((b, "plain"))
            cells.append(cell(r))
            if r and r.get("crossed"):
                npass += 1; tt += r.get("time_s", 0)
        out.append(f"| {tag} | " + " | ".join(cells) + f" | {npass} | {tt:.2f} |")
    return "\n".join(out)


if __name__ == "__main__":
    t = table(load(sys.argv[1]))
    print(t)
    if len(sys.argv) > 2:
        open(sys.argv[2], "w").write("# Population results (plain runs)\n\n" + t + "\n")
