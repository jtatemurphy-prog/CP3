"""copy a design to outputs + write job files for the runner; returns (design files, job files) for committing.
Each job id is <tag>_<bench>_<nudge>[_rec]. wall raises CSIM_WALL_S for exploration runs."""
import json, os, shutil, sys
OUT = "/mnt/user-data/outputs"


def jobs(design_path, tag, benches=("steps", "rope", "ramp", "bridge"), nudges=("plain",), video=True, record=False,
         wall=2400, suffix=""):
    os.makedirs(f"{OUT}/agent_designs", exist_ok=True); os.makedirs(f"{OUT}/agent_jobs", exist_ok=True)
    dst = f"{OUT}/agent_designs/{tag}.json"
    if os.path.abspath(design_path) != os.path.abspath(dst):
        shutil.copy(design_path, dst)
    files = []
    for b in benches:
        for n in nudges:
            jid = f"{tag}_{b}_{n}{suffix}"
            p = f"{OUT}/agent_jobs/{jid}.job.json"
            json.dump({"id": jid, "kind": "sim", "design": f"agent_designs/{tag}.json", "bench": b, "nudge": n,
                       "video": video and n == "plain", "record": record, "wall": wall}, open(p, "w"))
            files.append(p)
    return [dst], files


def commit_list(paths):
    """device_commit_files entries for staged outputs."""
    out = []
    for p in paths:
        rel = os.path.relpath(p, OUT).replace("/", "\\")
        out.append({"devicePath": "C:\\Users\\Tate Murphy\\Desktop\\CP3\\" + rel, "stagedPath": p})
    return out


if __name__ == "__main__":
    print(json.dumps(jobs(sys.argv[1], sys.argv[2])))
