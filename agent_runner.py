"""Job runner for the coding agent: watches agent_jobs/ for *.job.json files, runs them with the brick tooling and
writes agent_results/<id>.json. Start it with start_agent_runner.bat (double-click). Create agent_jobs/STOP to quit.

Job file fields:
  id        unique name (defaults to the file name)
  kind      "sim" (one physics run, default) | "validate" | "check" (same as `brick check`)
  design    path relative to this folder
  bench     steps | rope | ramp | bridge
  nudge     "plain" (default) | "offset_1mm" | "commands_50ms_late"     (sim only)
  video     true/false (sim only, default false)
  record    true/false: write every part's pose at 20 fps to record.json (sim only, default false)
"""
import glob
import importlib.machinery
import json
import os
import subprocess
import sys
import threading
import time
import traceback
import types
from concurrent.futures import ThreadPoolExecutor

H = os.path.dirname(os.path.abspath(__file__))
os.chdir(H)
JOBS = os.path.join(H, "agent_jobs")
RES = os.path.join(H, "agent_results")
os.makedirs(JOBS, exist_ok=True)
os.makedirs(RES, exist_ok=True)

_loader = importlib.machinery.SourceFileLoader("brick_mod", os.path.join(H, "brick"))
brick = types.ModuleType(_loader.name)
brick.__file__ = os.path.join(H, "brick")
_loader.exec_module(brick)  # sets the benchmark env vars in os.environ

NUDGE_ENV = {"plain": {}, "offset_1mm": {"LEGO_BENCH_START_DY": "0.001"},
             "commands_50ms_late": {"LEGO_BENCH_SETTLE_DELTA": "0.05"}}
LOG = open(os.path.join(H, "agent_runner.log"), "a", buffering=1)
lock = threading.Lock()


def log(msg):
    with lock:
        line = time.strftime("%H:%M:%S ") + msg
        print(line, flush=True)
        try:
            LOG.write(line + "\n")
        except Exception:  # noqa: BLE001
            pass


VSCRIPT = ("import importlib.machinery, types, json, sys, os\n"
           "H = sys.argv[1]; os.chdir(H)\n"
           "L = importlib.machinery.SourceFileLoader('brick_mod', os.path.join(H, 'brick'))\n"
           "b = types.ModuleType(L.name); b.__file__ = os.path.join(H, 'brick'); L.exec_module(b)\n"
           "print('@@JSON@@' + json.dumps(b.validate(sys.argv[2], sys.argv[3]), default=str))\n")


def validate_sub(path, bench):
    """validate in a child process: a native crash in the geometry code must not kill the runner"""
    p = subprocess.run([sys.executable, "-c", VSCRIPT, H, path, bench], cwd=H, capture_output=True, text=True,
                       timeout=900)
    i = p.stdout.rfind("@@JSON@@")
    if i < 0:
        return {"ok": False, "errors": ["validate crashed (exit %s): %s" % (p.returncode, (p.stderr or p.stdout)[-1500:])],
                "review": {}, "stats": {}}
    return json.loads(p.stdout[i + 8:])


def sim(job):
    path = os.path.abspath(os.path.join(H, job["design"]))
    bench, nudge = job["bench"], job.get("nudge", "plain")
    v = validate_sub(path, bench)
    if not v["ok"]:
        return {"error": "does not validate", "validate": v}
    d = os.path.join(H, "_sims", f"{job['id']}")
    os.makedirs(d, exist_ok=True)
    cmd = [sys.executable, os.path.join(H, "simulator", "run_csim.py"), "--design", path, "--obstacle", bench,
           "--level", "1", "--out", os.path.join(d, "result.json")]
    if job.get("video"):
        cmd += ["--video", "--video_dir", os.path.join(d, "video")]
    if job.get("record"):
        cmd += ["--record", os.path.join(d, "record.json")]
    e = dict(os.environ, **NUDGE_ENV[nudge], PYTHONPATH=os.path.join(H, "src"))
    if job.get("wall"):
        e["CSIM_WALL_S"] = str(job["wall"])
    t0 = time.time()
    with open(os.path.join(d, "sim.log"), "w") as f:
        try:
            subprocess.run(cmd, cwd=os.path.join(H, "simulator"), env=e, stdout=f, stderr=subprocess.STDOUT,
                           timeout=max(1800, 2 * float(job.get("wall") or 900)))
        except subprocess.TimeoutExpired:
            pass
    rp = os.path.join(d, "result.json")
    if not os.path.exists(rp):
        return {"error": "no result", "log_tail": open(os.path.join(d, "sim.log")).read()[-3000:], "dir": d}
    r = json.load(open(rp))
    r["dir"] = d
    r["wall"] = round(time.time() - t0, 1)
    r["review"] = v["review"]
    r["stats"] = v["stats"]
    if job.get("video"):
        r["contact_sheets"] = [p for p in (brick.contact(os.path.join(d, "video", c + ".mp4"),
                                                         os.path.join(d, f"contact_{c}.jpg"))
                                           for c in ("follow", "overview")) if p]
    return r


def do(job_path):
    jid = os.path.basename(job_path)[: -len(".job.json")]
    running = job_path[: -len(".job.json")] + ".running"
    try:
        os.replace(job_path, running)
    except OSError:
        return
    job = None
    try:
        job = json.load(open(running))
        job.setdefault("id", jid)
        kind = job.get("kind", "sim")
        log(f"start {jid} {kind} {job.get('bench')} {job.get('nudge', '')}")
        if kind == "validate":
            out = validate_sub(os.path.join(H, job["design"]), job["bench"])
        elif kind == "check":
            out = subprocess.run([sys.executable, os.path.join(H, "brick"), "check", job["design"], job["bench"]],
                                 cwd=H, capture_output=True, text=True).stdout
            try:
                out = json.loads(out)
            except Exception:  # noqa: BLE001
                out = {"raw": out[-4000:]}
        else:
            out = sim(job)
    except Exception:  # noqa: BLE001
        out = {"error": traceback.format_exc()}
    out = {"id": jid, "job": job, "result": out}
    tmp = os.path.join(RES, jid + ".tmp")
    json.dump(out, open(tmp, "w"), indent=1, default=str)
    os.replace(tmp, os.path.join(RES, jid + ".json"))
    try:
        os.remove(running)
    except OSError:
        pass
    r = out["result"] if isinstance(out["result"], dict) else {}
    log(f"done  {jid}: crossed={r.get('crossed')} failure={r.get('failure')} t={r.get('time_s')} "
        f"prog={r.get('max_progress')} err={str(r.get('error'))[:80]}")


def write_status(d):
    try:
        tmp = os.path.join(RES, "_runner_status.tmp")
        with open(tmp, "w") as f:
            json.dump(d, f)
        os.replace(tmp, os.path.join(RES, "_runner_status.json"))
    except Exception as e:  # noqa: BLE001  (a reader may hold the file open on Windows)
        log(f"status write failed: {e!r}")


def safe_do(q, busy):
    try:
        do(q)
    except Exception:  # noqa: BLE001
        log("job crashed: " + traceback.format_exc()[-800:])
    finally:
        busy.discard(q)


def main():
    n = int(os.environ.get("AGENT_WORKERS", max(2, (os.cpu_count() or 4) // 2)))
    log(f"runner up, {n} workers, cpu={os.cpu_count()}")
    write_status({"workers": n, "cpu": os.cpu_count(), "started": time.time()})
    busy = set()
    with ThreadPoolExecutor(n) as ex:
        for rp in glob.glob(os.path.join(JOBS, "*.running")):   # jobs cut off by a crash / sleep: run them again
            try:
                os.replace(rp, rp[: -len(".running")] + ".job.json"); log("requeued " + os.path.basename(rp))
            except Exception:
                pass
        stop = os.path.join(JOBS, "STOP")
        if os.path.exists(stop):
            try:
                os.remove(stop); log("removed stale STOP")
            except Exception:
                pass
        while not os.path.exists(os.path.join(JOBS, "STOP")):
            try:
                for p in sorted(glob.glob(os.path.join(JOBS, "*.job.json"))):
                    if p not in busy and len(busy) < n:
                        busy.add(p)
                        ex.submit(safe_do, p, busy)
                write_status({"workers": n, "cpu": os.cpu_count(), "heartbeat": time.time(), "busy": len(busy)})
            except Exception:  # noqa: BLE001
                log("loop error: " + traceback.format_exc()[-800:])
            time.sleep(3)
    log("STOP found, exiting")


if __name__ == "__main__":
    main()
