"""The calibrated simulator as the benchmark's simulator backend: one design on one bench level -> result.json in the
format the harness reads (crossed, failure, time_s, max_progress, ...) plus what this simulator adds (studs pulled
out, gear meshes that skipped or separated, leads unplugged), and optionally the follow / overview videos.
    python run_csim.py --design d.json --obstacle ramp --level 1 --out result.json [--video --video_dir DIR]
                       [--params calib.json] [--start_dy 0.001] [--delay 0.05]
The parameters default to $CSIM_PARAMS, else out/calib3/best.json, else out/calib2/best.json."""
import argparse
import json
import os
import sys
import time

H = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, H)


def default_params():
    for p in (os.environ.get("CSIM_PARAMS"), os.path.join(H, "out", "calib3", "best.json"),
              os.path.join(H, "out", "calib2", "best.json")):
        if p and os.path.exists(p):
            d = json.load(open(p))
            return d.get("params", d), p
    return {}, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--design", required=True)
    ap.add_argument("--obstacle", required=True)
    ap.add_argument("--level", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--params")
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--video_dir")
    ap.add_argument("--record", help="write every part's pose per frame (20 fps) to this JSON (external renderer)")
    ap.add_argument("--start_dy", type=float, default=float(os.environ.get("LEGO_BENCH_START_DY", 0) or 0))
    ap.add_argument("--delay", type=float, default=float(os.environ.get("LEGO_BENCH_SETTLE_DELTA", 0) or 0))
    a, _ = ap.parse_known_args()
    from csim import CrossSim
    from ai_pd_tasks.lego_bench import obstacles as OB

    if a.params:
        prm = json.load(open(a.params))
        prm, src = prm.get("params", prm), a.params
    else:
        prm, src = default_params()
    prm = dict(prm)
    sup = float(os.environ.get("LEGO_BENCH_SUPPLY_SCALE", 0) or 0)
    if sup and sup != 1.0:  # (reliability nudge) a weaker / stronger battery: motor torque and no-load speed scale
        for k_ in ("m_stall_nm", "l_stall_nm", "m_no_load_rad_s", "servo_torque_nm", "servo_w0_rad_s"):
            from csim import Params as _P
            prm[k_] = float(prm.get(k_, getattr(_P(), k_))) * sup
    if a.delay:  # (reliability nudge) every motor command after the first fires this much later (v12; it used to be
        # added to settle_s, which shifted nothing: the program clock starts at the end of the settle)
        prm["program_late_s"] = float(prm.get("program_late_s", 0.0)) + a.delay
    t0 = time.time()
    s = CrossSim(a.design, a.obstacle, a.level, prm)
    if a.start_dy:  # the car placed off the centreline (reliability nudge): move every spawn pose sideways
        import numpy as np
        s.world = {n: (pos + np.array([0.0, a.start_dy, 0.0]), R) for n, (pos, R) in s.world.items()}
        s._build()
    t_max = s.ob.time_limit_s + OB.FINAL_SETTLE_S + 0.5
    vid = None
    if a.video:
        os.makedirs(a.video_dir, exist_ok=True)
        vid = os.path.join(a.video_dir, "run.mp4")
    isaac = bool(vid) and os.environ.get("CSIM_RENDER", "mujoco") == "isaac"
    rec_path = a.record or (os.path.join(a.video_dir, "record.json") if isaac else None)
    if rec_path:
        s.record = []
    r = s.run(t_max, None if isaac else vid, fps=20, cams=("follow", "side"), stop_on_result=True)
    if rec_path:
        json.dump({"design": os.path.abspath(a.design), "bench": a.obstacle, "level": a.level, "fps": 20,
                   "start_gap_mm": s.design.start_gap_mm, "frames": s.record}, open(rec_path, "w"))
    if isaac:  # render the recorded run in Isaac Sim (RTX), as the benchmark's earlier videos
        import subprocess
        root = os.path.join(H, "..", "ai_pd_tasks")
        env = dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES", UV_NO_SYNC="1")
        rr = subprocess.run([os.path.join(root, ".venv", "bin", "python"), "scripts/render_replay.py", "--record",
                             os.path.abspath(rec_path), "--video_dir", os.path.abspath(a.video_dir),
                             "--kit_args=--/rtx/verifyDriverVersion/enabled=false"], cwd=root, env=env,
                            capture_output=True, text=True, timeout=1800)
        if not os.path.exists(os.path.join(a.video_dir, "follow.mp4")):  # fall back to the simulator's own renderer
            open(os.path.join(a.video_dir, "render_replay.log"), "w").write(rr.stdout[-4000:] + rr.stderr[-4000:])
            s2 = CrossSim(a.design, a.obstacle, a.level, prm)
            s2.run(t_max, vid, fps=20, cams=("follow", "side"), stop_on_result=True)
    if vid:  # the harness expects follow.mp4 and overview.mp4
        for src_cam, dst in (("follow", "follow"), ("side", "overview")):
            p = vid.replace(".mp4", f"_{src_cam}.mp4")
            if os.path.exists(p):
                os.replace(p, os.path.join(a.video_dir, f"{dst}.mp4"))
    out = {k: r.get(k) for k in ("obstacle", "level", "crossed", "time_s", "reached_goal_s", "max_progress",
                                  "max_progress_m", "failure")}
    out["crossed"] = bool(r.get("crossed") or r.get("passed"))
    if out.get("failure") is None and not out["crossed"]:
        out["failure"] = r.get("failure") or "time limit"
    out["max_progress"] = r.get("max_progress") if r.get("max_progress") is not None else 0.0
    out["simulator"] = "crossbot-mujoco"
    out["params_file"] = src
    out["diverged"] = r.get("diverged")
    out["broken_joints"] = [e for _, e in r.get("events", []) if e.startswith("stud released")][:20]
    out["gear_meshes"] = r.get("mesh")
    out["chains"] = r.get("chains")  # (v12) per chain: peak tension, sprocket gap range (vs as built), skips
    out["mesh_events_dropped"] = r.get("mesh_events_dropped")
    out["leads"] = r.get("cables")
    out["mechanisms"] = r.get("mechanisms")  # (v13) racks, shocks, treads, differentials
    out["stud_peak_load_ratio"] = r.get("stud_peak_n")
    out["motor_program_log"] = [f"{t:.2f} s {e}" for t, e in r.get("events", [])][:60]
    out["valid"] = True
    out["wall_s"] = round(time.time() - t0, 1)
    json.dump(out, open(a.out, "w"), indent=1, default=str)
    print(json.dumps({k: out[k] for k in ("crossed", "failure", "time_s", "max_progress")}))


if __name__ == "__main__":
    main()
