import sys, json, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import numpy as np
from ai_pd_tasks.lego_bench.design import Design, validate, Budget, place
def cog(path):
    d = Design.load(path); P = place(d); m = 0; c = np.zeros(3)
    for n, pl in P.items():
        mm = pl.bp.mass_g; m += mm; c += mm * pl.to_world(np.array(pl.bp.com_mm))
    r = validate(d, Budget.for_part_set("inventory")); cap = r.stats.get("capability", {})
    return dict(mass=round(m, 1), com=[round(v, 2) for v in c / m],
                **{k: cap.get(k) for k in ("rope_hang_depth_mm", "rope_roll_deg", "rope_clearance_mm", "tip_lateral_deg", "max_step_mm", "rope_wheel_bottom_above_ground_mm")})
if __name__ == "__main__":
    for p in sys.argv[1:]: print(p, cog(p))
