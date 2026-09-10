"""ARA* epsilon-bound sanity check (item 1 of the Stage 38.3 bottleneck plan).

For each Stage 38.3 synthetic search scenario (F/G/H, plus A as a control),
runs the SAME production epsilon schedule (1.70, 1.50, 1.30) used by
validate_stage38_3_mission_generalization.py, and separately runs
epsilon_schedule=(1.0,) with a large expansion cap. epsilon=1.0 makes
ara_star_search degenerate into plain weighted-A* with weight 1 -- i.e.
ordinary A* -- which is provably optimal for an admissible+consistent
heuristic. That gives a real, code-derived ground truth instead of a
hand-picked "preferred" candidate, so:

    found_cost(eps) / true_optimal_cost <= eps   (allowing float slack)

is checked directly. A violation would mean a genuine ARA* bug (stale
OPEN/INCONS keys, wrong termination test, etc). All values within bound
here means the F/G/H "planner missed the valley" behavior on 2026-09-10's
Stage 38.3 run is NOT a bug -- it is the epsilon guarantee doing exactly
its job, just satisfied with room to spare (found/optimal well under the
epsilon ceiling), which shifts the fix away from "tighten epsilon further"
and towards heuristic/search guidance (Stage 38.4 direction).

Run: python scripts/validate_epsilon_bound_control.py
"""
import dataclasses
import time

import numpy as np
from affine import Affine

from planner.astar import ara_star_search, msl_to_z_index
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0
PRODUCTION_EPSILONS = (1.70, 1.50, 1.30)
CONTROL_CAP = 2_000_000  # generous; eps=1.0 must reach phase_complete=True to be trusted as ground truth


def make_roi(elevation):
    h, w = elevation.shape
    return ROIData(elevation.astype(np.float32), Affine(30, 0, 0, 0, -30, h * 30), "EPSG:32636", w, h,
                   (0, 0, w * 30, h * 30), (30, 30), NODATA)


def synthetic_search_specs():
    """Identical grids to validate_stage38_3_mission_generalization.synthetic_search_specs()."""
    a = np.full((9, 35), 100.0)
    f = np.full((17, 65), 200.0)
    f[9:12, :] = 100.0
    g = np.full((19, 65), 260.0)
    g[6:9, :] = 180.0
    g[12:15, :] = 80.0
    g[:, :13] = 80.0
    g[:, 52:] = 80.0
    h = np.full((13, 65), 100.0)
    h[:, :13] = 200.0
    h[:, 52:] = 200.0
    return (
        ("A FLAT", a, (4, 3), (4, 31), 300.0, 300.0, 300.0),
        ("F NARROW SIDE VALLEY", f, (7, 4), (7, 60), 400.0, 300.0, 500.0),
        ("G TWO VALLEYS", g, (7, 4), (7, 60), 400.0, 280.0, 500.0),
        ("H HIGH-VALLEY-HIGH", h, (6, 4), (6, 60), 400.0, 300.0, 500.0),
    )


def run_one(elevation, start_rc, goal_rc, start_msl, min_msl, max_msl, epsilon_schedule, cap):
    config = dataclasses.replace(DEFAULT_CONFIG, normalized_w_altitude=1.25)
    terrain = TerrainQuery(make_roi(elevation))
    cfg = dataclasses.replace(config, cost_mode="normalized", altitude_reference_msl=min_msl,
                              normalized_w_distance=1.0, normalized_w_altitude=1.25,
                              normalized_altitude_scale_m=1000.0, goal_tolerance_xy_m=0.0,
                              goal_tolerance_z_m=0.0)
    primitives = build_primitive_set(cfg)
    z = msl_to_z_index(start_msl, cfg)
    start, goal = (start_rc[0], start_rc[1], z), (goal_rc[0], goal_rc[1], z)
    t0 = time.perf_counter()
    result = ara_star_search(start, goal, terrain, min_msl, max_msl, cfg, primitives,
                              epsilon_schedule=epsilon_schedule, max_expansions_cumulative=cap)
    return result, time.perf_counter() - t0


def main():
    any_violation = False
    for name, elevation, start_rc, goal_rc, start_msl, min_msl, max_msl in synthetic_search_specs():
        print(f"\n===== {name} =====")
        result_prod, _ = run_one(elevation, start_rc, goal_rc, start_msl, min_msl, max_msl,
                                  PRODUCTION_EPSILONS, cap=12_000)
        result_opt, wall_opt = run_one(elevation, start_rc, goal_rc, start_msl, min_msl, max_msl,
                                        (1.0,), cap=CONTROL_CAP)
        true_optimal = result_opt.final_incumbent_cost
        opt_complete = result_opt.phases[-1].phase_complete
        print(f"  eps=1.0 control (ground truth): cost={true_optimal:.6f} "
              f"expansions={result_opt.total_expanded} phase_complete={opt_complete} time={wall_opt:.2f}s")
        if not opt_complete:
            print("  WARNING: control did not reach phase_complete -- raise CONTROL_CAP, result is not trustworthy")
            any_violation = True
            continue
        for phase in result_prod.phases:
            ratio = phase.incumbent_cost / true_optimal
            ok = ratio <= phase.epsilon + 1e-9
            any_violation = any_violation or not ok
            print(f"  eps={phase.epsilon:.2f}: found={phase.incumbent_cost:.6f}  found/optimal={ratio:.4f}  "
                  f"bound<= {phase.epsilon:.2f}  {'OK' if ok else '*** VIOLATION -- ARA* BUG ***'}  "
                  f"OPEN_left={phase.open_size_at_end} INCONS_left={phase.incons_size_at_end}")
    print("\n" + ("*** AT LEAST ONE EPSILON-BOUND VIOLATION -- investigate ARA* implementation ***"
                  if any_violation else
                  "ALL SCENARIOS: found_cost respects its epsilon bound against the true optimum. "
                  "No ARA* bug. Remaining topology miss (F/G) is a heuristic/guidance limitation, not a search bug."))


if __name__ == "__main__":
    main()
