"""Stage 27: real 6.48km Aladagalar benchmark -- single new run at
epsilon_search=1.05 (target_suboptimality=1.05 fixed), the final point in
the Stage 25/26 epsilon sweep (1.5, 1.2, 1.1 already recorded in
project.md, NOT re-run here). NO astar.py/heuristic/cost/reopen/dominance
changes -- reuses Stage 26's exact run_one() harness and settings.
"""
import dataclasses
import math

from planner.astar import msl_to_z_index, validate_and_cost_path
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.benchmark_weighted_astar_epsilon_sweep import (
    AIRCRAFT_MSL, GOAL_COL, GOAL_ROW, START_COL, START_ROW, W_MSL, run_one,
)

EPSILON_SEARCH = 1.05


def main() -> None:
    cfg = dataclasses.replace(DEFAULT_CONFIG, msl_cost_weight=W_MSL)
    primitives = build_primitive_set(cfg)

    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    seg = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    seg_min = float(seg.min())

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, GOAL_COL, z0)

    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0

    direct_level_path = [(r, START_COL, z0) for r in range(START_ROW, GOAL_ROW + 1)]
    ok, incumbent_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg)
    print(f"initial incumbent (direct 216-edge level chain): valid={ok} "
          f"edges={len(direct_level_path) - 1} cost={incumbent_cost:.2f}")
    if not ok:
        print("!! Direct level path rejected -- falling back to incumbent_cost=inf")
        incumbent_cost, direct_level_path = math.inf, None
    print()

    run_one(EPSILON_SEARCH, cfg, primitives, tq, start, goal, min_search, max_search,
            incumbent_cost, direct_level_path)


if __name__ == "__main__":
    main()
