"""Stage 11: mini re-tuning of msl_cost_weight under the new fixed-scale
MSL normalization (Stage 10). No new cost terms, no formula changes --
vertical_cost_weight is held fixed at 1.0, only msl_cost_weight varies
over a small, deliberately narrow set: 0.0, 0.025, 0.05, 0.10.

Same real Aladaglar ROI scenario as Stages 9 and 10 (row=80, cols 90-128,
prototype cruise altitude derived from this segment's own terrain +
min_agl_m), so only the effect of msl_cost_weight is being measured.

weighted total cost is reported but NOT used to compare weights against
each other (it isn't comparable across different weights -- see Stage 9).
The physically-interpretable metrics are average/min/max MSL, geometric
path length, vertical motion, minimum observed AGL, and runtime/expanded
nodes.
"""
import csv
import dataclasses
import math
import time

from planner.astar import astar_search, msl_to_z_index
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery

MSL_WEIGHTS = (0.0, 0.025, 0.05, 0.10)
VERTICAL_WEIGHT = 1.0
CSV_PATH = "retuning_results.csv"


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)

    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    # Identical scenario construction to Stage 9 / Stage 10.
    row_idx, c_start, c_goal = 80, 90, 128
    seg = roi.elevation[row_idx, c_start:c_goal + 1]
    seg_min, seg_max = float(seg.min()), float(seg.max())
    cruise_msl = math.ceil((seg_max + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m + 20.0
    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = cruise_msl + 20.0
    z0 = msl_to_z_index(cruise_msl, cfg)
    start, goal = (row_idx, c_start, z0), (row_idx, c_goal, z0)

    print("=== Stage 11: mini re-tuning of msl_cost_weight (vertical_cost_weight fixed at 1.0) ===")
    print(f"Real Aladaglar ROI, row={row_idx}, col {c_start}->{c_goal} "
          f"({(c_goal - c_start) * cfg.xy_resolution_m:.0f}m), segment terrain "
          f"min={seg_min:.1f}m max={seg_max:.1f}m")
    print(f"PROTOTYPE cruise_msl={cruise_msl:.0f}m, search bounds=[{min_search:.0f},{max_search:.0f}]m "
          f"(z-grid aligned, {cfg.z_step_m}m step) -- same scenario as Stage 9/10, NOT a real mission altitude/route.")
    print(f"altitude_scaled at cruise = {cruise_msl / cfg.msl_scale_m:.3f}")
    print()

    rows = []
    all_safe = True
    for w_msl in MSL_WEIGHTS:
        c = dataclasses.replace(cfg, msl_cost_weight=w_msl, vertical_cost_weight=VERTICAL_WEIGHT)
        t0 = time.perf_counter()
        r = astar_search(start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
                          config=c, primitives=primitives, max_expansions=150_000)
        runtime_s = time.perf_counter() - t0

        safe = r.success and r.minimum_observed_agl >= cfg.min_agl_m - 1e-6
        all_safe = all_safe and (r.success and safe)

        row = {
            "w_MSL": w_msl, "status": r.status, "path_len": round(r.geometric_path_length, 2) if r.success else "",
            "avg_MSL": round(r.average_aircraft_msl, 2) if r.success else "",
            "min_MSL": round(r.minimum_aircraft_msl, 2) if r.success else "",
            "max_MSL": round(r.maximum_aircraft_msl, 2) if r.success else "",
            "climb": round(r.total_climb_m, 2) if r.success else "",
            "descent": round(r.total_descent_m, 2) if r.success else "",
            "vertical_total": round(r.total_vertical_motion_m, 2) if r.success else "",
            "min_AGL": round(r.minimum_observed_agl, 2) if r.success else "",
            "expanded": r.expanded_nodes, "generated": r.generated_neighbors,
            "max_open": r.max_open_size, "runtime_s": round(runtime_s, 2),
            "weighted_cost": round(r.total_cost, 2) if r.success else "",
        }
        rows.append(row)
        print(f"w_MSL={w_msl}: status={r.status} path_len={row['path_len']} avg_MSL={row['avg_MSL']} "
              f"min_MSL={row['min_MSL']} max_MSL={row['max_MSL']} climb={row['climb']} descent={row['descent']} "
              f"vertical_total={row['vertical_total']} min_AGL={row['min_AGL']} expanded={r.expanded_nodes} "
              f"generated={r.generated_neighbors} max_open={r.max_open_size} runtime={runtime_s:.2f}s "
              f"weighted_cost={row['weighted_cost']}")

    print()
    print("=== Summary table ===")
    cols = ["w_MSL", "path_len", "avg_MSL", "max_MSL", "vertical_total", "min_AGL", "expanded", "runtime_s"]
    widths = {"w_MSL": 7, "path_len": 9, "avg_MSL": 8, "max_MSL": 8, "vertical_total": 15, "min_AGL": 8,
              "expanded": 9, "runtime_s": 10}
    print("".join(f"{c:>{widths[c]}}" for c in cols))
    for row in rows:
        print("".join(f"{str(row[c]):>{widths[c]}}" for c in cols))

    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print()
    print(f"Wrote {CSV_PATH}")
    print(f"All successful runs kept minimum_observed_agl >= {cfg.min_agl_m}m: {all_safe}")


if __name__ == "__main__":
    main()
