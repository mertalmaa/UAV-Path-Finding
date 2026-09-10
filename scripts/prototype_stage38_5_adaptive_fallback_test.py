import sys
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import dataclasses as dc
from stage38_5_lib import make_roi, make_config, max_feasible_vertical_candidates, AXIAL_STEP
from planner.terrain import TerrainQuery
from planner.astar import msl_to_z_index

print("=== TEST 1: adaptive back-off (steep attempts blocked by a spike, shallower succeeds) ===")
# 1D strip, direction E, start at (row=2, col=2), descending. baseline terrain=100 everywhere
# except a single-cell spike near the start column.
for spike_elev in (280, 240, 220, 200, 180, 150, 100):
    elevation = np.full((5, 20), 100.0)
    elevation[2, 3] = spike_elev  # one cell east of start -- x ~ 30m from start's cell center
    terrain = TerrainQuery(make_roi(elevation))
    cfg = make_config(45.0, 45.0, min_msl_ref := 300.0)
    z = msl_to_z_index(400.0, cfg)
    attempts = []
    # call the search manually to record every n_cells attempt, not just the final pick
    import math
    from planner.primitives import MotionPrimitive, evaluate_primitive
    from planner.astar import state_to_xyz
    start_xyz = state_to_xyz((2, 2, z), terrain, cfg)
    dz = -cfg.z_step_m
    n_min = max(1, math.ceil(abs(dz) / math.tan(math.radians(45.0)) / AXIAL_STEP))
    chosen = None
    for n_cells in range(n_min, n_min + 6):
        horiz = n_cells * AXIAL_STEP
        prim = MotionPrimitive("E", 0, n_cells, dz, horiz, "descent")
        res = evaluate_primitive(start_xyz, prim, terrain, cfg)
        angle = math.degrees(math.atan2(abs(dz), horiz))
        attempts.append((n_cells, round(angle, 2), res.valid, res.reason, round(res.min_agl_m, 1) if res.min_agl_m == res.min_agl_m else None))
        if res.valid and chosen is None:
            chosen = (n_cells, angle)
    print(f"spike_elev={spike_elev}: attempts(n_cells, angle, valid, reason, min_agl)={attempts}")
    print(f"  -> chosen max-feasible: {chosen}")

print("\n=== TEST 2: max-feasible candidate exists AND level/moderate alternative still offered "
      "(planner not forced into greedy max-descent) ===")
# Build a small A* isolation: from start, one direction offers a valid MAX-FEASIBLE descent
# that leads into a dead end (needs a costly climb right after), while LEVEL continues cheaply
# to the goal. Confirm the search picks LEVEL (cheaper total), proving max-feasible is an
# offered CANDIDATE, not a forced/greedy move.
import dataclasses as dc2, math as math2
from stage38_5_lib import ara_star_generic, generate_neighbors_adaptive, state_path_report, compute_distance_reference, production_mission_policy
elevation2 = np.full((5, 12), 200.0)
elevation2[3, :] = 100.0  # a thin low band 1 row south -- but blocked by a wall right after entry
elevation2[3, 6] = 400.0  # a wall partway down the low band forces a costly climb if you dropped in early
cfg2 = make_config(45.0, 45.0, 300.0)
terrain2 = TerrainQuery(make_roi(elevation2))
z2 = msl_to_z_index(400.0, cfg2)
start2 = (2, 1, z2)
goal2 = (2, 10, z2)
d_ref2 = compute_distance_reference(start2, goal2, terrain2, cfg2)
def nfn(s):
    return generate_neighbors_adaptive(s, terrain2, cfg2, 300.0, 500.0, d_ref2, include_moderate=False)
phases = ara_star_generic(start2, goal2, terrain2, 300.0, 500.0, cfg2, epsilon_schedule=(1.0,),
                           max_expansions_cumulative=50_000, neighbor_fn=nfn)
policy2 = production_mission_policy(300.0)
q2 = state_path_report(phases[-1].incumbent_path, terrain2, cfg2, policy2, d_ref2)
rows_used = sorted(set(s[0] for s in phases[-1].incumbent_path))
print(f"optimal cost={phases[-1].incumbent_cost:.6f} mean_msl={q2['mean_msl']:.1f} rows_used={rows_used} "
      f"(row 2 = stayed high/LEVEL-ish, row 3 = dropped into low band)")
print(f"path: {phases[-1].incumbent_path}")
