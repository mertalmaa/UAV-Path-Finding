import sys, json, time
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stage38_5_lib import (build_f_wide, make_roi, make_config, ara_star_generic, state_path_report,
                            msl_to_z_index, compute_distance_reference, independent_safety_replay,
                            production_mission_policy)
from planner.primitives import build_primitive_set
from planner.astar import _generate_neighbors
from planner.terrain import TerrainQuery

spec = build_f_wide(cruise_cols=80)
print(f"F-WIDE grid shape={spec['elevation'].shape} width={spec['width']} "
      f"transition_cols_each_way={spec['transition_cols_each_way']} cruise_cols={spec['cruise_cols']} "
      f"start={spec['start_rc']} goal={spec['goal_rc']}")

terrain = TerrainQuery(make_roi(spec["elevation"]))
cfg = make_config(10.0, 10.0, spec["min_msl"])
primitives = build_primitive_set(cfg)
z = msl_to_z_index(spec["start_msl"], cfg)
start = (spec["start_rc"][0], spec["start_rc"][1], z)
goal = (spec["goal_rc"][0], spec["goal_rc"][1], z)
d_ref = compute_distance_reference(start, goal, terrain, cfg)
policy = production_mission_policy(spec["min_msl"])

cache = {}
stats_cache = {"hits": 0, "misses": 0, "actual_calls": 0}
def neighbor_fn(s):
    neighbors, *_ = _generate_neighbors(s, primitives, terrain, cfg, spec["min_msl"], spec["max_msl"], cache, stats_cache, d_ref, None, None, None, True, None)
    return neighbors

t0 = time.perf_counter()
opt_phases = ara_star_generic(start, goal, terrain, spec["min_msl"], spec["max_msl"], cfg,
                               epsilon_schedule=(1.0,), max_expansions_cumulative=1_000_000, neighbor_fn=neighbor_fn)
opt_wall = time.perf_counter() - t0
opt_p = opt_phases[-1]
opt_q = state_path_report(opt_p.incumbent_path, terrain, cfg, policy, d_ref)
opt_safety = independent_safety_replay(opt_p.incumbent_path, terrain, cfg)
print(f"\nGRAPH OPTIMUM (eps=1.0): cost={opt_p.incumbent_cost:.6f} exp={opt_p.cumulative_expansions} "
      f"complete={opt_p.phase_complete} wall={opt_wall:.1f}s mean_msl={opt_q['mean_msl']:.1f} min_msl={opt_q['min_msl']:.0f} "
      f"xy={opt_q['xy']:.1f}m safe={opt_safety['safe']} preferred={spec['preferred'](opt_q)}")

# descent/cruise/climb distance breakdown for the optimal path
path = opt_p.incumbent_path
from planner.astar import z_index_to_msl
rows_cols_msl = [(s[0], s[1], z_index_to_msl(s[2], cfg)) for s in path]
descent_cols = sum(1 for a, b in zip(rows_cols_msl, rows_cols_msl[1:]) if b[2] < a[2])
climb_cols = sum(1 for a, b in zip(rows_cols_msl, rows_cols_msl[1:]) if b[2] > a[2])
low_cruise_cols = sum(1 for r, c, m in rows_cols_msl if m <= spec["min_msl"] + 1e-6)
print(f"  path shape: n_states={len(path)} descent_steps={descent_cols} climb_steps={climb_cols} "
      f"states_at_floor_msl({spec['min_msl']:.0f})={low_cruise_cols}")

t0 = time.perf_counter()
ara_phases = ara_star_generic(start, goal, terrain, spec["min_msl"], spec["max_msl"], cfg,
                               epsilon_schedule=(1.70, 1.50, 1.30), max_expansions_cumulative=50_000, neighbor_fn=neighbor_fn)
ara_wall = time.perf_counter() - t0

print(f"\nARA* eps=1.7->1.5->1.3:")
results = {"grid_width": spec["width"], "optimal": dict(cost=opt_p.incumbent_cost, exp=opt_p.cumulative_expansions,
           mean_msl=opt_q["mean_msl"], min_msl=opt_q["min_msl"], xy=opt_q["xy"], safe=opt_safety["safe"],
           descent_steps=descent_cols, climb_steps=climb_cols, low_cruise_states=low_cruise_cols, wall=opt_wall),
           "ara_rows": []}
for ph in ara_phases:
    q = state_path_report(ph.incumbent_path, terrain, cfg, policy, d_ref)
    is_pref = bool(q and spec["preferred"](q))
    safety = independent_safety_replay(ph.incumbent_path, terrain, cfg)
    ratio = ph.incumbent_cost / opt_p.incumbent_cost
    print(f"  eps={ph.epsilon:.2f}: cum_exp={ph.cumulative_expansions} cost={ph.incumbent_cost:.6f} "
          f"found/opt={ratio:.4f} mean_msl={q['mean_msl']:.1f} preferred={is_pref} safe={safety['safe']} "
          f"OPEN={ph.open_size_at_end} INCONS={ph.incons_size_at_end} phase_complete={ph.phase_complete} "
          f"first_improve_exp={ph.first_incumbent_improvement_expansion}")
    results["ara_rows"].append(dict(epsilon=ph.epsilon, cum_exp=ph.cumulative_expansions, cost=ph.incumbent_cost,
                                     ratio=ratio, mean_msl=q["mean_msl"], preferred=is_pref, safe=safety["safe"],
                                     open=ph.open_size_at_end, incons=ph.incons_size_at_end))

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "outputs", "stage38_5_fwide_isolation.json"), "w") as f:
    json.dump(results, f, indent=2, default=str)
print("\nSaved.")
