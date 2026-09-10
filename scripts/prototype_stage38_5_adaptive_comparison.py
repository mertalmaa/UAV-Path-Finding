import sys, json, time
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stage38_5_lib import (original_fgh_specs, make_roi, make_config, ara_star_generic, state_path_report,
                            msl_to_z_index, compute_distance_reference, independent_safety_replay,
                            production_mission_policy, generate_neighbors_adaptive)
from planner.primitives import build_primitive_set
from planner.astar import _generate_neighbors
from planner.terrain import TerrainQuery

specs = original_fgh_specs()
configs = [
    ("A_fixed10", "fixed", 10.0),
    ("B_fixed30", "fixed", 30.0),
    ("C_fixed45", "fixed", 45.0),
    ("D_adaptive30", "adaptive", 30.0),
    ("E_adaptive45", "adaptive", 45.0),
]

results = {}
for name, spec in specs.items():
    terrain = TerrainQuery(make_roi(spec["elevation"]))
    results[name] = {}
    for label, mode, angle in configs:
        cfg = make_config(angle, angle, spec["min_msl"])
        z = msl_to_z_index(spec["start_msl"], cfg)
        start = (spec["start_rc"][0], spec["start_rc"][1], z)
        goal = (spec["goal_rc"][0], spec["goal_rc"][1], z)
        d_ref = compute_distance_reference(start, goal, terrain, cfg)
        policy = production_mission_policy(spec["min_msl"])

        branch_stats = {"generated": 0, "valid": 0, "calls": 0}
        if mode == "fixed":
            primitives = build_primitive_set(cfg)
            cache = {}
            stats_cache = {"hits": 0, "misses": 0, "actual_calls": 0}
            def neighbor_fn(s, terrain=terrain, cfg=cfg, primitives=primitives, cache=cache, stats_cache=stats_cache,
                             spec=spec, d_ref=d_ref, branch_stats=branch_stats):
                neighbors, _rej, gen, rej, *_ = _generate_neighbors(s, primitives, terrain, cfg, spec["min_msl"], spec["max_msl"],
                                                     cache, stats_cache, d_ref, None, None, None, True, None)
                branch_stats["generated"] += gen
                branch_stats["valid"] += len(neighbors)
                branch_stats["calls"] += 1
                return neighbors
            prim_count = len(primitives)
        else:
            def neighbor_fn(s, terrain=terrain, cfg=cfg, spec=spec, d_ref=d_ref, branch_stats=branch_stats):
                return generate_neighbors_adaptive(s, terrain, cfg, spec["min_msl"], spec["max_msl"], d_ref,
                                                    include_moderate=False, stats=branch_stats)
            prim_count = None  # not a fixed precomputed list

        t0 = time.perf_counter()
        ara_phases = ara_star_generic(start, goal, terrain, spec["min_msl"], spec["max_msl"], cfg,
                                       epsilon_schedule=(1.70, 1.50, 1.30), max_expansions_cumulative=20_000,
                                       neighbor_fn=neighbor_fn)
        wall = time.perf_counter() - t0

        rows = []
        first_pref_exp = None
        for ph in ara_phases:
            q = state_path_report(ph.incumbent_path, terrain, cfg, policy, d_ref)
            is_pref = bool(q and spec["preferred"](q))
            if is_pref and first_pref_exp is None:
                first_pref_exp = ph.cumulative_expansions
            safety = independent_safety_replay(ph.incumbent_path, terrain, cfg)
            rows.append(dict(epsilon=ph.epsilon, cum_exp=ph.cumulative_expansions, cost=ph.incumbent_cost,
                              mean_msl=q["mean_msl"] if q else None, min_msl=q["min_msl"] if q else None,
                              xy=q["xy"] if q else None, n_states=q["n_states"] if q else None,
                              climb=q["climb"] if q else None, descent=q["descent"] if q else None,
                              preferred=is_pref, safe=safety["safe"], open=ph.open_size_at_end,
                              incons=ph.incons_size_at_end))

        avg_branch = branch_stats["valid"] / branch_stats["calls"] if branch_stats["calls"] else float("nan")
        avg_gen = branch_stats["generated"] / branch_stats["calls"] if branch_stats["calls"] else float("nan")
        final = ara_phases[-1]
        final_q = state_path_report(final.incumbent_path, terrain, cfg, policy, d_ref)
        results[name][label] = dict(mode=mode, angle=angle, primitive_count=prim_count,
                                     avg_generated=avg_gen, avg_valid=avg_branch, calls=branch_stats["calls"],
                                     final_cost=final.incumbent_cost, final_exp=final.cumulative_expansions,
                                     final_mean_msl=final_q["mean_msl"], final_n_states=final_q["n_states"],
                                     first_pref_exp=first_pref_exp, wall=wall, rows=rows)
        print(f"{name} {label}: avg_valid_succ={avg_branch:.2f} (avg_generated={avg_gen:.2f}) "
              f"final_exp={final.cumulative_expansions} final_cost={final.incumbent_cost:.6f} "
              f"mean_msl={final_q['mean_msl']:.1f} n_states={final_q['n_states']} "
              f"first_pref_exp={first_pref_exp} wall={wall:.2f}s")

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "outputs", "stage38_5_adaptive_comparison.json"), "w") as f:
    json.dump(results, f, indent=2, default=str)
print("\nSaved.")
