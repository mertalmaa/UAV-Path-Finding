import sys, json, time
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stage38_5_lib import (original_fgh_specs, make_roi, make_config, ara_star_generic, state_path_report,
                            msl_to_z_index, compute_distance_reference, independent_safety_replay,
                            production_mission_policy)
from planner.primitives import build_primitive_set
from planner.astar import _generate_neighbors
from planner.terrain import TerrainQuery

specs = original_fgh_specs()
results = {}

for name, spec in specs.items():
    terrain = TerrainQuery(make_roi(spec["elevation"]))
    results[name] = {}
    for angle in (10, 20, 30, 45):
        cfg = make_config(angle, angle, spec["min_msl"])
        primitives = build_primitive_set(cfg)
        z = msl_to_z_index(spec["start_msl"], cfg)
        start = (spec["start_rc"][0], spec["start_rc"][1], z)
        goal = (spec["goal_rc"][0], spec["goal_rc"][1], z)
        d_ref = compute_distance_reference(start, goal, terrain, cfg)
        policy = production_mission_policy(spec["min_msl"])

        cache = {}
        stats_cache = {"hits": 0, "misses": 0, "actual_calls": 0}
        def neighbor_fn(s, terrain=terrain, cfg=cfg, primitives=primitives, cache=cache, stats_cache=stats_cache, spec=spec, d_ref=d_ref):
            neighbors, *_ = _generate_neighbors(s, primitives, terrain, cfg, spec["min_msl"], spec["max_msl"], cache, stats_cache, d_ref, None, None, None, True, None)
            return neighbors

        t0 = time.perf_counter()
        opt_phases = ara_star_generic(start, goal, terrain, spec["min_msl"], spec["max_msl"], cfg,
                                       epsilon_schedule=(1.0,), max_expansions_cumulative=400_000, neighbor_fn=neighbor_fn)
        opt_wall = time.perf_counter() - t0
        opt_p = opt_phases[-1]
        opt_q = state_path_report(opt_p.incumbent_path, terrain, cfg, policy, d_ref)
        opt_safety = independent_safety_replay(opt_p.incumbent_path, terrain, cfg)

        t0 = time.perf_counter()
        ara_phases = ara_star_generic(start, goal, terrain, spec["min_msl"], spec["max_msl"], cfg,
                                       epsilon_schedule=(1.70, 1.50, 1.30), max_expansions_cumulative=12_000, neighbor_fn=neighbor_fn)
        ara_wall = time.perf_counter() - t0

        ara_rows = []
        first_preferred_exp = None
        for ph in ara_phases:
            q = state_path_report(ph.incumbent_path, terrain, cfg, policy, d_ref)
            is_pref = bool(q and spec["preferred"](q))
            if is_pref and first_preferred_exp is None:
                first_preferred_exp = ph.cumulative_expansions
            safety = independent_safety_replay(ph.incumbent_path, terrain, cfg)
            ara_rows.append(dict(epsilon=ph.epsilon, cum_exp=ph.cumulative_expansions,
                                  first_improve_exp=ph.first_incumbent_improvement_expansion,
                                  cum_time=ph.cumulative_runtime_s, cost=ph.incumbent_cost,
                                  mean_msl=q["mean_msl"] if q else None, min_msl=q["min_msl"] if q else None,
                                  xy=q["xy"] if q else None, preferred=is_pref,
                                  open=ph.open_size_at_end, incons=ph.incons_size_at_end,
                                  phase_complete=ph.phase_complete, safe=safety["safe"]))

        results[name][angle] = dict(
            primitive_count=len(primitives),
            optimal=dict(cost=opt_p.incumbent_cost, exp=opt_p.cumulative_expansions, wall=opt_wall,
                         phase_complete=opt_p.phase_complete, mean_msl=opt_q["mean_msl"], min_msl=opt_q["min_msl"],
                         xy=opt_q["xy"], length_3d=opt_q["length_3d"], climb=opt_q["climb"], descent=opt_q["descent"],
                         n_states=opt_q["n_states"], safe=opt_safety["safe"], max_angle_used=opt_safety["max_angle"]),
            ara=dict(rows=ara_rows, wall=ara_wall, final_cost=ara_phases[-1].incumbent_cost,
                     found_over_optimal=ara_phases[-1].incumbent_cost / opt_p.incumbent_cost,
                     first_preferred_exp=first_preferred_exp),
        )
        print(f"{name} angle={angle}: optimal cost={opt_p.incumbent_cost:.6f} exp={opt_p.cumulative_expansions} "
              f"mean_msl={opt_q['mean_msl']:.1f} wall={opt_wall:.1f}s || ARA* final cost={ara_phases[-1].incumbent_cost:.6f} "
              f"found/opt={ara_phases[-1].incumbent_cost/opt_p.incumbent_cost:.4f} first_pref_exp={first_preferred_exp} "
              f"final_exp={ara_phases[-1].cumulative_expansions}")

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "outputs", "stage38_5_fixed_envelope_sweep.json"), "w") as f:
    json.dump(results, f, indent=2, default=str)
print("\nSaved.")
