# Current JSBSim decisions

1. The primary planner aircraft is the stock C172P at nominal 40 m/s IAS.
2. The characterization and replay stack use the same frozen stateless mixture
   policy from `production_mixture_policy.py`.
3. The canonical online artifact is
   `results/c172p_aircraft_profile_planner_safe_v3.json`.
4. Raw observations, categorical validity, and planner-safe capability are
   separate layers. A measured response is never automatically safe.
5. Capability is altitude-dependent. Left/right turns and climb/descent signs
   remain distinct; unsupported interpolation is reported unavailable.
6. The tested-safe V3 vertical envelope is selected from audited command rows,
   stability/repeatability, IAS retention, power reserve, saturation, and
   holdout error—not from raw observed maxima or a global safety factor.
7. Combined climb remains unavailable. Combined descent is exposed only in its
   validated range.
8. JSBSim is an offline characterization and final-replay tool, not an online
   planner expansion dependency.
9. A directly infeasible maneuver does not prove route-level unreachability.

Detailed superseded decisions and stage narratives are available in Git history;
the short architectural transition is recorded in `../docs/HISTORY.md`.
