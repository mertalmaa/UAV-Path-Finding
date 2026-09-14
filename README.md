# UAV Pathfinder

Terrain-aware 3D path-planning research code for a fixed-wing UAV. The current
planner combines conservative terrain/AGL checks, sparse CandidateZ altitude
events, explicit-target motion primitives, and a measured planner-safe C172P
aircraft profile.

## Repository layout

- `planner/` — production planning code.
- `tests/` — small canonical regression suite.
- `scripts/` — current data preparation and benchmark runners only.
- `jsbsim/` — offline C172P characterization pipeline and canonical profile.
- `working_dem/` — working terrain rasters used by the planner.
- `results/` — current planner benchmark evidence.
- `outputs/terrain_cache/` — reproducible terrain cache used by the real-terrain benchmark.
- `project.md` — current architecture, decisions, status, and blockers.
- `docs/HISTORY.md` — concise summary of superseded approaches.

## Quick validation

From the repository root:

```powershell
python -m unittest discover -v
```

This runs only fast synthetic/current-contract tests. It does not run Mission
B or C.

The canonical runtime aircraft artifact is:

```text
jsbsim/results/c172p_aircraft_profile_planner_safe_v3.json
```

Long real-terrain missions are benchmark workloads, not part of the quick test
suite. Mission A can be run with
`python -m scripts.benchmark_missions --mission A`; see `project.md` before
running the opt-in Mission B workload.
