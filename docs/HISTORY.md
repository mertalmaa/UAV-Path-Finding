# Historical architecture summary

Git history contains the detailed stage reports and raw intermediate outputs.
This file retains only decisions that help explain the current design.

## Altitude representation

The first planner used a global regular `z_step_m` lattice and fixed
climb/descent deltas. CandidateZ began as a sparse terrain-floor/ceiling event
generator and later became the production altitude-successor source when its
generator is supplied to search. Off-lattice mission altitudes are encoded as
stable integer keys, and explicit-target primitives preserve the requested
physical altitude without snapping.

The regular-Z search path still exists as a compatibility path when no
CandidateZ generator is provided. The independent coarse planner also retains
its own dense representation. Removing those paths is production work and was
intentionally not attempted during repository hygiene cleanup.

## Search state and experimental cost terms

Earlier experiments added trend/bucket/reversal history and dominance logic to
the search state. They were removed after showing that physical `(row, col,
altitude)` identity was sufficient and the extra state multiplied equivalent
nodes. Historical tuning CSVs and one-off comparison scripts are no longer kept
in the working tree.

Weighted A*, ARA*, coarse guides, and hard-corridor experiments established
useful performance behavior, but their numerous per-stage outputs were not
canonical artifacts. The production modules remain in `planner/`; old run
outputs and stage-specific harnesses were removed.

## Grid decision

Terrain-only 30/60/90 m comparisons were later re-gated with measured aircraft
turn geometry. The retained decision is 60 m as the default planning scale, 30
m for local refinement, and 90 m only for global/optional metadata. Grid
resolution is representation; it is not aircraft turn radius or motion length.

## Aircraft characterization

The early F-16 methodology and C172R/U4 investigations were superseded by a
stock C172P at nominal 40 m/s IAS. The current planner consumes the tested-safe
V3 profile. It keeps measured evidence separate from planner-safe availability,
uses altitude-dependent climb/descent capability, keeps left/right turns
separate, and rejects raw characterization artifacts at the runtime loader.

## Removed interfaces

The abandoned web/UI layer and its launcher configuration were removed. The
repository is now centered on the planner library, offline characterization,
canonical tests, and explicit benchmark runners.

## Valley-relative guidance and 90 km scaling (2026-09-16)

On the 90 km Bilecik ROI the planner flew near-straight lines (sinuosity ~1.01)
and, with the low-altitude AGL cost enabled, expansions grew ~10x with one
timeout. Diagnosis: the absolute-elevation guidance cost was normalised by the
whole-ROI relief, so its valley contrast vanished at 90 km (guide route mean
ground 474 m vs 489 m with no cost at all); the AGL term only affected z, which
the profile stage already optimises, and flooded 5 m z bins; g ignored the
guidance multiplier unless the AGL cost was on, so the search did not really
follow the guide.

Fix, now the default: `valley_relative` guidance cost (height above a 5 km local
floor, alpha 2.0), guidance multiplier in g, AGL cost off in A*, weight 1.3,
scipy Dijkstra for large guidance grids, round-robin counted on real
expansions. Rejected during tuning: `search_z_bin_m=25` (broke goal altitude
convergence), `guidance_queue_ratio=10`, a 3 km default ROI-edge margin
(pushed the 31 km canyon route onto ridges), alpha 2.5 without edge margin
(M90_02 2x detour).

## Repository cleanup (2026-09-16)

Removed `experiments/`, `scratch/` and superseded runners
(`scripts/benchmark_*`, `final_low_altitude_evaluation.py`,
`check_bilecik_stage1.py`, `run_long_31km_mission.py`,
`run_5_long_30km_structures.py`, the old `run_bilecik_90km_5_missions.py`).
The valley-guided 90 km runner took over the `run_bilecik_90km_5_missions.py`
name. Result files from removed runners remain under `results/` as archive.
