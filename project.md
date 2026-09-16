# UAV Pathfinder — current source of truth

Last reviewed: 2026-09-16.

This document describes the repository as it exists now. Earlier stage logs are
summarised in `docs/HISTORY.md`; Git history is the archive for removed
experiments and outputs.

## Mission contract

The planner seeks a safe fixed-wing path between mission endpoints over terrain.
A valid path stays inside the ROI bounds, keeps `min_agl_m` clearance (with the
configured lateral buffer) along every sampled primitive, and uses motion that
is feasible for the kinematic envelope.

- `FOUND` — a path was found; it is still validated by the profile stage.
- `OPEN_EXHAUSTED` (`no_path`) — frontier exhausted within the modelled space.
- `TIMEOUT` / `EXPANSION_LIMIT` — budget outcomes, not reachability claims.

**directly infeasible != unreachable** and **timeout != unreachable**.

## Frozen flight / trajectory contract

- Pose: `(x_m, y_m, z_msl_m, heading_deg)` in EPSG:32636 (UTM 36N), altitude MSL,
  navigation heading (North=0, clockwise), canonical value in `[0, 360)`.
- Zero wind, fixed 40.0 m/s planar speed; primitive duration = horizontal arc
  length / 40.
- Physical poses and trajectories stay continuous. DEM row/col is terrain
  indexing only; a `SearchKey` is bookkeeping only and never moves a pose.
- Safety samples the actual continuous trajectory; turns add circular-arc
  sagitta to chord coverage. `lateral_buffer_m` is a separate cached max-filter
  terrain field. Outside-ROI, NoData and below-AGL samples fail closed. The AGL
  boundary is inclusive.
- `DEFAULT_CONFIG.min_agl_m` is 200 m; Bilecik mission scripts use 100 m with a
  60 m lateral buffer and report their effective values.

## Kinematic envelope (`planner.fixed_wing_envelope`)

40 m/s, ±5 m/s climb/descent at all altitudes, 25° bank (R ≈ 349.89 m,
≈ 6.55°/s). A 60 m primitive lasts 1.5 s (Δz = ±7.5 m).

## Current architecture

### Search (`planner.pose_search`)

- Successors: 60 m straight level, 15° left/right level turns, 60 m straight
  climb/descent and, with `enable_combined_turns=True` (default), four 15°
  helical climbing/descending turns. Spiral/loiter/U-turn macros are not
  successors.
- `SearchKey(x_bin, y_bin, z_bin, heading_bin)` = 60 m / 5 m / 15°. One
  representative per key, replaced only at strictly lower g. Approximate:
  no completeness or optimality guarantee. Cross-altitude pruning was removed;
  `enable_pareto_z_pruning` has no effect.
- Dual-queue round-robin: `guidance_queue_ratio` (3) guided pops per anchor pop,
  counted over real expansions. Anchor heuristic: vertical-reachability 3D
  distance. Weight `search_heuristic_weight` = 1.3.

### Terrain guidance (`_TerrainGuidance`, default on)

- Reverse Dijkstra from the goal on the buffered DEM sampled every 3 px (90 m).
  Edge length is stretched by the climb/descent slope needed for the terrain
  step; targets/ceilings are propagated along the shortest-path tree.
- Cost field (`guidance_cost_mode`):
  - `valley_relative` (default): `1 + alpha·min(HAND/300 m, cap)`, HAND = height
    above the 5 km moving-minimum terrain floor; alpha 2.0, cap 3.0.
    Scale-invariant, so valleys at any absolute altitude are preferred.
  - `absolute_quadratic` (legacy): `1 + 2·((elev−lo)/(hi−lo))²` normalised by
    the whole ROI. Its contrast collapses on 90 km maps (guide route ≈ no-cost
    route).
- Optional ROI-edge repulsion `guidance_edge_margin_m` (default 0, clipped to
  10% of the ROI short side).
- `guidance_multiplier_in_g=True`: the A* g-cost integrates the same multiplier,
  keeping g and the guided heuristic in the same units. Without it the search
  follows the guide greedily and effectively flies the straight line.
- Grids ≥ 40 000 cells use a vectorised `scipy.sparse.csgraph` Dijkstra (tested
  identical to the heapq loop); 90 km build ≈ 1.5 s instead of ≈ 14 s.
- Guidance is soft ordering only; it never grants feasibility.

### Altitude

- The terrain-following profile stage (`planner.altitude_profile`,
  `planner.terrain_following`) solves the lowest safe altitude along the chosen
  ground track under the rate limits. `plan_terrain_following` feeds localized
  profile failures back to search as cost penalties (`max_feedback_passes`).
- `enable_low_altitude_cost` (default False) adds a z-dependent AGL term to g.
  On 90 km it multiplied z bins per XY cell (1.45 → 4.8) and expansions ~10×;
  keep it off unless doing controlled comparisons. When on with the `linear`
  shape, the guided heuristic charges the unavoidable descent ramp.

### Smoothing

`planner.local_trajectory_smoothing.apply_corridor_safe_local_bspline_smoothing`
smooths primitive junctions within a corridor deviation limit and re-checks
terrain.

### Legacy modules

`planner.astar`, `planner.candidate_z`, `planner.coarse`, `planner.transition`,
`planner.vertical_motion` are retained for compatibility tests; they are not
the production motion path.

## Validation

`python -m pytest -q tests` — 97 fast synthetic tests (contracts, safety,
envelope, pose search, guidance incl. valley-relative cost, scipy/heapq
equivalence and the g-multiplier switch, terrain following, smoothing).

## Current mission status (Bilecik)

- 31 km canyon (`scripts/run_single_shot_31km.py`): FOUND, 2002 expansions,
  2.6 s search, 34.62 km, 246–612 m MSL, min AGL 119.7 m.
- 90 km ×5 (`scripts/run_bilecik_90km_5_missions.py`): 5/5 FOUND, 8–13 s search,
  route/straight-line ratio 1.12–1.34, mean MSL 464–760 m (previous straight-line routes
  634–1019 m). JSON: `results/test_bilecik/bilecik_90km_5_missions_valley_benchmark.json`.
- Other scripts in `scripts/` were not re-run with the new defaults.

## Open issues

1. Smoothed max roll rate 43.6°/s exceeds the 15°/s cap on every 90 km mission
   (pre-existing); raw 31 km path shows 62°/s and 0.97 g vertical accel.
2. Detour vs. altitude trade-off is controlled by `valley_cost_alpha`; 2.5 or 3
   gives lower routes but up to 2× length on M90_02.
3. No spiral/U-turn macros; 15° is the only turn increment (zig-zag in gently
   curving valleys).
4. Search altitude during A* stays high (e.g. ~1300 m on M90_01); final altitude
   depends entirely on the profile stage.
