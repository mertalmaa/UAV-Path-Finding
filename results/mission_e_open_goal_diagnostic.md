# Mission E: why a goal-satisfying state waits in OPEN

## Baseline

- Reproduced: **True**; status `EXPANSION_LIMIT`.
- Expanded / generated / peak OPEN: 30,000 / 149,995 / 21,356.
- `enable_combined_turns`: `False`.

## First goal-satisfying OPEN insertion

- Expansion 28618; primitive `STRAIGHT_DESCENT`; key `{'x_bin': 11561, 'y_bin': 69777, 'z_bin': 780, 'heading_bin': 6}`.
- g / h / f: 9266.172 / 0.000 / 9266.172.
- XY / |Z| / 3D error: 60.000 / 4.364 / 60.158 m.
- Pose: (693680.844, 4186648.056, 3904.364, heading 90.0 deg).
- Raw heap size/rank immediately before insertion: 20,511 / 20,501 (raw heap includes stale entries).

## Active goal states at 30k

- Active non-stale goal-satisfying states: 9; minimum g / h / f: 9263.809 / 0.000 / 9263.809.
- Lowest-f state g / h / f: 9263.809 / 0.000 / 9263.809; active OPEN rank 20026.
- Lowest-f pose: (693676.460, 4186671.176, 3904.364, heading 75.0 deg); primitive `LEFT_LEVEL_TURN`; XY / |Z| 68.409 / 4.364 m.

## What popped after first goal insertion

- Pops: 1,382; f < first-goal f: 1,382; f ~= first-goal f: 0; f > first-goal f: 0.
- g median 6078.15; h median 3157.98; f median 9236.06.

| Distribution | g | h | f | XY error | |Z| error | 3D error |
|---|---:|---:|---:|---:|---:|---:|
| min | 3674.56 | 30.00 | 9235.85 | 120.00 | 4.36 | 120.08 |
| median | 6078.15 | 3157.98 | 9236.06 | 3243.60 | 150.37 | 3248.81 |
| max | 9206.17 | 5561.33 | 9236.20 | 5629.91 | 500.00 | 5652.07 |

## Production ordering and goal check

- Heap tuple: `(f_score, insertion_counter, node_id)`.
- Equal f values use insertion order; neither h nor g receives a secondary preference.
- Goal check location: after an active non-stale OPEN node is popped, before successor expansion. It is not checked during successor generation.

## Heuristic and cost

- `h = sqrt(max(|dx|-xy_tol,0)^2 + max(|dy|-xy_tol,0)^2 + max(|dz|-z_tol,0)^2)`.
- `g` is the sum of sampled geometric 3D trajectory lengths; it has no altitude, terrain, or other penalty in this pose-aware core.
- Goal-state g decomposition: physical 9263.809 m; penalty 0.000 m; matches g `True`.

| Representative | XY remaining m | |Z| remaining m | g | h | f |
|---|---:|---:|---:|---:|---:|
| start | 9300.00 | 500.00 | 0.00 | 9223.03 | 9223.03 |
| 25_percent_xy_progress | 6961.34 | 277.51 | 2354.65 | 6876.43 | 9231.08 |
| 50_percent_xy_progress | 4673.91 | 398.29 | 4635.93 | 4600.07 | 9236.01 |
| 75_percent_xy_progress | 2340.00 | 131.29 | 6979.48 | 2253.27 | 9232.74 |
| near_goal_wrong_z | 120.00 | 10.71 | 9205.84 | 30.01 | 9235.85 |
| correct_z_farther_xy | 1020.00 | 4.36 | 8306.17 | 930.00 | 9236.17 |
| first_goal_satisfying_open | 60.00 | 4.36 | 9266.17 | 0.00 | 9266.17 |

## F congestion and frontier diversity

- Active states with f below first-goal f: 20,029.

| Band around first-goal f | Active entries | XY bins | Z bins | Heading bins |
|---|---:|---:|---:|---:|
| +/- 0.1% | 10 | 1 | 5 | 3 |
| +/- 0.5% | 20,039 | 493 | 85 | 5 |
| +/- 1% | 20,047 | 494 | 85 | 5 |
| +/- 5% | 20,051 | 495 | 85 | 5 |

- Active f <= lowest-goal-f: 20,027 entries across 493 XY bins; mean/median Z bins per XY 29.61/29.00; altitude range 3898.02-4400.00 m MSL; mean heading bins/XY 1.76.

## Counterfactual rankings (offline only)

- Current `(f, counter)`: 20,026.
- Current f then lower h: 20,026.
- Current f then larger g: 20,026.
- Pure h: 2.

## Classification

**PRIMARY: D. Z DIVERSITY CREATES TOO MANY LOWER-F STATES**

**SECONDARY: A. HEURISTIC TOO WEAK FOR VERTICAL-TIMING PROGRESS**

All 1,382 pops after first goal insertion had strictly lower f; no equal-f or f-higher state was popped. The geometric heuristic leaves a broad f band while it does not encode vertical-timing progress. Exact-tie counterfactuals do not improve the goal rank, while pure-h would rank it 2nd. This rules out a heap/stale bug and makes insertion-order tie-breaking non-causal in this run.

Production behavior changed: **NO**.
