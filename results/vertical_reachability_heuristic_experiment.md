# Vertical-reachability admissible heuristic experiment

## Admissibility

For a goal-region residual `Dz`, BASIC has `|Vz| <= 5 m/s` and fixed horizontal speed `V=40 m/s`. Thus horizontal path length `H >= Dz*40/5`; displacement also gives `H >= Dxy`. Hence `H >= max(Dxy, Dz*40/5)`, and the triangle inequality gives `L3D >= sqrt(H^2 + Dz^2)`. Goal tolerances are removed before applying both residuals, preventing an overestimate inside the accepted region.

Admissibility proven: **YES**.

## Mission E

| Mode | Status | Expanded | Generated | Rejected | Peak OPEN | Runtime s | Best XY / |Z| m | First goal insertion | Goal pop | Goal OPEN rank | Active lower-f ahead | Expanded Z / XY mean/median | Active Z / XY mean/median | Vertical bound active |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Euclidean | EXPANSION_LIMIT | 30,000 | 149,995 | 95,073 | 21,356 | 31.73 | 120.00 / 4.36 | 28618 | - | 20501 | 20029 | 37.42/37.00 | 29.52/28.00 | 2 (0.01%) |
| Vertical | EXPANSION_LIMIT | 30,000 | 149,995 | 95,076 | 21,352 | 14.91 | 120.00 / 4.36 | 28618 | - | 20462 | 19939 | 37.41/37.00 | 29.52/28.00 | 0 (0.00%) |

- Expansion reduction: 0.00%.
- Peak OPEN reduction: 0.02%.
- Classification: **C. NEUTRAL**.

## A-F regression and path quality

| Mission | Baseline / vertical status | Expanded | Generated | Peak OPEN | Runtime s | Path delta m / % | Min AGL base / vertical | Goal error XY/Z base / vertical |
|---|---|---:|---:|---:|---:|---:|---:|---|
| A_easy_open | FOUND / FOUND | 1,201 / 1,050 | 6,000 / 5,245 | 1,109 / 945 | 0.61 / 1.13 | 0.000 / 0.000% | 134.65283203125 / 134.65283203125 | 82.88/0.00 / 82.88/0.00 |
| B_relief_affected | FOUND / FOUND | 466 / 465 | 2,325 / 2,320 | 388 / 388 | 0.23 / 0.24 | 0.000 / 0.000% | 104.22705078125 / 104.22705078125 | 89.26/6.34 / 89.26/6.34 |
| C_far_south_3km | FOUND / FOUND | 10,789 / 10,539 | 53,940 / 52,690 | 8,698 / 8,442 | 10.91 / 5.63 | 0.000 / 0.000% | 290.36487662790387 / 290.36487662790387 | 89.50/9.47 / 89.50/9.47 |
| D_far_east_3km | FOUND / FOUND | 8,928 / 8,291 | 44,635 / 41,450 | 6,900 / 6,352 | 10.26 / 4.60 | 0.000 / 0.000% | 228.1337890625 / 228.1337890625 | 82.45/6.34 / 82.45/6.34 |
| E_long_descent_9_3km | EXPANSION_LIMIT / EXPANSION_LIMIT | 30,000 / 30,000 | 149,995 / 149,995 | 21,356 / 21,352 | 31.73 / 14.91 | - | - / - | - |
| F_turn_required_diagonal_2_3km | FOUND / FOUND | 5,942 / 5,631 | 29,705 / 28,150 | 3,001 / 2,929 | 3.00 / 3.79 | 0.000 / 0.000% | 109.46497428415387 / 109.46497428415387 | 79.14/9.47 / 79.14/9.47 |

Safety regression: **NO** (the unchanged continuous evaluator accepted every reported FOUND path).

SearchKey changed: **NO**. Z bin changed: **NO**. Cost changed: **NO**. Dominance changed: **NO**.

Production default changed: **NO**. Recommend production adoption: **NO — experiment only; assess the controlled evidence before any policy decision.**
