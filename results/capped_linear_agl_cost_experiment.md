# Capped-linear terrain-relative AGL cost experiment

Files changed: `planner/config.py`, `planner/trajectory_safety.py`, `planner/pose_search.py`, `tests/test_pose_search.py`, this runner and result artifacts.

- AGL cost model: **CAPPED LINEAR**; hard minimum 100 m; soft target 120 m; full penalty at 500 m.
- Formula: `C_edge=Σ ds*(m0+m1)/2`; `m=1` for AGL<=120; `m=1+((AGL-120)/(500-120))*(M-1)` for 120<AGL<500; `m=M` at/above 500 m.
- Additional terrain queries caused by cost: **NO**. Safety records sample terrain from its already-built terrain field when opt-in cost is enabled; cost reuses that payload and calls no `terrain.query()`.
- Heuristic changed: **NO**. Heuristic still admissible: **YES**, because `m>=1`, hence every edge cost remains at least physical 3D length.

## A--F comparison

| M | Mission | Status | Expanded | Generated | Rejected | Peak | ms/exp | Path m | AGL min/mean/med/p90/max | C/D | Switch | Safety |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.00 | A_easy_open | FOUND | 1,201 | 6,000 | 3,225 | 1,109 | 0.522 | 1070.2 | 134.7/203.8/200.6/251.2/349.3 | 0/0 | 0 | PASS |
| 1.00 | B_relief_affected | FOUND | 466 | 2,325 | 1,377 | 388 | 0.521 | 1900.4 | 104.2/209.7/205.3/267.8/449.3 | 0/1 | 1 | PASS |
| 1.00 | C_far_south_3km | FOUND | 10,789 | 53,940 | 29,308 | 8,698 | 0.963 | 2922.6 | 290.4/527.3/515.1/703.4/733.3 | 1/2 | 3 | PASS |
| 1.00 | D_far_east_3km | FOUND | 8,928 | 44,635 | 25,247 | 6,900 | 1.102 | 2927.1 | 228.1/445.9/422.1/578.4/667.9 | 0/1 | 2 | PASS |
| 1.00 | E_long_descent_9_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 95,073 | 21,356 | 1.039 | - | - | - | - | NOT_FOUND |
| 1.00 | F_turn_required_diagonal_2_3km | FOUND | 5,942 | 29,705 | 18,486 | 3,001 | 1.090 | 2330.8 | 109.5/220.5/189.9/346.6/549.3 | 1/2 | 4 | PASS |
| 1.05 | A_easy_open | FOUND | 789 | 3,940 | 2,150 | 754 | 1.159 | 1070.7 | 122.0/191.9/187.9/238.5/349.3 | 1/2 | 2 | PASS |
| 1.05 | B_relief_affected | FOUND | 500 | 2,495 | 1,420 | 433 | 1.299 | 1900.1 | 101.1/207.5/202.2/267.8/449.3 | 1/1 | 3 | PASS |
| 1.05 | C_far_south_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 78,546 | 21,049 | 1.226 | - | - | - | - | NOT_FOUND |
| 1.05 | D_far_east_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 82,789 | 18,404 | 1.226 | - | - | - | - | NOT_FOUND |
| 1.05 | E_long_descent_9_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 90,211 | 19,978 | 1.126 | - | - | - | - | NOT_FOUND |
| 1.05 | F_turn_required_diagonal_2_3km | FOUND | 4,668 | 23,335 | 14,044 | 2,651 | 1.170 | 2319.9 | 103.5/232.4/191.8/359.2/549.3 | 0/1 | 2 | PASS |
| 1.10 | A_easy_open | FOUND | 795 | 3,970 | 2,157 | 777 | 1.177 | 1070.7 | 122.0/191.9/187.9/238.5/349.3 | 1/2 | 2 | PASS |
| 1.10 | B_relief_affected | FOUND | 517 | 2,580 | 1,469 | 468 | 1.174 | 1900.1 | 101.1/207.5/202.2/267.8/449.3 | 1/1 | 3 | PASS |
| 1.10 | C_far_south_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 77,930 | 21,210 | 1.220 | - | - | - | - | NOT_FOUND |
| 1.10 | D_far_east_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 84,397 | 18,582 | 0.983 | - | - | - | - | NOT_FOUND |
| 1.10 | E_long_descent_9_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 89,656 | 19,578 | 1.008 | - | - | - | - | NOT_FOUND |
| 1.10 | F_turn_required_diagonal_2_3km | FOUND | 4,493 | 22,460 | 13,622 | 2,771 | 1.102 | 2320.2 | 109.8/235.0/198.1/359.2/549.3 | 0/0 | 0 | PASS |

## AGL bands (metres / physical-path percent)

| M | Mission | 100-120 | 120-150 | 150-250 | 250-500 | >500 |
|---:|---:|---:|---:|---:|---:|---:|
| 1.00 | A_easy_open | 0.0 / 0.0% | 180.0 / 16.8% | 750.2 / 70.1% | 140.0 / 13.1% | 0.0 / 0.0% |
| 1.00 | B_relief_affected | 153.3 / 8.1% | 224.0 / 11.8% | 1179.1 / 62.0% | 344.0 / 18.1% | 0.0 / 0.0% |
| 1.00 | C_far_south_3km | 0.0 / 0.0% | 0.0 / 0.0% | 0.0 / 0.0% | 1204.2 / 41.2% | 1718.5 / 58.8% |
| 1.00 | D_far_east_3km | 0.0 / 0.0% | 0.0 / 0.0% | 130.0 / 4.4% | 1840.0 / 62.9% | 957.1 / 32.7% |
| 1.00 | F_turn_required_diagonal_2_3km | 222.9 / 9.6% | 559.8 / 24.0% | 970.0 / 41.6% | 459.8 / 19.7% | 118.2 / 5.1% |
| 1.05 | A_easy_open | 0.0 / 0.0% | 240.0 / 22.4% | 740.2 / 69.1% | 90.5 / 8.5% | 0.0 / 0.0% |
| 1.05 | B_relief_affected | 153.3 / 8.1% | 223.9 / 11.8% | 1178.9 / 62.0% | 344.0 / 18.1% | 0.0 / 0.0% |
| 1.05 | F_turn_required_diagonal_2_3km | 115.8 / 5.0% | 231.7 / 10.0% | 1325.3 / 57.1% | 531.2 / 22.9% | 115.9 / 5.0% |
| 1.10 | A_easy_open | 0.0 / 0.0% | 240.0 / 22.4% | 740.2 / 69.1% | 90.5 / 8.5% | 0.0 / 0.0% |
| 1.10 | B_relief_affected | 153.3 / 8.1% | 223.9 / 11.8% | 1178.9 / 62.0% | 344.0 / 18.1% | 0.0 / 0.0% |
| 1.10 | F_turn_required_diagonal_2_3km | 48.3 / 2.1% | 299.4 / 12.9% | 1296.3 / 55.9% | 560.2 / 24.1% | 115.9 / 5.0% |

Exact final primitive sequences/counts for every FOUND path are retained in JSON.

## Mission E

| M | Status | Expanded | Generated | Peak | Best XY/Z/3D | Z/(XY,H) mean/median | First goal OPEN | Goal pop | Partial AGL mean/med/p90 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.00 | EXPANSION_LIMIT | 30,000 | 149,995 | 21,356 | 120.0/4.4/120.1 | 36.06/36.00 | 28618 | - | 1213.6/886.9/2150.7 |
| 1.05 | EXPANSION_LIMIT | 30,000 | 149,995 | 19,978 | 5937.2/163.1/5942.2 | 14.93/12.00 | - | - | 1824.8/1813.2/2306.2 |
| 1.10 | EXPANSION_LIMIT | 30,000 | 149,995 | 19,578 | 6420.0/353.8/6429.7 | 12.72/11.00 | - | - | 2016.3/1934.6/2341.5 |

## Classification

- M=1.00: {"A_easy_open": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": -0.0, "path_length_change_percent": 0.0, "vertical_switch_change": 0}, "B_relief_affected": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": -0.0, "path_length_change_percent": 0.0, "vertical_switch_change": 0}, "C_far_south_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": -0.0, "path_length_change_percent": 0.0, "vertical_switch_change": 0}, "D_far_east_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": -0.0, "path_length_change_percent": 0.0, "vertical_switch_change": 0}, "E_long_descent_9_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "NOT FOUND"}, "F_turn_required_diagonal_2_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": -0.0, "path_length_change_percent": 0.0, "vertical_switch_change": 0}}
- M=1.05: {"A_easy_open": {"low_flight_effect": "TOO AGGRESSIVE", "search_effect": "IMPROVED", "path_quality": "VERTICAL ZIGZAG", "mean_agl_change_m": -11.884110838924443, "path_length_change_percent": 0.04980036118584774, "vertical_switch_change": 2}, "B_relief_affected": {"low_flight_effect": "WEAK", "search_effect": "MODERATE REGRESSION", "path_quality": "VERTICAL ZIGZAG", "mean_agl_change_m": -2.207028220783627, "path_length_change_percent": -0.016742638160083256, "vertical_switch_change": 2}, "C_far_south_3km": {"low_flight_effect": "NONE", "search_effect": "SEVERE REGRESSION", "path_quality": "NOT FOUND"}, "D_far_east_3km": {"low_flight_effect": "NONE", "search_effect": "SEVERE REGRESSION", "path_quality": "NOT FOUND"}, "E_long_descent_9_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "NOT FOUND"}, "F_turn_required_diagonal_2_3km": {"low_flight_effect": "NONE", "search_effect": "IMPROVED", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": 11.90443177700135, "path_length_change_percent": -0.46959887176952764, "vertical_switch_change": -2}}
- M=1.10: {"A_easy_open": {"low_flight_effect": "TOO AGGRESSIVE", "search_effect": "IMPROVED", "path_quality": "VERTICAL ZIGZAG", "mean_agl_change_m": -11.884110838924443, "path_length_change_percent": 0.04980036118584774, "vertical_switch_change": 2}, "B_relief_affected": {"low_flight_effect": "WEAK", "search_effect": "MODERATE REGRESSION", "path_quality": "VERTICAL ZIGZAG", "mean_agl_change_m": -2.207028220783627, "path_length_change_percent": -0.016742638160083256, "vertical_switch_change": 2}, "C_far_south_3km": {"low_flight_effect": "NONE", "search_effect": "SEVERE REGRESSION", "path_quality": "NOT FOUND"}, "D_far_east_3km": {"low_flight_effect": "NONE", "search_effect": "SEVERE REGRESSION", "path_quality": "NOT FOUND"}, "E_long_descent_9_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "NOT FOUND"}, "F_turn_required_diagonal_2_3km": {"low_flight_effect": "NONE", "search_effect": "IMPROVED", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": 14.484707700760538, "path_length_change_percent": -0.4587797726087084, "vertical_switch_change": -4}}

Terrain profile results: results/low_altitude_cost_profiles_lambda_1.00.png, results/low_altitude_cost_profiles_lambda_1.05.png.

Quadratic failure repeated: **YES**. C/D remain FOUND for positive multipliers: **NO**. SEARCH REGRESSION is assessed separately from cost CPU overhead via expanded/generated/peak/goal progress; runtime-per-expansion is reported but not used alone. LOW-FLIGHT IMPROVEMENT and zigzag are per-mission classifications. BEST EXPERIMENTAL MULTIPLIER: **NONE**. Recommend production adoption: **NO**; this is controlled evidence only, not a deployment decision.

Production default changed: **NO**. SearchKey changed: **NO**. Z bin changed: **NO**. Dominance changed: **NO**. Combined turns: **OFF**. RESULT: **PASS**.
