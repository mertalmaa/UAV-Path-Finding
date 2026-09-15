# Terrain-relative low-altitude soft-cost experiment

## Contract

Files changed: `planner/config.py`, `planner/pose_search.py`, `tests/test_pose_search.py`, this experimental runner, and its result artifacts.
- Hard minimum AGL: **100 m** (unchanged continuous safety authority).
- Soft desired AGL: **120 m**; scale: **100 m**.
- Formula: `C_edge = Σ ds * (m0+m1)/2`, `m=1+lambda*(max(0, AGL-120)/100)^2`.
- Heuristic changed: **NO**. It remains admissible because each multiplier is >=1, so edge cost is never below physical 3D length and the existing geometric lower bound remains a lower bound.

## A--F comparison

| Lambda | Mission | Status | Expanded | Generated | Peak OPEN | Path m | Mean AGL | 100-120% | 120-150% | 150-250% | 250-500% | >500% | C/D | Switches | Safety |
|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 0.00 | A_easy_open | FOUND | 1,201 | 6,000 | 1,109 | 1070.2 | 203.8 | 0.0 | 16.8 | 70.1 | 13.1 | 0.0 | 0/0 | 0 | PASS |
| 0.00 | B_relief_affected | FOUND | 466 | 2,325 | 388 | 1900.4 | 209.7 | 8.1 | 11.8 | 62.0 | 18.1 | 0.0 | 0/1 | 1 | PASS |
| 0.00 | C_far_south_3km | FOUND | 10,789 | 53,940 | 8,698 | 2922.6 | 527.3 | 0.0 | 0.0 | 0.0 | 41.2 | 58.8 | 1/2 | 3 | PASS |
| 0.00 | D_far_east_3km | FOUND | 8,928 | 44,635 | 6,900 | 2927.1 | 445.9 | 0.0 | 0.0 | 4.4 | 62.9 | 32.7 | 0/1 | 2 | PASS |
| 0.00 | E_long_descent_9_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 21,356 | - | - | - | - | - | - | - | - | - | NOT_FOUND |
| 0.00 | F_turn_required_diagonal_2_3km | FOUND | 5,942 | 29,705 | 3,001 | 2330.8 | 220.5 | 9.6 | 24.0 | 41.6 | 19.7 | 5.1 | 1/2 | 4 | PASS |
| 0.05 | A_easy_open | FOUND | 789 | 3,940 | 779 | 1070.7 | 191.9 | 0.0 | 22.4 | 69.1 | 8.5 | 0.0 | 1/2 | 2 | PASS |
| 0.05 | B_relief_affected | FOUND | 542 | 2,705 | 504 | 1900.1 | 207.5 | 8.1 | 11.8 | 62.0 | 18.1 | 0.0 | 1/1 | 3 | PASS |
| 0.05 | C_far_south_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 22,634 | - | - | - | - | - | - | - | - | - | NOT_FOUND |
| 0.05 | D_far_east_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 19,792 | - | - | - | - | - | - | - | - | - | NOT_FOUND |
| 0.05 | E_long_descent_9_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 20,877 | - | - | - | - | - | - | - | - | - | NOT_FOUND |
| 0.05 | F_turn_required_diagonal_2_3km | FOUND | 5,092 | 25,455 | 3,337 | 2319.9 | 232.4 | 5.0 | 10.0 | 57.1 | 22.9 | 5.0 | 0/1 | 2 | PASS |
| 0.10 | A_easy_open | FOUND | 818 | 4,085 | 831 | 1070.7 | 191.9 | 0.0 | 22.4 | 69.1 | 8.5 | 0.0 | 1/2 | 2 | PASS |
| 0.10 | B_relief_affected | FOUND | 590 | 2,945 | 589 | 1900.1 | 207.5 | 8.1 | 11.8 | 62.0 | 18.1 | 0.0 | 1/1 | 3 | PASS |
| 0.10 | C_far_south_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 22,758 | - | - | - | - | - | - | - | - | - | NOT_FOUND |
| 0.10 | D_far_east_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 20,817 | - | - | - | - | - | - | - | - | - | NOT_FOUND |
| 0.10 | E_long_descent_9_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 20,905 | - | - | - | - | - | - | - | - | - | NOT_FOUND |
| 0.10 | F_turn_required_diagonal_2_3km | FOUND | 5,582 | 27,905 | 3,712 | 2343.8 | 225.3 | 11.8 | 11.3 | 36.8 | 35.0 | 5.1 | 1/2 | 3 | PASS |
| 0.20 | A_easy_open | FOUND | 862 | 4,305 | 865 | 1070.7 | 191.9 | 0.0 | 22.4 | 69.1 | 8.5 | 0.0 | 1/2 | 2 | PASS |
| 0.20 | B_relief_affected | FOUND | 671 | 3,350 | 656 | 1900.1 | 207.5 | 8.1 | 11.8 | 62.0 | 18.1 | 0.0 | 1/1 | 3 | PASS |
| 0.20 | C_far_south_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 22,990 | - | - | - | - | - | - | - | - | - | NOT_FOUND |
| 0.20 | D_far_east_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 21,491 | - | - | - | - | - | - | - | - | - | NOT_FOUND |
| 0.20 | E_long_descent_9_3km | EXPANSION_LIMIT | 30,000 | 149,995 | 20,888 | - | - | - | - | - | - | - | - | - | NOT_FOUND |
| 0.20 | F_turn_required_diagonal_2_3km | FOUND | 6,166 | 30,825 | 4,181 | 2343.5 | 224.1 | 12.2 | 12.5 | 38.4 | 31.7 | 5.1 | 1/2 | 3 | PASS |

## Full AGL path statistics (FOUND paths)

| Lambda | Mission | Min | Mean | Median | P90 | Max |
|---:|---|---:|---:|---:|---:|---:|
| 0.00 | A_easy_open | 134.7 | 203.8 | 200.6 | 251.2 | 349.3 |
| 0.00 | B_relief_affected | 104.2 | 209.7 | 205.3 | 267.8 | 449.3 |
| 0.00 | C_far_south_3km | 290.4 | 527.3 | 515.1 | 703.4 | 733.3 |
| 0.00 | D_far_east_3km | 228.1 | 445.9 | 422.1 | 578.4 | 667.9 |
| 0.00 | F_turn_required_diagonal_2_3km | 109.5 | 220.5 | 189.9 | 346.6 | 549.3 |
| 0.05 | A_easy_open | 122.0 | 191.9 | 187.9 | 238.5 | 349.3 |
| 0.05 | B_relief_affected | 101.1 | 207.5 | 202.2 | 267.8 | 449.3 |
| 0.05 | F_turn_required_diagonal_2_3km | 103.5 | 232.4 | 191.8 | 359.2 | 549.3 |
| 0.10 | A_easy_open | 122.0 | 191.9 | 187.9 | 238.5 | 349.3 |
| 0.10 | B_relief_affected | 101.1 | 207.5 | 202.2 | 267.8 | 449.3 |
| 0.10 | F_turn_required_diagonal_2_3km | 100.4 | 225.3 | 211.3 | 311.7 | 549.3 |
| 0.20 | A_easy_open | 122.0 | 191.9 | 187.9 | 238.5 | 349.3 |
| 0.20 | B_relief_affected | 101.1 | 207.5 | 202.2 | 267.8 | 449.3 |
| 0.20 | F_turn_required_diagonal_2_3km | 100.4 | 224.1 | 208.0 | 311.7 | 549.3 |

## Mission E

| Lambda | Status | Expanded | Generated | Peak OPEN | Best XY / |Z| | Mean Z/(XY,H) | AGL mean/median/p90 |
|---:|---|---:|---:|---:|---:|---:|---:|
| 0.00 | EXPANSION_LIMIT | 30,000 | 149,995 | 21,356 | 120.0/4.4 | 36.06 | 1213.6/886.9/2150.7 |
| 0.05 | EXPANSION_LIMIT | 30,000 | 149,995 | 20,877 | 7258.1/302.9 | 9.31 | 2040.8/2035.5/2344.0 |
| 0.10 | EXPANSION_LIMIT | 30,000 | 149,995 | 20,905 | 7258.5/302.9 | 9.27 | 2040.8/2035.5/2344.0 |
| 0.20 | EXPANSION_LIMIT | 30,000 | 149,995 | 20,888 | 7260.0/283.9 | 9.28 | 2041.2/2036.0/2346.5 |

## Classification

- lambda=0.00: {"A_easy_open": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": -0.0, "path_length_change_percent": 0.0}, "B_relief_affected": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": -0.0, "path_length_change_percent": 0.0}, "C_far_south_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": -0.0, "path_length_change_percent": 0.0}, "D_far_east_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": -0.0, "path_length_change_percent": 0.0}, "E_long_descent_9_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "NOT_FOUND"}, "F_turn_required_diagonal_2_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": -0.0, "path_length_change_percent": 0.0}}
- lambda=0.05: {"A_easy_open": {"low_flight_effect": "TOO AGGRESSIVE", "search_effect": "IMPROVED", "path_quality": "VERTICAL ZIGZAG OBSERVED", "mean_agl_change_m": -11.884110838924443, "path_length_change_percent": 0.04980036118584774}, "B_relief_affected": {"low_flight_effect": "WEAK", "search_effect": "REGRESSED", "path_quality": "VERTICAL ZIGZAG OBSERVED", "mean_agl_change_m": -2.207028220783627, "path_length_change_percent": -0.016742638160083256}, "C_far_south_3km": {"low_flight_effect": "NONE", "search_effect": "REGRESSED", "path_quality": "NOT_FOUND"}, "D_far_east_3km": {"low_flight_effect": "NONE", "search_effect": "REGRESSED", "path_quality": "NOT_FOUND"}, "E_long_descent_9_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "NOT_FOUND"}, "F_turn_required_diagonal_2_3km": {"low_flight_effect": "NONE", "search_effect": "IMPROVED", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": 11.90443177700135, "path_length_change_percent": -0.46959887176952764}}
- lambda=0.10: {"A_easy_open": {"low_flight_effect": "TOO AGGRESSIVE", "search_effect": "IMPROVED", "path_quality": "VERTICAL ZIGZAG OBSERVED", "mean_agl_change_m": -11.884110838924443, "path_length_change_percent": 0.04980036118584774}, "B_relief_affected": {"low_flight_effect": "WEAK", "search_effect": "REGRESSED", "path_quality": "VERTICAL ZIGZAG OBSERVED", "mean_agl_change_m": -2.207028220783627, "path_length_change_percent": -0.016742638160083256}, "C_far_south_3km": {"low_flight_effect": "NONE", "search_effect": "REGRESSED", "path_quality": "NOT_FOUND"}, "D_far_east_3km": {"low_flight_effect": "NONE", "search_effect": "REGRESSED", "path_quality": "NOT_FOUND"}, "E_long_descent_9_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "NOT_FOUND"}, "F_turn_required_diagonal_2_3km": {"low_flight_effect": "NONE", "search_effect": "IMPROVED", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": 4.825364004760132, "path_length_change_percent": 0.5549872215033114}}
- lambda=0.20: {"A_easy_open": {"low_flight_effect": "TOO AGGRESSIVE", "search_effect": "IMPROVED", "path_quality": "VERTICAL ZIGZAG OBSERVED", "mean_agl_change_m": -11.884110838924443, "path_length_change_percent": 0.04980036118584774}, "B_relief_affected": {"low_flight_effect": "WEAK", "search_effect": "REGRESSED", "path_quality": "VERTICAL ZIGZAG OBSERVED", "mean_agl_change_m": -2.207028220783627, "path_length_change_percent": -0.016742638160083256}, "C_far_south_3km": {"low_flight_effect": "NONE", "search_effect": "REGRESSED", "path_quality": "NOT_FOUND"}, "D_far_east_3km": {"low_flight_effect": "NONE", "search_effect": "REGRESSED", "path_quality": "NOT_FOUND"}, "E_long_descent_9_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "NOT_FOUND"}, "F_turn_required_diagonal_2_3km": {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH", "mean_agl_change_m": 3.6233273376668933, "path_length_change_percent": 0.544771672920219}}

Terrain-following plots: results/low_altitude_cost_profiles_lambda_0.00.png, results/low_altitude_cost_profiles_lambda_0.10.png.

ZIGZAG OBSERVED and SEARCH REGRESSION are determined per-mission in the classification table; no smoothness, switch, climb, descent or look-ahead term was added. Small-terrain-dip overfollowing: **NO direct evidence in this sweep**; the soft cost instead materially regresses C/D/E search reachability at every positive lambda. BEST EXPERIMENTAL LAMBDA: **none**; do not select a production value. Mission E remains a separate vertical-schedule problem; this cost experiment does not claim to resolve it.

Production default changed: **NO**. SearchKey changed: **NO**. Z bin changed: **NO**. Dominance changed: **NO**. Combined turns: **OFF**. RESULT: **PASS**.
