# Pose-aware BASIC vs COMBINED benchmark (A-F)

Only `enable_combined_turns` changes between these runs. Search key, heuristic, cost, safety, goal tolerance, primitive distance, turn angle, and budgets are identical.

| Mission | Mode | Status | Expanded | Generated | Gen/expanded | Rejected | Peak OPEN | Runtime s | Path m | Min AGL m | Best XY / Z error m | Path primitive counts |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| A_easy_open | BASIC | FOUND | 1201 | 6000 | 5.00 | 3225 | 1109 | 0.58 | 1070.16 | 134.65 | 82.88 / 0.00 | LEFT_LEVEL_TURN:1, STRAIGHT_LEVEL:15 |
| A_easy_open | COMBINED | FOUND | 1405 | 12636 | 8.99 | 7815 | 2711 | 1.73 | 1065.38 | 117.10 | 84.11 / 8.64 | CLIMBING_RIGHT_TURN:1, DESCENDING_RIGHT_TURN:1, LEFT_LEVEL_TURN:2, STRAIGHT_CLIMB:1, STRAIGHT_DESCENT:1, STRAIGHT_LEVEL:3 |
| B_relief_affected | BASIC | FOUND | 466 | 2325 | 4.99 | 1377 | 388 | 0.28 | 1900.42 | 104.23 | 89.26 / 6.34 | LEFT_LEVEL_TURN:7, RIGHT_LEVEL_TURN:3, STRAIGHT_DESCENT:1, STRAIGHT_LEVEL:2 |
| B_relief_affected | COMBINED | FOUND | 1141 | 10260 | 8.99 | 6702 | 2036 | 1.41 | 1883.29 | 102.34 | 82.53 / 0.58 | CLIMBING_LEFT_TURN:1, LEFT_LEVEL_TURN:6, RIGHT_LEVEL_TURN:3, STRAIGHT_DESCENT:1, STRAIGHT_LEVEL:1 |
| C_far_south_3km | BASIC | FOUND | 10789 | 53940 | 5.00 | 29308 | 8698 | 5.23 | 2922.64 | 290.36 | 89.50 / 9.47 | LEFT_LEVEL_TURN:2, RIGHT_LEVEL_TURN:2, STRAIGHT_CLIMB:1, STRAIGHT_DESCENT:2, STRAIGHT_LEVEL:34 |
| C_far_south_3km | COMBINED | FOUND | 6673 | 60048 | 9.00 | 33661 | 14451 | 8.39 | 2919.71 | 303.05 | 87.66 / 5.89 | CLIMBING_LEFT_TURN:1, STRAIGHT_LEVEL:45 |
| D_far_east_3km | BASIC | FOUND | 8928 | 44635 | 5.00 | 25247 | 6900 | 4.35 | 2927.10 | 228.13 | 82.45 / 6.34 | LEFT_LEVEL_TURN:1, RIGHT_LEVEL_TURN:2, STRAIGHT_DESCENT:1, STRAIGHT_LEVEL:39 |
| D_far_east_3km | COMBINED | FOUND | 18743 | 168678 | 9.00 | 112983 | 24598 | 24.59 | 2927.10 | 228.13 | 82.45 / 6.34 | LEFT_LEVEL_TURN:1, RIGHT_LEVEL_TURN:2, STRAIGHT_DESCENT:1, STRAIGHT_LEVEL:39 |
| E_long_descent_9_3km | BASIC | EXPANSION_LIMIT | 30000 | 149995 | 5.00 | 95073 | 21356 | 13.88 | - | - | 120.00 / 4.36 | - |
| E_long_descent_9_3km | COMBINED | EXPANSION_LIMIT | 30000 | 269991 | 9.00 | 183807 | 47818 | 29.93 | - | - | 1380.00 / 74.14 | - |
| F_turn_required_diagonal_2_3km | BASIC | FOUND | 5942 | 29705 | 5.00 | 18486 | 3001 | 2.83 | 2330.84 | 109.46 | 79.14 / 9.47 | LEFT_LEVEL_TURN:5, RIGHT_LEVEL_TURN:6, STRAIGHT_CLIMB:1, STRAIGHT_DESCENT:2, STRAIGHT_LEVEL:4 |
| F_turn_required_diagonal_2_3km | COMBINED | FOUND | 7357 | 66204 | 9.00 | 46529 | 8368 | 9.15 | 2275.73 | 102.70 | 84.85 / 3.96 | CLIMBING_LEFT_TURN:4, CLIMBING_RIGHT_TURN:3, DESCENDING_RIGHT_TURN:1, LEFT_LEVEL_TURN:1, STRAIGHT_DESCENT:4, STRAIGHT_LEVEL:2 |

## Mission B combined-path verification

Ordered physical primitives:

```text
RIGHT_LEVEL_TURN -> RIGHT_LEVEL_TURN -> RIGHT_LEVEL_TURN -> STRAIGHT_DESCENT -> CLIMBING_LEFT_TURN -> LEFT_LEVEL_TURN -> LEFT_LEVEL_TURN -> LEFT_LEVEL_TURN -> LEFT_LEVEL_TURN -> LEFT_LEVEL_TURN -> STRAIGHT_LEVEL -> LEFT_LEVEL_TURN
```

Combined primitive present: **YES**.

## Mission E regression check

- BASIC: generated 149995; peak OPEN 21356; best XY 120.00 m.
- COMBINED: generated 269991; peak OPEN 47818; best XY 1380.00 m.
- Classification: **REGRESS**.

Search-level counters are intentionally separate from final reconstructed path primitive counts.
