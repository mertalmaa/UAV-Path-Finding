# Pose-aware BASIC vs COMBINED benchmark (A-F)

Only `enable_combined_turns` changes between these runs. Search key, heuristic, cost, safety, goal tolerance, primitive distance, turn angle, and budgets are identical.

| Mission | Mode | Status | Expanded | Generated | Gen/expanded | Rejected | Peak OPEN | Runtime s | Path m | Min AGL m | Best XY / Z error m | Path primitive counts |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| A_easy_open | BASIC | FOUND | 154 | 765 | 4.97 | 523 | 90 | 0.06 | 1080.00 | 134.65 | 60.00 / 0.00 | STRAIGHT_LEVEL:18 |
| A_easy_open | COMBINED | FOUND | 154 | 1377 | 8.94 | 1135 | 90 | 0.10 | 1080.00 | 134.65 | 60.00 / 0.00 | STRAIGHT_LEVEL:18 |
| B_relief_affected | BASIC | FOUND | 378 | 1885 | 4.99 | 1313 | 164 | 0.17 | 1788.04 | 100.30 | 87.23 / 7.50 | LEFT_LEVEL_TURN:7, RIGHT_LEVEL_TURN:4, STRAIGHT_CLIMB:1, STRAIGHT_LEVEL:12 |
| B_relief_affected | COMBINED | FOUND | 613 | 5508 | 8.99 | 4458 | 398 | 0.42 | 1788.04 | 100.30 | 87.23 / 7.50 | LEFT_LEVEL_TURN:7, RIGHT_LEVEL_TURN:4, STRAIGHT_CLIMB:1, STRAIGHT_LEVEL:12 |
| C_far_south_3km | BASIC | FOUND | 390 | 1945 | 4.99 | 1318 | 240 | 0.18 | 2940.00 | 303.05 | 60.00 / 0.00 | STRAIGHT_LEVEL:49 |
| C_far_south_3km | COMBINED | FOUND | 390 | 3501 | 8.98 | 2874 | 240 | 0.26 | 2940.00 | 303.05 | 60.00 / 0.00 | STRAIGHT_LEVEL:49 |
| D_far_east_3km | BASIC | FOUND | 334 | 1665 | 4.99 | 1061 | 270 | 0.16 | 2940.00 | 228.13 | 60.00 / 0.00 | STRAIGHT_LEVEL:49 |
| D_far_east_3km | COMBINED | FOUND | 334 | 2997 | 8.97 | 2393 | 270 | 0.23 | 2940.00 | 228.13 | 60.00 / 0.00 | STRAIGHT_LEVEL:49 |
| E_long_descent_9_3km | BASIC | FOUND | 7137 | 35680 | 5.00 | 22992 | 5554 | 2.73 | 9270.82 | 504.07 | 60.00 / 5.00 | STRAIGHT_DESCENT:66, STRAIGHT_LEVEL:88 |
| E_long_descent_9_3km | COMBINED | FOUND | 17333 | 155988 | 9.00 | 122340 | 13958 | 13.79 | 9270.82 | 504.07 | 60.00 / 5.00 | STRAIGHT_DESCENT:66, STRAIGHT_LEVEL:88 |
| F_turn_required_diagonal_2_3km | BASIC | FOUND | 404 | 2015 | 4.99 | 1431 | 158 | 0.17 | 2292.78 | 109.83 | 80.70 / 0.00 | LEFT_LEVEL_TURN:4, RIGHT_LEVEL_TURN:4, STRAIGHT_LEVEL:26 |
| F_turn_required_diagonal_2_3km | COMBINED | FOUND | 370 | 3321 | 8.98 | 2631 | 301 | 0.24 | 2280.47 | 102.26 | 89.14 / 7.50 | CLIMBING_LEFT_TURN:2, DESCENDING_LEFT_TURN:1, DESCENDING_RIGHT_TURN:1, LEFT_LEVEL_TURN:6, RIGHT_LEVEL_TURN:7, STRAIGHT_DESCENT:1, STRAIGHT_LEVEL:11 |

## Mission B combined-path verification

Ordered physical primitives:

```text
RIGHT_LEVEL_TURN -> RIGHT_LEVEL_TURN -> STRAIGHT_LEVEL -> STRAIGHT_LEVEL -> STRAIGHT_LEVEL -> RIGHT_LEVEL_TURN -> STRAIGHT_LEVEL -> STRAIGHT_LEVEL -> LEFT_LEVEL_TURN -> LEFT_LEVEL_TURN -> STRAIGHT_LEVEL -> STRAIGHT_LEVEL -> STRAIGHT_LEVEL -> LEFT_LEVEL_TURN -> LEFT_LEVEL_TURN -> STRAIGHT_CLIMB -> LEFT_LEVEL_TURN -> LEFT_LEVEL_TURN -> RIGHT_LEVEL_TURN -> LEFT_LEVEL_TURN -> STRAIGHT_LEVEL -> STRAIGHT_LEVEL -> STRAIGHT_LEVEL -> STRAIGHT_LEVEL
```

Combined primitive present: **NO**.

## Mission E regression check

- BASIC: generated 35680; peak OPEN 5554; best XY 60.00 m.
- COMBINED: generated 155988; peak OPEN 13958; best XY 60.00 m.
- Classification: **NEUTRAL**.

Search-level counters are intentionally separate from final reconstructed path primitive counts.
