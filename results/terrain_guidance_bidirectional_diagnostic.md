# Bidirectional feasible low-altitude envelope diagnostic

Diagnostic only; production A* behavior is unchanged.

- Horizon: 3 km; hard AGL 100 m; soft target 120 m; sample spacing 60 m.
- Climb and descent model: altitude-local `DerivedC172PEnvelope` safe rates at fixed 40 m/s.
- Projection: backward climb lower-bound pass, then repeated forward descent lower-bound pass until no sampled violations or 16 iterations; anchored physical start/end are never raised silently.

| Mission | Raw climb | Climb-only | Final climb | Final descent | Iter/converged | Final AGL min/mean/med/p90/max | Ray-ref mean/med/p90/max |
|---:|---:|---:|---:|---:|---:|---:|---:|
| C_far_south_3km | 16 | 0 | 0 | 0 | 2 / True | 256.2/478.3/478.4/626.8/657.2 | 0.2/0.3/0.3/0.4 |
| D_far_east_3km | 19 | 0 | 0 | 0 | 2 / True | 133.0/395.8/355.3/583.9/661.5 | 0.0/0.0/0.0/0.0 |
| F_turn_required_diagonal_2_3km | 17 | 0 | 0 | 0 | 2 / True | 120.0/212.1/184.5/309.1/549.3 | 27.7/12.1/85.2/133.1 |

## C_far_south_3km profile excess

{"current_path_excess": {"above_50m_percent": 52.0, "above_100m_percent": 10.0, "above_250m_percent": 0.0, "above_500m_percent": 0.0}, "join_raw_after_m": null, "leave_raw_before_goal_m": null, "final_vertical": {"agl": {"min": 256.20960452773306, "mean": 478.26370360366366, "median": 478.358321696488, "p90": 626.7549317265717, "max": 657.2450168179203}, "total_climb_m": 98.81118560611367, "total_descent_m": 108.27854729361843, "vertical_direction_switches": 1}}

## D_far_east_3km profile excess

{"current_path_excess": {"above_50m_percent": 56.0, "above_100m_percent": 8.0, "above_250m_percent": 0.0, "above_500m_percent": 0.0}, "join_raw_after_m": null, "leave_raw_before_goal_m": null, "final_vertical": {"agl": {"min": 132.9876733420915, "mean": 395.7723948609852, "median": 355.3164017411448, "p90": 583.871463043334, "max": 661.5429119467644}, "total_climb_m": 101.48919010176905, "total_descent_m": 107.83226448312962, "vertical_direction_switches": 1}}

## F_turn_required_diagonal_2_3km profile excess

{"current_path_excess": {"above_50m_percent": 0.0, "above_100m_percent": 0.0, "above_250m_percent": 0.0, "above_500m_percent": 0.0}, "join_raw_after_m": 905.7956832614273, "leave_raw_before_goal_m": 1206.9409977606879, "final_vertical": {"agl": {"min": 120.0, "mean": 212.1025292668387, "median": 184.4909207532137, "p90": 309.07067287414964, "max": 549.259521484375}, "total_climb_m": 75.24984547766644, "total_descent_m": 84.7172071651712, "vertical_direction_switches": 5}}

## Synthetic cases

- valley: raw drop 240.0 m; feasible drop 95.1 m; max above raw 196.8 m; final climb/descent violations 0/0.
- small_dip: raw drop 80.0 m; feasible drop 17.5 m; max above raw 62.5 m; final climb/descent violations 0/0.

Plot: results/terrain_guidance_bidirectional_profiles.png.

All final climb violations zero: **YES**. All final descent violations zero: **YES**. Profile significantly lower than current path: **NO**. 3 km heading-ray remains a reasonable approximation: **YES**. Promising for a first search-guidance integration experiment: **YES**, but only as a separately controlled experiment; no cost/heuristic/gating integration occurred here.

Production behavior changed: **NO**. Full test suite: see execution record. RESULT: **PASS**.
