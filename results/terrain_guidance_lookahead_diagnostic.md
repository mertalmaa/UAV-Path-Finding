# Lookahead preferred-altitude envelope diagnostic

Diagnostic only: no g/h/f, successor, safety, primitive, SearchKey or policy integration changed.

- Hard AGL: 100 m; soft preferred target: 120 m.
- Sampling: one terrain-cell spacing (60 m); horizons: 1 / 3 / 5 km, clamped at the reference goal.
- Climb model: reverse-Euler integration of the altitude-local `DerivedC172PEnvelope.straight_vertical(..., CLIMB)` safe rate at fixed 40 m/s. This is derived guidance, not an exact flight-optimality proof.
- Heading-ray classifications use p90 absolute difference: <=60 m GOOD, <=180 m ACCEPTABLE, otherwise MISLEADING.

## C_far_south_3km

| Horizon | Ray | Ray-ref mean/med/p90/max m | FULL/PARTIAL/INVALID | Raw climb viol. | Preferred climb | Descent | Max descent excess |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 km | GOOD | 3.2/0.0/0.4/76.2 | 50/0/0 | 16 | 6 | 15 | 21.55 |
| 3 km | GOOD | 0.2/0.3/0.3/0.4 | 44/6/0 | 16 | 2 | 11 | 21.55 |
| 5 km | GOOD | 0.3/0.3/0.3/0.3 | 10/40/0 | 16 | 2 | 11 | 21.55 |
## D_far_east_3km

| Horizon | Ray | Ray-ref mean/med/p90/max m | FULL/PARTIAL/INVALID | Raw climb viol. | Preferred climb | Descent | Max descent excess |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 km | GOOD | 0.1/0.0/0.0/3.2 | 50/0/0 | 19 | 0 | 14 | 32.13 |
| 3 km | GOOD | 0.0/0.0/0.0/0.0 | 44/6/0 | 19 | 0 | 14 | 32.13 |
| 5 km | GOOD | 0.0/0.0/0.0/0.0 | 10/40/0 | 19 | 0 | 14 | 32.13 |
## F_turn_required_diagonal_2_3km

| Horizon | Ray | Ray-ref mean/med/p90/max m | FULL/PARTIAL/INVALID | Raw climb viol. | Preferred climb | Descent | Max descent excess |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 km | ACCEPTABLE | 25.9/1.4/85.2/133.1 | 40/0/0 | 17 | 6 | 7 | 17.45 |
| 3 km | ACCEPTABLE | 27.7/12.1/85.2/133.1 | 40/0/0 | 17 | 6 | 7 | 17.45 |
| 5 km | ACCEPTABLE | 26.1/12.1/110.1/133.1 | 32/8/0 | 17 | 6 | 7 | 17.45 |

## Synthetic valley and dip checks

Valley max preferred-envelope elevation above raw +120: 184.9 m.
Small dip max preferred-envelope elevation above raw +120: 62.5 m.

Plot: results/terrain_guidance_lookahead_profiles.png.

Q1/Q5: raw +120 is compared against the back-propagated envelope in the synthetic valley/dip cases; the plotted envelope supplies the evidence without adding a smoothing filter. Q2: anticipation and Q4 descent feasibility are horizon-specific table metrics. Q3/Q11: provisional cheap-signal choice is **heading ray + 3 km**, diagnostic only. Q8/Q9: heading-ray agreement and validity are explicitly compared with the reference-path oracle; largest disagreements include future heading-change diagnostics in JSON. Q10: descent violations are reported separately because back-propagated climb protection does not guarantee descent feasibility. Q6/Q7: the current-path excess metrics show whether a materially lower profile exists, but no planner integration is authorized by this diagnostic.

Production behavior changed: **NO**. Full test suite: see execution record. RESULT: **PASS**.
