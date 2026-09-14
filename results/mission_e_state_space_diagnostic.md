# Mission E State-Space Diagnostic

## Baseline reproduction

- Reproduced without observer: **YES**
- Status: `search_limit_reached` / `EXPANSION_LIMIT`
- Expanded / generated / rejected: 30,000 / 149,995 / 95,073
- Peak OPEN: 21,356
- Best XY / Z / 3D error: 120.00 m / 4.36 m / 120.08 m
- Expanded altitude range: 3904.36-4400.00 m MSL
- Runtime without / with observer: 16.06 s / 16.14 s (observer overhead is expected and does not alter policy).

## State diversity

| Metric | Mean | Median | P90 | P95 | Max |
|---|---:|---:|---:|---:|---:|
| Z bins / XY | 37.42 | 37.00 | 62.00 | 72.00 | 82 |
| Heading bins / XY | 2.29 | 2.00 | 3.00 | 3.00 | 3 |
| Heading bins / (XY,Z) | 2.20 | 2.00 | 3.00 | 3.00 | 3 |
| Z bins / (XY,heading) | 36.06 | 36.00 | 60.90 | 68.00 | 82 |
| Full states / XY | 82.42 | 76.00 | 168.00 | 195.85 | 220 |

## Top 20 Z-diverse XY bins

| XY bin | Z | Head | (Z,H) | Expanded | Altitude range m | Goal distance min/median m |
|---:|---:|---:|---:|---:|---:|---:|
| (11484, 69777) | 82 | 3 | 218 | 218 | 3910.7-4400.0 | 4680.0/4686.2 |
| (11485, 69777) | 82 | 3 | 216 | 216 | 3904.4-4393.6 | 4620.0/4626.2 |
| (11483, 69777) | 81 | 3 | 219 | 219 | 3917.0-4400.0 | 4740.0/4746.2 |
| (11486, 69777) | 81 | 3 | 213 | 213 | 3904.4-4387.3 | 4560.0/4566.1 |
| (11482, 69777) | 80 | 3 | 220 | 220 | 3923.4-4400.0 | 4800.1/4806.3 |
| (11487, 69777) | 80 | 3 | 210 | 210 | 3904.4-4380.9 | 4500.0/4506.1 |
| (11488, 69777) | 79 | 3 | 207 | 207 | 3904.4-4374.6 | 4440.0/4446.0 |
| (11481, 69777) | 78 | 3 | 218 | 218 | 3929.7-4400.0 | 4860.1/4866.5 |
| (11489, 69777) | 78 | 3 | 204 | 204 | 3904.4-4368.2 | 4380.0/4386.0 |
| (11480, 69777) | 77 | 3 | 217 | 217 | 3936.1-4400.0 | 4920.1/4926.6 |
| (11490, 69777) | 77 | 3 | 201 | 201 | 3904.4-4361.9 | 4320.0/4326.0 |
| (11479, 69777) | 76 | 3 | 216 | 216 | 3942.4-4400.0 | 4980.2/4986.7 |
| (11491, 69777) | 76 | 3 | 198 | 198 | 3904.4-4355.5 | 4260.0/4265.9 |
| (11492, 69777) | 75 | 3 | 195 | 195 | 3904.4-4349.1 | 4200.0/4205.9 |
| (11478, 69777) | 74 | 3 | 214 | 214 | 3948.8-4400.0 | 5040.2/5046.9 |
| (11493, 69777) | 74 | 3 | 192 | 192 | 3904.4-4342.8 | 4140.0/4145.8 |
| (11477, 69777) | 73 | 3 | 211 | 211 | 3955.1-4400.0 | 5100.3/5107.0 |
| (11494, 69777) | 73 | 3 | 189 | 189 | 3904.4-4336.4 | 4080.0/4085.9 |
| (11476, 69777) | 72 | 3 | 208 | 208 | 3961.5-4400.0 | 5160.4/5167.1 |
| (11495, 69777) | 72 | 3 | 186 | 186 | 3904.4-4330.1 | 4020.0/4025.8 |

## Expanded altitude distribution (25 m bands)

| MSL band | Count | Percent |
|---:|---:|---:|
| 3900-3925 | 308 | 1.03% |
| 3925-3950 | 476 | 1.59% |
| 3950-3975 | 916 | 3.05% |
| 3975-4000 | 1072 | 3.57% |
| 4000-4025 | 1390 | 4.63% |
| 4025-4050 | 1196 | 3.99% |
| 4050-4075 | 1660 | 5.53% |
| 4075-4100 | 1702 | 5.67% |
| 4100-4125 | 1732 | 5.77% |
| 4125-4150 | 1746 | 5.82% |
| 4150-4175 | 1760 | 5.87% |
| 4175-4200 | 1764 | 5.88% |
| 4200-4225 | 1776 | 5.92% |
| 4225-4250 | 1776 | 5.92% |
| 4250-4275 | 1780 | 5.93% |
| 4275-4300 | 1784 | 5.95% |
| 4300-4325 | 1788 | 5.96% |
| 4325-4350 | 1792 | 5.97% |
| 4350-4375 | 1792 | 5.97% |
| 4375-4400 | 1344 | 4.48% |
| 4400-4425 | 446 | 1.49% |

## Altitude and goal-distance evolution

| Expansion window | Alt min/median/max | Best |Z| | Median |Z| | Median XY | Median 3D |
|---:|---:|---:|---:|---:|---:|
| 1-1000 | 4190.2/4342.8/4400.0 | 290.2 | 442.8 | 7920.0 | 7931.5 |
| 1001-5000 | 4075.8/4298.3/4400.0 | 175.8 | 398.3 | 7020.0 | 7031.6 |
| 5001-10000 | 4024.9/4253.8/4400.0 | 124.9 | 353.8 | 6416.0 | 6425.1 |
| 10001-20000 | 3955.1/4171.1/4400.0 | 55.1 | 271.1 | 5221.5 | 5230.7 |
| 20001-30000 | 3904.4/4075.8/4400.0 | 4.4 | 175.8 | 3960.0 | 3966.5 |

## Goal-proximity density

| XY radius | Expanded | XY bins | Mean Z / XY | Mean head / XY | Mean (Z,H) / XY | Best |Z| |
|---:|---:|---:|---:|---:|---:|---:|
| <=100 m | 0 | - | - | - | - | - |
| <=150 m | 2 | 1 | 2.00 | 1.00 | 2.00 | 4.36 |
| <=250 m | 11 | 3 | 3.67 | 1.00 | 3.67 | 4.36 |
| <=500 m | 46 | 7 | 6.57 | 1.00 | 6.57 | 4.36 |
| <=1000 m | 176 | 15 | 11.73 | 1.00 | 11.73 | 4.36 |

## Goal-useful expansions

A state is goal-useful only if one of its accepted OPEN insertions establishes a strict global best in XY, absolute Z error or 3D distance.

| Category | Expanded states | Percent of 30k |
|---:|---:|---:|
| XY | 667 | 2.22% |
| Z | 79 | 0.26% |
| D3 | 700 | 2.33% |
| none | 28720 | 95.73% |

## Same-key and primitive contribution

| Primitive | Generated | Unavailable | Safety reject | Physically valid | OPEN inserted | Dominated | Self | Expanded arrivals |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| STRAIGHT_LEVEL | 29999 | 0 | 0 | 29999 | 12853 | 17146 | 0 | 9880 |
| LEFT_LEVEL_TURN | 29999 | 0 | 0 | 29999 | 16722 | 13277 | 0 | 8297 |
| RIGHT_LEVEL_TURN | 29999 | 0 | 0 | 29999 | 16704 | 13295 | 0 | 8297 |
| STRAIGHT_CLIMB | 29999 | 27228 | 0 | 2771 | 840 | 1931 | 0 | 320 |
| STRAIGHT_DESCENT | 29999 | 0 | 0 | 29999 | 7803 | 22196 | 0 | 3205 |

## XY corridor spread

Perpendicular deviation (actual physical poses): median 24.21 m; p90 47.90 m; p95 48.33 m; max 48.75 m.

| Band | Expanded states |
|---:|---:|
| <=100m | 30000 |
| <=250m | 30000 |
| <=500m | 30000 |
| <=1000m | 30000 |
| >1000m | 0 |

## OPEN at 30k

- Heap entries including stale: 21,353; stale entries: 1,302.
- Active non-stale OPEN representatives: 20,051 across 495 XY bins.
- Active OPEN mean Z / heading bins per XY: 29.52 / 1.77.
- Best actual physical OPEN candidate: XY 23.52 m, |Z| 1.98 m, 3D 23.93 m.

That candidate is inside the 90 m XY / 10 m Z goal region but had not yet been popped when the fixed expansion budget fired.

## Final classification

**PRIMARY: A. Z DIVERSITY**

**SECONDARY: E. HEURISTIC / GOAL GUIDANCE**

Z multiplicity is widespread (median 37 and maximum 82 Z bins per XY), while heading diversity is only median 2 and all expanded poses stay within 49 m of the direct corridor. At 30k, an active non-stale OPEN representative is already inside the physical goal region, so frontier ordering/budget delays goal acceptance after vertical-state growth rather than a lateral detour or missing descent capability.

No SearchKey, heuristic, cost, primitive, safety, tolerance or expansion-limit setting was changed.
