# Mission E Z-state redundancy diagnostic

Diagnostic only. Both runs use BASIC (`enable_combined_turns=False`), the fixed 60 m / 5 m / 15 degree SearchKey and 30,000-expansion limit. No planner policy was changed.

## Baseline reproduction

| Run | Status | Expanded | Generated | Rejected | Peak OPEN | Runtime s | Best XY / |Z| m | First goal insert | Goal pop |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| EUCLIDEAN | EXPANSION_LIMIT | 30,000 | 149,995 | 95,073 | 21,356 | 14.17 | 120.00/4.36 | 28618 | - |
| VERTICAL_REACHABILITY | EXPANSION_LIMIT | 30,000 | 149,995 | 95,076 | 21,352 | 13.80 | 120.00/4.36 | 28618 | - |

## Z representatives per (XY, heading)

| Run | Mean | Median | P90 | P95 | Max |
|---|---:|---:|---:|---:|---:|
| EUCLIDEAN | 36.06 | 36.00 | 60.90 | 68.00 | 82 |
| VERTICAL_REACHABILITY | 36.06 | 36.00 | 60.90 | 68.00 | 82 |

## Pairwise directional candidates

A candidate means only: same XY/heading bucket, `g_A >= g_B`, and A is no closer to goal altitude. It is **not** a pruning rule. Local envelope/clearance checks are reported separately and terrain ahead is intentionally not inferred.

| Run | Pairs | Candidates | % pairs | Locally unrefuted |
|---:|---:|---:|---:|---:|
| EUCLIDEAN | 681,697 | 608 | 0.09% | 390 |
| VERTICAL_REACHABILITY | 681,704 | 608 | 0.09% | 390 |

### Redundancy ratio by location

| Run / region | Raw Z states | Obvious candidates | Non-dominated | Ratio |
|---:|---:|---:|---:|---:|
| EUCLIDEAN | 30000 | 608 | 29392 | 2.03% |
| VERTICAL_REACHABILITY | 30000 | 608 | 29392 | 2.03% |
| EUCLIDEAN first_third | 8438 | 0 | 8438 | 0.00% |
| EUCLIDEAN middle_third | 18403 | 337 | 18066 | 1.83% |
| EUCLIDEAN final_third | 3159 | 271 | 2888 | 8.58% |
| VERTICAL_REACHABILITY first_third | 8438 | 0 | 8438 | 0.00% |
| VERTICAL_REACHABILITY middle_third | 18403 | 337 | 18066 | 1.83% |
| VERTICAL_REACHABILITY final_third | 3159 | 271 | 2888 | 8.58% |
| EUCLIDEAN >5km | 16508 | 38 | 16470 | 0.23% |
| EUCLIDEAN 2-5km | 12502 | 443 | 12059 | 3.54% |
| EUCLIDEAN 1-2km | 814 | 100 | 714 | 12.29% |
| EUCLIDEAN 500-1000m | 130 | 22 | 108 | 16.92% |
| EUCLIDEAN <500m | 46 | 5 | 41 | 10.87% |
| VERTICAL_REACHABILITY >5km | 16508 | 38 | 16470 | 0.23% |
| VERTICAL_REACHABILITY 2-5km | 12504 | 443 | 12061 | 3.54% |
| VERTICAL_REACHABILITY 1-2km | 814 | 100 | 714 | 12.29% |
| VERTICAL_REACHABILITY 500-1000m | 130 | 22 | 108 | 16.92% |
| VERTICAL_REACHABILITY <500m | 44 | 5 | 39 | 11.36% |

### EUCLIDEAN: top 20 (XY, heading) groups

| (X,Y,H) bins | Z | Altitude m | g | Goal distance m | Terrain MSL m | AGL m | Candidate pairs |
|---:|---:|---:|---:|---:|---:|---:|---:|
| (11484, 69777, 6) | 82 | 3910.7-4400.0 | 4620.0-4645.8 | 4680.0-4706.6 | 3609.5-3609.5 | 301.2-790.5 | 4 |
| (11485, 69777, 6) | 82 | 3904.4-4393.6 | 4680.3-4706.2 | 4620.0-4646.3 | 3645.3-3645.3 | 259.0-748.3 | 4 |
| (11483, 69777, 6) | 81 | 3917.0-4400.0 | 4560.0-4585.5 | 4740.0-4766.3 | 3559.7-3559.7 | 357.3-840.3 | 4 |
| (11486, 69777, 6) | 81 | 3904.4-4387.3 | 4740.7-4766.2 | 4560.0-4586.0 | 3675.8-3675.8 | 228.5-711.4 | 4 |
| (11482, 69777, 6) | 80 | 3923.4-4400.0 | 4500.0-4525.2 | 4800.1-4826.0 | 3525.3-3525.3 | 398.1-874.7 | 4 |
| (11487, 69777, 6) | 80 | 3904.4-4380.9 | 4801.0-4826.2 | 4500.0-4525.6 | 3674.7-3674.7 | 229.7-706.3 | 4 |
| (11488, 69777, 6) | 79 | 3904.4-4374.6 | 4861.3-4886.2 | 4440.0-4465.3 | 3656.6-3656.6 | 247.8-718.0 | 4 |
| (11481, 69777, 6) | 78 | 3929.7-4400.0 | 4440.0-4464.8 | 4860.1-4885.7 | 3475.9-3475.9 | 453.9-924.1 | 3 |
| (11489, 69777, 6) | 78 | 3904.4-4368.2 | 4921.7-4946.2 | 4380.0-4405.0 | 3620.5-3620.5 | 283.8-747.7 | 4 |
| (11480, 69777, 6) | 77 | 3936.1-4400.0 | 4380.0-4404.5 | 4920.1-4945.3 | 3428.9-3428.9 | 507.1-971.1 | 3 |
| (11490, 69777, 6) | 77 | 3904.4-4361.9 | 4982.0-5006.2 | 4320.0-4344.6 | 3596.7-3596.7 | 307.7-765.2 | 4 |
| (11479, 69777, 6) | 76 | 3942.4-4400.0 | 4320.0-4344.2 | 4980.2-5005.0 | 3383.9-3383.9 | 558.5-1016.1 | 3 |
| (11491, 69777, 6) | 76 | 3904.4-4355.5 | 5042.4-5066.2 | 4260.0-4284.3 | 3565.8-3565.8 | 338.6-789.7 | 4 |
| (11492, 69777, 6) | 75 | 3904.4-4349.1 | 5102.7-5126.2 | 4200.0-4223.9 | 3541.4-3541.4 | 363.0-807.8 | 4 |
| (11478, 69777, 6) | 74 | 3948.8-4400.0 | 4260.0-4283.8 | 5040.2-5064.7 | 3353.0-3353.0 | 595.7-1047.0 | 2 |
| (11493, 69777, 6) | 74 | 3904.4-4342.8 | 5163.0-5186.2 | 4140.0-4163.6 | 3521.1-3521.1 | 383.3-821.7 | 4 |
| (11477, 69777, 6) | 73 | 3955.1-4400.0 | 4200.0-4223.5 | 5100.3-5124.5 | 3316.1-3316.1 | 639.0-1083.9 | 2 |
| (11494, 69777, 6) | 73 | 3904.4-4336.4 | 5223.4-5246.2 | 4080.0-4103.3 | 3493.5-3493.5 | 410.9-842.9 | 4 |
| (11476, 69777, 6) | 72 | 3961.5-4400.0 | 4140.0-4163.2 | 5160.4-5184.2 | 3278.1-3278.1 | 683.3-1121.9 | 2 |
| (11495, 69777, 6) | 72 | 3904.4-4330.1 | 5283.7-5306.2 | 4020.0-4042.9 | 3476.3-3476.3 | 428.1-853.8 | 4 |

### EUCLIDEAN: offline Z rebin of the 30,000 expanded physical representatives

| Z m | Unique keys | Reduction | Mean Z/XY | Mean Z/(XY,H) | States merged | g diff | AGL diff | Goal-Z diff |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.0 | 30,000 | 0.00% | 37.42 | 36.06 | 0 | 0 | 0 | 0 |
| 10.0 | 19,223 | 35.92% | 23.89 | 23.10 | 10777 | 0 | 0 | 0 |
| 20.0 | 10,137 | 66.21% | 12.56 | 12.18 | 19863 | 0 | 0 | 8878 |
| 50.0 | 4,704 | 84.32% | 5.72 | 5.65 | 25296 | 0 | 3612 | 3905 |
| 100.0 | 2,825 | 90.58% | 3.43 | 3.40 | 27175 | 0 | 2118 | 2226 |

### EUCLIDEAN: real Mission E vertical successor same-key collapse

Full SearchKey columns include XY. Because real straight primitives advance 60 m horizontally, they are expected to be near zero; the Z-component columns are the meaningful vertical-progress signal.

| Z m | Climb Z-component collapse | Descent Z-component collapse | Full-key climb / descent |
|---:|---:|---:|---:|
| 5.0 | 749/2,771 (27.03%) | 0/29,999 (0.00%) | 0/2,771 / 0/29,999 |
| 10.0 | 1,593/2,771 (57.49%) | 10,672/29,999 (35.57%) | 0/2,771 / 0/29,999 |
| 20.0 | 2,246/2,771 (81.05%) | 20,223/29,999 (67.41%) | 0/2,771 / 0/29,999 |
| 50.0 | 2,352/2,771 (84.88%) | 25,609/29,999 (85.37%) | 0/2,771 / 0/29,999 |
| 100.0 | 2,517/2,771 (90.83%) | 27,661/29,999 (92.21%) | 0/2,771 / 0/29,999 |

### VERTICAL_REACHABILITY: top 20 (XY, heading) groups

| (X,Y,H) bins | Z | Altitude m | g | Goal distance m | Terrain MSL m | AGL m | Candidate pairs |
|---:|---:|---:|---:|---:|---:|---:|---:|
| (11484, 69777, 6) | 82 | 3910.7-4400.0 | 4620.0-4645.8 | 4680.0-4706.6 | 3609.5-3609.5 | 301.2-790.5 | 4 |
| (11485, 69777, 6) | 82 | 3904.4-4393.6 | 4680.3-4706.2 | 4620.0-4646.3 | 3645.3-3645.3 | 259.0-748.3 | 4 |
| (11483, 69777, 6) | 81 | 3917.0-4400.0 | 4560.0-4585.5 | 4740.0-4766.3 | 3559.7-3559.7 | 357.3-840.3 | 4 |
| (11486, 69777, 6) | 81 | 3904.4-4387.3 | 4740.7-4766.2 | 4560.0-4586.0 | 3675.8-3675.8 | 228.5-711.4 | 4 |
| (11482, 69777, 6) | 80 | 3923.4-4400.0 | 4500.0-4525.2 | 4800.1-4826.0 | 3525.3-3525.3 | 398.1-874.7 | 4 |
| (11487, 69777, 6) | 80 | 3904.4-4380.9 | 4801.0-4826.2 | 4500.0-4525.6 | 3674.7-3674.7 | 229.7-706.3 | 4 |
| (11488, 69777, 6) | 79 | 3904.4-4374.6 | 4861.3-4886.2 | 4440.0-4465.3 | 3656.6-3656.6 | 247.8-718.0 | 4 |
| (11481, 69777, 6) | 78 | 3929.7-4400.0 | 4440.0-4464.8 | 4860.1-4885.7 | 3475.9-3475.9 | 453.9-924.1 | 3 |
| (11489, 69777, 6) | 78 | 3904.4-4368.2 | 4921.7-4946.2 | 4380.0-4405.0 | 3620.5-3620.5 | 283.8-747.7 | 4 |
| (11480, 69777, 6) | 77 | 3936.1-4400.0 | 4380.0-4404.5 | 4920.1-4945.3 | 3428.9-3428.9 | 507.1-971.1 | 3 |
| (11490, 69777, 6) | 77 | 3904.4-4361.9 | 4982.0-5006.2 | 4320.0-4344.6 | 3596.7-3596.7 | 307.7-765.2 | 4 |
| (11479, 69777, 6) | 76 | 3942.4-4400.0 | 4320.0-4344.2 | 4980.2-5005.0 | 3383.9-3383.9 | 558.5-1016.1 | 3 |
| (11491, 69777, 6) | 76 | 3904.4-4355.5 | 5042.4-5066.2 | 4260.0-4284.3 | 3565.8-3565.8 | 338.6-789.7 | 4 |
| (11492, 69777, 6) | 75 | 3904.4-4349.1 | 5102.7-5126.2 | 4200.0-4223.9 | 3541.4-3541.4 | 363.0-807.8 | 4 |
| (11478, 69777, 6) | 74 | 3948.8-4400.0 | 4260.0-4283.8 | 5040.2-5064.7 | 3353.0-3353.0 | 595.7-1047.0 | 2 |
| (11493, 69777, 6) | 74 | 3904.4-4342.8 | 5163.0-5186.2 | 4140.0-4163.6 | 3521.1-3521.1 | 383.3-821.7 | 4 |
| (11477, 69777, 6) | 73 | 3955.1-4400.0 | 4200.0-4223.5 | 5100.3-5124.5 | 3316.1-3316.1 | 639.0-1083.9 | 2 |
| (11494, 69777, 6) | 73 | 3904.4-4336.4 | 5223.4-5246.2 | 4080.0-4103.3 | 3493.5-3493.5 | 410.9-842.9 | 4 |
| (11476, 69777, 6) | 72 | 3961.5-4400.0 | 4140.0-4163.2 | 5160.4-5184.2 | 3278.1-3278.1 | 683.3-1121.9 | 2 |
| (11495, 69777, 6) | 72 | 3904.4-4330.1 | 5283.7-5306.2 | 4020.0-4042.9 | 3476.3-3476.3 | 428.1-853.8 | 4 |

### VERTICAL_REACHABILITY: offline Z rebin of the 30,000 expanded physical representatives

| Z m | Unique keys | Reduction | Mean Z/XY | Mean Z/(XY,H) | States merged | g diff | AGL diff | Goal-Z diff |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.0 | 30,000 | 0.00% | 37.41 | 36.06 | 0 | 0 | 0 | 0 |
| 10.0 | 19,222 | 35.93% | 23.89 | 23.10 | 10778 | 0 | 0 | 0 |
| 20.0 | 10,136 | 66.21% | 12.55 | 12.18 | 19864 | 0 | 0 | 8878 |
| 50.0 | 4,704 | 84.32% | 5.72 | 5.65 | 25296 | 0 | 3611 | 3907 |
| 100.0 | 2,825 | 90.58% | 3.43 | 3.40 | 27175 | 0 | 2117 | 2226 |

### VERTICAL_REACHABILITY: real Mission E vertical successor same-key collapse

Full SearchKey columns include XY. Because real straight primitives advance 60 m horizontally, they are expected to be near zero; the Z-component columns are the meaningful vertical-progress signal.

| Z m | Climb Z-component collapse | Descent Z-component collapse | Full-key climb / descent |
|---:|---:|---:|---:|
| 5.0 | 749/2,770 (27.04%) | 0/29,999 (0.00%) | 0/2,770 / 0/29,999 |
| 10.0 | 1,592/2,770 (57.47%) | 10,672/29,999 (35.57%) | 0/2,770 / 0/29,999 |
| 20.0 | 2,245/2,770 (81.05%) | 20,224/29,999 (67.42%) | 0/2,770 / 0/29,999 |
| 50.0 | 2,350/2,770 (84.84%) | 25,609/29,999 (85.37%) | 0/2,770 / 0/29,999 |
| 100.0 | 2,516/2,770 (90.83%) | 27,661/29,999 (92.21%) | 0/2,770 / 0/29,999 |

## Final decision evidence

EUCLIDEAN VS VERTICAL STRUCTURE: **SAME**. 5M Z REDUNDANCY: **LOW** (2.03% of expanded representatives are directional obvious-worse states). Z-DOMINANCE POTENTIAL: **LOW**: the pair screen is only a local candidate test, and terrain ahead is not inferable from a shared XY/heading bucket. COARSER Z RISK: **HIGH from 10 m**: actual vertical progress loses its Z component on 57.49% of climbs and 35.57% of descents; even 5 m loses 27.03% of climb steps. Thus 10/20/50/100 m could reduce offline state count but are not shown safe alternatives. The literal full SearchKey transition collapse remains zero because every tested straight primitive also advances horizontally. Vertical heuristic changes conclusion: **NO**.

Production behavior changed: **NO**.
Full test suite: see execution record.
