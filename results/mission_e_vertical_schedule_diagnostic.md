# Mission E vertical schedule combinatorics

Diagnostic only: production Euclidean A*, BASIC primitives, 60 m / 5 m / 15 degree SearchKey and 30,000 limit are unchanged.

## Baseline

BASELINE REPRODUCED: **YES**.
Status: `EXPANSION_LIMIT`; expanded/generated/rejected: 30,000/149,995/95,073; peak OPEN: 21,356; best XY/|Z|: 120.00/4.36 m.

Mean Z/(XY,H): **36.06**.

## Top high-diversity groups

| (X,Y,H) | Z | Altitude m | Full/RLE schedules | D count | L count | First D m | Switches | Lineages / max variants |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| (11484, 69777, 6) | 82 | 3910.7-4400.0 | 82/82 | 0-77 | 0-77 | 0-0 | 0-76 | 1/82 |
| (11485, 69777, 6) | 82 | 3904.4-4393.6 | 82/82 | 1-78 | 0-77 | 0-0 | 0-77 | 1/82 |
| (11483, 69777, 6) | 81 | 3917.0-4400.0 | 81/81 | 0-76 | 0-76 | 0-0 | 0-75 | 1/81 |
| (11486, 69777, 6) | 81 | 3904.4-4387.3 | 81/81 | 2-78 | 1-77 | 0-0 | 2-78 | 1/81 |
| (11482, 69777, 6) | 80 | 3923.4-4400.0 | 80/80 | 0-75 | 0-75 | 0-0 | 0-74 | 1/80 |
| (11487, 69777, 6) | 80 | 3904.4-4380.9 | 80/80 | 3-78 | 2-77 | 0-0 | 4-79 | 1/80 |
| (11488, 69777, 6) | 79 | 3904.4-4374.6 | 79/79 | 4-78 | 3-77 | 0-0 | 6-80 | 1/79 |
| (11481, 69777, 6) | 78 | 3929.7-4400.0 | 78/78 | 0-74 | 0-74 | 0-0 | 0-73 | 1/78 |
| (11489, 69777, 6) | 78 | 3904.4-4368.2 | 78/78 | 5-78 | 4-77 | 0-0 | 8-81 | 1/78 |
| (11480, 69777, 6) | 77 | 3936.1-4400.0 | 77/77 | 0-73 | 0-73 | 0-0 | 0-72 | 1/77 |
| (11490, 69777, 6) | 77 | 3904.4-4361.9 | 77/77 | 6-78 | 5-77 | 0-0 | 10-82 | 1/77 |
| (11479, 69777, 6) | 76 | 3942.4-4400.0 | 76/76 | 0-72 | 0-72 | 0-0 | 0-71 | 1/76 |
| (11491, 69777, 6) | 76 | 3904.4-4355.5 | 76/76 | 7-78 | 6-77 | 0-0 | 12-83 | 1/76 |
| (11492, 69777, 6) | 75 | 3904.4-4349.1 | 75/75 | 8-78 | 7-77 | 0-0 | 14-84 | 1/75 |
| (11478, 69777, 6) | 74 | 3948.8-4400.0 | 74/74 | 0-71 | 0-71 | 0-0 | 0-70 | 1/74 |
| (11493, 69777, 6) | 74 | 3904.4-4342.8 | 74/74 | 9-78 | 8-77 | 0-0 | 16-85 | 1/74 |
| (11477, 69777, 6) | 73 | 3955.1-4400.0 | 73/73 | 0-70 | 0-70 | 0-0 | 0-69 | 1/73 |
| (11494, 69777, 6) | 73 | 3904.4-4336.4 | 73/73 | 10-78 | 9-77 | 0-0 | 18-86 | 1/73 |
| (11476, 69777, 6) | 72 | 3961.5-4400.0 | 72/72 | 0-69 | 0-69 | 0-0 | 0-68 | 1/72 |
| (11495, 69777, 6) | 72 | 3904.4-4330.1 | 72/72 | 11-78 | 10-77 | 0-0 | 20-87 | 1/72 |

Representative schedules (highest / median / lowest altitude) are preserved in JSON for all top groups; their full strings are omitted here to keep the report inspectable.

## Timing, mode switches, and route sections

Same descent count but different timing accounts for **4.05%** of expanded Z representatives (1,216/30,000); remaining states differ in total descent amount or share identical timing.

| Route third | Distinct schedules | Z states | Schedule/Z |
|---:|---:|---:|---:|
| first | 2553 | 8438 | 0.303 |
| middle | 6512 | 18403 | 0.354 |
| final | 2055 | 3159 | 0.651 |

Mode switches per prefix: min/median/p90/max = 0/37.0/87.0/152; zero-switch states = 524. Full distribution is in JSON.

## Sustained vs fragmented

These classes overlap: EARLY/LATE describe onset; SUSTAINED/FRAGMENTED describe descent-run structure.

| Class | States | Avg g | Avg remaining XY | Avg |Z| | Avg Z/(XY,H) |
|---:|---:|---:|---:|---:|---:|
| NO_DESCENT_YET | 446 | 2125.0 | 7178.0 | 500.0 | 33.1 |
| EARLY_AT_START | 29552 | 4145.8 | 5168.4 | 282.0 | 46.7 |
| FRAGMENTED | 14179 | 4065.5 | 5244.5 | 361.5 | 46.7 |
| SUSTAINED | 2968 | 3257.3 | 6061.2 | 200.4 | 50.3 |
| LATE_AFTER_START | 2 | 247.6 | 9057.0 | 493.6 | 2.0 |

Thresholds are observed quantiles: {"sustained_fragmentation_ratio_p10": 0.18518518518518517, "fragmented_fragmentation_ratio_p90": 1.0, "early_definition": "first descent begins at start (observed mass point)", "late_definition": "first descent begins after start"}.

## Decision branching and terrain causality

LEVEL and DESCENT both OPEN-eligible: **4,801/30,000 (16.00%)**. Only level / only descent / neither: 8,052 / 3,002 / 14,145.
Descent→level interruptions: terrain-forced 0; non-terrain-forced 76; unknown 0.

## Near-goal and macro opportunity

Near-goal expanded states (<=500 m): 46; goal-satisfying OPEN states: 9; fragmented / sustained by observed thresholds: 26 / 0. Their descriptor samples are in JSON.
Best partial path descent-run summary: 76 runs, length distribution {1: 75, 3: 1}, max 3 steps; runs >=3: 1.

## Final classification

PRIMARY Z-DIVERSITY CAUSE: **A. VERTICAL TIMING COMBINATORICS**. In the top 20 groups, 1,540/1,540 representatives have distinct vertical RLE schedules while every inspected group has one geometric lineage. The narrower same-descent-count/different-order screen is 4.05%, so the evidence is repeated LEVEL-vs-DESCENT decisions (including different descent amounts), not merely permutations with an equal descent count. Of observed descent→level interruptions, 0 are terrain-forced and 76 are not. VERTICAL DECISION COMPRESSION POTENTIAL: **MODERATE**. LONGER VERTICAL PRIMITIVE JUSTIFIED: **NO**: near-goal/best-prefix evidence is predominantly fragmented, so a sustained-descent macro is not supported without a design that retains selectable timing and full terrain/envelope validation.

Production behavior changed: **NO**. Full test suite: see execution record. RESULT: **PASS**.
