# Pose-Aware Fixed-Wing Benchmark Report

## Run contract

- Planner horizontal speed: **40 m/s**, zero wind.
- Safety: continuous physical trajectory, true-curve swept terrain coverage.
- Effective minimum AGL: **100 m**; lateral buffer: **0 m**.
- Search key only: **60 m XY / 5 m Z / 15 deg heading**.
- Search policy: bounded approximate, one physical representative per key.
- Active primitives: straight level, left/right level turn, straight climb,
  straight descent. Spiral, loiter and combined turns are inactive.
- Goal acceptance: 90 m XY and 10 m altitude tolerance.

## Validation

`python -B -m unittest discover -v` completed successfully: **41 / 41 tests
passed**. The suite includes continuous-pose propagation, turn geometry,
curve-to-chord safety coverage, terrain/AGL rejection, search-key handling,
same-key replacement and self-transition accounting.

## Canonical Mission A/B

| Mission | Entry/goal MSL | Status | Expanded | Runtime | Path | Min AGL | Goal XY / Z error |
|---|---:|---|---:|---:|---:|---:|---:|
| A easy/open | 3500 / 3500 m | FOUND | 1,201 | 0.65 s | 1,070.16 m, 16 segments | 134.65 m | 82.88 m / 0.00 m |
| B relief | 3600 / 3600 m | FOUND | 466 | 0.25 s | 1,900.42 m, 13 segments | 104.23 m | 89.26 m / 6.34 m |

Mission B's physical solution contains 10 level turns and one descent. Its 37
same-key self-transitions were measured and suppressed as normal search loops;
they were not snapped or altered physically.

## Distant real-terrain missions

| Mission | Physical separation | MSL start/goal | Status | Expanded | Runtime | Path / closest result | Min AGL |
|---|---:|---:|---|---:|---:|---|---:|
| C south | 3.0 km | 3800 / 3800 m | FOUND | 10,789 | 5.97 s | 2,922.64 m, 41 segments | 290.36 m |
| D east | 3.0 km | 3800 / 3800 m | FOUND | 8,928 | 4.81 s | 2,927.10 m, 43 segments | 228.13 m |
| E long descent | 9.3 km | 4400 / 3900 m | EXPANSION_LIMIT | 30,000 | 15.40 s | best safe partial: 9,180 m, 153 segments | validated edge-by-edge |
| F turn-required diagonal | 2.29 km | 3700 / 3700 m | FOUND | 5,942 | 3.22 s | 2,330.84 m, 18 segments | 109.46 m |

### Mission E - controlled long descent finding

Mission E required a 500 m descent over a 9.3 km cross-ROI flight. It did not
reach a formal solution within the 30,000-expansion budget, but the best fully
validated physical chain reached 9,180 m traveled and `3904.36 m MSL`: only
4.36 m from the target altitude. It was 120 m from the goal in XY, just 30 m
outside the 90 m tolerance.

This demonstrates that descent primitives propagate physically. The limit is
search scale, not lack of descent capability: peak OPEN was 21,356, all 30,000
expanded keys were unique, and mean Z diversity was 37.42 bins per occupied XY
bin. `EXPANSION_LIMIT` is a budget outcome, not an unreachability claim.

### Mission F - required curved approach

Mission F starts at the actual navigation bearing to its diagonal goal
(`135 deg`); no artificial heading mismatch was introduced. Its found physical
solution has **5 left turns, 6 right turns, one climb and two descents**. The
final pose is inside the goal region (79.14 m XY and 9.47 m Z error), while
preserving continuous turn arcs and swept terrain safety.

## Visual evidence

- [Distant trajectories, including Mission E best partial path](far_pose_aware_trajectories.png)
- [Mission F turn-required continuous trajectory](turn_required_trajectory.png)
- [Mission A/B raw benchmark data](mission_ab_pose_aware_baseline.json)
- [Distant mission raw benchmark data](far_pose_aware_benchmark.json)
- [Mission F raw benchmark data](turn_required_benchmark.json)

## Interpretation

The basic fixed-wing model works across A/B, the turn-required diagonal route,
and the 3 km real-terrain routes. The 9.3 km controlled-descent stress case
provides clear scaling evidence: Z diversity and same-key dominance work grow
sharply before goal acceptance. This is the evidence base for the next
heuristic/key/cost tuning stage; no such tuning was applied in these runs.
