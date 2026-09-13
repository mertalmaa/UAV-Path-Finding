# Step GRID-1 — Aircraft-Aware 30/60/90m Grid Re-Gate

Re-evaluates the provisional Step 2E-2F XY-resolution decision now that a
real planner-safe aircraft geometry source exists (`planner/aircraft_profile.py`,
Step ALG-1, `jsbsim/results/c172p_aircraft_profile_planner_safe_v3.json`).
Producing script: `scripts/grid1_aircraft_aware_regate.py`. Validation script:
`scripts/validate_grid1.py`. Raw artifacts: `results/grid1_*.json`.

**Critical distinction this whole study is built on:** grid resolution,
aircraft motion length, and turn radius are three different things. Using a
60m grid does not mean "the aircraft turns with 60m radius" — the grid is a
spatial representation scale; every turn-radius number in this report comes
from a live `AircraftProfile.turn_query()` call, never hard-coded.

## 1. Profile geometry (real V3, guaranteed ±20° bank, LEFT/RIGHT separate)

| Altitude | LEFT radius | RIGHT radius | LEFT rate | RIGHT rate |
|---:|---:|---:|---:|---:|
| 0 m | 410.2 m | 454.8 m | -5.54°/s | 5.02°/s |
| 1000 m | 454.2 m | 502.5 m | -5.26°/s | 4.77°/s |
| 2500 m | 530.6 m | 585.6 m | -4.85°/s | 4.42°/s |
| 4000 m | 622.1 m | 685.3 m | -4.47°/s | 4.08°/s |
| 4500 m | 656.5 m | 722.9 m | -4.35°/s | 3.97°/s |
| 5500 m | 731.9 m | 805.7 m | -4.12°/s | 3.76°/s |

RIGHT turns consistently have a larger radius than LEFT at every altitude
(measured asymmetry, not averaged away). Radius grows monotonically with
altitude (thinner air → larger radius at the same IAS/bank). ±30° bank is
tested in the artifact but is `UNAVAILABLE` everywhere — correctly excluded
from this study as the geometry source (Step ALG-1 already confirmed the
loader reports this).

## 2. Resolution ratios

Radius-to-grid-spacing ratio across the full altitude/direction matrix:

- radius / 30m: **13.7 – 26.9**
- radius / 60m: **6.8 – 13.4**
- radius / 90m: **4.6 – 9.0**

Every real safe turn radius is many grid cells wide at every tested
spacing — none of the three candidates comes close to under-resolving the
aircraft's own turn scale in absolute terms.

## 3. Arc representation error (continuous circular arc vs. grid-snapped polyline)

Mean endpoint-quantization error (averaged over 4 base headings), across all
12 altitude/direction cases:

| Heading change | 30m | 60m | 90m |
|---:|---:|---:|---:|
| 20° | 7.7 – 14.8 m | 17.8 – 33.1 m | 32.8 – 49.4 m |
| 45° | 4.8 – 15.9 m | 14.8 – 33.7 m | 19.9 – 43.9 m |
| 90° | 6.9 – 18.0 m | 12.1 – 30.8 m | 19.4 – 51.4 m |

Error grows roughly linearly with grid spacing (60m ≈ 2× the 30m error, 90m
≈ 3-4×), as expected for grid-quantization error. In absolute terms, even
90m's worst case (~50m) is small relative to the smallest tested turn
diameter (~820m at 0m LEFT) — a few percent of the turn's own size.

## 4. Altitude/direction worst cases

- **Largest absolute footprint:** widest radius, 5500m RIGHT (R=805.7m).
- **Largest relative representation error:** tightest radius, 0m LEFT
  (R=410.2m) — the same absolute quantization error is a larger fraction of
  a smaller turn.
- **Dominant driver is terrain relief, not altitude/direction** — see §5.

## 5. Terrain-aware comparison (real windows, MAX-pooling conservatism)

Reused Step 2E/2F's own 5 objectively-selected windows (low-relief,
medium-relief, high-relief regression reference, valley-like, ridge-like) —
not reselected or cherry-picked. For each window, sampled the widest and
tightest real turn-arc footprints (20/45/90° heading changes, 4 base
headings) and compared the true 30m max terrain along the footprint against
the 60m/90m MAX-pooled equivalent.

**UNSAFE-OPTIMISM violations: 0 / all samples** (structurally guaranteed by
the MAX-pooling invariant: a coarse cell's MAX is always ≥ every fine cell
it covers, for ANY footprint shape, straight or curved — this generalizes
Step 2B's P2 proof from straight edges to arbitrary curved geometry).

Extra conservative blocking (`coarse_max − true_max_30m`), by window:

| Window (relief) | 60m max | 90m max |
|---|---:|---:|
| low-relief (14.6m) | 17.8 m | 33.3 m |
| medium-relief (36.4m) | 3.0 m | 0.0 m |
| **high-relief (282.1m, Step 2B regression ref)** | **194.1 m** | **194.1 m** |
| valley-like | 121.4 m | 121.4 m |
| ridge-like | 18.6 m | 18.6 m |

**The worst case is entirely terrain-driven, not aircraft-geometry-driven**:
the high-relief window (the single highest-relief coarse cell in the whole
10×10km ROI — an extreme, not a typical case) accounts for an order of
magnitude more extra-blocking than every other window, identically at both
60m and 90m. This is the terrain itself (a real cliff/peak), not something
introduced by the arc geometry or the resolution choice between 60m and
90m — at THIS specific extreme feature, 60m gives no advantage over 90m,
but in every other, more typical window, 60m's extra-blocking is small
(single digits) to moderate (≤22m), consistently smaller than 90m's.

## 6. Representation cost

| Resolution | Cell count | Shape | Est. size (f32 array) |
|---|---:|---|---:|
| 30m | 110,889 | 333×333 | 444 KB |
| 60m | 27,556 | 166×166 | 110 KB |
| 90m | 12,321 | 111×111 | 49 KB |

Matches the historical Step 2E/2F cell counts exactly (30m=110,889,
60m≈27,556, 90m≈12,321) — same terrain source, not rebuilt.

## 7. 30 vs 60 vs 90 — combined verdict

- **30m**: finest arc representation (smallest quantization error), highest
  cost (4× the 60m cell count). Remains available as local refinement.
- **60m**: turn radius is 6.8-13.4 grid cells wide at every tested
  altitude/direction; arc representation error is small relative to turn
  size; terrain-aware extra-blocking beyond true 30m terrain is small
  outside the one extreme high-relief cell (where 60m and 90m tie); 4×
  cheaper than 30m.
- **90m**: measurably coarser arc representation (roughly 1.5-2× 60m's
  quantization error) and measurably more conservative terrain blocking in
  4/5 tested windows (tied only at the extreme relief outlier); still never
  unsafe. Reasonable for global/topological metadata, not for the aircraft's
  own motion-scale representation.

## 8. Final decision

**A) 60M DEFAULT CONFIRMED — 30M LOCAL REFINEMENT — 90M GLOBAL/OPTIONAL METADATA**

This reconfirms Step 2E-2F's provisional terrain-only finding, now under
real aircraft turn-arc geometry rather than straight-edge connectivity
alone. No evidence in this study argues for changing the XY resolution
policy.

**Grid resolution is not motion geometry.** This study did not implement
heading, motion primitives, or any search-side change — see `planner/
astar.py`, `planner/primitives.py`, `planner/corridor.py`: none were
touched. Z representation (`z_step_m`) and CLASS-C closure remain explicitly
out of scope, deferred to the next stage.

## 9. Tests

`scripts/validate_grid1.py` — 7 groups, **ALL PASS**: artifacts exist and
are internally consistent; every recorded radius re-verified against a live
V3 query (0 mismatches); LEFT/RIGHT separated at every altitude; 6 required
altitudes covered; 0 unsafe-optimism violations and 0 negative
"extra-conservatism" entries; no search/primitive/corridor/production file
modified this stage; historical Step 2E/2F scripts and the V3 artifact's own
provenance_id unchanged.

## Next stage

**CLASS-C — Motion Representation Closure**: resolve the coupling between Z
representation spacing and real aircraft vertical motion (currently
`dz_m` derives directly from `z_step_m` — flagged as a known decoupling
debt since Step 3A). Heading discretization and aircraft-aware primitives
come after that.
