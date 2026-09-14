# UAV Pathfinder — current source of truth

Last reviewed: 2026-09-14.

This document describes the repository as it exists now. Historical stage logs
were consolidated into `docs/HISTORY.md`; Git history remains the authoritative
archive for raw stage reports and deleted experiment outputs.

## Mission contract

The planner seeks a safe fixed-wing path between exact mission endpoints over
terrain. A valid path must remain inside the configured search bounds, preserve
minimum AGL clearance along every sampled primitive, and use motion that is
geometrically and aircraft-capability feasible.

Failure semantics are intentionally distinct:

- `FOUND` means a path was found and must still pass explicit path validation.
- `NO_PATH` means the frontier was exhausted within the modeled state space.
- `TIMEOUT` and `EXPANSION_LIMIT` are budget outcomes, not reachability claims.
- A directly infeasible edge does not prove the destination is unreachable by a
  longer or differently shaped route.

In short: **directly infeasible != unreachable** and **timeout != unreachable**.

## Frozen flight / trajectory contract (0B)

This is the physical/navigation contract for the future pose-aware planner.
It freezes terminology and invariants only: it does **not** mean that
heading-aware search or curved primitives are already implemented.

### Physical pose and frame

A physical aircraft pose is conceptually:

```text
(x_m, y_m, z_msl_m, heading_deg)
```

- The coordinate frame is EPSG:32636 (UTM 36N): `x_m` is easting in metres
  and `y_m` is northing in metres.
- `z_msl_m` is altitude in metres MSL.  The model assumes DEM elevation and
  aircraft MSL altitude use a compatible vertical datum; that source-datum
  verification remains an external-data concern.
- `heading_deg` is the aircraft longitudinal-axis azimuth, not a separately
  stored ground track/course.  It is navigation heading: North=0 degrees,
  East=90, South=180, West=270.  Positive change is clockwise.  The canonical
  value is `heading_deg mod 360` in `[0, 360)`.
- Start heading is mandatory.  Goal heading is optional; when supplied, a
  future goal contract also carries a circular heading tolerance.  Heading
  tolerance must use wrapped angular difference, never ordinary subtraction.
- Heading discretization and state-key quantization are intentionally not
  selected by this contract.

### Zero-wind, fixed-speed kinematic convention

The first pose-aware planner has identically zero wind: east, north, and
vertical wind components are all `0.0 m/s`; there is no gust, turbulence,
drift, or wind state.

The C172P profile's `nominal_ias_context_mps` is frozen at **40.0 m/s** for
all planner primitives.  The planner's fixed-speed, zero-wind convention is:

```text
V_planar_mps = 40.0
ground-track direction = heading_deg
primitive_duration_s = actual horizontal trajectory arc length_m / 40.0
```

This is a planner kinematic convention, not a claim that IAS and TAS are
identical at every altitude, and does not make the planner time-state based.
The profile's reported nominal speed remains labelled IAS context.

### Grid, state, and trajectory separation

- A physical pose and its trajectory retain continuous floating-point
  coordinates.  They are never reconstructed from a DEM row/col or a
  quantized search key during successor propagation.
- DEM `(row, col)` is terrain indexing only.  A physical `(x, y)` may be
  mapped to row/col for terrain lookup, but that mapping must never move the
  aircraft to a cell centre.
- A search `state_key` is only open/closed/dominance bookkeeping.  It may be
  quantized later, but quantization must not silently alter a physically
  validated trajectory.
- Every future successor conceptually exposes `start_pose`,
  `physical_trajectory`, `end_pose`, and `state_key`.  Propagation, duration,
  cost, bounds, and safety use the actual trajectory/end pose.  Returned
  paths must preserve or reconstruct the validated physical trajectory, not
  merely a sequence of state keys.
- Safety samples the actual continuous trajectory.  Each sample may use
  `(x, y) -> (row, col) -> DEM elevation -> AGL`; reconstructing a straight
  segment between snapped state keys is forbidden for future curved motion.

### AGL and aircraft role

- `DEFAULT_CONFIG.min_agl_m` is **200 m**.
- The canonical Mission A/B benchmark runner explicitly overrides its
  effective `min_agl_m` to **100 m**.  Future benchmark artifacts must report
  their effective `min_agl_m` so those result classes cannot be confused.
- `AircraftProfile` is aircraft-neutral at its interface/core.  The current
  validated default profile and its provenance are C172P-specific.
- C172P is the current fixed-wing surrogate/reference aircraft used for
  characterization and validation.  Planner logic must not acquire
  C172P-specific branches.

## Current architecture

### Terrain and grid

- Terrain is queried in projected meters through `TerrainQuery`.
- Safety-conservative coarse terrain uses block maximum elevation; missing
  terrain is invalid rather than silently interpolated.
- Current grid decision: 60 m default planning scale, 30 m local refinement,
  and 90 m global/optional metadata.
- Grid resolution, motion length, and aircraft turn radius are separate
  concepts. A 60 m grid does not imply a 60 m turn radius.

### Altitude representation

- `CandidateZGenerator` deterministically derives terrain floor, ceiling, and
  exact mission altitude events.
- In CandidateZ search mode, successor altitudes come from CandidateZ events;
  off-lattice physical altitudes use stable integer encoding for state identity.
- `is_representable()` is a continuous floor-to-ceiling safety gate and is not
  equivalent to enumerating every candidate event.
- Representation answers whether an altitude may exist. It does not answer
  whether the aircraft can physically traverse an edge to it.
- CandidateZ is the single production altitude authority. Canonical state
  altitude IDs always decode to exact CandidateZ event altitudes; there is no
  regular-lattice fallback search mode.

### Motion and safety

- `planner.physical` supplies a passive, grid-independent continuous pose and
  sampled trajectory data model, plus mathematical straight/level-turn
  geometry from an already-consistent fixed-40 kinematic envelope.  It is not
  wired into A*, CandidateZ, or the active primitive set: current production
  motion remains the legacy grid-anchored straight-segment representation.
- Explicit-target primitives preserve arbitrary source/target altitudes without
  snapping to a regular altitude lattice.
- Multi-cell vertical motion sizes a horizontal horizon from the C172P profile's
  local safe vertical rate, then validates the concrete motion.
- A representable endpoint is not automatically a feasible edge.
- No residual climb progress, trend, bucket, reversal, or other search-history
  state is carried between edges.
- Primitive safety samples the full segment for terrain/AGL clearance; endpoint
  checks alone are insufficient.

### Aircraft profile

The only planner-accepted runtime artifact is:

`jsbsim/results/c172p_aircraft_profile_planner_safe_v3.json`

Contract:

- schema version 3 and `profile_stage=tested_planner_safe_envelope`;
- C172P, nominal 40 m/s IAS, domain 0–5500 m MSL;
- altitude-dependent straight, turn, climb, descent, and combined queries;
- left/right turn evidence remains separate;
- measured values never imply planner-safe availability;
- raw LUTs and superseded profile schemas are rejected by the production loader.

Planner-safe climb availability is strongest at low altitude, reduces through
the domain, and is unavailable at 5000–5500 m. Descent remains available across
the domain with a non-monotonic tested-safe command family. Combined climb is
unavailable; combined descent is available only in its validated high-altitude
range.

### Derived C172P LUT / planner envelope

`planner.derived_c172p.DerivedC172PEnvelope` is an explicit C172P reference
binding layered above the aircraft-neutral `AircraftProfile` interface. It
does not alter the V3 artifact or add an aircraft-specific branch to search;
it derives a complete, ideal fixed-wing envelope for future pose primitives.

- The frozen zero-wind planner speed remains **40 m/s**.
- V3 has validated level-turn rows at 20 degrees, not 25. The derived model's
  **25-degree reference bank** creates an analytic fixed-speed radius bound,
  but the selected radius is always the larger of that bound and the worst
  validated left/right LUT radius. It never claims 25-degree JSBSim validation
  or shrinks the tested-safe radius.
- The requested **5 m/s** vertical value is a hard magnitude ceiling, not an
  unsafe override: `abs(Vz_plan) = min(5 m/s, abs(Vz_LUT_safe))`. An
  `UNAVAILABLE` LUT answer remains unavailable.
- Each answer carries `LUT_SAFE_SOURCE`, `DERIVED_CONSERVATIVE`, or
  `UNSUPPORTED` provenance.

### Derived C172P maneuvers

The passive continuous layer now represents level straight, straight climb,
straight descent, left/right level turns, explicit `LOITER_ORBIT`, derived
climbing/descending turns, and full-circle `SPIRAL_UP` macros. Combined turns
use a declared 1.25 radius multiplier and 0.50 vertical-rate multiplier from
their separate source limits.
They are ideal kinematic planner maneuvers, **not** JSBSim-validated C172P
combined-flight claims.

One full spiral turn reports its physical orbit radius, horizontal arc,
duration, and altitude change. Terrain clearance and entry/exit-transition
margins are intentionally not hidden inside that radius. The added straight
vertical and helical trajectory geometry preserves continuous floating-point
poses and does not use DEM row/col or a search key.

### Conservative continuous trajectory terrain safety

`planner.trajectory_safety.evaluate_physical_trajectory_safety()` consumes a
`PhysicalTrajectory` directly and evaluates every actual `(x_m, y_m,
z_msl_m)` sample. It reuses the scalar terrain/AGL calculation used by legacy
primitive safety, but never reconstructs a trajectory from endpoints, grid
cells, or state keys.

- `effective_min_agl_m` and `required_max_sample_spacing_m` are explicit
  evaluator inputs. The AGL boundary is inclusive: `agl_m >= min_agl_m` is
  safe.
- The current working DEM's effective resolution is **30 m**
  (`DEFAULT_CONFIG.xy_resolution_m`). A trajectory records its observed sample
  interval and the evaluator retains an explicit spacing contract to avoid
  under-resolved or multi-revolution segments. For each accepted adjacent pair
  it performs conservative raster coverage of the physical chord.
- For turns/helices, chord coverage is expanded by an actual per-segment
  circular-arc sagitta: `R * (1 - cos(delta_heading_rad / 2))`, with `R`
  derived from that pair's horizontal arc distance and heading change. Thus a
  chord supercover alone is never mislabelled as true-curve coverage.
- `lateral_buffer_m` is separate from sagitta and `min_agl_m`. It creates a
  cacheable static terrain field whose value is the maximum DEM elevation in
  the requested horizontal neighborhood; NoData or a buffer extending beyond
  DEM coverage invalidates that field cell. `lateral_buffer_m=0` preserves
  centerline terrain semantics. A 100 m value, if selected later, is a planner
  safety/model margin and not a C172P wingspan claim.
- Physical ROI bounds are checked in continuous x/y before terrain lookup.
  Terrain row/col exists only inside `TerrainQuery`; no cell-centre snapping
  occurs. Outside-ROI, outside-DEM, NoData, invalid, and below-AGL samples
  fail closed. The compact result retains only minimum AGL and first failure,
  never a per-sample result list.
- With a chosen lateral buffer, the claim is **buffered terrain + point
  centerline**, within the current static DEM model. It remains distinct from
  full aircraft-volume safety: tracking error beyond the buffer, dynamic
  obstacles, wind drift, and vertical/DEM uncertainty remain future work.

### Pose-aware fixed-wing search baseline (2C--2D--3)

`planner.pose_search` is the active production search core and benchmark path.
It propagates a continuous `PhysicalPose(x_m, y_m, z_msl_m, heading_deg)` and
stores each incoming validated `PhysicalTrajectory` on the actual search node.
The only active primitives are 60 m level straight, left/right 15-degree level
turns, and 60 m horizontal-progress straight climb/descent. Spiral, loiter,
climbing turn and descending turn remain inactive.

`SearchKey(x_bin, y_bin, z_bin, heading_bin)` uses 60 m / 5 m / 15-degree
floor buckets with the fixed global origin and deterministic boundary epsilon.
It is strictly an open/closed/dominance key: it never reconstructs or moves a
physical pose. The first baseline is a **bounded approximate
single-representative search**: a same-key physical pose replaces the active
representative only at strictly lower g. Heap entries carry a node identity, so
an obsolete representative cannot expand after replacement. Parent links
identify actual nodes, not keys.

Every successor is constructed from `current_node.end_pose`, then passed to
continuous true-curve terrain/AGL safety before it receives a `SearchKey`.
The evaluator uses `config.min_agl_m`, `config.primitive_sample_spacing_m` and
`config.lateral_buffer_m` (first baseline: 100 m, 10 m, and 0 m respectively
for Mission A/B). Out-of-ROI, DEM and NoData failures remain fail-closed.

The edge cost is actual sampled geometric 3D trajectory length: a turn uses
its true arc and a climb/descent uses its continuous 3D travel length. The
heuristic is Euclidean 3D distance to the physical XY/Z goal tolerance region;
because every edge cost is at least its geometric endpoint displacement, it is
admissible. There are no new turn/climb/spiral weights or heuristic tuning.

Mission A/B start heading is the deterministic physical navigation bearing to
the physical goal. Their headed goal is optional and currently omitted. Since
fixed 60 m primitives cannot generally end at exact continuous goal XY, their
benchmark contract explicitly uses 90 m XY and 10 m altitude goal tolerances;
these are goal acceptance values, not state bins.

`CandidateZGenerator` is retained for historical terrain-event inspection and
legacy compatibility tests only. It is not called by pose-aware successor
generation and no dense Z lattice is reintroduced. `planner.astar` is likewise
a legacy compatibility surface; it is not an alternate production motion path.

## Current validation contract

`python -m unittest discover -v` covers:

- CandidateZ determinism;
- conservative terrain floor behavior;
- exact off-lattice mission altitudes;
- representation-neutral explicit-target primitives;
- multi-cell vertical motion;
- AircraftProfile V3 loading and raw-profile rejection;
- altitude-dependent aircraft capability;
- AGL and along-primitive terrain safety;
- a tiny CandidateZ-driven planner smoke test.

These tests are intentionally synthetic and fast. Long Mission B/C runs are not
part of repository-hygiene validation.

## Current mission status

The first pose-aware fixed-wing baseline is retained at
`results/mission_ab_pose_aware_baseline.json`. The historical terrain-floor
entry altitude placed the aircraft at only about 109 m AGL and caused every
first primitive to be rejected. The canonical pose-aware rerun therefore keeps
the same XY endpoints but explicitly defines an ROI-entry cruise altitude:
3500 m MSL for Mission A and 3600 m MSL for Mission B. This is a documented
mission initial-condition change, not search/cost/primitive tuning; it gives
the fixed-wing model enough initial swept-trajectory clearance to measure the
basic planner before spiral/loiter work.

The retained current benchmark evidence is `results/mission_ab_baseline.json`.

- Mission A: `FOUND`, safety PASS, minimum observed AGL about 100.07 m, exact
  goal, 1,429 expanded nodes.
- Mission B: `TIMEOUT` at the 300 s development watchdog after 22,249 expanded
  nodes. This is a performance/scaling blocker, not an unreachable verdict.
- Mission C was not run for the current CandidateZ + multi-cell bridge stage.

## Current blockers

1. Mission B does not complete within the recorded 300 s development watchdog.
   Candidate generation/edge evaluation performance must improve without cost
   retuning or weakening safety.
2. Heading-aware, aircraft-turn-radius motion primitives are not yet the active
   planner representation.
3. Full end-to-end Mission C evidence is absent for the current architecture.

## Permanent evidence retained

- Canonical planner-safe V3 aircraft profile and its referenced C172P
  characterization chain under `jsbsim/`.
- Current Mission A/B benchmark summary under `results/`.
- Working DEM and the cache used by the current real-terrain benchmark.

All stage-specific reports, duplicated planner audit JSON, old tuning tables,
and obsolete visualization/benchmark outputs are intentionally left to Git
history.
