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

This is the physical/navigation contract for the pose-aware planner. It
freezes terminology and invariants; the currently implemented successor policy
is described in the pose-aware search section below.

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

- `planner.physical` supplies the grid-independent continuous pose and sampled
  trajectory model used by the pose-aware A* core, including straight,
  level-turn, straight-vertical, and helical-turn geometry from the generic
  fixed-wing kinematic envelope.
- `planner.fixed_wing_envelope.FixedWingKinematicEnvelope` provides the authoritative
  flight performance envelope:
  - Horizontal kinematic speed: 40.0 m/s (zero-wind convention)
  - Max climb rate: +5.0 m/s
  - Max descent rate: -5.0 m/s
  - Bank angle: 25.0 deg (Turn radius = 349.89 m, Turn rate = 6.55 deg/s)
  - 60 m primitive horizontal distance ($dt = 1.5\text{ s}$, $\Delta z = \pm 7.5\text{ m}$)
- `planner.trajectory_safety.evaluate_physical_trajectory_safety()` consumes a
  `PhysicalTrajectory` directly and evaluates every actual `(x_m, y_m, z_msl_m)` sample.
- `effective_min_agl_m` and `required_max_sample_spacing_m` are explicit
  evaluator inputs. The AGL boundary is inclusive: `agl_m >= min_agl_m` is safe.
- The current working DEM's effective resolution is **30 m** (`DEFAULT_CONFIG.xy_resolution_m`).
  A trajectory records its observed sample interval and the evaluator retains an explicit
  spacing contract to avoid under-resolved or multi-revolution segments. For each accepted
  adjacent pair it performs conservative raster coverage of the physical chord.
- For turns/helices, chord coverage is expanded by an actual per-segment circular-arc
  sagitta: `R * (1 - cos(delta_heading_rad / 2))`.
- `lateral_buffer_m` is separate from sagitta and `min_agl_m`. It creates a cacheable
  static terrain field whose value is the maximum DEM elevation in the requested horizontal
  neighborhood.
- Physical ROI bounds are checked in continuous x/y before terrain lookup. Outside-ROI,
  outside-DEM, NoData, invalid, and below-AGL samples fail closed.

### Pose-aware fixed-wing search baseline (2C--2D--3)

`planner.pose_search` is the active production search core and benchmark path.
It propagates a continuous `PhysicalPose(x_m, y_m, z_msl_m, heading_deg)` and
stores each incoming validated `PhysicalTrajectory` on the actual search node.
The canonical default policy is BASIC: 60 m level straight, left/right
15-degree level turns, and 60 m horizontal-progress straight climb/descent.

Climbing/descending turns are implemented capabilities, not missing geometry.
When `PlannerConfig.enable_combined_turns=True`, four additional 15-degree
helical successors (`CLIMBING_LEFT_TURN`, `CLIMBING_RIGHT_TURN`,
`DESCENDING_LEFT_TURN`, `DESCENDING_RIGHT_TURN`) are eligible. They are
disabled by default so BASIC remains the canonical baseline; enabling them
does not bypass physical propagation, continuous true-curve safety, or
SearchKey/dominance. Spiral and loiter macros remain inactive.

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
- BASIC-vs-COMBINED pose-search activation, continuous safety, OPEN insertion,
  and final-path primitive identity.

These tests are intentionally synthetic and fast. Long Mission B/C runs are not
part of repository-hygiene validation.

## Current mission status

The controlled A--F benchmark source of truth is:

- `results/pose_aware_basic_af.json` for the canonical BASIC policy;
- `results/pose_aware_combined_af.json` for the exact same missions with only
  `enable_combined_turns=True` changed; and
- `results/pose_aware_basic_vs_combined.md` for the side-by-side comparison.

Both A/B retained endpoints use their documented explicit ROI-entry cruise
altitudes (3500 m MSL for A and 3600 m MSL for B). Those initial conditions
are unchanged across BASIC and COMBINED runs and are not search/cost tuning.

## Current blockers

1. The long Mission E controlled-descent case is the current scale diagnostic;
   the controlled BASIC/COMBINED comparison records whether global combined
   activation improves or regresses its bounded search performance.
2. Combined turns are available but intentionally disabled in the default
   policy pending evidence-driven activation rules. No heuristic, cost,
   SearchKey, or dominance tuning is implied by this switch.
3. Spiral and loiter macros remain inactive search capabilities.

## Permanent evidence retained

- Canonical planner-safe V3 aircraft profile and its referenced C172P
  characterization chain under `jsbsim/`.
- Controlled pose-aware BASIC/COMBINED A--F benchmark artifacts under `results/`.
- Working DEM and the cache used by the current real-terrain benchmark.

All stage-specific reports, duplicated planner audit JSON, old tuning tables,
and obsolete visualization/benchmark outputs are intentionally left to Git
history.
