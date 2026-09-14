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

### Search

The production search module provides A* and weighted A*. CandidateZ plus
AircraftProfile is the current altitude-successor path. Search budgets report
explicit termination reasons so performance limits cannot be confused with
proof of unreachability.

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
