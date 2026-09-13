# Step CLASS-C — CandidateZ + Physical Vertical-Motion Bridge Closure

Closes the missing link between Z representation (`planner/candidate_z.py`)
and real aircraft vertical capability (`planner/aircraft_profile.py`, Step
ALG-1's V3 artifact). Producing script: `scripts/classc_analysis.py`.
Validation: `scripts/validate_classc.py`. New production module:
`planner/vertical_motion.py`. Raw artifacts: `results/classc_*.json`.

**Not a Z-step re-gate.** No 10/20/40m comparison was performed and no
new final Z-step was chosen — see §1 for why the question turned out to
be different from what a re-gate would answer.

## 1. Current CandidateZ audit — the central finding

Before writing any code, `planner/candidate_z.py` and its actual call
sites in `planner/astar.py` were read in full. The finding changes the
shape of this whole stage:

**Production search's Z state is still the plain regular `z_step_m`
lattice** — `z_index_to_msl(z_index) = z_index * config.z_step_m`
(`planner/astar.py`). Every primitive's `dz_m` is fixed at exactly
`±config.z_step_m` (or 0) — confirmed in `planner/primitives.py`
(`dz = sign * config.z_step_m`) — regardless of search history.

`CandidateZGenerator.floor_for(row, col)` is consulted by production
(`_generate_neighbors`, Step 3E) **only as an efficiency prefilter** on
top of that lattice: it rejects a lattice-derived candidate altitude that
falls below the terrain+min_agl floor, before the costlier
`evaluate_primitive()` call. It never generates a new candidate altitude.

`CandidateZGenerator.generate()` — the actual CLASS A/B/C sparse
candidate **set** Step 3A/3B designed (terrain floor + mission exact
altitude + motion events) — **is not called anywhere in production**.
Its only caller in the entire repository is `scripts/
step3b_sparse_lazy_z_prototype.py`, an isolated unit-test script.
`MotionContext.provisional_events` always returns an empty set and,
because `generate()` is never called, is never even consulted.

**Consequence:** nothing in the codebase, before this stage, ever
compared a Z transition's implied vertical rate against real aircraft
capability. `evaluate_primitive()` checks terrain/AGL/climb-angle only.

Full structured answers to all 10 audit questions:
`results/classc_candidate_z_audit.json`.

## 2. Representation contract

Three concepts, kept formally separate (`results/
classc_representation_contract.json`):

- **Representability** — which altitudes can exist as planner state at
  all. Entirely `planner/candidate_z.py` + `planner/astar.py`'s concern.
  This stage's new code has no opinion on it.
- **Physical reachability** — whether the aircraft can actually fly from
  one representable altitude to another within a given motion duration.
  Answered by the new `evaluate_vertical_motion()`.
- **Instantiation** — whether a representable state was actually created
  during a particular search run. Entirely `planner/astar.py`'s concern
  (open/closed sets); invisible to the new module.

**Representable endpoint != feasible edge**, closed: a candidate altitude
existing never implies the aircraft can fly there in the time/distance a
primitive allows — that gap is exactly `evaluate_vertical_motion()`.

## 3. V3 aircraft vertical capability (live queries, no hard-coding)

7 altitudes × {CLIMB, DESCENT} = 14 live `AircraftProfile.vertical_query()`
calls (`results/classc_aircraft_vertical_queries.json`):

| Altitude | Climb | Descent |
|---:|---:|---:|
| 500 m | AVAILABLE, 4.21 m/s | AVAILABLE, -3.23 m/s |
| 1500 m | AVAILABLE, 4.25 m/s | AVAILABLE, -3.23 m/s |
| 2500 m | AVAILABLE, 3.19 m/s | AVAILABLE, -3.23 m/s |
| 3500 m | AVAILABLE, 2.15 m/s | AVAILABLE, -4.23 m/s |
| 4500 m | AVAILABLE, 2.16 m/s | AVAILABLE, -4.25 m/s |
| 5000 m | **UNAVAILABLE** | AVAILABLE, -4.25 m/s |
| 5500 m | **UNAVAILABLE** | AVAILABLE, -3.23 m/s |

Matches Step ALG-1's own recorded envelope exactly (5000/5500m climb
UNAVAILABLE, descent's non-monotonic -3/-4/-3 family progression) — this
stage re-confirms it via `vertical_motion.py`'s own call path, not a
separate hard-coded copy.

## 4. Aircraft/motion-event decision (section 8: A or B)

**Decision: A.** Aircraft/motion information does **not** add new
CandidateZ altitude candidates — it only determines edge feasibility
between two already-representable endpoints. This is not an arbitrary
preference; it is the only decision consistent with the audited reality
in §1 (CLASS C motion events are already unused/empty in production) and
it keeps CandidateZ search-history-independent, per the standing rule.

## 5. Physical transition contract

`planner/vertical_motion.py`, `evaluate_vertical_motion(source_altitude_m,
target_altitude_m, motion_duration_s, aircraft_profile)`:

```
required_vz = (target_altitude_m - source_altitude_m) / motion_duration_s
CLIMB:   FEASIBLE iff required_vz <= local safe climb capability
DESCENT: FEASIBLE iff |required_vz| <= |local safe descent capability|
```

Queried at `source_altitude_m` (a documented policy choice: "can this
aircraft begin this climb/descent from where it currently is"). Reads
`planner_safe.{climb,descent}_vz_mps` — the V3 schema's own naming
convention, never a hard-coded number; raises `ValueError` (a genuine
schema-contract violation) if an AVAILABLE row is missing that key.
Returns one of `FEASIBLE` / `PHYSICALLY_UNAVAILABLE` /
`OUT_OF_PROFILE_DOMAIN` / `INVALID_DURATION`, plus `required_vz_mps`,
`safe_vz_mps`, `delta_z_m`, `duration_s`, `availability` for diagnostics.
Purely functional — no instance state, no history, referentially
transparent. **Not called from `planner/astar.py` or anywhere in
successor generation** — a contract for a future primitive to call, not
wired in this stage.

## 6. Motion-horizon diagnostics (real observed Δz, not assumed 20m)

`results/classc_motion_horizon_diagnostics.json`. Two sources, both real:

- **Single primitive hop**: always exactly `±z_step_m` (20m) today —
  confirmed structurally, not assumed.
- **`floor_for()`'s real terrain-driven floor deltas**, sampled over the
  same 20×20-coarse-cell window Step 3D/3E/GRID-1 already used: **-20m to
  +560m**, always exact multiples of 20m (floor_for always ceils to the
  z_step_m ladder). Representative sample used for the diagnostic: 180,
  240, 280, 320, 350, 380, 414, 440 m.

For each, minimum climb/descent duration and approximate horizontal
distance (at nominal IAS=40 m/s) needed to cover it safely, e.g.:

| Δz | Climb min duration / ~distance | Descent min duration / ~distance |
|---:|---:|---:|
| 20 m | 9.3 s / ~373 m | 4.7 s / ~190 m |
| 180 m | 83.9 s / ~3355 m | 42.7 s / ~1707 m |
| 440 m | 205.0 s / ~8202 m | 104.3 s / ~4172 m |

This is a pure physical-scale diagnostic, **not a grid rule** — it shows
that real terrain-driven floor jumps require horizontal distances far
beyond a single 60m (or even 30m) grid hop, reinforcing why a future
vertical-motion primitive needs its own duration/path-length reasoning
rather than assuming one hop = one z_step.

## 7. Mission exact altitude handling

`evaluate_vertical_motion()` itself is representation-agnostic — it takes
plain floats and works correctly for off-lattice altitude pairs (contract
test F: 3251.5m → 3253.0m, FEASIBLE). Whether an off-lattice mission
altitude can exist AS SEARCH STATE remains `planner/candidate_z.py` +
`planner/astar.py`'s separate concern — and today's default
`msl_to_z_index(allow_snap=False)` **raises** rather than silently
mis-snapping, which is at least safe, but leaves `generate()`'s CLASS B
(mission exact altitude as its own representable event) genuinely unused.
**Flagged as a follow-up, not fixed this stage** (no successor-generation
changes were made).

## 8. Terrain-floor handling

`planner/candidate_z.py` was **not modified**. `floor_for()`'s
`ceil((elevation+min_agl)/z_step)*z_step` formula — never optimistic
downward snapping — is untouched, verified via git status.

## 9. Deterministic / lazy behavior

Unchanged: `floor_for()` remains a pure function of `(row, col)` plus
store/mission contents. `evaluate_vertical_motion()` adds no history —
repeated calls with identical inputs return identical results (contract
test, verified).

## 10. Synthetic transition contract tests

`results/classc_transition_contract_tests.json` — 10 cases (A-I, plus the
G/H edge-status cases), **ALL PASS**: too-short duration rejected, ample
duration accepted, 5000/5500m climb rejected (UNAVAILABLE), in-capability
climb/descent accepted, over-capability descent rejected, off-lattice
altitude pair handled correctly, out-of-domain and zero-duration status
codes correct.

## 11. Final CLASS-C decision

| # | Question | Answer |
|---|---|---|
| 1 | Current CandidateZ representation | The plain `z_step_m` lattice IS production state; `generate()`'s sparse set is unused |
| 2 | Regular/base lattice role | Still present, still IS the state space |
| 3 | Aircraft motion changes CandidateZ or only edge feasibility | Only edge feasibility (decision A) |
| 4 | Search-history-independent motion events needed | No |
| 5 | Residual vertical-progress state needed | **No** — future primitives must span complete transitions instead |
| 6 | Representable vs. physically-reachable closed | Yes |
| 7 | Mission exact-altitude bridge policy clear | Yes (contract handles it; representability itself flagged as follow-up) |
| 8 | Terrain-floor conservative | Yes, unchanged |
| 9 | Heading-aware primitives can use the contract directly | Yes |

## 12. Tests

`scripts/validate_classc.py` — 12 groups, **ALL PASS**: artifacts exist;
audit re-verified against live source code (not just trusted); no literal
20m constant in the new module; no Z-step re-gate; 14/14 live V3
round-trip; altitude-dependent capability genuinely varies (5 distinct
climb values); representable≠reachable enforced; 5000/5500m climb
rejected; off-lattice determinism; `planner/candidate_z.py` untouched; no
search/primitive/corridor file touched; new module has no instance state;
V3 source's own `provenance_id` unchanged.

## Blockers / follow-ups (disclosed, not fixed this stage)

- **Mission exact-altitude representability**: `msl_to_z_index(allow_snap=
  False)` raises for off-lattice mission altitudes rather than routing
  through `CandidateZGenerator.generate()`'s CLASS B event. Safe (fails
  loud) but leaves the intended architecture partially wired.
- **`CandidateZGenerator.generate()` / CLASS A-C sparse candidate set**
  remains unused in production; only `floor_for()` is live. Not a bug —
  a scope note for whoever eventually revisits the state-space question.

## Next stage

**HEADING-1 — Heading Discretization Design**, now combining: 60m XY
default (GRID-1), the CandidateZ representation contract (this stage), V3
turn radius/rate (GRID-1/ALG-1), and this stage's physical vertical-motion
contract. Then: aircraft-aware motion primitives.
