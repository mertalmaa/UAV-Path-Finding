# Step REP-1 — Production Sparse CandidateZ Integration + Legacy Z-Step Cleanup Audit

Closes the production mismatch Step CLASS-C's audit found: a real sparse
CandidateZ infrastructure existed but production search never actually
routed representability decisions through it beyond a narrow floor-only
prefilter. Producing scripts: `scripts/rep1_analysis.py`,
`scripts/rep1_decision.py`. Validation: `scripts/validate_rep1.py`.
Code change: `planner/candidate_z.py` (+`is_representable()`),
`planner/astar.py` (`_generate_neighbors` wiring). Raw artifacts:
`results/rep1_*.json`.

**This is not a Z-step re-gate.** No 10/20/40m comparison, no new
default z_step chosen.

## Central finding — why this stage is PASS, not the "full replacement" originally implied

`CandidateZGenerator.generate()`'s own return value is a **2-4 element
boundary-event set** (terrain floor, mission ceiling, exact start/goal).
Every interior `z_step_m`-ladder rung strictly between floor and
ceiling — which is what real multi-hop paths actually traverse, one
`±z_step_m` primitive hop at a time (`planner/primitives.py`: `dz = sign
* config.z_step_m`, confirmed the only assignment site) — is **never** a
member of that set. Using `generate()`'s literal output as a successor
membership test would reject nearly every legitimate, safe multi-hop
climb/descent. The regular `z_step_m` lattice is therefore a genuinely
necessary part of the search **state space** (`planner/astar.py`:
`CanonicalState = (row, col, z_index)`, `z_index_to_msl(z) = z_index *
z_step_m`) — not an obsolete layer sitting in front of a sparse
alternative that could simply replace it.

What **did** close, and closes it for real: a new, unified
`CandidateZGenerator.is_representable(row, col, z_msl)` predicate
(`planner/candidate_z.py`), built from the *same* existing, correct
infrastructure (`floor_for()`, `MissionContext`) — not a new candidate
system — now **is** the live production representability authority in
`planner/astar.py`'s `_generate_neighbors`, replacing the narrower
floor-only inline check. It additionally, correctly recognizes an
off-lattice mission start/goal altitude as representable **at its own
specific cell** — closing the exact-mission-altitude gap Step CLASS-C
left open, for the one case where it's actually reachable under the
current int-typed `z_index`.

## 1. z_step/z_index usage audit

Full results: `results/rep1_zstep_usage_audit.json`.

- **[A] Legacy dense/regular representation** (genuinely defines search
  state, still required): `planner/astar.py`'s `msl_to_z_index`/
  `z_index_to_msl`/`state_to_xyz`/`_generate_neighbors`; `planner/
  coarse_astar.py`'s parallel dense-lattice coarse search (never imports
  `CandidateZGenerator` at all, before or after this stage);
  `webapp/server.py` (the one live interactive tool — confirmed it never
  constructs or passes a `CandidateZGenerator` anywhere, before or after
  this stage — see Blockers).
- **[B] Still required utility**: `planner/primitives.py`'s
  `dz = sign * config.z_step_m`; `CandidateZGenerator.floor_for()`
  (now also the internal building block of `is_representable()`);
  `state_to_xyz`/`path_to_xyz` (display/geometry conversion only).
- **[C] Test/historical only**: every remaining hit — all in `scripts/`,
  none in `planner/`/`webapp/`.
- **[D] Unknown**: none — every site was confidently classified.

## 2. CandidateZ.generate() audit

Full results: `results/rep1_before_after_candidatez.json`. Candidate
families (unchanged from Step 3A/3B): CLASS A (terrain floor + ceiling),
CLASS B (exact mission start/goal), CLASS C (motion events — always
empty, Step 3A.1 status unchanged). Deterministic, MSL float units,
ascending-sorted, exact-dedup, terrain-floor-bounded. **Production usage
verdict: `generate()` itself remains unused in production after this
stage** — not because it's broken, but because its own event-set shape
is the wrong tool for a membership test (see Central Finding).

## 3. Production integration

`planner/astar.py`'s `_generate_neighbors`:

```
# before
floor = candidate_z_generator.floor_for(new_row, new_col)
if floor is None or new_z_msl < floor - 1e-9:
    reject("below_terrain_floor_sparse")

# after (Step REP-1)
if not candidate_z_generator.is_representable(new_row, new_col, new_z_msl):
    reject("below_terrain_floor_sparse")   # name kept for backward compatibility
```

`is_representable()` is strictly more complete than the old check (adds
exact mission-event recognition) while being **provably safety-neutral**:
it can only reject a candidate `evaluate_primitive()` would also reject
(same terrain+min_agl formula for its CLASS-A branch, exactly as
`floor_for()` always was), never accept one `evaluate_primitive()`
would reject — that function is unchanged and remains the sole safety
authority for mid-primitive checks.

## 4. Legacy dense-Z representation cleanup

**Not removed — genuinely required.** `results/rep1_decision.json`'s
`blocker` field: `planner/primitives.py`'s fixed `±z_step_m` primitives
need a dense, exactly-hashable integer lattice for correct multi-hop
accumulation (a float-typed altitude built from repeated addition risks
floating-point drift silently corrupting A*'s g-score/closed-set
dictionaries — a real optimality-breaking risk, not a style concern).
`CandidateZGenerator` alone cannot substitute for it (Central Finding).
**This is the disclosed blocker driving the PARTIAL classification below**
— per this project's own stated policy: report it, don't silently keep
or silently claim removal either way.

## 5. Mission exact altitude

Section 16's representability tests A-H, against real terrain (same
window Step 3D/3E/GRID-1/CLASS-C already used, an intentionally
off-lattice synthetic mission: start=3251.37m, goal=3583.91m — neither a
multiple of 20): **8/8 PASS** (`results/rep1_mission_altitude_tests.json`).
B and C directly confirm an off-lattice start/goal altitude is
`is_representable() == True` at its own cell — the exact behavior Step
CLASS-C flagged as missing.

## 6. Terrain floor

Unchanged: `floor_for()`'s `ceil((elevation+min_agl)/z_step)*z_step`
formula, still the sole CLASS-A source inside `is_representable()`.
Verified via source re-check (`validate_rep1.py` #4).

## 7. Deterministic / lazy behavior

Test E (repeated calls, identical inputs) and F (100-cell query set,
forward vs. shuffled order, independent generator instances) both PASS —
`is_representable()` has no order-dependent or accumulating state.
`is_representable()` itself contains no enumeration loop (verified by
source inspection) — a pure O(1) membership predicate, preserving lazy
instantiation exactly as `floor_for()` always did.

## 8. Sparse representation metrics

`results/rep1_sparse_representation_metrics.json`, reusing Step 3E/
CLEAN-1's own legacy-vs-sparse_lazy comparison (now under
`is_representable()`): `evaluate_primitive()` calls avoided —
**2,259 / 6,543 / 27,868** for Missions A/B/C respectively (identical to
the pre-REP-1 numbers, since none of these missions exercise the new
mission-event branch). Expanded-node count is **identical before/after**
in every mission (the prefilter never changes which path is found — the
one invariant this whole mechanism depends on). Wall-time deltas were
measured but are NOT claimed as a general speedup result (mixed
sign, dominated by machine-load noise at this scale) — see the
artifact's own `runtime_speedup_claim` field for the honest disclosure.

## 9. Real regression (Missions A/B/C)

`results/rep1_real_regression.json`. Rerun of `scripts/
step3e_production_representation_integration.py` AFTER the
`is_representable()` wiring:

| Mission | expanded | cost | path_len | safe | min_AGL | goal_err | raw_dem_reads |
|---|---:|---:|---:|---|---:|---:|---:|
| A_easy_open | 461 | 3179.61 | 20 | PASS | 101.7m | 0.00m | 0 |
| B_relief_affected | 1635 | 4972.03 | 25 | PASS | 100.1m | 0.00m | 0 |
| C_z_matters | 22570 | 6750.06 | 31 | PASS | 100.0m | 0.00m | 0 |

**Bit-identical** to every number recorded before this stage's changes
(Step CLEAN-1/GRID-1/CLASS-C). No crash. Zero regression.

## 10. Dead-code / config cleanup

`results/rep1_dead_code_cleanup.json`:

- **`CandidateZGenerator.generate()`/`MotionContext`/`GeneratorStats`**:
  NOT removed. Not production-used (confirmed), but `scripts/
  step3b_sparse_lazy_z_prototype.py` and `scripts/
  step3c_persistent_terrain_cache.py` (both historical, already-passed
  validation scripts) import these names directly from `planner/
  candidate_z.py` and would break if removed. Kept runnable, per the
  standing project rule that historical scripts stay replayable — not
  "belki lazım olur" hoarding.
- **`config.z_step_m`**: NOT removed, genuinely multi-role-required (see
  §4). Not renamed either — the name still accurately describes its role.
- **`webapp/server.py` never used `CandidateZGenerator`**: disclosed as a
  pre-existing gap, not fixed this stage (see Blockers).
- **`planner/coarse_astar.py` never imported `CandidateZGenerator`**:
  disclosed, not fixed — a second search implementation was out of this
  stage's explicit scope.
- No other dead code, stale imports, or orphaned config fields found.

## 11. Before / after architecture

**BEFORE**: production Z source-of-truth = the plain `z_step_m` lattice;
`CandidateZGenerator` consulted only via `floor_for()`, a narrow
terrain-only prefilter; off-lattice mission altitudes had no
representability path at all.

**AFTER**: production Z source-of-truth is **still** the plain
`z_step_m` lattice (disclosed, justified, not silently kept) — but
`CandidateZGenerator.is_representable()` is now the actual, complete
representability AUTHORITY gating every successor in `_generate_neighbors`
(terrain floor + ceiling + exact mission event), a real and verified
upgrade over the narrower pre-REP-1 check.

- `CandidateZ.generate()` production-used? **NO** (unchanged from before this stage).
- Regular dense `z_step` production-used? **YES** (necessarily — see §4).
- Obsolete `z_step` config removed? **STILL_REQUIRED_WITH_REASON** (§4/§10).
- Dead z-index helpers removed? **STILL_REQUIRED_WITH_REASON** (`generate()`/`MotionContext`, §10).

## 12. Tests

`scripts/validate_rep1.py` — 10 groups, **ALL PASS**: `is_representable()`
genuinely wired into `planner/astar.py` (old floor-only branch gone); 8/8
representability tests recorded and passing; off-lattice mission altitude
representable (live re-check); terrain floor formula unchanged;
deterministic (repeat + shuffled-order); arbitrary/below-floor altitudes
correctly rejected; no enumeration loop added; dead-code decisions
recorded with real reasons; Missions A/B/C regression bit-identical; no
search-algorithm/heading/primitive/corridor/guidance file touched
(`planner/primitives.py`, `corridor.py`, `coarse_astar.py`,
`fine_precompute.py`, `mission.py`, `webapp/server.py`,
`terrain_cache.py`, `aircraft_profile.py`, `vertical_motion.py` all
confirmed untouched; only `planner/astar.py` and `planner/candidate_z.py`
changed, as expected).

## Blockers (disclosed, per project policy — not silently resolved either way)

1. **The regular `z_step_m` lattice cannot be removed.** `planner/
   primitives.py`'s fixed-step primitives require it; `CandidateZGenerator`
   alone (any literal reading of `generate()`) cannot substitute — see
   Central Finding and §4. This is why this stage is **PARTIAL**, not
   PASS, on the literal "obsolete regular Z representation removed"
   criterion, even though real, verified integration progress was made
   everywhere else.
2. **`webapp/server.py` never used `CandidateZGenerator`**, before or
   after this stage — the live interactive tool's fine/coarse searches
   run with zero representability prefilter. Wiring it in is safety-
   neutral (proven by Step 3E/CLEAN-1's own regression) but requires
   building/attaching a persistent-cache-backed store per request, not
   exercised by this stage's regression harness — left as a follow-up
   rather than an under-tested change to the one live tool.
3. **`planner/coarse_astar.py` never imported `CandidateZGenerator`** —
   a second, parallel search implementation, out of this stage's
   explicit scope (Section 21 froze coarse search's own logic).

## Next stage

**HEADING-1 — Heading Discretization Design**, combining: 60m XY default
(GRID-1), the CandidateZ representability contract (this stage, now
including off-lattice mission events), V3 turn radius/rate (GRID-1/
ALG-1), and the CLASS-C physical vertical-motion contract. The
lattice-vs-sparse question this stage leaves PARTIAL is orthogonal to
heading discretization (a future heading-aware primitive defines its own
geometry-based Z transitions via CLASS-C's `evaluate_vertical_motion`)
and does not block starting that design.
