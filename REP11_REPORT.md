# Step REP-1.1 — Remove Regular Z-Lattice Dependency + Remove Web/UI Layer

Follows up on Step REP-1 (PARTIAL). Producing scripts: `scripts/
rep11_analysis.py`, `scripts/rep11_decision.py`. Validation: `scripts/
validate_rep11.py`. Code change: `planner/candidate_z.py` (`is_
representable()` simplified). Removed: `webapp/`, `.claude/launch.json`.
Raw artifacts: `results/rep11_*.json`.

**Short answer up front:** web/UI removed — yes. Old Z authority → new Z
authority: the regular `z_step_m` lattice remains the production Z
**value source**; `CandidateZGenerator.is_representable()` is now a
genuinely **continuous**, lattice-agnostic **gate** on those values (a
real, verified simplification), not yet their generator.
`CandidateZ.generate()` production-used: still no. `z_step` production
representation removed: no. Fixed-step primitive assumption removed: no.
Altitude state identity: unchanged (int `z_index`), verified stable.
Exact mission altitude: representable at the *query* level (yes), not
yet at the *state* level (no — same gap as REP-1, now precisely
located). Lazy/sparse genuinely active: yes. Hidden second Z authority:
`planner/coarse_astar.py`'s own separate lattice — pre-existing,
disclosed, not new. Missions A/B/C: bit-identical to REP-1, zero
regression. Heading: not added.

**This stage is PARTIAL, and per its own explicit failure policy,
READY FOR HEADING-1 = NO.**

## 1. REP-1 problem recap

REP-1 wired `CandidateZGenerator.is_representable()` into `planner/
astar.py`'s `_generate_neighbors`, closing the *query-level* exact-
mission-altitude gap, but left the regular `z_step_m` lattice as the
actual production Z value source — classified PARTIAL for that reason.
This stage was asked to close that gap entirely, and to remove the
web/UI layer.

## 2. Web removal

`webapp/server.py`, `webapp/static/index.html`, and `.claude/
launch.json` (its sole entry: `uvicorn webapp.server:app`, now dangling)
removed. Dependency audit (`results/rep11_web_removal_audit.json`):
webapp's own imports were confirmed, in Step REP-1's own audit, to be
ordinary `planner/` modules used independently elsewhere — nothing
web-specific existed inside `planner/`, so **zero planner code was
removed** as a consequence. No dependency manifest (`requirements.txt`
etc.) exists in this repo to clean up. Verdict: **WEB/UI LAYER =
REMOVED.**

## 3. Z dependency graph

`results/rep11_z_dependency_graph.json` — 11 nodes, 9 edges. Extends
Step REP-1's audit with explicit graph structure. Confirms `planner/
coarse_astar.py` remains a **second, independent, pre-existing** dense
lattice with zero `CandidateZGenerator` involvement (unchanged this
stage, disclosed per Section 20's explicit allowance rather than
silently left as a surprise).

## 4. CandidateZ production integration

`results/rep11_candidatez_production_audit.json`. `is_representable()`
is still the live gate in `_generate_neighbors` (unchanged call site from
REP-1); its own **rule** changed:

```
# REP-1
floor_for(row,col) <= z_msl <= ceiling  AND  z_msl on the z_step_m ladder (anchored at MSL 0)
# REP-1.1
floor_for(row,col) <= z_msl <= ceiling   (continuous -- no ladder requirement)
```

The ladder-alignment clause is removed because it was never a real
CandidateZ constraint — it was an *emergent property* of `planner/
primitives.py`'s own fixed `±z_step_m` arithmetic (every value that
arithmetic produces is on-ladder by construction), not something `is_
representable()` needed to separately enforce. Removing it makes the
predicate genuinely representation-neutral: it says nothing about *how*
a candidate altitude was computed, only whether it clears real terrain
through the mission ceiling. A future primitive computing an off-ladder
altitude would need **zero further change here**.

**What did NOT change**: the successor altitude *value* itself is still
computed by `_generate_neighbors`'s own `z0 + k*z_step_m` arithmetic.
`is_representable()` gates that value; it does not produce it.
`CandidateZGenerator.generate()` remains uncalled by any production code
path (re-verified by direct source inspection this stage, not just
trusted from REP-1's record).

## 5. Primitive fixed-step removal — NOT achieved, why

`planner/primitives.py`'s `dz = sign * config.z_step_m` (the only `dz_m`
assignment site, confirmed unchanged) was **not** redesigned this stage.
Two paths existed to fully remove it, both rejected as unsafe for this
stage's own constraints:

- **(a) Redesign primitive endpoint geometry** so climb/descent targets a
  CandidateZ-derived altitude instead of a fixed delta — explicitly
  forbidden (Section 8: no aircraft-aware primitive writing this stage;
  the real path-length/duration-aware geometry belongs to the HEADING/
  PRIMITIVE stage).
- **(b) Make `CanonicalState`'s z-component float-valued** (or a tagged
  union of lattice-index/exact-event) so an off-lattice value can be a
  *state*, not just a query answer — requires threading a per-mission
  altitude reference through `state_to_xyz`/`msl_to_z_index`/
  `z_index_to_msl` and their ~16 internal call sites in `planner/
  astar.py` alone (plus every external script calling these directly),
  with real risk of subtly breaking A*'s g-score/closed-set exactness
  if any site is missed or a reference is threaded inconsistently.
  Judged an unsafe, wide-blast-radius change to attempt without a
  dedicated redesign stage and much heavier re-verification than this
  stage's time allows.

Both are the same underlying issue REP-1 already identified, now
precisely narrowed to exactly these two options — neither is available
under this stage's own stated constraints (no primitive redesign, no
correctness-risking float-state-identity change).

## 6. Altitude state identity

`results/rep11_state_identity_tests.json`, `scripts/rep11_analysis.py`'s
`state_identity_tests()` — **6/6 PASS**: deterministic equality,
deterministic hashing, **zero floating drift over 1000 accumulated
steps** (int arithmetic under the hood), duplicate-candidate collapse,
deterministic ordering, and — the honest one — an off-lattice altitude
still **fails loud** (`ValueError`, not a silent wrong snap) rather than
being silently misrepresented. `z_index` (int) remains the identity;
verified stable, not merely assumed.

## 7. z_step / z_index cleanup

**Not removed.** `results/rep11_dead_code_cleanup.json`: `config.z_step_m`
remains genuinely required (primitives.py's `dz_m`, both search layers'
lattice arithmetic, `floor_for()`'s own formula). `CandidateZGenerator.
generate()`/`MotionContext`/`GeneratorStats` remain, unchanged from
REP-1's decision (historical `scripts/step3b`/`step3c` replay
dependency). No new dead code found beyond `webapp/` itself.

## 8. Mission altitude

`results/rep11_state_identity_tests.json` tests B/C: off-lattice start
(3251.37m)/goal (3583.91m) both `is_representable() == True` at their own
cells — **query-level solved, unchanged from REP-1, re-verified**. Test
`off_lattice_altitude_fails_loud_not_silently_wrong`: confirms the
**state-level** gap remains, honestly, not silently.

## 9. Terrain floor

Unchanged; `floor_for()`'s formula verified byte-identical via source
inspection (`validate_rep11.py` #5).

## 10. Lazy/deterministic behavior

Tests E, F (representability) and the state-identity suite all PASS —
no order-dependence, no accumulating state, no history parameter on
either `is_representable()` or `floor_for()` (test J, confirmed via
function-signature inspection, not just reading the docstring).

## 11. Sparse before/after metrics

`results/rep11_before_after_sparse_metrics.json` — reuses this stage's
own regression (below). `evaluate_primitive` calls avoided: **2,259 /
6,543 / 27,868** for Missions A/B/C — identical counts to REP-1, exactly
as predicted (no real-terrain candidate in these missions is ever
affected by the ladder-alignment clause's removal, since all of them are
arithmetic-derived and therefore always on-ladder regardless). Runtime
speedup not claimed (wall-clock noise dominates at this scale — see
disclosure in the artifact itself).

## 12. Real regressions (Missions A/B/C)

`results/rep11_real_regression.json`:

| Mission | expanded | cost | path_len | safe | min_AGL | raw_dem_reads |
|---|---:|---:|---:|---|---:|---:|
| A_easy_open | 461 | 3179.61 | 20 | PASS | 101.7m | 0 |
| B_relief_affected | 1635 | 4972.03 | 25 | PASS | 100.1m | 0 |
| C_z_matters | 22570 | 6750.06 | 31 | PASS | 100.0m | 0 |

**Bit-identical** to Step REP-1's own recorded numbers. No crash, no
safety regression.

## 13. Dead-code cleanup

`results/rep11_dead_code_cleanup.json` — `webapp/` fully removed; every
other REP-1 keep-decision reconfirmed with the same reasoning (historical
script replay-ability, genuine multi-role `z_step_m` requirement,
disclosed `coarse_astar.py` gap).

## 14. Final architecture audit

`results/rep11_final_architecture.json`, answering Section 27's 9
questions directly:

1. `CandidateZ.generate()` production-used? **No.**
2. `CandidateZ` single Z source-of-truth? **No.**
3. Regular `z_step` production representation remains? **Yes.**
4. `z_index` authority remains? **Yes.**
5. Fixed-step primitive assumption remains? **Yes.**
6. Web/UI fully removed? **Yes.**
7. Second hidden Z representation? `planner/coarse_astar.py`'s own —
   pre-existing, disclosed, genuinely different semantic, not new.
8. Exact mission altitude works? **Query-level yes, state-level no**
   (narrower than REP-1's framing, same underlying gap).
9. Lazy instantiation works? **Yes.**

## 15. Tests

`scripts/validate_rep11.py` — 9 groups, **ALL PASS**: web genuinely
removed (disk + git); `is_representable()` has no lattice-alignment code
left; `.generate()` re-confirmed unused via live source re-check (not
trusted from memory); 10 representability + 6 state-identity tests all
recorded and passing; terrain floor unchanged; Missions A/B/C safe; dead-
code/final-architecture decisions recorded with real reasons; no search/
primitive/corridor/guidance file touched; **the decision artifact itself
honestly reports `ready_for_heading_1: false`** — checked directly, not
assumed.

## 16. Blockers (unchanged in kind, narrower in scope than REP-1)

Same root cause as Step REP-1, now precisely reduced to exactly two
concrete, named implementation paths (§5), both correctly out of this
stage's safe scope. Nothing new was silently deferred — every listed
blocker was actively investigated and rejected for a stated, specific
reason, not left unexamined.

## Next stage

**Not HEADING-1.** Per this stage's own explicit failure policy, HEADING-1
does not start until the Z-authority question is genuinely closed. The
two concrete paths in §5 (primitive endpoint redesign, or a per-mission
altitude-reference threading through `planner/astar.py`'s conversion
functions) are candidates for a dedicated future stage; which one (or
whether the aircraft-aware primitive stage naturally resolves this as a
side effect) is a decision for the user, not assumed here.
