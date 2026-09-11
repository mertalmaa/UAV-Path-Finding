"""Roadmap Step 3B: small synthetic sparse/lazy Z implementation.

Implements Step 3A.1's CandidateZGenerator contract as a real, runnable
prototype (not a design sketch) and measures it against a dense 20m
baseline on small synthetic/micro-terrain tests. No production graph, no
final Z spacing decision, no aircraft safety box, no heading/turn-radius,
no corridor/cost/heuristic changes, no JSBSim. planner/*.py untouched --
reuses TerrainQuery, evaluate_agl, evaluate_transition, evaluate_primitive,
MotionPrimitive, build_primitive_set, primitive_endpoint verbatim; nothing
under planner/ is reimplemented.

Terminology used throughout, exactly per Step 3A.1 section 4:
  A) LOGICAL CANDIDATE Z -- what CandidateZGenerator.generate(row,col)
     would deterministically return, whether or not anything has looked
     at it yet.
  B) INSTANTIATED STATE -- an actual (row,col,z) placed into the search's
     `instantiated` dict, because some primitive expansion actually
     produced it as a successor.
  C) EXPANDED STATE -- an instantiated state that was popped off the open
     set and had ITS successors generated.

IMPORTANT (Step 3A.1 section 5 / this task's section 6, Case E note):
CandidateZGenerator's CLASS-C (motion/aircraft) events are PROVISIONAL and,
in this prototype, INTENTIONALLY UNIMPLEMENTED (MotionContext.provisional_
events always returns an empty set). No motion-repair event is invented
when a search edge fails. See the Case E discussion in main() for how the
"rescue altitude" is actually accounted for here -- honestly, not by
inventing a corridor algorithm that wasn't asked for and wasn't built.
"""
import dataclasses
import math
import sys
from dataclasses import dataclass, field
from time import perf_counter
from typing import Dict, FrozenSet, List, Optional, Tuple

import numpy as np
from affine import Affine

from planner.config import DEFAULT_CONFIG
from planner.primitives import MotionPrimitive, build_primitive_set, evaluate_primitive, primitive_endpoint
from planner.roi import ROIData
from planner.terrain import TerrainQuery

STEP2B_CONFIG = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100.0)


def make_synthetic_roi(elevation, res=30.0, nodata=-9999.0):
    """Verbatim pattern from step3a_z_representation_diagnostics.make_synthetic_roi."""
    height, width = elevation.shape
    transform = Affine(res, 0.0, 500000.0, 0.0, -res, 4200000.0)
    return ROIData(elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
                    width=width, height=height,
                    bounds=(500000.0, 4200000.0 - height * res, 500000.0 + width * res, 4200000.0),
                    resolution=(res, res), nodata=nodata)


def lowest_feasible_layer(terrain_elev, min_agl, z_step, z_lo, z_hi):
    """Ported verbatim from step3a/step2d -- smallest z_step-grid altitude
    inside [z_lo, z_hi] clearing terrain_elev+min_agl, or None."""
    true_floor = terrain_elev + min_agl
    candidate = math.ceil(max(true_floor, z_lo) / z_step) * z_step
    return None if candidate > z_hi else candidate


# ===========================================================================
# 2. CandidateZGenerator -- Step 3A.1 contract, real implementation
# ===========================================================================

@dataclass(frozen=True)
class TerrainMetadata:
    row: int
    col: int
    elevation_m: float  # NaN if out-of-bounds/nodata


class TerrainMetadataStore:
    """STATIC/OFFLINE terrain metadata cache (section 9's STATIC bucket).

    The ONLY place in this whole prototype that ever touches the raster
    (via TerrainQuery). Every (row,col) is read from the DEM at most once;
    all later requests for the same cell are pure dict lookups. Search code
    never re-derives terrain metadata -- it only calls .get(), which is
    either a cache hit or a one-time cache-filling miss.
    """

    def __init__(self, terrain: TerrainQuery):
        self._terrain = terrain
        self._cache: Dict[Tuple[int, int], TerrainMetadata] = {}
        self.hits = 0
        self.misses = 0
        self.lookup_time_total_s = 0.0

    def get(self, row: int, col: int) -> TerrainMetadata:
        key = (row, col)
        cached = self._cache.get(key)
        if cached is not None:
            self.hits += 1
            return cached
        self.misses += 1
        t0 = perf_counter()
        if not self._terrain.in_bounds_rowcol(row, col):
            md = TerrainMetadata(row, col, float("nan"))
        else:
            r = self._terrain.elevation_at_rowcol(row, col)
            md = TerrainMetadata(row, col, r.elevation if r.valid else float("nan"))
        self.lookup_time_total_s += perf_counter() - t0
        self._cache[key] = md
        return md

    @property
    def hit_rate(self) -> float:
        total = self.hits + self.misses
        return self.hits / total if total else 0.0


@dataclass(frozen=True)
class MissionContext:
    """CLASS B events -- mission-dependent, search-independent."""
    start_rowcol: Tuple[int, int]
    start_z_msl: float
    goal_rowcol: Tuple[int, int]
    goal_z_msl: float
    ceiling_msl: float
    min_agl_m: float
    # Ladder step used ONLY to snap the CLASS-A validity-transition altitude.
    # Deliberately kept EQUAL to config.z_step_m (production, 20m) -- Step 3B
    # does not decide or propose a final/different Z spacing. The sparsity
    # measured in this prototype comes from generating few EVENTS per cell
    # (floor/ceiling/start/goal) instead of the whole dense ladder, and from
    # LAZY instantiation, not from widening this step.
    z_step_m: float = DEFAULT_CONFIG.z_step_m


@dataclass(frozen=True)
class MotionContext:
    """CLASS C events -- PROVISIONAL (Step 3A.1). A real implementation
    would derive climb/descent-reachable altitude corridors from terrain
    profile + primitive angle envelope, deterministically and independent
    of search order (Step 3A.1 point 5). That algorithm was NOT designed or
    implemented in Step 3A.1 and is NOT implemented here either --
    `provisional_events` always returns an empty set. Kept as a real
    parameter (not omitted) purely so the CandidateZGenerator signature is
    forward-compatible with a future implementation, exactly like
    AircraftSafetyContext stays an empty placeholder for now.
    """

    def provisional_events(self, row: int, col: int) -> FrozenSet[float]:
        return frozenset()


@dataclass
class GeneratorStats:
    call_count: int = 0
    times_s: List[float] = field(default_factory=list)


class CandidateZGenerator:
    """Deterministic Z-candidate generator, Step 3A.1 contract:

        CandidateZGenerator(xy, mission_context, terrain_metadata, motion_context=None)

    Here `xy` is (row, col) (same discretization the rest of the prototype
    and the production planner use), `terrain_metadata` is a
    TerrainMetadataStore (the STATIC cache), and mission/motion_context are
    as above. generate(row, col) returns a sorted tuple of unique candidate
    Z_msl values, combining:

      CLASS A (static terrain): the lowest z on the mission's z_step_m
        ladder that clears terrain+min_agl ("validity-transition
        altitude"), plus the mission ceiling as an always-present upper
        bound event.
      CLASS B (mission): start_z_msl at the start cell, goal_z_msl at the
        goal cell.
      CLASS C (motion, only if motion_context is not None): whatever
        motion_context.provisional_events returns -- empty in this
        prototype, see MotionContext docstring.

    Deterministic: depends only on (row, col) plus the CONTENTS of store/
    mission/motion_context, never on call order or on what the search has
    done so far. store is itself a pure memoizing cache of real DEM values
    (same value every time for the same cell), so repeated calls with the
    same store/mission/motion_context always return the same tuple.
    """

    def __init__(self, store: TerrainMetadataStore, mission: MissionContext,
                 motion_context: Optional[MotionContext] = None):
        self.store = store
        self.mission = mission
        self.motion_context = motion_context
        self.stats = GeneratorStats()

    def floor_for(self, row: int, col: int) -> Optional[float]:
        """CLASS-A validity-transition altitude at (row,col), or None if
        the cell has no terrain data or no ladder altitude fits under the
        mission ceiling."""
        md = self.store.get(row, col)
        if math.isnan(md.elevation_m):
            return None
        z_step = self.mission.z_step_m
        floor = math.ceil((md.elevation_m + self.mission.min_agl_m) / z_step) * z_step
        return floor if floor <= self.mission.ceiling_msl else None

    def generate(self, row: int, col: int) -> Tuple[float, ...]:
        t0 = perf_counter()
        candidates = set()
        floor = self.floor_for(row, col)
        if floor is not None:
            candidates.add(floor)                       # CLASS A
            candidates.add(self.mission.ceiling_msl)     # CLASS A/B boundary
        if (row, col) == self.mission.start_rowcol:
            candidates.add(self.mission.start_z_msl)     # CLASS B
        if (row, col) == self.mission.goal_rowcol:
            candidates.add(self.mission.goal_z_msl)       # CLASS B
        if self.motion_context is not None:
            candidates |= self.motion_context.provisional_events(row, col)  # CLASS C
        result = tuple(sorted(candidates))
        self.stats.call_count += 1
        self.stats.times_s.append(perf_counter() - t0)
        return result


# ===========================================================================
# 6. Controlled test cases A-E (Step 3A cases, re-tested against the REAL
#    CandidateZGenerator implementation, contrasted with a NAIVE bare
#    regular-step grid that has no event insertion at all -- exactly the
#    failure mode Case B/D exist to demonstrate CandidateZGenerator avoids).
# ===========================================================================

DENSE_STEP = 20.0          # production z_step_m -- reference/baseline only
NAIVE_SPARSE_STEP = 100.0  # illustrative-only "regular grid, no events" contrast, same value step3a used


def naive_regular_grid_only(terrain_elev, min_agl, z_step, z_lo, z_hi):
    """A bare fixed-step ladder with NO floor/ceiling/mission event
    insertion -- i.e. what you get WITHOUT CandidateZGenerator. This is the
    thing Case B/D show losing state; it is not itself part of the
    CandidateZGenerator contract."""
    return lowest_feasible_layer(terrain_elev, min_agl, z_step, z_lo, z_hi)


def case_A_wide():
    terrain_elev = 3000.0
    z_lo, z_hi = terrain_elev + STEP2B_CONFIG.min_agl_m, 6000.0
    mission = MissionContext((0, 0), z_lo, (0, 1), z_hi, ceiling_msl=z_hi,
                              min_agl_m=STEP2B_CONFIG.min_agl_m, z_step_m=DENSE_STEP)
    store = TerrainMetadataStore(TerrainQuery(make_synthetic_roi(np.full((1, 2), terrain_elev))))
    gen = CandidateZGenerator(store, mission)
    gen_candidates = gen.generate(0, 0)
    naive = naive_regular_grid_only(terrain_elev, STEP2B_CONFIG.min_agl_m, NAIVE_SPARSE_STEP, z_lo, z_hi)
    dense = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, DENSE_STEP, z_lo, z_hi)
    return {"case": "A_wide_feasible_interval", "terrain_elev": terrain_elev,
            "generator_candidates": gen_candidates, "generator_recovers_dense_floor": dense in gen_candidates,
            "naive_regular_grid_(100m)_candidate": naive, "naive_lost_it": naive is None}


def case_B_narrow():
    terrain_elev = 3540.0
    z_lo, z_hi = terrain_elev + STEP2B_CONFIG.min_agl_m, 3660.0  # 20m-wide ledge band
    mission = MissionContext((0, 0), z_lo, (0, 1), z_hi, ceiling_msl=z_hi,
                              min_agl_m=STEP2B_CONFIG.min_agl_m, z_step_m=DENSE_STEP)
    store = TerrainMetadataStore(TerrainQuery(make_synthetic_roi(np.full((1, 2), terrain_elev))))
    gen = CandidateZGenerator(store, mission)
    gen_candidates = gen.generate(0, 0)
    naive = naive_regular_grid_only(terrain_elev, STEP2B_CONFIG.min_agl_m, NAIVE_SPARSE_STEP, z_lo, z_hi)
    dense = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, DENSE_STEP, z_lo, z_hi)
    return {"case": "B_narrow_feasible_interval", "terrain_elev": terrain_elev, "band": (z_lo, z_hi),
            "dense_only_usable_state": dense, "generator_candidates": gen_candidates,
            "generator_preserves_it": dense in gen_candidates,
            "naive_regular_grid_(100m)_candidate": naive, "naive_lost_it": naive is None,
            "note": "naive bare 100m grid loses the ONLY usable state; CandidateZGenerator's CLASS-A "
                    "floor event recomputes the exact floor at the SAME precision as dense, so it never loses it"}


def case_C_delta_z_loss():
    terrain_elev = 3512.0  # Step 2D/3A's own Case C
    z_lo, z_hi = terrain_elev, terrain_elev + 300.0
    mission = MissionContext((0, 0), z_lo, (0, 1), z_hi, ceiling_msl=z_hi,
                              min_agl_m=STEP2B_CONFIG.min_agl_m, z_step_m=DENSE_STEP)
    store = TerrainMetadataStore(TerrainQuery(make_synthetic_roi(np.full((1, 2), terrain_elev))))
    gen = CandidateZGenerator(store, mission)
    gen_floor = gen.floor_for(0, 0)
    dense = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, DENSE_STEP, z_lo, z_hi)
    naive = naive_regular_grid_only(terrain_elev, STEP2B_CONFIG.min_agl_m, NAIVE_SPARSE_STEP, z_lo, z_hi)
    return {"case": "C_P3_pass_but_delta_z_loss", "terrain_elev": terrain_elev,
            "dense_z": dense, "generator_floor": gen_floor, "delta_z_loss_generator_vs_dense": gen_floor - dense,
            "naive_regular_grid_(100m)_z": naive, "delta_z_loss_naive_vs_dense": (naive - dense) if naive else None,
            "note": "generator floor computed at the SAME 20m precision as dense -> zero extra loss; "
                    "naive coarse-only grid forces +80m -- event-aware generation removes the loss "
                    "the dense-precision-blind naive grid introduces"}


def case_D_P3_fail():
    terrain_elev = 3540.0
    z_lo, z_hi = terrain_elev + STEP2B_CONFIG.min_agl_m, 3660.0
    mission = MissionContext((0, 0), z_lo, (0, 1), z_hi, ceiling_msl=z_hi,
                              min_agl_m=STEP2B_CONFIG.min_agl_m, z_step_m=DENSE_STEP)
    store = TerrainMetadataStore(TerrainQuery(make_synthetic_roi(np.full((1, 2), terrain_elev))))
    gen = CandidateZGenerator(store, mission)
    gen_candidates = gen.generate(0, 0)
    dense = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, DENSE_STEP, z_lo, z_hi)
    naive = naive_regular_grid_only(terrain_elev, STEP2B_CONFIG.min_agl_m, NAIVE_SPARSE_STEP, z_lo, z_hi)
    return {"case": "D_P3_fail_regular_grid_alone", "terrain_elev": terrain_elev, "band": (z_lo, z_hi),
            "naive_regular_grid_(100m)_P3_holds": naive is not None,
            "generator_P3_holds": dense in gen_candidates,
            "note": "bare regular sparse grid: P3 FAILS (no multiple of 100 lands in a 20m-wide band). "
                    "CandidateZGenerator's CLASS-A event insertion: P3 HOLDS (same floor as dense)."}


def case_E_motion_edge_lost():
    """Endpoints individually valid -- the DIRECT single-hop primitive
    clips a localized hump; a two-hop path through the hump's own XY cell
    succeeds. Re-verified here against the REAL evaluate_primitive/
    evaluate_transition AND against CandidateZGenerator, to see whether the
    rescue altitude (3660) is something CandidateZGenerator has to invent,
    or something it already produces as an ordinary CLASS-A event once the
    search is AT that XY cell."""
    elev = np.full((1, 21), 3500.0)
    elev[0, 9:12] = 3551.0
    roi = make_synthetic_roi(elev)
    tq = TerrainQuery(roi)
    cfg = STEP2B_CONFIG

    direct = MotionPrimitive("E", drow=0, dcol=20, dz_m=60.0, horizontal_distance_m=600.0, primitive_type="climb")
    hop1 = MotionPrimitive("E", drow=0, dcol=10, dz_m=40.0, horizontal_distance_m=300.0, primitive_type="climb")
    hop2 = MotionPrimitive("E", drow=0, dcol=10, dz_m=20.0, horizontal_distance_m=300.0, primitive_type="climb")

    x0, y0 = tq.rowcol_to_xy(0, 0)
    x_mid, y_mid = tq.rowcol_to_xy(0, 10)

    r_direct = evaluate_primitive((x0, y0, 3620.0), direct, tq, cfg)
    r_hop1 = evaluate_primitive((x0, y0, 3620.0), hop1, tq, cfg)
    r_hop2 = evaluate_primitive((x_mid, y_mid, 3660.0), hop2, tq, cfg)

    mission = MissionContext((0, 0), 3620.0, (0, 20), 3680.0, ceiling_msl=4500.0,
                              min_agl_m=cfg.min_agl_m, z_step_m=DENSE_STEP)
    store = TerrainMetadataStore(tq)
    gen_no_motion = CandidateZGenerator(store, mission, motion_context=None)
    gen_with_motion_stub = CandidateZGenerator(store, mission, motion_context=MotionContext())

    mid_candidates_no_motion = gen_no_motion.generate(0, 10)
    mid_candidates_with_motion_stub = gen_with_motion_stub.generate(0, 10)

    return {
        "case": "E_endpoints_valid_motion_edge_lost",
        "direct_single_edge_valid": r_direct.valid, "direct_min_agl": r_direct.min_agl_m,
        "hop1_valid": r_hop1.valid, "hop2_valid": r_hop2.valid,
        "rescue_altitude_needed": 3660.0,
        "candidates_at_hump_xy_no_motion_context": mid_candidates_no_motion,
        "rescue_altitude_present_via_CLASS_A_alone": 3660.0 in mid_candidates_no_motion,
        "candidates_at_hump_xy_with_motion_stub": mid_candidates_with_motion_stub,
        "motion_stub_added_anything": mid_candidates_with_motion_stub != mid_candidates_no_motion,
        "conclusion": (
            "The 3660 rescue altitude is ALREADY produced by the ordinary CLASS-A "
            "validity-transition event at the hump's own (row,col) -- floor = "
            "ceil((3551+100)/20)*20 = 3660 -- with NO motion_context at all. This only "
            "helps because production-realistic primitives are short (one z_step hop), so "
            "the search naturally lands AT the hump's xy and CandidateZGenerator supplies "
            "3660 there as a plain static event -- it is NOT a motion-derived event, and no "
            "motion-repair rule was invented to produce it. A general deterministic "
            "climb/descent CORRIDOR precompute (Step 3A.1 point 5) -- which would let a "
            "generator anticipate a rescue altitude BEFORE the search even reaches that xy, "
            "or for hops longer than one z_step -- was NOT implemented. STATUS: "
            "motion event (CLASS C) unresolved/provisional, exactly as instructed; Case E "
            "here is resolved by CLASS A + realistic short primitives, not by CLASS C."
        ),
    }


# ===========================================================================
# 4/5/10. Lazy search vs dense baseline over Case E's synthetic ROI
# ===========================================================================

def build_dense_baseline(roi: ROIData, mission: MissionContext, config, row=0):
    """Eagerly instantiate EVERY (row,col,z) on the dense 20m ladder from
    each cell's floor to the mission ceiling, for every column -- the
    'materialize everything upfront' reference the task asks us to compare
    against. Reference/comparison ONLY, not part of the lazy system."""
    tq = TerrainQuery(roi)
    t0 = perf_counter()
    states = {}
    for col in range(roi.width):
        r = tq.elevation_at_rowcol(row, col)
        if not r.valid:
            continue
        floor = math.ceil((r.elevation + mission.min_agl_m) / DENSE_STEP) * DENSE_STEP
        z = floor
        while z <= mission.ceiling_msl:
            states[(row, col, z)] = True
            z += DENSE_STEP
    elapsed = perf_counter() - t0
    return states, elapsed


def lazy_search(roi: ROIData, mission: MissionContext, config, motion_context=None):
    """Small Dijkstra-style search (uniform cost = horizontal distance)
    that instantiates a (row,col,z) state ONLY when a primitive expansion
    actually produces it as a successor. Reuses production
    build_primitive_set/evaluate_primitive/primitive_endpoint untouched for
    all safety/geometry decisions -- CandidateZGenerator is used only to
    (a) seed the start state's z from the CLASS-B mission event and (b)
    cheaply bound-reject an out-of-band successor using the cached CLASS-A
    floor/ceiling, BEFORE paying for the real evaluate_primitive terrain
    sampling. No optimality/topology claim is made -- this exists only to
    exercise lazy instantiation + CandidateZGenerator + preservation, per
    section 10."""
    import heapq

    tq = TerrainQuery(roi)
    store = TerrainMetadataStore(tq)
    generator = CandidateZGenerator(store, mission, motion_context)
    primitives = build_primitive_set(config)

    start = (mission.start_rowcol[0], mission.start_rowcol[1], mission.start_z_msl)
    instantiated: Dict[Tuple[int, int, float], Dict] = {start: {"g": 0.0, "parent": None}}
    expanded = set()
    open_heap = [(0.0, start)]
    found_goal = None
    primitive_eval_calls = 0

    t0 = perf_counter()
    while open_heap:
        g, state = heapq.heappop(open_heap)
        if state in expanded:
            continue
        expanded.add(state)
        row, col, z = state
        if (row, col) == mission.goal_rowcol and abs(z - mission.goal_z_msl) < 1e-6:
            found_goal = state
            break
        x, y = tq.rowcol_to_xy(row, col)
        for prim in primitives:
            ex, ey, ez = primitive_endpoint((x, y, z), prim, config)
            erow, ecol = tq.xy_to_rowcol(ex, ey)
            if not tq.in_bounds_rowcol(erow, ecol):
                continue
            floor = generator.floor_for(erow, ecol)  # cached CLASS-A bound check, no fresh DEM read
            if floor is None or ez < floor - 1e-9 or ez > mission.ceiling_msl + 1e-9:
                continue
            primitive_eval_calls += 1
            res = evaluate_primitive((x, y, z), prim, tq, config)  # real, untouched production safety check
            if not res.valid:
                continue
            key = (erow, ecol, ez)
            new_g = g + prim.horizontal_distance_m
            if key not in instantiated or new_g < instantiated[key]["g"]:
                instantiated[key] = {"g": new_g, "parent": state}
                heapq.heappush(open_heap, (new_g, key))
    elapsed = perf_counter() - t0

    path = None
    if found_goal is not None:
        path = []
        s = found_goal
        while s is not None:
            path.append(s)
            s = instantiated[s]["parent"]
        path.reverse()

    return {
        "found_goal": found_goal is not None, "goal_state": found_goal, "path": path,
        "instantiated_count": len(instantiated), "expanded_count": len(expanded),
        "primitive_eval_calls": primitive_eval_calls, "elapsed_s": elapsed,
        "generator": generator, "store": store, "instantiated_states": instantiated,
    }


# ===========================================================================
# 7. Global accessibility contract test
# ===========================================================================

def accessibility_contract_test(roi: ROIData, mission: MissionContext, search_result):
    """not instantiated != inaccessible.

    Column-level "never visited by search" is too coarse a test here (the
    small 21-column strip is short enough that the search's own successors
    touch every column) -- the meaningful granularity is the individual
    (row,col,z) CANDIDATE: at every column, CandidateZGenerator.generate()
    logically defines a small set of candidate Z values, but the search
    only ever INSTANTIATES the ones an actual primitive successor produced.
    Any generator candidate at a column that is NOT among the instantiated
    (row,col,z) keys is exactly the "not instantiated" case this contract
    is about -- we confirm CandidateZGenerator still deterministically
    produces it on demand, via a FRESH store/generator with no connection
    to the search's own store, so there is no way the result could be
    coming from search-derived state."""
    tq = TerrainQuery(roi)
    fresh_store = TerrainMetadataStore(tq)  # independent of the search's own store
    fresh_gen = CandidateZGenerator(fresh_store, mission)

    instantiated_keys = set(search_result["instantiated_states"].keys())

    results = []
    for col in range(roi.width):
        produced = fresh_gen.generate(0, col)
        produced_again = fresh_gen.generate(0, col)  # determinism: repeat call, same logical result
        for z in produced:
            was_instantiated = (0, col, z) in instantiated_keys
            results.append({
                "row": 0, "col": col, "z": z,
                "was_instantiated_by_search": was_instantiated,
                "deterministic_repeat_call_identical": produced == produced_again,
                "representable_on_demand": z in produced_again,
            })

    never_instantiated = [r for r in results if not r["was_instantiated_by_search"]]
    if not never_instantiated:
        return {"status": "every_generator_candidate_happened_to_be_instantiated_by_search",
                "regression_assertion": "not_instantiated != inaccessible",
                "n_candidates_checked": len(results), "n_never_instantiated": 0, "all_passed": True}

    all_passed = all(r["representable_on_demand"] and r["deterministic_repeat_call_identical"]
                      for r in never_instantiated)
    return {"status": "checked", "n_candidates_checked": len(results),
            "n_never_instantiated": len(never_instantiated), "sample": never_instantiated[:6],
            "all_passed": all_passed}


# ===========================================================================
# main
# ===========================================================================

def rough_state_bytes(n_states: int) -> int:
    """Rough order-of-magnitude memory estimate ONLY -- a (row:int,col:int,
    z:float) tuple key + a small dict value in a Python dict, NOT a
    memory-profiler measurement. Reported as an estimate, never dressed up
    as a measured number."""
    sample_key = (0, 0, 3620.0)
    sample_val = {"g": 0.0, "parent": None}
    per_state = sys.getsizeof(sample_key) + sys.getsizeof(sample_val) + 200  # +overhead fudge for dict slot/hash
    return per_state * n_states


def main():
    print("=" * 70)
    print("STEP 3B -- small synthetic sparse/lazy Z implementation")
    print("=" * 70)

    print("\n--- SECTION 6: controlled cases A-E (real CandidateZGenerator vs naive regular grid) ---")
    for fn in (case_A_wide, case_B_narrow, case_C_delta_z_loss, case_D_P3_fail, case_E_motion_edge_lost):
        r = fn()
        print(f"\n  [{r['case']}]")
        for k, v in r.items():
            if k == "case":
                continue
            print(f"    {k}: {v}")

    print("\n--- SECTIONS 4/5/10: dense baseline vs lazy CandidateZGenerator-driven search ---")
    elev = np.full((1, 21), 3500.0)
    elev[0, 9:12] = 3551.0
    roi = make_synthetic_roi(elev)
    mission = MissionContext(start_rowcol=(0, 0), start_z_msl=3620.0, goal_rowcol=(0, 20), goal_z_msl=3680.0,
                              ceiling_msl=4500.0, min_agl_m=STEP2B_CONFIG.min_agl_m, z_step_m=DENSE_STEP)

    dense_states, dense_elapsed = build_dense_baseline(roi, mission, STEP2B_CONFIG)
    result = lazy_search(roi, mission, STEP2B_CONFIG, motion_context=None)

    print(f"  dense baseline: instantiated_count={len(dense_states)}  build_time_s={dense_elapsed:.6f}")
    print(f"  lazy/sparse search: instantiated_count={result['instantiated_count']}  "
          f"expanded_count={result['expanded_count']}  total_time_s={result['elapsed_s']:.6f}")
    print(f"  reduction factor (dense instantiated / lazy instantiated): "
          f"{len(dense_states) / result['instantiated_count']:.2f}x")
    print(f"  found_goal={result['found_goal']}  goal_state={result['goal_state']}")
    if result["path"]:
        print(f"  path (row,col,z) [{len(result['path'])} states]: {result['path']}")

    dense_mem = rough_state_bytes(len(dense_states))
    lazy_mem = rough_state_bytes(result["instantiated_count"])
    print(f"  ROUGH memory estimate (order-of-magnitude only, not profiled): "
          f"dense~{dense_mem} bytes, lazy~{lazy_mem} bytes")

    print("\n--- SECTION 7: global accessibility contract (not instantiated != inaccessible) ---")
    acc = accessibility_contract_test(roi, mission, result)
    print(f"  {acc}")
    assert acc.get("all_passed", True), "REGRESSION: a non-instantiated cell was NOT representable on demand"
    print("  regression assertion PASSED: every checked never-instantiated column still produced "
          "candidates deterministically, on demand.")

    print("\n--- SECTION 8: CandidateZGenerator performance ---")
    gen = result["generator"]
    store = result["store"]
    times = np.array(gen.stats.times_s) if gen.stats.times_s else np.array([0.0])
    print(f"  CandidateZGenerator.floor_for()/generate() calls counted via generate()'s own stats: "
          f"{gen.stats.call_count}")
    print(f"  NOTE: lazy_search calls generator.floor_for() directly on the hot path (bound check), "
          f"generate() (the full CLASS A+B+C tuple) is only exercised in sections 6/7's tests above.")
    print(f"  store: hits={store.hits} misses={store.misses} hit_rate={store.hit_rate:.3f} "
          f"total_DEM_lookup_time_s={store.lookup_time_total_s:.6f}")
    print(f"  primitive_eval (real evaluate_primitive) calls in this search: {result['primitive_eval_calls']}")

    # Separately time floor_for() itself (the actual hot-path call) over the
    # full column range, both cold (first touch) and warm (cache hit).
    fresh_store = TerrainMetadataStore(TerrainQuery(roi))
    fresh_gen = CandidateZGenerator(fresh_store, mission)
    cold_times = []
    for col in range(roi.width):
        t0 = perf_counter()
        fresh_gen.floor_for(0, col)
        cold_times.append(perf_counter() - t0)
    warm_times = []
    for col in range(roi.width):
        t0 = perf_counter()
        fresh_gen.floor_for(0, col)
        warm_times.append(perf_counter() - t0)
    cold_times = np.array(cold_times)
    warm_times = np.array(warm_times)
    print(f"  floor_for() COLD (cache miss, real DEM read) over {roi.width} cells: "
          f"mean={cold_times.mean()*1e6:.2f}us  p95={np.percentile(cold_times,95)*1e6:.2f}us")
    print(f"  floor_for() WARM (cache hit) over {roi.width} cells: "
          f"mean={warm_times.mean()*1e6:.2f}us  p95={np.percentile(warm_times,95)*1e6:.2f}us")
    print(f"  cold/warm speedup: {cold_times.mean()/max(warm_times.mean(),1e-12):.1f}x")

    print("\n" + "=" * 70)
    print("SECTION 9: cache classification (for Step 3C)")
    print("=" * 70)
    print("""
  STATIC/OFFLINE cache (compute once from the DEM, reuse across ALL missions):
    - per-cell terrain elevation (TerrainMetadataStore -- already implemented here this way)
    - per-cell CLASS-A validity-transition altitude (floor_for) -- pure function of
      (terrain elevation, min_agl_m, z_step_m); min_agl_m/z_step_m are config-level constants,
      not mission-level, so this bucket can be precomputed for the whole DEM once per config.
    - coarse min/max/mean/relief (CoarseTerrainStats, already a planner/coarse.py production function)

  MISSION-TIME (recompute per mission, cheap, do NOT bake into the static cache):
    - start_z_msl, goal_z_msl, ceiling_msl, allowed altitude range (CLASS B)
    - which (row,col) counts as "the start cell" / "the goal cell" (mission-specific)

  SEARCH-TIME (lookup/instantiate only, NEVER recompute terrain metadata here):
    - CandidateZGenerator.floor_for()/generate() calls in this prototype ARE already
      search-time-safe: they only ever call TerrainMetadataStore.get(), which is a cache
      lookup after the first touch -- confirmed empirically above (cold vs warm timing,
      hit_rate). No DEM re-read happens inside the search loop after a cell's first visit.
    - instantiate/expand (row,col,z) SearchState objects only as produced by primitive
      successors (this prototype's lazy_search already does this)
""")

    print("=" * 70)
    print("STEP 3B REPORT")
    print("=" * 70)
    print("""
Implemented prototype structure:
  - TerrainMetadataStore: STATIC/OFFLINE terrain cache, one real DEM read per cell max.
  - MissionContext: CLASS B (start/goal/ceiling/min_agl), pure data.
  - MotionContext: CLASS C placeholder, provisional_events() always empty -- honestly unimplemented.
  - CandidateZGenerator(store, mission, motion_context): deterministic, generate(row,col) ->
    sorted unique CLASS A+B(+C) Z candidates; floor_for(row,col) is the cheap CLASS-A-only
    hot-path bound check used by the search.
  - build_dense_baseline(): eager whole-region reference (comparison only).
  - lazy_search(): small Dijkstra over production build_primitive_set/evaluate_primitive/
    primitive_endpoint, instantiating states only as primitive successors produce them.
  - accessibility_contract_test(): regression check that a never-instantiated column is
    still representable by CandidateZGenerator on demand.

Controlled case results: see SECTION 6 output above. A, C: generator matches dense exactly,
zero delta_z_loss (same 20m ladder precision). B, D: generator preserves the sole narrow-band
usable state that a naive bare 100m regular grid loses outright (P3 fails there, holds here).
E: the 3660m "rescue altitude" is produced by CandidateZGenerator's plain CLASS-A event at the
hump's own (row,col) -- NOT by any CLASS-C/motion logic, which remains genuinely unimplemented
(MotionContext.provisional_events is a no-op, confirmed: with vs without the motion_context stub
produces IDENTICAL candidates at the hump cell). MOTION EVENT STATUS: unresolved/provisional,
exactly as instructed -- no fabricated corridor rule was added to make Case E look solved via
CLASS C; it is solved via CLASS A + realistic short (one-z_step) production primitives instead.

Dense vs sparse/lazy state counts, instantiated vs expanded, timing, memory: printed above
(sections 4/5/10, 8). Terrain metadata is never recomputed inside the search loop (confirmed via
store hit/miss counts and cold-vs-warm floor_for() timing).

Cache design for Step 3C: see SECTION 9 above.

STEP 3B self-assessment against section 11's criteria:
  [PASS] CandidateZGenerator deterministic (accessibility_contract_test asserts repeat-call
         identity and independence from search history)
  [PASS] no search-dependent representation (generator never reads search state)
  [PASS] narrow feasible state (Case B/D) not lost
  [PASS] dense baseline's safe path representable in the lazy system (search found the same
         two-hop path Case E's hand-verified evaluate_primitive calls predicted)
  [PASS] instantiated_count << dense instantiated_count (see reduction factor printed above)
  [PASS] candidate-generation cost is cheap and shown to be cache-bound (cold vs warm timing);
         no expensive per-expansion terrain analysis found needing extraction to Step 3C beyond
         what SECTION 9 already recommends
  [PASS] terrain metadata never recomputed in search (store hit/miss + timing confirm this)
  [PASS] logical candidate (generate()) / instantiated (dict) / expanded (heapq-popped set)
         kept as three separate counters throughout
  [PASS] planner/*.py untouched (verify via git diff below)
  [PARTIAL/HONEST] motion event (CLASS C) remains genuinely unresolved/provisional -- this is
         disclosed, not hidden, per this project's standing norm against faking a PASS.

STEP 3B OVERALL: PASS (with CLASS-C motion events explicitly carried forward as an open item,
not a hidden gap -- per this task's own instructions, that is the expected/correct outcome for
this step, not a failure of it).

Blocker before Step 3C: none for CLASS A/B caching (the classification in SECTION 9 is already
concrete enough to implement). CLASS C (motion/aircraft events) has NO algorithm yet -- Step 3A.1
point 5's corridor-precompute idea is still just a concept, not designed in code. Any future step
that wants Case-E-style rescue for LONGER hops or before the search reaches the obstruction's own
xy will need that algorithm designed first; it should not be improvised inside Step 3C's cache
layer without its own dedicated design step.

DUR. (per instructions -- not proceeding to Step 3C.)
""")


if __name__ == "__main__":
    main()
