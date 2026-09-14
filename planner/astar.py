"""Legacy CandidateZ/grid A* compatibility implementation.

The canonical state is ``(row, col, altitude_id)``. Its third component always
encodes an exact CandidateZ event altitude. It is retained only for historical
regressions and artifact inspection; production benchmark entry points use
``planner.pose_search``.  Neighbor generation is lazy and all hard terrain,
AGL, and transition checks remain delegated to the primitive safety evaluator.
"""
import heapq
import itertools
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from planner.candidate_z import CandidateZGenerator
from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.mission import mission_policy_from_config
from planner.primitives import (
    DIRECTIONS,
    MotionPrimitive,
    Point3,
    evaluate_primitive,
    primitive_for_single_step_target_altitude,
    primitive_for_target_altitude_over_horizon,
)
from planner.terrain import TerrainQuery
from planner.vertical_motion import (
    SafeVerticalRateResult,
    derive_minimum_horizontal_distance_m,
    evaluate_vertical_motion,
    query_safe_vertical_rate,
)

CanonicalState = Tuple[int, int, int]  # (row, col, altitude_id)


# --------------------------------------------------------------------------
# State <-> altitude conversion.
# --------------------------------------------------------------------------

# Fine-grained, lattice-independent altitude encoding for CandidateZ states.
# Micrometre precision is exact for any real aircraft altitude (float64 represents integers
# up to 2**53 exactly, and altitudes in this planner never approach 1e9 micrometres from a
# reasonable MSL origin), so this is a pure canonicalization -- no lattice alignment required, no
# accumulated drift (always derived fresh from a float via round(), never by adding integer deltas).
_CANDIDATE_Z_SCALE = 1_000_000.0  # micrometres


def encode_candidate_altitude(z_msl: float) -> int:
    """MSL altitude -> a stable, hashable, deterministic CandidateZ state key.

    This never snaps to a lattice: any real z_msl (including an off-lattice exact mission
    start/goal altitude) round-trips through this and decode_candidate_
    altitude() to itself (to micrometre precision)."""
    return round(z_msl * _CANDIDATE_Z_SCALE)


def decode_candidate_altitude(z_key: int) -> float:
    return z_key / _CANDIDATE_Z_SCALE


class _SearchLocalSafeVerticalRateCache:
    """Memoize immutable profile resolution for one A* invocation."""

    def __init__(self, aircraft_profile):
        self._aircraft_profile = aircraft_profile
        self._values: Dict[Tuple[int, str], SafeVerticalRateResult] = {}
        self.lookups = 0
        self.hits = 0
        self.misses = 0
        self.profile_query_time_s = 0.0

    def get(self, altitude_id: int, mode: str) -> SafeVerticalRateResult:
        self.lookups += 1
        key = (altitude_id, mode)
        cached = self._values.get(key)
        if cached is not None:
            self.hits += 1
            return cached
        self.misses += 1
        started = time.perf_counter()
        resolved = query_safe_vertical_rate(
            self._aircraft_profile, decode_candidate_altitude(altitude_id), mode
        )
        self.profile_query_time_s += time.perf_counter() - started
        self._values[key] = resolved
        return resolved


def state_to_xyz(
    state: CanonicalState,
    terrain: TerrainQuery,
) -> Point3:
    """Convert one CandidateZ canonical state to physical coordinates."""
    row, col, altitude_id = state
    x, y = terrain.rowcol_to_xy(row, col)
    return (x, y, decode_candidate_altitude(altitude_id))


def path_to_xyz(
    path: List[CanonicalState],
    terrain: TerrainQuery,
) -> List[Point3]:
    return [state_to_xyz(s, terrain) for s in path]


def _distance_to_goal_box(
    x: float, y: float, z: float,
    goal_x: float, goal_y: float, goal_z: float,
    tolerance_xy_m: float, tolerance_z_m: float,
) -> float:
    """Stage 33: minimum 3D distance from (x, y, z) to the axis-aligned goal
    tolerance box (goal_x +/- tolerance_xy_m, goal_y +/- tolerance_xy_m,
    goal_z +/- tolerance_z_m). 0.0 when the point is already inside.

    dx_out/dy_out/dz_out = max(abs(delta) - tolerance, 0) per axis --
    "how far outside the box on this axis, or 0 if already within it" --
    then the ordinary 3D norm of those. Reduces EXACTLY (bit-for-bit) to
    the plain point-to-point distance when both tolerances are 0.0, since
    max(abs(d) - 0.0, 0.0) == abs(d) always (subtracting then max-ing with
    0.0 changes nothing when abs(d) is already >= 0) -- every caller can
    use this unconditionally, with no separate exact-goal formula needed.
    """
    dx_out = max(abs(x - goal_x) - tolerance_xy_m, 0.0)
    dy_out = max(abs(y - goal_y) - tolerance_xy_m, 0.0)
    dz_out = max(abs(z - goal_z) - tolerance_z_m, 0.0)
    return math.sqrt(dx_out * dx_out + dy_out * dy_out + dz_out * dz_out)


def _state_in_goal_region(
    state: CanonicalState,
    goal: CanonicalState,
    terrain: TerrainQuery,
    config: PlannerConfig,
) -> bool:
    """Stage 33: is `state`'s physical position within the goal tolerance
    box around `goal`? Metric x/y come from the terrain's real affine
    transform (state_to_xyz), never a blind row/col*resolution shortcut --
    stays correct even if the grid were ever non-north-up. When both
    tolerances are exactly 0.0 (the default), takes the EXACT pre-Stage-33
    integer-tuple comparison verbatim (zero floating point involved at
    all) -- guarantees bit-for-bit identical behavior to every earlier
    stage in that case.

    NEVER a safety bypass: this only asks "is the position close enough",
    never "is it safe". A state only ever reaches this check after already
    passing evaluate_primitive() inside successor generation (AGL, terrain
    collision, NoData, bounds, climb/descent angle) -- an unsafe state is
    never even a candidate here, regardless of tolerance, so a goal box
    can never "rescue" an otherwise-invalid state into being a solution.
    """
    if config.goal_tolerance_xy_m == 0.0 and config.goal_tolerance_z_m == 0.0:
        return state == goal
    x1, y1, z1 = state_to_xyz(state, terrain)
    x2, y2, z2 = state_to_xyz(goal, terrain)
    return (
        abs(x1 - x2) <= config.goal_tolerance_xy_m
        and abs(y1 - y2) <= config.goal_tolerance_xy_m
        and abs(z1 - z2) <= config.goal_tolerance_z_m
    )
# --------------------------------------------------------------------------
# Search result
# --------------------------------------------------------------------------

# Canonical termination-reason taxonomy for astar_search.
# TIMEOUT/EXPANSION_LIMIT are budget cutoffs (this
# call's own watchdog, not a claim about the goal) -- see project.md
# "Step PERF-0" for the full TIMEOUT != UNREACHABLE / EXPANSION_LIMIT !=
# UNREACHABLE / DIRECTLY INFEASIBLE != UNREACHABLE contract.
_TERMINATION_REASON_BY_STATUS = {
    "success": "FOUND",
    "no_path": "NO_PATH",
    "search_limit_reached": "EXPANSION_LIMIT",
    "timeout": "TIMEOUT",
}

@dataclass(frozen=True)
class SearchResult:
    success: bool
    status: str  # "success" | "no_path" | "search_limit_reached" | "timeout"
    # Step PERF-0: same information as `status`, normalized to the canonical
    # FOUND/NO_PATH/EXPANSION_LIMIT/TIMEOUT taxonomy used across the whole
    # planner (see _TERMINATION_REASON_BY_STATUS). Prefer this field in new
    # code; `status` stays for existing callers. TIMEOUT and EXPANSION_LIMIT
    # are budget cutoffs, never a claim that the goal is unreachable -- only
    # NO_PATH (open_heap exhausted, no incumbent) is a genuine negative
    # result within the given search bounds.
    termination_reason: str
    path: List[CanonicalState]
    total_cost: float  # sum of weighted edge costs -- what A* actually minimized (== "total_weighted_cost")
    expanded_nodes: int
    generated_neighbors: int
    rejected_neighbors: int
    rejected_reason_counts: Dict[str, int] = field(default_factory=dict)
    max_open_size: int = 0
    runtime_s: float = 0.0
    # Diagnostics computed once, only for a found path -- NaN when success is False.
    geometric_path_length: float = float("nan")  # unweighted 3D length: sum(sqrt(d_xy^2 + dz^2))
    minimum_aircraft_msl: float = float("nan")
    maximum_aircraft_msl: float = float("nan")
    average_aircraft_msl: float = float("nan")  # geometric-length-weighted mean altitude
    minimum_observed_agl: float = float("nan")  # re-derived from evaluate_primitive() per path edge
    # Physical vertical-motion metrics -- NOT cost inputs anymore, just description of the path.
    total_climb_m: float = float("nan")  # sum of positive delta_z across path edges
    total_descent_m: float = float("nan")  # sum of abs(negative delta_z) across path edges
    total_vertical_motion_m: float = float("nan")  # total_climb_m + total_descent_m
    # Search-local safe vertical-rate cache diagnostics.
    vertical_rate_cache_lookups: int = 0
    vertical_rate_cache_hits: int = 0
    vertical_rate_cache_misses: int = 0
    vertical_rate_cache_hit_rate: float = float("nan")
    aircraft_profile_vertical_queries: int = 0
    aircraft_profile_vertical_query_time_s: float = 0.0
    # MSL-aware lower-bound heuristic (see _heuristic / _min_possible_aircraft_msl below).
    # heuristic_cost_multiplier is 1.0 whenever use_msl_lower_bound_heuristic=False, or when
    # the config's weights don't satisfy the non-negativity this bound relies on (safe fallback).
    minimum_possible_aircraft_msl: float = float("nan")
    heuristic_cost_multiplier: float = 1.0
    # Incumbent upper-bound / branch-and-bound pruning (Stage 23) -- all 0/False/inf
    # when use_incumbent_pruning=False, since there's no incumbent bound in effect.
    initial_incumbent_available: bool = False
    initial_incumbent_cost: float = float("inf")
    final_incumbent_cost: float = float("inf")
    incumbent_updates: int = 0
    incumbent_pruned_candidates: int = 0  # candidate generation-time prunes (point A)
    incumbent_heap_pops_skipped: int = 0  # heap pop-time prunes (point B) -- see astar_search
    incumbent_termination_triggered: bool = False  # loop ended via "min open f >= incumbent", not a goal pop
    incumbent_path: List[CanonicalState] = field(default_factory=list)  # best known solution's path, regardless of overall status
    # Weighted A* / certified bounded-suboptimal search (Stage 25) -- epsilon_search=1.0 and
    # target_suboptimality=None (both defaults) reproduce the pre-Stage-25 search exactly.
    epsilon_search: float = 1.0
    target_suboptimality: Optional[float] = None
    first_solution_cost: float = float("inf")
    first_solution_expanded: int = 0
    first_solution_runtime_s: float = float("nan")
    current_lower_bound: float = float("nan")
    final_bound_ratio: float = float("nan")  # final_incumbent_cost / current_lower_bound
    bounded_termination_triggered: bool = False  # stopped early via incumbent <= target_suboptimality * LB
    reopened_states: int = 0  # states re-added to open after being closed with a worse g (weighted-mode only)
    # Stage 32: which cost formula this run used, and (only for "normalized") the D_ref it computed
    # once from start/goal -- NaN under "legacy", where it's never computed/used.
    cost_mode: str = "legacy"
    distance_reference_m: float = float("nan")
    # Safe goal region (Stage 33) -- goal_tolerance_* mirror config (both 0.0 reproduces the exact
    # pre-Stage-33 goal condition). The three closest_* fields are always tracked (cheap, purely
    # observational -- never affect search behavior) so a FAILED run can still report how close the
    # search actually got, in real UTM meters, to both the goal box and the exact goal center.
    goal_tolerance_xy_m: float = 0.0
    goal_tolerance_z_m: float = 0.0
    closest_distance_to_goal_region_m: float = float("inf")  # 0.0 iff some expanded state ever entered the box
    closest_distance_to_goal_center_m: float = float("inf")  # plain 3D distance to the exact goal center
    closest_state_to_goal: Optional[CanonicalState] = None  # state that achieved the region minimum
    # State-space composition diagnostics -- always tracked (cheap and purely
    # observational, derived from `closed` at the end of the run).
    # Since Step CLEAN-1 (state == physical (row,col,z), no history dimension),
    # unique_expanded_xyz is exactly len(closed) -- kept for continuity with the
    # per-(row,col) Z-diversity fields below, which remain genuinely meaningful.
    unique_expanded_xy: int = 0
    unique_expanded_xyz: int = 0
    avg_z_states_per_xy: float = float("nan")
    max_z_states_in_one_xy: int = 0


def _heuristic(
    current: CanonicalState,
    goal: CanonicalState,
    terrain: TerrainQuery,
    config: PlannerConfig,
    cost_multiplier: float = 1.0,
    min_possible_msl: Optional[float] = None,
    use_vertical_reachability: bool = False,
    distance_reference_m: Optional[float] = None,
) -> float:
    """h = max(h_global, h_forward) -- both individually admissible and
    consistent lower bounds on the true remaining cost, so their max is
    too (standard result for combining admissible/consistent heuristics).

    h_global = D3D(current, goal) * cost_multiplier -- see
    _min_possible_aircraft_msl / _msl_lower_bound_multiplier (Stage 17).
    cost_multiplier=1.0 (the default) reproduces the plain-geometric
    heuristic exactly.

    h_forward (only computed when use_vertical_reachability=True and
    min_possible_msl is given) additionally exploits that the aircraft
    can't descend faster than config.max_descent_angle_deg allows -- see
    _forward_vertical_reachability_heuristic for the formula and
    project.md "Stage 21" for the two-part (admissible + consistent)
    proof. Generic/angle-agnostic: driven entirely by
    config.max_descent_angle_deg, never a hardcoded angle value, so a
    future different or continuous angle system needs no change here.

    cost_mode == "normalized" (Stage 32) takes a DIFFERENT, deliberately
    minimal path: h = normalized_w_distance * D3D / distance_reference_m.
    This is a valid admissible+consistent lower bound because the true
    normalized edge cost is always >= w_distance*dC_distance (the altitude
    term is non-negative -- see _compute_edge_cost_normalized),
    and sum(dC_distance) over any path from n to goal >= D3D(n,goal)/D_ref
    by the ordinary Euclidean triangle inequality. The legacy
    cost_multiplier / min_possible_msl / vertical-reachability machinery
    (Stage 17/21) is NOT applied here -- see project.md "Stage 32" section
    16 for why that's deliberate this stage (isolate normalized cost's own
    search behavior first, no legacy heuristic blindly reused).

    D3D (Stage 33): with a nonzero goal tolerance, "remaining distance"
    means distance to the NEAREST point of the goal box, not to its exact
    center -- using center-distance here would overestimate once a state
    is already within tolerance of one axis but not another, breaking
    admissibility. _distance_to_goal_box gives that minimum-to-box distance
    (0.0 once inside), and reduces bit-for-bit to the plain center distance
    when both tolerances are 0.0, so this is the correct D3D unconditionally.
    """
    x1, y1, z1 = state_to_xyz(current, terrain)
    x2, y2, z2 = state_to_xyz(goal, terrain)
    d3d = _distance_to_goal_box(x1, y1, z1, x2, y2, z2, config.goal_tolerance_xy_m, config.goal_tolerance_z_m)

    if config.cost_mode == "normalized":
        if distance_reference_m is None or distance_reference_m <= 0.0:
            raise ValueError(
                "cost_mode='normalized' requires a positive distance_reference_m for the heuristic"
            )
        return config.normalized_w_distance * d3d / distance_reference_m

    h_global = d3d * cost_multiplier

    if not use_vertical_reachability or min_possible_msl is None:
        return h_global

    h_forward = _forward_vertical_reachability_heuristic(z1, d3d, min_possible_msl, config)
    return max(h_global, h_forward)


def _forward_vertical_reachability_heuristic(
    current_altitude_msl: float,
    remaining_distance_m: float,
    minimum_possible_aircraft_msl: float,
    config: PlannerConfig,
) -> float:
    """Generic, angle-agnostic lower bound on the remaining MSL soft-cost,
    exploiting that altitude can only decrease at a rate bounded by
    sin(config.max_descent_angle_deg) per unit of flown arc length -- not
    "10 degrees" hardcoded; a continuous 0..30deg system, or any other
    config value, needs no change here, since only the config field is read.

    z_lb(s) = max(minimum_possible_aircraft_msl,
                  current_altitude_msl - s * sin(max_descent_angle_deg))
    for s in [0, remaining_distance_m] -- the most optimistic (lowest)
    the aircraft could possibly be at arc-length s into the remaining
    flight, regardless of which path it actually takes.

    h_forward = integral_0^D of (1 + msl_cost_weight * max(0,
        (z_lb(s) - msl_reference_m) / msl_scale_m)) ds

    computed exactly (no sampling loop): the integrand is piecewise-LINEAR
    in s with at most two interior breakpoints (where z_lb hits the floor,
    and where it crosses msl_reference_m), so each piece is integrated
    with the trapezoid rule, which is exact for a linear function.

    Proof of admissibility (handles arbitrary lateral detours -- see
    project.md "Stage 21"): for the true optimal remaining path (any
    length L >= D, by the triangle inequality), altitude at real arc-length
    s along it is >= z_lb(s) (the descent-rate + floor bound holds
    pointwise for ANY path, not just a specific one). Since the integrand
    is monotonic non-decreasing in altitude, the true cost's integrand is
    >= the z_lb integrand pointwise; since that integrand is also always
    >= 1 > 0, integrating it over the true (possibly longer) domain [0, L]
    is >= integrating just [0, D] -- so truncating to D never overestimates.

    Proof of consistency: for any real edge (n, n') with cost c, prefixing
    n''s own optimal relaxed continuation (length D(n'), reaching goal)
    with that edge gives a path of length c's-primitive-length + D(n') >=
    D(n) (triangle inequality) that starts at n's actual altitude and
    never exceeds the descent-rate limit (the real edge already respects
    it, by construction of every primitive) -- i.e., a valid candidate for
    n's own relaxed problem, with cost exactly c + h_forward(n'). Since
    h_forward(n) is the minimum over all such candidates, h_forward(n) <=
    c + h_forward(n') directly.

    Does no terrain sampling.
    """
    if config.max_descent_angle_deg is None:
        return 0.0  # no known rate limit to exploit -- 0 is still a trivially valid (if weak) lower bound
    D = remaining_distance_m
    if D <= 0.0:
        return 0.0

    descent_rate = math.sin(math.radians(config.max_descent_angle_deg))

    def z_lb(s: float) -> float:
        if descent_rate <= 1e-12:
            return current_altitude_msl
        return max(minimum_possible_aircraft_msl, current_altitude_msl - s * descent_rate)

    def cost_rate(z: float) -> float:
        return 1.0 + config.msl_cost_weight * max(0.0, (z - config.msl_reference_m) / config.msl_scale_m)

    breakpoints = {0.0, D}
    if descent_rate > 1e-12:
        s_floor = (current_altitude_msl - minimum_possible_aircraft_msl) / descent_rate
        if 0.0 < s_floor < D:
            breakpoints.add(s_floor)
        s_ref = (current_altitude_msl - config.msl_reference_m) / descent_rate
        if 0.0 < s_ref < D:
            breakpoints.add(s_ref)

    points = sorted(breakpoints)
    total = 0.0
    for a, b in zip(points, points[1:]):
        total += (b - a) * (cost_rate(z_lb(a)) + cost_rate(z_lb(b))) / 2.0
    return total


def _terrain_min_valid_elevation(terrain: TerrainQuery) -> float:
    """Scan the ROI's elevation array ONCE for its minimum valid (non-NoData)
    value. Callers must call this once before a search, never per heuristic
    evaluation -- see astar_search."""
    elevation = terrain.roi.elevation
    nodata = terrain.roi.nodata
    valid = elevation[elevation != nodata] if nodata is not None else elevation
    return float(valid.min())


def _min_possible_aircraft_msl(
    terrain_min_elevation_msl: float,
    min_search_altitude_msl: float,
    config: PlannerConfig,
) -> float:
    """Global lower bound: no valid (safe) state, anywhere in the ROI, can
    ever be below this MSL.

    Any safe state must clear its OWN local terrain by min_agl_m -- and
    local terrain is never below the ROI's global minimum, so
    terrain_min_elevation_msl + min_agl_m is a valid floor everywhere, not
    just at the single lowest point. The search's own altitude floor
    (min_search_altitude_msl) is an independent, always-true lower bound
    too (evaluate_primitive / the altitude-bounds filter never accepts a
    state below it). The true floor is whichever of the two is higher --
    the aircraft must respect BOTH.
    """
    agl_margin = config.min_agl_m if config.min_agl_m is not None else 0.0
    return max(min_search_altitude_msl, terrain_min_elevation_msl + agl_margin)


def _msl_lower_bound_multiplier(minimum_possible_aircraft_msl: float, config: PlannerConfig) -> Optional[float]:
    """The GLOBAL (search-wide constant) minimum_cost_multiplier such that,
    for every valid edge in this search:

        actual_edge_cost >= geometric_cost(edge) * minimum_cost_multiplier

    Proof sketch (see project.md "Stage 17" for the full derivation):
      actual_edge_cost = geometric_cost * (1 + msl_cost_weight * altitude_scaled(edge))
      Every valid state's MSL is >= minimum_possible_aircraft_msl (by
      construction -- see _min_possible_aircraft_msl), so an edge's mean
      altitude is too, so altitude_scaled(edge) >= scaled_msl_lb (monotonic).
      With msl_cost_weight >= 0: actual_edge_cost >= geometric_cost * (1 +
      msl_cost_weight * scaled_msl_lb) = geometric_cost * multiplier.

    Returns None -- signalling "fall back to the plain Euclidean heuristic"
    -- if msl_cost_weight < 0 or msl_scale_m <= 0: either breaks the proof
    above, so using this bound would risk inadmissibility. This module
    never assumes those stay non-negative; it checks, every call.
    """
    if config.msl_cost_weight < 0 or config.msl_scale_m <= 0:
        return None
    scaled_msl_lb = max(0.0, (minimum_possible_aircraft_msl - config.msl_reference_m) / config.msl_scale_m)
    return 1.0 + config.msl_cost_weight * scaled_msl_lb


def _altitude_scaled(mean_altitude_msl: float, config: PlannerConfig) -> float:
    """Fixed-scale MSL cost input. NOT clamped, NOT relative to search bounds.

    altitude_scaled = max(0, mean_altitude_msl - msl_reference_m) / msl_scale_m

    The same physical altitude always produces the same value here no
    matter what a given search call's min/max_search_altitude_msl are --
    e.g. 1400m msl -> 1.4 whether the search ceiling is 1500m or 2500m.
    Deliberately unbounded above: clamping to [0,1] was what let a wide
    search ceiling silently dilute the low-MSL preference.
    """
    return max(0.0, mean_altitude_msl - config.msl_reference_m) / config.msl_scale_m


def compute_distance_reference(
    start: CanonicalState,
    goal: CanonicalState,
    terrain: TerrainQuery,
) -> float:
    """D_ref for normalized cost mode (project.md "Stage 32"): the
    straight-line 3D distance between start and goal, computed ONCE per
    mission (never per-edge, never per-state, never inside the search
    loop). Every edge's normalized distance/altitude contribution is a
    ratio against this single constant -- independent of which path is
    actually flown."""
    x1, y1, z1 = state_to_xyz(start, terrain)
    x2, y2, z2 = state_to_xyz(goal, terrain)
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


def compute_edge_cost(
    primitive: MotionPrimitive,
    start_altitude_msl: float,
    config: PlannerConfig = DEFAULT_CONFIG,
    distance_reference_m: Optional[float] = None,
) -> float:
    """The actual A* edge cost for applying one primitive from a given
    altitude. A pure function of (primitive, start_altitude_msl, config,
    distance_reference_m) -- no search history is consulted (Step CLEAN-1
    removed the vertical_trend/trend_age_bucket reversal-cost term; see
    project.md "Step CLEAN-1" / "Step 3E.1" for why it was safe to remove).

    Public so callers
    -- validation/diagnostic scripts included -- can cost a hypothetical
    primitive sequence without duplicating this formula.

    Dispatches on config.cost_mode:

    "legacy" (default -- distance_reference_m is ignored entirely):
        edge_cost = geometric_cost * (1 + msl_cost_weight * altitude_scaled)

    "normalized" (Stage 32, opt-in -- see _compute_edge_cost_normalized):
        dimensionless dC_total = w_distance*dC_distance + w_altitude*
        dC_altitude. Requires distance_reference_m (see
        compute_distance_reference) and config.altitude_reference_msl to
        both be set -- raises ValueError otherwise rather than silently
        falling back.
    """
    if config.cost_mode == "normalized":
        if distance_reference_m is None or distance_reference_m <= 0.0:
            raise ValueError(
                "cost_mode='normalized' requires a positive distance_reference_m "
                "(see compute_distance_reference()) -- none was provided"
            )
        if config.altitude_reference_msl is None:
            raise ValueError(
                "cost_mode='normalized' requires config.altitude_reference_msl to be "
                "set explicitly (never derived from search bounds -- see project.md 'Stage 32')"
            )
        return _compute_edge_cost_normalized(primitive, start_altitude_msl, config, distance_reference_m)

    end_altitude_msl = start_altitude_msl + primitive.dz_m
    geometric_cost = math.sqrt(primitive.horizontal_distance_m ** 2 + primitive.dz_m ** 2)
    mean_altitude_msl = (start_altitude_msl + end_altitude_msl) / 2.0
    altitude_scaled = _altitude_scaled(mean_altitude_msl, config)
    return geometric_cost * (1.0 + config.msl_cost_weight * altitude_scaled)


def _compute_edge_cost_normalized(
    primitive: MotionPrimitive,
    start_altitude_msl: float,
    config: PlannerConfig,
    distance_reference_m: float,
) -> float:
    """Stage 32 dimensionless edge cost (project.md "Stage 32"), no
    history/reversal term (Step CLEAN-1):

        dC_distance = geometric_cost / D_ref
        excess_altitude = max(0, mean_MSL - altitude_reference_msl)
        dC_altitude = dC_distance * (excess_altitude / normalized_altitude_scale_m)
        dC_total = w_distance*dC_distance + w_altitude*dC_altitude

    Every term is non-negative (max(0,...) before any weighting) and the
    total is additive -- required for A* edge costs. Mission objective is
    still LOW ABSOLUTE MSL: excess_altitude is always (aircraft MSL -
    altitude_reference_msl), never (aircraft MSL - local terrain) -- this
    is not a terrain-following/preferred-AGL cost.
    """
    geometric_cost = math.hypot(primitive.horizontal_distance_m, primitive.dz_m)
    components = mission_policy_from_config(config).edge_components(
        geometric_cost,
        start_altitude_msl,
        start_altitude_msl + primitive.dz_m,
        distance_reference_m,
    )
    return components.total


def validate_path_safety(
    path: List[CanonicalState],
    terrain: TerrainQuery,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> bool:
    """Validate every CandidateZ path edge with the production safety authority."""
    if len(path) < 2:
        return False
    for s1, s2 in zip(path, path[1:]):
        r1, c1, _ = s1
        r2, c2, _ = s2
        z1_msl = decode_candidate_altitude(s1[2])
        z2_msl = decode_candidate_altitude(s2[2])
        direction = _direction_for_unit_delta(r2 - r1, c2 - c1)
        n_cells = max(abs(r2 - r1), abs(c2 - c1))
        prim = primitive_for_target_altitude_over_horizon(
            direction, z1_msl, z2_msl, n_cells, config
        )
        start_xyz = (*terrain.rowcol_to_xy(r1, c1), z1_msl)
        if prim is None or not evaluate_primitive(start_xyz, prim, terrain, config).valid:
            return False
    return True


_DIRECTION_BY_UNIT_DELTA: Dict[Tuple[int, int], str] = {v: k for k, v in DIRECTIONS.items()}


def _direction_for_unit_delta(drow: int, dcol: int) -> str:
    """Reverse-lookup for the CandidateZ-driven successor path.

    Step REP-1.2B: every edge was exactly one grid cell, so (drow, dcol)
    was already one of the 8 unit direction vectors.

    Step REP-1.2B.1: a vertical-changing edge may now span multiple cells
    (see primitive_for_target_altitude_over_horizon) -- but always a whole-
    number multiple of one of the 8 unit vectors (every primitive this
    module builds is a straight line in exactly one of those 8 compass
    directions, never a diagonal-of-a-diagonal), so reducing to
    (sign(drow), sign(dcol)) recovers the direction regardless of
    magnitude."""
    unit = (0 if drow == 0 else (1 if drow > 0 else -1), 0 if dcol == 0 else (1 if dcol > 0 else -1))
    try:
        return _DIRECTION_BY_UNIT_DELTA[unit]
    except KeyError:
        raise ValueError(f"({drow},{dcol}) does not reduce to one of the 8 unit step directions")


_MAX_VERTICAL_HORIZON_CELLS = 60  # Step REP-1.2B.1: practical sanity cap, see docstring below.


def _generate_candidate_z_neighbors(
    state: CanonicalState,
    terrain: TerrainQuery,
    config: PlannerConfig,
    min_search_altitude_msl: float,
    max_search_altitude_msl: float,
    distance_reference_m: Optional[float],
    candidate_z_generator: CandidateZGenerator,
    vertical_rate_cache: Optional[_SearchLocalSafeVerticalRateCache],
    aircraft_profile=None,
):
    """Step REP-1.2B: CandidateZGenerator IS the altitude successor source.
    Step REP-1.2B.1 (aircraft_profile, default None): adds a MULTI-CELL
    vertical-motion horizon on top of REP-1.2B's single-step mechanism --
    see the module-level docstring note below and project.md "Step
    REP-1.2B.1" for why the single-step-only version left real Mission A/B
    unreachable.

    For each of the 8 directions, one-grid-step candidates
    are tried -- the current altitude (level) plus every value
    candidate_z_generator.generate(new_row, new_col) returns for that
    single-step DESTINATION cell -- via primitive_for_single_step_target_
    altitude(). This block is UNCHANGED below (see _try_candidate_z_target)
    and remains active even when aircraft_profile is None, reproducing
    REP-1.2B's own behavior exactly in that case.

    With aircraft_profile given, each direction additionally grows a
    multi-cell horizon from N=2 up to the defensive cap. At every N it asks
    CandidateZ for the NEW destination cell ``(row + N*drow, col + N*dcol)``;
    destinations are not reused across N. The aircraft's local planner-safe
    climb/descent rate determines whether a candidate is reachable at that
    horizon. This is additive: it
    can only ADD successors the single-step mechanism missed (a climb/
    descent too big for one grid step but achievable over more distance),
    never remove or replace one the single-step mechanism already found --
    see project.md "Step REP-1.2B.1" section 13's DIRECTLY INFEASIBLE !=
    UNREACHABLE requirement.

    The search stops growing N for a direction after the first horizon that
    yields an accepted candidate. `_MAX_VERTICAL_HORIZON_CELLS` provides the
    upper bound when none is accepted earlier.
    """
    row, col, z_key = state
    start_altitude_msl = decode_candidate_altitude(z_key)
    start_xy = terrain.rowcol_to_xy(row, col)
    start_xyz = (start_xy[0], start_xy[1], start_altitude_msl)

    accepted: List[Tuple[CanonicalState, float]] = []
    rejected_counts: Dict[str, int] = {}
    generated = 0
    rejected = 0

    def reject(reason: str) -> None:
        nonlocal rejected
        rejected += 1
        rejected_counts[reason] = rejected_counts.get(reason, 0) + 1

    def try_candidate(direction: str, dest_row: int, dest_col: int, target_altitude: float, prim) -> None:
        """Shared validation/evaluation tail for one (direction, destination,
        target_altitude, already-built primitive) candidate -- used by both
        the REP-1.2B single-step block and the REP-1.2B.1 multi-cell block,
        so there is exactly one prefilter/safety/cost/accept sequence, not
        two drifting copies."""
        nonlocal generated
        generated += 1

        if prim is None:
            reject("angle_infeasible_for_horizon")
            return

        if not terrain.in_bounds_rowcol(dest_row, dest_col):
            reject("out_of_bounds")
            return

        if not (min_search_altitude_msl <= target_altitude <= max_search_altitude_msl):
            reject("altitude_search_bounds")
            return

        # Representability AT THE DESTINATION -- see planner/candidate_z.py's
        # is_representable() docstring; the destination's own floor/ceiling
        # may differ from wherever the candidate altitude was sourced from.
        if not candidate_z_generator.is_representable(dest_row, dest_col, target_altitude):
            reject("below_terrain_floor_sparse")
            return

        eval_result = evaluate_primitive(start_xyz, prim, terrain, config)

        if not eval_result.valid:
            reject(eval_result.reason)
            return

        edge_cost = compute_edge_cost(prim, start_altitude_msl, config, distance_reference_m)
        neighbor_state: CanonicalState = (dest_row, dest_col, encode_candidate_altitude(target_altitude))
        accepted.append((neighbor_state, edge_cost))

    for direction, (unit_drow, unit_dcol) in DIRECTIONS.items():
        # -- REP-1.2B: single grid-step, destination-sourced candidates (unchanged) --
        new_row = row + unit_drow
        new_col = col + unit_dcol

        if not terrain.in_bounds_rowcol(new_row, new_col):
            generated += 1
            reject("out_of_bounds")
        else:
            single_step_candidates = {start_altitude_msl} | set(candidate_z_generator.generate(new_row, new_col))
            for target_altitude in sorted(single_step_candidates):
                prim = primitive_for_single_step_target_altitude(direction, start_altitude_msl, target_altitude, config)
                if prim is None:
                    generated += 1
                    reject("angle_infeasible_single_step")
                    continue
                try_candidate(direction, new_row, new_col, target_altitude, prim)

        # -- REP-1.2B.1: multi-cell, destination-sourced, aircraft-capability-derived horizon --
        # Grows N (starting at 2 -- N=1 was already tried above) until the FIRST horizon at
        # which at least one of that destination's own CandidateZ candidates becomes reachable
        # at the aircraft's local planner-safe rate -- the deterministic MINIMUM horizon this
        # direction needs (Section 8), not a fixed/arbitrary cell count. Stops growing as soon
        # as one is found (Section 10 -- bounded, not "every candidate x every distance").
        if aircraft_profile is None:
            continue
        assert vertical_rate_cache is not None

        step = math.hypot(unit_drow, unit_dcol) * config.xy_resolution_m
        for n_cells in range(2, _MAX_VERTICAL_HORIZON_CELLS + 1):
            dest_row = row + unit_drow * n_cells
            dest_col = col + unit_dcol * n_cells
            if not terrain.in_bounds_rowcol(dest_row, dest_col):
                break  # farther N in this direction can only go further out of bounds

            found_at_this_horizon = False
            for target_altitude in sorted(set(candidate_z_generator.generate(dest_row, dest_col))):
                if abs(target_altitude - start_altitude_msl) < 1e-9:
                    continue  # no vertical motion needed -- already covered by "level" moves above

                mode = "CLIMB" if target_altitude > start_altitude_msl else "DESCENT"
                resolved_rate = vertical_rate_cache.get(z_key, mode)
                min_horizontal_m = derive_minimum_horizontal_distance_m(
                    start_altitude_msl, target_altitude, aircraft_profile, resolved_rate
                )
                if min_horizontal_m is None:
                    # PHYSICALLY_UNAVAILABLE / OUT_OF_DOMAIN at this altitude for this
                    # maneuver's mode -- per Step CLASS-C, no horizon can rescue this
                    # (Section 7/13). A DIFFERENT candidate (opposite sign of delta, a
                    # different mode) at this same N may still be fine, so `continue`,
                    # never `break`, out of this altitude-candidate loop.
                    continue
                required_n = max(1, math.ceil(min_horizontal_m / step - 1e-9))
                if required_n > n_cells:
                    continue  # this candidate needs more distance than N cells provides yet

                prim = primitive_for_target_altitude_over_horizon(direction, start_altitude_msl, target_altitude, n_cells, config)
                if prim is None:
                    continue

                # Authoritative aircraft-capability confirmation (Step CLASS-C contract) for
                # the CONCRETE (distance, duration) this horizon actually produces --
                # derive_minimum_horizontal_distance_m() above is a sizing helper only.
                duration_s = prim.horizontal_distance_m / aircraft_profile.manifest.nominal_ias_context_mps
                motion = evaluate_vertical_motion(
                    start_altitude_msl, target_altitude, duration_s, aircraft_profile, resolved_rate
                )
                if motion.status != "FEASIBLE":
                    continue

                try_candidate(direction, dest_row, dest_col, target_altitude, prim)
                found_at_this_horizon = True

            if found_at_this_horizon:
                break  # deterministic minimum horizon found for this direction -- stop growing N

    return accepted, rejected_counts, generated, rejected


def _reconstruct_path(came_from: Dict[CanonicalState, CanonicalState], start: CanonicalState, goal_state: CanonicalState) -> List[CanonicalState]:
    path = [goal_state]
    current = goal_state
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _path_altitude_metrics(
    path: List[CanonicalState],
    terrain: TerrainQuery,
) -> Dict[str, float]:
    """Unweighted geometric length, MSL altitude stats, and climb/descent
    totals for a found path. Purely descriptive now -- not cost inputs.

    average_aircraft_msl is weighted by each edge's geometric length (not a
    plain per-node average) so a long low-altitude leg actually outweighs a
    short high one -- otherwise sparse high-altitude waypoints could skew
    the average away from what was actually flown.
    """
    xyz = [state_to_xyz(s, terrain) for s in path]
    altitudes = [p[2] for p in xyz]

    if len(xyz) < 2:
        z = altitudes[0] if altitudes else float("nan")
        return {"geometric_path_length": 0.0, "minimum_aircraft_msl": z,
                "maximum_aircraft_msl": z, "average_aircraft_msl": z,
                "total_climb_m": 0.0, "total_descent_m": 0.0, "total_vertical_motion_m": 0.0}

    total_length = 0.0
    weighted_alt_sum = 0.0
    total_climb_m = 0.0
    total_descent_m = 0.0
    for (x1, y1, z1), (x2, y2, z2) in zip(xyz, xyz[1:]):
        seg_len = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        total_length += seg_len
        weighted_alt_sum += seg_len * (z1 + z2) / 2.0
        dz = z2 - z1
        if dz > 0:
            total_climb_m += dz
        elif dz < 0:
            total_descent_m += -dz

    average_msl = weighted_alt_sum / total_length if total_length > 0 else altitudes[0]
    return {
        "geometric_path_length": total_length,
        "minimum_aircraft_msl": min(altitudes),
        "maximum_aircraft_msl": max(altitudes),
        "average_aircraft_msl": average_msl,
        "total_climb_m": total_climb_m,
        "total_descent_m": total_descent_m,
        "total_vertical_motion_m": total_climb_m + total_descent_m,
    }


def _path_min_observed_agl(
    path: List[CanonicalState],
    terrain: TerrainQuery,
    config: PlannerConfig,
) -> float:
    """Re-derive minimum AGL by rebuilding each explicit CandidateZ edge."""
    if len(path) < 2:
        return float("nan")
    min_agl = math.inf
    for s1, s2 in zip(path, path[1:]):
        r1, c1, _ = s1
        r2, c2, _ = s2
        z1_msl = decode_candidate_altitude(s1[2])
        z2_msl = decode_candidate_altitude(s2[2])
        direction = _direction_for_unit_delta(r2 - r1, c2 - c1)
        n_cells = max(abs(r2 - r1), abs(c2 - c1))
        prim = primitive_for_target_altitude_over_horizon(
            direction, z1_msl, z2_msl, n_cells, config
        )
        if prim is None:
            continue
        start_xyz = (*terrain.rowcol_to_xy(r1, c1), z1_msl)
        result = evaluate_primitive(start_xyz, prim, terrain, config)
        min_agl = min(min_agl, result.min_agl_m)
    return min_agl if min_agl != math.inf else float("nan")


def _current_lower_bound(lb_heap: List[Tuple[float, int, CanonicalState]], closed: set) -> float:
    """Stage 25: the minimum g+h (unweighted, admissible) among states still
    PENDING (not yet closed/resolved) in lb_heap -- a safe lower bound on the
    true remaining optimal cost C*, used for the %-suboptimality certificate.

    lb_heap holds one entry per accepted candidate push (paired 1:1 with
    open_heap's own pushes, sharing the same counter values), MINUS whatever
    dominance/incumbent pruning already excluded before either push -- both
    exclusions are safe here too (see astar_search's docstring): a
    dominance-pruned candidate's own future cost is provably >= the state
    that dominates it, and an incumbent-pruned candidate's f_lb is already
    >= incumbent_cost >= C*, so neither could ever have been the true
    minimum in the first place. Entries for states that have since been
    closed are lazily, permanently discarded (they're resolved, no longer
    "pending") -- the top of the heap, once cleaned, IS the answer; it is
    only peeked, never consumed, so repeated calls stay valid.
    """
    while lb_heap:
        f_lb, _, state = lb_heap[0]
        if state in closed:
            heapq.heappop(lb_heap)
            continue
        return f_lb
    return math.inf


def astar_search(
    start: CanonicalState,
    goal: CanonicalState,
    terrain: TerrainQuery,
    min_search_altitude_msl: float,
    max_search_altitude_msl: float,
    candidate_z_generator: CandidateZGenerator,
    config: PlannerConfig = DEFAULT_CONFIG,
    max_expansions: Optional[int] = None,
    max_search_time_s: Optional[float] = None,
    use_msl_lower_bound_heuristic: bool = True,
    use_vertical_reachability_heuristic: bool = True,
    use_incumbent_pruning: bool = False,
    initial_incumbent_cost: float = math.inf,
    initial_incumbent_path: Optional[List[CanonicalState]] = None,
    epsilon_search: float = 1.0,
    target_suboptimality: Optional[float] = None,
    stop_on_first_solution: bool = False,
    aircraft_profile=None,
) -> SearchResult:
    """3D A* from start to goal using only the existing safe motion primitives.

    start/goal/every search state are ``(row, col, altitude_id)``. The
    altitude ID always decodes through CandidateZ's exact encoding.

    min_search_altitude_msl / max_search_altitude_msl bound the prototype
    search space -- they are NOT an aircraft flight ceiling. A neighbor
    outside this range is never generated.

    max_search_time_s (default None, Step PERF-0): a wall-clock budget on
    this call's ONLINE search loop only -- measured from the first
    instruction inside this function, so it never includes whatever the
    caller did to build terrain or the required CandidateZGenerator before
    calling this. Checked once per outer loop
    iteration (so it also catches a run that is spinning on incumbent-skip
    "continue"s without incrementing expanded_nodes, not just one that is
    genuinely expanding). None (the default) means no time budget --
    reproduces pre-Step-PERF-0 behavior exactly. This is a development/
    regression watchdog, not a claim about acceptable production search
    latency -- see project.md "Step PERF-0". A budget firing sets
    status="timeout" / termination_reason="TIMEOUT" -- this NEVER means
    the goal is unreachable, only that this call's budget ran out; the
    same is true of max_expansions -> "search_limit_reached"/
    "EXPANSION_LIMIT". Only "no_path"/"NO_PATH" (open_heap exhausted with
    no incumbent) is a genuine negative result within the given bounds.

    use_msl_lower_bound_heuristic (default True): scale the plain-Euclidean
    heuristic by a single search-wide constant reflecting the cheapest MSL
    this search could ever fly at (see _min_possible_aircraft_msl /
    _msl_lower_bound_multiplier) -- still a strict lower bound on the true
    remaining cost, never an overestimate, so A* optimality is unaffected;
    it only tightens how much of the real MSL cost the heuristic already
    "knows about" before expanding anything. Falls back to multiplier=1.0
    (plain Euclidean) automatically if the current config's weights don't
    satisfy this bound's non-negativity assumptions. Set False to
    reproduce the pre-Stage-17 heuristic exactly.

    use_vertical_reachability_heuristic (default True): additionally take
    the max with a max-descent-rate-aware forward envelope (see
    _forward_vertical_reachability_heuristic) -- tighter than the plain
    global-floor bound above when the current altitude is far above the
    floor and/or far from the goal, since it accounts for how much
    distance descending that far would actually require. Also a strict
    lower bound (proven admissible and consistent independently -- see
    that function's docstring); combining via max() with the bound above
    preserves both properties. Uses only config.max_descent_angle_deg
    (never a hardcoded angle) plus the same MSL cost parameters -- no
    terrain sampling, O(1) per call. Set False to reproduce the
    pre-Stage-21 heuristic exactly.

    use_incumbent_pruning (default False, Stage 23): exact branch-and-bound
    against a known-valid complete solution's cost (the "incumbent" --
    caller-supplied via initial_incumbent_cost/initial_incumbent_path after
    validation against the same CandidateZ edge and safety contracts; this
    function is agnostic to how that path was obtained).
    Since the heuristic is admissible, g(n)+h(n) is a lower bound on the
    cost of ANY complete solution passing through n; if that lower bound
    is already >= the incumbent's cost, n's branch cannot possibly beat
    the incumbent, so it is safe to discard (never explored) without
    losing the true optimum -- this is exact, not an approximation.
    Pruning is applied at candidate-generation time (a neighbor whose
    f=g+h already exceeds the incumbent is never pushed to the open heap
    -- incumbent_pruned_candidates) and independently at heap-pop time (a
    popped, non-stale entry whose own f exceeds the incumbent is not
    expanded -- incumbent_heap_pops_skipped); a candidate can only ever be
    counted at one of the two points, never both, since a generation-time
    prune means it's never pushed at all. Because entries pop off a
    min-heap in nondecreasing f order, the FIRST live (non-closed) pop
    whose f >= incumbent proves every remaining open entry does too --
    the search terminates right there (incumbent_termination_triggered)
    and reports the incumbent's own path/cost as the already-proven-
    optimal answer, rather than draining the rest of a possibly enormous
    heap one skip at a time. If the search itself later pops a genuine
    goal state with a strictly better g than the current incumbent, the
    incumbent is tightened (incumbent_updates) before the existing
    goal-pop termination (see above) returns it -- this is the same
    consistent-A*-optimal termination already in place, just now also
    capable of using a caller-supplied head start. If initial_incumbent
    _cost is math.inf (no usable initial incumbent), pruning simply never
    fires until/unless the search finds its own solution first, which
    -- as with all the flags above -- means this reproduces the pre-
    Stage-23 search exactly.

    epsilon_search (default 1.0), target_suboptimality (default None,
    Stage 25): Weighted A* with a certified bounded-suboptimality
    termination. TWO separate f-values exist per candidate: f_weighted =
    g + epsilon_search*h drives SEARCH ORDERING ONLY (which state gets
    expanded next -- open_heap's own priority); f_lb = g + h (unweighted,
    still admissible since h itself never changed) is the only value ever
    used for incumbent pruning, the %-suboptimality certificate, or any
    other safety/quality claim. epsilon_search >= 1.0 biases expansion
    toward the goal (classic Weighted A*); it is NEVER used for pruning or
    bounds, only for ordering -- see section 9 of the Stage 25 spec for
    why g+epsilon*h is unsafe there (it is not a valid lower bound on
    remaining cost once epsilon>1).

    Because f_weighted is not consistent for epsilon_search>1, a state
    already closed can later be reached with a strictly better g (the
    weighted order doesn't guarantee non-decreasing f_lb along the actual
    pop sequence the way pure f_lb ordering does) -- so closed membership
    is no longer permanent: whenever tentative_g < g_score.get(state,inf)
    for a state already in `closed`, it is removed from closed and
    reopened (reopened_states) with the improved g. This is provably inert
    when epsilon_search==1.0 (consistency guarantees no closed state can
    ever be improved), so it changes nothing about the exact/pre-Stage-25
    behavior -- it's the same code path, just never triggered there.

    target_suboptimality (e.g. 1.05) enables a running lower-bound
    tracker (lb_heap, parallel to open_heap, storing f_lb instead of
    f_weighted) so the search can compute, at any point, a safe LB = the
    minimum f_lb among all still-pending (non-closed) states -- proven
    <= the true remaining optimal cost C* by the same admissibility
    argument as always, now applied to the WHOLE frontier rather than a
    single popped node. Since incumbent_cost is always an achievable,
    validated real solution, incumbent_cost >= C* always; so the moment
    incumbent_cost <= target_suboptimality * LB, it follows algebraically
    that incumbent_cost / C* <= target_suboptimality -- a certified bound,
    not a guess, and NOT simply trusting epsilon_search's own (much
    looser) worst-case ratio. When target_suboptimality is set, the search
    does NOT stop at the first goal pop (unlike every mode above) --
    finding a goal state only updates the incumbent (first_solution_*
    recorded on the very first one) and the search keeps going, checking
    the certificate after every expansion, until either it fires
    (bounded_termination_triggered) or open_heap empties (meaning nothing
    is left to explore, so whatever incumbent exists -- if any -- is
    exactly optimal) or max_expansions is hit (search_limit_reached, but
    the best incumbent/LB/gap found so far are still reported). When
    target_suboptimality is None, behavior reverts to whatever the other
    flags already specify (stop at first goal pop, exactly as in every
    earlier stage) -- epsilon_search alone, without a target, just makes
    that first solution biased/found differently, with no certificate
    computed.

    stop_on_first_solution (default False, Stage 37.4): when True, the
    search breaks IMMEDIATELY at the first goal-region pop, exactly like
    the non-bounded-mode path already does -- even if target_
    suboptimality is set (bounded_mode), no certificate is pursued.
    Every running counter (expanded_nodes, max_open_size,
    reopened_states, generated_neighbors, ...) therefore reflects
    exactly the state at the moment the first complete solution was
    found, with nothing extra explored afterward -- no separate
    "snapshot" fields are needed for that. False (the default)
    reproduces the pre-Stage-37.4 behavior exactly (bounded_mode, if
    active, keeps searching for a tighter certified bound as before).

    candidate_z_generator is required. CandidateZ events are the sole
    altitude-successor source and exact event altitudes use the canonical
    CandidateZ state encoding. This function does not construct a generator.

    aircraft_profile (default None): when supplied, _generate_candidate_z_
    neighbors() additionally tries a multi-cell vertical-motion
    horizon derived from this profile's real planner-safe climb/descent
    rate (see that function's own docstring and planner.vertical_motion) --
    purely ADDITIVE on top of REP-1.2B's single-grid-step mechanism, never
    a replacement for it. None (the default) reproduces REP-1.2B's own
    behavior exactly.
    """
    if not isinstance(candidate_z_generator, CandidateZGenerator):
        raise TypeError("candidate_z_generator must be a CandidateZGenerator")

    t0 = time.perf_counter()
    counter = itertools.count()

    vertical_rate_cache = (
        _SearchLocalSafeVerticalRateCache(aircraft_profile)
        if aircraft_profile is not None else None
    )

    # Computed ONCE per search call, never inside the loop below.
    min_possible_msl = float("nan")
    cost_multiplier = 1.0
    vertical_reachability_active = False
    if use_msl_lower_bound_heuristic and config.cost_mode == "legacy":
        # Stage 32: this entire legacy MSL-lower-bound/vertical-reachability
        # machinery is specific to the legacy cost formula's own admissibility
        # proof -- skipped for cost_mode=="normalized", which uses its own,
        # deliberately minimal heuristic (see _heuristic's docstring).
        min_possible_msl = _min_possible_aircraft_msl(
            _terrain_min_valid_elevation(terrain), min_search_altitude_msl, config
        )
        multiplier = _msl_lower_bound_multiplier(min_possible_msl, config)
        if multiplier is not None:
            cost_multiplier = multiplier
            vertical_reachability_active = use_vertical_reachability_heuristic
        # else: config has a negative weight/non-positive scale -- stay at 1.0 (safe fallback),
        # and the vertical-reachability envelope stays off too (same non-negativity assumptions).
    heuristic_min_msl = min_possible_msl if vertical_reachability_active else None

    # Stage 32: D_ref, computed ONCE per search call (never per-edge/per-state),
    # only when cost_mode=="normalized" -- None (and unused) under "legacy".
    distance_reference_m = (
        compute_distance_reference(start, goal, terrain)
        if config.cost_mode == "normalized" else None
    )

    incumbent_cost = initial_incumbent_cost if use_incumbent_pruning else math.inf
    incumbent_path: List[CanonicalState] = (
        list(initial_incumbent_path) if (use_incumbent_pruning and initial_incumbent_path) else []
    )
    incumbent_stats = {"updates": 0, "pruned_candidates": 0, "heap_pops_skipped": 0}
    incumbent_termination_triggered = False

    bounded_mode = target_suboptimality is not None
    reopened_states = 0
    bounded_termination_triggered = False
    first_solution_cost = math.inf
    first_solution_expanded = 0
    first_solution_runtime_s = float("nan")
    # Stage 33: goal center in real UTM meters, computed once (never per-expansion),
    # for the closest-state-to-goal diagnostics below.
    goal_x, goal_y, goal_z = state_to_xyz(goal, terrain)
    closest_distance_to_goal_region_m = math.inf
    closest_distance_to_goal_center_m = math.inf
    closest_state_to_goal: Optional[CanonicalState] = None

    g_score: Dict[CanonicalState, float] = {start: 0.0}
    came_from: Dict[CanonicalState, CanonicalState] = {}
    closed = set()

    h_start = _heuristic(start, goal, terrain, config, cost_multiplier, heuristic_min_msl,
                         vertical_reachability_active, distance_reference_m)
    start_counter = next(counter)
    open_heap = [(epsilon_search * h_start, start_counter, start, h_start)]
    lb_heap: List[Tuple[float, int, CanonicalState]] = [(h_start, start_counter, start)] if bounded_mode else []
    max_open_size = 1

    expanded_nodes = 0
    generated_neighbors = 0
    rejected_neighbors = 0
    rejected_reason_counts: Dict[str, int] = {}
    status = "no_path"
    goal_state: Optional[CanonicalState] = None

    while open_heap:
        if max_search_time_s is not None and (time.perf_counter() - t0) >= max_search_time_s:
            status = "timeout"
            break

        max_open_size = max(max_open_size, len(open_heap))
        f_weighted_current, _, current, f_lb_current = heapq.heappop(open_heap)

        if current in closed:
            continue  # stale entry -- a better copy of this state was already expanded

        if use_incumbent_pruning and f_lb_current >= incumbent_cost:
            # Point B (heap-pop time) -- always checked against the UNWEIGHTED f_lb,
            # never f_weighted (see astar_search's Stage 25 docstring section 9).
            incumbent_stats["heap_pops_skipped"] += 1
            if epsilon_search == 1.0 and target_suboptimality is None:
                # Exact Stage 24 behavior: when epsilon_search==1.0, f_weighted==f_lb,
                # so open_heap's own pop order IS nondecreasing in f_lb -- this proves
                # every remaining entry also fails, so terminate now rather than
                # draining the rest one skip at a time. This shortcut is NOT valid
                # once epsilon_search>1 (weighted pop order no longer tracks f_lb), so
                # it stays gated to exactly the pre-Stage-25 case it was proven for.
                incumbent_termination_triggered = True
                status = "success"
                break
            continue  # weighted/bounded mode: only this candidate is proven inferior

        closed.add(current)
        expanded_nodes += 1

        x_c, y_c, z_c = state_to_xyz(current, terrain)
        region_dist = _distance_to_goal_box(x_c, y_c, z_c, goal_x, goal_y, goal_z,
                                             config.goal_tolerance_xy_m, config.goal_tolerance_z_m)
        if region_dist < closest_distance_to_goal_region_m:
            closest_distance_to_goal_region_m = region_dist
            closest_state_to_goal = current
        center_dist = math.sqrt((x_c - goal_x) ** 2 + (y_c - goal_y) ** 2 + (z_c - goal_z) ** 2)
        if center_dist < closest_distance_to_goal_center_m:
            closest_distance_to_goal_center_m = center_dist

        if _state_in_goal_region(current, goal, terrain, config):
            solution_cost = g_score[current]
            if first_solution_cost == math.inf:
                first_solution_cost = solution_cost
                first_solution_expanded = expanded_nodes
                first_solution_runtime_s = time.perf_counter() - t0
            if use_incumbent_pruning and solution_cost < incumbent_cost:
                incumbent_cost = solution_cost
                incumbent_stats["updates"] += 1
                incumbent_path = _reconstruct_path(came_from, start, current)
            if not bounded_mode or stop_on_first_solution:
                status = "success"
                goal_state = current
                break
            # bounded_mode (and stop_on_first_solution is False): don't stop here --
            # a tighter certified solution may still be reachable; fall through to
            # the certificate check below.

        if bounded_mode and incumbent_cost < math.inf:
            current_lb = _current_lower_bound(lb_heap, closed)
            if current_lb < math.inf and incumbent_cost <= target_suboptimality * current_lb:
                bounded_termination_triggered = True
                status = "success"
                break

        if max_expansions is not None and expanded_nodes >= max_expansions:
            status = "search_limit_reached"
            break

        neighbors, rej_counts, gen_count, rej_count = _generate_candidate_z_neighbors(
            current, terrain, config, min_search_altitude_msl, max_search_altitude_msl,
            distance_reference_m,
            candidate_z_generator, vertical_rate_cache, aircraft_profile,
        )
        generated_neighbors += gen_count
        rejected_neighbors += rej_count
        for reason, cnt in rej_counts.items():
            rejected_reason_counts[reason] = rejected_reason_counts.get(reason, 0) + cnt

        current_g = g_score[current]
        for neighbor_state, edge_cost in neighbors:
            tentative_g = current_g + edge_cost
            if tentative_g < g_score.get(neighbor_state, math.inf):
                if neighbor_state in closed:
                    # Reopen: with epsilon_search>1 the weighted pop order is not
                    # guaranteed nondecreasing in f_lb, so a strictly better g CAN
                    # arrive after a state was already closed (see astar_search's
                    # Stage 25 docstring). Provably inert when epsilon_search==1.0
                    # (consistency already guarantees this branch is never taken
                    # there), so this changes nothing about the exact-mode search.
                    closed.discard(neighbor_state)
                    reopened_states += 1
                g_score[neighbor_state] = tentative_g
                came_from[neighbor_state] = current

                h_val = _heuristic(neighbor_state, goal, terrain, config, cost_multiplier,
                                   heuristic_min_msl, vertical_reachability_active, distance_reference_m)
                f_lb = tentative_g + h_val  # admissible, unweighted -- the ONLY value used for pruning/bounds
                f_weighted = tentative_g + epsilon_search * h_val  # search ORDERING only, never a bound
                if use_incumbent_pruning and f_lb >= incumbent_cost:
                    # Point A (candidate-generation time), always against f_lb -- see
                    # astar_search's Stage 25 docstring section 9 for why f_weighted
                    # would be unsafe here. If it can't beat the incumbent, never push
                    # it (never counted at point B too, since a candidate pruned here
                    # is never pushed at all).
                    incumbent_stats["pruned_candidates"] += 1
                else:
                    c = next(counter)
                    heapq.heappush(open_heap, (f_weighted, c, neighbor_state, f_lb))
                    if bounded_mode:
                        heapq.heappush(lb_heap, (f_lb, c, neighbor_state))

    runtime_s = time.perf_counter() - t0

    # Stage 37.1: state-space composition diagnostics, derived once from the final
    # `closed` set (every state this search actually expanded) -- cheap
    # (one pass over at most max_expansions entries), always computed, never
    # affects search behavior. Since Step CLEAN-1, a search state IS its own
    # physical (row,col,z) -- no history dimension can multiply it -- so
    # unique_expanded_xyz == len(closed) exactly (kept as its own field
    # anyway, purely for continuity with per-(row,col) Z diversity below).
    xy_seen: Dict[Tuple[int, int], set] = {}
    for (r, c, z) in closed:
        xy_seen.setdefault((r, c), set()).add(z)
    unique_expanded_xy = len(xy_seen)
    unique_expanded_xyz = len(closed)
    avg_z_states_per_xy = unique_expanded_xyz / unique_expanded_xy if unique_expanded_xy else float("nan")
    max_z_states_in_one_xy = max((len(v) for v in xy_seen.values()), default=0)

    if status == "no_path" and incumbent_cost < math.inf:
        # open_heap ran out with nothing left unexplored -- the search is
        # complete, so whatever incumbent stands (initial or improved) is
        # exactly optimal within this graph, proven by exhaustion rather
        # than by the certificate or the exact-mode goal-pop.
        status = "success"

    current_lower_bound = _current_lower_bound(lb_heap, closed) if bounded_mode else float("nan")
    final_bound_ratio = (
        incumbent_cost / current_lower_bound
        if bounded_mode and incumbent_cost < math.inf and 0.0 < current_lower_bound < math.inf
        else float("nan")
    )

    vertical_lookups = vertical_rate_cache.lookups if vertical_rate_cache is not None else 0
    vertical_hits = vertical_rate_cache.hits if vertical_rate_cache is not None else 0
    vertical_misses = vertical_rate_cache.misses if vertical_rate_cache is not None else 0
    vertical_hit_rate = vertical_hits / vertical_lookups if vertical_lookups else float("nan")
    profile_query_time_s = (
        vertical_rate_cache.profile_query_time_s if vertical_rate_cache is not None else 0.0
    )

    if status == "success" and goal_state is not None:
        path = _reconstruct_path(came_from, start, goal_state)
        total_cost = g_score[goal_state]
        alt_metrics = _path_altitude_metrics(path, terrain)
        min_observed_agl = _path_min_observed_agl(path, terrain, config)
    elif status == "success" and goal_state is None:
        # The search proved the incumbent is (certifiably, or by exhaustion, or by
        # the exact-mode min-heap pop-order argument) at least as good as anything
        # left to explore, without ever itself popping a goal state. incumbent_path
        # already holds whichever is correct: the caller's initial_incumbent_path
        # if never internally improved, or the reconstructed better path from the
        # goal-pop(s) that updated it (see the loop above) otherwise.
        path = incumbent_path
        total_cost = incumbent_cost
        alt_metrics = _path_altitude_metrics(path, terrain)
        min_observed_agl = _path_min_observed_agl(path, terrain, config)
    else:
        path = []
        total_cost = float("nan")
        alt_metrics = {"geometric_path_length": float("nan"), "minimum_aircraft_msl": float("nan"),
                       "maximum_aircraft_msl": float("nan"), "average_aircraft_msl": float("nan"),
                       "total_climb_m": float("nan"), "total_descent_m": float("nan"),
                       "total_vertical_motion_m": float("nan")}
        min_observed_agl = float("nan")

    return SearchResult(
        success=(status == "success"),
        status=status,
        termination_reason=_TERMINATION_REASON_BY_STATUS[status],
        path=path,
        total_cost=total_cost,
        expanded_nodes=expanded_nodes,
        generated_neighbors=generated_neighbors,
        rejected_neighbors=rejected_neighbors,
        rejected_reason_counts=rejected_reason_counts,
        max_open_size=max_open_size,
        runtime_s=runtime_s,
        geometric_path_length=alt_metrics["geometric_path_length"],
        minimum_aircraft_msl=alt_metrics["minimum_aircraft_msl"],
        maximum_aircraft_msl=alt_metrics["maximum_aircraft_msl"],
        average_aircraft_msl=alt_metrics["average_aircraft_msl"],
        minimum_observed_agl=min_observed_agl,
        total_climb_m=alt_metrics["total_climb_m"],
        total_descent_m=alt_metrics["total_descent_m"],
        total_vertical_motion_m=alt_metrics["total_vertical_motion_m"],
        vertical_rate_cache_lookups=vertical_lookups,
        vertical_rate_cache_hits=vertical_hits,
        vertical_rate_cache_misses=vertical_misses,
        vertical_rate_cache_hit_rate=vertical_hit_rate,
        aircraft_profile_vertical_queries=vertical_misses,
        aircraft_profile_vertical_query_time_s=profile_query_time_s,
        minimum_possible_aircraft_msl=min_possible_msl,
        heuristic_cost_multiplier=cost_multiplier,
        initial_incumbent_available=use_incumbent_pruning and initial_incumbent_cost < math.inf,
        initial_incumbent_cost=initial_incumbent_cost if use_incumbent_pruning else math.inf,
        final_incumbent_cost=incumbent_cost,
        incumbent_updates=incumbent_stats["updates"],
        incumbent_pruned_candidates=incumbent_stats["pruned_candidates"],
        incumbent_heap_pops_skipped=incumbent_stats["heap_pops_skipped"],
        incumbent_termination_triggered=incumbent_termination_triggered,
        incumbent_path=incumbent_path,
        epsilon_search=epsilon_search,
        target_suboptimality=target_suboptimality,
        first_solution_cost=first_solution_cost,
        first_solution_expanded=first_solution_expanded,
        first_solution_runtime_s=first_solution_runtime_s,
        current_lower_bound=current_lower_bound,
        final_bound_ratio=final_bound_ratio,
        bounded_termination_triggered=bounded_termination_triggered,
        reopened_states=reopened_states,
        cost_mode=config.cost_mode,
        distance_reference_m=distance_reference_m if distance_reference_m is not None else float("nan"),
        goal_tolerance_xy_m=config.goal_tolerance_xy_m,
        goal_tolerance_z_m=config.goal_tolerance_z_m,
        closest_distance_to_goal_region_m=closest_distance_to_goal_region_m,
        closest_distance_to_goal_center_m=closest_distance_to_goal_center_m,
        closest_state_to_goal=closest_state_to_goal,
        unique_expanded_xy=unique_expanded_xy,
        unique_expanded_xyz=unique_expanded_xyz,
        avg_z_states_per_xy=avg_z_states_per_xy,
        max_z_states_in_one_xy=max_z_states_in_one_xy,
    )
