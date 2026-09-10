"""Terrain-aware 3D A* over the fixed motion-primitive set.

Physical state is integer (row, col, z_index) -- never raw float (x, y, z)
-- so the same physical state can't appear twice in the search due to
floating-point drift. z_index maps to altitude via z_msl = z_index *
config.z_step_m; conversion is explicit (msl_to_z_index) and never snaps
silently.

The search itself runs over an AUGMENTED state (row, col, z_index,
vertical_trend, trend_age_units), because the vertical-direction-reversal
cost (see below) depends on how long the aircraft has been climbing or
descending, not just where it is or which direction it was last going.
trend_age_units is a bounded, discretized (never raw float) memory of how
far the standing trend has run, in trend_age_unit_m increments, capped at
ceil(reversal_relax_distance_m / trend_age_unit_m) -- long enough to reach
"this reversal is free" without letting the state space grow unbounded.
start/goal, and SearchResult.path, are still plain physical (row, col,
z_index) tuples -- the goal condition only checks physical position, never
trend/age (see astar_search).

Neighbor generation is lazy: the 24-primitive set is tried against every
expanded state on demand, nothing is precomputed into a graph. All terrain,
AGL and climb/descent safety logic is delegated to the existing modules
(planner.primitives.evaluate_primitive, which itself uses planner.agl and
planner.transition) -- this module does not reimplement any of it.

Edge cost = plain 3D geometric primitive length, scaled by a soft low-MSL
altitude preference, plus a soft, SPACING-SENSITIVE penalty on
vertical-direction reversal (climb->descent or descent->climb -- see
_altitude_scaled / msl_cost_weight and vertical_reversal_cost_weight /
reversal_relax_distance_m below). A continuous descent or continuous climb
pays no vertical penalty at all, no matter how long. A reversal's cost
scales down the longer the trend it's reversing had been running, reaching
zero once that trend has covered reversal_relax_distance_m -- a reversal
driven by genuine large-scale terrain structure (mountain -> valley ->
mountain) is cheap or free; a reversal seconds after the last one (a
short-spaced zigzag) is expensive. This replaces BOTH the earlier
abs(delta_z)-per-primitive penalty AND the even earlier special-cased
"first descent->climb reversal is free" rule -- neither is part of the
production cost anymore (see project.md "Stage 12" / "Stage 14"). No
heading/turn-radius, no weighted-A* heuristic inflation.

MSL cost uses a FIXED reference/scale (config.msl_reference_m /
msl_scale_m), not the search call's min/max_search_altitude_msl -- see
project.md "Stage 10" for why.

AGL, and the max climb/descent angle, remain HARD constraints enforced
entirely by evaluate_primitive() / evaluate_agl() / evaluate_transition()
-- MSL altitude and vertical-reversal are only ever SOFT cost preferences
layered on top of edges that are already safe. Neither can make the
planner take an edge evaluate_primitive() rejected, and neither can stop
it from climbing when climbing is the only safe option -- they only ever
make an already-safe edge cost more, never forbid it. In particular,
max_climb_angle_deg / max_descent_angle_deg are an AIRCRAFT flight-path
angle limit, not a terrain-slope limit: terrain can be steeper than that
angle, and the planner simply starts climbing/descending earlier (using
more horizontal distance) to stay within it -- it is never required to
match the terrain's own slope.

Future cost architecture (not implemented here, noted for later stages):
    distance + low-MSL preference + spacing-sensitive reversal penalty
    (this stage)
    + optional preferred-AGL / terrain-following mode (separate future,
      opt-in operating mode -- a different objective than "prefer low
      absolute MSL", not a variant of it)
"""
import heapq
import itertools
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.mission import mission_policy_from_config
from planner.primitives import MotionPrimitive, Point3, PrimitiveEvalResult, build_primitive_set, evaluate_primitive
from planner.terrain import TerrainQuery

CanonicalState = Tuple[int, int, int]  # (row, col, z_index) -- physical, public (start/goal/path)
# (row, col, z_index, vertical_trend, trend_age_bucket) -- internal search state. trend_age_bucket
# is one of BUCKET_SHORT/BUCKET_MEDIUM/BUCKET_MATURE (Stage 22) -- see _next_trend_and_bucket.
AugmentedState = Tuple[int, int, int, int, int]
# (row, col, z_index, primitive_id) -- physical feasibility cache key. Deliberately excludes
# vertical_trend/trend_age_units: whether a primitive is physically safe from a given (row,col,z)
# never depends on how the aircraft got there (see evaluate_primitive() -- it only ever looks at
# terrain, AGL, and the primitive's own geometry).
PrimitiveCacheKey = Tuple[int, int, int, Tuple[int, int, float, str]]
PrimitiveId = Tuple[int, int, float, str]  # (drow, dcol, dz_m, primitive_type) -- stable, canonical
# (row, col, z_index, vertical_trend) -- dominance comparison group. Two augmented
# states only ever get compared for dominance if they share this key (see astar_search).
BaseKey = Tuple[int, int, int, int]


# --------------------------------------------------------------------------
# State <-> altitude conversion. Snapping is always explicit, never silent.
# --------------------------------------------------------------------------

def msl_to_z_index(z_msl: float, config: PlannerConfig = DEFAULT_CONFIG, allow_snap: bool = False) -> int:
    """MSL altitude -> z_index on the z_step_m grid.

    Raises ValueError if z_msl isn't (numerically) on the grid, unless
    allow_snap=True -- in which case it rounds to the nearest index. The
    caller decides to snap; this never happens implicitly inside search.
    """
    raw = z_msl / config.z_step_m
    nearest = round(raw)
    if not allow_snap and abs(raw - nearest) > 1e-6:
        raise ValueError(
            f"z_msl={z_msl} is not aligned to the {config.z_step_m}m z-grid "
            f"(nearest index {nearest} -> {nearest * config.z_step_m}m msl); "
            f"pass allow_snap=True to snap explicitly"
        )
    return int(nearest)


def z_index_to_msl(z_index: int, config: PlannerConfig = DEFAULT_CONFIG) -> float:
    return z_index * config.z_step_m


def state_to_xyz(state: CanonicalState, terrain: TerrainQuery, config: PlannerConfig = DEFAULT_CONFIG) -> Point3:
    row, col, z_index = state
    x, y = terrain.rowcol_to_xy(row, col)
    return (x, y, z_index_to_msl(z_index, config))


def path_to_xyz(path: List[CanonicalState], terrain: TerrainQuery, config: PlannerConfig = DEFAULT_CONFIG) -> List[Point3]:
    return [state_to_xyz(s, terrain, config) for s in path]


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
    passing evaluate_primitive() inside _generate_neighbors (AGL, terrain
    collision, NoData, bounds, climb/descent angle) -- an unsafe state is
    never even a candidate here, regardless of tolerance, so a goal box
    can never "rescue" an otherwise-invalid state into being a solution.
    """
    if config.goal_tolerance_xy_m == 0.0 and config.goal_tolerance_z_m == 0.0:
        return state == goal
    x1, y1, z1 = state_to_xyz(state, terrain, config)
    x2, y2, z2 = state_to_xyz(goal, terrain, config)
    return (
        abs(x1 - x2) <= config.goal_tolerance_xy_m
        and abs(y1 - y2) <= config.goal_tolerance_xy_m
        and abs(z1 - z2) <= config.goal_tolerance_z_m
    )


# --------------------------------------------------------------------------
# Search result
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class SearchResult:
    success: bool
    status: str  # "success" | "no_path" | "search_limit_reached"
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
    # Reversal/spacing metrics.
    total_vertical_reversal_count: int = 0  # every climb<->descent direction flip
    penalized_reversal_count: int = 0  # reversals with a non-zero (spacing < relax distance) penalty
    zero_penalty_long_spacing_reversal_count: int = 0  # reversals that came after >= relax distance -- effectively free
    minimum_reversal_spacing_m: float = float("nan")  # shortest standing-trend distance among this path's reversals
    average_reversal_spacing_m: float = float("nan")  # mean standing-trend distance among this path's reversals
    total_reversal_penalty: float = float("nan")  # sum of the (spacing-scaled) reversal costs
    # Physical-primitive-feasibility cache stats (see PrimitiveCacheKey) -- 0/nan when
    # use_primitive_cache=False, since there's no cache to report hits/misses for.
    primitive_cache_hits: int = 0
    primitive_cache_misses: int = 0
    primitive_cache_hit_rate: float = float("nan")
    actual_evaluate_primitive_calls: int = 0
    avoided_evaluate_primitive_calls: int = 0  # == primitive_cache_hits, reported separately per spec
    # Exact dominance pruning stats (see BaseKey) -- 0 when use_dominance_pruning=False.
    dominance_checks: int = 0  # candidates that reached the dominance-vs-frontier test
    dominance_pruned_candidates: int = 0  # candidates dominated by an existing frontier entry -- never pushed to open
    dominance_frontier_entries_removed: int = 0  # existing entries dropped because a new candidate dominated them
    dominated_heap_pops_skipped: int = 0  # heap pops whose (age, g) was no longer active in its frontier
    max_dominance_frontier_size: int = 0  # largest per-base-key frontier list seen during the whole search
    average_dominance_frontier_size: float = float("nan")  # mean size of all non-empty frontiers at search end
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
    closest_state_to_goal: Optional[CanonicalState] = None  # the (row,col,z_index) that achieved the region minimum
    # Corridor-constrained search (Stage 37) -- 0 when corridor_mask=None (no corridor check ever ran).
    corridor_reject_count: int = 0
    # 3D guidance tube (Stage 37.1) -- 0 when z_guide_grid/z_guide_tolerance_m=None.
    z_corridor_reject_count: int = 0
    # State-space composition diagnostics (Stage 37.1) -- always tracked (cheap, purely
    # observational, derived from `closed` at the end of the run), regardless of corridor use.
    unique_expanded_xy: int = 0
    unique_expanded_xyz: int = 0
    unique_full_states: int = 0
    avg_z_states_per_xy: float = float("nan")
    avg_history_states_per_xyz: float = float("nan")
    max_z_states_in_one_xy: int = 0


def _heuristic(
    current: AugmentedState,
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

    Ignores vertical_trend/trend_age_units on purpose -- goal is a
    physical position, and every trend/age variant of the goal state
    shares the same value, which is what makes "stop at the first
    goal-position pop" still A*-optimal (see astar_search).

    cost_mode == "normalized" (Stage 32) takes a DIFFERENT, deliberately
    minimal path: h = normalized_w_distance * D3D / distance_reference_m.
    This is a valid admissible+consistent lower bound because the true
    normalized edge cost is always >= w_distance*dC_distance (the altitude
    and reversal terms are non-negative -- see _compute_edge_cost_normalized),
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
    x1, y1, z1 = state_to_xyz((current[0], current[1], current[2]), terrain, config)
    x2, y2, z2 = state_to_xyz(goal, terrain, config)
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

    Ignores reversal_penalty entirely (it's always >= 0, so dropping it
    only loosens, never breaks, the bound) and does no terrain sampling.
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
      base_cost = geometric_cost * (1 + msl_cost_weight * altitude_scaled(edge))
      Every valid state's MSL is >= minimum_possible_aircraft_msl (by
      construction -- see _min_possible_aircraft_msl), so an edge's mean
      altitude is too, so altitude_scaled(edge) >= scaled_msl_lb (monotonic).
      With msl_cost_weight >= 0: base_cost >= geometric_cost * (1 +
      msl_cost_weight * scaled_msl_lb) = geometric_cost * multiplier.
      reversal_cost >= 0 (vertical_reversal_cost_weight >= 0, abs(dz) >= 0,
      factor in [0,1]) so actual_edge_cost = base_cost + reversal_cost >=
      base_cost >= geometric_cost * multiplier.

    Returns None -- signalling "fall back to the plain Euclidean heuristic"
    -- if msl_cost_weight < 0, msl_scale_m <= 0, or
    vertical_reversal_cost_weight < 0: any of those breaks a step of the
    proof above, so using this bound would risk inadmissibility. This
    module never assumes those stay non-negative; it checks, every call.
    """
    if config.msl_cost_weight < 0 or config.msl_scale_m <= 0 or config.vertical_reversal_cost_weight < 0:
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


def _vertical_mode(primitive: MotionPrimitive) -> int:
    """climb -> +1, descent -> -1, level -> 0."""
    if primitive.primitive_type == "climb":
        return 1
    if primitive.primitive_type == "descent":
        return -1
    return 0


def _max_trend_age_units(config: PlannerConfig) -> int:
    return math.ceil(config.reversal_relax_distance_m / config.trend_age_unit_m)


def _distance_to_age_units(distance_m: float, config: PlannerConfig) -> int:
    return min(_max_trend_age_units(config), round(distance_m / config.trend_age_unit_m))


def _next_trend_and_age(
    previous_trend: int,
    previous_age_units: int,
    primitive: MotionPrimitive,
    config: PlannerConfig,
) -> Tuple[int, int, bool, float]:
    """LEGACY exact model (Stage 14), kept only as a reference for Stage 22's
    before/after comparison -- NOT used by the production search anymore
    (see _next_trend_and_bucket). Advances the (vertical_trend,
    trend_age_units) state machine by one primitive. Returns (next_trend,
    next_age_units, is_reversal, reversal_factor).

    - A level primitive never reverses anything and never resets the age:
      if a trend is standing, the level move's horizontal distance is
      added to its age (capped); if no trend is standing yet, nothing
      changes.
    - A climb/descent continuing the standing trend (or starting the very
      first trend, previous_trend == 0) is never a reversal; its
      horizontal distance extends (or starts) the age.
    - A climb/descent directly opposing the standing trend IS a reversal.
      reversal_factor is 1.0 minus how much of reversal_relax_distance_m
      the just-ended trend had already covered (clamped to [0,1]) -- 1.0
      right after the previous reversal, 0.0 once the trend ran long
      enough. The new trend's age then starts fresh from this primitive's
      own horizontal distance (the old trend is over; it doesn't carry
      over into the new one).
    """
    mode = _vertical_mode(primitive)
    prim_units = _distance_to_age_units(primitive.horizontal_distance_m, config)
    max_units = _max_trend_age_units(config)

    if mode == 0:
        if previous_trend == 0:
            return previous_trend, previous_age_units, False, 0.0
        return previous_trend, min(max_units, previous_age_units + prim_units), False, 0.0

    if previous_trend == 0 or previous_trend == mode:
        next_age = prim_units if previous_trend == 0 else min(max_units, previous_age_units + prim_units)
        return mode, next_age, False, 0.0

    # previous_trend == -mode: a genuine direction reversal.
    previous_trend_distance_m = previous_age_units * config.trend_age_unit_m
    spacing_ratio = min(1.0, max(0.0, previous_trend_distance_m / config.reversal_relax_distance_m))
    reversal_factor = 1.0 - spacing_ratio
    return mode, prim_units, True, reversal_factor


# --------------------------------------------------------------------------
# Stage 22: 3-bucket trend-age compression (SHORT/MEDIUM/MATURE), replacing
# the 11-value 0..10 trend_age_units above in the production search. This
# is a deliberate, disclosed APPROXIMATION of the exact model above -- see
# each function's docstring and project.md "Stage 22" for exactly where
# and why it diverges (chains dominated by short/level primitives reach
# MATURE sooner than the exact 300m threshold would).
# --------------------------------------------------------------------------

BUCKET_SHORT, BUCKET_MEDIUM, BUCKET_MATURE = 0, 1, 2
_REVERSAL_FACTOR_BY_BUCKET = {BUCKET_SHORT: 1.0, BUCKET_MEDIUM: 0.5, BUCKET_MATURE: 0.0}


def _bucket_of_distance(distance_m: float, config: PlannerConfig) -> int:
    """SHORT: < reversal_relax_distance_m/2 (150m by default). MEDIUM: that
    up to reversal_relax_distance_m (300m). MATURE: >= that."""
    mature_threshold = config.reversal_relax_distance_m
    medium_threshold = mature_threshold / 2.0
    if distance_m < medium_threshold:
        return BUCKET_SHORT
    if distance_m < mature_threshold:
        return BUCKET_MEDIUM
    return BUCKET_MATURE


def _advance_bucket(bucket: int) -> int:
    return min(BUCKET_MATURE, bucket + 1)


def _next_trend_and_bucket(
    previous_trend: int,
    previous_bucket: int,
    primitive: MotionPrimitive,
    config: PlannerConfig,
) -> Tuple[int, int, bool, float]:
    """Advance the (vertical_trend, trend_age_bucket) state machine by one
    primitive. Returns (next_trend, next_bucket, is_reversal, reversal_factor).

    Same qualitative rules as the legacy exact model (level never resets
    or reverses anything and still advances a standing trend's maturity;
    climb/descent continuing the standing trend is never a reversal;
    climb/descent opposing it is) -- but since only 3 labels exist (no
    exact distance carried in the state), a fresh trend/reversal starts at
    bucket_of(this primitive's own horizontal distance) -- which, for
    every primitive in the current 24-primitive set (max length ~127m),
    is always SHORT -- and every CONTINUING step (climb/climb,
    descent/descent, or level-while-a-trend-stands) advances exactly one
    bucket, regardless of that step's own length.

    DISCLOSED APPROXIMATION: with our shortest primitive (30m level) and
    the 120m minimum climb/descent length, SHORT->MEDIUM after exactly one
    more step is never premature (minimum real distance at that point is
    120+30=150m, exactly the threshold). MEDIUM->MATURE after one more
    step CAN be premature for chains with several short (level) steps in
    a row -- e.g. descent(120) -> level(30, ->MEDIUM, real=150) ->
    level(30, ->MATURE by this rule, real=180) is called MATURE at 180m,
    while the exact model would still call it MEDIUM (spacing_ratio=0.6,
    partial penalty) until 300m. This trades exactness for only ever
    needing 3 labels; see project.md "Stage 22" and
    scripts/validate_trend_bucket.py for the disclosed numeric comparison.
    """
    mode = _vertical_mode(primitive)

    if mode == 0:
        if previous_trend == 0:
            return previous_trend, previous_bucket, False, 0.0
        return previous_trend, _advance_bucket(previous_bucket), False, 0.0

    if previous_trend == 0 or previous_trend == mode:
        next_bucket = (
            _bucket_of_distance(primitive.horizontal_distance_m, config)
            if previous_trend == 0
            else _advance_bucket(previous_bucket)
        )
        return mode, next_bucket, False, 0.0

    # previous_trend == -mode: a genuine direction reversal.
    reversal_factor = _REVERSAL_FACTOR_BY_BUCKET[previous_bucket]
    next_bucket = _bucket_of_distance(primitive.horizontal_distance_m, config)  # fresh trend starts now
    return mode, next_bucket, True, reversal_factor


def compute_distance_reference(
    start: CanonicalState,
    goal: CanonicalState,
    terrain: TerrainQuery,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> float:
    """D_ref for normalized cost mode (project.md "Stage 32"): the
    straight-line 3D distance between start and goal, computed ONCE per
    mission (never per-edge, never per-state, never inside the search
    loop). Every edge's normalized distance/altitude contribution is a
    ratio against this single constant -- independent of which path is
    actually flown."""
    x1, y1, z1 = state_to_xyz(start, terrain, config)
    x2, y2, z2 = state_to_xyz(goal, terrain, config)
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


def compute_edge_cost(
    primitive: MotionPrimitive,
    start_altitude_msl: float,
    previous_vertical_trend: int,
    previous_trend_bucket: int,
    config: PlannerConfig = DEFAULT_CONFIG,
    distance_reference_m: Optional[float] = None,
    disable_reversal_cost: bool = False,
) -> float:
    """The actual A* edge cost for applying one primitive from a given
    altitude and standing (vertical_trend, trend_age_bucket).

    Public (not just an internal detail of _generate_neighbors) so callers
    -- validation/diagnostic scripts included -- can cost a hypothetical
    primitive sequence without duplicating this formula.

    Dispatches on config.cost_mode:

    "legacy" (default, UNCHANGED since Stage 14/22 -- distance_reference_m
    is ignored entirely):
        base_cost = geometric_cost * (1 + msl_cost_weight * altitude_scaled)
        reversal_cost = vertical_reversal_cost_weight * abs(dz) * reversal_factor
                         (0 unless this primitive is a direction reversal)
        edge_cost = base_cost + reversal_cost

    "normalized" (Stage 32, opt-in -- see _compute_edge_cost_normalized):
        dimensionless dC_total = w_distance*dC_distance + w_altitude*
        dC_altitude + w_reversal*dC_reversal. Requires distance_reference_m
        (see compute_distance_reference) and config.altitude_reference_msl
        to both be set -- raises ValueError otherwise rather than silently
        falling back.

    No abs(delta_z) term outside of a reversal, and no term at all for a
    reversal starting from an already-MATURE standing trend -- see
    _next_trend_and_bucket (shared by both modes).

    disable_reversal_cost (default False, Stage 37.2): forces the
    reversal contribution to exactly 0.0 and skips consulting
    previous_vertical_trend/previous_trend_bucket entirely for it -- used
    by astar_search's history-free diagnostic mode, where trend/bucket
    are frozen (never updated), so a reversal determination against them
    would be meaningless. False (the default) reproduces the pre-
    Stage-37.2 formula exactly.
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
        return _compute_edge_cost_normalized(
            primitive, start_altitude_msl, previous_vertical_trend, previous_trend_bucket,
            config, distance_reference_m, disable_reversal_cost,
        )

    end_altitude_msl = start_altitude_msl + primitive.dz_m
    geometric_cost = math.sqrt(primitive.horizontal_distance_m ** 2 + primitive.dz_m ** 2)
    mean_altitude_msl = (start_altitude_msl + end_altitude_msl) / 2.0
    altitude_scaled = _altitude_scaled(mean_altitude_msl, config)
    base_cost = geometric_cost * (1.0 + config.msl_cost_weight * altitude_scaled)

    if disable_reversal_cost:
        return base_cost

    _, _, is_reversal, reversal_factor = _next_trend_and_bucket(
        previous_vertical_trend, previous_trend_bucket, primitive, config
    )
    reversal_cost = config.vertical_reversal_cost_weight * abs(primitive.dz_m) * reversal_factor if is_reversal else 0.0
    return base_cost + reversal_cost


def _compute_edge_cost_normalized(
    primitive: MotionPrimitive,
    start_altitude_msl: float,
    previous_vertical_trend: int,
    previous_trend_bucket: int,
    config: PlannerConfig,
    distance_reference_m: float,
    disable_reversal_cost: bool = False,
) -> float:
    """Stage 32 dimensionless edge cost (project.md "Stage 32" -- the
    production implementation of the Stage 31 diagnostic formula, with the
    reversal normalization corrected per that stage's instruction 9:
    R_edge_raw / D_ref, not the too-aggressive R_edge_raw / 20 from Stage 31).

        dC_distance = geometric_cost / D_ref
        excess_altitude = max(0, mean_MSL - altitude_reference_msl)
        dC_altitude = dC_distance * (excess_altitude / normalized_altitude_scale_m)
        dC_reversal = reversal_cost_raw / D_ref   (reversal_cost_raw: same
            raw formula as the legacy branch -- vertical_reversal_cost_weight
            * abs(dz) * reversal_factor, 0 unless this primitive is a
            direction reversal)
        dC_total = w_distance*dC_distance + w_altitude*dC_altitude + w_reversal*dC_reversal

    Every term is non-negative (max(0,...) before any weighting) and the
    total is additive -- required for A* edge costs. Mission objective is
    still LOW ABSOLUTE MSL: excess_altitude is always (aircraft MSL -
    altitude_reference_msl), never (aircraft MSL - local terrain) -- this
    is not a terrain-following/preferred-AGL cost.
    """
    if disable_reversal_cost:
        reversal_cost_raw = 0.0
    else:
        _, _, is_reversal, reversal_factor = _next_trend_and_bucket(
            previous_vertical_trend, previous_trend_bucket, primitive, config
        )
        reversal_cost_raw = (
            config.vertical_reversal_cost_weight * abs(primitive.dz_m) * reversal_factor if is_reversal else 0.0
        )
    geometric_cost = math.hypot(primitive.horizontal_distance_m, primitive.dz_m)
    components = mission_policy_from_config(config).edge_components(
        geometric_cost,
        start_altitude_msl,
        start_altitude_msl + primitive.dz_m,
        distance_reference_m,
        reversal_cost_raw,
    )
    return components.total


def validate_path_safety(
    path: List[CanonicalState],
    primitives: List[MotionPrimitive],
    terrain: TerrainQuery,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> bool:
    """Safety-only path validation; performs no costing or optimization."""
    if len(path) < 2:
        return False
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    for s1, s2 in zip(path, path[1:]):
        prim = by_delta.get((s2[0] - s1[0], s2[1] - s1[1], s2[2] - s1[2]))
        if prim is None or not evaluate_primitive(state_to_xyz(s1, terrain, config), prim, terrain, config).valid:
            return False
    return True


def validate_and_cost_path(
    path: List[CanonicalState],
    primitives: List[MotionPrimitive],
    terrain: TerrainQuery,
    config: PlannerConfig = DEFAULT_CONFIG,
    distance_reference_m: Optional[float] = None,
) -> Tuple[bool, float]:
    """Stage 23 incumbent support: validate every edge of a CANDIDATE initial
    incumbent path with the exact same safety authority the search itself
    uses (evaluate_primitive()) and cost it with the exact same production
    formula (compute_edge_cost, replayed through the real (vertical_trend,
    trend_age_bucket) state machine) -- never a separate/approximate
    formula, and never assuming anything about which physical path this
    is. Returns (False, inf) if any edge has no matching primitive or
    evaluate_primitive() rejects it -- the caller must treat that as "no
    usable initial incumbent" (pass cost=inf to astar_search) rather than
    trusting a partially-checked path.

    distance_reference_m (Stage 32): required, and used, only when
    config.cost_mode == "normalized" -- ignored under "legacy" (may be left
    None). The caller computes it once via compute_distance_reference(start,
    goal, ...) with the SAME start/goal passed to astar_search(), so the
    incumbent's own cost and the search's internal cost use an identical
    D_ref -- never recomputed per-path here.
    """
    # Backward-compatible incumbent convenience API.  Safety is decided by
    # the safety-only validator above; the remaining loop is objective replay.
    if not validate_path_safety(path, primitives, terrain, config):
        return False, math.inf
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    trend, bucket = 0, BUCKET_SHORT
    total_cost = 0.0
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        if prim is None:  # already excluded by validate_path_safety
            return False, math.inf
        start_xyz = state_to_xyz((r1, c1, z1), terrain, config)
        total_cost += compute_edge_cost(prim, start_xyz[2], trend, bucket, config, distance_reference_m)
        trend, bucket, _, _ = _next_trend_and_bucket(trend, bucket, prim, config)
    return True, total_cost


def _primitive_id(primitive: MotionPrimitive) -> PrimitiveId:
    """Stable, canonical identity for a primitive -- independent of which
    MotionPrimitive object instance is passed in, so a cache built against
    one primitive list still hits for an equal primitive from another."""
    return (primitive.drow, primitive.dcol, primitive.dz_m, primitive.primitive_type)


def _generate_neighbors(
    state: AugmentedState,
    primitives: List[MotionPrimitive],
    terrain: TerrainQuery,
    config: PlannerConfig,
    min_search_altitude_msl: float,
    max_search_altitude_msl: float,
    primitive_cache: Optional[Dict[PrimitiveCacheKey, PrimitiveEvalResult]],
    cache_stats: Dict[str, int],
    distance_reference_m: Optional[float] = None,
    corridor_mask: Optional[np.ndarray] = None,
    z_guide_grid: Optional[np.ndarray] = None,
    z_guide_tolerance_m: Optional[float] = None,
    freeze_history: bool = False,
    fine_precompute: Optional["FinePrecomputeResult"] = None,
):
    """current -> primitive -> candidate -> bounds -> XY corridor -> Z guide tube ->
    altitude bounds -> evaluate_primitive (or cache) -> neighbor.

    primitive_cache holds PHYSICAL feasibility results only, keyed by
    (row, col, z_index, primitive_id) -- never by vertical_trend or
    trend_age_units, because whether a primitive is safe from a given
    physical position never depends on search history (see module
    docstring). Pass None to disable caching entirely (use_primitive_cache
    =False). Cost (compute_edge_cost) and the next (trend, age) are ALWAYS
    computed fresh per augmented state below, never cached -- those DO
    depend on history/config preference.

    corridor_mask (Stage 37, default None): an optional boolean array,
    shape (terrain.roi.height, terrain.roi.width), True where (row, col)
    is inside the allowed XY corridor (e.g. planner.corridor's fine mask
    around a coarse guide path -- see planner/corridor.py). A successor
    whose (row, col) is outside the corridor is rejected ("outside_
    corridor") BEFORE the expensive evaluate_primitive()/cache lookup --
    corridor_reject_count. This is an XY-only prefilter, never a safety
    decision: it changes nothing about AGL/terrain/NoData/angle checks,
    which still apply to every candidate the corridor lets through. None
    (the default) reproduces the pre-Stage-37 behavior exactly -- no
    corridor check is ever performed.

    z_guide_grid / z_guide_tolerance_m (Stage 37.1, both default None):
    an optional per-(row,col) guidance altitude (MSL, e.g. planner.
    corridor's interpolated coarse-path Z -- see build_z_guide_grid) and
    a tolerance in meters. A successor that passed the XY corridor check
    but whose own altitude is farther than z_guide_tolerance_m from
    z_guide_grid[new_row, new_col] is rejected ("outside_z_guide_tube" --
    z_corridor_reject_count) -- again BEFORE evaluate_primitive, and
    again purely a guidance prefilter, never a safety decision. Only
    consulted when BOTH are given (and only meaningfully after the XY
    check already passed) -- None (the default) means no Z-tube
    narrowing at all, reproducing the pre-Stage-37.1 behavior exactly.

    fine_precompute (Stage 38.1, default None): a planner.fine_precompute.
    FinePrecomputeResult -- when given, REPLACES evaluate_primitive()/
    primitive_cache for the safety decision with an O(1) dense-array
    lookup (planner.fine_precompute.fine_precomputed_primitive_validity),
    keyed by (row, col, primitive_index, start_msl) -- z_index never
    enters it, matching primitive_cache's own key shape. primitive_cache
    is ignored entirely when this is given (no terrain re-sampling, no
    cache_stats hits/misses recorded -- the precompute has already
    superseded that role). None (the default) reproduces the pre-Stage-
    38.1 behavior exactly.

    Returns (accepted, rejected_reason_counts, generated_count, rejected_count,
    corridor_reject_count, z_corridor_reject_count).
    accepted is a list of (neighbor_augmented_state, edge_cost).
    """
    row, col, z_index, prev_trend, prev_bucket = state
    start_xyz = state_to_xyz((row, col, z_index), terrain, config)

    accepted: List[Tuple[AugmentedState, float]] = []
    rejected_counts: Dict[str, int] = {}
    generated = 0
    rejected = 0
    corridor_reject_count = 0
    z_corridor_reject_count = 0

    for prim_idx, prim in enumerate(primitives):
        generated += 1
        new_row = row + prim.drow
        new_col = col + prim.dcol
        dz_index = round(prim.dz_m / config.z_step_m)
        new_z_index = z_index + dz_index
        new_z_msl = z_index_to_msl(new_z_index, config)

        if not terrain.in_bounds_rowcol(new_row, new_col):
            rejected += 1
            rejected_counts["out_of_bounds"] = rejected_counts.get("out_of_bounds", 0) + 1
            continue

        if corridor_mask is not None and not corridor_mask[new_row, new_col]:
            rejected += 1
            corridor_reject_count += 1
            rejected_counts["outside_corridor"] = rejected_counts.get("outside_corridor", 0) + 1
            continue

        if z_guide_grid is not None and z_guide_tolerance_m is not None:
            z_guide = z_guide_grid[new_row, new_col]
            if abs(new_z_msl - z_guide) > z_guide_tolerance_m:
                rejected += 1
                z_corridor_reject_count += 1
                rejected_counts["outside_z_guide_tube"] = rejected_counts.get("outside_z_guide_tube", 0) + 1
                continue

        if not (min_search_altitude_msl <= new_z_msl <= max_search_altitude_msl):
            rejected += 1
            rejected_counts["altitude_search_bounds"] = rejected_counts.get("altitude_search_bounds", 0) + 1
            continue

        if fine_precompute is not None:
            from planner.fine_precompute import fine_precomputed_primitive_validity  # lazy: avoids import cycle
            valid, reason = fine_precomputed_primitive_validity(fine_precompute, row, col, prim_idx, start_xyz[2])
            if not valid:
                rejected += 1
                rejected_counts[reason] = rejected_counts.get(reason, 0) + 1
                continue
        else:
            if primitive_cache is None:
                eval_result = evaluate_primitive(start_xyz, prim, terrain, config)
                cache_stats["actual_calls"] += 1
            else:
                cache_key: PrimitiveCacheKey = (row, col, z_index, _primitive_id(prim))
                eval_result = primitive_cache.get(cache_key)
                if eval_result is None:
                    cache_stats["misses"] += 1
                    eval_result = evaluate_primitive(start_xyz, prim, terrain, config)
                    cache_stats["actual_calls"] += 1
                    primitive_cache[cache_key] = eval_result  # cache both VALID and INVALID results
                else:
                    cache_stats["hits"] += 1

            if not eval_result.valid:
                rejected += 1
                rejected_counts[eval_result.reason] = rejected_counts.get(eval_result.reason, 0) + 1
                continue

        # AGL/terrain/transition safety was already fully decided above by
        # evaluate_primitive() (or the cached result of it) -- everything
        # from here down is cost, not feasibility, and always recomputed
        # from this augmented state's own (prev_trend, prev_bucket): the MSL
        # and reversal terms only ever add to geometric_cost, and only
        # apply to edges already proven safe -- they can't forbid an edge,
        # and can't stop a climb that's the only safe way through.
        edge_cost = compute_edge_cost(prim, start_xyz[2], prev_trend, prev_bucket, config, distance_reference_m,
                                      disable_reversal_cost=freeze_history)
        if freeze_history:
            # Stage 37.2 diagnostic: trend/bucket are never updated -- every reachable
            # state inherits the SAME (frozen) history its start_aug began with, so the
            # augmented 5-tuple's last two fields are constant and the state is
            # effectively (row, col, z_index) only (see astar_search's docstring).
            next_trend, next_bucket = prev_trend, prev_bucket
        else:
            next_trend, next_bucket, _, _ = _next_trend_and_bucket(prev_trend, prev_bucket, prim, config)

        neighbor_state: AugmentedState = (new_row, new_col, new_z_index, next_trend, next_bucket)
        accepted.append((neighbor_state, edge_cost))

    return accepted, rejected_counts, generated, rejected, corridor_reject_count, z_corridor_reject_count


def _reconstruct_path(came_from: Dict[AugmentedState, AugmentedState], start: AugmentedState, goal_state: AugmentedState) -> List[CanonicalState]:
    aug_path = [goal_state]
    current = goal_state
    while current != start:
        current = came_from[current]
        aug_path.append(current)
    aug_path.reverse()
    return [(s[0], s[1], s[2]) for s in aug_path]  # strip trend/age for the public path


def _path_altitude_metrics(path: List[CanonicalState], terrain: TerrainQuery, config: PlannerConfig) -> Dict[str, float]:
    """Unweighted geometric length, MSL altitude stats, and climb/descent
    totals for a found path. Purely descriptive now -- not cost inputs.

    average_aircraft_msl is weighted by each edge's geometric length (not a
    plain per-node average) so a long low-altitude leg actually outweighs a
    short high one -- otherwise sparse high-altitude waypoints could skew
    the average away from what was actually flown.
    """
    xyz = [state_to_xyz(s, terrain, config) for s in path]
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
    primitives: List[MotionPrimitive],
    terrain: TerrainQuery,
    config: PlannerConfig,
) -> float:
    """Re-derive the minimum AGL along a found path via evaluate_primitive()
    on each edge -- reuses the existing safety validation, doesn't reimplement it."""
    if len(path) < 2:
        return float("nan")
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    min_agl = math.inf
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        if prim is None:
            continue  # shouldn't happen for a path this search produced
        start_xyz = state_to_xyz((r1, c1, z1), terrain, config)
        result = evaluate_primitive(start_xyz, prim, terrain, config)
        min_agl = min(min_agl, result.min_agl_m)
    return min_agl if min_agl != math.inf else float("nan")


def _path_vertical_reversal_metrics(
    path: List[CanonicalState],
    primitives: List[MotionPrimitive],
    config: PlannerConfig,
) -> Dict[str, float]:
    """Re-derive reversal count/spacing/penalty along a found path by
    replaying each edge's primitive through the same (trend,
    trend_age_bucket) state machine _generate_neighbors uses
    (_next_trend_and_bucket, Stage 22). Works purely from the physical
    path + primitive set -- doesn't require the augmented search states
    to have been kept around.

    minimum/average_reversal_spacing_m are now APPROXIMATE: since the
    bucket carries no exact distance, each reversal's "spacing" is
    reported as its bucket's own lower threshold (0 / 150 / 300m) rather
    than a real measured distance -- a disclosed consequence of the
    3-bucket compression, not a precise measurement.
    """
    empty = {
        "total_vertical_reversal_count": 0, "penalized_reversal_count": 0,
        "zero_penalty_long_spacing_reversal_count": 0,
        "minimum_reversal_spacing_m": float("nan"), "average_reversal_spacing_m": float("nan"),
        "total_reversal_penalty": 0.0,
    }
    if len(path) < 2:
        return empty

    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    trend = 0
    bucket = BUCKET_SHORT
    total_count = 0
    penalized_count = 0
    penalty = 0.0
    spacings: List[float] = []
    bucket_lower_threshold = {BUCKET_SHORT: 0.0, BUCKET_MEDIUM: config.reversal_relax_distance_m / 2.0,
                              BUCKET_MATURE: config.reversal_relax_distance_m}

    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        if prim is None:
            continue  # shouldn't happen for a path this search produced
        next_trend, next_bucket, is_reversal, factor = _next_trend_and_bucket(trend, bucket, prim, config)
        if is_reversal:
            total_count += 1
            spacings.append(bucket_lower_threshold[bucket])
            cost = config.vertical_reversal_cost_weight * abs(prim.dz_m) * factor
            penalty += cost
            if cost > 1e-9:
                penalized_count += 1
        trend, bucket = next_trend, next_bucket

    if not spacings:
        return empty

    return {
        "total_vertical_reversal_count": total_count,
        "penalized_reversal_count": penalized_count,
        "zero_penalty_long_spacing_reversal_count": total_count - penalized_count,
        "minimum_reversal_spacing_m": min(spacings),
        "average_reversal_spacing_m": sum(spacings) / len(spacings),
        "total_reversal_penalty": penalty,
    }


def _dominates(age_a: int, g_a: float, age_b: int, g_b: float) -> bool:
    """Does (age_a, g_a) dominate (age_b, g_b)? Only meaningful when both
    states share the same (row, col, z_index, vertical_trend) -- callers
    are responsible for that; this function only compares the pair.

    True iff g_a <= g_b AND age_a >= age_b (both required -- see module
    docstring / project.md "Stage 16" for why this is exact, not approximate).
    """
    return g_a <= g_b and age_a >= age_b


def _current_lower_bound(lb_heap: List[Tuple[float, int, AugmentedState]], closed: set) -> float:
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
    config: PlannerConfig = DEFAULT_CONFIG,
    primitives: Optional[List[MotionPrimitive]] = None,
    max_expansions: Optional[int] = None,
    use_primitive_cache: bool = True,
    use_dominance_pruning: bool = True,
    use_msl_lower_bound_heuristic: bool = True,
    use_vertical_reachability_heuristic: bool = True,
    use_incumbent_pruning: bool = False,
    initial_incumbent_cost: float = math.inf,
    initial_incumbent_path: Optional[List[CanonicalState]] = None,
    epsilon_search: float = 1.0,
    target_suboptimality: Optional[float] = None,
    corridor_mask: Optional[np.ndarray] = None,
    z_guide_grid: Optional[np.ndarray] = None,
    z_guide_tolerance_m: Optional[float] = None,
    freeze_history: bool = False,
    external_primitive_cache: Optional[Dict[PrimitiveCacheKey, PrimitiveEvalResult]] = None,
    stop_on_first_solution: bool = False,
) -> SearchResult:
    """3D A* from start to goal using only the existing safe motion primitives.

    start/goal are physical (row, col, z_index) -- the search runs over the
    augmented (row, col, z_index, vertical_trend, trend_age_units) state
    internally, starting at vertical_trend=0, trend_age_units=0, but the
    goal condition only ever checks physical position: any trend/age
    combination counts as having reached the goal. This is still
    A*-optimal because every trend/age variant of the goal shares the same
    heuristic value (straight-line distance to a physical point doesn't
    depend on either) -- see _heuristic's docstring.

    min_search_altitude_msl / max_search_altitude_msl bound the prototype
    search space -- they are NOT an aircraft flight ceiling. A neighbor
    outside this range is never generated.

    use_primitive_cache (default True): cache evaluate_primitive() results
    by physical (row, col, z_index, primitive_id) for the lifetime of this
    call only (never global/persistent -- a fresh dict every call, so a
    different terrain/ROI/config next call can't see stale results). Set
    False to reproduce the uncached behavior exactly, e.g. for a
    before/after benchmark -- it changes nothing about which path is
    found, only how many times evaluate_primitive() actually runs.

    use_dominance_pruning (default True): for states sharing the same
    (row, col, z_index, vertical_trend), drop an alternative whose g is no
    better AND whose trend_age is no larger than one already known -- it
    can never do better in the future either (proven exhaustively for
    _next_trend_and_age: higher age never increases a future reversal's
    cost, and never decreases it below the lower-age alternative's, for
    any shared future primitive sequence). Set False to reproduce
    pre-Stage-16 behavior exactly, e.g. for a before/after benchmark -- it
    changes nothing about the optimal cost found, only how many
    (age-)redundant states get expanded.

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
    caller-supplied via initial_incumbent_cost/initial_incumbent_path,
    typically produced by validate_and_cost_path() on some candidate path;
    this function is completely agnostic to what that path physically is).
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

    corridor_mask (default None, Stage 37): an optional boolean array,
    shape (terrain.roi.height, terrain.roi.width), True where (row, col)
    is inside an allowed XY corridor (e.g. planner.corridor's mask around
    a coarse guide path). A successor whose (row, col) falls outside it
    is rejected BEFORE the expensive evaluate_primitive()/cache lookup --
    see _generate_neighbors -- reported as corridor_reject_count. XY-only:
    never restricts z_index/altitude, and never changes AGL/terrain/
    NoData/angle safety, which still applies to every candidate the
    corridor lets through. None (the default) reproduces the pre-
    Stage-37 search exactly -- no corridor check is ever performed.

    z_guide_grid / z_guide_tolerance_m (default None, Stage 37.1): an
    optional per-(row,col) guidance MSL (e.g. planner.corridor.
    build_z_guide_grid's coarse-path-interpolated altitude) and a
    tolerance in meters -- a successor that already passed corridor_mask
    but whose own altitude is farther than z_guide_tolerance_m from that
    guidance value is rejected, again before evaluate_primitive, again
    purely a guidance narrowing (never changes AGL/terrain/NoData/angle
    safety). Reported as z_corridor_reject_count. Only meaningful when
    both are given; None (the default, either one) means no Z-tube
    narrowing at all, reproducing the pre-Stage-37.1 search exactly.

    freeze_history (default False, Stage 37.2): a diagnostic-only mode --
    NOT a production default -- that measures the search-state impact of
    vertical_trend/trend_age_bucket in isolation. When True, a
    successor's (trend, bucket) are never advanced via
    _next_trend_and_bucket; they simply inherit the CURRENT state's own
    (frozen at start_aug's initial (0, BUCKET_SHORT) forever), and
    compute_edge_cost is called with disable_reversal_cost=True (the
    reversal component is always exactly 0.0 -- there is no history left
    to reverse against). The augmented state therefore still has the
    same 5-tuple shape internally, but since its last two fields never
    vary, it is bijective with (row, col, z_index) alone -- no XYZ is
    ever duplicated across different histories, because there IS only
    one (frozen) history. Everything else (corridor_mask, z_guide_grid,
    incumbent, epsilon_search, cache, dominance, cost's distance/
    altitude terms) behaves exactly as already documented above. False
    (the default) reproduces the pre-Stage-37.2 search exactly.

    external_primitive_cache (default None, Stage 37.3): pass an existing
    dict (e.g. one returned by a previous call's own internal cache, if
    the caller kept a reference) to reuse and keep populating it across
    MULTIPLE astar_search() calls -- e.g. an epsilon sweep against the
    same terrain/config, where every call after the first can reuse
    already-evaluated (row, col, z_index, primitive_id) results instead
    of recomputing them. Only consulted when use_primitive_cache=True;
    ignored (a fresh cache is created, exactly as before) when it is
    False. None (the default) reproduces the pre-Stage-37.3 behavior
    exactly -- a brand new, empty cache every call, discarded after.

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
    """
    if primitives is None:
        primitives = build_primitive_set(config)

    t0 = time.perf_counter()
    counter = itertools.count()

    if use_primitive_cache:
        primitive_cache: Optional[Dict[PrimitiveCacheKey, PrimitiveEvalResult]] = (
            external_primitive_cache if external_primitive_cache is not None else {}
        )
    else:
        primitive_cache = None
    cache_stats = {"hits": 0, "misses": 0, "actual_calls": 0}

    # Per-base-key Pareto frontier of (age, g) pairs, age ascending / g strictly
    # increasing on the active front (see module docstring / project.md "Stage 16").
    frontier: Dict[BaseKey, List[Tuple[int, float]]] = {}
    dominance_stats = {"checks": 0, "pruned": 0, "removed": 0, "pop_skipped": 0, "max_frontier": 0}

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
        compute_distance_reference(start, goal, terrain, config) if config.cost_mode == "normalized" else None
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
    corridor_reject_count = 0
    z_corridor_reject_count = 0

    # Stage 33: goal center in real UTM meters, computed once (never per-expansion),
    # for the closest-state-to-goal diagnostics below.
    goal_x, goal_y, goal_z = state_to_xyz(goal, terrain, config)
    closest_distance_to_goal_region_m = math.inf
    closest_distance_to_goal_center_m = math.inf
    closest_state_to_goal: Optional[CanonicalState] = None

    start_aug: AugmentedState = (start[0], start[1], start[2], 0, BUCKET_SHORT)

    g_score: Dict[AugmentedState, float] = {start_aug: 0.0}
    came_from: Dict[AugmentedState, AugmentedState] = {}
    closed = set()

    if use_dominance_pruning:
        frontier[start_aug[:4]] = [(0, 0.0)]

    h_start = _heuristic(start_aug, goal, terrain, config, cost_multiplier, heuristic_min_msl,
                          vertical_reachability_active, distance_reference_m)
    start_counter = next(counter)
    open_heap = [(epsilon_search * h_start, start_counter, start_aug, h_start)]
    lb_heap: List[Tuple[float, int, AugmentedState]] = [(h_start, start_counter, start_aug)] if bounded_mode else []
    max_open_size = 1

    expanded_nodes = 0
    generated_neighbors = 0
    rejected_neighbors = 0
    rejected_reason_counts: Dict[str, int] = {}
    status = "no_path"
    goal_state: Optional[AugmentedState] = None

    while open_heap:
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

        if use_dominance_pruning:
            base_key = current[:4]
            age = current[4]
            if (age, g_score[current]) not in frontier.get(base_key, ()):
                dominance_stats["pop_skipped"] += 1
                continue  # this (age, g) was superseded by a dominating alternative after being pushed

        closed.add(current)
        expanded_nodes += 1

        current_physical = (current[0], current[1], current[2])
        x_c, y_c, z_c = state_to_xyz(current_physical, terrain, config)
        region_dist = _distance_to_goal_box(x_c, y_c, z_c, goal_x, goal_y, goal_z,
                                             config.goal_tolerance_xy_m, config.goal_tolerance_z_m)
        if region_dist < closest_distance_to_goal_region_m:
            closest_distance_to_goal_region_m = region_dist
            closest_state_to_goal = current_physical
        center_dist = math.sqrt((x_c - goal_x) ** 2 + (y_c - goal_y) ** 2 + (z_c - goal_z) ** 2)
        if center_dist < closest_distance_to_goal_center_m:
            closest_distance_to_goal_center_m = center_dist

        if _state_in_goal_region(current_physical, goal, terrain, config):
            solution_cost = g_score[current]
            if first_solution_cost == math.inf:
                first_solution_cost = solution_cost
                first_solution_expanded = expanded_nodes
                first_solution_runtime_s = time.perf_counter() - t0
            if use_incumbent_pruning and solution_cost < incumbent_cost:
                incumbent_cost = solution_cost
                incumbent_stats["updates"] += 1
                incumbent_path = _reconstruct_path(came_from, start_aug, current)
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

        neighbors, rej_counts, gen_count, rej_count, corridor_rej_count, z_corridor_rej_count = _generate_neighbors(
            current, primitives, terrain, config, min_search_altitude_msl, max_search_altitude_msl,
            primitive_cache, cache_stats, distance_reference_m, corridor_mask, z_guide_grid, z_guide_tolerance_m,
            freeze_history,
        )
        generated_neighbors += gen_count
        rejected_neighbors += rej_count
        corridor_reject_count += corridor_rej_count
        z_corridor_reject_count += z_corridor_rej_count
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

                push_to_heap = True
                if use_dominance_pruning:
                    base_key = neighbor_state[:4]
                    age_c = neighbor_state[4]
                    existing = frontier.get(base_key, [])
                    dominance_stats["checks"] += 1

                    if any(_dominates(age_e, g_e, age_c, tentative_g) for age_e, g_e in existing):
                        dominance_stats["pruned"] += 1
                        push_to_heap = False
                    else:
                        survivors = [(age_e, g_e) for age_e, g_e in existing
                                     if not _dominates(age_c, tentative_g, age_e, g_e)]
                        dominance_stats["removed"] += len(existing) - len(survivors)
                        survivors.append((age_c, tentative_g))
                        frontier[base_key] = survivors
                        dominance_stats["max_frontier"] = max(dominance_stats["max_frontier"], len(survivors))

                if push_to_heap:
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
    # `closed` set (every AugmentedState this search actually expanded) -- cheap
    # (one pass over at most max_expansions entries), always computed, never
    # affects search behavior.
    xy_seen: Dict[Tuple[int, int], set] = {}
    xyz_seen: set = set()
    for (r, c, z, _trend, _bucket) in closed:
        xy_seen.setdefault((r, c), set()).add(z)
        xyz_seen.add((r, c, z))
    unique_expanded_xy = len(xy_seen)
    unique_expanded_xyz = len(xyz_seen)
    unique_full_states = len(closed)
    avg_z_states_per_xy = unique_expanded_xyz / unique_expanded_xy if unique_expanded_xy else float("nan")
    avg_history_states_per_xyz = unique_full_states / unique_expanded_xyz if unique_expanded_xyz else float("nan")
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

    hits, misses = cache_stats["hits"], cache_stats["misses"]
    hit_rate = hits / (hits + misses) if use_primitive_cache and (hits + misses) > 0 else float("nan")

    if use_dominance_pruning and frontier:
        avg_frontier_size = sum(len(v) for v in frontier.values()) / len(frontier)
    else:
        avg_frontier_size = float("nan")

    if status == "success" and goal_state is not None:
        path = _reconstruct_path(came_from, start_aug, goal_state)
        total_cost = g_score[goal_state]
        alt_metrics = _path_altitude_metrics(path, terrain, config)
        min_observed_agl = _path_min_observed_agl(path, primitives, terrain, config)
        reversal_metrics = _path_vertical_reversal_metrics(path, primitives, config)
    elif status == "success" and goal_state is None:
        # The search proved the incumbent is (certifiably, or by exhaustion, or by
        # the exact-mode min-heap pop-order argument) at least as good as anything
        # left to explore, without ever itself popping a goal state. incumbent_path
        # already holds whichever is correct: the caller's initial_incumbent_path
        # if never internally improved, or the reconstructed better path from the
        # goal-pop(s) that updated it (see the loop above) otherwise.
        path = incumbent_path
        total_cost = incumbent_cost
        alt_metrics = _path_altitude_metrics(path, terrain, config)
        min_observed_agl = _path_min_observed_agl(path, primitives, terrain, config)
        reversal_metrics = _path_vertical_reversal_metrics(path, primitives, config)
    else:
        path = []
        total_cost = float("nan")
        alt_metrics = {"geometric_path_length": float("nan"), "minimum_aircraft_msl": float("nan"),
                       "maximum_aircraft_msl": float("nan"), "average_aircraft_msl": float("nan"),
                       "total_climb_m": float("nan"), "total_descent_m": float("nan"),
                       "total_vertical_motion_m": float("nan")}
        min_observed_agl = float("nan")
        reversal_metrics = {"total_vertical_reversal_count": 0, "penalized_reversal_count": 0,
                             "zero_penalty_long_spacing_reversal_count": 0,
                             "minimum_reversal_spacing_m": float("nan"), "average_reversal_spacing_m": float("nan"),
                             "total_reversal_penalty": float("nan")}

    return SearchResult(
        success=(status == "success"),
        status=status,
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
        total_vertical_reversal_count=reversal_metrics["total_vertical_reversal_count"],
        penalized_reversal_count=reversal_metrics["penalized_reversal_count"],
        zero_penalty_long_spacing_reversal_count=reversal_metrics["zero_penalty_long_spacing_reversal_count"],
        minimum_reversal_spacing_m=reversal_metrics["minimum_reversal_spacing_m"],
        average_reversal_spacing_m=reversal_metrics["average_reversal_spacing_m"],
        total_reversal_penalty=reversal_metrics["total_reversal_penalty"],
        primitive_cache_hits=hits,
        primitive_cache_misses=misses,
        primitive_cache_hit_rate=hit_rate,
        actual_evaluate_primitive_calls=cache_stats["actual_calls"],
        avoided_evaluate_primitive_calls=hits,
        dominance_checks=dominance_stats["checks"],
        dominance_pruned_candidates=dominance_stats["pruned"],
        dominance_frontier_entries_removed=dominance_stats["removed"],
        dominated_heap_pops_skipped=dominance_stats["pop_skipped"],
        max_dominance_frontier_size=dominance_stats["max_frontier"],
        average_dominance_frontier_size=avg_frontier_size,
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
        corridor_reject_count=corridor_reject_count,
        z_corridor_reject_count=z_corridor_reject_count,
        unique_expanded_xy=unique_expanded_xy,
        unique_expanded_xyz=unique_expanded_xyz,
        unique_full_states=unique_full_states,
        avg_z_states_per_xy=avg_z_states_per_xy,
        avg_history_states_per_xyz=avg_history_states_per_xyz,
        max_z_states_in_one_xy=max_z_states_in_one_xy,
    )


# --------------------------------------------------------------------------
# Stage 38: genuine ARA* (Anytime Repairing A*, Likhachev et al.) over the
# SAME fine-grid search graph as astar_search -- see ara_star_search's own
# docstring below for the full algorithm mapping.
# --------------------------------------------------------------------------

@dataclass
class ARAPhaseResult:
    epsilon: float
    added_expansions: int  # this phase's own expansions (cumulative_expansions delta)
    cumulative_expansions: int  # running total across every phase so far, shared 30k budget
    cumulative_runtime_s: float
    open_size_at_end: int
    incons_size_at_end: int
    closed_size_this_phase: int  # reset to 0 at the start of every phase
    g_value_improvement_count: int  # g(s) strictly improved (new discovery, reopen-as-INCONS, or incumbent update)
    incumbent_available: bool
    incumbent_cost: float
    diagnostic_lower_bound: float  # min(g+h) over OPEN u INCONS at phase end -- see docstring, NOT a certificate
    diagnostic_bound_ratio: float  # incumbent_cost / diagnostic_lower_bound, NaN if not computable
    xy_length_m: float
    length_3d_m: float
    min_msl: float
    mean_msl: float
    max_msl: float
    total_climb_m: float
    total_descent_m: float
    vertical_reversal_count: int  # diagnostic only (fresh replay), never a cost input here
    phase_complete: bool  # True: ImprovePath ran to its own termination; False: cut off by the 30k cap
    # Reporting-only snapshots.  They do not participate in ARA* ordering,
    # relaxation, stopping, or reuse; Stage 38.2 uses them to replay each
    # phase's incumbent without starting another search.
    first_incumbent_improvement_expansion: Optional[int] = None
    last_incumbent_improvement_expansion: Optional[int] = None
    incumbent_path: List[CanonicalState] = field(default_factory=list)


@dataclass
class ARASearchResult:
    path_found: bool  # True iff any incumbent (complete, safe -- goal-region-reaching) path was ever found
    refinement_limit_reached: bool  # True iff the cumulative 30k cap stopped the run before the schedule finished
    first_incumbent_cost: float
    first_incumbent_expanded: int
    first_incumbent_runtime_s: float
    first_incumbent_path: List[CanonicalState]
    final_incumbent_cost: float
    final_incumbent_path: List[CanonicalState]
    total_expanded: int
    total_runtime_s: float
    phases: List[ARAPhaseResult] = field(default_factory=list)


def ara_star_search(
    start: CanonicalState,
    goal: CanonicalState,
    terrain: TerrainQuery,
    min_search_altitude_msl: float,
    max_search_altitude_msl: float,
    config: PlannerConfig = DEFAULT_CONFIG,
    primitives: Optional[List[MotionPrimitive]] = None,
    epsilon_schedule: Tuple[float, ...] = (1.7, 1.5, 1.3, 1.1),
    max_expansions_cumulative: int = 30_000,
    corridor_mask: Optional[np.ndarray] = None,
    z_guide_grid: Optional[np.ndarray] = None,
    z_guide_tolerance_m: Optional[float] = None,
    use_primitive_cache: bool = True,
    fine_precompute: Optional["FinePrecomputeResult"] = None,
) -> ARASearchResult:
    """Genuine ARA* -- ONE persistent g/parent/OPEN/CLOSED/INCONS/incumbent
    search state is carried across the decreasing epsilon_schedule via
    repeated ImprovePath phases, never a series of independent astar_search()
    calls (each phase would then re-derive everything from scratch, which is
    exactly what ARA* exists to avoid). freeze_history=True semantics (Stage
    37.2/37.3: no trend/bucket, disable_reversal_cost) are hardwired ON --
    this refines exactly the Stage 37.3/37.4 configuration, so every edge
    still passes through the SAME _generate_neighbors -> evaluate_primitive
    terrain/AGL/angle safety checks, and the SAME corridor_mask/z_guide_grid/
    z_guide_tolerance_m prefilters, on every expansion, every phase.

    Goal-as-region adaptation: classic ARA* holds a single fixed sgoal whose
    g(sgoal) never gets expanded, only relaxed-into (like any other edge
    target), and ImprovePath's stopping test compares OPEN's min key against
    Key(sgoal). Here _state_in_goal_region replaces "is this state sgoal" --
    a candidate successor that lands inside the goal region has its g/parent
    updated exactly like any other relaxation, and if that g improves the
    running incumbent_cost/incumbent_state, the incumbent updates -- but the
    state is NEVER pushed into OPEN or INCONS (it is a sink: no path through
    the interior of the goal region needs to continue past its entry point).
    This exactly plays the role of g(sgoal), so incumbent_cost substitutes
    for Key(sgoal) in the termination test.

    Reopening: a state already CLOSED this phase that gets relaxed to a
    strictly better g is deferred into INCONS -- never immediately reopened
    into OPEN within the same phase (that would make this repeated weighted
    A*, not ARA*). At each epsilon decrease: OPEN absorbs all of INCONS
    (which is then cleared), every remaining OPEN key is recomputed with the
    new epsilon, and CLOSED is cleared -- but g, parent, the primitive_cache,
    and incumbent_cost/state all persist untouched, so no search information
    or edge evaluation is ever redone from scratch.

    ImprovePath terminates a phase when OPEN's minimum weighted key can no
    longer beat incumbent_cost (or OPEN empties outright) -- exactly the
    "min active weighted key >= incumbent" criterion the spec calls for.

    fine_precompute (Stage 38.1, default None): a planner.fine_precompute.
    FinePrecomputeResult (see that module) -- when given, every expansion's
    safety decision is an O(1) dense-array lookup instead of evaluate_
    primitive()/primitive_cache (use_primitive_cache is then ignored). It
    changes nothing about WHICH edges are safe -- only how that decision is
    computed -- so search order, costs, and results are unaffected; only
    wall-clock (and cache_hit_rate reporting, which becomes meaningless)
    change. None (the default) reproduces the pre-Stage-38.1 behavior
    exactly.

    max_expansions_cumulative caps TOTAL expansions across every phase
    combined (never reset per-phase). If it is reached mid-phase, the whole
    run stops immediately: refinement_limit_reached=True, and path_found
    reflects whatever incumbent already stands (independent of whether any
    phase's ImprovePath, or the schedule itself, ever completed).

    diagnostic_lower_bound (per phase) = min(g(s)+h(s)) over every state
    currently in OPEN or INCONS (the active, unresolved frontier) at that
    phase's end -- reported ONLY as a diagnostic anytime-quality ratio, never
    as a certified bound. Unlike astar_search's single-epsilon bounded_mode
    (which has an existing, separately proven admissibility argument for its
    %-suboptimality certificate), a state CLOSED in an earlier phase and
    never reopened into INCONS is only "resolved" in the ordinary weighted-A*
    sense (within that phase's own epsilon factor) -- there is no proof here
    that excluding it from the min is still a valid admissible lower bound
    once epsilon has since changed and the goal is a region rather than a
    single node. Reporting it honestly as a diagnostic (not a certificate)
    is the explicit instruction this stage was given.
    """
    if primitives is None:
        primitives = build_primitive_set(config)

    t0 = time.perf_counter()
    counter = itertools.count()

    primitive_cache: Optional[Dict[PrimitiveCacheKey, PrimitiveEvalResult]] = (
        {} if (use_primitive_cache and fine_precompute is None) else None
    )
    cache_stats = {"hits": 0, "misses": 0, "actual_calls": 0}

    distance_reference_m = (
        compute_distance_reference(start, goal, terrain, config) if config.cost_mode == "normalized" else None
    )

    start_aug: AugmentedState = (start[0], start[1], start[2], 0, BUCKET_SHORT)

    g: Dict[AugmentedState, float] = {start_aug: 0.0}
    parent: Dict[AugmentedState, AugmentedState] = {}

    open_members: set = {start_aug}
    incons_members: set = set()

    incumbent_cost = math.inf
    incumbent_state: Optional[AugmentedState] = None
    # Keep the accepted incumbent immutable.  Parent pointers for its
    # ancestors may improve in later phases even when the goal sink itself
    # is not relaxed again; reconstructing from the live parent map later
    # can therefore produce a different path whose recomputed cost no longer
    # equals incumbent_cost.  This snapshot is reporting/output state only.
    incumbent_path_snapshot: List[CanonicalState] = []

    first_incumbent_cost = math.inf
    first_incumbent_expanded = 0
    first_incumbent_runtime_s = float("nan")
    first_incumbent_path: List[CanonicalState] = []

    expanded_nodes = 0
    refinement_limit_reached = False
    budget_hit = False
    phases: List[ARAPhaseResult] = []

    def h_of(s: AugmentedState) -> float:
        return _heuristic(s, goal, terrain, config, 1.0, None, False, distance_reference_m)

    for epsilon in epsilon_schedule:
        phase_start_expanded = expanded_nodes
        phase_g_improvements = 0
        phase_first_incumbent_improvement_expansion: Optional[int] = None
        phase_last_incumbent_improvement_expansion: Optional[int] = None

        # OPEN = OPEN u INCONS; INCONS cleared; keys recomputed with the new
        # epsilon; CLOSED cleared -- g/parent/incumbent all persist untouched.
        open_members = open_members | incons_members
        incons_members = set()
        closed_this_phase: set = set()

        heap: List[Tuple[float, int, AugmentedState, float]] = []
        for s in open_members:
            h_val = h_of(s)
            heapq.heappush(heap, (g[s] + epsilon * h_val, next(counter), s, g[s] + h_val))

        phase_complete = False
        while heap:
            f_w_top, _, s_top, _ = heap[0]
            if s_top in closed_this_phase:
                heapq.heappop(heap)
                continue
            if incumbent_cost < math.inf and f_w_top >= incumbent_cost:
                phase_complete = True
                break

            _, _, s, _ = heapq.heappop(heap)
            if s in closed_this_phase:
                continue  # stale duplicate entry for an already-expanded state

            closed_this_phase.add(s)
            open_members.discard(s)
            expanded_nodes += 1

            if expanded_nodes >= max_expansions_cumulative:
                budget_hit = True
                break

            neighbors, _rej_counts, _gen, _rej, _corr_rej, _z_rej = _generate_neighbors(
                s, primitives, terrain, config, min_search_altitude_msl, max_search_altitude_msl,
                primitive_cache, cache_stats, distance_reference_m, corridor_mask, z_guide_grid,
                z_guide_tolerance_m, True,  # freeze_history hardwired for Stage 38
                fine_precompute,
            )
            g_s = g[s]
            for neighbor, edge_cost in neighbors:
                tentative_g = g_s + edge_cost
                if tentative_g < g.get(neighbor, math.inf):
                    g[neighbor] = tentative_g
                    parent[neighbor] = s
                    phase_g_improvements += 1

                    if _state_in_goal_region((neighbor[0], neighbor[1], neighbor[2]), goal, terrain, config):
                        # Sink, exactly like sgoal in classic ARA* -- g/parent update above is
                        # its "relaxation"; it is never pushed to OPEN/INCONS, since no path
                        # needs to continue past the goal region's entry point.
                        if tentative_g < incumbent_cost:
                            incumbent_cost = tentative_g
                            incumbent_state = neighbor
                            incumbent_path_snapshot = _reconstruct_path(parent, start_aug, neighbor)
                            if phase_first_incumbent_improvement_expansion is None:
                                phase_first_incumbent_improvement_expansion = expanded_nodes
                            phase_last_incumbent_improvement_expansion = expanded_nodes
                            if first_incumbent_cost == math.inf:
                                first_incumbent_cost = tentative_g
                                first_incumbent_expanded = expanded_nodes
                                first_incumbent_runtime_s = time.perf_counter() - t0
                                first_incumbent_path = _reconstruct_path(parent, start_aug, neighbor)
                    elif neighbor in closed_this_phase:
                        incons_members.add(neighbor)
                    else:
                        open_members.add(neighbor)
                        h_val = h_of(neighbor)
                        heapq.heappush(heap, (tentative_g + epsilon * h_val, next(counter), neighbor, tentative_g + h_val))

        if budget_hit:
            refinement_limit_reached = True
        elif not heap:
            phase_complete = True  # OPEN exhausted -- nothing left could ever beat the incumbent

        active_frontier = open_members | incons_members
        if active_frontier:
            diagnostic_lb = min(g[s] + h_of(s) for s in active_frontier)
        else:
            diagnostic_lb = math.inf
        diagnostic_bound_ratio = (
            incumbent_cost / diagnostic_lb
            if incumbent_cost < math.inf and 0.0 < diagnostic_lb < math.inf
            else float("nan")
        )

        if incumbent_state is not None:
            phase_path = list(incumbent_path_snapshot)
            phase_xyz = [state_to_xyz(s, terrain, config) for s in phase_path]
            xy_length_m = sum(
                math.hypot(phase_xyz[i + 1][0] - phase_xyz[i][0], phase_xyz[i + 1][1] - phase_xyz[i][1])
                for i in range(len(phase_xyz) - 1)
            )
            alt_metrics = _path_altitude_metrics(phase_path, terrain, config)
            reversal_metrics = _path_vertical_reversal_metrics(phase_path, primitives, config)
        else:
            xy_length_m = float("nan")
            alt_metrics = {"geometric_path_length": float("nan"), "minimum_aircraft_msl": float("nan"),
                           "maximum_aircraft_msl": float("nan"), "average_aircraft_msl": float("nan"),
                           "total_climb_m": float("nan"), "total_descent_m": float("nan")}
            reversal_metrics = {"total_vertical_reversal_count": 0}

        phases.append(ARAPhaseResult(
            epsilon=epsilon,
            added_expansions=expanded_nodes - phase_start_expanded,
            cumulative_expansions=expanded_nodes,
            cumulative_runtime_s=time.perf_counter() - t0,
            open_size_at_end=len(open_members),
            incons_size_at_end=len(incons_members),
            closed_size_this_phase=len(closed_this_phase),
            g_value_improvement_count=phase_g_improvements,
            incumbent_available=incumbent_cost < math.inf,
            incumbent_cost=incumbent_cost,
            diagnostic_lower_bound=diagnostic_lb,
            diagnostic_bound_ratio=diagnostic_bound_ratio,
            xy_length_m=xy_length_m,
            length_3d_m=alt_metrics["geometric_path_length"],
            min_msl=alt_metrics["minimum_aircraft_msl"],
            mean_msl=alt_metrics["average_aircraft_msl"],
            max_msl=alt_metrics["maximum_aircraft_msl"],
            total_climb_m=alt_metrics["total_climb_m"],
            total_descent_m=alt_metrics["total_descent_m"],
            vertical_reversal_count=reversal_metrics["total_vertical_reversal_count"],
            phase_complete=phase_complete,
            first_incumbent_improvement_expansion=phase_first_incumbent_improvement_expansion,
            last_incumbent_improvement_expansion=phase_last_incumbent_improvement_expansion,
            incumbent_path=phase_path if incumbent_state is not None else [],
        ))

        if budget_hit:
            break

    total_runtime_s = time.perf_counter() - t0
    final_incumbent_path = (
        list(incumbent_path_snapshot) if incumbent_state is not None else []
    )

    return ARASearchResult(
        path_found=incumbent_cost < math.inf,
        refinement_limit_reached=refinement_limit_reached,
        first_incumbent_cost=first_incumbent_cost,
        first_incumbent_expanded=first_incumbent_expanded,
        first_incumbent_runtime_s=first_incumbent_runtime_s,
        first_incumbent_path=first_incumbent_path,
        final_incumbent_cost=incumbent_cost,
        final_incumbent_path=final_incumbent_path,
        total_expanded=expanded_nodes,
        total_runtime_s=total_runtime_s,
        phases=phases,
    )
