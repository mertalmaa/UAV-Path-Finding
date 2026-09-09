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

from planner.config import DEFAULT_CONFIG, PlannerConfig
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


def _heuristic(
    current: AugmentedState,
    goal: CanonicalState,
    terrain: TerrainQuery,
    config: PlannerConfig,
    cost_multiplier: float = 1.0,
    min_possible_msl: Optional[float] = None,
    use_vertical_reachability: bool = False,
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
    """
    x1, y1, z1 = state_to_xyz((current[0], current[1], current[2]), terrain, config)
    x2, y2, z2 = state_to_xyz(goal, terrain, config)
    d3d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
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


def compute_edge_cost(
    primitive: MotionPrimitive,
    start_altitude_msl: float,
    previous_vertical_trend: int,
    previous_trend_bucket: int,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> float:
    """The actual A* edge cost for applying one primitive from a given
    altitude and standing (vertical_trend, trend_age_bucket).

    Public (not just an internal detail of _generate_neighbors) so callers
    -- validation/diagnostic scripts included -- can cost a hypothetical
    primitive sequence without duplicating this formula.

        base_cost = geometric_cost * (1 + msl_cost_weight * altitude_scaled)
        reversal_cost = vertical_reversal_cost_weight * abs(dz) * reversal_factor
                         (0 unless this primitive is a direction reversal)
        edge_cost = base_cost + reversal_cost

    No abs(delta_z) term outside of a reversal, and no term at all for a
    reversal starting from an already-MATURE standing trend -- see
    _next_trend_and_bucket.
    """
    end_altitude_msl = start_altitude_msl + primitive.dz_m
    geometric_cost = math.sqrt(primitive.horizontal_distance_m ** 2 + primitive.dz_m ** 2)
    mean_altitude_msl = (start_altitude_msl + end_altitude_msl) / 2.0
    altitude_scaled = _altitude_scaled(mean_altitude_msl, config)
    base_cost = geometric_cost * (1.0 + config.msl_cost_weight * altitude_scaled)

    _, _, is_reversal, reversal_factor = _next_trend_and_bucket(
        previous_vertical_trend, previous_trend_bucket, primitive, config
    )
    reversal_cost = config.vertical_reversal_cost_weight * abs(primitive.dz_m) * reversal_factor if is_reversal else 0.0
    return base_cost + reversal_cost


def validate_and_cost_path(
    path: List[CanonicalState],
    primitives: List[MotionPrimitive],
    terrain: TerrainQuery,
    config: PlannerConfig = DEFAULT_CONFIG,
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
    """
    if len(path) < 2:
        return False, math.inf
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    trend, bucket = 0, BUCKET_SHORT
    total_cost = 0.0
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        if prim is None:
            return False, math.inf
        start_xyz = state_to_xyz((r1, c1, z1), terrain, config)
        result = evaluate_primitive(start_xyz, prim, terrain, config)
        if not result.valid:
            return False, math.inf
        total_cost += compute_edge_cost(prim, start_xyz[2], trend, bucket, config)
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
):
    """current -> primitive -> candidate -> bounds -> altitude bounds -> evaluate_primitive (or cache) -> neighbor.

    primitive_cache holds PHYSICAL feasibility results only, keyed by
    (row, col, z_index, primitive_id) -- never by vertical_trend or
    trend_age_units, because whether a primitive is safe from a given
    physical position never depends on search history (see module
    docstring). Pass None to disable caching entirely (use_primitive_cache
    =False). Cost (compute_edge_cost) and the next (trend, age) are ALWAYS
    computed fresh per augmented state below, never cached -- those DO
    depend on history/config preference.

    Returns (accepted, rejected_reason_counts, generated_count, rejected_count).
    accepted is a list of (neighbor_augmented_state, edge_cost).
    """
    row, col, z_index, prev_trend, prev_bucket = state
    start_xyz = state_to_xyz((row, col, z_index), terrain, config)

    accepted: List[Tuple[AugmentedState, float]] = []
    rejected_counts: Dict[str, int] = {}
    generated = 0
    rejected = 0

    for prim in primitives:
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

        if not (min_search_altitude_msl <= new_z_msl <= max_search_altitude_msl):
            rejected += 1
            rejected_counts["altitude_search_bounds"] = rejected_counts.get("altitude_search_bounds", 0) + 1
            continue

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
        edge_cost = compute_edge_cost(prim, start_xyz[2], prev_trend, prev_bucket, config)
        next_trend, next_bucket, _, _ = _next_trend_and_bucket(prev_trend, prev_bucket, prim, config)

        neighbor_state: AugmentedState = (new_row, new_col, new_z_index, next_trend, next_bucket)
        accepted.append((neighbor_state, edge_cost))

    return accepted, rejected_counts, generated, rejected


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
    """
    if primitives is None:
        primitives = build_primitive_set(config)

    t0 = time.perf_counter()
    counter = itertools.count()

    primitive_cache: Optional[Dict[PrimitiveCacheKey, PrimitiveEvalResult]] = {} if use_primitive_cache else None
    cache_stats = {"hits": 0, "misses": 0, "actual_calls": 0}

    # Per-base-key Pareto frontier of (age, g) pairs, age ascending / g strictly
    # increasing on the active front (see module docstring / project.md "Stage 16").
    frontier: Dict[BaseKey, List[Tuple[int, float]]] = {}
    dominance_stats = {"checks": 0, "pruned": 0, "removed": 0, "pop_skipped": 0, "max_frontier": 0}

    # Computed ONCE per search call, never inside the loop below.
    min_possible_msl = float("nan")
    cost_multiplier = 1.0
    vertical_reachability_active = False
    if use_msl_lower_bound_heuristic:
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

    start_aug: AugmentedState = (start[0], start[1], start[2], 0, BUCKET_SHORT)

    g_score: Dict[AugmentedState, float] = {start_aug: 0.0}
    came_from: Dict[AugmentedState, AugmentedState] = {}
    closed = set()

    if use_dominance_pruning:
        frontier[start_aug[:4]] = [(0, 0.0)]

    h_start = _heuristic(start_aug, goal, terrain, config, cost_multiplier, heuristic_min_msl,
                          vertical_reachability_active)
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

        if (current[0], current[1], current[2]) == goal:
            solution_cost = g_score[current]
            if first_solution_cost == math.inf:
                first_solution_cost = solution_cost
                first_solution_expanded = expanded_nodes
                first_solution_runtime_s = time.perf_counter() - t0
            if use_incumbent_pruning and solution_cost < incumbent_cost:
                incumbent_cost = solution_cost
                incumbent_stats["updates"] += 1
                incumbent_path = _reconstruct_path(came_from, start_aug, current)
            if not bounded_mode:
                status = "success"
                goal_state = current
                break
            # bounded_mode: don't stop here -- a tighter certified solution may
            # still be reachable; fall through to the certificate check below.

        if bounded_mode and incumbent_cost < math.inf:
            current_lb = _current_lower_bound(lb_heap, closed)
            if current_lb < math.inf and incumbent_cost <= target_suboptimality * current_lb:
                bounded_termination_triggered = True
                status = "success"
                break

        if max_expansions is not None and expanded_nodes >= max_expansions:
            status = "search_limit_reached"
            break

        neighbors, rej_counts, gen_count, rej_count = _generate_neighbors(
            current, primitives, terrain, config, min_search_altitude_msl, max_search_altitude_msl,
            primitive_cache, cache_stats,
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
                                        heuristic_min_msl, vertical_reachability_active)
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
    )
