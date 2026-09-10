"""Stage 35: Simplified Coarse 3D A* -- a global GUIDE path over the 90m
coarse MAX-terrain DEM (planner.coarse.build_coarse_dem).

Deliberately much simpler than planner.astar's fine-grid search
(Stage 14-33): state is plain (row, col, z_index) -- no vertical trend, no
trend-age bucket, no dominance pruning, no reversal cost, no heading/
weighted-A*/incumbent machinery. This module does not touch planner.astar
at all; it is a separate, self-contained search meant to produce a coarse
guide, not a final flight path.

Safety reuses the SAME primitives the fine planner already validated --
planner.primitives.build_primitive_set/evaluate_primitive, which delegate
to planner.agl.evaluate_agl and planner.transition.evaluate_transition,
completely unmodified. Passing a PlannerConfig with xy_resolution_m=90/
z_step_m=40 and a TerrainQuery wrapping the coarse MAX ROIData makes those
same functions correctly enforce "aircraft_msl - coarse_MAX_terrain >=
min_agl_m" with zero reimplementation -- MIN/MEAN/RELIEF (planner.coarse.
CoarseTerrainStats) are never read here; they play no role in either cost
or safety, only in post-hoc path diagnostics (see coarse_path_relief_stats
below, and the calling script).

Cost (Stage 32's normalized objective, simplified -- no reversal term,
since this state has no trend memory to define one against):

    dC_distance = ds_3d / D_ref
    dC_altitude = dC_distance * max(0, mean_aircraft_msl - altitude_reference_msl) / altitude_scale_m
    edge_cost   = w_distance * dC_distance + w_altitude * dC_altitude

Heuristic: h = D3D(current, goal) / D_ref -- admissible because edge_cost
>= w_distance * dC_distance always (altitude term is non-negative), and
summing dC_distance over any path from n to goal is >= D3D(n, goal)/D_ref
by the ordinary Euclidean triangle inequality (same argument as planner.
astar's normalized-mode heuristic, Stage 32). Standard (unweighted,
epsilon=1.0) A*, no incumbent, no certified bound -- first goal pop is
already optimal.
"""
import heapq
import itertools
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from planner.astar import PrimitiveId, _primitive_id, state_to_xyz, z_index_to_msl
from planner.config import PlannerConfig
from planner.mission import MissionPolicy
from planner.primitives import MotionPrimitive, Point3, build_primitive_set, evaluate_primitive
from planner.terrain import TerrainQuery

CoarseState = Tuple[int, int, int]  # (row, col, z_index) -- ONLY this, nothing augmented.
PrecomputeKey = Tuple[int, int, PrimitiveId]  # (row, col, primitive_id) -- deliberately NO z_index (Stage 35.1)


@dataclass
class EndpointLiftResult:
    row: int
    col: int
    z_index: int
    msl: float
    original_msl: float
    cell_max_terrain_msl: float
    required_msl: float  # cell_max_terrain_msl + min_agl_m
    was_lifted: bool


def lift_endpoint_if_unsafe(
    row: int, col: int, original_msl: float, terrain: TerrainQuery, config: PlannerConfig,
) -> EndpointLiftResult:
    """Stage 35 endpoint policy: if the coarse MAX terrain at (row, col)
    would make `original_msl` unsafe (original_msl - cell_max_terrain <
    min_agl_m), lift the endpoint's altitude to the SMALLEST z-grid level
    that is both >= original_msl AND satisfies cell_max_terrain +
    min_agl_m -- never lower than the original mission altitude. A
    surrogate for this coarse search only; the fine mission goal (e.g.
    3760m) is untouched by this function.
    """
    cell = terrain.elevation_at_rowcol(row, col)
    if not cell.valid:
        raise ValueError(f"coarse endpoint ({row},{col}) has no valid terrain ({cell.reason})")
    required_msl = cell.elevation + config.min_agl_m
    if original_msl >= required_msl:
        z_index = round(original_msl / config.z_step_m)
        return EndpointLiftResult(row, col, z_index, original_msl, original_msl, cell.elevation, required_msl, False)
    lifted_index = math.ceil(required_msl / config.z_step_m)
    lifted_msl = lifted_index * config.z_step_m
    return EndpointLiftResult(row, col, lifted_index, lifted_msl, original_msl, cell.elevation, required_msl, True)


def compute_coarse_distance_reference(
    start: CoarseState, goal: CoarseState, terrain: TerrainQuery, config: PlannerConfig,
) -> float:
    """D_ref: straight-line 3D distance between the (possibly endpoint-
    lifted) coarse start/goal, computed ONCE per search call -- never
    per-edge/per-state."""
    x1, y1, z1 = state_to_xyz(start, terrain, config)
    x2, y2, z2 = state_to_xyz(goal, terrain, config)
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


@dataclass
class PrecomputedPrimitiveSafety:
    static_invalid: bool  # True if out_of_bounds/nodata anywhere along the path -- independent of start_msl
    static_invalid_reason: str  # "" if not static_invalid, else "out_of_bounds" or "nodata"
    required_start_msl: float  # meaningful only if not static_invalid; -inf if no sample constrained it


@dataclass
class PrecomputeStats:
    entry_count: int
    preprocessing_runtime_s: float
    static_invalid_count: int
    approx_memory_bytes: int


def precompute_coarse_primitive_safety(
    terrain: TerrainQuery,
    primitives: List[MotionPrimitive],
    config: PlannerConfig,
) -> Tuple[Dict[PrecomputeKey, PrecomputedPrimitiveSafety], PrecomputeStats]:
    """Stage 35.1: for every (row, col, primitive_id) on the coarse grid,
    precompute the exact same safety decision evaluate_primitive() would
    make, EXCEPT that the altitude-dependent part (AGL) is reduced to a
    single scalar per (row, col, primitive_id) -- required_start_msl --
    since terrain elevation and each sample's vertical offset are BOTH
    fixed once terrain and primitive geometry are fixed:

        aircraft_msl(i) = start_msl + primitive.dz_m * t_i
        valid_i iff aircraft_msl(i) - terrain_elevation(i) >= min_agl_m
              iff start_msl >= terrain_elevation(i) + min_agl_m - primitive.dz_m*t_i

    required_start_msl = max over samples i of the right-hand side above --
    a state is valid (for AGL) at THIS (row, col, primitive) iff its own
    start_msl >= required_start_msl; no per-sample terrain query is needed
    at search time anymore, just one dict lookup + one comparison.

    Endpoint transition validity (climb/descent angle) is NOT re-checked
    here: every primitive in `primitives` already passed that check once,
    permanently, inside build_primitive_set() -- it is a static property
    of the primitive's own geometry, never of position or start_msl, so
    re-testing it per (row, col) would be redundant work, not extra safety.

    out_of_bounds/nodata are still genuinely position-dependent (which
    primitive from which cell) but NOT start_msl-dependent -- if either
    occurs anywhere along a primitive's sampled path, that (row, col,
    primitive_id) is marked static_invalid regardless of altitude
    (matches evaluate_primitive's own "can't evaluate AGL without a real
    elevation" behavior for those two reasons).

    Z_index is deliberately NOT part of the cache key (per spec) -- the
    whole point is that one entry covers every possible start altitude
    for that (row, col, primitive).
    """
    t0 = time.perf_counter()
    cache: Dict[PrecomputeKey, PrecomputedPrimitiveSafety] = {}
    static_invalid_count = 0

    for row in range(terrain.roi.height):
        for col in range(terrain.roi.width):
            x1, y1 = terrain.rowcol_to_xy(row, col)
            for prim in primitives:
                dx = prim.dcol * config.xy_resolution_m
                dy = -prim.drow * config.xy_resolution_m
                n_intervals = max(1, math.ceil(prim.horizontal_distance_m / config.primitive_sample_spacing_m))

                static_invalid = False
                reason = ""
                required_start_msl = -math.inf
                for i in range(n_intervals + 1):
                    t = i / n_intervals
                    x, y = x1 + dx * t, y1 + dy * t
                    sample = terrain.query(x, y)
                    if not sample.valid:
                        static_invalid = True
                        reason = sample.reason
                        break
                    vertical_offset = prim.dz_m * t
                    needed = sample.elevation + config.min_agl_m - vertical_offset
                    if needed > required_start_msl:
                        required_start_msl = needed

                if static_invalid:
                    static_invalid_count += 1
                cache[(row, col, _primitive_id(prim))] = PrecomputedPrimitiveSafety(
                    static_invalid=static_invalid, static_invalid_reason=reason,
                    required_start_msl=required_start_msl,
                )

    preprocessing_runtime_s = time.perf_counter() - t0
    # Rough per-entry footprint: 3 Python object fields on a dataclass instance
    # plus the dict's own bucket/key overhead -- not exact, just an order-of-
    # magnitude estimate for reporting (spec asks for "approximate" memory).
    approx_bytes_per_entry = 200
    stats = PrecomputeStats(
        entry_count=len(cache), preprocessing_runtime_s=preprocessing_runtime_s,
        static_invalid_count=static_invalid_count,
        approx_memory_bytes=len(cache) * approx_bytes_per_entry,
    )
    return cache, stats


def precomputed_primitive_validity(
    cache: Dict[PrecomputeKey, PrecomputedPrimitiveSafety],
    row: int, col: int, primitive: MotionPrimitive, start_msl: float,
) -> Tuple[bool, str]:
    """O(1) replacement for evaluate_primitive()'s validity decision, using
    the precomputed cache -- see precompute_coarse_primitive_safety()."""
    entry = cache[(row, col, _primitive_id(primitive))]
    if entry.static_invalid:
        return False, entry.static_invalid_reason
    if start_msl >= entry.required_start_msl:
        return True, "ok"
    return False, "below_min_agl"


def compute_coarse_edge_cost(
    primitive: MotionPrimitive,
    start_altitude_msl: float,
    config: PlannerConfig,
    distance_reference_m: float,
    altitude_reference_msl: float,
    altitude_scale_m: float,
    w_distance: float,
    w_altitude: float,
) -> Tuple[float, float, float]:
    """Returns (total_cost, distance_component, altitude_component) --
    both components already weighted, so total == distance_component +
    altitude_component. No reversal term (see module docstring)."""
    policy = MissionPolicy(
        altitude_reference_msl=altitude_reference_msl,
        altitude_scale_m=altitude_scale_m,
        w_distance=w_distance,
        w_altitude=w_altitude,
        w_smoothness=0.0,
    )
    components = policy.edge_components(
        math.hypot(primitive.horizontal_distance_m, primitive.dz_m),
        start_altitude_msl,
        start_altitude_msl + primitive.dz_m,
        distance_reference_m,
    )
    return components.total, components.distance, components.altitude


def _coarse_heuristic(current: CoarseState, goal: CoarseState, terrain: TerrainQuery,
                       config: PlannerConfig, distance_reference_m: float, w_distance: float) -> float:
    x1, y1, z1 = state_to_xyz(current, terrain, config)
    x2, y2, z2 = state_to_xyz(goal, terrain, config)
    d3d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
    return w_distance * d3d / distance_reference_m


@dataclass
class CoarseSearchResult:
    success: bool
    status: str  # "success" | "no_path" | "search_limit_reached"
    path: List[CoarseState]
    total_cost: float
    total_distance_cost: float
    total_altitude_cost: float
    expanded_nodes: int
    generated_neighbors: int
    rejected_neighbors: int
    rejected_reason_counts: Dict[str, int] = field(default_factory=dict)
    max_open_size: int = 0
    runtime_s: float = 0.0
    distance_reference_m: float = float("nan")
    # FAIL diagnostics -- always tracked (cheap, observational).
    closest_state_to_goal: Optional[CoarseState] = None
    closest_distance_to_goal_m: float = float("inf")


def _generate_coarse_neighbors(
    state: CoarseState,
    primitives: List[MotionPrimitive],
    terrain: TerrainQuery,
    config: PlannerConfig,
    min_search_altitude_msl: float,
    max_search_altitude_msl: float,
    precomputed_safety: Optional[Dict[PrecomputeKey, PrecomputedPrimitiveSafety]] = None,
):
    """current -> primitive -> candidate -> bounds -> safety check -> neighbor.

    Safety check is evaluate_primitive() (the original per-sample terrain
    evaluator) when precomputed_safety is None -- the exact Stage 35
    behavior, byte-for-byte -- or precomputed_primitive_validity() (Stage
    35.1's O(1) cache lookup) otherwise. Both must make the identical
    accept/reject decision for the same (row, col, primitive, start_msl)
    -- see scripts/validate_coarse_precompute.py for the equivalence tests.

    No trend/bucket, no dominance. Returns (accepted, rejected_reason_counts,
    generated, rejected). accepted is a list of (neighbor_state, primitive).
    """
    row, col, z_index = state
    start_xyz = state_to_xyz(state, terrain, config)
    start_msl = start_xyz[2]

    accepted = []
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

        if precomputed_safety is not None:
            valid, reason = precomputed_primitive_validity(precomputed_safety, row, col, prim, start_msl)
        else:
            eval_result = evaluate_primitive(start_xyz, prim, terrain, config)
            valid, reason = eval_result.valid, eval_result.reason
        if not valid:
            rejected += 1
            rejected_counts[reason] = rejected_counts.get(reason, 0) + 1
            continue

        neighbor_state: CoarseState = (new_row, new_col, new_z_index)
        accepted.append((neighbor_state, prim))

    return accepted, rejected_counts, generated, rejected


def coarse_astar_search(
    start: CoarseState,
    goal: CoarseState,
    terrain: TerrainQuery,
    min_search_altitude_msl: float,
    max_search_altitude_msl: float,
    config: PlannerConfig,
    primitives: Optional[List[MotionPrimitive]] = None,
    max_expansions: Optional[int] = None,
    distance_reference_m: Optional[float] = None,
    altitude_reference_msl: Optional[float] = None,
    altitude_scale_m: Optional[float] = None,
    w_distance: Optional[float] = None,
    w_altitude: Optional[float] = None,
    precomputed_safety: Optional[Dict[PrecomputeKey, PrecomputedPrimitiveSafety]] = None,
    epsilon_search: float = 1.0,
) -> CoarseSearchResult:
    """Standard (epsilon=1.0 default, no dominance/incumbent) A* over
    plain (row, col, z_index) states on the coarse grid. Stops at the
    first goal-position pop.

    precomputed_safety (default None, Stage 35.1): pass the result of
    precompute_coarse_primitive_safety() to replace evaluate_primitive()'s
    per-sample terrain evaluation with an O(1) cache lookup for every
    safety decision -- see that function's docstring for the exact
    equivalence argument. None (the default) reproduces Stage 35's search
    byte-for-byte; this parameter changes NOTHING about search order,
    cost, heuristic, or which states get explored -- only how fast the
    safety check underneath each candidate resolves.

    epsilon_search (default 1.0, Stage 35.2): the ONLY thing this touches
    is the open-heap PRIORITY, f = g + epsilon_search*h, exactly like the
    fine planner's Weighted A* (Stage 25) -- g_score/g_distance/g_altitude
    (the actual accumulated cost) are always the true unweighted edge
    costs, never scaled by epsilon_search; the returned path's cost is
    the real cost of the physical path found, not an inflated one.
    epsilon_search=1.0 reproduces Stage 35/35.1 exactly (multiplying by
    1.0 changes no bits). epsilon_search>1 biases expansion toward the
    goal, generally finding A solution in fewer expansions at the cost of
    it not necessarily being the cheapest one -- acceptable here because
    this coarse search is only ever a GUIDE, not a final optimized path
    (see project.md "Stage 35.2"). No reopening logic was added (unlike
    Stage 25) -- this is a deliberately minimal ordering-only change, so
    a state closed under a suboptimal g stays closed; this can yield a
    higher-cost (but still complete and safe) path, never an unsafe one,
    since safety is entirely decided by precomputed_safety/evaluate_
    primitive, untouched by this parameter.
    """
    if primitives is None:
        primitives = build_primitive_set(config)
    if distance_reference_m is None:
        distance_reference_m = compute_coarse_distance_reference(start, goal, terrain, config)
    # Mission semantics come from the shared normalized config unless a
    # caller explicitly supplies an override.  No location-specific altitude
    # reference is embedded in the coarse planner.
    if altitude_reference_msl is None:
        if config.altitude_reference_msl is None:
            raise ValueError("coarse normalized objective requires altitude_reference_msl")
        altitude_reference_msl = config.altitude_reference_msl
    if altitude_scale_m is None:
        altitude_scale_m = config.normalized_altitude_scale_m
    if w_distance is None:
        w_distance = config.normalized_w_distance
    if w_altitude is None:
        w_altitude = config.normalized_w_altitude

    t0 = time.perf_counter()
    counter = itertools.count()

    g_score: Dict[CoarseState, float] = {start: 0.0}
    g_distance: Dict[CoarseState, float] = {start: 0.0}
    g_altitude: Dict[CoarseState, float] = {start: 0.0}
    came_from: Dict[CoarseState, CoarseState] = {}
    closed = set()

    h_start = _coarse_heuristic(start, goal, terrain, config, distance_reference_m, w_distance)
    open_heap = [(epsilon_search * h_start, next(counter), start)]
    max_open_size = 1

    goal_x, goal_y, goal_z = state_to_xyz(goal, terrain, config)
    closest_distance_to_goal_m = math.inf
    closest_state_to_goal: Optional[CoarseState] = None

    expanded_nodes = 0
    generated_neighbors = 0
    rejected_neighbors = 0
    rejected_reason_counts: Dict[str, int] = {}
    status = "no_path"
    goal_state: Optional[CoarseState] = None

    while open_heap:
        max_open_size = max(max_open_size, len(open_heap))
        _, _, current = heapq.heappop(open_heap)

        if current in closed:
            continue
        closed.add(current)
        expanded_nodes += 1

        x_c, y_c, z_c = state_to_xyz(current, terrain, config)
        dist = math.sqrt((x_c - goal_x) ** 2 + (y_c - goal_y) ** 2 + (z_c - goal_z) ** 2)
        if dist < closest_distance_to_goal_m:
            closest_distance_to_goal_m = dist
            closest_state_to_goal = current

        if current == goal:
            status = "success"
            goal_state = current
            break

        if max_expansions is not None and expanded_nodes >= max_expansions:
            status = "search_limit_reached"
            break

        neighbors, rej_counts, gen_count, rej_count = _generate_coarse_neighbors(
            current, primitives, terrain, config, min_search_altitude_msl, max_search_altitude_msl,
            precomputed_safety,
        )
        generated_neighbors += gen_count
        rejected_neighbors += rej_count
        for reason, cnt in rej_counts.items():
            rejected_reason_counts[reason] = rejected_reason_counts.get(reason, 0) + cnt

        current_g = g_score[current]
        for neighbor_state, prim in neighbors:
            if neighbor_state in closed:
                continue
            edge_cost, dist_cost, alt_cost = compute_coarse_edge_cost(
                prim, z_c, config, distance_reference_m,
                altitude_reference_msl, altitude_scale_m, w_distance, w_altitude,
            )
            tentative_g = current_g + edge_cost
            if tentative_g < g_score.get(neighbor_state, math.inf):
                g_score[neighbor_state] = tentative_g
                g_distance[neighbor_state] = g_distance[current] + dist_cost
                g_altitude[neighbor_state] = g_altitude[current] + alt_cost
                came_from[neighbor_state] = current
                h_val = _coarse_heuristic(neighbor_state, goal, terrain, config, distance_reference_m, w_distance)
                f_score = tentative_g + epsilon_search * h_val
                heapq.heappush(open_heap, (f_score, next(counter), neighbor_state))

    runtime_s = time.perf_counter() - t0

    if status == "success" and goal_state is not None:
        path = [goal_state]
        cur = goal_state
        while cur != start:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        total_cost = g_score[goal_state]
        total_distance_cost = g_distance[goal_state]
        total_altitude_cost = g_altitude[goal_state]
    else:
        path = []
        total_cost = float("nan")
        total_distance_cost = float("nan")
        total_altitude_cost = float("nan")

    return CoarseSearchResult(
        success=(status == "success"),
        status=status,
        path=path,
        total_cost=total_cost,
        total_distance_cost=total_distance_cost,
        total_altitude_cost=total_altitude_cost,
        expanded_nodes=expanded_nodes,
        generated_neighbors=generated_neighbors,
        rejected_neighbors=rejected_neighbors,
        rejected_reason_counts=rejected_reason_counts,
        max_open_size=max_open_size,
        runtime_s=runtime_s,
        distance_reference_m=distance_reference_m,
        closest_state_to_goal=closest_state_to_goal,
        closest_distance_to_goal_m=closest_distance_to_goal_m,
    )
