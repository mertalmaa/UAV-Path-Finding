"""Stage 38.5 shared diagnostic library. Diagnostic-only -- planner/astar.py
and planner/primitives.py are never modified; this reuses their public
pieces (evaluate_primitive, evaluate_transition, compute_edge_cost,
_heuristic, _state_in_goal_region, state_to_xyz, _reconstruct_path,
_path_altitude_metrics) and adds:
  - a synthetic F-WIDE terrain builder
  - an ADAPTIVE max-feasible vertical primitive generator (per-state, not
    a fixed precomputed list)
  - a single standalone ARA* loop parametrized by a neighbor-generation
    strategy (fixed primitive list, OR adaptive), so fixed vs adaptive
    comparisons run through IDENTICAL search code.
"""
import dataclasses as dc
import heapq
import itertools
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
from affine import Affine

import os; sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.astar import (
    AugmentedState,
    CanonicalState,
    BUCKET_SHORT,
    _heuristic,
    _path_altitude_metrics,
    _reconstruct_path,
    _state_in_goal_region,
    ara_star_search,
    compute_distance_reference,
    compute_edge_cost,
    msl_to_z_index,
    state_to_xyz,
    z_index_to_msl,
)
from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.mission import production_mission_policy
from planner.primitives import MotionPrimitive, DIRECTIONS, build_primitive_set, evaluate_primitive
from planner.roi import ROIData
from planner.terrain import TerrainQuery
from planner.transition import evaluate_transition
from planner.agl import evaluate_agl

NODATA = -9999.0


# ---------------------------------------------------------------------------
# terrain builders
# ---------------------------------------------------------------------------

def make_roi(elevation):
    h, w = elevation.shape
    return ROIData(elevation.astype(np.float32), Affine(30, 0, 0, 0, -30, h * 30), "EPSG:32636", w, h,
                   (0, 0, w * 30, h * 30), (30, 30), NODATA)


def original_fgh_specs():
    f = np.full((17, 65), 200.0)
    f[9:12, :] = 100.0
    g = np.full((19, 65), 260.0)
    g[6:9, :] = 180.0
    g[12:15, :] = 80.0
    g[:, :13] = 80.0
    g[:, 52:] = 80.0
    h = np.full((13, 65), 100.0)
    h[:, :13] = 200.0
    h[:, 52:] = 200.0
    return {
        "F": dict(elevation=f, start_rc=(7, 4), goal_rc=(7, 60), start_msl=400.0, min_msl=300.0, max_msl=500.0,
                  preferred=lambda q: q["mean_msl"] < 375),
        "G": dict(elevation=g, start_rc=(7, 4), goal_rc=(7, 60), start_msl=400.0, min_msl=280.0, max_msl=500.0,
                  preferred=lambda q: q["mean_msl"] < 365),
        "H": dict(elevation=h, start_rc=(6, 4), goal_rc=(6, 60), start_msl=400.0, min_msl=300.0, max_msl=500.0,
                  preferred=lambda q: q["mean_msl"] < 375),
    }


def build_f_wide(cruise_cols=80, transition_margin_cols=0):
    """F-WIDE: identical vertical geometry/terrain contrast to original F
    (rows 9-11 = terrain 100, else terrain 200; start/goal both row 7,
    same 2-row lateral offset into the valley), but stretched so a full
    10-degree descent+cruise+climb comfortably fits with margin to spare.

    At 10deg/z_step=20m, one z_step costs n_cells=4 axial columns; 5 z_steps
    (100m, 400->300) each way = 20 columns descent + 20 columns climb = 40
    transition columns (see Table 1). cruise_cols is the additional flat
    low-MSL cruise budget requested on top of that.
    """
    transition_cols_each_way = 20  # 5 steps * 4 cells/step at 10 deg (Table 1)
    width = 8 + 2 * transition_cols_each_way + cruise_cols + 2 * transition_margin_cols  # + start/goal buffer
    elevation = np.full((17, width), 200.0)
    elevation[9:12, :] = 100.0
    goal_col = width - 5
    return dict(elevation=elevation, start_rc=(7, 4), goal_rc=(7, goal_col), start_msl=400.0, min_msl=300.0,
                max_msl=500.0, preferred=lambda q: q["mean_msl"] < 375,
                width=width, transition_cols_each_way=transition_cols_each_way, cruise_cols=cruise_cols)


# ---------------------------------------------------------------------------
# fixed-primitive generation (wraps production build_primitive_set exactly)
# ---------------------------------------------------------------------------

def make_config(max_climb_deg, max_descent_deg, altitude_reference_msl, w_altitude=1.25):
    base = dc.replace(DEFAULT_CONFIG, max_climb_angle_deg=max_climb_deg, max_descent_angle_deg=max_descent_deg,
                       normalized_w_altitude=w_altitude)
    return dc.replace(base, cost_mode="normalized", altitude_reference_msl=altitude_reference_msl,
                       normalized_w_distance=1.0, normalized_w_altitude=w_altitude,
                       normalized_altitude_scale_m=1000.0, goal_tolerance_xy_m=0.0, goal_tolerance_z_m=0.0)


# ---------------------------------------------------------------------------
# ADAPTIVE max-feasible vertical primitive generator (diagnostic only)
# ---------------------------------------------------------------------------

AXIAL_STEP = 30.0
DIAG_STEP = 30.0 * math.sqrt(2.0)


def _n_cells_for_angle(dz_abs, angle_deg, step):
    return max(1, math.ceil(dz_abs / math.tan(math.radians(angle_deg)) / step))


def max_feasible_vertical_candidates(row, col, z_index, direction, unit_drow, unit_dcol, config,
                                      terrain, max_angle_deg, kind_sign, include_moderate=False,
                                      search_span=6):
    """kind_sign=+1 climb, -1 descent. Returns list of (MotionPrimitive, PrimitiveEvalResult)
    -- at most 2 entries (max-feasible + optional moderate), never a large
    angle enumeration. Searches n_cells from the STEEPEST kinematically-
    allowed value upward (shallower) until evaluate_primitive validates
    (terrain/AGL/bounds/NoData) or search_span is exhausted -- "backing off"
    from the configured max angle exactly as Stage 38.5 sec.4B specifies.
    """
    is_diag = unit_drow != 0 and unit_dcol != 0
    step = DIAG_STEP if is_diag else AXIAL_STEP
    dz = kind_sign * config.z_step_m
    kind = "climb" if kind_sign > 0 else "descent"
    n_min = _n_cells_for_angle(abs(dz), max_angle_deg, step)
    start_xyz = state_to_xyz((row, col, z_index), terrain, config)

    out = []
    max_feasible = None
    for n_cells in range(n_min, n_min + search_span):
        horiz = n_cells * step
        prim = MotionPrimitive(direction, unit_drow * n_cells, unit_dcol * n_cells, dz, horiz, kind)
        result = evaluate_primitive(start_xyz, prim, terrain, config)
        if result.valid:
            max_feasible = (prim, result, n_cells)
            break
    if max_feasible is None:
        return out
    out.append((max_feasible[0], max_feasible[1]))
    if include_moderate:
        moderate_n = max_feasible[2] * 2
        if moderate_n != max_feasible[2]:
            horiz = moderate_n * step
            prim2 = MotionPrimitive(direction, unit_drow * moderate_n, unit_dcol * moderate_n, dz, horiz, kind)
            result2 = evaluate_primitive(start_xyz, prim2, terrain, config)
            if result2.valid:
                out.append((prim2, result2))
    return out


def generate_neighbors_adaptive(state, terrain, config, min_msl, max_msl, distance_reference_m,
                                 include_moderate=False, stats=None):
    row, col, z_index, prev_trend, prev_bucket = state
    start_xyz = state_to_xyz((row, col, z_index), terrain, config)
    accepted = []
    generated = 0
    valid = 0
    for direction, (unit_drow, unit_dcol) in DIRECTIONS.items():
        # LEVEL
        level = MotionPrimitive(direction, unit_drow, unit_dcol, 0.0,
                                 DIAG_STEP if (unit_drow and unit_dcol) else AXIAL_STEP, "level")
        generated += 1
        res = evaluate_primitive(start_xyz, level, terrain, config)
        if res.valid:
            valid += 1
            edge_cost = compute_edge_cost(level, start_xyz[2], prev_trend, prev_bucket, config,
                                           distance_reference_m, disable_reversal_cost=True)
            new_row, new_col = row + level.drow, col + level.dcol
            if terrain.in_bounds_rowcol(new_row, new_col):
                accepted.append(((new_row, new_col, z_index, prev_trend, prev_bucket), edge_cost))

        for kind_sign, msl_bound_ok in ((+1, z_index_to_msl(z_index, config) < max_msl),
                                         (-1, z_index_to_msl(z_index, config) > min_msl)):
            if not msl_bound_ok:
                continue
            cands = max_feasible_vertical_candidates(row, col, z_index, direction, unit_drow, unit_dcol,
                                                      config, terrain, config.max_climb_angle_deg if kind_sign > 0
                                                      else config.max_descent_angle_deg, kind_sign,
                                                      include_moderate=include_moderate)
            for prim, res in cands:
                generated += 1
                new_row = row + prim.drow
                new_col = col + prim.dcol
                dz_index = round(prim.dz_m / config.z_step_m)
                new_z_index = z_index + dz_index
                new_z_msl = z_index_to_msl(new_z_index, config)
                if not terrain.in_bounds_rowcol(new_row, new_col):
                    continue
                if not (min_msl <= new_z_msl <= max_msl):
                    continue
                valid += 1
                edge_cost = compute_edge_cost(prim, start_xyz[2], prev_trend, prev_bucket, config,
                                               distance_reference_m, disable_reversal_cost=True)
                accepted.append(((new_row, new_col, new_z_index, prev_trend, prev_bucket), edge_cost))
    if stats is not None:
        stats["generated"] += generated
        stats["valid"] += valid
        stats["calls"] += 1
    return accepted


# ---------------------------------------------------------------------------
# standalone ARA*, parametrized by neighbor-generation strategy
# ---------------------------------------------------------------------------

@dataclass
class PhaseResult:
    epsilon: float
    added_expansions: int
    cumulative_expansions: int
    cumulative_runtime_s: float
    incumbent_cost: float
    open_size_at_end: int
    incons_size_at_end: int
    phase_complete: bool
    first_incumbent_improvement_expansion: Optional[int]
    incumbent_path: List = field(default_factory=list)


def ara_star_generic(start, goal, terrain, min_msl, max_msl, config, epsilon_schedule, max_expansions_cumulative,
                      neighbor_fn: Callable, branching_stats: Optional[dict] = None):
    """neighbor_fn(state) -> list[(neighbor_state, edge_cost)]. Same phase
    loop/termination semantics as planner.astar.ara_star_search (verified
    to reproduce it bit-for-bit when neighbor_fn wraps the fixed primitive
    list -- see self-check in each experiment runner)."""
    t0 = time.perf_counter()
    counter = itertools.count()
    distance_reference_m = compute_distance_reference(start, goal, terrain, config)
    start_aug: AugmentedState = (start[0], start[1], start[2], 0, BUCKET_SHORT)

    g: Dict[AugmentedState, float] = {start_aug: 0.0}
    parent: Dict[AugmentedState, AugmentedState] = {}
    open_members = {start_aug}
    incons_members = set()
    incumbent_cost = math.inf
    incumbent_state = None
    incumbent_path_snapshot: List[CanonicalState] = []
    expanded_nodes = 0
    phases: List[PhaseResult] = []

    def h_of(s):
        return _heuristic(s, goal, terrain, config, 1.0, None, False, distance_reference_m)

    for epsilon in epsilon_schedule:
        phase_start_expanded = expanded_nodes
        phase_first_incumbent_improvement_expansion = None
        open_members = open_members | incons_members
        incons_members = set()
        closed_this_phase = set()
        heap: List[Tuple[float, int, AugmentedState, float]] = []
        for s in open_members:
            h_val = h_of(s)
            heapq.heappush(heap, (g[s] + epsilon * h_val, next(counter), s, g[s] + h_val))
        phase_complete = False
        budget_hit = False
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
                continue
            closed_this_phase.add(s)
            open_members.discard(s)
            expanded_nodes += 1
            if expanded_nodes >= max_expansions_cumulative:
                budget_hit = True
                break
            neighbors = neighbor_fn(s)
            g_s = g[s]
            for neighbor, edge_cost in neighbors:
                tentative_g = g_s + edge_cost
                if tentative_g < g.get(neighbor, math.inf):
                    g[neighbor] = tentative_g
                    parent[neighbor] = s
                    if _state_in_goal_region((neighbor[0], neighbor[1], neighbor[2]), goal, terrain, config):
                        if tentative_g < incumbent_cost:
                            incumbent_cost = tentative_g
                            incumbent_state = neighbor
                            incumbent_path_snapshot = _reconstruct_path(parent, start_aug, neighbor)
                            if phase_first_incumbent_improvement_expansion is None:
                                phase_first_incumbent_improvement_expansion = expanded_nodes
                    elif neighbor in closed_this_phase:
                        incons_members.add(neighbor)
                    else:
                        open_members.add(neighbor)
                        h_val = h_of(neighbor)
                        heapq.heappush(heap, (tentative_g + epsilon * h_val, next(counter), neighbor, tentative_g + h_val))
        if not heap and not budget_hit:
            phase_complete = True
        phases.append(PhaseResult(
            epsilon=epsilon, added_expansions=expanded_nodes - phase_start_expanded,
            cumulative_expansions=expanded_nodes, cumulative_runtime_s=time.perf_counter() - t0,
            incumbent_cost=incumbent_cost, open_size_at_end=len(open_members), incons_size_at_end=len(incons_members),
            phase_complete=phase_complete, first_incumbent_improvement_expansion=phase_first_incumbent_improvement_expansion,
            incumbent_path=list(incumbent_path_snapshot),
        ))
        if budget_hit:
            break
    return phases


def state_path_report(path, terrain, config, policy, d_ref):
    if not path:
        return None
    xyz = [state_to_xyz(s, terrain, config) for s in path]
    distance = altitude = 0.0
    for p1, p2 in zip(xyz, xyz[1:]):
        comp = policy.edge_components(math.dist(p1, p2), p1[2], p2[2], d_ref)
        distance += comp.distance
        altitude += comp.altitude
    alt = _path_altitude_metrics(path, terrain, config)
    xy = sum(math.hypot(p2[0] - p1[0], p2[1] - p1[1]) for p1, p2 in zip(xyz, xyz[1:]))
    length_3d = sum(math.dist(p1, p2) for p1, p2 in zip(xyz, xyz[1:]))
    return {"mean_msl": alt["average_aircraft_msl"], "min_msl": alt["minimum_aircraft_msl"],
            "xy": xy, "length_3d": length_3d, "distance": distance, "altitude": altitude,
            "total": distance + altitude, "climb": alt["total_climb_m"], "descent": alt["total_descent_m"],
            "n_states": len(path)}


def independent_safety_replay(path, terrain, config):
    """Independent replay: re-checks AGL/bounds/NoData/angle along the
    final path using evaluate_transition + terrain sampling, exactly like
    validate_stage38_3_mission_generalization.validate_polyline_safety but
    operating on discrete states via state_to_xyz."""
    if not path or len(path) < 2:
        return {"safe": True, "min_agl": math.inf, "max_angle": 0.0}
    xyz = [state_to_xyz(s, terrain, config) for s in path]
    min_agl = math.inf
    max_angle = 0.0
    ok = True
    for p1, p2 in zip(xyz, xyz[1:]):
        tr = evaluate_transition(p1, p2, config)
        max_angle = max(max_angle, tr.flight_path_angle_deg)
        if not tr.valid:
            ok = False
            continue
        n = max(1, math.ceil(tr.horizontal_distance_m / config.primitive_sample_spacing_m))
        for i in range(n + 1):
            t = i / n
            x = p1[0] + (p2[0] - p1[0]) * t
            y = p1[1] + (p2[1] - p1[1]) * t
            z = p1[2] + (p2[2] - p1[2]) * t
            agl_res = evaluate_agl(terrain, x, y, z, config)
            if not agl_res.valid:
                ok = False
                continue
            min_agl = min(min_agl, agl_res.agl_m)
    return {"safe": ok, "min_agl": min_agl, "max_angle": max_angle}
