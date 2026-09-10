"""Stage 38.4 diagnostic prototype -- NOT a production change.

Implements a bucketed anchor-key + guidance tie-break variant of ARA*,
entirely as a standalone function that calls the SAME underlying pieces
planner/astar.py's real ara_star_search uses (_generate_neighbors,
_heuristic, compute_edge_cost via _generate_neighbors, _reconstruct_path,
_path_altitude_metrics, state_to_xyz). planner/astar.py itself is not
modified.

Design (see chat write-up for the full correctness argument):
  - heap tuple: (bucket, guidance_score, counter, state, raw_weighted_key)
    where bucket = floor(raw_weighted_key / BUCKET_WIDTH) * BUCKET_WIDTH.
  - raw_weighted_key is EXACTLY g + epsilon*h, the same admissible quantity
    the production code uses -- never modified by guidance.
  - guidance_score comes from a cheap, INADMISSIBLE, XY-only reverse-Dijkstra
    "achievable low-MSL" potential (goal outward), used ONLY to choose which
    state to expand next among states sharing a bucket -- never to reject a
    successor, never to change g, never to change the termination test.
  - termination test uses `bucket` (a valid lower bound on every raw key in
    that bucket, since bucket = floor(...) <= raw_key), NOT the guidance-
    perturbed heap order and NOT any single popped raw_weighted_key. This
    keeps the same "nothing left in OPEN can beat incumbent" argument sound;
    worst case it delays termination by < BUCKET_WIDTH, it never terminates
    early.
  - guidance_fn=None must reproduce the production ara_star_search bit-for-
    bit (verified below against the already-recorded Stage 38.3 numbers)
    before any guidance-enabled run is trusted.
"""
import dataclasses
import heapq
import itertools
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
from affine import Affine

sys.path.insert(0, "/mnt/user-data/uploads/uav_pathfinder")

from planner.astar import (
    AugmentedState,
    CanonicalState,
    BUCKET_SHORT,
    _generate_neighbors,
    _heuristic,
    _path_altitude_metrics,
    _path_vertical_reversal_metrics,
    _reconstruct_path,
    _state_in_goal_region,
    ara_star_search,
    compute_distance_reference,
    msl_to_z_index,
    state_to_xyz,
)
from planner.config import DEFAULT_CONFIG
from planner.mission import production_mission_policy
from planner.primitives import build_primitive_set
from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0


# ---------------------------------------------------------------------------
# scenario setup (identical to validate_stage38_3_mission_generalization.py)
# ---------------------------------------------------------------------------

def make_roi(elevation):
    h, w = elevation.shape
    return ROIData(elevation.astype(np.float32), Affine(30, 0, 0, 0, -30, h * 30), "EPSG:32636", w, h,
                   (0, 0, w * 30, h * 30), (30, 30), NODATA)


def synthetic_search_specs():
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
    return (
        ("F NARROW SIDE VALLEY", f, (7, 4), (7, 60), 400.0, 300.0, 500.0, lambda q: q["mean_msl"] < 375),
        ("G TWO VALLEYS", g, (7, 4), (7, 60), 400.0, 280.0, 500.0, lambda q: q["mean_msl"] < 365),
        ("H HIGH-VALLEY-HIGH", h, (6, 4), (6, 60), 400.0, 300.0, 500.0, lambda q: q["mean_msl"] < 375),
    )


DIRECTIONS8 = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]


# ---------------------------------------------------------------------------
# guidance signal: reverse-Dijkstra "achievable low-MSL" potential, goal outward
# ---------------------------------------------------------------------------

def build_low_msl_guidance(elevation: np.ndarray, config, goal_rc, altitude_reference_msl: float,
                            w_distance: float, w_altitude: float, altitude_scale_m: float,
                            distance_reference_m: float) -> np.ndarray:
    """guidance[r, c] = cheap 2D estimate (Dijkstra dist to goal) of mission
    cost IF the aircraft always flew at the lowest legal MSL each cell's
    terrain allows (terrain + min_agl_m). XY-only, ignores z entirely and
    ignores climb/descent rate feasibility -- deliberately optimistic/
    inadmissible, used only as a tie-break signal, never as a hard filter
    or as part of the anchor key used for the epsilon bound.
    """
    h, w = elevation.shape
    achievable_min_msl = elevation + config.min_agl_m
    step_axial = config.xy_resolution_m
    step_diag = config.xy_resolution_m * math.sqrt(2.0)

    dist = np.full((h, w), math.inf)
    goal_r, goal_c = goal_rc
    dist[goal_r, goal_c] = 0.0
    pq = [(0.0, goal_r, goal_c)]
    while pq:
        d, r, c = heapq.heappop(pq)
        if d > dist[r, c]:
            continue
        for dr, dc in DIRECTIONS8:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            step = step_diag if (dr != 0 and dc != 0) else step_axial
            mean_msl = (achievable_min_msl[r, c] + achievable_min_msl[nr, nc]) / 2.0
            excess = max(0.0, mean_msl - altitude_reference_msl)
            edge_cost = (w_distance * step + w_altitude * step * excess / altitude_scale_m) / distance_reference_m
            nd = d + edge_cost
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                heapq.heappush(pq, (nd, nr, nc))
    return dist


# ---------------------------------------------------------------------------
# ARA* with optional bucketed guidance tie-break (standalone, mirrors
# planner.astar.ara_star_search's phase loop exactly when guidance_fn=None)
# ---------------------------------------------------------------------------

@dataclass
class GuidedPhaseResult:
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


def ara_star_search_guided(start, goal, terrain, min_search_altitude_msl, max_search_altitude_msl, config,
                            primitives, epsilon_schedule, max_expansions_cumulative,
                            guidance_fn: Optional[Callable[[int, int], float]] = None,
                            bucket_width: float = 0.05):
    t0 = time.perf_counter()
    counter = itertools.count()
    primitive_cache: Dict = {}
    cache_stats = {"hits": 0, "misses": 0, "actual_calls": 0}
    distance_reference_m = compute_distance_reference(start, goal, terrain, config)
    start_aug: AugmentedState = (start[0], start[1], start[2], 0, BUCKET_SHORT)

    g: Dict[AugmentedState, float] = {start_aug: 0.0}
    parent: Dict[AugmentedState, AugmentedState] = {}
    open_members: set = {start_aug}
    incons_members: set = set()

    incumbent_cost = math.inf
    incumbent_state = None
    incumbent_path_snapshot: List[CanonicalState] = []
    expanded_nodes = 0
    phases: List[GuidedPhaseResult] = []

    def h_of(s):
        return _heuristic(s, goal, terrain, config, 1.0, None, False, distance_reference_m)

    def guidance_of(s):
        return 0.0 if guidance_fn is None else guidance_fn(s[0], s[1])

    for epsilon in epsilon_schedule:
        phase_start_expanded = expanded_nodes
        phase_first_incumbent_improvement_expansion = None

        open_members = open_members | incons_members
        incons_members = set()
        closed_this_phase: set = set()

        # heap tuple: (bucket, guidance, counter, state, raw_weighted_key)
        heap: List[Tuple[float, float, int, AugmentedState, float]] = []
        for s in open_members:
            h_val = h_of(s)
            raw_key = g[s] + epsilon * h_val
            bucket = math.floor(raw_key / bucket_width) * bucket_width if guidance_fn is not None else raw_key
            heapq.heappush(heap, (bucket, guidance_of(s), next(counter), s, raw_key))

        phase_complete = False
        while heap:
            bucket_top, _, _, s_top, _ = heap[0]
            if s_top in closed_this_phase:
                heapq.heappop(heap)
                continue
            # SAFE termination: bucket_top is a lower bound on every live raw
            # key (bucket = floor(raw_key/W)*W <= raw_key), so this can only
            # delay termination relative to the pure-anchor check, never
            # trigger it early -- the epsilon bound is never weakened.
            if incumbent_cost < math.inf and bucket_top >= incumbent_cost:
                phase_complete = True
                break

            _, _, _, s, _ = heapq.heappop(heap)
            if s in closed_this_phase:
                continue
            closed_this_phase.add(s)
            open_members.discard(s)
            expanded_nodes += 1
            if expanded_nodes >= max_expansions_cumulative:
                break

            neighbors, *_ = _generate_neighbors(
                s, primitives, terrain, config, min_search_altitude_msl, max_search_altitude_msl,
                primitive_cache, cache_stats, distance_reference_m, None, None, None, True, None,
            )
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
                        raw_key = tentative_g + epsilon * h_val
                        bucket = math.floor(raw_key / bucket_width) * bucket_width if guidance_fn is not None else raw_key
                        heapq.heappush(heap, (bucket, guidance_of(neighbor), next(counter), neighbor, raw_key))

        if not heap:
            phase_complete = True

        phases.append(GuidedPhaseResult(
            epsilon=epsilon,
            added_expansions=expanded_nodes - phase_start_expanded,
            cumulative_expansions=expanded_nodes,
            cumulative_runtime_s=time.perf_counter() - t0,
            incumbent_cost=incumbent_cost,
            open_size_at_end=len(open_members),
            incons_size_at_end=len(incons_members),
            phase_complete=phase_complete,
            first_incumbent_improvement_expansion=phase_first_incumbent_improvement_expansion,
            incumbent_path=list(incumbent_path_snapshot),
        ))

    return phases


def state_path_report(path, terrain, config, policy, primitives, d_ref):
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
    return {"mean_msl": alt["average_aircraft_msl"], "min_msl": alt["minimum_aircraft_msl"],
            "xy": xy, "distance": distance, "altitude": altitude, "total": distance + altitude}


def run_scenario(name, elevation, start_rc, goal_rc, start_msl, min_msl, max_msl, preferred_fn, guidance_fn_factory,
                  bucket_width=0.05, quiet=False):
    config = dataclasses.replace(DEFAULT_CONFIG, normalized_w_altitude=1.25)
    terrain = TerrainQuery(make_roi(elevation))
    cfg = dataclasses.replace(config, cost_mode="normalized", altitude_reference_msl=min_msl,
                              normalized_w_distance=1.0, normalized_w_altitude=1.25,
                              normalized_altitude_scale_m=1000.0, goal_tolerance_xy_m=0.0,
                              goal_tolerance_z_m=0.0)
    primitives = build_primitive_set(cfg)
    z = msl_to_z_index(start_msl, cfg)
    start, goal = (start_rc[0], start_rc[1], z), (goal_rc[0], goal_rc[1], z)
    d_ref = compute_distance_reference(start, goal, terrain, cfg)
    policy = production_mission_policy(min_msl)

    guidance_fn = None
    if guidance_fn_factory is not None:
        pot = build_low_msl_guidance(elevation, cfg, goal_rc, min_msl, 1.0, 1.25, 1000.0, d_ref)
        guidance_fn = lambda r, c: pot[r, c]

    t0 = time.perf_counter()
    phases = ara_star_search_guided(start, goal, terrain, min_msl, max_msl, cfg, primitives,
                                     epsilon_schedule=(1.70, 1.50, 1.30), max_expansions_cumulative=12_000,
                                     guidance_fn=guidance_fn, bucket_width=bucket_width)
    wall = time.perf_counter() - t0

    if not quiet:
        print(f"\n----- {name}  (guidance={'ON bw=' + str(bucket_width) if guidance_fn else 'OFF'}) -----")
    found_preferred_at = None
    last_q = None
    for phase in phases:
        q = state_path_report(phase.incumbent_path, terrain, cfg, policy, primitives, d_ref)
        last_q = q
        is_preferred = bool(q and preferred_fn(q))
        if is_preferred and found_preferred_at is None:
            found_preferred_at = (phase.epsilon, phase.cumulative_expansions)
        if not quiet:
            print(f"  eps={phase.epsilon:.2f}: first_improve_exp={phase.first_incumbent_improvement_expansion} "
                  f"cum_exp={phase.cumulative_expansions} time={phase.cumulative_runtime_s:.4f}s "
                  f"cost={q['total']:.6f} mean_msl={q['mean_msl']:.1f} min_msl={q['min_msl']:.0f} "
                  f"xy={q['xy']:.1f}m preferred={is_preferred} OPEN={phase.open_size_at_end} "
                  f"INCONS={phase.incons_size_at_end} phase_complete={phase.phase_complete}")
    if not quiet:
        print(f"  wall={wall:.3f}s  preferred_topology_first_found_at={found_preferred_at}")
    return phases, last_q, found_preferred_at, wall


def main():
    print("=== SELF-CHECK: guidance_fn=None must reproduce Stage 38.3 baseline bit-for-bit ===")
    for name, elevation, start_rc, goal_rc, start_msl, min_msl, max_msl, preferred_fn in synthetic_search_specs():
        run_scenario(name, elevation, start_rc, goal_rc, start_msl, min_msl, max_msl, preferred_fn, None)

    print("\n\n=== BUCKET-WIDTH SWEEP (guidance ON) ===")
    for bw in (0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05):
        print(f"\n--- bucket_width={bw} ---")
        for name, elevation, start_rc, goal_rc, start_msl, min_msl, max_msl, preferred_fn in synthetic_search_specs():
            phases, last_q, found_at, wall = run_scenario(
                name, elevation, start_rc, goal_rc, start_msl, min_msl, max_msl, preferred_fn,
                build_low_msl_guidance, bucket_width=bw, quiet=True)
            final = phases[-1]
            print(f"  {name}: final_cost={last_q['total']:.6f} mean_msl={last_q['mean_msl']:.1f} "
                  f"first_improve_exp={phases[0].first_incumbent_improvement_expansion} "
                  f"final_cum_exp={final.cumulative_expansions} wall={wall:.3f}s "
                  f"preferred_first_found_at={found_at}")


if __name__ == "__main__":
    main()
