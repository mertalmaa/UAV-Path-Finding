"""Stage 20: separate "is there a physically feasible path at all" from
"is A* search-space explosion the problem" for the Stage 19 6.48km
long-valley benchmark. Pure diagnostic -- no architecture change.

A) Direct level chain: 216 consecutive S-level primitives at a constant
   3760m, checked one-by-one with the existing evaluate_primitive() --
   no A* involved at all.
B) LEVEL-only A*: same start/goal/altitude, but the primitive list passed
   to astar_search() is filtered down to just the 8 level primitives
   (astar_search already accepts a custom primitives list -- no
   production code changed).
C) Full primitives (level+climb+descent), w_MSL=0.0 for this run only.
D) Reference to Stage 19's existing w_MSL=0.63 result (not rerun beyond
   the already-established 30k/150k attempts).
"""
import dataclasses
import math
import time

from planner.astar import astar_search, msl_to_z_index, state_to_xyz
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import load_roi
from planner.terrain import TerrainQuery

START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
AIRCRAFT_MSL = 3760.0


def test_a_direct_level_chain(cfg, primitives, tq) -> dict:
    print("=== Test A: direct level chain (no A*, pure evaluate_primitive) ===")
    level_s = next(p for p in primitives if p.direction == "S" and p.primitive_type == "level")
    assert level_s.drow == 1 and level_s.dcol == 0

    total = 0
    valid_count = 0
    invalid_count = 0
    min_agl = math.inf
    first_invalid = None

    row = START_ROW
    while row < GOAL_ROW:
        start_xyz = state_to_xyz((row, START_COL, msl_to_z_index(AIRCRAFT_MSL, cfg)), tq, cfg)
        result = evaluate_primitive(start_xyz, level_s, tq, cfg)
        total += 1
        if result.valid:
            valid_count += 1
            min_agl = min(min_agl, result.min_agl_m)
        else:
            invalid_count += 1
            if first_invalid is None:
                first_invalid = (row, START_COL, result.reason)
        row += level_s.drow

    print(f"  total_edges={total}  valid={valid_count}  invalid={invalid_count}")
    print(f"  minimum_observed_AGL={min_agl if min_agl != math.inf else 'n/a'}")
    print(f"  first_invalid_edge={first_invalid}")
    ok = invalid_count == 0
    print(f"  {'216/216 VALID -- feasible path proven to exist' if ok else 'CHAIN HAS INVALID EDGES -- see above'}")
    print()
    return {"total": total, "valid": valid_count, "invalid": invalid_count, "min_agl": min_agl,
            "first_invalid": first_invalid, "chain_valid": ok}


def test_b_level_only_astar(cfg, primitives, tq) -> dict:
    print("=== Test B: LEVEL-only A* (neighbor set restricted to the 8 level primitives) ===")
    level_only = [p for p in primitives if p.primitive_type == "level"]
    print(f"  primitive set size: {len(level_only)} (all level, no climb/descent)")

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, GOAL_COL, z0)

    t0 = time.perf_counter()
    result = astar_search(start, goal, tq, min_search_altitude_msl=AIRCRAFT_MSL, max_search_altitude_msl=AIRCRAFT_MSL,
                           config=cfg, primitives=level_only, max_expansions=30_000,
                           use_primitive_cache=True, use_dominance_pruning=False, use_msl_lower_bound_heuristic=True)
    wall = time.perf_counter() - t0

    print(f"  status={result.status} wall={wall:.2f}s runtime={result.runtime_s:.2f}s "
          f"expanded={result.expanded_nodes} max_open={result.max_open_size}")
    if result.success:
        print(f"  geometric_path_length={result.geometric_path_length:.1f} "
              f"minimum_observed_agl={result.minimum_observed_agl:.1f}")
    else:
        print("  !! LEVEL-only A* did NOT find the goal despite a proven-valid direct chain "
              "-- would indicate a search/goal/neighbor/state bug.")
    print()
    return {"result": result}


def test_c_full_primitives_w0(cfg, primitives, tq) -> dict:
    print("=== Test C: full primitives (level+climb+descent), w_MSL=0.0 (this run only) ===")
    c = dataclasses.replace(cfg, msl_cost_weight=0.0)
    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, GOAL_COL, z0)

    seg = tq.roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    min_search = math.ceil((float(seg.min()) + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0
    print(f"  w_MSL=0.0  bounds=[{min_search},{max_search}]")

    t0 = time.perf_counter()
    result = astar_search(start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
                           config=c, primitives=primitives, max_expansions=30_000,
                           use_primitive_cache=True, use_dominance_pruning=False, use_msl_lower_bound_heuristic=True)
    wall = time.perf_counter() - t0

    print(f"  status={result.status} wall={wall:.2f}s runtime={result.runtime_s:.2f}s "
          f"expanded={result.expanded_nodes} max_open={result.max_open_size} "
          f"multiplier={result.heuristic_cost_multiplier}")
    if result.success:
        print(f"  geometric_path_length={result.geometric_path_length:.1f} "
              f"minimum_observed_agl={result.minimum_observed_agl:.1f} "
              f"total_climb={result.total_climb_m:.1f} total_descent={result.total_descent_m:.1f} "
              f"reversals={result.total_vertical_reversal_count}")
    print()
    return {"result": result}


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    a = test_a_direct_level_chain(cfg, primitives, tq)
    if not a["chain_valid"]:
        print("STOPPING per instructions: direct level chain is not fully valid, physical feasibility "
              "assumption needs to be re-examined before any search-level testing.")
        return

    b = test_b_level_only_astar(cfg, primitives, tq)
    c = test_c_full_primitives_w0(cfg, primitives, tq)

    print("=== Result matrix ===")
    print(f"  A) direct level chain valid: {a['chain_valid']} (min_AGL={a['min_agl']:.1f})")
    print(f"  B) LEVEL-only A*: {b['result'].status}")
    print(f"  C) full primitives, w_MSL=0.0: {c['result'].status}")
    print("  D) full primitives, w_MSL=0.63: search_limit_reached at 30k (24.3s) and even 150k (106.6s) "
          "-- established in Stage 19, not rerun here.")


if __name__ == "__main__":
    main()
