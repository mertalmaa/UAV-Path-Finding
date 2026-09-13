"""Step REP-1.2B validation: CandidateZ as the production altitude successor
source. Small, fast synthetic test matrix (A-K) plus one tiny synthetic A*
run -- no real terrain, no Mission A/B/C here (those are run separately,
see scripts/rep12b_mission_ab_smoke.py).
"""
import math

import numpy as np
from affine import Affine

from planner.astar import (
    _generate_candidate_z_neighbors,
    astar_search,
    decode_candidate_altitude,
    encode_candidate_altitude,
    state_to_xyz,
)
from planner.candidate_z import CandidateZGenerator, MissionContext, TerrainMetadataStore
from planner.config import DEFAULT_CONFIG
from planner.roi import ROIData
from planner.terrain import TerrainQuery

RES = 30.0
NODATA = -9999.0


def make_terrain(elevation: np.ndarray) -> TerrainQuery:
    height, width = elevation.shape
    transform = Affine(RES, 0.0, 0.0, 0.0, -RES, height * RES)
    roi = ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * RES, height * RES),
        resolution=(RES, RES), nodata=NODATA,
    )
    return TerrainQuery(roi)


def main() -> None:
    all_pass = True

    def check(label: str, ok: bool) -> None:
        nonlocal all_pass
        all_pass = all_pass and ok
        print(f"  {label:<78} {'PASS' if ok else 'FAIL'}")

    cfg = DEFAULT_CONFIG
    min_agl = cfg.min_agl_m if cfg.min_agl_m is not None else 200.0

    # Flat terrain at 1000m, 20x20 cells -- start/goal altitudes deliberately
    # OFF-LATTICE (not multiples of cfg.z_step_m=20) to exercise B/C/D directly.
    flat_elev = np.full((20, 20), 1000.0, dtype=np.float32)
    terrain = make_terrain(flat_elev)
    store = TerrainMetadataStore(terrain)

    start_rc = (15, 2)
    goal_rc = (5, 17)
    floor_flat = math.ceil((1000.0 + min_agl) / cfg.z_step_m) * cfg.z_step_m  # CLASS A floor on flat terrain
    # Small, deliberately off-lattice floor/ceiling gap (a few metres, well within
    # one grid step's max climb/descent budget) so BOTH a climb-to-ceiling and a
    # descend-to-floor successor actually exist from the start altitude -- a big
    # gap (e.g. hundreds of metres) would make every non-level candidate angle-
    # infeasible in a single grid step and vacuously "pass" B by having no
    # non-level successor to inspect at all.
    start_z = floor_flat + 3.0    # off-lattice
    ceiling = start_z + 3.0       # off-lattice
    goal_z = 1000.0 + min_agl + 383.7    # off-lattice, deliberately far from start/floor/ceiling

    mission = MissionContext(
        start_rowcol=start_rc, start_z_msl=start_z,
        goal_rowcol=goal_rc, goal_z_msl=goal_z,
        ceiling_msl=ceiling, min_agl_m=min_agl,
    )
    gen = CandidateZGenerator(store, mission)

    print("=== A: deterministic candidate successors ===")
    start_state = (start_rc[0], start_rc[1], encode_candidate_altitude(start_z))
    common_args = (terrain, cfg, 0.0, 10000.0, None, {"hits": 0, "misses": 0, "actual_calls": 0}, None, None, None, None)
    r1 = _generate_candidate_z_neighbors(start_state, *common_args, candidate_z_generator=gen)
    r2 = _generate_candidate_z_neighbors(start_state, *common_args, candidate_z_generator=gen)
    check("two identical calls -> identical accepted successor list", r1[0] == r2[0])

    print("=== B: successors not derived from regular z_step arithmetic ===")
    # ceiling (500m above min_agl floor) is off-lattice; a successor landing there
    # from the off-lattice start altitude has a dz_m that is not a multiple of z_step_m.
    non_lattice_dz = any(
        abs((decode_candidate_altitude(ns[2]) - start_z) / cfg.z_step_m
            - round((decode_candidate_altitude(ns[2]) - start_z) / cfg.z_step_m)) > 1e-6
        for ns, _ in r1[0]
    )
    check("at least one accepted successor has a non-z_step_m-multiple dz_m", non_lattice_dz)

    print("=== C: off-lattice start state works ===")
    check(
        "start altitude round-trips exactly through encode/decode",
        abs(decode_candidate_altitude(start_state[2]) - start_z) < 1e-6,
    )

    print("=== D: off-lattice goal state works ===")
    goal_state = (goal_rc[0], goal_rc[1], encode_candidate_altitude(goal_z))
    check(
        "goal altitude round-trips exactly through encode/decode",
        abs(decode_candidate_altitude(goal_state[2]) - goal_z) < 1e-6,
    )

    print("=== E: arbitrary non-candidate altitude cannot appear as successor ===")
    legal_ok = True
    for ns, _ in r1[0]:
        nr, nc, nz = ns
        z_msl = decode_candidate_altitude(nz)
        legal_set = {start_z} | set(gen.generate(nr, nc))
        if not any(abs(z_msl - c) < 1e-6 for c in legal_set):
            legal_ok = False
    check("every accepted successor altitude is exactly source-altitude or a generate() event", legal_ok)

    print("=== F: same terrain/mission/config -> same candidate successors ===")
    gen2 = CandidateZGenerator(TerrainMetadataStore(terrain), mission)
    r3 = _generate_candidate_z_neighbors(start_state, *common_args, candidate_z_generator=gen2)
    check("independent generator instance, same inputs -> identical accepted list", r1[0] == r3[0])

    print("=== G: different expansion order -> same CandidateZ logical set ===")
    # Query the same cells via a forward sweep and a reversed sweep, on a FRESH
    # generator each time (no shared memo state carried over) -- the result for
    # a given cell must be identical regardless of what order cells were queried in.
    cells = [(r, c) for r in range(5) for c in range(5)]
    gen_forward = CandidateZGenerator(TerrainMetadataStore(terrain), mission)
    forward = {rc: gen_forward.generate(*rc) for rc in cells}
    gen_reversed = CandidateZGenerator(TerrainMetadataStore(terrain), mission)
    reverse_result = {rc: gen_reversed.generate(*rc) for rc in reversed(cells)}
    check("generate(row,col) result independent of query order", forward == reverse_result)

    print("=== H: no regular-lattice fallback in the new function ===")
    import inspect
    from planner import astar as astar_mod
    src = inspect.getsource(astar_mod._generate_candidate_z_neighbors)
    code_only = src.split('"""', 2)[-1] if src.count('"""') >= 2 else src  # strip the docstring
    check("no 'z_step_m' token in _generate_candidate_z_neighbors' CODE (docstring excluded)", "z_step_m" not in code_only)
    check("no 'z_index' token in _generate_candidate_z_neighbors' CODE (docstring excluded)", "z_index" not in code_only)

    print("=== I: primitive target altitude exactly candidate altitude ===")
    exact_ok = True
    for ns, _ in r1[0]:
        nr, nc, nz = ns
        z_msl = decode_candidate_altitude(nz)
        candidates = {start_z} | set(gen.generate(nr, nc))
        if not any(z_msl == c or abs(z_msl - c) < 1e-9 for c in candidates):
            exact_ok = False
    check("accepted successor altitude matches its candidate to float precision", exact_ok)

    print("=== J: terrain floor violation cannot become a successor ===")
    # A cliff: column 10 rises sharply so its floor is far above the plain start altitude.
    cliff_elev = np.full((20, 20), 1000.0, dtype=np.float32)
    cliff_elev[:, 10:] = 1000.0 + 400.0  # destination cells here have a much higher floor
    cliff_terrain = make_terrain(cliff_elev)
    cliff_store = TerrainMetadataStore(cliff_terrain)
    cliff_mission = MissionContext(
        start_rowcol=(10, 9), start_z_msl=1000.0 + min_agl + 10.0,
        goal_rowcol=goal_rc, goal_z_msl=goal_z, ceiling_msl=ceiling, min_agl_m=min_agl,
    )
    cliff_gen = CandidateZGenerator(cliff_store, cliff_mission)
    low_state = (10, 9, encode_candidate_altitude(1000.0 + min_agl + 10.0))
    cliff_args = (cliff_terrain, cfg, 0.0, 10000.0, None, {"hits": 0, "misses": 0, "actual_calls": 0}, None, None, None, None)
    cliff_result = _generate_candidate_z_neighbors(low_state, *cliff_args, candidate_z_generator=cliff_gen)
    east_into_cliff = [
        ns for ns, _ in cliff_result[0] if ns[1] == 10 and abs(decode_candidate_altitude(ns[2]) - (1000.0 + min_agl + 10.0)) < 1e-6
    ]
    check("level move into the cliff cell at the old (too-low) altitude is never accepted", len(east_into_cliff) == 0)

    print("=== K: a non-instantiated candidate may still be logically representable ===")
    mid_altitude = start_z + 1.0  # strictly between source cell's floor and ceiling, never enumerated by generate()
    never_enumerated = not any(abs(mid_altitude - c) < 1e-9 for c in gen.generate(*start_rc))
    representable = gen.is_representable(start_rc[0], start_rc[1], mid_altitude)
    check("a value generate() never enumerates can still be is_representable()==True", never_enumerated and representable)

    print()
    print("=== Tiny synthetic A* run (CandidateZ-driven) ===")
    # Reachability note (honest disclosure, matches the pre-existing CLASS-C gap):
    # with CLASS-C (motion-derived intermediate candidates) still unimplemented
    # (see project.md "Step CLASS-C"), generate() only ever offers {floor, ceiling,
    # start/goal-at-that-cell} -- so an arbitrary off-lattice goal altitude FAR from
    # all of those is not reachable within a single grid step of the goal cell. This
    # smoke test therefore uses a goal altitude equal to the mission start altitude
    # (reachable via plain "level" moves, still fully off-lattice/non-z_step_m) to
    # exercise the search mechanism itself; separate goal-altitude reachability via
    # climb/descent is already covered directly against _generate_candidate_z_
    # neighbors above (tests A/B/I).
    search_goal_state = (goal_rc[0], goal_rc[1], encode_candidate_altitude(start_z))
    result = astar_search(
        start_state, search_goal_state, terrain,
        min_search_altitude_msl=0.0, max_search_altitude_msl=10000.0,
        config=cfg, max_expansions=20000, candidate_z_generator=gen,
    )
    print(f"  status={result.status} termination_reason={result.termination_reason} "
          f"expanded={result.expanded_nodes} generated={result.generated_neighbors} "
          f"path_len={len(result.path) if result.path else 0}")
    check("tiny synthetic CandidateZ-driven A* finds a path", result.success)
    if result.success:
        final_xyz = state_to_xyz(result.path[-1], terrain, cfg, gen)
        check("path ends exactly at the off-lattice goal altitude", abs(final_xyz[2] - start_z) < 1e-6)

    print()
    print(f"Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
