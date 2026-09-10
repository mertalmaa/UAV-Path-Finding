"""Stage 38.1: Fine Corridor Safety Precompute -- an O(1) dense-NumPy-array
replacement for evaluate_primitive()'s per-sample terrain/AGL evaluation,
restricted to the fine (30m) XY corridor cells the ARA* refinement search
(planner.astar.ara_star_search) is actually allowed to visit.

Directly mirrors planner.coarse_astar's Stage 35.1
precompute_coarse_primitive_safety()/precomputed_primitive_validity() --
same underlying math, same "z_index never enters the key" invariant --
but stored as dense (H, W, P) NumPy arrays (not a Python dict keyed by
(row, col, primitive_id)) for the fine grid's much larger corridor cell
count, and restricted to corridor cells only (uncomputed cells outside the
corridor are marked invalid by default -- they can never legally be
expanded into anyway, since the existing corridor_mask check in
planner.astar._generate_neighbors already forbids it).

Per (row, col, primitive_id):

    static_invalid_reason_code: 0 = geometry/terrain clean (every sampled
        point along the primitive's path, from this (row, col), is
        in-bounds AND non-NoData) -- "valid" in the sense of NOT static-
        invalid; the AGL/altitude check below still applies.
      1 = out_of_bounds somewhere along the path
      2 = nodata somewhere along the path
      3 = never computed (cell outside the fine XY corridor)
    required_start_msl: max over primitive samples i of
        (terrain_elevation(i) + min_agl_m - primitive.dz_m * t_i)
      -- meaningless (left at +inf) when static_invalid_reason_code != 0.

A state's own current_msl (its OWN altitude at (row, col), BEFORE taking
the primitive -- never the destination's) is then checked against
required_start_msl directly: current_msl >= required_start_msl means every
AGL sample along the primitive clears min_agl_m, exactly reproducing
evaluate_primitive()'s decision without a single terrain re-sample at
search time.

Endpoint transition (climb/descent angle) validity is, as in the coarse
module, NOT re-checked here -- every primitive in `primitives` already
passed that check once, permanently, inside build_primitive_set(); it is a
static property of the primitive's own geometry, never of position or
start_msl.
"""
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from planner.astar import PrimitiveId, _primitive_id
from planner.config import PlannerConfig
from planner.primitives import MotionPrimitive
from planner.terrain import TerrainQuery

_REASON_TEXT = {0: "ok", 1: "out_of_bounds", 2: "nodata", 3: "outside_corridor_not_computed"}


@dataclass
class FinePrecomputeResult:
    static_invalid_reason_code: np.ndarray  # int8, shape (H, W, P) -- see module docstring
    required_start_msl: np.ndarray  # float32, shape (H, W, P)
    primitive_ids: List[PrimitiveId]  # primitives[i] <-> array index i, caller's responsibility to match order
    corridor_cell_count: int
    entry_count: int  # corridor_cell_count * len(primitives) -- the entries actually COMPUTED (not the dense total)
    preprocessing_runtime_s: float
    approx_memory_mb: float
    static_invalid_count: int  # among computed (corridor) entries only


def precompute_fine_corridor_primitive_safety(
    terrain: TerrainQuery,
    corridor_mask: np.ndarray,
    primitives: List[MotionPrimitive],
    config: PlannerConfig,
) -> FinePrecomputeResult:
    """Fill dense (H, W, P) arrays ONLY for (row, col) where corridor_mask
    is True -- every other cell keeps its default (reason_code=3,
    required_start_msl=+inf), i.e. unconditionally invalid. See module
    docstring for the exact per-sample math (identical to planner.
    coarse_astar's Stage 35.1 precompute, just on the fine grid/corridor).
    """
    t0 = time.perf_counter()
    height, width = terrain.roi.height, terrain.roi.width
    if corridor_mask.shape != (height, width):
        raise ValueError(f"corridor_mask shape {corridor_mask.shape} != terrain ROI shape {(height, width)}")

    n_primitives = len(primitives)
    static_invalid_reason_code = np.full((height, width, n_primitives), 3, dtype=np.int8)
    required_start_msl = np.full((height, width, n_primitives), math.inf, dtype=np.float32)

    static_invalid_count = 0
    rows, cols = np.nonzero(corridor_mask)
    corridor_cell_count = len(rows)

    for row, col in zip(rows.tolist(), cols.tolist()):
        x1, y1 = terrain.rowcol_to_xy(row, col)
        for p_idx, prim in enumerate(primitives):
            dx = prim.dcol * config.xy_resolution_m
            dy = -prim.drow * config.xy_resolution_m
            n_intervals = max(1, math.ceil(prim.horizontal_distance_m / config.primitive_sample_spacing_m))

            reason_code = 0
            needed_max = -math.inf
            for i in range(n_intervals + 1):
                t = i / n_intervals
                x, y = x1 + dx * t, y1 + dy * t
                sample = terrain.query(x, y)
                if not sample.valid:
                    reason_code = 1 if sample.reason == "out_of_bounds" else 2
                    break
                vertical_offset = prim.dz_m * t
                needed = sample.elevation + config.min_agl_m - vertical_offset
                if needed > needed_max:
                    needed_max = needed

            static_invalid_reason_code[row, col, p_idx] = reason_code
            if reason_code != 0:
                static_invalid_count += 1
            else:
                required_start_msl[row, col, p_idx] = needed_max

    preprocessing_runtime_s = time.perf_counter() - t0
    approx_memory_mb = (static_invalid_reason_code.nbytes + required_start_msl.nbytes) / (1024.0 * 1024.0)

    return FinePrecomputeResult(
        static_invalid_reason_code=static_invalid_reason_code,
        required_start_msl=required_start_msl,
        primitive_ids=[_primitive_id(p) for p in primitives],
        corridor_cell_count=corridor_cell_count,
        entry_count=corridor_cell_count * n_primitives,
        preprocessing_runtime_s=preprocessing_runtime_s,
        approx_memory_mb=approx_memory_mb,
        static_invalid_count=static_invalid_count,
    )


def fine_precomputed_primitive_validity(
    precompute: FinePrecomputeResult,
    row: int, col: int, primitive_index: int, start_msl: float,
) -> Tuple[bool, str]:
    """O(1) replacement for evaluate_primitive()'s validity decision --
    array lookup + one comparison, no terrain re-sampling. primitive_index
    must be this primitive's position in the SAME primitives list that was
    passed to precompute_fine_corridor_primitive_safety() (see module
    docstring: z_index is never part of the lookup)."""
    reason_code = precompute.static_invalid_reason_code[row, col, primitive_index]
    if reason_code != 0:
        return False, _REASON_TEXT[int(reason_code)]
    if start_msl >= precompute.required_start_msl[row, col, primitive_index]:
        return True, "ok"
    return False, "below_min_agl"
