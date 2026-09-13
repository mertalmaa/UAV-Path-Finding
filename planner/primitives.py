"""Motion primitives: the fixed set of small moves the planner may make,
and along-primitive terrain/AGL feasibility checking.

Two responsibilities, kept in one module because they are tightly coupled
at this stage:

  1. build_primitive_set() -- 8 raster directions x {level, climb, descent}.
     Endpoint geometry is validated with the existing evaluate_transition();
     nothing recomputes that logic here.

  2. evaluate_primitive() -- given an aircraft start state and one
     primitive, checks the endpoint transition, then samples terrain/AGL
     at points along the straight-line path (reusing evaluate_agl() for
     every sample) so a primitive that clears both endpoints but clips a
     ridge in the middle is still rejected.

No A*, no search, no cost, no heading/turn-radius, no primitive set beyond
this fixed shape.
"""
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from planner.agl import evaluate_agl
from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.terrain import TerrainQuery
from planner.transition import evaluate_transition

Point3 = Tuple[float, float, float]  # (x, y, z_msl)

# row increases downward in the raster (north-up transform), so "north"
# is drow=-1, not +1.
DIRECTIONS: Dict[str, Tuple[int, int]] = {
    "N": (-1, 0),
    "NE": (-1, 1),
    "E": (0, 1),
    "SE": (1, 1),
    "S": (1, 0),
    "SW": (1, -1),
    "W": (0, -1),
    "NW": (-1, -1),
}


@dataclass(frozen=True)
class MotionPrimitive:
    direction: str  # one of DIRECTIONS' keys
    drow: int
    dcol: int
    dz_m: float
    horizontal_distance_m: float
    primitive_type: str  # "level" | "climb" | "descent"


def _step_for_direction(direction: str, config: PlannerConfig) -> float:
    unit_drow, unit_dcol = DIRECTIONS[direction]
    is_diagonal = unit_drow != 0 and unit_dcol != 0
    return config.xy_resolution_m * math.sqrt(2.0) if is_diagonal else config.xy_resolution_m


def _level_primitive(direction: str, config: PlannerConfig) -> Optional[MotionPrimitive]:
    unit_drow, unit_dcol = DIRECTIONS[direction]
    step = _step_for_direction(direction, config)
    prim = MotionPrimitive(direction, unit_drow, unit_dcol, 0.0, step, "level")
    return prim if _endpoint_geometry_valid(prim, config) else None


def _climb_descent_primitive(
    direction: str, delta_altitude_m: float, config: PlannerConfig
) -> Optional[MotionPrimitive]:
    """Build a climb/descent primitive for an explicit, arbitrary altitude delta.

    Step REP-1.2A: this is the single primitive-shape implementation for
    non-level motion. It takes delta_altitude_m as a plain float -- there is
    no assumption that abs(delta_altitude_m) is any particular fixed step
    (e.g. config.z_step_m). build_primitive_set() below is just one caller
    that happens to request +-config.z_step_m (to preserve the regular-
    lattice search state representation, which is unchanged this stage);
    primitive_for_target_altitude() is another caller that requests whatever
    arbitrary delta an explicit source/target altitude pair implies.

    Horizontal distance is derived from the max angle for the delta's sign,
    rounded up to the smallest whole number of grid steps in that direction
    so the resulting angle is <= the limit. Returns None (not appended) if
    the resulting endpoint fails evaluate_transition().
    """
    if delta_altitude_m == 0.0:
        raise ValueError("delta_altitude_m must be non-zero; use _level_primitive() for level motion")
    unit_drow, unit_dcol = DIRECTIONS[direction]
    step = _step_for_direction(direction, config)
    kind = "climb" if delta_altitude_m > 0.0 else "descent"
    max_angle_deg = config.max_climb_angle_deg if kind == "climb" else config.max_descent_angle_deg
    required_horizontal = abs(delta_altitude_m) / math.tan(math.radians(max_angle_deg))
    n_cells = math.ceil(required_horizontal / step)
    horizontal = n_cells * step
    prim = MotionPrimitive(direction, unit_drow * n_cells, unit_dcol * n_cells, delta_altitude_m, horizontal, kind)
    return prim if _endpoint_geometry_valid(prim, config) else None


def primitive_for_target_altitude(
    direction: str,
    source_altitude_m: float,
    target_altitude_m: float,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> Optional[MotionPrimitive]:
    """Representation-neutral primitive constructor (Step REP-1.2A).

    dz_m is exactly target_altitude_m - source_altitude_m -- not snapped to
    config.z_step_m or any other lattice. source_altitude_m and
    target_altitude_m may be arbitrary off-lattice floats (e.g. 4127 ->
    4163). This function makes no claim about whether the resulting motion
    is physically flyable by any given aircraft (see planner.vertical_motion
    for that, separate question) -- it only builds the endpoint geometry and
    verifies it against evaluate_transition(), exactly like every other
    primitive in this module.
    """
    delta = target_altitude_m - source_altitude_m
    if delta == 0.0:
        return _level_primitive(direction, config)
    return _climb_descent_primitive(direction, delta, config)


def primitive_for_single_step_target_altitude(
    direction: str,
    source_altitude_m: float,
    target_altitude_m: float,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> Optional[MotionPrimitive]:
    """Step REP-1.2B: single-grid-cell-step primitive toward an explicit
    target altitude.

    Unlike primitive_for_target_altitude() (REP-1.2A), which derives
    however many grid cells the climb/descent angle limit requires for a
    GIVEN delta (lengthening the horizontal distance as needed), this
    always uses exactly ONE grid step's horizontal distance in `direction`
    and returns None if that delta is not achievable within it (the angle
    limit would be exceeded) -- there is no automatic lengthening here.

    This exists for a caller (planner.astar's CandidateZ-driven successor
    generator) that must know WHICH cell a move lands in before it can ask
    what altitudes are representable there -- fixing the footprint to a
    single grid step makes the destination cell independent of the
    requested delta, unlike the auto-lengthening constructor above.
    """
    delta = target_altitude_m - source_altitude_m
    unit_drow, unit_dcol = DIRECTIONS[direction]
    step = _step_for_direction(direction, config)
    kind = "level" if delta == 0.0 else ("climb" if delta > 0.0 else "descent")
    prim = MotionPrimitive(direction, unit_drow, unit_dcol, delta, step, kind)
    return prim if _endpoint_geometry_valid(prim, config) else None


def primitive_for_target_altitude_over_horizon(
    direction: str,
    source_altitude_m: float,
    target_altitude_m: float,
    n_cells: int,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> Optional[MotionPrimitive]:
    """Step REP-1.2B.1: explicit-horizon primitive toward a target altitude.

    Unlike primitive_for_single_step_target_altitude() (REP-1.2B, footprint
    fixed at 1 cell) and primitive_for_target_altitude() (REP-1.2A,
    footprint auto-derived from config's global climb/descent angle), this
    takes n_cells as an explicit caller-supplied horizon -- the caller
    (planner.astar's CandidateZ-driven successor generator) derives n_cells
    from the AIRCRAFT's own real planner-safe vertical rate (see
    planner.vertical_motion.derive_minimum_horizontal_distance_m), never
    from a fixed global angle. n_cells=1 reduces this to exactly
    primitive_for_single_step_target_altitude()'s geometry.

    Still reuses the SAME core validity check (_endpoint_geometry_valid,
    i.e. evaluate_transition's existing geometric/angle safety layer) as
    every other primitive in this module -- this is an ADDITIONAL
    constraint on top of aircraft-capability feasibility, not a
    replacement for it: a transition must satisfy both. Returns None if
    n_cells < 1, or if the resulting endpoint fails that check.
    """
    if n_cells < 1:
        return None
    delta = target_altitude_m - source_altitude_m
    unit_drow, unit_dcol = DIRECTIONS[direction]
    step = _step_for_direction(direction, config)
    kind = "level" if delta == 0.0 else ("climb" if delta > 0.0 else "descent")
    horizontal = n_cells * step
    prim = MotionPrimitive(direction, unit_drow * n_cells, unit_dcol * n_cells, delta, horizontal, kind)
    return prim if _endpoint_geometry_valid(prim, config) else None


def build_primitive_set(config: PlannerConfig = DEFAULT_CONFIG) -> List[MotionPrimitive]:
    """8-direction level/climb/descent primitive set.

    This is the fixed-lattice caller: it builds the primitive set the
    regular-Z-lattice search state representation (planner.astar's
    CanonicalState.z_index arithmetic) still relies on, by requesting
    climb/descent primitives with delta_altitude_m = +-config.z_step_m from
    the same representation-neutral _climb_descent_primitive() that
    primitive_for_target_altitude() uses for arbitrary deltas. There is no
    separate/duplicate climb-descent geometry implementation here.
    """
    if config.max_climb_angle_deg is None or config.max_descent_angle_deg is None:
        raise ValueError("config.max_climb_angle_deg / max_descent_angle_deg must be set")

    primitives: List[MotionPrimitive] = []

    for direction in DIRECTIONS:
        level = _level_primitive(direction, config)
        if level is not None:
            primitives.append(level)

        climb = _climb_descent_primitive(direction, config.z_step_m, config)
        if climb is not None:
            primitives.append(climb)

        descent = _climb_descent_primitive(direction, -config.z_step_m, config)
        if descent is not None:
            primitives.append(descent)

    return primitives


def _endpoint_geometry_valid(primitive: MotionPrimitive, config: PlannerConfig) -> bool:
    result = evaluate_transition((0.0, 0.0, 0.0), (primitive.horizontal_distance_m, 0.0, primitive.dz_m), config)
    return result.valid


def primitive_endpoint(start: Point3, primitive: MotionPrimitive, config: PlannerConfig = DEFAULT_CONFIG) -> Point3:
    x1, y1, z1 = start
    dx = primitive.dcol * config.xy_resolution_m
    dy = -primitive.drow * config.xy_resolution_m
    return (x1 + dx, y1 + dy, z1 + primitive.dz_m)


@dataclass(frozen=True)
class PrimitiveSample:
    index: int
    t: float  # fraction along the segment, 0.0 (start) .. 1.0 (end)
    x: float
    y: float
    altitude_msl: float
    terrain_elevation_msl: float  # NaN if terrain lookup failed
    agl_m: float  # NaN if terrain lookup failed
    valid: bool
    reason: str  # "ok" | "out_of_bounds" | "nodata" | "below_min_agl"


@dataclass(frozen=True)
class PrimitiveEvalResult:
    valid: bool
    reason: str  # "ok" | "transition_invalid" | "below_min_agl" | "out_of_bounds" | "nodata"
    start: Point3
    end: Point3
    primitive_type: str
    horizontal_distance_m: float
    delta_z_m: float
    sample_count: int
    min_agl_m: float
    min_agl_sample: Optional[PrimitiveSample]
    first_failure: Optional[PrimitiveSample]


def evaluate_primitive(
    start: Point3,
    primitive: MotionPrimitive,
    terrain: TerrainQuery,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> PrimitiveEvalResult:
    """Endpoint transition check, then terrain/AGL sampled along the path.

    Reuses evaluate_transition() for the endpoint climb/descent geometry
    and evaluate_agl() for every sample -- no terrain/AGL logic is
    reimplemented here.
    """
    x1, y1, z1 = start
    dx = primitive.dcol * config.xy_resolution_m
    dy = -primitive.drow * config.xy_resolution_m
    end = (x1 + dx, y1 + dy, z1 + primitive.dz_m)

    transition = evaluate_transition(start, end, config)
    if not transition.valid:
        return PrimitiveEvalResult(
            valid=False,
            reason="transition_invalid",
            start=start,
            end=end,
            primitive_type=primitive.primitive_type,
            horizontal_distance_m=transition.horizontal_distance_m,
            delta_z_m=transition.delta_z_m,
            sample_count=0,
            min_agl_m=float("nan"),
            min_agl_sample=None,
            first_failure=None,
        )

    n_intervals = max(1, math.ceil(transition.horizontal_distance_m / config.primitive_sample_spacing_m))

    samples: List[PrimitiveSample] = []
    for i in range(n_intervals + 1):
        t = i / n_intervals
        x, y = x1 + dx * t, y1 + dy * t
        altitude_msl = z1 + primitive.dz_m * t
        agl = evaluate_agl(terrain, x, y, altitude_msl, config)
        samples.append(PrimitiveSample(
            index=i, t=t, x=x, y=y, altitude_msl=altitude_msl,
            terrain_elevation_msl=agl.terrain_elevation_msl, agl_m=agl.agl_m,
            valid=agl.valid, reason=agl.reason,
        ))

    numeric = [s for s in samples if not math.isnan(s.agl_m)]
    min_sample = min(numeric, key=lambda s: s.agl_m) if numeric else samples[0]
    first_failure = next((s for s in samples if not s.valid), None)

    valid = first_failure is None
    reason = "ok" if valid else first_failure.reason

    return PrimitiveEvalResult(
        valid=valid,
        reason=reason,
        start=start,
        end=end,
        primitive_type=primitive.primitive_type,
        horizontal_distance_m=transition.horizontal_distance_m,
        delta_z_m=transition.delta_z_m,
        sample_count=len(samples),
        min_agl_m=min_sample.agl_m,
        min_agl_sample=min_sample,
        first_failure=first_failure,
    )
