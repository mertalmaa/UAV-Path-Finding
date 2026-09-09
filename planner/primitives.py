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


def build_primitive_set(config: PlannerConfig = DEFAULT_CONFIG) -> List[MotionPrimitive]:
    """8-direction level/climb/descent primitive set.

    Level primitives are a single grid cell. Climb/descent primitives'
    horizontal distance is derived from the max angle, not hard-coded:
    required_horizontal_distance = abs(dz) / tan(max_angle), rounded up to
    the smallest whole number of grid steps in that direction so the
    resulting angle is <= the limit. Every primitive is verified against
    evaluate_transition() and dropped if it doesn't come back valid.
    """
    if config.max_climb_angle_deg is None or config.max_descent_angle_deg is None:
        raise ValueError("config.max_climb_angle_deg / max_descent_angle_deg must be set")

    axial_step = config.xy_resolution_m
    diagonal_step = config.xy_resolution_m * math.sqrt(2.0)

    primitives: List[MotionPrimitive] = []

    for direction, (unit_drow, unit_dcol) in DIRECTIONS.items():
        is_diagonal = unit_drow != 0 and unit_dcol != 0
        step = diagonal_step if is_diagonal else axial_step

        level = MotionPrimitive(direction, unit_drow, unit_dcol, 0.0, step, "level")
        if _endpoint_geometry_valid(level, config):
            primitives.append(level)

        for kind, sign, max_angle_deg in (
            ("climb", +1, config.max_climb_angle_deg),
            ("descent", -1, config.max_descent_angle_deg),
        ):
            dz = sign * config.z_step_m
            required_horizontal = abs(dz) / math.tan(math.radians(max_angle_deg))
            n_cells = math.ceil(required_horizontal / step)
            horizontal = n_cells * step
            prim = MotionPrimitive(direction, unit_drow * n_cells, unit_dcol * n_cells, dz, horizontal, kind)
            if _endpoint_geometry_valid(prim, config):
                primitives.append(prim)

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
