"""Continuous physical pose and trajectory data contracts.

This module deliberately has no dependency on the grid, DEM, CandidateZ, or
A* state.  It is the pose-aware planner's physical layer: a future search
layer may associate a quantized key with a physical trajectory, but cannot
reconstruct that trajectory from the key.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Tuple


# Frozen by project.md's 0B flight/trajectory contract.  This is a planner
# kinematic convention under zero wind, not a claim that IAS equals TAS.
FIXED_PLANAR_SPEED_MPS = 40.0

_ENDPOINT_TOLERANCE_M = 1e-9
_HEADING_TOLERANCE_DEG = 1e-9
_KINEMATIC_TOLERANCE_DEG_S = 1e-9


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return value


def normalize_heading_deg(heading_deg: float) -> float:
    """Normalize a navigation heading into the canonical half-open interval.

    Navigation convention is North=0, East=90, South=180, West=270 degrees;
    positive change is clockwise.  Values must be finite so invalid numerical
    state cannot silently enter a physical trajectory.
    """
    heading = _finite(heading_deg, "heading_deg")
    normalized = heading % 360.0
    return 0.0 if normalized == 0.0 else normalized


def angular_distance_deg(first_heading_deg: float, second_heading_deg: float) -> float:
    """Smallest unsigned circular difference between two headings, in degrees."""
    first = normalize_heading_deg(first_heading_deg)
    second = normalize_heading_deg(second_heading_deg)
    return abs((second - first + 180.0) % 360.0 - 180.0)


@dataclass(frozen=True)
class PhysicalPose:
    """Continuous aircraft pose in EPSG:32636 metres and navigation degrees.

    This object intentionally carries neither DEM row/col nor a search-state
    key.  It is the authoritative start/end position for future propagation.
    """

    x_m: float
    y_m: float
    z_msl_m: float
    heading_deg: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x_m", _finite(self.x_m, "x_m"))
        object.__setattr__(self, "y_m", _finite(self.y_m, "y_m"))
        object.__setattr__(self, "z_msl_m", _finite(self.z_msl_m, "z_msl_m"))
        object.__setattr__(self, "heading_deg", normalize_heading_deg(self.heading_deg))


@dataclass(frozen=True)
class TrajectorySample:
    """One continuous physical trajectory sample.

    ``horizontal_distance_along_path_m`` is accumulated ground-track arc
    length, not a 3D segment length.  It is therefore the coordinate used to
    derive zero-wind duration under the fixed 40 m/s convention.
    """

    x_m: float
    y_m: float
    z_msl_m: float
    heading_deg: float
    horizontal_distance_along_path_m: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x_m", _finite(self.x_m, "x_m"))
        object.__setattr__(self, "y_m", _finite(self.y_m, "y_m"))
        object.__setattr__(self, "z_msl_m", _finite(self.z_msl_m, "z_msl_m"))
        object.__setattr__(self, "heading_deg", normalize_heading_deg(self.heading_deg))
        distance = _finite(self.horizontal_distance_along_path_m, "horizontal_distance_along_path_m")
        if distance < 0.0:
            raise ValueError("horizontal_distance_along_path_m must be non-negative")
        object.__setattr__(self, "horizontal_distance_along_path_m", distance)

    @property
    def pose(self) -> PhysicalPose:
        """The sample's physical pose without any grid/search conversion."""
        return PhysicalPose(self.x_m, self.y_m, self.z_msl_m, self.heading_deg)


def _same_pose(first: PhysicalPose, second: PhysicalPose) -> bool:
    return (
        math.isclose(first.x_m, second.x_m, rel_tol=0.0, abs_tol=_ENDPOINT_TOLERANCE_M)
        and math.isclose(first.y_m, second.y_m, rel_tol=0.0, abs_tol=_ENDPOINT_TOLERANCE_M)
        and math.isclose(first.z_msl_m, second.z_msl_m, rel_tol=0.0, abs_tol=_ENDPOINT_TOLERANCE_M)
        and angular_distance_deg(first.heading_deg, second.heading_deg) <= _HEADING_TOLERANCE_DEG
    )


@dataclass(frozen=True)
class PhysicalTrajectory:
    """A sampled, continuous path independent of a search representation.

    Samples include both endpoints, are ordered by non-decreasing horizontal
    ground-track distance, and retain their original physical coordinates.
    ``duration_s`` is derived rather than accepted from callers, preventing a
    trajectory from silently violating the frozen fixed-speed contract.
    """

    start_pose: PhysicalPose
    end_pose: PhysicalPose
    horizontal_arc_length_m: float
    samples: Tuple[TrajectorySample, ...]
    duration_s: float = field(init=False)
    max_sample_spacing_m: float = field(init=False)

    def __post_init__(self) -> None:
        horizontal_arc_length = _finite(self.horizontal_arc_length_m, "horizontal_arc_length_m")
        if horizontal_arc_length < 0.0:
            raise ValueError("horizontal_arc_length_m must be non-negative")
        samples = tuple(self.samples)
        if not samples:
            raise ValueError("samples must include the physical trajectory endpoints")
        if not _same_pose(samples[0].pose, self.start_pose):
            raise ValueError("first trajectory sample must equal start_pose")
        if not _same_pose(samples[-1].pose, self.end_pose):
            raise ValueError("last trajectory sample must equal end_pose")

        previous_distance = -1.0
        max_spacing = 0.0
        for sample in samples:
            if sample.horizontal_distance_along_path_m + _ENDPOINT_TOLERANCE_M < previous_distance:
                raise ValueError("trajectory samples must be ordered by horizontal distance")
            if previous_distance >= 0.0:
                max_spacing = max(max_spacing, sample.horizontal_distance_along_path_m - previous_distance)
            previous_distance = sample.horizontal_distance_along_path_m
        if not math.isclose(samples[0].horizontal_distance_along_path_m, 0.0,
                            rel_tol=0.0, abs_tol=_ENDPOINT_TOLERANCE_M):
            raise ValueError("first trajectory sample must be at horizontal distance zero")
        if not math.isclose(samples[-1].horizontal_distance_along_path_m, horizontal_arc_length,
                            rel_tol=0.0, abs_tol=_ENDPOINT_TOLERANCE_M):
            raise ValueError("last trajectory sample distance must equal horizontal_arc_length_m")

        object.__setattr__(self, "horizontal_arc_length_m", horizontal_arc_length)
        object.__setattr__(self, "samples", samples)
        object.__setattr__(self, "duration_s", horizontal_arc_length / FIXED_PLANAR_SPEED_MPS)
        # This is the observed maximum horizontal interval, not merely the
        # builder's requested spacing.  Safety can therefore enforce its own
        # sampling contract without reconstructing the physical trajectory.
        object.__setattr__(self, "max_sample_spacing_m", max_spacing)


@dataclass(frozen=True)
class PhysicalPrimitiveResult:
    """Future primitive result without an A* state or state key.

    Start/end are exposed as properties of the authoritative trajectory so
    duplicate, potentially divergent pose copies are not introduced.
    """

    trajectory: PhysicalTrajectory

    @property
    def start_pose(self) -> PhysicalPose:
        return self.trajectory.start_pose

    @property
    def end_pose(self) -> PhysicalPose:
        return self.trajectory.end_pose


@dataclass(frozen=True)
class LevelTurnKinematics:
    """One kinematically consistent, aircraft-neutral level-turn envelope.

    This value is the boundary between a capability adapter and pure geometry.
    It must already contain a fixed-40 conservative radius/rate pair (such as
    the 1A.1 adapter's result); trajectory geometry never independently reads
    radius and rate fields from an aircraft profile.
    """

    direction: str  # "LEFT" | "RIGHT"
    radius_m: float
    signed_turn_rate_deg_s: float

    def __post_init__(self) -> None:
        if self.direction not in ("LEFT", "RIGHT"):
            raise ValueError(f"direction must be LEFT or RIGHT, got {self.direction!r}")
        radius = _finite(self.radius_m, "radius_m")
        if radius <= 0.0:
            raise ValueError("radius_m must be positive")
        rate = _finite(self.signed_turn_rate_deg_s, "signed_turn_rate_deg_s")
        expected_sign = -1.0 if self.direction == "LEFT" else 1.0
        if rate == 0.0 or math.copysign(1.0, rate) != expected_sign:
            raise ValueError("signed_turn_rate_deg_s sign must match direction")
        expected_rate = expected_sign * (FIXED_PLANAR_SPEED_MPS / radius) * 180.0 / math.pi
        if not math.isclose(rate, expected_rate, rel_tol=0.0, abs_tol=_KINEMATIC_TOLERANCE_DEG_S):
            raise ValueError(
                "fixed-40 turn kinematics must satisfy "
                "abs(turn_rate_rad_s) == FIXED_PLANAR_SPEED_MPS / radius_m"
            )
        object.__setattr__(self, "radius_m", radius)
        object.__setattr__(self, "signed_turn_rate_deg_s", rate)


@dataclass(frozen=True)
class CombinedTurnKinematics:
    """A level turn paired with a constant climbing or descending rate.

    Its capability is supplied by an envelope policy.  This geometry module
    can represent a helix but cannot infer combined-aircraft capability from
    two individually-valid limits.
    """

    level_turn: LevelTurnKinematics
    signed_vertical_rate_mps: float

    def __post_init__(self) -> None:
        rate = _finite(self.signed_vertical_rate_mps, "signed_vertical_rate_mps")
        if rate == 0.0:
            raise ValueError("signed_vertical_rate_mps must be non-zero for a combined turn")
        object.__setattr__(self, "signed_vertical_rate_mps", rate)


def _interval_count(horizontal_arc_length_m: float, max_sample_spacing_m: float) -> int:
    spacing = _finite(max_sample_spacing_m, "max_sample_spacing_m")
    if spacing <= 0.0:
        raise ValueError("max_sample_spacing_m must be positive")
    return max(1, math.ceil(horizontal_arc_length_m / spacing))


def _trajectory_result(
    start_pose: PhysicalPose,
    end_pose: PhysicalPose,
    horizontal_arc_length_m: float,
    samples: Tuple[TrajectorySample, ...],
) -> PhysicalPrimitiveResult:
    return PhysicalPrimitiveResult(
        PhysicalTrajectory(
            start_pose=start_pose,
            end_pose=end_pose,
            horizontal_arc_length_m=horizontal_arc_length_m,
            samples=samples,
        )
    )


def build_straight_level_trajectory(
    start_pose: PhysicalPose,
    horizontal_distance_m: float,
    max_sample_spacing_m: float,
) -> PhysicalPrimitiveResult:
    """Build a continuous, level straight trajectory from a physical pose.

    Sample locations use the frozen navigation convention: a heading of zero
    advances in +Y (north), and a heading of 90 degrees advances in +X (east).
    The endpoint and all samples remain physical floating-point coordinates.
    """
    distance = _finite(horizontal_distance_m, "horizontal_distance_m")
    if distance < 0.0:
        raise ValueError("horizontal_distance_m must be non-negative")
    interval_count = _interval_count(distance, max_sample_spacing_m)
    heading_rad = math.radians(start_pose.heading_deg)
    samples = tuple(
        TrajectorySample(
            x_m=start_pose.x_m + distance * i / interval_count * math.sin(heading_rad),
            y_m=start_pose.y_m + distance * i / interval_count * math.cos(heading_rad),
            z_msl_m=start_pose.z_msl_m,
            heading_deg=start_pose.heading_deg,
            horizontal_distance_along_path_m=distance * i / interval_count,
        )
        for i in range(interval_count + 1)
    )
    end_sample = samples[-1]
    end_pose = PhysicalPose(end_sample.x_m, end_sample.y_m, end_sample.z_msl_m, end_sample.heading_deg)
    return _trajectory_result(start_pose, end_pose, distance, samples)


def build_straight_vertical_trajectory(
    start_pose: PhysicalPose,
    horizontal_distance_m: float,
    signed_vertical_rate_mps: float,
    max_sample_spacing_m: float,
) -> PhysicalPrimitiveResult:
    """Build a straight climb or descent at fixed 40 m/s horizontal speed.

    Positive Vz means climb and negative Vz means descent.  The caller's
    envelope establishes whether the rate is safe; this preserves geometry.
    """
    distance = _finite(horizontal_distance_m, "horizontal_distance_m")
    if distance < 0.0:
        raise ValueError("horizontal_distance_m must be non-negative")
    vertical_rate = _finite(signed_vertical_rate_mps, "signed_vertical_rate_mps")
    if vertical_rate == 0.0:
        raise ValueError("signed_vertical_rate_mps must be non-zero for vertical motion")
    interval_count = _interval_count(distance, max_sample_spacing_m)
    heading_rad = math.radians(start_pose.heading_deg)
    samples = tuple(
        TrajectorySample(
            x_m=start_pose.x_m + distance * i / interval_count * math.sin(heading_rad),
            y_m=start_pose.y_m + distance * i / interval_count * math.cos(heading_rad),
            z_msl_m=start_pose.z_msl_m + vertical_rate * (distance * i / interval_count) / FIXED_PLANAR_SPEED_MPS,
            heading_deg=start_pose.heading_deg,
            horizontal_distance_along_path_m=distance * i / interval_count,
        )
        for i in range(interval_count + 1)
    )
    end_sample = samples[-1]
    end_pose = PhysicalPose(end_sample.x_m, end_sample.y_m, end_sample.z_msl_m, end_sample.heading_deg)
    return _trajectory_result(start_pose, end_pose, distance, samples)


def level_turn_center(start_pose: PhysicalPose, kinematics: LevelTurnKinematics) -> Tuple[float, float]:
    """Return the centre of the fixed-radius level turn in physical UTM metres."""
    heading_rad = math.radians(start_pose.heading_deg)
    turn_sign = -1.0 if kinematics.direction == "LEFT" else 1.0
    return (
        start_pose.x_m + turn_sign * kinematics.radius_m * math.cos(heading_rad),
        start_pose.y_m - turn_sign * kinematics.radius_m * math.sin(heading_rad),
    )


def build_level_turn_trajectory(
    start_pose: PhysicalPose,
    kinematics: LevelTurnKinematics,
    heading_change_deg: float,
    max_sample_spacing_m: float,
) -> PhysicalPrimitiveResult:
    """Build a true circular, level arc from a fixed-40 turn envelope.

    ``heading_change_deg`` is an unsigned requested magnitude.  Direction
    supplies its sign: LEFT decreases navigation heading, RIGHT increases it.
    This is geometry only; it neither queries an aircraft profile nor grants
    an active planner successor/executable-flight status to the arc.
    """
    heading_change = _finite(heading_change_deg, "heading_change_deg")
    if heading_change < 0.0:
        raise ValueError("heading_change_deg must be an unsigned non-negative magnitude")
    interval_count = _interval_count(
        kinematics.radius_m * math.radians(heading_change), max_sample_spacing_m
    )
    turn_sign = -1.0 if kinematics.direction == "LEFT" else 1.0
    arc_length = kinematics.radius_m * math.radians(heading_change)
    center_x, center_y = level_turn_center(start_pose, kinematics)

    samples = []
    for i in range(interval_count + 1):
        distance = arc_length * i / interval_count
        heading_deg = start_pose.heading_deg + turn_sign * math.degrees(distance / kinematics.radius_m)
        heading_rad = math.radians(heading_deg)
        samples.append(TrajectorySample(
            x_m=center_x - turn_sign * kinematics.radius_m * math.cos(heading_rad),
            y_m=center_y + turn_sign * kinematics.radius_m * math.sin(heading_rad),
            z_msl_m=start_pose.z_msl_m,
            heading_deg=heading_deg,
            horizontal_distance_along_path_m=distance,
        ))

    end_sample = samples[-1]
    end_pose = PhysicalPose(end_sample.x_m, end_sample.y_m, end_sample.z_msl_m, end_sample.heading_deg)
    return _trajectory_result(start_pose, end_pose, arc_length, tuple(samples))


def build_helical_turn_trajectory(
    start_pose: PhysicalPose,
    kinematics: CombinedTurnKinematics,
    heading_change_deg: float,
    max_sample_spacing_m: float,
) -> PhysicalPrimitiveResult:
    """Build a continuous climbing or descending circular arc (helix).

    A positive-Vz full circle is a spiral-up macro.  This remains passive
    geometry until future successor integration and swept safety validation.
    """
    heading_change = _finite(heading_change_deg, "heading_change_deg")
    if heading_change < 0.0:
        raise ValueError("heading_change_deg must be an unsigned non-negative magnitude")
    turn = kinematics.level_turn
    arc_length = turn.radius_m * math.radians(heading_change)
    interval_count = _interval_count(arc_length, max_sample_spacing_m)
    turn_sign = -1.0 if turn.direction == "LEFT" else 1.0
    center_x, center_y = level_turn_center(start_pose, turn)

    samples = []
    for i in range(interval_count + 1):
        distance = arc_length * i / interval_count
        heading_deg = start_pose.heading_deg + turn_sign * math.degrees(distance / turn.radius_m)
        heading_rad = math.radians(heading_deg)
        samples.append(TrajectorySample(
            x_m=center_x - turn_sign * turn.radius_m * math.cos(heading_rad),
            y_m=center_y + turn_sign * turn.radius_m * math.sin(heading_rad),
            z_msl_m=start_pose.z_msl_m + kinematics.signed_vertical_rate_mps * distance / FIXED_PLANAR_SPEED_MPS,
            heading_deg=heading_deg,
            horizontal_distance_along_path_m=distance,
        ))

    end_sample = samples[-1]
    end_pose = PhysicalPose(end_sample.x_m, end_sample.y_m, end_sample.z_msl_m, end_sample.heading_deg)
    return _trajectory_result(start_pose, end_pose, arc_length, tuple(samples))
