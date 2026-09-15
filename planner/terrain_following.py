"""Terrain-following altitude planning on an existing continuous ground track.

This is a separate planning stage, with separate diagnostics from A*. It
preserves exact XY arcs and mission endpoints. BASIC turns stay level: an
existing level-turn radius must not silently become a combined-turn radius.
The output is piecewise linear altitude in horizontal arc length, under the
repository's fixed 40 m/s planar-speed model (no vertical acceleration state).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace
from typing import Optional, Sequence, Tuple, Any

from planner.altitude_profile import solve_lowest_altitude_profile
from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.fixed_wing_envelope import (
    FixedWingKinematicEnvelope,
    FixedWingKinematicModel,
    HORIZONTAL_SPEED_MPS,
    MAX_CLIMB_RATE_MPS,
    MAX_DESCENT_RATE_MPS,
)
from planner.physical import FIXED_PLANAR_SPEED_MPS, PhysicalPose, PhysicalTrajectory, angular_distance_deg
from planner.pose_search import GoalPose, GoalTolerance, PoseSearchResult, pose_aware_astar_search
from planner.terrain import TerrainQuery
from planner.trajectory_safety import (
    TerrainInfluenceCache, _covered_cells, curve_to_chord_deviation_m,
    evaluate_physical_trajectory_safety,
)


@dataclass(frozen=True)
class TerrainFollowingResult:
    success: bool
    status: str
    trajectories: Tuple[PhysicalTrajectory, ...] = ()
    minimum_agl_m: float = float("nan")
    runtime_s: float = 0.0
    refinement_passes: int = 0
    target_agl_m: float = 120.0


@dataclass(frozen=True)
class TerrainFollowingPlanResult:
    search_result: PoseSearchResult
    profile_result: Optional[TerrainFollowingResult]

    @property
    def success(self) -> bool:
        return self.profile_result is not None and self.profile_result.success

    @property
    def trajectories(self) -> Tuple[PhysicalTrajectory, ...]:
        return self.profile_result.trajectories if self.success else ()


def _resolve_target_agl(config: PlannerConfig, target: Optional[float]) -> float:
    if config.min_agl_m is None or not math.isfinite(config.min_agl_m) or config.min_agl_m < 0:
        raise ValueError("a finite nonnegative min_agl_m is required")
    resolved = max(config.min_agl_m, config.desired_agl_m) if target is None else target
    if not math.isfinite(resolved) or resolved < config.min_agl_m:
        raise ValueError("target_agl_m must be finite and at least min_agl_m")
    return resolved


def optimize_terrain_following_altitudes(
    trajectories: Sequence[PhysicalTrajectory], terrain: TerrainQuery,
    aircraft_profile: Optional[Any] = None, *, config: PlannerConfig = DEFAULT_CONFIG,
    target_agl_m: Optional[float] = None, max_climb_rate_mps: float = MAX_CLIMB_RATE_MPS,
    max_descent_rate_mps: float = MAX_DESCENT_RATE_MPS, max_refinement_passes: int = 16,
    envelope: Optional[FixedWingKinematicEnvelope] = None,
) -> TerrainFollowingResult:
    """Find a low safe altitude schedule for a supplied ground track under generic fixed-wing kinematics."""
    started = time.perf_counter()
    target_agl_m = _resolve_target_agl(config, target_agl_m)
    for value in (max_climb_rate_mps, max_descent_rate_mps):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("vertical rate caps must be finite and positive")
    if not isinstance(max_refinement_passes, int) or max_refinement_passes < 1:
        raise ValueError("max_refinement_passes must be a positive integer")
    source = tuple(trajectories)
    if not source:
        raise ValueError("at least one trajectory is required")
    for a, b in zip(source, source[1:]):
        if a.end_pose != b.start_pose:
            raise ValueError("trajectory chain must preserve continuous physical endpoints")

    def failure(reason: str, passes: int = 0) -> TerrainFollowingResult:
        return TerrainFollowingResult(False, reason, runtime_s=time.perf_counter() - started,
                                      refinement_passes=passes, target_agl_m=target_agl_m)

    cache = TerrainInfluenceCache(terrain)
    field = cache.field(config.lateral_buffer_m)
    if envelope is None:
        envelope = FixedWingKinematicEnvelope()

    stations = [source[0].samples[0]]
    slices = []
    edge_turn = []
    durations = []
    ground = [-math.inf]
    for trajectory in source:
        if trajectory.max_sample_spacing_m > config.primitive_sample_spacing_m:
            return failure("INSUFFICIENT_SAMPLE_DENSITY")
        begin = len(stations) - 1
        for a, b in zip(trajectory.samples, trajectory.samples[1:]):
            ds = b.horizontal_distance_along_path_m - a.horizontal_distance_along_path_m
            if ds <= 0:
                return failure("NONPOSITIVE_SAMPLE_INTERVAL")
            delta = angular_distance_deg(a.heading_deg, b.heading_deg)
            edge_turn.append((delta, ds))
            durations.append(ds / FIXED_PLANAR_SPEED_MPS)
            maximum = -math.inf
            for row, col in _covered_cells(terrain, a, b, curve_to_chord_deviation_m(a, b)):
                if not terrain.in_bounds_rowcol(row, col) or not field.valid[row, col]:
                    return failure("INVALID_TERRAIN_COVERAGE")
                maximum = max(maximum, float(field.elevation_msl[row, col]))
            if not math.isfinite(maximum):
                return failure("INVALID_TERRAIN_COVERAGE")
            ground[-1] = max(ground[-1], maximum)
            ground.append(maximum)
            stations.append(b)
        slices.append((begin, len(stations)))
    if len(stations) < 2:
        return failure("EMPTY_GROUND_TRACK")

    # No arbitrary MSL floor/ceiling: valid terrain may be below sea level.
    hard_floor = [z + config.min_agl_m for z in ground]
    desired_floor = [z + target_agl_m for z in ground]
    climb, descent = [], []
    for (angle, ds), dt in zip(edge_turn, durations):
        climb_cap = min(max_climb_rate_mps, envelope.max_climb_rate_mps)
        descent_cap = min(max_descent_rate_mps, envelope.max_descent_rate_mps)
        if angle >= 1e-8:
            radius = ds / math.radians(angle)
            if radius + 1e-6 < envelope.turn_radius_m:
                return failure("TURN_REQUIRES_HORIZONTAL_REPLAN")
            # A wider combined radius cannot be squeezed into a level arc.
            if (not config.enable_combined_turns or
                    radius + 1e-6 < envelope.turn_radius_m * envelope.model.combined_radius_factor):
                climb_cap = descent_cap = 0.0
            else:
                climb_cap = min(climb_cap, envelope.max_climb_rate_mps * envelope.model.combined_vertical_rate_factor)
                descent_cap = min(descent_cap, envelope.max_descent_rate_mps * envelope.model.combined_vertical_rate_factor)
        climb.append(dt * climb_cap)
        descent.append(dt * descent_cap)
    start_z, end_z = stations[0].z_msl_m, stations[-1].z_msl_m

    for passes in range(1, max_refinement_passes + 1):
        upper = [start_z]
        for budget in climb:
            upper.append(upper[-1] + budget)
        upper[-1] = min(upper[-1], end_z)
        for i in range(len(upper) - 2, -1, -1):
            upper[i] = min(upper[i], upper[i + 1] + descent[i])
        floor = [max(hard, min(wanted, cap))
                 for hard, wanted, cap in zip(hard_floor, desired_floor, upper)]
        solved = solve_lowest_altitude_profile(
            floor, climb, descent, start_altitude_msl_m=start_z,
            end_altitude_msl_m=end_z,
        )
        if not solved.feasible:
            return failure("FIXED_TRACK_" + solved.status, passes)
        altitudes = solved.altitudes_msl_m
        break

    for i, (angle, ds) in enumerate(edge_turn):
        if angle < 1e-8:
            continue
        actual_radius = ds / math.radians(angle)
        limit = envelope.level_turn(altitudes[i], "LEFT")
        if (limit.availability != "AVAILABLE" or limit.level_turn is None
                or actual_radius + 1e-6 < limit.level_turn.radius_m):
            return failure("TURN_REQUIRES_HORIZONTAL_REPLAN", passes)

    output = []
    min_agl = math.inf
    for original, (begin, end) in zip(source, slices):
        samples = tuple(replace(sample, z_msl_m=z)
                        for sample, z in zip(original.samples, altitudes[begin:end]))
        trajectory = PhysicalTrajectory(samples[0].pose, samples[-1].pose,
                                        original.horizontal_arc_length_m, samples)
        safety = evaluate_physical_trajectory_safety(
            trajectory, terrain, config.min_agl_m, config.primitive_sample_spacing_m,
            planning_bounds=terrain.roi.bounds, lateral_buffer_m=config.lateral_buffer_m,
            terrain_influence_cache=cache,
        )
        if not safety.is_safe:
            return failure("FINAL_SAFETY_" + str(safety.failure_reason), passes)
        min_agl = min(min_agl, safety.min_agl_m)
        output.append(trajectory)
    return TerrainFollowingResult(True, "FOUND", tuple(output), min_agl,
                                  time.perf_counter() - started, passes, target_agl_m)


def plan_terrain_following(
    start: PhysicalPose, goal: GoalPose, terrain: TerrainQuery,
    aircraft_profile: Optional[Any] = None,
    *, goal_tolerance: GoalTolerance, config: PlannerConfig = DEFAULT_CONFIG,
    target_agl_m: Optional[float] = None, max_expansions: Optional[int] = None,
    max_search_time_s: Optional[float] = None,
    envelope: Optional[FixedWingKinematicEnvelope] = None,
) -> TerrainFollowingPlanResult:
    """Search a ground track, then explicitly plan and validate low altitude."""
    target_agl_m = _resolve_target_agl(config, target_agl_m)
    if envelope is None:
        envelope = FixedWingKinematicEnvelope()
    search = pose_aware_astar_search(
        start, goal, terrain, goal_tolerance=goal_tolerance,
        config=config, max_expansions=max_expansions, max_search_time_s=max_search_time_s,
        envelope=envelope,
    )
    if not search.success:
        return TerrainFollowingPlanResult(search, None)
    if not search.trajectories:
        return TerrainFollowingPlanResult(search, TerrainFollowingResult(False, "EMPTY_GROUND_TRACK"))
    profile = optimize_terrain_following_altitudes(
        search.trajectories, terrain, config=config, target_agl_m=target_agl_m, envelope=envelope,
    )
    return TerrainFollowingPlanResult(search, profile)
