"""Pose-aware, continuous fixed-wing A*.

This is the production motion core for the fixed-wing planner. A
``PhysicalPose`` is propagated continuously; ``SearchKey`` is only the bounded
approximate one-representative dominance bucket. In particular, a key is
never converted back to a pose and terrain row/col is only consulted by the
continuous trajectory safety evaluator.

Climbing/descending turns share the continuous safety pipeline and are enabled
by default. BASIC remains configurable. Weighted ordering and optional Pareto
pruning are approximate; neither completeness nor a global suboptimality bound
is promised. Loiter and spiral macros are not search successors.
"""
from __future__ import annotations

import heapq
import itertools
import math
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Optional, Tuple

import numpy as np

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope, FixedWingKinematicModel
from planner.physical import (
    PhysicalPose,
    PhysicalTrajectory,
    angular_distance_deg,
    build_level_turn_trajectory,
    build_helical_turn_trajectory,
    build_straight_level_trajectory,
    build_straight_vertical_trajectory,
    normalize_heading_deg,
)
from planner.terrain import TerrainQuery
from planner.trajectory_safety import (
    TerrainInfluenceCache,
    TrajectorySafetyResult,
    evaluate_physical_trajectory_safety,
)


_BOUNDARY_EPSILON = 1e-9
# Guidance grids at/above this size use the vectorised scipy Dijkstra backend.
_FAST_GUIDANCE_MIN_CELLS = 40_000
_BASIC_PRIMITIVES = (
    "STRAIGHT_LEVEL", "LEFT_LEVEL_TURN", "RIGHT_LEVEL_TURN",
    "STRAIGHT_CLIMB", "STRAIGHT_DESCENT",
)
_COMBINED_TURN_PRIMITIVES = (
    "CLIMBING_LEFT_TURN", "CLIMBING_RIGHT_TURN",
    "DESCENDING_LEFT_TURN", "DESCENDING_RIGHT_TURN",
)


def active_primitive_names(config: PlannerConfig = DEFAULT_CONFIG) -> Tuple[str, ...]:
    """Return the configured successor labels without altering physical state."""
    return _BASIC_PRIMITIVES + (_COMBINED_TURN_PRIMITIVES if config.enable_combined_turns else ())


@dataclass(frozen=True, order=True)
class SearchKey:
    """Quantized bookkeeping identity; never a physical state."""

    x_bin: int
    y_bin: int
    z_bin: int
    heading_bin: int


@dataclass(frozen=True)
class GoalPose:
    """Continuous goal with an optional final navigation heading."""

    x_m: float
    y_m: float
    z_msl_m: float
    heading_deg: Optional[float] = None

    def __post_init__(self) -> None:
        for name in ("x_m", "y_m", "z_msl_m"):
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, value)
        if self.heading_deg is not None:
            object.__setattr__(self, "heading_deg", normalize_heading_deg(self.heading_deg))


@dataclass(frozen=True)
class GoalTolerance:
    """Physical, not bucket, goal tolerances."""

    xy_m: float
    altitude_m: float
    heading_deg: Optional[float] = None

    def __post_init__(self) -> None:
        for name in ("xy_m", "altitude_m"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
            object.__setattr__(self, name, value)
        if self.heading_deg is not None:
            value = float(self.heading_deg)
            if not math.isfinite(value) or value < 0.0 or value > 180.0:
                raise ValueError("heading_deg must be in [0, 180]")
            object.__setattr__(self, "heading_deg", value)


@dataclass(frozen=True)
class PoseSearchNode:
    """One actual representative retained for a SearchKey revision."""

    node_id: int
    key: SearchKey
    end_pose: PhysicalPose
    g_cost: float
    parent_node_id: Optional[int]
    incoming_trajectory: Optional[PhysicalTrajectory]
    incoming_primitive: Optional[str]


@dataclass(frozen=True)
class PoseSearchResult:
    success: bool
    status: str  # success | no_path | search_limit_reached | timeout
    termination_reason: str
    nodes: Tuple[PoseSearchNode, ...]
    total_cost: float
    expanded_nodes: int
    generated_neighbors: int
    rejected_neighbors: int
    rejected_reason_counts: Dict[str, int]
    max_open_size: int
    runtime_s: float
    unique_search_keys: int
    unique_xy_bins: int
    mean_heading_bins_per_xy: float
    median_heading_bins_per_xy: float
    max_heading_bins_per_xy: int
    mean_z_bins_per_xy: float
    median_z_bins_per_xy: float
    max_z_bins_per_xy: int
    same_key_collision_count: int
    same_key_replaced_lower_g: int
    same_key_rejected_existing_better: int
    same_key_self_transition_count: int
    same_key_self_transition_by_primitive: Dict[str, int]
    generated_by_primitive: Dict[str, int]
    open_inserted_by_primitive: Dict[str, int]
    expanded_arrivals_by_primitive: Dict[str, int]
    minimum_agl_m: float
    closest_xy_distance_to_goal_m: float
    closest_3d_distance_to_goal_m: float
    maximum_altitude_msl_m: float
    progress_checkpoints: Dict[int, Dict[str, float]]
    lateral_buffer_m: float
    effective_min_agl_m: float
    goal_xy_error_m: float = float("nan")
    goal_z_error_m: float = float("nan")
    final_heading_deg: float = float("nan")
    # Best actual physical chain is retained even when a budget ends a search.
    # It is diagnostic only and is never labelled a solution path.
    best_nodes: Tuple[PoseSearchNode, ...] = field(default_factory=tuple)

    @property
    def poses(self) -> Tuple[PhysicalPose, ...]:
        return tuple(node.end_pose for node in self.nodes)

    @property
    def trajectories(self) -> Tuple[PhysicalTrajectory, ...]:
        return tuple(node.incoming_trajectory for node in self.nodes[1:] if node.incoming_trajectory is not None)

    @property
    def continuous_path_length_m(self) -> float:
        return sum(_trajectory_3d_length(t) for t in self.trajectories)

    @property
    def path_primitives(self) -> Tuple[str, ...]:
        """Exact, ordered primitive sequence of the reconstructed solution."""
        return tuple(node.incoming_primitive for node in self.nodes[1:]
                     if node.incoming_primitive is not None)

    @property
    def path_primitive_counts(self) -> Dict[str, int]:
        """Counts only primitives that occur in the reconstructed solution."""
        return dict(sorted(Counter(self.path_primitives).items()))


def _floor_bin(value: float, size: float) -> int:
    if not math.isfinite(value) or not math.isfinite(size) or size <= 0.0:
        raise ValueError("quantization value must be finite and bin size positive")
    # 2B policy: a tiny fixed epsilon makes nominally exact decimal boundaries
    # deterministic without moving a physical coordinate.
    return math.floor(value / size + _BOUNDARY_EPSILON)


def search_key_for_pose(pose: PhysicalPose, config: PlannerConfig = DEFAULT_CONFIG) -> SearchKey:
    """Return the deterministic approximate key for an unchanged pose."""
    heading_width = float(config.search_heading_bin_deg)
    if heading_width <= 0.0 or 360.0 % heading_width > 1e-9:
        raise ValueError("search_heading_bin_deg must be a positive divisor of 360")
    heading_bins = round(360.0 / heading_width)
    heading_bin = _floor_bin(normalize_heading_deg(pose.heading_deg), heading_width) % heading_bins
    return SearchKey(
        _floor_bin(pose.x_m, config.search_xy_bin_m),
        _floor_bin(pose.y_m, config.search_xy_bin_m),
        _floor_bin(pose.z_msl_m, config.search_z_bin_m),
        heading_bin,
    )


def navigation_bearing_deg(start_x_m: float, start_y_m: float, goal_x_m: float, goal_y_m: float) -> float:
    """Navigation bearing (north=0, clockwise positive) from start to goal."""
    dx, dy = goal_x_m - start_x_m, goal_y_m - start_y_m
    if math.isclose(dx, 0.0, abs_tol=1e-12) and math.isclose(dy, 0.0, abs_tol=1e-12):
        raise ValueError("start and goal XY are identical; bearing is undefined")
    return normalize_heading_deg(math.degrees(math.atan2(dx, dy)))


def _trajectory_3d_length(trajectory: PhysicalTrajectory) -> float:
    return sum(math.sqrt(
        (second.x_m - first.x_m) ** 2 + (second.y_m - first.y_m) ** 2 +
        (second.z_msl_m - first.z_msl_m) ** 2
    ) for first, second in zip(trajectory.samples, trajectory.samples[1:]))


def _trajectory_edge_cost(
    trajectory: PhysicalTrajectory, safety: TrajectorySafetyResult, config: PlannerConfig,
    guidance=None,
) -> float:
    """Return the configured, non-negative cost of an already-safe trajectory.

    The default branch intentionally remains the previous geometric cost.  The
    opt-in experimental branch uses endpoint-trapezoidal integration of a
    terrain-relative AGL multiplier over each actual 3D sample segment. Sample
    terrain elevations are recorded by that same safety invocation, so cost
    evaluation performs no second terrain traversal.
    """
    physical_length = _trajectory_3d_length(trajectory)
    penalty_cost = 0.0
    if guidance is not None and getattr(guidance, "feedback_penalties", ()):
        for s in trajectory.samples:
            for px, py, pradius in guidance.feedback_penalties:
                if (s.x_m - px) ** 2 + (s.y_m - py) ** 2 <= pradius ** 2:
                    penalty_cost += 500.0
                    break
    if not config.enable_low_altitude_cost:
        if guidance is not None and getattr(config, "guidance_multiplier_in_g", False):
            samples = trajectory.samples
            weights = [guidance.multiplier(s) for s in samples]
            return sum(
                math.sqrt((b.x_m - a.x_m) ** 2 + (b.y_m - a.y_m) ** 2 + (b.z_msl_m - a.z_msl_m) ** 2)
                * (wa + wb) / 2.0
                for a, b, wa, wb in zip(samples, samples[1:], weights, weights[1:])
            ) + penalty_cost
        return physical_length + penalty_cost
    shape = config.low_altitude_cost_shape
    if shape not in ("quadratic", "linear", "capped_linear"):
        raise ValueError("low_altitude_cost_shape must be quadratic, linear or capped_linear")
    if shape in ("quadratic", "linear") and config.lambda_agl == 0.0 and guidance is None:
        return physical_length
    if shape == "capped_linear" and config.max_agl_cost_multiplier == 1.0 and guidance is None:
        return physical_length
    elevations = safety.sample_terrain_elevations_msl
    if len(elevations) != len(trajectory.samples):
        raise ValueError("low-altitude cost requires sample terrain recorded by trajectory safety")
    if shape in ("quadratic", "linear") and (config.agl_cost_scale_m <= 0.0 or config.lambda_agl < 0.0):
        raise ValueError("quadratic low-altitude cost requires positive scale and non-negative lambda")
    if shape == "capped_linear" and (
        config.full_penalty_agl_m <= config.desired_agl_m or config.max_agl_cost_multiplier < 1.0
    ):
        raise ValueError("capped-linear AGL cost requires full_penalty_agl_m > desired_agl_m and multiplier >= 1")

    def multiplier(sample, elevation_msl: float) -> float:
        if not math.isfinite(elevation_msl):
            raise ValueError("safe trajectory sample has no recorded terrain elevation")
        base = 1.0 if guidance is None else guidance.multiplier(sample)
        reference = elevation_msl + config.desired_agl_m
        if guidance is not None:
            reference = max(reference, guidance.target(sample))
        agl = sample.z_msl_m - reference + config.desired_agl_m
        if shape in ("quadratic", "linear"):
            excess = max(0.0, agl - config.desired_agl_m) / config.agl_cost_scale_m
            return base + config.lambda_agl * (excess * excess if shape == "quadratic" else excess)
        if agl <= config.desired_agl_m:
            return base
        ratio = min(1.0, (agl - config.desired_agl_m) /
                    (config.full_penalty_agl_m - config.desired_agl_m))
        return base + ratio * (config.max_agl_cost_multiplier - 1.0)

    return sum(
        math.sqrt(
            (second.x_m - first.x_m) ** 2 + (second.y_m - first.y_m) ** 2 +
            (second.z_msl_m - first.z_msl_m) ** 2
        ) * (multiplier(first, first_elevation) + multiplier(second, second_elevation)) / 2.0
        for first, second, first_elevation, second_elevation in zip(
            trajectory.samples, trajectory.samples[1:], elevations, elevations[1:]
        )
    ) + penalty_cost


def _goal_errors(pose: PhysicalPose, goal: GoalPose) -> Tuple[float, float, float]:
    xy = math.hypot(pose.x_m - goal.x_m, pose.y_m - goal.y_m)
    z = abs(pose.z_msl_m - goal.z_msl_m)
    d3 = math.sqrt(xy * xy + z * z)
    return xy, z, d3


def pose_in_goal(pose: PhysicalPose, goal: GoalPose, tolerance: GoalTolerance) -> bool:
    """Goal acceptance uses the exact propagated pose, never a SearchKey."""
    xy, z, _ = _goal_errors(pose, goal)
    if xy > tolerance.xy_m or z > tolerance.altitude_m:
        return False
    if goal.heading_deg is None:
        return True
    if tolerance.heading_deg is None:
        raise ValueError("a headed goal requires GoalTolerance.heading_deg")
    return angular_distance_deg(pose.heading_deg, goal.heading_deg) <= tolerance.heading_deg


def _heuristic(pose: PhysicalPose, goal: GoalPose, tolerance: GoalTolerance) -> float:
    """Admissible lower bound under geometric-3D edge cost."""
    horizontal = max(math.hypot(pose.x_m - goal.x_m, pose.y_m - goal.y_m) - tolerance.xy_m, 0.0)
    dz = max(abs(pose.z_msl_m - goal.z_msl_m) - tolerance.altitude_m, 0.0)
    return math.hypot(horizontal, dz)


def vertical_reachability_heuristic(pose, goal, tolerance, envelope=None):
    """Distance lower bound including the time needed to change altitude."""
    envelope = envelope or FixedWingKinematicEnvelope()
    horizontal = max(math.hypot(pose.x_m - goal.x_m, pose.y_m - goal.y_m) - tolerance.xy_m, 0.0)
    dz = max(abs(pose.z_msl_m - goal.z_msl_m) - tolerance.altitude_m, 0.0)
    rate = envelope.max_climb_rate_mps if pose.z_msl_m < goal.z_msl_m else envelope.max_descent_rate_mps
    # Combined motion may be faster in an explicitly supplied custom model.
    rate *= max(1.0, envelope.model.combined_vertical_rate_factor)
    required_horizontal = dz * envelope.horizontal_speed_mps / rate if dz else 0.0
    return math.hypot(max(horizontal, required_horizontal), dz)


class _TerrainGuidance:
    """Optional 2D topographic guidance; never grants physical feasibility.

    Dijkstra propagates backwards climb/descent envelopes along its successor
    tree on a sampled grid. This is a soft reference only: actual flight passes
    full trajectory safety. Grid costs need not lower-bound continuous costs.
    """

    def __init__(self, terrain, goal, tolerance, config, envelope, influence_cache, feedback_penalties=()):
        self.terrain = terrain
        self.config = config
        self.feedback_penalties = tuple(feedback_penalties)
        self.stride = getattr(config, "terrain_guidance_stride", 1)
        if not isinstance(self.stride, int) or isinstance(self.stride, bool) or self.stride < 1:
            raise ValueError("terrain_guidance_stride must be a positive integer")
        field = influence_cache.field(config.lateral_buffer_m)
        valid = field.valid[::self.stride, ::self.stride]
        elevation = field.elevation_msl[::self.stride, ::self.stride]
        self.distance = np.full(elevation.shape, np.inf)
        clearance = max(config.desired_agl_m, config.min_agl_m) if config.enable_low_altitude_cost else config.min_agl_m
        self.targets = elevation + clearance
        self.ceilings = np.full(elevation.shape, np.inf)
        rate_factor = max(1.0, envelope.model.combined_vertical_rate_factor)
        self.climb_slope = envelope.max_climb_rate_mps * rate_factor / envelope.horizontal_speed_mps
        self.descent_slope = envelope.max_descent_rate_mps * rate_factor / envelope.horizontal_speed_mps
        self.cost = np.ones(elevation.shape)
        if not np.any(valid):
            return
        lo, hi = float(elevation[valid].min()), float(elevation[valid].max())
        filled = np.where(valid, elevation, hi)
        rows, cols = elevation.shape
        cell_size_m = abs(terrain.roi.transform.a) * self.stride
        mode = getattr(config, "guidance_cost_mode", "absolute_quadratic")
        if mode == "absolute_quadratic":
            # Relief is a soft valley preference; invalid cells stay blocked.
            self.cost += 2.0 * ((filled - lo) / max(1.0, hi - lo)) ** 2
        elif mode == "valley_relative":
            from scipy import ndimage
            window = max(3, int(round(config.valley_window_m / cell_size_m)) | 1)
            # Invalid cells must not create artificial floors.
            floor = ndimage.minimum_filter(np.where(valid, elevation, np.inf), size=window, mode="nearest")
            floor = np.where(np.isfinite(floor), floor, filled)
            hand = np.clip(filled - floor, 0.0, None)
            self.cost += config.valley_cost_alpha * np.minimum(
                hand / max(1.0, config.valley_height_scale_m), config.valley_cost_cap)
        else:
            raise ValueError("guidance_cost_mode must be absolute_quadratic or valley_relative")
        # Edge repulsion never covers more than 10% of the ROI short side, so
        # small synthetic maps are not turned into one uniform penalty field.
        margin = min(getattr(config, "guidance_edge_margin_m", 0.0), 0.1 * min(rows, cols) * cell_size_m)
        if margin > 0.0:
            rr = np.minimum(np.arange(rows), np.arange(rows)[::-1])[:, None]
            cc = np.minimum(np.arange(cols), np.arange(cols)[::-1])[None, :]
            edge_distance = np.minimum(rr, cc) * cell_size_m
            self.cost += config.guidance_edge_cost * np.clip(1.0 - edge_distance / margin, 0.0, 1.0)
        for px, py, pradius in feedback_penalties:
            pr, pc = terrain.xy_to_rowcol(px, py)
            if terrain.in_bounds_rowcol(pr, pc):
                pr, pc = pr // self.stride, pc // self.stride
                radius_cells = max(1, int(pradius / cell_size_m))
                r_min, r_max = max(0, pr - radius_cells), min(rows, pr + radius_cells + 1)
                c_min, c_max = max(0, pc - radius_cells), min(cols, pc + radius_cells + 1)
                for rr in range(r_min, r_max):
                    for cc in range(c_min, c_max):
                        if (rr - pr) ** 2 + (cc - pc) ** 2 <= radius_cells ** 2:
                            self.cost[rr, cc] += 10.0
        gr, gc = terrain.xy_to_rowcol(goal.x_m, goal.y_m)
        if not terrain.in_bounds_rowcol(gr, gc):
            return
        gr, gc = gr // self.stride, gc // self.stride
        if not valid[gr, gc]:
            return
        self.distance[gr, gc] = 0.0
        self.targets[gr, gc] = max(self.targets[gr, gc], goal.z_msl_m - tolerance.altitude_m)
        self.ceilings[gr, gc] = goal.z_msl_m + tolerance.altitude_m
        if rows * cols >= _FAST_GUIDANCE_MIN_CELLS:
            self._build_fast(elevation, valid, clearance, gr, gc)
            return
        queue = [(0.0, gr, gc)]
        transform = terrain.roi.transform
        steps = [(dr, dc, self.stride * math.hypot(transform.a * dc + transform.b * dr,
                                    transform.d * dc + transform.e * dr))
                 for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
        while queue:
            distance, r, c = heapq.heappop(queue)
            if distance != self.distance[r, c]:
                continue
            for dr, dc, step in steps:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols and valid[nr, nc]):
                    continue
                # Reverse edge: flight goes from (nr,nc) to (r,c). A sharp
                # terrain drop needs descent distance, not just XY distance.
                delta = elevation[r, c] - elevation[nr, nc]
                flight_step = max(step, delta / self.climb_slope, -delta / self.descent_slope)
                candidate = distance + flight_step * (self.cost[r, c] + self.cost[nr, nc]) * .5
                if candidate < self.distance[nr, nc]:
                    self.distance[nr, nc] = candidate
                    self.targets[nr, nc] = max(elevation[nr, nc] + clearance,
                                                self.targets[r, c] - self.climb_slope * step)
                    self.ceilings[nr, nc] = self.ceilings[r, c] + self.descent_slope * step
                    heapq.heappush(queue, (candidate, nr, nc))

    def _build_fast(self, elevation, valid, clearance, gr, gc):
        """Same reverse-edge Dijkstra as the heapq loop, via scipy.csgraph.

        Targets/ceilings are propagated along the returned shortest-path tree in
        increasing distance order, exactly like the relaxation formulas above.
        """
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import dijkstra

        rows, cols = elevation.shape
        transform = self.terrain.roi.transform
        idx = np.arange(rows * cols).reshape(rows, cols)
        src, dst, weight, steps_list = [], [], [], []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if not (dr or dc):
                    continue
                step = self.stride * math.hypot(transform.a * dc + transform.b * dr,
                                                transform.d * dc + transform.e * dr)
                r0, r1 = max(0, -dr), rows - max(0, dr)
                c0, c1 = max(0, -dc), cols - max(0, dc)
                r_idx = idx[r0:r1, c0:c1].ravel()              # (r, c): nearer goal
                n_idx = idx[r0 + dr:r1 + dr, c0 + dc:c1 + dc].ravel()  # (nr, nc)
                ok = valid.ravel()[r_idx] & valid.ravel()[n_idx]
                r_idx, n_idx = r_idx[ok], n_idx[ok]
                delta = elevation.ravel()[r_idx] - elevation.ravel()[n_idx]
                flight = np.maximum(step, np.maximum(delta / self.climb_slope, -delta / self.descent_slope))
                src.append(r_idx)
                dst.append(n_idx)
                weight.append(flight * (self.cost.ravel()[r_idx] + self.cost.ravel()[n_idx]) * 0.5)
                steps_list.append(np.full(r_idx.shape, step))
        src = np.concatenate(src); dst = np.concatenate(dst)
        graph = coo_matrix((np.concatenate(weight), (src, dst)), shape=(rows * cols, rows * cols)).tocsr()
        step_graph = coo_matrix((np.concatenate(steps_list), (src, dst)), shape=(rows * cols, rows * cols)).tocsr()
        goal_index = int(idx[gr, gc])
        distance, pred = dijkstra(graph, directed=True, indices=goal_index, return_predecessors=True)
        self.distance = distance.reshape(rows, cols)
        order = np.argsort(distance, kind="stable")
        order = order[np.isfinite(distance[order])]
        targets = self.targets.ravel().copy()
        ceilings = self.ceilings.ravel().copy()
        floor = (elevation + clearance).ravel()
        pred_order = pred[order]
        has_parent = pred_order >= 0
        child, parent = order[has_parent], pred_order[has_parent]
        step_of = np.asarray(step_graph[parent, child]).ravel()
        tlist, clist, flist = targets.tolist(), ceilings.tolist(), floor.tolist()
        climb, descent = self.climb_slope, self.descent_slope
        for n, p, s in zip(child.tolist(), parent.tolist(), step_of.tolist()):
            tlist[n] = max(flist[n], tlist[p] - climb * s)
            clist[n] = clist[p] + descent * s
        self.targets = np.asarray(tlist).reshape(rows, cols)
        self.ceilings = np.asarray(clist).reshape(rows, cols)

    def value(self, array, pose, fallback):
        r, c = self.terrain.xy_to_rowcol(pose.x_m, pose.y_m)
        if not self.terrain.in_bounds_rowcol(r, c):
            return fallback
        r, c = r // self.stride, c // self.stride
        if not math.isfinite(array[r, c]):
            return fallback
        return float(array[r, c])

    def estimate(self, pose: PhysicalPose, lower_bound: float) -> float:
        distance = self.value(self.distance, pose, lower_bound)
        excess = max(0.0, pose.z_msl_m - self.value(self.ceilings, pose, pose.z_msl_m))
        estimate = distance + excess / self.descent_slope
        config = self.config
        if (config.enable_low_altitude_cost and config.low_altitude_cost_shape == "linear"
                and config.lambda_agl > 0.0 and math.isfinite(distance)):
            # Altitude above the guide target cannot vanish faster than the
            # descent slope allows; charge the linear AGL cost of that ramp so
            # lower-z duplicates of the same XY do not form an f-plateau.
            above = max(0.0, pose.z_msl_m - self.target(pose))
            ramp = min(distance, above / self.descent_slope)
            estimate += config.lambda_agl * (above / config.agl_cost_scale_m) * ramp * 0.5
        return max(lower_bound, estimate)

    def target(self, pose):
        return self.value(self.targets, pose, pose.z_msl_m)

    def multiplier(self, pose):
        return self.value(self.cost, pose, 1.0)


def _candidate_trajectories(
    pose: PhysicalPose, envelope: FixedWingKinematicEnvelope, config: PlannerConfig,
) -> Iterable[Tuple[str, Optional[PhysicalTrajectory]]]:
    spacing = config.primitive_sample_spacing_m
    yield "STRAIGHT_LEVEL", build_straight_level_trajectory(pose, 60.0, spacing).trajectory
    for direction, label in (("LEFT", "LEFT_LEVEL_TURN"), ("RIGHT", "RIGHT_LEVEL_TURN")):
        limit = envelope.level_turn(pose.z_msl_m, direction)
        if limit.availability == "AVAILABLE" and limit.level_turn is not None:
            yield label, build_level_turn_trajectory(pose, limit.level_turn, 15.0, spacing).trajectory
        else:
            yield label, None
    for mode, label in (("CLIMB", "STRAIGHT_CLIMB"), ("DESCENT", "STRAIGHT_DESCENT")):
        limit = envelope.straight_vertical(pose.z_msl_m, mode)
        if limit.availability == "AVAILABLE" and limit.signed_vertical_rate_mps is not None:
            yield label, build_straight_vertical_trajectory(
                pose, 60.0, limit.signed_vertical_rate_mps, spacing
            ).trajectory
        else:
            yield label, None
    if config.enable_combined_turns:
        for mode, mode_label in (("CLIMB", "CLIMBING"), ("DESCENT", "DESCENDING")):
            for direction, dir_label in (("LEFT", "LEFT"), ("RIGHT", "RIGHT")):
                label = f"{mode_label}_{dir_label}_TURN"
                limit = envelope.combined_turn(pose.z_msl_m, direction, mode)
                if limit.availability == "AVAILABLE" and limit.combined_turn is not None:
                    yield label, build_helical_turn_trajectory(
                        pose, limit.combined_turn, 15.0, spacing
                    ).trajectory
                else:
                    yield label, None


def _safety_reason(result: TrajectorySafetyResult) -> str:
    if result.is_safe:
        return "SAFE"
    return result.failure_reason or "OTHER_SAFETY"


def _median(values: Iterable[int]) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    midpoint = len(ordered) // 2
    return float(ordered[midpoint]) if len(ordered) % 2 else (ordered[midpoint - 1] + ordered[midpoint]) / 2.0


def pose_aware_astar_search(
    start: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    aircraft_profile: Optional[Any] = None,
    *,
    goal_tolerance: GoalTolerance,
    config: PlannerConfig = DEFAULT_CONFIG,
    max_expansions: Optional[int] = None,
    max_search_time_s: Optional[float] = None,
    diagnostics=None,
    envelope: Optional[FixedWingKinematicEnvelope] = None,
    feedback_penalties=(),
) -> PoseSearchResult:
    """Run bounded approximate single-representative fixed-wing A*."""
    if config.min_agl_m is None:
        raise ValueError("pose-aware search requires config.min_agl_m")
    if config.primitive_sample_spacing_m <= 0.0:
        raise ValueError("primitive_sample_spacing_m must be positive")
    if config.lateral_buffer_m < 0.0:
        raise ValueError("lateral_buffer_m must be non-negative")
    if not math.isfinite(config.search_heuristic_weight) or config.search_heuristic_weight < 1.0:
        raise ValueError("search_heuristic_weight must be finite and >= 1")
    if config.enable_low_altitude_cost:
        if (config.low_altitude_cost_shape not in ("linear", "quadratic", "capped_linear") or
                not all(math.isfinite(v) for v in (config.desired_agl_m, config.agl_cost_scale_m,
                        config.lambda_agl, config.full_penalty_agl_m, config.max_agl_cost_multiplier)) or
                config.agl_cost_scale_m <= 0 or config.lambda_agl < 0 or
                config.max_agl_cost_multiplier < 1 or
                (config.low_altitude_cost_shape == "capped_linear" and
                 config.full_penalty_agl_m <= config.desired_agl_m)):
            raise ValueError("invalid low-altitude cost configuration")
    if envelope is None:
        envelope = FixedWingKinematicEnvelope()
    influence_cache = TerrainInfluenceCache(terrain)
    started = time.perf_counter()
    start_safety = evaluate_physical_trajectory_safety(
        build_straight_level_trajectory(start, 0.0, config.primitive_sample_spacing_m).trajectory,
        terrain, config.min_agl_m, config.primitive_sample_spacing_m,
        planning_bounds=terrain.roi.bounds, lateral_buffer_m=config.lateral_buffer_m,
        terrain_influence_cache=influence_cache,
    )
    guidance = (_TerrainGuidance(terrain, goal, goal_tolerance, config, envelope, influence_cache, feedback_penalties)
                if (config.enable_terrain_guidance or feedback_penalties) else None)
    counter = itertools.count()
    next_node_id = itertools.count()
    start_key = search_key_for_pose(start, config)
    start_node = PoseSearchNode(next(next_node_id), start_key, start, 0.0, None, None, None)
    active: Dict[SearchKey, PoseSearchNode] = {start_key: start_node}
    all_nodes: Dict[int, PoseSearchNode] = {start_node.node_id: start_node}
    h_anchor_start = vertical_reachability_heuristic(start, goal, goal_tolerance, envelope)
    open_heap = [(config.search_heuristic_weight * h_anchor_start, next(counter), start_node.node_id)]
    open_guided = []
    if guidance is not None:
        h_guided_start = 0.0 if pose_in_goal(start, goal, goal_tolerance) else guidance.estimate(start, h_anchor_start)
        open_guided = [(config.search_heuristic_weight * h_guided_start, next(counter), start_node.node_id)]
    # Different z bins retain independent representatives. Neither goal-z
    # nor local-target error prunes across z bins.
    expanded_ids = set()
    expanded_keys = set()
    expanded = generated = rejected = 0
    collisions = replaced = existing_better = self_transitions = 0
    self_by_primitive: Counter[str] = Counter()
    max_open = 1
    reject_reasons: Counter[str] = Counter()
    if not start_safety.is_safe:
        open_heap.clear()
        open_guided.clear()
        reject_reasons["INVALID_START_" + _safety_reason(start_safety)] = 1
    generated_by_primitive: Counter[str] = Counter()
    open_inserted_by_primitive: Counter[str] = Counter()
    expanded_arrivals_by_primitive: Counter[str] = Counter()
    closest_xy, closest_3d = _goal_errors(start, goal)[0], _goal_errors(start, goal)[2]
    best_node, best_node_d3 = start_node, closest_3d
    maximum_altitude = start.z_msl_m
    progress: Dict[int, Dict[str, float]] = {}
    checkpoints = {1000, 5000, 10000, 20000, 30000}
    goal_node: Optional[PoseSearchNode] = None
    status = "no_path"
    step_count = 0

    while open_heap or open_guided:
        if max_search_time_s is not None and time.perf_counter() - started >= max_search_time_s:
            status = "timeout"
            break
        max_open = max(max_open, len(open_heap) + len(open_guided))
        step_count += 1
        if guidance is not None and open_guided:
            # K:1 Round-Robin interleave: guidance exploitation with anchor completeness
            k = max(1, getattr(config, "guidance_queue_ratio", 3))
            # Stale/duplicate pops do not advance the schedule, so the K:1
            # ratio holds for real expansions.
            use_guided = ((expanded + 1) % (k + 1) != 0)
            target_heap = open_guided if use_guided else open_heap
            if not target_heap:
                target_heap = open_guided if open_guided else open_heap
        else:
            target_heap = open_heap
        _, _, node_id = heapq.heappop(target_heap)
        node = all_nodes[node_id]
        # Versioned representative rule: old heap entries never expand after
        # a lower-g physical replacement of their same key.
        if active.get(node.key) is not node or node_id in expanded_ids:
            continue
        expanded_ids.add(node_id)
        expanded_keys.add(node.key)
        expanded += 1
        if node.incoming_primitive is not None:
            expanded_arrivals_by_primitive[node.incoming_primitive] += 1
        xy_error, _, d3_error = _goal_errors(node.end_pose, goal)
        if diagnostics is not None:
            diagnostics.on_expanded(expanded, node, xy_error, d3_error)
        closest_xy, closest_3d = min(closest_xy, xy_error), min(closest_3d, d3_error)
        if d3_error < best_node_d3 - 1e-12 or (
            math.isclose(d3_error, best_node_d3, rel_tol=0.0, abs_tol=1e-12)
            and node.g_cost < best_node.g_cost
        ):
            best_node, best_node_d3 = node, d3_error
        maximum_altitude = max(maximum_altitude, node.end_pose.z_msl_m)
        if expanded in checkpoints:
            progress[expanded] = {"best_xy_distance_m": closest_xy, "best_3d_distance_m": closest_3d,
                                  "maximum_altitude_msl_m": maximum_altitude}
        if pose_in_goal(node.end_pose, goal, goal_tolerance):
            goal_node, status = node, "success"
            break
        if max_expansions is not None and expanded >= max_expansions:
            status = "search_limit_reached"
            break

        for primitive, trajectory in _candidate_trajectories(node.end_pose, envelope, config):
            generated += 1
            generated_by_primitive[primitive] += 1
            if diagnostics is not None:
                diagnostics.on_successor(node, primitive, "GENERATED", None)
            if trajectory is None:
                if diagnostics is not None:
                    diagnostics.on_successor(node, primitive, "UNAVAILABLE_CAPABILITY", None)
                rejected += 1
                reject_reasons["UNAVAILABLE_CAPABILITY"] += 1
                continue
            safety = evaluate_physical_trajectory_safety(
                trajectory, terrain, config.min_agl_m, config.primitive_sample_spacing_m,
                planning_bounds=terrain.roi.bounds, lateral_buffer_m=config.lateral_buffer_m,
                terrain_influence_cache=influence_cache,
                record_sample_terrain=config.enable_low_altitude_cost,
            )
            if not safety.is_safe:
                if diagnostics is not None:
                    diagnostics.on_successor(node, primitive, _safety_reason(safety), None)
                rejected += 1
                reject_reasons[_safety_reason(safety)] += 1
                continue
            end_pose = trajectory.end_pose
            key = search_key_for_pose(end_pose, config)
            if diagnostics is not None:
                diagnostics.on_successor(node, primitive, "PHYSICALLY_VALID", end_pose)
            if key == node.key:
                if diagnostics is not None:
                    diagnostics.on_successor(node, primitive, "SAME_KEY_SELF_TRANSITION", end_pose)
                self_transitions += 1
                self_by_primitive[primitive] += 1
                rejected += 1
                reject_reasons["SAME_KEY_SELF_TRANSITION"] += 1
                continue
            candidate_g = node.g_cost + _trajectory_edge_cost(trajectory, safety, config, guidance)
            existing = active.get(key)
            if existing is not None:
                collisions += 1
                if candidate_g >= existing.g_cost - 1e-12:
                    if diagnostics is not None:
                        diagnostics.on_successor(node, primitive, "SAME_KEY_DOMINANCE", end_pose)
                    existing_better += 1
                    rejected += 1
                    reject_reasons["SAME_KEY_DOMINANCE"] += 1
                    continue
                replaced += 1
            successor = PoseSearchNode(
                next(next_node_id), key, end_pose, candidate_g, node.node_id, trajectory, primitive,
            )
            active[key] = successor
            all_nodes[successor.node_id] = successor
            open_inserted_by_primitive[primitive] += 1
            if diagnostics is not None:
                diagnostics.on_successor(
                    node, primitive,
                    "OPEN_INSERTED_REPLACEMENT" if existing is not None else "OPEN_INSERTED",
                    successor.end_pose,
                )
            h_anchor_end = vertical_reachability_heuristic(end_pose, goal, goal_tolerance, envelope)
            f_score = candidate_g + config.search_heuristic_weight * h_anchor_end
            heapq.heappush(open_heap, (f_score, next(counter), successor.node_id))
            if guidance is not None:
                h_guided_end = 0.0 if pose_in_goal(end_pose, goal, goal_tolerance) else guidance.estimate(end_pose, h_anchor_end)
                f_guided = candidate_g + config.search_heuristic_weight * h_guided_end
                heapq.heappush(open_guided, (f_guided, next(counter), successor.node_id))

    if status == "no_path" and not open_heap and not open_guided:
        status = "no_path"
    runtime = time.perf_counter() - started
    if diagnostics is not None:
        diagnostics.on_termination(active, all_nodes, expanded_ids, tuple(open_heap) + tuple(open_guided))
    def reconstruct(node: PoseSearchNode) -> Tuple[PoseSearchNode, ...]:
        reverse = []
        cursor: Optional[PoseSearchNode] = node
        while cursor is not None:
            reverse.append(cursor)
            cursor = all_nodes[cursor.parent_node_id] if cursor.parent_node_id is not None else None
        return tuple(reversed(reverse))

    best_path = reconstruct(goal_node if goal_node is not None else best_node)
    if goal_node is not None:
        path = best_path
        total_cost = goal_node.g_cost
        min_agl = min((evaluate_physical_trajectory_safety(
            trajectory, terrain, config.min_agl_m, config.primitive_sample_spacing_m,
            planning_bounds=terrain.roi.bounds, lateral_buffer_m=config.lateral_buffer_m,
            terrain_influence_cache=influence_cache,
        ).min_agl_m for trajectory in (item.incoming_trajectory for item in path[1:]) if trajectory is not None), default=start_safety.min_agl_m)
        goal_xy_error, goal_z_error, _ = _goal_errors(goal_node.end_pose, goal)
        final_heading = goal_node.end_pose.heading_deg
    else:
        path, total_cost, min_agl = (), float("nan"), float("nan")
        goal_xy_error = goal_z_error = final_heading = float("nan")

    xy_to_heading: Dict[Tuple[int, int], set] = defaultdict(set)
    xy_to_z: Dict[Tuple[int, int], set] = defaultdict(set)
    for key in expanded_keys:
        xy = (key.x_bin, key.y_bin)
        xy_to_heading[xy].add(key.heading_bin)
        xy_to_z[xy].add(key.z_bin)
    heading_counts = [len(v) for v in xy_to_heading.values()]
    z_counts = [len(v) for v in xy_to_z.values()]
    reasons = dict(sorted(reject_reasons.items()))
    termination = {"success": "FOUND", "no_path": "OPEN_EXHAUSTED", "search_limit_reached": "EXPANSION_LIMIT", "timeout": "TIMEOUT"}[status]
    if not start_safety.is_safe:
        termination = "INVALID_START_" + _safety_reason(start_safety)
    return PoseSearchResult(
        status == "success", status, termination, path, total_cost, expanded, generated, rejected, reasons,
        max_open, runtime, len(expanded_keys), len(xy_to_heading),
        sum(heading_counts) / len(heading_counts) if heading_counts else float("nan"), _median(heading_counts), max(heading_counts, default=0),
        sum(z_counts) / len(z_counts) if z_counts else float("nan"), _median(z_counts), max(z_counts, default=0),
        collisions, replaced, existing_better, self_transitions, dict(sorted(self_by_primitive.items())),
        dict(sorted(generated_by_primitive.items())), dict(sorted(open_inserted_by_primitive.items())),
        dict(sorted(expanded_arrivals_by_primitive.items())), min_agl, closest_xy, closest_3d, maximum_altitude, progress,
        config.lateral_buffer_m, config.min_agl_m, goal_xy_error, goal_z_error, final_heading,
        best_path,
    )


__all__ = [
    "GoalPose", "GoalTolerance", "PoseSearchNode", "PoseSearchResult", "SearchKey",
    "active_primitive_names",
    "navigation_bearing_deg", "pose_aware_astar_search", "pose_in_goal", "search_key_for_pose",
]
