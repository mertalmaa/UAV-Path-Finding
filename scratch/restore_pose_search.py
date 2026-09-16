with open('planner/pose_search.py', 'r', encoding='utf-8') as f:
    text = f.read()

# 1. Edge cost
part1_old = """    physical_length = _trajectory_3d_length(trajectory)
    if not config.enable_low_altitude_cost:
        return physical_length"""

part1_new = """    physical_length = _trajectory_3d_length(trajectory)
    penalty_cost = 0.0
    if guidance is not None and getattr(guidance, "feedback_penalties", ()):
        for s in trajectory.samples:
            for px, py, pradius in guidance.feedback_penalties:
                if (s.x_m - px) ** 2 + (s.y_m - py) ** 2 <= pradius ** 2:
                    penalty_cost += 500.0
                    break
    if not config.enable_low_altitude_cost:
        return physical_length + penalty_cost"""

assert part1_old in text
text = text.replace(part1_old, part1_new, 1)

part2_old = """        for first, second, first_elevation, second_elevation in zip(
            trajectory.samples, trajectory.samples[1:], elevations, elevations[1:]
        )
    )"""

part2_new = """        for first, second, first_elevation, second_elevation in zip(
            trajectory.samples, trajectory.samples[1:], elevations, elevations[1:]
        )
    ) + penalty_cost"""

assert part2_old in text
text = text.replace(part2_old, part2_new, 1)

# 2. _TerrainGuidance class
guidance_old = """class _TerrainGuidance:
    \"\"\"Optional 2D topographic guidance; never grants physical feasibility.

    Dijkstra also propagates a backwards climb envelope along its successor
    tree. This is a soft reference only: actual curved flight still passes
    full trajectory safety. Grid costs need not lower-bound continuous costs.
    \"\"\"

    def __init__(self, terrain, goal, tolerance, config, envelope, influence_cache):
        self.terrain = terrain
        field = influence_cache.field(config.lateral_buffer_m)
        valid = field.valid
        elevation = field.elevation_msl
        self.distance = np.full(elevation.shape, np.inf)
        self.targets = elevation + max(config.desired_agl_m, config.min_agl_m)
        self.cost = np.ones(elevation.shape)
        if not np.any(valid):
            return
        lo, hi = float(elevation[valid].min()), float(elevation[valid].max())
        filled = np.where(valid, elevation, hi)
        # Relief is a soft valley preference; invalid cells stay blocked.
        self.cost += 2.0 * ((filled - lo) / max(1.0, hi - lo)) ** 2
        rows, cols = elevation.shape
        gr, gc = terrain.xy_to_rowcol(goal.x_m, goal.y_m)
        if not terrain.in_bounds_rowcol(gr, gc) or not valid[gr, gc]:
            return
        self.distance[gr, gc] = 0.0
        self.targets[gr, gc] = max(self.targets[gr, gc], goal.z_msl_m - tolerance.altitude_m)
        queue = [(0.0, gr, gc)]
        transform = terrain.roi.transform
        steps = [(dr, dc, math.hypot(transform.a * dc + transform.b * dr,
                                    transform.d * dc + transform.e * dr))
                 for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
        climb_slope = envelope.max_climb_rate_mps / envelope.horizontal_speed_mps
        while queue:
            distance, r, c = heapq.heappop(queue)
            if distance != self.distance[r, c]:
                continue
            for dr, dc, step in steps:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols and valid[nr, nc]):
                    continue
                candidate = distance + step * (self.cost[r, c] + self.cost[nr, nc]) * .5
                if candidate < self.distance[nr, nc]:
                    self.distance[nr, nc] = candidate
                    self.targets[nr, nc] = max(elevation[nr, nc] + max(config.desired_agl_m, config.min_agl_m),
                                                self.targets[r, c] - climb_slope * step)
                    heapq.heappush(queue, (candidate, nr, nc))

    def value(self, array, pose, fallback):
        r, c = self.terrain.xy_to_rowcol(pose.x_m, pose.y_m)
        if not self.terrain.in_bounds_rowcol(r, c) or not math.isfinite(array[r, c]):
            return fallback
        return float(array[r, c])

    def target(self, pose):
        return self.value(self.targets, pose, pose.z_msl_m)

    def multiplier(self, pose):
        return self.value(self.cost, pose, 1.0)"""

guidance_new = """class _TerrainGuidance:
    \"\"\"Optional 2D topographic guidance; never grants physical feasibility.

    Dijkstra propagates backwards climb/descent envelopes along its successor
    tree on a sampled grid. This is a soft reference only: actual flight passes
    full trajectory safety. Grid costs need not lower-bound continuous costs.
    \"\"\"

    def __init__(self, terrain, goal, tolerance, config, envelope, influence_cache, feedback_penalties=()):
        self.terrain = terrain
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
        # Relief is a soft valley preference; invalid cells stay blocked.
        self.cost += 2.0 * ((filled - lo) / max(1.0, hi - lo)) ** 2
        rows, cols = elevation.shape
        cell_size_m = abs(terrain.roi.transform.a) * self.stride
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
        return max(lower_bound, distance + excess * 40.0 / 5.0)

    def target(self, pose):
        return self.value(self.targets, pose, pose.z_msl_m)

    def multiplier(self, pose):
        return self.value(self.cost, pose, 1.0)"""

assert guidance_old in text
text = text.replace(guidance_old, guidance_new, 1)

# 3. pose_aware_astar_search signature and body
sig_old = """def pose_aware_astar_search(
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
) -> PoseSearchResult:"""

sig_new = """def pose_aware_astar_search(
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
) -> PoseSearchResult:"""

assert sig_old in text
text = text.replace(sig_old, sig_new, 1)

# 4. guidance init and Pareto pruning removal
body_old = """    guidance = (_TerrainGuidance(terrain, goal, goal_tolerance, config, envelope, influence_cache)
                if config.enable_terrain_guidance and config.enable_low_altitude_cost else None)
    counter = itertools.count()
    next_node_id = itertools.count()
    start_key = search_key_for_pose(start, config)
    start_node = PoseSearchNode(next(next_node_id), start_key, start, 0.0, None, None, None)
    active: Dict[SearchKey, PoseSearchNode] = {start_key: start_node}
    all_nodes: Dict[int, PoseSearchNode] = {start_node.node_id: start_node}
    def heuristic(pose):
        lower = vertical_reachability_heuristic(pose, goal, goal_tolerance, envelope)
        if guidance is None or pose_in_goal(pose, goal, goal_tolerance):
            return lower
        return max(lower, guidance.value(guidance.distance, pose, lower))
    open_heap = [(config.search_heuristic_weight * heuristic(start), next(counter), start_node.node_id)]
    # This is approximate pruning, not a proof of physical dominance: distinct
    # altitudes can have different obstacle clearance. Keep it configurable.
    frontier = defaultdict(list)
    def altitude_error(pose):
        if guidance is not None:
            return abs(pose.z_msl_m - guidance.target(pose))
        if config.enable_low_altitude_cost:
            ground = terrain.query(pose.x_m, pose.y_m)
            if ground.valid:
                remaining = math.hypot(pose.x_m - goal.x_m, pose.y_m - goal.y_m)
                target = max(ground.elevation + max(config.desired_agl_m, config.min_agl_m),
                             goal.z_msl_m - envelope.max_climb_rate_mps * remaining / envelope.horizontal_speed_mps)
                return abs(pose.z_msl_m - target)
        return abs(pose.z_msl_m - goal.z_msl_m)
    frontier[(start_key.x_bin, start_key.y_bin, start_key.heading_bin)].append(start_node)"""

body_new = """    guidance = (_TerrainGuidance(terrain, goal, goal_tolerance, config, envelope, influence_cache, feedback_penalties)
                if (config.enable_terrain_guidance or feedback_penalties) else None)
    counter = itertools.count()
    next_node_id = itertools.count()
    start_key = search_key_for_pose(start, config)
    start_node = PoseSearchNode(next(next_node_id), start_key, start, 0.0, None, None, None)
    active: Dict[SearchKey, PoseSearchNode] = {start_key: start_node}
    all_nodes: Dict[int, PoseSearchNode] = {start_node.node_id: start_node}
    def heuristic(pose):
        lower = vertical_reachability_heuristic(pose, goal, goal_tolerance, envelope)
        if guidance is None or pose_in_goal(pose, goal, goal_tolerance):
            return lower
        return guidance.estimate(pose, lower)
    open_heap = [(config.search_heuristic_weight * heuristic(start), next(counter), start_node.node_id)]
    # Different z bins retain independent representatives. Neither goal-z
    # nor local-target error prunes across z bins."""

assert body_old in text
text = text.replace(body_old, body_new, 1)

# 5. Inner loop pareto removal
inner_old = """            candidate_g = node.g_cost + _trajectory_edge_cost(trajectory, safety, config, guidance)
            xyh = (key.x_bin, key.y_bin, key.heading_bin)
            if config.enable_pareto_z_pruning and not pose_in_goal(end_pose, goal, goal_tolerance):
                representatives = [item for item in frontier[xyh] if active.get(item.key) is item]
                frontier[xyh] = representatives
                if any(item.g_cost <= candidate_g + 1e-9 and
                       altitude_error(item.end_pose) <= altitude_error(end_pose) + 1e-9
                       for item in representatives):
                    rejected += 1
                    reject_reasons["PARETO_Z_DOMINANCE"] += 1
                    continue
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
            if config.enable_pareto_z_pruning:
                frontier[xyh] = [item for item in frontier[xyh]
                                 if not (candidate_g <= item.g_cost + 1e-9 and
                                         altitude_error(end_pose) <= altitude_error(item.end_pose) + 1e-9)]
                frontier[xyh].append(successor)
            all_nodes[successor.node_id] = successor"""

inner_new = """            candidate_g = node.g_cost + _trajectory_edge_cost(trajectory, safety, config, guidance)
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
            all_nodes[successor.node_id] = successor"""

assert inner_old in text
text = text.replace(inner_old, inner_new, 1)

with open('planner/pose_search.py', 'w', encoding='utf-8') as f:
    f.write(text)

print('Successfully restored planner/pose_search.py!')
