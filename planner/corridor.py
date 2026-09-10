"""Stage 36: fine-grid XY corridor mask around a coarse guide path.

Pure geometry -- no A*, no cost, no altitude. A corridor is an axis-
aligned-grid boolean mask over the FINE ROI: True where a fine cell
center's minimum horizontal (XY-only) distance to the coarse path's
polyline is <= half_width_m. Z/altitude is never consulted or
constrained here -- the fine planner (a later, separate stage) is meant
to be free to choose ANY safe MSL within a corridor cell; narrowing that
is explicitly out of scope for this module.
"""
from collections import deque
from typing import List, Tuple

import numpy as np

from planner.roi import ROIData

Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]


def fine_grid_centers(roi: ROIData) -> Tuple[np.ndarray, np.ndarray]:
    """(X, Y) UTM coordinates of every fine cell CENTER, as 2D arrays of
    shape (height, width) -- vectorized via the ROI's own affine
    transform (never a hardcoded resolution), matching exactly what
    planner.terrain.TerrainQuery.rowcol_to_xy(row, col) would return for
    each individual cell, one call at a time, without the Python-loop cost.
    """
    t = roi.transform
    if t.b != 0.0 or t.d != 0.0:
        raise ValueError("fine_grid_centers assumes a north-up (no rotation/shear) transform")
    cols = np.arange(roi.width)
    rows = np.arange(roi.height)
    x_1d = t.c + (cols + 0.5) * t.a
    y_1d = t.f + (rows + 0.5) * t.e
    x_2d, y_2d = np.meshgrid(x_1d, y_1d)  # both (height, width)
    return x_2d, y_2d


def _distance_to_segment(x: np.ndarray, y: np.ndarray, p1: Point2, p2: Point2) -> np.ndarray:
    """Vectorized point-to-segment distance for every (x, y) against one
    segment p1->p2 (a plain 2D geometric computation: project onto the
    segment, clamp to [0,1], measure distance to that clamped point)."""
    x1, y1 = p1
    x2, y2 = p2
    dx, dy = x2 - x1, y2 - y1
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0.0:
        return np.hypot(x - x1, y - y1)
    t = ((x - x1) * dx + (y - y1) * dy) / seg_len_sq
    t_clamped = np.clip(t, 0.0, 1.0)
    proj_x = x1 + t_clamped * dx
    proj_y = y1 + t_clamped * dy
    return np.hypot(x - proj_x, y - proj_y)


def min_distance_to_polyline(x: np.ndarray, y: np.ndarray, polyline_xy: List[Point2]) -> np.ndarray:
    """Minimum distance from every (x, y) to the polyline (a sequence of
    XY points connected by straight segments) -- the running elementwise
    min over each segment's own vectorized distance. A single-point
    "polyline" (len==1) degenerates to plain point distance."""
    if len(polyline_xy) == 1:
        return np.hypot(x - polyline_xy[0][0], y - polyline_xy[0][1])
    min_dist = np.full(x.shape, np.inf)
    for p1, p2 in zip(polyline_xy, polyline_xy[1:]):
        seg_dist = _distance_to_segment(x, y, p1, p2)
        min_dist = np.minimum(min_dist, seg_dist)
    return min_dist


def build_xy_corridor_mask(
    roi: ROIData, polyline_xy: List[Point2], half_width_m: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Returns (mask, min_distance) -- mask[row,col]=True iff that fine
    cell's center is within half_width_m of the polyline (XY only, Z
    never consulted). min_distance is the same array of raw distances,
    useful for width diagnostics without recomputing."""
    x, y = fine_grid_centers(roi)
    min_distance = min_distance_to_polyline(x, y, polyline_xy)
    mask = min_distance <= half_width_m
    return mask, min_distance


def build_z_guide_grid(roi: ROIData, polyline_xyz: List[Point3]) -> np.ndarray:
    """Stage 37.1: for every fine cell center, the coarse 3D path's own
    altitude (MSL), LINEARLY INTERPOLATED along whichever segment is
    closest to that cell in XY (the same clamped-projection distance as
    min_distance_to_polyline -- this just also carries Z along for the
    ride, picking the Z of whichever segment WINS the XY-distance
    comparison, elementwise, via a running argmin). A GUIDANCE value
    only -- never a safety bound; the fine planner's own AGL/terrain/
    NoData/angle checks are completely unaffected by it.
    """
    x, y = fine_grid_centers(roi)
    if len(polyline_xyz) == 1:
        return np.full(x.shape, polyline_xyz[0][2])

    best_dist = np.full(x.shape, np.inf)
    best_z = np.full(x.shape, np.nan)
    for p1, p2 in zip(polyline_xyz, polyline_xyz[1:]):
        x1, y1, z1 = p1
        x2, y2, z2 = p2
        dx, dy = x2 - x1, y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq == 0.0:
            t_clamped = np.zeros(x.shape)
        else:
            t = ((x - x1) * dx + (y - y1) * dy) / seg_len_sq
            t_clamped = np.clip(t, 0.0, 1.0)
        proj_x = x1 + t_clamped * dx
        proj_y = y1 + t_clamped * dy
        dist = np.hypot(x - proj_x, y - proj_y)
        proj_z = z1 + t_clamped * (z2 - z1)

        better = dist < best_dist
        best_dist = np.where(better, dist, best_dist)
        best_z = np.where(better, proj_z, best_z)
    return best_z


def bfs_connected(mask: np.ndarray, start_rc: Tuple[int, int], goal_rc: Tuple[int, int]) -> bool:
    """8-connected BFS reachability check within `mask` (True cells only),
    from start_rc to goal_rc. Plain Python/numpy -- no scipy dependency."""
    height, width = mask.shape
    if not mask[start_rc] or not mask[goal_rc]:
        return False
    visited = np.zeros_like(mask, dtype=bool)
    visited[start_rc] = True
    queue = deque([start_rc])
    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    while queue:
        r, c = queue.popleft()
        if (r, c) == goal_rc:
            return True
        for dr, dc in neighbors:
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width and mask[nr, nc] and not visited[nr, nc]:
                visited[nr, nc] = True
                queue.append((nr, nc))
    return visited[goal_rc]
