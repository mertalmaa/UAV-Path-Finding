"""Roadmap Step 3E: production home for the sparse/lazy Z representation.

Everything below CandidateZGenerator and its supporting dataclasses
(TerrainMetadata, TerrainMetadataStore, MissionContext, MotionContext,
GeneratorStats) is moved here VERBATIM from scripts/step3b_sparse_lazy_z_
prototype.py, which validated it (5 controlled cases A-E, real synthetic
terrain, PASS -- see project.md "Step 3B") before this module existed.
No logic changed in the move; scripts/step3b_sparse_lazy_z_prototype.py,
scripts/step3d_real_terrain_integration.py and scripts/
step3d1_goal_tolerance_regression.py now import these names from here
instead of defining/duplicating them, so there is exactly one
implementation, not two drifting copies.

CacheBackedTerrainMetadataStore is NEW for Step 3E: it generalizes Step
3D's CacheBacked60mStore (which was hard-coded to FACTOR60=2) to any
pooling factor already present in a planner.terrain_cache.TerrainCache,
so this module has no hidden 60m assumption of its own -- the caller
decides which cached factor to read.

CandidateZGenerator.floor_for() remains a per-cell BOUND CHECK only (the
cheapest terrain-derived lower bound on safe altitude at a cell), never a
restriction to a small discrete event set -- a candidate that clears the
floor still goes through the full evaluate_primitive() safety check
downstream. See planner/astar.py's _generate_neighbors for exactly how
production wires this in as a pre-filter (Step 3E) -- this module itself
has no dependency on planner.astar and performs no search.
"""
import math
from dataclasses import dataclass, field
from time import perf_counter
from typing import Dict, FrozenSet, List, Optional, Tuple

from planner.config import DEFAULT_CONFIG
from planner.terrain import TerrainQuery
from planner.terrain_cache import TerrainCache


@dataclass(frozen=True)
class TerrainMetadata:
    row: int
    col: int
    elevation_m: float  # NaN if out-of-bounds/nodata


class TerrainMetadataStore:
    """STATIC/OFFLINE terrain metadata cache (Step 3B section 9's STATIC
    bucket), live-DEM-backed via a TerrainQuery.

    The ONLY place in this store that ever touches the raster (via
    TerrainQuery). Every (row,col) is read from the DEM at most once; all
    later requests for the same cell are pure dict lookups. Search code
    never re-derives terrain metadata -- it only calls .get(), which is
    either a cache hit or a one-time cache-filling miss. See
    CacheBackedTerrainMetadataStore below for the persistent-cache-backed
    (zero raw DEM reads) alternative with the same .get() interface.
    """

    def __init__(self, terrain: TerrainQuery):
        self._terrain = terrain
        self._cache: Dict[Tuple[int, int], TerrainMetadata] = {}
        self.hits = 0
        self.misses = 0
        self.lookup_time_total_s = 0.0

    def get(self, row: int, col: int) -> TerrainMetadata:
        key = (row, col)
        cached = self._cache.get(key)
        if cached is not None:
            self.hits += 1
            return cached
        self.misses += 1
        t0 = perf_counter()
        if not self._terrain.in_bounds_rowcol(row, col):
            md = TerrainMetadata(row, col, float("nan"))
        else:
            r = self._terrain.elevation_at_rowcol(row, col)
            md = TerrainMetadata(row, col, r.elevation if r.valid else float("nan"))
        self.lookup_time_total_s += perf_counter() - t0
        self._cache[key] = md
        return md

    @property
    def hit_rate(self) -> float:
        total = self.hits + self.misses
        return self.hits / total if total else 0.0


class CacheBackedTerrainMetadataStore:
    """Step 3E: same .get(row,col)->TerrainMetadata interface as
    TerrainMetadataStore above, but backed by a persistent
    planner.terrain_cache.TerrainCache instead of a live TerrainQuery --
    generalizes Step 3D's CacheBacked60mStore (which hard-coded
    FACTOR60=2) to any pooling factor already present in the cache.
    Reads the cache's MAX-pooled elevation array for `factor` (the same
    safety-relevant, conservative surface planner.coarse.build_coarse_dem
    would produce -- see planner/terrain_cache.py's own docstring on what
    is cached and why). raw_dem_reads is fixed at 0 and exposed only for
    the same structural assertion Step 3D used: this class has no code
    path that could ever touch the original DEM file, so the count can
    never become nonzero.
    """

    def __init__(self, cache: TerrainCache, factor: int):
        if factor not in cache.available_factors:
            raise ValueError(f"factor={factor} not present in this cache (available: {cache.available_factors})")
        self._cache = cache
        self._factor = factor
        self.raw_dem_reads = 0

    def get(self, row: int, col: int) -> TerrainMetadata:
        try:
            elev = self._cache.max_elevation(self._factor, row, col)
        except (IndexError, KeyError):
            elev = float("nan")
        return TerrainMetadata(row, col, elev)


@dataclass(frozen=True)
class MissionContext:
    """CLASS B events -- mission-dependent, search-independent."""
    start_rowcol: Tuple[int, int]
    start_z_msl: float
    goal_rowcol: Tuple[int, int]
    goal_z_msl: float
    ceiling_msl: float
    min_agl_m: float
    # Ladder step used ONLY to snap the CLASS-A validity-transition altitude.
    # Deliberately kept EQUAL to config.z_step_m (production, 20m) -- this
    # module does not decide or propose a different Z spacing. The sparsity
    # comes from generating few EVENTS per cell (floor/ceiling/start/goal)
    # instead of the whole dense ladder, and from LAZY instantiation, not
    # from widening this step.
    z_step_m: float = DEFAULT_CONFIG.z_step_m


@dataclass(frozen=True)
class MotionContext:
    """CLASS C events -- PROVISIONAL (Step 3A.1). A real implementation
    would derive climb/descent-reachable altitude corridors from terrain
    profile + primitive angle envelope, deterministically and independent
    of search order (Step 3A.1 point 5). That algorithm was NOT designed or
    implemented in Step 3A.1/3B and is NOT implemented here either --
    `provisional_events` always returns an empty set. Kept as a real
    parameter (not omitted) purely so the CandidateZGenerator signature is
    forward-compatible with a future implementation, exactly like
    AircraftSafetyContext stays an empty placeholder for now. Step 3E does
    NOT change this status: CLASS-C remains unresolved/provisional.
    """

    def provisional_events(self, row: int, col: int) -> FrozenSet[float]:
        return frozenset()


@dataclass
class GeneratorStats:
    call_count: int = 0
    times_s: List[float] = field(default_factory=list)


class CandidateZGenerator:
    """Deterministic Z-candidate generator, Step 3A.1 contract:

        CandidateZGenerator(store, mission_context, motion_context=None)

    `store` exposes .get(row,col)->TerrainMetadata (either
    TerrainMetadataStore or CacheBackedTerrainMetadataStore -- both are
    interchangeable here, and floor_for()'s behavior is identical either
    way; only WHERE the elevation value ultimately comes from differs).
    mission/motion_context are as above. generate(row, col) returns a
    sorted tuple of unique candidate Z_msl values, combining:

      CLASS A (static terrain): the lowest z on the mission's z_step_m
        ladder that clears terrain+min_agl ("validity-transition
        altitude"), plus the mission ceiling as an always-present upper
        bound event.
      CLASS B (mission): start_z_msl at the start cell, goal_z_msl at the
        goal cell.
      CLASS C (motion, only if motion_context is not None): whatever
        motion_context.provisional_events returns -- empty, see
        MotionContext docstring.

    Deterministic: depends only on (row, col) plus the CONTENTS of store/
    mission/motion_context, never on call order or on what the search has
    done so far. store is itself a pure memoizing cache of real DEM/cache
    values (same value every time for the same cell), so repeated calls
    with the same store/mission/motion_context always return the same
    tuple.
    """

    def __init__(self, store, mission: MissionContext,
                 motion_context: Optional[MotionContext] = None):
        self.store = store
        self.mission = mission
        self.motion_context = motion_context
        self.stats = GeneratorStats()

    def floor_for(self, row: int, col: int) -> Optional[float]:
        """CLASS-A validity-transition altitude at (row,col), or None if
        the cell has no terrain data or no ladder altitude fits under the
        mission ceiling."""
        md = self.store.get(row, col)
        if math.isnan(md.elevation_m):
            return None
        z_step = self.mission.z_step_m
        floor = math.ceil((md.elevation_m + self.mission.min_agl_m) / z_step) * z_step
        return floor if floor <= self.mission.ceiling_msl else None

    def generate(self, row: int, col: int) -> Tuple[float, ...]:
        t0 = perf_counter()
        candidates = set()
        floor = self.floor_for(row, col)
        if floor is not None:
            candidates.add(floor)                       # CLASS A
            candidates.add(self.mission.ceiling_msl)     # CLASS A/B boundary
        if (row, col) == self.mission.start_rowcol:
            candidates.add(self.mission.start_z_msl)     # CLASS B
        if (row, col) == self.mission.goal_rowcol:
            candidates.add(self.mission.goal_z_msl)       # CLASS B
        if self.motion_context is not None:
            candidates |= self.motion_context.provisional_events(row, col)  # CLASS C
        result = tuple(sorted(candidates))
        self.stats.call_count += 1
        self.stats.times_s.append(perf_counter() - t0)
        return result
