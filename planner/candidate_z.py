"""Deterministic CandidateZ altitude-event generation.\n\nTerrain metadata supplies a conservative per-cell clearance-floor event; the\nmission supplies its exact start, goal, and ceiling events. Production A* uses\nthese events as its sole altitude-successor authority, while full primitive\nevaluation remains the final terrain/AGL safety authority.\n"""
import math
from dataclasses import dataclass
from time import perf_counter
from typing import Dict, FrozenSet, Optional, Tuple

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
    safety-relevant conservative block-maximum surface stored by the terrain
    cache -- see planner/terrain_cache.py's own docstring on what
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
    # Quantization used only to round the conservative CLASS-A terrain-floor
    # event upward. It does not define search-state identity or successor
    # arithmetic; CandidateZ still emits only a few events per cell.
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
    """Constant-memory CandidateZ generation diagnostics."""

    call_count: int = 0
    total_time_s: float = 0.0
    max_time_s: float = 0.0
    min_time_s: float = math.inf

    @property
    def average_time_s(self) -> float:
        return self.total_time_s / self.call_count if self.call_count else 0.0


class CandidateZGenerator:
    """Deterministic Z-candidate generator, Step 3A.1 contract:

        CandidateZGenerator(store, mission_context, motion_context=None)

    `store` exposes .get(row,col)->TerrainMetadata (either
    TerrainMetadataStore or CacheBackedTerrainMetadataStore -- both are
    interchangeable here, and floor_for()'s behavior is identical either
    way; only WHERE the elevation value ultimately comes from differs).
    mission/motion_context are as above. generate(row, col) returns a
    sorted tuple of unique candidate Z_msl values, combining:

      CLASS A (static terrain): the conservative floor event rounded upward
        by mission.z_step_m, plus the mission ceiling as an upper-bound event.
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
        elapsed_s = perf_counter() - t0
        self.stats.call_count += 1
        self.stats.total_time_s += elapsed_s
        self.stats.max_time_s = max(self.stats.max_time_s, elapsed_s)
        self.stats.min_time_s = min(self.stats.min_time_s, elapsed_s)
        return result

    def is_representable(self, row: int, col: int, z_msl: float, z_tol: float = 1e-6) -> bool:
        """Step REP-1.1: the unified representability predicate generate()
        was designed to describe, usable as a direct membership test
        against ANY candidate altitude (arithmetic-derived or otherwise) --
        generate()'s own return value is a 2-4 element boundary-EVENT set
        (floor, ceiling, start/goal), deliberately too sparse to double as
        that test, since every interior altitude strictly between floor
        and ceiling is never one of those events; see project.md "Step
        REP-1"/"Step REP-1.1" for the full audit.

        True iff z_msl is:
          CLASS A: floor_for(row,col) <= z_msl <= mission.ceiling_msl --
            a continuous range with no quantization requirement. This says
            nothing about how a candidate was computed, only whether it
            clears terrain+min_agl up to the mission ceiling.
          CLASS B: z_msl exactly equals this mission's start or goal
            altitude, AT that specific mission-designated cell (an exact,
            possibly off-lattice mission event).

        Pure membership test: never derives or caches a new candidate,
        same determinism/no-history guarantee as floor_for()."""
        if (row, col) == self.mission.start_rowcol and math.isclose(z_msl, self.mission.start_z_msl, abs_tol=z_tol):
            return True
        if (row, col) == self.mission.goal_rowcol and math.isclose(z_msl, self.mission.goal_z_msl, abs_tol=z_tol):
            return True
        floor = self.floor_for(row, col)
        if floor is None:
            return False
        return floor - z_tol <= z_msl <= self.mission.ceiling_msl + z_tol
