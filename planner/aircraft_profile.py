"""Step ALG-0: aircraft-performance LUT interface boundary.

This module does NOT change search behavior. Nothing in planner/astar.py,
planner/candidate_z.py, or planner/corridor.py imports from here, and
nothing here is called during a search. It exists purely so that when a
planner-safe aircraft LUT arrives (JSBSim side: U4 RAW characterization ->
U4.1/U4.2 validation/interpolation/derating), the planner has exactly ONE
clean place to read it from -- never the LUT file's own internal
structure, never a JSBSim property name.

Frozen stack this interface is scoped to (see project.md "JSBSim UAV
Pivot"): maneuver target -> frozen IAS/Vz/bank/beta outer-loop -> c172r
stock FCS -> JSBSim. JSBSim itself never runs during planner search --
by the time a LUT reaches AircraftProfile, all JSBSim work is already
done and baked into static exact-grid data.

RAW vs planner-safe (the one thing this module is strictest about):
U4's output (jsbsim/results/aircraft_lut_raw.json as of this stage) is a
RAW characterization artifact -- full of JSBSim-internal telemetry
(engine property names, controller-saturation/settling/repeatability
diagnostics, per-repeat run_ids) that the planner must never see, and
explicitly marked `"artifact_type": "RAW_AIRCRAFT_LUT"` /
`"planner_ready": false` on the JSBSim side. A RAW LUT is refused here
by construction (LutNotPlannerSafeError) -- see load_aircraft_profile().
The planner-safe shape this module DOES accept is deliberately simpler
than the RAW file: schema_version/aircraft_id/controller_stack_id/
provenance_id/units/grids + a turn_table/vertical_table of
(inputs -> status/outputs) rows, no telemetry. That derivation
(validated/interpolated/derated) is U4.1/U4.2's job, not this module's --
this stage only defines the CONTRACT the result must satisfy.

Units: every planner-facing quantity is SI -- meters, meters/second,
degrees, seconds. A LUT declaring anything else (knots, feet, radians)
fails schema validation rather than being silently converted.

Exact-grid only: no interpolation, no extrapolation, no derating. A
query for an altitude/bank/vertical-speed value not present in the LUT's
own declared grid raises AircraftProfileQueryError -- it does not snap
to the "nearest" value silently and does not fall back to an
approximation. This is a deliberate, temporary restriction for this
stage, not a permanent design limit.

Status semantics: every query returns one of "VALID" / "INFEASIBLE" /
"UNKNOWN" (mirroring JSBSim characterization semantics -- UNKNOWN means
"tested but inconclusive", e.g. a controller-tracking gate that didn't
settle cleanly, not "never measured"). UNKNOWN is never silently
upgraded to VALID or downgraded to INFEASIBLE anywhere in this module --
callers that cannot handle UNKNOWN must treat it as "cannot use this
maneuver", never as an implicit VALID.

Not implemented here, on purpose (future work, see project.md):
  - interpolation between grid points
  - derating (safety margin shrinkage of raw envelopes)
  - a heading-aware state (x, y, z, heading) or aircraft-aware motion
    primitives (straight / left turn / right turn / climb / descent /
    climbing turn) -- this module only defines the query CONTRACT those
    would eventually call
  - orbit/loiter/climbing-spiral maneuvers
A future heading-aware planner built on top of this interface must still
satisfy the permanent requirement: DIRECTLY INFEASIBLE != UNREACHABLE --
a maneuver this LUT reports INFEASIBLE at one exact grid point does not
mean the goal is unreachable; longer routes, turns, or loiter/climbing-
spiral maneuvers may still reach it. This module does not implement that
search-side reasoning; it only promises not to hide the distinction.
"""
import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Tuple

# --- Contract constants -----------------------------------------------
# The one schema version this stage's loader accepts. A LUT declaring
# anything else is refused (LutSchemaError), not best-effort-parsed.
SUPPORTED_SCHEMA_VERSION = 1

RAW_LUT_STAGE = "raw"
PLANNER_SAFE_LUT_STAGE = "planner_safe"

# The frozen aircraft/controller-stack identity this planner build is
# scoped to (project.md "JSBSim UAV Pivot" -- c172r, stock FCS, frozen
# maneuver-target outer-loop). CONTROLLER_STACK_ID is a placeholder
# label, not a real hash -- no controller-stack hashing scheme exists on
# the JSBSim side yet. It still gives fail-fast protection today: a LUT
# declaring any OTHER controller_stack_id is refused, and once a real
# hash exists this constant is a one-line update, not a redesign.
EXPECTED_AIRCRAFT_ID = "c172r"
EXPECTED_CONTROLLER_STACK_ID = "c172r-stock-fcs-frozen-outer-loop-v1"  # PLACEHOLDER, see above

VALID_STATUS_VALUES = frozenset({"VALID", "INFEASIBLE", "UNKNOWN"})

# Every planner-facing quantity is SI. A LUT must declare exactly these
# unit strings per quantity (see module docstring "Units").
REQUIRED_UNITS = {"altitude": "m", "speed": "m/s", "angle": "deg", "time": "s"}

_GRID_MATCH_TOL = 1e-6  # exact-grid equality tolerance, float noise only -- not interpolation


# --- Exceptions (mirrors planner/terrain_cache.py's TerrainCacheError family) ---

class AircraftProfileError(Exception):
    """Base class -- an aircraft-profile LUT request could not be satisfied safely."""


class LutSchemaError(AircraftProfileError):
    """A required field is missing/malformed, or schema_version is unsupported."""


class LutProvenanceError(AircraftProfileError):
    """aircraft_id / controller_stack_id does not match what the caller expects."""


class LutNotPlannerSafeError(AircraftProfileError):
    """lut_stage != "planner_safe" (e.g. a RAW characterization LUT) -- refused outright."""


class AircraftProfileQueryError(AircraftProfileError):
    """A query used a value not present on the LUT's own exact grid, or a
    (altitude, speed_context, bank/direction) / (altitude, speed_context,
    desired_vz) combination the LUT never characterized. No interpolation,
    no nearest-neighbor fallback -- see module docstring."""


# --- Data model ----------------------------------------------------------

@dataclass(frozen=True)
class AircraftLutManifest:
    """Identity, provenance, and grid declaration a planner-safe LUT must
    carry -- validated once at load time so query code never has to
    re-check any of this."""
    schema_version: int
    lut_stage: str  # "raw" | "planner_safe"
    aircraft_id: str
    controller_stack_id: str
    provenance_id: str
    units: Mapping[str, str]
    speed_contexts: Tuple[str, ...]
    altitude_grid_m: Tuple[float, ...]
    turn_bank_grid_deg: Tuple[float, ...]
    turn_directions: Tuple[str, ...]
    vertical_speed_grid_mps: Tuple[float, ...]


@dataclass(frozen=True)
class TurnQueryResult:
    """Response to AircraftProfile.turn_query(). turn_radius_m/
    turn_rate_deg_s/actual_bank_deg are None whenever the LUT entry
    itself has no meaningful value for them (e.g. INFEASIBLE); metadata
    is an opaque passthrough of whatever optional fields the planner-safe
    LUT chose to carry (never interpreted here)."""
    status: str  # "VALID" | "INFEASIBLE" | "UNKNOWN"
    turn_radius_m: Optional[float]
    turn_rate_deg_s: Optional[float]
    actual_bank_deg: Optional[float]
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class VerticalQueryResult:
    """Response to AircraftProfile.vertical_query()."""
    status: str  # "VALID" | "INFEASIBLE" | "UNKNOWN"
    achievable_vz_mps: Optional[float]
    gamma_deg: Optional[float]
    metadata: Mapping[str, Any] = field(default_factory=dict)


class AircraftProfile:
    """The single interface the planner will use to query aircraft-motion
    feasibility once a planner-safe LUT exists. Not wired into search
    (Step ALG-0) -- construct via load_aircraft_profile()/
    load_aircraft_profile_from_dict(), never by hand.

    Exact-grid only: turn_query()/vertical_query() require every input to
    match a value already present in the manifest's own grids (within
    float noise), and the specific combination to have an entry in the
    LUT -- see AircraftProfileQueryError.
    """

    def __init__(
        self,
        manifest: AircraftLutManifest,
        turn_table: Mapping[Tuple[float, str, float, str], TurnQueryResult],
        vertical_table: Mapping[Tuple[float, str, float], VerticalQueryResult],
    ):
        self.manifest = manifest
        self._turn_table = turn_table
        self._vertical_table = vertical_table

    def _snap(self, value: float, grid: Tuple[float, ...], what: str) -> float:
        for g in grid:
            if math.isclose(value, g, rel_tol=0.0, abs_tol=_GRID_MATCH_TOL):
                return g
        raise AircraftProfileQueryError(
            f"{what}={value} is not on this LUT's exact grid {grid} -- "
            f"no interpolation/extrapolation is supported by this interface"
        )

    def turn_query(self, altitude_m: float, speed_context: str, bank_deg: float, direction: str) -> TurnQueryResult:
        """Exact-grid turn-performance lookup. Raises AircraftProfileQueryError
        if altitude_m/bank_deg is off the declared grid, if speed_context/
        direction is not one the LUT declares, or if the LUT simply has no
        entry for the (altitude, speed_context, bank, direction) combination
        (a partially-populated grid is expected -- e.g. bank=0 may only ever
        appear with direction="STRAIGHT")."""
        if speed_context not in self.manifest.speed_contexts:
            raise AircraftProfileQueryError(
                f"speed_context={speed_context!r} not declared by this LUT "
                f"(available: {self.manifest.speed_contexts})"
            )
        if direction not in self.manifest.turn_directions:
            raise AircraftProfileQueryError(
                f"direction={direction!r} not declared by this LUT "
                f"(available: {self.manifest.turn_directions})"
            )
        alt = self._snap(altitude_m, self.manifest.altitude_grid_m, "altitude_m")
        bank = self._snap(bank_deg, self.manifest.turn_bank_grid_deg, "bank_deg")
        key = (alt, speed_context, bank, direction)
        result = self._turn_table.get(key)
        if result is None:
            raise AircraftProfileQueryError(
                f"no turn-table entry for (altitude_m={alt}, speed_context={speed_context!r}, "
                f"bank_deg={bank}, direction={direction!r}) -- this exact combination was never "
                f"characterized, even though each value individually is on the LUT's grid"
            )
        return result

    def vertical_query(self, altitude_m: float, speed_context: str, desired_vz_mps: float) -> VerticalQueryResult:
        """Exact-grid vertical-performance lookup. Same off-grid/uncharacterized
        rules as turn_query()."""
        if speed_context not in self.manifest.speed_contexts:
            raise AircraftProfileQueryError(
                f"speed_context={speed_context!r} not declared by this LUT "
                f"(available: {self.manifest.speed_contexts})"
            )
        alt = self._snap(altitude_m, self.manifest.altitude_grid_m, "altitude_m")
        vz = self._snap(desired_vz_mps, self.manifest.vertical_speed_grid_mps, "desired_vz_mps")
        key = (alt, speed_context, vz)
        result = self._vertical_table.get(key)
        if result is None:
            raise AircraftProfileQueryError(
                f"no vertical-table entry for (altitude_m={alt}, speed_context={speed_context!r}, "
                f"desired_vz_mps={vz}) -- this exact combination was never characterized, even "
                f"though each value individually is on the LUT's grid"
            )
        return result


# --- Loading / validation --------------------------------------------------

def _require(data: Mapping, key: str) -> Any:
    if key not in data:
        raise LutSchemaError(f"planner-safe LUT is missing required field {key!r}")
    return data[key]


def _require_str(data: Mapping, key: str) -> str:
    value = _require(data, key)
    if not isinstance(value, str) or not value:
        raise LutSchemaError(f"field {key!r} must be a non-empty string, got {value!r}")
    return value


def _require_number_tuple(data: Mapping, key: str) -> Tuple[float, ...]:
    value = _require(data, key)
    if not isinstance(value, (list, tuple)) or not value:
        raise LutSchemaError(f"field {key!r} must be a non-empty list of numbers, got {value!r}")
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
        raise LutSchemaError(f"field {key!r} must contain only numbers, got {value!r}")
    return tuple(float(v) for v in value)


def _require_str_tuple(data: Mapping, key: str) -> Tuple[str, ...]:
    value = _require(data, key)
    if not isinstance(value, (list, tuple)) or not value:
        raise LutSchemaError(f"field {key!r} must be a non-empty list of strings, got {value!r}")
    if not all(isinstance(v, str) and v for v in value):
        raise LutSchemaError(f"field {key!r} must contain only non-empty strings, got {value!r}")
    return tuple(value)


def _validate_units(data: Mapping) -> Mapping[str, str]:
    units = _require(data, "units")
    if not isinstance(units, Mapping):
        raise LutSchemaError(f"field 'units' must be an object, got {units!r}")
    for quantity, expected in REQUIRED_UNITS.items():
        actual = units.get(quantity)
        if actual != expected:
            raise LutSchemaError(
                f"units[{quantity!r}] must be {expected!r} (SI) -- this interface never converts "
                f"units, got {actual!r}"
            )
    return dict(units)


def _build_manifest(data: Mapping) -> AircraftLutManifest:
    schema_version = _require(data, "schema_version")
    if not isinstance(schema_version, int) or isinstance(schema_version, bool):
        raise LutSchemaError(f"'schema_version' must be an int, got {schema_version!r}")

    lut_stage = _require_str(data, "lut_stage")
    if lut_stage not in (RAW_LUT_STAGE, PLANNER_SAFE_LUT_STAGE):
        raise LutSchemaError(
            f"'lut_stage' must be {RAW_LUT_STAGE!r} or {PLANNER_SAFE_LUT_STAGE!r}, got {lut_stage!r}"
        )

    return AircraftLutManifest(
        schema_version=schema_version,
        lut_stage=lut_stage,
        aircraft_id=_require_str(data, "aircraft_id"),
        controller_stack_id=_require_str(data, "controller_stack_id"),
        provenance_id=_require_str(data, "provenance_id"),
        units=_validate_units(data),
        speed_contexts=_require_str_tuple(data, "speed_contexts"),
        altitude_grid_m=_require_number_tuple(data, "altitude_grid_m"),
        turn_bank_grid_deg=_require_number_tuple(data, "turn_bank_grid_deg"),
        turn_directions=_require_str_tuple(data, "turn_directions"),
        vertical_speed_grid_mps=_require_number_tuple(data, "vertical_speed_grid_mps"),
    )


def _validate_status(status: Any, where: str) -> str:
    if status not in VALID_STATUS_VALUES:
        raise LutSchemaError(f"{where}: status must be one of {sorted(VALID_STATUS_VALUES)}, got {status!r}")
    return status


def _build_turn_table(
    data: Mapping, manifest: AircraftLutManifest
) -> Dict[Tuple[float, str, float, str], TurnQueryResult]:
    rows = _require(data, "turn_table")
    if not isinstance(rows, (list, tuple)):
        raise LutSchemaError(f"'turn_table' must be a list, got {rows!r}")
    table: Dict[Tuple[float, str, float, str], TurnQueryResult] = {}
    for i, row in enumerate(rows):
        where = f"turn_table[{i}]"
        altitude_m = _require(row, "altitude_m")
        speed_context = _require_str(row, "speed_context")
        bank_deg = _require(row, "bank_deg")
        direction = _require_str(row, "direction")
        if speed_context not in manifest.speed_contexts:
            raise LutSchemaError(f"{where}: speed_context={speed_context!r} not in declared speed_contexts")
        if direction not in manifest.turn_directions:
            raise LutSchemaError(f"{where}: direction={direction!r} not in declared turn_directions")
        status = _validate_status(_require(row, "status"), where)
        key = (float(altitude_m), speed_context, float(bank_deg), direction)
        table[key] = TurnQueryResult(
            status=status,
            turn_radius_m=row.get("turn_radius_m"),
            turn_rate_deg_s=row.get("turn_rate_deg_s"),
            actual_bank_deg=row.get("actual_bank_deg"),
            metadata=row.get("metadata", {}),
        )
    return table


def _build_vertical_table(
    data: Mapping, manifest: AircraftLutManifest
) -> Dict[Tuple[float, str, float], VerticalQueryResult]:
    rows = _require(data, "vertical_table")
    if not isinstance(rows, (list, tuple)):
        raise LutSchemaError(f"'vertical_table' must be a list, got {rows!r}")
    table: Dict[Tuple[float, str, float], VerticalQueryResult] = {}
    for i, row in enumerate(rows):
        where = f"vertical_table[{i}]"
        altitude_m = _require(row, "altitude_m")
        speed_context = _require_str(row, "speed_context")
        desired_vz_mps = _require(row, "desired_vz_mps")
        if speed_context not in manifest.speed_contexts:
            raise LutSchemaError(f"{where}: speed_context={speed_context!r} not in declared speed_contexts")
        status = _validate_status(_require(row, "status"), where)
        key = (float(altitude_m), speed_context, float(desired_vz_mps))
        table[key] = VerticalQueryResult(
            status=status,
            achievable_vz_mps=row.get("achievable_vz_mps"),
            gamma_deg=row.get("gamma_deg"),
            metadata=row.get("metadata", {}),
        )
    return table


def load_aircraft_profile_from_dict(
    data: Mapping,
    *,
    expected_aircraft_id: str = EXPECTED_AIRCRAFT_ID,
    expected_controller_stack_id: str = EXPECTED_CONTROLLER_STACK_ID,
) -> AircraftProfile:
    """Validate a planner-safe LUT dict and build the query interface.
    Fail-fast, in this order: presence/type of schema fields, lut_stage
    (a RAW LUT is refused here before any further check even runs),
    schema_version, aircraft_id/controller_stack_id, units, then table
    contents. Never returns a partially-valid AircraftProfile."""
    manifest = _build_manifest(data)

    if manifest.lut_stage != PLANNER_SAFE_LUT_STAGE:
        raise LutNotPlannerSafeError(
            f"refusing to load a {manifest.lut_stage!r} LUT (provenance_id={manifest.provenance_id!r}) -- "
            f"only {PLANNER_SAFE_LUT_STAGE!r} LUTs may be used by the planner. A RAW characterization "
            f"artifact must go through validation/interpolation/derating first (see module docstring)."
        )
    if manifest.schema_version != SUPPORTED_SCHEMA_VERSION:
        raise LutSchemaError(
            f"unsupported schema_version={manifest.schema_version} (this loader only supports "
            f"{SUPPORTED_SCHEMA_VERSION})"
        )
    if manifest.aircraft_id != expected_aircraft_id:
        raise LutProvenanceError(
            f"LUT aircraft_id={manifest.aircraft_id!r} does not match expected {expected_aircraft_id!r}"
        )
    if manifest.controller_stack_id != expected_controller_stack_id:
        raise LutProvenanceError(
            f"LUT controller_stack_id={manifest.controller_stack_id!r} does not match expected "
            f"{expected_controller_stack_id!r} -- a LUT built under a different controller stack "
            f"cannot be assumed to describe the same flight envelope"
        )

    turn_table = _build_turn_table(data, manifest)
    vertical_table = _build_vertical_table(data, manifest)
    return AircraftProfile(manifest, turn_table, vertical_table)


def load_aircraft_profile(
    path: str,
    *,
    expected_aircraft_id: str = EXPECTED_AIRCRAFT_ID,
    expected_controller_stack_id: str = EXPECTED_CONTROLLER_STACK_ID,
) -> AircraftProfile:
    """File-based wrapper around load_aircraft_profile_from_dict()."""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return load_aircraft_profile_from_dict(
        data,
        expected_aircraft_id=expected_aircraft_id,
        expected_controller_stack_id=expected_controller_stack_id,
    )
