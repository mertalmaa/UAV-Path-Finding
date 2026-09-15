"""Validated aircraft-capability profile boundary for production search.\n\nThe loader accepts the canonical tested planner-safe profile schema. CandidateZ\nsuccessor generation uses these altitude-dependent limits to size and validate\nconcrete multi-cell vertical motion.\n"""
import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Tuple

# --- Contract constants -----------------------------------------------
SUPPORTED_SCHEMA_VERSION = 3
EXPECTED_PROFILE_SCHEMA_ID = "tested_safe_envelope_aircraft_capability_profile_v3"
# The one profile_stage value this loader accepts as planner-safe. Anything
# else (a RAW characterization artifact, or a future not-yet-recognized
# intermediate stage) is refused -- see load_aircraft_profile_from_dict().
EXPECTED_PROFILE_STAGE = "tested_planner_safe_envelope"

# The frozen aircraft/controller-stack identity THIS planner build defaults
# to (project.md "JSBSim UAV Pivot" -- c172p, stock FCS, frozen U1/U3 outer
# loop). Both are overridable per load_aircraft_profile() call -- passing a
# different expected_aircraft_id/expected_controller_stack_id is exactly how
# a second aircraft's profile gets loaded through this SAME code, with no
# aircraft-specific branch anywhere in this module.
EXPECTED_AIRCRAFT_ID = "c172p"
EXPECTED_CONTROLLER_STACK_ID = "c172p-stock-fcs-frozen-u1-u3-outer-loop-production-mixture-v1"

VALID_AVAILABILITY_VALUES = frozenset({"AVAILABLE", "UNAVAILABLE", "OUT_OF_DOMAIN"})

# Every planner-facing quantity is SI (see module docstring "Units").
REQUIRED_UNITS = {"altitude": "m", "angle": "deg", "radius": "m", "speed": "m/s", "turn_rate": "deg/s"}

# The maneuver families this stage's query contract needs. A profile is
# free to carry MORE families than this (forward-compatible); missing any
# of these is a schema error.
REQUIRED_CAPABILITY_FAMILIES = (
    "straight", "level_turn", "straight_climb", "straight_descent", "climbing_turn", "descending_turn",
)

_ALTITUDE_TOL = 1e-6  # exact-grid match tolerance, float noise only -- not a real interpolation window


# --- Exceptions (mirrors planner/terrain_cache.py's TerrainCacheError family) ---

class AircraftProfileError(Exception):
    """Base class -- an aircraft-profile request could not be satisfied safely."""


class LutSchemaError(AircraftProfileError):
    """A required field is missing/malformed, schema_version/profile_schema_id
    is unsupported, or a family's rows are internally inconsistent
    (duplicate rows, availability/planner_safe mismatch, bad status value)."""


class LutProvenanceError(AircraftProfileError):
    """aircraft_id / controller_stack_id does not match what the caller expects."""


class LutNotPlannerSafeError(AircraftProfileError):
    """planner_ready is False, or profile_stage isn't the recognized
    planner-safe value (e.g. a RAW characterization artifact) -- refused
    before any other validation runs."""


class AircraftProfileQueryError(AircraftProfileError):
    """A query used an altitude outside the profile's declared domain
    (OUT_OF_DOMAIN is returned as a normal ManeuverQueryResult, NOT this
    exception -- see straight_query/turn_query/vertical_query/
    combined_query), an unrecognized direction/mode, or asked a maneuver
    family for a combination it structurally cannot answer."""


# --- Data model ----------------------------------------------------------

@dataclass(frozen=True)
class AircraftProfileManifest:
    """Identity, provenance, and domain declaration a planner-safe profile
    must carry -- validated once at load time so query code never has to
    re-check any of this."""
    schema_version: int
    profile_schema_id: str
    profile_stage: str
    planner_ready: bool
    aircraft_id: str
    controller_stack_id: str
    provenance_id: str
    units: Mapping[str, str]
    altitude_grid_m: Tuple[float, ...]
    min_altitude_m: float
    max_altitude_m: float
    nominal_ias_context_mps: float


@dataclass(frozen=True)
class ManeuverQueryResult:
    """Response shape shared by straight_query/turn_query/vertical_query/
    combined_query -- deliberately generic (no aircraft- or maneuver-
    specific named fields) so the SAME type serves every query kind and
    every aircraft's profile.

    planner_safe: the row's own `planner_safe` dict, passed through
      UNCHANGED -- e.g. for a turn row this typically has turn_radius_m/
      turn_rate_deg_s/expected_ias_mps/power_margin_norm; for a climb row,
      climb_vz_mps/highest_tested_safe_climb_command_mps/etc. Empty when
      availability != "AVAILABLE". THIS is the production-facing answer.
    measured: the row's own `measured` dict (raw telemetry) when this
      result came from an EXACT row match, else None (no single measured
      data point exists at an interpolated altitude). Diagnostic ONLY --
      never use this to compute a primitive; see module docstring.
    evidence: the row's own `evidence` dict (confidence/validation_quality/
      why), when available from an exact match.
    reason_unavailable: convenience string (evidence's own
      `reason_unavailable` or `limiting_reason`, whichever the family
      uses) when availability != "AVAILABLE", else None.
    altitude_resolution: "EXACT" | "LINEAR_INTERPOLATED" |
      "CONSERVATIVE_ENDPOINT" | "INTERPOLATION_UNSUPPORTED" |
      "OUT_OF_DOMAIN" -- how this altitude was resolved; see module
      docstring "Altitude resolution".
    """
    availability: str  # "AVAILABLE" | "UNAVAILABLE" | "OUT_OF_DOMAIN"
    altitude_resolution: str
    planner_safe: Mapping[str, Any] = field(default_factory=dict)
    measured: Optional[Mapping[str, Any]] = None
    evidence: Optional[Mapping[str, Any]] = None
    reason_unavailable: Optional[str] = None


class _CapabilityFamily:
    """One `capabilities.<name>` entry: its query_policy plus a fast
    lookup over its rows. No maneuver-specific knowledge lives here --
    `find_row` matches purely on whatever key/value filters the caller
    (AircraftProfile's query methods) supplies."""

    def __init__(self, name: str, raw: Mapping[str, Any], altitude_grid: Tuple[float, ...]):
        self.name = name
        policy = raw.get("query_policy")
        if not isinstance(policy, Mapping):
            raise LutSchemaError(f"capabilities.{name}.query_policy is missing or not an object")
        self.safe_interpolation = policy.get("safe_interpolation")
        if self.safe_interpolation not in ("LINEAR", "CONSERVATIVE_ENDPOINT", "NONE"):
            raise LutSchemaError(
                f"capabilities.{name}.query_policy.safe_interpolation must be one of "
                f"LINEAR/CONSERVATIVE_ENDPOINT/NONE, got {self.safe_interpolation!r}"
            )
        blocked = policy.get("availability_interpolation_blocked_ranges_m", [])
        if not isinstance(blocked, list):
            raise LutSchemaError(f"capabilities.{name}.query_policy.availability_interpolation_blocked_ranges_m must be a list")
        self.blocked_ranges: List[Tuple[float, float]] = []
        for rng in blocked:
            if not (isinstance(rng, (list, tuple)) and len(rng) == 2):
                raise LutSchemaError(f"capabilities.{name}: bad blocked range {rng!r}")
            self.blocked_ranges.append((float(rng[0]), float(rng[1])))

        rows = raw.get("rows")
        if not isinstance(rows, list) or not rows:
            raise LutSchemaError(f"capabilities.{name}.rows must be a non-empty list")
        self.rows: List[Mapping[str, Any]] = []
        seen_keys = set()
        for i, row in enumerate(rows):
            where = f"capabilities.{name}.rows[{i}]"
            if "altitude_m" not in row:
                raise LutSchemaError(f"{where}: missing altitude_m")
            availability = row.get("availability")
            if availability not in VALID_AVAILABILITY_VALUES:
                raise LutSchemaError(f"{where}: availability must be one of {sorted(VALID_AVAILABILITY_VALUES)}, got {availability!r}")
            has_safe = row.get("planner_safe") is not None
            if availability == "AVAILABLE" and not has_safe:
                raise LutSchemaError(f"{where}: availability=AVAILABLE but planner_safe is null/missing")
            if availability != "AVAILABLE" and has_safe:
                raise LutSchemaError(f"{where}: availability={availability!r} but planner_safe is non-null (must be null when unavailable)")
            if "measured" not in row:
                raise LutSchemaError(f"{where}: missing 'measured' field -- measured/planner_safe separation must be structurally present")
            # Generic duplicate-row check: altitude plus whatever other scalar
            # discriminating keys this row declares (direction, command_bank_deg, ...) --
            # never a hard-coded key list.
            dim_keys = tuple(sorted(k for k in ("direction", "command_bank_deg") if k in row))
            dup_key = (float(row["altitude_m"]),) + tuple(row[k] for k in dim_keys)
            if dup_key in seen_keys:
                raise LutSchemaError(f"{where}: duplicate row for key {dup_key}")
            seen_keys.add(dup_key)
            self.rows.append(row)

    def find_row(self, altitude_m: float, filters: Mapping[str, Any]) -> Optional[Mapping[str, Any]]:
        matches = [
            r for r in self.rows
            if math.isclose(float(r["altitude_m"]), altitude_m, rel_tol=0.0, abs_tol=_ALTITUDE_TOL)
            and all(r.get(k) == v for k, v in filters.items())
        ]
        if len(matches) > 1:
            raise LutSchemaError(f"capabilities.{self.name}: ambiguous rows for altitude={altitude_m}, filters={filters}")
        return matches[0] if matches else None

    def in_blocked_range(self, altitude_m: float) -> bool:
        return any(lo <= altitude_m <= hi for lo, hi in self.blocked_ranges)

    @staticmethod
    def _reason(row: Mapping[str, Any]) -> Optional[str]:
        evidence = row.get("evidence") or {}
        return evidence.get("reason_unavailable") or evidence.get("limiting_reason")


def _linear_blend(lo: Mapping[str, Any], hi: Mapping[str, Any], frac: float) -> Dict[str, Any]:
    """Blend every numeric key present in BOTH planner_safe dicts;
    non-numeric or one-sided keys are dropped rather than guessed at."""
    out = {}
    for k, v_lo in lo.items():
        v_hi = hi.get(k)
        if isinstance(v_lo, (int, float)) and not isinstance(v_lo, bool) and isinstance(v_hi, (int, float)) and not isinstance(v_hi, bool):
            out[k] = v_lo + (v_hi - v_lo) * frac
    return out


def _pick_conservative_endpoint(row_lo: Mapping[str, Any], row_hi: Mapping[str, Any]) -> Tuple[Mapping[str, Any], str]:
    """CONSERVATIVE_ENDPOINT tie-break between two bracketing AVAILABLE
    rows -- see module docstring "Altitude resolution" for why this is a
    magnitude comparison (smaller |*_command_mps|), not an altitude-
    direction one. Falls back to the lower-altitude row, clearly flagged,
    if neither row's planner_safe carries a "*_command_mps"-suffixed key."""
    def command_key(ps: Mapping[str, Any]) -> Optional[str]:
        keys = [k for k in ps if k.endswith("_command_mps")]
        return keys[0] if len(keys) == 1 else None

    ps_lo, ps_hi = row_lo["planner_safe"], row_hi["planner_safe"]
    k_lo, k_hi = command_key(ps_lo), command_key(ps_hi)
    if k_lo is not None and k_lo == k_hi:
        return (row_lo, "CONSERVATIVE_ENDPOINT") if abs(ps_lo[k_lo]) <= abs(ps_hi[k_hi]) else (row_hi, "CONSERVATIVE_ENDPOINT")
    return row_lo, "CONSERVATIVE_ENDPOINT_LOWER_FALLBACK"


class AircraftProfile:
    """The single interface the planner will use to query aircraft-motion
    feasibility. Not wired into search (Step ALG-1 keeps that boundary) --
    construct via load_aircraft_profile()/load_aircraft_profile_from_dict(),
    never by hand.

    Every query method reads its answer out of the loaded artifact's own
    `capabilities` rows at call time -- nothing here is hard-coded per
    aircraft (see module docstring)."""

    def __init__(self, manifest: AircraftProfileManifest, families: Mapping[str, _CapabilityFamily]):
        self.manifest = manifest
        self._families = families

    # -- generic resolution, shared by all four public query methods --

    def _resolve(self, family_name: str, altitude_m: float, filters: Mapping[str, Any]) -> ManeuverQueryResult:
        m = self.manifest
        if altitude_m < m.min_altitude_m - _ALTITUDE_TOL or altitude_m > m.max_altitude_m + _ALTITUDE_TOL:
            return ManeuverQueryResult(availability="OUT_OF_DOMAIN", altitude_resolution="OUT_OF_DOMAIN")

        family = self._families[family_name]

        exact = self._snap_to_grid(altitude_m)
        if exact is not None:
            row = family.find_row(exact, filters)
            if row is None:
                raise AircraftProfileQueryError(
                    f"{family_name}: no row for altitude_m={exact}, filters={dict(filters)} -- this "
                    f"exact combination was never characterized, even though the altitude itself is "
                    f"on the canonical grid"
                )
            return self._result_from_row(row, "EXACT")

        lo_alt, hi_alt = self._bracket(altitude_m)
        row_lo = family.find_row(lo_alt, filters)
        row_hi = family.find_row(hi_alt, filters)
        if row_lo is None or row_hi is None:
            raise AircraftProfileQueryError(
                f"{family_name}: cannot bracket altitude_m={altitude_m} for filters={dict(filters)} -- "
                f"missing row at {lo_alt if row_lo is None else hi_alt}"
            )

        if family.in_blocked_range(altitude_m):
            reason = "altitude falls inside this family's availability_interpolation_blocked_ranges_m"
            return ManeuverQueryResult(availability="UNAVAILABLE", altitude_resolution="INTERMEDIATE_BLOCKED",
                                        reason_unavailable=reason)

        if row_lo["availability"] != "AVAILABLE" or row_hi["availability"] != "AVAILABLE":
            # Conservative: availability is never interpolated (see module docstring).
            # An intermediate point is only AVAILABLE if BOTH bracketing anchors are.
            reason = family._reason(row_lo) if row_lo["availability"] != "AVAILABLE" else family._reason(row_hi)
            return ManeuverQueryResult(availability="UNAVAILABLE", altitude_resolution="INTERMEDIATE_UNAVAILABLE",
                                        reason_unavailable=reason)

        if family.safe_interpolation == "LINEAR":
            frac = (altitude_m - lo_alt) / (hi_alt - lo_alt) if hi_alt > lo_alt else 0.0
            blended = _linear_blend(row_lo["planner_safe"], row_hi["planner_safe"], frac)
            return ManeuverQueryResult(availability="AVAILABLE", altitude_resolution="LINEAR_INTERPOLATED",
                                        planner_safe=blended)
        if family.safe_interpolation == "CONSERVATIVE_ENDPOINT":
            chosen_row, resolution = _pick_conservative_endpoint(row_lo, row_hi)
            return ManeuverQueryResult(availability="AVAILABLE", altitude_resolution=resolution,
                                        planner_safe=dict(chosen_row["planner_safe"]))
        # safe_interpolation == "NONE"
        return ManeuverQueryResult(
            availability="UNAVAILABLE", altitude_resolution="INTERPOLATION_UNSUPPORTED",
            reason_unavailable=f"{family_name} does not support intermediate-altitude queries (safe_interpolation=NONE)",
        )

    def _snap_to_grid(self, altitude_m: float) -> Optional[float]:
        for g in self.manifest.altitude_grid_m:
            if math.isclose(altitude_m, g, rel_tol=0.0, abs_tol=_ALTITUDE_TOL):
                return g
        return None

    def _bracket(self, altitude_m: float) -> Tuple[float, float]:
        grid = self.manifest.altitude_grid_m
        below = [g for g in grid if g < altitude_m]
        above = [g for g in grid if g > altitude_m]
        if not below or not above:
            raise AircraftProfileQueryError(f"altitude_m={altitude_m} cannot be bracketed by the canonical grid {grid}")
        return max(below), min(above)

    @staticmethod
    def _result_from_row(row: Mapping[str, Any], resolution: str) -> ManeuverQueryResult:
        availability = row["availability"]
        planner_safe = dict(row["planner_safe"]) if row.get("planner_safe") else {}
        reason = _CapabilityFamily._reason(row) if availability != "AVAILABLE" else None
        return ManeuverQueryResult(
            availability=availability, altitude_resolution=resolution, planner_safe=planner_safe,
            measured=row.get("measured"), evidence=row.get("evidence"), reason_unavailable=reason,
        )

    # -- public query contract (interface_contract in the artifact) --

    def altitude_interval_checkpoints(self, low_m: float, high_m: float) -> Tuple[float, ...]:
        """Query locations covering row boundaries and interpolation gaps.

        Supported family policies are linear, constant conservative endpoint,
        or unavailable between rows. Include blocked-range boundaries and an
        interior point of every partition, so checking only two available
        exact rows cannot accidentally authorize their unavailable interior.
        This exposes resolution boundaries, not raw measured capability data.
        """
        if not math.isfinite(low_m) or not math.isfinite(high_m) or low_m > high_m:
            raise ValueError("altitude interval must be finite and ordered")
        boundaries = {low_m, high_m}
        boundaries.update(z for z in self.manifest.altitude_grid_m if low_m < z < high_m)
        for family in self._families.values():
            for lo, hi in family.blocked_ranges:
                boundaries.update(z for z in (lo, hi) if low_m < z < high_m)
        ordered = sorted(boundaries)
        return tuple(sorted(boundaries | {(a + b) / 2.0 for a, b in zip(ordered, ordered[1:])}))

    def straight_query(self, altitude_m: float) -> ManeuverQueryResult:
        return self._resolve("straight", altitude_m, {})

    def turn_query(self, altitude_m: float, direction: str, bank_deg: float) -> ManeuverQueryResult:
        """bank_deg is the UNSIGNED magnitude (e.g. 20.0); direction ("LEFT"/
        "RIGHT") is the sole source of sign/handedness -- callers never need
        to know the artifact's own signed command_bank_deg convention."""
        if direction not in ("LEFT", "RIGHT"):
            raise AircraftProfileQueryError(f"direction must be LEFT or RIGHT, got {direction!r}")
        family = self._families["level_turn"]
        # Filter on direction + |command_bank_deg| -- the row's own sign convention
        # is internal; we match by direction and magnitude only.
        candidates = [r for r in family.rows if r.get("direction") == direction
                      and math.isclose(abs(float(r.get("command_bank_deg", math.nan))), bank_deg, abs_tol=_ALTITUDE_TOL)]
        if not candidates:
            raise AircraftProfileQueryError(
                f"level_turn: no direction={direction!r} bank_deg={bank_deg} rows at all -- "
                f"this bank magnitude was never characterized for this profile"
            )
        signed_bank = candidates[0]["command_bank_deg"]
        return self._resolve("level_turn", altitude_m, {"direction": direction, "command_bank_deg": signed_bank})

    def vertical_query(self, altitude_m: float, mode: str) -> ManeuverQueryResult:
        family_name = self._vertical_family(mode)
        return self._resolve(family_name, altitude_m, {})

    def combined_query(self, altitude_m: float, direction: str, mode: str) -> ManeuverQueryResult:
        if direction not in ("LEFT", "RIGHT"):
            raise AircraftProfileQueryError(f"direction must be LEFT or RIGHT, got {direction!r}")
        family_name = self._combined_family(mode)
        return self._resolve(family_name, altitude_m, {"direction": direction})

    @staticmethod
    def _vertical_family(mode: str) -> str:
        if mode == "CLIMB":
            return "straight_climb"
        if mode == "DESCENT":
            return "straight_descent"
        raise AircraftProfileQueryError(f"mode must be CLIMB or DESCENT, got {mode!r}")

    @staticmethod
    def _combined_family(mode: str) -> str:
        if mode == "CLIMB":
            return "climbing_turn"
        if mode == "DESCENT":
            return "descending_turn"
        raise AircraftProfileQueryError(f"mode must be CLIMB or DESCENT, got {mode!r}")


# --- Loading / validation --------------------------------------------------

def _require(data: Mapping, key: str, where: str = "profile") -> Any:
    if key not in data:
        raise LutSchemaError(f"{where} is missing required field {key!r}")
    return data[key]


def _require_str(data: Mapping, key: str, where: str = "profile") -> str:
    value = _require(data, key, where)
    if not isinstance(value, str) or not value:
        raise LutSchemaError(f"{where}: field {key!r} must be a non-empty string, got {value!r}")
    return value


def _require_bool(data: Mapping, key: str, where: str = "profile") -> bool:
    value = _require(data, key, where)
    if not isinstance(value, bool):
        raise LutSchemaError(f"{where}: field {key!r} must be a bool, got {value!r}")
    return value


def _require_number_tuple(data: Mapping, key: str, where: str = "profile") -> Tuple[float, ...]:
    value = _require(data, key, where)
    if not isinstance(value, (list, tuple)) or not value:
        raise LutSchemaError(f"{where}: field {key!r} must be a non-empty list of numbers, got {value!r}")
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
        raise LutSchemaError(f"{where}: field {key!r} must contain only numbers, got {value!r}")
    return tuple(float(v) for v in value)


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


def _build_manifest(data: Mapping) -> AircraftProfileManifest:
    schema_version = _require(data, "schema_version")
    if not isinstance(schema_version, int) or isinstance(schema_version, bool):
        raise LutSchemaError(f"'schema_version' must be an int, got {schema_version!r}")
    profile_schema_id = _require_str(data, "profile_schema_id")
    profile_stage = _require_str(data, "profile_stage")
    planner_ready = _require_bool(data, "planner_ready")

    identity = _require(data, "aircraft_identity")
    if not isinstance(identity, Mapping):
        raise LutSchemaError("'aircraft_identity' must be an object")
    aircraft_id = _require_str(identity, "aircraft_id", where="aircraft_identity")
    controller_stack_id = _require_str(identity, "controller_stack_id", where="aircraft_identity")

    provenance_id = _require_str(data, "provenance_id")

    domain = _require(data, "domain")
    if not isinstance(domain, Mapping):
        raise LutSchemaError("'domain' must be an object")
    altitude_grid = _require_number_tuple(domain, "canonical_altitude_grid_m", where="domain")
    if list(altitude_grid) != sorted(altitude_grid):
        raise LutSchemaError("domain.canonical_altitude_grid_m must be sorted ascending")
    min_alt = domain.get("min_altitude_m")
    max_alt = domain.get("max_altitude_m")
    if not isinstance(min_alt, (int, float)) or not isinstance(max_alt, (int, float)):
        raise LutSchemaError("domain.min_altitude_m/max_altitude_m must be numbers")
    if abs(min_alt - altitude_grid[0]) > _ALTITUDE_TOL or abs(max_alt - altitude_grid[-1]) > _ALTITUDE_TOL:
        raise LutSchemaError("domain min/max_altitude_m must equal the canonical grid's own endpoints")
    nominal_ias = domain.get("nominal_ias_context_mps")
    if not isinstance(nominal_ias, (int, float)) or isinstance(nominal_ias, bool):
        raise LutSchemaError("domain.nominal_ias_context_mps must be a number")

    return AircraftProfileManifest(
        schema_version=schema_version,
        profile_schema_id=profile_schema_id,
        profile_stage=profile_stage,
        planner_ready=planner_ready,
        aircraft_id=aircraft_id,
        controller_stack_id=controller_stack_id,
        provenance_id=provenance_id,
        units=_validate_units(data),
        altitude_grid_m=altitude_grid,
        min_altitude_m=float(min_alt),
        max_altitude_m=float(max_alt),
        nominal_ias_context_mps=float(nominal_ias),
    )


def _build_families(data: Mapping, manifest: AircraftProfileManifest) -> Dict[str, _CapabilityFamily]:
    capabilities = _require(data, "capabilities")
    if not isinstance(capabilities, Mapping):
        raise LutSchemaError("'capabilities' must be an object")
    missing = [f for f in REQUIRED_CAPABILITY_FAMILIES if f not in capabilities]
    if missing:
        raise LutSchemaError(f"'capabilities' is missing required families: {missing}")
    return {
        name: _CapabilityFamily(name, capabilities[name], manifest.altitude_grid_m)
        for name in REQUIRED_CAPABILITY_FAMILIES
    }


def load_aircraft_profile_from_dict(
    data: Mapping,
    *,
    expected_aircraft_id: str = EXPECTED_AIRCRAFT_ID,
    expected_controller_stack_id: str = EXPECTED_CONTROLLER_STACK_ID,
) -> AircraftProfile:
    """Validate a planner-safe capability profile dict and build the query
    interface. Fail-fast, in this order: presence/type of schema fields,
    planner_ready/profile_stage (a RAW artifact is refused here before any
    further check even runs), schema_version/profile_schema_id, aircraft_id/
    controller_stack_id, units, domain, then per-family row contents. Never
    returns a partially-valid AircraftProfile.

    expected_aircraft_id/expected_controller_stack_id default to THIS
    build's frozen c172p identity but are freely overridable -- this is
    the whole aircraft-swappability story: there is no code path here that
    only works for c172p."""
    # Checked via plain dict access, BEFORE any other schema requirement --
    # a RAW characterization artifact (e.g. jsbsim/results/
    # c172p_core_aircraft_lut_raw.json) doesn't even HAVE a profile_stage
    # field, so demanding it exist first (via _build_manifest) would reject
    # RAW artifacts with a generic "missing field" schema error instead of
    # the specific, informative not-planner-safe one. Fails closed: a
    # missing/non-bool planner_ready, or any profile_stage other than the
    # one recognized value, is refused.
    planner_ready = data.get("planner_ready")
    profile_stage = data.get("profile_stage")
    if planner_ready is not True or profile_stage != EXPECTED_PROFILE_STAGE:
        raise LutNotPlannerSafeError(
            f"refusing to load a non-planner-safe artifact (planner_ready={planner_ready!r}, "
            f"profile_stage={profile_stage!r}, artifact_type={data.get('artifact_type')!r}, "
            f"provenance_id={data.get('provenance_id')!r}) -- only planner_ready=true, "
            f"profile_stage={EXPECTED_PROFILE_STAGE!r} artifacts may be used by the planner. A RAW "
            f"characterization artifact must go through validation/interpolation/derating first "
            f"(see module docstring)."
        )

    manifest = _build_manifest(data)
    if manifest.schema_version != SUPPORTED_SCHEMA_VERSION:
        raise LutSchemaError(
            f"unsupported schema_version={manifest.schema_version} (this loader only supports "
            f"{SUPPORTED_SCHEMA_VERSION})"
        )
    if manifest.profile_schema_id != EXPECTED_PROFILE_SCHEMA_ID:
        raise LutSchemaError(
            f"unsupported profile_schema_id={manifest.profile_schema_id!r} (this loader only supports "
            f"{EXPECTED_PROFILE_SCHEMA_ID!r})"
        )
    if manifest.aircraft_id != expected_aircraft_id:
        raise LutProvenanceError(
            f"profile aircraft_id={manifest.aircraft_id!r} does not match expected {expected_aircraft_id!r}"
        )
    if manifest.controller_stack_id != expected_controller_stack_id:
        raise LutProvenanceError(
            f"profile controller_stack_id={manifest.controller_stack_id!r} does not match expected "
            f"{expected_controller_stack_id!r} -- a profile built under a different controller stack "
            f"cannot be assumed to describe the same flight envelope"
        )

    families = _build_families(data, manifest)
    return AircraftProfile(manifest, families)


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
