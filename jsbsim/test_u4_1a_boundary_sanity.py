from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"


def _load(name: str):
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    config = yaml.safe_load(
        (HERE / "u4_1a_boundary_sanity_configuration.yaml").read_text(
            encoding="utf-8"
        )
    )
    artifact = _load("u4_1a_boundary_sanity.json")
    provenance = _load("u4_1a_provenance.json")
    runs_artifact = _load("u4_1a_runs.json")
    points_artifact = _load("u4_1a_points.json")
    result = _load("u4_1a_result.json")
    raw_path = RESULTS / "aircraft_lut_raw.json"
    raw = _load("aircraft_lut_raw.json")
    pid = result["provenance_id"]

    assert config["representative_altitudes_m"] == [0, 1000, 2000]
    assert config["include_optional_2500_m_existing_edge_audit"] is True
    assert config["turn"]["outward_sequence_deg"] == [30.0, 35.0]
    assert config["turn"]["optional_single_point_deg"] == 40.0
    assert config["vertical"]["climb_outward_sequence_mps"] == [6.0, 7.0]
    assert config["vertical"]["descent_outward_sequence_mps"] == [-6.0, -7.0]
    assert config["scope"]["boundary_sanity_only"] is True
    assert all(
        value is False
        for key, value in config["scope"].items()
        if key != "boundary_sanity_only"
    )

    assert pid == artifact["provenance_id"] == provenance["provenance_id"]
    assert pid == runs_artifact["provenance_id"] == points_artifact["provenance_id"]
    assert artifact["parent_u4_provenance_id"] == raw["provenance_id"]
    assert config["parent_u4_provenance_id"] == raw["provenance_id"]
    assert _sha256(raw_path) == config["parent_raw_lut_sha256"]
    assert artifact["parent_raw_lut_sha256_before"] == _sha256(raw_path)
    assert artifact["parent_raw_lut_sha256_after"] == _sha256(raw_path)
    assert artifact["parent_raw_lut_unchanged"] is True
    for item in provenance["files"].values():
        path = Path(item["path"])
        assert path.is_file()
        assert _sha256(path) == item["sha256"]

    turn_edges = artifact["existing_turn_edges"]
    vertical_edges = artifact["existing_vertical_edges"]
    turn_probes = artifact["new_turn_probes"]
    vertical_probes = artifact["new_vertical_probes"]
    assert len(turn_edges) == 8
    assert len(vertical_edges) == 8
    assert {row["altitude_m"] for row in turn_edges} == {0.0, 1000.0, 2000.0, 2500.0}
    assert {row["altitude_m"] for row in vertical_edges} == {0.0, 1000.0, 2000.0, 2500.0}
    assert all(row["source"] == "U4_EXISTING_RAW_LUT" for row in turn_edges + vertical_edges)

    eligible_turns = {
        (row["altitude_m"], row["direction"])
        for row in turn_edges
        if row["status"] == "VALID"
    }
    probed_turns = {
        (row["altitude_m"], row["direction"]) for row in turn_probes
    }
    assert probed_turns == eligible_turns == {(1000.0, "RIGHT"), (2000.0, "RIGHT")}
    assert all(
        next(
            edge
            for edge in turn_edges
            if edge["altitude_m"] == probe["altitude_m"]
            and edge["direction"] == probe["direction"]
        )["status"]
        == "VALID"
        for probe in turn_probes
    )
    for altitude, direction in eligible_turns:
        probes = [
            row
            for row in turn_probes
            if row["altitude_m"] == altitude and row["direction"] == direction
        ]
        sign = -1.0 if direction == "LEFT" else 1.0
        targets = [row["target_bank_deg"] for row in probes]
        allowed = [sign * 30.0, sign * 35.0, sign * 40.0]
        assert targets == allowed[: len(targets)]
        non_valid = [index for index, row in enumerate(probes) if row["status"] != "VALID"]
        if non_valid:
            assert non_valid == [len(probes) - 1]
        if sign * 35.0 in targets:
            at_35 = probes[targets.index(sign * 35.0)]
            assert (sign * 40.0 in targets) == (at_35["status"] == "VALID")

    eligible_vertical = {
        (row["altitude_m"], "CLIMB" if row["target_vz_mps"] > 0 else "DESCENT")
        for row in vertical_edges
        if row["status"] == "VALID"
    }
    probed_vertical = {
        (row["altitude_m"], "CLIMB" if row["target_vz_mps"] > 0 else "DESCENT")
        for row in vertical_probes
    }
    assert eligible_vertical == probed_vertical == set()

    runs = runs_artifact["runs"]
    points = points_artifact["points"]
    assert result["new_turn_probe_count"] == len(turn_probes) == 3
    assert result["new_vertical_probe_count"] == len(vertical_probes) == 0
    assert result["executed_points"] == len(points) == 3
    assert result["executed_cold_start_runs"] == len(runs) == 3 * len(points) == 9
    assert all(len(point["run_ids"]) == 3 for point in points)
    assert all(row["repeatability"]["passed"] for row in turn_probes + vertical_probes)
    assert all(run["provenance_id"] == pid for run in runs)
    assert all(point["provenance_id"] == pid for point in points)

    assert len(artifact["altitude_results"]) == 4
    assert artifact["assessment"]["turn_grid_edge"] == "POSSIBLY ARTIFICIAL"
    assert artifact["assessment"]["turn_right_grid_edge"] == "POSSIBLY ARTIFICIAL"
    assert artifact["assessment"]["turn_left_grid_edge"] == "INCONCLUSIVE"
    assert artifact["assessment"]["vertical_grid_edge"] == "INCONCLUSIVE"
    assert artifact["assessment"]["climb_grid_edge"] == "NOT ARTIFICIAL"
    assert artifact["assessment"]["descent_grid_edge"] == "INCONCLUSIVE"
    assert artifact["assessment"]["highest_tested_valid_bank_magnitude_deg"] == 30.0
    assert artifact["assessment"]["highest_tested_valid_climb_mps"] == 2.0
    assert artifact["assessment"]["largest_tested_valid_descent_magnitude_mps"] == 2.0
    assert artifact["assessment"]["true_physical_maximum_extracted"] is False
    assert artifact["assessment"]["planner_safe_envelope_selected"] is False

    assert result["step"] == "U4.1A"
    assert result["step_status"] == "PASS"
    assert result["raw_u4_lut_unchanged"] is True
    assert result["ready_for_u4_1b_holdout_validation"] is True
    assert result["controller_tuning_performed"] is False
    assert result["stock_xml_modified"] is False
    assert result["planner_modified"] is False
    assert result["interpolation_performed"] is False
    assert result["derating_performed"] is False
    assert result["holdout_validation_started"] is False
    assert result["true_physical_maximum_extracted"] is False
    print("U4.1A boundary-sanity artifact and contract checks passed")


if __name__ == "__main__":
    main()
