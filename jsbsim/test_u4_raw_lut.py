from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"


def _load(name: str):
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def main() -> None:
    config = yaml.safe_load((HERE / "u4_raw_lut_configuration.yaml").read_text(encoding="utf-8"))
    provenance = _load("u4_provenance.json")
    result = _load("u4_result.json")
    lut = _load("aircraft_lut_raw.json")
    runs_artifact = _load("u4_runs.json")
    points_artifact = _load("u4_points.json")
    traces_artifact = _load("u4_traces.json")
    pid = result["provenance_id"]

    assert config["main_altitude_grid_m"] == list(range(0, 5001, 500))
    assert config["optional_straight_probe_altitudes_m"] == [5500, 6000]
    assert config["turn_bank_grid_deg"] == [0, -10, 10, -15, 15, -20, 20, -25, 25]
    assert config["vertical_speed_grid_mps"] == [-5, -4, -3, -2, 0, 2, 3, 4, 5]
    assert config["aircraft"]["model"] == "c172r"
    assert config["aircraft"]["nominal_ias_mps"] == 40.0
    assert config["future_holdout_plan"]["execute_in_u4"] is False

    assert pid == provenance["provenance_id"] == lut["provenance_id"]
    assert pid == runs_artifact["provenance_id"] == points_artifact["provenance_id"]
    assert pid == traces_artifact["provenance_id"]
    assert provenance["parent_u3_provenance_id"] == config["parent_provenance_id"]
    for item in provenance["files"].values():
        path = Path(item["path"])
        assert path.is_file()
        assert hashlib.sha256(path.read_bytes()).hexdigest() == item["sha256"]
    assert all(
        "missing_reference" not in item and hashlib.sha256(Path(item["path"]).read_bytes()).hexdigest() == item["sha256"]
        for item in provenance["aircraft_dependency_closure"]
    )

    runs = runs_artifact["runs"]
    points = points_artifact["points"]
    assert result["executed_points"] == len(points)
    assert result["executed_cold_start_runs"] == len(runs) == 3 * len(points)
    assert len(lut["straight_gates"]) == 11
    assert len(lut["optional_straight_probes"]) == 2
    assert len(lut["turn_lut"]) == result["turn_lut_rows"]
    assert len(lut["vertical_lut"]) == result["vertical_lut_rows"]
    assert result["turn_lut_rows"] == 9 * len(result["straight_valid_altitudes_m"])
    assert result["vertical_lut_rows"] == 9 * len(result["straight_valid_altitudes_m"])

    valid_altitudes = {
        row["altitude_m"] for row in lut["straight_gates"] if row["status"] == "VALID"
    }
    assert valid_altitudes == set(result["straight_valid_altitudes_m"])
    assert {row["altitude_m"] for row in lut["turn_lut"]} == valid_altitudes
    assert {row["altitude_m"] for row in lut["vertical_lut"]} == valid_altitudes
    assert not ({5500.0, 6000.0} & {row["altitude_m"] for row in lut["turn_lut"]})
    assert not ({5500.0, 6000.0} & {row["altitude_m"] for row in lut["vertical_lut"]})
    assert not (set(config["future_holdout_plan"]["altitude_grid_m"]) & valid_altitudes)

    statuses = set(config["status_contract"]["statuses"])
    failure_classes = set(config["status_contract"]["failure_classes"])
    all_rows = lut["straight_gates"] + lut["optional_straight_probes"] + lut["turn_lut"] + lut["vertical_lut"]
    assert all(row["status"] in statuses for row in all_rows)
    assert all(row["failure_class"] is None or row["failure_class"] in failure_classes for row in all_rows)
    assert all(row["repeatability"]["passed"] for row in all_rows)
    assert all(row["settling"]["all_repeats_settled_before_measurement"] for row in all_rows if row["status"] == "VALID")
    assert all(len(row["run_ids"]) == 3 for row in all_rows)
    assert all(
        math.isfinite(value)
        for row in all_rows
        for value in row.values()
        if isinstance(value, float)
    )

    for altitude in valid_altitudes:
        turns = [row for row in lut["turn_lut"] if row["altitude_m"] == altitude]
        vertical = [row for row in lut["vertical_lut"] if row["altitude_m"] == altitude]
        assert {row["target_bank_deg"] for row in turns} == set(config["turn_bank_grid_deg"])
        assert {row["target_vz_mps"] for row in vertical} == set(config["vertical_speed_grid_mps"])
        assert sum(row["direction"] == "LEFT" for row in turns) == 4
        assert sum(row["direction"] == "RIGHT" for row in turns) == 4
        assert sum(row["direction"] == "STRAIGHT" for row in turns) == 1
        assert all(row["measured_radius_m"] > 0 for row in turns if row["target_bank_deg"] != 0)
        assert all(row["theoretical_radius_m"] > 0 for row in turns if row["target_bank_deg"] != 0)
        zero_turn = next(row for row in turns if row["target_bank_deg"] == 0)
        zero_vertical = next(row for row in vertical if row["target_vz_mps"] == 0)
        gate = next(row for row in lut["straight_gates"] if row["altitude_m"] == altitude)
        assert zero_turn["zero_bank_reuses_straight_gate_runs"] is True
        assert zero_vertical["zero_vz_reuses_straight_gate_runs"] is True
        assert zero_turn["run_ids"] == zero_vertical["run_ids"] == gate["run_ids"]

    for point in points:
        metrics = point["repeatability"]["metrics"]
        assert {"ias_mps", "roll_deg", "vertical_speed_mps", "engine_0_throttle_pos_norm"} <= set(metrics)
        if point["maneuver"] == "turn":
            assert {"turn_rate_deg_s", "measured_radius_m"} <= set(metrics)
    assert all(
        sample["provenance_id"] == pid and math.isfinite(sample["mach"])
        for sample in traces_artifact["samples"]
    )

    assert lut["interpolated"] is False
    assert lut["derated"] is False
    assert lut["planner_ready"] is False
    assert lut["true_maximum_claimed"] is False
    assert result["controller_tuning_performed"] is False
    assert result["stock_xml_modified"] is False
    assert result["planner_modified"] is False
    assert result["interpolation_performed"] is False
    assert result["derating_performed"] is False
    assert result["holdouts_executed"] is False
    assert result["final_replay_implemented"] is False
    assert result["step_status"] == "PASS"
    assert result["raw_lut_ready_for_holdout_validation"] is True
    print("U4 raw LUT artifact and contract checks passed")


if __name__ == "__main__":
    main()

