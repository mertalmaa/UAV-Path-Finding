from __future__ import annotations

import json
import math
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"


def main() -> None:
    config = yaml.safe_load((HERE / "u3_aircraft_selection_configuration.yaml").read_text(encoding="utf-8"))
    result = json.loads((RESULTS / "u3_result.json").read_text(encoding="utf-8"))
    audit = json.loads((RESULTS / "u3_control_audit.json").read_text(encoding="utf-8"))
    points = json.loads((RESULTS / "u3_points.json").read_text(encoding="utf-8"))
    runs = json.loads((RESULTS / "u3_runs.json").read_text(encoding="utf-8"))
    inventory = json.loads((RESULTS / "u2_inventory.json").read_text(encoding="utf-8"))

    assert config["selection_policy"]["preferred_values_are_hard_gates"] is False
    assert config["scope"]["reuse_u2_inventory"] is True
    assert len(config["candidates"]) <= 6
    assert result["inventory_count"] == len(inventory) == 60
    assert result["serious_candidate_count"] == len(config["candidates"])
    assert len(audit) == len(config["candidates"])
    assert len(points) == 4 * len(config["candidates"])
    assert len(runs) == 3 * len(points)
    assert result["executed_points"] == len(points)
    assert result["executed_cold_start_runs"] == len(runs)
    assert result["full_lut_started"] is False
    assert result["final_replay_implemented"] is False
    assert result["planner_modified"] is False
    assert result["aircraft_xml_modified"] is False
    assert result["controller_gain_tuning_performed"] is False
    assert result["same_stack_lut_and_final_replay_required"] is True
    assert {row["model"] for row in audit} == set(config["candidates"])
    assert all(row["dependency_closure"] for row in audit)
    assert all("missing_reference" not in item for row in audit for item in row["dependency_closure"])
    assert all(point["repeatability"]["passed"] for point in points)
    assert all(point["measured"] is not None for point in points)
    assert all(
        math.isfinite(value)
        for point in points
        for value in point["measured"].values()
        if isinstance(value, (float, int))
    )
    assert result["step_status"] in {"PASS", "PARTIAL"}
    if result["step_status"] == "PASS":
        assert result["primary"] and result["backup"] and result["selected_stack"]
        assert any(row["metrics"]["all_four_logical_tests_clean"] for row in result["ranking"])
    assert [row["rank"] for row in result["ranking"]] == list(range(1, len(config["candidates"]) + 1))
    assert all(set(row["scores"].values()).issubset(set(range(6))) for row in result["ranking"])
    print("U3 artifact and contract checks passed")


if __name__ == "__main__":
    main()

