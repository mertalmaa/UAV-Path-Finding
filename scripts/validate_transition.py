"""Stage validation: climb/descent transition feasibility (planner/transition.py).

Confirms evaluate_transition() computes the correct horizontal distance
(true Euclidean, not axis-aligned), the correct flight-path angle, and the
correct VALID/INVALID decision against config.max_climb_angle_deg /
config.max_descent_angle_deg -- both 10.0 deg TEST PARAMETERS, not real
aircraft performance limits (see planner/config.py).
"""
import math

from planner.config import DEFAULT_CONFIG
from planner.transition import evaluate_transition

Z0 = 1000.0  # arbitrary base altitude MSL for all test points


def main() -> None:
    cfg = DEFAULT_CONFIG
    assert cfg.max_climb_angle_deg == 10.0 and cfg.max_descent_angle_deg == 10.0, \
        "this validation assumes the TEST angle limits of 10.0 deg"

    # d_xy=300.0 is a deliberately chosen value: tan(radians(10)) * 300 fed
    # back through atan2/degrees lands on exactly 10.0 in double precision
    # (verified empirically), giving a bit-exact boundary case instead of
    # an off-by-a-few-ULPs one.
    boundary_d_xy = 300.0
    boundary_dz = boundary_d_xy * math.tan(math.radians(10.0))
    assert math.degrees(math.atan2(boundary_dz, boundary_d_xy)) == 10.0

    cases = [
        ("1. level, 30m", (0, 0, Z0), (30, 0, Z0), True, "ok"),
        ("2. climb 120m/+20m", (0, 0, Z0), (120, 0, Z0 + 20), True, "ok"),
        ("3. climb 30m/+20m", (0, 0, Z0), (30, 0, Z0 + 20), False, "exceeds_max_climb_angle"),
        ("4. descent 120m/-20m", (0, 0, Z0), (120, 0, Z0 - 20), True, "ok"),
        ("5. descent 30m/-20m", (0, 0, Z0), (30, 0, Z0 - 20), False, "exceeds_max_descent_angle"),
        ("6. climb exact 10deg", (0, 0, Z0), (boundary_d_xy, 0, Z0 + boundary_dz), True, "ok"),
        ("7. climb just >10deg", (0, 0, Z0), (boundary_d_xy, 0, Z0 + boundary_dz + 1.0), False, "exceeds_max_climb_angle"),
        ("8. descent exact 10deg", (0, 0, Z0), (boundary_d_xy, 0, Z0 - boundary_dz), True, "ok"),
        ("9. diagonal dx=dy=30", (0, 0, Z0), (30, 30, Z0), True, "ok"),
        ("10. same xy, dz=+50", (500, 500, Z0), (500, 500, Z0 + 50), False, "vertical_jump"),
    ]

    print(f"{'Case':<24}{'d_xy':>10}{'delta_z':>10}{'angle':>10}{'limit':>8}"
          f"{'expect':>9}{'actual':>9}  Result")
    print("-" * 100)

    all_pass = True
    for label, start, end, expect_valid, expect_reason in cases:
        result = evaluate_transition(start, end, cfg)
        ok = (result.valid == expect_valid) and (result.reason == expect_reason)
        all_pass = all_pass and ok

        limit = cfg.max_climb_angle_deg if result.transition_type != "descent" else cfg.max_descent_angle_deg
        expect_str = "VALID" if expect_valid else "INVALID"
        actual_str = "VALID" if result.valid else "INVALID"
        status = "PASS" if ok else f"FAIL (reason={result.reason}, expected={expect_reason})"

        print(f"{label:<24}{result.horizontal_distance_m:>10.3f}{result.delta_z_m:>10.3f}"
              f"{result.flight_path_angle_deg:>10.4f}{limit:>8.1f}"
              f"{expect_str:>9}{actual_str:>9}  {status}")

    # Explicit no-motion case: same (x,y,z) for both ends. Not part of the
    # numbered spec list but exercises the required distinct handling of
    # d_xy==0 and delta_z==0.
    print()
    result = evaluate_transition((100, 100, Z0), (100, 100, Z0), cfg)
    ok = result.valid and result.reason == "no_motion" and result.transition_type == "level"
    all_pass = all_pass and ok
    print(f"no-motion (same x,y,z) -> valid={result.valid}, reason={result.reason}, "
          f"type={result.transition_type}  {'PASS' if ok else 'FAIL'}")

    print()
    print(f"Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
