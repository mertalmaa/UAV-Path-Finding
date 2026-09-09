"""Stage 14 validation: first descent->climb reversal is free.

Direct state-machine checks on hand-built physical paths (reusing
_path_vertical_reversal_metrics / _classify_transition directly, same
pattern as Stage 12's validate_reversal_cost.py), plus a compute_edge_cost
check that the actual dollar cost of the free vs. penalized transitions
matches.
"""
from planner.astar import _classify_transition, _path_vertical_reversal_metrics, compute_edge_cost
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set

CFG = DEFAULT_CONFIG


def build_path(start, prim_sequence):
    path = [start]
    r, c, z = start
    for prim in prim_sequence:
        r, c = r + prim.drow, c + prim.dcol
        z = z + round(prim.dz_m / CFG.z_step_m)
        path.append((r, c, z))
    return path


def main() -> None:
    primitives = build_primitive_set(CFG)
    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    descent_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "descent")
    level_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "level")

    w = CFG.vertical_reversal_cost_weight
    print(f"vertical_reversal_cost_weight = {w}")
    print()

    # (label, sequence, expect_total, expect_free, expect_penalized, expect_penalty)
    cases = [
        ("DES,DES,DES", [descent_e, descent_e, descent_e], 0, 0, 0, 0.0),
        ("DES,LEVEL,CLIMB (first D->C via level)", [descent_e, level_e, climb_e], 1, 1, 0, 0.0),
        ("DES,CLIMB,DES (first D->C free, then C->D penalized)", [descent_e, climb_e, descent_e], 2, 1, 1, w * 20.0),
        ("DES,CLIMB,DES,CLIMB (free + 2 penalized)", [descent_e, climb_e, descent_e, climb_e], 3, 1, 2, w * 40.0),
        ("DES,DES,CLIMB,CLIMB (desired smooth profile)", [descent_e, descent_e, climb_e, climb_e], 1, 1, 0, 0.0),
    ]

    all_ok = True
    for label, seq, exp_total, exp_free, exp_pen, exp_penalty in cases:
        path = build_path((5, 2, 100), seq)
        m = _path_vertical_reversal_metrics(path, primitives, CFG)
        ok = (
            m["total_vertical_reversal_count"] == exp_total
            and m["free_descent_to_climb_count"] == exp_free
            and m["penalized_reversal_count"] == exp_pen
            and abs(m["total_reversal_penalty"] - exp_penalty) < 1e-9
        )
        all_ok = all_ok and ok
        print(f"  {label}")
        print(f"    total={m['total_vertical_reversal_count']} (expect {exp_total})  "
              f"free={m['free_descent_to_climb_count']} (expect {exp_free})  "
              f"penalized={m['penalized_reversal_count']} (expect {exp_pen})  "
              f"penalty={m['total_reversal_penalty']:.2f} (expect {exp_penalty:.2f})  "
              f"{'PASS' if ok else 'FAIL'}")

    print()
    print("=== compute_edge_cost() direct check: first D->C costs the same as continuing level ===")
    # descent from 2840, then the reversing climb: previous_trend=-1, free_used=False
    cost_free_climb = compute_edge_cost(climb_e, 2820.0, -1, False, CFG)
    cost_plain_climb_no_trend = compute_edge_cost(climb_e, 2820.0, 0, False, CFG)  # no standing trend at all
    cost_penalized_climb = compute_edge_cost(climb_e, 2820.0, -1, True, CFG)  # free already used
    cost_climb_to_descent = compute_edge_cost(descent_e, 2840.0, 1, True, CFG)  # climb -> descent, always penalized

    ok2 = (
        abs(cost_free_climb - cost_plain_climb_no_trend) < 1e-9  # free reversal costs exactly the base cost
        and cost_penalized_climb > cost_free_climb
        and abs((cost_penalized_climb - cost_free_climb) - w * 20.0) < 1e-9
        and cost_climb_to_descent > compute_edge_cost(descent_e, 2840.0, 0, True, CFG)
    )
    print(f"  first D->C (free): {cost_free_climb:.4f}   same primitive, no standing trend: {cost_plain_climb_no_trend:.4f}")
    print(f"  second D->C (penalized): {cost_penalized_climb:.4f}   extra over free = {cost_penalized_climb - cost_free_climb:.4f} (expect {w * 20.0:.4f})")
    print(f"  {'PASS' if ok2 else 'FAIL'}")
    all_ok = all_ok and ok2

    print()
    print(f"Overall: {'ALL PASS' if all_ok else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
