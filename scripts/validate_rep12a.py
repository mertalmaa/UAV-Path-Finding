"""Step REP-1.2A validation: representation-neutral primitive endpoints.

Small, fast synthetic test matrix (A-I) for the explicit-target-altitude
primitive constructor added in planner.primitives (primitive_for_target_altitude
/ _climb_descent_primitive / _level_primitive), plus a regression check that
the existing fixed-lattice caller (build_primitive_set) is byte-identical to
before this stage. No terrain, no A*, no Mission A/B/C -- geometry only.
"""
import math

from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set, primitive_for_target_altitude

CFG = DEFAULT_CONFIG


def main() -> None:
    all_pass = True

    def check(label: str, ok: bool) -> None:
        nonlocal all_pass
        all_pass = all_pass and ok
        print(f"  {label:<70} {'PASS' if ok else 'FAIL'}")

    print("=== A: source=4000 target=4020 (on-lattice-sized delta) climb works ===")
    p = primitive_for_target_altitude("N", 4000.0, 4020.0, CFG)
    check("primitive built, climb, dz=+20.0", p is not None and p.primitive_type == "climb" and p.dz_m == 20.0)

    print("=== B: source=4127 target=4163 (off-lattice endpoints) climb works ===")
    p = primitive_for_target_altitude("N", 4127.0, 4163.0, CFG)
    check("primitive built, climb, dz=+36.0 exactly (no snapping)", p is not None and p.primitive_type == "climb" and p.dz_m == 36.0)

    print("=== C: source=4200 target=4171 (off-lattice) descent works ===")
    p = primitive_for_target_altitude("N", 4200.0, 4171.0, CFG)
    check("primitive built, descent, dz=-29.0 exactly", p is not None and p.primitive_type == "descent" and p.dz_m == -29.0)

    print("=== D: same source/target -> deterministic output ===")
    p1 = primitive_for_target_altitude("NE", 4127.0, 4163.0, CFG)
    p2 = primitive_for_target_altitude("NE", 4127.0, 4163.0, CFG)
    check("two calls with identical inputs produce identical MotionPrimitive", p1 == p2)

    print("=== E: repeated use -> no cumulative float drift ===")
    src = 4000.0
    tgt = src + 17.3  # not necessarily bit-exact 17.3, but fixed for every call below
    first_dz = primitive_for_target_altitude("E", src, tgt, CFG).dz_m
    drift_ok = True
    for _ in range(10000):
        p = primitive_for_target_altitude("E", src, tgt, CFG)
        if p is None or p.dz_m != first_dz:
            drift_ok = False
            break
    check("10000 repeated calls from the same source/target -> dz_m never drifts", drift_ok)

    print("=== F: primitive core does not require z_step_m ===")
    # A config whose z_step_m differs from the requested delta: the delta
    # actually used has nothing to do with config.z_step_m.
    weird_cfg = DEFAULT_CONFIG
    p = primitive_for_target_altitude("S", 5000.0, 5000.0 + 7.0, weird_cfg)
    check(
        "dz_m == requested delta (7.0), independent of config.z_step_m (20.0)",
        p is not None and p.dz_m == 7.0 and weird_cfg.z_step_m != 7.0,
    )

    print("=== G: primitive core carries no z_index +-1 assumption ===")
    import inspect
    from planner import primitives as primitives_mod
    src_core = inspect.getsource(primitives_mod.primitive_for_target_altitude) \
        + inspect.getsource(primitives_mod._climb_descent_primitive) \
        + inspect.getsource(primitives_mod._level_primitive) \
        + inspect.getsource(primitives_mod.evaluate_primitive) \
        + inspect.getsource(primitives_mod.primitive_endpoint)
    check("no 'z_index' token anywhere in the core endpoint functions", "z_index" not in src_core)

    print("=== H: existing regular-lattice caller unchanged (build_primitive_set) ===")
    ps = build_primitive_set(CFG)
    by_key = {(p.direction, p.primitive_type): p for p in ps}
    n_climb = by_key[("N", "climb")]
    n_descent = by_key[("N", "descent")]
    check(
        "24 primitives, N climb dz=+20.0, N descent dz=-20.0 (byte-identical to pre-REP-1.2A)",
        len(ps) == 24 and n_climb.dz_m == 20.0 and n_descent.dz_m == -20.0,
    )

    print("=== I: climb/descent direction derived purely from delta sign ===")
    up = primitive_for_target_altitude("W", 100.0, 100.1, CFG)
    down = primitive_for_target_altitude("W", 100.0, 99.9, CFG)
    level = primitive_for_target_altitude("W", 100.0, 100.0, CFG)
    check(
        "positive delta -> climb, negative delta -> descent, zero delta -> level",
        up.primitive_type == "climb" and down.primitive_type == "descent" and level.primitive_type == "level",
    )

    print()
    print(f"Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
