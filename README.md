# UAV Pathfinder

Terrain-aware 3D path-planning research code for a fixed-wing UAV. The authoritative
production planner uses a continuous fixed-wing pose-aware A* with conservative
terrain/AGL checks and an authoritative **Generic Constant-Performance Fixed-Wing Kinematic Model**:

- **Horizontal Kinematic Speed**: 40.0 m/s
- **Max Climb Rate**: +5.0 m/s (at all altitudes)
- **Max Descent Rate**: -5.0 m/s (at all altitudes)
- **Bank Angle**: 25.0 deg (Turn radius ≈ 349.89 m, Turn rate ≈ 6.55 deg/s)
- **Zero Wind** convention
- **Same limits at all altitudes** (no altitude-dependent degradation or LUT requirement)

## Repository layout

- `planner/` — production planning code (`planner.fixed_wing_envelope`, `planner.pose_search`, `planner.terrain_following`, etc.).
- `tests/` — small canonical regression suite.
- `scripts/` — current data preparation and benchmark runners.
- `jsbsim/` — historical reference aircraft characterization data.
- `working_dem/` — working terrain rasters used by the planner.
- `results/` — current planner benchmark evidence.
- `outputs/terrain_cache/` — reproducible terrain cache used by the real-terrain benchmark.
- `project.md` — current architecture, decisions, status, and blockers.
- `docs/` — design and architecture documents.

## Quick validation

From the repository root:

```powershell
python -m unittest discover -v
```

This runs fast synthetic and current-contract tests.

The full 27-scenario behavior coverage suite (M1–M7, B1–B6, V1–V5, F1–F3, Canonical A–F) is run via:

```powershell
python experiments/run_coverage_validation_suite.py
```
