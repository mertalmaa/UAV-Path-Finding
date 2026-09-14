# JSBSim C172P characterization

This directory contains the offline aircraft-characterization pipeline used to
produce the planner's tested-safe C172P profile. JSBSim is never executed during
A* node expansion.

## Current data flow

```text
stock C172P + frozen controller/mixture policy
  -> core and combined raw measurements
  -> independent holdout/interpolation audit
  -> planner-safe derivation
  -> altitude-dependent profile
  -> tested-safe V3 envelope
```

The online planner consumes only:

`results/c172p_aircraft_profile_planner_safe_v3.json`

Raw LUTs are evidence and regeneration inputs, not runtime-safe profiles. The
production loader in `planner/aircraft_profile.py` rejects them.

## Current pipeline families

- `u6a_*` — C172P core raw LUT.
- `u6b_*` — combined 3D characterization.
- `u6_1_*` — holdout/interpolation validation.
- `u6_2_*`, `u6_2_1_*`, `u6_2_2_*` — planner-safe profile derivation through V3.
- `production_mixture_policy.py` and `planner_safe_measurements.py` — frozen
  shared policy/helpers.

Earlier U1–U5 components and artifacts remain only where the current U6
pipeline imports or verifies them. Superseded reports are summarized in
`../docs/HISTORY.md` and retained in Git history.

Run the repository-level quick contract suite from the repository root:

```powershell
python -m unittest discover -v
```

That suite validates V3 loading, raw-profile rejection, altitude-dependent
capability, and planner integration without running JSBSim simulations.
