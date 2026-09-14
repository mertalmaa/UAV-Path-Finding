# Historical architecture summary

Git history contains the detailed stage reports and raw intermediate outputs.
This file retains only decisions that help explain the current design.

## Altitude representation

The first planner used a global regular `z_step_m` lattice and fixed
climb/descent deltas. CandidateZ began as a sparse terrain-floor/ceiling event
generator and later became the production altitude-successor source when its
generator is supplied to search. Off-lattice mission altitudes are encoded as
stable integer keys, and explicit-target primitives preserve the requested
physical altitude without snapping.

The regular-Z search path still exists as a compatibility path when no
CandidateZ generator is provided. The independent coarse planner also retains
its own dense representation. Removing those paths is production work and was
intentionally not attempted during repository hygiene cleanup.

## Search state and experimental cost terms

Earlier experiments added trend/bucket/reversal history and dominance logic to
the search state. They were removed after showing that physical `(row, col,
altitude)` identity was sufficient and the extra state multiplied equivalent
nodes. Historical tuning CSVs and one-off comparison scripts are no longer kept
in the working tree.

Weighted A*, ARA*, coarse guides, and hard-corridor experiments established
useful performance behavior, but their numerous per-stage outputs were not
canonical artifacts. The production modules remain in `planner/`; old run
outputs and stage-specific harnesses were removed.

## Grid decision

Terrain-only 30/60/90 m comparisons were later re-gated with measured aircraft
turn geometry. The retained decision is 60 m as the default planning scale, 30
m for local refinement, and 90 m only for global/optional metadata. Grid
resolution is representation; it is not aircraft turn radius or motion length.

## Aircraft characterization

The early F-16 methodology and C172R/U4 investigations were superseded by a
stock C172P at nominal 40 m/s IAS. The current planner consumes the tested-safe
V3 profile. It keeps measured evidence separate from planner-safe availability,
uses altitude-dependent climb/descent capability, keeps left/right turns
separate, and rejects raw characterization artifacts at the runtime loader.

## Removed interfaces

The abandoned web/UI layer and its launcher configuration were removed. The
repository is now centered on the planner library, offline characterization,
canonical tests, and explicit benchmark runners.
