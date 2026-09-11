# J2 — JSBSim / F-16 provenance and baseline sanity

Durum: **PASS — J2 complete, J3 not started**

## Scope

Bu aşama yalnız provenance, reference configuration, level-flight simulator
sanity ve cold-start repeatability doğrulamasıdır. Nominal CAS sweep, climb veya
descent characterization, turn testi, lookup üretimi, derating ve planner
entegrasyonu yapılmamıştır.

## Provenance

- Provenance ID: `j2-c25dabb783f4ac1ffdef`
- JSBSim Python package: `1.3.1`
- JSBSim compiled build: `1.3.1`, GitHub build `1837`
- JSBSim commit: `3b25f25e49b42d0489c04ac805674fc1450ca579`
- Build timestamp: `May 17 2026 14:28:34`
- Python: `3.14.7`, CPython 64-bit, MSC v.1944
- Timestep: `1/120 s = 0.008333333333333333 s`

FCS, F-16 aircraft XML'inin içinde tanımlıdır. Yüklenen diğer model bağımlılıkları
F100 engine, direct thruster ve F-16'ya ait pushback/hook system dosyalarıdır.
Tam absolute path ve SHA-256 kayıtları `results/j2_provenance.json` içindedir.

| Dosya rolü | SHA-256 |
|---|---|
| F-16 aircraft/FCS model | `78edd90534338204913c35ed50d7c222eecac1b83c454c73ad70bf6d4907ea2f` |
| F-16 reset example | `738c7d0f3495b072bcdb570c203c92cbd7ae5af3c4213f01183a46dd861a0c85` |
| F100-PW-229 engine | `92fd101fdcc60dc8f41f449a297e134419ff062f5293c2e58678fd07d8127e69` |
| direct thruster | `f116b6f6d25d4b2b46c2f0faa713e85d0064db7a634105bc96e391fb52e4dde5` |
| pushback system | `a3c0e2fc8a494b18b7158718cfc96d51fb34aac440965c0c73a1b7f33bb1a64b` |
| hook system | `9067ea104820d9d2d2f438b0361cbfdc9cea28a7969bd46eaa540fd7d9dfdd62` |
| JSBSim native Python binary | `4ab248200b605f3b5c31f6bedea734840a459caa96df11eb72fd4426e985a177` |
| J2 reference configuration | `7f781ea0b32b028b648d39bde81acb7b66029a891472ab3b28befb1dd01c9444` |
| J2 baseline harness | `d3ec410e45ea851e120ca16348f5a8457bcb4e3f1cda576ebc35ea9ed9bfe08f` |

## Reference configuration

Bu değerler gerçek F-16 operational specification değildir; frozen prototype
characterization context'idir.

- Loaded model ID: installed JSBSim `f16`
- XML FDM config name: `General Dynamics F-16A`
- XML dependency declarations: engine `F100-PW-229`, thruster `direct`, systems
  `pushback/hook`, inline flight control `F-16 FC`
- Empty weight: `17,400 lb`
- Pilot point mass: `230 lb`
- Internal fuel: `1,500 + 1,500 = 3,000 lb`
- External tank fuel contents: `0 + 0 lb`
- Total frozen weight: `20,630 lb = 9,357.610593 kg`
- Model-computed CG: `(-191.891711, 0, -3.574406) in`, JSBSim structural frame
- Fuel burn: frozen during J2 replay
- Gear: command `0`, actual position `0` — up
- Speedbrake: command `0`, actual position `0` — closed
- External stores: none
- Native FCS: active; `fcs/fbw-override = 0`
- Afterburner: prohibited; measured throttle-position norm remained `< 1.0`
- Atmosphere: JSBSim US Standard, zero temperature deviation
- Wind/gust: zero on all NED axes
- Turbulence type: `0` — disabled

## Units and conventions

- Requested altitude: metres MSL; JSBSim input conversion to `ic/h-sl-ft`
- Requested speed: KCAS
- Logged TAS: m/s
- Angles: degrees
- Body angular rates: degrees/s
- Gamma: positive climb, negative descent
- Wind components: local NED, ft/s
- CG: JSBSim structural frame, inches

## Sanity speed source

Installed JSBSim data contains F-16 ground/runway scripts but no airborne F-16
trim example. J2 therefore uses `335 KCAS` only as a sanity seed, based on an
upstream JSBSim F-16 trim reproduction. It is not a selected nominal speed and
not an aircraft limit. Nominal CAS selection remains J3 scope.

Upstream context: <https://github.com/JSBSim-Team/jsbsim/discussions/814>

## Level-flight sanity results

Every altitude was run three times using a fresh `FGFDMExec` instance:

```text
fresh instance -> load f16 -> set fixed configuration -> RunIC
-> engine running -> initialized frame -> full trim -> 30 s replay
-> final 20 s measurement window
```

| Requested altitude | Statuses | Mean actual altitude | Mean KCAS | Max CAS error | Mean TAS | Mach | Mean gamma | Max gamma error | Mean throttle pos. | Max control usage |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1500 m | 3× `VALID` | 1501.025 m | 334.897 | 0.0573% | 184.308 m/s | 0.5510 | 0.0267° | 0.0337° | 0.5078 | 0.0460 |
| 5000 m | 3× `VALID` | 5001.426 m | 334.884 | 0.0647% | 216.796 m/s | 0.6763 | 0.0317° | 0.0401° | 0.5982 | 0.0424 |
| 6000 m | 3× `VALID` | 6001.596 m | 334.875 | 0.0701% | 227.349 m/s | 0.7184 | 0.0340° | 0.0432° | 0.6272 | 0.0402 |

Maximum altitude drift over a 20 s measurement window was `2.695 m` at 6000 m.
All cases satisfied the frozen CAS ±2%, gamma ±0.5° and control-surface ≤90%
measurement acceptance rules. Settling was detected from the first replay
sample (`0.00833 s`) in every case because replay begins from full trim.

Engine N1/N2 remained positive, stalled/seized flags remained false, all
outputs were finite, fuel/weight/CG remained fixed, gear and speedbrake did not
move, FCS override remained zero, and wind/turbulence remained disabled.

## Cold-start repeatability

Each altitude's three independent runs produced identical classification and
effectively deterministic key metrics. Maximum relative deviation from the
three-run mean was `1.65e-16`, far below the approximate ±1% J1 criterion.

- Total runs: `9`
- `VALID`: `9`
- `INFEASIBLE`: `0`
- `UNKNOWN`: `0`
- Trim success: `9/9`

No infeasible flight condition was deliberately tested in J2. The classifier
was separately checked:

- Reliable accepted measurement -> `VALID`
- Trim/setup/numerical failure without physical evidence -> `UNKNOWN`
- Reliable confirmed unsustainable condition -> `INFEASIBLE`

Thus trim failure is not automatically mapped to `INFEASIBLE`.

## Artifacts

- `j2_reference_configuration.yaml`: frozen reference context
- `j2_baseline_sanity.py`: executable sanity harness
- `test_j2_baseline.py`: scope/config/status semantic tests
- `results/j2_provenance.json`: full provenance and hashes
- `results/j2_runs.json`: per-run requested/actual diagnostics
- `results/j2_summary.json`: gate and repeatability summary

## Gate conclusion

**J2 PASS.** Provenance is complete, model dependencies and configuration are
verified, all representative level-flight cases trim and replay, three-run
repeatability passes, logging/schema works, and UNKNOWN/INFEASIBLE semantics
are distinct. No J2 blocker prevents a future J3 start; J3 was not started.
