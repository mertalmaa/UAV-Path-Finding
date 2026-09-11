# J4B — Straight Climb / Descent Capability Characterization

## Sonuç

**J4B PASS.** 0–6000 m MSL main grid boyunca, 305 KCAS ve afterburner-off
context'inde 13 climb ve 13 descent capability boundary güvenilir biçimde
bracketlendi. Değerler gerçek maksimum/minimum değil, son `VALID` ile ilk
`INFEASIBLE` target arasındaki **ham ölçüm bracket'larıdır**.

- Provenance: `j4b-9fa3129fabfcda51c7a7`
- Parent J4A provenance: `j4a-2342f25aec9f6ff3314c`
- 281 unique gamma point, 385 total run
- 52 boundary-critical point × 3 cold start
- 231 `VALID`, 50 `INFEASIBLE`, 0 `UNKNOWN` unique point
- 26/26 boundary bracketed
- Runtime: `43.50 s` (bu makinedeki final batch)

## Controller architecture audit

```text
target gamma
  -> PI gamma outer loop
  -> fcs/elevator-cmd-norm
  -> native F-16 pitch FCS
  -> elevator surface

target 305 KCAS
  -> PI CAS outer loop around level-trim throttle
  -> fcs/throttle-cmd-norm
  -> FCS throttle gain
  -> engine
```

- Gamma controller pilot-equivalent normalized elevator command üretir;
  doğrudan elevator surface position basmaz.
- CAS controller throttle'ı ayarlar; throttle capability girdisi değil ölçülen
  controller/engine output'udur.
- Native FCS aktiftir; `fcs/fbw-override = 0`.
- Elevator command sınırı `[-0.30, +0.30]`.
- Throttle command sınırı `[0, 0.5]`; FCS throttle-position karşılığı `[0,1]`.
- Afterburner kapalıdır. Physical control-surface acceptance sınırı `0.90`.
- Gamma integrator `±20 deg·s`, CAS integrator `±100 knot·s` ile bounded;
  command clamp ile birlikte windup protection sağlar.
- J4A mimarisi korundu. CAS gain/ramp ayarı capability sweep'ten önce controller
  lag'inin yanlış aircraft boundary üretmesini önlemek için yapıldı; bu ayarlar
  planner veya capability throttle policy değildir.

Tracking failure tek başına `INFEASIBLE` değildir. `INFEASIBLE/thrust_limited`
yalnız setup/engine/native-FCS/output sağlıklı, gamma tracking ve authority mevcut,
measurement throttle command ortalaması ilgili dry sınırına `0.01` içinde,
pencere sınırda bitmiş ve final CAS hâlâ ±%2 dışında ise kullanıldı. Diğer
controller/setup sorunları `UNKNOWN` kalır.

## Sweep strategy

Her altitude'da gamma `0°` control case ile başlandı. Climb ve descent yönlerinde
J4A'nın doğruladığı `2°` büyüklüğünden itibaren `2°` coarse step kullanıldı ve
ilk non-VALID noktada duruldu. `VALID -> INFEASIBLE` aralığı `0.5°` step ile
refine edildi. Son `VALID` ve ilk `INFEASIBLE` noktalar üç cold-start ile tekrarlandı.

`40°` yalnız runaway search'i önleyen test guard'ıdır; aircraft limiti değildir.
Hiçbir final boundary guard'a dayanmadı.

## Altitude × climb/descent capability

| Alt MSL | Highest climb VALID | Climb bracket | Lowest descent VALID | Descent bracket | İlk non-VALID sınıfı |
|---:|---:|---:|---:|---:|---|
| 0 m | +31.0° | (+31.0°, +31.5°) | −8.0° | (−8.5°, −8.0°) | thrust-limited |
| 500 m | +29.0° | (+29.0°, +29.5°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |
| 1000 m | +27.0° | (+27.0°, +27.5°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |
| 1500 m | +25.5° | (+25.5°, +26.0°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |
| 2000 m | +24.0° | (+24.0°, +24.5°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |
| 2500 m | +22.5° | (+22.5°, +23.0°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |
| 3000 m | +21.0° | (+21.0°, +21.5°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |
| 3500 m | +20.0° | (+20.0°, +20.5°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |
| 4000 m | +18.5° | (+18.5°, +19.0°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |
| 4500 m | +17.5° | (+17.5°, +18.0°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |
| 5000 m | +16.0° | (+16.0°, +16.5°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |
| 5500 m | +15.0° | (+15.0°, +15.5°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |
| 6000 m | +14.0° | (+14.0°, +14.5°) | −8.5° | (−9.0°, −8.5°) | thrust-limited |

Örneğin 6000 m climb sonucu “maksimum +14°” değildir; ölçülen gerçek boundary
`(+14.0°, +14.5°)` içindedir.

## Boundary VALID-point evidence ve margins

Hücreler `throttle-pos / relevant throttle margin / AoA / Mach / surface usage`
biçimindedir. Climb margin `1-throttle`; descent margin `throttle` değeridir.

| Alt | Climb VALID evidence | Descent VALID evidence |
|---:|---:|---:|
| 0 | .99210/.00790/1.288°/.4955/.06226 | .00000/.00000/1.702°/.4565/.04326 |
| 500 | .99183/.00817/1.359°/.5089/.06131 | .00000/.00000/1.672°/.4711/.04314 |
| 1000 | .99111/.00889/1.426°/.5228/.06039 | .00000/.00000/1.693°/.4836/.04308 |
| 1500 | .99349/.00651/1.484°/.5369/.05978 | .00000/.00000/1.713°/.4967/.04304 |
| 2000 | .99542/.00458/1.537°/.5517/.05918 | .00000/.00000/1.730°/.5104/.04299 |
| 2500 | .99643/.00357/1.586°/.5673/.05870 | .00000/.00000/1.746°/.5249/.04296 |
| 3000 | .99599/.00401/1.632°/.5832/.05833 | .00000/.00000/1.760°/.5400/.04294 |
| 3500 | 1.00000/.00000/1.682°/.5992/.05836 | .00000/.00000/1.771°/.5561/.04294 |
| 4000 | .99882/.00118/1.724°/.6165/.05785 | .00000/.00000/1.777°/.5732/.04296 |
| 4500 | 1.00000/.00000/1.788°/.6322/.05693 | .00000/.00000/1.784°/.5910/.04299 |
| 5000 | .99982/.00018/1.813°/.6521/.05519 | .00000/.00000/1.798°/.6089/.04286 |
| 5500 | 1.00000/.00000/1.868°/.6698/.05352 | .00000/.00000/1.817°/.6273/.04172 |
| 6000 | 1.00000/.00000/1.906°/.6899/.05180 | .00000/.00000/1.836°/.6464/.04045 |

Control-surface reserve, `0.90 - surface usage`, bütün boundary VALID noktalarında
`0.8377–0.8596` aralığındadır. İlk INFEASIBLE noktaların tamamında CAS dry
throttle sınırında tolerans dışına çıkarken gamma kanalı ve surfaces authority
korudu; bu nedenle failure classification bütün 26 sınırda `thrust_limited`dır.
Descent tarafındaki sınıf minimum-thrust/available-drag yönünü ifade eder.

## Repeatability, domain boundary ve scope

Boundary belirleyen 52 noktanın her biri üç cold-start ile doğrulandı. En büyük
boundary repeatability relative deviation `1.38e-16` ve bütün repeatability
kapıları PASS'tir. 0 m descent ve 6000 m climb replay'lerinin supported domain
dışına yönelmesi status'u etkilemedi; synthetic terrain yalnız airborne test
fixture olarak kaldı.

Turn, coupled maneuver, macro primitive, lookup generation, derating ve planner
integration yapılmadı. Speed planner state'ine eklenmedi. J5 öncesi blocker yok;
J5 başlatılmadı.

