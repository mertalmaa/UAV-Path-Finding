# J3.1 — 0–6000 m MSL Domain Extension

## Sonuç

**J3.1 PASS.** Reusable prototype aircraft-characterization domain'i
`[0, 6000] m MSL` olarak genişletildi. Önceki 1500–6000 m J3 dataset'i yeniden
çalıştırılmadan korundu. Birleşik main-grid değerlendirmesinde **305 KCAS** yine
en dengeli aday oldu ve nominal CAS değişmedi.

- J3.1 provenance: `j3.1-7761df15481027018aac`
- Reuse edilen J3 provenance: `j3-7d701f785aae566708fc`
- Yeni main grid: 0, 500, 1000 m MSL
- Combined main grid: 0–6000 m, 500 m spacing, 13 altitude
- Yeni holdout: 250, 750, 1250 m MSL
- Yeni ölçüm: 45 main run + 9 holdout run
- Sonuç: 54/54 yeni run `VALID`; 18/18 yeni nokta `VALID`

## Zero-MSL airborne test fixture

JSBSim'in default terrain elevation'ı 0 m olduğundan, aircraft'i tam 0 m MSL'de
default koşulla başlatmak 0 AGL ground contact yaratıp trim'i `UNKNOWN` yapar.
Atmospheric altitude'u değiştirmeden bu yapay bağlantıyı kaldırmak için yalnız
J3.1 test fixture'ında terrain elevation `−1000 m MSL` olarak sabitlendi. Böylece
0 m MSL noktası 1000 m AGL'de, fakat standard-atmosphere hesabında hâlâ tam
0 m MSL'de ölçüldü. Bu planner terrain varsayımı veya operational-envelope
iddiası değildir; ayar provenance ve run diagnostics içinde kayıtlıdır.

## Yeni 0/500/1000 m candidate sonuçları

| Altitude | CAS | Status | Actual KCAS | TAS m/s | Mach | AoA | Throttle-pos | Max surface |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 275 | VALID | 274.930 | 141.440 | 0.416 | 2.579° | 0.394 | 0.0492 |
| 0 | 305 | VALID | 304.916 | 156.867 | 0.461 | 1.795° | 0.434 | 0.0472 |
| 0 | 335 | VALID | 334.902 | 172.295 | 0.506 | 1.210° | 0.471 | 0.0459 |
| 0 | 365 | VALID | 364.888 | 187.724 | 0.552 | 0.763° | 0.505 | 0.0451 |
| 0 | 395 | VALID | 394.874 | 203.152 | 0.597 | 0.414° | 0.537 | 0.0445 |
| 500 | 275 | VALID | 274.928 | 144.712 | 0.428 | 2.589° | 0.408 | 0.0492 |
| 500 | 305 | VALID | 304.915 | 160.454 | 0.474 | 1.805° | 0.447 | 0.0472 |
| 500 | 335 | VALID | 334.901 | 176.183 | 0.521 | 1.220° | 0.483 | 0.0459 |
| 500 | 365 | VALID | 364.886 | 191.900 | 0.567 | 0.773° | 0.516 | 0.0451 |
| 500 | 395 | VALID | 394.870 | 207.603 | 0.614 | 0.424° | 0.549 | 0.0443 |
| 1000 | 275 | VALID | 274.927 | 148.089 | 0.440 | 2.600° | 0.421 | 0.0493 |
| 1000 | 305 | VALID | 304.913 | 164.150 | 0.488 | 1.816° | 0.460 | 0.0472 |
| 1000 | 335 | VALID | 334.899 | 180.186 | 0.536 | 1.231° | 0.495 | 0.0460 |
| 1000 | 365 | VALID | 364.884 | 196.196 | 0.583 | 0.784° | 0.528 | 0.0451 |
| 1000 | 395 | VALID | 394.867 | 212.177 | 0.631 | 0.435° | 0.562 | 0.0440 |

Yeni main-grid run'larında maksimum CAS tracking error `%0.0634`, maksimum gamma
error `0.0441°`, maksimum altitude drift `2.458 m` ve maksimum speed drift
`0.215 KCAS` oldu. Bütün değerler J1 measurement acceptance sınırları içindedir.

## Birleşik 0–6000 m candidate comparison

| CAS | V/U/I | Mach range | Throttle range | Worst abs(AoA) | Max surface | Throttle reserve | Low-speed margin | Transonic margin | Min score |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 275 | 13/0/0 | 0.416–0.595 | 0.394–0.543 | 2.763° | 0.0497 | 0.394 | 0.0996 | 0.3385 | 0.0996 |
| **305** | **13/0/0** | **0.461–0.657** | **0.434–0.588** | **1.983°** | **0.0475** | **0.412** | **0.2196** | **0.2697** | **0.2196** |
| 335 | 13/0/0 | 0.506–0.718 | 0.471–0.627 | 1.399° | 0.0460 | 0.373 | 0.3395 | 0.2017 | 0.2017 |
| 365 | 13/0/0 | 0.552–0.779 | 0.505–0.662 | 0.950° | 0.0451 | 0.338 | 0.4594 | 0.1346 | 0.1346 |
| 395 | 13/0/0 | 0.597–0.838 | 0.537–0.694 | 0.586° | 0.0475 | 0.306 | 0.5793 | 0.0684 | 0.0684 |

`V/U/I`, `VALID/UNKNOWN/INFEASIBLE` main-altitude sayısıdır. J3'te belgelenen
formül, component'ler ve tie-breaker sırası değiştirilmedi. 305 KCAS'in minimum
skoru `0.2196`; sonraki aday 335 KCAS'in skoru `0.2017` oldu. Bu skor safety
derating değildir.

## 250/750/1250 m holdout validation

Holdout'lar candidate selection veya fitting girdisi yapılmadı.

| Holdout | Status | Actual KCAS | Mach | AoA | Throttle | CAS err | Gamma err | Alt drift | Holdout min score | Endpoint floor | Refine? |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 m | VALID | 304.915 | 0.4675 | 1.7998° | 0.4404 | 0.0501% | 0.0254° | 1.189 m | 0.2196615 | 0.2196585 | Hayır |
| 750 m | VALID | 304.914 | 0.4810 | 1.8101° | 0.4532 | 0.0512% | 0.0262° | 1.251 m | 0.2196554 | 0.2196522 | Hayır |
| 1250 m | VALID | 304.912 | 0.4950 | 1.8211° | 0.4662 | 0.0523% | 0.0271° | 1.317 m | 0.2196490 | 0.2196457 | Hayır |

Üç holdout da level trim, tracking ve margin kapılarını geçti. Her holdout'un
mevcut J3 formülüyle minimum marjı, komşu main-grid endpoint'lerinin daha kötü
olanından düşük değildir. Beklenmeyen discontinuity veya non-conservative
davranış saptanmadı; **local 250 m refinement gereken band yoktur**.

## Scope ve J4 gate

J3.1 yalnız domain extension yaptı. Climb/descent, turn, coupled turn, macro
primitive, lookup, planner integration, speed-state ve derating çalıştırılmadı.
J4 öncesi blocker yoktur; J4 başlatılmamıştır.
