# STEP U6.1 — C172P Holdout / Interpolation Validation

- Holdout: 62 point / 186 cold-start run (`core=46`, `combined=16`).
- Straight interpolation: çok düşük hata; throttle MAE `0.000054`, max `0.000151`.
- Level-turn radius: `±20°` direction-separated MAE yaklaşık `0.22–0.24 m`; `−30° @ 1250 m` nedeniyle severity interpolation `LIMITED`.
- Vertical actual-Vz: climb MAE `0.0008 m/s`, descent MAE `0.0002 m/s`; status yine interpolate edilmedi.
- Combined interpolation: continuous metrics güçlü; climb radius MAE `3.51 m`, descent radius MAE `1.44 m`, fakat high-alt categorical capability güvenilir biçimde interpolate edilemiyor.
- 4500 m climb: iki yön strict `INFEASIBLE`, full-throttle, fakat actual-stable `MARGINAL`.
- 5250 m climb: iki yön strict `INFEASIBLE`; sol `UNUSABLE`, sağ actual-stable `MARGINAL`.
- 1000 m left-climb radius: ground-track course-change wrap hatasına bağlı measurement outlier; physical left/right asymmetry değil.
- Unsupported region: straight climb `+2` için 4000–5500 ve combined climbing turn için 4000–5500 m categorical/capability kullanımı.
- U5/U5.1/U5.2/U6A/U6B source artifact'ları değişmedi.

## 1. Frozen stack

Yalnız `c172p @ nominal 40 m/s IAS`, current domain `0–5500 m` kullanıldı.
U6A/U6B ile aynı stock `FCS: c172`, frozen IAS/Vz/bank/beta outer-loop,
production mixture/propulsion policy, mass/fuel/CG, atmosphere, initialization,
`0.01 s` timestep ve measurement contract korundu. Controller tuning, aircraft
XML veya propulsion değişikliği yapılmadı.

## 2. Source artifacts

Prediction source'ları yalnız immutable
`results/c172p_core_aircraft_lut_raw.json` ve
`results/c172p_combined_3d_raw.json` anchor'larıdır. Prediction'lar holdout
execution başlamadan önce aynı target/direction için komşu altitude anchor'lar
arasında deterministik linear interpolation ile donduruldu. U5–U6B provenance,
result ve ilgili raw artifact hash'leri execution öncesi/sonrası aynıdır.
U6.1 generation provenance ID: `u6.1-73c7d2c4ba52ca611cc0`.

## 3. Holdout design

Core unseen midpoint altitude'ları `250/1250/2250/3250/4250/5250 m`'dir.
Her birinde straight; `±20°` level turn; ayrıca 1250/3250/5250 m'de `±30°`
turn çalıştırıldı. Vertical set 250–3250 m'de `−3/−2/+2/+3 m/s`, 4250/5250
m'de `−3/−2/+2 m/s`'dir. Böylece severe high-alt `+3` boundary araması ve full
250 m grid sweep yapılmadı.

Combined unseen midpoint'ler `1750/3250/4500/5250 m`; her birinde
`bank ±20° × Vz ±2.5 m/s` çalıştırıldı. Her point üç cold-start'tır. Toplam
`6 straight + 18 turn + 22 vertical + 16 combined = 62 point / 186 run`.

## 4. Straight validation

Straight continuous metrics altitude arasında lineer ve son derece düzgün
davrandı. Altı holdout'ta IAS MAE/max `0.00010/0.00013 m/s`, TAS
`0.00537/0.00670 m/s`, throttle `0.000054/0.000151`, pitch
`0.000065/0.000082°`, AoA `0.000053/0.000070°` oldu. Throttle relative error
mean `0.007%`, p90 `0.014%`'tür. Tüm straight holdout'lar repeatable'dır ve
topology discontinuity yoktur. Sonuç: **straight 0–5500 m continuous metrics
için INTERPOLATION_SUPPORTED**.

## 5. Turn interpolation validation

Left/right ayrı interpolate edildi; prediction turn-rate işareti bütün 18
holdout'ta doğru kaldı. `−20°` radius MAE `0.216 m`, max `0.284 m`, mean relative
error `%0.039`; `+20°` için MAE `0.244 m`, max `0.334 m`, mean `%0.039`'dur.
Turn-rate toplam MAE `0.0005°/s`, max `0.0015°/s`'dir. Bu evidence
**direction-separated ±20° level-turn interpolation'ını destekler**.

`±30°` yalnız üç altitude'da örneklendi. Sağ `+30°` ve 3250/5250 m sol sonuçlar
yakın olsa da `−30° @ 1250 m` prediction `353.8 m`, actual measured `442.8 m`
çıktı (`89.0 m`, `%20.10` error); aynı satırda turn-rate prediction çok yakındır.
Bu, mevcut `±30°` raw non-monotonic radius/extraction davranışıyla tutarlıdır.
Dolayısıyla sampled `±30°` geometry **LIMITED**; direction'ları ortalamak
asimetrileri saklayacağı için yasak kalmalıdır.

## 6. Vertical interpolation validation

Status categorical tutuldu; yalnız actual Vz, IAS, throttle, gamma ve pitch
interpolate edildi. Actual-Vz error:

| Target m/s | Count | MAE m/s | Max m/s | Mean relative % | Holdout status |
|---:|---:|---:|---:|---:|---|
| −3 | 6 | 0.00029 | 0.00147 | 0.009 | 6 UNKNOWN |
| −2 | 6 | 0.00019 | 0.00072 | 0.009 | 6 VALID |
| +2 | 6 | 0.00060 | 0.00231 | 0.029 | 3 VALID, 2 UNKNOWN, 1 INFEASIBLE |
| +3 | 4 | 0.00105 | 0.00301 | 0.033 | 4 UNKNOWN |

Bütün 22 vertical holdout actual-stable/repeatable'dır. Continuous actual-Vz
interpolation çok güçlüdür; ancak `+2` aynı düşük error'a rağmen farklı strict
status'lar üretir. Bu nedenle `−2` continuous descent supported, `−3` ve
low/mid `+2/+3` limited; high-alt climb capability/status ise discrete ve
conservative ele alınmalıdır.

## 7. Combined interpolation validation

Combined descending turn sekiz holdout'ta actual-stable'dır. Radius MAE/median/
max `1.44/1.53/2.37 m`, mean/p90 relative error `%0.243/%0.383`; actual-Vz MAE
`0.0005 m/s`, IAS MAE `0.0020 m/s`, throttle MAE `0.0006`'dır. Geometry ve
continuous response için **INTERPOLATION_SUPPORTED**, fakat anchor strict
status'ları numeric olarak interpolate edilmez.

Combined climbing turn continuous prediction da güçlüdür: radius
MAE/median/max `3.51/1.47/15.58 m`, mean/p90 relative `%0.644/%1.537`; Vz MAE
`0.0035 m/s`, IAS MAE `0.0411 m/s`, throttle MAE `0.0108`'dir. Bu radius summary
1000 m raw outlier yerine yalnız validation için audited corrected endpoint'i
kullanır. Buna karşın actual-stable/usability değişimleri continuous error'dan
türetilemez. Bu yüzden combined climb 1000–4000 `LIMITED`, 4000–5500
`UNSUPPORTED` capability interpolation'ıdır.

## 8. 4500 m critical result

4000→5000 anchor aralığındaki 4500 m climb iki yönde strict `INFEASIBLE`,
throttle `1.0`, actual IAS sol/sağ `38.74/39.02 m/s`, actual Vz
`2.547/2.534 m/s`; ikisi de repeatable ve actual-stable olduğundan raw
interpretation `MARGINAL`'dir. Continuous prediction çok yakındır: Vz error
yaklaşık `0.000/0.006 m/s`, IAS error `0.011/0.083 m/s`.

Bu sonuç U6B'deki 4000 `UNUSABLE` → 5000 `MARGINAL` categorical sıçramanın
continuous physics'te büyük bir ters trend olmadığını gösterir; 4000 m sol
controller/saturation acceptance kırılması lokaldir. Fakat status ve
actual-stable boundary yine lineer interpolate edilemez.

## 9. 5250 m critical result

5000→5500 aralığında 5250 m climb iki yönde strict `INFEASIBLE`, full-throttle
ve actual Vz `2.295/2.306 m/s`'dir. Sol yön settling/actual-stable contract'ı
geçmez ve `UNUSABLE`; sağ yön hâlâ actual-stable `MARGINAL`'dir. Prediction
continuous metrics'te çok yakındır: IAS error yaklaşık `0.008/0.003 m/s`, Vz
error `<0.001 m/s`; ancak yönlere göre stability transition farklı noktada
oluşur. Bu, high-alt combined climb için conservative/discrete treatment
gerektiğini doğrudan doğrular.

## 10. 1000 m outlier diagnostic

Mevcut üç U5.2/U6B repeat'in trajectory summary'si birebir aynıdır; ek
diagnostic run gerekmedi. Heading unwrap `−104.08°`, turn-rate `−5.170°/s` ile
doğru left-turn işaretini korurken stored ground-track change `+247.277°`
dalında kalmıştır. Bunun eşdeğer wrapped değeri `−112.723°`'dir. Stored radius
`194.82 m`; aynı arc length ve wrapped course change ile audited radius
`427.36 m` olur. Theory-relative difference `%57.75`'ten `%7.31`'e düşer.

Sonuç: **MEASUREMENT OUTLIER; gerçek left/right dynamic asymmetry değil.** U6B
raw row silinmedi veya değiştirilmedi. Raw radius interpolation'dan çıkarıldı;
yalnız validation prediction'ında audited wrapped-course radius kullanıldı ve
metadata ile işaretlendi.

## 11. Error distributions

| Dataset/family | Priority variable | Count | MAE | Median | Max | Mean relative % | P90 relative % |
|---|---|---:|---:|---:|---:|---:|---:|
| Core straight | throttle | 6 | 0.000054 | 0.000037 | 0.000151 | 0.007 | 0.014 |
| Core level turn | radius m | 18 | 5.152 | 0.213 | 89.003 | 1.159 | 0.077 |
| Core climb | actual Vz m/s | 10 | 0.0008 | 0.0003 | 0.0030 | 0.030 | 0.095 |
| Core descent | actual Vz m/s | 12 | 0.0002 | 0.0001 | 0.0015 | 0.009 | 0.031 |
| Combined climb | radius m | 8 | 3.510 | 1.468 | 15.576 | 0.644 | 1.537 |
| Combined climb | actual Vz m/s | 8 | 0.0035 | 0.0029 | 0.0095 | 0.133 | 0.279 |
| Combined climb | IAS m/s | 8 | 0.0411 | 0.0096 | 0.1442 | 0.104 | 0.258 |
| Combined climb | throttle | 8 | 0.0108 | 0.0059 | 0.0294 | 1.169 | 2.619 |
| Combined descent | radius m | 8 | 1.440 | 1.528 | 2.373 | 0.243 | 0.383 |
| Combined descent | actual Vz m/s | 8 | 0.0005 | 0.0005 | 0.0008 | 0.018 | 0.029 |

Core turn radius dağılımında tek `−30° @ 1250 m` outlier max/mean'i büyütür;
median ve p90 kalan noktaların sıkı dağılımını gösterir. Bu stage hiçbir yüzdeyi
planner safety threshold'üne çevirmedi.

## 12. Interpolation suitability map

| Maneuver / region | Label | Treatment |
|---|---|---|
| Straight continuous metrics, 0–5500 | INTERPOLATION_SUPPORTED | Linear altitude interpolation candidate |
| Level turn `±20°`, 0–5500 | INTERPOLATION_SUPPORTED | Left/right ayrı tutulmalı |
| Level turn sampled `±30°` | LIMITED | Raw radius diagnostics/conservative geometry gerekli |
| Straight descent `−2`, 0–5500 | INTERPOLATION_SUPPORTED | Status yine categorical |
| Straight descent `−3`, 0–5500 | LIMITED | Continuous iyi, acceptance değişken |
| Straight climb `+2`, 0–4000 | LIMITED | Continuous iyi, status interpolate edilmez |
| Straight climb `+2`, 4000–5500 | UNSUPPORTED | Power/status transition discrete ele alınmalı |
| Straight climb `+3`, sampled ≤3250 | LIMITED | Boundary-adjacent/non-monotonic raw response |
| Combined descending turn, 1000–5500 | INTERPOLATION_SUPPORTED | Direction-separated continuous interpolation |
| Combined climbing turn, 1000–2500 | LIMITED | Audited outlier endpoint gerekli |
| Combined climbing turn, 2500–4000 | LIMITED | Direction/stability transition yaklaşıyor |
| Combined climbing turn, 4000–5500 | UNSUPPORTED | Conservative/discrete capability treatment |

Soruların açık cevapları: straight metrics lineer interpolate edilebilir;
`±20°` radius yaklaşık desimetre ölçeğinde doğru tahmin edilir; left/right ayrı
tutulmalıdır; vertical actual-Vz MAE climb/descent için `0.0008/0.0002 m/s`'dir;
4000→5000 categorical non-monotonicity continuous response'ta büyük ters trend
değil acceptance boundary'dir; 4500 iki yön marginal actual-stable, 5250 sol
unusable/sağ marginal'dir; descending-turn interpolation stabildir; 1000 m sol
radius course-wrap measurement outlier'ıdır. Unsupported bölgeler high-alt
straight/combined climb capability; burada interpolation yerine conservative /
discrete treatment gerekir.

## 13. Artifacts/tests

- `results/u6_1_core_holdout_results.json`
- `results/u6_1_combined_holdout_results.json`
- `results/u6_1_interpolation_error_summary.json`
- `results/u6_1_interpolation_suitability.json`
- `results/u6_1_outlier_diagnostics.json`
- `results/u6_1_new_points.json`, `results/u6_1_new_runs.json`
- `results/u6_1_provenance.json`, `results/u6_1_result.json`
- `u6_1_holdout_interpolation_configuration.yaml`
- `u6_1_holdout_interpolation_validation.py`
- `test_u6_1_holdout_interpolation_validation.py`

Test; exact unseen grids/counts, prediction-before-run metadata, adjacent anchor
bracketing, left/right separation, continuous error fields/distributions,
categorical status preservation, critical 4500/5250 sonuçları, outlier audit /
raw immutability, suitability labels, 3 cold-start ve U5–U6B hash immutability
şartlarını doğrular. U5–U6.1 targeted artifact testleri PASS'tir.

## 14. project.md update

Kalıcı kayda U6.1 objective ve holdout gridleri, error summary, turn/vertical /
combined findings, 4500/5250 critical sonuçları, outlier root cause,
supported/limited/unsupported map, henüz derating yapılmadığı ve next stage
`U6.2 — Robustness + Planner-Safe Derating` olduğu eklendi. Raw telemetry
kopyalanmadı.

## 15. Blockers

U6.1 için blocker yoktur. Core ve combined interpolation genel olarak kısmi
doğrulandı: continuous metrics güçlü, fakat high-altitude climb categorical
capability interpolation desteklenmiyor. Planner-safe limits/safety factors bu
stage'de üretilmedi; U6.2 başlatılmadı.

STEP U6.1: PASS

CORE INTERPOLATION VALIDATED:
PARTIAL

COMBINED INTERPOLATION VALIDATED:
PARTIAL

OUTLIER RESOLVED:
YES

READY FOR U6.2 PLANNER-SAFE DERATING:
YES
