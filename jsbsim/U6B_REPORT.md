# STEP U6B — C172P Combined 3D Maneuver Validation

- Grid: 24 combined point; 12 U5.2 point reuse, 12 yeni point / 36 cold-start run.
- Climbing turn: 1000/2500 m `MARGINAL`, 4000 m `UNUSABLE`, 5000 m `MARGINAL`, 5500 m `UNUSABLE`.
- Descending turn: 1000/2500 m `MARGINAL`, 4000/5000/5500 m `USABLE`; primary 10/10 actual-stable.
- 5000 m: iki yön climb actual-stable fakat full throttle, strict `INFEASIBLE`; descent iki yönde `USABLE`.
- 5500 m: climb iki yönde full-throttle, IAS/Vz loss ve settling failure ile `UNUSABLE`; descent iki yönde `USABLE`.
- Major limitation: high-altitude climbing turn'de propulsion/throttle saturation, IAS/Vz retention ve 5500 m settling.
- U6A canonical LUT değişti mi: hayır; hash öncesi/sonrası aynıdır.

## 1. Frozen stack

Yalnız `c172p @ nominal 40 m/s IAS` kullanıldı. U6A'nın
`c172p-stock-fcs-frozen-u1-u3-outer-loop-production-mixture-v1` stack'i,
stock `FCS: c172`, production pressure-ratio mixture policy, aynı
mass/fuel/CG, US Standard sıfır-rüzgâr atmosferi, initialization, `0.01 s`
timestep ve measurement contract ile aynen korundu. PID/gain, aircraft XML,
mixture ve propulsion policy değiştirilmedi.

## 2. Test grid

Primary representative grid `1000/2500/4000/5000/5500 m × bank ±20° ×
Vz ±2.5 m/s` olarak 20 point'tir. 1000/4000/5500 m'deki 12 exact U5.2 c172p
combined point same-stack equivalence ile reuse edildi. 2500/5000 m primary
hücreleri için 8 yeni point çalıştırıldı.

1000 m primary setinin dört hücresi de actual-stable/repeatable ve climb
throttle `0.787–0.791` olduğundan yalnız bu altitude'da ikinci severity family
seçildi: `bank ±20° × Vz ±4 m/s`, 4 yeni point. `±30°` combined sweep ve başka
altitude'da severity testi yapılmadı. Toplam yeni çalışma 12 point / 36
cold-start run'dır; her point üç bağımsız cold-start'tır.

## 3. Climbing-turn results

| Altitude m | Interpretation | Actual-stable | Strict VALID | Actual Vz m/s | IAS m/s | Throttle |
|---:|---|---:|---:|---:|---:|---:|
| 1000 | MARGINAL | 2/2 | 1/2 | 2.61–2.69 | 40.04–40.06 | 0.787–0.791 |
| 2500 | MARGINAL | 2/2 | 1/2 | 2.62–2.69 | 40.08–40.10 | 0.842–0.847 |
| 4000 | UNUSABLE | 1/2 | 0/2 | 2.67–2.71 | 39.83–39.99 | 0.954–0.980 |
| 5000 | MARGINAL | 2/2 | 0/2 | 2.38 | 37.63–37.88 | 1.000 |
| 5500 | UNUSABLE | 0/2 | 0/2 | 2.21–2.23 | 36.53–36.77 | 1.000 |

1000/2500 m'de iki yön combined climb fiziksel olarak stable/repeatable'dır;
sol strict bank-tracking nedeniyle UNKNOWN kaldığından family MARGINAL'dir.
4000 m'de sağ yön actual-stable MARGINAL, sol yön strict UNKNOWN ve controller
saturation ile actual-stable contract dışıdır; family bu nedenle UNUSABLE'dır.
5000 m'de iki yön stable response verse de full throttle, yaklaşık `%5.3` IAS
loss ve strict INFEASIBLE sebebiyle yalnız MARGINAL raw evidence'tır. 5500 m'de
full throttle, yaklaşık `%8–9` IAS loss, Vz retention kaybı ve settling failure
iki yönü de UNUSABLE yapar. 5500 m sonucu stage failure değildir; doğrudan
combined climb capability bulgusudur.

1000 m `±4 m/s` severity climb'da sol/sağ actual `4.28/4.17 m/s`; ikisi de
stable/repeatable, sol strict UNKNOWN/MARGINAL, sağ strict VALID/USABLE'dır.
Bu sonuç daha yüksek irtifalara extrapolate edilmedi.

## 4. Descending-turn results

Primary `−2.5 m/s` descending turns bütün beş altitude ve iki yönde
actual-stable/repeatable (`10/10`) oldu. 1000 m'de iki strict UNKNOWN,
2500 m'de bir UNKNOWN bulunduğu için family MARGINAL; 4000/5000/5500 m'de iki
yön strict VALID ve USABLE'dır. Actual descent `−2.74…−2.82 m/s`, IAS
`39.82…40.00 m/s`; throttle irtifayla yaklaşık `0.37 → 0.65` yükselir ancak
saturation oluşmaz.

Turn geometry iyi korunur: primary descending radius ratio bütün gridde
`0.981–1.022`, yani U6A level-turn radius'una yaklaşık `%±2.2` içindedir.
Kalıcı left/right farkı U6A level-turn geometrisini izler; combined descent ek
bir yön bozulması üretmez. 1000 m `−4 m/s` severity actual `−4.29 m/s` ve exact
straight-reference retention ratio `1.030` olsa da IAS steady-window davranışı
nedeniyle iki yön actual-stable contract dışı/UNUSABLE'dır; bu yüzden primary
descent sonucu `−4 m/s`'ye genişletilmez.

## 5. Level-vs-combined radius

Primary descending turns yukarıdaki gibi level radius'u çok yakından korur.
Climbing radius ratio 2500 ve 4000 m'de `0.994–1.022`; 5000 m'de
`0.874–0.919`, 5500 m'de `0.816–0.868` olur. High-altitude climb'daki küçülme
iyileşme olarak yorumlanmaz: aynı hücrelerde IAS/Vz kaybı ve full-throttle
vardır, dolayısıyla hedeflenen nominal geometry korunmamaktadır.

1000 m sağ climb ratio `1.021` iken sol climb raw measured ratio `0.429`'dur.
Sol satırın theory-radius relative difference'i `0.577` ve strict reason'ı
`theory_radius_cross_check` içerir; bu açık bir measured-radius diagnostic
outlier/asymmetry'dir. Ham değer korunmuş, düzeltilmemiş ve planner-safe kabul
edilmemiştir. 1000 m `+4` severity de sol yönde benzer diagnostic outlier
(`radius_ratio=0.452`) gösterirken sağ yön `1.035`'tir.

## 6. Straight-vs-combined Vz

Primary hedef `±2.5 m/s`, U6A vertical grid ise `±2/±3 m/s` içerir. Exact
denominator olmadığı ve interpolation bu stage'de yasak olduğu için primary
`straight_vertical_reference_vz_mps` ile `vz_retention_ratio` bilinçli olarak
`null` bırakıldı; her row aynı altitude'daki iki bracketing U6A raw reference'ı
taşır. Bu boşluk uydurma/interpolated oranla doldurulmadı.

Primary direct target-achievement ratio yine ölçüldü. Descent büyüklüğü hedefin
yaklaşık `1.097–1.128` katıdır. Climb 1000–4000 m'de `1.043–1.084`, 5000 m'de
`0.953`, 5500 m'de `0.883–0.890` olur. Exact U6A `±4 m/s` denominator'ı bulunan
1000 m severity setinde Vz retention ratio climb için `0.987–1.013`, descent
için `1.030`'dur. Böylece exact straight-vs-combined retention yalnız uygun
denominator bulunan satırlarda raporlanmıştır.

## 7. Power/saturation trends

Combined descent'te power veya controller saturation temel sınır değildir.
Climb throttle 1000 m'de yaklaşık `0.79`, 2500 m'de `0.84–0.85`, 4000 m'de
`0.95–0.98`'e çıkar. 4000 m sol yön ilk controller-saturation/actual-stability
kırılmasıdır. 5000 ve 5500 m iki yönde throttle `1.0` ve strict INFEASIBLE'dır.
5000 m stable/repeatable response henüz vardır fakat marj yoktur; 5500 m'de IAS,
Vz ve settling birlikte bozulur. Ana high-altitude limitation controller tuning
değil propulsion/power boundary'dir.

## 8. Altitude trends

| Altitude m | Climbing turn | Climb radius ratio | Descending turn | Descent radius ratio | Ana yorum |
|---:|---|---:|---|---:|---|
| 1000 | MARGINAL | 0.429*–1.021 | MARGINAL | 0.987–1.022 | Güç marjı iyi; sol radius diagnostic outlier |
| 2500 | MARGINAL | 1.018–1.022 | MARGINAL | 0.983–1.012 | İki family actual-stable |
| 4000 | UNUSABLE | 0.994–1.021 | USABLE | 0.981–1.005 | Sol climb saturation/contract kırılması |
| 5000 | MARGINAL | 0.874–0.919 | USABLE | 0.981–1.004 | Climb full-throttle fakat actual-stable |
| 5500 | UNUSABLE | 0.816–0.868 | USABLE | 0.981–1.004 | Climb IAS/Vz/settling kaybı |

`*` Theory cross-check'i geçmeyen raw measured-radius outlier'ıdır.

## 9. Planner interpretation

Bu sınıflar yalnız representative RAW combined evidence'tır; henüz
planner-safe değildir. 1000/2500 m climbing turn MARGINAL, 4000 m UNUSABLE,
5000 m MARGINAL, 5500 m UNUSABLE'dır. Descending turn 1000/2500 m MARGINAL,
4000–5500 m USABLE'dır. UNKNOWN, INFEASIBLE'a çevrilmedi; actual-stable strict
non-VALID satırlar ayrı tutuldu.

Bir direct climbing turn'ün UNUSABLE/INFEASIBLE olması hedefin unreachable
olduğunu göstermez. İleride planner straight climb → level turn, turn → straight
climb veya daha uzun bir rota kullanabilir. **DIRECTLY INFEASIBLE !=
UNREACHABLE.** Independent core capability'lerin birleşimi de otomatik combined
capability sayılmaz.

## 10. Artifact/schema

Canonical U6A LUT değiştirilmedi. Ayrı
`results/c172p_combined_3d_raw.json`, 24 combined row taşır. Her satır target ve
actual IAS/bank/Vz, yön/family, strict status + usability + actual-stable,
turn-rate/measured radius, exact level reference ve radius ratio, exact vertical
reference varsa Vz retention ratio, yoksa raw brackets, gamma/pitch/AoA/beta/Nz,
propulsion, surfaces/controller, saturation, settling/repeatability, failure ve
reuse metadata içerir. Theory radius yalnız diagnostic alanıdır. Artifact
`planner_ready=false`, `full_envelope_claim=false`, `interpolated=false`,
`derated=false`, `holdout_validated=false`'dır.

## 11. Tests

Validation script exact five-altitude primary grid'i, tek 1000 m severity
family'sini, 24 unique canonical key'i, c172p/40 m/s/≤5500 scope'u, iki yön ve
iki combined family'yi, üç cold-start contract'ını, üç raw status'u,
measured/reference radius oranlarını, exact-denominator Vz policy'sini, 12-point
reuse + 12-point/36-run yeni inventory'yi, altitude classifications ve bütün
U5/U5.2/U6A source hash'lerinin değişmediğini doğrular. U5, U5.1, U5.2, U6A ve
U6B targeted artifact testleri PASS'tir.

## 12. project.md update

Kalıcı kayda U6B amacı, `1000/2500/4000/5000/5500 m` test grid'i,
climb/descent trendleri, level-vs-combined radius, exact-denominator Vz retention
policy'si, high-altitude combined climb limitation, `DIRECTLY INFEASIBLE !=
UNREACHABLE` ve next stage `U6.1 — Holdout / Interpolation Validation` eklendi.
Raw telemetry kopyalanmadı.

## 13. Blockers

U6B için blocker yoktur. 1000 m sol climbing radius diagnostic outlier'ı ve
primary `±2.5` için exact U6A vertical denominator bulunmaması U6.1'e açık
validation girdileridir; bu stage'de interpolation yapılmadı. U6.1 başlatılmadı.

STEP U6B: PASS

COMBINED 3D RAW DATA READY:
YES

READY FOR U6.1 HOLDOUT VALIDATION:
YES
