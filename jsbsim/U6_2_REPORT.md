# STEP U6.2 — C172P Robustness + Planner-Safe Derating and Aircraft Profile Generation

- Wrap bug fixed mi: **YES**, yeni derived-processing helper shortest-angle `[-180,+180)` uygular; historical raw değiştirilmedi.
- Planner-safe profile oluştu mu: **YES**, `results/c172p_aircraft_profile_planner_safe.json`.
- Safe turn: altitude-dependent, direction-separated `±20°`; `±30°` optional/non-guaranteed ve `UNAVAILABLE`.
- Safe climb: `+2 m/s`, 0–4500 m `AVAILABLE`; 5000–5500 m `UNAVAILABLE`.
- Safe descent: reviewed multi-evidence ile `−3 m/s`, 0–5500 m `AVAILABLE`.
- Combined climb: ilk safe profile'da tamamen `UNAVAILABLE`.
- Descending turn: `−2.5 m/s @ ±20°`, 3500–5500 m `AVAILABLE`; daha aşağısı conservative `UNAVAILABLE`.
- 5500 m: straight, `±20°` level turn, straight descent ve descending turn available; straight/combined climb ve `±30°` unavailable.
- Source artifact'ları değişti mi: **NO**; bütün U6A/U6B/U6.1 hash'leri aynı.

## 1. Measurement wrapping fix

Historical radius extraction koduna ve RAW artifact'lara dokunulmadı. Yeni
`planner_safe_measurements.py`, angle/course delta'yı canonical shortest-angle
`[-180°, +180°)` aralığına normalize eder; sequence unwrap ve arc/course radius
helper'ı sağlar. Regression seti `247.277° → −112.723°`, `−190° → 170°`,
`180° → −180°` ve `350/355/1/7 → 350/355/361/367°` unwrap'ını doğrular.

U6.1 outlier case aynı helper ile `194.82 m` stored raw radius yerine derived
processing'de `427.36 m` üretir. Düzeltme yalnız yeni processing pipeline'ına
aittir ve source outlier provenance'ı taşır; U6B raw satırı değiştirilmemiştir.

## 2. Derating methodology

Sabit yüzde/metre/Vz safety factor uydurulmadı. Turn uncertainty allowance,
aynı yön ve `20°` target için U6.1'de gözlenen **maximum absolute holdout
radius error** ile cold-start radius spread'in toplamıdır. Üç repeat deterministik
olduğundan spread `0`; allowance sol `0.284 m`, sağ `0.334 m`'dir. Safe radius
raw measured radius'tan bu evidence kadar daha büyük seçildi; asla küçültülmedi.

Vertical izinleri raw actual response, strict status, bütün anchor/holdout
repeatability, IAS retention, saturation ve throttle reserve birlikte kullanılarak
bandlandı. Combined izinleri U6B/U6.1 direction-specific stability ve
interpolation-suitability map'inden türetildi. UNKNOWN yalnız açık
`reviewed_unknown_to_available=true` ve review basis metadata'sıyla kullanıldı;
sessiz promotion yoktur.

## 3. Safe level-turn policy

İlk guaranteed turn primitive `±20°`'dir. Left/right hiçbir aşamada
ortalama alınmadı; query `altitude + signed bank + direction` ile ayrı safe
radius döndürür. Supported bölgede continuous altitude interpolation yalnız
safe continuous values için izinlidir; availability categorical kalır.

U6.1 sampled `±30°` radius LIMITED olduğu için bütün `±30°` rows profile'da
`OPTIONAL_NON_GUARANTEED_UNAVAILABLE` tutulur. Raw reference metadata'da korunur,
fakat planner-safe radius/rate verilmez.

## 4. Safe radius by altitude/direction

`±20°` planner-safe radius tablosu:

| Altitude m | LEFT safe m | RIGHT safe m |
|---:|---:|---:|
| 0 | 410.20 | 454.81 |
| 500 | 431.56 | 477.98 |
| 1000 | 454.17 | 502.52 |
| 1500 | 478.15 | 528.56 |
| 2000 | 503.60 | 556.20 |
| 2500 | 530.60 | 585.56 |
| 3000 | 559.28 | 616.77 |
| 3500 | 589.76 | 649.96 |
| 4000 | 622.14 | 685.29 |
| 4500 | 656.54 | 722.89 |
| 5000 | 693.08 | 762.96 |
| 5500 | 731.86 | 805.66 |

Her değer source radius + direction-specific holdout allowance'dır. Safe
turn-rate magnitude da aynı yöndeki maximum holdout rate error kadar azaltıldı.
Bu geometri uçağın gerçekte ölçülenden daha dar/hızlı dönebildiğini varsaymaz.

## 5. Safe climb policy

Straight climb `+2 m/s` command 0–4500 m'de AVAILABLE'dır. 0–1500 m raw status
UNKNOWN olsa da bütün source/holdout response'lar actual-stable ve repeatable,
IAS sağlıklı, saturation yoktur; promotion açık review metadata'sıyla yapılır.
2000–4500 m anchor'ları strict VALID'dir. Climb holdout maximum actual-Vz error
`0.00231 m/s`'dir.

4500 m son available anchor'da throttle reserve yaklaşık `0.098`'dir. 5000 m
raw `+2` UNKNOWN, IAS retention failure ve throttle `0.990`; 5500 m INFEASIBLE,
full-throttle/saturation'dır. U6.1 capability interpolation bu bölgede
UNSUPPORTED olduğundan 5000–5500 m discrete `UNAVAILABLE` bırakıldı. Continuous
actual-response tahmini permission'a çevrilmedi.

## 6. Safe descent policy

Planner-safe straight descent `−3 m/s`, 0–5500 m'de AVAILABLE'dır. Bu raw
UNKNOWN'u kör promotion değildir: 12/12 U6A anchor ve 6/6 U6.1 midpoint
actual-stable/repeatable, IAS healthy ve saturation-free'dir; actual response
yaklaşık `−3.23 m/s`, holdout max error `0.00147 m/s`'dir. Profile her altitude
row'unda actual reference, power margin, confidence ve review basis taşır.

Daha agresif `−4/−5 m/s` evidence karışık olduğundan safe profile'a alınmadı.
Böylece tek eski `MAX_DESCENT=−2` sabiti yerine altitude-indexed/evidence-backed
`−3` query rows üretildi, fakat gözlenen sınır otomatik maksimum yapılmadı.

## 7. Climbing-turn availability

Combined climb ilk planner-safe profile'da bütün domain ve iki yönde
`UNAVAILABLE`'dır. Low/mid raw evidence MARGINAL ve direction/status açısından
karışıktır; 4000–5500 m capability interpolation UNSUPPORTED, 4500/5000
full-throttle ve 5250/5500 degradation içerir. Optimistic continuous availability
üretilmedi. Bu policy stage failure değildir; planner daha sonra straight climb
+ level turn gibi sıralı alternatifler kullanabilir.

## 8. Descending-turn availability

Combined descending turn `bank ±20°`, `Vz −2.5 m/s` için 3500–5500 m'de
AVAILABLE'dır. Boundary, 3250 holdout'ın iki yön strict USABLE olması ve 4000
anchor'ın aynı sonucu vermesi üzerine bir sonraki profile grid noktası olan
3500 m'de açılır. 0–3000 m raw physical response stable olsa da mixed strict
status nedeniyle conservative `UNAVAILABLE` bırakıldı.

Safe combined-descent radius, direction-separated supported interpolation
reference'ına holdout maximum error ekler: sol `+2.234 m`, sağ `+2.373 m`.
3500→5500 safe radius sol yaklaşık `597.98→736.58 m`, sağ
`642.30→792.68 m`'dir.

## 9. Supported/limited/unsupported treatment

- `SUPPORTED`: straight continuous metrics, direction-separated `±20°` level
  turns, `−2` descent response ve combined descending-turn continuous geometry.
- `LIMITED`: `±30°`, reviewed `−3` descent, low/mid straight climb ve lower-alt
  combined climb. Yalnız açık validated subset/conservative treatment kullanılır.
- `UNSUPPORTED`: straight-climb capability 4000–5500 ve combined-climb
  capability 4000–5500. Availability interpolate edilmez; discrete band veya
  `UNAVAILABLE` kullanılır.

## 10. Robustness evidence

U6A canonical `204/204`, U6B combined `24/24` row repeatable; U6.1 holdout
`62/62` point üç cold-start'ta repeatable'dır. Level-turn radius cold-start
spread'i test edilen `±20°` holdout'larda `0 m`; direction-specific interpolation
max error allowance olarak taşınır. Vertical ve combined uncertainty metadata'sı
holdout max error, saturation margin, power evidence, confidence ve treatment
içerir. Full-throttle response normal primitive'e dönüştürülmedi.

## 11. Planner-safe profile schema

Canonical `results/c172p_aircraft_profile_planner_safe.json`:

- identity/domain/units/provenance/same-stack metadata,
- `AVAILABLE / UNAVAILABLE / OUT_OF_DOMAIN` logical semantics,
- `straight`, `level_turn`, `straight_climb`, `straight_descent`,
  `climbing_turn`, `descending_turn` capability families,
- altitude/direction-indexed safe values ve evidence metadata,
- supported-continuous vs discrete availability policy,
- generic `turn_query`, `vertical_query`, `combined_query` interface contract,
- mevcut exact-grid loader için `VALID/INFEASIBLE` compatibility tables

taşır. `>5500 m` query mevcut loader'ın off-grid error yoluyla logical
`OUT_OF_DOMAIN` sonucuna gider. Profile planner-ready'dir fakat production
search'e henüz bağlanmamıştır.

## 12. Swappability contract

Schema aircraft-neutral'dır: başka aircraft aynı identity, units, grids,
capability families, availability semantics ve query contract'ı sağlayan yeni
bir profile ile aynı loader/planner arayüzüne bağlanabilir. Profile içinde
aircraft kimliği data'dır; planner capability logic'inde `if aircraft ==
"c172p"` şartı gerektirmez. Mevcut loader, expected aircraft/stack parametreleri
verildiğinde artifact'ın legacy turn/vertical tablolarını başarıyla yükler.
Production integration ve loader genişletmesi sonraki ALG-1 stage'ine bırakıldı.

## 13. Tests

Testler shortest-angle/unwrap/radius regression'ı, source raw immutability,
safe radius'ın hiçbir source radius'tan küçük olmamasını, safe turn-rate'in
optimistic olmamasını, climb/descent'in raw evidence'i aşmamasını, reviewed
UNKNOWN metadata'sını, combined climb'ın sıfır AVAILABLE row taşımasını,
descending-turn discrete bandını, 5500 family policy'sini, `>5500` off-domain
query'yi, left/right ayrımını, generic loader compatibility'sini ve production
planner/search'in bağlanmadığını doğrular. U5–U6.2 targeted regression'ları
PASS'tir.

## 14. Source immutability

U6A core RAW, U6B combined RAW, U6.1 core/combined holdouts, error summary,
suitability ve outlier diagnostic artifact'larının SHA-256 değerleri derivation
öncesi/sonrası aynıdır. Yeni JSBSim run sayısı `0`'dır. Provenance ID:
`u6.2-2a36f7e78b747ec099a8`.

## 15. Known limitations

Profile ilk conservative sürümdür: `±30°` guaranteed değildir; combined climb
tamamen kapalıdır; combined descent yalnız 3500 m üzerinde açıktır; speed state
dimension değildir. Availability continuous interpolate edilmez. Orbit/loiter,
heading, primitives, transition/swept safety, grid re-gate ve search integration
yapılmadı. **DIRECTLY INFEASIBLE != UNREACHABLE.** Next stage ALG-1 —
AircraftProfile Integration / Profile Loader Check'tir ve başlatılmadı.

STEP U6.2: PASS

PLANNER-SAFE PROFILE READY:
YES

MEASUREMENT PIPELINE FIXED:
YES

AIRCRAFT-SWAPPABLE PROFILE:
YES

READY FOR AIRCRAFTPROFILE INTEGRATION:
YES
