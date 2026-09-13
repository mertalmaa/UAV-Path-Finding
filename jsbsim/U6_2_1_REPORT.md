# STEP U6.2.1 — Full Altitude-Dependent Aircraft Capability Profile Fix

- Bulunan global simplification'lar: climb safe response `+2 m/s`, descent safe
  response `−3 m/s`, combined descent safe Vz `−2.5 m/s`; combined climb'da ise
  UNAVAILABLE satırları measured response'u taşımıyordu. Straight ve turn
  telemetry'si de capability satırlarında eksikti.
- Kaldırılanlar: bütün aircraft capability global rate/geometry varsayımları.
  Nominal IAS `40 m/s` yalnız planner context istisnası olarak kaldı; actual IAS
  olduğu varsayılmadı.
- 12 altitude row: tamam (`0:500:5500 m`).
- Turn: altitude + direction dependent; LEFT/RIGHT ayrılmaya devam ediyor.
- Climb: measured maksimum ve safe local response altitude-dependent.
- Descent: measured maksimum ve safe local response altitude-dependent.
- Combined: measured/interpolated response ve safe geometry her altitude/yön
  için ayrı; unavailable satırlar da evidence taşıyor.
- Measured/safe ayrımı: bütün altı capability family'de tamam.
- Source data: değişmedi; yeni JSBSim run `0`.

## 1. Global capability audit

U6.2'nin `±20°` safe radius/rate değerleri zaten altitude ve direction'a göre
değişiyordu; bunlar korunup eksik IAS/TAS/Nz/beta/Vz/power/quality evidence'ı
eklendi. Sorun vertical ve combined katmanda doğrulandı: local actual response
metadata'da bulunsa bile planner-facing safe rate `+2`, `−3` veya `−2.5 m/s`
olarak global command sabitine eşitlenmişti. Ayrıca combined climb satırları
UNAVAILABLE iken measured response'u kaybediyordu.

Yeni v2 şemada global aircraft capability constant listesi boştur. `40 m/s IAS`
yalnız nominal query context'idir. Bank ve Vz command değerleri de fiziksel
capability sabiti değil, satırı seçen maneuver context'tir. Availability bandı
politika olarak belgelenir fakat sonuç her canonical altitude satırında ayrıca
taşınır.

## 2. Derivation rule

Her satır aşağıdaki zinciri açıkça taşır:

`altitude + family + direction/command → measured → evidence → planner_safe`

Safe değerler raw maksimum değildir. `±20°` turn'de direction-specific U6.1 max
radius/rate error allowance uygulanır. Climb'da local `+2` command actual response
değerinden U6.1 max error çıkarılır; daha yüksek stable observation yalnız
measured katmanda korunur. Descent'te local `−3` response büyüklüğü aynı error
kadar sıfıra yaklaştırılır; `−4.2 m/s` civarı observations safe maximum'a
promote edilmez. Combined descent'te radius artırılır, turn-rate ve descent
magnitude direction-specific holdout error kadar azaltılır.

## 3. Required altitude audit

`A=AVAILABLE`, `U=UNAVAILABLE`. `Meas/safe` sütununda safe değer unavailable ise
`—` gösterilir. Combined safe değer `radius m, Vz m/s` biçimindedir.

| Alt m | Straight IAS | Thr/margin | L/R safe radius m | L/R safe rate °/s | Meas/safe climb | C | Meas/safe descent | D | Comb C/D | Combined descent safe L/R | Main limitation |
|---:|---:|---:|---:|---:|---:|:--:|---:|:--:|:--:|---|---|
| 0 | 39.91 | 0.602/0.398 | 410.2/454.8 | −5.54/+5.02 | 4.21/2.16 | A | −3.23/−3.23 | A | U/U | — | combined climb disabled; no combined evidence |
| 500 | 39.92 | 0.621/0.379 | 431.6/478.0 | −5.40/+4.90 | 4.22/2.15 | A | −3.23/−3.23 | A | U/U | — | combined climb disabled; no combined evidence |
| 1000 | 39.93 | 0.640/0.360 | 454.2/502.5 | −5.26/+4.77 | 4.23/2.15 | A | −3.24/−3.23 | A | U/U | — | combined climb disabled |
| 1500 | 39.94 | 0.660/0.340 | 478.2/528.6 | −5.12/+4.65 | 5.14*/2.15 | A | −3.24/−3.23 | A | U/U | — | combined climb disabled; non-monotonic raw climb retained |
| 2000 | 39.95 | 0.679/0.321 | 503.6/556.2 | −4.98/+4.53 | 3.19/2.14 | A | −3.23/−3.23 | A | U/U | — | combined climb disabled |
| 2500 | 39.95 | 0.698/0.302 | 530.6/585.6 | −4.85/+4.42 | 3.20/2.14 | A | −3.23/−3.23 | A | U/U | — | combined climb disabled |
| 3000 | 39.96 | 0.718/0.282 | 559.3/616.8 | −4.72/+4.30 | 4.15*/2.14 | A | −4.22/−3.23 | A | U/U | — | combined climb disabled; raw non-monotonicity retained |
| 3500 | 39.96 | 0.738/0.262 | 589.8/650.0 | −4.60/+4.19 | 3.99*/2.15 | A | −4.23/−3.23 | A | U/A | L 598.0,−2.77; R 642.3,−2.78 | combined climb disabled |
| 4000 | 39.97 | 0.759/0.241 | 622.1/685.3 | −4.47/+4.08 | 3.16/2.15 | A | −4.24/−3.23 | A | U/A | L 627.5,−2.76; R 674.5,−2.77 | climb power boundary; combined climb |
| 4500 | 39.97 | 0.780/0.220 | 656.5/722.9 | −4.35/+3.97 | 3.01/2.16 | A | −4.25/−3.23 | A | U/A | L 662.5,−2.75; R 712.6,−2.76 | climb power boundary; combined climb |
| 5000 | 39.97 | 0.802/0.198 | 693.1/763.0 | −4.23/+3.86 | 2.86/— | U | −4.25/−3.23 | A | U/A | L 697.6,−2.75; R 750.6,−2.76 | climb power/IAS margin |
| 5500 | 39.98 | 0.825/0.175 | 731.9/805.7 | −4.12/+3.76 | 2.01/— | U | −3.23/−3.23 | A | U/A | L 736.6,−2.74; R 792.7,−2.75 | climb saturation/IAS margin |

`*` Tekil/non-monotonic stable raw observation; safe maximum yapılmadı. Confidence
family-specific'tir: straight ve `±20°` turn HIGH; reviewed descent MEDIUM;
available climb strict VALID anchor'da HIGH, reviewed UNKNOWN anchor'da MEDIUM;
unavailable capability WITHHELD; 0/500 m combined NO_EVIDENCE.

## 4. Straight rows

Her anchor measured nominal/actual IAS, IAS retention, TAS, actual Vz, throttle,
RPM, power, thrust, power margin, pitch, AoA, beta, settling, repeatability,
saturation, raw status/failure reason ve source provenance taşır. Böylece
`straight valid everywhere` global flag'i yerine 12 bağımsız aircraft response
satırı vardır. Straight interpolation U6.1'e göre LINEAR/SUPPORTED'dır.

## 5. Level turn rows

Her altitude'da `−20/+20/−30/+30°` ve LEFT/RIGHT ayrı tutulur. `±20°` satırları
measured bank/radius/rate, IAS/TAS/Nz/beta/Vz, power ve quality ile safe
bank/radius/rate içerir. Safe radius raw'dan küçük değildir; rate magnitude
validation error kadar azaltılmıştır. `±30°` UNAVAILABLE kalır fakat bütün raw
measured response'u, LIMITED validation quality ve unavailable reason'ı taşır.

## 6. Straight climb rows

Measured katman iki ayrı gerçeği korur: en yüksek observed stable response ve
safe derivation'ın kullandığı local `+2` response. Örneğin 4000 m'de measured
maximum `+3.159`, safe local capability `+2.153 m/s`; 5000 m'de measured
`+2.859 m/s` mevcut olmasına rağmen power/IAS boundary nedeniyle availability
UNAVAILABLE ve safe value `null`'dır. Böylece measured response ile availability
birbirinden ayrıdır.

Canonical 0–4500 m anchors AVAILABLE, 5000/5500 m UNAVAILABLE'dır. Intermediate
altitude'da continuous measured response üretilebilir; U6.1'in 4000–5500
UNSUPPORTED capability bandında availability interpolate edilmez. Örneğin
4250 m query measured değer döndürür fakat UNAVAILABLE/safe `null` kalır.

## 7. Straight descent rows

Measured maximum local ve altitude-dependent'tir: 3000–5000 m'de yaklaşık
`−4.22…−4.25 m/s`, diğer anchor'larda çoğunlukla `−3.23 m/s`. Safe değer raw
maximum değildir; her altitude'daki reviewed `−3` response U6.1 error kadar
sıfıra yaklaştırılır ve yaklaşık `−3.225…−3.234 m/s` arasında değişir. LIMITED
intermediate query'de conservative endpoint treatment uygulanır.

## 8. Combined rows

Combined climb bütün altitude ve iki yönde UNAVAILABLE kalır. Buna rağmen
1000–5500 m satırlarında measured/interpolated bank, Vz, radius, rate, IAS
retention, throttle/power margin, saturation, settling/repeatability ve source
basis vardır. 0/500 m'de extrapolation yapılmadığı için measured `null` ve
confidence NO_EVIDENCE'dır. 1000 m LEFT climb, historical raw `194.82 m` ile
derived corrected `427.36 m` değerini birlikte taşır.

Combined descent 0–3000 m UNAVAILABLE, 3500–5500 m AVAILABLE'dır. Available
satırlarda LEFT/RIGHT local safe radius/rate/Vz ayrıdır; tek global geometry veya
`−2.5 m/s` safe response kullanılmaz. Intermediate continuous values yalnız
U6.1 SUPPORTED direction-separated policy ile interpolate edilir.

## 9. Query contract

Aircraft-neutral `AltitudeAwareAircraftCapabilityProfile` şu executable
interface'i sağlar:

- `straight_query(altitude)`
- `turn_query(altitude, direction, bank)`
- `vertical_query(altitude, CLIMB/DESCENT)`
- `combined_query(altitude, direction, CLIMB/DESCENT)`

Exact anchor local row döndürür. SUPPORTED fields linear; LIMITED fields
conservative endpoint; UNSUPPORTED availability kapalıdır. Domain dışı query
`OUT_OF_DOMAIN` döndürür. Interface'te `c172p` branch'i yoktur; identity yalnız
artifact metadata'sıdır. Production planner/search'e bağlanmadı.

## 10. Artifacts and provenance

- `results/c172p_aircraft_profile_planner_safe_v2.json` — yeni canonical v2
- `results/u6_2_1_global_constant_audit.json`
- `results/u6_2_1_altitude_audit.json`
- `results/u6_2_1_provenance.json`
- `results/u6_2_1_result.json`
- `aircraft_capability_profile.py`
- `u6_2_1_altitude_profile.py`
- `u6_2_1_altitude_profile_configuration.yaml`
- `test_u6_2_1_altitude_profile.py`

Provenance ID `u6.2.1-bed6f2ed0cc76cd5efc6`'dır. U6A raw, U6B raw, U6.1
holdout/suitability ve bütün U6.2 artifact hash'leri generation öncesi/sonrası
aynıdır. Historical U6.2 v1 profile overwrite edilmedi.

## 11. Tests and limitations

Testler 12 exact anchor'ı, altı family'nin altitude rows'unu, full straight
metadata'yı, LEFT/RIGHT turn ayrımını, `±30°` measured-but-unavailable durumunu,
altitude-varying climb/descent measured ve safe değerlerini, unavailable climb
evidence'ının korunmasını, combined local geometry'yi, executable intermediate
query policy'lerini, boş global capability constant listesini, aircraft-neutral
şemayı ve source immutability'yi doğrular.

Bu stage yeni physical evidence üretmez; combined 0/500 m extrapolate edilmez,
combined climb açılmaz, high-alt climb zorla enable edilmez. Production planner,
search, heading ve primitives değiştirilmedi. Sonraki aşama başlatılmadı.

STEP U6.2.1: PASS

FULL ALTITUDE-DEPENDENT PROFILE: YES

GLOBAL AIRCRAFT CAPABILITY CONSTANTS REMOVED: YES

MEASURED + SAFE VALUES PRESERVED: YES

AIRCRAFT-SWAPPABLE: YES

READY FOR AIRCRAFTPROFILE INTEGRATION: YES
