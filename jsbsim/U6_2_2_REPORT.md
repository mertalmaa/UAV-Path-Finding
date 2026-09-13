# STEP U6.2.2 — Tested Planner-Safe Envelope Extraction

- Yeni JSBSim simulation: `0`.
- Audited vertical grid: 12 altitude × 8 nonzero command = `96` row.
- Safe climb artık `+2` family'sine bağlı değil: safe command sırası
  `+4/+3/+2/UNAVAILABLE` olarak altitude ile değişiyor.
- Safe descent artık `−3` family'sine bağlı değil: 3000–5000 m'de reviewed
  `−4` family safe envelope'a alındı.
- Measured maximum ve tested-safe maximum ayrı kaldı; `+5`, saturated/full-power
  ve inconsistent boundary response'lar promote edilmedi.
- Straight, LEFT/RIGHT turn ve combined altitude-dependent v2 verileri aynen
  korundu.
- Historical/raw/U6.2/U6.2.1 source artifact'ları değişmedi.

## 1. Eligibility contract

Her `±2/±3/±4/±5 m/s` row aşağıdaki ortak sözleşmeyle bağımsız denetlendi:

- actual-stable, repeatable ve settled;
- actual IAS error `≤5%` (frozen U5 actual-response contract'ı);
- throttle `<0.98` (U5/U5.1 USABLE power-margin contract'ı);
- throttle/surface saturation yok;
- controller-limited category yok;
- strict `INFEASIBLE` değil.

`VALID` güçlü evidence'dır fakat tek seçim şartı değildir. `UNKNOWN` yalnız tüm
numeric sağlık koşulları geçildiğinde ve kalan neden acceptance/tracking kaynaklı
olduğunda `REVIEWED_UNKNOWN` olarak alınır. Raw `ias_tracking` nedeni, actual IAS
`±5%` bandında ve diğer şartlar sağlandığında acceptance miss olarak belgelenir;
bu koşullar sağlanmadığında promotion yapılmaz. `INFEASIBLE`, saturation,
controller/power limitation veya yetersiz settling/stability hiçbir zaman safe
envelope'a alınmaz.

## 2. Uncertainty and neighboring consistency

Safe actual Vz, seçilen row'un measured actual Vz'sinden U6.1 holdout max mutlak
hatası kadar sıfıra doğru derate edilir. `±2/±3` exact target error'ı kullanır.
U6.1'de exact `±4` holdout olmadığı için `+4` için nearest validated `+3`, `−4`
için `−3` error'ı conservative proxy'dir; ayrıca aynı command en az bir komşu
500 m anchor'da da bütün eligibility koşullarını geçmelidir. Böylece isolated
raw maximum safe yapılmaz. `±5` bu şartlardan önce dahi stability/saturation/IAS
boundary'lerinde elenir.

## 3. U6.2.1 vs U6.2.2 vertical envelope

`Measured max`, actual-stable raw response'tur; safe seçimi değildir. `Cmd`,
highest tested-safe command family'dir.

| Alt m | Measured max climb | U6.2.1 safe climb | U6.2.2 Cmd / safe actual | Climb decision | Measured max descent | U6.2.1 safe descent | U6.2.2 Cmd / safe actual | Descent decision |
|---:|---:|---:|---:|---|---:|---:|---:|---|
| 0 | +4.209 | +2.158 | +4 / +4.206 | +4 repeatable/stable; +5 saturated | −3.227 | −3.225 | −3 / −3.225 | −4 stability contract dışı |
| 500 | +4.216 | +2.154 | +4 / +4.213 | +4 healthy; +5 saturated | −3.231 | −3.230 | −3 / −3.230 | −4 stability contract dışı |
| 1000 | +4.229 | +2.150 | +4 / +4.226 | +4 healthy; +5 power/saturation | −3.236 | −3.234 | −3 / −3.234 | −4 stability contract dışı |
| 1500 | +5.136* | +2.147 | +4 / +4.251 | +5 full-throttle/IAS loss; +4 selected | −3.235 | −3.234 | −3 / −3.234 | −4 stability contract dışı |
| 2000 | +3.186 | +2.144 | +3 / +3.183 | +4 saturation/controller limit | −3.234 | −3.232 | −3 / −3.232 | −4 stability contract dışı |
| 2500 | +3.196 | +2.143 | +3 / +3.193 | +4 power/saturation | −3.232 | −3.231 | −3 / −3.231 | −4 stability contract dışı |
| 3000 | +4.155* | +2.143 | +3 / +3.212 | +4 full-throttle/INFEASIBLE | −4.220 | −3.230 | −4 / −4.218 | −4 stable, repeatable, IAS healthy |
| 3500 | +3.986* | +2.146 | +2 / +2.146 | +3 stability miss; +4 saturated | −4.230 | −3.228 | −4 / −4.229 | −4 stable, repeatable, IAS healthy |
| 4000 | +3.159 | +2.152 | +2 / +2.152 | +3 full-throttle/INFEASIBLE | −4.239 | −3.227 | −4 / −4.238 | −4 stable, repeatable, IAS healthy |
| 4500 | +3.005 | +2.164 | +2 / +2.164 | +3 full-throttle/INFEASIBLE | −4.247 | −3.226 | −4 / −4.246 | −4 stable, repeatable, IAS healthy |
| 5000 | +2.859 | — | — | +2 throttle 0.990; higher targets saturated | −4.254 | −3.226 | −4 / −4.252 | −4 stable, repeatable, IAS healthy |
| 5500 | +2.006 | — | — | all climb targets full-throttle/saturated | −3.227 | −3.226 | −3 / −3.226 | −4 stability contract dışı |

`*` Daha yüksek measured response vardır fakat boundary/non-monotonic satır safe
seçilmemiştir. Değişmeyen safe değerler de deliberate'tir: 3500–4500 climb'da
bir sonraki family stability/power şartını; 0–2500 ve 5500 descent'te `−4`
stability şartını geçmez.

## 4. Full altitude envelope table

`A=AVAILABLE`, `U=UNAVAILABLE`. Turn ve combined değerler U6.2.1 v2 ile
birebir korunmuştur.

| Alt m | Safe climb Cmd/Vz | Safe descent Cmd/Vz | Straight thr/reserve | LEFT safe R/rate | RIGHT safe R/rate | Combined C/D | Main vertical limit |
|---:|---:|---:|---:|---:|---:|:---:|---|
| 0 | +4/+4.206 | −3/−3.225 | 0.602/0.398 | 410.2/−5.54 | 454.8/+5.02 | U/U | +5 saturation; −4 unstable |
| 500 | +4/+4.213 | −3/−3.230 | 0.621/0.379 | 431.6/−5.40 | 478.0/+4.90 | U/U | +5 saturation; −4 unstable |
| 1000 | +4/+4.226 | −3/−3.234 | 0.640/0.360 | 454.2/−5.26 | 502.5/+4.77 | U/U | +5 power/saturation; −4 unstable |
| 1500 | +4/+4.251 | −3/−3.234 | 0.660/0.340 | 478.2/−5.12 | 528.6/+4.65 | U/U | +5 full throttle; −4 unstable |
| 2000 | +3/+3.183 | −3/−3.232 | 0.679/0.321 | 503.6/−4.98 | 556.2/+4.53 | U/U | +4 saturation; −4 unstable |
| 2500 | +3/+3.193 | −3/−3.231 | 0.698/0.302 | 530.6/−4.85 | 585.6/+4.42 | U/U | +4 power/saturation; −4 unstable |
| 3000 | +3/+3.212 | −4/−4.218 | 0.718/0.282 | 559.3/−4.72 | 616.8/+4.30 | U/U | +4 climb full throttle; −5 descent IAS/stability |
| 3500 | +2/+2.146 | −4/−4.229 | 0.738/0.262 | 589.8/−4.60 | 650.0/+4.19 | U/A | +3 climb stability miss |
| 4000 | +2/+2.152 | −4/−4.238 | 0.759/0.241 | 622.1/−4.47 | 685.3/+4.08 | U/A | +3 climb saturated |
| 4500 | +2/+2.164 | −4/−4.246 | 0.780/0.220 | 656.5/−4.35 | 722.9/+3.97 | U/A | +3 climb saturated |
| 5000 | — (U) | −4/−4.252 | 0.802/0.198 | 693.1/−4.23 | 763.0/+3.86 | U/A | climb power boundary |
| 5500 | — (U) | −3/−3.226 | 0.825/0.175 | 731.9/−4.12 | 805.7/+3.76 | U/A | climb full throttle; −4 descent unstable |

## 5. Reviewed UNKNOWN decisions

Selected `+4` (0–1500), `+3` (2000–3000), `−3` ve selected `−4`
(3000–5000) rows'un çoğu strict UNKNOWN'dur. Her promotion row-level metadata'da
sekiz eligibility sonucu, raw categories/reasons, actual IAS retention,
throttle/reserve, saturation, settling/repeatability ve uncertainty source ile
işaretlidir. Bu genel UNKNOWN→AVAILABLE kuralı değildir. Örneğin 3500 `+3`
UNKNOWN olmasına rağmen actual-stable check'i geçmediğinden seçilmez; 5000 `+2`
UNKNOWN actual response üretse de throttle `0.990` olduğundan seçilmez.

## 6. Turn, straight and combined audit

Straight'ın 12 altitude-specific IAS/TAS/throttle/power rows'u değişmedi.
`±20°` measured/safe, altitude/direction-dependent turn radius/rate aynen
korundu; LEFT/RIGHT average edilmedi. `±30°` full measured response'u taşımaya ve
planner-safe UNAVAILABLE kalmaya devam eder. Yeni simplification bulunmadı.

Combined climb measured evidence ile birlikte UNAVAILABLE kaldı. Combined
descent 3500–5500 m'de altitude/direction-dependent safe radius/rate/Vz yapısını
korudu. Bu stage combined availability veya geometry'yi genişletmedi.

## 7. Intermediate altitude policy

V3 query interface exact anchor'da local tested envelope döndürür. Command
family değişen iki anchor arasında conservative endpoint seçilir; böylece
örneğin 1500 `+4` ile 2000 `+3` arasında `+4` varsayılmaz. U6.1'in 4000–5500
unsupported climb capability bandında intermediate availability kapalı kalır.
Measured layer ile availability yine ayrıdır. Domain dışı query
`OUT_OF_DOMAIN`'dir.

## 8. Artifacts

- `results/c172p_aircraft_profile_planner_safe_v3.json`
- `results/u6_2_2_tested_safe_envelope_audit.json`
- `results/u6_2_2_non_vertical_preservation_audit.json`
- `results/u6_2_2_provenance.json`
- `results/u6_2_2_result.json`
- `tested_envelope_profile.py`
- `u6_2_2_tested_safe_envelope.py`
- `u6_2_2_tested_safe_envelope_configuration.yaml`
- `test_u6_2_2_tested_safe_envelope.py`

Provenance ID `u6.2.2-b8e1351477e87cc148e9`'dur. V3 aircraft-neutral şemayı,
measured/planner-safe ayrımını, boş global capability constant listesini ve aynı
query family'lerini korur.

## 9. Tests and immutability

Testler 96 raw command audit'ini; climb'da `{+2,+3,+4}`, descent'te `{−3,−4}`
safe family kullanımını; saturated/full-throttle/INFEASIBLE satırların
seçilmediğini; UNKNOWN review metadata'sını; v2-v3 comparison'ı; straight/turn/
combined deep equality'yi; executable intermediate query'leri; aircraft-neutral
schema ve bütün U6A–U6.2.1 source hash'lerinin değişmediğini doğrular.

Yeni JSBSim run yapılmadı. Production AircraftProfile/search entegrasyonu veya
sonraki stage başlatılmadı.

STEP U6.2.2: PASS

FULL TESTED SAFE ENVELOPE EXTRACTED: YES

VERTICAL SAFE VALUES COMMAND-CONSTANT-INDEPENDENT: YES

READY FOR AIRCRAFTPROFILE INTEGRATION: YES
