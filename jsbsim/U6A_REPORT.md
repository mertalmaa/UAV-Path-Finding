# STEP U6A — C172P Core Raw LUT

- Reuse: 19 benzersiz same-stack point (`U5=12`, `U5.2=7`; U5.1 yeni raw point eklemedi).
- Yeni çalışma: 161 benzersiz point, 483 cold-start run.
- Straight grid: tamamlandı (`12/12`, tamamı strict `VALID`).
- Level-turn LUT: tamamlandı (`84/84`).
- Straight-vertical LUT: tamamlandı (`108/108`).
- 5500 m: straight güçlü; `±20/±30°` turn actual-stable/repeatable; strict descent `−2 m/s`; climb power/saturation sınırında ve strict usable değil.
- Canonical RAW LUT: hazır; planner-safe, interpolated veya derated değildir.

## 1. Frozen stack

Yalnız final primary `c172p`, nominal `40 m/s IAS` ile çalıştırıldı. Stack;
stock `FCS: c172`, U1–U3 frozen bounded IAS/Vz/bank/beta outer-loop,
`c172p-production-mixture-pressure-ratio-v1`, aynı U2 fixture
(`1865 lb`, `185 lb` fuel, gözlenen CG `x=42.005`, `y=−1.351`, `z=37.565 in`),
US Standard sıfır-rüzgâr atmosferi, `0.01 s` timestep ve aynı initialization /
measurement contract'tır. Gain, PID, XML, mixture veya propulsion policy
değişmedi. Final JSBSim replay aynı stack'i kullanmalıdır.

## 2. Reuse/new-run inventory

Exact aircraft + altitude + IAS + maneuver target ve aynı stack sözleşmesiyle
19 source point reuse edildi: U5'ten 12, U5.2'den 7. Her reused row
`source_stage`, `source_point_id`, `source_provenance` ve üç source run ID taşır.
U5.1 offline audit olduğundan ek raw source point sağlamadı; bütün U5.1
artifact'ları yine immutability hash setine alındı.

Eksik hücreler için 161 point / 483 benzersiz cold-start run üretildi:
`straight=5`, nonzero `turn=60`, nonzero `vertical=96`. Canonical tablolarda
12 straight + 84 turn + 108 vertical = 204 row vardır; turn `0°` ve vertical
`0 m/s` satırları aynı-altitude straight point'in açık, türetilmiş eşdeğeridir ve
yeniden simüle edilmiş duplicate değildir.

## 3. Straight baseline

0:500:5500 m'deki 12 anchor'ın tamamı strict `VALID`, settled, repeatable ve
actual-stable'dır. Actual IAS `39.91–39.98 m/s` aralığında kalır. Throttle
irtifayla düzenli biçimde `0.602`'den `0.825`'e yükselir; 5500 m straight power
margin yaklaşık `0.175`'tir. Dolayısıyla **c172p @ 40 m/s, test edilen 0–5500 m
straight grid'inin tamamında stable'dır**. Bu sonuç 6000 m veya true service
ceiling iddiası değildir.

## 4. Level-turn LUT

Her altitude'da `0/−10/+10/−20/+20/−30/+30°`, sol/sağ yönler ayrı olarak ve
üç cold-start ile tamamlandı. Status dağılımı `47 VALID / 37 UNKNOWN`; turn
satırında `INFEASIBLE` yoktur. Bütün `±20/±30°` hedefler actual-stable ve
repeatable'dır. `±10°` raw ölçümleri korunur, fakat kullanılan actual-stable
kontratı en az `15°` anlamlı bank istediğinden bu küçük-bank satırları “useful
turn” özetine sokulmadı.

Measured radius planner geometrisinin sonraki ana kaynağıdır; theory yalnız
diagnostic'tir. `±20°` radius irtifayla açıkça büyür: 0 m'de sol/sağ yaklaşık
`410/454 m`, 3000 m'de `559/616 m`, 5500 m'de `732/805 m`. `±30°` measured
radius sol/sağ asimetrisi ve bazı non-monotonic raw noktalar içerir; bunlar
düzeltilmedi. Genel olarak TAS ve radius artarken turn-rate büyüklüğü azalır:
useful uç değerler 0 m'de yaklaşık `−8.94/+8.10°/s`, 5500 m'de
`−6.59/+6.03°/s`'dir.

## 5. Straight vertical LUT

Her altitude'da `−5/−4/−3/−2/0/+2/+3/+4/+5 m/s` tamamlandı. Status dağılımı
`30 VALID / 56 UNKNOWN / 22 INFEASIBLE`'dır. Strict descent için bütün
altitude'larda en büyük tested sustainable VALID hedef `−2 m/s`'dir. Stable
actual descent çoğu altitude'da yaklaşık `−3.23 m/s`, 3000–5000 m'de yaklaşık
`−4.22…−4.25 m/s`'ye ulaşır; bunlar strict status gevşetmeden kaydedildi.

Strict climb sonucu 2000–4500 m'de en yüksek tested sustainable VALID hedef
`+2 m/s`; 0–1500 ve 5000–5500 m'de strict VALID pozitif hedef yoktur. Actual
stable response düşük altitude'da çoğunlukla yaklaşık `+4.21 m/s`, 4000 m'de
`+3.16`, 4500 m'de `+3.01`, 5000 m'de `+2.86`, 5500 m'de `+2.01 m/s` olur.
1500 m'deki `+5.14 m/s` actual-stable raw gözlem gibi tekil/non-monotonic
sonuçlar true maximum sayılmadı.

## 6. Status/failure interpretation

Strict semantics aynen korundu: `UNKNOWN != INFEASIBLE` ve tracking failure
physical response failure değildir. Tüm canonical satırlarda actual response,
settling/repeatability, saturation ve kategorize/raw failure reason bulunur.
Toplam status `89 VALID / 93 UNKNOWN / 22 INFEASIBLE`'dır.

Actual-stable fakat strict non-VALID 69 satır vardır (`62 UNKNOWN`,
`7 INFEASIBLE`). Örnekler: 5500 m `−20°` turn strict `UNKNOWN` iken actual bank
`−21.21°`, radius `732 m`, stable/repeatable; 5500 m `+2 m/s` climb strict
`INFEASIBLE` iken actual `+2.006 m/s` stable/repeatable'dır, fakat throttle
`1.0`, IAS `38.85 m/s` ve power/saturation/speed-retention kanıtı nedeniyle
promote edilmez. Sol `−20/−30°` turnlerin çoğundeki `UNKNOWN`, frozen strict
bank-tracking acceptance'tan gelir; fiziksel dönüş yokluğu olarak yorumlanmaz.

## 7. Altitude trends

| Altitude m | Straight IAS / throttle | Useful radius m | Turn rate °/s | Climb: strict VALID / highest observed stable | Descent: strict VALID / largest observed stable |
|---:|---:|---:|---:|---:|---:|
| 0 | 39.91 / 0.602 | 253–454 | −8.94…+8.10 | — / +4.21 | −2 / −3.23 |
| 500 | 39.92 / 0.621 | 266–478 | −8.68…+7.89 | — / +4.22 | −2 / −3.23 |
| 1000 | 39.93 / 0.640 | 281–502 | −8.44…+7.69 | — / +4.23 | −2 / −3.24 |
| 1500 | 39.94 / 0.660 | 278–528 | −8.21…+7.48 | — / +5.14* | −2 / −3.24 |
| 2000 | 39.95 / 0.679 | 271–556 | −7.98…+7.29 | +2 / +3.19 | −2 / −3.23 |
| 2500 | 39.95 / 0.698 | 265–585 | −7.76…+7.10 | +2 / +3.20 | −2 / −3.23 |
| 3000 | 39.96 / 0.718 | 260–616 | −7.55…+6.91 | +2 / +4.15* | −2 / −4.22 |
| 3500 | 39.96 / 0.738 | 255–650 | −7.34…+6.72 | +2 / +3.99* | −2 / −4.23 |
| 4000 | 39.97 / 0.759 | 252–685 | −7.14…+6.54 | +2 / +3.16 | −2 / −4.24 |
| 4500 | 39.97 / 0.780 | 322–723 | −6.95…+6.37 | +2 / +3.01 | −2 / −4.25 |
| 5000 | 39.97 / 0.802 | 313–763 | −6.76…+6.19 | — / +2.86 | −2 / −4.25 |
| 5500 | 39.98 / 0.825 | 306–805 | −6.59…+6.03 | — / +2.01 | −2 / −3.23 |

`*` Stable actual observation; strict target capability/maximum değildir.
Turn radius aralığının alt ucu `±30°` raw asymetrisinden etkilendiği için
monotonic envelope olarak kullanılmamalıdır. Power/saturation climb tarafında
önce `+5` hedefte (0 m'den itibaren), yaklaşık 2000 m'den sonra `+4`, 4000 m'den
sonra `+3`, 5500 m'de `+2 m/s` hedefte temel limitation olur. Straight ve level
turn'de buna eşdeğer throttle boundary görülmez.

Önemli soruların açık cevapları:

1. **Straight stable mı?** Evet; 0–5500 m'deki 12/12 anchor strict VALID ve
   actual-stable'dır.
2. **Turn radius nasıl değişiyor?** Özellikle `±20°` için altitude ile büyür;
   0 m sol/sağ `410/454 m`, 5500 m `732/805 m` ölçülmüştür.
3. **`±20/±30°` davranışı nasıl değişiyor?** Hepsi actual-stable/repeatable
   kalır; turn-rate büyüklüğü genel olarak azalır, radius artar. `±30°` raw
   radius'ta yön asimetrisi/non-monotonic ölçümler vardır; sol strict status
   çoğunlukla tracking nedeniyle UNKNOWN'dur.
4. **Climb altitude ile nasıl değişiyor?** Strict `+2 m/s` yalnız 2000–4500 m
   bandında VALID; yüksek irtifada stable actual tırmanış yaklaşık `+3.16`
   (4000), `+2.86` (5000), `+2.01 m/s`'ye (5500) düşer ve strict usability
   kaybolur.
5. **Descent altitude ile nasıl değişiyor?** `−2 m/s` bütün gridde strict
   VALID'dir; daha yüksek actual descent sıkça stable/repeatable olsa da IAS /
   tracking acceptance nedeniyle UNKNOWN kalır.
6. **Power/saturation nerede temel limitation?** Pozitif vertical family'de;
   `+5` düşük irtifadan, `+4` yaklaşık 2000 m'den, `+3` 4000 m'den ve `+2`
   5500 m'de throttle boundary'ye ulaşır.
7. **5500 m'de güvenilir family'ler hangileri?** Straight, actual-stable
   `±20/±30°` level turn ve moderate descent. Strict pozitif climb güvenilir
   değildir; sol turns actual-stable olsa da strict UNKNOWN'dur.
8. **Stable/repeatable fakat strict non-VALID neler?** Başlıca sol
   `−20/−30°` turns, çeşitli `−3/−4 m/s` descents ve bazı `+2/+3/+4 m/s`
   climbs; toplam 62 UNKNOWN ve 7 INFEASIBLE canonical satır bu ayrımı taşır.

## 8. 5500 m summary

- Straight: strict `VALID`, actual IAS `39.98 m/s`, throttle `0.825`.
- Level turn: `±20/±30°` actual-stable/repeatable. Sağ `+20/+30°` strict
  `VALID`; sol `−20/−30°` strict `UNKNOWN` (tracking acceptance). Measured
  radius sırasıyla sol/sağ `732/805 m` ve `306/500 m`.
- Climb: strict VALID pozitif hücre yok. `+2` hedef actual `+2.006 m/s` stable
  olsa da full throttle, IAS kaybı ve saturation nedeniyle `INFEASIBLE`.
  `+3/+4/+5` hedefler de strict `INFEASIBLE` ve actual-stable kontratı geçmez.
- Descent: `−2 m/s` strict `VALID`; `−3` hedef actual `−3.227 m/s` stable fakat
  speed-retention nedeniyle `UNKNOWN`; `−4/−5` stable kontratı geçmez.

Sonuç: 5500 m'de güvenilir strict çekirdek straight, sağ level-turn ve moderate
straight descent'tir. İki yönlü `±20/±30°` dönüş fiziksel/actual-stable olarak
güçlüdür, fakat sol yön strict UNKNOWN olarak kalır. Positive climb planner-safe
değildir.

## 9. Raw LUT schema

Canonical `results/c172p_core_aircraft_lut_raw.json`, ayrı
`straight_table`, `level_turn_table`, `straight_vertical_table` family'lerini
içerir; combined tablo yoktur. Ölçülen IAS/TAS/Vz, attitude/aero açıları,
throttle/RPM/power/thrust, controller/surface kullanımı, saturation, settling,
repeatability, strict status, actual-stable flag, failure reason ve reuse
metadata korunur. Turn satırları measured ve theoretical radius'u ayrı taşır.
Artifact `planner_ready=false`, `interpolated=false`, `derated=false` ve
`true_service_ceiling_claim=false` olarak işaretlidir.

## 10. Provenance

Provenance ID: `u6a-bce6d9cac10c00089ec6`. Canonical metadata ve ayrı
provenance artifact'ı aircraft/IAS/domain/grid, frozen controller/FCS ve mixture,
mass/fuel/CG, atmosphere, timestep, initialization, JSBSim/Python/platform,
repository commit, config/harness hash'leri ve generation ID içerir. U5, U5.1 ve
U5.2 source hash'leri execution öncesi/sonrası aynıdır.

## 11. Artifacts

- `results/c172p_core_aircraft_lut_raw.json`
- `results/u6a_new_points.json`
- `results/u6a_new_runs.json`
- `results/u6a_reuse_audit.json`
- `results/u6a_altitude_summary.json`
- `results/u6a_provenance.json`
- `results/u6a_result.json`
- `u6a_c172p_core_raw_lut_configuration.yaml`
- `u6a_c172p_core_raw_lut.py`
- `test_u6a_c172p_core_raw_lut.py`

## 12. Tests

Artifact testi exact grids/counts, schema, c172p/40 m/s/0–5500 scope,
duplicate-free canonical keys, left/right yön, üç status'un korunması, measured
radius/actual Vz, üç cold-start, same-stack reuse/provenance, combined row
yokluğu, RAW flags ve U5/U5.1/U5.2 hash immutability şartlarını doğrular.
U6A testi ve U5/U5.1/U5.2 targeted regression'ları PASS'tir.

## 13. project.md update

Kalıcı kayda final primary `c172p`, nominal `40 m/s`, current validated planner
domain `0–5500 m`, U6A bağımsız core characterization sonucu, canonical RAW LUT,
straight/turn/vertical eğilimleri, high-altitude climb limitation ve combined
3D'nin U6B'ye bırakıldığı eklendi. Raw telemetry kopyalanmadı.

## 14. Blockers

U6A için blocker yoktur. Bu stage independent core capability üretir;
`level turn + straight climb = climbing turn` varsayımı yapılmaz. U6B
başlatılmadı. Planner binding için daha sonra combined-3D validation,
interpolation/holdout ve planner-safe derating gerekir.

STEP U6A: PASS

C172P CORE RAW LUT READY:
YES

CURRENT DOMAIN:
0–5500 m @ nominal 40 m/s IAS

READY FOR U6B:
YES
