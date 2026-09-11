# J3 — Nominal CAS Selection + Throttle Policy Baseline

> J3.1 daha sonra domain'i 0–6000 m MSL'ye genişletti ve 305 KCAS kararını
> yeniden doğruladı. Güncel kapsam için `J3_1_DOMAIN_EXTENSION_REPORT.md` kullanılır;
> bu rapordaki özgün 1500–6000 m sonuçları değiştirilmemiştir.

## Sonuç

**J3 PASS.** 1500–6000 m MSL characterization domain'i için tek nominal hız
**305 KCAS** seçildi. Bu değer yalnız prototype characterization context'idir;
planner state'ine eklenmiş bir hız boyutu veya gerçek F-16 operasyon limiti değildir.

- Provenance: `j3-7d701f785aae566708fc`
- Parent J2 provenance: `j2-c25dabb783f4ac1ffdef`
- Grid: 10 irtifa × 5 CAS × 3 bağımsız cold start = 150 run
- Sonuç: 150 `VALID`, 0 `UNKNOWN`, 0 `INFEASIBLE`
- Trim: 150/150 başarılı
- En büyük cold-start relative deviation: `1.66e-16` (J1 yaklaşık %1 kapısının altında)

## Test tasarımı

J2'nin çalışan 335 KCAS noktası reference seed olarak tutuldu. Adaylar 30-knot
aralıkla iki yana genişletildi: **275, 305, 335, 365, 395 KCAS**. Bu değerler
F-16 limitleri olarak yorumlanmaz. Alt uç, modelde gözlenen 250 KCAS trailing-edge
flap schedule ayrımının üstünü; üst uç ise 6000 m'de Mach 0.9 schedule ayrımına
yaklaşan fakat onu geçmeyen rejimi örneklemek için seçildi.

Her noktada fresh JSBSim instance, J2 reference configuration, full trim, 30 s
replay ve son 20 s measurement window kullanıldı. Standard atmosphere, zero wind,
zero turbulence, fixed mass/fuel/CG, gear up, speedbrake closed, native FCS ve
afterburner-off şartları korundu.

JSBSim koordinat dönüşümünde nominal sıfır rüzgârın bazı koşullarda en fazla
`2.39e-12 ft/s` artık üretmesi nedeniyle, J3'te yalnız bu sayısal artık için
`1e-9 ft/s` tolerans kaydedildi. Gerçek rüzgârı maskelemediğini doğrulayan test
vardır; trim, CAS, gamma, drift veya control kapıları gevşetilmedi.

## Aday karşılaştırması

| CAS | V/U/I | Worst abs(AoA) | Throttle-pos | Mach | Max CAS err | Max gamma err | Max alt drift | Max speed drift | Max surface | Min score margin |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 275 | 10/0/0 | 2.763° | 0.435–0.543 | 0.453–0.595 | 0.058% | 0.030° | 1.683 m | 0.136 KCAS | 0.0497 | 0.0996 |
| **305** | **10/0/0** | **1.983°** | **0.473–0.588** | **0.502–0.657** | **0.064%** | **0.035°** | **2.137 m** | **0.164 KCAS** | **0.0475** | **0.2196** |
| 335 | 10/0/0 | 1.399° | 0.508–0.627 | 0.551–0.718 | 0.070% | 0.043° | 2.695 m | 0.201 KCAS | 0.0460 | 0.2017 |
| 365 | 10/0/0 | 0.950° | 0.541–0.662 | 0.600–0.779 | 0.075% | 0.051° | 3.278 m | 0.237 KCAS | 0.0451 | 0.1346 |
| 395 | 10/0/0 | 0.586° | 0.575–0.694 | 0.648–0.838 | 0.086% | 0.069° | 4.205 m | 0.298 KCAS | 0.0475 | 0.0684 |

`V/U/I` sırasıyla `VALID/UNKNOWN/INFEASIBLE` altitude sayısıdır. `Min score
margin`, yalnız adayları karşılaştıran boyutsuz gözlem skorudur; derating veya
operasyonel emniyet marjı değildir.

## Altitude × CAS sonuçları

Her hücre `status / Mach / throttle-pos` biçimindedir.

| Altitude MSL | 275 KCAS | 305 KCAS | 335 KCAS | 365 KCAS | 395 KCAS |
|---:|---:|---:|---:|---:|---:|
| 1500 m | V / .453 / .435 | V / .502 / .473 | V / .551 / .508 | V / .600 / .541 | V / .648 / .575 |
| 2000 m | V / .467 / .449 | V / .517 / .486 | V / .567 / .521 | V / .617 / .555 | V / .667 / .588 |
| 2500 m | V / .481 / .463 | V / .532 / .500 | V / .584 / .534 | V / .635 / .569 | V / .686 / .602 |
| 3000 m | V / .495 / .478 | V / .548 / .514 | V / .601 / .547 | V / .653 / .583 | V / .705 / .615 |
| 3500 m | V / .510 / .487 | V / .565 / .524 | V / .619 / .560 | V / .672 / .595 | V / .726 / .627 |
| 4000 m | V / .526 / .497 | V / .582 / .534 | V / .637 / .572 | V / .692 / .607 | V / .747 / .639 |
| 4500 m | V / .542 / .507 | V / .600 / .545 | V / .656 / .585 | V / .713 / .620 | V / .768 / .652 |
| 5000 m | V / .559 / .518 | V / .618 / .559 | V / .676 / .598 | V / .734 / .633 | V / .791 / .665 |
| 5500 m | V / .577 / .530 | V / .637 / .573 | V / .697 / .612 | V / .756 / .647 | V / .814 / .675 |
| 6000 m | V / .595 / .543 | V / .657 / .588 | V / .718 / .627 | V / .779 / .662 | V / .838 / .694 |

Tüm actual CAS ortalamaları target'ın %0.086'sı içinde kaldı. Ayrıntılı TAS,
dynamic pressure, beta, load factor, surface, rate, engine ve trim diagnostikleri
`results/j3_runs.json` ve `results/j3_points.json` içindedir.

## Neden 305 KCAS?

305 KCAS bütün grid'de level trim ve measurement kriterlerini sağladı. Ölçülen
actual CAS aralığı 304.894–304.911 KCAS, TAS 167.960–207.997 m/s, Mach
0.502–0.657, abs(AoA) 1.827–1.983°, throttle position 0.473–0.588 ve dynamic
pressure 14.277–14.924 kPa oldu.

Seçim, gözlenen beş marjın en küçüğünü maksimize etti: düşük-hız schedule
uzaklığı, Mach schedule uzaklığı, iki yönlü dry-throttle reserve, control-surface
reserve ve alpha-scheduler uzaklığı. 305 KCAS'in darboğaz skoru 0.2196 ile 335'in
0.2017, 275'in 0.0996, 365'in 0.1346 ve 395'in 0.0684 değerlerinden yüksektir.
Bu nedenle düşük schedule'a fazla yaklaşmadan, yüksek Mach/throttle tarafına da
gereksiz ilerlemeden en dengeli adaydır.

## Baseline throttle policy

- Level flight: her altitude için ölçülen trim throttle kullanılır.
- Level turn: aynı altitude'daki level-trim throttle ile initialize edilir;
  gerekli maneuver throttle sonraki turn characterization'da ölçülür.
- Climb: level-trim throttle ile initialize edilir; unaugmented climb policy ve
  maximum-climb koşulu J3'te tanımlanmaz.
- Descent: level-trim throttle ile initialize edilir; lower-throttle policy ve
  idle floor J3'te tanımlanmaz.
- Afterburner bütün characterization context'inde kapalıdır/prohibited.

Throttle, generic %90 control-surface saturation kuralına bağlanmamıştır. Ham
capability veya derated capability üretilmemiştir.

## Scope ve J4 gate

J3 yalnız straight-level nominal speed selection yaptı. Climb/descent limit,
turn, coupled turn, macro primitive, lookup, planner integration ve derating
çalıştırılmadı. J4 öncesi blocker yoktur; ancak J4'ün kapsamı bu rapor tarafından
başlatılmaz veya varsayılmaz.
