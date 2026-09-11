# J5A — Level Turn Sanity Gate

## Sonuç

**J5A PASS.** Validated F-16 prototype context'i, 305 KCAS ve afterburner-off
koşulunda 0/3000/5000/6000 m MSL üzerinde straight, modest left ve modest
right level-turn metodolojisi çalıştı. Toplam 12 noktanın tamamı üç bağımsız
cold-start ile `VALID` oldu; bu çalışma turn capability envelope değildir.

- Provenance: `j5a-1b8f02a2379f4eb9204e`
- Parent J4B provenance: `j4b-9fa3129fabfcda51c7a7`
- Target: straight `0°`, left `−20°`, right `+20°` bank
- 12 point × 3 cold start = 36 run
- 12 `VALID`, 0 `UNKNOWN`, 0 `INFEASIBLE`
- Final batch runtime: `4.03 s`

`±20°` bank, 20 saniyelik steady pencerede işaret, rate ve radius ölçümüne
yeterli heading değişimi üretirken maximum turn araştırmasına girmeyen modest
bir sanity hedefi olarak seçildi. Bu değer F-16 operational/capability limiti
değildir.

## Controller architecture

```text
target bank
  -> bank PI outer loop
  -> fcs/aileron-cmd-norm
  -> native F-16 roll-rate FCS
  -> ailerons/flaperons -> aircraft

target beta = 0
  -> beta proportional outer loop
  -> fcs/rudder-cmd-norm
  -> native F-16 yaw-rate/yaw-load FCS
  -> rudder -> aircraft

target gamma = 0
  -> J4B gamma PI
  -> fcs/elevator-cmd-norm
  -> native pitch FCS -> aircraft

target CAS = 305 KCAS
  -> J4B CAS PI
  -> fcs/throttle-cmd-norm
  -> dry engine command -> aircraft
```

Bank command edilir; heading rate command edilmez. Rudder surface doğrudan
dayatılmaz: beta=0 outer-loop yalnız pilot-equivalent rudder command üretir ve
native yaw FCS aktif kalır. Bütün surface position'lar model/FCS çıktısıdır.
`fcs/fbw-override=0`; afterburner kapalıdır.

## Turn sonuçları

Radius, 20 saniyelik measurement penceresindeki local north/east trajectory
arc length'inin unwrapped ground-track açısına bölünmesiyle ölçüldü. Theory
değeri actual TAS ve actual bank ile `V²/(g tan|phi|)` formülünden yalnız sanity
cross-check olarak hesaplandı.

| Alt MSL | Yön | Bank | Turn rate | Ölçülen radius | Theory radius | Fark | Beta | Nz | Throttle |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 m | LEFT | −20.148° | −1.1999°/s | 7492.6 m | 6839.3 m | 9.55% | −0.486° | 1.0524 | 0.4488 |
| 0 m | RIGHT | +20.138° | +1.1992°/s | 7491.4 m | 6837.0 m | 9.57% | +0.485° | 1.0511 | 0.4153 |
| 3000 m | LEFT | −20.165° | −1.0617°/s | 9716.6 m | 9007.0 m | 7.88% | −0.414° | 1.0532 | 0.5257 |
| 3000 m | RIGHT | +20.160° | +1.0608°/s | 9717.2 m | 9002.3 m | 7.94% | +0.413° | 1.0519 | 0.4992 |
| 5000 m | LEFT | −20.221° | −0.9736°/s | 11649.2 m | 10865.9 m | 7.21% | −0.378° | 1.0536 | 0.5666 |
| 5000 m | RIGHT | +20.217° | +0.9725°/s | 11653.4 m | 10860.5 m | 7.30% | +0.377° | 1.0522 | 0.5488 |
| 6000 m | LEFT | −20.320° | −0.9290°/s | 12808.1 m | 11909.8 m | 7.54% | −0.379° | 1.0535 | 0.5900 |
| 6000 m | RIGHT | +20.319° | +0.9278°/s | 12817.0 m | 11903.9 m | 7.67% | +0.378° | 1.0520 | 0.5841 |

Straight controls dört irtifada da `VALID` oldu: bank ve turn rate numerical
zero seviyesinde, CAS 305.000 KCAS, gamma `0.0008–0.0013°`, throttle position
`0.4341–0.5876` aralığındadır.

## Tracking, coordination ve symmetry

- Maximum CAS relative error: `0.0253%` (`±2%` gate'in çok içinde).
- Maximum gamma absolute error: `0.2925°` (`±0.5°` gate içinde).
- Maximum bank absolute error: `0.5736°` (`±1°` sanity gate içinde).
- Turn measurement mean gamma: `−0.2424°` ile `−0.1546°`; 20 s altitude
  delta `−13.55` ile `−8.80 m` arasında.
- Turn-rate stability relative deviation tüm turnlerde `%5` sınırının içinde.
- Theory-radius farkı `7.21–9.57%`; cross-check'in `%10` eşiği içinde.
- Beta `|0.378–0.486°|`, sol/sağ zıt işaretli ve kararlı. Bu bir operational
  beta limiti iddiası değildir.
- Left/right relative magnitude difference: radius en çok `0.0694%`, turn rate
  en çok `0.1323%`, bank en çok `0.0531%`, load factor en çok `0.1435%`.
- Throttle left/right farkı en çok `7.76%` (0 m); her iki taraf da CAS tracking
  gate'ini geçti. Fark altitude ile azalıp 6000 m'de `1.02%` oldu.
- Measurement penceresinde hiçbir outer-loop command saturation olmadı.
  Maximum physical control-surface usage `0.0731`, 0.90 sınırından uzaktır.

Beta controller'ın normalized rudder command'i steady turnlerde yüksek olsa da
native FCS sonrasında physical rudder/surface kullanımı düşük kaldı ve measurement
clamp oluşmadı. Daha yüksek-bank J5B aramasında bu controller authority ayrıca
izlenmelidir; command clamp'a dayanan bir failure aircraft inability olarak
sınıflandırılmamalıdır.

## Repeatability, status ve scope

Her altitude × maneuver noktası üç fresh JSBSim instance ile tekrarlandı.
Maximum cold-start relative deviation `1.60e-16`; 12/12 repeatability gate PASS.

Unreliable modest-target sonucu varsayılan olarak `UNKNOWN` olur. `INFEASIBLE`
yalnız ayrıca güvenilir aircraft inability kanıtı varsa mümkündür; J5A'da limit
aranmadığı için böyle bir iddia üretilmedi.

Full turn sweep, minimum-radius/maximum-rate capability, climbing/descending
turn, macro primitive, lookup, derating, 3D safety box ve planner integration
yapılmadı. Speed planner state'ine eklenmedi. J5B öncesi blocker yok; J5B
başlatılmadı.
