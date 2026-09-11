# J4A — Straight Climb / Descent Sanity Gate

## Sonuç

**J4A PASS.** 305 KCAS'ta level, modest climb ve modest descent metodolojisi
0/3000/5000/6000 m MSL representative altitude'larda güvenilir ve tekrarlanabilir
çalıştı.

- Provenance: `j4a-2342f25aec9f6ff3314c`
- Parent J3.1 provenance: `j3.1-7761df15481027018aac`
- 4 altitude × 3 maneuver × 3 cold start = 36 run
- 36/36 run ve 12/12 characterization point `VALID`
- `UNKNOWN`: 0; `INFEASIBLE`: 0

## Sanity gamma target seçimi

- Level control: `0°`
- Modest climb: `+2°`
- Modest descent: `−2°`

Simetrik ±2° hedefler altitude, vertical-speed ve throttle işaretlerini görünür
kılacak kadar büyük; modelin vertical capability boundary'sini aramayacak kadar
küçük seçildi. Bunlar gerçek F-16 limitleri veya capability sonuçları değildir.

Her run fresh instance ve 0° level trim ile başladı. Target gamma 3 s'de ramp
edildi. Native FCS korunurken, yalnız test harness'ına ait küçük gamma/CAS outer
loop'u elevator command ve dry-throttle command üretti. Replay 30 s, measurement
window son 20 s'dir. Bu controller planner veya final capability policy değildir.

## Representative-altitude sonuçları

| Alt | Maneuver | Target γ | Actual γ | Max γ err | Actual CAS | Max CAS err | V/S | Δalt/20s | Throttle | AoA | Pitch | Settled |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 m | level | 0° | 0.0001° | 0.0004° | 305.001 | 0.0006% | 0.000 m/s | +0.006 m | 0.4341 | 1.793° | 1.793° | 0.008 s |
| 0 m | climb | +2° | 2.0338° | 0.1049° | 302.878 | 1.1556% | +5.554 m/s | +111.086 m | 0.4969 | 1.830° | 3.864° | 3.317 s |
| 0 m | descent | −2° | −2.0298° | 0.0954° | 307.570 | 1.2269% | −5.579 m/s | −111.582 m | 0.3633 | 1.741° | −0.288° | 3.275 s |
| 3000 m | level | 0° | 0.0001° | 0.0005° | 305.002 | 0.0008% | 0.000 m/s | +0.008 m | 0.5137 | 1.864° | 1.864° | 0.008 s |
| 3000 m | climb | +2° | 2.0334° | 0.1159° | 302.423 | 1.2393% | +6.373 m/s | +127.467 m | 0.5844 | 1.911° | 3.944° | 3.408 s |
| 3000 m | descent | −2° | −2.0287° | 0.1023° | 308.011 | 1.3071% | −6.405 m/s | −128.092 m | 0.4357 | 1.803° | −0.226° | 3.358 s |
| 5000 m | level | 0° | 0.0001° | 0.0005° | 305.003 | 0.0010% | +0.001 m/s | +0.010 m | 0.5586 | 1.933° | 1.933° | 0.008 s |
| 5000 m | climb | +2° | 2.0338° | 0.1201° | 301.982 | 1.3138% | +7.008 m/s | +140.168 m | 0.6364 | 1.991° | 4.024° | 3.483 s |
| 5000 m | descent | −2° | −2.0281° | 0.1014° | 308.414 | 1.3797% | −7.046 m/s | −140.925 m | 0.4743 | 1.863° | −0.165° | 3.425 s |
| 6000 m | level | 0° | 0.0002° | 0.0006° | 305.002 | 0.0010% | +0.001 m/s | +0.011 m | 0.5876 | 1.979° | 1.979° | 0.008 s |
| 6000 m | climb | +2° | 2.0320° | 0.1165° | 301.778 | 1.3600% | +7.350 m/s | +146.996 m | 0.6690 | 2.042° | 4.074° | 3.500 s |
| 6000 m | descent | −2° | −2.0261° | 0.0952° | 308.630 | 1.4259% | −7.391 m/s | −147.820 m | 0.4998 | 1.903° | −0.123° | 3.442 s |

## Tracking, throttle ve sign gate

- Worst CAS tracking error: `1.4259%` < `2%`.
- Worst gamma tracking error: `0.1201°` < `0.5°`.
- En geç settling: `3.500 s`; 10 s'de başlayan 20 s window'dan önce.
- Throttle ordering her altitude'da `descent < level < climb`.
- En yüksek throttle position: `0.68185`; afterburner kullanılmadı.
- En yüksek normalized surface usage: `0.04884`; `%90` sınırından uzak.
- Outer-loop elevator veya throttle command saturation: 0 run.
- Positive gamma: positive vertical speed ve artan altitude — 12/12 climb run.
- Negative gamma: negative vertical speed ve azalan altitude — 12/12 descent run.
- Zero gamma: level davranış — 12/12 level run.

## Repeatability ve status semantics

Üç cold-start tekrarı arasındaki en büyük relative deviation `1.28e-16`; J1'in
yaklaşık `%1` kriterinden küçüktür. Tracking, controller, trim, settling veya
numerical failure fiziksel sınır kanıtı sayılmaz ve `UNKNOWN` üretecek şekilde
korunmuştur. J4A capability-limit search yapmadığından hiçbir sonuç otomatik
`INFEASIBLE` sınıflandırılmaz.

## Scope

Full altitude/gamma sweep, maximum climb/descent search, turn, coupled maneuver,
lookup, derating ve planner integration yapılmadı. Raw/derated capability alanları
boştur. J4B öncesi blocker yoktur; J4B başlatılmamıştır.

