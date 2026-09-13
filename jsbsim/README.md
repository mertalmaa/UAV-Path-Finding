# JSBSim offline aircraft characterization

> **U5 final freeze (2026-09-13):** F-16 artık primary planner aircraft değildir;
> J1–J5A artefact'ları yalnız methodology/regression reference olarak korunur.
> Sparse stock-aircraft re-screen sonunda primary `c172p @ 40 m/s IAS`, backup
> `DHC6 @ 60 m/s IAS` seçilmiş ve aircraft selection kapatılmıştır. Ayrıntılar
> `U5_REPORT.md` ve `results/u5_aircraft_freeze.json` içindedir.

Bu klasör, JSBSim'in çevrimdışı bir aircraft-characterization aracı olarak
kullanılacağı çalışma hattına aittir. Ana planner implementasyonu bu klasörün
kapsamında değildir.

Hedef akış:

```text
OFFLINE
JSBSim -> ölçüm -> konservatif küçük lookup

ONLINE
planner state -> lookup -> terrain/AGL validation -> successor

FINAL
trajectory -> JSBSim dynamic replay
```

Bu hattın amacı planner yolunun JSBSim trajectory'siyle yüzde yüz aynı
olmasını sağlamak değildir. Amaç, planner'ın kullandığı hareketlerin seçilmiş
operasyon zarfında uygulanabilir olmasını ve lookup'ın ölçülen kabiliyeti
iyimser biçimde aşmamasını sağlamaktır.

## Kapsam sınırı

- Tarihsel J1–J5A referans modeli: JSBSim F-16; gerçek veya
  production-calibrated aircraft değil. U0 sonrasında primary planner aircraft
  değildir.
- JSBSim, A* node expansion içinde çalıştırılmaz.
- İlk lookup sürümünde speed bir search-state bileşeni değildir.
- Heading entegrasyonuna kadar yalnız straight/vertical zarf planner tarafından
  tüketilebilir.
- AoA, throttle, load factor, surface deflection ve angular-rate geçmişleri
  planner lookup'ına taşınmaz; yalnız offline status/audit kanıtıdır.
- Ana planner dosyaları J1 kapsamında değiştirilmez.

## J1 dosyaları

- `J1_CONTRACT.md`: insan tarafından okunabilir characterization sözleşmesi.
- `characterization_contract.yaml`: aynı frozen J1 sözleşmesinin makinece
  okunabilir karşılığı.
- `DECISIONS.md`: kullanıcıyla birlikte kilitlenen kararların kaydı.

J1, F-16 methodology/regression reference'ı olarak frozen durumdadır. U0'dan
sonra yeni aircraft profile için doğrudan geçerli bir capability contract'ı
değildir.

## J2 durumu

J2 provenance ve baseline level-flight sanity gate'i tamamlandı: **PASS**.
Rapor ve yeniden üretilebilir girdiler:

- `J2_REPORT.md`
- `j2_reference_configuration.yaml`
- `j2_baseline_sanity.py`
- `test_j2_baseline.py`
- `results/j2_provenance.json`
- `results/j2_runs.json`
- `results/j2_summary.json`

## J3 durumu

J3 nominal-CAS selection ve baseline throttle-policy gate'i tamamlandı: **PASS**.
İlk 1500–6000 m domain'i için prototype fixed characterization speed **305 KCAS**
seçildi. Ayrıntılar `J3_REPORT.md` ve `results/j3_*.json` dosyalarındadır.

## J3.1 durumu

J3.1 domain extension tamamlandı: **PASS**. Reusable prototype aircraft domain'i
0–6000 m MSL'ye genişletildi; eski J3 verileri reuse edildi ve 305 KCAS kararı
korundu. 250/750/1250 m holdout'larında yerel refinement gereği çıkmadı.
Ayrıntılar `J3_1_DOMAIN_EXTENSION_REPORT.md` ve `results/j3_1_*.json` içindedir.

## J4A durumu

J4A straight climb/descent sanity gate tamamlandı: **PASS**. 305 KCAS'ta
0/3000/5000/6000 m MSL üzerinde level, +2° climb ve −2° descent metodolojisi
doğrulandı. Bu hedefler capability limitleri değildir. Ayrıntılar `J4A_REPORT.md`
ve `results/j4a_*.json` içindedir.

## J4B durumu

J4B straight climb/descent capability characterization tamamlandı: **PASS**.
305 KCAS ve afterburner-off context'inde 0–6000 m main grid boyunca 26 raw
capability boundary 0.5° genişliğinde bracketlendi. Ayrıntılar `J4B_REPORT.md`
ve `results/j4b_*.json` içindedir.

## J5A durumu

J5A level-turn methodology sanity gate tamamlandı: **PASS**. 305 KCAS'ta
0/3000/5000/6000 m MSL üzerinde straight ve `±20°` bank left/right turn
metodolojisi üç cold-start ile doğrulandı. Hedefler capability limitleri
değildir. Radius trajectory telemetry'den ölçülür; coordinated-turn theory
yalnız cross-check'tir. Ayrıntılar `J5A_REPORT.md` ve `results/j5a_*.json`
içindedir.

## U0 durumu

UAV aircraft pivot/model-selection gate tamamlandı: **PASS**. Generic TB2-like
planner profile `35–50 m/s IAS`, nominal `40 m/s`, operational bank sınırı
`±25°` ve operational vertical-speed sınırı `±5 m/s` olarak freeze edildi.
Bu değerler raw physical aircraft limitleri değildir. Birebir TB2 modeli
kurulumda yoktur ve böyle bir iddia yapılmaz. `DHC6` U1 için primary, `c182`
backup'tır. F-16 J5B çalıştırılmayacaktır.

## U1 durumu

UAV profile fail-fast model-suitability gate tamamlandı: **PASS**, fakat current
aircraft status **INCONCLUSIVE**. DHC6, frozen clean fixture'da 35 m/s
straight-level hedefini doğal biçimde sağlayamadığı için `REJECTED_FOR_PROFILE`
oldu; fail-fast gereği DHC6 turn/vertical testleri çalıştırılmadı. c182 dört
noktalı backup gate'te `BACKUP_INCONCLUSIVE` kaldı ve promising ilan edilmedi.
Full characterization/controller tuning/planner integration yapılmadı.
Ayrıntılar `U1_REPORT.md` ve `results/u1_*.json` içindedir.

## U2 durumu

Automated stock-aircraft candidate screening tamamlandı: **PASS**. Kurulu 60
modelin 49'u static metadata/configuration ile elendi; kalan 11 powered
fixed-wing aday frozen U1 controller ve 3 cold-start ile fail-fast tarandı.
Yalnız c172r ve J3Cub speed gate'i geçti, ikisi de `40 m/s IAS + 5 m/s Vz`
climb gate'inde throttle upper boundary'de kaldı. Turn gate'e ulaşan model
olmadı. PRIMARY/BACKUP seçilmedi: **NO SUITABLE STOCK JSBSIM MODEL**.
Aircraft tuning, stock XML değişikliği, planner entegrasyonu veya sonraki
mimari implementasyonu yapılmadı. Ayrıntılar `U2_REPORT.md` ve
`results/u2_*.json` içindedir.

## U3 durumu

Aircraft-driven stock selection tamamlandı: **PASS**. U2'nin exact profile
fail-fast gate'i terk edildi; 35–50 m/s, yaklaşık 25° bank ve yaklaşık 5 m/s Vz
yalnız preferred regime'dir. Reuse edilen 60-model inventory'den altı ciddi aday
aynı frozen minimal outer-loop ve üç cold-start policy ile karşılaştırıldı.
PRIMARY `c172r`, BACKUP `c172p` seçildi. LUT ve final replay aynı `c172r + U3
outer-loop + stock FCS` stack'ini kullanacaktır. Full LUT, planner ve final replay
başlatılmadı. Ayrıntılar `U3_REPORT.md` ve `results/u3_*.json` içindedir.

## U4 durumu

`c172r` raw aircraft LUT characterization tamamlandı: **PASS**. Nominal IAS
40 m/s ve 0:500:5000 m main altitude grid'de her altitude önce straight gate ile
test edildi. Straight gate 0–2500 m'de VALID; 3000–5000 m'de frozen full-rich
fixture engine'i sürdürülemediği için turn/vertical sweep fail-fast ile atlandı.
5500/6000 m straight probe'ları da INFEASIBLE oldu; bunlar true aircraft ceiling
iddiası değildir. Raw LUT 54 turn ve 54 vertical row içerir; left/right ayrıdır,
interpolation/derating/planner integration yapılmamıştır. Canonical artifact
`results/aircraft_lut_raw.json`, ayrıntılı rapor `U4_REPORT.md` içindedir.

<!-- REORDER (2026-09-13): U4.1A/.1/.2/.3 were previously appended at the
     end of this file (after U6.2.2), even though they happened
     chronologically between U4 and U5. Moved here to match actual
     chronology; no content changed. -->

## U4.1A durumu

Dar kapsamlı capability-boundary sanity tamamlandı: **PASS**. Yalnız U4'teki
existing edge'i VALID olan 1000/2000 m RIGHT-turn dalları genişletildi. 1000 m'de
+30° VALID ve +35° UNKNOWN; 2000 m'de +30° UNKNOWN bulundu. İlk non-VALID'de
duruldu ve +40° çalıştırılmadı. Hiçbir ±5 m/s vertical edge VALID olmadığı için
±6/±7 m/s probe yapılmadı. Sonuç `TURN GRID EDGE = POSSIBLY ARTIFICIAL`,
`VERTICAL GRID EDGE = INCONCLUSIVE`'dur. True physical maximum ve planner-safe
envelope çıkarılmadı; holdout validation başlatılmadı. Canonical raw LUT değişmeden
kaldı. Ayrıntılar `U4_1A_REPORT.md` ve `results/u4_1a_*.json` içindedir.

## U4.1A.1 durumu

Planner-relevant vertical capability refinement tamamlandı: **PASS**. 0/1000/
2000 m'de ±2…±5 m/s grid'i 0.5 m/s resolution'a indirildi; 24 U4 integer point
reuse edildi, 18 half-step point/54 cold-start yeni çalıştırıldı. Climb highest
tested sustainable VALID sırasıyla +2/+2.5/+2 m/s, descent ise her altitude'da
−2 m/s oldu. ±2 m/s true maximum veya hard planner limit değildir; 1000 m climb
sonucu measured capability'nin +2 ötesine uzanabildiğini gösterir. Failure'lar
power, controller ve tracking/acceptance olarak ayrıldı. Raw U4 LUT değişmedi;
planner/holdout/interpolation/derating başlatılmadı. Ayrıntılar
`U4_1A1_REPORT.md` ve `results/u4_1a1_*.json` içindedir.

## U4.1A.2 durumu

Aircraft suitability diagnostic tamamlandı: **PASS**. Vertical UNKNOWN audit'te
28 noktanın 23'ü near-target/stable, 5'i controller-limited bulundu; hiçbir status
gevşetilmedi. High-altitude baseline 2500 m VALID, 3000/3500 m INFEASIBLE sonucunu
reproduce etti. Yalnız altitude-pressure-referenced mixture handling değiştirilen
diagnostic 3000/3500/4000 m'de VALID oldu; root cause engine/mixture/propulsion
fixture'dır, service ceiling değildir. `C172R SUITABILITY = CONTINUE`, fakat
mixture policy production stack olarak onaylanıp affected raw data yenilenmeden
`READY FOR U4.1B = NO`'dur. Ayrıntılar `U4_1A2_REPORT.md` ve
`results/u4_1a2_*.json` içindedir.

## U4.1A.3 durumu

Production mixture policy freeze ve RAW LUT V2 regeneration tamamlandı:
**PASS**. Frozen rule `clip(atmosphere/P-psf / 2117.0, 0.0, 1.0)`; aynı
stateless policy LUT generation ve final replay için zorunludur. Policy 0–2500 m
command/history'sini de değiştirdiği için CASE B seçildi ve eski U4 point'i reuse
edilmeden 0:500:6000 m grid'de 221 point/663 cold-start yeniden üretildi. 13
straight gate'in tamamı VALID'dir.

5000 m straight ve turn bakımından anlamlı olsa da VALID climb yoktur; 5500 m'de
nonzero VALID vertical, 6000 m'de nonzero VALID turn/vertical yoktur. Bu nedenle
5000/5500/6000 candidate kümesinde recommended tested operational ceiling
**NONE**'dır; true service-ceiling iddiası değildir. RAW V2
`results/aircraft_lut_raw_v2.json`, ayrıntılar `U4_1A3_REPORT.md` ve
`results/u4_1a3_*.json` içindedir. U4.1B başlatılmadı; V2'nin pointwise validated
domain'i içinde ilerlemeye hazırdır.

## U5 durumu

Practical UAV-like stock-aircraft re-screen tamamlandı: **PASS**. Sekiz aday
sparse hız ve high-altitude mission screen ile karşılaştırıldı; PRIMARY
`c172p @ 40 m/s IAS`, BACKUP `DHC6 @ 60 m/s IAS` olarak freeze edildi.
Primary'nin test edilmiş usable high-altitude region'ı 5000/5500 m'dir;
6000 m'de stable straight/turn olmasına rağmen climb requirement eksiktir.
Aircraft selection kapalıdır. Full LUT, boundary search, tuning, planner
entegrasyonu ve sonraki aşamalar başlatılmadı. Ayrıntılar `U5_REPORT.md` ve
`results/u5_*.json` içindedir.

## U5.1 durumu

Existing-U5-data-first altitude/speed envelope audit tamamlandı: **PASS**.
189 point / 567 cold-start run değiştirilmeden yeniden analiz edildi; yeni
simulation çalıştırılmadı. Gerçek grid ve `NOT TESTED` hücreleri sekiz model
için görünür hale getirildi. `c172p @ 40 m/s` primary ve `DHC6 @ 60 m/s`
backup kararları **CONFIRMED**; dense 0–6000 m envelope iddiası yoktur.
1000/2000/4000 m eksikleri selection açısından kritik değildir. Aircraft
selection gerçekten frozen kalır. Ayrıntılar `U5_1_REPORT.md` ve
`results/u5_1_*.json` içindedir.

## U5.2 durumu

İki-finalist general validation tamamlandı: **PASS**. U5'ten 70 finalist
point/210 run reuse edildi; eksik nominal straight anchors, direct level-turn
radius references ve 32 representative combined climbing/descending turn için
46 point/138 yeni cold-start run çalıştırıldı. `c172p @ 40 m/s` PRIMARY,
`DHC6 @ 60 m/s` BACKUP olarak final freeze edildi. İki stack de sekiz nominal
altitude anchor'da straight-stable'dır; c172p daha küçük turn radius, daha geniş
high-alt power margin ve daha düşük complexity ile kazanır. Her iki modelin
5500/6000 m combined climbing-turn sınırlaması açıkça kaydedildi; planner-safe
combined domain henüz üretilmedi. Aircraft selection CLOSED. Ayrıntılar
`U5_2_REPORT.md` ve `results/u5_2_*.json` içindedir.

## U6A durumu

Final primary `c172p @ 40 m/s IAS` için 0:500:5500 m bağımsız CORE RAW LUT
tamamlandı: **PASS**. 19 same-stack point reuse edildi, eksik coverage için 161
point/483 cold-start run üretildi. Straight grid `12/12 VALID`; level-turn ve
straight-vertical tabloları sırasıyla 84/108 row ile tamamlandı. Canonical
`results/c172p_core_aircraft_lut_raw.json` measured turn radius ve actual Vz'yi,
strict `VALID/UNKNOWN/INFEASIBLE` semantics'ten ayrı actual-stable response'u ve
tam reuse provenance'ı korur. 5500 m straight ve level turns güçlüdür; strict
pozitif climb power/saturation nedeniyle usable değildir. Artifact raw'dır;
interpolation, holdout, derating, planner integration ve combined 3D yapılmadı.
U6B başlatılmadı. Ayrıntılar `U6A_REPORT.md` ve `results/u6a_*.json` içindedir.

## U6B durumu

Frozen `c172p @ 40 m/s` stack için representative combined-3D validation
tamamlandı: **PASS**. Primary grid `1000/2500/4000/5000/5500 m × ±20° ×
Vz ±2.5 m/s`'dir; 12 U5.2 point reuse edildi, primary eksikleri ve yalnız 1000
m `±4 m/s` severity seti için 12 point/36 cold-start run üretildi. Climbing
turn 1000/2500 m MARGINAL, 4000 m UNUSABLE, 5000 m full-throttle actual-stable
MARGINAL, 5500 m UNUSABLE'dır. Descending turn bütün gridde actual-stable ve
4000–5500 m USABLE'dır; radius level-turn reference'a yaklaşık `%±2.2` içinde
kalır. Ayrı `results/c172p_combined_3d_raw.json` raw evidence'tır; U6A LUT
değişmedi, interpolation/holdout/derating/planner integration yapılmadı.
`DIRECTLY INFEASIBLE != UNREACHABLE`; next stage U6.1'dir ve başlatılmadı.
Ayrıntılar `U6B_REPORT.md` ve `results/u6b_*.json` içindedir.

## U6.1 durumu

U6A/U6B anchor'ları arasında unseen midpoint holdout/interpolation validation
tamamlandı: **PASS**. Altı core ve dört combined holdout altitude'da 62 point /
186 cold-start run çalıştırıldı. Straight ve direction-separated `±20°`
level-turn continuous interpolation desteklenir; vertical actual-Vz error çok
düşük olsa da categorical status interpolate edilmez. Combined descent
continuous interpolation desteklenir. 4500/5250 m critical sonuçları high-alt
combined climb capability'nin conservative/discrete ele alınması gerektiğini
gösterir. 1000 m left-climb radius outlier'ı ground-track course-wrap ölçüm
hatası olarak çözüldü; raw U6B artifact değiştirilmedi. Core ve combined
validation sonucu PARTIAL; derating/planner-safe LUT/planner integration
yapılmadı. Next stage U6.2'dir ve başlatılmadı. Ayrıntılar `U6_1_REPORT.md` ve
`results/u6_1_*.json` içindedir.

## U6.2 durumu

U6A/U6B raw ve U6.1 holdout evidence'ından yeni simülasyon koşmadan
planner-safe c172p profili üretildi: **PASS**. Domain `0–5500 m @ 40 m/s IAS`;
`±20°` level-turn radius direction-separated holdout hata payıyla büyütülmüş,
`±30°` kapatılmıştır. Straight `+2 m/s` climb yalnız 0–4500 m, reviewed
`−3 m/s` descent 0–5500 m AVAILABLE'dır. Combined climb tamamen kapalı;
combined `±20°/−2.5 m/s` descent 3500–5500 m AVAILABLE'dır. Course-change wrap
hatası derived measurement pipeline'ında düzeltildi ve regression testiyle
kilitlendi; historical raw artifact'lar değişmedi. Canonical swappable artifact
`results/c172p_aircraft_profile_planner_safe.json`'dır. Production AircraftProfile
integration ve ALG-1 başlatılmadı. Ayrıntılar `U6_2_REPORT.md` ve
`results/u6_2_*.json` içindedir.

## U6.2.1 durumu

U6.2 capability profile'ın global vertical/combined simplification audit'i ve
full altitude-dependent fix'i tamamlandı: **PASS**. Yeni canonical v2, her
`0:500:5500 m` anchor'da straight, LEFT/RIGHT turn, climb, descent ve combined
measured response ile ayrı planner-safe katmanı taşır. `±30°` ve combined climb
UNAVAILABLE olsa bile measured evidence korunur. Climb/descent safe rate'leri ve
combined descent geometry artık local/altitude-dependent'tir; nominal 40 m/s IAS
yalnız query context olarak kalır. Historical source'lar değişmedi, yeni JSBSim
run yapılmadı ve production integration başlatılmadı. Ayrıntılar
`U6_2_1_REPORT.md`, `results/c172p_aircraft_profile_planner_safe_v2.json` ve
`results/u6_2_1_*.json` içindedir.

## U6.2.2 durumu

U6A'nın 12 altitude × 8 nonzero vertical command grid'inden full tested
planner-safe envelope çıkarıldı: **PASS**. Climb safe command 0–1500 m'de `+4`,
2000–3000 m'de `+3`, 3500–4500 m'de `+2 m/s`; 5000–5500 m'de UNAVAILABLE'dır.
Descent 0–2500 m'de `−3`, 3000–5000 m'de reviewed `−4`, 5500 m'de `−3 m/s`'dir.
Her seçim stability/repeatability, IAS, power reserve, saturation, controller,
status reason, holdout error ve neighbor consistency audit'ine dayanır. Straight,
turn ve combined v2 capability'leri korunmuştur. Yeni simulation ve production
integration yapılmadı. Ayrıntılar `U6_2_2_REPORT.md`, canonical
`results/c172p_aircraft_profile_planner_safe_v3.json` ve
`results/u6_2_2_*.json` içindedir.
