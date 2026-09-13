# JSBSim characterization karar kaydı

Bu dosya yalnız kullanıcıyla açıkça kararlaştırılmış seçimleri ve açık karar
noktalarını kaydeder. Öneriler otomatik olarak karar sayılmaz.

## Kilitlenmiş kararlar

| ID | Karar | Gerekçe |
|---|---|---|
| D-001 | JSBSim yalnız offline characterization ve final replay'de kullanılır. | Node expansion maliyetini ve simulator bağımlılığını search dışında tutmak. |
| D-002 | **U0 / D-017 ile superseded.** Tarihsel J1–J5A referans aircraft F-16'dır fakat production-calibrated gerçek aircraft kabul edilmez. | Prototype scope; artık primary planner aircraft kararı değildir. |
| D-003 | İlk sürümde speed search state'ine eklenmez. | State explosion'ı önlemek. |
| D-004 | Hard maximum altitude 6000 m MSL'dir. | Kullanıcı gereksinimi. |
| D-005 | Minimum altitude terrain + 100 m AGL'dir. | Kullanıcı gereksinimi; terrain validator boyunca uygulanır. |
| D-006 | Yüzde yüz trajectory/dynamic equality gerekmez. | Amaç konservatif kinematik uygulanabilirliktir. |
| D-007 | Ana planner J1 kapsamında değiştirilmez. | Characterization ve integration çalışma hatlarını ayırmak. |
| D-008 | Ana altitude spacing 500 m; 250 m midpoint holdout; non-conservative bulunan aralıkta yerel 250 m refinement. | Küçük lookup'ı korurken band içindeki iyimser temsili bağımsız noktada yakalamak. |
| D-009 | Provisional prototype acceptance profile: 3 repeat, 20 s steady window, 30 s maximum settling/replay allowance, CAS ±%2, gamma ±0.5°, sustained curvature ±%5, control surface ≤%90, temel metrik repeatability yaklaşık ±%1. | Exact trajectory eşitliği aramadan güvenilir ve tekrarlanabilir ölçüm üretmek. |
| D-010 | J1'de sabit capability derating yoktur; raw ve ilerideki derated capability ayrı tutulur, derating J3/J4 verileri sonrasında kararlaştırılır. | Veri görülmeden keyfî `%20` veya başka sabit margin kilitlememek. |
| D-013 | Characterization sonucu binary değildir: `VALID`, `INFEASIBLE`, `UNKNOWN`. `UNKNOWN`, infeasible sayılamaz ve pruning'de kullanılamaz. | Ölçüm/tolerans problemlerinin erişilebilir maneuver'ları yanlış budamasını önlemek. |
| D-011 | **J3.1 ile superseded.** İlk supported domain mevcut 10×10 km prototype ROI'nin gerçek DEM minimumundan hesaplandı: terrain minimum `1697.698364 m`, minimum required `1797.698364 m`, floor `1500 m`, ceiling `6000 m` MSL. Eski ölçümler korunur. | İlk prototype region'in ihtiyaç duyduğu altitude aralığını kapsayan tarihsel karar. |
| D-012 | Straight level/climb/descent yalnız first validation gate'tir. Gate PASS sonrası straight, left/right level turns ve left/right coupled climb/descent turns aynı harness ile full batch characterize edilir; macro roll-in/arc/roll-out ayrı kalır. | Önce measurement harness'i küçük kapsamda kanıtlamak, sonra final scope'u eksik bırakmadan batch çalışmak. |
| D-014 | `INFEASIBLE` exact capability limiti değildir; komşu `VALID` ve `INFEASIBLE` hedefler sınırı bracket eder, sonra sweep/refinement ile daraltılır. `UNKNOWN` boundary kanıtı değildir. | Ölçüm noktası ile gerçek envelope sınırını birbirine karıştırmamak. |
| D-015 | Reusable prototype aircraft-characterization domain'i `[0, 6000] m MSL`'ye genişletildi; 500 m main grid ve 250 m midpoint policy değişmedi. Önceki `[1500, 6000] m` verileri yeniden koşulmadan korunur. Bu bir gerçek F-16 operational-envelope iddiası değildir ve online `terrain + 100 m AGL` gate'ini değiştirmez. | Future planning regions may contain substantially lower terrain; characterization cost is still small at this stage, so the reusable prototype aircraft domain is extended to 0–6000 m before maneuver envelope generation begins. |
| D-016 | J3.1 birleşik 0–6000 m main-grid değerlendirmesinde 305 KCAS nominal hız olarak yeniden doğrulandı; holdout'lar selection/fitting'e katılmadı. | Mevcut J3 margin-score formülü ve tie-breaker'ları değiştirilmeden 305 KCAS en yüksek minimum marjı korudu. |

## Açık J1 kararları

Yok. Derating değeri ve maneuver-specific throttle policy'leri bilinçli olarak
sonraki characterization kararlarına bırakılmıştır;
bunlar kapanmamış J1 kararı değildir.

## U0 aircraft pivot kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-017 | D-002 primary-aircraft kısmı superseded: F-16 primary planner aircraft rolünden çıkarıldı; J1–J5A yalnız methodology/regression reference olarak korunur. | Yeni planner düşük hızlı generic fixed-wing UAV profile kullanacaktır. |
| D-018 | F-16 J5B ve yeni F-16 capability sweep'leri cancelled/frozen'dır. | Artık planner'a taşınmayacak aircraft context'inde yeni envelope maliyeti üretilmemesi. |
| D-019 | Generic TB2-like operational profile: IAS 35–50 m/s, nominal IAS 40 m/s, planner bank `|phi| <= 25°`, planner vertical speed `|Vz| <= 5 m/s`. Bunlar raw physical aircraft limitleri değil operational planner constraint'leridir. | Planner davranış sınırını aircraft raw capability iddiasından ayırmak. |
| D-020 | Kurulu model setinde birebir TB2 yoktur; exact TB2 model/capability claim yapılmaz ve başka model TB2 diye yeniden adlandırılmaz. | Provenance ve model doğruluğunu korumak. |
| D-021 | U1 minimal sanity için primary `DHC6`, backup `c182`'dir. Seçim capability PASS anlamına gelmez. | DHC6 low-speed/STOL ve turboprop dikey marj dengesi; c182 daha basit düşük-hız fallback'i. |
| D-022 | Sonraki adım yalnız U1 minimal sanity validation'dır; controller tuning, full characterization, planner/heading/orbit implementasyonu değildir. | Model uygunluğunu küçük ve fail-fast bir gate ile ölçmek. |
| D-023 | U1'de DHC6 `REJECTED_FOR_PROFILE`: frozen clean fixture'da 35 m/s straight-level point'i üç tekrarda stabil/sustained tutulamadı. Bu raw aircraft minimum-speed claim'i değildir. | Built-in trim failure ve bounded generic controller altında throttle lower-bound ile tekrarlanabilir IAS/Vz miss birlikte model/fixture mismatch gösterdi. |
| D-024 | c182 dört-point backup gate sonucu `BACKUP_INCONCLUSIVE`; current aircraft status `INCONCLUSIVE`. | Straight/turn noktalarında tracking gate'leri temiz kapanmadı; 40 m/s +5 m/s climb point'i throttle 1.0'da IAS 37.5 m/s'ye düştü. |
| D-025 | U1 PASS yalnız model-selection fail-fast sürecinin doğru işletildiğini ifade eder; kabul edilmiş aircraft veya full characterization anlamına gelmez. | DHC6 objektifçe elendi, backup gate çalıştı ve kapsam sınırları korundu. |

## U2 stock candidate screening kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-026 | Yerel 60-model stock inventory için static filter + 40/35 m/s speed + 40 m/s/+5 m/s climb + 35 m/s/±25° turn fail-fast sırası U2 screening contract'ıdır. Frozen U1 controller gain'leri model başına değiştirilmez. | Modeli profile zorla tune etmeden, ucuz ve sistematik aday discovery yapmak. |
| D-027 | U2 sonucu `NO SUITABLE STOCK JSBSIM MODEL`; PRIMARY/BACKUP atanmaz. Bu U2 için PASS'tir. | 49 model static elendi; 11 dynamic adaydan yalnız c172r/J3Cub speed gate'i geçti ve ikisi de throttle 1.0'da simultaneous climb gate'i geçemedi; turn'e ulaşan olmadı. |
| D-028 | Operational profile `35–50 m/s IAS`, nominal `40 m/s`, `|phi| <= 25°`, `|Vz| <= 5 m/s` değişmedi ve raw physical aircraft limitleri olarak yorumlanmaz. Sonraki mimari U2 içinde uygulanmaz. | Negatif stock-model sonucunu yeni profile, tuning'e veya sessiz planner değişikliğine dönüştürmemek. |

## U3 aircraft-driven selection kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-029 | D-026/D-027'nin exact fail-fast seçim yaklaşımı superseded: 35–50 m/s, yaklaşık 25° bank ve yaklaşık 5 m/s Vz artık preference'dır, hard aircraft-selection gate değildir. | Planner seçilen aircraft'ın ölçülmüş gerçek envelope'una uyacak; iyi bir modeli tek exact target yüzünden elememek. |
| D-030 | PRIMARY `c172r`, BACKUP `c172p`'dir. | Altı adayın aynı 24-point/72-repeat comparison'ında minimum tuning, temiz turn/vertical tracking, düşük surface usage ve repeatability bakımından en güçlü iki stock kombinasyon bunlardır. |
| D-031 | LUT ve final replay stack'i aynıdır: planner maneuver command → frozen U3 minimal IAS/Vz/bank/β outer loop → selected aircraft stock FCS → JSBSim dynamics. | Controller değişiminden doğan LUT/replay tutarsızlığını önlemek. |
| D-032 | İlk planner state hedefi `(x,y,z,heading)`; speed ilk LUT/planner sürümünde fixed nominal aircraft context'idir. | İlk heading-aware planner'da state explosion'ı sınırlamak ve aircraft working context'ini açık tutmak. |
| D-033 | Custom aircraft ve ArduPilot şimdilik kullanılmaz; stock aerodynamic/propulsion/mass/inertia XML'i profile uydurmak için tune edilmez. | Minimum-hack, açık provenance ve deterministic stock-model contract'ını korumak. |

## U4 raw c172r LUT kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-034 | U4 nominal speed context'i 40 m/s IAS, main altitude grid'i `0:500:5000 m`, optional straight probes 5500/6000 m'dir. Speed state dimension değildir. | Düşük-performanslı piston modelinde altitude dependence'i 500 m çözünürlükte ölçmek. |
| D-035 | Raw turn grid `0,±10,±15,±20,±25°`; raw vertical grid `-5,-4,-3,-2,0,+2,+3,+4,+5 m/s` olarak ayrı left/right ve climb/descent row'larıyla saklanır. | Symmetry veya capability compression yapmadan ölçülmüş raw response'u korumak. |
| D-036 | Straight gate 0–2500 m'de VALID; 3000–5000 m ve 5500/6000 probe'larında frozen full-rich stack engine'i sürdürülemedi. Bu exact frozen-stack sonucu true c172r ceiling değildir. | Failed trim object'i atılıp fresh untrimmed retry yapılmasına rağmen engine state/RPM/thrust üç tekrarda da sıfıra indi. |
| D-037 | Canonical `results/aircraft_lut_raw.json` yalnız raw tested-grid verisidir; interpolation, derating ve planner readiness iddiası yoktur. | U4.1 holdout ve U4.2 safe-derating aşamalarını atlamamak. |
| D-038 | U4.1 holdout planı `250,750,...,4750 m` independent JSBSim measurement ile main-grid interpolation error ölçümüdür; U4 içinde çalıştırılmaz. | Interpolation'ı fitting noktalarında değil bağımsız altitude midpoint'lerinde doğrulamak. |
| D-039 | U3 same-stack contract kalıcıdır: LUT ve final replay frozen U3 outer-loop + c172r stock FCS kullanır. | Characterization/replay controller mismatch'ini önlemek. |

## U4.1A boundary sanity kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-040 | Mentorun yaklaşık 35–50 m/s IAS, 25° bank ve 5 m/s Vz değerleri preferred operating region'dır; measured capability veya hard physical maximum değildir. Characterization envelope ile planner-safe envelope ayrı tutulur. | Operating preference'ı aircraft capability kanıtıyla karıştırmamak. |
| D-041 | U4.1A sonucu `TURN GRID EDGE = POSSIBLY ARTIFICIAL`: 1000 m RIGHT dalında +25° ve +30° VALID, +35° UNKNOWN; 2000 m RIGHT dalında +25° VALID, +30° UNKNOWN. LEFT/non-VALID edge'ler genişletilmez. | En az bir ölçüm capability'nin +25° grid dışına uzandığını gösterir; UNKNOWN true boundary değildir. |
| D-042 | U4.1A sonucu `VERTICAL GRID EDGE = INCONCLUSIVE`: +5 m/s climb edge'leri INFEASIBLE, −5 m/s descent edge'leri UNKNOWN olduğu için ±6/±7 m/s probe yapılmaz. | Outward exploration yalnız existing signed edge VALID ise açılır; UNKNOWN, INFEASIBLE sayılmaz. |
| D-043 | U4.1A ayrı artifact'tır; canonical U4 raw LUT immutable kalır. Bu stage true physical maximum, interpolation, derating veya planner-safe limit çıkarmaz. | Historical raw characterization ile boundary sanity gözlemlerinin provenance'ını ayırmak. |

## U4.1A.1 vertical refinement kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-044 | 0/1000/2000 m'de climb ve descent `2.0:0.5:5.0 m/s` magnitude grid'iyle ayrı refine edilir; U4 ile aynı contract'taki integer point'ler reuse edilir. | ±2 VALID ile ±5 non-VALID arasındaki planner-relevant capability boşluğunu duplicate koşu olmadan çözmek. |
| D-045 | Highest tested sustainable VALID climb 0/1000/2000 m'de `+2/+2.5/+2 m/s`; largest tested sustainable VALID descent her üç altitude'da `−2 m/s`'dir. Bunlar true maxima veya planner-safe hard limit değildir. | 1000 m climb +2.5 VALID iken diğer +2.5 ve bütün −2.5 sonuçları UNKNOWN'dır; altitude ve direction asymmetry korunmalıdır. |
| D-046 | Non-VALID vertical sonuçlar `POWER_LIMITED`, `CONTROLLER_LIMITED`, `TRACKING_ACCEPTANCE_LIMITED`, `AERO_STABILITY_LIMITED`, `OTHER_DIAGNOSTIC` olarak ayrılır. UNKNOWN hiçbir zaman INFEASIBLE veya physical aircraft boundary sayılmaz. | Yakın/repeatable actual response ile gerçek throttle/command-clamp kanıtını birbirine karıştırmamak. |
| D-047 | 40 m/s nominal context'teki `D = 40 × 100 / |actual Vz|` metriği yalnız path-planning etkisini yorumlar; planner'a bağlanmaz. U4 raw LUT immutable kalır. | Vertical capability'nin yatay mesafe maliyetini görünür kılarken integration/derating kararını sonraki stage'lere bırakmak. |

## U4.1A.2 suitability diagnostic kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-048 | 28 vertical UNKNOWN'un 23'ü `NEAR_TARGET_STABLE`, 5'i `CONTROLLER_LIMITED` diagnostic sınıfındadır; hiçbiri production VALID'e yükseltilmez. | Üç-repeat telemetry actual Vz'nin target'a yakın ve IAS'ın preferred range içinde kaldığını gösterse de U1 max-over-window acceptance contract'ı değişmemiştir. |
| D-049 | U1 acceptance eşikleri physical aircraft boundary veya planner-primitive usability tolerance'ı değildir; U4.1A.2'de gevşetilmez. | IAS/Vz/bank max instantaneous error ve continuous-settling gate'leri provisional model-suitability testinden gelir; diagnostic interpretation status contract'ı değiştirmez. |
| D-050 | 3000 m+ frozen-baseline failure primary root cause'u `ENGINE / MIXTURE / PROPULSION FIXTURE`'dır. Local c172r'de automatic altitude-compensated mixture yoktur; pressure-ratio mixture-only diagnostic 3000/3500/4000 m'de aynı controller ile VALID olmuştur. | Fresh untrimmed fallback baseline'da engine yine dururken yalnız mixture handling değişimi RPM/thrust/fuel flow ve straight tracking'i geri getirmiştir; trim/controller/aircraft-limit açıklamaları primary cause değildir. |
| D-051 | `C172R SUITABILITY = CONTINUE`; aircraft otomatik değiştirilmez. `READY FOR U4.1B = NO`: önce mixture handling production stack için açıkça onaylanıp freeze edilmeli ve etkilenen 3000 m+ raw characterization yenilenmelidir. | Diagnostic configuration production'a sessizce taşınamaz; mevcut canonical U4 raw LUT 3000 m+ interpolation için hâlâ geçerli main-grid data içermez. |
| D-052 | U4/U4.1A/U4.1A.1 artifacts historical ve immutable kalır; U4.1A.2 ayrı diagnostic artifacts kullanır. True service ceiling iddiası yapılmaz. | Baseline provenance'ı korumak ve diagnostic kanıtı production capability verisinden ayırmak. |

<!-- CATCH-UP (2026-09-13 repo audit): D-053 - D-065 below were missing --
     README.md and project.md already recorded U4.1A.3 through U6.2.2 as
     PASS, including the U5 aircraft freeze itself, but no corresponding
     rows existed in this decision log. Added from the existing README.md/
     *_REPORT.md record; no new decisions made, no existing ones changed. -->

## U4.1A.3 production mixture policy kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-053 | Production mixture policy frozen: `clip(atmosphere/P-psf / 2117.0, 0.0, 1.0)`. Aynı stateless policy LUT generation ve final replay için zorunludur. | D-050'nin bulduğu 3000 m+ engine/mixture/propulsion fixture kök nedenini production stack için resmen kapatmak. |
| D-054 | RAW LUT V2 (`results/aircraft_lut_raw_v2.json`) canonical raw artifact'tır; eski V1 (`results/aircraft_lut_raw.json`) tarihsel/immutable kalır. 5000/5500/6000 m'de recommended tested operational ceiling `NONE`'dır; true service-ceiling iddiası değildir. | Policy 0-2500 m command/history'sini de değiştirdiği için CASE B seçildi; 221 point/663 cold-start eski point reuse edilmeden yeniden üretildi. |

## U5 practical re-screen kararları (final aircraft freeze)

| ID | Karar | Gerekçe |
|---|---|---|
| D-055 | Aircraft selection **CLOSED**: PRIMARY `c172p @ 40 m/s IAS`, BACKUP `DHC6 @ 60 m/s IAS`. | Sekiz aday sparse hız ve high-altitude mission screen ile karşılaştırıldı; sonuçlar bu ikisini seçti. |
| D-056 | Primary'nin test edilmiş usable high-altitude region'ı 5000/5500 m'dir; 6000 m'de stable straight/turn olsa da climb requirement eksik olduğu için dahil değildir. | U5 sparse screen sonucu; full LUT/boundary search/tuning/planner entegrasyonu bu stage'de başlatılmadı. |

## U5.1 envelope audit kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-057 | D-055/D-056 **CONFIRMED**: c172p @ 40 m/s primary, DHC6 @ 60 m/s backup, yeni simulation koşulmadan. | 189 point/567 cold-start run existing-data-first yeniden analiz edildi; dense 0-6000 m envelope iddiası yoktur, 1000/2000/4000 m eksikleri selection açısından kritik değildir. |

## U5.2 finalist validation kararları (final freeze teyidi)

| ID | Karar | Gerekçe |
|---|---|---|
| D-058 | Aircraft selection **FINAL FREEZE**, aynı seçimle teyit edildi: c172p @ 40 m/s PRIMARY, DHC6 @ 60 m/s BACKUP. | İki-finalist general validation (70 finalist point/210 run reuse + 46 point/138 yeni run); c172p daha küçük turn radius, daha geniş high-alt power margin, daha düşük complexity ile kazanır. |
| D-059 | Her iki modelin 5500/6000 m combined climbing-turn sınırlaması açıkça kaydedildi; planner-safe combined domain bu stage'de üretilmedi. | Sekiz nominal altitude anchor'da straight-stable doğrulandı, combined domain sonraki stage'lere bırakıldı. |

## U6A core raw LUT kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-060 | Frozen primary `c172p @ 40 m/s IAS` için bağımsız CORE RAW LUT canonical artifact'ı: `results/c172p_core_aircraft_lut_raw.json`. Measured turn radius/actual Vz, strict `VALID/UNKNOWN/INFEASIBLE` semantics'ten ayrı tutulur. | 0:500:5500 m grid'de straight `12/12 VALID`; interpolation/holdout/derating/planner integration bu stage'de yapılmadı. |

## U6B combined-3D kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-061 | Combined-3D raw evidence ayrı artifact'ta tutulur: `results/c172p_combined_3d_raw.json`; U6A LUT değişmez. Climbing turn 4000/5500 m UNUSABLE; descending turn 4000-5500 m USABLE'dır. | Frozen c172p @ 40 m/s stack için representative `1000/2500/4000/5000/5500 m × ±20° × Vz ±2.5 m/s` grid testi; `DIRECTLY INFEASIBLE != UNREACHABLE` ilkesi korunur. |

## U6.1 holdout/interpolation kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-062 | Core ve combined validation sonucu **PARTIAL**: straight ve direction-separated `±20°` level-turn continuous interpolation desteklenir; vertical categorical status interpolate edilmez. 4500/5250 m'de high-alt combined climb capability conservative/discrete ele alınmalıdır. | Altı core ve dört combined holdout altitude'da 62 point/186 cold-start run; derating/planner-safe LUT/planner integration bu stage'de yapılmadı. |

## U6.2 planner-safe profile kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-063 | Planner-safe c172p profili üretildi (yeni simulation koşulmadan), canonical `results/c172p_aircraft_profile_planner_safe.json`. Domain `0-5500 m @ 40 m/s IAS`; `±20°` level-turn AVAILABLE, `±30°` kapatıldı; straight `+2 m/s` climb 0-4500 m, reviewed `-3 m/s` descent 0-5500 m; combined climb tamamen kapalı, combined `±20°/-2.5 m/s` descent 3500-5500 m AVAILABLE. | U6A/U6B raw ve U6.1 holdout evidence'ından derive edildi; course-change wrap hatası düzeltilip regression testiyle kilitlendi, historical raw artifact'lar değişmedi. |

## U6.2.1 full altitude-dependent kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-064 | Planner-safe profil v2 canonical: `results/c172p_aircraft_profile_planner_safe_v2.json`. Her `0:500:5500 m` anchor'da straight/LEFT-RIGHT turn/climb/descent/combined ayrı, altitude-dependent measured response taşır; `±30°` ve combined climb UNAVAILABLE olsa bile measured evidence korunur. | U6.2'nin global vertical/combined simplification'ının audit'i; yeni JSBSim run yapılmadı, historical source'lar değişmedi. |

## U6.2.2 tested envelope kararları

| ID | Karar | Gerekçe |
|---|---|---|
| D-065 | Full tested planner-safe envelope v3 canonical: `results/c172p_aircraft_profile_planner_safe_v3.json`. Climb safe command 0-1500 m `+4`, 2000-3000 m `+3`, 3500-4500 m `+2 m/s`, 5000-5500 m UNAVAILABLE; descent 0-2500 m `-3`, 3000-5000 m reviewed `-4`, 5500 m `-3 m/s`. | U6A'nın 12 altitude × 8 nonzero vertical command grid'inden extract edildi; her seçim stability/repeatability, IAS, power reserve, saturation, controller, holdout error ve neighbor consistency audit'ine dayanır. Straight/turn/combined v2 capability'leri korunmuştur. |
