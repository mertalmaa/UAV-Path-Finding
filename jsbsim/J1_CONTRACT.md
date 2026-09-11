# J1 — Aircraft characterization contract

Durum: **J1 FROZEN — J3.1 domain amendment işlendi**

## 1. Amaç

JSBSim F-16 referans modelinden, terrain-aware planner'ın kullanabileceği küçük
ve konservatif bir kinematik lookup üretmek için girişleri, çıkışları, kalite
kapılarını ve kapsam dışını sabitlemek.

J1 ölçüm yapmaz ve aircraft performans değeri üretmez. J1 yalnızca ilerideki
ölçümlerin aynı anlama gelmesini sağlayan sözleşmedir.

## 2. Fidelity hedefi

Hedef **trajectory equality** değildir. Üç ayrı doğruluk seviyesi tanımlanır:

1. **Hard safety correctness:** Lookup ölçülmüş sürdürülebilir kabiliyeti
   iyimser göstermemelidir. AGL, terrain, NoData ve 6000 m MSL ceiling ayrıca
   online hard gate olarak kalır.
2. **Kinematic usefulness:** Lookup; climb/descent eğimi, curvature ve primitive
   geometry'yi planner çözünürlüğü için yeterli doğrulukla temsil etmelidir.
3. **Dynamic plausibility:** Lookup'tan üretilen bir hareket JSBSim replay'de
   kararlı biçimde uygulanabilmelidir. Birebir zaman geçmişi eşleşmesi aranmaz.

Exact trajectory eşleşmesi aranmaz ve capability overprediction yasaktır.
Ancak güvenilir biçimde sınıflandırılamayan bir testin konservatif olduğu
gerekçesiyle otomatik olarak uygulanamaz sayılmasına da izin verilmez; böyle
bir sonuç `UNKNOWN` olur.

## 3. Sabit prototype varsayımları

| Konu | J1 kararı |
|---|---|
| Reference aircraft | JSBSim F-16 referans modeli |
| Model iddiası | Prototype reference; gerçek F-16 kalibrasyonu değil |
| Nominal start altitude | Yaklaşık 4000 m MSL |
| Maximum planning altitude | 6000 m MSL, hard inclusive ceiling |
| Minimum flight altitude | Noktasal olarak `terrain_msl + 100 m AGL` |
| Speed state | İlk sürümde yok |
| Reference speed türü | Sabit CAS; J3/J3.1 ile 305 KCAS seçildi ve doğrulandı |
| Atmosfer | Standard atmosphere; wind/gust/turbulence yok |
| Planner integration | J1 kapsamında yok |
| JSBSim in expansion | Yasak |

## 4. Supported characterization altitude domain

J3.1 domain amendment ile reusable prototype aircraft-characterization domain'i
**`[0, 6000] m MSL`** olarak kilitlenmiştir. Main grid 0 m anchor'dan başlayan
500 m aralıklıdır; midpoint holdout'lar 250 m offset'tedir. Lookup bu domain'in
dışına extrapolation yapamaz.

Önceki `[1500, 6000] m MSL` domain, mevcut 10×10 km prototype ROI'nin ölçülen
terrain minimumu `1697.698364 m` ve `terrain + 100 m AGL` ihtiyacından türetilmişti.
Bu veri ve eski J3 ölçümleri geçerliliğini korur; silinmemiş veya yeniden
çalıştırılmamıştır. Ancak gelecekteki planning region'ler belirgin biçimde daha
düşük terrain içerebilir ve maneuver-envelope üretimi başlamadan önce ek
characterization maliyeti küçüktür. Bu nedenle aircraft domain'i coğrafi ROI'den
ayrılarak 0 m MSL'ye genişletilmiştir.

Bu domain gerçek F-16 operational envelope iddiası değildir. Online minimum
uçuş irtifası hâlâ primitive boyunca `terrain_msl + 100 m AGL` hard gate'idir;
characterization floor'un 0 m olması bu kuralı gevşetmez.

## 5. Minimal lookup sözleşmesi

### 5.1 Lookup index'i

```text
(altitude_band_id, maneuver_id)
```

Altitude, mevcut planner `z` değerinden türetilir; yeni bir search-state boyutu
değildir. Speed de lookup index'i veya search-state boyutu değildir.

### 5.2 Online alanlar

| Alan | Zorunluluk | Anlam |
|---|---:|---|
| `feasible` | Zorunlu | Kabul edilmiş envelope bu maneuver'a izin veriyor mu? Characterization status ile aynı alan değildir. |
| `v_tas_mps` | Zorunlu | Sabit CAS politikasının band'deki TAS karşılığı |
| `gamma_min_deg` | Zorunlu | Kabul edilmiş en dik descent; negatif işaretli |
| `gamma_max_deg` | Zorunlu | Kabul edilmiş en dik climb; pozitif işaretli |
| `capability_value_kind` | Zorunlu | Değerlerin `raw` veya ileride `derated` capability'den geldiğini belirtir |
| `kappa_1pm` | Heading aşamasında | Kabul edilmiş signed horizontal curvature |
| `path_length_m` | Primitive aşamasında | Ölçülen/üretilen primitive uzunluğu |
| `duration_s` | Metadata | Tahmini süre; feasibility state'i değildir |
| `geometry_ref` | Non-ideal primitive'de | Terrain sampling için geometry kimliği |

`turn_radius_m` ayrıca zorunlu değildir; `1 / abs(kappa_1pm)` ile türetilir.
Straight maneuver için `kappa_1pm = 0` olur. Raw ve ileride üretilecek derated
capability değerleri aynı alanın üzerine yazılmaz; ayrı tutulur ve kullanılan
değer türü açıkça belirtilir.

### 5.3 İlk tüketilebilir lookup

Heading planner state'ine eklenene kadar online tüketim yalnız şunlarla
sınırlıdır:

```text
feasible, v_tas_mps, gamma_min_deg, gamma_max_deg, capability_value_kind
```

Turn verisi offline üretilebilir fakat heading entegrasyonundan önce planner'a
bağlanamaz.

## 6. Offline-only kanıt alanları

Aşağıdakiler lookup'ın online payload'ı değildir:

- Trim convergence ve residual'lar
- AoA ve sideslip
- Load factor
- Throttle ve engine regime
- Control command/position ve saturation
- Mach, dynamic pressure, CAS tracking error
- Body/angular rates
- Settling time ve steady-window varyansı
- Başlangıç/bitiş mass ve CG
- JSBSim/model/config hash'leri

Her lookup satırının audit kaydı bunları içerebilir.

## 7. Primitive boundary contract

Heading-aware ilk sürümde bank angle state'e eklenmeyecektir. Bu nedenle turn
primitive'leri aşağıdaki standart sınır koşuluna sahip olmalıdır:

- Başlangıç: nominal CAS, tanımlı gamma, kararlı, wings-level.
- Maneuver: roll-in + sustained bölüm + roll-out.
- Bitiş: nominal CAS/gamma toleransında, kararlı, wings-level.

Bu koşul sağlanmayan primitive'ler `(x,y,z,heading)` state'iyle güvenli biçimde
birbirine bağlanamaz. Sürekli banked maneuver chaining daha sonraki ayrı bir
state-model kararıdır.

## 8. Characterization acceptance ve sonuç semantiği

### 8.1 Provisional prototype acceptance profile

| Ölçüm kriteri | Değer |
|---|---:|
| Cold-start repeats | 3 |
| Steady measurement window | 20 s |
| Maximum settling/replay allowance | 30 s |
| CAS tracking tolerance | ±%2 |
| Gamma tracking tolerance | ±0.5° |
| Sustained curvature stability | ±%5 |
| Normalized control-surface usage | ≤%90 |
| Temel characterization metriklerinin cold-start repeatability'si | Yaklaşık ±%1 |

Throttle, generic normalized control-surface saturation sınırına bağlanmaz.
Throttle acceptance daha sonra her maneuver için ayrı policy ile tanımlanır.

CAS ±%2, gamma ±0.5° ve curvature ±%5 planner maneuver limitleri değildir.
Bunlar yalnız JSBSim ölçümünün güvenilir kabul edilme toleranslarıdır.
Capability limitleri gerçek ölçülen envelope'dan daha sonra üretilir.

### 8.2 Üç durumlu sonuç modeli

Her test noktası tam olarak bir status alır:

#### `VALID`

- Maneuver güvenilir şekilde ölçülmüştür.
- Acceptance kriterlerinin tamamı sağlanmıştır.
- Lookup üretiminde ve capability-boundary çıkarımında kullanılabilir.

#### `INFEASIBLE`

- Ölçüm sonucu güvenilirdir fakat aircraft/model hedef maneuver'ı gerçekten
  sürdürememiştir.
- Operational/physical limit, saturation, stall veya doğrulanmış unstable
  behavior gibi uygulanamazlık kanıtı vardır.
- Lookup satırı üretmez; capability envelope sınırı için negatif kanıt olarak
  kullanılabilir.

#### `UNKNOWN`

- Trim, settling, tracking tolerance, numerical issue veya repeatability
  problemi nedeniyle güvenilir karar çıkarılamamıştır.
- `INFEASIBLE` ile eşit değildir.
- Lookup üretiminde ve capability pruning'de kullanılamaz.
- İki `VALID` nokta arasında bulunsa bile üzerinden interpolation yapılamaz.
- Retest, farklı target veya tolerance review gerektirir.

Trim yakınsaması tek başına `VALID` üretmez. NaN/Inf, configuration uyuşmazlığı
veya tekrarlanamayan sonuç varsayılan olarak `UNKNOWN` üretir; ancak ayrı ve
güvenilir bir test maneuver'ın gerçekten sürdürülemez olduğunu gösterirse
`INFEASIBLE` verilebilir.

### 8.3 Infeasible boundary semantiği

`INFEASIBLE` test noktası exact capability limit değildir. Bir sweep'te:

```text
7° -> VALID
8° -> INFEASIBLE
```

ise yalnız `7° < gerçek capability sınırı < 8°` sonucu çıkar. Sınır daha sonra
sweep veya local refinement ile daraltılır. `UNKNOWN` bir sınırı bracket etmek
için kullanılamaz ve hiçbir zaman `INFEASIBLE` yerine geçmez.

### 8.4 Derating semantiği

- J1'de sabit yüzde derating kilitlenmez ve uygulanmaz.
- Özellikle varsayılan `%20` safety margin yoktur.
- Kabul edilmiş ham ölçümler `raw_capability` olarak korunur.
- Gelecekteki `derated_capability` aynı alanın üzerine yazılmaz; ayrı tutulur.
- Derating kararı J3/J4 verileri görüldükten sonra ayrıca alınır.

## 9. Altitude-band semantics

- Bandlar MSL tabanlıdır; AGL lookup index'i değildir.
- İlk characterization grid'i 500 m aralıklıdır ve MSL'de 500 m'nin tam
  katlarına hizalanır.
- Her iki ana noktanın arasındaki 250 m midpoint bağımsız holdout testidir;
  lookup üretiminde başlangıçta kullanılmaz.
- Midpoint holdout, 500 m temsilin ölçülen kabiliyeti iyimser gösterdiğini
  saptarsa yalnız ilgili 500 m aralık 250 m çözünürlüğe bölünür.
- Bir aralığın refinement kararı komşu ve ilgisiz bandları global olarak
  inceltmez.
- AGL her primitive boyunca terrain validator tarafından ayrıca ölçülür.
- Bir band değeri o band içindeki kabiliyetin konservatif temsilidir.
- `UNKNOWN` noktalar üzerinden interpolation veya capability pruning yapılamaz.
- `INFEASIBLE` noktalar yalnız güvenilir biçimde doğrulanmış envelope sınır
  bilgisi olarak kullanılabilir.
- Band dışına extrapolation yapılamaz.
- Band sınırı geçen primitive için başlangıç önerisi boundary splitting'dir;
  bu davranış lookup entegrasyonundan önce ayrıca kilitlenecektir.

## 10. Characterization workflow

### 10.1 First validation gate

İlk gerçek JSBSim characterization çalışması yalnız şu küçük scope'u kapsar:

- Steady level straight flight
- Straight climb
- Straight descent

Bu gate aircraft envelope'u tamamlamaz. Yalnız şu measurement/test harness
özelliklerini kanıtlar:

- Fresh/cold-start reset
- Trim
- Nominal speed handling
- Altitude initialization
- Settling detection
- 20 s measurement window ve 30 s maximum allowance
- Unit/sign conventions
- `VALID / INFEASIBLE / UNKNOWN` classification
- Üç cold-start tekrarda yaklaşık ±%1 repeatability
- Logging ve ortak record schema

Bu gate PASS etmeden full batch sweep başlatılamaz.

### 10.2 Full batch characterization

Validation gate PASS olduktan sonra aynı doğrulanmış harness, supported altitude
domain boyunca tek batch characterization akışında kullanılır. Sustained
maneuver characterization gereksiz micro-stage'lere bölünmez:

1. **Straight:** level, climb sweep, descent sweep.
2. **Level turns:** left/right sustained curvature ve radius sweep.
3. **Coupled turns:** climbing left/right ve descending left/right turn.

Straight-only yaklaşım yalnız ilk validation gate içindir; final
characterization scope straight-only değildir.

### 10.3 Macro primitive characterization

Macro primitive characterization transient geometry ürettiği için sustained
batch'ten ayrı kalır:

```text
wings-level start -> roll-in -> sustained arc -> roll-out -> wings-level end
```

### 10.4 Ortak test-record schema

Her test sonucu aynı alanları taşır:

- Altitude MSL
- Maneuver family
- Requested target
- Measured values
- Status: `VALID / INFEASIBLE / UNKNOWN`
- Raw capability
- Derated capability: başlangıçta boş/unresolved
- Diagnostics
- Provenance

## 11. J1 tamamlanma kriterleri

J1 ancak aşağıdaki kararlar kullanıcıyla kilitlendiğinde DONE sayılır:

1. ~~Altitude-band genişliği.~~ **Kilitlendi:** 500 m ana grid, 250 m midpoint
   holdout, non-conservative aralıkta yerel 250 m refinement.
2. ~~Fidelity/tolerance profili.~~ **Kilitlendi:** provisional prototype
   acceptance profile.
3. ~~Lookup capability safety derating politikası.~~ **Kilitlendi:** J1'de
   sabit derating yok; raw/derated alanlar ayrı, karar J3/J4 sonrasında.
4. ~~Supported characterization altitude domain ve sayısal floor.~~
   **J3.1 amendment ile kilitlendi:** reusable prototype aircraft domain'i
   `[0, 6000] m MSL`; önceki `[1500, 6000] m` verileri korunur.
5. ~~İlk maneuver kapsamı.~~ **Kilitlendi:** straight validation gate;
   ardından straight + level turns + coupled turns full batch; macro transient
   characterization ayrı.

Beş J1 kararı da kapatılmıştır. J1'in tamamlanması JSBSim'in kurulduğu veya bir
aircraft run yapıldığı anlamına gelmez. Bu çalışmada J2 başlatılmamıştır.
