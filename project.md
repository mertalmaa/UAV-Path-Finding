# UAV Pathfinder — Proje Notu

Terrain-aware fixed-wing UAV path planner. Amaç: DEM (Digital Elevation Model)
rasterından, arazi çarpışmasından kaçınan bir C-Space/pathfinding hattı
üretmek. Bu doküman mimari kararları ve o kararların gerekçelerini kayıt
altına alır; implementasyon durumu ilerledikçe güncellenir.

Referans doküman: `İHA Arazi Uyumlu Planlama.pdf` — genel mimari/araştırma
yol haritası. PDF bir roadmap'tir, bu dosyadaki kararlar PDF'nin tamamını
implement etme taahhüdü değildir; her aşama kendi promptunun scope'unda
kalır.

## MISSION OBJECTIVE CONTRACT

**Safety > all optimization objectives.** Terrain collision, minimum AGL,
bounds, NoData ve climb/descent limitleri hard feasibility koşullarıdır;
unsafe bir edge/path, soft cost'u ne kadar düşük olursa olsun aday değildir.

Safe path'ler arasında **low aircraft absolute MSL**, distance'tan biraz daha
önemli primary soft preference'tır; distance ise aşırı detour'ları engelleyen
güçlü regularizer olarak kalır. Bu strict lexicographic bir kural değildir.
Mevcut production candidate `w_distance=1.0`, `w_altitude=1.25`,
`altitude_scale_m=1000`; yaklaşık 100m daha düşük absolute MSL, yaklaşık
%10–15 ek path length'i haklı çıkarabilir. Bu bir hard threshold veya
location-specific kural değil, additive soft trade-off'tur.

**Low MSL = low aircraft absolute MSL.** Şunlar değildir: minimum AGL,
terrain following, lowest terrain elevation veya reduced safety margin.
Altitude reference explicit mission datum'dur; terrain elevation'dan veya
search altitude bound'larından türetilmez.

**Cost correctness != search effectiveness.** Mission cost bir rotayı doğru
şekilde tercih edebilir, fakat search algorithm bu rotayı verilen epsilon,
budget veya exploration geometry altında erken keşfedemeyebilir. Objective
ranking ve search discoverability ayrı test edilmeli ve ayrı raporlanmalıdır.

## Öncelik sırası

Güvenlik > pathfinding performansı > basitlik. Bu yüzden aşağıdaki
kararların çoğu "source terrain'i tehlikeli biçimde düşük temsil etmeme"
hedefine hizmet eder (ör. resampling yöntemi, NoData davranışı, validation
aşamaları).

## Pipeline (hedef, uçtan uca)

```
Source DEM (EPSG:4326, GLO-30)
  -> UTM working DEM (bir kerelik reprojection)
  -> N×N km ROI (metre, UTM)
  -> terrain query layer (nokta sorgusu)
  -> binary mask
  -> polygonization (UTM koordinatlarında)
  -> component extraction
  -> validity check
  -> simplification / tolerance sweep
  -> buffer / C-Space
  -> pathfinding (A*, motion primitive, vb.)
```

Bu pipeline'daki her ok ayrı bir implementasyon adımıdır; ileri adımlar
(binary mask'ten sonrası) henüz implement edilmedi.

## Çalışma bölgesi

Aladağlar/Toroslar (Niğde-Adana sınırı). Kaynak veri:
`copernicus_glo30_turkey/` klasöründeki Copernicus GLO-30 DSM mozaiği
(143 karo). Hedef karo: `Copernicus_DSM_COG_10_N37_00_E035_00_DEM.tif`
(CRS=EPSG:4326, 1 arcsec çözünürlük, 3600×3600 px, lon 35–36 / lat 37–38).
Source tile NoData/sentinel taraması yapıldı: temiz çıktı, gerçek nodata
değeri yok (`ds.nodata is None`, sentinel piksel bulunmadı).

## Mimari kararlar

- **Projected CRS: EPSG:32636 (UTM 36N).** Gerçek ROI bounds ile ölçülen
  distorsiyon +0.003–0.005% — bu bölge için ihmal edilebilir.
- **ROI merkezi: kaynak DEM'deki max-elevation pikseli**,
  lon=35.14833 / lat=37.80611 (Demirkazık yakını).
- **Resampling: `Resampling.max`** (block-maximum). Amaç muhafazakârlık:
  bir hücredeki en yüksek noktayı asla düşük temsil etme (average/bilinear
  zirveleri düzleştirip UAV için tehlikeli iyimser bir DEM üretebilir).
- **NoData asla geçerli elevation sayılmaz.** `dst_nodata=-9999`, working
  DEM üretiminde `src_nodata` set edilmez (kaynakta zaten yok).
- **8-connectivity, interior ring/hole korunuyor** — ileri aşama
  (polygonization) için karar, henüz kullanılmıyor.
- **Simplification buffer'dan önce yapılır; buffer'da round join
  zorunlu.** Tolerance sweep adayları: `0, 5, 10, 20, 30, 60` metre —
  ileri aşama kararları, henüz kullanılmıyor.
- **Working DEM reprojection yöntemi:** `rasterio.warp.calculate_default_transform`
  ile hedef 30 m çözünürlüğü zorlanan explicit `transform/width/height`
  hesaplanır, sonra `rasterio.vrt.WarpedVRT` + windowed read ile sadece
  ihtiyaç duyulan bölge okunur (tüm tile'ı belleğe almadan). Not: bu
  rasterio sürümünde (1.5.1) `WarpedVRT`'in `resolution` parametresi yok —
  verilirse sessizce yok sayılıp GDAL'ın kendi varsayılan çözünürlüğü
  kullanılıyor; bu yüzden explicit transform hesabı zorunlu.
- **Bilinen kısıt:** UTM working DEM'in tile köşelerinde grid-convergence
  kaynaklı nodata kamaları oluşabilir (geographic→UTM warp'ta kaynak
  footprint dışında kalan pikseller). Merkeze yakın ROI'ler güvenli; tile
  kenarına yakın gelecekteki ROI'ler için ayrıca kontrol gerekir.

## Implementasyon durumu (2026-09-09)

- `planner/config.py` — `PlannerConfig`: working DEM yolu, target CRS,
  ROI merkezi/boyutu, xy resolution, `z_step_m` (20 m, henüz kullanılmıyor),
  `min_agl_m`/`max_climb_angle_deg`/`max_descent_angle_deg` (fiziksel
  değer bilinmediği için bilerek `None`).
- `scripts/build_working_dem.py` — working DEM'i üretir:
  `working_dem/aladaglar_N37_E035_utm36n_max.tif`, 667×667 px, 30×30 m,
  EPSG:32636, merkez etrafında 20×20 km (bugünkü 10×10 km ROI'ye 5 km
  pay bırakacak şekilde).
- `planner/roi.py` — `ROIData` (elevation, transform, crs, width/height,
  bounds, resolution, nodata) + `load_roi()`: working DEM'den windowed
  read ile ROI'yi keser.
- `scripts/validate_roi.py` — ROI validation: CRS, resolution, extent,
  grid size, NoData, elevation sanity, dtype/shape. Sonuç: ALL PASS
  (10×10 km ROI, 333×333 px, 30×30 m, 0 nodata piksel, elevation
  1697.7–3700.4 m).
- `planner/terrain.py` — nokta bazlı terrain elevation sorgusu (UTM
  (x,y) → row/col → elevation), ROI'nin gerçek affine transform'unu
  kullanır, disk I/O yapmaz. Detay: bkz. modül docstring'i.
- `planner/agl.py` — `evaluate_agl()`: verilen `(x, y, aircraft_altitude_msl)`
  için `agl_m = aircraft_altitude_msl - terrain_elevation_msl` hesaplar,
  `agl_m < min_agl_m` ise INVALID (`below_min_agl`), terrain out-of-bounds/
  NoData ise INVALID. `TerrainQuery` üzerinden çalışır, terrain erişimini
  tekrar implement etmez. Bu bir cost fonksiyonu değildir, sadece hard
  feasibility gate'tir — ileride alçak uçuş tercihi AGL değil, MSL altitude
  üzerinden ayrı bir cost terimiyle eklenecek.
  **`config.min_agl_m = 200.0` şu an TEST PARAMETER'dır**, gerçek
  aircraft/mission requirement değildir; gerçek değer geldiğinde
  `planner/config.py`'den değiştirilecek.
- `planner/transition.py` — `evaluate_transition(start, end, config)`: iki
  `(x,y,z_msl)` state arasındaki gerçek yatay mesafeyi (Öklid) ve
  flight-path açısını (`atan2(abs(delta_z), d_xy)`) hesaplar, climb/descent
  açı limitine göre VALID/INVALID döner. Saf geometri — terrain sorgusu,
  AGL kontrolü, motion primitive veya cost yok; `d_xy==0` özel durumları
  (dikey sıçrama / no-motion) ayrı reason'larla ele alınıyor.
  **`config.max_climb_angle_deg = 10.0` ve `max_descent_angle_deg = 10.0`
  şu an TEST PARAMETER'dır**, gerçek aircraft performance requirement
  değildir.
- `planner/primitives.py` — `build_primitive_set()`: 8 raster yönü (N/NE/E/
  SE/S/SW/W/NW) × {level, climb, descent} = 24 primitive. Level = tek
  hücre; climb/descent'in yatay mesafesi hard-code değil,
  `abs(dz)/tan(max_angle)` formülünden ve o yöndeki en küçük tam grid-step
  sayısından türetiliyor (axial 4 hücre ≈120m, diagonal 3 hücre ≈127.28m).
  Her primitive `evaluate_transition()` ile doğrulanıyor, geometrik
  INVALID olan set'e girmiyor (bu üretim yönteminde hiçbiri elenmiyor,
  ama kontrol yine de yapılıyor). `evaluate_primitive()`: primitive
  boyunca (`primitive_sample_spacing_m = 10.0`, aircraft performance değil
  numerical sampling resolution parametresi) `evaluate_agl()` ile terrain
  sample alıyor, endpoint-only kontrolün kaçıracağı orta-yol ridge'lerini
  yakalıyor. Terrain/AGL/transition mantığı tekrar yazılmadı, mevcut
  modüller reuse edildi.

- `planner/astar.py` — terrain-aware 3D A*. Canonical state
  `(row, col, z_index)` integer (float state değil — floating-point drift
  nedeniyle aynı fiziksel state'in birden fazla kez oluşmasını engellemek
  için). `z_msl = z_index * z_step_m`; MSL↔z_index dönüşümü
  (`msl_to_z_index`/`z_index_to_msl`) explicit, `allow_snap=True`
  verilmeden hizasız bir altitude sessizce yuvarlanmıyor. Neighbor
  generation lazy: her expansion'da 24 primitive denenir, graph önceden
  kurulmaz; terrain/AGL/climb-descent kontrolü tamamen mevcut
  `evaluate_primitive()` üzerinden — A* içinde tekrar yazılmadı. Edge cost
  = primitive'in gerçek 3D geometrik uzunluğu
  (`sqrt(horizontal_distance²+dz²)`); heuristic = goal'a 3D Öklid mesafesi
  (gerçek UTM metre). Low-altitude MSL cost, weighted A*, 2D Dijkstra
  heuristic YOK — bilinçli olarak bu aşamanın kapsamı dışında.
  `min_search_altitude_msl`/`max_search_altitude_msl` her çağrıda açıkça
  verilmesi gereken **prototype search-space sınırları** — aircraft flight
  ceiling değil. `SearchResult`: `success`, `status`
  (`success|no_path|search_limit_reached` — ayrı ayrı raporlanıyor), `path`,
  `total_cost`, `expanded_nodes`, `generated_neighbors`,
  `rejected_neighbors`, `rejected_reason_counts`, `max_open_size`,
  `runtime_s`. 5 validation senaryosu (flat level path, tek-primitive
  climb, ridge detour, true no-path, gerçek Aladağlar ROI smoke test) ALL
  PASS; her başarılı path `evaluate_primitive()` ile edge-edge tekrar
  doğrulanıyor (consistency check, bağımsız final validator'ın yerine
  geçmiyor).

- **A* cost'una düşük-MSL soft preference eklendi** (`planner/astar.py`
  içinde, ayrı modül değil). `config.msl_cost_weight = 0.25` **TEST/TUNING
  PARAMETER**, `0.0` verilirse baseline geometrik A* birebir (bit-exact)
  geri gelir. `edge_cost = geometric_cost * (1 + msl_cost_weight *
  altitude_norm)`; `altitude_norm`, edge'in ortalama MSL'sinin
  `[min_search_altitude_msl, max_search_altitude_msl]` aralığına göre
  `[0,1]`'e normalize edilmiş hali (aralık sıfırsa penalty=0). Heuristic
  değişmedi (saf 3D Öklid) — gerçek edge cost her zaman
  `>= geometric_cost` olduğundan heuristic hâlâ admissible/consistent,
  optimality korunuyor. AGL/terrain/transition safety hâlâ tamamen
  `evaluate_primitive()` tarafından karar veriliyor; MSL sadece zaten
  güvenli kabul edilmiş edge'ler arasında soft bir tercih. `SearchResult`'a
  `geometric_path_length`, `minimum/maximum/average_aircraft_msl`,
  `minimum_observed_agl` eklendi (sonuncusu `evaluate_primitive()` ile
  path edge'leri üzerinden re-derive ediliyor, bağımsız bir final
  validator değil). 4 validation (weight=0 regresyon, low-MSL preference,
  AGL-supremacy, distance-vs-MSL trade-off) ALL PASS.

- **Vertical-motion (climb/descent) soft penalty eklendi** (`planner/astar.py`,
  ayrı modül değil). `config.vertical_cost_weight = 1.0` **TEST/TUNING
  PARAMETER**, `0.0` verilirse bir önceki adımın (low-MSL) davranışı
  birebir geri gelir. Edge cost formülü artık:
  `edge_cost = geometric_cost*(1+msl_cost_weight*altitude_norm) +
  vertical_cost_weight*abs(delta_z)`. Climb ve descent aynı şekilde
  cezalandırılıyor (ayrı weight yok, bilinçli olarak bu aşamada). Formül
  tek yerde: `compute_edge_cost()` — hem `_generate_neighbors()` hem
  validation scriptleri aynı fonksiyonu kullanıyor, tekrar yazılmıyor.
  Heuristic değişmedi. AGL/climb-descent-angle hard limitleri hâlâ tamamen
  `evaluate_primitive()` tarafından karar veriliyor; vertical penalty bir
  edge'i asla yasaklayamaz, sadece zaten-güvenli edge'ler arasında
  sıralama yapar — zorunlu climb senaryosunda bunu doğrulayan validation
  PASS (climb 60m, `minimum_observed_agl=200.8 ≥ 200`). `SearchResult`'a
  `total_climb_m`, `total_descent_m`, `total_vertical_motion_m` eklendi.
  5 validation (weight=0 regresyon, level-vs-gereksiz-vertical, low-MSL
  vs vertical trade-off, zorunlu climb, roller-coaster-vs-smooth) ALL PASS.

- **Cost tuning / sensitivity analizi yapıldı** (`scripts/tune_cost_weights.py`,
  yeni cost terimi veya formül değişikliği YOK). `msl_cost_weight ∈
  {0,0.25,0.5,1.0}` × `vertical_cost_weight ∈ {0,0.5,1.0,2.0}` = 16
  kombinasyon; 2 sentetik senaryo (Scenario A: dar 1-step dive koridoru,
  Scenario B: geniş 5-step dive gradyanı) tam 16-grid ile, gerçek
  Aladağlar ROI (row=80, col90-128, gerçek ~164m vadi, prototype
  cruise=2840m) 4 temsili kombinasyonla (gerçek DEM'de 16-grid runtime
  makul değil — tek kombinasyon 0.6-43s arası). Ayrıca search-altitude
  normalization sensitivity testi: aynı fiziksel senaryo/weight, sadece
  `[1300,1400]` vs `[1300,2400]` search bounds → geniş aralıkta MSL
  penaltı etkisi büyük ölçüde sönümleniyor (vertical_motion 120m→0m).
  Tüm 36 satır `tuning_results.csv`'ye yazıldı. Sonuç: iki prototype
  tuning adayı önerildi (bkz. sohbet), **config default'u henüz
  değiştirilmedi** — kullanıcı kararı bekleniyor.

- **Stage 10 — MSL cost normalization search-bounds'tan bağımsız hale
  getirildi.** Eski `altitude_norm` (search bounds'a göreceli, `[0,1]`'e
  clamp'li) kaldırıldı: aynı fiziksel 1400m edge, `[1300,1400]` search
  ceiling'inde `norm=1.0`, `[1300,2400]`'de `norm≈0.09` veriyordu — sadece
  ceiling genişlediği için aynı irtifa farklı maliyetleniyordu (sensitivity
  testinde Stage 9'da yakalandı). Yeni: `altitude_scaled = max(0,
  mean_altitude_msl - msl_reference_m) / msl_scale_m`, **clamp yok**,
  search bounds'tan **hiç parametre almıyor**. `config.msl_reference_m=0.0`
  (deniz seviyesi) ve `config.msl_scale_m=1000.0` **TEST/TUNING
  PARAMETER**. `compute_edge_cost()` artık `min/max_search_altitude_msl`
  parametrelerini almıyor (search bounds hâlâ `_generate_neighbors()`'da
  hangi altitude'ların keşfedileceğini belirliyor — sadece cost'tan
  çıkarıldı). **Önemli sonuç: `msl_cost_weight`'in anlamı değişti** —
  eski `[0,1]` skalada `0.25` ile yeni sınırsız skalada (gerçek Aladağlar
  senaryosunda cruise=2840m → `scaled=2.84`) `0.25` çok farklı şiddette;
  config default'u (`0.25`) bilinçli olarak DEĞİŞTİRİLMEDİ, re-tuning
  ayrı bir adıma bırakıldı. Gerçek ROI'de yeni normalization ile
  `(msl=0.25,vertical=1.0)` runtime'ı Stage 9'daki 21.4s'den **54.3s**'ye
  çıktı (4319 expanded node) — ama sonuç davranışı (hiç dalmadı) aynı
  kaldı; yani arama çok daha pahalı hale geldi ama karar değişmedi. Hard
  safety (AGL/climb/descent) regresyon testi PASS, hiç dokunulmadı.

- **Stage 11 — yeni normalization sonrası mini `w_MSL` re-tuning yapıldı,
  config default'u DEĞİŞTİRİLMEDİ.** Aynı Stage 9/10 gerçek Aladağlar
  senaryosu (row=80, col90-128, cruise=2840m), `vertical_cost_weight=1.0`
  sabit, `msl_cost_weight ∈ {0.0, 0.025, 0.05, 0.10}`. **Beklenmedik ve
  önemli bulgu: 4 weight'in HİÇBİRİNDE dalış davranışı tetiklenmedi**
  (`avg_MSL=2840.0`, `vertical_total=0.0` dört run'da da birebir aynı) —
  sadece `expanded_nodes` (39→1842) ve `runtime` (0.21s→22.24s, ~106×)
  hiçbir davranış değişikliği olmadan arttı. Kök neden: `vertical_cost_weight
  =1.0` ile tek bir 20m dalış-geri-çıkış'ın maliyeti (~40 birim, sabit) bu
  test edilen `msl_scale_m=1000` ölçeğinde ulaşılabilecek herhangi bir MSL
  tasarrufunu (~2-10 birim) yapısal olarak eziyor. Sonuç: bu aralıkta
  ölçülebilir bir low-MSL kazancı yok; `retuning_results.csv`'ye yazıldı.
  Öneri: `w_MSL=0.0` (ölçülen kazanç sıfırken runtime artışını göze almanın
  gerekçesi yok) — ama karar kullanıcıda, config default'u dokunulmadı.
  Daha geniş bir sweep (0.10 üstü) veya `vertical_cost_weight`'in yeniden
  gözden geçirilmesi ayrı, gelecekteki bir adımın konusu.

- **Stage 12 — vertical cost, `abs(delta_z)` modelinden
  vertical-direction-reversal modeline geçirildi.** Eski
  `vertical_cost_weight * abs(delta_z)` (her climb/descent adımını ayrı
  ayrı cezalandırıyordu) **kaldırıldı**, config'ten silindi. Yerine
  `vertical_reversal_cost_weight = 1.0` (TEST/TUNING PARAMETER): yalnızca
  climb↔descent yön değişimi (`reversal`) cezalandırılıyor, sürekli
  descent veya sürekli climb artık **ücretsiz** (MSL terimi hariç).
  A* state'i `(row,col,z_index)` → `(row,col,z_index,vertical_trend)`'e
  genişletildi (`vertical_trend ∈ {-1,0,+1}`) — reversal geçmişe bağlı
  olduğu için Markov özelliği state'e trend eklenmeden korunamıyordu.
  Level primitive trend'i sıfırlamıyor (`descent→level→level→descent`
  hâlâ tek bir descent trendi). Goal kontrolü yalnızca fiziksel
  `(row,col,z_index)` üzerinden — trend'den bağımsız (start/goal public
  API'si hâlâ 3-tuple, augmented state sadece iç arama mekanizmasında).
  `SearchResult`'a `vertical_reversal_count`, `total_reversal_penalty`
  eklendi; `total_climb_m`/`total_descent_m`/`total_vertical_motion_m`
  artık saf fiziksel metrik, cost bileşeni değil. **8 validation ALL
  PASS**: continuous climb/descent penaltısız, direct reversal
  (`descent→climb`, `climb→descent`) tam olarak 1 reversal + doğru
  formülle cezalandırılıyor, level adımıyla penalty kaçırılamıyor,
  roller-coaster (3 reversal, aynı endpoint/vertical-motion) düzgün
  alternatiften (1 reversal) kesin daha pahalı, A* gerçekten reversal'dan
  kaçınıyor, low-MSL + continuous-descent senaryosu (`w_MSL=0.025`) 0
  reversal ile düzgün 100m descent buldu. **Gerçek Aladağlar'da rota
  değişmedi** (Stage 11'deki "dalış yok" sonucu aynen duruyor — demek ki
  o bulgu eski vertical penalty modelinden değil, `msl_cost_weight`'in
  `msl_scale_m` ölçeğine göre zayıf kalmasından kaynaklanıyormuş).
  State-space büyümesi beklenen gibi gerçekleşti: aynı gerçek ROI
  senaryosunda (`w_MSL=0.025`) runtime Stage 11'in ~1.8s'sinden ~7-11s'ye
  çıktı (~4-6×, `(row,col,z)` artık her trend değeri için ayrı
  ziyaret edilebiliyor).

- **Stage 13 — gerçek Aladağlar senaryosunda "neden hiç inmiyor" sorusu
  sayısal olarak çözüldü (`scripts/diagnose_msl_breakeven.py`, planner
  kodunda DEĞİŞİKLİK yok, pure diagnostic).** Aynı senaryo (row=80,
  col90→128, cruise=2840m), LEVEL^A DESCENT^k LEVEL^B CLIMB^k LEVEL^C
  şablonlu 921 smooth candidate (k=0..4) `evaluate_primitive()` ile tek
  tek doğrulandı — **hiçbiri safety nedeniyle infeasible değil** (0
  invalid). Production cost `G (geometric) + w_MSL*M (MSL) + R
  (reversal)` bileşenlerine ayrıştırıldı (production `compute_edge_cost()`
  ile bit-exact tutarlılık doğrulandı). Sonuç: break-even `w_MSL` her k
  için **1.5-2.1** civarında çıktı — test edilen aralığın (0-0.10) **15-20
  katı**. Kök neden çok-faktörlü: (1) **baskın faktör — MSL formülasyonu/
  ölçeği**: `msl_scale_m=1000` sabit ölçeğinde, gerçekçi bir 20-80m
  irtifa değişimi `altitude_scaled`'i yalnızca ~0.02-0.08 azaltıyor,
  bu yüzden reversal cost'u tamamen hariç tutsak bile break-even hâlâ
  0.30-0.84 (test aralığının üstünde); (2) **ikincil, güçlendirici
  faktör — reversal cost**: tek zorunlu descent→climb geçişi sabit
  +20 birim ekliyor (bu, k'den bağımsız — sadece o tek primitive'in
  dz'sine bağlı), extra-cost'un ~%86'sını oluşturuyor ve break-even'i
  ~0.30'dan ~1.5-2.1'e fırlatıyor; (3) geometric detour maliyeti
  görece küçük (%14, ~3.3-13.2 birim, toplam path uzunluğunun ~%0.3-1.2'si).
  **Safety/geometry kök neden DEĞİL.** Sonuç kullanıcıya raporlandı,
  hiçbir weight veya formül değişikliği yapılmadı — karar bekleniyor.

- **Stage 14 — vertical reversal cost, sabit ("her reversal aynı") modelden
  spacing-sensitive (trend ne kadar uzun sürmüşse reversal o kadar ucuz)
  modele geçirildi; önceki oturumlar arasında diskte oluşmuş "ilk
  descent→climb ücretsiz" özel kuralı (`free_descent_to_climb_used`
  flag'i) KALDIRILDI, genel spacing mekanizmasıyla değiştirildi.**
  Config: `reversal_relax_distance_m=300.0`, `trend_age_unit_m=30.0`
  (TEST/TUNING PARAMETER). A* state `(row,col,z,trend)` →
  `(row,col,z,trend,trend_age_units)` (`trend_age_units` capped integer,
  0..10, ≈0-300m). `reversal_cost = vertical_reversal_cost_weight *
  abs(dz) * (1 - clamp(previous_trend_distance_m/300, 0, 1))` — sürekli
  climb/descent hâlâ tamamen ücretsiz, ≥300m'lik doğal terrain-kaynaklı
  reversal'lar ~ücretsiz, kısa-aralıklı zigzag'lar pahalı. 6 validation
  (continuous climb/descent, uzun-mesafeli doğal reversal, kısa reversal,
  roller-coaster, dağ→düzlük→dağ gerçek A* testi, terrain-slope>aircraft-
  angle testi) ALL PASS. **Aladağlar decomposition yeniden hesaplandı**
  (Stage 13'ün candidate'ları): yeni `R=0.000` (hepsinde — en iyi
  candidate'ların "low cruise" bölümü her zaman 300m eşiğini geçiyor),
  break-even `w_MSL` **1.5-2.1'den 0.30-0.84'e düştü** (reversal maliyeti
  kalktığı için ~3-5×). 3 analytical candidate önerildi (conservative≈1.01,
  balanced≈0.63, aggressive≈0.315) — **config default değiştirilmedi**.
  **KRİTİK performans bulgusu**: mevcut config default'u (`msl_cost_weight
  =0.25`) ile gerçek Aladağlar senaryosunda A* artık **351 saniyede,
  30.000 expansion'da bile tamamlanamadı** (`search_limit_reached`) —
  state-space `(trend,age)` ile teorik olarak ~33× büyüdü (Stage 12'nin
  3×'ünden), bu senaryoda pratikte arama artık intractable. İkinci
  (balanced candidate) run, talimat gereği bu yüzden çalıştırılmadı.
  Optimize edilmedi (kapsam dışı); physical-primitive-feasibility caching
  gelecekteki bir performans adımı için not edildi.

- **Stage 15 — physical primitive feasibility cache eklendi.**
  `evaluate_primitive()` sonuçları `(row, col, z_index, primitive_id)` ile
  (yalnızca tek bir `astar_search()` çağrısı boyunca yaşayan, global/kalıcı
  olmayan bir dict'te) cache'leniyor — `vertical_trend`/`trend_age_units`
  key'de YOK (fiziksel feasibility geçmişe bağlı değil), ama
  `compute_edge_cost()`/`_next_trend_and_age()` her augmented state için
  hep taze hesaplanıyor (asla cache'lenmiyor). `astar_search(...,
  use_primitive_cache=True)` (default True). VALID ve INVALID sonuçlar
  ikisi de cache'leniyor. 4 validation (cache ON/OFF path/cost/metrik
  birebir aynı, aynı fiziksel konumdan farklı trend/age ile aynı cache
  key ama farklı cost, INVALID edge tekrar terrain-sample edilmiyor,
  sentetik benchmark'ta expanded_nodes birebir aynı kalırken %88.8 daha
  az `evaluate_primitive` çağrısı + 5.3-5.4× wall-time hızlanma) ALL
  PASS. **Gerçek Aladağlar benchmark'ı** (Stage 14'ün aynı senaryosu,
  aynı 30.000 expansion limiti, `w_MSL=0.25` default): **351.3s → 21.6s
  (16.3× hızlanma)**, `expanded=30000` ve `max_open=33542` **birebir
  aynı** (cache search ordering'i hiç değiştirmedi), cache hit
  rate=%88.1 (598,848 hit / 80,824 miss). Durum hâlâ
  `search_limit_reached` — 30.000 expansion'da hâlâ çözüm bulunamıyor
  (bu beklenen, hata değil). **Sınıflandırma: (A) Cache büyük kazanç
  sağladı** — state-space hâlâ büyük (aynı node sayısı araştırılıyor,
  aynı max_open) ama pahalı terrain-evaluation maliyeti ciddi biçimde
  düştü. Bu adımda trend-age bucketing'e veya low-MSL tie-break'e
  dokunulmadı (ayrı adımlar için bırakıldı).

- **Stage 16 — exact dominance pruning eklendi.** Aynı
  `(row,col,z_index,vertical_trend)` grubu içinde, `(trend_age_units, g)`
  çiftleri arasında Pareto dominance: `A domine eder B'yi ⟺ g_A<=g_B VE
  age_A>=age_B` (`_dominates()`). Önce varsayım **tüm state uzayında
  exhaustive olarak doğrulandı** (her trend × her age_A≥age_B çifti ×
  her primitive, 3168/3168 kombinasyon, 0 ihlal) — implementasyona
  geçmeden önce. Her `base_key` için küçük bir Pareto frontier
  (`List[(age,g)]`, max 7-11 eleman) tutuluyor; dominate edilen candidate
  heap'e hiç girmiyor, dominate eden candidate frontier'daki eski
  entry'leri "aktif değil" yapıyor (silmiyor, `came_from`/`g_score`
  bozulmuyor — lazy invalidation, pop anında frontier'da aktif değilse
  expand edilmiyor). `astar_search(..., use_dominance_pruning=True)`
  (default True). 4 validation (dominance ilişkisi unit test + Case
  A-E, exhaustive monotonicity, OFF/ON aynı optimal cost, sentetik
  state-explosion benchmark: expanded_nodes %46 azaldı) ALL PASS.
  **Gerçek Aladağlar benchmark'ı (nüanslı sonuç):** `max_open` 33,542→
  **25,505 (-24%)**, ama `runtime` 21.6s→**24.97s (+15.6%, YAVAŞLADI)** —
  arama hâlâ 30.000 expansion cap'ine `search_limit_reached` ile
  ulaşıyor (hiç bitmiyor), bu yüzden dominance'ın "daha az toplam
  expansion" faydası bu spesifik senaryoda hiç gerçekleşmiyor
  (expansion sayısı zaten cap'e sabitli), sadece bookkeeping maliyeti
  (96,373 dominance check) ekleniyor. **Sınıflandırma: (B) Orta kazanç**
  — dominance gerçekten işe yarıyor (sentetik testlerde ve `max_open`'da
  net kazanç), ama bu spesifik "arama hiç bitmiyor" senaryosunda
  bookkeeping overhead'i net wall-clock faydasını geçici olarak
  gölgeliyor; asıl darboğaz hâlâ search/state-space'in kendisi.

- **Stage 17 — admissible MSL-aware lower-bound heuristic eklendi, ve
  gerçek Aladağlar senaryosu ilk kez 30k expansion cap'inden ÖNCE
  çözüldü.** Eski heuristic sadece 3D Öklid mesafesiydi (`h=D3D`), gerçek
  MSL maliyetini hiç hesaba katmıyordu. Yeni: `h = D3D * minimum_cost_
  multiplier`, `minimum_cost_multiplier = 1 + w_MSL * scaled_msl_lb`,
  `scaled_msl_lb` search boyunca hiçbir valid state'in altına
  inemeyeceği bir global MSL floor'dan (`minimum_possible_aircraft_msl =
  max(min_search_altitude_msl, terrain_min_valid + min_agl_m)`) türetiliyor
  — search başlamadan **bir kez** hesaplanıyor, terrain tekrar taranmıyor.
  Admissibility/consistency **implementasyondan önce kanıtlandı**: her
  edge'in gerçek maliyeti `>= geometric_cost * multiplier` (çünkü her
  edge'in ortalama MSL'si `minimum_possible_aircraft_msl`'den küçük
  olamaz, `w_MSL>=0` ise `altitude_scaled(edge)>=scaled_msl_lb`, ve
  `reversal_cost>=0`); consistency, multiplier'ın TÜM search boyunca
  sabit bir GLOBAL sayı olması sayesinde D3D'nin üçgen eşitsizliğinden
  direkt çıkıyor. `w_MSL<0`, `msl_scale_m<=0`, veya
  `vertical_reversal_cost_weight<0` durumunda otomatik olarak eski
  Euclidean heuristic'e (multiplier=1.0) fallback ediyor —
  `astar_search(..., use_msl_lower_bound_heuristic=True)` (default True).
  5 validation (unit test'ler, numeric new_h>=old_h, **9864 (state,
  primitive) kombinasyonunda 0 consistency ihlali**, OFF/ON aynı optimal
  cost, sentetik benchmark'ta expanded_nodes %61-95 arası düştü) ALL
  PASS. **Gerçek Aladağlar benchmark'ı (cache ON, dominance OFF, aynı
  Stage 15 senaryosu):** OFF fresh baseline ~27s/30,000 expanded/
  `search_limit_reached` (Stage 15'in 21.6s'sini doğruladı) → **ON: 1.95s,
  1776 expanded, `status=success`** — **arama ilk kez 30k cap'ine hiç
  ulaşmadan gerçek optimal path'i buldu** (`total_cost=1949.4`, Stage
  12'nin bağımsız olarak bulduğu değerle birebir eşleşiyor,
  `multiplier=1.665`, `min_possible_msl=2660` — bu senaryoda search
  bound'unun kendisi terrain-tabanlı floor'dan daha sıkı çıktı).
  **Sınıflandırma: (A) Büyük kazanç.**

- **Stage 18 — w_MSL kalibrasyonu / vertical path profile analizi
  (pure analysis, mimari değişikliği YOK).** `w_MSL ∈ {0.25 baseline,
  0.32 conservative, 0.63 balanced, 1.01 aggressive}` (Stage 13'ün
  analytical break-even adayları), cache ON + heuristic ON + dominance
  OFF sabit. Gerçek Aladağlar'da (aynı Stage 13/17 senaryosu) **4/4
  weight SUCCESS, runtime 1.97s-8.70s arası** (hiçbiri 15s'yi geçmedi
  veya cap'e gitmedi): `0.25`→hiç dalmadı; `0.32`→20m dalış
  (`first_descent_m=0`, hemen); `0.63` ve `1.01`→**ikisi de aynı 40m
  dalışta saturasyona uğradı** (daha fazla weight daha derin dalış
  vermedi — sabit 38-kolon bütçesinde k arttıkça climb/descent'e giden
  kolon sayısı da arttığından marjinal fayda azalıyor). **Hiçbir
  weight'te zigzag/roller-coaster yok** — her başarılı dalışta
  `reversals=1`, spacing-sensitive reversal modeli davranışı temiz
  tutuyor. `first_descent_m=0` her zaman (planner her zaman erken
  dalıyor, hiç geç kalmıyor) — istenen "erken descend + uzun low-cruise"
  profili sentetik VE gerçek terrain'de doğrulandı. Cost decomposition
  (`G+w*M+R`) her run'da production `compute_edge_cost` ile bit-exact
  eşleşti. Config default'u DEĞİŞTİRİLMEDİ — üç aday (conservative 0.32,
  balanced 0.63, aggressive 1.01≈0.63 ile aynı davranış) raporlandı,
  karar kullanıcıda.

- **Stage 19 — 6.48 km gerçek HIGH→VALLEY→HIGH terrain benchmark'ı
  (row=48→264, col=276): koordinatlar doğrulandı (start=3530.5m,
  goal=3539.5m, distance=6480.0m exact, valley min=3036.3m, relief=520.5m,
  0 NoData — hepsi beklenen değerlerle eşleşti), ama arama SUCCESS
  OLMADI.** `w_MSL=0.63` (bu run'a özel, default değiştirilmedi), aircraft
  MSL=3760m (start/goal AGL 229.5/220.5m, en kötü intervening AGL=203.2m
  — hepsi ≥200 güvenli), search bounds `[3240,3780]` (**27 z-step**, Stage
  17'nin ~10 step'ine göre çok daha geniş). **Dominance OFF: 30.000
  expansion/24.3s/`search_limit_reached`, `max_open=92,513`. Dominance ON:
  30.000 expansion/31.6s/`search_limit_reached`, `max_open=71,553` (yine
  de %22.7 daha düşük, ama hâlâ çözmüyor). 150.000 expansion'a çıkarılan
  ek deneme de 106.6s'de `search_limit_reached` kaldı** (`max_open=
  323,328`) — bu noktada tekrar denemeyi durdurdum. **Path bulunamadığı
  için hiçbir vertical-profile metriği (first_descent, min_MSL, dwell
  ratio vb.) raporlanamadı** — bu adımın "A mı B mi" sorusu (kısa
  benchmark mı yoksa 10° sistem mi yetersiz) **yanıtlanamadı**, çünkü
  ikisi de path verisi gerektiriyor. **Gerçek bulgu farklı ve yeni**:
  Stage 17'nin MSL-aware heuristic'i (multiplier=3.04 burada, Stage
  17'nin 1.665'inden bile daha güçlü) bu ölçekte YETERSİZ — darboğaz
  artık "heuristic zayıf" değil, **state-space'in ham büyüklüğü**
  (27 z-step × 216 satırlık uzun koridor × 33 trend/age varyantı,
  tahmini ~60× Stage 17'nin state-space'inden büyük). Bu, gelecekte
  uzun rotalar için ya daha güçlü/terrain-aware bir heuristic ya da
  search-space'i daraltan başka bir mekanizma gerektiğine işaret ediyor
  — ama bu adımın kapsamı dışında, sadece not edildi.

- **Stage 20 — Stage 19'un çözülememesinin kök nedeni kesin olarak
  teşhis edildi: fiziksel feasibility DEĞİL, low-MSL optimization'ın
  yarattığı search explosion.** 4 diagnostic test (mimari değişikliği
  yok): **A) direct 216-edge level chain** (A* kullanılmadan, tek tek
  `evaluate_primitive()`) → **216/216 VALID, min_AGL=203.2m** — feasible
  path'in var olduğu kanıtlandı. **B) LEVEL-only A*** (primitive set
  sadece 8 level primitive'e filtrelendi, production kodu değişmedi) →
  **SUCCESS, 217 expanded, 0.20s**, `geom_len=6480.0` (direct chain'le
  birebir). **C) full primitive set + w_MSL=0.0** (bu run'a özel, default
  değişmedi) → **SUCCESS, 217 expanded, 1.11s**, aynı düz rota (0 climb/
  descent, 0 reversal) — climb/descent primitive'leri mevcut olsa da MSL
  isteği olmadığı için hiç kullanılmadı. **D) full primitive set +
  w_MSL=0.63** → Stage 19'da zaten kurulan sonuç: 30k'da 24.3s, 150k'da
  106.6s, ikisi de `search_limit_reached`. **Sonuç: CASE 1 tam olarak
  doğrulandı** — "Terrain/10°/feasibility problemi değil. Low-MSL
  optimization ile oluşan search explosion ana problem." 10° primitive
  açı limiti hiçbir şekilde sorumlu değil (B ve C testleri zaten
  10°'lik climb/descent primitive'leri erişilebilir haldeyken bile
  sorunsuz, hızlı çözüldü). Sonraki müdahale w_MSL/heuristic/search
  katmanında olmalı, primitive-angle katmanında değil.

- **Stage 21 — generic (config-driven, açıya özel olmayan) vertical-
  reachability MSL lower-bound heuristic eklendi; matematiksel olarak
  hem admissible hem consistent kanıtlandı, ama gerçek 6.48km benchmark'ı
  hâlâ çözülemedi (sınıflandırma: C, yetersiz).** `h_forward`, relaxed
  bir problem olarak türetildi: kalan D3D mesafesi boyunca, irtifa
  `sin(config.max_descent_angle_deg)` oranından daha hızlı düşemez ve
  global floor'un altına inemez — bu "en iyimser" zarfın cost-rate
  integrali (`∫(1+w_MSL*scaled(z_lb(s)))ds`, 0..D) analitik/piecewise-
  linear olarak (en fazla 2 breakpoint, trapezoid — sampling loop YOK)
  hesaplanıyor. **Admissibility kanıtı**: gerçek path uzunluğu L>=D
  olsa da (lateral detour dahil), integrand her zaman ≥1 olduğu için
  [0,D]'ye kesmek asla overestimate etmiyor. **Consistency kanıtı**:
  herhangi bir gerçek edge (n,n') için, n''nin kendi optimal relaxed
  devamını bu edge'le başa eklemek n'nin KENDİ relaxed problemi için
  geçerli bir aday oluşturuyor (edge'in kendi açısı zaten limiti
  aşmadığından) → `h(n) <= edge_cost + h(n')` direkt çıkıyor. Yalnızca
  `config.max_descent_angle_deg` okuyor — **10°'ye özel kod YOK**,
  gelecekte 20°/30°/continuous 0..30° sistemine geçilirse bu heuristic
  değişmeden çalışır (7.5°/13.3°/22.5°/28.9° gibi primitive-dışı açılarla
  test edildi). Backward (goal-side climb) zarfı bu adımda
  **implement edilmedi** — "güvenlik > güç" ilkesiyle bilinçli olarak
  atlandı (consistency kanıtı bu turda tam netleştirilemedi).
  `astar_search(..., use_vertical_reachability_heuristic=True)` (default
  True), `max(h_global, h_forward)` olarak Stage 17'nin heuristic'iyle
  birleştiriliyor. 4 validation (açı-monotonluk 5°≥10°≥20°≥30°,
  continuous-uyumluluk, **9864 kombinasyonda 0 consistency ihlali**,
  OFF/ON aynı optimal cost, sentetik explosion testinde expanded_nodes
  %21.1 azaldı) ALL PASS. **Ama gerçek 6.48km/520m-relief benchmark'ında
  (w_MSL=0.63): 30k cap'te `search_limit_reached` (max_open=138,151,
  Stage 19'un 92,513'ünden DAHA YÜKSEK), 150k cap'te de
  `search_limit_reached` (max_open=377,653, Stage 20'nin 323,328'inden
  daha yüksek).** Yani bu spesifik ölçekte yeni heuristic performansı
  İYİLEŞTİRMEDİ, hatta bu metriklere göre hafifçe kötüleşti — **sınıflandırma
  C (yetersiz)**. Stage 20'nin kök-neden teşhisi (low-MSL search
  explosion) hâlâ geçerli; global+forward-envelope MSL heuristic'i tek
  başına bu ölçek için yeterli değil. Talimat gereği yeni bir heuristic
  bu turda implement edilmedi — sıradaki adaylar (terrain-aware 2D
  lower-bound, Dijkstra/wavefront, corridor/coarse planning) not edildi,
  henüz uygulanmadı.

- **Stage 22 — `trend_age_units` (0..10, 11 değer) production search'ten
  çıkarıldı, yerine 3 değerli `trend_age_bucket` (SHORT/MEDIUM/MATURE)
  geldi; amaç state-space'in kendisini küçültmekti (heuristic değil).**
  Bucket eşikleri yeni bir config alanı EKLEMEDEN, mevcut
  `reversal_relax_distance_m`'den türetildi: MATURE eşiği = 300m
  (`reversal_relax_distance_m`), MEDIUM eşiği = 150m (yarısı). Reversal
  factor: SHORT=1.0, MEDIUM=0.5, MATURE=0.0 (`reversal_cost =
  vertical_reversal_cost_weight * |dz| * factor`). Bucket geçiş kuralı:
  taze trend/reversal → `bucket_of(bu primitive'in kendi horizontal
  distance'ı)` (mevcut 24-primitive setinde bu her zaman SHORT, çünkü en
  uzun primitive ~127m); her CONTINUATION adımı (climb/climb,
  descent/descent, veya bir trend sürerken LEVEL) primitive'in kendi
  uzunluğuna bakmadan **tam bir bucket ilerletir** (MATURE'da tavan).
  LEVEL hiçbir zaman trend'i resetlemiyor, ama bir trend sürerken onun
  maturity'sini ilerletmeye devam ediyor (spesifik istendiği gibi).
  Eski tam (0..10) model `_next_trend_and_age` olarak, hiç silinmeden,
  sadece "LEGACY/reference" diye yeniden docstring'lenerek bırakıldı.
  **Açıkça ifşa edilmiş bir approximation var**: SHORT→MEDIUM geçişi bir
  adım sonra asla erken olamaz (min gerçek mesafe 120+30=150m, eşikle
  aynı), ama MEDIUM→MATURE bir adım sonra kısa/LEVEL adımlardan oluşan
  zincirlerde erken olabilir (örn. descent(120)+level(30)+level(30)=180m
  gerçek mesafede yeni model MATURE (0 penalty) diyor, eski model hâlâ
  MEDIUM (spacing_ratio=0.6, kısmi penalty) diyordu) — bu sayısal
  farkla birlikte `scripts/validate_trend_bucket.py`'de raporlandı.
  Cache key `(row,col,z_index,primitive_id)` değişmedi (bucket'tan
  etkilenmiyor). **10 validation (A-G behavior sequences) ALL PASS**:
  continuous climb/descent hâlâ 0 penalty, kısa spacing tam penalty
  (SHORT), orta spacing yarım penalty (MEDIUM), uzun spacing (LEVEL ile
  MATURE'a ilerlemiş) 0 penalty, LEVEL hiçbir zaman reversal'ı
  gizlemiyor/resetlemiyor. **Roller-coaster testi PASS**: kısa zigzag
  (des,climb,des,climb) toplam reversal penalty=60.0 > doğal uzun
  reversal (des,des,climb,climb) toplam penalty=10.0. **Eski/yeni
  davranış karşılaştırması**: davranış SINIFLARI korundu (continuous
  ucuz, kısa pahalı, roller-coaster > doğal uzun reversal, her ikisinde
  de) — ama tam sayılar farklı (örn. tek `des,climb` çifti: eski=12.0,
  yeni=20.0 çünkü eski model 120m'yi sürekli oranla 0.6 factor'e
  karşılık gelirken yeni model onu direkt SHORT/factor=1.0 sayıyor).
  **Sentetik state-explosion testinde (55×3 dar koridor, Stage-16 tipi)
  NEW belirgin şekilde daha hızlı**: expanded 833 (eski 3852, **%78.4
  azalma**), runtime 485ms (eski 5.26s, ~11x hızlı), unique_states
  eski'de 4203'tü (NEW'de dominance/age-yapısı farklı olduğu için
  doğrudan karşılaştırılabilir tek sayı yok, ama expanded/max_open/
  runtime üçü de NEW'de düşük), **aynı optimal cost (2593.32 == 2593.32)
  — path safety/optimallik approximation'dan etkilenmedi bu testte.**
  **AMA gerçek 6.48km/520m-relief benchmark'ında (aynı ayarlar: w_MSL=
  0.63, dominance OFF, msl_lower_bound_heuristic ON, vertical_
  reachability_heuristic OFF, cache ON) kazanç YOK, hatta hafif
  regresyon var**: 30k cap'te `search_limit_reached`, `max_open=100,280`
  (Stage 19'un eski-model baseline'ı 92,513'ünden **%8 DAHA YÜKSEK**),
  wall=39.58s (Stage 19'un 24.3s'inden daha yavaş). 150k cap retry'da da
  `search_limit_reached`, `max_open=355,681` (Stage 20'nin eski-model
  baseline'ı 323,328'inden **%10 DAHA YÜKSEK**), wall=263.53s (Stage
  20'nin 106.6s'inden **~2.5x daha yavaş**). **Kök neden wall-clock için
  net**: primitive cache hit-rate düştü (0.775/0.797, Stage 19-21'in
  0.85-0.90 aralığından belirgin daha düşük) — yani NEW model, aynı
  expanded-node sayısı için, farklı primitive-cache anahtarlarına daha
  fazla çeşitlilikte uğruyor (dominance pruning OFF olduğu için bu
  çeşitliliği hiçbir şey toplamıyor). max_open'daki hafif artış da (state
  -space'in KENDİSİ, sentetik testin tersine, bu ölçekte küçülmedi)
  gösteriyor ki bucket compression'ın küçük-sentetik testte gördüğü
  kazanç bu büyük/gerçek terrain'e genellemiyor — **sınıflandırma: C
  (yetersiz, path bulunamadı, state-space bu ölçekte küçülmedi/hafifçe
  büyüdü)**. Path bulunamadığı için hiçbir path-metriği (min_MSL,
  first_descent, dwell ratio, "erken iniş→uzun düşük-MSL cruise→geç
  tırmanış" profili vb.) bu adımda da raporlanamadı — Stage 19-21'le
  aynı sınırlama. Hard safety (AGL≥200, primitive açı≤10°, NoData/bounds)
  bu approximation'dan etkilenmedi (evaluate_primitive/evaluate_agl/
  evaluate_transition hiç değişmedi, sadece cost/state katmanı
  değişti). **Sonuç: trend_age büyük olasılıkla bu spesifik 6.48km
  benchmark'ının state-explosion'ının TEK/ANA kaynağı değildi** — onu
  3'e sıkıştırmak küçük sentetik testte devasa kazanç, ama bu büyük
  gerçek terrain'de ölçülebilir kazanç sağlamadı; asıl darboğaz (Stage
  20'nin teşhisiyle uyumlu) hâlâ low-MSL search'ün ham genişliği/
  heuristic'in bu ölçekte yetersizliği gibi görünüyor. Dominance pruning
  ile 3-bucket'ı BİRLİKTE test etmek mantıklı bir sıradaki adım olarak
  not edildi (bucket sayısı 3'e indiği için dominance frontier'ları artık
  çok daha basit/ucuz olurdu, ve tam da bu adımda gözlenen "cache
  hit-rate düşüşü/çeşitlilik artışı"nı toplayabilir) — bu turda
  **implement edilmedi**, kapsam dışı.

- **Stage 23 — 3-bucket state + exact dominance pruning (Stage 16'nın
  hiç değişmeyen `_dominates`/frontier koduyla) birlikte test edildi;
  sentetikte kazanç var ama gerçek 6.48km benchmark'ında Stage 16'nın
  aynı örüntüsü tekrarlandı: max_open küçüldü, wall-clock kötüleşti
  (sınıflandırma: C).** Dominance mekanizması Stage 16'dan beri 5.
  tuple elemanını genel "age-benzeri sıra" olarak işliyordu — Stage
  22'de bucket'a geçilince hiç yeniden doğrulanmamıştı. Bu adımda önce
  monotonicity kanıtlandı: aynı trend için, ortak herhangi bir gelecek
  primitive sequence'inde, daha mature bucket'ın kümülatif reversal
  cost'u asla daha düşük-mature'dan yüksek olamıyor — LEVEL/continuation
  adımları sırayı koruyor (`_advance_bucket` monoton), ve bir reversal
  anında `_REVERSAL_FACTOR_BY_BUCKET` bucket'a göre monoton düşüyor
  (SHORT=1.0≥MEDIUM=0.5≥MATURE=0.0); kritik olarak **reversal'dan sonra
  next_bucket sadece o primitive'in kendi mesafesine bakıyor, önceki
  bucket'tan tamamen bağımsız** — yani bir reversal'dan sonra iki
  trajectory'nin bucket'ı ve dolayısıyla TÜM gelecek maliyeti birebir
  eşitleniyor (Stage 16'nın age-reset kanıtının birebir analogu).
  **14,424 sequence × 2 başlangıç trend'i (uzunluk≤3, 24 primitive'in
  tam kombinasyonu) + 5 isimlendirilmiş senaryo (LEVEL→LEVEL, same-trend
  continuation, direct reversal, LEVEL-sonrası-reversal, reversal-sonrası
  -continuation) üzerinde 0 ihlal.** Dominance predicate'i (`_dominates`)
  hiç değiştirilmeden section 3'ün kuralıyla (`g_a<=g_b AND
  bucket_rank_a>=bucket_rank_b`) zaten birebir örtüşüyordu — 5 unit test
  (A-E, farklı vertical_trend'lerin asla karşılaştırılmadığı dahil) ALL
  PASS. **Sentetik state-explosion testinde (w_MSL=0.63, aynı Stage 22
  senaryosu) dominance ON belirgin iyileşme verdi**: expanded 1052→783
  (**%25.6 azalma**), max_open 674→301 (**%55.3 azalma**), runtime
  565.5ms→511.8ms (**0.91x, hafif iyileşme**), aynı optimal cost
  (3604.19==3604.19), `max_frontier=3` (teorik üst sınır, 3 bucket
  olduğu için) — bookkeeping burada gerçekten ucuz kaldı. Bu, %25 eşik
  kapısını geçti → gerçek benchmark'a geçildi (talimat gereği SADECE
  30k cap, retry YOK). **Gerçek 6.48km benchmark'ında (dominance ON,
  30k cap): `search_limit_reached`, wall=68.78s, max_open=79,912,
  cache_hit_rate=0.686, dominance checks=193,553/pruned=50,493/
  frontier_removed=81,200/pop_skipped=3,672, max_frontier=3
  (avg=1.676).** Stage 22'nin aynı benchmark'ının dominance-OFF 30k
  sonucuyla (`max_open=100,280`, wall=39.58s, cache_hit=0.775)
  kıyaslandığında: **max_open %20.3 azaldı (anlamlı ama sentetiğin
  %55'inden çok daha düşük), ama wall-clock %74 arttı (1.74x daha
  yavaş)** — hem bookkeeping (193k check) hem de düşen cache hit-rate
  (0.775→0.686, daha fazla gerçek `evaluate_primitive()` çağrısı)
  birlikte bunu açıklıyor; `max_frontier=3` küçük olsa da toplam check
  HACMİ (30k expansion × ~24 primitive × dominance-check) bu ölçekte
  yine de pahalı. **Stage 16'nın eski (0..10 age) modelle bulduğu aynı
  örüntü — "max_open küçülüyor ama capped/non-converging gerçek
  benchmark'ta wall-clock kötüleşiyor" — bucket'ın çok daha ufak
  frontier'ıyla (max 3 vs eskiden potansiyel 11) bile TEKRARLANDI.**
  **Sınıflandırma: C (az/negatif kazanç)** — state metriği iyileşti ama
  runtime kötüleşti, 30k'da hâlâ search_limit_reached. Path bulunamadığı
  için path-metriği raporlanamadı (Stage 19-23 boyunca aynı sınırlama).
  Bu turda YAPMAZ listesi (incumbent/branch-and-bound, 30°/variable
  angle, bucket/cost/heuristic değişikliği, terrain-aware/Dijkstra/
  corridor/GPU) hiçbiri implement edilmedi — sadece 3-bucket+dominance
  etkisi izole ölçüldü.

- **Stage 24 — incumbent upper-bound / branch-and-bound pruning eklendi
  (`astar_search(use_incumbent_pruning=...)`, `validate_and_cost_path()`);
  sentetikte doğru ve faydalı çalıştığı kanıtlandı, ama gerçek 6.48km
  benchmark'ında elimizdeki incumbent (direct level path) hiçbir candidate'i
  prune etmedi — sınıflandırma: Durum 4 (incumbent tek başına yetersiz).**
  Mekanizma tamamen genel: `validate_and_cost_path(path, primitives,
  terrain, config)` herhangi bir path'i `evaluate_primitive()` ile
  doğruluyor ve `compute_edge_cost`/`_next_trend_and_bucket` ile GERÇEK
  production formülüyle maliyetliyor (ayrı/yaklaşık formül yok); geçersiz
  bir edge varsa `(False, inf)` döner. Pruning kuralı (`g+h>=incumbent →
  prune`) admissibility'den direkt çıkıyor: `h<=h*` olduğu için
  `g+h<=g+h*=` o branch'teki HERHANGİ bir complete solution'ın gerçek
  maliyeti; incumbent geçerli bir complete solution olduğu için
  `g+h>=incumbent` ise branch optimumu iyileştiremez — **exact, epsilon
  yok**. `>=` (eşitlik dahil) kullanmak sadece eşit-cost ALTERNATİF
  path'leri bulmayı feda ediyor, OPTIMAL COST'u değil (proje zaten tek
  bir optimal cost arıyor). İki noktada uygulanıyor: **A) candidate
  generation** (`f=g+h>=incumbent` ise heap'e hiç push edilmiyor —
  `incumbent_pruned_candidates`) ve **B) heap pop zamanı** (`current in
  closed` kontrolünden SONRA, dominance kontrolünden ÖNCE: pop edilen
  entry stale değilse ve f'i incumbent'ı geçiyorsa, bu min-heap pop
  sırası nedeniyle OPEN'daki HER ŞEYİN de öyle olduğunu kanıtlıyor —
  `incumbent_heap_pops_skipped=1` sayılıp **hemen search TERMINATE
  ediliyor** (`incumbent_termination_triggered=True`), kalan heap'i tek
  tek boşaltmak yerine). A ve B ayrık kümeler üzerinde çalışıyor (A'da
  prune edilen hiç push edilmiyor, dolayısıyla B'ye hiç ulaşamıyor) —
  **çift sayım yapısal olarak imkânsız**. Mevcut consistent-A*
  goal-pop davranışı (fiziksel goal'a ulaşan İLK pop = kanıtlanmış
  optimal) hiç değişmedi — incumbent sadece bu noktaya ulaşmadan ÖNCE
  hangi dalların araştırılacağını buduyor; search kendi içinden daha iyi
  bir complete solution bulursa (`g_score[current]<incumbent_cost` goal
  pop anında) incumbent güncelleniyor (`incumbent_updates`) ama bu zaten
  var olan erken-çıkış noktasında oluyor, yeni bir termination şekli
  DEĞİL. **10 unit test (temel bound: KEEP/PRUNE/PRUNE/incumbent=inf→
  KEEP) ALL PASS. 11 (geçersiz initial path: AGL ihlali olan bir path
  doğru şekilde reddedildi, `incumbent_cost=inf`, search mimari bağımlılık
  olmadan normal devam edip feasible path buldu) PASS. 12 (incumbent
  update: initial=1762.20 → search 1755.91 buldu, 1 update sayıldı) PASS.
  13 (optimality: incumbent OFF/ON aynı optimal cost, 2193.06==2193.06)
  PASS.** Sentetik 4-way testte (Stage 22/23'ün aynı state-explosion
  senaryosu, ama initial incumbent geçerli olsun diye 1420m sabit
  irtifalı bir level chain kullanıldı): **A (ikisi OFF)=679 expanded/369
  max_open, B (dominance ON)=494/271, C (incumbent ON)=679/237 (expanded
  DEĞİŞMEDİ ama max_open %35.8 küçüldü — incumbent sadece "ölü" adayları
  OPEN'a hiç girmeden eliyor, gerçekte expand edilenleri değiştirmiyor),
  D (ikisi ON)=494/187 — dört run da AYNI optimal cost (2717.84)**.
  **Interaction analizi (15): C→D expanded %27.2, max_open %21.1 azaldı,
  runtime 0.95x (hafif iyileşme) — incumbent aktifken dominance artık net
  faydalı görünüyor (sentetikte).** Bu eşik (%25 azalma VEYA net runtime
  iyileşmesi) geçildiği için gerçek benchmark'a geçildi. **Gerçek 6.48km
  benchmark'ında initial incumbent (216 edge'lik direct level chain,
  `validate_and_cost_path` ile doğrulandı) cost=21,829.82 çıktı. RUN A
  (incumbent ON, dominance OFF, 30k cap): `search_limit_reached`,
  expanded=30,000, max_open=100,280, cache_hit=0.775 — **Stage 22'nin
  incumbent'sız aynı ayarlı çalışmasıyla BİREBİR AYNI** (expanded/
  max_open/cache_hit_rate rakamları tıpatıp eşleşiyor). RUN B (incumbent
  ON, dominance ON): `search_limit_reached`, expanded=30,000,
  max_open=79,912, cache_hit=0.686, dominance checks=193,553/
  pruned=50,493 — **Stage 23'ün incumbent'sız çalışmasıyla da BİREBİR
  AYNI**. **İki run'da da `incumbent_pruned_candidates=0,
  incumbent_heap_pops_skipped=0, incumbent_updates=0,
  incumbent_termination_triggered=False`** — yani bu 30k bütçe içinde
  incumbent bound (21,829.82) TEK BİR candidate'i bile prune etmedi.
  Kök neden: w_MSL=0.63 ile 3760m sabit irtifada kalan direct-level path
  çok pahalı bir upper bound (aracın kendi irtifasında tüm mesafeyi
  geçirdiği için maksimum MSL cezasını her adımda ödüyor); search vadiye
  inen dalların g'si zaten çok daha düşük başladığından, 30,000
  expansion boyunca hiçbir explored candidate'in f'i bu gevşek üst
  sınıra hiç yaklaşmadı. **Wall-clock RUN A=105.15s (Stage 22'nin
  39.58s'ine göre çok daha yüksek), RUN B=91.06s (Stage 23'ün 68.78s'ine
  göre yüksek) — ama expanded/max_open/cache_hit_rate/dominance
  istatistikleri BİREBİR AYNI olduğu için bu, koddan kaynaklanan gerçek
  bir regresyon OLAMAZ (yapılan iş matematiksel olarak identik); bu
  oturumda benchmark'ı izlemek için art arda çalıştırılan çok sayıda
  PowerShell tanı komutunun aynı makinede CPU'yu paylaşmasından
  kaynaklanan bir ÖLÇÜM KİRLİLİĞİ (confound) olarak işaretlendi — temiz/
  izole tekrar koşulmadıkça wall-clock kıyaslaması güvenilir değil, ama
  bu incumbent mekanizmasının kendisiyle ilgili bir bulgu değil.**
  **Sınıflandırma: Durum 4 (Section 21) — "İkisi de 30k SEARCH_LIMIT →
  incumbent tek başına yetersiz."** Path bulunamadı, path-metriği
  raporlanamadı. Hard safety incumbent path'in kendisi için de
  `validate_and_cost_path` üzerinden zaten garanti (aynı
  `evaluate_primitive()` otoritesi). Bu turda YAPMAZ listesi (variable
  angle, 30°, bucket/w_MSL/cost/heuristic değişikliği, terrain-aware/
  Dijkstra/corridor/coarse/GPU) hiçbiri implement edilmedi.

- **Stage 25 — Weighted A* (`g+epsilon*h` sadece search ordering için) +
  certified %5-bounded-suboptimal termination (`incumbent<=target*LB`)
  eklendi; sentetikte tamamen doğru/güvenli çalıştığı kanıtlandı, ama
  gerçek 6.48km benchmark'ında 30k'da TEK bir internal solution bile
  bulunamadı — sınıflandırma: C (yetersiz).** İki ayrı f: `f_weighted=
  g+epsilon_search*h` SADECE hangi node'un önce expand edileceğini
  belirliyor; `f_lb=g+h` (unweighted, admissible) incumbent pruning/
  certificate için TEK kullanılan değer — kodda ayrı tutuluyor
  (`open_heap` 4-tuple: `(f_weighted,counter,state,f_lb)`), hiçbir yerde
  `g+epsilon*h` pruning/bound için kullanılmıyor. **Reopening**: weighted
  sırada `f_weighted` artık consistent olmadığından (pop sırası f_lb'de
  monoton değil), zaten closed bir state'e daha iyi g ile ulaşılırsa
  `closed`'dan çıkarılıp tekrar open'a alınıyor (`reopened_states`) —
  `epsilon_search==1.0` iken bu asla tetiklenmiyor (consistency zaten
  garanti ediyor), yani exact mod hiç değişmedi; bu matematiksel
  argümanla kanıtlandı VE Stage 22/23/24'ün ÜÇ validation script'i de
  bu refactor'den SONRA tekrar çalıştırılıp **birebir aynı sayıları
  verdiği doğrulandı** (regresyon yok). **Certificate**: `bounded_mode`
  aktifken (yani `target_suboptimality` verilmişken) search ilk goal
  pop'ta DURMUYOR — incumbent güncelleniyor (`first_solution_*` ilk
  kez kaydediliyor) ve search, her expansion sonrası ayrı bir `lb_heap`
  (aynı candidate'lerin `f_lb` ile push edildiği paralel heap, closed
  olan entry'ler lazy şekilde kalıcı olarak atılıyor) üzerinden
  `LB=min(pending f_lb)` hesaplayıp `incumbent<=target*LB` olduğu anda
  duruyor — `incumbent>=C*` olduğu için bu cebirsel olarak
  `incumbent/C*<=target`'i kanıtlıyor (epsilon_search'ün kendi çok daha
  gevşek worst-case oranına güvenmeden). `target_suboptimality=None`
  iken bu makine tamamen devre dışı (varsayılan davranış = Stage 24
  ile birebir aynı). **Sentetik: 13-15 unit test (weighted priority
  ayrımı, incumbent'ın f_lb kullanması, %5 certificate aritmetiği) ALL
  PASS. 16 (epsilon sweep 1.05/1.2/1.5/2.0, exact C*=2424.20 referansına
  göre): hepsi kendi epsilon sınırının içinde kaldı (örn. epsilon=1.5 →
  ratio=1.0698<=1.5) PASS. 17 (certified: epsilon=1.5, target=1.05):
  `bounded_termination_triggered=True`, final_bound_ratio=1.0479 (<=1.05
  kanıtlandı), ama gerçek sonuç TESADÜFEN tam optimal çıktı
  (returned_cost==C* birebir, actual_ratio=1.0000) — yani certificate
  KONSERVATİF bir kanıt, gerçek kalite daha da iyi olabilir (spec'in
  section 5'teki tam senaryosu).** **Gerçek 6.48km benchmark'ında**
  (initial incumbent=21,829.82, epsilon=1.5, target=1.05, dominance
  OFF, 30k cap): `status=search_limit_reached`, **wall=12.99s** (Stage
  22-24'ün 40-105s aralığına göre ÇOK daha hızlı), **max_open=1061**
  (Stage 22-24'ün ~80,000-100,000'ine göre ~99% daha küçük!),
  cache_hit_rate=0.965 (çok daha yüksek), ama **first_solution_cost=inf
  — 30,000 expansion boyunca fiziksel goal'a TEK BİR KEZ bile
  ulaşılamadı** (`incumbent_updates=0`). `current_lower_bound=19,716.80`
  (initial incumbent'ın 21,829.82'sinden daha düşük, yani vadiye inen
  bazı partial path'lerin g+h'si zaten incumbent'tan ucuz görünüyor —
  önceki stage'lerin "vadiye inmek ucuzlaşıyor" bulgusuyla uyumlu), ama
  `final_bound_ratio=1.1072` (%5 hedefinin üstünde, certificate
  gelmedi). **reopened_states=26,714 / 30,000 expansion — yani
  expansion'ların ~%89'u bir reopen olayıyla çakışıyor: search küçük
  bir bölgede sürekli aynı state'leri daha iyi g ile tekrar açıp
  kapatarak THRASHING yapıyor, goal'a doğru ilerlemek yerine.** Kök
  neden hipotezi (ölçülmedi, sadece not edildi): epsilon=1.5 ile
  agresifleşen `g+epsilon*h` sıralaması, MSL-ağırlıklı heuristic'in
  (multiplier=3.04) baskın olduğu bu maliyet manzarasında "goal'a
  fiziksel mesafe" ile "düşük-MSL'ye erişim" arasında çelişkili bir
  yönlendirme üretebiliyor — search sürekli daha düşük-MSL bölgelere
  dalmayı "daha iyi f" sanıp orada thrashing yapıyor, goal'a fiziksel
  ilerleme sağlamıyor. **Talimat gereği epsilon=2.0 ile retry
  YAPILMADI** (spec'in section 23/24'ü açıkça yasaklıyor). **Sınıflandırma:
  C (yetersiz) — 30k'da direct incumbent'tan daha iyi complete solution
  bile bulunamadı.** Path bulunamadı, path-metriği/hard-safety kontrolü
  bu run için yapılamadı (sadece initial incumbent'ın kendi
  `validate_and_cost_path` garantisi geçerli). Bu turda YAPMAZ listesi
  (terrain-aware 2D, Dijkstra, corridor, coarse, variable angle, 30°,
  GPU, yeni cost, w_MSL tuning, bucket değiştirme, ARA*/decreasing-
  epsilon) hiçbiri implement edilmedi.

- **Stage 26 — epsilon sweep (1.20, 1.10), kod değişikliği YOK (sadece
  benchmark parametresi). epsilon küçüldükçe reopen ORANI düşmedi (hatta
  1.1'de arttı), ama epsilon=1.1'de search İLK KEZ fiziksel goal'a ulaştı
  ve 685 expansion / ~2.2s'de bir improving solution buldu — üç
  epsilon'dan hiçbiri A/B/C kategorilerine tam uymuyor, karma/nüanslı bir
  sonuç.** Karşılaştırma tablosu (aynı incumbent=21,829.82, target=1.05,
  30k cap, dominance OFF):
  ```
  eps   status              expanded  runtime  max_open  reopened  reopen%  1st_sol  incumbent   LB        ratio
  1.50  search_limit_reached  30000    12.99s     1,061    26,714    89.0%     no     21829.82  19716.80  1.1072
  1.20  search_limit_reached  30000    27.08s     3,303    26,649    88.8%     no     21829.82  19716.80  1.1072
  1.10  search_limit_reached  30000    36.30s   108,341    28,488    95.0%    YES     21668.85  19758.88  1.0967
  ```
  **epsilon=1.2, epsilon=1.5'in neredeyse birebir kopyası**: aynı
  current_lower_bound (19,716.80), aynı final_bound_ratio (1.1072), aynı
  reopen oranı (~%89), first solution yok — search yine SAME küçük
  bölgede thrashing yapıyor, epsilon'un 1.5→1.2 küçülmesi hiçbir niteliksel
  fark yaratmadı. **epsilon=1.1'de nitelik değişti**: max_open 108,341'e
  fırladı (Stage 22-24'ün eski-model ~%80-100k aralığına yakın —
  search artık çok daha GENİŞ bir alanı tarıyor, dar bölgede sıkışmıyor),
  reopen_ratio bile arttı (%95.0) ama bu sefer YARARLI reopen'lar (57
  incumbent update, first_solution_expanded=685'te bulundu — sadece
  ~2.2 saniyede!). **Bulunan path**: cost=21,668.85 (direct incumbent'a
  göre sadece **%0.74 iyileşme** — min_MSL=3660m (sadece 100m inip 100m
  çıkıyor), son 120m'de tırmanışa geçiyor (`last_climb_start_distance_m
  =6360.0`), 1 reversal, **SAFETY PASS** (min_AGL=203.2m, max_angle=
  9.46°). Bu, Stage 19-21'in aradığı "derin vadiye dalış" (520m relief)
  DEĞİL — yüzeysel/küçük bir düzeltme; `current_lower_bound=19,758.88`
  ile `final_incumbent=21,668.85` arasındaki gap (%9.67) tam da bunu
  gösteriyor: daha iyi bir çözüm (muhtemelen derin vadi rotası) hâlâ
  kanıtlanabilir şekilde mevcut, ama 30k cap içinde bulunamadı. **%5
  certificate hiçbirinde gelmedi.** Section 12'nin A/B/C kriterleri
  literal olarak hiçbirine tam uymuyor: A değil (reopen ratio düşmedi),
  C değil (1.1'de first solution bulundu), B'ye yakın ama "goal
  bulunmuyor" öncülü 1.1 için yanlış. **Yorum**: epsilon 1.5→1.2 aralığı
  aynı davranışı veriyor (etkisiz bölge), ama 1.2→1.1 arasında keskin bir
  GEÇİŞ var — search'in "dar bölgede sıkışma" modundan "genişçe tarama +
  hızlı-ama-yüzeysel çözüm bulma" moduna geçtiği bir eşik bölgesi
  (muhtemelen 1.1-1.2 arası) olduğuna işaret ediyor. Section 7'nin "goal'a
  fiziksel ilerleme" diagnostiği **implement edilmedi** (astar.py
  değişikliği gerektirdiği için spec'in kendi escape-clause'u kullanıldı,
  açıkça not edildi). Section 13 YAPMAZ listesi (epsilon=1.05/2.0 real
  run, ARA*, terrain-aware, dominance ON, vb.) hiçbiri implement edilmedi.

- **Stage 27 — epsilon=1.05 final test, kod değişikliği YOK. Sonuç: state
  explosion GERİ GELDİ — ε=1.10'un bulduğu tek çözüm bile bu kez
  bulunamadı, max_open Stage 22-24'ün eski-exact-model baseline'ını bile
  aştı. Sınıflandırma: Durum C (weighted A* aşamasını kapat, resolution/
  coarse-to-fine'a geç).** Karşılaştırma tablosu (aynı incumbent=
  21,829.82, target=1.05, 30k cap, dominance OFF):
  ```
  eps   status              exp@goal  runtime  expanded  max_open  reopen%  incumbent   LB        ratio    min_MSL
  1.50  search_limit_reached    --     12.99s    30000      1,061   89.0%    21829.82  19716.80  1.1072      --
  1.20  search_limit_reached    --     27.08s    30000      3,303   88.8%    21829.82  19716.80  1.1072      --
  1.10  search_limit_reached   685     36.30s    30000    108,341   95.0%    21668.85  19758.88  1.0967    3660.0
  1.05  search_limit_reached    --     64.53s    30000    221,404   59.8%    21829.82  20012.07  1.0908      --
  ```
  ε=1.05'te: `first_solution_found=False` (ε=1.10'un tersine, HİÇ bir
  fiziksel goal pop'u olmadı), `incumbent_pruned_candidates=0` (ε=1.10'un
  104,607'sine karşı — incumbent bound bu genişlikte artık HİÇ işe
  yaramıyor, her candidate'in f_lb'si incumbent'ın altında kalıyor),
  `max_open=221,404` (ε=1.10'un ~2 katı, ve Stage 22-24'ün eski-exact-model
  baseline'larının ~80-100k'sını da AŞIYOR), `runtime=64.53s` (ε=1.10'a
  göre **%78 daha yavaş**), `cache_hit_rate=0.893` (0.981'den düştü —
  search artık çok daha dağınık bir alanı tarıyor, aynı hücrelere daha az
  geri dönüyor). Tek "iyileşen" metrik `reopen_ratio` (%59.8, ε=1.10'un
  %95.0'ından ve ε=1.5/1.2'nin ~%89'undan daha düşük) — ama bu bir kazanç
  değil, sadece search'ün artık DAHA GENİŞ, daha az tekrar-ziyaret edilen
  bir alanı ilk kez tarıyor olmasının (klasik unweighted-A* tarzı geniş
  keşif) yan etkisi. `current_lower_bound=20,012.07` (üç öncekinden daha
  sıkı) ve `final_bound_ratio=1.0908` sayısal olarak ε=1.10'unkinden
  (1.0967) bile hafifçe daha iyi görünüyor — **ama bu YANILTICI**: ε=1.05
  hiçbir zaman incumbent'ı iyileştiremedi (hâlâ 21,829.82, direct level
  path), oran sadece LB'nin yükselmesinden geliyor; ε=1.10 gerçek, daha
  düşük bir incumbent (21,668.85) üretti. **Sonuç açık**: ε=1.5→1.2→1.10
  arası gözlenen "dar-bölge sıkışma → geniş-tarama+hızlı-yüzeysel-çözüm"
  geçişi, ε=1.10→1.05'te DEVAM ETMİYOR — tam tersine geri sıçrıyor,
  klasik state-explosion'a dönüyor (Section 12'nin **Durum C**'si tam
  isabetli: "state explosion geri geliyor, goal yine zorlaşıyor, ciddi
  runtime artışı"). **ε≈1.10, bu 6.48km problemi için gözlenen dar
  pencere içindeki tek "iyi" nokta gibi görünüyor** — ne 1.05 (çok geniş/
  yavaş) ne 1.5/1.2 (çok dar/sıkışık) onun bulduğu (yüzeysel de olsa)
  gerçek çözümü üretebildi. Goal-progress diagnostiği bu turda da
  (talimat gereği) eklenmedi. Section 13 YAPMAZ listesi (1.075/1.08/1.15/
  1.00 rerun, ARA*, terrain-aware, resolution, cost/w_MSL/dominance/
  heuristic değişikliği, GPU) hiçbiri implement edilmedi.

- **Stage 28 — Cost Function Validation / Route Ranking Test: search YOK, cost
  formülü/weight DEĞİŞMEDİ. 6 candidate rota (aynı 6.48km/w_MSL=0.63 gerçek
  Aladağlar senaryosu, START row=48,GOAL row=264,col=276) elle inşa edilip
  `validate_and_cost_path()`/`decompose_cost()` (gerçek production
  fonksiyonları) ile değerlendirildi. Sonuç: CASE A -- cost function doğru,
  ana darboğaz SEARCH.** 6/6 candidate VALID (C ve F ilk denemede terrain
  nedeniyle INVALID çıktı -- START'a 2 satır yakın terrain peak'i, ~3556.8m,
  200m AGL için >=3756.8m MSL gerektiriyor; descent'e hemen satır 48'de
  başlamak bunu ihlal ediyordu; 4 satırlık (120m) zorunlu güvenlik buffer'ı
  eklenince VALID oldu -- gerçek terrain'den ölçülmüş bir kısıt, tasarım
  tercihi değil). Sonuçlar (total_cost, ucuzdan pahalıya):
  ```
  C EARLY_DEEP_VALLEY   21,071.25  (G=6539.6 wM=14531.7 R=0.00, min_MSL=3400, depth=360m, 1 reversal)
  D LATE_DESCENT        21,533.92  (G=6539.6 wM=14994.3 R=0.00, min_MSL=3400, depth=360m, aynı G/depth/min_MSL, sadece SIRA farklı)
  B SHALLOW_VALLEY      21,668.85  (Stage 26 eps=1.10'un GERÇEK bulduğu path, min_MSL=3660, depth=100m, 1 reversal)
  A DIRECT_LEVEL         21,829.82 (G=6480.0 wM=15349.8 R=0.00, baseline, Stage 20/24'ün incumbent'ıyla bit-exact)
  E ROLLER_COASTER      22,702.98  (G=6539.6 wM=15463.4 R=700.00, 18x20m osilasyon, 35 reversal)
  F LONG_DETOUR         23,726.23  (G=7384.6 wM=16341.6 R=0.00, C ile AYNI min_MSL=3400 ama +845m detour)
  ```
  **6/6 candidate'ta `total_cost == G + w_MSL*M + R` bit-exact eşleşti**
  (production formülünün decomposition'ı doğrulandı). **KRİTİK bulgu**: C
  (21,071.25), B'den (21,668.85, Stage 26'nın weighted A* eps=1.10 ile
  GERÇEKTEN bulduğu path -- bu adımda bir kerelik, yeni arama değil, sadece
  o sonucun path array'ini diskte hiç kaydedilmemiş olduğu için tekrar
  üretmek amacıyla aynı çağrı bit-bit tekrarlandı) **%2.76 DAHA UCUZ** --
  yani cost function'a göre daha iyi (daha derin) bir vadi rotası zaten
  MEVCUT ve geçerli primitive'lerle ifade edilebilir, ama weighted A*
  30.000 expansion içinde onu bulamadı (Stage 25-27'nin "thrashing/state
  explosion" teşhisiyle tam örtüşüyor). **early vs late**: C, D'den %2.20
  ucuz (aynı G/depth/min_MSL/climb/descent -- SADECE sıra farklı) -- doğru
  yönde ama depth etkisine (A'dan C'ye %3.47) göre daha KÜÇÜK bir etki;
  D yine de hem B'den hem A'dan ucuz çıktı, yani "geç ama derin" cost
  function'a göre "erken ama sığ"dan daha iyi -- zamanlamadan çok toplam
  düşük-MSL miktarı baskın. **roller-coaster**: E, C'den %7.74 pahalı, hatta
  A'dan (direct level) bile %4.00 pahalı -- R=700.00 (E'nin toplam
  maliyetinin ~%3.1'i) katkı yapıyor ama asıl ceza G+w*M'den geliyor (R=0
  varsayılsa bile E hâlâ A'dan %0.79 pahalı kalıyor, çünkü 18-aşağı+18-yukarı
  net MSL faydası sıfırlıyor, sadece fazladan mesafe/dz ekliyor).
  **long detour**: F, 6 candidate'ın EN PAHALISI (C'den %12.59, A'dan %8.69
  pahalı) -- aynı min_MSL'e (3400) ulaşıyor ama +845m fazladan mesafe hem G
  hem M'i (aynı düşük irtifada daha fazla mesafe uçtuğu için) büyütüyor,
  hiçbir ekstra MSL faydası olmadan. (Not/kısıt: bu F candidate'ı C'nin
  ULAŞTIĞI derinlikten DAHA DERİN bir MSL açmıyor -- sadece aynı derinliğe
  daha uzun yoldan ulaşıyor; "gerçekten daha düşük MSL'ye detour'la erişim"
  senaryosu bu turda test edilmedi.) **Sonuç: w_MSL=0.63 mantıklı görünüyor**
  (detour/roller-coaster doğru cezalandırılıyor, depth doğru ödüllendiriliyor,
  degenere davranış yok). **CASE A seçildi**: cost function mission
  davranışını yeterince temsil ediyor, weighted A*'ın sadece ~100m alçalmış
  olmasının nedeni cost DEĞİL, search'ün 30k bütçede daha ucuz/derin dalı
  bulamamış olması. Bu turda hiçbir weight/formül/search değişikliği
  yapılmadı (tek istisna: candidate B'nin path'ini almak için Stage 26'nın
  eps=1.10 çağrısı bir kez, hiç değiştirilmeden tekrar çalıştırıldı --
  yeni arama/tuning değil, zaten tamamlanmış bir sonucun geri alınması).
  Script: `scripts/validate_cost_function_ranking.py`. Bir sonraki adım
  (resolution/coarse-to-fine, per CASE A) bu turda BAŞLATILMADI, karar
  kullanıcıda.

- **Stage 29 — Cost Weight Sensitivity / w_MSL Calibration: search YOK,
  cost formülü DEĞİŞMEDİ, sadece `msl_cost_weight` sweep edildi
  ({0.30,0.40,0.50,0.63,0.80,1.00,1.25}). Stage 28'in 6 candidate'ı
  (A-F, AYNI geometri, yeniden üretilmedi) + yeni candidate G
  (DEEPER_BUT_LONGER_DETOUR, gerçek terrain'den ölçülmüş, fabrike değil)
  her weight'te yeniden costlandı. SONUÇ: CASE C -- tek bir scalar w_MSL,
  SHALLOW_VALLEY'i DEEP'ten hiçbir zaman >%5 ayıramıyor (matematiksel
  tavan ~%4.41, kanıtlanmış); bu bir weight seçimi sorunu değil, cost
  FORMÜLASYONUNUN yapısal bir sınırı.** G/R her candidate'ta sweep
  boyunca bit-exact invariant kaldı (sadece w*M değişti) -- production
  formülü doğrulandı. **Terrain taraması** (col 220-296, rows 96-216
  benzeri pencereler): direct corridor'un (col276) 3400m floor'undan daha
  derin bir nokta yalnızca col~264-267'de, ve sadece **20m** daha derin
  (3380m) bulundu -- makul bir lateral mesafede bu ROI'de daha fazla derinlik
  YOK (uydurulmadı, ölçüldü). G candidate'ı bu 20m'lik tek adımı, col276'dan
  ~12 sütun (360m) batıya/geri bir detour ile VALID olarak erişiyor
  (geom_len 6480->6778m). **Ana bulgu (matematiksel, sweep verisinden analitik
  türetildi, yeni run gerektirmedi)**: her candidate X için relative_gap(w) =
  (ΔG_X + w*ΔM_X) / (G_C + w*M_C) -- w->sonsuz limiti = ΔM_X/M_C (sabit).
  **Level için bu tavan %5.63** (w≈2.66'da tam %5'i geçiyor, ama bu
  test edilen aralığın (0.30-1.25) 2x'inden fazlası, ve tavana çok yakın/
  kırılgan bir marj). **Shallow için tavan %4.41 -- HİÇBİR w_MSL değeri
  (w->sonsuz dahil) bunu %5'e çıkaramaz, matematiksel olarak imkansız**
  (breakeven_w hesaplaması "None" döndü). Kök neden: her candidate G/M'sinin
  büyük bir ortak/paylaşılan kısmı var (tüm rotalar ~3400-3760m aralığında
  uçuyor, `msl_reference_m=0` referansına göre bu zaten büyük bir "ortak
  taban" M üretiyor) -- w_MSL büyüdükçe HEM fark HEM taban büyüyor, oran
  kilitli kalıyor. **Test edilen weight'ler arasında hiçbir dejenerasyon
  bulunmadı**: LONG_DETOUR sürekli en kötü aday kaldı (%12.5-12.7 gap,
  hafifçe düşüyor ama hep yüksek), ROLLER_COASTER sürekli kötü kaldı
  (%7.2-8.5 gap, hep direct level'dan da pahalı), LATE_DESCENT sürekli
  EARLY'den kötü kaldı (%1.64-2.60, w büyüdükçe HAFİFÇE artıyor -- doğru
  yönde), ve **DEEPER_BUT_LONGER_DETOUR (G) hiçbir weight'te C'den ucuza
  düşmedi** (kendi asimptotik tavanı da pozitif, ~%4.38 -- yapısal olarak
  hiçbir w_MSL'de C'yi geçemez, "küçük irtifa kazancı için aşırı detour"
  tuzağı bu candidate için YOK). **Karar: w_MSL=0.63 mevcut haliyle
  yeterli/makul (mission ranking DOĞRU yönde, dejenerasyon yok), ama %5
  bounded-suboptimality hedefiyle mission preferences arasındaki
  ayrım tek bir scalar w_MSL ile YAPISAL OLARAK sağlanamıyor** -- özellikle
  Shallow-vs-Deep için. Bu, cost formülasyonunun (ör. nonlinear altitude
  term, early-dwell reward, mission-specific term) yeniden düşünülmesi
  gerektiğine işaret ediyor -- **bu turda hiçbir formül değişikliği
  implement edilmedi**, sadece teşhis edildi. Reversal weight ayrı bir
  sensitivity testine bırakıldı (bu turda dokunulmadı, mevcut veriyle
  acil bir sorun görünmüyor). Script:
  `scripts/calibrate_msl_weight_sensitivity.py`. Sonraki adım (cost
  formula rework vs. resolution/coarse-to-fine) bu turda BAŞLATILMADI,
  karar kullanıcıda.

- **Stage 30 — Cost Formulation Redesign / Common-Baseline Analysis: search
  YOK, production kodu DEĞİŞMEDİ. Stage 29'un 7 candidate'ı (aynı geometri)
  iki alternatif altitude-term formülasyonuyla yeniden costlandı: F1
  (shifted linear: reference=H_FLOOR) ve F2 (shifted quadratic: aynı
  reference, kare). SONUÇ: CASE B -- F1, common-baseline problemini
  gerçekten çözüyor, mission separation'ı %5 barajının rahatça üstüne
  çıkarıyor, dejenere detour davranışı yaratmıyor; production'a aday
  olarak öneriliyor (henüz uygulanmadı).** H_FLOOR, Stage 17'nin kendi
  `_min_possible_aircraft_msl()` fonksiyonu (hiç değiştirilmeden) + bu
  benchmark'ın `min_search_altitude_msl=3240` ile **3240.0m** çıktı (ROI
  geneli terrain min=1697.7m + 200m çok daha düşük kaldığı için floor,
  min_search'ün kendisinden geliyor) -- state/row/col'a bağlı değil, tüm
  sweep boyunca tek sabit sayı. **F0 @ w=0.63, Stage 28/29'un decomposition'ıyla
  7/7 candidate'ta bit-exact eşleşti** (G/M/R ayrı ayrı doğrulandı).
  **F1 (w_alt sweep 0.25-2.0)**: w_alt=0.50'de HEM shallow (%7.16) HEM level
  (%9.18) zaten %5 barajını geçiyor; w_alt=0.63'te shallow=%8.88,
  level=%11.40, late=%5.99 (üçü de >%5). **F2 (quadratic)** benzer yönde
  ama biraz daha yavaş ayrışıyor (w=0.5'te shallow=%5.08, level=%7.25;
  w=0.63'te shallow=%6.49, level=%9.25, late=%4.53 -- late hâlâ tam %5'i
  geçmiyor, F1'den bu açıdan biraz daha zayıf). **Kritik
  matematiksel dogrulama**: F1 hâlâ lineer olduğu için Stage 29'daki kapalı
  formül tekrar uygulanabildi -- asimptotik tavan (w->sonsuz) Level için
  **%5.63 -> %79.44**, Shallow için **%4.41 -> %61.58**, Late için
  (Stage 29'da hesaplanmamıştı, bu turda F0 icin de geriye dönük
  hesaplandı) **%3.19 -> %39.11**, Long-detour için F1 tavanı **%7.20**
  (F0'da bu adım için ayrıca hesaplanmamıştı), Deeper-detour(G)
  **%4.38 -> %1.80 (KÜÇÜLDÜ ama hâlâ pozitif --
  G test edilen HİÇBİR w_alt'ta (0.25-2.0) C'den ucuza düşmedi, "G_cheaper_
  than_C" hep False; degenerasyon YOK ama güvenlik marjı F0'a göre daralıyor,
  izlenmesi gereken bir sinyal).** **Roller-coaster hem C'den hem A'dan hep
  kötü kaldı, gap w büyüdükçe BÜYÜDÜ (F0'ın tersine)** -- F1 altında E
  7 candidate içinde EN KÖTÜSÜ hâline geldi (F0'da bu F idi). **ÖNEMLİ:
  reference değişikliği ranking'i olduğu gibi KORUMADI (section 18'in
  uyarısı doğrulandı)**: F0 sırası C<D<B<A<G<E<F idi; F1 (w=0.5/0.63)
  sırası **C<G<D<B<A<F<E** -- G, D/B/A'yı geçti (gerçek ~20m ekstra derinliği
  artık daha güçlü ödüllendiriliyor, ama C'yi hiçbir zaman geçmedi), ve
  F artık E'den daha iyi (E en kötü hâle geldi). Bu değişiklik mission
  açısından makul görünüyor (G, C'den sonra en "derin" aday olduğu için
  öne çıkması tutarlı) ama üretime alınmadan önce bilinçli kabul edilmesi
  gereken bir davranış değişikliği. **Common-baseline azalması sayısal**:
  M_raw(Deep) F0'da 23066.13 -> F1'de 1877.86 (**%91.9 azalma**), M_raw
  (Level) 24364.80 -> 3369.60 (**%86.2 azalma**) -- farklı oranlarda
  azaldığı için (translation etkisi, "sadece sabit çıkarmak" değil) ranking
  de değişti. **Heuristic uyumluluğu (sadece analiz, implement edilmedi)**:
  F1 hâlâ additive/non-negative, admissible heuristic türetilebilir --
  hatta Stage 17'nin GLOBAL multiplier bound'u H_FLOOR tanımı gereği
  otomatik olarak 1.0'a kilitleniyor (o mekanizma F1 altında anlamsızlaşıyor),
  ama Stage 21'in FORWARD (descent-rate envelope) heuristic'i aynı ispat
  yapısıyla F1'e uyarlanabilir görünüyor. F2 için de bir lower-bound
  prensipte türetilebilir (quadratic parça hâlâ tam entegre edilebilir)
  ama F0/F1'den belirgin daha karmaşık -- bu turda türetilmedi. **Karar:
  F1 (shifted linear, H_FLOOR=3240, w_alt≈0.63 civarı) bir sonraki
  production adayı olarak öneriliyor (CASE B) -- basit/additive/kolay-
  heuristic-uyumlu kalırken mission separation'ı gerçek biçimde çözüyor;
  F2 gereksiz karmaşıklık (nonlinear heuristic türetimi) getiriyor ve
  ölçülen faydası F1'den belirgin değil. Production'a bu turda
  UYGULANMADI** (config.py/astar.py dokunulmadı), sadece diagnostic.
  Script: `scripts/analyze_cost_formulations.py`. Sonraki adım (production
  cost update mi, yoksa mevcut cost'u koruyup coarse-to-fine'a geçmek mi)
  bu turda BAŞLATILMADI, karar kullanıcıda.

- **Stage 31 — Normalized Cost Function Design / Distance-Altitude
  Trade-off: search YOK, production kodu DEĞİŞMEDİ. Aynı 7 candidate
  boyutsuz bir C_distance/C_altitude/C_reversal yapısıyla yeniden
  costlandı. SONUÇ: CASE B -- normalization yapısal olarak değerli
  (component'ler artık aynı büyüklük mertebesinde, D/E/F/G artık C'ye
  karşı KANITLANABİLİR şekilde domine ediliyor -- herhangi bir pozitif
  weight'te asla C'yi geçemezler), ama test edilen H_scale=100m + weight
  set'leri kombinasyonu mission'ın "biraz daha önemli" ifadesine göre çok
  agresif ayrım üretiyor (%45-66) -- production'a geçmeden H_scale/weight
  yeniden kalibre edilmeli.** D_ref=6480.0000m (exact, start-goal
  straight-line). H_ref=3240m (Stage 30'un H_FLOOR'u, aynen), H_scale=100m
  (spesin verdiği diagnostic değer), R_scale=20 (=vertical_reversal_cost_
  weight(1.0)*z_step_m(20)*max_factor(1.0) -- tek bir en-kötü-durum
  reversal'ın ham maliyeti, keyfi değil). **Normalized component'ler
  (weight'ten bağımsız)**: her candidate'ta C_distance≈1.00-1.14 (D_ref'e
  göre neredeyse hepsi), C_altitude 2.90-5.20 arası (H_scale=100 nedeniyle
  distance'tan 3-5x büyük mertebede -- component'ler artık "aynı
  mertebede" ama altitude hâlâ sayısal olarak baskın), C_reversal yalnız
  E'de sıfır değil (E: 35.0 -- **R_scale=20 ile bu tek başına E'nin
  toplamını EZİYOR** (E toplamı SET_A'da 41.19, sırf reversal'dan 35.0 --
  distance+altitude'un ~6 katı); bu R_scale/weight_reversal kombinasyonu
  bilinçli olarak FİNALİZE EDİLMEDİ, ham R (=700 E için, 0 diğerlerinde)
  ayrıca raporlandı -- section 6'nın "yeterli veri yoksa finalleştirme"
  kaçış maddesi kullanıldı; bu diğer 6 candidate karşılaştırmasını
  ETKİLEMİYOR (hepsinin R=0). **Domination testi (en önemli yapısal
  bulgu)**: D, E, F, G'nin HEPSİ C'ye göre HEM C_distance HEM C_altitude'da
  daha kötü çıktı (`dC_distance>=0 VE dC_altitude>=0` dört candidate'ta
  da) -- yani **hiçbir pozitif (w_distance,w_altitude) çifti bu dört
  candidate'ı C'den ucuza düşüremez, matematiksel garanti** (Stage 29/30'un
  "asimptotik tavan pozitif kaldı" bulgusundan daha güçlü bir sonuç --
  orada sadece test edilen aralıkta geçmedi, burada YAPISAL OLARAK
  geçemez). Yalnız A ve B, C'ye göre hafifçe daha KISA (`dC_distance<0`)
  olduğu için gerçek bir trade-off (weight'e bağlı) hâlâ var -- bu
  beklenen/doğru davranış. **4 weight set sonucu**: hepsi zaten %5
  barajını çok rahat geçiyor (SET_A: shallow=%45.50, level=%58.68; SET_D:
  shallow=%48.02, level=%61.93) -- ama bu dramatik sıçrama (F1'in %8.88/
  %11.40'ından) esas olarak weight seçiminden değil, **H_scale=100'ün F1'in
  msl_scale=1000'inden 10x daha sıkı olmasından geliyor** -- section 9'un
  uyardığı "scale ile gizli tuning" riskinin somut bir örneği: weight
  set'leri (SET_A'dan SET_C'ye) gap'i sadece %45->%66 kaydırırken, H_scale
  tek başına F1'den buraya %9->%45+ sıçratıyor. **G ve F hâlâ hiçbir
  weight'te C'yi geçmiyor (kanıtlandı, yukarıya bkz), roller-coaster hâlâ
  açık ara en kötü** (ama bu ölçüm R_scale'e bağlı, yukarıda not edildi).
  **100m altitude <-> distance trade-off (analitik)**: `extra_distance_
  ratio=(w_altitude/w_distance)*(100/H_scale)` -- SET_A: %100.0 (1 km 100m
  daha alçak uçmak, 1 EKSTRA km'e eşdeğer), SET_B/D: %125.0, SET_C: %166.7
  -- H_scale=100 sabit tutulduğu sürece bu oranlar zaten oldukça güçlü,
  "biraz daha önemli" ifadesinin ima ettiğinden daha agresif olabilir.
  **Mission-length robustness (analitik, benchmark koşulmadı)**:
  C_distance ve C_altitude ikisi de D_ref ile orantılı ölçeklendiği sürece
  (fiziksel altitude profili aynı kalırsa) mission uzunluğundan bağımsız
  kalıyor; C_reversal ise zaten olay-sayısı bazlı, mesafeden bağımsız.
  **H_ref stabilite riski**: H_ref=3240m, `min_search_altitude_msl`'den
  geliyor -- bu bir SEARCH-CALL parametresi (bu benchmark'ın korîdor
  taramasına özel), fiziksel/mission sabiti değil; search bound'u
  değişirse AYNI fiziksel path'in cost'u değişebilir -- production için
  daha stabil bir global H_ref tanımı gerekli, bu turda implement
  edilmedi. **Karar: CASE B** -- normalization yapısal olarak faydalı
  (yorumlanabilir, domination garantisi güçlü) ama H_scale/weight
  kombinasyonu mission'ın "biraz daha önemli" ölçeğine göre yeniden
  kalibre edilmeli; F1 (Stage 30) hâlâ daha ölçülü bir ara adım olarak
  görünüyor. Production'a bu turda hiçbir değişiklik UYGULANMADI. Script:
  `scripts/analyze_normalized_cost.py`. Sonraki adım (H_scale/weight
  recalibration mı, yoksa F1'i mi tercih etmek, ya da coarse-to-fine'a
  geçmek mi) bu turda BAŞLATILMADI, karar kullanıcıda.

- **Stage 32 — Production Normalized Cost + Calibration + Final Aladağlar
  Testi: bu adım artık diagnostic-only DEĞİL, gerçek production kodu
  değişti.** `planner/config.py`'ye `cost_mode: str = "legacy"` (yeni
  default, DEĞİŞMEDİ) + normalized-mode-only alanlar eklendi:
  `altitude_reference_msl: Optional[float] = None` (artık explicit
  mission/cost parametresi, hiçbir search bound'undan TÜRETİLMİYOR),
  `normalized_altitude_scale_m=1000.0`, `normalized_w_distance=1.0`,
  `normalized_w_altitude=1.0`, `normalized_w_reversal=1.0` (hepsi
  cost_mode="legacy" iken tamamen atıl/okunmuyor). `planner/astar.py`'de:
  `compute_distance_reference(start,goal,terrain,config)` yeni public
  fonksiyon (D_ref'i BİR KEZ, search döngüsü dışında hesaplıyor);
  `compute_edge_cost()` artık `config.cost_mode`'a göre dispatch ediyor --
  legacy branch **hiç değişmeden aynı kaldı** (yeni `distance_reference_m:
  Optional[float]=None` parametresi legacy'de asla okunmuyor), normalized
  branch yeni `_compute_edge_cost_normalized()` fonksiyonuna gidiyor
  (`dC_distance=geom/D_ref`, `dC_altitude=dC_distance*(excess/H_scale)`,
  `dC_reversal=reversal_raw/D_ref` -- Stage 31'in aşırı agresif `R_raw/20`
  yerine spesin düzelttiği `R_raw/D_ref`); `_heuristic()` normalized mode
  için ayrı, kasıtlı minimal bir dal aldı (`h=w_distance*D3D/D_ref`,
  admissible+consistent ispatı: altitude/reversal terimleri hep >=0
  olduğundan gerçek maliyet her zaman `w_distance*dC_distance`'tan büyük,
  ve D3D üçgen eşitsizliğiyle sınır oluşturuyor) -- Stage 17/21'in legacy
  MSL-lower-bound/vertical-reachability makinesi normalized mode'da
  TAMAMEN ATLANIYOR (`use_msl_lower_bound_heuristic and config.cost_mode
  =="legacy"` guard'ı eklendi), spesin section 16 talimatı gereği.
  `validate_and_cost_path()`/`_generate_neighbors()` yeni opsiyonel
  `distance_reference_m` parametresi aldı (default None, legacy çağrılarda
  hiç kullanılmıyor, geriye dönük uyumlu). `SearchResult`'a `cost_mode` ve
  `distance_reference_m` alanları eklendi (raporlama için). **Legacy mode
  regresyon testleri (Stage 24 `validate_incumbent_pruning.py`, Stage 25
  `validate_weighted_astar.py`) tam bit-exact PASS** -- project.md'deki
  kayıtlı sayılarla (`cost=2717.84`, `expanded=679/494/679/494`, `C*=
  2424.1952`, `expanded=1308`, certified `ratio=1.0000<=1.05` vb.) birebir
  eşleşti; legacy davranış hiç bozulmadı. **Kalibrasyon** (`scripts/
  calibrate_normalized_production_cost.py`, GERÇEK production
  `compute_edge_cost`/`validate_and_cost_path` çağrılıyor, diagnostic
  formül tekrar YAZILMADI): 4 H_scale (500/750/1000/1500) x 2 w_altitude
  (1.0/1.25) = 8 kombinasyon, aynı 7 Stage 28-31 path'i üzerinde. **Seçilen
  konfigürasyon: H_scale=1000, w_altitude=1.25** (trade-off=%12.5 extra
  distance per 100m altitude advantage -- mission'ın %10-15 hedef bandının
  ortasına en yakın, ve Sh>5/Lv>5/G_never_beats_C üç kriteri de sağlayan
  aday): Shallow gap=%15.78, Level gap=%20.31, Late gap=%10.33, Roller
  gap=%28.68, Long-detour gap=%11.41, Deeper-detour(G) gap=%3.87 (hep
  pozitif, G hiçbir kombinasyonda C'yi geçmedi). **9 unit/regression testi
  (A-I) ALL PASS**: additivity (bit-exact manuel toplamla eşleşti),
  non-negative (tüm candidate'ların tüm edge'lerinde), ve en önemlisi (I)
  `altitude_reference_msl` search-bound bağımsızlığı -- level-only
  primitive set ile forced-identical fiziksel path, `[1300,1400]` vs
  `[800,1900]` search bound'larında **bit-exact aynı cost** verdi
  (`same_path=True same_cost=True`) -- Stage 31'in H_FLOOR riskinin artık
  giderildiği doğrulandı. **GERÇEK 6.48km Aladağlar testi (TEK run, 30k
  cap, retry YOK)**: `status=search_limit_reached`, wall=**64.38s**,
  expanded=30,000, max_open=**51,492** (legacy eps=1.10'un 108,341'inin
  yaklaşık YARISI), cache_hit=0.757, **reopened_states=0** (legacy eps=
  1.10'un %95.0 reopen-thrashing'inin TAM TERSİ -- hiç reopen olmadı),
  initial_incumbent (yeni normalized formülle) = **1.650000** (eski
  21829.82 legacy cost'u DEĞİL, kasıtlı olarak yeniden hesaplandı),
  **first_solution_found=False** (legacy eps=1.10'un 685 expansion'da
  bulduğu ilk çözümün aksine, normalized cost 30.000 expansion boyunca
  fiziksel goal'a HİÇ ulaşamadı), `current_lower_bound=1.094752`,
  `final_bound_ratio=1.5072` (incumbent hâlâ ilk direct-level path,
  hiç iyileşmedi). **Path bulunamadığı için fiziksel analiz (min/avg MSL,
  descent, dwell vb.) yapılamadı** -- legacy eps=1.10'un ulaştığı 3660m
  minimumla karşılaştırma mümkün olmadı. **Sınıflandırma: CASE C (search
  problemi devam ediyor)** -- cost davranışı kalibrasyonda kanıtlanabilir
  şekilde doğru (Stage 28-31'in bütün mission-separation kriterlerini
  sağlıyor, unit testler ALL PASS) ama gerçek ölçekte 30k'da hâlâ çözüm
  yok. **Olası kök neden (ölçülmedi, sadece hipotez)**: normalized mode'un
  KASITLI OLARAK minimal heuristic'i (`w_distance*D3D/D_ref`, legacy'nin
  Stage 17/21 MSL-farkındalı sınırlarının hiçbiri yok) düşük-MSL
  tercihini hiç "görmüyor" -- reopened_states=0 olması search'ün artık
  thrashing yapmadığını ama bunun yerine büyük bir alanı zayıf
  yönlendirmeyle GENİŞÇE (max_open yine de 51k) taradığını düşündürüyor;
  bu, legacy'nin Stage 17 MSL-aware heuristic'ini normalized cost'a
  uyarlamanın (bu turda YAPILMADI, spesin section 16 talimatı gereği)
  gelecekteki bir aday olabileceğine işaret ediyor. **Bu turda `cost_mode`
  global default'u "legacy" olarak KALDI, değiştirilmedi.** 150k retry
  YAPILMADI (spesin açık yasağı). Scriptler: `scripts/
  calibrate_normalized_production_cost.py`,
  `scripts/run_normalized_aladaglar_benchmark.py`. Sonraki adım
  (normalized+Stage17-tipi heuristic kombinasyonu mu, yoksa coarse-to-fine
  mı) bu turda BAŞLATILMADI, karar kullanıcıda.

  **Ek: epsilon=1.20 ve 1.30 ile 2 ek run (kod/cost/heuristic/config
  DEĞİŞMEDİ, sadece epsilon_search).** Script: `scripts/
  run_normalized_aladaglar_epsilon_sweep.py`. Sonuç: **üçü de (1.10/1.20/
  1.30) `search_limit_reached`, first_solution_found=False** -- epsilon
  artışı search'ü İYİLEŞTİRMEDİ, KÖTÜLEŞTİRDİ:
  ```
  eps               status  runtime  max_open  reopen  incumbent        LB   ratio
  1.10 search_limit_reached    64.30     51492       0  1.650000  1.094752  1.5072
  1.20 search_limit_reached    50.60     57389       0  1.650000  1.079845  1.5280
  1.30 search_limit_reached    40.14    119209   12039  1.650000  1.055286  1.5636
  ```
  max_open monoton büyüyor (51,492 -> 57,389 -> 119,209), ve eps=1.30'da
  reopen_ratio=%40.1 ile thrashing tekrar başlıyor (legacy'nin eps
  küçültmede gördüğü thrashing örüntüsünün ADAYI BURADA epsilon
  BÜYÜTMEDE ortaya çıkıyor -- ters yönde ama aynı patoloji). current_
  lower_bound eps büyüdükçe düşüyor (1.095->1.080->1.055, teorik optimum
  daha ucuza yaklaşıyor) ama hiçbirinde fiziksel goal'a ulaşılamadı.
  **1.10, üç değer içinde en iyisi** (en düşük max_open, reopen=0, en
  düşük final_bound_ratio). **CASE C teyit edildi, daha güçlü biçimde**:
  epsilon-tuning bu ölçekte normalized+minimal-heuristic kombinasyonunu
  kurtarmıyor. 150k retry ve 1.30 üstü epsilon denenmedi (spesin açık
  yasağı).

- **Stage 33 — Safe Goal Region (70x70x50m box, ±35m XY/±25m Z) production
  feature olarak eklendi (`goal_tolerance_xy_m`/`goal_tolerance_z_m`,
  default 0.0 = eski exact-goal davranışı bit-exact korunuyor); gerçek
  6.48km testinde box'a HİÇ ulaşılamadı, en yakın state hâlâ ~5.07km
  uzakta (yolun sadece ~%21'i) -- Stage 33'ün kendi hipotezi
  ÇÜRÜTÜLDÜ: exact-goal-discretization köken problem DEĞİL.
  Sınıflandırma: CASE C, coarse-to-fine için güçlü kanıt.** `planner/
  config.py`'ye 2 yeni alan (`goal_tolerance_xy_m/z_m: float = 0.0`).
  `planner/astar.py`'ye: `_distance_to_goal_box()` (axis-aligned box'a
  min 3D mesafe, `max(abs(delta)-tolerance,0)` per eksen, tolerance=0 iken
  bit-exact aynı plain distance'a indiriyor) ve `_state_in_goal_region()`
  (tolerance=0 iken hiç float'a girmeden eski `state==goal` integer-tuple
  karşılaştırmasını AYNEN kullanıyor -- section 1'in "birebir korunmalı"
  şartı kod seviyesinde garanti edildi, davranışsal umuda değil). Ana
  loop'taki goal-check `_state_in_goal_region(...)`'a geçti; x/y metrik
  mesafe HER ZAMAN `terrain.rowcol_to_xy` üzerinden (gerçek affine
  transform), row/col farkını kör kullanma YOK. **Safety asla gevşemedi**:
  bir state'in goal-check'e ulaşabilmesi için zaten `_generate_neighbors`
  içinde `evaluate_primitive()`'den geçmiş olması gerekiyor (AGL/terrain/
  NoData/bounds/açı) -- box'un kendisi hiçbir safety kuralını bypass
  etmiyor, mimari olarak edemez. **Heuristic'in DEĞİŞTİRİLMESİ zorunluydu**
  çünkü exact-center'a mesafe kullanmak artık bir OVERESTIMATE üretirdi
  (box'un herhangi bir noktasına ulaşmak yeterliyken merkeze mesafe daha
  büyük bir sayı verir) -- bu admissibility'yi (`h<=h*`) bozardı.
  `_heuristic()`'teki D3D hesaplaması `_distance_to_goal_box`'a geçti (hem
  legacy h_global/h_forward hem normalized mod için TEK ortak değişiklik
  noktası) -- tolerance=0 iken bit-exact aynı, yeni bir "exact mode"
  dalı gerekmedi. **9 unit test (A-K, spesin istediği tüm case'ler) ALL
  PASS**: A-G box-membership formülü (D/E boundary case'leri dahil, `<=`
  ile INCLUSIVE), H (goal box'ın TAMAMI unsafe yapıldı -- terrain aircraft'ın
  kendi irtifasında, box içindeki HİÇBİR z bile 200m AGL'yi sağlamıyor --
  search `no_path` ile bitti, `closest_distance_to_goal_region_m=25.00`
  yani box'a asla girmedi, kanıtlandı), I/J (box içi h=0, box dışı h=
  merkez-mesafesi DEĞİL yüzey-mesafesi -- 150m merkez mesafesi olan bir
  state h=115m verdi, tam beklenen `150-35` yüzey mesafesi), K (tolerance=0
  ile default çağrı ile explicit `goal_tolerance_xy_m=0.0/z_m=0.0` çağrısı
  arasında `expanded/max_open/cost` bit-exact eşleşti). **Diagnostic
  eklentisi (section 9, kritik teşhis gücü nedeniyle bu turda astar.py'ye
  eklendi -- Stage 26/27'nin section 7'sinden farklı olarak burada
  "büyük refactor gerekiyorsa atla" kaçışı YOKTU)**: her expand edilen
  state için O(1) `closest_distance_to_goal_region_m`/`_center_m`/
  `closest_state_to_goal` takibi, `SearchResult`'a 3 yeni alan --
  saf gözlemsel, search kararlarını hiç etkilemiyor. **Regresyon**:
  Stage 24/25'in validation script'leri (`validate_incumbent_pruning.py`,
  `validate_weighted_astar.py`) bu değişiklikten SONRA tekrar çalıştırıldı,
  kayıtlı sayılarla (expanded=679/494/679/494, C*=2424.1952, certified
  ratio=1.0000<=1.05, incumbent_updates=10, reopened=1679 vb.) bit-exact
  eşleşti -- legacy davranış hiç bozulmadı. **GERÇEK 6.48km testi (TEK
  run, Stage 32 normalized-cost baseline'ıyla AYNI: H_scale=1000,
  w_altitude=1.25, altitude_reference_msl=3240, epsilon=1.10, target=1.05,
  dominance OFF, 30k cap, retry YOK) + goal region ±35m/±25m**: initial
  incumbent (yeniden doğrulandı) = 1.650000 (Stage 32 ile aynı, tolerance
  cost'u etkilemiyor). `status=search_limit_reached`, wall=98.95s,
  expanded=30,000, max_open=51,475 (Stage 32'nin 51,492'sine PRATİKTE
  AYNI), cache_hit=0.757, reopened=0, `first_solution_found=False`
  (Stage 32'yle birebir aynı sonuç -- tolerance HİÇBİR fark yaratmadı).
  `final_bound_ratio=1.5148` (Stage 32'nin 1.5072'sine çok yakın).
  **Kritik teşhis**: `closest_distance_to_goal_region_m=5,066.80m` (box'a
  hâlâ 5+ KM uzakta!), `closest_distance_to_goal_center_m=5,102.51m`,
  en yakın state row=94 (start row=48, goal row=264 -- yolun sadece
  **~%21'i**, `col=276 msl=3600 terrain=3203.6 point_AGL=396.4` -- güvenli
  ve mantıklı bir ilk-inişe başlamış durum, ama goal'dan ÇOK uzak).
  **Sonuç kesin: exact-goal-discretization hipotezi ÇÜRÜTÜLDÜ** -- search
  "hedefin yakınına gelip tek bir hücreyi kaçırmıyor", 30.000 expansion
  içinde yolun beşte birinden fazlasını bile kat edemiyor. Bu, Stage
  20'den beri tekrar tekrar doğrulanan "ham search hacmi/state-explosion"
  teşhisini nihai olarak teyit ediyor -- **Sınıflandırma: CASE C**
  (spesin kendi kelimeleriyle: "exact goal condition kök problem değil,
  coarse-to-fine'a geçmek için güçlü kanıt oluşur"). Bu turda YAPMAZ
  listesi (coarse-to-fine, resolution, terrain-aware Dijkstra, corridor,
  epsilon sweep, cost tuning, yeni heuristic terimi, dominance, variable
  angle, heading, turn radius, GPU) hiçbiri implement edilmedi.

Henüz implement edilmedi: binary mask, polygonization, buffer/C-Space,
climb/descent için ayrı reversal weight, preferred-AGL/terrain-following
mode, heading/turn radius/Dubins, 2D Dijkstra heuristic,
EDT/buffer, global planner, görselleştirme, bağımsız final validator,
terrain-aware 2D / coarse-to-fine planlama (Stage 33'ün "exact-goal-discretization kök problem" hipotezi ÇÜRÜTÜLDÜ -- search 30k expansion'da goal'dan hâlâ 5+km/yolun %79'u kadar uzakta kalıyor; Stage 20-33 boyunca denenen state-space optimizasyonlarının (dominance/bucket/incumbent/weighted-A*/epsilon-sweep/normalized-cost/safe-goal-region) HİÇBİRİ bu 6.48km/520m-relief benchmarkını tek başına çözemedi), ARA*/decreasing-epsilon anytime search, low-MSL tie-break, msl_cost_weight default kararı,
backward (goal-side) vertical-reachability envelope, uzun-rota + yüksek-
w_MSL search explosion çözümü (terrain-aware/Dijkstra/wavefront/corridor
adayları not edildi, henüz implement edilmedi), variable-angle primitive
testi (bu problem çözülmeden yapılmayacak).

- **Stage 34 — Conservative 90m coarse DEM: sadece terrain representation,
  hiçbir A*/search çalıştırılmadı. MAX pooling (average/bilinear/nearest
  DEĞİL) ile 30m/333×333 fine DEM'den 90m/111×111 coarse DEM üretildi;
  gerçek ROI'nin 12.321 bloğunun HEPSİ conservatism testini geçti (0
  violation), global peak (3700.39m) korundu. PASS.** Yeni modül:
  `planner/coarse.py` (`build_coarse_dem(fine_roi, factor=3)`, mevcut
  `ROIData`/`TerrainQuery` mimarisine dokunmadan aynı `ROIData` yapısını
  üretiyor). NoData: bir blokta TEK bir NoData hücresi bile varsa (ya da
  gerçek NaN, sentinel ne olursa olsun) coarse hücre NoData oluyor — bu
  gerçek DEM'de hiç tetiklenmedi (nodata_count=0, fine ve coarse'da) ama
  4 sentetik testte (D dahil) doğrulandı. Affine transform fine'ın kendi
  a/b/c/d/e/f'inden türetildi (hardcoded UTM yok): pixel scale 30m->90m,
  origin (684110.84, 4191478.06) bit-exact korundu. Edge policy: sadece
  TAM 3×3 bloklar (gerçek ROI 333=3×111 tam bölündüğü için bu senaryoda
  `dropped_rows=dropped_cols=0`). **9 test (A-H + real-ROI) ALL PASS**:
  A (3×3 merkez peak 3650m korundu), B (6×6->2×2 elle hesaplanan 4 blok
  bit-exact), C (köşedeki peak korundu), D (NoData bloğu NoData'ya
  düştü), E (90m/-90m pixel + origin doğrulandı), F (world-coordinate
  round-trip fine(7,4)->UTM->coarse(2,1) doğru), G (gerçek ROI: 12,321
  blok, 0 violation), H (global max 3700.39m fine==coarse). **START/GOAL
  mapping**: START fine(48,276)->UTM(692405.84,4190023.06)->coarse(16,92);
  GOAL fine(264,276)->UTM(692405.84,4183543.06)->coarse(88,92) --
  transform-tabanlı mapping ile naive `row//3` INTEGER DIVISION birebir
  aynı sonucu verdi (333 tam bölündüğü için beklenen), mapping_error=
  42.43m (=sqrt(30²+30²), fine noktanın coarse hücre MERKEZİNE olan
  mesafesi -- mapping'in kendisinde bir hata değil). Bounds/extent: fine
  ve coarse aynı (684110.84,4181488.06)-(694100.84,4191478.06) kutusunu
  kapsıyor, sıfır shift. Coarse terrain istatistikleri: min=1703.59m
  (fine 1697.70'ten yüksek, max-pooling'in beklenen etkisi -- ortalama
  YUKARI kayıyor), max=3700.39m (fine ile bit-exact), mean=2860.61m
  (fine'ın 2840.06'sından yüksek, aynı sebep). GeoTIFF `working_dem/
  aladaglar_roi_coarse_90m_max.tif`'e yazıldı (CRS/transform/nodata/dtype
  doğrulandı); mevcut fine DEM dosyası HİÇ açılmadı-yazma modunda,
  sadece `load_roi()` ile okundu. Script: `scripts/validate_coarse_dem.py`.
  Bu turda YAPMAZ listesi (coarse/fine A*, corridor, cost/heuristic
  tuning, herhangi bir path search) hiçbiri implement edilmedi. Sonraki
  adım (Stage 35: coarse planner) bu turda BAŞLATILMADI, karar kullanıcıda.

- **Stage 34.5 — Coarse Terrain Statistics: her 90m coarse hücre için
  min/mean/max/relief eklendi (yalnız MAX'ın tek başına gizleyebileceği
  düşük-terrain bilgisini karakterize etmek için); hiçbir A*/cost'a
  bağlanmadı. 14 test (5 sentetik + F/G/H real-ROI) ALL PASS.**
  `planner/coarse.py`'ye `CoarseTerrainStats` dataclass (4 ayrı NumPy
  array: min_elevation/mean_elevation/max_elevation/relief, hepsi
  (111,111)) + `build_coarse_terrain_stats(fine_roi, factor=3)` eklendi.
  Stage 34'ün `build_coarse_dem`'i **davranış olarak değişmedi** --
  ortak crop/nodata-mask/transform mantığı `_crop_to_complete_blocks`/
  `_coarse_transform` helper'larına çıkarıldı (pure refactor, Stage 34'ün
  kendi validation script'i sonrasında tekrar çalıştırılıp bit-exact
  ALL PASS doğrulandı). **NoData policy Stage 34 ile birebir aynı**:
  bloktaki TEK bir NoData/NaN hücre → o coarse hücrenin min/mean/max/
  relief'inin HEPSİ NoData (8 valid+1 unknown durumunda bile kısmi
  inference YOK) -- gerçek ROI'de hiç tetiklenmedi (nodata_count=0),
  ama Test E'de sentetik doğrulandı. **MAX'ın safety rolü, MIN/MEAN'in
  yalnız guidance rolü kod docstring'inde açıkça belirtildi**: hard
  safety kararları için ileride SADECE `max_elevation` kullanılacak;
  `min_elevation`/`mean_elevation` asla bir safety margin'i temizlemek
  için kullanılmayacak (tek bir düşük 30m fine hücre 90m'lik bloğun
  min'ini gerçekte-geçilemez bir noktayı "geçilebilir" gibi
  göstermeden çok aşağı çekebilir -- bu caveat hem docstring'de hem
  diagnostic fonksiyon isimlerinde ("MAX alone hides low terrain",
  "NOT a valley claim") vurgulandı). **relief = max-min, terrain
  slope/climb-angle DEĞİL** -- kod ve raporlama boyunca bu ayrım
  korundu. **Gerçek ROI sonuçları**: MIN(1697.70-3690.02, mean=2819.49),
  MEAN(1701.53-3694.57, mean=2840.06), MAX(1703.59-3700.39, mean=
  2860.61) -- üç katman arasındaki sıralı artış (min<mean<max
  ortalamada) beklenen max-pooling etkisiyle tutarlı. **RELIEF**:
  min=0.84 max=**282.10m** mean=41.11 median=36.42 p90=73.25 p95=88.94
  -- yüksek-relief hücrelerin HEPSİ dağlık/sarp bölgelerde (örn.
  row=21,col=81: 3046→3328m tek bir 90×90m hücre içinde), START/GOAL
  hücreleri düşük relief'li (41.68m / 39.47m, nispeten düz alanlar).
  **F (relief==max-min) ve G (min<=mean<=max) 12.321 valid hücrenin
  HEPSİNDE 0 violation. H: yeni MAX layer, hem in-memory
  `build_coarse_dem()` hem diskteki mevcut max.tif ile bit-exact
  eşleşti** (max.tif SADECE okundu, hiç yeniden yazılmadı). 3 yeni
  GeoTIFF yazıldı: `working_dem/aladaglar_roi_coarse_90m_{min,mean,
  relief}.tif` (aynı shape/transform/CRS/bounds/nodata-policy, dtype:
  min/max/relief=fine'ın kendi dtype'ı float32, mean=her zaman float32).
  Script: `scripts/validate_coarse_terrain_stats.py`. Bu turda YAPMAZ
  listesi (coarse/fine A*, corridor, terrain/relief/min/mean-tabanlı
  cost/heuristic, herhangi bir path search) hiçbiri implement edilmedi
  -- **bu istatistikler henüz planner/cost'a bağlanmadı**. Sonraki adım
  (Stage 35: coarse planner) bu turda BAŞLATILMADI, karar kullanıcıda.

- **Stage 35 — Simplified Coarse 3D A*: PROJENİN Stage 19'dan beri İLK
  BAŞARILI 6.48km start-goal çözümü. Coarse (90m) A*, 24,959 expansion'da
  (30k cap altında) SUCCESS buldu; 31-node path fine 30m DEM üzerinde
  <=10m sample ile replay edildi ve 0 VIOLATION verdi (min_AGL=202.31m).**
  Yeni, tamamen izole modül: `planner/coarse_astar.py` -- `planner/astar.py`
  hiç değiştirilmedi/dokunulmadı. State SADECE `(row,col,z_index)` --
  trend/bucket/dominance/reversal/heading YOK. Safety, mevcut
  `planner.primitives.evaluate_primitive`/`planner.agl.evaluate_agl`/
  `planner.transition.evaluate_transition` fonksiyonları HİÇ
  DEĞİŞTİRİLMEDEN, sadece coarse config (xy_res=90m, z_step=40m,
  sample_spacing=30m) + coarse MAX `TerrainQuery` ile çağrılarak
  reuse edildi -- "aircraft_msl - coarse_MAX_terrain >= 200m" hard
  safety'si SIFIR yeni kod ile otomatik olarak doğru çalıştı. MIN/MEAN/
  RELIEF (`CoarseTerrainStats`) cost'a veya safety'ye HİÇ girmedi --
  sadece path-sonrası diagnostic. **Cost** (Stage 32'nin basitleştirilmiş
  hali, reversal terimi yok): `dC_distance=ds_3d/D_ref`,
  `dC_altitude=dC_distance*max(0,mean_MSL-3240)/1000`,
  `total=1.0*dC_distance+1.25*dC_altitude`. **Heuristic**:
  `h=D3D_to_goal/D_ref` (admissible, aynı üçgen-eşitsizliği argümanı).
  Standart A* (epsilon=1.0, dominance/incumbent/weighting YOK).
  **Endpoint lift policy** (`lift_endpoint_if_unsafe`): START coarse
  (16,92) cell_max=3556.81 -> required=3756.81<=3760 -> **lift YOK**
  (3760 kalıyor); GOAL coarse (88,92) cell_max=3560.36 ->
  required=3760.36>3760 -> **3800'e lift edildi** (bir z-step yukarı,
  40m). **10/10 sentetik test ALL PASS**: 24 primitive/max açı<=10°
  (test 1/2), flat terrain level path (3), düşük-MSL edge daha ucuz
  (4, cost 0.090 vs 0.1305), mid-segment ridge 2-hücrelik primitive'in
  KENDİ intermediate sampling'i ile yakalandı (5), coarse safety'nin
  YAPISAL olarak sadece stored (=MAX) elevation'ı kullandığı (MEAN bu
  modüle hiç girmiyor, import bile edilmiyor) doğrulandı (6), endpoint
  lift/no-lift (7/8, gerçek START/GOAL hücre değerleriyle bit-exact
  eşleşti), NoData crossing INVALID (9), state literal
  `(row,col,z_index)` (10). **GERÇEK 6.48km BENCHMARK (TEK run, 30k cap,
  retry YOK)**: `status=success`, runtime=247.76s (primitive cache
  KASITLI OLARAK yok, "basitleştirilmiş" talimatı gereği -- bu yüzden
  yavaş ama sınır içinde tamamlandı), expanded=24,959/30,000,
  max_open=12,348. **Path**: 31 node, xy_length=6480.0m (=D_ref, dead
  straight XY corridor), 3d_length=6541.9m, min_MSL=3360.0 (start'tan
  400m aşağı), mean_MSL=3545.8, max_MSL=3800.0 (lifted goal),
  total_climb=440.0/total_descent=400.0 (net +40 = 3800-3760, tutarlı),
  min_coarse_MAX_AGL=**200.0m** (tam sınırda -- düşük-MSL tercihi
  beklenen şekilde constraint'i zorluyor), max_flight_path_angle=8.43°
  (10° limitinin altında). Cost: distance=1.0095 altitude=0.4001
  total=1.4096. **Relief diagnostic** (sadece bilgi): path boyunca
  mean_relief=25.00 max_relief=65.89 p95=63.83 -- Stage 34.5'in ROI-geneli
  max relief'i (282.10m) ile kıyaslandığında bu path hiçbir aşırı-relief
  hücreden geçmiyor (makul/düz bir koridor). **FINE REPLAY (30m DEM,
  <=10m sample, `evaluate_agl`/`evaluate_transition` reuse edildi, yeni
  kod YAZILMADI)**: min_AGL=**202.31m** (coarse'un raporladığı 200.0'dan
  biraz daha gevşek -- beklenen, çünkü coarse MAX daha KONSERVATİF),
  max_angle=8.43° (coarse ile aynı), **violation_count=0**. CSV:
  `outputs/stage35_coarse_path.csv` (31 satır). **Bu, Stage 19'dan
  itibaren denenen HERHANGİ bir mekanizmanın (dominance/bucket/incumbent/
  weighted-A*/epsilon-sweep/normalized-cost/goal-region -- Stage 20-33)
  bulamadığı ilk tam start-goal çözümü** -- coarse-to-fine hipotezini
  (Stage 27'den beri öngörülen) doğruluyor. Bu turda YAPMAZ listesi
  (corridor, fine A*, epsilon sweep, terrain-aware heuristic, MIN/MEAN/
  RELIEF routing cost, Numba/GPU, heading/turn radius, variable angle,
  cost tuning) hiçbiri implement edilmedi. **Sonraki adım (corridor
  aşaması) için hazır** -- karar kullanıcıda, bu turda başlatılmadı.

- **Stage 35.1 — Coarse Primitive Safety Precomputation: search davranışı
  bit-exact korunarak search-only runtime 247.76s'den 4.70s'e düştü
  (~%53x), toplam (preprocessing+search) 41.86s (5.92x speedup). PASS.**
  `planner/coarse_astar.py`'ye `precompute_coarse_primitive_safety()`
  eklendi -- her `(row,col,primitive_id)` için (Z state key'e GİRMİYOR)
  `required_start_msl = max_i(terrain_max(i)+min_agl-primitive.dz_m*t_i)`
  önceden hesaplanıyor; search sırasında pahalı per-sample terrain
  query yerine tek bir `start_msl >= required_start_msl` karşılaştırması
  yeterli. Endpoint transition (açı) kontrolü tekrar YAPILMADI --
  `build_primitive_set()` bunu primitive'in kendi sabit geometrisi için
  zaten bir kere garantiliyor, konum/irtifadan bağımsız. out_of_bounds/
  nodata konuma bağlı ama irtifadan bağımsız olduğu için `static_invalid`
  olarak (irtifa ne olursa olsun) cache'leniyor. `coarse_astar_search()`'e
  opsiyonel `precomputed_safety=None` parametresi eklendi -- `None` iken
  (default) Stage 35 davranışı BİREBİR AYNI kalıyor (yeni kod pasif),
  verilince `_generate_coarse_neighbors` `evaluate_primitive()` yerine
  O(1) `precomputed_primitive_validity()` çağırıyor. **Eşdeğerlik
  testleri (1-7) TOPLAM ~40,000+ karşılaştırma, 0 MISMATCH**: flat (7200),
  ridge (2592), NoData (1728), bounds (768), level/climb/descent (3×1536),
  15 farklı start-MSL seviyesi (23040), gerçek ROI'de 4000 rastgele
  (row,col,primitive,start_msl) örneği -- hepsi eski (`evaluate_primitive`)
  ve yeni (cache lookup) arasında bit-exact aynı valid/reason verdi.
  **Precompute istatistikleri** (gerçek 111×111×24 grid): entry_count=
  **295,704** (=111×111×24 tam), preprocessing_time=**37.16s**,
  static_invalid_count=7,512 (~%2.5, sabit/statik olarak invalid --
  konumdan kaynaklı, irtifadan bağımsız), approx_memory=**59.14MB**.
  **GERÇEK 6.48km benchmark (TEK run, AYNI ayarlar: 30k cap, epsilon
  denenmedi, 150k retry yapılmadı)**: `status=success`,
  search_runtime=**4.70s** (baseline 247.76s'e göre **~52.7x** search-only
  hızlanma), expanded=**24,959** (Stage 35 ile BİREBİR AYNI), max_open
  aynı, path_node_count=31 (aynı), xy_length=6480.0m (aynı), min_MSL=
  3360.0/max_MSL=3800.0 (aynı), total_cost=1.409649 (aynı, 6 ondalık
  hane bit-exact) -- **davranış tamamen korundu, sadece hız değişti**.
  Toplam runtime (preprocessing+search)=41.86s, Stage 35'in 247.76s'ine
  göre **5.92x** toplam speedup (tek-seferlik önişleme maliyeti
  amortisman edilmedi bu turda -- cache birden fazla search'te
  yeniden kullanılırsa toplam speedup search-only'nin 52.7x'ine
  yaklaşabilir, bu turda test edilmedi). **Fine replay Stage 35 ile
  BİREBİR AYNI**: min_AGL=202.31m, max_angle=8.43°, violations=0.
  Script: `scripts/validate_coarse_precompute.py`. Bu turda YAPMAZ
  listesi (weighted A*, epsilon tuning, corridor, fine A*, heuristic/
  cost/state değişikliği, min/mean/relief routing, Numba/GPU, heading,
  variable angle) hiçbiri implement edilmedi. **Sonraki adım corridor
  olabilir** (coarse search artık pratik hızda tekrar tekrar
  çalıştırılabilir) -- karar kullanıcıda, bu turda başlatılmadı.

- **Stage 35.2 — Coarse Weighted A* Expansion Reduction: sadece open-heap
  priority (`f=g+epsilon*h`) değişti; epsilon=1.5 expansion'ı 24,959'dan
  836'ya (**%96.6 azalma**), search runtime'ı 4.70s'den 0.32s'e
  (**~15x**) düşürdü, cost sadece **%2.21** arttı, fine replay hâlâ PASS
  (0 violation). PASS -- epsilon=1.5 coarse guidance default'u olarak
  öneriliyor.** `planner/coarse_astar.py`'ye `coarse_astar_search`'e
  opsiyonel `epsilon_search=1.0` parametresi eklendi -- TEK etkisi
  open-heap'e push edilen `f_score = tentative_g + epsilon_search*h_val`
  (ve start düğümünün ilk push'u da aynı şekilde); `g_score`/
  `g_distance`/`g_altitude` (gerçek biriken maliyet) HİÇ epsilon ile
  ölçeklenmiyor -- döndürülen path'in cost'u her zaman gerçek fiziksel
  maliyet. Stage 25'in reopening mekanizması KASITLI OLARAK eklenmedi
  (spesin "yalnız ordering" talimatı gereği) -- `closed` set permanent
  kalıyor, bu daha yüksek-cost (ama hâlâ tam ve güvenli) bir path'e yol
  açabilir, asla güvensiz bir path'e (safety hâlâ tamamen precomputed
  cache/`evaluate_primitive`'in elinde, epsilon'dan bağımsız).
  `epsilon_search=1.0` Stage 35/35.1'i bit-exact reprodüksiyon (1.0 ile
  çarpmak hiçbir biti değiştirmiyor). **Precompute cache BİR KEZ
  oluşturuldu (39.9s) ve 3 epsilon'un HEPSİNDE reuse edildi** (spesin
  talimatı gereği, epsilon başına yeniden precompute YAPILMADI).
  ```
  eps   expansions  runtime   cost       cost_delta   path   status
  1.00       24,959   4.70s   1.409649   baseline     31     success (Stage 35.1 baseline, tekrar çalıştırılmadı)
  1.10       18,966   3.96s   1.409649   -0.00%       31     success
  1.25        9,248   2.29s   1.412427   +0.20%       31     success
  1.50          836   0.32s   1.440851   +2.21%       27     success
  ```
  **epsilon=1.10'da cost/path TAM AYNI kaldı** (31 node, aynı 6 ondalık
  hane -1.409649) -- sadece %24 daha az expansion, hiçbir kalite kaybı
  yok. **epsilon=1.25'te path uzunluğu (31 node) korunuyor, cost sadece
  %0.20 artıyor** -- %63 expansion azalması için ihmal edilebilir bir
  bedel. **epsilon=1.50'de path 27 node'a kısalıyor** (min_MSL 3360->3400,
  yani biraz daha az derine iniyor), cost %2.21 artıyor, ama expansion
  %96.6 azalıyor (836) ve runtime 0.32s'e düşüyor -- "%<5000 expansion,
  ~1s veya altı runtime" kriterini AÇIKÇA en güçlü şekilde sağlıyor.
  Üçünde de `min_coarse_MAX_AGL>=200m` (1.5'te 203.2m, marj korunuyor) ve
  `max_flight_path_angle=8.43°<=10°` -- **safety üçünde de hiç
  bozulmadı**. **Seçilen epsilon=1.5 için FINE REPLAY (tek run, diğer
  epsilon path'leri replay edilmedi)**: `min_AGL=209.90m` (>=200m),
  `max_angle=8.43°`, **violations=0** -- PASS. **Öneri: epsilon=1.5
  coarse guidance default'u olarak kullanılabilir** -- bu coarse search
  zaten final optimal path değil (fine planner corridor içinde asıl
  optimizasyonu yapacak), bu yüzden %2.21'lik cost artışı ve 27-vs-31
  node'luk path farkı kabul edilebilir; kazanılan ~15x search-hızı,
  coarse guide'ın corridor aşamasında (muhtemelen birden fazla kez)
  çalıştırılması ihtiyacı düşünüldüğünde değerli. Script:
  `scripts/validate_coarse_weighted.py`. Bu turda YAPMAZ listesi (disk
  cache, corridor, fine A*, cost/heuristic/state/primitive değişikliği,
  terrain-aware heuristic, Numba/GPU, eski PASS testlerini tekrar
  çalıştırma) hiçbiri implement edilmedi. Sonraki adım (corridor) bu
  turda BAŞLATILMADI, karar kullanıcıda.

- **Stage 36 — Coarse Path -> Fine XY Corridor: epsilon=1.5 coarse path
  etrafında ±300m XY-only corridor mask üretildi (fine A* ÇALIŞTIRILMADI).
  PASS.** Yeni modül `planner/corridor.py` (`build_xy_corridor_mask`,
  `min_distance_to_polyline`, `bfs_connected` -- scipy bağımlılığı yok,
  plain numpy/BFS). Epsilon=1.5 coarse path Stage 35.2'de diske
  kaydedilmemişti -- bu adımda TEK SEFER yeniden çalıştırıldı (aynı
  precompute cache mekanizması, 36.19s) ve `outputs/
  stage36_coarse_path_eps1.5.csv`'ye kaydedildi (27 node, Stage 35.2 ile
  aynı: expanded=836). Corridor SADECE XY -- Z/altitude hiç
  kısıtlanmadı; her fine hücre merkezinin coarse path polyline'ına
  (segment-clamp projeksiyonu, vektörize) minimum mesafesi <=300m ise
  `True`. **İstatistikler**: fine_grid=(333,333)=110,889 hücre,
  corridor_cell_count=**4,853**, coverage=**%4.38**, approx_area=4.37km²,
  **search-space XY reduction=%95.62** (sadece diagnostic, henüz bir hız
  iddiası değil). START (row=48,col=276, distance=42.4m) ve GOAL
  (row=264,col=276, distance=30.0m) doğal olarak corridor içinde çıktı
  (force-include hiç gerekmedi). **start<->goal 8-connected BFS ile XY
  bağlantılı (PASS)**. Perpendicular width taraması (10 örnek nokta):
  min=mean=max=**620m** (beklenen ~600m'den biraz fazla -- 10m adım
  quantization + polyline'ın bükümlü olduğu noktalarda convex köşe
  etkisi, hata değil). **7 test**: 1 (düz sentetik polyline, 260m
  içeride/340m dışarıda doğru), 2 (L-bükümde köşe boşluğu yok, clamp
  projeksiyonu doğru), 3 (start/goal dahil), 4 (mask shape==fine grid
  shape), 5 (XY connectivity), 6 (>305m olan 106,028 hücrenin hiçbiri
  yanlışlıkla dahil değil), 7 (<=295m olan 4,397 hücrenin hiçbiri
  yanlışlıkla hariç değil, ayrıca distance=300.0000m'ye en yakın hücre
  doğru şekilde `<=` ile dahil edildi) -- **ALL PASS**. Çıktılar:
  `outputs/stage36_coarse_path_eps1.5.csv`, `outputs/
  stage36_corridor_mask.npy`, `outputs/stage36_corridor_mask.tif`
  (fine DEM ile aynı transform/CRS/shape). Script: `scripts/
  build_fine_corridor.py`. Bu turda YAPMAZ listesi (fine A*, corridor
  width sweep, 600m retry, cost/heuristic değişikliği, coarse planner
  tuning, terrain stats cost, Z corridor, Numba/GPU) hiçbiri implement
  edilmedi. Sonraki adım (corridor içinde fine A*) bu turda
  BAŞLATILMADI, karar kullanıcıda.

- **Stage 37 — Corridor-Constrained Fine A*: fine A*'a opsiyonel
  `corridor_mask` eklendi (None=eski davranış bit-exact); gerçek
  6.48km testinde wall-clock %62.6 düştü (98.95s->37.00s) ve max_open
  %33.1 azaldı (51,475->34,435), AMA 30k'da hâlâ `search_limit_reached`
  -- expansion sayısı AYNI (30,000, cap'e bağlı, %0 değişim). Corridor
  30k yakınsama problemini ÇÖZMEDİ, sadece expansion'ı ucuzlattı.**
  `planner/astar.py`'ye: `_generate_neighbors`'a opsiyonel
  `corridor_mask: Optional[np.ndarray]` (shape=fine ROI shape, True=
  corridor içi) -- `out_of_bounds` kontrolünden SONRA, `evaluate_primitive`/
  cache'den ÖNCE kontrol ediliyor (`outside_corridor` reddi, pahalı
  terrain evaluation'a hiç gitmiyor). `astar_search`'e aynı isimli
  parametre + `SearchResult.corridor_reject_count` eklendi.
  **Corridor SADECE XY** -- z_index/altitude hiç kısıtlanmadı; AGL/
  terrain/NoData/açı kontrolleri corridor içindeki HER candidate için
  aynen devam ediyor (mimari olarak ayrık: corridor kontrolü geçen bir
  primitive hâlâ `evaluate_primitive`'den geçmek zorunda). `None`
  (default) -- Stage 24/25'in regresyon scriptleri (`validate_
  incumbent_pruning.py`, `validate_weighted_astar.py`) bu değişiklikten
  SONRA tekrar çalıştırılıp kayıtlı sayılarla (expanded=679/494/679/494,
  C*=2424.1952, certified ratio=1.0000<=1.05 vb.) bit-exact eşleşti --
  legacy davranış hiç bozulmadı. **4 küçük entegrasyon testi ALL PASS**:
  1 (`corridor_mask=None` == tamamen-True mask, birebir aynı accepted/
  rejected/corridor_reject sayıları), 2 (successor corridor dışında ->
  24/24 primitive `outside_corridor` ile reddedildi, `evaluate_primitive`
  hiç çağrılmadı), 3 (successor corridor içinde -> 24/24 normal
  evaluate_primitive'e gitti, hepsi kabul edildi), 4 (gerçek Stage 36
  mask'inde START(48,276) ve GOAL(264,276) corridor içinde, doğrulandı).
  **GERÇEK 6.48km testi (Stage 33 ile TÜM diğer ayarlar aynı: normalized
  cost, altitude_reference=3240, H_scale=1000, w_altitude=1.25,
  epsilon=1.10, target_suboptimality=1.05, dominance OFF, incumbent ON,
  goal tolerance ±35m/±25m, 30k cap, retry YOK -- eski baseline TEKRAR
  ÇALIŞTIRILMADI)**: `status=search_limit_reached`, wall=**37.00s**
  (Stage 33'ün 98.95s'ine göre **%62.6 daha hızlı**), expanded=30,000
  (Stage 33 ile AYNI -- cap'e ulaşıldığı için expansion sayısında
  değişim YOK, `%0.00`), max_open=**34,435** (Stage 33'ün 51,475'ine
  göre **%33.1 daha küçük**), `corridor_reject_count=53,790` (30k
  expansion boyunca reddedilen candidate'lerin büyük kısmı, pahalı
  terrain evaluation'a hiç gitmeden), cache_hit_rate=0.768.
  `first_solution_found=False`, `incumbent_updates=0` (Stage 33 ile
  aynı, hiç iyileşme yok). **closest_distance_to_goal_region_m=4,918.11m**
  (Stage 33'ün 5,066.80m'sine göre sadece **~%2.9 daha yakın**, en yakın
  state row=99 -- yolun sadece **~%23.6'sı**, Stage 33'ün de benzer
  ölçekteki ilerlemesiyle karşılaştırılabilir). **Sonuç: corridor,
  %95.62 XY-alan küçülmesine rağmen, search'ün GERÇEKTEN goal'e
  ulaşması için gereken expansion SAYISINI azaltmadı** -- sadece her
  expansion'ı ucuzlattı (daha az candidate pahalı terrain sample'ına
  gidiyor, bu yüzden wall-clock/max_open düştü). Bu, darboğazın büyük
  kısmının XY breadth'ten değil, dikey (z_index × trend × bucket) state
  space'in kendisinden kaynaklandığını düşündürüyor -- corridor tek
  başına yeterli değil, muhtemelen weighted-A*/epsilon ile BİRLİKTE
  (bu turda denenmedi, YAPMAZ listesinde) daha etkili olabilir. Path
  bulunamadığı için CSV yazılmadı. Script: `scripts/
  benchmark_corridor_fine_real.py`, `scripts/validate_astar_corridor.py`.
  **Not (transparanlık)**: `scripts/validate_primitive_cache.py` (Stage
  15'in eski scripti) `_generate_neighbors`'ın artık 5-tuple döndürmesi
  nedeniyle artık çalışmıyor -- proje konvansiyonu gereği (Stage 22'nin
  `compute_edge_cost` imza değişikliğinde olduğu gibi) eski scriptler
  geriye dönük düzeltilmiyor, sadece not ediliyor. Bu turda YAPMAZ
  listesi (corridor width sweep, 600m retry, coarse tuning, epsilon
  tuning, cost/heuristic/state değişikliği, heading/turn radius,
  variable angle, Numba/GPU, eski PASS testlerini tekrar çalıştırma)
  hiçbiri implement edilmedi. Sonraki adım (corridor+weighted-A*
  kombinasyonu mu, yoksa başka bir yaklaşım mı) bu turda BAŞLATILMADI,
  karar kullanıcıda.

- **Stage 37.1 — 3D Corridor Fine A*: Z-tube (±200m) hemen hemen HİÇ
  bağlayıcı çıkmadı (41 reject, XY'nin 53,790'ına karşı) -- expanded/
  max_open/wall-clock/closest-distance Stage 37 ile PRATİKTE AYNI. Yeni
  state-composition diagnostiği asıl darboğazı ortaya çıkardı: sadece
  1,012 benzersiz XY hücresi (4,853 corridor hücresinin ~%20.9'u)
  ziyaret edildi, ama bu KÜÇÜK alan içinde Z (~7.0x) ve history (~4.2x)
  çarpımsal olarak state'i şişiriyor -- darboğaz "Z" veya "history"
  tek başına değil, XY ilerlemesinin kendisinin zaten zayıf olması VE
  bunun üstüne Z×history çarpımının eklenmesi. SUCCESS gelmedi.**
  `planner/astar.py`'ye: `_generate_neighbors`/`astar_search`'e opsiyonel
  `z_guide_grid`/`z_guide_tolerance_m` (her ikisi de None=eski davranış,
  Stage 37 XY-only corridor bit-exact korunuyor) -- XY kontrolünden
  SONRA, `evaluate_primitive`'den ÖNCE: `abs(new_z_msl - z_guide_grid[
  new_row,new_col]) > z_guide_tolerance_m` ise `outside_z_guide_tube`
  reddi (`z_corridor_reject_count`). Ayrıca 6 yeni state-composition
  diagnostiği (`unique_expanded_xy/xyz`, `unique_full_states`, `avg_z_
  states_per_xy`, `avg_history_states_per_xyz`, `max_z_states_in_one_xy`)
  -- HER search çağrısında ücretsiz hesaplanıyor (sadece final `closed`
  set üzerinde bir geçiş), corridor kullanılmasa da. `planner/
  corridor.py`'ye `build_z_guide_grid()` eklendi (coarse 3D polyline'ın
  segment-clamp projeksiyonuyla aynı XY-en-yakın segmentten Z lineer
  interpolasyonu -- Stage 36'nın `min_distance_to_polyline`'ıyla aynı
  matematik, Z'yi de taşıyor). **Regresyon**: Stage 24/25 scriptleri
  (`validate_incumbent_pruning.py`, `validate_weighted_astar.py`) bu
  değişiklikten SONRA tekrar çalıştırılıp kayıtlı sayılarla bit-exact
  eşleşti. **5 yeni entegrasyon testi ALL PASS**: 1 (segment üzerinde Z
  interpolasyonu doğru: orta nokta ~1100m, başlangıç ~1000m), 2/3
  (z_guide=1300 (mevcut irtifaya yakın) -> 24/24 kabul; z_guide=1600
  (300m uzak, primitive'lerin ±20m'lik z_step'i bile bu farkı
  kapatamıyor) -> 24/24 `outside_z_guide_tube` ile reddedildi), 4 (XY
  reddi Z tüpünden ÖNCE tetikleniyor -- Z tüpü hiç kontrol edilmiyor),
  5 (gerçek coarse path: START z_guide=3760.0/diff=0.0, GOAL
  z_guide=3795.6/diff=35.6 -- ikisi de ±200m tüpüne DOĞAL olarak
  giriyor, force-include hiç gerekmedi). **GERÇEK 6.48km testi (Stage
  37 ile TÜM diğer ayarlar aynı, sadece Z-tube eklendi, eski baseline
  TEKRAR ÇALIŞTIRILMADI)**: `status=search_limit_reached`,
  expanded=30,000 (Stage 37 ile AYNI), max_open=**34,435 (Stage 37 ile
  BİREBİR AYNI)**, wall=37.80s (Stage 37'nin 37.00s'ine göre sadece
  `%2.17` fark, gürültü seviyesinde), `xy_corridor_reject_count=53,790`
  (BİREBİR AYNI), **`z_corridor_reject_count=41`** (ÇOK küçük -- Z-tube
  pratikte hiç bağlayıcı olmadı), `closest_distance_to_goal_region_m=
  4,918.11m` (BİREBİR AYNI), en yakın state hâlâ row=99. **State-
  composition diagnostiği (asıl bulgu)**: `unique_expanded_xy=1,012`
  (4,853 corridor hücresinin sadece **~%20.9'u** ziyaret edildi!),
  `unique_expanded_xyz=7,089`, `unique_full_states=30,000`,
  `avg_z_states_per_xy=7.005`, `avg_history_states_per_xyz=4.232`,
  `max_z_states_in_one_xy=13`. Çarpım tutarlı: 1,012×7.005×4.232≈30,000.
  **Yorum**: Z-tube'un neredeyse hiç tetiklenmemesi (41/53,831),
  search'ün DOĞAL DAVRANIŞININ zaten coarse path'in ±200m'lik
  civarında kaldığını gösteriyor -- yani Z-tube constraint DEĞİL,
  search zaten kendiliğinden o bölgede. Gerçek darboğaz: search
  30,000 expansion'ın HEPSİNİ corridor'un sadece beşte birlik bir
  diliminde (row~48-99 civarı) Z (~7x) ve history (~4.2x) kombinasyonlarını
  tekrar tekrar keşfederek harcıyor, corridor'un geri kalan ~%79'una
  hiç ilerleyemiyor. **Sınıflandırma: darboğaz Z VEYA history'nin
  TEKİ değil -- ikisinin ÇARPIMI, VE bunun üstüne XY ilerlemesinin
  kendisinin (corridor içinde bile) yavaş olması.** Path bulunamadığı
  için CSV yazılmadı. Scriptler: `scripts/validate_astar_3d_corridor.py`,
  `scripts/benchmark_3d_corridor_fine_real.py`. Bu turda YAPMAZ listesi
  (epsilon tuning, Z tolerance sweep, corridor width sweep, trend/bucket
  değişikliği, cost/heuristic değişikliği, coarse tuning, Numba/GPU,
  heading/turn radius, variable angle) hiçbiri implement edilmedi.
  Sonraki adım bu turda BAŞLATILMADI, karar kullanıcıda.

- **Stage 37.2 — Fine A* History-Free Diagnostic: trend/bucket kaldırılınca
  hedefe ilerleme ÇARPICI biçimde arttı (goal-box mesafesi 4,918.11m'den
  1,499.49m'e düştü, %69.5 azalma; en yakın state row=99'dan row=214'e
  ilerledi, yolun %23.6'sından %76.9'una) -- AMA yine de 30k'da SUCCESS
  gelmedi. History gerçekten büyük bir darboğazdı, ama tek başına yeterli
  değildi.** `planner/astar.py`'ye diagnostic-only `freeze_history=False`
  parametresi eklendi (production default DEĞİŞMEDİ) -- `True` iken
  `_generate_neighbors` `_next_trend_and_bucket`'ı hiç çağırmıyor, successor
  `(trend,bucket)`'ı ebeveyninkini DONMUŞ olarak devralıyor (başlangıçtaki
  `(0,BUCKET_SHORT)`'ta sonsuza kadar sabit kalıyor), ve `compute_edge_cost`
  `disable_reversal_cost=True` ile çağrılıyor (reversal terimi kesin 0.0).
  Augmented state hâlâ teknik olarak 5-tuple ama son iki alan hiç
  değişmediği için fiilen `(row,col,z_index)` ile bijektif -- Stage 37.1'in
  ZATEN var olan `unique_full_states`/`unique_expanded_xyz` diagnostiği bunu
  otomatik doğruluyor (`avg_history_states_per_xyz==1.0` garantisi).
  **Regresyon**: Stage 24/25/37/37.1 script'leri bu değişiklikten SONRA
  tekrar çalıştırılıp bit-exact eşleşti (freeze_history default=False
  hiçbir mevcut davranışı bozmadı). **4 yeni entegrasyon testi ALL PASS**:
  1 (frozen modda `unique_full_states==unique_expanded_xyz`, ratio=1.0),
  2 (aynı senaryoda normal modda ratio=2.740 iken frozen modda 1.0 --
  history'nin gerçekten XYZ'yi çoğaltmadığı doğrulandı), 3 (NoData/AGL
  reddi frozen/normal modda identik), 4 (`freeze_history=False`
  default'un normal davranışla bit-exact aynı olduğu, argüman verilmese
  de verilse de). **GERÇEK 6.48km testi (Stage 37.1 ile TÜM diğer ayarlar
  aynı, sadece `freeze_history=True`, eski baseline TEKRAR
  ÇALIŞTIRILMADI)**: `status=search_limit_reached` (**hâlâ FAIL**),
  expanded=30,000 (aynı cap), wall=**132.33s** (Stage 37.1'in 37.80s'ine
  göre **~3.5x daha yavaş** -- cache_hit_rate 0.768'den **0.013'e çöktü**,
  çünkü search artık aynı fiziksel hücreleri çok daha az tekrar
  ziyaret ediyor), max_open=25,318 (Stage 37.1'in 34,435'inden düşük),
  `reopened_states=463` (Stage 37.1'de 0'dı). **State-composition**:
  `unique_expanded_xy=3,034` (Stage 37.1'in 1,012'sinin **3x'i** --
  corridor'un ~%62.5'i artık ziyaret edildi, 4,853'ün), `unique_expanded_
  xyz=29,537`, `unique_full_states=29,537` (birebir aynı -- history
  çarpımı sıfır, tasarım gereği doğrulandı), `avg_z_states_per_xy=9.735`.
  **En kritik bulgu: `closest_distance_to_goal_region_m=1,499.49m`**
  (Stage 37.1'in 4,918.11m'sine göre **%69.5 daha yakın**), en yakın
  state row=**214** (col=279, Stage 37.1'in row=99'una göre — yolun
  **%76.9'u**, önceki %23.6'ya karşı). **Yorum**: history'nin (trend/
  bucket) kaldırılması search'ün gerçek FİZİKSEL ilerlemesini büyük
  ölçüde iyileştirdi -- bu, Stage 37.1'in "darboğaz Z ve history'nin
  ÇARPIMI" teşhisini DOĞRULUYOR ve history'nin bu çarpımdaki payının
  önemli/belirleyici olduğunu gösteriyor (arındırılınca ilerleme 3x'e
  yakın arttı). Ama SUCCESS hâlâ gelmedi -- yani history TEK BAŞINA
  darboğazın tamamı değildi; kalan mesafe (1,499m, corridor'un/rotanın
  son ~%23'ü) hâlâ 30k expansion'ı aşıyor, üstelik cache_hit_rate
  çöküşü nedeniyle KALAN her expansion da daha maliyetli hale geldi
  (wall-clock 3.5x arttı). **Sınıflandırma: ana sorun artık kısmen
  "search guidance" (goal'e daha agresif/etkili yönlendirme) ve kısmen
  hâlâ ham hacim (kalan ~%23'lük mesafe için 30k'nın yetmemesi) --
  history en büyük tek faktördü ama TEK faktör değildi.** Path
  bulunamadığı için CSV yazılmadı. Scriptler: `scripts/
  validate_astar_history_free.py`, `scripts/
  benchmark_history_free_fine_real.py`. Bu turda YAPMAZ listesi (epsilon
  sweep, corridor/Z tolerance değişikliği, heuristic/cost tuning, başka
  epsilon'lu Weighted A*, Numba/GPU, heading/turn radius, variable
  angle, eski PASS testlerini tekrar çalıştırma) hiçbiri implement
  edilmedi. Sonraki adım bu turda BAŞLATILMADI, karar kullanıcıda.

- **Stage 37.3 — History-Free Fine Weighted A* Sweep: 🎯 PROJENİN Stage
  20'den beri İLK BAŞARILI TAM 6.48km FINE PATH'İ (30m grid). epsilon=1.5
  VE epsilon=1.7, freeze_history=True + 3D corridor ile tam, güvenli bir
  path buldu (epsilon=1.3 bulamadı). Seçilen en iyi aday: epsilon=1.7,
  sadece 57 expansion'da ilk çözüm, fine terrain replay SAFETY PASS.**
  `planner/astar.py`'ye `external_primitive_cache` parametresi eklendi
  (default None = eski davranış bit-exact) -- üç epsilon run'ı TEK bir
  paylaşılan primitive cache dict'i kullanarak çalıştırıldı (spesin
  talimatı gereği, epsilon başına yeniden precompute/cache YOK).
  **KRİTİK METODOLOJİK DÜZELTME (script'in ilk versiyonunda kendi
  kendine yakalandı)**: `result.status=="success"` SADECE %5 certified
  bound kanıtlandığında true oluyor (`target_suboptimality=1.05` hâlâ
  aktif) -- ama gerçek, güvenli, TAM bir path çok daha ÖNCE bulunmuş
  olabilir (`first_solution_cost`/`incumbent_path` üzerinden) ve
  30k cap, certificate tamamlanmadan yetişebilir. İlk script çalıştırması
  epsilon=1.5/1.7 için `closest_distance_to_goal_region_m=0.00` gösterip
  hâlâ "FAIL" raporladı -- bu tutarsızlık fark edilip script düzeltildi:
  gerçek başarı ölçütü `first_solution_cost<inf` (`incumbent_path`), status
  değil. Düzeltme sonrası: **her ikisi de (1.5 ve 1.7) GERÇEKTEN tam path
  bulmuş.** **Sonuçlar** (paylaşılan cache, aynı 3D corridor/normalized
  cost/safety):
  ```
  eps   status               found_path  expansions  runtime  reopened  cost
  1.1   search_limit_reached  False       30000       132.33s  463       n/a  (Stage 37.2 baseline, tekrar çalıştırılmadı)
  1.3   search_limit_reached  False       30000       35.08s   24647     n/a
  1.5   search_limit_reached  True        30000       16.81s   28300     1.460453
  1.7   search_limit_reached  True        30000       13.37s   28840     1.497221
  ```
  epsilon=1.3: hâlâ path bulamadı (closest_distance=614.86m -- Stage
  37.2'nin 1499.49m'inden daha yakın ama hâlâ tam değil). epsilon=1.5:
  first_solution_expanded=**19,732**, 4 incumbent update, cost=1.460453,
  81 node. epsilon=1.7: first_solution_expanded=**57** (pratikte anında),
  140 incumbent update (agresif ordering çok daha fazla iyileştirme
  buluyor), cost=1.497221, 63 node -- eps=1.5'e göre sadece **%2.52**
  daha pahalı. **reopen_ratio üçünde de çok yüksek (%82-%96)** -- ama bu
  ARTIK zararlı thrashing değil, GERÇEK ilerlemeye dönüşen reopening
  (Stage 37.2'nin bulgusuyla tutarlı: history kaldırılınca reopening
  hacmi yüksek kalıyor ama artık anlamlı). **Seçim kriteri** (reopen_ratio
  <0.98 VE cost_delta<%10 kabul barajını her ikisi de geçti, en az
  `first_solution_expanded` tercih edildi): **epsilon=1.7 seçildi**
  (57 expansion, %2.52 cost farkı kabul edilebilir). **SADECE epsilon=1.7
  için fine DEM replay/validation yapıldı** (diğerleri replay edilmedi,
  spesin talimatı gereği): `path_node_count=63`, `3d_length=6534.4m`,
  `min_MSL=3520.0` `max_MSL=3780.0`, `min_AGL=200.8m` (>=200m marj
  korunuyor), `max_flight_path_angle=9.46°` (<=10° limitin altında),
  `total_climb=500.0` `total_descent=520.0`. **SAFETY CHECK: PASS.**
  Path: `outputs/stage37_3_best_path.csv`. **Bu, Stage 20'den beri
  denenen HERHANGİ bir mekanizmanın (dominance/bucket/incumbent/
  weighted-A*/epsilon-sweep/normalized-cost/goal-region/XY-corridor/3D-
  corridor/history-removal, tek tek veya art arda) ayrı ayrı BULAMADIĞI
  ilk tam, güvenli, gerçek-ölçekli (30m) 6.48km fine path'i** -- ancak
  bunun için GEREKLİ kombinasyon şuydu: coarse-guided 3D corridor +
  trend/bucket history'nin kaldırılması + orta-agresif weighted ordering
  (epsilon>=1.5). Script: `scripts/
  benchmark_history_free_epsilon_sweep.py`. Bu turda YAPMAZ listesi
  (epsilon=1.10 tekrar, corridor/cost/heuristic/history yapısı
  değişikliği) hiçbiri implement edilmedi. Sonraki adım bu turda
  BAŞLATILMADI, karar kullanıcıda.

- **Stage 37.4 — First-Solution Speed + Route Quality: epsilon=1.7,
  freeze_history=True, `stop_on_first_solution=True` ile ilk tam güvenli
  path SADECE 57 expansion / 0.33s'de bulundu. SAFETY PASS. Ama vertical
  smoothness diagnostiği path'in belirgin roller-coaster karakterli
  olduğunu ortaya çıkardı (30 reversal, 29'u <300m aralıklı).**
  `planner/astar.py`'ye `stop_on_first_solution=False` (default) eklendi
  -- `True` iken goal-region pop'unda `bounded_mode` aktif olsa bile
  HEMEN duruyor (certificate aranmıyor); tüm running counter'lar
  (`expanded_nodes`, `max_open_size`, `reopened_states`, `generated_
  neighbors`) doğal olarak "ilk çözüm anındaki" değerlerini yansıtıyor,
  ekstra snapshot alanı gerekmedi. Regresyon (Stage 24/25) bit-exact
  PASS. **GERÇEK TEK RUN sonuçları**: `first_solution_expanded=57`,
  `runtime=0.3427s`, `max_open=1164`, `reopened_states=0` (ilk
  çözümden önce hiç reopen olmamış), `generated_states=1344`,
  `cache_hit_rate=0.000` (soğuk cache, paylaşılan cache Stage 37.3'ten
  farklı olarak burada kullanılmadı -- spesin "tek run" talimatına
  uygun). **SAFETY**: min_AGL=206.31m (>=200m), max_angle=9.46°
  (<=10°), 0 NoData/bounds violation, fine DEM replay PASS. **DISTANCE**:
  xy_length=6450.0m, 3d_length=6537.7m, direct_distance=6480.0m,
  **detour=-0.46%** (aslında düz çizgiden biraz KISA -- xy_length ölçümü
  gerçek 3D path'in XY izdüşümü, 6480m tam olarak start-goal düz çizgisi
  değilse bu makul). **LOW-MSL QUALITY**: min_MSL=3520.0, max_MSL=3780.0,
  distance-weighted mean_MSL=3636.7, total_descent=540.0,
  total_climb=520.0, normalized_distance=1.008909,
  normalized_altitude=0.500324, **total_diagnostic_cost=1.509232**.
  **VERTICAL SMOOTHNESS (sadece ölçüldü, cost'a eklenmedi)**:
  `vertical_reversal_count=30`, **`short_reversal_count (<300m)=29`**
  (reversal'ların neredeyse HEPSİ kısa-aralıklı), `longest_continuous_
  descent=260.0m`, `longest_continuous_climb=40.0m`, `total_vertical_
  variation=1060.0m` -- 6480m'lik bir rotada 1060m toplam dikey hareket
  ve 29 kısa reversal, path'in ASCII profilinin gösterdiği tek-basit-
  vadi görünümünün ALTINDA, primitive seviyesinde belirgin bir "roller-
  coaster" karakteri olduğunu gösteriyor (reversal cost=0 olduğu için
  bu freeze_history modunda hiç cezalandırılmıyor). **REFERENCE
  COMPARISON**: direct level'a göre **%8.53 daha ucuz**; Stage 32'nin
  deep candidate'ine göre (SADECE aynı normalized distance+altitude
  objective açısından, global optimum kanıtı DEĞİL) **%10.05 daha
  pahalı** (min_MSL 120m daha yüksek, 3520 vs 3400); Stage 37.3'ün
  30k'ya kadar arıtılmış eps=1.7 sonucuna göre (cost=1.497221) **sadece
  %0.80 daha pahalı** -- yani search'ün geri kalan ~29,943 expansion'ı
  (57'den 30,000'e) sadece %0.80'lik bir iyileştirme sağlamış, ilk
  çözüm zaten neredeyse aynı kalitede. Path: `outputs/
  stage37_4_first_solution_eps17.csv`. Script: `scripts/
  benchmark_first_solution_eps17.py`. Bu turda YAPMAZ listesi (30k'ya
  devam, certificate arama, epsilon sweep, ARA*, cost tuning, corridor/
  history değişikliği) hiçbiri implement edilmedi. Sonraki adım bu
  turda BAŞLATILMADI, karar kullanıcıda.

- **Stage 38 — Fine ARA* Refinement: GERÇEK bir ARA* implement edildi
  (`planner/astar.py::ara_star_search`) -- epsilon_schedule=(1.7,1.5,1.3,1.1)
  boyunca TEK bir g/parent/OPEN/CLOSED/INCONS/incumbent state korunarak
  (asla ayrı ayrı `astar_search()` çağrıları değil), tek bir CUMULATIVE
  30,000-expansion bütçesiyle çalıştı. Sonuç: ε=1.7 ilk çözüm Stage 37.4
  baseline'ına (57 exp/0.343s/cost=1.509232) neredeyse birebir yakın
  geldi (54 exp/0.374s/cost=1.517405) -- ARA*'nin kendi search mekaniği
  bağımsız doğrulandı. ε=1.5'e geçişte cost 1.483210'a düştü (%2.25 FIRST'e
  göre daha ucuz), MSL 200m aşağı indi (3520->3320) -- ama ε=1.3 fazı
  bütçe tükenmeden TAMAMLANAMADI (30,000/30,000 kullanıldı), ε=1.1'e HİÇ
  geçilemedi.**
  `planner/astar.py`'ye YENİ `ara_star_search()` fonksiyonu + `ARAPhaseResult`/
  `ARASearchResult` dataclass'ları eklendi (mevcut `astar_search`'e HİÇBİR
  değişiklik yapılmadı -- tamamen ayrı, ek bir fonksiyon). Algoritma:
  `_generate_neighbors`'ı (freeze_history=True hardwired, aynı corridor_mask/
  z_guide_grid/z_guide_tolerance_m argümanlarıyla) ve `_heuristic`/
  `_state_in_goal_region`/`_reconstruct_path`/`_path_altitude_metrics`/
  `_path_vertical_reversal_metrics`'i DOĞRUDAN reuse ediyor -- hiçbir
  terrain/AGL/angle/corridor/cost/heuristic mantığı tekrar yazılmadı.
  Goal bir REGION olduğu için klasik ARA*'nin tekil `sgoal` düğümü yerine
  `incumbent_cost`/`incumbent_state` o rolü üstleniyor: goal-region'a giren
  bir successor normal bir relaxation gibi g/parent güncellemesi alıyor,
  ama OPEN/INCONS'a HİÇ itilmiyor (sink -- bölgenin içinden geçmeye gerek
  yok) ve eğer g'si incumbent'i iyileştiriyorsa incumbent güncelleniyor.
  Reopening: aynı faz içinde CLOSED bir state daha iyi g bulursa ANINDA
  reopen edilmiyor, INCONS'a ekleniyor (bu ARA*'yi tekrarlı Weighted A*'dan
  ayıran temel mekanizma). Epsilon düşürüldüğünde: `OPEN = OPEN ∪ INCONS`,
  INCONS temizleniyor, kalan OPEN elemanlarının key'leri yeni epsilon ile
  yeniden hesaplanıyor, CLOSED temizleniyor -- g/parent/incumbent/primitive_
  cache DOKUNULMADAN taşınıyor (hiçbir edge yeniden değerlendirilmiyor).
  ImprovePath'in faz sonlandırma testi: OPEN'ın en düşük weighted key'i
  incumbent_cost'u artık yenemeyecek hale gelince (`f_weighted_top >=
  incumbent_cost`) faz bitiyor -- spesifikasyondaki "OPEN'daki en iyi
  weighted lower key artık incumbent'i iyileştiremeyecek noktaya gelince"
  kriterinin birebir karşılığı. `diagnostic_lower_bound` (OPEN∪INCONS
  üzerinden min(g+h)) her faz sonunda hesaplanıyor AMA açıkça sadece
  DIAGNOSTIC olarak raporlanıyor, certificate DEĞİL -- çünkü önceki bir
  fazda CLOSED edilip bir daha hiç INCONS'a girmemiş bir state'in "çözülmüş"
  sayılması, bu multi-epsilon/region-goal ortamında ayrıca ispatlanmış bir
  admissibility argümanına sahip değil (astar_search'ün tek-epsilon
  bounded_mode certificate'inin aksine) -- yanlış certificate üretmeme
  talimatına bu şekilde uyuldu.
  **SONUÇLAR** (tek run, paylaşılan primitive cache, aynı 3D corridor/
  normalized cost/freeze_history=True):
  ```
  eps   added_exp  cum_exp   time(s)  incumbent    min_MSL  mean_MSL  complete
  1.70         56       56    0.440   1.509232       3520.0    3636.7    True
  1.50      16911    16967   86.208   1.483210       3520.0    3613.9    True
  1.30      13033    30000  124.226   1.483210       3320.0    3512.5   False (bütçe tükendi)
  1.10          --       --       --       --             --        --    (hiç başlamadı)
  ```
  **FIRST (ε=1.7 fazının ilk incumbent'ı, 54 expansion/0.374s) vs FINAL
  (bütçe tükendiğindeki en iyi incumbent, cost=1.483210) independent fine
  DEM replay**: FIRST cost=1.517405, min_MSL=3520.0, mean_MSL=3639.6,
  3d_length=6569.4m, xy_length=6480.0m, climb=540.0, descent=540.0,
  min_AGL=211.52m, max_angle=9.46°, reversal_count=30 (29 kısa-aralıklı --
  Stage 37.4'ün 30/29 bulgusuyla BİREBİR tutarlı). FINAL cost=1.483210,
  min_MSL=3320.0, mean_MSL=3492.6, 3d_length=6803.7m (+234.3m), xy_length=
  6729.4m (+249.4m), climb=440.0, descent=460.0, min_AGL=200.76m, max_angle=
  9.46°, **reversal_count=3 (sadece 2 kısa-aralıklı)** -- roller-coaster
  karakteri YAN ETKİ olarak büyük ölçüde ortadan kalktı (cost'a reversal
  hiç dahil değilken bile, düşük-epsilon search'ün doğal olarak daha az
  zigzag'lı bir rota bulması). **SAFETY: FIRST ve FINAL ikisi de PASS**
  (min_AGL>=200m, max_angle<=10°, 0 fine-DEM violation). **cost improvement
  FIRST->FINAL: +2.25%**. **Küçük gözlemlenen incelik**: eps=1.5 fazı
  sonundaki incumbent (min_MSL=3520, xy=6450.0) ile eps=1.3 fazı sonundaki
  incumbent (min_MSL=3320, xy=6729.4) 6 ondalıkta AYNI cost'u (1.483210)
  gösteriyor ama FARKLI bir fiziksel rota -- muhtemelen 6. ondalığın altında
  gerçek (çok küçük) bir iyileşme incumbent_state'i FARKLI bir goal-region
  girişine kaydırdı; şeffaflık için not edildi, gizlenmedi.
  **9 SORUYA CEVAP**: (1) İlk path yine ~57 exp/sub-second geldi mi? EVET
  (54 exp, 0.374s -- pratikte aynı). (2) ARA* search bilgisini gerçekten
  reuse etti mi? EVET -- tek g/parent/cache/expansion-counter tüm fazlar
  boyunca korundu, hiçbir faz sıfırdan başlamadı. (3) ε düştükçe cost
  anlamlı iyileşti mi? EVET ama azalan getiriyle -- esas iyileşme ε=1.7->1.5
  geçişinde geldi (+2.25%), ε=1.3'ün 13,033 expansion'ı görünür bir ek
  iyileşme getirmedi (bütçe tükendi, ε=1.1'e hiç ulaşılamadı). (4) MSL daha
  aşağı indi mi? EVET (-200m, 3520->3320). (5) XY rota değişti mi? EVET
  (+249.4m, %3.8 daha uzun -- düşük MSL için mesafe feda edildi, w_altitude=
  1.25>w_distance=1.0 ile tutarlı). (6) 30k'nın ne kadarı kullanıldı? TAMAMI
  (30,000/30,000, %100). (7) INCONS mekanizması eski %90+ reopening
  thrashing'ini azalttı mı? Dolaylı kanıt EVET -- INCONS/added_expansions
  oranı ε=1.5 fazında %63, ε=1.3 fazında %36 (Stage 37.3'ün ~%94-96 reopen_
  ratio'suna göre belirgin düşük; doğrudan aynı metrik değil ama analog).
  (8) Final path SAFE mi? EVET (PASS). (9) Refinement süreye değdi mi?
  KISMEN -- ε=1.7->1.5 geçişi (86s, +2.25% cost, reversal 30->3) net bir
  kazanç; ε=1.3'ün kalan ~124s'lik kısmı (13,033 expansion) görünür bir
  ek kazanç getirmeden bütçeyi tükettі -- Stage 37.4'ün "30k'nın geri kalanı
  sadece %0.80 iyileştirdi" bulgusuna göre ARA* bütçeyi ~3x daha iyi kullandı,
  ama mutlak getiri hâlâ mütevazı ve azalan. Path: `outputs/
  stage38_ara_first_path.csv`, `outputs/stage38_ara_final_path.csv`.
  Script: `scripts/benchmark_stage38_ara.py`. Bu turda YAPMAZ listesi (epsilon
  başına search restart, vertical profile optimizer, roller-coaster
  penalty, trend/bucket geri getirme, corridor değiştirme, cost tuning,
  Numba/GPU, heading/turn radius, eski benchmarkları tekrar çalıştırma)
  hiçbiri implement edilmedi. Sonraki adım bu turda BAŞLATILMADI, karar
  kullanıcıda.

- **Stage 38.1 — Fine Corridor Safety Precompute + ARA* Runtime Sweep:
  precompute search DAVRANIŞINI birebir aynı tuttu (aynı 54 exp/cost=
  1.517405 ilk çözüm, aynı eps=1.5/1.3 sonu incumbent=1.483210, aynı
  30,000/30,000 bütçe tüketimi) AMA search-only wall-clock'u ~6.3x
  hızlandırdı (124.23s -> 19.63s) -- SADECE 30k'lık AYNI expansion
  bütçesi içinde eps=1.2'ye hâlâ ULAŞILAMADI (bütçe expansion-count
  bazlı, wall-clock bazlı değil -- precompute expansion SAYISINI
  değiştirmiyor, sadece her expansion'ın maliyetini düşürüyor).**
  `planner/fine_precompute.py` (YENİ, izole modül -- `planner/astar.py`'nin
  ARA*/A* mantığına dokunmuyor, Stage 35.1'in coarse-grid precompute'unun
  fine-grid + corridor-restricted + dense-NumPy-array analoğu):
  `FinePrecomputeResult` (`static_invalid_reason_code`: int8 (H,W,P) --
  0=temiz, 1=out_of_bounds, 2=nodata, 3=corridor dışı/hiç hesaplanmadı;
  `required_start_msl`: float32 (H,W,P)), `precompute_fine_corridor_
  primitive_safety()` (SADECE corridor_mask=True hücreler için doldurur,
  matematik Stage 35.1 ile birebir aynı: `required_start_msl = max_i(
  terrain_elev(i) + min_agl - primitive.dz_m*t_i)`), `fine_precomputed_
  primitive_validity()` (O(1) array lookup + karşılaştırma). z_index
  cache key'e HİÇ girmiyor (spesin talimatı). `planner/astar.py`'ye
  minimal, geriye-dönük-uyumlu değişiklik: `_generate_neighbors`'a
  opsiyonel `fine_precompute=None` parametresi (verildiğinde primitive_
  cache/evaluate_primitive'i tamamen bypass ediyor; None=eski davranış
  bit-exact, `astar_search()` bunu HİÇ kullanmıyor, regresyon riski yok);
  `ara_star_search()`'e aynı isimde opsiyonel parametre eklendi (Stage 38
  ARA*'nin kendisine hiçbir mantık değişikliği yok, sadece safety-check
  yolunu değiştiriyor). Circular-import'tan kaçınmak için `planner.
  fine_precompute`'un fonksiyonu lazy-import edildi (fine_precompute
  modülü zaten `planner.astar`'dan import ettiği için).
  **EQUIVALENCE TESTLER (`scripts/validate_fine_precompute.py`)**: 7 test,
  hepsi ALL PASS, 0 mismatch -- flat/level-climb-descent, ridge, bounds,
  NoData, 30 farklı start-MSL seviyesi, corridor-dışı hücrelerin hep
  invalid döndüğü doğrulaması, VE gerçek Stage 36 corridor mask'ı üzerinde
  4,000 rastgele gerçek-corridor örneği (row/col/primitive/start_msl) --
  old evaluator (evaluate_primitive) vs new evaluator (fine_precomputed_
  primitive_validity) tam eşleşti. Gerçek corridor precompute: entry_
  count=116,472 (corridor_cell_count=4,853 x 24 primitive),
  preprocessing_time=18.26s (test script'inde) / 18.41s (benchmark
  script'inde), approx_memory_MB=12.69, static_invalid_count=0 (corridor
  hücrelerinin hiçbiri bounds/NoData'ya çarpmıyor -- Stage 37.3'ün
  bulgusuyla tutarlı). **ARA* SWEEP (`scripts/
  benchmark_stage38_1_ara_precompute.py`, precompute BİR KEZ oluşturulup
  TEK continuous ARA* run'a geçirildi, epsilon_schedule=(1.7,1.5,1.3,1.2),
  max_expansions_cumulative=30,000 -- Stage 38 ile AYNI, bu turda
  değiştirilmedi)**:
  ```
  eps   added_exp  cum_exp  phase_time  cum_time      cost   cost_gain  min_MSL  mean_MSL
  1.70         56       56      0.155s     0.155s  1.509232        n/a   3520.0    3636.7
  1.50      16911    16967     10.339s    10.494s  1.483210     +1.72%   3520.0    3613.9
  1.30      13033    30000      9.140s    19.634s  1.483210     +0.00%   3320.0    3512.5
  1.20        --         --       --          --       --          --      --        --   (bütçe tükendi, hiç başlamadı)
  ```
  **İLK ÇÖZÜM (ε=1.7)**: expanded=54, runtime=0.1019s, cost=1.517405 --
  Stage 38'in (precompute'suz) AYNI sonucuyla (54 exp, cost=1.517405)
  BİREBİR eşleşti -- precompute'un search KARARLARINI hiç değiştirmediğinin
  doğrudan kanıtı, sadece hızını değiştiriyor. **FINAL** (bütçe tükendiğinde,
  eps=1.3 fazı tamamlanmadan): cost=1.483210 (Stage 38'in final'iyle
  BİREBİR aynı, `%-0.000` fark), min_MSL=3320.0, 3d_length=6803.7m,
  independent fine DEM replay: min_AGL=200.76m, max_angle=9.46°,
  violations=0, **SAFETY: PASS**. Path: `outputs/
  stage38_1_ara_final_path.csv` (FIRST path yazılmadı -- Stage 38'in
  first-path CSV'siyle zaten birebir aynı olacağı için gereksiz tekrar).
  **KARAR SORULARI**: (1) 1.5 ne kadar kazandırdı/kaç saniye? +1.72% cost
  / 10.34s phase time (quality_gain_per_second=0.167) -- Stage 38'deki
  aynı geçişin 86.2s'sine göre ~8.3x daha hızlı ulaşıldı. (2) 1.3 ek olarak
  ne kazandırdı/kaç saniye? +0.00% (görünür iyileşme yok, 6 ondalıkta aynı
  cost) / 9.14s phase time, quality_gain_per_second=0.0000 -- Stage 38'in
  bulgusuyla tutarlı (1.3 fazı burada da tamamlanamadı, ama artık SADECE
  9.14s'de, önceki 124.23s'nin bir kısmı yerine). (3) 1.2 ek olarak ne
  kazandırdı/kaç saniye? CEVAPLANAMAZ -- ε=1.2 fazına HİÇ ulaşılamadı,
  30,000'lik expansion bütçesi ε=1.3'ün ortasında tükendi (precompute
  expansion SAYISINI değiştirmediği için bu, Stage 38 ile MATEMATİKSEL
  OLARAK AYNI nokta). (4) Precompute sonrası daha düşük epsilon'lar
  pratik hale geldi mi? **KISMEN/WALL-CLOCK açısından EVET, ama AYNI
  expansion bütçesiyle HAYIR.** Precompute search-only wall-clock'u
  124.23s'den 19.63s'e düşürdü (~6.3x, +18.41s tek-seferlik precompute
  dahil toplam ~38.04s, yine de ~3.3x uçtan uca hızlanma) -- AMA bütçe
  expansion-SAYISI bazlı olduğu için (wall-clock bazlı değil) aynı 30,000
  cap ile ε=1.2'ye hâlâ erişilemiyor; precompute'un asıl kazancı, DAHA
  YÜKSEK bir expansion bütçesinin artık çok daha ucuza (aynı wall-clock
  süresinde ~6x daha fazla expansion) karşılanabilir olması -- bu spesin
  kapsamı dışında olduğu için bütçe bu turda YÜKSELTİLMEDİ, sadece
  gözlem olarak raporlanıyor. (5) Production stopping point hangi
  epsilon olmalı? **ε=1.5** -- tüm ölçülebilir kalite kazancı (+1.72%
  cost, MSL aynı, reversal 30->12) oradan geliyor ve precompute ile artık
  ~10.5s'de (tek-seferlik ~18.4s precompute dahil ~28.9s) elde ediliyor;
  ε=1.3/1.2'nin bu bütçede EK bir ölçülebilir kazanç sağladığı
  GÖSTERİLEMEDİ (1.3 fazı %0.00 gösterdi, 1.2 hiç denenemedi) -- FAST
  için ε=1.7 (54 exp, <0.2s toplam), BALANCED/QUALITY için ε=1.5 önerilir;
  ε=1.3'e devam etmenin bu ölçümde somut bir gerekçesi yok. Bu turda
  YAPMAZ listesi (ε=1.10, corridor değiştirme, cost/heuristic tuning,
  vertical optimizer, roller-coaster penalty, heading/turn radius,
  Numba/GPU, disk cache, eski PASS benchmarklarını tekrar çalıştırma)
  hiçbiri implement edilmedi. Sonraki adım bu turda BAŞLATILMADI, karar
  kullanıcıda.

- **Stage 38.2 — Altitude Weight Test (`w_altitude=1.50`): HAYIR — tek
  continuous ARA* `epsilon_schedule=(1.70,1.50)` koşusu, eski ε=1.3 düşük-
  altitude avantajını daha erken yakalamadı.** Yalnız normalized altitude
  weight `1.25 -> 1.50` değiştirildi; `w_distance=1.0`, `freeze_history=True`,
  state=`(row,col,z)`, XY corridor ±300m, Z guide ±200m, fine safety precompute,
  30m XY, 20m Z, min AGL=200m, max angle=10°, H_ref=3240, H_scale=1000,
  heuristic, goal tolerance, ARA* ve precompute aynen korundu. ε=1.3/1.2/1.1
  çalıştırılmadı; eski benchmark/PASS testleri tekrar çalıştırılmadı.

  **Tek yeni search run sonucu:** ε=1.70 ilk incumbent'ı expansion=143 /
  0.196s'de buldu (cost=1.631084003); phase-end incumbent son kez cumulative
  expansion=368'de iyileşti. Faz 416 expansion / 0.654s'de tamamlandı.
  ε=1.50 fazında **hiç incumbent improvement olmadı**; 29,584 ek expansion
  ve 19.953s sonra cumulative 30,000 cap'e ulaşıldı, faz tamamlanamadı.
  Dolayısıyla final ε=1.50 accepted incumbent, ε=1.70 phase-end incumbent'ının
  aynısıdır. Precompute=20.143s, search=20.607s, uçtan uca=40.751s.

  ```
  phase | expansions                 | time                         | min MSL | mean MSL | XY length | cost
  1.70  | 416 (cum 416)              | 0.654s (cum 0.654s)          | 3540.0  | 3652.5   | 6462.4m   | 1.629699545
  1.50  | 29584 (cum 30000)          | 19.953s (cum 20.607s)        | 3540.0  | 3652.5   | 6462.4m   | 1.629699545
  ```

  Her iki phase-end accepted incumbent için rota metrikleri aynıdır:
  distance-weighted mean MSL=3652.5m, 3D length=6523.7m, total climb=360m,
  total descent=380m, reversal count=15, min AGL=200.05m, max angle=9.46°,
  violations=0. **Final ε=1.50 fine replay: PASS** (`AGL>=200`, `angle<=10°`,
  violation=0). Yeni objective içindeki cost ayrımı: distance component=
  1.006739302, altitude component=0.622960243, total=1.629699545 (search
  incumbent ile ~1.6e-15 içinde aynı). Bu cost eski `w_altitude=1.25`
  cost'larıyla doğrudan iyi/kötü diye karşılaştırılmadı.

  **Search çalıştırmadan kayıtlı yol replay'i:** Stage 38/38.1 ε=1.5 phase
  path'ini diske kaydetmediği için o yol recompute edilemedi. Kayıtlı eski
  ε=1.3 yolu (`outputs/stage38_1_ara_final_path.csv`) `w_altitude=1.50`
  altında replay edildi: min/mean MSL=3320.0/3512.5m, XY/3D=6729.4/6803.7m,
  climb/descent=440/460m, reversal=3, min AGL=200.76m, max angle=9.46°,
  violations=0; distance component=1.049955334, altitude component=
  0.429137998, total=**1.479093332**. Böylece yeni objective eski düşük-MSL
  yolu yeni run incumbent'ına göre gerçekten tercih ediyor (%9.24 daha düşük
  total); problem objective yönü değil, bu ε schedule/30k budget ile yolun
  accepted incumbent olarak erken yüzeye çıkarılamaması.

  **Kritik cevaplar:** (1) ε=1.7, 3520m altına inmedi; tersine min=3540m.
  (2) ε=1.5 eski ε=1.3'ün ~3320/~3513m davranışına yaklaşmadı; accepted
  incumbent 3540/3652.5m'de kaldı. (3) Düşük irtifa kazanımı olmadığı için
  bunun uğruna rota uzamadı; eski w=1.25 ε=1.7/1.5 phase-end XY=6450m
  referansına göre yalnız +12.4m (+%0.19). (4) Kötüleşme belirgin: ε=1.7
  added expansion 56->416; ε=1.5'e kadarki cumulative expansion
  16,967->30,000 (+%76.8) ve search runtime 10.494->20.607s (+%96.4), üstelik
  ε=1.5 tamamlanmadı. (5) Safety aynen korundu, PASS. (6) `w_altitude=1.50`
  objective olarak düşük yolu doğru sıralasa da **mevcut production stopping
  point ε=1.7/1.5 ile 1.25'ten daha uygun görünmüyor**: daha yüksek MSL,
  tamamlanmayan refinement ve daha yüksek search maliyeti verdi.

  **Raporlama düzeltmesi (search mimarisi değişmedi):** İlk ham çıktı,
  ε=1.50'de incumbent hiç güncellenmediği halde live parent map'ten yeniden
  path kurduğu için search cost=1.629699545 ile path-recomputed cost=
  1.521685628 arasında açık mismatch gösterdi. Neden, incumbent goal sink'in
  g'si tekrar relax edilmeden ancestor parent'larının sonraki fazda
  değişebilmesiydi; bu türetilmiş yol accepted incumbent değildir ve sonuçtan
  çıkarıldı. `ARAPhaseResult` artık yalnız raporlama için incumbent yolu,
  güncellendiği anda immutable snapshot olarak saklıyor; ordering/relaxation/
  stopping/OPEN-CLOSED-INCONS reuse'a dokunulmadı. Arama tekrar çalıştırılmadı.
  Script: `scripts/benchmark_stage38_2_altitude_weight.py`; tam düzeltilmiş
  kayıt: `scratch_stage38_2.log`; accepted yollar: `outputs/
  stage38_2_eps17_path.csv` ve aynı SHA-256'ya sahip `outputs/
  stage38_2_eps15_path.csv`. **Tek karar: HAYIR.** `w_altitude=1.50` ile
  ε=1.7/1.5, eski ε=1.3 düşük-altitude avantajını daha erken yakalamıyor.

- **Stage 38.3 — Mission Policy Generalization + Epsilon Robustness:
  MissionPolicy merkezileştirildi; objective A–K=11/11 PASS, fakat selected
  synthetic search yalnız FLAT'te preferred topology'yi buldu. Ana sonuç:
  OBJECTIVE DOĞRU, SEARCH/GLOBAL XY GUIDANCE DARBOĞAZ.** `planner/mission.py`
  tek normalized soft-objective authority oldu: `MissionPolicy`,
  `CostComponents`, production factory (`w_distance=1.0`, `w_altitude=1.25`,
  scale=1000) ve config adapter. Fine A*/ARA* normalized edge cost ile coarse
  edge cost aynı helper'a geçirildi; aynı level edge için common/fine/coarse
  cost=`0.028125000000`, max delta=`0` — consistency PASS. Coarse planner'daki
  location-specific `altitude_reference_msl=3240` default'u kaldırıldı;
  reference artık caller/config tarafından explicit mission datum olarak
  veriliyor. `PlannerConfig.normalized_w_altitude` production candidate 1.25
  oldu (legacy mode'a etkisiz); production weight bu stage'de değiştirilmedi.
  Safety-only `validate_path_safety()` costing/optimization'dan ayrıldı;
  eski `validate_and_cost_path()` yalnız backward-compatible safety+replay
  convenience wrapper olarak kaldı. Hard safety logic'in kendisi değişmedi.

  **TABLE 1 — Objective Tests (FULL SEARCH YOK; actual winner w_alt=1.25):**

  | scenario | candidate | length m | mean MSL | cost w=1.0 | cost w=1.25 | cost w=1.5 | expected winner | actual winner | result |
  |---|---|---:|---:|---:|---:|---:|---|---|---|
  | A FLAT | short | 4000.0 | 500.0 | 1.100000 | 1.125000 | 1.150000 | short | short | PASS |
  | A FLAT | same-MSL detour | 4123.1 | 500.0 | 1.133854 | 1.159623 | 1.185393 | short | short | PASS |
  | B LOW SLIGHT | high short | 4000.0 | 500.0 | 1.100000 | 1.125000 | 1.150000 | low 8% | low 8% | PASS |
  | B LOW SLIGHT | low 8% detour | 4332.1 | 420.0 | 1.104663 | 1.110076 | 1.115489 | low 8% | low 8% | PASS |
  | C LOW LARGE | high short | 4000.0 | 500.0 | 1.100000 | 1.125000 | 1.150000 | high short | high short | PASS |
  | C LOW LARGE | low 43% detour | 5736.9 | 427.3 | 1.473430 | 1.483233 | 1.493036 | high short | high short | PASS |
  | D RIDGE | ridge overflight | 4000.0 | 600.0 | 1.100000 | 1.125000 | 1.150000 | lower bypass | lower bypass | PASS |
  | D RIDGE | lower bypass | 4332.1 | 520.0 | 1.104663 | 1.110076 | 1.115489 | lower bypass | lower bypass | PASS |
  | E BROAD VALLEY | high cruise | 4000.0 | 500.0 | 1.100000 | 1.125000 | 1.150000 | descend/cruise | descend/cruise | PASS |
  | E BROAD VALLEY | descend/cruise/climb | 4016.6 | 415.1 | 1.019345 | 1.023147 | 1.026948 | descend/cruise | descend/cruise | PASS |
  | F NARROW SIDE | center high | 4000.0 | 500.0 | 1.100000 | 1.125000 | 1.150000 | side low | side low | PASS |
  | F NARROW SIDE | side low (3 cells) | 4025.6 | 417.7 | 1.024226 | 1.028681 | 1.033136 | side low | side low | PASS |
  | G TWO VALLEYS | near/high | 4000.0 | 500.0 | 1.100000 | 1.125000 | 1.150000 | far/deep | far/deep | PASS |
  | G TWO VALLEYS | far/deep | 4332.1 | 420.0 | 1.104663 | 1.110076 | 1.115489 | far/deep | far/deep | PASS |
  | H HIGH-VALLEY-HIGH | stay high | 4000.0 | 500.0 | 1.100000 | 1.125000 | 1.150000 | early descent | early descent | PASS |
  | H HIGH-VALLEY-HIGH | early descent | 4016.6 | 415.1 | 1.019345 | 1.023147 | 1.026948 | early descent | early descent | PASS |
  | I SAFETY OVERRIDE | high safe | 4000.0 | 500.0 | 1.150000 | 1.187500 | 1.225000 | high safe | high safe | PASS |
  | I SAFETY OVERRIDE | lowest unsafe | 4036.9 | 373.0 | 1.032425 | 1.038224 | 1.044022 | high safe | high safe | PASS (unsafe rejected) |
  | J LOW AGL vs MSL | terrain-follow/high MSL | 4000.0 | 600.0 | 1.100000 | 1.125000 | 1.150000 | higher-AGL/low-MSL | higher-AGL/low-MSL | PASS |
  | J LOW AGL vs MSL | higher-AGL/low-MSL | 4332.1 | 520.0 | 1.104663 | 1.110076 | 1.115489 | higher-AGL/low-MSL | higher-AGL/low-MSL | PASS |
  | K DIFFERENT ENDPOINTS | monotone | 4001.2 | 450.0 | 1.050000 | 1.062500 | 1.075000 | monotone | monotone | PASS |
  | K DIFFERENT ENDPOINTS | unneeded climb | 4010.5 | 507.6 | 1.110188 | 1.137158 | 1.164127 | monotone | monotone | PASS |

  Her candidate için CSV'de ayrıca geometric length, distance/altitude
  components, min/max MSL, climb/descent, safety, distance-weighted
  P25/P50/P75 ve reference+50m low-band fraction saklandı. Örnek: B düşük
  rota P25/P50/P75=`400/400/450`, low band=%100; high rota=`500/500/500`,
  low band=%0. Bu nedenle yalnız kısa bir min-MSL dip'i “iyi rota” sayılmadı.
  I senaryosunda unsafe düşük rota cost açısından çok ucuz (`1.038224 <
  1.187500`) olmasına rağmen AGL ihlali yüzünden INVALID; safety supremacy
  doğrudan doğrulandı. J, düşük AGL ile düşük absolute MSL'nin karışmadığını
  gösterdi.

  **Teorik trade-off:** düşük rota altitude reference üzerinde ve high rota
  100m yukarıdayken eşdeğer ekstra mesafe `w_altitude*(100/1000)` olur:
  w=1.0 → %10, w=1.25 → **%12.5**, w=1.5 → %15. Deneysel B (%8.30 detour,
  ~80m mean-MSL gain) w=1.25'te düşük rotayı seçti (`1.110076<1.125000`);
  C (%43.42 detour) yüksek/kısa rotayı seçti (`1.125000<1.483233`). Böylece
  intended “100m lower ≈ %10–15 detour” davranışı hem teorik hem deneysel
  aynı order'da doğrulandı; hard threshold değildir. Not: w=1.0'da B'nin
  high rotayı az farkla seçmesi (`1.100000<1.104663`), sweep'in weight
  seçmek yerine trade-off değişimini gerçekten gösterdiğini doğrular.

  **TABLE 2 — Search Tests** (`w_altitude=1.25` sabit, her scenario tek
  continuous ε=1.70→1.50→1.30 ARA*, max 12k fakat hiçbirinde cap'e
  yaklaşılmadı; expansions/time = phase added, parantez cumulative):

  | scenario | epsilon | expansions | runtime | mean MSL | path length | cost | preferred found? | OBJECTIVE/SEARCH status |
  |---|---:|---:|---:|---:|---:|---:|---|---|
  | A FLAT | 1.70 | 28 (28) | .0317s (.0317) | 300.0 | 840.0 | 1.000000 | yes | OBJECTIVE PASS + SEARCH PASS |
  | A FLAT | 1.50 | 0 (28) | .0063s (.0380) | 300.0 | 840.0 | 1.000000 | yes | OBJECTIVE PASS + SEARCH PASS |
  | A FLAT | 1.30 | 0 (28) | .0067s (.0447) | 300.0 | 840.0 | 1.000000 | yes | OBJECTIVE PASS + SEARCH PASS |
  | F NARROW SIDE | 1.70 | 14 (14) | .0924s (.0924) | 410.0 | 1703.2 | 1.153190 | no | OBJECTIVE PASS + SEARCH FAIL |
  | F NARROW SIDE | 1.50 | 0 (14) | .0293s (.1217) | 410.0 | 1703.2 | 1.153190 | no | OBJECTIVE PASS + SEARCH FAIL |
  | F NARROW SIDE | 1.30 | 0 (14) | .0237s (.1454) | 410.0 | 1703.2 | 1.153190 | no | OBJECTIVE PASS + SEARCH FAIL |
  | G TWO VALLEYS | 1.70 | 15 (15) | .0943s (.0943) | 390.0 | 1703.2 | 1.153190 | no | OBJECTIVE PASS + SEARCH FAIL |
  | G TWO VALLEYS | 1.50 | 0 (15) | .0181s (.1125) | 390.0 | 1703.2 | 1.153190 | no | OBJECTIVE PASS + SEARCH FAIL |
  | G TWO VALLEYS | 1.30 | 0 (15) | .0173s (.1298) | 390.0 | 1703.2 | 1.153190 | no | OBJECTIVE PASS + SEARCH FAIL |
  | H HIGH-VALLEY-HIGH | 1.70 | 49 (49) | .3399s (.3399) | 390.4 | 1777.5 | 1.177648 | no | OBJECTIVE PASS + SEARCH FAIL |
  | H HIGH-VALLEY-HIGH | 1.50 | 0 (49) | .0487s (.3886) | 390.4 | 1777.5 | 1.177648 | no | OBJECTIVE PASS + SEARCH FAIL |
  | H HIGH-VALLEY-HIGH | 1.30 | 34 (83) | .2009s (.5895) | 395.7 | 1703.2 | 1.135087 | no | OBJECTIVE PASS + SEARCH FAIL |

  Tüm search path'leri safety PASS. F/G'de ε=1.7 goal-directed high route'a
  çok hızlı bağlandı; ε=1.5/1.3 ImprovePath başlangıcında active key
  incumbent'i yenemediği için 0 ek expansion ve 0 INCONS ile aynı topology
  kaldı. H'de ε=1.3 34 ek expansion yaptı, INCONS=4 ve cost'u %3.61 düşürdü;
  ama mean MSL `390.4→395.7` yükselirken length `1777.5→1703.2` düştü:
  yeni düşük-MSL topology değil, distance refinement. ε=1.5 hiçbir selected
  scenario'da meaningful improvement getirmedi. Bu sonuç “daha düşük epsilon
  her zaman daha iyi” iddiasını çürütür; aynı zamanda mevcut ARA* phase
  stopping/reuse davranışının lateral topology keşfi için yeterli olmadığını
  gösterir. First-improvement expansions: A=28, F=14, G=15, H=49; yalnız H
  ε=1.3'te yeni incumbent expansion=83. Ayrıntılı distance/altitude component,
  climb/descent, safety, g-improvement ve INCONS alanları CSV'dedir.

  **TABLE 3 — Epsilon Summary:**

  | epsilon | typical strength | typical weakness | scenarios where useful | search cost |
  |---:|---|---|---|---|
  | 1.70 | Çok hızlı first path; düz/direct topology | Greedy goal direction; lateral/deep valley kaçırıyor | A; F/G/H'de yalnız hızlı fallback | 14–49 exp, .032–.340s phase |
  | 1.50 | Bu suite'te ek kalite yok | Inherited incumbent nedeniyle frontier hemen durabiliyor | Ölçülen scenario yok | 0 ek exp; .006–.049s rekey/diagnostic |
  | 1.30 | H'de daha kısa/cost-lower refinement | F/G'de yeni topology yok; altitude quality artmadı | H distance refinement | 0–34 ek exp; en çok .201s; INCONS yalnız H=4 |

  **Aladağlar replay-only (`w_altitude=1.25`, YENİ SEARCH YOK):** direct
  level: D/A/total=`1.000000/0.650000/1.650000`, length=6480m,
  min/mean=3760/3760m, climb/descent=0/0; ε=1.7 fast:
  `1.013794/0.503611/1.517405`, length=6569.4m, min/mean=3520/3637.4m,
  climb/descent=540/540; kayıtlı eski ε=1.3 low:
  `1.049955/0.357615/1.407570`, length=6803.7m, min/mean=3320/3512.5m,
  climb/descent=440/460. Üçü de replay safety PASS. ε=1.5 phase path eski
  stage'lerde persist edilmediği için mevcut değil. Stage 38.2 referansı
  aynen destekleniyor: w=1.5 objective eski low rotayı tercih etmişti ama
  search erken bulamamıştı — cost preference != discoverability.

  **15 final cevap:** (1) EVET, normalized mission semantics merkezi
  `MissionPolicy` oldu. (2) EVET, coarse/fine/ARA* için low MSL yalnız aircraft
  absolute MSL. (3) EVET, safety-only validation objective'den ayrı ve
  daima önce. (4) Objective A–K 11/11 PASS; selected search A PASS, F/G/H
  FAIL. (5) EVET, w=1.25 low-MSL slightly>distance davranışını B/D/E/F/G/H/J
  tekrar etti; C excessive detour'u reddetti. (6) EVET, teorik %12.5 ve
  deneysel %8.3-wins/%43.4-loses aynı order'da. (7) Cost tüm A–K'de doğru;
  I unsafe adayı filtreledi. (8) Search özellikle lateral F ve multi-valley G,
  ayrıca broad vertical opportunity H'de zorlandı. (9) ε=1.7 flat/directte
  güçlü ve hızlı, lateral alternatives'ta fazla greedy. (10) ε=1.5 bu küçük
  suite'te gerçek gain getirmedi; bu evrensel bir sabit kararı değildir.
  (11) ε=1.3 hiçbir testte yeni low-MSL topology bulmadı; yalnız H'de daha
  kısa rota/cost refinement verdi, F/G'de gereksizdi. (12) EVET: F/G/H ve
  Stage 38.2 açık OBJECTIVE PASS + SEARCH FAIL örnekleri. (13) EVET, ortak
  helper aynı edge'de numeric exact; farklı resolution approximation ileride
  ranking ayrışması yaratabilir ama bu suite'te gözlenmedi. (14) Cost tarafında
  flat/ridge/broad/narrow/two-valley/safety/endpoint çeşitliliği ve 11/11
  tekrar overfit işareti vermiyor; yine de B/D/G/J benzer candidate geometrisi
  kullandığı için tamamen bağımsız saha kanıtı değildir. Epsilon/corridor/
  lateral width/Z tube production constant olarak kilitlenmedi, config/adaptive
  kalmalı. (15) Sonraki ana darboğaz **heuristic/search guidance + global XY
  exploration**; mission cost değil. Local refinement ve vertical smoothing
  ayrı sonraki problemler, ama preferred topology bulunmadan ana engel değiller.

  Script: `scripts/validate_stage38_3_mission_generalization.py`. Ham tablolar:
  `outputs/stage38_3_objective_tests.csv`, `outputs/stage38_3_search_tests.csv`.
  Eski pahalı benchmark/PASS testleri çalıştırılmadı; production parameter
  değiştirilmedi; başka optimizer/tuning yapılmadı. Bu stage burada DURDU.

- **Stage 38.4 — Independent Search-Guidance Review + Controlled Experiment:
  TEŞHİS KISMEN DOĞRULANDI (no ARA* bug; heuristic distance-only), ÖNERİLEN
  ÇÖZÜM (terrain+min_agl XY guidance + heap reordering, test edilen haliyle)
  REDDEDİLDİ — F/G/H'de preferred topology'yi bulmadı, çoğu ayarda
  baseline'dan daha kötü sonuç verdi.** `planner/astar.py` bu stage'de HİÇ
  değiştirilmedi (checksum ile doğrulandı); tüm deney bağımsız bir prototip
  script'te (`scripts/prototype_stage38_4_guidance.py`, diagnostic-only,
  production'a dahil değil) yapıldı.

  **(1) Epsilon bound bağımsız doğrulama — CONFIRMED, bug yok.** F/G/H için
  aynı sentetik gridlerde `epsilon_schedule=(1.0,)` (=admissible+consistent
  heuristic için provably optimal düz A*) ground truth olarak kullanıldı.
  Heuristic'in normalized modda (`h = w_distance*D3D/D_ref`) hem admissible
  hem CONSISTENT olduğu, triangle-inequality argümanıyla ayrıca elle
  ispatlandı (tek fazlı eps=1.0 koşusunun INCONS'a hiç dokunmadan optimal
  verdiğini garanti eder). Sonuç:

  | scenario | eps=1.0 true optimal | eps=1.7/1.5 found | eps=1.3 found | found/optimal (1.3) | bound (<=1.3) |
  |---|---:|---:|---:|---:|---|
  | F NARROW SIDE VALLEY | 1.097325 (mean MSL 344.6) | 1.153190 | 1.153190 | 1.051 | OK |
  | G TWO VALLEYS | 1.129027 (mean MSL 381.4) | 1.153190 | 1.153190 | 1.021 | OK |
  | H HIGH-VALLEY-HIGH | 1.094188 (mean MSL 368.5) | 1.177648 | 1.135087 | 1.037 | OK |

  Hiçbir epsilon'da bound ihlali yok; ORAN bound'un belirgin altında (1.02–
  1.05), yani darboğaz "epsilon'u biraz daha sıkmak" değil. `astar.py`
  1985-2064 satırlarındaki faz döngüsü (heap her epsilon değişiminde
  `open_members|incons_members`'tan sıfırdan kuruluyor, termination
  `f_w_top>=incumbent_cost` klasik Key(sgoal) kriterine bire bir uyuyor,
  closed-ama-relax-edilen node'lar aynı fazda reopen edilmeyip INCONS'a
  düşüyor) kod okumasıyla da doğru bulundu — implementasyon riski YOK.
  `+0 expansion` OPEN boşaldığı için değil (F/G/H'de faz sonunda sırasıyla
  247/186/503-660 state hâlâ OPEN'da) "kalan hiçbir aday incumbent'ı
  yenemeyeceği kanıtlandığı an" duruyor — beklenen ARA* davranışı.
  `use_dominance_pruning`/bucket-dominance mekanizması `ara_star_search`
  içinde HİÇ kullanılmıyor (yalnız eski tek-epsilon `astar_search`'te var) —
  branching/dominance alternatif açıklaması elendi. corridor_mask/z_guide_grid
  bu testlerde `None` (kullanılmadı), goal_tolerance=0 (etkisiz),
  vertical-reachability heuristic normalized modda hiç çağrılmıyor (legacy
  moda özel) — hepsi elendi. `freeze_history=True` hardwired olduğu için
  reversal cost bu testlerde her zaman 0.

  **(2) Yeni yapısal bulgu (önceden belgelenmemiş): primitive/discretization
  friction.** F'nin gerçek optimal path'i (eps=1.0, 27 state) çıkarıldı: row=9
  boyunca ilerliyor, ama 400→300 MSL inişi 5 primitive×4 kolon = 20 kolon,
  300→400 çıkışı yine 20 kolon sürüyor; toplam ~56 kolonluk rotanın yalnız
  ~12 kolonu gerçek vadi tabanında (300 MSL) geçiyor. Bunun sebebi
  `max_climb/descent_angle_deg=10°` (config.py'de TEST PARAMETER olarak
  işaretli, gerçek gereksinim değil) + `z_step_m=20` kombinasyonunun her tek
  20m'lik irtifa adımı için ~4 grid hücresi (113-127m) zorunlu yatay mesafe
  dayatması. Vadi bandı yalnız 3 hücre genişliğinde. Bu, F/G/H'de gerçek
  kazancın neden mütevazı kaldığının (%2-8) ek ve doğrulanmış bir yapısal
  sebebi — heuristic körlüğüne ek, onun yerine değil.

  **(3) ANCHOR+GUIDANCE deneyi — REJECTED (bu haliyle).** Guidance sinyali:
  goal'dan reverse-Dijkstra, 2D XY-only, edge cost = mission'ın normalized
  formülüyle aynı ama `mean_msl` yerine `terrain+min_agl_m` ("achievable min
  MSL") kullanıyor. Sinyal YÖNÜ doğrulandı (F'de vadi hücreleri —terrain=100—
  ölçülebilir şekilde düşük/cazip skor alıyor). Entegrasyon: mevcut tek-heap
  yapısına dokunmadan, PROVEN-SAFE bir "bucketed anchor key" tasarımı
  (`bucket=floor(raw_key/W)*W`; heap `(bucket, guidance, counter, state,
  raw_key)`; termination `bucket_top>=incumbent_cost` — bucket, raw_key için
  geçerli bir ALT SINIR olduğundan termination asla ERKEN tetiklenmiyor,
  yalnız W kadar gecikebiliyor — production `astar.py` değişmedi). Bucket
  genişliği W∈{0.0005,...,0.05} tarandı:

  | W | F final cost | F mean MSL | preferred bulundu mu? | final cum. expansions |
  |---:|---:|---:|---|---:|
  | 0 (baseline, guidance OFF) | 1.153190 | 410.0 | Hayır | 14 |
  | 0.0005–0.002 | 1.153190 | 410.0 | Hayır | 14 |
  | 0.005 | 1.178535 | 430.0 | Hayır | 14 |
  | 0.01–0.02 | 1.196–1.200 | 445–447 | Hayır | 17–25 |
  | 0.05 | 1.199853 | 448.7 | Hayır | 284 |

  Hiçbir W'de F/G/H preferred topology'yi (mean_MSL eşiği Stage 38.3 ile
  aynı) bulmadı; W büyüdükçe sonuç baseline'dan DAHA KÖTÜ (cost ve mean MSL
  ikisi de yükseliyor), expansion sayısı arttı ama kaliteye dönüşmedi. Kök
  neden: guidance yalnız (row,col) — z-blind — oysa gerçek friction (2) tam
  olarak z-bağımlı (kaç primitive'le, hangi kolon aralığında inip
  çıkabileceğin). Z-blind sinyal, arama sırasını irtifa geçişinin gerçek
  maliyetini görmeyen bir yöne çekiyor. Bu, Stage 38.4 promptunda önceden
  sorulan "narrow valley yanlış cazip görünebilir mi / climb-descent
  reachability bunu bozabilir mi" risklerinin varsayımsal değil DENEYSEL
  doğrulanmış hali.

  **Karar:** MHA*/Focal ailesi (anchor+focal, tie-break, MHA*) teorik olarak
  bound-safe ama guidance sinyali reachability/z-tutarlılığı olmadan
  düzeltilmeden hiçbiri işe yaramaz — sırf literatürde tanıdık olduğu için
  seçilmedi, aksine üçü de bu haliyle reddedildi. Sonraki adım (bu stage'de
  YAPILMADI): guidance'ı ya coarse_astar.py'nin gerçek 3D-tutarlı edge
  cost'una reverse-Dijkstra ile bağlamak, ya da heap reordering yerine
  incumbent warm-start (cheap coarse path → verified seed) mimarisine
  geçmek — ikisi de bound/termination mantığına hiç dokunmuyor. Ayrıca F/G/H
  ölçeğinde gerçek kazanç zaten %2-8 (bkz. madde 1); bu karmaşıklığın
  gerçek Aladağlar ölçeğinde daha büyük bir kazanca karşılık gelip
  gelmeyeceği hâlâ açık soru.

  `validate_epsilon_bound_control.py` regression: gerekmedi (production
  search kodu değişmedi, checksum doğrulandı), önceki "no violation" sonucu
  hâlâ geçerli. Weight tuning, epsilon=1.0 production, expansion cap
  büyütme, macro primitives, 30°/45° açı değişikliği, vertical/local
  optimizer, corridor tuning, heading/turn radius, hard low-MSL pruning —
  hiçbiri bu stage'de yapılmadı (yalnız tartışıldı). Script:
  `scripts/prototype_stage38_4_guidance.py` (diagnostic-only). Bu stage
  burada DURDU.

- **Stage 38.5 — Search vs Vertical-Envelope Isolation + Adaptive
  Max-Feasible Vertical Primitive Study: CASE 1 (SEARCH DOMINANT)
  DOĞRULANDI — F-WIDE'da bolca dikey geçiş alanı olsa bile ARA* preferred
  topolojiyi hiç bulmuyor; sabit-zarf 10°→45° sweep'inde 12/12 hücrede
  `first_pref_exp=None`.** `planner/astar.py`/`primitives.py` bu stage'de
  HİÇ değiştirilmedi (checksum `8e1dd037...` doğrulandı); tüm deneyler
  `scripts/stage38_5_lib.py` + 4 ayrı prototip script'te, üretim
  `ara_star_search`'e bit-exact eşleşen bağımsız bir `ara_star_generic`
  sarmalayıcı üzerinden yapıldı (self-check: F/G/H'de production ile
  cost/expansion dizisi birebir aynı).

  **(1) Primitive geometry — configured açı ≠ realized açı, doğrulandı.**
  `build_primitive_set()`'ten gerçek primitive'ler çıkarıldı:

  | configured max açı | axial climb/descent (horiz, n_cells, realized) | diagonal climb/descent (horiz, n_cells, realized) |
  |---:|---|---|
  | 10° | 120.0m, 4, **9.46°** | 127.28m, 3, **8.93°** |
  | 20° | 60.0m, 2, **18.43°** | 84.85m, 2, **13.26°** |
  | 30° | 60.0m, 2, **18.43°** (20°'yle AYNI) | 42.43m, 1, **25.24°** |
  | 45° | 30.0m, 1, **33.69°** | 42.43m, 1, **25.24°** (30°'yle AYNI) |

  Her configured açıda toplam **24 primitive** (üretim değişmedi); dz her
  zaman tek bir `z_step_m=20m`, çok-adımlı "macro" primitive üretim kodunda
  yok. Analitik kontrol (`atan(20/30)=33.69°`, `atan(20/42.43)=25.24°`)
  koddan çıkan sayılarla birebir eşleşti — configured açı yalnızca "bu
  n_cells'i seç" eşiğidir, gerçekleşen flight-path açısı grid-kotalı
  (`n_cells` tam sayı) olduğundan configured değerin altında kalıyor ve
  20°/30° (axial) ile 30°/45° (diagonal) birbirinden AYRIŞMIYOR.

  **(2) Experiment A — F-WIDE (valley'nin longitudinal alanı bollaştırıldı,
  10° sabit): search hâlâ bulmuyor, graph optimum hâlâ buluyor.** Yeni
  senaryo (grid 17×128, 80 ekstra cruise kolonu + 2×20 kolonluk zorunlu
  descent/climb rampası, aynı terrain seviyeleri/MissionPolicy/grid/z_step/
  10°): eps=1.0 graph optimum `cost=1.045800`, `mean_MSL=321.4` (preferred
  eşiği 375'in belirgin altında, **preferred=True**, 90 state'in 76'sı
  vadi tabanında). **ARA* 1.7→1.5→1.3 hiçbirinde preferred'ı bulmadı**
  (`mean_MSL≈409-410`, `found/optimal=1.10-1.12`) — bu oran orijinal dar
  F'nin (1.05) bile ÜSTÜNDE, yani bol dikey geçiş alanı arama kalitesini
  İYİLEŞTİRMEDİ. Bu, darboğazı vertical envelope'tan search/guidance'a
  izole eden doğrudan kanıt.

  **(3) Experiment B — sabit zarf sweep (10°/20°/30°/45° × F/G/H, terrain/
  MissionPolicy/grid/z_step/corridor/goal-tolerance/cost/heuristic sabit):
  first_pref_exp=None → 12/12 hücrede.** ARA* hiçbir açıda hiçbir
  senaryoda preferred topolojiyi bulmadı. Ayrıca beklenmedik ek bulgu: **G
  senaryosunda graph optimum'un kendisi 10°/20°'de preferred DEĞİL**
  (mean_MSL=381.4/380.8, eşik 365'in üstünde), ama **30°/45°'te preferred
  OLUYOR** (mean_MSL=298.9/296.9) — yani G özelinde vertical envelope
  gerçekten graph-optimal çözümün kendisini değiştiriyor (F/H'de graph
  optimum her açıda zaten preferred). Bu nüans G için kısmi CASE 3 sinyali,
  ama ARA* G'de 30°/45°'te bile preferred'ı YİNE bulamadığından (final
  cost=1.150/1.197, preferred=False), arama darboğazı G'de de baskın
  kalıyor.

  **(4) Section 4B/5 — adaptive max-feasible primitive vs fixed, aynı açı:
  bu temiz sentetik terrainlerde İKİSİ AYNI sonucu veriyor (9/9
  karşılaştırma).** Adaptive generator'ın steepest (n_min) adayı bu
  terrainlerde her zaman zaten terrain-safe olduğundan hiçbir yönde
  shallower'a geri çekilme tetiklenmedi; `avg_valid_successor`,
  `final_exp`, `final_cost`, `mean_MSL` fixed ile birebir aynı. Tek fark:
  adaptive `avg_generated` (denenen aday sayısı) fixed'in 24'ünden düşük
  (ör. F@30°: 20.0 vs 24.0) — adaptive aynı geçerli successor sayısına
  (branching değişmeden) daha az `evaluate_primitive` denemesiyle ulaşıyor.
  **Branching açıyla monoton ARTMIYOR** — F@45°'te avg_valid=16.0, F@10°/
  30°'ün 20.0'ından DÜŞÜK (dik açı max/min MSL sınırına daha az hopta
  çarpıyor, bazı yönleri erken kapatıyor) — kullanıcının "45° all-discrete
  daha yüksek branching'e sahip olabilir" hipotezi bu ölçülen veride
  DOĞRULANMADI (tersi yönde: branching düştü).

  **(5) Section 6 — adaptive fallback deneyi (yapay spike/trap terrain):
  geri-çekilme mekanizması ve greedy-olmama davranışı ikisi de doğrulandı.**
  TEST 1 (spike=193, max=45°): denemeler 33.69°→18.43°→12.53°→9.46°
  (geçerli) — doğru shallower'a geri çekildi; spike=197/198'de 8 denemenin
  hepsi invalid → `None` (crash yok, unsafe seçim yok). TEST 2 (max-feasible
  descent mevcut ama hemen ardından duvar/costly-climb var): optimal search
  path'i tamamen LEVEL satırda kaldı, tuzak descent'e hiç girmedi — adaptive
  generator'ın max-feasible'i yalnızca bir ADAY olarak sunduğu, A*'ın seçim
  hakkını koruduğu doğrudan gösterildi.

  **(6) Section 9 (climb/descent asimetrisi) ÇALIŞTIRILMADI — kullanıcının
  kendi koşuluna göre.** Talimat açıkça "yalnızca simetrik sweep gerçek
  vertical-envelope duyarlılığı gösterirse" asimetri testine geç diyordu;
  simetrik sweep (madde 3) F/H'de graph-optimum'u hiç değiştirmedi ve
  ARA*'ı hiçbir açıda preferred'a getirmedi (G'de graph-optimum değişti
  ama ARA* yine bulamadı) — yani baskın sinyal search tarafında, vertical
  envelope tarafında değil. Bu koşul karşılanmadığı için asimetri testi
  atlandı; production karar için de hiçbir zaman kullanılmayacak (Section
  13/14 gereği zaten TEST PARAMETER kapsamında).

  **CASE kararı: CASE 1 (SEARCH DOMINANT), G'de ek bir kısmi CASE 3 nüansı
  ile.** F ve H'de vertical envelope (10°→45°) graph optimum'un preferred
  olma durumunu hiç değiştirmiyor — yalnız arama bunu bulamıyor. G'de
  envelope graph optimum'u da etkiliyor ama arama orada da başarısız
  kalıyor. Sonuç: birincil ve evrensel darboğaz search/guidance katmanı;
  vertical primitive/envelope değişiklikleri (fixed veya adaptive, 10-45°)
  bu üç sentetik senaryoda ARA*'ın erken keşif başarısızlığını TEK BAŞINA
  düzeltmiyor. Bir sonraki mantıklı yön, z-farkındalıklı/kaba 3D guidance
  veya coarse-to-fine warm-start mimarisi (Stage 38.4'ün REDDEDİLEN
  z-blind XY guidance'ı DEĞİL) — adaptive vertical primitive kendi başına
  yeterli değil, ama irtifa-geçiş verimliliği sorununu (Stage 38.4 madde 2)
  ayrı ve doğru şekilde çözüyor.

  Güvenlik: bu stage'deki her run (24+36 fixed-sweep satırı, 15×3
  adaptive-comparison fazı, F-WIDE 1+3 fazı, fallback testleri) bağımsız
  replay ile AGL≥200m/açı-limiti/bounds/NoData kontrolünden geçti — **0
  istisna**. Üretim `max_climb/descent_angle_deg` default'u (10°) hiç
  değiştirilmedi; 20°/30°/45° yalnızca diagnostic duyarlılık değeri olarak
  kullanıldı, safety relaxation olarak yorumlanmadı. Weight tuning,
  MissionPolicy değişikliği, yeni guidance implementasyonu, MHA*/Focal
  production entegrasyonu, corridor tuning, expansion cap artırımı,
  vertical/local optimizer, macro primitive implementasyonu, heading/turn
  radius, tam Aladağlar koşusu — hiçbiri bu stage'de yapılmadı. Scriptler:
  `scripts/stage38_5_lib.py`, `scripts/prototype_stage38_5_fixed_envelope_
  sweep.py`, `scripts/prototype_stage38_5_fwide_isolation.py`,
  `scripts/prototype_stage38_5_adaptive_comparison.py`,
  `scripts/prototype_stage38_5_adaptive_fallback_test.py` (hepsi
  diagnostic-only). Bu stage burada DURDU.

## Ortam

Python 3.14, rasterio 1.5.1, numpy 2.5.2, pyproj 3.8.0, shapely 2.1.2.
Proje artık git repo (main branch).
