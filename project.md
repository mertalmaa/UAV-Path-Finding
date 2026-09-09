# UAV Pathfinder — Proje Notu

Terrain-aware fixed-wing UAV path planner. Amaç: DEM (Digital Elevation Model)
rasterından, arazi çarpışmasından kaçınan bir C-Space/pathfinding hattı
üretmek. Bu doküman mimari kararları ve o kararların gerekçelerini kayıt
altına alır; implementasyon durumu ilerledikçe güncellenir.

Referans doküman: `İHA Arazi Uyumlu Planlama.pdf` — genel mimari/araştırma
yol haritası. PDF bir roadmap'tir, bu dosyadaki kararlar PDF'nin tamamını
implement etme taahhüdü değildir; her aşama kendi promptunun scope'unda
kalır.

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

Henüz implement edilmedi: binary mask, polygonization, buffer/C-Space,
climb/descent için ayrı reversal weight, preferred-AGL/terrain-following
mode, heading/turn radius/Dubins, 2D Dijkstra heuristic,
EDT/buffer, global planner, görselleştirme, bağımsız final validator,
ARA*/decreasing-epsilon anytime search, weighted A*'ın thrashing/reopen-hacmi sorununun kök nedeni (ölçülmedi), low-MSL tie-break, msl_cost_weight default kararı,
backward (goal-side) vertical-reachability envelope, uzun-rota + yüksek-
w_MSL search explosion çözümü (terrain-aware/Dijkstra/wavefront/corridor
adayları not edildi, henüz implement edilmedi), variable-angle primitive
testi (bu problem çözülmeden yapılmayacak).

## Ortam

Python 3.14, rasterio 1.5.1, numpy 2.5.2, pyproj 3.8.0, shapely 2.1.2.
Proje henüz git repo değil.
