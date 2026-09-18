# UAV Pathfinder

Sabit kanatlı İHA için araziye duyarlı 3B yol planlama araştırma kodu.
Planlayıcı, 30 m çözünürlüklü çalışma DEM'leri üzerinde 30×30 km ve 90×90 km
ROI'lerde tek seferde (ara nokta olmadan) başlangıç→hedef rotası üretir.
Dört çalışma bölgesi hazırdır: **Bilecik, Muğla, Ankara, Aladağlar**
(bkz. [`regions/README.md`](regions/README.md)). Referans bölge Bilecik'tir.

Planlayıcıya iki arayüz vardır: komut satırındaki görev scriptleri (`scripts/`)
ve tarayıcıda çalışan **Mission UI** (`mission_ui/`, bkz.
[`mission_ui/README.md`](mission_ui/README.md)).

Son güncelleme: 2026-09-18.

## Hızlı başlangıç (Windows)

`UAV_Pathfinder.bat` dosyasına çift tıklayın. Menüden kurulum, Mission UI,
örnek görevler, testler ve sistem kontrolü çalıştırılabilir; betik sanal ortamı
ve bağımlılıkları da kurabilir. Ayrıntı: [`scripts/launcher/README.md`](scripts/launcher/README.md).

Elle kurulum:

```powershell
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt

python -m pytest -q tests                      # planlayıcı testleri
python -m mission_ui.server --warm bilecik     # http://127.0.0.1:8765
```

## Hat (pipeline)

1. **Pose-aware A\*** (`planner.pose_search.pose_aware_astar_search`):
   sürekli `(x, y, z, heading)` pozlarını 60 m düz, 15° seviye/tırmanan/alçalan
   dönüş ve düz tırmanış/alçalış ilkelleriyle ilerletir. Her ilkel sürekli
   arazi/AGL ve yanal tampon denetiminden geçer.
2. **Vadi-bağıl arazi rehberi** (`_TerrainGuidance`, varsayılan açık):
   hedeften geriye 90 m ızgarada Dijkstra. Maliyet, noktanın 5 km çevresindeki
   en alçak zemine göre yüksekliğidir (HAND); bu yüzden harita boyutundan
   bağımsız olarak vadileri tercih eder. Aynı maliyet A\*'ın g-maliyetine de
   girer. İki kuyruklu (rehberli + anchor) round-robin arama.
3. **Arazi takibi irtifa profili**
   (`planner.terrain_following.optimize_terrain_following_altitudes` /
   `plan_terrain_following`): bulunan yatay iz üzerinde ±5 m/s sınırlarıyla en
   alçak güvenli irtifa profili; profil başarısız olursa yatay aramaya geri
   besleme.
4. **Koridor güvenli yerel B-spline yumuşatma**
   (`planner.local_trajectory_smoothing`).

İrtifayı A\* değil 3. aşama belirler. A\* içinde z'ye bağlı AGL maliyeti
varsayılan olarak kapalıdır (açıldığında 90 km'de z kovalarını patlatıyordu).

## Uçak modeli (`planner.fixed_wing_envelope`)

- Yatay hız 40.0 m/s, sıfır rüzgâr
- Tırmanış/alçalış ±5.0 m/s (tüm irtifalarda)
- Yatış 25° → dönüş yarıçapı ≈ 349.89 m, dönüş hızı ≈ 6.55°/s

## Önemli varsayılanlar (`planner/config.py`)

| Alan | Değer | Not |
|---|---|---|
| `enable_terrain_guidance` | `True` | |
| `guidance_cost_mode` | `"valley_relative"` | eski davranış: `"absolute_quadratic"` |
| `valley_window_m` / `valley_height_scale_m` | 5000 / 300 | |
| `valley_cost_alpha` / `valley_cost_cap` | 2.0 / 3.0 | büyük alpha → daha alçak ama daha uzun rota |
| `guidance_edge_margin_m` | 0 | ROI kenarına yakın vadileri öldürmemek için |
| `guidance_multiplier_in_g` | `True` | g ve rehber h aynı birimde |
| `search_heuristic_weight` | 1.3 | |
| `guidance_queue_ratio` | 3 | |
| `enable_low_altitude_cost` | `False` | |
| `min_agl_m` | 200 | görev scriptleri ve Mission UI 100 m kullanır |
| `lateral_buffer_m` | 0 | görev scriptleri ve Mission UI 60 m kullanır |
| `roi_size_m` | 30 000 | 90 km görevleri bunu `90_000` ile değiştirir |
| `working_dem_path` | `regions/bilecik/working_dem.tif` | bölge değiştirmek için `dataclasses.replace` |

Büyük rehber ızgaraları (≥40 000 hücre) scipy ile çözülür; 90 km ROI'de
rehber kurulumu ≈1.5 s.

## Mission UI (`mission_ui/`)

Planlayıcı kodunu değiştirmeyen, izole bir web arayüzü. Başlangıç/hedef
noktaları haritadan seçilir, gerçek hat arka plan işi olarak çalışır, rota 2B
ve 3B haritada ve irtifa grafiğinde incelenir. Node veya derleme adımı yoktur;
MapLibre GL JS, uPlot ve Fira fontları `mission_ui/web/vendor/` altında
gömülüdür, arayüz çevrimdışı çalışır.

```powershell
python -m mission_ui.server --warm bilecik     # http://127.0.0.1:8765
python -m pytest -q mission_ui/tests           # arayüz/arazi sözleşme testleri
```

Haritadaki arazi (gölgelendirme ve 3B kabartma) yalnızca **görsel**dir ve
`copernicus_glo30_turkey/` karolarını (~5 GB) gerektirmez: bu klasör yoksa sunucu
`mission_ui/.cache/terrain/` içindeki önbelleğe alınmış ulusal mozaikten (~65 MB)
çizer. Planlama, AGL ve irtifa profili her durumda `regions/<id>/working_dem.tif`
üzerinden hesaplanır. Ayrıntı ve paketleme:
[`mission_ui/README.md`](mission_ui/README.md) — "Running without the Copernicus sources".

API, LOD, irtifa semantiği ve mühendislik modu için de aynı dosyaya bakın.

## Bölgeler

| Bölge | Merkez (lon, lat) | ROI irtifa aralığı | Medyan | Karakter |
|---|---|---|---|---|
| `bilecik` | 30.30°E, 40.25°N | 35 – 1683 m | 757 m | Sakarya vadisi ve kanyonları (referans bölge) |
| `mugla` | 28.35°E, 37.22°N | -1 – 2292 m | 606 m | Deniz seviyesinden platoya tırmanış |
| `ankara` | 32.25°E, 40.25°N | 467 – 2389 m | 1099 m | Kirmir kanyonu, tepe aşma |
| `aladaglar` | 35.15°E, 37.81°N | 153 – 3700 m | 1481 m | Yüksek irtifa dağ aşma |

Her bölge 90×90 km ROI + 5 km tampon (100×100 km çalışma DEM'i), 30 m piksel,
EPSG:32636. Üretim hattı ve dosya sözleşmesi: [`regions/README.md`](regions/README.md).

## Çalıştırma

```powershell
python -B scripts/run_single_shot_31km.py                      # Bilecik 31 km kanyon
python -B scripts/run_bilecik_90km_5_missions.py               # 5 adet ~90 km görev
python -B scripts/run_bilecik_90km_5_missions.py --mission M90_02
python -m pytest -q tests                                      # 86 birim test
python -m pytest -q mission_ui/tests                           # 6 arayüz sözleşme testi
```

Diğer scriptler: `run_5_long_30km_single_shot.py`, `run_bilecik_30_tests.py`,
`run_bilecik_5_local_spline_benchmark.py` (+ `create_5mission_summary_atlas.py`),
`run_mavi_mavi_test.py`; veri hazırlığı: `build_working_dem.py`,
`build_regional_dems.py`. Bu scriptler yeni varsayılanlarla yeniden
çalıştırılmadı.

`build_regional_dems.py` ve `build_working_dem.py`, depoda bulunmayan
`copernicus_glo30_turkey/` kaynak karolarını (~5 GB) gerektirir; hazır
`regions/` verisiyle çalışmak için bunlara ihtiyaç yoktur.

## Güncel sonuçlar

Aşağıdaki sayılar `results/test_bilecik/` altındaki JSON dosyalarından okunmuştur;
her tablo kaynağını gösterir.

### 90 km × 5 görev — güncel varsayılanlar

`bilecik_90km_5_missions_valley_benchmark.json` (5/5 başarılı,
vadi-bağıl rehber, A\*'da AGL maliyeti kapalı, ağırlık 1.3):

| Görev | Arama | Toplam planlama | Rota (düz hatta oran) | Ort. MSL | Min AGL |
|---|---|---|---|---|---|
| M90_01 | 8.3 s | 13.4 s | 97.1 km (1.20) | 560 m | 119.7 m |
| M90_02 | 9.0 s | 14.4 s | 106.5 km (1.34) | 760 m | 119.7 m |
| M90_03 | 6.0 s | 11.0 s | 88.6 km (1.12) | 692 m | 118.7 m |
| M90_04 | 9.3 s | 14.5 s | 108.2 km (1.32) | 546 m | 120.0 m |
| M90_05 | 8.5 s | 13.8 s | 105.5 km (1.29) | 464 m | 119.1 m |

Toplam planlama = arama + irtifa profili + yumuşatma. Tüm görevlerde koridor güvenli.

### Aynı görevlerin eski ayarlarla karşılaştırması

| Ayar | Kaynak | Arama süresi | Rota | Sonuç |
|---|---|---|---|---|
| Rehbersiz düz ayar | `bilecik_90km_5_missions_benchmark.json` | 21 – 28 s | 81.6 – 92.9 km | 5/5 |
| A\*'da AGL maliyeti açık | `bilecik_90km_5_missions_lowalt_benchmark.json` | 54.5 – 107.8 s | 83.4 – 94.4 km | **4/5** — M90_02 301 s'de zaman aşımı |

Güncel ayar en kısa rotayı üretmez (düz ayar 82–93 km, güncel 89–108 km); daha
uzun ama belirgin biçimde daha alçak, vadi içinde kalan rota üretir ve aramayı
rehbersiz ayara göre ~2.5–4 kat, AGL maliyetli ayara göre ~6–13 kat hızlandırır.
Eski iki çalıştırmada ortalama MSL kaydedilmediği için
irtifa karşılaştırması yalnızca grafiklerden yapılabilir.

### 31 km Bilecik kanyonu (tek atış)

`single_shot_31km_multibank_result.json` — çok yatışlı (0° / 12° / 25°) varyant,
arama ağırlığı 1.2; grafik `single_shot_31km_4panel_fidelity.png`:

| | |
|---|---|
| Arama | 7202 düğüm, 15.2 s (toplam planlama 16.6 s) |
| Rota | 31.44 km, 524 ilkel, uçuş 13.1 dk |
| İrtifa (MSL) | 259.9 – 680.0 m |
| AGL | min 120.0 / ort. 165.5 / maks. 448.3 m |
| Yumuşatma | 327/327 birleşme, maks. koridor sapması 0.51 m |

**Metrik notu:** Arazi takibinden sonra minimum AGL her rotada ~120 m çıkar;
bu tek başına vadi kullanımını göstermez. Rotaları ortalama/maksimum MSL ve zemin
yüksekliğiyle karşılaştırın.

## Bilinen sınırlar

- Yumuşatma sonrası maksimum roll rate 43.6°/s (limit 15°/s); 31 km scriptinde
  ham rota üzerinde 62°/s. Değişiklikten önce de vardı, çözülmedi. Mission UI
  bunu uyarı (advisory) olarak raporlar.
- Kova tabanlı tek temsilcili arama yaklaşıktır; tamlık/optimalite garantisi yok.
- Spiral/loiter ve 180° dönüş makroları arama ilkeli değildir.
- Model rüzgâr, hız değişimi ve gerçek uçuş kontrol dinamiğini içermez.
- `planner/config.py` içindeki uçuş zarfı sayıları hâlâ test parametresidir,
  gerçek bir platform gereksiniminden türetilmemiştir.

## Dizin yapısı

```text
uav_pathfinder/
├── UAV_Pathfinder.bat        <- çift tıklanabilir menülü başlatıcı
├── README.md                 <- bu dosya
├── requirements.txt
├── Teknik Rapor.pdf          <- yöntem ve ölçümlerin ayrıntılı raporu
├── planner/                  <- üretim planlayıcı kodu
├── mission_ui/               <- web arayüzü (server/ + web/), kendi README'si
├── regions/                  <- 4 bölgenin çalışma DEM'leri, kendi README'si
├── results/                  <- benchmark çıktıları (JSON + PNG)
├── scripts/                  <- görev/benchmark çalıştırıcıları, DEM hazırlığı,
│                                paketleme ve launcher
└── tests/                    <- hızlı sentetik regresyon testleri
```

`results/` içindeki üst seviye JSON/MD dosyaları silinmiş eski deneylerden kalan
arşivdir; güncel çıktılar `results/test_bilecik/` altındadır.
`mission_ui/.cache/` arazi karo önbelleğidir ve istendiğinde yeniden üretilir
(sürüm kontrolüne girmez).
