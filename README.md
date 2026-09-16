# UAV Pathfinder

Sabit kanatlı İHA için araziye duyarlı 3B yol planlama araştırma kodu.
Güncel çalışma bölgesi **Bilecik** (`regions/bilecik/working_dem.tif`, UTM 36N,
30 m piksel). Planlayıcı hem 30×30 km hem 90×90 km ROI üzerinde tek seferde
(ara nokta olmadan) başlangıç→hedef rotası üretir.

Son güncelleme: 2026-09-16.

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
| `min_agl_m` | 200 | görev scriptleri 100 m kullanır |

Büyük rehber ızgaraları (≥40 000 hücre) scipy ile çözülür; 90 km ROI'de
rehber kurulumu ≈1.5 s.

## Çalıştırma

```powershell
python -B scripts/run_single_shot_31km.py                      # Bilecik 31 km kanyon
python -B scripts/run_bilecik_90km_5_missions.py               # 5 adet ~90 km görev
python -B scripts/run_bilecik_90km_5_missions.py --mission M90_02
python -m pytest -q tests                                      # 97 birim test
```

Diğer scriptler: `run_5_long_30km_single_shot.py`, `run_bilecik_30_tests.py`,
`run_bilecik_5_local_spline_benchmark.py` (+ `create_5mission_summary_atlas.py`),
`run_mavi_mavi_test.py`; veri hazırlığı: `build_working_dem.py`,
`build_regional_dems.py`. Bu scriptler yeni varsayılanlarla yeniden
çalıştırılmadı.

## Güncel sonuçlar

31 km (Bilecik kanyonu, `results/test_bilecik/single_shot_31km_4panel_fidelity.png`):

| | Eski (mutlak rehber + A\*'da AGL maliyeti) | Güncel |
|---|---|---|
| Arama | 3750 düğüm, 4.9 s | 2002 düğüm, 2.6 s |
| Rota | 31.34 km | 34.62 km |
| Ortalama / maks. irtifa (MSL) | 415 / 624 m | 352 / 612 m |

90 km (`results/test_bilecik/bilecik_90km_5_missions_valley_benchmark.json`,
grafikler `results/test_bilecik/plots/m90_0X_valley_fidelity_report.png`):

| Görev | Arama | Rota (düz hatta oran) | Ort. MSL | Eski düz ayar ort. MSL |
|---|---|---|---|---|
| M90_01 | 9.2 s | 97.1 km (1.20) | 560 m | 707 m |
| M90_02 | 11.5 s | 106.5 km (1.34) | 760 m | 1019 m |
| M90_03 | 8.1 s | 88.6 km (1.12) | 692 m | 758 m |
| M90_04 | 12.6 s | 108.2 km (1.32) | 546 m | 634 m |
| M90_05 | 11.1 s | 105.5 km (1.29) | 464 m | 822 m |

Eski A\*'da AGL maliyetli ayar aynı görevlerde 54–108 s sürüyordu, M90_02 300 s'de
zaman aşımına düştü (`bilecik_90km_5_missions_lowalt_benchmark.json`).
Tüm görevlerde minimum AGL ≈ 118–120 m, koridor güvenli.

**Metrik notu:** Arazi takibinden sonra ortalama AGL her rotada ~120 m çıkar;
vadi kullanımını göstermez. Rotaları ortalama/maksimum MSL ve zemin
yüksekliğiyle karşılaştırın.

## Bilinen sınırlar

- Yumuşatma sonrası maksimum roll rate 43.6°/s (limit 15°/s); 31 km scriptinde
  ham rota üzerinde 62°/s. Değişiklikten önce de vardı, çözülmedi.
- Kova tabanlı tek temsilcili arama yaklaşıktır; tamlık/optimalite garantisi yok.
- Spiral/loiter ve 180° dönüş makroları arama ilkeli değildir.
- Model rüzgâr, hız değişimi ve gerçek uçuş kontrol dinamiğini içermez.

## Dizin yapısı

- `planner/` — üretim planlayıcı kodu.
- `tests/` — hızlı sentetik regresyon testleri.
- `scripts/` — görev/benchmark çalıştırıcıları ve DEM hazırlığı.
- `regions/`, `working_dem/` — çalışma DEM'leri (bkz. ilgili README'ler).
- `results/` — benchmark çıktıları (`results/test_bilecik/` güncel; üst
  seviyedeki JSON/MD dosyaları silinmiş eski deneylerden kalan arşivdir).
- `outputs/terrain_cache/` — eski A–F benchmarkının arazi önbelleği (arşiv).
- `project.md` — mimari, sözleşmeler ve durum. `docs/HISTORY.md` — tarihçe.
- `gelecek/` — yol haritası ve yapılacaklar.
