# PROMPT: Sabit Kanatlı İHA Yol Planlayıcısı Dikey Hareket (Alçalma/Dikilme), EXPANSION_LIMIT ve Dönüş Manevralarının Çözümü

> **Kullanım Talimatı:** Bu dosya, projeyi devralacak yapay zeka kodlama ajanına (AI Agent) doğrudan verilecek detaylı mimari brifing ve görev istemidir.

---

## 1. GÖREV TANIMI VE HEDEF (OBJECTIVE)

Bu repodaki ([UAV-Path-Finding](file:///c:/Users/merta/Desktop/UAV-Path-Finding)) sabit kanatlı İHA yol planlama sistemi, 3B arazide (Copernicus DEM) A* algoritması ile fiziksel olarak uçulabilir rotalar üretmektedir. Ancak sistemde çözülmesi gereken **3 kritik problem** bulunmaktadır:

1. **Alçalma ve Arazi Takibi Yapmama (Düz Uçuş Takıntısı):** İHA görevlerin neredeyse tamamında (Görev A, C, D) araziye yaklaşmamakta, vadi tabanına inmemekte ve sürekli başladığı yüksek irtifada dümdüz (`STRAIGHT_LEVEL`) uçmaktadır.
2. **Arama Patlaması (`EXPANSION_LIMIT`):** Alçalmanın zorunlu olduğu görevlerde (örn. 500m irtifa kaybı gereken Mission E) veya yumuşak arazi cezası eklendiğinde A* algoritması 30.000 düğüm sınırına çarpıp çökmektedir (`EXPANSION_LIMIT` / `TIMEOUT`).
3. **Kombine Dönüş Primitiflerinin (`DESCENDING_LEFT_TURN`, vb.) Kararsızlığı:** İHA viraj alırken aynı anda tırmanma veya alçalma hareketlerini (`CLIMBING_...`, `DESCENDING_...`) standart aramalarda tutarlı kullanamamakta veya arama uzayı şişmektedir.

**Hedefiniz:** Bu sorunların kök nedenlerini ortadan kaldırmak, repodaki deneysel scriptlerde (`experiments/`) ve en son `scripts/benchmark_low_flight.py` üzerinde doğrulanmış olan çözümleri ana üretim planlayıcısına ([planner/pose_search.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/pose_search.py)) tam uyumlu şekilde entegre etmek ve tüm benchmark görevlerinin (A–F ve B1–B6) başarıyla tamamlanmasını sağlamaktır.

---

## 2. ÇOK ÖNEMLİ: TUZAK DOSYA UYARISI

> [!CAUTION]
> **`planner/vertical_motion.py` DOSYASI İLE VAKİT KAYBETMEYİN!**
> * [planner/vertical_motion.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/vertical_motion.py) dosyası, eski ve terk edilmiş olan [planner/astar.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/astar.py) kafes aramasından kalma **ölü bir koddur**.
> * Aktif çalışan üretim motoru olan [planner/pose_search.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/pose_search.py), `vertical_motion.py` dosyasını **ASLA çağırmamakta ve import etmemektedir**.
> * Üretim aramasında dikey hareketler doğrudan [planner/fixed_wing_envelope.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/fixed_wing_envelope.py) ve [planner/physical.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/physical.py) (`build_straight_vertical_trajectory`) üzerinden üretilmektedir.
> * Yapacağınız tüm geliştirmeler **[planner/pose_search.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/pose_search.py)** ve **[planner/config.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/config.py)** üzerinde olmalıdır.

---

## 3. SORUNLARIN KÖK NEDENLERİ (ROOT CAUSES)

### Kök Neden A: Maliyet Fonksiyonu Geometrik Uzunluktan İbarettir
[planner/pose_search.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/pose_search.py#L229) içindeki `_trajectory_edge_cost`:
$$\text{Cost} = \sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2}$$
* Düz uçulduğunda adım maliyeti: $\sqrt{60^2 + 0^2} = 60.00\text{ m}$.
* Alçalış yapıldığında adım maliyeti: $\sqrt{60^2 + 7.5^2} = 60.47\text{ m}$.
* A* yol uzunluğunu minimize ettiği için, araziye çarpmadığı müddetçe alçalmak yolu uzatır (hipotenüs). Vadiye inip tekrar hedefe çıkmak düz gitmekten her zaman daha uzundur. Dolayısıyla A* alçalan düğümleri kuyruğun arkasına atar ve asla seçmez.

### Kök Neden B: Görev Başlangıç/Bitiş İrtifaları ve Kinematik Sınır (EN KRİTİK FARK)
[scripts/benchmark_far_missions.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/scripts/benchmark_far_missions.py#L36) içinde görevler yüksek seyir irtifasında başlar ve biter:
* Görev C (3.0 km): `start_z = 3800m`, `goal_z = 3800m`. (Zemin ~2500m).
* 40 m/s hızla 3 km yolculuk **75 saniye** sürer.
* Maksimum tırmanma/alçalma hızı $\pm 5.0\text{ m/s}$'dir.
* Uçak 37.5 saniye inip 37.5 saniye çıkarsa en fazla:
  $$\Delta z_{\max} = 37.5 \times 5.0 = 187.5\text{ metre}$$
  alçalabilir. Yani uçak fiziksel olarak $3800 - 187.5 = 3612.5\text{ m}$ altına **İ-NE-MEZ**. Zemin 2500m'de iken uçağın vadiye inmesini beklemek fizik kurallarına aykırıdır; uçak ya yere inemez ya da hedefe tırmanmaya vakti kalmaz.
* **Kanıt:** `endpoint-mode=agl` (araziye göre irtifa) verildiğinde uçak **29 adım boyunca kesintisiz alçalabilmektedir** (`STRAIGHT_DESCENT: 29`). Yani alçalmama sorununun yarısı algoritma, diğer yarısı ise test görevlerindeki sabit 3800m seyir sözleşmesidir.

### Kök Neden C: AGL Cezası Eklenince Arama Patlaması (Heuristic Mismatch)
* Alçalmayı zorlamak için kenar maliyetine yumuşak AGL cezası eklendiğinde (`lambda_agl > 0`), sezgisel fonksiyon $h(n)$ arazi cezasından habersiz kalır (Öklid mesafesi hesaplamaya devam eder).
* $g$ hızla artarken $h$ sabit kaldığı için A* körleşir ve 30.000 düğüm sınırında çöker (`EXPANSION_LIMIT`).
* Ayrıca tırmanma/alçalma mod geçişleri cezalandırılmadığı için uçak zikzak yapar (*"VERTICAL ZIGZAG OBSERVED"*).

### Kök Neden D: Mission E Dikey Durum Patlaması (Vertical Combinatorics)
* Mission E'de (4400m $\to$ 3900m) 500 metre alçalma zorunludur.
* Ancak 9.3 km boyunca alçalmanın nerede yapılacağına dair binlerce permütasyon oluşur.
* Tolerans boşluğu nedeniyle düz giden düğümler geçici olarak daha düşük $f$ alır ve arama kilitlenir. Bu sorun `search_heuristic_weight = 1.01` (Weighted A*) ve `enable_pareto_z_pruning = True` ile çözülmektedir.

---

## 4. DENEYLERDE ÇALIŞTIĞI KANITLANMIŞ ÇÖZÜMLER (REFERENCE IMPLEMENTATIONS)

Bu sorunlar repodaki şu dosyalarda çözülmüş ve test edilmiştir:

### Referans 1: [scripts/benchmark_low_flight.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/scripts/benchmark_low_flight.py)
* **Çalıştırma:** `python -B -m scripts.benchmark_low_flight --suite valley --endpoint-mode agl`
* **Sonuç:** İHA tam **29 adım boyunca sürekli alçalarak (`STRAIGHT_DESCENT: 29`)** yalnızca **43 düğümle ve 0.035 saniyede** vadiye inmiş, doğrulanmış minimum AGL tam **100.0 metre** olmuştur.

### Referans 2: [experiments/run_authoritative_valley_validation.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/experiments/run_authoritative_valley_validation.py)
* **Kanıt:** [results/authoritative_valley_validation.json](file:///c:/Users/merta/Desktop/UAV-Path-Finding/results/authoritative_valley_validation.json)
* Baseline 30.000 düğümde çökerken, bu script **44 adım boyunca sürekli alçalarak (`STRAIGHT_DESCENT: 44`)** 2.27 saniyede hedefe ulaşmıştır.
* **Mekanizma:** 2B Dijkstra Vadi Sezgiseli + AGL Yumuşak Maliyeti + Pareto Dominance.

### Referans 3: [experiments/vertical_bottleneck_mitigation_experiment.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/experiments/vertical_bottleneck_mitigation_experiment.py)
* **Kanıt Raporu:** [results/vertical_bottleneck_mitigation_report.md](file:///c:/Users/merta/Desktop/UAV-Path-Finding/results/vertical_bottleneck_mitigation_report.md)
* Mission E'yi **2.79 saniyede (6.049 düğümle)** çözmüş, Mission C ve D'yi 50 kat hızlandırmıştır.
* **Mekanizma:** Weighted A* ($w=1.01$) + Pareto Z-Dominance.

### Referans 4: [planner/terrain_following.py](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/terrain_following.py)
* Yatay rota bulunduktan sonra iki geçişli (backward-forward dynamic programming) irtifa optimizasyonu (`optimize_terrain_following_altitudes`) araziye teğet profil üretir.

---

## 5. ADIM ADIM UYGULAMA PLANI (ACTION ITEMS FOR THE AGENT)

### Adım 1: `planner/config.py` Parametrelerini Senkronize Edin
* `enable_combined_turns = True` (Kombine dönüşler aktif)
* `search_heuristic_weight = 1.01` (Arama ilerleyişini garanti eden ağırlık)
* `enable_pareto_z_pruning = True` (Dikey patlamayı önleyen Pareto elemesi)
* `enable_low_altitude_cost` ve `enable_terrain_guidance` bayraklarının `pose_search.py` üzerindeki etkisini kontrollü şekilde benchmark edin.

### Adım 2: A* Arama Maliyetine AGL Teşviki Verin
* Eğer hedef irtifası serbestse veya hedef vadideyse, aramanın yüksekte kalmasını önlemek için kenar maliyetine yumuşak AGL terimi ekleyin:
  $$\text{cost} = \ell_{\text{3D}} \times \left(1.0 + \lambda \times \frac{\max(0, \text{AGL} - AGL_{\text{target}})}{H_{\text{scale}}}\right)$$
* Aşırı iniş-çıkışı (zikzak / yunuslama) önlemek için $\Delta z$ değişimlerine küçük bir yumuşatma cezası verin.

### Adım 3: Görev İrtifa Sözleşmesini Arazi Farkındalıklı Hale Getirin
* Benchmark scriptlerinde uçağı 3800m gibi yapay seyir irtifalarına kilitlemeyin.
* Vadi veya arazi takibi test edilirken, başlangıç ve hedef irtifalarını arazinin üstünde tanımlayın ($z = \text{elevation} + 120\text{m}$) veya hedef AGL toleransını esnetin.

### Adım 4: Kombine Dönüşleri (`enable_combined_turns`) Doğrulayın
* Viraj alırken alçalma (`DESCENDING_LEFT_TURN`, `DESCENDING_RIGHT_TURN`) ve tırmanma (`CLIMBING_...`) manevralarının arama sırasında seçildiğini ve yörünge güvenliğini sağladığını doğrulayın.

---

## 6. DOĞRULAMA VE BAŞARI KRİTERLERİ (VERIFICATION)

Yapılan değişikliklerin başarılı sayılması için aşağıdaki 3 test paketi eksiksiz geçmelidir:

1. **Birim Testlerin Geçmesi:**
   ```powershell
   python -m unittest discover -v
   ```
   Tüm testler (80/80) hatasız (0 failure, 0 error) tamamlanmalıdır.

2. **Düşük İrtifa ve Vadi Alçalma Benchmarkı:**
   ```powershell
   python -B -m scripts.benchmark_low_flight --suite valley --endpoint-mode agl
   ```
   * İHA'nın vadi boyunca en az 20+ adım alçaldığı (`STRAIGHT_DESCENT`) ve minimum AGL'nin $\ge 100\text{ m}$ güvenli sınırını koruduğu görülmelidir.

3. **Genişletilmiş Görev Benchmarkı:**
   ```powershell
   python -B -m scripts.benchmark_low_flight
   ```
   Tüm B1–B6 ve A–F görevlerinin `FOUND` olarak tamamlandığı ve `results/low_flight/fixed/report.md` altında AGL düşüşü sağlandığı raporlanmalıdır.

---

## 7. YENİ BÖLGESEL ÇALIŞMA HARİTALARI (30×30 km ROI + 5 km BUFFER)

Aladağlar'ın aşırı sarp yapısı (3800m zirveler) dışında, alçak irtifa uçuşu, vadi içi süzülüş ve kanyon geçişlerini test etmek için Türkiye'den 3 farklı bölgenin 30×30 km (5 km tampon marjı ile 40×40 km, 1333×1333 piksel) çalışma DEM'leri üretilmiş ve ayrı klasörlere yerleştirilmiştir:

### 1. Bilecik – Sakarya Vadisi ve Kanyonları (`regions/bilecik/` ve `working_dem/bilecik/`)
* **Merkez:** Lon `30.30°E`, Lat `40.25°N` | **CRS:** `EPSG:32636` (UTM 36N)
* **İrtifa:** Vadi tabanı **97 m** – Zirveler **1278 m** (Medyan: 586 m).
* **Topoğrafik Özellik:** Sakarya Nehri boyunca uzanan derin kanyon yatağı (Osmaneli, Vezirhan, Gölpazarı). 400–800m kot farkı, nehir kıvrımları ve alçak irtifa vadi içi süzülüş manevraları için kusursuzdur.
* **Dosyalar:** `working_dem.tif` (40×40 km), `roi_30km_dem.tif` (30×30 km), `coarse_90m_*.tif` (333×333 px), `terrain_cache/` (`arrays.npz` + `manifest.json`).

### 2. Muğla – Gökova Körfezi, Ula Kanyonu ve Sakar Geçidi (`regions/mugla/` ve `working_dem/mugla/`)
* **Merkez:** Lon `28.35°E`, Lat `37.22°N` | **CRS:** `EPSG:32636` (UTM 36N)
* **İrtifa:** Deniz seviyesi **0 m** – Dağ silsilesi **1891 m** (Medyan: 766 m).
* **Topoğrafik Özellik:** Gökova sahilinden başlayıp Sakar Geçidi ve Ula kanyonu boyunca Muğla platosuna tırmanan dik falezler ve plato inişleri. Deniz seviyesinden 1000+ metreye tırmanma ve platodan sahile alçalma testleri için idealdir.
* **Dosyalar:** `working_dem.tif` (40×40 km), `roi_30km_dem.tif` (30×30 km), `coarse_90m_*.tif` (333×333 px), `terrain_cache/` (`arrays.npz` + `manifest.json`).

### 3. Ankara – Güdül, Kirmir Çayı Kanyonu ve Ayaş (`regions/ankara/` ve `working_dem/ankara/`)
* **Merkez:** Lon `32.25°E`, Lat `40.25°N` | **CRS:** `EPSG:32636` (UTM 36N)
* **İrtifa:** Kanyon tabanı **518 m** – Platolar **1984 m** (Medyan: 1063 m).
* **Topoğrafik Özellik:** Kirmir Çayı Kanyonu kaya yerleşimleri, dar nehir koridorları, volkanik platolar ve dalgalı tepeler. Tepe aşma (ridge hopping) ve kanyon koridoru takip testleri için tasarlanmıştır.
* **Dosyalar:** `working_dem.tif` (40×40 km), `roi_30km_dem.tif` (30×30 km), `coarse_90m_*.tif` (333×333 px), `terrain_cache/` (`arrays.npz` + `manifest.json`).

### Bölge Yükleme Örneği:
```python
import dataclasses
from pathlib import Path
from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi

# Bilecik 30x30 km haritasını yükleme:
cfg = dataclasses.replace(
    DEFAULT_CONFIG,
    working_dem_path=Path("regions/bilecik/working_dem.tif"),
    roi_center_lonlat=(30.30, 40.25),
    roi_size_m=30_000.0,
    target_crs="EPSG:32636",
)
roi = load_roi(cfg)
```
Tüm bölgelerin DEM'lerini yeniden üretmek için:
```powershell
python scripts/build_regional_dems.py
```

