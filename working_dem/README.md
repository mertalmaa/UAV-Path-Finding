# Working DEM Standardı ve Bölgesel Veri İşleme Kılavuzu (90×90 km Standart)

Bu dizin ([`regions/`](file:///c:/Users/PC_10004_YD26/Desktop/uav_pathfinder/regions) ve [`working_dem/`](file:///c:/Users/PC_10004_YD26/Desktop/uav_pathfinder/working_dem)), sabit kanatlı İHA yol planlayıcısı için Copernicus GLO-30 verilerinden türetilmiş, uçuş güvenliği garantili çalışma DEM'lerini (Working DEM), kaba ızgaraları ve kalıcı arazi ön belleklerini barındırır.

4 seçilmiş bölge (**Bilecik, Muğla, Ankara, Aladağlar**) için standart operasyon alanı **90×90 km** ($3000 \times 3000$ piksel) ve $+5\text{ km}$ tampon marjı ile toplam çalışma alanı **100×100 km** ($3333 \times 3333$ piksel) olarak üretilmiştir.

---

## 1. Standart Veri İşleme Hattı (Pipeline Standardı)

Her veri seti için [`scripts/build_regional_dems.py`](file:///c:/Users/PC_10004_YD26/Desktop/uav_pathfinder/scripts/build_regional_dems.py) scripti üzerinden şu adımlar uygulanır:

### 1.1. Kaynak Veri ve Mozaikleme
* **Kaynak:** Copernicus GLO-30 DSM (1 yay-saniyesi / ~30 metre, `EPSG:4326`, `copernicus_glo30_turkey/`).
* **Mozaikleme:** 90×90 km (100×100 km çalışma alanı) sınırlarını kapsayan $2 \times 2$ karo (4 karo / bölge) `rasterio.merge` ile birleştirilir.
* **Hedef Projeksiyon (CRS):** `EPSG:32636` (WGS 84 / UTM Zone 36N).
* **Piksel Boyutu:** `calculate_default_transform` ile tam **30.0 metre × 30.0 metre** olarak sabitlenir (`xy_resolution_m = 30.0`).

### 1.2. Blok-Maksimum Güvenlik Yeniden Örneklemesi (`Resampling.max`)
* Reprojection (`WarpedVRT`) sırasında bilineer veya kübik enterpolasyon **kullanılmaz**.
* Bunun yerine **Block-Maximum (`rasterio.enums.Resampling.max`)** kullanılır. 
* **Gerekçe:** Dağ zirveleri, sarp kayalıklar veya elektrik direkleri gibi engeller ortalama alınarak asla eritilmez veya alçaltılamaz; planlayıcı her zaman en muhafazakar (en yüksek) araziyi görür.

### 1.3. Harita Sınırı ve Yatay Tampon (Horizontal Margin Buffer)
* **ROI (Region of Interest):** $90\text{ km} \times 90\text{ km}$ ($3000 \times 3000$ piksel, `ROI_SIZE_M = 90_000.0 m`).
* **Yatay Tampon Marjı (Horizontal Buffer Margin):** ROI merkezinden dışarıya doğru her 4 yöne (Kuzey, Güney, Doğu, Batı) **$+5\text{ km}$ marj** eklenir (`BUFFER_MARGIN_M = 5000.0 m`).
* **Çalışma DEM'i (Working DEM Toplam Boyutu):** $90\text{ km} + 2 \times 5\text{ km} = \mathbf{100\text{ km} \times 100\text{ km}}$ ($3333 \times 3333$ piksel, `HALF_EXTENT_M = 50_000.0 m`).
* **Yatay Tamponun Amacı:** İHA görev alanının (ROI) sınırlarına yaklaştığında veya yanal güvenlik tamponu (`lateral_buffer_m = 60m`) hesaplandığında hücrelerin harita dışına (`outside_dem`) taşmasını ve gereksiz planlama kilitlenmelerini önlemek.

### 1.4. Çok Ölçekli Havuzlama (Multi-scale Coarse Pooling)
1. **60m Planlama Seviyesi (`factor=2`):**
   * Her $2 \times 2$ (30m) blok havuzlanarak 60m kaba hücreler ($1500 \times 1500$ piksel) üretilir.
   * `min`, `max`, `mean`, `relief` istatistikleri [`planner/terrain_cache.py`](file:///c:/Users/PC_10004_YD26/Desktop/uav_pathfinder/planner/terrain_cache.py) üzerinden `arrays.npz` içine kaydedilir.
   * Planlayıcının 60m durum kovası (`search_xy_bin_m = 60.0`) ve 60m primitif adımı (`PRIMITIVE_HORIZONTAL_DISTANCE_M = 60.0`) ile doğrudan eşleşir.
2. **90m Genel Rehberlik Seviyesi (`factor=3`):**
   * Her $3 \times 3$ (30m) blok havuzlanarak 90m kaba hücreler ($1000 \times 1000$ piksel) üretilir.
   * `coarse_90m_max.tif`, `min`, `mean`, `relief` olarak dışa aktarılır.
   * Dijkstra tabanlı arazi kılavuzluğu (`enable_terrain_guidance`) için kullanılır.

---

## 2. Bölgeler ve Topoğrafik Özellikler (90×90 km)

| Bölge Klasörü | Bölge Adı ve Özellikleri | Merkez (Lon, Lat) | UTM CRS | İrtifa Aralığı (Min - Max) | Medyan | Kullanım Amacı |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **[`bilecik/`](file:///c:/Users/PC_10004_YD26/Desktop/uav_pathfinder/working_dem/bilecik)** | **Bilecik – Sakarya Vadisi & Kanyonları**<br>Osmaneli, Vezirhan, Gölpazarı, İnhisar | `30.30°E, 40.25°N` | `EPSG:32636` | **35 m – 1683 m** | 757 m | Alçak irtifa uçuşu, vadi içi süzülüş, 400-800m kanyon kot farkları, Sakarya havzası geçişleri |
| **[`mugla/`](file:///c:/Users/PC_10004_YD26/Desktop/uav_pathfinder/working_dem/mugla)** | **Muğla – Gökova Körfezi & Sakar Geçidi**<br>Akyaka, Ula Kanyonu, Milas, Yatağan | `28.35°E, 37.22°N` | `EPSG:32636` | **-1 m – 2292 m** | 606 m | Deniz seviyesinden (0m) 2000m+ platoya tırmanma, sahil-dağ geçişi, falez aşma |
| **[`ankara/`](file:///c:/Users/PC_10004_YD26/Desktop/uav_pathfinder/working_dem/ankara)** | **Ankara – Güdül & Kirmir Çayı**<br>Kirmir Kanyonu, Ayaş, Beypazarı, Kızılcahamam | `32.25°E, 40.25°N` | `EPSG:32636` | **467 m – 2389 m** | 1099 m | Kanyon tabanı takibi, dalgalı volkanik tepeler, tepe aşma (ridge hopping) rotaları |
| **[`aladaglar/`](file:///c:/Users/PC_10004_YD26/Desktop/uav_pathfinder/working_dem/aladaglar)** | **Aladağlar – Demirkazık & Çamardı**<br>Demirkazık (3756m), Bolkar Dağları, Ecemiş Fayı | `35.15°E, 37.81°N` | `EPSG:32636` | **153 m – 3700 m** | 1481 m | Yüksek irtifa dağ aşma, aşırı sarp duvarlar, buzul vadileri, yüksek irtifa seyrüsefer |

---

## 3. Klasör İçeriği ve Dosya Görevleri

Her bölge klasörü (`working_dem/<bolge>/` ve `regions/<bolge>/`) şu standart dosya setine sahiptir:

```text
working_dem/
├── README.md                      <- Bu kılavuz ve teknik standart belgesi
├── bilecik/
│   ├── working_dem.tif            <- 100x100 km (90 km ROI + 5 km tampon), 30m UTM, 3333x3333 px
│   ├── roi_90km_dem.tif           <- 90x90 km net görev alanı DEM'i (3000x3000 px)
│   ├── coarse_90m_max.tif         <- 90m kaba blok maksimumu (1000x1000 px)
│   ├── coarse_90m_min.tif         <- 90m kaba blok minimumu (vadi tespiti)
│   ├── coarse_90m_mean.tif        <- 90m kaba blok ortalaması
│   ├── coarse_90m_relief.tif      <- 90m kaba blok pürüzlülük / rölyef (max - min)
│   ├── region_info.json           <- Koordinat, sınır, CRS ve irtifa metadatası
│   └── terrain_cache/
│       ├── arrays.npz             <- Fine (30m), f2 (60m) ve f3 (90m) sıkıştırılmış dizileri
│       └── manifest.json          <- SHA-256 bütünlük ve kod versiyonu damgası
├── mugla/                         <- (Aynı dosya yapısı)
├── ankara/                        <- (Aynı dosya yapısı)
└── aladaglar/                     <- (Aynı dosya yapısı)
```

---

## 4. Python ile Bölge Yükleme ve Kullanım

```python
import dataclasses
from pathlib import Path
from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi
from planner.terrain import TerrainQuery

# İstenen bölgeyi seçin: "bilecik", "mugla", "ankara" veya "aladaglar"
region_id = "bilecik"

centers = {
    "bilecik": (30.30, 40.25),
    "mugla": (28.35, 37.22),
    "ankara": (32.25, 40.25),
    "aladaglar": (35.15, 37.81),
}

config = dataclasses.replace(
    DEFAULT_CONFIG,
    working_dem_path=Path(f"working_dem/{region_id}/working_dem.tif"),
    roi_center_lonlat=centers[region_id],
    roi_size_m=90_000.0,      # 90x90 km operasyon alanı
    target_crs="EPSG:32636",   # UTM 36N
    lateral_buffer_m=60.0,     # 60m yanal güvenlik tamponu
)

# 90x90 km ROI yükle (3000x3000 piksel)
roi = load_roi(config)
terrain = TerrainQuery(roi)

print(f"{region_id.upper()} yüklendi:")
print(f"  Boyut: {roi.width}x{roi.height} px ({roi.resolution[0]}m)")
print(f"  Sınırlar (UTM): {roi.bounds}")
```

---

## 5. Yeniden Üretilebilirlik (Reproducibility)

Tüm haritalar, `copernicus_glo30_turkey/` dizinindeki kaynak karolardan tek bir komutla sıfırdan yeniden üretilebilir:

```powershell
python scripts/build_regional_dems.py
```
