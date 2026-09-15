# Working DEM Standardı ve Bölgesel Veri İşleme Kılavuzu

Bu dizin ([`working_dem/`](file:///c:/Users/merta/Desktop/UAV-Path-Finding/working_dem)), sabit kanatlı İHA yol planlayıcısı için Copernicus GLO-30 verilerinden türetilmiş, uçuş güvenliği garantili çalışma DEM'lerini (Working DEM), kaba ızgaraları ve kalıcı arazi ön belleklerini barındırır.

Her bir bölge için uygulanan matematiksel ve geometrik adımlar bu belgede standardize edilmiştir.

---

## 1. Standart Veri İşleme Hattı (Pipeline Standardı)

Her veri seti için [`scripts/build_working_dem.py`](file:///c:/Users/merta/Desktop/UAV-Path-Finding/scripts/build_working_dem.py) ve [`scripts/build_regional_dems.py`](file:///c:/Users/merta/Desktop/UAV-Path-Finding/scripts/build_regional_dems.py) scriptleri üzerinden şu adımlar uygulanır:

### 1.1. Kaynak Veri ve Projeksiyon
* **Kaynak:** Copernicus GLO-30 DSM (1 yay-saniyesi / ~30 metre, `EPSG:4326`).
* **Hedef Projeksiyon (CRS):** `EPSG:32636` (WGS 84 / UTM Zone 36N).
* **Piksel Boyutu:** `calculate_default_transform` ile tam **30.0 metre × 30.0 metre** olarak sabitlenir (`xy_resolution_m = 30.0`).

### 1.2. Blok-Maksimum Güvenlik Yeniden Örneklemesi (`Resampling.max`)
* Reprojection (WarpedVRT) sırasında bilineer veya kübik enterpolasyon **kullanılmaz**.
* Bunun yerine **Block-Maximum (`rasterio.enums.Resampling.max`)** kullanılır. 
* **Gerekçe:** Dağ zirveleri, sarp kayalıklar veya elektrik direkleri gibi engeller ortalama alınarak asla eritilmez veya alçaltılamaz; planlayıcı her zaman en muhafazakar (en yüksek) araziyi görür.

### 1.3. Harita Sınırı ve Yatay Tampon (Horizontal Margin Buffer)
* **ROI (Region of Interest):** Görevin icra edildiği net operasyonel alan.
  * *Aladağlar (referans):* $10\text{ km} \times 10\text{ km}$ ($333 \times 333$ piksel).
  * *Yeni Bölgeler (Bilecik, Muğla, Ankara):* $30\text{ km} \times 30\text{ km}$ ($1000 \times 1000$ piksel).
* **Yatay Tampon Marjı (Horizontal Buffer Margin):** ROI merkezinden dışarıya doğru her 4 yöne (Kuzey, Güney, Doğu, Batı) **$+5\text{ km}$ marj** eklenir (`BUFFER_MARGIN_M = 5000.0 m`).
* **Çalışma DEM'i (Working DEM Toplam Boyutu):**
  * *Aladağlar:* $10\text{ km} + 2 \times 5\text{ km} = \mathbf{20\text{ km} \times 20\text{ km}}$ ($667 \times 667$ piksel, `HALF_EXTENT_M = 10_000.0 m`).
  * *Bilecik, Muğla, Ankara:* $30\text{ km} + 2 \times 5\text{ km} = \mathbf{40\text{ km} \times 40\text{ km}}$ ($1333 \times 1333$ piksel, `HALF_EXTENT_M = 20_000.0 m`).
* **Yatay Tamponun Amacı:** İHA görev alanının (ROI) sınırlarına yaklaştığında veya yanal güvenlik tamponu (`lateral_buffer_m = 60m`) hesaplandığında hücrelerin harita dışına (`outside_dem`) taşmasını ve gereksiz planlama kilitlenmelerini önlemek.

### 1.4. Çok Ölçekli Havuzlama (Multi-scale Coarse Pooling)
1. **60m Planlama Seviyesi (`factor=2`):**
   * Her $2 \times 2$ (30m) blok havuzlanarak 60m kaba hücreler üretilir.
   * `min`, `max`, `mean`, `relief` istatistikleri [`planner/terrain_cache.py`](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/terrain_cache.py) üzerinden `arrays.npz` içine kaydedilir.
   * Planlayıcının 60m durum kovası (`search_xy_bin_m = 60.0`) ve 60m primitif adımı (`PRIMITIVE_HORIZONTAL_DISTANCE_M = 60.0`) ile doğrudan eşleşir.
2. **90m Genel Rehberlik Seviyesi (`factor=3`):**
   * Her $3 \times 3$ (30m) blok havuzlanarak 90m kaba hücreler üretilir.
   * `coarse_90m_max.tif`, `min`, `mean`, `relief` olarak dışa aktarılır.
   * Dijkstra tabanlı arazi kılavuzluğu (`enable_terrain_guidance`) için kullanılır.

### 1.5. Yanal / Yatay Güvenlik Tamponu Uyumluluğu (`lateral_buffer_m`)
* [`planner/trajectory_safety.py`](file:///c:/Users/merta/Desktop/UAV-Path-Finding/planner/trajectory_safety.py) içindeki `TerrainInfluenceCache` sınıfı, rotanın merkez hattından yatayda 60 metre yarıçapındaki (`lateral_buffer_m = 60.0`) en yüksek araziyi dilate eder.
* Harita kenarlarındaki 5 km'lik yatay marj sayesinde, bu 60m'lik genişletme ROI sınırlarında dahi sıfır hata ile çalışır.

---

## 2. Bölgeler ve Topoğrafik Özellikler

| Bölge Klasörü | Bölge Adı ve Özellikleri | Merkez (Lon, Lat) | UTM CRS | İrtifa Aralığı (Min - Max) | Medyan | Kullanım Amacı |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`aladaglar_*.tif`** | **Aladağlar / Demirkazık**<br>Yüksek dağlık, sarp zirveler | `35.15°E, 37.81°N` | `EPSG:32636` | **1500 m – 3756 m** | 2450 m | Yüksek irtifa dağ aşma, aşırı sarp engel sakınma |
| **[`bilecik/`](file:///c:/Users/merta/Desktop/UAV-Path-Finding/working_dem/bilecik)** | **Bilecik – Sakarya Vadisi & Kanyonları**<br>Osmaneli, Vezirhan, Gölpazarı | `30.30°E, 40.25°N` | `EPSG:32636` | **97 m – 1278 m** | 586 m | Alçak irtifa uçuşu, vadi içi süzülüş, 400-800m kanyon kot farkları |
| **[`mugla/`](file:///c:/Users/merta/Desktop/UAV-Path-Finding/working_dem/mugla)** | **Muğla – Gökova Körfezi & Sakar Geçidi**<br>Akyaka, Ula Kanyonu, Kıran Dağları | `28.35°E, 37.22°N` | `EPSG:32636` | **0 m – 1891 m** | 766 m | Deniz seviyesinden (0m) platoya tırmanma, falez aşma, sahil-dağ geçişi |
| **[`ankara/`](file:///c:/Users/merta/Desktop/UAV-Path-Finding/working_dem/ankara)** | **Ankara – Güdül & Kirmir Çayı**<br>Kirmir Kanyonu, Ayaş, Beypazarı | `32.25°E, 40.25°N` | `EPSG:32636` | **518 m – 1984 m** | 1063 m | Kanyon tabanı takibi, dalgalı tepeler, tepe aşma (ridge hopping) |

---

## 3. Klasör İçeriği ve Dosya Görevleri

Her bölge klasörü (`working_dem/<bolge>/` ve `regions/<bolge>/`) şu standart dosya setine sahiptir:

```text
working_dem/
├── README.md                      <- Bu kılavuz ve teknik standart belgesi
├── aladaglar_N37_E035_utm36n_max.tif (20x20 km, 30m)
├── aladaglar_roi_coarse_90m_*.tif (10x10 km, 90m)
├── bilecik/
│   ├── working_dem.tif            <- 40x40 km (30 km ROI + 5 km tampon), 30m UTM, NoData=-9999
│   ├── roi_30km_dem.tif           <- 30x30 km net görev alanı DEM'i (1000x1000 px)
│   ├── coarse_90m_max.tif         <- 90m kaba blok maksimumu (333x333 px)
│   ├── coarse_90m_min.tif         <- 90m kaba blok minimumu (vadi tespiti)
│   ├── coarse_90m_mean.tif        <- 90m kaba blok ortalaması
│   ├── coarse_90m_relief.tif      <- 90m kaba blok pürüzlülük / rölyef (max - min)
│   ├── region_info.json           <- Koordinat, sınır, CRS ve irtifa metadatası
│   └── terrain_cache/
│       ├── arrays.npz             <- Fine (30m), f2 (60m) ve f3 (90m) sıkıştırılmış dizileri
│       └── manifest.json          <- SHA-256 bütünlük ve kod versiyonu damgası
├── mugla/                         <- (Aynı dosya yapısı)
└── ankara/                        <- (Aynı dosya yapısı)
```

---

## 4. Python ile Bölge Yükleme ve Kullanım

```python
import dataclasses
from pathlib import Path
from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi
from planner.terrain import TerrainQuery

# İstenen bölgeyi seçin: "bilecik", "mugla" veya "ankara"
region_id = "bilecik"

centers = {
    "bilecik": (30.30, 40.25),
    "mugla": (28.35, 37.22),
    "ankara": (32.25, 40.25),
}

config = dataclasses.replace(
    DEFAULT_CONFIG,
    working_dem_path=Path(f"working_dem/{region_id}/working_dem.tif"),
    roi_center_lonlat=centers[region_id],
    roi_size_m=30_000.0,      # 30x30 km operasyon alanı
    target_crs="EPSG:32636",   # UTM 36N
    lateral_buffer_m=60.0,     # 60m yanal güvenlik tamponu
)

# 30x30 km ROI yükle (1000x1000 piksel)
roi = load_roi(config)
terrain = TerrainQuery(roi)

print(f"{region_id.upper()} yüklendi:")
print(f"  Boyut: {roi.width}x{roi.height} px ({roi.resolution[0]}m)")
print(f"  Sınırlar (UTM): {roi.bounds}")
```

---

## 5. Yeniden Üretilebilirlik (Reproducibility)

Tüm haritalar, kaynak karolardan tek bir komutla sıfırdan yeniden üretilebilir:

```powershell
# Üç yeni bölgeyi (Bilecik, Muğla, Ankara) yeniden üretmek için:
python scripts/build_regional_dems.py

# Orijinal Aladağlar DEM'ini yeniden üretmek için:
python scripts/build_working_dem.py
```
