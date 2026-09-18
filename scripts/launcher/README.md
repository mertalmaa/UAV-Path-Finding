# Başlatıcı (`scripts/launcher/`)

Proje kökündeki **`UAV_Pathfinder.bat`** dosyasına çift tıklandığında açılan
menülü başlatıcı. Amaç, komut satırı bilmeyen birinin de kurulumu yapıp
Mission UI'ı açabilmesi ve örnek görevleri çalıştırabilmesidir.

| Dosya | Görev |
|---|---|
| `../../UAV_Pathfinder.bat` | Kök dizindeki giriş noktası; yalnızca doğru klasörde ve `-ExecutionPolicy Bypass` ile PowerShell betiğini açar |
| `uav_launcher.ps1` | Menü, Python seçimi, süreç yönetimi |
| `system_check.py` | Ortam kontrolü (Python sürümü, paketler, bölge verisi, önbellek) |

## Menü

| # | Ne yapar |
|---|---|
| 1 | `python -m mission_ui.server` sunucusunu ayrı bir pencerede başlatır, port cevap verince tarayıcıyı açar; Enter'a basınca süreci ağaç halinde kapatır (planlayıcı alt süreci dahil) |
| 2 | `scripts/run_single_shot_31km.py` — Bilecik 31 km kanyon görevi |
| 3 | `scripts/run_bilecik_90km_5_missions.py` — 5 × ~90 km; istenirse tek görev (`M90_01` … `M90_05`) |
| 4 | `pytest -q tests` ve `pytest -q mission_ui/tests` |
| 5 | `.venv` oluşturur ve `requirements.txt` kurar |
| 6 | `system_check.py` |
| 7 | `results/test_bilecik` klasörünü Gezgin'de açar |

## Python seçimi

Sırayla denenir: `.venv\Scripts\python.exe` → `py -3` → `python`.
Menüdeki üst satır hangisinin kullanıldığını gösterir. Hiçbiri yoksa menü
5) Kurulum'a yönlendirir; Python kurulu değilse `python.org` kurulumunda
"Add python.exe to PATH" işaretlenmelidir.

## Port

Varsayılan 8765 (arazi karoları 8766). Port doluysa başlatıcı 8767, 8769 …
diye ikişer artırarak boş bir çift arar. Sabitlemek için:

```powershell
UAV_Pathfinder.bat -Port 8800 -Region mugla
```

Parametreler: `-Region <id>` (bilecik | mugla | ankara | aladaglar),
`-Port <n>`, `-NoBrowser`.

## `system_check.py` tek başına

```powershell
python scripts/launcher/system_check.py
```

Çıkış kodu 0 ise ortam hazır, 1 ise zorunlu bir bileşen eksiktir. Zorunlular:
Python ≥ 3.9, numpy, scipy, rasterio, pyproj, affine, Pillow ve en az bir
kullanılabilir bölge. matplotlib ve pytest yalnızca `scripts/` ve testler için
gerekir; eksiklerse uyarı verilir, hata değil.

## Neden .exe değil

Depoda daha önce derlenmiş bir `UAV_Pathfinder.exe` (C# konsol başlatıcısı)
bulunuyor, ancak kaynağı depoda tutulmuyordu. Aynı işi yapan bu betikler
okunabilir, sürüm kontrolünde diff'lenebilir ve derleme zinciri gerektirmez.
Gerçekten tek dosyalık bir .exe isteniyorsa iki yol vardır:

* bu `.ps1`'i bir .exe içine sarmak (yine Python kurulumu gerektirir), veya
* PyInstaller ile planlayıcıyı paketlemek — rasterio/GDAL ikilileri yüzünden
  ~300 MB'ı aşar ve Windows üzerinde derlenmesi gerekir.
