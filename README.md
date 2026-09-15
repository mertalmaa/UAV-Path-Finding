# UAV Pathfinder

Terrain-aware 3D path-planning research code for a fixed-wing UAV. The authoritative
production planner uses a continuous fixed-wing pose-aware A* with conservative
terrain/AGL checks and an authoritative **Generic Constant-Performance Fixed-Wing Kinematic Model**:

- **Horizontal Kinematic Speed**: 40.0 m/s
- **Max Climb Rate**: +5.0 m/s (at all altitudes)
- **Max Descent Rate**: -5.0 m/s (at all altitudes)
- **Bank Angle**: 25.0 deg (Turn radius ≈ 349.89 m, Turn rate ≈ 6.55 deg/s)
- **Zero Wind** convention
- **Same limits at all altitudes** (no altitude-dependent degradation or LUT requirement)

## Alçak uçuşu çalıştırma

```powershell
python -B -m scripts.benchmark_low_flight
```

Bu komut, paylaşılan B1–B6 senaryolarını, A–F görevlerini ve batı vadisini
**üretim planlayıcısıyla** çalıştırır. Önce yatay rota aranır; ardından seçilen
rota üzerinde uçak sınırlarıyla mümkün olan en alçak irtifa profili hesaplanır.
Varsayılan hedef ve sert minimum arazi açıklığı 100 metredir. Son rotanın her
parçası tekrar arazi çarpışması/açıklığı açısından doğrulanır.

- [B1–B6 önce/sonra grafikleri](results/low_flight/fixed/behavior.png)
- [A–F önce/sonra grafikleri](results/low_flight/fixed/canonical.png)
- [Sonuç raporu](results/low_flight/fixed/report.md)
- JSON dosyaları aynı klasörde gerçek koordinatları, irtifaları, AGL ve hız ölçümlerini içerir.

Başlangıç ve hedef irtifalarını araziye göre tanımlanan vadi örneği:

```powershell
python -B -m scripts.benchmark_low_flight --suite valley --endpoint-mode agl --endpoint-clearance 130 --target-agl 100
```

`--endpoint-clearance` başlangıç/hedef açıklığıdır; `--target-agl` uçuş boyunca
istenen açıklıktır. Başlangıçta doğrudan 100 metre seçmek, yakındaki yükselen
arazi nedeniyle ilk manevrayı olanaksız kılabilir. AGL uç noktaları her görevde
uçulabilirlik garantisi vermez; başarısız görevler açıkça raporlanır.

İsteğe bağlı doğrusal AGL maliyeti, topografik Dijkstra ve ileri arazi/tırmanma
rehberi için `--guided-search` ekleyin. Bu seçenek batı vadisinde doğrulandı;
her görevde daha hızlı veya daha iyi rota üreteceği garanti edilmez.

Koddan alçak uçuş için `planner.terrain_following.plan_terrain_following(...)`
kullanın ve **dönen `plan.trajectories` rotasını** tüketin. `search_result`
ilk aramanın tanısal sonucudur; optimize edilmiş alçak uçuş rotası değildir.
`target_agl_m=100` için `config.min_agl_m=100` seçilmelidir. Genel yapılandırmanın
mevcut 200 m emniyet değeri otomatik olarak düşürülmez.

### Son doğrulama ve sınırlar

- 80 birim testi geçti; eski ve eksik JSBSim JSON bağımlılığı kaldırıldı.
- A–F: BASIC ve COMBINED modlarının tamamında `FOUND`.
- B1–B6 + A–F + vadi: 13/13 alçak uçuş planı başarılı, minimum AGL ≥100 m,
  dikey hızlar ±5 m/s içinde.
- Birleşik dönüşler varsayılan olarak açık. Arama: ağırlıklı A* (`w=1.01`),
  dikey erişilebilirlik rehberi ve kapatılabilir yaklaşık Pareto budaması.

Alçalma sınırını araziyi değiştirerek gizlemek gerekmiyor. Örneğin 40 m/s ile
3 km uçuşta, aynı irtifada başlayıp biten uçak ±5 m/s sınırıyla en fazla
187.5 m alçalabilir. Daha düşük uçuş için başlangıç/hedef AGL'sini veya görev
mesafesini uygun seçin. Mevcut Aladağlar arazisini koruduk.

En düşük profil garantisi **seçilen yatay rota, örnekleme ve kinematik model**
içindir. Tüm olası 3B rotalar arasında küresel en iyi rota garantisi değildir.
Pareto budaması farklı irtifalardaki faydalı alternatifleri eleyebilir;
tanılama için `enable_pareto_z_pruning=False` kullanılabilir. Model rüzgâr,
dikey ivme/pitch geçişleri ve gerçek uçuş kontrol dinamiğini içermez.

## Repository layout

- `planner/` — production planning code (`planner.fixed_wing_envelope`, `planner.pose_search`, `planner.terrain_following`, etc.).
- `tests/` — small canonical regression suite.
- `scripts/` — current data preparation and benchmark runners.
- `working_dem/` — working terrain rasters used by the planner.
- `results/` — current planner benchmark evidence.
- `outputs/terrain_cache/` — reproducible terrain cache used by the real-terrain benchmark.
- `project.md` — current architecture, decisions, status, and blockers.
- `docs/` — design and architecture documents.

## Quick validation

From the repository root:

```powershell
python -m unittest discover -v
```

This runs fast synthetic and current-contract tests.

Current production benchmarks:

```powershell
python -B -m scripts.benchmark_pose_aware_af
python -B -m scripts.benchmark_terrain_following --real-terrain
python -B -m scripts.benchmark_low_flight
```

`experiments/` contains historical research runners, including older copies of
the search algorithm. Use the commands above for current production evidence.
