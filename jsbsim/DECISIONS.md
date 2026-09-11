# JSBSim characterization karar kaydı

Bu dosya yalnız kullanıcıyla açıkça kararlaştırılmış seçimleri ve açık karar
noktalarını kaydeder. Öneriler otomatik olarak karar sayılmaz.

## Kilitlenmiş kararlar

| ID | Karar | Gerekçe |
|---|---|---|
| D-001 | JSBSim yalnız offline characterization ve final replay'de kullanılır. | Node expansion maliyetini ve simulator bağımlılığını search dışında tutmak. |
| D-002 | Referans aircraft F-16'dır fakat production-calibrated gerçek aircraft kabul edilmez. | Prototype scope. |
| D-003 | İlk sürümde speed search state'ine eklenmez. | State explosion'ı önlemek. |
| D-004 | Hard maximum altitude 6000 m MSL'dir. | Kullanıcı gereksinimi. |
| D-005 | Minimum altitude terrain + 100 m AGL'dir. | Kullanıcı gereksinimi; terrain validator boyunca uygulanır. |
| D-006 | Yüzde yüz trajectory/dynamic equality gerekmez. | Amaç konservatif kinematik uygulanabilirliktir. |
| D-007 | Ana planner J1 kapsamında değiştirilmez. | Characterization ve integration çalışma hatlarını ayırmak. |
| D-008 | Ana altitude spacing 500 m; 250 m midpoint holdout; non-conservative bulunan aralıkta yerel 250 m refinement. | Küçük lookup'ı korurken band içindeki iyimser temsili bağımsız noktada yakalamak. |
| D-009 | Provisional prototype acceptance profile: 3 repeat, 20 s steady window, 30 s maximum settling/replay allowance, CAS ±%2, gamma ±0.5°, sustained curvature ±%5, control surface ≤%90, temel metrik repeatability yaklaşık ±%1. | Exact trajectory eşitliği aramadan güvenilir ve tekrarlanabilir ölçüm üretmek. |
| D-010 | J1'de sabit capability derating yoktur; raw ve ilerideki derated capability ayrı tutulur, derating J3/J4 verileri sonrasında kararlaştırılır. | Veri görülmeden keyfî `%20` veya başka sabit margin kilitlememek. |
| D-013 | Characterization sonucu binary değildir: `VALID`, `INFEASIBLE`, `UNKNOWN`. `UNKNOWN`, infeasible sayılamaz ve pruning'de kullanılamaz. | Ölçüm/tolerans problemlerinin erişilebilir maneuver'ları yanlış budamasını önlemek. |
| D-011 | **J3.1 ile superseded.** İlk supported domain mevcut 10×10 km prototype ROI'nin gerçek DEM minimumundan hesaplandı: terrain minimum `1697.698364 m`, minimum required `1797.698364 m`, floor `1500 m`, ceiling `6000 m` MSL. Eski ölçümler korunur. | İlk prototype region'in ihtiyaç duyduğu altitude aralığını kapsayan tarihsel karar. |
| D-012 | Straight level/climb/descent yalnız first validation gate'tir. Gate PASS sonrası straight, left/right level turns ve left/right coupled climb/descent turns aynı harness ile full batch characterize edilir; macro roll-in/arc/roll-out ayrı kalır. | Önce measurement harness'i küçük kapsamda kanıtlamak, sonra final scope'u eksik bırakmadan batch çalışmak. |
| D-014 | `INFEASIBLE` exact capability limiti değildir; komşu `VALID` ve `INFEASIBLE` hedefler sınırı bracket eder, sonra sweep/refinement ile daraltılır. `UNKNOWN` boundary kanıtı değildir. | Ölçüm noktası ile gerçek envelope sınırını birbirine karıştırmamak. |
| D-015 | Reusable prototype aircraft-characterization domain'i `[0, 6000] m MSL`'ye genişletildi; 500 m main grid ve 250 m midpoint policy değişmedi. Önceki `[1500, 6000] m` verileri yeniden koşulmadan korunur. Bu bir gerçek F-16 operational-envelope iddiası değildir ve online `terrain + 100 m AGL` gate'ini değiştirmez. | Future planning regions may contain substantially lower terrain; characterization cost is still small at this stage, so the reusable prototype aircraft domain is extended to 0–6000 m before maneuver envelope generation begins. |
| D-016 | J3.1 birleşik 0–6000 m main-grid değerlendirmesinde 305 KCAS nominal hız olarak yeniden doğrulandı; holdout'lar selection/fitting'e katılmadı. | Mevcut J3 margin-score formülü ve tie-breaker'ları değiştirilmeden 305 KCAS en yüksek minimum marjı korudu. |

## Açık J1 kararları

Yok. Derating değeri ve maneuver-specific throttle policy'leri bilinçli olarak
sonraki characterization kararlarına bırakılmıştır;
bunlar kapanmamış J1 kararı değildir.
