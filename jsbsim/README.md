# JSBSim offline aircraft characterization

Bu klasör, JSBSim'in çevrimdışı bir aircraft-characterization aracı olarak
kullanılacağı çalışma hattına aittir. Ana planner implementasyonu bu klasörün
kapsamında değildir.

Hedef akış:

```text
OFFLINE
JSBSim -> ölçüm -> konservatif küçük lookup

ONLINE
planner state -> lookup -> terrain/AGL validation -> successor

FINAL
trajectory -> JSBSim dynamic replay
```

Bu hattın amacı planner yolunun JSBSim trajectory'siyle yüzde yüz aynı
olmasını sağlamak değildir. Amaç, planner'ın kullandığı hareketlerin seçilmiş
operasyon zarfında uygulanabilir olmasını ve lookup'ın ölçülen kabiliyeti
iyimser biçimde aşmamasını sağlamaktır.

## Kapsam sınırı

- Referans model: JSBSim F-16; gerçek veya production-calibrated aircraft değil.
- JSBSim, A* node expansion içinde çalıştırılmaz.
- İlk lookup sürümünde speed bir search-state bileşeni değildir.
- Heading entegrasyonuna kadar yalnız straight/vertical zarf planner tarafından
  tüketilebilir.
- AoA, throttle, load factor, surface deflection ve angular-rate geçmişleri
  planner lookup'ına taşınmaz; yalnız offline status/audit kanıtıdır.
- Ana planner dosyaları J1 kapsamında değiştirilmez.

## J1 dosyaları

- `J1_CONTRACT.md`: insan tarafından okunabilir characterization sözleşmesi.
- `characterization_contract.yaml`: aynı frozen J1 sözleşmesinin makinece
  okunabilir karşılığı.
- `DECISIONS.md`: kullanıcıyla birlikte kilitlenen kararların kaydı.

J1 frozen durumdadır. JSBSim kurulumu/çalıştırması, lookup üretimi ve planner
entegrasyonu bu aşamada yapılmamıştır.

## J2 durumu

J2 provenance ve baseline level-flight sanity gate'i tamamlandı: **PASS**.
Rapor ve yeniden üretilebilir girdiler:

- `J2_REPORT.md`
- `j2_reference_configuration.yaml`
- `j2_baseline_sanity.py`
- `test_j2_baseline.py`
- `results/j2_provenance.json`
- `results/j2_runs.json`
- `results/j2_summary.json`

## J3 durumu

J3 nominal-CAS selection ve baseline throttle-policy gate'i tamamlandı: **PASS**.
İlk 1500–6000 m domain'i için prototype fixed characterization speed **305 KCAS**
seçildi. Ayrıntılar `J3_REPORT.md` ve `results/j3_*.json` dosyalarındadır.

## J3.1 durumu

J3.1 domain extension tamamlandı: **PASS**. Reusable prototype aircraft domain'i
0–6000 m MSL'ye genişletildi; eski J3 verileri reuse edildi ve 305 KCAS kararı
korundu. 250/750/1250 m holdout'larında yerel refinement gereği çıkmadı.
Ayrıntılar `J3_1_DOMAIN_EXTENSION_REPORT.md` ve `results/j3_1_*.json` içindedir.

## J4A durumu

J4A straight climb/descent sanity gate tamamlandı: **PASS**. 305 KCAS'ta
0/3000/5000/6000 m MSL üzerinde level, +2° climb ve −2° descent metodolojisi
doğrulandı. Bu hedefler capability limitleri değildir. Ayrıntılar `J4A_REPORT.md`
ve `results/j4a_*.json` içindedir.

## J4B durumu

J4B straight climb/descent capability characterization tamamlandı: **PASS**.
305 KCAS ve afterburner-off context'inde 0–6000 m main grid boyunca 26 raw
capability boundary 0.5° genişliğinde bracketlendi. Ayrıntılar `J4B_REPORT.md`
ve `results/j4b_*.json` içindedir.

## J5A durumu

J5A level-turn methodology sanity gate tamamlandı: **PASS**. 305 KCAS'ta
0/3000/5000/6000 m MSL üzerinde straight ve `±20°` bank left/right turn
metodolojisi üç cold-start ile doğrulandı. Hedefler capability limitleri
değildir. Radius trajectory telemetry'den ölçülür; coordinated-turn theory
yalnız cross-check'tir. Ayrıntılar `J5A_REPORT.md` ve `results/j5a_*.json`
içindedir.
