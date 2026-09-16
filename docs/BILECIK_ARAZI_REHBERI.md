# Bilecik: alçalma mesafesi kullanan arazi rehberi

> **Güncel durum (2026-09-16):** Bu rapordaki ölçümler eski `absolute_quadratic`
> rehber maliyetiyle alınmıştır ve `scripts/check_bilecik_stage1.py` kaldırıldı.
> Rehber artık varsayılan olarak açık ve `valley_relative` (yerel vadi tabanına
> göre yükseklik) maliyetini kullanıyor; g-maliyetine de giriyor. Güncel
> sonuçlar için `README.md` ve `project.md`.


M08, aynı 10 saniye / 30.000 düğüm arama bütçesinde çözüldü. Son irtifa
profili de başarılı; sert minimum AGL ve yanal tampon değiştirilmedi.

## Değişiklik

`enable_terrain_guidance=True` artık düşük irtifa maliyeti kapalıyken de arama
sırasını yönlendirebilir. Varsayılan olarak kapalıdır. Geometrik maliyet ve
mevcut fiziksel güvenlik denetimleri bu kullanımda aynı kalır.

Arazi rehberi, 30 m DEM üzerinde her üçüncü pikseli kullanarak yaklaşık
90 m aralıklı bir graf oluşturur. Hedeften ters yönde yayılan maliyet,
komşu arazi yükseklikleri arasındaki tırmanış/alçalma için gereken yatay
mesafeyi de içerir. Seçilen rehber ağacı boyunca alt ve üst irtifa referansları
taşınır; bu referanslardan sapma arama önceliğine ek mesafe olarak yansır.

Bu değerler fiziksel erişilebilirlik kanıtı veya admissible alt sınır değildir.
Örneklenmiş arazi küçük engelleri kaçırabilir; bütün gerçek hareketler orijinal
çözünürlükteki arazi denetiminden geçer. Rehber hiçbir durumu doğrudan elemez.
Rehber bağlantısı olmayan yerlerde geometrik/dikey erişilebilirlik sezgisine
dönülür. Kova bazlı yaklaşık aramanın mevcut sınırlamaları sürer.

## Ölçüm

Komut: `python -B -m scripts.check_bilecik_stage1 --guided`

5 görev, 100 m sert minimum AGL, 60 m yanal tampon, 120 m profil hedefi.
Aşağıdaki yeni süreler rehber hazırlığını içerir; son irtifa profili çözümü
arama süresinin dışındadır. Önceki sütun 1. aşamada kaydedilmiş ölçümlerdir;
çok tekrarlı istatistiksel benchmark değildir.

| Görev | Önceki arama | Rehberli arama | Önceki / yeni düğüm | Son profil |
|---|---:|---:|---:|---|
| M01 | 1.315 s | 1.264 s | 1584 / 102 | FOUND |
| M03 | 0.422 s | 1.199 s | 260 / 42 | FOUND |
| M08 | 10 s TIMEOUT | 1.455 s | 14578 / 380 | FOUND |
| M11 | 1.023 s | 1.319 s | 1317 / 64 | FOUND |
| M18 | 0.269 s | 1.297 s | 84 / 69 | FOUND |

M08 arama yolu yaklaşık 4830.52 m; arama minimum AGL 100.18 m, son profil
minimum AGL 107.68 m. Profilin 120 m hedefi sert sınır değildir.

Önceki ayrı teşhis çalışmasında genişletilmiş bütçeyle M08, 20.79 saniyede
31.539 düğümle bulunmuştu. Yeni çalışma aynı başarısızlık bütçesine dönerek
çözüm üretir. En kısa rota bulunduğu iddia edilmez.

Beş görevin tamamında arama ve profil başarılıdır. Bununla birlikte M03,
M11 ve M18 toplam arama süresi rehberin hazırlık maliyeti nedeniyle arttı.
Bu yüzden özellik genel varsayılan yapılmadı. Kolay görevlerde her zaman
hızlanma veya tüm Bilecik görevlerinde başarı garantisi yoktur.

Ham sonuçlar: `results/test_bilecik/terrain_guided.json`.

## Kontroller

`python -B -m unittest discover -s tests`: 88 test geçti. Yeni kontroller:
yavaş alçalmanın rehber maliyetine etkisi, fazla irtifaya ek mesafe,
bağlantısız/harita dışı durumda geri dönüş, kısmi kenar hücresi, geçersiz
çözünürlük, düşük irtifa maliyeti kapalıyken geometrik maliyetin korunması ve
geçersiz başlangıcın güvenlik tarafından reddedilmesi.
