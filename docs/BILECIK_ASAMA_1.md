# Bilecik — Durum tanımı, uçak kısıtları ve irtifa elemesi

## Yapılan düzeltme

Arama gerçek `(x, y, z, heading)` konumlarını ilerletir. Kayıt anahtarı
60 m × 60 m × 5 m × 15° çözünürlüklü yaklaşık bir kovadır; uçağın konumu
bu kovanın merkezine taşınmaz.

Önceden aynı yatay hücre ve yönde daha ucuz ve hedef/tercih edilen irtifaya
daha yakın bir durum, farklı irtifadaki durumu eleyebiliyordu. Bu ölçüt,
ilerideki engeli aşma veya alçalabilme yeteneğini kanıtlamaz. İrtifalar arası
bu eleme kaldırıldı. Her z kovası kendi temsilcisini korur. Eski
`enable_pareto_z_pruning=True` kullanan çağrılar da hatalı elemeyi açamaz;
alan geriye uyumluluk için etkisiz olarak tutulur.

Aynı tam anahtarda hâlâ düşük maliyetli tek temsilci tutulur; aynı anahtara
dönen geçişler hâlâ elenir. Dolayısıyla arama tam veya küresel optimal değildir.
Çoklu kova içi temsilci bu değişikliğin kapsamında değildir. Daha çok irtifa
korumak daha fazla zaman/bellek gerektirebilir.

## Uçak modeli

- Sabit 40 m/s yatay hız, sıfır rüzgâr.
- Varsayılan tırmanış/alçalma sınırı 5 m/s, yatış 25°, yarıçap yaklaşık 350 m.
- Düz ilkel 60 m; 15° dönüş yaklaşık 91.6 m yay uzunluğunda.
- Model parametrelerinde geçersiz sayılar, pozitif olmayan oranlar,
  geçersiz yatış açıları ve yatış sınırını aşan birleşik dönüş yarıçapı
  çarpanları reddedilir.
- Hareket üreticisi süreyi 40 m/s ile hesapladığı için modelde farklı yatay
  hız verilmesi artık açık hata üretir.
- İvme, yatış geçiş hızı, enerji ve stall modellenmez. Bu bir kinematik
  araştırma modelidir; gerçek uçak performans onayı değildir.

Aramadaki irtifa fiziksel yolun parçasıdır. Sonraki arazi takip aşaması aynı
yatay iz üzerinde ayrı irtifa çözümü ve doğrulama yapar. İki aşamanın başarı
durumları ayrı raporlanır.

## Bilecik kontrolü

Varsayılan DEM ve 30 km çalışma alanı Bilecik'e taşındı. Tarihî bölge veri
dosyaları silinmedi. Güncel geliştirme yol haritasının değerlendirme kapsamı
Bilecik ile sınırlandı.

Komut: `python -B -m scripts.check_bilecik_stage1`

Sert minimum AGL: 100 m; yanal tampon: 60 m; profil hedefi: 120 m.
Görev başına arama bütçesi: 10 saniye / 30.000 genişletme.

| Görev | Arama | Arama süresi | Genişletme | Son profil | Son profil minimum AGL |
|---|---|---:|---:|---|---:|
| M01 — Sakarya kuzey geçişi | FOUND | 1.315 s | 1584 | FOUND | 114.49 m |
| M03 — Vezirhan dar geçit | FOUND | 0.422 s | 260 | FOUND | 120.00 m |
| M08 — Kısa dik alçalma | TIMEOUT | 10.000 s | 14578 | Çalıştırılmadı | — |
| M11 — Vadiden platoya tırmanış | FOUND | 1.023 s | 1317 | FOUND | 120.00 m |
| M18 — Sırtlar arasındaki geçit | FOUND | 0.269 s | 84 | FOUND | 120.00 m |

120 m profil hedefi sert alt sınır değildir; M01'deki 114.49 m, 100 m sert
sınırın üzerindedir. M08 zaman aşımı, fiziksel olarak yol bulunmadığının
kanıtı değildir. Bu beş görev tüm 30 görev için başarı iddiası oluşturmaz.
Önce/sonra performans karşılaştırması yapılmadı; hızlanma iddiası yoktur.

Ham sonuç: `results/test_bilecik/stage1_altitude_preservation.json`.

## Regresyon doğrulaması

`python -B -m unittest discover -s tests`: 82 test geçti.
Yeni kontrol, gerçek kinematik parçaları ve arazi denetimini kullanan kontrollü
bir geçiş grafında ucuz düz varış çıkmazken erken tırmanış/alçalmanın hedefe
ulaşmasını doğrular. Eski bayrağın iki değeri ve yumuşak irtifa maliyetinin
açık/kapalı durumları kapsanır. Model parametre denetimi ayrıca kontrol edilir.
