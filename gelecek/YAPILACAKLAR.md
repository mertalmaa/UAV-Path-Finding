# Sabit Kanatlı İHA Yol Planlayıcı — Gelecek Geliştirme Yol Haritası (Roadmap)

Bu doküman; danışman hocanın prensipleri (**"Tüp mantığı olmayacak, uçağa yapay sınır konmayacak, çıkmaz sokakta geri dönebilecek"**), sabit kanatlı İHA'nın aerodinamik gerçekleri (**"Akrobatik hareket yok, spiral tırmanış var"**) ve yapılan kapsamlı kod denetimi (*code audit*) doğrultusunda hazırlanmış resmi yol haritasıdır.

---

## 1. Temel Mimari İlkeler (Kırmızı Çizgiler)

1. **Tüp / Koridor Duvarı KESİNLİKLE YOK:** Uçak 30×30 km'lik haritanın her koordinatına gitmekte %100 serbesttir.
2. **Sabit Kanatlı İHA Dinamikleri:** Uçak akrobatik hareket yapamaz; dağ aşmak ve kör kanyondan çıkmak için **Spiral Yükselme (Tirbüşon)** ve **$180^\circ$ Koordineli Dönüş** kullanır.
3. **Tek Parça (Single-Shot) Çözüm:** 30+ km rotalar araya elle ara nokta koyulmadan, tek seferde A ve B koordinatları arasından çözülecektir.
4. **Doğrulanmış Fiziksel Model Parametreleri (Tek Kaynak):**
   * **Sabit Seyir Sürati ($V$):** $40.0\text{ m/s}$ ($144\text{ km/s}$)
   * **Yatış Açısı ($\phi$):** $25.0^\circ$
   * **Dönüş Yarıçapı ($R$):** $349.89\text{ metre}$ (Dönüş çapı: $\approx 700\text{ metre}$)
   * **Azami Tırmanma/Alçalma:** $5.0\text{ m/s}$ (%12.5 eğim, $7.13^\circ$ tırmanış açısı)
   * **Tam Spiral ($360^\circ$ Tirbüşon):** $54.96\text{ saniye}$ sürede $2.20\text{ km}$ yay kat eder ve **$+274.80\text{ metre}$** net irtifa kazanır.

---

## 2. Yapılması ve Eklenmesi Gereken 7 Kritik Geliştirme

### 1. Pareto Budamasının Tırmanış ve Spiralleri Öldürmesini Düzeltmek (Acil & En Kritik)
* **Sorun (`pose_search.py:600`):** Mevcut Pareto budaması, aynı $60\times 60\text{ m}$ karesinde hedef irtifaya daha yakın ve maliyeti az olan durumu tutup diğerini eliyor (`altitude_error = abs(pose.z - goal.z)`).  
  Önündeki dağı aşmak için tırmanan veya spiral yükselme atan bir rota, hedef irtifadan uzaklaştığı ve yol kat ettiği için **algoritma tarafından "daha kötü" sanılarak acımasızca çöpe atılıyor!**
* **Çözüm:** 
  * Pareto elemesinde hedef irtifa farkı yerine, **önündeki yerel arazi engellerine göre irtifa marjı** baz alınacak.
  * Tırmanan ve spiral atan rotalara kalkan sağlanacak; böylece engeli aşacak geçerli rotalar daha doğarken öldürülmeyecek.

### 2. Manevra Çözünürlüğünü Artırmak ve Sabit Açı Kısıtını Kırmak
* **Sorun (`pose_search.py:390`):** Uçağın dönüş açısı **sabit $15^\circ$** ve tek bir yarıçapa ($R=349.9\text{m}$) kilitlenmiştir. Hafif $4^\circ-5^\circ$ kıvrılan vadilerde uçak mecburen $15^\circ$ dönüp karşı dağa yönelmekte, sonra düzeltmek için $15^\circ$ ters dönerek **zikzak (S-virajları) çizmektedir.**
* **Çözüm (İki Aşamalı):**
  * **Hafif Kavis İlkesi (Gentle Turn):** Kütüphaneye $15^\circ$ sert dönüşün yanına **$5^\circ$ hafif dönüş** ($10^\circ$ yatış açısıyla) ve **$+2.5\text{ m/s}$ tatlı tırmanış** eklenecek.
  * **Yörünge Eğrisi Yumuşatma (B-Spline Smoothing):** A* $15^\circ$'lik adımlarla kaba koridoru bulduktan hemen sonra, analitik bir B-Spline filtresi uçağın $25^\circ$ yatış dinamiğine göre rotayı **kaymak gibi pürüzsüz sürekli bir eğriye** dönüştürecek.

### 3. Ters Arazi Sezgisi (Reverse Terrain-Aware Heuristic — Tüpsüz Vadi Rehberi)
* **Amaç:** A*'ın 30 km ötedeki hedefe kuş uçuşu düz çizgiyle bakıp dağ duvarlarına toslayarak 80.000 düğümde kilitlenmesini önlemek.
* **Ne Yapılacak:**
  * 90m Coarse DEM üzerinde hedeften geriye doğru 0.03 saniyede coğrafi bir vadi maliyet dalgası (Reverse Dijkstra) hesaplanacak.
  * Uçağın önüne hiçbir tüp veya duvar örülmeyecek; A* bu akıllı pusula sayesinde açık vadi tabanlarını kendiliğinden hissedecek.
  * Çıkmaz kanyonların hedefe maliyeti sonsuz çıkacağı için algoritma çıkmaz vadilere girmeyi baştan reddedecek.

### 4. Spiral Yükselme Makrosunun Arama Ağacına Entegrasyonu (`SPIRAL_UP`)
* **Amaç:** Önüne uçağın %12.5 tırmanış açısını aşan dik bir dağ duvarı çıktığında uçağın spiral çizerek yükselmesi.
* **Ne Yapılacak:**
  * `planner/fixed_wing_envelope.py`'deki `spiral_up` fonksiyonu arama motoruna bağlanacak.
  * Uçak kanyon tabanında $700\text{ m}$ çapında güvenli bir çember çizerek tek turda **$+274.8\text{ m}$** irtifa kazanacak ve dağın zirve kotuna çıkıp yoluna devam edecek.

### 5. Kanyon İçi $180^\circ$ Acil Geri Dönüş İlkesi (`U_TURN_180`)
* **Amaç:** İHA ucu kapalı bir kör kanyona girdiğinde sıkışıp kalmadan geri dönebilmesi.
* **Ne Yapılacak:**
  * Kanyon genişliğinin $\ge 700\text{ m}$ olduğu emniyetli alanda $180^\circ$ koordineli dönüş ilkesi işletilecek.
  * Uçak geldiği vadiye geri süzülecek (fiziksel backtracking) ve bir önceki vadi çatalından diğer açık rotaya sapacak.

### 6. Hibrit / Çok Ölçekli İlkel Kütüphanesi (Multi-Scale Primitives)
* **Amaç:** 31 km'lik yolda derinliği 517 adımdan 120 adımın altına indirerek arama süresini milisaniyelere çekmek.
* **Ne Yapılacak:**
  * **Mikro İlkel (60m):** Kanyon virajları, dar boğazlar ve hedefe varış için.
  * **Makro İlkel (180m - 240m düz hat):** Açık vadiler ve düz rotalar için.

### 7. Yatay İz ile Dikey Optimizasyon Arasında Geri Besleme (Feedback Loop)
* **Sorun:** Yatay $(X,Y)$ izi yüksekten bulunup sonra aşağı yapıştırılmaya çalışıldığında, kanyon daralırsa sistem kilitleniyor.
* **Çözüm:** Eğer dikey optimizasyon (`optimize_terrain_following_altitudes`) yanal tampon ihlali bildirirse, üst planlayıcıya geri besleme yapıp o bölgede yatay rotayı daha geniş bir vadiye kaydıracak.

---

## 3. Güncel Durum (2026-09-16)

| # | Madde | Durum |
| :---: | :--- | :--- |
| 1 | Pareto budaması | **Tamamlandı.** İrtifalar arası eleme kaldırıldı; `enable_pareto_z_pruning` etkisiz. |
| 2 | Manevra çözünürlüğü | **Kısmen.** Yerel B-spline yumuşatma var (`local_trajectory_smoothing`); 5° hafif dönüş ilkeli yok. Yumuşatma sonrası roll rate 43.6°/s, limit 15°/s. |
| 3 | Ters arazi sezgisi | **Tamamlandı.** Vadi-bağıl (HAND) rehber varsayılan; 90 km'de 5/5 görev 8–13 s, rotalar vadilere giriyor. |
| 4 | `SPIRAL_UP` / `U_TURN_180` | **Yapılmadı.** Zarfta `spiral_up` hesabı var, arama ilkeli değil. |
| 5 | Hibrit ilkeller (60 m + 180 m) | **Yapılmadı.** |
| 6 | Yatay-dikey geri besleme | **Tamamlandı (temel).** `plan_terrain_following(max_feedback_passes=...)` profil hatasını arama cezasına çevirir. |
| 7 | Tek tuş otonom motor | **Yapılmadı.** |

Sıradaki öncelikler: roll rate / dikey ivme limitlerinin yumuşatmada sağlanması,
hafif dönüş ilkeli, spiral/U-dönüş makroları.

## 4. Uygulama ve Kodlama Öncelik Sırası (ilk plan)

| Sıra | Geliştirme Maddesi | Neden Bu Sırada? |
| :---: | :--- | :--- |
| **1** | **Pareto Budamasını Düzeltmek** | Bu düzeltilmezse Spiral ve dağ aşma tırmanışları üretildiği an çöpe atılır. |
| **2** | **Manevra Çözünürlüğü (5° Kavis & Spline)** | Kanyonda zikzak çizmeyi anında bitirir, uçuşu pürüzsüzleştirir. |
| **3** | **Ters Arazi Sezgisi (Reverse Heuristic)** | Tüp koymadan 31 km'yi tek parça çözmenin ana anahtarıdır. |
| **4** | **`SPIRAL_UP` ve `U_TURN_180` Makroları** | Çıkmaz falez ve kör kanyonlardan kaçış kabiliyeti kazandırır. |
| **5** | **Hibrit İlkeller (60m + 180m)** | Arama derinliğini düşürüp hızı 3-4 katına çıkarır. |
| **6** | **Yatay-Dikey Geri Besleme Döngüsü** | Kanyon içi alçak uçuşun yatay rotaya tam oturmasını sağlar. |
| **7** | **Tek Tuş Otonom Motor (`auto_mission_planner.py`)** | Sadece A ve B koordinatlarıyla çalışan nihai arayüz. |
