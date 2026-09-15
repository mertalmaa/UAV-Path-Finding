# Sabit Kanatlı İHA Yol Planlayıcı — Gelecek Geliştirme Yol Haritası (Roadmap)

Bu doküman, hocanın koyduğu kurallar (**"Tüp mantığı olmayacak, uçağa yapay sınır konmayacak, çıkmaz sokakta geri dönebilecek"**) ve sabit kanatlı İHA'nın gerçek dinamikleri (**"Akrobatik hareket yok, spiral tırmanış var"**) doğrultusunda sisteme eklenmesi gereken geliştirmeleri özetler.

---

## 1. Temel Mimari İlkeler (Kırmızı Çizgiler)
1. **Tüp / Koridor Duvarı KESİNLİKLE YOK:** Uçak 30×30 km'lik haritanın her noktasına gitmekte tamamen serbesttir.
2. **Sabit Kanatlı İHA Uyumluluğu:** İmmelmann veya dik Şandel gibi aşırı G çeken akrobatik hareketler yerine **Spiral Yükselme (Tirbüşon)** ve **$180^\circ$ Vadi İçi Dönüş** kullanılacaktır.
3. **Manuel Waypoint Bağımlılığı Bitiyor:** 30+ km rotalar araya elle ara nokta koyulmadan, tek parça (*single-shot*) olarak A ve B koordinatları arasından çözülecektir.

---

## 2. Yapılması ve Eklenmesi Gereken 5 Temel Geliştirme

### 1. Ters Arazi Sezgisi (Reverse Terrain-Aware Heuristic)
* **Amaç:** A* algoritmasının hedefe körü körüne düz çizgiyle bakıp dağlara toslamasını engellemek.
* **Ne Yapılacak:**
  * 90m Coarse DEM üzerinde hedeften geriye doğru 0.03 saniyede çalışan coğrafi bir maliyet dalgası (Reverse Dijkstra) hesaplanacak.
  * A*, tüpe hapsedilmeden bu akıllı pusula sayesinde açık vadi tabanlarını kendiliğinden hissedecek.
  * 31 km ve üzeri uçuşlar tüpsüz ve ara waypoint'siz tek seferde bulunabilecek.

### 2. Spiral Yükselme Makrosunun Arama Ağacına Entegrasyonu (`SPIRAL_UP`)
* **Amaç:** Önüne uçağın azami tırmanış açısını (%9) aşan dik bir dağ duvarı veya kanyon amfitiyatrosu çıktığında uçağın olduğu yerde yükselmesi.
* **Ne Yapılacak:**
  * `planner/fixed_wing_envelope.py` içinde zaten matematiksel olarak tanımlı olan `spiral_up` (360° dönüşte ~112m irtifa kazanımı) fonksiyonu arama motoruna (`pose_search.py`) aktif bir hareket ilkesi olarak eklenecek.
  * Uçak kanyonun tabanında 1 veya 2 tur dönerek dağın zirve kotuna çıkacak ve engeli aşacak.

### 3. Kanyon İçi $180^\circ$ Acil Geri Dönüş İlkesi (`U_TURN_180`)
* **Amaç:** İHA ucu kapalı veya dönüş yarıçapından ($R < 233\text{ m}$) daha dar bir kör kanyona girdiğinde sıkışıp kalmasını (çıkmaz sokak) önlemek.
* **Ne Yapılacak:**
  * Kanyon genişliği elverdiği noktada $180^\circ$ koordineli dönüş ilkesi tetiklenecek.
  * Uçak geldiği vadiye geri dönecek (fiziksel backtracking) ve bir önceki vadi çatalından diğer açık rotaya sapacak.

### 4. Hibrit / Çok Ölçekli İlkel Kütüphanesi (Multi-Scale Primitives)
* **Amaç:** 31 km'lik yolda arama ağacı derinliğini 517 adımdan 100 adımın altına indirmek ve arama süresini milisaniyelere çekmek.
* **Ne Yapılacak:**
  * Kütüphaneye iki farklı adım boyutu tanımlanacak:
    * **Mikro İlkel (60m, $15^\circ$ viraj):** Dar kanyonlar, keskin virajlar ve hedefe yaklaşma için.
    * **Makro İlkel (180m - 240m düz hat):** Açık vadiler ve düz rotalar için.
  * Bu sayede kanyonda kıvraklık korunurken, açık arazide hız 3-4 katına çıkacak.

### 5. Uçtan Uca Tek Tuş Otonom Uçuş Motoru (`auto_mission_planner.py`)
* **Amaç:** Kullanıcının yalnızca Başlangıç (A) ve Hedef (B) vermesi.
* **Ne Yapılacak:**
  * Akıllı Arazi Sezgisi + Hibrit İlkel + Spiral Yükselme birleştirilerek tek bir komutla çalışan uçtan uca planlayıcı oluşturulacak.
  * Çıktı olarak 2 panelli yüksek çözünürlüklü uçuş haritası ve manevra telemetrisi otomatik üretilecek.
