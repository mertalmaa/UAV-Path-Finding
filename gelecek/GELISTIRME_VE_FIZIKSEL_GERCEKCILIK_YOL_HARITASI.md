# Sabit Kanatlı İHA Yol Planlayıcı: Doğrulanmış Fiziksel Kısıtlar, Mimari ve Geliştirme Yol Haritası

Bu doküman; sabit kanatlı İHA yol planlama sisteminin ([UAV-Path-Finding](file:///c:/Users/PC_10004_YD26/Desktop/uav_pathfinder)) aerodinamik, kinematik ve algoritmik temellerini sağlamlaştırmak; garantili alt sınırlar ile yaklaşık arama sezgisellerini birbirinden net şekilde ayırmak ve sistemi devralacak yapay zeka ajanlarına (AI Agent) **matematiksel ve fiziksel olarak gerekçelendirilmiş bir mühendislik brifingi** sunmak amacıyla hazırlanmıştır.

---

## 1. Temel Mimari Çerçeve (Architectural Baseline)

### 1.1. Durum Tanımı ve Hibrit Arama Yapısı
* **Arama Uzayı:** `planner/pose_search.py`, sürekli $\text{Pose}(x, y, z, \psi)$ durumlarını ilerletir. Doğrusal ilkeller 60 m; dönüş ilkelleri 15° ve varsayılan yarıçapta yaklaşık 91.6 m yay uzunluğundadır. `SearchKey`, fiziksel konumu değiştirmeyen 60 m × 60 m × 5 m × 15° kayıt kovasıdır.
* **İrtifa İyileştirme:** Bulunan fiziksel trajektori zinciri, [`planner/altitude_profile.py`](file:///c:/Users/PC_10004_YD26/Desktop/uav_pathfinder/planner/altitude_profile.py) içinde iki geçişli (backward-forward pass) fark kısıtı çözücüsüyle araziye en yakın uçulabilir irtifa zarfına oturtulur.
* **Mimari Netlik (Arama İrtifası vs. Son Profil):** Arama sırasındaki $z$, her ilkelin arazi açıklığıyla birlikte denetlendiği gerçek fiziksel irtifadır. Sonraki profil aşaması aynı yatay iz üzerinde irtifayı yeniden çözer ve ayrıca doğrular; aramanın başarılı olması profil aşamasının başarılı olacağını garanti etmez.
* **Uçak Kısıtları:** Mevcut model sıfır rüzgâr, 40 m/s yatay hız, varsayılan ±5 m/s dikey hız ve 25° yatış kullanır. `FixedWingKinematicModel` fiziksel olmayan parametreleri ve hareket üreticisiyle çelişen hızları reddeder. Hız, yatış geçişi, dikey ivme ve enerji durum değişkeni değildir; bu aşama aerodinamik uçulabilirlik garantisi vermez. Eski `PlannerConfig.max_climb_angle_deg` / `max_descent_angle_deg` alanları pose aramasının uçak zarfını belirlemez.

---

## 2. Mimari Tercihler ve Yöntem Karşılaştırmaları

Mevcut mimariyi savunurken diğer yöntemleri toptan geçersiz saymak yerine, yöntemlerin **hesaplama dinamikleri ve kabulleri** arasındaki farklar gözetilmelidir:

| Yaklaşım | Temsil ve Kinematik Uyumluluk | Hesaplama ve Ölçeklenebilirlik |
|---|---|---|
| **Düz Voksel Izgara (3D Grid) A\* / Dijkstra** | Yönelim kısıtını ($\psi$) ve $R=\frac{V^2}{g\tan\phi}$ dönüş yarıçapını doğrudan içermez; bulunan yolların uçuş dinamiğine uydurulması için ek yumuşatma gerekir. | Durum uzayı homojendir; ancak kinematik kısıtlar eklendiğinde GPU üzerinde sıralı öncelik kuyrukları ve bellek erişim desenleri nedeniyle performans kaybı yaşanabilir. |
| **Örnekleme Tabanlı Yöntemler (RRT / RRT\*)** | Dubins 3D eğrileriyle yönelim kısıtı çözülebilir; ancak $30\times 30\text{ km}$ haritada dar kanyonlar hacimce küçük kaldığından (*Narrow Passage* problemi) vadi içine örnek düşme olasılığı düşüktür. | GPU üzerinde paralel mesafe ve çarpışma testlerine çok uygundur; ancak sabit kanatta yeniden bağlama (*rewiring*) baş açısı sürekliliğini bozabilir. |
| **Pose-Aware A\* + İki Geçişli DP İrtifa Zarfı (Mevcut Mimari)** | Her ilkel adımında uçağın anlık baş açısı, yatış limiti ve dikey tırmanış bütçesi korunur. Dikey arama karmaşıklığı 1B dinamik programlama ile ayrıştırılır. | CPU üzerinde 30 km'lik görevlerde milisaniyeler seviyesinde deterministik çözüm üretir; ancak yatay izin dikey tırmanış için yetersiz kaldığı durumlarda geri besleme gerektirir. |

---

## 3. Geliştirme Maddelerinin Matematiksel ve Fiziksel Gerekçelendirilmesi

### 3.1. Pareto Budamasının Netleştirilmesi (Yaklaşık Arama vs. Garantili Baskınlık)
* **Problem:** Bir $(x, y, \psi)$ hücresinde iki farklı irtifadaki durum karşılaştırılırken salt hedef irtifaya yakınlığa bakmak, ilerideki bir dağı aşmak için tırmanan geçerli durumları erkenden eler.
* **Analitik Yaklaşım:** Durum $A$, durum $B$'yi ancak ve ancak **$B$'nin gelecekte erişebileceği tüm geçerli durumları $A$ da en fazla aynı maliyetle sağlayabiliyorsa** kesin olarak domine edebilir. Yüksek irtifa; tavan irtifası aşımı, gereksiz yol uzaması veya alçalma bütçesi kısıtı nedeniyle her zaman avantajlı değildir.
* **Uygulanan 1. Aşama:** Hedef veya tercih edilen arazi irtifasına yakınlıkla yapılan irtifalar arası eleme kaldırıldı. Her farklı `z_bin` bağımsız tutulur; eski `enable_pareto_z_pruning` alanı çağıran kodlarla uyumluluk için durur ancak `True` olsa da eleme yapmaz. Aynı tam `SearchKey` içinde düşük maliyetli tek temsilci tutulması hâlâ yaklaşık bir tercihtir. Bu aşama kova içi çoklu temsilci eklemez; tamlık veya küresel optimalite garantisi vermez. Korunan irtifa sayısının artması arama süresi ve belleğini artırabilir.

---

### 3.2. Yatay-Dikey Geri Besleme Döngüsü (Coupled Feedback Loop)
* **Problem:** Yatay arama sırasında seçilen vadi koridoru veya dağ geçidi, uçağın dikey tırmanış bütçesi ($\pm 5\text{ m/s}$) için yetersiz yatay mesafe sunuyorsa, dikey çözücü (`altitude_profile`) çözümsüzlük (`INFEASIBLE`) döner.
* **Uygulama:** Geri besleme son aşamaya bırakılmamalı, mimarinin çekirdeğine entegre edilmelidir:
  1. Dikey çözücü başarısız olduğunda, hangi istasyonda ve ne kadar ek tırmanış mesafesi/kot farkı gerektiğini (`failure_index`, `required_delta_s`) raporlar.
  2. Üst planlayıcı bu bilgiyi kullanarak o bölgede yatay rotayı genişletir (örneğin vadide S-çizerek mesafe kazanma veya tırmanış spirali ekleme).

---

### 3.3. Kısıtlı Yörünge Yumuşatma ve Süreklilik ($C^1$ vs. $C^2$, Koridor Emniyeti)
* **Analitik Düzeltme:**
  - İki parabolik parçayı uca eklemek otomatik olarak **$C^2$ süreklilik sağlamaz**; eklem noktasında ivme sıçrıyorsa profil yalnızca $C^1$'dir.
  - Gerçek $C^2$ süreklilik için ivmenin sürekli olması ($\dot{a}_z \le \text{jerk}_{\max}$ veya sürekli eğrilik türevi $\dot{\kappa}$) gereklidir.
* **Yanal Güvenlik Kısıtı:**
  - B-Spline veya Klotoid filtreleri asla serbest uzayda körlemesine uygulanmamalıdır.
  - Yumuşatılmış sürekli eğri, **DEM güvenlik tamponu (`lateral_buffer_m`) ve minimum AGL sınırları içinde kalarak** çözülmelidir (Constrained Spline Optimization).

---

### 3.4. Birleşik Uçuş Mekaniği (Hız – Yatış – Tırmanış – Stall İlişkisi)
Sakin havada koordineli yatay dönüş yarıçapı:
$$R = \frac{V^2}{g \tan\phi}$$

* **Hız Değişiminin Geometrik Etkisi:**
  - $V = 40\text{ m/s}$, $\phi = 25^\circ \implies R \approx 349.9\text{ m}$.
  - Hız süzülüşte $V = 47\text{ m/s}$ olursa $\implies R \approx 483.5\text{ m}$ (%38 daha geniş dönüş).
  - Dolayısıyla dinamik hız eklendiğinde, daha önce $R=350\text{m}$ ile bulunan yatay izin geçerliliği bozulabilir. Hız ve dönüş yarıçapı birlikte planlanmalıdır.
* **Yatışta Tırmanma ve Stall Marjı:**
  - Virajda uçağın taşıma kuvveti eğildiği için ($L \cos\phi = W$) yük faktörü artar: $n = \frac{1}{\cos\phi}$.
  - Yatışlı uçuşta stall sürati artar: $V_s(\phi) = V_{s0}\sqrt{n}$. ($25^\circ$ yatışta $n = 1.103 \implies V_s$ yaklaşık %5 artar).
  - Yatış sırasında motor gücünün bir kısmı indüklenmiş sürüklemeyi yenmeye harcandığından, $25^\circ$ yatışta tırmanış hızı düz uçuştaki $+5.0\text{ m/s}$ seviyesinde kalamaz; dikey hız bütçesi yatış açısına bağlı olarak daraltılmalıdır.
* **Yatış Hızı Limiti:** Maksimum yatış açısının ($\phi_{\max}$) yanı sıra uçağın yatış değişim hızı ($\dot{\phi} \le 15^\circ/\text{s}$) manevra sürekliliğine dahil edilmelidir.

---

### 3.5. Taktik Makrolar: Koşullu Spiral ve $180^\circ$ Geri Dönüş
* **Spiral Yükselme (`SPIRAL_UP`):**
  - $R = 349.9\text{ m}$ çemberde $2.20\text{ km}$ yay boyunca rüzgârsız havada tırmanış hesabı, yatıştaki net tırmanma gücüyle kalibre edilmelidir.
  - Rüzgâr altında uçulduğunda hava kütlesindeki çember yer ekseninde kayar (trokoid); kanyon duvarına sürüklenmemek için yer izi düzeltmesi yapılmalıdır.
* **Kanyon İçi Acil Dönüş (`U_TURN_180`):**
  - Kanyon taban genişliği dönüş çapından ($2R \approx 700\text{ m}$) dar ise fiziksel olarak $180^\circ$ dönüş yapılamaz.
  - Asıl taktiksel başarı, çıkmaza girdikten sonra dönmek değil; **dönüş imkânı olan son genişliği geçmeden önce çıkmazı önceden sezmektir (Point of No Return).**

---

### 3.6. Ters Arazi Sezgisi (Reverse Heuristic) ve Kabul Edilebilirlik (Admissibility)
* **Teorik Zorunluluk:** Coarse DEM üzerinde hedeften geriye doğru hesaplanan bir maliyet dalgası, otomatik olarak admisible (alt sınır) bir sezgisel **değildir**. Eğer yüksek arazileri cezalandırmak için keyfi ağırlıklar eklenirse, sezgisel gerçek uçuş maliyetini fazla tahmin edebilir (*overestimation*) ve üzerinden uçulabilecek geçerli dağ geçitlerini dışlayabilir.
* **Tasarım:**
  1. **Seçenek A (Garantili Alt Sınır):** Coarse DEM üzerindeki topografik uzaklık, 3B fiziksel hareketin maliyetini asla aşmayacak şekilde gevşetilmiş bir graf üzerinde kanıtlanmış alt sınır olarak tanımlanır ($h(s) \le c^*(s)$).
  2. **Seçenek B (Multi-Heuristic A\* - MHA\*):** Tutarlı bir geometrik ana sezgiselin yanında, vadi tabanı tercihini yönlendiren ek sezgisel ayrı bir öncelik kuyruğunda bağımsız olarak işletilir.

---

## 4. Yeniden Sıralanmış Uygulama Öncelik Matrisi (Roadmap)

Geliştirme adımları, bağımlılıklar ve kısıtların birbirini doğrulama sırasına göre şu şekilde yapılandırılmıştır:

| Sıra | Modül / Geliştirme | Odak Noktası | Başarı Ölçütü ve Doğrulama |
| :---: | :--- | :--- | :--- |
| **1** | **Durum Tanımı ve Pareto Elemesi** | `planner.pose_search` | Farklı irtifa kovalarının hedef irtifaya yakınlık nedeniyle elenmemesi; erken tırmanış ve alçalma regresyonları. |
| **2** | **Yatay-Dikey Geri Besleme Döngüsü** | `planner.terrain_following` | Dikey çözülemeyen rotaların üst arayıcıya dönüp alternatif koridor açması. |
| **3** | **Kısıtlı Yörünge Yumuşatma & İvme** | Analitik Filtreler | Yanal emniyet tamponunu ihlal etmeyen, $a_z$ ve $\dot{\phi}$ limitli sürekli geçişler. |
| **4** | **Ters Arazi Sezgisi & Çok Ölçekli İlkel** | `planner.terrain_guidance` | 30 km'lik rotalarda kör kanyonlara girmeden düğüm sayısının %70 azaltılması. |
| **5** | **Spiral ve U-Dönüşü Makroları** | `planner.fixed_wing_envelope` | Kanyon genişliği $\ge 700\text{m}$ olan bölgelerde falez aşma ve kontrollü geri dönüş. |
| **6** | **Rüzgâr ve Değişken Hızlı Ortak Planlama** | Aerodinamik Zarf | $R(V,\phi)$ ve $V_s(\phi)$ kısıtları altında rüzgâr sürüklenmesinin hesaba katılması. |

---

## 5. Kapsamlı Değerlendirme ve Raporlama Kriterleri

Sistemin başarısı yalnızca *"arama süresi (saniye)"* ile değil, çok boyutlu uçuş kriterleriyle değerlendirilecektir:
1. **Çözüm Başarı Oranı:** Bu geliştirme aşamasında yalnızca Bilecik görevleri değerlendirilir. `python -B -m scripts.check_bilecik_stage1`, seçilmiş 5 görevin arama ve irtifa profili sonuçlarını ayrı ayrı `results/test_bilecik/stage1_altitude_preservation.json` dosyasına yazar. Bu alt küme, 30 görevin tamamını temsil eden bir başarı oranı değildir.
2. **Minimum AGL ve Yanal Tampon:** Rota boyunca gözlemlenen en düşük arazi açıklığı ve emniyet ihlali olmaması.
3. **Manevra Sınırı Uyumu:** Gerçekleşen maksimum yatış açısı ($\phi$), yatış hızı ($\dot{\phi}$), dikey ivme ($a_z$) ve stall güvenlik marjı ($V/V_s$).
4. **Başarısızlık Teşhisi:** Çözülemeyen senaryolarda nedenin açıkça raporlanması (`EXPANSION_LIMIT`, `TIMEOUT`, `NO_CORRIDOR_WIDTH`, `INSUFFICIENT_CLIMB_DISTANCE`).
