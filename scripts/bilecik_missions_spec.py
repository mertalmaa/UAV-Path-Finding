"""30 Diverse Operational Test Missions in Bilecik.

Tests all aspects of fixed-wing low-altitude flight, maneuvers, canyon-following,
steep descents, climb-outs, ridge crossing, obstacle avoidance, and turns.
"""
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from pathlib import Path
import numpy as np

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg


@dataclass(frozen=True)
class BilecikMissionDef:
    id: str
    name: str
    category: str
    description: str
    purpose: str
    start_xy: Tuple[float, float]
    goal_xy: Tuple[float, float]
    start_alt_offset_m: float  # above ground or specific MSL
    goal_alt_offset_m: float
    start_heading_deg: Optional[float] = None  # None -> auto bearing to goal
    goal_tolerance_xy_m: float = 90.0
    goal_tolerance_z_m: float = 15.0


BILECIK_MISSIONS: List[BilecikMissionDef] = [
    # -------------------------------------------------------------------------
    # CATEGORY 1: Kanyon ve Vadi Tabanı Takibi (Canyon & Riverbed Following)
    # -------------------------------------------------------------------------
    BilecikMissionDef(
        id="M01",
        name="Sakarya_Riverbed_North_Transit",
        category="Canyon & Riverbed Following",
        description="Sakarya Nehri kanyon tabanı boyunca kuzeye doğru alçak irtifa seyir.",
        purpose="Düz kanyon tabanında araziye teğet (120m AGL) stabil seyir ve nehir yatağı takibi.",
        start_xy=(257000.0, 4467000.0),
        goal_xy=(257000.0, 4470500.0),
        start_alt_offset_m=130.0,
        goal_alt_offset_m=130.0,
        start_heading_deg=0.0,
    ),
    BilecikMissionDef(
        id="M02",
        name="Sakarya_Riverbed_South_Transit",
        category="Canyon & Riverbed Following",
        description="Sakarya Nehri vadisi boyunca güneye doğru alçak irtifa süzülüş.",
        purpose="Ters yönde (güneye) nehir yatağı takibi ve minimum AGL sınırlarının korunumu.",
        start_xy=(258500.0, 4467000.0),
        goal_xy=(258500.0, 4464000.0),
        start_alt_offset_m=130.0,
        goal_alt_offset_m=130.0,
        start_heading_deg=180.0,
    ),
    BilecikMissionDef(
        id="M03",
        name="Vezirhan_Narrow_Gorge_Passage",
        category="Canyon & Riverbed Following",
        description="Vezirhan mevkiinde vadinin iki dağ yamacı arasında daraldığı boğaz geçişi.",
        purpose="Her iki yanda dik kanyon duvarları varken dar koridorda güvenli merkez hattı uçuşu.",
        start_xy=(259000.0, 4460000.0),
        goal_xy=(259000.0, 4463000.0),
        start_alt_offset_m=140.0,
        goal_alt_offset_m=140.0,
        start_heading_deg=0.0,
    ),
    BilecikMissionDef(
        id="M04",
        name="Osmaneli_Meander_River_Bend",
        category="Canyon & Riverbed Following",
        description="Nehir kıvrımını (menderes) takip eden hafif kavisli vadi geçişi.",
        purpose="Vadi tabanında viraj alırken yanal tampon (lateral buffer) ile yamaçlara çarpmama.",
        start_xy=(257500.0, 4468000.0),
        goal_xy=(259500.0, 4470500.0),
        start_alt_offset_m=140.0,
        goal_alt_offset_m=140.0,
    ),
    BilecikMissionDef(
        id="M05",
        name="Deep_Canyon_Corridor_4km",
        category="Canyon & Riverbed Following",
        description="Sakarya ana kanyonu içinde 4 km boyunca kesintisiz alçak irtifa koridor uçuşu.",
        purpose="Uzun mesafeli vadi içi alçak irtifa (low AGL) uçuş stabilitesi ve tutarlı yörünge.",
        start_xy=(258000.0, 4450000.0),
        goal_xy=(258000.0, 4454000.0),
        start_alt_offset_m=130.0,
        goal_alt_offset_m=130.0,
        start_heading_deg=0.0,
    ),

    # -------------------------------------------------------------------------
    # CATEGORY 2: Sarp Vadiden İniş ve Dik Alçalma (Steep Valley Descent)
    # -------------------------------------------------------------------------
    BilecikMissionDef(
        id="M06",
        name="East_Plateau_To_Riverbed_Descent",
        category="Steep Valley Descent",
        description="Doğu platosundan (800m) Sakarya kanyon tabanına (250m) dik alçalma.",
        purpose="550 metrelik irtifa farkında sürekli alçalma (`STRAIGHT_DESCENT`) ve dalış kontrolü.",
        start_xy=(263000.0, 4460000.0),
        goal_xy=(259000.0, 4460000.0),
        start_alt_offset_m=150.0,
        goal_alt_offset_m=130.0,
        start_heading_deg=270.0,
    ),
    BilecikMissionDef(
        id="M07",
        name="Golpazari_Highland_Descent",
        category="Steep Valley Descent",
        description="Gölpazarı yükseklerinden vadi içi koluna doğru süzülüş.",
        purpose="Yüksek irtifadan tali vadiye süzülürken yamaç eğimine uygun sabit alçalma oranı.",
        start_xy=(272000.0, 4462000.0),
        goal_xy=(268000.0, 4460000.0),
        start_alt_offset_m=150.0,
        goal_alt_offset_m=140.0,
    ),
    BilecikMissionDef(
        id="M08",
        name="Short_Steep_Valley_Drop",
        category="Steep Valley Descent",
        description="Kısa mesafede (2 km) yamaç terasından kanyon dibine hızlı irtifa kaybı.",
        purpose="Maksimum alçalma hızında (-5 m/s) sınırları zorlayarak vadi tabanına oturma.",
        start_xy=(261500.0, 4465000.0),
        goal_xy=(258500.0, 4465000.0),
        start_alt_offset_m=150.0,
        goal_alt_offset_m=130.0,
        start_heading_deg=270.0,
    ),
    BilecikMissionDef(
        id="M09",
        name="North_Ridge_To_Basin_Descent",
        category="Steep Valley Descent",
        description="Kuzey sırtından Osmaneli havzasına doğru güneybatı yönlü sürekli iniş.",
        purpose="Köşegen doğrultuda arazi eğimi boyunca kombine dönüş ve alçalma manevraları.",
        start_xy=(265000.0, 4471000.0),
        goal_xy=(261000.0, 4468000.0),
        start_alt_offset_m=150.0,
        goal_alt_offset_m=130.0,
    ),
    BilecikMissionDef(
        id="M10",
        name="Canyon_Wall_Diagonal_Descent",
        category="Steep Valley Descent",
        description="Kanyon duvarı boyunca çapraz süzülerek tabana iniş.",
        purpose="Uçurum kenarından vadi yatağına çapraz açıyla yaklaşma ve güvenli frenleme.",
        start_xy=(262000.0, 4455000.0),
        goal_xy=(258500.0, 4453000.0),
        start_alt_offset_m=160.0,
        goal_alt_offset_m=130.0,
    ),

    # -------------------------------------------------------------------------
    # CATEGORY 3: Vadiden Tırmanış ve Kaçış (Valley Climb-Out & Escape)
    # -------------------------------------------------------------------------
    BilecikMissionDef(
        id="M11",
        name="Riverbed_To_Plateau_Climb",
        category="Valley Climb-Out & Escape",
        description="Vadi tabanından (200m) doğu platosuna (750m) sürekli tırmanış.",
        purpose="+550m irtifa kazanımında kesintisiz tırmanış (`STRAIGHT_CLIMB`) kabiliyeti.",
        start_xy=(259000.0, 4461000.0),
        goal_xy=(264000.0, 4461000.0),
        start_alt_offset_m=130.0,
        goal_alt_offset_m=160.0,
        start_heading_deg=90.0,
    ),
    BilecikMissionDef(
        id="M12",
        name="Rapid_Canyon_Escape_Climb",
        category="Valley Climb-Out & Escape",
        description="Dar kanyon içerisinden acil kaçış tırmanışı.",
        purpose="Maksimum tırmanma açısıyla (+5 m/s) kanyon kenarını aşarak emniyet irtifasına çıkma.",
        start_xy=(258500.0, 4464000.0),
        goal_xy=(261500.0, 4464000.0),
        start_alt_offset_m=130.0,
        goal_alt_offset_m=160.0,
        start_heading_deg=90.0,
    ),
    BilecikMissionDef(
        id="M13",
        name="South_Gorge_Climb_Out",
        category="Valley Climb-Out & Escape",
        description="Güney kanyon kesiminden yüksek sırtlara doğru doğu yönlü tırmanış.",
        purpose="Yüksek yamaç gradyanında stall veya kinematic ihlal yapmadan irtifa alma.",
        start_xy=(259000.0, 4452000.0),
        goal_xy=(263000.0, 4452000.0),
        start_alt_offset_m=130.0,
        goal_alt_offset_m=160.0,
        start_heading_deg=90.0,
    ),
    BilecikMissionDef(
        id="M14",
        name="Stepped_Hillside_Climb",
        category="Valley Climb-Out & Escape",
        description="Kademeli tepeler üzerinden aşamalı tırmanış rotası.",
        purpose="Tırmanma ve düz uçuş adımlarının dengeli dağılımı ile tepe sırtına yerleşme.",
        start_xy=(260500.0, 4457000.0),
        goal_xy=(264500.0, 4457000.0),
        start_alt_offset_m=130.0,
        goal_alt_offset_m=150.0,
        start_heading_deg=90.0,
    ),
    BilecikMissionDef(
        id="M15",
        name="Northwest_Valley_Exit_Climb",
        category="Valley Climb-Out & Escape",
        description="Osmaneli vadisinden kuzeydoğu yaylalarına tırmanışlı çıkış.",
        purpose="Alçak tabandan yüksek sırtlara doğru açısal tırmanış ve arazi temizleme.",
        start_xy=(260000.0, 4468000.0),
        goal_xy=(264000.0, 4471000.0),
        start_alt_offset_m=130.0,
        goal_alt_offset_m=160.0,
    ),

    # -------------------------------------------------------------------------
    # CATEGORY 4: Sırt Aşma ve Tepe Engeli Sakınma (Ridge Crossing & Peak Detour)
    # -------------------------------------------------------------------------
    BilecikMissionDef(
        id="M16",
        name="Direct_Peak_Bypass_Left",
        category="Ridge Crossing & Peak Detour",
        description="Doğrudan rotanın üzerinde 900m zirve varken soldan baypas manevrası.",
        purpose="Aşırı yüksek zirveye tırmanmak yerine yatayda vadiye saparak enerjiyi koruma.",
        start_xy=(263000.0, 4465000.0),
        goal_xy=(269000.0, 4465000.0),
        start_alt_offset_m=150.0,
        goal_alt_offset_m=150.0,
        start_heading_deg=90.0,
    ),
    BilecikMissionDef(
        id="M17",
        name="Direct_Peak_Bypass_Right",
        category="Ridge Crossing & Peak Detour",
        description="Merkez sırt üzerindeki engebeli tepeyi sağdan dolanarak geçiş.",
        purpose="Sağ yöne dönüşle tepe eteğindeki alçak koridoru seçme davranışı.",
        start_xy=(264000.0, 4458000.0),
        goal_xy=(269000.0, 4458000.0),
        start_alt_offset_m=150.0,
        goal_alt_offset_m=150.0,
        start_heading_deg=90.0,
    ),
    BilecikMissionDef(
        id="M18",
        name="Saddle_Pass_Through_Ridge",
        category="Ridge Crossing & Peak Detour",
        description="İki yüksek zirve arasındaki en alçak boyun (bel / saddle) noktasından geçiş.",
        purpose="En az irtifa harcayan doğal dağ geçidini (saddle) bularak sırtı aşma.",
        start_xy=(262000.0, 4462000.0),
        goal_xy=(267000.0, 4462000.0),
        start_alt_offset_m=150.0,
        goal_alt_offset_m=150.0,
        start_heading_deg=90.0,
    ),
    BilecikMissionDef(
        id="M19",
        name="Double_Ridge_Hop",
        category="Ridge Crossing & Peak Detour",
        description="Aralarında tali vadi bulunan iki ardışık dağ sırtını peş peşe aşma.",
        purpose="Tırmanış-alçalış-tırmanış döngüsünde yunuslama yapmadan kararlı arazi aşımı.",
        start_xy=(265000.0, 4455000.0),
        goal_xy=(272000.0, 4455000.0),
        start_alt_offset_m=150.0,
        goal_alt_offset_m=150.0,
        start_heading_deg=90.0,
    ),
    BilecikMissionDef(
        id="M20",
        name="Highest_Peak_1278m_Circumnavigation",
        category="Ridge Crossing & Peak Detour",
        description="Bilecik bölgesinin en yüksek zirvesinin (1278m) etrafından güvenli dolanma.",
        purpose="1278 metrelik sarp zirveye çarpmadan eteklerindeki güvenli koridordan çevrel geçiş.",
        start_xy=(268000.0, 4454000.0),
        goal_xy=(273000.0, 4454000.0),
        start_alt_offset_m=140.0,
        goal_alt_offset_m=140.0,
    ),

    # -------------------------------------------------------------------------
    # CATEGORY 5: Dar Alanda Dönüş ve Manevra (Confined Turning Maneuvers)
    # -------------------------------------------------------------------------
    BilecikMissionDef(
        id="M21",
        name="Valley_90Deg_Left_Turn",
        category="Confined Turning Maneuvers",
        description="Vadi koridoru içinde 90 derece sola keskin dönüş.",
        purpose="Uçağın dönüş yarıçapının (~350m) kanyon genişliğine sığdığını ve duvara çarpmadığını kanıtlama.",
        start_xy=(258500.0, 4461000.0),
        goal_xy=(256500.0, 4463000.0),
        start_alt_offset_m=140.0,
        goal_alt_offset_m=140.0,
        start_heading_deg=0.0,
    ),
    BilecikMissionDef(
        id="M22",
        name="Valley_90Deg_Right_Turn",
        category="Confined Turning Maneuvers",
        description="Vadi kavşağında 90 derece sağa dönüş yaparak yan vadiye giriş.",
        purpose="Ana vadiden yan dere yatağına 90 derecelik sağ dönüşle kusursuz geçiş.",
        start_xy=(258500.0, 4461000.0),
        goal_xy=(261000.0, 4463000.0),
        start_alt_offset_m=140.0,
        goal_alt_offset_m=150.0,
        start_heading_deg=0.0,
    ),
    BilecikMissionDef(
        id="M23",
        name="Canyon_180Deg_Turnaround",
        category="Confined Turning Maneuvers",
        description="Geniş vadi çanağında 180 derece U-dönüşü yaparak geri dönme.",
        purpose="Giriş doğrultusunun tersine dönerek güvenli manevra tamamlama (`180_Deg_Turn`).",
        start_xy=(258500.0, 4462000.0),
        goal_xy=(258500.0, 4460500.0),
        start_alt_offset_m=150.0,
        goal_alt_offset_m=150.0,
        start_heading_deg=0.0,
    ),
    BilecikMissionDef(
        id="M24",
        name="S_Curved_Canyon_Slalom",
        category="Confined Turning Maneuvers",
        description="S-şeklindeki kanyon kıvrımında ardışık sol ve sağ dönüşlerle slalom.",
        purpose="Ardışık zıt yönlü dönüşlerin (`LEFT_TURN` + `RIGHT_TURN`) süreklilik ve yumuşaklığı.",
        start_xy=(258000.0, 4463000.0),
        goal_xy=(259000.0, 4467000.0),
        start_alt_offset_m=140.0,
        goal_alt_offset_m=140.0,
        start_heading_deg=0.0,
    ),
    BilecikMissionDef(
        id="M25",
        name="Descending_Left_Turn_Into_Basin",
        category="Confined Turning Maneuvers",
        description="Yüksek yamaçtan vadi çanağına doğru dönerken aynı anda alçalma.",
        purpose="Kombine dönüş (`DESCENDING_LEFT_TURN`) primitifinin 3B arazi üzerinde doğrulanması.",
        start_xy=(262000.0, 4466000.0),
        goal_xy=(259000.0, 4468000.0),
        start_alt_offset_m=160.0,
        goal_alt_offset_m=130.0,
        start_heading_deg=315.0,
    ),

    # -------------------------------------------------------------------------
    # CATEGORY 6: Dalgalı Arazi Takibi ve Uzun Seyir (Rolling Terrain & Long Range)
    # -------------------------------------------------------------------------
    BilecikMissionDef(
        id="M26",
        name="Rolling_Hills_Terrain_Following_3km",
        category="Rolling Terrain & Long Range",
        description="Doğu platosunun dalgalı tepeleri üzerinde 3 km arazi takibi.",
        purpose="Tepeler ve çukurlar arasında yumuşak irtifa adaptasyonu (rollercoaster).",
        start_xy=(272000.0, 4458000.0),
        goal_xy=(275000.0, 4458000.0),
        start_alt_offset_m=140.0,
        goal_alt_offset_m=140.0,
        start_heading_deg=90.0,
    ),
    BilecikMissionDef(
        id="M27",
        name="Multi_Valley_Rollercoaster_Transit",
        category="Rolling Terrain & Long Range",
        description="Birbirini izleyen iki vadi tabanı ve bir ara tepe üzerinden 3.5 km uçuş.",
        purpose="Arazi yükselip alçalırken dinamik tırmanma ve alçalma oranlarının test edilmesi.",
        start_xy=(268000.0, 4464000.0),
        goal_xy=(272000.0, 4466000.0),
        start_alt_offset_m=140.0,
        goal_alt_offset_m=140.0,
    ),
    BilecikMissionDef(
        id="M28",
        name="Cross_Country_Transit_5km",
        category="Rolling Terrain & Long Range",
        description="Vadiden başlayıp doğu dağlık alanına uzanan 5 km uzun menzilli görev.",
        purpose="Uzun rotada arama algoritmasının performans ve düğüm bütçesi dayanıklılığı.",
        start_xy=(259000.0, 4458000.0),
        goal_xy=(264000.0, 4458000.0),
        start_alt_offset_m=140.0,
        goal_alt_offset_m=150.0,
        start_heading_deg=90.0,
    ),
    BilecikMissionDef(
        id="M29",
        name="Plateau_Cliff_Edge_Parallel_Run",
        category="Rolling Terrain & Long Range",
        description="Kanyon falez hattına paralel şekilde uçurum kenarından 3.5 km seyir.",
        purpose="Yatayda tek taraflı sarp yamaç varken stabil hat tutma ve tampon güvenliği.",
        start_xy=(261000.0, 4452000.0),
        goal_xy=(261000.0, 4455500.0),
        start_alt_offset_m=140.0,
        goal_alt_offset_m=140.0,
        start_heading_deg=0.0,
    ),
    BilecikMissionDef(
        id="M30",
        name="Complex_Tributary_Network_Transit",
        category="Rolling Terrain & Long Range",
        description="Tali vadiler ağından ana kanyona bağlanan 4.5 km karmaşık rota.",
        purpose="Engebeli ve çok yönlü topoğrafyada optimum vadiler rotasını keşfetme kabiliyeti.",
        start_xy=(266000.0, 4469000.0),
        goal_xy=(260000.0, 4467000.0),
        start_alt_offset_m=150.0,
        goal_alt_offset_m=130.0,
    ),
]
