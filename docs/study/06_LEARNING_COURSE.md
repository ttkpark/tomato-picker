# 인지 → 행동 학습 과정: 손목 카메라가 줄기를 보고 자르기까지
> 책(도서관) → 논문 완독 → 오픈소스 완독 → 이 로봇에서 해보기. 한 모듈 = 한 질문.

00~05가 "무엇이 있는가"의 지도라면, 이 문서는 **어떤 순서로 무엇을 읽고 무엇을 손으로 해 봐야
그 지도가 이해되는가**다. 이미지 처리·OS 수준(드라이버, 프레임 버퍼, 필터)은 일부러 뺐다.

---

## 0. 한 장면을 일곱 질문으로 쪼갠다

"D405가 줄기를 보고 가위가 자른다" 사이에는 끊을 수 있는 고리가 일곱 개 있다.
각 모듈은 고리 하나를 맡고, 고리마다 **이 저장소에 이미 그 답을 구현한 코드**가 있다.

```
 RGB 화소 (u,v) + 깊이 d
   │ M1 이 화소는 카메라 앞 공간의 어느 점인가?        (카메라 모델·깊이 센서·역투영)
   ▼
 카메라 좌표 P_cam (X,Y,Z)
   │ M2 그 점은 팔 기준으로 어디인가?                    (강체변환·TF·손-눈 보정)
   ▼
 팔 좌표 P_base
   │ M3 화면에서 어느 화소들이 '줄기'인가?              (분할 + RGB와 깊이를 합치기)
   ▼
 줄기 점군
   │ M4 줄기의 어디를, 어느 방향에서 자르나?            (점군 기하·절단점·접근 자세)
   ▼
 목표 자세 T_target (위치 + 방향)
   │ M5 손끝을 거기로 어떻게 보내나? 5축이면 다 되나?   (기구학·역기구학·궤적)
   ▼
 관절각 q
   │ M6 보정이 틀렸을 때 어떻게 그래도 닿나?           (비주얼 서보잉·더듬기·폐루프)
   ▼
 절단
     M7 (선택) 위 전부를 시연에서 한 번에 배울 수 있나?  (모방학습: ACT·Diffusion Policy)
```

**핵심 직관 하나만 먼저**: RGB는 "무엇인가"를, 깊이는 "어디인가"를 안다. 둘을 합치는 방법은
거의 언제나 같다 — **RGB에서 화소를 고르고(M3), 그 화소들의 깊이로 3D 점을 만든 뒤(M1),
그 점들의 모양으로 자를 곳을 정한다(M4).** 나머지는 그 점을 팔 언어로 옮기는 일(M2·M5)과
틀렸을 때 고치는 일(M6)이다.

---

## 1. 교재 — 경희대 도서관에서 빌릴 것

⚠ **경희대 수업계획서는 외부에 공개돼 있지 않고, 도서관 검색도 로그인 뒤에 있다**(2026-10-22 확인).
아래는 국내외 로봇공학·컴퓨터비전 과목에서 표준으로 쓰는 교재다. 소장 여부는 직접 확인하라:
- 소장 검색: <https://lib.khu.ac.kr> → 없으면 **희망도서 신청 / 상호대차**. Springer·Cambridge 책은
  전자책(SpringerLink)으로 학내망에서 열리는 경우가 많다.
- 교과목 추천도서: 학교 포털의 **수업계획서 조회**에서 기계공학과·전자공학과·컴퓨터공학과의
  "로봇공학", "컴퓨터비전", "로봇비전", "메카트로닉스" 계열 과목을 검색해 교재 칸을 보라.
  거기 나온 책이 아래 표와 겹치면 그 과목 강의자료를 같이 쓰면 된다.

| # | 책 | 이 과정에서의 역할 | 읽을 부분 |
|---|---|---|---|
| **B1** | Peter Corke, *Robotics, Vision and Control: Fundamental Algorithms in Python*, 3rd ed., Springer 2023 | **척추.** 이 질문(보고 → 움직이기)을 정확히 한 권으로 다루는 유일한 교과서. 코드가 붙어 있다 | 2장(자세 표현), 7·8장(팔 기구학·속도), 13장(영상 형성·카메라 보정), 14장(여러 영상), **15·16장(시각 기반 제어)** |
| **B2** | Kevin Lynch & Frank Park(박종우), *Modern Robotics*, Cambridge 2017 — **무료 PDF** modernrobotics.org | 강체변환·기구학의 수학. SE(3)·스크류를 제대로 | 3장(강체운동), 4·5·6장(순·속도·역기구학), 9장(궤적), 12장(파지) |
| **B3** | Richard Szeliski, *Computer Vision: Algorithms and Applications*, 2nd ed., 2022 — **무료 PDF** szeliski.org/Book | 비전 전반의 참고서 | 2장(영상 형성), 4장(모델 피팅·RANSAC), 6장(인식·분할), 11.1절(카메라 보정), 12장(깊이 추정) |
| **B4** | Hartley & Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge 2004 | 카메라 기하의 원전. 막힐 때 찾아보는 책 | 6장(카메라 모델), 7장(P 행렬 추정), 4장(DLT·RANSAC) |
| **B5** | Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, 2nd ed., Wiley 2020 | 국내 로봇공학 과목 단골 교재. 기구학과 **시각 기반 제어 장**이 짧고 명료 | 기구학 장들 + Vision-Based Control 장 |
| **B6** | Siciliano & Khatib (eds), *Springer Handbook of Robotics*, 2nd ed., 2016 | 분야별 요약. 각 장 끝 참고문헌이 논문 지도 | "Visual Servoing", "3-D Vision for Navigation and Grasping", "Grasping", "Robotics in Agriculture and Forestry" 장 |
| **B7** | 오일석, 『컴퓨터 비전』, 한빛아카데미 2014 | 한국어로 개념 잡기(B3 대신 먼저 보기 좋다) | 기하·3차원 비전 부분 |
| **B8** | 표윤석·임태훈, 『ROS 2로 시작하는 로봇 프로그래밍』, 루비페이퍼 | TF·토픽·노드 — ros2/ 계통을 읽기 위한 최소한 | TF2, 메시지·토픽 장 |

**고르는 법**: 시간이 없으면 **B1 하나 + B2의 3~6장**이면 M1~M6이 다 덮인다. 나머지는 막힐 때 찾는 사전이다.

---

## 2. 로봇팔을 움직이는 교재

팔을 움직이는 건 네 층이다: **기구학**(자세 계산) → **궤적·계획**(경로) → **제어**(따라가기) → **파지**(물체 다루기).

### 먼저 볼 것 — 하나만 고르면

| 책 | 왜 |
|---|---|
| **Lynch & Park, *Modern Robotics*** (Cambridge 2017) | 무료 PDF + **강의 영상**(Coursera, modernrobotics.org). 짧은 영상 단위로 쪼개져 있어 혼자 공부하기 가장 좋다. 팔 움직임 전체를 덮는다: 3장 강체운동, 4장 순기구학, 5장 속도·야코비안, 6장 역기구학, 8장 동역학, 9장 궤적, 10장 운동계획, 11장 제어, 12장 파지. 책과 1:1 대응하는 코드 라이브러리도 있다. |

### 같은 내용을 다른 각도로 (국내 수업 교재로 흔한 것)

| 책 | 특징 |
|---|---|
| **John Craig, *Introduction to Robotics: Mechanics and Control*** (4판, Pearson) | 국내 "로봇공학" 수업에서 가장 흔한 교재. D-H 표기 기반, 예제 위주라 손계산 연습에 좋다. |
| **Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*** (2판, Wiley 2020) | 기구학~제어가 짧고 명료. **시각 기반 제어 장**이 있어 M6과 이어진다. |
| **Siciliano, Sciavicco, Villani, Oriolo, *Robotics: Modelling, Planning and Control*** (Springer 2009) | 위 두 권보다 깊다. 궤적 계획·힘 제어 장이 특히 좋다. |

### 경로 계획 — 장애물(줄기·와이어) 피해 가기

| 책 | 특징 |
|---|---|
| **Steven LaValle, *Planning Algorithms*** (Cambridge 2006, **무료** lavalle.pl) | MoveIt 2가 쓰는 OMPL(RRT·PRM)의 원전. 5장(표본 기반 계획)이 핵심. |
| **Choset 외, *Principles of Robot Motion*** (MIT Press 2005) | 같은 주제를 더 교과서적으로. |

### 더 깊이 — 필요해지면

| 책 | 특징 |
|---|---|
| **Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*** (CRC 1994, **무료 PDF**) | Modern Robotics의 수학적 원조. 파지 장이 엄밀. |
| **Matthew Mason, *Mechanics of Robotic Manipulation*** (MIT Press 2001) | 밀기·쥐기·마찰 — "왜 미끄러지나"를 이해할 때. |
| **Corke, *Robotics, Vision and Control*** (3판) | 위 B1. 7~9장이 팔, 전부 파이썬으로 바로 돌려볼 수 있다. |

### 이 로봇 기준 추천 순서

1. **Modern Robotics 3~6장 + 강의 영상** → [`kinematics.py`](../../src/tomato_picker/hardware/kinematics.py)를 나란히 읽는다. SO-101이 5축이라 왜 제자리 yaw가 안 되는지가 5·6장(야코비안·특이점)에서 설명된다.
2. **9장(궤적)** → 프리셋 재생·보간([`presets.py`](../../src/tomato_picker/hardware/presets.py))과 비교.
3. **11장(제어)** + Spong 제어 장 → STS3215 서보가 내부에서 하는 위치 제어와, 우리가 바깥에서 하는 "목표각을 어떻게 줄까"의 경계.
4. 줄기·와이어 사이로 들어가야 할 때 → LaValle 5장 + MoveIt 2.

무료로 바로 시작 가능: Modern Robotics, LaValle, Murray-Li-Sastry. Craig·Spong은 도서관 소장 확인.

---

## 3. 논문 읽는 법 — "완독"의 정의

S. Keshav, "How to Read a Paper", *ACM SIGCOMM CCR* 2007 (2쪽, 무료)의 3회 읽기를 쓴다.
1. **1회(10분)**: 제목·초록·그림·결론. "이 논문이 푸는 문제를 한 문장으로" 쓸 수 있으면 통과.
2. **2회(1시간)**: 식은 건너뛰고 방법·실험을 따라간다. 표/그래프의 축을 다 읽는다.
3. **3회(반나절~)**: **저자가 한 것을 내가 다시 만든다고 가정하고** 읽는다 — 식을 직접 유도하거나
   코드가 있으면 돌린다. 숨은 가정(카메라 위치, 조명, 실패 사례를 어떻게 셌는지)을 적는다.

완독 노트는 논문마다 5줄로 남긴다: **문제 / 입력→출력 / 핵심 아이디어 / 숨은 가정 / 우리 로봇에선?**
마지막 줄이 이 과정의 목적이다.

⚠ **출처를 확인하고 읽어라.** 아래 목록은 저자·학회·연도를 확인한 것만 넣었다.
`02_PAPERS_CATALOG.md`는 2026-10-24에 한 편씩 검증해 고쳤다(없는 논문 13편 교체, 9편 서지·수치 정정).
그래도 *(2차 인용)* 표시 수치는 원문에서 다시 확인하고, **DOI를 열어** 읽어라.
IEEE Xplore·ScienceDirect·Wiley 원문은 경희대 도서관 전자자원(학내망/원격접속)으로 연다.

---

## 4. 모듈별 과정

각 모듈 = **질문 → 왜 어려운가(이 로봇에서 실제로 겪은 것) → 책 → 논문 → 코드 완독 → 해보기 → 졸업 질문.**
졸업 질문에 **종이 한 장으로** 답할 수 있으면 다음 모듈로 간다.

### M1. 화소 하나는 공간의 어느 점인가 — 카메라 모델과 깊이 센서 (2주)

- **핵심 식**: 핀홀 역투영 `X=(u−cx)·Z/fx, Y=(v−cy)·Z/fy, Z=d` (`01_SPATIAL_PERCEPTION.md` §1).
  식은 쉽다. 어려운 건 **d가 얼마나 믿을 만한가**다.
- **왜 어려운가**: D405는 투광기 없는 스테레오라 **무늬 없는 면·가는 물체·경계에서 깊이가 흔들리거나
  빈다**. 줄기 2~5mm는 그 흔들림보다 가늘다(`docs/개발일지-2026-09-11.md` "거리를 맞힐 수 있다"는
  틀린 전제). Astra는 구조광이라 60cm 안쪽을 못 보고, 컬러와 깊이가 정렬돼 있지 않다.
- **책**: B1 13장 · B3 2장, 12장 앞부분 · B4 6장
- **논문(완독)**
  1. Z. Zhang, "A Flexible New Technique for Camera Calibration", *IEEE TPAMI* 2000 — 바둑판으로 K를 푸는 법. 지금도 OpenCV가 이걸 쓴다.
  2. L. Keselman et al., "Intel RealSense Stereoscopic Depth Cameras", *CVPR Workshops* 2017 (arXiv 1705.05548) — 스테레오 깊이 오차가 **거리²에 비례**하는 이유.
- **코드 완독**
  - librealsense `include/librealsense2/rsutil.h` — `rs2_deproject_pixel_to_point` / `rs2_project_point_to_pixel`. 왜곡 모델 포함 200줄 남짓.
  - 이 저장소 [`hardware/depth_camera.py`](../../src/tomato_picker/hardware/depth_camera.py) — 단위·유효거리·정렬을 **meta.json에서 읽는** 이유(D405 0.1mm vs Astra 1mm).
- **해보기**
  - D405 앞에 자를 세우고 7·15·30·50cm에서 같은 점의 깊이를 100프레임 모아 **표준편차 vs 거리** 그래프를 그린다. Keselman의 거리² 법칙이 맞는지 본다.
  - 같은 실험을 면봉(가는 표적)으로. 경계 화소의 "날아다니는 점(flying pixel)"을 직접 찾는다.
- **졸업 질문**: 줄기 하나의 깊이를 **한 화소로 읽으면 왜 안 되는가**, 그럼 어떻게 읽나?

### M2. 그 점은 팔 기준으로 어디인가 — 강체변환·TF·손-눈 보정 (2주)

- **핵심 식**: `P_base = T_base_tool(q) · T_tool_cam · P_cam`. 앞은 기구학(M5)이 주고,
  가운데 `T_tool_cam`은 **아무도 모르니 실측에서 푼다** — 그게 손-눈 보정 `AX = XB`다.
- **왜 어려운가**: 이 로봇은 손-눈 보정 잔차가 44mm에서 안 내려갔다(`ros2/tools/visual_servo.py` 머리말,
  메모 "평행이동 실측의 3~5배"). 최소자승은 입력이 쓰레기여도 답을 낸다 — **잔차를 읽는 법**이 이 모듈의 핵심.
- **책**: B2 3장(SE(3), 지수좌표) · B1 2장 · B8 TF2 장
- **논문(완독)**
  1. R. Tsai & R. Lenz, "A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration", *IEEE Trans. Robotics and Automation* 1989 — 원조.
  2. F. Park & B. Martin, "Robot Sensor Calibration: Solving AX=XB on the Euclidean Group", *IEEE TRA* 1994 — B2 저자의 해법. 회전을 리 군에서 푼다.
  3. T. Foote, "tf: The Transform Library", *IEEE TePRA* 2013 — 좌표계가 **시간**을 가진다는 것(왜 HTTP 폴링으로는 TF가 안 맞는지).
- **코드 완독**
  - OpenCV `modules/calib3d/src/calibration_handeye.cpp` — `calibrateHandEye`가 Tsai·Park·Horaud·Andreff·Daniilidis 다섯 방법을 한 파일에 구현.
  - 이 저장소 [`hardware/handeye.py`](../../src/tomato_picker/hardware/handeye.py)(424줄, numpy만) — `fixed`/`on_arm` 두 식. 위 논문을 읽은 뒤 읽으면 줄마다 대응이 보인다.
- **해보기**
  - `python tools/handeye_check.py` 를 돌리고, 테스트 하나를 골라 **입력에 1°/5mm 잡음을 넣으면 잔차가 얼마나 커지는지** 직접 바꿔 본다.
  - 같은 데이터를 OpenCV `calibrateHandEye`의 5개 방법으로 풀어 `handeye.py`와 비교한다(PC에서, 젯슨 불필요).
- **졸업 질문**: 잔차 44mm가 나왔다. **회전이 틀린 것과 평행이동이 틀린 것을 데이터로 어떻게 구별하나?**

### M3. 화면에서 어느 화소가 줄기인가 — 분할, 그리고 RGB+깊이 합치기 (2주)

- **핵심**: 2D 인스턴스 분할로 마스크를 얻고 → 마스크 안 화소만 역투영(M1) → 깊이로 **뒤 배경을 잘라낸다**.
  "RGB와 깊이를 같이 쓴다"의 실체는 대부분 이것이다.
- **왜 어려운가**: 가는 표적은 마스크 경계가 곧 깊이 경계라 배경 깊이가 섞인다. 또
  **가장 가까운 것은 자기 손가락이다**(개발일지 9/11 틀린 전제 ②).
- **책**: B3 6장(인식·분할) · B7 해당 장
- **논문(완독)**
  1. K. He et al., "Mask R-CNN", *ICCV* 2017 (arXiv 1703.06870) — 인스턴스 분할의 기본형. YOLO-seg도 이 문제 정의를 따른다.
  2. **I. Sa et al., "Peduncle Detection of Sweet Pepper for Autonomous Crop Harvesting — Combined Color and 3-D Information", *IEEE RA-L* 2017 (arXiv 1701.08608)** — 이 과정의 **중심 논문**. 제목 그대로 "색 + 3D로 꼭지를 찾는다". 특징 설계·평가 방식을 통째로 따라 할 만하다.
  3. A. Kirillov et al., "Segment Anything", *ICCV* 2023 (arXiv 2304.02643) — 라벨 없이 클릭 한 번으로 마스크. `click_server`의 "사람이 줄기를 찍는다"와 바로 붙는다.
- **코드 완독**
  - 이 저장소 [`tomato_perception/yolo_seg.py`](../../ros2/src/tomato_perception/tomato_perception/yolo_seg.py) + [`detect_node.py`](../../ros2/src/tomato_perception/tomato_perception/detect_node.py) — 마스크 → 깊이 거절 → 3D까지.
  - Laboro Tomato 데이터셋(github.com/laboroai/LaboroTomato)의 주석 형식 — 무엇을 "한 개체"로 라벨링했는지.
- **해보기**
  - 같은 줄기 사진에서 **마스크 전체 깊이의 중앙값 / 최솟값 / 평균**을 비교한다. 어느 것이 손가락·배경에 오염되는지.
  - `python ros2/tools/yolo_seg_check.py` 를 읽고 돌린다.
- **졸업 질문**: 분할 마스크가 완벽해도 **3D 점이 틀릴 수 있는 이유 두 가지**는?

### M4. 줄기의 어디를 어느 방향에서 자르나 — 점군 기하 (2주)

- **핵심**: 줄기 점군 → 잡음 제거 → **직선(축) 피팅(PCA 또는 RANSAC)** → 과실 쪽에서 일정 거리 위를
  절단점으로 → 가위 날이 축에 **수직**이 되도록 접근 방향을 정한다. 그게 "6-DoF 절단 자세"다.
- **왜 어려운가**: 줄기 축이 카메라 쪽을 향하면 점이 한 덩어리로 뭉쳐 축이 안 나온다.
  5축 팔은 모든 방향에서 접근할 수 없다(M5) — **"자를 수 있는 자세"와 "팔이 갈 수 있는 자세"의 교집합**을 찾는 문제다.
- **책**: B3 4장(RANSAC·강건 피팅) · B2 12장(파지·접촉) · B6 "Grasping" 장
- **논문(완독)**
  1. M. Fischler & R. Bolles, "Random Sample Consensus", *Comm. ACM* 1981 — RANSAC 원전. 8쪽.
  2. **T. Yoshida, T. Fukao, T. Hasegawa, "Fast Detection of Tomato Peduncle Using Point Cloud with a Harvesting Robot", *J. Robotics and Mechatronics* 30(2), 2018** 및 **"Cutting Point Detection Using a Robot with Point Clouds for Tomato Harvesting", *JRM* 32(2), 2020** — **토마토 꼭지 절단점을 RGB-D 점군에서 찾는** 바로 그 문제. JRM은 공개 접근(fujipress.jp).
  3. A. ten Pas, M. Gualtieri, K. Saenko, R. Platt, "Grasp Pose Detection in Point Clouds", *IJRR* 2017 — 점군에서 파지 자세 후보를 만들고 거르는 일반론. 절단 자세도 같은 틀이다.
- **코드 완독**
  - Open3D `PointCloudSegmentation.cpp`(SegmentPlane — RANSAC 구현) · `create_from_rgbd_image`.
  - 이 저장소 [`tomato_perception/stem_cut.py`](../../ros2/src/tomato_perception/tomato_perception/stem_cut.py) — 절단 자세·깊이 평활화·비직교(shear) 거절. 검사 [`stem_cut_check.py`](../../ros2/tools/stem_cut_check.py)를 먼저 읽으면 **무엇을 막으려 했는지**가 보인다.
- **해보기**
  - Open3D로 D405 한 프레임을 점군으로 만들고, 줄기 마스크 점만 남겨 PCA 축을 그려 본다. 카메라를 줄기 축에 대해 0°/45°/80° 돌려 찍으며 **축 추정이 언제 무너지는지** 기록한다.
- **졸업 질문**: 줄기 축 추정의 **불확실성**을 숫자로 내려면? 그 숫자로 "이 프레임은 자르지 마라"를 어떻게 정하나?

### M5. 손끝을 거기로 어떻게 보내나 — 기구학과 5축의 한계 (2주)

- **핵심**: 순기구학 `T(q)`, 역기구학 `q = IK(T)`, 야코비안 `ẋ = J(q)q̇`. 5축이라 6-DoF 목표 중 하나는 **포기**해야 한다.
- **왜 어려운가**: SO-101은 5축이라 제자리 yaw가 일반적으로 불가능하고(집게가 바닥을 볼 때만 roll=yaw),
  STS3215는 0.088°/틱보다 작은 지령이 물리적으로 0이다. 중력 처짐이 영점 오차가 된다.
- **책**: B2 4·5·6·9장(이 모듈의 본체) · B1 7·8장 · B5 기구학 장
- **논문(완독)**
  1. D. Coleman et al., "Reducing the Barrier to Entry of Complex Robotic Software: a MoveIt! Case Study", *JOSER* 2014 — 계획기·IK·충돌검사를 어떻게 나눴는가.
  2. S. Macenski et al., "Robot Operating System 2: Design, Architecture, and Uses in the Wild", *Science Robotics* 2022 — ros2/ 계통을 왜 그렇게 짰는지의 배경.
- **코드 완독**
  - NxRLab `ModernRobotics` 파이썬 라이브러리(`modern_robotics/core.py`) — B2 식이 거의 한 줄씩 함수다. 한 파일, 완독 가능.
  - 이 저장소 [`hardware/kinematics.py`](../../src/tomato_picker/hardware/kinematics.py)(255줄, `math`만) — 위 라이브러리와 나란히 놓고 읽는다.
- **해보기**
  - `python tools/arm_cartesian_check.py` 통과 후, `kinematics.py`의 순기구학을 `ModernRobotics.FKinSpace`로 다시 짜서 **같은 q에서 같은 T**가 나오는지 assert로 확인한다.
  - 줄기 축 방향 몇 개를 주고 "5축으로 수직 절단이 가능한가"를 판정하는 함수를 짜 본다(M4와 이어짐).
- **졸업 질문**: 6-DoF 절단 자세가 주어졌을 때 5축 팔이 **무엇을 포기하는 게 가장 덜 나쁜가**? 왜?

### M6. 보정이 틀려도 닿게 하려면 — 비주얼 서보잉과 더듬기 (2주)

- **핵심**: 열린 루프("계산하고 간다")는 M1~M5의 오차가 전부 쌓인다. 닫힌 루프("보면서 간다")는
  **화면에서 목표와 집게 사이 오차를 직접 줄이므로** 보정 오차가 상쇄된다. 이 로봇이 실제로 간 길이다.
- **왜 어려운가**: 화소 오차가 줄어드는데 실패하는 경우 — 가는 표적에서 정합이 **배경을 따라간다**(메모 "Thin target tracking fails").
  깊이 방향은 화면에서 안 보인다 → "60%만 가서 물어보고 8mm 더"(개발일지 9/11).
- **책**: **B1 15·16장(이 과정의 클라이맥스)** · B5 Vision-Based Control 장 · B6 "Visual Servoing" 장
- **논문(완독)**
  1. S. Hutchinson, G. Hager, P. Corke, "A Tutorial on Visual Servo Control", *IEEE TRA* 1996.
  2. F. Chaumette & S. Hutchinson, "Visual Servo Control, Part I: Basic Approaches", *IEEE RA Magazine* 2006 (+ Part II, 2007) — IBVS vs PBVS, 상호작용 행렬. **짧고 결정적**.
  3. D. Morrison, P. Corke, J. Leitner, "Closing the Loop for Robotic Grasping: A Real-time, Generative Grasp Synthesis Approach", *RSS* 2018 (GG-CNN) — 깊이 영상만으로 매 프레임 파지점을 다시 내는 폐루프 파지.
- **코드 완독**
  - github.com/dougsm/ggcnn — 작은 저장소, 완독 가능.
  - Corke의 `machinevision-toolbox-python` / `robotics-toolbox-python`의 시각 서보 예제(B1 15장과 짝).
  - 이 저장소 [`ros2/tools/visual_servo.py`](../../ros2/tools/visual_servo.py)(머리말이 곧 설계 문서: **실측 야코비안**) → [`grasp_probe.py`](../../ros2/tools/grasp_probe.py) → [`stem_grasp.py`](../../ros2/tools/stem_grasp.py).
- **해보기**(⚠ 실기 — `docs/인수인계-2026-09-03.md`를 먼저 읽고, 사람이 보는 앞에서)
  - `visual_servo.py`의 실측 야코비안을 B1 15장의 **해석적 상호작용 행렬**과 비교한다: 같은 자세에서 둘이 얼마나 다른가?
  - 면봉으로 `--to 200` 을 반복하며 **화소 오차 vs 실제 집게-표적 거리(자로 잼)**를 같이 기록한다. 둘이 갈라지는 순간이 이 모듈의 교훈이다.
- **졸업 질문**: IBVS와 PBVS 중 **손-눈 보정 잔차가 큰 이 로봇**에 맞는 것은? 깊이 축은 어떻게 처리하나?

### M7. (선택) 시연에서 한 번에 배우기 — 모방학습 (2주)

- **핵심**: M1~M6을 사람이 설계하는 대신, 리더암 시연(이미 하고 있는 미러링)에서 **영상 → 관절 명령**을 통째로 학습.
  설계형 파이프라인과 **비교 대상**으로 알아 둘 가치가 있다.
- **논문(완독)**
  1. T. Zhao et al., "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware", *RSS* 2023 (ACT, arXiv 2304.13705) — SO-101 같은 저가 팔을 전제로 쓴 논문.
  2. C. Chi et al., "Diffusion Policy", *RSS* 2023 (arXiv 2303.04137).
- **코드 완독**: 젯슨에 깔린 lerobot의 `policies/act/modeling_act.py`.
- **졸업 질문**: 줄기 절단을 모방학습으로 하면 M1~M6의 **어떤 문제가 사라지고 어떤 문제가 남나**?

---

## 5. 큰 그림 — 남들은 어떻게 해 왔나 (과정 중간중간 1편씩)

시스템 전체를 만든 팀들의 논문이다. **M3 전후로 하나, M6 끝나고 하나**씩 읽으면 각 모듈이 전체의 어디인지 잡힌다.

| 순서 | 논문 | 볼 것 |
|---|---|---|
| 1 | C. W. Bac, E. J. van Henten, J. Hemming, Y. Edan, "Harvesting Robots for High-Value Crops: State-of-the-Art Review and Challenges Ahead", *J. Field Robotics* 2014 | 30년간 성공률이 왜 안 올랐나 — 실패 원인의 분류 |
| 2 | H. Yaguchi, K. Nagahama, T. Hasegawa, M. Inaba, "Development of an Autonomous Tomato Harvesting Robot with Rotational Plucking Gripper", *IROS* 2016 | **토마토**, 자르지 않고 비틀어 따기(성공률 ~60%) — 절단과의 맞교환 |
| 3 | C. Lehnert et al., "Autonomous Sweet Pepper Harvesting for Protected Cropping Systems", *IEEE RA-L* 2017 | Sa 2017(M3)을 실제 로봇(Harvey)에 붙인 결과. 흡착 + 절단 |
| 4 | B. Arad et al., "Development of a Sweet Pepper Harvesting Robot", *J. Field Robotics* 2020 (SWEEPER) | 온실 실증, 손목 카메라 · 실패 분석 |
| 5 | Y. Xiong, Y. Ge, P. From, "An Autonomous Strawberry-Harvesting Robot: Design, Development, Integration, and Field Evaluation", *J. Field Robotics* 2020 | 줄기를 **잡고 자르는** 집게, 가림 속 접근 |
| 6 | G. Kootstra et al., "Selective Harvesting Robotics: Current Research, Trends, and Future Directions", *Current Robotics Reports* 2021 (DOI 10.1007/s43154-020-00034-1) | 짧은 최신 개관 |
| 7 | V. Rajendran et al., "Towards Autonomous Selective Harvesting: A Review of Robot Perception, Robot Design, Motion Planning and Control", *J. Field Robotics* 2024 (arXiv 2304.09617) | 이 과정 M1~M6과 같은 축으로 정리된 개관 — 마지막에 읽고 빈칸 점검 |

---

## 6. 일정 예시 (주 6~8시간 기준, 약 14주)

| 주 | 모듈 | 책 | 논문 | 손 |
|---|---|---|---|---|
| 1–2 | M1 | B1 13 · B3 2 | Zhang · Keselman | 깊이 잡음 vs 거리 |
| 3 | (큰 그림) | — | Bac 2014 | — |
| 4–5 | M2 | B2 3 · B1 2 | Tsai-Lenz · Park-Martin · Foote | handeye 잡음 주입·OpenCV 비교 |
| 6–7 | M3 | B3 6 | Mask R-CNN · **Sa 2017** · SAM | 마스크 깊이 통계 |
| 8 | (큰 그림) | — | Lehnert 2017 · Yaguchi 2016 | — |
| 9–10 | M4 | B3 4 · B2 12 | RANSAC · **Yoshida 2018/2020** · GPD | 줄기 축 PCA·각도별 붕괴 |
| 11–12 | M5 | B2 4–6, 9 | MoveIt · ROS 2 | FK 교차검증·5축 가능성 판정 |
| 13–14 | M6 | **B1 15–16** | Hutchinson · Chaumette · GG-CNN | 야코비안 비교·화소 vs 실거리 |
| (+2) | M7 | — | ACT · Diffusion Policy | lerobot ACT 읽기 |
| 마지막 | — | — | Arad · Xiong · Kootstra · Rajendran | 졸업 질문 7개를 한 문서로 |

**마지막 산출물**: 졸업 질문 7개의 답을 이 로봇의 숫자(잔차, 깊이 표준편차, 성공률)로 채운 한 문서.
그게 곧 "이 로봇이 왜 아직 줄기를 못 자르는가 / 무엇을 바꾸면 되는가"의 답이다.
