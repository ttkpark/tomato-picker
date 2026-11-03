# 토마토 수확 로봇 핵심 연구 논문 24편 카탈로그
> 과실 인식 · 3D 공간지각 · 줄기 절단점 · 엔드이펙터 · 필드 시스템

## ⚠ 검증 기록 (2026-10-24)

이 문서의 첫 판(2026-09-21)은 AI가 만든 목록이라 **24편 중 상당수가 없는 논문이거나, 실재하더라도
학술지·연도·수치가 틀려 있었다.** 2026-10-24에 한 편씩 검색해 다음 원칙으로 고쳤다.

- **실재하는 논문만 남긴다.** 찾지 못한 항목은 같은 주제의 **확인된 논문으로 바꿨다**(🔁 표시).
- **수치는 초록·출판사 페이지에서 확인한 것만** 적는다. 확인 못 한 옛 수치는 지웠다.
  남의 논문(리뷰·인용)을 통해서만 본 수치는 *(2차 인용)* 으로 표시했다 — 원문에서 다시 확인하라.
- **"우리 로봇에선"** 줄은 논문의 주장이 아니라 **이 저장소 관점의 해석**이다.

| 표시 | 뜻 | 편수 |
|---|---|---|
| ✅ | 실재 · 서지 정보 맞음(수치만 고침) | 2 |
| ✏️ | 실재하지만 학술지·연도·저자·수치가 틀렸던 것 → 고침 | 9 |
| 🔁 | 찾을 수 없어 같은 주제의 실재 논문으로 교체 | 13 |

읽기 전에 **DOI를 열어 원문을 확인**하라(학내망·도서관 원격접속). 방법은 [`06_LEARNING_COURSE.md`](06_LEARNING_COURSE.md) §3.

---

## 목차
1. [Category A: 과실 검출, 인스턴스 분할 및 숙도 (5편)](#category-a-과실-검출-인스턴스-분할-및-숙도-5편)
2. [Category B: 3D 공간지각, 점군 처리 및 자세 추정 (5편)](#category-b-3d-공간지각-점군-처리-및-자세-추정-5편)
3. [Category C: 줄기(Peduncle) 검출 및 절단점 (5편)](#category-c-줄기peduncle-검출-및-절단점-5편)
4. [Category D: 엔드이펙터와 절단 기구 (4편)](#category-d-엔드이펙터와-절단-기구-4편)
5. [Category E: 비주얼 서보잉 및 필드 시스템 (5편)](#category-e-비주얼-서보잉-및-필드-시스템-5편)

---

## Category A: 과실 검출, 인스턴스 분할 및 숙도 (5편)

### [Paper 01] Deep Fruit Detection in Orchards ✏️
- **저자/연도**: Bargoti, S., & Underwood, J. (2017)
- **학회**: *IEEE ICRA 2017* · arXiv [1610.03677](https://arxiv.org/abs/1610.03677)
  - 고침: 학술지가 아니라 **학회 논문**이다. 같은 저자의 *JFR* 2017 논문은 제목이 다른 분할 논문이다.
- **내용**: 과수원 영상(사과·망고·아몬드)에 **Faster R-CNN**을 적용하고, 데이터 증강·학습 영상 수에 따른 성능을 분석했다. 데이터셋(ACFR Orchard Fruit Dataset)을 공개했다.
- **수치**: F1 — 사과 **0.904**, 망고 **0.908**, 아몬드 **0.775**.
  - 고침: 옛 판의 "세 과일 모두 0.90 이상"은 틀렸다. 아몬드는 영상 한 장에 열매가 1000개가 넘을 만큼 작고 빽빽해서 낮다. "최초" 주장, 정밀도 92%, 앵커 1:1은 확인되지 않아 지웠다(과일 딥러닝 검출은 Sa et al., *DeepFruits*, 2016이 더 앞선다).
- **우리 로봇에선**: **작고 빽빽한 표적일수록 검출기가 약하다**는 숫자 증거다. 줄기는 아몬드보다도 가늘다.

---

### [Paper 02] YOLO-Tomato: A Robust Algorithm for Tomato Detection Based on YOLOv3 ✏️
- **저자/연도**: Liu, G., Nouaze, J. C., Touko Mbouembe, P. L., & Kim, J. H. (2020)
- **학술지**: *Sensors*, 20(7), 2145 · DOI [10.3390/s20072145](https://doi.org/10.3390/s20072145) (공개 접근)
  - 고침: 옛 판의 "Lawal 2021, *Frontiers in Plant Science*"는 틀렸다. Lawal(2021, *Scientific Reports*)은 이름이 비슷한 **다른 논문**이다.
- **내용**: YOLOv3에 **dense 연결**(특징 재사용)을 넣고, 사각 상자 대신 **원형 상자(C-Bbox)**를 써서 NMS의 IoU 계산을 토마토 모양에 맞췄다. 조명 변화·가지/잎 가림·과실 겹침이 대상이다.
- **수치**: AP 93.91%, F1 96.4% *(2차 인용)*.
  - 지움: Mish·SPP, "50% 가림에서 91.2%"는 확인되지 않았다.
- **우리 로봇에선**: 모양에 대한 사전지식(토마토는 둥글다)을 검출기 구조에 넣는 예다. 줄기의 사전지식은 "가늘고 길다"이다.

---

### [Paper 03] An Efficient Tomato-Detection Method Based on Improved YOLOv4-tiny Model in Complex Environment 🔁
- **저자/연도**: Touko Mbouembe, P. L., Liu, G., Sikati, J., Kim, S. C., & Kim, J. H. (2023)
- **학술지**: *Frontiers in Plant Science*, 14, 1150958 · DOI [10.3389/fpls.2023.1150958](https://doi.org/10.3389/fpls.2023.1150958) (공개 접근)
  - 교체 이유: 옛 판의 "Liu, Noh & Shu 2020, CEA 178, 105748(숙도 3단계 YOLOv4-Tiny, Jetson 38.5 FPS)"은 찾을 수 없었다. 같은 모델 계열(YOLOv4-tiny 경량화)의 실재 논문으로 바꿨다. **숙도 분류가 아니라 검출 논문**이다.
- **내용**: YOLOv4-tiny의 백본을 바꾸고, tiny CSP-SPP 블록과 CARAFE 업샘플링을 넥에 넣었다.
- **수치**: 정밀도 96.3%, 재현율 95%, F1 95.6%, mAP(IoU 0.5~0.95) 82.8%, 영상당 1.9 ms.
- **우리 로봇에선**: 젯슨에서 돌릴 경량 검출기를 고를 때 무엇을 바꿔 속도를 얻는지 보는 사례다. 1.9 ms는 데스크톱 GPU 기준이니 젯슨에서는 직접 재야 한다.

---

### [Paper 04] Tomato Fruit Detection and Counting in Greenhouses Using Deep Learning 🔁
- **저자/연도**: Afonso, M., Fonteijn, H., Fiorentin, F. S., Lensink, D., Mooij, M., Faber, N., Polder, G., & Wehrens, R. (2020)
- **학술지**: *Frontiers in Plant Science*, 11, 571299 · DOI [10.3389/fpls.2020.571299](https://doi.org/10.3389/fpls.2020.571299) (공개 접근)
  - 교체 이유: 옛 판의 "Rong et al. 2022, YOLOv7 숙도 검출, *Front. Plant Sci.* 13, 1024479"는 찾을 수 없었다.
- **내용**: 상업 온실에서 **RealSense**로 찍은 영상에 **Mask R-CNN**을 적용해 토마토를 검출하고 셌다. Mask R-CNN이 깊이를 **암묵적으로 배워** 뒤쪽 줄의 열매(배경)를 걸러낸다는 관찰이 있다. 데이터셋 = *Rob2Pheno Annotated Tomato Image Dataset*(4TU.ResearchData).
- **수치**: 원문 표에서 직접 확인할 것(초록에는 "기존 연구와 같거나 낫다"로만 나온다).
- **우리 로봇에선**: **같은 계열 카메라(RealSense) + 실제 온실**이라 가장 가까운 비교 대상이다. "앞줄 열매만 고르기"는 깊이 거절(`detect_node`)과 같은 문제다.

---

### [Paper 05] Laboro Tomato (데이터셋) ✏️
- **출처**: Laboro.AI, GitHub [laboroai/LaboroTomato](https://github.com/laboroai/LaboroTomato) · 미러 [dataset-ninja/laboro-tomato](https://github.com/dataset-ninja/laboro-tomato)
  - 고침: **논문이 아니라 데이터셋**이다. 옛 판의 "Wang et al. 2022, arXiv:2009.05244"는 전혀 다른 논문(적대적 비디오 방어)의 번호였다. 벤치마크 점수 주장도 지웠다.
- **내용**: 온실 토마토 영상, **인스턴스 분할 마스크**. 6클래스 = 크기 2종(일반 `b_`, 방울 `l_`) × 숙도 3단계(빨간 면적 90% 이상 = 완숙 / 30~89% = 반숙 / 0~30% = 미숙).
- **규모**: 804장, 10,610개체, 학습 643 / 시험 161 *(Dataset Ninja 기준)*.
- **라이선스**: CC BY-NC-SA 4.0(비상업) *(Dataset Ninja 기준, 원 저장소에서 다시 확인)*.
- **우리 로봇에선**: 사전학습용이다. **줄기 클래스는 없으니** 줄기 분할에는 직접 라벨링이 필요하다.

---

## Category B: 3D 공간지각, 점군 처리 및 자세 추정 (5편)

### [Paper 06] Fruit Detection, Segmentation and 3D Visualisation of Environments in Apple Orchards (DaSNet-V2) ✏️
- **저자/연도**: Kang, H., & Chen, C. (2020)
- **학술지**: *Computers and Electronics in Agriculture*, 171, 105302 · DOI [10.1016/j.compag.2020.105302](https://doi.org/10.1016/j.compag.2020.105302) · arXiv [1911.12889](https://arxiv.org/abs/1911.12889)
  - 고침: 옛 판의 "*IEEE Trans. Cybernetics* 51(12)"는 틀렸다. **사과 과수원** 논문이고 토마토는 없다. "6D 포즈 추정", "형태 완성 모듈", "중심 오차 8.2 mm"는 이 논문에 없다.
- **내용**: 원스테이지 네트워크 하나로 과실 검출 + 과실 인스턴스 분할 + **가지 의미 분할**을 같이 한다. 경량 백본 LW-net을 쓰고, RGB-D로 결과를 3D로 보여 준다.
- **수치**: (LW-net) 검출 F1 0.844, 과실 분할 mIoU 0.858, 가지 분할 mIoU 0.795. GTX-1080Ti 30 ms, Jetson TX2 265 ms.
- **우리 로봇에선**: 과실과 **가지를 같이** 분할하는 구조다. 줄기·와이어를 장애물로 보려면 이렇게 해야 한다. TX2 265 ms는 젯슨 예산을 가늠하는 기준이 된다.

---

### [Paper 07] Development and Evaluation of Automated Localisation and Reconstruction of All Fruits on Tomato Plants in a Greenhouse Based on Multi-View Perception and 3D Multi-Object Tracking 🔁
- **저자/연도**: Rapado-Rincón, D., van Henten, E. J., & Kootstra, G. (2023)
- **학술지**: *Biosystems Engineering*, 231, 78–91 · DOI [10.1016/j.biosystemseng.2023.06.003](https://doi.org/10.1016/j.biosystemseng.2023.06.003) (공개 접근, CC BY) · arXiv [2211.02760](https://arxiv.org/abs/2211.02760)
  - 교체 이유: 옛 판의 "Sun et al. 2020, *Biosystems Eng.* 198(HSV + 유클리드 군집 + RANSAC 구 피팅)"은 찾을 수 없었다.
- **내용**: 여러 시점에서 본 토마토를 **3D 다중 객체 추적**으로 같은 열매끼리 묶어, 가림을 넘어 한 그루의 열매 전부를 3D로 복원한다. 검출 단계는 Mask R-CNN 마스크로 점군을 잘라 열매마다 **구(sphere)를 피팅해 중심**을 잡는다(후속 MinkSORT 논문의 설명).
- **수치**: 열매 수 추정 최대 오차 5.08%, 추적 정확도 최대 71.47%.
- **우리 로봇에선**: 한 장으로 안 보이면 **여러 장으로 보고 묶는다**는 접근이다. `space_map`의 토마토 찾기가 같은 문제를 원거리에서 푼다.

---

### [Paper 08] Guava Detection and Pose Estimation Using a Low-Cost RGB-D Sensor in the Field 🔁
- **저자/연도**: Lin, G., Tang, Y., Zou, X., Xiong, J., & Li, J. (2019)
- **학술지**: *Sensors*, 19(2), 428 · DOI [10.3390/s19020428](https://doi.org/10.3390/s19020428) (공개 접근)
  - 교체 이유: 같은 저자의 "*CEA* 176, 105654(PointNet++ 기반 구아바/토마토)"는 찾을 수 없었다. 실재하는 이쪽이 내용상 가장 가깝다.
- **내용**: FCN으로 RGB를 과실·가지로 분할 → 점군에서 **유클리드 군집**으로 열매를 하나씩 분리 → 가지를 **3D 선분**으로 재구성 → 열매 중심과 가장 가까운 가지로 **열매 자세**를 정한다(Kinect v2, 437장).
- **수치**: 정밀도 0.983, 재현율 0.948, **자세 오차 23.43° ± 14.18°**, 열매당 0.565 s.
- **우리 로봇에선**: 검출은 잘 돼도 **방향 추정은 20°대로 거칠다**는 실측이다. 줄기 축 추정(M4)의 불확실성을 생각할 때 기준점이 된다.

---

### [Paper 09] Tomato Harvesting Robotic System Based on Deep-ToMaToS: Deep Learning Network Using Transformation Loss for 6D Pose Estimation of Maturity Classified Tomatoes with Side-Stem ✏️
- **저자/연도**: Kim, J., Pyo, H., Jang, I., Kang, J., Ju, B.-K., & Ko, K.-E. (2022)
- **학술지**: *Computers and Electronics in Agriculture*, 201, 107300
  - 고침: 옛 판의 "*IEEE Access* 11, 2023"은 틀렸다. "언리얼 엔진 합성 10만 장", "ADD 84.6%"는 확인되지 않아 지웠다.
- **내용**: 토마토의 **숙도 3단계 분류**와 **6D 자세(3D 위치 + 3D 회전)**를 한 네트워크에서 동시에 추정한다. 곁줄기(side-stem)가 붙은 토마토가 대상이다. 변환 손실(transformation loss)을 쓴다.
- **수치**: 원문에서 확인할 것.
- **우리 로봇에선**: **국내 연구진**의 토마토 6D 자세 논문이라 저자에게 물어볼 수도 있다. 자세를 학습으로 바로 내는 방식이라, 기하로 푸는 `stem_cut`과 비교해 볼 만하다.

---

### [Paper 10] 3D Pose Estimation of Tomato Peduncle Nodes Using Deep Keypoint Detection and Point Cloud 🔁
- **저자/연도**: Ci, J., Wang, X., Rapado-Rincón, D., Burusa, A. K., & Kootstra, G. (2024)
- **학술지**: *Biosystems Engineering*, 243, 57–69 · arXiv [2311.04699](https://arxiv.org/abs/2311.04699)
  - 교체 이유: 옛 판의 "Magrini et al. 2020, *Autonomous Robots* 44(6)(스테레오 + EKF 추적)"은 찾을 수 없었다.
- **내용**: RGB에서 꼭지 마디의 **해부학적 키포인트 4개**를 검출하고, 점군 정보를 합쳐 꼭지와 **주줄기 사이의 3D 각도**를 낸다.
- **수치**: 검출 AP@0.5 0.96, 키포인트 PDJ@0.2 94.31%, 각도 MAE 위쪽 11.38° / 아래쪽 9.93°.
- **우리 로봇에선**: "2D에서 점을 고르고 → 깊이로 3D를 만든다"(06 과정 M3→M1)를 **줄기에** 그대로 적용한 예다. 각도 오차 10°가 현재 수준이다.

---

## Category C: 줄기(Peduncle) 검출 및 절단점 (5편)

### [Paper 11] Fast Detection of Tomato Peduncle Using Point Cloud with a Harvesting Robot 🔁
- **저자/연도**: Yoshida, T., Fukao, T., & Hasegawa, T. (2018)
- **학술지**: *Journal of Robotics and Mechatronics*, 30(2), 180–186 · DOI [10.20965/jrm.2018.p0180](https://doi.org/10.20965/jrm.2018.p0180) (공개 접근)
  - 교체 이유: 옛 판의 "Chen et al. 2021, *CEA* 182, 106008(Mask R-CNN + 꽃받침 위 10~15 mm, 오차 4.7 mm)"은 찾을 수 없었다.
- **내용**: RGB-D 점군에서 꼭지를 찾는다. 계산을 줄이려고 **두 해상도의 복셀화**를 하고, 꼭지가 만족해야 할 **세 조건으로 에너지 함수**를 세워 최소화해 절단점을 정한다. 실제 농장에서 꼭지를 잘라 수확했다(Ritsumeikan University · Denso).
- **우리 로봇에선**: **토마토 꼭지 절단점을 RGB-D 점군에서 찾는**, 바로 우리 문제다. 06 과정 M4의 중심 논문이다.

---

### [Paper 12] Cutting Point Detection Using a Robot with Point Clouds for Tomato Harvesting 🔁
- **저자/연도**: Yoshida, T., Fukao, T., & Hasegawa, T. (2020)
- **학술지**: *Journal of Robotics and Mechatronics*, 32(2), 437–444 (공개 접근, fujipress.jp)
  - 교체 이유: 옛 판의 "Rong et al. 2023, *CEA* 205, 107611(Zhang-Suen 골격선)"은 찾을 수 없었다.
- **내용**: Paper 11의 후속으로 절단점 검출을 다룬다. *(초록만 확인했다 — 원문을 읽고 이 칸을 채울 것.)*
- **우리 로봇에선**: 11과 이어 읽으면 2년 동안 무엇을 고쳤는지가 보인다.

---

### [Paper 13] A Peduncle Detection Method of Tomato for Autonomous Harvesting 🔁
- **저자/연도**: Rong, J., Dai, G., & Wang, P. (2021 온라인, 2022 권호)
- **학술지**: *Complex & Intelligent Systems*, 8(4), 2955–2969 · DOI [10.1007/s40747-021-00522-7](https://doi.org/10.1007/s40747-021-00522-7)
  - 교체 이유: 옛 판의 "Li et al. 2021, *RA-L* 6(4)(PointNet++ 줄기 국소좌표계)"는 찾을 수 없었다.
- **내용**: 멀리서는 **YOLOv4-Tiny**로 찾고, 가까이서는 **YOLACT++**로 꼭지를 분할한다. 꼭지 마스크에 **최소자승 곡선을 피팅**해 곡선 위 키포인트 3개를 잡고, 기하 모델로 꼭지 자세를 낸다.
- **수치**: YOLACT++ mAP 73.1, 프레임당 0.109 s. 30회 시험 평균 오차 yaw 4.98°, pitch 4.75°.
- **우리 로봇에선**: **2D 마스크에서 모양을 먼저 잡고 깊이를 붙인다** — `stem_cut`과 같은 방향이다. "원거리 검출 → 근거리 분할" 두 단계는 Astra → D405 역할 분담과 같다.

---

### [Paper 14] Peduncle Detection of Sweet Pepper for Autonomous Crop Harvesting — Combined Color and 3-D Information 🔁
- **저자/연도**: Sa, I., Lehnert, C., et al. (2017)
- **학술지**: *IEEE Robotics and Automation Letters*, 2(2), 765–772 · arXiv [1701.08608](https://arxiv.org/abs/1701.08608)
  - 교체 이유: 옛 판의 "Taqi et al. 2023, *Front. Plant Sci.* 14, 1148821"은 찾을 수 없었다. 이 논문으로 바꿨다.
- **내용**: RGB-D의 **색 + 기하** 특징으로 지도학습해 3D에서 꼭지를 찾는다. 주석 단 3D 데이터를 공개했다.
- **수치**: 노지 파프리카에서 꼭지 검출 AUC 0.71.
- **우리 로봇에선**: 06 과정 M3의 **중심 논문**이다. AUC 0.71은 "꼭지 찾기는 열매 찾기보다 훨씬 어렵다"는 기준선이다.

---

### [Paper 15] Collision-Free Path Planning for a Guava-Harvesting Robot Based on Recurrent Deep Reinforcement Learning 🔁
- **저자/연도**: Lin, G., Zhu, L., Li, J., Zou, X., & Tang, Y. (2021)
- **학술지**: *Computers and Electronics in Agriculture*, 188, 106350 · DOI [10.1016/j.compag.2021.106350](https://doi.org/10.1016/j.compag.2021.106350)
  - 교체 이유: 옛 판의 "Luo et al. 2018, *IJABE* 11(6)(줄기 인식 + RRT*)"는 찾을 수 없었다.
- **내용**: 순환신경망 + **DDPG**(강화학습)로 팔의 무충돌 경로를 빠르게 계획한다. Paper 08(같은 연구실)의 인식 결과와 이어진다.
- **수치**: 성공률 90.9% *(2차 인용)*.
- **우리 로봇에선**: 고전적 계획기(RRT·MoveIt)와 학습 계획기를 비교하는 자리다. 06 과정 M5 이후에 읽는다.

---

## Category D: 엔드이펙터와 절단 기구 (4편)

### [Paper 16] Dual-Arm Cooperation and Implementing for Robotic Harvesting Tomato Using Binocular Vision ✏️
- **저자/연도**: Ling, X., Zhao, Y., Gong, L., Liu, C., & Wang, T. (2019)
- **학술지**: *Robotics and Autonomous Systems*, 114, 134–143
  - 고침: 옛 판의 "*CEA* 162", "9.2초, 91.8%, 손상 0%", "실리콘 손가락 + 회전 가위"는 확인되지 않거나 틀렸다.
- **내용**: 양팔 협업이다. 쌍안 카메라로 위치를 잡고, 한 팔이 **진공컵으로 잡고** 다른 팔이 **넓은 범위 절단**을 한다.
- **수치**: 성공률 87.5%, 사이클 30초 이상 *(2차 인용, 리뷰 논문 기준)*.
- **우리 로봇에선**: 잡는 팔과 자르는 팔을 나누면 성공률은 오르지만 느리다. 팔이 하나인 우리는 **잡기 + 자르기를 한 손끝에** 넣어야 한다(Paper 18).

---

### [Paper 17] End-Effectors for Tomato Harvesting Robot 🔁
- **저자/연도**: Monta, M., Kondo, N., & Ting, K. C. (1998)
- **학술지**: *Artificial Intelligence Review*, 12, 11–25 · DOI [10.1023/A:1006595416751](https://doi.org/10.1023/A:1006595416751)
  - 교체 이유: 옛 판의 "Wang et al. 2019, *Trans. ASABE* 62(6)(진공 벨로우즈 + 3지 소프트)"는 찾을 수 없었다. **흡착 + 손가락** 조합의 원조 격인 실재 논문으로 바꿨다.
- **내용**: 평행판 손가락 2개 + **흡착 패드**. 흡착으로 열매를 떼어 끌어온 뒤 손가락이 꼭지 마디에서 분리한다. 첫 시제품은 **꼭지가 짧은 열매**에서 실패했다(손가락이 잡기 전에 패드에서 떨어짐). 두 번째 시제품은 열매 위치·패드 압력 센서를 더해 무손상 수확에 성공했다.
- **우리 로봇에선**: "패드에서 떨어진다"처럼 **실패 모드를 적어 둔 논문**이다. 우리 잡기 실패 기록(개발일지 9/11)과 나란히 읽을 것.

---

### [Paper 18] Linkage Integrated Fin Ray Gripper Capable of Safe Adaptive Grasping for Tomato Harvesting 🔁
- **저자/연도**: 아주대학교 연구진 (2025) — *저자명은 원문에서 확인*
- **학술지**: *Computers and Electronics in Agriculture*, 232 (2025년 5월)
  - 교체 이유: 옛 판의 "Ji et al. 2021, *RA-L* 6(3)(Fin-Ray + 내장 가위, 450 g)"은 찾을 수 없었다. 같은 아이디어(**추가 구동기 없이 쥐면 잘린다**)의 실재 논문이다.
- **내용**: **Fin Ray** 손가락이 유연하게 휘는 움직임이 **링크를 밀어 줄기를 자른다** — 잡기와 자르기가 한 동작이다. 의사강체 모델(pseudo-rigid body model)로 파지력을 분산해 과실 손상을 막고, 링크는 토크와 회전각을 고려해 설계했다.
- **수치**: 최대 절단력 13.9 N, 실험실에서 지름 73 mm까지의 실제 토마토 수확.
- **우리 로봇에선**: SO-101 집게 하나로 잡고 자르려면 가장 직접적인 참고 설계다. **국내 연구진**이다.

---

### [Paper 19] An Overview of End Effectors in Agricultural Robotic Harvesting Systems 🔁
- **저자/연도**: Vrochidou, E., Tsakalidou, V. N., Kalathas, I., Gkrimpizis, T., Pachidis, T., & Kaburlasos, V. G. (2022)
- **학술지**: *Agriculture*, 12(8), 1240 · DOI [10.3390/agriculture12081240](https://doi.org/10.3390/agriculture12081240) (공개 접근)
  - 교체 이유: 옛 판의 "Sepúlveda et al. 2020, *Agronomy* 10(9), 1373(절단 방식별 에너지 순위)"은 찾을 수 없었다.
- **내용**: 수확용 엔드이펙터 개관이다. 분리 방식·엔드이펙터 종류·부가 센서를 정리하고, 작물별로 엔드이펙터를 고르는 성능 지표를 제시한다. **접촉 파지형 집게가 가장 흔하고**, 연구는 토마토·사과·파프리카에 몰려 있다.
- **우리 로봇에선**: D 카테고리를 읽기 전에 지도로 먼저 본다.

---

## Category E: 비주얼 서보잉 및 필드 시스템 (5편)

### [Paper 20] Autonomous Sweet Pepper Harvesting for Protected Cropping Systems (Harvey) ✅
- **저자/연도**: Lehnert, C., et al. (2017)
- **학술지**: *IEEE Robotics and Automation Letters*, 2(2), 872–879 · arXiv [1706.02023](https://arxiv.org/abs/1706.02023)
- **내용**: QUT의 파프리카 수확 로봇 Harvey. **흡착으로 잡고 → 커터로 자르는** 엔드이펙터, RGB-D 인식(Paper 14와 이어짐), 두 품종으로 온실 시험.
- **수치**: 수확 성공률 — 작물을 손대지 않음 **46%**, 잎을 정리함 **58%**. 유리한 품종에서는 **90%를 떼어 냈다** → 저자들은 파지를 개선하면 크게 오른다고 본다.
  - 고침: 옛 판의 "76%, 42초"는 이 논문의 숫자가 아니다(후속 *JFR* 2020 논문의 것으로 보이나 확인하지 못했다). "실패의 60%가 도달 불가"도 확인되지 않아 지웠다.
- **우리 로봇에선**: 병목이 **인식이 아니라 잡기**였다는 실측이다. 우리 잡기 기록(48회 2성공 → 3회 3성공)과 같은 결론이다.

---

### [Paper 21] Harvesting Robots for High-Value Crops: State-of-the-Art Review and Challenges Ahead ✏️
- **저자/연도**: Bac, C. W., van Henten, E. J., Hemming, J., & Edan, Y. (2014)
- **학술지**: *Journal of Field Robotics*, 31(6), 888–911
  - 고침: 옛 판의 "*CEA* 104, 125–141"은 틀렸다. "사이클 타임 비율(탐색 15% · 이동 45% …)"과 "상용화 기준 85% / 15초 / 5%"는 확인되지 않아 지웠다.
- **내용**: 30년간의 수확 로봇 50대를 정량 비교한 리뷰다.
- **수치**: 평균 수확 성공률 **약 66%**, 30년 동안 **성능이 오르지 않았고**, 50대 중 **상용화된 것이 없다**.
- **제안한 네 과제**: 작업 단순화(작물을 로봇에 맞추기) · 로봇 개선 · 요구사항 정의와 성능 측정 · 실제 도입을 위한 추가 요구.
- **우리 로봇에선**: "작물을 로봇에 맞춘다"(잎 정리·재배 방식)도 정당한 해법이라는 근거다. 데모 무대 설계에 쓸 수 있다.

---

### [Paper 22] Design of an Eye-in-Hand Sensing and Servo Control Framework for Harvesting Robotics in Dense Vegetation ✏️
- **저자/연도**: Barth, R., Hemming, J., & van Henten, E. J. (2016)
- **학술지**: *Biosystems Engineering*, 146, 71–84 — EurAgEng Outstanding Paper Award 2018
  - 고침: 제목이 달랐다. "안착 오차 18.4 → 2.1 mm(88% 감소)"는 확인되지 않아 지웠다.
- **내용**: 빽빽한 잎 속 수확 로봇은 **여러 시점과 실시간 궤적 수정**이 필요하다는 전제로, 손목 카메라(eye-in-hand) 인식·제어 소프트웨어 틀을 **ROS 노드**들로 설계했다(작업 제어, 팔 동작, 영상 획득, 과실 검출, 시각 서보, SLAM). SWEEPER(Paper 23)의 기반이다.
- **우리 로봇에선**: `ros2/` 계통에서 노드를 나누는 방식과 비교해 읽을 것. 시각 서보가 시스템의 어디에 끼는지 보인다.

---

### [Paper 23] Development of a Sweet Pepper Harvesting Robot (SWEEPER) ✅
- **저자/연도**: Arad, B., et al. (2020)
- **학술지**: *Journal of Field Robotics*, 37(6), 1027–1039
- **내용**: EU 과제 SWEEPER의 통합 설계와 상업 온실 실증(4주, 열매 262개, 여러 품종·재배 조건).
- **수치**: 성공률 — 가장 유리한 작물 조건 **61%**, 현재 재배 조건 **18%**. 평균 사이클 **24초**, 그중 약 50%가 물류(열매 배출 7.8 s, 차체 이동 4.7 s). 팔을 빠르게 돌리면 실험실에서 15초까지 줄었다.
  - 고침: 옛 판의 "조건 개선 시 84%"와 "잎을 걷어내는 능동 인식"은 확인되지 않아 지웠다.
- **우리 로봇에선**: **같은 로봇이 작물 조건만으로 18% ↔ 61%**가 된다. 시연 결과를 말할 때 조건을 같이 적어야 하는 이유다.

---

### [Paper 24] Design, Integration, and Field Evaluation of a Robotic Apple Harvester ✏️
- **저자/연도**: Silwal, A., et al. (2017)
- **학술지**: *Journal of Field Robotics*, 34(6), 1140–1159
  - 고침: 옛 판의 "*Trans. ASABE* 60(4)", "사이클 7.2초", "글로벌 + 로컬 2카메라 구조"는 확인되지 않거나 틀렸다.
- **내용**: 7자유도 사과 수확기의 설계·통합·과수원 시험. 작업 단계마다 시간을 따로 재서 어디를 줄일지 분석했다.
- **수치**: 150개 시도 중 **127개 성공(84%)**, 열매당 평균 **6.0초**, 영상 위치 추정 1.5초.
- **우리 로봇에선**: **단계별 시간 분해**는 우리도 바로 할 수 있다(찾기 · 다가가기 · 잡기 · 자르기 · 돌아오기). 어디가 느린지 숫자로 말하게 된다.

---

다음 문서인 [`03_OPENSOURCE_GUIDE.md`](03_OPENSOURCE_GUIDE.md)에서는 오픈소스를 다룬다.
