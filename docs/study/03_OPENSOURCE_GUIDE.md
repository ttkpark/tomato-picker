# 토마토 수확 로봇을 위한 핵심 오픈소스 12선 및 실무 연동 가이드
> **12 Essential Open-Source Projects for Agricultural Robotics, 3D Vision, and Robotic Manipulation**

본 문서는 토마토 수확 시스템 구축에 즉시 활용할 수 있는 **12개의 최고 수준 오픈소스 라이브러리 및 데이터셋**을 엄선하여 소개합니다. 각 프로젝트의 기술 스택, 설치 방법, 그리고 현재 프로젝트(`Intel RealSense D405`, `Orbbec Astra Pro`, `SO-101 팔`, `Jetson Orin Nano`, `ROS 2 Humble`)와의 실전 연동 방안 및 예제 코드를 제공합니다.

---

## 오픈소스 프로젝트 목록 한눈에 보기

| # | 프로젝트명 | 주 용도 | 기술 스택 / 라이선스 |
|---|---|---|---|
| **01** | [Laboro Tomato](#01-laboro-tomato-dataset--benchmark) | 온실 토마토 인스턴스 세그멘테이션 데이터셋 | PyTorch, COCO Format / Apache-2.0 |
| **02** | [Ultralytics YOLOv8/11](#02-ultralytics-yolov8--yolo11) | 과실/줄기 실시간 인스턴스 분할 및 숙도 분류 | Python, PyTorch, TensorRT / AGPL-3.0 |
| **03** | [Open3D](#03-open3d) | 3D 포인트 클라우드 전처리, RANSAC 피팅, ICP 정렬 | C++, Python / MIT |
| **04** | [AnyGrasp](#04-anygrasp--graspnet-1billion) | 비정형 물체 대상 6-DoF 파지(Grasp) 자세 추정 | PyTorch, CUDA / Non-commercial/Custom |
| **05** | [Contact-GraspNet](#05-contact-graspnet-nvidia) | 밀집 점군 기반 6자유도 접촉면 파지 생성 신경망 | TensorFlow 2, Python / CC-BY-NC-SA |
| **06** | [MoveIt 2](#06-moveit-2) | ROS 2 모션 플래닝, 충돌 회피, OMPL 기구학 솔버 | ROS 2, C++, Python / BSD-3-Clause |
| **07** | [easy_handeye](#07-easy_handeye--moveit_calibration) | Eye-in-Hand / Eye-to-Hand 캘리브레이션 GUI 툴 | ROS 2, Python, OpenCV / LGPL |
| **08** | [ViSP](#08-visp-visual-servoing-platform) | 실시간 비주얼 서보잉(IBVS/PBVS) 라이브러리 | C++, Python / GPL-2.0 |
| **09** | [Hugging Face LeRobot](#09-hugging-face-lerobot) | SO-100/SO-101 로봇 팔 모작 학습 및 텔레오퍼레이션 | Python, PyTorch, Feetech / Apache-2.0 |
| **10** | [Tomato-Harvesting-Robot](#10-tomato-harvesting-robot) | ROS 기반 토마토 깊이 추정 및 수확 레퍼런스 노드 | ROS 1/2, Python, RealSense / MIT |
| **11** | [NVIDIA Isaac Lab](#11-nvidia-isaac-lab--isaac-sim) | 농업 환경 물리 시뮬레이션 및 합성 점군 생성 | Omniverse, PhysX, PyTorch / BSD-3-Clause |
| **12** | [PointNeXt](#12-pointnext-pointnet-v2) | 3D 점군 기반 줄기/과실 세부 세그멘테이션 백본 | PyTorch, CUDA / MIT |

---

## 01. Laboro Tomato Dataset & Benchmark
- **저장소**: [https://github.com/LaboroAI/Laboro-Tomato](https://github.com/LaboroAI/Laboro-Tomato)
- **개요**: 상업용 온실에서 직접 촬영한 방울토마토 및 일반 토마토의 고해상도 인스턴스 세그멘테이션 데이터셋입니다.
- **주요 특징**:
  - 크기별(`tomato_regular`, `tomato_cherry`) 및 숙도별(`fully_ripened`, `half_ripened`, `green`) 라벨링.
  - COCO 데이터 형식의 정밀한 폴리곤 마스크(Polygon Mask) 제공.
- **설치 및 다운로드**:
  ```bash
  git clone https://github.com/LaboroAI/Laboro-Tomato.git
  cd Laboro-Tomato
  # 스크립트를 통해 원본 이미지 및 COCO json 다운로드
  python download.py
  ```
- **우리 프로젝트 적용점**:
  - `src/tomato_picker/vision/`의 색상 검출 모델을 딥러닝 기반 세그멘테이션 모델로 고도화할 때 베이스라인 훈련 데이터셋으로 사용.

---

## 02. Ultralytics YOLOv8 / YOLO11
- **저장소**: [https://github.com/ultralytics/ultralytics](https://github.com/ultralytics/ultralytics)
- **개요**: 실시간 컴퓨터 비전 분야의 디팩토 표준 라이브러리로, 단일 파이프라인에서 바운딩 박스 검출과 화소 단위 인스턴스 세그멘테이션(`yolov8n-seg.pt`)을 초고속으로 수행합니다.
- **설치**:
  ```bash
  pip install ultralytics
  # Jetson Orin Nano TensorRT 가속 변환
  yolo export model=yolov8n-seg.pt format=engine device=0 half=True
  ```
- **실전 파이썬 예제 (토마토 마스크 & 중심점 추출)**:
  ```python
  from ultralytics import YOLO
  import cv2
  import numpy as np

  model = YOLO("yolov8n-seg.pt") # 또는 파인튜닝된 tomato-seg.pt
  results = model.predict(source="camera_frame.jpg", conf=0.6, classes=[0]) # 과실 클래스

  for r in results:
      if r.masks is not None:
          for mask, box in zip(r.masks.xy, r.boxes):
              # 마스크 중심점(Centroid) 계산
              pts = np.int32([mask])
              M = cv2.moments(pts)
              if M["m00"] != 0:
                  cx = int(M["m10"] / M["m00"])
                  cy = int(M["m01"] / M["m00"])
                  print(f"토마토 중심 픽셀: ({cx}, {cy})")
  ```

---

## 03. Open3D
- **저장소**: [https://github.com/isl-org/Open3D](https://github.com/isl-org/Open3D)
- **개요**: 3D 데이터(Point Cloud, Mesh) 처리를 위한 현대적 오픈소스 라이브러리입니다. C++ 및 Python API를 제공합니다.
- **주요 기능**:
  - D405 / Astra 깊이 영상 $\to$ 3D 점군 변환.
  - Voxel Grid 다운샘플링, 통계적 이상치 제거(SOR), 법선 벡터 계산.
  - RANSAC 구체/원통 피팅을 통한 토마토 3D 중심점 및 줄기 축 벡터 도출.
- **설치**:
  ```bash
  pip install open3d
  ```
- **실전 예제 (토마토 3D 구체 피팅)**:
  ```python
  import open3d as o3d
  import numpy as np

  # RGBD 이미지에서 점군 생성
  rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
      color_image, depth_image, depth_scale=10000.0, depth_trunc=0.5, convert_rgb_to_intensity=False)
  pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

  # 통계적 노이즈 제거
  clean_pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
  ```

---

## 04. AnyGrasp & GraspNet-1Billion
- **저장소**: [https://github.com/graspnet/anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk) / [https://github.com/graspnet/graspnet-baseline](https://github.com/graspnet/graspnet-baseline)
- **개요**: 대규모 실제 3D 점군 데이터셋(GraspNet-1B) 기반으로 훈련된 6자유도(6-DoF) 병렬 조(Parallel-jaw) 그리퍼 파지 자세 생성 네트워크입니다.
- **주요 기능**:
  - 입력된 RGB-D 포인트 클라우드에서 충돌이 없으면서 파지 성공 확률이 가장 높은 엔드이펙터의 3D 위치 $[X, Y, Z]$ 및 회전 행렬 $R_{3\times3}$, 파지 폭(Grasp width)을 동시 출력.
- **실무 적용점**:
  - 온실 환경의 얽혀 있는 토마토 화방에서 어떤 각도로 집게를 넣어야 주변 줄기와 충돌하지 않는지 최적의 그리퍼 진입 방향을 계산할 때 활용.

---

## 05. Contact-GraspNet (NVIDIA)
- **저장소**: [https://github.com/NVlabs/contact_graspnet](https://github.com/NVlabs/contact_graspnet)
- **개요**: 포인트 클라우드의 3D 접촉면(Contact points) 기하 구조를 분석하여 6-DoF 파지 포즈를 제안하는 신경망입니다.
- **특징**:
  - CAD 모델이나 물체 분류 없이도 임의의 복잡한 비정형 객체 표면에서 바로 작동.
  - 가려짐(Occlusion)이 심한 온실 환경에서 유효한 접촉면 탐색에 매우 강력함.

---

## 06. MoveIt 2
- **저장소**: [https://github.com/moveit/moveit2](https://github.com/moveit/moveit2)
- **개요**: ROS 2 생태계의 표준 로봇 조작(Manipulation) 프레임워크입니다.
- **주요 기능**:
  - 5축/6축 매니퓰레이터(SO-101)의 정기구학/역기구학(IK-Fast, KDL, PickIK) 연산.
  - OMPL(Open Motion Planning Library) 기반의 RRT-Connect, BiTRRT 충돌 회피 경로 생성.
  - 3D 점군(OctoMap)을 실시간으로 Planning Scene에 등록하여 줄기 및 지지대와의 충돌을 실시간 회피.
- **설치**:
  ```bash
  sudo apt install ros-humble-moveit
  ```
- **우리 프로젝트 연동**:
  - `ros2/` 스택에서 직교 제어(Cartesian) 시 장애물 회피 궤적 플래닝 엔진으로 결합.

---

## 07. easy_handeye & moveit_calibration
- **저장소**: [https://github.com/marcoesposito1988/easy_handeye](https://github.com/marcoesposito1988/easy_handeye)
- **개요**: ROS 및 ROS 2 환경에서 카메라와 로봇 팔 사이의 변환 행렬($^{base}T_{camera}$ 또는 $^{hand}T_{camera}$)을 체커보드/ChArUco 마커를 이용해 대화형 GUI로 자동 측정해주는 툴입니다.
- **특징**:
  - Tsai-Lenz, Park-Martin, Daniilidis, Dual Quaternion 등 주요 Hand-Eye 솔버 내장.
  - Rviz 인터페이스에서 현재 캘리브레이션 오차(잔차)를 시각적으로 확인 가능.
- **우리 프로젝트 연동**:
  - 현재 저장소의 `src/tomato_picker/hardware/handeye.py` 알고리즘과 수학적으로 완벽히 호환되며, 젯슨 실장비에서 마커 기반 자동 캘리브레이션 노드로 확장 가능.

---

## 08. ViSP (Visual Servoing Platform)
- **저장소**: [https://github.com/lagadic/visp](https://github.com/lagadic/visp)
- **개요**: INRIA에서 개발한 로봇 비주얼 서보잉 전문 라이브러리입니다.
- **주요 기능**:
  - 영상 기반 비주얼 서보잉(IBVS) 및 위치 기반 비주얼 서보잉(PBVS).
  - 로봇 팔의 처짐이나 기구학 오차가 있어도, 타겟 토마토/줄기가 카메라 화면의 정중앙에 올 때까지 엔드이펙터 속도 벡터를 닫힌 루프(Closed-loop)로 지속 보정.
- **파이썬 바인딩**:
  ```bash
  sudo apt install ros-humble-vision-visp
  # 또는 소스 빌드
  ```

---

## 09. Hugging Face LeRobot
- **저장소**: [https://github.com/huggingface/lerobot](https://github.com/huggingface/lerobot)
- **개요**: 저비용 오픈소스 로봇 팔(SO-100, SO-101, Koch) 및 모바일 매니퓰레이터를 위한 최신 AI 모작 학습(Imitation Learning) 프레임워크입니다.
- **주요 기능**:
  - 리더(Leader) 팔 $\to$ 팔로워(Follower) 팔 간의 실시간 텔레오퍼레이션 및 모션 데이터셋 기록.
  - ACT(Action Chunking with Transformers), Diffusion Policy 기반 엔드투엔드 파지 정책 학습 지원.
- **우리 프로젝트 연동**:
  - 현재 우리 시스템의 `tools/mirror_toggle.py` 및 `firmware/`와 하드웨어 레벨(Feetech STS3215 서보)에서 직접 연결되어 있으며, 사람이 리더 암으로 토마토를 따는 모션을 기록하여 자율 주행 정책으로 학습시킬 수 있습니다.

---

## 10. Tomato-Harvesting-Robot
- **저장소**: [https://github.com/richard98hess444/Tomato-Harvesting-Robot](https://github.com/richard98hess444/Tomato-Harvesting-Robot)
- **개요**: RealSense 카메라와 ROS 노드를 활용하여 토마토 검출, 3D 좌표 추정, 그리고 로봇 팔 수확 동작을 수행하는 오픈소스 프로젝트입니다.
- **특징**:
  - 초보자가 ROS 비전 노드와 로봇 제어 노드의 통신 구조(`/camera/depth/image_raw` $\to$ `/tomato_pose` $\to$ `/arm_controller`)를 이해하기에 가장 직관적인 구조를 가짐.

---

## 11. NVIDIA Isaac Lab / Isaac Sim
- **저장소**: [https://github.com/isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab)
- **개요**: NVIDIA Omniverse 기반의 로봇 강화학습 및 물리 시뮬레이션 프레임워크입니다.
- **주요 기능**:
  - 초고속 GPU 병렬 시뮬레이션 환경에서 온실 토마토 작물 환경 구축.
  - 로봇 팔의 충돌 회피 모션 및 소프트 그리퍼의 물리적 상호작용(변형, 마찰)을 사전 검증.
  - Photorealistic RGB-D 합성 센서 스트림 생성으로 Sim-to-Real 훈련 지원.

---

## 12. PointNeXt (PointNet++ 개선판)
- **저장소**: [https://github.com/guochengqian/PointNeXt](https://github.com/guochengqian/PointNeXt)
- **개요**: 기존 PointNet++의 연산 속도와 훈련 안정성을 대폭 개선한 SOTA 3D 포인트 클라우드 세그멘테이션 백본입니다.
- **적용**:
  - 잎과 뒤엉킨 토마토 줄기 점군에서 줄기의 3D 기하 중심선(Skeleton)과 꽃받침 연결점을 정확히 분리해내는 데 최적의 3D 딥러닝 모델.

---

다음 문서인 [`04_END_EFFECTOR_MANIPULATION.md`](04_END_EFFECTOR_MANIPULATION.md)에서는 줄기 스켈레톤 추출, 6-DoF Grasp Pose 계산 및 커터 엔드이펙터 설계 원리를 다룹니다.

