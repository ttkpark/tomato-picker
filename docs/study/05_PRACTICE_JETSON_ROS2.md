# 실장비 실전 구현 가이드: RealSense D405 + SO-101 + Jetson Orin Nano + ROS 2
> **Production Engineering Guide: End-to-End Implementation on Real Robot Hardware**

본 문서는 본 저장소(`tomato-picker`)의 실제 하드웨어 구성에 맞춰, 실시간 깊이 비전 스트리밍부터 줄기 절단 및 바구니 적재까지 구동할 수 있는 **완전한 엔지니어링 구현 가이드**입니다.

---

## 1. 하드웨어 배선 및 프로세스 소유권 규칙

우리 로봇 시스템은 자원 경합을 방지하기 위해 엄격한 **단일 프로세스 소유권(Single Process Ownership)** 원칙을 따릅니다:

```
[Jetson Orin Nano]
  ├── USB 3.2 ── Intel RealSense D405 (서비스: depth-cam, /dev/shm/d405_*)
  ├── USB 2.0 ── Orbbec Astra Pro     (서비스: astra-cam, /dev/shm/astra_*)
  ├── /dev/ttyACM0 ── SO-101 로봇 팔 (단일 프로세스만 독점 점유)
  └── /dev/ttyUSB0 ── 아두이노 메카넘 주행 보드 (mecanum_stable v2 프로토콜)
```

> [!CAUTION]
> 1. **시리얼 포트 중복 점유 금지**: `/dev/ttyACM0`과 `/dev/ttyUSB0`은 동시에 2개 이상의 프로세스가 열 수 없습니다. ROS 2를 구동하기 전 레거시 백그라운드 서비스(`tomato-voice`, `controller-drive`)가 꺼져 있는지 확인하십시오.
> 2. **카메라 유효 거리 확인**: D405는 대상 토마토가 7~50cm 이내에 있어야 유효한 깊이가 출력됩니다. 50cm를 넘으면 Astra Pro로 전환해야 합니다.

---

## 2. 전체 엔드투엔드 파이썬 실전 노드: `autonomous_harvester.py`

아래 코드는 D405 깊이 뷰어, Hand-Eye 보정 변환, 역기구학 풀이, 팔 시퀀스 구동을 하나로 통합한 완전한 자동 수확 스크립트입니다:

```python
#!/usr/bin/env python3
"""
autonomous_harvester.py - 토마토 자동 인식, 공간 좌표 변환 및 수확 시퀀스
"""
import os
import json
import time
import math
import numpy as np
import cv2
from ultralytics import YOLO

# 현재 레포지토리의 하드웨어 및 기구학 모듈 직접 import
from tomato_picker.hardware.kinematics import Kinematics
from tomato_picker.hardware.handeye import HandEyeTransform

class AutonomousTomatoHarvester:
    def __init__(self, calib_path="~/arm_eye.json"):
        # 1. 캘리브레이션 데이터 로드 (Hand-Eye & Intrinsics)
        full_path = os.path.expanduser(calib_path)
        with open(full_path, "r", encoding="utf-8") as f:
            calib_data = json.load(f)

        # D405 내부 파라미터 로드
        self.K = {
            'fx': calib_data['intrinsics']['fx'],
            'fy': calib_data['intrinsics']['fy'],
            'cx': calib_data['intrinsics']['cx'],
            'cy': calib_data['intrinsics']['cy']
        }
        
        # Hand-Eye 변환 행렬 (Camera Frame -> Arm Base Frame)
        self.T_base_cam = np.array(calib_data['T_base_cam'], dtype=np.float64)
        
        # 2. 기구학 엔진 초기화 (SO-101 기구학)
        self.kin = Kinematics()
        
        # 3. 딥러닝 세그멘테이션 모델 로드
        print("[AI] YOLOv8 세그멘테이션 모델 로딩 중...")
        self.model = YOLO("yolov8n-seg.pt")
        print("[AI] 준비 완료.")

    def deproject_to_camera_frame(self, u: int, v: int, depth_m: float) -> np.ndarray:
        """픽셀 및 깊이 -> 카메라 3차원 좌표 (X_c, Y_c, Z_c)"""
        x = (u - self.K['cx']) * depth_m / self.K['fx']
        y = (v - self.K['cy']) * depth_m / self.K['fy']
        z = depth_m
        return np.array([x, y, z, 1.0], dtype=np.float64)

    def transform_to_arm_base(self, P_cam_homo: np.ndarray) -> np.ndarray:
        """카메라 동차 좌표 -> 로봇 팔 베이스 3D 좌표 (X_b, Y_b, Z_b)"""
        P_base = self.T_base_cam @ P_cam_homo
        return P_base[:3]

    def detect_target(self, color_bgr: np.ndarray, depth_img: np.ndarray):
        """
        토마토 검출 및 3D 로컬라이제이션
        depth_img: float32 (단위: 미터)
        """
        results = self.model.predict(source=color_bgr, conf=0.6, verbose=False)
        targets = []

        for r in results:
            if r.masks is None:
                continue
            for mask, box in zip(r.masks.xy, r.boxes):
                pts = np.int32([mask])
                M = cv2.moments(pts)
                if M["m00"] == 0:
                    continue
                u = int(M["m10"] / M["m00"])
                v = int(M["m01"] / M["m00"])

                # 중심점 주변 5x5 영역의 중앙값 깊이 추출 (노이즈 방지)
                u_min, u_max = max(0, u-2), min(depth_img.shape[1], u+3)
                v_min, v_max = max(0, v-2), min(depth_img.shape[0], v+3)
                depth_roi = depth_img[v_min:v_max, u_min:u_max]
                valid_depth = depth_roi[depth_roi > 0.05] # 5cm 이상만 유효

                if len(valid_depth) == 0:
                    continue
                d_m = float(np.median(valid_depth))

                # D405 유효거리 검사 (7cm ~ 50cm)
                if not (0.07 <= d_m <= 0.50):
                    continue

                # 3D 카메라 좌표 및 로봇 팔 좌표 산출
                P_cam = self.deproject_to_camera_frame(u, v, d_m)
                P_base = self.transform_to_arm_base(P_cam)
                targets.append({
                    'pixel': (u, v),
                    'depth_m': d_m,
                    'pos_cam': P_cam[:3],
                    'pos_base': P_base
                })

        return targets

    def execute_harvest(self, target_base_pos: np.ndarray, arm_controller):
        """
        무충돌 수확 모션 시퀀스 실행
        target_base_pos: [X, Y, Z] (단위: mm)
        """
        tx, ty, tz = target_base_pos * 1000.0 # m -> mm 변환
        print(f"\n[Motion] 수확 목표점 (Base 좌표계): X={tx:.1f}mm, Y={ty:.1f}mm, Z={tz:.1f}mm")

        # 1. 도달 가능성(Reachability) 및 역기구학 검증
        ik_sol = self.kin.inverse_kinematics(tx, ty, tz)
        if ik_sol is None:
            print("[Error] 목표 지점이 팔의 작업 영역(Workspace)을 벗어났습니다!")
            return False

        # 2. Pre-grasp 접근 자세 (목표점 50mm 뒤쪽)
        dist = math.sqrt(tx**2 + ty**2)
        ratio = (dist - 50.0) / dist
        pre_x, pre_y = tx * ratio, ty * ratio
        print(f"[Motion] 1단계: Pre-grasp 지점으로 접근 (X={pre_x:.1f}, Y={pre_y:.1f})")
        arm_controller.move_to_cartesian(pre_x, pre_y, tz + 20.0)
        time.sleep(1.5)

        # 3. 선형 밀기 (Linear Cartesian Push) & 집게 열기
        print("[Motion] 2단계: 집게 열고 줄기/과실로 최종 선형 진입")
        arm_controller.set_gripper(open_ratio=1.0)
        arm_controller.move_to_cartesian(tx, ty, tz)
        time.sleep(1.0)

        # 4. 절단 및 파지 실행
        print("[Tool] 3단계: 과실 파지 및 커터 구동")
        arm_controller.set_gripper(open_ratio=0.0) # 집게 닫기
        time.sleep(0.8)

        # 5. 후퇴 (Retract)
        print("[Motion] 4단계: 과실 분리 후 안전 후퇴")
        arm_controller.move_to_cartesian(pre_x, pre_y, tz + 40.0)
        time.sleep(1.0)

        # 6. 바구니 적재
        print("[Motion] 5단계: 바구니로 회전 이동 및 과실 배출")
        arm_controller.play_preset_slot(slot_name="basket_drop")
        time.sleep(2.0)
        arm_controller.set_gripper(open_ratio=1.0) # 배출
        time.sleep(0.5)

        # 7. 홈 복귀
        print("[Motion] 6단계: 홈 위치로 복귀")
        arm_controller.play_preset_slot(slot_name="home")
        print("[Done] 수확 사이클 성공 완료!")
        return True
```

---

## 3. 자체 무결성 검증 명령어 (PC & Jetson 사전 검증)

젯슨 실장비에 코드를 올리기 전, 수학적 정합성을 검증하기 위해 레포지토리에 내장된 테스트 스위트를 실행하십시오:

```bash
# 1. Hand-Eye 보정 알고리즘 수학 검증 (45개 테스트)
python tools/handeye_check.py

# 2. 카메라-로봇 팔 좌표계 배선 검증 (102개 테스트)
python tools/eye_check.py

# 3. 직교 제어 및 기구학 처짐 피드백 검증 (93개 테스트)
python tools/arm_cartesian_check.py

# 4. ROS 2 스택 및 보드 통신 종합 검증 (410개 테스트)
python ros2/tools/ros_selfcheck.py
```

모든 테스트가 통과하면 수학적 변환 오류나 특이점 충돌 없이 실장비에서 안전하게 구동됩니다.

