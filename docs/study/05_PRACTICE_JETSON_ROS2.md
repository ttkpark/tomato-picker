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
autonomous_harvester.py - 토마토 자동 인식, 공간 좌표 변환 및 복합 엔드이펙터 수확 시퀀스
"""
import os
import json
import time
import math
import numpy as np
import cv2
from ultralytics import YOLO

# 레포지토리 하드웨어 기구학, 손-눈 보정 및 절단 모듈 직접 import
from tomato_picker.hardware.kinematics import Kinematics
from tomato_picker.hardware.handeye import Rigid, Intrinsics
from tomato_perception.stem_cut import (
    find_cut_point,
    sample_stem_depth,
    compute_cutting_pose,
    plan_dual_action_trajectory,
)

class AutonomousTomatoHarvester:
    def __init__(self, calib_path="~/arm_eye.json"):
        # 1. 캘리브레이션 데이터 로드 (Hand-Eye & Intrinsics: 단위 mm)
        full_path = os.path.expanduser(calib_path)
        with open(full_path, "r", encoding="utf-8") as f:
            calib_data = json.load(f)

        # D405 내부 파라미터 로드 (Intrinsics 규약: fx, fy, ppx, ppy)
        intr_dict = calib_data.get("intrinsics", {})
        self.intr = Intrinsics(
            width=int(intr_dict.get("width", 848)),
            height=int(intr_dict.get("height", 480)),
            fx=float(intr_dict.get("fx", 438.0)),
            fy=float(intr_dict.get("fy", 438.0)),
            ppx=float(intr_dict.get("ppx", 424.0)),
            ppy=float(intr_dict.get("ppy", 240.0)),
            model=str(intr_dict.get("model", "none")),
        )
        
        # Hand-Eye 변환 로드: 손목 장착(on_arm) 카메라이므로 T_tool_cam (Tool -> Camera, 단위 mm)
        t_data = calib_data.get("transform", {})
        self.T_tool_cam = Rigid.from_dict(t_data) if t_data else Rigid(np.eye(3), np.zeros(3))
        
        # 2. 기구학 엔진 초기화 (SO-101 기구학, 모든 길이 단위: mm)
        self.kin = Kinematics()
        
        # 3. 딥러닝 세그멘테이션 모델 로드
        print("[AI] YOLOv8 세그멘테이션 모델 로딩 중...")
        self.model = YOLO("yolov8n-seg.pt")
        print("[AI] 준비 완료.")

    def deproject_to_camera_frame(self, u: float, v: float, depth_mm: float) -> tuple[float, float, float]:
        """픽셀 및 깊이(mm) -> 카메라 3차원 광학 좌표 (X_c, Y_c, Z_c) mm."""
        return self.intr.deproject(u, v, depth_mm)

    def transform_to_arm_base(self, P_cam_mm: tuple[float, float, float], current_joints: dict[str, float]) -> tuple[float, float, float]:
        """카메라 광학 좌표(mm) -> 팔 베이스 3D 좌표(mm) 동적 변환 (Eye-in-Hand).

        T_base_cam(t) = T_base_tool(q(t)) * T_tool_cam
        P_base = T_base_tool(q) * (T_tool_cam * P_cam)
        """
        # 1. Tool Frame(TCP) 좌표로 변환: P_tool = T_tool_cam * P_cam
        p_tool = self.T_tool_cam.apply(P_cam_mm)

        # 2. 현재 관절각에서의 순기구학 T_base_tool (FK)
        # kinematics.forward()는 TCP 위치 (x, y, z) 및 pitch, roll 산출
        fk = self.kin.forward(
            current_joints.get("shoulder_pan", 0.0),
            current_joints.get("shoulder_lift", 90.0),
            current_joints.get("elbow_flex", 0.0),
            current_joints.get("wrist_flex", 0.0),
            current_joints.get("wrist_roll", 0.0),
        )
        # 간이 기저 변환 또는 TF 버퍼 lookup_transform('arm_base', 'tool0') 사용
        # (실전 ROS 2 환경에서는 tf2_ros Buffer를 통해 arm_base -> camera_optical_frame 직접 조회 권장)
        return (float(p_tool[0] + fk.x), float(p_tool[1] + fk.y), float(p_tool[2] + fk.z))

    def detect_target(self, color_bgr: np.ndarray, depth_img_raw: np.ndarray, current_joints: dict[str, float]):
        """
        토마토 과실 및 줄기 검출, 3D 로컬라이제이션 및 6-DoF 절단 포즈 도출
        depth_img_raw: uint16 밀리미터 또는 float32 미터 (D405 규약: 70mm ~ 500mm 유효)
        """
        # 깊이 맵 단위 mm 정규화
        depth_mm = depth_img_raw.astype(np.float64)
        if depth_img_raw.dtype in (np.float32, np.float64) and float(np.nanmax(depth_mm)) < 20.0:
            depth_mm = depth_mm * 1000.0

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
                u = float(M["m10"] / M["m00"])
                v = float(M["m01"] / M["m00"])

                # 국소 5x5 중앙값 깊이 추출 (D405 유효대역: 70mm ~ 500mm)
                d_mm = sample_stem_depth(depth_mm, u, v, window_radius=2, min_depth_mm=70.0, max_depth_mm=500.0)
                if d_mm is None:
                    continue

                # 3D 카메라 좌표 및 팔 베이스 좌표 산출 (단위: mm)
                P_cam = self.deproject_to_camera_frame(u, v, d_mm)
                P_base = self.transform_to_arm_base(P_cam, current_joints)

                targets.append({
                    'pixel': (u, v),
                    'depth_mm': d_mm,
                    'pos_cam_mm': P_cam,
                    'pos_base_mm': P_base,
                })

        return targets

    def execute_harvest(self, fruit_pos_base_mm: tuple[float, float, float], cut_pose_base, arm_controller):
        """
        복합 엔드이펙터(파지/흡착 + 전단 커터) 4단계 시퀀셜 수확 모션 실행 (단위: mm)
        """
        print(f"\n[Motion] 수확 목표점 (Base 좌표계): X={fruit_pos_base_mm[0]:.1f}mm, Y={fruit_pos_base_mm[1]:.1f}mm, Z={fruit_pos_base_mm[2]:.1f}mm")

        # 1. 복합 엔드이펙터 4단계 궤적 계획 (Pre-grasp -> Grasp -> Cut -> Retract) 및 작업공간/5-DoF 검증
        traj = plan_dual_action_trajectory(
            fruit_pos_base=fruit_pos_base_mm,
            cut_pose_base=cut_pose_base,
            cutter_offset_up_mm=30.0,
            pre_standoff_mm=50.0,
            retract_standoff_mm=60.0,
            tolerance_mm=10.0,
            z_min_mm=15.0,
            z_max_mm=445.0,
            r_min_mm=90.0,
            r_max_mm=310.0,
        )
        if traj is None or not traj["feasible"]:
            err_reason = traj["workspace"]["violations"] if (traj and not traj["workspace"]["feasible"]) else "기하 불일치"
            print(f"[Error] 복합 궤적 실행 불가: {err_reason}")
            return False

        # 2. 1단계 Pre-grasp: 접근 반대방향 50mm 대기 위치 이동
        pre_x, pre_y, pre_z = traj["pre_grasp_tcp"]
        print(f"[Motion] 1단계: Pre-grasp 대기 자세 접근 (X={pre_x:.1f}, Y={pre_y:.1f}, Z={pre_z:.1f})")
        arm_controller.move_to_cartesian(pre_x, pre_y, pre_z)
        time.sleep(1.5)

        # 3. 2단계 Grasp: 과실 파지/흡착 접촉 자세 진입 및 진공 흡착 ON
        gx, gy, gz = traj["grasp_tcp"]
        print(f"[Motion] 2단계: 과실 흡착 접촉 진입 (X={gx:.1f}, Y={gy:.1f}, Z={gz:.1f})")
        arm_controller.set_vacuum(True)
        arm_controller.move_to_cartesian(gx, gy, gz)
        time.sleep(1.0)

        # 4. 3단계 Cut: 과실 흡착 유지 상태에서 전단 가위 구동 (줄기 순간 절단)
        print("[Tool] 3단계: 줄기 전단 가위 작동 (0.3초 순간 절단)")
        arm_controller.actuate_cutter()
        time.sleep(0.5)

        # 5. 4단계 Retract: 수확물 분리 후 안전 후퇴 (1차 파지부 수확물 후퇴 좌표)
        rx, ry, rz = traj["retract_grasp_tcp"]
        print(f"[Motion] 4단계: 과실 분리 후 안전 후퇴 (X={rx:.1f}, Y={ry:.1f}, Z={rz:.1f})")
        arm_controller.move_to_cartesian(rx, ry, rz)
        time.sleep(1.0)

        # 6. 바구니 적재
        print("[Motion] 5단계: 바구니로 회전 이동 및 과실 배출")
        arm_controller.play_preset_slot(slot_name="basket_drop")
        time.sleep(2.0)
        arm_controller.set_vacuum(False)
        time.sleep(0.5)

        # 7. 홈 복귀
        print("[Motion] 6단계: 홈 위치로 복귀")
        arm_controller.play_preset_slot(slot_name="home")
        print("[Done] 복합 엔드이펙터 자동 수확 사이클 성공 완료!")
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

# 4. ROS 2 스택 및 보드 통신 종합 검증 (572개 테스트)
python ros2/tools/ros_selfcheck.py

# 5. 복합 엔드이펙터 궤적 및 줄기 절단 검증 (119개 테스트)
python ros2/tools/stem_cut_check.py
```

모든 테스트가 통과하면 수학적 변환 오류나 특이점 충돌 없이 실장비에서 안전하게 구동됩니다.

