#!/usr/bin/env python3
"""줄기 절단점 도출 자체검증 — 카메라도 실제 줄기도 없이 개발 PC에서 돈다.

    python ros2/tools/stem_cut_check.py

`tomato_perception/stem_cut.py`의 세선화·절단점 계산을 합성 마스크로 시험한다.
실기 검증(진짜 줄기 마스크에서 통하는지)은 이 자체검증의 범위 밖이다 —
2026-09-11 개발일지가 이미 "가는 표적은 시뮬레이션이 아니라 실측으로만 검증된다"
고 못박아 뒀다. 여기서 보는 것은 **수학이 스스로 모순되지 않는가**뿐이다.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8")

HERE = os.path.dirname(os.path.abspath(__file__))
ROS2 = os.path.dirname(HERE)
SRC = os.path.join(ROS2, "src")
sys.path.insert(0, os.path.join(SRC, "tomato_perception"))

from tomato_perception.stem_cut import (  # noqa: E402
    CutPoint, CutPose3D, compute_cutting_pose,
    compute_pre_grasp_pose, compute_retract_pose,
    evaluate_5dof_cut_alignment, find_cut_point,
    sample_stem_depth, skeleton_points, transform_cut_pose,
    zhang_suen_thin,
)

FAILED: list[str] = []
PASSED = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global PASSED
    if ok:
        PASSED += 1
        print(f"  ok   {name}" + (f"  ({detail})" if detail else ""))
    else:
        FAILED.append(name)
        print(f"  FAIL {name}  {detail}")


def _straight_stem(length=60, width=3, x0=50) -> np.ndarray:
    """수직 직선 줄기 마스크 — 위쪽(y=0)이 과실 쪽."""
    mask = np.zeros((length, 100), dtype=bool)
    mask[:, x0 - width // 2: x0 + width // 2 + 1] = True
    return mask


def _fruit_at_top(x0=50, r=15, length=60) -> np.ndarray:
    mask = np.zeros((length, 100), dtype=bool)
    yy, xx = np.ogrid[:length, :100]
    mask[((xx - x0) ** 2 + (yy - 0) ** 2) <= r * r] = True
    return mask


def test_thinning() -> None:
    print("\n[세선화] Zhang-Suen 기본 성질")
    thick = np.zeros((30, 30), dtype=bool)
    thick[10:20, 10:20] = True  # 10x10 사각형
    skel = zhang_suen_thin(thick)
    check("세선화 결과가 원본보다 화소가 훨씬 적다",
          0 < skel.sum() < thick.sum(), f"{skel.sum()}/{thick.sum()}")
    check("빈 마스크는 빈 스켈레톤", zhang_suen_thin(np.zeros((10, 10), bool)).sum() == 0)

    line = np.zeros((30, 30), dtype=bool)
    line[15, 5:25] = True  # 이미 1픽셀 두께
    skel_line = zhang_suen_thin(line)
    check("이미 얇은 선은 거의 그대로 유지된다(과도한 침식 없음)",
          skel_line.sum() >= line.sum() * 0.5, f"{skel_line.sum()}/{line.sum()}")


def test_skeleton_points() -> None:
    print("\n[좌표 추출] skeleton_points")
    empty = np.zeros((10, 10), dtype=bool)
    pts = skeleton_points(empty)
    check("빈 스켈레톤은 shape (0,2)", pts.shape == (0, 2), f"{pts.shape}")

    mask = np.zeros((10, 10), dtype=bool)
    mask[3, 4] = True
    pts2 = skeleton_points(mask)
    check("단일 화소 좌표가 (u=4, v=3)으로 나온다(x,y 순서 확인)",
          pts2.shape == (1, 2) and tuple(pts2[0]) == (4.0, 3.0), f"{pts2}")


def test_find_cut_point_straight() -> None:
    print("\n[절단점] 직선 줄기 — 꽃받침에서 정확히 offset만큼 떨어진 점")
    stem = _straight_stem()
    fruit = _fruit_at_top()
    px_per_mm = 1.0  # 계산 단순화를 위해 1px=1mm로 둔 시험 좌표계
    cut = find_cut_point(stem, fruit, px_per_mm=px_per_mm, cut_offset_mm=12.0)
    check("절단점을 찾는다", cut is not None, f"{cut}")
    if cut is not None:
        check("절단점이 꽃받침(y≈0)에서 약 12px 아래(y≈12)",
              abs(cut.v - 12.0) <= 2.0, f"v={cut.v:.1f}")
        check("접선이 줄기 방향(수직, |ty|가 |tx|보다 훨씬 큼)",
              abs(cut.tangent[1]) > abs(cut.tangent[0]) * 3,
              f"tangent={cut.tangent}")
        check("접선이 단위 벡터", abs(np.hypot(*cut.tangent) - 1.0) < 1e-6,
              f"{np.hypot(*cut.tangent):.4f}")


def test_find_cut_point_rejections() -> None:
    print("\n[절단점] 거절 조건 — 없는 것을 만들어 내지 않는가")
    empty = np.zeros((60, 100), dtype=bool)
    fruit = _fruit_at_top()
    check("줄기 마스크가 비어 있으면 None", find_cut_point(empty, fruit, 1.0) is None)

    stem = _straight_stem()
    check("과실 마스크가 비어 있으면 None", find_cut_point(stem, empty, 1.0) is None)

    # 과실과 줄기가 안 맞닿는 경우 (동떨어진 두 마스크)
    far_fruit = np.zeros((60, 100), dtype=bool)
    far_fruit[40:50, 10:20] = True
    check("과실과 줄기가 안 맞닿으면 None(분할 어긋남 방어)",
          find_cut_point(stem, far_fruit, 1.0) is None)

    short_stem = _straight_stem(length=5)  # offset(12mm)보다 훨씬 짧다
    short_fruit = np.zeros((5, 100), dtype=bool)
    yy, xx = np.ogrid[:5, :100]
    short_fruit[((xx - 50) ** 2 + yy ** 2) <= 15 ** 2] = True
    check("줄기가 offset보다 짧으면 None(억지로 만들지 않는다)",
          find_cut_point(short_stem, short_fruit, 1.0, cut_offset_mm=12.0) is None)

    check("px_per_mm <= 0이면 None(무효 축척 거절)",
          find_cut_point(stem, fruit, px_per_mm=0.0) is None and
          find_cut_point(stem, fruit, px_per_mm=-1.0) is None)
    check("cut_offset_mm <= 0이면 None(비물리적 오프셋 거절)",
          find_cut_point(stem, fruit, px_per_mm=1.0, cut_offset_mm=0.0) is None and
          find_cut_point(stem, fruit, px_per_mm=1.0, cut_offset_mm=-5.0) is None)


def test_offset_scales_with_px_per_mm() -> None:
    print("\n[단위] px_per_mm 환산이 실제로 거리에 반영된다")
    stem = _straight_stem(length=80)
    fruit = _fruit_at_top(length=80)
    cut_close = find_cut_point(stem, fruit, px_per_mm=0.5, cut_offset_mm=12.0)
    cut_far = find_cut_point(stem, fruit, px_per_mm=2.0, cut_offset_mm=12.0)
    check("px_per_mm이 클수록(카메라에 가까울수록) 절단점이 화면상 더 멀리(큰 v)",
          cut_close is not None and cut_far is not None and cut_far.v > cut_close.v,
          f"close.v={cut_close.v if cut_close else None} far.v={cut_far.v if cut_far else None}")


def test_sample_stem_depth() -> None:
    print("\n[깊이 평활화] sample_stem_depth")
    dmap = np.zeros((100, 100), dtype=np.float32)
    # 절단 화소 (50, 50) 주변에 유효 깊이와 노이즈(0 및 이상치) 배치
    dmap[48:53, 48:53] = 0.0  # 기본 0 (결손)
    dmap[49, 50] = 210.0
    dmap[50, 50] = 200.0
    dmap[51, 50] = 190.0
    dmap[50, 51] = 950.0      # 상한(900mm) 초과 이상치
    dmap[50, 49] = 30.0       # 하한(60mm) 미만 이상치

    depth = sample_stem_depth(dmap, 50.0, 50.0, window_radius=2)
    check("결손(0) 및 유효범위 밖 이상치를 배제하고 정상 중앙값(200.0mm)을 얻는다",
          depth is not None and abs(depth - 200.0) < 1e-4, f"depth={depth}")

    # 모든 화소가 0인 영역
    zero_map = np.zeros((50, 50), dtype=np.float32)
    check("유효 깊이가 전무한 영역은 None을 반환한다",
          sample_stem_depth(zero_map, 25.0, 25.0) is None)

    # 영상 경계 밖
    check("영상 경계 밖 화소는 None을 반환한다",
          sample_stem_depth(dmap, -10.0, 50.0) is None and
          sample_stem_depth(dmap, 50.0, 150.0) is None)

    # 비수치(NaN/Inf) 및 인자 결함
    check("화소 좌표가 NaN 또는 Inf이면 None(비수치 거절)",
          sample_stem_depth(dmap, float('nan'), 50.0) is None and
          sample_stem_depth(dmap, 50.0, float('inf')) is None)
    check("depth_map이 None 또는 빈 배열이면 None(입력 결함 방어)",
          sample_stem_depth(None, 50.0, 50.0) is None and
          sample_stem_depth(np.zeros((0, 0), dtype=np.float32), 0, 0) is None)
    check("window_radius < 0이면 None(유효 범위 거절)",
          sample_stem_depth(dmap, 50.0, 50.0, window_radius=-1) is None)


class _DummyIntrinsics:
    def __init__(self, fx=400.0, fy=400.0, ppx=50.0, ppy=50.0):
        self.fx = fx
        self.fy = fy
        self.ppx = ppx
        self.ppy = ppy

    def deproject(self, u: float, v: float, z_mm: float) -> tuple[float, float, float]:
        x = (u - self.ppx) * z_mm / self.fx
        y = (v - self.ppy) * z_mm / self.fy
        return (float(x), float(y), float(z_mm))


def test_compute_cutting_pose() -> None:
    print("\n[6-DoF 절단 포즈] compute_cutting_pose")
    intr = _DummyIntrinsics(fx=400.0, fy=400.0, ppx=50.0, ppy=50.0)

    # 1. 수직 하향 줄기 (tangent = [0, 1])
    cut_straight = CutPoint(u=50.0, v=60.0, tangent=(0.0, 1.0))
    pose = compute_cutting_pose(cut_straight, depth_mm=200.0, intr=intr)
    check("수직 줄기에 대해 6-DoF CutPose3D를 산출한다", pose is not None)
    if pose is not None:
        check("3D 절단 위치가 광학계 deproject 결과(0, 5, 200)와 일치한다",
              abs(pose.position_mm[0] - 0.0) < 1e-4 and
              abs(pose.position_mm[1] - 5.0) < 1e-4 and
              abs(pose.position_mm[2] - 200.0) < 1e-4,
              f"pos={pose.position_mm}")
        check("줄기 축 z_cut이 [0, 1, 0]이다",
              abs(pose.z_cut[0] - 0.0) < 1e-4 and
              abs(pose.z_cut[1] - 1.0) < 1e-4 and
              abs(pose.z_cut[2] - 0.0) < 1e-4,
              f"z_cut={pose.z_cut}")
        check("접근 벡터 x_cut이 [-1, 0, 0]이다 (시선 v_cam과 z_cut의 외적)",
              abs(pose.x_cut[0] - (-1.0)) < 1e-4 and
              abs(pose.x_cut[1] - 0.0) < 1e-4 and
              abs(pose.x_cut[2] - 0.0) < 1e-4,
              f"x_cut={pose.x_cut}")
        check("날 정렬 y_cut이 [0, 0, 1]이다 (z_cut x x_cut)",
              abs(pose.y_cut[0] - 0.0) < 1e-4 and
              abs(pose.y_cut[1] - 0.0) < 1e-4 and
              abs(pose.y_cut[2] - 1.0) < 1e-4,
              f"y_cut={pose.y_cut}")
        r = pose.rotation_matrix
        det_r = float(np.linalg.det(r))
        check("회전 행렬이 우수계 SO(3) 직교 기저(det=1.0)를 만족한다",
              abs(det_r - 1.0) < 1e-6, f"det={det_r:.6f}")
        check("회전 행렬의 직교성(R.T @ R = I)이 성립한다",
              np.allclose(r.T @ r, np.eye(3), atol=1e-6))

    # 2. 사선 줄기 (tangent = [0.6, 0.8])
    cut_slanted = CutPoint(u=50.0, v=50.0, tangent=(0.6, 0.8))
    pose_slant = compute_cutting_pose(cut_slanted, depth_mm=300.0, intr=intr)
    check("사선 줄기에 대해 6-DoF 절단 포즈를 산출한다", pose_slant is not None)
    if pose_slant is not None:
        r_s = pose_slant.rotation_matrix
        check("사선 줄기 회전 행렬 det=1.0 및 직교성을 만족한다",
              abs(np.linalg.det(r_s) - 1.0) < 1e-6 and np.allclose(r_s.T @ r_s, np.eye(3), atol=1e-6),
              f"det={np.linalg.det(r_s):.6f}")
        check("접근 벡터 x_cut이 카메라 시선 v_cam=[0,0,1]과 완벽히 직교한다 (x_z=0)",
              abs(pose_slant.x_cut[2]) < 1e-6, f"x_z={pose_slant.x_cut[2]}")
        check("날 정렬 y_cut이 카메라 전방 방향(y_z > 0)을 향한다",
              pose_slant.y_cut[2] > 0.0, f"y_z={pose_slant.y_cut[2]:.4f}")

    # 3. 3D 줄기 자세 (tangent = [0.6, 0.0, 0.8])
    cut_3d = CutPoint(u=50.0, v=50.0, tangent=(0.6, 0.0, 0.8))
    pose_3d = compute_cutting_pose(cut_3d, depth_mm=250.0, intr=intr)
    check("3D 줄기 접선에 대해 6-DoF 절단 포즈를 산출한다", pose_3d is not None)
    if pose_3d is not None:
        r_3d = pose_3d.rotation_matrix
        check("3D 줄기 회전 행렬이 SO(3)(det=1.0, R.T@R=I)를 만족한다",
              abs(np.linalg.det(r_3d) - 1.0) < 1e-6 and np.allclose(r_3d.T @ r_3d, np.eye(3), atol=1e-6))
        check("3D 줄기 접근 벡터 x_cut이 광축 v_cam과 직교한다(x_z=0)",
              abs(pose_3d.x_cut[2]) < 1e-6)

    # 4. 거절 조건 (무효 깊이, 특이점 및 입력 결함)
    check("depth_mm <= 0.0은 None(무효 깊이 거절)",
          compute_cutting_pose(cut_straight, depth_mm=0.0, intr=intr) is None and
          compute_cutting_pose(cut_straight, depth_mm=-50.0, intr=intr) is None)
    zero_tangent = CutPoint(u=50.0, v=50.0, tangent=(0.0, 0.0))
    check("접선 크기가 0이면 None(방향 부재 거절)",
          compute_cutting_pose(zero_tangent, depth_mm=200.0, intr=intr) is None)
    singularity_tangent = CutPoint(u=50.0, v=50.0, tangent=(0.0, 0.0, 1.0))
    check("줄기가 카메라 광축과 평행하면 None(특이점 거절)",
          compute_cutting_pose(singularity_tangent, depth_mm=200.0, intr=intr) is None)
    check("cut_point가 None이면 None(결측 방어)",
          compute_cutting_pose(None, depth_mm=200.0, intr=intr) is None)
    check("depth_mm이 NaN 또는 Inf이면 None(비수치 거절)",
          compute_cutting_pose(cut_straight, depth_mm=float('nan'), intr=intr) is None and
          compute_cutting_pose(cut_straight, depth_mm=float('inf'), intr=intr) is None)


def test_pre_grasp_and_retract() -> None:
    print("\n[모션 위치] compute_pre_grasp_pose & compute_retract_pose")
    intr = _DummyIntrinsics(fx=400.0, fy=400.0, ppx=50.0, ppy=50.0)
    cut = CutPoint(u=50.0, v=60.0, tangent=(0.0, 1.0))
    pose = compute_cutting_pose(cut, depth_mm=200.0, intr=intr)
    assert pose is not None

    # 1. Pre-grasp (기본 standoff = 50.0mm, x_cut = [-1, 0, 0])
    pre_pos = compute_pre_grasp_pose(pose, standoff_mm=50.0)
    check("Pre-grasp 위치가 접근 벡터 반대 방향으로 정확히 50mm 후퇴한다 (50, 5, 200)",
          pre_pos is not None and
          abs(pre_pos[0] - 50.0) < 1e-4 and
          abs(pre_pos[1] - 5.0) < 1e-4 and
          abs(pre_pos[2] - 200.0) < 1e-4,
          f"pre={pre_pos}")

    # 2. Retract (기본 retract = 60.0mm)
    ret_pos = compute_retract_pose(pose, retract_mm=60.0)
    check("Retract 위치가 접근 벡터 반대 방향으로 정확히 60mm 후퇴한다 (60, 5, 200)",
          ret_pos is not None and
          abs(ret_pos[0] - 60.0) < 1e-4 and
          abs(ret_pos[1] - 5.0) < 1e-4 and
          abs(ret_pos[2] - 200.0) < 1e-4,
          f"ret={ret_pos}")

    # 3. 거절 조건 (결측 및 비수치)
    check("cut_pose=None 시 Pre-grasp/Retract 모두 None",
          compute_pre_grasp_pose(None) is None and compute_retract_pose(None) is None)
    check("거리 음수(<0) 또는 NaN 시 거절",
          compute_pre_grasp_pose(pose, standoff_mm=-10.0) is None and
          compute_retract_pose(pose, retract_mm=float('nan')) is None)


def test_transform_cut_pose() -> None:
    print("\n[좌표계 변환] transform_cut_pose")
    intr = _DummyIntrinsics(fx=400.0, fy=400.0, ppx=50.0, ppy=50.0)
    cut = CutPoint(u=50.0, v=60.0, tangent=(0.0, 1.0))
    pose = compute_cutting_pose(cut, depth_mm=200.0, intr=intr)
    assert pose is not None

    # Z축 기준 +90도 회전 및 평행이동 (100, 200, 300)
    R_z90 = np.array([[0.0, -1.0, 0.0],
                      [1.0,  0.0, 0.0],
                      [0.0,  0.0, 1.0]], dtype=np.float64)
    t_vec = np.array([100.0, 200.0, 300.0], dtype=np.float64)

    # 1. 튜플 (R, t) 입력
    t_pose = transform_cut_pose(pose, (R_z90, t_vec))
    check("강체 변환 후 CutPose3D를 산출한다", t_pose is not None)
    if t_pose is not None:
        check("변환 후 위치가 정확히 일치한다 (95, 200, 500)",
              abs(t_pose.position_mm[0] - 95.0) < 1e-4 and
              abs(t_pose.position_mm[1] - 200.0) < 1e-4 and
              abs(t_pose.position_mm[2] - 500.0) < 1e-4,
              f"pos={t_pose.position_mm}")
        check("접근 벡터 x_cut이 회전되어 [0, -1, 0]이다",
              abs(t_pose.x_cut[0] - 0.0) < 1e-4 and
              abs(t_pose.x_cut[1] - (-1.0)) < 1e-4 and
              abs(t_pose.x_cut[2] - 0.0) < 1e-4,
              f"x_cut={t_pose.x_cut}")
        r_new = t_pose.rotation_matrix
        check("변환 후 회전 행렬이 SO(3)(det=1.0, R.T@R=I)를 엄밀히 유지한다",
              abs(float(np.linalg.det(r_new)) - 1.0) < 1e-6 and
              np.allclose(r_new.T @ r_new, np.eye(3), atol=1e-6))

    # 2. 4x4 동차 행렬 입력
    T_mat = np.eye(4, dtype=np.float64)
    T_mat[:3, :3] = R_z90
    T_mat[:3, 3] = t_vec
    t_mat_pose = transform_cut_pose(pose, T_mat)
    check("4x4 동차 변환 행렬 입력으로도 동일한 변환 결과를 산출한다",
          t_mat_pose is not None and
          abs(t_mat_pose.position_mm[0] - 95.0) < 1e-4)

    # 3. 비직교/비SO(3) 행렬 거절
    R_bad = np.array([[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])  # det=2.0 (스케일 변형)
    check("스케일 변형 또는 비SO(3) 변환은 None(강체 불변성 위배 거절)",
          transform_cut_pose(pose, (R_bad, t_vec)) is None)
    check("cut_pose=None 또는 transform 결함 시 None",
          transform_cut_pose(None, (R_z90, t_vec)) is None and
          transform_cut_pose(pose, "invalid_transform") is None)


def test_evaluate_5dof_cut_alignment() -> None:
    print("\n[5-DoF 기구학 진단] evaluate_5dof_cut_alignment")
    # 1. 완벽 정합 사례 (pan = 30°, pitch = 15°, approach가 arm 평면과 완전 일치)
    pan = math.radians(30.0)
    pitch = math.radians(15.0)
    x_aligned = (math.cos(pitch) * math.cos(pan), math.cos(pitch) * math.sin(pan), math.sin(pitch))
    l_0 = (-math.sin(pan), math.cos(pan), 0.0)
    u_0 = (-math.sin(pitch) * math.cos(pan), -math.sin(pitch) * math.sin(pan), math.cos(pitch))
    y_aligned = l_0
    z_aligned = tuple(np.cross(x_aligned, y_aligned))
    r_aligned = np.column_stack([x_aligned, y_aligned, z_aligned])

    pose_aligned = CutPose3D(
        position_mm=(float(200.0 * math.cos(pan)), float(200.0 * math.sin(pan)), 100.0),
        rotation_matrix=r_aligned,
        x_cut=x_aligned,
        y_cut=y_aligned,
        z_cut=z_aligned,
        depth_mm=200.0,
    )
    diag1 = evaluate_5dof_cut_alignment(pose_aligned)
    check("완벽 정합 자세에 대해 5-DoF 진단 결과를 산출한다", diag1 is not None)
    if diag1 is not None:
        check("pan 각도가 30.0도와 일치한다", abs(diag1["pan_deg"] - 30.0) < 1e-4)
        check("pitch 각도가 15.0도와 일치한다", abs(diag1["pitch_deg"] - 15.0) < 1e-4)
        check("yaw 편차 및 3D 정렬 오차가 0.0도이다",
              abs(diag1["yaw_mismatch_deg"]) < 1e-4 and abs(diag1["alignment_angle_deg"]) < 1e-4)
        check("최적 롤 각도가 0.0도이며 가동범위 내이다",
              abs(diag1["optimal_roll_deg"]) < 1e-4 and diag1["within_roll_limits"] is True)
        check("수직 하향 자세가 아니다 (is_vertical_down=False)", diag1["is_vertical_down"] is False)

    # 2. 180도 커터 날 대칭 및 케이블 감김 한계 해결 사례 (optimal_roll = 120° -> -60°)
    y_120 = tuple(math.cos(math.radians(120.0)) * np.array(l_0) + math.sin(math.radians(120.0)) * np.array(u_0))
    z_120 = tuple(np.cross(x_aligned, y_120))
    r_120 = np.column_stack([x_aligned, y_120, z_120])
    pose_120 = CutPose3D(
        position_mm=pose_aligned.position_mm,
        rotation_matrix=r_120,
        x_cut=x_aligned,
        y_cut=y_120,
        z_cut=z_120,
        depth_mm=200.0,
    )
    diag2 = evaluate_5dof_cut_alignment(pose_120, roll_limit_deg=97.9)
    check("120도 최적 롤에 대해 180도 대칭 보정 후 -60도를 도출한다",
          diag2 is not None and
          abs(diag2["optimal_roll_deg"] - 120.0) < 1e-4 and
          abs(diag2["achievable_roll_deg"] - (-60.0)) < 1e-4 and
          diag2["within_roll_limits"] is True)

    # 3. 횡방향 접근 (yaw 편차 90°, 5-DoF 정렬 오차 90°)
    x_lateral = (-math.sin(pan), math.cos(pan), 0.0)  # 횡방향 직교
    y_lateral = (0.0, 0.0, 1.0)
    z_lateral = tuple(np.cross(x_lateral, y_lateral))
    pose_lateral = CutPose3D(
        position_mm=pose_aligned.position_mm,
        rotation_matrix=np.column_stack([x_lateral, y_lateral, z_lateral]),
        x_cut=x_lateral,
        y_cut=y_lateral,
        z_cut=z_lateral,
        depth_mm=200.0,
    )
    diag_lat = evaluate_5dof_cut_alignment(pose_lateral)
    check("횡방향 직교 접근 시 정렬 오차가 90도(차체 회전 필요)로 정확히 진단된다",
          diag_lat is not None and abs(diag_lat["alignment_angle_deg"] - 90.0) < 1e-4)

    # 4. 수직 하향 자세 예외 (pitch = -90°, is_vertical_down=True)
    x_vert = (0.0, 0.0, -1.0)
    y_vert = (0.0, 1.0, 0.0)
    z_vert = (1.0, 0.0, 0.0)
    pose_vert = CutPose3D(
        position_mm=(150.0, 0.0, 50.0),
        rotation_matrix=np.column_stack([x_vert, y_vert, z_vert]),
        x_cut=x_vert,
        y_cut=y_vert,
        z_cut=z_vert,
        depth_mm=200.0,
    )
    diag_vert = evaluate_5dof_cut_alignment(pose_vert)
    check("수직 하향 자세(pitch=-90°)에서 is_vertical_down=True(Roll=Yaw 예외)를 판정한다",
          diag_vert is not None and diag_vert["is_vertical_down"] is True)

    # 5. 특이점 및 결측 거절
    pose_zero_r = CutPose3D(
        position_mm=(0.0, 0.0, 100.0),
        rotation_matrix=r_aligned,
        x_cut=x_aligned, y_cut=y_aligned, z_cut=z_aligned, depth_mm=200.0,
    )
    check("pan 원점 특이점(r_xy=0)은 None(특이점 거절)",
          evaluate_5dof_cut_alignment(pose_zero_r) is None)
    check("cut_pose_base=None 시 None(결측 방어)",
          evaluate_5dof_cut_alignment(None) is None)


def main() -> int:
    test_thinning()
    test_skeleton_points()
    test_find_cut_point_straight()
    test_find_cut_point_rejections()
    test_offset_scales_with_px_per_mm()
    test_sample_stem_depth()
    test_compute_cutting_pose()
    test_pre_grasp_and_retract()
    test_transform_cut_pose()
    test_evaluate_5dof_cut_alignment()

    print(f"\n{'='*60}")
    if FAILED:
        print(f"❌ {len(FAILED)}개 실패 / {PASSED + len(FAILED)}개 중")
        for name in FAILED:
            print(f"   - {name}")
        return 1
    print(f"✅ 전부 통과 ({PASSED}개)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
