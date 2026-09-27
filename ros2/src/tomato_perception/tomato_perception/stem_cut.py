"""줄기(peduncle) 2D 마스크 → 절단점 3D 좌표. numpy만 쓴다(cv2도 rclpy도 없다).

study 문서(`docs/study/04_END_EFFECTOR_MANIPULATION.md` §2)가 제안하는 방식을
그대로 따른다: 3D 점군에서 직접 원통을 피팅하면 줄기(1~3mm)가 노이즈에 묻혀
끊긴다(2026-09-11 개발일지 — 가는 표적은 조각 정합이 배경을 따라간다, 실측
0/10). 대신 **2D 세그멘테이션 마스크에서 먼저 스켈레톤을 뽑고, 그 위의 한 점만
깊이와 역투영**하면 깊이 노이즈가 스켈레톤 추출에 섞이지 않는다.

절단점 선정: 과실(꽃받침) 마스크와 줄기 마스크가 맞닿는 지점에서 줄기를 따라
`CUT_OFFSET_MM`(기본 12mm, 논문 실무치 10~15mm 중간값)만큼 떨어진 스켈레톤
화소. 그 지점의 접선 벡터가 절단 날 정렬(y_cut)의 기준이 된다.

⚠ 이 모듈은 "스켈레톤에서 절단점을 뽑는 계산"만 한다 — 마스크를 누가 주느냐는
   상관없다(`yolo_seg.py`의 줄기 클래스 마스크를 기대하지만 결합돼 있지 않다).
"""

from __future__ import annotations

import math
from typing import Any

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CutPoint:
    """줄기 위 절단점 — 아직 3D 아님(픽셀 좌표 + 접선 방향)."""

    u: float
    v: float
    tangent: tuple[float, float] | tuple[float, float, float]  # 단위 벡터 (2D 화면 좌표계 또는 3D 광학 좌표계)


@dataclass(frozen=True)
class CutPose3D:
    """줄기 위 3D 절단점 및 6-DoF 절단 자세 (카메라 광학 좌표계 기준).

    docs/study/04_END_EFFECTOR_MANIPULATION.md §2.2 명세 준수:
    - position_mm: (x, y, z) 3D 절단 위치 (mm)
    - rotation_matrix: 3x3 직교 회전 행렬 [x_cut, y_cut, z_cut]
    - x_cut: 진입/접근(approach) 단위 벡터 (v_cam과 z_cut에 수직인 횡방향)
    - y_cut: 전단 날 정렬(blade alignment) 단위 벡터 (z_cut x x_cut)
    - z_cut: 줄기 진행 축(stem axis) 단위 벡터 (t_stem)
    - depth_mm: 절단점 깊이 (mm)
    """

    position_mm: tuple[float, float, float]
    rotation_matrix: np.ndarray  # shape (3, 3)
    x_cut: tuple[float, float, float]
    y_cut: tuple[float, float, float]
    z_cut: tuple[float, float, float]
    depth_mm: float


def zhang_suen_thin(mask: np.ndarray) -> np.ndarray:
    """이진 마스크(bool) → 1픽셀 두께 스켈레톤(bool). Zhang-Suen 세선화.

    cv2.ximgproc가 있으면 더 빠르지만 opencv-contrib 의존을 늘리지 않으려고
    표준 알고리즘을 직접 짠다 — 마스크가 보통 수백 화소라 순수 파이썬 루프도
    실용적 속도로 끝난다(자체검증 기준 100x100 마스크 <50ms).
    """
    img = mask.astype(np.uint8).copy()
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            padded = np.pad(img, 1)
            p2 = padded[0:-2, 1:-1]
            p3 = padded[0:-2, 2:]
            p4 = padded[1:-1, 2:]
            p5 = padded[2:, 2:]
            p6 = padded[2:, 1:-1]
            p7 = padded[2:, 0:-2]
            p8 = padded[1:-1, 0:-2]
            p9 = padded[0:-2, 0:-2]
            neighbors = [p2, p3, p4, p5, p6, p7, p8, p9]
            b = sum(neighbors)
            seq = neighbors + [p2]
            a = np.zeros_like(img, dtype=np.int32)
            for i in range(8):
                a += ((seq[i] == 0) & (seq[i + 1] == 1)).astype(np.int32)

            cond_base = (img == 1) & (b >= 2) & (b <= 6) & (a == 1)
            if step == 0:
                cond = cond_base & ((p2 * p4 * p6) == 0) & ((p4 * p6 * p8) == 0)
            else:
                cond = cond_base & ((p2 * p4 * p8) == 0) & ((p2 * p6 * p8) == 0)

            if cond.any():
                img[cond] = 0
                changed = True
    return img.astype(bool)


def skeleton_points(skeleton: np.ndarray) -> np.ndarray:
    """스켈레톤의 화소 좌표를 (N,2) [u,v] 배열로. 비어 있으면 shape (0,2)."""
    ys, xs = np.nonzero(skeleton)
    return np.stack([xs, ys], axis=1).astype(np.float64)


def _nearest_index(points: np.ndarray, target: np.ndarray) -> int:
    d2 = np.sum((points - target) ** 2, axis=1)
    return int(np.argmin(d2))


def find_cut_point(
    stem_mask: np.ndarray,
    fruit_mask: np.ndarray,
    px_per_mm: float,
    cut_offset_mm: float = 12.0,
) -> CutPoint | None:
    """줄기+과실 마스크 → 절단점. 못 찾으면 None(안 만들어 낸다).

    px_per_mm: 그 깊이에서의 화소/mm 환산(`fx / depth_mm` 등, 호출부 책임).
    실패 사유:
      - 줄기 스켈레톤이 비어 있다 (마스크가 너무 작거나 끊겼다)
      - 과실과 줄기가 맞닿는 화소가 없다 (분할이 어긋났다)
      - 스켈레톤이 꽃받침 접점에서 cut_offset_mm만큼 뻗어나갈 만큼 길지 않다
    """
    if px_per_mm <= 0.0 or cut_offset_mm <= 0.0:
        return None
    if stem_mask.shape != fruit_mask.shape:
        return None  # 다른 프레임/해상도의 마스크를 섞은 것 — 조용히 계산하지 않는다
    if stem_mask.sum() == 0 or fruit_mask.sum() == 0:
        return None

    skel = zhang_suen_thin(stem_mask)
    pts = skeleton_points(skel)
    if len(pts) < 2:
        return None

    # 과실 마스크를 1화소 팽창해 스켈레톤과의 접점(꽃받침 근접점)을 찾는다.
    # cv2.dilate 없이 4방향 시프트 OR로 대체 — 의존성을 늘리지 않는다.
    grown = fruit_mask.copy()
    grown[1:, :] |= fruit_mask[:-1, :]
    grown[:-1, :] |= fruit_mask[1:, :]
    grown[:, 1:] |= fruit_mask[:, :-1]
    grown[:, :-1] |= fruit_mask[:, 1:]

    contact = grown[pts[:, 1].astype(int), pts[:, 0].astype(int)]
    if not contact.any():
        return None
    calyx_idx = np.nonzero(contact)[0]
    calyx_pt = pts[calyx_idx[0]]

    # 꽃받침 접점에서 스켈레톤을 따라 누적 거리로 offset_px만큼 걸어간다.
    offset_px = cut_offset_mm * px_per_mm
    order = [int(calyx_idx[0])]
    remaining = list(range(len(pts)))
    remaining.remove(order[0])
    cur = calyx_pt
    total = 0.0
    while remaining:
        nxt_local = _nearest_index(pts[remaining], cur)
        nxt = remaining[nxt_local]
        step = float(np.linalg.norm(pts[nxt] - cur))
        if step > 3.0:  # 스켈레톤이 여기서 끊겼다(다른 가지) — 더 안 간다
            break
        total += step
        cur = pts[nxt]
        order.append(nxt)
        remaining.pop(nxt_local)
        if total >= offset_px:
            break

    if total < offset_px:
        return None  # 줄기가 절단 지점까지 닿을 만큼 길게 안 보인다

    cut_idx = order[-1]
    cut_pt = pts[cut_idx]

    # 접선 = 절단점 주변 몇 화소의 평균 방향(국소 노이즈에 덜 민감하게).
    lo = max(0, len(order) - 4)
    seg = pts[order[lo:]]
    if len(seg) < 2:
        tangent = np.array([1.0, 0.0])
    else:
        tangent = seg[-1] - seg[0]
    norm = float(np.linalg.norm(tangent))
    tangent = tangent / norm if norm > 1e-9 else np.array([1.0, 0.0])

    return CutPoint(u=float(cut_pt[0]), v=float(cut_pt[1]),
                     tangent=(float(tangent[0]), float(tangent[1])))


def sample_stem_depth(
    depth_map: np.ndarray,
    u: float,
    v: float,
    window_radius: int = 3,
    min_depth_mm: float = 60.0,
    max_depth_mm: float = 900.0,
) -> float | None:
    """줄기 절단점 주변 국소 깊이 평활화 (Median Depth).

    1~3mm 직경의 가는 줄기는 깊이 맵 결손(0값)이나 배경 난반사가 잦으므로,
    절단 화소 (u, v) 주변 (2*radius+1)^2 영역에서 유효 대역([min, max] mm)
    화소들의 중앙값을 취한다. 유효 화소가 하나도 없으면 None 반환.
    """
    if depth_map is None or not isinstance(depth_map, np.ndarray) or depth_map.size == 0:
        return None
    if not (isinstance(u, (int, float)) and isinstance(v, (int, float)) and
            math.isfinite(u) and math.isfinite(v)):
        return None
    if window_radius < 0:
        return None
    h, w = depth_map.shape[:2]
    iu = int(round(u))
    iv = int(round(v))
    u_min = max(0, iu - window_radius)
    u_max = min(w, iu + window_radius + 1)
    v_min = max(0, iv - window_radius)
    v_max = min(h, iv + window_radius + 1)

    if u_min >= u_max or v_min >= v_max:
        return None

    patch = depth_map[v_min:v_max, u_min:u_max].astype(np.float64)
    valid = patch[(patch >= min_depth_mm) & (patch <= max_depth_mm) & np.isfinite(patch)]
    if len(valid) == 0:
        return None
    return float(np.median(valid))


def compute_cutting_pose(
    cut_point: CutPoint,
    depth_mm: float,
    intr: Any,
) -> CutPose3D | None:
    """2D/3D 절단점(CutPoint) + 깊이(mm) + 카메라 내부파라미터 → 6-DoF 절단 자세.

    docs/study/04_END_EFFECTOR_MANIPULATION.md §2.2 수학 공식 구현:
    1. 3D 역투영: P_cut = deproject(u, v, depth_mm) (카메라 광학 좌표계)
    2. 줄기 축 단위 벡터 z_cut: 접선 (tu, tv[, tz]) 정규화 (t_stem)
    3. 접근 벡터 x_cut: 카메라 시선 v_cam=(0,0,1)과 z_cut의 외적 정규화
       x_cut = (v_cam x z_cut) / ||v_cam x z_cut||
    4. 전단 날 정렬 벡터 y_cut: y_cut = z_cut x x_cut
    5. 회전 행렬 R_cut = [x_cut, y_cut, z_cut] (우수계 SO(3), det(R) = +1.0)

    거절 사유:
    - cut_point is None
    - not math.isfinite(depth_mm) or depth_mm <= 0.0 (무효 깊이)
    - ||v_cam x z_cut|| < 1e-4 (줄기가 카메라 광축과 평행한 특이점)
    """
    if cut_point is None:
        return None
    if not (isinstance(depth_mm, (int, float)) and math.isfinite(depth_mm) and depth_mm > 0.0):
        return None
    if not (isinstance(cut_point.u, (int, float)) and isinstance(cut_point.v, (int, float)) and
            math.isfinite(cut_point.u) and math.isfinite(cut_point.v)):
        return None

    # 1. 3D 역투영
    if hasattr(intr, "deproject"):
        p_cut = intr.deproject(cut_point.u, cut_point.v, depth_mm)
    else:
        fx = getattr(intr, "fx", 438.0)
        fy = getattr(intr, "fy", 438.0)
        ppx = getattr(intr, "ppx", 424.0)
        ppy = getattr(intr, "ppy", 240.0)
        if fx <= 0.0 or fy <= 0.0:
            return None
        x = (cut_point.u - ppx) * depth_mm / fx
        y = (cut_point.v - ppy) * depth_mm / fy
        p_cut = (float(x), float(y), float(depth_mm))

    # 2. 줄기 축 벡터 z_cut (2D 접선 또는 3D 접선)
    if not hasattr(cut_point, "tangent") or cut_point.tangent is None:
        return None
    t = cut_point.tangent
    if len(t) == 2:
        zx, zy, zz = float(t[0]), float(t[1]), 0.0
    elif len(t) == 3:
        zx, zy, zz = float(t[0]), float(t[1]), float(t[2])
    else:
        return None
    if not (math.isfinite(zx) and math.isfinite(zy) and math.isfinite(zz)):
        return None

    norm_3d = float(np.hypot(np.hypot(zx, zy), zz))
    if norm_3d < 1e-9:
        return None
    z_cut = np.array([zx / norm_3d, zy / norm_3d, zz / norm_3d], dtype=np.float64)

    # 3. 접근 벡터 x_cut (v_cam = [0, 0, 1])
    # v_cam x z_cut = [-z_cut[1], z_cut[0], 0.0]
    cross_cam = np.array([-z_cut[1], z_cut[0], 0.0], dtype=np.float64)
    sin_theta = float(np.linalg.norm(cross_cam))
    if sin_theta < 1e-4:
        return None  # 광축 평행 특이점 (v_cam과 z_cut이 평행하여 접근 방향 불능)
    x_cut = cross_cam / sin_theta

    # 4. 가위 날 정렬 벡터 y_cut = z_cut x x_cut
    y_cut = np.cross(z_cut, x_cut)
    y_norm = float(np.linalg.norm(y_cut))
    if y_norm < 1e-9:
        return None
    y_cut = y_cut / y_norm

    # 5. 회전 행렬 R = [x_cut, y_cut, z_cut]
    rot_matrix = np.column_stack([x_cut, y_cut, z_cut])

    return CutPose3D(
        position_mm=(float(p_cut[0]), float(p_cut[1]), float(p_cut[2])),
        rotation_matrix=rot_matrix,
        x_cut=(float(x_cut[0]), float(x_cut[1]), float(x_cut[2])),
        y_cut=(float(y_cut[0]), float(y_cut[1]), float(y_cut[2])),
        z_cut=(float(z_cut[0]), float(z_cut[1]), float(z_cut[2])),
        depth_mm=float(depth_mm),
    )


def compute_pre_grasp_pose(
    cut_pose: CutPose3D | None,
    standoff_mm: float = 50.0,
) -> tuple[float, float, float] | None:
    """절단점 진입 전 대기 위치 (Pre-grasp Pose).

    docs/study/04_END_EFFECTOR_MANIPULATION.md §5 Step 3 명세:
    - 엔드이펙터 접근 벡터(x_cut)의 역방향으로 standoff_mm(기본 50mm) 후퇴한 3D 위치.
    - P_pre = P_cut - standoff_mm * x_cut

    거절:
    - cut_pose is None
    - standoff_mm < 0.0 또는 비수치(NaN/Inf)
    """
    if cut_pose is None:
        return None
    if not (isinstance(standoff_mm, (int, float)) and math.isfinite(standoff_mm) and standoff_mm >= 0.0):
        return None
    px, py, pz = cut_pose.position_mm
    ax, ay, az = cut_pose.x_cut
    if not (all(math.isfinite(v) for v in (px, py, pz)) and all(math.isfinite(v) for v in (ax, ay, az))):
        return None
    return (
        float(px - standoff_mm * ax),
        float(py - standoff_mm * ay),
        float(pz - standoff_mm * az),
    )


def compute_retract_pose(
    cut_pose: CutPose3D | None,
    retract_mm: float = 60.0,
) -> tuple[float, float, float] | None:
    """절단 완료 후 과실 안전 후퇴 위치 (Retract Pose).

    docs/study/04_END_EFFECTOR_MANIPULATION.md §5 Step 9 명세:
    - 절단점으로부터 접근 벡터(x_cut)의 역방향으로 retract_mm(기본 60mm) 후퇴한 3D 위치.
    - P_retract = P_cut - retract_mm * x_cut

    거절:
    - cut_pose is None
    - retract_mm < 0.0 또는 비수치(NaN/Inf)
    """
    if cut_pose is None:
        return None
    if not (isinstance(retract_mm, (int, float)) and math.isfinite(retract_mm) and retract_mm >= 0.0):
        return None
    px, py, pz = cut_pose.position_mm
    ax, ay, az = cut_pose.x_cut
    if not (all(math.isfinite(v) for v in (px, py, pz)) and all(math.isfinite(v) for v in (ax, ay, az))):
        return None
    return (
        float(px - retract_mm * ax),
        float(py - retract_mm * ay),
        float(pz - retract_mm * az),
    )


def transform_cut_pose(
    cut_pose: CutPose3D | None,
    transform: Any,
) -> CutPose3D | None:
    """CutPose3D를 강체 변환(Rigid / 4x4 matrix / (R, t))으로 다른 좌표계로 변환.

    T = (R, t)
    P_new = R @ P + t
    x_new = R @ x_cut
    y_new = R @ y_cut
    z_new = R @ z_cut
    R_new = R @ cut_pose.rotation_matrix

    SO(3) 정규직교 기저(det(R)=1.0, R.T@R=I) 보존.
    """
    if cut_pose is None:
        return None
    try:
        if hasattr(transform, "R") and hasattr(transform, "t"):
            R = np.asarray(transform.R, dtype=np.float64)
            t_raw = np.asarray(transform.t, dtype=np.float64)
        elif isinstance(transform, np.ndarray) and transform.shape == (4, 4):
            R = transform[:3, :3].astype(np.float64)
            t_raw = transform[:3, 3].astype(np.float64)
        elif isinstance(transform, (tuple, list)) and len(transform) == 2:
            R = np.asarray(transform[0], dtype=np.float64)
            t_raw = np.asarray(transform[1], dtype=np.float64)
        else:
            return None

        if R.shape != (3, 3) or t_raw.size != 3:
            return None
        t = t_raw.reshape(3)
    except (ValueError, TypeError, IndexError):
        return None

    if not (np.all(np.isfinite(R)) and np.all(np.isfinite(t))):
        return None
    det_r = float(np.linalg.det(R))
    if abs(det_r - 1.0) > 1e-4:
        return None  # 순수 회전(SO(3))이 아니거나 반사/스케일 왜곡인 경우 거절
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-4):
        return None  # 비직교(전단 shear/비강체) 변환 거절

    p_old = np.array(cut_pose.position_mm, dtype=np.float64)
    if not np.all(np.isfinite(p_old)):
        return None
    p_new = R @ p_old + t

    x_new = R @ np.array(cut_pose.x_cut, dtype=np.float64)
    y_new = R @ np.array(cut_pose.y_cut, dtype=np.float64)
    z_new = R @ np.array(cut_pose.z_cut, dtype=np.float64)
    rot_new = R @ cut_pose.rotation_matrix

    x_norm = float(np.linalg.norm(x_new))
    y_norm = float(np.linalg.norm(y_new))
    z_norm = float(np.linalg.norm(z_new))
    if x_norm < 1e-9 or y_norm < 1e-9 or z_norm < 1e-9:
        return None

    return CutPose3D(
        position_mm=(float(p_new[0]), float(p_new[1]), float(p_new[2])),
        rotation_matrix=rot_new,
        x_cut=(float(x_new[0] / x_norm), float(x_new[1] / x_norm), float(x_new[2] / x_norm)),
        y_cut=(float(y_new[0] / y_norm), float(y_new[1] / y_norm), float(y_new[2] / y_norm)),
        z_cut=(float(z_new[0] / z_norm), float(z_new[1] / z_norm), float(z_new[2] / z_norm)),
        depth_mm=float(cut_pose.depth_mm),
    )


def evaluate_5dof_cut_alignment(
    cut_pose_base: CutPose3D | None,
    roll_limit_deg: float = 97.90,
) -> dict[str, float | bool] | None:
    """5-DoF 매니퓰레이터(SO-101)의 6-DoF 절단 포즈 기구학적 정합성 및 접근 여유각 진단.

    docs/study/04_END_EFFECTOR_MANIPULATION.md §3 명세 준수:
    1. SO-101은 손목 독립 Yaw가 없어, 팔의 접근 방위각(pan)은 위치 (x, y)에 고정된다:
       pan_deg = atan2(y, x)
    2. 원하는 절단 접근 벡터 x_cut과의 수평 방위각 편차:
       yaw_mismatch_deg = atan2(x_cut.y, x_cut.x) - pan_deg
    3. 예외 조건 (is_vertical_down):
       집게가 바닥을 수직으로 내려다보는 자세(pitch <= -75°)일 때,
       wrist_roll이 물리적으로 yaw 회전 역할을 대체 가능.
    4. 3D 정렬 오차각:
       alignment_angle_deg = acos(x_cut · a_arm)
    5. 최적 커터 날 회전각 및 180° 대칭성 해결:
       커터/집게는 180° 반전에 대칭이므로, 케이블 감김 한계([-roll_limit_deg, roll_limit_deg])를
       벗어날 경우 +-180° 보정하여 안전 가동범위 내 각도를 보장한다.
       (span 195.8° >= 180°이므로 수학적으로 항상 존재).
    """
    if cut_pose_base is None:
        return None
    if not (isinstance(roll_limit_deg, (int, float)) and math.isfinite(roll_limit_deg) and roll_limit_deg > 0.0):
        return None
    px, py, pz = cut_pose_base.position_mm
    ax, ay, az = cut_pose_base.x_cut
    bx, by, bz = cut_pose_base.y_cut

    if not (all(math.isfinite(v) for v in (px, py, pz)) and
            all(math.isfinite(v) for v in (ax, ay, az)) and
            all(math.isfinite(v) for v in (bx, by, bz))):
        return None

    r_xy = math.hypot(px, py)
    if r_xy < 1e-4:
        return None  # pan 특이점 (원점 바로 위)

    pan_rad = math.atan2(py, px)
    pan_deg = math.degrees(pan_rad)

    # 절단 접근 벡터의 pitch 각도 (elevation)
    pitch_rad = math.asin(max(-1.0, min(1.0, az)))
    pitch_deg = math.degrees(pitch_rad)

    # 접근 벡터의 수평 방위각 및 pan 편차
    approach_yaw_rad = math.atan2(ay, ax)
    yaw_mismatch_rad = (approach_yaw_rad - pan_rad + math.pi) % (2.0 * math.pi) - math.pi
    yaw_mismatch_deg = math.degrees(yaw_mismatch_rad)

    # 수직 하향 예외 판정 (study 04 §3.1)
    is_vertical_down = bool(pitch_deg <= -75.0)
    if is_vertical_down and math.hypot(ax, ay) < 1e-4:
        # 수직 하향 특이점: 수평 yaw가 정의되지 않으며 wrist_roll이 yaw 역할을 온전히 대체함
        yaw_mismatch_deg = 0.0

    # SO-101 5-DoF 암이 취할 수 있는 단위 접근 벡터
    a_arm = np.array([
        math.cos(pitch_rad) * math.cos(pan_rad),
        math.cos(pitch_rad) * math.sin(pan_rad),
        math.sin(pitch_rad),
    ], dtype=np.float64)

    # 원하는 3D 접근 벡터와 실현 가능 벡터 간의 내적 및 오차각
    x_cut_vec = np.array([ax, ay, az], dtype=np.float64)
    dot_align = float(np.dot(x_cut_vec, a_arm))
    dot_clamped = max(-1.0, min(1.0, dot_align))
    align_angle_deg = math.degrees(math.acos(dot_clamped))

    # 도구 롤 0 자세에서의 lateral 및 up 기저 벡터
    l_0 = np.array([-math.sin(pan_rad), math.cos(pan_rad), 0.0], dtype=np.float64)
    u_0 = np.array([
        -math.sin(pitch_rad) * math.cos(pan_rad),
        -math.sin(pitch_rad) * math.sin(pan_rad),
        math.cos(pitch_rad),
    ], dtype=np.float64)

    y_cut_vec = np.array([bx, by, bz], dtype=np.float64)
    y_l = float(np.dot(y_cut_vec, l_0))
    y_u = float(np.dot(y_cut_vec, u_0))

    opt_roll_rad = math.atan2(y_u, y_l)
    opt_roll_deg = math.degrees(opt_roll_rad)

    # 180도 절단 날 대칭성 및 케이블 감김 한계 해결
    achievable_roll = opt_roll_deg
    if achievable_roll > roll_limit_deg:
        achievable_roll -= 180.0
    elif achievable_roll < -roll_limit_deg:
        achievable_roll += 180.0

    within_limits = bool(-roll_limit_deg <= achievable_roll <= roll_limit_deg)

    return {
        "pan_deg": float(pan_deg),
        "pitch_deg": float(pitch_deg),
        "yaw_mismatch_deg": float(yaw_mismatch_deg),
        "is_vertical_down": is_vertical_down,
        "alignment_angle_deg": float(align_angle_deg),
        "optimal_roll_deg": float(opt_roll_deg),
        "achievable_roll_deg": float(achievable_roll),
        "within_roll_limits": within_limits,
    }
