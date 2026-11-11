#!/usr/bin/env python3
"""**팔 + 손목 카메라 관측 모형** — 관절 읽음값 → "고정 표적의 점이 화면 어디에 찍히나". numpy뿐.

    python ros2/tools/kincal_model.py        # 자체검증

수집기(kincal_collect: 계획·안전), 풀이(kincal_solve: 맞춤), 트윈이 **같은 식**을 쓴다.
기구학 뼈대는 `kinematics.forward` + `handeye.tool_frame_joints` 그대로이고(자체검증이 일치를 본다),
그 위에 실물이 도면과 다른 만큼만 얹는다:

  sgn_pan·sgn_roll  FK와 실물의 거울상(9/4 "평행이동 3~5배"의 원인) — ±1, 풀이가 4조합을 다 본다
  off_*             관절 영점 오차(도). pan은 표적 자세가, roll은 카메라 자세가 흡수하므로 없다
  d0·l1·l2          링크 길이. z0는 표적 높이가, l3는 카메라 평행이동이 흡수하므로 못 푼다(고정)
  sag_*             엔코더가 못 보는 처짐(도 / 중력토크) — 링크·기어가 휘는 몫
  bl_*              백래시(도) × 마지막 접근 방향(±1)
  cam_r·cam_t       T_tool_cam (손-눈)
  board_*·wall_*    T_base_표적 (책상 ChArUco · 벽 4점)
  f_scale·k1        카메라 초점 배율·방사 왜곡(공장값 검사용)
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))
from tomato_picker.hardware import kinematics as kin  # noqa: E402

J = kin.JOINTS
_G = kin.ArmGeometry()
# 중력 토크 모양 — twin_io.gravity와 같은 질량(kg)·손 무게중심(mm). 바꾸면 둘 다.
M1, M_ELBOW, M2, M_HAND, HAND_D = 0.06, 0.055, 0.115, 0.21, 70.0

DEFAULT = {
    "sgn_pan": 1.0, "sgn_roll": 1.0, "roll_scale": 1.0,
    "off_lift": 0.0, "off_elbow": 0.0, "off_wrist": 0.0,
    "z0": _G.z0, "d0": _G.d0, "l1": _G.l1, "l2": _G.l2, "l3": _G.l3,
    "sag_lift": 0.0, "sag_elbow": 0.0, "sag_wrist": 0.0,
    "bl_pan": 0.0, "bl_lift": 0.0, "bl_elbow": 0.0, "bl_wrist": 0.0, "bl_roll": 0.0,
    "cam_r": [0.0, 0.0, 0.0], "cam_t": [0.0, 0.0, 0.0],
    "board_r": [0.0, 0.0, 0.0], "board_t": [0.0, 0.0, 0.0],
    "wall_r": None, "wall_t": None,
    "f_scale": 1.0, "k1": 0.0,
}


def rodrigues(r) -> np.ndarray:
    r = np.asarray(r, float)
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3)
    k = r / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def rotvec(R) -> np.ndarray:
    R = np.asarray(R, float)
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    if np.pi - th < 1e-6:                      # 180° 근처 — 대각에서 축을 뽑는다
        a = np.sqrt(np.maximum((np.diag(R) + 1) / 2, 0))
        i = int(np.argmax(a))
        a = np.sign(R[i] + R[:, i] + 1e-12) * a
        a[i] = abs(a[i])
        return a / np.linalg.norm(a) * th
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(th)) * th


def board_points(spec: dict, scale: float) -> np.ndarray:
    """ChArUco 안쪽 모서리 id → 보드 좌표(mm). OpenCV 규약: id = 행×(nx−1)+열, 자리 = ((열+1)·칸, (행+1)·칸, 0)."""
    sq = spec["square_mm"] * scale
    return np.array([[(c + 1) * sq, (r + 1) * sq, 0.0]
                     for r in range(spec["ny"] - 1) for c in range(spec["nx"] - 1)])


def chain(P: dict, deg, dirs=None) -> dict:
    """관절 읽음값(도, N×5 · `Calib.to_deg` 결과) → 팔 위 점들과 카메라 자세(전부 base 좌표, mm)."""
    q = np.atleast_2d(np.asarray(deg, float))
    d = np.zeros_like(q) if dirs is None else np.atleast_2d(np.asarray(dirs, float))
    pan = np.radians(P["sgn_pan"] * q[:, 0] + P["bl_pan"] * d[:, 0])
    # 처짐: 엔코더가 읽은 각 기준의 중력 토크(자기 자신에 걸린 보정은 2차라 무시)
    a1n = np.radians(q[:, 1])
    a2n = a1n + np.radians(q[:, 2])
    a3n = a2n + np.radians(q[:, 3])
    tw = M_HAND * HAND_D * np.cos(a3n)
    te = tw + (M_HAND + M2 / 2) * P["l2"] * np.cos(a2n)
    tl = te + (M_HAND + M2 + M_ELBOW + M1 / 2) * P["l1"] * np.cos(a1n)
    a1 = np.radians(q[:, 1] + P["off_lift"] - P["sag_lift"] * tl + P["bl_lift"] * d[:, 1])
    a2 = a1 + np.radians(q[:, 2] + P["off_elbow"] - P["sag_elbow"] * te + P["bl_elbow"] * d[:, 2])
    a3 = a2 + np.radians(q[:, 3] + P["off_wrist"] - P["sag_wrist"] * tw + P["bl_wrist"] * d[:, 3])
    roll = np.radians(P["sgn_roll"] * P["roll_scale"] * q[:, 4] + P["bl_roll"] * d[:, 4])
    cp, sp, z = np.cos(pan), np.sin(pan), np.zeros(len(q))

    def at(r, h):
        return np.stack([r * cp, r * sp, h], axis=1)

    r1 = P["d0"] + P["l1"] * np.cos(a1)
    z1 = P["z0"] + P["l1"] * np.sin(a1)
    r2, z2 = r1 + P["l2"] * np.cos(a2), z1 + P["l2"] * np.sin(a2)
    r3, z3 = r2 + P["l3"] * np.cos(a3), z2 + P["l3"] * np.sin(a3)
    approach = np.stack([np.cos(a3) * cp, np.cos(a3) * sp, np.sin(a3)], axis=1)
    lateral = np.stack([-sp, cp, z], axis=1)
    up = np.stack([-np.sin(a3) * cp, -np.sin(a3) * sp, np.cos(a3)], axis=1)
    cr, sr = np.cos(roll)[:, None], np.sin(roll)[:, None]
    lat, upr = lateral * cr + up * sr, -lateral * sr + up * cr
    R_bt = np.stack([approach, lat, upr], axis=2)                 # 열 = approach·lateral·up
    tcp = at(r3, z3)
    R_tc, t_tc = rodrigues(P["cam_r"]), np.asarray(P["cam_t"], float)
    return {"shoulder": at(np.full(len(q), P["d0"]), np.full(len(q), P["z0"])), "elbow": at(r1, z1),
            "wrist": at(r2, z2), "tcp": tcp, "tip": tcp + 35.0 * approach,
            "R_bt": R_bt, "R_bc": R_bt @ R_tc, "t_bc": tcp + R_bt @ t_tc}


def project(P: dict, intr: dict, R_bc, t_bc, X_base, idx=None):
    """base 좌표 점(M×3) → 화소(M×2)와 카메라 z(M). idx = 점마다 어느 자세인가(없으면 자세 하나에 전부)."""
    X = np.atleast_2d(np.asarray(X_base, float))
    if idx is None:
        idx = np.zeros(len(X), int)
    pc = np.einsum("mji,mj->mi", R_bc[idx], X - t_bc[idx])       # R^T (X − t)
    zc = pc[:, 2]
    zs = np.where(np.abs(zc) < 1e-6, 1e-6, zc)
    x, y = pc[:, 0] / zs, pc[:, 1] / zs
    k = 1.0 + P["k1"] * (x * x + y * y)
    uv = np.stack([intr["fx"] * P["f_scale"] * x * k + intr["ppx"],
                   intr["fy"] * P["f_scale"] * y * k + intr["ppy"]], axis=1)
    return uv, zc


def target_base(P: dict, which: str, pts_local) -> np.ndarray:
    return np.asarray(pts_local, float) @ rodrigues(P[which + "_r"]).T + np.asarray(P[which + "_t"], float)


def scene(P: dict, spec: dict, scale: float, dots_wh=None) -> dict:
    """트윈 3D 미리보기에 그릴 장면 — 책상 평면·보드 종이·(알면) 벽 종이. 사각형 꼭짓점 mm.

    ⚠ 좌표는 **도면 FK 좌표**다(조작대 JS의 fk()·kinematics.forward가 그리는 그 틀). 실물이 FK와 거울상이면
      (sgn_pan < 0) 실제 좌표의 y를 뒤집어 준다 — 그래야 도면 FK로 그린 팔과 장면이 서로 맞는다. `mirror`가 그걸 알린다.
    """
    sq = spec["square_mm"] * scale
    W, H = spec["nx"] * sq, spec["ny"] * sq
    Rb, tb = rodrigues(P["board_r"]), np.asarray(P["board_t"], float)
    board = target_base(P, "board", [[0, 0, 0], [W, 0, 0], [W, H, 0], [0, H, 0]])
    n = Rb[:, 2]
    foot = -(n @ (np.zeros(3) - tb)) * n                        # pan 축 발치(원점)를 책상 평면에 내린 점
    c = (board.mean(axis=0) + foot) / 2
    ex, ey = Rb[:, 0], Rb[:, 1]
    up = n if n @ (np.array([P["d0"], 0.0, P["z0"]]) - tb) > 0 else -n
    table = np.array([c + a * 380 * ex + b * 380 * ey - 1.0 * up for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    quads = [{"name": "table", "pts": table, "color": 0x6b7785, "opacity": 0.35},
             {"name": "board", "pts": board + 0.5 * up, "color": 0xf4f4f4, "opacity": 0.95}]
    if P.get("wall_r") is not None and dots_wh:
        w, h = dots_wh
        quads.append({"name": "wall", "color": 0xfff3c4, "opacity": 0.95,
                      "pts": target_base(P, "wall", [[-25, -25, 0], [w + 25, -25, 0], [w + 25, h + 25, 0], [-25, h + 25, 0]])})
    mir = P["sgn_pan"] < 0
    for q in quads:
        q["pts"] = (q["pts"] * ([1.0, -1.0, 1.0] if mir else [1.0, 1.0, 1.0])).round(1).tolist()
    return {"mirror": bool(mir), "quads": quads, "table_n": up.round(4).tolist(),
            "tilt_deg": round(float(np.degrees(np.arccos(min(1.0, abs(up[2]))))), 1)}


def _selftest() -> None:
    from tomato_picker.hardware.handeye import tool_frame_joints
    rng = np.random.default_rng(0)
    for _ in range(50):
        deg = dict(zip(J, rng.uniform((-90, -10, -10, -90, -150), (90, 170, 170, 90, 150))))
        for rs in (1.0, -1.0):
            P = dict(DEFAULT, sgn_roll=rs)
            c = chain(P, [deg[j] for j in J])
            T = tool_frame_joints(deg, _G, rs)
            assert np.allclose(c["R_bt"][0], T.R, atol=1e-9) and np.allclose(c["tcp"][0], T.t, atol=1e-9)
    R = rodrigues([0.3, -1.2, 2.0])
    assert np.allclose(rodrigues(rotvec(R)), R, atol=1e-9)
    assert np.allclose(rodrigues(rotvec(rodrigues([np.pi, 0, 0]))), rodrigues([np.pi, 0, 0]), atol=1e-6)
    try:                                                        # 모서리 번호 규약이 OpenCV와 같은가(cv2가 있으면)
        import cv2
        spec = {"nx": 5, "ny": 7, "square_mm": 35.0, "marker_mm": 26.0}
        b = cv2.aruco.CharucoBoard((5, 7), 35.0 * 0.945, 26.0 * 0.945,
                                   cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        assert np.allclose(b.getChessboardCorners(), board_points(spec, 0.945), atol=1e-4)
    except ImportError:
        pass
    # 거울: FK(−pan, −roll) = S·FK(pan, roll)·S  (S = y 뒤집기)
    q = np.array([[20.0, 50, 60, -30, 40]])
    a, b = chain(DEFAULT, q), chain(dict(DEFAULT, sgn_pan=-1.0, sgn_roll=-1.0), q)
    S = np.diag([1.0, -1.0, 1.0])
    assert np.allclose(b["R_bt"][0], S @ a["R_bt"][0] @ S, atol=1e-9) and np.allclose(b["tcp"][0], S @ a["tcp"][0])
    uv, zc = project(DEFAULT, {"fx": 400.0, "fy": 400.0, "ppx": 320.0, "ppy": 240.0},
                     np.eye(3)[None], np.zeros((1, 3)), [[10.0, -20.0, 100.0]])
    assert np.allclose(uv, [[360.0, 160.0]]) and zc[0] == 100.0
    sc = scene(dict(DEFAULT, sgn_pan=-1.0, board_t=[200.0, 50.0, -10.0]), {"nx": 5, "ny": 7, "square_mm": 35.0}, 1.0)
    assert sc["mirror"] and sc["quads"][1]["pts"][0] == [200.0, -50.0, -9.5] and sc["tilt_deg"] == 0.0, sc
    print("kincal_model OK")


if __name__ == "__main__":
    _selftest()
