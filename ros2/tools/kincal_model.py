#!/usr/bin/env python3
"""**팔(SO-101) + 손목 카메라 관측 모형** — 관절 읽음값 → "고정 표적의 점이 화면 어디에 찍히나". numpy뿐.

    python ros2/tools/kincal_model.py        # 자체검증

수집기(kincal_collect: 계획·안전), 풀이(kincal_solve: 맞춤), 트윈이 **같은 식**을 쓴다.

뼈대 = **공식 SO-101 URDF**(TheRobotStudio/SO-ARM100 `Simulation/SO101/so101_new_calib.urdf`)의 관절 사슬 그대로.
2026-11-12 처음엔 `kinematics.forward`의 평면 근사(z0·d0·l1·l2·l3)로 풀었는데 d0가 −31.5 → +31mm로 뒤집히고
책상이 13° 기운 것으로 나왔다(사진으로는 밑동이 평평하다). 평면 근사에 없는 것 — 상완이 28mm 꺾여 있고
(팔꿈치 축이 링크 선에서 14° 벗어남), 어깨 축이 pan 축 옆으로 18mm 비켜 있고, 손목 굴림 축이 61·18mm 어긋나 있다 —
을 풀이가 영점·링크·책상 기울기로 메우고 있었다. CAD 치수는 믿고(안 푼다), 실물이 CAD와 다른 것만 푼다:

  sgn_*             관절마다 서보가 느는 방향 ↔ URDF 축 방향(±1) — 풀이가 조합을 다 본다
  off_lift·elbow·wrist  관절 영점(도): URDF의 0 ↔ 서보 눈금 2048. pan은 표적 방위가, roll은 카메라 자세가 흡수한다
  geo_lift·elbow·wrist·roll  **이 팔이 CAD와 다른 만큼** — 관절 틀을 자기 틀에서 조금 옮기고(dx,dy[,dz] mm) 축을 조금
                    기울인다(rx,ry rad). 좁은 자세만 보면 영점이 다 메워 주지만 넓게 움직이면 드러난다(11/12: 좁은 쪽 1.7px일 때
                    넓은 쪽 5~35px). 자기 축 둘레 회전은 영점과 같고, 나란한 세 축(lift·elbow·wrist)의 축 방향 이동은 하나만
                    보이고, roll의 축 방향 이동은 카메라 자리가 흡수하므로 뺐다 → 5+4+4+4 = 17개
  play_*            **넘어가는 유격** — 팔 무게중심이 어깨 축 뒤로 넘어가면(어깨 중력 토크가 play_tau0 아래로) 팔 전체가
                    어깨 축과 나란한 축 둘레로 play_deg만큼 덜컥 넘어간다(11/12 실측: 토크 0 근처에서 −7~−8° 계단, 엔코더엔 안 보임).
                    회전 중심(어깨 축에서 play_pr·play_pz mm)도 푼다 — 어깨 축이면 관절 체결 유격, 발치면 밑동이 까딱이는 것
  sag_*             엔코더가 못 보는 처짐(도 / 중력토크 kg·mm) — 링크·기어가 휘는 몫
  bl_*              백래시(도) × 마지막 접근 방향(±1)
  cam_r·cam_t       T_gripper_link→카메라 (손-눈)
  board_yaw·board_t·board_tilt  책상 ChArUco: **책상은 밑동과 한 평면**이라 방위 1 + 자리 3, 기울기는 ±1.5° 사전값 안
  wall_r·wall_t     벽 4점 종이(자유 6)
  f_scale·k1·roll_scale  카메라 초점 배율·방사 왜곡·roll 눈금(공장값 검사용)

관절 입력 q = **서보 눈금 도**((틱 − 2048)·360/4096). `Calib.to_deg` 도에서 오는 길은 `tick_map`.
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

# so101_new_calib.urdf의 관절 origin (xyz mm, rpy rad) — 부모 링크 틀에서 본 관절 틀. 축은 전부 그 틀의 z.
URDF = (
    ("shoulder_pan", (38.8353, -8.97657e-06, 62.4), (3.14159, 4.18253e-17, -3.14159)),
    ("shoulder_lift", (-30.3992, -18.2778, -54.2), (-1.5708, -1.5708, 0.0)),
    ("elbow_flex", (-112.57, -28.0, 0.0), (0.0, 0.0, 1.5708)),
    ("wrist_flex", (-134.9, 5.2, 0.0), (0.0, 0.0, -1.5708)),
    ("wrist_roll", (0.0, -61.1, 18.1), (1.5708, 0.0486795, 3.14159)),
)
URDF_TCP = ((-7.9, -0.218121, -98.1274), (0.0, 3.14159, 0.0))     # gripper_link → gripper_frame_link(무는 자리, z = 찌르는 방향)
# **밑동이 넘어가는 자세** — 팔 밑동이 책상에 고정돼 있지 않으면, 팔 무게중심이 어깨 축 뒤로 넘어갈 때(어깨 중력 토크가
# 0 근처 아래로) 밑동이 발판 모서리를 축으로 까딱 넘어간다. 2026-11-12 실측: 토크 ≲ 5 kg·mm에서 팔 전체가 7~11° 돌았고
# (회전 중심 = 어깨 축 77mm 아래·78mm 옆 = 발판 모서리), 10~17에서도 가끔 1° 안팎 흔들렸다. 엔코더엔 안 보인다.
# 수집기는 이 아래로 안 가고(가는 길에도), 풀이는 이 아래 자세를 뺀다. 밑동을 고정하면 0으로 내려도 된다.
ROCK_TAU = 18.0
TIP_MM = 15.0                                                     # 무는 자리에서 손끝까지(안전 여유용 어림)

DEFAULT = {
    "sgn_pan": 1.0, "sgn_lift": 1.0, "sgn_elbow": 1.0, "sgn_wrist": 1.0, "sgn_roll": 1.0, "roll_scale": 1.0,
    "off_lift": 0.0, "off_elbow": 0.0, "off_wrist": 0.0,
    "geo_lift": [0.0] * 5, "geo_elbow": [0.0] * 4, "geo_wrist": [0.0] * 4, "geo_roll": [0.0] * 4,
    "play_deg": 0.0, "play_tau0": 0.0, "play_w": 4.0, "play_pr": 0.0, "play_pz": 0.0,
    "sag_lift": 0.0, "sag_elbow": 0.0, "sag_wrist": 0.0,
    "bl_pan": 0.0, "bl_lift": 0.0, "bl_elbow": 0.0, "bl_wrist": 0.0, "bl_roll": 0.0,
    "cam_r": [0.0, 0.0, 0.0], "cam_t": [0.0, 0.0, 0.0],
    "board_yaw": 0.0, "board_tilt": [0.0, 0.0], "board_t": [0.0, 0.0, 0.0], "board_flip": -1.0,
    "wall_r": None, "wall_t": None,
    "f_scale": 1.0, "k1": 0.0,
}
_SGN = ("sgn_pan", "sgn_lift", "sgn_elbow", "sgn_wrist", "sgn_roll")


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


def _fixed(xyz, rpy) -> np.ndarray:
    """URDF origin → 4×4. rpy는 고정축 XYZ(= Rz·Ry·Rx)."""
    r, p, y = rpy
    T = np.eye(4)
    T[:3, :3] = rodrigues([0, 0, y]) @ rodrigues([0, p, 0]) @ rodrigues([r, 0, 0])
    T[:3, 3] = xyz
    return T


_F = [_fixed(xyz, rpy) for _, xyz, rpy in URDF]
_F_TCP = _fixed(*URDF_TCP)
PAN_XY = _F[0][:2, 3].copy()                   # pan 축이 base_link 원점에서 비켜 있다(x 38.8mm)


def tick_map(cal: dict, cart: dict):
    """`Calib.to_deg` 도 → 서보 눈금 도.  q = A·deg + B  (관절 5개).

    ⚠ **팔 드라이버가 주는 값은 이미 도(度)다** — lerobot `SOFollowerRobotConfig.use_degrees` 기본값이 True라
      읽음값 = (틱 − 범위 가운데)·360/4095 이다(2026-11-12 젯슨 설치본에서 확인). 그런데 `arm_calib.Calib.to_deg`는
      그 값을 −100..100 정규값으로 보고 `per = 범위폭/200`을 한 번 더 곱한다 → to_deg의 도는 실제보다
      pan ×1.217 · lift ×1.029 · elbow ×1.033 · wrist ×1.039 크다(roll만 0.999로 실측해 덮어써 뒀다).
      평면 모형이 링크를 7% 짧게, 책상을 13° 기울게 푼 것이 이 눈금 오차를 메우던 것이다.
      그래서 여기서는 to_deg를 **되돌려**(읽음값 = zero + sign·(deg − ref)/per) 눈금에서 바로 도를 만든다."""
    A, B = [], []
    over = cart.get("deg_per_norm") or {}
    for j in J:
        v = (cart.get("signs") or {}).get(j)
        sg = -1.0 if (v is not None and float(v) < 0) else 1.0
        ref, zero = float(cart["ref_deg"].get(j, 0.0)), float(cart["zero"].get(j, 0.0))
        mn, mx = float(cal[j]["range_min"]), float(cal[j]["range_max"])
        per = abs(float(over[j])) if over.get(j) else (mx - mn) * 360.0 / 4096.0 / 200.0
        k = 4095.0 / 4096.0                                     # lerobot은 360/4095로 나눈다
        A.append(sg / per * k)
        B.append((zero - sg * ref / per) * k + ((mn + mx) / 2.0 - 2048.0) * 360.0 / 4096.0)
    return np.array(A), np.array(B)


def board_points(spec: dict, scale: float) -> np.ndarray:
    """ChArUco 안쪽 모서리 id → 보드 좌표(mm). OpenCV 규약: id = 행×(nx−1)+열, 자리 = ((열+1)·칸, (행+1)·칸, 0)."""
    sq = spec["square_mm"] * scale
    return np.array([[(c + 1) * sq, (r + 1) * sq, 0.0]
                     for r in range(spec["ny"] - 1) for c in range(spec["nx"] - 1)])


def board_R(P: dict) -> np.ndarray:
    """보드 틀 → base. 책상 위에 누운 종이: z축이 base ±z(board_flip), 방위 yaw, 기울기는 작은 값."""
    tx, ty = P["board_tilt"]
    return rodrigues([0, 0, P["board_yaw"]]) @ rodrigues([tx, ty, 0.0]) @ np.diag([1.0, P["board_flip"], P["board_flip"]])


def _rz(th: np.ndarray) -> np.ndarray:
    T = np.zeros((len(th), 4, 4))
    c, s = np.cos(th), np.sin(th)
    T[:, 0, 0], T[:, 0, 1], T[:, 1, 0], T[:, 1, 1], T[:, 2, 2], T[:, 3, 3] = c, -s, s, c, 1.0, 1.0
    return T


def frames(P: dict) -> list:
    """이 팔의 관절 틀 5개(부모 링크 → 관절, 4×4) = URDF 값 · 풀어낸 어긋남(geo_*). 없으면 URDF 그대로."""
    F = [_F[0]]
    for i, k in ((1, "geo_lift"), (2, "geo_elbow"), (3, "geo_wrist"), (4, "geo_roll")):
        g = P.get(k)
        if g is None or not np.any(g):
            F.append(_F[i])
            continue
        dx, dy = g[0], g[1]
        dz, rx, ry = (g[2], g[3], g[4]) if len(g) == 5 else (0.0, g[2], g[3])
        D = np.eye(4)
        D[:3, :3] = rodrigues([rx, ry, 0.0])
        D[:3, 3] = [dx, dy, dz]
        F.append(_F[i] @ D)
    return F


def _fk(th: np.ndarray, F=_F):
    """관절각(rad, N×5) → (관절 틀 5개(회전 전, N×4×4), gripper_link 틀)."""
    T = np.broadcast_to(np.eye(4), (len(th), 4, 4))
    O = []
    for i in range(5):
        T = T @ F[i]
        O.append(T)
        T = T @ _rz(th[:, i])
    return O, T


def _gravity(O, Tg) -> list:
    """관절 틀들 → 중력 토크 [어깨, 팔꿈치, 손목] (kg·mm, 각 관절 축 +방향 둘레, 그 관절보다 바깥 질량만)."""
    p = [o[:, :3, 3] for o in O]
    tcp = (Tg @ _F_TCP)[:, :3, 3]
    hand = tcp - p[3]
    hand = p[3] + HAND_D * hand / np.linalg.norm(hand, axis=1, keepdims=True)
    mass = ((M1, (p[1] + p[2]) / 2), (M_ELBOW, p[2]), (M2, (p[2] + p[3]) / 2), (M_HAND, hand))
    down = np.array([0.0, 0.0, -1.0])
    return [sum(m * np.einsum("nj,nj->n", np.cross(x - p[i], down), O[i][:, :3, 2]) for m, x in outer)
            for i, outer in ((1, mass), (2, mass[2:]), (3, mass[3:]))]


def lift_torque(P: dict, q) -> np.ndarray:
    """자세마다 어깨 축 둘레 중력 토크(kg·mm) — `ROCK_TAU` 판정용. 양수 = 팔이 앞으로 쏠림."""
    q = np.atleast_2d(np.asarray(q, float))
    deg = np.array([P[k] for k in _SGN]) * q + [0.0, P["off_lift"], P["off_elbow"], P["off_wrist"], 0.0]
    return _gravity(*_fk(np.radians(deg), frames(P)))[0]


def chain(P: dict, q, dirs=None) -> dict:
    """서보 눈금 도(N×5) → 팔 위 점들과 카메라 자세(전부 base_link 좌표, mm)."""
    q = np.atleast_2d(np.asarray(q, float))
    d = np.zeros_like(q) if dirs is None else np.atleast_2d(np.asarray(dirs, float))
    sg = np.array([P[k] for k in _SGN])
    off = np.array([0.0, P["off_lift"], P["off_elbow"], P["off_wrist"], 0.0])
    bl = np.array([P["bl_pan"], P["bl_lift"], P["bl_elbow"], P["bl_wrist"], P["bl_roll"]])
    scale = np.array([1.0, 1.0, 1.0, 1.0, P["roll_scale"]])
    deg = sg * q * scale + off + bl * d
    sag = np.array([P["sag_lift"], P["sag_elbow"], P["sag_wrist"]])
    F = frames(P)
    play = float(P.get("play_deg") or 0.0)
    tau = None
    if np.any(sag != 0.0) or play:                              # 중력 토크는 처짐 없는 자세에서 한 번 잰다
        tau = _gravity(*_fk(np.radians(deg), F))
        for k, i in enumerate((1, 2, 3)):
            deg[:, i] = deg[:, i] + sag[k] * tau[k]
    O, Tg = _fk(np.radians(deg), F)
    Tt = Tg @ _F_TCP
    R_g, p_g = Tg[:, :3, :3], Tg[:, :3, 3]
    tcp = Tt[:, :3, 3]
    R_gc, t_gc = rodrigues(P["cam_r"]), np.asarray(P["cam_t"], float)
    out = {"shoulder": O[1][:, :3, 3], "elbow": O[2][:, :3, 3], "wrist": O[3][:, :3, 3], "roll": O[4][:, :3, 3],
           "tcp": tcp, "tip": tcp + TIP_MM * Tt[:, :3, 2], "R_bt": R_g, "p_g": p_g,
           "R_bc": R_g @ R_gc, "t_bc": p_g + R_g @ t_gc}
    if play:                                                    # 넘어가는 유격: 어깨 축과 나란한 축 둘레로 팔 전체를 돌린다
        s_ = 0.5 * (1.0 - np.tanh((tau[0] - P["play_tau0"]) / max(0.5, abs(P["play_w"]))))
        ang = np.radians(play) * s_
        a = O[1][:, :3, 2]
        er = np.cross(a, [0.0, 0.0, 1.0])
        er = er / np.linalg.norm(er, axis=1, keepdims=True)     # 팔 평면 안의 수평 방향
        c = out["shoulder"] + P["play_pr"] * er + P["play_pz"] * np.array([0.0, 0.0, 1.0])
        K = np.zeros((len(q), 3, 3))
        K[:, 0, 1], K[:, 0, 2], K[:, 1, 0], K[:, 1, 2], K[:, 2, 0], K[:, 2, 1] = -a[:, 2], a[:, 1], a[:, 2], -a[:, 0], -a[:, 1], a[:, 0]
        Rp = np.eye(3) + np.sin(ang)[:, None, None] * K + (1 - np.cos(ang))[:, None, None] * (K @ K)
        for k in ("shoulder", "elbow", "wrist", "roll", "tcp", "tip", "p_g", "t_bc"):
            out[k] = c + np.einsum("nij,nj->ni", Rp, out[k] - c)
        for k in ("R_bt", "R_bc"):
            out[k] = Rp @ out[k]
        out["play"] = s_
    return out


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
    R = board_R(P) if which == "board" else rodrigues(P[which + "_r"])
    return np.asarray(pts_local, float) @ R.T + np.asarray(P[which + "_t"], float)


def scene(P: dict, spec: dict, scale: float, dots_wh=None) -> dict:
    """트윈 3D 미리보기에 그릴 장면(base_link 좌표 mm) — 책상 평면·보드 종이·(알면) 벽 종이의 사각형 꼭짓점."""
    sq = spec["square_mm"] * scale
    W, H = spec["nx"] * sq, spec["ny"] * sq
    Rb, tb = board_R(P), np.asarray(P["board_t"], float)
    board = target_base(P, "board", [[0, 0, 0], [W, 0, 0], [W, H, 0], [0, H, 0]])
    up = Rb[:, 2] if Rb[2, 2] > 0 else -Rb[:, 2]
    foot = np.array([PAN_XY[0], PAN_XY[1], 0.0])
    foot = foot - ((foot - tb) @ up) * up                        # pan 축 발치를 책상 평면에 내린 점
    c = (board.mean(axis=0) + foot) / 2
    table = np.array([c + a * 380 * Rb[:, 0] + b * 380 * Rb[:, 1] - 1.0 * up for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    quads = [{"name": "table", "pts": table, "color": 0x6b7785, "opacity": 0.35},
             {"name": "board", "pts": board + 0.5 * up, "color": 0xf4f4f4, "opacity": 0.95}]
    if P.get("wall_r") is not None and dots_wh:
        w, h = dots_wh
        quads.append({"name": "wall", "color": 0xfff3c4, "opacity": 0.95,
                      "pts": target_base(P, "wall", [[-25, -25, 0], [w + 25, -25, 0], [w + 25, h + 25, 0], [-25, h + 25, 0]])})
    for qd in quads:
        qd["pts"] = np.asarray(qd["pts"]).round(1).tolist()
    return {"quads": quads, "table_z": round(float(foot[2]), 1),
            "tilt_deg": round(float(np.degrees(np.arccos(min(1.0, abs(up[2]))))), 2)}


def _selftest() -> None:
    R = rodrigues([0.3, -1.2, 2.0])
    assert np.allclose(rodrigues(rotvec(R)), R, atol=1e-9)
    assert np.allclose(rodrigues(rotvec(rodrigues([np.pi, 0, 0]))), rodrigues([np.pi, 0, 0]), atol=1e-6)
    try:                                                        # 모서리 번호 규약이 OpenCV와 같은가(cv2가 있으면)
        import cv2
        b = cv2.aruco.CharucoBoard((5, 7), 35.0 * 0.945, 26.0 * 0.945,
                                   cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        assert np.allclose(b.getChessboardCorners(), board_points({"nx": 5, "ny": 7, "square_mm": 35.0}, 0.945), atol=1e-4)
    except ImportError:
        pass
    # URDF 사슬: 0 자세에서 어깨 축 높이 = 62.4+54.2, 상완·전완 길이 = CAD 값, pan은 PAN_XY 둘레의 수직축 회전
    c0 = chain(DEFAULT, np.zeros((1, 5)))
    assert abs(c0["shoulder"][0, 2] - 116.6) < 1e-3, c0["shoulder"]
    assert abs(np.linalg.norm(c0["elbow"][0] - c0["shoulder"][0]) - np.hypot(112.57, 28.0)) < 1e-6
    assert abs(np.linalg.norm(c0["wrist"][0] - c0["elbow"][0]) - np.hypot(134.9, 5.2)) < 1e-6
    q = np.array([[25.0, 40.0, -30.0, 20.0, 50.0]])
    a, b = chain(DEFAULT, q), chain(DEFAULT, q + [[30.0, 0, 0, 0, 0]])
    for k in ("elbow", "wrist", "tcp"):
        ra, rb = a[k][0, :2] - PAN_XY, b[k][0, :2] - PAN_XY
        assert abs(np.linalg.norm(ra) - np.linalg.norm(rb)) < 1e-2 and abs(a[k][0, 2] - b[k][0, 2]) < 1e-2, k   # URDF의 π가 3.14159로 잘려 있다
        ang = np.degrees(np.arctan2(ra[0] * rb[1] - ra[1] * rb[0], ra @ rb))
        assert abs(abs(ang) - 30.0) < 1e-3, ang                 # (URDF pan 축은 아래를 본다 — 부호는 풀이가 정한다)
    assert np.allclose(a["R_bt"][0] @ a["R_bt"][0].T, np.eye(3), atol=1e-9)
    # geo: 팔꿈치 틀을 자기 x로 5mm 옮기면 그 바깥(손목·무는 자리)만 정확히 5mm 간다. 축 기울기는 길이를 안 바꾼다.
    g = chain(dict(DEFAULT, geo_elbow=[5.0, 0.0, 0.0, 0.0]), q)
    assert np.allclose(g["shoulder"], a["shoulder"]) and abs(np.linalg.norm(g["elbow"][0] - a["elbow"][0]) - 5.0) < 1e-9
    assert abs(np.linalg.norm(g["tcp"][0] - a["tcp"][0]) - 5.0) < 1e-9
    g = chain(dict(DEFAULT, geo_wrist=[0.0, 0.0, 0.03, -0.02]), q)
    assert np.allclose(g["wrist"], a["wrist"]) and not np.allclose(g["tcp"], a["tcp"], atol=0.5)
    assert abs(np.linalg.norm(g["roll"][0] - g["wrist"][0]) - np.linalg.norm(a["roll"][0] - a["wrist"][0])) < 1e-9
    # lift·elbow·wrist는 한 평면 안에서 돈다(축이 서로 나란하다) — roll만 그 평면 밖
    O, _ = _fk(np.radians(q))
    assert all(abs(abs(O[1][0, :3, 2] @ O[i][0, :3, 2]) - 1.0) < 1e-4 for i in (2, 3))
    # 처짐: 수평으로 뻗으면 중력 쪽으로(끝이 내려간다), 양의 compliance에서
    flat = min((chain(DEFAULT, [[0.0, l, e, 0.0, 0.0]])["tcp"][0, 2], l, e) for l in range(-90, 91, 10) for e in range(-90, 91, 10))
    hi = max((np.hypot(*(chain(DEFAULT, [[0.0, l, e, 0.0, 0.0]])["tcp"][0, :2] - PAN_XY)), l, e)
             for l in range(-90, 91, 10) for e in range(-90, 91, 10))
    qs = [[0.0, hi[1], hi[2], 0.0, 0.0]]
    z0 = chain(DEFAULT, qs)["tcp"][0, 2]
    z1 = chain(dict(DEFAULT, sag_lift=0.03, sag_elbow=0.03, sag_wrist=0.03), qs)["tcp"][0, 2]
    assert z1 < z0 - 1.0, (z0, z1, flat)
    # 넘어가는 유격: 문턱보다 토크가 한참 크면 아무 일도 없고, 한참 작으면 어깨 축 둘레로 play_deg만큼 — 어깨 영점을 그만큼 옮긴 것과 같다
    pl = dict(DEFAULT, play_deg=-7.0, play_tau0=1e6, play_w=2.0)
    assert np.allclose(chain(pl, q)["tcp"], chain(dict(DEFAULT, off_lift=-7.0), q)["tcp"], atol=1e-6)
    assert np.allclose(chain(dict(pl, play_tau0=-1e6), q)["tcp"], a["tcp"], atol=1e-6)
    far = chain(dict(pl, play_pz=-100.0), q)                    # 회전 중심이 발치 쪽이면 어깨 축도 같이 움직인다
    assert np.linalg.norm(far["shoulder"][0] - a["shoulder"][0]) > 5.0
    # tick_map: 젯슨 실값(2026-11-12)으로 왕복 — 정규값 0 ↔ 범위 가운데 틱
    cal = {"shoulder_pan": {"range_min": 560, "range_max": 3330}, "shoulder_lift": {"range_min": 1009, "range_max": 3350},
           "elbow_flex": {"range_min": 808, "range_max": 3159}, "wrist_flex": {"range_min": 735, "range_max": 3099},
           "wrist_roll": {"range_min": 0, "range_max": 4095}}
    cart = {"zero": {"shoulder_pan": -11.516, "shoulder_lift": -18.242, "elbow_flex": -60.352, "wrist_flex": -1.758, "wrist_roll": 0.0},
            "ref_deg": {"shoulder_lift": 90.0}, "signs": {}, "deg_per_norm": {"wrist_roll": 0.999}}
    A, B = tick_map(cal, cart)
    per = (3350 - 1009) * 360.0 / 4096.0 / 200.0
    read = 30.0                                                 # 드라이버가 준 값(도) = (틱 − 가운데)·360/4095
    tick = (1009 + 3350) / 2 + read * 4095 / 360
    to_deg = 90.0 + (read - (-18.242)) * per                    # Calib.to_deg가 그 값을 바꿔 놓은 도
    assert abs(A[1] * to_deg + B[1] - (tick - 2048) * 360 / 4096) < 1e-9
    assert abs(A[4] * (50.0 * 0.999) + B[4] - (2047.5 + 50.0 * 4095 / 360 - 2048) * 360 / 4096) < 1e-9
    uv, zc = project(DEFAULT, {"fx": 400.0, "fy": 400.0, "ppx": 320.0, "ppy": 240.0},
                     np.eye(3)[None], np.zeros((1, 3)), [[10.0, -20.0, 100.0]])
    assert np.allclose(uv, [[360.0, 160.0]]) and zc[0] == 100.0
    sc = scene(dict(DEFAULT, board_t=[200.0, 50.0, -10.0]), {"nx": 5, "ny": 7, "square_mm": 35.0}, 1.0)
    assert sc["tilt_deg"] == 0.0 and sc["table_z"] == -10.0 and sc["quads"][1]["pts"][0] == [200.0, 50.0, -9.5], sc
    print("kincal_model OK")


if __name__ == "__main__":
    _selftest()
