#!/usr/bin/env python3
"""**집 안 공간 지도** — 차체 앞 Astra 깊이로 3D 점구름 + 2D 점유격자를 쌓고,
지도 바닥을 찍으면 거기까지 길을 짜서 톡톡 간다.

    ~/lerobot/.venv/bin/python ros2/tools/space_map.py snap          # 안 움직이고 한 장
    ~/lerobot/.venv/bin/python ros2/tools/space_map.py scan          # 제자리 한 바퀴
    ~/lerobot/.venv/bin/python ros2/tools/space_map.py go --x 1.5 --y 0.3
    ~/lerobot/.venv/bin/python ros2/tools/space_map.py clear
    python ros2/tools/space_map.py selftest                          # PC, 장치 없이

조작대(click_server, job=space)와 이 명령줄이 같은 경로다. 왜·한계 = docs/space-map.md.

⚠ **자세는 바퀴가 아니라 지도와 맞춰서 안다** — 이 차체엔 엔코더가 없고, 넛지 한 번이
  얼마나 가는지는 정지마찰이 정한다(CLAUDE.md 1번 병). 깊이 한 장을 바닥 위 띠만
  잘라 2D 스캔으로 접고, 지도 우도장에 가장 잘 맞는 (x,y,θ)를 전수 탐색한다
  (상관 스캔매칭, Olson 2009 / Hector SLAM 계열).
⚠ **바닥은 안 보인다** — 마루가 IR을 비껴 반사해 깊이가 0이다(2026-10-22 실측, 아래
  절반이 통째로 비었다). 그래서 빈 칸은 바닥이 아니라 **장애물까지 쏜 광선**으로만 칠한다.
⚠ **Astra는 60cm 안을 못 본다** — 가까운 장애물은 멀리서 봐 둔 지도로만 피한다.
⚠ 주행 포트는 한 프로세스 — tomato-voice·base_nudge와 동시에 못 돈다.
⚠ **회전은 IMU가 먼저 말한다**(2026-10-22~, tools/imu_yaw.py → /dev/shm/imu.json). 마지막 자세 이후
  자이로가 잰 회전을 θ 짐작으로 쓰고 맞춤은 그 둘레 ±IMU_WIN_DEG만 찾는다 — 곧은 벽의 모호함이
  사라지고, 맞춤이 거절돼도 θ는 맞는다. IMU가 없거나 다시 켜졌으면(boot가 다르면) 예전처럼 지도만 믿는다.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))

RES = 0.05                     # 칸 5cm
N = 300                        # 15m × 15m, 원점 = 가운데
OCC, FREE = 0.6, 0.6           # 로그오즈 문턱
Q_MIN = 0.6                    # 일치율 = 맞은 점 / (맞은 점 + 빈 곳에 떨어진 점) — 모르는 곳은 안 센다
HIT_MIN = 20                   # 지도와 겹친 점이 이보다 적으면 자세를 못 믿는다(화면 거의 전부가 처음 보는 곳)
MAP = os.path.expanduser("~/space_map.npz")
MOUNT_FILE = os.path.expanduser("~/space_mount.json")
IMU_FILE = "/dev/shm/imu.json"
IMU_WIN_DEG = 5.0              # IMU가 있으면 θ는 IMU 짐작 ±이만큼만 찾는다(넛지 한 번의 자이로 오차 ≪ 1°)
# 장착값 — 케이스 CAD(tomato_case.scad astra_*) 기준 추정. **실측해서 MOUNT_FILE로 덮어라.**
# x=회전 중심→렌즈(앞+), h=바닥→렌즈, tilt=위로+, floor_cut/top=장애물로 칠 높이 띠, radius=차체 반지름
# y=렌즈가 회전 중심에서 왼쪽(+)으로, yaw_deg=카메라가 정면에서 왼쪽(+)으로 돈 각 — 제자리 회전의 **실제 회전 중심**
#   기준이다(메카넘은 기하 중심이 아니라 미끄러지며 돈다). 2026-10-22 실기 두 바퀴 자기 보정: x 0.345·y −0.10(yaw는 안 보여 0)
#   (이웃 장 겹침 10.4→5.7cm, 한 바퀴로 맞춰 다른 바퀴에 써도 10.5→5.6·9.9→7.9cm) — 젯슨 ~/space_mount.json에 있다.
MOUNT = {"x": 0.12, "y": 0.0, "yaw_deg": 0.0, "h": 0.15, "tilt_deg": 0.0, "floor_cut": 0.05, "top": 0.9, "radius": 0.22,
         # 손목 D405용: 회전판 축이 차체 원점에서 얼마나 앞(arm_x)·바닥에서 얼마나 위(arm_z)인가.
         # so101_geometry.yaml mount(60·76.5mm)와 같은 값 — ⚠ 케이스 장착 전 값이라 실측해서 덮어라.
         "arm_x": 0.06, "arm_z": 0.0765,
         # 토마토 검출 카메라(tomato-vision): 가로 화각과 정면 대비 방향(왼쪽 +). ⚠ 실측 전 추정값 —
         # 정면에 토마토를 두고 [토마토 찾기]의 cx가 화면 가운데인지, 옆에 두고 각이 맞는지로 맞춘다.
         "tcam_hfov": 66.0, "tcam_yaw_deg": 0.0}


def mount() -> dict:
    m = dict(MOUNT)
    try:
        m.update(json.load(open(MOUNT_FILE)))
    except (OSError, ValueError):
        pass
    return m


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def idx(xy) -> np.ndarray:
    return np.floor(np.asarray(xy, float) / RES).astype(int) + N // 2


# ── 깊이 → 점 ──────────────────────────────────────────────────────────
def cloud(depth: np.ndarray, meta: dict, m: dict, stride: int = 3) -> np.ndarray:
    """깊이 → 차체 좌표 점 (x 앞, y 왼, z 위, m). 단위·유효거리는 **meta가 말한다**."""
    it = meta["intrinsics"]
    d = depth[::stride, ::stride].astype(np.float32) * (float(meta["depth_scale_mm"]) / 1000.0)
    v, u = np.nonzero((d >= meta["min_mm"] / 1000.0) & (d <= meta["max_mm"] / 1000.0))
    Z = d[v, u]
    uu = u * stride
    if meta.get("mirrored"):              # Astra 깊이는 거울상이다(astra_cam meta) — 화면 왼쪽 = 실제 오른쪽
        uu = int(meta.get("width", depth.shape[1])) - 1 - uu
    X = (uu - it["ppx"]) * Z / it["fx"]
    Y = (v * stride - it["ppy"]) * Z / it["fy"]
    t = math.radians(m["tilt_deg"])
    c, s = math.cos(t), math.sin(t)
    fwd, left = Z * c + Y * s, -X
    cy, sy = math.cos(math.radians(m["yaw_deg"])), math.sin(math.radians(m["yaw_deg"]))
    return np.stack([m["x"] + cy * fwd - sy * left, m["y"] + sy * fwd + cy * left, m["h"] + Z * s - Y * c], 1)


def scan2d(pts: np.ndarray, m: dict) -> np.ndarray:
    """높이 띠 안의 점을 0.5° 부채살마다 **가장 가까운 것 하나**로 접는다 → (K,2) 차체 좌표."""
    p = pts[(pts[:, 2] > m["floor_cut"]) & (pts[:, 2] < m["top"])]
    if len(p) == 0:
        return np.zeros((0, 2))
    dx, dy = p[:, 0] - m["x"], p[:, 1] - m["y"]
    a = np.round(np.degrees(np.arctan2(dy, dx)) * 2).astype(int)
    o = np.lexsort((np.hypot(dx, dy), a))
    _, first = np.unique(a[o], return_index=True)
    return p[o[first], :2]


# ── 지도 ───────────────────────────────────────────────────────────────
def empty() -> dict:
    return {"lo": np.zeros((N, N), np.float32), "vox": np.zeros((0, 3), np.int16),
            "vn": np.zeros(0, np.uint16),   # 복셀마다 본 장 수 — 한 번만 본 칸은 대개 깊이 잡음이다
            "pose": np.zeros(3), "q": 0.0, "frames": 0,
            "path": np.zeros((0, 2)), "goal": np.zeros(0),
            "imu": np.full(2, np.nan),
            "tomato": np.zeros((0, 4))}   # 찾은 토마토 (x, y, 본 횟수, 거리 아는가 1/0)  # 지금 자세를 정할 때의 IMU (boot, yaw rad) — 다음 회전 짐작의 기준


def load() -> dict:
    s = empty()
    try:
        with np.load(MAP) as z:
            for k in s:
                if k in z.files:          # 옛 지도엔 새 키가 없다 — 없다고 지도 전체를 버리지 않는다
                    s[k] = z[k] if z[k].ndim else z[k].item()
    except (OSError, KeyError, ValueError):
        pass
    s["pose"] = np.asarray(s["pose"], float)
    if len(s["vn"]) != len(s["vox"]):         # 횟수가 없던 옛 지도 — 전부 한 번 본 것으로
        s["vn"] = np.ones(len(s["vox"]), np.uint16)
    return s


def add_vox(s: dict, k: np.ndarray) -> None:
    """복셀을 더하고 본 횟수를 센다 — 한 장 안의 여러 점은 칸마다 한 번으로.
    2026-10-22 실기: 같은 자리 두 바퀴에서 새 복셀 중 첫 지도 8cm 안은 27%(방향 어긋남은 없었다) — 잡음 두께."""
    k = np.unique(k.astype(np.int16), axis=0)
    allv = np.concatenate([s["vox"], k])
    u, inv = np.unique(allv, axis=0, return_inverse=True)
    w = np.concatenate([s["vn"].astype(np.int64), np.ones(len(k), np.int64)])
    s["vox"], s["vn"] = u, np.minimum(np.bincount(inv.ravel(), w, len(u)), 65535).astype(np.uint16)


def solid(s: dict) -> np.ndarray:
    """화면에 그릴 복셀. ⚠ "두 번 넘게 본 칸만"은 한 바퀴 지도를 거의 지운다(2026-10-22 실기: 10,529 → 647) —
    20°씩 돌면 한 표면이 2~3장에 걸치지만 5cm 칸에선 잡음으로 매번 다른 칸에 떨어진다. 걸러 쓰려면 칸을 키우거나
    이웃을 함께 세야 한다. 지금은 다 그린다(횟수는 계속 센다)."""
    return s["vox"]


def save(s: dict) -> None:
    tmp = MAP + ".tmp.npz"
    np.savez_compressed(tmp, **{k: np.asarray(v) for k, v in s.items()})
    os.replace(tmp, MAP)          # 조작대가 반쯤 쓴 파일을 읽지 않게


def imu_now(max_age: float = 1.0):
    """IMU 발행기의 (boot, yaw rad, 정지?) — 없거나 1초 넘게 낡았거나 영점 전이면 None."""
    try:
        d = json.load(open(IMU_FILE))
    except (OSError, ValueError):
        return None
    if not d.get("ready") or time.time() - float(d.get("ts", 0)) > max_age:
        return None
    return float(d["boot"]), math.radians(float(d["yaw_deg"])), bool(d.get("still"))


def imu_prior(s: dict, prior, win_deg: float, imu):
    """마지막 자세 이후 IMU가 잰 회전으로 θ 짐작을 바꾸고 탐색 폭을 좁힌다 → (prior, win_deg, 회전 rad|None)."""
    prior = np.asarray(prior, float).copy()
    ref = np.asarray(s["imu"], float)
    if imu is None or not ref[0] == imu[0]:    # nan(기준 없음)이나 발행기가 다시 켜졌으면 못 쓴다
        return prior, win_deg, None
    d = imu[1] - ref[1]
    prior[2] = wrap(float(s["pose"][2]) + d)
    return prior, min(win_deg, IMU_WIN_DEG), d


def to_world(pose, p: np.ndarray) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    out = p.copy()
    out[:, 0] = x + c * p[:, 0] - s * p[:, 1]
    out[:, 1] = y + s * p[:, 0] + c * p[:, 1]
    return out


def integrate(s: dict, scan: np.ndarray, pts: np.ndarray, m: dict) -> None:
    """광선으로 빈 칸을, 끝점으로 장애물을 칠하고 3D 점을 5cm 복셀로 더한다."""
    lo = s["lo"]
    if len(scan):
        o = to_world(s["pose"], np.array([[m["x"], m["y"]]]))[0]
        hits = to_world(s["pose"], scan)
        d = hits - o
        L = np.hypot(d[:, 0], d[:, 1])
        f = np.arange(0.0, 1.0, RES / 2 / max(float(L.max()), RES))
        ray = o + d[:, None, :] * f[None, :, None]
        keep = f[None, :] * L[:, None] < L[:, None] - RES
        fi, hi = idx(ray[keep]), idx(hits)
        fi = fi[((fi >= 0) & (fi < N)).all(1)]
        hi = hi[((hi >= 0) & (hi < N)).all(1)]
        ff = np.setdiff1d(np.unique(fi[:, 0] * N + fi[:, 1]), hi[:, 0] * N + hi[:, 1])
        lo.flat[ff] -= 0.4
        lo.flat[np.unique(hi[:, 0] * N + hi[:, 1])] += 0.9
        np.clip(lo, -4.0, 4.0, out=lo)
    if len(pts):
        k = np.floor(to_world(s["pose"], pts) / RES).astype(np.int16)
        add_vox(s, k)


def wrist_to_body(p_arm_mm: np.ndarray, yaw_deg: float, m: dict) -> np.ndarray:
    """팔 base 좌표(mm, 회전판 위) → 차체 좌표(m). URDF와 같은 사슬: 회전판 축에서 Rz(yaw)."""
    t = math.radians(yaw_deg)
    c, s = math.cos(t), math.sin(t)
    p = np.asarray(p_arm_mm, float) / 1000.0
    return np.stack([m["arm_x"] + c * p[:, 0] - s * p[:, 1],
                     s * p[:, 0] + c * p[:, 1],
                     m["arm_z"] + p[:, 2]], 1)


def wrist_cloud(m: dict, stride: int = 4) -> np.ndarray:
    """손목 D405 한 장 → 차체 좌표 점(m). 팔 자세·회전판·손-눈 보정을 그 자리에서 읽는다."""
    sys.path.insert(0, os.path.join(HERE, "..", "src", "tomato_bridge"))
    from tomato_bridge.follower_io import FollowerIO
    from tomato_picker.config import DEPTH_CAMERAS
    from tomato_picker.hardware.cartesian import CartesianArm
    from tomato_picker.hardware.eye import EyeConfig
    from tomato_picker.hardware.handeye import tool_frame
    cfg = EyeConfig(camera="d405")
    if cfg.mount != "on_arm" or cfg.transform is None or not cfg.good:
        raise RuntimeError("D405 손-눈 보정(on_arm)이 없거나 나쁘다 — 먼저 보정한다")
    io = FollowerIO(hold_torque=True)
    try:
        pose, yaw = CartesianArm(io).pose(), io.read_yaw()
    finally:
        io.hold_close()
    spec = DEPTH_CAMERAS["d405"]
    after = time.time()                     # 팔을 읽은 뒤에 찍힌 장이어야 그 자세의 화면이다
    for _ in range(30):
        meta = json.load(open(spec["meta"]))
        if float(meta["ts"]) > after:
            break
        time.sleep(0.1)
    else:
        raise RuntimeError("D405 새 프레임이 없다 — depth-cam이 죽었다")
    dep = np.load(spec["depth"])
    # ponytail: 핀홀만(왜곡 계수 무시) — 5cm 칸에선 안 보인다. 칸을 줄이면 Intrinsics.deproject로.
    it = meta["intrinsics"]
    d = dep[::stride, ::stride].astype(np.float32) * float(meta["depth_scale_mm"])
    v, u = np.nonzero((d >= meta["min_mm"]) & (d <= meta["max_mm"]))
    Z = d[v, u]
    cam = np.stack([(u * stride - it["ppx"]) * Z / it["fx"], (v * stride - it["ppy"]) * Z / it["fy"], Z], 1)
    arm = tool_frame(pose).apply(cfg.transform.apply(cam))
    print("  손목: 회전판 %.1f° · 집게 (%.0f, %.0f, %.0f)mm · D405 %d점" % (yaw, pose.x, pose.y, pose.z, len(Z)))
    return wrist_to_body(arm, yaw, m)


def integrate_near(s: dict, pts: np.ndarray, m: dict) -> int:
    """손목 점을 3D 복셀에 더하고, 높이 띠 안의 점은 장애물로 칠한다(Astra 60cm 사각 메우기).
    ponytail: 끝점만 — 빈 칸 광선은 안 쏜다(손목 화각이 좁고 자세가 제각각). 치운 물건은 Astra가 지운다."""
    w = to_world(s["pose"], pts)
    band = w[(pts[:, 2] > m["floor_cut"]) & (pts[:, 2] < m["top"])]
    hi = idx(band[:, :2])
    hi = hi[((hi >= 0) & (hi < N)).all(1)]
    cells = np.unique(hi[:, 0] * N + hi[:, 1])
    s["lo"].flat[cells] += 0.9
    np.clip(s["lo"], -4.0, 4.0, out=s["lo"])
    if len(w):
        add_vox(s, np.floor(w / RES))
    return len(cells)


def field(lo: np.ndarray) -> np.ndarray:
    """장애물 칸 1, 이웃으로 갈수록 0.6배씩 — 맞춤의 우도장."""
    f = (lo > OCC).astype(np.float32)
    for _ in range(3):
        g = f.copy()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            g = np.maximum(g, np.roll(np.roll(f, dx, 0), dy, 1) * 0.6)
        f = g
    return f


NEAR = 0.05                    # 짐작에서 칸² 당 깎는 점수(맞은 점 1개 ≈ 1점). 모의 가기에서 0.2도 비슷 — 손으로 옮긴 뒤 0.5m 다시 찾기를 덜 막는 쪽
HIT = 0.36                     # 우도 이 이상 = 장애물 2칸 안에 떨어졌다


def match(s: dict, scan: np.ndarray, prior, win_xy: float, win_deg: float):
    """prior 둘레를 전수 탐색해 (자세, 일치율, 맞은 점 수, 모호) — 지도가 비었으면 (prior, 0, 0, False).

    ⚠ **처음 보는 곳은 틀린 게 아니다**(2026-10-22 실측). 화각이 58°라 35° 돌면 화면의
      60%가 지도에 없는 곳인데, 예전엔 전체 평균으로 매겨 그런 장을 전부 버렸다 → 지도가
      그쪽으로 영영 안 자랐고, 자세는 짐작만으로 263°까지 흘렀다. 이제 **모르는 칸에
      떨어진 점은 빼고**, 지도가 "빈 곳"이라 아는 칸에 떨어진 점만 모순으로 깎는다.
    """
    lo = s["lo"]
    F = field(lo)
    prior = np.asarray(prior, float)
    if F.max() < 1 or len(scan) < 15:
        return prior, 0.0, 0, False
    # ⚠ **조리개 문제**(2026-10-22 모의로 잡음) — 처음 보는 쪽 벽의 점은 지도에 없어 0점인데, 차체를
    #   이미 아는 쪽으로 밀면 그 점들이 벽선을 따라 아는 칸에 얹혀 점수가 오른다. 잡음·미끄럼 0, θ 정확인데도
    #   걸음마다 탐색 한계(15cm)까지 미끄러졌다(반시계로 돌면 늘 오른쪽으로). **짐작 자리에서 지도가 모르는
    #   칸에 떨어지는 점은 처음부터 빼고** 맞춘다 — 지도에 더하기는 전부 다 한다(integrate).
    pk = idx(to_world(prior, scan))
    inb = ((pk >= 0) & (pk < N)).all(1)
    known = dilate(lo != 0, 1)
    sel = np.zeros(len(scan), bool)
    sel[inb] = known[pk[inb, 0], pk[inb, 1]]
    if sel.sum() < 15:
        return prior, 0.0, 0, False
    scan = scan[sel]
    contra = (lo < -FREE) & (F < HIT)
    S = F - 0.5 * contra                        # 탐색 점수: 맞으면 +, 빈 곳에 장애물을 놓으면 −
    oi = np.arange(-round(win_xy / RES), round(win_xy / RES) + 1)
    # 동점이면 짐작에 가까운 쪽 — 벽 하나만 보이면 벽을 따라 미끄러져도 점수가 같다(합성 방에서 0.5m)
    # 짐작에서 멀수록 조금씩 깎는다 — 벽을 따라 미끄러져도 점수가 같거나 몇 점 오르는 자리로 튀지 않게
    # (모의 가기: 0.19m 튀었다 돌아오며 좌우로 흔들렸다). 진짜 고침은 맞은 점 수십 개로 이 값을 넘는다.
    near = NEAR * (oi[:, None] ** 2 + oi[None, :] ** 2)
    best, bp = -1e9, prior
    ths = np.radians(np.arange(-win_deg, win_deg + 1e-9, 1.0))
    per = np.empty(len(ths))
    for k, dth in enumerate(ths):
        p = idx(to_world((prior[0], prior[1], prior[2] + dth), scan))
        IX = p[:, 0, None, None] + oi[None, :, None]
        IY = p[:, 1, None, None] + oi[None, None, :]
        ok = (IX >= 0) & (IX < N) & (IY >= 0) & (IY < N)
        sc = np.where(ok, S[IX.clip(0, N - 1), IY.clip(0, N - 1)], 0).sum(0) - near - 1e-4 * abs(math.degrees(dth))
        i, j = np.unravel_index(int(sc.argmax()), sc.shape)
        per[k] = sc[i, j]
        if sc[i, j] > best:
            best = float(sc[i, j])
            bp = np.array([prior[0] + oi[i] * RES, prior[1] + oi[j] * RES, wrap(prior[2] + dth)])
    c = idx(to_world(bp, scan))
    c = c[((c >= 0) & (c < N)).all(1)]
    hit = int((F[c[:, 0], c[:, 1]] >= HIT).sum())
    bad = int(contra[c[:, 0], c[:, 1]].sum())
    # ⚠ **모호함** — 화각 58°의 한 장은 곧은 벽 한 토막일 때가 많고, 그건 방의 다른 벽에도
    #   똑같이 맞는다(합성 방에서 참 +100°와 가짜 −170°가 둘 다 일치 1.0). 15° 넘게 떨어진
    #   다른 방향이 거의 같은 점수면 고르지 않는다 — 틀린 자세로 지도를 더럽히는 것보다 싸다.
    far = np.abs((ths - wrap(bp[2] - prior[2]) + math.pi) % (2 * math.pi) - math.pi) > math.radians(15)
    amb = bool(far.any() and per[far].max() >= 0.9 * best)
    return bp, hit / max(1, hit + bad), hit, amb


# ── 길 ─────────────────────────────────────────────────────────────────
def dilate(mask: np.ndarray, r: int) -> np.ndarray:
    out = mask.copy()
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if 0 < dx * dx + dy * dy <= r * r:
                out |= np.roll(np.roll(mask, dx, 0), dy, 1)
    return out


def plan(lo: np.ndarray, start, goal, radius: float):
    """A*(8방향). 장애물은 차체 반지름만큼 부풀리고, 모르는 칸은 1.5배 비싸게 간다.
    → ([[x,y],...], "") 또는 (None, 이유)."""
    occ = lo > OCC
    r = int(math.ceil(radius / RES))
    block = dilate(occ, r)
    si, gi = tuple(int(v) for v in idx(start)), tuple(int(v) for v in idx(goal))
    if not all(0 <= v < N for v in si + gi):
        return None, "지도 밖이다"
    if block[gi]:
        return None, "목표가 장애물에 너무 가깝다(차체 반지름 %.2fm)" % radius
    # 벽 옆에서 출발해도 빠져나오게 — 출발 둘레는 부풀림을 풀고 진짜 장애물 칸만 남긴다
    x0, y0 = si
    win = (slice(max(0, x0 - r), x0 + r + 1), slice(max(0, y0 - r), y0 + r + 1))
    block[win] = occ[win]
    block[si] = False
    cost = np.where(lo < -FREE, 1.0, 1.5)
    D2 = math.sqrt(2)
    g = {si: 0.0}
    came = {}
    h = lambda a: max(abs(a[0] - gi[0]), abs(a[1] - gi[1])) + (D2 - 1) * min(abs(a[0] - gi[0]), abs(a[1] - gi[1]))  # noqa: E731
    pq = [(h(si), si)]
    while pq:
        _, c = heapq.heappop(pq)
        if c == gi:
            break
        gc = g[c]
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            n = (c[0] + dx, c[1] + dy)
            if not (0 <= n[0] < N and 0 <= n[1] < N) or block[n]:
                continue
            gn = gc + (D2 if dx and dy else 1.0) * cost[n]
            if gn < g.get(n, 1e18):
                g[n], came[n] = gn, c
                heapq.heappush(pq, (gn + h(n), n))
    if gi not in g:
        return None, "길이 없다(장애물로 막혔다)"
    cells = [gi]
    while cells[-1] != si:
        cells.append(came[cells[-1]])
    cells = cells[::-1][::3] + [gi]
    path = [[(a - N // 2 + 0.5) * RES, (b - N // 2 + 0.5) * RES] for a, b in cells]
    path[-1] = [float(goal[0]), float(goal[1])]
    return path, ""


# ── 실기 ───────────────────────────────────────────────────────────────
def frame(after: float, timeout: float = 40.0):
    """after 이후에 찍힌 Astra 깊이 한 장. 깊이 0%면 기다린다(발행기가 30초 뒤 스스로 다시 연다)."""
    from tomato_picker.config import DEPTH_CAMERAS
    spec = DEPTH_CAMERAS["astra"]
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            meta = json.load(open(spec["meta"]))
            if float(meta["ts"]) > after and float(meta.get("valid_frac", 0)) > 0.05:
                return np.load(spec["depth"]), meta
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(0.1)
    raise RuntimeError("Astra 새 프레임이 %.0f초째 없다 — astra-cam이 죽었거나 깊이 0%%"
                       "(60cm 안을 봤다)" % timeout)


START_COVER = 0.5              # 출발 맞춤의 겹침 하한(겹친 점/스캔 점) — localize 주석
NEAR_VALID = 0.08              # Astra 유효 깊이 비율이 이보다 낮으면 정면 60cm 안에 뭔가 있다(투광기가 꺼진다, 10/23 실기 0.00)


def astra_blocked(wait: float = 0.4) -> bool:
    """움직인 뒤 wait초 기다린 장이 '거의 다 무효'이거나 **아예 안 오는가** — 그러면 frame()은 40초를 헛기다린다.
    ⚠ 60cm 안을 30초 보면 발행기가 스스로 다시 열고, 그동안 meta가 멈춘다 — 낡은 meta를 '안 막힘'으로 보면
    40초 헛기다렸다(10/23). 3초 안에 새 장이 없으면 막힌 것으로 본다."""
    from tomato_picker.config import DEPTH_CAMERAS
    time.sleep(wait)
    t0 = time.time()
    while True:
        try:
            meta = json.load(open(DEPTH_CAMERAS["astra"]["meta"]))
            if time.time() - float(meta.get("ts", 0)) < 2:
                return float(meta.get("valid_frac", 1)) < NEAR_VALID
        except (OSError, ValueError):
            pass
        if time.time() - t0 > 3:
            return True
        time.sleep(0.2)


def refine(s: dict, pts: np.ndarray, m: dict, pose) -> np.ndarray:
    """격자 맞춤(5cm·1°, 2D 스캔 100점) 결과를 **지도 전체의 장애물 칸에 대한 점구름 ICP**로 다듬는다.
    한 장끼리 ICP는 벽 하나만 겹치면 벽을 따라 미끄러졌지만(되돌림), 방의 벽·모서리가 다 든 지도에는 평행이동이 선다.
    θ는 IMU가 더 낫다 — 격자 결과에서 3°·25cm 넘게 옮기자는 답은 안 쓴다(2026-10-23 실기: 이동 뒤 0.2~0.4m 미끄럼)."""
    occ = np.argwhere(s["lo"] > OCC)
    if len(occ) < 50:
        return pose
    dst = (occ - N // 2 + 0.5) * RES
    dst = dst[np.hypot(dst[:, 0] - pose[0], dst[:, 1] - pose[1]) < 4.5]
    got, frac, res = icp2d(flat(pts, m), dst, pose, iters=30)
    if frac >= 0.5 and res < 0.04 and np.hypot(*(got[:2] - pose[:2])) < 0.25 and abs(wrap(got[2] - pose[2])) < math.radians(3):
        return got
    return pose


def localize(s: dict, m: dict, prior, win_xy: float, win_deg: float, after: float,
             min_cover: float = 0.0) -> bool:
    """새 장 하나로 자세를 맞추고, 믿을 만하면 지도에 넣고 저장한다 → 지도와 맞춰 받아들였나.
    min_cover = 겹친 점 / 스캔 점의 하한. 출발 맞춤은 0.5를 쓴다 — 40/115(0.35)·일치 0.87짜리가
    0.36m 틀렸다(10/23). 기본 0은 예전 그대로(스캔처럼 처음 보는 곳이 많은 장을 버리지 않게)."""
    dep, meta = frame(after)
    imu = imu_now()
    prior, win_deg, d_imu = imu_prior(s, prior, win_deg, imu)
    pts = cloud(dep, meta, m)
    sc = scan2d(pts, m)
    pose, q, hit, amb = match(s, sc, prior, win_xy, win_deg)
    cover = hit / max(1, len(sc))
    strong = q >= Q_MIN and hit >= HIT_MIN and cover >= min_cover
    if not amb and strong:
        pose = refine(s, pts, m, pose)
    ok = False
    if amb and (s["lo"] > OCC).sum() >= 20:
        s["pose"] = np.asarray(prior, float)
        print("  ⚠ 모호 — 다른 방향도 거의 똑같이 맞는다(곧은 벽만 보인다). 이 장은 안 넣는다."
              " 손으로 돌렸으면 돌린 각도를 적어 [한 장 담기]")
    elif (s["lo"] > OCC).sum() < 20 or strong:
        s["pose"] = pose
        integrate(s, sc, pts, m)
        s["frames"] = int(s["frames"]) + 1
        ok = (s["lo"] > OCC).sum() >= 20 and strong
    else:
        s["pose"] = np.asarray(prior, float)
        print("  ⚠ 일치 %.2f(기준 %.2f)·겹친 점 %d(기준 %d)·겹침 %.0f%%(기준 %.0f%%) — 이 장은 지도에 안 넣는다(자세는 짐작)" %
              (q, Q_MIN, hit, HIT_MIN, 100 * cover, 100 * min_cover))
    s["q"] = q
    if imu is not None:
        s["imu"] = np.array(imu[:2])
    save(s)
    x, y, th = s["pose"]
    print("  자세 x=%.2f y=%.2f θ=%.0f° · 일치 %.2f · 겹친 점 %d/%d · 복셀 %d · %s" %
          (x, y, math.degrees(th), q, hit, len(sc), len(s["vox"]),
           "IMU 회전 %+.1f°" % math.degrees(d_imu) if d_imu is not None else
           ("IMU 기준 잡음" if imu is not None else "IMU 없음")))
    return bool(ok)


CLOSE_DEG = 70.0               # 고리 닫기: 처음 이만큼의 장이 기준, 한 바퀴 뒤 이만큼 더 돌며 다시 본다
COAST = 0.7                    # IMU가 목표의 이만큼 돌았다고 하면 지령을 끊는다 — 나머지는 감속 슬루가 민다


def turn(link, w: int, secs: float, deg: float) -> float:
    """회전 넛지. IMU가 있으면 deg만큼 돌 때까지(최대 secs×2.5) 밀고, 없으면 secs 동안 → IMU가 잰 도(없으면 nan).

    정지마찰 때문에 같은 지령이 0°도 40°도 된다(CLAUDE.md 1번 병) — 시간이 아니라 **돈 양**으로 끊는다.
    """
    from base_nudge import drive
    i0 = imu_now()
    if i0 is None:
        drive(link, 0, 0, w, secs)
        return float("nan")
    t0 = t_kick = time.monotonic()
    w_now, moved = w, False
    try:
        while time.monotonic() - t0 < secs * 2.5:
            i = imu_now()
            got = abs(math.degrees(i[1] - i0[1])) if i is not None and i[0] == i0[0] else 0.0
            if got >= deg * COAST:
                break
            moved = moved or got > 1.0
            # 아직 1°도 못 돌았으면 0.5초마다 +30 — 정지마찰 문턱을 IMU로 더듬는다(각도로 끊으니 넘쳐도 안전)
            if not moved and time.monotonic() - t_kick > 0.5 and abs(w_now) < 255:
                w_now = int(math.copysign(min(255, abs(w_now) + 30), w))
                t_kick = time.monotonic()
            link.set_velocity(0, 0, w_now)
            time.sleep(0.03)
    finally:
        link.stop()
    if w_now != w:
        print("  회전 지령 %d → %d 에서 움직였다(정지마찰)" % (w, w_now) if moved else
              "  회전 지령 %d → %d 까지 올려도 안 움직였다" % (w, w_now))
    for _ in range(30):                        # 멈출 때까지(감속 슬루) 기다려야 돈 양이 다 잡힌다
        time.sleep(0.1)
        i = imu_now()
        if i is not None and i[2]:
            break
    i = imu_now()
    got = math.degrees(i[1] - i0[1]) if i is not None and i[0] == i0[0] else float("nan")
    if abs(got) < 0.5:
        print("  ⚠ 지령은 나갔는데 IMU %.1f° — 안 돌았다(정지마찰·모터 전원·바퀴 걸림)" % got)
    return got


def hand_turn(deg: float, timeout: float = 60.0) -> float:
    """사람이 손으로 돌리는 한 걸음 — IMU가 deg×COAST 넘게 돌았고 **멈췄으면** 돈 도(못 기다리면 nan).
    모터 전원이 없어도 IMU만 있으면 한 바퀴 지도를 만든다(2026-10-22: 바퀴가 안 돌던 날 만든 길)."""
    i0 = imu_now()
    if i0 is None:
        return float("nan")
    print("  손으로 %.0f° 남짓 돌리고 멈춘다…" % deg)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        i = imu_now()
        if i is not None and i[0] == i0[0] and abs(math.degrees(i[1] - i0[1])) >= deg * COAST and i[2]:
            return math.degrees(i[1] - i0[1])
        time.sleep(0.1)
    return float("nan")


def flat(pts: np.ndarray, m: dict, n: int = 800) -> np.ndarray:
    """높이 띠 안의 3D 점 → 2D(x,y) n개로 고르게 솎는다(ICP용)."""
    p = pts[(pts[:, 2] > m["floor_cut"]) & (pts[:, 2] < m["top"])][:, :2]
    return p[np.linspace(0, len(p) - 1, n).astype(int)] if len(p) > n else p


def icp2d(src: np.ndarray, dst: np.ndarray, pose, iters: int = 40):
    """차체 좌표 점 src를 pose로 옮겨 dst(지도 좌표)에 맞춘다(점-점 ICP, 먼 짝은 버린다) → (pose, 짝 비율, 평균 잔차 m).
    numpy만 — 젯슨엔 scipy가 없다. 800×3000 전수 거리라 한 번에 수십 ms."""
    pose = np.asarray(pose, float).copy()
    if len(src) < 30 or len(dst) < 30:
        return pose, 0.0, float("inf")
    if len(dst) > 3000:
        dst = dst[np.linspace(0, len(dst) - 1, 3000).astype(int)]
    frac, res = 0.0, float("inf")
    for it in range(iters):
        w = to_world(pose, src)
        d2 = ((w[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        dd = np.sqrt(d2[np.arange(len(w)), j])
        ok = dd < max(0.3 * (1 - it / iters), 0.08)       # 처음엔 넓게, 끝엔 8cm 안의 짝만
        frac, res = float(ok.mean()), float(dd[ok].mean()) if ok.any() else float("inf")
        if ok.sum() < 30:
            return pose, 0.0, float("inf")
        a_, b_ = src[ok], dst[j[ok]]
        ca, cb = a_.mean(0), b_.mean(0)
        H = (a_ - ca).T @ (b_ - cb)
        th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        c, sn = math.cos(th), math.sin(th)
        new = np.array([cb[0] - (c * ca[0] - sn * ca[1]), cb[1] - (sn * ca[0] + c * ca[1]), th])
        done = np.hypot(*(new[:2] - pose[:2])) < 1e-4 and abs(wrap(new[2] - pose[2])) < 1e-5
        pose = new
        if done:
            break
    return pose, frac, res


def register(base: dict, shots, pose0, k: float, m: dict) -> np.ndarray:
    """새 자리의 한 바퀴(360°)를 **통째로** 기존 지도에 맞춰 시작 자리를 고친다 → pose0.
    시작 자리를 한 장(화각 58°)으로 잡으면 벽을 따라 0.2~0.3m 미끄러졌고(2026-10-23 실기: 그 바퀴가 지도와 28%만 맞았다),
    한 바퀴에는 벽 여러 면과 모서리가 다 들어 있어 미끄러질 데가 없다. 격자 전수 탐색(±0.5m·±10°) → 점구름 ICP 다듬기."""
    if (base["lo"] > OCC).sum() < 100:
        return pose0                                   # 첫 바퀴 — 맞출 지도가 없다
    sc_all = np.concatenate([to_world((0.0, 0.0, r * k), sc_) for r, _, sc_ in shots if len(sc_)])
    # θ는 IMU가 이미 안다 — ±10°로 찾으면 양끝이 20° 떨어져 "모호"에 걸렸다(2026-10-23 실기 세 번째 자리)
    got, q, hit, amb = match(base, sc_all, pose0, 0.5, 6)
    if amb or q < Q_MIN or hit < 50:
        print("  ⚠ 새 자리 맞춤 실패(일치 %.2f · 겹친 점 %d · 모호 %s) — 시작 짐작 그대로" % (q, hit, amb))
        return np.asarray(pose0, float)
    occ = np.argwhere(base["lo"] > OCC)
    L = np.concatenate([to_world((0.0, 0.0, r * k), flat(p_, m, 200)) for r, p_, _ in shots])
    ic, frac, res = icp2d(L, (occ - N // 2 + 0.5) * RES, got, iters=30)
    if frac >= 0.6 and res < 0.035 and np.hypot(*(ic[:2] - got[:2])) < 0.15 and abs(wrap(ic[2] - got[2])) < math.radians(2):
        got = ic
    d = got - np.asarray(pose0, float)
    print("  새 자리 맞춤(한 바퀴 통째): 일치 %.2f · 겹친 점 %d/%d · 시작 짐작에서 %.2fm %.1f° 고침" % (
        q, hit, len(sc_all), float(np.hypot(d[0], d[1])), math.degrees(wrap(d[2]))))
    return got


def rebuild(s: dict, base: dict, pose0, shots, k: float, m: dict) -> None:
    """모은 장으로 지도를 다시 짓는다 — θ = IMU × k, 자리 = 제자리.
    ⚠ 걸음마다 바로 앞 장과 ICP로 평행이동을 따라가 봤다(2026-10-22 모의) — 점-점 ICP도 벽 하나만 겹치면 벽을 따라
      미끄러져 "돌며 밀린 길이"가 참(0.03~0.2m)의 5~30배(0.9~1.5m)로 나왔고 지도가 94% → 25~45%로 나빠졌다. 되돌렸다.
    ponytail: 돌며 옆으로 밀리는 만큼은 못 본다 — 실측 어긋남(고리 닫기가 찍는다)이 크면 회전 중심 모델을 붙인다."""
    for kk, v in base.items():
        s[kk] = v
    for r, p_, sc_ in shots:
        s["pose"] = pose0 + [0, 0, r * k]
        s["pose"][2] = wrap(s["pose"][2])
        integrate(s, sc_, p_, m)
        s["frames"] = int(s["frames"]) + 1


def scan_imu(a, m, s, link, base: dict) -> int:
    """IMU 제자리 한 바퀴 — **걸음마다 맞추지 않는다.** θ=IMU, 자리=그대로로 장을 모으고(화면엔 바로 더해
    보여 준다), 한 바퀴를 넘기면 **첫 장과 마지막 장을 맞춰(고리 닫기) IMU 스케일을 고친 뒤** 지도를 다시 짓는다.

    왜(2026-10-22 모의, 방 5×3.5m·상자 둘·깊이 잡음·미끄럼·IMU 스케일 2% 틀림): 복셀이 표면 7.5cm 안에 든 비율이
    걸음마다 맞춤 37~43% · θ만 맞춤 53~74% · **스케일 맞는 IMU만 84~88%** · 참 자세 97%. 5cm·1° 격자 맞춤이
    잘 맞춘 자이로보다 거칠다. 남는 오차는 스케일 하나인데, 그건 한 바퀴 돈 뒤 같은 곳을 다시 보면 잴 수 있다.
    ponytail: 제자리라 자리는 0으로 둔다 — 메카넘이 돌며 옆으로 밀리는 만큼(실측 전)은 못 본다. 크면 고리 닫기에서 xy도 나눠 준다.
    """
    pose0 = s["pose"].copy()
    for _ in range(100):                      # 첫 장은 고리 닫기의 기준 — 도는 중에 찍히면 지도 전체가 비뚤어진다
        i = imu_now()
        if i is not None and i[2]:
            break
        time.sleep(0.1)
    dep, meta = frame(time.time())
    i0 = imu_now()
    pts = cloud(dep, meta, m)
    shots = [(0.0, pts, scan2d(pts, m))]
    stuck, rel = 0, 0.0
    steps = max(a.steps, int(math.ceil(400.0 / (a.deg * COAST))))   # 걸음 수가 아니라 한 바퀴가 목표다
    for i in range(steps):
        g = hand_turn(a.deg) if link is None else turn(link, a.w, a.wsecs, a.deg)
        if not abs(g) >= 0.5:                       # nan(IMU 끊김) 포함
            stuck += 1
            if stuck >= 5:
                print("⛔ 다섯 번 연속 안 돌았다 — 회전 크기·시간을 올리거나 모터 전원을 본다(손 스캔이면 60초 안에 돌린다)")
                return 1
            # 2026-10-22 실기: 7.8V에선 연달아 밀면 보드가 브라운아웃 리셋을 되풀이하며 255에서도 안 돈다 —
            # 20초 쉬고 나니 바로 돌았다. 전원이 숨 돌릴 틈을 준다.
            if link is not None:
                time.sleep(3.0)
            continue
        stuck = 0
        try:
            # 벽 60cm 안을 보면 Astra가 투광기를 끄고 발행기가 30초 뒤 다시 연다(~40초) — 그만큼은 기다린다
            dep, meta = frame(time.time() + 0.4, timeout=60.0)
        except RuntimeError as e:
            print("  ⚠ 이 장은 건너뛴다 — %s" % e)    # 장 하나 때문에 한 바퀴를 버리지 않는다(2026-10-23: 벽 가까운 자리)
            continue
        im = imu_now()
        if im is None or im[0] != i0[0]:
            print("⛔ IMU가 끊겼거나 다시 켜졌다 — 이 스캔은 버린다")
            return 1
        rel = im[1] - i0[1]
        pts = cloud(dep, meta, m)
        sc = scan2d(pts, m)
        shots.append((rel, pts, sc))
        s["pose"] = pose0 + [0, 0, rel]
        s["pose"][2] = wrap(s["pose"][2])
        integrate(s, sc, pts, m)
        s["frames"] = int(s["frames"]) + 1
        s["imu"] = np.array(im[:2])
        save(s)
        print("[%d/%d] IMU 누적 %.1f° (이번 %.1f°) · 복셀 %d" % (i + 1, steps, math.degrees(rel), g, len(s["vox"])))
        if abs(rel) >= 2 * math.pi + math.radians(CLOSE_DEG):
            break
    record(shots, m)
    return close_and_build(s, base, pose0, shots, m, (i0[0], i0[1] + rel))


SCANS = os.path.expanduser("~/space_scans")


def record(shots, m: dict) -> None:
    """한 바퀴의 장(IMU 각·점구름)을 남긴다 — PC에서 `replay`로 고리 닫기·다시 짓기를 실기 데이터로 되돌려 본다.
    (2026-10-22: 실기 장이 지도에 녹아 사라져 고리 닫기를 실제 방에서 조정할 수 없었다)"""
    try:
        os.makedirs(SCANS, exist_ok=True)
        f = os.path.join(SCANS, time.strftime("scan-%Y%m%d-%H%M%S.npz"))
        np.savez_compressed(f, r=np.array([r for r, _, _ in shots]),
                            n=np.array([len(p_) for _, p_, _ in shots]),
                            pts=np.concatenate([p_ for _, p_, _ in shots]).astype(np.float32),
                            ns=np.array([len(sc_) for _, _, sc_ in shots]),
                            sc=np.concatenate([sc_ for _, _, sc_ in shots]).astype(np.float32),
                            mount=json.dumps(m))          # 점을 편 장착값 — selfcal이 이 위에 더한다
        print("  장 기록 → %s" % f)
    except OSError as e:
        print("  ⚠ 장 기록 실패: %s" % e)


def load_shots(f: str):
    z = np.load(f)
    pi_, si_ = np.cumsum(np.r_[0, z["n"]]), np.cumsum(np.r_[0, z["ns"]])
    return [(float(z["r"][i]), z["pts"][pi_[i]:pi_[i + 1]].astype(float), z["sc"][si_[i]:si_[i + 1]].astype(float))
            for i in range(len(z["r"]))]


def close_and_build(s, base, pose0, shots, m, imu_end) -> int:
    """한 바퀴 넘게 모은 장 → 고리 닫기(ICP)로 IMU 스케일을 재고 지도를 다시 짓는다. scan_imu·replay 공용."""
    rel = shots[-1][0]
    if abs(rel) < 2 * math.pi:
        print("걸음을 다 썼다 — IMU %.0f°. 한 바퀴가 안 돼 스케일은 못 고친다(회전은 IMU 그대로)" % math.degrees(rel))
        if len(shots) > 1:
            pose0 = register(base, shots, pose0, 1.0, m)
            rebuild(s, base, pose0, shots, 1.0, m)
            if imu_end is not None:
                s["imu"] = np.array(imu_end)
            save(s)
        return 0
    # 고리 닫기 — 처음 CLOSE_DEG까지의 장의 점들에, 한 바퀴 뒤 같은 곳을 다시 본 장의 점을 **점구름 ICP**로 맞춘다.
    #   5cm 격자·2D 스캔 100점 맞춤은 가구 많은 방(2026-10-22 실기)에서 장마다 k가 0.962~1.0385로 흩어졌다.
    ref = np.concatenate([to_world((0.0, 0.0, r), flat(p_, m)) for r, p_, _ in shots
                          if abs(r) <= math.radians(CLOSE_DEG)])
    ks, xy = [], []
    for r, p_, _ in shots:
        if abs(r) >= 2 * math.pi - math.radians(10):
            # IMU 스케일 3% = 400°에 12° — 한 곳에서만 출발하면 그만큼 먼 답에 못 닿는다(모의). ±12°를 3°씩 출발해 고른다.
            src = flat(p_, m)
            best = max((icp2d(src, ref, (0.0, 0.0, wrap(r + math.radians(o)))) for o in range(-12, 13, 3)),
                       key=lambda g: (round(g[1], 2), -g[2]))
            got, frac, res = best
            if frac >= 0.5 and res < 0.05 and np.hypot(*got[:2]) < 0.3:   # 제자리인데 30cm 넘게 밀렸다 = 엉뚱한 답
                ks.append((r + wrap(got[2] - wrap(r))) / r)
                xy.append(got[:2])
    # 엉뚱한 답이 섞여도 끌려가지 않게 — **서로 0.75% 안에 가장 많이 모인 무리**만 쓴다. (중앙값 기준으로 거르면
    #   2026-10-23 실기처럼 0.8955·0.973이 중앙값을 끌어내려 좋은 답 0.997·0.9996까지 버렸다)
    if ks:
        sup = [sum(abs(v - u) <= 0.0075 for u in ks) for v in ks]       # ±0.75% → 무리 폭이 아래 판정(1.5%) 안
        c0 = ks[int(np.argmax(sup))]
        keep = [i for i, v in enumerate(ks) if abs(v - c0) <= 0.0075]
        ks, xy = [ks[i] for i in keep], [xy[i] for i in keep]
    k = float(np.median(ks)) if ks else 1.0
    d = np.median(np.array(xy), 0) if xy else np.zeros(2)
    print("고리 닫기(ICP): 맞춘 장 %d · 참 회전 = IMU × %.4f %s · 한 바퀴 뒤 자리 어긋남 %.2fm (%.2f, %.2f)" % (
        len(ks), k, np.round(ks, 4).tolist(), float(np.hypot(*d)), d[0], d[1]))
    # ⚠ 추정끼리 맞을 때만 — 흩어지면 IMU(데이터시트 스케일)를 믿는 편이 낫다(실기: 0.9707을 그대로 써 400°에 12° 비틀었다)
    spread = (max(ks) - min(ks)) if len(ks) >= 2 else 1.0
    if len(ks) < 2 or abs(k - 1) > 0.04 or spread > 0.015:
        print("  ⚠ 고리 닫기를 못 믿는다(맞춘 장 %d · 흩어짐 %.3f) — 회전은 IMU 그대로 둔다" % (len(ks), spread))
        k = 1.0
    if abs(k - 1) > 0.005:
        print("  IMU 스케일 %.4f — imu-yaw.service 의 IMU_SCALE 에 곱하면 다음부터 처음부터 맞는다" % k)
    pose0 = register(base, shots, pose0, k, m)
    rebuild(s, base, pose0, shots, k, m)
    if imu_end is not None:
        s["imu"] = np.array(imu_end)  # 기준 = 마지막 장의 IMU(자세 θ는 고친 값)
    save(s)
    print("✅ 한 바퀴 — 고친 회전 %.1f°로 지도를 다시 지었다 · 복셀 %d" % (math.degrees(rel * k), len(s["vox"])))
    return 0




def overlap(shots, dx: float, dy: float, dyaw: float, k: float, pairs, m0: dict) -> float:
    """장들을 θ = IMU × k로 겹쳤을 때 이웃 장 점끼리의 평균 거리(m, 25cm 넘는 것은 안 겹치는 곳이라 뺀다).
    렌즈를 (m0.x+dx, m0.y+dy)·yaw+dyaw로 옮겨 다시 편 것처럼 점을 고친다(기록 점은 m0로 편 차체 좌표)."""
    c, s_ = math.cos(dyaw), math.sin(dyaw)
    W = []
    for r, q in shots:
        v = q - [m0["x"], m0["y"]]
        W.append(to_world((0.0, 0.0, r * k), np.c_[m0["x"] + dx + c * v[:, 0] - s_ * v[:, 1],
                                                   m0["y"] + dy + s_ * v[:, 0] + c * v[:, 1]]))
    tot = cnt = 0
    for i, j in pairs:
        d = np.sqrt(((W[j][:, None] - W[i][None]) ** 2).sum(-1).min(1))
        d = d[d < 0.25]
        if len(d) > 30:
            tot, cnt = tot + d.mean(), cnt + 1
    return tot / max(cnt, 1)


def selfcal_cmd(files, write: bool) -> int:
    """기록한 제자리 한 바퀴들로 **장착값(렌즈 x·y)을 자기 보정**한다 — 이웃 장 겹침이 가장 작게.
    값은 회전 중심 기준이다(메카넘이 실제로 도는 축). 2026-10-23 실기: x 0.345·y −0.10(CAD x 0.12),
    두 바퀴 일관성 52→80%. 바퀴마다 k(IMU 스케일)는 따로 푼다. 격자 좌표하강이라 PC에서 바퀴당 수십 초."""
    data = []
    for f in files:
        z = np.load(f)
        m0 = {**MOUNT, **json.loads(str(z["mount"]))} if "mount" in z.files else dict(MOUNT)
        sh = [(r, flat(p_, m0, 500)) for r, p_, _ in load_shots(f)]
        rs = np.array([r for r, _ in sh])
        n = len(sh)
        pairs = [(i, i + 1) for i in range(n - 1)] + [(i, i + 2) for i in range(n - 2)]
        pairs += [(i, j) for i in range(n) for j in range(i + 1, n) if abs(rs[j] - rs[i] - 2 * math.pi) < math.radians(15)]
        data.append((sh, pairs, m0))
    if len({json.dumps({k: d[2][k] for k in ("x", "y", "yaw_deg")}) for d in data}) != 1:
        print("⛔ 기록마다 장착값이 다르다 — 같은 장착값으로 찍은 기록끼리만 넣는다")
        return 2
    m0 = data[0][2]
    p = {"dx": 0.0, "dy": 0.0, "dyaw": 0.0, **{"k%d" % i: 1.0 for i in range(len(data))}}
    J = lambda q: sum(overlap(sh, q["dx"], q["dy"], q["dyaw"], q["k%d" % i], pr, m0)   # noqa: E731
                      for i, (sh, pr, _) in enumerate(data)) / len(data)
    # ⚠ 카메라 yaw는 풀지 않는다 — 제자리 회전에선 거의 안 보인다(실기: ±3°에 겹침 0.057~0.061m, 모의: 참 0°를 −3°로).
    grid = {"dx": np.arange(-0.15, 0.301, 0.025), "dy": np.arange(-0.15, 0.151, 0.025),
            **{"k%d" % i: np.arange(0.97, 1.031, 0.005) for i in range(len(data))}}
    j0 = J(p)
    for _ in range(3):
        for key, vals in grid.items():
            p[key] = float(vals[int(np.argmin([J({**p, key: v}) for v in vals]))])
    j1 = J(p)
    new = {"x": round(m0["x"] + p["dx"], 3), "y": round(m0["y"] + p["dy"], 3),
           "yaw_deg": round(m0["yaw_deg"] + math.degrees(p["dyaw"]), 1)}
    print("이웃 장 겹침 %.3f → %.3f m · IMU k %s" % (j0, j1, [round(p["k%d" % i], 3) for i in range(len(data))]))
    print("장착값 %s → %s" % ({k: m0[k] for k in new}, new))
    if j1 > 0.9 * j0:
        print("  ⚠ 10%도 안 줄었다 — 장착값 탓이 아니다. 안 쓴다")
        return 0
    if write:
        cur = {}
        try:
            cur = json.load(open(MOUNT_FILE))
        except (OSError, ValueError):
            pass
        cur.update(new)
        with open(MOUNT_FILE, "w") as fh:
            json.dump(cur, fh, ensure_ascii=False)
        print("  → %s 에 썼다(다음 스캔부터)" % MOUNT_FILE)
    return 0


def replay_cmd(f: str, m: dict) -> int:
    """기록한 한 바퀴를 빈 지도에 다시 짓는다(장치 없이) — ~/space_map.npz를 덮으니 PC에서 쓴다."""
    s = empty()
    shots = load_shots(f)
    print("장 %d · IMU %.0f°" % (len(shots), math.degrees(shots[-1][0])))
    return close_and_build(s, empty(), np.zeros(3), shots, m, None)


def scan_cmd(a, m) -> int:
    from base_nudge import open_link
    if astra_blocked(0):
        print("⛔ 정면 60cm 안에 물체(Astra 깊이가 꺼졌다) — 트인 쪽을 보게 돌리고 다시(첫 장을 40초 헛기다린다)")
        return 1
    s = load()
    # 시작 맞춤 **전** 지도 — 한 장(58°)으로 잡은 시작 자리는 벽을 따라 0.2~0.3m 틀릴 수 있어(2026-10-23 실기),
    # 그 장을 섞지 않은 지도에 한 바퀴를 통째로 맞춘다(register).
    base0 = {k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in s.items()}
    localize(s, m, s["pose"], 0.4, 30, time.time())
    if getattr(a, "hand", False):
        if imu_now() is None:
            print("⛔ 손으로 돌리는 스캔은 IMU가 있어야 한다 — systemctl status imu-yaw")
            return 1
        return scan_imu(a, m, s, None, base0)
    if not yaw_front_ok():
        return 1
    link = open_link()
    if link is not None and imu_now() is not None:
        try:
            return scan_imu(a, m, s, link, base0)
        finally:
            link.close()
    if link is None:
        return 1
    est, total, stuck = 0.0, 0.0, 0
    sum_map = sum_gyro = 0.0                   # 지도가 받아들인 걸음만 — IMU 스케일 보정용
    try:
        for i in range(a.steps):
            prev = s["pose"].copy()
            gyro = turn(link, a.w, a.wsecs, a.deg)
            ok = localize(s, m, prev + [0, 0, est], 0.15, 60, time.time() + 0.4)
            d = wrap(s["pose"][2] - prev[2])
            print("[%d/%d] 넛지 한 번 = %.0f° (IMU %.1f°)" % (i + 1, a.steps, math.degrees(d), gyro))
            if ok and abs(gyro) > 3:
                sum_map, sum_gyro = sum_map + math.degrees(d), sum_gyro + gyro
                if abs(sum_gyro) > 90:
                    # ⚠ 맞춤은 IMU ±5° 안에서만 찾으니 스케일이 5%/걸음 넘게 틀리면 여기서도 안 보인다
                    print("  IMU 스케일: 지도/IMU = %.3f (%.0f° 기준) — 1에서 3%% 넘게 다르면 IMU_SCALE에 곱한다"
                          % (sum_map / sum_gyro, sum_gyro))
            if abs(d) > math.radians(30):
                # 화각 58°에서 30° 넘게 돌면 겹치는 곳이 절반이 안 된다 — 맞춤이 위태롭다
                print("  ⚠ 한 번에 너무 많이 돈다 — 회전 초를 줄여라(20° 안팎이 좋다)")
            if abs(d) < math.radians(2):
                stuck += 1
                if stuck >= 3:
                    print("⛔ 세 번 연속 안 돌았다 — 회전 크기·시간을 올려라(정지마찰)")
                    return 1
                continue
            stuck, est, total = 0, d, total + abs(d)
            if total >= 2 * math.pi:
                print("✅ 한 바퀴(%.0f°)" % math.degrees(total))
                return 0
        print("걸음을 다 썼다 — %.0f° 돌았다" % math.degrees(total))
        return 0
    finally:
        link.close()


def step_cmd(a, m) -> int:
    """손으로 한 걸음(앞뒤·옆·회전) → 지도로 자세를 다시 맞추고 그 장을 더한다."""
    from base_nudge import drive, open_link
    s = load()
    prev = s["pose"].copy()
    if a.vx > 0 and astra_blocked(0):
        print("⛔ 정면 60cm 안에 물체 — 앞으로는 안 간다(뒤·옆·회전은 된다)")
        return 1
    link = open_link()
    if link is None:
        return 1
    try:
        if a.w and not (a.vx or a.vy):
            print("회전 걸음 — IMU %.1f°" % turn(link, a.w, a.secs, a.deg))
        else:
            drive(link, a.vx, a.vy, a.w, a.secs)
    finally:
        link.close()
    # 짐작만 준다 — 실제로 얼마 갔는지는 맞춤이 정한다(넛지 크기는 바닥·배터리마다 다르다).
    f, l = 0.10 * np.sign(a.vx), 0.08 * np.sign(a.vy)
    c, sn = math.cos(prev[2]), math.sin(prev[2])
    prior = prev + [f * c - l * sn, f * sn + l * c, math.radians(20) * np.sign(a.w)]
    if astra_blocked():
        print("⚠ 움직인 뒤 정면 60cm 안에 물체 — 자세를 못 맞춘다(지도는 그대로). 뒤로 한 걸음 빼라")
        return 1
    localize(s, m, prior, 0.05 if (a.w and not (a.vx or a.vy)) else 0.3, 50, time.time() + 0.4)
    d = s["pose"] - prev
    print("한 걸음 → %.2fm %.0f°" % (float(np.hypot(d[0], d[1])), math.degrees(wrap(d[2]))))
    return 0


def wrist_cmd(m) -> int:
    s = load()
    k = integrate_near(s, wrist_cloud(m), m)
    save(s)
    print("손목 D405를 지도에 더했다 — 장애물 칸 %d · 복셀 %d" % (k, len(s["vox"])))
    return 0


YAW_FRONT_DEG = 15.0           # 회전판이 기본 자리(BASE_YAW_HOME_DEG)에서 이보다 벗어나면 팔이 차체 옆으로 나온다 — 돌거나 다니지 않는다


def turntable_deg():
    """회전판 각(도, 왼쪽 +) — 팔 버스에서 ID 7만 읽는다(팔 관절 토크는 안 건드린다). 못 읽으면 None."""
    try:
        sys.path.insert(0, os.path.join(HERE, "..", "src", "tomato_bridge"))
        from tomato_bridge.follower_io import BASE_YAW_ID, FollowerIO, yaw_deg_from_raw
        FollowerIO._find_lerobot()                                  # noqa: SLF001 - 컨테이너 밖에선 venv
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
        from tomato_picker.config import ARM_FOLLOWER_SERIAL, ARM_LEADER_SERIAL, ARM_SERIAL_PORT
        from tomato_picker.hardware.ports import resolve_arm_port
        port = resolve_arm_port(ARM_SERIAL_PORT, follower_serial=ARM_FOLLOWER_SERIAL,
                                leader_serial=ARM_LEADER_SERIAL)
        bus = FeetechMotorsBus(port, {"y": Motor(BASE_YAW_ID, "sts3215", MotorNormMode.DEGREES)})
        bus.connect(handshake=False)
        try:
            raw, _, _ = bus._read(56, 2, BASE_YAW_ID, num_retry=2)   # noqa: SLF001 - Present_Position
        finally:
            bus.disconnect(disable_torque=False)
        return yaw_deg_from_raw(raw)
    except Exception as exc:                                         # noqa: BLE001
        print("  ⚠ 회전판을 못 읽었다(%s) — 정면이라고 믿고 간다" % str(exc)[:60])
        return None


def yaw_front_ok() -> bool:
    sys.path.insert(0, os.path.join(HERE, "..", "src", "tomato_bridge"))
    from tomato_bridge.follower_io import BASE_YAW_HOME_DEG
    y = turntable_deg()
    if y is not None and abs(y - BASE_YAW_HOME_DEG) > YAW_FRONT_DEG:
        print("⛔ 회전판이 %.0f°(기본 %.0f°) — 팔이 차체 옆으로 나와 돌거나 다니면 걸린다(10/23에 걸림). "
              "[대기 자세]부터" % (y, BASE_YAW_HOME_DEG))
        return False
    return True


TOMATO_STATUS = "/dev/shm/tomato_status"
TOMATO_GUESS = 1.5             # 지도에 벽이 없어 거리를 모르면 이만큼 앞에 둔다(화면에 '거리 모름'으로)


def tomato_bearings(st: dict, m: dict) -> list:
    """tomato-vision 상태 → 차체 기준 방위(rad, 왼쪽 +). 화면 오른쪽 = 로봇 오른쪽(−)."""
    w = float(st.get("w", 1280))
    f = (w / 2) / math.tan(math.radians(m["tcam_hfov"]) / 2)
    return [math.radians(m["tcam_yaw_deg"]) + math.atan2(w / 2 - float(cx), f)
            for cx, _ in st.get("positions", [])]


def tomato_elevations(st: dict, m: dict) -> list:
    """tomato-vision 상태 → 렌즈에서 본 높이각(rad, 위 +). 화소는 정사각이라 가로 화각의 f를 그대로 쓴다."""
    w, h = float(st.get("w", 1280)), float(st.get("h", 720))
    f = (w / 2) / math.tan(math.radians(m["tcam_hfov"]) / 2)
    return [math.atan2(h / 2 - float(cy), f) for _, cy in st.get("positions", [])]


def tomato_fresh(after: float, timeout: float = 4.0):
    """after 뒤에 쓰인 tomato-vision 상태(없으면 None). 옛 발행기는 ts가 없다 — 그러면 못 믿는다."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            st = json.load(open(TOMATO_STATUS))
            if float(st.get("ts", 0)) > after:
                return st
        except (OSError, ValueError):
            pass
        time.sleep(0.1)
    return None


def ray_hit(lo: np.ndarray, xy, th: float, rmin: float = 0.4, rmax: float = 4.0) -> float:
    """xy에서 th 방향으로 처음 만나는 장애물 칸까지 거리(m). 없으면 nan."""
    c, s_ = math.cos(th), math.sin(th)
    for r in np.arange(rmin, rmax, RES / 2):
        i = idx([xy[0] + r * c, xy[1] + r * s_])
        if not ((i >= 0) & (i < N)).all():
            break
        if lo[i[0], i[1]] > OCC:
            return float(r)
    return float("nan")


# 제자리 회전에 비워 둘 반지름. 실측 회전 중심이 차체 **뒤쪽**이라(렌즈가 중심에서 0.345m 앞, 렌즈 ≈ 맨 앞)
# 앞 모서리가 √(0.36²+0.14²) ≈ 0.39m를 그린다 — 차체 반지름 0.22+0.12로는 모자라 10/23 세 번 가구에 걸렸다.
SPIN_R = 0.45


def clearance(lo: np.ndarray, xy, rmax: float = 1.0) -> float:
    """xy에서 가장 가까운 장애물 칸까지 거리(m). rmax 안에 없으면 rmax."""
    c, r = idx(xy), int(rmax / RES)
    lo_ = np.clip(c - r, 0, N)
    win = lo[lo_[0]:min(N, c[0] + r + 1), lo_[1]:min(N, c[1] + r + 1)] > OCC
    if not win.any():
        return rmax
    ii, jj = np.nonzero(win)
    return float(min(rmax, np.hypot(ii + lo_[0] - c[0], jj + lo_[1] - c[1]).min() * RES))


TOMATO_SAME = 0.3              # 이 거리 안의 자리는 같은 나무로 묶는다(m)


def slice_valid(dep: np.ndarray, meta: dict, m: dict, b: float, half: float = math.radians(2),
                e: float | None = None) -> float:
    """차체 기준 방위 b(±half)에 드는 깊이 열들 가운데 유효 깊이의 비율. 행은 높이각 e(±ELEV_HALF)가 있으면
    그 줄, 없으면 위아래 1/4 뺀 가운데. 가까운(60cm 안) 물체는 깊이가 0이라 띠에 **뒤쪽 배경만** 남는다 —
    그걸 거리로 쓰면 1m 넘게 틀린다(10/23)."""
    it, w = meta["intrinsics"], dep.shape[1]
    u = np.arange(w)
    uu = (int(meta.get("width", w)) - 1 - u) if meta.get("mirrored") else u   # cloud()와 같은 거울 처리
    ang = np.arctan2(-(uu - it["ppx"]), it["fx"]) + math.radians(m["yaw_deg"])   # 화면 오른쪽 = 차체 오른쪽(−)
    cols = np.nonzero(np.abs((ang - b + math.pi) % (2 * math.pi) - math.pi) < half)[0]
    if not len(cols):
        return 0.0
    h = dep.shape[0]
    if e is None:
        r0, r1 = h // 4, 3 * h // 4
    else:                                   # cloud()와 같은 기울기 규약: 행 v의 높이각 = tilt − atan((v−ppy)/fy)
        t = math.radians(m["tilt_deg"])
        r0, r1 = (int(np.clip(it["ppy"] + it["fy"] * math.tan(t - e + k * ELEV_HALF), 0, h)) for k in (-1, 1))
        if r1 <= r0:
            return 0.0
    d = dep[r0:r1, cols].astype(np.float32) * float(meta["depth_scale_mm"])
    return float(((d >= meta["min_mm"]) & (d <= meta["max_mm"])).mean())


SLICE_MIN = 0.3                # 그 방위 띠의 유효 깊이가 이보다 적으면 거리를 모르는 것으로(가깝거나 가려짐)
# 토마토 높이각 ±이만큼의 줄만 본다. 세로 띠 전체를 보면 바닥·흰 화분(깊이 0)과 뒤 벽이 섞여
# 1.13m 토마토를 1.83m로 쟀다 — 10/24 큰 방에서 찾기 7번이 전부 '?'였던 원인.
ELEV_HALF = math.radians(3)


def tomato_ranges(m: dict, bearings, ts: float, elevs=None) -> list:
    """같은 순간 Astra 깊이에서 각 방위(차체 기준)의 거리를 **렌즈에서** 잰다 — 그 방위 ±2°(높이각을 주면
    그 ±ELEV_HALF 줄) 띠의 가까운 20%. 컬러와 깊이는 정렬돼 있지 않지만 방향은 같다. 유효 깊이가 SLICE_MIN 밑이면 nan."""
    try:
        dep, meta = frame(ts - 0.3, timeout=2.0)
    except RuntimeError:
        return [float("nan")] * len(bearings)
    pts = cloud(dep, meta, m)
    rel = pts[:, :2] - [m["x"], m["y"]]
    d, ang = np.hypot(rel[:, 0], rel[:, 1]), np.arctan2(rel[:, 1], rel[:, 0])
    band = (pts[:, 2] > m["floor_cut"]) & (pts[:, 2] < m["top"])
    elv = np.arctan2(pts[:, 2] - m["h"], d)
    out = []
    for b, e in zip(bearings, elevs if elevs is not None else [None] * len(bearings)):
        if slice_valid(dep, meta, m, b, e=e) < SLICE_MIN:
            out.append(float("nan"))
            continue
        sel = band & (np.abs((ang - b + math.pi) % (2 * math.pi) - math.pi) < math.radians(2))
        if e is not None:
            sel &= np.abs(elv - e) < ELEV_HALF
        out.append(float(np.percentile(d[sel], 20)) if sel.sum() >= 5 else float("nan"))
    return out


def place_tomatoes(s: dict, seen, min_count: int = 2) -> np.ndarray:
    """본 것들 [(렌즈 x, y, 세계 방위, 렌즈→거리|nan)] → 지도 위 토마토 자리 [(x, y, 본 횟수, 거리 아는가)].

    카메라는 회전 중심에서 0.36m 떨어져 같이 돈다 — 방위를 중심에서 쏘면 자리가 0.3m씩 흩어졌다(10/23 세 번).
    거리를 못 잰 것은 지도 벽까지 광선, 그것도 없으면 1.5m. 한 번만 본 자리는 오검출로 버린다."""
    pts = []
    # 깊이로 잰 것이 하나라도 있으면 못 잰 것은 버린다 — 화면 가장자리(컬러 66° > 깊이 58°)에서 본 것이
    # 광선으로 먼 벽에 떨어져 가짜 토마토가 됐다(10/23: (1.38, 1.93))
    if any(not math.isnan(r) for *_, r in seen):
        seen = [q for q in seen if not math.isnan(q[3])]
    for ox, oy, b, r in seen:
        known = not math.isnan(r)
        if not known:
            r = ray_hit(s["lo"], (ox, oy), b)
            known = not math.isnan(r)
            r = r if known else TOMATO_GUESS
        pts.append((ox + r * math.cos(b), oy + r * math.sin(b), known))
    out, left = [], list(range(len(pts)))
    while left:
        near = lambda i: [j for j in left if math.hypot(pts[j][0] - pts[i][0], pts[j][1] - pts[i][1]) < TOMATO_SAME]  # noqa: E731
        grp = near(max(left, key=lambda i: len(near(i))))
        left = [j for j in left if j not in grp]
        if len(grp) >= min_count:
            out.append([float(np.mean([pts[j][0] for j in grp])), float(np.mean([pts[j][1] for j in grp])),
                        len(grp), 1.0 if any(pts[j][2] for j in grp) else 0.0])
    return np.array(sorted(out, key=lambda t: -t[2])).reshape(-1, 4)


def find_cmd(a, m) -> int:
    """제자리에서 한 바퀴 돌며 tomato-vision이 본 토마토의 방위를 모아, 지도 벽에 쏴서 자리를 정한다."""
    from base_nudge import open_link
    if imu_now() is None:
        print("⛔ IMU가 없다 — systemctl status imu-yaw (방위를 IMU로 잰다)")
        return 1
    if tomato_fresh(time.time() - 3) is None:
        print("⛔ tomato-vision 상태가 3초째 안 바뀐다(또는 옛 발행기라 ts가 없다) — systemctl status tomato-vision")
        return 1
    if astra_blocked(0):
        print("⛔ 정면 60cm 안에 물체(Astra 깊이가 꺼졌다) — 트인 쪽으로 돌리거나 뒤로 빼고 다시")
        return 1
    s = load()
    # 제자리에서 돌면 한 바퀴에 0.1m쯤 밀린다(메카넘) — 스캔·찾기를 거듭하다 서랍장·화분에 닿았다(10/23).
    # 자리를 모르거나 둘레에 장애물이 있으면 돌지 않는다.
    if not localize(s, m, s["pose"], 0.4, 30, time.time(), min_cover=START_COVER):
        print("⛔ 지금 자리를 지도로 못 맞췄다(60cm 안을 보고 있거나 지도 밖) — 넓은 곳으로 옮기고 [제자리 한 바퀴 스캔]부터")
        return 1
    gap, need = clearance(s["lo"], s["pose"][:2]), m.get("spin_r", SPIN_R)
    if gap < need:
        print("⛔ 둘레 %.2fm 안에 장애물 — 돌면 부딪힌다(%.2fm 이상 띄운다, 회전판은 정면에)" % (gap, need))
        return 1
    if not yaw_front_ok():
        return 1
    th0, i0, seen, total, stuck = float(s["pose"][2]), imu_now(), [], 0.0, 0
    link = None if a.hand else open_link()
    if not a.hand and link is None:
        return 1
    try:
        for k in range(max(a.steps, int(3 * 360 / a.deg))):   # 360°를 채울 때까지 — 걸음이 1~4°로 끝나는 일이 잦다(10/23 실기 22걸음에 247°)
            st = tomato_fresh(time.time() + 0.3)         # 멈춘 뒤에 찍힌 판정
            i = imu_now()
            if st is None or i is None or i[0] != i0[0]:
                print("⛔ 검출이나 IMU가 끊겼다")
                return 1
            th = wrap(th0 + i[1] - i0[1])
            bs = tomato_bearings(st, m)
            lens = to_world([s["pose"][0], s["pose"][1], th], np.array([[m["x"], m["y"]]]))[0]
            rs = tomato_ranges(m, bs, float(st["ts"]), tomato_elevations(st, m)) if bs else []
            seen += [(lens[0], lens[1], wrap(th + b), r) for b, r in zip(bs, rs)]
            if bs:
                print("    거리(렌즈) %s" % ["%.2fm" % r if r == r else "?" for r in rs])
            print("[%d] 방향 %.0f° · 토마토 %d (cx %s)" % (k + 1, math.degrees(th), len(bs),
                                                    [int(c) for c, _ in st.get("positions", [])]))
            if total >= 360:
                break
            got = hand_turn(a.deg) if a.hand else turn(link, a.w, a.wsecs, a.deg)
            if math.isnan(got) or abs(got) < 2:
                stuck += 1
                if stuck >= 3:
                    print("⛔ 세 번 연속 안 돌았다 — 회전 크기를 올려라(정지마찰)")
                    break
                continue
            stuck, total = 0, total + abs(got)
    finally:
        if link is not None:
            link.close()
    s["tomato"] = place_tomatoes(s, seen)
    save(s)
    if not len(s["tomato"]):
        print("토마토를 못 찾았다(두 번 넘게 본 방향이 없다) — 본 방위 %d개, %.0f° 돌았다" % (len(seen), total))
        return 0
    for x, y, k, known in s["tomato"]:
        print("🍅 (%.2f, %.2f) · %d번 봄 · %s" % (x, y, k, "거리 잼(Astra 깊이, 없으면 지도 벽)" if known else
                                                 "거리 모름(지도에 그 방향 벽이 없다 — 1.5m로 둠)"))
    return 0


FACE_TOL_PX = 100               # 토마토 검출 cx가 화면 가운데에서 이만큼(≈6°) 안이면 정면 — 80은 557(오차 83)에서 헛돌았다


def tomato_cx(sec: float = 2.0):
    """sec 동안 tomato-vision 판정을 모아 cx 중앙값(없으면 None) — 검출이 프레임마다 깜빡인다(10/23)."""
    t0, cs, w = time.time(), [], 1280
    while time.time() - t0 < sec:
        st = tomato_fresh(time.time())
        if st:
            w = int(st.get("w", w))
            cs += [float(p[0]) for p in st.get("positions", [])]
    return (float(np.median(cs)), w) if cs else (None, w)


def face_cmd(a, m) -> int:
    """찾아 둔 토마토 쪽으로 차체를 돌려 앞 카메라 정면에 둔다 — 지도 방위로 한 번에 돌고(IMU), 검출로 다듬는다.
    10/23 실기: 지도 방위만으로 −57.5° 돌아 cx 670(정면)."""
    from base_nudge import open_link
    s = load()
    if not len(s["tomato"]):
        print("⛔ 찾아 둔 토마토가 없다 — [🍅 토마토 찾기]부터")
        return 1
    if astra_blocked(0):
        print("⛔ 정면 60cm 안에 물체(Astra 깊이가 꺼졌다) — 트인 쪽으로 돌리거나 뒤로 빼고 다시")
        return 1
    if not localize(s, m, s["pose"], 0.4, 30, time.time(), min_cover=START_COVER):
        print("⛔ 지금 자리를 지도로 못 맞췄다 — 트인 쪽을 보게 돌리거나 [한 장 담기] 뒤 다시")
        return 1
    gap, need = clearance(s["lo"], s["pose"][:2]), m.get("spin_r", SPIN_R)
    if gap < need or not yaw_front_ok():
        if gap < need:
            print("⛔ 둘레 %.2fm 안에 장애물 — 돌면 부딪힌다" % gap)
        return 1
    t = s["tomato"][0]
    lens = to_world(s["pose"], np.array([[m["x"], m["y"]]]))[0]
    b = math.degrees(wrap(math.atan2(t[1] - lens[1], t[0] - lens[0]) - s["pose"][2]))
    link = open_link()
    if link is None:
        return 1
    try:
        print("지도 방위 %.0f° (자세 θ %.0f°, 토마토 (%.2f, %.2f))" % (b, math.degrees(s["pose"][2]), t[0], t[1]))
        cx0, _ = tomato_cx(1.5)
        if cx0 is not None:
            # 이미 보이면 지도 방위로 돌지 않는다 — 가까이선 토마토 자리 오차 0.1~0.2m가 방위로 16°까지 커졌다(10/23)
            print("  지금 화면에 토마토가 있다(cx %.0f) — 검출로만 다듬는다" % cx0)
        elif abs(b) > 5:
            print("  → IMU %.1f° 돌았다" % turn(link, int(math.copysign(a.w, b)), a.wsecs, abs(b)))
        for k in range(8):
            cx, w = tomato_cx()
            if cx is None:
                print("[%d] 2초 동안 안 보인다 — 지도 방위 쪽으로 8° 더" % k)
                turn(link, int(math.copysign(a.w, b)), a.wsecs, 8)
                continue
            err = w / 2 - cx
            print("[%d] cx 중앙값 %.0f (오차 %+.0f)" % (k, cx, err))
            if abs(err) < FACE_TOL_PX:
                print("✅ 토마토가 정면이다")
                return 0
            f = (w / 2) / math.tan(math.radians(m["tcam_hfov"]) / 2)
            d = math.degrees(math.atan2(err, f))
            turn(link, int(math.copysign(a.w, d)), a.wsecs, max(5.0, abs(d)))
        print("⛔ 여덟 번 다듬어도 정면에 안 왔다")
        return 1
    finally:
        link.close()
        # 돈 만큼 지도 자세를 맞춰 둔다(다음 일이 짐작으로 시작하지 않게). 끝에 가구를 보고 섰으면 건너뛴다 —
        # 안 그러면 40초 헛기다리다 예외로 끝났다(10/23 서랍장 앞)
        if astra_blocked():
            print("  ⚠ 끝 자리에서 정면 60cm 안이 막혀 자세를 못 맞췄다 — 다음 일 전에 뒤로 빼라")
        else:
            localize(s, m, s["pose"], 0.1, 90, time.time() + 0.4)


APPROACH_STANDOFF = 0.7        # 토마토에서 이만큼 앞에 선다 — 도착 허용 0.2 + 차체 앞 0.2 + 여유


def approach_goal(s: dict, standoff: float = APPROACH_STANDOFF):
    """지금 자리 → 첫 토마토로 가는 선 위, 토마토에서 standoff 앞의 점. 이미 그 안이면 지금 자리."""
    t, p = s["tomato"][0], s["pose"]
    d = math.hypot(t[0] - p[0], t[1] - p[1])
    k = max(0.0, (d - standoff) / d) if d > 1e-6 else 0.0
    return p[0] + (t[0] - p[0]) * k, p[1] + (t[1] - p[1]) * k


def approach_cmd(a, m) -> int:
    """🍅 토마토 앞으로 — 가기(70cm 앞) → 정면 보기. 거리를 모르는 토마토(지도 벽도 없음)로는 안 간다."""
    s = load()
    if not len(s["tomato"]):
        print("⛔ 찾아 둔 토마토가 없다 — [🍅 토마토 찾기]부터")
        return 1
    if not s["tomato"][0][3]:
        print("⛔ 이 토마토는 거리를 모른다(1.5m 짐작) — 넓은 곳에서 [🍅 토마토 찾기]를 다시")
        return 1
    # 먼저 정면 보기 — 자리 맞춤을 두 번(시작·끝) 하고 토마토를 정면에 둔다. 그 **뒤의** 자세로 거리를 잰다.
    # 첫 맞춤만 믿었더니 겹친 점 40/115짜리 자세가 0.36m 앞이라 '이미 도착'으로 보고 안 갔다(10/23).
    if face_cmd(a, m) != 0:
        print("  (정면 맞추기 실패 — 그래도 지도 길로 간다)")
    s = load()
    gx, gy = approach_goal(s)
    if math.hypot(gx - s["pose"][0], gy - s["pose"][1]) > a.tol:
        print("가기 → (%.2f, %.2f)" % (gx, gy))
        a.x, a.y = gx, gy
        if go_cmd(a, m) != 0:
            return 1
    else:
        print("이미 토마토 %.1fm 앞이다" % APPROACH_STANDOFF)
    s = load()
    if clearance(s["lo"], s["pose"][:2]) >= m.get("spin_r", SPIN_R):
        return face_cmd(a, m)
    cx, w = tomato_cx()
    if cx is not None and abs(w / 2 - cx) < FACE_TOL_PX:
        print("✅ 토마토 앞이고 정면이다(cx %.0f) — 둘레가 좁아 더 돌지 않는다" % cx)
        return 0
    print("⚠ 토마토 앞에 왔지만 둘레가 좁아 돌지 않는다 — 토마토 cx %s" % ("못 봄" if cx is None else "%.0f" % cx))
    return 1


NEAR_MM = 450.0                # 🍅 손목 근접: 손목 D405에서 토마토가 이 거리(mm)가 되면 멈춘다(D405 근거리 ≤500)
NEAR_STOP_MM = 200.0           # 화면 가운데 아래(화분 쪽)에 이보다 가까운 것이 보이면 멈춘다
NEAR_TURN_DEG = 6.0            # 토마토가 손목 화면 가운데에서 이보다 벗어나면 먼저 돈다


def wrist_tomato():
    """손목 D405(컬러=깊이 정렬)의 열매 → (u, 거리 mm, 가로 각 °, DepthView) | None.
    깊이가 잡히는(유효 거리 안) 것 가운데 가장 큰 것, 없으면 가장 큰 것(거리 nan). **익음으로 거르지 않는다** —
    가까이선 조명에 주황으로 보여 '안 익음'이 되고, 그 사이 2m 뒤 빨간 의자가 '익은 열매'로 뽑혔다(10/24).
    대기 자세(팔을 뒤로 접음)에서만 화면이 바로 서고 주행 정면을 본다 — 화면 오른쪽 = 차체 오른쪽."""
    from tomato_picker.hardware.depth_camera import DepthError, DepthView
    from tomato_picker.vision.color_detect import detect_fruits
    v = DepthView(camera="d405")
    v.require_color_aligned()
    fr = []
    for f in detect_fruits(v.color_bgr()):
        try:
            mm = v.depth_mm_at(*f.position)
        except DepthError:
            mm = float("nan")
        fr.append((mm == mm, f.area, f.position[0], mm))
    if not fr:
        return None
    _, _, u, mm = max(fr)
    it = v.intrinsics()
    return u, mm, math.degrees(math.atan2(it.ppx - u, it.fx)), v


def wrist_front_mm(v) -> float:
    """손목 화면 가운데 아래 칸(가로 25~75%, 세로 40~70%)의 가까운 5% 깊이(mm). 집게 손가락(아래 70% 밑)은 뺀다."""
    meta = v.meta()
    dep = v.depth_raw().astype(np.float32) * float(meta["depth_scale_mm"])
    h, w = dep.shape
    d = dep[int(h * 0.4):int(h * 0.7), int(w * 0.25):int(w * 0.75)]
    d = d[(d >= meta["min_mm"]) & (d <= meta["max_mm"])]
    return float(np.percentile(d, 5)) if d.size > 50 else float("inf")


def near_cmd(a, m) -> int:
    """🍅 손목 근접 — 토마토 앞(Astra 60cm 사각)에서 손목 D405 거리로 조금씩 다가가 NEAR_MM에 선다."""
    from base_nudge import drive, open_link
    if not yaw_front_ok():
        return 1
    link = open_link()
    if link is None:
        return 1
    try:
        last = None
        for k in range(10):
            got = wrist_tomato()
            if got is None:
                print("⛔ 손목 D405에 익은 토마토가 안 보인다 — [🍅 토마토 앞으로]로 정면에 선 뒤 다시")
                return 1
            u, mm, ang, v = got
            front = wrist_front_mm(v)
            print("[%d] 손목 토마토 u %.0f · %s · 가로 %+.1f° · 앞 아래 %.0fmm" % (
                k, u, "%.0fmm" % mm if mm == mm else "거리 ?", ang, front))
            if mm == mm and mm <= a.near_mm:
                print("✅ 손목 D405에서 토마토 %.0fmm — 근접 끝" % mm)
                return 0
            if front < NEAR_STOP_MM:
                print("⛔ 손목 화면 아래 %.0fmm에 물체(화분?) — 더 안 간다" % front)
                return 1
            if abs(ang) > NEAR_TURN_DEG:
                turn(link, int(math.copysign(a.w, ang)), a.wsecs, abs(ang))
                continue
            if mm != mm:
                print("⛔ 토마토 거리를 못 잰다(D405 유효 %s) — 더 안 간다" % v.meta().get("valid_frac"))
                return 1
            # 걸음을 더 줄일 수 없다(정지마찰: 0.6초는 0) — 남은 게 직전 걸음의 절반 밑이면 한 걸음 더가 더 크게 넘친다.
            # 10/24 실기: 484mm에서 34mm 남기고 한 걸음 → 349mm(135mm 넘침)
            if last is not None and mm - a.near_mm < 0.5 * (last - mm):
                print("✅ 손목 D405에서 토마토 %.0fmm — 한 걸음(%.0fmm)이 남은 %.0fmm보다 커서 여기서 멈춘다" % (
                    mm, last - mm, mm - a.near_mm))
                return 0
            # 정지마찰: 200·1.0초 = 7~12cm. 안 움직였으면(10mm 미만) 0.3초 늘린다
            secs = a.vsecs if (last is None or last - mm > 10) else a.vsecs + 0.3
            drive(link, a.vx, 0, 0, secs if mm - a.near_mm > 120 else max(0.7, secs * 0.7))
            last = mm
        print("⛔ 열 번 다가가도 %.0fmm에 못 닿았다" % a.near_mm)
        return 1
    finally:
        link.close()


def lookahead(path, xy, dist: float):
    for p in path:
        if math.hypot(p[0] - xy[0], p[1] - xy[1]) >= dist:
            return p
    return path[-1]


def go_cmd(a, m) -> int:
    from base_nudge import drive, open_link
    s = load()
    goal = np.array([a.x, a.y])
    if astra_blocked(0):
        print("⛔ 정면 60cm 안에 물체(Astra 깊이가 꺼졌다) — 뒤로 빼고 다시")
        return 1
    if not yaw_front_ok():
        return 1
    # 사람이 손으로 옮겼을 수 있다 — 처음엔 넓게 찾는다
    if not localize(s, m, s["pose"], 0.5, 40, time.time(), min_cover=START_COVER):
        # 짐작 자세로 길을 짜면 엉뚱한 데로 간다(10/23: 서랍장을 보고 출발 — 겹친 점 0/5)
        print("⛔ 출발 자리를 지도로 못 맞췄다 — 트인 쪽을 보게 돌리거나 [한 장 담기] 뒤 다시")
        return 1
    link = open_link()
    if link is None:
        return 1
    est_w, est_v, stuck, veer = math.radians(20), 0.10, 0, 0
    try:
        for i in range(a.max_steps):
            xy, th = s["pose"][:2], s["pose"][2]
            left = float(np.hypot(*(goal - xy)))
            if left < a.tol:
                print("✅ 도착 — 남은 %.2fm" % left)
                return 0
            path, why = plan(s["lo"], xy, goal, m["radius"])
            if path is None:
                print("⛔", why)
                return 1
            s["path"], s["goal"] = np.array(path), goal
            la = lookahead(path, xy, 0.35)
            err = wrap(math.atan2(la[1] - xy[1], la[0] - xy[0]) - th)
            prev = s["pose"].copy()
            # 전진마다 4~5°씩 틀어진다(10/23 실기) — 멀리선 20°까지 두지만 0.5m 안에선 10°에서 고친다
            # (안 그러면 도착할 때 표적이 화면 끝으로 밀린다)
            turn_th = math.radians(10 if left < 0.5 else 20)
            if abs(err) > turn_th and left < a.tol * 1.5:
                # 허용 바로 바깥에서 방향만 틀어진 것 — 돌면 엉뚱한 쪽(10/23: 0.23m에서 26° 돌아 서랍장에 붙음)을 본다
                print("✅ 도착(허용 근처 %.2fm — 마지막 회전은 생략)" % left)
                return 0
            if abs(err) > turn_th:
                sg = 1 if err > 0 else -1              # w=+ 가 반시계(왼쪽) — base_nudge 실측
                turn(link, sg * a.w, a.wsecs, min(abs(math.degrees(err)), 30.0))
                prior = prev + [0, 0, sg * est_w]
                what = "회전 %s" % ("왼" if sg > 0 else "오른")
            else:
                i0 = imu_now()
                drive(link, a.vx, 0, 0, a.vsecs)
                prior = prev + [est_v * math.cos(th), est_v * math.sin(th), 0]
                what = "전진"
                # ⚠ 전진이 회전으로 바뀐다 = 바퀴가 걸렸거나 미끄러진다(2026-10-23 실기: 전진마다 +26~30°, 그 자리 지도가 25° 틀어졌다)
                time.sleep(0.5)
                i1 = imu_now()
                turned = math.degrees(i1[1] - i0[1]) if i0 is not None and i1 is not None and i0[0] == i1[0] else 0.0
                if abs(turned) > 10:
                    veer += 1
                    print("  ⚠ 전진했는데 %.0f° 돌았다 — 바퀴가 걸렸거나 바닥이 미끄럽다" % turned)
                    if veer >= 2:
                        print("⛔ 전진이 두 번 연달아 회전이 됐다 — 이 자리 바닥·바퀴를 본다(이대로면 지도가 틀어진다)")
                        return 1
                else:
                    veer = 0
            if astra_blocked():
                print("⛔ %s 뒤 정면 60cm 안에 물체 — 멈춘다(10/23 실기: 목표 0.23m 앞 회전에서 서랍장에 붙음)" % what)
                return 1
            # 제자리 회전이면 자리는 거의 그대로다(장착값이 실제 회전 중심 기준) — 넓게 찾으면 벽을 따라 0.1~0.3m 미끄러졌다
            localize(s, m, prior, 0.05 if what != "전진" else 0.3, 25, time.time() + 0.4)
            dxy = float(np.hypot(*(s["pose"][:2] - prev[:2])))
            dth = wrap(s["pose"][2] - prev[2])
            print("[%d] %s → %.2fm %.0f° · 목표까지 %.2fm" %
                  (i + 1, what, dxy, math.degrees(dth), left))
            if dxy < 0.01 and abs(dth) < math.radians(1):
                stuck += 1
                if stuck >= 3:
                    print("⛔ 세 번 연속 안 움직였다 — 크기·시간을 올려라(정지마찰)")
                    return 1
                continue
            stuck = 0
            if what == "전진":
                est_v = max(0.02, dxy)
            else:
                est_w = max(math.radians(3), abs(dth))
        print("⛔ 걸음을 다 썼다")
        return 1
    finally:
        link.close()


# ── 장치 없이 ──────────────────────────────────────────────────────────
def _room_scan(pose, fov_deg=58.0, m=None):
    """방(-2..3 × -1.5..2) + 상자(1..1.4 × 0.3..0.9)를 2D 광선으로 쏜 차체 좌표 스캔."""
    out = []
    cc, ss = math.cos(pose[2]), math.sin(pose[2])
    ox, oy = pose[0] + m["x"] * cc, pose[1] + m["x"] * ss
    for a in np.radians(np.arange(-fov_deg / 2, fov_deg / 2, 0.5)):
        c, s = math.cos(pose[2] + a), math.sin(pose[2] + a)
        for r in np.arange(0.6, 4.0, 0.01):
            x, y = ox + r * c, oy + r * s
            if not (-2 < x < 3 and -1.5 < y < 2) or (1 < x < 1.4 and 0.3 < y < 0.9):
                dx, dy = x - pose[0], y - pose[1]
                out.append([cc * dx + ss * dy, -ss * dx + cc * dy])
                break
    return np.array(out)


def selftest() -> int:
    m = dict(MOUNT)
    # ① 기하 — 2m 앞 평평한 벽
    meta = {"intrinsics": {"fx": 570.0, "fy": 570.0, "ppx": 319.5, "ppy": 239.5},
            "depth_scale_mm": 1.0, "min_mm": 600.0, "max_mm": 4000.0}
    dep = np.full((480, 640), 2000, np.uint16)
    p = cloud(dep, meta, m, stride=1)
    c = p[240 * 640 + 320]
    assert abs(c[0] - (m["x"] + 2.0)) < 1e-3 and abs(c[1]) < 0.01 and abs(c[2] - m["h"]) < 0.01, c
    assert p[240 * 640, 1] > 1.0, "화면 왼쪽(u=0)은 차체 왼쪽(y+)이어야 한다"
    my = {**m, "y": -0.1, "yaw_deg": 90.0}                  # 렌즈가 오른쪽 10cm에서 왼쪽을 본다
    cy_ = cloud(dep, meta, my, stride=1)[240 * 640 + 320]
    assert abs(cy_[0] - m["x"]) < 0.01 and abs(cy_[1] - (-0.1 + 2.0)) < 0.01, cy_
    dep2 = np.zeros((480, 640), np.uint16)
    dep2[:, :100] = 2000                                   # 화면 왼쪽에만 벽
    assert cloud(dep2, meta, m, stride=4)[:, 1].mean() > 0.5, "거울 아님: 화면 왼쪽 = 차체 왼쪽"
    assert cloud(dep2, {**meta, "mirrored": True, "width": 640}, m, stride=4)[:, 1].mean() < -0.5, "거울상: 화면 왼쪽 = 차체 오른쪽"
    assert len(scan2d(p, m)) > 100
    # ② 맞춤 — 돌며 지도를 만든 뒤, 틀린 짐작에서 참 자세를 되찾는가
    s = empty()
    for k in range(15):
        s["pose"] = np.array([0.0, 0.0, math.radians(25 * k)])
        integrate(s, _room_scan(s["pose"], m=m), np.zeros((0, 3)), m)
    true = np.array([0.12, -0.08, math.radians(40)])
    got, q, hit, _ = match(s, _room_scan(true, m=m), true + [-0.08, 0.06, math.radians(-12)], 0.15, 25)
    assert np.hypot(*(got[:2] - true[:2])) < 0.06 and abs(wrap(got[2] - true[2])) < math.radians(2), (got, true, q)
    assert q > 0.9 and hit >= HIT_MIN, (q, hit)
    # ②-1 손으로 ~100° 돌려 놓고 [한 장 담기] — 각도를 모르면 곧은 벽이 반대쪽에도 맞아
    #   **모호로 거절**해야 하고(고르면 틀린다), "90° 돌렸다"를 주면 찾아야 한다
    t90 = np.array([0.05, 0.0, math.radians(100)])
    g90, _, _, amb = match(s, _room_scan(t90, m=m), [0.0, 0.0, 0.0], 0.5, 180)
    assert amb or abs(wrap(g90[2] - t90[2])) < math.radians(3), ("모호를 못 알아채고 틀린 답", g90)
    g90, q90, h90, amb = match(s, _room_scan(t90, m=m), [0.0, 0.0, math.radians(90)], 0.5, 45)
    assert not amb and np.hypot(*(g90[:2] - t90[:2])) < 0.08 and abs(wrap(g90[2] - t90[2])) < math.radians(3), (g90, q90)
    # ②-2 처음 보는 쪽으로 35° — 화면 절반이 지도에 없어도 **받아들인다**(예전엔 버렸다)
    s3 = empty()
    integrate(s3, _room_scan([0.0, 0.0, 0.0], m=m), np.zeros((0, 3)), m)
    t35 = np.array([0.0, 0.0, math.radians(35)])
    g35, q35, h35, _ = match(s3, _room_scan(t35, m=m), [0.0, 0.0, math.radians(20)], 0.15, 60)
    assert abs(wrap(g35[2] - t35[2])) < math.radians(2) and q35 >= Q_MIN and h35 >= HIT_MIN, (g35, q35, h35)
    # ③ 길 — 상자를 돌아가고, 벽 속 목표는 거절
    path, why = plan(s["lo"], (0, 0), (2.2, 0.6), m["radius"])
    assert path, why
    occ = dilate(s["lo"] > OCC, 2)
    assert not any(occ[tuple(idx(q_))] for q_ in path), "길이 장애물을 지난다"
    assert plan(s["lo"], (0, 0), (3.0, 0.0), m["radius"])[0] is None
    # ④ 손목 사슬 — 회전판 0°면 팔 base 원점이 (arm_x, 0, arm_z), 90°면 팔의 앞이 차체 왼쪽
    w0 = wrist_to_body(np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 50.0]]), 0.0, m)
    w9 = wrist_to_body(np.array([[100.0, 0.0, 50.0]]), 90.0, m)[0]
    assert np.allclose(w0[0], [m["arm_x"], 0, m["arm_z"]]) and np.allclose(w0[1], [m["arm_x"] + 0.1, 0, m["arm_z"] + 0.05])
    assert np.allclose(w9, [m["arm_x"], 0.1, m["arm_z"] + 0.05]), w9
    s2 = empty()
    k = integrate_near(s2, np.array([[0.3, 0.0, 0.2], [0.3, 0.0, 0.01]]), m)
    assert k == 1 and s2["lo"][tuple(idx([0.3, 0.0]))] > 0 and len(s2["vox"]) == 2, "바닥 점은 장애물이 아니다"
    integrate_near(s2, np.array([[0.3, 0.0, 0.2], [0.31, 0.0, 0.2], [0.9, 0.0, 0.2]]), m)
    assert sorted(s2["vn"].tolist()) == [1, 1, 1, 2], ("한 장에서 같은 칸은 한 번, 두 장이면 2", s2["vn"])
    # ⑤ IMU 짐작 — 기준과 같은 boot면 θ = 마지막 θ + 자이로 회전, 폭은 좁힌다. 다시 켜졌으면 안 쓴다.
    si = empty()
    si["pose"] = np.array([0.0, 0.0, math.radians(170)])
    si["imu"] = np.array([5.0, math.radians(10)])
    pr, wd, d = imu_prior(si, [0.1, 0.0, 0.0], 60, (5.0, math.radians(30), True))
    assert abs(math.degrees(pr[2]) + 170) < 1e-6 and wd == IMU_WIN_DEG and pr[0] == 0.1, (pr, wd)
    assert imu_prior(si, [0, 0, 0.3], 60, (6.0, 0.0, True))[2] is None, "다른 boot의 yaw를 이었다"
    assert imu_prior(empty(), [0, 0, 0.3], 60, (5.0, 0.0, True))[1] == 60, "기준 없이 폭을 좁혔다"
    # ⑤-1 IMU 폭이면 곧은 벽의 모호함이 없다 — ②-1에서 모호했던 100°를 짐작만으로 찾는다
    gi, _, _, amb = match(s, _room_scan(t90, m=m), [0.0, 0.0, math.radians(102)], 0.5, IMU_WIN_DEG)
    assert not amb and abs(wrap(gi[2] - t90[2])) < math.radians(3), gi
    # ⑥ 조리개 — 지도에 한 장만 있고 15° 돌린 장을 맞추면, 처음 보는 쪽 벽 점을 아는 벽에 얹으려고
    #   미끄러지면 안 된다(옛 코드 0.21m, 모의 스캔에서 걸음마다 15cm씩 흘렀다)
    s6 = empty()
    integrate(s6, _room_scan([0.3, 0.2, 0.0], m=m), np.zeros((0, 3)), m)
    t6 = np.array([0.3, 0.2, math.radians(15)])
    g6 = match(s6, _room_scan(t6, m=m), t6, 0.15, 0)[0]
    assert np.hypot(*(g6[:2] - t6[:2])) < 0.06, ("조리개 미끄럼", g6)
    # ⑦ ICP — 벽 두 면 + 상자 모서리에서 알려진 변환(7cm, −12cm, 6.3°)을 3° 틀린 짐작에서 되찾는다
    rg = np.random.default_rng(0)
    dst = np.concatenate([np.c_[np.linspace(-2, 3, 400), np.full(400, 2.0)],
                          np.c_[np.full(300, 3.0), np.linspace(-1.5, 2, 300)],
                          np.c_[np.linspace(1, 1.4, 50), np.full(50, 0.3)]])
    t7 = np.array([0.07, -0.12, math.radians(6.3)])
    c7, s7 = math.cos(t7[2]), math.sin(t7[2])
    src = np.c_[c7 * (dst[:, 0] - t7[0]) + s7 * (dst[:, 1] - t7[1]),
                -s7 * (dst[:, 0] - t7[0]) + c7 * (dst[:, 1] - t7[1])] + rg.normal(0, 0.01, dst.shape)
    g7, f7, _ = icp2d(src, dst, (0, 0, math.radians(3)))
    assert np.hypot(*(g7[:2] - t7[:2])) < 0.01 and abs(wrap(g7[2] - t7[2])) < math.radians(0.3) and f7 > 0.9, g7
    # ⑤ 토마토 찾기 — 화면 가운데=정면, 오른쪽 끝=−화각/2, ±180° 경계를 넘어 묶고, 지도 벽에 쏴서 자리를 정한다
    mt = dict(m, tcam_hfov=60.0, tcam_yaw_deg=0.0)
    b = tomato_bearings({"w": 1280, "positions": [[640, 300], [1280, 300]]}, mt)
    assert abs(b[0]) < 1e-9 and abs(b[1] + math.radians(30)) < 1e-9, b
    st = empty()
    st["lo"][tuple(idx([2.0, 0.0]))] = 2.0           # 2m 앞 장애물 한 칸
    nan = float("nan")
    t = place_tomatoes(st, [(0, 0, 0.0, nan), (0, 0, 0.01, nan), (0, 0, math.radians(90), nan)])
    assert len(t) == 1 and abs(t[0][0] - 2.0) < 0.06 and t[0][2] == 2 and t[0][3] == 1, t
    t2 = place_tomatoes(st, [(0, 0, math.radians(-90), nan)] * 2)
    assert t2[0][3] == 0 and abs(t2[0][1] + TOMATO_GUESS) < 1e-9, "벽이 없으면 거리 모름"
    # 렌즈가 돌며 다른 자리에서 같은 나무를 보면(방위는 달라도) 한 자리로 묶인다
    tgt = (1.0, 1.0)
    obs = [(ox, oy, math.atan2(tgt[1] - oy, tgt[0] - ox), math.hypot(tgt[0] - ox, tgt[1] - oy))
           for ox, oy in ((0.36, 0.0), (0.25, 0.25), (0.0, 0.36))]
    t3 = place_tomatoes(st, obs)
    assert len(t3) == 1 and t3[0][2] == 3 and math.hypot(t3[0][0] - 1, t3[0][1] - 1) < 1e-6, t3
    t4 = place_tomatoes(st, obs + [(0, 0, 0.0, nan)] * 2)            # 못 잰 둘은 벽(2m)으로 가지 않는다
    assert len(t4) == 1 and t4[0][2] == 3, t4
    assert abs(clearance(st["lo"], [0.0, 0.0]) - 1.0) < 1e-9 and abs(clearance(st["lo"], [1.7, 0.0]) - 0.3) < 0.06
    # ⑥ 근접 정지 — 새 장이 거의 다 무효면 막고, 낡은 장(발행기 멈춤)으로는 막지 않는다
    import tempfile
    from tomato_picker.config import DEPTH_CAMERAS
    keep, f = DEPTH_CAMERAS["astra"]["meta"], os.path.join(tempfile.mkdtemp(), "astra_meta.json")
    DEPTH_CAMERAS["astra"]["meta"] = f
    try:
        for v, age, want in ((0.0, 0, True), (0.5, 0, False), (0.5, 10, True)):   # 낡은 장 = 발행기 재시작 중 = 막힘
            json.dump({"valid_frac": v, "ts": time.time() - age}, open(f, "w"))
            assert astra_blocked(0) == want, (v, age, want)
    finally:
        DEPTH_CAMERAS["astra"]["meta"] = keep
    # ⑦ 토마토 앞 목표 — 1.2m 떨어진 토마토면 0.5m 가서 0.7m 앞에, 이미 가까우면 제자리
    sa = empty(); sa["tomato"] = np.array([[1.2, 0.0, 3, 1.0]])
    assert np.allclose(approach_goal(sa), (0.5, 0.0))
    sa["pose"] = np.array([0.8, 0.0, 0.0])
    assert np.allclose(approach_goal(sa), (0.8, 0.0))
    # ⑧ 방위 띠 유효 비율 — 가운데가 비면(가까운 물체) 0, 차 있으면 1, 거울상이면 방위가 좌우 뒤집힌다
    mt8 = {"intrinsics": {"fx": 570.0, "fy": 570.0, "ppx": 319.5, "ppy": 239.5},
           "depth_scale_mm": 1.0, "min_mm": 600.0, "max_mm": 4000.0, "width": 640}
    dp = np.full((480, 640), 2000, np.uint16)
    dp[:, 300:340] = 0                                  # 정면이 가까워서 0
    assert slice_valid(dp, mt8, dict(m, yaw_deg=0.0), 0.0) < 0.1
    assert slice_valid(dp, mt8, dict(m, yaw_deg=0.0), math.radians(20)) > 0.9
    dp2 = np.full((480, 640), 2000, np.uint16); dp2[:, :100] = 0     # 화면 왼쪽 끝이 빔
    assert slice_valid(dp2, mt8, dict(m, yaw_deg=0.0), math.radians(25)) < 0.1      # 거울 아님: 왼쪽 = 차체 왼쪽(+)
    assert slice_valid(dp2, dict(mt8, mirrored=True), dict(m, yaw_deg=0.0), math.radians(-25)) < 0.1
    # ⑨ 토마토 높이 줄만 — 나머지(바닥·흰 화분 0, 4m 넘는 배경)가 비어도 토마토 높이(위 10°) 줄이 차 있으면 잰다(10/24)
    m9 = dict(m, yaw_deg=0.0, tilt_deg=0.0)
    e9 = tomato_elevations({"w": 1280, "h": 720, "positions": [[640, 360 - 985.3 * math.tan(math.radians(10))]]}, m9)[0]
    assert abs(e9 - math.radians(10)) < 1e-3, e9
    dp9 = np.zeros((480, 640), np.uint16); dp9[110:170] = 1130      # 토마토 줄(행 ≈139)만 차 있다
    assert slice_valid(dp9, mt8, m9, 0.0) < SLICE_MIN <= slice_valid(dp9, mt8, m9, 0.0, e=e9)
    assert slice_valid(dp9, mt8, m9, 0.0, e=math.radians(-15)) == 0.0                 # 바닥 줄은 비었다
    print("space_map selftest ✅ 맞춤 오차 %.3fm %.1f° · 점수 %.2f · 길 %d점" %
          (np.hypot(*(got[:2] - true[:2])), math.degrees(abs(wrap(got[2] - true[2]))), q, len(path)))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["snap", "scan", "go", "step", "wrist", "find", "face", "approach", "near", "clear", "selftest", "replay", "selfcal"])
    ap.add_argument("file", nargs="*", help="replay·selfcal: ~/space_scans/scan-*.npz")
    ap.add_argument("--write", action="store_true", help="selfcal: 찾은 장착값을 ~/space_mount.json에 쓴다")
    ap.add_argument("--steps", type=int, default=16)
    ap.add_argument("--w", type=int, default=150, help="회전 넛지 크기(+=왼쪽)")
    ap.add_argument("--wsecs", type=float, default=0.8, help="회전 넛지 시간(IMU 있으면 최대 ×2.5까지 --deg를 채운다)")
    ap.add_argument("--deg", type=float, default=20.0, help="IMU 있을 때 회전 넛지 한 번의 목표(도)")
    ap.add_argument("--hand", action="store_true", help="scan: 바퀴 대신 사람이 손으로 돌린다(IMU 필요, 멈출 때마다 한 장)")
    ap.add_argument("--vx", type=int, default=200, help="전진 넛지 크기 — 150·1.2초는 0, 200·1.0초는 7~12cm(10/23 실기, 케이스 얹은 차체)")
    ap.add_argument("--vsecs", type=float, default=1.0)   # 0.6은 슬루 램프가 못 올라가 0(10/22), 0.8도 150에선 0(10/23)
    ap.add_argument("--vy", type=int, default=0, help="step: 옆걸음(+=왼쪽)")
    ap.add_argument("--secs", type=float, default=0.8, help="step: 걸음 시간")
    ap.add_argument("--x", type=float)
    ap.add_argument("--y", type=float)
    ap.add_argument("--turn", type=float, help="snap: 손으로 돌린 대략 각도(도, +=왼쪽)")
    ap.add_argument("--tol", type=float, default=0.2)
    ap.add_argument("--max-steps", type=int, default=60)
    ap.add_argument("--near-mm", type=float, default=NEAR_MM, help="near: 손목 D405에서 토마토까지 이 거리(mm)에 선다")
    a = ap.parse_args()
    if a.mode == "selftest":
        return selftest()
    if a.mode == "clear":
        for f in (MAP,):
            if os.path.exists(f):
                os.remove(f)
        print("지도를 지웠다 — 다음 장의 자리가 원점(0,0,0°)이 된다")
        return 0
    m = mount()
    print("장착", m, "(바꾸려면 %s)" % MOUNT_FILE)
    if a.mode == "replay":
        return replay_cmd(a.file[0], m)
    if a.mode == "selfcal":
        return selfcal_cmd(a.file, a.write)
    if a.mode == "snap":
        s = load()
        # 사람이 손으로 돌리거나 옮겨 놓고 누르는 버튼이다 — 자리는 ±0.5m 찾는다.
        # 돌린 각도(--turn)를 주면 그 둘레 ±45°, 안 주면 전 방향 — 전 방향은 곧은 벽만
        # 보이면 **모호로 거절**된다(match). 그래도 **지도에 있는 쪽을 봐야** 찾는다.
        prior = s["pose"] + [0, 0, math.radians(a.turn or 0)]
        localize(s, m, prior, 0.5, 45 if a.turn is not None else 180, time.time())
        return 0
    if a.mode == "scan":
        return scan_cmd(a, m)
    if a.mode == "step":
        if not (a.vx or a.vy or a.w):
            print("step 은 --vx/--vy/--w 중 하나가 0이 아니어야 한다")
            return 2
        return step_cmd(a, m)
    if a.mode == "wrist":
        return wrist_cmd(m)
    if a.mode == "find":
        return find_cmd(a, m)
    if a.mode == "face":
        return face_cmd(a, m)
    if a.mode == "approach":
        return approach_cmd(a, m)
    if a.mode == "near":
        return near_cmd(a, m)
    if a.x is None or a.y is None:
        print("go 는 --x --y (지도 좌표, m)가 있어야 한다")
        return 2
    return go_cmd(a, m)


if __name__ == "__main__":
    raise SystemExit(main())
