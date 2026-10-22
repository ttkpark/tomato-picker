#!/usr/bin/env python3
"""space_map 제자리 스캔을 **가짜 방·가짜 차체·가짜 IMU로 끝까지** 돌려 지도 품질을 잰다 (PC, 장치 없이, ~40초).

    python ros2/tools/space_map_sim.py            # IMU, 스케일 2% 틀림 — 합격선 단언(복셀 75%·θ 5°)
    python ros2/tools/space_map_sim.py noimu 0    # IMU 없이(비교용, 단언 없음)

가짜 방 5×3.5×2.5m + 상자 둘을 Astra 내부값으로 광선 추적해 깊이를 만든다(바닥은 0 = 실측처럼 IR이 안 돌아온다,
거리² 잡음). 차체는 회전 지령 200 아래면 안 돈다(정지마찰), 돌면 35°/s에 옆으로 조금 밀린다. IMU는 스케일 2% 틀림.
`scan_cmd`·`turn`·`localize`·`match`는 **실기 코드 그대로**고 장치 함수(frame·imu_now·주행 보드)만 바꿔 끼운다.

지표: 지도 복셀이 참 표면 7.5cm 안에 든 비율(참 자세로 지으면 97%), 2D 장애물 칸이 10cm 안에 든 비율, 끝 θ 오차.
환경변수로 조건을 바꾼다: SIM_SCALE · SIM_NOISE · SIM_SLIP · SIM_W · SIM_STEPS · SIM_IMUWIN · SIM_XY/SIM_TH(맞춤 폭 강제)
· SIM_ORACLE=1(맞춤 대신 참 자세 = 지표의 천장). 2026-10-22에 조리개 미끄럼과 고리 닫기를 이걸로 찾고 고쳤다.
"""
import math
import os
import sys
import tempfile
import time
import types

import numpy as np

USE_IMU = (sys.argv[1:2] or ["imu"])[0] == "imu"
SEED = int(sys.argv[2]) if len(sys.argv) > 2 else 0
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import space_map as sm  # noqa: E402

rng = np.random.default_rng(SEED)
ROOM = (-2.0, 3.0, -1.5, 2.0, 0.0, 2.5)        # x0 x1 y0 y1 z0 z1
BOX = (1.0, 1.4, 0.3, 0.9, 0.0, 0.8)
BOX2 = (-1.2, -0.8, -1.5, -1.0, 0.0, 1.2)      # 구석 장
W, H = 640, 480
IT = {"fx": 570.3, "fy": 570.3, "ppx": 319.5, "ppy": 239.5}
M = dict(sm.MOUNT)

true = np.array([0.3, 0.2, 0.0])              # 참 자세 — 지도 원점과 다르다(지도는 첫 장 자리 = 원점)
T = {"w": 0, "vx": 0, "t": time.monotonic(), "yaw_imu": 0.0, "bias": math.radians(0.05)}
THR_V, SPEED = 140, 0.12                        # 전진: 지령 140 아래면 안 간다, 넘으면 0.12m/s
THR = 200                                       # 정지마찰 문턱(회전 지령)
RATE = math.radians(35)                         # 문턱 넘으면 35°/s
SCALE_ERR = 1.02                                # IMU 스케일 2% 틀림


def physics():
    now = time.monotonic()
    dt, T["t"] = now - T["t"], now
    if abs(T["w"]) >= THR:
        d = math.copysign(RATE, T["w"]) * dt
        true[2] += d
        # 순수 회전이 아니다 — 옆으로 밀린다(2026-09-09 실측)
        true[:2] += rng.normal(0, 0.01, 2) * dt * float(os.environ.get("SIM_SLIP", 5))
        T["yaw_imu"] += d * SCALE_ERR
    if abs(T["vx"]) >= THR_V:
        v = math.copysign(SPEED, T["vx"]) * dt
        true[0] += v * math.cos(true[2])
        true[1] += v * math.sin(true[2])
        true[2] += rng.normal(0, math.radians(2)) * dt            # 전진 중 조금 틀어진다(IMU도 본다)
        T["yaw_imu"] += (true[2] - T.get("th_prev", true[2]))
    T["th_prev"] = true[2]
    T["yaw_imu"] += T["bias"] * dt


def imu_now(max_age=1.0):
    physics()
    if not USE_IMU:
        return None
    return 1.0, T["yaw_imu"] + rng.normal(0, math.radians(0.02)), T["w"] == 0 and T["vx"] == 0


class Link:
    def set_velocity(self, vx, vy, w):
        physics()
        T["w"], T["vx"] = w, vx

    def stop(self):
        physics()
        T["w"] = T["vx"] = 0

    def close(self):
        pass


def drive(link, vx, vy, w, secs, dither=0):
    t1 = time.monotonic()
    while time.monotonic() - t1 < secs:
        link.set_velocity(vx, vy, w)
        time.sleep(0.03)
    link.stop()


def render():
    """참 자세에서 본 깊이(mm, Z거리). 바닥은 IR이 안 돌아와 0(실측과 같게)."""
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X, Y = (u - IT["ppx"]) / IT["fx"], (v - IT["ppy"]) / IT["fy"]
    c, s = math.cos(true[2]), math.sin(true[2])
    db = np.stack([np.ones_like(X), -X, -Y], -1)                      # 차체 좌표 방향(Z=1 기준)
    d = np.stack([c * db[..., 0] - s * db[..., 1], s * db[..., 0] + c * db[..., 1], db[..., 2]], -1)
    lx = float(os.environ.get("SIM_TRUE_X", M["x"]))     # 실제 렌즈 위치(코드는 M["x"]로 믿는다) — selfcal 시험용
    o = np.array([true[0] + lx * c, true[1] + lx * s, M["h"]])
    with np.errstate(divide="ignore", invalid="ignore"):
        lo = (np.array(ROOM[0::2]) - o) / d
        hi = (np.array(ROOM[1::2]) - o) / d
        t_exit = np.nanmin(np.where(d > 0, hi, np.where(d < 0, lo, np.inf)), -1)
        z_hit = o[2] + d[..., 2] * t_exit
        t = np.where(z_hit <= ROOM[4] + 1e-6, np.nan, t_exit)          # 바닥 = 안 보임
        for B in (BOX, BOX2):
            a = (np.array(B[0::2]) - o) / d
            b = (np.array(B[1::2]) - o) / d
            tn = np.nanmax(np.minimum(a, b), -1)
            tf = np.nanmin(np.maximum(a, b), -1)
            hit = (tn <= tf) & (tn > 0)
            t = np.where(hit & ~(tn > t), tn, t)
    Zmm = t * 1000.0                                                  # 방향의 앞 성분이 1이라 t = 앞 거리
    Zmm += rng.normal(0, 1, Zmm.shape) * (Zmm / 1000.0) ** 2 * float(os.environ.get("SIM_NOISE", 3))  # 거리² 잡음
    Zmm = np.where((Zmm >= 600) & (Zmm <= 4000), Zmm, 0)
    return np.nan_to_num(Zmm).astype(np.uint16)


META = {"intrinsics": IT, "depth_scale_mm": 1.0, "min_mm": 600.0, "max_mm": 4000.0, "valid_frac": 1.0}


def frame(after, timeout=40.0):
    physics()
    return render(), META


sm.imu_now = imu_now
sm.IMU_WIN_DEG = float(os.environ.get("SIM_IMUWIN", sm.IMU_WIN_DEG))
SCALE_ERR = float(os.environ.get("SIM_SCALE", SCALE_ERR))
sm.frame = frame
sys.modules["base_nudge"] = types.SimpleNamespace(open_link=lambda *a, **k: Link(), drive=drive)
sm.MAP = os.path.join(tempfile.mkdtemp(), "m.npz")
sm.SCANS = os.path.join(os.path.dirname(sm.MAP), "scans")   # 홈에 기록을 남기지 않는다


def surf_dist(p):
    """점(지도 좌표 → 참 좌표)에서 가장 가까운 참 표면까지(m)."""
    def box_out(p, B):          # 상자 바깥 거리
        dx = np.maximum(np.maximum(B[0] - p[:, 0], p[:, 0] - B[1]), 0)
        dy = np.maximum(np.maximum(B[2] - p[:, 1], p[:, 1] - B[3]), 0)
        dz = np.maximum(np.maximum(B[4] - p[:, 2], p[:, 2] - B[5]), 0)
        inside = (dx == 0) & (dy == 0) & (dz == 0)
        din = np.min(np.stack([p[:, 0] - B[0], B[1] - p[:, 0], p[:, 1] - B[2], B[3] - p[:, 1],
                               p[:, 2] - B[4], B[5] - p[:, 2]], 1), 1)
        return np.where(inside, din, np.sqrt(dx * dx + dy * dy + dz * dz))
    walls = np.min(np.stack([np.abs(p[:, 0] - ROOM[0]), np.abs(p[:, 0] - ROOM[1]),
                             np.abs(p[:, 1] - ROOM[2]), np.abs(p[:, 1] - ROOM[3]),
                             np.abs(p[:, 2] - ROOM[5])], 1), 1)
    return np.minimum(walls, np.minimum(box_out(p, BOX), box_out(p, BOX2)))


start = true.copy()
HAND = bool(os.environ.get("SIM_HAND"))
if HAND:                                  # 사람이 손으로: 0.45초 돌리고 1.2초 멈추기를 되풀이(모터 지령 없음)
    import threading

    def _hand():
        time.sleep(3.0)                   # 버튼을 누르고 나서 돌리기 시작한다
        while True:
            T["w"] = 255
            time.sleep(0.45)
            T["w"] = 0
            time.sleep(1.2)
    threading.Thread(target=_hand, daemon=True).start()
if os.environ.get("SIM_XY"):           # 맞춤 탐색 폭 강제(xy m, θ °)
    _m = sm.match
    _xy, _th = float(os.environ["SIM_XY"]), float(os.environ["SIM_TH"])
    sm.match = lambda s_, scan, prior, wx, wd: _m(s_, scan, prior, min(wx, _xy), min(wd, _th))
if os.environ.get("SIM_ORACLE"):        # 천장: 맞춤 대신 참 자세(지도 좌표)를 준다
    def oracle(s_, scan, prior, win_xy, win_deg):
        c0, s0 = math.cos(-start[2]), math.sin(-start[2])
        d = true - start
        return np.array([c0 * d[0] - s0 * d[1], s0 * d[0] + c0 * d[1], sm.wrap(d[2])]), 1.0, 99, False
    sm.match = oracle
args = types.SimpleNamespace(steps=int(os.environ.get("SIM_STEPS", 24)), w=int(os.environ.get("SIM_W", 150)), wsecs=float(os.environ.get("SIM_WSECS", 0.8)), deg=20.0, hand=HAND)
t0 = time.time()
rc = sm.scan_cmd(args, M)
s = sm.load()
# 지도 좌표 → 참 좌표: 지도 원점 = 첫 장의 참 자세
c, sn = math.cos(start[2]), math.sin(start[2])
vox = (s["vox"].astype(float) + 0.5) * sm.RES
pw = np.stack([start[0] + c * vox[:, 0] - sn * vox[:, 1], start[1] + sn * vox[:, 0] + c * vox[:, 1], vox[:, 2]], 1)
dv = surf_dist(pw)
occ = np.argwhere(s["lo"] > sm.OCC)
oc = (occ - sm.N // 2 + 0.5) * sm.RES
ow = np.stack([start[0] + c * oc[:, 0] - sn * oc[:, 1], start[1] + sn * oc[:, 0] + c * oc[:, 1], np.full(len(oc), 0.3)], 1)
do = surf_dist(ow)
est_th = s["pose"][2] + start[2]
print("\n=== %s seed %d rc=%d %.0fs ===" % ("IMU" if USE_IMU else "IMU 없음", SEED, rc, time.time() - t0))
print("참 이동 %.2fm (지도 좌표 %.2f, %.2f)" % ((float(np.hypot(*(true[:2] - start[:2]))),) +
      tuple(np.array([[c, sn], [-sn, c]]) @ (true[:2] - start[:2]))))
print("참 회전 %.0f° · 지도 θ 오차 %.1f°" % (math.degrees(true[2] - start[2]),
                                       math.degrees(sm.wrap(est_th - true[2]))))
print("복셀 %d · 표면 7.5cm 안 %.1f%% · 중앙값 %.3fm" % (len(dv), 100 * (dv < 0.075).mean(), np.median(dv)))
print("장애물 칸 %d · 표면 10cm 안 %.1f%%" % (len(do), 100 * (do < 0.10).mean()))
err = abs(math.degrees(sm.wrap(est_th - true[2])))
if os.environ.get("SIM_GO"):            # 이어서 목표로 가기 — 지도 좌표 SIM_GO="x,y"
    gx, gy = (float(v) for v in os.environ["SIM_GO"].split(","))
    ga = types.SimpleNamespace(x=gx, y=gy, w=150, wsecs=0.8, vx=150, vsecs=0.8, tol=0.2, max_steps=40, deg=20.0)
    rc2 = sm.go_cmd(ga, M)
    s = sm.load()
    tm = np.array([[c, sn], [-sn, c]]) @ (true[:2] - start[:2])        # 참 자리(지도 좌표)
    print("=== 가기 rc=%d · 참 자리 (%.2f, %.2f) · 목표까지 %.2fm · 지도 자세 오차 %.2fm %.1f°" % (
        rc2, tm[0], tm[1], float(np.hypot(tm[0] - gx, tm[1] - gy)), float(np.hypot(*(s["pose"][:2] - tm))),
        math.degrees(sm.wrap(s["pose"][2] + start[2] - true[2]))))
    if os.environ.get("SIM_SCAN2"):    # 새 자리에서 한 바퀴 — 몰래 밀어 놓아(시작 맞춤 미끄럼 흉내) register가 고치는지
        from scipy.spatial import cKDTree
        before = s["vox"].astype(int).copy()
        kick = [float(v) for v in os.environ.get("SIM_KICK", "0,0").split(",")]
        true[0] += kick[0]; true[1] += kick[1]
        sm.scan_cmd(args, M)
        s = sm.load()
        A = set(map(tuple, before))
        new = np.array([v for v in map(tuple, s["vox"].astype(int)) if v not in A])
        dn, _ = cKDTree(before).query(new)
        pw2 = (new.astype(float) + 0.5) * sm.RES
        pw2 = np.stack([start[0] + c * pw2[:, 0] - sn * pw2[:, 1], start[1] + sn * pw2[:, 0] + c * pw2[:, 1], pw2[:, 2]], 1)
        tm = np.array([[c, sn], [-sn, c]]) @ (true[:2] - start[:2])
        if os.environ.get("SIM_PLOT"):
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            bad = surf_dist(pw2) >= 0.075
            fig, ax = plt.subplots(1, 2, figsize=(14, 6))
            ax[0].scatter(pw2[~bad, 0], pw2[~bad, 1], s=1, c="g"); ax[0].scatter(pw2[bad, 0], pw2[bad, 1], s=1, c="r")
            ax[1].scatter(np.hypot(pw2[:, 0] - true[0], pw2[:, 1] - true[1]), pw2[:, 2], s=1, c=np.where(bad, "r", "g"))
            ax[0].plot(true[0], true[1], "k^"); ax[0].set_aspect("equal")
            plt.savefig(os.environ["SIM_PLOT"], dpi=60)
        print("=== 새 자리 스캔 · 새 복셀 %d · 첫 지도 8cm 안 %.1f%% · 참 표면 7.5cm 안 %.1f%% · 지도 자세 오차 %.2fm" % (
            len(new), 100 * (dn <= 1.5).mean(), 100 * (surf_dist(pw2) < 0.075).mean(), float(np.hypot(*(s["pose"][:2] - tm)))))
if USE_IMU and not os.environ.get("SIM_SCALE") and not os.environ.get("SIM_XY"):
    assert rc == 0 and (dv < 0.075).mean() >= 0.75 and err <= 5.0, ("지도 품질 합격선 미달", rc, (dv < 0.075).mean(), err)
    print("space_map_sim ✅ 합격선(복셀 75%·θ 5°)")
