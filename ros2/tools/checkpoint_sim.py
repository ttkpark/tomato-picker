#!/usr/bin/env python3
"""체크포인트 1·2·3차(docs/체크포인트-2026-11.md)를 **가상 3D 공간에서** 규약대로 돌려 판정하고, 사람이 돌려 볼 수 있는
3D 재생 페이지(docs/checkpoint_sim.html — 브라우저로 열면 된다, 서버 없음)를 만든다. PC에서, 장치 없이.

    PYTHONIOENCODING=utf-8 python ros2/tools/checkpoint_sim.py              # 셋 다 (3차 10회 포함, 몇 분)
    python ros2/tools/checkpoint_sim.py 1 3 --n 5 --seed 1                  # 1차·3차만 5회씩
    python ros2/tools/checkpoint_sim.py 2 --yaw imu                          # 2차, 요를 IMU로

무엇을 흉내 내나
  1차  모형 줄기 잡기 20회 — 팔 기구학(kinematics.py 실측 링크)·주사위 옮김(좌우 ±5cm·앞뒤 ±3cm·회전 ±20°)·
       손목 카메라 잡음(가는 줄기 깊이 σ15mm·가로 3·세로 8)·손-눈 표 편차 σ5·처짐 — 레시피는 11/02 것 그대로
       (몸 돌리기 pan → 시작 맞춤 → 8mm 곧장 전진 → 깊이 75에서 닫기, 첫 깊이+30mm 상한). 판정 = 규약 4종.
  2차  E2(1m 사각형 3바퀴, 엔코더 위치 제어, 지도 없이) — 메카넘 바퀴 4개·엔코더 CPR 양자화·바퀴 미끄럼·반지름/윤거 틀림.
       E1·E3·E4·E5는 기판이 있어야 재는 것이라 여기 없다.
  3차  방 안 임의 자리 1그루 — space_map_sim의 가짜 방(Astra 광선 추적)에 화분·줄기·열매를 세우고 **실기 코드 그대로**
       scan→find→approach→near→reach를 돌린 뒤 1차의 잡기로 잇는다. 사람 개입 0, 5분.
못 보는 것: 서보 토크·전원 브라운아웃·YOLO 오검출·조명·케이블. 숫자(잡음 σ)는 전부 위쪽 상수 — 실측이 바뀌면 거기만 고친다.
시계는 가짜(sleep이 시간을 민다)라 실기 1시간치가 몇 분에 끝난다.
"""
import argparse
import datetime
import json
import math
import os
import sys
import tempfile
import time
import types

import numpy as np


class _Clock:                      # 가짜 시계 — 실기 코드의 sleep·monotonic·time을 그대로 두되 기다리지 않는다
    t = 1.7e9


def _sleep(s):
    _Clock.t += max(0.0, float(s))


time.sleep, time.monotonic, time.time = _sleep, (lambda: _Clock.t), (lambda: _Clock.t)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))
ap = argparse.ArgumentParser()
ap.add_argument("which", nargs="*", default=["1", "2", "3"], help="1 2 3 가운데")
ap.add_argument("--n", type=int, default=0, help="회수(기본 = 규약: 1차 20·2차 10·3차 10)")
ap.add_argument("--only", type=int, nargs="*", default=[], help="이 회(1부터)만 돌린다 — 회마다 씨앗이 따로라 한 회만 다시 볼 수 있다")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--yaw", choices=("enc", "imu", "both"), default="both", help="2차 요 출처(기본 둘 다 돌려 나란히 보인다)")
ap.add_argument("--html-only", action="store_true", help="지난 결과(임시 JSON)로 페이지만 다시 쓴다")
ap.add_argument("--out", default=os.path.join(HERE, "..", "..", "docs", "checkpoint_sim.html"))
A = ap.parse_args()
sys.argv = [sys.argv[0], "imu", "0"]          # space_map_sim은 argv로 IMU 유무를 정한다
import space_map as sm                          # noqa: E402
import space_map_sim as world                   # noqa: E402  가짜 방·차체·IMU·Astra
from tomato_picker.hardware import kinematics as kin  # noqa: E402

# ── 팔·잡기 모델 (1차·3차 공통) ─────────────────────────────────────────
GEOM = kin.ArmGeometry()
ARM_X = sm.MOUNT["arm_x"]          # 차체 회전 중심 → pan 축(앞, m)
ARM_Z = 0.30                       # 바닥 → 팔 base(케이스 위 회전판, m) ⚠ 추정 — 실측해 덮어라
START_TCP = kin.ToolPose(x=170.0, y=0.0, z=170.0, pitch=-5.0)   # 잡기 시작 자세(줄기 110~150mm 앞, 다 뻗으면 391)
CAM_BACK = 75.0                    # 손목 D405는 TCP에서 접근축 뒤로 이만큼(손끝 76~78mm, 10/30)
STOP_Z = 75.0                      # 카메라가 본 줄기 깊이가 이만큼이면 닫는다(무는거리 75, 11/02)
STEP_MM = 8.0                      # 곧장 전진 한 걸음
GAP_HALF = 14.0                    # 닫을 때 줄기가 집게 축에서 가로로 이 안이면 문다(벌린 폭 ≈ 36mm, 줄기 10)
DEPTH_WIN = (-12.0, 18.0)          # 줄기가 TCP 기준 접근축으로 이 안이면 손끝 패드에 든다
BAND_HALF = 20.0                   # 노란 띠 세로 ±이만큼 — 벗어나면 꼭지·고리(엉뚱한 곳)
PUSH_MM = 40.0                     # 이보다 넘치면 지지대를 민다(충돌 → 즉시 중단)
NOISE = {"depth": 15.0, "lat": 3.0, "up": 8.0,   # σ mm: 가는 줄기 깊이(걸음마다 ±30 튐, 11/01)·시작 맞춤 8px≈3mm·세로 맞춤 병목(11/02)
         "bias": 5.0, "sag": 3.0}                # 손-눈/by_z 표 편차(회마다)·토크 처짐
T_SETUP, T_STEP, T_LIMIT = 10.0, 1.8, 60.0       # 초: 클릭·맞춤 / 걸음당 / 규약 상한(실측 24~48초 → 10+16걸음×1.8)
FRUIT_R, BAND_UP = 0.035, 0.02     # 열매 반지름 · 띠 중심은 열매 꼭대기에서 이만큼 위(m)


def grasp(rng, stem):
    """줄기 띠 중심(팔 base 좌표 mm: x 앞 y 왼 z 위) → 판정. 레시피 = 11/02 stem_grasp(--aim click --trust-first-z)."""
    fr, t = [], T_SETUP
    out = {"frames": fr, "stem": [round(float(v), 1) for v in stem]}

    def rec(j, grip, note):
        fr.append({"t": round(t, 1), "j": [round(float(j[k]), 2) for k in kin.JOINTS], "grip": grip, "note": note})

    def verdict(cls, e, grip_ok=False):
        out.update(ok=grip_ok, cls=cls, secs=round(t, 1), err=[round(float(v), 1) for v in e],
                   grip=round(float(rng.uniform(6.2, 6.9) if grip_ok else rng.uniform(5.0, 5.4)), 1))
        return out

    j = kin.inverse(START_TCP)
    rec(j, 0, "시작 자세")
    bias = rng.normal(0, NOISE["bias"], 3)
    meas = stem + bias + rng.normal(0, [NOISE["depth"], NOISE["lat"], NOISE["up"]])   # 클릭한 줄기(카메라 축 ≈ 접근축)
    pan = math.atan2(meas[1], meas[0])                                               # 몸 돌리기(grasp_repeat)
    a = np.array([math.cos(pan), math.sin(pan)])
    along0, lat0 = float(stem[:2] @ a), float(stem[0] * -a[1] + stem[1] * a[0])      # 참 줄기(접근축 좌표)
    e_lat = bias[1] + rng.normal(0, NOISE["lat"])                                    # 시작 맞춤 뒤 남는 가로 오차(집게−줄기)
    e_up = bias[2] + rng.normal(0, NOISE["up"]) - rng.normal(NOISE["sag"], 1.0)      # 세로 오차(맞춤 편차 − 처짐)
    tcp_z = stem[2] + e_up                                                           # 집게가 실제로 선 높이
    r = START_TCP.r
    try:
        j = kin.inverse(kin.ToolPose(x=r * a[0], y=r * a[1], z=tcp_z, pitch=START_TCP.pitch))
    except kin.Unreachable:
        return verdict("빈손", [along0 - r, e_lat, e_up])
    rec(j, 0, "몸 돌리기 %.0f° · 시작 맞춤" % math.degrees(pan))
    z1 = along0 - (r - CAM_BACK) + bias[0] + rng.normal(0, NOISE["depth"])         # 첫 깊이(카메라→줄기)
    need = max(0.0, z1 - STOP_Z)                  # --trust-first-z: 첫 깊이로 정한 거리만큼 곧장 간다(걸음마다 다시 재지 않는다 —
    gone = 0.0                                    # 가는 줄기 깊이는 걸음마다 ±30 튀어 그걸 믿으면 멈출 자리가 널뛴다, 11/01)
    while gone < need:
        t += T_STEP
        if t > T_LIMIT:
            return verdict("시간초과", [along0 - (r + gone), e_lat, e_up])
        gone = min(need, gone + STEP_MM)
        try:
            j = kin.inverse(kin.ToolPose(x=(r + gone) * a[0], y=(r + gone) * a[1], z=tcp_z, pitch=START_TCP.pitch))
        except kin.Unreachable:
            return verdict("빈손", [along0 - (r + gone), e_lat, e_up])
        rec(j, 0, "전진 %.0f/%.0fmm" % (gone, need))
    e_along = (r + gone) - along0                                                    # + = 줄기 지나침
    t += 1.5
    rec(j, 1, "닫기")
    if t > T_LIMIT:
        return verdict("시간초과", [e_along, e_lat, e_up])
    if e_along > PUSH_MM:
        return verdict("충돌", [e_along, e_lat, e_up])
    ok = DEPTH_WIN[0] <= e_along <= DEPTH_WIN[1] and abs(e_lat) <= GAP_HALF and abs(e_up) <= BAND_HALF
    if ok:
        return verdict("성공", [e_along, e_lat, e_up], True)
    return verdict("엉뚱한 곳" if abs(e_up) > BAND_HALF and abs(e_lat) <= GAP_HALF else "빈손", [e_along, e_lat, e_up])


def stem_in_arm(robot, stem_w):
    """세계 좌표 줄기(m) → 팔 base 좌표(mm). robot = (x, y, θ)."""
    c, s_ = math.cos(robot[2]), math.sin(robot[2])
    d = np.array(stem_w[:2]) - robot[:2]
    x, y = c * d[0] + s_ * d[1], -s_ * d[0] + c * d[1]
    return np.array([(x - ARM_X) * 1000, y * 1000, (stem_w[2] - ARM_Z) * 1000])


# ── 1차: 바로 앞 모형 줄기 20회 ─────────────────────────────────────────
def cp1(n, seed):
    rng = np.random.default_rng(seed)
    fruit_z = 0.42
    band_z = fruit_z + FRUIT_R + BAND_UP
    trials = []
    for i in range(n):
        lat, fa, yaw = rng.uniform(-50, 50), rng.uniform(-30, 30), rng.uniform(-20, 20)
        nom = np.array([ARM_X + (START_TCP.x + 130 + fa) / 1000, lat / 1000])    # 차체 좌표(m), 회전 전
        c, s_ = math.cos(math.radians(-yaw)), math.sin(math.radians(-yaw))      # 로봇이 +yaw 돌았다 = 무대가 −yaw
        stem_w = [c * nom[0] - s_ * nom[1], s_ * nom[0] + c * nom[1], band_z]
        g = grasp(rng, stem_in_arm(np.zeros(3), stem_w))
        g.update(i=i + 1, move=[round(lat / 10, 1), round(fa / 10, 1), round(yaw, 0)],
                 plant=[round(v, 3) for v in stem_w[:2]] + [fruit_z])
        trials.append(g)
        print("[1차 %2d] 옮김 좌우 %+.1f 앞뒤 %+.1f 회전 %+.0f° → %s %s %.0f초 집게 %.1f (오차 앞 %+.0f 옆 %+.0f 위 %+.0f)" % (
            i + 1, lat / 10, fa / 10, yaw, "✅" if g["ok"] else "❌", g["cls"], g["secs"], g["grip"], *g["err"]))
    ok = sum(t["ok"] for t in trials)
    return {"kind": 1, "name": "1차 — 바로 앞 모형 줄기 잡기", "due": "11/30", "rule": "20회 중 ≥10회", "n": n, "ok": ok,
            "passed": ok >= 10 * n / 20.0, "trials": trials, "robot": [0, 0, 0]}


# ── 2차: E2 엔코더 위치 제어 사각형 ────────────────────────────────────
WHEEL_R, LXY, CPR = 0.040, 0.30, 1320          # ⚠ 추정: 바퀴 반지름 m · lx+ly m · 4체배 CPR — 보드가 오면 N i cpr dia로 실측
ODO_ERR = {"r": 0.005, "lxy": 0.01, "slip": 0.02, "creep": 0.01}   # 반지름 0.5%·윤거 1% 틀림, 바퀴 미끄럼 σ2%, 옆 기어감 1%
IMU_ERR = {"scale": 0.003, "bias_dps": 0.05}  # 한 바퀴 자기 보정 뒤 자이로(space_map scan이 스케일을 고친다)


def cp2(n, seed, yaw_src):
    rng = np.random.default_rng(seed + 100)
    runs = []
    for i in range(n):
        r_true, lxy_true = WHEEL_R * (1 + rng.normal(0, ODO_ERR["r"])), LXY * (1 + rng.normal(0, ODO_ERR["lxy"]))
        imu_scale, imu_bias = 1 + rng.normal(0, IMU_ERR["scale"]), math.radians(rng.normal(0, IMU_ERR["bias_dps"]))
        truth, odom, enc, cnt, imu = np.zeros(3), np.zeros(3), np.zeros(4), np.zeros(4, np.int64), 0.0
        trace, slip, dt, t, k = [], np.zeros(4), 0.02, 0.0, 0
        wps = [(1, 0), (1, 1), (0, 1), (0, 0)] * 3 + [None]      # 마지막 None = 출발 방향(0°)으로 되돌기
        M = np.array([[1, -1, -LXY], [1, 1, LXY], [1, 1, -LXY], [1, -1, LXY]]) / WHEEL_R     # vx vy w → 바퀴 rad/s
        Mt = np.array([[1, -1, -lxy_true], [1, 1, lxy_true], [1, 1, -lxy_true], [1, -1, lxy_true]]) / r_true
        Fk = np.linalg.pinv(M)
        Ft = np.linalg.pinv(Mt)
        for wp in wps:
            gx, gy = wp or (0.0, 0.0)
            head = math.atan2(gy - odom[1], gx - odom[0]) if wp else 0.0
            phase = "turn"
            while t < 600:
                if k % 25 == 0:
                    slip = rng.normal(0, ODO_ERR["slip"], 4)
                if phase == "turn":
                    e = sm.wrap(head - odom[2])
                    if abs(e) < math.radians(1):
                        if wp is None:
                            break
                        phase = "drive"
                        continue
                    cmd = np.array([0, 0, float(np.clip(2.5 * e, -1.0, 1.0))])
                    cmd[2] = math.copysign(max(0.2, abs(cmd[2])), cmd[2])
                else:
                    d = np.array([gx, gy]) - odom[:2]
                    if np.hypot(*d) < 0.02:
                        break
                    e = sm.wrap(math.atan2(d[1], d[0]) - odom[2])
                    cmd = np.array([min(0.2, max(0.05, 1.0 * np.hypot(*d))), 0, float(np.clip(2.0 * e, -0.5, 0.5))])
                w_cmd = M @ cmd                                                        # 바퀴는 PID로 지령대로 돈다
                ground = w_cmd * (1 - slip)                                            # 땅에는 미끄럼만큼 덜 전해진다
                v = Ft @ ground
                v[1] += rng.normal(0, ODO_ERR["creep"]) * abs(v[0])                   # 롤러가 옆으로 기어간다
                c, s_ = math.cos(truth[2]), math.sin(truth[2])
                truth += [(c * v[0] - s_ * v[1]) * dt, (s_ * v[0] + c * v[1]) * dt, v[2] * dt]
                imu += (v[2] * imu_scale + imu_bias) * dt
                enc += w_cmd * dt
                new = np.floor(enc * CPR / (2 * math.pi)).astype(np.int64)
                vo = Fk @ ((new - cnt) * 2 * math.pi / CPR / dt)
                cnt = new
                co, so = math.cos(odom[2]), math.sin(odom[2])
                odom += [(co * vo[0] - so * vo[1]) * dt, (so * vo[0] + co * vo[1]) * dt, vo[2] * dt]
                if yaw_src == "imu":
                    odom[2] = imu
                t += dt
                k += 1
                if k % 5 == 0:
                    trace.append([round(t, 2)] + [round(float(x), 4) for x in truth] + [round(float(x), 4) for x in odom])
        pe, ae = float(np.hypot(*truth[:2])), abs(math.degrees(sm.wrap(truth[2])))
        ok = pe <= 0.10 and ae <= 3.0
        runs.append({"i": i + 1, "ok": ok, "pos_err": round(pe, 3), "ang_err": round(ae, 2), "secs": round(t, 0),
                     "trace": trace, "cls": "성공" if ok else ("위치 %.0fcm" % (pe * 100) if pe > 0.10 else "각도 %.1f°" % ae)})
        print("[2차 %2d] 3바퀴 %.0f초 → 위치 오차 %.1fcm · 각도 %.1f° %s" % (i + 1, t, pe * 100, ae, "✅" if ok else "❌"))
    ok = sum(r["ok"] for r in runs)
    return {"kind": 2, "tab": "2차·요=%s" % ("IMU" if yaw_src == "imu" else "엔코더"),
            "name": "2차 — E2 1m 사각형 3바퀴(엔코더 위치 제어, 요=%s)" % ("IMU" if yaw_src == "imu" else "엔코더"),
            "due": "12/30", "rule": "위치 ≤10cm · 각도 ≤3° (지도 보정 없이)", "n": n, "ok": ok, "passed": ok == n,
            "trials": runs, "yaw": yaw_src}


# ── 3차: 방 안 임의 자리 1그루 찾아가서 잡기 ──────────────────────────
D405 = {"fx": 438.0, "ppx": 424.0, "w": 848, "min_mm": 70.0, "max_mm": 900.0}   # 손목 D405(depth_cam MIN/MAX_MM)
TCAM = {"w": 1280, "h": 720, "hfov": sm.MOUNT["tcam_hfov"], "detect_p": 0.85, "px": 5.0}   # tomato-vision(Astra 컬러)
FRUIT_Z = 0.45                   # 열매 중심 높이(m) — 규약 30~50cm
POT = (0.20, 0.15)               # 화분 한 변·높이(m)
STEM_MAX_PAN = 45.0              # 잡기 시작 자세에서 줄기가 이보다 옆이면 접근 실패
STEM_ALONG = (60.0, 190.0)       # 시작 TCP에서 줄기까지(mm) 이 밖이면 접근 실패(팔이 못 갚는다 — 사거리)
T3_LIMIT = 300.0


def seg_hits_box(p, q, B):
    """선분 p→q(2D)가 상자 B(x0 x1 y0 y1 ...)를 지나나(가림 판정)."""
    d = q - p
    t0, t1 = 0.0, 1.0
    for k in range(2):
        if abs(d[k]) < 1e-9:
            if not (B[2 * k] <= p[k] <= B[2 * k + 1]):
                return False
            continue
        a, b = (B[2 * k] - p[k]) / d[k], (B[2 * k + 1] - p[k]) / d[k]
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
    return t0 <= t1


def cp3(n, seed):
    m = dict(world.M)
    results = []
    for i in range(n):
        if A.only and i + 1 not in A.only:
            continue
        rng = np.random.default_rng(seed * 1000 + i)
        world.rng = rng
        sm.MAP = os.path.join(tempfile.mkdtemp(), "m.npz")
        sm.SCANS = os.path.join(os.path.dirname(sm.MAP), "scans")
        # 무대: 화분 자리·로봇 출발을 주사위로 — 둘 다 상자·벽에서 떨어지고(회전 여유 0.45), 서로 1~2m
        def free(x, y, r):
            return (world.ROOM[0] + r < x < world.ROOM[1] - r and world.ROOM[2] + r < y < world.ROOM[3] - r and
                    all(not (B[0] - r < x < B[1] + r and B[2] - r < y < B[3] + r) for B in world.BOXES[:2]))
        while True:
            px, py = rng.uniform(world.ROOM[0], world.ROOM[1]), rng.uniform(world.ROOM[2], world.ROOM[3])
            if free(px, py, 0.5):
                break
        while True:
            rx, ry, rth = rng.uniform(world.ROOM[0], world.ROOM[1]), rng.uniform(world.ROOM[2], world.ROOM[3]), rng.uniform(-math.pi, math.pi)
            if free(rx, ry, 0.6) and 1.0 <= math.hypot(px - rx, py - ry) <= 2.0:
                break
        h = POT[0] / 2
        plant_boxes = [(px - h, px + h, py - h, py + h, 0.0, POT[1]),                              # 화분
                       (px - 0.005, px + 0.005, py - 0.005, py + 0.005, POT[1], FRUIT_Z + FRUIT_R + 0.08),   # 줄기
                       (px - FRUIT_R, px + FRUIT_R, py - FRUIT_R, py + FRUIT_R, FRUIT_Z - FRUIT_R, FRUIT_Z + FRUIT_R)]  # 열매
        world.BOXES[:] = [world.BOX, world.BOX2] + plant_boxes
        world.true[:] = [rx, ry, rth]
        world.start[:] = world.true
        world.T.update(w=0, vx=0, t=time.monotonic(), yaw_imu=0.0, th_prev=rth)
        stem_w = np.array([px, py, FRUIT_Z + FRUIT_R + BAND_UP])
        t0 = time.time()
        trace, est, events = [], [], []

        _phys = world.physics

        def physics():
            _phys()
            p = world.true
            if not trace or math.hypot(p[0] - trace[-1][1], p[1] - trace[-1][2]) > 0.01 or abs(p[2] - trace[-1][3]) > math.radians(1):
                trace.append([round(time.time() - t0, 1), round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 4)])
        world.physics = physics

        def localize(s_, *a, **k):
            ok = _loc(s_, *a, **k)
            w = sm.to_world(world.start, np.array([s_["pose"][:2]]))[0]
            est.append([round(time.time() - t0, 1), round(float(w[0]), 3), round(float(w[1]), 3),
                        round(float(sm.wrap(s_["pose"][2] + world.start[2])), 4), bool(ok)])
            return ok
        _loc = sm.localize
        sm.localize = localize

        # 앞 카메라(tomato-vision) 검출: 참 열매를 Astra 컬러 화각으로 투영 — 가림(상자)·깜빡임·화소 잡음
        def tomato_fresh(after, timeout=4.0):
            time.sleep(0.1)
            physics()
            p = world.true
            c, s_ = math.cos(p[2]), math.sin(p[2])
            lens = np.array([p[0] + m["x"] * c, p[1] + m["x"] * s_])
            d = np.array([px, py]) - lens
            b = sm.wrap(math.atan2(d[1], d[0]) - p[2])
            f = (TCAM["w"] / 2) / math.tan(math.radians(TCAM["hfov"]) / 2)
            pos = []
            if abs(b) < math.radians(TCAM["hfov"] / 2) and 0.3 < np.hypot(*d) < 4.5 and rng.random() < TCAM["detect_p"] \
                    and not any(seg_hits_box(lens, np.array([px, py]), B) for B in world.BOXES[:2]):
                cx = TCAM["w"] / 2 - f * math.tan(b) + rng.normal(0, TCAM["px"])
                cy = TCAM["h"] / 2 - f * math.tan(math.atan2(FRUIT_Z - m["h"], np.hypot(*d))) + rng.normal(0, TCAM["px"])
                pos.append([float(cx), float(cy)])
            return {"ts": time.time(), "w": TCAM["w"], "h": TCAM["h"], "positions": pos}
        sm.tomato_fresh = tomato_fresh

        def cam_pose():          # 손목 D405(대기·잡기 시작 자세 둘 다 정면, TCP 뒤 75mm) 세계 좌표
            p = world.true
            c, s_ = math.cos(p[2]), math.sin(p[2])
            ax = ARM_X + (START_TCP.x - CAM_BACK) / 1000
            return np.array([p[0] + ax * c, p[1] + ax * s_]), p[2]

        def wrist_tomato(prefer_u=None, waiting_pose=False):
            physics()
            cam, th = cam_pose()
            d = np.array([px, py]) - cam
            b = sm.wrap(math.atan2(d[1], d[0]) - th)
            dist = float(np.hypot(*d)) * 1000 - FRUIT_R * 1000          # 열매 앞면까지
            if abs(b) > math.radians(43) or dist > 1500 or dist < 30 \
                    or any(seg_hits_box(cam, np.array([px, py]), B) for B in world.BOXES[:2]):
                return None
            mm = dist + rng.normal(0, 3) if D405["min_mm"] <= dist <= D405["max_mm"] else float("nan")
            u = D405["ppx"] - D405["fx"] * math.tan(b) + rng.normal(0, 2)
            v = types.SimpleNamespace(intrinsics=lambda: types.SimpleNamespace(fx=D405["fx"], ppx=D405["ppx"]),
                                      meta=lambda: {"valid_frac": 0.9})
            return u, mm, math.degrees(math.atan2(D405["ppx"] - u, D405["fx"])), v
        sm.wrist_tomato = wrist_tomato
        sm.reach_tomato = wrist_tomato                    # 잡기 자세 표적(실기는 깊이+주황) — 모형은 같은 열매
        sm.wrist_front_mm = lambda v: float("inf")        # 화분은 손목 화면 아래 칸보다 낮다(ponytail: 안 본다)
        sm.front_hint = lambda m: None                    # 손목 첫 표적 짐작(앞 카메라 YOLO) — 모형엔 빨간 통이 없다
        sm.subprocess = types.SimpleNamespace(call=lambda *a, **k: 0)   # arm_stage(잡기 시작 자세) — 팔은 모델이다

        a = types.SimpleNamespace(steps=24, w=150, wsecs=0.8, deg=20.0, hand=False, vx=200, vsecs=1.0, tol=0.2,
                                  max_steps=60, near_mm=sm.NEAR_MM, reach_mm=sm.REACH_MM, x=0.0, y=0.0)
        cls, ok, g, stage = None, False, None, ""
        try:
            for stage, fn, bad in (("스캔", sm.scan_cmd, "못 찾음"), ("찾기", sm.find_cmd, "못 찾음"),
                                   ("앞으로", sm.approach_cmd, "접근 실패"), ("손목 근접", sm.near_cmd, "접근 실패"),
                                   ("잡기 거리", sm.reach_cmd, "접근 실패")):
                events.append([round(time.time() - t0, 1), stage])
                print("── 3차 %d · %s" % (i + 1, stage))
                rc = fn(a, m)
                if stage == "찾기" and rc == 0 and not len(sm.load()["tomato"]):
                    rc = 1
                if rc != 0:
                    cls = bad
                    break
                if time.time() - t0 > T3_LIMIT:
                    cls = "시간초과"
                    break
            else:
                events.append([round(time.time() - t0, 1), "잡기"])
                sa = stem_in_arm(world.true, stem_w)
                pan, along = math.degrees(math.atan2(sa[1], sa[0])), float(np.hypot(sa[0], sa[1])) - START_TCP.r
                if abs(pan) > STEM_MAX_PAN or not (STEM_ALONG[0] <= along <= STEM_ALONG[1]):
                    cls = "접근 실패"
                    print("  줄기가 팔 기준 옆 %.0f° · 앞 %.0fmm — 팔이 못 갚는다" % (pan, along))
                else:
                    g = grasp(rng, sa)
                    _Clock.t += g["secs"]
                    print("  잡기 %s %s · 오차 앞 %+.0f 옆 %+.0f 위 %+.0f · 줄기(팔) %.0f, %.0f, %.0f" % (
                        "✅" if g["ok"] else "❌", g["cls"], *g["err"], *sa))
                    cls, ok = g["cls"], g["ok"] and (time.time() - t0) <= T3_LIMIT
                    if not ok and g["ok"]:
                        cls = "시간초과"
        except Exception as e:                       # 실기 코드가 던지면 그 회는 실패 — 사람 개입 0
            cls = cls or ("%s 예외 %s" % (stage, type(e).__name__))
            print("  ⛔ 예외:", e)
        finally:
            world.physics = _phys
            sm.localize = _loc
        s = sm.load()
        occ = np.argwhere(s["lo"] > sm.OCC)
        occ_w = sm.to_world(world.start, (occ - sm.N // 2 + 0.5) * sm.RES) if len(occ) else np.zeros((0, 2))
        tom = sm.to_world(world.start, s["tomato"][:, :2])[0].tolist() if len(s["tomato"]) else None
        path = sm.to_world(world.start, np.asarray(s["path"])).round(3).tolist() if len(s["path"]) else []
        secs = round(time.time() - t0, 0)
        p = world.true
        res = {"i": i + 1, "ok": ok, "cls": cls, "secs": secs, "plant": [round(px, 3), round(py, 3), FRUIT_Z],
               "start": [round(rx, 3), round(ry, 3), round(rth, 4)], "end": [round(float(v), 4) for v in p],
               "trace": trace, "est": est, "events": events, "map": occ_w.round(3).tolist(), "tomato_est": tom, "path": path,
               "grasp": g, "dist_end": round(float(np.hypot(px - p[0], py - p[1])), 3)}
        results.append(res)
        print("[3차 %2d] 출발 (%.2f, %.2f, %.0f°) · 화분 (%.2f, %.2f) → %s %s %.0f초 · 끝 거리 %.2fm" % (
            i + 1, rx, ry, math.degrees(rth), px, py, "✅" if ok else "❌", cls, secs, res["dist_end"]))
    world.BOXES[:] = [world.BOX, world.BOX2]
    ok = sum(r["ok"] for r in results)
    return {"kind": 3, "name": "3차 — 방 안 임의 자리 1그루 찾아가서 잡기", "due": "1/31", "rule": "10회 중 ≥5회, 사람 개입 0, 5분",
            "n": n, "ok": ok, "passed": ok >= 5 * n / 10.0, "trials": results,
            "room": list(world.ROOM), "boxes": [list(world.BOX), list(world.BOX2)]}


# ── 페이지 ──────────────────────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="ko"><head><meta charset="utf-8"><title>체크포인트 가상 검증 — 3D 재생</title>
<style>
body{margin:0;font:14px/1.4 system-ui,sans-serif;background:#111;color:#eee;display:flex;height:100vh}
#side{width:380px;overflow:auto;padding:12px;background:#1b1b1b;border-right:1px solid #333}
#view{flex:1;position:relative}#c{width:100%;height:100%;display:block}
h1{font-size:17px;margin:0 0 8px}h2{font-size:15px;margin:14px 0 6px}
table{border-collapse:collapse;width:100%;font-size:12px}td,th{border-bottom:1px solid #333;padding:3px 4px;text-align:left}
tr.sel{background:#2a3a55}tr{cursor:pointer}.ok{color:#7f7}.bad{color:#f77}
.tab{display:inline-block;padding:5px 9px;margin:2px;border:1px solid #555;border-radius:4px;cursor:pointer}.tab.on{background:#2a3a55}
#bar{position:absolute;left:0;right:0;bottom:0;padding:8px;background:#000a;display:flex;gap:8px;align-items:center}
#bar input{flex:1}#note{position:absolute;top:8px;left:8px;background:#000a;padding:6px 8px;border-radius:4px;white-space:pre}
.v{font-size:13px;padding:6px;border-radius:4px;margin:4px 0}.v.ok{background:#153}.v.bad{background:#512}
small{color:#aaa}
.row{display:flex;gap:6px;align-items:center;margin:3px 0;font-size:12px}.row input[type=range]{flex:1}.row label{min-width:40px}.row input[type=number]{width:60px}
#wl{position:absolute;right:8px;background:#000a;padding:3px 6px;border-radius:4px;font-size:12px;display:none}
</style></head><body>
<div id="side"><h1>체크포인트 가상 검증</h1><small id="meta"></small><div id="tabs"></div><div id="sum"></div><div id="list"></div><div id="legend"></div></div>
<div id="view"><canvas id="c"></canvas><div id="note"></div><div id="wl">손목 D405가 보는 것(848×480 화각)</div>
<div id="bar"><button id="play">▶</button><input id="tl" type="range" min="0" max="1000" value="0"><span id="tt"></span></div></div>
<script id="data" type="application/json">__DATA__</script>
<script type="importmap">{"imports":{"three":"https://unpkg.com/three@0.160.0/build/three.module.js","three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const D=JSON.parse(document.getElementById('data').textContent);const G=D.geom;
const cv=document.getElementById('c'),R=new THREE.WebGLRenderer({canvas:cv,antialias:true});
const sc=new THREE.Scene();sc.background=new THREE.Color(0x111111);
const cam=new THREE.PerspectiveCamera(50,1,0.01,100);cam.position.set(2.5,-2.5,2.2);cam.up.set(0,0,1);
const ctl=new OrbitControls(cam,cv);sc.add(new THREE.AmbientLight(0xffffff,0.7));const L=new THREE.DirectionalLight(0xffffff,0.8);L.position.set(2,-3,5);sc.add(L);
sc.add(new THREE.GridHelper(10,20,0x333333,0x222222).rotateX(Math.PI/2));
const mat=(c,o)=>new THREE.MeshLambertMaterial({color:c,transparent:o!==undefined,opacity:o??1});
let root=new THREE.Group();sc.add(root);
function box(B,c,o){const g=new THREE.Mesh(new THREE.BoxGeometry(B[1]-B[0],B[3]-B[2],B[5]-B[4]),mat(c,o));g.position.set((B[0]+B[1])/2,(B[2]+B[3])/2,(B[4]+B[5])/2);return g}
function line(pts,c,w){const g=new THREE.BufferGeometry().setFromPoints(pts.map(p=>new THREE.Vector3(p[0],p[1],p[2]??0.02)));return new THREE.Line(g,new THREE.LineBasicMaterial({color:c}))}
function plant(x,y,fz){const g=new THREE.Group();const pot=new THREE.Mesh(new THREE.CylinderGeometry(0.1,0.08,0.15,16),mat(0x8b5a2b));pot.rotateX(Math.PI/2);pot.position.set(x,y,0.075);g.add(pot);
 const st=new THREE.Mesh(new THREE.CylinderGeometry(0.005,0.005,fz+0.12-0.15,8),mat(0x3a8f3a));st.rotateX(Math.PI/2);st.position.set(x,y,(fz+0.12+0.15)/2);g.add(st);
 const fr=new THREE.Mesh(new THREE.SphereGeometry(0.035,16,12),mat(0xe03030));fr.position.set(x,y,fz);g.add(fr);
 const bd=new THREE.Mesh(new THREE.CylinderGeometry(0.008,0.008,0.04,8),mat(0xf0d000));bd.rotateX(Math.PI/2);bd.position.set(x,y,fz+0.035+0.02);g.add(bd);return g}
function robot(){const g=new THREE.Group();const b=new THREE.Mesh(new THREE.BoxGeometry(0.44,0.30,0.22),mat(0x4a6fa5,0.85));b.position.z=0.15;g.add(b);
 for(const [x,y] of [[0.15,0.17],[0.15,-0.17],[-0.15,0.17],[-0.15,-0.17]]){const w=new THREE.Mesh(new THREE.CylinderGeometry(0.04,0.04,0.04,12),mat(0x222));w.position.set(x,y,0.04);g.add(w)}
 const nose=new THREE.Mesh(new THREE.ConeGeometry(0.03,0.08,8),mat(0xffcc00));nose.rotateZ(-Math.PI/2);nose.position.set(0.26,0,0.2);g.add(nose);
 const arm=new THREE.Group();arm.position.set(G.arm_x,0,G.arm_z);g.add(arm);g.arm=arm;return g}
function fk(j){const d=Math.PI/180,pan=j[0]*d,a1=j[1]*d,a2=a1+j[2]*d,a3=a2+j[3]*d;const P=[[G.d0,G.z0]];let r=G.d0,z=G.z0;
 for(const [l,a] of [[G.l1,a1],[G.l2,a2],[G.l3,a3]]){r+=l*Math.cos(a);z+=l*Math.sin(a);P.push([r,z])}
 return {pts:[[0,0,0]].concat(P.map(p=>[p[0]*Math.cos(pan)/1000,p[0]*Math.sin(pan)/1000,p[1]/1000])),pan,a3}}
function drawArm(arm,j,grip){arm.clear();const f=fk(j);for(let i=0;i<f.pts.length-1;i++){const a=new THREE.Vector3(...f.pts[i]),b=new THREE.Vector3(...f.pts[i+1]);const h=a.distanceTo(b);const m=new THREE.Mesh(new THREE.CylinderGeometry(0.014,0.014,h,8),mat(i==3?0xdddddd:0xffa030));m.position.copy(a).lerp(b,0.5);m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),b.clone().sub(a).normalize());arm.add(m)}
 const tip=new THREE.Vector3(...f.pts[4]);const gap=grip?0.006:0.018;const lat=new THREE.Vector3(-Math.sin(f.pan),Math.cos(f.pan),0);
 for(const s of [-1,1]){const fg=new THREE.Mesh(new THREE.BoxGeometry(0.03,0.006,0.03),mat(0x999999));fg.position.copy(tip).addScaledVector(lat,s*gap);fg.rotation.z=f.pan;arm.add(fg)}}
function walls(Rm){return [[Rm[0],Rm[0]+0.02,Rm[2],Rm[3],0,1.2],[Rm[1]-0.02,Rm[1],Rm[2],Rm[3],0,1.2],[Rm[0],Rm[1],Rm[2],Rm[2]+0.02,0,1.2],[Rm[0],Rm[1],Rm[3]-0.02,Rm[3],0,1.2]]}
let cur={cp:0,trial:0},playing=false,tfrac=0,frames=null;
const tabs=document.getElementById('tabs'),sum=document.getElementById('sum'),list=document.getElementById('list'),note=document.getElementById('note'),tl=document.getElementById('tl'),tt=document.getElementById('tt');
document.getElementById('meta').textContent='만든 때 '+D.when+' · seed '+D.seed+' · ros2/tools/checkpoint_sim.py';
D.cps.forEach((c,i)=>{const t=document.createElement('span');t.className='tab';t.textContent=(c.tab||c.name.split(' ')[0])+(c.passed?' ✅':' ❌');t.onclick=()=>show(i,0);tabs.appendChild(t)});
function show(ci,ti){P.on=false;bar.style.display='';wl.style.display='none';cur={cp:ci,trial:ti};[...tabs.children].forEach((t,i)=>t.classList.toggle('on',i==ci));const c=D.cps[ci],T=c.trials[ti],K=c.kind;
 sum.innerHTML=`<h2>${c.name}</h2><div class="v ${c.passed?'ok':'bad'}">${c.passed?'합격':'불합격'} — ${c.ok}/${c.n} (규약: ${c.rule}, 기한 ${c.due})</div><small>${D.notes[K-1]}</small>`;
 let h='<table><tr><th>회</th><th>결과</th>'+(K==1?'<th>옮김 좌우/앞뒤/회전</th><th>오차 앞/옆/위 mm</th>':K==2?'<th>위치 cm</th><th>각도 °</th>':'<th>출발→화분 m</th><th>끝 거리</th>')+'<th>초</th></tr>';
 c.trials.forEach((t,i)=>{h+=`<tr class="${i==ti?'sel':''}" data-i="${i}"><td>${t.i}</td><td class="${t.ok?'ok':'bad'}">${t.ok?'✅':'❌'} ${t.cls}</td>`+
  (K==1?`<td>${t.move.join(' / ')}</td><td>${t.err.join(' / ')}</td>`:K==2?`<td>${(t.pos_err*100).toFixed(1)}</td><td>${t.ang_err}</td>`:
  `<td>${Math.hypot(t.plant[0]-t.start[0],t.plant[1]-t.start[1]).toFixed(2)}</td><td>${t.dist_end}m</td>`)+`<td>${t.secs}</td></tr>`});
 list.innerHTML=h+'</table>';list.querySelectorAll('tr[data-i]').forEach(r=>r.onclick=()=>show(ci,+r.dataset.i));
 build(K,T);tfrac=0;tl.value=0;playing=true;document.getElementById('play').textContent='⏸'}
let rb,armJ,trace,est,plantPos;
// ── 🎮 직접 조작 — 같은 모형·같은 판정 창으로 사람이 몰아 본다 ──
const bar=document.getElementById('bar'),wl=document.getElementById('wl');
const P={on:false,pose:[0,0,0],j:[0,150,-145,-10,0],grip:0,plant:[1.2,0,0.45],keys:{},pl:null,room:[-2,3,-1.5,2],
 wc:new THREE.PerspectiveCamera(2*Math.atan(240/G.d405.fx)*57.2958,G.d405.w/480,0.02,5)};
const J=[['pan',-110,110],['lift',0,180],['elbow',-180,20],['wrist',-110,110],['roll',-180,180]];
const ptab=document.createElement('span');ptab.className='tab';ptab.textContent='🎮 직접 조작';ptab.onclick=showPlay;tabs.appendChild(ptab);
function setPlant(){if(P.pl)root.remove(P.pl);P.pl=plant(P.plant[0],P.plant[1],P.plant[2]);root.add(P.pl)}
function buildPlay(){sc.remove(root);root=new THREE.Group();sc.add(root);rb=robot();root.add(rb);frames=null;trace=est=null;
 const C3=D.cps.find(c=>c.kind==3);P.room=C3?C3.room:[-2,3,-1.5,2];for(const w of walls(P.room))root.add(box(w,0x666666,0.35));for(const B of (C3?C3.boxes:[]))root.add(box(B,0x8a7a60));P.pl=null;setPlant();
 document.getElementById('legend').innerHTML='<small>모형은 모의가 쓰는 것 그대로 — 화분 지름 20cm·높이 15cm, 줄기 1cm, 열매 공 r 3.5cm, 노란 띠 = 잡는 자리(열매 위 5.5cm). 상자·벽은 가짜 방(space_map_sim). 차체 0.44×0.30, 팔 base는 앞 '+G.arm_x+'m·높이 '+G.arm_z+'m.</small>'}
function showPlay(){P.on=true;cur={cp:-1,trial:0};[...tabs.children].forEach((t,i)=>t.classList.toggle('on',i==D.cps.length));playing=false;bar.style.display='none';wl.style.display='';
 const c1=D.cps.find(c=>c.kind==1);const j0=c1&&c1.trials[0].frames?c1.trials[0].frames[0].j:[0,150,-145,-10,0];P.j=j0.slice();
 sum.innerHTML='<h2>직접 조작 — 토마토 모형이 얼마나 그럴듯한지 눈으로</h2><small>화면을 한 번 클릭한 뒤 <b>W/S</b> 앞뒤 · <b>A/D</b> 게걸음 · <b>Q/E</b> 회전 · <b>Shift</b> 빠르게. 팔은 슬라이더. 오른쪽 아래 작은 화면 = 손목 D405 시점.</small>';
 let h='<h2>팔 관절(°)</h2>';J.forEach((q,i)=>h+=`<div class="row"><label>${q[0]}</label><input type="range" data-j="${i}" min="${q[1]}" max="${q[2]}" step="0.5" value="${P.j[i]}"><span id="jv${i}">${P.j[i]}</span></div>`);
 h+='<div class="row"><label>집게</label><input type="checkbox" id="grip"> 닫기</div><div class="row"><button id="pz">시작 자세</button><button id="pw">대기 자세(주행)</button><button id="pr">로봇 제자리</button></div>';
 h+=`<h2>화분(m)</h2><div class="row"><label>x</label><input type="number" id="px" step="0.05" value="${P.plant[0]}"><label>y</label><input type="number" id="py" step="0.05" value="${P.plant[1]}"><label>열매 z</label><input type="number" id="pzv" step="0.01" value="${P.plant[2]}"></div>`;
 h+='<h2>읽기</h2><pre id="ro" style="font-size:12px;white-space:pre-wrap;margin:0"></pre><small>줄기 표적까지의 앞/옆/위는 checkpoint_sim 판정 그대로 — 앞 '+G.win.depth[0]+'~'+G.win.depth[1]+' · 옆 ±'+G.win.gap+' · 위 ±'+G.win.band+'mm 안이면 닫을 때 잡힌다, 앞 '+G.win.push+' 넘으면 충돌.</small>';
 list.innerHTML=h;
 const setJ=j=>{P.j=j.slice();list.querySelectorAll('input[data-j]').forEach(s=>{s.value=P.j[+s.dataset.j];document.getElementById('jv'+s.dataset.j).textContent=s.value})};
 list.querySelectorAll('input[data-j]').forEach(s=>s.oninput=()=>{P.j[+s.dataset.j]=+s.value;document.getElementById('jv'+s.dataset.j).textContent=s.value});
 document.getElementById('grip').onchange=e=>P.grip=e.target.checked?1:0;
 document.getElementById('pz').onclick=()=>setJ(j0);document.getElementById('pw').onclick=()=>setJ([0,80,-150,90,0]);document.getElementById('pr').onclick=()=>{P.pose=[0,0,0]};
 for(const [id,k] of [['px',0],['py',1],['pzv',2]])document.getElementById(id).oninput=e=>{P.plant[k]=+e.target.value;setPlant()};
 buildPlay()}
window.addEventListener('keydown',e=>{if(e.target.tagName=='INPUT')return;P.keys[e.key.toLowerCase()]=1;P.keys.shift=e.shiftKey;if(P.on&&'wasdqe'.includes(e.key.toLowerCase()))e.preventDefault()});
window.addEventListener('keyup',e=>{P.keys[e.key.toLowerCase()]=0;P.keys.shift=e.shiftKey});
function playTick(dt){const k=P.keys,f=k.shift?2:1;const vx=((k.w?1:0)-(k.s?1:0))*0.25*f,vy=((k.a?1:0)-(k.d?1:0))*0.25*f,w=((k.q?1:0)-(k.e?1:0))*0.8*f;
 const [x,y,th]=P.pose,Rm=P.room;P.pose=[Math.min(Rm[1]-0.25,Math.max(Rm[0]+0.25,x+(vx*Math.cos(th)-vy*Math.sin(th))*dt)),Math.min(Rm[3]-0.25,Math.max(Rm[2]+0.25,y+(vx*Math.sin(th)+vy*Math.cos(th))*dt)),th+w*dt];
 rb.position.set(P.pose[0],P.pose[1],0);rb.rotation.z=P.pose[2];drawArm(rb.arm,P.j,P.grip);rb.updateMatrixWorld(true);
 const F=fk(P.j),tip=rb.arm.localToWorld(new THREE.Vector3(...F.pts[4])),p3=rb.arm.localToWorld(new THREE.Vector3(...F.pts[3]));
 const dir=tip.clone().sub(p3).normalize(),hz=new THREE.Vector3(dir.x,dir.y,0).normalize(),lat=new THREE.Vector3(-hz.y,hz.x,0);
 const d=new THREE.Vector3(P.plant[0],P.plant[1],P.plant[2]+0.055).sub(tip);const along=-d.dot(hz)*1000,side=d.dot(lat)*1000,up=d.z*1000;
 const W=G.win;const v=along>W.push?'충돌(지지대를 민다)':(along>=W.depth[0]&&along<=W.depth[1]&&Math.abs(side)<=W.gap&&Math.abs(up)<=W.band)?'닫으면 잡힌다 ✅':(Math.abs(up)>W.band&&Math.abs(side)<=W.gap)?'엉뚱한 곳(띠 밖)':'빈손';
 const wc=P.wc;wc.position.copy(tip).addScaledVector(dir,-0.075);wc.up.set(0,0,1);wc.lookAt(tip.clone().add(dir));
 const fr=new THREE.Vector3(P.plant[0],P.plant[1],P.plant[2]).sub(wc.position),mm=fr.length()*1000,b=Math.atan2(lat.dot(fr),hz.dot(fr));
 const seen=hz.dot(fr)>0&&Math.abs(b)<Math.atan(G.d405.ppx/G.d405.fx)&&mm>=G.d405.min&&mm<=G.d405.max;
 document.getElementById('ro').textContent=`차체 (${P.pose[0].toFixed(2)}, ${P.pose[1].toFixed(2)}, ${(P.pose[2]*57.3).toFixed(0)}°)\n손끝(세계) (${tip.x.toFixed(3)}, ${tip.y.toFixed(3)}, ${tip.z.toFixed(3)})\n줄기 표적까지  앞 ${along.toFixed(0)} · 옆 ${side.toFixed(0)} · 위 ${up.toFixed(0)} mm\n→ ${P.grip?v:'(집게 열림) '+v}\n손목 D405: 열매 ${mm.toFixed(0)}mm · 가로 ${(b*57.3).toFixed(1)}° · u ${(G.d405.ppx-G.d405.fx*Math.tan(b)).toFixed(0)}${seen?'':'  — 안 보임/유효 '+G.d405.min+'~'+G.d405.max+'mm 밖'}`;
 note.textContent=''}
function build(K,T){sc.remove(root);root=new THREE.Group();sc.add(root);rb=robot();root.add(rb);frames=null;trace=est=null;
 if(K==1){root.add(plant(T.plant[0],T.plant[1],T.plant[2]));frames=T.frames;rb.position.set(0,0,0);rb.rotation.z=0;
  document.getElementById('legend').innerHTML='<small>주황=팔 · 회색 판=집게 · 노란 띠=줄기 표적. 회전 ±20°는 무대를 반대로 돌려 흉내 냈다.</small>'}
 else if(K==2){root.add(line([[0,0],[1,0],[1,1],[0,1],[0,0]],0x555555));trace=T.trace.map(r=>[r[1],r[2],r[3]]);est=T.trace.map(r=>[r[4],r[5],r[6]]);
  root.add(line(trace.map(p=>[p[0],p[1],0.02]),0x40ff80));root.add(line(est.map(p=>[p[0],p[1],0.03]),0xffa040));
  document.getElementById('legend').innerHTML='<small>회색=목표 사각형 · 초록=실제 자취 · 주황=엔코더가 믿는 자취(로봇은 주황을 따라 돈다). 끝에서 둘의 차이가 E2 오차.</small>'}
 else{const C3=D.cps[cur.cp],Rm=C3.room;for(const w of walls(Rm))root.add(box(w,0x666666,0.35));
  for(const B of C3.boxes)root.add(box(B,0x8a7a60));root.add(plant(T.plant[0],T.plant[1],T.plant[2]));
  if(T.map.length){const im=new THREE.InstancedMesh(new THREE.BoxGeometry(0.045,0.045,0.02),mat(0x40c0ff,0.6),T.map.length);const o=new THREE.Object3D();T.map.forEach((p,i)=>{o.position.set(p[0],p[1],0.01);o.updateMatrix();im.setMatrixAt(i,o.matrix)});root.add(im)}
  if(T.path.length)root.add(line(T.path.map(p=>[p[0],p[1],0.05]),0xffff00));
  if(T.tomato_est){const s=new THREE.Mesh(new THREE.SphereGeometry(0.05,8,8),mat(0xff80ff,0.7));s.position.set(T.tomato_est[0],T.tomato_est[1],0.6);root.add(s)}
  trace=T.trace;est=T.est;root.add(line(trace.map(p=>[p[1],p[2],0.02]),0x40ff80));if(est.length)root.add(line(est.map(p=>[p[1],p[2],0.04]),0xffa040));
  frames=T.grasp?T.grasp.frames:null;
  document.getElementById('legend').innerHTML='<small>초록=실제 자취 · 주황=지도가 믿는 자세 · 하늘색 칸=지은 점유격자 · 노랑=계획한 길 · 분홍 공=찾은 토마토 자리. 끝에 팔이 잡는다.</small>'}
}
function tick(){const K=D.cps[cur.cp].kind,T=D.cps[cur.cp].trials[cur.trial];let s='';
 if(K==1){const n=frames.length,k=Math.min(n-1,Math.floor(tfrac*n));const f=frames[k];drawArm(rb.arm,f.j,f.grip);s=`${T.i}회 · ${f.t}s · ${f.note}\n줄기(팔 base) ${T.stem.join(', ')}mm\n${T.ok?'✅':'❌'} ${T.cls} · 집게값 ${T.grip}`;tt.textContent=f.t+'s'}
 else if(K==2){const n=trace.length,k=Math.min(n-1,Math.floor(tfrac*n));rb.position.set(trace[k][0],trace[k][1],0);rb.rotation.z=trace[k][2];
  const dx=trace[k][0]-est[k][0],dy=trace[k][1]-est[k][1];s=`${T.i}회 · ${T.trace[k][0]}s\n실제 (${trace[k][0].toFixed(2)}, ${trace[k][1].toFixed(2)}, ${(trace[k][2]*57.3).toFixed(1)}°)\n엔코더 (${est[k][0].toFixed(2)}, ${est[k][1].toFixed(2)}, ${(est[k][2]*57.3).toFixed(1)}°)\n차이 ${(Math.hypot(dx,dy)*100).toFixed(1)}cm · ${((trace[k][2]-est[k][2])*57.3).toFixed(1)}°`;tt.textContent=T.trace[k][0]+'s'}
 else{const tend=T.secs,t=tfrac*tend;let k=0;while(k<trace.length-1&&trace[k+1][0]<=t)k++;const p=trace[k]||[0,...T.start];rb.position.set(p[1],p[2],0);rb.rotation.z=p[3];
  let ev='';for(const e of T.events)if(e[0]<=t)ev=e[1];let e=null;for(const q of est)if(q[0]<=t)e=q;
  if(frames&&ev=='잡기'){const tg=T.events[T.events.length-1][0];const n=frames.length;const k2=Math.min(n-1,Math.max(0,Math.floor((t-tg)/(tend-tg+1e-6)*n)));drawArm(rb.arm,frames[k2].j,frames[k2].grip);s='\n'+frames[k2].note}else drawArm(rb.arm,[0,80,-150,90,0],0);
  s=`${T.i}회 · ${t.toFixed(0)}s · ${ev}${s}\n실제 (${p[1].toFixed(2)}, ${p[2].toFixed(2)}, ${(p[3]*57.3).toFixed(0)}°)`+(e?`\n지도 (${e[1].toFixed(2)}, ${e[2].toFixed(2)}, ${(e[3]*57.3).toFixed(0)}°) 차이 ${(Math.hypot(e[1]-p[1],e[2]-p[2])*100).toFixed(0)}cm`:'')+`\n${T.ok?'✅':'❌'} ${T.cls}`;tt.textContent=t.toFixed(0)+'s'}
 note.textContent=s}
tl.oninput=()=>{tfrac=tl.value/1000;playing=false;document.getElementById('play').textContent='▶'};
document.getElementById('play').onclick=()=>{playing=!playing;document.getElementById('play').textContent=playing?'⏸':'▶'};
let last=performance.now();
function loop(now){const dt=(now-last)/1000;last=now;if(playing){const K=D.cps[cur.cp].kind;tfrac+=dt/(K==1?8:K==2?20:30);if(tfrac>=1){tfrac=1;playing=false;document.getElementById('play').textContent='▶'}tl.value=tfrac*1000}
 const w=cv.clientWidth,h=cv.clientHeight;if(cv.width!=w||cv.height!=h){R.setSize(w,h,false);cam.aspect=w/h;cam.updateProjectionMatrix()}
 if(P.on)playTick(dt);else tick();ctl.update();R.setScissorTest(false);R.setViewport(0,0,w,h);R.render(sc,cam);
 if(P.on){const iw=Math.min(380,Math.floor(w*0.35)),ih=Math.floor(iw*480/G.d405.w);wl.style.bottom=(ih+12)+'px';R.setScissorTest(true);R.setScissor(w-iw-8,8,iw,ih);R.setViewport(w-iw-8,8,iw,ih);R.render(sc,P.wc);R.setScissorTest(false);R.setViewport(0,0,w,h)}
 requestAnimationFrame(loop)}
show(0,0);requestAnimationFrame(loop);
</script></body></html>
"""

NOTES = [
    "모델: 팔 기구학 실측 링크(kinematics.py) · 손목 D405 잡음(가는 줄기 깊이 σ15mm·가로 3·세로 8) · 손-눈 표 편차 σ5 · 처짐 3mm. "
    "못 보는 것: 서보 토크·전원·조명. 세로 맞춤이 큰 통(11/02 병목)이면 '엉뚱한 곳'으로 나온다.",
    "모델: 메카넘 4바퀴 역기구학·엔코더 CPR 1320 양자화·바퀴 미끄럼 σ2%·반지름 0.5%/윤거 1% 틀림(추정). "
    "요를 엔코더만으로 재면 윤거 1%가 1080°에 10° 넘게 쌓인다 — 통과하려면 IMU 요(--yaw imu)나 윤거 보정이 필요하다.",
    "모델: space_map_sim 가짜 방(Astra 광선 추적, 바닥 안 보임, 60cm 사각) + 화분·줄기·열매. scan→find→approach→near→reach는 "
    "실기 코드 그대로(장치만 바꿔 끼움), 잡기는 1차 모델. tomato-vision은 열매 투영(깜빡임 15%·가림)으로 흉내.",
]


GEOM_JS = {"win": {"gap": GAP_HALF, "depth": list(DEPTH_WIN), "band": BAND_HALF, "push": PUSH_MM},
           "d405": {"fx": D405["fx"], "ppx": D405["ppx"], "w": D405["w"], "min": D405["min_mm"], "max": D405["max_mm"]}}


def page(blob: str) -> str:
    d = json.loads(blob)
    d["geom"].update(GEOM_JS)          # 🎮 직접 조작 탭이 판정 창·손목 카메라 값을 쓴다
    blob = json.dumps(d, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    return HTML.replace("__DATA__", blob.replace("</", "<\\/"))


JSON_KEEP = os.path.join(tempfile.gettempdir(), "checkpoint_sim.json")   # 지난 결과 — 페이지 틀만 고칠 때 --html-only


def main():
    which = set(A.which)
    cps = []
    if A.html_only:
        with open(A.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(page(open(JSON_KEEP, encoding="utf-8").read()))
        print("→", os.path.normpath(A.out))
        return 0
    if "1" in which:
        cps.append(cp1(A.n or 20, A.seed))
    if "2" in which:
        for y in (("enc", "imu") if A.yaw == "both" else (A.yaw,)):
            cps.append(cp2(A.n or 10, A.seed, y))
    if "3" in which:
        cps.append(cp3(A.n or 10, A.seed))
    print("\n=== 가상 체크포인트 ===")
    for c in cps:
        print("%s  %d/%d  %s  (%s)" % ("✅" if c["passed"] else "❌", c["ok"], c["n"], c["name"], c["rule"]))
    data = {"when": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
            "seed": A.seed, "geom": {"d0": GEOM.d0, "z0": GEOM.z0, "l1": GEOM.l1, "l2": GEOM.l2, "l3": GEOM.l3, "arm_x": ARM_X, "arm_z": ARM_Z},
            "cps": cps, "notes": NOTES}
    blob = json.dumps(data, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    html = page(blob)
    if which == {"1", "2", "3"}:
        with open(JSON_KEEP, "w", encoding="utf-8") as f:
            f.write(blob)
        with open(A.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(html)
        print("→ %s (%.1f MB) — 브라우저로 연다" % (os.path.normpath(A.out), len(html.encode("utf-8")) / 1e6))
    else:
        out = os.path.join(tempfile.gettempdir(), "checkpoint_sim_part.html")
        with open(out, "w", encoding="utf-8", newline="\n") as f:
            f.write(html)
        print("→ %s (일부만 돌렸다 — docs 것은 셋 다 돌릴 때만 덮는다)" % out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
