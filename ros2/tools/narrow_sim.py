#!/usr/bin/env python3
"""좁은 방 따라가기(체크포인트 0차) — 실기 코드(follow_trials 순서) 그대로, 벽에 부딪히는 가짜 방에서. PC, 장치 없이.

    PYTHONIOENCODING=utf-8 python ros2/tools/narrow_sim.py --room 3x3 --route look --sets 6
    python ros2/tools/narrow_sim.py --selftest

checkpoint_sim의 3차 방에 없던 것 넷을 넣었다:
  ① 충돌 — 차체 사각형이 벽·가구·화분에 닿으면 안 움직인다(앞바퀴 엔코더는 stall). 그 전엔 벽을 뚫고 갔다.
  ② Astra 꺼짐 — 60cm 안을 보면 투광기가 꺼져 **새 장이 안 오고**, 발행기가 40초 뒤 다시 연다(10/23 실측).
  ③ 손목 D405 사진 — 둘러보기(look_sweep·look_fruit·look_axis)를 합성 사진·깊이로 실기 코드 그대로 돌린다.
  ④ 회차 누적 — follow_trials처럼 3회를 이어 돈다(앞 회가 실패하면 그 자리에서 다음 회).
못 보는 것: 배터리 처짐·보드 리셋·조명·빨간 의자 같은 오검출·팔이 가구에 걸림. 방 크기·장착 치수는 아래 상수.
"""
import argparse
import contextlib
import io
import json
import math
import os
import sys
import tempfile
import time
import types

ap = argparse.ArgumentParser()
ap.add_argument("--room", default="3x3", help="WxD m, 가구 넷은 방 크기와 상관없이 같다")
ap.add_argument("--route", choices=("look", "scan"), default="look")
ap.add_argument("--sets", type=int, default=6, help="3회 묶음 수")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--start", choices=("near", "far"), default="near",
                help="near = follow_trials(근접 끝자리에서 물러나 돌기, 앞 회가 실패하면 그 자리) · far = 회마다 토마토에서 1~2.2m 임의 자리·방향(0차 '큰 방' 규칙)")
ap.add_argument("--verbose", action="store_true")
ap.add_argument("--selftest", action="store_true")
ARGS = ap.parse_args()
sys.argv = [sys.argv[0]]
import numpy as np                              # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import checkpoint_sim as c                      # noqa: E402  가짜 시계·가짜 방·차체를 가져온다
sm, world = c.sm, c.world
import follow_trials as ft                      # noqa: E402  실기 순서(STARTS)
import tomato_trials as tt                      # noqa: E402
from tomato_picker.hardware import depth_camera  # noqa: E402

# 치수(m): 차체 = 회전 중심 기준 사각형, Astra 코까지 포함(mount.x 0.345 = 10/23 자기 보정)
FOOT = (-0.16, 0.37, -0.16, 0.16)
ASTRA_MIN_MM, ASTRA_REOPEN_S = 600.0, float(os.environ.get("NARROW_REOPEN", 40.0))
ASTRA_NEAR_FRAC = float(os.environ.get("NARROW_FRAC", 0.15))   # 화면의 이만큼이 60cm 안이면 투광기가 꺼진다 ⚠ 실측 전 — 벽은 확실히 꺼지고(10/23) 화분은 안 꺼진 적이 있다(10/24)
LENS_R = sm.LOOK_CAM_R_M                 # 대기 자세 렌즈~어깨 축
PAN_OFF = 0.08                           # 어깨 축이 회전판 축에서 벗어난 길이 — 같은 열매가 650mm에서 13° 벌어진다(11/06) ⚠ 그 값으로 맞춘 추정
WRIST_H = c.FRUIT_Z + 0.206              # 대기 자세 렌즈 높이: 토마토가 렌즈보다 ≈206mm 아래(TOMATO_BELOW_MM 실측)
W, H = 848, 480
IT = types.SimpleNamespace(fx=438.0, fy=438.0, ppx=424.0, ppy=240.0)
LIMIT_S = ft.LIMIT_S
COAST_DEG, COAST_RATIO = 8.0, 0.57       # 끊은 뒤 감속 슬루가 더 미는 양: 20° 걸음이 14°에 끊겨 22.2°로 끝난다(11/06 실기). 짧으면 속도가 덜 붙어 비례, 큰 회전의 상한은 실측 전

ST = {"contact": 0, "off_t": None, "deadline": float("inf"), "cam": (0.0, 0.0), "furn": [], "plant": None, "fruit": None,
      "stake": None, "red": 1.0, "light": 1.0, "seq": 0}


def foot_pts(p):
    xs = np.linspace(FOOT[0], FOOT[1], 7)
    ys = np.linspace(FOOT[2], FOOT[3], 5)
    b = np.array([(x, y) for x in xs for y in (ys[0], ys[-1])] + [(x, y) for y in ys for x in (xs[0], xs[-1])])
    co, si = math.cos(p[2]), math.sin(p[2])
    return np.c_[p[0] + co * b[:, 0] - si * b[:, 1], p[1] + si * b[:, 0] + co * b[:, 1]]


def solids():
    return ST["furn"] + [ST["plant"], ST["stake"]]


def hits(p, margin=0.0):
    pts = foot_pts(p)
    R = world.ROOM
    if (pts[:, 0] < R[0] + margin).any() or (pts[:, 0] > R[1] - margin).any() or \
            (pts[:, 1] < R[2] + margin).any() or (pts[:, 1] > R[3] - margin).any():
        return True
    return any(((pts[:, 0] > B[0] - margin) & (pts[:, 0] < B[1] + margin) &
                (pts[:, 1] > B[2] - margin) & (pts[:, 1] < B[3] + margin)).any() for B in solids())


def physics():
    now = time.monotonic()
    dt, world.T["t"] = now - world.T["t"], now
    T, rng = world.T, world.rng
    while dt > 1e-6:
        h = min(dt, 0.05)
        dt -= h
        w = T.get("w", 0)
        vx, vy = T.get("vx", 0), T.get("vy", 0)
        q = world.true.copy()
        co = T.get("coast", 0.0)
        if co and abs(w) < world.THR:
            step_ = math.copysign(min(abs(co), world.RATE * h), co)
            T["coast"] = co - step_
            q[2] += step_
        elif abs(w) >= world.THR:
            q[2] += math.copysign(world.RATE, w) * h
            T["spun"] = T.get("spun", 0.0) + world.RATE * h
            q[:2] += rng.normal(0, 0.05, 2) * h          # 돌며 밀림(space_map_sim SIM_SLIP 기본)
        elif abs(vx) >= world.THR_V or abs(vy) >= world.THR_V:
            sx = math.copysign(world.SPEED, vx) * h if abs(vx) >= world.THR_V else 0.0
            sy = math.copysign(world.SPEED, vy) * h if abs(vy) >= world.THR_V else 0.0
            q[0] += sx * math.cos(q[2]) - sy * math.sin(q[2])
            q[1] += sx * math.sin(q[2]) + sy * math.cos(q[2])
            q[2] += rng.normal(0, math.radians(2)) * h
        else:
            continue
        if hits(q):
            ST["contact"] += 1
            T["blocked"] = True
            T["coast"] = 0.0
        else:
            T["yaw_imu"] += (q[2] - world.true[2]) * (world.SCALE_ERR if abs(w) >= world.THR or co else 1.0)
            world.true[:] = q
            T["blocked"] = False
    T["yaw_imu"] += T["bias"] * max(now - T.get("t_b", now), 0.0)
    T["t_b"] = now
    if now > ST["deadline"]:
        ST["deadline"] = float("inf")
        raise TimeoutError("5분")


class Link:
    def set_velocity(self, vx, vy, w):
        physics()
        if abs(world.T.get("w", 0)) >= world.THR and abs(w) < world.THR:
            world.T["coast"] = math.copysign(min(math.radians(COAST_DEG), COAST_RATIO * world.T.get("spun", 0.0)), world.T["w"])
            world.T["spun"] = 0.0
        world.T.update(vx=vx, vy=vy, w=w)

    def stop(self):
        self.set_velocity(0, 0, 0)

    def close(self):
        pass

    def send_raw(self, payload):
        return True

    def drive_ticks(self, ticks, vx=0, vy=0, w=0, timeout=10.0, resumes=2):
        """앞바퀴 카운트로 끊는다. 후진은 목표를 1.25배 넘고(11/06 1387/1111), 막히면 stall."""
        over = 1.25 if vx < 0 else 1.05
        want, moved, t0, stuck = ticks * sm.MM_PER_COUNT / 1000.0 * over, 0.0, time.monotonic(), 0.0
        try:
            while moved < want and time.monotonic() - t0 < timeout:
                prev = world.true[:2].copy()
                self.set_velocity(vx, vy, 0)
                time.sleep(0.05)
                physics()
                d = float(np.hypot(*(world.true[:2] - prev)))
                moved += d
                stuck = stuck + 0.05 if d < 1e-4 else 0.0
                if stuck > 0.5:
                    return {"state": "stall", "got": int(moved / (sm.MM_PER_COUNT / 1000.0)), "target": ticks, "resumes": 0, "why": "stall"}
        finally:
            self.stop()
        got = int(moved / (sm.MM_PER_COUNT / 1000.0))
        return {"state": "done" if moved >= want else "stall", "got": got, "target": ticks, "resumes": 0, "why": ""}


def trace(o, d, sphere=True):
    """광선 (H,W,3) → (앞 거리 t, 물체 번호). 0 벽·바닥 / 1~ 가구 / -1 화분 / -2 지지대 / -3 줄기 / -4 열매."""
    R = world.ROOM
    with np.errstate(divide="ignore", invalid="ignore"):
        hi = (np.array(R[1::2]) - o) / d
        lo = (np.array(R[0::2]) - o) / d
        t = np.nanmin(np.where(d > 0, hi, np.where(d < 0, lo, np.inf)), -1)
        ids = np.zeros(t.shape, int)
        items = [(i + 1, B) for i, B in enumerate(ST["furn"])] + [(-1, ST["plant"]), (-2, ST["stake"]), (-3, ST["stem"])]
        for k, B in items:
            a, b = (np.array(B[0::2]) - o) / d, (np.array(B[1::2]) - o) / d
            tn, tf = np.nanmax(np.minimum(a, b), -1), np.nanmin(np.maximum(a, b), -1)
            hit = (tn <= tf) & (tn > 0) & (tn < t)
            t, ids = np.where(hit, tn, t), np.where(hit, k, ids)
        if sphere:
            oc = o - ST["fruit"]
            qa, qb, qc = (d * d).sum(-1), 2 * (d * oc).sum(-1), (oc * oc).sum() - c.FRUIT_R ** 2
            disc = qb * qb - 4 * qa * qc
            ts = np.where(disc > 0, (-qb - np.sqrt(np.maximum(disc, 0))) / (2 * qa), np.inf)
            hit = (ts > 0) & (ts < t)
            t, ids = np.where(hit, ts, t), np.where(hit, -4, ids)
    return t, ids


def astra_near():
    """Astra 화면 안에 60cm보다 가까운 것이 있나(거친 격자)."""
    p = world.true
    co, si = math.cos(p[2]), math.sin(p[2])
    u, v = np.meshgrid(np.linspace(0, 639, 48), np.linspace(0, 479, 36))
    X, Y = (u - world.IT["ppx"]) / world.IT["fx"], (v - world.IT["ppy"]) / world.IT["fy"]
    d = np.stack([co + si * X, si - co * X, -Y], -1)
    o = np.array([p[0] + world.M["x"] * co, p[1] + world.M["x"] * si, world.M["h"]])
    t, _ = trace(o, d, sphere=False)
    zf = o[2] + d[..., 2] * t > 0.001
    return float(((t * 1000 < ASTRA_MIN_MM) & zf).mean()) > ASTRA_NEAR_FRAC


def astra_state():
    now = time.monotonic()
    near = astra_near()
    if ST["off_t"] is None:
        if near:
            ST["off_t"] = now
    elif now - ST["off_t"] >= ASTRA_REOPEN_S:
        ST["off_t"] = now if near else None
    return ST["off_t"] is None


def frame(after, timeout=40.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        physics()
        if astra_state() and time.time() > after:
            return world.render(), world.META
        time.sleep(0.5)
    raise RuntimeError("Astra 새 프레임이 %.0f초째 없다 — astra-cam이 죽었거나 깊이 0%%(60cm 안을 봤다)" % timeout)


def astra_blocked(wait=0.4):
    time.sleep(wait)
    physics()
    return not astra_state()


class FakeIO:
    def __init__(self, st):
        self.st = st

    def move_yaw(self, yaw):
        time.sleep(abs(yaw - self.st["yaw"]) / 40.0 + 0.3)
        self.st["yaw"] = yaw

    def write(self, norm, secs=0.5):
        time.sleep(secs)
        self.st["pan"] = norm["shoulder_pan"]

    def hold_close(self):
        pass


ARM = {"yaw": -88.9, "pan": 0.6}


def look_arm():
    park = dict(zip(("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"), (float(x) for x in sm.PARK.split(","))))
    return FakeIO(ARM), types.SimpleNamespace(to_norm=lambda d: d), park, -88.9


def d405_image():
    """지금 회전판·어깨 자세의 손목 D405 사진·깊이(합성)."""
    physics()
    p = world.true
    tb = math.radians(ARM["yaw"] + 88.9)                      # 회전판: 기본 자리 = 0
    b = tb - math.radians(ARM["pan"] - 0.6)                   # 화면 방위 = 판 − 어깨(look_bearing)
    sx = sm.MOUNT["arm_x"] + PAN_OFF * math.cos(tb)           # 어깨 축
    sy = PAN_OFF * math.sin(tb)
    lx, ly = sx + LENS_R * math.cos(b), sy + LENS_R * math.sin(b)
    co, si = math.cos(p[2]), math.sin(p[2])
    o = np.array([p[0] + co * lx - si * ly, p[1] + si * lx + co * ly, WRIST_H])
    th = p[2] + b
    ct, st = math.cos(sm.WRIST_TILT_RAD), math.sin(sm.WRIST_TILT_RAD)
    f = np.array([math.cos(th) * ct, math.sin(th) * ct, -st])
    le = np.array([-math.sin(th), math.cos(th), 0.0])
    up = np.cross(f, le)
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X, Y = (IT.ppx - u) / IT.fx, (IT.ppy - v) / IT.fy
    d = f + X[..., None] * le + Y[..., None] * up
    t, ids = trace(o, d)
    pts = o + d * t[..., None]
    bgr = np.empty((H, W, 3), np.float32)
    bgr[...] = (120, 120, 120)
    bgr[pts[..., 2] < 0.002] = (70, 75, 80)                   # 바닥(깊이는 잡힌다 — D405는 스테레오)
    bgr[ids > 0] = (60, 90, 120)                              # 가구
    bgr[ids == -1] = (40, 40, 50)
    bgr[(ids == -2) | (ids == -3)] = (40, 130, 40)
    fr = ids == -4
    if fr.any():
        nz = (pts[..., 2][fr] - ST["fruit"][2]) / c.FRUIT_R
        red = nz > ST["red_cut"]                               # 위쪽만 익었다 — 500mm에서 빨간 부분 환산 50mm(11/06)
        bgr[fr] = np.where(red[:, None], np.array([25.0, 25.0, 190.0]), np.array([40.0, 150.0, 120.0]))
    bgr = np.clip(bgr * ST["light"] + world.rng.normal(0, 4, bgr.shape), 0, 255).astype(np.uint8)
    return bgr, t * 1000.0


class FakeView:
    def __init__(self, camera="d405"):
        self.key, self.img = None, None

    def _img(self):
        key = (tuple(np.round(world.true, 4)), ARM["yaw"], ARM["pan"])
        if key != self.key:
            self.key, self.img = key, d405_image()
        return self.img

    def intrinsics(self):
        return IT

    def meta(self):
        ST["seq"] += 1
        return {"ts": time.time(), "seq": ST["seq"], "depth_scale_mm": 1.0, "min_mm": 70.0, "max_mm": 900.0, "valid_frac": 0.9}

    def depth_raw(self):
        mm = self._img()[1]
        rng = world.rng
        out = mm + rng.normal(0, 1, mm.shape) * (mm / 1000.0) ** 2 * 8.0
        out = np.where(rng.random(mm.shape) < np.clip((mm - 500.0) / 1500.0, 0.03, 0.6), 0.0, out)      # 멀수록 구멍(11/05)
        return np.clip(out, 0, 65535).astype(np.uint16)

    def color_bgr(self):
        return self._img()[0]


def tomato_fresh(after, timeout=4.0):
    time.sleep(0.1)
    physics()
    p, m, rng = world.true, world.M, world.rng
    co, si = math.cos(p[2]), math.sin(p[2])
    lens = np.array([p[0] + m["x"] * co, p[1] + m["x"] * si])
    fxy = ST["fruit"][:2]
    d = fxy - lens
    b = sm.wrap(math.atan2(d[1], d[0]) - p[2])
    f = (c.TCAM["w"] / 2) / math.tan(math.radians(c.TCAM["hfov"]) / 2)
    pos = []
    if abs(b) < math.radians(c.TCAM["hfov"] / 2) and 0.3 < np.hypot(*d) < 4.5 and rng.random() < c.TCAM["detect_p"] \
            and not any(c.seg_hits_box(lens, fxy, B) for B in ST["furn"]):
        cx = c.TCAM["w"] / 2 - f * math.tan(b) + rng.normal(0, c.TCAM["px"])
        cy = c.TCAM["h"] / 2 - f * math.tan(math.atan2(c.FRUIT_Z - m["h"], np.hypot(*d))) + rng.normal(0, c.TCAM["px"])
        pos.append([float(cx), float(cy)])
    return {"ts": time.time(), "w": c.TCAM["w"], "h": c.TCAM["h"], "positions": pos}


def wrist_tomato(prefer_u=None, waiting_pose=False):
    physics()
    p, rng = world.true, world.rng
    ax = sm.MOUNT["arm_x"] + LENS_R
    cam = np.array([p[0] + ax * math.cos(p[2]), p[1] + ax * math.sin(p[2])])
    fxy = ST["fruit"][:2]
    d = fxy - cam
    b = sm.wrap(math.atan2(d[1], d[0]) - p[2])
    dist = float(np.hypot(*d)) * 1000 - c.FRUIT_R * 1000
    if abs(b) > math.radians(43) or dist > 1500 or dist < 30 or any(c.seg_hits_box(cam, fxy, B) for B in ST["furn"]):
        return None
    mm = dist + rng.normal(0, 3) if 70.0 <= dist <= 900.0 else float("nan")
    if mm == mm and dist > 500 and rng.random() < 0.35:
        mm = float("nan")                                      # 50cm 너머 작은 열매 깊이가 자주 빈다(11/05: 열에 서넛)
    u = IT.ppx - IT.fx * math.tan(b) + rng.normal(0, 2)
    return u, mm, math.degrees(math.atan2(IT.ppx - u, IT.fx)), types.SimpleNamespace(
        intrinsics=lambda: IT, meta=lambda: {"valid_frac": 0.9})


def install(tmp):
    world.physics = physics
    world.sys.modules["base_nudge"].open_link = lambda *a, **k: Link()
    tt.open_link = lambda *a, **k: Link()
    world.M["x"], world.M["y"] = 0.345, 0.0                     # 10/23 자기 보정(y는 회전으로 안 갈려 0)
    sm.frame, sm.astra_blocked = frame, astra_blocked
    _imu = world.imu_now

    def imu_now(max_age=1.0):
        i = _imu(max_age)
        return None if i is None else (i[0], i[1], i[2] and not world.T.get("coast", 0.0))
    sm.imu_now = imu_now
    sm.tomato_fresh, sm.wrist_tomato, sm.reach_tomato = tomato_fresh, wrist_tomato, wrist_tomato
    sm.wrist_front_mm = lambda v: float("inf")
    sm.front_hint = lambda m: None
    sm.subprocess = types.SimpleNamespace(call=lambda *a, **k: 0)
    sm._look_arm = look_arm
    depth_camera.DepthView = FakeView
    sm.LOOK_DIR = os.path.join(tmp, "look")
    sm.MAP = os.path.join(tmp, "m.npz")
    sm.SCANS = os.path.join(tmp, "scans")
    tt.snap = lambda *a, **k: None


def make_room(wx, wy, rng):
    """방 + 가구 넷(서랍장·선반은 벽에, 의자·배낭은 아무 데나) + 화분. 로봇은 근접 끝자리에서 출발."""
    world.ROOM = (0.0, wx, 0.0, wy, 0.0, 2.5)
    for _ in range(2000):
        furn = []
        for w_, d_, h_ in ((1.1, 0.45, 0.9), (1.0, 0.35, 1.6)):
            s = rng.integers(4)
            if s < 2:
                x = rng.uniform(0, max(wx - w_, 0.01))
                y = 0.0 if s == 0 else wy - d_
                furn.append((x, x + w_, y, y + d_, 0.0, h_))
            else:
                y = rng.uniform(0, max(wy - w_, 0.01))
                x = 0.0 if s == 2 else wx - d_
                furn.append((x, x + d_, y, y + w_, 0.0, h_))
        for w_, h_ in ((0.45, 0.85), (0.32, 0.45)):
            x, y = rng.uniform(0, wx - w_), rng.uniform(0, wy - w_)
            furn.append((x, x + w_, y, y + w_, 0.0, h_))
        px, py = rng.uniform(0.3, wx - 0.3), rng.uniform(0.3, wy - 0.3)
        if any(B[0] - 0.25 < px < B[1] + 0.25 and B[2] - 0.25 < py < B[3] + 0.25 for B in furn):
            continue
        h = c.POT[0] / 2
        a = rng.uniform(-math.pi, math.pi)
        sx, sy = px + 0.045 * math.cos(a), py + 0.045 * math.sin(a)
        ST.update(furn=furn, plant=(px - h, px + h, py - h, py + h, 0.0, c.POT[1]),
                  stem=(px - 0.005, px + 0.005, py - 0.005, py + 0.005, c.POT[1], c.FRUIT_Z + c.FRUIT_R + 0.08),
                  stake=(sx - 0.006, sx + 0.006, sy - 0.006, sy + 0.006, c.POT[1], 0.75),
                  fruit=np.array([px, py, c.FRUIT_Z]), red_cut=rng.uniform(-0.4, 0.2), light=rng.uniform(0.45, 1.0))
        th = rng.uniform(-math.pi, math.pi)
        r = sm.MOUNT["arm_x"] + LENS_R + 0.45 + c.FRUIT_R
        rob = np.array([px - r * math.cos(th), py - r * math.sin(th), th])
        if not hits(rob, 0.03):
            world.BOXES[:] = furn + [ST["plant"], ST["stem"], ST["stake"],
                                     (px - c.FRUIT_R, px + c.FRUIT_R, py - c.FRUIT_R, py + c.FRUIT_R, c.FRUIT_Z - c.FRUIT_R, c.FRUIT_Z + c.FRUIT_R)]
            world.true[:] = rob
            world.start[:] = rob
            world.T.update(w=0, vx=0, vy=0, coast=0.0, t=time.monotonic(), yaw_imu=0.0, th_prev=rob[2], t_b=time.monotonic())
            ST["off_t"] = None
            ARM.update(yaw=-88.9, pan=0.6)
            return rob
    raise RuntimeError("방이 너무 좁아 시작 자리가 없다")


ACT = types.SimpleNamespace(steps=16, w=150, wsecs=0.8, deg=20.0, hand=False, vx=150, vsecs=1.0, tol=0.2, max_steps=60,
                            near_mm=sm.NEAR_MM, reach_mm=sm.REACH_MM, x=0.0, y=0.0, face=True, rounds=6, reach=2.0)


def step(mode):
    fn = {"look": sm.look_cmd, "scan": sm.scan_cmd, "find": sm.find_cmd, "approach": sm.approach_cmd, "near": sm.near_cmd}[mode]
    a = types.SimpleNamespace(**vars(ACT))
    a.vx = 200 if mode == "approach" else 150
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            rc = fn(a, world.M)
    except TimeoutError:
        raise
    except Exception as e:                       # 실기라면 트레이스백 → rc≠0
        rc = 1
        buf.write("예외 %s: %s\n" % (type(e).__name__, e))
    return rc, buf.getvalue().splitlines()


def place_far(rng):
    px, py = ST["fruit"][:2]
    for _ in range(3000):
        r, a = rng.uniform(1.0, 2.2), rng.uniform(-math.pi, math.pi)
        rob = np.array([px + r * math.cos(a), py + r * math.sin(a), rng.uniform(-math.pi, math.pi)])
        if not hits(rob, 0.05):
            world.true[:] = rob
            world.start[:] = rob
            world.T.update(w=0, vx=0, vy=0, coast=0.0, t=time.monotonic(), yaw_imu=0.0, th_prev=rob[2], t_b=time.monotonic())
            ST["off_t"] = None
            return True
    return False


def trial(back, deg, reposition, route):
    """follow_trials 한 회 → dict. reposition=False면 앞 회 실패 자리에서 그대로."""
    c0 = ST["contact"]
    buf = io.StringIO()
    if reposition:
        with contextlib.redirect_stdout(buf):
            tt.reposition(back, deg, 150)
    time.sleep(1.5)
    setup = buf.getvalue().splitlines()
    c1 = ST["contact"]
    t0 = time.time()
    ST["deadline"] = t0 + LIMIT_S
    kind, why, log = "", "", []
    if route == "scan" and os.path.exists(sm.MAP):
        os.remove(sm.MAP)
    if route == "look" and os.path.exists(sm.MAP):
        os.remove(sm.MAP)
    steps = ("look", "near") if route == "look" else ("scan", "find", "approach", "near")
    try:
        for mode in steps:
            rc, lines = step(mode)
            log += ["## %s rc=%s" % (mode, rc)] + lines
            if rc != 0:
                kind = "못 찾음" if any("찾아 둔 토마토가 없다" in x or "보지 못했다" in x for x in lines) else ft.FAIL.get(mode, "접근 실패")
                why = mode + ": " + (([x for x in lines if "⛔" in x or "예외" in x] or lines[-1:] or [""])[-1])
                break
    except TimeoutError:
        kind, why = "시간초과", "5분"
    ST["deadline"] = float("inf")
    world.T.update(w=0, vx=0, vy=0)
    secs = time.time() - t0
    end = tt.measure()
    if not kind and not (abs(end[0] - sm.NEAR_MM) <= tt.OK_TOL_MM and abs(end[1]) <= tt.OK_ANG):
        kind, why = "접근 실패", "끝 %.0fmm %.1f°" % (end[0], end[1])
    bumped = ST["contact"] > c1
    if bumped and not kind:
        kind = "충돌"
    return {"ok": not kind, "kind": kind or "-", "why": why[:110], "secs": round(secs), "mm": round(end[0], 0) if end[0] == end[0] else None,
            "ang": round(end[1], 1) if end[1] == end[1] else None, "bump_setup": c1 > c0, "bump": bumped,
            "reposition": setup[-3:], "log": log if ARGS.verbose else []}


def run(wx, wy, route, sets, seed):
    out = []
    for k in range(sets):
        rng = np.random.default_rng(seed * 1000 + k)
        world.rng = rng
        make_room(wx, wy, rng)
        fails = False
        for n, (back, deg) in enumerate(ft.STARTS, 1):
            if ARGS.start == "far" and not place_far(rng):
                continue
            r = trial(back, deg, ARGS.start == "near" and not fails, route)
            r.update(set=k, n=n)
            out.append(r)
            fails = not r["ok"]
            print("[%s %s 묶음%d-%d] %s %s %3d초 끝 %s/%s%s · %s" % (
                ARGS.room, route, k, n, "✅" if r["ok"] else "❌", r["kind"], r["secs"], r["mm"], r["ang"],
                " 💥" if r["bump"] else "", r["why"]), flush=True)
            for ln in r["log"]:
                print("    " + ln)
    return out


def selftest():
    rng = np.random.default_rng(1)
    world.rng = rng
    rob = make_room(3.0, 3.0, rng)
    assert not hits(rob) and rob is not None
    # ① 벽 쪽으로 밀면 막힌다
    ST["furn"] = []
    world.BOXES[:] = []
    world.ROOM = (0.0, 2.0, 0.0, 2.0, 0.0, 2.5)
    ST["plant"] = ST["stake"] = ST["stem"] = (9, 9.1, 9, 9.1, 0, 1)
    ST["fruit"] = np.array([9.0, 9.0, 0.45])
    world.true[:] = (1.0, 1.0, 0.0)
    world.T.update(w=0, vx=0, vy=0, t=time.monotonic())
    r = Link().drive_ticks(int(2000 / sm.MM_PER_COUNT), 220, timeout=30.0)
    assert r["state"] == "stall" and world.true[0] + FOOT[1] <= 2.0 + 1e-6, (r, world.true)
    assert ST["contact"] > 0
    # ② 벽 앞 60cm 안: 새 장이 안 온다. 물러나 트이면 40초 뒤 다시 온다
    world.ROOM = (0.0, 5.0, 0.0, 5.0, 0.0, 2.5)
    world.true[:] = (5.0 - 0.345 - 0.3, 2.5, 0.0)
    ST["off_t"] = None
    assert astra_blocked(0)
    try:
        frame(time.time(), timeout=5.0)
        raise AssertionError("꺼졌는데 장이 왔다")
    except RuntimeError:
        pass
    world.true[:] = (2.0, 2.5, 0.0)
    time.sleep(ASTRA_REOPEN_S)
    assert frame(time.time(), timeout=2.0)[0].any()
    # 감속 슬루: 20° 걸음이 실기처럼 약 22°로 끝난다
    world.true[:] = (2.5, 2.5, 0.0)
    world.T.update(w=0, vx=0, vy=0, coast=0.0, yaw_imu=0.0, t=time.monotonic())
    got = sm.turn(Link(), 250, 0.8, 20.0)
    assert 19.0 < got < 24.0, got
    # ③ 합성 사진으로 실기 look_fruit이 열매를 찾는다, 방향이 맞다(왼쪽 +)
    world.true[:] = (2.0, 2.5, 0.0)
    ST.update(light=0.8, red_cut=-1.5)
    ax = sm.MOUNT["arm_x"] + PAN_OFF + LENS_R
    ST["fruit"] = np.array([2.0 + ax + 0.5, 2.5 + 0.15, c.FRUIT_Z])
    bgr, mm = d405_image()
    fr = sm.look_fruit(bgr, np.nan_to_num(mm).astype(np.float32), IT)
    assert fr, "look_fruit이 못 찾았다"
    ang = math.degrees(math.atan2(IT.ppx - fr[0][0], IT.fx))
    assert 12 < ang < 22, ang
    assert abs(fr[0][1] - 500) < 30, fr
    print("narrow_sim selftest 통과")


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as tmp:
        install(tmp)
        if ARGS.selftest:
            selftest()
            sys.exit(0)
        wx, wy = (float(v) for v in ARGS.room.split("x"))
        res = run(wx, wy, ARGS.route, ARGS.sets, ARGS.seed)
    k = {}
    for r in res:
        k[r["kind"]] = k.get(r["kind"], 0) + 1
    print("== %s %s: %d/%d 성공 · 부딪힘 %d회 · %s" % (ARGS.room, ARGS.route, sum(r["ok"] for r in res), len(res), sum(r["bump"] for r in res), k))
    d = os.path.expanduser("~/narrow_sim")
    os.makedirs(d, exist_ok=True)
    json.dump(res, open(os.path.join(d, "%s-%s-%s.json" % (ARGS.room, ARGS.route, ARGS.start)), "w", encoding="utf-8"), ensure_ascii=False, indent=0)
