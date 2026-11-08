#!/usr/bin/env python3
"""**팔 + 손목 카메라 보정이 정말 맞는가** — 참값을 아는 가짜 팔에서 실기 풀이를 돌려 푼 값과 참값을 비교한다. PC, 장치 없이.

    PYTHONIOENCODING=utf-8 python ros2/tools/handeye_sim.py --mc 40
    python ros2/tools/handeye_sim.py --real ~/handeye_samples.json    # 9/4 실측 원자료를 같은 모형으로 다시 푼다
    python ros2/tools/handeye_sim.py --selftest

잔차는 '자기 식과 얼마나 맞나'일 뿐 맞았다는 뜻이 아니다. 여기선 참 T_tool_cam을 아니까 진짜 오차와
"그 보정으로 줄기를 보고 손끝을 보내면 집게 창에 드나"를 바로 잰다.
가짜 팔은 **FK와 팬·롤 부호가 반대**다 — 9/4 실측이 그렇게 말한다(--real: 그대로 풀면 잔차 20.5mm·|t| 435, 뒤집으면 12.8mm·|t| 109).
풀이: A 지금(4점 표적 무게중심, handeye_resolve) · A 부호 고침 · A+ 넓은 자세 · B ChArUco 전자세(cv2.calibrateHandEye) ·
C B에 관절 영점·링크·보이지 않는 처짐까지 같이 풀기. 자세 A는 9/4 실기 11자세 그대로.
⚠ 숫자(잡음·처짐)는 SCENES의 가정이다 — 실측 전. 조명·검출 실패·서보 과부하는 없다.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, "..", "..", "src")]
_a = sys.argv
sys.argv = _a[:1]
import handeye_resolve as hr                                   # noqa: E402  실기 풀이
sys.argv = _a
import cv2                                                     # noqa: E402
from scipy.optimize import least_squares                       # noqa: E402
from tomato_picker.hardware import kinematics as kin           # noqa: E402

J = kin.JOINTS
GEOM = dict(z0=119.5, d0=-31.5, l1=116.5, l2=138.0, l3=168.0)
FX, PP, SIZE = 438.0, (424.0, 240.0), (848, 480)
K_CAM = np.array([[FX, 0, PP[0]], [0, FX, PP[1]], [0, 0, 1.0]])
LIM = {"shoulder_pan": (-107.7, 135.7), "shoulder_lift": (5.9, 211.6), "elbow_flex": (-41.0, 165.7), "wrist_flex": (-102.1, 105.7)}
POSES_0904 = [(-18.4, 11.7, 47.2, 97.6, -1.3), (17.7, 23.0, 63.0, 100.6, -61.6), (-9.2, 21.5, 32.4, 102.6, -61.6),
              (-9.2, 21.4, 32.4, 102.4, 58.8), (17.7, 23.1, 62.9, 100.7, -1.3), (-18.4, 23.0, 62.9, 100.7, -31.4),
              (-18.4, 23.1, 62.9, 100.7, 28.6), (17.7, 12.3, 62.9, 89.7, -31.4), (17.8, 12.3, 63.0, 89.7, 28.6),
              (-18.4, 12.3, 62.9, 96.7, 58.7), (-9.1, 11.7, 47.2, 97.5, -31.4)]        # handeye_samples.json 2026-09-04
START = (1.6, 93.9, 115.7, -34.7, 6.4)     # 잡기 시작 자세(규약)
WIN = {"approach": (-12.0, 18.0), "lateral": 14.0, "up": 20.0}   # 집게 창(checkpoint_sim DEPTH_WIN·GAP_HALF·BAND_HALF)
M1, M_ELBOW, M2, M_HAND, HAND_D = 0.06, 0.055, 0.115, 0.21, 70.0  # arm_twin과 같은 비율

# 가정 — off: 관절 영점 σ도 / link: 링크 σmm / back: 자세마다 관절 흔들림 σ도(단방향 접근 실측 0.5) / flex: 다 뻗었을 때 엔코더에 안 보이는 처짐 mm
# play: 브래킷 유격(도, mm, 자세마다) / px·depth: 표적 잡음 / scale: 깊이 배율(target_check 실측 −1.24%)
SCENES = {"이상(잡음만)": dict(off=0.0, link=0.0, back=0.0, flex=0.0, play=(0.0, 0.0), px=0.3, depth=1.0, scale=0.0),
          "현실 추정": dict(off=1.0, link=1.5, back=0.3, flex=6.0, play=(0.0, 0.0), px=0.3, depth=1.5, scale=0.0124),
          "처짐 큼 15mm": dict(off=1.0, link=1.5, back=0.3, flex=15.0, play=(0.0, 0.0), px=0.3, depth=1.5, scale=0.0124),
          "브래킷 헐거움": dict(off=1.0, link=1.5, back=0.3, flex=6.0, play=(1.0, 2.0), px=0.3, depth=1.5, scale=0.0124),
          "영점 3°": dict(off=3.0, link=1.5, back=0.3, flex=6.0, play=(0.0, 0.0), px=0.3, depth=1.5, scale=0.0124)}


def rotvec(v):
    return cv2.Rodrigues(np.asarray(v, float).reshape(3, 1))[0]


def x_true():
    """참 T_tool_cam — 9/4 실측을 부호 고쳐 푼 값(|t| 109, 자로 잰 손끝 76~78과는 정의가 다르다: TCP는 l3 끝)."""
    z = np.array([0.93, 0.16, 0.33])
    z /= np.linalg.norm(z)
    y = np.array([-0.35, 0.13, 0.93])
    y -= z * (y @ z)
    y /= np.linalg.norm(y)
    return np.c_[np.cross(y, z), y, z], np.array([-100.0, -21.0, -36.0])


def flipq(q):
    """실제 팔의 관절 ← 엔코더 값: 팬·롤 부호가 FK와 반대."""
    q = np.array(q, float)
    q[0], q[4] = -q[0], -q[4]
    return q


def gravity(q):
    a1 = math.radians(q["shoulder_lift"])
    a2 = a1 + math.radians(q["elbow_flex"])
    a3 = a2 + math.radians(q["wrist_flex"])
    c1, c2, c3 = math.cos(a1), math.cos(a2), math.cos(a3)
    wrist = M_HAND * HAND_D * c3
    elbow = wrist + M_HAND * GEOM["l2"] * c2 + M2 * GEOM["l2"] / 2 * c2
    lift = elbow + (M_HAND + M2 + M_ELBOW) * GEOM["l1"] * c1 + M1 * GEOM["l1"] / 2 * c1
    return {"shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist}


_flat = dict(zip(J, (0, 0, 0, 0, 0)))
TIP_PER_C = (kin.forward(_flat, kin.ArmGeometry(**GEOM)).z - kin.forward(
    {j: _flat[j] - 1e-3 * gravity(_flat).get(j, 0.0) for j in J}, kin.ArmGeometry(**GEOM)).z) / 1e-3   # c당 끝점 처짐 mm(다 뻗은 자세)


def flex_c(mm):
    return mm / TIP_PER_C


def joints(q, off=None, c=0.0):
    d = dict(zip(J, q)) if not isinstance(q, dict) else dict(q)
    if off is not None:
        for j, v in zip(J, off):
            d[j] += v
    if c:
        for j, v in gravity(d).items():
            d[j] -= c * v
    return d


def frame(q, geom, off=None, c=0.0):
    R, t = hr.tool_frame(joints(q, off, c), kin.ArmGeometry(**geom), 1.0)
    return R, t


class World:
    def __init__(self, sc, rng):
        self.sc, self.rng = sc, rng
        self.off = rng.normal(0, sc["off"], 5) if sc["off"] else np.zeros(5)
        self.geom = dict(GEOM, l1=GEOM["l1"] + rng.normal(0, sc["link"] or 1e-9), l2=GEOM["l2"] + rng.normal(0, sc["link"] or 1e-9),
                         l3=GEOM["l3"] + rng.normal(0, sc["link"] or 1e-9))
        self.c = flex_c(sc["flex"])
        self.Rx, self.tx = x_true()
        # 벽 표적: 기준 자세에서 320mm 앞, 카메라를 대략 마주본다
        R, t = self.cam(np.mean(POSES_0904, axis=0))
        self.Rb = R @ rotvec(rng.normal(0, 0.1, 3))
        self.tb = t + R @ np.array([0, 0, 320.0])

    def cam(self, q, shake=False):
        """참 T_base_cam. shake면 자세마다의 흔들림·유격을 넣는다."""
        sc, rng = self.sc, self.rng
        off = self.off + (rng.normal(0, sc["back"], 5) if shake and sc["back"] else 0.0)
        R, t = frame(flipq(q), self.geom, off, self.c)
        Rx, tx = self.Rx, self.tx
        if shake and sc["play"][0]:
            Rx = Rx @ rotvec(np.radians(rng.normal(0, sc["play"][0], 3)))
            tx = tx + rng.normal(0, sc["play"][1], 3)
        return R @ Rx, R @ tx + t

    def tool(self, q):
        return frame(flipq(q), self.geom, self.off, self.c)


BOARD = np.array([(x, y, 0.0) for x in np.arange(-75, 76, 30.0) for y in np.arange(-105, 106, 30.0)])     # ChArUco 안쪽 모서리 6×8
DOTS = {"tl": (-50.0, -87.25, 0.0), "tr": (50.0, -87.25, 0.0), "bl": (-50.0, 87.25, 0.0), "br": (50.0, 87.25, 0.0)}


def to_cam(W, P, cam):
    return (P @ W.Rb.T + W.tb - cam[1]) @ cam[0]       # 표적→base→카메라


def in_view(pc):
    z = pc[:, 2]
    if (z < 150).any() or (z > 500).any():
        return False
    u = pc[:, 0] / z * FX + PP[0]
    v = pc[:, 1] / z * FX + PP[1]
    return bool((u > 10).all() and (u < SIZE[0] - 10).all() and (v > 10).all() and (v < SIZE[1] - 10).all())


def see_dots(W, q, fixed=True):
    """4점 표적 → handeye_resolve 표본(잡음 포함). fixed=False면 지금처럼 부호 틀린 엔코더 값을 FK에 넣는다. 안 보이면 None."""
    cam = W.cam(q, shake=True)
    P = np.array(list(DOTS.values()))
    pc = to_cam(W, P, cam)
    if not in_view(pc):
        return None
    sc, rng = W.sc, W.rng
    z = pc[:, 2]
    pn = pc + np.c_[z * rng.normal(0, sc["px"], 4) / FX, z * rng.normal(0, sc["px"], 4) / FX, rng.normal(0, sc["depth"], 4)]
    pn = pn * (1.0 + sc["scale"])
    return {"joints_deg": dict(zip(J, flipq(q) if fixed else q)), "dots_mm": {k: p.tolist() for k, p in zip(DOTS, pn)}, "label": ""}


def see_board(W, q):
    """ChArUco 모서리 → (rvec, tvec, 화소) 표적→카메라. 안 보이면 None."""
    cam = W.cam(q, shake=True)
    pc = to_cam(W, BOARD, cam)
    if not in_view(pc):
        return None
    uv = pc[:, :2] / pc[:, 2:] * FX + PP + W.rng.normal(0, W.sc["px"], (len(pc), 2))
    ok, rv, tv = cv2.solvePnP(BOARD, uv, K_CAM, None, flags=cv2.SOLVEPNP_ITERATIVE)
    return (cv2.Rodrigues(rv)[0], tv.ravel(), uv) if ok else None


def wide_poses(W, n, rng):
    """보이는 자세 가운데 서로 먼 것 n개 — 실기 제약(r≤200, 관절 한계, 손목 한계)을 지킨다."""
    home = np.mean(POSES_0904, axis=0)
    cand = []
    for _ in range(4000):
        d = rng.uniform((-28, -18, -28, -22, -75), (28, 22, 18, 22, 75))
        q = home + np.array([d[0], d[1], d[2], d[3] - d[1] - d[2], d[4]])
        if any(not (LIM[j][0] + 2 < q[i] < LIM[j][1] - 2) for i, j in enumerate(J[:4])):
            continue
        p = kin.forward(dict(zip(J, q)), kin.ArmGeometry(**GEOM))
        if math.hypot(p.x, p.y) > 200 or not in_view(to_cam(W, BOARD, W.cam(q))):
            continue
        cand.append((d, q))
        if len(cand) >= 500:
            break
    if not cand:
        return []
    ds = np.array([c[0] for c in cand]) / np.array([28, 20, 23, 22, 75])
    pick = [int(np.argmin((ds ** 2).sum(1)))]
    while len(pick) < min(n, len(cand)):
        pick.append(int(np.argmax(((ds[:, None] - ds[pick]) ** 2).sum(-1).min(1))))
    return [cand[i][1] for i in pick]


def solve_dots(samples, tries=4000):
    a = hr.solve(samples, kin.ArmGeometry(**GEOM), 1.0, ("mid",), iters=3000)
    b = hr.solve_global(samples, kin.ArmGeometry(**GEOM), 1.0, ("mid",), tries=tries)
    g = b if b[0] < a[0] else a
    return {"R": g[1], "t": g[2], "rms": g[0]}


def solve_board(W, poses):
    Rg, tg, Rc, tc, q2, uv = [], [], [], [], [], []
    for q in poses:
        s = see_board(W, q)
        if s is None:
            continue
        R, t = frame(flipq(q), GEOM)
        Rg.append(R), tg.append(t), Rc.append(s[0]), tc.append(s[1]), q2.append(q), uv.append(s[2])
    Rx, tx = cv2.calibrateHandEye(Rg, tg, Rc, tc, method=cv2.CALIB_HAND_EYE_PARK)
    tx = tx.ravel()
    # 표적 위치 평균·잔차(점 위치 mm) — 지금 합격선과 같은 잣대
    B = [Rg[i] @ (Rx @ tc[i] + tx) + tg[i] for i in range(len(Rg))]
    rms = float(np.sqrt(((np.array(B) - np.mean(B, 0)) ** 2).sum(1).mean()))
    return {"R": Rx, "t": tx, "rms": rms, "q": q2, "uv": uv, "Rg": Rg, "tg": tg, "Rc": Rc, "tc": tc}


def solve_full(W, b):
    """B의 답에서 시작해 X·표적·관절 영점·l1 l2·처짐 계수를 화소 잔차로 같이 푼다(약한 사전: 영점 3°, 링크 3mm)."""
    Rb0 = b["Rg"][0] @ b["R"] @ b["Rc"][0]
    tb0 = b["Rg"][0] @ (b["R"] @ b["tc"][0] + b["t"]) + b["tg"][0]
    x0 = np.r_[cv2.Rodrigues(b["R"])[0].ravel(), b["t"], cv2.Rodrigues(Rb0)[0].ravel(), tb0, np.zeros(4), 0.0, 0.0, 0.0]

    def unpack(x):
        g = dict(GEOM, l1=GEOM["l1"] + x[16], l2=GEOM["l2"] + x[17])
        return rotvec(x[:3]), x[3:6], rotvec(x[6:9]), x[9:12], np.r_[x[12:16], 0.0], g, x[18] * flex_c(10.0)

    def res(x):
        Rx, tx, Rb, tb, off, g, c_ = unpack(x)
        out = []
        for q, uv in zip(b["q"], b["uv"]):
            R, t = frame(flipq(q), g, off, c_)
            pc = (BOARD @ Rb.T + tb - (R @ tx + t)) @ (R @ Rx)
            out.append((pc[:, :2] / pc[:, 2:] * FX + PP - uv).ravel())
        return np.r_[np.concatenate(out), x[12:16] / 3.0, x[16:18] / 3.0, x[18:19] * 0.1]

    s = least_squares(res, x0, method="lm", max_nfev=400)
    Rx, tx, _, _, off, g, c_ = unpack(s.x)
    return {"R": Rx, "t": tx, "off": off, "geom": g, "c": c_, "rms_px": float(np.sqrt((s.fun[:-7] ** 2).mean()))}


def ik(est, q0, target):
    """추정 모형으로 손끝을 target에 — pitch 유지, 4관절 Gauss-Newton."""
    q = np.array(q0, float)
    a30 = sum(q[1:4])

    def f(v):
        R, t = frame(flipq(v) if est["flip"] else v, est["geom"], est["off"], est["c"])
        return np.r_[t, (sum(v[1:4]) - a30) * 3.0]
    tgt = np.r_[target, 0.0]
    for _ in range(8):
        e = tgt - f(q)
        Jn = np.array([(f(q + np.eye(5)[i] * 0.2) - f(q)) / 0.2 for i in range(4)]).T
        q[:4] += np.linalg.lstsq(Jn, e, rcond=None)[0]
    return q


def grasp_eval(W, est, n=120, steps=1):
    """줄기를 보고(카메라 z 110~150, 옆 ±40, 위 ±15) 손끝을 보낸다. steps=2면 75mm 남기고 다시 보고 마저 간다. → 창 안 비율·RMS mm."""
    rng = W.rng
    hit, errs = 0, []
    Rc0, tc0 = W.cam(START)
    Rtool0 = frame(flipq(START), GEOM)[0]
    for _ in range(n):
        pc = np.array([rng.uniform(-40, 40), rng.uniform(-15, 15), rng.uniform(110, 150)])
        p_true = Rc0 @ pc + tc0

        def est_point(q):
            Rc, tc = W.cam(q)
            obs = (p_true - tc) @ Rc * (1.0 + W.sc["scale"])       # 잡음은 뺀다 — 순수 보정 몫
            Re, te = frame(flipq(q) if est["flip"] else q, est["geom"], est["off"], est["c"])
            return Re @ (est["R"] @ obs + est["t"]) + te

        q = np.array(START, float)
        goal = est_point(q)
        if steps == 2:
            app = Rtool0[:, 0]
            q = ik(est, q, goal - app * 75.0)
            goal = est_point(q)
        q = ik(est, q, goal)
        e = Rtool0.T @ (W.tool(q)[1] - p_true)
        errs.append(e)
        hit += WIN["approach"][0] <= e[0] <= WIN["approach"][1] and abs(e[1]) <= WIN["lateral"] and abs(e[2]) <= WIN["up"]
    errs = np.array(errs)
    return hit / n, np.sqrt((errs ** 2).mean(0))


def one_run(sc, seed):
    rng = np.random.default_rng(seed)
    W = World(sc, rng)
    wide = wide_poses(W, 24, rng)
    ident = dict(geom=GEOM, off=None, c=0.0, flip=True)
    rows = {}
    Rt, tt = W.Rx, W.tx
    ang = lambda R: math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R.T @ Rt) - 1) / 2))))

    def add(name, m, est):
        rows[name] = dict(terr=float(np.linalg.norm(m["t"] - tt)), tnorm=float(np.linalg.norm(m["t"])), rerr=ang(m["R"]), rms=m.get("rms", float("nan")),
                          hit1=grasp_eval(W, est)[0], hit2=grasp_eval(W, est, steps=2)[0])

    s00 = [x for x in (see_dots(W, q, fixed=False) for q in POSES_0904) if x]
    if len(s00) >= 5:
        m = solve_dots(s00)
        add("A 지금(부호 틀림) 11자세", m, dict(ident, R=m["R"], t=m["t"], flip=False))
    s0 = [x for x in (see_dots(W, q) for q in POSES_0904) if x]
    if len(s0) >= 5:
        m = solve_dots(s0)
        add("A 부호 고침 11자세", m, dict(ident, R=m["R"], t=m["t"]))
    s1 = [x for x in (see_dots(W, q) for q in wide) if x]
    if len(s1) >= 8:
        m = solve_dots(s1)
        add("A+ 넓은 자세", m, dict(ident, R=m["R"], t=m["t"]))
    if len(wide) >= 8:
        b = solve_board(W, wide)
        add("B ChArUco", b, dict(ident, R=b["R"], t=b["t"]))
        f = solve_full(W, b)
        add("C 처짐까지", f, dict(geom=f["geom"], off=f["off"], c=f["c"], R=f["R"], t=f["t"], flip=True))
    R0 = Rt @ rotvec(np.radians(rng.normal(0, 3.0, 3)))
    t0 = tt + rng.normal(0, 5.0, 3)
    add("자로 잼(±5mm·±3°)", dict(R=R0, t=t0), dict(ident, R=R0, t=t0))
    return rows


def summary(all_rows):
    names = []
    for r in all_rows:
        names += [k for k in r if k not in names]
    print("| 풀이 | 참값과 t 차이(중앙값) | \\|t\\| 90%% 범위 | 회전 오차 | 잔차 | 한 번에 보내기 창 안 | 75mm에서 다시 보기 창 안 |".replace("%%", "%"))
    print("|---|---|---|---|---|---|---|")
    for n in names:
        v = [r[n] for r in all_rows if n in r]
        g = lambda k: np.array([x[k] for x in v])
        print("| %s | %.1fmm | %.0f~%.0f | %.1f° | %.1f | %.0f%% | %.0f%% |" % (n, np.median(g("terr")), *np.percentile(g("tnorm"), (5, 95)), np.median(g("rerr")),
              np.nanmedian(g("rms")), 100 * g("hit1").mean(), 100 * g("hit2").mean()))


def real(path, tries=30000):
    """실측 원자료: 팬·롤 부호 네 조합을 전역으로 푼다(R_x 무작위 + 나머지 선형). 하나씩 빼도 답이 그대로인지 본다."""
    S = json.load(open(path, encoding="utf-8"))["samples"]
    Q = np.array([[x["joints_deg"][j] for j in J] for x in S])
    mid = np.array([np.mean(list(x["dots_mm"].values()), 0) for x in S])      # 4점이든 ChArUco 모서리든
    rng = np.random.default_rng(0)
    qt = rng.normal(size=(tries, 4))
    qt /= np.linalg.norm(qt, axis=1, keepdims=True)
    a, b, c, d = qt.T
    RX = np.stack([1 - 2 * (c * c + d * d), 2 * (b * c - d * a), 2 * (b * d + c * a), 2 * (b * c + d * a), 1 - 2 * (b * b + d * d),
                   2 * (c * d - b * a), 2 * (b * d - c * a), 2 * (c * d + b * a), 1 - 2 * (b * b + c * c)], 1).reshape(-1, 3, 3)

    def solve(Qs, pts):
        n = len(Qs)
        F = [frame(v, GEOM) for v in Qs]
        Rs, bs = np.array([f[0] for f in F]), np.array([f[1] for f in F])
        M = np.zeros((3 * n, 6))
        for i in range(n):
            M[3 * i:3 * i + 3, :3], M[3 * i:3 * i + 3, 3:] = np.eye(3), -Rs[i]
        y = (np.einsum("nij,knj->kni", Rs, np.einsum("kij,nj->kni", RX, pts)) + bs[None]).reshape(len(RX), -1)
        sol = y @ np.linalg.pinv(M).T
        rms = np.sqrt((((sol @ M.T - y).reshape(len(RX), n, 3)) ** 2).sum(2).mean(1))
        k = int(np.argmin(rms))
        return float(rms[k]), sol[k, 3:], RX[k]

    print("표본 %d" % len(S))
    for ps, rs in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        q2 = Q * np.array([ps, 1, 1, 1, rs])
        r, t, R = solve(q2, mid)
        print("  팬 %+d 롤 %+d: 잔차 %5.1fmm · |t| %3.0f · t %s · 카메라 앞이 도구 축으로 %s" % (ps, rs, r, np.linalg.norm(t), np.round(t), np.round(R[:, 2], 2)))
    q2 = Q * np.array([-1, 1, 1, 1, -1])
    print("  팬−·롤−에서 표본 하나씩 빼기 (잔차, |t|, 접근축 성분):")
    for k in range(len(S)):
        keep = np.arange(len(S)) != k
        r, t, _ = solve(q2[keep], mid[keep])
        print("    −%2d: %5.1fmm |t| %3.0f t %s" % (k, r, np.linalg.norm(t), np.round(t)))


def selftest():
    rng = np.random.default_rng(3)
    sc = dict(SCENES["이상(잡음만)"], px=0.0, depth=0.0)
    W = World(sc, rng)
    wide = wide_poses(W, 24, rng)
    assert len(wide) >= 12, len(wide)
    b = solve_board(W, wide)
    assert np.linalg.norm(b["t"] - W.tx) < 1.0, b["t"]
    m = solve_dots([x for x in (see_dots(W, q) for q in wide) if x])
    assert np.linalg.norm(m["t"] - W.tx) < 3.0 and m["rms"] < 0.5, (m["t"], m["rms"])
    hit, rms = grasp_eval(W, dict(geom=GEOM, off=None, c=0.0, flip=True, R=W.Rx, t=W.tx), n=20)
    assert hit == 1.0 and rms.max() < 0.5, (hit, rms)
    bad = dict(geom=GEOM, off=None, c=0.0, flip=True, R=W.Rx, t=W.tx + np.array([40.0, 0, 0]))
    assert grasp_eval(W, bad, n=20)[0] == 0.0, "t 40mm 틀리면 창에 못 든다"
    wrong = solve_dots([x for x in (see_dots(W, q, fixed=False) for q in POSES_0904) if x])
    assert np.linalg.norm(wrong["t"] - W.tx) > 100, ("부호가 틀리면 잡음 없어도 t가 크게 틀려야 한다", wrong["t"])
    c_ = flex_c(10.0)
    z0 = frame(_flat_q(), GEOM)[1][2]
    z1 = frame(_flat_q(), GEOM, None, c_)[1][2]
    assert abs(z0 - z1 - 10.0) < 1.5 and z1 < z0, (z0, z1)
    print("handeye_sim selftest 통과")


def _flat_q():
    return (0.0, 0.0, 0.0, 0.0, 0.0)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mc", type=int, default=40)
    ap.add_argument("--real", default="")
    ap.add_argument("--selftest", action="store_true")
    A = ap.parse_args()
    if A.selftest:
        selftest()
    elif A.real:
        real(A.real)
    else:
        for name, sc in SCENES.items():
            print("\n### %s  %s" % (name, sc))
            rows = [one_run(sc, 100 + i) for i in range(A.mc)]
            summary(rows)
            sys.stdout.flush()
