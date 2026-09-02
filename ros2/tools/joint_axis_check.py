#!/usr/bin/env python3
"""관절 **하나씩** 돌려 보고, 그 관절이 실제로 어떤 축을 어느 눈금으로 도는지 잰다.

    ~/lerobot/.venv/bin/python ros2/tools/joint_axis_check.py            (다섯 축 전부)
    ~/lerobot/.venv/bin/python ros2/tools/joint_axis_check.py --joint elbow_flex

⚠ `tomato-voice`는 내리고 `depth-cam`은 켠 채로. 표적이 보이는 자세에서 시작한다.

────────────────────────────────────────────────────────────────────────
왜 하나씩인가

손-눈 보정이 안 맞을 때, 잔차 하나만 보면 **어느 관절이 범인인지 알 수 없다.**
2026-09-01에 나는 그걸 모른 채 격자·부호·풀이법을 차례로 고쳐 봤고 전부 헛수고였다.

관절 하나만 움직이면 나머지는 상수가 되어 식에서 빠진다. 링크 길이도 영점도
안 들어간다:

    p(θ) = R_B^T ( Rot_z(-k·θ) u - t_B )

    θ   서보가 말하는 각도          k   **실제 각도 / 서보가 말하는 각도**
    u   회전축 좌표계에서 본 표적    R_B, t_B  회전축 좌표계 → 카메라

미지수는 아홉 개(R_B 3, u 3, t_B 3)에 눈금 k 하나뿐이다. k가 1에서 벗어나면
그 관절의 `span/200` 환산이 틀린 것이고, 잔차가 크면 그 관절이 **한 축으로
깨끗이 돌지 않는다**(유격·휨)는 뜻이다.

⚠ 이 검사는 **표적이 어디 있는지 몰라도** 된다. 표적은 미지수 u로 함께 풀린다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

import target_check as tc  # noqa: E402
from tomato_picker.hardware import kinematics as kin  # noqa: E402

DEG_PER_TICK = 360.0 / 4096.0
CAL = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json")
CART = os.path.expanduser("~/arm_cartesian.json")
MOUNT_Z_MM = 76.5
FLOOR_MARGIN_MM = 10.0
STEP_DEG = 8.0
BACKOFF_DEG = 6.0
SETTLE = 1.3
DOTS = ("tl", "tr", "bl", "br")
# 관절마다 표적을 잃지 않는 선에서 가능한 한 넓게.
# ⚠ 좁게 훑으면 잔차는 낮아도 **축 방향이 안 정해진다** — 2026-09-01:
#   pan을 19.7°만 훑었더니 축이 실행마다 16.8° 달라졌다.
SPAN = {"shoulder_pan": 26.0, "shoulder_lift": 20.0, "elbow_flex": 20.0,
        "wrist_flex": 20.0, "wrist_roll": 60.0}


def rot_z(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def fit(th: np.ndarray, P: np.ndarray, k: float, tries: int, rng) -> tuple:
    """주어진 눈금 k에서 (R_B, u, t_B)를 풀고 잔차를 돌려준다.

    R_B를 고정하면 (u, t_B)는 선형이다 — 그래서 회전만 훑는다.
    """
    n = len(th)
    if P.ndim == 2:                       # 옛 형식(무게중심 하나)도 받는다
        P = P[:, None, :]
    K = P.shape[1]
    # 점마다 u_m 을 따로 두고 t_B 는 공유한다 — 미지수 3K+3.
    M = np.zeros((3 * n * K, 3 * K + 3))
    for i, t in enumerate(th):
        Rz = rot_z(-k * t)
        for m in range(K):
            r0 = 3 * (i * K + m)
            M[r0:r0 + 3, 3 * m:3 * m + 3] = Rz
            M[r0:r0 + 3, 3 * K:3 * K + 3] = -np.eye(3)
    Mp = np.linalg.pinv(M)
    q = rng.normal(size=(tries, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    best = None
    for a, b, c, d in q:
        R = np.array([
            [1 - 2 * (c * c + d * d), 2 * (b * c - d * a), 2 * (b * d + c * a)],
            [2 * (b * c + d * a), 1 - 2 * (b * b + d * d), 2 * (c * d - b * a)],
            [2 * (b * d - c * a), 2 * (c * d + b * a), 1 - 2 * (b * b + c * c)]])
        y = np.einsum('nkj,ij->nki', P, R).reshape(-1)
        sol = Mp @ y
        r = (M @ sol - y).reshape(-1, 3)
        rms = float(np.sqrt((r ** 2).sum(axis=1).mean()))
        if best is None or rms < best[0]:
            best = (rms, R, sol)
    return best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="", help="비우면 다섯 축 전부")
    ap.add_argument("--steps", type=int, default=11)
    ap.add_argument("--tries", type=int, default=20000)
    args = ap.parse_args()

    cal = json.load(open(CAL))
    spans = {}
    for name, c in cal.items():
        try:
            spans[name] = abs(int(c["range_max"]) - int(c["range_min"])) * DEG_PER_TICK
        except (KeyError, TypeError, ValueError):
            pass
    cart = json.load(open(CART))
    zero, ref, signs = cart["zero"], cart["ref_deg"], cart.get("signs", {})
    geom = kin.ArmGeometry()

    def sign(j):
        v = signs.get(j)
        return -1.0 if (v is not None and float(v) < 0) else 1.0

    # ⚠ **실측 눈금이 보정표를 이긴다.** 2026-09-01: `wrist_roll`은 우리가
    #   계산한 각도의 0.56배만 실제로 돌았다(관절축 측정 잔차 24.8mm → 2.6mm,
    #   화면회전 실측 0.549와도 일치). 손목 굴림에 감속이 들어 있다는 뜻이다.
    #   보정표의 틱→도 환산(360/4096)은 그 감속을 모른다. 그래서 `~/arm_cartesian.json`
    #   의 `deg_per_norm`에 실측값을 넣고, 두 계통이 그걸 같이 쓴다.
    over = cart.get("deg_per_norm") or {}

    def per(j):
        v = over.get(j)
        if v:
            return abs(float(v))
        s = spans.get(j)
        return abs(s) / 200.0 if s else (1.8 if j == "wrist_roll" else 0.9)

    def to_deg(n):
        return {j: ref.get(j, 0.0) + sign(j) * (float(n.get(j, 0.0)) - zero.get(j, 0.0)) * per(j)
                for j in kin.JOINTS}

    def to_norm(d):
        return {j: zero.get(j, 0.0) + (float(d[j]) - ref.get(j, 0.0)) / (sign(j) * per(j))
                for j in d if j in kin.JOINTS}

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    home = to_deg(io.read())
    io.write(to_norm(home), 0.5)

    def goto(d):
        cur = to_deg(io.read())
        big = max(abs(d[j] - cur[j]) for j in kin.JOINTS)
        steps = max(1, int(math.ceil(big / STEP_DEG)))
        for s in range(1, steps + 1):
            mid = {j: cur[j] + (d[j] - cur[j]) * s / steps for j in kin.JOINTS}
            if kin.forward(mid, geom).z < -MOUNT_Z_MM + FLOOR_MARGIN_MM:
                return None
            io.write(to_norm(mid), 0.85)
        return True

    def move(d):
        """늘 같은 쪽에서 다가간다 — 백래시를 상수로 만든다."""
        if goto({j: d[j] - BACKOFF_DEG for j in kin.JOINTS}) is None:
            return None
        if goto(d) is None:
            return None
        time.sleep(SETTLE)
        return to_deg(io.read())

    def look():
        try:
            m = tc.measure()
        except Exception:                         # noqa: BLE001
            return None
        if not m or not m.get("ok"):
            return None
        return np.array([m["points_mm"][k] for k in DOTS])

    joints = [args.joint] if args.joint else list(kin.JOINTS)
    print("관절          표본  실제 회전폭   눈금 k      잔차     판정")
    rng = np.random.default_rng(0)
    axes, raw = {}, {}
    for j in joints:
        span = SPAN.get(j, 16.0)
        offs = np.linspace(-span, span, args.steps)
        th, P = [], []
        for o in offs:
            d = dict(home)
            d[j] = home[j] + float(o)
            got = move(d)
            if got is None:
                continue
            v = look()
            if v is None:
                continue
            # ⚠ 무게중심 하나만 쓰면 **회전축 위에 얹힌 관절에서 신호가 사라진다.**
            #   2026-09-01: wrist_roll은 표적이 축 위에 거의 있어서 축 방향이
            #   실행마다 20° 달라졌다(잔차는 1.9mm로 낮았는데도). 네 점은 200mm에
            #   걸쳐 있어 축에서 먼 점이 생기고, 그게 조건수를 살린다.
            #   ⚠ 이름표(tl/tr/…)는 화면 기준이라 굴리면 섞인다. 그래서 이름 대신
            #     **직전 프레임과 가장 가까운 점끼리** 이어 붙인다 — 걸음이 작으니
            #     이 대응은 안전하다.
            if P:
                prev = np.asarray(P[-1])
                used, order = set(), []
                for q in prev:
                    d2 = [(float(np.sum((c - q) ** 2)), i)
                          for i, c in enumerate(v) if i not in used]
                    d2.sort()
                    used.add(d2[0][1])
                    order.append(d2[0][1])
                v = v[order]
            th.append(got[j] - home[j])
            P.append(v)
        move(dict(home))
        if len(th) < 5:
            print(f"  {j:<14}{len(th):3}   (표본 부족 — 표적을 잃었다)")
            continue
        th = np.array(th)
        P = np.array(P)   # (n, 4, 3)
        best = None
        for k in np.arange(0.60, 1.451, 0.01):
            rms, R, sol = fit(th, P, float(k), args.tries // 8, rng)
            if best is None or rms < best[0]:
                best = (rms, float(k), R, sol)
        rms, k, R, sol = best
        # 눈금을 1로 고정했을 때와 견준다 — k가 진짜로 필요한가
        rms1 = fit(th, P, 1.0, args.tries, rng)[0]
        verdict = ("눈금 이상" if abs(k - 1.0) > 0.05 and rms < rms1 * 0.6
                   else ("한 축으로 안 돈다" if rms > 6.0 else "정상"))
        print(f"  {j:<14}{len(th):3}   {np.ptp(th):7.1f}°   {k:5.2f}  "
              f"{rms:7.2f}mm (k=1이면 {rms1:.1f})   {verdict}")
        # 회전축을 **카메라 좌표계**로 꺼내 둔다. p = R_B^T (Rot_z(-kθ)u - t_B) 이므로
        # 관절축(그 좌표계의 z)은 카메라계에서 R_B의 셋째 행이다.
        axes[j] = np.array(R[2, :], dtype=float)
        axes[j] /= np.linalg.norm(axes[j])
        raw[j] = {"th": th.tolist(), "P": P.tolist(), "k": k, "rms": rms}

    io.hold_close()

    # ── 축들이 서로 이루는 각 ─────────────────────────────────────────
    # 모델은 lift·elbow·wrist_flex 세 축이 **평행**하고 pan은 그것들과 **수직**,
    # wrist_roll도 수직이라고 가정한다(`a3 = lift + elbow + wrist_flex`가 그 가정이다).
    # 그 가정이 몇 도나 어긋나 있는지 여기서 처음으로 실측된다.
    if len(axes) >= 2:
        print()
        print("[축이 서로 이루는 각] 모델의 가정과 견준다")
        want = {("shoulder_lift", "elbow_flex"): 0.0,
                ("shoulder_lift", "wrist_flex"): 0.0,
                ("elbow_flex", "wrist_flex"): 0.0,
                ("shoulder_pan", "shoulder_lift"): 90.0,
                ("shoulder_pan", "elbow_flex"): 90.0,
                ("shoulder_pan", "wrist_flex"): 90.0,
                ("wrist_roll", "wrist_flex"): 90.0}
        for (a, b), w in want.items():
            if a not in axes or b not in axes:
                continue
            # 축의 부호는 이 방법으로 안 정해지므로 **예각으로** 본다.
            c = abs(float(np.dot(axes[a], axes[b])))
            ang = math.degrees(math.acos(max(0.0, min(1.0, c))))
            err = abs(ang - w)
            mark = "  ⚠" if err > 4.0 else ""
            print(f"  {a:<14}~ {b:<14} 기대 {w:4.0f}°  실측 {ang:6.1f}°"
                  f"  차이 {err:5.1f}°{mark}")
        with open(os.path.expanduser("~/joint_axes.json"), "w", encoding="utf-8") as fh:
            # ⚠ **측정할 때의 자세를 반드시 같이 남긴다.** 축의 방향과 위치는
            #   그 자세에서만 뜻이 있다. 2026-09-01에 이걸 빠뜨려 나중에 자세를
            #   추정치로 넣었고, 거리 잔차 31mm의 상당 부분이 거기서 왔다.
            json.dump({"home_deg": {k2: float(v) for k2, v in home.items()},
                       "axes": {k2: v.tolist() for k2, v in axes.items()}, "raw": raw},
                      fh, ensure_ascii=False)
        print("  원자료 ~/joint_axes.json")

    print("")
    print("읽는 법 — k는 **실제 각도 / 서보가 말하는 각도**다.")
    print("  k가 1에서 크게 벗어나면 그 관절의 `span/200` 환산이 틀렸다.")
    print("  k는 1인데 잔차가 크면 그 관절이 한 축으로 깨끗이 안 돈다(유격·휨).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
