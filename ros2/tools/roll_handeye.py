#!/usr/bin/env python3
"""굴림만 쓴 표본으로 **R_x 와 t_x 를 기구학과 무관하게** 푼다.

    python ros2/tools/roll_handeye.py ~/roll_samples.json

⚠ PC에서 돈다. `roll_scale_check.py`가 남긴 JSON만 있으면 된다.

────────────────────────────────────────────────────────────────────────
왜 이게 결정적인가

전체 손-눈 보정은 두 가지를 한꺼번에 믿어야 한다 — (가) 관절→도구 **자세**,
(나) 관절→도구 **위치**. 잔차가 크게 나올 때 어느 쪽이 범인인지 알 수 없다.

그런데 **`wrist_roll` 하나만 움직인 표본**에서는 도구 위치가 아예 안 변한다
(집게 끝이 굴림축 위에 있다). 그래서 팔의 위치 모델이 식에서 통째로 빠진다:

    R_x p_i = Rot_x(-θ_i) u - t_x          (u = 표적까지의 벡터, 도구계, 상수)

미지수는 R_x(3) + u(3) + t_x(3) = 9개뿐이고, 링크 길이도 영점도 안 들어간다.
여기서 나온 t_x가 **카메라가 도구에서 실제로 얼마나 벗어나 있는지**다.

2026-09-01: 전체 보정은 |t_x| = 600mm 라는 불가능한 값을 고집했고, 나는 그게
데이터의 성질인 줄 알았다. 굴림만 떼어 재면 그 주장을 반증할 수 있다.
"""

from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np


def rot_x(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rpy(R):
    return (math.degrees(math.atan2(R[2, 1], R[2, 2])),
            math.degrees(-math.asin(max(-1.0, min(1.0, R[2, 0])))),
            math.degrees(math.atan2(R[1, 0], R[0, 0])))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?",
                    default=os.path.expanduser("~/roll_samples.json"))
    ap.add_argument("--tries", type=int, default=200000)
    ap.add_argument("--beta", type=float, default=-1.0,
                    help="광축과 굴림축이 이루는 각(도). 주면 그 원뿔 위만 훑는다 "
                         "— roll_scale_check 가 화면 기울기로 따로 잰 값")
    args = ap.parse_args()

    data = json.load(open(args.path, encoding="utf-8"))
    S = data["samples"]
    th = np.array([s["roll_deg"] for s in S])
    th = th - th.mean()
    P = np.array([np.mean([s["dots_mm"][k] for k in ("tl", "tr", "bl", "br")], axis=0)
                  for s in S])          # 무게중심 — 점 이름이 섞여도 같은 물리적 점
    n = len(S)
    print(f"굴림 표본 {n}개 · {th.min():+.1f}° ~ {th.max():+.1f}° "
          f"(폭 {np.ptp(th):.0f}°)")
    print(f"표적까지 거리 {np.linalg.norm(P, axis=1).min():.0f}~"
          f"{np.linalg.norm(P, axis=1).max():.0f}mm "
          f"(흔들림 {np.ptp(np.linalg.norm(P, axis=1)):.0f}mm)")

    # [Rot_x(-θ), -I] [u; t_x] = R_x p
    M = np.zeros((3 * n, 6))
    for i, t in enumerate(th):
        M[3 * i:3 * i + 3, 0:3] = rot_x(-t)
        M[3 * i:3 * i + 3, 3:6] = -np.eye(3)
    Mp = np.linalg.pinv(M)

    rng = np.random.default_rng(0)
    q = rng.normal(size=(args.tries, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    cone = math.cos(math.radians(args.beta)) if args.beta >= 0 else None
    best, tried = None, 0
    for a, b, c, d in q:
        R = np.array([
            [1 - 2 * (c * c + d * d), 2 * (b * c - d * a), 2 * (b * d + c * a)],
            [2 * (b * c + d * a), 1 - 2 * (b * b + d * d), 2 * (c * d - b * a)],
            [2 * (b * d - c * a), 2 * (c * d + b * a), 1 - 2 * (b * b + c * c)]])
        if cone is not None and abs(abs(R[0, 2]) - cone) > 0.015:
            continue
        tried += 1
        y = (P @ R.T).reshape(-1)
        sol = Mp @ y
        r = (M @ sol - y).reshape(-1, 3)
        rms = float(np.sqrt((r ** 2).sum(axis=1).mean()))
        if best is None or rms < best[0]:
            best = (rms, R, sol)

    rms, R, sol = best
    u, tx = sol[:3], sol[3:]
    beta = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(R[0, 2]))))))
    print(f"\n훑은 회전 {tried}개 · 잔차 RMS {rms:.2f}mm")
    print(f"  t_tool_cam = ({tx[0]:7.1f},{tx[1]:7.1f},{tx[2]:7.1f}) mm · |t| {np.linalg.norm(tx):.0f}mm")
    print("  R_x rpy = (%.1f, %.1f, %.1f)°" % rpy(R))
    print(f"  광축-굴림축 각 {beta:.1f}°  "
          f"(화면 기울기로 따로 잰 값과 맞아야 한다)")
    print(f"  표적까지 벡터(도구계, 굴림 0에서) = ({u[0]:.0f},{u[1]:.0f},{u[2]:.0f}) "
          f"· |u| {np.linalg.norm(u):.0f}mm")

    # 굴림축에서 카메라까지의 수직거리 — 이게 굴릴 때 카메라가 그리는 원의 반지름.
    print(f"  굴림축에서 카메라까지 {math.hypot(tx[1], tx[2]):.0f}mm "
          f"(축 방향으로는 {tx[0]:+.0f}mm)")
    if rms > 8.0:
        print("\n⚠ 굴림만으로도 안 맞는다 — 카메라 점이나 굴림 모델을 의심하라.")
    else:
        print("\n✅ 굴림만 보면 잘 맞는다 — R_x 와 t_x 는 이 값을 믿을 수 있다.")
        print("   전체 보정이 이와 크게 다른 답을 낸다면, 틀린 곳은 **팔의 위치 모델**이다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
