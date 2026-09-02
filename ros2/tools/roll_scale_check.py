#!/usr/bin/env python3
"""`wrist_roll`이 **정말** 그만큼 도는지 카메라로 직접 잰다.

    ~/lerobot/.venv/bin/python ros2/tools/roll_scale_check.py

⚠ `tomato-voice.service`는 내리고, `depth-cam.service`는 **켠 채로**.
   표적(점 네 개)이 지금 화면에 보이는 자세에서 시작해야 한다.

────────────────────────────────────────────────────────────────────────
왜 이 도구가 있는가

손-눈 보정의 잔차가 안 떨어져서 원자료를 뜯어 보니, 회전 오차 5.3° 중 대부분이
`wrist_roll` 하나에서 나왔다. 그 관절의 각도에 **0.63을 곱하면** 오차가 1.9°로
떨어진다 — 즉 서보가 "150° 돌았다"고 말할 때 카메라는 95°쯤만 돌았다는 뜻이다.

그 주장은 보정을 거치지 않고 **직접** 확인할 수 있다. 카메라가 자기 광축 둘레로
도는 양은 곧 **화면 속 표적이 도는 각도**다. 그래서 손목을 조금씩 굴리며 표적의
기울기를 재고, 지령한 각도와 견준다. 기울기가 1:1이면 눈금은 맞는 것이고,
0.63:1이면 눈금이 틀린 것이다.

이 저장소의 규칙 그대로다 — **게인을 만지기 전에 그 크기가 실제로 움직이는지 재라.**

⚠ 표적의 기울기는 180°마다 같아 보인다(직사각형이라 점 이름이 섞인다). 그래서
   주성분 방향으로 재고, 한 걸음이 90°를 넘지 않게 잘게 나눠 이어 붙인다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

import target_check as tc  # noqa: E402
from target_check import COLOR, find_dots  # noqa: E402
from tomato_picker.hardware import kinematics as kin  # noqa: E402

DEG_PER_TICK = 360.0 / 4096.0
CAL = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json")
CART = os.path.expanduser("~/arm_cartesian.json")
SETTLE = 1.2
JOINT = "wrist_roll"


def pattern_angle(dots) -> float | None:
    """화면 속 네 점이 이루는 무늬의 기울기(도) — 주성분 방향. 180° 모호.

    점 이름에 기대지 않는다. 손목이 구르면 `order_dots`의 좌상/우상이 다른
    물리적 점을 가리키므로, 이름을 쓰면 여기서부터 이미 틀린다.
    """
    if len(dots) != 4:
        return None
    P = np.array(dots, dtype=float)
    P = P - P.mean(axis=0)
    vt = np.linalg.svd(P)[2]
    return math.degrees(math.atan2(vt[0][1], vt[0][0]))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=12.0,
                    help="한 걸음의 지령 각도(도). 90°를 넘으면 이어 붙이기가 깨진다")
    ap.add_argument("--span", type=float, default=96.0, help="양쪽으로 이만큼까지")
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

    print(f"보정표의 {JOINT}: 가동폭 {spans.get(JOINT, 0):.1f}° "
          f"→ {per(JOINT):.4f}°/정규화 (4096틱=360°라는 가정에서 나온 값)")

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    home = to_deg(io.read())
    io.write(to_norm(home), 0.5)

    lo = ref.get(JOINT, 0.0) + sign(JOINT) * (-98.0 - zero.get(JOINT, 0.0)) * per(JOINT)
    hi = ref.get(JOINT, 0.0) + sign(JOINT) * (98.0 - zero.get(JOINT, 0.0)) * per(JOINT)
    lo, hi = min(lo, hi), max(lo, hi)

    offs = [0.0]
    v = args.step
    while v <= args.span + 1e-6:
        offs += [v, -v]
        v += args.step
    offs = sorted(o for o in offs if lo + 1.0 <= home[JOINT] + o <= hi - 1.0)

    print(f"{JOINT} 를 {home[JOINT]:.1f}° 에서 {offs[0]:+.0f}~{offs[-1]:+.0f}° 훑는다")
    print("  지령(도)  실제(도)   화면 기울기(도)  점")

    obs, dump = [], []
    for o in offs:
        d = dict(home)
        d[JOINT] = home[JOINT] + o
        io.write(to_norm(d), 1.0)
        time.sleep(SETTLE)
        got = to_deg(io.read())
        bgr = cv2.imread(COLOR)
        dots = find_dots(bgr) if bgr is not None else []
        ang = pattern_angle(dots)
        print(f"  {o:+8.1f}  {got[JOINT] - home[JOINT]:+8.1f}   "
              + (f"{ang:+8.2f}" if ang is not None else "      --")
              + f"      {len(dots)}")
        if ang is not None:
            obs.append((got[JOINT] - home[JOINT], ang))
        # ⚠ 3D 점도 남긴다. 굴림만 바꾼 표본은 **기구학과 무관하게** R_x와 t_x를
        #   준다 — 팔의 나머지가 고정이라 위치 모델이 식에서 통째로 빠지기 때문이다.
        #   손-눈 보정이 안 풀릴 때 "카메라가 도구에서 얼마나 벗어나 있나"를
        #   따로 재는 유일한 길이다.
        try:
            m = tc.measure()
        except Exception:                     # noqa: BLE001
            m = None
        if m and m.get("ok"):
            dump.append({"roll_deg": float(got[JOINT]),
                         "joints_deg": {k: float(v) for k, v in got.items()},
                         "dots_mm": {k: [float(c) for c in v]
                                     for k, v in m["points_mm"].items()},
                         "image_angle_deg": float(ang)})

    io.write(to_norm(home), 1.0)
    io.hold_close()

    path = os.path.expanduser("~/roll_samples.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"joint": JOINT, "home_deg": home, "samples": dump},
                  fh, ensure_ascii=False, indent=1)
    print(f"원자료 {path} ({len(dump)}개)")

    if len(obs) < 4:
        print("\n❌ 표본이 모자라다 — 표적이 계속 보이는 자세에서 다시.")
        return 1

    # 180° 모호를 푼다: 지령 순서대로 정렬해 이웃과 90° 넘게 벌어지면 180°를 더한다.
    obs.sort(key=lambda p: p[0])
    cmds = np.array([p[0] for p in obs])
    meas = np.array([p[1] for p in obs])
    for i in range(1, len(meas)):
        while meas[i] - meas[i - 1] > 90.0:
            meas[i] -= 180.0
        while meas[i] - meas[i - 1] < -90.0:
            meas[i] += 180.0

    # ⚠ 180° 모호가 한 점이라도 잘못 풀리면 직선 맞춤이 통째로 끌려간다
    #   (2026-09-01 실측: 한 점이 180° 튀어 기울기가 0.55 → 0.449로 읽혔다).
    #   그래서 맞춘 뒤 크게 벗어난 점을 빼고 다시 맞춘다.
    keep = np.ones(len(cmds), dtype=bool)
    slope = icept = 0.0
    for _ in range(3):
        slope, icept = np.polyfit(cmds[keep], meas[keep], 1)
        r = np.abs(meas - (slope * cmds + icept))
        lim = max(6.0, 3.0 * float(np.median(r[keep])))
        new = r <= lim
        if new.sum() < 4 or (new == keep).all():
            break
        keep = new
    if (~keep).any():
        print(f"(직선에서 크게 벗어난 {int((~keep).sum())}점을 뺐다 — 180° 모호가 "
              "잘못 풀린 것)")
    resid = float(np.sqrt(np.mean((meas[keep] - (slope * cmds[keep] + icept)) ** 2)))
    print(f"\n화면 기울기 = {slope:+.4f} x 지령 + {icept:.1f}  (맞춤 잔차 {resid:.2f}°)")
    print(f"기울기의 크기 {abs(slope):.3f} — 1.000 이면 눈금이 맞다")

    # ⚠ 화면이 도는 각도가 관절 각도와 1:1인 것은 **광축이 굴림축과 나란할
    #   때뿐**이다. 기울기 = k · cos β 이고, k(실제 회전 / 서보가 말하는 회전)와
    #   β(광축이 굴림축에서 기운 각)를 이 측정 하나로는 못 가른다.
    #   2026-09-01: 여기서 0.549가 나왔고 나는 "β=57°인 정상"으로 읽었는데,
    #   `joint_axis_check.py`가 3D로 재 보니 β=14°에 **k=0.555**였다.
    #   즉 눈금 쪽이 범인이었다. 이 도구만으로 단정하지 말 것 —
    #   **`joint_axis_check.py`와 같이 읽어야** 둘이 갈린다.
    beta1 = math.degrees(math.acos(max(0.0, min(1.0, abs(slope)))))
    print(f"→ 이 값 하나로는 못 가른다: k=1이면 광축이 {beta1:.0f}° 기운 것이고,")
    print(f"   광축이 나란하다면(β=0) 눈금이 {abs(slope):.3f}배인 것이다.")
    print("   3D로 가르려면 joint_axis_check.py --joint wrist_roll 을 보라.")
    return 0
    beta = math.degrees(math.acos(max(0.0, min(1.0, abs(slope)))))
    print(f"→ 광축이 굴림축에서 약 {beta:.0f}° 기울어 있다는 뜻이다 (cos {beta:.0f}° = {abs(slope):.2f}).")
    print("  브래킷이 기울어 달렸다면 이게 정상이며, **눈금 오류가 아니다** —")
    print("  그 기울기는 손-눈 보정의 R_x가 담는 값이다.")
    print("  눈금을 의심하려면 화면 기울기가 아니라 **팔 전체가 그리는 호**를 재야 한다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
