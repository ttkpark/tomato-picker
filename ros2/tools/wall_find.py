#!/usr/bin/env python3
"""**깊이로 벽을 찾는다** — 카메라가 어느 굴림각에서 벽을 정면으로 보는가.

    ~/lerobot/.venv/bin/python ros2/tools/wall_find.py --joint wrist_roll

⚠ `tomato-voice`는 내리고 `depth-cam`은 켠 채로.

────────────────────────────────────────────────────────────────────────
왜 색이 아니라 깊이인가

2026-09-01, 표적을 다시 찾느라 손목 화면을 여러 장 눈으로 보며 관절을 맞혀
봤는데 계속 빗나갔다. 이 팔의 카메라는 **집게 축과 거의 90° 틀어져** 달려
있어서, 집게가 향한 곳과 카메라가 보는 곳이 사람 직관과 다르다.

깊이는 그 직관이 필요 없다. 벽을 정면으로 보면 화면 대부분이 **가깝고 균일한
한 평면**이 된다(광축과 이루는 각이 작다). 방을 가로질러 보면 깊이가 넓게
퍼진다. 그 차이는 숫자 하나로 갈린다.
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
SETTLE = 1.2
NEAR_MM = 700.0


def scene():
    """지금 깊이 프레임의 요약 — (가까운 화소 비율, 중앙값 거리, 평면 정도)."""
    try:
        depth = np.load(tc.DEPTH).astype(float)
        meta = json.load(open(tc.META))
    except Exception:                              # noqa: BLE001
        return None
    d = depth[depth > 0] * float(meta.get("depth_scale_mm", 1.0))
    if d.size < 500:
        return None
    near = float((d < NEAR_MM).mean())
    med = float(np.median(d))
    # 균일함 — 벽을 정면으로 보면 깊이의 사분위폭이 좁다
    iqr = float(np.percentile(d, 75) - np.percentile(d, 25))
    return near, med, iqr


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="wrist_roll", choices=list(kin.JOINTS))
    ap.add_argument("--from", dest="lo", type=float, default=None)
    ap.add_argument("--to", dest="hi", type=float, default=None)
    ap.add_argument("--step", type=float, default=25.0)
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

    j = args.joint
    ends = sorted(ref.get(j, 0.0) + sign(j) * (e - zero.get(j, 0.0)) * per(j)
                  for e in (-98.0, 98.0))
    lo = ends[0] if args.lo is None else max(ends[0], args.lo)
    hi = ends[1] if args.hi is None else min(ends[1], args.hi)

    print(f"{j} 를 {lo:.0f}° ~ {hi:.0f}° 훑으며 **깊이로** 벽을 찾는다")
    print("  각도    가까운화소%  중앙거리mm  사분위폭mm  점  비고")

    stops, v = [], lo
    while v <= hi + 1e-6:
        stops.append(round(v, 1))
        v += args.step

    best, rows = None, []
    for v in stops:
        d = dict(home)
        d[j] = v
        p = kin.forward(d, geom)
        if p.z < -MOUNT_Z_MM + FLOOR_MARGIN_MM:
            continue
        cur = to_deg(io.read())
        big = max(abs(d[k] - cur[k]) for k in kin.JOINTS)
        for st in range(1, max(1, int(math.ceil(big / STEP_DEG))) + 1):
            mid = {k: cur[k] + (d[k] - cur[k]) * st / max(1, int(math.ceil(big / STEP_DEG)))
                   for k in kin.JOINTS}
            io.write(to_norm(mid), 0.9)
        time.sleep(SETTLE)
        s = scene()
        if s is None:
            print(f"  {v:6.1f}   (깊이를 못 읽음)")
            continue
        near, med, iqr = s
        try:
            m = tc.measure()
            n = 4 if (m and m.get("ok")) else 0
        except Exception:                          # noqa: BLE001
            n = 0
        rows.append((near, v))
        mark = "★ 표적!" if n == 4 else ("벽 같다" if near > 0.6 and iqr < 250 else "")
        print(f"  {v:6.1f}   {near*100:8.0f}   {med:9.0f}   {iqr:9.0f}  {n:2}  {mark}")
        if n == 4 and best is None:
            best = v

    if best is None and rows:
        rows.sort(reverse=True)
        best = rows[0][1]
        print(f"\n표적은 못 봤지만 가장 벽 같은 곳은 {best:.1f}° 다 — 거기 가 둔다")
    if best is not None:
        d = dict(home)
        d[j] = best
        cur = to_deg(io.read())
        big = max(abs(d[k] - cur[k]) for k in kin.JOINTS)
        steps = max(1, int(math.ceil(big / STEP_DEG)))
        for st in range(1, steps + 1):
            io.write(to_norm({k: cur[k] + (d[k] - cur[k]) * st / steps
                              for k in kin.JOINTS}), 0.9)
    io.hold_close()
    print("⚠ 토크를 켠 채로 둔다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
