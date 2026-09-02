#!/usr/bin/env python3
"""**같은 관절값으로 돌아오면 카메라가 같은 것을 보는가.**

    ~/lerobot/.venv/bin/python ros2/tools/repeat_check.py --cycles 5

⚠ `tomato-voice.service`는 내리고, `depth-cam.service`는 켠 채로.
   표적(점 네 개)이 지금 보이는 자세에서 시작한다.

────────────────────────────────────────────────────────────────────────
왜 이것부터 재야 하는가

손-눈 보정은 "카메라가 팔에 단단히 붙어 있고, 관절값이 자세를 결정한다"를
전제한다. 그 전제가 몇 mm짜리인지 모르면 **잔차가 큰 게 모델 탓인지 기계 탓인지
알 수 없다** — 2026-09-01에 나는 그걸 모른 채 모델만 네 번 고쳐 봤다.

여기서 재는 것은 단 하나다: 출발 자세에서 표적을 보고 → 크게 한 바퀴 움직였다가
→ 같은 관절값으로 돌아와 다시 본다. 그 둘의 차이가 **이 로봇이 낼 수 있는 최고
성능의 하한**이다. 보정 잔차가 그 아래로 내려갈 수는 없다.

읽는 법:
  · 관절 읽기값은 같은데 본 것이 다르다 → 엔코더가 못 보는 곳이 흔들린다
    (브래킷·기어 백래시·나사). 모델을 고쳐도 안 줄어든다.
  · 관절 읽기값부터 다르다 → 서보가 그 자리로 못 돌아온다(부하·정지마찰).
  · 둘 다 작다 → 기계는 성하다. 잔차는 모델 탓이다.
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
SETTLE = 1.4
DOTS = ("tl", "tr", "bl", "br")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=5)
    ap.add_argument("--unidir", action="store_true",
                    help="모든 자세를 **같은 쪽에서** 접근한다 (백래시를 상수로 만든다)")
    ap.add_argument("--backoff", type=float, default=6.0,
                    help="단방향 접근에서 뒤로 물러났다 오는 크기(도). 백래시보다 커야 한다")
    ap.add_argument("--excursion", type=float, default=25.0,
                    help="한 바퀴 나갔다 오는 크기(도)")
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
    io.write(to_norm(home), 0.6)

    def move(target):
        cur = to_deg(io.read())
        big = max(abs(target[j] - cur[j]) for j in kin.JOINTS)
        steps = max(1, int(math.ceil(big / STEP_DEG)))
        for s in range(1, steps + 1):
            mid = {j: cur[j] + (target[j] - cur[j]) * s / steps for j in kin.JOINTS}
            if kin.forward(mid, geom).z < -MOUNT_Z_MM + FLOOR_MARGIN_MM:
                return None                     # 바닥에 닿는 경로는 안 간다
            io.write(to_norm(mid), 0.9)
        time.sleep(SETTLE)
        return to_deg(io.read())

    def approach(target):
        """목표 자세로 가되, **마지막 움직임의 방향을 항상 같게** 한다.

        ⚠ 이 팔에는 백래시가 있다. 2026-09-01 실측: 같은 관절값이라도 어느 쪽에서
          왔느냐에 따라 카메라가 본 것이 **30mm** 달랐다(한쪽 5mm, 반대쪽 30mm).
          백래시를 없앨 수는 없지만 **일정하게** 만들 수는 있다 — 늘 한쪽에서
          다가가면 톱니가 늘 같은 면에 닿는다. 그러면 그 오차는 보정이 흡수하는
          상수가 된다. CNC·3D프린터가 쓰는 그 방법이다.
        """
        if not args.unidir:
            return move(target)
        back = {j: target[j] - args.backoff for j in kin.JOINTS}
        if move(back) is None:
            return None
        return move(target)

    def look():
        try:
            m = tc.measure()
        except Exception:                       # noqa: BLE001
            return None
        if not m or not m.get("ok"):
            return None
        return np.mean([m["points_mm"][k] for k in DOTS], axis=0)

    # ⚠ 기준 관측도 **다른 자세들과 똑같은 방식으로 도착해서** 찍어야 한다.
    #   2026-09-01 실측: 기준만 `arm_stage`로(백오프 없이) 도착해 찍었더니,
    #   이후 단방향 접근으로 돌아온 관측들이 6회 내리 37~38mm 떨어져 나왔다.
    #   6회끼리는 1.2mm 안에서 일치했다 — 흩어짐이 아니라 **백래시 오프셋**이다.
    #   그걸 "채집 도중 뭔가 움직였다"로 읽고 멀쩡한 표본을 두 번 버렸다.
    approach(dict(home))
    base = look()
    if base is None:
        print("❌ 지금 자세에서 표적이 안 보인다 — 보이는 자세에서 시작하라.")
        io.hold_close()
        return 1
    print("출발 자세 " + " ".join(f"{j.split('_')[0]}={home[j]:7.2f}" for j in kin.JOINTS))
    print(f"표적 무게중심(카메라계) ({base[0]:.1f}, {base[1]:.1f}, {base[2]:.1f}) "
          f"· 거리 {np.linalg.norm(base):.1f}mm")
    print("\n한 바퀴 나갔다 돌아와 같은 것을 보는지 본다")
    print("  회차   관절 최대차(°)   본 것의 차이(mm)   거리(mm)")

    # 나갔다 오는 곳 — 관절마다 방향을 달리해 백래시를 양쪽에서 건드린다.
    away = [{"shoulder_pan": +1, "shoulder_lift": -1, "elbow_flex": +1,
             "wrist_flex": -1, "wrist_roll": +1},
            {"shoulder_pan": -1, "shoulder_lift": +1, "elbow_flex": -1,
             "wrist_flex": +1, "wrist_roll": -1}]

    seen, jerr = [base], []
    for k in range(args.cycles):
        d = dict(home)
        for j, s in away[k % 2].items():
            d[j] = home[j] + s * args.excursion
        if move(d) is None:
            print(f"  {k+1:3}   (나가는 길이 바닥에 걸려 건너뜀)")
            continue
        got = approach(dict(home))
        if got is None:
            print(f"  {k+1:3}   (돌아오는 길이 바닥에 걸려 건너뜀)")
            continue
        now = look()
        je = max(abs(got[j] - home[j]) for j in kin.JOINTS)
        jerr.append(je)
        if now is None:
            print(f"  {k+1:3}   {je:8.2f}        표적을 잃음")
            continue
        seen.append(now)
        print(f"  {k+1:3}   {je:8.2f}        {np.linalg.norm(now - base):8.2f}"
              f"          {np.linalg.norm(now):8.1f}")

    io.hold_close()

    if len(seen) < 3:
        print("\n❌ 표본이 모자라다.")
        return 1
    S = np.array(seen)
    spread = float(np.sqrt(((S - S.mean(axis=0)) ** 2).sum(axis=1).mean()))
    worst = float(max(np.linalg.norm(p - base) for p in S))
    print(f"\n같은 관절값에서 본 것의 흩어짐 RMS {spread:.2f}mm · 최대 {worst:.2f}mm")
    print(f"관절 읽기값의 되돌아옴 오차 최대 {max(jerr):.2f}°" if jerr else "")

    print("\n읽는 법 —")
    if spread < 3.0:
        print("  ✅ 기계는 성하다. 보정 잔차가 크다면 그건 **모델 탓**이다.")
    elif spread < 10.0:
        print("  ⚠ 이 정도면 보정 잔차 하한이 그만큼이다. 15mm 목표는 아직 가능하다.")
    else:
        print("  ❌ 같은 자리로 돌아와도 이만큼 다르게 본다 — **모델을 아무리 고쳐도**")
        print("     보정이 이 값 아래로 못 내려간다. 브래킷·나사·기어 유격을 먼저 잡아야 한다.")
    if jerr and max(jerr) > 2.0:
        print(f"  ⚠ 관절 읽기값부터 {max(jerr):.1f}° 어긋난다 — 서보가 그 자리로 못 돌아온다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
