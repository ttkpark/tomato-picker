#!/usr/bin/env python3
"""표적이 **보이는 자세를 찾는다** — 관절 하나를 훑으며 매 정거장에서 세어 본다.

    ~/lerobot/.venv/bin/python ros2/tools/target_sweep.py --joint wrist_flex
    ... --joint shoulder_lift --from -10 --to 60 --step 10

⚠ `tomato-voice.service`는 내리고, `depth-cam.service`는 **켠 채로**
   (이 도구는 그 서비스가 `/dev/shm`에 쓰는 프레임을 읽는다).

────────────────────────────────────────────────────────────────────────
왜 이 도구가 필요한가

손-눈 보정은 "카메라가 어디를 보는가"를 **푸는** 절차다. 그러니 그걸 풀기 전에
그 답을 알고 있다고 가정하면 안 된다 — 2026-08-31, FK가 "거의 수평"이라고 한
자세에서 카메라는 바닥을 보고 있었다. 어느 쪽이 틀렸는지는 보정이 끝나야 안다.

그래서 추론 대신 **훑는다.** 관절 하나를 조금씩 돌리며 매번 점을 세고, 4개가
다 보이는 구간을 찾는다. 그 자세가 `handeye_collect.py --home here`의 출발점이다.

⚠ 사람이 보는 앞에서만 — 손목에 카메라와 USB3 케이블이 달려 있다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

import target_check as tc  # noqa: E402
from target_check import COLOR, find_dots, order_dots  # noqa: E402
from tomato_picker.hardware import kinematics as kin  # noqa: E402

DEG_PER_TICK = 360.0 / 4096.0
CAL = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json")
CART = os.path.expanduser("~/arm_cartesian.json")

# 바닥은 팔 base(마운트)보다 이만큼 아래에 있다 — 실측 76.5mm
# (`ros2/src/tomato_description/config/so101_geometry.yaml` 의 mount.z 와 같은 값).
# ⚠ 예전에는 "지금 자리보다 1mm 아래"를 바닥으로 삼았다. 그러면 팔이 낮게
#   늘어져 있을 때 **1.8mm 내려갔다 다시 오르는 정상 경로까지 막혀** 빠져나올
#   수가 없다(2026-09-01, 복구 불가 상태로 두 번 갇혔다). 바닥은 팔이 어디
#   있느냐와 무관한 값이다.
MOUNT_Z_MM = 76.5
FLOOR_MARGIN_MM = 10.0
STEP_SECS = 1.2
SETTLE = 1.0        # 서비스가 새 프레임을 쓰고 흔들림이 멎을 때까지
NORM_LIMIT = 98.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="wrist_flex", choices=list(kin.JOINTS))
    ap.add_argument("--from", dest="lo", type=float, default=None)
    ap.add_argument("--to", dest="hi", type=float, default=None)
    ap.add_argument("--step", type=float, default=15.0)
    ap.add_argument("--save", default="/dev/shm/sweep",
                    help="정거장마다 사진을 남길 접두사 (빈 값이면 안 남김)")
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

    # 이 관절이 실제로 갈 수 있는 도(度) 범위 — 정규화 -100..100에서 나온다.
    # ⚠ 교시 자세를 중심으로 대칭이 아니다.
    j = args.joint
    ends = sorted(ref.get(j, 0.0) + sign(j) * (e - zero.get(j, 0.0)) * per(j)
                  for e in (-NORM_LIMIT, NORM_LIMIT))
    lo = ends[0] if args.lo is None else max(ends[0], args.lo)
    hi = ends[1] if args.hi is None else min(ends[1], args.hi)

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    now = to_deg(io.read())
    io.write(to_norm(now), 0.4)          # 붙자마자 붙든다 (안 그러면 처진다)

    print(f"{j} 를 {lo:.1f}° ~ {hi:.1f}° 까지 {args.step:.0f}°씩 훑는다 "
          f"(지금 {now[j]:.1f}°)")
    print("  각도    TCP(x,y,z)            pitch    점  비고")

    # 지금 값에서 가까운 쪽 끝으로 먼저 가고 반대편까지 — 헛걸음을 줄인다.
    first, last = (lo, hi) if abs(now[j] - lo) <= abs(now[j] - hi) else (hi, lo)
    stops, v = [], first
    stepv = math.copysign(args.step, last - first)
    while (stepv > 0 and v <= last + 1e-6) or (stepv < 0 and v >= last - 1e-6):
        stops.append(round(v, 2))
        v += stepv

    best = []
    for v in stops:
        d = dict(now)
        d[j] = v
        p = kin.forward(d, geom)
        note = ""
        if p.z < -MOUNT_Z_MM + FLOOR_MARGIN_MM:
            note = "바닥아래 — 건너뜀"
            print(f"  {v:6.1f}  ({p.x:6.1f},{p.y:5.1f},{p.z:6.1f})  {p.pitch:7.1f}   -  {note}")
            continue
        io.write(to_norm(d), STEP_SECS)
        time.sleep(SETTLE)
        got = to_deg(io.read())
        gp = kin.forward(got, geom)
        err = abs(got[j] - v)

        bgr = cv2.imread(COLOR)
        dots = find_dots(bgr) if bgr is not None else []
        # ⚠ "넷을 찾았다"가 "표적을 찾았다"는 뜻이 아니다. 잡동사니 많은 방에서는
        #   어두운 덩이 넷이 아무렇게나 잡힌다(2026-09-01: 책상·상자를 표적으로
        #   읽었다). 아는 직사각형과 맞는지까지 봐야 한다.
        shaped = False
        if len(dots) == 4:
            try:
                m = tc.measure()
                shaped = bool(m and m.get("ok"))
            except Exception:                     # noqa: BLE001
                shaped = False
        n = len(dots) if shaped else 0
        if args.save:
            path = f"{args.save}_{j}_{v:+07.1f}.jpg"
            if bgr is not None:
                out = bgr.copy()
                for (dx, dy) in dots:
                    cv2.circle(out, (int(dx), int(dy)), 12, (0, 0, 255), 2)
                cv2.imwrite(path, out, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if n == 4:
            note = "★ 넷 다 보인다"
            best.append((v, dots))
        elif len(dots):
            note = f"{len(dots)}개 보이나 표적 모양이 아니다"
        if err > 4.0:
            note += f"  ⚠ 추종오차 {err:.1f}°"
        print(f"  {v:6.1f}  ({gp.x:6.1f},{gp.y:5.1f},{gp.z:6.1f})  {gp.pitch:7.1f}  {n:2}  {note}")

    if best:
        mid = best[len(best) // 2][0]
        print(f"\n✅ 넷 다 보이는 구간: {[b[0] for b in best]}")
        print(f"   가운데 {mid:.1f}° 로 가 둔다 — 여기가 보정의 출발점이다.")
        d = dict(now)
        d[j] = mid
        io.write(to_norm(d), STEP_SECS)
        time.sleep(SETTLE)
        final = to_deg(io.read())
        print("   " + " ".join(f"{k.split('_')[0]}={final[k]:.1f}" for k in kin.JOINTS))
    else:
        print("\n❌ 이 관절만으로는 표적이 안 보인다 — 다른 관절도 훑어야 한다.")
    print("⚠ 토크를 켠 채로 둔다.")
    io.hold_close()
    return 0 if best else 1


if __name__ == "__main__":
    raise SystemExit(main())
