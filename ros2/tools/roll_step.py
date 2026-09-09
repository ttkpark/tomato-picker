#!/usr/bin/env python3
"""wrist_roll(5번)만 **작게·천천히** 돌린다 — 카메라 수평을 눈으로 맞출 때.
    ~/lerobot/.venv/bin/python ros2/tools/roll_step.py --delta 10     (정규값 +10 = +10°)
    ~/lerobot/.venv/bin/python ros2/tools/roll_step.py --to -36       (정규값 절대치)
    ~/lerobot/.venv/bin/python ros2/tools/roll_step.py                (읽기만)
⚠ `tomato-voice.service`를 내린 채로. 나머지 관절은 지금 자리를 붙든다.
────────────────────────────────────────────────────────────────────────
왜 따로 있나 — 2026-09-05에 롤을 "도(度)"로 시켰다가 157° 점프가 나가
과부하 잠금(OverEle)에 걸리고 팔이 떨어졌다. 정규값 1단위 = 1.8°(4096틱=200단위)
이고 `~/arm_cartesian.json`의 deg_per_norm.wrist_roll(0.999)은 그 값이 아니다.
그래서 이 도구는 **정규값으로만**, 한 번에 |delta| ≤ 15°, 창(±180°)
안에서만 움직이고, 움직인 뒤 5번의 토크가 살아 있는지 확인한다.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

# ⚠ 9/9 실측: 이 팔의 wrist_roll 정규값은 **도(度)**다(raw 3010 → 84.6 = (3010−2048)·360/4096).
#   창은 ±180. 메모리의 "1.8°/단위"는 ±100 틀 가정이라 이 모터엔 맞지 않는다.
MAX_STEP = 15.0          # 정규값(도). 그 이상은 두 번 부른다.
LIMIT = 175.0            # 창 끝 5도 앞에서 멈춘다(raw 0/4095 랩 근처를 피한다)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--delta", type=float, default=None, help="정규값 상대 이동")
    ap.add_argument("--to", type=float, default=None, help="정규값 절대 이동")
    ap.add_argument("--secs", type=float, default=1.2)
    args = ap.parse_args()
    if args.delta is not None and args.to is not None:
        ap.error("--delta 와 --to 는 하나만")

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    io._connect()                      # noqa: SLF001
    bus = io._follower.bus             # noqa: SLF001
    now = io.read()
    roll = now["wrist_roll"]
    print(f"지금 wrist_roll = {roll:.2f} (정규값=도, 1단위=11.4틱)   전체 {now}")

    target = None
    if args.delta is not None:
        target = roll + args.delta
    elif args.to is not None:
        target = args.to
    if target is None:
        io.hold_close()
        return 0

    step = target - roll
    if abs(step) > MAX_STEP + 1e-6:
        print(f"⚠ 한 번에 {step:+.1f}은 너무 크다(|{MAX_STEP}| 이하). 나눠 부른다.")
        io.hold_close()
        return 2
    if not (-LIMIT <= target <= LIMIT):
        print(f"⚠ 목표 {target:.1f}이 창(±{LIMIT}) 밖 — 이 방향은 홈 오프셋을 옮겨야 한다.")
        io.hold_close()
        return 3

    goal = dict(now)
    goal["wrist_roll"] = target
    bus.enable_torque()
    io.write(goal, max(0.6, args.secs))
    time.sleep(0.4)
    after = io.read()
    try:
        te = bus.read("Torque_Enable", "wrist_roll")
    except Exception as exc:  # noqa: BLE001
        te = f"? ({exc})"
    err = after["wrist_roll"] - target
    print(f"이동 후 wrist_roll = {after['wrist_roll']:.2f}  (목표 {target:.2f}, 오차 {err:+.2f})  토크={te}")
    if te in (0, False):
        print("⚠ 5번 토크가 꺼졌다 — 하드스톱/케이블에 걸렸을 가능성. 반대로 조금 돌려 풀어라.")
        io.hold_close()
        return 4
    if abs(err) > 3.0:
        print("⚠ 목표에 못 갔다(추종 오차 3 초과) — 막힌 것일 수 있다. 더 밀지 말 것.")
    io.hold_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
