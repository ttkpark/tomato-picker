#!/usr/bin/env python3
"""wrist_roll(5번)의 창(±100)을 옮긴다 — 필요한 방향이 창 밖일 때.
    ~/lerobot/.venv/bin/python ros2/tools/roll_rehome.py                 (읽기만)
    ~/lerobot/.venv/bin/python ros2/tools/roll_rehome.py --shift 1500    (present를 +1500틱 올린다)
    ~/lerobot/.venv/bin/python ros2/tools/roll_rehome.py --center        (지금 자리를 2048=정규 0으로)
⚠ `tomato-voice.service`를 내린 채로. 팔은 안 움직인다 — **숫자만** 바꾼다.
────────────────────────────────────────────────────────────────────────
왜 필요한가 — 2026-09-05 `lerobot-calibrate`가 5번 homing을 덮어써서, 카메라를
수평으로 세우는 자리가 정규값 창(-100..100) **밖**으로 나갔다(9/9 실측: 수평까지
-50이 더 필요한데 지금 자리가 -86). Present_Position = 절대엔코더 − Homing_Offset
이므로 homing을 N만큼 **줄이면** present가 N만큼 **오른다**. 창을 옮긴 뒤
`roll_step.py`로 조금씩 돌려 수평을 찾고, 거기서 `--center`로 그 자리를 2048로
박으면 실험 도구의 roll=0이 "수평"이 된다(`~/arm_cartesian.json` zero.wrist_roll=0).

⚠ homing 레지스터는 부호-크기 11비트(±2047). 그 밖은 거절한다.
⚠ `--center`는 lerobot의 set_half_turn_homings를 쓴다 — 그 함수가 먼저
   reset_calibration(범위 0..4095, homing 0)을 하므로 범위는 0..4095로 되돌려 쓴다
   (5번은 원래 0..4095다).
⚠ 둘 다 하드웨어와 JSON(`~/.cache/huggingface/lerobot/calibration/robots/so_follower/*.json`)을
   **같이** 바꾼다 — 둘이 어긋나면 다음 연결 때 JSON이 하드웨어를 덮어쓴다.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import os
import shutil
import sys
import time

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

CART = os.path.expanduser("~/arm_cartesian.json")
J = "wrist_roll"


def raw(bus, field: str):
    return bus.read(field, J, normalize=False)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--shift", type=int, default=None, help="present를 이만큼(틱) 올린다(음수면 내린다)")
    ap.add_argument("--center", action="store_true", help="지금 자리를 2048(정규 0)로 박고 zero.wrist_roll=0")
    args = ap.parse_args()
    if args.shift is not None and args.center:
        ap.error("--shift 와 --center 는 하나만")

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    io._connect()                      # noqa: SLF001
    robot = io._follower                # noqa: SLF001
    bus = robot.bus
    cal = robot.calibration[J]
    before = raw(bus, "Present_Position")
    print(f"지금: present={before} homing={raw(bus, 'Homing_Offset')} "
          f"limit={raw(bus, 'Min_Position_Limit')}..{raw(bus, 'Max_Position_Limit')} "
          f"JSON={cal}  정규={io.read()[J]:.2f}")

    if args.shift is None and not args.center:
        io.hold_close()
        return 0

    shutil.copy(robot.calibration_fpath, str(robot.calibration_fpath) + f".bak-{time.strftime('%m%d-%H%M%S')}")

    # ⚠ 토크를 켠 채 homing을 바꾸면 서보는 **옛 Goal(원시값)**을 향해 튄다 —
    #   present의 뜻이 바뀌었는데 goal은 그대로라서. 5번 토크를 끄고 숫자를 바꾼 뒤,
    #   goal을 새 present로 맞추고 나서 토크를 켠다(set_half_turn_homings도 쓰기를 한다).
    bus.disable_torque(J)
    if args.shift is not None:
        new_h = int(cal.homing_offset) - int(args.shift)
        if not (-2047 <= new_h <= 2047):
            print(f"⚠ homing {new_h}은 ±2047 밖 — 거절")
            io.hold_close()
            return 2
        want = before + args.shift
        if not (0 <= want <= 4095):
            print(f"⚠ 옮긴 뒤 present {want}가 0..4095 밖 — 거절")
            io.hold_close()
            return 2
        new_cal = dataclasses.replace(cal, homing_offset=new_h)
    else:
        offs = bus.set_half_turn_homings([J])
        new_h = int(offs[J])
        new_cal = dataclasses.replace(cal, homing_offset=new_h, range_min=0, range_max=4095)

    robot.calibration[J] = new_cal
    bus.write_calibration(robot.calibration)
    robot._save_calibration()          # noqa: SLF001
    time.sleep(0.2)
    after = raw(bus, "Present_Position")
    bus.write("Goal_Position", J, int(after), normalize=False)
    bus.enable_torque(J)
    time.sleep(0.3)
    after = raw(bus, "Present_Position")
    print(f"뒤:   present={after} homing={raw(bus, 'Homing_Offset')} "
          f"limit={raw(bus, 'Min_Position_Limit')}..{raw(bus, 'Max_Position_Limit')} "
          f"JSON={robot.calibration[J]}  정규={io.read()[J]:.2f}")
    if args.center:
        if os.path.exists(CART):
            shutil.copy(CART, CART + f".bak-{time.strftime('%m%d-%H%M%S')}")
            with open(CART, encoding="utf-8") as f:
                d = json.load(f)
            d.setdefault("zero", {})[J] = 0.0
            with open(CART, "w", encoding="utf-8") as f:
                json.dump(d, f, indent=2, ensure_ascii=False)
            print(f"{CART}: zero.{J} = 0.0")
        if abs(after - 2047) > 3:
            print(f"⚠ 중앙이 2047이 아니다({after}) — set_half_turn_homings 결과를 확인할 것")
    io.hold_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
