#!/usr/bin/env python3
"""서보가 왜 힘을 놓았는지 묻는다 — 부하·전압·온도·토크상태.

    ~/lerobot/.venv/bin/python ros2/tools/servo_diag.py            (읽기만)
    ~/lerobot/.venv/bin/python ros2/tools/servo_diag.py --hold     (토크 켜고 유지)

⚠ `tomato-voice.service`를 먼저 내려야 한다 — 팔 포트는 한 프로세스만 연다.

────────────────────────────────────────────────────────────────────────
왜 만들었나

팔을 뻗다가 **"갑자기 힘이 추욱 늘어졌다"**. 서서히 못 따라간 게 아니라 한순간에
토크가 사라진 것이고, 그건 STS3215의 **과부하 보호**가 스스로 토크를 끈 증상이다
(부하가 한계를 넘은 채 일정 시간이 지나면 서보가 토크를 내려놓는다).

그 가설을 숫자로 확인한다:
  · `Torque_Enable`이 0이면 → **서보가 스스로 껐다**(우리가 켰는데도 0이면 확정)
  · `Present_Load`가 한계 근처면 → 그 자세가 이 팔에 무리다
  · `Present_Voltage`가 떨어졌으면 → 토크가 아니라 **전원** 문제다
  · `Present_Temperature`가 높으면 → 과열 보호

⚠ 원인이 셋(과부하·전원·과열)이고 증상이 하나라서, 숫자를 안 보면 셋 다
   그럴듯해 보인다. 이 프로젝트가 반복해서 배운 것 — **게인을 만지기 전에 재라.**
"""

from __future__ import annotations

import argparse
import os
import sys
import time

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

from tomato_picker.hardware import kinematics as kin  # noqa: E402

# 이름이 펌웨어/SDK 판에 따라 조금씩 달라서 후보를 순서대로 시도한다.
FIELDS = (
    ("Torque_Enable", "토크"),
    ("Present_Load", "부하"),
    ("Present_Current", "전류"),
    ("Present_Voltage", "전압"),
    ("Present_Temperature", "온도"),
    ("Present_Position", "위치"),
    ("Goal_Position", "목표"),
    ("Moving", "이동중"),
)


def read_all(bus, joints):
    out = {}
    for reg, label in FIELDS:
        try:
            vals = bus.sync_read(reg)
        except Exception:
            try:
                vals = {j: bus.read(reg, j) for j in joints}
            except Exception as exc:  # noqa: BLE001
                out[label] = f"(못 읽음: {type(exc).__name__})"
                continue
        out[label] = {j: vals.get(j) for j in joints}
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold", action="store_true",
                    help="토크를 켜고 현재 자세를 목표로 잡아 유지한다")
    args = ap.parse_args()

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    io._connect()                      # noqa: SLF001 - 진단이라 내부를 직접 쓴다
    bus = io._follower.bus             # noqa: SLF001
    joints = list(kin.JOINTS) + ["gripper"]

    print("=== 지금 상태 (연결 직후 — FollowerIO는 연결 시 토크를 끈다) ===")
    for label, vals in read_all(bus, joints).items():
        print(f"  {label:<6} {vals}")

    if not args.hold:
        print("\n(--hold 를 주면 토크를 켜고 그 자리를 잡는다)")
        io.hold_close()
        return 0

    print("\n=== 토크를 켜고 현재 자세를 목표로 준다 ===")
    now = io.read()
    bus.enable_torque()
    io.write(now, 0.6)                 # 지금 자리를 목표로 → 그 자세를 붙든다
    time.sleep(1.0)

    print("=== 유지 중 상태 ===")
    after = read_all(bus, joints)
    for label, vals in after.items():
        print(f"  {label:<6} {vals}")

    te = after.get("토크")
    if isinstance(te, dict):
        off = [j for j, v in te.items() if v in (0, False)]
        if off:
            print(f"\n⚠ 토크를 켰는데도 꺼져 있는 관절: {off}")
            print("   → **서보가 스스로 껐다.** 과부하/과열 보호다. "
                  "그 자세가 이 팔에 무리라는 뜻이므로 더 접은 자세로 가야 한다.")
        else:
            print("\n토크가 전 관절에서 켜져 있다 — 이 자세는 버틴다.")

    print("\n30초간 부하를 지켜본다 (5초마다)")
    for _ in range(6):
        time.sleep(5.0)
        snap = read_all(bus, joints)
        print(f"  부하 {snap.get('부하')}  전압 {snap.get('전압')}  "
              f"온도 {snap.get('온도')}  토크 {snap.get('토크')}")

    print("\n⚠ 토크를 켠 채로 둔다 — 여기서 끄면 팔이 그 자리에서 떨어진다.")
    io.hold_close()   # ⚠ disconnect()는 닫으면서 토크를 끈다 — 팔이 떨어진다
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
