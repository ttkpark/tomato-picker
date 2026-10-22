#!/usr/bin/env python3
"""케이스 회전판(base_yaw, 팔 버스 ID 7)만 돌린다 — 팔 관절은 안 건드린다.

    ~/lerobot/.venv/bin/python ros2/tools/base_yaw.py 45      # 왼쪽 45°
    ~/lerobot/.venv/bin/python ros2/tools/base_yaw.py 0       # 정면
    ~/lerobot/.venv/bin/python ros2/tools/base_yaw.py --read

각도 규약·한계는 config.BASE_YAW_* 와 follower_io.yaw_raw_from_deg 가 정한다.
⚠ 토크를 **켠 채로** 포트를 닫는다(`hold_close`) — grip_set.py와 같은 이유.
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))


def main() -> int:
    from tomato_bridge.follower_io import FollowerIO, yaw_raw_from_deg
    read_only = sys.argv[1:2] == ["--read"]
    if not read_only:
        try:
            deg = float(sys.argv[1])
            yaw_raw_from_deg(deg)               # 포트 열기 전에 한계부터 거절
        except (IndexError, ValueError) as exc:
            print("쓰기: base_yaw.py <도> | base_yaw.py --read  —", exc)
            return 2
    io = FollowerIO(hold_torque=True)
    try:
        if not read_only:
            io.move_yaw(deg)
        print("base_yaw 지금 %.1f°" % io.read_yaw())
    finally:
        io.hold_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
