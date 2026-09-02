#!/usr/bin/env python3
"""집게만 여닫는다 — 팔은 안 건드린다.

    ~/lerobot/.venv/bin/python ros2/tools/grip_set.py 78     # 연다
    ~/lerobot/.venv/bin/python ros2/tools/grip_set.py 4      # 닫는다

⚠ 토크를 **켠 채로** 포트를 닫는다(`hold_close`). 그냥 disconnect 하면
  토크가 풀려 팔이 주저앉는다 — 2026-09-02에 그것 때문에 자세가 74° 무너졌다.
"""

from __future__ import annotations

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "tomato_bridge"))


def main() -> int:
    try:
        v = float(sys.argv[1])
    except (IndexError, ValueError):
        print("쓰기: grip_set.py <0~100>")
        return 2
    v = max(0.0, min(100.0, v))
    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    try:
        n = io.read()
        n["gripper"] = v
        io.write(n, 0.9)
        time.sleep(1.1)
        print("집게 %.0f → 지금 %.1f" % (v, float(io.read().get("gripper", -1))))
    finally:
        io.hold_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
