#!/usr/bin/env python3
"""**바퀴 톡톡** — 최대속도로 아주 짧게 끊어 치고, 안 움직이면 조금씩 길게. 원하는 만큼 갈 때까지.

    ~/lerobot/.venv/bin/python ros2/tools/base_tap.py --x 30      # 앞으로 30mm
    ~/lerobot/.venv/bin/python ros2/tools/base_tap.py --y -40     # 오른쪽으로 40mm (vy+ = 왼쪽)
    ~/lerobot/.venv/bin/python ros2/tools/base_tap.py --w 10      # 왼쪽(반시계)으로 10°

⚠ 연속 저속은 정지마찰에 0이다(이 로봇 1번 병). 사람 키보드 조작은 255로 짧게 쳐서 3cm 단위가 됐다(11/02).
   그래서 크기는 255로 고정하고 **시간**만 0.06s부터 ×1.35씩 늘린다 — 움직인 걸음이 나오면 그 길이를 유지.
⚠ 얼마 갔는지는 **잰다**: 앞뒤 = 손목 D405로 본 노란 띠 깊이 변화, 옆 = 띠 화면 이동(×z/fx), 회전 = IMU.
   팔을 시작 자세(앞을 봄)에 두고 쓴다. 띠가 안 보이면 앞뒤·옆은 멈춘다.
⚠ 주행 포트는 한 프로세스 — 한 번 열어 끝까지 쓴다(열 때마다 Uno가 리셋된다).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

IMU = "/dev/shm/imu.json"
FAST = 255


def yaw():
    try:
        d = json.load(open(IMU))
        return float(d["yaw_deg"]) if time.time() - float(d.get("ts", 0)) < 2.0 else None
    except (OSError, ValueError, KeyError):
        return None


def look(axis):
    """축에 맞는 '지금 값' — x: 띠 깊이(mm), y: 띠 u·z/fx(mm), w: IMU 요(도)."""
    if axis == "w":
        return yaw()
    import grasp_repeat as gr
    import grip_uv_measure as gm
    time.sleep(0.35)                     # 멈춘 뒤 프레임이 새로 오게
    vals = [b for b in (gr.band() for _ in range(3)) if b]
    if not vals:
        return None
    u = sorted(b[0] for b in vals)[len(vals) // 2]
    z = sorted(b[2] for b in vals)[len(vals) // 2]
    if axis == "x":
        return z
    fx = float(json.load(open(gm.META))["intrinsics"]["fx"])
    return u * z / fx


def gone_by(axis, before, after):
    """지령 방향으로 간 양(+). x: 앞으로 가면 띠가 가까워진다. y: 왼쪽으로 가면 띠가 화면 오른쪽으로."""
    d = after - before
    if axis == "x":
        return -d / 0.8                      # 카메라가 숙여 있어 수평 1mm에 깊이 ≈0.8mm
    if axis == "w":
        return (d + 180.0) % 360.0 - 180.0
    return d


def tap_move(link, axis, target, secs0=0.06, grow=1.35, max_secs=0.5, max_taps=25, moved_min=None):
    import base_nudge as bn
    sign = 1 if target > 0 else -1
    moved_min = moved_min or (0.7 if axis == "w" else 3.0)
    vec = {"x": (FAST * sign, 0, 0), "y": (0, FAST * sign, 0), "w": (0, 0, FAST * sign)}[axis]
    total, secs = 0.0, secs0
    cur = look(axis)
    if cur is None:
        print("  잴 수가 없다(%s) — 안 움직인다" % ("IMU" if axis == "w" else "노란 띠"))
        return 0.0
    for n in range(1, max_taps + 1):
        bn.drive(link, *vec, secs)
        nxt = look(axis)
        if nxt is None:
            print("  톡 %d: %.2fs → 잴 수 없게 됐다 — 멈춘다" % (n, secs))
            break
        g = gone_by(axis, cur, nxt) * sign
        total += g
        cur = nxt
        print("  톡 %d: %.2fs → %+.1f%s (합 %+.1f / %+.1f)" % (n, secs, g, "°" if axis == "w" else "mm",
                                                         total * sign, target))
        if total >= abs(target) * 0.85:
            break
        if g < moved_min:
            secs = min(max_secs, secs * grow)    # 안 움직였다 — 조금 길게
        elif total + g > abs(target) * 1.15 and secs > secs0:
            secs = max(secs0, secs / grow)       # 다음 걸음이면 넘친다 — 조금 짧게
    return total * sign


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--x", type=float, default=0.0, help="앞으로 mm(+앞)")
    ap.add_argument("--y", type=float, default=0.0, help="옆으로 mm(+왼쪽)")
    ap.add_argument("--w", type=float, default=0.0, help="회전 도(+반시계=왼쪽)")
    a = ap.parse_args()
    if not (a.x or a.y or a.w):
        print("갈 양이 없다")
        return 1
    if abs(a.x) > 150 or abs(a.y) > 150 or abs(a.w) > 45:
        print("한 번에 150mm·45°까지 — 나눠서")
        return 1
    import base_nudge as bn
    link = bn.open_link()
    if link is None:
        return 1
    try:
        for axis, t in (("w", a.w), ("x", a.x), ("y", a.y)):
            if t:
                print("%s %+.0f:" % (axis, t))
                got = tap_move(link, axis, t)
                print("  → %+.1f" % got)
    finally:
        link.stop()
        time.sleep(0.3)
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
