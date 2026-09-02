#!/usr/bin/env python3
"""**손끝을 base 좌표로 옮긴다** — dx,dy,dz mm, 또는 접근축을 따라.

    ~/lerobot/.venv/bin/python ros2/tools/tool_jog.py --dz 60        # 60mm 올린다
    ~/lerobot/.venv/bin/python ros2/tools/tool_jog.py --along -130   # 130mm 물러난다
    ~/lerobot/.venv/bin/python ros2/tools/tool_jog.py --dx 40 --dz -20

⚠ `tomato-voice`는 내리고. **사람이 보는 앞에서만.**

왜 또 하나 만드나 — `arm_stage.py`는 **관절 각도**를 받고 `cartesian.py`는
레거시 팔 객체를 받는다. ROS 계통의 도구들(`grasp_probe`·`stem_grasp`)은
이미 관절↔각도 변환과 안전 검사를 갖고 있으니, 여기서는 그 둘을 그대로 쓰고
**목표 방향만 바꿔 끼운다**. 기구학 야코비안은 잡음이 없어서(깊이 추정과
달리) 20mm 걸음에 부호가 뒤집히지 않는다 — 2026-09-02의 교훈.

⚠ 한 번에 크게 가지 않는다. `--piece`(기본 15mm)씩 쪼개어 매 조각마다
  야코비안을 다시 세운다 — 선형화는 가까이서만 맞다.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

from tomato_picker.hardware import kinematics as kin      # noqa: E402
import grasp_probe as gp                                   # noqa: E402

DEG_PER_TICK = 360.0 / 4096.0
JOINTS4 = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dx", type=float, default=0.0)
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--dz", type=float, default=0.0)
    ap.add_argument("--along", type=float, default=0.0,
                    help="접근축을 따라 mm (+앞 / -뒤)")
    ap.add_argument("--piece", type=float, default=25.0)
    ap.add_argument("--hold-pitch", action="store_true",
                    help="집게 방향을 지금 그대로 두려 애쓴다(wrist_flex로 보상)")
    ap.add_argument("--horiz", type=float, default=0.0,
                    help="접근축의 **수평 투영**을 따라 mm — 올린 높이를 지키며 다가간다")
    ap.add_argument("--free-pitch", action="store_true",
                    help="자세를 묶지 않는다 — 올릴 여유가 없을 때만")
    ap.add_argument("--pitch", type=float, default=0.0,
                    help="집게 피치를 이만큼 바꾼다(도, +위) — wrist_flex로만")
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args()

    cal = json.load(open(gp.CAL))
    spans = {}
    for name, c in cal.items():
        try:
            spans[name] = abs(int(c["range_max"]) - int(c["range_min"])) * DEG_PER_TICK
        except (KeyError, TypeError, ValueError):
            pass
    cart = json.load(open(gp.CART))
    zero, ref, signs = cart["zero"], cart["ref_deg"], cart.get("signs", {})
    over = cart.get("deg_per_norm") or {}
    geom = kin.ArmGeometry()

    def sign(j):
        v = signs.get(j)
        return -1.0 if (v is not None and float(v) < 0) else 1.0

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

    def legal(d, cur):
        nm, cm = to_norm(d), to_norm(cur)
        for j, v in nm.items():
            if abs(v) > max(98.0, abs(cm.get(j, 0.0))) + 1e-6:
                return False, "%s 한계" % j
        floor = min(-gp.MOUNT_Z_MM + gp.FLOOR_MARGIN_MM, kin.forward(cur, geom).z - 1.0)
        if kin.forward(d, geom).z < floor:
            return False, "바닥"
        return True, ""

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)

    def press_to(target, tries=3, tol=0.6, cap=5.0):
        """**도착할 때까지 밀어 넣는다** — 중력 처짐을 적분으로 갚는다.

        ⚠ 이 팔은 명령한 자리보다 2~3° 아래에 선다(어깨 -2.5°, 팔꿈치 -1.8°
          실측, 2026-09-02). 한 걸음이 그보다 작으면 **순효과가 음수**가 되어
          올리랬는데 내려간다 — 60mm 올리랬더니 15mm 내려갔다.
          그래서 모자란 만큼을 명령에 더해 다시 보낸다.
        """
        cmd = dict(target)
        for _ in range(tries):
            io.write(dict(to_norm(cmd), gripper=io.read().get("gripper", 40.0)), 0.9)
            time.sleep(0.7)
            got = to_deg(io.read())
            miss = {j: target[j] - got[j] for j in kin.JOINTS}
            if max(abs(v) for v in miss.values()) <= tol:
                return got
            for j in kin.JOINTS:
                cmd[j] = max(min(cmd[j] + miss[j], target[j] + cap), target[j] - cap)
            ok, _why = legal(cmd, got)
            if not ok:
                return got
        return to_deg(io.read())

    try:
        cur = to_deg(io.read())
        p0 = kin.forward(cur, geom)
        if args.pitch:
            # ⚠ 피치만 바꿀 땐 wrist_flex 하나만 쓴다 — 어깨·팔꿈치를 섞으면
            #   손끝이 같이 움직여서 "자세만 되돌린다"가 아니게 된다.
            tgt = dict(cur)
            tgt["wrist_flex"] += args.pitch
            ok, why = legal(tgt, cur)
            if not ok:
                print("피치를 못 바꾼다 (%s)" % why)
                return 1
            got = press_to(tgt)
            print("피치 %+.1f° → wrist_flex %.1f°" % (args.pitch, got["wrist_flex"]))
            if not (args.dx or args.dy or args.dz or args.along):
                return 0
            cur = got
            p0 = kin.forward(cur, geom)
        want = np.array([args.dx, args.dy, args.dz], float)
        if args.along:
            want = want + gp.approach_dir(cur, geom) * args.along
        if args.horiz:
            # ⚠ 접근축은 보통 아래를 향한다(이 자세에서 −41°). 줄기 높이로
            #   올려 놓고 그 축을 따라 나아가면 올린 만큼 도로 내려간다.
            #   그래서 **수평 성분만** 뽑아 쓴다.
            h = gp.approach_dir(cur, geom) * np.array([1.0, 1.0, 0.0])
            n = float(np.linalg.norm(h))
            if n < 1e-6:
                print("접근축이 수직이라 수평 성분이 없다")
                return 1
            want = want + h / n * args.horiz
        total = float(np.linalg.norm(want))
        print("지금 TCP (%.1f, %.1f, %.1f) · 옮길 것 (%+.1f, %+.1f, %+.1f) = %.1fmm"
              % (p0.x, p0.y, p0.z, want[0], want[1], want[2], total))
        if total < 0.5:
            return 0
        if args.dry:
            return 0

        n = max(1, int(round(total / args.piece)))
        unit = want / n
        for k in range(n):
            cur = to_deg(io.read())
            b0 = np.array([kin.forward(cur, geom).x, kin.forward(cur, geom).y,
                           kin.forward(cur, geom).z])
            J, cols = [], []
            for j in JOINTS4:
                e = dict(cur)
                e[j] += 0.5
                p1 = kin.forward(e, geom)
                J.append((np.array([p1.x, p1.y, p1.z]) - b0) / 0.5)
                cols.append(j)
            # ⚠ **자세를 함께 묶는다.** 위치 3식에 관절 4개면 자유도가 하나
            #   남고, 그 자유도가 실제로 쓰이면 손목 카메라가 확 돈다 —
            #   2026-09-02: 70mm 올리는 동안 카메라가 32° 아래로 돌아
            #   열매가 화면 위로 사라졌다. 피치(lift+elbow+wrist)를 고정하면
            #   미지수 4·식 4로 답이 하나가 된다.
            if args.free_pitch:
                A = np.array(J).T
                rhs = unit
            else:
                A = np.vstack([np.array(J).T, np.array([[0.0, 1.0, 1.0, 1.0]])])
                rhs = np.append(unit, 0.0)
            sol, *_ = np.linalg.lstsq(A, rhs, rcond=None)
            nxt = dict(cur)
            for j, w in zip(cols, sol):
                nxt[j] += float(w)
            ok, why = legal(nxt, cur)
            if not ok:
                print("  %d/%d  갈 수 없다 (%s) — 멈춘다" % (k + 1, n, why))
                break
            got = press_to(nxt)
            p = kin.forward(got, geom)
            print("  %d/%d  TCP (%.1f, %.1f, %.1f)  잔차 %.1fmm"
                  % (k + 1, n, p.x, p.y, p.z, float(np.linalg.norm(A @ sol - rhs))))
        p = kin.forward(to_deg(io.read()), geom)
        print("끝  TCP (%.1f, %.1f, %.1f)  — 처음에서 (%+.1f, %+.1f, %+.1f)"
              % (p.x, p.y, p.z, p.x - p0.x, p.y - p0.y, p.z - p0.z))
        return 0
    finally:
        io.hold_close()


if __name__ == "__main__":
    raise SystemExit(main())
