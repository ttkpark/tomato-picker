#!/usr/bin/env python3
"""특이점에서 빠져나오기 — **관절 공간으로** 팔을 뻗은 자세로 옮긴다.

    ~/lerobot/.venv/bin/python ros2/tools/arm_extend.py --dry     (계산만)
    ~/lerobot/.venv/bin/python ros2/tools/arm_extend.py           (실제로 움직임)

⚠ `tomato-voice.service`를 먼저 내려야 한다 — 팔 포트는 한 프로세스만 연다.

────────────────────────────────────────────────────────────────────────
왜 이 도구가 필요한가

좌표 유닛(`cartesian.py`)은 **지금 자세**의 `signed_radius`가 90mm보다 작으면
어떤 이동도 거절한다. 옳은 가드다 — 집게가 pan축 위에 올라앉으면 "앞으로 5mm"가
어느 방향인지 정의되지 않고, `signed_radius`가 음수면 FK의 방위각이 실제 pan보다
180° 뒤집혀 있어 FK→IK 왕복이 거울상으로 튄다.

그런데 그 가드는 **팔이 못 움직인다는 뜻이 아니다.** 관절을 조금 펴면 빠져나온다.
대시보드에는 임의 관절값을 보내는 API가 없어서(현재자세 저장 / 저장된 프리셋
재생뿐) 여기서 lerobot 버스를 직접 잡는다.

빠져나온 뒤에는 좌표 이동이 열리므로, 이 도구는 **한 번만** 쓰면 된다.

────────────────────────────────────────────────────────────────────────
안전

  · 목표까지 한 번에 가지 않고 **관절당 최대 STEP_DEG씩** 나눠 간다.
  · 매 구간마다 FK로 TCP를 계산해 **바닥(z_min)·사거리**를 미리 검사한다.
    하나라도 걸리면 시작조차 하지 않는다(--dry가 그걸 보여 준다).
  · ⚠ 손목에 카메라와 USB3 케이블이 달려 있다. 큰 관절 회전은 케이블을 감을 수
    있으므로 **사람이 보는 앞에서만** 쓴다. 코드는 케이블을 볼 수 없다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

from tomato_picker.hardware import kinematics as kin  # noqa: E402

DEG_PER_TICK = 360.0 / 4096.0
CAL = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json")
CART = os.path.expanduser("~/arm_cartesian.json")

STEP_DEG = 12.0        # 한 구간에서 어느 관절도 이 이상 안 움직인다
SECS_PER_STEP = 1.2
R_TARGET = 150.0       # 좌표 가드(90mm)에서 충분히 떨어진 곳까지

# 목표 자세 — 집게가 **수평 앞**을 보는 표준 자세.
#   lift 80° = 상완이 거의 수직, elbow -80° = 전완이 수평, wrist 0° = 손목 곧게
#   → a1=80, a2=0, a3=0 이므로 pitch=0(수평), r≈250mm, z≈169mm
TARGET_DEG = {"shoulder_lift": 80.0, "elbow_flex": -80.0, "wrist_flex": 0.0}


def load_frame():
    cal = json.load(open(CAL))
    spans = {}
    for name, c in cal.items():
        try:
            spans[name] = abs(int(c["range_max"]) - int(c["range_min"])) * DEG_PER_TICK
        except (KeyError, TypeError, ValueError):
            pass
    cart = json.load(open(CART))
    return spans, cart["zero"], cart["ref_deg"], cart.get("signs", {})


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true", help="계산만 하고 안 움직인다")
    ap.add_argument("--target", default="",
                    help="목표 관절각(도) pan,lift,elbow,wflex,wroll. "
                         "비우면 기본 목표(집게가 수평 앞)")
    ap.add_argument("--hold", action="store_true",
                    help="끝나고 토크를 켠 채 둔다 (안 그러면 팔이 떨어진다)")
    args = ap.parse_args()

    spans, zero, ref, signs = load_frame()
    geom = kin.ArmGeometry()

    def sign(j):
        v = signs.get(j)
        return -1.0 if (v is not None and float(v) < 0) else 1.0

    def dpn(j):
        s = spans.get(j)
        return abs(s) / 200.0 if s else (1.8 if j == "wrist_roll" else 0.9)

    def to_deg(norms):
        return {j: ref.get(j, 0.0) + sign(j) * (float(norms.get(j, 0.0)) - zero.get(j, 0.0)) * dpn(j)
                for j in kin.JOINTS}

    def to_norm(degs):
        return {j: zero.get(j, 0.0) + (float(degs[j]) - ref.get(j, 0.0)) / (sign(j) * dpn(j))
                for j in degs if j in kin.JOINTS}

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO()
    now_norms = io.read()
    now = to_deg(now_norms)
    p0 = kin.forward(now, geom)
    r0 = kin.signed_radius(now, geom)
    print(f"지금  " + " ".join(f"{j.split('_')[0]}={now[j]:6.1f}" for j in kin.JOINTS))
    print(f"      TCP ({p0.x:6.1f},{p0.y:6.1f},{p0.z:6.1f}) pitch {p0.pitch:6.1f}° "
          f"signed_r {r0:6.1f}mm")

    target = dict(now)
    if args.target.strip():
        target.update(dict(zip(kin.JOINTS,
                               [float(v) for v in args.target.split(",")])))
    else:
        target.update(TARGET_DEG)
    pt = kin.forward(target, geom)
    rt = kin.signed_radius(target, geom)
    print(f"목표  " + " ".join(f"{j.split('_')[0]}={target[j]:6.1f}" for j in kin.JOINTS))
    print(f"      TCP ({pt.x:6.1f},{pt.y:6.1f},{pt.z:6.1f}) pitch {pt.pitch:6.1f}° "
          f"signed_r {rt:6.1f}mm")

    # 관절 최대 변화량으로 구간 수를 정한다 — 어느 관절도 STEP_DEG를 안 넘게.
    biggest = max(abs(target[j] - now[j]) for j in kin.JOINTS)
    steps = max(1, int(math.ceil(biggest / STEP_DEG)))
    print(f"\n경로 {steps}구간 (최대 관절 변화 {biggest:.0f}° · 구간당 ≤{STEP_DEG:.0f}°)")

    path, bad = [], []
    for i in range(1, steps + 1):
        degs = {j: now[j] + (target[j] - now[j]) * i / steps for j in kin.JOINTS}
        pose = kin.forward(degs, geom)
        r = kin.signed_radius(degs, geom)
        note = []
        if pose.z < 15.0:
            note.append("바닥아래")
        if math.hypot(pose.x, pose.y) > geom.reach_max + 1e-6:
            note.append("사거리밖")
        for j, lim in (("shoulder_lift", spans.get("shoulder_lift", 205) / 2),
                       ("elbow_flex", spans.get("elbow_flex", 206) / 2),
                       ("wrist_flex", spans.get("wrist_flex", 210) / 2)):
            if abs(degs[j] - ref.get(j, 0.0)) > lim:
                note.append(f"{j}한계")
        if note:
            bad.append(i)
        path.append((i, degs, pose, r, " ".join(note)))
        print(f"  {i:2}/{steps}  TCP ({pose.x:6.1f},{pose.y:6.1f},{pose.z:6.1f}) "
              f"pitch {pose.pitch:6.1f}° r {r:6.1f}  {' '.join(note)}")

    if bad:
        print(f"\n❌ {len(bad)}개 구간이 안전 검사에 걸린다 — 움직이지 않는다.")
        return 1
    print("\n✅ 모든 구간이 바닥 위·사거리 안·관절한계 안이다.")

    if args.dry:
        print("(--dry 이므로 여기서 멈춘다)")
        io.close()
        return 0

    print("\n움직인다 — 구간마다 실제 자세를 되읽어 확인한다.")
    for i, degs, pose, r, _ in path:
        io.write(to_norm(degs), SECS_PER_STEP)
        got = to_deg(io.read())
        gp = kin.forward(got, geom)
        gr = kin.signed_radius(got, geom)
        err = max(abs(got[j] - degs[j]) for j in kin.JOINTS)
        print(f"  {i:2}/{len(path)}  실제 TCP ({gp.x:6.1f},{gp.y:6.1f},{gp.z:6.1f}) "
              f"pitch {gp.pitch:6.1f}° r {gr:6.1f}  (관절 오차 최대 {err:.1f}°)")
        if err > 20.0:
            print("     ⚠ 지령과 실제가 20° 넘게 다르다 — 무언가에 걸렸을 수 있다. 중단.")
            io.close()
            return 1

    final = to_deg(io.read())
    fp = kin.forward(final, geom)
    fr = kin.signed_radius(final, geom)
    print(f"\n끝  TCP ({fp.x:.1f}, {fp.y:.1f}, {fp.z:.1f}) pitch {fp.pitch:.1f}° "
          f"signed_r {fr:.1f}mm")
    print("좌표 이동 가능" if fr >= 90 else "⚠ 아직 가드 안쪽이다")
    if args.hold:
        # 토크를 끄지 않고 닫는다 — 끄면 그 자리에서 떨어진다.
        io._follower.disconnect()      # noqa: SLF001
    else:
        io.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
