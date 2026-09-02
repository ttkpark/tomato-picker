#!/usr/bin/env python3
"""관절을 **하나씩 순서대로** 움직여 목표 자세로 — 팔을 최대한 접은 채로 간다.

    ~/lerobot/.venv/bin/python ros2/tools/arm_stage.py --target "0,91,-74.5,-86.5,-3.6"
    ... --dry     (계산만)

⚠ `tomato-voice.service`를 내린 채로.

────────────────────────────────────────────────────────────────────────
왜 필요한가 — **선형 보간이 팔을 최대로 펴며 지나간다**

모든 관절을 동시에 목표로 보간하면, 중간 어딘가에서 팔이 쭉 펴진다. 2026-08-31
실측: 접힌 자세(r=47mm)에서 내려다보는 자세(r=160mm)로 가는데 중간에 **r=277mm**
까지 부풀었고, 거기서 서보가 놓았다(추종 오차 5.8° → 13.5° → 22.9°).
목적지도 출발지도 안전한데 **가는 길이 위험한** 경우다.

관절을 순서대로 움직이면 훨씬 접힌 채로 간다. 같은 이동을 lift → elbow →
wrist_flex 순으로 나누면 최대 r이 **277 → 222mm**로 줄어든다.

순서가 왜 그런가 — 어깨를 먼저 세우면(lift) 팔 전체가 위로 접히고, 그 상태에서
팔꿈치를 접으면(elbow) 여전히 몸통 가까이 있다. 손목(wrist_flex)은 마지막에
돌린다. 마지막 링크(95mm)가 수평을 지나는 순간이 r이 가장 부푸는 때이므로,
그때 앞의 두 링크가 최대한 접혀 있어야 한다.

⚠ 그래도 완전히는 못 피한다. 관절 한계(elbow ±103°) 때문에 마지막 링크가
   수평을 지날 때 r은 220mm 아래로 못 내려간다. 이 팔에 카메라를 얹은 채로는
   그 근처가 한계선이다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

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
STEP_DEG = 8.0
SECS = 1.0
MAX_TRACK_ERR = 12.0
# 이 순서로 하나씩 움직인다. 마지막 링크를 마지막에 돌리는 것이 요점이다.
ORDER = (("shoulder_pan",), ("shoulder_lift",), ("elbow_flex",),
         ("wrist_flex",), ("wrist_roll",))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", required=True,
                    help="목표 관절각(도) pan,lift,elbow,wflex,wroll")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--order", default="",
                    help="관절 순서를 손으로 지정한다(쉼표). 비우면 |r|이 작은 쪽부터 자동. "
                         "자동이 바닥에 걸릴 때 쓴다 — 접힌 관절을 먼저 돌리면 "
                         "팔이 낮게 지나가지 않는다")
    ap.add_argument("--hold", action="store_true", default=True)
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

    # ⚠ **실측 눈금이 보정표를 이긴다** (`~/arm_cartesian.json`의 deg_per_norm).
    #   2026-09-01: `wrist_roll`은 계산한 각도의 0.56배만 실제로 돌았다 —
    #   관절축 측정 잔차가 24.8mm에서 2.6mm로 떨어졌고, 화면회전 실측
    #   0.549와도 맞는다. 손목 굴림에 감속이 있어 틱→도(360/4096)가 안 통한다.
    over = cart.get("deg_per_norm") or {}

    def dpn(j):
        v = over.get(j)
        if v:
            return abs(float(v))
        s = spans.get(j)
        return abs(s) / 200.0 if s else (1.8 if j == "wrist_roll" else 0.9)

    def to_deg(n):
        return {j: ref.get(j, 0.0) + sign(j) * (float(n.get(j, 0.0)) - zero.get(j, 0.0)) * dpn(j)
                for j in kin.JOINTS}

    def to_norm(d):
        return {j: zero.get(j, 0.0) + (float(d[j]) - ref.get(j, 0.0)) / (sign(j) * dpn(j))
                for j in d if j in kin.JOINTS}

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    now = to_deg(io.read())
    io.write(to_norm(now), 0.4)          # 붙자마자 지금 자리를 붙든다 (안 그러면 처진다)

    target = dict(now)
    target.update(dict(zip(kin.JOINTS, [float(v) for v in args.target.split(",")])))

    def show(tag, d):
        p = kin.forward(d, geom)
        print(f"  {tag}  " + " ".join(f"{j.split('_')[0]}={d[j]:7.1f}" for j in kin.JOINTS)
              + f"   TCP({p.x:6.1f},{p.y:6.1f},{p.z:6.1f}) pitch{p.pitch:7.1f} "
                f"r{kin.signed_radius(d, geom):7.1f}")

    show("지금", now)
    show("목표", target)

    # ⚠ 진짜 바닥(마운트 아래 76.5mm)을 쓴다. 팔이 늘어져 z가 음수여도 그건
    #    아직 바닥 위다. 시작 자세가 그보다도 낮으면 그때만 시작점을 기준 삼는다.
    now_norm = to_norm(now)
    z_floor = min(-MOUNT_Z_MM + FLOOR_MARGIN_MM, kin.forward(now, geom).z - 1.0)

    def leg(cur, joint):
        """한 관절만 목표로 옮기는 구간 — 경유점과 최대 |r|, 위반 목록."""
        nxt = dict(cur)
        nxt[joint] = target[joint]
        big = max(abs(nxt[j] - cur[j]) for j in kin.JOINTS)
        if big < 0.05:
            return None
        steps = max(1, int(math.ceil(big / STEP_DEG)))
        pts = []
        for s in range(1, steps + 1):
            mid = {j: cur[j] + (nxt[j] - cur[j]) * s / steps for j in kin.JOINTS}
            p = kin.forward(mid, geom)
            bad = []
            if p.z < z_floor:
                bad.append("바닥아래")
            # ⚠ 가동범위는 **정규화값 -100..100**이지, 교시 자세를 중심으로
            #   대칭인 게 아니다. 영점을 어디서 잡았느냐에 따라 한쪽으로
            #   크게 치우친다(실측: elbow는 -14.7° ~ +191.1°). 도(度)로
            #   ±span/2를 보면 멀쩡한 자세를 막고 위험한 자세를 통과시킨다.
            nm = to_norm(mid)
            for j in kin.JOINTS:
                # ⚠ **지금 자리를 가두면 안 된다.** 중력에 밀려 이미 한계를 조금
                #   넘어 있으면 "≤98"을 고집하는 순간 어느 경로도 못 만든다 —
                #   2026-09-02, elbow가 -101.8에서 팔이 통째로 갇혔다. 바닥 가드에
                #   넣었던 같은 원칙을 관절 한계에도 적용한다: **더 나빠지지만
                #   않으면** 통과.
                if abs(nm[j]) > max(98.0, abs(now_norm.get(j, 0.0))) + 1e-6:
                    bad.append(f"{j}한계")
            pts.append((mid, p, abs(kin.signed_radius(mid, geom)), bad))
        return nxt, pts

    # **순서를 고정하지 않는다.** 남은 관절 중 "그 구간의 최대 |r|이 가장 작은"
    # 것을 매번 고른다. 고정 순서(lift 먼저)는 늘어진 자세에서 시작하면 r을
    # 307mm까지 부풀렸다 — 팔이 이미 뻗어 있으면 **먼저 접어야** 한다.
    print("\n단계별 경로 (관절 하나씩 · 순서는 |r|이 작은 쪽부터 자동)")
    stages, cur, worst = [], dict(now), 0.0
    remaining = [j for j in kin.JOINTS if abs(target[j] - cur[j]) >= 0.05]
    forced = [j.strip() for j in args.order.split(",") if j.strip()]
    for j in forced:
        if j not in kin.JOINTS:
            print(f"❌ --order 에 없는 관절: {j}")
            io.hold_close()
            return 1
    while remaining:
        scored = []
        for j in remaining:
            got = leg(cur, j)
            if got is None:
                continue
            nxt, pts = got
            # 손으로 순서를 준 관절은 그 순서대로, 나머지는 |r|이 작은 쪽부터.
            rank = forced.index(j) if j in forced else len(forced) + 1
            scored.append(((rank, max(q[2] for q in pts)), j, nxt, pts))
        if not scored:
            break
        scored.sort(key=lambda x: x[0])
        (_, peak), joint, nxt, pts = scored[0]
        stages.append((joint, pts))
        worst = max(worst, peak)
        rr = [q[2] for q in pts]
        flags = sorted({b for q in pts for b in q[3]})
        print(f"  {joint:<14} {len(pts):2}구간 · |r| {min(rr):5.0f}~{max(rr):5.0f}mm"
              + (f"  ⚠ {' '.join(flags)}" if flags else ""))
        cur = nxt
        remaining.remove(joint)

    print(f"\n경로 전체 최대 |r| = {worst:.0f}mm "
          f"(선형 보간이었다면 277mm에서 서보가 놓았다)")
    if any(q[3] for _, pts in stages for q in pts):
        print("❌ 안전 검사에 걸린 구간이 있다 — 움직이지 않는다.")
        io.hold_close()
        return 1
    if args.dry:
        print("(--dry 이므로 여기서 멈춘다)")
        io.hold_close()
        return 0

    print("\n움직인다 — 단계마다 실제 자세를 되읽어 확인한다.")
    for name, pts in stages:
        for i, (mid, _, r, _) in enumerate(pts, 1):
            io.write(to_norm(mid), SECS)
            got = to_deg(io.read())
            err = max(abs(got[j] - mid[j]) for j in kin.JOINTS)
            gp = kin.forward(got, geom)
            gr = kin.signed_radius(got, geom)
            print(f"  {name:<14} {i:2}/{len(pts)}  TCP({gp.x:6.1f},{gp.y:6.1f},"
                  f"{gp.z:6.1f}) pitch{gp.pitch:7.1f} r{gr:7.1f}  오차 {err:5.1f}°")
            if err > MAX_TRACK_ERR:
                print(f"     ⚠ {MAX_TRACK_ERR:.0f}°를 넘었다 — 서보가 놓았다. 중단.")
                io.hold_close()
                return 1

    final = to_deg(io.read())
    show("끝", final)
    print("\n⚠ 토크를 켠 채로 둔다.")
    io.hold_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
