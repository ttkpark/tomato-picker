#!/usr/bin/env python3
"""기구학 영점을 지금 자세로 잡는다 — 팔을 **교시 자세로 세워 둔 상태**에서.

    ~/lerobot/.venv/bin/python ros2/tools/set_zero.py --dry   (지금 상태만 보고)
    ~/lerobot/.venv/bin/python ros2/tools/set_zero.py         (잡고 검증하고 붙든다)

⚠ `tomato-voice.service`를 내린 채로. 대시보드의 [3D 좌표 영점]과 **같은 코드**를
   쓴다(`cartesian.FrameConfig`) — 형식이 갈리면 두 계통이 다른 영점을 믿는다.

────────────────────────────────────────────────────────────────────────
교시 자세 = **어깨는 정면, 나머지는 곧게 위로** (lift=90, elbow=0, wrist=0)

수평으로 편 자세를 안 쓰는 이유 — 힘을 빼면 중력이 끌어내리고 그 처짐이 그대로
영점 오차가 된다. 수직은 문틀에 대보면 눈으로 맞는다.

⚠ 이 자세에서도 집게는 pan축 **바로 위가 아니다.** d0(2번 축의 수평 오프셋)만큼
   비켜서 있다 — 2026-08-31 실측 -31.5mm(뒤쪽). 그래서 검증도 x≈d0로 본다.

⚠ 영점을 다시 잡으면 저장된 프리셋의 **좌표 해석**이 바뀐다(관절값 자체는 그대로라
   재생 동작은 같다). 그리고 손-눈 보정은 **무효가 된다** — 도구 좌표계의 뜻이
   바뀌기 때문이다. 그래서 잡기 전에 파일을 백업한다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

from tomato_picker.config import ARM_CART_ZERO_POSE_DEG  # noqa: E402
from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware.cartesian import FrameConfig  # noqa: E402

TOL_DEG = 0.05
TOL_MM = 0.5


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true", help="잡지 않고 지금 상태만 본다")
    args = ap.parse_args()

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    norms = io.read()                    # ⚠ 읽기만 — 잡기 전에 팔을 움직이면 안 된다
    cfg = FrameConfig()
    geom = cfg.geometry()

    print("교시 자세 정의: "
          + " ".join(f"{j.split('_')[0]}={ARM_CART_ZERO_POSE_DEG.get(j, 0.0):.0f}°"
                     for j in kin.JOINTS))
    print(f"링크(실측): z0={geom.z0} d0={geom.d0} l1={geom.l1} l2={geom.l2} l3={geom.l3}")
    print(f"\n지금 정규화값: " + " ".join(f"{j.split('_')[0]}={norms.get(j, 0.0):7.2f}"
                                      for j in kin.JOINTS))

    if cfg.has_zero:
        old_zero = cfg.zero()
        spans = io.spans_deg()
        print("\n옛 영점과의 차이:")
        worst = 0.0
        for j in kin.JOINTS:
            dn = float(norms.get(j, 0.0)) - old_zero[j]
            per = abs(spans.get(j, 180.0)) / 200.0 if spans.get(j) else 0.9
            dd = dn * per * cfg.sign(j)
            worst = max(worst, abs(dd))
            print(f"  {j:<15}{dn:+8.2f} 정규화 = {dd:+7.2f}°")
        print(f"  가장 큰 틀어짐 {worst:.2f}°")

    if args.dry:
        print("\n(--dry 이므로 잡지 않는다)")
        io.hold_close()
        return 0

    # ── 백업 ──
    path = cfg.path
    if os.path.exists(path):
        backup = path + time.strftime(".%Y%m%d-%H%M%S.bak")
        shutil.copy2(path, backup)
        print(f"\n백업: {backup}")

    cfg.set_zero({j: float(norms.get(j, 0.0)) for j in kin.JOINTS},
                 ARM_CART_ZERO_POSE_DEG)
    cfg.reload()
    print(f"영점 저장: {path}")

    # ── 검증: 지금 자세를 새 영점으로 읽으면 교시 자세가 나와야 한다 ──
    print("\n[검증] 지금 자세를 새 영점으로 읽는다")
    zero, ref = cfg.zero(), cfg.ref_deg()
    spans = io.spans_deg()

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

    degs = {j: ref[j] + cfg.sign(j) * (float(norms.get(j, 0.0)) - zero[j]) * per(j)
            for j in kin.JOINTS}
    ok = True
    for j in kin.JOINTS:
        want = ARM_CART_ZERO_POSE_DEG.get(j, 0.0)
        good = abs(degs[j] - want) < TOL_DEG
        ok &= good
        print(f"  {'ok  ' if good else 'FAIL'} {j:<15}{degs[j]:8.2f}° (기대 {want:.0f}°)")

    pose = kin.forward(degs, geom)
    up = geom.z0 + geom.l1 + geom.l2 + geom.l3
    good = (abs(pose.x - geom.d0) < TOL_MM and abs(pose.y) < TOL_MM
            and abs(pose.z - up) < TOL_MM and abs(pose.pitch - 90.0) < TOL_DEG)
    ok &= good
    print(f"  {'ok  ' if good else 'FAIL'} TCP ({pose.x:.1f}, {pose.y:.1f}, {pose.z:.1f}) "
          f"pitch {pose.pitch:.1f}°  (기대 {geom.d0:.1f}, 0, {up:.0f}, 90°)")

    # ── 붙든다 — 지금 토크가 꺼져 있어 놓으면 쓰러진다 ──
    print("\n토크를 켜고 이 자세를 붙든다")
    io.write(norms, 0.6)
    io.hold_close()

    print("\n" + ("✅ 영점을 잡았고 검증도 통과했다." if ok
                  else "❌ 검증이 통과하지 못했다 — 백업으로 되돌릴 수 있다."))
    if ok:
        print("⚠ 손-눈 보정은 이제 무효다(도구 좌표계의 뜻이 바뀌었다). 다시 잡아야 한다.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
