#!/usr/bin/env python3
"""**부하 중에** 서보를 들여다본다 — 어느 관절이, 얼마나, 왜 놓는가.

    ~/lerobot/.venv/bin/python ros2/tools/servo_load_probe.py

⚠ `tomato-voice.service`를 내린 채로. 팔이 뻗으므로 **사람이 보는 앞에서만.**

────────────────────────────────────────────────────────────────────────
왜 필요한가

팔을 뻗다가 **"갑자기 힘이 추욱"** 빠지는 일이 두 번 재현됐다(r≈250~277mm).
레지스터를 보니 원인 구조는 분명하다:

    Overload_Torque = 80      부하 80%가
    Protection_Time = 200     2초 지속되면
    Protective_Torque = 20    토크를 **20%로 떨어뜨린다**   ← 이게 "추욱"이다

그런데 `Torque_Limit`은 1000(100%)이라 **설정으로 묶어 둔 것이 아니다.** 서보가
진짜로 자기 한계에 닿은 것이다. 남은 질문은 셋이고 처방이 서로 다르다:

  · **전원이 부족한가**      → 부하 중 `Present_Voltage`가 떨어진다 (배선·전원 문제)
  · **어느 관절이 한계인가**  → 그 관절의 `Present_Load`만 치솟는다 (그쪽 하중을 줄인다)
  · **전류 보호인가**        → `Present_Current`가 `Protection_Current`에 붙는다

무부하로 재면 셋 다 0으로 보인다. **뻗는 동안** 재야 갈린다.

⚠ 보호가 걸리면 즉시 멈추고 물러난다. 데이터가 목적이지 부수는 게 목적이 아니다.
"""

from __future__ import annotations

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
JOINTS = list(kin.JOINTS)

STEP_DEG = 6.0        # 천천히 — 부하가 쌓이는 걸 볼 수 있게
SECS = 1.0
R_STOP = 300.0        # 여기까지만 뻗어 본다


def main() -> int:
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
        return abs(s) / 200.0 if s else 0.9

    def to_deg(n):
        return {j: ref.get(j, 0.0) + sign(j) * (float(n.get(j, 0.0)) - zero.get(j, 0.0)) * dpn(j)
                for j in JOINTS}

    def to_norm(d):
        return {j: zero.get(j, 0.0) + (float(d[j]) - ref.get(j, 0.0)) / (sign(j) * dpn(j))
                for j in d if j in JOINTS}

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    bus = io._follower.bus if io.connected else None    # noqa: SLF001
    now = to_deg(io.read())
    io.write(to_norm(now), 0.4)
    bus = io._follower.bus                              # noqa: SLF001

    def snap():
        out = {}
        for reg in ("Present_Load", "Present_Current", "Present_Voltage",
                    "Present_Temperature", "Torque_Enable"):
            try:
                out[reg] = bus.sync_read(reg)
            except Exception:                            # noqa: BLE001
                out[reg] = {}
        return out

    print("기준 " + " ".join(f"{j.split('_')[0]}={now[j]:6.1f}" for j in JOINTS))
    s = snap()
    print(f"  무부하: 전압 {[s['Present_Voltage'].get(j) for j in JOINTS]} "
          f"부하 {[s['Present_Load'].get(j) for j in JOINTS]}")

    # 팔을 **펴는** 방향으로 — a1과 a2를 함께 줄여 수평으로 뻗는다.
    print(f"\n뻗으면서 잰다 (수평거리 {R_STOP:.0f}mm까지, {STEP_DEG:.0f}°씩)")
    print("  r(mm)  " + "  ".join(f"{j.split('_')[0][:5]:>5}" for j in JOINTS)
          + "   전압(V)  최고온도")
    target = dict(now)
    tripped = False
    for k in range(1, 40):
        d = dict(now)
        # lift를 내리고 elbow를 펴서 앞으로 뻗는다 (pitch는 wrist로 유지)
        d["shoulder_lift"] = now["shoulder_lift"] - STEP_DEG * k * 0.5
        d["elbow_flex"] = now["elbow_flex"] - STEP_DEG * k
        d["wrist_flex"] = now["wrist_flex"] + STEP_DEG * k * 1.5
        over = [j for j in JOINTS
                if abs(d[j] - ref.get(j, 0.0)) > spans.get(j, 200.0) / 2.0 - 2.0]
        if over:
            print(f"  관절 한계에 닿아 멈춘다: {over}")
            break
        p = kin.forward(d, geom)
        r = math.hypot(p.x, p.y)
        if p.z < 20.0:
            print(f"  바닥에 가까워 멈춘다 (z={p.z:.0f}mm)")
            break
        io.write(to_norm(d), SECS)
        time.sleep(0.5)
        s = snap()
        got = to_deg(io.read())
        gp = kin.forward(got, geom)
        gr = math.hypot(gp.x, gp.y)
        load = [s["Present_Load"].get(j) for j in JOINTS]
        volt = [s["Present_Voltage"].get(j) or 0 for j in JOINTS]
        temp = [s["Present_Temperature"].get(j) or 0 for j in JOINTS]
        te = [s["Torque_Enable"].get(j) for j in JOINTS]
        print(f"  {gr:5.0f}  " + "  ".join(f"{(v if v is not None else -1):5}" for v in load)
              + f"   {min(volt)/10:.1f}~{max(volt)/10:.1f}  {max(temp)}C")
        if any(v == 0 for v in te):
            off = [JOINTS[i] for i, v in enumerate(te) if v == 0]
            print(f"  ⚠ 토크가 꺼진 관절: {off}  ← **여기가 놓았다**")
            tripped = True
            break
        err = max(abs(got[j] - d[j]) for j in JOINTS)
        if err > 12.0:
            print(f"  ⚠ 추종 오차 {err:.1f}° — 놓기 직전이다. 멈춘다.")
            tripped = True
            break
        if gr >= R_STOP:
            print(f"  목표 {R_STOP:.0f}mm 도달 — 보호가 안 걸렸다.")
            break

    print("\n돌아간다")
    cur = to_deg(io.read())
    n = max(1, int(math.ceil(max(abs(now[j] - cur[j]) for j in JOINTS) / STEP_DEG)))
    for i in range(1, n + 1):
        io.write(to_norm({j: cur[j] + (now[j] - cur[j]) * i / n for j in JOINTS}), SECS)
    s = snap()
    print(f"  복귀 후: 부하 {[s['Present_Load'].get(j) for j in JOINTS]} "
          f"온도 {[s['Present_Temperature'].get(j) for j in JOINTS]}")
    if tripped:
        print("\n읽는 법:")
        print("  · 전압이 11V 아래로 떨어졌다면 → **전원 문제** (배선·전류용량)")
        print("  · 한 관절의 부하만 치솟았다면  → 그 관절 하중을 줄여야 한다")
        print("  · 전압도 멀쩡하고 부하가 고루 높다면 → 이 팔의 물리적 한계다")
    io.hold_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
