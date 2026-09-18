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
import time

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

from tomato_picker.hardware import escape as es  # noqa: E402
from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware import settle as st  # noqa: E402

DEG_PER_TICK = 360.0 / 4096.0
CAL = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json")
CART = os.path.expanduser("~/arm_cartesian.json")

# ── 탈출 규칙은 `hardware/escape.py` 하나가 가진다 ─────────────────────────
# 2026-09-18(T49)까지 12°·바닥·한계·목표자세가 **이 파일 안에만** 있었다. 같은
# 규칙을 ROS 쪽(`arm_source.move_joints_deg`)과 채점 도구(`move5_check`의 prep)도
# 쓰게 되어 라이브러리로 옮겼다 — 베끼면 말없이 갈라지고, 갈라지면 한쪽이
# 통과시킨 걸음을 다른 쪽이 거절한다. 이름은 그대로 두어 부르는 쪽은 안 바뀐다.
MOUNT_Z_MM = es.MOUNT_Z_MM
FLOOR_MARGIN_MM = es.FLOOR_MARGIN_MM
STEP_DEG = es.STEP_DEG
TARGET_DEG = es.TARGET_DEG
LIMIT_NORM = es.LIMIT_NORM
clamp_norm = es.clamp_norm
limit_violations = es.worsening_limits

SECS_PER_STEP = es.SECS_PER_STEP

RECORD_DIR = os.path.join(REPO, "docs", "시험기록")


def load_frame():
    cal = json.load(open(CAL))
    spans = {}
    for name, c in cal.items():
        try:
            spans[name] = abs(int(c["range_max"]) - int(c["range_min"])) * DEG_PER_TICK
        except (KeyError, TypeError, ValueError):
            pass
    cart = json.load(open(CART))
    return (spans, cart["zero"], cart["ref_deg"], cart.get("signs", {}),
            cart.get("deg_per_norm") or {})


def main() -> int:
    ap = argparse.ArgumentParser(
        description="특이점에서 빠져나오기 — 관절 공간으로 팔을 뻗은 자세로 옮긴다. "
                    "종료 시 팔이 떨어지지 않도록 항상 토크를 켠 채 포트를 닫는다(hold_close).")
    ap.add_argument("--dry", action="store_true", help="계산만 하고 안 움직인다")
    ap.add_argument("--target", default="",
                    help="목표 관절각(도) pan,lift,elbow,wflex,wroll. "
                         "비우면 기본 목표(집게가 수평 앞)")
    ap.add_argument("--no-settle", action="store_true",
                    help="마지막 걸음 뒤 되먹임(처짐 지우기)을 하지 않는다")
    ap.add_argument("--record", default="",
                    help="되먹임 기록 jsonl 경로 (기본 docs/시험기록/settle-<오늘>.jsonl)")
    args = ap.parse_args()

    spans, zero, ref, signs, over = load_frame()
    geom = kin.ArmGeometry()

    def sign(j):
        v = signs.get(j)
        return -1.0 if (v is not None and float(v) < 0) else 1.0

    # ⚠ **실측 눈금이 보정표를 이긴다** (`~/arm_cartesian.json`의 deg_per_norm).
    #   2026-09-01: `wrist_roll`은 계산한 각도의 0.56배만 실제로 돌았다 —
    #   관절축 측정 잔차가 24.8mm에서 2.6mm로 떨어졌고, 화면회전 실측
    #   0.549와도 맞는다. 손목 굴림에 감속이 있어 틱→도(360/4096)가 안 통한다.
    def dpn(j):
        v = over.get(j)
        if v:
            return abs(float(v))
        s = spans.get(j)
        return abs(s) / 200.0 if s else (1.8 if j == "wrist_roll" else 0.9)

    def to_deg(norms):
        return {j: ref.get(j, 0.0) + sign(j) * (float(norms.get(j, 0.0)) - zero.get(j, 0.0)) * dpn(j)
                for j in kin.JOINTS}

    def to_norm(degs):
        return {j: zero.get(j, 0.0) + (float(degs[j]) - ref.get(j, 0.0)) / (sign(j) * dpn(j))
                for j in degs if j in kin.JOINTS}

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
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
    clamped, hit = clamp_norm(to_norm(target))
    if hit:
        print("⚠ 가동범위 밖이라 목표를 잘랐다: " + ", ".join(
            f"{j} {to_norm(target)[j]:.1f}→{clamped[j]:.1f}" for j in hit))
    target = to_deg({**now_norms, **clamped})
    pt = kin.forward(target, geom)
    rt = kin.signed_radius(target, geom)
    print(f"목표  " + " ".join(f"{j.split('_')[0]}={target[j]:6.1f}" for j in kin.JOINTS))
    print(f"      TCP ({pt.x:6.1f},{pt.y:6.1f},{pt.z:6.1f}) pitch {pt.pitch:6.1f}° "
          f"signed_r {rt:6.1f}mm")

    # 걸음 쪼개기·안전 검사는 **escape.plan 하나가** 한다(ROS 쪽과 같은 규칙).
    steps_info = es.plan(now, target, now_norms, to_norm, geom)
    steps = steps_info[-1]["of"]
    biggest = max(abs(target[j] - now[j]) for j in kin.JOINTS)
    print(f"\n경로 {steps}구간 (최대 관절 변화 {biggest:.0f}° · 구간당 ≤{es.STEP_DEG:.0f}°)")

    path = []
    for step in steps_info:
        pose, note = step["pose"], " ".join(step["notes"])
        path.append((step["i"], step["degs"], pose, step["signed_r"], note))
        print(f"  {step['i']:2}/{steps}  TCP ({pose.x:6.1f},{pose.y:6.1f},{pose.z:6.1f}) "
              f"pitch {pose.pitch:6.1f}° r {step['signed_r']:6.1f}  {note}")

    bad = es.blocked(steps_info)
    if bad:
        print(f"\n❌ {len(bad)}개 구간이 안전 검사에 걸린다 — 움직이지 않는다.")
        return 1
    print("\n✅ 모든 구간이 바닥 위·사거리 안·관절한계 안이다.")

    if args.dry:
        print("(--dry 이므로 여기서 멈춘다)")
        io.hold_close()
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
            io.hold_close()
            return 1

    # ── 마지막 걸음 뒤: 처짐을 되먹임으로 지운다 ──────────────────────────
    # 여기까지의 루프는 "지령을 한 번 쓰고 되읽어 오차를 찍기만" 했다. 오차를
    # 보고도 아무것도 안 하면 팔은 중력이 누른 만큼 못 미친 자리에 선 채 끝난다
    # (2026-09-18 실측 관절 오차 A자세 1.1~1.3° / B자세 4.0~4.9°, §22).
    # ⚠ 같은 값을 다시 보내는 것으로는 안 된다 — 이미 정상상태다. 오차만큼 **더** 준다.
    def deliver(cmd):
        """보정 지령을 **실제로 보낼 수 있는 값**으로 깎는다.

        보정은 목표를 지나쳐 가는 값이라 그냥 보내면 한 걸음 상한·가동범위·바닥을
        넘을 수 있다. 관절마다 STEP_DEG 안으로 자르고 가동범위로 한 번 더 자른 뒤,
        그 자세가 바닥/사거리를 어기면 **보정을 통째로 버린다**(안전이 처짐보다 세다).
        settle은 깎인 관절을 '한계에 눌림'으로 보고 그 관절에 더 안 민다.
        """
        capped = {j: target[j] + max(-STEP_DEG, min(STEP_DEG, float(v) - target[j]))
                  for j, v in cmd.items()}
        norms, _ = clamp_norm(to_norm(capped))
        out = to_deg({**now_norms, **norms})
        pose = kin.forward(out, geom)
        if (pose.z < -MOUNT_Z_MM + FLOOR_MARGIN_MM
                or math.hypot(pose.x, pose.y) > geom.reach_max + 1e-6):
            print("     ⚠ 보정 자세가 바닥/사거리를 어긴다 — 되먹임을 버린다.")
            return dict(target)
        return {j: out[j] for j in cmd}

    cfg = st.SettleConfig(rounds=0) if args.no_settle else st.SettleConfig()
    res = st.settle(target,
                    measure=lambda: to_deg(io.read()),
                    send=lambda degs: io.write(to_norm(degs), SECS_PER_STEP),
                    deliver=deliver, cfg=cfg)
    print("\n되먹임 " + res.describe())
    for r in res.rounds:
        gain = "" if r.gain is None else f"  개선 {r.gain * 100:+.1f}%"
        print(f"  {r.n}회  오차 {r.err_deg:5.2f}° ({r.worst}){gain}")
    if not args.no_settle:
        row = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "tool": "arm_extend",
               "target_deg": {j: round(target[j], 2) for j in kin.JOINTS},
               "settle": res.as_record()}
        path = args.record or os.path.join(
            RECORD_DIR, f"settle-{time.strftime('%Y-%m-%d')}.jsonl")
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(row, ensure_ascii=False) + "\n")
            print(f"  기록 {path}")
        except OSError as exc:
            # 기록을 못 남기는 것이 팔을 세워 둔 채 죽을 이유는 아니다.
            print(f"  ⚠ 기록 실패: {exc}")

    final = to_deg(io.read())
    fp = kin.forward(final, geom)
    fr = kin.signed_radius(final, geom)
    print(f"\n끝  TCP ({fp.x:.1f}, {fp.y:.1f}, {fp.z:.1f}) pitch {fp.pitch:.1f}° "
          f"signed_r {fr:.1f}mm")
    print("좌표 이동 가능" if fr >= 90 else "⚠ 아직 가드 안쪽이다")
    # ⚠ 토크를 끄지 않고 닫는다 — 끄면 그 자리에서 팔이 떨어진다(T70, §26).
    io.hold_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
