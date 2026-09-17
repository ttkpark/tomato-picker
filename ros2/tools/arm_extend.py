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

from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware import settle as st  # noqa: E402

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
STEP_DEG = 12.0        # 한 구간에서 어느 관절도 이 이상 안 움직인다
# 되먹임 기록이 남는 곳 — move5_check와 같은 규칙(날짜별 jsonl, 한 줄이 한 번의 이동).
RECORD_DIR = os.path.join(REPO, "docs", "시험기록")
SECS_PER_STEP = 1.2
R_TARGET = 150.0       # 좌표 가드(90mm)에서 충분히 떨어진 곳까지

# 목표 자세 — 집게가 **수평 앞**을 보는 표준 자세.
#   lift 80° = 상완이 거의 수직, elbow -80° = 전완이 수평, wrist 0° = 손목 곧게
#   → a1=80, a2=0, a3=0 이므로 pitch=0(수평), r≈250mm, z≈169mm
TARGET_DEG = {"shoulder_lift": 80.0, "elbow_flex": -80.0, "wrist_flex": 0.0}

# 정규화 가동범위는 -100..100이다. 98을 쓰는 건 끝에 2칸을 남겨 두려는 것.
LIMIT_NORM = 98.0


def clamp_norm(target_norm, limit=LIMIT_NORM):
    """목표 정규화값을 가동범위 안으로 **자른다** — 못 간다고 거절하지 않는다.

    왜 자르나 — 2026-09-18: 기본 목표 `elbow_flex=-80°`는 지금 보정표에서
    정규화 -137.8이다(가동범위 밖). 거절하면 팔이 특이점에 갇힌 채로 아무 데도
    못 간다. 잘라도 `signed_radius`가 219mm(가드 90mm의 2.4배)라 **목적은 달성된다**
    — 뻗는 자세의 목적은 pitch 0°가 아니라 가드를 빠져나오는 것이다.
    자른 관절은 부르는 쪽이 사람에게 말해 준다(무엇이 왜 덜 갔는지 보여야 한다).
    """
    out, hit = {}, []
    for j, v in target_norm.items():
        c = max(-limit, min(limit, float(v)))
        if abs(c - float(v)) > 1e-9:
            hit.append(j)
        out[j] = c
    return out, hit


def limit_violations(now_norm, step_norm, limit=LIMIT_NORM):
    """이 구간에서 **더 밖으로 나가는** 관절만 돌려준다.

    ⚠ 이미 범위 밖에 있는 관절을 "한계"라고 막으면 빠져나올 수가 없다 —
    바닥을 "지금 자리보다 1mm 아래"로 잡았다가 두 번 갇혔던 것과 같은 병이다
    (위 MOUNT_Z_MM 주석). 2026-09-18 실측: `shoulder_pan`이 정규화 98.81,
    `elbow_flex`가 103.16으로 **가만히 있는데도** 21구간 전부가 막혀 있었다.
    기준은 "범위 밖이냐"가 아니라 "이 걸음이 상황을 더 나쁘게 하느냐"다.
    """
    bad = []
    for j, v in step_norm.items():
        v = float(v)
        if abs(v) <= limit:
            continue
        if abs(v) > abs(float(now_norm.get(j, v))) + 1e-6:
            bad.append(j)
    return bad


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
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true", help="계산만 하고 안 움직인다")
    ap.add_argument("--target", default="",
                    help="목표 관절각(도) pan,lift,elbow,wflex,wroll. "
                         "비우면 기본 목표(집게가 수평 앞)")
    ap.add_argument("--hold", action="store_true",
                    help="끝나고 토크를 켠 채 둔다 (안 그러면 팔이 떨어진다)")
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
        if pose.z < -MOUNT_Z_MM + FLOOR_MARGIN_MM:
            note.append("바닥아래")
        if math.hypot(pose.x, pose.y) > geom.reach_max + 1e-6:
            note.append("사거리밖")
        # ⚠ 가동범위는 정규화값 -100..100이다 — 교시 자세 중심의 대칭이 아니다.
        nm = to_norm(degs)
        for j in limit_violations(now_norms, nm):
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
    if args.hold:
        # 토크를 끄지 않고 닫는다 — 끄면 그 자리에서 떨어진다.
        io.hold_close()
    else:
        io.hold_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
