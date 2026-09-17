#!/usr/bin/env python3
"""카테시안 유닛 자체검증 — **팔도 젯슨도 없이** 개발 PC에서 돈다.

    python tools/arm_cartesian_check.py

무엇을 확인하나:
  ① FK → IK → FK 왕복이 제자리로 돌아오는가 (기구학이 맞는가)
  ② 조그가 **요청한 만큼, 요청한 방향으로** 끝점을 옮기는가
  ③ 제자리 회전이 **정말 제자리인가** (끝점이 안 움직이는가)
  ④ 안전 검사가 실제로 막는가 (너무 작다/너무 크다/사거리 밖/바닥 아래/영점 없음)
  ⑤ 교시 자세(곧게 세운 팔)를 영점으로 잡으면 각도와 좌표가 맞게 나오는가
  ⑦ 상한(80mm)보다 먼 목표를 **여러 걸음으로** 가는가 (travel_to · 졸업기준3)
  ⑧ 직선 경로가 관절공간에서 막히면 **다른 길로 가는가** (졸업기준3 · T33)

왜 이걸 만들었나 — 좌표 이동의 버그는 "팔이 엉뚱한 데로 간다"로 나타나고,
그건 부러진 집게로 배우게 된다. 여기서 걸리는 종류의 실수(부호, 라디안/도,
왕복 불일치)는 실물에서 확인할 이유가 없다.
"""

from __future__ import annotations

import math
import os
import random
import sys
import tempfile

# ⚠ 여기만 서드파티가 없다 — kinematics·cartesian·config는 표준 라이브러리만 쓴다.
#   그래서 selfcheck_deps.require()를 안 부른다. 그 사실을 ros_selfcheck의
#   [의존성] 검사가 지킨다(numpy가 끼어들면 거기서 걸린다).

# 윈도우 콘솔(cp949)에서도 한글/기호가 안 깨지게 — main.py와 같은 처리.
# (이 도구의 존재 이유가 "개발 PC에서 돈다"인데, 출력의 em dash 하나에
#  UnicodeEncodeError로 죽으면 검증 자체를 못 한다.)
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8")

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from tomato_picker import config  # noqa: E402
from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware.cartesian import (  # noqa: E402
    MAX_TRAVEL_STEPS, MAX_TRAVEL_STEPS_JOINT, PROGRESS_STALL_STEPS, ArmStuck,
    CartesianArm, SimJointIO, plan_joint_steps, plan_steps,
)
from tomato_picker.hardware.kinematics import ArmGeometry  # noqa: E402
from tomato_picker.hardware.settle import SettleConfig, settle  # noqa: E402
from tomato_picker.config import ARM_CART_ZERO_POSE_DEG as ZERO_POSE  # noqa: E402

FAILED: list[str] = []
PASSED = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global PASSED
    if ok:
        PASSED += 1
        print(f"  ok   {name}" + (f"  ({detail})" if detail else ""))
    else:
        FAILED.append(name)
        print(f"  FAIL {name}  {detail}")


def expect_error(name: str, fn, must_contain: str = "") -> None:
    try:
        fn()
    except Exception as exc:  # noqa: BLE001 - 막혔는지가 관심사
        text = str(exc)
        check(name, must_contain in text, f"거절함: {text[:70]}")
        return
    check(name, False, "막았어야 하는데 통과했다")


# ----------------------------------------------------------------------

def test_roundtrip(geom: ArmGeometry) -> None:
    print("\n① 기구학 왕복 (FK → IK → FK)")
    random.seed(7)
    worst = 0.0
    tried = skipped = 0
    for _ in range(20000):
        j = {
            "shoulder_pan": random.uniform(-100, 100),
            "shoulder_lift": random.uniform(-20, 90),
            "elbow_flex": random.uniform(-150, -10),
            "wrist_flex": random.uniform(-100, 60),
            "wrist_roll": random.uniform(-180, 180),
        }
        pose = kin.forward(j, geom)
        if kin.signed_radius(j, geom) < 30:   # 몸통 뒤로 접힌 자세는 좌표가 모호하다
            skipped += 1
            continue
        tried += 1
        back = kin.inverse(pose, geom, seed_pan=j["shoulder_pan"])
        again = kin.forward(back, geom)
        worst = max(worst, abs(pose.x - again.x), abs(pose.y - again.y),
                    abs(pose.z - again.z), abs(pose.pitch - again.pitch))
        for name in kin.ARM_JOINTS:            # 관절값 자체도 돌아와야 한다
            worst = max(worst, abs(kin.wrap180(back[name] - j[name])))
    check("무작위 20000자세 왕복 오차 < 1e-6", worst < 1e-6,
          f"최대 {worst:.2e} · {tried}개 검사 · 접힌자세 {skipped}개 제외")


def test_jog() -> None:
    print("\n② 조그 — 요청한 방향으로 요청한 만큼")
    arm = fresh_arm()
    start = arm.pose()
    for axis, dx, dy, dz in (("x", 20, 0, 0), ("y", 0, 25, 0), ("z", 0, 0, 30),
                             ("-z", 0, 0, -15)):
        before = arm.pose()
        arm.jog(dx=dx, dy=dy, dz=dz)
        after = arm.pose()
        got = (after.x - before.x, after.y - before.y, after.z - before.z)
        err = max(abs(got[0] - dx), abs(got[1] - dy), abs(got[2] - dz))
        check(f"base {axis} {dx or dy or dz:+g}mm", err < 1e-6,
              f"실제 Δ=({got[0]:+.3f}, {got[1]:+.3f}, {got[2]:+.3f})")
    check("pitch는 조그로 안 변한다", abs(arm.pose().pitch - start.pitch) < 1e-9)

    print("\n   도구 좌표계 — 집게가 보는 쪽으로 전진")
    arm = fresh_arm()
    # ⚠ 2026-08-31 실측으로 l3가 95 → 168mm가 되면서 이 각이 -30°면 손목이
    #    사거리를 넘는다(팔이 커지면 같은 각도라도 손목이 훨씬 뒤로 물러난다).
    #    시험의 뜻(집게가 보는 쪽으로 전진)은 그대로고 크기만 줄인다.
    arm.jog(dpitch=-15)                      # 집게를 15° 더 숙인다
    before = arm.pose()
    arm.jog(dx=20, frame="tool")             # 보는 쪽으로 20mm
    after = arm.pose()
    d = (after.x - before.x, after.y - before.y, after.z - before.z)
    ph = math.radians(before.pitch)
    want = (20 * math.cos(ph), 0.0, 20 * math.sin(ph))
    check(f"tool 전진 20mm (pitch {before.pitch:.0f}°) = 수평{want[0]:.1f} / 수직{want[2]:+.1f}",
          max(abs(d[i] - want[i]) for i in range(3)) < 1e-6,
          f"Δ=({d[0]:+.2f}, {d[1]:+.2f}, {d[2]:+.2f})")
    check("전진해도 집게 각은 그대로", abs(after.pitch - before.pitch) < 1e-9)


def test_spin() -> None:
    print("\n③ 제자리 회전 — 끝점이 움직이면 실패다")
    arm = fresh_arm()
    arm.jog(dx=-40, dz=30)                   # 무대 한가운데쯤으로
    before = arm.pose()
    joints_before = arm.joints_deg()

    arm.spin(45, axis="roll")
    after = arm.pose()
    moved = math.dist((before.x, before.y, before.z), (after.x, after.y, after.z))
    check("roll 45° — 끝점 이동 0mm", moved < 1e-9, f"{moved:.2e}mm")
    check("roll 45° — roll만 45° 변함", abs(after.roll - before.roll - 45) < 1e-9)
    changed = [j for j in kin.ARM_JOINTS
               if abs(arm.joints_deg()[j] - joints_before[j]) > 1e-9]
    check("roll — 다른 관절은 안 움직인다", not changed, f"움직인 관절: {changed or '없음'}")

    before = arm.pose()
    arm.spin(-20, axis="pitch")
    after = arm.pose()
    moved = math.dist((before.x, before.y, before.z), (after.x, after.y, after.z))
    check("pitch -20° — 끝점 이동 0mm", moved < 1e-6, f"{moved:.2e}mm")
    check("pitch -20° — 각만 바뀜", abs(after.pitch - before.pitch + 20) < 1e-9)

    print("\n   yaw — 바닥을 볼 때만 된다(5축의 한계)")
    arm = fresh_arm()
    expect_error("서 있는 집게로 yaw 요청 → 거절", lambda: arm.spin(20, axis="yaw"),
                 "제자리 yaw 회전은 안 됩니다")
    # 위에서 내려다보고 집는 자세 — 이 팔에서 yaw가 되는 유일한 구간이다.
    arm = fresh_arm(TOPDOWN_POSE_DEG)
    before = arm.pose()
    check("집게가 바닥을 본다 (pitch≈-90°)", abs(before.pitch + 90) < 1e-6,
          f"pitch={before.pitch:.1f}°")
    arm.spin(30, axis="yaw")
    after = arm.pose()
    moved = math.dist((before.x, before.y, before.z), (after.x, after.y, after.z))
    check("yaw 30° — 끝점 이동 0mm", moved < 1e-9, f"{moved:.2e}mm")
    check("yaw 30° = roll -30° (축이 아래를 보므로 반대)",
          abs(after.roll - before.roll + 30) < 1e-9, f"roll {before.roll:.0f}→{after.roll:.0f}")


def test_guards() -> None:
    print("\n④ 안전 검사 — 막아야 할 것을 막는가")
    arm = fresh_arm()
    expect_error("0.01mm 조그 (서보 분해능 아래)", lambda: arm.jog(dx=0.01),
                 "실제로는 안 움직입니다")
    expect_error("300mm 점프 (한 번에 너무 큼)", lambda: arm.jog(dx=300),
                 "너무 큽니다")
    # 아래 셋은 **한 걸음 안쪽**의 목표라 크기 검사를 통과한다 — 그래야 작업영역
    # 검사가 실제로 도는지 확인된다(크기 검사가 먼저 막아버리면 시험이 헛돈다).
    low = fresh_arm(TOPDOWN_POSE_DEG)                    # z=76mm에서 시작
    expect_error("바닥 아래로 (z=10, 하한 15)", lambda: low.move_to(z=10), "바닥 아래")
    near = fresh_arm(TOPDOWN_POSE_DEG)                   # x=135mm에서 시작
    expect_error("몸통 안쪽으로 (x=85, 하한 90)", lambda: near.move_to(x=85), "너무 가깝")
    far = fresh_arm()
    # ⚠ 목표 x를 **기하에서 계산한다.** 숫자를 박아 두면 링크 길이를 고칠 때마다
    #    이 시험이 깨진다 — 2026-08-31 실측 반영 때 두 번 깨졌다(사거리가 346 →
    #    422 → 391mm로 움직였고, 박아 둔 320·390이 각각 "닿는다"·"한 걸음보다
    #    멀다"로 뜻이 바뀌었다). 한 걸음(80mm) 안쪽에서 가장 먼 곳을 고른다.
    x_far = far.pose().x + 79.0
    expect_error(f"사거리 밖 (x={x_far:.0f})", lambda: far.move_to(x=x_far), "사거리")

    arm = fresh_arm()
    arm.config.clear_zero()
    expect_error("영점 없이 이동 요청", lambda: arm.jog(dx=10), "기구학 영점이 없습니다")

    # 관절 한계 — 정규화 ±100이 가동 끝. 팔을 끝까지 밀면 걸려야 한다.
    arm = fresh_arm()
    hit = None
    for _ in range(200):
        try:
            arm.jog(dz=20)
        except RuntimeError as exc:
            hit = str(exc)
            break
    check("위로 계속 밀면 언젠가 막힌다", hit is not None,
          (hit or "")[:80])

    print("\n   특이점 보호 — 좌표 방향이 정해지지 않는 구간")
    back = fresh_arm({"shoulder_pan": 0.0, "shoulder_lift": 150.0, "elbow_flex": -30.0,
                      "wrist_flex": -30.0, "wrist_roll": 0.0})
    expect_error("몸통 뒤로 넘어간 자세에서 조그", lambda: back.jog(dx=10),
                 "몸통 뒤로 넘어가")


def test_travel() -> None:
    """상한보다 **먼 목표**로 여러 걸음에 가는가 (졸업기준3).

    2026-09-18 T26 실기: arm_extend로 특이점을 빠져나온 뒤 move5_check 5회가
    전부 한 걸음 상한(80mm)에 거절됐다(572~720mm). 상한은 좌표 오타 방어라
    키울 수 없으니 걸음을 늘려 푼다 — 여기서 확인하는 것은 ① 걸음 수가
    ceil(거리/상한) 이상인가 ② **각 걸음이 상한 이하인가**(쪼갠 척하고 한 번에
    뛰면 상한이 없는 것과 같다) ③ 도착점이 목표와 같은가다.
    """
    print("\n⑦ 먼 좌표로 여러 걸음 이동 (travel_to)")
    max_mm = config.ARM_CART_MAX_STEP_MM

    now = kin.ToolPose(x=200.0, y=0.0, z=100.0, pitch=0.0)
    for dist in (572.0, 686.0, 720.0, 696.0, 650.0):     # 09-18 실기의 거절 거리
        target = now.replace(x=now.x + dist)
        steps = plan_steps(now, target)
        need = math.ceil(dist / max_mm)
        hops = [math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                for a, b in zip([now] + steps[:-1], steps)]
        check(f"{dist:.0f}mm → {len(steps)}걸음 (ceil={need})", len(steps) >= need,
              f"걸음={len(steps)}")
        check(f"{dist:.0f}mm의 각 걸음이 상한 {max_mm:.0f}mm 이하",
              max(hops) <= max_mm + 1e-6, f"최대 {max(hops):.1f}mm")
    check("한 걸음 안쪽 목표는 한 걸음이다",
          len(plan_steps(now, now.replace(x=now.x + 50.0))) == 1)
    check("마지막 걸음은 정확히 목표다",
          plan_steps(now, now.replace(x=400.0))[-1].x == 400.0)
    big = plan_steps(now, now.replace(pitch=now.pitch + 170.0))
    check("회전이 크면 각도 상한으로도 쪼갠다",
          len(big) >= math.ceil(170.0 / config.ARM_CART_MAX_STEP_DEG)
          and max(abs(b.pitch - a.pitch) for a, b in zip([now] + big[:-1], big))
              <= config.ARM_CART_MAX_STEP_DEG + 1e-6,
          f"걸음={len(big)}")

    # 가짜 팔로 실제로 걸어 본다 — 계획만 맞고 실행이 안 되면 의미가 없다.
    arm = fresh_arm()
    start = arm.pose()
    far = start.replace(x=start.x - 150.0, z=start.z + 120.0)
    d = math.dist((start.x, start.y, start.z), (far.x, far.y, far.z))
    writes_before = arm._io.writes
    note = arm.travel_to(x=far.x, y=far.y, z=far.z)
    end = arm.pose()
    check(f"{d:.0f}mm를 travel_to로 갔다", "걸음으로 이동 완료" in note, note)
    check("걸음마다 한 번씩 썼다(한 번에 안 뛰었다)",
          arm._io.writes - writes_before >= math.ceil(d / max_mm),
          f"쓰기 {arm._io.writes - writes_before}회 / 필요 {math.ceil(d / max_mm)}회")
    check("도착점이 목표와 같다 (1mm 안)",
          math.dist((end.x, end.y, end.z), (far.x, far.y, far.z)) < 1.0,
          f"오차 {math.dist((end.x, end.y, end.z), (far.x, far.y, far.z)):.3f}mm")

    # 상한을 키워서 푼 게 아니라는 확인 — move_to는 여전히 거절해야 한다.
    arm2 = fresh_arm()
    expect_error("move_to는 여전히 80mm 상한을 지킨다",
                 lambda: arm2.move_to(x=arm2.pose().x - 150.0), "너무 큽니다")
    # 목표가 애초에 갈 수 없으면 **한 걸음도 움직이지 않는다** — 절반쯤 가 놓고
    # 거절하면 팔이 엉뚱한 자리에 서고 사람은 무엇이 틀렸는지 모른다.
    arm3 = fresh_arm()
    before = arm3.pose()
    writes = arm3._io.writes
    expect_error("사거리 밖 목표는 travel_to도 거절",
                 lambda: arm3.travel_to(x=before.x + 300.0), "사거리")
    check("거절당한 뒤 팔이 그대로 있다", arm3._io.writes == writes
          and math.dist((arm3.pose().x, arm3.pose().y, arm3.pose().z),
                        (before.x, before.y, before.z)) < 1e-6)
    check("걸음 수 상한이 있다(무한 루프 방어)", MAX_TRAVEL_STEPS >= 10,
          f"{MAX_TRAVEL_STEPS}걸음")


# 2026-09-18 젯슨 실측 — 이 팔의 캘리브레이션(lerobot range_min/max)과 영점.
#   ~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json
#   ~/arm_cartesian.json
# ⚠ 숫자를 여기 박아 둔 이유: 이 검사는 "어떤 팔이든 된다"가 아니라 **그날 5/5로
#   거절당한 바로 그 팔에서 되는가**를 묻는다. 팔을 다시 캘리브레이션하면 이
#   숫자도 같이 갱신해야 한다(그때는 이 검사가 빨개져서 알려 준다).
DEG_PER_TICK = 360.0 / 4096.0
JETSON_SPANS = {"shoulder_pan": (3330 - 560) * DEG_PER_TICK,
                "shoulder_lift": (3350 - 1009) * DEG_PER_TICK,
                "elbow_flex": (3159 - 808) * DEG_PER_TICK,
                "wrist_flex": (3099 - 735) * DEG_PER_TICK,
                "wrist_roll": 4095 * DEG_PER_TICK}
JETSON_ZERO = {"shoulder_pan": -11.516483516483516, "shoulder_lift": -18.24175824175824,
               "elbow_flex": -60.35164835164835, "wrist_flex": -1.7582417582417582,
               "wrist_roll": 0.0}
# arm_extend.py가 세운 시작 자세. elbow_flex는 정규화 -98(가동 끝)까지 잘린 값이라
# **한계에 붙어 있다** — 직교 직선이 첫 걸음부터 막히는 이유가 여기 있다.
CYCLE20_START_DEG = {"shoulder_pan": 133.3, "shoulder_lift": 80.0, "elbow_flex": -38.9,
                     "wrist_flex": 0.0, "wrist_roll": 88.8}
# 사이클20 실기가 뽑은 표적 5개(mm, 접근 pitch°). 기록 =
# docs/시험기록/move-to-point-2026-09-18.jsonl
CYCLE20_TARGETS = [(265.2, 181.9, 355.8, 15.2), (378.7, 7.5, 264.0, 4.5),
                   (315.3, -12.9, 400.3, 33.6), (291.3, -116.6, 372.4, 18.2),
                   (219.1, -75.4, 481.4, 56.6)]
CYCLE20_STANDOFF_MM = 30.0


class LaggyJointIO(SimJointIO):
    """지령의 일부만 따라가는 가짜 팔 — **실물 서보의 추종오차**를 흉내 낸다.

    SimJointIO는 지령이 즉시 정확히 이뤄진다고 본다. 그래서 "계획한 웨이포인트를
    그대로 따라가면 된다"는 틀린 가정이 시뮬에서는 절대 안 깨진다 — 실기에서만
    깨졌다(2026-09-18: 관절 오차 최대 4.2° ≈ 팔 끝에서 ~28mm).
    """

    def __init__(self, *, deliver: float = 0.7, **kw) -> None:
        super().__init__(**kw)
        self.deliver = float(deliver)

    def write(self, target: dict, secs: float) -> None:
        partial = {k: self.joints.get(k, 0.0)
                   + (float(v) - self.joints.get(k, 0.0)) * self.deliver
                   for k, v in target.items()}
        super().write(partial, secs)


class DroopJointIO(SimJointIO):
    """**중력이 누른 만큼 못 미친 자리에서 멈추는** 가짜 팔 — 처짐(정상상태).

    LaggyJointIO와 결정적으로 다르다: 저쪽은 "남은 거리의 70%"라 **같은 지령을
    다시 보내기만 해도** 언젠가 닿는다. 이쪽은 지령에서 늘 같은 만큼 모자란
    자리에 서므로 같은 지령을 몇 번을 보내도 그 자리다 — 2026-09-18 실기의
    관절 오차(A자세 1.1~1.3° / B자세 4.0~4.9°가 자세마다 **거의 일정**)가 이
    모양이었다(docs/인수인계-2026-09-04.md §22). 이걸 지우는 유일한 길이
    "오차만큼 더" 주는 되먹임이다.

    `stiction`을 주면 보정의 일부만 먹는다(§22-1: 4.92→3.70→3.50→2.84°로
    기하급수로 안 줄었다 — 정지마찰·백래시가 섞여 있다는 뜻).
    """

    def __init__(self, *, droop_norm: float = 3.0, joint: str = "shoulder_lift",
                 stiction: float = 0.0, **kw) -> None:
        super().__init__(**kw)
        self.droop_norm = float(droop_norm)
        self.droop_joint = joint
        self.stiction = float(stiction)

    def write(self, target: dict, secs: float) -> None:
        out = {}
        for k, v in target.items():
            v = float(v)
            if k == self.droop_joint:
                want = v - self.droop_norm
                # 정지마찰: 지금 자리에서 want까지의 일부만 실제로 간다.
                now = self.joints.get(k, 0.0)
                v = want - (want - now) * self.stiction
            out[k] = v
        super().write(out, secs)


class CountingCall:
    """부른 횟수와 인자를 세는 껍데기 — settle이 **몇 번 보냈나**를 본다."""

    def __init__(self, fn=None) -> None:
        self.calls: list = []
        self._fn = fn

    def __call__(self, *a):
        self.calls.append(a[0] if a else None)
        if self._fn:
            return self._fn(*a)


def test_settle() -> None:
    print("\n⑨ 마지막 걸음의 처짐 되먹임 (settle — 졸업기준3)")
    cfg = SettleConfig()
    check("세 숫자는 config.py에서 온다(코드에 안 박혔다)",
          (cfg.rounds, cfg.stop_deg, cfg.min_gain, cfg.stall_rounds)
          == (config.ARM_SETTLE_ROUNDS, config.ARM_SETTLE_STOP_DEG,
              config.ARM_SETTLE_MIN_GAIN, config.ARM_SETTLE_STALL_ROUNDS),
          f"{cfg.rounds}회 / {cfg.stop_deg}° / {cfg.min_gain} / {cfg.stall_rounds}회")

    want = {"shoulder_lift": 40.0, "elbow_flex": -30.0}

    # ① 수렴하면 멈춘다 — 처짐이 지워지면 더 안 보낸다.
    droop = {"shoulder_lift": 3.0}
    state = {j: want[j] - droop.get(j, 0.0) for j in want}
    sent = CountingCall(lambda cmd: state.update(
        {j: float(v) - droop.get(j, 0.0) for j, v in cmd.items()}))
    res = settle(want, measure=lambda: dict(state), send=sent)
    check("수렴하면 멈춘다 (되먹임 1회로 문턱 아래)",
          res.stop == "converged" and len(sent.calls) == 1
          and res.err_deg <= cfg.stop_deg,
          f"{res.stop} · {len(sent.calls)}회 · {res.start_deg:.2f}°→{res.err_deg:.2f}°")
    lift = "shoulder_lift"
    check("같은 값을 다시 보내는 게 아니다 — 목표를 지나친 지령을 보낸다",
          sent.calls[0][lift] > want[lift] + 1e-6,
          f"목표 {want[lift]:.1f}° → 지령 {sent.calls[0][lift]:.1f}°")

    # ①-b **초과지령을 계속 물고 있어야 서는 팔**(스프링: 실제 = 지령의 70%).
    #     2026-09-18 실기가 이 모양이었다 — `목표 + (목표-실제)`로 매회 다시 세우면
    #     4.83°→1.23°로 좋아졌다가 1.67→2.21°로 **되돌아갔다**(직전 보정을 내려놓으니
    #     중력이 도로 끌어내렸다). 누적(지령 += 오차)만 이 팔에서 수렴한다.
    hold = 0.7
    spring = {j: want[j] * hold for j in want}
    sent_b = CountingCall(lambda cmd: spring.update(
        {j: float(cmd[j]) * hold for j in cmd}))
    res_b = settle(want, measure=lambda: dict(spring), send=sent_b)
    check("초과지령을 물고 있어야 서는 팔에서도 수렴한다(보정을 누적한다)",
          res_b.stop == "converged" and not res_b.restored,
          f"{res_b.stop} · {res_b.start_deg:.2f}°→{res_b.err_deg:.2f}° "
          f"· {len(sent_b.calls)}회")

    # ② 포화면 무한히 안 돈다 — 아무리 밀어도 안 움직이는 팔.
    stuck = {j: want[j] - 4.0 for j in want}
    sent2 = CountingCall()
    res2 = settle(want, measure=lambda: dict(stuck), send=sent2)
    check("포화면 무한히 안 돈다 (연속 2회 안 줄면 그만)",
          res2.stop == "saturated" and len(sent2.calls) == cfg.stall_rounds,
          f"{res2.stop} · {len(sent2.calls)}회 보냄 / 예산 {cfg.rounds}회")
    check("포화 판정이 예산보다 먼저 온다(그래야 뜻이 있다)",
          cfg.stall_rounds < cfg.rounds, f"{cfg.stall_rounds} < {cfg.rounds}")

    # ③ 예산 — 조금씩 줄기만 해도 N회에서 멈춘다.
    slow = {j: want[j] - 6.0 for j in want}
    sent3 = CountingCall(lambda cmd: slow.update(
        {j: want[j] - (want[j] - slow[j]) * 0.5 for j in slow}))
    res3 = settle(want, measure=lambda: dict(slow), send=sent3)
    check("반씩만 줄면 예산에서 멈춘다",
          res3.stop == "budget" and len(sent3.calls) == cfg.rounds,
          f"{res3.stop} · {len(sent3.calls)}회 · {res3.start_deg:.2f}°→{res3.err_deg:.2f}°")

    # ④ 소프트 한계에 눌린 관절은 오차에서 뺀다 (§22-1의 elbow_flex).
    #    안 그러면 "지울수록 오차가 커지는" 거짓 보고가 나온다.
    hard = {"shoulder_lift": want["shoulder_lift"] - 3.0,
            "elbow_flex": want["elbow_flex"] + 2.0}

    def deliver(cmd):
        out = dict(cmd)
        # elbow_flex는 이미 가동 끝이라 목표보다 더는 못 준다.
        out["elbow_flex"] = max(out["elbow_flex"], want["elbow_flex"])
        return out

    def send4(cmd):
        for j, v in cmd.items():
            if j == "elbow_flex":
                continue          # 한계에 눌려 실제로는 안 움직인다
            hard[j] = float(v) - 3.0

    res4 = settle(want, measure=lambda: dict(hard), send=send4, deliver=deliver)
    check("한계에 눌린 관절을 알아낸다", res4.clamped == ("elbow_flex",),
          str(res4.clamped))
    check("눌린 관절은 오차에서 빠진다 (지울수록 커지는 거짓 보고 방지)",
          res4.err_deg < res4.start_deg and res4.rounds[-1].worst != "elbow_flex",
          f"{res4.start_deg:.2f}° → {res4.err_deg:.2f}° (worst={res4.rounds[-1].worst})")

    # ④-b 되먹임이 **나빠지면** 가장 좋았던 지령으로 되돌린다 (2026-09-18 실기 A자세).
    #     한 관절을 고치면 다른 관절이 딸려 움직인다 — 더 틀어 놓고 끝나면 안 쓰느니만 못하다.
    worse = {"shoulder_lift": want["shoulder_lift"] - 3.0, "elbow_flex": want["elbow_flex"]}
    seq = {"n": 0}
    best_cmd = {}

    def send_worse(cmd):
        seq["n"] += 1
        if seq["n"] == 1:                    # 1회차는 좋아진다
            best_cmd.update(cmd)
            worse["shoulder_lift"] = want["shoulder_lift"] - 0.9
        elif cmd == best_cmd:                # 되돌린 지령은 그 자리를 되찾는다
            worse["shoulder_lift"] = want["shoulder_lift"] - 0.9
        else:                                # 그 뒤로는 도로 나빠진다
            worse["shoulder_lift"] = want["shoulder_lift"] - 2.0 - seq["n"] * 0.5

    res_w = settle(want, measure=lambda: dict(worse), send=send_worse)
    check("나빠지면 가장 좋았던 지령으로 되돌린다", res_w.restored
          and abs(res_w.err_deg - 0.9) < 1e-6,
          f"{res_w.stop} · 되돌림={res_w.restored} · 최종 {res_w.err_deg:.2f}°")
    check("되돌리는 것은 한 번뿐이다(여기서 또 재면 루프가 된다)",
          seq["n"] <= SettleConfig().rounds + 1, f"{seq['n']}회 보냄")

    # ⑤ 껐으면 한 번도 안 보낸다. 그래도 0회차 측정은 남는다.
    sent5 = CountingCall()
    res5 = settle(want, measure=lambda: {j: want[j] - 5.0 for j in want},
                  send=sent5, cfg=SettleConfig(rounds=0))
    check("되먹임을 끄면 한 번도 안 보낸다(그래도 오차는 잰다)",
          not sent5.calls and res5.stop == "off" and res5.err_deg > 4.0,
          f"{res5.stop} · {len(res5.rounds)}회차 기록 · {res5.err_deg:.2f}°")

    # ⑥ 분해능 아래 보정은 **보내지 않는다** (이 저장소의 1번 병).
    tiny = {j: want[j] - 0.3 for j in want}
    sent6 = CountingCall()
    res6 = settle(want, measure=lambda: dict(tiny), send=sent6,
                  cfg=SettleConfig(stop_deg=0.05, min_cmd_deg=1.0))
    check("서보 분해능 아래 보정은 보냈다고 하지 않는다",
          res6.stop == "undeliverable" and not sent6.calls, res6.stop)

    # ⑦ 발산 즉시 포화 — gain < -min_gain이면 추가 지령 없이 SATURATED. (T52)
    #   2026-09-18 실기 기록(arm-load-boundary-2026-09-18.jsonl) 4건:
    #   0.66°→1.40°→2.21°→3.39°(3~5배 발산), stall_rounds=2를 기다리는 동안 악화됨.
    #   직전보다 오차가 min_gain 비율 이상 커진 것이 확인되면 즉시 멈춰야 한다.
    diverge_state = {"shoulder_lift": want["shoulder_lift"] - 4.0,
                     "elbow_flex": want["elbow_flex"]}
    div_seq = {"n": 0}

    def send_diverge(cmd):
        div_seq["n"] += 1
        # 보낼수록 나빠진다 — 실기 Line1의 0.66°→1.40°→2.21° 패턴을 흉내낸다.
        diverge_state["shoulder_lift"] -= 2.0

    sent_div = CountingCall(send_diverge)
    # stall_rounds=2, min_gain=0.10로 기본값 사용
    res_div = settle(want, measure=lambda: dict(diverge_state), send=sent_div)
    check("발산(gain < -min_gain)이면 stall_rounds만큼 안 기다리고 즉시 SATURATED",
          res_div.stop == "saturated" and sent_div.calls
          and all(
              (rnd.gain or 0.0) < -cfg.min_gain
              for rnd in res_div.rounds[1:]  # 0회차는 gain 없음
              if rnd.worst != "되돌림"
          ),
          f"{res_div.stop} · {len(sent_div.calls)}회 · "
          f"gains={[round(r.gain, 3) for r in res_div.rounds if r.gain is not None]}")

    # ⑦-b 발산 즉시 포화가 추가 지령 횟수를 줄인다 — rounds 예산보다 적어야 한다.
    #   복구(restore) 지령 1회가 추가될 수 있어 <= stall_rounds + 1이 상한이다.
    check("발산 즉시 종료는 rounds 예산을 다 쓰지 않는다",
          len(sent_div.calls) <= cfg.rounds,
          f"보낸 횟수={len(sent_div.calls)} / rounds={cfg.rounds}")

    # ⑦ 실제 이동에 붙었는가 — 처짐 팔로 travel_to를 끝까지 간다.
    io_ = DroopJointIO(spans=JETSON_SPANS, droop_norm=2.5, stiction=0.35)
    arm = CartesianArm(io_, path=tmp_path())
    arm.config.set_zero(JETSON_ZERO)
    io_.joints.update(arm.to_norms(CYCLE20_START_DEG))
    goal = standoff_pose(*CYCLE20_TARGETS[2])
    note = arm.travel_to(x=goal.x, y=goal.y, z=goal.z, pitch=goal.pitch, roll=goal.roll)
    got = arm.pose()
    gap = math.dist((got.x, got.y, got.z), (goal.x, goal.y, goal.z))
    check("이동 뒤 되먹임이 실제로 돈다", (arm.last_settle or {}).get("sent", 0) >= 1,
          str(arm.last_settle)[:90])
    check("되먹임 횟수와 최종 오차가 기록에 남는다",
          {"sent", "err_deg", "start_err_deg", "stop"} <= set(arm.last_settle or {}),
          f"sent={arm.last_settle['sent']} err={arm.last_settle['err_deg']}° "
          f"stop={arm.last_settle['stop']}")
    check("사람이 읽는 말에도 되먹임이 보인다", "되먹임" in note, note[-60:])
    check(f"처짐 팔도 15mm 안으로 도착한다 (졸업기준3) — {gap:.1f}mm", gap <= 15.0,
          f"{gap:.2f}mm")

    # 되먹임을 껐을 때보다 **실제로 가까이** 서는가 — 이게 이 기능의 존재 이유다.
    io2 = DroopJointIO(spans=JETSON_SPANS, droop_norm=2.5, stiction=0.35)
    arm2 = CartesianArm(io2, path=tmp_path())
    arm2.config.set_zero(JETSON_ZERO)
    io2.joints.update(arm2.to_norms(CYCLE20_START_DEG))
    arm2._settle = lambda end_degs, geom, secs: settle(
        end_degs, measure=lambda: arm2.to_degrees(io2.read()),
        send=lambda d: None, cfg=SettleConfig(rounds=0))
    arm2.travel_to(x=goal.x, y=goal.y, z=goal.z, pitch=goal.pitch, roll=goal.roll)
    raw = arm2.pose()
    gap0 = math.dist((raw.x, raw.y, raw.z), (goal.x, goal.y, goal.z))
    check(f"되먹임이 도착 오차를 줄인다 ({gap0:.1f}mm → {gap:.1f}mm)",
          gap < gap0 - 1.0, f"껐을 때 {gap0:.2f}mm / 켰을 때 {gap:.2f}mm")


def jetson_arm(degrees: dict | None = None) -> CartesianArm:
    """2026-09-18 젯슨의 보정표·영점을 그대로 쓰는 가짜 팔."""
    io_ = SimJointIO(spans=JETSON_SPANS)
    arm = CartesianArm(io_, path=tmp_path())
    arm.config.set_zero(JETSON_ZERO)
    io_.joints.update(arm.to_norms(degrees or CYCLE20_START_DEG))
    return arm


def standoff_pose(x: float, y: float, z: float, pitch: float) -> kin.ToolPose:
    """arm_node가 `/arm/move_to_point`에서 만드는 것과 같은 스탠드오프 자세."""
    pose = kin.ToolPose(x=x, y=y, z=z, pitch=pitch)
    dx, dy, dz = kin.offset_in_tool_frame(pose, -CYCLE20_STANDOFF_MM, 0.0, 0.0)
    return pose.replace(x=pose.x + dx, y=pose.y + dy, z=pose.z + dz)


def test_travel_path() -> None:
    """직교 직선이 관절공간에서 불가능할 때 **다른 길로 가는가** (T33).

    2026-09-18 실기: 목표 사전검사는 5/5 통과했는데 5/5가 **첫 걸음**에서
    거절됐다(`elbow_flex` 정규화 -124~-133, 한계 ±98). 직교공간의 직선은
    관절공간의 직선이 아니다 — 양 끝이 다 갈 수 있어도 그 사이는 아닐 수 있다.

    여기서 못 박는 것: ① 그 시작 자세에서 그 표적 5개로 **갈 길이 있다**
    ② 직선만으로는 여전히 막힌다(= 이 검사가 실제로 대체 경로를 시험한다)
    ③ 관절공간 보간의 성질(끝점 일치·한 걸음 상한 둘·볼록성)
    ④ 직선이 되는 목표에서는 **직선을 쓴다**(대체 경로가 기본이 되면 안 된다).
    """
    print("\n⑧ 직선이 막히면 다른 길로 (travel_to · T33)")
    geom = ArmGeometry()

    # ③ 계획의 성질 — 실행 없이 순수 계산으로
    end = {"shoulder_pan": -40.0, "shoulder_lift": 30.0, "elbow_flex": -60.0,
           "wrist_flex": 20.0, "wrist_roll": -30.0}
    steps = plan_joint_steps(CYCLE20_START_DEG, end, geom)
    check("관절 보간의 마지막 걸음이 정확히 목표 관절각이다",
          all(abs(steps[-1][j] - end[j]) < 1e-9 for j in kin.JOINTS),
          str({j: round(steps[-1][j], 3) for j in kin.JOINTS}))
    worst_deg = max(abs(b[j] - a[j]) for a, b in zip([CYCLE20_START_DEG] + steps[:-1], steps)
                    for j in kin.JOINTS)
    check(f"관절 한 걸음이 {config.ARM_CART_MAX_STEP_JOINT_DEG:.0f}° 이하",
          worst_deg <= config.ARM_CART_MAX_STEP_JOINT_DEG + 1e-6, f"최대 {worst_deg:.2f}°")
    poses = [kin.forward(d, geom) for d in [CYCLE20_START_DEG] + steps]
    worst_mm = max(math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                   for a, b in zip(poses, poses[1:]))
    check(f"관절 한 걸음의 끝점 이동도 {config.ARM_CART_MAX_STEP_MM:.0f}mm 이하 "
          "(각도 상한 하나로는 못 막는다)",
          worst_mm <= config.ARM_CART_MAX_STEP_MM + 1e-6, f"최대 {worst_mm:.1f}mm")
    check("관절 보간은 걸음 수 상한이 있다", MAX_TRAVEL_STEPS_JOINT >= MAX_TRAVEL_STEPS,
          f"{MAX_TRAVEL_STEPS_JOINT}걸음")

    # ② 직선만으로는 막힌다 — 이 전제가 깨지면 아래 ①이 아무것도 증명하지 않는다
    probe = jetson_arm()
    now = probe.pose()
    blocked = 0
    for x, y, z, pitch in CYCLE20_TARGETS:
        target = standoff_pose(x, y, z, pitch)
        line = plan_steps(now, target)
        if probe._walk(probe._io.read(), now, line, geom, joint_space=False) is not None:  # noqa: SLF001
            blocked += 1
    check("사이클20의 표적 5개는 직선 경로로는 여전히 막힌다(검사가 헛돌지 않는다)",
          blocked == 5, f"{blocked}/5만 막힌다")

    # ① 그래도 **갈 길이 있다** — 졸업기준3이 걸려 있던 자리
    reached = 0
    for i, (x, y, z, pitch) in enumerate(CYCLE20_TARGETS, 1):
        arm = jetson_arm()
        target = standoff_pose(x, y, z, pitch)
        try:
            note = arm.travel_to(x=target.x, y=target.y, z=target.z,
                                 pitch=target.pitch, roll=target.roll)
        except RuntimeError as exc:
            check(f"표적{i}로 갈 길이 있다", False, str(exc))
            continue
        end_pose = arm.pose()
        err = math.dist((end_pose.x, end_pose.y, end_pose.z), (target.x, target.y, target.z))
        check(f"표적{i}({x:.0f},{y:.0f},{z:.0f})로 갈 길이 있다 — 오차 {err:.2f}mm",
              err < 1.0 and "관절공간" in note, note[:90])
        reached += err < 1.0
    check("사이클20 표적 5개 전부 도달 (졸업기준3의 경로 장애물)", reached == 5,
          f"{reached}/5")

    # ④ 직선이 되는 목표에서는 직선을 쓴다 — 대체 경로가 기본이 되면 안 된다
    plain = fresh_arm()
    start = plain.pose()
    note = plain.travel_to(x=start.x - 150.0, z=start.z + 120.0)
    check("직선으로 갈 수 있으면 직선으로 간다(대체 경로를 안 쓴다)",
          "관절공간" not in note, note)

    # 서보가 지령에 못 미쳐도 간다 — 걸음마다 **남은 길을 다시 짠다**
    # (2026-09-18 실기 3/5가 "3걸음 중 2번째에서 110mm는 너무 큽니다"였다:
    #  고정 웨이포인트를 쓰면 못 미친 만큼이 다음 걸음에 얹혀 가드에 걸린다).
    lag = LaggyJointIO(spans=JETSON_SPANS, deliver=0.7)
    slow = CartesianArm(lag, path=tmp_path())
    slow.config.set_zero(JETSON_ZERO)
    lag.joints.update(slow.to_norms(CYCLE20_START_DEG))
    goal = standoff_pose(*CYCLE20_TARGETS[0])
    try:
        note = slow.travel_to(x=goal.x, y=goal.y, z=goal.z,
                              pitch=goal.pitch, roll=goal.roll)
        got = slow.pose()
        gap = math.dist((got.x, got.y, got.z), (goal.x, goal.y, goal.z))
        check("서보가 지령의 70%만 따라가도 한 걸음 상한에 안 걸린다",
              "너무 큽니다" not in note, note[:100])
        check(f"그래도 목표 근처까지 간다 — 남은 거리 {gap:.0f}mm", gap < 80.0,
              f"{gap:.1f}mm 남음")
    except RuntimeError as exc:
        check("서보가 지령의 70%만 따라가도 한 걸음 상한에 안 걸린다", False, str(exc))

    # 서보가 아예 안 따라오면 **예산을 다 태우지 않고** 남은 거리를 말하고 멈춘다
    # (09-18 실기 3/5가 32걸음을 다 태우고 err=None으로 끝났다 — 몇 mm 모자랐는지
    #  조차 기록에 안 남았다. 다시 짜기가 영원한 재시도가 되면 안 된다).
    stuck_io = LaggyJointIO(spans=JETSON_SPANS, deliver=0.0)
    stuck_arm = CartesianArm(stuck_io, path=tmp_path())
    stuck_arm.config.set_zero(JETSON_ZERO)
    stuck_io.joints.update(stuck_arm.to_norms(CYCLE20_START_DEG))
    goal2 = standoff_pose(*CYCLE20_TARGETS[1])
    stuck_writes_before = stuck_io.writes
    try:
        stuck_arm.travel_to(x=goal2.x, y=goal2.y, z=goal2.z,
                            pitch=goal2.pitch, roll=goal2.roll)
        check("서보가 안 따라오면 ArmStuck을 올린다(ok=True로 성공인 체하지 않는다)",
              False, "예외가 없었다 — travel_to가 성공으로 반환했다")
    except ArmStuck as exc:
        msg = str(exc)
        check("서보가 안 따라오면 '더 안 갑니다'라고 남은 거리를 말한다",
              "더 안 갑니다" in msg and "mm" in msg, msg[:110])
        check("그때 예산(32걸음)을 다 태우지 않는다",
              stuck_io.writes - stuck_writes_before <= PROGRESS_STALL_STEPS + 1,
              f"쓰기 {stuck_io.writes - stuck_writes_before}회 / stall {PROGRESS_STALL_STEPS}회")

    # 갈 수 없는 목표는 **한 걸음도 안 움직이고** 목표 검사에서 먼저 걸린다
    guard = jetson_arm()
    before = guard._io.writes  # noqa: SLF001
    expect_error("바닥 아래 목표는 경로를 짜기 전에 거절",
                 lambda: guard.travel_to(z=5.0), "바닥 아래")
    check("거절당한 뒤 팔이 그대로 있다(관절공간 경로도 안 시도)",
          guard._io.writes == before, f"쓰기 {guard._io.writes - before}회")  # noqa: SLF001

    # 둘 다 막히면 **둘 다** 이유를 말한다 — 한쪽만 말하면 다음 사람이 헤맨다.
    # 아래 자리는 실제로 두 길이 다 바닥을 뚫는다(무작위 탐색으로 찾은 실례).
    stuck = jetson_arm({"shoulder_pan": 7.4, "shoulder_lift": 12.5, "elbow_flex": -31.3,
                        "wrist_flex": -52.3, "wrist_roll": 0.0})
    writes = stuck._io.writes  # noqa: SLF001
    try:
        stuck.travel_to(x=-169.5, y=226.6, z=286.6, pitch=-8.7, roll=0.0)
        check("두 길이 다 막히면 둘 다 이유를 말한다", False, "거절하지 않았다")
    except RuntimeError as exc:
        check("두 길이 다 막히면 둘 다 이유를 말한다",
              "직선 경로:" in str(exc) and "관절공간 경로:" in str(exc), str(exc)[:120])
        check("두 길이 다 막혀도 팔은 제자리다(반쯤 가다 멈추지 않는다)",
              stuck._io.writes == writes, f"쓰기 {stuck._io.writes - writes}회")  # noqa: SLF001


def test_zero_pose(geom: ArmGeometry) -> None:
    """영점 = "어깨는 정면, 나머지는 곧게 위로". 이게 틀리면 전부 90° 틀어진다."""
    print("\n⑤ 교시 자세 — 곧게 세운 팔이 영점")
    io_ = SimJointIO()
    arm = CartesianArm(io_, path=tmp_path())
    arm.config.set_zero({j: 0.0 for j in kin.JOINTS})   # "지금 세워 뒀다"

    degs = arm.joints_deg()
    want = ZERO_POSE
    err = max(abs(degs[j] - want[j]) for j in kin.JOINTS)
    check("세운 자세를 pan 0 / lift 90 / elbow 0 / wrist 0으로 읽는다", err < 1e-9,
          " ".join(f"{j[:5]}={degs[j]:.0f}" for j in kin.JOINTS))

    pose = arm.pose()
    up = geom.z0 + geom.l1 + geom.l2 + geom.l3
    # ⚠ 곧게 세워도 집게는 pan축 **바로 위가 아니다** — d0(2번 축의 수평 오프셋)
    #    만큼 비켜서 있다. 2026-08-31 실측에서 d0 = -31.5mm(뒤쪽)로 밝혀졌고,
    #    그 전까지 0으로 두었던 탓에 이 시험이 "x는 0"이라고 적혀 있었다.
    check("집게 높이 = z0+l1+l2+l3, 수평위치 = d0",
          abs(pose.x - geom.d0) < 1e-9 and abs(pose.y) < 1e-9
          and abs(pose.z - up) < 1e-9,
          f"x={pose.x:.1f} (기대 {geom.d0:.1f}) y={pose.y:.1f} "
          f"z={pose.z:.1f} (기대 {up:.0f})")
    check("집게가 하늘을 본다 (pitch +90°)", abs(pose.pitch - 90) < 1e-9,
          f"pitch={pose.pitch:.1f}°")
    # 세운 자세의 수평거리는 |d0|뿐이라 여전히 가드에 막힌다. 다만 d0가 음수면
    # signed_radius도 음수라 가드가 "몸통 뒤로 넘어가 있습니다" 쪽 문장을 고른다
    # — 둘 다 같은 가드이고, 여기서는 **막혔는지**만 본다.
    expect_error("세운 채로 조그하면 막힌다(특이점)", lambda: arm.jog(dx=10),
                 "수평거리")

    # 교시 자세가 파일에 남아야 한다 — 안 남으면 나중에 기준을 바꿀 때
    # 이미 잡아둔 팔이 조용히 90° 틀어진다.
    saved = arm.config.snapshot()["ref_deg"]
    check("교시 자세가 설정 파일에 기록된다",
          all(abs(saved[j] - want[j]) < 1e-9 for j in kin.JOINTS), str(saved))

    # 실제로 쓰는 자세로 내려서 좌표가 맞는지 (수평으로 편 자세 = lift 0)
    io_.joints.update(arm.to_norms({"shoulder_pan": 0.0, "shoulder_lift": 0.0,
                                    "elbow_flex": 0.0, "wrist_flex": 0.0,
                                    "wrist_roll": 0.0}))
    flat = arm.pose()
    check("거기서 수평으로 펴면 x = 최대 사거리",
          abs(flat.x - geom.reach_max) < 1e-9 and abs(flat.z - geom.z0) < 1e-9,
          f"x={flat.x:.1f} z={flat.z:.1f}")


def test_snapshot() -> None:
    print("\n⑥ 대시보드 스냅샷")
    arm = fresh_arm()
    snap = arm.snapshot()
    ok = (snap["ready"] and snap["pose"] and not snap["stale"]
          and snap["joints"] is not None and snap["error"] is None)
    check("정상 스냅샷", bool(ok), str(snap["pose"]))
    lock = arm._io.busy_lock()               # noqa: SLF001 - 일부러 바쁘게 만든다
    lock.acquire()
    try:
        import threading
        result = {}
        t = threading.Thread(target=lambda: result.update(other=arm.snapshot()))
        t.start(); t.join(timeout=2)
        check("이동 중 폴링은 기다리지 않고 stale을 준다",
              t.is_alive() is False and result.get("other", {}).get("stale") is True)
    finally:
        lock.release()


# ----------------------------------------------------------------------

def tmp_path() -> str:
    fd, path = tempfile.mkstemp(prefix="arm_cart_test_", suffix=".json")
    os.close(fd)
    os.unlink(path)
    return path


# 시험 시작 자세 — 관절 전부 0(앞으로 수평 = 최대 사거리)에서는 어느 쪽으로도
# 못 가므로, 실제로 쓰는 "약간 접고 집게를 숙인" 자세에서 시작한다.
WORK_POSE_DEG = {"shoulder_pan": 0.0, "shoulder_lift": 75.0, "elbow_flex": -85.0,
                 "wrist_flex": -20.0, "wrist_roll": 0.0}
# 위에서 내려다보는 자세(pitch = -90°) — 제자리 yaw 회전이 되는 유일한 구간.
TOPDOWN_POSE_DEG = {"shoulder_pan": 0.0, "shoulder_lift": 90.0, "elbow_flex": -90.0,
                    "wrist_flex": -90.0, "wrist_roll": 0.0}


def fresh_arm(degrees: dict[str, float] | None = None) -> CartesianArm:
    """가짜 팔 + 영점 등록 + 작업 자세로 이동해 둔 유닛.

    영점은 "모든 정규화값 0 = 관절 전부 0°"로 잡는다(시뮬이니 그렇게 정의할 수
    있다). 실물에서는 사람이 팔을 수평으로 펴고 [영점 등록]을 누르는 그 절차다.
    """
    io_ = SimJointIO()
    arm = CartesianArm(io_, path=tmp_path())
    arm.config.set_zero({j: 0.0 for j in kin.JOINTS})
    io_.joints.update(arm.to_norms(degrees or WORK_POSE_DEG))
    return arm


def main() -> int:
    geom = ArmGeometry()
    print(f"링크 길이: z0={geom.z0} d0={geom.d0} l1={geom.l1} l2={geom.l2} l3={geom.l3} "
          f"→ 최대 사거리 {geom.reach_max:.0f}mm")
    print("교시 자세(= [영점 등록]을 누를 때의 자세): "
          + " ".join(f"{j}={ZERO_POSE[j]:.0f}°" for j in kin.JOINTS))
    print(f"  그 자세의 집게 위치: {kin.forward(ZERO_POSE, geom).as_dict()}")

    test_roundtrip(geom)
    test_jog()
    test_spin()
    test_guards()
    test_zero_pose(geom)
    test_snapshot()
    test_travel()
    test_travel_path()
    test_settle()

    print()
    if FAILED:
        print(f"❌ {len(FAILED)}개 실패 / {PASSED + len(FAILED)}개 중")
        for name in FAILED:
            print(f"   - {name}")
        return 1
    print(f"✅ 전부 통과 ({PASSED}개)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
