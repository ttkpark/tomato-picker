#!/usr/bin/env python3
"""카테시안 유닛 자체검증 — **팔도 젯슨도 없이** 개발 PC에서 돈다.

    python tools/arm_cartesian_check.py

무엇을 확인하나:
  ① FK → IK → FK 왕복이 제자리로 돌아오는가 (기구학이 맞는가)
  ② 조그가 **요청한 만큼, 요청한 방향으로** 끝점을 옮기는가
  ③ 제자리 회전이 **정말 제자리인가** (끝점이 안 움직이는가)
  ④ 안전 검사가 실제로 막는가 (너무 작다/너무 크다/사거리 밖/바닥 아래/영점 없음)
  ⑤ 교시 자세(곧게 세운 팔)를 영점으로 잡으면 각도와 좌표가 맞게 나오는가

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

# 윈도우 콘솔(cp949)에서도 한글/기호가 안 깨지게 — main.py와 같은 처리.
# (이 도구의 존재 이유가 "개발 PC에서 돈다"인데, 출력의 em dash 하나에
#  UnicodeEncodeError로 죽으면 검증 자체를 못 한다.)
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8")

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware.cartesian import CartesianArm, SimJointIO  # noqa: E402
from tomato_picker.hardware.kinematics import ArmGeometry  # noqa: E402
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
    arm.jog(dpitch=-30)                      # 집게를 30° 더 숙인다
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
    far = fresh_arm()                                    # x=245mm에서 시작
    expect_error("사거리 밖 (x=320)", lambda: far.move_to(x=320), "사거리")

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
    check("집게가 회전축 바로 위, 높이 = z0+l1+l2+l3",
          abs(pose.x) < 1e-9 and abs(pose.y) < 1e-9 and abs(pose.z - up) < 1e-9,
          f"x={pose.x:.1f} y={pose.y:.1f} z={pose.z:.1f} (기대 {up:.0f})")
    check("집게가 하늘을 본다 (pitch +90°)", abs(pose.pitch - 90) < 1e-9,
          f"pitch={pose.pitch:.1f}°")
    expect_error("세운 채로 조그하면 막힌다(특이점)", lambda: arm.jog(dx=10),
                 "수직으로 서 있습니다")

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
