"""특이점 탈출 — **관절공간으로** 가드 밖까지 뻗는 경로를 짠다(팔을 열지 않는다).

좌표 유닛(`cartesian.py`)은 지금 자세의 `signed_radius`가 `ARM_CART_R_MIN`보다
작으면 어떤 좌표 이동도 거절한다. 옳은 가드다 — 집게가 pan축 위에 올라앉으면
"앞으로 5mm"가 어느 방향인지 정의되지 않는다. 하지만 그건 **팔이 못 움직인다는
뜻이 아니다**: 관절을 조금 펴면 빠져나온다.

그 "펴는 규칙"이 원래 `ros2/tools/arm_extend.py` 안에만 있었다. 2026-09-18(T49)에
여기로 옮겼다 — 규칙을 쓰는 곳이 셋으로 늘었기 때문이다:

  · `ros2/tools/arm_extend.py`      사람이 젯슨에서 손으로 뻗을 때 (lerobot 버스)
  · `tomato_bridge/arm_source.py`   ROS가 팔의 주인일 때 한 걸음씩 (arm/joint_command)
  · `ros2/tools/move5_check.py`     졸업기준3을 재기 **전에** 스스로 뻗을 때

세 곳이 각자 12°·바닥·한계를 베끼면 말없이 갈라진다. 갈라지면 한쪽은 통과시킨
걸음을 다른 쪽이 거절하고, 그 거절은 "팔이 못 간다"로 기록에 남는다.

⚠ 이 모듈은 **포트를 열지 않는다.** 재는 것과 보내는 것은 부르는 쪽이 준다
(`settle.py`와 같은 규칙 — 그래서 PC에서 검증된다).
"""

from __future__ import annotations

import math

from ..config import (ARM_CART_MAX_STEP_JOINT_DEG, ARM_CART_NORM_MARGIN,
                      ARM_CART_R_MIN)
from . import kinematics as kin

# 한 구간에서 어느 관절도 이 이상 안 움직인다. 좌표 유닛의 관절걸음 상한과
# **같은 값을 쓴다** — 같은 팔을 두 규칙으로 밀면 한쪽만 통과하는 걸음이 생긴다.
STEP_DEG = ARM_CART_MAX_STEP_JOINT_DEG
LIMIT_NORM = 100.0 - ARM_CART_NORM_MARGIN

# 바닥은 팔 base(마운트)보다 이만큼 아래에 있다 — 실측 76.5mm
# (`ros2/src/tomato_description/config/so101_geometry.yaml` 의 mount.z 와 같은 값,
#  ros_selfcheck [탈출]이 그 일치를 강제한다).
# ⚠ 예전에는 "지금 자리보다 1mm 아래"를 바닥으로 삼았다. 그러면 팔이 낮게
#   늘어져 있을 때 **1.8mm 내려갔다 다시 오르는 정상 경로까지 막혀** 빠져나올
#   수가 없다(2026-09-01, 복구 불가 상태로 두 번 갇혔다).
# ⚠ 좌표 유닛의 바닥(`ARM_CART_Z_MIN`=15mm)을 여기서 쓰면 안 된다 — 토크가
#   빠져 주저앉은 팔은 z가 −66mm다(2026-09-18 실측: 컨테이너가 뜨자 +416→−66mm).
#   15mm를 바닥으로 보면 **그 자세에서는 첫 걸음부터 전부 막혀** 탈출이 불가능하다.
MOUNT_Z_MM = 76.5
FLOOR_MARGIN_MM = 10.0

# 목표 자세 — 집게가 **수평 앞**을 보는 표준 자세.
#   lift 80° = 상완이 거의 수직, elbow -80° = 전완이 수평, wrist 0° = 손목 곧게
#   → a1=80, a2=0, a3=0 이므로 pitch=0(수평), r≈250mm, z≈169mm
# ⚠ `wrist_roll`과 `shoulder_pan`은 **일부러 안 건드린다** — 손목에 카메라와
#   USB3 케이블이 달려 있어 큰 회전이 케이블을 감는다. 코드는 케이블을 못 본다.
TARGET_DEG = {"shoulder_lift": 80.0, "elbow_flex": -80.0, "wrist_flex": 0.0}

GUARD_MM = ARM_CART_R_MIN
# 한 걸음에 주는 보간 시간(초). 걸음이 12° 이하이므로 1.2초면 서보가 따라온다 —
# 더 짧게 주면 지령과 실제가 벌어진 채로 다음 걸음이 시작된다.
SECS_PER_STEP = 1.2


def floor_z() -> float:
    """탈출 경로가 지켜야 하는 z 하한(mm, 마운트 평면 기준)."""
    return -MOUNT_Z_MM + FLOOR_MARGIN_MM


def needs_escape(degs: dict[str, float], geom: kin.ArmGeometry | None = None,
                 guard_mm: float = GUARD_MM) -> bool:
    """이 자세는 좌표 이동을 거절당하는가(`cartesian._require_state`와 같은 판정)."""
    return kin.signed_radius(degs, geom or kin.ArmGeometry()) < guard_mm


def clamp_norm(target_norm: dict[str, float], limit: float = LIMIT_NORM):
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


def worsening_limits(now_norm: dict[str, float], step_norm: dict[str, float],
                     limit: float = LIMIT_NORM) -> list[str]:
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


def plan(now_deg: dict[str, float], target_deg: dict[str, float],
         now_norm: dict[str, float], to_norm, geom: kin.ArmGeometry | None = None,
         step_deg: float = STEP_DEG, limit: float = LIMIT_NORM) -> list[dict]:
    """지금 관절각 → 목표 관절각을 걸음들로 쪼개고 **걸음마다 검사해 둔다**.

    돌려주는 걸음 하나: {"i", "of", "degs", "pose", "signed_r", "notes"}.
    `notes`가 빈 리스트가 아니면 그 걸음은 안전 검사에 걸린 것이다 — 부르는 쪽은
    **하나라도 걸리면 시작조차 하지 않는다**(절반 가서 멈추면 그 자리가 다음
    시도의 시작 자세가 된다, 2026-09-18에 실제로 그렇게 굴렀다).

    `to_norm`은 각도→정규화값 변환(보정표를 아는 쪽이 준다). 한계 판정이
    정규화 공간에서만 뜻이 있어서 함수로 받는다 — 이 모듈은 파일을 안 읽는다.
    """
    geom = geom or kin.ArmGeometry()
    biggest = max(abs(float(target_deg[j]) - float(now_deg[j])) for j in kin.JOINTS)
    steps = max(1, int(math.ceil(biggest / step_deg - 1e-9)))
    out = []
    for i in range(1, steps + 1):
        degs = {j: float(now_deg[j]) + (float(target_deg[j]) - float(now_deg[j])) * i / steps
                for j in kin.JOINTS}
        pose = kin.forward(degs, geom)
        notes = []
        if pose.z < floor_z():
            notes.append("바닥아래")
        if math.hypot(pose.x, pose.y) > geom.reach_max + 1e-6:
            notes.append("사거리밖")
        for j in worsening_limits(now_norm, to_norm(degs), limit):
            notes.append(f"{j}한계")
        out.append({"i": i, "of": steps, "degs": degs, "pose": pose,
                    "signed_r": kin.signed_radius(degs, geom), "notes": notes})
    return out


def blocked(steps: list[dict]) -> list[int]:
    """안전 검사에 걸린 걸음 번호들. 비어 있지 않으면 움직이지 않는다."""
    return [s["i"] for s in steps if s["notes"]]


# 한 걸음을 보낸 뒤 실제가 지령에 **못 미치는 것은 정상이다** — 중력이 누른 만큼
# 처진다(2026-09-18 실측: A자세 1.1~1.3°, 팔을 뻗은 B자세 4.0~4.9°). 그래서
# "다 왔다"의 기준을 0으로 두면 영원히 안 끝나고, 처짐을 "안 움직였다"로 읽으면
# 멀쩡한 뻗기를 중간에 포기한다. 두 숫자가 그 경계를 정한다.
ARRIVE_DEG = 1.0        # 남은 길이 이보다 작으면 도착으로 본다
MIN_GAIN_DEG = 0.5      # 한 걸음에 남은 길이 이만큼도 안 줄면 더 가 봐야 소용없다

# 보내는 걸음을 상한보다 이만큼 **작게** 짠다. 받는 쪽은 자기가 따로 읽은 자세로
# 크기를 다시 재는데(arm_source.move_joints_deg), 그 읽기가 한 박자 늦으면 꽉 찬
# 12°짜리 걸음이 12.x°로 읽혀 거절된다 — 거절당한 걸음은 "팔이 못 간다"로 기록에
# 남는다. 여유 1°는 걸음 수를 거의 안 늘리면서 그 경계를 비켜 준다.
SEND_MARGIN_DEG = 1.0


def walk(target_deg: dict[str, float], measure, send, to_norm,
         geom: kin.ArmGeometry | None = None, step_deg: float = STEP_DEG,
         limit: float = LIMIT_NORM, budget: int | None = None,
         arrive_deg: float = ARRIVE_DEG, min_gain_deg: float = MIN_GAIN_DEG,
         margin_deg: float = SEND_MARGIN_DEG) -> dict:
    """목표 관절각까지 **걸음마다 실제 자세를 되읽어 다시 짜며** 걸어간다.

    왜 이 함수가 필요한가 (T56, 2026-09-18) — `plan()`이 짜 준 걸음들을 그대로
    차례로 보내면 **두 걸음째부터 거절된다.** 한 걸음(12°)을 보내면 팔은 중력
    처짐으로 지령보다 4~5° 못 미친 자리에 서는데, 받는 쪽(`arm_source.
    move_joints_deg`)은 **지금 실제 자세에서 다시 plan** 해 두 걸음 이상이면
    거절하기 때문이다: 12 + 4.3 = 16.3° > 12°. 사이클42의 실기 기록에 정확히 그
    숫자가 남아 있고, 그 거절 때문에 5회가 전부 자세 가드에 막혀 **거짓 0/5**가
    됐다. 받는 쪽의 계약("쪼개는 쪽이 걸음마다 실제 자세를 되읽어야 한다")이
    옳다 — 그 되읽기를 여기서 한 번만 구현해 부르는 쪽들이 나눠 쓴다.

    ⚠ 이 함수도 **포트를 안 연다.** 재는 것(`measure() -> 관절각 dict`)과
    보내는 것(`send(관절각 dict)`)은 부르는 쪽이 준다. `send`가 거절하면
    예외를 올려라 — 여기서 삼키지 않는다(뻗은 척하는 것보다 멈추는 것이 낫다).

    돌려주는 것: {"reached", "sent", "gap_deg", "last", "planned", "notes", "detail"}.
    `reached`가 False여도 `last`까지는 **실제로 갔다** — 부르는 쪽은 그 자세로
    목적(가드 탈출)이 이뤄졌는지 스스로 판정한다.
    """
    geom = geom or kin.ArmGeometry()
    hop_deg = max(1.0, float(step_deg) - float(margin_deg))
    now = {j: float(v) for j, v in measure().items() if j in kin.JOINTS}
    planned = plan(now, {**now, **target_deg}, to_norm(now), to_norm, geom,
                   hop_deg, limit)
    # 예산은 계획 걸음수의 3배 — 처짐 때문에 같은 구간을 두어 번 더 밟는 것은
    # 정상이고, 그보다 많이 밟는다면 진전이 없는 것이다(아래 min_gain이 먼저
    # 잡지만, 되읽기가 흔들릴 때를 대비한 마지막 울타리다).
    if budget is None:
        budget = max(6, len(planned) * 3)
    sent, gap_prev = 0, None
    while True:
        target = {**now, **{j: float(v) for j, v in target_deg.items()
                            if j in kin.JOINTS}}
        gap = max(abs(target[j] - now[j]) for j in kin.JOINTS)
        if gap <= arrive_deg:
            return _walk_result(True, now, planned, sent, gap, [],
                                f"{sent}걸음으로 목표에 닿았다(남은 길 {gap:.1f}°)")
        if gap_prev is not None and gap_prev - gap < min_gain_deg:
            return _walk_result(False, now, planned, sent, gap, [],
                                f"{gap:.1f}°를 남기고 더 안 줄어든다"
                                f"(직전 {gap_prev:.1f}° → {gap:.1f}°) — "
                                "서보가 그 자세를 더 못 든다")
        if sent >= budget:
            return _walk_result(False, now, planned, sent, gap, [],
                                f"{budget}걸음을 다 쓰고도 {gap:.1f}° 남았다")
        steps = plan(now, target, to_norm(now), to_norm, geom, hop_deg, limit)
        head = steps[0]
        if head["notes"]:
            return _walk_result(False, now, planned, sent, gap, head["notes"],
                                f"{sent + 1}걸음째가 안전 검사에 걸린다: "
                                + " ".join(head["notes"])
                                + f" (TCP z={head['pose'].z:.0f}mm "
                                  f"r={head['signed_r']:.0f}mm)")
        send(head["degs"])
        sent += 1
        gap_prev = gap
        now = {j: float(v) for j, v in measure().items() if j in kin.JOINTS}


def _walk_result(reached, now, planned, sent, gap, notes, detail) -> dict:
    return {"reached": reached, "sent": sent, "gap_deg": round(gap, 2),
            "last": dict(now), "planned": len(planned), "notes": list(notes),
            "detail": detail}
