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
