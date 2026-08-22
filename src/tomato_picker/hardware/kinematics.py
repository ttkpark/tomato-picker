"""SO-101(5축 + 집게) 팔의 순/역기구학 — **하드웨어도 lerobot도 없이 도는 순수 계산**.

왜 따로 떼어놓나 — 좌표 이동은 "숫자가 맞는가"가 전부인데, 그걸 확인하려고
매번 젯슨에 올려 팔을 흔들 수는 없다. 여기에는 import가 math 하나뿐이라
개발 PC에서 `tools/arm_cartesian_check.py`로 왕복(FK→IK→FK)을 바로 검증한다.

────────────────────────────────────────────────────────────────────────
좌표계 (base frame)
    원점 = shoulder_pan(요) 축이 마운트 평면을 지나는 점
    +x = pan=0일 때 팔이 뻗는 방향(앞)   +y = 왼쪽   +z = 위
    pitch = 집게 축이 수평면에서 들린 각 (+ = 위를 봄, -90° = 바닥을 봄)
    roll  = 집게 축을 중심으로 한 회전 (= wrist_roll 그대로)

관절 각도 규약 (전부 도(°), **전부 0이면 팔이 앞으로 수평하게 쭉 뻗은 자세**)
    shoulder_pan   + = 위에서 봤을 때 반시계(왼쪽)
    shoulder_lift  + = 상완이 위로
    elbow_flex     + = 전완이 상완 대비 위로   (절대각 = lift + elbow)
    wrist_flex     + = 집게가 전완 대비 위로   (절대각 = lift + elbow + wrist = pitch)
    wrist_roll     + = 집게 축 기준 반시계

각도의 기준(0° = 수평)과 **사람이 영점을 잡을 때 두는 자세**는 별개다 —
후자는 config.ARM_CART_ZERO_POSE_DEG이고, 지금은 **곧게 위로 세운 자세**
(lift=90, elbow=0, wrist=0)를 쓴다. 수평은 힘을 빼는 순간 중력이 끌어내려
그 처짐이 영점 오차가 되지만, 수직은 벽 모서리에 대보면 눈으로 맞는다.
(모터 엔코더 중앙값을 영점으로 삼으면 그게 기구학적으로 어디인지 아무도 모른다.)

⚠ 세워 놓은 그 자세는 수평거리 r≈0이라 **좌표계의 특이점**이다(그 점에서는
  pan이 무슨 값이든 같은 위치다). 영점을 잡은 직후 바로 조그가 안 되는 건
  버그가 아니라 이것 때문이다 — 프리셋으로 앞으로 뻗은 뒤 쓰면 된다.

────────────────────────────────────────────────────────────────────────
이 팔이 **할 수 없는 것**을 먼저 적어둔다 — 5축이라 6자유도가 안 나온다.
  · 집게의 **요(yaw)는 위치에 묶여 있다.** pan은 팔 전체를 돌리므로 집게는
    항상 원점에서 바깥을 향한다. 즉 "제자리에서 물체를 좌우로 비틀기"는
    일반적으로 불가능하다.
  · 단 **집게가 바닥을 볼 때(pitch≈-90°)는 wrist_roll이 곧 yaw다.** 위에서
    집는 자세에서는 제자리 yaw 회전이 공짜로 된다. (이 예외를 적는 이유:
    실제로 토마토를 위에서 집을 때가 회전이 필요한 거의 유일한 순간이다.)
  · 그래서 제자리 회전은 두 축만 제공한다: roll(집게 축) + pitch(손목 들기).
    pitch 회전은 IK로 **끝점 좌표를 고정한 채** 각만 바꿔 만든다.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

# lerobot 모터 이름 그대로 — 관찰값 키는 f"{이름}.pos"다.
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
ARM_JOINTS = JOINTS[:4]  # 위치·pitch에 관여하는 축(roll은 끝점을 안 옮긴다)


class Unreachable(ValueError):
    """IK가 풀리지 않는 목표 — 사거리 밖이거나, 그 pitch로는 못 닿는다."""


@dataclass(frozen=True)
class ArmGeometry:
    """링크 길이(mm). ⚠ **자로 재서 고쳐라** — 기본값은 SO-101 도면 근사치다.

    재는 법(팔 힘 빼고 앞으로 쭉 뻗은 상태에서 축과 축 사이):
      z0 = 마운트 평면 → shoulder_lift 축 높이
      d0 = pan 축 → shoulder_lift 축 수평 오프셋(대개 거의 0)
      l1 = shoulder_lift 축 → elbow_flex 축
      l2 = elbow_flex 축 → wrist_flex 축
      l3 = wrist_flex 축 → **집게가 물건을 무는 지점**(TCP). 손끝이 아니라 무는 곳.

    길이가 틀리면 어떻게 되나 — FK와 IK가 **같은 틀린 값**을 쓰므로 조그(상대
    이동)는 방향은 맞고 크기만 비례해서 어긋난다. 절대 좌표 이동은 그만큼 틀린다.
    """

    z0: float = 55.0
    d0: float = 0.0
    l1: float = 116.0
    l2: float = 135.0
    l3: float = 95.0

    @property
    def reach_max(self) -> float:
        """pan 축에서 잰 최대 수평 사거리(mm) — 완전히 펼쳤을 때."""
        return self.d0 + self.l1 + self.l2 + self.l3

    @property
    def wrist_reach(self) -> float:
        return self.l1 + self.l2

    @property
    def wrist_reach_min(self) -> float:
        return abs(self.l1 - self.l2)


@dataclass(frozen=True)
class ToolPose:
    """집게(TCP)의 자세. 위치는 mm, 각은 도."""

    x: float
    y: float
    z: float
    pitch: float
    roll: float = 0.0

    @property
    def r(self) -> float:
        """pan 축에서 잰 수평 거리."""
        return math.hypot(self.x, self.y)

    def replace(self, **kw) -> "ToolPose":
        return ToolPose(
            x=kw.get("x", self.x), y=kw.get("y", self.y), z=kw.get("z", self.z),
            pitch=kw.get("pitch", self.pitch), roll=kw.get("roll", self.roll),
        )

    def as_dict(self) -> dict[str, float]:
        return {"x": round(self.x, 2), "y": round(self.y, 2), "z": round(self.z, 2),
                "pitch": round(self.pitch, 2), "roll": round(self.roll, 2)}


# ----------------------------------------------------------------------
# 순기구학
# ----------------------------------------------------------------------

def forward(joints: Mapping[str, float], geom: ArmGeometry = ArmGeometry()) -> ToolPose:
    """관절 각(도) → 집게 자세. 없는 관절은 0으로 본다."""
    pan = math.radians(joints.get("shoulder_pan", 0.0))
    lift = math.radians(joints.get("shoulder_lift", 0.0))
    elbow = math.radians(joints.get("elbow_flex", 0.0))
    wrist = math.radians(joints.get("wrist_flex", 0.0))

    a1, a2, a3 = lift, lift + elbow, lift + elbow + wrist
    r = geom.d0 + geom.l1 * math.cos(a1) + geom.l2 * math.cos(a2) + geom.l3 * math.cos(a3)
    z = geom.z0 + geom.l1 * math.sin(a1) + geom.l2 * math.sin(a2) + geom.l3 * math.sin(a3)
    return ToolPose(
        x=r * math.cos(pan), y=r * math.sin(pan), z=z,
        pitch=math.degrees(a3), roll=float(joints.get("wrist_roll", 0.0)),
    )


def signed_radius(joints: Mapping[str, float], geom: ArmGeometry = ArmGeometry()) -> float:
    """어깨 평면에서의 **부호 있는** 수평거리. FK의 x,y는 이 부호를 잃는다.

    왜 따로 필요한가 — 팔이 몸통 위로 접혀 pan 축 **뒤로** 넘어가면 r이 음수가
    되는데, (x,y)=(r·cosθ, r·sinθ)는 방위각이 180° 뒤집힌 같은 점으로 보인다.
    그 좌표를 IK에 도로 넣으면 전혀 다른 자세가 나온다(왕복이 깨진다).
    좌표 조그를 시작하기 전에 이 값이 충분히 **양수**인지 먼저 확인해야 한다.
    """
    lift = math.radians(joints.get("shoulder_lift", 0.0))
    elbow = math.radians(joints.get("elbow_flex", 0.0))
    wrist = math.radians(joints.get("wrist_flex", 0.0))
    a1, a2, a3 = lift, lift + elbow, lift + elbow + wrist
    return (geom.d0 + geom.l1 * math.cos(a1) + geom.l2 * math.cos(a2)
            + geom.l3 * math.cos(a3))


# ----------------------------------------------------------------------
# 역기구학
# ----------------------------------------------------------------------

def inverse(
    pose: ToolPose,
    geom: ArmGeometry = ArmGeometry(),
    elbow_up: bool = True,
    seed_pan: float = 0.0,
) -> dict[str, float]:
    """집게 자세 → 관절 각(도). 못 닿으면 Unreachable(**되는 데까지 가지 않는다**).

    elbow_up=True면 팔꿈치를 어깨-손목 현(弦) **위로** 접는다. 책상 위 팔은
    이쪽이 맞다 — 아래로 접으면 팔꿈치가 바닥/무대를 친다.

    seed_pan은 목표가 pan 축 바로 위(r≈0)라 방위각이 정의되지 않을 때 쓸 값.
    "지금 pan을 그대로 두라"는 뜻이라, 조그 중에 팔이 홱 도는 사고를 막는다.

    ⚠ 못 닿을 때 가까운 해로 **몰래 대체하지 않는다.** 이 로봇에서 가장 흔한
    사고는 "지령은 나갔는데 딴 데로 갔다"이고, 그 절반은 이런 조용한 폴백이다.
    """
    r = math.hypot(pose.x, pose.y)
    pan = seed_pan if r < 1e-6 else math.degrees(math.atan2(pose.y, pose.x))

    pitch = math.radians(pose.pitch)
    # 손목(wrist_flex 축) 중심을 어깨 기준 평면좌표로.
    rw = r - geom.d0 - geom.l3 * math.cos(pitch)
    zw = pose.z - geom.z0 - geom.l3 * math.sin(pitch)
    c = math.hypot(rw, zw)

    if c > geom.wrist_reach + 1e-6:
        raise Unreachable(
            f"사거리 밖 — 어깨에서 손목까지 {c:.0f}mm가 필요한데 최대 "
            f"{geom.wrist_reach:.0f}mm입니다 (목표 x={pose.x:.0f} y={pose.y:.0f} "
            f"z={pose.z:.0f} pitch={pose.pitch:.0f}°). 더 가깝게 잡거나 pitch를 바꾸세요."
        )
    if c < geom.wrist_reach_min - 1e-6:
        raise Unreachable(
            f"너무 가깝다 — 어깨-손목 {c:.0f}mm는 최소 {geom.wrist_reach_min:.0f}mm보다 "
            "짧습니다(팔이 스스로를 통과해야 함)."
        )

    cos_elbow = (c * c - geom.l1 ** 2 - geom.l2 ** 2) / (2 * geom.l1 * geom.l2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    q3 = math.acos(cos_elbow)
    # q3>0이면 전완이 상완 대비 반시계로 접힌다 = 팔꿈치가 현 **아래**.
    # 그래서 "팔꿈치 위"는 음수 해다.
    if elbow_up:
        q3 = -q3

    a1 = math.atan2(zw, rw) - math.atan2(geom.l2 * math.sin(q3),
                                         geom.l1 + geom.l2 * math.cos(q3))
    q4 = pitch - a1 - q3

    return {
        "shoulder_pan": wrap180(pan),
        "shoulder_lift": math.degrees(a1),
        "elbow_flex": math.degrees(q3),
        "wrist_flex": math.degrees(q4),
        "wrist_roll": float(pose.roll),
    }


def wrap180(deg: float) -> float:
    """-180..180으로 접는다 — 359°와 -1°가 다른 자세로 취급되지 않게."""
    return (deg + 180.0) % 360.0 - 180.0


# ----------------------------------------------------------------------
# 도구 좌표계 (집게가 보는 방향 기준 이동)
# ----------------------------------------------------------------------

def tool_axes(pose: ToolPose) -> tuple[tuple[float, float, float], ...]:
    """(approach, lateral, up) 단위벡터 — base 좌표로 표현한 집게 좌표축.

      approach = 집게가 향하는 방향(앞으로 찔러 넣는 축)
      lateral  = 그 축의 왼쪽(수평)
      up       = 두 축에 수직(집게 기준 위)

    pitch=0, pan=0이면 각각 +x, +y, +z가 되어 base 좌표와 일치한다.
    """
    th = math.atan2(pose.y, pose.x)
    ph = math.radians(pose.pitch)
    approach = (math.cos(ph) * math.cos(th), math.cos(ph) * math.sin(th), math.sin(ph))
    lateral = (-math.sin(th), math.cos(th), 0.0)
    up = (-math.sin(ph) * math.cos(th), -math.sin(ph) * math.sin(th), math.cos(ph))
    return approach, lateral, up


def offset_in_tool_frame(pose: ToolPose, da: float, dl: float, du: float
                         ) -> tuple[float, float, float]:
    """도구 좌표 이동량 → base 좌표 이동량(mm)."""
    approach, lateral, up = tool_axes(pose)
    return (
        da * approach[0] + dl * lateral[0] + du * up[0],
        da * approach[1] + dl * lateral[1] + du * up[1],
        da * approach[2] + dl * lateral[2] + du * up[2],
    )
