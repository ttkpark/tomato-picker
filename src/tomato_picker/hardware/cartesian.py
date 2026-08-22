"""**카테시안 유닛** — 집게가 문 물건을 xyz로 옮기고 제자리에서 돌린다.

지금까지 이 팔은 "저장한 자세를 재생"만 할 수 있었다(presets.py). 그래서
"5mm만 더 앞으로", "잡은 채로 90° 돌려"가 안 됐다 — 그런 자세를 미리 저장해
두지 않았으면 방법이 없었다. 이 유닛이 그 사이를 메운다.

    지금 관절값 ─FK→ 지금 집게 좌표 ─(요청한 만큼 더함)→ 목표 좌표 ─IK→ 목표 관절값

계산은 [`kinematics.py`](kinematics.py)가 하고, 여기서는 **로봇 쪽 현실**만 다룬다:
  ① lerobot 정규화값(-100..100) ↔ 실제 각도(°) 환산
  ② 기구학 영점 — "관절 전부 0 = 앞으로 수평하게 뻗음"을 실제 팔에 맞춰 잡기
  ③ 안전 — 사거리·관절한계·바닥·한 번에 움직일 최대량
  ④ **"움직이라고 했는데 안 움직이는" 크기를 거절하기** (아래 참고)

⚠ 이 로봇의 1번 병이 팔에도 있다 — CLAUDE.md는 바퀴의 정지마찰을 말하지만
   서보에도 같은 문턱이 있다. 0.1mm 조그는 관절 각으로 0.02°, 서보 분해능
   (STS3215 = 0.088°/tick) **아래**라 지령이 나가도 물리적으로 0이다. 그래서
   목표 관절 변화량이 ARM_CART_MIN_JOINT_DEG보다 작으면 **조용히 보내지 않고
   거절한다.** "보냈는데 안 움직인다"로 한 시간 태우는 대신 즉시 말해준다.

⚠ 집게(gripper)는 이 유닛이 **건드리지 않는다.** 물고 있는 물건을 옮기는 게
   목적인데 매 조그마다 집게에 새 목표를 주면, 눌린 현재값을 목표로 다시 써서
   무는 힘이 조금씩 풀린다(몇 번 반복하면 떨어뜨린다). 집게는 set_grip()으로
   **명시적으로 요청할 때만** 움직인다.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import threading
import time
from typing import Protocol

from ..config import (
    ARM_CART_ELBOW_UP,
    ARM_CART_FILE,
    ARM_CART_MAX_STEP_DEG,
    ARM_CART_MAX_STEP_MM,
    ARM_CART_MIN_JOINT_DEG,
    ARM_CART_MOVE_SECS,
    ARM_CART_NORM_MARGIN,
    ARM_CART_R_MIN,
    ARM_CART_SIGNS,
    ARM_CART_STEP_DEG,
    ARM_CART_STEP_MM,
    ARM_CART_Z_MIN,
    ARM_CART_ZERO_POSE_DEG,
    ARM_GEOM_D0,
    ARM_GEOM_L1,
    ARM_GEOM_L2,
    ARM_GEOM_L3,
    ARM_GEOM_Z0,
)
from . import kinematics as kin
from .kinematics import ArmGeometry, ToolPose, Unreachable

GRIPPER = "gripper"
# STS3215는 한 바퀴가 4096틱 — 정규화값 폭을 실제 각도로 바꿀 때 쓴다.
DEG_PER_TICK = 360.0 / 4096.0
# 캘리브레이션을 못 읽을 때 쓰는 대체값: 관절 가동폭을 180°로 가정
# (정규화 -100..100 = 180° → 0.9°/단위). wrist_roll은 한 바퀴라 1.8이 맞지만,
# 실제 팔에서는 아래 spans()가 진짜 값을 준다 — 여긴 순수 시뮬용 기본값이다.
FALLBACK_DEG_PER_NORM = {"wrist_roll": 1.8}
FALLBACK_DEG_PER_NORM_DEFAULT = 0.9


class JointIO(Protocol):
    """카테시안 유닛이 팔에 요구하는 최소한 — 이것만 있으면 Mock에도 붙는다."""

    def read(self) -> dict[str, float]:
        """지금 관절 정규화값 {"shoulder_pan": -12.3, ...} (`.pos` 접미사 없이)."""

    def write(self, target: dict[str, float], secs: float) -> None:
        """목표 정규화값으로 secs초에 걸쳐 보간 이동(블로킹). 준 관절만 움직인다."""

    def spans_deg(self) -> dict[str, float]:
        """관절별 **정규화 -100..100이 실제 몇 도인지**. 모르면 빈 dict."""

    def busy_lock(self):
        """이동과 읽기를 직렬화할 락(`with`로 쓸 수 있고, `acquire(blocking=False)`를
        지원하면 상태 조회가 이동을 기다리지 않는다)."""

    def before_move(self) -> None:
        """이동 직전 훅 — 실물 팔은 여기서 미러링을 끈다(목표가 둘이면 싸운다)."""


# ----------------------------------------------------------------------
# 영점·부호·링크길이 — 팔마다 다른 값, 파일에 남는다
# ----------------------------------------------------------------------

class FrameConfig:
    """~/arm_cartesian.json — 정규화값을 기구학 각도로 바꾸는 데 필요한 전부.

    ┌ zero[j]         [영점 등록]을 누른 순간의 정규화값
    ├ ref_deg[j]      그때 팔이 취하고 있던 **기구학 각도**(교시 자세)
    ├ signs[j]        정규화값이 커질 때 각도가 커지면 +1, 반대면 -1
    ├ deg_per_norm[j] 정규화 1단위 = 몇 도 (비우면 팔의 캘리브레이션에서 읽는다)
    └ geometry        링크 길이(mm) — config.py 기본값을 현장에서 덮어쓸 때

    zero와 ref_deg를 **함께** 적는 이유 — 교시 자세를 나중에 바꾸면(수평→수직처럼)
    이미 잡아둔 영점의 뜻이 통째로 달라진다. 그때 무슨 자세로 잡았는지가 파일에
    없으면 기존 팔이 조용히 90° 틀어진다. 그래서 파일이 스스로를 설명하게 둔다.

    부호를 왜 파일로 빼나 — 서보를 어느 방향으로 조립했는지는 팔마다 다르고,
    코드를 읽어서는 알 수 없다. **힘 빼고 손으로 움직이며 화면 숫자를 보는 것**이
    유일하게 확실한 확인법이고(docs/arm-cartesian.md), 그 결과를 여기 남긴다.
    """

    def __init__(self, path: str = ARM_CART_FILE) -> None:
        self._path = os.path.expanduser(path)
        self._lock = threading.RLock()
        self._data: dict = {}
        self.reload()

    @property
    def path(self) -> str:
        return self._path

    def reload(self) -> None:
        with self._lock:
            try:
                with open(self._path, encoding="utf-8") as f:
                    raw = json.load(f)
            except (OSError, ValueError):
                raw = {}
            if not isinstance(raw, dict):
                raw = {}
            raw.setdefault("zero", {})
            raw.setdefault("ref_deg", {})
            raw.setdefault("signs", {})
            raw.setdefault("deg_per_norm", {})
            raw.setdefault("geometry", {})
            self._data = raw

    def _save(self) -> None:
        directory = os.path.dirname(self._path) or "."
        os.makedirs(directory, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=directory, prefix=".cart-", suffix=".json")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(self._data, f, ensure_ascii=False, indent=2)
            os.replace(tmp, self._path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    # --- 읽기 ---

    @property
    def has_zero(self) -> bool:
        with self._lock:
            return bool(self._data["zero"])

    def zero(self) -> dict[str, float]:
        with self._lock:
            return {j: float(self._data["zero"].get(j, 0.0)) for j in kin.JOINTS}

    def ref_deg(self) -> dict[str, float]:
        """영점을 잡은 자세의 기구학 각도. 없으면 지금 config의 교시 자세."""
        with self._lock:
            saved = dict(self._data.get("ref_deg") or {})
        return {j: float(saved.get(j, ARM_CART_ZERO_POSE_DEG.get(j, 0.0)))
                for j in kin.JOINTS}

    def sign(self, joint: str) -> float:
        with self._lock:
            raw = self._data["signs"].get(joint)
        if raw is None:
            raw = ARM_CART_SIGNS.get(joint, 1)
        return -1.0 if float(raw) < 0 else 1.0

    def deg_per_norm_override(self, joint: str) -> float | None:
        with self._lock:
            raw = self._data["deg_per_norm"].get(joint)
        return None if raw in (None, "") else float(raw)

    def geometry(self) -> ArmGeometry:
        with self._lock:
            g = dict(self._data["geometry"])
        return ArmGeometry(
            z0=float(g.get("z0", ARM_GEOM_Z0)),
            d0=float(g.get("d0", ARM_GEOM_D0)),
            l1=float(g.get("l1", ARM_GEOM_L1)),
            l2=float(g.get("l2", ARM_GEOM_L2)),
            l3=float(g.get("l3", ARM_GEOM_L3)),
        )

    # --- 쓰기 ---

    def set_zero(self, norms: dict[str, float],
                 ref_deg: dict[str, float] | None = None) -> None:
        ref = ref_deg if ref_deg is not None else ARM_CART_ZERO_POSE_DEG
        with self._lock:
            self._data["zero"] = {j: float(norms[j]) for j in kin.JOINTS if j in norms}
            self._data["ref_deg"] = {j: float(ref.get(j, 0.0)) for j in kin.JOINTS}
            self._save()

    def clear_zero(self) -> None:
        with self._lock:
            self._data["zero"] = {}
            self._data["ref_deg"] = {}
            self._save()

    def set_signs(self, signs: dict[str, float]) -> None:
        with self._lock:
            for j, v in signs.items():
                if j in kin.JOINTS:
                    self._data["signs"][j] = -1 if float(v) < 0 else 1
            self._save()

    def set_geometry(self, values: dict[str, float]) -> None:
        with self._lock:
            for k, v in values.items():
                if k in ("z0", "d0", "l1", "l2", "l3") and v is not None:
                    self._data["geometry"][k] = float(v)
            self._save()

    def snapshot(self) -> dict:
        geom = self.geometry()
        return {
            "path": self._path,
            "has_zero": self.has_zero,
            "zero": self.zero() if self.has_zero else {},
            "ref_deg": self.ref_deg(),
            "signs": {j: self.sign(j) for j in kin.JOINTS},
            "geometry": {"z0": geom.z0, "d0": geom.d0,
                         "l1": geom.l1, "l2": geom.l2, "l3": geom.l3},
            "reach_max": round(geom.reach_max, 1),
        }


# ----------------------------------------------------------------------
# 유닛 본체
# ----------------------------------------------------------------------

class CartesianArm:
    """xyz 이동 + 제자리 회전. 관절 하나하나가 아니라 **집게 좌표**로 말한다."""

    # 관절 읽기가 실패한 뒤 다시 물어보기까지 쉬는 시간(초).
    READ_BACKOFF_SEC = 5.0

    def __init__(self, io: JointIO, path: str = ARM_CART_FILE) -> None:
        self._io = io
        self.config = FrameConfig(path)
        self._last_pose: ToolPose | None = None
        self._last_note = ""
        # 읽기가 실패하면 잠깐 쉰다 — 아래 snapshot() 참고.
        self._read_quiet_until = 0.0
        self._last_error: str | None = None

    # --- 정규화값 ↔ 각도 ---

    def _deg_per_norm(self, joint: str) -> float:
        override = self.config.deg_per_norm_override(joint)
        if override:
            return override
        span = (self._io.spans_deg() or {}).get(joint)
        if span:
            return abs(float(span)) / 200.0
        return FALLBACK_DEG_PER_NORM.get(joint, FALLBACK_DEG_PER_NORM_DEFAULT)

    def to_degrees(self, norms: dict[str, float]) -> dict[str, float]:
        """정규화값 → 기구학 각도. 디버깅·툴링에서 쓰라고 공개해 둔다."""
        return self._to_deg(norms)

    def to_norms(self, degs: dict[str, float]) -> dict[str, float]:
        """기구학 각도 → 정규화값(위의 역)."""
        return self._to_norm(degs)

    # 정규화값 ─→ 각도:  ref + 부호 × (지금 − 영점) × (도/단위)
    # ref가 0이 아닌 이유는 교시 자세가 "곧게 세운" 자세이기 때문이다
    # (그 자세에서 lift는 0°가 아니라 90°다). config.ARM_CART_ZERO_POSE_DEG 참고.

    def _to_deg(self, norms: dict[str, float]) -> dict[str, float]:
        zero, ref = self.config.zero(), self.config.ref_deg()
        return {
            j: ref[j] + self.config.sign(j)
               * (float(norms.get(j, 0.0)) - zero[j]) * self._deg_per_norm(j)
            for j in kin.JOINTS
        }

    def _to_norm(self, degs: dict[str, float]) -> dict[str, float]:
        zero, ref = self.config.zero(), self.config.ref_deg()
        out = {}
        for j, deg in degs.items():
            if j not in kin.JOINTS:
                continue
            out[j] = zero[j] + (float(deg) - ref[j]) / (self.config.sign(j)
                                                        * self._deg_per_norm(j))
        return out

    # --- 상태 ---

    def joints_deg(self) -> dict[str, float]:
        """지금 관절 각(기구학 규약, 도)."""
        return self._to_deg(self._io.read())

    def pose(self) -> ToolPose:
        """지금 집게 좌표. 영점이 없으면 그대로 계산하되 값은 믿을 수 없다."""
        degs = self.joints_deg()
        pose = kin.forward(degs, self.config.geometry())
        self._last_pose = pose
        return pose

    def snapshot(self, live: bool = True) -> dict:
        """대시보드용 한 덩이. live=True라도 **버스가 바쁘면 기다리지 않는다**.

        1초 폴링이 이동(1.5초 블로킹)과 겹치면 화면 전체가 멈춘다 — 그래서 락을
        논블로킹으로 잡아보고, 실패하면 마지막으로 계산한 좌표를 stale로 준다.
        """
        base = {
            "ready": self.config.has_zero,
            "config": self.config.snapshot(),
            "limits": {"z_min": ARM_CART_Z_MIN, "r_min": ARM_CART_R_MIN,
                       "max_step_mm": ARM_CART_MAX_STEP_MM,
                       "max_step_deg": ARM_CART_MAX_STEP_DEG},
            "step_mm": ARM_CART_STEP_MM, "step_deg": ARM_CART_STEP_DEG,
            "note": self._last_note,
        }
        # 팔이 빠졌을 때 1초 폴링마다 재연결을 시도하면(=_with_retry) 포트 탐색과
        # 버스 확인에 매번 수백 ms가 들어가 화면 전체가 끈적해진다. 한 번 실패하면
        # READ_BACKOFF_SEC 동안은 묻지 않는다 — 어차피 그 사이 고쳐지지 않는다.
        quiet = time.monotonic() < self._read_quiet_until
        lock = self._io.busy_lock()
        acquired = False if (quiet or not live) else _try_acquire(lock)
        if not acquired:
            pose = self._last_pose
            return {**base, "stale": True,
                    "pose": pose.as_dict() if pose else None,
                    "joints": None,
                    "error": self._last_error if quiet else
                             (None if pose else "아직 읽은 적 없음")}
        try:
            degs = self.joints_deg()
            pose = kin.forward(degs, self.config.geometry())
            self._last_pose = pose
            r_signed = kin.signed_radius(degs, self.config.geometry())
            self._last_error = None
            return {
                **base, "stale": False, "error": None,
                "pose": pose.as_dict(),
                "joints": {j: round(v, 1) for j, v in degs.items()},
                "r": round(r_signed, 1),
                "folded": r_signed < ARM_CART_R_MIN,
            }
        except Exception as exc:  # noqa: BLE001 - 상태 조회가 화면을 죽이면 안 됨
            self._last_error = str(exc)
            self._read_quiet_until = time.monotonic() + self.READ_BACKOFF_SEC
            return {**base, "stale": True, "pose": None, "joints": None, "error": str(exc)}
        finally:
            _release(lock)

    # --- 영점·설정 ---

    ZERO_POSE_LABEL = "어깨는 정면, 나머지는 곧게 위로"

    def set_zero(self) -> str:
        """**지금 자세**를 교시 자세(config.ARM_CART_ZERO_POSE_DEG)로 등록.

        절차: 힘 빼기 → 어깨(pan)를 정면으로 두고 **팔을 곧게 위로 세운다**
        (상완·전완·집게가 한 줄로 수직 — 벽 모서리에 대보면 눈으로 맞는다) → 이 버튼.
        이 한 번이 끝나야 xyz 숫자가 실제 mm와 맞는다.
        """
        norms = self._io.read()
        self.config.set_zero(norms, ARM_CART_ZERO_POSE_DEG)
        geom = self.config.geometry()
        pose = kin.forward(ARM_CART_ZERO_POSE_DEG, geom)
        self._last_pose = pose
        return (f"기구학 영점 등록 — 지금 자세를 '{self.ZERO_POSE_LABEL}'로 봅니다 "
                f"(집게가 바닥에서 {pose.z:.0f}mm 위, 사거리 {geom.reach_max:.0f}mm). "
                "⚠ 세운 자세는 좌표계 한가운데라 조그가 막힙니다 — "
                "프리셋으로 앞으로 뻗은 뒤 쓰세요.")

    def clear_zero(self) -> str:
        self.config.clear_zero()
        return "기구학 영점 해제 — 좌표 이동을 쓰려면 다시 등록하세요."

    # --- 이동 ---

    def move_to(self, x: float | None = None, y: float | None = None,
                z: float | None = None, pitch: float | None = None,
                roll: float | None = None, secs: float | None = None) -> str:
        """절대 좌표로 이동. None인 축은 지금 값을 유지한다."""
        norms, now = self._require_state()
        target = ToolPose(
            x=now.x if x is None else float(x),
            y=now.y if y is None else float(y),
            z=now.z if z is None else float(z),
            pitch=now.pitch if pitch is None else float(pitch),
            roll=now.roll if roll is None else float(roll),
        )
        return self._go(norms, now, target, secs)

    def jog(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
            dpitch: float = 0.0, droll: float = 0.0, frame: str = "base",
            secs: float | None = None) -> str:
        """상대 이동. frame="base"면 로봇 기준 xyz, "tool"이면 집게 기준.

        tool 프레임: dx=집게가 보는 쪽으로 전진, dy=집게 기준 왼쪽, dz=집게 기준 위.
        토마토에 **똑바로 다가갈 때**는 tool 쪽이 훨씬 쓰기 쉽다(비스듬히 든
        집게를 base xyz로 밀면 세 축을 동시에 계산해야 한다).
        """
        norms, now = self._require_state()
        if frame == "tool":
            dx, dy, dz = kin.offset_in_tool_frame(now, dx, dy, dz)
        elif frame != "base":
            raise ValueError(f"frame은 base 또는 tool만 됩니다 (받은 값: {frame})")
        target = ToolPose(now.x + dx, now.y + dy, now.z + dz,
                          now.pitch + dpitch, now.roll + droll)
        return self._go(norms, now, target, secs)

    def spin(self, degrees: float, axis: str = "roll", secs: float | None = None) -> str:
        """**문 물건을 제자리에서 회전.** 끝점 좌표는 그대로 두고 각만 바꾼다.

          roll  집게 축을 중심으로 비틀기 — wrist_roll 하나만 돈다(가장 확실).
          pitch 집게를 들었다 내리기 — IK로 끝점을 고정한 채 손목 각만 바꾼다.
          yaw   좌우로 비틀기 — **집게가 바닥을 볼 때(pitch≈-90°)만 된다.**
                그때는 집게 축이 곧 수직축이라 roll이 곧 yaw이기 때문.
                아니면 5축 팔에서 yaw는 위치에 묶여 있어 불가능하다(kinematics.py).
        """
        norms, now = self._require_state()
        axis = (axis or "roll").lower()
        if axis == "roll":
            return self._go(norms, now, now.replace(roll=now.roll + degrees), secs)
        if axis == "pitch":
            return self._go(norms, now, now.replace(pitch=now.pitch + degrees), secs)
        if axis == "yaw":
            if abs(now.pitch + 90.0) > 15.0:
                raise ValueError(
                    f"지금 집게 pitch가 {now.pitch:.0f}°라 제자리 yaw 회전은 안 됩니다. "
                    "5축 팔은 집게 방향이 위치에 묶여 있어, 집게가 **바닥을 볼 때**"
                    "(pitch≈-90°)만 wrist_roll이 yaw가 됩니다. 먼저 pitch를 -90으로 "
                    "내리거나, 물체를 옆에서 잡았다면 roll로 돌리세요."
                )
            # 집게 축이 아래(-z)를 보므로 축 기준 +회전은 위에서 볼 때 시계방향이다.
            return self._go(norms, now, now.replace(roll=now.roll - degrees), secs)
        raise ValueError(f"axis는 roll/pitch/yaw만 됩니다 (받은 값: {axis})")

    def set_grip(self, percent: float, secs: float | None = None) -> str:
        """집게만 여닫는다(0=닫힘, 100=열림). 좌표 이동은 집게를 절대 안 건드린다."""
        value = max(0.0, min(100.0, float(percent)))
        self._io.before_move()
        with self._io.busy_lock():
            self._io.write({GRIPPER: value}, secs or ARM_CART_MOVE_SECS)
        return f"집게 {value:.0f}% ({'닫힘' if value < 15 else '열림' if value > 70 else '중간'})"

    # --- 내부 ---

    def _require_state(self) -> tuple[dict[str, float], ToolPose]:
        """이동해도 되는 상태인지 확인하고 (정규화값, 현재 좌표)를 준다.

        ⚠ 관절 읽기는 **여기 한 번뿐**이다. 예전엔 좌표·pan시드·변화량 검사가
        각각 읽어서 조그 한 번에 시리얼 왕복이 세 번 났다 — 팔 버스는 미러링·
        프리셋과 공유하는 자원이라, 아낄 수 있는 왕복은 아끼는 게 맞다.
        """
        if not self.config.has_zero:
            raise RuntimeError(
                "기구학 영점이 없습니다 — /settings의 [3D 좌표 영점]에서 먼저 등록하세요. "
                f"(힘 빼고 '{self.ZERO_POSE_LABEL}' 자세로 세운 뒤 누릅니다.)"
            )
        norms = self._io.read()
        degs = self._to_deg(norms)
        geom = self.config.geometry()
        r = kin.signed_radius(degs, geom)
        if r < ARM_CART_R_MIN:
            # 증상은 같아도 원인이 둘이라 문장을 나눈다 — 영점을 막 잡은 사람은
            # "세워 놨으니 당연한 것"을 알아야 하고, r이 음수인 사람은 팔이 몸통
            # 뒤로 넘어갔다는 걸 알아야 한다. 한 문장으로 뭉치면 둘 다 헤맨다.
            why = ("팔이 몸통 뒤로 넘어가 있습니다" if r < 0 else
                   "팔이 거의 수직으로 서 있습니다(영점 자세가 여기입니다)")
            raise RuntimeError(
                f"{why} — 수평거리 {r:.0f}mm < {ARM_CART_R_MIN:.0f}mm. 이 근처에서는 "
                "집게가 회전축 위에 있어 xyz 방향이 정해지지 않습니다. 프리셋으로 "
                "앞으로 뻗은 자세를 먼저 만든 뒤 좌표 이동을 쓰세요."
            )
        pose = kin.forward(degs, geom)
        self._last_pose = pose
        return norms, pose

    def _go(self, now_norms: dict[str, float], now: ToolPose,
            target: ToolPose, secs: float | None) -> str:
        geom = self.config.geometry()
        self._check_step(now, target)
        self._check_workspace(target, geom)

        seed = self._to_deg(now_norms).get("shoulder_pan", 0.0)
        try:
            degs = kin.inverse(target, geom, elbow_up=ARM_CART_ELBOW_UP, seed_pan=seed)
        except Unreachable as exc:
            raise RuntimeError(f"거기까지 못 갑니다 — {exc}") from exc

        norms = self._to_norm(degs)
        self._check_joint_limits(norms, degs)
        moved = self._check_actually_moves(norms, now_norms)

        self._io.before_move()
        with self._io.busy_lock():
            self._io.write(norms, secs if secs is not None else ARM_CART_MOVE_SECS)
            self._last_pose = target
        d = math.dist((now.x, now.y, now.z), (target.x, target.y, target.z))
        self._last_note = (
            f"x={target.x:.0f} y={target.y:.0f} z={target.z:.0f} "
            f"pitch={target.pitch:.0f}° roll={target.roll:.0f}°"
        )
        return (f"이동 {d:.1f}mm → {self._last_note} "
                f"(관절 최대 {moved:.1f}° 변화)")

    def _check_step(self, now: ToolPose, target: ToolPose) -> None:
        """한 번에 너무 크게 움직이는 걸 막는다 — 좌표 오타 하나가 팔을 던진다."""
        d = math.dist((now.x, now.y, now.z), (target.x, target.y, target.z))
        if d > ARM_CART_MAX_STEP_MM:
            raise RuntimeError(
                f"한 번에 {d:.0f}mm는 너무 큽니다(상한 {ARM_CART_MAX_STEP_MM:.0f}mm). "
                "나눠서 가세요 — 큰 이동은 프리셋으로 대략 자세를 잡은 뒤 좌표로 다듬는 게 안전합니다."
            )
        da = max(abs(kin.wrap180(target.pitch - now.pitch)),
                 abs(kin.wrap180(target.roll - now.roll)))
        if da > ARM_CART_MAX_STEP_DEG:
            raise RuntimeError(
                f"한 번에 {da:.0f}° 회전은 너무 큽니다(상한 {ARM_CART_MAX_STEP_DEG:.0f}°)."
            )

    def _check_workspace(self, target: ToolPose, geom: ArmGeometry) -> None:
        if target.z < ARM_CART_Z_MIN:
            raise RuntimeError(
                f"z={target.z:.0f}mm는 바닥 아래입니다(하한 {ARM_CART_Z_MIN:.0f}mm) — "
                "무대를 긁습니다."
            )
        if target.r < ARM_CART_R_MIN:
            raise RuntimeError(
                f"목표가 몸통에 너무 가깝습니다(수평 {target.r:.0f}mm < "
                f"{ARM_CART_R_MIN:.0f}mm) — 자기 몸을 칩니다."
            )
        if target.r > geom.reach_max:
            raise RuntimeError(
                f"목표 수평거리 {target.r:.0f}mm가 최대 사거리 {geom.reach_max:.0f}mm를 넘습니다."
            )

    def _check_joint_limits(self, norms: dict[str, float], degs: dict[str, float]) -> None:
        """정규화 ±100이 캘리브레이션된 가동 끝이다. **잘라서 보내지 않는다.**

        끝까지 밀어붙이면 서보가 Overload로 굳는다(이 팔에서 실제로 겪은 고장).
        그래서 여유(ARM_CART_NORM_MARGIN)를 남기고, 넘으면 어느 관절인지 말한다.
        """
        limit = 100.0 - ARM_CART_NORM_MARGIN
        over = {j: v for j, v in norms.items() if abs(v) > limit}
        if over:
            detail = ", ".join(
                f"{j} {v:+.0f}(한계 ±{limit:.0f}, {degs[j]:+.0f}°)" for j, v in sorted(over.items())
            )
            raise RuntimeError(
                f"관절 가동범위를 벗어납니다: {detail} — 그 방향으로는 더 못 갑니다."
            )

    def _check_actually_moves(self, norms: dict[str, float],
                              current: dict[str, float]) -> float:
        """서보 분해능 아래 지령을 거절한다. 반환값은 최대 관절 변화량(도).

        ⚠ CLAUDE.md의 "지령은 나가는데 아무 일도 안 일어난다"의 팔 버전이다.
        1mm 조그가 관절로 0.2°면 서보 2틱 — 되지만, 0.1mm는 0.02°로 0틱이라
        **물리적으로 아무 일도 안 일어난다.** 그걸 성공이라고 말하면 안 된다.
        """
        biggest = 0.0
        for j, v in norms.items():
            delta_norm = abs(v - float(current.get(j, v)))
            biggest = max(biggest, delta_norm * self._deg_per_norm(j))
        if biggest < ARM_CART_MIN_JOINT_DEG:
            raise RuntimeError(
                f"너무 작아서 실제로는 안 움직입니다 — 필요한 관절 변화가 {biggest:.3f}°"
                f"(문턱 {ARM_CART_MIN_JOINT_DEG}°, 서보 분해능 {DEG_PER_TICK:.3f}°/틱). "
                "스텝을 키우세요."
            )
        return biggest


def _try_acquire(lock) -> bool:
    """RLock이면 논블로킹으로 잡아보고, 아니면 그냥 잡는다."""
    acquire = getattr(lock, "acquire", None)
    if acquire is None:
        return True
    try:
        return bool(acquire(blocking=False))
    except TypeError:
        return bool(acquire(False))


def _release(lock) -> None:
    release = getattr(lock, "release", None)
    if release is not None:
        release()


# ----------------------------------------------------------------------
# 팔이 없을 때 — 개발 PC에서 화면과 계산을 그대로 돌린다
# ----------------------------------------------------------------------

class SimJointIO:
    """메모리 안의 가짜 팔. Mock 대시보드와 자체 검증(tools/…check.py)이 쓴다.

    실물과 다른 점은 **이동이 즉시 끝난다**는 것뿐이라, 좌표 계산·안전 검사·
    화면 표시는 젯슨에 올리기 전에 여기서 전부 검증된다.
    """

    # 정규화 -100..100이 실제 몇 도인지. 실물에서는 캘리브레이션이 알려주는 값이라
    # 팔마다 다르다 — 여기서는 SO-101을 끝에서 끝까지 쓸었을 때의 대략치를 쓴다.
    DEFAULT_SPANS = {"shoulder_pan": 240.0, "shoulder_lift": 240.0, "elbow_flex": 240.0,
                     "wrist_flex": 240.0, "wrist_roll": 360.0}

    def __init__(self, joints: dict[str, float] | None = None,
                 spans: dict[str, float] | None = None) -> None:
        self.joints = {j: 0.0 for j in kin.JOINTS}
        self.joints[GRIPPER] = 30.0
        if joints:
            self.joints.update(joints)
        self.spans = dict(self.DEFAULT_SPANS if spans is None else spans)
        self.writes = 0
        self._lock = threading.RLock()

    def read(self) -> dict[str, float]:
        return dict(self.joints)

    def write(self, target: dict[str, float], secs: float) -> None:
        self.joints.update({k: float(v) for k, v in target.items()})
        self.writes += 1

    def spans_deg(self) -> dict[str, float]:
        return dict(self.spans)

    def busy_lock(self):
        return self._lock

    def before_move(self) -> None:
        pass
