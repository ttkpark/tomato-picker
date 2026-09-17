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
    ARM_CART_MAX_STEP_JOINT_DEG,
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
    ARM_ID,
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
# 정규화 ±100이 캘리브레이션된 가동 끝이고, 그 앞에서 멈출 여유를 남긴 값.
NORM_LIMIT = 100.0 - ARM_CART_NORM_MARGIN
# lerobot이 캘리브레이션(관절별 raw 구간)을 남기는 곳. **팔 포트를 열지 않고**
# 정규화 1단위가 몇 도인지 아는 유일한 길이라, 포트를 남이 쥐고 있을 때 쓴다.
LEROBOT_CALIBRATION_DIR = "~/.cache/huggingface/lerobot/calibration/robots"


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
# 정규화값 ↔ 각도 — 팔이 없어도 되는 순수 계산
# ----------------------------------------------------------------------
# 식은 **여기 하나뿐이어야 한다.** 팔이 있을 때(CartesianArm)와 파일만 있을 때
# (NormLimits)가 각자 식을 들고 있으면, 같은 팔을 두 도구가 다르게 믿는다 —
# 이 저장소가 계통 경계에서 계속 경계하는 바로 그 병이다.
#
# 정규화값 ─→ 각도:  ref + 부호 × (지금 − 영점) × (도/단위)
# ref가 0이 아닌 이유는 교시 자세가 "곧게 세운" 자세이기 때문이다
# (그 자세에서 lift는 0°가 아니라 90°다). config.ARM_CART_ZERO_POSE_DEG 참고.


def norms_to_degrees(norms: dict[str, float], *, zero: dict[str, float],
                     ref: dict[str, float], signs: dict[str, float],
                     deg_per_norm: dict[str, float]) -> dict[str, float]:
    """정규화값 → 기구학 각도. 빠진 관절은 0으로 본다(kin.JOINTS 전부를 돌려준다)."""
    return {
        j: ref[j] + signs[j] * (float(norms.get(j, 0.0)) - zero[j]) * deg_per_norm[j]
        for j in kin.JOINTS
    }


def degrees_to_norms(degs: dict[str, float], *, zero: dict[str, float],
                     ref: dict[str, float], signs: dict[str, float],
                     deg_per_norm: dict[str, float]) -> dict[str, float]:
    """기구학 각도 → 정규화값(위의 역). **준 관절만** 돌려준다."""
    out = {}
    for j, deg in degs.items():
        if j not in kin.JOINTS:
            continue
        out[j] = zero[j] + (float(deg) - ref[j]) / (signs[j] * deg_per_norm[j])
    return out


def calibration_path(arm_id: str = ARM_ID,
                     root: str = LEROBOT_CALIBRATION_DIR) -> str | None:
    """lerobot 캘리브레이션 파일을 찾는다. 없으면 None.

    로봇 종류 디렉터리 이름(so_follower / so101_follower …)은 lerobot 판마다
    달라서 **이름으로 짚지 않고 훑는다** — 판이 바뀔 때마다 상수를 고치게 두면
    조용히 "못 읽었다"가 되고, 그러면 아래 load_norm_limits가 한계를 모른 채
    돈다(그게 09-18에 못 가는 표적을 다섯 개 뽑은 원인이다).
    """
    base = os.path.expanduser(root)
    try:
        kinds = sorted(os.listdir(base))
    except OSError:
        return None
    for kind in kinds:
        path = os.path.join(base, kind, f"{arm_id}.json")
        if os.path.isfile(path):
            return path
    return None


def spans_deg_from_calibration(arm_id: str = ARM_ID,
                               root: str = LEROBOT_CALIBRATION_DIR) -> dict[str, float]:
    """정규화 -100..100이 실제 몇 도인지를 **파일에서** 읽는다.

    JointIO.spans_deg()와 같은 식((range_max−range_min)×도/틱)이다. 다른 점은
    팔을 안 연다는 것뿐 — 포트는 한 프로세스만 열 수 있으므로(CLAUDE.md),
    ROS가 팔을 쥐고 도는 동안 바깥 도구가 이 숫자를 얻는 길은 이것뿐이다.
    """
    path = calibration_path(arm_id, root)
    if not path:
        return {}
    try:
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)
    except (OSError, ValueError):
        return {}
    out: dict[str, float] = {}
    for name, c in (raw or {}).items():
        try:
            out[name] = abs(int(c["range_max"]) - int(c["range_min"])) * DEG_PER_TICK
        except (KeyError, TypeError, ValueError):
            continue
    return out


class NormLimits:
    """관절 가동범위 판정 — **팔을 열지 않고** 파일만 읽어서.

    왜 있나: 2026-09-18 실기에서 `/arm/move_to_point` 5회가 5회 다
    `elbow_flex` 정규화 −121~−138(한계 ±98)로 거절됐다. 표적을 뽑은 쪽이
    **사거리만** 보고 있었기 때문이다 — 사거리는 링크 길이(기구학)가 정하고
    가동범위는 캘리브레이션이 정한다. **다른 것이다.** 갈 수 없는 자리를
    시험하면 0/5가 나오고, 그 0은 팔이 아니라 도구가 만든 0이다.
    """

    def __init__(self, *, zero: dict[str, float], ref: dict[str, float],
                 signs: dict[str, float], deg_per_norm: dict[str, float],
                 source: str, limit: float = NORM_LIMIT) -> None:
        self.zero = dict(zero)
        self.ref = dict(ref)
        self.signs = dict(signs)
        self.deg_per_norm = dict(deg_per_norm)
        self.source = source
        self.limit = float(limit)

    def norms(self, degs: dict[str, float]) -> dict[str, float]:
        return degrees_to_norms(degs, zero=self.zero, ref=self.ref,
                                signs=self.signs, deg_per_norm=self.deg_per_norm)

    def violations(self, degs: dict[str, float]) -> dict[str, float]:
        """한계를 넘은 관절만 {이름: 정규화값}. `_check_joint_limits`와 같은 판정."""
        return {j: v for j, v in self.norms(degs).items() if abs(v) > self.limit}

    def degree_range(self, joint: str) -> tuple[float, float]:
        """이 관절이 한계 안에서 취할 수 있는 **기구학 각도 구간**.

        표적을 뽑는 쪽이 쓴다 — 못 가는 구간에서 던져 놓고 기각하면 영원히
        안 뽑히는 수가 있다(이 팔의 elbow_flex가 실제로 그랬다: 뽑기 구간
        −100~−20° 중 −38.9°까지만 살아 있다).
        """
        lo = self.ref[joint] + self.signs[joint] * (-self.limit - self.zero[joint])             * self.deg_per_norm[joint]
        hi = self.ref[joint] + self.signs[joint] * (self.limit - self.zero[joint])             * self.deg_per_norm[joint]
        return (min(lo, hi), max(lo, hi))

    def describe(self, degs: dict[str, float]) -> str:
        over = self.violations(degs)
        if not over:
            return ""
        return ", ".join(f"{j} {v:+.0f}(한계 ±{self.limit:.0f})"
                         for j, v in sorted(over.items()))


def load_norm_limits(path: str = ARM_CART_FILE, arm_id: str = ARM_ID,
                     root: str = LEROBOT_CALIBRATION_DIR,
                     ) -> tuple[NormLimits | None, str]:
    """영점 파일 + 캘리브레이션 파일로 NormLimits를 만든다. (limits, 한 줄 설명).

    **짐작으로 채우지 않는다.** 영점이 없거나 관절 하나라도 도/단위를 모르면
    None을 준다 — FALLBACK_DEG_PER_NORM(0.9)로 메우면 갈 수 있는 표적을
    거절하거나 못 가는 표적을 통과시키는데, 둘 다 조용히 틀린다. 모르면
    "모른다"고 말하는 편이 낫다(부르는 쪽이 기록에 limits=none을 남긴다).
    """
    config = FrameConfig(path)
    if not config.has_zero:
        return None, f"영점이 없다({config.path}) — 이 PC에서는 가동범위를 모른다"
    spans = spans_deg_from_calibration(arm_id, root)
    deg_per_norm: dict[str, float] = {}
    unknown = []
    for j in kin.JOINTS:
        override = config.deg_per_norm_override(j)
        if override:
            deg_per_norm[j] = override
            continue
        span = spans.get(j)
        if span:
            deg_per_norm[j] = abs(float(span)) / 200.0
            continue
        unknown.append(j)
    if unknown:
        where = calibration_path(arm_id, root) or f"{root}/*/{arm_id}.json (없음)"
        return None, f"도/단위를 모르는 관절: {', '.join(unknown)} — {where}"
    cal = calibration_path(arm_id, root)
    source = config.path + (f" + {cal}" if cal else " (deg_per_norm 전부 파일에 적혀 있다)")
    limits = NormLimits(zero=config.zero(), ref=config.ref_deg(),
                        signs={j: config.sign(j) for j in kin.JOINTS},
                        deg_per_norm=deg_per_norm, source=source)
    return limits, f"한계 ±{limits.limit:.0f} ← {source}"


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

    # 식 자체는 모듈 함수(norms_to_degrees/degrees_to_norms)에 있다 — 팔이 없는
    # 도구(NormLimits)와 **같은 줄**을 쓰게 하려고 밖으로 뺐다.

    def _frame(self) -> dict:
        """지금 팔의 정규화↔각도 변환표. 도/단위는 캘리브레이션에서 읽는다."""
        return {
            "zero": self.config.zero(),
            "ref": self.config.ref_deg(),
            "signs": {j: self.config.sign(j) for j in kin.JOINTS},
            "deg_per_norm": {j: self._deg_per_norm(j) for j in kin.JOINTS},
        }

    def _to_deg(self, norms: dict[str, float]) -> dict[str, float]:
        return norms_to_degrees(norms, **self._frame())

    def _to_norm(self, degs: dict[str, float]) -> dict[str, float]:
        return degrees_to_norms(degs, **self._frame())

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
                       "max_step_deg": ARM_CART_MAX_STEP_DEG,
                       "max_step_joint_deg": ARM_CART_MAX_STEP_JOINT_DEG},
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

    def travel_to(self, x: float | None = None, y: float | None = None,
                  z: float | None = None, pitch: float | None = None,
                  roll: float | None = None, secs: float | None = None) -> str:
        """**먼 좌표로 여러 걸음에 걸쳐** 이동. 한 걸음 상한은 그대로 지킨다.

        `move_to`는 한 번에 ARM_CART_MAX_STEP_MM(80mm)까지만 간다 — 좌표 오타
        하나가 팔을 던지는 것을 막는 값이다. 그래서 "임의의 자리로 보내라"는
        일(졸업기준3 · `/arm/move_to_point`)은 `move_to` 하나로는 **원리상**
        불가능했다(2026-09-18 T26: 572~720mm 5/5 거절).

        쪼개는 책임을 여기에 둔 이유: 걸음마다 `_go`를 다시 지나가야 바닥·몸통·
        사거리·관절한계 검사를 **매 걸음** 받을 수 있고, 그 검사들은 이 유닛
        안에만 있다. 호출자가 쪼개면 그 검사를 호출자가 베껴야 한다.
        ⚠ 그래도 **`move_to`를 조용히 바꾸지는 않았다** — 조그 버튼이 오타를
          막는 것은 그대로 남아야 해서, 긴 이동은 이름이 다른 길로만 열어 둔다.

        중간에 막히면 **어느 걸음에서 왜**인지를 말하고 멈춘다(그 자리에 선다).

        ⚠ **직교 직선은 관절공간 직선이 아니다.** 양 끝이 둘 다 갈 수 있는
          자리여도 그 사이가 관절한계 밖일 수 있다 — 2026-09-18 실기에서
          `/arm/move_to_point` 5회가 5회 다 **첫 걸음**에서 거절됐다
          (`elbow_flex` 정규화 −124~−133, 한계 ±98). 목표는 5/5 통과한
          뒤였다. 그래서 여기는 길을 **둘** 가진다: 직교 직선을 먼저 걸어
          보고, 막히면 관절공간 보간으로 돌아간다(`plan_joint_steps`).
          고르는 이유는 docs/arm-cartesian.md §4에 적어 뒀다.
        """
        norms, now = self._require_state()
        target = ToolPose(
            x=now.x if x is None else float(x),
            y=now.y if y is None else float(y),
            z=now.z if z is None else float(z),
            pitch=now.pitch if pitch is None else float(pitch),
            roll=now.roll if roll is None else float(roll),
        )
        geom = self.config.geometry()

        # **목표부터 본다** — 갈 수 없는 곳으로 절반쯤 가 놓고 거절하면, 팔은
        # 엉뚱한 자리에 서 있고 사람은 무엇이 틀렸는지 모른다.
        self._check_workspace(target, geom)
        now_degs = self._to_deg(norms)
        try:
            end_degs = kin.inverse(target, geom, elbow_up=ARM_CART_ELBOW_UP,
                                   seed_pan=now_degs.get("shoulder_pan", 0.0))
        except Unreachable as exc:
            raise RuntimeError(f"거기까지 못 갑니다 — {exc}") from exc
        self._check_joint_limits(self._to_norm(end_degs), end_degs)

        # 첫 번째 길 — 직교 직선. **움직이기 전에 끝까지 걸어 본다**(위 ⚠).
        line = plan_steps(now, target)
        if len(line) > MAX_TRAVEL_STEPS:
            line_block = (f"{len(line)}걸음이 필요하다(상한 {MAX_TRAVEL_STEPS}걸음) — "
                          "그렇게 먼 목표는 좌표가 잘못됐을 가능성이 크다")
        else:
            line_block = self._walk(norms, now, line, geom, joint_space=False)
        if line_block is None:
            return self._run_path(norms, now, line, geom, secs, joint_space=False,
                                  target=target, end_degs=end_degs)

        # 두 번째 길 — 관절공간. 여기서도 걸음마다 같은 검사를 받는다.
        jsteps = plan_joint_steps(now_degs, end_degs, geom)
        if len(jsteps) > MAX_TRAVEL_STEPS_JOINT:
            jblock = (f"{len(jsteps)}걸음이 필요하다"
                      f"(상한 {MAX_TRAVEL_STEPS_JOINT}걸음)")
        else:
            jblock = self._walk(norms, now, jsteps, geom, joint_space=True)
        if jblock is not None:
            raise RuntimeError(
                "갈 길이 없습니다 — 직선 경로: " + line_block
                + " / 관절공간 경로: " + jblock
                + ". 프리셋이나 arm_extend로 중간 자세를 먼저 만든 뒤 다시 부르세요."
            )
        return self._run_path(norms, now, jsteps, geom, secs, joint_space=True,
                              target=target, end_degs=end_degs,
                              why=f"직선 경로가 막혀 관절공간으로 돌아갔다({line_block})")

    # --- 경로 (travel_to가 쓴다) ---

    def _walk(self, norms: dict[str, float], now: ToolPose, steps: list,
              geom: ArmGeometry, *, joint_space: bool) -> str | None:
        """경로를 **팔을 안 움직이고** 끝까지 걸어 본다. 막히면 그 이유, 아니면 None.

        왜 미리 걷나 — 절반쯤 가서 거절하면 팔은 엉뚱한 자리에 서 있고, 그
        자리가 다음 시도의 시작 자세가 된다(09-18에 실제로 그렇게 굴렀다).
        길이 둘이므로 **고르기 전에** 둘 다 걸어 볼 수 있어야 한다.

        검사는 `_go`/`_go_joints`가 실제로 할 것과 **같은 것을 같은 순서로** 한다.
        다음 걸음의 상태는 지령값으로 잇는다(실제로는 걸음마다 다시 읽는다 —
        서보가 조금 못 미쳐도 그때 다시 검사를 받으므로 여기서는 지령이면 된다).
        """
        pose, cur = now, dict(norms)
        for i, step in enumerate(steps, 1):
            try:
                if joint_space:
                    degs = dict(step)
                    nxt = kin.forward(degs, geom)
                else:
                    nxt = step
                    degs = kin.inverse(nxt, geom, elbow_up=ARM_CART_ELBOW_UP,
                                       seed_pan=self._to_deg(cur).get("shoulder_pan", 0.0))
                self._check_step(pose, nxt)
                self._check_workspace(nxt, geom)
                step_norms = self._to_norm(degs)
                # 관절공간 길은 **빠져나오는 길**이라 이미 범위 밖에 선 관절을
                # 그대로 막지 않는다(`_check_joint_limits`의 current 참고).
                self._check_joint_limits(step_norms, degs,
                                         current=cur if joint_space else None)
                self._check_actually_moves(step_norms, cur)
                # 걸음 사이마다 `_require_state`가 다시 도는 것과 같은 가드다 —
                # 여기서 안 보면 "갈 수 있다"고 해 놓고 중간에 죽는다.
                r = kin.signed_radius(degs, geom)
                if r < ARM_CART_R_MIN:
                    raise RuntimeError(
                        f"도중에 수평거리가 {r:.0f}mm로 떨어진다(하한 {ARM_CART_R_MIN:.0f}mm)"
                    )
            except (RuntimeError, Unreachable, ValueError) as exc:
                return f"{len(steps)}걸음 중 {i}번째에서 막힌다 — {exc}"
            pose, cur = nxt, {**cur, **step_norms}
        return None

    def _run_path(self, norms: dict[str, float], now: ToolPose, steps: list,
                  geom: ArmGeometry, secs: float | None, *, joint_space: bool,
                  target: ToolPose, end_degs: dict[str, float], why: str = "") -> str:
        """`_walk`가 통과시킨 경로를 실제로 걷는다.

        ⚠ 계획한 웨이포인트를 그대로 따라가지 **않는다** — 걸음마다 지금 자리에서
          남은 길을 다시 짠다. 서보는 지령에 조금 못 미친 자리에 선다(arm_extend
          실측: 관절 오차 최대 4.2° ≈ 팔 끝에서 ~28mm). 고정 웨이포인트를 쓰면
          그 못 미친 만큼이 다음 걸음에 **얹혀** 한 걸음 상한을 넘고, "3걸음 중
          2번째에서 110mm는 너무 큽니다"로 가드가 제 실수에 걸린다(2026-09-18
          실기 3/5가 이것이었다). 지금 자리에서 다시 짜면 걸음은 언제나 상한
          이하이고, 못 미친 만큼은 다음 걸음이 메운다.

        `steps`는 `_walk`가 통과시킨 계획이다 — 여기서는 **걸음 수 예산**과
        사람에게 보여 줄 숫자로만 쓴다(길 자체는 같은 함수가 같은 규칙으로 다시 짠다).
        """
        budget = MAX_TRAVEL_STEPS_JOINT if joint_space else MAX_TRAVEL_STEPS
        planned = len(steps)
        walked = 0
        stalled = 0

        def left() -> float:
            """남은 길 — 관절공간이면 최대 관절 각(°), 직선이면 끝점 거리(mm)."""
            if joint_space:
                cur = self._to_deg(norms)
                return max(abs(float(end_degs[j]) - cur.get(j, 0.0)) for j in kin.JOINTS)
            return math.dist((now.x, now.y, now.z), (target.x, target.y, target.z))

        floor = PROGRESS_MIN_DEG if joint_space else PROGRESS_MIN_MM
        unit = "°" if joint_space else "mm"
        gap = left()
        while walked < budget:
            remaining = (plan_joint_steps(self._to_deg(norms), end_degs, geom)
                         if joint_space else plan_steps(now, target))
            last = len(remaining) == 1
            try:
                if joint_space:
                    self._go_joints(norms, now, remaining[0], secs)
                else:
                    self._go(norms, now, remaining[0], secs)
            except RuntimeError as exc:
                # 마지막 한 걸음이 "서보 분해능 아래"면 그건 실패가 아니라 **도착**이다
                # — 남은 거리가 틱 하나보다 작다는 뜻이다. 그 밖에는 그대로 올린다.
                if last and walked and "너무 작아서" in str(exc):
                    break
                raise RuntimeError(
                    f"{planned}걸음 중 {walked + 1}번째에서 멈췄습니다 — {exc}"
                ) from exc
            walked += 1
            # 걸음마다 **실제로 어디에 있는지 다시 읽는다.** 자세 가드도 다시 받는다.
            norms, now = self._require_state()
            if last:
                break
            # 남은 길을 다시 짜는 걸음은 **줄어들 때만** 뜻이 있다. 서보가 못
            # 버티는 자세(중력 처짐)에서는 같은 지령을 다시 보내도 같은 자리에
            # 선다 — 2026-09-18 실기에서 3/5가 그렇게 예산 32걸음을 다 태웠다.
            # 진전이 없으면 **그 사실과 남은 거리를 말하고** 멈춘다.
            now_gap = left()
            if gap - now_gap < floor:
                stalled += 1
                if stalled >= PROGRESS_STALL_STEPS:
                    away = math.dist((now.x, now.y, now.z),
                                     (target.x, target.y, target.z))
                    tail = f" [{why}]" if why else ""
                    return (
                        f"{walked}걸음에서 더 안 갑니다 — 목표에서 {away:.0f}mm "
                        f"떨어진 자리에 섰고 {PROGRESS_STALL_STEPS}걸음째 남은 길이 "
                        f"{floor:.1f}{unit}도 안 줄었습니다(서보 추종오차·중력 처짐). "
                        f"도착 x={now.x:.0f} y={now.y:.0f} z={now.z:.0f}{tail}"
                    )
            else:
                stalled = 0
            gap = now_gap
        else:
            raise RuntimeError(
                f"{budget}걸음을 걷고도 목표에 못 닿았습니다 — 서보가 지령을 계속 "
                "못 따라가고 있습니다(전원·부하·토크를 보세요)."
            )
        tail = f" [{why}]" if why else ""
        return (f"{walked}걸음으로 이동 완료 → {self._last_note} "
                f"(도착 x={now.x:.0f} y={now.y:.0f} z={now.z:.0f}){tail}")

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

    def _go_joints(self, now_norms: dict[str, float], now: ToolPose,
                   degs: dict[str, float], secs: float | None) -> str:
        """**관절각을 직접 주는** 한 걸음. 직교 직선이 막혔을 때만 쓴다.

        `_go`와 다른 것은 "어디로 갈지"를 IK가 아니라 부르는 쪽이 이미 정했다는
        것뿐이다 — 그래서 도착 좌표는 FK로 얻는다. 검사는 **같은 것을 같은
        순서로** 받는다(바닥·몸통·사거리·관절한계·크기). 집게는 안 건드린다.
        """
        geom = self.config.geometry()
        nxt = kin.forward(degs, geom)
        self._check_step(now, nxt)
        self._check_workspace(nxt, geom)
        norms = self._to_norm(degs)
        self._check_joint_limits(norms, degs, current=now_norms)
        moved = self._check_actually_moves(norms, now_norms)

        self._io.before_move()
        with self._io.busy_lock():
            self._io.write(norms, secs if secs is not None else ARM_CART_MOVE_SECS)
            self._last_pose = nxt
        d = math.dist((now.x, now.y, now.z), (nxt.x, nxt.y, nxt.z))
        self._last_note = (
            f"x={nxt.x:.0f} y={nxt.y:.0f} z={nxt.z:.0f} "
            f"pitch={nxt.pitch:.0f}° roll={nxt.roll:.0f}°"
        )
        return (f"관절 이동 {d:.1f}mm → {self._last_note} "
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

    def _check_joint_limits(self, norms: dict[str, float], degs: dict[str, float],
                            current: dict[str, float] | None = None) -> None:
        """정규화 ±100이 캘리브레이션된 가동 끝이다. **잘라서 보내지 않는다.**

        끝까지 밀어붙이면 서보가 Overload로 굳는다(이 팔에서 실제로 겪은 고장).
        그래서 여유(ARM_CART_NORM_MARGIN)를 남기고, 넘으면 어느 관절인지 말한다.

        `current`를 주면 **이미 범위 밖에 선 관절**은 "더 나빠지지 않는 한" 통과
        시킨다. 왜 — 범위 밖을 무조건 막으면 거기서 빠져나올 수가 없다
        (arm_extend.py가 같은 이유로 같은 규칙을 쓴다: 2026-09-18 실측으로
        `shoulder_pan` 98.8 · `elbow_flex` 103.2로 **가만히 있는데도** 모든
        걸음이 막혀 있었다). 기준은 "범위 밖이냐"가 아니라 "이 걸음이 상황을
        더 나쁘게 하느냐"다. ⚠ 조그·`move_to`는 current를 안 준다 — 빠져나오는
        길에서만 느슨해야 하고, 평소에는 가드가 가드여야 한다.
        """
        limit = NORM_LIMIT
        over = {}
        for j, v in norms.items():
            if abs(v) <= limit:
                continue
            if current is not None and abs(v) <= abs(float(current.get(j, v))) + 1e-6:
                continue
            over[j] = v
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


MAX_TRAVEL_STEPS = 16
"""`travel_to`가 한 번에 허용하는 걸음 수 상한.

사거리가 {reach}mm이므로 도달 가능한 두 점 사이 직선거리는 아무리 멀어도
2·사거리(≈780mm)를 못 넘는다 — 상한 80mm면 10걸음이다. 16으로 잡아 두면
정상 목표는 전부 통과하고, 그보다 많이 필요한 값은 애초에 갈 수 없는 목표다
(목표 자체 검사에서 먼저 걸린다). 무한 루프 방어용 값이다.
"""


def plan_steps(now: ToolPose, target: ToolPose,
               max_step_mm: float = ARM_CART_MAX_STEP_MM,
               max_step_deg: float = ARM_CART_MAX_STEP_DEG) -> list[ToolPose]:
    """지금 자세에서 목표까지를 **상한 이하 걸음들**로 쪼갠 중간 목표 목록.

    마지막 원소는 정확히 `target`이다(근처까지 갔다고 도달이라 하지 않는다).
    직선 보간이며 각(pitch·roll)도 같은 걸음 수로 나눈다 — 이동과 회전 중
    더 많이 쪼개야 하는 쪽이 걸음 수를 정한다.

    ⚠ **상한을 키워서 푸는 게 아니다.** 상한(ARM_CART_MAX_STEP_MM)은 서보가 한
      번에 뛰면 위험해서 있는 값이고, 여기서 하는 일은 그 상한을 지키면서 여러
      번 가는 것이다. 각 걸음은 `_go`를 그대로 지나가므로 바닥·몸통·사거리·
      관절한계 검사를 **걸음마다** 다시 받는다.
    """
    d = math.dist((now.x, now.y, now.z), (target.x, target.y, target.z))
    da = max(abs(kin.wrap180(target.pitch - now.pitch)),
             abs(kin.wrap180(target.roll - now.roll)))
    n = max(1,
            math.ceil(d / max_step_mm - 1e-9),
            math.ceil(da / max_step_deg - 1e-9))
    return [ToolPose(
        x=now.x + (target.x - now.x) * i / n,
        y=now.y + (target.y - now.y) * i / n,
        z=now.z + (target.z - now.z) * i / n,
        pitch=now.pitch + kin.wrap180(target.pitch - now.pitch) * i / n,
        roll=now.roll + kin.wrap180(target.roll - now.roll) * i / n,
    ) for i in range(1, n + 1)]


PROGRESS_MIN_MM = 2.0
PROGRESS_MIN_DEG = 0.5
PROGRESS_STALL_STEPS = 2
"""걸음이 "진전"으로 쳐지는 최소량과, 그게 몇 걸음 이어지면 멈추는가.

걸음마다 남은 길을 다시 짜는 구조(`_run_path`)에는 함정이 하나 있다 — **서보가
물리적으로 못 버티는 자세**에서는 같은 지령을 몇 번 보내도 같은 자리에 선다.
그러면 "다시 짜기"가 영원한 재시도가 된다(2026-09-18 실기: 3/5가 예산 32걸음을
다 태우고 `err=None`으로 끝났다 — 몇 mm 모자랐는지조차 기록에 안 남았다).

⚠ **재는 자가 길마다 다르다.** 직선 길은 끝점이 목표로 곧장 가므로 "남은 mm"가
  곧 진전이다. 관절공간 길은 **호를 그린다** — 팔을 접었다 펴는 동안 끝점과
  목표의 직선거리가 잠깐 안 줄거나 늘 수도 있다. 거기에 mm 자를 대면 정상적인
  경로를 "안 간다"고 잘라 버린다(09-18 실기에서 3/5가 177~304mm를 남기고 그렇게
  잘렸다). 관절공간의 진전은 **남은 관절 각도**로 잰다 — 그쪽은 보간이 단조롭다.
"""

MAX_TRAVEL_STEPS_JOINT = 32
"""관절공간 경로의 걸음 수 상한.

직선(16)보다 큰 이유는 한 걸음의 뜻이 다르기 때문이다 — 여기서는 "관절이 몇 도
도는가"가 상한이라, 팔을 뒤에서 앞으로 크게 접어 넘기는 경로는 관절 240°를 12°씩
20걸음 넘게 간다(arm_extend가 특이점을 빠져나올 때 실제로 21구간을 썼다).
그래도 상한은 있어야 한다 — 무한 루프 방어이자, 한 번의 이동이 서비스 타임아웃
(move5_check는 90초)을 넘기지 않게 하는 값이다. 32걸음 × 0.6초 ≈ 20초.
"""


def plan_joint_steps(now_degs: dict[str, float], end_degs: dict[str, float],
                     geom: ArmGeometry = ArmGeometry(),
                     max_step_deg: float = ARM_CART_MAX_STEP_JOINT_DEG,
                     max_step_mm: float = ARM_CART_MAX_STEP_MM,
                     max_steps: int = MAX_TRAVEL_STEPS_JOINT) -> list[dict[str, float]]:
    """지금 관절각에서 목표 관절각까지를 **관절공간 선형보간**으로 쪼갠다.

    `plan_steps`(직교 직선)와 무엇이 다른가 — **중간 자세가 관절한계 안에 머문다.**
    한계는 관절마다 구간 하나씩이고 정규화값은 각도의 1차식이라, 양 끝이 모두
    한계 안이면 그 사이를 잇는 선분도 통째로 한계 안이다(볼록집합). 직교 직선에는
    그런 보장이 전혀 없다 — 2026-09-18 실기가 그걸 5/5로 보여 줬다.

    대가는 **집게가 지나가는 자리를 예측하기 어렵다**는 것이다(직선이 아니라 호를
    그린다). 그래서 두 번째 길이고, 지나가는 자리는 걸음마다 FK로 다시 본다.

    걸음 수는 상한 **둘**이 정한다: 관절 하나가 한 걸음에 max_step_deg를 넘지
    않고, 집게가 한 걸음에 max_step_mm를 넘게 움직이지 않는다. 뒤쪽이 따로 필요한
    이유는 팔을 편 자세에서 관절 12°가 끝점에서 90mm가 되기 때문이다 —
    각도 상한 하나로는 "한 번에 팔을 던지지 않는다"를 못 지킨다.
    """
    # ⚠ **wrap180을 쓰지 않는다.** 이 관절들은 한 바퀴 도는 축이 아니라 양 끝이
    # 막힌 구간이다 — "짧은 쪽으로 돌기"는 그 구간 **밖으로** 나가는 길일 수 있다
    # (실측: pan −84.9° → 102.5°를 wrap하면 −257.5°로 가서 한계 −105.3°를 뚫는다).
    # 그냥 두 각을 곧장 이으면 값이 두 끝 사이에 머물러 위의 볼록성이 성립한다.
    delta = {j: float(end_degs.get(j, 0.0)) - float(now_degs.get(j, 0.0))
             for j in kin.JOINTS}
    biggest = max(abs(v) for v in delta.values())
    n = max(1, math.ceil(biggest / max_step_deg - 1e-9))

    def path(count: int) -> list[dict[str, float]]:
        return [{j: float(now_degs.get(j, 0.0)) + delta[j] * i / count for j in kin.JOINTS}
                for i in range(1, count + 1)]

    def worst_hop_mm(steps: list[dict[str, float]]) -> float:
        prev = kin.forward(now_degs, geom)
        worst = 0.0
        for degs in steps:
            here = kin.forward(degs, geom)
            worst = max(worst, math.dist((prev.x, prev.y, prev.z), (here.x, here.y, here.z)))
            prev = here
        return worst

    # 끝점 거리는 걸음 수에 정확히 반비례하지 않는다(호다) — 그래서 재고 늘린다.
    # 상한에 닿으면 그대로 돌려준다: 여기서 조용히 자르면 "쪼갠 척하고 한 번에
    # 뛰는" 경로가 되므로, 넘는 걸음은 `_walk`의 `_check_step`이 말하게 둔다.
    steps = path(n)
    while n < max_steps:
        worst = worst_hop_mm(steps)
        if worst <= max_step_mm:
            break
        n = min(max_steps, max(n + 1, math.ceil(n * worst / max_step_mm)))
        steps = path(n)
    return steps


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
