"""`/cmd_vel`(m/s, rad/s) → 주행 보드 한 줄. **여기가 단위 경계다.**

ROS는 m/s·rad/s로 말하고(REP-103), 보드 계약 v2는 mm/s·mdeg/s로 말하며
([`docs/보드-계약.md`](../../../../docs/보드-계약.md) §2①), 지금 꽂혀 있는 Uno
펌웨어는 **duty(-255..255)** 로 말한다. 세 단위가 만나는 곳은 이 파일 하나이고,
그래서 이 파일만 조심하면 된다.

  Twist ──m/s→mm/s──▶ 물리 지령 ──┬─ cap.units=1 ─▶  "C vx vy w"   (그대로)
                                  └─ cap.units=0 ─▶  "V dx dy dw"  (duty 환산)

**rclpy가 여기 없다.** 계산이 맞는지는 젯슨에 올려 보지 않고 PC에서 확인한다
(`ros2/tools/ros_selfcheck.py`) — kinematics.py·handeye.py와 같은 규칙이다.

────────────────────────────────────────────────────────────────────────
이 파일이 지키는 원칙 셋 (전부 이미 비싸게 배운 것들이다)

① **조용히 폴백하지 않는다.** 보드가 물리 단위를 받겠다고 해 놓고(`units=1`)
   캘리브레이션이 없으면(`calib=0`), duty로 몰래 내려가지 않고 **거절한다.**
   보드계약 §5.4의 `nocalib`이 그 뜻이고, cartesian.py가 팔에서 이미 지키는
   원칙이다.

② **정지마찰은 feedforward로 넘는다.** duty 경로에서 `duty = Ks + Kv·|v|`로
   쓰는 이유 — 이 로봇의 1번 병("지령은 나가는데 아무 일도 안 일어난다")은
   문턱 아래 크기가 물리적으로 0이기 때문이다(실측 문턱 ≈90). 비례항만
   쓰면 저속 지령이 통째로 사라진다.

③ **환산이 실측이 아니면 그렇다고 말한다.** duty↔속도 곡선은 아직 아무도 재지
   않았다. `DutyCalib.measured=False`면 모든 계획에 그 사실이 note로 붙고,
   노드가 그걸 로그와 진단 토픽에 그대로 흘린다. 근사값을 진실인 척하지 않는다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# 보드가 안 움직인다고 봐도 되는 크기. 부동소수 잔여물(1e-17 m/s)이 정지를
# 주행으로 바꾸지 않게 하는 문턱일 뿐, 물리적 의미는 없다.
EPS_MMS = 0.5
EPS_MDEGS = 500.0  # 0.5°/s


# ----------------------------------------------------------------------
# 프레이밍 — 보드계약 §4
# ----------------------------------------------------------------------

def checksum(payload: str) -> str:
    """페이로드 전 바이트의 XOR, 대문자 2자리 HEX."""
    crc = 0
    for ch in payload:
        crc ^= ord(ch)
    return f"{crc:02X}"


def framed(payload: str) -> bytes:
    """`<payload>*<XOR>\\n` — 펌웨어 v2가 받는 형식 그대로."""
    return f"{payload}*{checksum(payload)}\n".encode("ascii")


# ----------------------------------------------------------------------
# cap — 보드가 자기 능력을 말한다 (보드계약 §6)
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class Caps:
    """`cap ...` 한 줄을 파싱한 것. **모르는 보드는 레거시로 본다.**

    상한(vmax/vymax/wmax)이 0이면 "보드가 안 알려줬다"는 뜻이고, 그때는 클램프를
    하지 않는다 — 모르는 상한을 코드에 박아 넣는 것이 보드계약 §6이 없애려는
    바로 그 문제다.
    """

    proto: int = 1
    board: str = "unknown"
    fw: str = ""
    board_id: str = ""
    units: bool = False          # True = `C`(물리 단위) 지원
    closed_loop: bool = False
    calib: bool = False
    estop_hw: bool = False
    vmax_mms: int = 0
    vymax_mms: int = 0
    wmax_mdegs: int = 0

    @staticmethod
    def legacy() -> "Caps":
        """3초 안에 `cap`이 안 오면 이것 — 지금 꽂혀 있는 Uno 펌웨어다."""
        return Caps(proto=1, board="uno-moebius")

    @staticmethod
    def parse(line: str) -> "Caps":
        """`cap proto=2 fw=... units=1 ...` → Caps.

        모르는 필드는 무시한다(보드가 우리보다 새로울 수 있다). 아는 필드가
        깨져 있으면 그 필드만 기본값으로 두고 나머지는 살린다 — 한 글자 때문에
        보드 전체를 레거시 취급하면 진단이 더 어려워진다.
        """
        if not line or not line.strip().startswith("cap"):
            raise ValueError(f"cap 줄이 아니다: {line!r}")
        fields: dict[str, str] = {}
        for token in line.split()[1:]:
            if "=" in token:
                k, _, v = token.partition("=")
                fields[k] = v

        def _int(key: str, default: int = 0) -> int:
            try:
                return int(fields.get(key, default))
            except ValueError:
                return default

        return Caps(
            proto=_int("proto", 1),
            board=fields.get("board", "unknown"),
            fw=fields.get("fw", ""),
            board_id=fields.get("id", ""),
            units=_int("units") == 1,
            closed_loop=_int("closed_loop") == 1,
            calib=_int("calib") == 1,
            estop_hw=_int("estop_hw") == 1,
            vmax_mms=_int("vmax"),
            vymax_mms=_int("vymax"),
            wmax_mdegs=_int("wmax"),
        )


# ----------------------------------------------------------------------
# duty 환산 — 레거시 보드에서만 쓴다 (보드계약 §8)
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class DutyCalib:
    """속도 ↔ duty. **보드의 소유물**이지 코드의 상수가 아니다(보드계약 §8).

    `duty = Ks + Kv·|v|`  — Ks가 정지마찰을 넘기는 몫, Kv가 속도에 비례하는 몫.

    기본값의 출처:
      · `ks`/`ks_w` = 90 — CLAUDE.md의 **실측 문턱**이다(제자리 펄스 0.12s에서
        회전 w=75→0.00°, 100→0.20°, 진행 60→2.2px, 90→9.3px).
      · `kv`/`kv_w` = **아무도 안 쟀다.** 아래 `measured=False`가 그 뜻이다.
        재는 법은 docs/ros2-이행계획.md의 "duty 곡선 재기"에 있다.

    ⚠ 이 값이 틀리면 `/cmd_vel`의 m/s는 **비율만 맞고 크기는 틀린다.** 방향과
      상대 크기는 옳으므로 원격조종은 되지만, "0.2m/s로 3초 가라"는 안 된다.
    """

    ks: int = 90                 # 진행/게걸음 정지마찰 문턱 (duty)
    kv: float = 0.35             # duty per (mm/s)  → 300mm/s에서 duty 195
    ks_w: int = 90               # 회전 정지마찰 문턱 (duty)
    kv_w: float = 1.1            # duty per (°/s)   → 90°/s에서 duty 189
    max_duty: int = 255          # 지령 크기의 천장. BASE_MAX_PWM(듀티 상한)과 다른 층이다
    measured: bool = False       # ⚠ True로 바꾸려면 실제로 재야 한다

    @property
    def vmax_mms(self) -> float:
        """이 보드가 낼 수 있는 최고 직진 속도(추정)."""
        return max(0.0, (self.max_duty - self.ks) / self.kv) if self.kv > 0 else 0.0

    @property
    def wmax_degs(self) -> float:
        return max(0.0, (self.max_duty - self.ks_w) / self.kv_w) if self.kv_w > 0 else 0.0

    def duty_linear(self, v_mms: float) -> int:
        return _feedforward(v_mms, self.ks, self.kv, self.max_duty)

    def duty_angular(self, w_degs: float) -> int:
        return _feedforward(w_degs, self.ks_w, self.kv_w, self.max_duty)


def _feedforward(v: float, ks: int, kv: float, cap: int) -> int:
    """0이면 0, 아니면 **문턱을 먼저 넘고** 크기에 비례해서 더한다."""
    if abs(v) < 1e-9:
        return 0
    duty = min(float(cap), ks + kv * abs(v))
    return int(round(math.copysign(duty, v)))


# ----------------------------------------------------------------------
# 축 부호 — 보드계약 §14.1이 아직 안 닫혔다
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class AxisSigns:
    """ROS 규약(+x 앞, +y 왼쪽, +z 반시계)을 이 보드의 부호로 옮기는 마지막 한 겹.

    ⚠ **이 dataclass가 존재한다는 것 자체가 규약이 없다는 증거다**(보드계약 §14.1).
      실기에서 한 번 확정하면 값을 파라미터 기본값에 박고, 이 주석을 지우고,
      계약 문서의 결정 항목을 닫아라. 런타임 토글로 남겨 두면 다음 사람이 또
      "부호를 뒤집어도 똑같다"로 하루를 태운다.
    """

    vx: int = 1
    vy: int = 1
    w: int = 1


# ----------------------------------------------------------------------
# 계획 — 한 번의 /cmd_vel이 무엇이 되는가
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class Command:
    """보드로 나갈 것 + **왜 그렇게 됐는지**.

    `notes`가 비어 있지 않으면 지령이 요청 그대로가 아니라는 뜻이다. 노드는 이걸
    로그와 진단 토픽에 그대로 흘린다 — 조용히 깎인 지령이 이 로봇에서 가장 비싼
    실패 양식이기 때문이다.
    """

    payload: str | None = None                  # `C ...` / `S` — send_raw로 그대로
    duty: tuple[int, int, int] | None = None    # `V ...` — MotorLink.set_velocity로
    physical: tuple[int, int, int] = (0, 0, 0)  # mm/s, mm/s, mdeg/s (항상 채운다)
    notes: tuple[str, ...] = ()
    rejected: bool = False
    reason: str = ""

    @property
    def moving(self) -> bool:
        return any(self.physical) and not self.rejected


def to_physical(vx_ms: float, vy_ms: float, wz_rads: float,
                signs: AxisSigns = AxisSigns()) -> tuple[int, int, int]:
    """m/s, rad/s → mm/s, mm/s, mdeg/s (정수). **보드계약 §2①의 경계다.**

    정수로 반올림하는 이유는 계약 그대로다 — AVR에서 `%f`는 비싸고, 텍스트로
    왕복해도 비트가 안 바뀌어야 한다.
    """
    return (
        int(round(vx_ms * 1000.0)) * signs.vx,
        int(round(vy_ms * 1000.0)) * signs.vy,
        int(round(math.degrees(wz_rads) * 1000.0)) * signs.w,
    )


def plan(vx_ms: float, vy_ms: float, wz_rads: float,
         caps: Caps = Caps.legacy(),
         calib: DutyCalib = DutyCalib(),
         signs: AxisSigns = AxisSigns(),
         estop: bool = False) -> Command:
    """Twist 하나 → 보드로 나갈 한 줄."""
    notes: list[str] = []

    if estop:
        return Command(payload="S", physical=(0, 0, 0), rejected=True,
                       reason="비상정지 래치 중 — 해제 전에는 어떤 지령도 안 나간다")

    if not all(map(math.isfinite, (vx_ms, vy_ms, wz_rads))):
        return Command(payload="S", rejected=True,
                       reason=f"NaN/inf 지령 (vx={vx_ms} vy={vy_ms} wz={wz_rads})")

    vx, vy, w = to_physical(vx_ms, vy_ms, wz_rads, signs)

    # 보드가 알려준 상한에만 맞춘다. 안 알려줬으면(0) 자르지 않는다 — 모르는
    # 상한을 코드에 박는 것이 보드계약 §6이 없애려는 문제다.
    vx, note = _clamp(vx, caps.vmax_mms, "vx", "mm/s")
    notes += note
    vy, note = _clamp(vy, caps.vymax_mms, "vy", "mm/s")
    notes += note
    w, note = _clamp(w, caps.wmax_mdegs, "w", "mdeg/s")
    notes += note

    if abs(vx) < EPS_MMS and abs(vy) < EPS_MMS and abs(w) < EPS_MDEGS:
        # 정지는 슬루를 타지 않는다(보드계약 §10.2). 0을 보내는 것과 `S`는 다르다.
        return Command(payload="S", physical=(0, 0, 0), notes=tuple(notes))

    # ── 물리 단위 경로 (새 보드) ──
    if caps.units:
        if not caps.calib:
            # §5.4 nocalib — **duty로 몰래 내려가지 않는다.**
            return Command(
                physical=(vx, vy, w), rejected=True, notes=tuple(notes),
                reason="보드가 물리 단위를 받는다고 했지만 캘리브레이션이 없다"
                       "(cap.calib=0). 캘리브레이션을 먼저 재라 — "
                       "duty로 조용히 내려가지 않는다.")
        return Command(payload=f"C {vx} {vy} {w}", physical=(vx, vy, w),
                       notes=tuple(notes))

    # ── 레거시 duty 경로 (지금 꽂혀 있는 Uno) ──
    if not calib.measured:
        notes.append("duty 환산이 실측이 아니다 — 방향과 상대 크기는 맞지만 "
                     "m/s의 절대 크기는 못 믿는다 (DutyCalib.measured=False)")
    dx = calib.duty_linear(vx)
    dy = calib.duty_linear(vy)
    dw = calib.duty_angular(w / 1000.0)

    for value, limit, name, unit in ((vx, calib.vmax_mms, "vx", "mm/s"),
                                     (vy, calib.vmax_mms, "vy", "mm/s"),
                                     (w / 1000.0, calib.wmax_degs, "w", "°/s")):
        if limit and abs(value) > limit + 1e-6:
            notes.append(f"{name} {abs(value):.0f}{unit}는 이 보드의 추정 최고"
                         f"({limit:.0f}{unit})를 넘는다 — duty 상한에서 잘린다")

    # 메카넘 믹싱은 보드가 한다. 세 축 duty의 합이 천장을 넘으면 바퀴 하나가
    # 포화되어 **차체가 요청한 방향으로 안 간다**(회전이 먼저 먹힌다).
    if abs(dx) + abs(dy) + abs(dw) > calib.max_duty:
        notes.append(f"세 축 duty 합 {abs(dx) + abs(dy) + abs(dw)}이 천장"
                     f"({calib.max_duty})을 넘는다 — 바퀴가 포화되어 진행 방향이 "
                     "요청과 달라진다. 크기를 줄여라")

    return Command(duty=(dx, dy, dw), physical=(vx, vy, w), notes=tuple(notes))


def _clamp(value: int, limit: int, name: str, unit: str) -> tuple[int, list[str]]:
    if not limit or abs(value) <= limit:
        return value, []
    clamped = int(math.copysign(limit, value))
    return clamped, [f"{name} {value}{unit} → {clamped}{unit} (보드가 말한 상한)"]


__all__ = ["AxisSigns", "Caps", "Command", "DutyCalib", "checksum", "framed",
           "plan", "to_physical", "EPS_MMS", "EPS_MDEGS"]
