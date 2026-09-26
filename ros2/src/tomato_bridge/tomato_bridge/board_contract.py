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
# 응답 및 프로토콜 파싱 — 보드계약 §4, §5, §12
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class Response:
    """보드 응답 한 줄을 파싱한 결과 (보드계약 §4, §5).

    보드 응답 형식:
      - `ok <cmd> [args...]`
      - `nak <code_or_reason>`
      - `cap ...`
      - `hb ...`
      - `boot ...`
    """

    raw: str
    kind: str             # "ok", "nak", "cap", "hb", "boot", "unknown"
    cmd: str = ""         # kind=="ok"일 때 대상 명령 (예: "S", "X")
    code: str = ""        # kind=="nak"일 때 사유 코드 (예: "crc", "nocrc", "unsupported", "nocalib", "estop", "range")
    args: tuple[str, ...] = ()
    cap: Caps | None = None
    hb: Heartbeat | None = None

    @property
    def is_ok(self) -> bool:
        return self.kind == "ok"

    @property
    def is_nak(self) -> bool:
        return self.kind == "nak"


def parse_response(line: str) -> Response:
    """보드에서 수신된 텍스트 한 줄 파싱."""
    clean = line.strip()
    if not clean:
        return Response(raw=line, kind="unknown")

    # 체크섬 분리: <payload>*<HEX>
    payload = clean
    if "*" in clean:
        payload, _, _ = clean.partition("*")
        payload = payload.strip()

    tokens = payload.split()
    if not tokens:
        return Response(raw=line, kind="unknown")

    header = tokens[0]
    if header == "ok":
        cmd = tokens[1] if len(tokens) > 1 else ""
        args = tuple(tokens[2:]) if len(tokens) > 2 else ()
        return Response(raw=line, kind="ok", cmd=cmd, args=args)
    elif header == "nak":
        code = tokens[1] if len(tokens) > 1 else ""
        args = tuple(tokens[2:]) if len(tokens) > 2 else ()
        return Response(raw=line, kind="nak", code=code, args=args)
    elif header == "cap":
        try:
            c = Caps.parse(payload)
            return Response(raw=line, kind="cap", cap=c, args=tuple(tokens[1:]))
        except Exception:
            return Response(raw=line, kind="cap", args=tuple(tokens[1:]))
    elif header == "hb":
        try:
            h = Heartbeat.parse(payload)
            return Response(raw=line, kind="hb", hb=h, args=tuple(tokens[1:]))
        except Exception:
            return Response(raw=line, kind="hb", args=tuple(tokens[1:]))
    elif header == "boot":
        return Response(raw=line, kind="boot", args=tuple(tokens[1:]))
    else:
        return Response(raw=line, kind="unknown", args=tuple(tokens))


class ProtocolParser:
    """보드계약 §4, §5.4, §12 프로토콜 수신 및 상태 머신.

    체크섬 검증, strict CRC 모드 전환, nak 코드 추적 및 cap/hb 이벤트 처리를 관장한다.
    """

    def __init__(self, expected_proto: int = 2) -> None:
        self.expected_proto = expected_proto
        self.strict_crc = False
        self.last_nak: str = ""
        self.nak_counts: dict[str, int] = {}
        self.caps: Caps | None = None
        self.last_hb: Heartbeat | None = None
        self.proto_mismatch = False

    def feed_line(self, line: str) -> Response:
        """한 줄 수신 처리 및 프로토콜 계약 규칙 갱신."""
        clean = line.strip()
        if not clean:
            return Response(raw=line, kind="unknown")

        # 체크섬 검증 (§4)
        if "*" in clean:
            payload, _, hex_crc = clean.rpartition("*")
            expected_crc = checksum(payload)
            if hex_crc.upper() != expected_crc.upper():
                self.last_nak = "crc"
                self.nak_counts["crc"] = self.nak_counts.get("crc", 0) + 1
                return Response(raw=line, kind="nak", code="crc", args=("체크섬 불일치",))
            # 정상 체크섬을 수신하면 strict CRC로 승격 (§4)
            self.strict_crc = True
            clean = payload

        resp = parse_response(clean)
        if resp.kind == "nak":
            self.last_nak = resp.code
            self.nak_counts[resp.code] = self.nak_counts.get(resp.code, 0) + 1
        elif resp.kind == "cap" and resp.cap is not None:
            self.caps = resp.cap
            # proto 불일치 검증 (§6, §12)
            if resp.cap.proto != self.expected_proto:
                self.proto_mismatch = True
            else:
                self.proto_mismatch = False
        elif resp.kind == "hb" and resp.hb is not None:
            self.last_hb = resp.hb

        return resp


ResponseParser = ProtocolParser



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
    enc: int = 0
    vin: bool = False
    amp: bool = False
    pwm_hz: int = 0
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
            enc=_int("enc"),
            vin=_int("vin") == 1,
            amp=_int("amp") == 1,
            pwm_hz=_int("pwm_hz"),
            vmax_mms=_int("vmax"),
            vymax_mms=_int("vymax"),
            wmax_mdegs=_int("wmax"),
        )


# ----------------------------------------------------------------------
# hb — 하트비트 파싱 (보드계약 §7)
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class Heartbeat:
    """`hb <ms> rx=... st=...` 한 줄을 파싱한 것 (보드계약 §7).

    비트 0: 비상정지 래치
    비트 1: 소프트 데드맨 작동 중
    비트 2: 하드 데드맨 작동 중
    비트 3: 캘리브레이션 유효
    비트 4: 드라이버 폴트
    비트 5: 출력 포화
    비트 6: 저전압 경고
    """

    ms: int = 0
    rx: int = 0
    bad: int = 0
    i2c: int = 0
    wdt: int = 0
    st: int = 0
    tgt: tuple[int, int, int] = (0, 0, 0)
    act: tuple[int, int, int] | None = None
    vin_mv: int | None = None
    amp_ma: int | None = None

    @property
    def estop_latched(self) -> bool:
        return bool(self.st & (1 << 0))

    @property
    def soft_deadman(self) -> bool:
        return bool(self.st & (1 << 1))

    @property
    def hard_deadman(self) -> bool:
        return bool(self.st & (1 << 2))

    @property
    def calib_valid(self) -> bool:
        return bool(self.st & (1 << 3))

    @property
    def driver_fault(self) -> bool:
        return bool(self.st & (1 << 4))

    @property
    def output_saturated(self) -> bool:
        return bool(self.st & (1 << 5))

    @property
    def low_voltage(self) -> bool:
        return bool(self.st & (1 << 6))

    @staticmethod
    def parse(line: str) -> "Heartbeat":
        """`hb <ms> rx=... bad=... i2c=... wdt=... st=...` → Heartbeat."""
        if not line or not line.strip().startswith("hb"):
            raise ValueError(f"hb 줄이 아니다: {line!r}")
        tokens = line.split()
        ms = 0
        if len(tokens) >= 2 and tokens[1].isdigit():
            ms = int(tokens[1])

        fields: dict[str, str] = {}
        for token in tokens[1:]:
            if "=" in token:
                k, _, v = token.partition("=")
                fields[k] = v

        def _int(key: str, default: int = 0, base: int = 10) -> int:
            try:
                return int(fields.get(key, str(default)), base)
            except ValueError:
                return default

        def _triplet(key: str) -> tuple[int, int, int] | None:
            if key not in fields:
                return None
            parts = fields[key].split(",")
            if len(parts) == 3:
                try:
                    return (int(parts[0]), int(parts[1]), int(parts[2]))
                except ValueError:
                    return (0, 0, 0)
            return None

        st_val = 0
        if "st" in fields:
            st_str = fields["st"]
            if st_str.startswith("0x") or st_str.startswith("0X"):
                st_val = _int("st", 0, 16)
            else:
                try:
                    st_val = int(st_str, 16)
                except ValueError:
                    st_val = 0

        tgt = _triplet("tgt") or (0, 0, 0)
        act = _triplet("act")
        vin = _int("vin", -1) if "vin" in fields else None
        amp = _int("amp", -1) if "amp" in fields else None

        return Heartbeat(
            ms=ms,
            rx=_int("rx"),
            bad=_int("bad"),
            i2c=_int("i2c"),
            wdt=_int("wdt"),
            st=st_val,
            tgt=tgt,
            act=act,
            vin_mv=vin,
            amp_ma=amp,
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
# 축 부호 — 보드계약 §14.1 결정 완료 (2026-09-26)
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class AxisSigns:
    """ROS REP-103 규약(+x 앞, +y 왼쪽, +z 반시계)을 보드의 물리 부호로 옮기는 변환 계층.

    보드계약 §14.1 공식 확정:
      · ROS 기본 및 폐루프(Stm32Base, SimBase): AxisSigns(1, 1, 1) - REP-103 네이티브.
      · 현행 Uno 펌웨어 v1(UnoAdapterBase): AxisSigns(1, -1, 1) - vy=-1 (펌웨어 curVy>0 우평행 반전),
        w=1 (실제 mixing 및 실측 w=+ 반시계 보존).
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


# ----------------------------------------------------------------------
# Telemetry & MobileBase 프로토콜 — 보드계약 §11.1
# ----------------------------------------------------------------------

from typing import Any, Protocol, runtime_checkable


@dataclass(frozen=True)
class Telemetry:
    """보드 텔레메트리 스냅샷 (보드계약 §11.1).

    `hb` 하트비트 파싱 결과(Heartbeat)와 링크 품질 통계를 상위 계층에 전달한다.
    """

    ms: int = 0
    rx: int = 0
    bad: int = 0
    i2c: int = 0
    wdt: int = 0
    st: int = 0
    tgt: tuple[int, int, int] = (0, 0, 0)
    act: tuple[int, int, int] | None = None
    vin_mv: int | None = None
    amp_ma: int | None = None

    @property
    def estop_latched(self) -> bool:
        return bool(self.st & (1 << 0))

    @property
    def soft_deadman(self) -> bool:
        return bool(self.st & (1 << 1))

    @property
    def hard_deadman(self) -> bool:
        return bool(self.st & (1 << 2))

    @property
    def calib_valid(self) -> bool:
        return bool(self.st & (1 << 3))

    @property
    def driver_fault(self) -> bool:
        return bool(self.st & (1 << 4))

    @property
    def output_saturated(self) -> bool:
        return bool(self.st & (1 << 5))

    @property
    def low_voltage(self) -> bool:
        return bool(self.st & (1 << 6))

    @classmethod
    def from_heartbeat(cls, hb: Heartbeat) -> "Telemetry":
        return cls(
            ms=hb.ms,
            rx=hb.rx,
            bad=hb.bad,
            i2c=hb.i2c,
            wdt=hb.wdt,
            st=hb.st,
            tgt=hb.tgt,
            act=hb.act,
            vin_mv=hb.vin_mv,
            amp_ma=hb.amp_ma,
        )

    def to_dict(self) -> dict[str, Any]:
        """ROS2 토픽 발행 및 대시보드 연동을 위한 딕셔너리 직렬화."""
        return {
            "ms": self.ms,
            "rx": self.rx,
            "bad": self.bad,
            "i2c": self.i2c,
            "wdt": self.wdt,
            "st": self.st,
            "tgt": list(self.tgt),
            "act": list(self.act) if self.act is not None else None,
            "vin_mv": self.vin_mv,
            "amp_ma": self.amp_ma,
            "estop_latched": self.estop_latched,
            "soft_deadman": self.soft_deadman,
            "hard_deadman": self.hard_deadman,
            "calib_valid": self.calib_valid,
            "driver_fault": self.driver_fault,
            "output_saturated": self.output_saturated,
            "low_voltage": self.low_voltage,
        }


@runtime_checkable
class MobileBase(Protocol):
    """보드 계약 v2를 만족하는 이동 베이스 인터페이스 (보드계약 §11.1).

    Uno(개루프)든 STM32(폐루프)든 시뮬레이터든, 이 계약을 만족하면
    위쪽 코드를 고치지 않고 갈아끼울 수 있다.
    """

    def set_velocity(self, vx_mms: int, vy_mms: int, w_mdegs: int) -> None:
        """물리 단위 차체 속도 지령 (mm/s, mm/s, mdeg/s)."""
        ...

    def stop(self) -> None:
        """즉시 정지 (슬루 무시 S 지령)."""
        ...

    def estop(self, on: bool) -> None:
        """비상정지 래치 (X 1 / X 0)."""
        ...

    def caps(self) -> Caps:
        """보드 능력 선언 (없으면 레거시 프로파일)."""
        ...

    def telemetry(self) -> Telemetry:
        """tgt/act/vin/amp/st/링크품질 텔레메트리 스냅샷."""
        ...


@runtime_checkable
class LegacyDutyControl(Protocol):
    """선택적 레거시 duty 제어 인터페이스 (보드계약 §11.1).

    caps().units == 0인 보드나 진단 화면에서만 쓰이며,
    일반 주행 로직은 이 인터페이스를 알지 못한다.
    """

    def set_duty(self, dx: int, dy: int, dw: int) -> None:
        """duty 제어 지령."""
        ...


class MockBase:
    """계약 테스트용 무동작 베이스 구현체 (보드계약 §11.2)."""

    def __init__(self, caps: Caps | None = None, signs: AxisSigns | None = None) -> None:
        self._caps = caps or Caps.legacy()
        self._signs = signs or AxisSigns()
        self._stopped = False
        self._estopped = False
        self._last_cmd = (0, 0, 0)

    def set_velocity(self, vx_mms: int, vy_mms: int, w_mdegs: int) -> None:
        if self._estopped or (vx_mms == 0 and vy_mms == 0 and w_mdegs == 0):
            self.stop()
            return
        self._last_cmd = (vx_mms * self._signs.vx, vy_mms * self._signs.vy, w_mdegs * self._signs.w)
        self._stopped = False

    def stop(self) -> None:
        self._stopped = True
        self._last_cmd = (0, 0, 0)

    def estop(self, on: bool) -> None:
        self._estopped = on
        if on:
            self.stop()

    def caps(self) -> Caps:
        return self._caps

    def telemetry(self) -> Telemetry:
        st_val = 0x01 if self._estopped else 0x00
        if self._caps.calib:
            st_val |= 0x08
        return Telemetry(tgt=self._last_cmd, act=(0, 0, 0) if self._caps.closed_loop else None, st=st_val)


class SimBase:
    """1차 지연 + 정지마찰 물리 모델을 갖는 시뮬레이터 베이스 (보드계약 §11.2)."""

    def __init__(self, caps: Caps | None = None, ks_mms: int = 50, ks_w_mdegs: int = 15000,
                 deadman_enabled: bool = False, signs: AxisSigns | None = None) -> None:
        self._caps = caps or Caps.parse("cap proto=2 fw=3.0.0 board=sim id=SIM001 "
                                        "units=1 closed_loop=1 calib=1 vmax=800 vymax=600 wmax=180000")
        self._ks_mms = ks_mms
        self._ks_w = ks_w_mdegs
        self._signs = signs or AxisSigns()
        self._tgt = (0, 0, 0)
        self._act = (0, 0, 0)
        self._estopped = False
        self._saturated = False
        self._ms = 0
        self._deadman_enabled = deadman_enabled
        self._last_cmd_ms = 0
        self._soft_deadman = False
        self._hard_deadman = False

    def set_velocity(self, vx_mms: int, vy_mms: int, w_mdegs: int) -> None:
        if self._estopped:
            self._tgt = (0, 0, 0)
            return
        self._tgt = (vx_mms * self._signs.vx, vy_mms * self._signs.vy, w_mdegs * self._signs.w)
        self._last_cmd_ms = self._ms
        self._soft_deadman = False
        self._hard_deadman = False

    def stop(self) -> None:
        self._tgt = (0, 0, 0)
        self._act = (0, 0, 0)
        self._saturated = False
        self._soft_deadman = False
        self._hard_deadman = False

    def estop(self, on: bool) -> None:
        self._estopped = on
        if on:
            self.stop()

    def step(self, dt_sec: float = 0.05) -> None:
        """물리 시뮬레이션 한 스텝 진행 (1차 지연 + 정지마찰 문턱 + 물리 상한 클램프 + 데드맨 계층)."""
        self._ms += int(dt_sec * 1000)
        if self._estopped:
            self._act = (0, 0, 0)
            self._saturated = False
            return

        # 데드맨 계층 감시 (보드계약 §9, §12)
        # deadman_enabled=True일 때, 지령 수신 후 300ms 초과 시 소프트 데드맨(감속), 1000ms 초과 시 하드 데드맨(즉시 0)
        effective_tgt_tuple = self._tgt
        if self._deadman_enabled and (self._tgt[0] != 0 or self._tgt[1] != 0 or self._tgt[2] != 0 or self._hard_deadman):
            elapsed_ms = self._ms - self._last_cmd_ms
            if elapsed_ms >= 1000 or self._hard_deadman:
                self._hard_deadman = True
                self._soft_deadman = False
                self._act = (0, 0, 0)
                self._saturated = False
                return
            elif elapsed_ms >= 300:
                self._soft_deadman = True
                effective_tgt_tuple = (0, 0, 0)
            else:
                self._soft_deadman = False
                self._hard_deadman = False
        else:
            self._soft_deadman = False
            self._hard_deadman = False

        def _sim_axis(tgt_val: int, act_val: int, ks_val: int, limit_val: int) -> tuple[int, bool]:
            saturated = False
            effective_tgt = tgt_val
            if limit_val > 0 and abs(tgt_val) > limit_val:
                effective_tgt = int(math.copysign(limit_val, tgt_val))
                saturated = True

            if abs(effective_tgt) < ks_val and abs(effective_tgt) == 0:
                # 감속 정지
                if abs(act_val) <= 2:
                    return 0, saturated
                return int(act_val * 0.7), saturated

            if abs(effective_tgt) < ks_val:
                # 정지마찰 문턱 미만이면 물리적으로 0
                return int(act_val * 0.5), saturated

            # 1차 지연 필터
            diff = effective_tgt - act_val
            if abs(diff) <= 2:
                next_act = effective_tgt
            else:
                alpha = min(1.0, dt_sec / 0.15)
                delta = alpha * diff
                if abs(delta) < 1.0:
                    delta = math.copysign(1.0, diff)
                next_act = int(round(act_val + delta))

            if limit_val > 0 and abs(next_act) >= limit_val:
                next_act = int(math.copysign(limit_val, next_act))
                saturated = True
            return next_act, saturated

        act_x, sat_x = _sim_axis(effective_tgt_tuple[0], self._act[0], self._ks_mms, self._caps.vmax_mms)
        act_y, sat_y = _sim_axis(effective_tgt_tuple[1], self._act[1], self._ks_mms, self._caps.vymax_mms)
        act_w, sat_w = _sim_axis(effective_tgt_tuple[2], self._act[2], self._ks_w, self._caps.wmax_mdegs)
        self._act = (act_x, act_y, act_w)
        self._saturated = sat_x or sat_y or sat_w

    def caps(self) -> Caps:
        return self._caps

    def telemetry(self) -> Telemetry:
        st_val = 0x01 if self._estopped else 0x00
        if self._soft_deadman:
            st_val |= 0x02
        if self._hard_deadman:
            st_val |= 0x04
        if self._caps.calib:
            st_val |= 0x08
        if getattr(self, "_saturated", False):
            st_val |= 0x20
        return Telemetry(ms=self._ms, tgt=self._tgt, act=self._act, st=st_val, vin_mv=12600, amp_ma=450)


class UnoAdapterBase:
    """현행 Arduino Uno + 캘리브레이션 테이블 기반 어댑터 베이스 (보드계약 §11.2, §13 단계 1).

    개루프 Uno 모터보드에 캘리브레이션 테이블(DutyCalib)을 결합하여,
    상위 FSM 및 노드가 요구하는 MobileBase 물리 단위 인터페이스(mm/s, mdeg/s)를
    제공한다. 또한 진단 및 레거시 화면 호환을 위해 LegacyDutyControl(set_duty)도 구현한다.
    """

    def __init__(self, motor_link: Any = None, calib: DutyCalib | None = None,
                 signs: AxisSigns | None = None) -> None:
        self._link = motor_link
        self._calib = calib or DutyCalib()
        # 보드계약 v2 §14.1: Uno 펌웨어 v1 물리 mixing에 따라 vy=-1 (우평행+ -> 좌평행+ 반전), w=1 (반시계 보존)
        self._signs = signs or AxisSigns(vx=1, vy=-1, w=1)
        self._caps = Caps.legacy()
        self._tgt = (0, 0, 0)
        self._last_duty = (0, 0, 0)
        self._estopped = False
        self._stopped = False

    def set_velocity(self, vx_mms: int, vy_mms: int, w_mdegs: int) -> None:
        if self._estopped:
            self.stop()
            return
        cmd = plan(vx_mms / 1000.0, vy_mms / 1000.0, math.radians(w_mdegs / 1000.0),
                   caps=self._caps, calib=self._calib, signs=self._signs, estop=self._estopped)
        self._last_cmd = cmd
        self._tgt = cmd.physical
        if cmd.rejected or cmd.payload == "S" or cmd.duty is None:
            self.stop()
            return
        self._stopped = False
        self._last_duty = cmd.duty
        if self._link is not None:
            self._link.set_velocity(*cmd.duty)

    def set_duty(self, dx: int, dy: int, dw: int) -> None:
        """LegacyDutyControl 프로토콜 지원 (선택적 레거시 duty 제어)."""
        if self._estopped:
            return
        if dx == 0 and dy == 0 and dw == 0:
            self.stop()
            return
        self._last_duty = (dx, dy, dw)
        self._stopped = False
        if self._link is not None:
            self._link.set_velocity(dx, dy, dw)

    def stop(self) -> None:
        self._tgt = (0, 0, 0)
        self._last_duty = (0, 0, 0)
        self._stopped = True
        if self._link is not None:
            self._link.stop()

    def estop(self, on: bool) -> None:
        self._estopped = on
        if on:
            self.stop()

    def close(self) -> None:
        """하위 링크 자원 해제."""
        self.stop()
        if self._link is not None and hasattr(self._link, "close"):
            try:
                self._link.close()
            except Exception:  # noqa: BLE001
                pass

    def caps(self) -> Caps:
        return self._caps

    def telemetry(self) -> Telemetry:
        st_val = 0x01 if self._estopped else 0x00
        if self._calib.measured:
            st_val |= 0x08
        if self._link is not None and hasattr(self._link, "stats"):
            st = self._link.stats()
            # Uno 하트비트 st 및 링크 통계에서 Telemetry 합성
            return Telemetry(
                ms=st.get("fw_ms", 0),
                rx=st.get("fw_rx", 0),
                bad=st.get("fw_bad", 0),
                i2c=st.get("i2c_err", 0),
                wdt=st.get("wdt_near", 0),
                st=st_val,
                tgt=self._tgt,
                act=None,  # Uno 개루프는 act 속도 센서 없음
                vin_mv=None,
                amp_ma=None,
            )
        return Telemetry(tgt=self._tgt, act=None, st=st_val)


class Stm32Base:
    """STM32 기반 폐루프 모터보드 구현체 (보드계약 §11.2, §12, §13 단계 4).

    보드 계약 v2 프로토콜을 온전히 준수하는 하드웨어 모터 링크 연동 구현체:
      - 물리 속도 지령 `C <vx> <vy> <w>` (mm/s, mdeg/s) 전송.
      - 즉시 정지 `S` (슬루 무시) 및 비상정지 래치 `X 1` / `X 0` 지원.
      - `ProtocolParser`와 연동하여 보드의 `cap`, `hb`, `ok`, `nak`, `boot` 프레임 처리.
      - 텔레메트리 `Telemetry`를 통해 실측 속도(`act`), 목표치(`tgt`), 전압/전류 및 상태 비트(`st`) 제공.
    """

    def __init__(self, motor_link: Any = None, caps: Caps | None = None,
                 expected_proto: int = 2, signs: AxisSigns | None = None) -> None:
        self._link = motor_link
        self._caps = caps or Caps(
            proto=2,
            board="stm32-closed-loop",
            fw="4.0.0",
            board_id="STM32-001",
            units=True,
            closed_loop=True,
            calib=True,
            estop_hw=True,
            enc=4,
            vin=True,
            amp=True,
            pwm_hz=20000,
            vmax_mms=800,
            vymax_mms=600,
            wmax_mdegs=180000,
        )
        self._parser = ProtocolParser(expected_proto=expected_proto)
        self._signs = signs or AxisSigns()
        self._tgt = (0, 0, 0)
        self._act: tuple[int, int, int] | None = (0, 0, 0)
        self._estopped = False
        self._stopped = True
        self._last_cmd: Command | None = None

    def feed_line(self, line: str) -> Response:
        """하위 링크나 시리얼 수신 한 줄을 프로토콜 파서에 공급하고 내부 상태 갱신."""
        resp = self._parser.feed_line(line)
        if resp.kind == "cap" and resp.cap is not None:
            self._caps = resp.cap
        elif resp.kind == "hb" and resp.hb is not None:
            self._tgt = resp.hb.tgt
            if resp.hb.act is not None:
                self._act = resp.hb.act
            if resp.hb.estop_latched != self._estopped:
                self._estopped = resp.hb.estop_latched
            self._stopped = (self._tgt == (0, 0, 0))
        elif resp.kind == "ok" and resp.cmd == "X":
            if resp.args and resp.args[0] == "1":
                self._estopped = True
            elif resp.args and resp.args[0] == "0":
                self._estopped = False
        elif resp.kind == "nak" and resp.code == "estop":
            self._estopped = True
            self.stop()
        return resp

    def set_velocity(self, vx_mms: int, vy_mms: int, w_mdegs: int) -> None:
        """물리 단위 속도 지령 (mm/s, mdeg/s) 전송 (보드계약 §2, §5.1, §12)."""
        cmd = plan(vx_mms / 1000.0, vy_mms / 1000.0, math.radians(w_mdegs / 1000.0),
                   caps=self._caps, signs=self._signs, estop=self._estopped)
        self._last_cmd = cmd
        self._tgt = cmd.physical

        if self._estopped or cmd.rejected or cmd.payload == "S" or (vx_mms == 0 and vy_mms == 0 and w_mdegs == 0):
            self.stop()
            return

        self._stopped = False
        if self._link is not None:
            if hasattr(self._link, "send_raw"):
                self._link.send_raw(cmd.payload)
            elif hasattr(self._link, "write"):
                self._link.write(framed(cmd.payload))

    def stop(self) -> None:
        """즉시 정지 (슬루 무시 S 지령 전송, 보드계약 §10.2, §12)."""
        self._tgt = (0, 0, 0)
        self._stopped = True
        if self._link is not None:
            if hasattr(self._link, "stop"):
                self._link.stop()
            elif hasattr(self._link, "send_raw"):
                self._link.send_raw("S")
            elif hasattr(self._link, "write"):
                self._link.write(framed("S"))

    def estop(self, on: bool) -> None:
        """비상정지 래치 (X 1 / X 0, 보드계약 §5.1, §9, §12)."""
        self._estopped = on
        payload = "X 1" if on else "X 0"
        if on:
            self._tgt = (0, 0, 0)
            self._stopped = True
        if self._link is not None:
            if hasattr(self._link, "send_raw"):
                self._link.send_raw(payload)
            elif hasattr(self._link, "write"):
                self._link.write(framed(payload))

    def close(self) -> None:
        """안전 정지 및 링크 리소스 정리."""
        self.stop()
        if self._link is not None and hasattr(self._link, "close"):
            try:
                self._link.close()
            except Exception:  # noqa: BLE001
                pass

    def caps(self) -> Caps:
        return self._caps

    def telemetry(self) -> Telemetry:
        if self._parser.last_hb is not None:
            hb = self._parser.last_hb
            st_val = hb.st
            if self._estopped:
                st_val |= 0x01
            else:
                st_val &= ~0x01
            return Telemetry(
                ms=hb.ms,
                rx=hb.rx,
                bad=hb.bad,
                i2c=hb.i2c,
                wdt=hb.wdt,
                st=st_val,
                tgt=self._tgt,
                act=hb.act if hb.act is not None else self._act,
                vin_mv=hb.vin_mv,
                amp_ma=hb.amp_ma,
            )

        st_val = 0x01 if self._estopped else 0x00
        if self._caps.calib:
            st_val |= 0x08
        return Telemetry(tgt=self._tgt, act=self._act, st=st_val, vin_mv=12600, amp_ma=400)


__all__ = ["AxisSigns", "Caps", "Command", "DutyCalib", "Heartbeat", "LegacyDutyControl",
           "MockBase", "MobileBase", "ProtocolParser", "Response", "ResponseParser", "SimBase", "Stm32Base", "Telemetry",
           "UnoAdapterBase", "checksum", "framed", "parse_response", "plan",
           "to_physical", "EPS_MMS", "EPS_MDEGS"]



