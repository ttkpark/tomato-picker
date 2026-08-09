"""메카넘 모터보드(Arduino Uno + PCA9685)와의 **상시 시리얼 링크**.

firmware/mecanum_stable/mecanum_stable.ino **v2** 프로토콜 전용.

왜 새로 만들었나 — 기존 JetsonBase의 "매 호출마다 새로 연결" 방식의 대가:
  · 호출 하나에 2초(Uno DTR 리셋 대기)가 붙어 연속 조작이 사실상 불가능했다.
  · 그 2초 동안 포트 락을 잡고 있어 음성/대시보드/게임패드가 서로를 막았다.
  · 지령 재전송이 파이썬 루프 안에 섞여 있어, 젯슨이 바쁘면(Whisper·YOLO)
    400ms 데드맨을 넘겨 펌웨어가 멈췄다 다시 출발 → "주웅 주웅 주우웅".

과거에 상시 연결을 포기했던 이유(2026-07-08: 오래 열어두면 "ok"는 오는데
바퀴가 안 도는 현상)는 **DTR/RTS 상태와 응답 폭주**가 실제 원인으로 보인다.
그래서 이 구현은 그 셋을 정면으로 막는다:
  1. **DTR/RTS를 내린 채로 연다** — Uno가 리셋되지 않으니 2초 대기가 사라지고,
     USB 재열거 중 DTR 토글로 보드가 재부팅되는 일도 없다.
  2. **전용 송신 스레드**가 정확히 20ms(50Hz) 주기로 지령을 재전송한다 —
     파이썬 GIL이 밀려도 이 스레드 하나만 살아 있으면 데드맨에 안 걸린다.
  3. **체크섬(XOR)** 을 붙인다 — 깨진 "V 255 0 0"이 실행되는 사고를 펌웨어가 거부한다.
  4. **하트비트 감시** — 1초마다 오는 `hb ... rx= bad=`로 링크가 살아 있는지,
     프레임이 깨지고 있는지를 숫자로 안다(대시보드에 그대로 표시).
  5. **자동 재연결** — 쓰기 실패 시 포트를 다시 찾아 연다(CH340이 끊겼다 붙으면
     ttyUSB0→ttyUSB1로 번호가 바뀌므로 매번 resolve).

가감속(슬루)은 **펌웨어가** 한다(v2). 파이썬이 램프를 만들면 지연·지터가 그대로
전류 스파이크가 되지만, 5ms 틱을 도는 AVR이 하면 매끄럽다.
"""

from __future__ import annotations

import threading
import time

import serial  # pyserial

from ..config import BASE_SERIAL_BAUD, BASE_SERIAL_PORT
from .ports import resolve_base_port


def _framed(payload: str) -> bytes:
    """페이로드에 XOR 체크섬을 붙여 한 줄로 만든다 — 펌웨어 v2의 "*HH" 형식."""
    crc = 0
    for ch in payload:
        crc ^= ord(ch)
    return f"{payload}*{crc:02X}\n".encode("ascii")


class MotorLink:
    """모터보드와의 상시 링크. 지령은 논블로킹, 재전송은 백그라운드 스레드가 담당."""

    # 송신 주기. 펌웨어 소프트 데드맨(300ms)의 15배 여유 — 젯슨이 아무리 밀려도
    # 이 정도면 한 번은 통과한다. 115200baud에 ~20바이트라 대역폭 부담은 없다.
    SEND_INTERVAL_SEC = 0.02
    # 정지 상태에서는 굳이 50Hz로 떠들 필요가 없다(링크 유지용 최소 주기).
    IDLE_INTERVAL_SEC = 0.1
    # 상위(브라우저/게임패드)가 이 시간 넘게 갱신을 안 하면 지령을 0으로 본다.
    # 펌웨어 데드맨과 이중 안전장치 — 브라우저 탭이 죽어도 로봇이 선다.
    STALE_SEC = 0.5
    # 재연결 시도 간격(포트가 아직 안 붙었을 때 과하게 두드리지 않게).
    RECONNECT_SEC = 1.0
    # 하트비트가 이만큼 끊기면 **포트가 열려 있어도** 보드가 죽은 것으로 보고
    # 링크를 다시 연다. 2026-08-09 실사고: USB는 멀쩡히 붙어 있는데(ch341 disconnect
    # 로그도 없다) 보드가 조용해졌고, 젯슨은 14분 동안 아무도 안 듣는 포트에 V를
    # 계속 써 넣고 있었다 — 대시보드엔 "연결됨"으로 뜨고 바퀴만 안 움직였다.
    # 포트를 닫았다 다시 열면 커널 DTR 토글로 Uno가 리셋되는데(_open 주석 참고),
    # 그게 멈춘 보드를 원격에서 살리는 유일한 수단이다. 펌웨어 hb는 1Hz라
    # 5초 침묵은 확실한 사망 신호(부트로더 ~2초보다도 넉넉하다).
    HB_DEAD_SEC = 5.0

    def __init__(
        self,
        port: str = BASE_SERIAL_PORT,
        baud: int = BASE_SERIAL_BAUD,
        on_connect=None,
    ) -> None:
        # on_connect: 링크가 (다시) 열린 직후 이 스레드에서 호출된다. 보드가
        # 리셋되면 F/P/R 설정이 날아가므로 상위가 여기서 되밀어넣는다 —
        # 주행 명령을 기다렸다 적용하면 "첫 주행만 예전 소리"가 난다.
        self._on_connect = on_connect
        self._port_fallback = port
        self._baud = baud
        self._lock = threading.Lock()
        self._target: tuple[int, int, int] = (0, 0, 0)
        self._target_ts = 0.0
        self._stop_flag = False       # 다음 송신에서 S를 한 번 보낸다
        self._closing = False
        self._ser: serial.Serial | None = None
        # 링크 상태(대시보드 표시용)
        self._connected = False
        self._port_now: str | None = None
        self._last_hb = 0.0
        self._opened_at = 0.0     # 이번 연결을 연 시각(첫 하트비트 대기 기준)
        self._hb_resets = 0       # 하트비트 끊김으로 링크를 되살린 횟수
        self._rx_ok = 0
        self._rx_bad = 0
        self._nak = 0
        self._acks: list[str] = []   # 펌웨어 "ok ..." 확인 응답(최근 4개)
        self._pending_on_connect = False  # 첫 하트비트를 기다리는 중인가
        self._rx_buf = b""               # 줄 중간에서 끊긴 수신 조각
        self._last_error: str | None = None
        # 연결이 새로 열릴 때마다 증가. 보드가 리셋되면 P/F/R 설정이 기본값으로
        # 돌아가므로, 상위(JetsonBase)가 이 값을 보고 튜닝을 다시 밀어넣는다.
        self._generation = 0
        self._thread = threading.Thread(target=self._run, daemon=True, name="motor-link")
        self._thread.start()

    # --- 공개 API (전부 논블로킹) ---

    def set_velocity(self, vx: int = 0, vy: int = 0, w: int = 0) -> None:
        """지금부터 이 속도로 가라. STALE_SEC 안에 다시 부르지 않으면 자동으로 선다."""
        with self._lock:
            self._target = (_clamp(vx), _clamp(vy), _clamp(w))
            self._target_ts = time.monotonic()

    def stop(self) -> None:
        """즉시 정지 — 목표를 0으로 두고 펌웨어에 하드 스톱(S)을 한 번 보낸다."""
        with self._lock:
            self._target = (0, 0, 0)
            self._target_ts = time.monotonic()
            self._stop_flag = True

    def send_raw(self, payload: str) -> bool:
        """설정용 단발 명령(P/L/R/F/T/?). 성공 여부를 돌려준다."""
        ser = self._ser
        if ser is None:
            return False
        try:
            ser.write(_framed(payload))
            ser.flush()
            return True
        except Exception as exc:  # noqa: BLE001 - 링크 오류는 재연결 루프가 처리
            self._fail(exc)
            return False

    def stats(self) -> dict:
        """링크 품질 스냅샷 — 대시보드 배지/디버깅용."""
        age = time.monotonic() - self._last_hb if self._last_hb else None
        return {
            # ⚠ 여기 "connected"는 **하트비트까지 본** 판정이다(connected 프로퍼티).
            # 예전엔 포트 열림 플래그를 그대로 내보내서, 보드가 죽어도 대시보드엔
            # 파란 "연결됨"이 떠 있었다 — 바퀴가 왜 안 도는지 화면만 봐선 몰랐다.
            "connected": self.connected,
            "port_open": self._connected,   # 포트 자체가 열려 있는지(진단용)
            "port": self._port_now,
            "hb_age": round(age, 2) if age is not None else None,
            "fw_rx": self._rx_ok,
            "fw_bad": self._rx_bad,
            "nak": self._nak,
            "hb_resets": self._hb_resets,
            "acks": list(self._acks),
            "error": self._last_error,
            "target": self._target,
        }

    @property
    def generation(self) -> int:
        """연결 세대. 값이 바뀌었으면 보드가 새로 붙은(=설정이 날아간) 것."""
        return self._generation

    @property
    def connected(self) -> bool:
        # 하트비트가 3초 넘게 없으면 포트는 열려 있어도 보드가 죽은 것으로 본다.
        return self._connected and (not self._last_hb or time.monotonic() - self._last_hb < 3.0)

    def close(self) -> None:
        self._closing = True
        self._thread.join(timeout=2.0)
        self._shutdown_port()

    # --- 내부 ---

    def _open(self) -> bool:
        port = resolve_base_port(self._port_fallback)
        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = self._baud
            ser.timeout = 0
            ser.write_timeout = 0.5
            # ★ 핵심: 열기 전에 DTR/RTS를 내려둔다. Uno는 DTR 하강엣지에서
            #   리셋되므로, 이걸 안 하면 매 연결마다 부트로더 2초를 기다려야 하고
            #   USB가 잠깐 흔들릴 때마다 보드가 재부팅된다.
            ser.dtr = False
            ser.rts = False
            # 이 포트를 독점한다. 예전(호출마다 열고 닫기)에는 게임패드 서비스
            # (controller_drive.py)와 번갈아 쓰는 게 우연히 굴러갔지만, 이제는
            # 상시 점유하므로 두 프로세스가 동시에 쓰면 지령이 섞여 깨진다.
            # 독점하면 나중에 붙는 쪽이 "Device or resource busy"로 **명확히**
            # 실패한다 — 조용히 이상하게 도는 것보다 낫다(대시보드에 그대로 표시).
            ser.exclusive = True
            ser.open()
            ser.reset_input_buffer()
            self._rx_buf = b""
        except Exception as exc:  # noqa: BLE001 - 아직 안 붙었을 뿐
            self._last_error = f"{port}: {exc}"
            self._connected = False
            return False
        self._ser = ser
        self._port_now = port
        self._connected = True
        self._last_error = None
        self._generation += 1
        self._opened_at = time.monotonic()
        self._last_hb = 0.0   # 새 연결 — 옛 하트비트 시각은 무효
        # 이미 돌고 있던 보드일 수도, 방금 리셋된 보드일 수도 있다 — 어느 쪽이든
        # 확실히 멈춘 상태에서 시작한다.
        try:
            ser.write(_framed("S"))
            ser.flush()
        except Exception:  # noqa: BLE001
            pass
        # ⚠ on_connect를 **여기서 바로 부르면 안 된다.** dtr/rts를 내리고 열어도
        # 커널이 포트를 열 때 DTR을 순간 토글해 Uno가 리셋된다(2026-08-06 실측:
        # 연결 직후 보드의 rx 카운터가 0부터 다시 시작했다). 그 직후 ~2초는
        # 부트로더 구간이라 이때 보낸 F/P/R은 통째로 먹히고 응답도 없다.
        # 그래서 "보드가 살아났다"는 확실한 신호 = **첫 하트비트**를 기다린다.
        # (예전 코드의 blind sleep(2.0)이 하던 일을, 기다리지 않고 이벤트로.)
        self._pending_on_connect = self._on_connect is not None
        return True

    def _shutdown_port(self) -> None:
        ser, self._ser = self._ser, None
        self._connected = False
        if ser is None:
            return
        try:
            ser.write(_framed("S"))
            ser.flush()
        except Exception:  # noqa: BLE001 - 이미 죽은 포트면 그냥 닫는다
            pass
        try:
            ser.close()
        except Exception:  # noqa: BLE001
            pass

    def _fail(self, exc: Exception) -> None:
        self._last_error = str(exc)
        self._shutdown_port()

    def _run(self) -> None:
        last_reconnect = 0.0
        while not self._closing:
            if self._ser is None:
                now = time.monotonic()
                if now - last_reconnect < self.RECONNECT_SEC:
                    time.sleep(0.05)
                    continue
                last_reconnect = now
                if not self._open():
                    continue

            moving = self._tick()
            self._drain_input()
            self._watch_heartbeat()
            time.sleep(self.SEND_INTERVAL_SEC if moving else self.IDLE_INTERVAL_SEC)

    def _watch_heartbeat(self) -> None:
        """말이 없어진 보드를 되살린다 — 포트 재개방 = Uno 리셋.

        USB가 빠지면 write가 터져서 재연결 루프가 알아서 돌지만, 보드만 조용히
        죽는 경우(전원 순간강하·펌웨어 정지)엔 포트가 멀쩡히 열려 있어 아무도
        눈치채지 못한다. 하트비트가 유일한 생존 신호이므로 그걸로 판단한다.
        """
        if self._ser is None:
            return
        # 첫 하트비트 전(부트로더 구간)에는 연 시각을 기준으로 센다.
        last = self._last_hb or self._opened_at
        if time.monotonic() - last < self.HB_DEAD_SEC:
            return
        self._hb_resets += 1
        self._last_error = (
            f"하트비트 {self.HB_DEAD_SEC:.0f}초 이상 없음 — 링크 재시작(보드 리셋) "
            f"#{self._hb_resets}"
        )
        print(f"  [base] {self._last_error}")
        # 닫고 나면 _run이 RECONNECT_SEC 뒤에 다시 열고, 그때 DTR로 보드가 선다.
        self._shutdown_port()

    def _tick(self) -> bool:
        """지령 한 번 송신. 실제로 움직이는 중이면 True(→ 빠른 주기 유지)."""
        with self._lock:
            target, ts = self._target, self._target_ts
            stop_flag, self._stop_flag = self._stop_flag, False
        fresh = (time.monotonic() - ts) < self.STALE_SEC
        vx, vy, w = target if fresh else (0, 0, 0)
        ser = self._ser
        if ser is None:
            return False
        try:
            if stop_flag:
                ser.write(_framed("S"))
            ser.write(_framed(f"V {vx} {vy} {w}"))
            ser.flush()
        except Exception as exc:  # noqa: BLE001 - 케이블이 빠졌거나 보드가 죽었다
            self._fail(exc)
            return False
        return bool(vx or vy or w)

    def _drain_input(self) -> None:
        """펌웨어 응답을 읽어 링크 상태를 갱신한다.

        읽지 않으면 커널 수신 버퍼가 차서 결국 링크가 막힌다 — 하트비트를
        1초마다 보내는 펌웨어와 사는 이상 이 배수구는 필수다.
        """
        ser = self._ser
        if ser is None:
            return
        try:
            if not ser.in_waiting:
                return
            data = ser.read(ser.in_waiting)
        except Exception as exc:  # noqa: BLE001
            self._fail(exc)
            return
        # 읽기 경계가 줄 중간에 떨어지는 건 흔하다 — 남은 조각을 다음 읽기에
        # 이어붙인다. 예전엔 그냥 버려서 하트비트/카운터를 이따금 놓쳤다.
        data, _, self._rx_buf = (self._rx_buf + data).rpartition(b"\n")
        for raw in data.split(b"\n"):
            line = raw.decode("ascii", "ignore").strip()
            if not line:
                continue
            if line.startswith("hb "):
                self._last_hb = time.monotonic()
                if self._pending_on_connect:
                    # 보드가 부팅을 마치고 말을 걸어왔다 — 이제 설정이 먹는다.
                    self._pending_on_connect = False
                    try:
                        self._on_connect(self)
                    except Exception as exc:  # noqa: BLE001 - 튜닝 실패가 링크를 죽이면 안 됨
                        self._last_error = f"on_connect: {exc}"
                for token in line.split():
                    if token.startswith("rx="):
                        self._rx_ok = _int_or(token[3:], self._rx_ok)
                    elif token.startswith("bad="):
                        self._rx_bad = _int_or(token[4:], self._rx_bad)
            elif line.startswith("ok "):
                # 설정계열(F/P/R/S) 확인 응답. 튜닝이 **실제로 보드에 먹혔는지**를
                # 화면에서 눈으로 확인할 수 있게 마지막 것들을 들고 있는다.
                if not line.startswith("ok S"):
                    self._acks.append(line)
                    del self._acks[:-4]
            elif line.startswith("nak"):
                self._nak += 1
                self._last_error = line


def _clamp(v: int) -> int:
    return max(-255, min(255, int(v)))


def _int_or(text: str, default: int) -> int:
    try:
        return int(text)
    except ValueError:
        return default
