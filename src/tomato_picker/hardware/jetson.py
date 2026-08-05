"""젯슨(Orin) ↔ Arduino Uno 시리얼로 메카넘 베이스를 구동하는 실물 구현.

프로토콜은 firmware/mecanum_stable/mecanum_stable.ino와 반드시 일치해야
한다. 이 펌웨어는 PS2 없이 젯슨의 시리얼 속도 지령만으로 구동되고, PCA9685
(I2C)로 4륜을 돌린다(2026-07-08 확인):

  젯슨 → "V <vx> <vy> <w>\n"  속도 지령(-255..255, 부호=방향). 데드맨 400ms
                              — 이 시간 안에 새 명령이 안 오면 자동 정지.
  젯슨 → "S\n"                즉시 정지
  Arduino → "ok ...\n"        명령 처리 확인
  Arduino → "hb <ms>\n"       1초 주기 하트비트

⚠ 예전 버전은 firmware/mecanum_serial.ino의 "G <ticks>"/"DONE" 프로토콜을
가정했는데, 실제 보드에 그 펌웨어가 올라간 적이 없어 처음부터 동작하지
않는 코드였다. drive_to()의 거리 기반 이동은 이 펌웨어가 엔코더 피드백을
안 줘서(오픈루프) 아직 캘리브레이션되지 않았다 — 필요해지면 시간×속도
근사치로 구현할 것. 지금 당장 쓸 수 있는 건 drive_forward/drive_backward(seconds)뿐.

⚠ **연결을 오래 열어두고 재사용하면 안 된다** — 음성 서비스가 시작 시
한 번 연결해 계속 재사용하는 방식으로는 실기 테스트에서 명령은 "성공"
응답이 오는데도 바퀴가 실제로 안 움직이는 현상이 반복 재현됐다(2026-07-08,
CH340 장시간 연결 안정성 이슈로 추정 — moebius-mecanum-hardware 메모의
"USB 연결 불안정" 항목과 같은 계열). 반면 매번 새로 연결(DTR 리셋 포함)해서
쓰고 닫는 방식은 여러 차례 반복해도 한 번도 실패하지 않았다. 그래서
drive_forward/drive_backward가 매 호출마다 새로 연결한다 — 느리지만(호출당
~2초 추가) 확실하다.
"""

from __future__ import annotations

import threading
import time

import serial  # pyserial

from ..config import (
    BASE_DRIVE_RAMP_SEC,
    BASE_DRIVE_RAMP_START_FRACTION,
    BASE_DRIVE_RESEND_INTERVAL_SEC,
    BASE_DRIVE_SPEED,
    BASE_SERIAL_BAUD,
    BASE_SERIAL_PORT,
)
from .base import MobileBase
from .ports import resolve_base_port


class JetsonBase(MobileBase):
    """PCA9685 기반 메카넘 베이스 — mecanum_stable.ino의 V/S 속도 프로토콜.

    호출마다 새로 연결한다(위 모듈 docstring 참고) — 인스턴스 생성 자체는
    포트를 열지 않으므로 가볍고, 하드웨어가 실제로 없어도 생성자는 안 죽는다.
    """

    def __init__(self, port: str = BASE_SERIAL_PORT, baud: int = BASE_SERIAL_BAUD) -> None:
        self.position = 0.0  # MobileBase 인터페이스 호환용 — 실제 위치추적 없음(오픈루프).
        self._port_fallback = port
        self._baud = baud
        # 음성 인텐트 워커와 대시보드 수동조작이 동시에 포트를 열면 둘 다 깨진다.
        self._lock = threading.Lock()
        # --- 홀드 주행(키보드 "누르는 동안 계속") 상태 ---
        self._hold_lock = threading.Lock()
        self._hold_target: tuple[int, int, int] = (0, 0, 0)
        self._hold_ts = 0.0
        self._hold_thread: threading.Thread | None = None

    @property
    def _port(self) -> str:
        """호출 시점에 실제 존재하는 경로를 다시 찾는다 — CH340이 끊겼다 붙으면
        ttyUSB0→ttyUSB1로 번호가 바뀔 수 있다(팔에서 실제로 겪은 문제)."""
        return resolve_base_port(self._port_fallback)

    def drive_to(self, distance: float) -> None:
        raise NotImplementedError(
            "drive_to는 이 펌웨어(mecanum_stable, 오픈루프)에서 아직 캘리브레이션되지 "
            "않았다. drive_forward(seconds)를 대신 쓸 것."
        )

    def drive_forward(self, seconds: float, speed: int = BASE_DRIVE_SPEED) -> None:
        """seconds초 동안 전진(vx=+speed)한 뒤 정지(블로킹). 매번 새로 연결."""
        self.drive(seconds, vx=speed)

    def drive_backward(self, seconds: float, speed: int = BASE_DRIVE_SPEED) -> None:
        """seconds초 동안 후진(vx=-speed)한 뒤 정지(블로킹). 매번 새로 연결."""
        self.drive(seconds, vx=-speed)

    def drive(self, seconds: float, vx: int = 0, vy: int = 0, w: int = 0) -> None:
        """메카넘 3자유도 지령을 seconds초 동안 유지한 뒤 정지(블로킹).

        vx=전후, vy=좌우 게걸음, w=제자리 회전. 각각 -255..255.
        대시보드 수동조작 화면이 이걸 직접 쓴다(전진 전용 API로는 게걸음·회전을
        못 시켜서 추가).
        """
        self._pulse(seconds, vx, vy, w)

    def stop(self) -> None:
        """비상 정지 — 새로 연결해서 즉시 S를 보낸다(연결에 ~2초 걸림).

        참고: 펌웨어 데드맨이 400ms라 명령이 끊기면 어차피 자동 정지한다.
        이건 '지금 확실히 멈춰라'를 명시적으로 보내는 경로.
        """
        with self._lock:
            ser = serial.Serial(self._port, self._baud, timeout=1.0)
            try:
                time.sleep(2.0)
                ser.write(b"S\n")
                ser.flush()
            finally:
                ser.close()

    def close(self) -> None:
        pass  # 상시 연결을 유지하지 않으므로 정리할 게 없다.

    # --- 홀드 주행: 키를 누르고 있는 동안 계속 굴린다 ---
    #
    # 왜 drive(seconds)로는 안 되나: 이 클래스는 호출마다 포트를 새로 열고
    # Uno의 DTR 리셋 때문에 2초를 기다린다. 키 하나에 2초씩 붙으면 연속
    # 조작이 불가능하다. 그래서 홀드 모드에서는 연결을 **열어둔 채** 백그라운드
    # 스레드가 100ms마다 지령을 재전송하고, 브라우저가 갱신을 멈추면
    # (=키를 뗐거나 페이지가 죽었으면) 즉시 S를 보낸다.
    #
    # 안전: 브라우저 갱신이 HOLD_STALE_SEC 넘게 끊기면 정지 → 펌웨어 데드맨
    # (400ms)과 이중 안전장치. 정지 후 HOLD_IDLE_CLOSE_SEC 동안 조용하면
    # 포트를 닫아, 평소엔 기존의 "매번 새로 연결" 방식이 그대로 유지된다.
    HOLD_STALE_SEC = 0.5
    HOLD_IDLE_CLOSE_SEC = 3.0
    # 데드맨(400ms) 대비 8배 여유 — 부하 걸린 젯슨에서 스레드가 밀려도
    # 주행이 끊기지 않게(BASE_DRIVE_RESEND_INTERVAL_SEC 주석 참고).
    HOLD_SEND_INTERVAL_SEC = 0.05

    def hold(self, vx: int = 0, vy: int = 0, w: int = 0) -> None:
        """지금부터 이 속도로 계속 가라(논블로킹). (0,0,0)이면 정지."""
        with self._hold_lock:
            self._hold_target = (vx, vy, w)
            self._hold_ts = time.monotonic()
            alive = self._hold_thread is not None and self._hold_thread.is_alive()
            if not alive:
                self._hold_thread = threading.Thread(target=self._hold_loop, daemon=True)
                self._hold_thread.start()

    def _hold_loop(self) -> None:
        with self._lock:  # 블로킹 drive()와 포트를 다투지 않게
            ser = serial.Serial(self._port, self._baud, timeout=1.0)
            try:
                time.sleep(2.0)  # Uno DTR 리셋 대기 — 세션당 딱 한 번
                ser.reset_input_buffer()
                moving_since: float | None = None
                idle_since = time.monotonic()
                while True:
                    now = time.monotonic()
                    with self._hold_lock:
                        target, ts = self._hold_target, self._hold_ts
                    fresh = (now - ts) < self.HOLD_STALE_SEC
                    if fresh and any(target):
                        if moving_since is None:
                            moving_since = now  # 이 순간부터 소프트 스타트 램프
                        elapsed = now - moving_since
                        vx, vy, w = target
                        ser.write(
                            f"V {self._ramped(vx, elapsed)} {self._ramped(vy, elapsed)} "
                            f"{self._ramped(w, elapsed)}\n".encode("ascii")
                        )
                        ser.flush()
                        idle_since = now
                    else:
                        moving_since = None
                        ser.write(b"S\n")
                        ser.flush()
                        if now - idle_since > self.HOLD_IDLE_CLOSE_SEC:
                            break
                    time.sleep(self.HOLD_SEND_INTERVAL_SEC)
            finally:
                try:
                    ser.write(b"S\n")
                    ser.flush()
                finally:
                    ser.close()

    def _pulse(self, seconds: float, vx: int, vy: int = 0, w: int = 0) -> None:
        """새 연결을 열고, 데드맨(400ms)보다 짧은 주기로 vx를 재전송하다 정지 후 닫는다.

        처음 BASE_DRIVE_RAMP_SEC 동안은 소프트 스타트 — 정지 상태에서 목표
        속도를 한 번에 때리면 4륜 인러시가 겹쳐 12V 레일이 무너지고 젯슨이
        리셋된 이력이 있다([[battery-power-system]]). 램프를 걸면 최고속도는
        그대로 쓰면서 기동 피크만 낮출 수 있다.
        """
        with self._lock:
            ser = serial.Serial(self._port, self._baud, timeout=1.0)
            try:
                time.sleep(2.0)  # Uno가 DTR로 리셋되어 부트로더+setup()이 끝나길 대기
                ser.reset_input_buffer()
                start = time.monotonic()
                deadline = start + seconds
                while True:
                    now = time.monotonic()
                    if now >= deadline:
                        break
                    elapsed = now - start
                    cmd = (
                        f"V {self._ramped(vx, elapsed)} "
                        f"{self._ramped(vy, elapsed)} {self._ramped(w, elapsed)}\n"
                    )
                    ser.write(cmd.encode("ascii"))
                    ser.flush()
                    time.sleep(BASE_DRIVE_RESEND_INTERVAL_SEC)
                ser.write(b"S\n")
                ser.flush()
            finally:
                ser.close()

    @staticmethod
    def _ramped(vx: int, elapsed: float) -> int:
        """기동 후 elapsed초 시점에 보낼 속도. 램프 구간이 끝나면 vx 그대로."""
        if BASE_DRIVE_RAMP_SEC <= 0 or elapsed >= BASE_DRIVE_RAMP_SEC:
            return vx
        f = BASE_DRIVE_RAMP_START_FRACTION
        scale = f + (1.0 - f) * (elapsed / BASE_DRIVE_RAMP_SEC)
        return int(vx * scale)
