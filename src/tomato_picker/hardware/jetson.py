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
"""

from __future__ import annotations

import time

import serial  # pyserial

from ..config import (
    BASE_DRIVE_RESEND_INTERVAL_SEC,
    BASE_DRIVE_SPEED,
    BASE_SERIAL_BAUD,
    BASE_SERIAL_PORT,
)
from .base import MobileBase


class JetsonBase(MobileBase):
    """PCA9685 기반 메카넘 베이스 — mecanum_stable.ino의 V/S 속도 프로토콜."""

    def __init__(self, port: str = BASE_SERIAL_PORT, baud: int = BASE_SERIAL_BAUD) -> None:
        self.position = 0.0  # MobileBase 인터페이스 호환용 — 실제 위치추적 없음(오픈루프).
        # Uno는 USB 연결 시(DTR) 자동 리셋되어 부트로더가 ~1.5-2s 잡아먹는다.
        self._ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2.0)
        self._ser.reset_input_buffer()

    def drive_to(self, distance: float) -> None:
        raise NotImplementedError(
            "drive_to는 이 펌웨어(mecanum_stable, 오픈루프)에서 아직 캘리브레이션되지 "
            "않았다. drive_forward(seconds)를 대신 쓸 것."
        )

    def drive_forward(self, seconds: float, speed: int = BASE_DRIVE_SPEED) -> None:
        """seconds초 동안 전진(vx=+speed)한 뒤 정지(블로킹)."""
        self._pulse(seconds, speed)

    def drive_backward(self, seconds: float, speed: int = BASE_DRIVE_SPEED) -> None:
        """seconds초 동안 후진(vx=-speed)한 뒤 정지(블로킹)."""
        self._pulse(seconds, -speed)

    def _pulse(self, seconds: float, vx: int) -> None:
        """데드맨(400ms)보다 짧은 주기로 vx를 계속 재전송하다 seconds 후 정지."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._send(f"V {vx} 0 0")
            time.sleep(BASE_DRIVE_RESEND_INTERVAL_SEC)
        self.stop()

    def stop(self) -> None:
        """비상 정지."""
        self._send("S")

    def close(self) -> None:
        self.stop()
        self._ser.close()

    def _send(self, line: str) -> None:
        self._ser.write((line + "\n").encode("ascii"))
        self._ser.flush()
