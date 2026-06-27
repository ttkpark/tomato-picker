"""젯슨(Orin) ↔ Arduino Uno 시리얼로 메카넘 베이스를 구동하는 실물 구현.

설계: 실시간 PWM/엔코더 타이밍은 Arduino가 전담하고, 젯슨은 "목표 거리"만
한 줄 명령으로 던진 뒤 도착 신호를 기다린다. 그래서 base.py의 블로킹
시맨틱(drive_to가 도착할 때까지 반환 안 함)이 자연스럽게 맞는다.

프로토콜(텍스트, 줄단위, 보드레이트 config.BASE_SERIAL_BAUD):
  젯슨 → Arduino:  "G <ticks>\n"   목표 엔코더 틱만큼 직선 이동 (부호 = 방향)
  Arduino → 젯슨:  "DONE\n"        도착해 정지 완료
  젯슨 → Arduino:  "P\n"  → "OK\n" 연결 점검(핑)
  젯슨 → Arduino:  "S\n"           즉시 정지(비상)

대응 펌웨어: firmware/mecanum_serial/mecanum_serial.ino
"""

from __future__ import annotations

import time

import serial  # pyserial

from ..config import BASE_SERIAL_BAUD, BASE_SERIAL_PORT, BASE_TICKS_PER_CM
from .base import MobileBase


class JetsonBase(MobileBase):
    """USB 시리얼로 Arduino Uno를 제어하는 메카넘 베이스."""

    def __init__(
        self,
        port: str = BASE_SERIAL_PORT,
        baud: int = BASE_SERIAL_BAUD,
        ticks_per_cm: float = BASE_TICKS_PER_CM,
        move_timeout: float = 30.0,
    ) -> None:
        self._ticks_per_cm = ticks_per_cm
        self._move_timeout = move_timeout
        self.position = 0.0  # 기준점 기준 현재 거리(cm). MockBase와 동일 의미.

        # Uno는 USB 연결 시 자동 리셋되어 부트로더가 ~1.5s 잡아먹는다. 잠깐 대기.
        self._ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2.0)
        self._ser.reset_input_buffer()

    def ping(self) -> bool:
        """연결 점검. Arduino가 'OK'로 답하면 True."""
        self._send("P")
        return self._read_line() == "OK"

    def drive_to(self, distance: float) -> None:
        """기준점에서 distance(cm) 위치까지 직선 이동(블로킹)."""
        delta_cm = distance - self.position
        ticks = round(delta_cm * self._ticks_per_cm)
        if ticks != 0:
            self._send(f"G {ticks}")
            self._wait_for("DONE", self._move_timeout)
        self.position = distance

    def stop(self) -> None:
        """비상 정지."""
        self._send("S")

    def close(self) -> None:
        self._ser.close()

    # --- 시리얼 입출력 ---

    def _send(self, line: str) -> None:
        self._ser.write((line + "\n").encode("ascii"))
        self._ser.flush()

    def _read_line(self) -> str:
        return self._ser.readline().decode("ascii", errors="replace").strip()

    def _wait_for(self, token: str, timeout: float) -> None:
        """timeout 안에 token 줄이 올 때까지 대기. 안 오면 정지시키고 예외."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._read_line()
            if line == token:
                return
            # 빈 줄(readline 타임아웃)이나 디버그 줄은 무시하고 계속 기다린다.
        self.stop()
        raise TimeoutError(f"Arduino 응답 '{token}' 대기 {timeout}s 초과")
