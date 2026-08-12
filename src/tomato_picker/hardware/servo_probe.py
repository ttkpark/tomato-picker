"""서보 버스가 살아 있는지 **먼저** 짧게 물어본다 (연결 전 사전 점검).

왜 필요한가 — USB 어댑터가 열리는 것과 서보가 응답하는 것은 **다른 사건**이다.
어댑터는 12V가 없어도 USB만으로 열거되므로, 팔 전원이 꺼진 채 케이블만 꽂혀
있으면 `/dev/ttyACM0`은 멀쩡히 생긴다. 그 상태로 lerobot에 넘기면 그쪽이 응답을
기다리며 **무한정 블로킹**되고, 그게 서비스 시작의 첫 단계라 바퀴·라인·마이크
초기화까지 통째로 멈춘다 — 화면엔 "장비 상태 확인 중..."만 남는다.
(2026-08-12 실사고: 프로세스가 select()에서 2분 넘게 굳어 있었다.)

여기서 **원시 SCS 프레임으로 0.3초씩** 물어보면 그 구분이 즉시 난다:
  · 응답 있음  → 서보 살아 있음. 정상 연결로 진행.
  · 무응답     → 12V 전원 또는 버스 배선 문제. **그렇게 말하고 빠르게 실패**한다.

프로토콜(Feetech SCS/STS): [0xFF 0xFF ID LEN INST CHK], PING은 LEN=2 INST=1,
체크섬은 ID+LEN+INST의 하위 8비트를 뒤집은 값. 응답은 6바이트 상태 패킷.
lerobot을 거치지 않는 이유는 그쪽 타임아웃을 우리가 제어할 수 없어서다.
"""

from __future__ import annotations

# 팔 서보 ID(SO-101은 1~6). 하나라도 답하면 버스가 살아 있는 것으로 본다.
SERVO_IDS = (1, 2, 3, 4, 5, 6)
BAUD = 1_000_000
PER_ID_TIMEOUT = 0.25


def ping_servos(port: str, ids=SERVO_IDS, timeout: float = PER_ID_TIMEOUT) -> list[int]:
    """응답한 서보 ID 목록. 포트를 못 열거나 아무도 안 답하면 빈 리스트.

    호출자는 이 결과가 비었으면 **연결을 시도하지 말 것** — 거기서 굳는다.
    """
    import serial

    alive: list[int] = []
    try:
        with serial.Serial(port, BAUD, timeout=timeout, write_timeout=timeout) as ser:
            for sid in ids:
                checksum = (~(sid + 0x02 + 0x01)) & 0xFF
                try:
                    ser.reset_input_buffer()
                    ser.write(bytes([0xFF, 0xFF, sid, 0x02, 0x01, checksum]))
                    # 응답 헤더(0xFF 0xFF)만 확인하면 충분하다 — 우리가 알고 싶은
                    # 건 "버스에 전기가 흐르고 누가 듣고 있나"뿐이다.
                    if ser.read(2) == b"\xff\xff":
                        alive.append(sid)
                        break          # 하나면 족하다. 나머지는 lerobot이 확인한다.
                except Exception:      # noqa: BLE001 - 한 ID 실패는 다음 ID로
                    continue
    except Exception:                  # noqa: BLE001 - 포트가 없거나 이미 점유됨
        return []
    return alive


def require_live_bus(port: str) -> None:
    """버스가 죽어 있으면 **원인을 말하며** 즉시 실패한다(블로킹 대신).

    메시지는 화면에 그대로 뜨므로, 다음에 무엇을 할지까지 적는다.
    """
    if ping_servos(port):
        return
    raise RuntimeError(
        f"서보가 응답하지 않습니다({port}는 열림) — 팔 드라이버 보드의 "
        "**12V 전원**을 확인하세요. USB는 전원이 없어도 잡히므로 포트가 보이는 "
        "것만으로는 연결됐다고 볼 수 없습니다."
    )
