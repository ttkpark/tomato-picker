"""시리얼 장치 경로를 **재열거에 견디게** 찾아준다.

배경(2026-08-05 실기): 팔의 USB-시리얼(CH343, 1a86:55d3)이 주행 중 반복적으로
끊겼다 붙었고, 그때마다 커널이 번호를 새로 매겨 `/dev/ttyACM0` → `ttyACM1`로
바뀌었다. 코드가 ttyACM0을 하드코딩하고 있어서

  1. 서비스 기동 때 잡은 fd는 죽은 장치를 가리키고
  2. lerobot(scservo)이 그 fd에 쓰다 실패하며 내부 `is_using` 플래그가 True로
     남아, 이후 모든 명령이 "[TxRxResult] Port is in use!"로 영구히 막혔다

`/dev/serial/by-id/`는 USB 시리얼번호 기반이라 재열거해도 이름이 그대로다.
그래서 by-id를 1순위로 쓰고, 없으면 VID:PID로 sysfs를 훑고, 그것도 실패하면
예전 하드코딩 경로로 떨어진다.

⚠ 이건 증상 완화지 근본 해결이 아니다. 끊김 자체는 케이블 접촉/전원 문제이며
(dmesg에 `device descriptor read/64, error -32` 동반), 양품 케이블과 안정된
전원이 진짜 해결책이다.
"""

from __future__ import annotations

import glob
import os

# by-id 이름에 이 문자열이 들어가면 해당 장치로 본다.
ARM_BY_ID_HINT = "USB_Single_Serial"   # CH343 (SO-101 팔로워 보드)
BASE_BY_ID_HINT = "USB_Serial-if00"    # CH340 (메카넘 모터보드)

BY_ID_DIR = "/dev/serial/by-id"


def _by_id(hint: str) -> str | None:
    """by-id 심볼릭 링크 중 hint를 포함하는 첫 경로. 링크가 가리키는 실체가
    실제로 존재할 때만 돌려준다(장치가 빠지면 링크도 사라지지만 안전하게)."""
    try:
        names = sorted(os.listdir(BY_ID_DIR))
    except OSError:
        return None
    for name in names:
        if hint in name:
            path = f"{BY_ID_DIR}/{name}"
            if os.path.exists(os.path.realpath(path)):
                return path
    return None


def _first_existing(patterns: list[str]) -> str | None:
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return None


def resolve_arm_port(fallback: str) -> str:
    """팔 시리얼 경로. by-id → /dev/ttyACM* 스캔 → fallback 순."""
    return _by_id(ARM_BY_ID_HINT) or _first_existing(["/dev/ttyACM*"]) or fallback


def resolve_base_port(fallback: str) -> str:
    """베이스 시리얼 경로. by-id → /dev/ttyUSB* 스캔 → fallback 순."""
    return _by_id(BASE_BY_ID_HINT) or _first_existing(["/dev/ttyUSB*"]) or fallback


def list_arm_ports() -> list[str]:
    """팔 계열(ttyACM) 후보 경로 전부. 리더/팔로워가 둘 다 CH343이라
    by-id 이름도 시리얼번호로만 갈리므로, 어느 쪽인지는 **사용자가 고른다**
    (웹 화면의 리더 포트 선택). by-id를 앞에 두어 재열거에 안전한 경로를 우선."""
    ports: list[str] = []
    try:
        for name in sorted(os.listdir(BY_ID_DIR)):
            if ARM_BY_ID_HINT in name:
                path = f"{BY_ID_DIR}/{name}"
                if os.path.exists(os.path.realpath(path)):
                    ports.append(path)
    except OSError:
        pass
    seen = {os.path.realpath(p) for p in ports}
    for path in sorted(glob.glob("/dev/ttyACM*")):
        if os.path.realpath(path) not in seen:
            ports.append(path)
    return ports


def resolve_leader_port(exclude: str | None) -> str | None:
    """리더암 경로 — 팔로워가 이미 잡은 포트를 뺀 첫 후보.

    exclude에는 팔로워가 **실제로 열어둔** 경로를 넘긴다. by-id 링크와 실경로가
    섞여 들어와도 같은 장치를 두 번 열지 않도록 realpath로 비교한다.
    """
    blocked = {os.path.realpath(exclude)} if exclude else set()
    for path in list_arm_ports():
        if os.path.realpath(path) not in blocked:
            return path
    return None
