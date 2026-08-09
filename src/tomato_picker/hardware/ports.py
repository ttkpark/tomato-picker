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


def _by_id_all(hint: str) -> list[str]:
    """by-id 심볼릭 링크 중 hint를 포함하는 경로 전부(이름순). 링크가 가리키는
    실체가 실제로 존재할 때만 넣는다(장치가 빠지면 링크도 사라지지만 안전하게)."""
    try:
        names = sorted(os.listdir(BY_ID_DIR))
    except OSError:
        return []
    found = []
    for name in names:
        if hint in name:
            path = f"{BY_ID_DIR}/{name}"
            if os.path.exists(os.path.realpath(path)):
                found.append(path)
    return found


def _by_id(hint: str) -> str | None:
    """hint를 포함하는 첫 by-id 경로.

    ⚠ 후보가 여럿일 수 있는 장치(팔: 리더/팔로워)에는 쓰면 안 된다 — 이름만으로는
    구분이 안 된다. 그런 장치는 by_id_serial()로 시리얼번호를 못 박아 쓸 것.
    """
    found = _by_id_all(hint)
    return found[0] if found else None


def _first_existing(patterns: list[str], exclude: set[str] | None = None) -> str | None:
    blocked = exclude or set()
    for pattern in patterns:
        for path in sorted(glob.glob(pattern)):
            if os.path.realpath(path) not in blocked:
                return path
    return None


def by_id_serial(serial: str) -> str | None:
    """USB 시리얼번호로 by-id 링크를 찾는다 (`..._USB_Single_Serial_<serial>-if00`).

    리더/팔로워를 **확실히** 갈라내는 유일한 방법 — 둘 다 같은 CH343이라
    VID:PID도 product 문자열도 같고, ttyACM 번호는 꽂는 순서마다 바뀐다.
    """
    if not serial:
        return None
    try:
        names = sorted(os.listdir(BY_ID_DIR))
    except OSError:
        return None
    for name in names:
        if serial in name:
            path = f"{BY_ID_DIR}/{name}"
            if os.path.exists(os.path.realpath(path)):
                return path
    return None


def resolve_arm_port(
    fallback: str, follower_serial: str = "", leader_serial: str = ""
) -> str:
    """**팔로워** 팔의 시리얼 경로.

    follower_serial이 있으면 그 시리얼의 by-id 링크만 쓴다. 못 찾으면 폴백하지
    않고 **명확히 실패한다** — 여기서 아무 ttyACM이나 집으면 리더암이 팔로워로
    열려 프리셋 재생에 리더가 움직인다(2026-08-09 실사고). 팔로워가 안 꽂혔으면
    붙을 때까지 못 붙는 게 맞다(voice_mode의 재연결 루프가 알아서 다시 잡는다).
    """
    if follower_serial:
        pinned = by_id_serial(follower_serial)
        if pinned:
            return pinned
        raise RuntimeError(
            f"팔로워 팔(시리얼 {follower_serial})을 찾지 못했습니다 — USB를 확인하세요. "
            "리더암을 잘못 잡는 사고를 막으려고 다른 포트로 폴백하지 않습니다 "
            "(젯슨에서 `ls -l /dev/serial/by-id/`로 확인)."
        )
    # 시리얼 미지정(구형 설정) — 옛 자동탐색이되, 리더로 알려진 놈은 반드시 뺀다.
    known_leader = by_id_serial(leader_serial)
    blocked = {os.path.realpath(known_leader)} if known_leader else set()
    for path in _by_id_all(ARM_BY_ID_HINT):
        if os.path.realpath(path) not in blocked:
            return path
    return _first_existing(["/dev/ttyACM*"], blocked) or fallback


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


def resolve_leader_port(exclude: str | None, leader_serial: str = "") -> str | None:
    """리더암 경로 — 시리얼로 못 박은 리더가 1순위, 없으면 팔로워를 뺀 첫 후보.

    exclude에는 팔로워가 **실제로 열어둔** 경로를 넘긴다. by-id 링크와 실경로가
    섞여 들어와도 같은 장치를 두 번 열지 않도록 realpath로 비교한다.
    """
    blocked = {os.path.realpath(exclude)} if exclude else set()
    pinned = by_id_serial(leader_serial)
    if pinned and os.path.realpath(pinned) not in blocked:
        return pinned
    for path in list_arm_ports():
        if os.path.realpath(path) not in blocked:
            return path
    return None
