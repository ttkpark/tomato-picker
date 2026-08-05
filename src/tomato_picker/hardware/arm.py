"""SO-101 Follower 팔 — 프리셋 포즈 보간 재생으로 구현한 실물 RobotArm.

controller_drive.py의 play_preset() 로직(현재 자세→목표 자세 선형 보간)을
그대로 가져온다. 프리셋은 PS2 컨트롤러로 저장한 ~/arm_presets.json에 있고,
1→2→3→4 재생이 "접근→집기→들기→놓기" 전체 수확 시퀀스임이 실기로 확인됐다
([[tomato-pick-sequence]]). pick_fruit()과 place_in_basket()이 스킬 함수로
나뉘어 있으므로 이 시퀀스를 앞/뒤 절반으로 쪼개 배정한다(config.py 참고).

RobotArm.pick()의 position 인자는 아직 쓰이지 않는다 — 프리셋 재생은
비전 좌표와 무관한 고정 시퀀스이기 때문. 좌표 기반 접근(비전 서보잉)은
색검출이 부착/낙과 판정 이상으로 확장될 때 추가한다.
"""

from __future__ import annotations

import json
import os
import threading
import time

from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

from ..config import (
    ARM_HOME_PRESET,
    ARM_ID,
    ARM_MOVE_FPS,
    ARM_MOVE_SECS,
    ARM_PICK_PRESETS,
    ARM_PLACE_PRESETS,
    ARM_PRESET_FILE,
    ARM_SERIAL_PORT,
)
from .base import RobotArm
from .ports import resolve_arm_port


def _pose_only(observation: dict) -> dict[str, float]:
    return {k: float(v) for k, v in observation.items() if k.endswith(".pos")}


class LerobotArm(RobotArm):
    """arm_presets.json의 저장 자세를 순서대로 보간 재생하는 실물 팔."""

    def __init__(
        self,
        port: str = ARM_SERIAL_PORT,
        arm_id: str = ARM_ID,
        preset_file: str = ARM_PRESET_FILE,
    ) -> None:
        self._port_fallback = port
        self._arm_id = arm_id
        # 두 스레드(음성 인텐트 워커 / 대시보드 수동조작)가 같은 시리얼 버스를
        # 동시에 건드리면 scservo가 "Port is in use!"를 낸다 — 여기서 직렬화한다.
        self._lock = threading.RLock()
        self._follower: SOFollower | None = None
        self._connect()
        with open(os.path.expanduser(preset_file), encoding="utf-8") as f:
            self._presets: dict[str, dict] = json.load(f)

    @property
    def preset_ids(self) -> list[int]:
        """저장된 프리셋 번호(오름차순) — 대시보드 수동조작 버튼 생성용."""
        return sorted(int(k) for k in self._presets if k.isdigit())

    def _connect(self) -> None:
        """지금 실제로 존재하는 경로를 다시 찾아 연결한다(재열거 대응)."""
        port = resolve_arm_port(self._port_fallback)
        follower = SOFollower(SOFollowerRobotConfig(port=port, id=self._arm_id))
        follower.connect(calibrate=False)
        follower.bus.disable_torque()
        self._follower = follower
        self._port = port

    def _reconnect(self) -> None:
        """죽은 연결을 버리고 새로 연다. USB가 빠졌다 다시 붙으면 장치 번호가
        바뀌므로(ttyACM0→ACM1) 경로 재탐색이 핵심."""
        old, self._follower = self._follower, None
        try:
            if old is not None:
                old.disconnect()
        except Exception:  # noqa: BLE001 - 이미 죽은 연결이라 실패가 정상
            pass
        self._connect()

    def _with_retry(self, fn):
        """시리얼 오류면 한 번 재연결 후 재시도. 팔이 끊겼다 붙었을 때
        서비스를 재시작하지 않고도 다음 명령이 살아나게 한다."""
        with self._lock:
            try:
                return fn()
            except Exception as first:  # noqa: BLE001 - 어떤 시리얼 오류든 복구 시도
                print(f"  [arm] 명령 실패({first}) — 재연결 후 1회 재시도")
                try:
                    self._reconnect()
                except Exception as exc:  # noqa: BLE001
                    raise RuntimeError(f"팔 재연결 실패: {exc}") from first
                return fn()

    def pick(self, position: tuple[float, float]) -> None:
        """position은 아직 미사용 — 고정 프리셋 시퀀스만 재생."""
        self._play_sequence(ARM_PICK_PRESETS)

    def place_in_basket(self) -> None:
        self._play_sequence(ARM_PLACE_PRESETS)

    def home(self) -> None:
        self._play_preset(ARM_HOME_PRESET)

    def demo_move(self) -> None:
        """음성 명령 트리거용 — 검증된 전체 시퀀스(1→2→3→4) 재생."""
        self._play_sequence(ARM_PICK_PRESETS + ARM_PLACE_PRESETS)

    def play_preset(self, preset_id: int) -> None:
        """프리셋 하나 재생 — 대시보드 수동조작에서 직접 부른다."""
        self._play_preset(preset_id)

    def relax(self) -> None:
        """토크를 풀어 손으로 자세를 바꿀 수 있게 한다(수동조작 화면의 '힘 빼기')."""
        self._with_retry(lambda: self._follower.bus.disable_torque())

    def close(self) -> None:
        with self._lock:
            if self._follower is None:
                return
            try:
                self._follower.bus.disable_torque()
                self._follower.disconnect()
            except Exception:  # noqa: BLE001 - 종료 경로에서 실패는 무시
                pass

    # --- 내부 ---

    def _play_sequence(self, preset_ids: list[int]) -> None:
        for preset_id in preset_ids:
            self._play_preset(preset_id)

    def _play_preset(
        self, preset_id: int, secs: float = ARM_MOVE_SECS, fps: int = ARM_MOVE_FPS
    ) -> None:
        target = self._presets.get(str(preset_id))
        if not target:
            raise KeyError(f"프리셋 {preset_id}가 {ARM_PRESET_FILE}에 없습니다.")
        self._with_retry(lambda: self._move_to(target, secs, fps))

    def _move_to(self, target: dict, secs: float, fps: int) -> None:
        self._follower.bus.enable_torque()
        current = _pose_only(self._follower.get_observation())
        steps = max(2, int(secs * fps))
        # 스텝마다 sleep(secs/steps)만 하면 send_action에 걸린 시간이 누적돼
        # 실제론 목표보다 오래 걸리고 움직임이 뚝뚝 끊긴다. 절대 시각 기준으로
        # 다음 스텝 시각을 맞춰 남은 시간만 잔다(늦었으면 안 잔다).
        start = time.monotonic()
        interval = secs / steps
        for step in range(1, steps + 1):
            action = {
                k: current.get(k, target[k]) + (target[k] - current.get(k, target[k])) * step / steps
                for k in target
            }
            self._follower.send_action(action)
            slack = (start + step * interval) - time.monotonic()
            if slack > 0:
                time.sleep(slack)
