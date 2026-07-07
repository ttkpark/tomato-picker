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
        self._follower = SOFollower(SOFollowerRobotConfig(port=port, id=arm_id))
        self._follower.connect(calibrate=False)
        self._follower.bus.disable_torque()
        with open(os.path.expanduser(preset_file), encoding="utf-8") as f:
            self._presets: dict[str, dict] = json.load(f)

    def pick(self, position: tuple[float, float]) -> None:
        """position은 아직 미사용 — 고정 프리셋 시퀀스만 재생."""
        self._play_sequence(ARM_PICK_PRESETS)

    def place_in_basket(self) -> None:
        self._play_sequence(ARM_PLACE_PRESETS)

    def home(self) -> None:
        self._play_preset(ARM_HOME_PRESET)

    def close(self) -> None:
        self._follower.bus.disable_torque()
        self._follower.disconnect()

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
        self._follower.bus.enable_torque()
        current = _pose_only(self._follower.get_observation())
        steps = max(2, int(secs * fps))
        for step in range(1, steps + 1):
            action = {
                k: current.get(k, target[k]) + (target[k] - current.get(k, target[k])) * step / steps
                for k in target
            }
            self._follower.send_action(action)
            time.sleep(secs / steps)
