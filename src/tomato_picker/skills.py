"""스킬 함수 — 로봇이 할 수 있는 결정적 동작들.

이 5개 함수가 시스템의 핵심 API다. Claude에는 tool로 노출되고(orchestrator),
오프라인 데모(demo_mode)에서는 직접 순서대로 호출된다.
즉 "AI 경로"와 "폴백 경로"가 같은 함수를 공유하므로 동작이 일관된다.
"""

from __future__ import annotations

from .config import TREES, USE_LLM_VISION
from .hardware.base import Camera, MobileBase, RobotArm
from .vision.color_detect import Fruit, detect_fruits


class TomatoPicker:
    """하드웨어 + 비전을 묶어 스킬 함수를 제공하는 로봇 파사드."""

    def __init__(self, base: MobileBase, arm: RobotArm, camera: Camera) -> None:
        self._base = base
        self._arm = arm
        self._camera = camera
        self._current_tree: int | None = None
        self._last_scan: list[Fruit] = []
        self.basket_count = 0

    # --- 스킬 함수 (Claude tool로 노출되는 단위) ---

    def drive_to_tree(self, tree_id: int) -> str:
        if tree_id not in TREES:
            return f"오류: 알 수 없는 나무 id {tree_id} (가능: {sorted(TREES)})"
        self._base.drive_to(TREES[tree_id])
        self._current_tree = tree_id
        self._last_scan = []
        return f"나무 {tree_id} 앞 도착."

    def scan_tree(self) -> dict:
        """카메라로 열매를 검출. fruit id는 이번 스캔 한정 인덱스."""
        if self._current_tree is None:
            return {"error": "먼저 drive_to_tree로 나무 앞으로 이동하세요."}
        frame = self._camera.capture()
        if USE_LLM_VISION:
            # 온라인일 때만: 이미지를 Claude에 보내 판단. (지연시 import만 무겁게)
            from .vision.claude_vision import detect_fruits_llm

            self._last_scan = detect_fruits_llm(frame)
        else:
            self._last_scan = detect_fruits(frame)
        fruits = [{"id": i, **f.as_dict()} for i, f in enumerate(self._last_scan)]
        ripe = sum(1 for f in self._last_scan if f.ripe)
        return {"tree_id": self._current_tree, "fruits": fruits, "ripe_count": ripe}

    def pick_fruit(self, fruit_id: int) -> str:
        if not 0 <= fruit_id < len(self._last_scan):
            return f"오류: fruit id {fruit_id} 없음. scan_tree를 먼저 호출하세요."
        fruit = self._last_scan[fruit_id]
        if not fruit.ripe:
            return f"건너뜀: fruit {fruit_id}는 초록(미익음)이라 수확 안 함."
        self._arm.pick(fruit.position)
        return f"fruit {fruit_id} 땄음(그리퍼 보유 중)."

    def place_in_basket(self) -> str:
        self._arm.place_in_basket()
        self.basket_count += 1
        return f"바구니에 담음. 누적 {self.basket_count}개."

    def home(self) -> str:
        self._arm.home()
        return "홈 포지션 복귀 완료."
