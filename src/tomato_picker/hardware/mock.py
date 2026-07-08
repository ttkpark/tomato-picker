"""하드웨어 없이 PC에서 도는 Mock 구현.

동작을 콘솔에 출력하고, 카메라는 무대를 흉내 낸 합성 이미지를 만든다.
나무마다 빨강/초록 열매가 정해진 위치에 박혀 있어 색검출 파이프라인을
실제 영상 없이도 끝까지 검증할 수 있다.
"""

from __future__ import annotations

import time

import cv2
import numpy as np

from .base import Camera, MobileBase, RobotArm


def _log(msg: str) -> None:
    print(f"  [hw] {msg}")


class MockBase(MobileBase):
    def __init__(self) -> None:
        self.position = 0.0

    def drive_to(self, distance: float) -> None:
        _log(f"베이스 이동: {self.position:.0f} → {distance:.0f}")
        time.sleep(0.2)
        self.position = distance

    def drive_forward(self, seconds: float) -> None:
        _log(f"베이스: {seconds:.1f}초간 전진(음성 트리거)")
        time.sleep(min(seconds, 0.3))

    def drive_backward(self, seconds: float) -> None:
        _log(f"베이스: {seconds:.1f}초간 후진(음성 트리거)")
        time.sleep(min(seconds, 0.3))


class MockArm(RobotArm):
    def pick(self, position: tuple[float, float]) -> None:
        _log(f"팔: ({position[0]:.0f}, {position[1]:.0f}) 열매 따기 시퀀스")
        time.sleep(0.3)

    def place_in_basket(self) -> None:
        _log("팔: 바구니에 담기")
        time.sleep(0.2)

    def home(self) -> None:
        _log("팔: 홈 포지션 복귀")
        time.sleep(0.2)

    def demo_move(self) -> None:
        _log("팔: 데모 동작(음성 트리거)")
        time.sleep(0.3)


class MockCamera(Camera):
    """현재 베이스 위치에 따라 무대 한 나무를 합성해 보여준다."""

    def __init__(self, base: MockBase) -> None:
        self._base = base
        # 나무 거리 → (BGR 색, (x, y) 위치) 열매 목록.
        # OpenCV는 BGR이므로 빨강=(0,0,255), 초록=(0,200,0).
        self._stage: dict[float, list[tuple[tuple[int, int, int], tuple[int, int]]]] = {
            0.0: [((0, 0, 255), (160, 200)), ((0, 200, 0), (420, 260))],
            40.0: [((0, 0, 255), (200, 180)), ((0, 0, 255), (430, 300))],
            80.0: [((0, 200, 0), (240, 220))],
        }

    def capture(self) -> np.ndarray:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        for color, (x, y) in self._stage.get(self._base.position, []):
            cv2.circle(frame, (x, y), 35, color, thickness=-1)
        return frame
