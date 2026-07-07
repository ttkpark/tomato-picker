"""하드웨어 인터페이스 정의.

상위 코드(스킬 함수)는 이 추상 인터페이스에만 의존한다.
Mock이든 실제 장비든 같은 메서드를 구현하면 그대로 교체된다.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class MobileBase(ABC):
    """메카넘 휠 이동 베이스 — 직선 이동 전용."""

    @abstractmethod
    def drive_to(self, distance: float) -> None:
        """기준점에서 지정 거리 위치까지 직선 이동(블로킹)."""


class RobotArm(ABC):
    """LeRobot SO-ARM101 류 로봇팔 — 사전 정의 시퀀스 재생."""

    @abstractmethod
    def pick(self, position: tuple[float, float]) -> None:
        """주어진 (x, y) 위치의 열매를 따는 시퀀스를 재생(블로킹)."""

    @abstractmethod
    def place_in_basket(self) -> None:
        """그리퍼에 든 열매를 바구니에 놓는 시퀀스를 재생(블로킹)."""

    @abstractmethod
    def home(self) -> None:
        """홈 포지션으로 복귀."""

    @abstractmethod
    def demo_move(self) -> None:
        """음성 명령("팔 움직여") 등으로 트리거하는 전체 시연 동작."""


class Camera(ABC):
    """전면/손목 카메라 — 한 프레임을 BGR ndarray로 반환."""

    @abstractmethod
    def capture(self) -> np.ndarray:
        """현재 프레임을 (H, W, 3) BGR 배열로 캡처."""
