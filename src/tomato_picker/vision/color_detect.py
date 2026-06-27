"""색 기반 열매 검출.

단순 HSV 마스킹이라 조명만 잡히면 신뢰성이 높다. 여유가 생기면
이 모듈만 YOLO/뎁스로 교체하면 되도록 입출력 형태를 고정해 둔다.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ..config import GREEN_HSV_RANGE, MIN_FRUIT_AREA_PX, RED_HSV_RANGES


@dataclass
class Fruit:
    """검출된 열매 한 개."""

    position: tuple[int, int]  # 프레임 내 (x, y) 중심 좌표
    ripe: bool                 # 빨강이면 True(수확 대상)
    area: int                  # 픽셀 면적 (가까울수록 큼)

    def as_dict(self) -> dict:
        """Claude tool_result로 넘길 JSON 직렬화 형태."""
        return {"position": list(self.position), "ripe": self.ripe, "area": self.area}


def _mask_to_fruits(mask: np.ndarray, ripe: bool) -> list[Fruit]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    fruits: list[Fruit] = []
    for c in contours:
        area = int(cv2.contourArea(c))
        if area < MIN_FRUIT_AREA_PX:
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        cx = int(m["m10"] / m["m00"])
        cy = int(m["m01"] / m["m00"])
        fruits.append(Fruit(position=(cx, cy), ripe=ripe, area=area))
    return fruits


def detect_fruits(frame_bgr: np.ndarray) -> list[Fruit]:
    """BGR 프레임에서 빨강/초록 열매를 검출해 면적 내림차순으로 반환."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    red_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in RED_HSV_RANGES:
        red_mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

    lo, hi = GREEN_HSV_RANGE
    green_mask = cv2.inRange(hsv, np.array(lo), np.array(hi))

    fruits = _mask_to_fruits(red_mask, ripe=True) + _mask_to_fruits(green_mask, ripe=False)
    fruits.sort(key=lambda f: f.area, reverse=True)
    return fruits
