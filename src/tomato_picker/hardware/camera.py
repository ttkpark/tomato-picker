"""젯슨에 꽂은 USB 웹캠(베이스 전면 고정) 실물 구현.

기본 YUYV로 열면 5fps로 떨어지는 카메라라 MJPG 고정이 필수다.
카메라를 매 캡처마다 열고 닫으면 느리고 자동노출이 매번 리셋되므로
연결은 __init__에서 한 번만 하고, capture()는 열린 채로 프레임만 읽는다.
"""

from __future__ import annotations

import cv2
import numpy as np

from ..config import (
    CAMERA_FOURCC,
    CAMERA_HEIGHT,
    CAMERA_INDEX,
    CAMERA_WARMUP_FRAMES,
    CAMERA_WIDTH,
)
from .base import Camera


class JetsonCamera(Camera):
    """MJPG 강제 설정 + 자동노출 워밍업을 거친 실물 USB 카메라."""

    def __init__(
        self,
        index: int = CAMERA_INDEX,
        width: int = CAMERA_WIDTH,
        height: int = CAMERA_HEIGHT,
        warmup_frames: int = CAMERA_WARMUP_FRAMES,
    ) -> None:
        self._cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*CAMERA_FOURCC))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self._cap.isOpened():
            raise RuntimeError(f"카메라(index={index})를 열 수 없습니다.")
        for _ in range(warmup_frames):
            self._cap.read()

    def capture(self) -> np.ndarray:
        """V4L2 버퍼에 쌓인 오래된 프레임을 흘려보내고 최신 프레임을 반환."""
        for _ in range(4):
            self._cap.grab()
        ok, frame = self._cap.read()
        if not ok:
            raise RuntimeError("카메라 프레임을 읽지 못했습니다.")
        return frame

    def close(self) -> None:
        self._cap.release()
