"""카메라 프레임을 색검출로 주석(박스+개수)해 최신 JPEG로 보관하고,
토마토 개수 변화를 log_hub(SSE)로 발행한다. 대시보드의 /video가 이 JPEG를
MJPEG로 흘려보낸다.

색검출은 오프라인·빠름(HSV)이라 GPU가 필요 없다. 다만 빨간 물체는 무엇이든
토마토로 셀 수 있으니(장면에 빨간 인형 등이 있으면 오검출), 개수는 '익은(빨강)
열매' 기준으로만 센다.
"""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np

from ..vision.color_detect import detect_fruits
from .log_hub import LogHub


def _now() -> str:
    return time.strftime("%H:%M:%S")


def _placeholder() -> bytes:
    img = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.putText(img, "camera starting...", (140, 190),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
    ok, buf = cv2.imencode(".jpg", img)
    return buf.tobytes() if ok else b""


def _annotate(frame: np.ndarray, ripe: list) -> np.ndarray:
    """익은 열매에 원을 그리고 좌상단에 개수를 표기(cv2 폰트라 영문)."""
    out = frame.copy()
    for f in ripe:
        x, y = f.position
        r = max(14, int(f.area ** 0.5))
        cv2.circle(out, (x, y), r, (0, 0, 255), 3)
    label = f"tomatoes: {len(ripe)}"
    cv2.putText(out, label, (14, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 0), 6)
    cv2.putText(out, label, (14, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (60, 220, 60), 2)
    return out


class VisionStreamer:
    """카메라를 주기적으로 캡처·검출해 최신 주석 JPEG를 보관하는 워커."""

    def __init__(self, camera, log_hub: LogHub, fps: float = 8.0,
                 jpeg_quality: int = 70) -> None:
        self._camera = camera
        self._log_hub = log_hub
        self._interval = 1.0 / fps
        self._quality = jpeg_quality
        self._lock = threading.Lock()
        self._jpeg = _placeholder()
        self._count = -1
        self._stop = False

    def start(self) -> None:
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self) -> None:
        self._stop = True

    def latest_jpeg(self) -> bytes:
        with self._lock:
            return self._jpeg

    def _loop(self) -> None:
        self._log_hub.publish({"ts": _now(), "kind": "status", "text": "카메라 비전 시작 — 토마토 탐지 중"})
        fail = 0
        while not self._stop:
            t0 = time.monotonic()
            try:
                frame = self._camera.capture()
                fail = 0
                ripe = [f for f in detect_fruits(frame) if f.ripe]
                annotated = _annotate(frame, ripe)
                ok, buf = cv2.imencode(".jpg", annotated,
                                       [cv2.IMWRITE_JPEG_QUALITY, self._quality])
                if ok:
                    with self._lock:
                        self._jpeg = buf.tobytes()
                if len(ripe) != self._count:
                    self._count = len(ripe)
                    self._log_hub.publish({
                        "ts": _now(), "kind": "count",
                        "text": f"토마토 {self._count}개 인식", "count": self._count,
                    })
            except Exception as exc:  # noqa: BLE001 - 카메라 순단에도 워커가 죽지 않게
                fail += 1
                if fail == 1:
                    self._log_hub.publish({"ts": _now(), "kind": "error", "text": f"카메라 오류: {exc}"})
                time.sleep(0.5)
            dt = time.monotonic() - t0
            if dt < self._interval:
                time.sleep(self._interval - dt)
