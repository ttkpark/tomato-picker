"""별도 GPU 비전 프로세스(tools/tomato_vision.py)가 /dev/shm에 쓴 주석 프레임·개수를
읽어 대시보드에 공급한다. 이 클래스는 파일만 읽으므로 torch/ultralytics가 필요 없어
음성 서비스 venv(~/lerobot/.venv)를 그대로 쓴다. server.py의 /video가 latest_jpeg()를
호출하고, 개수 변화는 log_hub(SSE)로 발행한다.
"""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np

from .log_hub import LogHub


def _now() -> str:
    return time.strftime("%H:%M:%S")


def _placeholder() -> bytes:
    img = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.putText(img, "vision starting...", (150, 190),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
    ok, buf = cv2.imencode(".jpg", img)
    return buf.tobytes() if ok else b""


class SharedFrameSource:
    """/dev/shm의 최신 JPEG를 /video로 흘려보내고, 개수 파일 변화를 log_hub로 발행."""

    def __init__(self, log_hub: LogHub, jpeg_path: str, count_path: str,
                 poll_sec: float = 0.3) -> None:
        self._log_hub = log_hub
        self._jpeg_path = jpeg_path
        self._count_path = count_path
        self._poll = poll_sec
        self._placeholder = _placeholder()
        self._stop = False

    def start(self) -> None:
        threading.Thread(target=self._count_loop, daemon=True).start()

    def stop(self) -> None:
        self._stop = True

    def latest_jpeg(self) -> bytes:
        try:
            with open(self._jpeg_path, "rb") as f:
                data = f.read()
            return data if data else self._placeholder
        except OSError:
            return self._placeholder

    def _count_loop(self) -> None:
        last = None
        while not self._stop:
            try:
                with open(self._count_path, encoding="ascii") as f:
                    n = int(f.read().strip())
                if n != last:
                    last = n
                    self._log_hub.publish({
                        "ts": _now(), "kind": "count",
                        "text": f"토마토 {n}개 인식", "count": n,
                    })
            except (OSError, ValueError):
                pass
            time.sleep(self._poll)
