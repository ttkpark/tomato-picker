"""스레드 간 실시간 로그 배포 — 인식 루프(발행)와 HTTP SSE 핸들러(구독)를 잇는다."""

from __future__ import annotations

import queue
import threading
from collections import deque


class LogHub:
    def __init__(self, history_size: int = 200) -> None:
        self._lock = threading.Lock()
        self._subscribers: list[queue.Queue] = []
        self._history: deque[dict] = deque(maxlen=history_size)

    def publish(self, event: dict) -> None:
        with self._lock:
            self._history.append(event)
            subs = list(self._subscribers)
        for q in subs:
            try:
                q.put_nowait(event)
            except queue.Full:
                pass  # 느린 구독자는 최신 이벤트를 놓칠 수 있음(연결 유지가 더 중요)

    def subscribe(self) -> tuple[queue.Queue, list[dict]]:
        q: queue.Queue = queue.Queue(maxsize=100)
        with self._lock:
            self._subscribers.append(q)
            history = list(self._history)
        return q, history

    def unsubscribe(self, q: queue.Queue) -> None:
        with self._lock:
            if q in self._subscribers:
                self._subscribers.remove(q)
