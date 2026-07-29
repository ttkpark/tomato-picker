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

    def publish(self, event: dict, *, latest_only: bool = False) -> None:
        """latest_only=True면 같은 kind의 이전 이벤트를 history에서 지우고 최신 하나만 남긴다.
        개수·위치처럼 자주 갱신되는 상태값이 history(200칸)를 밀어내지 않게 하면서도,
        새로 접속한 브라우저가 현재 상태를 바로 받도록."""
        with self._lock:
            if latest_only:
                kind = event.get("kind")
                stale = [e for e in self._history if e.get("kind") == kind]
                for e in stale:
                    self._history.remove(e)
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
