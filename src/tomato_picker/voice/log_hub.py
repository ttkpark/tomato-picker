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
        # kind별 "최신 상태" 이벤트. deque와 분리해 **고정**한다 — 아래 참고.
        self._latest: dict[str, dict] = {}

    def publish(self, event: dict, *, latest_only: bool = False) -> None:
        """latest_only=True면 이 kind의 최신 하나를 **history 밖에 고정**해 둔다.

        ⚠ 예전엔 deque 안에서 같은 kind를 지우고 다시 넣는 방식이었는데, 그러면
        다른 이벤트가 200개 쌓이는 순간 **deque 끝으로 밀려나 증발**한다.
        실사고(2026-08-12): 마이크가 주변 소음("아, 아, 아...")을 밤새 주워 담아
        heard 이벤트가 히스토리를 가득 채웠고, 시작 때 한 번 발행된 장비 상태(hw)
        이벤트가 밀려나 — 새로 연 대시보드가 "장비 상태 확인 중..."에서 영영
        멈췄다. 상태 이벤트는 로그가 아니라 **현재값**이므로 유량과 무관하게
        살아 있어야 한다.
        """
        with self._lock:
            if latest_only:
                self._latest[str(event.get("kind"))] = event
            else:
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
            # 상태(최신값)를 먼저 — 새 브라우저가 로그를 다 읽기 전에 배지부터 뜬다.
            history = list(self._latest.values()) + list(self._history)
        return q, history

    def unsubscribe(self, q: queue.Queue) -> None:
        with self._lock:
            if q in self._subscribers:
                self._subscribers.remove(q)
