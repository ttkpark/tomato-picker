"""호출어 게이트 — "안녕"이라고 부른 뒤에만 명령을 받는다.

**왜.** 마이크는 늘 켜져 있고 부스에는 사람이 말한다. 명령 낱말을 아무리 잘
고르고 근사 매칭을 조여도, 옆에서 나눈 대화가 언젠가는 명령으로 읽힌다 —
그때 실제로 바퀴가 구르고 팔이 나간다. 호출어는 그 사고를 구조적으로 막는다:
**부르지 않으면 아무것도 실행하지 않는다.**

**창(window)을 쓰는 이유.** "안녕 아래 토마토 따줘"처럼 한 번에 말하는 사람도
있고, "안녕" → (로봇이 응답) → "아래 토마토 따줘"처럼 나눠 말하는 사람도 있다.
호출어가 들리면 일정 시간 창을 열어 둘 다 되게 하고, 명령을 하나 실행할 때마다
창을 다시 연장한다(연속 조작 중에 다시 부르게 하면 성가시다).

**못 알아들었을 때가 진짜 문제다.** 호출어를 놓치면 사용자는 "명령이 안 먹는다"고만
느낀다. 그래서 **명령은 알아들었는데 창이 닫혀 있는 경우**를 로그에 또렷이 남긴다
("…를 들었지만 대기 중 — 먼저 '안녕'이라고 부르세요"). 무엇이 빠졌는지 화면에
보이면 사용자가 스스로 고칠 수 있다.

끄는 것도 한 번에 되어야 한다 — 데모 직전에 호출어가 안 걸리면 /settings에서
꺼서 예전처럼 항상 듣게 만들 수 있다(그 상태도 화면에 표시된다).
"""

from __future__ import annotations

import json
import os
import threading
import time

from ..config import (
    VOICE_WAKE_ENABLED,
    VOICE_WAKE_FILE,
    VOICE_WAKE_WINDOW_SEC,
)
from . import korean, words

# 창 길이의 상식적인 한계. 너무 짧으면 말하다 닫히고, 너무 길면 호출어가 무의미해진다.
MIN_WINDOW_SEC = 3.0
MAX_WINDOW_SEC = 300.0


class WakeGate:
    def __init__(self, path: str | None = VOICE_WAKE_FILE) -> None:
        self._path = os.path.expanduser(path) if path else None
        self._lock = threading.Lock()
        self._enabled = bool(VOICE_WAKE_ENABLED)
        self._window = float(VOICE_WAKE_WINDOW_SEC)
        # monotonic 기준 — 시계가 바뀌어도(NTP 동기) 창이 튀지 않는다.
        self._until = 0.0
        self._load()

    # ---------------- 저장 ----------------

    def _load(self) -> None:
        if self._path is None:
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                saved = json.load(f)
        except (OSError, ValueError):
            return
        if isinstance(saved.get("enabled"), bool):
            self._enabled = saved["enabled"]
        try:
            self._window = self._clamp(float(saved.get("window_sec", self._window)))
        except (TypeError, ValueError):
            pass

    def _save(self) -> None:
        if self._path is None:
            return
        try:
            with open(self._path, "w", encoding="utf-8") as f:
                json.dump({"enabled": self._enabled, "window_sec": self._window}, f)
        except OSError as exc:
            print(f"  [wake] 저장 실패: {exc}")

    @staticmethod
    def _clamp(sec: float) -> float:
        return max(MIN_WINDOW_SEC, min(MAX_WINDOW_SEC, sec))

    # ---------------- 판정 ----------------

    def heard(self, text: str) -> bool:
        """이 발화에 호출어가 들어 있나. 낱말은 다른 명령어와 같은 사전에서 온다."""
        return korean.contains(text, words.STORE.get("wake"))

    def open(self) -> None:
        """창을 연다(호출어를 들었을 때)."""
        with self._lock:
            self._until = time.monotonic() + self._window

    def touch(self) -> None:
        """명령을 하나 받았으니 창을 연장한다 — 연속 조작 중 다시 부르지 않게."""
        with self._lock:
            if self._until:
                self._until = time.monotonic() + self._window

    def is_open(self) -> bool:
        with self._lock:
            return time.monotonic() < self._until

    def allows(self) -> bool:
        """지금 명령을 실행해도 되나. 게이트가 꺼져 있으면 항상 허용."""
        with self._lock:
            return (not self._enabled) or time.monotonic() < self._until

    def remaining(self) -> float:
        with self._lock:
            return max(0.0, self._until - time.monotonic())

    # ---------------- 설정 ----------------

    def configure(self, enabled: bool | None = None, window_sec: float | None = None) -> str:
        with self._lock:
            if enabled is not None:
                self._enabled = bool(enabled)
                if not self._enabled:
                    self._until = 0.0      # 껐다 켜도 옛 창이 남아 있지 않게
            if window_sec is not None:
                self._window = self._clamp(float(window_sec))
            enabled_now, window_now = self._enabled, self._window
            self._save()
        if not enabled_now:
            return "호출어 끔 — 부르지 않아도 모든 명령을 바로 실행합니다"
        return f"호출어 켬 — 부른 뒤 {window_now:.0f}초 동안 명령을 받습니다"

    def status(self) -> dict:
        with self._lock:
            enabled, window, until = self._enabled, self._window, self._until
        remaining = max(0.0, until - time.monotonic())
        return {
            "enabled": enabled,
            "window_sec": window,
            "open": (not enabled) or remaining > 0,
            "remaining_sec": round(remaining, 1),
            "wake_words": words.STORE.get("wake"),
        }

    def call_hint(self) -> str:
        """'"안녕"이라고' — 지금 사전에 든 첫 호출어로 안내한다(낱말을 바꿔도 따라간다)."""
        first = (words.STORE.get("wake") or ["안녕"])[0]
        return f'"{first}"이라고'

    def badge(self) -> str:
        """대시보드 배지 한 줄 — 지금 듣고 있는지 사람이 바로 알게."""
        st = self.status()
        if not st["enabled"]:
            return "🎤 항상 듣는 중 (호출어 꺼짐)"
        if st["remaining_sec"] > 0:
            return f"🎤 듣는 중 — {st['remaining_sec']:.0f}초"
        first = (st["wake_words"] or ["안녕"])[0]
        return f"💤 대기 중 — \"{first}\"이라고 부르세요"


GATE = WakeGate()
