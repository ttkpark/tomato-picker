"""마이크 → VAD → STT → 인텐트 매칭을 잇는 메인 루프.

인식된 텍스트/발화 시작 시점을 모두 log_hub로 발행해 실시간 뷰어에서
"듣고 있다"는 걸 눈으로 확인할 수 있게 한다.

이 카메라의 USB 오디오 코덱은 클럭이 불안정해 arecord가 이따금
"Input/output error"로 죽는다(2026-07-08 실측). 그래서 마이크가 끊기면
조용히 서비스가 끝나버리는 대신 재연결하며 로그로 알린다.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

from .intents import match_intent
from .log_hub import LogHub
from .mic_stream import MicStream, MicStreamError
from .stt import WhisperSTT
from .vad import SpeechSegmenter

RECONNECT_BACKOFF_SEC = 2.0


def _now() -> str:
    return time.strftime("%H:%M:%S")


class VoiceController:
    def __init__(self, log_hub: LogHub, on_intent: Callable[[str], None] | None = None) -> None:
        self._log_hub = log_hub
        self._on_intent = on_intent

    def run(self) -> None:
        """블로킹 루프. STT 모델 로드 포함 — 첫 실행에 수십 초 걸린다.

        마이크가 죽으면(MicStreamError) 잠깐 쉬었다 재연결을 무한 반복한다
        — USB 마이크가 원래 이따금 끊기는 하드웨어라, 서비스가 조용히
        멈추는 것보다 계속 재시도하며 로그로 알리는 편이 낫다.
        """
        self._log_hub.publish({"ts": _now(), "kind": "status", "text": "Whisper 모델 로딩 중..."})
        stt = WhisperSTT()
        self._log_hub.publish({"ts": _now(), "kind": "status", "text": "준비 완료 — 듣는 중"})

        segmenter = SpeechSegmenter()
        while True:
            try:
                self._listen_until_error(stt, segmenter)
            except MicStreamError as exc:
                self._log_hub.publish(
                    {"ts": _now(), "kind": "error", "text": f"마이크 끊김: {exc} — {RECONNECT_BACKOFF_SEC:.0f}초 후 재연결"}
                )
                time.sleep(RECONNECT_BACKOFF_SEC)

    def _listen_until_error(self, stt: WhisperSTT, segmenter: SpeechSegmenter) -> None:
        mic = MicStream()
        self._log_hub.publish({"ts": _now(), "kind": "status", "text": f"마이크 연결됨({mic.device})"})
        try:
            for chunk in mic.chunks():
                utterance, _level, started = segmenter.feed(chunk)
                if started:
                    self._log_hub.publish({"ts": _now(), "kind": "heard", "text": "(발화 감지...)"})
                if utterance is None:
                    continue

                text = stt.transcribe(utterance)
                if not text:
                    self._log_hub.publish({"ts": _now(), "kind": "heard", "text": "(인식 실패/무음)"})
                    continue

                intent = match_intent(text)
                kind = "intent" if intent else "heard"
                label = f"{text}" + (f"  → 인텐트: {intent}" if intent else "")
                self._log_hub.publish({"ts": _now(), "kind": kind, "text": label})

                if intent and self._on_intent:
                    # 별도 스레드로 실행 — 팔 동작(수 초 블로킹)이 마이크 읽기를
                    # 막으면 arecord 파이프가 밀려 다음 발화를 놓칠 수 있다.
                    threading.Thread(target=self._on_intent, args=(intent,), daemon=True).start()
        finally:
            mic.close()
