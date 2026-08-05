"""마이크 → VAD → STT → 인텐트 매칭을 잇는 메인 루프.

인식된 텍스트/발화 시작 시점을 모두 log_hub로 발행해 실시간 뷰어에서
"듣고 있다"는 걸 눈으로 확인할 수 있게 한다.

이 카메라의 USB 오디오 코덱은 클럭이 불안정해 arecord가 이따금
"Input/output error"로 죽는다(2026-07-08 실측). 그래서 마이크가 끊기면
조용히 서비스가 끝나버리는 대신 재연결하며 로그로 알린다.
"""

from __future__ import annotations

import queue
import threading
import time
import wave
from collections.abc import Callable

from ..config import MIC_SAMPLE_RATE
from .intents import match_intent
from .log_hub import LogHub
from .mic_stream import MicStream, MicStreamError
from .stt import WhisperSTT
from .vad import SpeechSegmenter

RECONNECT_BACKOFF_SEC = 2.0
# 인식 실패한 발화를 귀로 확인할 수 있게 /dev/shm에 최근 것만 돌려가며 남긴다
# (tmpfs라 SD카드 수명에 영향 없음, 재부팅하면 사라짐).
FAIL_WAV_DIR = "/dev/shm"
FAIL_WAV_SLOTS = 5


def _now() -> str:
    return time.strftime("%H:%M:%S")


def _dump_wav(utterance) -> str | None:
    """실패한 발화를 WAV로 저장하고 경로 반환. 실패해도 인식 루프는 멈추지 않는다."""
    global _fail_slot
    try:
        path = f"{FAIL_WAV_DIR}/voice_fail_{_fail_slot}.wav"
        _fail_slot = (_fail_slot + 1) % FAIL_WAV_SLOTS
        with wave.open(path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(MIC_SAMPLE_RATE)
            w.writeframes(utterance.tobytes())
        return path
    except OSError:
        return None


_fail_slot = 0


class VoiceController:
    def __init__(
        self,
        log_hub: LogHub,
        on_intent: Callable[[str], None] | None = None,
        on_mic_state: Callable[[str], None] | None = None,
    ) -> None:
        self._log_hub = log_hub
        self._on_intent = on_intent
        # 대시보드 상단 장비 배지용 — 마이크가 붙었나/끊겼나를 'ok'/'down'으로 알린다.
        self._on_mic_state = on_mic_state
        self._intent_queue: queue.Queue[str] = queue.Queue()

    def _mic_state(self, state: str) -> None:
        if self._on_mic_state:
            self._on_mic_state(state)

    def run(self) -> None:
        """블로킹 루프. STT 모델 로드 포함 — 첫 실행에 수십 초 걸린다.

        마이크가 죽으면(MicStreamError) 잠깐 쉬었다 재연결을 무한 반복한다
        — USB 마이크가 원래 이따금 끊기는 하드웨어라, 서비스가 조용히
        멈추는 것보다 계속 재시도하며 로그로 알리는 편이 낫다.
        """
        self._log_hub.publish({"ts": _now(), "kind": "status", "text": "Whisper 모델 로딩 중..."})
        stt = WhisperSTT()
        self._log_hub.publish({"ts": _now(), "kind": "status", "text": "준비 완료 — 듣는 중"})

        if self._on_intent:
            # 인텐트 실행은 이 워커 스레드 하나가 큐에서 꺼내 순차로 처리한다.
            # 인식마다 새 스레드를 띄우면(예전 방식) "토마토"와 "앞으로 가"가
            # 겹쳐 인식됐을 때 두 스레드가 동시에 같은 시리얼 포트에 써서
            # 명령이 깨지는 문제가 있었다(2026-07-08 실측 — 에러 없이 조용히
            # 무시됨). 큐 하나로 직렬화하면 마이크 읽기는 안 막으면서도
            # 하드웨어 명령은 겹치지 않는다.
            threading.Thread(target=self._intent_worker, daemon=True).start()

        segmenter = SpeechSegmenter()
        while True:
            try:
                self._listen_until_error(stt, segmenter)
            except MicStreamError as exc:
                self._mic_state("down")
                self._log_hub.publish(
                    {"ts": _now(), "kind": "error", "text": f"마이크 끊김: {exc} — {RECONNECT_BACKOFF_SEC:.0f}초 후 재연결"}
                )
                time.sleep(RECONNECT_BACKOFF_SEC)

    def _listen_until_error(self, stt: WhisperSTT, segmenter: SpeechSegmenter) -> None:
        mic = MicStream()
        self._mic_state("ok")
        self._log_hub.publish({"ts": _now(), "kind": "status", "text": f"마이크 연결됨({mic.device})"})
        try:
            for chunk in mic.chunks():
                utterance, _level, started = segmenter.feed(chunk)
                if started:
                    self._log_hub.publish({"ts": _now(), "kind": "heard", "text": "(발화 감지...)"})
                if utterance is None:
                    continue

                text, info = stt.transcribe_debug(utterance)
                if not text:
                    # 실패는 원인을 눈으로 가릴 수 있게 근거를 같이 남긴다:
                    # gated에 뭔가 있으면 오디오는 들어왔고 no_speech 게이트가 막은 것,
                    # 비어 있으면 입력 자체가 음성이 아니다(마이크/게인/거리 문제).
                    gated = info.get("gated") or ""
                    detail = (f"{info['sec']}s peak={info['peak']} rms={info['rms']}"
                              + (f" 게이트막음='{gated}'" if gated else " 오디오에 음성 없음"))
                    path = _dump_wav(utterance)
                    self._log_hub.publish({
                        "ts": _now(), "kind": "heard",
                        "text": f"(인식 실패) {detail}" + (f" → {path}" if path else ""),
                    })
                    continue

                intent = match_intent(text)
                kind = "intent" if intent else "heard"
                label = f"{text}" + (f"  → 인텐트: {intent}" if intent else "")
                self._log_hub.publish({"ts": _now(), "kind": kind, "text": label})

                if intent and self._on_intent:
                    self._intent_queue.put(intent)
        finally:
            mic.close()

    def _intent_worker(self) -> None:
        """인텐트를 큐에서 하나씩 꺼내 순차 실행 — 하드웨어 명령이 겹치지 않게."""
        assert self._on_intent is not None
        while True:
            intent = self._intent_queue.get()
            try:
                self._on_intent(intent)
            except Exception as exc:  # noqa: BLE001 - 워커가 죽으면 이후 명령이 전부 무시됨
                self._log_hub.publish({"ts": _now(), "kind": "error", "text": f"'{intent}' 처리 중 오류: {exc}"})
