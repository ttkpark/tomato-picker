"""에너지(RMS) 기반 발화구간 검출.

silero-vad 같은 학습 모델 없이 임계값 하나로 시작한다 — 조용한 실내
데모 환경에는 충분하고, 로그에 rms가 그대로 찍히니 소음이 있으면
config.VAD_RMS_THRESHOLD만 조정하면 된다.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from ..config import (
    VAD_MAX_UTTERANCE_SEC,
    VAD_MIN_SPEECH_SEC,
    VAD_PRE_ROLL_SEC,
    VAD_RMS_THRESHOLD,
    VAD_SILENCE_HANGOVER_SEC,
)
from .mic_stream import CHUNK_SECONDS


def rms(chunk: np.ndarray) -> float:
    return float(np.sqrt(np.mean(chunk.astype(np.float64) ** 2)))


class SpeechSegmenter:
    """chunk를 하나씩 먹여 완결된 발화 구간(int16 ndarray)을 돌려받는다."""

    def __init__(
        self,
        threshold: float = VAD_RMS_THRESHOLD,
        min_speech_sec: float = VAD_MIN_SPEECH_SEC,
        silence_hangover_sec: float = VAD_SILENCE_HANGOVER_SEC,
        max_utterance_sec: float = VAD_MAX_UTTERANCE_SEC,
        pre_roll_sec: float = VAD_PRE_ROLL_SEC,
    ) -> None:
        self._threshold = threshold
        self._min_speech_chunks = max(1, int(min_speech_sec / CHUNK_SECONDS))
        self._silence_hangover_chunks = max(1, int(silence_hangover_sec / CHUNK_SECONDS))
        self._max_chunks = max(1, int(max_utterance_sec / CHUNK_SECONDS))
        pre_roll_chunks = max(0, int(pre_roll_sec / CHUNK_SECONDS))

        self._in_speech = False
        self._buffer: list[np.ndarray] = []
        self._silence_run = 0
        # 임계값을 넘기 직전 몇 청크를 항상 들고 있다가, 발화 시작 시 앞에
        # 붙인다 — "팔"처럼 파열음으로 시작하는 단어는 VAD가 트리거되는
        # 순간 이미 어택 일부가 지나가 있어 초성이 잘리는 경우가 있다
        # (2026-07-08 실측: "팔"이 "파는"/"잘"/"타는"으로 자주 오인식).
        self._pre_roll: deque[np.ndarray] = deque(maxlen=pre_roll_chunks) if pre_roll_chunks else deque()

    def feed(self, chunk: np.ndarray) -> tuple[np.ndarray | None, float, bool]:
        """chunk 하나 처리.

        반환: (완결된 발화 or None, 이번 chunk의 rms, 발화 시작 시점이었는지)
        """
        level = rms(chunk)
        speaking = level >= self._threshold
        started = False

        if not self._in_speech:
            if speaking:
                self._in_speech = True
                self._buffer = list(self._pre_roll) + [chunk]
                self._pre_roll.clear()
                self._silence_run = 0
                started = True
            else:
                self._pre_roll.append(chunk)
            return None, level, started

        self._buffer.append(chunk)
        self._silence_run = 0 if speaking else self._silence_run + 1

        ended_by_silence = self._silence_run >= self._silence_hangover_chunks
        ended_by_length = len(self._buffer) >= self._max_chunks
        if not (ended_by_silence or ended_by_length):
            return None, level, started

        utterance = np.concatenate(self._buffer) if len(self._buffer) >= self._min_speech_chunks else None
        self._in_speech = False
        self._buffer = []
        self._silence_run = 0
        return utterance, level, started
