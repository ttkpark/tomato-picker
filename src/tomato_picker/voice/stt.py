"""Whisper(faster-whisper) 기반 한국어 STT — 온디바이스, 젯슨 CPU(int8)."""

from __future__ import annotations

import numpy as np
from faster_whisper import WhisperModel

from ..config import (
    MIC_SAMPLE_RATE,
    WHISPER_COMPUTE_TYPE,
    WHISPER_DEVICE,
    WHISPER_LANGUAGE,
    WHISPER_MODEL_SIZE,
    WHISPER_NORMALIZE_GAIN_CAP,
    WHISPER_NORMALIZE_MIN_PEAK,
    WHISPER_NORMALIZE_TARGET_PEAK,
)

assert MIC_SAMPLE_RATE == 16000, "faster-whisper는 16kHz 입력을 전제로 한다."


def _normalize(audio: np.ndarray) -> np.ndarray:
    """피크를 WHISPER_NORMALIZE_TARGET_PEAK까지 끌어올린다.

    거의 무음(peak가 MIN 미만)이면 잡음만 증폭될 뿐이라 건드리지 않는다.
    """
    peak = float(np.max(np.abs(audio)))
    if peak < WHISPER_NORMALIZE_MIN_PEAK:
        return audio
    gain = min(WHISPER_NORMALIZE_TARGET_PEAK / peak, WHISPER_NORMALIZE_GAIN_CAP)
    return audio * gain


class WhisperSTT:
    """모델을 한 번 로드해두고 재사용(로드 자체가 수십 초라 매번 하면 안 됨)."""

    def __init__(
        self,
        model_size: str = WHISPER_MODEL_SIZE,
        device: str = WHISPER_DEVICE,
        compute_type: str = WHISPER_COMPUTE_TYPE,
        language: str = WHISPER_LANGUAGE,
    ) -> None:
        self._model = WhisperModel(model_size, device=device, compute_type=compute_type)
        self._language = language

    def transcribe(self, utterance_int16: np.ndarray) -> str:
        audio = _normalize(utterance_int16.astype(np.float32) / 32768.0)
        segments, _info = self._model.transcribe(audio, language=self._language, beam_size=1)
        return "".join(seg.text for seg in segments).strip()
