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
        # temperature fallback(품질 임계 미달 시 온도 올려 재디코딩)을 끈다.
        # 애매한 오디오에서 재시도가 5~6회 반복돼 지연이 1초→3~12초로 튀는 게
        # 실측(2026-07-21)으로 확인됨. 짧은 명령어 인식엔 1차 디코딩이면 충분.
        # temperature=0.0(스칼라)이면 fallback 온도 리스트가 하나뿐이라
        # compression_ratio/log_prob 임계가 걸려도 재디코딩이 안 일어난다.
        # no_speech_threshold는 절대 끄지 마라 — 이게 무음/노이즈 구간의
        # Whisper 환각("이 영상은" 등)을 걸러주는 유일한 게이트다(2026-07-21
        # 실측: 끄면 노이즈에 환각, 켜면 ''로 억제. 발화 지연엔 영향 없음).
        segments, _info = self._model.transcribe(
            audio,
            language=self._language,
            beam_size=1,
            temperature=0.0,
            compression_ratio_threshold=None,
            log_prob_threshold=None,
        )
        return "".join(seg.text for seg in segments).strip()
