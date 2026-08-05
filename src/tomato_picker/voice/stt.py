"""Whisper(faster-whisper) 기반 한국어 STT — 온디바이스, 젯슨 CPU(int8)."""

from __future__ import annotations

import re

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


def _squash_repeats(text: str) -> str:
    """디코더 반복 루프 결과("ㅋㅋㅋ…" 수백 자)를 3회로 줄인다.

    temperature fallback을 꺼둔 탓에 compression_ratio 임계가 안 걸려서
    반복이 그대로 나온다(2026-07-30 실측). 인텐트 매칭엔 무해하지만 대시보드
    로그를 한 줄로 도배하므로 표시 전에 접는다.
    """
    # 한 글자("ㅋㅋㅋ…")뿐 아니라 "아, 아, 아, …"처럼 짧은 토막 반복도 접는다.
    return re.sub(r"(.{1,5}?)\1{3,}", r"\1\1\1", text)


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
        return self.transcribe_debug(utterance_int16)[0]

    def transcribe_debug(self, utterance_int16: np.ndarray) -> tuple[str, dict]:
        """(텍스트, 진단정보)를 돌려준다.

        빈 문자열이 나왔을 때 원인을 눈으로 가릴 수 있게, no_speech 게이트를 끈
        2차 디코딩을 한 번 더 돌려 '게이트가 막은 가설'을 진단정보에 담는다.
        게이트가 뭔가를 막았다면 마이크는 살아있고 문턱/발음 문제, 2차 디코딩도
        비었다면 입력 오디오 자체가 음성이 아니다(마이크·게인 문제).
        재디코딩은 실패한 발화에서만 일어나므로 평소 지연에는 영향이 없다.
        """
        peak_raw = float(np.max(np.abs(utterance_int16))) / 32768.0
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
        segs = list(segments)
        text = _squash_repeats("".join(seg.text for seg in segs).strip())
        info = {
            "sec": round(len(utterance_int16) / MIC_SAMPLE_RATE, 2),
            "peak": round(peak_raw, 3),
            "rms": int(np.sqrt(np.mean(utterance_int16.astype(np.float64) ** 2))),
            "normalized": peak_raw >= WHISPER_NORMALIZE_MIN_PEAK,
        }
        if segs:
            info["no_speech"] = round(min(s.no_speech_prob for s in segs), 3)
        if not text:
            info["gated"] = self._raw_hypothesis(audio)
        return text, info

    def _raw_hypothesis(self, audio: np.ndarray) -> str:
        """no_speech 게이트를 끄고 다시 디코딩 — 게이트가 무엇을 막았는지 본다."""
        segments, _info = self._model.transcribe(
            audio,
            language=self._language,
            beam_size=1,
            temperature=0.0,
            compression_ratio_threshold=None,
            log_prob_threshold=None,
            no_speech_threshold=1.0,
        )
        return "".join(seg.text for seg in segments).strip()
