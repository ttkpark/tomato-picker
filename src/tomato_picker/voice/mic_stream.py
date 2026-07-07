"""arecord 서브프로세스로 마이크를 스트리밍 캡처.

pyaudio/sounddevice 같은 네이티브 바인딩 없이, 이미 동작 확인된 `arecord`를
그대로 파이프로 읽는다. 젯슨에 이미 있는 도구라 의존성 추가가 없다.
"""

from __future__ import annotations

import subprocess
from collections.abc import Iterator

import numpy as np

from ..config import MIC_ALSA_DEVICE, MIC_SAMPLE_RATE

CHUNK_SECONDS = 0.1
BYTES_PER_SAMPLE = 2  # S16_LE


class MicStream:
    """`arecord`로 raw PCM(S16LE, mono)을 CHUNK_SECONDS 단위 청크로 흘려준다."""

    def __init__(self, device: str = MIC_ALSA_DEVICE, sample_rate: int = MIC_SAMPLE_RATE) -> None:
        self._sample_rate = sample_rate
        self._chunk_bytes = int(sample_rate * CHUNK_SECONDS) * BYTES_PER_SAMPLE
        self._proc = subprocess.Popen(
            [
                "arecord",
                "-D", device,
                "-f", "S16_LE",
                "-r", str(sample_rate),
                "-c", "1",
                "-t", "raw",
                "-q",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )

    def chunks(self) -> Iterator[np.ndarray]:
        """CHUNK_SECONDS 길이의 int16 모노 배열을 계속 yield. 스트림 끊기면 종료."""
        assert self._proc.stdout is not None
        while True:
            raw = self._proc.stdout.read(self._chunk_bytes)
            if len(raw) < self._chunk_bytes:
                return
            yield np.frombuffer(raw, dtype=np.int16)

    def close(self) -> None:
        self._proc.terminate()
        self._proc.wait(timeout=2.0)
