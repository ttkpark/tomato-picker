"""arecord 서브프로세스로 마이크를 스트리밍 캡처.

pyaudio/sounddevice 같은 네이티브 바인딩 없이, 이미 동작 확인된 `arecord`를
그대로 파이프로 읽는다. 젯슨에 이미 있는 도구라 의존성 추가가 없다.

카드 번호는 재부팅/USB 재연결마다 바뀔 수 있음이 실측으로 확인됐다
(2026-07-08: 카메라 마이크가 card 2 → card 0으로 이동). 그래서 숫자를
고정하지 않고 `arecord -l`에서 카드 이름으로 찾는다.
"""

from __future__ import annotations

import re
import subprocess
from collections.abc import Iterator

import numpy as np

from ..config import MIC_ALSA_CARD_NAME, MIC_ALSA_DEVICE_FALLBACK, MIC_SAMPLE_RATE

CHUNK_SECONDS = 0.1
BYTES_PER_SAMPLE = 2  # S16_LE


class MicStreamError(RuntimeError):
    """arecord가 의도치 않게 죽었을 때(USB I/O 오류 등)."""


def resolve_alsa_device(card_name_hint: str = MIC_ALSA_CARD_NAME) -> str:
    """`arecord -l`에서 이름에 card_name_hint가 들어간 첫 카드를 찾아 plughw 문자열로."""
    try:
        out = subprocess.run(["arecord", "-l"], capture_output=True, text=True, timeout=5).stdout
    except (subprocess.SubprocessError, FileNotFoundError):
        return MIC_ALSA_DEVICE_FALLBACK
    for line in out.splitlines():
        m = re.match(r"card (\d+): .*" + re.escape(card_name_hint), line)
        if m:
            return f"plughw:{m.group(1)},0"
    return MIC_ALSA_DEVICE_FALLBACK


class MicStream:
    """`arecord`로 raw PCM(S16LE, mono)을 CHUNK_SECONDS 단위 청크로 흘려준다."""

    def __init__(self, device: str | None = None, sample_rate: int = MIC_SAMPLE_RATE) -> None:
        self.device = device or resolve_alsa_device()
        self._sample_rate = sample_rate
        self._chunk_bytes = int(sample_rate * CHUNK_SECONDS) * BYTES_PER_SAMPLE
        self._stopping = False
        self._proc = subprocess.Popen(
            [
                "arecord",
                "-D", self.device,
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
        """CHUNK_SECONDS 길이의 int16 모노 배열을 계속 yield.

        의도적으로 close()된 게 아닌데 스트림이 끊기면(USB I/O 오류 등)
        MicStreamError를 던진다 — 호출 측이 재연결할지 판단하게.
        """
        assert self._proc.stdout is not None
        while True:
            raw = self._proc.stdout.read(self._chunk_bytes)
            if len(raw) < self._chunk_bytes:
                if self._stopping:
                    return
                raise MicStreamError(f"arecord({self.device}) 스트림 끊김 — USB 마이크 I/O 오류 가능성")
            yield np.frombuffer(raw, dtype=np.int16)

    def close(self) -> None:
        self._stopping = True
        self._proc.terminate()
        try:
            self._proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._proc.kill()
