"""arecord 서브프로세스로 마이크를 스트리밍 캡처.

pyaudio/sounddevice 같은 네이티브 바인딩 없이, 이미 동작 확인된 `arecord`를
그대로 파이프로 읽는다. 젯슨에 이미 있는 도구라 의존성 추가가 없다.

카드 번호는 재부팅/USB 재연결마다 바뀔 수 있음이 실측으로 확인됐다
(2026-07-08: 카메라 마이크가 card 2 → card 0으로 이동). 그래서 숫자를
고정하지 않고 `arecord -l`에서 카드 이름으로 찾는다.

⚠ `plughw`(ALSA plug 리샘플 레이어)로 열면 이 카메라는 완전 무음이 잡힌다
(2026-07-08 실측) — 이 카메라의 오디오 코덱이 광고한 48kHz와 실제 클럭이
안 맞아(dmesg: "current rate 33186 is different from the runtime rate
48000") plug의 리샘플러가 깨지는 것으로 보인다. `hw`(리샘플 없는 raw)로
열고 48kHz 그대로 받은 뒤 파이썬에서 16kHz로 직접 다운샘플하면 정상
캡처된다. Windows에서는 같은 마이크가 멀쩡히 동작해 하드웨어 결함은
아니고 리눅스 usb-audio 드라이버 쪽 클럭/리샘플 이슈로 판단.
"""

from __future__ import annotations

import re
import subprocess
from collections.abc import Iterator

import numpy as np

from ..config import MIC_ALSA_CARD_NAME, MIC_ALSA_DEVICE_FALLBACK, MIC_NATIVE_SAMPLE_RATE, MIC_SAMPLE_RATE

CHUNK_SECONDS = 0.1
BYTES_PER_SAMPLE = 2  # S16_LE


class MicStreamError(RuntimeError):
    """arecord가 의도치 않게 죽었을 때(USB I/O 오류 등)."""


def resolve_alsa_device(card_name_hint: str = MIC_ALSA_CARD_NAME) -> str:
    """`arecord -l`에서 이름에 card_name_hint가 들어간 첫 카드를 찾아 raw hw 문자열로.

    plughw가 아니라 hw인 이유는 위 모듈 docstring 참고.
    """
    try:
        out = subprocess.run(["arecord", "-l"], capture_output=True, text=True, timeout=5).stdout
    except (subprocess.SubprocessError, FileNotFoundError):
        return MIC_ALSA_DEVICE_FALLBACK
    for line in out.splitlines():
        m = re.match(r"card (\d+): .*" + re.escape(card_name_hint), line)
        if m:
            return f"hw:{m.group(1)},0"
    return MIC_ALSA_DEVICE_FALLBACK


def _downsample(native_chunk: np.ndarray, factor: int) -> np.ndarray:
    """factor개씩 평균 내는 박스필터 다운샘플. 음성 명령 인식 용도로는 충분."""
    usable = (len(native_chunk) // factor) * factor
    return native_chunk[:usable].reshape(-1, factor).mean(axis=1).astype(np.int16)


class MicStream:
    """`arecord`로 raw PCM(S16LE, mono, 네이티브 레이트)을 CHUNK_SECONDS 단위로
    흘려주되, 반환 직전 target_rate(기본 16kHz)로 다운샘플해 돌려준다."""

    def __init__(
        self,
        device: str | None = None,
        native_rate: int = MIC_NATIVE_SAMPLE_RATE,
        target_rate: int = MIC_SAMPLE_RATE,
    ) -> None:
        if native_rate % target_rate != 0:
            raise ValueError("native_rate는 target_rate의 정수배여야 한다(단순 박스필터 다운샘플).")
        self.device = device or resolve_alsa_device()
        self._factor = native_rate // target_rate
        self._native_chunk_bytes = int(native_rate * CHUNK_SECONDS) * BYTES_PER_SAMPLE
        self._stopping = False
        self._proc = subprocess.Popen(
            [
                "arecord",
                "-D", self.device,
                "-f", "S16_LE",
                "-r", str(native_rate),
                "-c", "1",
                "-t", "raw",
                "-q",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )

    def chunks(self) -> Iterator[np.ndarray]:
        """CHUNK_SECONDS 길이의 int16 모노 배열(target_rate 기준)을 계속 yield.

        의도적으로 close()된 게 아닌데 스트림이 끊기면(USB I/O 오류 등)
        MicStreamError를 던진다 — 호출 측이 재연결할지 판단하게.
        """
        assert self._proc.stdout is not None
        while True:
            raw = self._proc.stdout.read(self._native_chunk_bytes)
            if len(raw) < self._native_chunk_bytes:
                if self._stopping:
                    return
                raise MicStreamError(f"arecord({self.device}) 스트림 끊김 — USB 마이크 I/O 오류 가능성")
            yield _downsample(np.frombuffer(raw, dtype=np.int16), self._factor)

    def close(self) -> None:
        self._stopping = True
        self._proc.terminate()
        try:
            self._proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._proc.kill()
