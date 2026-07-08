"""arecord 서브프로세스로 마이크를 스트리밍 캡처.

pyaudio/sounddevice 같은 네이티브 바인딩 없이, 이미 동작 확인된 `arecord`를
그대로 파이프로 읽는다. 젯슨에 이미 있는 도구라 의존성 추가가 없다.

카드 번호와 이름은 재부팅/USB 재연결/마이크 교체마다 바뀌는 게 실측으로
확인됐다(2026-07-08: 카메라 마이크가 card 2→0, 이후 웹캠으로 교체하니
이름도 "Camera"→"webcam"). 그래서 특정 이름을 찾지 않고 젯슨 내장
오디오(APE)가 아닌 첫 카드를 자동으로 쓴다.

⚠ 원래 쓰던 카메라(2ce5:c672)는 `plughw`(ALSA plug 리샘플 레이어)로 열면
완전 무음이 잡혔다(2026-07-08 실측) — 그 카메라의 오디오 코덱이 광고한
48kHz와 실제 클럭이 안 맞아(dmesg: "current rate 33186 is different from
the runtime rate 48000") plug의 리샘플러가 깨지는 것으로 보임. `hw`(리샘플
없는 raw)로 열고 광고 레이트 그대로 받은 뒤 파이썬에서 다운샘플하면 정상
캡처됐다. Windows에서는 같은 마이크가 멀쩡히 동작해 하드웨어 결함이
아니라 리눅스 usb-audio 드라이버 쪽 클럭/리샘플 이슈로 판단. 그래서 항상
hw로 열도록 해뒀다 — 다음에 어떤 마이크를 꽂아도 안전한 경로.
"""

from __future__ import annotations

import re
import subprocess
from collections.abc import Iterator

import numpy as np

from ..config import MIC_ALSA_DEVICE_FALLBACK, MIC_ALSA_EXCLUDE_NAME, MIC_NATIVE_SAMPLE_RATE, MIC_SAMPLE_RATE

CHUNK_SECONDS = 0.1
BYTES_PER_SAMPLE = 2  # S16_LE


class MicStreamError(RuntimeError):
    """arecord가 의도치 않게 죽었을 때(USB I/O 오류 등)."""


def resolve_alsa_device(exclude_name: str = MIC_ALSA_EXCLUDE_NAME) -> str:
    """`arecord -l`에서 exclude_name(젯슨 내장 오디오)이 아닌 첫 카드를 raw hw 문자열로.

    plughw가 아니라 hw인 이유는 위 모듈 docstring 참고.
    """
    try:
        out = subprocess.run(["arecord", "-l"], capture_output=True, text=True, timeout=5).stdout
    except (subprocess.SubprocessError, FileNotFoundError):
        return MIC_ALSA_DEVICE_FALLBACK
    for line in out.splitlines():
        m = re.match(r"card (\d+): (.*)", line)
        if m and exclude_name not in m.group(2):
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
