"""별도 GPU 비전 프로세스(tools/tomato_vision.py)가 /dev/shm에 쓴 주석 프레임·개수를
읽어 대시보드에 공급한다. 이 클래스는 파일만 읽으므로 torch/ultralytics가 필요 없어
음성 서비스 venv(~/lerobot/.venv)를 그대로 쓴다. server.py의 /video가 latest_jpeg()를
호출하고, 개수 변화는 log_hub(SSE)로 발행한다.
"""

from __future__ import annotations

import json
import threading
import time

import cv2
import numpy as np

from .log_hub import LogHub


# 좌표 지터로 갱신이 도배되지 않게 이 픽셀 격자로 반올림해서 변화를 판단한다.
POS_QUANT = 16
# 변화가 없어도 이 주기로는 재발행해 배너가 낡은 값에 머무르지 않게 한다.
HEARTBEAT_SEC = 5.0


def _now() -> str:
    return time.strftime("%H:%M:%S")


def _placeholder() -> bytes:
    img = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.putText(img, "vision starting...", (150, 190),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
    ok, buf = cv2.imencode(".jpg", img)
    return buf.tobytes() if ok else b""


class SharedFrameSource:
    """/dev/shm의 최신 JPEG를 /video로 흘려보내고, 개수 파일 변화를 log_hub로 발행."""

    def __init__(self, log_hub: LogHub, jpeg_path: str, status_path: str,
                 poll_sec: float = 0.4, prefer_path: str | None = None) -> None:
        self._log_hub = log_hub
        self._jpeg_path = jpeg_path
        self._status_path = status_path
        self._prefer_path = prefer_path
        self._poll = poll_sec
        self._placeholder = _placeholder()
        self._stop = False

    def start(self) -> None:
        threading.Thread(target=self._count_loop, daemon=True).start()

    def start_line_publisher(self) -> None:
        """라인 검출 상태를 대시보드로 발행(바닥 카메라 전용).

        토마토 개수와 같은 latest_only 채널을 쓰므로 로그를 도배하지 않고
        상단 배지만 갈아끼운다.
        """
        threading.Thread(target=self._line_loop, daemon=True).start()

    def _line_loop(self) -> None:
        last_key = None
        last_sent = 0.0
        while not self._stop:
            st = self.latest_raw_status()
            if st is not None:
                found = bool(st.get("found"))
                mark = bool(st.get("mark"))
                offset = st.get("offset_px")
                # 픽셀 지터로 도배되지 않게 10px 격자로 반올림해 변화를 판단한다.
                key = (found, mark, None if offset is None else round(offset / 10))
                now = time.monotonic()
                if key != last_key or now - last_sent >= HEARTBEAT_SEC:
                    last_key, last_sent = key, now
                    self._log_hub.publish({
                        "ts": _now(), "kind": "line",
                        "found": found, "mark": mark,
                        "offset_px": offset, "angle_deg": st.get("angle_deg"),
                    }, latest_only=True)
            time.sleep(self._poll)

    def stop(self) -> None:
        self._stop = True

    def latest_raw_status(self) -> dict | None:
        """상태 파일을 그대로 파싱해 돌려준다(스키마 해석 없이).

        토마토 상태는 latest_status()가 정규화해서 쓰지만, 라인 검출 상태처럼
        필드가 다른 소비자도 같은 파일 릴레이를 쓰기 때문에 원본 접근이 필요하다.
        """
        try:
            with open(self._status_path, encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    def latest_status(self) -> dict:
        """비전이 마지막으로 쓴 상태 {"air","fallen","positions"}.

        수동조작 화면의 "감지된 토마토 높이로 팔 이동"이 여기서 y좌표를 가져간다
        (SSE는 브라우저로만 흐르므로 서버가 다시 파일을 읽는다 — 파일이 곧 진실).
        """
        try:
            with open(self._status_path, encoding="utf-8") as f:
                st = json.load(f)
        except (OSError, ValueError):
            return {"air": 0, "fallen": 0, "positions": []}
        return {
            "air": int(st.get("air", 0)),
            "fallen": int(st.get("fallen", 0)),
            "positions": st.get("positions", []),
        }

    def latest_jpeg(self) -> bytes:
        """최신 JPEG. prefer_path(검출 오버레이)가 있으면 그걸 우선 쓴다.

        오버레이 프로세스가 아직 안 떴거나 죽어도 원본 프레임으로 자동 폴백 —
        화면이 검게 죽는 것보다 "검출선 없는 영상"이 낫다.
        """
        for path in (self._prefer_path, self._jpeg_path):
            if not path:
                continue
            try:
                with open(path, "rb") as f:
                    data = f.read()
            except OSError:
                continue
            if data:
                return data
        return self._placeholder

    def _count_loop(self) -> None:
        """개수·낙과·위치 중 하나라도 바뀌면 발행(+HEARTBEAT_SEC마다 재발행).

        예전엔 공중 개수가 바뀔 때만 발행해서, 개수가 그대로면 낙과 수와 좌표가
        옛 값에 고정됐다(영상 오버레이와 배너가 어긋나던 원인). 좌표는 픽셀 지터로
        매 프레임 흔들리므로 POS_QUANT 격자로 반올림해 비교한다.
        """
        last_key = None
        last_sent = 0.0
        while not self._stop:
            try:
                with open(self._status_path, encoding="utf-8") as f:
                    st = json.load(f)
                air = int(st.get("air", 0))
                fallen = int(st.get("fallen", 0))
                positions = st.get("positions", [])
                key = (air, fallen, tuple(
                    (round(x / POS_QUANT), round(y / POS_QUANT)) for x, y in positions))
                now = time.monotonic()
                if key != last_key or now - last_sent >= HEARTBEAT_SEC:
                    last_key, last_sent = key, now
                    pos_str = ", ".join(f"({x},{y})" for x, y in positions) or "-"
                    self._log_hub.publish({
                        "ts": _now(), "kind": "count",
                        "count": air, "fallen": fallen, "positions": positions,
                        "text": f"공중 토마토 {air}개 (낙과 {fallen}) 위치: {pos_str}",
                    }, latest_only=True)
            except (OSError, ValueError, json.JSONDecodeError):
                pass
            time.sleep(self._poll)
