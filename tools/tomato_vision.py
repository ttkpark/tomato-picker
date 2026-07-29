"""온디바이스 GPU 토마토 검출 프로세스 (독립 실행, vision venv 전용).

카메라를 잡아 YOLO-World(오픈보캐뷸러리)로 'tomato'를 검출하고, 박스+개수를
그린 주석 프레임을 /dev/shm에 쓴다. 음성 서비스(다른 venv)는 이 공유 파일을
읽어 대시보드에 영상·개수를 보여준다. 이렇게 프로세스/venv를 분리해 작동 중인
음성·팔 스택(~/lerobot/.venv, CPU torch)을 건드리지 않는다.

self-contained: 프로젝트 config를 임포트하지 않는다(다른 venv라 의존성 충돌 방지).
파라미터는 아래 상수/환경변수로만.
"""

from __future__ import annotations

import os
import time
from collections import deque

import cv2
import numpy as np
from ultralytics import YOLOWorld

CAMERA_INDEX = int(os.environ.get("TV_CAMERA_INDEX", "0"))
WIDTH = int(os.environ.get("TV_WIDTH", "1280"))
HEIGHT = int(os.environ.get("TV_HEIGHT", "720"))
CONF = float(os.environ.get("TV_CONF", "0.10"))
MODEL = os.environ.get("TV_MODEL", "yolov8s-worldv2.pt")
CLASSES = [c.strip() for c in os.environ.get("TV_CLASSES", "tomato").split(",") if c.strip()]
JPEG_PATH = os.environ.get("TV_JPEG", "/dev/shm/tomato_vision.jpg")
COUNT_PATH = os.environ.get("TV_COUNT", "/dev/shm/tomato_count")
TARGET_FPS = float(os.environ.get("TV_FPS", "8"))
JPEG_QUALITY = int(os.environ.get("TV_JPEG_QUALITY", "70"))
# 검출이 프레임마다 깜빡여도(어두운 조명 등) 개수가 안 튀도록, 최근 HOLD_SEC초
# 안에 잡힌 최댓값을 보고한다(시간 기반 유지). 60초면 1분 안에 본 토마토는 계속
# 개수로 유지되고, 토마토를 치우면 1분 뒤 0으로 내려간다.
HOLD_SEC = float(os.environ.get("TV_HOLD_SEC", "60"))


def _atomic_write(path: str, data: bytes) -> None:
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def _open_camera() -> cv2.VideoCapture:
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        raise RuntimeError(f"카메라(index={CAMERA_INDEX})를 열 수 없습니다.")
    for _ in range(12):
        cap.read()
    return cap


def _annotate(frame: np.ndarray, boxes, count: int) -> np.ndarray:
    """박스는 이번 프레임 raw로, 개수 라벨은 평활된 count로 표시."""
    out = frame
    if boxes is not None:
        for b in boxes:
            x1, y1, x2, y2 = (int(v) for v in b.xyxy[0].tolist())
            conf = float(b.conf[0])
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.putText(out, f"tomato {conf:.2f}", (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    label = f"tomatoes: {count}"
    cv2.putText(out, label, (14, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 0), 6)
    cv2.putText(out, label, (14, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (60, 220, 60), 2)
    return out


def main() -> None:
    print(f"[tomato_vision] 모델 로딩({MODEL}), 클래스={CLASSES}, conf={CONF}", flush=True)
    model = YOLOWorld(MODEL)
    model.set_classes(CLASSES)
    interval = 1.0 / TARGET_FPS
    cap = None
    last_count = -1
    history: deque[tuple[float, int]] = deque()  # (시각, raw개수) — 최근 HOLD_SEC초 유지
    fail = 0
    while True:
        t0 = time.monotonic()
        try:
            if cap is None:
                cap = _open_camera()
                print("[tomato_vision] 카메라 연결됨", flush=True)
            for _ in range(4):
                cap.grab()
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("프레임 읽기 실패")
            fail = 0
            res = model.predict(frame, device="cuda", conf=CONF, verbose=False)
            boxes = res[0].boxes
            raw = 0 if boxes is None else len(boxes)
            now = time.monotonic()
            history.append((now, raw))
            while history and now - history[0][0] > HOLD_SEC:
                history.popleft()
            count = max(r for _, r in history)  # 최근 HOLD_SEC초 최댓값 = 안정적
            annotated = _annotate(frame, boxes, count)
            ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                _atomic_write(JPEG_PATH, buf.tobytes())
            if count != last_count:
                last_count = count
                _atomic_write(COUNT_PATH, str(count).encode("ascii"))
                print(f"[tomato_vision] 토마토 {count}개", flush=True)
        except Exception as exc:  # noqa: BLE001 - 카메라 순단에도 계속 재시도
            fail += 1
            if fail == 1:
                print(f"[tomato_vision] 오류: {exc} — 재시도", flush=True)
            if cap is not None:
                cap.release()
                cap = None
            time.sleep(1.0)
        dt = time.monotonic() - t0
        if dt < interval:
            time.sleep(interval - dt)


if __name__ == "__main__":
    main()
