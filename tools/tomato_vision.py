"""온디바이스 GPU 토마토 검출 프로세스 (독립 실행, vision venv 전용).

카메라를 잡아 YOLO-World(오픈보캐뷸러리)로 'tomato'를 검출하고, 박스+개수를
그린 주석 프레임을 /dev/shm에 쓴다. 음성 서비스(다른 venv)는 이 공유 파일을
읽어 대시보드에 영상·개수를 보여준다. 이렇게 프로세스/venv를 분리해 작동 중인
음성·팔 스택(~/lerobot/.venv, CPU torch)을 건드리지 않는다.

self-contained: 프로젝트 config를 임포트하지 않는다(다른 venv라 의존성 충돌 방지).
파라미터는 아래 상수/환경변수로만.
"""

from __future__ import annotations

import glob
import json
import os
import time
from collections import deque

import cv2
import numpy as np
from ultralytics import YOLOWorld
WIDTH = int(os.environ.get("TV_WIDTH", "1280"))
HEIGHT = int(os.environ.get("TV_HEIGHT", "720"))
CONF = float(os.environ.get("TV_CONF", "0.05"))
MODEL = os.environ.get("TV_MODEL", "yolov8s-worldv2.pt")
# 추론 입력 해상도. **기본값(640)에 넣으면 안 된다** — 1280x720 프레임이 절반으로
# 줄면서 열매가 검출 한계 아래로 내려가, 여섯 중 가까운 두 개만 잡히던 원인이었다
# (2026-08-13). 캡처 해상도 그대로 넣으면 6/6. ultralytics는 32의 배수만 받는다.
IMGSZ = max(320, round(int(os.environ.get("TV_IMGSZ", str(WIDTH))) / 32) * 32)
CLASSES = [c.strip() for c in os.environ.get("TV_CLASSES", "tomato").split(",") if c.strip()]
JPEG_PATH = os.environ.get("TV_JPEG", "/dev/shm/tomato_vision.jpg")
COUNT_PATH = os.environ.get("TV_COUNT", "/dev/shm/tomato_count")
STATUS_PATH = os.environ.get("TV_STATUS", "/dev/shm/tomato_status")
# 이 y좌표(px)보다 위(작은 y)면 '공중(부착)', 아래(큰 y)면 '바닥(낙과)'.
# 카메라가 고정이라 장면에 맞게 한 번 교정하면 된다(720p 기준 기본 600).
GROUND_LINE_Y = int(os.environ.get("TV_GROUND_Y", "600"))
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


def _resolve_camera_source() -> int | str:
    """무대용 USB 웹캠을 **안정 경로**로 찾는다.

    CSI 카메라(IMX219, 바닥 라인캠) 추가 후 /dev/video0은 부팅 순서에 따라
    CSI가 차지한다. 숫자 인덱스 0으로 열면 이 서비스가 CSI 노드를 선점해
    라인캠은 물론 Argus 데몬까지 마비시킨다(2026-08-09 실사고 — "Failed to
    create CaptureSession"의 정체가 이거였다). /dev/v4l/by-id/ 심볼릭 링크는
    USB 장치 식별자 기반이라 열거 순서와 무관하게 항상 웹캠만 가리킨다.

    웹캠이 안 보이면 인덱스 0으로 폴백하지 않고 **명확히 실패**한다 —
    systemd가 5초마다 재시작하므로 케이블만 다시 꽂으면 알아서 살아난다.
    """
    dev = os.environ.get("TV_CAMERA_DEV")
    if dev:
        return dev
    idx = os.environ.get("TV_CAMERA_INDEX")  # 수동 디버깅용 명시 오버라이드
    if idx is not None:
        return int(idx)
    cands = sorted(glob.glob("/dev/v4l/by-id/usb-*-video-index0"))
    if not cands:
        raise RuntimeError(
            "USB 웹캠을 못 찾았습니다(/dev/v4l/by-id/usb-* 없음) — 케이블 확인. "
            "CSI 라인캠(video0)은 이 서비스가 잡으면 안 되므로 인덱스 폴백은 안 한다."
        )
    # ⚠ **깊이 카메라의 RGB 노드를 웹캠으로 착각하지 않는다.** 2026-09-01,
    #   Astra Pro를 꽂자 by-id 목록의 **첫 줄**이 바뀌었고(A < I), 이 서비스는
    #   아무 말 없이 카메라를 갈아탔다. 무대를 보던 화면이 다른 곳을 보게
    #   됐는데 에러는 하나도 안 났다 — 무대 학습이 오검출을 무대로 삼는
    #   사고(CLAUDE.md)와 같은 종류의 조용한 고장이다.
    #   그래서 깊이 카메라는 **뒤로 미룬다**. 진짜 웹캠이 있으면 그쪽이 이기고,
    #   없으면 깊이 카메라의 RGB라도 쓰되 **어느 것을 골랐는지 찍는다.**
    depthish = ("astra", "realsense", "orbbec", "depth_camera")
    ranked = sorted(cands, key=lambda p: (any(k in p.lower() for k in depthish), p))
    pick = ranked[0]
    print(f"[tomato_vision] 카메라 선택: {os.path.basename(pick)}"
          + (f" (후보 {len(cands)}개)" if len(cands) > 1 else ""), flush=True)
    if any(k in pick.lower() for k in depthish):
        print("[tomato_vision] ⚠ 이건 깊이 카메라의 RGB 노드다 — 무대용 웹캠이 "
              "안 보여서 대신 쓴다. 의도한 게 아니면 TV_CAMERA_DEV로 지정하라.",
              flush=True)
    return pick


def _open_camera() -> cv2.VideoCapture:
    source = _resolve_camera_source()
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        raise RuntimeError(f"카메라({source})를 열 수 없습니다.")
    for _ in range(12):
        cap.read()
    return cap


def _classify(boxes, width: int) -> tuple[list, list]:
    """각 검출을 공중/바닥으로 분류. (air_list, ground_list) 반환.
    각 항목 = (x1,y1,x2,y2,conf,cx,cy). 중심 cy < GROUND_LINE_Y면 공중."""
    air, ground = [], []
    if boxes is not None:
        for b in boxes:
            x1, y1, x2, y2 = (int(v) for v in b.xyxy[0].tolist())
            conf = float(b.conf[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            item = (x1, y1, x2, y2, conf, cx, cy)
            (air if cy < GROUND_LINE_Y else ground).append(item)
    return air, ground


def _draw(frame: np.ndarray, air: list, ground: list, air_count: int) -> np.ndarray:
    """기준선 + 공중(초록)/바닥(회색) 박스 + 위치 라벨 + 공중 개수."""
    out = frame
    h, w = out.shape[:2]
    cv2.line(out, (0, GROUND_LINE_Y), (w, GROUND_LINE_Y), (0, 140, 255), 2)
    cv2.putText(out, "ground line", (w - 190, GROUND_LINE_Y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)
    for x1, y1, x2, y2, conf, cx, cy in air:
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 0), 3)
        cv2.putText(out, f"air ({cx},{cy})", (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 0), 2)
    for x1, y1, x2, y2, conf, cx, cy in ground:
        cv2.rectangle(out, (x1, y1), (x2, y2), (150, 150, 150), 2)
        cv2.putText(out, f"ground ({cx},{cy})", (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 2)
    label = f"air tomatoes: {air_count}   fallen: {len(ground)}"
    cv2.putText(out, label, (14, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 6)
    cv2.putText(out, label, (14, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (60, 220, 60), 2)
    return out


def main() -> None:
    print(f"[tomato_vision] 모델 로딩({MODEL}), 클래스={CLASSES}, "
          f"conf={CONF}, imgsz={IMGSZ}", flush=True)
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
            res = model.predict(frame, device="cuda", conf=CONF, imgsz=IMGSZ,
                                verbose=False)
            air, ground = _classify(res[0].boxes, frame.shape[1])
            raw = len(air)  # 공중(부착) 토마토만 센다
            now = time.monotonic()
            history.append((now, raw))
            while history and now - history[0][0] > HOLD_SEC:
                history.popleft()
            count = max(r for _, r in history)  # 최근 HOLD_SEC초 최댓값 = 안정적
            positions = [[cx, cy] for *_, cx, cy in air]
            annotated = _draw(frame, air, ground, count)
            ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                _atomic_write(JPEG_PATH, buf.tobytes())
            # 위치는 매 프레임 갱신(상태 파일), 개수는 변할 때만 로그
            _atomic_write(STATUS_PATH, json.dumps(
                {"air": count, "fallen": len(ground), "positions": positions}).encode("utf-8"))
            if count != last_count:
                last_count = count
                _atomic_write(COUNT_PATH, str(count).encode("ascii"))
                print(f"[tomato_vision] 공중 토마토 {count}개, 위치 {positions}", flush=True)
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
