"""바닥 CSI 카메라 프레임에서 **검은 라인**을 검출해 /dev/shm에 결과를 쓴다.

파이프라인 (기존 tomato_vision과 같은 /dev/shm 릴레이 패턴):

    line-cam.service (gst, nvarguscamerasrc)  →  /dev/shm/line_cam.jpg   (원본)
    line-follow.service (이 파일)             →  /dev/shm/line_view.jpg  (검출 오버레이)
                                              →  /dev/shm/line_status    (JSON)
    tomato-voice (대시보드)                    →  화면에 영상 + 상태 배지

왜 gst가 파일로 넘겨주나 — 이 젯슨의 vision venv OpenCV는 **GStreamer 지원 없이**
빌드돼 있어(`cv2.getBuildInformation()` → `GStreamer: NO`) `cv2.VideoCapture`로
nvarguscamerasrc를 직접 열 수 없다. V4L2로 CSI를 직접 열면 Bayer raw가 나오고
ISP(디베이어·화이트밸런스)를 거치지 않아 쓸 수 없다. 그래서 ISP를 태우는 일은
gst에 맡기고 여기서는 JPEG만 받아 처리한다. 지연은 한 프레임(~30ms) 수준.

⚠ 카메라가 **전면 우측 바퀴 앞**에 달려 차체 중심이 아니다. 따라서 "라인이 화면
중앙"이 곧 "로봇이 라인 위"가 아니다. LF_CENTER_X로 그 오프셋을 교정한다
(로봇을 라인 정중앙에 손으로 놓고, 그때 검출된 x를 이 값으로 넣으면 된다).
"""

from __future__ import annotations

import json
import os
import time

import cv2
import numpy as np

SRC = os.environ.get("LF_SRC", "/dev/shm/line_cam.jpg")
VIEW = os.environ.get("LF_VIEW", "/dev/shm/line_view.jpg")
STATUS = os.environ.get("LF_STATUS", "/dev/shm/line_status")
FPS = float(os.environ.get("LF_FPS", "15"))
JPEG_QUALITY = int(os.environ.get("LF_JPEG_QUALITY", "70"))

# 관심영역(ROI): 화면 세로 비율. 아래쪽(발밑)일수록 즉각적이고, 위쪽일수록 선행시야.
ROI_TOP = float(os.environ.get("LF_ROI_TOP", "0.50"))
ROI_BOT = float(os.environ.get("LF_ROI_BOT", "0.98"))
# 로봇이 라인 정중앙일 때 라인이 찍히는 x좌표(px). 미설정이면 화면 중앙.
CENTER_X = os.environ.get("LF_CENTER_X")
# 이 픽셀 수보다 어두운 점이 적으면 "라인 없음"으로 본다(노이즈 방지).
MIN_PIX = int(os.environ.get("LF_MIN_PIX", "150"))
# --- 유령 라인 방지 (2026-08-09 실측) ---
# Otsu는 "밝기를 두 무리로 가르는" 알고리즘이라 화면에 라인이 하나도 없어도
# 마루 나뭇결을 반으로 갈라 없는 라인을 만들어낸다(실제로 offset -563px 오검출).
# 그래서 "검은 테이프"의 물리적 특징 세 가지를 통과해야 라인으로 인정한다.
#  ① 대비: 어두운 무리와 밝은 무리의 평균 밝기 차. 진짜 테이프면 크게 벌어진다.
MIN_CONTRAST = float(os.environ.get("LF_MIN_CONTRAST", "45"))
#  ② 폭: 테이프는 화면 폭의 일부만 차지한다(너무 좁으면 노이즈, 넓으면 그림자).
MIN_W_FRAC = float(os.environ.get("LF_MIN_W_FRAC", "0.02"))
MAX_W_FRAC = float(os.environ.get("LF_MAX_W_FRAC", "0.35"))
#  ③ 면적: ROI의 이 비율을 넘게 검으면 라인이 아니라 그늘/장애물이다.
MAX_AREA_FRAC = float(os.environ.get("LF_MAX_AREA_FRAC", "0.45"))
# 가로 정지마크 판정: 하단 밴드의 가로 폭 중 이 비율 이상이 검으면 마크.
MARK_FRAC = float(os.environ.get("LF_MARK_FRAC", "0.55"))
# 고정 임계값(0~255). 미설정이면 Otsu 자동 임계.
THRESH = os.environ.get("LF_THRESH")


def _atomic_write(path: str, data: bytes) -> None:
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def _read_frame() -> np.ndarray | None:
    """원본 JPEG를 읽어 디코드. gst multifilesink는 원자적 교체가 아니라 쓰는
    도중을 읽으면 찢어진 JPEG가 나온다 — 그 프레임은 조용히 건너뛴다."""
    try:
        with open(SRC, "rb") as f:
            data = f.read()
    except OSError:
        return None
    if not data:
        return None
    return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)


def _binarize(gray: np.ndarray) -> tuple[np.ndarray, float]:
    """어두운 라인 = 255인 이진 영상과, 두 무리의 밝기 **대비**를 함께 돌려준다.

    Otsu는 장면 밝기가 바뀌어도 따라가므로 조명이 변하는 시연장에 맞다. 대신
    "가를 게 없어도 가른다"는 성질이 있어 대비를 같이 재서 호출부가 판단한다.
    """
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    if THRESH:
        level = float(THRESH)
        _, binary = cv2.threshold(blur, int(level), 255, cv2.THRESH_BINARY_INV)
    else:
        level, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((5, 5), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    dark = blur[binary > 0]
    bright = blur[binary == 0]
    if dark.size == 0 or bright.size == 0:
        return binary, 0.0
    return binary, float(bright.mean() - dark.mean())


def _row_center(binary: np.ndarray, y0: int, y1: int, frame_w: int) -> tuple[float | None, int]:
    """[y0,y1) 밴드에서 라인 중심 x와 검은 픽셀 수.

    가장 큰 연결 덩어리의 무게중심을 쓴다 — 단순 평균은 화면 구석의 그림자나
    전선 같은 다른 어두운 것이 섞이면 중심이 통째로 끌려간다. 그리고 그 덩어리가
    **테이프처럼 생겼는지**(폭·면적) 확인해 유령 라인을 버린다.
    """
    band = binary[y0:y1]
    count = int(cv2.countNonZero(band))
    if count < MIN_PIX or band.size == 0:
        return None, count
    if count > band.size * MAX_AREA_FRAC:
        return None, count      # ROI 절반 이상이 검다 = 그늘/장애물이지 라인이 아니다
    num, _labels, stats, centroids = cv2.connectedComponentsWithStats(band, connectivity=8)
    if num <= 1:
        return None, count
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if int(stats[largest, cv2.CC_STAT_AREA]) < MIN_PIX:
        return None, count
    width = int(stats[largest, cv2.CC_STAT_WIDTH])
    if not (frame_w * MIN_W_FRAC <= width <= frame_w * MAX_W_FRAC):
        return None, count      # 테이프 폭이 아니다
    return float(centroids[largest][0]), count


def _annotate(frame, roi_y0, roi_y1, center_x, near, far, mark, offset, contrast) -> np.ndarray:
    view = frame.copy()
    h, w = view.shape[:2]
    cv2.rectangle(view, (0, roi_y0), (w - 1, roi_y1), (255, 200, 0), 2)
    # 기준선(로봇 중심) — 라인을 여기에 맞추는 게 목표
    cv2.line(view, (int(center_x), roi_y0), (int(center_x), roi_y1), (0, 255, 255), 2)
    for cx, cy, color in (
        (near, int(roi_y1 - (roi_y1 - roi_y0) * 0.15), (0, 255, 0)),
        (far, int(roi_y0 + (roi_y1 - roi_y0) * 0.15), (0, 160, 255)),
    ):
        if cx is not None:
            cv2.circle(view, (int(cx), cy), 10, color, -1)
    if near is not None:
        cv2.line(view, (int(center_x), roi_y1 - 4), (int(near), roi_y1 - 4), (0, 255, 0), 3)
    label = f"offset {offset:+.0f}px" if offset is not None else "LINE LOST"
    color = (0, 255, 0) if offset is not None else (0, 0, 255)
    cv2.putText(view, label, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
    # 대비를 항상 표시 — 라인이 안 잡힐 때 "테이프가 흐린 건지 ROI가 빗나간
    # 건지"를 화면만 보고 판단할 수 있다(임계값은 LF_MIN_CONTRAST).
    cv2.putText(view, f"contrast {contrast:.0f}/{MIN_CONTRAST:.0f}", (12, h - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 220, 0), 2)
    if mark:
        cv2.putText(view, "STOP MARK", (12, 74), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    return view


def main() -> None:
    print(f"[line_follow] {SRC} → {VIEW} / {STATUS}", flush=True)
    period = 1.0 / max(1.0, FPS)
    last_shape = None
    while True:
        start = time.monotonic()
        frame = _read_frame()
        if frame is None:
            time.sleep(0.2)
            continue

        h, w = frame.shape[:2]
        if (h, w) != last_shape:
            last_shape = (h, w)
            print(f"[line_follow] 프레임 {w}x{h}", flush=True)
        roi_y0, roi_y1 = int(h * ROI_TOP), int(h * ROI_BOT)
        center_x = float(CENTER_X) if CENTER_X else w / 2.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        binary, contrast = _binarize(gray)
        # 대비가 약하면 검출 자체를 포기한다 — 라인 없는 마루에서 Otsu가
        # 나뭇결을 갈라 만든 유령 라인이 여기서 전부 걸린다.
        if contrast < MIN_CONTRAST:
            binary = np.zeros_like(binary)
        roi = binary[roi_y0:roi_y1]
        band = roi_y1 - roi_y0

        # 가까운 밴드(즉각 조향) / 먼 밴드(선행 = 진행각 추정)
        near_x, near_n = _row_center(binary, roi_y1 - int(band * 0.30), roi_y1, w)
        far_x, _ = _row_center(binary, roi_y0, roi_y0 + int(band * 0.30), w)

        offset = None if near_x is None else near_x - center_x
        angle = None
        if near_x is not None and far_x is not None:
            # 화면 위쪽이 멀리 — 두 점을 이은 선의 기울기(도). +면 오른쪽으로 휘는 중.
            dy = band * 0.70
            angle = float(np.degrees(np.arctan2(far_x - near_x, dy)))

        # 정지마크: 하단 밴드에서 한 행의 검은 폭이 화면 폭의 MARK_FRAC 이상.
        # 주행 라인은 가늘어서 절대 이 폭이 안 나온다 — 가로 마크만 걸린다.
        mark = False
        if roi.size:
            widest = int(roi.sum(axis=1).max() / 255)
            mark = widest >= w * MARK_FRAC

        status = {
            "ts": time.time(),
            "found": near_x is not None,
            "offset_px": None if offset is None else round(offset, 1),
            # -1..+1 정규화 — 조향 게인을 해상도와 무관하게 만든다
            "offset_norm": None if offset is None else round(offset / (w / 2.0), 3),
            "angle_deg": None if angle is None else round(angle, 1),
            "mark": bool(mark),
            "center_x": round(center_x, 1),
            "pixels": near_n,
            "width": w,
            # 튜닝용 — 검출이 안 될 때 "테이프 대비가 부족한지"를 숫자로 본다.
            "contrast": round(contrast, 1),
        }
        _atomic_write(STATUS, json.dumps(status).encode("utf-8"))

        view = _annotate(frame, roi_y0, roi_y1, center_x, near_x, far_x, mark, offset, contrast)
        ok, buf = cv2.imencode(".jpg", view, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok:
            _atomic_write(VIEW, buf.tobytes())

        slack = period - (time.monotonic() - start)
        if slack > 0:
            time.sleep(slack)


if __name__ == "__main__":
    main()
