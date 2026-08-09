"""바닥 CSI 카메라로 **주행 테이프와 마커**를 검출해 /dev/shm에 결과를 쓴다.

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


코스 기하 (2026-08-09 실측 확정 — 이게 이 파일의 전제다)
-----------------------------------------------------------
테이프는 **진행 방향과 나란히** 깔려 있고(무대 앞을 좌우로 가로지름), 로봇은 그
위를 게걸음으로 오간다. 그래서 바닥 카메라에는 테이프가 **가로 띠**로 보인다.

    ┌───────────── 바닥 카메라 화면 ─────────────┐
    │  ▓▓▓ 검은 마커(코스 끝)                     │
    │ ═══════════════════════════════  ← 주행 테이프 (가로 띠)
    │            ▲ 띠의 y = 무대와의 거리          │
    └────────────────────────────────────────────┘

  · 띠의 **세로 위치(y)** = 무대까지의 거리 오차  → vx(전후)로 보정
  · 띠의 **기울기(angle)** = 로봇 요(yaw) 오차     → w(회전)로 보정
  · 화면 **가로(x)** = 진행 방향                    → vy(게걸음)로 이동
  · 양끝 **검은 테이프** = 코스 끝(정지)
  · 중간 **색 마스킹테이프** = 토마토 스테이션 표식

⚠ 흔한 "검은 선을 따라간다" 구성이 **아니다.** 세로선 x-오프셋으로 조향하는
   코드로 착각하고 고치지 말 것.


색 분리 (2026-08-09 실측)
-----------------------------------------------------------
주행 테이프가 **밝은 회색**이고 바닥이 **밝은 원목**이라 밝기 차이는 겨우 18로
약하다. 대신 테이프는 **무채색**, 마루는 **따뜻한 갈색(채도 높음)**이라
`score = V - 2*S`로 보면 확실히 갈린다:

    흰 테이프 124  /  마루 56  /  검은 마커 10

밝기만 쓰면(예전 Otsu 방식) 마루 나뭇결을 갈라 유령 라인을 만들어냈다.
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

# --- 주행 테이프(무채색·밝음) ---
# score = V - 2*S 의 임계. 실측: 테이프 124, 마루 56 → 그 사이인 90.
TAPE_SCORE = float(os.environ.get("LF_TAPE_SCORE", "90"))
# 한 행이 "띠"로 인정되려면 화면 폭의 이 비율 이상이 테이프여야 한다.
# 테이프는 화면을 가로지르므로 값이 크고, 얼룩·반사는 이 폭이 안 나온다.
BAND_ROW_FRAC = float(os.environ.get("LF_BAND_ROW_FRAC", "0.35"))
# 띠 두께 허용범위(화면 높이 대비). 너무 두꺼우면 밝은 바닥 전체를 잡은 것.
MIN_BAND_FRAC = float(os.environ.get("LF_MIN_BAND_FRAC", "0.03"))
MAX_BAND_FRAC = float(os.environ.get("LF_MAX_BAND_FRAC", "0.45"))
# 로봇이 무대와 올바른 거리에 있을 때 띠 중심이 오는 y(px). 미설정이면 화면 중앙.
# ★ 장착 후 반드시 실측해 넣을 것(로봇을 정위치에 놓고 band_y를 읽어 그 값).
TARGET_Y = os.environ.get("LF_TARGET_Y")

# --- 코스 끝 표식(검은 테이프) ---
DARK_V = float(os.environ.get("LF_DARK_V", "105"))    # 이보다 어두우면 후보
DARK_S = float(os.environ.get("LF_DARK_S", "90"))     # 채도가 높으면 색마커지 검정이 아니다
DARK_MIN_AREA = int(os.environ.get("LF_DARK_MIN_AREA", "4000"))
# 끝 마커는 주행 테이프 **끝에 붙어 있으므로** 띠와 세로로 가까워야 한다.
# 이 조건이 없으면 화면에 들어온 검은 전선·그림자를 코스 끝으로 오인해
# 주행이 엉뚱한 데서 멈춘다(2026-08-09 실측: 우하단 케이블을 END로 잡음).
DARK_NEAR_BAND = float(os.environ.get("LF_DARK_NEAR_BAND", "0.30"))  # 화면 높이 대비

# --- 스테이션 표식(색 마스킹테이프) ---
COLOR_S = float(os.environ.get("LF_COLOR_S", "90"))   # 이보다 채도가 높으면 색마커
COLOR_V = float(os.environ.get("LF_COLOR_V", "60"))   # 너무 어두우면 그림자
COLOR_MIN_AREA = int(os.environ.get("LF_COLOR_MIN_AREA", "2500"))

# OpenCV Hue는 0~179. 데모에서 헷갈리지 않게 한국어 이름으로 바꿔 보고한다.
HUE_NAMES = [
    (10, "빨강"), (22, "주황"), (33, "노랑"), (85, "초록"),
    (100, "청록"), (130, "파랑"), (160, "보라"), (170, "분홍"), (180, "빨강"),
]


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


def _hue_name(hue: float) -> str:
    for limit, name in HUE_NAMES:
        if hue < limit:
            return name
    return "빨강"


def _largest_blob(mask: np.ndarray, min_area: int):
    """가장 큰 연결 덩어리 (중심x, 중심y, 폭, 높이, 면적). 없으면 None."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    num, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    if num <= 1:
        return None
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[idx, cv2.CC_STAT_AREA])
    if area < min_area:
        return None
    return (
        float(centroids[idx][0]), float(centroids[idx][1]),
        int(stats[idx, cv2.CC_STAT_WIDTH]), int(stats[idx, cv2.CC_STAT_HEIGHT]), area,
    )


def _band_center(tape: np.ndarray, x0: int, x1: int, min_count: int) -> float | None:
    """[x0,x1) 열구간에서 테이프 픽셀의 무게중심 y. 진행각 추정용."""
    part = tape[:, x0:x1]
    ys, _xs = np.nonzero(part)
    if ys.size < min_count:
        return None
    return float(ys.mean())


def _detect(frame: np.ndarray) -> dict:
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV).astype(np.int16)
    H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    target_y = float(TARGET_Y) if TARGET_Y else h / 2.0

    # --- 주행 테이프: 무채색이면서 밝은 영역 ---
    score = V - 2 * S
    tape = ((score > TAPE_SCORE) * 255).astype(np.uint8)
    tape = cv2.morphologyEx(tape, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))

    rows = (tape > 0).sum(axis=1)
    band_rows = np.nonzero(rows > w * BAND_ROW_FRAC)[0]
    band_y = thickness = angle = None
    found = False
    if band_rows.size:
        thickness = int(band_rows.max() - band_rows.min() + 1)
        if h * MIN_BAND_FRAC <= thickness <= h * MAX_BAND_FRAC:
            found = True
            # 무게중심 — 띠 가장자리가 번져도 중심은 안정적이다.
            band_y = float(np.average(band_rows, weights=rows[band_rows]))
            # 좌/우 1/3의 띠 높이 차이 = 로봇 요(yaw) 오차
            left = _band_center(tape, 0, w // 3, int(w * 0.02))
            right = _band_center(tape, 2 * w // 3, w, int(w * 0.02))
            if left is not None and right is not None:
                angle = float(np.degrees(np.arctan2(right - left, 2 * w / 3)))

    # --- 코스 끝(검은 테이프) ---
    dark = (((V < DARK_V) & (S < DARK_S)) * 255).astype(np.uint8)
    end_blob = _largest_blob(dark, DARK_MIN_AREA)
    end_marker = None
    # 띠를 못 찾은 프레임에서는 근접 조건을 걸 기준이 없으므로 그냥 통과시킨다.
    if end_blob and band_y is not None and abs(end_blob[1] - band_y) > h * DARK_NEAR_BAND:
        end_blob = None     # 띠에서 먼 어두운 덩어리 = 전선/그림자
    if end_blob:
        cx, cy, bw, bh, area = end_blob
        end_marker = {
            "x": round(cx, 1), "y": round(cy, 1), "w": bw, "h": bh, "area": area,
            # 화면 어느 쪽에 있나 — 진행 방향 판단용(왼쪽 끝인지 오른쪽 끝인지)
            "side": "left" if cx < w / 2 else "right",
        }

    # --- 스테이션(색 마스킹테이프) ---
    color = (((S > COLOR_S) & (V > COLOR_V)) * 255).astype(np.uint8)
    color_blob = _largest_blob(color, COLOR_MIN_AREA)
    color_marker = None
    if color_blob:
        cx, cy, bw, bh, area = color_blob
        mask = np.zeros((h, w), np.uint8)
        cv2.circle(mask, (int(cx), int(cy)), max(6, min(bw, bh) // 3), 255, -1)
        hue = float(np.median(H[(mask > 0) & (S > COLOR_S)])) if np.any((mask > 0) & (S > COLOR_S)) else 0.0
        color_marker = {
            "x": round(cx, 1), "y": round(cy, 1), "w": bw, "h": bh, "area": area,
            "hue": round(hue, 1), "name": _hue_name(hue),
        }

    return {
        "ts": time.time(),
        "found": found,
        "band_y": None if band_y is None else round(band_y, 1),
        "thickness": thickness,
        # +면 띠가 기준선보다 아래(카메라에 가까움). 부호↔전후 방향 대응은
        # 장착 후 실측으로 확정한다(주행 제어를 붙일 때 LF_INVERT로 뒤집는다).
        "offset_y_px": None if band_y is None else round(band_y - target_y, 1),
        "offset_y_norm": None if band_y is None else round((band_y - target_y) / (h / 2.0), 3),
        "angle_deg": None if angle is None else round(angle, 1),
        "target_y": round(target_y, 1),
        "end_marker": end_marker,
        "color_marker": color_marker,
        "width": w, "height": h,
    }


def _annotate(frame: np.ndarray, st: dict) -> np.ndarray:
    view = frame.copy()
    h, w = view.shape[:2]
    # 기준선(로봇이 정위치일 때 띠가 와야 할 높이)
    ty = int(st["target_y"])
    cv2.line(view, (0, ty), (w - 1, ty), (0, 255, 255), 2)
    cv2.putText(view, "target", (w - 110, ty - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    if st["found"]:
        by = int(st["band_y"])
        cv2.line(view, (0, by), (w - 1, by), (0, 255, 0), 3)
        cv2.line(view, (w // 2, ty), (w // 2, by), (0, 255, 0), 3)
        txt = f"dy {st['offset_y_px']:+.0f}px"
        if st["angle_deg"] is not None:
            txt += f"  yaw {st['angle_deg']:+.1f}deg"
        cv2.putText(view, txt, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    else:
        cv2.putText(view, "TAPE LOST", (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    em = st["end_marker"]
    if em:
        x, y = int(em["x"]), int(em["y"])
        cv2.rectangle(view, (x - em["w"] // 2, y - em["h"] // 2),
                      (x + em["w"] // 2, y + em["h"] // 2), (0, 0, 255), 3)
        cv2.putText(view, f"END({em['side']})", (x - 60, y - em["h"] // 2 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    cm = st["color_marker"]
    if cm:
        x, y = int(cm["x"]), int(cm["y"])
        cv2.rectangle(view, (x - cm["w"] // 2, y - cm["h"] // 2),
                      (x + cm["w"] // 2, y + cm["h"] // 2), (255, 0, 255), 3)
        cv2.putText(view, f"STATION h{cm['hue']:.0f}", (x - 80, y - cm["h"] // 2 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
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
        if frame.shape[:2] != last_shape:
            last_shape = frame.shape[:2]
            print(f"[line_follow] 프레임 {frame.shape[1]}x{frame.shape[0]}", flush=True)

        st = _detect(frame)
        _atomic_write(STATUS, json.dumps(st, ensure_ascii=False).encode("utf-8"))
        ok, buf = cv2.imencode(".jpg", _annotate(frame, st),
                               [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok:
            _atomic_write(VIEW, buf.tobytes())

        slack = period - (time.monotonic() - start)
        if slack > 0:
            time.sleep(slack)


if __name__ == "__main__":
    main()
