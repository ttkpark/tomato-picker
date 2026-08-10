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

  · 띠의 **세로 위치(y)** = 테이프까지의 거리 오차 → **vy**(게걸음)로 보정
  · 띠의 **기울기(angle)** = 로봇 요(yaw) 오차     → w(회전)로 보정
  · 화면 **가로(x)** = 진행 방향                    → **vx**(전후)로 이동
  · **색 마스킹테이프** = 지점 표식. 흰 도화지 코스에서는 **양끝단 포함 4지점**을
    전부 색으로 표시한다(배경이 통제돼 색이 믿을 만해졌기 때문).
  · 검은 끝마커는 원목 코스(극성 light)에서만 쓴다 — 극성이 dark면 테이프 자체가
    어두워서 구분이 안 되므로 검출하지 않는다.

⚠ 흔한 "검은 선을 따라간다" 구성이 **아니다.** 세로선 x-오프셋으로 조향하는
   코드로 착각하고 고치지 말 것. 축 대응은 hardware/line_drive.py가 확정한다.


색 분리 — 절대값도, 극성도 고정하지 않는다 (2026-08-09 실측)
-----------------------------------------------------------
코스가 두 번 바뀌었고 그때마다 전제가 깨졌다. 그래서 지금은 **아무것도 가정하지 않는다.**

  ① 원목 바닥 + 밝은 회색 테이프  → 테이프가 배경보다 **밝고 무채색**
  ② 흰 도화지 + 어두운 테이프      → 테이프가 배경보다 **어두움**

**극성은 자동 판별한다.** 두 가정으로 각각 마스크를 만들어보고, **가로 띠가 실제로
만들어지는 쪽**(화면을 가로지르고 두께가 범위 안)을 채택한다. 설정으로 두면 코스를
바꿀 때마다 틀린다.

**임계는 Otsu**(2-모드 자동 분리)로 뽑는다. 백분위(상위 N%)를 쓰면 "테이프가 화면의
몇 %인가"를 가정하게 되는데, 카메라 높이와 코스에 따라 그게 깨진다 — 도화지 코스에서
띠가 화면의 21%라 "상위 8%" 임계가 띠 안쪽에 떨어져 검출이 통째로 실패했다.

**절대 임계는 쓰지 않는다.** AWB가 흔들려 화면이 보라빛으로 물든 순간 이렇게 무너졌다:

                     score 중앙값   테이프   절대임계 90 통과
    중립 프레임            66        124        16.4%  → 정상
    보라 캐스트             7         58         0.0%  → 전멸(TAPE LOST)

게다가 캐스트를 받으면 **어두운 것의 채도가 프레임에서 가장 높아져**(113 vs 중앙값 66)
색 마커로 오인됐다. 그래서 **어두운 것은 밝기로만** 가른다.

유령 검출은 임계가 아니라 **물리적 성질**로 막는다:
    · 선택 영역이 배경보다 뚜렷할 것 (separation ≥ 22)
    · 화면 폭의 35% 이상을 가로지르는 행이 있을 것
    · 띠 두께가 화면 높이의 3~45%일 것

실측 참고값 (흰 도화지 코스): 배경 V=151 S=11 / 테이프 V=70 S=58 / 색마커 V=174 S=162
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

# --- 주행 테이프 (극성 자동 · Otsu 임계) ---
# 자세한 배경은 모듈 docstring의 "색 분리" 절 참고.
TAPE_MARGIN = float(os.environ.get("LF_TAPE_MARGIN", "22"))  # 중앙값보다 이만큼은 높아야
# 한 행이 "띠"로 인정되려면 화면 폭의 이 비율 이상이 테이프여야 한다.
# 테이프는 화면을 가로지르므로 값이 크고, 얼룩·반사는 이 폭이 안 나온다.
BAND_ROW_FRAC = float(os.environ.get("LF_BAND_ROW_FRAC", "0.35"))
# 띠 두께 허용범위(화면 높이 대비). 너무 두꺼우면 밝은 바닥 전체를 잡은 것.
MIN_BAND_FRAC = float(os.environ.get("LF_MIN_BAND_FRAC", "0.03"))
MAX_BAND_FRAC = float(os.environ.get("LF_MAX_BAND_FRAC", "0.45"))
# 로봇이 무대와 올바른 거리에 있을 때 띠 중심이 오는 y(px). 미설정이면 화면 중앙.
TARGET_Y = os.environ.get("LF_TARGET_Y")
# 런타임 교정: 이 파일에 숫자가 있으면 위 환경변수보다 우선한다. 대시보드의
# "현재 위치를 기준으로" 버튼이 여기에 쓴다 — 코스(도화지·테이프 색)가 바뀔
# 때마다 서비스 파일을 고치고 재시작하는 건 현장에서 너무 느리다.
TARGET_Y_FILE = os.environ.get("LF_TARGET_Y_FILE", "/dev/shm/line_target_y")

# --- 시각 오도메트리 (바닥 무늬 흐름으로 이동량 측정) ---
# 바퀴 엔코더가 없고 메카넘은 슬립이 심해 "몇 초 굴렸다"로는 변위를 못 믿는다.
# 바닥 카메라가 보는 나뭇결이 프레임마다 얼마나 흘렀는지를 위상상관으로 재면
# **실제로 움직인 거리**가 나온다(슬립·배터리 전압과 무관). 저속일수록 정확.
ODOM = os.environ.get("LF_ODOM", "1") not in ("0", "false", "no")
ODOM_SCALE = float(os.environ.get("LF_ODOM_SCALE", "0.25"))   # 축소 배율(속도↑)
# 위상상관 신뢰도가 이보다 낮으면 그 프레임은 버린다(모션블러·균일면 대비).
ODOM_MIN_RESPONSE = float(os.environ.get("LF_ODOM_MIN_RESPONSE", "0.05"))
# 한 프레임에 이보다 크게 튀면 오검출로 보고 버린다(원본 px 기준).
ODOM_MAX_JUMP = float(os.environ.get("LF_ODOM_MAX_JUMP", "120"))

# --- 코스 끝 표식(검은 테이프) ---
# 여기도 상대값: 프레임 밝기 중앙값보다 이만큼 어두우면 "검은 테이프".
# 실측으로 끝마커는 중앙값보다 63(중립)~75(보라캐스트) 어두웠다.
# ⚠ 채도로는 검정을 못 가른다 — 보라 캐스트에서 끝마커의 채도가 113으로
# **프레임에서 가장 높아져** 색 마커로 오인됐다(2026-08-09). 그래서 밝기로만 가른다.
DARK_MARGIN = float(os.environ.get("LF_DARK_MARGIN", "40"))
DARK_MIN_AREA = int(os.environ.get("LF_DARK_MIN_AREA", "4000"))
# 끝 마커는 주행 테이프 **끝에 붙어 있으므로** 띠와 세로로 가까워야 한다.
# 이 조건이 없으면 화면에 들어온 검은 전선·그림자를 코스 끝으로 오인해
# 주행이 엉뚱한 데서 멈춘다(2026-08-09 실측: 우하단 케이블을 END로 잡음).
DARK_NEAR_BAND = float(os.environ.get("LF_DARK_NEAR_BAND", "0.30"))  # 화면 높이 대비

# --- 지점 표식(색 마스킹테이프) ---
# 배경이 통제되지 않으면(원목 + 조명 캐스트) 색상(hue)을 믿을 수 없어 기본 꺼짐이었다.
# 흰 도화지 코스에서는 배경 S=11 / 색마커 S=162로 압도적으로 갈려 켤 만하다
# → deploy/line-follow.service에 LF_COLOR=1.
COLOR_ENABLED = os.environ.get("LF_COLOR", "0") not in ("0", "false", "no", "")
# 상대값 + **어두운 건 제외**. 색 마스킹테이프는 밝고 진하지만, 검은 테이프는
# 캐스트를 받으면 채도만 높고 어둡다 — 이 둘을 밝기로 갈라야 한다.
COLOR_S_MARGIN = float(os.environ.get("LF_COLOR_S_MARGIN", "50"))  # 중앙값 대비
COLOR_S_MIN = float(os.environ.get("LF_COLOR_S_MIN", "80"))        # 절대 하한(무채색 장면 방지)
COLOR_MIN_AREA = int(os.environ.get("LF_COLOR_MIN_AREA", "2500"))

# --- 마커의 **역할** (색 이름이 아니라 이게 진짜 의미다) ---
# 코스 구성: [주황]─[노랑]─[노랑]─[주황]
#   주황 = 양끝 지점(끝점).  노랑 = 중간 지점.
# ⚠ 색 이름으로 부르면 헷갈린다 — 주황 테이프가 OpenCV hue 6으로 잡혀 화면에
#   "빨강 스테이션"으로 떴다(2026-08-10). 조명에 따라 hue는 몇씩 밀리므로
#   **이름이 아니라 역할**을 보고해야 한다. 역할 판정은 여기 한 곳에서만 한다
#   (상위 LineDriver는 이 결과를 그대로 쓴다 — 두 군데서 정하면 어긋난다).
#   초록 = **경유 표식**. 지점이 아니다 — 지점 간격이 멀어 카메라가 아무 마커도
#   못 보는 구간이 생기자 그걸 메우려고 중간에 붙였다(2026-08-10). 위치 추적을
#   끊기지 않게 하는 게 목적이므로 **지점 번호를 바꾸면 안 된다**.
HUE_END = tuple(int(v) for v in os.environ.get("LF_HUE_END", "0,18").split(","))
HUE_MID = tuple(int(v) for v in os.environ.get("LF_HUE_MID", "19,40").split(","))
HUE_WAY = tuple(int(v) for v in os.environ.get("LF_HUE_WAY", "41,90").split(","))
ROLE_LABELS = {"end": "끝점", "mid": "중간지점", "way": "경유"}

# OpenCV Hue는 0~179. 역할 밖의 색이 잡혔을 때 무엇이 보였는지 알려주는 보조 이름.
HUE_NAMES = [
    (10, "빨강"), (22, "주황"), (33, "노랑"), (85, "초록"),
    (100, "청록"), (130, "파랑"), (160, "보라"), (170, "분홍"), (180, "빨강"),
]


def _hue_role(hue: float) -> str | None:
    """hue → 코스에서의 역할. 'end'(주황) | 'mid'(노랑) | None(정의 안 된 색)."""
    if HUE_END[0] <= hue <= HUE_END[1]:
        return "end"
    if HUE_MID[0] <= hue <= HUE_MID[1]:
        return "mid"
    if HUE_WAY[0] <= hue <= HUE_WAY[1]:
        return "way"
    return None


def _atomic_write(path: str, data: bytes) -> None:
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def _read_frame() -> np.ndarray | None:
    """원본 JPEG를 읽어 디코드. 찢어진 프레임은 건너뛴다.

    ⚠ gst `multifilesink`는 같은 경로에 **비원자적으로** 덮어쓴다. 쓰는 도중에
    읽으면 앞부분만 있는 JPEG가 나오는데, `cv2.imdecode`는 이걸 **실패로 알리지
    않고** 나머지를 회색으로 채운 이미지를 돌려준다. 그래서 정지된 장면인데도
    검출이 깜빡였다(2026-08-09: 같은 화면에서 found가 True/False를 오갔고,
    오버레이 아래쪽이 뿌옇게 뭉개져 있었다 — 그게 잘린 프레임의 흔적).

    JPEG는 반드시 EOI 마커(FFD9)로 끝나므로 그걸로 완결성을 확인한다.
    """
    try:
        with open(SRC, "rb") as f:
            data = f.read()
    except OSError:
        return None
    if len(data) < 4 or not data.rstrip(b"\x00").endswith(b"\xff\xd9"):
        return None     # 아직 다 안 써졌다 — 다음 주기에 다시 읽는다
    return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)


def _read_target_y(default: float) -> float:
    """런타임 교정 파일 → 환경변수 → 기본값(화면 중앙) 순."""
    try:
        with open(TARGET_Y_FILE) as f:
            return float(f.read().strip())
    except (OSError, ValueError):
        pass
    try:
        return float(TARGET_Y) if TARGET_Y else default
    except ValueError:
        return default


class _Odometry:
    """연속 프레임의 위상상관으로 바닥 이동량을 누적한다.

    cv2.phaseCorrelate는 두 영상 사이의 **평행이동**을 서브픽셀로 찾는다. 바닥은
    통째로 같이 움직이므로(테이프도 바닥의 일부다) 회전이 작은 구간에서는 이게
    곧 로봇의 이동량이다. 축소해서 돌리므로 비용도 싸다.

    ⚠ 절대 위치가 아니라 **증분의 누적**이라 시간이 지나면 조금씩 흐른다.
    그래서 상위(LineDriver)가 끝단 검은 테이프를 만날 때마다 원점을 다시 잡는다.
    """

    def __init__(self) -> None:
        self._prev = None
        self._hann = None
        self.x = 0.0          # 누적 이동(원본 px). 화면 가로 = 진행 방향
        self.y = 0.0
        self.response = 0.0
        self.dx = 0.0         # 직전 프레임 증분(속도 추정용)
        self.dy = 0.0

    def update(self, gray: np.ndarray) -> None:
        small = cv2.resize(gray, None, fx=ODOM_SCALE, fy=ODOM_SCALE,
                           interpolation=cv2.INTER_AREA).astype(np.float32)
        if self._prev is None or self._prev.shape != small.shape:
            self._prev = small
            self._hann = cv2.createHanningWindow(small.shape[::-1], cv2.CV_32F)
            return
        (sx, sy), response = cv2.phaseCorrelate(self._prev, small, self._hann)
        self._prev = small
        self.response = float(response)
        if response < ODOM_MIN_RESPONSE:
            self.dx = self.dy = 0.0
            return
        # 축소분 되돌리기. 부호: 영상 내용이 왼쪽으로 흐르면 로봇은 오른쪽으로 갔다.
        dx, dy = -sx / ODOM_SCALE, -sy / ODOM_SCALE
        if abs(dx) > ODOM_MAX_JUMP or abs(dy) > ODOM_MAX_JUMP:
            self.dx = self.dy = 0.0
            return
        self.dx, self.dy = float(dx), float(dy)
        self.x += self.dx
        self.y += self.dy


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


def _band_angle(tape: np.ndarray, r0: int, r1: int) -> float | None:
    """띠의 기울기(도). 열마다 띠 중심 y를 구해 직선을 최소자승으로 맞춘다.

    ⚠ 예전엔 화면을 좌/우 1/3으로 나눠 두 무게중심의 차이로 각을 냈는데,
    ① 표본이 두 점뿐이라 잡음에 그대로 흔들리고 ② **띠 밖의 밝은 것**(흰 무대,
    반사, 하얀 케이블)이 그 1/3 안에 들어오면 중심이 통째로 끌려갔다. 그 결과
    각이 실제와 무관하게 튀어 회전 보정이 발산했다(2026-08-09 실기).

    지금은 **검출된 띠 행 범위 안에서만** 열별 중심을 구하고, 유효한 열이 화면
    폭의 절반을 넘을 때만 각을 낸다 — 표본이 수백 개라 훨씬 안정적이다.
    """
    band = tape[r0:r1]
    if band.size == 0:
        return None
    counts = (band > 0).sum(axis=0)
    ys = np.arange(band.shape[0], dtype=np.float32)
    # 열별 무게중심 y (픽셀이 충분한 열만)
    valid = counts >= max(3, band.shape[0] // 6)
    if valid.sum() < band.shape[1] * 0.5:
        return None
    centers = ((band > 0) * ys[:, None]).sum(axis=0)[valid] / counts[valid]
    xs = np.nonzero(valid)[0].astype(np.float32)
    slope = float(np.polyfit(xs, centers, 1)[0])
    return float(np.degrees(np.arctan(slope)))


def _detect(frame: np.ndarray, odom: "_Odometry | None" = None) -> dict:
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV).astype(np.int16)
    H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    target_y = _read_target_y(h / 2.0)

    # --- 주행 테이프: 극성 자동 판별 ---
    # 코스 배경이 바뀌면 테이프 극성도 뒤집힌다(2026-08-09: 원목 위 밝은 회색
    # 테이프 → 흰 도화지 위 **어두운** 테이프). 어느 쪽인지 설정으로 두면 바꿀 때마다
    # 틀리므로, 두 가정을 다 세워보고 **가로 띠가 실제로 만들어지는 쪽**을 채택한다.
    #   light: 배경보다 밝고 무채색 (V - 2S 가 큼)   — 원목 바닥 + 흰 테이프
    #   dark : 배경보다 어두움     (중앙값 - V 가 큼) — 흰 도화지 + 검은 테이프
    med_v = float(np.median(V))
    med_s = float(np.median(S))
    scores = {"light": V - 2 * S, "dark": int(med_v) - V}
    best = None
    for polarity, score in scores.items():
        med_score = float(np.median(score))
        # 임계는 **Otsu**로 뽑는다 — 백분위(상위 N%)는 테이프가 화면에서 차지하는
        # 비율을 가정하는데, 그 가정이 카메라 높이/코스에 따라 깨진다.
        # (2026-08-09: 흰 도화지 코스에서 띠가 화면의 21%라 "상위 8%" 임계가
        #  띠 안쪽에 떨어져 검출이 통째로 실패했다.) Otsu는 비율을 가정하지 않는다.
        # 유령 검출은 아래 separation + 띠 기하 검사가 막는다.
        otsu, _ = cv2.threshold(np.clip(score, 0, 255).astype(np.uint8), 0, 255,
                                cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        thr = max(float(otsu), med_score + TAPE_MARGIN)
        mask = ((score > thr) * 255).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
        sel = score[mask > 0]
        sep = float(sel.mean() - med_score) if sel.size else 0.0
        if sep < TAPE_MARGIN:
            continue
        # 가로로 화면을 가로지르는 띠가 실제로 만들어지는지 — 두께까지 맞아야 채택.
        rows = (mask > 0).sum(axis=1)
        band = np.nonzero(rows > w * BAND_ROW_FRAC)[0]
        if not band.size:
            continue
        thick = int(band.max() - band.min() + 1)
        if not (h * MIN_BAND_FRAC <= thick <= h * MAX_BAND_FRAC):
            continue
        if best is None or sep > best[0]:
            best = (sep, polarity, mask, rows, band, thick, thr)

    if best is None:
        tape = np.zeros((h, w), np.uint8)
        separation, polarity, tape_thr = 0.0, None, 0.0
        rows = np.zeros(h, dtype=int)
        band_rows = np.array([], dtype=int)
    else:
        separation, polarity, tape, rows, band_rows, _thick, tape_thr = best

    band_y = thickness = angle = None
    found = False
    if band_rows.size:
        thickness = int(band_rows.max() - band_rows.min() + 1)
        if h * MIN_BAND_FRAC <= thickness <= h * MAX_BAND_FRAC:
            found = True
            # 무게중심 — 띠 가장자리가 번져도 중심은 안정적이다.
            band_y = float(np.average(band_rows, weights=rows[band_rows]))
            # 띠의 기울기 = 로봇 요(yaw) 오차. 띠 행 범위 안에서만 재서
            # 화면의 다른 밝은 것에 끌려가지 않게 한다.
            margin = max(4, thickness // 4)
            angle = _band_angle(tape, max(0, int(band_rows.min()) - margin),
                                min(h, int(band_rows.max()) + margin + 1))

    # --- 코스 끝(검은 테이프) ---
    # ⚠ 극성이 dark면 **주행 테이프 자체가 어두우므로** 이 검출은 의미가 없다
    #   (실제로 테이프를 통째로 "END"로 잡았다 — 2026-08-09 흰 도화지 코스).
    #   그 구성에서는 양끝단도 색 마커로 표시하므로 아래 색 검출이 대신한다.
    dark_mask = V < (med_v - DARK_MARGIN)
    end_marker = None
    if polarity == "light":
        end_blob = _largest_blob((dark_mask * 255).astype(np.uint8), DARK_MIN_AREA)
        # 띠를 못 찾은 프레임에서는 근접 조건을 걸 기준이 없으므로 그냥 통과시킨다.
        if end_blob and band_y is not None and abs(end_blob[1] - band_y) > h * DARK_NEAR_BAND:
            end_blob = None     # 띠에서 먼 어두운 덩어리 = 전선/그림자
        if end_blob:
            cx, cy, bw, bh, area = end_blob
            end_marker = {
                "x": round(cx, 1), "y": round(cy, 1), "w": bw, "h": bh, "area": area,
                "side": "left" if cx < w / 2 else "right",
            }

    # --- 지점 표식(색 마스킹테이프) ---
    # 흰 도화지 배경에서는 채도가 압도적으로 갈린다(배경 S≈11, 테이프 58, 색마커 162).
    # 한 화면에 여러 개가 보일 수 있으므로 **전부** 돌려주고, 화면 중앙에 가장
    # 가까운 것을 대표(color_marker)로 삼는다 — 정지 판정이 그걸 기준으로 돈다.
    color_thr = max(COLOR_S_MIN, med_s + COLOR_S_MARGIN)
    markers: list[dict] = []
    if COLOR_ENABLED:
        color = (((S > color_thr) & ~dark_mask) * 255).astype(np.uint8)
        color = cv2.morphologyEx(color, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
        num, _lbl, stats, cents = cv2.connectedComponentsWithStats(color, 8)
        for i in range(1, num):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < COLOR_MIN_AREA:
                continue
            cx, cy = float(cents[i][0]), float(cents[i][1])
            spot = np.zeros((h, w), np.uint8)
            cv2.circle(spot, (int(cx), int(cy)),
                       max(6, min(stats[i, cv2.CC_STAT_WIDTH],
                                  stats[i, cv2.CC_STAT_HEIGHT]) // 3), 255, -1)
            hot = (spot > 0) & (S > color_thr)
            hue = float(np.median(H[hot])) if np.any(hot) else 0.0
            role = _hue_role(hue)
            markers.append({
                "x": round(cx, 1), "y": round(cy, 1),
                "w": int(stats[i, cv2.CC_STAT_WIDTH]), "h": int(stats[i, cv2.CC_STAT_HEIGHT]),
                "area": area, "hue": round(hue, 1),
                "role": role,                       # 'end' | 'mid' | None
                # 화면에 찍을 이름 — 역할이 있으면 역할로 부른다(색 이름은 헷갈린다)
                "name": ROLE_LABELS.get(role) or f"미정의({_hue_name(hue)})",
            })
        markers.sort(key=lambda m: m["x"])
    color_marker = min(markers, key=lambda m: abs(m["x"] - w / 2)) if markers else None

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
        "markers": markers,          # 화면에 보이는 지점 표식 전부(x 오름차순)
        "polarity": polarity,        # 테이프가 배경보다 dark/light 중 어느 쪽인지
        "width": w, "height": h,
        # 시각 오도메트리 — 누적 이동(px)과 직전 증분. 절대 위치가 아니라
        # 증분 누적이므로 상위가 끝단 마커에서 원점을 다시 잡는다.
        "odom_x_px": None if odom is None else round(odom.x, 1),
        "odom_y_px": None if odom is None else round(odom.y, 1),
        "odom_dx": None if odom is None else round(odom.dx, 2),
        "odom_conf": None if odom is None else round(odom.response, 3),
        # 튜닝/진단용 — 검출이 안 될 때 임계가 어디에 걸렸는지 숫자로 본다.
        "tape_thr": round(tape_thr, 1),
        "separation": round(separation, 1),
        "med_v": round(med_v, 1), "med_s": round(med_s, 1),
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

    # 지점 표식은 보이는 것 전부 그린다 — 대표(중앙에 가장 가까운 것)만 굵게.
    rep = st["color_marker"]
    for cm in st.get("markers") or []:
        x, y = int(cm["x"]), int(cm["y"])
        main = rep is not None and abs(cm["x"] - rep["x"]) < 1e-6
        cv2.rectangle(view, (x - cm["w"] // 2, y - cm["h"] // 2),
                      (x + cm["w"] // 2, y + cm["h"] // 2),
                      (255, 0, 255), 4 if main else 2)
        tag = {"end": "END", "mid": "MID", "way": "WAY"}.get(cm.get("role"), "?")
        cv2.putText(view, f"{tag} h{cm['hue']:.0f}", (x - 60, y - cm["h"] // 2 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
    cv2.putText(view, f"pol={st.get('polarity')}", (12, h - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 2)
    return view


def main() -> None:
    print(f"[line_follow] {SRC} → {VIEW} / {STATUS}", flush=True)
    period = 1.0 / max(1.0, FPS)
    last_shape = None
    odom = _Odometry() if ODOM else None
    while True:
        start = time.monotonic()
        frame = _read_frame()
        if frame is None:
            time.sleep(0.2)
            continue
        if frame.shape[:2] != last_shape:
            last_shape = frame.shape[:2]
            print(f"[line_follow] 프레임 {frame.shape[1]}x{frame.shape[0]}", flush=True)

        if odom is not None:
            odom.update(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        st = _detect(frame, odom)
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
