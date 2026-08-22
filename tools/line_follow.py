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
    · 화면 폭의 28% 이상을 가로지르는 행이 있을 것
    · 띠 두께가 화면 높이의 3~45%일 것
    · 자격을 갖춘 덩어리가 **여럿이면 뚜렷함 × 가로지름**이 가장 큰 것을 고른다
      (가로지름만 보면 화면 끝까지 이어지는 마룻바닥 경계에 진다 — 2026-08-18)

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
# --- 조명이 물들 때를 위한 두 가지 (2026-08-17 현장: "배경이 빨개지면 못 잡는다") ---
# ⚠ 무대 조명이 마젠타로 물들면 **자주색 테이프가 같은 색 계열이라 같이 밝아진다.**
#   실측: 정상 조명에서 테이프 V=59 / 종이 V=145 (차이 86)
#         마젠타 조명에서 테이프 V=108 / 종이 V=141 (차이 **32**) — 37%로 무너진다.
#   종이 노이즈 σ가 4.5인데 대비가 32면 Otsu 마스크가 너덜너덜해져, 띠 한복판이
#   숭숭 뚫리고 조각들이 두께 1~3%로 갈린다("가로지름 19%", "두께 3%·0%").
#   대비 자체는 되살릴 수 없지만 **노이즈는 지울 수 있다** — 블러를 키우고 구멍을
#   메우면 같은 프레임이 그대로 살아난다(실측: 실패 6프레임 전부 복구).
BLUR = int(os.environ.get("LF_BLUR", "15")) | 1        # 홀수여야 한다
TAPE_CLOSE = int(os.environ.get("LF_TAPE_CLOSE", "31")) | 1
# 한 행이 "띠"로 인정되려면 화면 폭의 이 비율 이상이 테이프여야 한다.
# 테이프는 화면을 가로지르므로 값이 크고, 얼룩·반사는 이 폭이 안 나온다.
# ⚠ 0.35였다가 0.28로(2026-08-11). **코스 끝점에서는 테이프가 화면을 다
#   가로지르지 못하는 게 구조적으로 정상**이다 — 왼쪽 끝이면 오른쪽으로만
#   이어지니 화면의 1/3~1/2만 덮는다. 실측: 좌측 끝점 정위치에서 34%가 나와
#   35% 임계에 1%p 차이로 "테이프 없음"이 됐고, 출발 전 점검이 모든 주행
#   버튼을 거부했다("로봇이 안 움직여"). 코스 밖 이탈(우측 끝 너머 타일)은
#   22%였으므로 0.28은 그 둘을 가른다. 유령 검출은 여전히 separation·두께
#   검사가 막는다.
BAND_ROW_FRAC = float(os.environ.get("LF_BAND_ROW_FRAC", "0.28"))
# 띠가 "이 열에 있다"고 볼 최소 세로 점유율(띠 두께 대비). 끝점에서 **어느 쪽으로
# 코스가 이어지는지**를 판정할 때 쓴다 — 가장자리 번짐에 안 흔들리게 넉넉히.
BAND_COL_FRAC = float(os.environ.get("LF_BAND_COL_FRAC", "0.40"))
# 띠 두께 허용범위(화면 높이 대비). 너무 두꺼우면 밝은 바닥 전체를 잡은 것.
MIN_BAND_FRAC = float(os.environ.get("LF_MIN_BAND_FRAC", "0.03"))
MAX_BAND_FRAC = float(os.environ.get("LF_MAX_BAND_FRAC", "0.45"))
# --- 띠 연속성: 바닥의 띠는 한 프레임에 순간이동하지 않는다 ---
# 15fps에서 한 프레임 사이 띠는 몇 px 움직인다(실측 주행 중 dy 변화 20~50px/0.7초).
# 이보다 크게 뛰면 로봇이 옮겨간 게 아니라 **다른 것을 잡은 것**이다.
# 2026-08-17: 코스 끝을 지나쳐 서자 테이프가 화면의 23%만 남아 탈락하고 원목 마루
# 경계가 뽑혔다 — band_y 360 → 703(dy 343px)을 found=True로 내보냈다.
BAND_JUMP_PX = float(os.environ.get("LF_BAND_JUMP_PX", "120"))
# 직전 띠가 이보다 오래됐으면 검사하지 않는다(막 살아난 참이라 비교 대상이 없다).
BAND_JUMP_SEC = float(os.environ.get("LF_BAND_JUMP_SEC", "0.8"))
# 연속으로 이만큼 거부하면 새 위치를 현실로 받아들인다 — 손으로 옮겼거나 코스를
# 새로 깔았을 때 영영 "테이프 없음"에 잠기면 안 된다(≈0.7초 @15fps).
BAND_JUMP_MAX_REJECT = int(os.environ.get("LF_BAND_JUMP_MAX_REJECT", "10"))
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
# ⚠ 마커는 테이프 **바로 옆**에 붙어 있다 — 띠에서 세로로 이보다 먼 색 덩어리는
# 코스 밖이다. 2026-08-11 실사고: 로봇이 코스 오른쪽 끝을 지나 마루 타일 위를
# 보자 타일 무늬가 중간지점(h19~21) 5개 + 끝점(주황빛) 2개로 잡혔다. 가짜
# 마커가 지점 세기를 무너뜨리고(번호 널뜀), 가짜 끝점이 변위 원점까지 다시
# 잡아 오도메트리를 망쳤다(변위가 음수로 튀던 것). 띠 근접 조건이 이걸 끊는다
# (검은 끝마커의 DARK_NEAR_BAND와 같은 원리).
# ⚠ 고정 비율(화면의 30%)로 했더니 **진짜 끝점을 버렸다**(실측: 마커가 커서
#   중심이 띠에서 220px, 한계 216px). 마커 크기에 따라 중심 거리는 얼마든지
#   커지므로, "붙어 있다"를 그대로 수식으로 쓴다:
#   |마커중심y - 띠중심y| ≤ 띠 절반두께 + 마커 절반높이 + 여유
COLOR_NEAR_GAP = float(os.environ.get("LF_COLOR_NEAR_GAP", "80"))  # px: 허용 틈

# --- 마커의 **색 갈래** ---
# 코스 구성(2026-08-17, 2지점): [노랑]────────[주황]
#                               지점0          지점1
#
# ⚠ **여기서는 색까지만 정한다 — 어느 지점인지는 정하지 않는다.**
#   색 → 지점 번호는 config.LINE_MARKER_STATION 한 곳이 정한다. 두 군데서
#   정하면 코스를 바꿀 때 한쪽만 고쳐져 어긋난다(이 파일은 별도 venv라 그
#   config를 import할 수도 없다).
# ⚠ 갈래 이름은 hue에서 **뽑지 않는다**. 주황 테이프가 OpenCV hue 6으로 잡혀
#   화면에 "빨강 스테이션"으로 떴다(2026-08-10). 조명에 따라 hue는 몇씩 밀리므로
#   hue 창이 갈래를 정하고, 이름은 그 갈래에 고정으로 붙인다.
# ⚠ 'end'/'mid'라는 키 이름은 옛 코스(양끝=주황, 중간=노랑)에서 온 것이고
#   지금은 **색을 가리키는 이름일 뿐**이다. 2지점 코스에서는 둘 다 코스의 끝이다.
#   상태 JSON·저장 파일이 이 키를 쓰고 있어 이름만 그대로 둔다.
#   초록 = **경유 표식**. 지점이 아니다 — 지점 간격이 멀어 카메라가 아무 마커도
#   못 보는 구간이 생기자 그걸 메우려고 중간에 붙였다(2026-08-10). 위치 추적을
#   끊기지 않게 하는 게 목적이므로 **지점 번호를 바꾸면 안 된다**.
HUE_END = tuple(int(v) for v in os.environ.get("LF_HUE_END", "0,18").split(","))
HUE_MID = tuple(int(v) for v in os.environ.get("LF_HUE_MID", "19,40").split(","))
HUE_WAY = tuple(int(v) for v in os.environ.get("LF_HUE_WAY", "41,90").split(","))
ROLE_LABELS = {"end": "주황", "mid": "노랑", "way": "초록(경유)"}

# OpenCV Hue는 0~179. 역할 밖의 색이 잡혔을 때 무엇이 보였는지 알려주는 보조 이름.
HUE_NAMES = [
    (10, "빨강"), (22, "주황"), (33, "노랑"), (85, "초록"),
    (100, "청록"), (130, "파랑"), (160, "보라"), (170, "분홍"), (180, "빨강"),
]


def _role_pixels(H: np.ndarray) -> np.ndarray:
    """**우리가 아는 색**(주황·노랑·초록)인 픽셀만 True.

    ⚠ 예전엔 색 마커에서 "어두운 것"을 밝기로 뺐다(`& ~dark_mask`). 주행 테이프가
    채도까지 높아서(실측 S=76~82, 문턱 80) 그러지 않으면 화면의 10만 px짜리
    덩어리가 통째로 마커로 잡혔기 때문이다. 그런데 그 그물에 **초록 마커가 같이
    걸렸다** — 2026-08-17 실측: 초록 H=73 S=104 **V=83**, 어두움 문턱이 93이라
    99%가 제거됐다. "초록은 인식이 안 된다"고 알고 코스에서 뺐던 게 이것이다.

    밝기로 색을 가리려 한 게 잘못이었다. 테이프를 막는 건 **hue**가 할 일이다 —
    테이프는 H=152~157(자주)이라 어느 역할 창에도 안 들어간다. 정의된 창의
    픽셀만 남기면 테이프는 픽셀 단위로 빠지고, 그 위에 붙은 주황 마커는 테이프와
    한 덩어리로 뭉치지도 않는다(실측: 면적 51487 → 51756, 사실상 그대로).
    """
    ok = np.zeros(H.shape, bool)
    for lo, hi in (HUE_END, HUE_MID, HUE_WAY):
        ok |= (H >= lo) & (H <= hi)
    return ok


def _hue_role(hue: float) -> str | None:
    """hue → 코스에서의 역할. 'end'(주황) | 'mid'(노랑) | 'way'(초록) | None."""
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


# 띠 연속성 — 직전에 인정한 띠의 위치와 시각, 그리고 연속 거부 횟수.
_BAND_LAST: dict = {"y": None, "ts": 0.0, "rejects": 0}


def _band_is_continuous(band_y: float | None) -> tuple[bool, str]:
    """이 띠가 **직전 띠에서 이어진 것**인가. 아니면 (False, 사유).

    ⚠ 2026-08-17 실기: 코스 오른쪽 끝을 지나쳐 서자 화면에 테이프가 왼쪽 23%만
    남아 행 임계(28%)를 못 넘었고, 대신 **아래쪽 원목 마루 경계**가 띠로 뽑혔다.
    band_y가 한 프레임에 360 → 703으로 뛰었고(dy 1.5px → 343px), 검출기는 그걸
    found=True로 자신 있게 내보냈다. 그 값을 믿고 정렬하면 엉뚱한 데로 간다.

    **바닥의 띠는 한 프레임(≈1/15초)에 300px를 못 움직인다.** 그러니 그런 점프는
    "로봇이 옮겨간 것"이 아니라 "다른 것을 잡은 것"이다. 거부하고 테이프 없음으로
    둔다 — 코스 끝에서는 그게 사실이기도 하다.

    ⚠ 단 **영영 잠기면 안 된다.** 사람이 로봇을 손으로 옮기거나 코스를 새로 깔면
    진짜로 띠가 멀리 가 있다. 그래서 (a) 직전 띠가 오래됐으면 검사하지 않고,
    (b) 연속으로 몇 번 거부하면 새 위치를 현실로 받아들인다.
    """
    now = time.monotonic()
    last_y, last_ts = _BAND_LAST["y"], _BAND_LAST["ts"]
    if band_y is None:
        return True, ""                      # 애초에 못 찾았다 — 여기서 할 말이 없다
    fresh = last_y is not None and (now - last_ts) <= BAND_JUMP_SEC
    if fresh and abs(band_y - last_y) > BAND_JUMP_PX:
        _BAND_LAST["rejects"] += 1
        if _BAND_LAST["rejects"] <= BAND_JUMP_MAX_REJECT:
            return False, (f"띠가 한 프레임에 {abs(band_y - last_y):.0f}px 튐"
                           f"(>{BAND_JUMP_PX:.0f}) — 다른 것을 잡은 것으로 보고 버림"
                           f" [{_BAND_LAST['rejects']}/{BAND_JUMP_MAX_REJECT}]")
        # 계속 같은 곳이 보인다 = 정말 옮겨간 것이다. 새 위치를 받아들인다.
    _BAND_LAST.update(y=band_y, ts=now, rejects=0)
    return True, ""


def _contiguous(idx: np.ndarray) -> list[tuple[int, int]]:
    """정렬된 인덱스 배열 → 끊기지 않은 구간 [(시작, 끝), …].

    "띠로 인정된 행"들을 **덩어리로 갈라 보기 위한** 것이다. 이게 없으면 화면
    맨 위의 물건과 맨 아래의 물건이 한 띠로 취급된다(아래 _detect 주석 참고).
    """
    if not idx.size:
        return []
    breaks = np.nonzero(np.diff(idx) > 1)[0]
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]]))
    return [(int(a), int(b)) for a, b in zip(starts, ends)]


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
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (BLUR, BLUR), 0),
                       cv2.COLOR_BGR2HSV).astype(np.int16)
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
    # 왜 못 찾았는지를 남긴다. "테이프 없음"만 보면 코스 끝이라 그런 건지, 색이
    # 무너진 건지, 띠가 너무 두꺼운 건지 화면만 봐선 알 수 없다(2026-08-11).
    reasons: list[str] = []
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
        # ★ 구멍 메우기. 조명이 물들면 대비가 무너져 마스크가 **너덜너덜해진다** —
        #   띠 한복판이 숭숭 뚫려 행 채움이 임계 아래로 떨어지고, 남은 조각들이
        #   두께 1~3%짜리 덩어리로 갈린다("가로지름 19%", "두께 3%·0%"가 그 흔적).
        #   구멍만 메우면 같은 프레임이 그대로 살아난다(아래 실측).
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                np.ones((TAPE_CLOSE, TAPE_CLOSE), np.uint8))
        sel = score[mask > 0]
        sep = float(sel.mean() - med_score) if sel.size else 0.0
        if sep < TAPE_MARGIN:
            reasons.append(f"{polarity}: 배경과 구분 안 됨(separation {sep:.0f}<{TAPE_MARGIN:.0f})")
            continue
        # 가로로 화면을 가로지르는 띠가 실제로 만들어지는지 — 두께까지 맞아야 채택.
        rows = (mask > 0).sum(axis=1)
        band = np.nonzero(rows > w * BAND_ROW_FRAC)[0]
        if not band.size:
            # 가장 잘 채워진 행이 화면 폭의 몇 %인지 알려준다 — 코스 끝에서
            # 띠가 화면 밖으로 나가면 이 값이 임계 아래로 떨어진다.
            reasons.append(f"{polarity}: 띠가 화면을 안 가로지름"
                           f"(최대 {100.0 * rows.max() / w:.0f}% < {100 * BAND_ROW_FRAC:.0f}%)")
            continue
        # 자격 행들을 **연속 덩어리로 쪼갠 뒤** 덩어리마다 두께를 잰다.
        # ⚠ 예전엔 band.max() - band.min()으로 한 번에 쟀는데, 그건 띠의 두께가
        #   아니라 **맨 위 자격행에서 맨 아래 자격행까지의 거리**다. 화면에 가로로
        #   긴 것이 하나만 더 있으면 — 밑단에 걸친 원목 마루, 위쪽 마커 판 —
        #   진짜 띠와 통째로 묶여 그 사이 빈 도화지까지 두께로 세어졌다.
        #   2026-08-17 실측: 띠 99px(14%) + 밑단 마루 30px + 사이 258px = 387px(54%)
        #   → 45% 초과로 **멀쩡한 띠가 16프레임 중 15프레임에서 버려졌다**(같은 원인
        #   으로 그 전엔 자주색 마커 판이, 그 전엔 마루 경계가 물었다). 덩어리별로
        #   재면 같은 16프레임이 전부 통과한다. 각도도 같이 살아난다 — 387px 범위
        #   에서는 _band_angle의 열 조건이 안 차 angle이 계속 None이었다.
        chunks = _contiguous(band)
        # 덩어리마다 **뚜렷함(separation)과 가로지름**을 따로 잰다. 둘 다 물리적
        # 성질이고, 둘 다 있어야 주행 테이프다 — 아래 순위 계산에 쓴다.
        cands = []
        for r0, r1 in chunks:
            if not (h * MIN_BAND_FRAC <= r1 - r0 + 1 <= h * MAX_BAND_FRAC):
                continue
            sub = mask[r0:r1 + 1] > 0
            chunk_sel = score[r0:r1 + 1][sub]
            csep = float(chunk_sel.mean() - med_score) if chunk_sel.size else 0.0
            cross = float(rows[r0:r1 + 1].mean()) / w
            cands.append((csep * cross, csep, cross, r0, r1))
        if not cands:
            spans = " · ".join(f"{100.0 * (r1 - r0 + 1) / h:.0f}%" for r0, r1 in chunks)
            reasons.append(f"{polarity}: 띠 두께가 범위 밖"
                           f"({100 * MIN_BAND_FRAC:.0f}~{100 * MAX_BAND_FRAC:.0f}%)"
                           f" — 덩어리 {len(chunks)}개, 두께 {spans}")
            continue
        # 덩어리가 여럿이면 **뚜렷함 × 가로지름**이 가장 큰 것이 주행 테이프다.
        # ⚠ 예전엔 **가로지름만** 봤다("마루·그림자는 화면을 덜 가로지른다").
        #   2026-08-18 새 무대에서 그 전제가 반증됐다 — 마룻바닥 경계는 화면
        #   **끝까지** 이어지는데, 진짜 테이프는 위에 붙은 색 마커와 가장자리
        #   때문에 오히려 덜 찬다. 같은 프레임 실측(극성 dark):
        #       자주 테이프 r255-371  가로지름 86.3%  separation 72.4  (V=68 S=83 H=159)
        #       마루 경계   r594-719  가로지름 93.0%  separation 41.6  (V=98 S=25 H=5)
        #   가로지름만 보면 마루가 이겨(93.0>86.3) band_y가 313 대신 656으로 잡혔고,
        #   dy가 +302px로 떠서 정렬이 무대에서 멀어지는 쪽으로 밀었다. 게다가 색
        #   마커 근접 검사가 **틀린 띠**를 기준으로 돌아 멀쩡한 마커까지 버려졌다
        #   (markers_dropped=1) — "테이프는 잡히는데 마커가 없다"의 정체가 이것이다.
        #   뚜렷함을 곱하면 62.5 대 38.7로 갈린다. 테이프는 배경과 **뚜렷하게**
        #   구분되라고 붙이는 물건이니, 그게 마루 경계보다 흐릴 수는 없다.
        #   가로지름을 곱으로 남겨 두는 이유: 뚜렷함만 보면 화면을 조금만 가로지르는
        #   아주 검은 것(전선·그림자)이 이길 수 있다.
        rank, csep, _cross, r0, r1 = max(cands)
        band = np.arange(r0, r1 + 1)
        thick = int(r1 - r0 + 1)
        # 극성도 **선택된 덩어리**의 순위로 겨룬다 — 마스크 전체의 sep은 두 덩어리를
        # 섞어 버려서, 배경이 넓게 물든 극성이 이길 수 있다.
        if best is None or rank > best[0]:
            best = (rank, polarity, mask, rows, band, thick, thr, csep)

    if best is None:
        tape = np.zeros((h, w), np.uint8)
        separation, polarity, tape_thr, band_rank = 0.0, None, 0.0, 0.0
        rows = np.zeros(h, dtype=int)
        band_rows = np.array([], dtype=int)
    else:
        # separation은 **선택된 띠 덩어리**의 뚜렷함이다(마스크 전체가 아니라).
        band_rank, polarity, tape, rows, band_rows, _thick, tape_thr, separation = best

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

    # --- 띠는 한 프레임에 순간이동하지 않는다 ---
    ok, jump_reason = _band_is_continuous(band_y)
    if not ok:
        found, band_y, thickness, angle = False, None, None, None
        reasons.append(jump_reason)

    # --- 띠가 가로로 **어디까지 이어지는가** ---
    # 끝점(주황)에 섰을 때 "어느 쪽 끝인가"를 이걸로 안다. 코스는 한쪽으로만
    # 이어지므로, 테이프가 오른쪽에만 있으면 여기가 **왼쪽 끝**이다.
    # 이건 절대 관측이라 진행 방향을 몰라도, 손으로 옮겼어도 항상 맞는다
    # (예전엔 진행 방향으로 추측해서 반대로 찍히곤 했다 — 2026-08-11).
    band_left_frac = band_right_frac = None
    if found and band_rows.size:
        strip = tape[int(band_rows.min()):int(band_rows.max()) + 1, :] > 0
        cols = strip.mean(axis=0) >= BAND_COL_FRAC      # 열마다 띠가 있나
        half = w // 2
        band_left_frac = round(float(cols[:half].mean()), 3)
        band_right_frac = round(float(cols[half:].mean()), 3)

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
        # 채도 + **정의된 hue**. 밝기(dark_mask)로 가르면 어두운 초록이 같이 죽는다.
        color = (((S > color_thr) & _role_pixels(H)) * 255).astype(np.uint8)
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
    # 띠에서 세로로 먼 마커는 코스 밖(타일 등) — 버리되, 몇 개 버렸는지는 남긴다.
    markers_dropped = 0
    if band_y is not None and markers:
        half_band = (thickness or 0) / 2
        near_band = [m for m in markers
                     if abs(m["y"] - band_y) <= half_band + m["h"] / 2 + COLOR_NEAR_GAP]
        markers_dropped = len(markers) - len(near_band)
        markers = near_band
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
        # 띠가 화면 좌/우 절반에서 차지하는 가로 비율. 끝점에서 어느 쪽 끝인지를
        # 가른다 — 큰 쪽이 "코스가 계속되는 방향"이다.
        "band_left_frac": band_left_frac,
        "band_right_frac": band_right_frac,
        # 못 찾았을 때 **왜**인지. 찾았으면 None.
        "found_reason": None if found else (" · ".join(reasons) or "테이프 후보 없음"),
        "target_y": round(target_y, 1),
        "end_marker": end_marker,
        "color_marker": color_marker,
        "markers": markers,          # 화면에 보이는 지점 표식 전부(x 오름차순, 띠 근접만)
        "markers_dropped": markers_dropped,   # 띠에서 멀어 버린 색 덩어리 수(코스 밖 의심)
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
        # 선택된 띠 덩어리의 뚜렷함, 그리고 그것이 이긴 순위(뚜렷함 × 가로지름).
        # 가짜 띠(마루 경계 등)와 겨룰 때 무엇으로 이겼는지 숫자로 남긴다.
        "separation": round(separation, 1),
        "band_rank": round(band_rank, 1),
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
        # 띠가 좌/우로 얼마나 이어지는지 — 끝점 판정의 근거를 눈으로 확인할 수 있게.
        if st.get("band_left_frac") is not None:
            txt += f"  tape L{st['band_left_frac']:.2f} R{st['band_right_frac']:.2f}"
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
