"""무대/하드웨어/모델 설정값을 한 곳에 모은다.

부스 데모는 결정적이어야 하므로, 나무 위치·색 임계값 같은 상수는
코드 곳곳에 흩어두지 말고 여기서만 바꾼다.
"""

from __future__ import annotations

# --- Claude (오케스트레이션) ---
# 최신·최상위 모델. adaptive thinking과 함께 사용한다.
CLAUDE_MODEL = "claude-opus-4-8"

# 열매 인식 방식.
#   False = HSV 색검출 (오프라인·빠름·CUDA 불필요) — 기본/폴백
#   True  = Claude 비전 (이미지를 Claude에 보내 판단·온라인) — 색만으로 애매할 때
# 둘 다 같은 list[Fruit]를 돌려주므로 scan_tree가 투명하게 갈아끼운다.
USE_LLM_VISION = False

# --- 베이스(메카넘) 시리얼 연결 ---
# True면 main.build_robot()이 MockBase 대신 JetsonBase(실물)를 쓴다.
USE_REAL_BASE = False
# Orin에서 Arduino Uno는 보통 /dev/ttyACM0. Windows 테스트면 "COM5" 식.
BASE_SERIAL_PORT = "/dev/ttyACM0"
BASE_SERIAL_BAUD = 115200
# 엔코더 보정값: 바퀴를 알려진 거리(예 50cm)만큼 굴려 나온 틱 ÷ 거리.
# 처음엔 대략값, 실측 후 교정한다. TREES 단위(cm)와 맞춘다.
BASE_TICKS_PER_CM = 20.0

# --- 무대 구성 ---
# 나무 모형 3개 일렬. id → 베이스가 직선 이동할 목표 거리(cm 등 임의 단위).
TREES: dict[int, float] = {
    1: 0.0,
    2: 40.0,
    3: 80.0,
}

# --- 색 검출 임계값 (HSV) ---
# 빨강은 Hue가 0과 180 양끝으로 갈라져 두 구간을 OR로 합친다.
RED_HSV_RANGES = [
    ((0, 120, 70), (10, 255, 255)),
    ((170, 120, 70), (180, 255, 255)),
]
GREEN_HSV_RANGE = ((35, 80, 40), (85, 255, 255))

# 노이즈 제거: 이 픽셀 면적보다 작은 덩어리는 열매로 보지 않는다.
MIN_FRUIT_AREA_PX = 400
