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

# --- 카메라(USB, 지면 고정 스탠드 — 베이스에 안 붙어있어 위치 불변) ---
# True면 main.build_robot()이 MockCamera 대신 JetsonCamera(실물)를 쓴다.
USE_REAL_CAMERA = False
CAMERA_INDEX = 0  # /dev/video0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
# MJPG가 아니면(기본 YUYV) 5fps로 떨어지는 카메라라 반드시 MJPG로 연다.
CAMERA_FOURCC = "MJPG"
# 자동노출이 안정되기 전 프레임은 어둡고 파랗게 나와 버린다.
CAMERA_WARMUP_FRAMES = 20
# 부착/낙과 판정 기준선: 이 y좌표(px) 아래는 "바닥"으로 본다.
# 카메라가 지면에 고정돼 위치가 안 변하므로 한 번만 실측해 교정하면 된다.
CAMERA_GROUND_LINE_Y = 600

# --- 로봇팔(SO-101 Follower, 프리셋 재생) ---
# True면 main.build_robot()이 MockArm 대신 LerobotArm(실물)을 쓴다.
USE_REAL_ARM = False
ARM_SERIAL_PORT = "/dev/ttyACM0"
ARM_ID = "tomato_follower"
# controller_drive.py가 쓰는 것과 같은 프리셋 파일(PS2 컨트롤러로 저장).
ARM_PRESET_FILE = "~/arm_presets.json"
# 프리셋 1→2→3→4 = 접근→집기→들기→놓기(2026-06-27 확인된 전체 수확 시퀀스).
# pick_fruit()가 1,2(접근+집기)를, place_in_basket()이 3,4(들기+놓기)를 재생한다.
# ⚠ 추정 매핑 — 실기 테스트 후 다르면 여기 숫자만 바꾸면 된다.
ARM_PICK_PRESETS = [1, 2]
ARM_PLACE_PRESETS = [3, 4]
ARM_HOME_PRESET = 1
ARM_MOVE_SECS = 1.5
ARM_MOVE_FPS = 50

# --- 음성 명령 (온디바이스 STT, Jetson 단독 추론) ---
# 카메라 내장 마이크(2ce5:c672). ALSA 카드 번호가 재부팅/USB 재연결마다
# 바뀌는 게 실측으로 확인돼(2026-07-08: card 2 → card 0), 번호 대신
# `arecord -l` 카드 이름으로 찾는다(mic_stream.resolve_alsa_device).
# 이름으로도 못 찾으면 아래 폴백 번호를 쓴다 — 그때그때 `arecord -l`로 갱신.
# ⚠ 반드시 hw(raw), plughw 아님 — 이유는 mic_stream.py 모듈 docstring 참고
# (plug 리샘플 레이어를 거치면 이 카메라는 완전 무음이 잡힘, 2026-07-08 실측).
MIC_ALSA_CARD_NAME = "Camera"
MIC_ALSA_DEVICE_FALLBACK = "hw:0,0"
# 카메라가 광고하는 48kHz 그대로 raw로 받고, 코드에서 16kHz로 다운샘플한다.
MIC_NATIVE_SAMPLE_RATE = 48000
MIC_SAMPLE_RATE = 16000

# 에너지 기반 발화 구간 검출(VAD). 조용한 방 기준 대략값 — 소음 있으면
# VOICE_LOG_HTTP_PORT 페이지에서 rms 로그 보며 재조정.
VAD_RMS_THRESHOLD = 500.0
VAD_MIN_SPEECH_SEC = 0.3   # 이보다 짧은 소리는 잡음으로 버림
VAD_SILENCE_HANGOVER_SEC = 0.6  # 발화 끝난 뒤 이만큼 조용하면 구간 종료
VAD_MAX_UTTERANCE_SEC = 8.0     # 한 발화 최대 길이(안전장치)

# Whisper STT. 이 젯슨의 ctranslate2(aarch64) 빌드는 CUDA를 못 잡아
# CPU(int8)로만 돈다 — tiny가 2초 발화 기준 추론 ~8초로 그나마 쓸만하다.
WHISPER_MODEL_SIZE = "tiny"
WHISPER_DEVICE = "cpu"
WHISPER_COMPUTE_TYPE = "int8"
WHISPER_LANGUAGE = "ko"

# 이 카메라 마이크는 평소 말소리 레벨이 낮아(실측 2026-07-08: 보통 톤은
# 대부분 인식 실패, 크게 외쳐야 인식됨) STT에 넣기 전 피크 정규화로
# 증폭한다. 거의 무음(peak<MIN)까지 증폭하면 잡음만 커지니 그런 구간은
# 건드리지 않는다. GAIN_CAP은 무음에 가까운 낮은 피크가 우연히 MIN을
# 넘었을 때 과증폭(귀청 터지는 잡음)되는 걸 막는 안전장치.
WHISPER_NORMALIZE_TARGET_PEAK = 0.9
WHISPER_NORMALIZE_MIN_PEAK = 0.02
WHISPER_NORMALIZE_GAIN_CAP = 20.0

# 인식된 텍스트에 이 키워드 중 하나라도 포함되면 해당 인텐트로 매칭.
# 순서 무관, 부분 문자열 매칭(짧은 발화라 오검출보다 미검출이 더 걱정이라 느슨하게).
VOICE_INTENTS: dict[str, list[str]] = {
    "arm_move": ["팔 움직여", "팔 움직여줘", "팔움직여", "움직여줘", "팔 이동", "팔 이동해"],
}

# 실시간 인식 로그를 브라우저로 보는 내장 HTTP(SSE) 서버.
# 사용법: 젯슨에서 voice 실행 후 PC 브라우저로 http://192.168.0.8:<포트>
VOICE_LOG_HTTP_PORT = 8090
VOICE_LOG_HISTORY = 200  # 새로고침 시 보여줄 과거 로그 줄 수

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
