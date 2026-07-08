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
USE_REAL_BASE = True
# ⚠ ttyACM0은 팔(SO-101 Follower) 포트다 — 베이스는 CH340이라 ttyUSB0
# (2026-07-08 확인, firmware/README.md 참고). 예전엔 이게 틀려있었음.
BASE_SERIAL_PORT = "/dev/ttyUSB0"
BASE_SERIAL_BAUD = 115200
# 실제 펌웨어는 firmware/mecanum_stable/mecanum_stable.ino — 젯슨이
# "V vx vy w" 속도 지령을 400ms마다 새로 보내야 하는 데드맨 프로토콜.
# (예전 BASE_TICKS_PER_CM/JetsonBase.drive_to는 mecanum_serial.ino의
# G-ticks 프로토콜을 가정했는데 실제 보드는 그 펌웨어를 쓴 적이 없어
# 애초에 동작한 적 없는 코드였음 — 2026-07-08 확인 후 폐기.)
BASE_DRIVE_SPEED = 220            # V vx 크기. -255..255. 2026-07-08 150→220(사용자 확인 후 상향).
BASE_DRIVE_RESEND_INTERVAL_SEC = 0.15  # 데드맨(400ms) 안에 재전송해야 함
BASE_DRIVE_FORWARD_SECONDS = 1.5  # 음성 "앞으로 가" 기본 전진 시간

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
USE_REAL_ARM = True
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
# 마이크 카드가 재부팅/USB 재연결/장치 교체마다 번호와 이름이 다 바뀌는 게
# 실측으로 확인돼(2026-07-08: 카메라 card 2→0, 이후 웹캠으로 교체하니
# 이름도 "Camera"→"webcam"), 특정 이름을 하드코딩하지 않는다. 대신
# `arecord -l`에서 젯슨 내장 오디오(APE)가 아닌 첫 카드를 자동으로 쓴다
# (mic_stream.resolve_alsa_device). 못 찾으면 아래 폴백 번호를 쓴다.
MIC_ALSA_EXCLUDE_NAME = "APE"  # 젯슨 내장 오디오 — 이건 마이크가 아니라 제외
MIC_ALSA_DEVICE_FALLBACK = "hw:0,0"
# 원래 카메라(2ce5:c672)는 plughw로 열면 클럭 버그로 완전 무음이 잡혀서
# (mic_stream.py 모듈 docstring 참고) hw(raw)로 광고 레이트 그대로 받고
# 코드에서 다운샘플하는 우회책을 씀. 지금 쓰는 마이크가 16kHz를 네이티브
# 지원하면(예: 교체한 웹캠, `/proc/asound/card0/stream0`로 확인) 아래를
# 16000으로 맞춰 다운샘플을 생략해도 된다 — factor=1이라 코드는 그대로 동작.
MIC_NATIVE_SAMPLE_RATE = 16000
MIC_SAMPLE_RATE = 16000

# 에너지 기반 발화 구간 검출(VAD). 마이크마다 바닥 노이즈 레벨이 크게
# 달라서(카메라 마이크는 ~120, 웹캠은 ~1500~1800) 마이크 바꾸면 반드시
# VOICE_LOG_HTTP_PORT 페이지에서 rms 로그 보며 재조정해야 한다.
VAD_RMS_THRESHOLD = 3000.0
VAD_MIN_SPEECH_SEC = 0.3   # 이보다 짧은 소리는 잡음으로 버림
VAD_SILENCE_HANGOVER_SEC = 0.6  # 발화 끝난 뒤 이만큼 조용하면 구간 종료
VAD_MAX_UTTERANCE_SEC = 8.0     # 한 발화 최대 길이(안전장치)
# 발화 시작 직전 이만큼을 항상 앞에 붙인다 — "팔"같은 파열음 초성이
# 트리거 순간에 잘리는 걸 방지(2026-07-08 실측, vad.py 참고).
VAD_PRE_ROLL_SEC = 0.2

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
# "팔"이 STT에서 "파는"/"잘" 등으로 자주 오인식되는 게 실측으로 확인돼
# (2026-07-08) "팔"을 필수로 요구하지 않고 "움직여" 계열만으로도 매칭한다.
VOICE_INTENTS: dict[str, list[str]] = {
    "arm_move": [
        "팔 움직여", "팔 움직여줘", "팔움직여", "팔 이동", "팔 이동해",
        "움직여줘", "움직여봐", "움직여라", "움직여", "움직이라",
    ],
    # 실제로 바퀴가 구르는 명령이라 오작동 리스크가 더 크다 — "가"처럼
    # 짧고 흔한 음절 하나만으로는 매칭하지 않고 "전진"/"앞으로" 계열만 인정.
    "drive_forward": [
        "앞으로 가", "앞으로가", "앞으로 가자", "앞으로가자",
        "전진해", "전진 해", "전진", "앞으로 이동",
    ],
    # "토마토" 한 단어로 수확 시퀀스 전체(전진→프리셋→후진) 트리거.
    "tomato_pick": ["토마토"],
}

# --- "토마토" 명령 시퀀스: 전진(하드코딩 거리) → 팔 프리셋 → 후진(복귀) ---
# 비전으로 토마토 위치를 찾는 게 아니라 고정 시간만큼 이동하는 임시 버전.
# 실측 후 시간을 조정한다.
TOMATO_APPROACH_SECONDS = 1.0  # 토마토 앞까지 전진
TOMATO_RETREAT_SECONDS = 1.0   # 대기 위치로 후진 복귀

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
