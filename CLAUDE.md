# CLAUDE.md — 에이전트 오리엔테이션

토마토 따기 로봇(LG Physical AI 데모). "언어 → 인식 → 행동". 상세 배경은
[README.md](README.md) · [PROJECT_토마토로봇.md](PROJECT_토마토로봇.md) · [스마트로봇_명세서.md](스마트로봇_명세서.md).

## 소프트웨어 진입점
- `main.py` (demo/run) → `src/tomato_picker/` (skills.py=스킬5종, orchestrator.py=Claude tool-use,
  demo_mode.py=오프라인 폴백, hardware/=Base·Arm·Camera 추상화, vision/=색검출).

## 하드웨어 제어 — **현재 구성 (중요: 옛 문서와 혼동 주의)**
로봇 = **메카넘 베이스(Moebius Uno + PCA9685)** + **SO-101 팔로워(집게)** + 젯슨 Orin Nano.

- **주행 펌웨어 = [`firmware/mecanum_stable/`](firmware/mecanum_stable/) v2** ✅ (아두이노/CH340, `/dev/ttyUSB0`).
  젯슨이 **시리얼 라인 프로토콜**로 구동: `V vx vy w`(속도, -255..255) / `S`(정지) / `T i d`(모터캘리)
  / `P n`(PWM) / `L a b c d`(극성) / `R accel decel`(슬루) / `F hz`(PWM주파수). baud 115200.
  **v2 = 소음·끊김 대책**: PWM 50Hz→1500Hz(“우우웅”의 원인), 펌웨어 슬루(가감속),
  소프트 데드맨(300ms 감속/1000ms 하드), **XOR 체크섬 `*HH`**(한 번 받으면 무체크섬 `V` 거부),
  `V` 무응답 + 1초 하트비트에 링크 카운터. 젯슨 쪽은
  [`hardware/motor_link.py`](src/tomato_picker/hardware/motor_link.py)가 **상시 연결 + 20ms 재전송
  스레드 + 자동 재연결**로 물린다. ⚠ 포트는 **한 프로세스만**(`exclusive=True`) —
  `tomato-voice`와 `controller-drive`를 동시에 켜지 말 것.
  ⚠ **소음↔속도는 맞바꾸는 관계**(PCA9685 상한 1526Hz라 초음파 불가). 대시보드
  "모터 튜닝" 패널에서 `F`(주파수)/`P`(듀티상한)/`R`(슬루)을 재플래시 없이 바꿔 찾는다.
  현재 기본 700Hz·듀티 2600. ⚠ 포트를 열면 커널 DTR 토글로 **Uno가 리셋**되므로
  설정은 **첫 하트비트 이후**에 넣어야 먹는다(부트로더 ~2초가 다 먹어버림).
  ⚠️ **`firmware/PS2-MOTOR/`는 레거시**(PS2 직접조종, 불안정). `mecanum_serial/`은 폐기. 새 작업은 `mecanum_stable` 기준.
  모터↔바퀴 실측: A=FR B=FL C=RR D=RL, `POL={-1,1,-1,1}`.
- **게임패드 주행 + 팔 프리셋 = [`tools/controller_drive.py`](tools/controller_drive.py)** (젯슨, lerobot venv).
  Switch 프로콘/Xbox360 게임패드(evdev)를 읽어 → 메카넘 `V` 지령 + 팔 프리셋 재생.
  **systemd `controller-drive` 서비스로 부팅 자동실행**([`deploy/`](deploy/)). 조작: **LT(ZL) 데드맨=누르는 동안만 주행**,
  왼스틱=이동/오른스틱=회전/D패드=디지털, **LB/RB=팔 프리셋(이전/다음)**. 셋업·이슈=[`docs/jetson-gamepad-setup.md`](docs/jetson-gamepad-setup.md).
- **팔 제어 = 대시보드 `/control`** ✅ 권장 — 리더암 연결·미러링·프리셋 **슬롯 0~9** 저장/재생·
  **높이 앵커 보간**을 브라우저에서 전부 한다([`docs/dashboard-presets.md`](docs/dashboard-presets.md)).
  구현: [`hardware/presets.py`](src/tomato_picker/hardware/presets.py)(저장소·보간) +
  [`hardware/arm.py`](src/tomato_picker/hardware/arm.py)(미러링·재생).
  프리셋=`~/arm_presets.json` — 옛 형식 호환, 이름·앵커는 `"__meta__"` 키에.
  대안: [`tools/mirror_toggle.py`](tools/mirror_toggle.py)(터미널·리더 필요, 레거시) 또는
  `controller_drive.py`의 프리셋 재생(리더 없이 팔로워만).
- **hid-nintendo**: 이 젯슨 커널엔 없어서 out-of-tree 빌드해 설치함(게임패드 필수). 빌드법=`docs/jetson-gamepad-setup.md`.

## 젯슨 접속
- `ssh server@192.168.0.8` (IP는 DHCP라 바뀔 수 있음 / 공개키 등록·passwordless sudo).
  Bash 툴은 `-i /c/Users/parkg/.ssh/id_ed25519` 명시 필요(HOME 다름).
- 장치: `/dev/ttyUSB0`=모터보드, `/dev/ttyACM0`=팔로버, 게임패드=USB/BT.

## 알려진 함정
- **USB 접촉/전원 불안정**: 게임패드 `error -71`(케이블 손상), CH340 반복 disconnect. `controller_drive.py`는
  모터 시리얼 **자동재연결** 내장. 근본은 양품 케이블 + 안정 전원(전류제한 넉넉히).
- **PS2X는 이 보드에서 `millis()`를 얼림**(timer0 간섭) → 주행 펌웨어에서 PS2 완전 배제, 안전은 HW워치독.
- 게임패드 스틱 드리프트/latch 폭주 방지 위해 **데드맨(LT 홀드) 필수 설계**.

## Git
- origin: `github.com/ttkpark/tomato-picker`. 기본 브랜치 `master`.
