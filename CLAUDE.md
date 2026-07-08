# CLAUDE.md — 에이전트 오리엔테이션

토마토 따기 로봇(LG Physical AI 데모). "언어 → 인식 → 행동". 상세 배경은
[README.md](README.md) · [PROJECT_토마토로봇.md](PROJECT_토마토로봇.md) · [스마트로봇_명세서.md](스마트로봇_명세서.md).

## 소프트웨어 진입점
- `main.py` (demo/run) → `src/tomato_picker/` (skills.py=스킬5종, orchestrator.py=Claude tool-use,
  demo_mode.py=오프라인 폴백, hardware/=Base·Arm·Camera 추상화, vision/=색검출).

## 하드웨어 제어 — **현재 구성 (중요: 옛 문서와 혼동 주의)**
로봇 = **메카넘 베이스(Moebius Uno + PCA9685)** + **SO-101 팔로워(집게)** + 젯슨 Orin Nano.

- **주행 펌웨어 = [`firmware/mecanum_stable/`](firmware/mecanum_stable/)** ✅ (아두이노/CH340, `/dev/ttyUSB0`).
  젯슨이 **시리얼 라인 프로토콜**로 구동: `V vx vy w`(속도, -255..255) / `S`(정지) / `T i d`(모터캘리)
  / `P n`(PWM) / `L a b c d`(극성). 데드맨(400ms 무명령→정지) + HW워치독. baud 115200.
  ⚠️ **`firmware/PS2-MOTOR/`는 레거시**(PS2 직접조종, 불안정). `mecanum_serial/`은 폐기. 새 작업은 `mecanum_stable` 기준.
  모터↔바퀴 실측: A=FR B=FL C=RR D=RL, `POL={-1,1,-1,1}`.
- **게임패드 주행 + 팔 프리셋 = [`tools/controller_drive.py`](tools/controller_drive.py)** (젯슨, lerobot venv).
  Switch 프로콘/Xbox360 게임패드(evdev)를 읽어 → 메카넘 `V` 지령 + 팔 프리셋 재생.
  **systemd `controller-drive` 서비스로 부팅 자동실행**([`deploy/`](deploy/)). 조작: **LT(ZL) 데드맨=누르는 동안만 주행**,
  왼스틱=이동/오른스틱=회전/D패드=디지털, **LB/RB=팔 프리셋(이전/다음)**. 셋업·이슈=[`docs/jetson-gamepad-setup.md`](docs/jetson-gamepad-setup.md).
- **팔 제어 = [`tools/mirror_toggle.py`](tools/mirror_toggle.py)**(리더→팔로워 미러링+프리셋; 리더 필요) 또는
  `controller_drive.py`의 프리셋 재생(리더 없이 팔로워만). 프리셋=`~/arm_presets.json`(슬롯 1~4).
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
