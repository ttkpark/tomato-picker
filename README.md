# tomato-picker

LG Physical AI 부스용 **토마토 따기 로봇** 데모 소프트웨어.
음성/텍스트 명령을 Claude가 받아 → 로봇이 색검출로 익은 토마토를 찾고 →
직접 따서 바구니에 담는다. (개념: *"언어 → 인식 → 행동"*)

> 설계 원칙: AI는 **두 층**(① 색 기반 열매 인식 ② Claude 함수콜 계획)에만.
> 이동·집기 등 저수준 동작은 **결정적 스킬 함수**로 처리한다.
> 자세한 배경은 [스마트로봇_명세서.md](스마트로봇_명세서.md), [PROJECT_토마토로봇.md](PROJECT_토마토로봇.md) 참고.

## 구조

```
main.py                         진입점 (demo / run)
src/tomato_picker/
├── config.py                   나무 위치·색 임계값·모델 ID
├── skills.py                   스킬 함수 5종 (핵심 API)
├── orchestrator.py             Claude tool-use 루프 (온라인)
├── demo_mode.py                오프라인 자동 시퀀스 (폴백, 필수)
├── hardware/                   하드웨어 추상화
│   ├── base.py                 인터페이스 (Base/Arm/Camera)
│   ├── mock.py                 PC용 Mock 구현
│   └── jetson.py               젯슨(실장비) 구현
└── vision/
    ├── color_detect.py         HSV 색검출 (빨강=익음/초록=미익음)
    └── claude_vision.py        Claude 비전 보조

firmware/                       아두이노 펌웨어 (메카넘 주행 + 팔 보조) → firmware/README.md
tools/mirror_toggle.py          젯슨: SO-101 미러링 + 모션 프리셋 (PS2 패드 연동)
```

핵심: 스킬 함수 5종(`drive_to_tree` · `scan_tree` · `pick_fruit` ·
`place_in_basket` · `home`)을 온라인(Claude)과 오프라인(데모) 경로가
**공유**하므로 두 모드의 동작이 일관된다.

## 설치

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## 실행

```powershell
# 오프라인 데모 (망/API 키 불필요) — 지금 바로 실행 가능
python main.py demo

# Claude 오케스트레이션 (.env에 ANTHROPIC_API_KEY 필요)
copy .env.example .env   # 키 입력
python main.py run "익은 토마토 다 따줘"
```

현재는 **Mock 하드웨어**로 동작한다(합성 영상으로 색검출까지 전부 검증).
실제 장비가 오면 `main.py`의 `build_robot()`에서 하드웨어 구현만 교체한다.

## 하드웨어 제어 (실장비 — 메카넘 베이스 + SO-101 팔)

실장비는 **젯슨 Orin Nano**(JetPack)에서 구동한다. 두 갈래로 제어한다:

- **메카넘 베이스 + 팔 보조 입력** — Moebius Uno + PS2 패드. 펌웨어
  [firmware/PS2-MOTOR](firmware/PS2-MOTOR/) 가 PS2로 본체를 직접 굴리고, L1/R1 버튼은
  시리얼로 젯슨에 신호를 보낸다. 빌드·조작·핀맵은 [firmware/README.md](firmware/README.md).
- **SO-101 팔 (LeRobot)** — 리더→팔로워 텔레오프 미러링 + 모션 프리셋.
  젯슨에서 [tools/mirror_toggle.py](tools/mirror_toggle.py) 실행:

  | 조작 | 동작 |
  |---|---|
  | PS2 방향키 / □○ | 본체 주행 / 평행이동 |
  | **L1** | 팔 미러링 ON(리더 추종) ↔ OFF(limp) 토글 |
  | **R1 + △○✕□** | 미러링 ON: 현재 자세 **저장** / OFF: 저장 자세 **재생** (슬롯 1~4) |

  프리셋은 `~/arm_presets.json`에 영속. **토마토 따기 = 프리셋 1→2→3→4 순차 재생.**

> 셋업 메모(LeRobot 설치, 모터 ID·캘리브레이션, wrist_roll 엔코더 보정, CH340 드라이버
> 빌드 등)는 개발자 노트 참조. 실장비 구현은 `hardware/jetson.py`.

## 다음 단계

- [x] SO-101 팔 셋업(모터 ID·캘리브레이션) + 텔레오프 미러링 + 모션 프리셋
- [x] 메카넘 베이스 PS2 주행 펌웨어
- [ ] 모션 프리셋 시퀀스 **원버튼 자동 재생**(1→2→3→4) — 반복 수확
- [ ] 비전(토마토 검출) → 팔 동작 좌표 연동 / 시퀀스 자동 트리거
- [ ] `hardware/jetson.py`를 스킬 함수(`pick_fruit` 등)에 완전 연결
- [ ] 색 임계값을 실제 조명/열매로 튜닝 (`config.py`)
- [ ] (선택) 음성 입력(STT) → `run` 명령 연결
