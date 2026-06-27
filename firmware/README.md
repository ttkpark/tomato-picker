# firmware — 아두이노 펌웨어

토마토피커의 하드웨어 보조 제어를 담당하는 Arduino(Uno) 펌웨어 모음.

| 폴더 | 상태 | 설명 |
|---|---|---|
| **`PS2-MOTOR/`** | ✅ **현재 사용** | 메카넘 주행(PS2 수동조종) + L1 미러링 토글 + R1 모션 프리셋 통합 |
| `mecanum_serial/` | 참고용 | 젯슨이 시리얼 `G <ticks>` 명령으로 직선 이동시키는 자율주행 컨셉. ⚠️ 직결 HR8833 핀 가정 — **이 보드(PCA9685)와 안 맞아 재작성 필요** |

대상 보드: **Moebius MecanumRobot (Arduino Uno, CH340 클론)** + PS2 무선 컨트롤러.
모터는 **PCA9685(I2C)** 경유, PS2 핀은 `CLK=12 / CMD=11 / SEL=10 / DAT=13`.

---

## PS2-MOTOR (현재 펌웨어)

한 보드에서 **본체 주행 + 팔 제어 보조**를 모두 처리한다. 주행은 아두이노가
자체적으로(PS2→PCA9685), 팔 미러링/프리셋은 **시리얼로 젯슨에 신호만** 보내고
실제 팔 동작은 젯슨([../tools/mirror_toggle.py](../tools/mirror_toggle.py))이 LeRobot으로 수행한다.

### 조작

| 입력 | 동작 |
|---|---|
| **↑ / ↓** | 전진 / 후진 (누른 동안만) |
| **← / →** | 좌회전 / 우회전 |
| **□ / ○** | 좌 / 우 평행이동(strafe) — *R1 안 누른 상태* |
| **L1** | 팔 미러링 ON/OFF 토글 → 시리얼 `TOGGLE` |
| **R1 + △/○/✕/□** | 모션 프리셋 1/2/3/4 → 시리얼 `PRESET 1~4` |

- 주행은 **누르고 있는 동안만** 이동, 떼면 즉시 정지.
- 미러링 ON에서 `R1+버튼` = 현재 팔 자세 **저장**, OFF에서 = 저장 자세 **재생**(판단은 젯슨).

### 시리얼 프로토콜 (115200 baud, 줄단위)

```
Arduino → "TOGGLE"     L1 눌림 (미러링 토글)
Arduino → "PRESET <n>" R1+면버튼 (1=△ 2=○ 3=✕ 4=□)
Arduino → "Found Controller..." / "Controller_type: N"  부팅 배너
```

### 견고성 처리 (이 보드 특유 이슈 대응)

- **PS2 핀13 노이즈**: DAT가 온보드 LED와 핀을 공유해 한 프레임 걸러 손상프레임이
  섞인다 → ① 아날로그 4축이 모두 255/0 이거나 버튼 7개↑ 동시눌림이면 프레임 스킵
  ② 방향키 뗌은 2프레임 디바운스, 프리셋/토글은 엣지검출+리프랙토리.
- **컨트롤러 무응답 시**: 원본은 `resetFunc()`로 무한 재부팅 → 제거하고 **재부팅 없이
  `config_gamepad` 재시도**.
- **워치독**: 주행 중 보드가 행되면 PCA9685가 마지막 PWM을 래치해 **폭주**할 수 있어
  `WDTO_250MS` 워치독 적용(0.25초 내 자동 리셋→정지). 120ms는 PS2X 재동기화 지연으로
  오리셋 발생해 250ms가 안정 하한.
- **속도**: `Motor_PWM = 600`(약 30%). 전류↓ → 전원 출렁임/행 빈도↓.
  근본적으로 brownout이 잦으면 **BAT(7.4~11.1V LiPo) 충전/굵은 전원선**이 필요.

---

## 빌드 / 업로드 (arduino-cli)

젯슨/PC 어디서든 `arduino-cli`로 가능. 필요한 라이브러리:

- **PS2X_lib** — github madsci1016/Arduino-PS2X
- **FaBo_PWM_PCA9685** — Moebius 키트 동봉. ⚠️ `src/`의 `*(1)*` **중복 파일 삭제 필수**
  (안 하면 ld 중복심볼 에러)

```bash
# 컴파일
arduino-cli compile --fqbn arduino:avr:uno firmware/PS2-MOTOR

# 업로드 (포트는 환경에 맞게: 젯슨=CH340→/dev/ttyUSB0, Windows=COM8 등)
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno firmware/PS2-MOTOR
```

> 젯슨(JetPack)에서 CH340이 `ttyUSB`로 안 잡히면 커널에 `ch341` 모듈이 없는 것.
> 소스로 빌드해 넣어야 한다(메인 [README](../README.md) 또는 배포 노트 참고).

---

## 팔 제어 연동

젯슨에서 [../tools/mirror_toggle.py](../tools/mirror_toggle.py)를 실행하면 이 펌웨어의
`TOGGLE`/`PRESET` 신호를 받아 SO-101 리더→팔로워 미러링과 모션 프리셋(저장/재생)을 수행한다.
자세한 사용법은 그 스크립트의 docstring 참고.
