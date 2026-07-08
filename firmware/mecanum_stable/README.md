# mecanum_stable — 젯슨 시리얼 주행용 안정 펌웨어 (현재 사용)

토마토피커 메카넘 베이스(Moebius MecanumRobot, Arduino Uno + PCA9685)의 **현재 주행 펌웨어**.
젯슨이 시리얼로 속도지령을 보내면 4바퀴 메카넘을 구동한다. **PS2를 완전히 배제**하고
(불안정 요소 제거) 젯슨↔아두이노 시리얼 링크에 집중한 안정판.

> 기존 `firmware/PS2-MOTOR/`(PS2 직접조종)와 `firmware/mecanum_serial/`(직결핀 가정, 미완)을
> 대체한다. 상위 제어(게임패드/자율)는 젯슨의 `tools/controller_drive.py`가 담당.

## 하드웨어
- **Arduino Uno (CH340)** + **PCA9685(I2C)** 모터 드라이버. 라이브러리: `FaBo_PWM_PCA9685`.
  (⚠️ `src/`의 `*(1)*` 중복파일 삭제 필수 — 안 하면 ld 중복심볼 에러)
- 젯슨에서 `/dev/ttyUSB0`(CH340 `1a86:7523`), Windows `COM*`.
- **모터↔바퀴 실측 매핑(2026-07-03)**: PCA9685 채널 기준 **A=FR(앞우), B=FL(앞좌), C=RR(뒤우), D=RL(뒤좌)**.
  각 바퀴 물리전진 극성 보정 `POL[4] = {-1, 1, -1, 1}` (우측 A·C 반전 → 전진 정합. 실측 확정).

## 시리얼 프로토콜 (115200 baud, 라인단위 `\n`)
```
V <vx> <vy> <w>   속도지령. vx=전진+, vy=우평행+, w=시계회전+. 각 -255..255
S                 즉시 정지
T <i> <d>         모터 i(0=A/FR 1=B/FL 2=C/RR 3=D/RL) 방향 d(1/-1) 단독 1초 (방향 캘리브레이션)
P <n>             최대 PWM 스케일 설정(기본 2000)
L <a> <b> <c> <d> 런타임 극성 변경 (POL, 각 1/-1). 방향 캘리브레이션용
?                 상태 1줄 출력
```
응답: 명령 처리 시 `ok ...`, 1초마다 `hb <millis>` 하트비트.

메카넘 mixing: `FL=vx-vy-w, FR=vx+vy+w, RL=vx+vy-w, RR=vx-vy+w` → 바퀴별 `POL` 적용.
(strafe/회전 부호는 실기에서 `L`·`T`로 최종 확정할 것. 전진만 확정됨.)

## 안정성 4중 방어 (이 HW의 실패들에서 도출)
1. **데드맨**: 400ms 내 새 `V`/`S` 없으면 자동 `STOP` — 상위(젯슨) 끊겨도 폭주 없음.
2. **하드웨어 워치독** `WDTO_250MS` + 매 loop `wdt_reset()` — I2C/코드 행(hang) 자동복구.
   (이 HW의 안정 하한. 120ms는 PS2X 없이도 가끔 초과, 250ms가 오리셋 0.)
3. **논블로킹 loop** — 긴 delay 없음.
4. **PS2 완전 분리** — PS2X 비트뱅잉이 이 보드에서 `millis()`를 얼리는(timer0 간섭) 문제가
   있어 주행 펌웨어에서 제거. 안전은 HW 워치독(레지스터 기반, millis 무관)으로.

## 빌드 / 업로드
```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/mecanum_stable
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno firmware/mecanum_stable   # 젯슨
# Windows: -p COM14 등
```
필요 라이브러리: `FaBo_PWM_PCA9685`(중복파일 제거), `PS2X_lib`는 이 펌웨어엔 불필요.

## 상위 통합
젯슨의 [`tools/controller_drive.py`](../../tools/controller_drive.py)가 게임패드(evdev)를 읽어
이 프로토콜(`V`/`S`)로 주행 + 팔 프리셋 재생. 젯슨 셋업(hid-nintendo 빌드, systemd 자동실행,
알려진 이슈)은 [`docs/jetson-gamepad-setup.md`](../../docs/jetson-gamepad-setup.md) 참고.
