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

## v2 (현재) — 소음·끊김 대책

v1에서 "**우우웅**" 소리와 "주웅 주웅 주우웅" 끊김이 났다. 원인은 둘이었고 v2가 둘 다 잡는다.

| 증상 | 원인 | v2의 조치 |
|---|---|---|
| 주행 중 저주파 "우우웅" | **PWM 50Hz** — 사람이 그대로 듣는 대역. 모터가 초당 50번 끊기며 도는 소리 | `set_hz(1500)` → PCA9685 상한(실제 1526Hz)으로. 가청 저역 이탈 + 토크 리플 감소 |
| 주행이 뚝뚝 끊김 | 데드맨 400ms 초과 시 **즉시 하드 정지** → 다음 지령에 재출발 | 소프트 데드맨(300ms에 목표만 0으로, 슬루로 감속) + 하드는 1000ms |
| 기동 시 젯슨 브라운아웃 | 정지→목표속도를 한 번에 인가 = 4륜 인러시 동시 피크 | **펌웨어 슬루**(5ms 틱, 틱당 ACCEL=6) — 젯슨 지터와 무관하게 매끄러움 |
| 깨진 프레임이 그대로 실행 | 검증 없음 | **XOR 체크섬** + strict 모드 자동 전환 |
| 50Hz 지령마다 `ok` 응답 | TX 버퍼 포화 → loop 블로킹 | `V`는 무응답. 상태는 1초 하트비트에 실어 보냄 |

> ⚠ 1526Hz에서 드라이버가 뜨거워지거나 저속에서 기동을 못 하면 런타임 `F` 명령으로
> 내려가며 찾을 것(1000 → 700 → 400). 50Hz로 되돌리면 소음도 돌아온다.

## 시리얼 프로토콜 (115200 baud, 라인단위 `\n`)
```
V <vx> <vy> <w>   속도지령. vx=전진+, vy=우평행+, w=시계회전+. 각 -255..255  (무응답)
S                 즉시 정지 (슬루 무시)
T <i> <d>         모터 i(0=A/FR 1=B/FL 2=C/RR 3=D/RL) 방향 d(1/-1) 단독 1초 (방향 캘리브레이션)
P <n>             최대 PWM 스케일 설정(기본 2000 / 4095)
L <a> <b> <c> <d> 런타임 극성 변경 (POL, 각 1/-1). 방향 캘리브레이션용
R <accel> <decel> 슬루 스텝(틱당 변화량 상한). 기본 6 / 12
F <hz>            PWM 주파수(24~1526). 소음/드라이버 튜닝용
?                 상태 1줄 출력
```
**체크섬**: 모든 라인 끝에 `*HH`를 붙일 수 있다(HH = 앞부분 전체의 XOR, 2자리 16진수).
예) `V 140 0 0*43`. 한 번이라도 올바른 체크섬을 받으면 그 뒤로 **무체크섬 `V`는 거부**된다
(`nak nocrc`) — USB 노이즈로 깨진 속도지령이 실행되는 사고를 막는다. 젯슨 쪽 구현은
[`motor_link.py`](../../src/tomato_picker/hardware/motor_link.py)와
[`controller_drive.py`](../../tools/controller_drive.py) 둘 다 체크섬을 붙인다.

응답: `ok ...`(설정계열), `nak crc`/`nak nocrc`(거부), 1초마다
`hb <ms> rx=<수신수> bad=<거부수> v=<vx,vy,w>` 하트비트 — 이 카운터가 링크 품질 지표다.

> ⚠ **모터보드 포트는 한 프로세스만** 쓸 수 있다. 젯슨 쪽이 이제 상시 점유
> (`exclusive=True`)하므로 `tomato-voice`(대시보드)와 `controller-drive`(게임패드)를
> 동시에 켜면 나중 쪽이 "Device or resource busy"로 실패한다. 둘 중 하나만 켤 것.

메카넘 mixing: `FL=vx-vy-w, FR=vx+vy+w, RL=vx+vy-w, RR=vx-vy+w` → 바퀴별 `POL` 적용.
(strafe/회전 부호는 실기에서 `L`·`T`로 최종 확정할 것. 전진만 확정됨.)

## 안정성 방어 (이 HW의 실패들에서 도출)
1. **데드맨**: 300ms 내 새 `V`/`S` 없으면 목표 0으로 감속, 1000ms 침묵이면 즉시 0 —
   상위(젯슨) 끊겨도 폭주 없음. v1은 400ms에서 곧바로 하드 컷이라 소리로 들렸다.
2. **하드웨어 워치독** `WDTO_250MS` + 매 loop `wdt_reset()` — I2C/코드 행(hang) 자동복구.
   (이 HW의 안정 하한. 120ms는 PS2X 없이도 가끔 초과, 250ms가 오리셋 0.)
3. **논블로킹 loop** — 긴 delay 없음.
4. **PS2 완전 분리** — PS2X 비트뱅잉이 이 보드에서 `millis()`를 얼리는(timer0 간섭) 문제가
   있어 주행 펌웨어에서 제거. 안전은 HW 워치독(레지스터 기반, millis 무관)으로.

## 빌드 / 업로드
```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/mecanum_stable
arduino-cli upload -p COM14 --fqbn arduino:avr:uno firmware/mecanum_stable   # 보드를 PC에 꽂았을 때
```

**보드가 젯슨에 붙어 있을 때**(평소 상태) — 젯슨에는 arduino-cli도 FaBo 라이브러리도 없다.
PC에서 .hex만 만들어 avrdude로 굽는다(2026-08-06 실제 사용한 경로):

```bash
# PC(Windows, git bash)
arduino-cli compile --fqbn arduino:avr:uno --output-dir /tmp/tpfw firmware/mecanum_stable
scp /tmp/tpfw/mecanum_stable.ino.hex server@192.168.0.8:/tmp/

# 젯슨
sudo apt-get install -y avrdude          # 최초 1회
sudo systemctl stop tomato-voice         # ★ 대시보드가 ttyUSB0를 독점하므로 반드시 먼저
avrdude -c arduino -p atmega328p -P /dev/ttyUSB0 -b 115200 -D -U flash:w:/tmp/mecanum_stable.ino.hex:i
sudo systemctl restart tomato-voice
```

**업로드 확인법** — v1과 v2는 겉으로 잘 안 구분된다(둘 다 굴러간다). 이렇게 본다:

```bash
stty -F /dev/ttyUSB0 115200 raw -echo && timeout 4 cat /dev/ttyUSB0
```
- v2: `mecanum_stable v2 ready...` 배너 + `hb 1001 rx=0 bad=0 v=0,0,0`
- v1: 배너에 v1, 하트비트는 `hb 1001`뿐이고 지령마다 `ok V 0 0 0`을 되쏜다

또는 대시보드 `/control` 맨 위 링크 카드에서 **`수신` 숫자가 늘어나면 v2**다
(v1은 `rx=`를 안 보내 0에 멈춰 있다).
필요 라이브러리: `FaBo_PWM_PCA9685`(중복파일 제거), `PS2X_lib`는 이 펌웨어엔 불필요.

## 상위 통합
젯슨의 [`tools/controller_drive.py`](../../tools/controller_drive.py)가 게임패드(evdev)를 읽어
이 프로토콜(`V`/`S`)로 주행 + 팔 프리셋 재생. 젯슨 셋업(hid-nintendo 빌드, systemd 자동실행,
알려진 이슈)은 [`docs/jetson-gamepad-setup.md`](../../docs/jetson-gamepad-setup.md) 참고.
