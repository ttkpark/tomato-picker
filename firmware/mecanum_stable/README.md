# mecanum_stable — 젯슨 시리얼 주행용 안정 펌웨어 (현재 사용)

토마토피커 메카넘 베이스(Moebius MecanumRobot, Arduino Uno + PCA9685)의 **현재 주행 펌웨어**.
젯슨이 시리얼로 속도지령을 보내면 4바퀴 메카넘을 구동한다. **PS2를 완전히 배제**하고
(불안정 요소 제거) 젯슨↔아두이노 시리얼 링크에 집중한 안정판.

> 기존 `firmware/PS2-MOTOR/`(PS2 직접조종)와 `firmware/mecanum_serial/`(직결핀 가정, 미완)을
> 대체한다. 상위 제어는 젯슨의 `motor_link.py`(대시보드·조작대)와 ROS `cmd_vel_node`가 담당.

## 하드웨어
- **Arduino Uno (CH340)** + **PCA9685(I2C)** 모터 드라이버. 라이브러리: `FaBo_PWM_PCA9685`.
  (⚠️ `src/`의 `*(1)*` 중복파일 삭제 필수 — 안 하면 ld 중복심볼 에러)
- 젯슨에서 `/dev/ttyUSB0`(CH340 `1a86:7523`), Windows `COM*`.
- **모터↔바퀴 실측 매핑(2026-10-22, 케이스 장착 후)**: PCA9685 채널 기준 **A=RR(뒤우), B=RL(뒤좌), C=FR(앞우), D=FL(앞좌)**. (옛 2026-07-03: A=FR B=FL C=RR D=RL)
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
T <i> <d>         모터 i(0=A/RR 1=B/RL 2=C/FR 3=D/FL) 방향 d(1/-1) 단독 1초 (방향 캘리브레이션)
P <n>             최대 PWM 스케일 설정(기본 2000 / 4095)
L <a> <b> <c> <d> 런타임 극성 변경 (POL, 각 1/-1). 방향 캘리브레이션용
R <accel> <decel> 슬루 스텝(틱당 변화량 상한). 기본 6 / 12
F <hz>            PWM 주파수(24~1526). 소음/드라이버 튜닝용
?                 상태 1줄 출력
M <id> <t> <vx> <vy> <w>   거리 이동(v2.3): 앞바퀴 엔코더 t카운트만큼 가고 선다
H                 살아 있음(무응답) — M 도중 데드맨용. M 도중 V가 오면 이동 취소
K <slow> <min> <stall_ms> <sag_mv> <bg_mv>   M·전압 감시 튜닝(0=그대로). 기본 400 100 500 4300 1100
```
**체크섬**: 모든 라인 끝에 `*HH`를 붙일 수 있다(HH = 앞부분 전체의 XOR, 2자리 16진수).
예) `V 140 0 0*43`. 한 번이라도 올바른 체크섬을 받으면 그 뒤로 **무체크섬 `V`는 거부**된다
(`nak nocrc`) — USB 노이즈로 깨진 속도지령이 실행되는 사고를 막는다. 젯슨 쪽 구현은
[`motor_link.py`](../../src/tomato_picker/hardware/motor_link.py)가 체크섬을 붙인다.

응답: `ok ...`(설정계열), `nak crc`/`nak nocrc`(거부), 1초마다
`hb <ms> rx=<수신수> bad=<거부수> v=<vx,vy,w> enc=<A,B,C,D>` 하트비트 — 이 카운터가 링크 품질 지표다.

**엔코더(v2.2)** — 쉴드가 Uno에 물려 둔 건 M1·M2(JP1·JP2) 엔코더뿐이다(M3·M4는 NC, 회로도
`../MecanumRobot-Arduino-Uno-master/ARDUINO_HR8833.pdf`). ENC1 B=D2 A=D3 → JP1 = PCA ch6/7 = **D(FL)**,
ENC2 B=D4 A=D6 → JP2 = ch4/5 = **C(FR)**. PCINT2 하나로 4체배. `enc=`는 STM32와 같은 A,B,C,D 순서이고
**A·B는 늘 0**. 부호는 `T 2 1`·`T 3 1`의 `T auto-stop enc=<1초 카운트>`가 양수가 되게 `ENC_DIR`로 맞춘다.

⚠ **2026-11-04 실기: 이 쉴드에선 엔코더가 Uno 핀까지 안 온다 — 카운트는 늘 0.** 바퀴는 돌았다(IMU 4.6°/s·still=False)
그런데 D2·D3·D4·D6이 한 번도 안 바뀌었고, `E` 탐침(핀을 H/L로 충전했다 놓고 읽기)에서 **D2~D7·D8~D13·A0·A2·A3
전부 `10`=떠 있음**(밖에서 당기는 핀은 A1=RGB `00` 하나). 엔코더 출력은 10k 풀업이라 연결돼 있으면 `11`이나 `00`이
나와야 한다. 회로도와 실물이 다르다(혹은 패턴 단선). 쓰려면 **JP1 3·4번 → D2·D3, JP2 3·4번 → D4·D6을 선으로 잇는다**
(쉴드 VCC50이 살아 있어야 엔코더가 돈다). 그 전엔 v2.3 `M`이 걸림(500ms)으로 바로 선다.

## v2.3 — 거리 이동 `M` + 브라운아웃 대비

이 보드의 리셋은 대부분 모터 기동 때 **로직 5V가 처지는 것**(XL1509가 모터와 같은 BAT에서 만든다, `cause=EXT|BOD`)이다.
거리 제어를 젯슨에 두면 그 리셋 한 번에 "얼마나 갔는지"가 사라지고, 젯슨 지연이 그대로 거리 오차가 된다. 그래서 보드가 센다.

| 장치 | 하는 일 |
|---|---|
| `M` 보드 안 루프 | 5ms마다 앞바퀴 진행(도는 바퀴만, 이동 방향 +)을 보고 남은 `slow` 카운트부터 비례 감속, 최저 `min`(정지마찰 ≈90 위). 도착하면 코스팅 정지 → 150ms 뒤 굴러간 것까지 세어 `mv done <id> <got> <target> <ms>` |
| 걸림 차단 | 지령 중인 앞바퀴가 `stall_ms` 동안 앞으로 2카운트도 못 가면 즉시 정지 `mv stall ... wheel=C\|D sag=N`. 막힌 모터 = 최대 전류 = 브라운아웃 원인이라 이게 곧 차단이다. 거꾸로 도는 것(ENC_DIR 틀림)도 여기 걸린다 |
| 로직 전압 감시 | AVcc를 내부 1.1V 밴드갭으로 잰다(핀 불필요). `sag_mv` 아래면 100ms 동안 **가속 금지**(감속은 허용). hb에 `vcc= vmin=(지난 1초 최저) sag=(처짐 횟수)` |
| 리셋 뒤 진행 보존 | 이동 기록을 `.noinit` SRAM에 체크섬과 함께 둔다. warm 리셋(BOD·DTR·WDT)이면 부팅 줄 다음에 `mv lost <id> <got> <target>`. **보드는 절대 혼자 이어 가지 않는다** — 처진 전원으로 다시 출발하면 같은 리셋을 반복한다 |
| 젯슨 쪽 재시도 | `MotorLink.drive_ticks()`: lost면 보드 부팅·튜닝 재적용을 기다린 뒤 **남은 만큼만, 속도 ×0.7**(하한 120)로 최대 2번. stall·abort·시간초과는 다시 안 간다 |
| 데드맨 | 이동 중엔 젯슨이 V 대신 `H`를 50Hz로 보낸다. 300ms 끊기면 `mv abort ... deadman` — 젯슨이 죽으면 남은 거리가 있어도 선다 |

⚠ 단위는 **앞바퀴 카운트**다(mm 아님). 메카넘은 옆걸음·회전에서 바퀴가 미끄러지므로 축마다 mm/카운트가 다르다 — 젯슨이 축별로 잰다.
⚠ 뒷바퀴만 도는 지령(`vy = -w` 꼴)은 셀 수 없어 `nak mv blind`.
⚠ `sag_mv`를 실제 Vcc보다 높게 두면 가속이 늘 막힌다 → `mv stall ... sag=큰수`로 드러난다. 밴드갭은 칩마다 ±10%라
  멀티미터로 5V 핀을 재서 `K 0 0 0 0 <bg>`로 hb `vcc=`를 맞춘다.
⚠ `W`(워치독 자가진단)를 이동 중에 보내면 일부러 리셋해 `mv lost` 경로를 책상에서 시험할 수 있다.

### 젯슨이 켜지면 (굽기 → 확인)
1. PC: `arduino-cli compile --fqbn arduino:avr:uno --output-dir /tmp/tpfw firmware/mecanum_stable` → scp → 아래 "빌드 / 업로드"대로 avrdude.
   주행 포트를 쓰는 것(`click-server`·`tomato-voice`)을 먼저 내린다.
2. 배너 `v2.3` + hb에 `vcc=` 확인. `vcc`가 멀티미터와 다르면 `K 0 0 0 0 <bg>`.
3. `T 3 1`·`T 2 1` → `T auto-stop enc=`가 **양수**여야 한다. 음수면 `ENC_DIR`의 그 칸(2=C, 3=D) 부호를 뒤집어 다시 굽는다.
4. 바퀴 둘레를 재고, `T`의 1초 카운트·바퀴 한 바퀴 카운트를 적는다.
5. `python ros2/tools/base_nudge.py --vx 130 --ticks 800` → `state=done`, 자로 잰 거리 ÷ got = 전진 mm/카운트. 옆걸음(`--vy`)·회전(`--w`)도 따로.
6. 브라운아웃 경로: 이동 중 `W` → `mv lost`가 오고 `drive_ticks`가 남은 만큼 다시 가는지.

> ⚠ **모터보드 포트는 한 프로세스만** 쓸 수 있다. 젯슨 쪽이 이제 상시 점유
> (`exclusive=True`)하므로 `tomato-voice`(대시보드)·`base_nudge`·ROS `cmd_vel_node`를
> 동시에 켜면 나중 쪽이 "Device or resource busy"로 실패한다. 하나만 켤 것.

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
젯슨의 [`motor_link.py`](../../src/tomato_picker/hardware/motor_link.py)(대시보드)·
[`base_nudge.py`](../../ros2/tools/base_nudge.py)(조작대)·ROS `cmd_vel_node`가 이 프로토콜(`V`/`S`)을 쓴다.
(게임패드 드라이버 `controller_drive.py`는 2026-10-22 폐지.)
