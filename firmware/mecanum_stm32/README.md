# mecanum_stm32 — STM32F405 주행 보드 펌웨어 (mecanum_stable 대체)

[`mecanum_stable`](../mecanum_stable/) v2.1(Uno + PCA9685 + HR8833)을 **같은 프로토콜로** 대체한다.
젯슨 [`motor_link.py`](../../src/tomato_picker/hardware/motor_link.py)·`base_nudge`·ROS `cmd_vel_node`는
안 고친다 — 배너 앞머리(`mecanum_stable`), `boot #`, `hb` 앞부분 형식이 그대로다.
배경·합격 기준 = [stm32-자세보드-목표.md](../../docs/stm32-자세보드-목표.md)(G2·G4) · 명세서 v0.1(2026-11-02).

## 하드웨어 (기판과 1:1 — 핀을 바꾸면 기판도 바뀐다)
- **STM32F405RGT6**, HSE 8MHz → 168MHz. 크리스털이 안 뜨면 HSI로 돈다(`?`의 `clk=HSI`로 드러남).
- 링크: USART1 PA9/PA10 ↔ 절연기 ↔ CH340C. 젯슨에선 **여전히 `/dev/ttyUSB0`(1a86:7523)**.
- 모터 DRV8874 ×4, PWM 모드(IN1/IN2):

| 바퀴 | 타이머 IN1/IN2 | 엔코더 타이머 A/B | nFAULT | IPROPI |
|---|---|---|---|---|
| 0 A/RR | TIM8 CH1 PC6 / CH2 PC7 | TIM2 PA15 / PB3 (32비트) | PC10 | PC0 |
| 1 B/RL | TIM8 CH3 PC8 / CH4 PC9 | TIM3 PB4 / PB5 | PC11 | PC1 |
| 2 C/FR | TIM9 CH1 PA2 / CH2 PA3 | TIM4 PB6 / PB7 | PC12 | PC2 |
| 3 D/FL | TIM12 CH1 PB14 / CH2 PB15 | TIM5 PA0 / PA1 (32비트) | PD2 | PC3 |

- nSLEEP 공통 PB12(기판 풀다운 — 펌웨어가 PWM을 0에 둔 뒤 깨운다) ·
  **배터리 감지 PC4**(역접속 FET 뒤·E-stop 릴레이 앞, **100k/10k**) · 5V 감지 PC5(10k/10k) ·
  **E-stop 감지 PB13**(10k 직렬 + 100k 풀다운. HIGH=릴레이 붙음/모터 허용, LOW=눌림·선 빠짐) ·
  LED PB0(하트비트)·PB1(폴트) · 남은 헤더: I2C1 PB8/PB9, SPI1 PA4~PA7, IMU INT PB10.
- 보정 손잡이: 배터리 분압비 = `B` 명령(런타임), `V5_DIV_X100`·`IPROPI_MA_PER_V` = 스케치 위쪽 상수.

## Uno(v2.1)와 다른 점
| 항목 | Uno v2.1 | STM32 v3.1 |
|---|---|---|
| PWM | PCA9685 I2C, 24~1526Hz | 타이머 직결 24~40000Hz, **기본 20kHz**(가청 밖) |
| 속도 제어 | 개루프 | 바퀴별 **엔코더 PID**(`G`로 켬, 안 켜면 개루프 그대로) |
| 정지 | S·데드맨(소프트) | + **하드웨어 E-stop**(릴레이가 VM 차단, PB13으로 감지) |
| 배터리 | 모름 | 1초 평균 `vin=`, 저전압 2초 → 감속 정지 |
| I2C 행 | 워치독 1순위 용의자 | 원리적으로 없음. `i2c=`/`i2cerr=`는 항상 0 |
| 포트를 열면 | DTR로 **리셋** | **리셋 안 됨**(DTR/RTS 배선 없음). 멈춘 보드는 IWDG가 살린다 |
| 리셋 원인 | 워치독 ISR 자백 + SRAM 흔적 | **RCC_CSR**(`WDT`/`POWER`/`BOD`/`SW`/`EXT`) + BKPSRAM 흔적 |
| BOD | 퓨즈 | 첫 부팅에 옵션바이트 **BOR Level 3**(≈2.7V)로 한 번 굽는다 |
| 엔코더 | 못 읽음 | 4채널 하드웨어 직교 카운터 |

⚠ **젯슨 `base_tuning`의 F를 700 → 20000으로** — 젯슨이 첫 `hb` 뒤 F/P/R(+이제 G/B)을 되밀어 넣기 때문에
안 바꾸면 펌웨어 기본 20kHz가 700Hz로 덮인다(젯슨 코드는 그대로, 값만). 듀티 2000·슬루 6/12 기본은 Uno와 같다.
`P`는 0..4095 척도 그대로.

## 명령 (v2.1 것 + 아래. `C`/`X`/`M`/`E`는 아직 안 함)
```
B r lo           배터리 분압비 ×1000(기본 11000 = 100k/10k)·저전압 문턱 mV(기본 6400, 0=감시 끔) → "ok B r lo"
G i kv ks kp ki  바퀴 i(0..3, -1=전체) 속도 PID, 전부 ×1000. kv=0이면 그 바퀴 개루프 → "ok G ..."
N i cpr dia_um   엔코더 CPR(4체배 후)·바퀴 지름 — 저장해서 ?에 rpm으로만 보여 준다 → "ok N ..."
Q   cap proto=1 fw=3.1.0 board=stm32f405 units=0 closed_loop=1 enc=4 estop_hw=1 ...   (부팅 때도 1번)
U   정지 → 시스템 부트로더 (아래 굽기)
T i d   끝에 "T auto-stop enc=<증분>" — 바퀴가 앞으로 돌았는데 음수면 그 바퀴 ENC_DIR을 -1로
?   st 줄 + 바퀴별 "wh i G=kv,ks,kp,ki loop=open|closed|FAULT spd= rpm= N=" + boot 줄
```
**설정은 비휘발성에 저장하지 않는다**(`E` 없음) — F/P/R처럼 젯슨이 연결 때마다 다시 넣는다.

**hb**: `hb <ms> rx= bad= i2c=0 wdt= v=<vx,vy,w>`(여기까진 Uno와 같음) 뒤에
`enc=a,b,c,d vin=<mV> v5= amp=<합 mA> ampw=a,b,c,d flt= fltn= spd=<cps a,b,c,d> estop=0|1 lowbat=0|1 clf=<비트>`.

**이벤트 줄**(한 번씩만): `estop 1`/`estop 0`, `lowbat 1`/`lowbat 0`, `clfault i`.

## E-stop · 저전압 동작
- **E-stop**: PB13이 20ms 연속 LOW → 즉시 정지(슬루 무시)·드라이버 재움·`estop 1`. 그동안 `V`는 **조용히 무시**
  (50Hz로 오니 nak를 쏟지 않는다 — `hb`의 `estop=1`로 보인다), `T`는 `nak estop`.
  **풀려도 다시 굴러가지 않는다** — 목표는 0인 채, 다음 `V`가 와야 간다. 켜질 때 이미 눌려 있으면 재운 채로 시작.
  모터 전원 자체는 릴레이가 물리적으로 끊는다 — 펌웨어는 보조(소프트 정지 S·데드맨도 그대로).
- **저전압**: 100ms 표본 10개 평균(1초)이 `lo` 아래로 **2초 연속** → `lowbat 1`, 목표 0(**평소 DECEL 슬루로 감속**),
  이후 `V` 조용히 무시. `lo+300mV` 위로 올라오면 `lowbat 0`. 분압 배선이 없거나 틀리면 `vin`이 0 근처라
  3초 뒤 영영 안 움직인다 — 그땐 `B 11000 0`(감시 끔)으로 확인할 것.

## 속도 PID
```
duty = sign(t)·ks + kv·t + kp·(t − 측정) + ki·∫(t − 측정)     [duty: 0..4095(MAX_PWM 척도), 속도: 카운트/초]
t = (바퀴 지령 / 255) · cps_full,   cps_full = (MAX_PWM − ks) / kv
```
- `V`의 뜻은 그대로("−255..255 = 최고속 대비 비율"). `V 255`는 이제 "피드포워드만으로 꽉 차는 속도"로 **조절**된다.
- 100Hz, 속도 = 엔코더 차분의 4샘플(40ms) 평균. 목표 0 → 적분 비우고 듀티 0(코스트) — 정지에서 안 떤다.
- 적분은 적분항 혼자 ±MAX_PWM을 못 넘게 자른다(포화 방지). 출력은 ±MAX_PWM으로 자른다.
- **폭주 방지**: 듀티가 80%를 넘는 채 0.5초 동안 반대로 돌거나 목표의 10%도 못 내면(엔코더 부호 틀림·선 빠짐·막힘)
  그 바퀴만 개루프로 떨어지고 `clfault i`, `hb`의 `clf=` 비트. 같은 바퀴에 `G`를 다시 보내면 풀린다.

**튜닝 순서** (바퀴를 띄우고):
1. `T i 1` ×4 → 바퀴가 앞으로 돌 때 `enc=`가 **양수**인가. 음수면 그 바퀴 `ENC_DIR`을 −1로(스케치 상수).
2. 개루프로 `V 255 0 0` → `hb`의 `spd=`로 **꽉 찬 듀티에서의 cps**(cps_full)를 읽는다.
3. 정지마찰 듀티 `ks` = 바퀴가 막 움직이기 시작하는 듀티(0..4095 척도. Uno의 Ks≈90/255 ≈ MAX_PWM·0.35).
4. `kv ≈ (MAX_PWM − ks) / cps_full`. 예: MAX_PWM 4095, ks 1400, cps_full 3000 → kv ≈ 0.9 → `G -1 900 1400000 0 0`.
5. `kp`를 작게(예: 0.2 → `200`)부터 올려 지령 변화에 따라가게, 남는 오차가 있으면 `ki`를 조금.
   흔들리면 내린다. `?`의 `wh` 줄과 `hb`의 `spd=`로 본다.

## 명세서 v0.1 대응
| 명세서 항목 | 펌웨어 |
|---|---|
| PWM 10~20kHz | 기본 20kHz (`F`로 24~40000) |
| 엔코더 4채널 쿼드러처 → 속도 PID | TIM2·3·4·5 엔코더 모드 + `G` PID (기본 꺼짐) |
| UART 115200, V/S/T/P/L/? 호환 | 그대로 (+B/G/N/Q/U/W) |
| 하드웨어 E-stop + GPIO 감지 → 보고 | PB13, `estop 1/0` 이벤트·`hb estop=` |
| 배터리 ADC, 저전압 시 감속→정지 | PC4 100k/10k, `B`, `lowbat` |
| BOR 2.7V 이상 | 옵션바이트 BOR Level 3 |
| 젯슨 코드 수정 없음 | 젯슨은 그대로 — `base_tuning` 값(F=20000, 이후 G)만 |

## 빌드
```bash
arduino-cli config add board_manager.additional_urls https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
arduino-cli core update-index && arduino-cli core install STMicroelectronics:stm32   # 3.0.0에서 확인
arduino-cli compile --fqbn STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX --output-dir /tmp/stmfw firmware/mecanum_stm32
```
⚠ `build_opt.h`가 **`Serial`을 USART1(PA9/PA10)로 옮긴다** — 이 변형의 기본 `Serial`은 UART4(PA0/PA1)라
엔코더 D와 겹친다. 이 파일을 지우면 컴파일은 되는데 젯슨과 말을 못 한다.

## 굽기
**처음 1회**(빈 칩): SWD(ST-Link) 또는 BOOT0 버튼을 누른 채 리셋 → 아래 stm32flash.

**그 뒤로는 젯슨에서 원격으로** — BOOT0 안 만진다:
```bash
scp /tmp/stmfw/mecanum_stm32.ino.bin server@100.85.187.87:/tmp/
# 젯슨
sudo systemctl stop click-server tomato-voice        # ★ ttyUSB0 독점 해제
stty -F /dev/ttyUSB0 115200 raw -echo && printf 'U\n' > /dev/ttyUSB0
stm32flash -b 115200 -w /tmp/mecanum_stm32.ino.bin -v -R /dev/ttyUSB0   # -R = 끝나고 리셋
sudo systemctl start click-server
```
- `-g 0x08000000` 대신 **`-R`** 을 쓴다 — 부트로더가 바꿔 둔 클럭(PLL) 위에서 앱이 뜨면
  `SystemClock_Config`가 PLL 재설정을 거절당할 수 있다. 리셋은 그걸 지운다.
- 확인: 부팅 줄이 `boot #N warm cause=SW ...` 다음에 `mecanum_stable v3.1-stm32 ready.`

## 확인 순서 (실기)
1. `?` → `clk=HSE`, `estop=0`인가. 2. `hb`가 1초마다, `vin=`이 배터리 전압과 맞나(안 맞으면 `B`로 비율 보정).
3. E-stop 버튼 → `estop 1`·바퀴 즉시 정지, 놓고 → `estop 0`·안 움직임, 다음 `V`에 움직임.
4. `T i 1` ×4 → 바퀴가 앞으로 + `enc=` 양수(CPR×회전수 근처). 5. 위 튜닝 순서로 `G`.
6. `W` → `cause=WDT last=test`로 돌아오나.
