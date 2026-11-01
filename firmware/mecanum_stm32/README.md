# mecanum_stm32 — STM32F405 주행 보드 펌웨어 (mecanum_stable 대체)

[`mecanum_stable`](../mecanum_stable/) v2.1(Uno + PCA9685 + HR8833)을 **같은 프로토콜로** 대체한다.
젯슨 [`motor_link.py`](../../src/tomato_picker/hardware/motor_link.py)·`base_nudge`·ROS `cmd_vel_node`는
안 고친다 — 배너 앞머리(`mecanum_stable`), `boot #`, `hb` 형식이 그대로다.
배경·합격 기준 = [stm32-자세보드-목표.md](../../docs/stm32-자세보드-목표.md)(G2·G4).

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

- nSLEEP 공통 PB12(기판 풀다운 — 펌웨어가 PWM을 0에 둔 뒤 깨운다) · VM 감지 PC4(47k/10k) ·
  5V 감지 PC5(10k/10k) · LED PB0(하트비트)·PB1(폴트) · 남은 헤더: I2C1 PB8/PB9, SPI1 PA4~PA7, IMU INT PB10.
- 분압비·IPROPI 환산은 스케치 위쪽 `VM_DIV_X100`·`IPROPI_MA_PER_V`가 **보정 손잡이**다(저항 오차를 실측으로 맞출 것).

## Uno(v2.1)와 다른 점
| 항목 | Uno v2.1 | STM32 v3.0 |
|---|---|---|
| PWM | PCA9685 I2C, 24~1526Hz | 타이머 직결, **24~40000Hz** (20kHz면 초음파 — 소음↔속도 맞바꿈 해소) |
| I2C 행 | 워치독 1순위 용의자 | 원리적으로 없음. `i2c=`/`i2cerr=`는 항상 0 |
| 포트를 열면 | DTR로 **리셋** | **리셋 안 됨**(DTR/RTS 배선 없음). 멈춘 보드는 IWDG가 살린다 |
| 리셋 원인 | 워치독 ISR 자백 + SRAM 흔적 | **RCC_CSR**(`WDT`/`POWER`/`BOD`/`SW`/`EXT`) + BKPSRAM 흔적 |
| BOD | 퓨즈 | 첫 부팅에 옵션바이트 **BOR Level 3**(≈2.7V)로 한 번 굽는다 |
| 엔코더 | 못 읽음 | 4채널 하드웨어 직교 카운터, `hb`의 `enc=` |
| 전압·전류 | 없음 | `vin=`(mV) `v5=` `amp=`(합, mA) `ampw=a,b,c,d` `flt=`(nFAULT 비트) `fltn=`(폴트 횟수) |

기본값(1500Hz·듀티 2000·슬루 6/12)은 Uno와 **일부러 같다** — 젯슨이 첫 `hb` 뒤 F/P/R을 되밀어 넣는다.
`P`는 0..4095 척도 그대로라 대시보드 값이 그대로 통한다.

## 새 명령 (계약 예약 문자 C/X/G/N/M/B/E는 안 씀)
```
Q   cap proto=1 fw=3.0.0 board=stm32f405 units=0 closed_loop=0 enc=4 calib=0 ...   (부팅 때도 1번)
U   정지 → 시스템 부트로더 (아래 굽기)
T i d   끝에 "T auto-stop enc=<증분>" — 바퀴가 앞으로 돌았는데 음수면 그 바퀴 ENC_DIR을 -1로
```

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
- 확인: 부팅 줄이 `boot #N warm cause=SW ...` 다음에 `mecanum_stable v3.0-stm32 ready.`

## 확인 순서 (실기)
1. `?` → `clk=HSE`인가. 2. `hb`가 1초마다, `vin=`이 배터리 전압과 맞나(분압비 보정).
3. `T i 1` ×4 → 바퀴가 앞으로 + `enc=` 양수(CPR×회전수 근처). 4. `W` → `cause=WDT last=test`로 돌아오나.
