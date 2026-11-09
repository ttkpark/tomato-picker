# mecanum_stm32 — STM32F405 주행 보드 펌웨어 v4 (네이티브)

Uno `mecanum_stable` v2.3을 **같은 프로토콜로** 대체한다. 젯슨 `motor_link.py`·`base_nudge`·`space_map`(거리 이동 `M`)은
안 고친다. 동시에 [보드 계약](../../docs/보드-계약.md)(`cap proto=2`, `C` 물리 단위, `E` 저장, `st=`/`tgt=`/`act=`)을 채워
ROS `Stm32Base`도 그대로 붙는다.

v3.2(아두이노 코어 `.ino`, 2026-11-02)를 2026-11-09에 **CMSIS 레지스터 직결 C로 다시 썼다** — HAL·아두이노 없음, 힙 없음.
옛 스케치는 git 이력(`c273a85`)에 있다.

## 구조
| 파일 | 하는 일 | 시험 |
|---|---|---|
| [`src/core.c`](src/core.c) | 프로토콜·슬루·믹싱·PID·데드맨·거리 이동·E-stop 래치·용착·저전압·막힘 지킴이·하트비트 | PC에서 가짜 하드웨어로 |
| [`src/cfgstore.c`](src/cfgstore.c) | 설정 기록(플래시 섹터 2개, 전원 끊김에 안전) | PC에서 끊김을 워드마다 흉내 |
| [`src/hw.c`](src/hw.c) | 클럭·타이머·ADC DMA·UART DMA·IWDG·플래시·BOR·MPU·폴트·부트로더 | 크로스 빌드 `-Werror` |
| [`src/board.h`](src/board.h) | 핀표(기판 `gen_pcb.py`와 1:1, 11/09 기판 검토에서 전부 일치 확인) | |
| [`test/`](test/) | `test_core.c` 시나리오 17개 + `check_jetson_parsers.py`(젯슨 **실제 파서**에 펌웨어 출력을 넣는다) | |

실행 문맥은 둘이다. **TIM6 1kHz 인터럽트**가 엔코더·속도 추정·PID·슬루(5ms)·데드맨·거리 이동을 돌리고 출력은 여기서만 쓴다.
**main 루프**는 수신·명령·하트비트·E-stop 디바운스·전압 감시를 한다. ISR은 직렬을 만지지 않고 이벤트 큐에 넣는다(main이 내보냄).
계약 §10.1의 "제어 주기는 타이머 인터럽트로 고정"이 이것이다.

| 인터럽트 | 우선순위 | |
|---|---|---|
| EXTI13 (E-stop) | 0 | 무엇이 돌든 드라이버부터 재운다. main의 크리티컬 구역(BASEPRI)도 이건 못 막는다 |
| TIM6 (제어) | 1 | |
| SysTick | 2 | |
| DMA2 S7 (송신) | 4 | |

## 하드웨어 (기판과 1:1)
- **STM32F405RGT6**, HSE 8MHz → 168MHz. 크리스털이 안 뜨면 HSI로 같은 168MHz. 돌다가 크리스털이 멈추면 **CSS → NMI가 HSI로 되살린다**(보율·PWM이 그대로, `? clk=HSI`).
- 링크 USART1 PA9/PA10 ↔ ADuM1201 ↔ CH340C → 젯슨 `/dev/ttyUSB0`(1a86:7523). 수신 DMA 순환 512B, 송신 DMA 고리 2KB — **송신은 막지 않는다**(넘치면 줄째 버리고 `hb txd=`로 센다).

| 바퀴 | PWM IN1/IN2 | 엔코더 | nFAULT | IPROPI |
|---|---|---|---|---|
| 0 A/RR | TIM8 CH1 PC6 / CH2 PC7 | TIM2 PA15/PB3 (32비트) | PC10 | PC0 |
| 1 B/RL | TIM8 CH3 PC8 / CH4 PC9 | TIM3 PB4/PB5 | PC11 | PC1 |
| 2 C/FR | TIM9 CH1 PA2 / CH2 PA3 | TIM4 PB6/PB7 | PC12 | PC2 |
| 3 D/FL | TIM12 CH1 PB14 / CH2 PB15 | TIM5 PA0/PA1 (32비트) | PD2 | PC3 |

nSLEEP PB12(공통) · 배터리 PC4 · VM(릴레이 뒤) PC5 · 5V PB0 · 3V3 = VREFINT+공장 보정값 · E-stop 감지 PB13 · LED PB11(하트비트)/PB1(폴트).

## 명령 (라인, `\n`, 선택 `*HH` XOR 체크섬 — 한 번 받으면 무체크섬 `V`/`C`/`M`은 `nak nocrc`)
| 명령 | 뜻 | 응답 |
|---|---|---|
| `V vx vy w` | duty −255..255(레거시, Uno와 같은 믹싱) | 없음 |
| `C vx vy w` | mm/s, mm/s, mdeg/s — **REP-103**(+vy 왼쪽, +w 반시계). 캘리브레이션 필요 | 없음 · `nak nocalib` · `nak range C` |
| `S` | 즉시 정지(슬루·PID 건너뜀). E-stop 래치는 **안** 푼다 | `ok S` |
| `X 0` / `X 1` / `X` | 래치 해제(버튼이 풀려 있을 때만) / 소프트 래치 / 상태 | `ok X 0` · `nak estop held` · `nak weld` |
| `T i d` | 바퀴 i 단독 1초(듀티 200/255) | `ok T i d` → `T auto-stop enc=<증분>` |
| `M id ticks vx vy w` | 거리 이동(Uno v2.3) — **네 바퀴 엔코더**로 센다. 이동 중엔 `H`로 데드맨 | `ok M id ticks` → `mv done|stall|abort ...` |
| `M lx ly` | 기하(μm, 계약 §5.2) — 인자 개수로 구분 | `ok M lx ly` |
| `H` | 살아 있음 | 없음 |
| `K slow min stall sag bg [guard]` | 이동 감속 구간·최저 지령·걸림 ms·5V 처짐 문턱 mV·(무시)·막힘 지킴이 ms. 0 = 그대로 | `ok K ...` |
| `P n` · `R a d` · `F hz` | 듀티 상한 0..4095 · 슬루 · PWM 24..40000Hz(기본 20k) | `ok P` `ok R` `ok F <실제 값>` |
| `L a b c d [ea eb ec ed]` | 모터 극성 · (선택) **엔코더 부호** — v3.2엔 상수였다 | `ok L POL=… ENC=…` |
| `B ratio lo` | 분압비 ×1000(기본 11000) · 저전압 mV(기본 6400, 0=끔) | `ok B` |
| `G i kv ks kp ki` | 바퀴 i(−1=전체) 게인 ×1000. kv=0이면 개루프. 폭주 판정도 새로 | `ok G` |
| `N i cpr dia_um` | 엔코더 CPR(4체배 후)·바퀴 지름 | `ok N` |
| `E` / `E 0` | 지금 설정을 플래시에 저장 / 지우고 공장값. **바퀴가 서 있을 때만** | `ok E <seq> calib=` · `nak busy E` · `nak flash` |
| `Q` | `cap proto=2 fw=4.0.0 board=stm32f405 id=<UID> … calib= vmax= vymax= wmax= … pwm_hz=` | |
| `?` | `st …` + 바퀴별 `wh …` + `boot …` | |
| `W` · `U` | 워치독 자가진단(일부러 멈춤) · 시스템 부트로더로 | |
| 그 밖 | | `nak unsupported <글자>` — 조용히 무시하지 않는다 |

인자는 끝까지 엄격하게 읽는다(`V 1 2 x` → `nak range V`, 10자리 넘침 → `nak range`). 50Hz 구동 지령이 거절될 땐
`nak estop`/`nak lowbat`/`nak nocalib`/`nak guard`를 **초당 한 번만** 보낸다(송신이 안 막히게).

**보드가 내보내는 줄엔 `*`를 쓰지 않는다** — ROS 파서가 체크섬으로 읽는다(v3.2 배너의 `(+*CRC)`가 그랬다).

## 출력
- 부팅: `boot #N warm|cold cause=WDT|POWER|BOD|SW|EXT|LPWR last=<국면> i2cerr=0 near= mcusr=0x.. r2=0x0 [fault pc= lr= cfsr= hfsr= addr=]`
  → (이동 중 리셋이었으면) `mv lost id got target` → `mecanum_stable v4.0.0-stm32 ready. …` → `cap …`.
  폴트는 처리기가 출력부터 끄고 BKPSRAM에 PC·LR·CFSR을 남긴 뒤 리셋한다 — 다음 부팅 줄에 나온다.
- `hb <ms> rx= bad= i2c=0 wdt= v= enc= vin= vm= v5= v33= vmin= sag= amp= ampw= flt= fltn= spd= estop= lowbat= clf= rly= st=<hex> [tgt=] [act=] [mv=id,got] txd=`
  - `rx`/`bad`는 **32비트**다. v3.2·Uno의 16비트는 50Hz에서 22분마다 되감겨 젯슨이 재부팅으로 오해했다.
  - `st` 비트(계약 §7): 0 E-stop 래치 · 1 소프트 데드맨 · 2 하드 데드맨 · 3 캘리브레이션 유효 · 4 드라이버 폴트 · 5 PID 포화 · 6 저전압.
  - `act`(실측 차체 속도, REP-103)는 캘리브레이션이 있을 때만, `tgt`는 `C` 모드일 때만.
- 이벤트(한 번씩): `estop 1|0` · `weld 1` · `rlyopen 1|0` · `lowbat 1|0` · `clfault i` · `guard i` · `deadman HARD` · `mv …` · `T auto-stop enc=`.

## 안전 동작
- **데드맨**(계약 §9): 300ms 무지령 → 목표 0(슬루 감속), 1000ms → 즉시 0 + `deadman HARD`(한 번). 이동(`M`) 중 `H`가 끊기면 `mv abort … deadman`.
- **E-stop**: PB13 하강 → 인터럽트가 **그 자리에서** nSLEEP LOW·PWM 0, 하드웨어 층이 그 뒤로 출력·깨우기를 **거부**한다(판단 층이 실수로 다시 켜도 못 켠다).
  20ms 디바운스 뒤 `estop 1`·래치. 버튼을 놓아도 래치(`hb estop=2`), `X 0`으로만 풀린다(`S`로는 안 풀림 — 젯슨이 S를 자동으로 보낸다).
  잡음으로 인터럽트만 섰으면 20ms 뒤 지우고 0에서 다시.
- **릴레이 감시**: 눌림인데 VM이 **배터리의 85% 넘게** 1초 → `weld 1`(전원을 껐다 켜야 풀림). 문턱을 v3.2의 50%·300ms에서 올렸다 —
  E-stop 뒤 차체를 손으로 밀면 역기전력이 VM을 올려 오검출했다(11/09 기판 검토). 코일은 붙었는데 VM이 없음 → `rlyopen 1`.
- **저전압**: 1초 평균이 문턱 아래 2초 → `lowbat 1`·감속 정지·구동 거절, 문턱+0.3V로 풀림.
- **5V 처짐**: 지난 100ms 최저가 `sag` 문턱(기본 4.5V) 아래면 100ms 동안 가속 금지(줄이는 건 허용). `hb vmin= sag=`.
- **막힘 지킴이**(새로): 어느 바퀴든 듀티 50% 넘게 미는데 1.5초째 안 돈다 → 정지 + `guard i`. 그 뒤 **0 지령이나 `S`가 올 때까지** 구동 지령 거절
  (젯슨이 V를 50Hz로 계속 보내므로 안 막으면 1.5초마다 막힌 모터를 다시 민다). 정지 전류는 Uno 시절 리셋의 주범이었다. `K`의 6번째 인자, 0=끔.
- **워치독**: IWDG 250ms. main은 **제어 인터럽트가 살아 있을 때만** 워치독을 준다 — 둘 중 하나만 죽어도 리셋.
  명령마다 예산을 새로 주되(계약 §10.3) 이것도 같은 검사를 거친다 — 안 그러면 50Hz 지령이 죽은 제어 인터럽트를 가려
  출력이 마지막 듀티에 얼어붙는다. 플래시 지우기(최대 2초) 동안만 32초로 늘린다.
- **경쟁 조건**: 드라이버 깨우기·PWM 쓰기는 쓴 **뒤에** E-stop 표시를 한 번 더 본다 — 검사와 쓰기 사이에 E-stop 인터럽트가 끼면 그쪽이 이긴다.
- **스택 보호**: 스택은 CCM 64KB 통째, 바닥 32B를 MPU로 접근 금지 — 넘치면 데이터를 덮지 않고 폴트로 드러난다.
- **BOR**: 첫 부팅에 옵션바이트 Level 3(≈2.7V)로 한 번 굽는다(`? bor=3`).

## 속도 제어 (계약 §10.1)
```
duty = sign(t)·ks + kv·t + kp·(t − 측정) + ki·∫(t − 측정)      [duty 0..4095 척도, 속도 카운트/초]
```
- 1kHz. 적분은 **포화 쪽으로는 안 쌓는다**(clamping) — 와인드업이 "1번 병"의 PID 판이다.
- 속도 = **창이 늘어나는 M법**: 최근 20ms에 8카운트가 안 쌓이면 창을 200ms까지 늘린다(저속 양자화 대책, 계약의 M/T 혼합).
- `V`: 폐루프 바퀴는 목표 = (지령/255)·cps_full, cps_full = (MAX_PWM − ks)/kv. `C`: 역기구학으로 바퀴 mm/s → cps.
- 폭주(0.8·상한 넘게 밀며 0.5초째 반대로 돌거나 목표의 10%도 못 냄) → 그 바퀴 `clfault i`, 피드포워드만으로 계속. `G`를 다시 주면 풀린다.
- `C`의 부호: 이 차체에서 `V`의 +vy는 오른쪽, +w는 반시계로 실측됐다(계약 §14.1 AxisSigns vy=−1, w=+1) → `C`는 V 믹싱에 vy만 뒤집어 넣는다.
  ⚠ 실물에서 `C 0 100 0`이 **왼쪽**으로 가는지 첫날 확인할 것.

## 설정 저장
- 섹터 10·11(0x080C0000, 0x080E0000, 128KB씩) — **펌웨어 영역(섹터 0~9) 밖**이라 `stm32flash`로 다시 구워도 설정이 남는다.
- 기록을 이어 쓰고, 자리가 없거나 찢긴 쓰기가 있으면 다른 섹터를 지우고 넘어간다. 옛 섹터는 다음에 넘어갈 때까지 둔다 →
  **어느 순간 전원이 나가도 온전한 기록 하나는 남는다**(시험: 기록 중 모든 워드 위치에서 끊어 봤다).
- `CFG_VER`이 다르거나 값이 범위 밖이면 공장값으로 뜬다(`? cfg=factory#-1`).
- 젯슨은 연결 때마다 F/P/R을 다시 넣는다(그대로). 저장은 `E`로 사람이 확정한다(계약 §5.2).

## 빌드 · 시험
```bash
make                                    # arm-none-eabi-gcc 14 — build/mecanum_stm32.{elf,bin,hex}, ≈29KB
make test HOSTCC="python -m ziglang cc" # PC 시험(Windows는 pip install ziglang, 리눅스는 make test)
./build/test_core.exe dump > build/dump.txt && python test/check_jetson_parsers.py build/dump.txt
```
CMSIS·스타트업은 [`vendor/cmsis/`](vendor/cmsis/)(STM32CubeF4 1.28.2, Apache-2.0)에 들어 있다 — 큐브 설치 없이 빌드된다.

## 굽기
**처음 1회**(빈 칩): SWD(ST-Link) 또는 BOOT0을 누른 채 리셋 → 아래 stm32flash.
**그 뒤로는 젯슨에서** — BOOT0 안 만진다:
```bash
scp build/mecanum_stm32.bin server@100.85.187.87:/tmp/
# 젯슨
sudo systemctl stop click-server tomato-voice        # ★ ttyUSB0 독점 해제
stty -F /dev/ttyUSB0 115200 raw -echo && printf 'U\n' > /dev/ttyUSB0
stm32flash -b 115200 -w /tmp/mecanum_stm32.bin -v -R /dev/ttyUSB0
sudo systemctl start click-server
```
부트로더 진입은 리셋 직후 `SystemInit`(아무 장치도 켜기 전)에서 한다 — IWDG가 이미 켜진 채로 뛰면 굽는 도중 물린다.

## 첫날 확인 (보드가 오면)
1. `?` → `clk=HSE bor=3 estop=0 cfg=factory`. `hb`의 `vin`이 멀티미터와 맞나(아니면 `B`), `v5`·`v33`.
2. 젯슨 `base_tuning`의 `hz_stm32`가 20000인가(젯슨이 보드를 알아보고 이 칸을 민다).
3. 바퀴를 띄우고 `T i 1` ×4 → 앞으로 + `enc=` 양수. 음수면 `L … ea eb ec ed`로 그 바퀴 엔코더 부호를 고친다.
4. E-stop: 누름 → 즉시 정지·`estop 1`, 놓음 → `estop 0`·`estop=2`·안 움직임, `S` → 그대로, `X 0` → 풀림. 1초 넘게 `weld 1`이 안 나와야 한다.
5. 게인: 개루프 `V 255`에서 `spd=`(cps_full) → 정지마찰 듀티 `ks` → `kv ≈ (MAX_PWM − ks)/cps_full` → `G -1 kv ks 200 1000`부터.
6. `N -1 1320 78600`, `M <lx> <ly>`(μm), `Q`의 `calib=1 vmax=`, `C 200 0 0` → `act`≈200, `C 0 100 0`이 **왼쪽**인가. `E`로 저장.
7. `W` → `boot … cause=WDT last=test`. 젯슨에서 거리 이동(`space_map` 엔코더 걸음)이 `mv done`으로 끝나는가.
