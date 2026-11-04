// mecanum_stable — 젯슨 시리얼 주행 펌웨어 v2 (PS2 없음).
// 목표: 무슨 일이 있어도 폭주/행 없이, 젯슨과 안정적으로 통신. 그리고 **조용히** 굴러갈 것.
//
// v2에서 바뀐 것 (v1의 "우우웅" 소음 + 주행 끊김을 잡는다):
//  (1) PWM 50Hz → 1500Hz.  ★ "우우웅"의 진짜 원인. 50Hz는 사람이 소리로 듣는 대역이라
//      모터가 초당 50번 끊기며 도는 게 그대로 저주파 울림이 된다. PCA9685의 상한(1526Hz)
//      근처로 올리면 가청 저역을 벗어나 훨씬 조용하고, 토크 리플도 줄어 저속이 부드러워진다.
//  (2) 슬루(가감속) 제한.  V 지령을 그대로 때리지 않고 5ms 틱마다 목표로 서서히 접근한다.
//      → 기동 인러시 피크가 낮아져 12V 레일 붕괴/젯슨 리셋 위험이 준다([[battery-power-system]]),
//        젯슨 쪽 지령이 한두 번 밀려도 속도가 튀지 않는다.
//  (3) 소프트 데드맨.  명령이 끊기면 즉시 STOP(=속도 0을 때림)이 아니라 목표를 0으로 두고
//      슬루로 감속한다. v1에서 "주웅 주웅 주우웅"으로 들리던 끊김이 이 하드 컷이었다.
//      완전 침묵이 HARD_TIMEOUT을 넘으면 그때는 즉시 0(진짜 안전 정지).
//  (4) 체크섬.  라인 끝에 "*HH"(앞부분 전체의 XOR, 2자리 16진수)를 붙이면 검증한다.
//      한 번이라도 올바른 체크섬을 받으면 그 뒤로는 체크섬 없는 V를 거부한다(strict 모드
//      자동 전환) — USB 노이즈로 깨진 "V 255 0 0"이 그대로 실행되는 사고를 막는다.
//  (5) V 응답 침묵.  50Hz로 들어오는 지령마다 "ok"를 되쏘면 TX 버퍼가 차서 loop가 막힌다.
//      상태는 1초 하트비트에 실어 보낸다(수신/오류 카운터 포함 = 링크 품질 지표).
//
// 프로토콜 (라인 단위, '\n' 종료. 모든 라인에 선택적으로 "*HH" 체크섬을 붙일 수 있다):
//   V <vx> <vy> <w>   속도지령. vx=전진+, vy=우평행+, w=시계회전+. 각 -255..255. (무응답)
//   S                 즉시 정지 (슬루 무시, 하드)
//   T <i> <d>         모터 i(0=A/RR 1=B/RL 2=C/FR 3=D/FL) 방향 d(1/-1) 단독 1초 (캘리브레이션)
//   P <n>             최대 PWM 스케일 설정 (기본 2000 / 4095)
//   L <a><b><c><d>    바퀴 극성 (각 1/-1)
//   R <accel> <decel> 슬루 스텝(틱당 변화량 상한). 기본 6 / 12
//   F <hz>            PWM 주파수 변경 (24~1526). 소음 튜닝용
//   ?                 상태 1줄 출력
//   M <id> <ticks> <vx> <vy> <w>   거리 이동(v2.3): 앞바퀴 엔코더로 ticks만큼 가고 선다. 끝나면 "mv done|stall|abort ..."
//   H                 살아 있음(무응답). M 도중엔 V 대신 이것으로 데드맨을 채운다 — V가 오면 이동은 취소된다
//   K <slow> <min> <stall_ms> <sag_mv> <bg_mv>   M 감속 구간·최저 지령·걸림 판정·전압 처짐 문턱·밴드갭 보정 (0=그대로)
// 응답: 처리 확인 "ok ...", 체크섬 오류 "nak crc", 1초 주기 "hb <ms> rx=<n> bad=<n> v=<vx,vy,w>" enc=<A,B,C,D>
//       vcc=<mV> vmin=<mV> sag=<n> [mv=<id>,<got>]
//
// v2.3 — 거리 이동과 브라운아웃 대비. 이 보드의 리셋은 대부분 모터 기동 때 로직 5V(XL1509, 모터와 같은 BAT)가
//   처져서 난다(EXT|BOD). 그래서:
//  (1) M은 보드가 직접 센다 — 젯슨 지연과 무관하게 5ms마다 진행을 보고, 남은 거리가 slow 안이면 줄이되
//      정지마찰(≈90) 아래로는 안 내린다. 앞바퀴 둘 중 **도는 쪽만** 센다(뒷바퀴는 못 읽는다).
//  (2) 걸림 = 즉시 정지. 지령 중인 앞바퀴가 stall_ms 동안 앞으로 2카운트도 못 가면 선다. 막힌 모터는
//      정지 전류(최대 전류)를 먹으므로 이게 곧 브라운아웃 차단이고, "지령은 나가는데 안 움직인다"를 숫자로 보고한다.
//  (3) 로직 전압 감시 — AVcc를 1.1V 밴드갭으로 잰다(핀 불필요). sag_mv 아래로 처지면 100ms 동안 가속을
//      멈춘다(줄이는 건 허용). vmin=으로 리셋 직전 전압을 하트비트가 남긴다.
//  (4) 리셋돼도 진행이 남는다 — 이동 기록을 .noinit에 체크섬과 함께 둔다. warm 리셋(BOD·DTR·WDT)이면
//      부팅 줄 다음에 "mv lost <id> <got> <target>"을 뱉는다 — 젯슨이 남은 만큼만 다시 시킨다.
//      리셋 뒤엔 절대 스스로 이어 가지 않는다(전원이 처진 채로 다시 출발하면 같은 리셋을 반복한다).
//
// v2.2 — 엔코더. 쉴드가 Uno에 물려 둔 앞바퀴 둘(C=FR, D=FL)을 4체배로 센다(뒷바퀴 A·B는 쉴드 NC라 0).
//   T i 1 의 자동정지 줄이 "T auto-stop enc=<그 1초 동안 카운트>"를 싣는다.
//
// v2.1 — "USB 전원은 멀쩡한데 보드만 재부팅한다"를 규명하기 위한 계측 + 그 원인 차단.
//   2026-08-11 실측: 2분 사이 보드가 28번 재부팅했는데 dmesg에 USB 이벤트가 **하나도**
//   없었다(포트는 열린 채였고 hb_resets=0). 즉 리셋은 통신선이 아니라 AVR 자신에게서
//   나왔다는 뜻인데, 워치독(=loop 블로킹)인지 브라운아웃(=전원)인지 알 방법이 없었다.
//   이 둘은 정반대 처방이 필요하므로 먼저 **구분**할 수 있게 만든다.
//
//  (1) 리셋 원인 보고. MCUSR(WDRF/BORF/EXTRF/PORF)을 main()보다 먼저 도는 .init3에서
//      낚아챈다 — 부트로더가 지워버리기 전에. optiboot가 r2에 남긴 사본도 같이 챙긴다.
//  (2) SRAM 흔적(.noinit). SRAM은 리셋으로 안 지워지고 **전원이 실제로 끊겨야** 사라진다.
//      magic이 살아 있으면 warm(전원 유지 · 칩만 리셋), 깨져 있으면 cold(전원이 나감).
//      이 한 글자가 "USB 전원 들어와 있는데 왜?"에 그대로 답한다.
//  (3) 죽기 직전 국면(last=). I2C 쓰는 중이었는지, F의 delay(100) 중이었는지를 남긴다.
//      워치독이 물었다면 **어디서** 물렸는지가 곧 원인이다.
//  (4) I2C 하드닝. AVR TWI는 버스가 물리면 endTransmission()에서 **타임아웃 없이 무한
//      대기**한다(모터 스위칭 노이즈로 SDA가 눌리는 상황). 워치독이 물 1순위 후보라
//      Wire 타임아웃을 걸어 행 대신 실패로 끝내고, 그 횟수를 하트비트로 보고한다.
//      덤으로 100kHz→400kHz: applyCurrent()가 ~9.5ms→~2.4ms로 줄어 5ms 슬루 틱이
//      비로소 제 주기를 찾는다(100kHz에선 틱보다 I2C가 길어 슬루가 절반 속도였다).
//  (5) 명령마다 wdt_reset(). F 두 개가 한 loop 패스에 들어오면 delay(100)×2 = 200ms라
//      250ms 워치독에 아슬아슬했다. 명령 단위로 워치독 예산을 새로 준다.
#include <avr/wdt.h>
#include <Wire.h>
#include "FaBoPWM_PCA9685.h"

FaBoPWM faboPWM;

// ---------------- 리셋 원인 추적 (v2.1) ----------------
#define BOOT_MAGIC 0xB07Eu

// .noinit = 리셋으로 초기화되지 않는 SRAM 영역. 전원이 실제로 나가야 값이 깨진다.
// ⚠ 전부 volatile이어야 한다. 이 값들을 읽는 쪽은 **다음 부팅** 또는 ISR이라,
//   컴파일러 눈에는 "쓰기만 하고 아무도 안 읽는 변수"로 보인다. 실제로 v2.1 첫
//   빌드에서 `lastPhase = PH_TEST; for(;;){}` 의 저장이 죽은 코드로 제거돼
//   워치독 테스트가 last=test 대신 last=idle을 보고했다. 흔적을 남기는 코드는
//   최적화 대상이 되면 안 된다.
volatile uint16_t bootMagic  __attribute__((section(".noinit")));
volatile uint16_t bootCount  __attribute__((section(".noinit")));
volatile uint8_t  lastPhase  __attribute__((section(".noinit")));  // 지금 하고 있는 일
volatile uint8_t  prevPhase  __attribute__((section(".noinit")));  // 리셋 직전에 하던 일
volatile uint8_t  mcusrSave  __attribute__((section(".noinit")));
volatile uint8_t  r2Save     __attribute__((section(".noinit")));
volatile uint8_t  wdtHit     __attribute__((section(".noinit")));  // 워치독이 물었다는 자백
volatile uint8_t  wdtPhase   __attribute__((section(".noinit")));  // 물릴 때 하던 일

// ⚠ 2026-08-11 실측: 이 보드의 부트로더는 MCUSR을 **지우고 r2에 사본도 안 남긴다**
//   (mcusr=0x0, r2=0xAF ← MCUSR이 쓰지도 않는 상위 비트가 켜진 잔여값).
//   그래서 MCUSR만 믿으면 "WDT BOD EXT POR 전부"라는 무의미한 답이 나온다.
//   → 워치독을 **인터럽트 우선 모드(WDIE|WDE)** 로 돌린다. 첫 타임아웃은 리셋이
//     아니라 ISR을 부르고, 거기서 .noinit에 자백을 남긴 뒤 다음 주기에 진짜
//     리셋된다. 부트로더가 뭘 지우든 상관없이 "워치독이었다"가 확정된다.
#define WDT_MAGIC 0x5Au

volatile uint8_t wdtNear = 0;   // ISR이 떴다가 **살아난** 횟수(=250ms 막혔다 복구)

enum { PH_BOOT = 0, PH_IDLE, PH_RX, PH_I2C, PH_SETHZ, PH_STOP, PH_TEST };

// 워치독을 인터럽트+리셋 모드로 무장. avr-libc의 wdt_enable()은 WDE만 켜므로
// 레지스터를 직접 쓴다(WDCE 타이밍드 시퀀스 필수).
void armWatchdog(){
  cli();
  wdt_reset();
  MCUSR &= ~_BV(WDRF);
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP2);   // WDP2 = 250ms
  sei();
}

// 첫 타임아웃(250ms)에 여기로 온다. WDIE는 자동으로 꺼지므로, 여기서 못 살아나면
// 다음 250ms에 진짜 리셋이 난다 — 즉 완전 정지까지는 최대 500ms.
// (모터 안전은 원래도 CPU가 살아 있어야 도는 데드맨이 맡으므로 실질 차이는 없고,
//  대신 "왜 죽었는지"를 얻는다. 얻는 게 훨씬 크다.)
ISR(WDT_vect){
  wdtHit   = WDT_MAGIC;
  wdtPhase = lastPhase;
  wdtNear++;
}

// ★ main()보다 먼저 도는 .init3. 여기서 MCUSR을 읽어야 하는 이유:
//   Uno 부트로더(optiboot)는 MCUSR을 읽고 **0으로 지운 뒤** 스케치로 넘어간다.
//   setup()에서 읽으면 이미 늦어 항상 0이다. r2는 optiboot 6+ 가 지우기 전에
//   남겨두는 사본이라 부트로더 버전에 따라 이쪽이 답을 갖고 있다.
//   wdt_disable()도 여기서 — 워치독 리셋 후 WDT가 켜진 채로 남아 무한 리셋
//   루프에 빠지는 고전적 함정을 막는다(구형 부트로더에서 실제로 일어난다).
void earlyInit(void) __attribute__((naked, used, section(".init3")));
void earlyInit(void) {
  __asm__ __volatile__("mov %0, r2\n" : "=r"(r2Save));
  mcusrSave = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

uint16_t i2cErr = 0;    // Wire 타임아웃 횟수 = I2C 버스가 물린 횟수

// PCA9685 채널 (모터 A/B/C/D 각 2채널)
#define A1 0
#define A2 1
#define B1 2
#define B2 3
#define C1 4
#define C2 5
#define D1 6
#define D2 7

// 실측 매핑(2026-10-22, 케이스 장착 후 — 차체의 앞이 바뀌었다): A=RR, B=RL, C=FR, D=FL
// (옛 2026-07-02: A=FR B=FL C=RR D=RL). T i 1 로 넷 다 '앞'으로 돌아 POL은 그대로.
// 각 바퀴 "물리적 전진"이 어느 전기방향인지 부호로 보정 (캘리브레이션으로 확정).
// idx: 0=A(RR) 1=B(RL) 2=C(FR) 3=D(FL)
// 2026-07-03 실측: V140,0,0에서 좌(B,D)전진/우(A,C)후진 → 우측 반전으로 전진 정합.
int8_t POL[4] = {-1, 1, -1, 1};   // 모터 A반전 B정 C반전 D정(바퀴 자리와 무관 — 모터 배선의 극성). 런타임 'L'로 변경가능
const uint8_t CH1[4] = {A1, B1, C1, D1};
const uint8_t CH2[4] = {A2, B2, C2, D2};

int   MAX_PWM = 2000;             // PWM 최대 스케일 (PCA9685는 12bit=4095)

// ★ 소음의 핵심. 50Hz는 그대로 "우우웅"으로 들린다. PCA9685 상한이 1526Hz라
//   초음파(>20kHz)까지는 못 올리지만, 1500Hz면 저주파 울림은 사라지고 얇은 고음만 남는다.
//   드라이버가 이 주파수를 못 따라가 발열하면 F 명령으로 내려가며 찾을 것(1000/700/400...).
const uint16_t PWM_HZ_DEFAULT = 1500;

// 데드맨: 소프트(감속 시작) → 하드(즉시 0). v1은 400ms에서 곧바로 하드였다.
const unsigned long CMD_TIMEOUT  = 300;   // ms: 새 명령 없으면 목표를 0으로 (슬루로 감속)
const unsigned long HARD_TIMEOUT = 1000;  // ms: 이만큼 완전 침묵이면 즉시 0

// 슬루(틱당 변화량 상한). TICK_MS=5ms이므로 6 → 약 1200/초 = 0→255에 210ms.
const unsigned long TICK_MS = 5;
int ACCEL = 6;    // 속도를 "올릴" 때 (인러시 억제)
int DECEL = 12;   // 속도를 "내릴" 때 (정지는 빠르게 = 안전)

unsigned long lastCmd = 0;
unsigned long lastTick = 0;
unsigned long lastHb = 0;
unsigned long testUntil = 0;      // T 명령 자동정지 시각

int tgtVx = 0, tgtVy = 0, tgtW = 0;   // 젯슨이 원하는 속도
int curVx = 0, curVy = 0, curW = 0;   // 실제로 인가 중인 속도(슬루 결과)

// 링크 품질 카운터 — 하트비트로 젯슨에 보고한다.
uint16_t rxOk = 0, rxBad = 0;
bool strictCrc = false;   // 올바른 체크섬을 한 번 받으면 true → 이후 무체크섬 V 거부

bool testing = false;     // T 캘리브레이션 진행 중(슬루/데드맨 우회)
uint8_t testWheel = 0;
int32_t testEnc0 = 0;

// ---------------- 엔코더 (v2.2) ----------------
// 쉴드는 M1(JP1)·M2(JP2) 엔코더만 Uno에 물려 있다(M3·M4는 NC — 회로도 ARDUINO_HR8833.pdf):
//   ENC1 B=D2 A=D3 → JP1 = M1 = PCA ch6/7 = 모터 D(FL)
//   ENC2 B=D4 A=D6 → JP2 = M2 = PCA ch4/5 = 모터 C(FR)
// 넷 다 PORTD라 PCINT2 하나로 4체배 쿼드러처를 센다. 모터 A·B(뒷바퀴)는 읽을 수 없어 0으로 보고한다.
// 부호: 바퀴 전진 = + 가 되게 ENC_DIR을 맞춘다(T 3 1 / T 2 1 의 "T auto-stop enc="가 음수면 뒤집기).
const int8_t ENC_DIR[4] = {1, 1, 1, 1};
const int8_t QD[16] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};   // [이전(A,B)<<2 | 지금(A,B)]
volatile int32_t encRaw[2];       // 0=D(FL) 1=C(FR)
volatile uint8_t encPins;
ISR(PCINT2_vect){
  uint8_t p = PIND, q = encPins; encPins = p;
  encRaw[0] += QD[(q>>2&2 | q>>2&1) << 2 | (p>>2&2 | p>>2&1)];   // A=PD3 B=PD2
  encRaw[1] += QD[(q>>5&2 | q>>4&1) << 2 | (p>>5&2 | p>>4&1)];   // A=PD6 B=PD4
}
void encInit(){
  DDRD &= ~(_BV(2)|_BV(3)|_BV(4)|_BV(6));
  PORTD |= _BV(2)|_BV(3)|_BV(4)|_BV(6);   // 내부 풀업 — 모터 쪽 10k가 이미 있지만 선이 빠졌을 때 떠다니지 않게
  encPins = PIND;
  PCMSK2 = _BV(PCINT18)|_BV(PCINT19)|_BV(PCINT20)|_BV(PCINT22);
  PCIFR = _BV(PCIF2); PCICR |= _BV(PCIE2);
}
// 바퀴 i(0=A..3=D)의 누적 카운트, 전진 = +. 읽을 수 없는 A·B는 0.
int32_t encGet(uint8_t i){
  if(i < 2) return 0;
  uint8_t s = SREG; cli(); int32_t v = encRaw[i == 3 ? 0 : 1]; SREG = s;
  return v * POL[i] * ENC_DIR[i];
}

// ---------------- 로직 전압 감시 (v2.3) ----------------
// AVcc를 기준으로 내부 1.1V 밴드갭을 재면 Vcc = bg × 1024 / ADC. 밴드갭은 칩마다 ±10%라 BG_MV가 보정 손잡이다
// (멀티미터로 5V 핀을 재서 hb vcc=와 맞춘다). analogRead를 안 쓰므로 MUX를 밴드갭에 고정해 두고 틱마다 하나씩 읽는다.
int BG_MV = 1100;
int VSAG_MV = 4300;           // ⚠ 너무 높이면 가속이 늘 막혀 "안 움직인다"가 된다 — 그때는 M이 stall로 보고한다
uint16_t vccMv = 0, vccMin = 0xFFFF, sagCount = 0;
unsigned long sagUntil = 0;
uint8_t vccSkip = 4;          // MUX 전환 직후 몇 개는 밴드갭이 덜 서서 낮게 나온다
void vccInit(){ ADMUX = _BV(REFS0) | 0x0E; ADCSRA |= _BV(ADSC); }
void vccTick(unsigned long now){
  if(ADCSRA & _BV(ADSC)) return;          // 변환 중(104µs) — 다음 틱에
  uint16_t a = ADC; ADCSRA |= _BV(ADSC);
  if(vccSkip){ vccSkip--; return; }
  if(!a) return;
  vccMv = (uint32_t)BG_MV * 1024 / a;
  if(vccMv < vccMin) vccMin = vccMv;
  if((int)vccMv < VSAG_MV){
    if((long)(now - sagUntil) >= 0) sagCount++;   // 새 처짐 사건만 센다
    sagUntil = now + 100;
  }
}
bool sagging(unsigned long now){ return (long)(sagUntil - now) > 0; }

// ---------------- 거리 이동 M (v2.3) ----------------
enum { MV_IDLE = 0, MV_RUN, MV_SETTLE };
const uint8_t MV_WHEEL[2] = {3, 2};      // 읽을 수 있는 앞바퀴: D(FL), C(FR)
uint8_t mvState = MV_IDLE;
uint16_t mvId = 0;
int32_t mvTarget = 0;
int mvVx, mvVy, mvW, mvPeak;             // 이동 속도(감속 전), 가장 큰 바퀴 지령
int8_t mvSign[2];                        // 이 이동에서 앞바퀴가 도는 방향(0 = 안 돈다 → 안 센다)
int32_t mvEnc0[2], mvMark[2];
unsigned long mvT0, mvMarkMs[2], mvSettleUntil;
int MV_SLOW = 400;        // 남은 카운트가 이 안이면 비례 감속
int MV_MIN = 100;         // 감속해도 가장 큰 바퀴 지령은 이 아래로 안 내린다(정지마찰 ≈90)
int MV_STALL_MS = 500;    // 슬루 램프(ACCEL 3이면 0→255 425ms)보다 길게

// 리셋을 넘어 살아남는 이동 기록. 체크섬이 맞을 때만 믿는다(SRAM 오염으로 boot #16129가 찍힌 적이 있다).
volatile uint16_t mvRecId     __attribute__((section(".noinit")));
volatile int32_t  mvRecTarget __attribute__((section(".noinit")));
volatile int32_t  mvRecGot    __attribute__((section(".noinit")));
volatile uint16_t mvRecChk    __attribute__((section(".noinit")));
uint16_t mvChk(uint16_t id, int32_t t, int32_t g){
  return id ^ (uint16_t)t ^ (uint16_t)(t >> 16) ^ (uint16_t)g ^ (uint16_t)(g >> 16) ^ 0xA55Au;
}

// 한 바퀴에 부호있는 속도 s(-255..255) 인가. wheel=0..3
void driveWheel(uint8_t w, int s){
  s = (int)s * POL[w];
  int mag = abs(s);
  if(mag > 255) mag = 255;
  int pwm = (long)mag * MAX_PWM / 255;   // 0..MAX_PWM
  if(s > 0){ faboPWM.set_channel_value(CH1[w], pwm); faboPWM.set_channel_value(CH2[w], 0); }
  else if(s < 0){ faboPWM.set_channel_value(CH1[w], 0); faboPWM.set_channel_value(CH2[w], pwm); }
  else { faboPWM.set_channel_value(CH1[w], 0); faboPWM.set_channel_value(CH2[w], 0); }
}

// 메카넘 mixing. 표준 X-롤러 가정 (부호는 캘리브레이션으로 확정):
//  FL = vx - vy - w ; FR = vx + vy + w ; RL = vx + vy - w ; RR = vx - vy + w
// 바퀴→채널: A=RR, B=RL, C=FR, D=FL (2026-10-22)
void applyCurrent(){
  // ★ 흔적을 먼저 남긴다. 여기서 리셋되면 다음 부팅이 last=I2C로 보고하고,
  //   그게 곧 "TWI 버스가 물려 워치독이 물었다"는 증거다.
  lastPhase = PH_I2C;
  int FL = curVx - curVy - curW;
  int FR = curVx + curVy + curW;
  int RL = curVx + curVy - curW;
  int RR = curVx - curVy + curW;
  driveWheel(0, RR);  // A
  driveWheel(1, RL);  // B
  driveWheel(2, FR);  // C
  driveWheel(3, FL);  // D
  // 타임아웃이 걸렸다면 위 8채널 중 일부는 안 써졌다 — 다음 틱이 다시 쓰므로
  // 복구는 저절로 되고, 여기서는 **몇 번 물렸는지**만 센다(하트비트로 보고).
  if(Wire.getWireTimeoutFlag()){ i2cErr++; Wire.clearWireTimeoutFlag(); }
  lastPhase = PH_IDLE;
}

// 즉시 정지 — 슬루를 건너뛴다(비상/안전 경로).
void hardStop(){
  lastPhase = PH_STOP;
  tgtVx = tgtVy = tgtW = 0;
  curVx = curVy = curW = 0;
  testing = false;
  applyCurrent();
}

// cur을 tgt 쪽으로 한 틱만큼 이동. 크기가 커지는 방향이면 ACCEL, 줄어드는 방향이면 DECEL.
// (부호가 바뀌는 구간은 "일단 0으로 줄어드는" 국면이라 DECEL이 걸려 자연히 빠르게 지난다.)
int approach(int cur, int tgt){
  if(cur == tgt) return cur;
  int step = (abs(tgt) > abs(cur)) ? ACCEL : DECEL;
  if(tgt > cur){ cur += step; if(cur > tgt) cur = tgt; }
  else         { cur -= step; if(cur < tgt) cur = tgt; }
  return cur;
}

// 전압이 처진 동안: 줄이는 쪽으로만 움직인다(가속 = 전류를 더 끌어오는 일).
int approachSag(int cur, int tgt){
  int n = approach(cur, tgt);
  return abs(n) > abs(cur) ? cur : n;
}

// 앞바퀴(도는 쪽만) 진행의 평균, 이동 방향 = +.
int32_t mvProgress(){
  int32_t sum = 0; uint8_t n = 0;
  for(uint8_t k = 0; k < 2; k++) if(mvSign[k]){ sum += (encGet(MV_WHEEL[k]) - mvEnc0[k]) * mvSign[k]; n++; }
  return n ? sum / n : 0;
}

void mvReport(const __FlashStringHelper* kind, int32_t got){
  Serial.print(F("mv ")); Serial.print(kind); Serial.print(' '); Serial.print(mvId);
  Serial.print(' '); Serial.print(got); Serial.print(' '); Serial.print(mvTarget);
}

void mvEnd(){ mvState = MV_IDLE; mvRecId = 0; }

// 진행 중인 이동을 끊는다(새 지령·데드맨). 바퀴는 부른 쪽이 세운다.
void mvAbort(const __FlashStringHelper* why){
  if(mvState == MV_IDLE) return;
  mvReport(F("abort"), mvProgress()); Serial.print(' '); Serial.println(why);
  mvEnd();
}

// M <id> <ticks> <vx> <vy> <w>
void mvStart(uint16_t id, int32_t ticks, int vx, int vy, int w){
  int mix[2] = { vx - vy - w, vx + vy + w };            // FL, FR (applyCurrent와 같은 식)
  int m4[4]  = { mix[0], mix[1], vx + vy - w, vx - vy + w };
  if(id == 0 || ticks <= 0){ Serial.println(F("nak mv args")); return; }
  if(!mix[0] && !mix[1]){ Serial.println(F("nak mv blind")); return; }   // 뒷바퀴만 도는 지령 = 셀 수 없다
  mvAbort(F("M"));
  if(testing) hardStop();
  mvPeak = 0;
  for(uint8_t i = 0; i < 4; i++) if(abs(m4[i]) > mvPeak) mvPeak = abs(m4[i]);
  unsigned long now = millis();
  for(uint8_t k = 0; k < 2; k++){
    mvSign[k] = mix[k] > 0 ? 1 : mix[k] < 0 ? -1 : 0;
    mvEnc0[k] = mvMark[k] = encGet(MV_WHEEL[k]);
    mvMarkMs[k] = now;
  }
  mvId = id; mvTarget = ticks; mvVx = vx; mvVy = vy; mvW = w; mvT0 = now;
  mvState = MV_RUN;
  tgtVx = vx; tgtVy = vy; tgtW = w;
  Serial.print(F("ok M ")); Serial.print(id); Serial.print(' '); Serial.println(ticks);
}

// 5ms 틱마다. 목표 속도(tgt*)를 정하고 끝·걸림을 판정한다.
void mvTick(unsigned long now){
  int32_t got = mvProgress();
  if(mvState == MV_SETTLE){
    // 세운 뒤 굴러간 만큼까지 센다 — 젯슨이 다음 보정 이동을 이 숫자로 정한다
    if((long)(now - mvSettleUntil) < 0) return;
    mvReport(F("done"), got); Serial.print(' '); Serial.println(now - mvT0);
    mvEnd();
    return;
  }
  mvRecGot = got; mvRecTarget = mvTarget; mvRecChk = mvChk(mvId, mvTarget, got); mvRecId = mvId;
  int32_t rem = mvTarget - got;
  if(rem <= 0){
    hardStop();                        // 양 채널 0 = 코스팅. 역기전력으로 레일을 때리지 않는다
    mvState = MV_SETTLE; mvSettleUntil = now + 150;
    return;
  }
  for(uint8_t k = 0; k < 2; k++){
    if(!mvSign[k]) continue;
    int32_t e = encGet(MV_WHEEL[k]);
    if((e - mvMark[k]) * mvSign[k] >= 2){ mvMark[k] = e; mvMarkMs[k] = now; }
    else if(now - mvMarkMs[k] > (unsigned long)MV_STALL_MS){
      // 뒤로 도는 것도(ENC_DIR 부호가 틀렸을 때) 여기 걸린다 — 목표에 영영 못 가니 세운다
      hardStop();
      mvReport(F("stall"), got); Serial.print(F(" wheel=")); Serial.print((char)('A' + MV_WHEEL[k]));
      Serial.print(F(" sag=")); Serial.println(sagCount);
      mvEnd();
      return;
    }
  }
  int32_t s = 256;
  if(rem < MV_SLOW) s = rem * 256 / MV_SLOW;
  int32_t sMin = (int32_t)MV_MIN * 256 / mvPeak;
  if(sMin > 256) sMin = 256;
  if(s < sMin) s = sMin;
  tgtVx = mvVx * s / 256; tgtVy = mvVy * s / 256; tgtW = mvW * s / 256;
}

// ---- 시리얼 라인 버퍼 ----
char buf[48];
uint8_t blen = 0;

int32_t nextLong(char*& p){
  while(*p==' ') p++;
  int8_t sign=1;
  if(*p=='-'){ sign=-1; p++; } else if(*p=='+'){ p++; }
  int32_t v=0; uint8_t n=0;
  while(*p>='0' && *p<='9'){ if(n++ < 9) v=v*10+(*p-'0'); p++; }   // 9자리에서 자른다(넘침 방지)
  return sign*v;
}

int nextInt(char*& p){   // AVR int = 16비트 — 넘치면 뒤집히지 않게 자른다
  int32_t v = nextLong(p);
  return v > 32767 ? 32767 : v < -32767 ? -32767 : (int)v;
}

int8_t hexVal(char c){
  if(c>='0'&&c<='9') return c-'0';
  if(c>='a'&&c<='f') return c-'a'+10;
  if(c>='A'&&c<='F') return c-'A'+10;
  return -1;
}

// 라인 끝의 "*HH"를 검사하고 잘라낸다.
//  1  = 체크섬이 있고 일치      0 = 체크섬 없음      -1 = 있는데 불일치/형식오류
int8_t verifyCrc(char* line){
  char* star = NULL;
  for(char* p=line; *p; p++) if(*p=='*') star = p;
  if(!star) return 0;
  int8_t hi = hexVal(star[1]);
  int8_t lo = star[1] ? hexVal(star[2]) : -1;
  if(hi < 0 || lo < 0) return -1;
  uint8_t want = (uint8_t)((hi<<4) | lo);
  uint8_t got = 0;
  for(char* p=line; p<star; p++) got ^= (uint8_t)(*p);
  *star = 0;                       // 페이로드만 남긴다
  return (got == want) ? 1 : -1;
}

const __FlashStringHelper* phaseName(uint8_t p){
  switch(p){
    case PH_IDLE:  return F("idle");
    case PH_RX:    return F("serial");
    case PH_I2C:   return F("I2C");     // ← PCA9685 쓰는 중 = TWI 버스 행 의심
    case PH_SETHZ: return F("setHz");   // ← F 명령의 delay(100)
    case PH_STOP:  return F("stop");
    case PH_TEST:  return F("test");
    default:       return F("boot");
  }
}

bool bootWarm = false;    // SRAM이 살아남았나 (= 전원이 유지된 채 칩만 리셋)
bool bootByWdt = false;   // 워치독 ISR이 자백을 남겼나

// 부팅 흔적을 확정한다 — setup에서 **딱 한 번**만.
void latchBootInfo(){
  bootWarm  = (bootMagic == BOOT_MAGIC);
  bootByWdt = bootWarm && (wdtHit == WDT_MAGIC);
  if(bootWarm){ prevPhase = bootByWdt ? wdtPhase : lastPhase; bootCount++; }
  else        { prevPhase = PH_BOOT; bootCount = 1; }   // 콜드 = SRAM이 깨졌다
  bootMagic = BOOT_MAGIC;
  wdtHit = 0;               // 다음 부팅이 이 자백을 물려받지 않게 지운다
  lastPhase = PH_BOOT;
}

// "왜 재부팅했나"를 한 줄로. 젯슨(motor_link)이 이 줄을 파싱해 대시보드에 띄운다.
//
// 판정 근거 — MCUSR이 아니라 **우리가 남긴 흔적**이다(이 부트로더는 MCUSR을 지운다):
//   WDT    : 워치독 ISR이 자백을 남겼다 → loop가 250ms 넘게 막혔다. last=가 어디서인지.
//   POWER  : SRAM이 깨졌다(cold) → 전원이 실제로 나갔다. USB/전원계 문제.
//   EXT|BOD: warm인데 워치독은 아니다 → 리셋핀(DTR=젯슨이 포트를 연 것) 아니면
//            브라운아웃. 젯슨은 자기가 포트를 열었는지 아니까 그쪽에서 갈린다.
void printBootReport(){
  Serial.print(F("boot #")); Serial.print(bootCount);
  Serial.print(bootWarm ? F(" warm") : F(" cold"));
  Serial.print(F(" cause="));
  if(bootByWdt)       Serial.print(F("WDT"));
  else if(!bootWarm)  Serial.print(F("POWER"));
  else                Serial.print(F("EXT|BOD"));
  Serial.print(F(" last=")); Serial.print(phaseName(prevPhase));
  Serial.print(F(" i2cerr=")); Serial.print(i2cErr);
  Serial.print(F(" near=")); Serial.print(wdtNear);
  // 참고용 원시값. 이 보드에선 둘 다 못 믿지만(부트로더가 지움), 다른 보드로
  // 옮겼을 때 유효하면 그때는 더 정확한 정보다. mcusr 상위 4비트는 항상 0이어야
  // 하므로 그걸로 "믿을 수 있는 값인지"를 판별할 수 있다.
  Serial.print(F(" mcusr=0x")); Serial.print(mcusrSave, HEX);
  Serial.print(F(" r2=0x")); Serial.println(r2Save, HEX);
}

void handleLine(char* line){
  // ★ 명령 하나마다 워치독 예산을 새로 준다. wdt_reset()은 loop() 맨 위에만
  //   있었는데, 버퍼에 쌓인 명령은 **한 loop 패스 안에서 전부** 처리된다.
  //   F가 두 개 들어오면 delay(100)×2 = 200ms라 250ms에 아슬아슬했고, 그 사이
  //   V 몇 개가 더 끼면 넘긴다 — 재부팅→튜닝 재적용(F,P,R)→다시 재부팅의
  //   자기증폭 고리가 여기서 시작됐을 수 있다(2026-08-11 acks에 R이 두 번 찍혔다).
  wdt_reset();
  lastPhase = PH_RX;
  int8_t crc = verifyCrc(line);
  if(crc < 0){
    rxBad++;
    Serial.println(F("nak crc"));
    return;                        // 깨진 라인은 절대 실행하지 않는다
  }
  if(crc > 0) strictCrc = true;    // 정상 체크섬을 받은 순간부터 strict
  else if(strictCrc && (line[0]=='V' || line[0]=='M')){   // 바퀴를 움직이는 명령만
    // strict 전환 뒤 무체크섬 속도지령 = 노이즈이거나 옛 클라이언트. 주행만 막고 알린다.
    rxBad++;
    Serial.println(F("nak nocrc"));
    return;
  }

  rxOk++;
  lastCmd = millis();
  char c = line[0];
  char* p = line+1;
  if(c=='V'){
    int vx=nextInt(p), vy=nextInt(p), w=nextInt(p);
    if(vx>255)vx=255; if(vx<-255)vx=-255;
    if(vy>255)vy=255; if(vy<-255)vy=-255;
    if(w >255)w =255; if(w <-255)w =-255;
    mvAbort(F("V"));               // 사람·상위가 직접 잡았다 — 이동보다 우선
    tgtVx=vx; tgtVy=vy; tgtW=w;
    testing = false;
    // 응답 없음 — 50Hz로 들어오므로 되쏘면 TX 버퍼가 차서 loop가 막힌다.
  } else if(c=='S'){
    mvAbort(F("S")); hardStop(); Serial.println(F("ok S"));
  } else if(c=='T'){
    int i=nextInt(p), d=nextInt(p);
    if(i>=0 && i<4){
      mvAbort(F("T")); hardStop();
      driveWheel(i, d>=0 ? 200 : -200);
      testing = true;
      testUntil = millis()+1000;
      testWheel = i; testEnc0 = encGet(i);
      Serial.print(F("ok T ")); Serial.print(i); Serial.print(' '); Serial.println(d);
    }
  } else if(c=='P'){
    int n=nextInt(p); if(n>0&&n<=4095){ MAX_PWM=n; }
    Serial.print(F("ok P ")); Serial.println(MAX_PWM);
  } else if(c=='L'){
    for(uint8_t i=0;i<4;i++){ int v=nextInt(p); POL[i] = (v<0)?-1:1; }
    mvAbort(F("L")); hardStop();
    Serial.print(F("ok L POL=")); for(uint8_t i=0;i<4;i++){Serial.print((int)POL[i]);Serial.print(',');} Serial.println();
  } else if(c=='R'){
    int a=nextInt(p), d=nextInt(p);
    if(a>0 && a<=255) ACCEL=a;
    if(d>0 && d<=255) DECEL=d;
    Serial.print(F("ok R ")); Serial.print(ACCEL); Serial.print(' '); Serial.println(DECEL);
  } else if(c=='F'){
    int hz=nextInt(p);
    if(hz>=24 && hz<=1526){
      // set_hz는 라이브러리 안에서 delay(100)을 태운다(PCA9685 재시작 대기).
      // 그 100ms가 워치독 예산을 통째로 먹으므로 앞뒤로 감아준다.
      lastPhase = PH_SETHZ;
      wdt_reset(); faboPWM.set_hz(hz); wdt_reset();
      applyCurrent();
    }
    Serial.print(F("ok F ")); Serial.println(hz);
  } else if(c=='M'){
    int32_t id=nextLong(p), ticks=nextLong(p);
    int vx=nextInt(p), vy=nextInt(p), w=nextInt(p);
    mvStart(id > 0 && id <= 65535 ? (uint16_t)id : 0, ticks,
            constrain(vx,-255,255), constrain(vy,-255,255), constrain(w,-255,255));
  } else if(c=='H'){
    // 살아 있음 — 데드맨(lastCmd)은 위에서 이미 채웠다. 응답 없음.
  } else if(c=='K'){
    int a=nextInt(p), b=nextInt(p), st=nextInt(p), sg=nextInt(p), bg=nextInt(p);
    if(a>0) MV_SLOW=a;
    if(b>0 && b<=255) MV_MIN=b;
    if(st>0) MV_STALL_MS=st;
    if(sg>0) VSAG_MV=sg;
    if(bg>=900 && bg<=1300) BG_MV=bg;
    Serial.print(F("ok K ")); Serial.print(MV_SLOW); Serial.print(' '); Serial.print(MV_MIN);
    Serial.print(' '); Serial.print(MV_STALL_MS); Serial.print(' '); Serial.print(VSAG_MV);
    Serial.print(' '); Serial.println(BG_MV);
  } else if(c=='W'){
    // ⚠ 이동(M) 도중이면 기록을 **안 지운다** — 재부팅 뒤 "mv lost"가 나와야 정상(브라운아웃 경로 시험).
    // 워치독 자가진단 — **일부러** loop를 멈춘다. 이 계측의 존재 이유가
    // "워치독이 물었다"를 신뢰성 있게 보고하는 것인데, 그게 실제로 동작하는지
    // 확인할 방법이 없으면 0이 나와도 그게 '안 물렸다'인지 '못 잡았다'인지 모른다.
    // 안전: 먼저 바퀴를 세우고 멈춘다. 250ms 뒤 ISR이 자백을 남기고,
    //      다시 250ms 뒤 리셋되어 boot 줄에 cause=WDT last=test로 나온다.
    hardStop();
    Serial.println(F("ok W (일부러 멈춥니다 — 워치독이 500ms 안에 살려야 정상)"));
    Serial.flush();
    lastPhase = PH_TEST;
    for(;;) { }
  } else if(c=='?'){
    Serial.print(F("st cur=")); Serial.print(curVx); Serial.print(','); Serial.print(curVy);
    Serial.print(','); Serial.print(curW);
    Serial.print(F(" MAX_PWM=")); Serial.print(MAX_PWM);
    Serial.print(F(" slew=")); Serial.print(ACCEL); Serial.print('/'); Serial.print(DECEL);
    Serial.print(F(" strict=")); Serial.print(strictCrc);
    Serial.print(F(" i2cerr=")); Serial.print(i2cErr);
    Serial.print(F(" POL=")); for(uint8_t i=0;i<4;i++){Serial.print(POL[i]);Serial.print(',');}
    Serial.print(F(" pind=0x")); Serial.print(PIND, HEX);   // 엔코더 핀 실레벨: bit2,3=FL bit4,6=FR
    Serial.println();
    printBootReport();
  }
}

// 리셋 직전에 하던 이동을 보고한다(warm + 체크섬이 맞을 때만). 다시 출발은 젯슨이 정한다.
void reportLostMove(){
  if(bootWarm && mvRecId && mvRecChk == mvChk(mvRecId, mvRecTarget, mvRecGot)){
    Serial.print(F("mv lost ")); Serial.print(mvRecId); Serial.print(' ');
    Serial.print(mvRecGot); Serial.print(' '); Serial.println(mvRecTarget);
  }
  mvRecId = 0;
}

void setup(){
  wdt_disable();
  latchBootInfo();          // .init3가 남긴 흔적을 확정 — 출력보다 먼저
  Serial.begin(115200);
  // ★ I2C 하드닝. AVR TWI는 버스가 물리면(모터 스위칭 노이즈로 SDA가 눌리면)
  //   endTransmission()에서 **타임아웃 없이 무한 대기**한다 → loop가 250ms를 넘겨
  //   워치독이 문다. "USB는 멀쩡한데 보드만 재부팅"의 1순위 후보라 여기를 막는다.
  //   (Wire.begin()은 FaBoPWM 생성자가 이미 불렀다 — 전역 객체라 setup보다 먼저 돈다.)
  // ⚠ setWireTimeout/getWireTimeoutFlag는 arduino:avr **1.8.1+** 에만 있다.
  //   흔히 쓰는 `#if defined(WIRE_HAS_TIMEOUT)` 가드는 이 코어(1.8.8)에 그 매크로가
  //   없어서 **블록을 통째로 날려버린다** — 컴파일은 되는데 타임아웃이 안 걸린
  //   펌웨어가 나온다. 그러니 가드 없이 직접 부른다(구형 코어면 컴파일 에러로
  //   즉시 드러나는 편이 조용히 무력화되는 것보다 낫다).
  Wire.setWireTimeout(3000, true);   // 3ms 넘으면 포기 + TWI 리셋
  // 100kHz → 400kHz. applyCurrent()는 채널당 4트랜잭션 × 8채널이라 100kHz에서
  // ~9.5ms가 걸렸다 — 슬루 틱(5ms)보다 길어서 램프가 사실상 절반 속도로 돌았다.
  Wire.setClock(400000);
  if(faboPWM.begin()){
    Serial.println(F("Find PCA9685 OK"));
    faboPWM.init(0);              // 부팅 순간 채널 전부 0 — v1의 init(300)은 기동 시 미세 통전
  } else {
    Serial.println(F("PCA9685 begin FALSE"));
  }
  faboPWM.set_hz(PWM_HZ_DEFAULT); // ★ 50 → 1500Hz. 저주파 "우우웅" 제거
  hardStop();
  encInit();
  lastCmd = lastTick = millis();
  // 배너보다 **먼저** 리셋 원인을 뱉는다 — 젯슨은 배너를 보고 "재부팅했다"고
  // 판정하므로, 그 시점엔 원인 줄이 이미 도착해 있어야 함께 로그에 남는다.
  printBootReport();
  reportLostMove();         // 배너보다 먼저 — 젯슨이 재부팅을 알아챌 때 이미 손에 있게
  vccInit();
  Serial.println(F("mecanum_stable v2.3 ready. proto: V vx vy w | S | T i d | P n | L .. | R a d | F hz | M id t vx vy w | H | K .. | ? (+*CRC)"));
  armWatchdog();   // HW 워치독 250ms — 인터럽트 우선(원인을 남기고 리셋)
}

void loop(){
  wdt_reset();
  // ISR이 한 번 뜨면 WDIE가 자동으로 꺼진다(그 다음 타임아웃이 진짜 리셋).
  // 여기까지 왔다는 건 막혔다가 **살아났다**는 뜻이므로, 다시 무장해 다음
  // 사고도 잡는다. wdtNear가 늘어나는데 재부팅은 없다면 = 아슬아슬한 근접 사고.
  if(!(WDTCSR & _BV(WDIE))) armWatchdog();

  // 시리얼 수신 (논블로킹)
  while(Serial.available()){
    char ch = Serial.read();
    if(ch=='\n' || ch=='\r'){
      if(blen>0){ buf[blen]=0; handleLine(buf); blen=0; }
    } else if(blen < sizeof(buf)-1){
      buf[blen++]=ch;
    } else {
      blen = 0;   // 오버플로 라인은 통째로 버린다(깨진 프레임을 실행하지 않게)
    }
  }

  // T 테스트 자동정지
  if(testing && (long)(millis()-testUntil) >= 0){
    hardStop(); Serial.print(F("T auto-stop enc=")); Serial.println(encGet(testWheel) - testEnc0);
  }

  unsigned long now = millis();

  // 데드맨: 소프트(감속) → 하드(즉시 0)
  if(!testing){
    unsigned long silence = now - lastCmd;
    if(silence > HARD_TIMEOUT){
      mvAbort(F("deadman"));
      if(curVx || curVy || curW){ hardStop(); Serial.println(F("deadman HARD")); }
    } else if(silence > CMD_TIMEOUT){
      mvAbort(F("deadman"));      // 젯슨이 죽었다 — 남은 거리가 있어도 혼자 가지 않는다
      tgtVx = tgtVy = tgtW = 0;   // 슬루로 부드럽게 선다 — v1의 "주웅 주웅" 끊김 제거
    }
  }

  // 슬루 틱: 목표로 조금씩 이동
  if(!testing && (now - lastTick) >= TICK_MS){
    lastTick = now;
    vccTick(now);
    if(mvState) mvTick(now);
    bool sag = sagging(now);
    int nx = sag ? approachSag(curVx, tgtVx) : approach(curVx, tgtVx);
    int ny = sag ? approachSag(curVy, tgtVy) : approach(curVy, tgtVy);
    int nw = sag ? approachSag(curW,  tgtW)  : approach(curW,  tgtW);
    if(nx!=curVx || ny!=curVy || nw!=curW){
      curVx=nx; curVy=ny; curW=nw;
      applyCurrent();
    }
  }

  // 하트비트 1s — 링크 품질(수신/오류 카운터)과 실제 인가 속도를 젯슨에 보고
  if(now - lastHb > 1000){
    lastHb = now;
    Serial.print(F("hb ")); Serial.print(now);
    Serial.print(F(" rx=")); Serial.print(rxOk);
    Serial.print(F(" bad=")); Serial.print(rxBad);
    // I2C가 물린 횟수. 0이 아니면 TWI 버스가 노이즈로 눌리고 있다는 직접 증거다
    // (v2.1 전에는 이때 그냥 행에 걸려 워치독 리셋으로 끝났다).
    Serial.print(F(" i2c=")); Serial.print(i2cErr);
    Serial.print(F(" wdt=")); Serial.print(wdtNear);
    Serial.print(F(" v=")); Serial.print(curVx); Serial.print(',');
    Serial.print(curVy); Serial.print(','); Serial.print(curW);
    Serial.print(F(" enc="));   // STM32와 같은 순서 A,B,C,D — Uno는 A·B가 늘 0(쉴드 NC)
    for(uint8_t i = 0; i < 4; i++){ Serial.print(encGet(i)); if(i < 3) Serial.print(','); }
    // 로직 5V — vmin은 지난 1초의 최저값. 리셋 직전 hb의 vmin이 브라운아웃이었는지를 말한다
    Serial.print(F(" vcc=")); Serial.print(vccMv);
    Serial.print(F(" vmin=")); Serial.print(vccMin == 0xFFFF ? vccMv : vccMin);
    Serial.print(F(" sag=")); Serial.print(sagCount);
    vccMin = 0xFFFF;
    if(mvState){ Serial.print(F(" mv=")); Serial.print(mvId); Serial.print(','); Serial.print(mvProgress()); }
    Serial.println();
  }

  lastPhase = PH_IDLE;   // 한 바퀴를 무사히 돌았다 — 여기서 죽으면 last=idle
}
