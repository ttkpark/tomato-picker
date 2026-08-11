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
//   T <i> <d>         모터 i(0=A/FR 1=B/FL 2=C/RR 3=D/RL) 방향 d(1/-1) 단독 1초 (캘리브레이션)
//   P <n>             최대 PWM 스케일 설정 (기본 2000 / 4095)
//   L <a><b><c><d>    바퀴 극성 (각 1/-1)
//   R <accel> <decel> 슬루 스텝(틱당 변화량 상한). 기본 6 / 12
//   F <hz>            PWM 주파수 변경 (24~1526). 소음 튜닝용
//   ?                 상태 1줄 출력
// 응답: 처리 확인 "ok ...", 체크섬 오류 "nak crc", 1초 주기 "hb <ms> rx=<n> bad=<n> v=<vx,vy,w>"
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

// 실측 매핑(2026-07-02): A=FR, B=FL, C=RR, D=RL
// 각 바퀴 "물리적 전진"이 어느 전기방향인지 부호로 보정 (캘리브레이션으로 확정).
// idx: 0=A(FR) 1=B(FL) 2=C(RR) 3=D(RL)
// 2026-07-03 실측: V140,0,0에서 좌(B,D)전진/우(A,C)후진 → 우측 반전으로 전진 정합.
int8_t POL[4] = {-1, 1, -1, 1};   // A(FR)반전 B(FL)정 C(RR)반전 D(RL)정. 런타임 'L'로 변경가능
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
// 바퀴→채널: A=FR, B=FL, C=RR, D=RL
void applyCurrent(){
  // ★ 흔적을 먼저 남긴다. 여기서 리셋되면 다음 부팅이 last=I2C로 보고하고,
  //   그게 곧 "TWI 버스가 물려 워치독이 물었다"는 증거다.
  lastPhase = PH_I2C;
  int FL = curVx - curVy - curW;
  int FR = curVx + curVy + curW;
  int RL = curVx + curVy - curW;
  int RR = curVx - curVy + curW;
  driveWheel(0, FR);  // A
  driveWheel(1, FL);  // B
  driveWheel(2, RR);  // C
  driveWheel(3, RL);  // D
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

// ---- 시리얼 라인 버퍼 ----
char buf[48];
uint8_t blen = 0;

int nextInt(char*& p){
  while(*p==' ') p++;
  int sign=1;
  if(*p=='-'){ sign=-1; p++; } else if(*p=='+'){ p++; }
  int v=0; bool any=false;
  while(*p>='0' && *p<='9'){ v=v*10+(*p-'0'); p++; any=true; }
  return any ? sign*v : 0;
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
  else if(strictCrc && line[0]=='V'){
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
    tgtVx=vx; tgtVy=vy; tgtW=w;
    testing = false;
    // 응답 없음 — 50Hz로 들어오므로 되쏘면 TX 버퍼가 차서 loop가 막힌다.
  } else if(c=='S'){
    hardStop(); Serial.println(F("ok S"));
  } else if(c=='T'){
    int i=nextInt(p), d=nextInt(p);
    if(i>=0 && i<4){
      hardStop();
      driveWheel(i, d>=0 ? 200 : -200);
      testing = true;
      testUntil = millis()+1000;
      Serial.print(F("ok T ")); Serial.print(i); Serial.print(' '); Serial.println(d);
    }
  } else if(c=='P'){
    int n=nextInt(p); if(n>0&&n<=4095){ MAX_PWM=n; }
    Serial.print(F("ok P ")); Serial.println(MAX_PWM);
  } else if(c=='L'){
    for(uint8_t i=0;i<4;i++){ int v=nextInt(p); POL[i] = (v<0)?-1:1; }
    hardStop();
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
  } else if(c=='W'){
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
    Serial.println();
    printBootReport();
  }
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
  lastCmd = lastTick = millis();
  // 배너보다 **먼저** 리셋 원인을 뱉는다 — 젯슨은 배너를 보고 "재부팅했다"고
  // 판정하므로, 그 시점엔 원인 줄이 이미 도착해 있어야 함께 로그에 남는다.
  printBootReport();
  Serial.println(F("mecanum_stable v2.1 ready. proto: V vx vy w | S | T i d | P n | L .. | R a d | F hz | ? (+*CRC)"));
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
  if(testing && (long)(millis()-testUntil) >= 0){ hardStop(); Serial.println(F("T auto-stop")); }

  unsigned long now = millis();

  // 데드맨: 소프트(감속) → 하드(즉시 0)
  if(!testing){
    unsigned long silence = now - lastCmd;
    if(silence > HARD_TIMEOUT){
      if(curVx || curVy || curW){ hardStop(); Serial.println(F("deadman HARD")); }
    } else if(silence > CMD_TIMEOUT){
      tgtVx = tgtVy = tgtW = 0;   // 슬루로 부드럽게 선다 — v1의 "주웅 주웅" 끊김 제거
    }
  }

  // 슬루 틱: 목표로 조금씩 이동
  if(!testing && (now - lastTick) >= TICK_MS){
    lastTick = now;
    int nx = approach(curVx, tgtVx);
    int ny = approach(curVy, tgtVy);
    int nw = approach(curW,  tgtW);
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
    Serial.print(curVy); Serial.print(','); Serial.println(curW);
  }

  lastPhase = PH_IDLE;   // 한 바퀴를 무사히 돌았다 — 여기서 죽으면 last=idle
}
