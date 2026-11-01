// mecanum_stm32 — 젯슨 시리얼 주행 펌웨어 v3 (STM32F405 보드). mecanum_stable v2.1(Uno+PCA9685)을 대체한다.
// 프로토콜은 v2.1과 같다 — 젯슨 motor_link.py를 한 줄도 안 고치고 붙는다. 바뀐 것은 보드 쪽뿐:
//
//  (1) PCA9685(I2C) 대신 타이머 PWM 직결. v2.1 워치독의 1순위 용의자였던 "I2C 버스 행"이
//      원리적으로 없다. 호환을 위해 i2c=/i2cerr= 칸은 남기되 항상 0이다.
//  (2) PWM 24~40000Hz. PCA9685 상한(1526Hz)이 없으니 20kHz(초음파)로 올려 "소음↔속도" 맞바꿈을
//      끊을 수 있다. 그래도 기본값은 Uno와 같은 1500Hz — 젯슨이 첫 hb 뒤 F/P/R을 되밀어 넣으므로
//      기본을 바꾸면 "보드를 바꿔서 달라진 것"과 "주파수를 바꿔서 달라진 것"이 섞인다.
//  (3) 엔코더 4채널을 **타이머 엔코더 모드**로 센다. 인터럽트로 세지 않는다 — 카운트는 하드웨어가
//      하고 loop는 읽기만 하므로, loop가 직렬 송신으로 몇 ms 막혀도 펄스를 하나도 안 잃는다.
//      TIM2/TIM5는 32비트, TIM3/TIM4는 16비트라 소프트웨어로 32비트로 이어 붙인다.
//  (4) 젯슨이 포트를 열어도 **리셋되지 않는다.** CH340의 DTR/RTS는 절연기 너머로 건너오지 않는다
//      (배선 자체가 없다). Uno는 포트를 다시 열면 DTR로 리셋됐고, 그게 "멈춘 보드를 원격으로 살리는
//      유일한 수단"이었다(motor_link HB_DEAD_SEC). 여기선 IWDG 혼자 그 일을 맡는다 — IWDG는 한 번
//      켜면 소프트웨어로 끌 수 없어서 오히려 믿을 만하다.
//  (5) 리셋 원인은 RCC_CSR이 정확히 남긴다(AVR처럼 부트로더가 지우지 않는다). 그래서 v2.1의
//      "워치독 인터럽트로 자백 받기" 꼼수는 필요 없다. 흔적(부팅 수·마지막 국면)은 BKPSRAM에 둔다 —
//      이 코어의 링커 스크립트엔 .noinit이 없다. 전원이 실제로 나가면 지워진다 = warm/cold 판정.
//  (6) BOR 문턱을 옵션바이트로 Level 3(≈2.7V)에 못 박는다. 공장값은 BOR 꺼짐이라 3.3V 레일이 처져도
//      리셋 대신 "이상 동작"을 한다 — 그러면 cause=BOD가 영영 안 찍힌다.
//  (7) U = 시스템 부트로더 진입. 젯슨에서 같은 포트로 stm32flash가 굽는다(BOOT0 버튼 불필요).
//
// 프로토콜 (라인 단위, '\n' 종료. 모든 라인에 선택적으로 "*HH" 체크섬):
//   V <vx> <vy> <w>   속도지령. vx=전진+, vy=우평행+, w=시계회전+. 각 -255..255. (무응답)
//   S                 즉시 정지 (슬루 무시, 하드)
//   T <i> <d>         모터 i(0=A/RR 1=B/RL 2=C/FR 3=D/FL) 방향 d(1/-1) 단독 1초. 끝에 엔코더 증분을 보고
//   P <n>             최대 PWM 스케일 (0..4095 척도 유지. 기본 2000)
//   L <a><b><c><d>    바퀴 극성 (각 1/-1)
//   R <accel> <decel> 슬루 스텝. 기본 6 / 12
//   F <hz>            PWM 주파수 (24~40000)
//   W                 워치독 자가진단(일부러 멈춤 → cause=WDT last=test 로 돌아와야 정상)
//   Q                 cap 한 줄 (보드계약 §6, proto=1 레거시 프로파일)
//   U                 정지 후 시스템 부트로더로 (stm32flash)
//   ?                 상태 + boot 보고
// hb: "hb <ms> rx= bad= i2c=0 wdt= v=<vx,vy,w>" 뒤에 enc= vin= v5= amp= ampw= flt= fltn= 을 덧붙인다
//     (motor_link·board_contract 둘 다 모르는 키는 무시한다).
#include <IWatchdog.h>
#include "stm32f4xx_ll_adc.h"

// ---------------- 배선 (기판과 1:1 — 바꾸면 기판도 바뀐다) ----------------
// 모터 = DRV8874 ×4, PWM 모드(IN1/IN2). IN1=PWM·IN2=0 → 정방향 구동/코스트 — PCA9685+HR8833과 같은 감쇠.
struct MotorPwm { TIM_TypeDef* t; uint8_t c1, c2; };
const MotorPwm MOT[4] = {
  {TIM8, 1, 2},    // A/RR  PC6 / PC7
  {TIM8, 3, 4},    // B/RL  PC8 / PC9
  {TIM9, 1, 2},    // C/FR  PA2 / PA3
  {TIM12, 1, 2},   // D/FL  PB14 / PB15
};
TIM_TypeDef* const PWM_TIMS[3] = {TIM8, TIM9, TIM12};
// 엔코더: A=TIM2(PA15/PB3) B=TIM3(PB4/PB5) C=TIM4(PB6/PB7) D=TIM5(PA0/PA1)
TIM_TypeDef* const ENC_T[4] = {TIM2, TIM3, TIM4, TIM5};
const bool ENC16[4] = {false, true, true, false};
// A/B 상 순서는 모터 배선에 달렸다. "전기적 + 구동에서 카운트가 느는" 쪽이 +1.
// T i 1 의 "T auto-stop enc=" 가 음수로 나오면 그 바퀴를 -1로 바꿀 것.
const int8_t ENC_DIR[4] = {1, 1, 1, 1};   // ponytail: 상수 — 모터를 자주 바꾸게 되면 런타임 명령으로
const uint32_t PIN_NFAULT[4] = {PC10, PC11, PC12, PD2};
const uint32_t PIN_IPROPI[4] = {PC0, PC1, PC2, PC3};
#define PIN_NSLEEP PB12   // 4개 드라이버 공통. 기판 풀다운 → 펌웨어가 깨우기 전엔 출력 Hi-Z
#define PIN_VM     PC4    // 47k/10k
#define PIN_V5     PC5    // 10k/10k
#define PIN_LED    PB0    // 하트비트마다 깜빡
#define PIN_LEDF   PB1    // 드라이버 폴트
// 실측 보정 손잡이 — 저항 오차·IPROPI 편차는 여기서 맞춘다.
const uint32_t VM_DIV_X100 = 570;    // (47k+10k)/10k
const uint32_t V5_DIV_X100 = 200;
const uint32_t IPROPI_MA_PER_V = 2222; // 1kΩ × 450µA/A = 0.45V/A

// ---------------- 리셋 흔적 (BKPSRAM: 리셋엔 살아남고 전원이 나가면 깨진다) ----------------
#define BOOT_MAGIC    0xB07E5732u
#define BOOTLDR_MAGIC 0x5AFEB007u
struct BootTrace {
  uint32_t magic;
  uint32_t bootCount;
  uint32_t enterBootloader;   // U가 남긴다 → 다음 부팅 맨 앞에서 부트로더로 뛴다
  uint8_t  lastPhase;         // 지금 하고 있는 일
  uint8_t  prevPhase;         // 리셋 직전에 하던 일
};
#define BT ((volatile BootTrace*)BKPSRAM_BASE)

enum { PH_BOOT = 0, PH_IDLE, PH_RX, PH_PWM, PH_SETHZ, PH_STOP, PH_TEST };

static void backupOn(){
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;     (void)RCC->APB1ENR;
  PWR->CR |= PWR_CR_DBP;
  RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN; (void)RCC->AHB1ENR;
}

// ★ 리셋 직후 아무것도 켜기 전에(클럭·SysTick·IWDG 전부 리셋 상태) 부트로더로 뛴다.
//   실행 중에 바로 뛰면 켜 둔 타이머·인터럽트·클럭을 하나하나 되돌려야 하고, 무엇보다 IWDG는
//   끌 수가 없어 부트로더가 굽는 도중에 물린다. 리셋 한 번이 그걸 전부 공짜로 해 준다.
// ⚠ constructor(101)로는 늦다 — 코어의 premain()이 같은 101로 먼저 돌며 HAL_Init·클럭·SysTick을
//   켠다(.init_array 실측 순서). .preinit_array는 그보다 앞에서 돈다.
extern "C" void earlyBoot(void){
  backupOn();
  if(BT->enterBootloader != BOOTLDR_MAGIC) return;
  BT->enterBootloader = 0;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; (void)RCC->APB2ENR;
  SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0;          // 0번지 = 시스템 메모리
  const volatile uint32_t* sys = (const volatile uint32_t*)0x1FFF0000u;
  uint32_t sp = sys[0], pc = sys[1];
  __set_MSP(sp);
  ((void (*)(void))pc)();
}
extern "C" __attribute__((used, section(".preinit_array"))) void (*const earlyBootHook)(void) = earlyBoot;

// ---------------- 클럭: 8MHz HSE → 168MHz (실패하면 HSI로) ----------------
// generic F405 변형은 HSI를 쓴다. 기판에 크리스털이 있으니 HSE로 — UART 보율 오차가 준다.
bool clkHse = false;
extern "C" void SystemClock_Config(void){
  RCC_OscInitTypeDef o = {};
  RCC_ClkInitTypeDef c = {};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  o.HSEState = RCC_HSE_ON;
  o.PLL.PLLState = RCC_PLL_ON;
  o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  o.PLL.PLLM = 4; o.PLL.PLLN = 168; o.PLL.PLLP = RCC_PLLP_DIV2; o.PLL.PLLQ = 7;   // 8/4*168/2=168
  clkHse = (HAL_RCC_OscConfig(&o) == HAL_OK);
  if(!clkHse){
    // 크리스털이 안 뜬다(납땜·부품) — 멈추지 말고 HSI로라도 돈다. ? 의 clk= 로 드러난다.
    o = RCC_OscInitTypeDef{};
    o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    o.HSIState = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState = RCC_PLL_ON;
    o.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    o.PLL.PLLM = 8; o.PLL.PLLN = 168; o.PLL.PLLP = RCC_PLLP_DIV2; o.PLL.PLLQ = 7;  // 16/8*168/2=168
    if(HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();
  }
  c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  c.AHBCLKDivider = RCC_SYSCLK_DIV1;
  c.APB1CLKDivider = RCC_HCLK_DIV4;   // 42MHz → APB1 타이머 84MHz
  c.APB2CLKDivider = RCC_HCLK_DIV2;   // 84MHz → APB2 타이머 168MHz
  if(HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

// ---------------- 상태 (v2.1과 같은 이름·같은 기본값) ----------------
int8_t POL[4] = {-1, 1, -1, 1};   // 모터 A반전 B정 C반전 D정. 런타임 'L'로 변경가능
int MAX_PWM = 2000;               // 0..4095 척도 (PCA9685 시절 그대로 — 젯슨의 P 값이 그대로 통한다)
const uint16_t PWM_HZ_DEFAULT = 1500;
uint16_t pwmHz = PWM_HZ_DEFAULT;

const unsigned long CMD_TIMEOUT  = 300;
const unsigned long HARD_TIMEOUT = 1000;
const unsigned long TICK_MS = 5;
int ACCEL = 6;
int DECEL = 12;

unsigned long lastCmd = 0, lastTick = 0, lastHb = 0, testUntil = 0, lastLoop = 0;
int tgtVx = 0, tgtVy = 0, tgtW = 0;
int curVx = 0, curVy = 0, curW = 0;
uint16_t rxOk = 0, rxBad = 0;
bool strictCrc = false;
bool testing = false;
int8_t testWheel = -1;
int32_t testEnc0 = 0;
uint16_t wdtNear = 0;             // loop 한 바퀴가 200ms를 넘긴 횟수(=워치독 근접 사고)

uint32_t encLast[4];
int32_t  encAcc[4];               // 바퀴 전진 = +
uint8_t  fltMask = 0;
uint16_t fltCount = 0;

bool bootWarm = false;
uint32_t csrSave = 0;

// ---------------- 레지스터 도우미 ----------------
static void pinAF(GPIO_TypeDef* g, uint8_t n, uint8_t af, bool pullup){
  volatile uint32_t& afr = g->AFR[n >> 3];
  afr = (afr & ~(0xFu << (4 * (n & 7)))) | ((uint32_t)af << (4 * (n & 7)));   // AF 먼저, 그다음 모드
  g->PUPDR   = (g->PUPDR   & ~(3u << (2 * n))) | ((pullup ? 1u : 0u) << (2 * n));
  g->OSPEEDR = (g->OSPEEDR & ~(3u << (2 * n))) | (2u << (2 * n));
  g->MODER   = (g->MODER   & ~(3u << (2 * n))) | (2u << (2 * n));
}

static uint32_t timClk(TIM_TypeDef* t){
  bool apb2 = (t == TIM8 || t == TIM9);
  uint32_t pclk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  bool divided = RCC->CFGR & (apb2 ? RCC_CFGR_PPRE2_2 : RCC_CFGR_PPRE1_2);
  return divided ? pclk * 2 : pclk;   // APB가 분주되면 타이머 클럭은 2배
}

static inline void setCcr(TIM_TypeDef* t, uint8_t ch, uint32_t v){ (&t->CCR1)[ch - 1] = v; }

// 세 PWM 타이머를 같은 주파수로. ARR이 16비트(TIM9/12)에 들어가도록 PSC를 고른다.
static void setHz(uint32_t hz){
  for(TIM_TypeDef* t : PWM_TIMS){
    uint32_t ticks = timClk(t) / hz;
    uint32_t psc = ticks / 65536;
    t->PSC = psc;
    t->ARR = ticks / (psc + 1) - 1;
    t->EGR = TIM_EGR_UG;   // 즉시 반영. PCA9685의 delay(100) 재시작 같은 건 없다
  }
  pwmHz = hz;
}

static void pwmTimInit(TIM_TypeDef* t, bool four){
  t->CR1 = TIM_CR1_ARPE;
  t->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  if(four) t->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
  t->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | (four ? (TIM_CCER_CC3E | TIM_CCER_CC4E) : 0);
  if(t == TIM8) t->BDTR = TIM_BDTR_MOE;   // 고급 타이머는 MOE 없으면 출력이 안 나간다
}

// 엔코더 모드 3 = 두 상의 양 에지 = 4체배. 입력 필터 0xF(fDTS/32, 8샘플 ≈ 3µs) — 모터 스위칭
// 잡음 가시를 하드웨어에서 거른다. 정상 엔코더 에지 간격(수십 µs)보다 한참 짧다.
static void encInit(TIM_TypeDef* t){
  t->SMCR  = 3u;
  t->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | (0xFu << TIM_CCMR1_IC1F_Pos) | (0xFu << TIM_CCMR1_IC2F_Pos);
  t->CCER  = 0;
  t->ARR   = 0xFFFFFFFFu;   // 16비트 타이머엔 0xFFFF로 들어간다
  t->CNT   = 0;
  t->CR1   = TIM_CR1_CEN;
}

// 16비트 카운터는 차분을 int16으로 잘라 이어 붙인다 — 한 loop 사이에 32768카운트를 넘길 일은 없다.
void encPoll(){
  for(uint8_t i = 0; i < 4; i++){
    uint32_t c = ENC_T[i]->CNT;
    int32_t d = ENC16[i] ? (int32_t)(int16_t)(uint16_t)(c - encLast[i]) : (int32_t)(c - encLast[i]);
    encLast[i] = c;
    encAcc[i] += d * POL[i] * ENC_DIR[i];
  }
}

void faultPoll(){
  uint8_t m = 0;
  for(uint8_t i = 0; i < 4; i++) if(digitalRead(PIN_NFAULT[i]) == LOW) m |= 1u << i;
  for(uint8_t rise = m & ~fltMask; rise; rise >>= 1) fltCount += rise & 1;
  fltMask = m;
  digitalWrite(PIN_LEDF, m ? HIGH : LOW);
}

static uint32_t vdda(){
#ifdef AVREF
  uint32_t r = analogRead(AVREF);
  if(r) return __LL_ADC_CALC_VREFANALOG_VOLTAGE(r, LL_ADC_RESOLUTION_12B);
#endif
  return 3300;
}
static uint32_t adcMv(uint32_t pin, uint32_t va){ return (uint32_t)analogRead(pin) * va / 4095; }

// ---------------- 구동 (v2.1과 같은 수식) ----------------
void driveWheel(uint8_t w, int s){
  s = (int)s * POL[w];
  int mag = abs(s);
  if(mag > 255) mag = 255;
  const MotorPwm& m = MOT[w];
  uint32_t pwm = (uint32_t)mag * MAX_PWM / 255;          // 0..MAX_PWM (PCA9685 척도)
  uint32_t ccr = pwm * (m.t->ARR + 1) / 4095;            // → 이 타이머의 비교값
  setCcr(m.t, m.c1, s > 0 ? ccr : 0);
  setCcr(m.t, m.c2, s < 0 ? ccr : 0);
}

// 메카넘 mixing. FL = vx-vy-w ; FR = vx+vy+w ; RL = vx+vy-w ; RR = vx-vy+w
void applyCurrent(){
  BT->lastPhase = PH_PWM;
  int FL = curVx - curVy - curW;
  int FR = curVx + curVy + curW;
  int RL = curVx + curVy - curW;
  int RR = curVx - curVy + curW;
  driveWheel(0, RR);  // A
  driveWheel(1, RL);  // B
  driveWheel(2, FR);  // C
  driveWheel(3, FL);  // D
  BT->lastPhase = PH_IDLE;
}

void hardStop(){
  BT->lastPhase = PH_STOP;
  tgtVx = tgtVy = tgtW = 0;
  curVx = curVy = curW = 0;
  testing = false;
  applyCurrent();
}

int approach(int cur, int tgt){
  if(cur == tgt) return cur;
  int step = (abs(tgt) > abs(cur)) ? ACCEL : DECEL;
  if(tgt > cur){ cur += step; if(cur > tgt) cur = tgt; }
  else         { cur -= step; if(cur < tgt) cur = tgt; }
  return cur;
}

// ---------------- 시리얼 라인 (v2.1 그대로) ----------------
char buf[48];
uint8_t blen = 0;

int nextInt(char*& p){
  while(*p == ' ') p++;
  int sign = 1;
  if(*p == '-'){ sign = -1; p++; } else if(*p == '+'){ p++; }
  int v = 0; bool any = false;
  while(*p >= '0' && *p <= '9'){ v = v * 10 + (*p - '0'); p++; any = true; }
  return any ? sign * v : 0;
}

int8_t hexVal(char c){
  if(c >= '0' && c <= '9') return c - '0';
  if(c >= 'a' && c <= 'f') return c - 'a' + 10;
  if(c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

int8_t verifyCrc(char* line){
  char* star = NULL;
  for(char* p = line; *p; p++) if(*p == '*') star = p;
  if(!star) return 0;
  int8_t hi = hexVal(star[1]);
  int8_t lo = star[1] ? hexVal(star[2]) : -1;
  if(hi < 0 || lo < 0) return -1;
  uint8_t want = (uint8_t)((hi << 4) | lo);
  uint8_t got = 0;
  for(char* p = line; p < star; p++) got ^= (uint8_t)(*p);
  *star = 0;
  return (got == want) ? 1 : -1;
}

const char* phaseName(uint8_t p){
  switch(p){
    case PH_IDLE:  return "idle";
    case PH_RX:    return "serial";
    case PH_PWM:   return "pwm";
    case PH_SETHZ: return "setHz";
    case PH_STOP:  return "stop";
    case PH_TEST:  return "test";
    default:       return "boot";
  }
}

void latchBootInfo(){
  csrSave = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;           // 다음 부팅이 이번 원인을 물려받지 않게
  bootWarm = (BT->magic == BOOT_MAGIC);
  if(bootWarm){ BT->prevPhase = BT->lastPhase; BT->bootCount = BT->bootCount + 1; }
  else        { BT->prevPhase = PH_BOOT; BT->bootCount = 1; BT->enterBootloader = 0; }
  BT->magic = BOOT_MAGIC;
  BT->lastPhase = PH_BOOT;
}

// RCC_CSR 판독. POR이면 BOR도 같이 켜지므로 POR을 먼저 본다. PIN은 내부 리셋에도 켜지므로 마지막.
const char* bootCause(){
  if(csrSave & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) return "WDT";
  if(csrSave & RCC_CSR_PORRSTF) return "POWER";
  if(csrSave & RCC_CSR_BORRSTF) return "BOD";
  if(csrSave & RCC_CSR_SFTRSTF) return "SW";
  if(csrSave & RCC_CSR_PINRSTF) return "EXT";
  return "?";
}

// motor_link가 파싱하는 형식 그대로. mcusr= 칸에는 RCC_CSR 상위 바이트(플래그)를 싣는다.
void printBootReport(){
  Serial.print("boot #"); Serial.print(BT->bootCount);
  Serial.print(bootWarm ? " warm" : " cold");
  Serial.print(" cause="); Serial.print(bootCause());
  Serial.print(" last="); Serial.print(phaseName(BT->prevPhase));
  Serial.print(" i2cerr=0 near="); Serial.print(wdtNear);
  Serial.print(" mcusr=0x"); Serial.print(csrSave >> 24, HEX);
  Serial.println(" r2=0x0");
}

void printCap(){
  Serial.print("cap proto=1 fw=3.0.0 board=stm32f405 units=0 closed_loop=0 enc=4 calib=0 vin=1 amp=1 estop_hw=0 pwm_hz=");
  Serial.println(pwmHz);
}

// BOR 꺼짐(공장값)이면 Level 3로 한 번만 굽는다. 같은 값이면 안 건드린다(옵션바이트 마모 방지).
void ensureBor(){
  if((FLASH->OPTCR & FLASH_OPTCR_BOR_LEV) == OB_BOR_LEVEL3) return;
  FLASH_OBProgramInitTypeDef ob = {};
  ob.OptionType = OPTIONBYTE_BOR;
  ob.BORLevel = OB_BOR_LEVEL3;
  HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
  HAL_FLASHEx_OBProgram(&ob);
  HAL_FLASH_OB_Launch();
  HAL_FLASH_OB_Lock(); HAL_FLASH_Lock();
}

void handleLine(char* line){
  IWatchdog.reload();   // 명령 하나마다 예산을 새로 준다(v2.1과 같은 이유)
  BT->lastPhase = PH_RX;
  int8_t crc = verifyCrc(line);
  if(crc < 0){
    rxBad++;
    Serial.println("nak crc");
    return;
  }
  if(crc > 0) strictCrc = true;
  else if(strictCrc && line[0] == 'V'){
    rxBad++;
    Serial.println("nak nocrc");
    return;
  }

  rxOk++;
  lastCmd = millis();
  char c = line[0];
  char* p = line + 1;
  if(c == 'V'){
    int vx = nextInt(p), vy = nextInt(p), w = nextInt(p);
    if(vx > 255) vx = 255; if(vx < -255) vx = -255;
    if(vy > 255) vy = 255; if(vy < -255) vy = -255;
    if(w  > 255) w  = 255; if(w  < -255) w  = -255;
    tgtVx = vx; tgtVy = vy; tgtW = w;
    testing = false;
  } else if(c == 'S'){
    hardStop(); Serial.println("ok S");
  } else if(c == 'T'){
    int i = nextInt(p), d = nextInt(p);
    if(i >= 0 && i < 4){
      hardStop();
      encPoll();
      testWheel = i; testEnc0 = encAcc[i];
      driveWheel(i, d >= 0 ? 200 : -200);
      testing = true;
      testUntil = millis() + 1000;
      Serial.print("ok T "); Serial.print(i); Serial.print(' '); Serial.println(d);
    }
  } else if(c == 'P'){
    int n = nextInt(p); if(n > 0 && n <= 4095){ MAX_PWM = n; }
    Serial.print("ok P "); Serial.println(MAX_PWM);
  } else if(c == 'L'){
    for(uint8_t i = 0; i < 4; i++){ int v = nextInt(p); POL[i] = (v < 0) ? -1 : 1; }
    hardStop();
    Serial.print("ok L POL="); for(uint8_t i = 0; i < 4; i++){ Serial.print((int)POL[i]); Serial.print(','); } Serial.println();
  } else if(c == 'R'){
    int a = nextInt(p), d = nextInt(p);
    if(a > 0 && a <= 255) ACCEL = a;
    if(d > 0 && d <= 255) DECEL = d;
    Serial.print("ok R "); Serial.print(ACCEL); Serial.print(' '); Serial.println(DECEL);
  } else if(c == 'F'){
    int hz = nextInt(p);
    if(hz >= 24 && hz <= 40000){
      BT->lastPhase = PH_SETHZ;
      setHz(hz);
      applyCurrent();
    }
    Serial.print("ok F "); Serial.println(hz);
  } else if(c == 'W'){
    hardStop();
    Serial.println("ok W (일부러 멈춥니다 — 워치독이 500ms 안에 살려야 정상)");
    Serial.flush();
    BT->lastPhase = PH_TEST;
    for(;;) { }
  } else if(c == 'Q'){
    printCap();
  } else if(c == 'U'){
    // 바퀴부터 세운다. 드라이버도 재워 둔다 — 부트로더 동안 PWM 핀은 떠 있다.
    hardStop();
    digitalWrite(PIN_NSLEEP, LOW);
    Serial.println("ok U (시스템 부트로더 — stm32flash로 굽고 -g 0x08000000 으로 돌아옴)");
    Serial.flush();
    BT->enterBootloader = BOOTLDR_MAGIC;
    NVIC_SystemReset();
  } else if(c == '?'){
    Serial.print("st cur="); Serial.print(curVx); Serial.print(','); Serial.print(curVy);
    Serial.print(','); Serial.print(curW);
    Serial.print(" MAX_PWM="); Serial.print(MAX_PWM);
    Serial.print(" slew="); Serial.print(ACCEL); Serial.print('/'); Serial.print(DECEL);
    Serial.print(" strict="); Serial.print(strictCrc);
    Serial.print(" i2cerr=0");
    Serial.print(" POL="); for(uint8_t i = 0; i < 4; i++){ Serial.print(POL[i]); Serial.print(','); }
    Serial.print(" hz="); Serial.print(pwmHz);
    Serial.print(" clk="); Serial.print(clkHse ? "HSE" : "HSI");
    Serial.println();
    printBootReport();
  }
}

void setup(){
  backupOn();
  latchBootInfo();          // RCC_CSR을 지우기 전에 확정 — 출력보다 먼저

  // 드라이버는 PWM이 0으로 자리 잡을 때까지 재워 둔다(기판 풀다운과 같은 쪽).
  pinMode(PIN_NSLEEP, OUTPUT); digitalWrite(PIN_NSLEEP, LOW);
  pinMode(PIN_LED, OUTPUT);    pinMode(PIN_LEDF, OUTPUT);
  for(uint8_t i = 0; i < 4; i++) pinMode(PIN_NFAULT[i], INPUT_PULLUP);
  analogReadResolution(12);

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN | RCC_APB2ENR_TIM9EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM12EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN;
  (void)RCC->APB1ENR;

  pwmTimInit(TIM8, true); pwmTimInit(TIM9, false); pwmTimInit(TIM12, false);
  setHz(PWM_HZ_DEFAULT);
  for(TIM_TypeDef* t : PWM_TIMS) t->CR1 |= TIM_CR1_CEN;
  pinAF(GPIOC, 6, 3, false); pinAF(GPIOC, 7, 3, false);   // TIM8 CH1/2
  pinAF(GPIOC, 8, 3, false); pinAF(GPIOC, 9, 3, false);   // TIM8 CH3/4
  pinAF(GPIOA, 2, 3, false); pinAF(GPIOA, 3, 3, false);   // TIM9 CH1/2
  pinAF(GPIOB, 14, 9, false); pinAF(GPIOB, 15, 9, false); // TIM12 CH1/2

  // F4엔 AFIO 리맵이 없다 — PA15/PB3/PB4(JTAG)는 AF만 바꾸면 타이머가 된다. SWD(PA13/14)는 그대로.
  pinAF(GPIOA, 15, 1, true); pinAF(GPIOB, 3, 1, true);    // TIM2 (A)
  pinAF(GPIOB, 4, 2, true);  pinAF(GPIOB, 5, 2, true);    // TIM3 (B)
  pinAF(GPIOB, 6, 2, true);  pinAF(GPIOB, 7, 2, true);    // TIM4 (C)
  pinAF(GPIOA, 0, 2, true);  pinAF(GPIOA, 1, 2, true);    // TIM5 (D)
  for(uint8_t i = 0; i < 4; i++){ encInit(ENC_T[i]); encLast[i] = 0; encAcc[i] = 0; }

  Serial.begin(115200);     // USART1 PA9/PA10 (build_opt.h — 기본 Serial은 UART4=PA0/PA1이라 엔코더 D와 겹친다)
  hardStop();
  digitalWrite(PIN_NSLEEP, HIGH);   // 이제 깨운다 (tWAKE ~1ms, 그동안은 어차피 0)
  ensureBor();
  lastCmd = lastTick = lastLoop = millis();
  printBootReport();
  Serial.println("mecanum_stable v3.0-stm32 ready. proto: V vx vy w | S | T i d | P n | L .. | R a d | F hz | W | Q | U | ? (+*CRC)");
  printCap();
  IWatchdog.begin(250000);  // 250ms. LSI 오차로 실제 170~470ms
}

void loop(){
  IWatchdog.reload();
  unsigned long now = millis();
  if(now - lastLoop > 200) wdtNear++;   // 리셋까진 안 갔지만 아슬아슬했다
  lastLoop = now;

  encPoll();
  faultPoll();

  while(Serial.available()){
    char ch = Serial.read();
    if(ch == '\n' || ch == '\r'){
      if(blen > 0){ buf[blen] = 0; handleLine(buf); blen = 0; }
    } else if(blen < sizeof(buf) - 1){
      buf[blen++] = ch;
    } else {
      blen = 0;
    }
  }

  if(testing && (long)(millis() - testUntil) >= 0){
    encPoll();
    int32_t d = encAcc[testWheel] - testEnc0;
    hardStop();
    Serial.print("T auto-stop enc="); Serial.println(d);
  }

  now = millis();

  if(!testing){
    unsigned long silence = now - lastCmd;
    if(silence > HARD_TIMEOUT){
      if(curVx || curVy || curW){ hardStop(); Serial.println("deadman HARD"); }
    } else if(silence > CMD_TIMEOUT){
      tgtVx = tgtVy = tgtW = 0;
    }
  }

  if(!testing && (now - lastTick) >= TICK_MS){
    lastTick = now;
    int nx = approach(curVx, tgtVx);
    int ny = approach(curVy, tgtVy);
    int nw = approach(curW,  tgtW);
    if(nx != curVx || ny != curVy || nw != curW){
      curVx = nx; curVy = ny; curW = nw;
      applyCurrent();
    }
  }

  if(now - lastHb > 1000){
    lastHb = now;
    uint32_t va = vdda();
    uint32_t amp[4], total = 0;
    for(uint8_t i = 0; i < 4; i++){ amp[i] = adcMv(PIN_IPROPI[i], va) * IPROPI_MA_PER_V / 1000; total += amp[i]; }
    Serial.print("hb "); Serial.print(now);
    Serial.print(" rx="); Serial.print(rxOk);
    Serial.print(" bad="); Serial.print(rxBad);
    Serial.print(" i2c=0 wdt="); Serial.print(wdtNear);
    Serial.print(" v="); Serial.print(curVx); Serial.print(','); Serial.print(curVy); Serial.print(','); Serial.print(curW);
    Serial.print(" enc=");
    for(uint8_t i = 0; i < 4; i++){ Serial.print(encAcc[i]); if(i < 3) Serial.print(','); }
    Serial.print(" vin="); Serial.print(adcMv(PIN_VM, va) * VM_DIV_X100 / 100);
    Serial.print(" v5="); Serial.print(adcMv(PIN_V5, va) * V5_DIV_X100 / 100);
    Serial.print(" amp="); Serial.print(total);
    Serial.print(" ampw=");
    for(uint8_t i = 0; i < 4; i++){ Serial.print(amp[i]); if(i < 3) Serial.print(','); }
    Serial.print(" flt="); Serial.print(fltMask);
    Serial.print(" fltn="); Serial.println(fltCount);
    digitalToggle(PIN_LED);
  }

  BT->lastPhase = PH_IDLE;
}
