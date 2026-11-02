// mecanum_stm32 — 젯슨 시리얼 주행 펌웨어 v3 (STM32F405 보드). mecanum_stable v2.1(Uno+PCA9685)을 대체한다.
// 프로토콜은 v2.1과 같다 — 젯슨 motor_link.py를 한 줄도 안 고치고 붙는다. 바뀐 것은 보드 쪽뿐:
//
//  (1) PCA9685(I2C) 대신 타이머 PWM 직결. v2.1 워치독의 1순위 용의자였던 "I2C 버스 행"이
//      원리적으로 없다. 호환을 위해 i2c=/i2cerr= 칸은 남기되 항상 0이다.
//  (2) PWM 24~40000Hz, 기본 **20kHz**(명세서 v0.1: 10~20kHz — 가청 대역 밖). PCA9685 상한(1526Hz)이
//      없어 "소음↔속도" 맞바꿈이 끊긴다. ⚠ 젯슨이 첫 hb 뒤 base_tuning의 F(지금 700)를 되밀어 넣으니
//      이 보드에선 그 값을 20000으로 바꿔야 기본이 산다(젯슨 코드는 그대로).
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
//  (8) 하드웨어 E-stop(PB13). 버튼이 릴레이 코일을 끊어 VM을 **물리적으로** 자른다. 펌웨어는
//      눌리는 순간 인터럽트로 드라이버부터 재운다(접점이 전류 0에서 떨어지게 — 용착 방지).
//      **풀려도 다시 안 간다(래치)** — S를 받아야 풀린다. 젯슨이 V를 50Hz로 계속 보내므로 래치가 없으면
//      버튼을 놓는 순간 로봇이 다시 달린다. 소프트 정지(S·데드맨)는 그대로 보조.
//  (8a) 릴레이 감시(PC5 = 릴레이 뒤 VM, 100k/10k). 눌렸는데 VM이 남아 있으면 접점 용착(weld) — 전원을
//      껐다 켜기 전엔 안 풀린다. 코일이 붙었는데 VM이 없으면 rlyopen.
//  (9) 배터리 감시(PC4, 100k/10k). 1초 평균이 문턱 아래로 2초 → 감속 정지 후 V 무시.
//  (10) 바퀴별 속도 PID(엔코더). G로 kv>0을 준 바퀴만 폐루프 — 안 주면 Uno와 똑같은 개루프다.
//       V는 그대로 "−255..255 = 최고속 대비 비율"이라 젯슨은 안 바뀐다.
//
// 프로토콜 (라인 단위, '\n' 종료. 모든 라인에 선택적으로 "*HH" 체크섬):
//   V <vx> <vy> <w>   속도지령. vx=전진+, vy=우평행+, w=시계회전+. 각 -255..255. (무응답)
//   S                 즉시 정지 (슬루 무시, 하드)
//   T <i> <d>         모터 i(0=A/RR 1=B/RL 2=C/FR 3=D/FL) 방향 d(1/-1) 단독 1초. 끝에 엔코더 증분을 보고
//   P <n>             최대 PWM 스케일 (0..4095 척도 유지. 기본 2000)
//   L <a><b><c><d>    바퀴 극성 (각 1/-1)
//   R <accel> <decel> 슬루 스텝. 기본 6 / 12
//   F <hz>            PWM 주파수 (24~40000, 기본 20000)
//   B <ratio_x1000> <lo_mv>   배터리 분압비(기본 11000=11.0배)·저전압 문턱(기본 6400, 0=끔)
//   G <i> <kv> <ks> <kp> <ki> 바퀴 i(0..3, -1=전체) 속도 PID. 전부 ×1000. kv=0이면 그 바퀴 개루프
//   N <i> <cpr> <dia_um>      엔코더 CPR(4체배 후)·바퀴 지름 — 저장·보고만(rpm)
//   W                 워치독 자가진단(일부러 멈춤 → cause=WDT last=test 로 돌아와야 정상)
//   Q                 cap 한 줄 (보드계약 §6, proto=1 레거시 프로파일)
//   U                 정지 후 시스템 부트로더로 (stm32flash)
//   ?                 상태 + boot 보고
// hb: "hb <ms> rx= bad= i2c=0 wdt= v=<vx,vy,w>" 뒤에 enc= vin= vm= amp= ampw= flt= fltn=
//     spd= estop=(0 정상/1 눌림/2 풀렸지만 래치) lowbat= clf= rly=(1 닫힘/0 열림/W 용착/O 안 붙음) 를 덧붙인다
// 이벤트 줄: estop 1/0 · lowbat 1/0 · clfault i · weld 1 · rlyopen 1/0
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
#define PIN_VBAT   PC4    // 배터리(역접속 FET 뒤, E-stop 릴레이 앞) 100k/10k
#define PIN_ESTOP  PB13   // HIGH=릴레이 코일 붙음(모터 허용) / LOW=E-stop 눌림·버튼 회로 끊김 → VM 차단
#define PIN_VM     PC5    // 릴레이 **뒤** 모터 레일 VM, 100k/10k(배터리와 같은 비 — B의 ratio를 같이 쓴다)
#define PIN_LED    PB0    // 하트비트마다 깜빡
#define PIN_LEDF   PB1    // 드라이버 폴트
// 실측 보정 손잡이 — 저항 오차·IPROPI 편차는 여기서 맞춘다.
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
const uint16_t PWM_HZ_DEFAULT = 20000;   // 명세서 v0.1. 젯슨 base_tuning F도 20000으로
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

// E-stop: 눌림은 인터럽트로 즉시(디바운스 없음), 이벤트·풀림 판정은 20ms 디바운스.
// estop = 지금 눌림(디바운스 후), estopLatch = 눌렸던 적이 있고 아직 X 0으로 안 풀림.
bool estop = false, estopRaw = false, estopLatch = false;
volatile bool estopIsrHit = false;    // ISR이 드라이버를 재웠다 — 진짜 눌림인지 loop가 가린다
unsigned long estopEdge = 0;
const unsigned long ESTOP_DEBOUNCE = 20;

// 릴레이 감시: 100ms 표본이 3번 연속(300ms) 어긋나면 판정. 배터리 3V 아래(벤치·배터리 없음)는 판정 안 함.
uint32_t vmMv = 0;
bool weld = false, rlyOpen = false;   // weld는 전원을 껐다 켜야 풀린다
uint8_t rlyBadN = 0;
const uint32_t RLY_MIN_VIN = 3000;
const uint8_t RLY_HOLD_N = 3;
static inline bool driversAllowed(){ return !estop && !estopLatch && !weld && !estopIsrHit; }

// 배터리: 100ms마다 재서 10개 평균(1초). lo_mv 아래 2초 → 감속 정지. lo_mv+300 위로 풀림.
uint32_t batRatio = 11000;        // ×1000 = (100k+10k)/10k
uint32_t batLo = 6400;            // mV. 2S Li-ion 3.2V/셀. 0=감시 끔
uint32_t batBuf[10]; uint8_t batN = 0, batI = 0;
uint32_t batMv = 0;
bool lowbat = false;
unsigned long lastBat = 0, lowSince = 0;
const uint32_t BAT_HYST = 300;
const unsigned long BAT_HOLD = 2000;

// ---------------- 바퀴 속도 PID ----------------
// 단위: 속도 = 카운트/초(cps, 4체배 후), 듀티 = 0..4095(MAX_PWM 척도). G 값은 ×1000 고정소수점.
//   duty = sign(t)·ks + kv·t + kp·e + ki·∫e      (t = 목표 cps, e = t − 측정)
// V 호환: t = (바퀴 지령/255) · cps_full,  cps_full = (MAX_PWM − ks)/kv = 피드포워드만으로 꽉 차는 속도.
struct Wheel {
  int32_t kv = 0, ks = 0, kp = 0, ki = 0;   // ×1000. kv=0 → 개루프
  int32_t cpr = 0, diaUm = 0;               // N: 보고용
  float integ = 0, spd = 0;                 // ∫e, 측정 cps(이동평균)
  float hist[4] = {0, 0, 0, 0}; uint8_t hi = 0;
  uint16_t badMs = 0;
  int32_t duty = 0;
};
Wheel WH[4];
int wcmd[4] = {0, 0, 0, 0};        // 바퀴 지령(−255..255, 바퀴 전진=+, POL 적용 전) A..D
int32_t encPrev[4];
uint8_t clfMask = 0;               // 폐루프 폭주로 개루프로 떨어진 바퀴
unsigned long lastCtl = 0;
const unsigned long CTL_MS = 10;   // 100Hz
static inline bool closedLoop(uint8_t w){ return WH[w].kv > 0 && !(clfMask & (1u << w)); }
int32_t pidStep(Wheel& h, float tgt, float dt, int32_t maxPwm, bool& fault);   // ⚠ 직접 선언 — 안 하면 Arduino가 Wheel보다 위에 원형을 만든다

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
// 부호 있는 듀티(−MAX_PWM..MAX_PWM, 바퀴 전진=+)를 극성 적용해 내보낸다.
void setDuty(uint8_t w, int32_t d){
  d *= POL[w];
  uint32_t mag = (uint32_t)(d < 0 ? -d : d);
  if(mag > (uint32_t)MAX_PWM) mag = MAX_PWM;
  const MotorPwm& m = MOT[w];
  uint32_t ccr = mag * (m.t->ARR + 1) / 4095;            // → 이 타이머의 비교값
  setCcr(m.t, m.c1, d > 0 ? ccr : 0);
  setCcr(m.t, m.c2, d < 0 ? ccr : 0);
}

static inline int32_t openDuty(int s){
  if(s > 255) s = 255;
  if(s < -255) s = -255;
  return (int32_t)s * MAX_PWM / 255;                     // 0..MAX_PWM (PCA9685 척도)
}

void driveWheel(uint8_t w, int s){ setDuty(w, openDuty(s)); }

// 폐루프 바퀴는 control()이 100Hz로 내보낸다. 지령 0이면 여기서 바로 0 — 정지는 기다리지 않는다.
void outWheel(uint8_t w){
  if(!closedLoop(w)){ setDuty(w, openDuty(wcmd[w])); return; }
  if(wcmd[w] == 0){ WH[w].integ = 0; WH[w].badMs = 0; WH[w].duty = 0; setDuty(w, 0); }
}

// 메카넘 mixing. FL = vx-vy-w ; FR = vx+vy+w ; RL = vx+vy-w ; RR = vx-vy+w
void applyCurrent(){
  BT->lastPhase = PH_PWM;
  int FL = curVx - curVy - curW;
  int FR = curVx + curVy + curW;
  int RL = curVx + curVy - curW;
  int RR = curVx - curVy + curW;
  wcmd[0] = RR; wcmd[1] = RL; wcmd[2] = FR; wcmd[3] = FL;   // A B C D
  for(uint8_t w = 0; w < 4; w++) outWheel(w);
  BT->lastPhase = PH_IDLE;
}

void hardStop(){
  BT->lastPhase = PH_STOP;
  tgtVx = tgtVy = tgtW = 0;
  curVx = curVy = curW = 0;
  testing = false;
  applyCurrent();
}

// 한 바퀴 PID 한 걸음. 목표 0이면 적분·폭주 판정을 비우고 0(코스트) — 정지에서 떨지 않게.
// 반환: 듀티. 폭주(듀티 80%↑로 0.5초째 반대로 돌거나 목표의 10%도 못 낼 때) → fault=true.
int32_t pidStep(Wheel& h, float tgt, float dt, int32_t maxPwm, bool& fault){
  fault = false;
  if(tgt == 0){ h.integ = 0; h.badMs = 0; return 0; }
  float kv = h.kv / 1000.f, ks = h.ks / 1000.f, kp = h.kp / 1000.f, ki = h.ki / 1000.f;
  float e = tgt - h.spd;
  float u0 = (tgt > 0 ? ks : -ks) + kv * tgt + kp * e;
  if(ki > 0){                                       // 적분 포화 방지: 적분항만으로 ±maxPwm을 못 넘게
    float lim = maxPwm / ki;
    h.integ += e * dt;
    if(h.integ > lim) h.integ = lim;
    if(h.integ < -lim) h.integ = -lim;
  }
  float u = u0 + ki * h.integ;
  if(u > maxPwm) u = maxPwm;
  if(u < -maxPwm) u = -maxPwm;
  bool stuck = fabsf(u) > 0.8f * maxPwm && (h.spd * tgt < 0 || fabsf(h.spd) < 0.1f * fabsf(tgt));
  h.badMs = stuck ? (uint16_t)min(60000.f, h.badMs + dt * 1000) : 0;
  if(h.badMs > 500) fault = true;
  return (int32_t)u;
}

// 100Hz: 엔코더 차분 → cps(4샘플 평균) → 폐루프 바퀴 듀티.
void control(unsigned long now){
  float dt = (now - lastCtl) / 1000.f;
  lastCtl = now;
  if(dt <= 0 || dt > 0.1f) dt = CTL_MS / 1000.f;   // 첫 바퀴·긴 막힘은 기본 주기로
  for(uint8_t w = 0; w < 4; w++){
    Wheel& h = WH[w];
    int32_t d = encAcc[w] - encPrev[w]; encPrev[w] = encAcc[w];
    h.hist[h.hi] = d / dt; h.hi = (h.hi + 1) & 3;
    h.spd = (h.hist[0] + h.hist[1] + h.hist[2] + h.hist[3]) / 4;
    if(!closedLoop(w) || testing || !driversAllowed()) continue;
    float cpsFull = (MAX_PWM - h.ks / 1000.f) / (h.kv / 1000.f);
    if(cpsFull <= 0){ h.duty = 0; setDuty(w, 0); continue; }   // ks ≥ MAX_PWM — 설정이 틀렸다
    bool fault;
    h.duty = pidStep(h, wcmd[w] / 255.f * cpsFull, dt, MAX_PWM, fault);
    if(fault){
      clfMask |= 1u << w;
      Serial.print("clfault "); Serial.println(w);
      outWheel(w);                                   // 개루프로 떨어진다
    } else setDuty(w, h.duty);
  }
}

// ★ PB13 하강 에지 = 버튼이 코일을 끊은 순간. 릴레이 코일엔 역기전력 다이오드가 있어 접점은 수 ms 뒤에야
//   떨어진다. 그 사이 드라이버를 먼저 재우면 접점이 **전류 0에서** 열린다 — 아크가 안 나야 접점이 안 붙는다
//   (용착 방지). 디바운스는 여기선 안 한다: 잡음으로 한 번 서는 건 loop가 20ms 뒤 되살린다(estopPoll).
//   레지스터 직접 쓰기만 — HAL·Serial은 ISR에서 안 부른다.
void estopIsr(){
  GPIOB->BSRR = 1u << (12 + 16);                       // PB12 nSLEEP LOW → 출력 Hi-Z
  TIM8->CCR1 = 0; TIM8->CCR2 = 0; TIM8->CCR3 = 0; TIM8->CCR4 = 0;
  TIM9->CCR1 = 0; TIM9->CCR2 = 0;
  TIM12->CCR1 = 0; TIM12->CCR2 = 0;
  estopIsrHit = true;
}

// 드라이버는 허용될 때만 깨어 있다 — nSLEEP는 이 한 곳에서만 쥔다.
static inline void updSleep(){ digitalWrite(PIN_NSLEEP, driversAllowed() ? HIGH : LOW); }

// E-stop: 20ms 디바운스로 이벤트. 눌림 → 래치. 풀림 → 보고만(래치 유지, S가 와야 깨운다).
void estopPoll(unsigned long now){
  bool raw = digitalRead(PIN_ESTOP) == LOW;
  if(raw != estopRaw){ estopRaw = raw; estopEdge = now; }
  bool stable = now - estopEdge >= ESTOP_DEBOUNCE;
  if(stable && raw != estop){
    estop = raw;
    if(estop){ estopLatch = true; hardStop(); }
    Serial.println(estop ? "estop 1" : "estop 0");
  }
  if(estopIsrHit && stable){
    // 진짜 눌림이면 위에서 래치가 걸렸다. 안 눌렸으면 잡음 — 0에서 다시 슬루로 출발(지령은 V가 다시 채운다).
    if(!estop) hardStop();
    estopIsrHit = false;
  }
  updSleep();
}

// 릴레이 감시(100ms마다 batPoll이 부른다). 코일 상태 = 버튼 상태(PB13이 코일 전압을 본다).
//   눌림(코일 꺼짐)인데 VM > 배터리/2 → 접점 용착. ⚠ 하드웨어 E-stop이 더는 안 듣는다는 뜻이라
//   S로 못 풀게 한다 — 사람이 릴레이를 보고 전원을 껐다 켜야 한다.
//   ⚠ VM에 블리더가 없으면 드라이버가 잔 뒤 레일 커패시터(≈40µF)가 안 빠져 오검출한다 — 기판 몫.
//   코일 붙음인데 VM < 배터리/2(배터리가 문턱 위일 때) → rlyopen(접점이 안 붙음). VM이 돌아오면 풀린다.
void relayCheck(){
  bool vmOn = vmMv * 2 > batMv;
  if(batMv < RLY_MIN_VIN || batN < 10){ rlyBadN = 0; return; }
  bool bad = estop ? vmOn : (!vmOn && batMv > batLo);
  if(!bad){
    rlyBadN = 0;
    if(rlyOpen){ rlyOpen = false; Serial.println("rlyopen 0"); }
    return;
  }
  if(rlyBadN < 255) rlyBadN++;
  if(rlyBadN < RLY_HOLD_N) return;
  if(estop && !weld){ weld = true; hardStop(); updSleep(); Serial.println("weld 1"); }
  if(!estop && !rlyOpen){ rlyOpen = true; Serial.println("rlyopen 1"); }
}

static char rlyState(){ return weld ? 'W' : rlyOpen ? 'O' : (vmMv * 2 > batMv ? '1' : '0'); }
static uint8_t estopState(){ return estop ? 1 : estopLatch ? 2 : 0; }

// 배터리: 100ms 표본 10개 평균. 문턱 아래 2초 연속 → lowbat(목표 0 → 슬루 감속). 문턱+300 위 → 해제.
void batPoll(unsigned long now){
  if(now - lastBat < 100) return;
  lastBat = now;
  uint32_t va = vdda();
  batBuf[batI] = adcMv(PIN_VBAT, va) * batRatio / 1000;
  vmMv = adcMv(PIN_VM, va) * batRatio / 1000;        // 평균 안 함 — relayCheck의 3연속이 거름망
  batI = (batI + 1) % 10;
  if(batN < 10) batN++;
  uint32_t sum = 0;
  for(uint8_t i = 0; i < batN; i++) sum += batBuf[i];
  batMv = sum / batN;
  relayCheck();
  if(!batLo){                                        // 감시 끔
    if(lowbat){ lowbat = false; Serial.println("lowbat 0"); }
    lowSince = now; return;
  }
  if(batN < 10){ lowSince = now; return; }           // 평균이 찰 때까지는 판정 안 함
  if(!lowbat){
    if(batMv >= batLo) lowSince = now;
    else if(now - lowSince >= BAT_HOLD){ lowbat = true; tgtVx = tgtVy = tgtW = 0; Serial.println("lowbat 1"); }
  } else if(batMv > batLo + BAT_HYST){ lowbat = false; lowSince = now; Serial.println("lowbat 0"); }
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
  Serial.print("cap proto=1 fw=3.2.0 board=stm32f405 units=0 closed_loop=1 enc=4 calib=0 vin=1 amp=1 estop_hw=1 pwm_hz=");
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
    if(!driversAllowed() || lowbat) return;   // 50Hz로 오는 지령이라 nak를 쏟지 않는다 — hb의 estop=/rly=/lowbat=로 보인다
    int vx = nextInt(p), vy = nextInt(p), w = nextInt(p);
    if(vx > 255) vx = 255; if(vx < -255) vx = -255;
    if(vy > 255) vy = 255; if(vy < -255) vy = -255;
    if(w  > 255) w  = 255; if(w  < -255) w  = -255;
    tgtVx = vx; tgtVy = vy; tgtW = w;
    testing = false;
  } else if(c == 'S'){
    hardStop();                    // S는 정지일 뿐 — E-stop 래치는 안 푼다. 젯슨이 S를 자동으로 많이 보낸다
    Serial.println("ok S");        // (이동 끝·포트 재연결·넛지 끝) → S로 풀면 자동 시퀀스가 사람 확인 없이 재출발한다
  } else if(c == 'X'){
    // 보드 계약 §4: X 1 = 소프트 래치 걸기, X 0 = 해제(사람 확인 — 대시보드/조작대 [비상정지 해제] 버튼만 보낸다).
    // 버튼이 눌린 채면 못 풀고, 용착(weld)은 전원을 껐다 켜야 풀린다. 인자가 없으면 상태만 답한다.
    while(*p == ' ') p++;
    if(*p == '1'){ estopLatch = true; hardStop(); updSleep(); Serial.println("ok X 1"); }
    else if(*p == '0'){
      if(weld)        Serial.println("nak weld");
      else if(estop)  Serial.println("nak estop held");
      else { estopLatch = false; updSleep(); Serial.println("ok X 0"); }
    } else { Serial.print("ok X "); Serial.println(estopState()); }
  } else if(c == 'T'){
    int i = nextInt(p), d = nextInt(p);
    if(!driversAllowed()){ Serial.println("nak estop"); return; }
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
  } else if(c == 'B'){
    int r = nextInt(p), lo = nextInt(p);
    if(r > 0) batRatio = r;
    if(lo >= 0) batLo = lo;
    batN = 0;                       // 비율이 바뀌면 평균을 새로
    Serial.print("ok B "); Serial.print(batRatio); Serial.print(' '); Serial.println(batLo);
  } else if(c == 'G'){
    int i = nextInt(p); int32_t kv = nextInt(p), ks = nextInt(p), kp = nextInt(p), ki = nextInt(p);
    if(i < -1 || i > 3 || kv < 0 || ks < 0 || kp < 0 || ki < 0){ Serial.println("nak arg"); return; }
    for(uint8_t w = 0; w < 4; w++){
      if(i >= 0 && w != i) continue;
      WH[w].kv = kv; WH[w].ks = ks; WH[w].kp = kp; WH[w].ki = ki;
      WH[w].integ = 0; WH[w].badMs = 0; clfMask &= ~(1u << w);   // 새 게인 = 폭주 판정도 새로
      outWheel(w);
    }
    Serial.print("ok G "); Serial.print(i); Serial.print(' '); Serial.print(kv); Serial.print(' ');
    Serial.print(ks); Serial.print(' '); Serial.print(kp); Serial.print(' '); Serial.println(ki);
  } else if(c == 'N'){
    int i = nextInt(p); int32_t cpr = nextInt(p), dia = nextInt(p);
    if(i < -1 || i > 3 || cpr < 0 || dia < 0){ Serial.println("nak arg"); return; }
    for(uint8_t w = 0; w < 4; w++) if(i < 0 || w == i){ WH[w].cpr = cpr; WH[w].diaUm = dia; }
    Serial.print("ok N "); Serial.print(i); Serial.print(' '); Serial.print(cpr); Serial.print(' '); Serial.println(dia);
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
    Serial.print(" estop="); Serial.print(estopState());
    Serial.print(" vm="); Serial.print(vmMv);
    Serial.print(" rly="); Serial.print(rlyState());
    Serial.print(" bat="); Serial.print(batMv); Serial.print('/'); Serial.print(batLo);
    Serial.print(" ratio="); Serial.print(batRatio);
    Serial.print(" lowbat="); Serial.print(lowbat);
    Serial.println();
    for(uint8_t w = 0; w < 4; w++){
      Serial.print("wh "); Serial.print(w);
      Serial.print(" G="); Serial.print(WH[w].kv); Serial.print(','); Serial.print(WH[w].ks); Serial.print(',');
      Serial.print(WH[w].kp); Serial.print(','); Serial.print(WH[w].ki);
      Serial.print(closedLoop(w) ? " loop=closed" : (clfMask & (1u << w)) ? " loop=FAULT" : " loop=open");
      Serial.print(" spd="); Serial.print((int32_t)WH[w].spd);
      if(WH[w].cpr){ Serial.print(" rpm="); Serial.print((int32_t)(WH[w].spd * 60 / WH[w].cpr)); }
      Serial.print(" N="); Serial.print(WH[w].cpr); Serial.print(','); Serial.println(WH[w].diaUm);
    }
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
  pinMode(PIN_ESTOP, INPUT);    // 기판 100k 풀다운 — 선이 빠지면 LOW = E-stop 쪽으로 안전
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
  estop = estopRaw = estopLatch = (digitalRead(PIN_ESTOP) == LOW);   // 켜질 때 이미 눌려 있으면 래치·재운 채로
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), estopIsr, FALLING);
  updSleep();               // 허용이면 이제 깨운다 (tWAKE ~1ms, 그동안은 어차피 0)
  ensureBor();
  lastCmd = lastTick = lastLoop = millis();
  printBootReport();
  Serial.println("mecanum_stable v3.2-stm32 ready. proto: V vx vy w | S | T i d | P n | L .. | R a d | F hz | B r lo | G i kv ks kp ki | N i cpr dia | W | Q | U | ? (+*CRC)");
  printCap();
  if(estop) Serial.println("estop 1");
  for(uint8_t i = 0; i < 4; i++) encPrev[i] = encAcc[i];
  lastCtl = lastBat = lowSince = millis();
  IWatchdog.begin(250000);  // 250ms. LSI 오차로 실제 170~470ms
}

void loop(){
  IWatchdog.reload();
  unsigned long now = millis();
  if(now - lastLoop > 200) wdtNear++;   // 리셋까진 안 갔지만 아슬아슬했다
  lastLoop = now;

  encPoll();
  faultPoll();
  estopPoll(now);
  batPoll(now);

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

  if(now - lastCtl >= CTL_MS) control(now);

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
    Serial.print(" vin="); Serial.print(batMv);
    Serial.print(" vm="); Serial.print(vmMv);
    Serial.print(" amp="); Serial.print(total);
    Serial.print(" ampw=");
    for(uint8_t i = 0; i < 4; i++){ Serial.print(amp[i]); if(i < 3) Serial.print(','); }
    Serial.print(" flt="); Serial.print(fltMask);
    Serial.print(" fltn="); Serial.print(fltCount);
    Serial.print(" spd=");
    for(uint8_t i = 0; i < 4; i++){ Serial.print((int32_t)WH[i].spd); if(i < 3) Serial.print(','); }
    Serial.print(" estop="); Serial.print(estopState());
    Serial.print(" lowbat="); Serial.print(lowbat);
    Serial.print(" clf="); Serial.print(clfMask);
    Serial.print(" rly="); Serial.println(rlyState());
    digitalToggle(PIN_LED);
  }

  BT->lastPhase = PH_IDLE;
}
