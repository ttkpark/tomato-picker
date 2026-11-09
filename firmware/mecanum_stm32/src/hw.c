// hw.c — STM32F405 레지스터 직결(HAL 없음). core가 부르는 hw_if.h와, 부팅·폴트·인터럽트 처리.
//
// 인터럽트 우선순위(숫자가 작을수록 먼저):
//   0 EXTI15_10  E-stop — 무엇이 돌고 있든 드라이버를 먼저 재운다
//   1 TIM6       1kHz 제어(core_tick_1khz)
//   2 SysTick    ms
//   4 DMA2_S7    직렬 송신
// main의 크리티컬 구역은 BASEPRI로 1 이하를 막는다 — E-stop은 절대 막지 않는다.
#include "board.h"
#include "cfgstore.h"
#include "hw_if.h"
#include "stm32f4xx.h"

#include <string.h>

// ---------------- 리셋 흔적 (BKPSRAM: 리셋엔 살아남고 전원이 나가면 깨진다) ----------------
#define BOOT_MAGIC 0xB07E5732u
#define BL_MAGIC 0x5AFEB007u
#define FAULT_MAGIC 0xFA017ED0u
typedef struct {
    uint32_t magic, boot_count, enter_bl, phase, prev_phase;
    uint32_t mv_id; int32_t mv_got, mv_target; uint32_t mv_chk;
    uint32_t fault_magic, f_pc, f_lr, f_cfsr, f_hfsr, f_addr;
} bkp_t;
#define BKP ((volatile bkp_t *)BKPSRAM_BASE)

static void backup_on(void) {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; (void)RCC->APB1ENR;
    PWR->CR |= PWR_CR_DBP;
    RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN; (void)RCC->AHB1ENR;
}

// startup이 .data/.bss를 채우기 **전에** 부른다 — 여기선 전역 변수를 쓰면 안 된다.
// U가 남긴 표시가 있으면 아무것도 켜기 전에(클럭·타이머·IWDG 전부 리셋 상태) 시스템 부트로더로 뛴다.
void SystemInit(void) {
    SCB->CPACR |= (3u << 20) | (3u << 22);           // FPU
    backup_on();
    if (BKP->enter_bl == BL_MAGIC) {
        BKP->enter_bl = 0;
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; (void)RCC->APB2ENR;
        SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0;   // 0번지 = 시스템 메모리
        const volatile uint32_t *sys = (const volatile uint32_t *)0x1FFF0000u;
        __set_MSP(sys[0]);
        ((void (*)(void))sys[1])();
    }
}

// ---------------- 공용 ----------------
uint32_t SystemCoreClock = 16000000u;               // CMSIS 기호. clock_init이 168MHz로 바꾼다
void _init(void) {}                                  // -nostartfiles: newlib __libc_init_array가 찾는 빈 고리
void _fini(void) {}
static volatile uint32_t ms;
static bool clk_hse;

static void gpio_mode(GPIO_TypeDef *g, uint32_t n, uint32_t mode, uint32_t pull, uint32_t af) {
    if (mode == 2u) g->AFR[n >> 3] = (g->AFR[n >> 3] & ~(0xFu << (4u * (n & 7u)))) | (af << (4u * (n & 7u)));
    g->PUPDR = (g->PUPDR & ~(3u << (2u * n))) | (pull << (2u * n));
    g->OSPEEDR = (g->OSPEEDR & ~(3u << (2u * n))) | (2u << (2u * n));
    g->OTYPER &= ~(1u << n);
    g->MODER = (g->MODER & ~(3u << (2u * n))) | (mode << (2u * n));   // 모드는 마지막 — AF가 먼저 서야 튀지 않는다
}
enum { M_IN = 0, M_OUT = 1, M_AF = 2, M_AN = 3 };
enum { P_NO = 0, P_UP = 1, P_DOWN = 2 };

static inline void nsleep_low(void) { GPIOB->BSRR = 1u << (12 + 16); }
static inline void pwm_regs_zero(void) {
    TIM8->CCR1 = 0; TIM8->CCR2 = 0; TIM8->CCR3 = 0; TIM8->CCR4 = 0;
    TIM9->CCR1 = 0; TIM9->CCR2 = 0; TIM12->CCR1 = 0; TIM12->CCR2 = 0;
}

// ---------------- 클럭: 8MHz HSE → 168MHz, 크리스털이 안 뜨면 HSI로 같은 168MHz ----------------
static bool pll_on(bool hse) {
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {}
    // VCO 입력 2MHz: HSE 8/4, HSI 16/8. ×168 = 336MHz, /2 = 168MHz, /7 = 48MHz
    RCC->PLLCFGR = (1u << 29) |                       // 예약 비트 — 리셋값(1)을 지킨다
                   (hse ? (4u | RCC_PLLCFGR_PLLSRC_HSE) : 8u) | (168u << RCC_PLLCFGR_PLLN_Pos) |
                   (0u << RCC_PLLCFGR_PLLP_Pos) | (7u << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CR |= RCC_CR_PLLON;
    for (uint32_t t = 0; !(RCC->CR & RCC_CR_PLLRDY); t++) if (t > 2000000u) return false;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    for (uint32_t t = 0; (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL; t++) if (t > 2000000u) return false;
    return true;
}

static void clock_init(void) {
    PWR->CR |= PWR_CR_VOS;                                   // 스케일 1(168MHz)
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CR |= RCC_CR_HSEON;
    for (uint32_t t = 0; t < 1000000u; t++) if (RCC->CR & RCC_CR_HSERDY) { clk_hse = true; break; }
    if (clk_hse && pll_on(true)) {
        RCC->CR |= RCC_CR_CSSON;                             // 크리스털이 멈추면 NMI → HSI로 되살린다
    } else {
        clk_hse = false;
        RCC->CR &= ~RCC_CR_HSEON;
        pll_on(false);
    }
    SystemCoreClock = SYSCLK_HZ;
}

void NMI_Handler(void) {
    if (RCC->CIR & RCC_CIR_CSSF) {
        RCC->CIR |= RCC_CIR_CSSC;
        // CSS: 하드웨어가 이미 HSI로 바꾸고 PLL을 껐다. 같은 168MHz를 HSI로 다시 만든다 — 보율·PWM 주파수가 그대로다.
        clk_hse = false;
        pll_on(false);
        TIM8->BDTR |= TIM_BDTR_MOE;                          // CSS 사건은 TIM8 브레이크로도 가서 출력을 끈다
    }
}

void SysTick_Handler(void) { ms++; }
uint32_t hw_ms(void) { return ms; }
bool hw_clk_hse(void) { return clk_hse; }

// ---------------- 크리티컬 구역 ----------------
#define PRIO_BITS 4u
void hw_crit_enter(void) { __set_BASEPRI(1u << (8u - PRIO_BITS)); __DSB(); __ISB(); }
void hw_crit_exit(void) { __set_BASEPRI(0); }

// ---------------- PWM ----------------
typedef struct { TIM_TypeDef *t; volatile uint32_t *c1, *c2; } mot_t;
static mot_t MOT[NW];
static TIM_TypeDef *const PWM_T[3] = { TIM8, TIM9, TIM12 };
static volatile bool estop_hit;   // E-stop 인터럽트가 출력을 끊었다 — 지워질 때까지 출력·깨우기 거부

static uint32_t tim_clk(TIM_TypeDef *t) { return (t == TIM8 || t == TIM9) ? 2u * PCLK2_HZ : 2u * PCLK1_HZ; }

uint32_t hw_pwm_set_hz(uint32_t hz) {
    if (hz < 24u) hz = 24u;
    if (hz > 40000u) hz = 40000u;
    uint32_t got = hz;
    for (int i = 0; i < 3; i++) {
        TIM_TypeDef *t = PWM_T[i];
        uint32_t ticks = tim_clk(t) / hz, psc = ticks / 65536u;
        uint32_t arr = ticks / (psc + 1u) - 1u;
        t->CR1 &= ~TIM_CR1_CEN;
        t->PSC = psc;
        t->ARR = arr;
        t->CCR1 = 0; t->CCR2 = 0;
        if (t == TIM8) { t->CCR3 = 0; t->CCR4 = 0; }
        t->EGR = TIM_EGR_UG;
        t->CR1 |= TIM_CR1_CEN;
        if (t == TIM9) got = tim_clk(t) / ((psc + 1u) * (arr + 1u));
    }
    return got;
}

void hw_pwm_out(int w, int32_t d) {
    if (w < 0 || w >= NW) return;
    if (estop_hit) d = 0;
    uint32_t mag = (uint32_t)(d < 0 ? -d : d);
    if (mag > 4095u) mag = 4095u;
    uint32_t ccr = mag * (MOT[w].t->ARR + 1u) / 4095u;
    // 한쪽을 먼저 0으로 — 방향이 바뀌는 순간 두 입력이 함께 서면 브레이크(짧은 단락 전류)가 된다
    if (d > 0) { *MOT[w].c2 = 0; *MOT[w].c1 = ccr; }
    else if (d < 0) { *MOT[w].c1 = 0; *MOT[w].c2 = ccr; }
    else { *MOT[w].c1 = 0; *MOT[w].c2 = 0; }
    if (estop_hit) { *MOT[w].c1 = 0; *MOT[w].c2 = 0; }   // 위 검사와 쓰기 사이에 E-stop이 끼었으면 되돌린다
}
void hw_pwm_zero_all(void) { pwm_regs_zero(); }

void hw_drv_enable(bool en) {
    if (en && !estop_hit) {
        GPIOB->BSRR = 1u << 12;                      // tWAKE ≈1ms 동안은 어차피 출력이 0
        if (estop_hit) nsleep_low();                 // 검사와 쓰기 사이에 E-stop 인터럽트가 끼었다 — 그쪽이 이긴다
    } else nsleep_low();
}

static void pwm_tim_init(TIM_TypeDef *t, bool four) {
    t->CR1 = TIM_CR1_ARPE;
    t->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    if (four) t->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    t->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | (four ? (TIM_CCER_CC3E | TIM_CCER_CC4E) : 0u);
    if (t == TIM8) t->BDTR = TIM_BDTR_MOE;           // 고급 타이머는 MOE가 있어야 출력이 나간다
}

// ---------------- 엔코더 ----------------
static TIM_TypeDef *const ENC_T[NW] = { TIM2, TIM3, TIM4, TIM5 };
static int32_t enc_acc[NW];
static uint16_t enc_last16[NW];

static void enc_init(TIM_TypeDef *t) {
    t->SMCR = 3u;                                    // 엔코더 모드 3 = 두 상 양 에지 = 4체배
    // 입력 필터 fDTS/32·8표본(≈3µs @84MHz) — 모터 스위칭 가시를 하드웨어에서 거른다. 정상 에지 간격은 수십 µs
    t->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | (0xFu << TIM_CCMR1_IC1F_Pos) | (0xFu << TIM_CCMR1_IC2F_Pos);
    t->CCER = 0;
    t->ARR = 0xFFFFFFFFu;                            // 16비트 타이머엔 0xFFFF로 들어간다
    t->CNT = 0;
    t->CR1 = TIM_CR1_CEN;
}
static void enc_update(void) {                      // 1kHz — 16비트 카운터는 차분을 이어 붙인다(1ms에 32767 못 넘는다)
    for (int i = 0; i < NW; i++) {
        uint32_t c = ENC_T[i]->CNT;
        if (i == 1 || i == 2) { enc_acc[i] += (int16_t)(uint16_t)((uint16_t)c - enc_last16[i]); enc_last16[i] = (uint16_t)c; }
        else enc_acc[i] = (int32_t)c;
    }
}
int32_t hw_enc_raw(int w) { return (w >= 0 && w < NW) ? enc_acc[w] : 0; }

uint8_t hw_nfault_mask(void) {
    uint32_t c = GPIOC->IDR, d = GPIOD->IDR;
    return (uint8_t)((!(c & (1u << 10)) ? 1u : 0u) | (!(c & (1u << 11)) ? 2u : 0u) |
                     (!(c & (1u << 12)) ? 4u : 0u) | (!(d & (1u << 2)) ? 8u : 0u));
}

// ---------------- E-stop ----------------
bool hw_estop_ok(void) { return (GPIOB->IDR & (1u << 13)) != 0; }
bool hw_estop_isr_hit(void) { return estop_hit; }
void hw_estop_isr_clear(void) { if (hw_estop_ok()) estop_hit = false; }

// ★ PB13 하강 = 버튼이 코일을 끊은 순간. 코일 다이오드 때문에 접점은 수 ms 늦게 떨어진다 — 그 사이 드라이버를
//   먼저 재워 접점이 전류 0에서 열리게 한다(아크가 없어야 접점이 안 붙는다). 디바운스 없이, 레지스터만.
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR13) {
        EXTI->PR = EXTI_PR_PR13;
        nsleep_low();
        pwm_regs_zero();
        estop_hit = true;
    }
}

// ---------------- ADC: 8채널 연속 스캔 → DMA 순환 → 1kHz 누적 ----------------
enum { AD_IA, AD_IB, AD_IC, AD_ID, AD_VBAT, AD_VM, AD_V5, AD_VREF, AD_N };
static volatile uint16_t adc_buf[AD_N];
static uint32_t adc_sum[AD_N], adc_cnt;
static uint16_t adc_v5_min = 0xFFFF;
#define VREFINT_CAL (*(const volatile uint16_t *)0x1FFF7A2Au)   // 30°C·VDDA 3.3V에서 잰 공장값

static void adc_init(void) {
    ADC->CCR = ADC_CCR_ADCPRE_0 | ADC_CCR_TSVREFE;   // PCLK2/4 = 21MHz, VREFINT 켬
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->SMPR1 = 0x07FFFFFFu;                       // ch10~18: 480사이클(23µs) — 100k/10k 분압의 9kΩ 원천을 채운다
    ADC1->SMPR2 = 0x3FFFFFFFu;                       // ch0~9
    ADC1->SQR3 = (10u << 0) | (11u << 5) | (12u << 10) | (13u << 15) | (14u << 20) | (15u << 25);
    ADC1->SQR2 = (8u << 0) | (17u << 5);
    ADC1->SQR1 = (AD_N - 1u) << ADC_SQR1_L_Pos;
    DMA2_Stream0->CR = 0;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}
    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_buf;
    DMA2_Stream0->NDTR = AD_N;
    DMA2_Stream0->CR = (0u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC |
                       DMA_SxCR_CIRC | DMA_SxCR_EN;
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
    for (volatile int i = 0; i < 1000; i++) {}       // tSTAB
    ADC1->CR2 |= ADC_CR2_SWSTART;
}
static void adc_accumulate(void) {
    for (int i = 0; i < AD_N; i++) adc_sum[i] += adc_buf[i];
    if (adc_buf[AD_V5] < adc_v5_min) adc_v5_min = adc_buf[AD_V5];
    adc_cnt++;
}
void hw_adc_take(adc_t *o) {
    uint32_t s[AD_N], n;
    uint16_t v5min;
    hw_crit_enter();
    memcpy(s, adc_sum, sizeof s); n = adc_cnt; v5min = adc_v5_min;
    memset(adc_sum, 0, sizeof adc_sum); adc_cnt = 0; adc_v5_min = 0xFFFF;
    hw_crit_exit();
    memset(o, 0, sizeof *o);
    if (!n || !s[AD_VREF]) return;
    uint32_t vref_avg = s[AD_VREF] / n;
    uint32_t vdda = 3300u * VREFINT_CAL / (vref_avg ? vref_avg : 1u);
    if (vdda < 2400u || vdda > 3600u) vdda = 3300u;  // 보정값이 엉뚱하면 공칭값
    o->valid = true;
    o->vdda_mv = vdda;
#define PIN_MV(x) ((uint32_t)((uint64_t)(x) * vdda / (4095u * n)))
    for (int i = 0; i < NW; i++) o->ipropi_mv[i] = PIN_MV(s[AD_IA + i]);
    o->vbat_pin_mv = PIN_MV(s[AD_VBAT]);
    o->vm_pin_mv = PIN_MV(s[AD_VM]);
    o->v5_pin_mv = PIN_MV(s[AD_V5]);
    o->v5_min_pin_mv = (uint32_t)v5min * vdda / 4095u;
}

// ---------------- 직렬: USART1, 수신 DMA 순환 / 송신 DMA 고리 ----------------
#define RXN 512u
#define TXN 2048u
static volatile uint8_t rx_dma[RXN];
static uint32_t rx_rd;
static uint8_t tx_ring[TXN];
static volatile uint32_t tx_head, tx_tail, tx_busy_len;
static uint32_t tx_drops;

static void tx_kick(void) {                          // DMA가 놀고 있으면 이어 붙은 구간을 보낸다
    if (tx_busy_len || tx_head == tx_tail) return;
    uint32_t len = (tx_head > tx_tail ? tx_head : TXN) - tx_tail;
    tx_busy_len = len;
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream7->CR & DMA_SxCR_EN) {}
    DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;
    DMA2_Stream7->M0AR = (uint32_t)&tx_ring[tx_tail];
    DMA2_Stream7->NDTR = len;
    DMA2_Stream7->CR |= DMA_SxCR_EN;
}
void DMA2_Stream7_IRQHandler(void) {
    if (DMA2->HISR & (DMA_HISR_TCIF7 | DMA_HISR_TEIF7)) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CTEIF7;
        tx_tail = (tx_tail + tx_busy_len) % TXN;
        tx_busy_len = 0;
        tx_kick();
    }
}
bool hw_tx(const char *s, uint32_t n) {
    NVIC_DisableIRQ(DMA2_Stream7_IRQn);
    uint32_t used = (tx_head + TXN - tx_tail) % TXN;
    bool ok = n < TXN - 1u - used;
    if (ok) {
        for (uint32_t i = 0; i < n; i++) { tx_ring[tx_head] = (uint8_t)s[i]; tx_head = (tx_head + 1u) % TXN; }
        tx_kick();
    } else tx_drops++;                               // 막지 않는다 — 줄째 버리고 센다(hb txd=)
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    return ok;
}
uint32_t hw_tx_drops(void) { return tx_drops; }
static void tx_flush(uint32_t timeout_ms) {          // 리셋 직전에만
    uint32_t t0 = ms;
    while ((tx_head != tx_tail || tx_busy_len) && ms - t0 < timeout_ms) {}
    while (!(USART1->SR & USART_SR_TC) && ms - t0 < timeout_ms + 5u) {}
}

int hw_rx(void) {
    if (USART1->SR & USART_SR_ORE) { (void)USART1->DR; }   // 넘침 플래그만 지운다(DMA는 계속 받는다)
    uint32_t wr = RXN - DMA2_Stream2->NDTR;
    if (wr >= RXN) wr = 0;
    if (rx_rd == wr) return -1;
    int c = rx_dma[rx_rd];
    rx_rd = (rx_rd + 1u) % RXN;
    return c;
}

static void uart_init(void) {
    USART1->CR1 = 0;
    // 84MHz / (16 × 45.5625) = 115226 baud (0.02%)
    USART1->BRR = (45u << 4) | 9u;
    USART1->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
    DMA2_Stream2->CR = 0;
    while (DMA2_Stream2->CR & DMA_SxCR_EN) {}
    DMA2_Stream2->PAR = (uint32_t)&USART1->DR;
    DMA2_Stream2->M0AR = (uint32_t)rx_dma;
    DMA2_Stream2->NDTR = RXN;
    DMA2_Stream2->CR = (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;
    DMA2_Stream7->CR = 0;
    while (DMA2_Stream7->CR & DMA_SxCR_EN) {}
    DMA2_Stream7->PAR = (uint32_t)&USART1->DR;
    DMA2_Stream7->CR = (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

// ---------------- 워치독 ----------------
static void iwdg_set(uint32_t pr, uint32_t rlr) {
    IWDG->KR = 0x5555u;
    while (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) {}
    IWDG->PR = pr;
    IWDG->RLR = rlr;
    while (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) {}
    IWDG->KR = 0xAAAAu;
}
static void iwdg_start(void) {
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;   // 디버거로 멈춰 둔 동안은 안 문다
    IWDG->KR = 0xCCCCu;
    iwdg_set(3u, 249u);                               // LSI 32kHz /32 → 250ms (LSI 오차로 실제 ≈170~470ms)
}
void hw_wdt_kick(void) { IWDG->KR = 0xAAAAu; }

// ---------------- 리셋 흔적 ----------------
void hw_phase(uint8_t ph) { BKP->phase = ph; }
static uint32_t mv_chk(uint32_t id, int32_t t, int32_t g) {
    return id ^ (uint32_t)t * 2654435761u ^ (uint32_t)g * 40503u ^ 0xA55AA55Au;
}
void hw_mv_record(uint32_t id, int32_t got, int32_t target) {
    BKP->mv_id = 0;                                  // 쓰는 도중 리셋돼도 반쪽 기록이 믿어지지 않게
    BKP->mv_got = got; BKP->mv_target = target; BKP->mv_chk = mv_chk(id, target, got);
    BKP->mv_id = id;
}

// ---------------- 폴트: 출력부터 끄고, 어디서 죽었는지 남기고, 리셋 ----------------
__attribute__((used)) void fault_c(uint32_t pc, uint32_t lr) {
    nsleep_low();
    pwm_regs_zero();
    BKP->f_pc = pc; BKP->f_lr = lr; BKP->f_cfsr = SCB->CFSR; BKP->f_hfsr = SCB->HFSR;
    BKP->f_addr = (SCB->CFSR & SCB_CFSR_MMARVALID_Msk) ? SCB->MMFAR : (SCB->CFSR & SCB_CFSR_BFARVALID_Msk) ? SCB->BFAR : 0u;
    BKP->fault_magic = FAULT_MAGIC;
    BKP->phase = PH_FAULT;
    NVIC_SystemReset();
}
// 스택이 넘쳐 보호 구역에 닿은 경우에도 돌도록: MPU를 끄고, 프레임에서 PC·LR만 꺼낸 뒤 스택을 꼭대기로 되돌린다.
__attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile(
        "ldr r3, =0xE000ED94 \n"      // MPU->CTRL
        "movs r2, #0 \n"
        "str r2, [r3] \n"
        "dsb \n isb \n"
        "tst lr, #4 \n"
        "ite eq \n"
        "mrseq r0, msp \n"
        "mrsne r0, psp \n"
        "ldr r1, [r0, #20] \n"        // 쌓인 LR
        "ldr r0, [r0, #24] \n"        // 쌓인 PC
        "ldr r3, =_estack \n"
        "mov sp, r3 \n"
        "b fault_c \n");
}
void MemManage_Handler(void) __attribute__((alias("HardFault_Handler")));
void BusFault_Handler(void) __attribute__((alias("HardFault_Handler")));
void UsageFault_Handler(void) __attribute__((alias("HardFault_Handler")));

// 스택(CCM 64KB, 꼭대기에서 아래로) 바닥 32바이트를 접근 금지로 — 넘치면 조용히 다른 걸 덮지 않고 폴트로 드러난다.
static void mpu_guard(void) {
    MPU->RNR = 0;
    MPU->RBAR = 0x10000000u;
    MPU->RASR = MPU_RASR_XN_Msk | (0u << MPU_RASR_AP_Pos) | (4u << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;   // 32B, 접근 금지
    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk;
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    __DSB(); __ISB();
}

// ---------------- 플래시: 설정 섹터 ----------------
static void flash_cache_reset(void) {                // 지우고 쓴 뒤 데이터 캐시가 옛 내용을 들고 있지 않게
    FLASH->ACR &= ~FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;
}
static bool flash_wait(void) {
    while (FLASH->SR & FLASH_SR_BSY) {}
    uint32_t err = FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_SOP);
    FLASH->SR = err | FLASH_SR_EOP;
    return err == 0;
}
static void flash_unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) { FLASH->KEYR = 0x45670123u; FLASH->KEYR = 0xCDEF89ABu; }
    FLASH->SR = FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_SOP | FLASH_SR_EOP;
}
static bool fl_erase(int s) {
    flash_unlock();
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER | ((s ? CFG_SEC_B : CFG_SEC_A) << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    bool ok = flash_wait();
    FLASH->CR = FLASH_CR_LOCK;
    flash_cache_reset();
    return ok;
}
static bool fl_program(int s, uint32_t off, const uint32_t *w, uint32_t n) {
    volatile uint32_t *dst = (volatile uint32_t *)(s ? CFG_ADDR_B : CFG_ADDR_A) + off;
    flash_unlock();
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
    bool ok = true;
    for (uint32_t i = 0; i < n && ok; i++) { dst[i] = w[i]; ok = flash_wait(); }
    FLASH->CR = FLASH_CR_LOCK;
    flash_cache_reset();
    return ok;
}
static const cfgstore_io_t CFGIO = {
    { (const uint32_t *)CFG_ADDR_A, (const uint32_t *)CFG_ADDR_B }, CFG_SEC_SIZE, fl_erase, fl_program
};
bool hw_cfg_load(cfg_t *c) { return cfgstore_load(&CFGIO, c, sizeof *c) >= 0; }
// 128KB 섹터 지우기는 최대 ≈2초다. 그동안 CPU는 플래시에서 명령을 못 읽어 멈추므로(제어 인터럽트 포함) core가
// 바퀴를 세운 뒤에만 부른다. 워치독은 잠깐 32초로 늘린다.
// 드라이버는 저장 전 상태로 되돌린다 — 래치 중이었으면 계속 자야 한다(core가 정한 상태를 hw가 뒤집지 않는다)
int32_t hw_cfg_save(const cfg_t *c) {
    bool awake = (GPIOB->ODR & (1u << 12)) != 0;
    nsleep_low();
    iwdg_set(6u, 4095u);
    int32_t seq = cfgstore_save(&CFGIO, c, sizeof *c);
    iwdg_set(3u, 249u);
    hw_drv_enable(awake);
    return seq;
}
bool hw_cfg_erase(void) {
    bool awake = (GPIOB->ODR & (1u << 12)) != 0;
    nsleep_low();
    iwdg_set(6u, 4095u);
    bool ok = cfgstore_wipe(&CFGIO);
    iwdg_set(3u, 249u);
    hw_drv_enable(awake);
    return ok;
}

// BOR이 꺼져 있으면(공장값) Level 3(≈2.7V)으로 한 번만 굽는다 — 3.3V가 처질 때 '이상 동작' 대신 리셋으로.
static uint32_t bor_ensure(void) {
    uint32_t lev = (FLASH->OPTCR & FLASH_OPTCR_BOR_LEV) >> FLASH_OPTCR_BOR_LEV_Pos;   // 3=꺼짐 2=L1 1=L2 0=L3
    if (lev != 0u) {
        if (FLASH->OPTCR & FLASH_OPTCR_OPTLOCK) { FLASH->OPTKEYR = 0x08192A3Bu; FLASH->OPTKEYR = 0x4C5D6E7Fu; }
        while (FLASH->SR & FLASH_SR_BSY) {}
        FLASH->OPTCR = (FLASH->OPTCR & ~FLASH_OPTCR_BOR_LEV) | FLASH_OPTCR_OPTSTRT;
        while (FLASH->SR & FLASH_SR_BSY) {}
        FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;
        lev = (FLASH->OPTCR & FLASH_OPTCR_BOR_LEV) >> FLASH_OPTCR_BOR_LEV_Pos;
    }
    return 3u - lev;                                   // 0=꺼짐 … 3=Level 3
}

// ---------------- 재부팅 ----------------
void hw_reboot_bootloader(void) {
    nsleep_low();
    pwm_regs_zero();
    tx_flush(50u);
    BKP->enter_bl = BL_MAGIC;
    NVIC_SystemReset();
}
void hw_hang_for_wdt(void) {
    tx_flush(50u);
    for (;;) {}                                      // 제어 인터럽트는 계속 돌며 출력을 0으로 둔다 — 워치독만 굶긴다
}

// ---------------- LED ----------------
void hw_led(int which, bool on) {
    uint32_t pin = which ? 1u : 11u;
    GPIOB->BSRR = on ? (1u << pin) : (1u << (pin + 16u));
}

// ---------------- 1kHz 제어 타이머 ----------------
void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = ~TIM_SR_UIF;
        enc_update();
        adc_accumulate();
        core_tick_1khz();
    }
}

// ---------------- 부팅 ----------------
static uint32_t uid32(void) {
    const uint32_t *u = (const uint32_t *)UID_BASE;
    uint32_t w[3] = { u[0], u[1], u[2] };
    return crc32_calc(w, sizeof w);
}

static void gpio_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR;
    // 드라이버부터 재운다(기판 풀다운과 같은 쪽) — PWM이 0에 자리 잡기 전엔 깨우지 않는다
    nsleep_low();
    gpio_mode(GPIOB, 12, M_OUT, P_NO, 0);
    gpio_mode(GPIOB, 11, M_OUT, P_NO, 0);
    gpio_mode(GPIOB, 1, M_OUT, P_NO, 0);
    gpio_mode(GPIOB, 13, M_IN, P_NO, 0);              // 기판 100k 풀다운 — 선이 빠지면 LOW = E-stop 쪽
    gpio_mode(GPIOC, 10, M_IN, P_UP, 0); gpio_mode(GPIOC, 11, M_IN, P_UP, 0);
    gpio_mode(GPIOC, 12, M_IN, P_UP, 0); gpio_mode(GPIOD, 2, M_IN, P_UP, 0);
    for (uint32_t n = 0; n <= 5; n++) gpio_mode(GPIOC, n, M_AN, P_NO, 0);
    gpio_mode(GPIOB, 0, M_AN, P_NO, 0);
    gpio_mode(GPIOA, 9, M_AF, P_NO, 7); gpio_mode(GPIOA, 10, M_AF, P_UP, 7);
}

static void periph_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_USART1EN |
                    RCC_APB2ENR_SYSCFGEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN |
                    RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM6EN;
    (void)RCC->APB1ENR;
    pwm_tim_init(TIM8, true); pwm_tim_init(TIM9, false); pwm_tim_init(TIM12, false);
    MOT[0] = (mot_t){ TIM8, &TIM8->CCR1, &TIM8->CCR2 };
    MOT[1] = (mot_t){ TIM8, &TIM8->CCR3, &TIM8->CCR4 };
    MOT[2] = (mot_t){ TIM9, &TIM9->CCR1, &TIM9->CCR2 };
    MOT[3] = (mot_t){ TIM12, &TIM12->CCR1, &TIM12->CCR2 };
    hw_pwm_set_hz(20000u);
    for (uint32_t n = 6; n <= 9; n++) gpio_mode(GPIOC, n, M_AF, P_NO, 3);
    gpio_mode(GPIOA, 2, M_AF, P_NO, 3); gpio_mode(GPIOA, 3, M_AF, P_NO, 3);
    gpio_mode(GPIOB, 14, M_AF, P_NO, 9); gpio_mode(GPIOB, 15, M_AF, P_NO, 9);
    for (int i = 0; i < NW; i++) enc_init(ENC_T[i]);
    gpio_mode(GPIOA, 15, M_AF, P_UP, 1); gpio_mode(GPIOB, 3, M_AF, P_UP, 1);
    gpio_mode(GPIOB, 4, M_AF, P_UP, 2); gpio_mode(GPIOB, 5, M_AF, P_UP, 2);
    gpio_mode(GPIOB, 6, M_AF, P_UP, 2); gpio_mode(GPIOB, 7, M_AF, P_UP, 2);
    gpio_mode(GPIOA, 0, M_AF, P_UP, 2); gpio_mode(GPIOA, 1, M_AF, P_UP, 2);
    adc_init();
    uart_init();
    // E-stop 인터럽트
    SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] & ~SYSCFG_EXTICR4_EXTI13) | SYSCFG_EXTICR4_EXTI13_PB;
    EXTI->FTSR |= EXTI_FTSR_TR13;
    EXTI->PR = EXTI_PR_PR13;
    EXTI->IMR |= EXTI_IMR_MR13;
    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(DMA2_Stream7_IRQn, 4);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    // 1kHz 제어 타이머(켜는 건 core_init 뒤)
    TIM6->PSC = 2u * PCLK1_HZ / 1000000u - 1u;
    TIM6->ARR = 999u;
    TIM6->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM6_DAC_IRQn, 1);
}

static void boot_info(boot_info_t *bi) {
    uint32_t csr = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;                        // 다음 부팅이 이번 원인을 물려받지 않게
    memset(bi, 0, sizeof *bi);
    bi->csr = csr;
    bi->cause = (csr & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) ? "WDT" : (csr & RCC_CSR_LPWRRSTF) ? "LPWR"
              : (csr & RCC_CSR_PORRSTF) ? "POWER" : (csr & RCC_CSR_BORRSTF) ? "BOD" : (csr & RCC_CSR_SFTRSTF) ? "SW"
              : (csr & RCC_CSR_PINRSTF) ? "EXT" : "?";
    bi->warm = BKP->magic == BOOT_MAGIC;
    if (bi->warm) { BKP->prev_phase = BKP->phase; BKP->boot_count = BKP->boot_count + 1u; }
    else {
        BKP->prev_phase = PH_BOOT; BKP->boot_count = 1u; BKP->enter_bl = 0;
        BKP->mv_id = 0; BKP->fault_magic = 0;
    }
    BKP->magic = BOOT_MAGIC;
    BKP->phase = PH_BOOT;
    bi->boot_count = BKP->boot_count;
    bi->prev_phase = (uint8_t)BKP->prev_phase;
    if (bi->warm && BKP->fault_magic == FAULT_MAGIC) {
        bi->fault = true;
        bi->fault_pc = BKP->f_pc; bi->fault_lr = BKP->f_lr; bi->fault_cfsr = BKP->f_cfsr;
        bi->fault_hfsr = BKP->f_hfsr; bi->fault_addr = BKP->f_addr;
    }
    BKP->fault_magic = 0;
    if (bi->warm && BKP->mv_id && BKP->mv_chk == mv_chk(BKP->mv_id, BKP->mv_target, BKP->mv_got)) {
        bi->mv_lost = true;
        bi->mv_id = BKP->mv_id; bi->mv_got = BKP->mv_got; bi->mv_target = BKP->mv_target;
    }
    bi->clk_hse = clk_hse;
    bi->uid = uid32();
}

int main(void) {
    gpio_init();                                     // 드라이버 재우기가 맨 먼저
    clock_init();
    SysTick->LOAD = SYSCLK_HZ / 1000u - 1u;
    SysTick->VAL = 0;
    NVIC_SetPriority(SysTick_IRQn, 2);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    mpu_guard();
    boot_info_t bi;
    boot_info(&bi);
    bi.bor = bor_ensure();
    periph_init();
    iwdg_start();
    core_init(&bi);
    TIM6->CR1 = TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    for (;;) core_poll();
}
