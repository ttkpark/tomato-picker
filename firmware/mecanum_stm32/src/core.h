// core.h — 하드웨어를 모르는 판단 층(프로토콜·주행·안전·설정). 보드 위에선 hw.c가, PC 시험에선 test/fake_hw.c가
// hw_if.h를 채운다. 여기 있는 것은 전부 PC에서 돌려 볼 수 있다(test/test_core.c).
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define FW_VERSION "4.0.0"
#define NW 4                      // 바퀴 0=A/RR 1=B/RL 2=C/FR 3=D/FL (Uno·기판과 같은 순서)

// 리셋 직전 하던 일(BKPSRAM에 남는다). Uno의 last=와 같은 이름.
enum { PH_BOOT = 0, PH_IDLE, PH_RX, PH_PWM, PH_SETHZ, PH_STOP, PH_TEST, PH_FLASH, PH_FAULT };

// 비휘발성 설정. 순서를 바꾸면 CFG_VER를 올린다(옛 기록은 버리고 공장값으로 뜬다).
#define CFG_VER 4
typedef struct {
    int32_t ver;
    int32_t pol[NW];              // 모터 극성 ±1 (L)
    int32_t encdir[NW];           // 엔코더 부호 ±1 (L 5~8번째 인자)
    int32_t max_pwm;              // 듀티 상한 0..4095 (P)
    int32_t accel, decel;         // 슬루, 5ms당 (R)
    int32_t pwm_hz;               // (F)
    int32_t bat_ratio, bat_lo;    // 분압비 ×1000, 저전압 mV (B)
    int32_t kv[NW], ks[NW], kp[NW], ki[NW];   // ×1000 (G)
    int32_t cpr[NW], dia_um[NW];  // (N)
    int32_t lx_um, ly_um;         // 기하 (M lx ly)
    int32_t mv_slow, mv_min, mv_stall_ms, sag_mv, guard_ms;   // (K)
} cfg_t;

typedef struct {                  // hw가 부팅 때 한 번 채운다
    uint32_t boot_count;
    bool warm;                    // BKPSRAM이 살아 있었다 = 전원은 안 나갔다
    const char *cause;            // WDT POWER BOD SW EXT LPWR ?
    uint8_t prev_phase;
    uint32_t csr;                 // RCC_CSR 원문
    bool fault;                   // 직전 리셋이 폴트 처리기에서 왔다
    uint32_t fault_pc, fault_lr, fault_cfsr, fault_hfsr, fault_addr;
    bool mv_lost;                 // 이동 중에 리셋됐다
    uint32_t mv_id;
    int32_t mv_got, mv_target;
    bool clk_hse;
    uint32_t bor;                 // 0=꺼짐 1..3=Level
    uint32_t uid;
} boot_info_t;

typedef struct {                  // hw_adc_take()가 돌려주는 평균(핀 전압 mV, VDDA로 보정 끝)
    bool valid;
    uint32_t vdda_mv;
    uint32_t vbat_pin_mv, vm_pin_mv, v5_pin_mv, v5_min_pin_mv;
    uint32_t ipropi_mv[NW];
} adc_t;

void core_init(const boot_info_t *bi);   // main()이 하드웨어를 다 켠 뒤 한 번
void core_tick_1khz(void);               // 제어 타이머 인터럽트(1kHz)
void core_poll(void);                    // main 루프 — 수신·하트비트·느린 감시
void core_cfg_defaults(cfg_t *c);
bool core_cfg_valid(const cfg_t *c);

// 시험·진단용 읽기 창
typedef struct {
    int32_t cur[3], tgt[3];
    int mode;                     // 'V' / 'C'
    bool estop, latch, weld, lowbat, drv_ok, testing, moving;
    uint8_t clf;
    float spd[NW];
    int32_t duty[NW];
    int32_t enc[NW];
    uint32_t rx_ok, rx_bad;
    cfg_t cfg;
} core_view_t;
void core_view(core_view_t *v);
