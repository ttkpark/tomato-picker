// hw_if.h — core가 하드웨어에게 시키는 일 전부. 보드 = hw.c, PC 시험 = test/fake_hw.c.
#pragma once
#include "core.h"

uint32_t hw_ms(void);

// 출력: duty는 극성까지 적용된 부호 있는 값, 크기는 0..4095 척도(PCA9685 시절 척도 유지).
void hw_pwm_out(int w, int32_t duty);
void hw_pwm_zero_all(void);
uint32_t hw_pwm_set_hz(uint32_t hz);      // 실제로 건 주파수
void hw_drv_enable(bool en);              // nSLEEP. false면 드라이버 출력 Hi-Z

int32_t hw_enc_raw(int w);                // 32비트로 이어 붙인 카운트(부호 처리 전)
uint8_t hw_nfault_mask(void);             // 비트 i = 바퀴 i 드라이버 폴트
bool hw_estop_ok(void);                   // 릴레이 코일 붙음 = 모터 허용
bool hw_estop_isr_hit(void);              // E-stop 인터럽트가 출력을 끊었다(지울 때까지 유지)
void hw_estop_isr_clear(void);
void hw_adc_take(adc_t *out);             // 지난 호출 이후 평균

bool hw_tx(const char *s, uint32_t n);    // 통째로 넣거나 통째로 버린다(막지 않는다)
uint32_t hw_tx_drops(void);
int hw_rx(void);                          // 한 바이트, 없으면 -1

bool hw_clk_hse(void);                    // 지금 크리스털로 도는가(NMI가 HSI로 바꿨으면 false)
void hw_led(int which, bool on);          // 0=하트비트 1=폴트
void hw_wdt_kick(void);
void hw_phase(uint8_t ph);                // 리셋 흔적의 '지금 하는 일'
void hw_mv_record(uint32_t id, int32_t got, int32_t target);   // id 0 = 지움

bool hw_cfg_load(cfg_t *c);               // 가장 최근의 온전한 기록
int32_t hw_cfg_save(const cfg_t *c);      // 기록 번호, 실패 <0. 바퀴가 서 있을 때만 부를 것(플래시 지우는 동안 CPU가 선다)
bool hw_cfg_erase(void);

void hw_reboot_bootloader(void);          // 리셋 후 시스템 부트로더로
void hw_hang_for_wdt(void);               // 워치독 자가진단: 영원히 멈춘다
void hw_crit_enter(void);                 // 제어 인터럽트만 막는다(E-stop 인터럽트는 막지 않는다)
void hw_crit_exit(void);
