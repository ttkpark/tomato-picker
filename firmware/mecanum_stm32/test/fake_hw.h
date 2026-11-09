// 가짜 하드웨어 — core를 PC에서 돌린다. 바퀴는 1차 지연 + 정지마찰, 엔코더 부호·막힘은 시험이 바꾼다.
#pragma once
#include "../src/cfgstore.h"
#include "../src/hw_if.h"

#define FAKE_SEC_WORDS 512u            // 2KB 섹터(작게 — 넘어가기를 자주 시험)
extern uint32_t fk_ms;
extern bool fk_estop_ok, fk_estop_hit, fk_drv_en;
extern int32_t fk_duty[NW];            // hw에 실제로 나간 값(극성 적용 후)
extern float fk_pos[NW];               // 바퀴 전진 카운트(실물)
extern int fk_pol_true[NW], fk_enc_true[NW];   // 실물 배선
extern bool fk_blocked[NW];
extern bool fk_relay_open;             // 코일은 붙었는데 접점이 안 붙는다
extern bool fk_relay_weld;             // 코일이 떨어져도 접점이 붙어 있다
extern uint32_t fk_vbat_mv, fk_vm_mv, fk_v5_mv; // 실제 전압(분압 전)
extern uint8_t fk_nfault;
extern uint32_t fk_wdt_kicks;
extern char fk_out[200000];
extern uint32_t fk_out_n;
extern uint32_t fk_flash[2][FAKE_SEC_WORDS];
extern int fk_fail_after;              // ≥0이면 그만큼 워드를 쓴 뒤 '전원이 나간다'
extern const cfgstore_io_t fk_io;
extern bool fk_bootloader, fk_hang;

void fk_reset_world(void);
void fk_rx(const char *line);          // 체크섬 없이 한 줄
void fk_rx_crc(const char *payload);   // "*HH" 붙여서
void fk_step(uint32_t n_ms);           // 1ms마다 물리 → tick → poll
