#include "fake_hw.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t fk_ms;
bool fk_estop_ok = true, fk_estop_hit, fk_drv_en;
int32_t fk_duty[NW];
float fk_pos[NW];
int fk_pol_true[NW] = { -1, 1, -1, 1 }, fk_enc_true[NW] = { 1, 1, 1, 1 };
bool fk_blocked[NW];
bool fk_relay_open, fk_relay_weld;
uint32_t fk_vbat_mv = 12000, fk_vm_mv = 12000, fk_v5_mv = 5000;
uint8_t fk_nfault;
uint32_t fk_wdt_kicks;
char fk_out[200000];
uint32_t fk_out_n;
uint32_t fk_flash[2][FAKE_SEC_WORDS];
int fk_fail_after = -1;
bool fk_bootloader, fk_hang;
static float vel[NW];                  // 바퀴 전진 cps
static char rxq[4096];
static uint32_t rx_h, rx_t;
static uint32_t hz = 20000;

#define CPS_MAX 5000.f                 // 듀티 4095에서 무부하 속도
#define STICTION 0.12f                 // 듀티 12% 아래는 안 돈다

void fk_reset_world(void) {
    fk_ms = 1000; fk_estop_ok = true; fk_estop_hit = false; fk_drv_en = false;
    memset(fk_duty, 0, sizeof fk_duty); memset(fk_pos, 0, sizeof fk_pos); memset(vel, 0, sizeof vel);
    memset(fk_blocked, 0, sizeof fk_blocked); fk_relay_open = fk_relay_weld = false;
    for (int i = 0; i < NW; i++) fk_enc_true[i] = 1;
    fk_vbat_mv = 12000; fk_vm_mv = 12000; fk_v5_mv = 5000; fk_nfault = 0;
    fk_out_n = 0; fk_out[0] = 0; rx_h = rx_t = 0; fk_bootloader = fk_hang = false;
}

static void physics(void) {
    for (int w = 0; w < NW; w++) {
        float d = fk_drv_en && !fk_estop_hit ? fk_duty[w] * (float)fk_pol_true[w] / 4095.f : 0.f;
        float target = fabsf(d) < STICTION ? 0.f : d * CPS_MAX;
        if (fk_blocked[w]) target = 0.f;
        vel[w] += (target - vel[w]) * 0.05f;     // τ ≈ 20ms
        if (fk_blocked[w]) vel[w] = 0.f;
        fk_pos[w] += vel[w] * 0.001f;
    }
    if (fk_relay_open) fk_vm_mv = 0;
    else if (fk_estop_ok || fk_relay_weld) fk_vm_mv = fk_vbat_mv;
    else if (fk_vm_mv > fk_vbat_mv) fk_vm_mv = 0;        // 릴레이가 끊었다 → 방전 저항이 VM을 비운다(시험이 정한 역기전력은 둔다)
    else if (!fk_estop_ok && fk_vm_mv == fk_vbat_mv) fk_vm_mv = 0;
}

void fk_step(uint32_t n) {
    while (n--) {
        fk_ms++;
        physics();
        core_tick_1khz();
        core_poll();
    }
}

void fk_rx(const char *line) {
    for (const char *p = line; *p; p++) { rxq[rx_h] = *p; rx_h = (rx_h + 1u) % sizeof rxq; }
    rxq[rx_h] = '\n'; rx_h = (rx_h + 1u) % sizeof rxq;
}
void fk_rx_crc(const char *payload) {
    char b[128]; uint8_t x = 0;
    for (const char *p = payload; *p; p++) x ^= (uint8_t)*p;
    snprintf(b, sizeof b, "%s*%02X", payload, x);
    fk_rx(b);
}

// ---- hw_if ----
uint32_t hw_ms(void) { return fk_ms; }
void hw_pwm_out(int w, int32_t d) { fk_duty[w] = fk_estop_hit ? 0 : d; }
void hw_pwm_zero_all(void) { memset(fk_duty, 0, sizeof fk_duty); }
uint32_t hw_pwm_set_hz(uint32_t h) { hz = h; return h; }
void hw_drv_enable(bool en) { fk_drv_en = en && !fk_estop_hit; }
int32_t hw_enc_raw(int w) { return (int32_t)lroundf(fk_pos[w]) * fk_enc_true[w] * fk_pol_true[w]; }
uint8_t hw_nfault_mask(void) { return fk_nfault; }
bool hw_estop_ok(void) { return fk_estop_ok; }
bool hw_estop_isr_hit(void) { return fk_estop_hit; }
void hw_estop_isr_clear(void) { if (fk_estop_ok) fk_estop_hit = false; }
void hw_adc_take(adc_t *o) {
    memset(o, 0, sizeof *o);
    o->valid = true; o->vdda_mv = 3300;
    o->vbat_pin_mv = fk_vbat_mv * 1000u / 11000u;
    o->vm_pin_mv = fk_vm_mv * 1000u / 11000u;
    o->v5_pin_mv = o->v5_min_pin_mv = fk_v5_mv / 2u;
    for (int i = 0; i < NW; i++) o->ipropi_mv[i] = (uint32_t)(fabsf((float)fk_duty[i]) / 4095.f * 450.f);
}
bool hw_tx(const char *s, uint32_t n) {
    if (fk_out_n + n + 1 >= sizeof fk_out) { fk_out_n = 0; }
    memcpy(fk_out + fk_out_n, s, n); fk_out_n += n; fk_out[fk_out_n] = 0;
    return true;
}
uint32_t hw_tx_drops(void) { return 0; }
int hw_rx(void) {
    if (rx_t == rx_h) return -1;
    int c = (unsigned char)rxq[rx_t]; rx_t = (rx_t + 1u) % sizeof rxq; return c;
}
bool hw_clk_hse(void) { return true; }
void hw_led(int which, bool on) { (void)which; (void)on; }
void hw_wdt_kick(void) { fk_wdt_kicks++; }
void hw_phase(uint8_t ph) { (void)ph; }
void hw_mv_record(uint32_t id, int32_t got, int32_t target) { (void)id; (void)got; (void)target; }

static bool f_erase(int s) {
    for (uint32_t i = 0; i < FAKE_SEC_WORDS; i++) fk_flash[s][i] = 0xFFFFFFFFu;
    return true;
}
static bool f_prog(int s, uint32_t off, const uint32_t *w, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        if (fk_fail_after == 0) return false;          // 전원이 나갔다 — 여기서 멈춘다
        if (fk_fail_after > 0) fk_fail_after--;
        if (off + i >= FAKE_SEC_WORDS) return false;
        fk_flash[s][off + i] &= w[i];                  // 플래시는 1→0만
    }
    return true;
}
const cfgstore_io_t fk_io = { { fk_flash[0], fk_flash[1] }, FAKE_SEC_WORDS * 4u, f_erase, f_prog };
bool hw_cfg_load(cfg_t *c) { return cfgstore_load(&fk_io, c, sizeof *c) >= 0; }
int32_t hw_cfg_save(const cfg_t *c) { return cfgstore_save(&fk_io, c, sizeof *c); }
bool hw_cfg_erase(void) { return cfgstore_wipe(&fk_io); }
void hw_reboot_bootloader(void) { fk_bootloader = true; }
void hw_hang_for_wdt(void) { fk_hang = true; }
void hw_crit_enter(void) {}
void hw_crit_exit(void) {}
