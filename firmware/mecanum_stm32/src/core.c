// core.c — 주행 보드의 판단 전부. 하드웨어는 hw_if.h로만 만진다.
//
// 실행 문맥은 둘이다:
//   ISR  core_tick_1khz()  — 엔코더·속도 추정·PID·슬루(5ms)·데드맨·거리 이동·바퀴 시험. 출력은 여기서만 쓴다.
//   main core_poll()       — 수신·명령·하트비트·E-stop 디바운스·배터리·릴레이 감시.
// main이 제어 상태를 바꿀 때는 hw_crit_enter()로 제어 인터럽트를 막는다(E-stop 인터럽트는 안 막힌다).
// ISR이 젯슨에 알릴 일(이동 끝·폴트·데드맨)은 이벤트 큐에 넣고 main이 내보낸다 — ISR은 직렬을 안 만진다.
#include "core.h"
#include "hw_if.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---------------- 상수 ----------------
#define TICK_SLEW_MS 5u
#define CMD_SOFT_MS 300u            // 계약 §9 층2
#define CMD_HARD_MS 1000u           // 계약 §9 층1
#define ESTOP_DEBOUNCE_MS 20u
#define TEST_MS 1000u
#define TEST_DUTY 200               // /255
#define SETTLE_MS 150u
#define BAT_HOLD_MS 2000u
#define BAT_HYST_MV 300u
#define RLY_MIN_VIN_MV 3000u
#define RLY_HOLD_N 3u                // rlyopen: 300ms
#define WELD_HOLD_N 10u              // weld: 1초 — 손으로 밀면 역기전력이 VM을 잠깐 올린다(배터리의 85%까진 못 간다)
#define IPROPI_MA_PER_V 2222u       // 1kΩ × 450µA/A — DRV8874 IPROPI
#define V5_RATIO 2000u              // ×1000, 10k/10k
#define NAK_REPEAT_MS 1000u         // 50Hz 구동 지령에 대한 nak는 초당 한 번만
#define SPD_RING 256u               // 1ms 표본 고리(바퀴마다)
#define SPD_MIN_WIN 20u             // 속도 창(ms) — 최소
#define SPD_MAX_WIN 200u            //               — 최대(저속)
#define SPD_MIN_COUNTS 8            // 창 안에 이만큼 쌓이면 그 창으로 끊는다(M/T 혼합)
#define PI_F 3.14159265f

// ---------------- 상태 ----------------
static cfg_t C;                     // 지금 설정(저장본과 다를 수 있다)
static boot_info_t BI;
static bool cfg_from_flash;
static int32_t cfg_seq = -1;

// 지령 — 'V' = duty(−255..255) 단위, 'C' = mm/s, mm/s, mdeg/s(REP-103: +vy 왼쪽, +w 반시계)
static volatile int mode = 'V';
static volatile int32_t tgt[3], cur[3];
static volatile uint32_t last_cmd_ms;
static volatile bool hard_tripped, soft_active;

// 출력·폐루프
static volatile float wtarget[NW];  // 폐루프 목표 cps(바퀴 전진 +) 또는 개루프 비율(−1..1)
static volatile bool wclosed[NW];   // 이번 지령에서 이 바퀴가 폐루프인가
static volatile int32_t duty[NW];   // 마지막 출력(바퀴 전진 +, 극성 적용 전)
static float integ[NW];
static uint32_t bad_ms[NW], guard_cnt[NW];
static volatile uint8_t clf_mask;   // 폐루프 폭주로 개루프로 떨어진 바퀴
static volatile bool guard_hold;    // 막힘 지킴이가 세웠다 — 0 지령이나 S가 올 때까지 구동 지령 거절
static volatile bool sat_seen;      // 지난 하트비트 이후 PID 포화가 있었나

// 엔코더·속도
static int32_t enc_last_raw[NW];
static volatile int32_t enc_acc[NW];    // 바퀴 전진 = +
static int32_t ring[NW][SPD_RING];
static uint32_t ring_i;
static volatile float spd[NW];          // cps

// 시험(T)
static volatile bool testing;
static volatile int test_wheel;
static volatile uint32_t test_until;
static int32_t test_enc0;

// 거리 이동(M)
enum { MV_IDLE = 0, MV_RUN, MV_SETTLE };
static volatile int mv_state;
static uint32_t mv_id;
static int32_t mv_target, mv_v[3], mv_peak, mv_sign[NW], mv_enc0[NW], mv_mark[NW];
static uint32_t mv_mark_ms[NW], mv_t0, mv_settle_until;

// 안전
static volatile bool estop, latch, weld, drv_ok = false;
static bool estop_raw;
static uint32_t estop_edge;
static bool rly_open;
static uint8_t rly_bad_n;
static bool lowbat;
static uint32_t low_since;
static uint32_t bat_buf[10], bat_n, bat_i, bat_mv, vm_mv, v5_mv, v5_min_mv = 0xFFFFFFFFu, vdda_mv, amp_ma[NW];
static volatile uint32_t sag_until;
static uint32_t sag_count;
static uint8_t flt_mask;
static uint32_t flt_count;

// 링크
static char rxbuf[64];
static uint32_t rxlen;
static bool rx_overflow;
static uint32_t rx_ok, rx_bad;
static bool strict_crc;
static uint32_t last_loop_ms, wdt_near, last_hb_ms, last_100_ms;
static uint32_t nak_ms[4];            // 0=estop 1=lowbat 2=nocalib 3=guard — 반복 nak 억제
static volatile uint32_t isr_alive;

// ---------------- 이벤트 큐(ISR → main) ----------------
enum { EV_MV_DONE = 1, EV_MV_STALL, EV_MV_ABORT, EV_CLFAULT, EV_DEADMAN, EV_TEST, EV_GUARD };
typedef struct { uint8_t type; uint8_t arg; int32_t a, b, c, d; } ev_t;
#define EVQ 16u
static ev_t evq[EVQ];
static volatile uint32_t ev_head, ev_tail, ev_drops;
static void ev_push(uint8_t type, uint8_t arg, int32_t a, int32_t b, int32_t c, int32_t d) {
    uint32_t h = ev_head, n = (h + 1u) % EVQ;
    if (n == ev_tail) { ev_drops++; return; }
    evq[h] = (ev_t){ type, arg, a, b, c, d };
    ev_head = n;
}

// ---------------- 출력 문자열(작은 조립기 — printf 없음) ----------------
typedef struct { char b[400]; uint32_t n; } out_t;
static void o_s(out_t *o, const char *s) { while (*s && o->n < sizeof o->b - 1) o->b[o->n++] = *s++; }
static void o_c(out_t *o, char c) { if (o->n < sizeof o->b - 1) o->b[o->n++] = c; }
static void o_i(out_t *o, int32_t v) {
    char t[12]; int k = 0; uint32_t u = v < 0 ? 0u - (uint32_t)v : (uint32_t)v;
    do { t[k++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    if (v < 0) o_c(o, '-');
    while (k) o_c(o, t[--k]);
}
static void o_u(out_t *o, uint32_t u) {
    char t[11]; int k = 0;
    do { t[k++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    while (k) o_c(o, t[--k]);
}
static void o_x(out_t *o, uint32_t v, int digits) {
    static const char hx[] = "0123456789ABCDEF";
    for (int k = digits - 1; k >= 0; k--) o_c(o, hx[(v >> (4 * k)) & 15u]);
}
static void o_kv(out_t *o, const char *k, int32_t v) { o_c(o, ' '); o_s(o, k); o_c(o, '='); o_i(o, v); }
static void o_arr(out_t *o, const char *k, const int32_t *a, int n) {
    o_c(o, ' '); o_s(o, k); o_c(o, '=');
    for (int i = 0; i < n; i++) { if (i) o_c(o, ','); o_i(o, a[i]); }
}
static void o_send(out_t *o) { o->b[o->n++] = '\n'; hw_tx(o->b, o->n); o->n = 0; }
static void say(const char *s) { out_t o = { .n = 0 }; o_s(&o, s); o_send(&o); }

// ---------------- 설정 ----------------
void core_cfg_defaults(cfg_t *c) {
    memset(c, 0, sizeof *c);
    c->ver = CFG_VER;
    const int32_t pol[NW] = { -1, 1, -1, 1 };   // 2026-10-22 실측(Uno와 같은 모터·배선)
    for (int i = 0; i < NW; i++) { c->pol[i] = pol[i]; c->encdir[i] = 1; }
    c->max_pwm = 2000; c->accel = 6; c->decel = 12; c->pwm_hz = 20000;
    c->bat_ratio = 11000; c->bat_lo = 6400;
    c->mv_slow = 400; c->mv_min = 100; c->mv_stall_ms = 500; c->sag_mv = 4500; c->guard_ms = 1500;
}

static bool pm1(int32_t v) { return v == 1 || v == -1; }
bool core_cfg_valid(const cfg_t *c) {
    if (c->ver != CFG_VER) return false;
    for (int i = 0; i < NW; i++) {
        if (!pm1(c->pol[i]) || !pm1(c->encdir[i])) return false;
        if (c->kv[i] < 0 || c->ks[i] < 0 || c->kp[i] < 0 || c->ki[i] < 0) return false;
        if (c->cpr[i] < 0 || c->dia_um[i] < 0) return false;
    }
    return c->max_pwm >= 1 && c->max_pwm <= 4095 && c->accel >= 1 && c->accel <= 255 && c->decel >= 1 &&
           c->decel <= 255 && c->pwm_hz >= 24 && c->pwm_hz <= 40000 && c->bat_ratio > 0 && c->bat_lo >= 0 &&
           c->lx_um >= 0 && c->ly_um >= 0 && c->mv_slow > 0 && c->mv_min >= 1 && c->mv_min <= 255 &&
           c->mv_stall_ms > 0 && c->sag_mv >= 0 && c->guard_ms >= 0;
}

// 물리 단위(C)를 쓸 수 있는가 — 바퀴마다 엔코더·지름·속도 이득, 그리고 기하
static bool calib_ok(void) {
    if (C.lx_um <= 0 || C.ly_um <= 0) return false;
    for (int i = 0; i < NW; i++)
        if (C.cpr[i] <= 0 || C.dia_um[i] <= 0 || C.kv[i] <= 0 || C.ks[i] >= C.max_pwm * 1000) return false;
    return true;
}
static float cps_full(int w) {            // 피드포워드만으로 듀티가 꽉 차는 속도
    return ((float)C.max_pwm - C.ks[w] / 1000.f) / (C.kv[w] / 1000.f);
}
static float mm_per_count(int w) { return PI_F * (C.dia_um[w] / 1000.f) / (float)C.cpr[w]; }
static int32_t vmax_mms(void) {           // 네 바퀴 중 가장 느린 바퀴가 정한다
    if (!calib_ok()) return 0;
    float m = 1e9f;
    for (int i = 0; i < NW; i++) { float v = cps_full(i) * mm_per_count(i); if (v < m) m = v; }
    return (int32_t)m;
}
static int32_t wmax_mdegs(void) {
    float k = (C.lx_um + C.ly_um) / 1000.f;
    return calib_ok() && k > 0 ? (int32_t)(vmax_mms() / k * 180000.f / PI_F) : 0;
}

// ---------------- 출력 ----------------
static void out_wheel(int w, int32_t d) {        // d: 바퀴 전진 +, 0..max_pwm
    if (d > C.max_pwm) d = C.max_pwm;
    if (d < -C.max_pwm) d = -C.max_pwm;
    duty[w] = d;
    hw_pwm_out(w, drv_ok ? d * C.pol[w] : 0);
}
static void zero_outputs(void) {
    for (int w = 0; w < NW; w++) { duty[w] = 0; integ[w] = 0; bad_ms[w] = 0; guard_cnt[w] = 0; wtarget[w] = 0; }
    hw_pwm_zero_all();
}

static void mv_end(void) { mv_state = MV_IDLE; hw_mv_record(0, 0, 0); }
static int32_t mv_progress(void) {
    int32_t sum = 0, n = 0;
    for (int w = 0; w < NW; w++) if (mv_sign[w]) { sum += (enc_acc[w] - mv_enc0[w]) * mv_sign[w]; n++; }
    return n ? sum / n : 0;
}
static void mv_abort(uint8_t why) {        // 호출자가 크리티컬 구역 안에서
    if (mv_state == MV_IDLE) return;
    ev_push(EV_MV_ABORT, why, (int32_t)mv_id, mv_progress(), mv_target, 0);
    mv_end();
}
enum { AB_V = 1, AB_S, AB_M, AB_T, AB_X, AB_ESTOP, AB_DEADMAN, AB_LOWBAT, AB_C, AB_GUARD };
static const char *abort_name(uint8_t a) {
    static const char *n[] = { "?", "V", "S", "M", "T", "X", "estop", "deadman", "lowbat", "C", "guard" };
    return a < sizeof n / sizeof n[0] ? n[a] : "?";
}

// 즉시 정지 — 슬루·PID를 건너뛴다(계약 §10.2). ISR 또는 크리티컬 구역 안에서만.
static void hard_stop(void) {
    hw_phase(PH_STOP);
    for (int i = 0; i < 3; i++) { tgt[i] = 0; cur[i] = 0; }
    testing = false;
    zero_outputs();
    hw_phase(PH_IDLE);
}

// cur → 바퀴 목표. V는 Uno와 같은 믹싱(바퀴별 ±255 자르기 포함), C는 역기구학으로 cps.
static void mix(void) {
    hw_phase(PH_PWM);
    if (mode == 'V') {
        int32_t FL = cur[0] - cur[1] - cur[2], FR = cur[0] + cur[1] + cur[2];
        int32_t RL = cur[0] + cur[1] - cur[2], RR = cur[0] - cur[1] + cur[2];
        int32_t wc[NW] = { RR, RL, FR, FL };
        for (int w = 0; w < NW; w++) {
            int32_t s = wc[w] > 255 ? 255 : wc[w] < -255 ? -255 : wc[w];
            wclosed[w] = C.kv[w] > 0 && !(clf_mask & (1u << w));
            wtarget[w] = wclosed[w] ? s / 255.f * cps_full(w) : s / 255.f;
        }
    } else {
        // REP-103(+vy 왼쪽, +w 반시계). 이 차체에서 V의 +vy는 오른쪽, V의 +w는 반시계로 실측됐다(계약 §14.1 AxisSigns
        // vy=−1, w=+1) → C는 V 믹싱에 vy만 뒤집어 넣는다. k = lx+ly, ω[rad/s].
        float vx = (float)cur[0], vy = -(float)cur[1];
        float kw = (C.lx_um + C.ly_um) / 1000.f * (cur[2] / 1000.f) * PI_F / 180.f;
        float FL = vx - vy - kw, FR = vx + vy + kw, RL = vx + vy - kw, RR = vx - vy + kw;
        float wm[NW] = { RR, RL, FR, FL };
        for (int w = 0; w < NW; w++) {
            wclosed[w] = true;                               // C는 calib_ok에서만 — 폭주한 바퀴는 피드포워드만(control())
            wtarget[w] = wm[w] / mm_per_count(w);
        }
    }
    hw_phase(PH_IDLE);
}

static int32_t approach(int32_t c, int32_t t, int32_t up, int32_t down) {
    if (c == t) return c;
    int32_t st = (labs(t) > labs(c)) ? up : down;
    if (t > c) { c += st; if (c > t) c = t; } else { c -= st; if (c < t) c = t; }
    return c;
}

// ---------------- 거리 이동 M (Uno v2.3과 같은 뜻, 엔코더는 네 바퀴 다) ----------------
static bool mv_start(uint32_t id, int32_t ticks, int32_t vx, int32_t vy, int32_t w, out_t *o) {
    int32_t m4[NW] = { vx - vy + w, vx + vy - w, vx + vy + w, vx - vy - w };   // RR RL FR FL
    if (id == 0 || id > 65535u || ticks <= 0) { o_s(o, "nak mv args"); return false; }
    int32_t peak = 0;
    for (int i = 0; i < NW; i++) if (labs(m4[i]) > peak) peak = labs(m4[i]);
    if (!peak) { o_s(o, "nak mv blind"); return false; }
    mv_abort(AB_M);
    if (testing) hard_stop();
    uint32_t now = hw_ms();
    for (int i = 0; i < NW; i++) {
        mv_sign[i] = m4[i] > 0 ? 1 : m4[i] < 0 ? -1 : 0;
        mv_enc0[i] = mv_mark[i] = enc_acc[i];
        mv_mark_ms[i] = now;
    }
    mv_id = id; mv_target = ticks; mv_v[0] = vx; mv_v[1] = vy; mv_v[2] = w; mv_peak = peak; mv_t0 = now;
    mode = 'V';
    tgt[0] = vx; tgt[1] = vy; tgt[2] = w;
    mv_state = MV_RUN;
    o_s(o, "ok M "); o_u(o, id); o_c(o, ' '); o_i(o, ticks);
    return true;
}

static void mv_tick(uint32_t now) {        // 5ms마다(ISR)
    int32_t got = mv_progress();
    if (mv_state == MV_SETTLE) {
        if ((int32_t)(now - mv_settle_until) < 0) return;
        ev_push(EV_MV_DONE, 0, (int32_t)mv_id, got, mv_target, (int32_t)(now - mv_t0));
        mv_end();
        return;
    }
    hw_mv_record(mv_id, got, mv_target);
    int32_t rem = mv_target - got;
    if (rem <= 0) { hard_stop(); mv_state = MV_SETTLE; mv_settle_until = now + SETTLE_MS; return; }
    for (int i = 0; i < NW; i++) {
        if (!mv_sign[i]) continue;
        if ((enc_acc[i] - mv_mark[i]) * mv_sign[i] >= 2) { mv_mark[i] = enc_acc[i]; mv_mark_ms[i] = now; }
        else if (now - mv_mark_ms[i] > (uint32_t)C.mv_stall_ms) {
            hard_stop();
            ev_push(EV_MV_STALL, (uint8_t)i, (int32_t)mv_id, got, mv_target, (int32_t)sag_count);
            mv_end();
            return;
        }
    }
    int32_t s = rem < C.mv_slow ? rem * 256 / C.mv_slow : 256;
    int32_t smin = C.mv_min * 256 / mv_peak;
    if (smin > 256) smin = 256;
    if (s < smin) s = smin;
    for (int k = 0; k < 3; k++) tgt[k] = mv_v[k] * s / 256;
}

// ---------------- 1kHz 제어 (ISR) ----------------
static float spd_estimate(int w) {
    uint32_t now_i = ring_i;
    int32_t c = ring[w][now_i % SPD_RING];
    for (uint32_t k = SPD_MIN_WIN; k <= SPD_MAX_WIN; k++) {
        int32_t d = c - ring[w][(now_i - k) % SPD_RING];
        if (labs(d) >= SPD_MIN_COUNTS || k == SPD_MAX_WIN) return d * 1000.f / (float)k;
    }
    return 0;
}

// 한 바퀴 PID 한 걸음. 계약 §10.1: 피드포워드(kv·t + ks·sign) + PI 잔차, 적분은 포화 쪽으로는 안 쌓는다(clamping).
static int32_t pid(int w, float t, float meas, bool *fault) {
    *fault = false;
    if (t == 0.f) { integ[w] = 0; bad_ms[w] = 0; return 0; }
    float kv = C.kv[w] / 1000.f, ks = C.ks[w] / 1000.f, kp = C.kp[w] / 1000.f, ki = C.ki[w] / 1000.f;
    float mx = (float)C.max_pwm, e = t - meas;
    float u0 = (t > 0 ? ks : -ks) + kv * t + kp * e;
    float u = u0 + ki * integ[w];
    bool sat_hi = u >= mx, sat_lo = u <= -mx;
    if (ki > 0 && !((sat_hi && e > 0) || (sat_lo && e < 0))) {
        integ[w] += e * 0.001f;
        float lim = 2.f * mx / ki;              // 적분항 혼자 ±2·상한을 못 넘게(2차 안전망)
        if (integ[w] > lim) integ[w] = lim;
        if (integ[w] < -lim) integ[w] = -lim;
        u = u0 + ki * integ[w];
    }
    if (u > mx) { u = mx; sat_seen = true; }
    if (u < -mx) { u = -mx; sat_seen = true; }
    // 폭주 감시: 0.8·상한 넘게 밀면서 0.5초째 반대로 돌거나 목표의 10%도 못 낸다(엔코더 부호·선 빠짐·막힘)
    bool stuck = fabsf(u) > 0.8f * mx && (meas * t < 0 || fabsf(meas) < 0.1f * fabsf(t));
    bad_ms[w] = stuck ? bad_ms[w] + 1u : 0u;
    if (bad_ms[w] > 500u) *fault = true;
    return (int32_t)u;
}

static void control_wheels(void) {
    for (int w = 0; w < NW; w++) {
        if (testing) {
            if (w == test_wheel) continue;          // 시험 바퀴는 tick이 직접 쓴다
            out_wheel(w, 0);
            continue;
        }
        float t = wtarget[w];
        int32_t d;
        if (!wclosed[w]) {
            d = (int32_t)(t * C.max_pwm);
        } else if (clf_mask & (1u << w)) {          // 폭주했던 바퀴: 피드포워드만
            float ks = C.ks[w] / 1000.f, kv = C.kv[w] / 1000.f;
            d = t == 0.f ? 0 : (int32_t)((t > 0 ? ks : -ks) + kv * t);
        } else {
            bool fault;
            d = pid(w, t, spd[w], &fault);
            if (fault) {
                clf_mask |= (uint8_t)(1u << w);
                integ[w] = 0;
                ev_push(EV_CLFAULT, (uint8_t)w, 0, 0, 0, 0);
            }
        }
        out_wheel(w, d);
        // 막힘 지킴이(개루프·폐루프 공통): 듀티 절반 넘게 미는데 guard_ms 동안 바퀴가 거의 안 돈다 → 정지.
        // 막힌 모터는 정지 전류를 먹어 전원을 끌어내린다(Uno 시절 리셋의 주범).
        if (C.guard_ms && labs(d) > C.max_pwm / 2 && fabsf(spd[w]) < 20.f) {
            if (++guard_cnt[w] > (uint32_t)C.guard_ms) {
                mv_abort(AB_GUARD);
                hard_stop();
                guard_hold = true;          // 젯슨은 V를 50Hz로 계속 보낸다 — 안 막으면 1.5초마다 막힌 모터를 다시 민다
                ev_push(EV_GUARD, (uint8_t)w, 0, 0, 0, 0);
                return;
            }
        } else guard_cnt[w] = 0;
    }
}

void core_tick_1khz(void) {
    isr_alive++;
    uint32_t now = hw_ms();
    // 엔코더: 하드웨어 카운터 차분을 부호 붙여 누적(16비트 타이머도 hw가 32비트로 이어 준다)
    ring_i++;
    for (int w = 0; w < NW; w++) {
        int32_t r = hw_enc_raw(w), d = r - enc_last_raw[w];
        enc_last_raw[w] = r;
        enc_acc[w] += d * C.encdir[w] * C.pol[w];
        ring[w][ring_i % SPD_RING] = enc_acc[w];
        spd[w] = spd_estimate(w);
    }
    if (!drv_ok) { if (cur[0] || cur[1] || cur[2] || testing) hard_stop(); return; }

    if (testing) {
        if ((int32_t)(now - test_until) >= 0) {
            ev_push(EV_TEST, (uint8_t)test_wheel, enc_acc[test_wheel] - test_enc0, 0, 0, 0);
            hard_stop();
        } else out_wheel(test_wheel, (wtarget[test_wheel] < 0 ? -1 : 1) * TEST_DUTY * C.max_pwm / 255);
        control_wheels();
        return;
    }

    if (isr_alive % TICK_SLEW_MS == 0) {   // 5ms — 인터럽트 횟수로 센다(시각 나머지는 한 번 밀리면 틱을 건너뛴다)
        uint32_t silence = now - last_cmd_ms;
        if (silence > CMD_HARD_MS) {
            soft_active = true;
            if (cur[0] || cur[1] || cur[2] || mv_state == MV_RUN) {
                mv_abort(AB_DEADMAN);
                hard_stop();
                if (!hard_tripped) { hard_tripped = true; ev_push(EV_DEADMAN, 0, 0, 0, 0, 0); }
            }
        } else if (silence > CMD_SOFT_MS) {
            soft_active = true;
            mv_abort(AB_DEADMAN);
            tgt[0] = tgt[1] = tgt[2] = 0;
        } else {
            soft_active = false;
            hard_tripped = false;
        }
        if (mv_state != MV_IDLE) mv_tick(now);
        bool sag = (int32_t)(sag_until - now) > 0;
        int32_t sc[3] = { 1, 1, 1 };
        if (mode == 'C') {        // 슬루 단위를 축마다 최고속 비율로 — R의 뜻(5ms당 255분의 몇)을 그대로 둔다
            int32_t vm = vmax_mms(), wm = wmax_mdegs();
            sc[0] = sc[1] = vm / 255 > 0 ? vm / 255 : 1;
            sc[2] = wm / 255 > 0 ? wm / 255 : 1;
        }
        bool changed = false;
        for (int k = 0; k < 3; k++) {
            int32_t n = approach(cur[k], tgt[k], C.accel * sc[k], C.decel * sc[k]);
            if (sag && labs(n) > labs(cur[k])) n = cur[k];      // 전원이 처진 동안엔 가속 금지(줄이는 건 허용)
            if (n != cur[k]) { cur[k] = n; changed = true; }
        }
        if (changed || isr_alive % 50u == 0) mix();   // 50ms마다 다시 — 설정(G·P)이 바뀐 것도 반영
    }
    control_wheels();
}

// ---------------- 수신 ----------------
static int8_t hexv(char c) {
    if (c >= '0' && c <= '9') return (int8_t)(c - '0');
    if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
    return -1;
}
// 체크섬 "*HH": 없음 0, 맞음 1, 틀림 −1. 맞으면 '*'를 지운다.
static int verify_crc(char *line) {
    char *star = 0;
    for (char *p = line; *p; p++) if (*p == '*') star = p;
    if (!star) return 0;
    if (!star[1] || !star[2] || star[3]) return -1;
    int8_t hi = hexv(star[1]), lo = hexv(star[2]);
    if (hi < 0 || lo < 0) return -1;
    uint8_t x = 0;
    for (char *p = line; p < star; p++) x ^= (uint8_t)*p;
    *star = 0;
    return x == (uint8_t)((hi << 4) | lo) ? 1 : -1;
}

// 정수 인자를 끝까지 읽는다. 숫자가 아닌 글자가 끼면 실패(조용히 0으로 읽지 않는다).
static int parse_args(char *p, int32_t *a, int max) {
    int n = 0;
    for (;;) {
        while (*p == ' ') p++;
        if (!*p) return n;
        if (n == max) return -1;
        int32_t sign = 1, v = 0, digits = 0;
        if (*p == '-' || *p == '+') { if (*p == '-') sign = -1; p++; }
        while (*p >= '0' && *p <= '9') {
            if (++digits > 9) return -1;
            v = v * 10 + (*p - '0');
            p++;
        }
        if (!digits || (*p && *p != ' ')) return -1;
        a[n++] = sign * v;
    }
}

static int32_t clamp255(int32_t v) { return v > 255 ? 255 : v < -255 ? -255 : v; }
static bool drivers_allowed(void) { return !estop && !latch && !weld && !hw_estop_isr_hit(); }
static void upd_sleep(void) { drv_ok = drivers_allowed(); hw_drv_enable(drv_ok); }
static char rly_state(void) { return weld ? 'W' : rly_open ? 'O' : (vm_mv * 2 > bat_mv ? '1' : '0'); }
static int estop_state(void) { return estop ? 1 : latch ? 2 : 0; }

static bool nak_limited(int k, uint32_t now, out_t *o, const char *msg) {
    if (now - nak_ms[k] < NAK_REPEAT_MS && nak_ms[k]) return false;
    nak_ms[k] = now ? now : 1;
    o_s(o, msg);
    return true;
}

static void print_cap(void) {
    out_t o = { .n = 0 };
    o_s(&o, "cap proto=2 fw=" FW_VERSION " board=stm32f405 id="); o_x(&o, BI.uid, 8);
    o_s(&o, " drive=mecanum wheels=4 units=1 closed_loop=1 enc=4");
    o_kv(&o, "calib", calib_ok()); o_kv(&o, "vmax", vmax_mms()); o_kv(&o, "vymax", vmax_mms());
    o_kv(&o, "wmax", wmax_mdegs()); o_s(&o, " vin=1 amp=1 estop_hw=1"); o_kv(&o, "pwm_hz", C.pwm_hz);
    o_send(&o);
}

static const char *phase_name(uint8_t p) {
    static const char *n[] = { "boot", "idle", "serial", "pwm", "setHz", "stop", "test", "flash", "fault" };
    return p < sizeof n / sizeof n[0] ? n[p] : "boot";
}

static void print_boot(void) {           // motor_link가 파싱하는 형식(Uno v2.1과 같다) + 폴트 정보
    out_t o = { .n = 0 };
    o_s(&o, "boot #"); o_u(&o, BI.boot_count); o_s(&o, BI.warm ? " warm" : " cold");
    o_s(&o, " cause="); o_s(&o, BI.cause); o_s(&o, " last="); o_s(&o, phase_name(BI.prev_phase));
    o_s(&o, " i2cerr=0 near="); o_u(&o, wdt_near); o_s(&o, " mcusr=0x"); o_x(&o, BI.csr >> 24, 2); o_s(&o, " r2=0x0");
    if (BI.fault) {
        o_s(&o, " fault pc=0x"); o_x(&o, BI.fault_pc, 8); o_s(&o, " lr=0x"); o_x(&o, BI.fault_lr, 8);
        o_s(&o, " cfsr=0x"); o_x(&o, BI.fault_cfsr, 8); o_s(&o, " hfsr=0x"); o_x(&o, BI.fault_hfsr, 8);
        o_s(&o, " addr=0x"); o_x(&o, BI.fault_addr, 8);
    }
    o_send(&o);
}

static void print_status(void) {
    out_t o = { .n = 0 };
    int32_t c3[3] = { cur[0], cur[1], cur[2] };
    o_s(&o, "st"); o_arr(&o, "cur", c3, 3); o_kv(&o, "MAX_PWM", C.max_pwm);
    o_s(&o, " slew="); o_i(&o, C.accel); o_c(&o, '/'); o_i(&o, C.decel);
    o_kv(&o, "strict", strict_crc); o_s(&o, " i2cerr=0");
    o_s(&o, " POL="); for (int i = 0; i < NW; i++) { o_i(&o, C.pol[i]); o_c(&o, ','); }
    o_s(&o, " ENC="); for (int i = 0; i < NW; i++) { o_i(&o, C.encdir[i]); o_c(&o, ','); }
    o_kv(&o, "hz", C.pwm_hz); o_s(&o, hw_clk_hse() ? " clk=HSE" : " clk=HSI"); o_kv(&o, "bor", (int32_t)BI.bor);
    o_kv(&o, "estop", estop_state()); o_kv(&o, "vm", (int32_t)vm_mv); o_s(&o, " rly="); o_c(&o, rly_state());
    o_s(&o, " bat="); o_i(&o, (int32_t)bat_mv); o_c(&o, '/'); o_i(&o, C.bat_lo); o_kv(&o, "ratio", C.bat_ratio);
    o_kv(&o, "lowbat", lowbat); o_s(&o, mode == 'C' ? " mode=C" : " mode=V");
    o_s(&o, cfg_from_flash ? " cfg=flash#" : " cfg=factory#"); o_i(&o, cfg_seq);
    o_kv(&o, "calib", calib_ok()); o_s(&o, " geo="); o_i(&o, C.lx_um); o_c(&o, ','); o_i(&o, C.ly_um);
    o_s(&o, " K="); o_i(&o, C.mv_slow); o_c(&o, ','); o_i(&o, C.mv_min); o_c(&o, ','); o_i(&o, C.mv_stall_ms);
    o_c(&o, ','); o_i(&o, C.sag_mv); o_c(&o, ','); o_i(&o, C.guard_ms);
    o_kv(&o, "evdrop", (int32_t)ev_drops);
    o_send(&o);
    for (int w = 0; w < NW; w++) {
        o_s(&o, "wh "); o_i(&o, w);
        o_s(&o, " G="); o_i(&o, C.kv[w]); o_c(&o, ','); o_i(&o, C.ks[w]); o_c(&o, ','); o_i(&o, C.kp[w]); o_c(&o, ',');
        o_i(&o, C.ki[w]);
        o_s(&o, C.kv[w] <= 0 ? " loop=open" : (clf_mask & (1u << w)) ? " loop=FAULT" : " loop=closed");
        o_kv(&o, "spd", (int32_t)spd[w]);
        if (C.cpr[w]) o_kv(&o, "rpm", (int32_t)(spd[w] * 60.f / (float)C.cpr[w]));
        o_s(&o, " N="); o_i(&o, C.cpr[w]); o_c(&o, ','); o_i(&o, C.dia_um[w]);
        o_send(&o);
    }
    print_boot();
}

static void heartbeat(uint32_t now) {
    out_t o = { .n = 0 };
    int32_t v[3] = { cur[0], cur[1], cur[2] }, e[NW], a[NW], s[NW];
    uint32_t tot = 0;
    for (int i = 0; i < NW; i++) { e[i] = enc_acc[i]; a[i] = (int32_t)amp_ma[i]; tot += amp_ma[i]; s[i] = (int32_t)spd[i]; }
    o_s(&o, "hb "); o_u(&o, now); o_kv(&o, "rx", (int32_t)rx_ok); o_kv(&o, "bad", (int32_t)rx_bad);
    o_s(&o, " i2c=0"); o_kv(&o, "wdt", (int32_t)wdt_near); o_arr(&o, "v", v, 3); o_arr(&o, "enc", e, NW);
    o_kv(&o, "vin", (int32_t)bat_mv); o_kv(&o, "vm", (int32_t)vm_mv); o_kv(&o, "v5", (int32_t)v5_mv);
    o_kv(&o, "v33", (int32_t)vdda_mv);
    o_kv(&o, "vmin", v5_min_mv == 0xFFFFFFFFu ? (int32_t)v5_mv : (int32_t)v5_min_mv); o_kv(&o, "sag", (int32_t)sag_count);
    o_kv(&o, "amp", (int32_t)tot); o_arr(&o, "ampw", a, NW); o_kv(&o, "flt", flt_mask); o_kv(&o, "fltn", (int32_t)flt_count);
    o_arr(&o, "spd", s, NW); o_kv(&o, "estop", estop_state()); o_kv(&o, "lowbat", lowbat); o_kv(&o, "clf", clf_mask);
    o_s(&o, " rly="); o_c(&o, rly_state());
    // 계약 §7 상태 비트
    uint32_t st = (latch || estop ? 1u : 0u) | (soft_active ? 2u : 0u) | (hard_tripped ? 4u : 0u) |
                  (calib_ok() ? 8u : 0u) | (flt_mask ? 16u : 0u) | (sat_seen ? 32u : 0u) | (lowbat ? 64u : 0u);
    sat_seen = false;
    o_s(&o, " st="); o_x(&o, st, 2);
    if (calib_ok()) {
        if (mode == 'C') { int32_t t3[3] = { tgt[0], tgt[1], tgt[2] }; o_arr(&o, "tgt", t3, 3); }
        // 실측 차체 속도(REP-103) — 역기구학의 역
        float wv[NW];
        for (int i = 0; i < NW; i++) wv[i] = spd[i] * mm_per_count(i);   // RR RL FR FL
        float vx = (wv[0] + wv[1] + wv[2] + wv[3]) / 4.f;
        float vyV = (-wv[0] + wv[1] + wv[2] - wv[3]) / 4.f;           // V 믹싱 기준(+오른쪽)
        float kw = (wv[0] - wv[1] + wv[2] - wv[3]) / 4.f;
        float k = (C.lx_um + C.ly_um) / 1000.f;
        int32_t act[3] = { (int32_t)vx, (int32_t)(-vyV), (int32_t)(kw / k * 180000.f / PI_F) };
        o_arr(&o, "act", act, 3);
    }
    if (mv_state == MV_RUN) { o_s(&o, " mv="); o_u(&o, mv_id); o_c(&o, ','); o_i(&o, mv_progress()); }
    o_kv(&o, "txd", (int32_t)hw_tx_drops());
    o_send(&o);
    v5_min_mv = 0xFFFFFFFFu;
}

static void emit_events(void) {
    while (ev_tail != ev_head) {
        ev_t e = evq[ev_tail];
        ev_tail = (ev_tail + 1u) % EVQ;
        out_t o = { .n = 0 };
        switch (e.type) {
        case EV_MV_DONE: o_s(&o, "mv done "); o_i(&o, e.a); o_c(&o, ' '); o_i(&o, e.b); o_c(&o, ' '); o_i(&o, e.c);
            o_c(&o, ' '); o_i(&o, e.d); break;
        case EV_MV_STALL: o_s(&o, "mv stall "); o_i(&o, e.a); o_c(&o, ' '); o_i(&o, e.b); o_c(&o, ' '); o_i(&o, e.c);
            o_s(&o, " wheel="); o_c(&o, (char)('A' + e.arg)); o_kv(&o, "sag", e.d); break;
        case EV_MV_ABORT: o_s(&o, "mv abort "); o_i(&o, e.a); o_c(&o, ' '); o_i(&o, e.b); o_c(&o, ' '); o_i(&o, e.c);
            o_c(&o, ' '); o_s(&o, abort_name(e.arg)); break;
        case EV_CLFAULT: o_s(&o, "clfault "); o_i(&o, e.arg); break;
        case EV_DEADMAN: o_s(&o, "deadman HARD"); break;
        case EV_TEST: o_s(&o, "T auto-stop enc="); o_i(&o, e.a); break;
        case EV_GUARD: o_s(&o, "guard "); o_i(&o, e.arg); o_s(&o, " (막힘 — 바퀴가 안 도는데 듀티 50% 넘게 밀었다)"); break;
        default: continue;
        }
        o_send(&o);
    }
}

// ---------------- 명령 ----------------
static void handle_line(char *line) {
    hw_wdt_kick();                        // 명령 하나마다 워치독 예산(계약 §10.3)
    hw_phase(PH_RX);
    uint32_t now = hw_ms();
    out_t o = { .n = 0 };
    int crc = verify_crc(line);
    if (crc < 0) { rx_bad++; say("nak crc"); return; }
    char c = line[0];
    bool drive = (c == 'V' || c == 'C' || c == 'M');
    if (crc > 0) strict_crc = true;
    else if (strict_crc && drive) { rx_bad++; say("nak nocrc"); return; }
    if (!c) { rx_bad++; return; }
    if (line[1] && line[1] != ' ') { rx_bad++; o_s(&o, "nak unsupported "); o_s(&o, line); o_send(&o); return; }
    int32_t a[8];
    int n = parse_args(line + 1, a, 8);
    if (n < 0) { rx_bad++; o_s(&o, "nak range "); o_c(&o, c); o_send(&o); return; }
    rx_ok++;
    last_cmd_ms = now;

    hw_crit_enter();
    switch (c) {
    case 'V': case 'C': {
        if (n != 3) { o_s(&o, "nak range "); o_c(&o, c); break; }
        if (!drivers_allowed()) { nak_limited(0, now, &o, "nak estop"); break; }
        if (lowbat) { nak_limited(1, now, &o, "nak lowbat"); break; }
        if (guard_hold) {
            if (a[0] || a[1] || a[2]) { nak_limited(3, now, &o, "nak guard (막혀서 섰다 — 0 지령이나 S로 확인)"); break; }
            guard_hold = false;
        }
        if (c == 'C') {
            if (!calib_ok()) { nak_limited(2, now, &o, "nak nocalib"); break; }
            int32_t vm = vmax_mms(), wm = wmax_mdegs();
            if (labs(a[0]) > vm || labs(a[1]) > vm || labs(a[2]) > wm) { o_s(&o, "nak range C"); break; }
        } else { a[0] = clamp255(a[0]); a[1] = clamp255(a[1]); a[2] = clamp255(a[2]); }
        mv_abort(c == 'V' ? AB_V : AB_C);
        if (testing) hard_stop();
        if (mode != c) { cur[0] = cur[1] = cur[2] = 0; mode = c; }   // 단위가 바뀐다 — 0에서 다시 슬루
        tgt[0] = a[0]; tgt[1] = a[1]; tgt[2] = a[2];
        break;
    }
    case 'H': break;                      // 살아 있음 — 데드맨만 채운다(M 도중)
    case 'S': mv_abort(AB_S); hard_stop(); guard_hold = false; o_s(&o, "ok S"); break;   // E-stop 래치는 안 푼다(사람 확인 = X 0)
    case 'X':
        if (n == 1 && a[0] == 1) { latch = true; mv_abort(AB_X); hard_stop(); upd_sleep(); o_s(&o, "ok X 1"); }
        else if (n == 1 && a[0] == 0) {
            if (weld) o_s(&o, "nak weld");
            else if (estop || !hw_estop_ok()) o_s(&o, "nak estop held");
            else { latch = false; hw_estop_isr_clear(); upd_sleep(); o_s(&o, "ok X 0"); }
        } else if (n == 0) { o_s(&o, "ok X "); o_i(&o, estop_state()); }
        else o_s(&o, "nak range X");
        break;
    case 'T':
        if (n != 2 || a[0] < 0 || a[0] >= NW) { o_s(&o, "nak range T"); break; }
        if (!drivers_allowed()) { o_s(&o, "nak estop"); break; }
        mv_abort(AB_T);
        hard_stop();
        test_wheel = a[0];
        test_enc0 = enc_acc[a[0]];
        wtarget[a[0]] = a[1] >= 0 ? 1.f : -1.f;
        test_until = now + TEST_MS;
        testing = true;
        o_s(&o, "ok T "); o_i(&o, a[0]); o_c(&o, ' '); o_i(&o, a[1]);
        break;
    case 'P':
        if (n == 1 && a[0] >= 1 && a[0] <= 4095) C.max_pwm = a[0];
        o_s(&o, "ok P "); o_i(&o, C.max_pwm);
        break;
    case 'L':
        if (n != 4 && n != 8) { o_s(&o, "nak range L"); break; }
        for (int i = 0; i < NW; i++) C.pol[i] = a[i] < 0 ? -1 : 1;
        if (n == 8) for (int i = 0; i < NW; i++) C.encdir[i] = a[4 + i] < 0 ? -1 : 1;
        mv_abort(AB_S);
        hard_stop();
        o_s(&o, "ok L POL="); for (int i = 0; i < NW; i++) { o_i(&o, C.pol[i]); o_c(&o, ','); }
        o_s(&o, " ENC="); for (int i = 0; i < NW; i++) { o_i(&o, C.encdir[i]); o_c(&o, ','); }
        break;
    case 'R':
        if (n >= 1 && a[0] > 0 && a[0] <= 255) C.accel = a[0];
        if (n >= 2 && a[1] > 0 && a[1] <= 255) C.decel = a[1];
        o_s(&o, "ok R "); o_i(&o, C.accel); o_c(&o, ' '); o_i(&o, C.decel);
        break;
    case 'F':
        if (n == 1 && a[0] >= 24 && a[0] <= 40000) {
            hw_phase(PH_SETHZ);
            C.pwm_hz = (int32_t)hw_pwm_set_hz((uint32_t)a[0]);
            hw_phase(PH_IDLE);
        }
        o_s(&o, "ok F "); o_i(&o, C.pwm_hz);
        break;
    case 'B':
        if (n != 2 || a[0] <= 0 || a[1] < 0) { o_s(&o, "nak range B"); break; }
        C.bat_ratio = a[0]; C.bat_lo = a[1]; bat_n = 0;
        o_s(&o, "ok B "); o_i(&o, C.bat_ratio); o_c(&o, ' '); o_i(&o, C.bat_lo);
        break;
    case 'G': {
        if (n != 5 || a[0] < -1 || a[0] >= NW || a[1] < 0 || a[2] < 0 || a[3] < 0 || a[4] < 0) { o_s(&o, "nak range G"); break; }
        for (int w = 0; w < NW; w++) {
            if (a[0] >= 0 && w != a[0]) continue;
            C.kv[w] = a[1]; C.ks[w] = a[2]; C.kp[w] = a[3]; C.ki[w] = a[4];
            integ[w] = 0; bad_ms[w] = 0; clf_mask &= (uint8_t)~(1u << w);   // 새 게인 = 폭주 판정도 새로
        }
        mix();
        o_s(&o, "ok G "); for (int i = 0; i < 5; i++) { if (i) o_c(&o, ' '); o_i(&o, a[i]); }
        break;
    }
    case 'N':
        if (n != 3 || a[0] < -1 || a[0] >= NW || a[1] < 0 || a[2] < 0) { o_s(&o, "nak range N"); break; }
        for (int w = 0; w < NW; w++) if (a[0] < 0 || w == a[0]) { C.cpr[w] = a[1]; C.dia_um[w] = a[2]; }
        o_s(&o, "ok N "); o_i(&o, a[0]); o_c(&o, ' '); o_i(&o, a[1]); o_c(&o, ' '); o_i(&o, a[2]);
        break;
    case 'M':
        if (n == 2) {                     // 계약 §5.2: 기하
            if (a[0] < 0 || a[1] < 0) { o_s(&o, "nak range M"); break; }
            C.lx_um = a[0]; C.ly_um = a[1];
            o_s(&o, "ok M "); o_i(&o, C.lx_um); o_c(&o, ' '); o_i(&o, C.ly_um);
        } else if (n == 5) {              // Uno v2.3: 거리 이동
            if (!drivers_allowed()) { o_s(&o, "nak mv estop"); break; }
            if (lowbat) { o_s(&o, "nak mv lowbat"); break; }
            if (guard_hold) { o_s(&o, "nak mv guard"); break; }
            mv_start((uint32_t)a[0], a[1], clamp255(a[2]), clamp255(a[3]), clamp255(a[4]), &o);
        } else o_s(&o, "nak range M");
        break;
    case 'K':
        if (n < 5 || n > 6) { o_s(&o, "nak range K"); break; }
        if (a[0] > 0) C.mv_slow = a[0];
        if (a[1] > 0 && a[1] <= 255) C.mv_min = a[1];
        if (a[2] > 0) C.mv_stall_ms = a[2];
        if (a[3] > 0) C.sag_mv = a[3];
        if (n == 6 && a[5] >= 0) C.guard_ms = a[5];
        o_s(&o, "ok K "); o_i(&o, C.mv_slow); o_c(&o, ' '); o_i(&o, C.mv_min); o_c(&o, ' '); o_i(&o, C.mv_stall_ms);
        o_c(&o, ' '); o_i(&o, C.sag_mv); o_s(&o, " 0 "); o_i(&o, C.guard_ms);
        break;
    case 'E':
        if (cur[0] || cur[1] || cur[2] || testing || mv_state != MV_IDLE) { o_s(&o, "nak busy E (바퀴가 서 있을 때만)"); break; }
        hw_crit_exit();
        hw_phase(PH_FLASH);
        if (n == 1 && a[0] == 0) {
            bool ok = hw_cfg_erase();
            hw_crit_enter();
            core_cfg_defaults(&C);
            hw_pwm_set_hz((uint32_t)C.pwm_hz);
            cfg_from_flash = false; cfg_seq = -1;
            o_s(&o, ok ? "ok E 0" : "nak flash");
        } else if (n == 0) {
            int32_t seq = hw_cfg_save(&C);
            hw_crit_enter();
            if (seq < 0) o_s(&o, "nak flash");
            else { cfg_from_flash = true; cfg_seq = seq; o_s(&o, "ok E "); o_i(&o, seq); o_kv(&o, "calib", calib_ok()); }
        } else { hw_crit_enter(); o_s(&o, "nak range E"); }
        hw_phase(PH_IDLE);
        last_cmd_ms = hw_ms();
        break;
    case 'Q': hw_crit_exit(); print_cap(); return;
    case '?': hw_crit_exit(); print_status(); return;
    case 'W':
        mv_abort(AB_S);
        hard_stop();
        hw_drv_enable(false);
        hw_crit_exit();
        say("ok W (일부러 멈춥니다 — 워치독이 500ms 안에 살려야 정상)");
        hw_phase(PH_TEST);
        hw_hang_for_wdt();
        return;
    case 'U':
        mv_abort(AB_S);
        hard_stop();
        hw_drv_enable(false);
        hw_crit_exit();
        say("ok U (시스템 부트로더 — stm32flash로 굽고 -R로 돌아옴)");
        hw_reboot_bootloader();
        return;
    default:
        hw_crit_exit();
        rx_ok--; rx_bad++;
        o_s(&o, "nak unsupported "); o_c(&o, c); o_send(&o);
        return;
    }
    hw_crit_exit();
    hw_phase(PH_IDLE);
    if (o.n) o_send(&o);
}

// ---------------- 느린 감시(main) ----------------
static void estop_poll(uint32_t now) {
    bool raw = !hw_estop_ok();
    if (raw != estop_raw) { estop_raw = raw; estop_edge = now; }
    bool stable = now - estop_edge >= ESTOP_DEBOUNCE_MS;
    if (stable && raw != estop) {
        hw_crit_enter();
        estop = raw;
        if (estop) { latch = true; mv_abort(AB_ESTOP); hard_stop(); }
        hw_crit_exit();
        say(estop ? "estop 1" : "estop 0");
    }
    if (hw_estop_isr_hit() && stable && !estop && !latch) {
        // 인터럽트는 섰는데 20ms 뒤에도 안 눌려 있다 = 잡음. 0에서 다시(지령은 젯슨이 다시 채운다)
        hw_crit_enter(); hard_stop(); hw_crit_exit();
        hw_estop_isr_clear();
    }
    if (drv_ok != drivers_allowed()) { hw_crit_enter(); if (!drivers_allowed()) hard_stop(); upd_sleep(); hw_crit_exit(); }
}

static void relay_check(void) {
    if (bat_mv < RLY_MIN_VIN_MV || bat_n < 10u) { rly_bad_n = 0; return; }
    bool vm_on = vm_mv * 2u > bat_mv;
    // 눌림인데 VM이 배터리에 붙어 있다 = 접점 용착. 기준을 배터리의 85%로 — 차체를 손으로 밀면 드라이버 몸체
    // 다이오드가 역기전력을 VM으로 올리는데(기판 검토 11/09), 그건 배터리 전압 근처까지는 못 간다.
    bool bad = estop ? (vm_mv * 100u > bat_mv * 85u) : (!vm_on && bat_mv > (uint32_t)C.bat_lo);
    if (!bad) {
        rly_bad_n = 0;
        if (rly_open) { rly_open = false; say("rlyopen 0"); }
        return;
    }
    if (rly_bad_n < 255u) rly_bad_n++;
    if (rly_bad_n < (estop ? WELD_HOLD_N : RLY_HOLD_N)) return;
    if (estop && !weld) {                 // 접점이 붙었다 — 하드웨어 E-stop이 안 듣는다. 전원을 껐다 켜야 풀린다
        hw_crit_enter(); weld = true; hard_stop(); upd_sleep(); hw_crit_exit();
        say("weld 1");
    }
    if (!estop && !rly_open) { rly_open = true; say("rlyopen 1"); }
}

static void slow_100ms(uint32_t now) {
    adc_t ad;
    hw_adc_take(&ad);
    if (!ad.valid) return;
    vdda_mv = ad.vdda_mv;
    bat_buf[bat_i] = (uint32_t)((uint64_t)ad.vbat_pin_mv * (uint32_t)C.bat_ratio / 1000u);
    bat_i = (bat_i + 1u) % 10u;
    if (bat_n < 10u) bat_n++;
    uint32_t sum = 0;
    for (uint32_t i = 0; i < bat_n; i++) sum += bat_buf[i];
    bat_mv = sum / bat_n;
    vm_mv = (uint32_t)((uint64_t)ad.vm_pin_mv * (uint32_t)C.bat_ratio / 1000u);
    v5_mv = ad.v5_pin_mv * V5_RATIO / 1000u;
    uint32_t v5min = ad.v5_min_pin_mv * V5_RATIO / 1000u;
    if (v5min < v5_min_mv) v5_min_mv = v5min;
    if (C.sag_mv && v5min < (uint32_t)C.sag_mv) {    // 로직 5V 처짐 — 100ms 동안 가속 금지(Uno v2.3과 같은 뜻)
        if ((int32_t)(now - sag_until) >= 0) sag_count++;
        sag_until = now + 100u;
    }
    for (int i = 0; i < NW; i++) amp_ma[i] = ad.ipropi_mv[i] * IPROPI_MA_PER_V / 1000u;
    relay_check();
    // 저전압: 1초 평균이 문턱 아래 2초 → 감속 정지 후 V/C/M 거절, 문턱+0.3V 위로 풀림
    if (!C.bat_lo) { if (lowbat) { lowbat = false; say("lowbat 0"); } low_since = now; return; }
    if (bat_n < 10u) { low_since = now; return; }
    if (!lowbat) {
        if (bat_mv >= (uint32_t)C.bat_lo) low_since = now;
        else if (now - low_since >= BAT_HOLD_MS) {
            hw_crit_enter(); lowbat = true; mv_abort(AB_LOWBAT); tgt[0] = tgt[1] = tgt[2] = 0; hw_crit_exit();
            say("lowbat 1");
        }
    } else if (bat_mv > (uint32_t)C.bat_lo + BAT_HYST_MV) { lowbat = false; low_since = now; say("lowbat 0"); }
}

static void fault_poll(void) {
    uint8_t m = hw_nfault_mask();
    for (uint8_t rise = (uint8_t)(m & ~flt_mask); rise; rise >>= 1) flt_count += rise & 1u;
    flt_mask = m;
    hw_led(1, m != 0 || weld || latch || estop);
}

// ---------------- 진입점 ----------------
void core_init(const boot_info_t *bi) {
    BI = *bi;
    core_cfg_defaults(&C);
    cfg_t f;
    if (hw_cfg_load(&f) && core_cfg_valid(&f)) { C = f; cfg_from_flash = true; }
    hw_pwm_set_hz((uint32_t)C.pwm_hz);
    for (int w = 0; w < NW; w++) enc_last_raw[w] = hw_enc_raw(w);
    uint32_t now = hw_ms();
    last_cmd_ms = last_loop_ms = last_hb_ms = last_100_ms = low_since = estop_edge = now;
    zero_outputs();
    // 켜질 때 이미 눌려 있으면 래치·재운 채로 시작. 안 눌려 있으면 래치 없음(전원 투입 = 새 출발)
    estop = estop_raw = latch = !hw_estop_ok();
    upd_sleep();
    print_boot();
    if (BI.mv_lost) {                     // 이동 중 리셋 — 진행만 알린다. 스스로 이어 가지 않는다(Uno v2.3과 같다)
        out_t o = { .n = 0 };
        o_s(&o, "mv lost "); o_u(&o, BI.mv_id); o_c(&o, ' '); o_i(&o, BI.mv_got); o_c(&o, ' '); o_i(&o, BI.mv_target);
        o_send(&o);
    }
    hw_mv_record(0, 0, 0);
    say("mecanum_stable v" FW_VERSION "-stm32 ready. proto: V vx vy w | C vx vy w | S | X 0/1 | T i d | P n | "
        "L a b c d [ea eb ec ed] | R a d | F hz | B r lo | G i kv ks kp ki | N i cpr dia | M lx ly | "
        "M id t vx vy w | H | K slow min stall sag bg [guard] | E [0] | W | Q | U | ? (+ XOR 체크섬)");
    // ⚠ 보드가 내보내는 줄엔 '*'를 쓰지 않는다 — ROS board_contract가 '*' 뒤를 체크섬으로 읽어 nak crc로 센다(11/09 파서 대조)
    print_cap();
    if (estop) say("estop 1");
}

void core_poll(void) {
    static uint32_t alive_seen;
    uint32_t now = hw_ms();
    if (now - last_loop_ms > 200u) wdt_near++;   // 리셋까진 안 갔지만 아슬아슬했다
    last_loop_ms = now;
    // 워치독은 main과 제어 인터럽트가 **둘 다** 살아 있을 때만 준다
    if (isr_alive != alive_seen) { alive_seen = isr_alive; hw_wdt_kick(); }

    int ch;
    while ((ch = hw_rx()) >= 0) {
        if (ch == '\n' || ch == '\r') {
            if (rx_overflow) { rx_bad++; say("nak range long"); }
            else if (rxlen) { rxbuf[rxlen] = 0; handle_line(rxbuf); }
            rxlen = 0; rx_overflow = false;
        } else if (rxlen < sizeof rxbuf - 1u) rxbuf[rxlen++] = (char)ch;
        else rx_overflow = true;
    }
    estop_poll(now);
    fault_poll();
    emit_events();
    if (now - last_100_ms >= 100u) { last_100_ms = now; slow_100ms(now); }
    if (now - last_hb_ms >= 1000u) {
        last_hb_ms = now;
        heartbeat(now);
        static bool led;
        led = !led;
        hw_led(0, led);
    }
    hw_phase(PH_IDLE);
}

void core_view(core_view_t *v) {
    for (int i = 0; i < 3; i++) { v->cur[i] = cur[i]; v->tgt[i] = tgt[i]; }
    v->mode = mode; v->estop = estop; v->latch = latch; v->weld = weld; v->lowbat = lowbat; v->drv_ok = drv_ok;
    v->testing = testing; v->moving = mv_state != MV_IDLE; v->clf = clf_mask;
    for (int i = 0; i < NW; i++) { v->spd[i] = spd[i]; v->duty[i] = duty[i]; v->enc[i] = enc_acc[i]; }
    v->rx_ok = rx_ok; v->rx_bad = rx_bad; v->cfg = C;
}
