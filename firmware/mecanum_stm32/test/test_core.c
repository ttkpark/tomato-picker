// core 시험 — 시나리오마다 새 프로세스(정적 상태를 깨끗이). 인자 없이 돌리면 전부를 차례로 띄운다.
#include "fake_hw.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails;
static uint32_t mark;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void boot_with(boot_info_t bi) { fk_reset_world(); core_init(&bi); }
static void boot(void) {
    boot_info_t bi = { .boot_count = 1, .cause = "POWER", .clk_hse = true, .bor = 3, .uid = 0xA3F2C918u };
    boot_with(bi);
}
static void m(void) { mark = fk_out_n; }
static const char *seen(const char *s) { return strstr(fk_out + mark, s); }
static int count_of(const char *s) {
    int n = 0; const char *p = fk_out + mark;
    while ((p = strstr(p, s))) { n++; p++; }
    return n;
}
static core_view_t V(void) { core_view_t v; core_view(&v); return v; }
static float fwd(int w) { return fk_pos[w]; }

// ---------------- 시나리오 ----------------
static void t_boot(void) {
    boot();
    CHECK(strstr(fk_out, "boot #1 cold cause=POWER last=boot i2cerr=0 near=0 mcusr=0x00 r2=0x0"), "boot 줄: %s", fk_out);
    CHECK(strstr(fk_out, "mecanum_stable v4.0.0-stm32 ready."), "배너(motor_link BOOT_BANNER)");
    CHECK(!strchr(fk_out, '*'), "보드 출력에 '*' 없음(ROS 파서가 체크섬으로 읽는다)");
    CHECK(strstr(fk_out, "cap proto=2 fw=4.0.0 board=stm32f405 id=A3F2C918 drive=mecanum wheels=4 units=1"), "cap");
    CHECK(strstr(fk_out, " calib=0 "), "공장값엔 calib=0");
    CHECK(!fk_drv_en || V().drv_ok, "드라이버 상태 일관");
    m(); fk_step(1000);
    CHECK(seen("hb "), "1초에 hb");
    CHECK(seen(" rx=0 bad=0 i2c=0 wdt=0 v=0,0,0 enc=0,0,0,0 vin="), "hb 앞머리: %s", fk_out + mark);
    { const char *p = seen(" vin="); int vin = p ? atoi(p + 5) : 0; CHECK(vin > 11900 && vin < 12100, "vin %d", vin); }
    CHECK(seen(" estop=0") && seen(" rly=1") && seen(" st=02"), "hb 상태(조용 → 소프트 데드맨 비트1)");
    CHECK(fk_wdt_kicks > 900, "워치독은 ISR이 살아 있을 때 준다(%u)", fk_wdt_kicks);
}

static void t_boot_lost_fault(void) {
    boot_info_t bi = { .boot_count = 7, .warm = true, .cause = "BOD", .prev_phase = PH_PWM, .csr = 0x0A000000u,
                       .fault = true, .fault_pc = 0x08001234u, .fault_cfsr = 0x82u,
                       .mv_lost = true, .mv_id = 42, .mv_got = 300, .mv_target = 800, .clk_hse = true, .uid = 1 };
    boot_with(bi);
    CHECK(strstr(fk_out, "boot #7 warm cause=BOD last=pwm") && strstr(fk_out, " fault pc=0x08001234"), "warm·폴트: %s", fk_out);
    CHECK(strstr(fk_out, "mv lost 42 300 800\n"), "mv lost");
    char *b = strstr(fk_out, "boot #"), *l = strstr(fk_out, "mv lost"), *r = strstr(fk_out, "mecanum_stable");
    CHECK(b && l && r && b < l && l < r, "순서: boot → mv lost → 배너");
}

static void t_crc_and_parse(void) {
    boot();
    m(); fk_rx("V 50 0 0*00"); fk_step(2);
    CHECK(seen("nak crc"), "틀린 체크섬");
    CHECK(V().tgt[0] == 0, "틀린 체크섬은 실행 안 함");
    fk_rx_crc("V 50 0 0"); fk_step(2);
    CHECK(V().tgt[0] == 50, "맞는 체크섬 → 실행");
    m(); fk_rx("V 60 0 0"); fk_step(2);
    CHECK(seen("nak nocrc") && V().tgt[0] == 50, "strict 뒤 무체크섬 구동은 거절");
    m(); fk_rx("P 3000"); fk_step(2);
    CHECK(seen("ok P 3000"), "설정은 무체크섬도 받는다");
    m(); fk_rx_crc("V 1 2 x"); fk_step(2);
    CHECK(seen("nak range V"), "숫자 아닌 인자");
    m(); fk_rx_crc("Z"); fk_step(2);
    CHECK(seen("nak unsupported Z"), "모르는 명령은 조용히 무시하지 않는다");
    m(); fk_rx_crc("VV 1 2 3"); fk_step(2);
    CHECK(seen("nak unsupported VV"), "명령 글자 뒤 공백 필수");
    m(); char lng[200]; memset(lng, '1', sizeof lng); lng[0] = 'V'; lng[1] = ' '; lng[199] = 0; fk_rx(lng); fk_step(2);
    CHECK(seen("nak range long"), "긴 줄");
    m(); fk_rx_crc("V 99999999999 0 0"); fk_step(2);
    CHECK(seen("nak range V"), "10자리 숫자 넘침");
    m(); fk_rx_crc("V 900 -900 0"); fk_step(2);
    CHECK(V().tgt[0] == 255 && V().tgt[1] == -255, "±255로 자른다");
}

static void t_slew_and_mix(void) {
    boot();
    fk_rx_crc("V 120 0 0");
    fk_step(5);
    CHECK(V().cur[0] == 6, "5ms에 accel 6 (%d)", V().cur[0]);
    for (int i = 0; i < 20; i++) { fk_step(100); fk_rx_crc("V 120 0 0"); }
    CHECK(V().cur[0] == 120, "목표 도달");
    // 전진: 네 바퀴 다 앞으로(실물 극성으로)
    for (int w = 0; w < NW; w++) CHECK(fk_duty[w] * fk_pol_true[w] > 0, "바퀴 %d 전진 듀티 %d", w, fk_duty[w]);
    CHECK(abs(fk_duty[0]) == 120 * 2000 / 255, "개루프 듀티 = v·MAX_PWM/255 (%d)", fk_duty[0]);
    for (int w = 0; w < NW; w++) CHECK(fwd(w) > 100, "바퀴 %d 굴렀다 %.0f", w, fwd(w));
    float before = fwd(0);
    m(); fk_rx_crc("S"); fk_step(1);
    CHECK(seen("ok S") && V().cur[0] == 0 && fk_duty[0] == 0, "S = 즉시 0");
    fk_step(300);
    CHECK(fwd(0) - before < 60, "S 뒤 곧 선다");
}

static void t_deadman(void) {
    boot();
    fk_rx_crc("R 4 1");                          // 감속을 느리게 → 하드 데드맨까지 굴러가게
    for (int i = 0; i < 10; i++) { fk_step(100); fk_rx_crc("V 200 0 0"); }
    CHECK(V().cur[0] == 200, "달리는 중");
    m(); fk_step(295);
    CHECK(V().tgt[0] == 200, "300ms까지는 그대로");
    fk_step(15);
    CHECK(V().tgt[0] == 0 && V().cur[0] < 200, "300ms 넘으면 목표 0·감속");
    fk_step(700);
    CHECK(seen("deadman HARD") && V().cur[0] == 0 && fk_duty[1] == 0, "1000ms 넘으면 즉시 0 + 한 번 알림");
    fk_step(2000);
    CHECK(count_of("deadman HARD") == 1, "알림은 한 번");
}

static void t_estop(void) {
    boot();
    for (int i = 0; i < 5; i++) { fk_rx_crc("V 150 0 0"); fk_step(100); }
    CHECK(fk_duty[2] != 0, "달리는 중");
    // 버튼: 인터럽트가 출력·드라이버를 먼저 끊는다
    fk_estop_ok = false; fk_estop_hit = true; fk_drv_en = false;
    for (int w = 0; w < NW; w++) fk_duty[w] = 0;
    m(); fk_rx_crc("V 150 0 0"); fk_step(5);
    for (int w = 0; w < NW; w++) CHECK(fk_duty[w] == 0, "눌림 직후 출력 0 (바퀴 %d)", w);
    fk_step(30);
    CHECK(seen("estop 1") && V().latch, "20ms 뒤 estop 1·래치");
    fk_estop_ok = true; fk_step(30);
    CHECK(seen("estop 0") && V().latch, "놓아도 래치");
    m(); for (int i = 0; i < 20; i++) { fk_rx_crc("V 150 0 0"); fk_step(100); }
    CHECK(!fk_drv_en && fk_duty[0] == 0, "래치 중엔 안 움직인다");
    CHECK(count_of("nak estop") >= 1 && count_of("nak estop") <= 3, "nak estop은 초당 한 번(%d)", count_of("nak estop"));
    CHECK(seen(" estop=2"), "hb estop=2");
    m(); fk_rx_crc("S"); fk_step(5);
    CHECK(V().latch, "S로는 안 풀린다");
    m(); fk_rx_crc("X 0"); fk_step(5);
    CHECK(seen("ok X 0") && !V().latch && fk_drv_en, "X 0으로 풀린다");
    for (int i = 0; i < 5; i++) { fk_rx_crc("V 150 0 0"); fk_step(100); }
    CHECK(fk_duty[0] != 0, "풀린 뒤 다시 달린다");
    // 눌린 채 X 0
    fk_estop_ok = false; fk_estop_hit = true; fk_step(30);
    m(); fk_rx_crc("X 0"); fk_step(5);
    CHECK(seen("nak estop held"), "눌린 채 X 0 거절");
    // 잡음: 인터럽트만 서고 버튼은 안 눌림
    fk_estop_ok = true; fk_step(30); fk_rx_crc("X 0"); fk_step(5);
    fk_estop_hit = true; fk_step(30);
    CHECK(!fk_estop_hit && !V().latch, "잡음 인터럽트는 20ms 뒤 지운다");
    // 켜질 때 눌려 있으면
    fk_reset_world(); fk_estop_ok = false;
    boot_info_t bi = { .boot_count = 1, .cause = "POWER", .uid = 1 };
    core_init(&bi);
    CHECK(strstr(fk_out, "estop 1") && V().latch && !fk_drv_en, "켜질 때 눌림 → 래치");
}

static void t_weld(void) {
    boot();
    fk_estop_ok = false; fk_estop_hit = true; fk_step(1500);   // 릴레이가 끊었다, 평균이 찰 때까지
    fk_relay_weld = true;                                        // 눌렸는데 접점이 붙어 VM이 남아 있다
    m(); fk_step(700);
    CHECK(!seen("weld 1"), "1초 전에는 판정 안 함");
    fk_step(600);
    CHECK(seen("weld 1") && V().weld, "VM이 배터리에 붙어 있으면 weld");
    fk_estop_ok = true; fk_step(50);
    m(); fk_rx_crc("X 0"); fk_step(5);
    CHECK(seen("nak weld") && !fk_drv_en, "weld는 X 0으로도 안 풀린다");
    // 손으로 민 역기전력(배터리의 40%)은 weld가 아니다
    boot();
    fk_estop_ok = false; fk_estop_hit = true; fk_step(1500);
    fk_vm_mv = fk_vbat_mv * 4 / 10;
    m(); fk_step(3000);
    CHECK(!seen("weld 1"), "역기전력은 weld가 아니다");
}

static void t_rlyopen(void) {
    boot();
    fk_step(1500);
    fk_vbat_mv = 12000;
    // 코일은 붙었는데(버튼 정상) VM이 없다
    fk_estop_ok = true;
    m(); fk_relay_open = true; fk_step(600);
    CHECK(seen("rlyopen 1"), "rlyopen");
    m(); fk_relay_open = false; fk_step(300);
    CHECK(seen("rlyopen 0"), "접점이 돌아오면 rlyopen 0");
}

static void t_lowbat(void) {
    boot();
    fk_rx_crc("B 11000 9000");
    fk_step(1200);
    fk_vbat_mv = 8000;
    for (int i = 0; i < 30; i++) { fk_rx_crc("V 100 0 0"); fk_step(100); }
    CHECK(seen("lowbat 1") && V().lowbat && V().tgt[0] == 0, "저전압 → lowbat·목표 0");
    m(); for (int i = 0; i < 10; i++) { fk_rx_crc("V 100 0 0"); fk_step(100); }
    CHECK(seen("nak lowbat") && V().cur[0] == 0, "저전압 중 V 거절");
    fk_vbat_mv = 9400;
    for (int i = 0; i < 25; i++) { fk_rx_crc("V 0 0 0"); fk_step(100); }
    CHECK(seen("lowbat 0") && !V().lowbat, "문턱+300mV 위로 풀림");
}

static void t_test_wheel(void) {
    boot();
    m(); fk_rx_crc("T 2 1"); fk_step(5);
    CHECK(seen("ok T 2 1"), "ok T");
    CHECK(fk_duty[2] * fk_pol_true[2] > 0 && fk_duty[0] == 0, "시험 바퀴만 앞으로");
    fk_step(1100);
    const char *p = seen("T auto-stop enc=");
    CHECK(p && atoi(p + 16) > 200, "1초 뒤 자동 정지 + 증분 양수");
    CHECK(fk_duty[2] == 0, "멈췄다");
    fk_enc_true[3] = -1;                          // 엔코더 선이 뒤집혀 있다
    m(); fk_rx_crc("T 3 1"); fk_step(1100);
    p = seen("T auto-stop enc=");
    CHECK(p && atoi(p + 16) < -200, "부호가 틀리면 음수로 드러난다");
    m(); fk_rx_crc("L -1 1 -1 1 1 1 1 -1"); fk_step(2);
    CHECK(seen("ok L POL=-1,1,-1,1, ENC=1,1,1,-1,"), "L로 엔코더 부호를 고친다");
    m(); fk_rx_crc("T 3 1"); fk_step(1100);
    p = seen("T auto-stop enc=");
    CHECK(p && atoi(p + 16) > 200, "고친 뒤 양수");
}

static void t_move(void) {
    boot();
    m(); fk_rx_crc("M 7 800 150 0 0"); fk_step(5);
    CHECK(seen("ok M 7 800"), "ok M");
    for (int i = 0; i < 60 && !seen("mv done"); i++) { fk_rx_crc("H"); fk_step(100); }
    const char *p = seen("mv done 7 ");
    CHECK(p, "mv done");
    if (p) { int got = atoi(p + 10); CHECK(got >= 800 && got < 950, "거리(%d)", got); }
    CHECK(V().cur[0] == 0 && !V().moving, "끝나면 정지");
    // 걸림
    m(); for (int w = 0; w < NW; w++) fk_blocked[w] = true;
    fk_rx_crc("M 8 800 150 0 0");
    for (int i = 0; i < 20 && !seen("mv stall"); i++) { fk_rx_crc("H"); fk_step(100); }
    CHECK(seen("mv stall 8 ") && seen(" wheel="), "걸리면 stall + 바퀴");
    for (int w = 0; w < NW; w++) fk_blocked[w] = false;
    // V가 오면 취소
    m(); fk_rx_crc("M 9 5000 150 0 0"); fk_step(300); fk_rx_crc("V 0 0 0"); fk_step(10);
    CHECK(seen("mv abort 9 ") && seen(" V\n"), "V가 이동을 취소");
    // 지령이 끊기면(H 없음) 데드맨이 취소
    m(); fk_rx_crc("M 10 5000 150 0 0"); fk_step(400);
    CHECK(seen("mv abort 10 ") && seen(" deadman"), "H가 끊기면 데드맨 취소");
    m(); fk_rx_crc("M 0 100 150 0 0"); fk_rx_crc("M 11 100 0 0 0"); fk_step(3);
    CHECK(seen("nak mv args") && seen("nak mv blind"), "인자 검사");
    m(); fk_rx_crc("M 12 4000 150 0 0");
    for (int i = 0; i < 12; i++) { fk_rx_crc("H"); fk_step(100); }
    CHECK(seen(" mv=12,"), "이동 중 hb에 mv=id,got");
}

static void t_guard(void) {
    boot();
    fk_rx_crc("P 4095");
    for (int w = 0; w < NW; w++) fk_blocked[w] = true;
    m(); for (int i = 0; i < 25; i++) { fk_rx_crc("V 255 0 0"); fk_step(100); }
    CHECK(seen("guard "), "막힌 채 듀티 50%% 넘게 1.5초 → guard");
    CHECK(V().cur[0] == 0 && fk_duty[0] == 0, "guard 뒤 젯슨이 V를 계속 보내도 정지");
    CHECK(count_of("\nguard ") == 1 && seen("nak guard"), "한 번만 서고, 계속 오는 V는 nak guard");
    for (int w = 0; w < NW; w++) fk_blocked[w] = false;
    fk_rx_crc("V 0 0 0"); fk_step(5);
    for (int i = 0; i < 5; i++) { fk_rx_crc("V 120 0 0"); fk_step(100); }
    CHECK(V().cur[0] > 0, "0 지령을 한 번 받으면 다시 간다");
}

static void gains(void) {
    // 실물: 4095 → 5000cps, 정지마찰 12% → ks ≈ 0.12·4095 = 491, kv = 0.819
    fk_rx_crc("G -1 819 500000 300 3000");
    fk_rx_crc("N -1 1320 78600");
    fk_rx_crc("M 95000 85000");
    fk_rx_crc("P 4095");
}

static void t_closed_loop(void) {
    boot();
    gains();
    fk_step(5);
    for (int i = 0; i < 30; i++) { fk_rx_crc("V 128 0 0"); fk_step(100); }
    core_view_t v = V();
    float full = (4095.f - 500.f) / 0.819f, want = 128.f / 255.f * full;
    for (int w = 0; w < NW; w++)
        CHECK(fabsf(v.spd[w] - want) < 0.08f * want, "폐루프 바퀴 %d 속도 %.0f ≈ %.0f", w, v.spd[w], want);
    // 엔코더 부호가 틀린 바퀴는 0.5초 안에 개루프로 떨어진다
    m(); fk_enc_true[1] = -1;
    for (int i = 0; i < 20; i++) { fk_rx_crc("V 128 0 0"); fk_step(100); }
    CHECK(seen("clfault 1") && (V().clf & 2), "clfault");
    m(); fk_rx_crc("G 1 819 500000 300 3000"); fk_step(2);
    CHECK(!(V().clf & 2), "G를 다시 주면 풀린다");
}

static void t_units_C(void) {
    boot();
    m(); fk_rx_crc("C 100 0 0"); fk_step(5);
    CHECK(seen("nak nocalib"), "캘리브레이션 없이 C → nocalib");
    gains();
    m(); fk_rx_crc("Q"); fk_step(2);
    CHECK(seen(" calib=1 ") && seen(" vmax="), "calib=1 + vmax: %s", fk_out + mark);
    const char *vp = seen(" vmax=");
    int vmax = vp ? atoi(vp + 6) : 0;
    CHECK(vmax > 600 && vmax < 900, "vmax ≈ (4095−500)/0.819·0.187 = 821 (%d)", vmax);
    m(); fk_rx_crc("C 300 0 0");
    for (int i = 0; i < 30; i++) { fk_rx_crc("C 300 0 0"); fk_step(100); }
    const char *a = strstr(fk_out + mark, "act=");
    const char *last = a; while (a) { last = a; a = strstr(a + 1, "act="); }
    int ax = last ? atoi(last + 4) : 0;
    CHECK(ax > 270 && ax < 330, "C 300 → act.vx ≈ 300 (%d)", ax);
    CHECK(seen("tgt=300,0,0"), "tgt");
    // REP-103: +vy = 왼쪽. 이 차체의 V +vy는 오른쪽(계약 §14.1) → C +vy는 V −vy와 같은 바퀴 방향
    fk_rx_crc("S"); fk_step(5);
    for (int i = 0; i < 10; i++) { fk_rx_crc("C 0 200 0"); fk_step(100); }
    int cs[NW]; for (int w = 0; w < NW; w++) cs[w] = fk_duty[w] > 0 ? 1 : fk_duty[w] < 0 ? -1 : 0;
    fk_rx_crc("S"); fk_step(5);
    for (int i = 0; i < 10; i++) { fk_rx_crc("V 0 -100 0"); fk_step(100); }
    for (int w = 0; w < NW; w++) CHECK(cs[w] != 0 && cs[w] == (fk_duty[w] > 0 ? 1 : -1), "C +vy == V −vy (바퀴 %d)", w);
    // 회전: C +w(반시계) == V +w
    fk_rx_crc("S"); fk_step(5);
    for (int i = 0; i < 10; i++) { fk_rx_crc("C 0 0 30000"); fk_step(100); }
    for (int w = 0; w < NW; w++) cs[w] = fk_duty[w] > 0 ? 1 : fk_duty[w] < 0 ? -1 : 0;
    fk_rx_crc("S"); fk_step(5);
    for (int i = 0; i < 10; i++) { fk_rx_crc("V 0 0 100"); fk_step(100); }
    for (int w = 0; w < NW; w++) CHECK(cs[w] != 0 && cs[w] == (fk_duty[w] > 0 ? 1 : -1), "C +w == V +w (바퀴 %d)", w);
    m(); fk_rx_crc("C 5000 0 0"); fk_step(2);
    CHECK(seen("nak range C"), "vmax 밖은 range");
}

static void t_cfg_flash(void) {
    for (int s = 0; s < 2; s++) for (uint32_t i = 0; i < FAKE_SEC_WORDS; i++) fk_flash[s][i] = 0xFFFFFFFFu;
    boot();
    gains();
    fk_rx_crc("R 9 15");
    m(); fk_rx_crc("E"); fk_step(2);
    CHECK(seen("ok E 1") && seen(" calib=1"), "E 저장: %s", fk_out + mark);
    fk_rx_crc("V 100 0 0"); fk_step(50);
    m(); fk_rx_crc("E"); fk_step(2);
    CHECK(seen("nak busy E"), "움직이는 중엔 저장 거절");
    fk_rx_crc("S"); fk_step(2);
    boot();                                       // 재부팅
    m(); fk_rx_crc("?"); fk_step(2);
    CHECK(seen(" slew=9/15") && seen("cfg=flash#1") && seen(" calib=1"), "재부팅 뒤 복원: %s", fk_out + mark);
    m(); fk_rx_crc("E 0"); fk_step(2);
    CHECK(seen("ok E 0"), "E 0");
    boot();
    m(); fk_rx_crc("?"); fk_step(2);
    CHECK(seen(" slew=6/12") && seen("cfg=factory"), "E 0 뒤 공장값");
}

static void t_cfgstore_power_loss(void) {
    for (int s = 0; s < 2; s++) for (uint32_t i = 0; i < FAKE_SEC_WORDS; i++) fk_flash[s][i] = 0xFFFFFFFFu;
    cfg_t a, b;
    core_cfg_defaults(&a);
    int32_t last = 0;
    // 섹터 2KB에 기록 ~0.4KB → 몇 번마다 넘어간다. 매번 앞의 것이 남아 있는지 본다
    for (int i = 0; i < 40; i++) {
        a.max_pwm = 1000 + i;
        int32_t seq = cfgstore_save(&fk_io, &a, sizeof a);
        CHECK(seq == last + 1, "seq %d", seq);
        last = seq;
        CHECK(cfgstore_load(&fk_io, &b, sizeof b) == seq && b.max_pwm == 1000 + i, "읽기 %d", i);
    }
    // 쓰는 도중 전원이 나간다 — 어느 워드에서 끊겨도 옛 기록이 읽혀야 한다
    int words = (int)((sizeof a + 3) / 4 + 4);
    for (int cut = 0; cut < words + 2; cut++) {
        int32_t before = cfgstore_load(&fk_io, &b, sizeof b);
        int keep = b.max_pwm;
        a.max_pwm = 3000 + cut;
        fk_fail_after = cut;
        int32_t seq = cfgstore_save(&fk_io, &a, sizeof a);
        fk_fail_after = -1;
        int32_t now = cfgstore_load(&fk_io, &b, sizeof b);
        if (seq < 0) CHECK(now == before && b.max_pwm == keep, "끊김 %d: 옛 기록 유지 (%d→%d, %d)", cut, before, now, b.max_pwm);
        else CHECK(now == seq && b.max_pwm == 3000 + cut, "끊김 %d: 다 썼으면 새 기록", cut);
        // 다음 저장은 정상적으로 된다(찢긴 자리는 건너뛴다)
        a.max_pwm = 4000 + cut;
        int32_t s2 = cfgstore_save(&fk_io, &a, sizeof a);
        CHECK(s2 > 0 && cfgstore_load(&fk_io, &b, sizeof b) == s2 && b.max_pwm == 4000 + cut, "끊김 %d 뒤 저장", cut);
    }
    // 기록 하나가 깨지면(비트 반전) 그 앞의 것
    int32_t s3 = cfgstore_save(&fk_io, &a, sizeof a);
    for (int s = 0; s < 2; s++) for (uint32_t i = 0; i + 3 < FAKE_SEC_WORDS; i++)
        if (fk_flash[s][i] == 0x4746434Du && (int32_t)fk_flash[s][i + 1] == s3) fk_flash[s][i + 8] ^= 0x10u;
    CHECK(cfgstore_load(&fk_io, &b, sizeof b) == s3 - 1, "CRC 틀린 최신 기록은 버린다");
    // 다른 판(len이 다름)의 기록은 무시
    CHECK(cfgstore_load(&fk_io, &b, sizeof b - 4) < 0, "길이가 다른 기록은 무시");
}

static void t_misc(void) {
    boot();
    m(); fk_rx_crc("K 300 120 400 0 0"); fk_step(2);
    CHECK(seen("ok K 300 120 400 4500 0 1500"), "K(0=그대로)");
    m(); fk_rx_crc("F 50000"); fk_step(2);
    CHECK(seen("ok F 20000"), "범위 밖 F는 그대로");
    m(); fk_rx_crc("F 16000"); fk_step(2);
    CHECK(seen("ok F 16000"), "F");
    m(); fk_rx_crc("X 1"); fk_step(2);
    CHECK(seen("ok X 1") && V().latch && !fk_drv_en, "X 1 = 소프트 래치");
    m(); fk_rx_crc("X"); fk_step(2);
    CHECK(seen("ok X 2"), "X 상태");
    fk_rx_crc("X 0"); fk_step(2);
    m(); fk_nfault = 4; fk_step(1000);
    CHECK(seen(" flt=4 fltn=1") && seen(" st=12"), "nFAULT → flt·st 비트4: %s", fk_out + mark);
    fk_nfault = 0;
    m(); fk_rx_crc("U"); fk_step(1);
    CHECK(fk_bootloader && seen("ok U"), "U");
    boot();
    m(); fk_rx_crc("W"); fk_step(1);
    CHECK(fk_hang && seen("ok W"), "W");
    // rx 카운터는 32비트 — 65535에서 되감기지 않는다(젯슨이 재부팅으로 오해)
    boot();
    for (int i = 0; i < 70000; i++) { fk_rx("H"); if (i % 500 == 499) fk_step(1); }
    fk_step(1000);
    CHECK(V().rx_ok >= 70000, "rx 32비트 — 65535에서 안 되감김 (%u)", V().rx_ok);
}

typedef struct { const char *name; void (*fn)(void); } sc_t;
static const sc_t SC[] = {
    { "boot", t_boot }, { "boot_lost_fault", t_boot_lost_fault }, { "crc_parse", t_crc_and_parse },
    { "slew_mix", t_slew_and_mix }, { "deadman", t_deadman }, { "estop", t_estop }, { "weld", t_weld },
    { "rlyopen", t_rlyopen }, { "lowbat", t_lowbat }, { "test_wheel", t_test_wheel }, { "move", t_move },
    { "guard", t_guard }, { "closed_loop", t_closed_loop }, { "units_C", t_units_C }, { "cfg_flash", t_cfg_flash },
    { "cfgstore_power_loss", t_cfgstore_power_loss }, { "misc", t_misc },
};
#define NSC (int)(sizeof SC / sizeof SC[0])

// 젯슨 파서 시험(check_jetson_parsers.py)이 읽을 출력 — 부팅·하트비트·이동·이벤트·캘리브레이션 뒤 C
static void t_dump(void) {
    boot();
    fk_step(1100);
    fk_rx_crc("M 3 600 150 0 0");
    for (int i = 0; i < 30 && !strstr(fk_out, "mv done"); i++) { fk_rx_crc("H"); fk_step(100); }
    fk_rx_crc("F 20000"); fk_rx_crc("P 2600"); fk_rx_crc("R 4 12");
    gains();
    for (int i = 0; i < 25; i++) { fk_rx_crc("C 250 0 0"); fk_step(100); }
    fk_estop_ok = false; fk_estop_hit = true; fk_step(1100);
    fk_estop_ok = true; fk_step(50); fk_rx_crc("X 0"); fk_step(1100);
    fk_rx_crc("Q"); fk_step(2);
    fwrite(fk_out, 1, fk_out_n, stdout);
}

int main(int argc, char **argv) {
    if (argc > 1 && !strcmp(argv[1], "dump")) { t_dump(); return fails ? 1 : 0; }
    if (argc > 1) {
        for (int i = 0; i < NSC; i++)
            if (!strcmp(argv[1], SC[i].name)) { SC[i].fn(); return fails ? 1 : 0; }
        return 2;
    }
    int bad = 0;
    for (int i = 0; i < NSC; i++) {
        char cmd[512];
        snprintf(cmd, sizeof cmd, "\"%s\" %s", argv[0], SC[i].name);
#ifdef _WIN32
        for (char *c = cmd; *c; c++) if (*c == '/') *c = 0x5C;   // 역슬래시 — cmd.exe는 ./a/b를 못 읽는다
#endif
        int rc = system(cmd);
        printf("%s %s\n", rc ? "FAIL" : "ok  ", SC[i].name);
        bad += rc != 0;
    }
    printf("%s: %d/%d 시나리오\n", bad ? "실패" : "통과", NSC - bad, NSC);
    return bad ? 1 : 0;
}
