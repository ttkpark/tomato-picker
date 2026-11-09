// tomato_case.scad — 토마토 로봇 케이스 v2 (2026-11-05): 넓은 몸통 + 각진 바퀴 여유 + 받침 기둥
// 스케치 = docs/design/case-sketch-2026-11-05.png · 옛 판(151.9 폭, 젯슨 밑 벅부스트) = git 이력
// 값 출처 docs/design/3D-parts-mapping.md
//
// 좌표: X=좌우, Y=앞뒤, Z=위. Y=0 면이 전면(Astra), X=0 면이 왼쪽. 원점 = 트레이 바닥 밑면 앞-왼 모서리.
// 상판 윗면은 Z = −riser_h. 몸통 가운데 = 체결 패턴 두 벌의 가운데 = 회전판 중심.
//
// 안쪽 배치(위에서 본 것):
//   앞   : 제어 기판 100×70(Uno와 같은 구멍) — 위에 Astra 크래들
//   가운데: 젯슨 + 그 위 18mm 띄운 회전 서보·베어링 — 정중앙
//   왼쪽 : 배터리(세움), 앞-왼 벽에 전원 패널(전압계·OLED·스위치·충전 잭)
//   오른쪽: 벅부스트 2장(세로로 나란히), 젯슨과의 틈에 IMU, 앞-오른쪽에 예비 보스 칸
//   뒤   : 비움 — USB 케이블 칸(젯슨 단자가 이쪽). 뒷벽은 떼는 판(rear_cover)
//
// 부품별 STL: openscad -D 'part="tray"' -o tray.stl tomato_case.scad
//   part = assembly | inside | tray | lid | clamp | platform | astra_cap | riser | skirt | rear_cover | fit_test | clash
//   clash = 부품끼리 겹치는 곳만 남긴다. 비어 있어야 정상.
//
// 값의 출처: [STL] 3D/ 원본 · [DS] 데이터시트 · [자] 줄자·캘리퍼 실측
//            [추정] 사진에서 바퀴 지름 78.6으로 환산(11/05) — 출력 전 자로 잰다(README "출력 전 실측")

part = "assembly";

/* ===================== 공통 ===================== */
wall   = 3;      // [STL] 기존 파츠 벽·판 두께
floor_t = 3;     // [STL]
m3_clear = 3.4;  // [STL]
m3_tap   = 2.9;  // [STL] TOPLeft 아두이노 보스 2.93
m2_tap   = 1.8;
$fn = 48;
bed_x = 256; bed_y = 256;        // [자] 프린터 베드

/* ===================== 몸통 ===================== */
body_w = 252;                // 젯슨 양옆에 71.5씩 — 제어 기판을 돌려도 들어가는 폭. 베드 안
body_d = 250;                // 크래들 앞 돌출 3을 더해 253 ≤ 베드
tray_h = 100;                // 바닥 3 + 받침 12 + 젯슨 35 + 팬 위 공기 18 + 서보 32
corner_cut = 14;             // 세로 모서리 깎기 — 돌 때 가구에 덜 걸린다
cx = body_w/2;  cy = body_d/2;

/* ===================== 차체 체결 (확정) ===================== */
// 50×30 M3 패턴 두 벌. 30 변이 길이(Y) 방향 — 바깥나사 195.5 − 안쪽나사 136 = 59.5 ≈ 30+30.
plate_w = 151.9;             // [자] 차체 상판 가로(버니어). 몸통은 이보다 넓어 상판 옆으로 내려간다
mount_dx = 50;               // [STL] 가로(X)
mount_dy = 30;               // [STL] 길이(Y)
mount_gap = 136;             // [자] TOPLeft 뒷줄 나사 ↔ TOPRight 앞줄 나사 (PDF 8쪽)
mount_pitch = mount_dy + mount_gap;  // 166 = 두 패턴 중심 간격 (바깥나사 195.5와 0.5mm 차이로 검산됨)
mount_centers = [[cx, cy - mount_pitch/2], [cx, cy + mount_pitch/2]];

// 받침 기둥 — 스케치의 네 점선. 패턴 하나에 블록 하나(나사 4개)라 팔을 옆으로 뻗어도 안 흔들린다.
riser_h = 15;                // 나사 M3×25 + 너트(상판 3 + 기둥 15 + 바닥 3 + 너트)
riser_size = [62, 42];

/* ===================== 바퀴 여유 (치마) ===================== */
// 사진: 바퀴 윗면은 상판 밑에 있다 → 바닥은 원래 안 닿는다. 치마가 상판 옆으로 내려가며 바퀴 윗부분을 덮으므로
// 거기만 사다리꼴로 판다(그림 2의 각진 여유). 치마는 없어도 동작한다 — 배선 가림·겉모습용.
wheel_dia = 78.6;            // [자] 2026-11-05
wheelbase = 165;             // [자] 2026-11-09 앞뒤 바퀴 중심 간격
wheel_track = 212;           // [자] 2026-11-09 좌우 바퀴 중심 간격 → 바퀴 바깥끼리 249, 몸통(252)은 한쪽 1.5만 더 넓다
wheel_w = 37;                // [자] 2026-11-09 바퀴 두께
wheel_out = 50;              // [자] 2026-11-09 알루미늄 뼈대 끝 ↔ 바퀴 바깥 끝(축+연결부+바퀴) → 뼈대 폭 149 (상판 151.9와 검산)
wheel_gap = 3;               // [추정] 상판 밑면 ↔ 바퀴 윗면. 바퀴가 치마에 닿는지(part="clash")가 이 값에 달렸다
skirt_h = 30;                // 바닥 밑으로 — 상판 윗면 아래 15까지
notch_h = 25;                // 위 5mm 띠만 남긴다
notch_side = [wheel_dia + 34, wheel_dia - 8];    // 옆벽 사다리꼴 [아래 폭, 위 폭]
notch_end  = [wheel_w + 34, wheel_w - 4];        // 앞뒷벽 사다리꼴

/* ===================== 제어 기판 — 앞 가운데 ===================== */
// 구멍은 Uno 그대로 — Uno+쉴드(68.6×53.4)도, STM32 보드(100×70, 구멍이 같은 자리)도 같은 보스에 앉는다.
uno_holes = [[13.97, 2.54], [15.24, 50.8], [66.04, 7.62], [66.04, 35.56]];
uno_size = [68.6, 53.4];     // [DS]
stm32_size = [100, 70];      // hardware_design/mecanum_stm32_pcb
board_pos = [cx - stm32_size[0]/2, 4.5];
board_standoff_h = 8;        // 보드 밑 납땜 핀 여유
uno_stack_h = 30.0;          // [자] 2026-09-29 아두이노 밑면→Moebius 쉴드 위 가장 높은 부품
standoff_sq = 7;             // [STL] TOPLeft 보스 7×7

/* ===================== 젯슨 — 정중앙 ===================== */
// 젯슨 Orin Nano 개발자 키트(WiFi 안테나 포함) — 고정 구멍이 없어 얹고 네 모서리 턱으로 막는다. 단자는 뒤로.
jetson_size  = [103, 90.5];  // [DS] 103 × 90.5 × 35 (방열판·팬 포함)
jetson_h     = 35;           // [DS]
jetson_pos   = [cx - jetson_size[0]/2, cy - jetson_size[1]/2];
jetson_standoff_h = 12;      // 밑으로 선을 지나게 — 옛 판은 밑에 벅부스트를 넣어 여유가 없었다
jetson_fence_h = 6;

/* ===================== 배터리 — 왼쪽 ===================== */
// 배터리 4S3P — [STL] BOTTOM 관통 터널 43 × 57 × 134. 세워서 긴 쪽을 앞뒤(Y)로. batt = [긴 쪽, 두께, 높이]
batt = [134, 43, 57];
batt_lip = 2;  batt_lip_h = 10;
batt_pos = [28, cy - batt[0]/2];   // X 28~71, 왼벽과 25 띄운다(선·밴드)

/* ===================== 전원 — 오른쪽 ===================== */
// 벅부스트 2개 (12V→팔, 7.5V→바퀴) — 긴 쪽을 앞뒤로 나란히. 위아래가 다 트여 손이 들어간다.
buck_size = [76.6, 45.0];    // [자] 2026-09-26
buck_hole_d = 3.5;           // [자]
buck_inset = 1.1 + buck_hole_d/2;   // [자] 구멍 가장자리↔판 가장자리 1.1
buck_holes = [for (u = [buck_inset, buck_size[0] - buck_inset],
                   v = [buck_inset, buck_size[1] - buck_inset]) [u, v]];  // 중심 간격 70.9 × 39.3
buck_standoff_h = 5;
buck_parts_h = 14.2;         // [자] 2026-09-27 (판+부품 전체 두께)
buck_x = body_w - wall - 2 - buck_size[1];        // 판의 짧은 쪽 45가 X
buck_y = [cy - buck_size[0] - 3, cy + 3];         // 앞 판 / 뒤 판 (앞끝 45.4, 뒤끝 204.6)

// IMU MPU6050(GY-521) — 젯슨과 벅부스트 사이, 몸통 앞뒤 가운데. 주변에 진동원 없고 위에서 손이 닿는다.
imu_size = [16, 21];         // [DS] 일반 GY-521
imu_hole_gap = 15.2;         // [DS] 앞쪽 가장자리 구멍 둘 (Ø3)
imu_pos = [jetson_pos[0] + jetson_size[0] + 3.5, cy - imu_size[1]/2];

// 앞-오른쪽 예비 칸 — 전류 센서·단자대·새 보드용 M3 보스(20mm 격자)
spare_pos = [cx + stm32_size[0]/2 + 8, 12];
spare_grid = [[0, 0], [20, 0], [0, 20], [20, 20]];

/* ===================== 전원 패널 — 왼벽 앞쪽 ===================== */
// [y, z, 폭(Y), 높이(Z)] — 벽 안쪽 몸통은 왼벽~X 28 사이, 배터리 앞(y < 58)에 들어간다.
meter_cut  = [44.7, 28.2];   // [STL] BOTTOM 옆벽의 전압계 창과 같은 규격
panel_y0   = 17;
meter_win  = [panel_y0, 64, meter_cut[0], meter_cut[1]];
oled_win   = [panel_y0 + 10, 38, 25, 15];     // SSD1306 0.96" 표시부
oled_board = 27.3;  oled_hole = 23.5;         // [DS] 일반 모듈. M2 나사 4개
oled_center = [oled_win[0] + oled_win[2]/2, oled_win[1] + oled_win[3]/2];
switch_win = [panel_y0, 12, 18, 14];          // 로커 KCD1 (옛 판과 같은 구멍)
jack_win   = [panel_y0 + 26, 13, 12, 12];     // DC 충전 잭 Ø8 + 여유

/* ===================== 뒷벽 — 떼는 판 ===================== */
rear_win = [40, 8, body_w - 80, tray_h - 18];  // [x, z, 폭, 높이] 손이 들어가는 창
cover_lap = 8;                                 // 판이 창 둘레를 덮는 폭
cover_t = 2.5;
cover_slot = [120, 16];                        // 판 아래쪽 케이블 출구(바닥까지 열림)
cover_screws = [for (x = [rear_win[0] - cover_lap/2, rear_win[0] + rear_win[2] + cover_lap/2],
                     z = [rear_win[1] + rear_win[3]*0.15, rear_win[1] + rear_win[3]*0.85]) [x, z]];

/* ===================== 뚜껑 / 회전부 — 치수 그대로(지금 회전판·누름 링·베어링·서보 재사용) ===================== */
lid_t = 4;

// 회전 서보 STS3215 C018 (12V · 1/345 · 30kg·cm) — 팔과 같은 부품·같은 버스(ID 7)
// [DS] 제조사 외형도 feetechrc.com/525603.html (2026-09-26 확인)
servo_body = [45.23, 24.73, 35]; // 출력축은 45.23×24.73 면
servo_shaft_off = 45.23/2 - 12.5;   // 축이 몸체 중심에서 12.5 → 가까운 끝에서 10.1
horn_dia = 19.95;                // 원판 혼(6061), 두께 4.5
horn_pcd = 14; horn_hole = 3.4;  // 혼 쪽이 M3 암나사 4개 → 회전판은 M3 통과
horn_top = 4.3;                  // 서보 윗면 → 혼 윗면 (스플라인 3.4 + 허브 위 0.9)
servo_raise = 3;                 // 서보를 뚜껑 판 속으로 이만큼 올린다 — 혼 나사를 M3×20으로 끝내려고(2026-09-29)
horn_cb = 3;                     // 회전판 위 혼 나사머리 카운터보어 깊이 (판 6 → 밑에 3 남음)

rot_center = [cx, cy];
/* ===================== Astra Pro 전면 크래들 (2026-09-29, 자리만 옮김) ===================== */
// **트레이 앞 윗면 홈에 눕혀 끼우고 캡으로 눌러 나사 고정**한다(접이식 받침대의 틸트는 힘이 약해 안 쓴다 — 사용자).
astra = [165, 30, 40];       // [DS] Orbbec Astra Pro 데이터시트 v1.2 "Size 165mm x 30mm x 40mm" (mybotshop.de/Datasheet/Orbbec_Astra_Pro_Final.pdf, 2026-11-09 확인)
                             //      orbbec.com Astra Series 페이지는 165×48×40·310g — 48은 접은 받침대까지(= 30 + 18, 실측 astra_bracket_t 14와 4 차이)
astra_end_r = 15;            // [사진] 좌우 끝 반지름 — 높이의 반(15)이면 완전한 반원. 실물이 덜 둥글면 줄인다
astra_fov = [60, 49.5];      // [DS] 깊이 시야 H60° × V49.5° (대각 73°). 신형 페이지는 H58.4° V45.5° — 좁은 쪽이 아니라 넓은 쪽으로 턱 검사
astra_mount = "M6x7.2";      // [DS] 본체 밑 나사 = M6 깊이 7.2(Orbbec 포럼 공식 답변 2018-01-15) — 1/4"-20은 받침대 밑이지 본체가 아니다. 크래들은 어차피 안 쓴다
astra_clear = 0.6;           // 홈 여유(전체)
astra_yc = 23;               // 본체 앞뒤 중심 — 크래들 앞면이 앞벽과 한 면(y=0), 렌즈 면은 벽 안쪽 면(y=3). 2026-11-09: 사진의 옛 판처럼 앞으로 튀어나오지 않게
astra_zc = tray_h + 5;       // 본체 중심 높이
astra_tilt = 0;              // 위로 기울임(도). 무대가 높으면 5~10 — 홈과 캡이 같이 돈다
astra_end_wall = 9;          // 양 끝벽 — 나사 4개가 다 여기 든다(머리 홈 6.5에 살 1.25씩). 2026-11-09: 뒷벽 6엔 머리 홈이 안 들어가 뚫렸다
astra_back = 6;              // 뒷벽 (나사가 들어간다, M3 탭 2.9에 살 1.5씩) — 끝벽과 같은 6. cradle_y1 = 49 유지(회전판 가장자리 50)
astra_lip = 5;               // 앞면 아래·위 턱 높이 — 렌즈 띠는 비운다
// 접이식 받침대(60×23, 1/4" 너트)는 분리가 안 된다(사용자) → 접은 채 본체 밑에 매달린다. 크래들 바닥 구멍으로 트레이 안에 들어간다.
astra_bracket = [60, 23];    // [자] 받침대 판 (X, Y)
astra_bracket_t = 14;        // [자] 2026-09-29 접었을 때 본체 밑으로 나오는 두께
astra_bracket_yo = 0;        // 받침대 중심이 본체 앞뒤 중심에서 어긋난 양(+뒤)
astra_pocket = [astra_bracket[0] + 2, astra_bracket[1] + 3];   // 관통 구멍 (케이블도 여기로)
cradle_w  = astra[0] + astra_clear + 2*astra_end_wall;
cradle_x0 = cx - cradle_w/2;
cradle_y0 = astra_yc - astra[2]/2 - wall;          // 0 — 앞벽과 한 면
cradle_y1 = astra_yc + astra[2]/2 + astra_back;     // 49
cradle_z0 = astra_zc - astra[1]/2 - wall;           // 87
cap_t = 3;
cradle_top = astra_zc;                              // 홈 블록 윗면 = 본체 중심 높이 (캡이 여기서 시작)
cap_h = astra[1]/2 + astra_clear/2 + cap_t;         // 캡 높이 (cradle_top 기준)
cap_screws = [for (x = [cradle_x0 + astra_end_wall/2, cradle_x0 + cradle_w - astra_end_wall/2], y = [cradle_y0 + 12, cradle_y1 - 12]) [x, y]];
cap_cb_d = 6.5;                                     // 캡 나사머리 홈
lid_notch_y = cradle_y1 + 0.5;                      // 뚜껑은 크래들을 ㄷ자로 비켜 앞 모서리까지 덮는다

/* 나머지 회전부 — 옛 판과 같은 값 */
// 회전 베어링 = 얇은 깊은홈 6816-2RS (80 × 100 × 10) — 팔 무게·기울임을 받는다. 서보 축으로 받지 말 것.
brg = [80, 100, 10];         // [DS] 안지름, 바깥지름, 두께
brg_fit_od = 0.2;            // [가정] fit_test 중간값(2026-09-27)
brg_fit_id = -0.1;           // [가정] fit_test 중간값(2026-09-27)
seat_shoulder_h = 3;         // 바깥 링 밑 받침 높이
seat_wall = 6;               // 받침 벽 → 받침 바깥지름 112
seat_r = brg[1]/2 + seat_wall;
clamp_t = 2;                 // 바깥 누름 링 두께 — 나사 M3×12 = 링 2 + 받침 구멍 10
clamp_screw_r = brg[1]/2 + seat_wall/2;
head_groove_w = 8;  head_groove_d = 3;   // 회전판 밑 나사머리 홈
hub_wall = 3;
snap_lip = 0.5;  snap_h = 1.2;  snap_slots = 6;
plat_z = seat_shoulder_h + brg[2] + clamp_t + 1;   // 뚜껑 윗면 → 회전판 밑면 = 16

// 180° 기계 제한 — 스토퍼 핀은 조립 전에 부러졌지만(README) 지금 회전판을 그대로 쓰니 홈은 남긴다.
travel_deg = 180;
stop_r = 32;  stop_pin_d = 5;  stop_groove_depth = 3;
stop_deg = 180;
cable_r = 20;  cable_w = 12;  cable_deg = 180;

plat_dia = 150;  plat_t = 6;
coupler_h = plat_z - (horn_top - lid_t) - servo_raise;   // 혼 윗면 → 회전판 밑면 = 12.7
// SO-101 받침 체결 4구멍 — [자] 2026-09-27 전수 실측. 회전판 좌표 = 케이스 좌표(−Y = 앞).
so101_front_gap = 56.7;  // [자] 실측 — BOTTOM 계산값(67.5)은 폐기
so101_holes = [[-so101_front_gap/2, -15.0], [so101_front_gap/2, -15.0], [-64.4/2, 55.0], [64.4/2, 55.0]];
so101_offset = [0, 0];
insert_d = 4.2;

brand_text = "ForNerds";  brand_size = 12;  brand_emboss = 0.8;

/* 뚜껑 기둥: 네 모서리 + 왼쪽 가운데 + 오른쪽 앞. 위아래로 인서트(위 = 뚜껑, 아래 = 치마) */
post_d = 9;
lid_corners = [[12, 12], [body_w - 12, 12], [12, body_d - 12], [body_w - 12, body_d - 12],
               [9, cy], [body_w - 9, 34]];

/* ===================== 검사 ===================== */
if (body_w > bed_x || body_d - cradle_y0 > bed_y) echo("⚠ 트레이가 베드를 넘는다", body_w, body_d - cradle_y0);
if (plat_dia > min(bed_x, bed_y)) echo("⚠ 회전판이 베드를 넘는다");
if (body_w < plate_w + 2*wall + 2) echo("⚠ 치마가 상판에 걸린다");
if (abs(wheel_track + wheel_w - 2*wheel_out - plate_w) > 5) echo("⚠ 바퀴 실측으로 낸 뼈대 폭이 상판과 5mm 넘게 다르다", wheel_track + wheel_w - 2*wheel_out);
if (wheel_track + wheel_w > body_w) echo("⚠ 바퀴가 몸통 밖으로 나간다");
if (seat_r > cx - 3) echo("⚠ 베어링 받침이 뚜껑 폭을 넘는다");
if (stop_r + stop_pin_d/2 + 1 > brg[0]/2 - hub_wall) echo("⚠ 스토퍼 핀이 허브 벽에 닿는다");
if (cable_r + cable_w/2 + 3 > stop_r - (stop_pin_d + 1)/2) echo("⚠ 배선 슬롯과 스토퍼 홈 사이 살이 3mm 미만");
if (cable_r - cable_w/2 < (horn_dia + 3)/2 + 2) echo("⚠ 배선 슬롯이 혼 구멍과 붙는다");
if (floor_t + jetson_standoff_h + jetson_h + 10 > tray_h - servo_body[2] + servo_raise) echo("⚠ 팬 위 공기가 10mm 미만");
if (floor_t + board_standoff_h + uno_stack_h > astra_zc - astra[1]/2 - astra_bracket_t - 2)
    echo("⚠ 접힌 Astra 받침대가 기판 스택에 닿는다");
if (board_pos[1] + stm32_size[1] > jetson_pos[1]) echo("⚠ 제어 기판이 젯슨 자리로 들어간다");
if (batt_pos[0] + batt[1] > jetson_pos[0] - 3) echo("⚠ 배터리와 젯슨 사이 3mm 미만");
if (panel_y0 + meter_cut[0] > batt_pos[1] + 7) echo("⚠ 전압계가 배터리와 겹친다");
if (imu_pos[0] + imu_size[0] > buck_x - 0.5) echo("⚠ IMU가 벅부스트에 닿는다");
if (jetson_pos[1] + jetson_size[1] + 60 > body_d - wall) echo("⚠ 뒤 케이블 칸이 60mm 미만");
if (cradle_y1 > rot_center[1] - plat_dia/2) echo("⚠ Astra 크래들이 회전판 밑으로 들어간다");
if (astra_end_wall - astra_clear/2 < cap_cb_d + 2) echo("⚠ 캡 나사머리 홈이 끝벽을 뚫는다");
if (astra[1]/2 - astra_lip < wall * tan(astra_fov[1]/2) + 3) echo("⚠ 앞 턱이 Astra 세로 시야를 가린다");   // 턱 안쪽 모서리는 렌즈 면보다 wall 앞, 중심에서 15−lip 위
if (lid_notch_y + 3 > rot_center[1] - seat_r) echo("⚠ 뚜껑 앞 홈이 베어링 받침과 겹친다");
if (servo_raise > lid_t - 1) echo("⚠ 서보 포켓이 뚜껑 판을 뚫는다");
if (plat_t - horn_cb < 2.5) echo("⚠ 혼 나사머리 밑 판이 너무 얇다");
if (plat_z - seat_shoulder_h - brg[2] - clamp_t + head_groove_d < 3) echo("⚠ 누름 링 나사머리가 회전판에 닿는다");
for (h = so101_holes) let (r = norm(h)) if (abs(r - clamp_screw_r) < head_groove_w/2 + insert_d/2 + 1) echo("⚠ SO-101 인서트가 나사머리 홈에 걸린다", r);
if (cap_h - 10 < 3) echo("⚠ 캡 나사머리 밑 살이 3mm 미만");
echo("몸통", body_w, "x", body_d, "x", tray_h + lid_t, " 상판 위 회전판 윗면", riser_h + tray_h + lid_t + plat_z + plat_t,
     " 회전중심", rot_center, " 패턴중심", mount_centers, " 뒤 케이블 칸", body_d - wall - jetson_pos[1] - jetson_size[1]);
/* ===================== 조각 ===================== */
module holes_pattern(c, dx, dy, d, h) {
    for (sx = [-1, 1], sy = [-1, 1])
        translate([c[0] + sx*dx/2, c[1] + sy*dy/2, -1]) cylinder(h = h + 2, d = d);
}
module ring_holes(pcd, d, h, n = 4, a0 = 45) {
    for (i = [0:n-1]) rotate(a0 + i*360/n) translate([pcd/2, 0, -1]) cylinder(h = h + 2, d = d);
}
module arc(r, w, h, a0, span) {
    rotate(a0) rotate_extrude(angle = span) translate([r - w/2, 0]) square([w, h]);
    for (a = [a0, a0 + span]) rotate(a) translate([r, 0, 0]) cylinder(h = h, d = w);
}
module boss(h, hole) {
    difference() {
        translate([-standoff_sq/2, -standoff_sq/2, 0]) cube([standoff_sq, standoff_sq, h]);
        translate([0, 0, 0.6]) cylinder(h = h, d = hole);   // 바닥 막음
    }
}
module bosses(pos, holes, h, hole = m3_tap) {
    translate([pos[0], pos[1], floor_t]) for (p = holes) translate(p) boss(h, hole);
}
// 몸통 평면 모양 — 모서리를 깎은 사각형. inset만큼 안으로.
module footprint(inset = 0) {
    c = max(0.01, corner_cut - inset*0.59);
    x0 = inset; x1 = body_w - inset; y0 = inset; y1 = body_d - inset;
    polygon([[x0 + c, y0], [x1 - c, y0], [x1, y0 + c], [x1, y1 - c], [x1 - c, y1], [x0 + c, y1], [x0, y1 - c], [x0, y0 + c]]);
}
// 벽을 관통하는 창 — X면(왼벽): [y, z, 폭, 높이]
module left_win(w) { translate([-1, w[0], w[1]]) cube([wall + 2, w[2], w[3]]); }

/* ----- 0) Astra 크래들 + 캡 ----- */
module pill(L, h, d) {   // Astra 본체 모양 — 앞에서 본 면(X 폭 L, Z 높이 h)의 좌우 끝만 반원, 앞뒤(Y 깊이 d)는 곧다
    r = min(astra_end_r, h/2);   // 2026-11-09 사용자: 보는 방향엔 곡면이 없고 좌우로만 있다(전엔 앞뒤 모서리를 둥글렸다)
    hull() for (x = [-(L/2 - r), L/2 - r], z = [-(h/2 - r), h/2 - r])
        translate([x, 0, z]) rotate([90, 0, 0]) cylinder(h = d, r = r, center = true, $fn = 96);
}
module astra_body(clear = astra_clear) {   // 본체 자리 (홈·캡 공통, 기울임 포함)
    translate([cx, astra_yc, astra_zc]) rotate([astra_tilt, 0, 0])
        pill(astra[0] + clear, astra[1] + clear, astra[2] + clear);
}
module astra_cradle() {
    difference() {
        translate([cradle_x0, cradle_y0, cradle_z0]) cube([cradle_w, cradle_y1 - cradle_y0, cradle_top - cradle_z0]);
        for (c = cap_screws) translate([c[0], c[1], cradle_top - 10]) cylinder(h = 11, d = m3_tap);
    }
}
module astra_cuts() {   // 트레이 전체에서 판다 — 앞벽이 홈 안에 남지 않게
    astra_body();
    // 앞면 렌즈 창 — 아래 턱(astra_lip)만 남기고 앞을 연다
    translate([cradle_x0 + astra_end_wall, cradle_y0 - 1, astra_zc - astra[1]/2 + astra_lip])
        cube([cradle_w - 2*astra_end_wall, wall + 2 + (0 - cradle_y0), astra[1]]);
    // 받침대·케이블 관통 구멍
    translate([cx - astra_pocket[0]/2, astra_yc + astra_bracket_yo - astra_pocket[1]/2, cradle_z0 - 1])
        cube([astra_pocket[0], astra_pocket[1], astra_zc - cradle_z0 + 2]);
}
module astra_cap() {   // 위에서 덮어 나사 4개로 조인다. 좌표는 조립 위치(밑면 = cradle_top)
    difference() {
        translate([cradle_x0, cradle_y0, cradle_top]) cube([cradle_w, cradle_y1 - cradle_y0, cap_h]);
        astra_body();
        // 앞면 렌즈 창 — 위 턱(astra_lip)만 남긴다
        translate([cradle_x0 + astra_end_wall, cradle_y0 - 1, cradle_top - 1])
            cube([cradle_w - 2*astra_end_wall, wall + 2, astra[1]/2 - astra_lip + 1]);
        for (c = cap_screws) {
            translate([c[0], c[1], cradle_top - 1]) cylinder(h = cap_h + 2, d = m3_clear);
            translate([c[0], c[1], cradle_top + 10]) cylinder(h = cap_h, d = cap_cb_d);   // 머리를 깊이 묻어 M3×20 (크래들 10 + 캡 10)
        }
    }
}

/* ----- 1) 트레이 ----- */
module cable_tie(x, y) {   // 케이블 타이 고리 — 바닥에서 뜬 다리
    translate([x, y, floor_t]) difference() {
        translate([-7, -3, 0]) cube([14, 6, 7]);
        translate([-4, -4, -0.01]) cube([8, 8, 4]);
    }
}
module tray() {
    difference() {
        tray_raw();
        astra_cuts();
        for (s = cover_screws) translate([s[0], body_d - wall - 6, s[1]]) rotate([-90, 0, 0]) cylinder(h = wall + 7, d = m3_tap);
    }
}
module tray_raw() {
    difference() {
        linear_extrude(tray_h) footprint();
        translate([0, 0, floor_t]) linear_extrude(tray_h) footprint(wall);
        // 체결 패턴 — 나사가 바닥을 지나 안쪽에서 너트로 받는다
        for (c = mount_centers) holes_pattern(c, mount_dx, mount_dy, m3_clear, floor_t);
        // 상판으로 내려가는 선 구멍 — 바퀴 쪽 네 군데 + 기둥 양옆
        for (y = [cy - wheelbase/2 - 8, cy + wheelbase/2 + 2], x = [wall + 26, body_w - wall - 26 - 26])
            translate([x, y, -1]) cube([26, 16, floor_t + 2]);
        // 젯슨 밑 공기 — 받침 사이
        for (x = [0 : 14 : 70]) translate([jetson_pos[0] + 12 + x, jetson_pos[1] + 12, -1]) cube([6, jetson_size[1] - 24, floor_t + 2]);
        // 배터리 고정 밴드 슬롯 — 앞뒤 둘, 양옆
        for (y = [batt_pos[1] + batt[0]*0.25 - 10, batt_pos[1] + batt[0]*0.75 - 10], x = [batt_pos[0] - 6, batt_pos[0] + batt[1] + 1.5])
            translate([x, y, -1]) cube([3, 20, floor_t + 2]);
        // 환기 — 왼벽은 배터리 위, 오른벽은 벅부스트 옆 전 높이
        for (y = [cy - 55 : 12 : cy + 55]) translate([-1, y, 66]) cube([wall + 2, 6, tray_h - 76]);
        for (y = [buck_y[0] + 6 : 12 : buck_y[1] + buck_size[0] - 10]) translate([body_w - wall - 1, y, 12]) cube([wall + 2, 6, tray_h - 22]);
        // 전원 패널 — 왼벽 앞쪽
        left_win(meter_win);
        left_win(oled_win);
        left_win(switch_win);
        left_win(jack_win);
        // 뒷벽 창
        translate([rear_win[0], body_d - wall - 1, rear_win[1]]) cube([rear_win[2], wall + 2, rear_win[3]]);
    }
    // 뚜껑 기둥: 위 = 뚜껑 인서트 M3×12, 아래 = 치마 인서트
    for (c = lid_corners) translate([c[0], c[1], 0])
        difference() {
            cylinder(h = tray_h, d = post_d);
            translate([0, 0, tray_h - 6]) cylinder(h = 7, d = insert_d);
            translate([0, 0, tray_h - 9]) cylinder(h = 4, d = 2.6);   // 나사 끝 도피
            translate([0, 0, -1]) cylinder(h = 7, d = insert_d);
        }
    // 제어 기판 보스 — Uno 구멍 패턴(STM32도 같은 자리)
    bosses(board_pos, uno_holes, board_standoff_h);
    // 벅부스트 보스 — 판을 돌려 놓는다(긴 쪽 = Y)
    for (y = buck_y) translate([buck_x, y, floor_t]) for (p = buck_holes) translate([p[1], p[0], 0]) boss(buck_standoff_h, m3_tap);
    // 예비 칸 보스
    bosses(spare_pos, spare_grid, 5);
    // IMU 보스 2개 — M2, 구멍이 앞쪽 가장자리. 진동이 심하면 양면 폼으로 바꾼다
    translate([imu_pos[0] + imu_size[0]/2, imu_pos[1] + 2.5, floor_t])
        for (s = [-1, 1]) translate([s*imu_hole_gap/2, 0, 0]) difference() {
            cylinder(h = 5, d = 5);
            translate([0, 0, 1]) cylinder(h = 5, d = m2_tap);
        }
    translate([imu_pos[0], imu_pos[1] + imu_size[1] - 3, floor_t]) cube([imu_size[0], 3, 4]);   // 뒤 끝 받침
    // 젯슨: 얹는다 — 좌우 받침 레일 + 네 모서리 ㄱ자 턱. 레일 사이로 선이 지난다
    translate([jetson_pos[0], jetson_pos[1], floor_t]) {
        for (x = [2, jetson_size[0] - 8]) translate([x, 8, 0]) cube([6, jetson_size[1] - 16, jetson_standoff_h]);
        for (sx = [0, 1], sy = [0, 1]) translate([sx*jetson_size[0], sy*jetson_size[1], 0]) mirror([sx, 0, 0]) mirror([0, sy, 0]) {
            translate([-3, -3, 0]) cube([sy == 1 ? 10 : 18, 2.5, jetson_standoff_h + (sy == 1 ? 2 : jetson_fence_h)]);   // 뒤(단자 쪽)는 2만 올린다
            translate([-3, -3, 0]) cube([2.5, 18, jetson_standoff_h + jetson_fence_h]);
            cube([8, 8, jetson_standoff_h]);
        }
    }
    // 배터리 턱 — 좌우 긴 턱 + 앞뒤 끝
    translate([batt_pos[0], batt_pos[1], floor_t]) {
        for (x = [-batt_lip - 0.5, batt[1] + 0.5]) translate([x, 0, 0]) cube([batt_lip, batt[0], batt_lip_h]);
        for (y = [-batt_lip - 0.5, batt[0] + 0.5]) translate([0, y, 0]) cube([batt[1], batt_lip, batt_lip_h]);
    }
    // OLED 나사 기둥 (왼벽 안쪽, M2)
    for (sy = [-1, 1], sz = [-1, 1]) translate([wall - 0.01, oled_center[0] + sy*oled_hole/2, oled_center[1] + sz*oled_hole/2])
        rotate([0, 90, 0]) difference() {
            cylinder(h = 5, d = 5);
            translate([0, 0, 1]) cylinder(h = 5, d = m2_tap);
        }
    // 뒷벽 판 나사 보스
    for (s = cover_screws) translate([s[0], body_d - wall - 8, s[1]]) rotate([-90, 0, 0]) cylinder(h = 8.01, d = 7);
    // 케이블 타이 고리 — 뒤 칸
    for (x = [cx - 50, cx, cx + 50]) cable_tie(x, body_d - 22);
    astra_cradle();
    translate([cx, 0, 42]) rotate([90, 0, 0])
        linear_extrude(brand_emboss)
            text(brand_text, size = brand_size, halign = "center", valign = "center",
                 font = "Liberation Sans:style=Bold");
}

/* ----- 2) 뚜껑 ----- */
module lid() {
    difference() {
        union() {
            linear_extrude(lid_t) difference() {
                footprint();
                translate([cradle_x0 - 0.5, -5]) square([cradle_w + 1, lid_notch_y + 5]);
            }
            // 베어링 받침: 바깥 링만 받는 턱 + 둘레 벽
            translate([rot_center[0], rot_center[1], lid_t - 0.01]) difference() {
                cylinder(h = seat_shoulder_h + brg[2], r = seat_r, $fn = 120);
                translate([0, 0, seat_shoulder_h]) cylinder(h = brg[2] + 1, d = brg[1] + brg_fit_od, $fn = 180);
                translate([0, 0, -1]) cylinder(h = seat_shoulder_h + 2, r = brg[1]/2 - 3.5, $fn = 120);
                translate([0, 0, 2]) ring_holes(2*clamp_screw_r, m3_tap, seat_shoulder_h + brg[2]);   // 누름 링 나사 (깊이 11, M3×12)
            }
        }
        for (c = lid_corners) translate([c[0], c[1], -1]) cylinder(h = lid_t + 2, d = m3_clear);
        // 서보 몸체가 뚜껑 판 속으로 servo_raise만큼 들어오는 포켓
        translate([rot_center[0] - servo_shaft_off - 0.2, rot_center[1] - servo_body[1]/2 - 0.2, -1])
            cube([servo_body[0] + 0.4, servo_body[1] + 0.4, servo_raise + 1]);
        translate([rot_center[0], rot_center[1], 0]) {
            translate([0, 0, -1]) cylinder(h = lid_t + 2, d = horn_dia + 3);
            translate([0, 0, -1]) arc(cable_r, cable_w, lid_t + 2, cable_deg - travel_deg/2, travel_deg);
            translate([0, 0, lid_t - stop_groove_depth])
                arc(stop_r, stop_pin_d + 1, stop_groove_depth + 1, stop_deg - travel_deg/2, travel_deg);
        }
        // 환기 — 베어링 받침 양옆과 뒤
        for (x = [18 : 9 : cx - seat_r - 8], s = [0, 1]) translate([s == 0 ? x : body_w - x - 4, cy - 55, -1]) cube([4, 110, lid_t + 2]);
        for (x = [cx - 70 : 10 : cx + 66]) translate([x, cy + seat_r + 6, -1]) cube([5, 30, lid_t + 2]);
    }
    translate([rot_center[0] - servo_shaft_off, rot_center[1] - servo_body[1]/2, -servo_body[2] + servo_raise])
        difference() {
            translate([-wall, -wall, 0]) cube([servo_body[0] + 2*wall, servo_body[1] + 2*wall, servo_body[2]]);
            translate([-0.2, -0.2, -1]) cube([servo_body[0] + 0.4, servo_body[1] + 0.4, servo_body[2] + 2]);
            translate([servo_body[0] - 1, 4, -1]) cube([wall + 2, servo_body[1] - 8, 10]);
            // 긴 두 면만 관통 — 한 바퀴 자르면 띠가 뚜껑에서 떨어진다
            for (z = [8, 24]) translate([6, -wall - 1, z]) cube([servo_body[0] - 12, servo_body[1] + 2*wall + 2, 4]);
        }
}

/* ----- 2b) 바깥 링 누름 링 — 그대로 ----- */
module clamp() {
    difference() {
        cylinder(h = clamp_t, r = seat_r, $fn = 120);
        translate([0, 0, -1]) cylinder(h = clamp_t + 2, d = brg[1] - 6, $fn = 120);   // 바깥 링 위 3mm를 덮는다
        ring_holes(2*clamp_screw_r, m3_clear, clamp_t);
    }
}

/* ----- 3) 회전판 — 그대로 ----- */
module platform() {
    hub_len = plat_z - seat_shoulder_h;          // 회전판 밑면 → 안쪽 링 밑면
    lip_h = plat_z - seat_shoulder_h - brg[2];   // 안쪽 링 위 턱 높이
    difference() {
        cylinder(h = plat_t, d = plat_dia, $fn = 120);
        ring_holes(horn_pcd, horn_hole, plat_t);
        // 누름 링 나사머리 도피 홈 (밑면 원형)
        translate([0, 0, -1]) difference() {
            cylinder(h = head_groove_d + 1, r = clamp_screw_r + head_groove_w/2, $fn = 120);
            translate([0, 0, -1]) cylinder(h = head_groove_d + 3, r = clamp_screw_r - head_groove_w/2, $fn = 120);
        }
        translate([0, 0, plat_t - horn_cb]) ring_holes(horn_pcd, 6.5, horn_cb + 1);   // 나사머리 자리
        translate([0, 0, -1]) cylinder(h = plat_t + 2, d = 3);
        rotate(cable_deg) translate([cable_r, 0, -1]) cylinder(h = plat_t + 2, d = cable_w - 1);
        for (h = so101_holes) translate([so101_offset[0] + h[0], so101_offset[1] + h[1], -1])
            cylinder(h = plat_t + 2, d = insert_d);
    }
    // 안쪽 링 위 턱 (바깥 링·누름 링에는 안 닿게 반지름 +4까지만)
    translate([0, 0, -lip_h]) difference() {
        cylinder(h = lip_h + 0.5, r = brg[0]/2 + 4, $fn = 120);
        translate([0, 0, -1]) cylinder(h = lip_h + 2, r = brg[0]/2 - hub_wall, $fn = 120);
    }
    // 허브: 안쪽 링에 끼우고, 끝의 걸쇠가 링 밑을 잡는다. 세로 홈으로 손가락처럼 휜다.
    translate([0, 0, -hub_len - snap_h]) difference() {
        union() {
            cylinder(h = hub_len + snap_h + 0.5, d = brg[0] + brg_fit_id, $fn = 180);
            // 걸쇠 턱: 밑은 비스듬히(밀어 넣기), 위는 평평(빠지지 않게)
            cylinder(h = snap_h, d1 = brg[0] + brg_fit_id, d2 = brg[0] + brg_fit_id + 2*snap_lip, $fn = 180);
        }
        translate([0, 0, -1]) cylinder(h = hub_len + snap_h + 3, d = brg[0] - 2*hub_wall, $fn = 120);
        for (i = [0 : snap_slots - 1]) rotate(i*360/snap_slots + 30)
            translate([0, -1, -1]) cube([brg[0], 2, 9 + 1]);
    }
    // 혼 ↔ 회전판 연결 기둥
    translate([0, 0, -coupler_h]) difference() {
        cylinder(h = coupler_h + 0.5, d = horn_dia);   // 판 속으로 0.5 겹쳐 한 덩어리로
        ring_holes(horn_pcd, horn_hole, coupler_h + 0.5);
        translate([0, 0, -1]) cylinder(h = coupler_h + 2.5, d = 3.4);
    }
    rotate(stop_deg) translate([stop_r, 0, -(plat_z + stop_groove_depth - 0.5)])
        cylinder(h = plat_z + stop_groove_depth, d = stop_pin_d);
}

/* ----- 4) 받침 기둥 — 두 개, 패턴 하나에 하나. 좌표는 조립 위치의 패턴 중심 기준(윗면 Z=0) ----- */
module riser() {
    translate([0, 0, -riser_h]) difference() {
        translate([-riser_size[0]/2, -riser_size[1]/2, 0]) cube([riser_size[0], riser_size[1], riser_h]);
        holes_pattern([0, 0], mount_dx, mount_dy, m3_clear, riser_h);
        // 살 빼기 + 가운데로 선이 지나갈 수 있게
        translate([-14, -riser_size[1]/2 - 1, -1]) cube([28, riser_size[1] + 2, riser_h - 3 + 1]);
    }
}

/* ----- 5) 치마 — 바닥 밑 둘레, 바퀴 자리는 사다리꼴로 판다. 윗면 Z=0 ----- */
notch_depth = corner_cut + wall + 2;   // 그 벽만 판다 — 몸통을 관통하면 맞은편 벽까지 깎인다
module notch_x(x0, yc) {   // 옆벽(X면) 사다리꼴 — x0부터 notch_depth만큼
    translate([x0, yc, -skirt_h - 0.01]) rotate([90, 0, 90]) linear_extrude(notch_depth)
        polygon([[-notch_side[0]/2, 0], [notch_side[0]/2, 0], [notch_side[1]/2, notch_h], [-notch_side[1]/2, notch_h]]);
}
module notch_y(xc, y1) {   // 앞뒷벽(Y면) 사다리꼴 — y1에서 −Y로 notch_depth만큼
    translate([xc, y1, -skirt_h - 0.01]) rotate([90, 0, 0]) linear_extrude(notch_depth)
        polygon([[-notch_end[0]/2, 0], [notch_end[0]/2, 0], [notch_end[1]/2, notch_h], [-notch_end[1]/2, notch_h]]);
}
module skirt() {
    difference() {
        union() {
            translate([0, 0, -skirt_h]) linear_extrude(skirt_h) difference() { footprint(); footprint(wall); }
            // 기둥 자리 귀 — 트레이 기둥 밑 인서트로 위로 조인다
            intersection() {
                translate([0, 0, -8]) linear_extrude(8) footprint();
                for (c = lid_corners) translate([c[0], c[1], -8]) hull() {
                    cylinder(h = 8, d = post_d + 2);
                    translate([c[0] < cx ? -12 : 12, c[1] == cy || c[1] == 34 ? 0 : (c[1] < cy ? -12 : 12), 0]) cylinder(h = 8, d = 2);
                }
            }
        }
        for (c = lid_corners) translate([c[0], c[1], -9]) cylinder(h = 10, d = m3_clear);
        for (y = [cy - wheelbase/2, cy + wheelbase/2], x0 = [-1, body_w + 1 - notch_depth]) notch_x(x0, y);
        for (x = [cx - wheel_track/2, cx + wheel_track/2], y1 = [notch_depth - 1, body_d + 1]) notch_y(x, y1);
    }
}

/* ----- 6) 뒷벽 판 — 트레이 뒤에서 나사 4개. 아래 가운데가 케이블 출구 ----- */
module rear_cover() {
    difference() {
        translate([rear_win[0] - cover_lap, body_d, rear_win[1] - cover_lap + 2])
            cube([rear_win[2] + 2*cover_lap, cover_t, rear_win[3] + 2*cover_lap - 2]);
        for (s = cover_screws) translate([s[0], body_d - 1, s[1]]) rotate([-90, 0, 0]) cylinder(h = cover_t + 2, d = m3_clear);
        // 케이블 출구 — 아래 가장자리까지 트였다
        translate([cx - cover_slot[0]/2, body_d - 1, rear_win[1] - cover_lap]) cube([cover_slot[0], cover_t + 2, cover_slot[1] + cover_lap]);
        // 환기
        for (x = [cx - 70 : 10 : cx + 66]) translate([x, body_d - 1, rear_win[1] + cover_slot[1] + 8]) cube([5, cover_t + 2, 40]);
    }
}

/* ----- 7) 끼움 시험 링 ----- */
module fit_test() {
    for (i = [0 : 2]) {
        translate([i*120, 0, 0]) difference() {       // 받침: 바깥지름 +0.0 / +0.2 / +0.4
            union() {
                cylinder(h = 5, r = brg[1]/2 + 3, $fn = 120);
                for (k = [0 : i]) rotate(k*12) translate([brg[1]/2 + 3, 0, 0]) cylinder(h = 5, d = 3);
            }
            translate([0, 0, -1]) cylinder(h = 7, d = brg[1] + 0.2*i, $fn = 180);
        }
        translate([i*120, 110, 0]) difference() {     // 허브: 안지름 −0.2 / −0.1 / 0.0
            union() {
                cylinder(h = 5, d = brg[0] - 0.2 + 0.1*i, $fn = 180);
                for (k = [0 : i]) rotate(k*12) translate([brg[0]/2 - hub_wall, 0, 0]) cylinder(h = 5, d = 3);
            }
            translate([0, 0, -1]) cylinder(h = 7, d = brg[0] - 2*hub_wall - 2, $fn = 120);
        }
    }
}

/* ===================== 안에 들어가는 것 (유령) — 렌더·간섭 검사 공용 ===================== */
module g_jetson()  { translate([jetson_pos[0], jetson_pos[1], floor_t + jetson_standoff_h]) cube([jetson_size[0], jetson_size[1], jetson_h]); }
module g_battery() { translate([batt_pos[0], batt_pos[1], floor_t]) cube([batt[1], batt[0], batt[2]]); }
module g_bucks()   { for (y = buck_y) translate([buck_x, y, floor_t + buck_standoff_h]) cube([buck_size[1], buck_size[0], buck_parts_h]); }
module g_board()   { translate([board_pos[0], board_pos[1], floor_t + board_standoff_h]) cube([stm32_size[0], stm32_size[1], 22]); }  // 1000µF 20 + 판 1.6
module g_uno()     { translate([board_pos[0], board_pos[1], floor_t + board_standoff_h]) cube([uno_size[0], uno_size[1], uno_stack_h]); }
module g_servo()   { translate([rot_center[0] - servo_shaft_off, rot_center[1] - servo_body[1]/2, tray_h + servo_raise - servo_body[2]]) cube([servo_body[0], servo_body[1], servo_body[2]]); }
module g_astra()   {
    astra_body(0);
    translate([cx - astra_bracket[0]/2, astra_yc + astra_bracket_yo - astra_bracket[1]/2, astra_zc - astra[1]/2 - astra_bracket_t])
        cube([astra_bracket[0], astra_bracket[1], astra_bracket_t + 1]);
}
module g_imu()     { translate([imu_pos[0], imu_pos[1], floor_t + 5]) cube([imu_size[0], imu_size[1], 3]); }
module g_panel()   {                                 // 벽 안쪽 몸통
    translate([wall, meter_win[0], meter_win[1]]) cube([22, meter_win[2], meter_win[3]]);
    translate([wall, switch_win[0], switch_win[1]]) cube([24, switch_win[2], switch_win[3]]);
    translate([wall, jack_win[0], jack_win[1]]) cube([28, jack_win[2], jack_win[3]]);
    translate([wall + 5, oled_center[0] - oled_board/2, oled_center[1] - oled_board/2]) cube([5, oled_board, oled_board]);
}
module g_usb()     {                                 // 젯슨 단자에 꽂은 플러그 + 굽는 자리 60mm
    translate([jetson_pos[0], jetson_pos[1] + jetson_size[1] + 1, floor_t + jetson_standoff_h + 2.5]) cube([jetson_size[0], 60, 30]);
}
module g_stuff() { g_jetson(); g_battery(); g_bucks(); g_board(); g_servo(); g_astra(); g_imu(); g_panel(); g_usb(); }
module printed() {
    tray();
    translate([0, 0, tray_h]) lid();
    astra_cap();
    translate([0, 0, 0]) skirt();
    rear_cover();
    for (c = mount_centers) translate([c[0], c[1], 0]) riser();
}
module g_wheels() {   // 차체 바퀴 — 치마 간섭 검사용
    for (sx = [-1, 1], sy = [-1, 1]) translate([cx + sx*wheel_track/2, cy + sy*wheelbase/2, -riser_h - 3 - wheel_gap - wheel_dia/2])
        rotate([0, 90, 0]) cylinder(h = wheel_w, d = wheel_dia, center = true, $fn = 96);
}
module clash_pairs() {
    intersection() { printed(); g_wheels(); }
    intersection() { printed(); g_stuff(); }
    intersection() { g_jetson(); g_servo(); }
    intersection() { g_jetson(); g_imu(); }
    intersection() { g_jetson(); g_battery(); }
    intersection() { g_jetson(); g_bucks(); }
    intersection() { g_jetson(); g_board(); }
    intersection() { g_battery(); g_panel(); }
    intersection() { g_bucks(); g_imu(); }
    intersection() { g_board(); g_astra(); }
    intersection() { g_uno(); g_astra(); }
    intersection() { g_usb(); g_bucks(); }
    intersection() { g_usb(); g_servo(); }
}

/* ===================== 출력 ===================== */
if (part == "tray") tray();
else if (part == "lid") lid();
else if (part == "clamp") clamp();
else if (part == "platform") platform();
else if (part == "fit_test") fit_test();
else if (part == "riser") translate([0, 0, riser_h]) riser();
else if (part == "skirt") translate([0, 0, skirt_h]) skirt();
else if (part == "rear_cover") rotate([-90, 0, 0]) translate([0, -body_d - cover_t, 0]) rear_cover();   // 눕혀 베드에
else if (part == "astra_cap") rotate([180, 0, 0]) translate([0, 0, -(cradle_top + cap_h)]) astra_cap();   // 뒤집어 베드에
else if (part == "clash") clash_pairs();
else if (part == "inside") {     // 뚜껑 뗀 모습 — 배치 확인용
    color("lightgray") tray();
    color("dimgray") skirt();
    color("seagreen") g_board();
    color("steelblue") g_jetson();
    color("firebrick") g_battery();
    color("goldenrod") g_bucks();
    color("purple") g_imu();
    color("black") g_panel();
    color("black", 0.75) g_astra();
    color("orange") astra_cap();
    color("khaki", 0.35) g_usb();
}
else {
    lz = tray_h + lid_t;
    %color("black") astra_body(0);                                          // Astra 본체 (유령)
    color("gold") astra_cap();
    color("lightgray") tray();
    color("dimgray") skirt();
    color("gray") rear_cover();
    color("silver") translate([0, 0, tray_h]) lid();
    %translate([rot_center[0], rot_center[1], lz + seat_shoulder_h])     // 6816 (구매품, 유령)
        difference() { cylinder(h = brg[2], d = brg[1]); translate([0, 0, -1]) cylinder(h = brg[2] + 2, d = brg[0]); }
    color("gray") translate([rot_center[0], rot_center[1], lz + seat_shoulder_h + brg[2]]) clamp();
    color("orange") translate([rot_center[0], rot_center[1], lz + plat_z]) platform();
    for (c = mount_centers) color("darkslategray") translate([c[0], c[1], 0]) riser();
    // 차체(유령): 상판 + 바퀴 — 치수 일부 [추정]
    %translate([cx - plate_w/2, cy - 110, -riser_h - 3]) cube([plate_w, 220, 3]);
    %g_wheels();
}
