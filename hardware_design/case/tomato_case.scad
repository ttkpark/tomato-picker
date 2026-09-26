// tomato_case.scad — 토마토 로봇 케이스: 전장 트레이 + 뚜껑 + 180° 회전판
// 스케치 docs/design/case-sketch-2026-09-24.jpg · 실측 docs/design/3D-parts-mapping.md
//
// 좌표: X=좌우, Y=앞뒤, Z=위. Y=0 면이 전면 = 아두이노 쪽 = 팔이 보는 쪽(전체 외관.pdf 4쪽).
// 원점 = 트레이 바닥 앞-왼 모서리.
// 기준은 차체 외곽이 아니라 **차체 상판의 나사 구멍**이다 — 차체 길이는 모르고,
// 바퀴가 폭 바깥에 있어 케이스가 앞뒤로 튀어나와도 간섭이 없다.
//
// 부품별 STL: openscad -D 'part="tray"' -o tray.stl tomato_case.scad
//             (part = "assembly" | "tray" | "lid" | "platform")
//
// 값의 출처:
//   [STL]  3D/ 원본을 잘라 잰 값          [DS] 부품 데이터시트
//   [자]   전체 외관.pdf 줄자·캘리퍼 실측  [실측] 아직 모름 — 지금 값은 형상 확인용

part = "assembly";

/* ===================== 공통 ===================== */
wall   = 3;      // [STL] 기존 파츠 벽·판 두께
floor_t = 3;     // [STL]
m3_clear = 3.4;  // [STL]
m3_tap   = 2.9;  // [STL] TOPLeft 아두이노 보스 2.93
m2_tap   = 1.8;
$fn = 48;

/* ===================== 차체 체결 (확정) ===================== */
// 50×30 M3 패턴 두 벌. 30 변이 길이(Y) 방향 — 바깥나사 195.5 − 안쪽나사 136 = 59.5 ≈ 30+30.
mount_dx = 50;               // [STL] 가로(X)
mount_dy = 30;               // [STL] 길이(Y)
mount_gap = 136;             // [자] TOPLeft 뒷줄 나사 ↔ TOPRight 앞줄 나사 (PDF 8쪽)
mount_pitch = mount_dy + mount_gap;  // 166 = 두 패턴 중심 간격 (바깥나사 195.5와 0.5mm 차이로 검산됨)
front_row_y = 20;            // 설계값: 케이스 앞벽 → 앞 패턴 앞줄 나사

case_w = 151.9;              // [자] 차체 상판 가로(버니어). TOPRight(150)가 이 폭으로 문제없이 달려 있었다
mount_x = case_w / 2;        // [STL] TOPRight에서 패턴이 판 가운데(0.5mm 이내)
mount_front_c = front_row_y + mount_dy/2;
mount_rear_c  = mount_front_c + mount_pitch;
mount_centers = [[mount_x, mount_front_c], [mount_x, mount_rear_c]];

tray_h = 70;                 // 젯슨(보드+방열판+팬 ~40) + 받침 + 배선

/* ===================== 보드 ===================== */
// 아두이노 Uno + Moebius 쉴드 — 전면. [DS] 구멍 좌표, [STL] TOPLeft와 ±0.2 일치
uno_holes = [[13.97, 2.54], [15.24, 50.8], [66.04, 7.62], [66.04, 35.56]];
uno_pos = [44, 6];           // 차체 나사머리(앞 패턴) 위를 지나도록 받침을 높인다
uno_standoff_h = 8;          // 나사머리 2.4 + 보드 밑 납땜 핀 여유
standoff_sq = 7;             // [STL] TOPLeft 보스 7×7

// 젯슨 Orin Nano 개발자 키트 — 후면. 입출력 단자가 뒤쪽 긴 변에 오게 둔다.
jetson_size  = [100, 79];    // [DS]
jetson_holes = [[4, 4], [96, 4], [4, 75], [96, 75]];  // [실측] 캐리어 보드 구멍
jetson_pos = [(case_w - 100)/2, 140];  // 서보 걸이(회전 중심 아래) 뒤로 비켜 둔다
jetson_standoff_h = 6;

case_d = jetson_pos[1] + jetson_size[1] + 3 + wall;   // 젯슨 뒤 3mm 여유 = 225

// 벅부스트 2개 (12V→팔, 7.5V→바퀴) — 배선 슬롯이 떨어지는 왼쪽 가운데
buck_holes = [[3, 3], [40, 3], [3, 18], [40, 18]];   // [실측] 모듈 구멍
buck_pos = [[8, 66], [8, 100]];                      // [실측]

// 전압·전류계 — 오른쪽 벽 창으로 표시부를 밖에 보이게
meter_cut = [45.5, 26.5];    // [실측] 흔한 패널 규격(DSN-VC288류)
meter_y = 72; meter_z = 22;

// 바닥 카메라 (RPi Cam v2.1 / IMX219) — 전면 아래 선반, 바닥을 내려다봄
cam_board = [25, 24];        // [DS]
cam_holes = [[2, 2], [23, 2], [2, 14.5], [23, 14.5]];  // [DS] 21 × 12.5, M2
cam_lens_hole = 9;
cam_shelf = [36, 22];        // 선반 폭·앞으로 튀어나온 길이

// 뒷벽 단자 구멍 [x, z, w, h]: 젯슨 입출력 한 줄 + 충전 잭 + 전원 스위치
rear_ports = [
    [jetson_pos[0] + 4, floor_t + jetson_standoff_h, 92, 24],  // [실측] 젯슨 단자 띠
    [6,  20, 12, 12],                                            // [실측] DC 충전 잭 (Ø8 + 여유)
    [case_w - 26, 20, 20, 14],                                   // [실측] 로커 스위치 KCD1
];

/* ===================== 뚜껑 / 회전부 ===================== */
lid_t = 4;
lid_screw_inset = 7;

// 팔 받침이 지금 두 판 사이에 있다 → 회전 중심 = 두 패턴의 가운데
rot_center = [case_w/2, (mount_front_c + mount_rear_c)/2];

// 회전 베어링 (레이지수잔 구매품) — 팔 무게·모멘트를 받는다. 서보 축으로 받지 말 것.
brg_od  = 120;               // [실측]
brg_h   = 8;                 // [실측] = 뚜껑 윗면 ↔ 회전판 밑면 간격
brg_pcd_lower = 108; brg_pcd_upper = 92;  // [실측]
brg_hole = 3.4;

// 회전 서보 STS3215 (팔과 같은 부품·같은 버스)
servo_body = [45.2, 24.7, 35];   // [DS] 확인 요
servo_shaft_off = 11;            // [실측] 몸체 끝 → 출력축
horn_dia = 22;                   // [실측]
horn_pcd = 14; horn_hole = 2.2;  // [실측]

// 180° 기계 제한 — 배선이 감겨 끊기지 않게
travel_deg = 180;
stop_r = 40;  stop_pin_d = 5;  stop_groove_depth = 3;
stop_deg = 0;

// 배선 — 서보가 중심을 차지하므로 호형 슬롯. 서보 몸체(+X) 반대편, 벅부스트 위로 떨어진다.
cable_r = 28;  cable_w = 14;  cable_deg = 180;

/* ===================== 회전판 ===================== */
plat_dia = 150;  plat_t = 6;
so101_holes = [[-40, -30], [40, -30], [-40, 30], [40, 30]];  // [실측] SO-101 받침 바닥 구멍
so101_offset = [0, 5];           // [실측]
orbbec_pos = [0, -62];           // Orbbec 1/4"-20 자리 (−Y = 전면)
quarter_inch = 6.6;

brand_text = "ForNerds";  brand_size = 12;  brand_emboss = 0.8;

bed_x = 256; bed_y = 256;        // [실측] 프린터 베드

/* ===================== 검사 ===================== */
if (case_w > bed_x || case_d > bed_y) echo("⚠ 트레이/뚜껑이 베드를 넘는다", case_w, case_d);
if (plat_dia > min(bed_x, bed_y)) echo("⚠ 회전판이 베드를 넘는다");
if (cable_r + cable_w/2 > brg_pcd_upper/2 - 5) echo("⚠ 배선 슬롯이 베어링 볼트와 겹친다");
if (stop_r > brg_pcd_upper/2 - 5 || stop_r < cable_r + cable_w/2 + 4) echo("⚠ 스토퍼 반경 재조정");
if (mount_rear_c + mount_dy/2 > case_d - wall) echo("⚠ 뒤 패턴이 케이스 밖");
if (rot_center[1] + servo_body[1]/2 + wall > jetson_pos[1]) echo("⚠ 서보 걸이가 젯슨과 겹친다");
echo("케이스", case_w, "x", case_d, "x", tray_h + lid_t, " 회전중심", rot_center,
     " 패턴중심", mount_centers);

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
lid_corners = [for (x = [lid_screw_inset, case_w - lid_screw_inset],
                    y = [lid_screw_inset, case_d - lid_screw_inset]) [x, y]];

/* ----- 1) 트레이 ----- */
module tray() {
    difference() {
        union() {
            cube([case_w, case_d, tray_h]);
            // 바닥 카메라 선반 (앞벽 아래로 튀어나옴)
            translate([case_w/2 - cam_shelf[0]/2, -cam_shelf[1], 0]) cube([cam_shelf[0], cam_shelf[1] + wall, floor_t]);
        }
        translate([wall, wall, floor_t]) cube([case_w - 2*wall, case_d - 2*wall, tray_h]);
        for (c = mount_centers) holes_pattern(c, mount_dx, mount_dy, m3_clear, floor_t);
        // 카메라: 렌즈 구멍 + M2, 판은 선반 밑에 붙인다
        translate([case_w/2, -cam_shelf[1]/2, 0]) {
            translate([0, 0, -1]) cylinder(h = floor_t + 2, d = cam_lens_hole);
            translate([-cam_board[0]/2, -cam_board[1]/2 + 2, 0])
                for (h = cam_holes) translate([h[0], h[1], -1]) cylinder(h = floor_t + 2, d = m2_tap);
        }
        // 카메라 리본 통과 (선반 → 트레이 안)
        translate([case_w/2 - 9, -1, floor_t]) cube([18, wall + 2, 3]);
        // 환기: 젯슨 옆 양쪽 벽
        for (x = [-1, case_w - wall - 1], y = [jetson_pos[1] + 6 : 10 : jetson_pos[1] + jetson_size[1] - 8])
            translate([x, y, 18]) cube([wall + 2, 5, tray_h - 30]);
        // 전압·전류계 창 (오른쪽 벽)
        translate([case_w - wall - 1, meter_y, meter_z]) cube([wall + 2, meter_cut[0], meter_cut[1]]);
        for (p = rear_ports) translate([p[0], case_d - wall - 1, p[1]]) cube([p[2], wall + 2, p[3]]);
    }
    for (c = lid_corners) translate([c[0], c[1], 0])
        difference() {
            cylinder(h = tray_h, d = 9);
            translate([0, 0, tray_h - 12]) cylinder(h = 13, d = m3_tap);
        }
    bosses(uno_pos, uno_holes, uno_standoff_h);
    bosses(jetson_pos, jetson_holes, jetson_standoff_h);
    for (b = buck_pos) bosses(b, buck_holes, 5);
    translate([case_w/2, 0, tray_h/2 + 8]) rotate([90, 0, 0])
        linear_extrude(brand_emboss)
            text(brand_text, size = brand_size, halign = "center", valign = "center",
                 font = "Liberation Sans:style=Bold");
}

/* ----- 2) 뚜껑 ----- */
module lid() {
    difference() {
        cube([case_w, case_d, lid_t]);
        for (c = lid_corners) translate([c[0], c[1], -1]) cylinder(h = lid_t + 2, d = m3_clear);
        translate([rot_center[0], rot_center[1], 0]) {
            translate([0, 0, -1]) cylinder(h = lid_t + 2, d = horn_dia + 3);
            ring_holes(brg_pcd_lower, brg_hole, lid_t);
            translate([0, 0, -1]) arc(cable_r, cable_w, lid_t + 2, cable_deg - travel_deg/2, travel_deg);
            translate([0, 0, lid_t - stop_groove_depth])
                arc(stop_r, stop_pin_d + 1, stop_groove_depth + 1, stop_deg - travel_deg/2, travel_deg);
        }
        // 젯슨 위 환기 (회전판 뒤쪽 바깥)
        for (x = [0 : 8 : 88]) translate([jetson_pos[0] + 6 + x, rot_center[1] + plat_dia/2 + 4, -1])
            cube([4, case_d - (rot_center[1] + plat_dia/2 + 4) - 10, lid_t + 2]);
    }
    translate([rot_center[0] - servo_shaft_off, rot_center[1] - servo_body[1]/2, -servo_body[2]])
        difference() {
            translate([-wall, -wall, 0]) cube([servo_body[0] + 2*wall, servo_body[1] + 2*wall, servo_body[2]]);
            translate([-0.2, -0.2, -1]) cube([servo_body[0] + 0.4, servo_body[1] + 0.4, servo_body[2] + 2]);
            translate([servo_body[0] - 1, 4, -1]) cube([wall + 2, servo_body[1] - 8, 10]);
            // 긴 두 면만 관통 — 한 바퀴 자르면 띠가 뚜껑에서 떨어진다
            for (z = [8, 24]) translate([6, -wall - 1, z]) cube([servo_body[0] - 12, servo_body[1] + 2*wall + 2, 4]);
        }
}

/* ----- 3) 회전판 ----- */
module platform() {
    difference() {
        cylinder(h = plat_t, d = plat_dia, $fn = 120);
        ring_holes(horn_pcd, horn_hole, plat_t);
        translate([0, 0, -1]) cylinder(h = plat_t + 2, d = 3);
        ring_holes(brg_pcd_upper, brg_hole, plat_t);
        rotate(cable_deg) translate([cable_r, 0, -1]) cylinder(h = plat_t + 2, d = cable_w - 1);
        for (h = so101_holes) translate([so101_offset[0] + h[0], so101_offset[1] + h[1], -1])
            cylinder(h = plat_t + 2, d = m3_clear);
        translate([orbbec_pos[0], orbbec_pos[1], -1]) cylinder(h = plat_t + 2, d = quarter_inch);
    }
    rotate(stop_deg) translate([stop_r, 0, -(brg_h + stop_groove_depth - 0.5)])
        cylinder(h = brg_h + stop_groove_depth - 0.5, d = stop_pin_d);
}

/* ===================== 출력 ===================== */
if (part == "tray") tray();
else if (part == "lid") lid();
else if (part == "platform") platform();
else {
    color("lightgray") tray();
    color("silver", 0.8) translate([0, 0, tray_h]) lid();
    color("orange") translate([rot_center[0], rot_center[1], tray_h + lid_t + brg_h]) platform();
    %translate([rot_center[0], rot_center[1], tray_h + lid_t])
        difference() { cylinder(h = brg_h, d = brg_od); translate([0, 0, -1]) cylinder(h = brg_h + 2, d = brg_od - 30); }
}
