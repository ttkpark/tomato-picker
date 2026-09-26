// tomato_case.scad — 토마토 로봇 케이스: 전장 트레이 + 뚜껑 + 180° 회전판
// 스케치 docs/design/case-sketch-2026-09-24.jpg · 실측 docs/design/3D-parts-mapping.md
//
// 좌표: X=좌우, Y=앞뒤(Y=0 면이 전면=팔이 보는 쪽), Z=위. 원점=트레이 바닥 앞-왼 모서리.
// 부품별 STL 뽑기: openscad -D 'part="tray"' -o tray.stl tomato_case.scad
//                  (part = "assembly" | "tray" | "lid" | "platform")
//
// 값의 출처 표시:
//   [STL]  3D/ 폴더 원본을 잘라 잰 값 — 믿어도 됨
//   [DS]   부품 데이터시트 값
//   [실측] 아직 모름 — 자로 재서 고칠 것 (지금 값은 형상을 보기 위한 가짜)

part = "assembly";

/* ===================== 공통 ===================== */
wall   = 3;      // [STL] 기존 파츠 벽·판 두께와 동일
floor_t = 3;     // [STL]
m3_clear = 3.4;  // [STL] 통과 구멍 (TOPRight·BOTTOM이 3.4 / TOPLeft 3.22)
m3_tap   = 2.9;  // [STL] 셀프탭 (TOPLeft 아두이노 보스 2.93)
$fn = 48;

/* ===================== 차체 / 트레이 ===================== */
chassis_w = 160;   // [실측] 차체 상판 가로 (X)
chassis_d = 240;   // [실측] 차체 상판 세로 (Y)
tray_h    = 70;    // 젯슨+방열팬 높이 여유 (젯슨 보드 21 + 팬·방열판 ~30 + 배선)

// 차체 상판 체결 패턴 — [STL] TOPLeft·TOPRight 공용: M3, 50 × 30
mount_dx = 50;  mount_dy = 30;
mount_centers = [[chassis_w/2, 60], [chassis_w/2, 180]];  // [실측] 두 벌의 중심 위치

// 프린터 베드 (넘으면 콘솔 경고) — [실측] 쓰는 프린터
bed_x = 256; bed_y = 256;

/* ===================== 보드 ===================== */
// Arduino Uno R3 — [DS] 공식 구멍 좌표, [STL] TOPLeft에서 ±0.2 일치 확인
uno_holes = [[13.97, 2.54], [15.24, 50.8], [66.04, 7.62], [66.04, 35.56]];
uno_pos = [10, 150];              // [실측] 보드 원점(좌하) 위치
standoff_sq = 7; standoff_h = 4;  // [STL] TOPLeft 보스 7×7, 높이 4

// Jetson Orin Nano 개발자 키트 — [DS] 보드 100 × 79
jetson_holes = [[4, 4], [96, 4], [4, 75], [96, 75]];  // [실측] 캐리어 보드 구멍
jetson_pos = [30, 6];             // [실측] 서보 걸이(회전 중심 아래)와 겹치지 않게
jetson_standoff_h = 6;            // 밑면 부품 여유

// 배터리 13.2V 팩 — 낮은 벽으로 둘러 밴드로 묶음
batt_size = [70, 60, 30];         // [실측]
batt_pos  = [84, 150];            // [실측]
batt_fence_h = 12;

// 전원 컨버터
pconv_holes = [[3, 3], [47, 3], [3, 22], [47, 22]];  // [실측]
pconv_pos = [10, 110];            // [실측]

// 뒷벽 포트 구멍 (전원 스위치·USB·충전) — [실측] [x, z, w, h]
rear_ports = [[20, 15, 30, 20], [100, 15, 40, 25]];

/* ===================== 뚜껑 / 회전부 ===================== */
lid_t = 4;
lid_screw_inset = 7;

rot_center = [chassis_w/2, 110];  // [실측] 회전 중심 — 무게중심상 차체 가운데 근처가 유리

// 회전 베어링 (레이지수잔 구매품) — 팔 무게·모멘트를 여기서 받는다. 서보 축으로 받지 말 것.
brg_od  = 120;   // [실측] 바깥 지름
brg_h   = 8;     // [실측] 두께 = 뚜껑 윗면과 회전판 밑면 사이 간격
brg_pcd_lower = 108; brg_pcd_upper = 92;  // [실측] 하/상 링 볼트원
brg_hole = 3.4;

// 회전 서보 — STS3215 (팔과 같은 부품, 드라이버 공유)
servo_body = [45.2, 24.7, 35];    // [DS] STS3215 몸체 (확인 요)
servo_shaft_off = 11;             // [실측] 몸체 끝→출력축 거리
horn_dia = 22;                    // [실측] 원판 혼 지름
horn_pcd = 14; horn_hole = 2.2;   // [실측] 혼 나사원·나사(M2)

// 180° 제한 — 배선이 감겨 끊어지지 않게 기계적으로 막는다
travel_deg = 180;
stop_r = 40;  stop_pin_d = 5;  stop_groove_depth = 3;

// 배선 통로 — 서보가 중심을 차지하므로 중심 대신 호형 슬롯. 판의 구멍이 이 슬롯을 따라 돈다.
// 서보 몸체가 +X로 뻗으므로 슬롯은 반대편(−X, 180°) 가운데.
cable_r = 28;  cable_w = 14;  cable_deg = 180;
stop_deg = 0;  // 스토퍼 홈 가운데 방향

/* ===================== 회전판 ===================== */
plat_dia = 150;  plat_t = 6;
so101_holes = [[-40, -30], [40, -30], [-40, 30], [40, 30]];  // [실측] SO-101 베이스 바닥 구멍(회전 중심 기준)
so101_offset = [0, 5];            // [실측] 팔 베이스 위치
orbbec_pos = [0, -62];            // Orbbec 1/4"-20 삼각대 나사 자리 (전면 가장자리)
quarter_inch = 6.6;

// 브랜드
brand_text = "ForNerds";  brand_size = 12;  brand_emboss = 0.8;

/* ===================== 검사 ===================== */
if (chassis_w > bed_x || chassis_d > bed_y)
    echo("⚠ 트레이/뚜껑이 베드를 넘는다 — 반으로 나눠 출력할 것", chassis_w, chassis_d);
if (plat_dia > min(bed_x, bed_y)) echo("⚠ 회전판이 베드를 넘는다");
if (cable_r + cable_w/2 > brg_pcd_upper/2 - 5) echo("⚠ 배선 슬롯이 베어링 볼트와 겹친다");
if (stop_r > brg_pcd_upper/2 - 5 || stop_r < cable_r + cable_w/2 + 4) echo("⚠ 스토퍼 반경 재조정");

/* ===================== 조각 ===================== */
module holes_pattern(c, dx, dy, d, h) {
    for (sx = [-1, 1], sy = [-1, 1])
        translate([c[0] + sx*dx/2, c[1] + sy*dy/2, -1]) cylinder(h = h + 2, d = d);
}
module ring_holes(pcd, d, h, n = 4, a0 = 45) {
    for (i = [0:n-1]) rotate(a0 + i*360/n) translate([pcd/2, 0, -1]) cylinder(h = h + 2, d = d);
}
module arc(r, w, h, a0, span) {  // 호형 띠 (a0에서 span만큼)
    rotate(a0) rotate_extrude(angle = span) translate([r - w/2, 0]) square([w, h]);
    for (a = [a0, a0 + span]) rotate(a) translate([r, 0, 0]) cylinder(h = h, d = w);
}
module boss(h, hole) {
    difference() {
        translate([-standoff_sq/2, -standoff_sq/2, 0]) cube([standoff_sq, standoff_sq, h]);
        translate([0, 0, 0.6]) cylinder(h = h, d = hole);  // 바닥 0.6 막음 — 아래로 안 샌다
    }
}
lid_corners = [for (x = [lid_screw_inset, chassis_w - lid_screw_inset],
                    y = [lid_screw_inset, chassis_d - lid_screw_inset]) [x, y]];

/* ----- 1) 트레이 ----- */
module tray() {
    difference() {
        cube([chassis_w, chassis_d, tray_h]);
        translate([wall, wall, floor_t]) cube([chassis_w - 2*wall, chassis_d - 2*wall, tray_h]);
        // 차체 체결 (기존 나사 자리)
        for (c = mount_centers) holes_pattern(c, mount_dx, mount_dy, m3_clear, floor_t);
        // 좌우 환기 슬롯
        for (x = [-1, chassis_w - wall - 1], y = [30 : 12 : chassis_d - 40])
            translate([x, y, 20]) cube([wall + 2, 6, tray_h - 32]);
        // 뒷벽 포트
        for (p = rear_ports) translate([p[0], chassis_d - wall - 1, p[1]]) cube([p[2], wall + 2, p[3]]);
    }
    // 뚜껑 나사 기둥
    for (c = lid_corners) translate([c[0], c[1], 0])
        difference() {
            cylinder(h = tray_h, d = 9);
            translate([0, 0, tray_h - 12]) cylinder(h = 13, d = m3_tap);
        }
    // 보드 기둥
    translate([uno_pos[0], uno_pos[1], floor_t])
        for (h = uno_holes) translate(h) boss(standoff_h, m3_tap);
    translate([jetson_pos[0], jetson_pos[1], floor_t])
        for (h = jetson_holes) translate(h) boss(jetson_standoff_h, m3_tap);
    translate([pconv_pos[0], pconv_pos[1], floor_t])
        for (h = pconv_holes) translate(h) boss(standoff_h, m3_tap);
    // 배터리 울타리 (앞뒤로 밴드 슬롯)
    translate([batt_pos[0], batt_pos[1], floor_t]) difference() {
        translate([-wall, -wall, 0]) cube([batt_size[0] + 2*wall, batt_size[1] + 2*wall, batt_fence_h]);
        cube([batt_size[0], batt_size[1], batt_fence_h + 1]);
        translate([batt_size[0]/2 - 10, -wall - 1, 3]) cube([20, batt_size[1] + 2*wall + 2, 3]);
    }
    // 브랜드 양각 (전면 외벽)
    translate([chassis_w/2, 0, tray_h/2]) rotate([90, 0, 0])
        linear_extrude(brand_emboss)
            text(brand_text, size = brand_size, halign = "center", valign = "center",
                 font = "Liberation Sans:style=Bold");
}

/* ----- 2) 뚜껑 (회전 베어링·서보 받침) ----- */
module lid() {
    difference() {
        cube([chassis_w, chassis_d, lid_t]);
        for (c = lid_corners) translate([c[0], c[1], -1]) cylinder(h = lid_t + 2, d = m3_clear);
        translate([rot_center[0], rot_center[1], 0]) {
            translate([0, 0, -1]) cylinder(h = lid_t + 2, d = horn_dia + 3);      // 혼 통과
            ring_holes(brg_pcd_lower, brg_hole, lid_t);                           // 베어링 하부 링
            translate([0, 0, -1]) arc(cable_r, cable_w, lid_t + 2, cable_deg - travel_deg/2, travel_deg);
            translate([0, 0, lid_t - stop_groove_depth])                          // 스토퍼 홈
                arc(stop_r, stop_pin_d + 1, stop_groove_depth + 1, stop_deg - travel_deg/2, travel_deg);
        }
        // 젯슨 위 환기
        for (x = [0 : 8 : 90]) translate([jetson_pos[0] + 5 + x, jetson_pos[1] + 10, -1]) cube([4, 50, lid_t + 2]);
    }
    // 서보 걸이 (뚜껑 밑, 출력축이 회전 중심에 오게)
    translate([rot_center[0] - servo_shaft_off, rot_center[1] - servo_body[1]/2, -servo_body[2]])
        difference() {
            translate([-wall, -wall, 0]) cube([servo_body[0] + 2*wall, servo_body[1] + 2*wall, servo_body[2]]);
            translate([-0.2, -0.2, -1]) cube([servo_body[0] + 0.4, servo_body[1] + 0.4, servo_body[2] + 2]);
            // 선 빠질 틈 + 케이블타이 슬롯
            translate([servo_body[0] - 1, 4, -1]) cube([wall + 2, servo_body[1] - 8, 10]);
            // 긴 두 면만 관통, 모서리 기둥은 남긴다 (한 바퀴 자르면 띠가 뚜껑에서 떨어진다)
            for (z = [8, 24]) translate([6, -wall - 1, z]) cube([servo_body[0] - 12, servo_body[1] + 2*wall + 2, 4]);
        }
}

/* ----- 3) 회전판 (팔 + Orbbec) ----- */
module platform() {
    difference() {
        cylinder(h = plat_t, d = plat_dia, $fn = 120);
        ring_holes(horn_pcd, horn_hole, plat_t);                                 // 서보 혼
        translate([0, 0, -1]) cylinder(h = plat_t + 2, d = 3);                   // 혼 가운데 나사
        ring_holes(brg_pcd_upper, brg_hole, plat_t);                             // 베어링 상부 링
        rotate(cable_deg) translate([cable_r, 0, -1]) cylinder(h = plat_t + 2, d = cable_w - 1);  // 배선 구멍
        for (h = so101_holes) translate([so101_offset[0] + h[0], so101_offset[1] + h[1], -1])
            cylinder(h = plat_t + 2, d = m3_clear);
        translate([orbbec_pos[0], orbbec_pos[1], -1]) cylinder(h = plat_t + 2, d = quarter_inch);
    }
    // 스토퍼 핀 (밑면, 뚜껑 홈에 들어감)
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
    %translate([rot_center[0], rot_center[1], tray_h + lid_t])                   // 베어링 (구매품, 유령)
        difference() { cylinder(h = brg_h, d = brg_od); translate([0, 0, -1]) cylinder(h = brg_h + 2, d = brg_od - 30); }
}
