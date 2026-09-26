// tomato_case.scad — 토마토 로봇 3단 케이스 (메카넘 베이스 위 전장부 + 회전 플랫폼)
// 2026-09-25 스케치(docs/design/case-sketch-2026-09-24.jpg) 기반.
// 실측 전 단계: 아래 [PLACEHOLDER] 표시 값은 전부 추정치. 실측 나오면 이 블록만 고치면
// 전체 모델이 다시 맞춰진다 — 파일 다른 곳은 손댈 필요 없음.

/* ===================== 파라미터 ===================== */

wall = 3;           // 벽 두께 (FDM 적층 안정성 기준, 0.4mm 노즐 x 7.5회)
clearance = 1.0;     // 부품 삽입 여유

// --- 메카넘 베이스 상판 (섀시) --- [PLACEHOLDER: 실측 필요]
base_w = 220;         // ponytail: 추정치, 실측 후 교체
base_d = 180;         // ponytail: 추정치, 실측 후 교체
base_h = 70;          // 전장부 내부 높이 (배터리+젯슨 세워 넣을 여유)

// --- 전장부 고정판에 들어가는 부품 (실측 데이터시트 값) ---
// Jetson Orin Nano Dev Kit: 100 x 79 x 21 mm (공식 치수)
jetson_w = 100; jetson_d = 79; jetson_h = 21;
// Arduino Uno R3: 68.6 x 53.4 mm
ardu_w = 68.6; ardu_d = 53.4;
// 배터리 13.2V 팩 — [PLACEHOLDER] 실물 없어 표준 18650 4S 팩 크기로 추정
batt_w = 70; batt_d = 60; batt_h = 30;  // ponytail: 추정치, 실측 후 교체
// 전원 컨버터(Power Conv) — [PLACEHOLDER] 소형 DC-DC 벅 모듈 통상 크기
pconv_w = 50; pconv_d = 25;
// Fornords 카메라(원거리 고정) 마운트 — [PLACEHOLDER] Orbbec Astra류 바디 기준 추정
cam_fixed_w = 60; cam_fixed_d = 30; cam_fixed_h = 30;

// --- 회전 플랫폼 ---
// STS3215 서보(회전 액추에이터로 재사용, 기존 팔과 동일 부품): 40 x 20 x 40.5mm (공식 치수)
servo_w = 40; servo_d = 20; servo_h = 40.5;
platform_dia = 140;      // 회전판 지름 — 팔 SO-101 베이스 + 근접카메라 얹을 여유
platform_h = 8;          // 회전판 두께
turntable_bore = 8;      // 서보 혼 축 관통 구멍 지름 (혼 규격에 맞춰 조정)
stopper_deg = 180;        // 기계적 스토퍼 허용 회전각 (왕복 180도, 초과 시 배선 파손 방지)

// 배선 관통 통로 (회전축 중심, 팔전원/시리얼/D405 USB/서보신호 4가닥)
cable_bore = 18;          // 여유 루프 포함 관통 구멍 지름

/* ===================== 모듈 ===================== */

// 1) 하부 고정판 — 메카넘 베이스 위에 얹는 트레이. 배터리/전원/아두이노/젯슨/Fornords 카메라 마운트.
module base_tray() {
    difference() {
        // 바깥 셸
        cube([base_w, base_d, base_h]);
        // 내부 비움 (트레이형, 바닥+벽만 남김)
        translate([wall, wall, wall])
            cube([base_w - 2*wall, base_d - 2*wall, base_h]); // 위쪽 뚫림(뚜껑 없음, 방열)
    }

    // 부품 고정 보스(나사 기둥) — 배치는 스케치 순서(좌→우): Fornords cam / Servo / BAT / PowerConv / Ardu / Jetson
    // 좌표는 상판 내부 기준 원점(wall, wall)에서의 상대 배치, placeholder 치수 바뀌면 자동 재배치됨
    translate([wall + 10, wall + 10, 0])
        mount_pad(cam_fixed_w, cam_fixed_d);

    translate([wall + 10 + cam_fixed_w + 15, wall + 10, 0])
        mount_pad(ardu_w, ardu_d);

    translate([base_w/2 - batt_w/2, base_d - wall - batt_d - 10, 0])
        mount_pad(batt_w, batt_d);

    translate([base_w - wall - jetson_w - 10, wall + 10, 0])
        mount_pad(jetson_w, jetson_d);

    translate([base_w - wall - pconv_w - 10, base_d - wall - pconv_d - 10, 0])
        mount_pad(pconv_w, pconv_d);
}

// 부품 하나당 4귀퉁이 M3 보스
module mount_pad(w, d, boss_h = 6, boss_dia = 6, hole_dia = 2.6) {
    inset = 4;
    for (x = [inset, w - inset])
        for (y = [inset, d - inset])
            translate([x, y, 0])
                difference() {
                    cylinder(h = boss_h, d = boss_dia, $fn = 24);
                    cylinder(h = boss_h, d = hole_dia, $fn = 24); // M3 self-tap
                }
}

// 2) 회전축 허브 — base_tray 중앙에 서보를 세워 넣고, 배선이 지나갈 중공축.
module rotation_hub() {
    hub_dia = servo_h + 10; // 서보 감싸는 원통
    difference() {
        union() {
            cylinder(h = base_h, d = hub_dia, $fn = 60);
        }
        // 배선 관통 보어 (중심)
        translate([0, 0, -1])
            cylinder(h = base_h + 2, d = cable_bore, $fn = 40);
        // 서보 포켓 (사각)
        translate([-servo_w/2, -servo_d/2, base_h - servo_h])
            cube([servo_w, servo_d, servo_h + 1]);
    }
}

// 3) 회전 플랫폼 — 팔 SO-101 베이스 + 근접 카메라(D405) 마운트, 180도 스토퍼 포함.
module rotation_platform() {
    difference() {
        union() {
            cylinder(h = platform_h, d = platform_dia, $fn = 90);
            // 스토퍼 핀 (하부 턱에 걸림, stopper_deg 왕복 제한)
            translate([platform_dia/2 - 6, 0, platform_h])
                cylinder(h = 6, d = 5, $fn = 20);
        }
        // 축 관통 + 배선 보어
        translate([0, 0, -1])
            cylinder(h = platform_h + 2, d = cable_bore, $fn = 40);
        // 서보 혼 체결 구멍(원형 패턴, 혼 규격에 맞춰 개수/반경 조정 필요)
        for (a = [0:60:300])
            rotate([0, 0, a])
                translate([turntable_bore/2 + 4, 0, -1])
                    cylinder(h = platform_h + 2, d = 2.2, $fn = 12);
    }
}

// 하부 트레이에 스토퍼 걸림턱 — rotation_platform의 핀이 이 턱 안쪽에서만 움직이게 제한
module stopper_wall() {
    // stopper_deg(기본 180) 만큼만 열어둔 링 벽. 나머지 구간은 벽으로 막아 핀이 못 지나감.
    ring_r = platform_dia/2 - 6;
    difference() {
        cylinder(h = 10, r = ring_r + 3, $fn = 90);
        cylinder(h = 10, r = ring_r - 3, $fn = 90);
        // 열린 구간(stopper_deg)만 잘라냄 — 나머지가 막힌 벽(스토퍼)
        rotate([0, 0, -stopper_deg/2])
            rotate_extrude(angle = stopper_deg, $fn = 90)
                translate([ring_r - 3, 0])
                    square([6, 10]);
    }
}

/* ===================== 조립 미리보기 ===================== */
color("lightgray") base_tray();
translate([base_w/2, base_d/2, 0]) {
    color("gray") rotation_hub();
    color("orange") translate([0, 0, base_h]) stopper_wall();
    color("lightgreen") translate([0, 0, base_h + 10]) rotation_platform();
}
