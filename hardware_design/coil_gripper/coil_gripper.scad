// coil_gripper.scad — 제본 코일용 집게 덧판 (2026-11-15)
// 지금 달린 SO-101 집게 두 손가락을 그대로 두고, 안쪽 면에 얹어 케이블타이 2줄로 묶는다.
//   flat  = 평판. 앞면에 안감을 붙인다: 우드락 5mm(foam=5, 기본) 또는 스펀지 테이프 1mm(-D foam=1, 움직이는 쪽만 다시 뽑는다)
//   round = 묶음 단면을 따라 굽은 받침. 옆을 누르면서 아래를 받친다
//   wedge = 손끝 쐐기. 손가락 끝 계단 3칸에만 얹는 짧은 판(기본 79.6 × 33.6)에 우드락 토막(76 × 30)이 들어가는 2mm 턱(포켓)을 둘렀다.
//   j     = J자(11/13 스케치 2안). 곧은 벽으로 옆을 누르고 아래 호(j_lip)로 받친다. round에서 위로 감싸는 호를 뺀 것
//           11/15 손으로 만든 '우드락 손끝 덧댐'의 인쇄판 — 움직이는 쪽에 쐐기를 넣어 물었을 때 우드락 두 면이 평행이 된다. 타이 1줄
//
// STL: openscad -D 'part="flat_fixed"' -o stl/flat_fixed.stl coil_gripper.scad
//   part = flat_fixed | flat_moving | round_fixed | round_moving | wedge_fixed | wedge_moving | j_fixed | j_moving   (인쇄물, 판 앞면이 바닥)
//          asm_flat | asm_round | asm_wedge | asm_j        (손가락·코일과 같이 본 조립 모습)
//          check_flat | check_round | check_wedge | check_j  (덧판과 손가락이 겹치는 곳만 남긴다. 비어 있어야 정상)
//
// 좌표 = Wrist_Roll_Follower_SO101.stl 그대로: x = 움직이는 집게 쪽, y = 집게 회전축, z = 손끝 쪽.
// 값의 출처: [STL] ref/ 의 공식 SO-101 STL을 잘라 잰 값 · [URDF] 공식 so101_new_calib
//            [추정] 실측 전. 인쇄 전에 README "인쇄 전에 잴 것"을 본다

part = "asm_flat";
chk = 0;          // check_*에서 0 = 셋 다, 1 = 고정판↔고정 손가락, 2 = 움직이는 판↔움직이는 손가락, 3 = 판끼리
$fn = 64;

/* ============ 잡는 물건 · 재료 ============ */
grip_w  = 18;     // [추정] 4개 묶음 폭(사진 17.3~18.5). 두 판이 이 폭에서 평행이 된다
grip_h  = 12.5;   // [추정] 4개 묶음 높이(사진 11.8~12.9) — round만 쓴다
foam    = 5;      // flat 안감 두께(한쪽): 우드락 5 / 스펀지 테이프 1. 움직이는 쪽 쐐기 각이 이 값으로 정해진다(고정 쪽은 같다)
plate_w = 160;    // flat 판 길이(회전축 방향). 우드락 300을 가운데 160에서 받친다
plate_h = 45;     // flat 넓은 판 높이(손끝에서 위로)
j_lip   = 50;     // j: 아래로 받치는 호의 각(스케치 값). 크면 잘 받치지만 더 벌려야 놓인다
j_wall  = 34;     // j: 묶음 중심에서 위로 올라가는 곧은 벽 높이(스케치 값)
wedge_foam = [76, 30];  // wedge: 우드락 토막 크기 [길이(회전축 방향), 높이]. 두께는 foam. 높이를 키우면 움직이는 판이 손가락 뿌리 쪽에 닿는다(assert가 막는다)
pocket_d = 2;     // wedge: 턱 높이(판 앞면에서 솟는 양). 우드락 두께보다 낮아야 한다
rim_t  = 1.5;     // wedge: 턱 너비
pocket_clr = 0.3; // wedge: 우드락과 턱 사이 여유(한쪽)
wedge_over = 8;   // wedge: 판이 고정 손가락 끝(105.4)보다 내려오는 길이
round_len = 180;  // round 받침 길이. 베드 220에 들어가는 값
round_gap = 0.5;  // round 안쪽 여유(한쪽)
arc_up  = 70;     // round: 옆에서 위로 감싸는 각
arc_lip = 45;     // round: 아래로 받치는 각. 크면 잘 받치지만 더 벌려야 놓인다

/* ============ 인쇄 ============ */
t      = 3;       // 판 두께
tr     = 2;       // round 받침 벽
clr    = 0.3;     // 손가락과의 틈(한쪽)
cheek  = 2.4;     // 손가락 옆을 잡는 볼 두께
cheek_d = 7;      // 볼 높이(안쪽 면에서 손가락 등 쪽으로). 손가락 등보다 낮아야 타이가 등을 누른다
tie_w  = 5.5;     // 케이블타이 홈 폭(타이 폭 4.8 이하)
tie_t  = 2.4;     // 타이 구멍 두께 방향
tie_d  = 1.4;     // 판 앞면 타이 홈 깊이 — 타이가 앞면 위로 안 올라온다
is_wedge = (part == "wedge_fixed" || part == "wedge_moving" || part == "asm_wedge" || part == "check_wedge");
z_end  = is_wedge ? 105.4 + wedge_over : 109;   // 덧판 아래 끝. 손가락 끝(고정 105.4)보다 길어야 바닥에 손가락이 먼저 안 닿는다

/* ============ SO-101 집게 [STL][URDF] ============ */
PIV = [20.2, 24.35];        // [URDF] 집게 회전축 (x, z)
// 안쪽 면 계단 [시작 r, u]. r = 손끝 방향 거리, u = 덧판 쪽이 −인 두께 방향.
// 고정 손가락: r = z, u = −x.  움직이는 손가락: r = 회전축에서의 거리, u = 손가락 자체 x.
F_STEP = [[40, 15.0], [46, 13.3], [66, 11.6], [85, 9.9], [95, 7.9]];
M_STEP = [[22.3, -8.3], [62, -10.3], [72, -12.3]];
// 반폭 [r, hw]
F_HW = [[44,12.7],[46,11.9],[50,10.8],[55,9.85],[60,9.2],[66,8.6],[75,7.97],[85,7.2],
        [90,6.7],[92.5,6.37],[95,5.95],[97.5,5.45],[100,4.75]];
M_HW = [[25,10.95],[26,10.65],[30,9.87],[35,9.2],[40,8.7],[50,7.98],[60,7.28],[66,6.72],
        [70,6.18],[72,5.85],[74,5.43],[76,4.9]];
// wedge는 끝 계단 3칸만 쓴다: 고정 r 76~(틈 3.7 · 2.0 · 0), 움직이는 쪽 r 53~. 타이는 맨 위 칸에 1줄
F_R0 = is_wedge ? 76 : 44;  F_R1 = 100;  F_TIE = is_wedge ? [80.5] : [52, 74];
M_R0 = is_wedge ? 53 : 40;  M_R1 = 76;   M_TIE = is_wedge ? [57.5] : [46, 68];   // 뿌리(r 25)부터 얹으면 벌림각이 33°로 커진다 — 40부터
FS = is_wedge ? [[F_R0, 11.6], [85, 9.9], [95, 7.9]] : F_STEP;   // 안장에 쓰는 고정 쪽 계단
M_BODY_END = 84;   // 움직이는 쪽 몸통은 손가락 끝(82)에서 끊는다

/* ============ 유도값 ============ */
is_j = (part == "j_fixed" || part == "j_moving" || part == "asm_j" || part == "check_j");
assert(is_num(j_lip) && j_lip > 0 && j_lip < 90, "j_lip: 0 초과 90 미만의 각이어야 한다");
assert(is_num(j_wall) && j_wall > 0, "j_wall: 0보다 커야 한다");
lip = is_j ? j_lip : arc_lip;   // 아래로 받치는 각
up  = is_j ? 0 : arc_up;        // 위로 감싸는 각 — j는 곧은 벽(판)이 대신한다
is_round = is_j || (part == "round_fixed" || part == "round_moving" || part == "asm_round" || part == "check_round");   // j도 round의 받침·치수를 쓴다
ra = grip_w/2 + round_gap;                 // round 안쪽 반축(가로)
rb = grip_h/2 + round_gap;                 // round 안쪽 반축(세로)
xb = -F_STEP[4][1] + clr;                  // 고정 판 뒷면
xf = xb + t;                               // 고정 판 앞면
gap = is_round ? 2*ra : 2*foam + grip_w;   // 두 판 앞면 사이
xm = xf + gap;                             // 움직이는 판 앞면
zc = z_end - rb;                           // round: 묶음 중심 높이(묶음 바닥 = z_end)
xc = xf + ra;

// 물었을 때 집게가 벌어진 각. 판 두께 t가 손가락 안쪽 면의 모든 계단 모서리 앞에 들어가는 가장 작은 각.
function th_i(u, r) = asin((xm + t + clr - PIV[0]) / sqrt(u*u + r*r)) - atan2(u, r);
theta = max(th_i(M_STEP[0][1], M_R0), th_i(M_STEP[1][1], M_STEP[1][0]), th_i(M_STEP[2][1], M_STEP[2][0]));
function mz(u, r) = PIV[1] - u*sin(theta) + r*cos(theta);      // 움직이는 손가락 위의 점 → z
zm0 = mz(M_STEP[0][1], M_R0);
neck_w = 2*(F_HW[0][1] + clr + cheek + 0.4 + tie_t + 3);
// 움직이는 판은 zm0(안장 시작 높이)보다 위로 못 올라간다 — 그 위는 손가락 안쪽 면이 판 뒷면보다 안쪽에 있다(theta는 r ≥ M_R0 모서리만 피한다)
if (is_wedge) assert(wz0 >= zm0 - 0.01, str("wedge: 판 위 끝(", wz0, ")이 안장 시작(", zm0, ")보다 높다 — wedge_foam[1]을 줄이거나 wedge_over를 키워라"));
echo(str("theta(물었을 때 벌림각) = ", theta, " deg, 판 사이 = ", gap, " mm, 좁은 목 폭 = ", neck_w));

/* ============ 손가락 한 개에 얹는 안장 (자기 좌표: X=u, Y=y, Z=r) ============ */
// 덧판 쪽 공간: 안쪽 면(계단)보다 clr 만큼 물러난 반공간
module pad_side(step, r0) {
    pts = concat(
        [[-90, r0], [step[0][1] - clr, r0]],
        [for (i = [1 : len(step)-1]) each [[step[i-1][1] - clr, step[i][0] - clr], [step[i][1] - clr, step[i][0] - clr]]],
        [[step[len(step)-1][1] - clr, 150], [-90, 150]]);
    rotate([90, 0, 0]) linear_extrude(200, center = true) polygon(pts);
}
// 손가락 폭 윤곽을 d 만큼 키운 기둥(u 방향으로 −90 ~ umax)
module width_env(hw, r1, d, umax) {
    n = len(hw);
    right = [for (p = hw) if (p[0] <= r1) [p[0], p[1] + d]];
    last = right[len(right)-1];
    pts = concat(right, [[150, last[1]], [150, -last[1]]], [for (i = [len(right)-1 : -1 : 0]) [right[i][0], -right[i][1]]]);
    translate([umax, 0, 0]) rotate([0, -90, 0]) linear_extrude(umax + 90) polygon(pts);
}
module saddle(step, hw, r0, r1, rend = 150) {
    umax = step[0][1] + cheek_d;
    // 몸통: 계단을 메워 판까지
    intersection() { pad_side(step, r0); width_env(hw, r1, clr + cheek, 60); translate([-90, -50, r0]) cube([200, 100, rend - r0]); }
    // 볼: 손가락 양옆
    difference() {
        intersection() {
            width_env(hw, r1, clr + cheek, umax);
            translate([-90, -50, r0]) cube([200, 100, r1 - r0]);
        }
        width_env(hw, r1, clr, 60);
    }
}

/* ============ 판 · 타이 ============ */
// dir = +1: 판이 앞면에서 +x 쪽으로 두껍다(움직이는 쪽), −1: −x 쪽(고정 쪽)
module plate(xfront, dir, z0, wide_w, wide_z0, wide_z1) {
    x0 = dir > 0 ? xfront : xfront - t;
    translate([x0, -neck_w/2, z0]) cube([t, neck_w, max(wide_z0 - z0, 0.01)]);
    translate([x0, -wide_w/2, wide_z0]) cube([t, wide_w, wide_z1 - wide_z0]);
}
module tie_cut(xfront, dir, z, hw_here) {
    ys = hw_here + clr + cheek + 0.4;
    x0 = dir > 0 ? xfront - 1 : xfront - t - 1;
    for (s = [-1, 1]) translate([x0, s > 0 ? ys : -ys - tie_t, z - tie_w/2]) cube([t + 2, tie_t, tie_w]);
    translate([dir > 0 ? xfront - 1 : xfront - tie_d, -(ys + tie_t), z - tie_w/2]) cube([tie_d + 1, 2*(ys + tie_t), tie_w]);
}
module rim(xfront, side) {
    x0 = side < 0 ? xfront - 0.01 : xfront - pocket_d;
    difference() {
        translate([x0, -ww/2, wz0]) cube([pocket_d + 0.01, ww, wedge_h]);
        translate([x0 - 1, -ww/2 + rim_t, wz0 + rim_t]) cube([pocket_d + 3, ww - 2*rim_t, wedge_h - 2*rim_t]);
    }
}
// round 받침 단면 (x, z). side = −1 고정 쪽, +1 움직이는 쪽
module cradle(side) {
    n = 40;
    function p(R1, R2, a) = [xc + side * R1 * cos(a), zc + R2 * sin(a)];
    outer = [for (i = [0 : n]) p(ra + tr, rb + tr, -up + (up + lip) * i / n)];
    inner = [for (i = [n : -1 : 0]) p(ra, rb, -up + (up + lip) * i / n)];
    rotate([90, 0, 0]) linear_extrude(round_len, center = true) polygon(concat(outer, inner));
}
wz1 = is_round ? zc + (rb + tr) * sin(lip) * 0.6 : z_end;   // 넓은 판 아래 끝
wedge_h = wedge_foam[1] + 2*(rim_t + pocket_clr);
wz0 = is_round ? zc - (is_j ? j_wall : 26) : is_wedge ? z_end - wedge_h : z_end - plate_h;                      // 넓은 판 위 끝
ww  = is_round ? round_len : is_wedge ? wedge_foam[0] + 2*(rim_t + pocket_clr) : plate_w;

module fixed_part() {
    difference() {
        union() {
            intersection() {
                mirror([1, 0, 0]) saddle(FS, F_HW, F_R0, F_R1);
                translate([-60, -100, 0]) cube([60 + xf, 200, wz1]);
            }
            plate(xf, -1, F_R0, ww, wz0, wz1);
            if (is_round) cradle(-1);
            if (is_wedge) rim(xf, -1);
        }
        for (z = F_TIE) tie_cut(xf, -1, z, lookup(z, F_HW));
    }
}
module moving_part() {       // 물었을 때 자세(theta)로 그린다
    difference() {
        union() {
            intersection() {
                translate([PIV[0], 0, PIV[1]]) rotate([0, theta, 0]) saddle(M_STEP, M_HW, M_R0, M_R1, M_BODY_END);
                translate([xm, -100, 0]) cube([90, 200, wz1]);
            }
            plate(xm, +1, zm0, ww, max(wz0, zm0), wz1);
            if (is_round) cradle(+1);
            if (is_wedge) rim(xm, +1);
        }
        for (r = M_TIE) tie_cut(xm, +1, mz(M_STEP[0][1], r), lookup(r, M_HW));
    }
}

/* ============ 참고 형상 ============ */
module jaw_fixed()  { import("ref/Wrist_Roll_Follower_SO101.stl", convexity = 6); }
module jaw_moving() { translate([PIV[0], 0, PIV[1]]) rotate([0, theta, 0]) rotate([-90, 0, 0]) import("ref/Moving_Jaw_SO101.stl", convexity = 6); }
module bundle() {
    color("white") translate([xf + gap/2, 0, z_end - grip_h/2]) rotate([90, 0, 0])
        scale([grip_w/2, grip_h/2, 1]) cylinder(r = 1, h = 290, center = true);
}
module foams() {
    fl = is_wedge ? wedge_foam[0] : foam >= 3 ? 300 : plate_w;  fh = is_wedge ? wedge_foam[1] : foam >= 3 ? 50 : plate_h;   // 우드락은 300 × 50, 테이프는 판 크기대로, wedge는 포켓 안
    zb = is_wedge ? rim_t + pocket_clr : 0;
    color("khaki", 0.7) for (x = [xf, xm - foam]) translate([x, -fl/2, z_end - zb - fh]) cube([foam, fl, fh]);
}
module asm() {
    color("dimgray") jaw_fixed();
    color("gray") jaw_moving();
    color("orange") fixed_part();
    color("tomato") moving_part();
    bundle();
    if (!is_round) foams();
}

/* ============ 출력 ============ */
rim_h = is_wedge ? pocket_d : 0;   // wedge는 턱 끝이 바닥, 판은 2mm 떠서 다리(브리지)로 뽑힌다
if (part == "flat_fixed" || part == "round_fixed" || part == "wedge_fixed" || part == "j_fixed")
    translate([0, 0, xf + rim_h]) rotate([0, 90, 0]) fixed_part();            // 앞면(x = xf)이 바닥
else if (part == "flat_moving" || part == "round_moving" || part == "wedge_moving" || part == "j_moving")
    translate([0, 0, -xm + rim_h]) rotate([0, -90, 0]) moving_part();         // 앞면(x = xm)이 바닥
else if (part == "asm_flat" || part == "asm_round" || part == "asm_wedge" || part == "asm_j") asm();
else if (part == "check_flat" || part == "check_round" || part == "check_wedge" || part == "check_j") {
    if (chk == 0 || chk == 1) intersection() { fixed_part(); jaw_fixed(); }
    if (chk == 0 || chk == 2) intersection() { moving_part(); jaw_moving(); }
    if (chk == 0 || chk == 3) intersection() { fixed_part(); moving_part(); }
}
