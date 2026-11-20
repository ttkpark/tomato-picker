// sketch_b.scad — 11/13 손 스케치(분리조립형 J자: 슬리브 + 긴 핑거)를 실제 집게 형상에 얹어 본 그림. 인쇄물 아님.
// openscad -o img/sketch_b.png sketch_b.scad
$fn = 64;
PIV = [20.2, 24.35];  W = 18;  H = 12.5;  L = 300;  z_end = 109;
tk = 2.6;                                   // J자 벽
x0 = -7.9 + tk;                             // 묶음 왼쪽 면(고정 손가락 끝 면 = −7.9)
x1 = x0 + W + 1;                            // 묶음 오른쪽 면(여유 0.5씩)
theta = asin((x1 + tk - PIV[0]) / sqrt(12.3*12.3 + 72*72)) + atan2(12.3, 72);
module jaws() {
    color("dimgray") import("ref/Wrist_Roll_Follower_SO101.stl", convexity = 6);
    color("gray") translate([PIV[0], 0, PIV[1]]) rotate([0, theta, 0]) rotate([-90, 0, 0]) import("ref/Moving_Jaw_SO101.stl", convexity = 6);
}
module bundle() { color("white") translate([(x0 + x1)/2, 0, z_end - H/2]) rotate([90, 0, 0]) scale([W/2, H/2, 1]) cylinder(r = 1, h = 290, center = true); }
module jf(s) {                              // J자 단면: 위로 곧은 벽 + 아래를 받치는 호
    ri = [(x1 - x0)/2, H/2 + 0.5];  cx = (x0 + x1)/2;  cz = z_end - H/2;
    function p(d, a) = [cx + s*(ri[0] + d)*cos(a), cz + (ri[1] + d)*sin(a)];
    pts = concat([[cx + s*(ri[0] + tk), cz - 34], [cx + s*ri[0], cz - 34]], [for (a = [0 : 5 : 50]) p(0, a)], [for (a = [50 : -5 : 0]) p(tk, a)]);
    rotate([90, 0, 0]) linear_extrude(L, center = true) polygon(pts);
}
jaws(); bundle();
{
    color("orange") { jf(-1); translate([-24, -17, 58]) cube([24 + x0 - tk, 34, 30]); }
    color("tomato") { jf(1); translate([PIV[0], 0, PIV[1]]) rotate([0, theta, 0]) translate([-13, -17, 36]) cube([14, 34, 30]); }
}
