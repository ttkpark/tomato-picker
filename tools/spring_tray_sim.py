"""제본 코일 4개 묶음을 트레이 칸에 넣는 방식 비교 — MuJoCo 물리 시뮬레이션.

    python tools/spring_tray_sim.py              # 창으로 본다: 권장 방식 → 실패 예 둘 (1/4 속도)
    python tools/spring_tray_sim.py selftest     # 모형이 아는 사실을 재현하는가
    python tools/spring_tray_sim.py sweep        # 방식별 허용 오차 창 → docs/2026-10-21_스프링-분석/spring_tray_sim.json
    python tools/spring_tray_sim.py one drop --dy 2 --h 8 --png out.png
    python tools/spring_tray_sim.py compare      # 옮기는 방법 둘 비교(밖에서 4개 만들어 넣기 ↔ 세트 하나 넣고 트레이 위에서 포개기) → docs/2026-10-21_스프링-분석/spring_method_compare.json

단위는 mm·g로 적고 MuJoCo에는 m·kg로 넘긴다. 좌표: X = 코일 길이, Y = 칸 가로, Z = 위.

⚠ 치수 대부분이 사진에서 읽은 추정이다(P 딕셔너리의 주석). 그래서 결과는 "몇 %"가 아니라
   "어느 방식이 오차를 더 받아 주는가"와 "어느 치수에 민감한가"로 읽는다. 샘플을 재면 P만 고친다.

묶음 모형: 아래 세트·위 세트 = 강체 둘. 위 세트는 무게로만 얹혀 있고
   축 방향으로는 이(tooth)가 맞물려 eng(mm)만큼 들려야 한 피치 넘어간다(= "피치가 어긋난다").
   처짐(휨)은 없다 — 9.5mm는 탄성으로 버틴다는 공장 설명을 따른 단순화.
"""
import argparse, itertools, json, math, os, sys
import numpy as np
import mujoco

PITCH, NLOOP = 8.47, 34
L = PITCH * (NLOOP - 1)          # 279.5 고리 중심 간 길이

P = dict(                         # ── 바꿀 수 있는 가정 ──
    s=24.0,        # 칸 간격(리브 중심 간). 11/13 보정 사진에서 24.0±0.7(4개 리브 간격 24.0·24.3·23.3)
    rib_prof=((0.0, 10.0), (1.0, 7.1), (2.0, 5.4), (3.0, 4.3), (12.7, 2.3)),  # 리브 단면 (높이 mm, 폭 mm) — 버니어 11/14. 마지막 점 = 꼭대기(리브 높이 12.7)
    rib_end=32.3,  # 테두리 안쪽 ~ 리브 시작(버니어). 길이 방향: 32.3 | 리브 73.7 | 홈 25 | 리브 35.7 | 홈 25 | 리브 73.7 | 32.3 = 297.7
    notch_x=30.35, # 홈 중심(트레이 가운데 기준 ±). 85.7 = 첫 홈 시작 ~ 둘째 홈 끝(홈 둘 50 + 가운데 리브 35.7)
    notch_w=25.0,  # 홈 폭(버니어: 두 홈 합 50)
    fe=12.0,       # 끝 여백 핑거의 길이 방향 폭. 리브 끝에서 fe_gap만큼 띄운다
    fe_gap=5.0,    # 리브 끝 ~ 끝 여백 핑거 사이 여유 = 길이 방향 허용 오차
    fl=15.0,       # 홈 핑거의 길이 방향 폭 → 길이 여유 ±(25 − 15)/2
    rim_h=12.7,    # 트레이 가장자리 높이(버니어 11/14)
    ax=9.0,        # 묶음 끝 ~ 테두리 여유(한쪽): (298 − 279.5) / 2
    half_w=8.0,    # 묶음 반폭: 4개 한 세트 폭 16.0(버니어 11/14). 흰색은 18.0 → 9.0
    height=12.7,   # 묶음 높이(버니어 11/14)
    eng=1.5,       # 위 세트가 축 방향으로 넘어가려면 들려야 하는 높이(추정 — 덮는 방식이라 얕다)
    sag=9.0,       # 양끝을 잡았을 때 가운데 처짐(mm). 실측 10~12(11/14 영상, 손 간격 약 270) → 핑거 간격 255로 환산 8~10. 0 = 휘지 않음
    mass=14.0,     # 세트 하나(g): 실측 4개 = 28g(검정 11mm판, 1g 저울 11/17). 예전 24는 1mm 강선 가정
    mu=0.2,        # 마찰(트레이가 미끄럽다는 공장 설명)
    zeta=0.3,      # 접촉 감쇠비: 0.3 ≈ 반발계수 0.37. 작을수록 "팅"
    tf=2.0,        # 핑거(판) 두께 — 묶음 사이 틈이 8mm(24 − 16)뿐이라 얇아야 한다
    fz=2.0,        # 핑거 아랫단 높이(묶음 바닥 기준)
    open=1.5,      # 놓을 때 한쪽으로 벌리는 양 — 많이 벌리면 옆 칸 묶음에 닿는다
    comp=0.0,      # 핑거가 묶음을 옆으로 봐주는 무르기(N/m). 0 = 딱딱하게 문다. 스펀지 패드처럼 무르면 돌기가 묶음 끝을 칸 안으로 밀어 넣을 수 있다
    only="",       # "" = 4개 묶음을 통째로 / "bot" = 아래 세트만 넣는다(위 세트는 없는 셈) / "top" = 아래 세트가 이미 놓인 위에 위 세트를 포갠다
    flat=0,        # 1 = 리브·테두리·옆 묶음이 없는 평평한 테이블(트레이 밖에서 포개기)
    fingers="ends",  # ends = 리브가 없는 양 끝 여백(테두리 벽 앞 32.3)을 잡는다(11/14 채택) / gaps = 리브 홈 두 곳 / long = 길이 72% 긴 판
)
LANE = 2                          # 가운데 칸에 넣는다(양옆 칸은 이미 차 있다 = 최악)
DT = 0.0002
TQ = 20                           # weld의 회전 구속 가중 — 기본 1이면 가벼운 묶음이 잡은 점에서 헛돈다(처짐이 3배로 나온다)


def lane_y(i, p): return (i - 2) * p["s"]


def build(p):
    m = 1e-3
    inner = L + 2 * p["ax"]
    g = []                                         # (body, xml)
    A = lambda body, s: g.append((body, s))
    cap = lambda y, z, r, x0, x1: f'<geom type="capsule" size="{r*m}" fromto="{x0*m} {y*m} {z*m} {x1*m} {y*m} {z*m}"/>'
    # 트레이: 바닥, 테두리, 리브 4개(둥근 등)
    A("w", f'<geom name="floor" type="box" size="{(inner/2+20)*m} {(2.5*p["s"]+20)*m} 0.005" pos="0 0 -0.005" class="tray"/>')
    wy = 2.5 * p["s"]
    for sx in (() if p["flat"] else (-1, 1)):
        A("w", f'<geom type="box" class="tray" size="0.002 {wy*m} {p["rim_h"]/2*m}" pos="{sx*(inner/2+2)*m} 0 {p["rim_h"]/2*m}"/>')
        A("w", f'<geom type="box" class="tray" size="{inner/2*m} 0.002 {p["rim_h"]/2*m}" pos="0 {sx*(wy+2)*m} {p["rim_h"]/2*m}"/>')
    rl, pr = inner / 2 - p["rib_end"], [(z, w / 2) for z, w in p["rib_prof"]]
    rh = pr[-1][0]
    rr, nx, nw = pr[-1][1], p["notch_x"], p["notch_w"] / 2
    for k in range(0 if p["flat"] else 4):
        y = (k - 1.5) * p["s"]
        for xa, xb in ((-rl, -nx - nw), (-nx + nw, nx - nw), (nx + nw, rl)):   # 홈 둘로 세 토막
            A("w", f'<geom type="capsule" class="tray" size="{rr*m}" fromto="{(xa+rr)*m} {y*m} {(rh-rr)*m} {(xb-rr)*m} {y*m} {(rh-rr)*m}"/>')
            for (z0, w0), (z1, w1) in zip(pr, pr[1:]):  # 퍼지는 옆면 = 기울인 얇은 판
                for sg in (-1, 1):
                    an = math.atan2(z1 - z0, sg * (w1 - w0))
                    A("w", f'<geom type="box" class="tray" size="{(xb-xa)/2*m} {math.hypot(z1-z0, w1-w0)/2*m} 0.0003" pos="{(xa+xb)/2*m} {(y+sg*(w0+w1)/2)*m} {(z0+z1)/2*m}" quat="{math.cos(an/2)} {math.sin(an/2)} 0 0"/>')
    for i in (() if p["flat"] else (LANE - 1, LANE + 1)):   # 옆 칸에 이미 든 묶음
        A("w", f'<geom name="nb{i}" type="box" class="tray" size="{L/2*m} {p["half_w"]*m} {p["height"]/2*m}" pos="0 {lane_y(i,p)*m} {p["height"]/2*m}"/>')
    # 묶음: 아래 세트 = 바닥 레일 2 + 옆 레일 2 + 이 / 위 세트 = 얹히는 레일 2 + 안쪽 용골 2 + 지붕 2 + 이
    # ponytail: 처짐 = 세트마다 가운데 경첩 하나(V자). 곡선이 필요해지면 토막 수를 늘린다
    hw, H, x0, x1 = p["half_w"], p["height"], -L / 2, L / 2
    def rail(b, y, z, r, xa, xb):                   # 가운데(x=0)에서 두 토막으로
        A(b + "L", cap(y, z, r, xa, 0)); A(b + "R", cap(y, z, r, 0, xb))
    sr = 0.8; zs = H / 2 - sr                     # 가장 넓은 곳 = 높이 가운데(둥근 단면)
    for y in (-3, 3): rail("bot", y, 1.2, 1.2, x0, x1)
    for y in (-(hw - sr), hw - sr): rail("bot", y, zs, sr, x0, x1)
    o = PITCH / 2                                   # 위 세트는 반 피치 어긋나 있다
    for y in (-(hw - sr), hw - sr): rail("top", y, zs + 2 * sr, sr, x0 + o, x1 + o)
    for y in (-(hw - 3 * sr - 0.2), hw - 3 * sr - 0.2): rail("top", y, zs, sr, x0 + o, x1 + o)
    for y in (-3, 3): rail("top", y, H - 1.2, 1.2, x0 + o, x1 + o)
    tr, tz = 1.6, 3.0
    for i in range(NLOOP):
        for b, x, z in (("bot", x0 + i * PITCH, tz), ("top", x0 + i * PITCH + o, tz + 2 * tr - p["eng"])):
            A(b + ("L" if x < 0 else "R"), f'<geom type="sphere" size="{tr*m}" pos="{x*m} 0 {z*m}"/>')
    # 핑거(판): 트레이·옆 묶음과의 간섭만 본다. 잡는 힘은 weld가 대신한다
    o4 = PITCH / 4                                   # run()이 묶음을 -PITCH/4 옮기므로 핑거는 그만큼 되돌려 트레이 기준 자리에 둔다
    xe = inner / 2 - p["rib_end"] + p["fe_gap"] + p["fe"] / 2     # 끝 여백 핑거 중심(트레이 가운데 기준)
    segs = {"long": [(0, L * 0.72)], "ends": [(-xe + o4, p["fe"]), (xe + o4, p["fe"])],
            "gaps": [(-p["notch_x"] + o4, p["fl"]), (p["notch_x"] + o4, p["fl"])]}[p["fingers"]]
    for side, body in ((-1, "pl"), (1, "pr")):
        for cx, ln in segs:
            A(body, f'<geom type="box" class="plate" size="{ln/2*m} {p["tf"]/2*m} 0.006" pos="{cx*m} {side*(hw+0.3+p["tf"]/2)*m} {(p["fz"]+6)*m}"/>')
    G = lambda b: "\n".join(s for bb, s in g if bb == b)
    mass = p["mass"] * 1e-3
    half = lambda sx: f'<inertial pos="{sx*L/4*m} 0 {H/2*m}" mass="{mass/2}" diaginertia="{mass/2*0.005**2} {mass/2*(L/2*m)**2/12} {mass/2*(L/2*m)**2/12}"/>'
    # 잡는 점 둘(끝 여백 핑거 자리)은 경첩으로 물린다 → 가운데가 자중으로 처진다. 경첩 강성은 처짐 p["sag"]가 나오게 정한다
    a = xe * m
    kh = mass * 9.81 * a * a / (8 * max(p["sag"], 0.3) * m)        # 반쪽 무게 모멘트 = k · 2θ, θ = sag / a
    ch = math.sqrt(kh * mass / 2 * (L / 2 * m) ** 2 / 3)
    tiny = '<inertial pos="0 0 0" mass="0.0005" diaginertia="2e-8 2e-8 2e-8"/>'
    soft = f'<joint type="slide" axis="0 1 0" stiffness="{p["comp"]}" damping="{2*math.sqrt(p["comp"]*mass)}"/>' if p["comp"] > 0 else ""
    sets = ""
    ghost = 'contype="0" conaffinity="0" ' if p["only"] == "bot" else ""      # 아래 세트만 넣을 때 위 세트는 안 닿는 유령
    for b, col in (("bot", ""), ("top", f'<geom {ghost}rgba=".45 .78 .92 {0.08 if ghost else 1}" ')):
        gl, gr_ = (G(b + "L"), G(b + "R")) if not col else (G(b + "L").replace("<geom ", col), G(b + "R").replace("<geom ", col))
        sets += f'<body name="{b}"><freejoint name="j{b}"/>{half(-1)}{gl}<body name="{b}R"><joint type="hinge" axis="0 1 0" pos="0 0 {H/2*m}" stiffness="{kh}" damping="{ch}"/>{half(1)}{gr_}</body></body>'
    xml = f"""<mujoco><option timestep="{DT}" integrator="implicitfast" cone="elliptic"/>
<size nconmax="2000" njmax="6000"/>
<default><geom friction="{p['mu']} 0.002 0.0001" solref="0.003 {p['zeta']}" rgba=".95 .96 .97 1"/>
 <default class="tray"><geom conaffinity="3" rgba=".6 .85 .95 .35"/></default>
 <default class="plate"><geom contype="2" conaffinity="0" rgba=".9 .5 .2 .8"/></default></default>
<visual><global offwidth="1280" offheight="720"/></visual>
<worldbody><light pos="0 -0.3 0.5" dir="0 0.5 -1"/>{G('w')}
<body name="hand" mocap="true">
 <body name="gl" pos="{(-xe+o4)*m} 0 {H/2*m}">{soft}<joint type="hinge" axis="0 1 0" damping="1e-5"/>{tiny}</body>
 <body name="gr" pos="{(xe+o4)*m} 0 {H/2*m}"><joint type="slide" axis="1 0 0" damping="0.02"/>{soft}<joint type="hinge" axis="0 1 0" damping="1e-5"/>{tiny}</body>
</body>
<body name="pl" mocap="true">{G('pl')}</body><body name="pr" mocap="true">{G('pr')}</body>
{sets}</worldbody>
<equality>{"".join(f'<weld body1="{g}" body2="{b}" anchor="{sx*xe*m+o4*m} 0 {H/2*m}" torquescale="{TQ}" solref="0.002 1"/>' for g, sx, bs in (("gl", -1, ("bot", "top")), ("gr", 1, ("botR", "topR"))) for b in bs)}</equality>
</mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


def quat(yaw, pitch):
    """Z로 yaw 돌린 뒤 몸체 Y로 pitch(+면 +X 끝이 내려간다)."""
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return np.array([cy * cp, -sy * sp, cy * sp, sy * cp])


def rot(q):
    R = np.zeros(9); mujoco.mju_quat2Mat(R, q); return R.reshape(3, 3)


def _ang(c):
    """cos → 도. 수치 오차로 ±1을 살짝 넘거나 NaN(발산)이어도 죽지 않는다 — NaN은 180°로 쳐서 실패로 센다."""
    return 180.0 if not np.isfinite(c) else math.degrees(math.acos(max(-1.0, min(1.0, float(c)))))


def run(p, mode="drop", dy=0.0, yaw=0.0, dx=0.0, h=0.0, v=80.0, tilt=4.0, vx=500.0, by=0.0, bx=0.0, v2=None, open_t=0.03, frames=None, model=None, show=False):
    """한 번 넣어 본다. 길이 mm, 각도 deg, 속도 mm/s. h = 놓는 순간 묶음 바닥의 높이(바닥 기준).
    v2 = 눕히는 속도: 가운데가 닿기 직전(처짐 + 3mm)부터는 v 대신 v2로 내린다(None이면 끝까지 v). open_t = 핑거를 벌리는 데 걸리는 시간(s).
    p["only"] == "top"이면 아래 세트는 (bx, by)만큼 어긋난 자리에 이미 놓여 있고, 손은 위 세트만 들고 온다(h = 다 포개진 높이 위로 남은 거리)."""
    only = p["only"]; BOT, TOP = [0, 2], [1, 3]     # equality 순서: (gl,bot) (gl,top) (gr,botR) (gr,topR)
    m = model or build(p); d = mujoco.MjData(m)
    mm = 1e-3; yl = lane_y(LANE, p); ywr = math.radians(yaw); z0 = 25.0
    dx -= PITCH / 4                                 # 위 세트가 반 피치 나와 있으므로 묶음 전체를 칸 가운데에
    ib, it = m.body("bot").id, m.body("top").id
    ab, at = (int(m.jnt_qposadr[m.joint(n).id]) for n in ("jbot", "jtop")); vb, vt = (int(m.jnt_dofadr[m.joint(n).id]) for n in ("jbot", "jtop"))
    # 핑거는 mocap(고정체 취급)이라 엔진이 트레이와의 접촉을 걸러 버린다 → 거리를 직접 잰다
    plates = [i for i in range(m.ngeom) if m.geom_contype[i] == 2]
    trayg = [i for i in range(m.ngeom) if m.geom_conaffinity[i] == 3 and m.geom(i).name != "floor"]

    if mode == "push":                              # 기준선: 바닥에 놓인 묶음을 축 방향으로 밀어 테두리에 부딪힌다
        t_rel, T = 0.0, 0.8
        pose = lambda t: (np.array([-PITCH / 4, yl, 0.0]), 0.0)
    else:
        th = math.radians(tilt) if mode == "tilt" else 0.0
        zt = L / 2 * math.sin(th)                   # 기울인 채 낮은 끝이 닿을 때의 중심 높이
        za = max(h, p["sag"] + 3.0) if v2 and not th else h      # 여기까지는 빨리, 그 아래는 v2로 눕힌다
        ta = (z0 - za) / v; t1 = ta + ((za - h) / v2 if v2 else 0.0)
        t2 = t1 + (0.3 if th else 0.0); t_rel = t2 + 0.05; T = t_rel + open_t + 0.9
        def pose(t):
            if t < ta: return np.array([dx, yl + dy, z0 + zt - v * t]), th
            if t < t1: return np.array([dx, yl + dy, za - v2 * (t - ta)]), th
            if t < t2: a = th * (1 - (t - t1) / 0.3); return np.array([dx, yl + dy, h + L / 2 * math.sin(a)]), a
            return np.array([dx, yl + dy, h + max(0.0, t - t_rel - open_t - 0.1) * 100]), 0.0

    def place_hand(t):
        pos, a = pose(t); q = quat(ywr, a); R = rot(q)
        op = min(1.0, max(0.0, (t - t_rel) / open_t)) * p["open"]
        d.mocap_pos[0] = pos * mm; d.mocap_quat[0] = q
        for k, sgn in ((1, -1), (2, 1)): d.mocap_pos[k] = (pos + R @ np.array([0, sgn * op, 0])) * mm; d.mocap_quat[k] = q
        return pos, q

    pos, q = place_hand(0.0)
    d.qpos[ab:ab + 3] = d.qpos[at:at + 3] = pos * mm; d.qpos[ab + 3:ab + 7] = d.qpos[at + 3:at + 7] = q
    if only == "top": d.qpos[ab:ab + 3] = np.array([-PITCH / 4 + bx, yl + by, 0.0]) * mm; d.qpos[ab + 3:ab + 7] = [1, 0, 0, 0]; d.eq_active[BOT] = 0
    if mode == "push": d.eq_active[:] = 0; d.qvel[vb] = d.qvel[vt] = vx * mm
    mujoco.mj_forward(m, d)

    finger_hit = 0.0; hop = 0.0; sag = 0.0; released = mode == "push"; slip = 0.0; vfall = 0.0   # slip = 놓은 뒤 위 세트가 아래 세트 대비 가장 많이 밀린 양, vfall = 가장 빠른 낙하 속도
    if show:                                        # 창을 띄워 1/4 속도로 보여 준다
        import time; from mujoco import viewer as mjv
        vw = mjv.launch_passive(m, d); vw.cam.lookat[:] = [0.06, yl * mm, 0.01]; vw.cam.distance = 0.3; vw.cam.azimuth = 150; vw.cam.elevation = -25
    for step in range(int(T / DT)):
        t = step * DT
        place_hand(t)
        if not released and t >= t_rel: sag = float(d.mocap_pos[0][2] - d.xpos[ib][2]) / mm; d.eq_active[BOT if only == "bot" else slice(None)] = 0; released = True
        mujoco.mj_step(m, d)
        if show and step % 25 == 0:
            if not vw.is_running(): break
            vw.sync(); time.sleep(0.02)
        if step % 10 == 0:
            if step % 50 == 0 and mode != "push" and t < t_rel + open_t + 0.1:
                finger_hit = max(finger_hit, -min((mujoco.mj_geomDistance(m, d, a, b, 0.001, None) for a in plates for b in trayg), default=0.0) / mm)
            if released:
                Rb = d.xmat[ib].reshape(3, 3); rl_ = Rb.T @ (d.xpos[it] - d.xpos[ib]) / mm; hop = max(hop, float(rl_[2]))
                slip = max(slip, float(np.hypot(rl_[0], rl_[1]))); vfall = max(vfall, float(-d.qvel[vb + 2]) / mm)
        if frames is not None and step % 100 == 0:
            frames.append([round(t, 3)] + [round(float(x), 5) for x in np.concatenate([d.qpos[ab:ab + 7], d.qpos[at:at + 7], d.mocap_pos[1], d.mocap_pos[2], d.mocap_quat[0]])])

    if show: vw.close()
    Rb = d.xmat[ib].reshape(3, 3); rel = Rb.T @ (d.xpos[it] - d.xpos[ib]) / mm
    pb = d.xpos[ib] / mm; tiltb = _ang(Rb[2, 2])
    reltilt = _ang((Rb.T @ d.xmat[it].reshape(3, 3))[2, 2])
    if not np.isfinite(d.qpos).all(): pb = np.array([0.0, 99.0, 99.0]); rel = np.array([99.0, 99.0, 99.0])   # 발산 = 실패
    seated = pb[2] < 0.5 and tiltb < 1.5 and (p["flat"] or abs(pb[1] - yl) < p["s"] / 2)
    if only == "bot": rel = np.zeros(3); reltilt = 0.0     # 위 세트는 유령 — 아래 세트가 앉았는지만 본다
    intact = abs(rel[0]) < PITCH / 4 and abs(rel[1]) < 1.0 and abs(rel[2]) < 0.6 and reltilt < 2.0
    out = "FINGER" if finger_hit > 0.3 else "OK" if seated and intact else "SKIP" if seated else "RIB"
    return dict(out=out, y=round(pb[1] - yl, 2), z=round(pb[2], 2), tilt=round(tiltb, 2), rel=[round(float(x), 2) for x in rel],
                hop=round(hop, 2), slip=round(slip, 2), vfall=round(vfall), finger=round(finger_hit, 2), sag=round(sag, 1), model=m, data=d)


LABEL = dict(OK="들어감", FINGER="핑거 간섭", RIB="리브·옆칸에 얹힘", SKIP="위 세트 어긋남")


def window(p, model, axis="dy", **kw):
    """오차(dy 가로 / dx 길이)를 0.5mm씩 늘려 가며 양쪽 다 성공하는 최대값. 첫 실패의 종류도 돌려준다."""
    ok, why = -0.5, "OK"
    for dy in np.arange(0, 9.01, 0.5):
        rs = [run(p, model=model, **{axis: s * dy}, **kw)["out"] for s in ((1,) if dy == 0 else (1, -1))]
        bad = [r for r in rs if r != "OK"]
        if bad: why = bad[0]; break
        ok = float(dy)
    return ok, why


def selftest():
    p = dict(P); m = build(p)
    r = run(dict(P, fingers="long"), "drop", h=0); assert r["out"] == "FINGER", ("긴 판은 칸 안에서 리브에 닿아야 한다", r)
    r = run(p, "drop", h=0, model=m); assert r["out"] == "OK", ("끝 여백 핑거: 오차 0·닿을 때 놓기는 들어가야 한다", r)
    r = run(p, "drop", h=3, dx=p["fe_gap"] + 1.5, model=m); assert r["out"] == "FINGER", ("끝 여백 핑거가 여백을 벗어나면 리브에 닿아야 한다", r)
    r = run(dict(P, fingers="gaps"), "drop", h=0, dx=p["notch_w"]); assert r["out"] == "FINGER", ("홈 핑거가 홈을 벗어나면 리브에 닿아야 한다", r)
    r = run(p, "push", vx=1000, model=m); assert r["out"] == "SKIP", ("1 m/s로 밀어 테두리에 부딪히면 어긋나야 한다(공장 경험)", r)
    r = run(p, "push", vx=50, model=m); assert r["out"] == "OK", ("살살 밀면 안 어긋나야 한다", r)
    r = run(p, "drop", h=0, dy=p["s"] / 2, model=m); assert r["out"] != "OK", ("리브 위에 놓으면 실패해야 한다", r)
    r = run(p, "drop", h=18, model=m); assert abs(r["sag"] - p["sag"]) < 2.0, ("양끝을 잡고 들면 가운데가 p[sag]만큼 처져야 한다", r["sag"])
    print("selftest OK")


def _row(job):
    tag, p, axis, kw = job; w, why = window(p, build(p), axis, **kw)
    return dict(tag=tag, axis=axis, p={k: p[k] for k in p if p[k] != P[k]}, kw=kw, win=w, why=why)


def sweep(path):
    from multiprocessing import Pool
    jobs = []; E = dict(P)
    J = lambda grp, tag, p, axis="dy", **kw: jobs.append((f"{grp}|{tag}", p, axis, kw))
    for fingers in ("ends", "gaps", "long"):
        p = dict(P, fingers=fingers)
        for h in (0, 1, 2, 3, 5, 8, 12):
            for yaw in (0, 1, 2): J(f"핑거 {fingers}", f"drop h={h} yaw={yaw}", p, mode="drop", h=h, yaw=yaw)
        for yaw in (0, 1, 2): J(f"핑거 {fingers}", f"tilt 4° yaw={yaw}", p, mode="tilt", h=0, yaw=yaw)
    for h in (0, 3, 8): J("길이 방향 오차(ends)", f"drop h={h}", E, "dx", mode="drop", h=h)
    J("길이 방향 오차(ends)", "tilt 4°", E, "dx", mode="tilt", h=0)
    for h, v in itertools.product((0, 3, 8), (20, 80, 200, 400)): J("내리는 속도(ends)", f"drop h={h} v={v}", E, mode="drop", h=h, v=v)
    for key, vals in (("s", (21, 27)), ("eng", (0.7, 3.0)), ("zeta", (0.15, 0.6)), ("mu", (0.1, 0.4)), ("tf", (1.0, 3.0)), ("open", (1.0, 3.0)), ("fe", (6.0, 16.0)), ("half_w", (7.5, 9.0))):
        for val in vals:
            for h in (0, 3, 8): J("모르는 치수 민감도(ends)", f"{key}={val} drop h={h}", dict(E, **{key: val}), mode="drop", h=h)
    with Pool() as pool: rows = pool.map(_row, jobs, chunksize=1)
    grp = None
    for r in rows:
        g, tag = r["tag"].split("|")
        if g != grp: grp = g; print(f"\n== {g} ==")
        print(f"{tag:<26} {'가로' if r['axis'] == 'dy' else '길이'} 허용 ±{r['win']:>4.1f}mm  막는 것: {LABEL[r['why']]}")
    print("\n== 축 방향으로 밀어 테두리에 부딪힘(공장이 겪은 '팅') ==")
    m = build(P)
    for vx in (50, 100, 200, 300, 500, 700, 1000):
        r = run(P, "push", vx=vx, model=m); rows.append(dict(tag=f"push|vx={vx}", out=r["out"], rel=r["rel"], hop=r["hop"]))
        print(f"push {vx:>5} mm/s → {LABEL[r['out']]}  위 세트 밀림 {r['rel'][0]:+.1f}mm  튐 {r['hop']:.1f}mm")
    json.dump(dict(P=P, pitch=PITCH, L=L, rows=rows), open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("→", path)


STAGES = (  # (방법, 단계 이름, P 덮어쓰기, run 인자)
    ("1", "1-가 테이블에서 포개기", dict(only="top", flat=1), {}),
    ("1", "1-나 4개 묶음을 칸에 넣기", {}, {}),
    ("2", "2-가 세트 하나를 칸에 넣기", dict(only="bot"), {}),
    ("2", "2-나 칸 안에서 포개기(아래 세트 가운데)", dict(only="top"), {}),
    ("2", "2-나 칸 안에서 포개기(아래 세트 1mm 치우침)", dict(only="top"), dict(by=1.0)),
    ("2", "2-나 칸 안에서 포개기(아래 세트 2mm 치우침)", dict(only="top"), dict(by=2.0)),
)
HS = (0, 3, 8, 12)


def compare(path):
    """두 방법을 단계로 쪼개 단계마다 허용 오차 창을 잰다. 방법의 창 = 가장 좁은 단계의 창(놓는 높이는 단계마다 제일 좋은 것)."""
    from multiprocessing import Pool
    jobs = []
    for meth, name, pp, kw in STAGES:
        for h in HS:
            for yaw in (0, 1, 2): jobs.append((f"{name}|dy h={h} yaw={yaw}", dict(P, **pp), "dy", dict(kw, mode="drop", h=h, yaw=yaw)))
            jobs.append((f"{name}|dx h={h} yaw=0", dict(P, **pp), "dx", dict(kw, mode="drop", h=h)))
    with Pool() as pool: rows = pool.map(_row, jobs, chunksize=1)
    best = {}                                       # (단계, 축, yaw) → (창, h, 막는 것)
    for r in rows:
        name, tag = r["tag"].split("|"); ax, hs, ys = tag.split(); k = (name, ax, int(ys[4:]))
        if k not in best or r["win"] > best[k][0]: best[k] = (r["win"], int(hs[2:]), r["why"])
    cols = [("dy", 0), ("dy", 1), ("dy", 2), ("dx", 0)]
    print(f"{'단계':<34}" + "".join(f"{('가로' if a == 'dy' else '길이') + f' {y}°':>16}" for a, y in cols))
    for meth, name, pp, kw in STAGES:
        print(f"{name:<34}" + "".join(f"  ±{best[(name, a, y)][0]:>4.1f} (h={best[(name, a, y)][1]:>2})" for a, y in cols))
    summ = {}
    for meth in ("1", "2"):
        names = [n for mm_, n, _, _ in STAGES if mm_ == meth]
        summ[meth] = {f"{a}{y}": min(best[(n, a, y)][0] for n in names) for a, y in cols}
        print(f"방법 {meth} = 가장 좁은 단계: " + "  ".join(f"{k} ±{v:.1f}" for k, v in summ[meth].items()))
    json.dump(dict(P=P, stages=[s[1] for s in STAGES], rows=rows, best={"|".join(map(str, k)): v for k, v in best.items()}, summary=summ),
              open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("→", path)


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"): sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(); ap.add_argument("cmd", nargs="?", default="view", choices=["view", "selftest", "sweep", "one", "compare"]); ap.add_argument("mode", nargs="?", default="drop")
    for k in ("dy", "yaw", "dx", "h", "v", "tilt", "vx"): ap.add_argument("--" + k, type=float)
    for k, v in P.items(): ap.add_argument("--" + k, type=type(v), dest="P_" + k)
    ap.add_argument("--png"); ap.add_argument("--traj"); ap.add_argument("--show", action="store_true")
    a = ap.parse_args()
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if a.cmd == "view":                             # 인자 없이 실행 = 권장 방식과 실패 예를 창으로 본다
        shows = [("권장: 양 끝 여백을 잡고, 처진 가운데가 먼저 앉은 뒤 끝이 12mm 위일 때 놓기 (가로 +1mm, 틀어짐 1°)", dict(P), dict(mode="drop", h=12, dy=1, yaw=1)),
                 ("실패 예: 8mm 위에서 떨어뜨리기 (가로 +5mm) — 리브에 걸린다", dict(P), dict(mode="drop", h=8, dy=5)),
                 ("실패 예: 1 m/s로 밀어 테두리에 부딪힘 — 위 세트가 어긋난다", dict(P), dict(mode="push", vx=1000))]
        for title, p, kw in shows:
            print(title, flush=True); r = run(p, show=True, **kw); print("  →", LABEL[r["out"]], flush=True)
        print("다른 명령: selftest · sweep · one drop --h 4 --dy 2 (끝에 --show를 붙이면 창으로)")
    elif a.cmd == "selftest": selftest()
    elif a.cmd == "sweep": sweep(os.path.join(here, "docs", "spring_tray_sim.json"))
    elif a.cmd == "compare": compare(os.path.join(here, "docs", "spring_method_compare.json"))
    else:
        p = dict(P, **{k[2:]: v for k, v in vars(a).items() if k.startswith("P_") and v is not None})
        kw = {k: getattr(a, k) for k in ("dy", "yaw", "dx", "h", "v", "tilt", "vx") if getattr(a, k) is not None}
        fr = [] if a.traj else None
        r = run(p, a.mode, frames=fr, show=a.show, **kw)
        print(LABEL[r["out"]], {k: r[k] for k in ("y", "z", "tilt", "rel", "hop", "finger", "sag")})
        if a.traj: json.dump(dict(p=p, mode=a.mode, kw=kw, out=r["out"], frames=fr), open(a.traj, "w"))
        if a.png:
            from PIL import Image
            R = mujoco.Renderer(r["model"], 720, 1280); cam = mujoco.MjvCamera()
            cam.lookat[:] = [0.11, 0, 0.005]; cam.distance = 0.13; cam.azimuth = 180; cam.elevation = -12
            R.update_scene(r["data"], cam); Image.fromarray(R.render()).save(a.png); print("→", a.png)
