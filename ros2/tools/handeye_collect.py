#!/usr/bin/env python3
"""손-눈 보정(`on_arm`)을 **처음부터 끝까지 스스로** — 이동·촬영·풀이·저장.

    ~/lerobot/.venv/bin/python ros2/tools/handeye_collect.py --dry    (계산만)
    ~/lerobot/.venv/bin/python ros2/tools/handeye_collect.py          (수집+풀이)
    ~/lerobot/.venv/bin/python ros2/tools/handeye_collect.py --save   (좋으면 저장)

⚠ `tomato-voice.service`를 내린 채로 돌린다 — 팔 포트는 한 프로세스만 연다.
   (그 서비스는 연결할 때 토크를 끄므로, 중간에 켜면 팔이 늘어진다.)

────────────────────────────────────────────────────────────────────────
왜 대시보드를 안 쓰고 여기서 다 하는가 — 세 가지가 한꺼번에 걸렸다.

① **좌표 이동이 막힌다.** `cartesian`은 지금 자세의 `signed_radius`가 90mm보다
   작으면 어떤 이동도 거절한다. 카메라가 표적을 보려면 팔이 접혀야 하고, 그
   자세의 `signed_radius`는 20mm 언저리다. 옳은 가드지만 여기서는 길을 막는다
   → **관절 공간으로 움직인다.**

② **뻗으면 서보가 놓는다.** 2026-08-31 실측: r≈260mm로 뻗다가 pitch 50° 부근에서
   **"갑자기 힘이 추욱"** 빠졌다(과부하 보호). 접힌 영역은 중력 모멘트가 작다
   → **접힌 채로 움직인다.**

③ **접힌 자세에서는 레거시의 도구 좌표계가 틀릴 수 있다.** `handeye.tool_frame`은
   방위각을 `atan2(y, x)`로 되찾는데 `signed_radius`가 음수면 180° 뒤집힌다
   → **관절값 `pan`으로 직접 만든다**(`tool_frame_from_joints`).

────────────────────────────────────────────────────────────────────────
자세를 어떻게 흩는가 — **회전만으로는 안 된다** (2026-08-31에 배운 것)

처음엔 `pan`과 `wrist_flex`만 흔들었다. 결과는 잔차 64mm에 `t_tool_cam`이
−575mm(팔 전체가 346mm인데!). 원인:

  **카메라가 pan 축 바로 위에 올라앉아 있다.** 접힌 자세의 TCP 수평거리가
  15mm뿐이라, pan을 ±28° 돌려도 카메라는 거의 제자리에서 **돌기만** 한다.
  실측이 그걸 말했다 — pan을 48° 돌리는 동안 카메라↔표적 거리가 5mm밖에
  안 변했다. 그러면 `t_x`가 관측되지 않고, 선형해의 `R_x`가 회전에서 멀어져
  SO(3) 투영에서 깨지며, **잔차까지 커진다.**

그래서 축을 넷으로 늘렸다. 앞의 둘은 회전, 뒤의 둘은 **평행이동**이다:

  · `pan`    — base z축 회전 (중력 모멘트가 안 변한다 = 공짜로 안전)
  · `pitch`  — wrist_flex 로 만드는 팔 평면 회전
  · `lift`   — 어깨를 굽혀 카메라를 **옮긴다** (pitch는 wrist_flex로 되돌린다)
  · `elbow`  — 팔꿈치를 굽혀 카메라를 **옮긴다** (마찬가지로 되돌린다)

`lift`/`elbow`를 움직이면 pitch가 따라 변하므로 `wrist_flex`로 그만큼 빼서
**자세는 그대로 두고 위치만** 바꾼다. 그게 `t_x`를 보이게 하는 유일한 길이다.

⚠ `wrist_roll`은 **고정**한다. 레거시가 도구 좌표계에서 roll을 빼고 계산하므로,
   안 건드리면 두 계통이 같은 답을 낸다. 돌리는 순간 갈린다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))
sys.path.insert(0, HERE)

import target_check as tc  # noqa: E402
from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware.handeye import (  # noqa: E402
    CalibrationError, Rigid, solve_on_arm,
)

DEG_PER_TICK = 360.0 / 4096.0
CAL = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json")
CART = os.path.expanduser("~/arm_cartesian.json")

STEP_DEG = 10.0        # 이동 중 한 구간에서 어느 관절도 이 이상 안 움직인다
SETTLE_SEC = 1.2       # 멈춘 뒤 이만큼 기다렸다 찍는다 (흔들림이 곧 잔차다)
MAX_TRACK_ERR = 12.0   # 지령과 실제가 이만큼 벌어지면 중단 (서보가 놓은 것)
MAX_TCP_R = 200.0      # 이보다 뻗지 않는다 — 과부하 보호가 걸린 영역을 피한다
EDGE_MARGIN = 10.0     # 화면 가장자리 여유(px) — 점 네 개가 다 들어와야 한다
PROBE_DEG = 8.0        # 야코비안 시험각
W, H = 848.0, 480.0

# 후보 격자. 앞 둘은 회전(도), 뒤 둘은 평행이동을 만드는 관절 변화(도).
# ⚠ lift 범위가 좁으면 **카메라가 안 움직인다** — 2026-08-31에 그것 때문에
#   기선이 32mm뿐이었고 보정이 안 풀렸다. 표적을 가로로 눕혀 화면 여유가
#   생겼으므로 lift를 넓게 쓴다(카메라 이동 폭 ~110mm를 노린다).
PAN_GRID = (-28.0, -14.0, 0.0, 14.0, 28.0)
PITCH_GRID = (-14.0, -4.0, 6.0, 16.0)
LIFT_GRID = (-20.0, -10.0, 0.0, 12.0, 24.0)
ELBOW_GRID = (-16.0, 0.0, 16.0)
WANT_POSES = 16


def tool_frame_from_joints(degs: dict, geom: kin.ArmGeometry) -> Rigid:
    """관절각 → `T_base_tool`. **`handeye.tool_frame`을 대신한다.**

    같은 규약(열이 approach·lateral·up)을 쓰되 방위각을 `atan2(y, x)`가 아니라
    관절값 `shoulder_pan` 그대로 쓴다. 팔이 몸통 위로 접혀 `signed_radius`가
    음수가 되면 FK의 (x, y)는 방위각이 180° 뒤집힌 같은 점으로 보이고, `atan2`는
    그 뒤집힌 값을 돌려준다 — 위치는 맞지만 **회전이 거울상이 된다.**
    """
    pan = math.radians(degs["shoulder_pan"])
    a3 = math.radians(degs["shoulder_lift"] + degs["elbow_flex"] + degs["wrist_flex"])
    approach = (math.cos(a3) * math.cos(pan), math.cos(a3) * math.sin(pan), math.sin(a3))
    lateral = (-math.sin(pan), math.cos(pan), 0.0)
    up = (-math.sin(a3) * math.cos(pan), -math.sin(a3) * math.sin(pan), math.cos(a3))
    pose = kin.forward(degs, geom)
    return Rigid(np.array([approach, lateral, up], dtype=float).T,
                 np.array([pose.x, pose.y, pose.z], dtype=float))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--save", action="store_true")
    ap.add_argument("--dot", default="tl", choices=("tl", "tr", "bl", "br"))
    ap.add_argument("--home", default="2.8,-9.1,100.8,83.2,-3.6",
                    help="시작 자세(도): pan,lift,elbow,wflex,wroll. "
                         "'here'면 지금 자세를 그대로 쓴다")
    args = ap.parse_args()

    cal = json.load(open(CAL))
    spans = {}
    for name, c in cal.items():
        try:
            spans[name] = abs(int(c["range_max"]) - int(c["range_min"])) * DEG_PER_TICK
        except (KeyError, TypeError, ValueError):
            pass
    cart = json.load(open(CART))
    zero, ref, signs = cart["zero"], cart["ref_deg"], cart.get("signs", {})
    geom = kin.ArmGeometry()

    def sign(j):
        v = signs.get(j)
        return -1.0 if (v is not None and float(v) < 0) else 1.0

    def dpn(j):
        s = spans.get(j)
        return abs(s) / 200.0 if s else (1.8 if j == "wrist_roll" else 0.9)

    def to_deg(n):
        return {j: ref.get(j, 0.0) + sign(j) * (float(n.get(j, 0.0)) - zero.get(j, 0.0)) * dpn(j)
                for j in kin.JOINTS}

    def to_norm(d):
        return {j: zero.get(j, 0.0) + (float(d[j]) - ref.get(j, 0.0)) / (sign(j) * dpn(j))
                for j in d if j in kin.JOINTS}

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO()

    def move(degs, secs=0.9):
        cur = to_deg(io.read())
        biggest = max(abs(degs[j] - cur[j]) for j in kin.JOINTS)
        steps = max(1, int(math.ceil(biggest / STEP_DEG)))
        for st in range(1, steps + 1):
            mid = {j: cur[j] + (degs[j] - cur[j]) * st / steps for j in kin.JOINTS}
            io.write(to_norm(mid), secs)
        time.sleep(SETTLE_SEC)
        got = to_deg(io.read())
        return got, max(abs(got[j] - degs[j]) for j in kin.JOINTS)

    def look():
        m = tc.measure()
        if not m["ok"]:
            return None
        us = [m["dots"][k][0] for k in ("tl", "tr", "bl", "br")]
        vs = [m["dots"][k][1] for k in ("tl", "tr", "bl", "br")]
        m["box"] = (min(us), min(vs), max(us), max(vs))
        return m

    # ⚠ 붙는 순간 팔이 늘어진다 — `FollowerIO._connect`가 `disable_torque`를 하기
    #    때문이다(안전한 기본값이다: 켜자마자 힘이 들어가면 사람이 놀란다).
    #    그래서 **연결 직후 곧바로 지금 자리를 목표로 줘서 붙든다.** 이걸 안 하면
    #    표적이 보이도록 맞춰 둔 자세를 매번 잃고, 중력에 끌려 내려간 자리에서
    #    시작하게 된다(실측: 붙을 때마다 lift가 -9° → -96°로 처졌다).
    io.write(io.read(), 0.4)

    if args.home.strip().lower() == "here":
        home = to_deg(io.read())
    else:
        vals = [float(v) for v in args.home.split(",")]
        want = dict(zip(kin.JOINTS, vals))
        print("시작 자세로 이동 — " + " ".join(f"{j.split('_')[0]}={want[j]:.1f}"
                                            for j in kin.JOINTS))
        home, herr = move(want)
        print(f"  도착 (관절 오차 {herr:.1f}°)")
    hp = kin.forward(home, geom)
    print("기준 자세  " + " ".join(f"{j.split('_')[0]}={home[j]:6.1f}" for j in kin.JOINTS))
    print(f"           TCP ({hp.x:6.1f},{hp.y:6.1f},{hp.z:6.1f}) pitch {hp.pitch:6.1f}° "
          f"signed_r {kin.signed_radius(home, geom):6.1f}mm")

    base_view = look()
    if base_view is None:
        print("❌ 지금 자세에서 4점이 안 보인다 — 카메라를 표적 쪽으로 맞춘 뒤 다시.")
        io.close()
        return 1
    print(f"           표적 상자 {tuple(round(v) for v in base_view['box'])} · "
          f"거리 {base_view['plane_mm']:.0f}mm")

    def pose_for(dpan, dpitch, dlift, delbow):
        """네 축 → 관절각. lift/elbow가 만든 pitch 변화는 wrist_flex로 되돌린다."""
        d = dict(home)
        d["shoulder_pan"] += dpan
        d["shoulder_lift"] += dlift
        d["elbow_flex"] += delbow
        d["wrist_flex"] += dpitch - dlift - delbow
        return d

    def compensable(dpan, dpitch, dlift, delbow):
        """`wrist_flex`가 **실제로 상쇄할 수 있는** 조합인가.

        ⚠ 이걸 안 보면 조용히 틀린다. lift를 -12° 움직이며 wrist_flex를 +12°로
          상쇄하려 했는데 wrist_flex가 이미 93°였고 한계가 105°라 다 못 갔다.
          그러면 "자세는 그대로, 위치만 이동"이라는 전제가 깨지는데 **아무도
          말해주지 않는다** — 실측에서 a3가 9.6° 어긋난 채로 표본이 들어갔다.
        """
        d = pose_for(dpan, dpitch, dlift, delbow)
        lim = spans.get("wrist_flex", 210.0) / 2.0
        return abs(d["wrist_flex"] - ref.get("wrist_flex", 0.0)) <= lim - 2.0

    # ── 야코비안: 각 축이 화면을 몇 px 옮기는가 (실측) ──
    print(f"\n[야코비안] 축마다 {PROBE_DEG:.0f}°가 화면을 몇 px 옮기는지 실측")
    axes = ("pan", "pitch", "lift", "elbow")
    jac = {}
    for k, name in enumerate(axes):
        done = False
        for sgn in (+1.0, -1.0):
            amt = [0.0, 0.0, 0.0, 0.0]
            amt[k] = sgn * PROBE_DEG
            got, e = move(pose_for(*amt))
            v = look()
            if v is not None and e <= MAX_TRACK_ERR:
                du = (v["box"][0] - base_view["box"][0]) / (sgn * PROBE_DEG)
                dv = (v["box"][1] - base_view["box"][1]) / (sgn * PROBE_DEG)
                jac[name] = (du, dv)
                print(f"  {name:<7} {du:+7.2f}, {dv:+7.2f} px/도")
                done = True
                break
            print(f"  {name:<7} {sgn*PROBE_DEG:+.0f}°에서 표적을 잃음 — 반대로")
        if not done:
            print(f"❌ {name} 축의 화면 이동량을 못 쟀다.")
            io.close()
            return 1
        move(dict(home))

    bw = base_view["box"][2] - base_view["box"][0]
    bh = base_view["box"][3] - base_view["box"][1]
    u0, v0 = base_view["box"][0], base_view["box"][1]

    def visible(a):
        du = sum(jac[n][0] * a[i] for i, n in enumerate(axes))
        dv = sum(jac[n][1] * a[i] for i, n in enumerate(axes))
        return (EDGE_MARGIN <= u0 + du and u0 + du + bw <= W - EDGE_MARGIN
                and EDGE_MARGIN <= v0 + dv and v0 + dv + bh <= H - EDGE_MARGIN)

    cands = []
    for dp in PAN_GRID:
        for dt in PITCH_GRID:
            for dl in LIFT_GRID:
                for de in ELBOW_GRID:
                    a = (dp, dt, dl, de)
                    if not visible(a) or not compensable(*a):
                        continue
                    degs = pose_for(*a)
                    pose = kin.forward(degs, geom)
                    bad = any(abs(degs[j] - ref.get(j, 0.0)) > spans.get(j, 200.0) / 2.0
                              for j in kin.JOINTS)
                    if bad or pose.z < 15.0 or math.hypot(pose.x, pose.y) > MAX_TCP_R:
                        continue
                    cands.append((a, degs, pose))
    total = len(PAN_GRID) * len(PITCH_GRID) * len(LIFT_GRID) * len(ELBOW_GRID)
    print(f"\n[후보] 격자 {total}개 중 쓸 수 있는 것 {len(cands)}개")

    # 회전과 평행이동을 **둘 다** 넓게 — 이미 고른 것들에서 가장 먼 후보를 집는다.
    picked = [c for c in cands if c[0] == (0.0, 0.0, 0.0, 0.0)] or [cands[0]]
    while len(picked) < WANT_POSES and len(picked) < len(cands):
        best, bestd = None, -1.0
        for c in cands:
            if any(c[0] == p[0] for p in picked):
                continue
            d = min(sum((c[0][i] - p[0][i]) ** 2 for i in range(4)) for p in picked)
            if d > bestd:
                best, bestd = c, d
        if best is None:
            break
        picked.append(best)

    xs = np.array([[p[2].x, p[2].y, p[2].z] for p in picked])
    print(f"계획 {len(picked)}자세 · TCP 이동 폭 "
          f"x {np.ptp(xs[:,0]):.0f} y {np.ptp(xs[:,1]):.0f} z {np.ptp(xs[:,2]):.0f} mm")
    for a, degs, pose in picked:
        print(f"  pan{a[0]:+5.0f} pit{a[1]:+5.0f} lif{a[2]:+5.0f} elb{a[3]:+5.0f}  "
              f"TCP ({pose.x:6.1f},{pose.y:6.1f},{pose.z:6.1f}) pitch {pose.pitch:6.1f}°")

    if args.dry:
        print("\n(--dry 이므로 움직이지 않는다)")
        io.close()
        return 0

    cam_pts, frames, labels, all_dots, cam_org = [], [], [], [], []
    for a, degs, _ in picked:
        label = f"pan{a[0]:+.0f} pit{a[1]:+.0f} lif{a[2]:+.0f} elb{a[3]:+.0f}"
        got, err = move(degs)
        if err > MAX_TRACK_ERR:
            print(f"  {label:<34} ⚠ 지령과 {err:.1f}° 차이 — 서보가 놓았다. 중단.")
            break
        m = look()
        if m is None:
            print(f"  {label:<34} 표적 안 보임 — 건너뜀")
            continue
        pt = m["points_mm"][args.dot]
        cam_pts.append(pt)
        frames.append(tool_frame_from_joints(got, geom))
        labels.append(label)
        all_dots.append(m["points_mm"])
        cam_org.append(kin.forward(got, geom))
        print(f"  {label:<34} ✓ {len(cam_pts):2}  카메라점 "
              f"({pt[0]:7.1f},{pt[1]:7.1f},{pt[2]:7.1f}) · 거리 {m['plane_mm']:.0f}mm")

    print(f"\n표본 {len(cam_pts)}개")
    if len(cam_pts) < 8:
        print("❌ 8개 미만 — 풀 수 없다.")
        io.close()
        return 1

    # 조건수 진단 — 지난번 실패의 원인이 여기였다.
    d = np.array([math.dist((0, 0, 0), p) for p in cam_pts])
    tt = np.array([[f.t[0], f.t[1], f.t[2]] for f in frames])
    print(f"  카메라↔표적 거리 {d.min():.0f}~{d.max():.0f}mm (폭 {np.ptp(d):.0f})")
    print(f"  도구 원점 이동 폭 x {np.ptp(tt[:,0]):.0f} y {np.ptp(tt[:,1]):.0f} "
          f"z {np.ptp(tt[:,2]):.0f} mm")

    try:
        fit = solve_on_arm(cam_pts, frames)
    except CalibrationError as exc:
        print(f"❌ {exc}")
        io.close()
        return 1

    print(f"\n{fit.summary()}")
    print(f"  T_tool_cam  t = {np.round(fit.transform.t, 1).tolist()} mm")
    print(f"              rpy = {tuple(round(v,1) for v in fit.transform.rpy_deg)}°")
    if fit.marker_base:
        print(f"  표적 위치(팔 base 기준) = {tuple(round(v,1) for v in fit.marker_base)} mm")
    w = fit.worst_index()
    if 0 <= w < len(labels):
        print(f"  가장 어긋난 표본: {w}번 ({labels[w]}) {fit.per_sample_mm[w]:.1f}mm")

    print("\n[교차검증] 네 점을 각각 마커로 삼아 따로 푼다")
    sols = {}
    for key in ("tl", "tr", "bl", "br"):
        try:
            f2 = solve_on_arm([dd[key] for dd in all_dots], frames)
            sols[key] = f2
            print(f"  {key}: 잔차 {f2.rms_mm:5.1f}mm · t = {np.round(f2.transform.t,1).tolist()}")
        except CalibrationError as exc:
            print(f"  {key}: 못 풀었다 — {str(exc)[:60]}")
    if len(sols) >= 2:
        ts = np.array([s.transform.t for s in sols.values()])
        spread = float(np.linalg.norm(ts.max(axis=0) - ts.min(axis=0)))
        print(f"  네 해의 카메라 위치 흩어짐 {spread:.1f}mm "
              + ("— 잘 일치한다" if spread < 10 else "⚠ 크다"))

    if args.save and fit.good:
        from tomato_picker.hardware.eye import EyeConfig
        cfg = EyeConfig()
        cfg.store("on_arm", fit, None, note="handeye_collect.py (관절공간 수집)")
        print(f"\n저장: {cfg.path}")
    elif args.save:
        print(f"\n잔차가 커서 저장하지 않았다 ({fit.rms_mm:.1f}mm).")

    print("\n⚠ 토크를 켠 채로 둔다 (여기서 끄면 팔이 떨어진다).")
    io._follower.disconnect()          # noqa: SLF001
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
