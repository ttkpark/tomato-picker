#!/usr/bin/env python3
"""**카메라가 어느 쪽을 보는지 그 자리에서 잰다** — 손-눈 회전의 국소 실측.

    ~/lerobot/.venv/bin/python ros2/tools/cam_frame.py            # 재고 저장
    ~/lerobot/.venv/bin/python ros2/tools/cam_frame.py --to 412,170
        → 화면 그 점을 집게가 무는 자리로 데려가려면 base 좌표로 얼마인지

⚠ `tomato-voice`는 내리고 `depth-cam`은 켠 채로. **사람이 보는 앞에서만.**

────────────────────────────────────────────────────────────────────────
왜 이걸 따로 재나

전체 손-눈 보정(`handeye.py`)은 잔차 44mm에서 안 내려갔고, 원인을 못 찾았다.
그 사이 "위로 올려라 / 오른쪽으로 돌려라"를 **부호를 추측해서** 넣다가 몇 번씩
엉뚱한 데를 보게 만들었다(2026-09-02~03: 손목 화면이 천장·선반·벽을 차례로
봤다). 손목 화면이 상하 반전이라는 것도 뒤늦게 알았다.

그런데 **잡는 데 필요한 건 전역 보정이 아니다.** 지금 이 자세에서 카메라
좌표의 변위 하나를 base 좌표의 변위로 옮길 수만 있으면 된다. 그건 회전 하나고,
**팔을 알려진 방향으로 조금 움직여 정지한 점이 화면에서 어디로 가는지 보면**
바로 나온다. 링크 길이도 영점도 안 들어간다.

  정지점 X 에 대해  X_cam = R (X − t).  카메라가 Δ(base)만큼 **평행이동**하면
  ΔX_cam = −R Δ.  그러니 알려진 Δ 두 개로 R 의 두 열을 읽고, 나머지 한 열은
  회전이라는 사실(열이 정규직교)에서 외적으로 채운다.

⚠ **재는 동안 카메라가 돌면 안 된다.** 그래서 피치를 묶고 pan을 고정한 채
  움직인다 — 그 제약 때문에 잴 수 있는 방향이 둘뿐이고, 그래서 외적이 필요하다.
⚠ 정지점은 **템플릿으로 추적**한다. 깊이 덩이 무게중심은 화면 밖으로 나가는
  화소 때문에 저 혼자 움직인다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

from tomato_picker.hardware import kinematics as kin      # noqa: E402
from tomato_picker.hardware.handeye import Intrinsics     # noqa: E402
import grasp_probe as gp                                   # noqa: E402
import visual_servo as vs                                  # noqa: E402

OUT = os.path.expanduser("~/cam_frame.json")
DEG_PER_TICK = 360.0 / 4096.0
PATCH = 48                 # 추적할 조각의 반 크기
SETTLE = 1.0


def frame():
    bgr = cv2.imread(vs.COLOR)
    if bgr is None:
        return None
    try:
        dep = np.load(vs.DEPTH).astype(float)
        meta = json.load(open(vs.META))
    except Exception:                                      # noqa: BLE001
        return None
    return (bgr, dep * float(meta.get("depth_scale_mm", 1.0)),
            Intrinsics.from_dict(meta["intrinsics"]))


def pick_patches(bgr, dep, n=5, zmax=450.0):
    """추적하기 좋은 조각 **여러 개** — 결이 뚜렷하고, 깊이가 성하고, 가까운 데.

    ⚠ 조각 하나로 재면 못 쓴다 — 2026-09-03에 525mm짜리 한 조각으로 쟀더니
      두 열의 직교오차가 0.709였다(0이어야 한다). 깊이 잡음이 걸음과 비슷한
      크기였기 때문이다. **가까울수록** 같은 잡음이 각도로는 작아지고, 여러
      개의 중앙값을 쓰면 하나가 엉뚱해도 답이 안 흔들린다.
    """
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    h, w = g.shape
    cand = []
    for v in range(PATCH + 20, h - PATCH - 20, 30):
        for u in range(PATCH + 20, w - PATCH - 20, 30):
            d = dep[v - 10:v + 11, u - 10:u + 11]
            if (d > 0).mean() < 0.9:
                continue
            z = float(np.median(d[d > 0]))
            if not (90.0 < z < zmax):
                continue
            if float(np.std(d[d > 0])) > 12.0:      # 가장자리(깊이가 튀는 데)는 뺀다
                continue
            s = float(cv2.Laplacian(g[v - PATCH:v + PATCH, u - PATCH:u + PATCH],
                                    cv2.CV_32F).var())
            cand.append((s, u, v, z))
    cand.sort(key=lambda t: -t[0])
    out = []
    for s, u, v, z in cand:
        if all(math.hypot(u - o[1], v - o[2]) > 90 for o in out):
            out.append((s, u, v, z))
        if len(out) >= n:
            break
    return out


def track(tpl, bgr, dep, intr, near):
    """조각을 찾아 3D 점으로 — (X, u, v, 점수)."""
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    r = cv2.matchTemplate(g, tpl, cv2.TM_CCOEFF_NORMED)
    _mn, mx, _ml, ml = cv2.minMaxLoc(r)
    u = ml[0] + tpl.shape[1] // 2
    v = ml[1] + tpl.shape[0] // 2
    if mx < 0.72 or math.hypot(u - near[0], v - near[1]) > 220:
        return None
    d = dep[v - 8:v + 9, u - 8:u + 9]
    d = d[d > 0]
    if d.size < 40:
        return None
    z = float(np.median(d))
    return np.array(intr.deproject(float(u), float(v), z)), u, v, mx


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", type=float, default=22.0, help="한 방향으로 이만큼 움직여 잰다")
    ap.add_argument("--to", default="", help="이 화면점(u,v)을 집게 자리로 데려갈 변위를 낸다")
    ap.add_argument("--grip-uv", default="471,395", help="집게가 무는 화면 자리")
    ap.add_argument("--grip-z", type=float, default=80.0, help="집게가 무는 거리(mm)")
    ap.add_argument("--patches", type=int, default=6)
    ap.add_argument("--zmax", type=float, default=450.0, help="이보다 먼 조각은 안 쓴다")
    ap.add_argument("--maxdeg", type=float, default=20.0,
                    help="한 관절이 이 이상 움직여야 하면 포기 — 피치가 묶여 있으니 커도 된다")
    ap.add_argument("--reuse", action="store_true", help="다시 재지 않고 저장된 값을 쓴다")
    args = ap.parse_args()

    cal = json.load(open(gp.CAL))
    spans = {}
    for name, c in cal.items():
        try:
            spans[name] = abs(int(c["range_max"]) - int(c["range_min"])) * DEG_PER_TICK
        except (KeyError, TypeError, ValueError):
            pass
    cart = json.load(open(gp.CART))
    zero, ref, signs = cart["zero"], cart["ref_deg"], cart.get("signs", {})
    over = cart.get("deg_per_norm") or {}
    geom = kin.ArmGeometry()

    def sign(j):
        v = signs.get(j)
        return -1.0 if (v is not None and float(v) < 0) else 1.0

    def per(j):
        v = over.get(j)
        if v:
            return abs(float(v))
        s = spans.get(j)
        return abs(s) / 200.0 if s else (1.8 if j == "wrist_roll" else 0.9)

    def to_deg(n):
        return {j: ref.get(j, 0.0) + sign(j) * (float(n.get(j, 0.0)) - zero.get(j, 0.0)) * per(j)
                for j in kin.JOINTS}

    def to_norm(d):
        return {j: zero.get(j, 0.0) + (float(d[j]) - ref.get(j, 0.0)) / (sign(j) * per(j))
                for j in d if j in kin.JOINTS}

    def legal(d, cur):
        nm, cm = to_norm(d), to_norm(cur)
        for j, v in nm.items():
            if abs(v) > max(96.0, abs(cm.get(j, 0.0))) + 1e-6:
                return False, "%s 한계" % j
        floor = min(-gp.MOUNT_Z_MM + gp.FLOOR_MARGIN_MM, kin.forward(cur, geom).z - 1.0)
        if kin.forward(d, geom).z < floor:
            return False, "바닥"
        return True, ""

    def solve(cur, want):
        """피치를 묶고 want(base, mm)만큼 병진하는 관절 변화."""
        b0 = kin.forward(cur, geom)
        cols = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
        rows = []
        for j in cols:
            e = dict(cur)
            e[j] += 0.5
            p = kin.forward(e, geom)
            rows.append([(p.x - b0.x) / 0.5, (p.y - b0.y) / 0.5, (p.z - b0.z) / 0.5])
        A = np.vstack([np.array(rows).T, np.array([[0.0, 1.0, 1.0, 1.0]])])
        rhs = np.append(want, 0.0)
        sol, *_ = np.linalg.lstsq(A, rhs, rcond=None)
        return dict(zip(cols, sol)), float(np.linalg.norm(A @ sol - rhs)), float(np.max(np.abs(sol)))

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)

    def press_to(target, tries=3, tol=0.5, cap=5.0):
        cmd = dict(target)
        got = to_deg(io.read())
        for _ in range(tries):
            n = to_norm(cmd)
            n["gripper"] = io.read().get("gripper", 70.0)
            io.write(n, 1.0)
            time.sleep(SETTLE)
            got = to_deg(io.read())
            miss = {j: target[j] - got[j] for j in kin.JOINTS}
            if max(abs(v) for v in miss.values()) <= tol:
                return got
            for j in kin.JOINTS:
                cmd[j] = max(min(cmd[j] + miss[j], target[j] + cap), target[j] - cap)
            if not legal(cmd, got)[0]:
                break
        return got

    try:
        home = to_deg(io.read())
        R = None
        if args.reuse and os.path.exists(OUT):
            R = np.array(json.load(open(OUT))["R_cam_from_base"])
            print("저장된 값을 쓴다:", OUT)
        else:
            f = frame()
            if f is None:
                print("❌ 깊이 프레임을 못 읽었다.")
                return 1
            bgr, dep, intr = f
            pks = pick_patches(bgr, dep, args.patches, args.zmax)
            if not pks:
                pks = pick_patches(bgr, dep, args.patches, 900.0)
            if not pks:
                print("❌ 추적할 조각이 없다 — 결이 있는 곳을 보게 하고 다시.")
                return 1
            gray0 = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            tpls = [(gray0[pv - PATCH:pv + PATCH, pu - PATCH:pu + PATCH].copy(), pu, pv)
                    for _s, pu, pv, _z in pks]
            print("추적할 조각 %d개: %s" % (len(pks),
                  " ".join("(%d,%d)%.0fmm" % (u, v, z) for _s, u, v, z in pks)))

            pan = math.radians(home["shoulder_pan"])
            e_r = np.array([math.cos(pan), math.sin(pan), 0.0])   # 팔이 뻗는 쪽(수평)
            e_z = np.array([0.0, 0.0, 1.0])
            def look_all(near_hint=True):
                f2 = frame()
                if f2 is None:
                    return None
                out = []
                for tpl, pu, pv in tpls:
                    t = track(tpl, f2[0], f2[1], f2[2], (pu, pv))
                    out.append(t)
                return out

            base_pts = look_all()
            p_home = kin.forward(home, geom)
            b_home = np.array([p_home.x, p_home.y, p_home.z])
            DB, DC = [], []
            for name, e in (("반지름", e_r), ("수직", e_z)):
                for sgn in (+1, -1):
                    scale = None
                    for k in (1.0, 0.7, 0.45):
                        dd, res, big = solve(home, e * (args.probe * sgn * k))
                        if res <= 0.06 * args.probe and big <= args.maxdeg:
                            scale = (dd, k)
                            break
                    if scale is None:
                        print("  %s %+d: 자세를 지키며 못 간다" % (name, sgn))
                        continue
                    dd, _k = scale
                    tgt = dict(home)
                    for j, w in dd.items():
                        tgt[j] += float(w)
                    ok, why = legal(tgt, home)
                    if not ok:
                        print("  %s %+d: %s" % (name, sgn, why))
                        continue
                    got = press_to(tgt)
                    time.sleep(0.4)
                    seen = look_all()
                    pg = kin.forward(got, geom)
                    # ⚠ **명령이 아니라 실제로 간 거리를 쓴다.** 중력 처짐 때문에
                    #   명령의 절반만 가는 일이 흔하고, 그러면 열의 크기가
                    #   0.53처럼 나와 회전이 아니게 된다(2026-09-03).
                    db = np.array([pg.x, pg.y, pg.z]) - b_home
                    press_to(dict(home))
                    if seen is None or float(np.linalg.norm(db)) < 4.0:
                        print("  %s %+d: 실제 이동 %.1fmm — 버린다"
                              % (name, sgn, float(np.linalg.norm(db))))
                        continue
                    n_ok = 0
                    for i, t in enumerate(seen):
                        if t is None or base_pts is None or base_pts[i] is None:
                            continue
                        DB.append(db)
                        DC.append(t[0] - base_pts[i][0])
                        n_ok += 1
                    print("  %s %+d: 실제 %.1fmm · 조각 %d개" % (name, sgn,
                          float(np.linalg.norm(db)), n_ok))
            if len(DB) < 6:
                print("표본이 모자란다(%d) — 결이 있고 가까운 데를 보게 하고 다시." % len(DB))
                return 1
            DB = np.array(DB).T                      # 3xN
            DC = np.array(DC).T                      # 3xN
            M = -DC @ np.linalg.pinv(DB)             # ΔX_cam = −R Δbase
            u_, s_, vt = np.linalg.svd(M)
            R = u_ @ np.diag([1.0, 1.0, float(np.sign(np.linalg.det(u_ @ vt)))]) @ vt
            resid = DC + R @ DB
            print("\n표본 %d개 · 특이값 %s (셋 다 1에 가까워야 한다)"
                  % (DB.shape[1], np.round(s_, 2)))
            print("맞춘 뒤 남은 오차 중앙값 %.1fmm"
                  % float(np.median(np.linalg.norm(resid, axis=0))))
            json.dump({"R_cam_from_base": R.tolist(), "pose_deg": home,
                       "probe_mm": args.probe, "samples": int(DB.shape[1]),
                       "sv": [float(x) for x in s_],
                       "when": time.strftime("%Y-%m-%d %H:%M:%S")},
                      open(OUT, "w"), indent=1)
            print("저장:", OUT)

        print("\nbase 축이 카메라계에서 향하는 쪽:")
        for k, nm in enumerate(("base +x", "base +y", "base +z(위)")):
            print("   %-12s → 카메라계 (%+.2f, %+.2f, %+.2f)" % (nm, *R[:, k]))

        if args.to:
            f = frame()
            if f is None:
                return 1
            _bgr, dep, intr = f
            tu, tv = (float(x) for x in args.to.split(","))
            d = dep[int(tv) - 9:int(tv) + 10, int(tu) - 9:int(tu) + 10]
            d = d[d > 0]
            if d.size < 12:
                print("그 점에 깊이가 없다.")
                return 1
            S = np.array(intr.deproject(tu, tv, float(np.percentile(d, 25))))
            gu, gv = (float(x) for x in args.grip_uv.split(","))
            G = np.array(intr.deproject(gu, gv, args.grip_z))
            move = R.T @ (S - G)           # 카메라계 변위 → base 변위
            print("\n줄기 카메라계 (%.1f, %.1f, %.1f)  집게 (%.1f, %.1f, %.1f)"
                  % (*S, *G))
            print("**집게를 그리로 옮기려면 base 좌표로 "
                  "(%+.1f, %+.1f, %+.1f) mm — 모두 %.0fmm**"
                  % (*move, float(np.linalg.norm(move))))
            print("   tool_jog.py --dx %.1f --dy %.1f --dz %.1f"
                  % (move[0], move[1], move[2]))
        return 0
    finally:
        io.hold_close()


if __name__ == "__main__":
    raise SystemExit(main())
