#!/usr/bin/env python3
"""**집게로 벽을 눌러** 파지점을 잰다 — 사람 손 없이.

    ~/lerobot/.venv/bin/python ros2/tools/grasp_probe.py --touches 3

⚠ `tomato-voice`는 내리고 `depth-cam`은 켠 채로. 평평한 면(표적 종이가 붙은 벽)이
   손목 화면에 보이는 자세에서 시작한다. **사람이 보는 앞에서.**

────────────────────────────────────────────────────────────────────────
무엇을 재는가, 왜 이렇게 재는가

시각 서보잉은 300mm에서 130mm까지 매번 잘 붙는다. 막히는 것은 마지막 5~8cm뿐이고,
그 이유는 하나다 — **"집게가 무는 자리"가 카메라 좌표로 어디인지 모른다.**
카메라와 집게는 같은 링크에 붙어 있으니 그건 상수 벡터 하나다. 그 하나만 알면
마지막 구간을 눈 없이도 정확히 갈 수 있다.

문제는 그 자리를 볼 수 없다는 것이었다. 손가락은 무광 검정이라 깊이가 안 잡히고,
바깥 카메라에서는 팔 전체가 같은 색이라 끝을 못 가른다.

그런데 **깊이가 안 잡히는 것 자체가 신호다.** 집게를 평평한 벽에 대고 누르면:

  · 벽은 깊이 영상에 또렷한 평면으로 잡힌다 → 평면의 식을 안다
  · 손가락은 그 평면 앞에서 **구멍**(깊이 없음)으로 남는다 → 화면 어디인지 안다
  · 그리고 닿아 있으므로 손끝은 **그 평면 위에** 있다

그러면 손끝의 3D 좌표가 한 번의 접촉으로 정해진다: 구멍 끝 화소의 시선과 벽
평면이 만나는 점. 여러 자세에서 되풀이해 흩어짐을 보면 믿을 만한지도 알 수 있다.

접촉은 **추종 오차**로 안다 — 벽에 막히면 서보가 지령을 못 따라간다. 이 저장소의
1번 병("지령은 나가는데 아무 일도 안 일어난다")을 거꾸로 쓰는 셈이다.
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

from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware.handeye import Intrinsics  # noqa: E402

DEG_PER_TICK = 360.0 / 4096.0
CAL = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json")
CART = os.path.expanduser("~/arm_cartesian.json")
COLOR, DEPTH, META = ("/dev/shm/d405_color.jpg", "/dev/shm/d405_depth.npy",
                      "/dev/shm/d405_meta.json")
MOUNT_Z_MM = 76.5
FLOOR_MARGIN_MM = 10.0
TOUCH_ERR_DEG = 2.5        # 이만큼 못 따라가면 무언가에 닿은 것이다
STEP_MM = 4.0              # 한 걸음에 벽에 이만큼 다가간다
MAX_STEPS = 70
SETTLE = 0.8
PROBE_DEG = 2.0


def frame():
    try:
        dep = np.load(DEPTH).astype(float)
        meta = json.load(open(META))
    except Exception:                                  # noqa: BLE001
        return None
    return dep * float(meta.get("depth_scale_mm", 1.0)), Intrinsics.from_dict(meta["intrinsics"])


def wall_plane(dep, intr, skip_bottom=0.45):
    """보이는 가장 큰 평면 — (법선 n, 거리 d) 로 n·X = d (카메라계, mm).

    ⚠ 화면 **아래쪽은 빼고** 맞춘다. 벽에 다가갈수록 집게가 아래를 가득 채워
      평면이 망가진다 — 2026-09-02, 접촉 직전에 벽 거리가 155 ↔ 197mm로 튀었다.
      집게는 늘 아래에 맺히므로(여닫기로 실측) 위쪽만 보면 벽만 남는다.
    """
    ok = (dep > 55) & (dep < 1400)
    ok[int(dep.shape[0] * (1.0 - skip_bottom)):, :] = False
    ys, xs = np.nonzero(ok)
    if ys.size < 1200:
        return None
    k = np.random.default_rng(0).choice(ys.size, size=min(5000, ys.size), replace=False)
    ys, xs = ys[k], xs[k]
    z = dep[ys, xs]
    pts = np.stack([(xs - intr.ppx) * z / intr.fx, (ys - intr.ppy) * z / intr.fy, z], axis=1)
    rng = np.random.default_rng(1)
    best = (0, None)
    for _ in range(240):
        s = pts[rng.choice(pts.shape[0], 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        ln = np.linalg.norm(n)
        if ln < 1e-6:
            continue
        n = n / ln
        cnt = int((np.abs((pts - s[0]) @ n) < 8.0).sum())
        if cnt > best[0]:
            best = (cnt, (n, s[0]))
    # ⚠ 가까이 갈수록 집게가 화면을 가려 내부점 비율이 떨어진다. 45%를
    #   고집하면 접촉 직전에 평면을 잃는다 — 2026-09-02, 세 번 다 그랬다.
    if best[1] is None or best[0] < pts.shape[0] * 0.28:
        return None
    n, p0 = best[1]
    inl = pts[np.abs((pts - p0) @ n) < 8.0]
    n = np.linalg.svd(inl - inl.mean(axis=0))[2][2]
    if n[2] < 0:
        n = -n
    return n, float(n @ inl.mean(axis=0)), best[0] / pts.shape[0]


def finger_tip(bgr, dep, intr, n, d):
    """벽 앞의 **검은 집게**를 찾아 그 끝의 3D 좌표를 돌려준다.

    ⚠ 처음엔 깊이 구멍으로 찾으려 했다(손가락은 무광 검정이라 스테레오가 안
      잡힌다). 그런데 구멍은 그림자·가장자리에도 생겨서, 화면 왼쪽 그림자를
      집게로 짚었다 — 2026-09-02. 벽에 대고 누를 때는 배경이 **밝은 벽**이므로
      색으로 잡는 편이 훨씬 또렷하다: 아래쪽에서 가장 큰 검은 덩이.

    끝은 그 덩이의 **위쪽 끝**이다 — 이 카메라에서 집게는 아래에서 위로 뻗는다.
    """
    h, w = dep.shape
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    dark = (g < max(70, int(np.percentile(g, 25) * 0.6))).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    band = np.zeros_like(dark)
    band[int(h * 0.55):, int(w * 0.20):int(w * 0.85)] = 1
    m = dark * band
    nc, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, nc):
        a = int(st[i, cv2.CC_STAT_AREA])
        if a < 700:
            continue
        if best is None or a > best[0]:
            best = (a, i)
    if best is None:
        return None
    i = best[1]
    ys, xs = np.nonzero(lab == i)
    # ⚠ 가장 위쪽 화소 하나를 쓰면 **두 손가락 중 더 높은 쪽**이 잡혀 좌우가
    #   번갈아 나온다 — 2026-09-02, x가 -15.8 ~ +22.8mm로 흔들렸다. 무는 자리는
    #   둘 사이 가운데이므로 위쪽 몇 줄의 가로 평균을 쓴다.
    top = ys.min()
    sel = ys <= top + 12
    u, v = float(xs[sel].mean()), float(ys[sel].mean())
    ray = np.array(intr.deproject(u, v, 1.0))
    tip = ray * (d / float(n @ ray))       # 시선과 벽 평면이 만나는 점
    return tip, u, v, best[0]


def approach_dir(d, geom):
    """도구가 향하는 방향(base 좌표) — 집게가 뻗는 쪽."""
    pan = math.radians(d["shoulder_pan"])
    a3 = math.radians(d["shoulder_lift"] + d["elbow_flex"] + d["wrist_flex"])
    return np.array([math.cos(a3) * math.cos(pan),
                     math.cos(a3) * math.sin(pan), math.sin(a3)])


def step_along(d, geom, mm, joints=("shoulder_lift", "elbow_flex", "wrist_flex")):
    """도구 원점을 **접근축을 따라** mm 만큼 옮기는 관절 변화량.

    ⚠ 벽 거리의 변화로 방향을 추정하면 안 된다 — 평면 맞춤의 잡음(±3mm)이
      야코비안 부호를 뒤집어 팔이 뒤로 갔다(2026-09-02: 382 → 394mm).
      기구학은 잡음이 없다. 회전 모델은 1°까지 맞으니 20mm 걸음에는 충분하다.
    """
    p0 = kin.forward(d, geom)
    b0 = np.array([p0.x, p0.y, p0.z])
    J = []
    for j in joints:
        e = dict(d)
        e[j] += 0.5
        p1 = kin.forward(e, geom)
        J.append((np.array([p1.x, p1.y, p1.z]) - b0) / 0.5)
    A = np.array(J).T                      # (3 x k) mm/도
    want = approach_dir(d, geom) * mm
    sol, *_ = np.linalg.lstsq(A, want, rcond=None)
    return dict(zip(joints, sol)), float(np.linalg.norm(A @ sol - want))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--touches", type=int, default=3)
    ap.add_argument("--dry", action="store_true", help="누르지 않고 지금 화면만 본다")
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
            if abs(v) > max(98.0, abs(cm.get(j, 0.0))) + 1e-6:
                return False
        floor = min(-MOUNT_Z_MM + FLOOR_MARGIN_MM, kin.forward(cur, geom).z - 1.0)
        return kin.forward(d, geom).z >= floor

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    grip = 3.0                                   # 닫은 채로 — 무는 자리를 재는 것이다

    def go(d, secs=0.9):
        n = to_norm(d)
        n["gripper"] = grip
        io.write(n, secs)
        time.sleep(SETTLE)
        got = to_deg(io.read())
        return got, max(abs(got[j] - d[j]) for j in kin.JOINTS)

    here = to_deg(io.read())
    go(here, 0.6)
    f = frame()
    if f is None:
        print("❌ 깊이 프레임을 못 읽었다.")
        io.hold_close()
        return 1
    dep, intr = f
    pl = wall_plane(dep, intr)
    if pl is None:
        print("❌ 평평한 면이 안 보인다 — 벽(표적 종이)이 보이는 자세에서 시작하라.")
        io.hold_close()
        return 1
    print("벽까지 %.0fmm · 화소의 %.0f%%가 그 평면 위" % (pl[1], pl[2] * 100))
    if args.dry:
        bgr = cv2.imread(COLOR)
        hit = finger_tip(bgr, dep, intr, pl[0], pl[1])
        print("지금 집게 끝:", "없음" if hit is None
              else "화면(%.0f,%.0f) 넓이%d · 카메라계 (%.1f,%.1f,%.1f)"
              % (hit[1], hit[2], hit[3], hit[0][0], hit[0][1], hit[0][2]))
        io.hold_close()
        return 0

    tips = []
    base0 = to_deg(io.read())
    # 자세를 조금씩 달리해 눌러야 **평면 제약이 서로 다른 방향**이 되어 답이 정해진다.
    for t in range(args.touches):
        tilt = (t - (args.touches - 1) / 2.0) * 7.0
        go(dict(base0), 1.4)          # ⚠ 늘 같은 곳에서 출발한다 — 앞 접근이
        start = dict(base0)           #    남긴 자세에서 이어가면 한계에 갇힌다.
        start["wrist_flex"] = base0["wrist_flex"] + tilt
        start["shoulder_lift"] = base0["shoulder_lift"] - tilt
        if not legal(start, to_deg(io.read())):
            print("  %d번 자세는 한계 — 건너뜀" % (t + 1))
            continue
        go(start, 1.2)
        print("\n[%d/%d] 기울기 %+.0f° 에서 벽으로 다가간다" % (t + 1, args.touches, tilt))

        f0 = frame()
        p0 = wall_plane(*f0) if f0 else None
        if p0 is None:
            print("   평면을 못 잡았다 — 건너뜀")
            continue

        touched, miss, stall = None, 0, 0
        for k in range(MAX_STEPS):
            f1 = frame()
            p1 = wall_plane(*f1) if f1 else None
            if p1 is None:
                miss += 1
                if miss >= 3:
                    print("   평면을 잇달아 잃었다 — 멈춘다")
                    break
                time.sleep(0.5)
                continue
            miss = 0
            cur = to_deg(io.read())
            want = float(np.clip(0.30 * (p1[1] - 45.0), 2.5,
                                 20.0 if p1[1] > 260 else 7.0))
            dq, resid = step_along(cur, geom, want)
            nxt = dict(cur)
            for j, v in dq.items():
                nxt[j] += float(np.clip(v, -8.0, 8.0))
            if not legal(nxt, cur):
                print("   한계에 걸렸다 — 멈춘다")
                break
            got, err = go(nxt, 0.8)
            f2 = frame()
            p2 = wall_plane(*f2) if f2 else None
            dist = p2[1] if p2 else -1
            moved = (p1[1] - dist) if dist > 0 else 0.0
            print("   %2d  벽 %5.0fmm (전진 %4.1f 실제 %4.1f)  추종오차 %4.1f°"
                  % (k + 1, dist, want, moved, err), end="")
            # 접촉 = **앞으로 가라고 했는데 안 가까워진다**. 이 팔은 중력 처짐만으로
            # 추종오차 2.0°가 늘 떠 있어서 오차만으로는 못 가른다.
            if dist > 0 and want > 3.5 and moved < 0.5 * want:
                stall += 1
            else:
                stall = 0
            # 전진이 죽고 **동시에** 서보가 밀리면 그게 접촉이다. 하나만으로는
            # 잡음이나 한계와 못 가른다.
            if (stall >= 1 and err > 2.2) or stall >= 3 or err > TOUCH_ERR_DEG + 1.5:
                print("   ← 닿았다")
                touched = (f2, p2)
                break
            print("")
        if touched is None:
            print("   접촉을 못 잡았다")
            continue

        (dep2, intr2), pl2 = touched
        if pl2 is None:
            print("   닿았는데 평면을 잃었다 — 못 쓴다")
        else:
            hit = finger_tip(cv2.imread(COLOR), dep2, intr2, pl2[0], pl2[1])
            if hit is None:
                print("   집게를 못 찾았다 — 못 쓴다")
            else:
                tip, u, v, a = hit
                tips.append(tip)
                print("   손끝 화면(%.0f,%.0f) → 카메라계 (%.1f, %.1f, %.1f) mm"
                      % (u, v, tip[0], tip[1], tip[2]))
        # 물러난다 — 접근축을 따라 뒤로.
        cur = to_deg(io.read())
        dq, _ = step_along(cur, geom, -60.0)
        back = dict(cur)
        for j, v in dq.items():
            back[j] += float(np.clip(v, -20.0, 20.0))
        if legal(back, cur):
            go(back, 1.4)

    go(dict(base0), 1.4)
    io.hold_close()

    print()
    if not tips:
        print("❌ 쓸 만한 접촉이 없었다.")
        return 1
    T = np.array(tips)
    m = T.mean(axis=0)
    spread = float(np.sqrt(((T - m) ** 2).sum(axis=1).mean())) if len(T) > 1 else -1
    print("파지점(카메라 좌표) = (%.1f, %.1f, %.1f) mm   [%d회]" % (m[0], m[1], m[2], len(T)))
    if spread >= 0:
        print("  회차 간 흩어짐 %.1f mm" % spread)
    out = os.path.expanduser("~/grasp_point.json")
    with open(out, "w", encoding="utf-8") as fh:
        json.dump({"grasp_cam_mm": m.tolist(), "spread_mm": spread,
                   "touches": [t.tolist() for t in tips]}, fh, ensure_ascii=False, indent=1)
    print("  저장: %s" % out)
    print("\n이 값이 있으면 마지막 몇 cm는 눈 없이 간다 —")
    print("  줄기를 카메라 좌표로 재고, 이 점으로 오도록 팔을 옮기면 된다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
