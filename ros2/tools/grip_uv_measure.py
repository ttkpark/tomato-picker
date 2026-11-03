#!/usr/bin/env python3
"""**깊이별 겨냥점(by_z) 다시 재기** — 줄기를 물린 자리에서 수평으로 물러나며 노란 띠를 본다.

    ~/lerobot/.venv/bin/python ros2/tools/grip_uv_measure.py            # 재서 옛 표와 비교만
    ~/lerobot/.venv/bin/python ros2/tools/grip_uv_measure.py --save     # ~/grip_uv.json by_z 교체(백업 남김)
    python ros2/tools/grip_uv_measure.py --selftest                      # PC, 띠 검출 확인

준비(사람): 줄기(고리 손잡이~꽃받침 사이)에 **노란 테이프 띠(폭 5~8mm)**를 한 바퀴 감는다.
열린 집게를 줄기 앞(깊이 ≈86mm)에 두고, 사람이 화분을 옮겨 띠를 두 손가락 사이 **높이 가운데**에 둔 뒤 닫는다.
→ 띠 = 무는 자리. 문 채로는 띠가 가려져도 된다(0번은 건너뛰고 물러난 뒤부터 잰다).

⚠ 왜 띠인가: 세로 줄기는 위아래가 화면에 안 보여서(11/02 13~21회의 병) 맨 줄기로는 v를 못 잰다.
⚠ 왜 다시 재나: 10/30 표로 화면 8px까지 맞췄는데 집게는 줄기보다 높게 닫혔다(11/02 21회).
   그 뒤 팔 연결(P 게인 24·configure 생략)과 처짐 되먹임이 바뀌었다.
⚠ 물러나기는 `tool_jog --horiz`(stem_grasp `step_keep`과 같은 수평 전진의 반대)라야 표가 맞는다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
COLOR, DEPTH, META = "/dev/shm/d405_color.jpg", "/dev/shm/d405_depth.npy", "/dev/shm/d405_meta.json"
GRIP_UV = os.path.expanduser("~/grip_uv.json")
PY = sys.executable


def band_uvz(bgr, dep_mm, prev=None, jump_px=120.0):
    """노란 띠 화소의 중심 (u, v)와 깊이 중앙값 z(mm). 못 찾으면 None.
    집게가 띠 가운데를 가려 두 조각이 되므로 조각을 합친다.
    ⚠ 색상 24~40: 11/02 실측 테이프 H≈30, 주황 토마토 H 13~20 — tape_point의 18은 토마토를 띠로 잡았다."""
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = ((hsv[:, :, 0] > 24) & (hsv[:, :, 0] < 40)
         & (hsv[:, :, 1] > 120) & (hsv[:, :, 2] > 50)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    keep = [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= 150
            and (prev is None or math.hypot(cen[i][0] - prev[0], cen[i][1] - prev[1]) <= jump_px)]
    zs = {}
    for i in keep:
        d = dep_mm[lab == i]
        d = d[d > 0]
        if d.size >= 20:
            zs[i] = float(np.median(d))
    if not zs:
        return None
    # ⚠ 주황 토마토의 밝은 곳도 노랑 문턱을 넘는다 — 띠는 줄기 위라 **가장 가까운** 노랑이다. 그것과 15mm 안만 남긴다.
    near = min(zs.values())
    keep = [i for i, z in zs.items() if z <= near + 15.0]
    sel = np.isin(lab, keep)
    vs_, us_ = np.nonzero(sel)
    d = dep_mm[sel]
    d = d[d > 0]
    if d.size < 20:
        return None
    return float(us_.mean()), float(vs_.mean()), float(np.median(d))


def tomato_top_uvz(bgr, dep_mm, near_mm=150.0, max_z=250.0, min_area=1500):
    """토마토(주황) 꼭대기 (u, v)와 그 둘레 깊이 z(mm). 맨 줄기(띠 없음)의 기준점 — 줄기는 토마토 꼭대기 바로 위다.
    ⚠ 마룻바닥도 주황 색상(H 15~20)이라 색만으로는 토마토와 한 덩어리가 된다(11/03 시작 사진 53장, 폭 200px) —
    가장 가까운 주황 깊이 +near_mm 안만 남긴다(바닥은 멀다). ⚠ 토마토가 커서(지름 ≈10cm) 가까이선 깊이가 60~200mm에
    걸친다 — 50mm로 자르니 꼭대기가 잘려 None이었다(11/03 실측 near 62·띠 151). max_z 너머에 주황뿐이면(바닥만) None.
    u는 꼭대기 쪽 15% 줄의 평균(집게가 아래를 가려도 된다)."""
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    dm = cv2.medianBlur(dep_mm.astype(np.float32), 5)      # 한 점 튄 깊이가 '가장 가까운 것'이 되지 않게
    m = ((hsv[:, :, 0] >= 3) & (hsv[:, :, 0] <= 22)
         & (hsv[:, :, 1] > 110) & (hsv[:, :, 2] > 70) & (dm > 0))
    if m.sum() < 500:
        return None
    near = float(dm[m].min())
    if near > max_z:
        return None
    m = cv2.morphologyEx((m & (dm <= near + near_mm)).astype(np.uint8), cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    keep = [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= min_area]
    if not keep:
        return None
    i = max(keep, key=lambda j: st[j, cv2.CC_STAT_AREA])
    vs, us = np.nonzero(lab == i)
    vtop = float(vs.min())
    top = vs <= vtop + 0.15 * (vs.max() - vtop)
    d = dep_mm[vs[top], us[top]]
    d = d[d > 0]
    if d.size < 20:
        return None
    return float(us[top].mean()), vtop, float(np.median(d))


def grab(after):
    """after(초) 이후에 쓰인 프레임을 기다려 읽는다 — 움직이기 전 프레임으로 재면 표가 통째로 틀린다."""
    import cv2
    t_end = time.time() + 5.0
    while time.time() < t_end:
        if os.path.getmtime(COLOR) > after + 0.3 and os.path.getmtime(DEPTH) > after + 0.3:
            time.sleep(0.15)
            bgr = cv2.imread(COLOR)
            sc = float(json.load(open(META)).get("depth_scale_mm", 1.0))
            dep = np.load(DEPTH).astype(float) * sc
            if bgr is not None and dep.shape[:2] == bgr.shape[:2]:
                return bgr, dep
        time.sleep(0.1)
    raise SystemExit("새 프레임이 안 온다 — depth-cam이 도는지 보라")


def jog(*a):
    r = subprocess.run([PY, os.path.join(HERE, "tool_jog.py"), *a])
    if r.returncode:
        raise SystemExit("tool_jog %s 실패(rc %d) — 멈춘다" % (" ".join(a), r.returncode))


def pose():
    """지금 TCP 높이(mm)와 피치(lift+elbow+wrist, 도) — 실제로 읽은 자리(FK)."""
    sys.path.insert(0, HERE)
    sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))
    sys.path.insert(0, os.path.join(HERE, "..", "src", "tomato_bridge"))
    import arm_calib
    from tomato_picker.hardware import kinematics as kin
    from tomato_bridge.follower_io import FollowerIO
    cal = arm_calib.Calib()
    io = FollowerIO(hold_torque=True)
    try:
        d = cal.to_deg(io.read())
    finally:
        io.hold_close()
    return kin.forward(d, cal.geom).z, d["shoulder_lift"] + d["elbow_flex"] + d["wrist_flex"]


def measure(steps, step_mm, grip_open, left_mm, fx, fy):
    table = []
    prev = None
    h0, p0 = pose()      # 문 자리의 TCP 높이·피치 — 표는 이 높이·피치로 수평 전진할 때의 화면이다
    raw = []
    for k in range(steps + 1):
        if k == 1:
            # 놓기와 같은 순서 — 열기 → 고정 턱 떼기(왼쪽) → 빼기. 안 떼면 고정 턱이 줄기를 끌고 나온다(11/01, 11/02 1걸음 z 그대로)
            jog("--grip", str(grip_open), "--left", str(left_mm))
            jog("--horiz", str(-step_mm))
        elif k > 1:
            jog("--horiz", str(-step_mm))
        h, pt = pose()
        t0 = time.time()
        pts = [band_uvz(*grab(t0 + 0.4 * i), prev=prev) for i in range(3)]
        pts = [p for p in pts if p]
        if not pts:
            if k == 0:       # 문 채로는 띠가 집게에 가려질 수 있다 — 물러난 뒤부터 잰다
                print("  0: 문 자리에선 띠가 안 보인다(집게가 가림) — 물러나며 잰다")
                continue
            print("  %d: 노란 띠를 못 찾았다 — 여기까지" % k)
            break
        u, v, z = (float(np.median([p[j] for p in pts])) for j in range(3))
        if k >= 1:
            u -= fx * left_mm / z      # 왼쪽으로 비킨 만큼 화면에서 오른쪽으로 밀려 보인다 — 무는 자리로 되돌린다
        dz = z - table[-1][0] if table else 0.0
        if table and dz < 0.3 * step_mm:
            print("  %d: 깊이가 %.0fmm밖에 안 늘었다 — 못 물러났다(한계?), 여기까지" % (k, dz))
            break
        raw.append((k, z, u, v, h, pt))
        warn = "  ⚠ 깊이가 %.0fmm 늘었다(명령 %.0f) — 줄기가 끌려왔나?" % (dz, step_mm) \
            if table and abs(dz - step_mm) > 8.0 else ""
        print("  %d: z %.0fmm  (%.0f, %.0f)%s" % (k, z, u, v, warn))
        table.append([round(z, 1), round(u, 1), round(v, 1)])
        prev = (u, v)
    # ⚠ 물러나는 동안 팔이 처지고(11/02: 5걸음에 14mm) 피치도 샌다(3걸음에 4°) — 카메라가 낮아지거나 숙여지면
    #   띠가 화면에서 움직인다. 잡기의 전진은 처짐 되먹임(go)이라 문 자리 높이·피치를 지킨다. 그래서 읽은 자세로 되돌린다:
    #   Δv = −fy·cosθ·Δh/z − fy·tan(Δpitch), cosθ = 수평 1mm당 깊이 변화(카메라가 숙인 각).
    #   11/02 첫 판은 높이만 갚았더니 104mm에서 실제보다 28px 위였다(피치 몫).
    st = [(b[1] - a[1]) / step_mm for a, b in zip(raw, raw[1:]) if a[0] >= 1]
    c = float(np.clip(np.median(st), 0.3, 1.0)) if st else 0.8
    for row, (k, z, u, v, h, pt) in zip(table[-len(raw):], raw):
        row[2] = round(v - fy * c * (h - h0) / z - fy * math.tan(math.radians(pt - p0)), 1)
        print("  %d: 높이 %+.1fmm 피치 %+.1f° → v %.0f → %.0f (cosθ %.2f)" % (k, h - h0, pt - p0, v, row[2], c))
    return table


def compare(new, old):
    sys.path.insert(0, HERE)
    from stem_grasp import uv_at_depth
    for z, u, v in new:
        o = uv_at_depth(old, z) if old else None
        print("  z %5.0f  새 (%4.0f,%4.0f)" % (z, u, v)
              + ("  옛 (%4.0f,%4.0f)  차 du %+4.0f dv %+4.0f" % (o[0], o[1], u - o[0], v - o[1]) if o else ""))


def selftest():
    import cv2
    bgr = np.zeros((480, 848, 3), np.uint8)
    dep = np.full((480, 848), 400.0)
    yel = (0, 220, 230)                                    # BGR 노랑
    cv2.rectangle(bgr, (420, 330), (440, 345), yel, -1)    # 집게 위로 삐져나온 조각
    cv2.rectangle(bgr, (420, 375), (440, 390), yel, -1)    # 아래 조각 — 가운데(346~374)는 집게가 가림
    dep[330:391, 420:441] = 86.0
    u, v, z = band_uvz(bgr, dep)
    assert abs(u - 430) < 1 and abs(v - 360) < 1 and abs(z - 86) < 1, (u, v, z)
    assert band_uvz(bgr, dep, prev=(700, 100)) is None     # 멀리 튄 것은 버린다
    t = bgr.copy()
    cv2.rectangle(t, (300, 400), (380, 470), yel, -1)      # 뒤의 노란 토마토(더 멀다)
    u2, v2, _ = band_uvz(t, dep)
    assert abs(u2 - 430) < 1 and abs(v2 - 360) < 1, (u2, v2)
    assert band_uvz(np.zeros_like(bgr), dep) is None
    # 토마토 꼭대기: 주황 바닥(멀다, 화면 아래 절반)과 가까운 주황 공 — 공 꼭대기 (430, 380)@120mm
    f = np.zeros((480, 848, 3), np.uint8)
    f[300:, :] = (90, 140, 200)                             # 마룻바닥 비슷한 주황
    cv2.circle(f, (430, 420), 40, (20, 110, 240), -1)
    d = np.full((480, 848), 400.0)
    cv2.circle(d, (430, 420), 40, 120.0, -1)
    u, v, z = tomato_top_uvz(f, d)
    assert abs(u - 430) < 2 and abs(v - 380) < 2 and abs(z - 120) < 1, (u, v, z)   # 열기 연산이 꼭대기 1px를 깎는다
    assert tomato_top_uvz(f, np.full((480, 848), 400.0)) is None   # 공 없이 바닥만(멀다)이면 None
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--steps", type=int, default=5, help="물러나는 횟수(기본 5 → 86~190mm 근처)")
    ap.add_argument("--step-mm", type=float, default=20.0)
    ap.add_argument("--grip-open", type=float, default=78.0, help="첫 걸음 전에 여는 집게 값")
    ap.add_argument("--left-mm", type=float, default=12.0, help="열고 나서 고정 턱을 떼는 옆 이동(u는 보정)")
    ap.add_argument("--save", action="store_true", help="~/grip_uv.json의 by_z를 바꾼다(.bak 남김)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    try:
        g = json.load(open(GRIP_UV))
    except Exception:                                      # noqa: BLE001
        g = {}
    print("줄기를 문 자리에서 시작 — 노란 띠가 집게 위아래로 보여야 한다")
    try:
        fx = float(json.load(open(META))["intrinsics"]["fx"])
        fy = float(json.load(open(META))["intrinsics"]["fy"])
    except Exception:                                      # noqa: BLE001
        fx = fy = 438.0
    new = measure(a.steps, a.step_mm, a.grip_open, a.left_mm, fx, fy)
    if len(new) < 3:
        raise SystemExit("점이 %d개뿐 — 표로 못 쓴다(저장 안 함)" % len(new))
    compare(new, g.get("by_z"))
    if a.save:
        if os.path.exists(GRIP_UV):
            shutil.copy(GRIP_UV, GRIP_UV + ".bak-" + time.strftime("%m%d-%H%M"))
        g["by_z"] = new
        g["by_z_when"] = time.strftime("%Y-%m-%d %H:%M")
        json.dump(g, open(GRIP_UV, "w"), indent=1)
        print("저장: %s by_z %d점" % (GRIP_UV, len(new)))
    else:
        print("(저장 안 함 — 맞으면 --save로 다시, 또는 위 표를 손으로)")


if __name__ == "__main__":
    main()
