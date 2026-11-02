#!/usr/bin/env python3
"""**깊이별 겨냥점(by_z) 다시 재기** — 줄기를 물린 자리에서 수평으로 물러나며 노란 띠를 본다.

    ~/lerobot/.venv/bin/python ros2/tools/grip_uv_measure.py            # 재서 옛 표와 비교만
    ~/lerobot/.venv/bin/python ros2/tools/grip_uv_measure.py --save     # ~/grip_uv.json by_z 교체(백업 남김)
    python ros2/tools/grip_uv_measure.py --selftest                      # PC, 띠 검출 확인

준비(사람): 줄기에 **노란 테이프 띠(폭 25~30mm)**를 감고, 시작 자세에서 집게가 그 띠 **가운데**를
물게 한다(무는거리 80). 집게 위아래로 띠가 삐져나와야 한다 — 보이는 띠 화소의 중심 = 집게 가운데.

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
    색 문턱은 stem_grasp.tape_point와 같다. 집게가 띠 가운데를 가려 두 조각이 되므로 조각을 합친다."""
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = ((hsv[:, :, 0] > 18) & (hsv[:, :, 0] < 40)
         & (hsv[:, :, 1] > 60) & (hsv[:, :, 2] > 80)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    keep = [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= 150
            and (prev is None or math.hypot(cen[i][0] - prev[0], cen[i][1] - prev[1]) <= jump_px)]
    if not keep:
        return None
    sel = np.isin(lab, keep)
    vs_, us_ = np.nonzero(sel)
    d = dep_mm[sel]
    d = d[d > 0]
    if d.size < 20:
        return None
    return float(us_.mean()), float(vs_.mean()), float(np.median(d))


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


def measure(steps, step_mm, grip_open):
    table = []
    prev = None
    for k in range(steps + 1):
        if k == 1:
            jog("--grip", str(grip_open), "--horiz", str(-step_mm))
        elif k > 1:
            jog("--horiz", str(-step_mm))
        t0 = time.time()
        pts = [band_uvz(*grab(t0 + 0.4 * i), prev=prev) for i in range(3)]
        pts = [p for p in pts if p]
        if not pts:
            print("  %d: 노란 띠를 못 찾았다 — 여기까지" % k)
            break
        u, v, z = (float(np.median([p[j] for p in pts])) for j in range(3))
        dz = z - table[-1][0] if table else 0.0
        warn = "  ⚠ 깊이가 %.0fmm 늘었다(명령 %.0f) — 줄기가 끌려왔나?" % (dz, step_mm) \
            if table and abs(dz - step_mm) > 8.0 else ""
        print("  %d: z %.0fmm  (%.0f, %.0f)%s" % (k, z, u, v, warn))
        table.append([round(z, 1), round(u, 1), round(v, 1)])
        prev = (u, v)
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
    assert band_uvz(np.zeros_like(bgr), dep) is None
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--steps", type=int, default=5, help="물러나는 횟수(기본 5 → 86~190mm 근처)")
    ap.add_argument("--step-mm", type=float, default=20.0)
    ap.add_argument("--grip-open", type=float, default=78.0, help="첫 걸음 전에 여는 집게 값")
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
    new = measure(a.steps, a.step_mm, a.grip_open)
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
