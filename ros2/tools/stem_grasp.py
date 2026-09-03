#!/usr/bin/env python3
"""**줄기를 문다** — 화면으로 겨누고, 닿을 때까지 나아가고, 닫는다.

    ~/lerobot/.venv/bin/python ros2/tools/stem_grasp.py --dry     # 겨냥만 본다
    ~/lerobot/.venv/bin/python ros2/tools/stem_grasp.py           # 물기까지

⚠ `tomato-voice`는 내리고 `depth-cam`은 켠 채로. **사람이 보는 앞에서만.**

────────────────────────────────────────────────────────────────────────
왜 깊이 목표를 버렸나

`visual_servo.py`는 "줄기를 카메라계 (u,v,z) 한 점으로 끌어온다"였고, 그 z는
`grasp_probe.py`가 벽을 눌러 잰 263mm였다. 그런데 **그 263mm이 틀렸다.**
벽 누르기의 접촉 판정이 일찍 울리면 손끝은 벽에 닿지도 않았는데 손끝 자리를
**벽 평면 위로** 찍는다 — 그러면 z는 손끝 거리가 아니라 그때의 벽 거리가 된다.
2026-09-02: 그 값을 목표로 놓자 서보가 걸음마다 "+130mm 물러나라"고 했고,
바깥 카메라에는 집게와 열매 사이가 뻔히 8cm 비어 있었다.

그래서 z를 **목표로 쓰지 않고 관측으로만 쓴다.**

  1. 화면에서 줄기를 집게 자리(`TARGET_UV`)로 끌어온다 — 여기엔 깊이가 없다.
  2. 접근축을 따라 조금씩 나아간다(`step_along`, 기구학이라 잡음이 없다).
  3. **줄기의 깊이가 걸음만큼 줄지 않으면 닿은 것이다.** 20mm를 명령했는데
     깊이가 4mm밖에 안 줄었다면 앞을 무언가가 막고 있다 — 그게 열매다.
  4. 닫는다.

이 절차의 부산물이 곧 우리가 못 재고 있던 값이다: **닿은 순간의 깊이가
"카메라에서 손끝까지"**다. 벽처럼 조기 접촉으로 오염되지 않는다 — 여기서는
"안 줄었다"가 곧 접촉의 정의이기 때문이다.

⚠ 손-눈 보정도, 링크 길이도, 영점도 식에 안 들어간다. 카메라와 집게가 같은
  링크에 붙어 있다는 사실 하나만 쓴다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))

from tomato_picker.hardware import kinematics as kin      # noqa: E402
from tomato_picker.hardware.handeye import Intrinsics      # noqa: E402
import arm_calib                                            # noqa: E402
import grasp_probe as gp                                   # noqa: E402
import visual_servo as vs                                  # noqa: E402

DEG_PER_TICK = arm_calib.DEG_PER_TICK
CAL = gp.CAL
CART = gp.CART
MOUNT_Z_MM = gp.MOUNT_Z_MM
FLOOR_MARGIN_MM = gp.FLOOR_MARGIN_MM
FRUIT_MM = 70.0          # 실측: 134.6mm에서 폭 227화소, fx 438 → 69.8mm
SETTLE = 0.8

# ⚠ 겨냥에 wrist_flex를 쓰지 않는다 — 이 자세에서 −97(한계 −98)로 박혀 있고,
#   화면을 아래로 내리려면 **더 음수**가 필요해서 14걸음 내내 한 발도 못 뗐다
#   (2026-09-02). 어깨를 올리면 카메라가 올라가 장면이 아래로 내려온다 —
#   같은 일을 여유가 60 넘게 남은 관절로 한다.
AIM = ("shoulder_pan", "shoulder_lift", "elbow_flex")
GRIP_OPEN, GRIP_SHUT = 78.0, 4.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true", help="움직이지 않고 지금 겨냥만 본다")
    ap.add_argument("--adv", type=float, default=14.0, help="한 걸음에 나아갈 mm")
    ap.add_argument("--max-adv", type=float, default=200.0, help="다 합쳐 이 이상은 안 간다")
    ap.add_argument("--steps", type=int, default=22)
    ap.add_argument("--tol", type=float, default=28.0, help="이 화소 안이면 겨냥은 됐다")
    ap.add_argument("--gain", type=float, default=0.55)
    ap.add_argument("--max-turn", type=float, default=6.0, help="한 걸음에 한 관절 최대 도수")
    ap.add_argument("--probe", type=float, default=2.5, help="야코비안을 잴 때 흔드는 도수")
    ap.add_argument("--rejacobian", type=int, default=4, help="몇 걸음마다 다시 재는가")
    ap.add_argument("--aim", choices=("click", "mark", "top", "stem", "fruit", "auto"),
                     default="top")
    ap.add_argument("--mark", default="", help="줄기를 화면에서 여기라고 알려 준다 u,v")
    ap.add_argument("--click-file", default=os.path.expanduser("~/click_target.json"),
                    help="클릭 페이지(click_server.py)가 남긴 표적")
    ap.add_argument("--above-mm", type=float, default=16.0,
                    help="열매에서 줄기 쪽으로 이만큼 떨어진 데를 문다")
    ap.add_argument("--target", default="", help="겨눌 화면좌표 u,v (비우면 실측한 집게자리)")
    ap.add_argument("--stop-z", type=float, default=84.0,
                    help="겨눈 자리가 이 mm까지 오면 문다 — 집게가 무는 거리(실측 75~80mm)")
    ap.add_argument("--no-close", action="store_true", help="닿아도 닫지 않는다")
    args = ap.parse_args()

    if args.target:
        tu, tv = (float(x) for x in args.target.split(","))
    else:
        tu, tv = vs.TARGET_UV
        try:                                   # 페이지에서 고쳐 찍은 십자가 이긴다
            g = json.load(open(os.path.expanduser("~/grip_uv.json")))
            tu, tv = float(g["u"]), float(g["v"])
            print("십자를 사람이 고쳐 찍었다 [%s]" % g.get("when", ""))
        except Exception:                                  # noqa: BLE001
            pass
    print("겨눌 자리 = 화면 (%.0f, %.0f)  [집게가 무는 자리, 여닫아 실측]" % (tu, tv))

    # ⚠ 한계 판정은 arm_calib.Calib 하나뿐이다(자세한 이유는 arm_calib.py
    #   머리말) — 정규값 ±98 같은 임의 숫자 대신 서보 캘리브레이션에서 나온
    #   진짜 물리 가동범위를 쓰므로, "교시자세를 중심으로 대칭이 아니라서
    #   98을 그냥 쓰면 못 움직인다"는 예전 문제 자체가 없다.
    calib = arm_calib.Calib()
    geom = calib.geom
    to_deg, to_norm, legal = calib.to_deg, calib.to_norm, calib.legal

    def fruit_box(prev):
        """빨간 열매의 (넓이, 중심u, 윗변v, 너비, 깊이) — 테두리까지 필요하다."""
        import cv2
        bgr = cv2.imread(vs.COLOR)
        if bgr is None:
            return None
        try:
            dep = np.load(vs.DEPTH).astype(float)
            meta = json.load(open(vs.META))
        except Exception:                              # noqa: BLE001
            return None
        sc = float(meta.get("depth_scale_mm", 1.0))
        fxv = float(meta["intrinsics"]["fx"])
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m = ((hsv[:, :, 1] > 80) & (hsv[:, :, 2] > 50)
             & ((hsv[:, :, 0] < 14) | (hsv[:, :, 0] > 165))).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
        best = None
        for i in range(1, n):
            a = int(st[i, cv2.CC_STAT_AREA])
            if a < 600:
                continue
            w = float(st[i, cv2.CC_STAT_WIDTH])
            hgt = float(st[i, cv2.CC_STAT_HEIGHT])
            x0 = float(st[i, cv2.CC_STAT_LEFT])
            # ⚠ **둥근 것만 열매다.** 선반의 주황 안전조끼가 화면 왼쪽에서
            #   30000화소로 잡혀 서보가 그쪽으로 팔을 끌고 갔다 — 8px까지
            #   "수렴"해 놓고 보니 집게가 바닥을 향해 코를 박고 있었다
            #   (2026-09-03). 조끼는 길쭉하고 성글다: 그 둘로 가른다.
            if not (0.55 < w / max(hgt, 1.0) < 1.8):
                continue
            if a < 0.55 * w * hgt:
                continue
            d = dep[lab == i] * sc
            d = d[d > 0]
            # ⚠ **깊이가 없다고 열매를 버리면 안 된다.** D405는 70mm보다 가까운
            #   것을 통째로 비운다 — 2026-09-02에 16mm 나아간 직후 "놓쳤다"고
            #   멈춘 게 이것이었다. 무는 순간이 바로 그 거리다.
            #   열매는 지름을 아니까(실측 70mm) 크기로 이어서 잰다.
            edge = (x0 <= 1 or x0 + w >= bgr.shape[1] - 2)
            if d.size >= 25:
                z = float(np.percentile(d, 20))
            elif not edge and w > 8:
                z = fxv * FRUIT_MM / w
            else:
                continue
            u, v = float(cen[i][0]), float(cen[i][1])
            if prev is not None and math.hypot(u - prev[0], v - prev[1]) > 220.0:
                continue
            score = a if prev is None else -math.hypot(u - prev[0], v - prev[1])
            if best is None or score > best[0]:
                # ⚠ 화면은 **뒤집혀 있지 않다.** 2026-09-02에 "링이 열매 아래로
                #   찍히니 상하 반전"이라고 판단했는데, 그건 *아래쪽* 링이었다.
                #   9-03에 집게가 화면 아래, 열매가 화면 위로 함께 찍힌 장면으로
                #   확정했다 — 세상에서 위인 것이 화면에서도 위다. 그러니 줄기는
                #   열매 **윗변 위**에 있다.
                top = float(st[i, cv2.CC_STAT_TOP])
                best = (score, a, u, top, w, z, top <= 2.0)
        return None if best is None else best[1:]

    def tape_point(prev):
        """**노란 테이프 아래 지점** — 열매가 없어질 때의 대체 표적.

        ⚠ 무인 반복 중 열매가 떨어지거나 다 땄으면 `fruit_box`가 계속 None을
          준다. 그러면 서보가 매번 "겨눌 것을 못 찾았다"로 멈춰야 하는데,
          사람이 없으면 그걸 다시 살릴 수 없다. 노란 테이프를 표식으로 남겨
          두면 **테이프 아래 지점**을 계속 겨눌 수 있다 — 열매 대신 쓰는
          고정 표적이다. 둥글기 조건은 안 건다(테이프는 안 둥글다).
        """
        import cv2
        bgr = cv2.imread(vs.COLOR)
        if bgr is None:
            return None
        try:
            dep = np.load(vs.DEPTH).astype(float)
            meta = json.load(open(vs.META))
        except Exception:                              # noqa: BLE001
            return None
        sc = float(meta.get("depth_scale_mm", 1.0))
        fxv = float(meta["intrinsics"]["fx"])
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m = ((hsv[:, :, 0] > 18) & (hsv[:, :, 0] < 40)
             & (hsv[:, :, 1] > 60) & (hsv[:, :, 2] > 80)).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
        best = None
        for i in range(1, n):
            a = int(st[i, cv2.CC_STAT_AREA])
            if a < 200:
                continue
            u, v = float(cen[i][0]), float(cen[i][1])
            if prev is not None and math.hypot(u - prev[0], v - prev[1]) > 260.0:
                continue
            if best is None or a > st[best, cv2.CC_STAT_AREA]:
                best = i
        if best is None:
            return None
        bottom = float(st[best, cv2.CC_STAT_TOP] + st[best, cv2.CC_STAT_HEIGHT])
        u = float(cen[best][0])
        d = dep[lab == best] * sc
        d = d[d > 0]
        z = float(np.percentile(d, 30)) if d.size >= 10 else 200.0
        below_px = args.above_mm * fxv / max(z, 60.0)
        return u, bottom + below_px, z, int(st[best, cv2.CC_STAT_AREA]), False

    def red_nearby(uu, vv, radius=55, min_frac=0.08):
        """**물기 직전 마지막 확인** — 겨눈 자리 둘레가 정말 빨간가.

        ⚠ 색·모양만으로 고른 표적이 오래 추적하는 동안 화분틀 지지대 같은
          고정 구조물로 새어나갈 수 있다 — 2026-09-03 실측: 화소오차가
          330→15까지 아주 매끄럽게 줄어(추적 자체는 멀쩡해 보였다) "물었다"고
          했는데, 바깥 카메라로 보니 열매 둘 다 자리·방향이 그대로였고
          팔은 화분 받침 높이(z≈118mm)까지 내려가 있었다 — 지지대를 문
          것이다. **이동은 문제가 없었다. 처음부터·도중에 겨눈 자리가 틀렸을
          뿐이다.** 그래서 이동이 아니라 "닫기 직전"에 마지막으로 색을 본다 —
          거기 빨간 게 없으면 안 닫는다.
        """
        import cv2
        bgr = cv2.imread(vs.COLOR)
        if bgr is None:
            return True, 1.0               # 못 읽으면 판단 불가 — 기존 동작 유지
        h, w = bgr.shape[:2]
        y0, y1 = max(0, int(vv) - radius), min(h, int(vv) + radius)
        x0, x1 = max(0, int(uu) - radius), min(w, int(uu) + radius)
        if y1 <= y0 or x1 <= x0:
            return True, 1.0
        hsv = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        m = ((hsv[:, :, 1] > 80) & (hsv[:, :, 2] > 50)
             & ((hsv[:, :, 0] < 14) | (hsv[:, :, 0] > 165)))
        frac = float(m.mean())
        return frac >= min_frac, frac

    def clamp(d, cur):
        """한계를 **거부하지 말고 잘라라** — 막힌 축 하나 때문에 멈추면
        나머지 자유로운 축까지 얼어붙는다(2026-09-02: 14걸음 내리 0mm).
        진짜 물리 범위(arm_calib.Calib.joint_range — legal()과 같은 기준)로
        각 관절을 자른 뒤, 바닥만 통째로 확인한다."""
        cut = {}
        out = dict(d)
        for j, v in d.items():
            lim = calib.joint_range(j)
            if lim is None:
                continue
            lo, hi = lim
            m = (hi - lo) * arm_calib.LIMIT_MARGIN
            nv = max(lo + m, min(hi - m, v))
            if abs(nv - v) > 1e-9:
                cut[j] = nv - v
            out[j] = nv
        floor = min(-MOUNT_Z_MM + FLOOR_MARGIN_MM, kin.forward(cur, geom).z - 1.0)
        for k in (1.0, 0.6, 0.3):
            trial = {j: cur[j] + (out[j] - cur[j]) * k for j in kin.JOINTS}
            if kin.forward(trial, geom).z >= floor:
                return trial, cut
        return cur, None

    def move_base(cur, want3):
        """base 좌표로 want3(mm)만큼 — 집게 방향은 그대로. (관절변화, 잔차, 최대각)"""
        b0 = kin.forward(cur, geom)
        cols = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
        rows = []
        for j in cols:
            e = dict(cur)
            e[j] += 0.5
            p1 = kin.forward(e, geom)
            rows.append([(p1.x - b0.x) / 0.5, (p1.y - b0.y) / 0.5, (p1.z - b0.z) / 0.5])
        A = np.vstack([np.array(rows).T, np.array([[0.0, 1.0, 1.0, 1.0]])])
        rhs = np.append(np.asarray(want3, float), 0.0)
        sol, *_ = np.linalg.lstsq(A, rhs, rcond=None)
        return (dict(zip(cols, sol)), float(np.linalg.norm(A @ sol - rhs)),
                float(np.max(np.abs(sol))))

    def step_keep(cur, mm):
        """접근축을 따라 mm — **집게 방향은 그대로 두고**.

        ⚠ 4관절 최소노름 병진은 위치만 맞추고 자세를 놔둔다. 그 자유도가
          실제로 쓰이면 카메라가 확 돈다 — 2026-09-02에 12mm 나아가랬는데
          손목 카메라가 천장을 봤고, 겨눈 자리의 깊이가 123→309mm로 뛰어
          "닿았다"는 오판까지 났다. 그래서 **피치(lift+elbow+wrist)를 고정**
          하는 식을 한 줄 더 세운다. 미지수 4, 식 4 — 답이 하나로 정해진다.
        """
        b0 = kin.forward(cur, geom)
        cols = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
        rows = []
        for j in cols:
            e = dict(cur)
            e[j] += 0.5
            p1 = kin.forward(e, geom)
            rows.append([(p1.x - b0.x) / 0.5, (p1.y - b0.y) / 0.5, (p1.z - b0.z) / 0.5])
        A = np.vstack([np.array(rows).T, np.array([[0.0, 1.0, 1.0, 1.0]])])
        want = np.append(gp.approach_dir(cur, geom) * mm, 0.0)
        sol, *_ = np.linalg.lstsq(A, want, rcond=None)
        res = float(np.linalg.norm(A @ sol - want))
        return dict(zip(cols, sol)), res, float(np.max(np.abs(sol)))

    def ray_depth(uu, vv, half=9):
        """겨눈 자리 **바로 그 광선**의 거리 — 무엇이든 거기 있는 것까지.

        정렬이 끝나면 열매는 화면 아래로 빠져 안 보인다(줄기를 집게 자리에
        맞추면 열매는 그 아래에 있으니 당연하다). 2026-09-02에 그 때문에
        마지막 걸음에서 엉뚱한 빨간 것으로 갈아타고 "닿았다"고 했다.
        그러니 **덩이를 쫓지 말고 그 자리를 재라** — 겨냥이 맞았다면 그
        화소에 있는 것이 줄기다. 유효 깊이가 없으면 70mm보다 가깝다는 뜻이고,
        그것도 정보다(D405는 그보다 가까운 것을 통째로 비운다).
        """
        import cv2
        try:
            dep = np.load(vs.DEPTH).astype(float)
            meta = json.load(open(vs.META))
        except Exception:                              # noqa: BLE001
            return None
        sc = float(meta.get("depth_scale_mm", 1.0))
        h, w = dep.shape[:2]
        y0, y1 = max(0, int(vv) - half), min(h, int(vv) + half + 1)
        x0, x1 = max(0, int(uu) - half), min(w, int(uu) + half + 1)
        if y1 <= y0 or x1 <= x0:
            return None
        d = dep[y0:y1, x0:x1].reshape(-1) * sc
        d = d[d > 0]
        return None if d.size < 12 else float(np.percentile(d, 25))

    def ray_depth_med(uu, vv, n=3):
        """⚠ 한 장으로 판단하지 않는다 — 줄기는 가늘어서 한 프레임이 배경을
           집으면 63mm가 355mm로 읽힌다(2026-09-02: 82→355→66→194→67→63→193).
           그 튐 하나가 "닿았다"도 "놓쳤다"도 만들어 냈다. 중앙값을 쓴다."""
        vals = []
        for _ in range(max(1, n)):
            r = ray_depth(uu, vv)
            if r is not None:
                vals.append(r)
            time.sleep(0.12)
        return float(np.median(vals)) if vals else None

    markz = [0.0]
    tpl = [None]                 # (회색조 조각, 반크기)

    def track_mark(prev, R=70):
        """**찍어 준 점을 그림으로 따라간다** — 조각 정합(다중 배율).

        ⚠ 처음엔 "직전 자리 둘레에서 비슷한 깊이의 무게중심"으로 따라갔는데,
          줄기가 가늘고 화분 틀이 같은 깊이라 **덩이가 줄기를 따라 미끄러졌다.**
          그러면 야코비안의 부호가 걸음마다 뒤집혀 겨냥이 진동한다 —
          2026-09-03: 152 → 95 → 143 → 101px 를 오갔다.
          그림 조각은 그런 미끄러짐이 없다. 다가갈수록 커지므로 **배율을
          셋 놓고** 맞춰 보고, 잘 맞은 판(0.8 이상)으로 조각을 갱신한다.
        ⚠ 깊이는 정합된 자리에서 **따로** 읽는다 — 추적에 깊이를 안 쓴다.
        """
        import cv2
        bgr = cv2.imread(vs.COLOR)
        if bgr is None:
            return None
        g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        h, w = g.shape[:2]
        uu, vv = int(prev[0]), int(prev[1])
        if tpl[0] is None:
            k = 26
            y0, y1 = max(0, vv - k), min(h, vv + k)
            x0, x1 = max(0, uu - k), min(w, uu + k)
            if y1 - y0 < 12 or x1 - x0 < 12:
                return None
            tpl[0] = g[y0:y1, x0:x1].copy()
        t = tpl[0]
        th, tw = t.shape[:2]
        y0, y1 = max(0, vv - R - th // 2), min(h, vv + R + th // 2)
        x0, x1 = max(0, uu - R - tw // 2), min(w, uu + R + tw // 2)
        win = g[y0:y1, x0:x1]
        best = None
        for sc in (0.82, 1.0, 1.22):
            tt = t if abs(sc - 1.0) < 1e-6 else cv2.resize(t, None, fx=sc, fy=sc)
            if win.shape[0] < tt.shape[0] or win.shape[1] < tt.shape[1]:
                continue
            r = cv2.matchTemplate(win, tt, cv2.TM_CCOEFF_NORMED)
            _mn, mx, _ml, ml = cv2.minMaxLoc(r)
            if best is None or mx > best[0]:
                best = (float(mx), ml[0] + tt.shape[1] / 2.0 + x0,
                        ml[1] + tt.shape[0] / 2.0 + y0, tt.shape[1])
        if best is None or best[0] < 0.60:
            # ⚠ 0.45는 너무 헐거웠다 — 2026-09-03 실측: 애매한 매칭(0.45~0.6)을
            #   그대로 받아 엉뚱한 자리(배경·화분틀)를 "확신"하며 쫓아갔고,
            #   그 뒤로 걸음마다 깊이가 64→3405→64mm로 널뛰면서도 멈추지
            #   않았다. 애매하면 **놓친 것으로 치고 안전하게 실패**하는 편이
            #   허공에서 계속 겨누는 것보다 낫다 — 그러면 재시도 경로
            #   (반경을 넓혀 다시 찾기)로 간다.
            return None
        score, u2, v2, tw2 = best
        if score > 0.80:                       # 잘 맞았을 때만 조각을 새로 뜬다
            k = max(14, int(tw2 / 2))
            yy0, yy1 = max(0, int(v2) - k), min(h, int(v2) + k)
            xx0, xx1 = max(0, int(u2) - k), min(w, int(u2) + k)
            if yy1 - yy0 > 12 and xx1 - xx0 > 12:
                tpl[0] = g[yy0:yy1, xx0:xx1].copy()
        z2 = ray_depth(u2, v2)
        if z2 is None:
            z2 = markz[0] if markz[0] > 0 else -1.0
        markz[0] = z2
        return u2, v2, z2, int(score * 1000), False

    def see(prev):
        """(u, v, z, 넓이) — 겨눌 것.

        ⚠ **줄기는 초록이 아니다** — 은색 테이프로 감겨 있다(2026-09-02 바깥
          사진에서 확인). 그래서 초록 검출은 언제나 화분 틀의 세로대를 잡았고
          "줄기 181mm"라고 말하면서 엉뚱한 데를 겨눴다.

        `top` 모드는 색을 아예 안 쓴다. **줄기는 늘 열매 윗변 바로 위에 있고
        열매와 같은 깊이다** — 그 두 가지면 자리가 정해진다. 열매는 크고 빨개서
        놓칠 일이 없으니, 못 믿을 검출을 못 믿을 검출로 받치지 않는다.
        """
        if args.aim in ("mark", "click"):
            return track_mark(prev if prev is not None else mark0)
        if args.aim == "top":
            fb = fruit_box(prev)
            if fb is None:
                return tape_point(prev)          # 열매가 없다 — 노란 테이프로
            a, fu, ftop, fw, fz, clipped = fb
            try:
                fx = json.load(open(vs.META))["intrinsics"]["fx"]
            except Exception:                          # noqa: BLE001
                fx = 438.0
            up = args.above_mm * fx / max(fz, 60.0)    # mm → 화소
            return fu, ftop - up, fz, a, clipped
        if args.aim == "stem":
            r = vs.look_stem(prev=prev, near=400.0)
            if r is not None:
                return r[1], r[2], r[3], r[0], False
        f = vs.look(prev=prev)
        if f is None:
            return None
        return f[1], f[2], f[3], f[0], False

    mark0 = None
    if args.mark:
        mark0 = tuple(float(x) for x in args.mark.split(","))
    click_stamp = [None]

    def read_click():
        """클릭 페이지가 남긴 표적 — (u, v, 언제). 없으면 None.

        ⚠ 매 걸음 다시 읽는다. 도중에 사람이 다시 찍으면 **그 순간 표적이
          바뀐다** — 서보가 엉뚱한 빨간 것을 물고 갈 때 사람이 손으로 끊을
          유일한 방법이다(2026-09-03: 조끼와 책상 열매로 두 번 새 나갔다)."""
        try:
            t = json.load(open(args.click_file))
            return float(t["u"]), float(t["v"]), str(t.get("when", ""))
        except Exception:                                  # noqa: BLE001
            return None

    if args.aim == "click":
        c = read_click()
        if c is None:
            print("아직 찍힌 점이 없다 — 브라우저에서 http://<젯슨IP>:8090/ 을 열고")
            print("손목 화면의 물 자리를 한 번 눌러라. 그 다음 다시 실행한다.")
            return 1
        mark0 = (c[0], c[1])
        click_stamp[0] = c[2]
        print("클릭한 자리 화면 (%.0f, %.0f)  [%s]" % (c[0], c[1], c[2]))
    if args.aim == "auto":
        # ⚠ **"top"을 걸음마다 다시 쓰지 않는다.** top은 매 프레임 색으로
        #   다시 찾는데, 겨냥 걸음이 크면(2~3도씩) 하이라이트·자세 변화로
        #   한두 걸음 만에 놓친다(2026-09-03 실측: step 2~3에서 놓침).
        #   그래서 **처음 한 번만** 색으로 찾고, 그 다음부터는 이미 사람이
        #   찍은 것과 똑같이 **그림 조각을 따라간다**(track_mark) — 사람이
        #   클릭하는 대신 이 첫 검출이 클릭을 대신할 뿐이다.
        fb = fruit_box(None)
        if fb is not None:
            a, fu, ftop, _fw, fz, _clipped = fb
            try:
                fx = json.load(open(vs.META))["intrinsics"]["fx"]
            except Exception:                              # noqa: BLE001
                fx = 438.0
            up = args.above_mm * fx / max(fz, 60.0)
            # ⚠ 아주 가까우면(열매가 화면을 거의 채우면) 계산한 줄기 자리가
            #   화면 밖(음수)으로 나간다 — mm→화소 환산이 거리에 반비례해서
            #   커지기 때문이다. 화면 안으로 잘라야 track_mark가 조각을 뜰 수
            #   있다(2026-09-03: 굽힌 자세에서 깊이 87mm일 때 v=-42가 나왔다).
            mark0 = (min(846.0, max(1.0, fu)), min(478.0, max(2.0, ftop - up)))
            print("스스로 찾았다 — 열매 화면(%.0f,%.0f) 깊이%.0fmm 넓이%d → 줄기(%.0f,%.0f)"
                  % (fu, ftop, fz, a, mark0[0], mark0[1]))
        else:
            tp = tape_point(None)
            if tp is None:
                print("아무것도 못 찾았다 — 열매도 노란 테이프도 안 보인다.")
                return 1
            mark0 = (tp[0], tp[1])
            print("열매를 못 찾아 노란 테이프 아래로 겨눈다 — 화면(%.0f,%.0f) 깊이%.0fmm"
                  % (mark0[0], mark0[1], tp[2]))
        args.aim = "mark"          # 이후는 이미 검증된 추적을 그대로 쓴다
    if args.aim == "mark" and mark0 is None:
        print("--aim mark 에는 --mark u,v 가 필요하다")
        return 1

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    grip = [GRIP_OPEN]

    def go(d, secs=0.9, tries=3, tol=0.6, cap=5.0):
        """**도착할 때까지 밀어 넣는다** — 중력 처짐을 적분으로 갚는다.

        ⚠ 이 팔은 명령한 자리보다 2~3° 아래에 선다(실측 2026-09-02: 어깨
          -2.5°, 팔꿈치 -1.8°, 손목 -1.0°). 걸음이 그보다 작으면 **순효과가
          음수**가 되어 "올려라"가 내려가는 결과가 된다 — 70mm 올리랬는데
          15mm 내려갔다. 서보의 겨냥 걸음도 보통 1~3°라 정확히 이 함정에
          걸린다. 그래서 모자란 만큼을 명령에 더해 다시 보낸다."""
        cmd = dict(d)
        got = to_deg(io.read())
        worst = max(abs(got[j] - d[j]) for j in kin.JOINTS)
        for _ in range(max(1, tries)):
            n = to_norm(cmd)
            n["gripper"] = grip[0]
            io.write(n, secs)
            time.sleep(SETTLE)
            got = to_deg(io.read())
            miss = {j: d[j] - got[j] for j in kin.JOINTS}
            worst = max(abs(v) for v in miss.values())
            if worst <= tol:
                break
            for j in kin.JOINTS:
                cmd[j] = max(min(cmd[j] + miss[j], d[j] + cap), d[j] - cap)
            if not legal(cmd, got)[0]:
                break
        # ⚠ 조작대 3D 미리보기(click_server.py의 k3dParseLog)가 이 형식을
        #   찾아 "지금"을 실시간으로 갱신한다 — arm_stage.py의 show()와 같은
        #   문자열이어야 한다. 그게 없으면 잡기가 도는 동안 미리보기가
        #   멈춰 있다(2026-09-03).
        print("  자세  " + " ".join("%s=%7.1f" % (j.split("_")[0], got[j]) for j in kin.JOINTS))
        return got, worst

    try:
        cur = to_deg(io.read())
        go(cur, 0.6)
        s = see(None)
        if s is None:
            print("❌ 겨눌 것을 못 찾았다 — 열매가 보이는 자세에서 시작하라.")
            return 1
        u, v, z, a, _clip = s
        print("지금: %s 화면(%.0f,%.0f) 깊이 %.0fmm 넓이%d  → 남은 화소 %.0f"
              % (args.aim, u, v, z, a, math.hypot(u - tu, v - tv)))
        if args.dry:
            # ⚠ **여기서만 깊이를 믿는다.** 실제로 무는 동안엔 깊이를 목표로
            #   안 쓴다(이 파일 머리말 — 조기접촉에 오염된다). 하지만 여긴
            #   팔을 안 움직이니 잘못돼도 미리보기 한 줄이 틀릴 뿐이다.
            #   지금 찾은 줄기 화면자리(u,v)의 깊이 + cam_frame.py가 **지금
            #   자세 근처에서** 잰 국소 회전(R)으로, "줄기를 집게 무는 자리로
            #   데려가려면 base로 얼마나"를 낸다(S=줄기, G=집게 자리 — 아직
            #   안 맞춰져 있으면 S와 G가 화면에서도 떨어져 있으니 그 차이가
            #   곧 겨냥 보정+전진을 한꺼번에 담는다). move_base(순수 기구학,
            #   물기 전혀 안 함)로 그 변위에 해당하는 관절값을 계산해 찍는다.
            #   cam_frame.py를 지금 자세와 먼 데서 돌렸으면 이 미리보기도
            #   같이 틀어진다 — R은 그 자세 근처에서만 맞다(파일 머리말).
            try:
                cf = json.load(open(os.path.expanduser("~/cam_frame.json")))
                R = np.array(cf["R_cam_from_base"])
                meta = json.load(open(vs.META))
                intr = Intrinsics.from_dict(meta["intrinsics"])
                S = np.array(intr.deproject(u, v, z))
                try:
                    gg = json.load(open(os.path.expanduser("~/grip_uv.json")))
                    gu, gv = float(gg["u"]), float(gg["v"])
                except Exception:                            # noqa: BLE001
                    gu, gv = 471.0, 395.0
                G = np.array(intr.deproject(gu, gv, 80.0))
                move = R.T @ (S - G)
                dd, _res, _big = move_base(cur, move)
                tgt = dict(cur)
                for j, w in dd.items():
                    tgt[j] += float(w)
                print("  목표  " + " ".join("%s=%7.1f" % (j.split("_")[0], tgt[j])
                                            for j in kin.JOINTS))
                print("  base로 %.0fmm 옮기면 그 자리 (cam_frame.json은 %s 근처에서 잰 값)"
                      % (float(np.linalg.norm(move)), cf.get("when", "?")))
            except Exception as e:                           # noqa: BLE001
                print("  목표 미리보기 계산 실패 (%s) — cam_frame.py를 지금 자세 "
                      "근처에서 먼저 돌렸는지 확인" % e)
            return 0

        def see2(prev):
            """놓치면 **기억을 버리고** 다시 본다 — 한 걸음이 크면 열매가
            추적 반경 밖으로 튀는데, 그때 포기하면 서보가 첫 걸음에 끝난다."""
            r = see(prev)
            if r is not None:
                return r
            if args.aim in ("mark", "click"):
                # 못 찾으면 **창을 넓혀** 한 번 더 — 걸음이 예상보다 컸을 뿐인데
                # 표적을 버리면 서보가 두 걸음 만에 끝난다(2026-09-02).
                for rr in (140, 220):
                    r = track_mark(prev, rr)
                    if r is not None:
                        return r
                return None
            if prev is None:
                return None
            # ⚠ **한 프레임 실패로 포기하지 않는다.** D405 컬러는 이따금 찢어진
            #   JPEG를 낸다(이 저장소가 라인 검출에서 이미 겪은 병) — 그러면
            #   실제로 열매가 그대로 있어도 그 한 장만 못 찾는다. 새 프레임을
            #   몇 번 다시 읽어 본 뒤에만 진짜로 놓친 것으로 친다.
            for _ in range(3):
                time.sleep(0.25)
                r = see(None)
                if r is not None:
                    return r
            return None

        gone, JI, prev, locked, stall, miss_streak = 0.0, None, (u, v), False, 0, 0
        best_dir = [None, None]      # 실측으로 고른 전진 방향과 그 효과
        print("\n 걸음   겨냥       화소   깊이    나아감   한 일")
        for step in range(args.steps):
            if args.aim == "click":
                c = read_click()
                if c is not None and c[2] != click_stamp[0]:
                    click_stamp[0] = c[2]
                    prev, locked, markz[0], tpl[0] = (c[0], c[1]), False, 0.0, None
                    print("  ↻ 사람이 다시 찍었다 → (%.0f, %.0f)" % (c[0], c[1]))
            s = see2(prev)
            if s is None:
                print("  %3d    (놓쳤다)" % step)
                break
            u, v, z, a, clipped = s
            prev = (u, v)
            err = np.array([tu - u, tv - v])
            en = float(np.linalg.norm(err))
            if en <= args.tol:
                locked, miss_streak = True, 0
            elif locked:
                miss_streak += 1
            if locked:
                zr = ray_depth_med(tu, tv)
                if zr is not None:
                    z = zr
            print("  %3d  (%4.0f,%4.0f) %5.0f  %5.0f  %6.1f  " % (step, u, v, en, z, gone),
                  end="")

            cur = to_deg(io.read())

            # ⚠ **한 번 놓친 프레임에 잘 가던 접근을 통째로 버리지 않는다.**
            #   2026-09-03: 깊이 351→136mm까지 순조롭게 좁혀 놓고, 딱 한 프레임
            #   추적이 튀자(en 8→191) 그 자리에서 겨냥으로 되돌아갔고, 그 뒤로
            #   회복을 못 해 실패했다. 깊이는 추적(u,v)과 무관하게 겨눈 자리
            #   (tu,tv)에서 **따로** 재므로(위의 ray_depth_med) 이미 믿을 만하다
            #   — 그러니 튄 게 2연속을 넘기 전까지는 겨냥으로 돌아가지 않고
            #   그 깊이를 믿고 계속 나아간다.
            trust_advance = locked and miss_streak <= 2

            # ── 1. 겨냥이 틀어졌으면 먼저 화면을 맞춘다 (깊이는 안 쓴다) ──
            if trust_advance and en > args.tol:
                print("(놓침 %d/2 — 깊이를 믿고 계속 나아간다) " % miss_streak, end="")
            elif clipped and en > args.tol:
                # 윗변이 화면 밖이면 겨냥점을 알 수 없다 — 이미 맞춰 놓았으니
                # 여기서 다시 겨누면 없는 오차를 쫓는다. 나아가기만 한다.
                print("(윗변 잘림 — 겨냥 보류) ", end="")
            elif en > args.tol:
                if JI is None or step % args.rejacobian == 0:
                    cols, J = [], []
                    for j in AIM:
                        p = dict(cur)
                        p[j] += args.probe
                        ok, why = legal(p, cur)
                        if not ok:
                            p[j] = cur[j] - args.probe
                            ok, why = legal(p, cur)
                            if not ok:
                                continue
                        d0 = p[j] - cur[j]
                        go(p, 0.7)
                        r = see2(prev)
                        go(dict(cur), 0.7)
                        if r is None:
                            continue
                        J.append(np.array([r[0] - u, r[1] - v]) / d0)
                        cols.append(j)
                    if len(cols) < 1:
                        print("⚠ 야코비안을 못 쟀다")
                        break
                    JI = (np.array(J).T, cols)
                A, cols = JI
                sol, *_ = np.linalg.lstsq(A, err * args.gain, rcond=None)
                sol = np.clip(sol, -args.max_turn, args.max_turn)
                nxt = dict(cur)
                for j, w in zip(cols, sol):
                    nxt[j] += float(w)
                nxt, cut = clamp(nxt, cur)
                if cut is None:
                    print("겨냥 못함(바닥)")
                    continue
                go(nxt, 0.9)
                # ⚠ 추적창을 **예측 자리로 미리 옮긴다.** 야코비안이 이미 "이
                #   걸음이면 화면이 몇 px 움직인다"를 말해 준다 — 그걸 안 쓰고
                #   제자리에서 찾으면 큰 걸음마다 표적을 잃는다.
                pred = A @ sol
                prev = (float(u + pred[0]), float(v + pred[1]))
                print("겨냥 " + " ".join("%s%+.1f" % (j.split("_")[0], w)
                                        for j, w in zip(cols, sol))
                      + "  (다음 %0.f,%0.f 예상)" % prev)
                continue

            # ── 2. 겨냥이 됐다 — 나아간다 ──
            # ⚠ **집게가 무는 거리를 알면 접촉을 기다릴 이유가 없다.**
            #   손끝은 카메라에서 75~80mm 앞이다(집게가 찍힌 화면에서 실측:
            #   무는 자리 80mm, 손가락 끝 66mm). 겨눈 광선이 그 거리를 읽으면
            #   이미 줄기가 손가락 사이에 있는 것이다. 2026-09-02에 첫 걸음의
            #   깊이가 이미 82mm였는데도 "닿을 때까지" 더 밀어서 열매를
            #   떨어뜨렸다 — 두 번.
            if args.stop_z and locked and z <= args.stop_z:
                print("줄기가 %.0fmm — 무는 거리(%.0fmm)에 들어왔다. 총 %.0fmm 나아갔다"
                      % (z, args.stop_z, gone))
                ok_red, frac = red_nearby(tu, tv)
                if not ok_red:
                    print("⚠ 물기 직전 확인 — 그 자리 둘레가 빨갛지 않다(빨간 비율 %.0f%%,"
                          " 열매가 아닌 것 같다). **안 닫는다.**" % (frac * 100))
                    return 1
                if not args.no_close:
                    n = to_norm(to_deg(io.read()))
                    n["gripper"] = GRIP_SHUT
                    io.write(n, 1.2)
                    time.sleep(1.2)
                    print("🍅 집게를 닫았다.")
                json.dump({"cam_to_tip_mm": float(z), "tip_uv": [tu, tv],
                           "aim": args.aim, "how": "stop_z",
                           "when": time.strftime("%Y-%m-%d %H:%M:%S")},
                          open(os.path.expanduser("~/tip_depth.json"), "w"), indent=1)
                return 0
            if gone >= args.max_adv:
                print("⚠ %.0fmm까지 갔다 — 더 안 간다" % gone)
                break
            want = min(args.adv, args.max_adv - gone)
            if args.stop_z and locked and z > args.stop_z:
                # 남은 거리보다 큰 걸음은 **줄기를 밀어낸다**. 딱 그만큼만 간다.
                want = min(want, max(2.0, z - args.stop_z))
            # ⚠ 전진에서는 **자르면 안 된다.** 병진은 세 관절의 합으로만
            #   병진이다 — 한계에 걸린 wrist_flex만 잘라내면 남은 둘이 그대로
            #   돌아 **회전**이 되어 버린다. 2026-09-02: 16mm 나아가랬더니
            #   카메라가 천장을 봤다. 그래서 자르는 대신 **걸음을 줄인다.**
            # ⚠ 세 관절로만 병진하면 팔꿈치·손목이 먼저 한계에 닿는다 —
            #   pan을 넣으면 같은 병진을 네 관절이 나눠 져서 여유가 생긴다.
            # 세 갈래를 차례로 — 자세를 지키는 병진이 가장 낫지만, 팔이 한계
            # 근처면 그것만 고집하다 한 발도 못 뗀다(2026-09-03). 피치가 조금
            # 도는 건 다음 걸음에서 겨냥으로 갚으면 된다.
            nxt, why = None, "한계"
            for mode in ("keep", "free4", "free3"):
                for k in (1.0, 0.6, 0.3):
                    if mode == "keep":
                        dd, res, big = step_keep(cur, want * k)
                        if res > 0.06 * max(want * k, 1.0) or big > 8.0:
                            why = "자세를 지키며 갈 수 없다"
                            continue
                    else:
                        js = (("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
                              if mode == "free4"
                              else ("shoulder_lift", "elbow_flex", "wrist_flex"))
                        dd, res = gp.step_along(cur, geom, want * k, joints=js)
                        if res > 0.06 * max(want * k, 1.0):
                            why = "병진을 못 만든다"
                            continue
                    cand = dict(cur)
                    for j, w in dd.items():
                        cand[j] += float(w)
                    ok, why = legal(cand, cur)
                    if ok:
                        nxt, want = cand, want * k
                        break
                if nxt is not None:
                    break
            if nxt is None and best_dir[0] is None:
                # ⚠ **접근축이라는 가정을 버린다.** 카메라 광축과 집게 축이
                #   다르고, 한계에 걸리면 그 축으론 한 발도 못 뗀다. 그러면
                #   "어느 쪽으로 가야 겨눈 자리의 깊이가 주는가"를 **직접
                #   재서** 정한다 — 모델이 필요 없다(2026-09-03).
                a = gp.approach_dir(cur, geom)
                h = a * np.array([1.0, 1.0, 0.0])
                h = h / max(float(np.linalg.norm(h)), 1e-6)
                cand = {"접근": a, "수평": h, "위": np.array([0.0, 0.0, 1.0]),
                        "아래": np.array([0.0, 0.0, -1.0]),
                        "수평뒤": -h, "옆": np.array([-h[1], h[0], 0.0])}
                print("전진이 막혔다 — 방향을 재 본다")
                z0 = ray_depth_med(tu, tv, 3)
                for nm, e in cand.items():
                    dd, res, big = move_base(cur, e * 12.0)
                    if res > 1.0 or big > 9.0:
                        continue
                    trial = dict(cur)
                    for j, w in dd.items():
                        trial[j] += float(w)
                    if not legal(trial, cur)[0]:
                        continue
                    go(trial, 0.9)
                    z1 = ray_depth_med(tu, tv, 3)
                    go(dict(cur), 0.9)
                    if z0 is None or z1 is None:
                        continue
                    print("   %-6s 12mm → 깊이 %.0f → %.0f (%+.0f)" % (nm, z0, z1, z1 - z0))
                    if best_dir[1] is None or (z1 - z0) < best_dir[1]:
                        best_dir[0], best_dir[1] = e, z1 - z0
                if best_dir[0] is None or best_dir[1] > -2.0:
                    print("어느 쪽으로도 가까워지지 않는다 — 멈춘다")
                    break
                print("   → 그쪽으로 간다 (12mm에 %.0fmm 준다)" % (-best_dir[1]))
            if nxt is None and best_dir[0] is not None:
                for k in (1.0, 0.6, 0.3):
                    dd, res, big = move_base(cur, best_dir[0] * (want * k))
                    if res > 1.0 or big > 9.0:
                        continue
                    cand2 = dict(cur)
                    for j, w in dd.items():
                        cand2[j] += float(w)
                    if legal(cand2, cur)[0]:
                        nxt, want = cand2, want * k
                        break
            if nxt is None:
                print("나아갈 수 없다(%s)" % why)
                break
            got, terr = go(nxt, 1.0)
            time.sleep(0.35)
            z2 = ray_depth_med(tu, tv) if locked else None
            if z2 is None:
                r = see2(prev)
                if r is None:
                    if locked:
                        print("나아감 %.0fmm → 겨눈 자리에 깊이가 없다 "
                              "(70mm보다 가깝다) — **닿았다고 본다**" % want)
                        z2 = 0.0
                    else:
                        print("나아감 %.0fmm — 그리고 놓쳤다" % want)
                        break
                else:
                    z2 = r[2]
            dz = z - z2
            gone += want
            print("나아감 %.0fmm → 깊이 %.0f→%.0f (%+.0f)" % (want, z, z2, -dz))
            # ⚠ 가는 줄기는 판독이 ±30mm 튄다(2026-09-03: 130→142, 122→152).
            #   그걸 곧바로 "놓쳤다"로 읽으면 걸음마다 겨냥으로 되돌아가
            #   20mm 가는 데 스무 걸음을 쓴다. **추세**만 보고, 정말로 크게
            #   어긋날 때만 되돌린다.
            if dz > 3.0 * want or z2 > z + 2.5 * want:
                # 걸음보다 크게 줄거나 **오히려 멀어졌으면** 겨눈 광선이 대상을
                # 놓친 것이다. 그것을 접촉으로 읽으면 허공에서 집게가 닫힌다.
                print("   ⚠ 깊이가 걸음과 안 맞는다 — 대상을 놓쳤다, 다시 겨눈다")
                locked = False
                continue
            # ── 3. 걸음만큼 안 줄면 닿은 것이다 ──
            # ⚠ **깊이가 늘어난 것은 접촉이 아니다.** 가는 줄기라 한 프레임이
            #   배경을 물면 +12mm씩 튀는데, dz<0.35*want 만 보면 그 튐이 곧
            #   "닿았다"가 된다 — 2026-09-03에 40mm를 남기고 허공에서 닫았다.
            #   닿았다면 깊이는 **줄기를 멈춘 채로** 있지, 멀어지지 않는다.
            if 0.0 <= dz < 0.35 * want:
                stall += 1
            else:
                stall = 0
            if stall >= 2:
                print("\n🍅 **닿았다** — %.0fmm를 명령했는데 깊이는 %.0fmm밖에 안 줄었다."
                      % (want, dz))
                print("   ⇒ 카메라에서 손끝까지 = **%.0f mm** (이번 실측)" % r[2])
                ok_red, frac = red_nearby(tu, tv)
                if not ok_red:
                    print("⚠ 물기 직전 확인 — 그 자리 둘레가 빨갛지 않다(빨간 비율 %.0f%%,"
                          " 열매가 아닌 것 같다). **안 닫는다.**" % (frac * 100))
                    return 1
                if not args.no_close:
                    n = to_norm(to_deg(io.read()))
                    n["gripper"] = GRIP_SHUT
                    io.write(n, 1.2)
                    time.sleep(1.2)
                    print("   집게를 닫았다.")
                json.dump({"cam_to_tip_mm": float(z2),
                           "tip_uv": [tu, tv], "aim": args.aim,
                           "when": time.strftime("%Y-%m-%d %H:%M:%S")},
                          open(os.path.expanduser("~/tip_depth.json"), "w"), indent=1)
                return 0
        print("\n(다 썼다 — 안 닿았다)")
        return 1
    finally:
        io.hold_close()


if __name__ == "__main__":
    raise SystemExit(main())
