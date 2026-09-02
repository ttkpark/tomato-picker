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
import grasp_probe as gp                                   # noqa: E402
import visual_servo as vs                                  # noqa: E402

DEG_PER_TICK = 360.0 / 4096.0
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


def load_cart():
    cal = json.load(open(CAL))
    spans = {}
    for name, c in cal.items():
        try:
            spans[name] = abs(int(c["range_max"]) - int(c["range_min"])) * DEG_PER_TICK
        except (KeyError, TypeError, ValueError):
            pass
    return spans, json.load(open(CART))


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
    ap.add_argument("--aim", choices=("click", "mark", "top", "stem", "fruit"), default="top")
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

    spans, cart = load_cart()
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
        """⚠ 한계는 **지금 자세를 가두지 않게** 잡는다 — 정규화 범위는
           교시자세를 중심으로 대칭이 아니라서 98을 그냥 쓰면 못 움직인다."""
        nm, cm = to_norm(d), to_norm(cur)
        for j, v in nm.items():
            if abs(v) > max(98.0, abs(cm.get(j, 0.0))) + 1e-6:
                return False, "%s 한계" % j
        floor = min(-MOUNT_Z_MM + FLOOR_MARGIN_MM, kin.forward(cur, geom).z - 1.0)
        if kin.forward(d, geom).z < floor:
            return False, "바닥"
        return True, ""

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

    def clamp(d, cur):
        """한계를 **거부하지 말고 잘라라** — 막힌 축 하나 때문에 멈추면
        나머지 자유로운 축까지 얼어붙는다(2026-09-02: 14걸음 내리 0mm).
        정규화 한계로 각 관절을 자른 뒤, 바닥만 통째로 확인한다."""
        cm = to_norm(cur)
        nm = to_norm(d)
        cut = {}
        for j, v in nm.items():
            lim = max(98.0, abs(cm.get(j, 0.0)))
            nv = max(-lim, min(lim, v))
            if abs(nv - v) > 1e-9:
                cut[j] = nv - v
            nm[j] = nv
        out = to_deg(nm)
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
        if best is None or best[0] < 0.45:
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
                return None
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
        for _ in range(max(1, tries)):
            n = to_norm(cmd)
            n["gripper"] = grip[0]
            io.write(n, secs)
            time.sleep(SETTLE)
            got = to_deg(io.read())
            miss = {j: d[j] - got[j] for j in kin.JOINTS}
            worst = max(abs(v) for v in miss.values())
            if worst <= tol:
                return got, worst
            for j in kin.JOINTS:
                cmd[j] = max(min(cmd[j] + miss[j], d[j] + cap), d[j] - cap)
            if not legal(cmd, got)[0]:
                break
        return got, max(abs(got[j] - d[j]) for j in kin.JOINTS)

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
            return see(None) if prev is not None else None

        gone, JI, prev, locked, stall = 0.0, None, (u, v), False, 0
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
                locked = True
            if locked:
                zr = ray_depth_med(tu, tv)
                if zr is not None:
                    z = zr
            print("  %3d  (%4.0f,%4.0f) %5.0f  %5.0f  %6.1f  " % (step, u, v, en, z, gone),
                  end="")

            cur = to_deg(io.read())

            # ── 1. 겨냥이 틀어졌으면 먼저 화면을 맞춘다 (깊이는 안 쓴다) ──
            if clipped and en > args.tol:
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
