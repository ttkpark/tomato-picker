#!/usr/bin/env python3
"""**보면서 다가간다** — 열매를 집게 자리로 끌어오는 시각 서보잉.

    ~/lerobot/.venv/bin/python ros2/tools/visual_servo.py --to 200
    ~/lerobot/.venv/bin/python ros2/tools/visual_servo.py --to 120 --steps 8

⚠ `tomato-voice`는 내리고 `depth-cam`은 켠 채로. **사람이 보는 앞에서만.**

────────────────────────────────────────────────────────────────────────
왜 이 방식인가 — 보정이 틀려도 닿는다

손-눈 보정(`T_tool_cam`)의 잔차가 44mm에서 안 내려갔다. 링크 길이·영점·회전을
차례로 배제했고 원인은 아직 모른다. 그런데 **잡는 데는 그게 필요 없다.**

카메라와 집게는 같은 링크에 붙어 있다. 그래서 "집게가 무는 자리"를 **카메라
좌표로** 쓰면 상수 하나다 — 링크 길이도 영점도 손-눈도 식에 안 들어간다.
그러면 목표는 "열매를 base 좌표 어디로 옮긴다"가 아니라 **"화면에서 열매를
그 상수 자리로 끌어온다"**가 되고, 보정 오차가 양쪽에서 상쇄된다.

여기서 쓰는 것은 **실측 야코비안**뿐이다: 관절을 조금 흔들어 화면이 몇 px,
깊이가 몇 mm 움직이는지 그 자리에서 재고, 그 역으로 한 걸음 간다. 자세가
바뀌면 야코비안도 바뀌므로 **매 걸음 다시 잰다.** 모델을 안 믿는 대신 매번
물어보는 방식이다 — 이 저장소의 "게인을 만지기 전에 크기를 재라"와 같은 태도다.

⚠ 목표 화면좌표(`TARGET_UV`)는 **집게를 여닫아 실측**한 값이다. 손가락은 검은
   무광이라 깊이가 안 잡히지만, 여닫을 때 **변하는 화소**로는 또렷이 잡힌다.
   브래킷을 건드리면 이 값을 다시 재야 한다(`--probe-fingers`).
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
# 집게가 무는 자리. `grasp_probe.py`가 **벽을 눌러 실측한** 값이 있으면 그것을
# 쓰고, 없으면 여닫기 차이로 얻은 어림값을 쓴다.
#
# ⚠ 실측이 바로잡아 준 것 하나: 파지점은 카메라에서 **240mm 앞**이다. 그동안
#   나는 "가능한 한 가까이"를 목표로 열매를 100mm까지 끌고 가려 했고, 그
#   거리에서 열매가 화면보다 커져 매번 눈이 감겼다. 목표 거리를 알면 그
#   구간에 갈 이유가 없다 — 문제가 사라진다.
GRASP_FILE = os.path.expanduser("~/grasp_point.json")
TARGET_UV = (471.0, 395.0)
TARGET_Z = None


def _load_grasp():
    global TARGET_UV, TARGET_Z
    try:
        g = json.load(open(GRASP_FILE, encoding="utf-8"))["grasp_cam_mm"]
        k = json.load(open(META))["intrinsics"]
        TARGET_UV = (k["ppx"] + k["fx"] * g[0] / g[2],
                     k["ppy"] + k["fy"] * g[1] / g[2])
        TARGET_Z = float(g[2])
    except Exception:                                  # noqa: BLE001
        pass
# ⚠ 축을 셋만 쓰면 하나가 한계에 붙는 순간 **한 걸음도 못 간다.**
#   2026-09-02: elbow가 정규화 -97.8(한계 98)에서 막혔다. 목표는 셋
#   (u, v, 깊이)인데 축을 넷 주면 여유가 생겨 한계를 피해 간다.
AXES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
PROBE_DEG = 5.0
MAX_STEP_DEG = 10.0        # 한 걸음에 어느 관절도 이 이상 안 움직인다
NEAR_MM = 450.0            # 이보다 먼 것은 열매가 아니다(조끼·상자)
# 열매의 실제 지름(mm) — 2026-09-02 실측 74mm. 깊이를 못 믿을 때
# 화면에서의 크기로 거리를 대신 재는 데 쓴다.
FRUIT_MM = 74.0
SETTLE = 0.9


def look_stem(prev=None, radius=200.0, near=260.0):
    """**줄기**를 직접 본다 — 초록이고, 가깝고, 작다.

    ⚠ 열매를 보면 마지막 10cm에서 화면보다 커져 잘린다(74mm짜리가 100mm 앞에서
      324px). 잘린 덩이는 중심도 크기도 틀어져서 "닿았다"고 하고도 90mm가
      남았다 — 2026-09-02. 줄기는 가까이 가도 작으니 그 문제가 없고, 애초에
      잡아야 할 것도 줄기다.

    ⚠ 화분 틀도 초록이다. 가까운 것만 봐도 틀의 세로대가 같이 걸린다 —
      2026-09-02에 검출이 넓이 157~4868, 깊이 131~244mm를 오가며 계속 갈아탔다.
      그래서 **열매를 먼저 찾고, 그 옆에 붙은 초록만** 본다. 줄기는 열매에
      붙어 있다는 사실이 틀과 가르는 유일한 단서다.
    """
    bgr = cv2.imread(COLOR)
    if bgr is None:
        return None
    try:
        dep = np.load(DEPTH).astype(float)
        meta = json.load(open(META))
    except Exception:                              # noqa: BLE001
        return None
    intr = Intrinsics.from_dict(meta["intrinsics"])
    sc = float(meta.get("depth_scale_mm", 1.0))
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = ((hsv[:, :, 0] > 32) & (hsv[:, :, 0] < 92)
         & (hsv[:, :, 1] > 45) & (hsv[:, :, 2] > 40)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    fruit = look()                       # 열매가 어디 있는지 먼저 안다
    if fruit is None:
        return None
    fu, fv, fz = fruit[1], fruit[2], fruit[3]
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, n):
        a = int(st[i, cv2.CC_STAT_AREA])
        if a < 120:
            continue
        d = dep[lab == i] * sc
        d = d[d > 0]
        if d.size < 15:
            continue
        z = float(np.percentile(d, 25))
        if z > near:
            continue
        u, v = float(cen[i][0]), float(cen[i][1])
        # 열매에 **붙어 있어야** 줄기다 — 화면에서도 가깝고 깊이도 비슷해야 한다.
        if math.hypot(u - fu, v - fv) > 260.0 or abs(z - fz) > 60.0:
            continue
        if prev is not None and math.hypot(u - prev[0], v - prev[1]) > radius:
            continue
        score = -a if prev is None else -math.hypot(u - prev[0], v - prev[1])
        if best is None or score > best[0]:
            best = (score, a, u, v, z,
                    np.array(intr.deproject(u, v, z)))
    return None if best is None else best[1:]


def look(prev=None, radius=180.0, aim='center'):
    """빨간 열매 하나 — (넓이, u, v, 깊이, 카메라계 점).

    ⚠ `prev`(직전 화면좌표)를 주면 **그 근처의 것**을 고른다. 안 그러면 한 번
      놓친 순간 다른 빨간 것으로 갈아탄다 — 2026-09-02, 열매가 화면에서 밀린
      뒤 흰 화분 쪽 다른 것을 붙잡고 엉뚱한 데로 갔다. 조명이 바뀌면(방 불이
      꺼지고 작업등이 켜졌다) 색이 흔들려 더 잘 벌어진다.
      **한 번 잡은 것은 계속 그것으로 본다**가 원칙이다.
    """
    bgr = cv2.imread(COLOR)
    if bgr is None:
        return None
    try:
        dep = np.load(DEPTH).astype(float)
        meta = json.load(open(META))
    except Exception:                              # noqa: BLE001
        return None
    intr = Intrinsics.from_dict(meta["intrinsics"])
    sc = float(meta.get("depth_scale_mm", 1.0))
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # ⚠ 색만 보면 안 된다 — 선반의 주황 조끼가 2m 앞에서 15000화소로 잡혔다.
    #   **가까운 것만** 남긴다.
    m = ((hsv[:, :, 1] > 80) & (hsv[:, :, 2] > 50)
         & ((hsv[:, :, 0] < 14) | (hsv[:, :, 0] > 165))).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, n):
        a = int(st[i, cv2.CC_STAT_AREA])
        if a < 600:
            continue
        d = dep[lab == i] * sc
        d = d[d > 0]
        w = float(st[i, cv2.CC_STAT_WIDTH])
        # ⚠ 깊이는 두 가지로 배신한다.
        #   (가) 아주 가까우면(D405 최소 ~70mm) 통째로 비어 버린다 — 2026-09-02에
        #       85mm에서 야코비안을 못 재고 멈췄다.
        #   (나) 번들거리는 표면 + 밝은 조명이면 **틀린 값**을 준다. 같은 날
        #       200mm 앞의 열매가 1544mm로 읽혔고(마스크 가장자리의 배경이
        #       중앙값을 끌고 갔다) "멀다"며 버려졌다. **없는 것보다 나쁘다.**
        #   그래서 화면에서의 **크기**로 따로 재서 견준다. 열매 지름은 안다.
        z_size = intr.fx * FRUIT_MM / w if w > 4 else -1.0
        z_stereo = float(np.percentile(d, 20)) if d.size >= 25 else -1.0
        if z_stereo > 0 and z_size > 0 and abs(z_stereo - z_size) > 0.5 * z_size:
            z = z_size                      # 스테레오가 크기와 크게 어긋난다 — 크기를 믿는다
        elif z_stereo > 0:
            z = z_stereo
        else:
            z = z_size
        if z <= 0 or z > NEAR_MM:
            continue
        u, v = float(cen[i][0]), float(cen[i][1])
        # ⚠ **잡을 곳은 열매가 아니라 그 위의 줄기다.** 열매 중심을 집게에 맞추면
        #   줄기는 늘 그보다 위에 남는다 — 2026-09-02, 그래서 집게가 줄기를 지나쳤다.
        #   열매의 **윗변**(줄기가 붙는 자리)을 조준점으로 삼는다. 화면이 90° 돌아
        #   달려 있으므로 "위"는 세로가 아니라 **집게 자리에서 먼 쪽**이다.
        if aim == "stem":
            ys, xs = np.nonzero(lab == i)
            k = int(np.argmin(ys))              # 화면에서 가장 위쪽 화소
            u, v = float(xs[k]), float(ys[k]) + 12.0
        if prev is not None:
            d2 = math.hypot(u - prev[0], v - prev[1])
            if d2 > radius:
                continue
            score = -d2                      # 가까울수록 좋다
        else:
            score = float(a)                 # 처음엔 가장 큰 것
        pt = (np.array(intr.deproject(u, v, z)) if z > 0 else None)
        if best is None or score > best[0]:
            best = (score, a, u, v, z, pt)
    if best is None:
        return None
    return (best[1], best[2], best[3], best[4], best[5])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--to", type=float, default=-1.0,
                    help="이 깊이(mm)까지 다가간다. 비우면 실측한 파지점 거리")
    ap.add_argument("--steps", type=int, default=6)
    ap.add_argument("--gain", type=float, default=0.6)
    ap.add_argument("--max-step", type=float, default=MAX_STEP_DEG,
                    help="한 걸음에 어느 관절도 이 이상 안 움직인다")
    ap.add_argument("--aim", default="center", choices=("center", "stem", "green"),
                    help="center=열매 중심 · stem=열매 윗변(줄기가 붙는 자리)")
    ap.add_argument("--dry", action="store_true", help="야코비안만 재고 안 움직인다")
    args = ap.parse_args()
    _load_grasp()
    if args.to < 0 and TARGET_Z:
        args.to = TARGET_Z

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

    def legal(d, cur=None):
        """갈 수 있는 자세인가.

        ⚠ **가드가 지금 있는 자리를 가두면 안 된다.** 관절이 이미 한계를 조금
          넘어 있으면(중력으로 밀렸거나 앞 걸음이 끝까지 갔거나) "|norm| ≤ 98"을
          고집하는 순간 **어느 방향으로도 못 간다** — 2026-09-02, 네 축이 전부
          "양쪽 다 한계"로 나와 한 걸음도 못 갔다. 바닥 가드에서 이미 겪은 병이다.
          그래서 지금보다 **더 나빠지지만 않으면** 통과시킨다.
        """
        nm = to_norm(d)
        cm = to_norm(cur) if cur else {}
        for j, v in nm.items():
            lim = max(98.0, abs(cm.get(j, 0.0)))
            if abs(v) > lim + 1e-6:
                return False
        floor = -MOUNT_Z_MM + FLOOR_MARGIN_MM
        if cur:
            floor = min(floor, kin.forward(cur, geom).z - 1.0)
        return kin.forward(d, geom).z >= floor

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)
    grip = io.read().get("gripper", 60.0)

    def go(d, secs=1.0):
        n = to_norm(d)
        n["gripper"] = grip
        io.write(n, secs)
        time.sleep(SETTLE)
        return to_deg(io.read())

    here = to_deg(io.read())
    go(here, 0.4)
    _see = (lambda p=None: look_stem(p)) if args.aim == 'green' else (lambda p=None: look(p, aim=args.aim))
    s = _see()
    last_uv = None if s is None else (s[1], s[2])
    if s is None:
        print("❌ 가까운 빨간 열매가 안 보인다 — 보이는 자세에서 시작하라.")
        io.hold_close()
        return 1
    print(f"목표 화면자리 ({TARGET_UV[0]:.0f}, {TARGET_UV[1]:.0f}) · 깊이 {args.to:.0f}mm"
          + ("  (벽을 눌러 실측한 파지점)" if TARGET_Z else "  (어림값)"))
    print("  걸음  넓이   화면(u,v)      깊이   남은거리(px, mm)   움직인 관절")

    for step in range(args.steps + 1):
        s = _see(last_uv)
        if s is None and last_uv is None:
            s = _see()
        # ⚠ 한 번 잡은 뒤에는 **전체 검색으로 되돌아가지 않는다.** 되돌아가면
        #   놓친 순간 화면에서 가장 큰 다른 초록(화분 틀)으로 갈아타고, 서보가
        #   엉뚱한 데로 끌려간다 — 2026-09-02, 3걸음째에 243px을 건너뛰었다.
        #   놓쳤으면 멈추는 편이 낫다.
        if s is not None:
            last_uv = (s[1], s[2])
        if s is None:
            print("  열매를 놓쳤다 — 멈춘다.")
            break
        du = TARGET_UV[0] - s[1]
        dv = TARGET_UV[1] - s[2]
        have_z = s[3] > 0
        dz = (args.to - s[3]) if have_z else 0.0
        print(f"  {step:3}  {s[0]:6}  ({s[1]:5.0f},{s[2]:5.0f})  "
              + (f"{s[3]:5.0f}mm  " if have_z else "  없음  ")
              + f"({math.hypot(du, dv):5.0f}px, {dz:+5.0f}mm)", end="")
        if math.hypot(du, dv) < 18.0 and (not have_z or abs(dz) < 12.0):
            print("   ✅ 닿았다")
            break
        if step == args.steps:
            print("   (걸음 다 씀)")
            break

        # ── 야코비안을 **여기서** 다시 잰다 (자세가 바뀌면 값도 바뀐다) ──
        base = to_deg(io.read())
        # ⚠ 가까울수록 작게 흔든다. 멀 때 쓰던 ±5°를 100mm 앞에서 쓰면 열매가
        #   화면 밖으로 튀어 야코비안을 못 잰다(2026-09-02: 108mm에서 그랬다).
        probe0 = float(np.clip(PROBE_DEG * s[3] / 250.0, 1.5, PROBE_DEG))
        J, cols = [], []
        for j in AXES:
            probe = probe0
            d = dict(base)
            d[j] += probe
            if not legal(d, base):
                d[j] = base[j] - probe
                probe = -probe0
                if not legal(d, base):
                    print(f"     [{j}] 양쪽 다 한계", end="")
                    continue
            go(d)
            t = _see(last_uv)
            go(dict(base))
            if t is None:
                print(f"     [{j}] 흔드니 열매를 놓침", end="")
                continue
            dzp = ((t[3] - s[3]) / probe) if (have_z and t[3] > 0) else 0.0
            J.append([(t[1] - s[1]) / probe, (t[2] - s[2]) / probe, dzp])
            cols.append(j)
        if len(cols) < 2:
            print("     ⚠ 야코비안을 못 쟀다 — 멈춘다.")
            break
        A = np.array(J).T                       # (3 x k): 행=(u,v,z), 열=관절
        want = np.array([du, dv, dz])
        sol, *_ = np.linalg.lstsq(A, want * args.gain, rcond=None)
        # 가까울수록 걸음도 작게 — 큰 걸음이 곧 열매를 치는 걸음이다.
        cap = float(np.clip(args.max_step * s[3] / 250.0, 1.5, args.max_step))
        sol = np.clip(sol, -cap, cap)
        nxt = dict(base)
        for j, v in zip(cols, sol):
            nxt[j] += float(v)
        # ⚠ 걸리면 걸음을 통째로 버리지 않는다 — **관절마다 갈 수 있는 만큼만
        #   잘라서** 간다. 한 관절이 한계에 붙었다고 나머지까지 못 갈 이유가 없다.
        #   (통째로 거부하면 "어느 방향으로도 못 간다"며 마지막 몇 cm에서 선다.)
        cm = to_norm(base)
        nxt = dict(base)
        for j, v in zip(cols, sol):
            want = base[j] + float(v)
            lim = max(98.0, abs(cm.get(j, 0.0)))
            n_want = to_norm({**base, j: want})[j]
            if abs(n_want) > lim:
                n_want = math.copysign(lim, n_want)
                want = ref.get(j, 0.0) + sign(j) * (n_want - zero.get(j, 0.0)) * per(j)
            nxt[j] = want
        shrink = 0
        while not legal(nxt, base) and shrink < 5:      # 남은 건 바닥뿐이다
            nxt = {j: base[j] + (nxt[j] - base[j]) * 0.5 for j in nxt}
            shrink += 1
        if not legal(nxt, base):
            print("     ⚠ 바닥에 걸린다 — 멈춘다.")
            break
        if max(abs(nxt[j] - base[j]) for j in kin.JOINTS) < 0.2:
            print("     ⚠ 모든 관절이 한계다 — 더 못 간다.")
            break
        got = go(nxt, 1.2)
        print("     " + " ".join(f"{j.split('_')[0]}{v:+.1f}" for j, v in zip(cols, sol)))

    fin = _see(last_uv)
    if fin is not None:
        print(f"\n끝: 화면({fin[1]:.0f},{fin[2]:.0f}) 깊이 {fin[3]:.0f}mm · "
              f"카메라계 ({fin[4][0]:.1f}, {fin[4][1]:.1f}, {fin[4][2]:.1f}) mm")
    print("⚠ 토크를 켠 채로 둔다.")
    io.hold_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
