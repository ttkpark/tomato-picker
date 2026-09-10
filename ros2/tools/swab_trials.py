#!/usr/bin/env python3
"""**면봉 잡기 성공률**(과제 0 연습) — 시작자세 → 흰 솜 끝 찾기 → stem_grasp → 들어 확인 → 놓기.
    ~/lerobot/.venv/bin/python ros2/tools/swab_trials.py --trials 5
    ~/lerobot/.venv/bin/python ros2/tools/swab_trials.py --trials 1 --keep   (끝나도 안 놓는다)
⚠ `tomato-voice`는 내리고 `depth-cam`은 켠 채로. 베이스는 안 건드린다.
────────────────────────────────────────────────────────────────────────
왜 — 열매가 없어서(2026-09-09) 화분 위 면봉으로 "잡기 시늉"을 반복한다. 사람이 손으로
잡아 본 자세(9/10 새벽, 정규값 pan −28.6 lift −86.3 elbow 67.7 wrist 38.5 roll 0)에서
80mm 올리고 피치를 20° 내린 자세를 **시작자세**로 삼는다(도 틀 START). 거기서 솜 끝이
화면 (393,315) 근처에 보였고, `stem_grasp.py --aim mark`가 13mm 나아가 닫았다 —
그런데 들어 보니 **면봉은 화분에 그대로**였다(집게가 솜 옆을 물었다). 그래서 이
스크립트는 **"닫았다"를 성공으로 안 친다.** 닫은 뒤 70mm 들고 아래를 내려다봐서
흰 솜이 화분에 남아 있으면 실패다.
⚠ 판정은 흰 덩이(S<45, V>190)의 자리로 한다 — 화분 벽·종이도 희다. 크기(80~1500px)와
   높이(화면 아래 절반)로 거른다. 조명이 바뀌면 문턱이 흔들린다 — 사진을 남기니 눈으로 재확인.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
# 도 틀 (arm_stage.py). ⚠ **시작자세는 표적이 집게 자리에 이미 가깝도록 고른다** —
# 9/10 새벽엔 솜이 화면 위(v≈150)에 있고 집게 무는 자리는 (492,408)이라 처음부터
# 250화소가 벌어져 있었고, 22걸음을 다 써도 못 좁혔다(11회 0성공). wrist_flex를 75→56으로
# 내려(카메라를 위로 들어) 솜을 화면 아래로 내리고 pan을 −6→−3으로 맞추자 **10화소**가 됐다.
# 겨냥이 거의 끝난 자리에서 시작하면 남은 일은 접근축을 따라 나아가는 것뿐이다.
# ⚠ 9/10 저녁: 사용자가 면봉 둘을 지지대에 다시 꽂았다. 네 자세를 재 보니 pan −30에서
#   표적이 집게 자리에서 52화소로 가장 가깝다. 이 자세에서 솜 머리는 (483,275) 근처다.
#   ⚠ 자세를 바꾸면 이 두 값을 **같이** 다시 재라 — 하나만 고치면 표적을 못 찾는다.
# ⚠ 9/10 23시: 집게 무는 자리를 다시 재고(407,350) 남은 면봉(오른쪽 지지대)에 맞춘 자세.
#   여기서 솜 머리가 (392,298) — 무는 자리 바로 위 54화소다.
START = "45,10,120,62,0"
TIP_NEAR = (392, 320)
COLOR = "/dev/shm/d405_color.jpg"
DEPTH = "/dev/shm/d405_depth.npy"
META = "/dev/shm/d405_meta.json"
GRIP_EMPTY = 4.8        # 빈 집게가 닫히는 자리(실측 9/10). 물면 이보다 벌어진 채 선다.
GRIPF = os.path.expanduser("~/grip_uv.json")


def bite_uv(default=(407, 350)):
    """집게가 **실제로 무는 화면 자리**. `~/grip_uv.json`이 이긴다.

    ⚠ 이 값이 틀리면 겨냥이 아무리 정확해도 허공을 문다 — 2026-09-10 22:30에
      그것 때문에 여섯 번을 헛집었다. 옛 값 (492,408)은 9/3에 잰 것인데, 그 뒤
      wrist_roll을 두 번 재중심하면서 화면이 돌아 **그 화소는 배경(180mm)**이 됐다.
      지금 값은 닫은 집게의 검은 패드 끝을 깊이로 찾아 잰 것(76mm, 실거리 79mm).
      ⚠ 롤을 다시 만지면 **반드시 다시 재라**.
    """
    try:
        g = json.load(open(GRIPF))
        return float(g["u"]), float(g["v"])
    except Exception:                                      # noqa: BLE001
        return default
# ⚠ 실측 9/10: 빈 집게 4.8, **면봉 자루(지름 2mm)를 물었을 때 6.2**. 그래서 문턱은 5.5다.
#   처음엔 7.0으로 뒀다가 **실제로 집어 올린 시도를 실패로 찍었다**(사진으로 확인).
#   자루가 얇아 여유가 1.4단위뿐이니, 판정은 넓이(lifted_near)로 한 번 더 받친다.
# 실측 9/10: --grip-shut 0 · 토크 상한 900으로 **빈손**을 닫으면 2.81에 선다.
# 솜 머리(지름 5mm)를 물면 그보다 훨씬 벌어진 채 서므로 문턱은 4.5로 넉넉히 둔다.
# 실측 9/10 23:15 (--grip-shut 0 · 토크 900): **빈손 2.81 · 솜 머리를 물면 3.7~3.9.**
# 여유가 1단위뿐이라 문턱은 3.3. (4.5로 뒀더니 실제로 물어 올린 것을 놓쳤다.)
GRIP_HELD = 3.3
OUT = os.path.expanduser("~/swab_trials")
LOG = os.path.join(OUT, "trials.jsonl")


def run(argv, timeout):
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired as e:
        return 124, str(e)


def tool(name, *args, timeout=120):
    return run([PY, os.path.join(HERE, name), *args], timeout)


def snap(tag):
    """컬러와 **깊이까지** 함께 남긴다 — 판정이 깊이로 바뀌었기 때문이다."""
    time.sleep(1.2)
    dst = os.path.join(OUT, tag + ".jpg")
    shutil.copy(COLOR, dst)
    try:
        shutil.copy(DEPTH, os.path.join(OUT, tag + ".npy"))
        shutil.copy(META, os.path.join(OUT, tag + ".json"))
    except Exception:                                      # noqa: BLE001
        pass
    return dst


def near_target(tag=None, max_mm=200.0, min_mm=95.0, max_area=4000, near_uv=None):
    """**집을 수 있는 거리에 있는 덩이 중 집게 자리에 가장 가까운 것** (u, v, z, 넓이).

    ⚠ 처음엔 "가장 가까운 것"을 골랐는데, 화분 테두리 한 귀퉁이가 면봉보다 가까우면
      띠가 그쪽에 열려 엉뚱한 데를 겨눴다(9/10 09:47: (252,451) 97mm를 표적으로 골라
      팔이 반대로 갔다). 가까움은 **표적을 고르는 기준이 아니라 배경을 거르는 기준**이다.
      실제 기준은 "집게가 무는 자리에 가까운 것" — 시작자세를 표적에 붙여 뒀기 때문이다.
    ⚠ 보라색 집게(자기 손)는 깊이에서 지운다. D405 하한 70mm 아래 값도 안 믿는다.
    """
    import cv2
    import numpy as np
    if near_uv is None:
        near_uv = bite_uv()
    dp = os.path.join(OUT, tag + ".npy") if tag else DEPTH
    mp = os.path.join(OUT, tag + ".json") if tag else META
    cp = os.path.join(OUT, tag + ".jpg") if tag else COLOR
    try:
        dep = np.load(dp).astype(float) * json.load(open(mp))["depth_scale_mm"]
    except Exception:                                      # noqa: BLE001
        return None
    bgr = cv2.imread(cp)
    if bgr is not None and bgr.shape[:2] == dep.shape[:2]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # 보라색 몸통 + 검은 고무 패드 = 자기 손. 9/10: 검은 패드(75mm)를 표적으로 골랐다.
        own = (((hsv[:, :, 0] > 115) & (hsv[:, :, 0] < 165) & (hsv[:, :, 1] > 55))
               | (hsv[:, :, 2] < 55)).astype(np.uint8)
        dep = np.where(cv2.dilate(own, np.ones((9, 9), np.uint8)) > 0, 0.0, dep)
    m = ((dep > min_mm) & (dep < max_mm)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    cand = []
    for i in range(1, n):
        a = int(st[i, cv2.CC_STAT_AREA])
        if a < 100 or a > max_area:
            continue
        u, v = float(cen[i][0]), float(cen[i][1])
        d = (u - near_uv[0]) ** 2 + (v - near_uv[1]) ** 2
        cand.append((d, u, v, a, i))
    if not cand:
        return None
    # 집게 자리 근처(220화소)를 먼저 본다. 거기 아무것도 없으면(놓은 면봉이 굴러갔을 때)
    # **가장 가까운 것**으로 물러선다 — 표적이 아예 없다고 말하는 것보다 낫다.
    near = [c for c in cand if c[0] <= 240.0 ** 2]
    if near:
        # ⚠ **면봉의 윗 팁을 잡아야 한다**(사용자 요청). 집게 앞 240화소 안에는 보통
        #   화분 테두리(아래·넓다)와 솜 머리(위·작다)가 같이 잡히는데, 자루 한가운데나
        #   테두리를 물면 미끄러진다 — 솜 머리는 지름 5mm에 눌려서 잘 물린다.
        #   그래서 **화면에서 가장 위에 있는 것**을 고른다. 서 있는 면봉의 머리가 늘 위다.
        best = min(near, key=lambda c: c[2])
    else:
        best = min(cand, key=lambda c: float(np.percentile(dep[lab == c[4]][dep[lab == c[4]] > 0], 20))
                   if (dep[lab == c[4]] > 0).any() else 9e9)
    _d, u, v, a, idx = best
    dd = dep[lab == idx]
    dd = dd[dd > 0]
    return u, v, (float(np.percentile(dd, 20)) if dd.size else -1.0), a


def grip_now(shut=0.0, torque=900):
    """집게를 **잡을 때와 같은 힘으로** 다시 닫고 선 자리를 읽는다 — 물면 덜 닫힌다.

    ⚠ 예전엔 여기서 `grip_set.py 4`(약한 기본 토크)를 썼다. 그러면 기준선이 잡을 때와
      달라져(빈손 4.8 vs 2.8) **닫지도 않은 시도를 성공으로 찍었다**(9/10 10:33).
      판정과 실제 동작은 같은 조건이어야 한다."""
    rc, out = tool("grip_set.py", "%.0f" % shut, "--torque", "%d" % torque, timeout=240)
    m = re.search(r"지금\s+([0-9.]+)", out)
    if m:
        return float(m.group(1))
    # ⚠ 못 읽으면 **모른다고 말한다** — 예전엔 조용히 None을 돌려주고 판정이
    #   "unknown"으로 끝나, 실제로 물어 올린 시도를 아무도 성공으로 못 셌다(9/10 23:13).
    print("  ⚠ 집게 값을 못 읽었다 (rc=%d): %s" % (rc, " ".join(out.split())[-120:]))
    return None


def white_blobs(path, y_min=0, y_max=480):
    """작은 흰 덩이들 [(u, v, w, h, area)] — 솜 끝 후보."""
    import cv2
    import numpy as np
    im = cv2.imread(path)
    if im is None:
        return []
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    # ⚠ 문턱은 S<80·V>150 — 9/10 실측: 이 방 조명에서 솜은 V≈142, S≈20으로 "흰색"치고
    #   어둡다. V>190은 솜을 버리고 화분 테두리·종이만 남겼다(사용자: "면봉 잘 있는데?
    #   인식 못하네"). 대신 **둥근 것만** 받는다(채움률>0.55, 종횡비>0.6) — 테두리 조각은
    #   가늘고 길다. ⚠ 채움률 문턱은 0.45 — 0.55로 잡았더니 솜(0.53)이 떨어지고 대신
    #   **바닥의 흰 케이블 뭉치**(0.55, 지름 48px)가 뽑혀 팔이 그리로 갔다(9/10 00:53).
    #   종횡비도 0.6→0.5로 — 솜이 비스듬히 보이면 25×15로 납작해진다(실측 0.60에서 탈락).
    m = ((hsv[:, :, 1] < 80) & (hsv[:, :, 2] > 150)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m[:y_min, :] = 0
    m[y_max:, :] = 0
    n, _lab, st, cen = cv2.connectedComponentsWithStats(m)
    out = []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if 120 <= a <= 2500 and w <= 90 and h <= 90 and a / (w * h) > 0.45 and min(w, h) / max(w, h) > 0.5:
            out.append((float(cen[i][0]), float(cen[i][1]), int(w), int(h), int(a)))
    return out


def pick_tip(blobs, near=TIP_NEAR, max_px=220.0):
    """TIP_NEAR에서 가장 가까운 덩이 — **너무 멀면 없는 것으로 친다.**

    ⚠ 거리 제한이 없으면 솜을 놓쳤을 때 화면 어디의 흰 것이든(케이블·바닥) 뽑아서
      팔을 엉뚱한 데로 보낸다. 못 찾으면 실패로 끝내는 편이 낫다."""
    if not blobs:
        return None
    b = min(blobs, key=lambda b: (b[0] - near[0]) ** 2 + (b[1] - near[1]) ** 2)
    if (b[0] - near[0]) ** 2 + (b[1] - near[1]) ** 2 > max_px ** 2:
        return None
    return b


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--keep", action="store_true", help="마지막 시도 뒤 놓지 않는다")
    ap.add_argument("--stop-z", type=float, default=84.0)
    ap.add_argument("--aim", default="near",
                    help="stem_grasp 겨냥 방식 — near(깊이 띠, 기본) / white(색) / mark(조각 정합). "
                         "흰 면봉이 흰 화분 테두리와 겹쳐 색으로는 못 가르기에 깊이를 기본으로 둔다")
    ap.add_argument("--max-dz", type=float, default=70.0, help="stem_grasp에 넘길 깊이 도약 문턱")
    # ⚠ 큰 걸음은 가는 표적을 추적창 밖으로 던진다(9/10: 한 걸음에 118화소 이동).
    ap.add_argument("--max-turn", type=float, default=3.0, help="한 걸음에 한 관절 최대 도수")
    ap.add_argument("--gain", type=float, default=0.6, help="겨냥 게인 — 기본 0.55는 수렴이 느렸다")
    # ⚠ 문턱을 열매 기본값(28화소)으로 두면 **빈 채로 닫는다.** 79mm에서 14화소는 옆으로
    #   2.5mm인데 면봉 자루는 지름 2mm다 — 집게 사이로 안 들어온다(9/10 10:22 실측:
    #   겨냥 14화소·79mm에서 닫았으나 집게 4.9=빈손). 가는 것을 물려면 화소도 가늘게.
    # ⚠ 문턱 8화소는 **너무 좁아 영영 겨냥만 한다**(9/10: 30걸음 내내 17화소에서 맴돌다
    #   무는 거리에 못 닿았다). 솜 머리는 지름 5mm라 95mm에서 12화소(≈2.6mm)면 집게 안에
    #   들어온다 — 자루(2mm)를 물 때와 달리 여유가 있다.
    ap.add_argument("--tol", type=float, default=12.0, help="이 화소 안이면 겨냥이 됐다")
    # ⚠ 사용자 관찰(9/10): "잡긴 했는데 접지력이 작아 못 올렸어." 가는 자루(2mm) 대신
    #   **위쪽 솜 머리**(5mm, 눌린다)를 물고, 더 깊이 닫고, 집게 토크 상한을 그 순간만 올린다.
    ap.add_argument("--near-top", type=float, default=0.3, help="표적 덩이의 윗부분 이 비율을 겨눈다")
    ap.add_argument("--grip-shut", type=float, default=0.0, help="닫을 때 집게 값 — 작을수록 세게")
    ap.add_argument("--grip-torque", type=int, default=900, help="닫는 순간 집게 토크 상한(0=그대로)")
    ap.add_argument("--steps", type=int, default=45)
    # ⚠ 한 걸음이 9초쯤이라 45걸음이면 7분을 넘긴다 — 예전 420초 제한이 **다 되기 전에
    #   잘라 버려** 무는 순간을 못 봤다(9/10 11:01).
    ap.add_argument("--timeout", type=int, default=700, help="stem_grasp 한 번의 제한(초)")
    ap.add_argument("--max-adv", type=float, default=45.0, help="이보다 더 나아가야 하면 겨냥이 틀린 것")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)

    r = subprocess.run(["systemctl", "is-active", "tomato-voice"], capture_output=True, text=True)
    if r.stdout.strip() == "active":
        print("❌ tomato-voice가 켜져 있다 — 팔 포트를 못 연다. sudo systemctl stop tomato-voice")
        return 1

    results = []
    for k in range(1, args.trials + 1):
        t0 = time.time()
        tag = time.strftime("%m%d-%H%M%S") + f"-t{k}"
        rec = {"tag": tag, "trial": k}
        print(f"\n═══ 시도 {k}/{args.trials}  {tag}")

        rc, out = tool("grip_set.py", "78")
        rc, out = tool("arm_stage.py", "--target=" + START, timeout=90)
        rec["stage_rc"] = rc
        if rc != 0:
            print("  ❌ 시작자세로 못 갔다:", out.strip().splitlines()[-1:])
            rec["why"] = "stage"
            results.append(rec)
            continue
        snap(tag + "-0start")
        tip = near_target(tag + "-0start")
        if tip is None:
            print("  ❌ 시작자세에서 가까운 표적이 안 보인다")
            rec["why"] = "no_tip"
            results.append(rec)
            continue
        u, v, z_before, a0 = tip
        rec["tip_uv"] = [round(u), round(v)]
        rec["tip_z"] = round(z_before)
        print(f"  표적 화면 ({u:.0f},{v:.0f}) 깊이 {z_before:.0f}mm 넓이 {a0}")

        # ⚠ --max-adv를 짧게 준다. 2026-09-10 시도 1: 솜 끝(약 100mm 앞)의 깊이가 배경
        #   값(239mm)으로 읽혀 stem_grasp가 120mm를 밀고 들어갔고, 화분을 밀고 지지대와
        #   면봉을 쓰러뜨렸다. 시작자세에서 솜 끝까지는 50mm 안쪽이어야 정상이므로 그
        #   이상 나아가는 건 겨냥이 틀렸다는 뜻이다 — 실패로 끝내는 편이 무대를 부수는
        #   것보다 낫다(무인 반복은 무대를 스스로 못 고친다).
        # ── 1) 겨냥만 한다 ─────────────────────────────────────────────────
        # ⚠ 나아가는 일을 화면에 맡기면 표적이 커지고·손이 가리고·깊이가 튀는 셋이 한꺼번에
        #   와서 겨냥이 벌어진다(9/10 22:00: 80화소인 채로 끝까지 가 허공을 물었다).
        #   겨냥이 맞은 그 순간이 가장 믿을 만하니 거기서 멈추고, 나아가기는 기구학으로 한다.
        rc, out = tool("stem_grasp.py", "--aim", args.aim, "--mark", f"{u:.0f},{v:.0f}",
                       "--aim-only",
                       "--no-red-check", "--stop-z", str(args.stop_z),
                       "--max-adv", str(args.max_adv), "--thin",
                       "--max-dz", str(args.max_dz), "--gain", str(args.gain),
                       "--tol", str(args.tol), "--near-top", str(args.near_top),
                       "--max-turn", str(args.max_turn),
                       "--grip-shut", str(args.grip_shut),
                       "--grip-torque", str(args.grip_torque), "--trust-first-z",
                       "--steps", str(args.steps), timeout=args.timeout)
        rec["grasp_rc"] = rc
        tail = [l for l in out.splitlines() if l.strip()][-3:]
        rec["grasp_tail"] = tail
        print("  겨냥 rc=%d  %s" % (rc, " | ".join(tail)))

        # ── 2) 겨냥이 맞았으면 그 깊이만큼 뻗어서 문다 ─────────────────────
        if rc == 0:
            try:
                tip = json.load(open(os.path.expanduser("~/tip_depth.json")))
                z_aim = float(tip["cam_to_tip_mm"])
                rec["aim_z"] = round(z_aim)
                rec["aim_err_px"] = round(float(tip.get("err_px", -1)), 1)
            except Exception:                              # noqa: BLE001
                z_aim = -1.0
            reach = max(0.0, min(args.max_adv, z_aim - args.stop_z)) if z_aim > 0 else 0.0
            rec["reach_mm"] = round(reach)
            print("  뻗기: %.0fmm (겨냥 깊이 %.0fmm − 무는 거리 %.0fmm)"
                  % (reach, z_aim, args.stop_z))
            if reach > 3.0:
                rc2, out2 = tool("tool_jog.py", "--along", "%.0f" % reach,
                                 "--piece", "12", timeout=240)
                rec["jog_rc"] = rc2
                last = [l for l in out2.splitlines() if l.strip()][-1:]
                print("  뻗기 rc=%d  %s" % (rc2, " ".join(last)))
            tool("grip_set.py", "%.0f" % args.grip_shut, "--torque", "%d" % args.grip_torque)
        snap(tag + "-1closed")
        with open(os.path.join(OUT, tag + "-grasp.log"), "w") as f:
            f.write(out)

        # ── 물었는가: **집게가 선 자리**가 가장 정직하다 ──
        # ⚠ 사진 속 흰 덩이로 판정하던 것을 버렸다(9/10 09:01: 실제로 집어 올렸는데
        #   화분 테두리를 "화분에 남은 면봉"으로 세어 실패로 찍었다). 빈 집게는 4.8에
        #   서고, 면봉 자루를 물면 그보다 벌어진 채 선다 — 그건 카메라가 아니라 물리다.
        grip = grip_now(args.grip_shut, args.grip_torque)
        rec["grip"] = grip
        tool("tool_jog.py", "--dz", "70", "--piece", "20")
        snap(tag + "-2lifted")
        held = near_target(tag + "-2lifted", max_mm=150.0, min_mm=60.0)
        rec["lifted_near"] = None if held is None else [round(held[0]), round(held[1]),
                                                       round(held[2]), held[3]]
        area = (rec["lifted_near"] or [0, 0, 0, 0])[3]
        if grip is not None and grip >= GRIP_HELD:
            verdict = "success" if area >= 400 else "success_weak"
        elif rc != 0:
            verdict = "fail_grasp_rc"
        elif grip is not None:
            verdict = "fail_empty_close"
        else:
            verdict = "unknown"
        rec["verdict"] = verdict
        rec["secs"] = round(time.time() - t0, 1)
        print(f"  판정: {verdict}   (집게 {grip}, 빈집게 {GRIP_EMPTY} · 든 뒤 가까운 것 {rec['lifted_near']})")
        results.append(rec)
        with open(LOG, "a") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

        # ── 놓기: 내려서 열고 ──
        if not (args.keep and k == args.trials):
            tool("tool_jog.py", "--dz", "-70", "--piece", "20")
            tool("grip_set.py", "78")
            snap(tag + "-3released")

    n = len(results)
    ok = sum(1 for r in results if str(r.get("verdict", "")).startswith("success"))
    print(f"\n합계: {ok}/{n} 성공  ({', '.join(r.get('verdict', r.get('why', '?')) for r in results)})")
    print(f"사진·로그: {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
