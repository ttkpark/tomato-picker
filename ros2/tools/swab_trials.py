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
START = "-3,10,120,56,0"
TIP_NEAR = (497, 399)
COLOR = "/dev/shm/d405_color.jpg"
DEPTH = "/dev/shm/d405_depth.npy"
META = "/dev/shm/d405_meta.json"
GRIP_EMPTY = 4.8        # 빈 집게가 닫히는 자리(실측 9/10). 물면 이보다 벌어진 채 선다.
GRIP_HELD = 7.0         # 이보다 벌어져 있으면 **뭔가 물고 있다**
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


def near_target(tag=None, max_mm=200.0, band=25.0, max_area=8000):
    """**카메라에 가장 가까운 덩이** (u, v, z, 넓이) — 집으려는 것은 늘 가까운 쪽이다.

    ⚠ 색으로 찾던 것을 깊이로 바꿨다. 흰 면봉이 **흰 화분 테두리와 겹치면** 색으로는
      한 덩이가 되어 둥글기 검사에 떨어진다(9/10: 6mm 나아가자마자 표적이 사라졌다).
      깊이로 보면 면봉 90mm, 테두리·벽은 150mm 넘는다 — 겹쳐도 갈린다.
    """
    import cv2
    import numpy as np
    dp = os.path.join(OUT, tag + ".npy") if tag else DEPTH
    mp = os.path.join(OUT, tag + ".json") if tag else META
    try:
        dep = np.load(dp).astype(float) * json.load(open(mp))["depth_scale_mm"]
    except Exception:                                      # noqa: BLE001
        return None
    # ⚠ 자기 손가락을 표적으로 고르지 않게 **보라색 집게를 지운다**(9/10: 63mm짜리
    #   덩이를 매번 표적으로 골랐다). 그리고 D405 하한(70mm) 아래 값은 안 믿는다.
    cp = os.path.join(OUT, tag + ".jpg") if tag else COLOR
    bgr = cv2.imread(cp)
    if bgr is not None and bgr.shape[:2] == dep.shape[:2]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        own = ((hsv[:, :, 0] > 115) & (hsv[:, :, 0] < 165) & (hsv[:, :, 1] > 55)).astype(np.uint8)
        dep = np.where(cv2.dilate(own, np.ones((9, 9), np.uint8)) > 0, 0.0, dep)
    lower = dep[dep.shape[0] // 3:, :]
    vals = lower[(lower > 75.0) & (lower < max_mm)]
    if vals.size < 200:
        return None
    z0 = float(np.percentile(vals, 2)) + band * 0.5
    m = ((dep > max(75.0, z0 - band)) & (dep < z0 + band)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, n):
        a = int(st[i, cv2.CC_STAT_AREA])
        if a < 60 or a > max_area:
            continue
        if best is None or a > best[0]:
            best = (a, float(cen[i][0]), float(cen[i][1]), i)
    if best is None:
        return None
    a, u, v, idx = best
    d = dep[lab == idx]
    d = d[d > 0]
    return u, v, (float(np.percentile(d, 20)) if d.size else z0), a


def grip_now():
    """집게를 닫으라고 한 번 더 시키고 **선 자리**를 읽는다 — 물면 덜 닫힌다."""
    rc, out = tool("grip_set.py", "4")
    m = re.search(r"지금\s+([0-9.]+)", out)
    return float(m.group(1)) if m else None


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
    ap.add_argument("--gain", type=float, default=0.8, help="겨냥 게인 — 기본 0.55는 수렴이 느렸다")
    ap.add_argument("--steps", type=int, default=30)
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
        rc, out = tool("stem_grasp.py", "--aim", args.aim, "--mark", f"{u:.0f},{v:.0f}",
                       "--no-red-check", "--stop-z", str(args.stop_z),
                       "--max-adv", str(args.max_adv), "--thin",
                       "--max-dz", str(args.max_dz), "--gain", str(args.gain),
                       "--steps", str(args.steps), timeout=420)
        rec["grasp_rc"] = rc
        tail = [l for l in out.splitlines() if l.strip()][-3:]
        rec["grasp_tail"] = tail
        print("  stem_grasp rc=%d  %s" % (rc, " | ".join(tail)))
        snap(tag + "-1closed")
        with open(os.path.join(OUT, tag + "-grasp.log"), "w") as f:
            f.write(out)

        # ── 물었는가: **집게가 선 자리**가 가장 정직하다 ──
        # ⚠ 사진 속 흰 덩이로 판정하던 것을 버렸다(9/10 09:01: 실제로 집어 올렸는데
        #   화분 테두리를 "화분에 남은 면봉"으로 세어 실패로 찍었다). 빈 집게는 4.8에
        #   서고, 면봉 자루를 물면 그보다 벌어진 채 선다 — 그건 카메라가 아니라 물리다.
        grip = grip_now()
        rec["grip"] = grip
        tool("tool_jog.py", "--dz", "70", "--piece", "20")
        snap(tag + "-2lifted")
        held = near_target(tag + "-2lifted", max_mm=150.0)
        rec["lifted_near"] = None if held is None else [round(held[0]), round(held[1]),
                                                       round(held[2]), held[3]]
        if grip is not None and grip >= GRIP_HELD:
            verdict = "success"
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
    ok = sum(1 for r in results if r.get("verdict") == "success")
    print(f"\n합계: {ok}/{n} 성공  ({', '.join(r.get('verdict', r.get('why', '?')) for r in results)})")
    print(f"사진·로그: {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
