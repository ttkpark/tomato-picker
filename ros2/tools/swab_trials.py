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
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
# 도 틀 (arm_stage.py). 9/10 01시: 시도 1이 화분을 밀어 자리가 바뀌었고, 면봉은 **오른쪽**
# 지지대에 서 있다. 이 자세에서 솜 끝이 화면 (540,150) 근처, TCP z≈160.
START = "-6,10,120,75,0"
TIP_NEAR = (540, 150)
COLOR = "/dev/shm/d405_color.jpg"
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
    time.sleep(1.2)
    dst = os.path.join(OUT, tag + ".jpg")
    shutil.copy(COLOR, dst)
    return dst


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
    ap.add_argument("--aim", default="white", help="stem_grasp 겨냥 방식 — white(색 재검출) / mark(조각 정합)")
    ap.add_argument("--max-dz", type=float, default=70.0, help="stem_grasp에 넘길 깊이 도약 문턱")
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
        before = snap(tag + "-0start")
        tip = pick_tip(white_blobs(before, y_min=60))
        if tip is None:
            print("  ❌ 시작자세에서 솜 끝이 안 보인다")
            rec["why"] = "no_tip"
            results.append(rec)
            continue
        u, v = tip[0], tip[1]
        rec["tip_uv"] = [round(u), round(v)]
        print(f"  솜 끝 화면 ({u:.0f},{v:.0f}) 넓이 {tip[4]}")

        # ⚠ --max-adv를 짧게 준다. 2026-09-10 시도 1: 솜 끝(약 100mm 앞)의 깊이가 배경
        #   값(239mm)으로 읽혀 stem_grasp가 120mm를 밀고 들어갔고, 화분을 밀고 지지대와
        #   면봉을 쓰러뜨렸다. 시작자세에서 솜 끝까지는 50mm 안쪽이어야 정상이므로 그
        #   이상 나아가는 건 겨냥이 틀렸다는 뜻이다 — 실패로 끝내는 편이 무대를 부수는
        #   것보다 낫다(무인 반복은 무대를 스스로 못 고친다).
        rc, out = tool("stem_grasp.py", "--aim", args.aim, "--mark", f"{u:.0f},{v:.0f}",
                       "--no-red-check", "--stop-z", str(args.stop_z),
                       "--max-adv", str(args.max_adv), "--thin",
                       "--max-dz", str(args.max_dz), timeout=420)
        rec["grasp_rc"] = rc
        tail = [l for l in out.splitlines() if l.strip()][-3:]
        rec["grasp_tail"] = tail
        print("  stem_grasp rc=%d  %s" % (rc, " | ".join(tail)))
        snap(tag + "-1closed")
        with open(os.path.join(OUT, tag + "-grasp.log"), "w") as f:
            f.write(out)

        # ── 들어 올려 확인 ──
        tool("tool_jog.py", "--dz", "70", "--piece", "20")
        tool("tool_jog.py", "--pitch", "25")
        look = snap(tag + "-2lifted")
        in_pot = [b for b in white_blobs(look, y_min=200, y_max=360)]
        in_jaw = [b for b in white_blobs(look, y_min=360) if 360 <= b[0] <= 540]
        rec["in_pot"] = [[round(b[0]), round(b[1]), b[4]] for b in in_pot]
        rec["in_jaw"] = [[round(b[0]), round(b[1]), b[4]] for b in in_jaw]
        if rc == 0 and not in_pot and in_jaw:
            verdict = "success"
        elif in_pot:
            verdict = "fail_left_in_pot"
        elif rc != 0:
            verdict = "fail_grasp_rc"
        else:
            verdict = "unknown"
        rec["verdict"] = verdict
        rec["secs"] = round(time.time() - t0, 1)
        print(f"  판정: {verdict}   (화분에 흰 덩이 {len(in_pot)}, 집게 근처 {len(in_jaw)})")
        results.append(rec)
        with open(LOG, "a") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

        # ── 놓기: 내려서 열고 ──
        if not (args.keep and k == args.trials):
            tool("tool_jog.py", "--pitch", "-25")
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
