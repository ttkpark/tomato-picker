#!/usr/bin/env python3
"""**바깥 카메라로** 집게를 열매의 줄기까지 데려간다.

    python tools/outside_servo.py --steps 6

⚠ PC에서 돈다. 젯슨에 ssh로 관절만 명령하고, 눈은 **이 PC에 붙은 웹캠**이다.

────────────────────────────────────────────────────────────────────────
왜 바깥 카메라인가

손목 카메라는 마지막 10cm에서 눈이 감긴다 — 열매(지름 74mm)가 100mm 앞에 오면
화면(848x480)보다 커져 잘리고, 잘린 덩이의 중심과 크기가 둘 다 틀어진다.
2026-09-02에 그것 때문에 "닿았다"고 하고도 90mm가 남아 있었다.

바깥 카메라는 **집게와 열매를 한 화면에서 같이 본다.** 그러면 좁혀야 할 것이
"깊이"가 아니라 화면 위의 벡터 하나가 되고, 그 벡터는 가까워질수록 오히려
더 잘 보인다. 손-눈 보정도, 기구학도, 깊이도 필요 없다.

야코비안(관절 1도가 화면을 몇 px 옮기는가)은 **매번 실측한다** — 자세가
바뀌면 값도 바뀌고, 모델을 믿을 이유가 없다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
JETSON = os.environ.get("TOMATO_JETSON", "server@192.168.0.6")
KEY = os.environ.get("TOMATO_KEY", "/c/Users/GH/.ssh/id_ed25519")
SHOT = os.path.join(HERE, "..", ".work", "_os.jpg")
AXES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
OPEN, SHUT = 78.0, 4.0     # 집게를 여닫아 손가락을 드러내는 두 값


def ssh(cmd: str, timeout: int = 300) -> str:
    p = subprocess.run(["ssh", "-i", KEY, "-o", "BatchMode=yes",
                        "-o", "StrictHostKeyChecking=no", JETSON, cmd],
                       capture_output=True, text=True, timeout=timeout)
    return p.stdout


def pose() -> dict:
    out = ssh("~/lerobot/.venv/bin/python ~/tomato-picker/ros2/tools/arm_stage.py "
              "--dry '--target=0,0,0,0,0' 2>&1 | head -1")
    # "  지금  shoulder=  57.8 shoulder=  39.3 elbow= -13.3 wrist= -98.1 wrist=   5.7 ..."
    nums = [float(v) for v in
            __import__("re").findall(r"=\s*(-?\d+\.?\d*)", out)][:5]
    if len(nums) < 5:
        raise RuntimeError("자세를 못 읽었다: " + out[:120])
    return dict(zip(AXES, nums))


def move(p: dict, timeout: int = 300) -> None:
    t = ",".join("%.2f" % p[j] for j in AXES)
    ssh("~/lerobot/.venv/bin/python ~/tomato-picker/ros2/tools/arm_stage.py "
        f"'--target={t}' >/dev/null 2>&1", timeout=timeout)


def shot() -> np.ndarray | None:
    os.makedirs(os.path.dirname(SHOT), exist_ok=True)
    subprocess.run([sys.executable, os.path.join(HERE, "pc_cam.py"), SHOT],
                   capture_output=True, text=True, timeout=90)
    return cv2.imread(SHOT)


def grip(v: float) -> None:
    """집게를 이만큼 벌린다/닫는다."""
    ssh("~/lerobot/.venv/bin/python /tmp/grip_open.py %.0f >/dev/null 2>&1" % v,
        timeout=120)


def blob(mask: np.ndarray):
    m = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    return dict(x=int(st[i, cv2.CC_STAT_LEFT]), y=int(st[i, cv2.CC_STAT_TOP]),
                w=int(st[i, cv2.CC_STAT_WIDTH]), h=int(st[i, cv2.CC_STAT_HEIGHT]),
                cx=float(cen[i][0]), cy=float(cen[i][1]),
                a=int(st[i, cv2.CC_STAT_AREA]))


def see():
    """(집게 끝, 줄기 자리) — 둘 다 바깥 화면 좌표.

    ⚠ **집게를 색으로 찾지 않는다.** 팔 전체가 보라색이고 검은 부분도 여기저기
      있어서, 자세가 바뀌면 팔꿈치나 서보 하우징을 집게로 착각한다 —
      2026-09-02에 그것 때문에 서보가 오히려 벌어졌다.

      대신 **집게를 여닫아** 두 장을 찍고 그 차이를 본다. 장면에서 그때 움직이는
      것은 손가락뿐이다. 색도 표식도 필요 없고, 조명이 바뀌어도 성립한다.
      손목 카메라에서 이미 같은 방법으로 집게 자리를 찾았다(64 → 7796 화소).
    """
    grip(OPEN)
    a = shot()
    grip(SHUT)
    b = shot()
    grip(OPEN)
    if a is None or b is None:
        return None
    ga = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY).astype(float)
    gb = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY).astype(float)
    hsv = cv2.cvtColor(a, cv2.COLOR_BGR2HSV)
    t = blob((hsv[:, :, 1] > 70) & (hsv[:, :, 2] > 90)
             & ((hsv[:, :, 0] < 20) | (hsv[:, :, 0] > 168)))       # 주황 열매
    arm = blob((hsv[:, :, 0] > 120) & (hsv[:, :, 0] < 160)
               & (hsv[:, :, 1] > 60) & (hsv[:, :, 2] > 60))        # 보라 팔(위치만 씀)
    if not t or not arm:
        return None
    stem = np.array([t["cx"], t["y"] - 12.0])                      # 열매 윗변 위

    # ⚠ 이 각도에서 손가락은 작고 앞뒤로 겹쳐 보여 변화가 미미하다(606화소).
    #   그래서 (가) 문턱을 낮추고 (나) **팔 근처만** 본다. 안 그러면 방 어딘가의
    #   잡음이 가장 큰 덩이가 되어 엉뚱한 데를 집게로 읽는다.
    diff = np.abs(ga - gb)
    box = np.zeros(diff.shape, np.uint8)
    x0, x1 = max(0, arm["x"] - 60), min(diff.shape[1], arm["x"] + arm["w"] + 120)
    y0, y1 = max(0, arm["y"] - 60), min(diff.shape[0], arm["y"] + arm["h"] + 120)
    box[y0:y1, x0:x1] = 1
    thr = max(9.0, float(np.percentile(diff[box > 0], 99.5)) * 0.45)
    m = cv2.morphologyEx(((diff > thr) & (box > 0)).astype(np.uint8),
                         cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, n):
        if st[i, cv2.CC_STAT_AREA] < 60:
            continue
        d = math.hypot(cen[i][0] - stem[0], cen[i][1] - stem[1])
        if best is None or d < best[0]:        # 열매에 **가장 가까운** 변화 덩이
            best = (d, i)
    if best is None:
        return None
    i = best[1]
    ys, xs = np.nonzero(lab == i)
    k = int(np.argmin((xs - stem[0]) ** 2 + (ys - stem[1]) ** 2))
    tip = np.array([float(xs[k]), float(ys[k])])
    f = dict(a=int(st[i, cv2.CC_STAT_AREA]), cx=float(cen[i][0]), cy=float(cen[i][1]),
             x=int(st[i, cv2.CC_STAT_LEFT]), y=int(st[i, cv2.CC_STAT_TOP]),
             w=int(st[i, cv2.CC_STAT_WIDTH]), h=int(st[i, cv2.CC_STAT_HEIGHT]))
    return tip, stem, f, t


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=6)
    ap.add_argument("--gain", type=float, default=0.5)
    ap.add_argument("--probe", type=float, default=5.0)
    ap.add_argument("--max-step", type=float, default=8.0)
    ap.add_argument("--tol", type=float, default=25.0, help="이 픽셀 안이면 끝")
    args = ap.parse_args()

    use = ("shoulder_pan", "shoulder_lift")
    print("  걸음   집게끝        줄기        남은 거리(px)   움직인 관절")
    for step in range(args.steps + 1):
        s = see()
        if s is None:
            print("   집게나 열매를 못 찾았다 — 멈춘다.")
            return 1
        tip, stem, g, t = s
        d = stem - tip
        print("  %4d  (%4.0f,%4.0f)  (%4.0f,%4.0f)   %6.0f" %
              (step, tip[0], tip[1], stem[0], stem[1], np.linalg.norm(d)), end="")
        if np.linalg.norm(d) < args.tol:
            print("   ✅ 닿았다")
            return 0
        if step == args.steps:
            print("   (걸음 다 씀)")
            return 0

        base = pose()
        J, cols = [], []
        for j in use:
            p = dict(base)
            p[j] += args.probe
            move(p)
            r = see()
            move(dict(base))
            if r is None:
                continue
            J.append((r[0] - tip) / args.probe)
            cols.append(j)
        if len(cols) < 2:
            print("     ⚠ 야코비안을 못 쟀다 — 멈춘다.")
            return 1
        A = np.array(J).T
        sol, *_ = np.linalg.lstsq(A, d * args.gain, rcond=None)
        sol = np.clip(sol, -args.max_step, args.max_step)
        nxt = dict(base)
        for j, v in zip(cols, sol):
            nxt[j] += float(v)
        move(nxt)
        print("     " + " ".join("%s%+.1f" % (j.split("_")[0], v)
                                 for j, v in zip(cols, sol)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
