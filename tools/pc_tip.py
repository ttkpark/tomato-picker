#!/usr/bin/env python3
"""바깥 화면에서 **집게 끝**과 **줄기 자리**를 찾는다 — 화소 좌표로.

    python tools/pc_tip.py                 # 찍고 찾아서 표시 이미지까지
    python tools/pc_tip.py .work/p14.jpg   # 이미 찍은 사진에서

⚠ PC에서 돈다. 로봇을 건드리지 않는다.

왜 이게 필요한가 — 손목 카메라는 집게가 향하는 쪽을 안 보고, 손-눈 회전을
국소로 재려던 시도도 깊이 잡음에 무너졌다(2026-09-03: 특이값 1.84/0.91,
잔차 20mm). 그런데 **바깥 화면에는 집게와 줄기가 같이 찍힌다.** 그러면 좁혀야
할 것이 3D 변환이 아니라 화면 위 벡터 하나가 되고, "위로 얼마·앞으로 얼마"는
`tool_jog.py --dz/--horiz`로 바로 명령할 수 있다. 둘의 화소↔mm 관계는
**아는 크기만큼 움직여 보고 재면** 된다 — 모델이 필요 없다.

⚠ 집게 끝은 색으로 찾지 않고 **열매에 가장 가까운 팔 화소**로 찾는다. 팔 전체가
  같은 색이라 "가장 오른쪽"은 자세가 바뀌면 팔꿈치를 가리킨다.
"""

from __future__ import annotations

import os
import subprocess
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def find(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    fruit = ((s > 90) & (v > 90) & ((h < 18) | (h > 168))).astype(np.uint8)
    fruit = cv2.morphologyEx(fruit, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    # ⚠ 집게가 열매 앞을 가로지르면 주황 덩이가 위아래로 쪼개진다 — 그러면
    #   가장 큰 덩이가 1436화소짜리 조각이 되어 "열매 없음"이 된다
    #   (2026-09-03). 크게 닫아 하나로 잇는다.
    fruit = cv2.morphologyEx(fruit, cv2.MORPH_CLOSE, np.ones((41, 41), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(fruit, 8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[i, cv2.CC_STAT_AREA] < 900:
        return None
    fx, fy = float(cen[i][0]), float(cen[i][1])
    ftop = float(st[i, cv2.CC_STAT_TOP])
    fw = float(st[i, cv2.CC_STAT_WIDTH])
    # 줄기 = 열매 **바로 위**(바깥 화면은 뒤집히지 않았다). 열매 지름 74mm를
    # 자로 삼아 20mm 위를 무는 자리로 본다.
    ppm = fw / 74.0
    stem = (fx, ftop - 20.0 * ppm)

    arm = ((h > 100) & (h < 140) & (s > 80) & (v > 40)).astype(np.uint8)
    arm = cv2.morphologyEx(arm, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n2, lab2, st2, cen2 = cv2.connectedComponentsWithStats(arm, 8)
    if n2 < 2:
        return None
    k = 1 + int(np.argmax(st2[1:, cv2.CC_STAT_AREA]))
    ys, xs = np.nonzero(lab2 == k)
    # ⚠ "줄기에 가장 가까운 팔 화소"는 집게 끝이 아니라 **팔 윤곽의 어깨**를
    #   짚는다(2026-09-03: 열매 옆을 지나는 몸통을 끝으로 읽었다).
    #   팔은 밑동이 고정돼 있으니 **밑동에서 가장 먼 화소**가 끝이다.
    root = np.array([float(xs.min()), float(ys.max())])      # 왼쪽 아래 = 밑동 쪽
    far = (xs - root[0]) ** 2 + (ys - root[1]) ** 2
    j = int(np.argmax(far))
    tip = (float(xs[j]), float(ys[j]))
    return dict(tip=tip, stem=stem, fruit=(fx, fy), ppm=ppm,
                fruit_px=int(st[i, cv2.CC_STAT_AREA]), arm_px=int(st2[k, cv2.CC_STAT_AREA]))


def main() -> int:
    if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
        path = sys.argv[1]
    else:
        path = os.path.join(HERE, "..", ".work", "_pt.jpg")
        subprocess.run([sys.executable, os.path.join(HERE, "pc_cam.py"), path],
                       capture_output=True, text=True, timeout=90)
    bgr = cv2.imread(path)
    if bgr is None:
        print("사진을 못 읽었다:", path)
        return 1
    r = find(bgr)
    if r is None:
        print("집게나 열매를 못 찾았다")
        return 1
    print("집게끝 (%.0f, %.0f) · 줄기 (%.0f, %.0f) · 남은 (%+.0f, %+.0f) 화소 "
          "· 배율 %.2f px/mm → (%+.0f, %+.0f) mm"
          % (r["tip"][0], r["tip"][1], r["stem"][0], r["stem"][1],
             r["stem"][0] - r["tip"][0], r["stem"][1] - r["tip"][1], r["ppm"],
             (r["stem"][0] - r["tip"][0]) / r["ppm"], (r["stem"][1] - r["tip"][1]) / r["ppm"]))
    o = bgr.copy()
    cv2.drawMarker(o, (int(r["tip"][0]), int(r["tip"][1])), (255, 0, 255),
                   cv2.MARKER_TILTED_CROSS, 26, 3)
    cv2.drawMarker(o, (int(r["stem"][0]), int(r["stem"][1])), (0, 255, 255),
                   cv2.MARKER_CROSS, 26, 3)
    cv2.line(o, (int(r["tip"][0]), int(r["tip"][1])),
             (int(r["stem"][0]), int(r["stem"][1])), (0, 255, 0), 2)
    out = os.path.join(HERE, "..", ".work", "_pt_mark.jpg")
    cv2.imwrite(out, o)
    print("표시:", os.path.normpath(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
