#!/usr/bin/env python3
"""이 PC(윈도우)에 붙은 웹캠으로 한 장 찍는다 — **밖에서 로봇을 본다.**

    python tools/pc_cam.py shot.jpg
    python tools/pc_cam.py shot.jpg --index 1 --list

⚠ 로봇의 카메라(D405)가 아니다. 그쪽은 젯슨에 붙어 있고 손목에 달려 있어서
   **자기 자신을 못 본다.** 팔이 의도대로 움직였는지, 케이블이 감기지 않았는지,
   표적이 제자리에 있는지는 밖에서 봐야 안다.

⚠ 윈도우에서는 DirectShow 백엔드를 명시해야 한다. 기본(MSMF)은 첫 프레임까지
   몇 초씩 걸리거나 조용히 검은 화면을 준다.

⚠ 첫 몇 장은 버린다. 자동노출·자동초점이 자리를 잡기 전 프레임은 어둡거나
   흐리다 — 그걸 보고 "조명이 어둡다"고 판단하면 엉뚱한 데를 고치게 된다.
"""

from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np

WARMUP = 20


def probe(limit: int = 5) -> list[tuple[int, int, int]]:
    """열리는 카메라 번호와 해상도를 훑는다."""
    found = []
    for i in range(limit):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                found.append((i, frame.shape[1], frame.shape[0]))
        cap.release()
    return found


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default="shot.jpg")
    ap.add_argument("--index", type=int, default=-1,
                    help="-1이면 까맣지 않은 첫 카메라를 고른다 "
                         "(이 PC의 0번은 열리지만 검은 화면만 준다)")
    ap.add_argument("--list", action="store_true", help="쓸 수 있는 카메라만 훑는다")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--warmup", type=int, default=WARMUP)
    args = ap.parse_args()

    if args.list:
        for i, w, h in probe():
            print(f"  카메라 {i}: {w}x{h}")
        return 0

    index = args.index
    if index < 0:
        # ⚠ "열렸다"가 "찍힌다"는 뜻이 아니고, "안 까맣다"가 "진짜 카메라"라는
        #    뜻도 아니다. 이 PC에는 셋이 잡히는데 0번은 까맣고, OBS 가상 카메라는
        #    "카메라 꺼짐" **정지 그림**을 멀쩡한 밝기로 내놓는다. 그래서 밝기만
        #    보면 로봇 대신 OBS 로고를 찍는다(2026-09-01 실측).
        #    진짜 카메라는 센서 잡음 때문에 연속 두 장이 미세하게 다르다.
        best = (-1.0, -1)
        for i in range(5):
            c = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            a = b = None
            if c.isOpened():
                for k in range(16):
                    ok, f = c.read()
                    if ok and f is not None:
                        a = b
                        b = f
            c.release()
            if a is None or b is None:
                continue
            bright = float(cv2.cvtColor(b, cv2.COLOR_BGR2GRAY).mean())
            live = float(np.abs(a.astype(np.int16) - b.astype(np.int16)).mean())
            print(f"  카메라 {i}: 밝기 {bright:.0f} · 프레임 변화 {live:.2f}",
                  file=sys.stderr)
            if bright > 8 and live > 0.3 and live > best[0]:
                best = (live, i)
        index = best[1]
        if index < 0:
            print("살아 있는 카메라를 못 찾았다 (까맣거나 정지 화면뿐).",
                  file=sys.stderr)
            return 1

    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"카메라 {index}를 못 열었다. 쓸 수 있는 것:", file=sys.stderr)
        for i, w, h in probe():
            print(f"  {i}: {w}x{h}", file=sys.stderr)
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    frame = None
    for _ in range(max(1, args.warmup)):
        ok, f = cap.read()
        if ok and f is not None:
            frame = f
    cap.release()

    if frame is None:
        print("프레임을 못 읽었다 — 다른 앱이 카메라를 쥐고 있을 수 있다.",
              file=sys.stderr)
        return 1
    cv2.imwrite(args.out, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    print(f"{args.out}  {frame.shape[1]}x{frame.shape[0]}  "
          f"밝기 {g.min()}/{g.mean():.0f}/{g.max()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
