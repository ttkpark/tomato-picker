#!/usr/bin/env python3
"""시험 영상 mp4. 끝내기 = `<out>.stop` 파일을 만든다(또는 SIGTERM/Ctrl+C) — 어느 쪽이든 파일을 닫는다.
    젯슨: ~/.venvs/vision/bin/python tools/cam_record.py ~/follow_trials/x.mp4        앞 카메라(tomato_raw)+손목 D405, 5fps
    PC  : python tools/cam_record.py x.mp4 --pc 1                                    PC에 붙은 전경 카메라(DirectShow 번호), 10fps
⚠ PC 카메라가 검은 화면이면 Windows 카메라 앱이 쥐고 있는 것이다(11/09) — 닫고 다시."""
import os
import signal
import sys
import time

import cv2
import numpy as np

SRC = ("/dev/shm/tomato_raw.jpg", "/dev/shm/d405_color.jpg")
H = 360


def main(out: str, pc: int | None) -> None:
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    fps = 10 if pc is not None else 5
    cap = None
    if pc is not None:
        cap = cv2.VideoCapture(pc, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    w = size = None
    try:
        while not os.path.exists(out + ".stop"):
            t0 = time.time()
            if cap is not None:
                ok, f = cap.read()
                f = f if ok else None
            else:
                ims = [cv2.imread(p) for p in SRC]
                f = None if any(i is None for i in ims) else \
                    np.hstack([cv2.resize(i, (i.shape[1] * H // i.shape[0], H)) for i in ims])   # 쓰는 중인 장은 None
            if f is not None:
                cv2.putText(f, time.strftime("%H:%M:%S"), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
                if w is None:
                    size = (f.shape[1], f.shape[0])
                    w = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
                if (f.shape[1], f.shape[0]) == size:     # 크기가 다른 장은 VideoWriter가 조용히 버린다 — 여기서 거른다
                    w.write(f)
            time.sleep(max(0.0, 1.0 / fps - (time.time() - t0)))
    finally:
        if w is not None:
            w.release()
        if cap is not None:
            cap.release()
        if os.path.exists(out + ".stop"):
            os.remove(out + ".stop")


if __name__ == "__main__":
    a = sys.argv[1:]
    main(a[0], int(a[a.index("--pc") + 1]) if "--pc" in a else None)
