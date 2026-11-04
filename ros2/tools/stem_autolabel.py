#!/usr/bin/env python3
"""**노란 띠 사진 → 줄기 검출 학습 데이터(YOLO)** — 사람이 그리지 않는다.

    python ros2/tools/stem_autolabel.py data/stem_raw data/stem_yolo      # 라벨 + 띠 지운 사진 + 확인용 모음
    python ros2/tools/stem_autolabel.py --selftest

띠는 줄기에 감은 것이라 **띠 자리 = 줄기 자리**다. 다만 모델이 "노랑 = 줄기"로 배우면 띠 없는 실기에서 못 쓴다 →
띠 화소를 둘레 색으로 메워(inpaint) 지운 사진을 학습에 쓴다. 상자는 띠 하나가 아니라 **띠 위아래로 이어진
줄기 토막 전체**(꼭지 위 ~ 고리 허브 아래)로 잡는다 — 띠 없는 줄기도 그 모양으로 알아보게.
무는 높이는 상자 안 비율로 남긴다(`grip_ratio` — 상자 위에서 띠 가운데까지 / 상자 높이).

⚠ 색만 쓴다(사진엔 깊이가 없다). 토마토 하이라이트도 노랑 문턱을 넘으므로(11/02) 띠 후보는
'세로로 길쭉하지 않고 · 채도 높고 · 위아래로 채도 낮은 줄기 화소가 이어지는' 덩어리만 받는다.
⚠ 확인용 모음(`_check.jpg`)을 **사람이 한 번 훑어볼 것** — 자동 라벨이 틀리면 모델이 틀리게 배운다.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import sys

import numpy as np

H_LO, H_HI, S_MIN, V_MIN = 24, 40, 120, 50        # grip_uv_measure.band_uvz와 같은 노랑
# 줄기 테이프 = **회녹색**: 11/03 실측 H≈54(34~68)·S≈52·V≈107. 채도만 보면 베이지 벽(H≈20, S≈30, V>190)도 걸린다.
STEM_H, STEM_S, STEM_V = (30, 95), (15, 110), (40, 190)


def yellow_mask(hsv):
    import cv2
    m = ((hsv[:, :, 0] > H_LO) & (hsv[:, :, 0] < H_HI) & (hsv[:, :, 1] > S_MIN) & (hsv[:, :, 2] > V_MIN))
    return cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))


def find_band(hsv):
    """띠 (x0, y0, x1, y1) 또는 None. 집게가 가운데를 가려 두 조각이 될 수 있어 가로가 겹치는 조각을 합친다."""
    import cv2
    m = yellow_mask(hsv)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    cands = [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= 150]
    if not cands:
        return None
    best = max(cands, key=lambda i: st[i, cv2.CC_STAT_AREA])
    x, y, w, h = (int(st[best, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    if w > 3.0 * h + 40 or w > 140:                    # 넓적한 노랑 = 토마토 하이라이트·바닥
        return None
    x0, y0, x1, y1 = x, y, x + w, y + h
    for i in cands:                                    # 같은 기둥의 다른 조각(집게에 끊긴 아래쪽)
        if i == best:
            continue
        xi, yi, wi, hi = (int(st[i, k]) for k in (0, 1, 2, 3))
        if min(x1, xi + wi) - max(x0, xi) > 0.5 * min(w, wi) and abs(yi - y1) < 60:
            x0, y0, x1, y1 = min(x0, xi), min(y0, yi), max(x1, xi + wi), max(y1, yi + hi)
    return x0, y0, x1, y1


def stem_extent(hsv, band, max_px=160):
    """띠 위아래로 줄기(채도 낮은 화소가 띠 폭 안에 절반 이상)가 이어지는 한 늘린다 → (y_top, y_bot)."""
    x0, y0, x1, y1 = band
    cx, hw = (x0 + x1) // 2, max(4, (x1 - x0) // 2)
    a, b = max(0, cx - hw), min(hsv.shape[1], cx + hw + 1)

    def stemmy(y):
        h, s, v = hsv[y, a:b, 0], hsv[y, a:b, 1], hsv[y, a:b, 2]
        ok = ((h >= STEM_H[0]) & (h <= STEM_H[1]) & (s >= STEM_S[0]) & (s <= STEM_S[1])
              & (v >= STEM_V[0]) & (v <= STEM_V[1]))
        return float(np.mean(ok)) >= 0.5

    top, miss = y0, 0
    for y in range(y0 - 1, max(-1, y0 - max_px), -1):
        miss = 0 if stemmy(y) else miss + 1
        if miss > 4:
            break
        top = y
    bot, miss = y1, 0
    for y in range(y1 + 1, min(hsv.shape[0], y1 + max_px)):
        miss = 0 if stemmy(y) else miss + 1
        if miss > 4:
            break
        bot = y
    return top, bot


def label_one(bgr):
    """사진 한 장 → (띠 지운 사진, YOLO 상자 (cx, cy, w, h) 0~1, grip_ratio) 또는 None."""
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    band = find_band(hsv)
    if band is None:
        return None
    x0, y0, x1, y1 = band
    top, bot = stem_extent(hsv, band)
    if bot - top < (y1 - y0) + 10:                     # 위아래로 줄기가 안 이어진다 = 띠가 아닐 수 있다
        return None
    pad = max(3, (x1 - x0) // 4)
    bx0, bx1 = max(0, x0 - pad), min(bgr.shape[1] - 1, x1 + pad)
    m = cv2.dilate(yellow_mask(hsv), np.ones((7, 7), np.uint8))
    clean = cv2.inpaint(bgr, m, 5, cv2.INPAINT_TELEA)
    H, W = bgr.shape[:2]
    box = ((bx0 + bx1) / 2 / W, (top + bot) / 2 / H, (bx1 - bx0) / W, (bot - top) / H)
    ratio = ((y0 + y1) / 2 - top) / max(1.0, bot - top)
    return clean, box, ratio


def build(src, dst, val_frac=0.15, seed=0):
    import cv2
    files = sorted(f for f in os.listdir(src) if f.lower().endswith(".jpg"))
    random.Random(seed).shuffle(files)
    ok, ratios, thumbs = 0, [], []
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(dst, sub), exist_ok=True)
    for k, f in enumerate(files):
        bgr = cv2.imread(os.path.join(src, f))
        if bgr is None:
            continue
        r = label_one(bgr)
        if r is None:
            continue
        clean, (cx, cy, w, h), ratio = r
        part = "val" if k < val_frac * len(files) else "train"
        stem = os.path.splitext(f)[0]
        cv2.imwrite(os.path.join(dst, "images", part, stem + ".jpg"), clean)
        with open(os.path.join(dst, "labels", part, stem + ".txt"), "w") as fh:
            fh.write("0 %.6f %.6f %.6f %.6f\n" % (cx, cy, w, h))
        ok += 1
        ratios.append(ratio)
        if len(thumbs) < 48:
            Hh, Ww = clean.shape[:2]
            v = clean.copy()
            p0 = (int((cx - w / 2) * Ww), int((cy - h / 2) * Hh))
            p1 = (int((cx + w / 2) * Ww), int((cy + h / 2) * Hh))
            cv2.rectangle(v, p0, p1, (0, 0, 255), 2)
            gy = int(p0[1] + ratio * (p1[1] - p0[1]))
            cv2.line(v, (p0[0] - 10, gy), (p1[0] + 10, gy), (255, 0, 255), 2)
            thumbs.append(cv2.resize(v, (212, 120)))
    with open(os.path.join(dst, "data.yaml"), "w") as fh:
        fh.write("path: %s\ntrain: images/train\nval: images/val\nnames:\n  0: stem\n" % os.path.abspath(dst))
    meta = {"n": ok, "of": len(files), "grip_ratio": float(np.median(ratios)) if ratios else None}
    json.dump(meta, open(os.path.join(dst, "autolabel.json"), "w"), indent=1)
    if thumbs:
        while len(thumbs) % 6:
            thumbs.append(np.zeros_like(thumbs[0]))
        rows = [np.hstack(thumbs[i:i + 6]) for i in range(0, len(thumbs), 6)]
        cv2.imwrite(os.path.join(dst, "_check.jpg"), np.vstack(rows))
    print("라벨 %d / %d장 · 무는 높이 비율 중앙값 %s → %s" % (ok, len(files), meta["grip_ratio"], dst))
    return meta


def selftest():
    import cv2
    img = np.full((480, 848, 3), (200, 200, 200), np.uint8)          # 회색 벽
    cv2.rectangle(img, (410, 100), (430, 300), (80, 115, 95), -1)     # 회녹색 줄기(H≈50, S≈77)
    cv2.rectangle(img, (408, 180), (432, 210), (0, 220, 230), -1)     # 노란 띠
    cv2.circle(img, (420, 380), 70, (40, 120, 240), -1)               # 주황 토마토(채도 높음 → 줄기 아님)
    r = label_one(img)
    assert r is not None
    clean, (cx, cy, w, h), ratio = r
    H, W = img.shape[:2]
    top, bot = (cy - h / 2) * H, (cy + h / 2) * H
    assert abs(cx * W - 420) < 4 and top < 120 and 290 < bot < 312, (cx * W, top, bot)
    assert 0.3 < ratio < 0.6, ratio
    hsv = cv2.cvtColor(clean, cv2.COLOR_BGR2HSV)
    assert yellow_mask(hsv).sum() == 0                               # 띠가 지워졌다
    flat = np.full((480, 848, 3), (200, 200, 200), np.uint8)
    cv2.rectangle(flat, (300, 300), (600, 330), (0, 220, 230), -1)    # 넓적한 노랑(하이라이트) → 버린다
    assert label_one(flat) is None
    print("selftest OK")
    return 0


def main():
    if "--selftest" in sys.argv:
        return selftest()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src")
    ap.add_argument("dst")
    a = ap.parse_args()
    build(a.src, a.dst)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
