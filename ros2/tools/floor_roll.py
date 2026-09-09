#!/usr/bin/env python3
"""D405 깊이로 **바닥 평면**을 맞춰 카메라의 롤(수평 기울기)을 잰다 — 눈 대신 숫자로.
    ~/lerobot/.venv/bin/python ros2/tools/floor_roll.py            (지금 프레임 한 번)
    ~/lerobot/.venv/bin/python ros2/tools/floor_roll.py --n 5      (5프레임 중앙값)
────────────────────────────────────────────────────────────────────────
왜 — wrist_roll이 몇 도 돌아가 있는지 컬러 화면만 보고는 못 정한다(바닥 널판 방향은
팔이 어디를 보느냐에 따라 달라서 기준이 못 된다). 바닥은 수평이니 **바닥 법선이
카메라 x축 성분을 갖지 않을 때**가 "수평"이다. 카메라 좌표: x=오른쪽, y=아래, z=앞.
법선 n=(nx,ny,nz)를 위쪽(카메라 쪽, ny<0)으로 맞춘 뒤 roll = atan2(nx, −ny) [도].
0이면 수평, 부호는 roll_step의 +/−와 실측으로 잇는다(카메라를 돌려 두 번 재면 안다).
⚠ 평면이 바닥이 아닐 수 있다(벽·화분·집게). RANSAC + 화면 아래 2/3 + 유효 깊이만
   쓰고, 내점 비율과 카메라-평면 거리를 같이 찍어 상식 검사한다.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

SHM = "/dev/shm"


def load(cam: str = "d405"):
    meta = json.load(open(os.path.join(SHM, f"{cam}_meta.json")))
    depth = np.load(os.path.join(SHM, f"{cam}_depth.npy"))
    return meta, depth


def fit(meta, depth, seed=0):
    it = meta["intrinsics"]
    fx, fy, cx, cy = it["fx"], it["fy"], it["ppx"], it["ppy"]
    scale = float(meta["depth_scale_mm"])
    h, w = depth.shape
    step = 6
    vs, us = np.mgrid[h // 3:h:step, 0:w:step]
    z = depth[vs, us].astype(np.float64) * scale
    ok = (z > meta.get("min_mm", 70.0)) & (z < meta.get("max_mm", 900.0))
    us, vs, z = us[ok].astype(np.float64), vs[ok].astype(np.float64), z[ok]
    x = (us - cx) / fx * z
    y = (vs - cy) / fy * z
    P = np.stack([x, y, z], axis=1)
    if len(P) < 200:
        return None
    rng = np.random.default_rng(seed)
    best = (0, None)
    for _ in range(300):
        idx = rng.choice(len(P), 3, replace=False)
        a, b, c = P[idx]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-6:
            continue
        n /= nn
        d = -n @ a
        dist = np.abs(P @ n + d)
        inl = int((dist < 6.0).sum())
        if inl > best[0]:
            best = (inl, (n, d))
    if best[1] is None:
        return None
    n, d = best[1]
    dist = np.abs(P @ n + d)
    Q = P[dist < 6.0]
    c0 = Q.mean(axis=0)
    _, _, vt = np.linalg.svd(Q - c0)
    n = vt[2]
    if n[1] > 0:          # 위쪽(카메라 쪽) 법선으로: 바닥을 내려다보면 ny<0
        n = -n
    d = -n @ c0
    roll = np.degrees(np.arctan2(n[0], -n[1]))
    tilt = np.degrees(np.arccos(min(1.0, -n[1])))     # 카메라 -y축과 법선 사이 각
    return dict(roll=float(roll), tilt=float(tilt), n=[float(v) for v in n],
                dist_mm=float(abs(d)), inlier=len(Q) / len(P), pts=len(P))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--cam", default="d405")
    args = ap.parse_args()
    rolls, last = [], None
    seq = None
    for _ in range(args.n):
        for _ in range(40):
            meta, depth = load(args.cam)
            if meta.get("seq") != seq:
                seq = meta.get("seq")
                break
            time.sleep(0.05)
        r = fit(meta, depth, seed=len(rolls))
        if r is None:
            print("⚠ 바닥 점이 부족하다(유효 깊이 없음)")
            continue
        last = r
        rolls.append(r["roll"])
        print(f"seq={seq} roll={r['roll']:+6.1f}°  tilt={r['tilt']:5.1f}°  "
              f"dist={r['dist_mm']:.0f}mm  inlier={r['inlier']:.2f} n={np.round(r['n'], 3)}")
    if not rolls:
        return 2
    med = float(np.median(rolls))
    print(f"ROLL {med:+.1f}")
    if last and last["inlier"] < 0.5:
        print("⚠ 내점 비율이 낮다 — 바닥이 아닌 평면일 수 있다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
