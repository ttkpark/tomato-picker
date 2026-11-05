#!/usr/bin/env python3
"""구석 토마토 접근 실험 — 자리를 바꿔 가며 N번 손목 근접을 시키고, 회차마다 숫자와 사진을 남긴다.
    ~/lerobot/.venv/bin/python ros2/tools/tomato_trials.py [--near-mm 450]
성공 = near가 정상으로 끝나고, 끝난 뒤 다시 잰 손목 거리 near-mm±50 · 가로 각 ≤ 15°(docs/체크포인트-2026-11.md 0차 기준).
결과 = ~/tomato_trials/<시각>/ (trials.json + 회차별 시작·끝 사진).
⚠ 뒤는 카메라가 없다 — 물러나는 자리는 사람이 비워 둔다. 대기 자세(회전판 기본 자리)에서 돌린다."""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "..", "src"), HERE]
import space_map as sm  # noqa: E402
from base_nudge import open_link  # noqa: E402

# (물러날 mm, 그 뒤 돌 각도 °, +=왼쪽) — 회마다 물러나서 시작한다(제자리면 이미 가까워 접근이 아니다)
STARTS = [(400, 0), (500, 20), (600, -20)]
SHOTS = ("d405_color.jpg", "d405_depth.jpg", "tomato_raw.jpg", "astra_depth.jpg")
OK_TOL_MM, OK_ANG = 50.0, 15.0


def snap(out: str, tag: str) -> None:
    for f in SHOTS:
        try:
            shutil.copy("/dev/shm/" + f, os.path.join(out, "%s-%s" % (tag, f)))
        except OSError as e:
            print("  사진 못 남김:", e)


def measure(n: int = 5):
    """손목 거리·가로 각 n번의 중앙값 → (mm, 각, 본 횟수). 못 보면 (nan, nan, 0)."""
    mm, ang = [], []
    for _ in range(n):
        g = sm.wrist_tomato(waiting_pose=True)
        if g is not None:
            ang.append(g[2])
            if g[1] == g[1]:
                mm.append(g[1])
        time.sleep(0.3)
    med = lambda v: sorted(v)[len(v) // 2] if v else float("nan")
    return med(mm), med(ang), len(ang)


def reposition(back: float, deg: float, w: int) -> dict:
    link = open_link()
    if link is None:
        raise RuntimeError("주행 보드를 못 열었다")
    try:
        r = sm.step_mm(link, -back, 220)
        print("  %.0fmm 물러남 → %s %d/%d" % (back, r["state"], r["got"], r["target"]))
        got = sm.turn(link, int(math.copysign(w, deg)), 0.8, abs(deg))
        print("  %+.0f° 돌기 → IMU %+.1f°" % (deg, got))
        return {"back": r, "turned": got}
    finally:
        link.close()


def search(w: int) -> bool:
    """토마토가 손목 화면에 없으면 양쪽으로 돌아 찾는다(최대 ±60°)."""
    link = open_link()
    if link is None:
        return False
    try:
        for deg in (30, 30, -90, -30):
            sm.turn(link, int(math.copysign(w, deg)), 0.8, abs(deg))
            time.sleep(sm.TURN_SETTLE_S)
            if sm.wrist_tomato(waiting_pose=True) is not None:
                return True
    finally:
        link.close()
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--near-mm", type=float, default=sm.NEAR_MM)
    ap.add_argument("--vx", type=int, default=150)
    ap.add_argument("--w", type=int, default=150)
    a = ap.parse_args()
    a.wsecs, a.vsecs = 0.8, 1.0
    m = sm.mount()
    out = os.path.expanduser("~/tomato_trials/" + time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out)
    rows = []
    for n, (back, deg) in enumerate(STARTS, 1):
        print("\n=== %d회차 — 물러나기 %dmm, 돌기 %+d° ===" % (n, back, deg), flush=True)
        t0, row = time.time(), {"n": n, "back": back, "deg": deg}
        if back or deg:
            row["reposition"] = reposition(back, deg, a.w)
        row["start"] = measure()
        snap(out, "%d-start" % n)
        print("  시작: 거리 %.0fmm · 각 %+.1f° · 5번 중 %d번 봄" % row["start"], flush=True)
        rc, tries = 1, 0
        for tries in range(1, 4):
            if sm.wrist_tomato(waiting_pose=True) is None and not search(a.w):
                print("  ⛔ 돌아봐도 토마토가 안 보인다")
                continue
            rc = sm.near_cmd(a, m)
            if rc == 0:
                break
            time.sleep(1.0)
        end = measure()
        snap(out, "%d-end" % n)
        ok = rc == 0 and abs(end[0] - a.near_mm) <= OK_TOL_MM and abs(end[1]) <= OK_ANG
        row.update(end=end, rc=rc, tries=tries, ok=ok, secs=round(time.time() - t0, 1))
        rows.append(row)
        print("  끝: 거리 %.0fmm · 각 %+.1f° · %d초 · 시도 %d번 → %s" % (
            end[0], end[1], row["secs"], tries, "성공" if ok else "실패"), flush=True)
        json.dump(rows, open(os.path.join(out, "trials.json"), "w"), ensure_ascii=False, indent=1)
    print("\n결과 %d/%d 성공 · %s" % (sum(r["ok"] for r in rows), len(rows), out))
    return 0 if all(r["ok"] for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
