#!/usr/bin/env python3
"""체크포인트 0차 — 따라가기 판정 3회: 지도 지우기 → 스캔 → 찾기 → 앞으로 → 손목 근접. 일은 전부 조작대 API(/run)로 시킨다.
    ~/lerobot/.venv/bin/python ros2/tools/follow_trials.py
성공 = 근접 종료 450±50mm · 손목 화면 가운데 ±15° · 충돌 0(사진으로 본다) · 5분 안. 지도는 회마다 지운다(옛 토마토를 쓰지 않게).
결과 = ~/follow_trials/<시각>/table.md + 회차별 로그·사진. 실패 종류: 못 찾음 / 접근 실패 / 시간초과.
⚠ 출발 자리로 옮길 때만 주행 포트를 직접 쓴다(측정 구간 밖). 뒤는 카메라가 없다 — 물러나는 자리는 비어 있어야 한다."""
from __future__ import annotations

import json
import os
import shutil
import sys
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "..", "src"), HERE]
import space_map as sm  # noqa: E402
import tomato_trials as tt  # noqa: E402

API = "http://127.0.0.1:8090"
LIMIT_S = 300.0
STARTS = [(300, 60), (200, -70), (300, 120)]     # (물러날 mm, 그 뒤 돌 °, +=왼쪽) — 토마토가 화면 밖에 있게 돌려 놓는다
STEPS = [("clear", {}), ("scan", {"w": 150}), ("find", {"w": 150}),
         ("approach", {"w": 150, "vx": 200}), ("near", {"w": 150, "vx": 150})]
FAIL = {"scan": "못 찾음", "find": "못 찾음", "approach": "접근 실패", "near": "접근 실패"}


def call(path, body=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(API + path, data, {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        return json.load(e)


def job(mode, args, deadline):
    """조작대에 일을 시키고 끝날 때까지 본다 → (코드, 그 일의 로그). 기한을 넘기면 끊고 코드 None."""
    start = call("/log?tail=1")["next"]
    r = call("/run", {"job": "space", "args": dict(args, mode=mode)})
    if not r.get("ok"):
        return 1, ["조작대 거절: %s" % r.get("why")]
    timed_out = False
    while True:
        time.sleep(1.0)
        st = call("/state")
        if not st["running"]:
            break
        if time.time() > deadline:
            timed_out = True
            call("/stop", {})
            deadline = float("inf")
    lines = call("/log?since=%d" % start)["lines"]
    return (None if timed_out else st["rc"]), lines


def main() -> int:
    out = os.path.expanduser("~/follow_trials/" + time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out)
    if os.path.exists(sm.MAP):
        shutil.copy(sm.MAP, os.path.join(out, "map-before.npz"))
    rows = []
    for n, (back, deg) in enumerate(STARTS, 1):
        print("\n=== %d회차 — %dmm 물러나 %+d° ===" % (n, back, deg), flush=True)
        tt.reposition(back, deg, 150)
        time.sleep(1.5)
        t0, kind = time.time(), ""
        tt.snap(out, "%d-start" % n)
        with open(os.path.join(out, "%d.log" % n), "w") as lf:
            for mode, args in STEPS:
                rc, lines = job(mode, args, t0 + LIMIT_S)
                lf.write("\n## %s rc=%s\n%s\n" % (mode, rc, "\n".join(lines)))
                lf.flush()
                last = lines[-2] if len(lines) > 1 else (lines[0] if lines else "")
                print("  %s rc=%s · %.0f초 · %s" % (mode, rc, time.time() - t0, last), flush=True)
                if rc is None:
                    kind = "시간초과"
                elif rc != 0:
                    kind = FAIL.get(mode, "접근 실패")
                if kind:
                    break
        secs = time.time() - t0
        end = tt.measure()
        tt.snap(out, "%d-end" % n)
        if not kind and not (abs(end[0] - sm.NEAR_MM) <= tt.OK_TOL_MM and abs(end[1]) <= tt.OK_ANG):
            kind = "접근 실패"
        rows.append((n, "%dmm 뒤, %+d°" % (back, deg), "실패" if kind else "성공", kind or "-",
                     "%.0f" % secs, "%.0f" % end[0], "%+.1f" % end[1]))
        print("  → %s %s · %.0f초 · 끝 %.0fmm %+.1f°" % (rows[-1][2], kind, secs, end[0], end[1]), flush=True)
        with open(os.path.join(out, "table.md"), "w") as f:
            f.write("| 회 | 출발 | 결과 | 실패 종류 | 소요초 | 근접 종료 거리 | 가로 각 |\n|---|---|---|---|---|---|---|\n")
            f.writelines("| " + " | ".join(map(str, r)) + " |\n" for r in rows)
    print("\n결과 %d/%d · %s" % (sum(r[2] == "성공" for r in rows), len(rows), out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
