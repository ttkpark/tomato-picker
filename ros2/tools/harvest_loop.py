#!/usr/bin/env python3
"""**사람 없이 반복해서 잡는다** — 대기 자세 → 겨냥·전진·물기 → 놓기 → 반복.

    ~/lerobot/.venv/bin/python ros2/tools/harvest_loop.py
    ~/lerobot/.venv/bin/python ros2/tools/harvest_loop.py --hours 6 --rest 25

⚠ `tomato-voice`는 내리고. 시작할 때 한 번만 확인한다 — 도는 중에 살아나면
  실패가 반복되는 것으로 드러난다(로그에 찍힌다).

────────────────────────────────────────────────────────────────────────
왜 이렇게 만드나

`stem_grasp.py --aim top`은 이미 **클릭 없이** 열매를 스스로 찾는다(빨간
덩이 → 둥근 것만 → 윗변 위가 줄기). 그런데 사람이 자리를 비우면 열매가
떨어지거나 다 딴 뒤에는 찾을 표적이 없다 — 그래서 `stem_grasp.py`의
`tape_point()`가 **노란 테이프 아래 지점**을 대체 표적으로 준다(2026-09-03
추가, 실물 테이프로는 아직 확인 못 했다). 이 스크립트는 그 위에서 park→
grasp→open을 무인으로 돌리는 껍데기일 뿐, 겨냥·안전 로직은 전부
`stem_grasp.py`에 이미 있는 것을 그대로 쓴다(다시 구현하지 않는다).

⚠ 한 사이클이 실패해도(rc!=0) 죽지 않는다 — 다음 사이클에서 다시 선다.
  연속 실패가 쌓이면(기본 6회) **쉬는 시간을 늘린다** — 조명이 바뀌었거나
  카메라가 잠깐 죽은 것일 수 있는데, 그때마다 곧바로 다시 들이받으면
  같은 실패만 반복해 서보 온도만 올린다(실측 37~41°C, §서보_diag).
  10사이클마다도 한 번 길게 쉰다 — 계속 뻗고 접는 동작 자체의 발열 때문.

⚠ **"성공"(rc==0)이 늘 진짜 열매를 물었다는 뜻은 아니었다.** 2026-09-03:
  화소오차가 330→15까지 아주 매끄럽게 줄어 물었다고 보고했는데, 바깥
  카메라로 보니 열매 둘 다 그대로였고 팔은 화분 받침 높이(z≈118mm)까지
  내려가 있었다 — 화분틀 지지대를 문 것이다. **이동(추적→접근)은 문제가
  없었다. 색·모양만으로 고른 표적이 오래 추적하는 동안 지지대로 새어나갈
  수 있다는 게 문제다.** 그래서 `stem_grasp.py`가 **닫기 직전에 그 자리
  둘레가 정말 빨간지 마지막으로 확인**하게 고쳤다(`red_nearby`) — 아니면
  안 닫고 실패로 끝낸다. 이 무인 루프의 성공률은 여전히 낮을 수 있지만
  (실측: 실제 시도 중 대부분 실패, 성공은 드묾), **실패는 안전하게
  실패한다**는 것과 **성공으로 보고되면 진짜라는 것**은 이제 더 믿을 만하다.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
PARK = "60,65,0,-92,6"
# ⚠ wrist_flex -100 → -92 (2026-09-05). 팔 범위를 다시 잡은 뒤 wrist_flex 가동범위가
#   -102°까지라, -100은 정규값 -98을 넘어 arm_stage.py의 경로 안전검사에 걸린다
#   (실측: PARK·STAGE_CLOSE 둘 다 "wrist_flex한계"로 거절). 영점도 같이 다시 잡혀
#   이 문자열이 가리키는 물리 자세도 예전과 조금 다르다 — 열매가 보이는지는
#   tools/harvest_trials.py --preflight 로 확인할 것.
# ⚠ PARK은 곧게 편 특이점이다 — 거기서부터 겨냥(pan/lift/elbow)에 관절
#   범위를 다 쓰고 나면 정작 물 때까지 나아갈 elbow_flex가 안 남는다
#   (2026-09-03 실측: 16걸음에 elbow_flex 한계로 멈춤, 깊이 163mm에서 못
#   내려감). ⚠ 반대로 너무 굽혀도 안 된다 — 처음 골랐던 "59.6,35.7,39.8,
#   -100.5,0.0"은 elbow_flex를 이미 다 써서 겨냥 도중 관절이 얼어붙었다
#   (화소가 25걸음 내내 그대로 — "겨냥 못함" 없이도 실제로는 안 움직였다).
#   이 자세는 그 중간이다 — 실측으로 겨냥→깊이 351→136mm까지 매끄럽게
#   좁힌 적이 있다. ⚠ 열매 자리가 바뀌면 다시 찾아야 한다(§인수인계
#   2026-09-03) — 그래서 이게 안 보이면 PARK로 물러나 처음부터(더 멀지만
#   더 일반적으로) 다시 시도한다.
STAGE_CLOSE = "60,55,15,-92,6"      # wrist_flex -100 → -92, PARK과 같은 이유
STATE = os.path.expanduser("~/harvest_loop.json")


def run(argv, timeout):
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired as e:
        return 124, (e.stdout or "") + "\n(timeout)"


def save_state(**kw):
    kw["when"] = time.strftime("%Y-%m-%d %H:%M:%S")
    try:
        json.dump(kw, open(STATE, "w"), indent=1)
    except OSError:
        pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hours", type=float, default=6.0, help="이 시간이 지나면 멈춘다")
    ap.add_argument("--max-cycles", type=int, default=2000)
    ap.add_argument("--rest", type=float, default=25.0, help="사이클 사이 기본 휴식(초)")
    ap.add_argument("--cool-every", type=int, default=10, help="이 사이클마다 길게 쉰다")
    ap.add_argument("--cool-rest", type=float, default=180.0)
    ap.add_argument("--fail-rest", type=float, default=300.0,
                     help="연속 실패가 --fail-after 를 넘으면 이만큼 쉰다")
    ap.add_argument("--fail-after", type=int, default=6)
    ap.add_argument("--steps", type=int, default=30)
    ap.add_argument("--adv", type=float, default=8.0)
    ap.add_argument("--max-turn", type=float, default=2.0)
    ap.add_argument("--gain", type=float, default=0.35)
    ap.add_argument("--tol", type=float, default=35.0)
    ap.add_argument("--stop-z", type=float, default=88.0)
    ap.add_argument("--rejacobian", type=int, default=6)
    args = ap.parse_args()

    r = subprocess.run(["systemctl", "is-active", "tomato-voice"],
                        capture_output=True, text=True, timeout=4)
    if r.stdout.strip() == "active":
        print("⚠ tomato-voice 가 켜져 있다 — 팔 포트를 뺏겨 전부 실패한다. "
              "sudo systemctl stop tomato-voice 먼저.")
        return 1

    deadline = time.time() + args.hours * 3600.0
    cycles = ok = fail = consec_fail = 0
    print("무인 반복 시작 — 최대 %.1f시간 · %d사이클 · 사이클마다 최소 %.0f초 휴식"
          % (args.hours, args.max_cycles, args.rest))

    while time.time() < deadline and cycles < args.max_cycles:
        cycles += 1
        t0 = time.time()
        print("\n=== 사이클 %d (누적 성공 %d/%d) ===" % (cycles, ok, cycles - 1))

        def grasp_argv():
            return [PY, os.path.join(HERE, "stem_grasp.py"),
                    "--aim", "auto",
                    "--steps", str(args.steps),
                    "--adv", str(args.adv),
                    "--max-turn", str(args.max_turn),
                    "--gain", str(args.gain),
                    "--tol", str(args.tol),
                    "--stop-z", str(args.stop_z),
                    "--rejacobian", str(args.rejacobian)]

        # ── 1차: 이미 가까운 굽힌 자세에서 — 겨냥만 맞추면 된다 ──
        rc, out = run([PY, os.path.join(HERE, "arm_stage.py"), "--target=" + STAGE_CLOSE], 40)
        print(out.strip()[-300:])
        rc, out = run(grasp_argv(), 280) if rc == 0 else (1, "")
        print(out.strip()[-1200:])

        # ── 안 됐으면 2차: PARK에서 — 멀지만 더 일반적으로 다시 찾는다 ──
        if rc != 0:
            print("— 굽힌 자세에서 안 됐다. 대기 자세에서 다시 시도한다 —")
            rc, out = run([PY, os.path.join(HERE, "arm_stage.py"), "--target=" + PARK], 40)
            print(out.strip()[-300:])
            rc, out = run(grasp_argv(), 320) if rc == 0 else (1, "")
            print(out.strip()[-1200:])

        if rc == 0:
            ok, consec_fail = ok + 1, 0
            time.sleep(1.5)
            rc2, out2 = run([PY, os.path.join(HERE, "grip_set.py"), "78"], 20)
            print(out2.strip()[-200:])
            if rc2 != 0:
                print("⚠ 집게를 못 열었다 — 다음 사이클의 park가 다시 시도한다")
        else:
            fail, consec_fail = fail + 1, consec_fail + 1

        run([PY, os.path.join(HERE, "arm_stage.py"), "--target=" + PARK], 40)

        save_state(cycles=cycles, ok=ok, fail=fail, consec_fail=consec_fail)

        if consec_fail >= args.fail_after:
            print("⚠ 연속 실패 %d회 — %.0f초 길게 쉰다 (조명/카메라 확인 시간)"
                  % (consec_fail, args.fail_rest))
            time.sleep(args.fail_rest)
            consec_fail = 0
        elif cycles % args.cool_every == 0:
            print("— %d사이클마다 쉬는 시간 — %.0f초 (서보 발열)" % (args.cool_every, args.cool_rest))
            time.sleep(args.cool_rest)
        else:
            rest = max(0.0, args.rest - (time.time() - t0))
            time.sleep(rest)

    print("\n끝 — %d사이클 중 %d성공. 대기 자세로 돌아간다." % (cycles, ok))
    run([PY, os.path.join(HERE, "arm_stage.py"), "--target=" + PARK], 40)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
