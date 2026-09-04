#!/usr/bin/env python3
"""**잡기 실험을 표로 만든다** — 시작자세×겨냥방식 조합마다 N번 시도해 성공률을 잰다.

    python tools/harvest_trials.py --preflight            # 각 시작자세에서 열매가 보이나
    python tools/harvest_trials.py --trials 3             # 전체 조합 × 3회
    python tools/harvest_trials.py --only bent_auto --trials 5

PC(윈도우)에서 돈다 — 젯슨에는 SSH로 시키고, **바깥 웹캠(`tools/pc_cam.py`)으로
시도 전후를 찍어 남긴다.** 왜 바깥 카메라인가: 9/3 실측에서 `stem_grasp.py`가
rc=0("물었다")을 돌려줬는데 바깥에서 보니 열매 둘 다 그대로였고 화분 지지대를
물고 있었다. 스크립트의 자기 보고만으로는 성공을 못 믿는다 — 사진이 심판이다.

⚠ `tomato-voice`는 내린 채로(포트 8090 충돌·팔 포트 독점). `depth-cam`은 켜고.
⚠ 젯슨 IP는 DHCP라 바뀐다 — `--host`로 준다.

────────────────────────────────────────────────────────────────────────
무엇을 가르나

  시작자세  far(대기, 곧게 편 특이점 근처) / bent(굽힘) / bent2(더 굽힘)
            — 9/3 인수인계 §6: "마지막 40mm"가 막힌 건 대기자세가 특이점이라서였다.
              굽힌 자세면 10mm 가는 데 3.2°면 되는데 편 자세는 84°가 필요했다.
  겨냥      auto(색으로 한 번 찾고 그림조각 추적) / top(매 걸음 색으로 다시 찾음)

⚠ 시작자세 문자열은 2026-09-05 재캘리브레이션 기준이다(wrist_flex -100은 새
  범위(-102)에 붙어 arm_stage.py가 거절한다 → -92). 영점이 바뀌면 다시 재라.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..")

JET_PY = "~/lerobot/.venv/bin/python"
JET_REPO = "~/tomato-picker"

# ⚠ 2026-09-05 재캘리브레이션(범위+영점) 뒤의 값이다. 옛 문자열(60,65,0,-100,6 등)을
#   서보 정규값을 거쳐 새 도(度) 틀로 옮겼다 — 영점이 바뀌면 같은 숫자가 다른 물리
#   자세를 가리키기 때문(pan +27°, elbow -26°가 그 차이). wrist_roll은 0 = 앞을 볼 때
#   카메라가 수평인 자세(재홈 뒤). 영점을 또 잡으면 다시 옮겨야 한다.
POSES = {
    "far": "86.9,85.8,-26.3,-95,0",     # 대기 자세(PARK) — 곧게 편 쪽, 특이점 근처
    "bent": "86.9,75.8,-11.2,-95,0",    # 굽힌 시작자세(STAGE_CLOSE)
    "bent2": "86.9,65.7,3.8,-95,0",     # 더 굽힘
}
PARK = POSES["far"]

CONFIGS = {
    "far_auto":   dict(pose="far",   aim="auto"),
    "far_top":    dict(pose="far",   aim="top"),
    "bent_auto":  dict(pose="bent",  aim="auto"),
    "bent_top":   dict(pose="bent",  aim="top"),
    "bent2_auto": dict(pose="bent2", aim="auto"),
    "bent2_top":  dict(pose="bent2", aim="top"),
}
GRASP_ARGS = dict(steps=30, adv=8.0, max_turn=2.0, gain=0.35, tol=35.0,
                  stop_z=88.0, rejacobian=6)   # harvest_loop.py 기본값과 같다

# 직접 관절보간으로 대기자세로 — arm_stage.py의 경로검사가 큰 이동에서 자주
# 걸리므로(2026-09-04) 그게 거절하면 이걸로 돌아간다. 관절마다 8° 이하로 잘라
# 보간하니 한 번에 크게 뻗지 않는다.
PARK_FALLBACK = r'''
import sys, json, os, math, time
sys.path.insert(0, "src"); sys.path.insert(0, "ros2/tools"); sys.path.insert(0, "ros2/src/tomato_bridge")
import arm_calib
from tomato_picker.hardware import kinematics as kin
from tomato_bridge.follower_io import FollowerIO
c = arm_calib.Calib(); io = FollowerIO(hold_torque=True)
target = dict(zip(kin.JOINTS, [%s]))
cur = c.to_deg(io.read())
big = max(abs(target[j]-cur[j]) for j in kin.JOINTS)
steps = max(1, int(math.ceil(big/8.0)))
for s in range(1, steps+1):
    mid = {j: cur[j]+(target[j]-cur[j])*s/steps for j in kin.JOINTS}
    n = c.to_norm(mid); n["gripper"] = 78.0
    io.write(n, 0.8)
time.sleep(1.0)
got = c.to_deg(io.read())
print("park-fallback 도착", {j: round(got[j],1) for j in kin.JOINTS})
io.hold_close()
'''


def ssh(args, cmd: str, timeout: float) -> tuple[int, str]:
    full = ["ssh", "-i", args.key, "-o", "ConnectTimeout=8", "-o", "BatchMode=yes",
            f"server@{args.host}", cmd]
    try:
        r = subprocess.run(full, capture_output=True, timeout=timeout,
                           encoding="utf-8", errors="replace")
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b"")
        if isinstance(out, bytes):
            out = out.decode("utf-8", "replace")
        return 124, out + "\n(timeout)"


def jet(args, rel_cmd: str, timeout: float) -> tuple[int, str]:
    return ssh(args, f"cd {JET_REPO} && {rel_cmd}", timeout)


def snap(args, path: str) -> bool:
    """바깥 웹캠 한 장. 실패해도 실험은 계속 — 사진은 심판이지 전제조건이 아니다."""
    try:
        r = subprocess.run([sys.executable, os.path.join(HERE, "pc_cam.py"), path],
                           capture_output=True, timeout=40, encoding="utf-8", errors="replace")
        return r.returncode == 0
    except Exception:                                      # noqa: BLE001
        return False


def wrist(args, path: str) -> bool:
    try:
        r = subprocess.run(["curl", "-s", "--max-time", "6",
                            f"http://{args.host}:8090/frame.jpg", "-o", path],
                           capture_output=True, timeout=12)
        return r.returncode == 0 and os.path.getsize(path) > 1000
    except Exception:                                      # noqa: BLE001
        return False


def stage(args, pose: str) -> tuple[int, str]:
    rc, out = jet(args, f"{JET_PY} ros2/tools/arm_stage.py --target={pose}", 90)
    if rc != 0:
        rc2, out2 = jet(args, f"{JET_PY} - <<'PYEOF'\n{PARK_FALLBACK % pose}\nPYEOF", 90)
        out += "\n[arm_stage 거절 → 직접 관절보간]\n" + out2
        rc = rc2
    return rc, out


def grasp(args, cfg: dict) -> tuple[int, str, float]:
    g = GRASP_ARGS
    cmd = (f"{JET_PY} ros2/tools/stem_grasp.py --aim {cfg['aim']} --steps {g['steps']} "
           f"--adv {g['adv']} --max-turn {g['max_turn']} --gain {g['gain']} "
           f"--tol {g['tol']} --stop-z {g['stop_z']} --rejacobian {g['rejacobian']}")
    t0 = time.time()
    rc, out = jet(args, cmd, args.grasp_timeout)
    return rc, out, time.time() - t0


def parse(out: str) -> dict:
    """스크립트 출력에서 **왜 끝났는지**를 뽑는다 — rc 하나로는 모른다."""
    d = {}
    d["steps"] = len(re.findall(r"^\s+\d+\s+\(\s*-?\d+,\s*-?\d+\)", out, re.M))
    m = re.search(r"줄기가 (\d+)mm — 무는 거리", out)
    d["stop_depth_mm"] = int(m.group(1)) if m else None
    d["closed"] = "집게를 닫았다" in out
    if "겨눌 것을 못 찾았다" in out or "아무것도 못 찾았다" in out:
        d["why"] = "no_target"
    elif "둘레가 빨갛지 않다" in out:
        d["why"] = "red_check_fail"
    elif d["closed"]:
        d["why"] = "closed"
    elif "놓쳤다" in out:
        d["why"] = "lost_track"
    elif "어느 쪽으로도 가까워지지 않는다" in out:
        d["why"] = "blocked"
    elif "야코비안을 못 쟀다" in out:
        d["why"] = "no_jacobian"
    elif "더 안 간다" in out:
        d["why"] = "max_adv"
    elif "(timeout)" in out:
        d["why"] = "timeout"
    else:
        d["why"] = "steps_exhausted" if d["steps"] >= GRASP_ARGS["steps"] - 1 else "other"
    m = re.search(r"스스로 찾았다 — 열매 화면\((\d+),(\d+)\) 깊이(\d+)mm", out)
    d["fruit_at_start"] = [int(m.group(1)), int(m.group(2)), int(m.group(3))] if m else None
    return d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.6")
    ap.add_argument("--key", default=os.path.expanduser("~/.ssh/id_ed25519"))
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--only", default="", help="쉼표로 조합 이름을 골라 돌린다")
    ap.add_argument("--preflight", action="store_true",
                    help="움직이지 않고(--dry) 시작자세마다 열매가 보이는지만 본다")
    ap.add_argument("--rest", type=float, default=20.0)
    ap.add_argument("--cool-every", type=int, default=8)
    ap.add_argument("--cool-rest", type=float, default=120.0)
    ap.add_argument("--grasp-timeout", type=float, default=300.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    out_dir = args.out or os.path.join(os.environ.get("TEMP", "."), "harvest_trials",
                                       time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    print(f"결과 → {out_dir}")

    rc, o = jet(args, "systemctl is-active tomato-voice depth-cam click-server", 20)
    st = o.split()
    if len(st) >= 3 and (st[0] != "inactive" or st[1] != "active"):
        print(f"❌ 서비스 상태가 아니다 (tomato-voice={st[0]} depth-cam={st[1]}) — 멈춘다.")
        return 1

    names = [n.strip() for n in args.only.split(",") if n.strip()] or list(CONFIGS)

    if args.preflight:
        print("\n[사전점검] 시작자세마다 열매가 보이는가 (--dry, 겨냥만)")
        for pose_name in sorted({CONFIGS[n]["pose"] for n in names}):
            rc, o = stage(args, POSES[pose_name])
            if rc != 0:
                print(f"  {pose_name:<6} ❌ 시작자세로 못 감\n{o[-400:]}")
                continue
            rc, o = jet(args, f"{JET_PY} ros2/tools/stem_grasp.py --aim auto --dry", 60)
            m = re.search(r"지금: .*", o)
            found = re.search(r"스스로 찾았다.*", o)
            wrist(args, os.path.join(out_dir, f"preflight_{pose_name}.jpg"))
            print(f"  {pose_name:<6} {'✅' if rc == 0 else '❌'} "
                  f"{(found.group(0) if found else (m.group(0) if m else o.strip()[-200:]))}")
        stage(args, PARK)
        return 0

    log = open(os.path.join(out_dir, "results.jsonl"), "a", encoding="utf-8")
    n_done = 0
    try:
        for trial in range(args.trials):
            for name in names:
                cfg = CONFIGS[name]
                tag = f"{name}#{trial+1}"
                print(f"\n=== {tag} ===")
                rec = {"config": name, "trial": trial + 1, "pose": POSES[cfg["pose"]],
                       "aim": cfg["aim"], "when": time.strftime("%Y-%m-%d %H:%M:%S")}

                rc, o = stage(args, POSES[cfg["pose"]])
                rec["stage_rc"] = rc
                if rc != 0:
                    rec["why"] = "stage_fail"
                    print(f"  시작자세 실패\n{o[-300:]}")
                    log.write(json.dumps(rec, ensure_ascii=False) + "\n"); log.flush()
                    continue

                snap(args, os.path.join(out_dir, f"{tag}_before.jpg"))
                rc, o, secs = grasp(args, cfg)
                open(os.path.join(out_dir, f"{tag}.log"), "w", encoding="utf-8").write(o)
                snap(args, os.path.join(out_dir, f"{tag}_after.jpg"))
                wrist(args, os.path.join(out_dir, f"{tag}_wrist.jpg"))

                rec.update(parse(o))
                rec.update({"rc": rc, "secs": round(secs, 1)})
                rec["success"] = (rc == 0 and rec["closed"])
                print(f"  rc={rc} steps={rec['steps']} why={rec['why']} "
                      f"depth={rec['stop_depth_mm']} {secs:.0f}s "
                      f"{'✅ 물었다' if rec['success'] else '—'}")
                log.write(json.dumps(rec, ensure_ascii=False) + "\n"); log.flush()

                time.sleep(1.0)
                jet(args, f"{JET_PY} ros2/tools/grip_set.py 78", 30)
                rc, o = stage(args, PARK)
                if rc != 0:
                    print(f"  ⚠ 대기자세 복귀 실패 — 다음 시도의 시작자세가 대신 옮긴다\n{o[-200:]}")

                n_done += 1
                if n_done % args.cool_every == 0:
                    print(f"  — 서보 식히기 {args.cool_rest:.0f}s —")
                    time.sleep(args.cool_rest)
                else:
                    time.sleep(args.rest)
    except KeyboardInterrupt:
        print("\n중단 — 집게 열고 대기자세로")
    finally:
        jet(args, f"{JET_PY} ros2/tools/grip_set.py 78", 30)
        stage(args, PARK)
        log.close()

    # ── 표 ──
    rows = [json.loads(l) for l in open(os.path.join(out_dir, "results.jsonl"), encoding="utf-8")]
    print("\n조합           시도  성공  왜 끝났나")
    for name in names:
        rs = [r for r in rows if r["config"] == name]
        if not rs:
            continue
        ok = sum(1 for r in rs if r.get("success"))
        whys = {}
        for r in rs:
            whys[r.get("why", "?")] = whys.get(r.get("why", "?"), 0) + 1
        print(f"{name:<14} {len(rs):4}  {ok:4}  " + " ".join(f"{k}×{v}" for k, v in whys.items()))
    print(f"\n사진·로그: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
