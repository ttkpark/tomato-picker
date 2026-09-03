#!/usr/bin/env python3
"""손-눈 보정(`on_arm`)을 **처음부터 끝까지 스스로** — 이동·촬영·풀이·저장.

    ~/lerobot/.venv/bin/python ros2/tools/handeye_collect.py --dry    (계산만)
    ~/lerobot/.venv/bin/python ros2/tools/handeye_collect.py          (수집+풀이)
    ~/lerobot/.venv/bin/python ros2/tools/handeye_collect.py --save   (좋으면 저장)

⚠ `tomato-voice.service`를 내린 채로 돌린다 — 팔 포트는 한 프로세스만 연다.
   (그 서비스는 연결할 때 토크를 끄므로, 중간에 켜면 팔이 늘어진다.)

────────────────────────────────────────────────────────────────────────
왜 대시보드를 안 쓰고 여기서 다 하는가 — 세 가지가 한꺼번에 걸렸다.

① **좌표 이동이 막힌다.** `cartesian`은 지금 자세의 `signed_radius`가 90mm보다
   작으면 어떤 이동도 거절한다. 카메라가 표적을 보려면 팔이 접혀야 하고, 그
   자세의 `signed_radius`는 20mm 언저리다. 옳은 가드지만 여기서는 길을 막는다
   → **관절 공간으로 움직인다.**

② **뻗으면 서보가 놓는다.** 2026-08-31 실측: r≈260mm로 뻗다가 pitch 50° 부근에서
   **"갑자기 힘이 추욱"** 빠졌다(과부하 보호). 접힌 영역은 중력 모멘트가 작다
   → **접힌 채로 움직인다.**

③ **접힌 자세에서는 레거시의 도구 좌표계가 틀릴 수 있다.** `handeye.tool_frame`은
   방위각을 `atan2(y, x)`로 되찾는데 `signed_radius`가 음수면 180° 뒤집힌다
   → **관절값 `pan`으로 직접 만든다**(`tool_frame_from_joints`).

────────────────────────────────────────────────────────────────────────
자세를 어떻게 흩는가 — **회전만으로는 안 된다** (2026-08-31에 배운 것)

처음엔 `pan`과 `wrist_flex`만 흔들었다. 결과는 잔차 64mm에 `t_tool_cam`이
−575mm(팔 전체가 346mm인데!). 원인:

  **카메라가 pan 축 바로 위에 올라앉아 있다.** 접힌 자세의 TCP 수평거리가
  15mm뿐이라, pan을 ±28° 돌려도 카메라는 거의 제자리에서 **돌기만** 한다.
  실측이 그걸 말했다 — pan을 48° 돌리는 동안 카메라↔표적 거리가 5mm밖에
  안 변했다. 그러면 `t_x`가 관측되지 않고, 선형해의 `R_x`가 회전에서 멀어져
  SO(3) 투영에서 깨지며, **잔차까지 커진다.**

그래서 축을 넷으로 늘렸다. 앞의 둘은 회전, 뒤의 둘은 **평행이동**이다:

  · `pan`    — base z축 회전 (중력 모멘트가 안 변한다 = 공짜로 안전)
  · `pitch`  — wrist_flex 로 만드는 팔 평면 회전
  · `lift`   — 어깨를 굽혀 카메라를 **옮긴다** (pitch는 wrist_flex로 되돌린다)
  · `elbow`  — 팔꿈치를 굽혀 카메라를 **옮긴다** (마찬가지로 되돌린다)

`lift`/`elbow`를 움직이면 pitch가 따라 변하므로 `wrist_flex`로 그만큼 빼서
**자세는 그대로 두고 위치만** 바꾼다. 그게 `t_x`를 보이게 하는 유일한 길이다.

⚠ `wrist_roll`은 **고정**한다. 레거시가 도구 좌표계에서 roll을 빼고 계산하므로,
   안 건드리면 두 계통이 같은 답을 낸다. 돌리는 순간 갈린다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))
sys.path.insert(0, HERE)

import target_check as tc  # noqa: E402
from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware.handeye import (  # noqa: E402
    CalibrationError, Rigid, solve_on_arm,
)

DEG_PER_TICK = 360.0 / 4096.0
CAL = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json")
CART = os.path.expanduser("~/arm_cartesian.json")
# 조작대(click_server.py)의 3D 미리보기가 여기서 읽어 실시간으로 따라 그린다.
# ⚠ 이 스크립트는 SSH로 직접 돌아 조작대의 "일" 체계(JOB.lines/log)를
#   안 거친다 — 그래서 로그를 긁어 지금 자세를 얻는 기존 방식(k3dParseLog)이
#   안 통한다. 파일로 직접 남긴다(2026-09-03).
POSE_VIEW = os.path.expanduser("~/arm_pose_view.json")

STEP_DEG = 10.0        # 이동 중 한 구간에서 어느 관절도 이 이상 안 움직인다
SETTLE_SEC = 1.2
# 단방향 접근에서 뒤로 물러났다 오는 크기(도) — 백래시(실측 ~2°)보다 커야 한다.
BACKOFF_DEG = 6.0       # 멈춘 뒤 이만큼 기다렸다 찍는다 (흔들림이 곧 잔차다)
MAX_TRACK_ERR = 12.0   # 지령과 실제가 이만큼 벌어지면 중단 (서보가 놓은 것)
# 바닥은 팔 base(마운트)보다 이만큼 아래에 있다 — 실측 76.5mm
# (`ros2/src/tomato_description/config/so101_geometry.yaml` 의 mount.z 와 같은 값).
# ⚠ 예전에는 "지금 자리보다 1mm 아래"를 바닥으로 삼았다. 그러면 팔이 낮게
#   늘어져 있을 때 **1.8mm 내려갔다 다시 오르는 정상 경로까지 막혀** 빠져나올
#   수가 없다(2026-09-01, 복구 불가 상태로 두 번 갇혔다). 바닥은 팔이 어디
#   있느냐와 무관한 값이다.
MOUNT_Z_MM = 76.5
FLOOR_MARGIN_MM = 10.0
MAX_TCP_R = 200.0      # 이보다 뻗지 않는다 — 과부하 보호가 걸린 영역을 피한다
EDGE_MARGIN = 10.0     # 화면 가장자리 여유(px) — 점 네 개가 다 들어와야 한다
PROBE_DEG = 8.0        # 야코비안 시험각
W, H = 848.0, 480.0

# 후보 격자. 앞 둘은 회전(도), 뒤 둘은 평행이동을 만드는 관절 변화(도).
# ⚠ lift 범위가 좁으면 **카메라가 안 움직인다** — 2026-08-31에 그것 때문에
#   기선이 32mm뿐이었고 보정이 안 풀렸다. 표적을 가로로 눕혀 화면 여유가
#   생겼으므로 lift를 넓게 쓴다(카메라 이동 폭 ~110mm를 노린다).
# ⚠ 격자는 **지금 자세를 중심으로 대칭일 필요가 없다.** 2026-08-31: 기준 자세의
#   elbow가 한계(191.2°)에 붙어 있어 ELBOW_GRID 셋 중 -16만 통과했고, 그 -16이
#   표적을 화면 위로 100px 밀어 pitch가 쓸 세로 여유를 다 먹었다 — 결국 12자세가
#   전부 같은 pitch였다. **회전축이 하나뿐인 표본으로는 eye-in-hand가 안 풀린다**
#   (잔차 41.5mm, 카메라 위치 -858mm라는 헛된 답이 나왔다). 한계에 붙은 관절은
#   한쪽으로만 뻗어야 하고, 그만큼 pitch도 같은 쪽으로 따라가야 한다.
# ⚠ 2026-09-03 실측 두 차례:
#   1차 — pan=±28°에서 13개 중 9개를 놓쳤다(그중 7개가 pan=-28). pan=-14인
#     것만 4개 다 살았다. 회전(pan·pitch)은 화면 밖으로 나가는 값이라 좁히고,
#     평행이동(lift·elbow)은 정면을 유지한 채 기선만 벌리므로 넓혔다.
#   2차 — 그런데도 lift는 **+8°짜리 야코비안 시험각에서마저** 표적을 잃어
#     반대 부호로 다시 쟀다(비대칭). pitch는 음수 쪽만 전부 실패했다(성공한
#     6개가 전부 양수). lift는 다시 좁히고, pitch는 음수를 줄이고 양수 쪽으로
#     기울였다.
PAN_GRID = (-18.0, -9.0, 0.0, 9.0, 18.0)
PITCH_GRID = (-10.0, -3.0, 4.0, 11.0, 18.0)
LIFT_GRID = (-20.0, -10.0, 0.0, 10.0, 20.0)
ELBOW_GRID = (-45.0, -30.0, -15.0, 0.0)
# 접근축 둘레 회전. 표적을 화면에서 **회전만** 시키므로 여유를 거의 안 먹는데,
# t_x(카메라가 도구에서 얼마나 벗어나 있는가)를 가르는 것은 이 축이다.
ROLL_GRID = (-60.0, -30.0, 0.0, 30.0, 60.0)
ROLL_SIGN = 1.0        # 접근축 둘레 회전의 부호 — 실측으로 정한다(해가 더 잘 맞는 쪽)
WANT_POSES = 16


def tool_frame_from_joints(degs: dict, geom: kin.ArmGeometry) -> Rigid:
    """관절각 → `T_base_tool`. **`handeye.tool_frame`을 대신한다.**

    같은 규약(열이 approach·lateral·up)을 쓰되 방위각을 `atan2(y, x)`가 아니라
    관절값 `shoulder_pan` 그대로 쓴다. 팔이 몸통 위로 접혀 `signed_radius`가
    음수가 되면 FK의 (x, y)는 방위각이 180° 뒤집힌 같은 점으로 보이고, `atan2`는
    그 뒤집힌 값을 돌려준다 — 위치는 맞지만 **회전이 거울상이 된다.**
    """
    pan = math.radians(degs["shoulder_pan"])
    a3 = math.radians(degs["shoulder_lift"] + degs["elbow_flex"] + degs["wrist_flex"])
    approach = np.array([math.cos(a3) * math.cos(pan),
                         math.cos(a3) * math.sin(pan), math.sin(a3)])
    lateral = np.array([-math.sin(pan), math.cos(pan), 0.0])
    up = np.array([-math.sin(a3) * math.cos(pan),
                   -math.sin(a3) * math.sin(pan), math.cos(a3)])

    # ⚠ **wrist_roll을 반드시 넣는다.** 카메라는 5번 관절 *뒤에* 달려 있으므로
    #   손목을 굴리면 카메라가 접근축 둘레로 돈다. 레거시의 `handeye.tool_frame`은
    #   이걸 빼고 있는데, 그러면 (가) 보정 표본에 roll을 못 쓰고 — eye-in-hand에서
    #   t_x를 가르는 게 바로 이 축이다 — (나) 보정 때와 다른 roll로 집으러 가면
    #   카메라 좌표가 조용히 틀린다.
    roll = math.radians(ROLL_SIGN * float(degs.get("wrist_roll", 0.0)))
    lat = lateral * math.cos(roll) + up * math.sin(roll)
    upr = -lateral * math.sin(roll) + up * math.cos(roll)
    pose = kin.forward(degs, geom)
    return Rigid(np.array([approach, lat, upr], dtype=float).T,
                 np.array([pose.x, pose.y, pose.z], dtype=float))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--save", action="store_true")
    ap.add_argument("--dot", default="tl", choices=("tl", "tr", "bl", "br"))
    ap.add_argument("--home", default="2.8,-9.1,100.8,83.2,-3.6",
                    help="시작 자세(도): pan,lift,elbow,wflex,wroll. "
                         "'here'면 지금 자세를 그대로 쓴다")
    args = ap.parse_args()

    cal = json.load(open(CAL))
    spans = {}
    for name, c in cal.items():
        try:
            spans[name] = abs(int(c["range_max"]) - int(c["range_min"])) * DEG_PER_TICK
        except (KeyError, TypeError, ValueError):
            pass
    cart = json.load(open(CART))
    zero, ref, signs = cart["zero"], cart["ref_deg"], cart.get("signs", {})
    geom = kin.ArmGeometry()

    def sign(j):
        v = signs.get(j)
        return -1.0 if (v is not None and float(v) < 0) else 1.0

    # ⚠ **실측 눈금이 보정표를 이긴다** (`~/arm_cartesian.json`의 deg_per_norm).
    #   2026-09-01: `wrist_roll`은 계산한 각도의 0.56배만 실제로 돌았다 —
    #   관절축 측정 잔차가 24.8mm에서 2.6mm로 떨어졌고, 화면회전 실측
    #   0.549와도 맞는다. 손목 굴림에 감속이 있어 틱→도(360/4096)가 안 통한다.
    over = cart.get("deg_per_norm") or {}

    def dpn(j):
        v = over.get(j)
        if v:
            return abs(float(v))
        s = spans.get(j)
        return abs(s) / 200.0 if s else (1.8 if j == "wrist_roll" else 0.9)

    def to_deg(n):
        return {j: ref.get(j, 0.0) + sign(j) * (float(n.get(j, 0.0)) - zero.get(j, 0.0)) * dpn(j)
                for j in kin.JOINTS}

    def to_norm(d):
        return {j: zero.get(j, 0.0) + (float(d[j]) - ref.get(j, 0.0)) / (sign(j) * dpn(j))
                for j in d if j in kin.JOINTS}

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)

    def save_pose_view(d):
        try:
            json.dump({"deg": {j: float(d[j]) for j in kin.JOINTS}, "ts": time.time()},
                      open(POSE_VIEW, "w"))
        except Exception:                                      # noqa: BLE001
            pass

    def _goto(degs, secs=0.9):
        cur = to_deg(io.read())
        biggest = max(abs(degs[j] - cur[j]) for j in kin.JOINTS)
        steps = max(1, int(math.ceil(biggest / STEP_DEG)))
        for st in range(1, steps + 1):
            mid = {j: cur[j] + (degs[j] - cur[j]) * st / steps for j in kin.JOINTS}
            io.write(to_norm(mid), secs)
            save_pose_view(mid)

    def move(degs, secs=0.9):
        """목표 자세로 — **마지막 움직임의 방향을 늘 같게** 해서 간다.

        ⚠ 이 팔에는 백래시가 있다. 2026-09-01 실측(`repeat_check.py`):
          같은 관절값이라도 어느 쪽에서 왔느냐에 따라 카메라가 본 것이 달랐다.

              그냥 접근    흩어짐 15.4mm (최대 31.5) · 관절 1.94°
              단방향 접근  흩어짐  2.5mm (최대  8.1) · 관절 0.50°

          이걸 모르고 모델만 고치면 영원히 안 맞는다 — 보정 잔차의 **하한**이
          기계에서 정해지기 때문이다. 백래시를 없앨 수는 없어도 **일정하게**
          만들 수는 있다: 늘 한쪽에서 다가가면 톱니가 늘 같은 면에 닿고, 그
          오차는 보정이 흡수하는 상수가 된다. CNC가 쓰는 그 방법이다.
        """
        _goto({j: degs[j] - BACKOFF_DEG for j in kin.JOINTS}, secs)
        _goto(degs, secs)
        time.sleep(SETTLE_SEC)
        got = to_deg(io.read())
        save_pose_view(got)
        return got, max(abs(got[j] - degs[j]) for j in kin.JOINTS)

    def look():
        try:
            m = tc.measure()
        except Exception as e:            # noqa: BLE001
            # 한 프레임이 나쁘다고 채집 전체가 죽으면 안 된다 — 팔은 토크가 걸린
            # 채로 남고, 그때까지 모은 표본도 함께 사라진다.
            print(f"    (프레임 하나를 못 읽었다: {e})")
            return None
        if not m["ok"]:
            return None
        us = [m["dots"][k][0] for k in ("tl", "tr", "bl", "br")]
        vs = [m["dots"][k][1] for k in ("tl", "tr", "bl", "br")]
        m["box"] = (min(us), min(vs), max(us), max(vs))
        return m

    # ⚠ 붙는 순간 팔이 늘어진다 — `FollowerIO._connect`가 `disable_torque`를 하기
    #    때문이다(안전한 기본값이다: 켜자마자 힘이 들어가면 사람이 놀란다).
    #    그래서 **연결 직후 곧바로 지금 자리를 목표로 줘서 붙든다.** 이걸 안 하면
    #    표적이 보이도록 맞춰 둔 자세를 매번 잃고, 중력에 끌려 내려간 자리에서
    #    시작하게 된다(실측: 붙을 때마다 lift가 -9° → -96°로 처졌다).
    io.write(io.read(), 0.4)

    if args.home.strip().lower() == "here":
        home = to_deg(io.read())
    else:
        vals = [float(v) for v in args.home.split(",")]
        want = dict(zip(kin.JOINTS, vals))
        print("시작 자세로 이동 — " + " ".join(f"{j.split('_')[0]}={want[j]:.1f}"
                                            for j in kin.JOINTS))
        home, herr = move(want)
        print(f"  도착 (관절 오차 {herr:.1f}°)")
    hp = kin.forward(home, geom)
    print("기준 자세  " + " ".join(f"{j.split('_')[0]}={home[j]:6.1f}" for j in kin.JOINTS))
    print(f"           TCP ({hp.x:6.1f},{hp.y:6.1f},{hp.z:6.1f}) pitch {hp.pitch:6.1f}° "
          f"signed_r {kin.signed_radius(home, geom):6.1f}mm")

    # ⚠ 기준 관측도 **다른 자세들과 똑같은 방식으로 도착해서** 찍어야 한다.
    #   2026-09-01 실측: 기준만 `arm_stage`로(백오프 없이) 도착해 찍었더니,
    #   이후 단방향 접근으로 돌아온 관측들이 6회 내리 37~38mm 떨어져 나왔다.
    #   6회끼리는 1.2mm 안에서 일치했다 — 흩어짐이 아니라 **백래시 오프셋**이다.
    #   그걸 "채집 도중 뭔가 움직였다"로 읽고 멀쩡한 표본을 두 번 버렸다.
    move(dict(home))
    base_view = look()
    if base_view is None:
        print("❌ 지금 자세에서 4점이 안 보인다 — 카메라를 표적 쪽으로 맞춘 뒤 다시.")
        io.hold_close()
        return 1
    print(f"           표적 상자 {tuple(round(v) for v in base_view['box'])} · "
          f"거리 {base_view['plane_mm']:.0f}mm")

    _K0 = json.load(open(tc.META))["intrinsics"]
    CX, CY = float(_K0["ppx"]), float(_K0["ppy"])

    def center_target():
        """표적을 화면 가운데로 끌어온다 — **야코비안을 재기 전에.**

        ⚠ 2026-09-01: 표적이 화면 오른쪽 끝(571~797 / 폭 848)에 붙어 있어서
          pan을 어느 쪽으로 흔들어도 표적을 잃었고, 채집이 야코비안 단계에서
          그대로 죽었다. 가장자리에서는 **잴 수조차 없다** — 계획을 세우기 전에
          여유부터 만들어야 한다.

        pan은 가로, wrist_flex는 세로를 움직인다. 이득은 그때그때 작은 시험각으로
        직접 잰다(자세마다 다르다).
        """
        nonlocal base_view, home
        for _ in range(4):
            b = base_view["box"]
            cu, cv = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
            if abs(cu - CX) < 60.0 and abs(cv - CY) < 45.0:
                return True
            gains = {}
            for j, idx in (("shoulder_pan", 0), ("wrist_flex", 1)):
                for probe in (4.0, -4.0):
                    d = dict(home)
                    d[j] += probe
                    move(d)
                    v = look()
                    if v is not None:
                        pb = v["box"]
                        g = (((pb[0] + pb[2]) / 2.0 if idx == 0 else (pb[1] + pb[3]) / 2.0)
                             - (cu if idx == 0 else cv)) / probe
                        if abs(g) > 0.5:
                            gains[j] = g
                            break
                move(dict(home))
            if "shoulder_pan" not in gains or "wrist_flex" not in gains:
                print("  가운데로 못 끌어온다 — 이득을 못 쟀다")
                return False
            d = dict(home)
            d["shoulder_pan"] += max(-18.0, min(18.0, (CX - cu) / gains["shoulder_pan"]))
            d["wrist_flex"] += max(-18.0, min(18.0, (CY - cv) / gains["wrist_flex"]))
            got, _ = move(d)
            v = look()
            if v is None:
                move(dict(home))
                print("  가운데로 옮기다 표적을 잃었다 — 되돌렸다")
                return False
            home, base_view = got, v
            b = v["box"]
            print(f"  가운데로: 상자 중심 ({(b[0]+b[2])/2:.0f},{(b[1]+b[3])/2:.0f}) "
                  f"← 광축 ({CX:.0f},{CY:.0f})")
        return True

    center_target()

    def pose_for(dpan, dpitch, dlift, delbow, droll=0.0):
        """다섯 축 → 관절각. lift/elbow가 만든 pitch 변화는 wrist_flex로 되돌린다."""
        d = dict(home)
        d["shoulder_pan"] += dpan
        d["shoulder_lift"] += dlift
        d["elbow_flex"] += delbow
        d["wrist_flex"] += dpitch - dlift - delbow
        d["wrist_roll"] += droll
        return d

    def compensable(dpan, dpitch, dlift, delbow, droll=0.0):
        """`wrist_flex`가 **실제로 상쇄할 수 있는** 조합인가.

        ⚠ 이걸 안 보면 조용히 틀린다. lift를 -12° 움직이며 wrist_flex를 +12°로
          상쇄하려 했는데 wrist_flex가 이미 93°였고 한계가 105°라 다 못 갔다.
          그러면 "자세는 그대로, 위치만 이동"이라는 전제가 깨지는데 **아무도
          말해주지 않는다** — 실측에서 a3가 9.6° 어긋난 채로 표본이 들어갔다.
        """
        d = pose_for(dpan, dpitch, dlift, delbow, droll)
        return (abs(to_norm(d)["wrist_flex"]) <= 98.0
                and abs(to_norm(d)["wrist_roll"]) <= 98.0)

    # ── 야코비안: 각 축이 화면을 몇 px 옮기는가 (실측) ──
    print(f"\n[야코비안] 축마다 {PROBE_DEG:.0f}°가 화면을 몇 px 옮기는지 실측")
    axes = ("pan", "pitch", "lift", "elbow", "roll")
    jac = {}
    for k, name in enumerate(axes):
        done = False
        for sgn in (+1.0, -1.0):
            amt = [0.0, 0.0, 0.0, 0.0, 0.0]
            amt[k] = sgn * PROBE_DEG
            got, e = move(pose_for(*amt))
            v = look()
            if v is not None and e <= MAX_TRACK_ERR:
                du = (v["box"][0] - base_view["box"][0]) / (sgn * PROBE_DEG)
                dv = (v["box"][1] - base_view["box"][1]) / (sgn * PROBE_DEG)
                jac[name] = (du, dv)
                print(f"  {name:<7} {du:+7.2f}, {dv:+7.2f} px/도")
                done = True
                break
            print(f"  {name:<7} {sgn*PROBE_DEG:+.0f}°에서 표적을 잃음 — 반대로")
        if not done:
            print(f"❌ {name} 축의 화면 이동량을 못 쟀다.")
            io.hold_close()
            return 1
        move(dict(home))

    bw = base_view["box"][2] - base_view["box"][0]
    bh = base_view["box"][3] - base_view["box"][1]
    u0, v0 = base_view["box"][0], base_view["box"][1]

    # 광축이 화면에서 어디인가 — roll은 이 점 둘레로 돈다.
    _K = json.load(open(tc.META))["intrinsics"]
    PPX, PPY = float(_K["ppx"]), float(_K["ppy"])

    def visible(a):
        """이 자세에서 점 네 개가 다 화면에 남는가 (계획 단계의 어림).

        ⚠ roll만 다르게 다룬다. 나머지 축은 표적을 **옮기지만**, roll은 광축 둘레로
          **돌린다** — 8°에서 잰 px/도를 60°로 늘려 쓰면 크게 틀린다. 그래서 roll은
          상자를 주점(cx, cy) 둘레로 돌려 실제로 커지는 만큼을 계산한다.
          도는 **방향(부호)은 아직 모르므로 양쪽 다 들어맞을 때만** 통과시킨다 —
          헛걸음 한 번이 팔을 10초 움직이는 값이다.
        """
        idx = axes.index("roll")
        du = sum(jac[n][0] * a[i] for i, n in enumerate(axes) if n != "roll")
        dv = sum(jac[n][1] * a[i] for i, n in enumerate(axes) if n != "roll")
        cu, cv = u0 + du + bw / 2.0, v0 + dv + bh / 2.0
        th = math.radians(a[idx])
        if abs(th) < 1e-9:
            return (EDGE_MARGIN <= cu - bw / 2.0 and cu + bw / 2.0 <= W - EDGE_MARGIN
                    and EDGE_MARGIN <= cv - bh / 2.0 and cv + bh / 2.0 <= H - EDGE_MARGIN)
        # 돌아간 상자의 외접 크기
        rw = abs(bw * math.cos(th)) + abs(bh * math.sin(th))
        rh = abs(bw * math.sin(th)) + abs(bh * math.cos(th))
        for sgn in (+1.0, -1.0):
            c, s_ = math.cos(sgn * th), math.sin(sgn * th)
            ou, ov = cu - PPX, cv - PPY
            nu = PPX + ou * c - ov * s_
            nv = PPY + ou * s_ + ov * c
            if not (EDGE_MARGIN <= nu - rw / 2.0 and nu + rw / 2.0 <= W - EDGE_MARGIN
                    and EDGE_MARGIN <= nv - rh / 2.0 and nv + rh / 2.0 <= H - EDGE_MARGIN):
                return False
        return True

    cands = []
    for dp in PAN_GRID:
        for dt in PITCH_GRID:
            for dl in LIFT_GRID:
                for de in ELBOW_GRID:
                    for dr in ROLL_GRID:
                        a = (dp, dt, dl, de, dr)
                        if not visible(a) or not compensable(*a):
                            continue
                        degs = pose_for(*a)
                        pose = kin.forward(degs, geom)
                        # ⚠ 가동범위는 정규화값 -100..100이다(교시 자세 중심의 대칭이
                        #   아니다). 도로 ±span/2를 보면 멀쩡한 자세를 막는다.
                        nm = to_norm(degs)
                        bad = any(abs(nm[j]) > 98.0 for j in kin.JOINTS)
                        if bad or pose.z < -MOUNT_Z_MM + FLOOR_MARGIN_MM or math.hypot(pose.x, pose.y) > MAX_TCP_R:
                            continue
                        cands.append((a, degs, pose))
    total = (len(PAN_GRID) * len(PITCH_GRID) * len(LIFT_GRID)
             * len(ELBOW_GRID) * len(ROLL_GRID))
    print(f"\n[후보] 격자 {total}개 중 쓸 수 있는 것 {len(cands)}개")

    # 회전과 평행이동을 **둘 다** 넓게 — 이미 고른 것들에서 가장 먼 후보를 집는다.
    picked = [c for c in cands if c[0] == (0.0, 0.0, 0.0, 0.0, 0.0)] or [cands[0]]
    while len(picked) < WANT_POSES and len(picked) < len(cands):
        best, bestd = None, -1.0
        for c in cands:
            if any(c[0] == p[0] for p in picked):
                continue
            d = min(sum((c[0][i] - p[0][i]) ** 2 for i in range(5)) for p in picked)
            if d > bestd:
                best, bestd = c, d
        if best is None:
            break
        picked.append(best)

    # ── 회전 다양성 검사 — 표본을 다 찍고 나서 알면 늦다 ──
    # eye-in-hand는 R_x를 **도구가 여러 방향을 봤을 때만** 가른다. pan 하나만
    # 변하면 그 축 둘레의 회전은 방정식에서 상쇄돼 남지 않는다. 최소자승은
    # 그래도 답을 내놓는다 — 쓰레기 답을. 그러니 여기서 막는다.
    spread = {n: sorted({c[0][i] for c in picked})
              for i, n in enumerate(axes)}
    print("  축별 값: " + " · ".join(f"{n} {v}" for n, v in spread.items()))
    thin = [n for n in ("pan", "pitch") if len(spread[n]) < 2]
    if thin:
        print("")
        print(f"❌ 회전축 {thin} 이(가) 한 값뿐이다 — 이 표본으로는 손-눈 보정이")
        print("   **원리적으로** 안 풀린다(축 둘레 회전이 방정식에서 사라진다).")
        print("   화면 여유가 없어서다. 셋 중 하나를 해야 한다:")
        print("     · 표적을 더 작게 만든다 (지금 화면에서 상자가 너무 크다)")
        print("     · 팔을 표적에서 더 멀리 둔다")
        print("     · 한계에 붙은 관절(지금 elbow)을 안쪽으로 옮긴 자세에서 다시 시작한다")
        io.hold_close()
        return 1

    xs = np.array([[p[2].x, p[2].y, p[2].z] for p in picked])
    print(f"계획 {len(picked)}자세 · TCP 이동 폭 "
          f"x {np.ptp(xs[:,0]):.0f} y {np.ptp(xs[:,1]):.0f} z {np.ptp(xs[:,2]):.0f} mm")
    for a, degs, pose in picked:
        print(f"  pan{a[0]:+5.0f} pit{a[1]:+5.0f} lif{a[2]:+5.0f} "
              f"elb{a[3]:+5.0f} rol{a[4]:+5.0f}  "
              f"TCP ({pose.x:6.1f},{pose.y:6.1f},{pose.z:6.1f}) pitch {pose.pitch:6.1f}°")

    if args.dry:
        print("\n(--dry 이므로 움직이지 않는다)")
        io.hold_close()
        return 0

    cam_pts, frames, labels, all_dots, cam_org = [], [], [], [], []
    raw = []          # ⚠ 원자료를 남긴다 — 가설 하나 시험하려고 팔을 다시 움직이지 않게.
    for a, degs, _ in picked:
        label = (f"pan{a[0]:+.0f} pit{a[1]:+.0f} lif{a[2]:+.0f} "
                 f"elb{a[3]:+.0f} rol{a[4]:+.0f}")
        got, err = move(degs)
        if err > MAX_TRACK_ERR:
            print(f"  {label:<34} ⚠ 지령과 {err:.1f}° 차이 — 서보가 놓았다. 중단.")
            break
        m = look()
        if m is None:
            print(f"  {label:<34} 표적 안 보임 — 건너뜀")
            continue
        pt = m["points_mm"][args.dot]
        recon = m.get("reconstructed")   # 점 3개만 보여서 재구성한 자리 이름(있으면)
        cam_pts.append(pt)
        frames.append(tool_frame_from_joints(got, geom))
        labels.append(label)
        all_dots.append(m["points_mm"])
        cam_org.append(kin.forward(got, geom))
        raw.append({"label": label, "joints_deg": {k: float(v) for k, v in got.items()},
                    "dots_mm": {k: [float(c) for c in v] for k, v in m["points_mm"].items()},
                    "dots_px": {k: [float(c) for c in v] for k, v in m["dots"].items()},
                    "plane_mm": float(m["plane_mm"]), "tilt_deg": float(m["tilt_deg"]),
                    "reconstructed": recon})
        tag = f" ⚠재구성({recon})" if recon else ""
        print(f"  {label:<34} ✓ {len(cam_pts):2}  카메라점 "
              f"({pt[0]:7.1f},{pt[1]:7.1f},{pt[2]:7.1f}) · 거리 {m['plane_mm']:.0f}mm{tag}")

    print(f"\n표본 {len(cam_pts)}개")
    dump = os.path.expanduser("~/handeye_samples.json")
    with open(dump, "w", encoding="utf-8") as fh:
        json.dump({"geom": {"z0": geom.z0, "d0": geom.d0, "l1": geom.l1,
                            "l2": geom.l2, "l3": geom.l3},
                   "expect_mm": {"w": 100.0, "h": 174.5},
                   "marker": args.dot, "samples": raw}, fh, ensure_ascii=False, indent=1)
    print(f"  원자료 {dump} ({len(raw)}개) — 다시 풀 때 팔을 안 움직여도 된다")

    # ── 표류 검사: 처음 자리로 돌아가 같은 것을 다시 본다 ──────────────
    # 손-눈 보정은 "카메라가 팔에 **단단히** 붙어 있다"를 전제한다. 그런데 채집
    # 도중에 서보가 놓거나 브래킷이 밀리면 그 전제가 조용히 깨지고, 표본들이
    # 서로 다른 강체를 말하게 된다 — 잔차만 보고는 절대 못 알아챈다.
    # 그래서 마지막에 출발 자세로 돌아가 처음과 같은 것을 보는지 확인한다.
    move(dict(home))
    again = look()
    if again is None:
        print("  ⚠ 표류 검사 실패 — 처음 자리에서 표적이 안 보인다. 무언가 움직였다.")
    else:
        d0 = np.mean([base_view["points_mm"][k] for k in ("tl", "tr", "bl", "br")], axis=0)
        d1 = np.mean([again["points_mm"][k] for k in ("tl", "tr", "bl", "br")], axis=0)
        drift = float(np.linalg.norm(np.array(d1) - np.array(d0)))
        mark = "  ⚠ 채집 도중 무언가 움직였다 — 이 표본들은 못 믿는다" if drift > 5.0 else ""
        print(f"  표류 검사: 출발 자세에서 표적이 {drift:.1f}mm 옮겨 보인다{mark}")
    if len(cam_pts) < 8:
        print("❌ 8개 미만 — 풀 수 없다.")
        io.hold_close()
        return 1

    # 조건수 진단 — 지난번 실패의 원인이 여기였다.
    d = np.array([math.dist((0, 0, 0), p) for p in cam_pts])
    tt = np.array([[f.t[0], f.t[1], f.t[2]] for f in frames])
    print(f"  카메라↔표적 거리 {d.min():.0f}~{d.max():.0f}mm (폭 {np.ptp(d):.0f})")
    print(f"  도구 원점 이동 폭 x {np.ptp(tt[:,0]):.0f} y {np.ptp(tt[:,1]):.0f} "
          f"z {np.ptp(tt[:,2]):.0f} mm")

    try:
        fit = solve_on_arm(cam_pts, frames)
    except CalibrationError as exc:
        print(f"❌ {exc}")
        io.hold_close()
        return 1

    print(f"\n{fit.summary()}")
    print(f"  T_tool_cam  t = {np.round(fit.transform.t, 1).tolist()} mm")
    print(f"              rpy = {tuple(round(v,1) for v in fit.transform.rpy_deg)}°")
    if fit.marker_base:
        print(f"  표적 위치(팔 base 기준) = {tuple(round(v,1) for v in fit.marker_base)} mm")
    w = fit.worst_index()
    if 0 <= w < len(labels):
        print(f"  가장 어긋난 표본: {w}번 ({labels[w]}) {fit.per_sample_mm[w]:.1f}mm")

    # ⚠ 점 3개만 보여 재구성한 표본은 그 재구성된 자리가 나머지 셋의
    #   평행사변형 등식으로 정의상 완벽히 맞아떨어진다 — 그 자리를 마커로
    #   삼는 교차검증 칸은 "독립 관측끼리 일치하나"를 재는 게 아니게 된다.
    #   위 ✓ 줄의 "⚠재구성(...)" 표시로 어떤 표본이 그런지 알 수 있다.
    print("\n[교차검증] 네 점을 각각 마커로 삼아 따로 푼다")
    sols = {}
    for key in ("tl", "tr", "bl", "br"):
        try:
            f2 = solve_on_arm([dd[key] for dd in all_dots], frames)
            sols[key] = f2
            print(f"  {key}: 잔차 {f2.rms_mm:5.1f}mm · t = {np.round(f2.transform.t,1).tolist()}")
        except CalibrationError as exc:
            print(f"  {key}: 못 풀었다 — {str(exc)[:60]}")
    if len(sols) >= 2:
        ts = np.array([s.transform.t for s in sols.values()])
        spread = float(np.linalg.norm(ts.max(axis=0) - ts.min(axis=0)))
        print(f"  네 해의 카메라 위치 흩어짐 {spread:.1f}mm "
              + ("— 잘 일치한다" if spread < 10 else "⚠ 크다"))

    if args.save and fit.good:
        from tomato_picker.hardware.eye import EyeConfig
        cfg = EyeConfig()
        cfg.store("on_arm", fit, None, note="handeye_collect.py (관절공간 수집)")
        print(f"\n저장: {cfg.path}")
    elif args.save:
        print(f"\n잔차가 커서 저장하지 않았다 ({fit.rms_mm:.1f}mm).")

    print("\n⚠ 토크를 켠 채로 둔다 (여기서 끄면 팔이 떨어진다).")
    io.hold_close()          # noqa: SLF001
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
