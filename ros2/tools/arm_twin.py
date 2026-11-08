#!/usr/bin/env python3
"""**가짜 팔(처짐 + 마찰) 위에서 실기 도구를 그대로 돌린다** — PC, 장치 없이.

    python ros2/tools/arm_twin.py          # 과거 사고 재현 시험(후향 검증)

checkpoint_sim의 잡기는 오차를 '주사위로 넣는' 모형이라 코드가 오차에 어떻게 반응하는지는 못 본다.
여기는 FollowerIO만 바꿔 끼우고 tool_jog는 **git에 있는 그 시점 코드 그대로** 돈다.

모형 — 관절마다: 평형 = 목표 − 중력토크/K, 평형 ±f 안이면 안 움직인다(정지마찰).
K·f 둘은 맞춘 값이고, 맞출 때 안 쓴 실측을 맞히는지로 모형을 판정한다.
⚠ 충돌·전원·검출기·브래킷 유격은 이 모형에 없다.
"""
from __future__ import annotations

import atexit
import contextlib
import io as _io
import json
import math
import os
import re
import runpy
import shutil
import subprocess
import sys
import tempfile
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
for p in (HERE, os.path.join(REPO, "src"), os.path.join(REPO, "ros2", "src", "tomato_bridge")):
    sys.path.insert(0, p)

from tomato_picker.hardware import kinematics as kin  # noqa: E402
import arm_calib  # noqa: E402

# 젯슨 실값(2026-11-06 복사). 관절 범위·영점이 바뀌면 다시 복사한다.
CAL = {"shoulder_pan": [560, 3330], "shoulder_lift": [1009, 3350], "elbow_flex": [808, 3159],
       "wrist_flex": [735, 3099], "wrist_roll": [0, 4095], "gripper": [1238, 2766]}
CART = {"zero": {"shoulder_pan": -11.516, "shoulder_lift": -18.242, "elbow_flex": -60.352,
                 "wrist_flex": -1.758, "wrist_roll": 0.0},
        "ref_deg": {"shoulder_pan": 0.0, "shoulder_lift": 90.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0},
        "signs": {}, "deg_per_norm": {"wrist_roll": 0.999}}
START = dict(zip(kin.JOINTS, (1.6, 93.9, 115.7, -34.7, 6.4)))   # 규약 시작 자세(도). FK r≈−327: 잡을 때 팔은 FK −x 쪽을 본다
# 질량(kg)은 부품 추정: 서보 0.055, 손목+집게+D405 뭉치. 크기는 K가 흡수하고 여기선 비율만 뜻이 있다.
M1, M_ELBOW, M2, M_HAND = 0.06, 0.055, 0.115, 0.21
HAND_D = 70.0                                                    # wrist 축 → 손 무게중심(mm)
TICK = 360.0 / 4096.0


def _calib():
    d = tempfile.mkdtemp(prefix="arm_twin_")
    atexit.register(shutil.rmtree, d, True)
    a, b = os.path.join(d, "cal.json"), os.path.join(d, "cart.json")
    json.dump({k: {"range_min": lo, "range_max": hi} for k, (lo, hi) in CAL.items()}, open(a, "w"))
    json.dump(CART, open(b, "w"))
    return a, b


_CAL_PATHS = _calib()
arm_calib.Calib.__init__.__defaults__ = _CAL_PATHS
calib = arm_calib.Calib()


def gravity(q):
    """관절 각(도) → 관절별 중력 토크(비율, kg·mm). 양수 = 각을 줄이는 쪽(아래로)."""
    g = calib.geom
    a1 = math.radians(q["shoulder_lift"])
    a2 = a1 + math.radians(q["elbow_flex"])
    a3 = a2 + math.radians(q["wrist_flex"])
    c1, c2, c3 = math.cos(a1), math.cos(a2), math.cos(a3)
    wrist = M_HAND * HAND_D * c3
    elbow = wrist + M_HAND * g.l2 * c2 + M2 * g.l2 / 2 * c2
    lift = elbow + (M_HAND + M2 + M_ELBOW) * g.l1 * c1 + M1 * g.l1 / 2 * c1
    return {"shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist}


class Twin:
    def __init__(self, K, f, start=START):
        self.K, self.f = K, f              # K: kg·mm/도, f: 정지마찰 폭(도)
        self.g = dict(calib.to_norm(start), gripper=40.0)  # 목표(정규값, FollowerIO와 같은 단위)
        self.q = dict(start)                # 실제 각(도)
        self.settle()

    def settle(self):
        goal = calib.to_deg(self.g)
        for _ in range(30):                 # 토크가 자세에 달려 있어 고정점을 찾는다
            tau = gravity(self.q)
            for j in kin.JOINTS:
                eq = goal[j] - tau.get(j, 0.0) / self.K
                self.q[j] = min(max(self.q[j], eq - self.f), eq + self.f)

    def write(self, target):
        for j, v in target.items():
            if j in kin.JOINTS:            # 서보 분해능 — 목표는 틱 단위로만 들어간다
                per = calib.per(j)
                v = round(float(v) * per / TICK) * TICK / per
            self.g[j] = float(v)
        self.settle()

    def read(self):
        return dict(calib.to_norm(self.q), gripper=self.g["gripper"])

    def tcp(self):
        return kin.forward(self.q, calib.geom)


CURRENT = {"twin": None}


class FakeIO:
    """FollowerIO에서 도구들이 부르는 만큼만."""

    def __init__(self, hold_torque=False, **_):
        pass

    def read(self):
        return CURRENT["twin"].read()

    def goal(self):
        return dict(CURRENT["twin"].g)

    def write(self, target, secs):
        CURRENT["twin"].write(target)

    def hold_close(self):
        pass

    close = hold_close


_mod = types.ModuleType("tomato_bridge.follower_io")
_mod.FollowerIO = FakeIO
sys.modules["tomato_bridge.follower_io"] = _mod
time.sleep = lambda s: None


def run_tool(path, *args):
    """도구를 argv까지 그대로 돌린다. 출력은 삼키고 끝 코드를 돌려준다."""
    sys.argv = [path, *map(str, args)]
    buf = _io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            runpy.run_path(path, run_name="__main__")
            code = 0
        except SystemExit as e:
            code = e.code or 0
    return code, buf.getvalue()


def old_tool(rev):
    """그 시점의 tool_jog를 임시 파일로 꺼낸다."""
    src = subprocess.run(["git", "-C", REPO, "show", f"{rev}:ros2/tools/tool_jog.py"],
                         capture_output=True, check=True).stdout
    fd, path = tempfile.mkstemp(suffix="_tool_jog.py")
    atexit.register(os.remove, path)
    with os.fdopen(fd, "wb") as f:
        f.write(src)
    return path


OLD, NEW = old_tool("1b68edf^"), os.path.join(HERE, "tool_jog.py")


def arrive(t):
    """실기는 시작 자세에 press로 도착해 있다 — 처짐을 갚아 둔 목표로 출발한다."""
    for _ in range(6):
        t.write(calib.to_norm({j: calib.to_deg(t.g)[j] + START[j] - t.q[j] for j in kin.JOINTS}))


def _run_ok(path, *args):
    code, out = run_tool(path, *args)
    if code or "멈춘다" in out or "안 간다" in out:   # 한계에 걸려 덜 간 값을 '맞았다'로 읽지 않는다
        raise RuntimeError(out)
    return out


def jog(path, K, f, *args, pre=None):
    """시작 자세(pre가 있으면 먼저 그만큼 나아간 뒤)에서 도구 한 번
    → (끝 dz, 끝 d피치, 걸음별 최저 dz, 쓰기별 최저 dz). 걸음별 = 도구가 찍는 TCP = 실기 기록이 본 값."""
    t = CURRENT["twin"] = Twin(K, f)
    arrive(t)
    if pre:
        _run_ok(path, *pre)                # 실기 순서: 나아가서 → 물러난다
    p0, low, write = t.tcp(), [0.0], t.write

    def spy(target):
        write(target)
        low[0] = min(low[0], t.tcp().z - p0.z)
    t.write = spy
    out = _run_ok(path, *args)
    p1 = t.tcp()
    steps = [float(m) - p0.z for m in re.findall(r"TCP \([^,]+, [^,]+, ([-\d.]+)\)", out)][1:]
    return p1.z - p0.z, p1.pitch - p0.pitch, min(steps), low[0]


def close_drop(K, f):
    """옛 닫기: dict(io.read(), gripper=g) — 실측 z −15mm(11/02)."""
    t = CURRENT["twin"] = Twin(K, f)
    arrive(t)
    z0 = t.tcp().z
    t.write(dict(t.read(), gripper=5.0))
    return t.tcp().z - z0


def fit():
    """맞춤은 닫기 −15mm + 조그 87mm −24mm 둘만. 나머지 실측은 안 본다."""
    best = None
    for K in (5, 7, 10, 14, 20, 28, 40, 56, 80, 110, 160, 220, 320, 450, 640, 900, 1300, 1800, 2600, 3600, 5000):
        for f in (0.0, 0.1, 0.2, 0.3, 0.45, 0.6, 0.8, 1.0, 1.3):
            e = (close_drop(K, f) + 15) ** 2 + (jog(OLD, K, f, "--horiz", 87)[0] + 24) ** 2
            if best is None or e < best[0]:
                best = (e, K, f)
    return best[1], best[2]


if __name__ == "__main__":
    K, f = fit()
    PRE = ("--horiz", 50)                  # 시작 자세에선 물러날 때 팔꿈치 한계에 걸린다. 실기도 나아간 자리에서 물러났다
    INTENDED = -7.1                        # --along -100의 의도된 dz(접근축이 기울어 있다)
    old, new = jog(OLD, K, f, "--along", -100, pre=PRE), jog(NEW, K, f, "--along", -100, pre=PRE)
    back = {d: jog(NEW, K, f, "--horiz", -35, "--dz", d, pre=PRE) for d in (0, 6, 10)}   # 실측: f5b3bc2, 11/04
    # (구분, 항목, 실측, 모형, 허용)
    rows = [
        ("맞춤", "옛 닫기 z", -15, close_drop(K, f), 6),
        ("맞춤", "옛 조그 --horiz 87 z", -24, jog(OLD, K, f, "--horiz", 87)[0], 10),
        ("검증", "옛 조그 --along -100 z", -25, old[0], 10),
        ("검증", "옛 조그 3걸음 |피치|", 4, abs(jog(OLD, K, f, "--horiz", 75)[1]), 1.6),
        ("검증", "새 조그 빠지기 dz0  도중 최저", -5, back[0][2], 2),
        ("검증", "새 조그 빠지기 dz6  도중 최저", -2.5, back[6][2], 2),
        ("검증", "새 조그 빠지기 dz10 도중 최저", 1.9, back[10][2], 2),
    ]
    print("K=%g f=%g  시작 자세 처짐(도): %s" % (K, f, {j: round(v / K, 2) for j, v in gravity(START).items()}))
    print("%-4s %-30s %7s %7s" % ("", "항목", "실측", "모형"))
    for kind, name, real, sim, tol in rows:
        print("%-4s %-30s %7.1f %7.1f  %s" % (kind, name, real, sim, "" if abs(sim - real) <= tol else "✗"))
    print("--along -100 끝 z의 의도 대비 오차: 옛 %+.1f → 새 %+.1f" % (old[0] - INTENDED, new[0] - INTENDED))
    print("모형 예측(실기 미확인): 걸음 안 첫 쓰기마다 순간 최저 z 옛 %+.1f / 새 %+.1f" % (old[3], new[3]))
    miss = [n for k, n, real, sim, tol in rows if k == "검증" and abs(sim - real) > tol]
    total = sum(r[0] == "검증" for r in rows)
    print("후향 검증: %d/%d 맞힘%s" % (total - len(miss), total, " · 못 맞힘: " + ", ".join(miss) if miss else ""))
    assert abs(new[0] - INTENDED) < abs(old[0] - INTENDED) / 2, "고친 코드가 옛 코드보다 나아야 한다"
    raise SystemExit(1 if miss else 0)
