"""**가짜 팔(디지털 트윈)** — `TOMATO_TWIN=1`이면 `follower_io.FollowerIO` 자리에 끼워진다. 도구는 모른다.

    TOMATO_TWIN=1 python ros2/tools/arm_stage.py --read      # 어느 도구든 그대로
    python ros2/src/tomato_bridge/tomato_bridge/twin_io.py   # 자체검증

모형 = arm_twin.py 준정적 모형 그대로(관절마다 평형 = 목표 − 중력토크/K, 평형 ±f 안이면 안 움직임, 틱 분해능).
K·f는 arm_twin.fit() 2026-11-08 값. ⚠ 백래시·충돌·전원·검출기는 없다.

상태는 **파일**(`~/twin_arm.json`)에 산다 — 조작대(click_server)가 도구를 자식 프로세스로 띄우므로, 메모리에 두면
도구마다 새 팔이 된다. 읽을 때마다 `~/arm_pose_view.json`을 써서 조작대 /state·3D 미리보기가 따라오고,
`~/twin_log.jsonl`에 지령·실측·TCP를 남긴다(손끝 오차 mm = |FK(실측) − FK(지령)|).
"""
from __future__ import annotations

import json
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
for p in (os.path.join(REPO, "src"), os.path.join(REPO, "ros2", "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)

from tomato_picker.hardware import kinematics as kin  # noqa: E402

STATE = os.environ.get("TOMATO_TWIN_STATE", os.path.expanduser("~/twin_arm.json"))
POSE_VIEW = os.path.expanduser("~/arm_pose_view.json")
LOG = os.path.expanduser("~/twin_log.jsonl")
TWIN_DIR = os.path.expanduser("~/twin")
TICK = 360.0 / 4096.0
K, F = 28.0, 0.2                 # arm_twin.fit() 2026-11-08 (K: kg·mm/도, f: 정지마찰 폭 도)
M1, M_ELBOW, M2, M_HAND, HAND_D = 0.06, 0.055, 0.115, 0.21, 70.0
START = dict(zip(kin.JOINTS, (1.6, 93.9, 115.7, -34.7, 6.4)))
try:
    from tomato_picker.config import BASE_YAW_HOME_DEG, BASE_YAW_LIMIT_DEG
except ImportError:  # pragma: no cover
    BASE_YAW_HOME_DEG, BASE_YAW_LIMIT_DEG = -88.9, 90.0

# 젯슨 실값(arm_twin.py와 같은 사본, 2026-11-06). 관절 범위·영점이 바뀌면 다시 복사한다.
CAL = {"shoulder_pan": [560, 3330], "shoulder_lift": [1009, 3350], "elbow_flex": [808, 3159],
       "wrist_flex": [735, 3099], "wrist_roll": [0, 4095], "gripper": [1238, 2766]}
CART = {"zero": {"shoulder_pan": -11.516, "shoulder_lift": -18.242, "elbow_flex": -60.352,
                 "wrist_flex": -1.758, "wrist_roll": 0.0},
        "ref_deg": {"shoulder_pan": 0.0, "shoulder_lift": 90.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0},
        "signs": {}, "deg_per_norm": {"wrist_roll": 0.999}}


def ensure_calib() -> tuple[str, str]:
    """PC에 보정 파일이 없으면 `~/twin/`에 사본을 쓰고 TOMATO_CAL/TOMATO_CART를 가리킨다(이미 있으면 그대로)."""
    os.makedirs(TWIN_DIR, exist_ok=True)
    a, b = os.path.join(TWIN_DIR, "cal.json"), os.path.join(TWIN_DIR, "cart.json")
    if not os.path.exists(a):
        json.dump({k: {"range_min": lo, "range_max": hi} for k, (lo, hi) in CAL.items()}, open(a, "w"))
    if not os.path.exists(b):
        json.dump(CART, open(b, "w"))
    os.environ.setdefault("TOMATO_CAL", a)
    os.environ.setdefault("TOMATO_CART", b)
    return os.environ["TOMATO_CAL"], os.environ["TOMATO_CART"]


def _calib():
    ensure_calib()
    import arm_calib  # noqa: E402  (ros2/tools)
    return arm_calib.Calib(os.environ["TOMATO_CAL"], os.environ["TOMATO_CART"])


def gravity(q: dict, geom) -> dict:
    """관절 각(도) → 관절별 중력 토크(비율). 양수 = 각을 줄이는 쪽(아래로). arm_twin.gravity와 같다."""
    a1 = math.radians(q["shoulder_lift"])
    a2 = a1 + math.radians(q["elbow_flex"])
    a3 = a2 + math.radians(q["wrist_flex"])
    c1, c2, c3 = math.cos(a1), math.cos(a2), math.cos(a3)
    wrist = M_HAND * HAND_D * c3
    elbow = wrist + M_HAND * geom.l2 * c2 + M2 * geom.l2 / 2 * c2
    lift = elbow + (M_HAND + M2 + M_ELBOW) * geom.l1 * c1 + M1 * geom.l1 / 2 * c1
    return {"shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist}


def settle(q: dict, goal_deg: dict, geom, K: float = K, f: float = F) -> dict:
    q = dict(q)
    for _ in range(30):                     # 토크가 자세에 달려 있어 고정점을 찾는다
        tau = gravity(q, geom)
        for j in kin.JOINTS:
            eq = goal_deg[j] - tau.get(j, 0.0) / K
            q[j] = min(max(q[j], eq - f), eq + f)
    return q


def _atomic_dump(obj, path):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=1, ensure_ascii=False)
    os.replace(tmp, path)


class FollowerIO:
    """실기 FollowerIO가 도구에 내주는 표면 그대로(read/goal/write/yaw/spans_deg/busy_lock/before_move/닫기)."""

    def __init__(self, port: str = "", arm_id: str = "", move_fps: int = 30, hold_torque: bool = False) -> None:
        self._lock = threading.RLock()
        self.calib = _calib()
        self.K, self.f = K, F
        self._load()

    # ── 상태 파일 ──────────────────────────────────────────────
    def _load(self) -> None:
        try:
            s = json.load(open(STATE))
            self.g, self.q, self.yaw = s["g"], s["q"], float(s.get("yaw", BASE_YAW_HOME_DEG))
        except (OSError, ValueError, KeyError):
            self.q = dict(START)
            self.g = dict(self.calib.to_norm(START), gripper=40.0)
            self.yaw = BASE_YAW_HOME_DEG
            self.q = settle(self.q, self.calib.to_deg(self.g), self.calib.geom, self.K, self.f)
            self._save()

    def _save(self) -> None:
        _atomic_dump({"g": self.g, "q": self.q, "yaw": self.yaw, "ts": time.time(), "K": self.K, "f": self.f}, STATE)
        self._view()

    def _view(self) -> None:
        goal = self.calib.to_deg({j: self.g[j] for j in kin.JOINTS})
        pq, pg = kin.forward(self.q, self.calib.geom), kin.forward(goal, self.calib.geom)
        err = math.dist((pq.x, pq.y, pq.z), (pg.x, pg.y, pg.z))
        rec = {"ts": time.time(), "twin": True,
               "deg": {j: round(self.q[j], 2) for j in kin.JOINTS}, "goal_deg": {j: round(goal[j], 2) for j in kin.JOINTS},
               "tcp": [round(pq.x, 1), round(pq.y, 1), round(pq.z, 1)], "goal_tcp": [round(pg.x, 1), round(pg.y, 1), round(pg.z, 1)],
               "err_mm": round(err, 1), "gripper": round(float(self.g.get("gripper", 0.0)), 1), "yaw": round(self.yaw, 1)}
        try:
            _atomic_dump(rec, POSE_VIEW)
            with open(LOG, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except OSError:
            pass

    # ── 연결 흉내 ──────────────────────────────────────────────
    def describe(self) -> str:
        return "twin(%s)" % STATE

    @property
    def connected(self) -> bool:
        return True

    def _connect(self) -> None:
        pass

    def close(self) -> None:
        pass

    hold_close = close

    def relax(self) -> None:
        pass

    # ── JointIO ───────────────────────────────────────────────
    def read(self) -> dict[str, float]:
        with self._lock:
            self._load()
            self._view()
            return dict(self.calib.to_norm(self.q), gripper=float(self.g.get("gripper", 0.0)))

    def goal(self) -> dict[str, float]:
        with self._lock:
            self._load()
            return dict(self.g)

    def write(self, target: dict[str, float], secs: float) -> None:
        with self._lock:
            self._load()
            for j, v in target.items():
                v = float(v)
                if j in kin.JOINTS:               # 서보 분해능 — 목표는 틱 단위로만 들어간다
                    per = self.calib.per(j)
                    v = round(v * per / TICK) * TICK / per
                self.g[j] = v
            self.q = settle(self.q, self.calib.to_deg({j: self.g[j] for j in kin.JOINTS}), self.calib.geom, self.K, self.f)
            self._save()

    def read_yaw(self) -> float:
        with self._lock:
            self._load()
            return self.yaw

    def move_yaw(self, deg: float, secs: float | None = None) -> None:
        with self._lock:
            self._load()
            self.yaw = max(-BASE_YAW_LIMIT_DEG, min(BASE_YAW_LIMIT_DEG, float(deg)))
            self._save()

    def spans_deg(self) -> dict[str, float]:
        return dict(self.calib.spans)

    def busy_lock(self):
        return self._lock

    def before_move(self) -> None:
        pass


def _selftest() -> None:
    import tempfile
    global STATE, POSE_VIEW, LOG
    d = tempfile.mkdtemp(prefix="twin_io_")
    STATE, POSE_VIEW, LOG = (os.path.join(d, n) for n in ("state.json", "view.json", "log.jsonl"))
    io = FollowerIO(hold_torque=True)
    deg0 = io.calib.to_deg(io.read())
    assert all(abs(deg0[j] - START[j]) < 3.0 for j in kin.JOINTS), deg0            # 시작 자세 근처(처짐만큼)
    goal = {j: 0.0 for j in kin.JOINTS}
    goal["shoulder_lift"] = 45.0
    io.write(dict(io.calib.to_norm(goal), gripper=20.0), 0.5)
    got = io.calib.to_deg(io.read())
    assert got["shoulder_lift"] < 45.0 - 0.3, got                                  # 중력은 아래로 처진다
    assert abs(got["shoulder_pan"] - 0.0) < 0.5, got                               # pan에는 중력 항이 없다
    v = json.load(open(POSE_VIEW))
    assert v["twin"] and v["err_mm"] > 0.0 and v["gripper"] == 20.0, v
    io2 = FollowerIO()                                                              # 다른 프로세스 흉내 — 같은 팔
    assert io2.goal()["gripper"] == 20.0 and abs(io2.read()["shoulder_lift"] - io.read()["shoulder_lift"]) < 1e-9
    io2.move_yaw(500.0)
    assert io.read_yaw() == BASE_YAW_LIMIT_DEG, io.read_yaw()                        # 한계로 자른다
    per = io.calib.per("elbow_flex")
    io.write({"elbow_flex": io.goal()["elbow_flex"] + 0.01 * TICK / per}, 0.1)      # 틱 아래 지령 = 0
    assert abs(io.goal()["elbow_flex"] - io2.goal()["elbow_flex"]) < 1e-9
    assert sum(1 for _ in open(LOG)) >= 4
    print("twin_io OK  처짐 lift %.2f° elbow %.2f°  err %.1fmm" % (
        45.0 - got["shoulder_lift"], 0.0 - got["elbow_flex"], v["err_mm"]))


if __name__ == "__main__":
    _selftest()
