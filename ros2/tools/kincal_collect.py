#!/usr/bin/env python3
"""**팔 자세 ↔ 손목 카메라 사진 쌍을 무인으로 모은다** (젯슨, 역보정용 원자료).

    sudo systemctl stop click-server                       # 팔 포트는 한 프로세스만
    ~/lerobot/.venv/bin/python ros2/tools/kincal_collect.py --out ~/kincal/s1                 # 씨앗(모형 없이 지금 자리 둘레)
    ~/lerobot/.venv/bin/python ros2/tools/kincal_collect.py --out ~/kincal/s2 --model m.json  # 모형으로 계획(넓게)
    touch ~/kincal/STOP                                     # 곱게 끝내기(대기 자세로 돌아가 토크 켠 채 닫는다)

풀이는 PC에서 `kincal_solve.py`. 표적은 **안 움직이는** 책상 ChArUco(+ 벽 4점), 움직이는 건 팔 관절 5개뿐이다
(집게·회전판·바퀴는 안 건드린다).

한 표본 = 지령(goal)·마지막 접근 방향(dirs)·도착 뒤 읽음 2번(read1 → 사진 → read2)·온도/전압/부하·사진·검출 화소.
**실제 움직임을 담으려고** 접근 방향을 관절마다 무작위로 바꾼다(백래시가 상수로 숨지 않고 드러난다) —
handeye_collect의 "늘 한쪽에서"와 반대인데, 저쪽은 백래시를 없애려는 것이고 이쪽은 재려는 것이다.
20번마다 같은 기준 자세로 돌아가 반복도·표류(브래킷 밀림·열)를 본다.

안전 — 씨앗: 시작 자세보다 낮아지지 않고(도면 FK 기준) 둘레 ±30° 안. 계획: 모형이 푼 책상 평면 위 여유,
보드 바깥 변 너머(벽) 금지, 몸통 뒤 금지, 자기 몸 근처 금지, 가는 길 4°마다 같은 검사.
서보 온도 58°C면 식을 때까지 쉬고, 전압 11.0V 아래·추종 오차 3연속이면 대기 자세로 가서 끝낸다.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "ros2", "src", "tomato_bridge"))
sys.path.insert(0, HERE)

import arm_calib  # noqa: E402
import kincal_model as km  # noqa: E402
import target_check as tc  # noqa: E402

J = km.J
STEP_DEG, STEP_SEC, SETTLE_SEC, BACKOFF_DEG = 10.0, 0.8, 1.0, 6.0
MAX_TRACK_ERR = 6.0          # 평소 0.3~2.1°(처짐). 책상에 닿으면 서보가 밀려 이게 먼저 벌어진다
TEMP_PAUSE, TEMP_RESUME, VOLT_MIN = 58, 50, 110          # °C, °C, 0.1V
ROLL_RANGE = (-70.0, 110.0)                              # 손목 굴림: 11/12 −91.5 지령에 −77.9에서 섰다(카메라 케이블/스토퍼) — 음수 쪽은 −70까지만
SEED_SPAN = np.array([30.0, 25.0, 25.0, 30.0, 35.0])     # 씨앗: 시작 자세에서 이 이상 안 벗어난다
SEED_STEP = np.array([10.0, 8.0, 8.0, 10.0, 20.0])
MIN_CORNERS = 8
STOP_FILE = os.path.expanduser("~/kincal/STOP")
POSE_VIEW = os.path.expanduser("~/arm_pose_view.json")
LEFT_MM = 130.0                                          # 보드를 볼 때 왼쪽(가방 쪽)으로 pan 축에서 이만큼까지만
# 책상 위 여유(mm) — 팔꿈치·손목·무는점·손끝·카메라
CLEAR = {"elbow": 50.0, "wrist": 45.0, "roll": 40.0, "tcp": 35.0, "tip": 30.0, "t_bc": 40.0}   # 투명 관은 신경 쓰지 않는다(사용자 11/12)


class Planner:
    """모형(P)으로 '이 자세가 안전한가 · 표적이 보이나'를 한꺼번에(N자세) 계산한다. 자세는 `Calib.to_deg` 도로 받는다."""

    def __init__(self, P: dict, intr: dict, cal: dict, cart: dict, wallplane: dict | None = None, wall_pts=None):
        self.P, self.intr = P, intr
        self.A, self.B = km.tick_map(cal, cart)
        self.board = km.target_base(P, "board", km.board_points(tc.CHARUCO, tc.PRINT_SCALE))
        Rb, tb = km.board_R(P), np.asarray(P["board_t"], float)
        self.n = Rb[:, 2] if Rb[2, 2] > 0 else -Rb[:, 2]          # 위(base +z 쪽)
        self.d = float(self.n @ tb)
        self.axis = np.array([km.PAN_XY[0], km.PAN_XY[1]])
        u = np.array([*(self.board.mean(axis=0)[:2] - self.axis), 0.0])
        self.u = u / np.linalg.norm(u)
        self.left = np.cross([0.0, 0.0, 1.0], self.u)            # 보드를 보고 섰을 때 왼쪽 — 가방·맥미니가 있는 쪽(11/12 사진)
        # 벽: 깊이 영상으로 찾은 평면(kincal_solve --wallplane, n·X = d)이 있으면 그 45mm 앞까지. 없으면 종이가 벽과 나란하다고
        # 보고 보드의 바깥쪽 두 변을 벽으로 친다(보수적 — 11/12 깊이 실측: 벽은 끝 모서리 줄에서 70mm 너머였다).
        if wallplane:
            self.walls = [(np.asarray(wallplane["n"], float), float(wallplane["d"]) - 45.0)]
        else:
            self.walls = []
            for a in (Rb[:, 0], Rb[:, 1]):
                a = a if a @ self.u >= 0 else -a
                self.walls.append((a, float((self.board @ a).max()) - 10.0))
        self.wall_pts = None if wall_pts is None else np.asarray(wall_pts, float)   # 벽 마커의 네 모서리(풀이가 찾은 것)

    def chain(self, deg) -> dict:
        return km.chain(self.P, self.A * np.atleast_2d(deg) + self.B)

    def _rad(self, X) -> np.ndarray:
        """pan 축에서 잰 수평 거리."""
        return np.hypot(X[:, 0] - self.axis[0], X[:, 1] - self.axis[1])

    def margin(self, deg) -> np.ndarray:
        """자세마다 가장 빠듯한 여유(mm). 0 이상이면 안전."""
        c = self.chain(deg)
        m = np.full(len(c["tcp"]), 1e9)
        foot = np.array([self.axis[0], self.axis[1], 0.0])
        for k, need in CLEAR.items():
            X = c[k]
            m = np.minimum(m, X @ self.n - self.d - need)
            for a, lim in self.walls:
                m = np.minimum(m, lim - X @ a)
            m = np.minimum(m, (X - foot) @ self.u + 100.0)       # 몸통 뒤로 10cm까지만
            m = np.minimum(m, LEFT_MM - (X - foot) @ self.left)
        m = np.minimum(m, 250.0 - self._rad(c["tcp"]))
        # 밑동이 넘어가는 자세(팔 무게중심이 어깨 뒤로)는 안 간다 — 가는 길에 지나가도 밑동이 까딱이며 자리를 옮길 수 있다
        m = np.minimum(m, km.lift_torque(self.P, self.A * np.atleast_2d(deg) + self.B) - km.ROCK_TAU)
        zs = c["shoulder"][:, 2]                                 # 자기 몸(밑동·어깨 서보) 둘레: 가까우면 어깨 축보다 위에 있어야 한다
        for k, r_min, z_min in (("tcp", 95.0, 50.0), ("tip", 95.0, 50.0), ("t_bc", 95.0, 50.0), ("roll", 60.0, 40.0)):
            X = c[k]
            m = np.minimum(m, np.maximum(self._rad(X) - r_min, X[:, 2] - (zs + z_min)))
        return m

    def seen(self, deg, pts=None) -> np.ndarray:
        """자세마다 화면 안(가장자리 25px 안쪽)에 들어오는 표적 점 수. 너무 가깝거나 비스듬하면 0."""
        pts = self.board if pts is None else pts
        c = self.chain(deg)
        N, M = len(c["tcp"]), len(pts)
        idx = np.repeat(np.arange(N), M)
        uv, zc = km.project(self.P, self.intr, c["R_bc"], c["t_bc"], np.tile(pts, (N, 1)), idx)
        ok = ((uv[:, 0] > 25) & (uv[:, 0] < self.intr["width"] - 25) & (uv[:, 1] > 25)
              & (uv[:, 1] < self.intr["height"] - 25) & (zc > 85.0) & (zc < 520.0)).reshape(N, M)
        ray = pts.mean(axis=0) - c["t_bc"]
        cosi = np.abs(ray @ self.n) / np.linalg.norm(ray, axis=1)
        return ok.sum(axis=1) * (cosi > math.cos(math.radians(72.0)) if pts is self.board else 1)

    def path_ok(self, a, b) -> bool:
        a, b = np.asarray(a, float), np.asarray(b, float)
        k = max(2, int(math.ceil(np.abs(b - a).max() / 4.0)) + 1)
        m = self.margin(a + (b - a) * np.linspace(0.0, 1.0, k)[:, None])
        return bool(m.min() >= min(0.0, m[0]) - 1e-6)           # 출발이 이미 빠듯하면 더 나빠지지만 않으면 된다


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--model", default="", help="kincal_solve가 낸 모형(json). 없으면 씨앗 수집")
    ap.add_argument("--near", nargs="*", default=[], help="이 세션들에서 이미 가 본 자세 둘레로만 간다(모형을 멀리까지 안 믿는다)")
    ap.add_argument("--radius", type=float, default=20.0, help="--near: 가 본 자세에서 관절마다 이 도 안")
    ap.add_argument("--n", type=int, default=10 ** 6)
    ap.add_argument("--hours", type=float, default=24.0)
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()
    out = os.path.expanduser(args.out)
    os.makedirs(os.path.join(out, "img"), exist_ok=True)
    os.makedirs(os.path.join(out, "depth"), exist_ok=True)
    rng = np.random.default_rng(args.seed)
    cal = arm_calib.Calib()
    lims = np.array([cal.joint_range(j) for j in J])
    pad = (lims[:, 1] - lims[:, 0]) * arm_calib.LIMIT_MARGIN
    lo, hi = lims[:, 0] + pad, lims[:, 1] - pad
    lo[4], hi[4] = max(lo[4], ROLL_RANGE[0]), min(hi[4], ROLL_RANGE[1])
    meta0 = json.load(open(tc.META))
    intr = meta0["intrinsics"]
    plan = None
    if args.model:
        model = json.load(open(os.path.expanduser(args.model)))
        plan = Planner(model["P"], intr, json.load(open(arm_calib.CAL)), json.load(open(arm_calib.CART)),
                       model.get("wallplane"), (model.get("wall") or {}).get("wall_pts"))
    visited = np.array([json.loads(line)["deg"] for d in args.near
                        for line in open(os.path.join(os.path.expanduser(d), "samples.jsonl"), encoding="utf-8")]
                       or np.zeros((0, 5)))

    def draw(n: int) -> np.ndarray:
        """후보 자세 n개 — 가 본 자세가 있으면 그 둘레(±radius), 없으면 가동범위 전체에서 고르게."""
        if not len(visited):
            return rng.uniform(lo, hi, size=(n, 5))
        base = visited[rng.integers(len(visited), size=n)]
        return np.clip(base + rng.uniform(-args.radius, args.radius, size=(n, 5)), lo, hi)

    detector = cv2.aruco.CharucoDetector(tc.charuco_board())

    from tomato_bridge.follower_io import FollowerIO
    io = FollowerIO(hold_torque=True)

    def vec(d):
        return np.array([float(d[j]) for j in J])

    def dic(v):
        return dict(zip(J, (float(x) for x in v)))

    def reg(name):
        """원격 측정 — **세 번 읽은 중앙값**. 버스가 가끔 엉뚱한 값을 준다(2026-11-12: 40°C 사이에 58·60°C,
        12.1V 사이에 10.8V가 한 번씩 — 그걸 믿고 수집이 두 번 멈췄다)."""
        rs = [io._with_retry(lambda: io._follower.bus.sync_read(name, normalize=False)) for _ in range(3)]   # noqa: SLF001
        return {k: sorted(r[k] for r in rs)[1] for k in rs[0]}

    read0 = io.read()
    goal0 = io.goal()
    park = vec(cal.to_deg(goal0))
    if np.abs(park - vec(cal.to_deg(read0))).max() > 8.0:      # 토크가 꺼져 있었다 — 옛 Goal은 남의 것
        park = vec(cal.to_deg(read0))
        io.write(cal.to_norm(dic(park)), 0.4)
    if plan is None:                                           # 씨앗은 시작 자세 둘레다 — 시작 자세가 여유 밖이어도 거기까진 쓴다
        lo, hi = np.minimum(lo, park), np.maximum(hi, park)
    from tomato_picker.hardware import kinematics as kin

    def nominal(q):
        """모형이 없을 때(씨앗)의 어림 — 도면 평면 FK의 손목·무는점 높이와 수평 거리. 상대 비교에만 쓴다."""
        a1 = np.radians(q[:, 1])
        a2 = a1 + np.radians(q[:, 2])
        a3 = a2 + np.radians(q[:, 3])
        g = kin.ArmGeometry()
        zw = g.z0 + g.l1 * np.sin(a1) + g.l2 * np.sin(a2)
        return {"wrist": zw, "tcp": zw + g.l3 * np.sin(a3),
                "r": g.d0 + g.l1 * np.cos(a1) + g.l2 * np.cos(a2) + g.l3 * np.cos(a3)}

    seed_floor = {k: float(nominal(park[None])[k][0]) - 3.0 for k in ("wrist", "tcp")}

    def seed_ok(q) -> bool:
        q = np.atleast_2d(q)
        c = nominal(q)
        return bool(np.all(q >= lo - 1e-6) and np.all(q <= hi + 1e-6) and np.all(np.abs(q - park) <= SEED_SPAN + 1e-6)
                    and all(np.all(c[k] >= z) for k, z in seed_floor.items()) and np.all(np.abs(c["r"]) <= 210.0))

    def path_ok(a, b) -> bool:
        if plan is not None:
            return plan.path_ok(a, b)
        k = max(2, int(math.ceil(np.abs(b - a).max() / 4.0)) + 1)
        return seed_ok((a + (b - a) * np.linspace(0.0, 1.0, k)[:, None])[1:])    # 출발점은 이미 서 있는 자리다(틱 양자화로 0.1mm 밑일 수 있다)

    json.dump({"started": time.strftime("%Y-%m-%d %H:%M:%S"), "argv": sys.argv[1:], "intrinsics": intr,
               "depth_scale_mm": meta0["depth_scale_mm"], "print_scale": tc.PRINT_SCALE, "charuco": tc.CHARUCO,
               "dots_mm": [tc.EXPECT_W, tc.EXPECT_H], "park_deg": dic(park), "limits_deg": [lo.tolist(), hi.tolist()],
               "cal": json.load(open(arm_calib.CAL)), "cart": json.load(open(arm_calib.CART)),
               "model": json.load(open(os.path.expanduser(args.model))) if args.model else None},
              open(os.path.join(out, "session.json"), "w"), ensure_ascii=False, indent=1)
    log = open(os.path.join(out, "samples.jsonl"), "a", encoding="utf-8")
    i0 = sum(1 for _ in open(os.path.join(out, "samples.jsonl"), encoding="utf-8"))

    at = {"q": park.copy()}          # 마지막으로 보낸 목표 — 버스에서 되읽지 않는다(오독 한 번이 길 검사를 속인다)

    def goto(target) -> None:
        cur = at["q"]
        steps = max(1, int(math.ceil(np.abs(target - cur).max() / STEP_DEG)))
        for s in range(1, steps + 1):
            mid = cur + (target - cur) * s / steps
            io.write(cal.to_norm(dic(mid)), STEP_SEC)
            try:
                json.dump({"deg": dic(mid), "ts": time.time()}, open(POSE_VIEW, "w"))
            except OSError:
                pass
        at["q"] = np.array(target, float)

    def grab(after_ts: float):
        """멈춘 뒤에 찍힌 프레임 하나(메타·원본 jpg·그림). 발행기는 os.replace로 쓰므로 반쪽 파일은 없다."""
        for _ in range(40):
            try:
                m1 = json.load(open(tc.META))
                raw = open(tc.COLOR, "rb").read()
                m2 = json.load(open(tc.META))
            except (OSError, ValueError):
                time.sleep(0.05)
                continue
            if m2["seq"] - m1["seq"] <= 1 and m1["ts"] > after_ts + 0.25:
                img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    return m1, raw, img
            time.sleep(0.1)
        return None

    stop = {"why": ""}

    def on_term(*_):
        stop["why"] = "SIGTERM"
    signal.signal(signal.SIGTERM, on_term)

    # 계획 모드의 기준 자세(hub) = 여유가 가장 넉넉하면서 보드가 잘 보이는 자세. 막힌 길은 여길 거쳐 간다.
    hub = park
    if plan is not None:
        cand = draw(40000)
        good = (plan.margin(cand) > 5.0) & (plan.seen(cand) >= 14)     # 여유가 가장 큰 것을 아래서 고른다(여유엔 토크 항도 섞여 있다)
        if not good.any():
            print("❌ 모형으로는 안전하면서 보드가 보이는 자세가 없다 — 모형이 틀렸다.")
            io.hold_close()
            return 1
        cand = cand[good]
        hub = cand[int(np.argmax(plan.margin(cand)))]
        if not plan.path_ok(park, hub):
            print("❌ 시작 자세에서 기준 자세로 가는 길이 안전하지 않다 — 손으로 팔을 들어 놓고 다시.")
            io.hold_close()
            return 1
        print("기준 자세(hub) " + " ".join("%.1f" % v for v in hub) + "  여유 %.0fmm" % plan.margin(hub[None])[0])

    def next_pose(anchor):
        """→ (목표 관절 도, 종류). 못 찾으면 None."""
        if plan is None:
            for _ in range(200):
                q = np.clip(anchor + rng.uniform(-SEED_STEP, SEED_STEP), np.maximum(lo, park - SEED_SPAN),
                            np.minimum(hi, park + SEED_SPAN))
                if seed_ok(q):
                    return q, "seed"
            return None
        cand = draw(30000)
        safe = plan.margin(cand) >= 0.0
        kind = "board"
        if plan.wall_pts is not None and rng.random() < 0.25:
            pick = safe & (plan.seen(cand, plan.wall_pts) >= 4)
            kind = "wall"
        elif plan.wall_pts is None and rng.random() < 0.15:
            # 벽 4점 종이 찾기: 카메라가 거의 수평으로 보드 바깥(벽 쪽)을 보는 자세
            c = plan.chain(cand)
            ax = c["R_bc"][:, :, 2]
            toward = np.max(np.stack([ax @ a for a, _ in plan.walls]), axis=0)
            pick = safe & (toward > 0.8) & (np.abs(ax @ plan.n) < 0.4) & (c["t_bc"] @ plan.n - plan.d > 110.0)
            kind = "scan"
        else:
            pick = safe & (plan.seen(cand) >= MIN_CORNERS)
        if not pick.any():
            pick, kind = safe & (plan.seen(cand) >= MIN_CORNERS), "board"
        if not pick.any():
            return None
        ids = np.flatnonzero(pick)
        return cand[int(rng.choice(ids))], kind

    t_end = time.time() + args.hours * 3600.0
    anchor, misses, bad_track, i = park.copy(), 0, 0, i0
    skip_ref, rejects = False, 0
    try:
        os.remove(STOP_FILE)
    except OSError:
        pass
    print("수집 시작 — %s · %s · 시작 자세 %s" % (out, "계획(모형)" if plan else "씨앗", " ".join("%.1f" % v for v in park)))
    try:
        while i - i0 < args.n and time.time() < t_end and not stop["why"]:
            if os.path.exists(STOP_FILE):
                stop["why"] = "STOP 파일"
                break
            # ── 건강 검사 ──
            temp, volt, load = reg("Present_Temperature"), reg("Present_Voltage"), reg("Present_Load")
            # ⚠ 전압은 **서보 여섯의 중앙값**으로 본다 — 공급 전압이 처지면 여섯이 다 처진다. 하나만 10.8V인 건 오독이다
            #   (11/12: 474장 중 전 서보 12.0~12.3V, 한 서보만 11.1·11.2·11.5V가 한 번씩. 그 한 번에 3라운드가 멈췄다).
            vmed = sorted(volt.values())[len(volt) // 2]
            if vmed < VOLT_MIN:
                stop["why"] = "전압 %.1fV" % (vmed / 10.0)
                break
            if max(temp.values()) >= TEMP_PAUSE:
                time.sleep(3.0)                                 # 온도는 3초 만에 20°C 못 오른다 — 다시 읽어도 뜨거울 때만 믿는다
                temp = reg("Present_Temperature")
            if max(temp.values()) >= TEMP_PAUSE:
                print("  서보 %d°C — 기준 자세에서 식힌다" % max(temp.values()), flush=True)
                if path_ok(at["q"], hub):
                    goto(hub)
                while max(reg("Present_Temperature").values()) > TEMP_RESUME and not os.path.exists(STOP_FILE):
                    time.sleep(60)
                continue
            # ── 다음 자세 ──
            cur = at["q"]
            if (i - i0) % 20 == 0 and not skip_ref:
                target, kind = hub.copy(), "ref"
                # 기준 자세는 되도록 같은 쪽(+)에서 다가간다. 그 길이 막히면 반대쪽(씨앗에선 +쪽 물러남이 책상 쪽일 수 있다).
                dirs = next((d for d in (np.ones(5), -np.ones(5))
                             if path_ok(cur, target - d * BACKOFF_DEG) and path_ok(target - d * BACKOFF_DEG, target)),
                            np.ones(5))
            else:
                nxt = next_pose(anchor)
                if nxt is None:
                    stop["why"] = "갈 수 있는 자세가 없다"
                    break
                target, kind = nxt
                dirs = rng.choice([-1.0, 1.0], size=5)
            back = target - dirs * BACKOFF_DEG
            flip = (back < lo) | (back > hi)                    # 물러날 자리가 한계 밖이면 그 관절은 반대쪽에서
            dirs[flip] *= -1
            back = target - dirs * BACKOFF_DEG
            via = None
            if not (path_ok(cur, back) and path_ok(back, target)):
                if path_ok(cur, hub) and path_ok(hub, back) and path_ok(back, target):
                    via = hub
                else:
                    skip_ref, rejects = kind == "ref", rejects + 1   # 이 자세는 버리고 다시 뽑는다
                    if rejects > 2000:
                        stop["why"] = "안전한 길을 2000번 못 찾았다"
                    continue
            skip_ref, rejects = False, 0
            if via is not None:
                goto(via)
            goto(back)
            goto(target)
            time.sleep(SETTLE_SEC)
            t_still = time.time()
            read1 = io.read()
            got = grab(t_still)
            read2 = io.read()
            if got is None:
                print("  프레임을 못 받았다 — depth-cam이 도는가", flush=True)
                time.sleep(5)
                continue
            meta, raw, img = got
            deg = vec(cal.to_deg(read2))
            err = float(np.abs(deg - target).max())
            corners, ids, _, _ = detector.detectBoard(img)
            ch = {} if ids is None else {str(int(k)): [float(u), float(v)]
                                         for k, (u, v) in zip(ids.ravel(), corners.reshape(-1, 2))}
            dots = [] if len(ch) >= 4 else [[float(u), float(v)] for u, v in tc.find_dots(img)]
            name = "img/%06d.jpg" % i
            with open(os.path.join(out, name), "wb") as fh:
                fh.write(raw)
            depth = ""
            if (i - i0) % 25 == 0 or len(dots) >= 3:
                depth = "depth/%06d.npz" % i
                np.savez_compressed(os.path.join(out, depth), d=np.load(tc.DEPTH))
            rec = {"i": i, "t": round(time.time(), 2), "kind": kind, "goal_deg": target.round(3).tolist(),
                   "dirs": dirs.tolist(), "via": via is not None,
                   "read1": {j: round(read1[j], 4) for j in J}, "read2": {j: round(read2[j], 4) for j in J},
                   "deg": deg.round(4).tolist(), "gripper": round(read2.get("gripper", 0.0), 2), "err_deg": round(err, 2),
                   "temp": list(temp.values()), "volt": list(volt.values()), "load": list(load.values()),
                   "seq": meta["seq"], "frame_ts": meta["ts"], "exposure": meta.get("exposure"),
                   "ch": ch, "dots": dots, "img": name, "depth": depth}
            log.write(json.dumps(rec, ensure_ascii=False) + "\n")
            log.flush()
            print("%5d %-5s %s  모서리 %2d 점 %d  오차 %.1f°  %d°C %.1fV" % (
                i, kind, " ".join("%6.1f" % v for v in target), len(ch), len(dots), err,
                max(temp.values()), vmed / 10.0), flush=True)
            i += 1
            if err > MAX_TRACK_ERR:
                bad_track += 1
                print("  ⚠ 지령과 %.1f° — 서보가 놓았다(%d연속). 기준 자세로." % (err, bad_track), flush=True)
                if bad_track >= 3:
                    stop["why"] = "추종 오차 3연속"
                    break
                goto(hub)
                time.sleep(20)
                continue
            bad_track = 0
            if plan is None and kind == "seed":
                if len(ch) >= 6:
                    misses = 0
                    if rng.random() < 0.7:
                        anchor = target
                else:
                    misses += 1
                    if misses >= 5:
                        anchor, misses = park.copy(), 0
    except KeyboardInterrupt:
        stop["why"] = "Ctrl-C"
    except Exception as e:                                      # noqa: BLE001 — 팔은 토크 켠 채 그 자리에 둔다
        print("❌ 예외로 중단: %r — 팔은 그 자리에 붙들어 둔다." % (e,), flush=True)
        io.hold_close()
        raise
    print("끝 — %s · 이번 %d장(누적 %d)" % (stop["why"] or "목표 도달", i - i0, i), flush=True)
    cur = at["q"]
    if path_ok(cur, hub) and path_ok(hub, park):
        goto(hub)
        goto(park)
        print("시작 자세로 돌아왔다. 토크는 켠 채 둔다.")
    else:
        print("⚠ 시작 자세로 가는 길이 안전하지 않아 그 자리에 둔다(토크 켬).")
    io.hold_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
