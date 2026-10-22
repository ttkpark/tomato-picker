#!/usr/bin/env python3
"""**집 안 공간 지도** — 차체 앞 Astra 깊이로 3D 점구름 + 2D 점유격자를 쌓고,
지도 바닥을 찍으면 거기까지 길을 짜서 톡톡 간다.

    ~/lerobot/.venv/bin/python ros2/tools/space_map.py snap          # 안 움직이고 한 장
    ~/lerobot/.venv/bin/python ros2/tools/space_map.py scan          # 제자리 한 바퀴
    ~/lerobot/.venv/bin/python ros2/tools/space_map.py go --x 1.5 --y 0.3
    ~/lerobot/.venv/bin/python ros2/tools/space_map.py clear
    python ros2/tools/space_map.py selftest                          # PC, 장치 없이

조작대(click_server, job=space)와 이 명령줄이 같은 경로다. 왜·한계 = docs/space-map.md.

⚠ **자세는 바퀴가 아니라 지도와 맞춰서 안다** — 이 차체엔 엔코더가 없고, 넛지 한 번이
  얼마나 가는지는 정지마찰이 정한다(CLAUDE.md 1번 병). 깊이 한 장을 바닥 위 띠만
  잘라 2D 스캔으로 접고, 지도 우도장에 가장 잘 맞는 (x,y,θ)를 전수 탐색한다
  (상관 스캔매칭, Olson 2009 / Hector SLAM 계열).
⚠ **바닥은 안 보인다** — 마루가 IR을 비껴 반사해 깊이가 0이다(2026-10-22 실측, 아래
  절반이 통째로 비었다). 그래서 빈 칸은 바닥이 아니라 **장애물까지 쏜 광선**으로만 칠한다.
⚠ **Astra는 60cm 안을 못 본다** — 가까운 장애물은 멀리서 봐 둔 지도로만 피한다.
⚠ 주행 포트는 한 프로세스 — tomato-voice·base_nudge와 동시에 못 돈다.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))

RES = 0.05                     # 칸 5cm
N = 300                        # 15m × 15m, 원점 = 가운데
OCC, FREE = 0.6, 0.6           # 로그오즈 문턱
Q_MIN = 0.3                    # 맞춤 점수(점당 우도 평균) — 이보다 낮으면 그 장은 지도에 안 넣는다
MAP = os.path.expanduser("~/space_map.npz")
MOUNT_FILE = os.path.expanduser("~/space_mount.json")
# 장착값 — 케이스 CAD(tomato_case.scad astra_*) 기준 추정. **실측해서 MOUNT_FILE로 덮어라.**
# x=회전 중심→렌즈(앞+), h=바닥→렌즈, tilt=위로+, floor_cut/top=장애물로 칠 높이 띠, radius=차체 반지름
MOUNT = {"x": 0.12, "h": 0.15, "tilt_deg": 0.0, "floor_cut": 0.05, "top": 0.9, "radius": 0.22}


def mount() -> dict:
    m = dict(MOUNT)
    try:
        m.update(json.load(open(MOUNT_FILE)))
    except (OSError, ValueError):
        pass
    return m


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def idx(xy) -> np.ndarray:
    return np.floor(np.asarray(xy, float) / RES).astype(int) + N // 2


# ── 깊이 → 점 ──────────────────────────────────────────────────────────
def cloud(depth: np.ndarray, meta: dict, m: dict, stride: int = 3) -> np.ndarray:
    """깊이 → 차체 좌표 점 (x 앞, y 왼, z 위, m). 단위·유효거리는 **meta가 말한다**."""
    it = meta["intrinsics"]
    d = depth[::stride, ::stride].astype(np.float32) * (float(meta["depth_scale_mm"]) / 1000.0)
    v, u = np.nonzero((d >= meta["min_mm"] / 1000.0) & (d <= meta["max_mm"] / 1000.0))
    Z = d[v, u]
    X = (u * stride - it["ppx"]) * Z / it["fx"]
    Y = (v * stride - it["ppy"]) * Z / it["fy"]
    t = math.radians(m["tilt_deg"])
    c, s = math.cos(t), math.sin(t)
    return np.stack([m["x"] + Z * c + Y * s, -X, m["h"] + Z * s - Y * c], 1)


def scan2d(pts: np.ndarray, m: dict) -> np.ndarray:
    """높이 띠 안의 점을 0.5° 부채살마다 **가장 가까운 것 하나**로 접는다 → (K,2) 차체 좌표."""
    p = pts[(pts[:, 2] > m["floor_cut"]) & (pts[:, 2] < m["top"])]
    if len(p) == 0:
        return np.zeros((0, 2))
    dx = p[:, 0] - m["x"]
    a = np.round(np.degrees(np.arctan2(p[:, 1], dx)) * 2).astype(int)
    o = np.lexsort((np.hypot(dx, p[:, 1]), a))
    _, first = np.unique(a[o], return_index=True)
    return p[o[first], :2]


# ── 지도 ───────────────────────────────────────────────────────────────
def empty() -> dict:
    return {"lo": np.zeros((N, N), np.float32), "vox": np.zeros((0, 3), np.int16),
            "pose": np.zeros(3), "q": 0.0, "frames": 0,
            "path": np.zeros((0, 2)), "goal": np.zeros(0)}


def load() -> dict:
    s = empty()
    try:
        with np.load(MAP) as z:
            for k in s:
                s[k] = z[k] if z[k].ndim else z[k].item()
    except (OSError, KeyError, ValueError):
        pass
    s["pose"] = np.asarray(s["pose"], float)
    return s


def save(s: dict) -> None:
    tmp = MAP + ".tmp.npz"
    np.savez_compressed(tmp, **{k: np.asarray(v) for k, v in s.items()})
    os.replace(tmp, MAP)          # 조작대가 반쯤 쓴 파일을 읽지 않게


def to_world(pose, p: np.ndarray) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    out = p.copy()
    out[:, 0] = x + c * p[:, 0] - s * p[:, 1]
    out[:, 1] = y + s * p[:, 0] + c * p[:, 1]
    return out


def integrate(s: dict, scan: np.ndarray, pts: np.ndarray, m: dict) -> None:
    """광선으로 빈 칸을, 끝점으로 장애물을 칠하고 3D 점을 5cm 복셀로 더한다."""
    lo = s["lo"]
    if len(scan):
        o = to_world(s["pose"], np.array([[m["x"], 0.0]]))[0]
        hits = to_world(s["pose"], scan)
        d = hits - o
        L = np.hypot(d[:, 0], d[:, 1])
        f = np.arange(0.0, 1.0, RES / 2 / max(float(L.max()), RES))
        ray = o + d[:, None, :] * f[None, :, None]
        keep = f[None, :] * L[:, None] < L[:, None] - RES
        fi, hi = idx(ray[keep]), idx(hits)
        fi = fi[((fi >= 0) & (fi < N)).all(1)]
        hi = hi[((hi >= 0) & (hi < N)).all(1)]
        ff = np.setdiff1d(np.unique(fi[:, 0] * N + fi[:, 1]), hi[:, 0] * N + hi[:, 1])
        lo.flat[ff] -= 0.4
        lo.flat[np.unique(hi[:, 0] * N + hi[:, 1])] += 0.9
        np.clip(lo, -4.0, 4.0, out=lo)
    if len(pts):
        k = np.floor(to_world(s["pose"], pts) / RES).astype(np.int16)
        s["vox"] = np.unique(np.concatenate([s["vox"], k]), axis=0)


def field(lo: np.ndarray) -> np.ndarray:
    """장애물 칸 1, 이웃으로 갈수록 0.6배씩 — 맞춤의 우도장."""
    f = (lo > OCC).astype(np.float32)
    for _ in range(3):
        g = f.copy()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            g = np.maximum(g, np.roll(np.roll(f, dx, 0), dy, 1) * 0.6)
        f = g
    return f


def match(s: dict, scan: np.ndarray, prior, win_xy: float, win_deg: float):
    """prior 둘레를 전수 탐색해 (자세, 점수) — 지도가 비었으면 (prior, 0)."""
    F = field(s["lo"])
    prior = np.asarray(prior, float)
    if F.max() < 1 or len(scan) < 15:
        return prior, 0.0
    oi = np.arange(-round(win_xy / RES), round(win_xy / RES) + 1)
    best, bp = -1.0, prior
    for dth in np.radians(np.arange(-win_deg, win_deg + 1e-9, 1.0)):
        p = idx(to_world((prior[0], prior[1], prior[2] + dth), scan))
        IX = p[:, 0, None, None] + oi[None, :, None]
        IY = p[:, 1, None, None] + oi[None, None, :]
        ok = (IX >= 0) & (IX < N) & (IY >= 0) & (IY < N)
        sc = np.where(ok, F[IX.clip(0, N - 1), IY.clip(0, N - 1)], 0).sum(0)
        i, j = np.unravel_index(int(sc.argmax()), sc.shape)
        if sc[i, j] > best:
            best = float(sc[i, j])
            bp = np.array([prior[0] + oi[i] * RES, prior[1] + oi[j] * RES, wrap(prior[2] + dth)])
    return bp, best / len(scan)


# ── 길 ─────────────────────────────────────────────────────────────────
def dilate(mask: np.ndarray, r: int) -> np.ndarray:
    out = mask.copy()
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if 0 < dx * dx + dy * dy <= r * r:
                out |= np.roll(np.roll(mask, dx, 0), dy, 1)
    return out


def plan(lo: np.ndarray, start, goal, radius: float):
    """A*(8방향). 장애물은 차체 반지름만큼 부풀리고, 모르는 칸은 1.5배 비싸게 간다.
    → ([[x,y],...], "") 또는 (None, 이유)."""
    occ = lo > OCC
    r = int(math.ceil(radius / RES))
    block = dilate(occ, r)
    si, gi = tuple(int(v) for v in idx(start)), tuple(int(v) for v in idx(goal))
    if not all(0 <= v < N for v in si + gi):
        return None, "지도 밖이다"
    if block[gi]:
        return None, "목표가 장애물에 너무 가깝다(차체 반지름 %.2fm)" % radius
    # 벽 옆에서 출발해도 빠져나오게 — 출발 둘레는 부풀림을 풀고 진짜 장애물 칸만 남긴다
    x0, y0 = si
    win = (slice(max(0, x0 - r), x0 + r + 1), slice(max(0, y0 - r), y0 + r + 1))
    block[win] = occ[win]
    block[si] = False
    cost = np.where(lo < -FREE, 1.0, 1.5)
    D2 = math.sqrt(2)
    g = {si: 0.0}
    came = {}
    h = lambda a: max(abs(a[0] - gi[0]), abs(a[1] - gi[1])) + (D2 - 1) * min(abs(a[0] - gi[0]), abs(a[1] - gi[1]))  # noqa: E731
    pq = [(h(si), si)]
    while pq:
        _, c = heapq.heappop(pq)
        if c == gi:
            break
        gc = g[c]
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            n = (c[0] + dx, c[1] + dy)
            if not (0 <= n[0] < N and 0 <= n[1] < N) or block[n]:
                continue
            gn = gc + (D2 if dx and dy else 1.0) * cost[n]
            if gn < g.get(n, 1e18):
                g[n], came[n] = gn, c
                heapq.heappush(pq, (gn + h(n), n))
    if gi not in g:
        return None, "길이 없다(장애물로 막혔다)"
    cells = [gi]
    while cells[-1] != si:
        cells.append(came[cells[-1]])
    cells = cells[::-1][::3] + [gi]
    path = [[(a - N // 2 + 0.5) * RES, (b - N // 2 + 0.5) * RES] for a, b in cells]
    path[-1] = [float(goal[0]), float(goal[1])]
    return path, ""


# ── 실기 ───────────────────────────────────────────────────────────────
def frame(after: float, timeout: float = 40.0):
    """after 이후에 찍힌 Astra 깊이 한 장. 깊이 0%면 기다린다(발행기가 30초 뒤 스스로 다시 연다)."""
    from tomato_picker.config import DEPTH_CAMERAS
    spec = DEPTH_CAMERAS["astra"]
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            meta = json.load(open(spec["meta"]))
            if float(meta["ts"]) > after and float(meta.get("valid_frac", 0)) > 0.05:
                return np.load(spec["depth"]), meta
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(0.1)
    raise RuntimeError("Astra 새 프레임이 %.0f초째 없다 — astra-cam이 죽었거나 깊이 0%%"
                       "(60cm 안을 봤다)" % timeout)


def localize(s: dict, m: dict, prior, win_xy: float, win_deg: float, after: float) -> float:
    """새 장 하나로 자세를 맞추고, 믿을 만하면 지도에 넣고 저장한다 → 점수."""
    dep, meta = frame(after)
    pts = cloud(dep, meta, m)
    sc = scan2d(pts, m)
    pose, q = match(s, sc, prior, win_xy, win_deg)
    if (s["lo"] > OCC).sum() < 20 or q >= Q_MIN:
        s["pose"] = pose
        integrate(s, sc, pts, m)
        s["frames"] = int(s["frames"]) + 1
    else:
        s["pose"] = np.asarray(prior, float)
        print("  ⚠ 맞춤 %.2f < %.2f — 이 장은 지도에 안 넣는다(자세는 추정값)" % (q, Q_MIN))
    s["q"] = q
    save(s)
    x, y, th = s["pose"]
    print("  자세 x=%.2f y=%.2f θ=%.0f° · 맞춤 %.2f · 스캔 %d점 · 복셀 %d" %
          (x, y, math.degrees(th), q, len(sc), len(s["vox"])))
    return q


def scan_cmd(a, m) -> int:
    from base_nudge import drive, open_link
    s = load()
    localize(s, m, s["pose"], 0.4, 30, time.time())
    link = open_link()
    if link is None:
        return 1
    est, total, stuck = 0.0, 0.0, 0
    try:
        for i in range(a.steps):
            prev = s["pose"].copy()
            drive(link, 0, 0, a.w, a.wsecs)
            localize(s, m, prev + [0, 0, est], 0.15, 45, time.time() + 0.4)
            d = wrap(s["pose"][2] - prev[2])
            print("[%d/%d] 넛지 한 번 = %.0f°" % (i + 1, a.steps, math.degrees(d)))
            if abs(d) < math.radians(2):
                stuck += 1
                if stuck >= 3:
                    print("⛔ 세 번 연속 안 돌았다 — 회전 크기·시간을 올려라(정지마찰)")
                    return 1
                continue
            stuck, est, total = 0, d, total + abs(d)
            if total >= 2 * math.pi:
                print("✅ 한 바퀴(%.0f°)" % math.degrees(total))
                return 0
        print("걸음을 다 썼다 — %.0f° 돌았다" % math.degrees(total))
        return 0
    finally:
        link.close()


def lookahead(path, xy, dist: float):
    for p in path:
        if math.hypot(p[0] - xy[0], p[1] - xy[1]) >= dist:
            return p
    return path[-1]


def go_cmd(a, m) -> int:
    from base_nudge import drive, open_link
    s = load()
    goal = np.array([a.x, a.y])
    # 사람이 손으로 옮겼을 수 있다 — 처음엔 넓게 찾는다
    localize(s, m, s["pose"], 0.5, 40, time.time())
    link = open_link()
    if link is None:
        return 1
    est_w, est_v, stuck = math.radians(20), 0.10, 0
    try:
        for i in range(a.max_steps):
            xy, th = s["pose"][:2], s["pose"][2]
            left = float(np.hypot(*(goal - xy)))
            if left < a.tol:
                print("✅ 도착 — 남은 %.2fm" % left)
                return 0
            path, why = plan(s["lo"], xy, goal, m["radius"])
            if path is None:
                print("⛔", why)
                return 1
            s["path"], s["goal"] = np.array(path), goal
            la = lookahead(path, xy, 0.35)
            err = wrap(math.atan2(la[1] - xy[1], la[0] - xy[0]) - th)
            prev = s["pose"].copy()
            if abs(err) > math.radians(20):
                sg = 1 if err > 0 else -1              # w=+ 가 반시계(왼쪽) — base_nudge 실측
                drive(link, 0, 0, sg * a.w, a.wsecs)
                prior = prev + [0, 0, sg * est_w]
                what = "회전 %s" % ("왼" if sg > 0 else "오른")
            else:
                drive(link, a.vx, 0, 0, a.vsecs)
                prior = prev + [est_v * math.cos(th), est_v * math.sin(th), 0]
                what = "전진"
            localize(s, m, prior, 0.3, 25, time.time() + 0.4)
            dxy = float(np.hypot(*(s["pose"][:2] - prev[:2])))
            dth = wrap(s["pose"][2] - prev[2])
            print("[%d] %s → %.2fm %.0f° · 목표까지 %.2fm" %
                  (i + 1, what, dxy, math.degrees(dth), left))
            if dxy < 0.01 and abs(dth) < math.radians(1):
                stuck += 1
                if stuck >= 3:
                    print("⛔ 세 번 연속 안 움직였다 — 크기·시간을 올려라(정지마찰)")
                    return 1
                continue
            stuck = 0
            if what == "전진":
                est_v = max(0.02, dxy)
            else:
                est_w = max(math.radians(3), abs(dth))
        print("⛔ 걸음을 다 썼다")
        return 1
    finally:
        link.close()


# ── 장치 없이 ──────────────────────────────────────────────────────────
def _room_scan(pose, fov_deg=58.0, m=None):
    """방(-2..3 × -1.5..2) + 상자(1..1.4 × 0.3..0.9)를 2D 광선으로 쏜 차체 좌표 스캔."""
    out = []
    cc, ss = math.cos(pose[2]), math.sin(pose[2])
    ox, oy = pose[0] + m["x"] * cc, pose[1] + m["x"] * ss
    for a in np.radians(np.arange(-fov_deg / 2, fov_deg / 2, 0.5)):
        c, s = math.cos(pose[2] + a), math.sin(pose[2] + a)
        for r in np.arange(0.6, 4.0, 0.01):
            x, y = ox + r * c, oy + r * s
            if not (-2 < x < 3 and -1.5 < y < 2) or (1 < x < 1.4 and 0.3 < y < 0.9):
                dx, dy = x - pose[0], y - pose[1]
                out.append([cc * dx + ss * dy, -ss * dx + cc * dy])
                break
    return np.array(out)


def selftest() -> int:
    m = dict(MOUNT)
    # ① 기하 — 2m 앞 평평한 벽
    meta = {"intrinsics": {"fx": 570.0, "fy": 570.0, "ppx": 319.5, "ppy": 239.5},
            "depth_scale_mm": 1.0, "min_mm": 600.0, "max_mm": 4000.0}
    dep = np.full((480, 640), 2000, np.uint16)
    p = cloud(dep, meta, m, stride=1)
    c = p[240 * 640 + 320]
    assert abs(c[0] - (m["x"] + 2.0)) < 1e-3 and abs(c[1]) < 0.01 and abs(c[2] - m["h"]) < 0.01, c
    assert p[240 * 640, 1] > 1.0, "화면 왼쪽(u=0)은 차체 왼쪽(y+)이어야 한다"
    assert len(scan2d(p, m)) > 100
    # ② 맞춤 — 돌며 지도를 만든 뒤, 틀린 짐작에서 참 자세를 되찾는가
    s = empty()
    for k in range(15):
        s["pose"] = np.array([0.0, 0.0, math.radians(25 * k)])
        integrate(s, _room_scan(s["pose"], m=m), np.zeros((0, 3)), m)
    true = np.array([0.12, -0.08, math.radians(40)])
    got, q = match(s, _room_scan(true, m=m), true + [-0.08, 0.06, math.radians(-12)], 0.15, 25)
    assert np.hypot(*(got[:2] - true[:2])) < 0.06 and abs(wrap(got[2] - true[2])) < math.radians(2), (got, true, q)
    assert q > 0.6, q
    # ③ 길 — 상자를 돌아가고, 벽 속 목표는 거절
    path, why = plan(s["lo"], (0, 0), (2.2, 0.6), m["radius"])
    assert path, why
    occ = dilate(s["lo"] > OCC, 2)
    assert not any(occ[tuple(idx(q_))] for q_ in path), "길이 장애물을 지난다"
    assert plan(s["lo"], (0, 0), (3.0, 0.0), m["radius"])[0] is None
    print("space_map selftest ✅ 맞춤 오차 %.3fm %.1f° · 점수 %.2f · 길 %d점" %
          (np.hypot(*(got[:2] - true[:2])), math.degrees(abs(wrap(got[2] - true[2]))), q, len(path)))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["snap", "scan", "go", "clear", "selftest"])
    ap.add_argument("--steps", type=int, default=16)
    ap.add_argument("--w", type=int, default=150, help="회전 넛지 크기(+=왼쪽)")
    ap.add_argument("--wsecs", type=float, default=0.8)
    ap.add_argument("--vx", type=int, default=150, help="전진 넛지 크기")
    ap.add_argument("--vsecs", type=float, default=0.6)
    ap.add_argument("--x", type=float)
    ap.add_argument("--y", type=float)
    ap.add_argument("--tol", type=float, default=0.2)
    ap.add_argument("--max-steps", type=int, default=60)
    a = ap.parse_args()
    if a.mode == "selftest":
        return selftest()
    if a.mode == "clear":
        for f in (MAP,):
            if os.path.exists(f):
                os.remove(f)
        print("지도를 지웠다 — 다음 장의 자리가 원점(0,0,0°)이 된다")
        return 0
    m = mount()
    print("장착", m, "(바꾸려면 %s)" % MOUNT_FILE)
    if a.mode == "snap":
        s = load()
        localize(s, m, s["pose"], 0.3, 20, time.time())
        return 0
    if a.mode == "scan":
        return scan_cmd(a, m)
    if a.x is None or a.y is None:
        print("go 는 --x --y (지도 좌표, m)가 있어야 한다")
        return 2
    return go_cmd(a, m)


if __name__ == "__main__":
    raise SystemExit(main())
