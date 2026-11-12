#!/usr/bin/env python3
"""**트윈의 눈** — 가짜 팔(twin_io)의 지금 자세에서 손목 D405가 볼 화면을 그려 `depth-cam`과 **같은 파일**로 낸다.

    python ros2/tools/twin_cam.py                       # 자체검증(그린 그림에서 보드를 다시 찾아 모형 예측과 맞는가)
    python ros2/tools/twin_cam.py --compare data/kincal/s6 81 out.jpg     # 실제 사진 ↔ 같은 자세의 가짜 화면 나란히
    (평소엔 twin_server.py가 스레드로 띄운다)

컬러 `/dev/shm/d405_color.jpg` · 깊이 `d405_depth.npy`(0.1mm 단위) · `d405_meta.json` · 보기용 `d405_depth.jpg`.
도구들이 그 경로를 그대로 읽으므로(윈도우에선 `C:\\dev\\shm\\`) **도구는 안 고친다** — target_check·조작대 화면·
handeye_collect가 트윈에서 그대로 돈다.

그리는 것 = 역보정 모형(`~/twin/kincal.json`, kincal_solve)이 아는 것뿐: 책상 평면, ChArUco 종이, (알면) 벽 평면과 HOME 마커.
화소마다 광선을 쏴 가장 가까운 평면을 찾는다 — 깊이가 공짜로 나오고, 평면이 카메라 뒤로 넘어가도 안 깨진다.
⚠ 없는 것: 집게 손가락(실제 화면 아래쪽에 늘 보인다)·토마토·그림자·흐림·노출·깊이 잡음과 구멍.
   그래서 여기서 통과한 **기하**(어디를 보나, 몇 mm인가)는 믿어도 되고, **검출이 되느냐**는 실기에서만 안다.
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kincal_model as km  # noqa: E402

SHM = "/dev/shm"
MODEL = os.environ.get("TOMATO_KINCAL") or os.path.expanduser("~/twin/kincal.json")
STATE = os.environ.get("TOMATO_TWIN_STATE", os.path.expanduser("~/twin_arm.json"))
PX_PER_MM = 6.0                  # 종이 무늬 해상도
DEPTH_SCALE_MM = 0.1
NEAR_MM, FAR_MM = 70.0, 900.0    # D405가 깊이를 주는 범위(depth-cam meta와 같은 값)
DESK, WALL, PAPER = (205, 212, 216), (168, 188, 200), (245, 245, 245)      # BGR
MARKER_ID, MARKER_MM = 49, 160.0


class Scene:
    """모형 하나 → 평면 목록. 평면 = 원점 o, 축 ex·ey(mm), 법선 n, 무늬(없으면 단색), 무늬가 덮는 범위."""

    def __init__(self, M: dict):
        self.M, self.P, self.it = M, M["P"], M["intr"]
        P, s = self.P, M["print_scale"]
        spec = M["charuco"]
        dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.get("dict", "DICT_4X4_50")))
        sq = spec["square_mm"] * s
        W, H = spec["nx"] * sq, spec["ny"] * sq
        board = cv2.aruco.CharucoBoard((spec["nx"], spec["ny"]), sq, spec["marker_mm"] * s, dic)
        tex = board.generateImage((int(round(W * PX_PER_MM)), int(round(H * PX_PER_MM))))
        m = int(15 * PX_PER_MM)                                  # 종이의 흰 여백 15mm
        tex = cv2.copyMakeBorder(tex, m, m, m, m, cv2.BORDER_CONSTANT, value=245)
        Rb, tb = km.board_R(P), np.asarray(P["board_t"], float)
        up = Rb[:, 2] if Rb[2, 2] > 0 else -Rb[:, 2]
        self.planes = [
            {"o": tb, "ex": Rb[:, 0], "ey": Rb[:, 1], "n": up, "color": DESK},                                  # 책상(끝없음)
            {"o": tb - 15.0 * Rb[:, 0] - 15.0 * Rb[:, 1] + 0.3 * up, "ex": Rb[:, 0], "ey": Rb[:, 1], "n": up,    # 보드 종이
             "tex": cv2.cvtColor(tex, cv2.COLOR_GRAY2BGR)},
        ]
        wp = M.get("wallplane")
        if wp:
            n = np.asarray(wp["n"], float)
            ex = np.cross(up, n)
            ex /= np.linalg.norm(ex)
            self.planes.append({"o": n * float(wp["d"]), "ex": ex, "ey": np.cross(n, ex), "n": -n, "color": WALL, "above": (up, float(up @ tb))})
        if P.get("wall_r") is not None:
            h = MARKER_MM * s / 2.0
            mk = cv2.aruco.generateImageMarker(dic, MARKER_ID, int(round(2 * h * PX_PER_MM)))
            mk = cv2.copyMakeBorder(mk, m, m, m, m, cv2.BORDER_CONSTANT, value=245)
            Rw, tw = km.rodrigues(P["wall_r"]), np.asarray(P["wall_t"], float)
            # 마커 틀: x 오른쪽·y 위·z 종이 밖(OpenCV 마커 모서리 순서 = 좌상·우상·우하·좌하) → 무늬는 위에서 아래로
            self.planes.append({"o": tw + Rw @ [-h - 15.0, h + 15.0, 0.6], "ex": Rw[:, 0], "ey": -Rw[:, 1], "n": Rw[:, 2],
                                "tex": cv2.cvtColor(mk, cv2.COLOR_GRAY2BGR)})
        w, h_ = int(self.it["width"]), int(self.it["height"])
        v, u = np.mgrid[0:h_, 0:w]
        self.rays = np.stack([(u - self.it["ppx"]) / (self.it["fx"] * P["f_scale"]),
                              (v - self.it["ppy"]) / (self.it["fy"] * P["f_scale"]), np.ones_like(u, float)], axis=2)
        self.A, self.B = (np.asarray(x, float) for x in M["tick_map"])

    def pose(self, deg) -> tuple:
        """`Calib.to_deg` 도(관절 5개) → 카메라 자세(R_base_cam, t)."""
        c = km.chain(self.P, self.A * np.asarray(deg, float) + self.B)
        return c["R_bc"][0], c["t_bc"][0]

    def render(self, R: np.ndarray, t: np.ndarray):
        """카메라 자세 → (BGR 화면, 깊이 uint16[0.1mm], 유효 비율)."""
        h_, w, _ = self.rays.shape
        d = self.rays @ R.T                                      # base 좌표의 광선(카메라 z = 1)
        best = np.full((h_, w), np.inf)
        img = np.zeros((h_, w, 3), np.uint8)
        for pl in self.planes:
            den = d @ pl["n"]
            with np.errstate(divide="ignore", invalid="ignore"):
                z = ((pl["o"] - t) @ pl["n"]) / den              # 카메라 z 깊이(mm) — 광선의 z 성분이 1이라 그대로다
            hit = (z > 1.0) & (z < best) & (den < 0)             # 앞면만(법선이 카메라를 볼 때)
            if not hit.any():
                continue
            X = t + d[hit] * z[hit][:, None]
            if "above" in pl:                                    # 벽은 책상 위쪽만
                keep = X @ pl["above"][0] >= pl["above"][1]
                idx = np.flatnonzero(hit.ravel())[keep]
                hit = np.zeros(h_ * w, bool)
                hit[idx] = True
                hit = hit.reshape(h_, w)
                X = X[keep]
            if "tex" in pl:
                tex = pl["tex"]
                a = ((X - pl["o"]) @ pl["ex"]) * PX_PER_MM
                b = ((X - pl["o"]) @ pl["ey"]) * PX_PER_MM
                ins = (a >= 0) & (a < tex.shape[1] - 1) & (b >= 0) & (b < tex.shape[0] - 1)
                idx = np.flatnonzero(hit.ravel())[ins]
                hit = np.zeros(h_ * w, bool)
                hit[idx] = True
                hit = hit.reshape(h_, w)
                img[hit] = tex[b[ins].astype(int), a[ins].astype(int)]
            else:
                img[hit] = pl["color"]
            best[hit] = z[hit]
        img = cv2.GaussianBlur(img, (3, 3), 0)                   # 무늬 경계의 계단을 눅인다(검출기가 실제 사진처럼 받게)
        ok = np.isfinite(best) & (best >= NEAR_MM) & (best <= FAR_MM)
        depth = np.where(ok, best / DEPTH_SCALE_MM, 0).astype(np.uint16)
        return img, depth, float(ok.mean())


def _replace(tmp: str, path: str) -> None:
    for _ in range(5):                                           # 윈도우: 읽는 쪽이 파일을 쥐고 있으면 잠깐 막힌다
        try:
            os.replace(tmp, path)
            return
        except PermissionError:
            time.sleep(0.02)


def publish(sc: Scene, img, depth, valid: float, seq: int, full: bool = True) -> None:
    """full=False면 메타(ts·seq)만 새로 쓴다 — 자세가 그대로면 그림은 같다. 윈도우에선 파일을 바꿔치는 순간 읽는 쪽이
    PermissionError를 볼 수 있어(드물다) 바꿔치는 횟수를 줄인다."""
    os.makedirs(SHM, exist_ok=True)
    it = sc.it
    for name, data in () if not full else (("d405_color.jpg", cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()),
                       ("d405_depth.jpg", cv2.imencode(".jpg", cv2.applyColorMap(
                           np.where(depth > 0, 255 - np.clip(depth * DEPTH_SCALE_MM / FAR_MM * 255, 0, 255), 0).astype(np.uint8),
                           cv2.COLORMAP_TURBO))[1].tobytes())):
        with open(os.path.join(SHM, name + ".tmp"), "wb") as fh:
            fh.write(data)
        _replace(os.path.join(SHM, name + ".tmp"), os.path.join(SHM, name))
    if full:
        with open(os.path.join(SHM, "d405_depth.npy.tmp"), "wb") as fh:
            np.save(fh, depth)
        _replace(os.path.join(SHM, "d405_depth.npy.tmp"), os.path.join(SHM, "d405_depth.npy"))
    meta = {"seq": seq, "ts": time.time(), "serial": "twin", "camera": "d405", "twin": True,
            "width": it["width"], "height": it["height"], "intrinsics": it, "depth_scale_mm": DEPTH_SCALE_MM,
            "valid_frac": round(valid, 3), "filters": False, "color_aligned": True, "has_color": True,
            "min_mm": NEAR_MM, "max_mm": FAR_MM, "near_mm": [NEAR_MM, 500.0], "publish_fps": 5.0, "measured_fps": 5.0}
    with open(os.path.join(SHM, "d405_meta.json.tmp"), "w") as fh:
        json.dump(meta, fh)
    _replace(os.path.join(SHM, "d405_meta.json.tmp"), os.path.join(SHM, "d405_meta.json"))


def run(stop: threading.Event | None = None, fps: float = 5.0) -> None:
    """가짜 팔 상태 파일을 지켜보며 계속 낸다. 자세가 안 바뀌어도 메타(ts·seq)는 새로 쓴다 — 읽는 쪽이 '살아 있나'를 그걸로 본다."""
    sc = Scene(json.load(open(MODEL, encoding="utf-8")))
    seq, last, frame = 0, None, None
    while stop is None or not stop.is_set():
        try:
            q = json.load(open(STATE, encoding="utf-8"))["q"]
            key = tuple(round(float(q[j]), 3) for j in km.J)
            moved = key != last
            if moved:
                frame, last = sc.render(*sc.pose(key)), key
            if frame is not None:
                seq += 1
                publish(sc, *frame, seq, full=moved)
        except (OSError, ValueError, KeyError):                  # 상태 파일을 쓰는 중이었다 — 다음 박자에
            pass
        time.sleep(1.0 / fps)


def start() -> threading.Thread | None:
    """twin_server가 부른다. 모형이 없으면 눈 없이 뜬다(예전 그대로)."""
    if not os.path.exists(MODEL):
        return None
    th = threading.Thread(target=run, daemon=True, name="twin_cam")
    th.start()
    return th


def compare(session: str, i: int, out: str) -> None:
    """수집한 실제 사진 한 장 ↔ 같은 관절 읽음에서 그린 가짜 화면. 양쪽에서 보드 모서리를 찾아 화소 차이를 낸다."""
    sc = Scene(json.load(open(MODEL, encoding="utf-8")))
    rec = next(r for r in map(json.loads, open(os.path.join(session, "samples.jsonl"), encoding="utf-8")) if r["i"] == i)
    real = cv2.imread(os.path.join(session, rec["img"]))
    fake, depth, valid = sc.render(*sc.pose(rec["deg"]))
    spec, s = sc.M["charuco"], sc.M["print_scale"]
    board = cv2.aruco.CharucoBoard((spec["nx"], spec["ny"]), spec["square_mm"] * s, spec["marker_mm"] * s,
                                   cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    cc, ci, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(fake)
    got = {} if ci is None else {int(k): p for k, p in zip(ci.ravel(), cc.reshape(-1, 2))}
    d = [np.linalg.norm(got[int(k)] - np.array(v)) for k, v in rec["ch"].items() if int(k) in got]
    msg = "#%d  real %d corners / twin %d" % (i, len(rec["ch"]), len(got))
    if d:
        msg += "  |  same corners %d: median %.1f px, max %.1f px" % (len(d), np.median(d), max(d))
    print(msg)
    for im, label in ((real, "REAL"), (fake, "TWIN")):
        cv2.putText(im, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    cv2.imwrite(out, np.hstack([real, fake]))


def _selftest() -> None:
    """그린 화면에서 ChArUco를 다시 찾으면 모형이 말한 화소에 있고, 깊이는 책상까지의 거리와 맞는가."""
    P = dict(km.DEFAULT, off_lift=3.0, off_elbow=-6.0, off_wrist=10.0, cam_r=[0.02, 0.03, 1.55], cam_t=[54.0, 5.0, -33.0],
             board_yaw=0.25, board_t=[250.0, 20.0, -2.0], f_scale=1.0)
    M = {"P": P, "intr": {"fx": 438.0, "fy": 437.0, "ppx": 422.7, "ppy": 229.3, "width": 848, "height": 480},
         "print_scale": 0.945, "charuco": {"nx": 5, "ny": 7, "square_mm": 35.0, "marker_mm": 26.0, "dict": "DICT_4X4_50"},
         "tick_map": [[1.0] * 5, [0.0] * 5], "wallplane": {"n": [1.0, 0.0, 0.0], "d": 420.0}}
    sc = Scene(M)
    bp = km.target_base(P, "board", km.board_points(M["charuco"], 0.945))
    rng = np.random.default_rng(0)
    board = cv2.aruco.CharucoBoard((5, 7), 35.0 * 0.945, 26.0 * 0.945, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    det = cv2.aruco.CharucoDetector(board)
    seen = 0
    for _ in range(200):                                         # 팔은 거치지 않는다 — 보드를 내려다보는 카메라를 바로 세운다
        t = bp.mean(axis=0) + rng.uniform((-90, -90, 230), (90, 90, 330))
        z = bp.mean(axis=0) + rng.uniform(-25, 25, 3) - t
        z /= np.linalg.norm(z)
        x = np.cross(z, rng.normal(size=3))
        x /= np.linalg.norm(x)
        R = np.stack([x, np.cross(z, x), z], axis=1)
        c = {"R_bc": R[None], "t_bc": t[None]}
        uv, zc = km.project(P, M["intr"], c["R_bc"], c["t_bc"], bp)
        if not (np.all(uv[:, 0] > 40) and np.all(uv[:, 0] < 808) and np.all(uv[:, 1] > 40) and np.all(uv[:, 1] < 440)):
            continue
        img, depth, valid = sc.render(R, t)
        cc, ci, _, _ = det.detectBoard(img)
        assert ci is not None and len(ci) >= 12, (t, None if ci is None else len(ci))
        e = np.linalg.norm(cc.reshape(-1, 2) - uv[ci.ravel()], axis=1)
        assert np.median(e) < 0.9 and e.max() < 1.8, (t, np.median(e), e.max())        # 그림이 모형과 같은 기하다
        k = int(ci.ravel()[0])
        u, v = (int(round(x)) for x in uv[k])
        patch = depth[v - 6:v + 7, u - 6:u + 7].astype(float) * DEPTH_SCALE_MM
        assert abs(np.median(patch[patch > 0]) - zc[k]) < 2.0, (np.median(patch), zc[k])   # 깊이 = 그 점까지의 카메라 z
        seen += 1
        if seen >= 5:
            break
    assert seen >= 3, "보드가 다 보이는 자세를 못 찾았다(%d)" % seen
    print("twin_cam OK  %d자세에서 그린 보드가 모형 예측과 0.9px 안(모형 자체 오차 1.7px보다 작다)" % seen)


if __name__ == "__main__":
    if len(sys.argv) >= 5 and sys.argv[1] == "--compare":
        compare(sys.argv[2], int(sys.argv[3]), sys.argv[4])
    elif len(sys.argv) >= 2 and sys.argv[1] == "--run":
        run()
    else:
        _selftest()
