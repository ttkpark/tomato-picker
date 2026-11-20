"""스프링 처짐 측정 — 모니터 격자 앞에서 찍은 영상에서 스프링 중심선을 mm로 뽑는다.

  python tools/spring_sag_measure.py            # docs/2026-10-21_스프링-분석/2026-11-13_spring-sag-data.js + 확인 그림
  python tools/spring_sag_measure.py --selftest # 격자 맞춤 수학만(영상 없이)

방법: ① 빈 화면 프레임에 격자 모형(호모그래피 + 방사 왜곡)을 맞춘다 — 격자 한 칸 = 80px,
5칸 = 108.0mm(자로 잰 값). ② 빈 화면과의 차분으로 스프링+손을 떼고, 큰 덩어리(손)를 지운다.
③ 열마다 띠의 가운데 → 격자 좌표 → mm.
한계: 스프링이 화면에서 뜬 만큼 시차로 커 보인다(붙여서 찍었다). 손 밑은 안 보여 잇는다.
"""
import json
import os
import sys

import numpy as np

CELL_MM = 108.0 / 5          # 실측: 5칸 = 108.0mm
MEDIA = "docs/2026-10-21_스프링-분석/spring-media/휘어짐/"
V2 = (MEDIA + "WIN_20261113_17_38_25_Pro.mp4", 0.5, 1.0)    # (영상, 빈 화면 시각 초, 확대) — 영상마다 카메라 자리가 달라 격자를 따로 맞춘다
V3 = (MEDIA + "WIN_20261113_17_54_15_Pro.mp4", 30.0, 1.0)
V4 = (MEDIA + "20261114_081106.mp4", 4.0, 1.5)             # 휴대폰 720p — 화소 문턱들이 1080p에 맞춰져 있어 1.5배로 키워 쓴다
# 장면: 이름 → (영상, 시각 초, 설명). 시각은 1초 간격으로 훑어 흔들림이 가라앉은 곳을 고른 것.
SHOTS = {
    "single_tip": (V3, 75.0, "한 개 · 손끝 받침"),
    "double_tip": (V3, 115.0, "두 개 겹침 · 손끝 받침"),
    "single_center": (V2, 9.0, "한 개 · 손가락으로 집음"),
    "double_center": (V2, 108.2, "두 개 겹침 · 손가락으로 집음"),
    "single_ends": (V2, 44.0, "한 개 · 양끝"),
    # 11/14: 로봇이 잡을 자리(양 끝)를 손끝으로 잡고 수평으로 든 장면. 손이 안 가리고 가라앉은 시각만 골랐다
    "bundle4_a_ends": (V4, 12.0, "4개 묶음(검정) · 양끝 ①"),
    "bundle4_b_ends": (V4, 32.0, "4개 묶음(검정) · 양끝 ②"),
    "bundle4_c_ends": (V4, 80.0, "4개 묶음(검정) · 양끝 ③"),
    "white_a_ends": (V4, 48.0, "흰색 · 양끝 ①"),
    "white_b_ends": (V4, 56.0, "흰색 · 양끝 ②"),
    "white_c_ends": (V4, 58.0, "흰색 · 양끝 ③"),
    "white_d_ends": (V4, 66.0, "흰색 · 양끝 ④"),
}
SUPPORT_HALF_MM = 12.0       # 손끝 받침 좌우로 이만큼은 손가락에 가려 팔에서 뺀다
OUT_JS = "docs/2026-10-21_스프링-분석/2026-11-13_spring-sag-data.js"


def project(p, XY):
    """격자 좌표(칸, 중심 십자 = 0,0) → 카메라 화소."""
    h = np.append(p[:8], 1.0).reshape(3, 3)
    q = np.c_[XY, np.ones(len(XY))] @ h.T
    u = q[:, :2] / q[:, 2:3]
    r2 = (u ** 2).sum(1, keepdims=True) / 1e6
    return p[8:10] + u * (1 + p[10] * r2 + p[11] * r2 ** 2)


def unproject(p, uv):
    """카메라 화소 → 격자 좌표. project를 뉴턴으로 뒤집는다."""
    uv = np.asarray(uv, float)
    XY = (uv - p[8:10] - [p[2], p[5]]) / np.hypot(p[0], p[3])   # 대충의 첫 값
    for _ in range(30):
        f = project(p, XY) - uv
        e = 1e-4
        jx = (project(p, XY + [e, 0]) - project(p, XY)) / e
        jy = (project(p, XY + [0, e]) - project(p, XY)) / e
        det = jx[:, 0] * jy[:, 1] - jx[:, 1] * jy[:, 0]
        XY = XY - np.c_[(jy[:, 1] * f[:, 0] - jy[:, 0] * f[:, 1]) / det,
                        (-jx[:, 1] * f[:, 0] + jx[:, 0] * f[:, 1]) / det]
    return XY


def grid_points(rx, ry, step=0.1):
    t = np.arange(-rx, rx + 1e-9, step)
    s = np.arange(-ry, ry + 1e-9, step)
    v = [np.c_[np.full_like(s, i), s] for i in range(-int(rx), int(rx) + 1)]
    h = [np.c_[t, np.full_like(t, j)] for j in range(-int(ry), int(ry) + 1)]
    return np.vstack(v + h)


def fit_grid(dist, center, scale=83.0):
    """dist = 선까지 거리 그림(화소). 가운데서부터 넓혀 가며 맞춘다(한 칸 밀림 방지)."""
    from scipy.ndimage import map_coordinates
    from scipy.optimize import least_squares
    H, W = dist.shape
    p = np.array([scale, 0, center[0] - W / 2, 0, scale, center[1] - H / 2, 0, 0, W / 2, H / 2, 0, 0], float)

    def res(q, XY):
        uv = project(q, XY)
        d = map_coordinates(dist, [uv[:, 1], uv[:, 0]], order=1, mode="constant", cval=8.0)
        return np.minimum(d, 8.0)

    for rx, ry, free in ((3, 3, 6), (6, 4, 8), (9, 5, 12), (11, 6, 12)):
        XY = grid_points(rx, ry)
        idx = list(range(8)) + ([10, 11] if free > 8 else [])
        idx = idx[:free] if free <= 8 else idx

        def f(z):
            q = p.copy(); q[idx] = z
            return res(q, XY)
        p[idx] = least_squares(f, p[idx], loss="soft_l1", f_scale=1.5, x_scale="jac").x
    return p, float(np.mean(res(p, grid_points(11, 6)) < 2.0))


def elastica(k, th0, L, n=60):
    """자중으로 처지는 외팔보(큰 처짐). k = 단위길이 무게/휨강성 [1/mm^3], th0 = 뿌리 기울기(아래가 +, rad).
    뿌리는 보의 아무 점이어도 된다 — 그 점에서 자유단까지는 그대로 외팔보다."""
    s = np.linspace(0, L, n)
    ds = s[1] - s[0]
    cum = lambda v: np.r_[0, np.cumsum((v[1:] + v[:-1]) / 2) * ds]
    phi = np.full(n, th0)
    for _ in range(60):
        x = cum(np.cos(phi))
        tail = cum(x)                                   # M(s) = w * [ int_s^L x dt - x(s)(L-s) ]
        mom = (tail[-1] - tail) - x * (L - s)
        phi = 0.5 * phi + 0.5 * (th0 + k * cum(mom))
    return cum(np.cos(phi)), -cum(np.sin(phi))


def fit_arm(xy):
    """xy = 뿌리에서 자유단 쪽으로 정렬된 팔의 점(mm, 뿌리 = 0,0 근처, 바깥이 +x). → k, th0, rms."""
    from scipy.optimize import least_squares
    co = np.polyfit(xy[:, 0], xy[:, 1], 3)
    xs = np.linspace(xy[0, 0], xy[-1, 0], 50)
    L = float(np.hypot(np.diff(xs), np.diff(np.polyval(co, xs))).sum())

    def res(z):
        mx, my = elastica(z[0] * 1e-7, z[1], L)
        return np.interp(xy[:, 0] - xy[0, 0], mx, my) + z[2] - xy[:, 1]
    z = least_squares(res, [5.0, 0.1, xy[0, 1]]).x
    return z[0] * 1e-7, z[1], L, float(np.sqrt(np.mean(res(z) ** 2)))


def selftest():
    mx, my = elastica(8e-7, 0.05, 140.0)                # 만든 처짐을 되찾는가
    k, th, L, rms = fit_arm(np.c_[mx, my][np.r_[0:60:3, 59]])  # 끝점 포함(길이 3% = k 10%)
    assert abs(k / 8e-7 - 1) < 0.03 and abs(th - 0.05) < 0.01, (k, th)
    small = elastica(1e-9, 0.0, 100.0)[1][-1]            # 작은 처짐 = wL^4/8EI
    assert abs(small / -(1e-9 * 100.0 ** 4 / 8) - 1) < 0.01, small
    true = np.array([80, 1.5, 40, -1.2, 81, 30, 2e-5, -1e-5, 960, 540, -0.12, 0.02])
    XY = np.random.default_rng(0).uniform(-10, 10, (200, 2))
    back = unproject(true, project(true, XY))
    assert np.abs(back - XY).max() < 1e-6, np.abs(back - XY).max()
    print("selftest ok - project<->unproject 왕복 오차 < 1e-6칸")


def grab(cap, sec, zoom=1.0):
    import cv2
    cap.set(0, sec * 1000)
    ok, f = cap.read()
    assert ok, f"{sec}s 프레임을 못 읽음"
    return f if zoom == 1.0 else cv2.resize(f, None, fx=zoom, fy=zoom, interpolation=cv2.INTER_CUBIC)


def centerline(cv2, empty, frame):
    """빈 화면과의 차분에서 스프링 띠의 가운데 화소 줄. 손(큰 덩어리)은 지운다."""
    g0 = cv2.GaussianBlur(cv2.cvtColor(empty, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.int16)
    g1 = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.int16)
    fg = ((g0 - g1) > 28).astype(np.uint8)
    k = lambda n: cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))
    band = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, k(17))
    hand = cv2.dilate(cv2.morphologyEx(band, cv2.MORPH_OPEN, k(75)), k(61))
    spring = cv2.morphologyEx(band & (1 - hand), cv2.MORPH_OPEN, k(9))
    n, lab, st, _ = cv2.connectedComponentsWithStats(spring)
    keep = np.isin(lab, [i for i in range(1, n) if st[i, 4] > 1500 and st[i, 2] > 60])
    pts = []
    for x in range(0, keep.shape[1], 6):
        ys = np.flatnonzero(keep[:, x])
        if len(ys) > 8:
            pts.append((x, (ys[0] + ys[-1]) / 2, ys[-1] - ys[0]))
    pts = np.array(pts).reshape(-1, 3)
    if len(pts):                                         # 밑에서 받친 손가락이 붙은 열은 띠가 두꺼워진다 → 뺀다
        pts = pts[pts[:, 2] < 1.45 * np.median(pts[:, 2])]
    return pts, keep, hand


def fit_video(cv2, cap, empty_s, out_dir, tag, zoom=1.0):
    """빈 화면 프레임 → 격자 모형 p."""
    empty = grab(cap, empty_s, zoom)
    gray = cv2.cvtColor(empty, cv2.COLOR_BGR2GRAY)
    lines = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))) > 18
    dist = cv2.distanceTransform((~lines).astype(np.uint8), cv2.DIST_L2, 3).astype(np.float32)
    b, g, r = [c.astype(np.int16) for c in cv2.split(empty)]   # 중심 십자 = 빨간 두 굵은 선의 교점
    red = (r - np.maximum(g, b)) > 60
    cx, cy = int(np.argmax(red.sum(0))), int(np.argmax(red.sum(1)))
    p, hit = fit_grid(dist, (cx, cy))
    print(f"격자 맞춤: 중심 화소 ({cx},{cy}) · 격자점이 선 위(2px 안)에 놓인 비율 {hit:.0%}")
    assert hit > 0.6, "격자 맞춤 실패 — 빈 화면 시각이나 격자 그림을 확인"
    if out_dir:
        ov = empty.copy()
        for q in project(p, grid_points(11, 6, 0.05)).astype(int):
            cv2.circle(ov, tuple(int(v) for v in q), 1, (0, 255, 0), -1)
        cv2.imwrite(f"{out_dir}/grid_fit_{tag}.jpg", ov)
    return empty, p


def main():
    import cv2
    out_dir = os.environ.get("SAG_DEBUG", "")
    vids = {}
    data = {}
    for name, ((video, empty_s, zoom), sec, label) in SHOTS.items():
        if video not in vids:
            cap = cv2.VideoCapture(video)
            assert cap.isOpened(), video
            vids[video] = (cap,) + fit_video(cv2, cap, empty_s, out_dir, len(vids), zoom)
        cap, empty, p = vids[video]
        frame = grab(cap, sec, zoom)
        pts, keep, hand = centerline(cv2, empty, frame)
        assert len(pts) > 20, f"{name}: 스프링을 못 찾음"
        mm = unproject(p, pts[:, :2]) * CELL_MM
        mm[:, 1] *= -1                                    # 화면 아래 = −y(위가 +)
        mm = mm[np.argsort(mm[:, 0])]
        # 손 밑 빈 구간은 잇고, 매끄럽게: 호 길이가 아니라 x에 대한 5차 맞춤(처짐 곡선은 완만하다)
        co = np.polyfit(mm[:, 0], mm[:, 1], 5)
        xs = np.linspace(mm[0, 0], mm[-1, 0], 61)
        ys = np.polyval(co, xs)
        rms = float(np.sqrt(np.mean((np.polyval(co, mm[:, 0]) - mm[:, 1]) ** 2)))
        top = int(np.argmax(ys))
        arc = float(np.hypot(np.diff(xs), np.diff(ys)).sum())
        gap = float(np.max(np.diff(mm[:, 0])))
        edge = unproject(p, np.c_[pts[:, 0], pts[:, 1] - pts[:, 2] / 2]) - unproject(p, np.c_[pts[:, 0], pts[:, 1] + pts[:, 2] / 2])
        width = float(np.median(np.hypot(edge[:, 0], edge[:, 1])) * CELL_MM)
        d = {"label": label, "sec": sec, "arc_mm": round(arc, 1), "fit_rms_mm": round(rms, 2),
             "hidden_gap_mm": round(gap, 1), "band_mm": round(width, 1),
             "left_drop_mm": round(float(ys[top] - ys[0]), 1), "left_reach_mm": round(float(xs[top] - xs[0]), 1),
             "right_drop_mm": round(float(ys[top] - ys[-1]), 1), "right_reach_mm": round(float(xs[-1] - xs[top]), 1),
             # 3D용: 꼭대기를 원점으로 옮긴 중심선
             "pts": [[round(float(x - xs[top]), 2), round(float(y - ys[top]), 2)] for x, y in zip(xs, ys)]}
        # 3D에서 "본 것"과 "이은 것"을 가르도록: 가려진 구간(가장 큰 빈틈)
        gi = int(np.argmax(np.diff(mm[:, 0])))
        lo, hi = float(mm[gi, 0]), float(mm[gi + 1, 0])
        if hi - lo < 2 * SUPPORT_HALF_MM:                 # 손끝 받침: 가려진 틈이 작다 → 꼭대기 좌우로 가른다
            lo, hi = float(xs[top]) - SUPPORT_HALF_MM, float(xs[top]) + SUPPORT_HALF_MM
        d["hidden_x"] = [round(lo - float(xs[top]), 1), round(hi - float(xs[top]), 1)]
        if not name.endswith("ends"):                     # 보이는 팔마다 자중 처짐 모형을 맞춘다
            d["arms"] = []
            for side, run in (("left", mm[mm[:, 0] <= lo][::-1] * [-1, 1]), ("right", mm[mm[:, 0] >= hi])):
                run = run - [run[0, 0], 0]
                if run[-1, 0] < 80:                       # 짧으면 기울기와 강성을 못 가른다
                    d["arms"].append({"side": side, "seen_mm": round(float(run[-1, 0]), 1), "fit": None})
                    continue
                k, th, L, r = fit_arm(run)
                d["arms"].append({"side": side, "seen_mm": round(L, 1), "k_1e7": round(k * 1e7, 2),
                                  "root_tilt_deg": round(float(np.degrees(th)), 1), "rms_mm": round(r, 2),
                                  "drop_mm": round(float(run[0, 1] - run[-1, 1]), 1),
                                  # 같은 잣대: 수평으로 물린 140mm 외팔보의 끝 처짐
                                  "droop140_mm": round(float(-elastica(k, 0.0, 140.0)[1][-1]), 1)})
        if name.endswith("ends"):                         # 양끝 잡기: 끝을 잇는 현에서 가장 처진 깊이
            chord = ys[0] + (ys[-1] - ys[0]) * (xs - xs[0]) / (xs[-1] - xs[0])
            d["mid_sag_mm"] = round(float(np.max(chord - ys)), 1)
            d["span_mm"] = round(float(xs[-1] - xs[0]), 1)
        data[name] = d
        print(name, {k: v for k, v in d.items() if k != "pts"})
        if out_dir:
            ov = frame.copy()
            ov[keep] = (0.5 * ov[keep] + [0, 110, 0]).astype(np.uint8)
            ov[hand > 0] = (0.6 * ov[hand > 0] + [90, 0, 0]).astype(np.uint8)
            back = np.c_[xs, -ys] / CELL_MM
            for q in project(p, back).astype(int):
                cv2.circle(ov, tuple(int(v) for v in q), 4, (0, 0, 255), -1)
            cv2.imwrite(f"{out_dir}/{name}.jpg", ov)

    with open(OUT_JS, "w", encoding="utf-8", newline="\n") as f:
        f.write("// tools/spring_sag_measure.py가 만든다 — 손으로 고치지 말 것. 단위 mm, 원점 = 가장 높은 점.\n")
        f.write("window.SPRING_SAG = " + json.dumps(data, ensure_ascii=False, indent=1) + ";\n")
    print("->", OUT_JS)


if __name__ == "__main__":
    selftest() if "--selftest" in sys.argv else main()
