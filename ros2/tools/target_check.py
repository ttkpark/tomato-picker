#!/usr/bin/env python3
"""**알고 있는 자로 깊이 사슬을 잰다** — 벽에 붙인 4점 표적으로.

    ~/.venvs/vision/bin/python ros2/tools/target_check.py        (젯슨에서)

ROS도 컨테이너도 필요 없고 **서비스를 내릴 필요도 없다** — 이미 도는
`depth-cam.service`가 /dev/shm에 써 둔 프레임을 그대로 읽는다.

────────────────────────────────────────────────────────────────────────
무엇을 재나

손-눈 보정은 "카메라 좌표 → 팔 좌표"를 푼다. 그 **앞 단계**, 즉
"픽셀 + 깊이 → 카메라 좌표"가 맞는지는 아무도 안 재 봤다. 거기가 틀리면
보정은 그 오차를 그대로 흡수해서 잔차는 작게 나오고, 팔은 헛집는다.

종이 위 네 점의 **실제 간격을 우리가 알고 있으므로**(가로 100mm, 세로 174.5mm)
그걸로 잰다. 나오는 값이 100mm가 아니면 내부파라미터나 깊이 단위가 틀린 것이다.

  · 변 길이  → 스케일이 맞는가 (fx·fy·depth_scale 전부가 여기 걸린다)
  · 대각선   → 가로세로가 따로 틀리지 않았는가
  · 평면성   → 네 점이 한 평면에 있는가 (깊이 잡음의 크기)
  · 법선     → 카메라가 벽을 얼마나 정면으로 보고 있는가

⚠ **검은 점 위의 깊이를 그대로 쓰지 않는다.** 검정은 IR을 흡수해 구멍이 나기
   쉽다. 대신 **흰 종이의 깊이로 평면을 맞추고**, 그 평면과 점의 시선이 만나는
   곳을 점의 3D 좌표로 삼는다. 종이는 평평하므로 이게 물리적으로 옳고,
   검은 점의 깊이 결손에도 흔들리지 않는다.
"""

from __future__ import annotations

import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "src"))
from tomato_picker.hardware.handeye import Intrinsics  # noqa: E402

COLOR = "/dev/shm/d405_color.jpg"
DEPTH = "/dev/shm/d405_depth.npy"
META = "/dev/shm/d405_meta.json"

# 벽에 붙인 표적의 **실측 간격**
EXPECT_W = 100.0
EXPECT_H = 174.5

FAILED: list[str] = []
PASSED = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global PASSED
    if ok:
        PASSED += 1
        print(f"  ok   {name}" + (f"  ({detail})" if detail else ""))
    else:
        FAILED.append(name)
        print(f"  FAIL {name}  {detail}")


def find_dots(bgr: np.ndarray) -> list[tuple[float, float]]:
    """어두운 원형 덩이 중 **주위가 밝은 것**만 고른다.

    주위 밝기를 보는 이유 — 화면에는 종이 밖에도 어두운 것들이 있다(가구·그림자).
    종이 위의 점만 테두리가 흰색이다. 이 한 줄이 오검출을 거의 다 없앤다.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # 프레임 상대 임계 — 절대값은 조명이 바뀌면 무너진다(이 저장소의 규칙).
    thresh = float(np.percentile(blur, 60)) * 0.55
    dark = (blur < thresh).astype(np.uint8) * 255

    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for c in contours:
        area = cv2.contourArea(c)
        if not 40.0 < area < 5000.0:
            continue
        perim = cv2.arcLength(c, True)
        if perim <= 0:
            continue
        circularity = 4.0 * np.pi * area / (perim * perim)
        if circularity < 0.6:          # 원이 아닌 것(가구 모서리 등) 제외
            continue
        m = cv2.moments(c)
        u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
        r = np.sqrt(area / np.pi)
        # 점 둘레의 고리가 **그 점보다 훨씬 밝아야** 한다 = 흰 종이 위의 검은 점.
        #
        # ⚠ 처음엔 "고리 밝기 > 화면 75%분위"로 걸렀는데 네 점이 **전부** 버려졌다.
        #    화면 대부분이 벽과 종이라 75%분위가 136까지 올라가는데, 점 둘레
        #    종이는 129~136이었다(실측). 간발의 차로 떨어진 것이다.
        #    절대 밝기는 조명이 조금만 달라져도 이렇게 뒤집힌다 —
        #    **대비**로 보면 안 뒤집힌다(이 저장소가 라인 검출에서 배운 것과 같다).
        ring = np.zeros(gray.shape, np.uint8)
        cv2.circle(ring, (int(u), int(v)), max(3, int(r * 2.6)), 255, -1)
        cv2.circle(ring, (int(u), int(v)), max(1, int(r * 1.5)), 0, -1)
        if not (ring > 0).any():
            continue
        core = np.zeros(gray.shape, np.uint8)
        cv2.circle(core, (int(u), int(v)), max(1, int(r * 0.7)), 255, -1)
        contrast = float(gray[ring > 0].mean()) - float(gray[core > 0].mean())
        if contrast < 30.0:
            continue
        found.append((u, v, area))

    found.sort(key=lambda t: -t[2])
    return [(u, v) for u, v, _ in found[:4]]


def order_dots(dots):
    """좌상·우상·좌하·우하 순서로."""
    dots = sorted(dots, key=lambda p: p[1])          # y 기준 위/아래
    top = sorted(dots[:2], key=lambda p: p[0])
    bottom = sorted(dots[2:], key=lambda p: p[0])
    return top[0], top[1], bottom[0], bottom[1]


def fit_plane(points: np.ndarray):
    """(N,3) → (법선 n, 거리 d) 로 n·X = d. 최소자승(SVD)."""
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid)
    normal = vt[2]
    if normal[2] < 0:            # 카메라 쪽을 보게 부호를 맞춘다
        normal = -normal
    return normal, float(normal @ centroid)


def measure() -> dict:
    """검사 없이 숫자만 — 팔을 움직이며 반복 호출하는 쪽(handeye_run)이 쓴다."""
    meta = json.load(open(META))
    intr = Intrinsics.from_dict(meta["intrinsics"])
    bgr = cv2.imread(COLOR)
    depth_mm = np.load(DEPTH).astype(float) * meta["depth_scale_mm"]
    dots = find_dots(bgr)
    if len(dots) != 4:
        return {"ok": False, "why": f"점이 {len(dots)}개 보인다(4개여야 한다)",
                "age": None}
    tl, tr, bl, br = order_dots(dots)
    us = [p[0] for p in (tl, tr, bl, br)]
    vs = [p[1] for p in (tl, tr, bl, br)]
    u0, u1 = int(min(us)) - 10, int(max(us)) + 10
    v0, v1 = int(min(vs)) - 10, int(max(vs)) + 10
    patch = depth_mm[max(0, v0):v1, max(0, u0):u1]
    ys, xs = np.mgrid[max(0, v0):v1, max(0, u0):u1]
    valid = patch > 0
    xs_v, ys_v, zs_v = xs[valid], ys[valid], patch[valid]
    if len(xs_v) < 300:
        return {"ok": False, "why": f"종이 깊이 화소가 {len(xs_v)}개뿐", "age": None}
    cap = 3000
    if len(xs_v) > cap:
        step = len(xs_v) // cap + 1
        xs_v, ys_v, zs_v = xs_v[::step], ys_v[::step], zs_v[::step]
    pts = np.array([intr.deproject(float(x), float(y), float(z))
                    for x, y, z in zip(xs_v, ys_v, zs_v)])
    normal, d = fit_plane(pts)

    def on_plane(u, v):
        ray = np.array(intr.deproject(u, v, 1.0))
        return ray * (d / float(normal @ ray))

    P = {k: on_plane(*p).tolist()
         for k, p in (("tl", tl), ("tr", tr), ("bl", bl), ("br", br))}
    edge = float(np.linalg.norm(np.array(P["tl"]) - np.array(P["tr"])))
    return {"ok": True, "dots": {"tl": tl, "tr": tr, "bl": bl, "br": br},
            "points_mm": P, "plane_mm": d,
            "tilt_deg": float(np.degrees(np.arccos(min(1.0, abs(normal[2]))))),
            "edge_top_mm": edge, "age": time.time() - meta["ts"]}


def main() -> int:
    if "--json" in sys.argv:
        print(json.dumps(measure(), ensure_ascii=False))
        return 0
    for path in (COLOR, DEPTH, META):
        if not os.path.exists(path):
            print(f"❌ {path} 가 없다 — depth-cam.service 가 도는지 확인하라.")
            return 1

    meta = json.load(open(META))
    intr = Intrinsics.from_dict(meta["intrinsics"])
    bgr = cv2.imread(COLOR)
    depth_mm = np.load(DEPTH).astype(float) * meta["depth_scale_mm"]

    print(f"카메라 {intr.width}x{intr.height} fx={intr.fx:.1f} fy={intr.fy:.1f} "
          f"ppx={intr.ppx:.1f} ppy={intr.ppy:.1f} · 왜곡모델 {intr.model}")
    print(f"깊이 단위 {meta['depth_scale_mm']:.4f}mm/단위 · 유효 {meta['valid_frac']:.1%}\n")

    print("[검출] 종이 위의 검은 점")
    dots = find_dots(bgr)
    check("점 4개", len(dots) == 4, f"{len(dots)}개 발견")
    if len(dots) != 4:
        print("       종이 전체가 화면에 들어오는지, 점이 가려지지 않았는지 보라.")
        return 1
    tl, tr, bl, br = order_dots(dots)
    for name, (u, v) in (("좌상", tl), ("우상", tr), ("좌하", bl), ("우하", br)):
        print(f"       {name} 픽셀 ({u:6.1f}, {v:6.1f})")

    # ── 흰 종이로 평면을 맞춘다 (검은 점의 깊이는 안 믿는다) ──
    us = [p[0] for p in (tl, tr, bl, br)]
    vs = [p[1] for p in (tl, tr, bl, br)]
    u0, u1 = int(min(us)) - 10, int(max(us)) + 10
    v0, v1 = int(min(vs)) - 10, int(max(vs)) + 10
    patch = depth_mm[max(0, v0):v1, max(0, u0):u1]
    ys, xs = np.mgrid[max(0, v0):v1, max(0, u0):u1]
    valid = patch > 0
    xs_v, ys_v, zs_v = xs[valid], ys[valid], patch[valid]

    # ⚠ 화소마다 파이썬 루프를 돌리면 안 된다 — 종이 영역만도 3만 화소라
    #    젯슨에서 **OOM으로 죽는다**(실측 rc=137, 스왑이 이미 꽉 차 있었다).
    #    평면 하나 맞추는 데 3만 점이 필요하지도 않다. 고르게 추려 쓴다.
    #    (왜곡 모델을 직접 벡터화하지 않는 이유: Intrinsics.deproject와 미세하게
    #     달라지면 그 차이가 그대로 "스케일이 조금 틀린" 것으로 나타난다.
    #     같은 함수를 쓰되 적게 부르는 쪽이 안전하다.)
    cap = 4000
    if len(xs_v) > cap:
        step = len(xs_v) // cap + 1
        xs_v, ys_v, zs_v = xs_v[::step], ys_v[::step], zs_v[::step]
    pts = np.array([intr.deproject(float(x), float(y), float(z))
                    for x, y, z in zip(xs_v, ys_v, zs_v)])

    print(f"\n[평면] 종이 영역 유효 화소 {len(pts)}개")
    check("평면을 맞출 만큼 깊이가 있다", len(pts) > 300, f"{len(pts)}개 (추려 씀)")
    normal, d = fit_plane(pts)
    resid = np.abs(pts @ normal - d)
    check("종이가 평평하다", float(np.percentile(resid, 95)) < 8.0,
          f"평면까지 거리 중앙값 {np.median(resid):.2f}mm · 95%tile "
          f"{np.percentile(resid, 95):.2f}mm")

    tilt = np.degrees(np.arccos(min(1.0, abs(normal[2]))))
    print(f"       벽 법선 {np.round(normal, 4).tolist()} · 광축과 {tilt:.1f}°")
    print(f"       카메라 → 벽 수직거리 {d / max(1e-9, abs(normal[2])) * abs(normal[2]):.0f}mm "
          f"(평면까지 {d:.0f}mm)")

    def on_plane(u: float, v: float) -> np.ndarray:
        """그 픽셀의 시선과 평면이 만나는 점 — 검은 점의 깊이 결손에 안 흔들린다."""
        ray = np.array(intr.deproject(u, v, 1.0))
        return ray * (d / float(normal @ ray))

    P = {k: on_plane(*p) for k, p in (("tl", tl), ("tr", tr), ("bl", bl), ("br", br))}

    print("\n[스케일] 아는 길이로 잰다")
    edges = (("위 가로", "tl", "tr", EXPECT_W), ("아래 가로", "bl", "br", EXPECT_W),
             ("왼 세로", "tl", "bl", EXPECT_H), ("오른 세로", "tr", "br", EXPECT_H))
    errs = []
    for label, a, b, want in edges:
        got = float(np.linalg.norm(P[a] - P[b]))
        err = got - want
        errs.append(err)
        check(f"{label} {want:.1f}mm", abs(err) < 5.0,
              f"실측 {got:6.1f}mm · 오차 {err:+.1f}mm ({err / want:+.1%})")

    diag = np.hypot(EXPECT_W, EXPECT_H)
    for label, a, b in (("대각 ↘", "tl", "br"), ("대각 ↗", "tr", "bl")):
        got = float(np.linalg.norm(P[a] - P[b]))
        check(f"{label} {diag:.1f}mm", abs(got - diag) < 6.0,
              f"실측 {got:6.1f}mm · 오차 {got - diag:+.1f}mm")

    bias = float(np.mean(errs))
    print(f"\n       평균 편향 {bias:+.2f}mm  →  스케일 오차 {bias / EXPECT_W:+.2%}")
    if abs(bias / EXPECT_W) > 0.02:
        print("       ⚠ 2%를 넘는다 — 내부파라미터(fx·fy)나 깊이 단위를 의심하라. "
              "보정을 아무리 잘해도 이 오차는 안 없어진다.")

    print()
    if FAILED:
        print(f"❌ {len(FAILED)}개 실패 / {PASSED + len(FAILED)}개 중")
        for name in FAILED:
            print(f"   - {name}")
        return 1
    print(f"✅ 전부 통과 ({PASSED}개)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
