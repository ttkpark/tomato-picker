#!/usr/bin/env python3
"""손-눈 변환을 **최적화 없이 구성한다** — 관절별 축 측정에서 곧바로.

    python ros2/tools/handeye_from_axes.py ~/joint_axes.json --home "0,73.5,11,-84,180"

⚠ PC에서 돈다. `joint_axis_check.py`가 남긴 JSON만 있으면 된다.

────────────────────────────────────────────────────────────────────────
왜 이렇게 푸는가

여러 자세를 한꺼번에 넣고 전역으로 푸는 방식은 2026-09-01 내내 실패했다(잔차
70~100mm). 그런데 **관절을 하나씩만** 돌려 재 보니 전부 깨끗했다 — 잔차
1.0~1.5mm, 눈금 ≈ 1, 축의 배치도 모델과 2~5° 안에서 일치. 개별 측정은 성한데
합친 답만 틀린다면, 굳이 합쳐서 풀 이유가 없다. **재서 조립하면 된다.**

관절 j 하나만 돌리면 카메라는 그 축 둘레의 원을 그린다. 그 측정에서 두 가지가
나온다:

    a_j   축의 방향   (카메라 좌표계)
    d_j   축에서 카메라까지의 **수직거리**

같은 두 가지를 모델은 도구 좌표계에서 계산해 준다. 그러면

    회전   R_x a_j(카메라) = a_j(도구)          축 다섯 개로 Kabsch 한 번
    위치   dist(t_x, 축_j) = d_j                 거리 다섯 개로 최소자승

⚠ 축 방향의 **부호**는 이 측정으로 안 정해진다(원을 어느 쪽으로 도는지는
   각도의 부호에 흡수된다). 그래서 부호 조합을 다 훑어 가장 잘 맞는 것을 고른다.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))

from tomato_picker.hardware import kinematics as kin  # noqa: E402

ORDER = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def rot_z(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def expmap(w: np.ndarray) -> np.ndarray:
    """축각 벡터 → 회전행렬 (로드리게스)."""
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    a = w / th
    K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)


def refit(th, P, k, tries=20000, seed=0):
    """관절 하나의 (R_B, u, t_B)를 푼다 — 거칠게 훑은 뒤 **국소 정밀화**.

    ⚠ 무작위 회전만으로는 부족하다. 2026-09-01: 6만 개를 훑고도 같은 관절의
      축이 실행마다 17° 달라졌고 카메라까지의 거리가 121 → 199mm로 튀었다.
      잔차는 둘 다 ~1.9mm였다 — **잔차가 낮다고 답이 정해진 게 아니다.**
      SO(3)를 6만 점으로 덮으면 축당 해상도가 9°밖에 안 된다. 훑기는 골짜기를
      **찾는** 데까지만 쓰고, 바닥은 미분으로 내려가야 한다.
    """
    th = np.asarray(th, dtype=float)
    P = np.asarray(P, dtype=float)
    n = len(th)
    if P.ndim == 2:                       # 옛 형식(무게중심 하나)도 받는다
        P = P[:, None, :]
    K = P.shape[1]
    # 점마다 u_m 을 따로 두고 t_B 는 공유한다 — 미지수 3K+3.
    M = np.zeros((3 * n * K, 3 * K + 3))
    for i, t in enumerate(th):
        Rz = rot_z(-k * t)
        for m in range(K):
            r0 = 3 * (i * K + m)
            M[r0:r0 + 3, 3 * m:3 * m + 3] = Rz
            M[r0:r0 + 3, 3 * K:3 * K + 3] = -np.eye(3)
    Mp = np.linalg.pinv(M)

    def cost(R):
        y = np.einsum('nkj,ij->nki', P, R).reshape(-1)
        sol = Mp @ y
        r = (M @ sol - y).reshape(-1, 3)
        return float(np.sqrt((r ** 2).sum(axis=1).mean())), sol

    rng = np.random.default_rng(seed)
    q = rng.normal(size=(tries, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    best = None
    for a, b, c, d in q:
        R = np.array([
            [1 - 2 * (c * c + d * d), 2 * (b * c - d * a), 2 * (b * d + c * a)],
            [2 * (b * c + d * a), 1 - 2 * (b * b + d * d), 2 * (c * d - b * a)],
            [2 * (b * d - c * a), 2 * (c * d + b * a), 1 - 2 * (b * b + c * c)]])
        rms, _ = cost(R)
        if best is None or rms < best[0]:
            best = (rms, R)
    rms, R = best

    # 국소 정밀화 — 회전 3자유도만 좌표하강으로 좁혀 간다.
    step = math.radians(4.0)
    while step > 1e-5:
        moved = False
        for ax in range(3):
            for s in (+1.0, -1.0):
                w = np.zeros(3)
                w[ax] = s * step
                Rn = expmap(w) @ R
                rn, _ = cost(Rn)
                if rn < rms - 1e-9:
                    rms, R, moved = rn, Rn, True
        if not moved:
            step *= 0.5
    _, sol = cost(R)
    axis_cam = R[2, :] / np.linalg.norm(R[2, :])
    dist = float(math.hypot(sol[3 * K], sol[3 * K + 1]))
    return rms, axis_cam, dist


def model_axes(home: dict, geom: kin.ArmGeometry):
    """도구 좌표계에서 본 각 관절의 (축 방향, 축 위의 한 점).

    ⚠ 도구 좌표계는 `tool_frame_from_joints`와 같은 규약이다 —
       열이 [approach, lat, up], 원점이 TCP.
    """
    pan = math.radians(home["shoulder_pan"])
    a1 = math.radians(home["shoulder_lift"])
    a2 = a1 + math.radians(home["elbow_flex"])
    a3 = a2 + math.radians(home["wrist_flex"])
    roll = math.radians(home["wrist_roll"])
    cp, sp = math.cos(pan), math.sin(pan)

    def seg(a, ln):
        return np.array([ln * math.cos(a) * cp, ln * math.cos(a) * sp, ln * math.sin(a)])

    p1 = np.array([geom.d0 * cp, geom.d0 * sp, geom.z0])      # shoulder_lift 축 위
    p2 = p1 + seg(a1, geom.l1)                                # elbow 축 위
    p3 = p2 + seg(a2, geom.l2)                                # wrist_flex 축 위
    tcp = p3 + seg(a3, geom.l3)

    lateral = np.array([-sp, cp, 0.0])                        # 굽힘 세 축의 공통 방향
    approach = np.array([math.cos(a3) * cp, math.cos(a3) * sp, math.sin(a3)])
    up = np.array([-math.sin(a3) * cp, -math.sin(a3) * sp, math.cos(a3)])
    lat = lateral * math.cos(roll) + up * math.sin(roll)
    upr = -lateral * math.sin(roll) + up * math.cos(roll)
    R_tool = np.array([approach, lat, upr]).T                 # 열이 축

    pts = {"shoulder_pan": np.zeros(3), "shoulder_lift": p1, "elbow_flex": p2,
           "wrist_flex": p3, "wrist_roll": tcp}

    # ⚠ 축의 **부호를 손으로 정하지 않는다.** 2026-09-01: 부호 조합을 훑게 했더니
    #   180° 뒤집힌 답을 골랐다 — 이 팔은 축 방향이 실질적으로 둘뿐(굽힘 셋이
    #   평행 + 굴림)이라, 둘을 같이 뒤집으면 **똑같이 잘 맞는 다른 회전**이 있다.
    #   부호는 모델이 이미 알고 있다: 관절을 +Δ 돌렸을 때 도구 자세가 실제로
    #   어느 쪽으로 도는지 **미분해서** 꺼내면 된다. 추측할 자리가 아니다.
    def frame_of(d):
        pa = math.radians(d["shoulder_pan"])
        b1 = math.radians(d["shoulder_lift"])
        b2 = b1 + math.radians(d["elbow_flex"])
        b3 = b2 + math.radians(d["wrist_flex"])
        ro = math.radians(d["wrist_roll"])
        c2, s2 = math.cos(pa), math.sin(pa)
        ap = np.array([math.cos(b3) * c2, math.cos(b3) * s2, math.sin(b3)])
        la = np.array([-s2, c2, 0.0])
        u = np.array([-math.sin(b3) * c2, -math.sin(b3) * s2, math.cos(b3)])
        return np.array([ap, la * math.cos(ro) + u * math.sin(ro),
                         -la * math.sin(ro) + u * math.cos(ro)]).T

    R0 = frame_of(home)
    out = {}
    for j in pts:
        d = dict(home)
        d[j] = d[j] + 0.5
        dR = frame_of(d) @ R0.T                     # base 좌표계에서의 미소회전
        w = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
        n = np.linalg.norm(w)
        if n < 1e-9:                                # 도구 자세를 안 바꾸는 축은 없다
            continue
        axis_base = w / n                           # +Δ 가 만드는 회전의 축(오른손)
        out[j] = (R0.T @ axis_base, R0.T @ (pts[j] - tcp))
    return out, R_tool, tcp


def kabsch_rot(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """A(N,3)의 방향을 B(N,3)에 맞추는 회전 (평행이동 없음)."""
    U, _, Vt = np.linalg.svd(A.T @ B)
    d = float(np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0)
    return Vt.T @ np.diag([1.0, 1.0, d]) @ U.T


def dist_to_line(p, direction, point):
    v = p - point
    return float(np.linalg.norm(v - np.dot(v, direction) * direction))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default=os.path.expanduser("~/joint_axes.json"))
    ap.add_argument("--home", required=True,
                    help="측정할 때의 자세(도): pan,lift,elbow,wflex,wroll")
    ap.add_argument("--save", default="", help="이 경로에 T_tool_cam 을 쓴다")
    args = ap.parse_args()

    data = json.load(open(args.path, encoding="utf-8"))
    home = dict(zip(ORDER, [float(v) for v in args.home.split(",")]))
    geom = kin.ArmGeometry()
    lines, R_tool, tcp = model_axes(home, geom)

    print(f"자세 " + " ".join(f"{j.split('_')[0]}={home[j]:.1f}" for j in ORDER))
    print(f"TCP(base) ({tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f})")
    print()
    print("관절            축 잔차   축에서 카메라까지(측정)")
    meas_axis, meas_dist = {}, {}
    for j in ORDER:
        r = data["raw"].get(j)
        if r is None:
            print(f"  {j:<15}(측정 없음)")
            continue
        rms, ax, d = refit(r["th"], r["P"], 1.0)
        meas_axis[j], meas_dist[j] = ax, d
        print(f"  {j:<15}{rms:6.2f}mm        {d:7.1f}mm")

    if len(meas_axis) < 2:
        print("❌ 축이 둘 미만이다 — 풀 수 없다.")
        return 1

    # ── 회전: 축 방향의 부호 조합을 다 훑는다 ──────────────────────────
    names = [j for j in ORDER if j in meas_axis]
    A = np.array([meas_axis[j] for j in names])
    B = np.array([lines[j][0] for j in names])
    # 부호는 이제 양쪽 다 모델이 정한 것이라 훑지 않는다.
    Rx = kabsch_rot(A, B)
    err = [math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(Rx @ a, b))))))
           for a, b in zip(A, B)]
    mean_err = float(np.mean(err))
    print()
    print(f"[회전] R_x — 축 다섯 개를 한꺼번에 맞춘 결과 (평균 어긋남 {mean_err:.2f}°)")
    for j, e in zip(names, err):
        print(f"  {j:<15}{e:6.2f}°" + ("  ⚠" if e > 5.0 else ""))

    # ── 위치: 축까지의 거리 다섯 개로 t_x 를 푼다 ─────────────────────
    # dist(t, 축_j) = d_j — 비선형이라 가우스-뉴턴 몇 바퀴.
    t = np.zeros(3)
    for _ in range(200):
        J, r = [], []
        for j in names:
            d_dir, q = lines[j]
            v = t - q
            perp = v - np.dot(v, d_dir) * d_dir
            n = np.linalg.norm(perp)
            if n < 1e-6:
                continue
            J.append(perp / n - np.dot(perp / n, d_dir) * d_dir)
            r.append(n - meas_dist[j])
        if not J:
            break
        J = np.array(J)
        r = np.array(r)
        step, *_ = np.linalg.lstsq(J, -r, rcond=None)
        t = t + step
        if np.linalg.norm(step) < 1e-6:
            break
    res = [dist_to_line(t, *lines[j]) - meas_dist[j] for j in names]
    print()
    print(f"[위치] t_tool_cam = ({t[0]:7.1f}, {t[1]:7.1f}, {t[2]:7.1f}) mm "
          f"· |t| {np.linalg.norm(t):.0f}mm")
    for j, e in zip(names, res):
        print(f"  {j:<15}거리 오차 {e:+7.1f}mm" + ("  ⚠" if abs(e) > 8.0 else ""))
    rms_pos = float(np.sqrt(np.mean(np.square(res))))
    print(f"  거리 잔차 RMS {rms_pos:.1f}mm")

    print()
    if mean_err < 5.0 and rms_pos < 8.0:
        print("✅ 축과 거리가 모두 맞는다 — 이 손-눈 변환을 믿을 수 있다.")
    else:
        print("⚠ 아직 안 맞는다. 축이 어긋나면 회전 모델을, 거리가 어긋나면")
        print("   링크 길이(z0·d0·l1·l2·l3)를 의심하라 — 어느 관절인지 위에 있다.")

    if args.save:
        with open(os.path.expanduser(args.save), "w", encoding="utf-8") as fh:
            json.dump({"mode": "on_arm", "R": Rx.tolist(), "t": t.tolist(),
                       "axis_err_deg": mean_err, "dist_rms_mm": rms_pos,
                       "home_deg": home}, fh, ensure_ascii=False, indent=1)
        print(f"저장: {args.save}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
