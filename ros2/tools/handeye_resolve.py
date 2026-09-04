#!/usr/bin/env python3
"""저장해 둔 원자료로 손-눈 보정을 **다시 푼다** — 팔을 건드리지 않고.

    python ros2/tools/handeye_resolve.py ~/handeye_samples.json
    python ros2/tools/handeye_resolve.py samples.json --save ~/arm_eye.json

⚠ 팔도 카메라도 필요 없다. **PC에서 돈다**(이 저장소의 규칙: 숫자는 젯슨에
   올리기 전에 확인한다). `handeye_collect.py`가 남긴 JSON만 있으면 된다.

────────────────────────────────────────────────────────────────────────
왜 따로 있는가

가설 하나를 시험하려고 팔을 10분씩 움직이는 건 낭비다. 2026-08-31에 그렇게
네 번 돌렸고 매번 "잔차 42mm"라는 같은 답만 얻었다. 채집(팔이 필요)과
풀이(필요 없음)를 갈라 두면 풀이 쪽은 몇 초에 한 번씩 시험할 수 있다.

여기서 시험하는 것 셋:

  ① **roll 부호** — `wrist_roll`이 접근축 둘레로 어느 쪽으로 도는지는 부호가
     둘뿐이다. 양쪽 다 풀어 보고 잔차가 낮은 쪽이 맞는 쪽이다. 두 답이 비슷하면
     그건 roll이 충분히 안 변했다는 뜻이지 "둘 다 맞다"는 뜻이 아니다.
  ② **점 하나 vs 네 점** — 점 하나로 풀면 미지수 9개에 제약이 약하다. 네 점은
     **서로의 거리가 알려져 있어**(100 x 174.5mm 직사각형) 표적의 자세까지 묶인다.
  ③ **표본 하나씩 빼 보기**(`--drop`) — 한 표본이 잔차를 지배하면 잘못 찍힌 것이다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))

from tomato_picker.config import ARM_CART_ZERO_POSE_DEG as REF_DEG  # noqa: E402
from tomato_picker.hardware import kinematics as kin  # noqa: E402

DOTS = ("tl", "tr", "bl", "br")


def tool_frame(degs: dict, geom: kin.ArmGeometry, roll_sign: float):
    """관절각 → (R, t).

    ⚠ `handeye_collect.tool_frame_from_joints`와 **같은 식**이어야 한다.
      여기서 고른 부호는 그쪽에도 넣어야 두 도구가 같은 것을 뜻한다.
    """
    pan = math.radians(degs["shoulder_pan"])
    a3 = math.radians(degs["shoulder_lift"] + degs["elbow_flex"] + degs["wrist_flex"])
    approach = np.array([math.cos(a3) * math.cos(pan),
                         math.cos(a3) * math.sin(pan), math.sin(a3)])
    lateral = np.array([-math.sin(pan), math.cos(pan), 0.0])
    up = np.array([-math.sin(a3) * math.cos(pan),
                   -math.sin(a3) * math.sin(pan), math.cos(a3)])
    roll = math.radians(roll_sign * float(degs.get("wrist_roll", 0.0)))
    lat = lateral * math.cos(roll) + up * math.sin(roll)
    upr = -lateral * math.sin(roll) + up * math.cos(roll)
    p = kin.forward(degs, geom)
    return (np.array([approach, lat, upr]).T,
            np.array([p.x, p.y, p.z], dtype=float))


def marker_points(s: dict, which: str) -> list:
    """한 표본에서 마커로 쓸 카메라계 점들.

    ⚠ `tl`·`tr`… 라는 **이름은 화면 기준**이다(`order_dots`가 화면의 위/아래,
      왼/오른쪽으로 정한다). 손목이 구르면 같은 이름이 **다른 물리적 점**을
      가리킨다 — 2026-09-01 실측에서 roll이 118° 변했으므로 이름표가 섞였다.
      그래서 기본은 `mid`, 곧 **네 점의 무게중심**이다. 무게중심은 이름이
      어떻게 섞이든 같은 물리적 점이다(치환에 불변).
    """
    if which == "mid":
        return [list(np.mean([s["dots_mm"][k] for k in DOTS], axis=0))]
    return [s["dots_mm"][which]]


def plane_normal(s: dict) -> np.ndarray:
    """네 점이 이루는 평면의 법선(카메라계) — **이름표가 필요 없다.**

    이름이 섞여도 네 점이 만드는 평면은 그대로다. 그래서 회전 모델만 따로
    시험할 수 있다: 모든 자세에서 `R_tool(i) R_x n_cam(i)` 가 같은 벡터
    (벽의 법선)여야 한다. 안 맞으면 **회전이 틀린 것**이고, 잘 맞는데도 잔차가
    크면 틀린 곳은 회전이 아니라 평행이동·링크길이다.
    """
    P = np.array([s["dots_mm"][k] for k in DOTS])
    P = P - P.mean(axis=0)
    n = np.linalg.svd(P)[2][2]
    return n / np.linalg.norm(n) * (-1.0 if n[2] > 0 else 1.0)


def scaled(degs: dict, k: dict) -> dict:
    """관절 눈금을 다시 재는 것 — `deg' = ref + k * (deg - ref)`.

    ⚠ 왜 이런 게 필요한가. 관절각은 서보의 정규화값에서 나오고, 환산 계수는
      `span/200`, `span = |range_max - range_min| * 360/4096` 이다. 즉 **4096틱이
      정확히 360°라는 가정**이 들어 있다. 그 가정이 8% 틀리면 100° 움직인 관절이
      8° 어긋난다 — 380mm 거리에서 53mm다. 이 저장소의 1번 병과 같은 모양이다:
      "지령은 맞는데 실제가 다르다".

    ref는 교시 자세(영점을 잡은 자세)라 그 점에서는 눈금이 뭐든 값이 같다.
    """
    return {j: REF_DEG.get(j, 0.0) + k.get(j, 1.0) * (degs[j] - REF_DEG.get(j, 0.0))
            for j in degs}


def normal_misfit(samples, geom, roll_sign: float, k: dict) -> float:
    """이름표 없이 회전만 본다 — 벽 법선이 자세마다 얼마나 어긋나는가(도)."""
    Rt = [tool_frame(scaled(s["joints_deg"], k), geom, roll_sign)[0] for s in samples]
    nc = np.array([plane_normal(s) for s in samples])
    Rx, nb = np.eye(3), None
    for _ in range(60):
        nb = np.mean([Rt[i] @ Rx @ nc[i] for i in range(len(samples))], axis=0)
        nb = nb / np.linalg.norm(nb)
        Rx = kabsch(nc, np.array([Rt[i].T @ nb for i in range(len(samples))]))[0]
    ang = [math.degrees(math.acos(max(-1.0, min(1.0, float((Rt[i] @ Rx @ nc[i]) @ nb)))))
           for i in range(len(samples))]
    return float(np.mean(ang))


def kabsch(A: np.ndarray, B: np.ndarray):
    """A(N,3)를 B(N,3)에 맞추는 강체변환 (R, t): R @ a + t ≈ b."""
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    d = float(np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0)
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, cb - R @ ca


def solve(samples, geom, roll_sign: float, dots, iters: int = 80):
    """eye-in-hand: p_base = R_i (R_x p_i + t_x) + b_i.

    번갈아 푼다 — (R_x, t_x 고정 → 표적 위치) ↔ (표적 고정 → R_x, t_x).
    양쪽 다 닫힌 해(평균·Kabsch)라서 몇 바퀴면 멎는다.
    """
    frames = [tool_frame(s["joints_deg"], geom, roll_sign) for s in samples]
    obs = np.array([sum((marker_points(s, k) for k in dots), []) for s in samples])
    n, K = obs.shape[0], obs.shape[1]

    Rx, tx = np.eye(3), np.zeros(3)
    best = None
    for _ in range(iters):
        P = np.zeros((K, 3))
        for k in range(K):
            P[k] = np.mean([R @ (Rx @ obs[i, k] + tx) + t
                            for i, (R, t) in enumerate(frames)], axis=0)
        src = np.concatenate([obs[i] for i in range(n)], axis=0)
        dst = np.concatenate([np.array([frames[i][0].T @ (P[k] - frames[i][1])
                                        for k in range(K)])
                              for i in range(n)], axis=0)
        Rx, tx = kabsch(src, dst)
        res = np.array([np.linalg.norm(frames[i][0] @ (Rx @ obs[i, k] + tx)
                                       + frames[i][1] - P[k])
                        for i in range(n) for k in range(K)])
        rms = float(np.sqrt((res ** 2).mean()))
        if best is None or rms < best[0] - 1e-9:
            best = (rms, Rx.copy(), tx.copy(), P.copy(), res.copy())
        else:
            break
    return best


def solve_global(samples, geom, roll_sign: float, dots, tries: int = 40000, seed: int = 0):
    """**전역**으로 푼다 — 회전만 훑고, 나머지는 닫힌 해로.

    ⚠ 왜 필요한가. 교대 최소화(Kabsch ↔ 평균)는 단조롭게 줄지만 **극소에 갇힌다.**
      2026-09-01에 그것이 |t_x| = 640mm 라는 물리적으로 불가능한 답을 계속 냈고,
      나는 그걸 데이터의 성질로 오해할 뻔했다. 답이 이상할 때 데이터를 의심하기
      전에 **푸는 방법부터** 의심해야 한다.

    R_x를 고정하면 식이 선형이 된다:
        p_base - R_i t_x = R_i R_x p_i + b_i
    미지수 (p_base, t_x) 6개짜리 최소자승이다. 그래서 회전 3자유도만 훑으면 된다.
    """
    frames = [tool_frame(s["joints_deg"], geom, roll_sign) for s in samples]
    obs = np.array([sum((marker_points(s, k) for k in dots), []) for s in samples])
    n, K = obs.shape[0], obs.shape[1]

    # 설계행렬 M은 R_x에 안 딸린다 — 한 번만 만들고 유사역행렬을 재활용한다.
    # (이게 없으면 회전 4만 개를 훑는 데 몇 분이 걸린다.)
    M = np.zeros((3 * n * K, 3 * K + 3))
    r = 0
    for i, (R, _b) in enumerate(frames):
        for k in range(K):
            M[r:r + 3, 3 * k:3 * k + 3] = np.eye(3)
            M[r:r + 3, 3 * K:3 * K + 3] = -R
            r += 3
    Mp = np.linalg.pinv(M)
    Rs = np.array([f[0] for f in frames])
    bs = np.array([f[1] for f in frames])

    def residual_for(Rx):
        y = (np.einsum("nij,nkj->nki", Rs, obs @ Rx.T) + bs[:, None, :]).reshape(-1)
        sol = Mp @ y
        res = (M @ sol - y).reshape(-1, 3)
        return float(np.sqrt((res ** 2).sum(axis=1).mean())), sol

    rng = np.random.default_rng(seed)
    q = rng.normal(size=(tries, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    best = None
    for a, b_, c, d in q:
        Rx = np.array([
            [1 - 2 * (c * c + d * d), 2 * (b_ * c - d * a), 2 * (b_ * d + c * a)],
            [2 * (b_ * c + d * a), 1 - 2 * (b_ * b_ + d * d), 2 * (c * d - b_ * a)],
            [2 * (b_ * d - c * a), 2 * (c * d + b_ * a), 1 - 2 * (b_ * b_ + c * c)]])
        rms, sol = residual_for(Rx)
        if best is None or rms < best[0]:
            best = (rms, Rx, sol)
    rms, Rx, sol = best
    P = sol[:3 * K].reshape(K, 3)
    tx = sol[3 * K:]
    res = np.array([np.linalg.norm(frames[i][0] @ (Rx @ obs[i, k] + tx)
                                   + frames[i][1] - P[k])
                    for i in range(n) for k in range(K)])
    return (rms, Rx, tx, P, res)


def solve_fixed_t(samples, geom, roll_sign: float, dots, tx, tries: int = 20000, seed: int = 0):
    """`t_x`를 **자로 잰 값으로 고정**하고 회전(R_x) 3자유도만 훑는다.

    ⚠ 왜 필요한가 — 2026-09-04~05: 회전은 여러 번 깨끗하게(벽 법선 어긋남
      2.6~3.7°) 나오는데 `|t_x|`가 매번 실측(75~80mm)의 4~6배로 나오는 문제가
      재현됐다. 카메라 내부파라미터·링크 길이·`d0`·기하 전체 스케일을 다
      배제했는데도 안 풀렸다 — 미지수 6개(R_x 3 + t_x 3)를 전부 데이터에만
      맡기면 이렇게 다른 곳(회전)이 좋아 보여도 t_x 쪽으로 오차가 몰릴 수
      있다는 뜻이다. `l1`·`l2`·`l3`처럼 **자로 직접 잰 t_x**를 상수로 박으면
      미지수가 3개(R_x)로 줄어 훨씬 덜 degenerate하다.

    t_x는 도구 좌표(approach, lateral, up) — 즉 TCP에서 카메라 렌즈 중심까지의
    변위를 "집게가 찌르는 방향/왼쪽/위" 축으로 잰 값이다(l1·l2·l3와 같은
    축-축 벡터 관례).
    """
    frames = [tool_frame(s["joints_deg"], geom, roll_sign) for s in samples]
    obs = np.array([sum((marker_points(s, k) for k in dots), []) for s in samples])
    n, K = obs.shape[0], obs.shape[1]
    tx = np.asarray(tx, dtype=float)

    def residual_for(Rx):
        # pred[i,k] = 그 자세에서 R_x·(관측점 k) + t_x를 base 좌표로 옮긴 것.
        pred = np.einsum("nij,nkj->nki", np.array([f[0] for f in frames]),
                         obs @ Rx.T + tx) + np.array([f[1] for f in frames])[:, None, :]
        P = pred.mean(axis=0)                      # 마커 위치 — 표본 평균이 최소자승 해
        res = np.linalg.norm(pred - P[None, :, :], axis=2)
        return float(np.sqrt((res ** 2).mean())), P, res.reshape(-1)

    rng = np.random.default_rng(seed)
    q = rng.normal(size=(tries, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    best = None
    for a, b_, c, d in q:
        Rx = np.array([
            [1 - 2 * (c * c + d * d), 2 * (b_ * c - d * a), 2 * (b_ * d + c * a)],
            [2 * (b_ * c + d * a), 1 - 2 * (b_ * b_ + d * d), 2 * (c * d - b_ * a)],
            [2 * (b_ * d - c * a), 2 * (c * d + b_ * a), 1 - 2 * (b_ * b_ + c * c)]])
        rms, P, res = residual_for(Rx)
        if best is None or rms < best[0]:
            best = (rms, Rx, P, res)
    rms, Rx, P, res = best

    # ⚠ 전역 탐색(무작위 사원수)은 **기초를 찾는 용도**다 — 3자유도 회전
    #   공간을 2만 개 표본으로 훑으면 올바른 부호는 확실히 갈리지만(합성
    #   시험: 틀린 부호 38mm vs 맞는 부호 7mm), 정밀도는 잡음 수준(0.3mm)
    #   근처까지 못 간다. 그 자리에서 국소 흔들기로 다듬는다.
    rng2 = np.random.default_rng(seed + 1)
    step = 0.2
    for _ in range(400):
        pert = rng2.normal(scale=step, size=3)
        ang = np.linalg.norm(pert)
        if ang < 1e-9:
            continue
        axis = pert / ang
        Kx = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        dR = np.eye(3) + np.sin(ang) * Kx + (1 - np.cos(ang)) * (Kx @ Kx)
        r2, P2, res2 = residual_for(dR @ Rx)
        if r2 < rms:
            rms, Rx, P, res = r2, dR @ Rx, P2, res2
        else:
            step *= 0.985
    return (rms, Rx, tx, P, res)


def rpy(R):
    return (math.degrees(math.atan2(R[2, 1], R[2, 2])),
            math.degrees(-math.asin(max(-1.0, min(1.0, R[2, 0])))),
            math.degrees(math.atan2(R[1, 0], R[0, 0])))


def report(tag, got, expect):
    rms, Rx, tx, P, res = got
    print(f"\n[{tag}]  잔차 RMS {rms:6.2f}mm · 최대 {res.max():6.2f}mm")
    print(f"   t_tool_cam = ({tx[0]:7.1f},{tx[1]:7.1f},{tx[2]:7.1f}) mm "
          f"· |t| {np.linalg.norm(tx):5.0f}mm")
    print("   rpy = (%.1f, %.1f, %.1f)°" % rpy(Rx))
    if P.shape[0] == 4:
        idx = {k: i for i, k in enumerate(DOTS)}
        want = (("tl", "tr", expect["w"]), ("bl", "br", expect["w"]),
                ("tl", "bl", expect["h"]), ("tr", "br", expect["h"]))
        errs = [float(np.linalg.norm(P[idx[a]] - P[idx[b]])) - w for a, b, w in want]
        print("   표적 변 길이 오차 " + " ".join(f"{e:+.1f}" for e in errs) + " mm")
        print("   표적 네 점(팔 base) " + " ".join(
            f"{k}({P[idx[k]][0]:.0f},{P[idx[k]][1]:.0f},{P[idx[k]][2]:.0f})" for k in DOTS))
    else:
        print(f"   표적 위치(팔 base) = ({P[0][0]:.0f}, {P[0][1]:.0f}, {P[0][2]:.0f}) mm")
    return rms


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?",
                    default=os.path.expanduser("~/handeye_samples.json"))
    ap.add_argument("--save", default="", help="이 경로에 T_tool_cam 을 쓴다")
    ap.add_argument("--drop", type=int, default=-1, help="표본 하나를 빼고 푼다(0부터)")
    ap.add_argument("--fit-scale", action="store_true",
                    help="관절 눈금(도/정규화)을 데이터로 맞춰 본다")
    ap.add_argument("--fit-geom", action="store_true",
                    help="l1·l2 를 데이터로 맞춰 본다 (나머지는 갈리지 않는다)")
    ap.add_argument("--tries", type=int, default=20000,
                    help="전역 탐색에서 훑을 회전 개수")
    ap.add_argument("--loo", action="store_true",
                    help="표본을 하나씩 빼 보며 어느 것이 잔차를 지배하는지 본다")
    ap.add_argument("--expect-t-mm", type=float, nargs=2, default=None,
                    metavar=("MIN", "MAX"),
                    help="자로 따로 잰 카메라~손끝 거리(mm) 범위 — 이 안에 드는 "
                         "해만 고른다. 부호(roll)가 애매하면 **잔차가 더 낮은 쪽이 "
                         "틀린 부호일 수 있다**(2026-09-04 실측: 틀린 부호가 잔차는 "
                         "1~2mm 더 낮은데 |t|=250~310mm라는, 팔 길이보다 긴 자리에 "
                         "카메라가 있다는 답을 냈다). 이 범위를 주면 그런 답을 "
                         "자동으로 버린다.")
    ap.add_argument("--fix-t", type=float, nargs=3, default=None,
                    metavar=("APPROACH", "LATERAL", "UP"),
                    help="T_tool_cam(mm)을 자로 잰 값으로 **고정**하고 회전(R_x) "
                         "3자유도만 훑는다. l1·l2·l3와 같은 축-축 벡터 관례 — "
                         "TCP에서 카메라 렌즈 중심까지, (집게가 찌르는 방향, "
                         "그 왼쪽, 위) 순서로 mm. 2026-09-05: 회전은 여러 번 "
                         "깨끗하게 나오는데 |t_x|가 매번 실측의 4~6배로 나와서 "
                         "6자유도를 전부 데이터에 맡기는 걸 포기하고 만든 경로다.")
    args = ap.parse_args()

    data = json.load(open(args.path, encoding="utf-8"))
    samples = data["samples"]
    if 0 <= args.drop < len(samples):
        print(f"(표본 {args.drop}번 [{samples[args.drop]['label']}] 을 뺀다)")
        samples = [s for i, s in enumerate(samples) if i != args.drop]
    g = data["geom"]
    geom = kin.ArmGeometry(z0=g["z0"], d0=g["d0"], l1=g["l1"], l2=g["l2"], l3=g["l3"])
    expect = data.get("expect_mm", {"w": 100.0, "h": 174.5})
    marker = data.get("marker", "tl")

    print(f"표본 {len(samples)}개 · 링크 z0={g['z0']} d0={g['d0']} "
          f"l1={g['l1']} l2={g['l2']} l3={g['l3']}")
    rolls = sorted({round(float(s["joints_deg"]["wrist_roll"]), 1) for s in samples})
    print(f"wrist_roll 값 {rolls}")

    # ── 회전만 따로 시험한다 (이름표가 필요 없는 검사) ────────────────────
    # n_base = R_tool(i) R_x n_cam(i) 가 모든 자세에서 같아야 한다. 여기가
    # 어긋나면 회전 모델(관절→도구 자세)이 틀린 것이고, 여기가 맞는데 잔차가
    # 크면 틀린 곳은 회전이 아니다 — 평행이동이나 링크 길이다.
    print("\n[회전만] 벽 법선이 자세마다 같은 곳을 가리키는가")
    rot_misfit = {}
    for sign in (+1.0, -1.0):
        Rt = [tool_frame(s["joints_deg"], geom, sign)[0] for s in samples]
        nc = np.array([plane_normal(s) for s in samples])
        Rx = np.eye(3)
        for _ in range(200):
            nb = np.mean([Rt[i] @ Rx @ nc[i] for i in range(len(samples))], axis=0)
            nb /= np.linalg.norm(nb)
            Rx = kabsch(nc, np.array([Rt[i].T @ nb for i in range(len(samples))]))[0]
        ang = [math.degrees(math.acos(max(-1.0, min(1.0, float((Rt[i] @ Rx @ nc[i]) @ nb)))))
               for i in range(len(samples))]
        rot_misfit[sign] = float(np.mean(ang))
        print(f"  roll부호 {sign:+.0f}: 어긋남 평균 {np.mean(ang):5.2f}° "
              f"· 최대 {max(ang):5.2f}°  (벽 법선 {nb.round(3)})")
    # ⚠ 아래 눈금·링크 맞추기가 **같은 부호를 계속 써야 한다.** 예전에는 링크
    #   맞추기가 부호를 1.0으로 못박고 있었다 — 이 로봇 데이터로는 그게 틀린
    #   부호였다(93.8° vs 11.4°, 2026-09-04). 틀린 부호로 링크 길이를 맞추면
    #   그 오차를 링크 길이가 흡수해 버려 엉뚱한 l1·l2가 "맞는 값"으로 나온다.
    sign0 = +1.0 if rot_misfit[+1.0] <= rot_misfit[-1.0] else -1.0

    # ── 관절 눈금을 데이터로 맞춰 본다 (이름표 없는 회전 검사만 쓴다) ──
    # 회전이 5° 어긋난다면 원인 후보는 둘이다: (가) 눈금(도/정규화)이 틀렸다,
    # (나) 기계가 휜다. (가)라면 관절마다 하나의 배율로 크게 줄어든다.
    if args.fit_scale:
        sign = sign0
        k = {j: 1.0 for j in kin.JOINTS}
        base = normal_misfit(samples, geom, sign, k)
        print("")
        print(f"[눈금 맞추기] 배율 1.000 일 때 회전 어긋남 {base:.2f}°")
        for _ in range(6):
            for j in kin.JOINTS:
                best = (normal_misfit(samples, geom, sign, k), k[j])
                # ⚠ 0까지 연다. `wrist_roll` 배율이 0으로 가면 그건 눈금이 아니라
                #   **카메라가 그 관절보다 앞(=같이 안 도는 링크)에 달려 있다**는 뜻이다.
                #   범위를 좁게 잡으면 하한에 붙은 값을 "8% 눈금 오차"로 오해한다.
                for v in np.arange(0.0, 1.501, 0.005):
                    trial = dict(k)
                    trial[j] = float(v)
                    m = normal_misfit(samples, geom, sign, trial)
                    if m < best[0] - 1e-6:
                        best = (m, float(v))
                k[j] = best[1]
        fin = normal_misfit(samples, geom, sign, k)
        print("   " + " ".join(f"{j.split('_')[0]}x{k[j]:.3f}" for j in kin.JOINTS))
        print(f"   → 회전 어긋남 {base:.2f}° → {fin:.2f}°")
        if fin > base * 0.5:
            print("   ⚠ 배율로 반도 못 줄었다 — 눈금이 아니라 **기계가 휜다**는 뜻이다")
        else:
            print("   ✅ 눈금이 원인이다. 이 배율을 환산식에 반영해야 한다.")
        samples = [dict(s, joints_deg=scaled(s["joints_deg"], k)) for s in samples]
        print("   (아래 풀이는 맞춘 눈금을 적용한 것이다)")

    # ── 링크 길이를 데이터로 맞춰 본다 ─────────────────────────────────
    # ⚠ 다섯 개를 다 풀면 안 된다. **갈리지 않는 것이 섞여 있다:**
    #   · l3 는 도구 원점을 접근축 방향으로 민다 — t_x 의 x성분과 **완전히 같은 짓**
    #     이다. 둘을 같이 풀면 무한히 많은 답이 나온다.
    #   · z0 는 모든 자세를 base z로 같은 만큼 민다 — 표적 위치(자유 미지수)가
    #     그대로 흡수한다.
    #   · d0 는 pan이 변해야 갈린다. 이 표본은 pan이 거의 안 변했다.
    #   그래서 **l1, l2 만** 푼다. 이 둘은 lift·elbow가 변할 때 도구 원점을
    #   서로 다르게 움직이므로 실제로 갈린다.
    if args.fit_geom:
        dots0 = DOTS if len(samples) >= 8 else ("mid",)
        base_rms = solve(samples, geom, sign0, ("mid",))[0]
        print("")
        print(f"[링크 맞추기] 실측 l1={geom.l1:.1f} l2={geom.l2:.1f} 에서 잔차 {base_rms:.1f}mm")
        # ⚠ **물리적 한계로 묶는다.** 안 묶었더니 l1=-86mm 같은 음수 길이로
        #   달아나며 잔차만 낮췄다(2026-09-01). 자로 잰 값이 20mm씩 틀릴 리는
        #   없으니, 그 밖으로 나가는 답은 맞춘 게 아니라 **모델을 망가뜨린 것**이다.
        L1, L2 = geom.l1, geom.l2
        best = (base_rms, L1, L2)
        for _ in range(4):
            for idx in (0, 1):
                for delta in np.arange(-20.0, 20.1, 0.5):
                    l1 = (L1 + delta) if idx == 0 else best[1]
                    l2 = (L2 + delta) if idx == 1 else best[2]
                    g2 = kin.ArmGeometry(z0=geom.z0, d0=geom.d0, l1=l1, l2=l2, l3=geom.l3)
                    r = solve(samples, g2, sign0, ("mid",))[0]
                    if r < best[0] - 1e-6:
                        best = (r, l1, l2)
        rms2, l1, l2 = best
        print(f"   → l1={l1:.1f} ({l1 - geom.l1:+.1f})  l2={l2:.1f} ({l2 - geom.l2:+.1f})"
              f"   잔차 {base_rms:.1f} → {rms2:.1f}mm")
        if rms2 < base_rms * 0.6:
            print("   ✅ 링크 길이가 원인이었다. 이 값으로 이어서 푼다.")
            geom = kin.ArmGeometry(z0=geom.z0, d0=geom.d0, l1=l1, l2=l2, l3=geom.l3)
        else:
            print("   ⚠ 링크로는 반도 못 줄었다 — 남은 오차는 다른 데 있다.")
        print("")

    results, tmags, gots = {}, {}, {}
    if args.fix_t:
        # ── t_x 고정 경로 — 회전 3자유도만 훑는다 ─────────────────────────
        # "네 점"은 안 쓴다 — 지금까지 늘 "무게중심"보다 훨씬 나빴고(마커 넷을
        # 독립으로 풀 만큼 표본이 넉넉한 적이 없었다), t_x가 고정이면 그 격차가
        # 줄어들 이유가 없다.
        print(f"\n[t_x 고정] {tuple(args.fix_t)} mm 로 고정하고 회전만 훑는다")
        name = "무게중심(t고정)"
        for sign in (+1.0, -1.0):
            got = solve_fixed_t(samples, geom, sign, ("mid",), args.fix_t, tries=args.tries)
            results[(sign, name)] = report(f"roll부호 {sign:+.0f} · {name}", got, expect)
            tmags[(sign, name)] = float(np.linalg.norm(got[2]))
            gots[(sign, name)] = got
        (sign, name), rms = min(results.items(), key=lambda kv: kv[1])
    else:
        for sign in (+1.0, -1.0):
            for name, dots in (("무게중심", ("mid",)), ("네 점", DOTS)):
                a = solve(samples, geom, sign, dots)
                b = solve_global(samples, geom, sign, dots, tries=args.tries)
                got = b if b[0] < a[0] else a
                tag = "전역" if got is b else "교대"
                results[(sign, name)] = report(
                    f"roll부호 {sign:+.0f} · {name} · {tag}"
                    f" (교대 {a[0]:.1f} / 전역 {b[0]:.1f}mm)", got, expect)
                tmags[(sign, name)] = float(np.linalg.norm(got[2]))
                gots[(sign, name)] = got

        naive_key, naive_rms = min(results.items(), key=lambda kv: kv[1])
        if args.expect_t_mm:
            lo, hi = args.expect_t_mm
            plausible = {k: v for k, v in results.items() if lo <= tmags[k] <= hi}
            if not plausible:
                print(f"\n⚠ 넷 다 |t_tool_cam|이 기대 범위({lo:.0f}~{hi:.0f}mm) 밖이다 — "
                      "잔차만으로는 답을 못 고른다. 표본을 더 넓게 흩어 다시 채집하라.")
                (sign, name), rms = naive_key, naive_rms
            else:
                (sign, name), rms = min(plausible.items(), key=lambda kv: kv[1])
                if (sign, name) != naive_key:
                    print(f"\n⚠ 잔차만 보면 roll{naive_key[0]:+.0f}·{naive_key[1]}이 이긴다"
                          f"(잔차 {naive_rms:.2f}mm) — 그런데 |t_tool_cam| "
                          f"{tmags[naive_key]:.0f}mm로 기대 범위({lo:.0f}~{hi:.0f}mm) 밖이다. "
                          f"범위 안에서 가장 낮은 roll{sign:+.0f}·{name}"
                          f"(|t| {tmags[(sign, name)]:.0f}mm, 잔차 {rms:.2f}mm)를 대신 쓴다 — "
                          "잔차가 낮다고 그 부호가 맞는 것은 아니다.")
        else:
            (sign, name), rms = naive_key, naive_rms

    print(f"\n▶ 가장 잘 맞는 조합: roll부호 {sign:+.0f} · {name} (잔차 {rms:.2f}mm, "
          f"|t_tool_cam| {tmags[(sign, name)]:.0f}mm)")
    other = results[(-sign, name)]
    print(f"   반대 부호는 {other:.2f}mm(|t| {tmags[(-sign, name)]:.0f}mm) — "
          + ("부호가 뚜렷이 갈린다" if other > rms * 1.3
             else "⚠ 잔차만 보면 둘이 비슷하다 — roll이 충분히 안 변했거나, "
                  "--expect-t-mm 없이는 부호를 못 가른다."))

    # ── 잔차가 **어느 축과 함께 커지는가** ──────────────────────────────
    # 이 저장소의 1번 병("증상이 원인을 안 가리킨다")을 여기서도 피하려는 것이다.
    # 42mm라는 숫자만 보면 무엇을 고칠지 알 수 없다. 그런데 잔차가 특정 관절과
    # 같이 커지면 **그 관절의 모델이 틀렸다**고 말할 수 있다 — 예컨대 roll과
    # 상관이 크면 접근축 둘레 회전(부호나 축 위치)이, pitch와 크면 a3가 범인이다.
    dots = DOTS if name == "네 점" else ("mid",)
    _, _, _, _, res = gots[(sign, name)]
    per = res.reshape(len(samples), -1).mean(axis=1)
    print("\n[상관] 잔차가 어느 축과 함께 커지는가 (|r|>0.6이면 그 축의 모델을 의심)")
    for j in kin.JOINTS:
        v = np.array([float(s["joints_deg"][j]) for s in samples])
        if np.ptp(v) < 1.0:
            print(f"  {j:<15} 변화 없음 — 이 표본으로는 못 가른다")
            continue
        c = float(np.corrcoef(v, per)[0, 1])
        mark = "  ⚠" if abs(c) > 0.6 else ""
        print(f"  {j:<15} 변화 폭 {np.ptp(v):5.1f}° · 상관 {c:+.2f}{mark}")
    print("  표본별 잔차 " + " ".join(f"{p:.0f}" for p in per) + " mm")

    if args.loo:
        print("\n[하나씩 빼 보기] 뺐을 때 잔차가 크게 떨어지면 그 표본이 문제다")
        dots = DOTS if name == "네 점" else ("mid",)
        for i in range(len(samples)):
            sub = [s for k, s in enumerate(samples) if k != i]
            r = solve(sub, geom, sign, dots)[0]
            flag = "  ← 이걸 빼면 크게 좋아진다" if r < rms * 0.75 else ""
            print(f"  {i:2} {samples[i]['label']:<36} {r:6.2f}mm{flag}")

    if rms > 15.0:
        print("\n⚠ 15mm를 넘는다 — 집게가 헛집는다. 저장하면 안 된다.")
    if args.save:
        # ⚠ **`EyeConfig`가 읽는 형식 그대로 써야 한다.** 예전 버전은 여기서
        #   {"mode","R","t",...}를 직접 json.dump했는데, `Eye.cam_to_base`가
        #   찾는 건 `transform:{"R","t"}` 감싼 모양이다 — 이 파일의 예전
        #   docstring 예시(`--save ~/arm_eye.json`)를 그대로 따라 하면 파일은
        #   생기지만 `has_calibration`이 계속 False라 "보정이 없다"는 채로
        #   조용히 남는다. `EyeConfig.store()`를 그대로 불러써서 그 함정을
        #   원천봉쇄한다 — Astra 몫(`cameras.*`)을 지우지 않는 병합도 덤으로 얻는다.
        _, Rx, tx, _, res = gots[(sign, name)]
        sys.path.insert(0, os.path.join(REPO, "src"))
        from tomato_picker.hardware.handeye import Fit, Rigid  # noqa: E402
        from tomato_picker.hardware.eye import EyeConfig  # noqa: E402

        fit = Fit(transform=Rigid(Rx, tx), rms_mm=rms, max_mm=float(res.max()),
                  per_sample_mm=[float(v) for v in res], samples=len(samples),
                  scale_hint=1.0)
        note = (f"handeye_resolve.py · roll부호{sign:+.0f} · {name} · "
                f"표본 {len(samples)}개" + (" · 눈금맞춤 적용" if args.fit_scale else "")
                + (" · l1/l2 재추정" if args.fit_geom else ""))
        cfg = EyeConfig(path=args.save)
        cfg.set_mount("on_arm")
        cfg.store("on_arm", fit, None, note=note)
        print(f"저장: {cfg.path} ({'good' if fit.good else '⚠ 기준(15mm) 초과'})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
