#!/usr/bin/env python3
"""**역보정 풀이** — kincal_collect가 모은 (관절 읽음, 사진 속 표적 화소) 쌍에서 팔·카메라 모형을 푼다 (PC, scipy).

    python ros2/tools/kincal_solve.py <세션 폴더>... [--out model.json] [--stage auto|A|B|C|D]
    python ros2/tools/kincal_solve.py --selftest

재는 것은 **재투영 오차(px)** 하나다: 모형이 "이 관절값이면 보드 모서리가 화면 여기"라고 한 자리와 실제 검출 자리의 차.
깊이를 안 쓰므로 D405 깊이 잡음·검은 칸 구멍과 무관하고, 미터 눈금은 팔 링크 길이와 보드 칸(인쇄 배율 포함)이 준다.

기구학 뼈대는 공식 SO-101 URDF(치수는 안 푼다, kincal_model 머리말). 책상 보드는 밑동과 한 평면으로 묶는다.
단계(뒤로 갈수록 미지수가 는다) — **남겨 둔 자세(20%)에서의 오차**가 줄 때만 다음 단계를 받아들인다.
최소자승은 미지수를 늘리면 맞춘 쪽 잔차는 언제나 준다. 그건 증거가 아니다.
  A 손-눈 6 + 보드(방위·자리·작은 기울기) + 관절 영점 3     B + 관절 틀이 CAD와 다른 만큼(geo 17)
  C + 처짐 3 · 백래시 5     D + 초점·왜곡·roll 눈금
밑동이 넘어가는 자세(어깨 토크 < ROCK_TAU)는 빼고 푼다 — 그건 팔이 아니라 놓인 상태의 문제다.
서보 방향 ↔ URDF 축 부호 32조합은 A에서 다 풀어 가장 낮은 것을 고른다.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kincal_model as km  # noqa: E402

A = ["cam_r", "cam_t", "board_yaw", "board_t", "board_tilt", "off_lift", "off_elbow", "off_wrist"]
B = A + ["geo_lift", "geo_elbow", "geo_wrist", "geo_roll"]
C = B + ["sag_lift", "sag_elbow", "sag_wrist", "bl_pan", "bl_lift", "bl_elbow", "bl_wrist", "bl_roll"]
D = C + ["f_scale", "k1", "roll_scale"]
STAGES = {"A": A, "B": B, "C": C, "D": D}
PLAY = ["play_deg", "play_tau0", "play_w", "play_pr", "play_pz"]     # 진단용(--play): 밑동이 넘어가는 크기·회전 중심
GEO_SIGMA = {5: [5.0, 5.0, 5.0, 0.035, 0.035], 4: [5.0, 5.0, 0.035, 0.035]}     # 옮김 5mm · 기울기 2°
# 사전값(σ) — 자료가 말해 주면 묻히고, 자료가 못 보는 방향에서만 값을 붙든다.
# board_tilt만은 **물리 사실**이다: 팔 밑동과 종이가 같은 책상 위에 평평히 있다(11/12 사진 확인) → ±1.5°.
# 책상 높이도 마찬가지: URDF base_link의 z=0이 밑동 바닥이고 그 바닥이 책상에 닿아 있다 → 보드 z = 0 ± 10mm.
# 이 둘은 관측 수에 비례해 세게 건다(STRONG) — 약하게 걸었더니 영점 175°·책상 z +183mm인 "뒤집힌 세계" 해로 빠졌다.
PRIOR = {"play_deg": 15.0, "play_tau0": 20.0, "play_w": 6.0, "play_pr": 100.0, "play_pz": 100.0, "off_lift": 20.0, "off_elbow": 20.0, "off_wrist": 20.0, "board_tilt": 0.026,
         "sag_lift": 0.05, "sag_elbow": 0.05, "sag_wrist": 0.05,
         "bl_pan": 1.5, "bl_lift": 1.5, "bl_elbow": 1.5, "bl_wrist": 1.5, "bl_roll": 1.5,
         "f_scale": 0.02, "k1": 0.1, "roll_scale": 0.03}
STRONG_Z_MM = 40.0           # 밑동 아래에 발판이 있다(사진) — 책상이 base_link z=0보다 수 cm 아래일 수 있다
# **넘을 수 없는 담** — 사전값(무게)만으로는 기울기가 170°로 넘어가 뒤집힌 해에 앉았다. 물리적으로 불가능한 쪽은 아예 막는다.
BOUND = {"play_deg": 20.0, "play_tau0": 40.0, "play_pr": 200.0, "play_pz": 200.0, "board_tilt": 0.06, "off_lift": 45.0, "off_elbow": 45.0, "off_wrist": 45.0}
BOARD_Z = (-90.0, 30.0)
XSCALE = {"cam_r": 1.0, "cam_t": 50.0, "board_yaw": 1.0, "board_t": 50.0}
LOSS_PX = 2.0
MAX_READ_JITTER = 0.4        # 사진 앞뒤 읽음이 이만큼(도) 다르면 팔이 아직 움직이던 것 — 버린다
MIN_CORNERS = 4


def load(dirs: list) -> dict:
    """세션 폴더들 → 관측 배열. 관절은 수집기가 적은 `deg`(read2를 그 세션 보정표로 바꾼 도)를 서보 눈금 도 q로 옮겨 쓴다."""
    q, dr, uv, cid, fidx, meta = [], [], [], [], [], []
    servo = []                                                  # (지령 도, 읽음 도, 접근 방향) — 보드가 안 보여도 쓴다
    ses = None
    for d in dirs:
        s = json.load(open(os.path.join(d, "session.json"), encoding="utf-8"))
        if ses is not None and (s["print_scale"] != ses["print_scale"] or s["cart"] != ses["cart"] or s["cal"] != ses["cal"]):
            raise SystemExit("세션끼리 보정표·인쇄 배율이 다르다 — 같이 못 푼다: " + d)
        ses = s
        Am, Bm = km.tick_map(s["cal"], s["cart"])
        for line in open(os.path.join(d, "samples.jsonl"), encoding="utf-8"):
            r = json.loads(line)
            if max(abs(r["read1"][j] - r["read2"][j]) for j in km.J) > MAX_READ_JITTER:
                continue
            servo.append((r["goal_deg"], r["deg"], r["dirs"]))
            qq = (Am * np.array(r["deg"]) + Bm).tolist()
            if len(r["ch"]) < MIN_CORNERS:
                continue
            k = len(q)
            q.append(qq)
            dr.append(r["dirs"])
            meta.append((d, r["i"], r["kind"]))
            for c, p in r["ch"].items():
                cid.append(int(c))
                uv.append(p)
                fidx.append(k)
    if ses is None or not q:
        raise SystemExit("쓸 표본이 없다")
    return {"q": np.array(q), "dirs": np.array(dr), "uv": np.array(uv), "cid": np.array(cid),
            "fidx": np.array(fidx), "meta": meta, "intr": ses["intrinsics"], "ses": ses, "servo": np.array(servo),
            "bp": km.board_points(ses["charuco"], ses["print_scale"])}


def _pack(P, names):
    return np.concatenate([np.atleast_1d(np.asarray(P[n], float)) for n in names])


def _unpack(P0, names, x):
    P, k = dict(P0), 0
    for n in names:
        if isinstance(P0[n], (list, np.ndarray)):
            m = len(P0[n])
            P[n] = x[k:k + m].tolist()
            k += m
        else:
            P[n] = float(x[k])
            k += 1
    return P


def reproj(P, Dt, frames=None):
    """모서리마다 (예측 − 검출) px, 카메라 z(mm). frames = 쓸 자세 번호(없으면 전부)."""
    m = np.ones(len(Dt["fidx"]), bool) if frames is None else np.isin(Dt["fidx"], frames)
    c = km.chain(P, Dt["q"], Dt["dirs"])
    X = km.target_base(P, "board", Dt["bp"])[Dt["cid"][m]]
    uv, zc = km.project(P, Dt["intr"], c["R_bc"], c["t_bc"], X, Dt["fidx"][m])
    return uv - Dt["uv"][m], zc


def fit(P0, names, Dt, frames=None, max_nfev=400):
    pri = [n for n in names if n in PRIOR]

    def res(x):
        P = _unpack(P0, names, x)
        e, _ = reproj(P, Dt, frames)
        w = np.sqrt(e.size / 50.0)                              # 물리 사실 둘은 관측 50개에 1개꼴 무게로
        return np.concatenate([e.ravel() / LOSS_PX]
                              + [np.atleast_1d(np.asarray(P[n]) - np.asarray(km.DEFAULT[n])) / PRIOR[n] * (w if n == "board_tilt" else 1.0)
                                 for n in pri]
                              + [np.asarray(P[n]) / GEO_SIGMA[len(P[n])] for n in names if n.startswith("geo_")]
                              + ([[w * P["board_t"][2] / STRONG_Z_MM]] if "board_t" in names else []))

    xs = np.concatenate([GEO_SIGMA[len(P0[n])] if n.startswith("geo_") else
                         [XSCALE.get(n, PRIOR.get(n, 1.0))] * len(np.atleast_1d(P0[n])) for n in names])
    lb, ub = [], []
    for n in names:
        m = len(np.atleast_1d(P0[n]))
        b = BOUND.get(n, np.inf)
        lo, hi = [-b] * m, [b] * m
        if n == "board_t":
            lo[2], hi[2] = BOARD_Z
        if n == "play_w":
            lo, hi = [1.0], [20.0]
        lb += lo
        ub += hi
    x0 = np.clip(_pack(P0, names), np.array(lb) + 1e-9, np.array(ub) - 1e-9)
    r = least_squares(res, x0, loss="soft_l1", x_scale=xs, max_nfev=max_nfev, bounds=(lb, ub))
    return _unpack(P0, names, r.x)


def rms(e) -> float:
    return float(np.sqrt((e ** 2).sum(axis=1).mean()))


def _pnp(Dt):
    """자세마다 보드↔카메라(PnP). 모서리 6개 이상인 장만. → (자세 번호들, R_cam_board들, t들)"""
    import cv2
    it = Dt["intr"]
    K = np.array([[it["fx"], 0, it["ppx"]], [0, it["fy"], it["ppy"]], [0, 0, 1.0]])
    fs, Rt, tt = [], [], []
    for f in range(len(Dt["q"])):
        m = Dt["fidx"] == f
        if m.sum() < 6:
            continue
        ok, rv, tv = cv2.solvePnP(Dt["bp"][Dt["cid"][m]], Dt["uv"][m], K, None, flags=cv2.SOLVEPNP_IPPE)
        if ok:
            fs.append(f)
            Rt.append(cv2.Rodrigues(rv)[0])
            tt.append(tv.ravel())
    if len(fs) < 5:
        raise SystemExit("모서리 6개 이상 보이는 자세가 5개 미만 — 초기값을 못 잡는다")
    return np.array(fs), Rt, tt


def init_signs(Dt, pnp, signs):
    """한 부호 조합의 초기값: OpenCV 손-눈 → 보드를 책상에 눕힌 자세로 → (거친 A단계 맞춤 RMS, 모형)."""
    import cv2
    fs, Rt, tt = pnp
    P = dict(km.DEFAULT, **dict(zip(km._SGN, signs)))
    c = km.chain(P, Dt["q"])
    Rg, tg = [c["R_bt"][f] for f in fs], [c["p_g"][f] for f in fs]
    best = None
    for method in (cv2.CALIB_HAND_EYE_TSAI, cv2.CALIB_HAND_EYE_PARK):
        try:
            Rx, tx = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=method)
        except cv2.error:
            continue
        if not (np.all(np.isfinite(Rx)) and np.all(np.isfinite(tx))):
            continue
        # 보드 자세: 자세마다 Rg·Rx·Rt 를 내어 z축 평균으로 위/아래(flip)를, x축 평균으로 방위를 정한다
        Rb = np.array([Rg[k] @ Rx @ Rt[k] for k in range(len(fs))])
        tb = np.median([Rg[k] @ (Rx @ tt[k] + tx.ravel()) + tg[k] for k in range(len(fs))], axis=0)
        # 종이는 앞면이 위다 → 보드 z축(종이 속으로)은 base −z (board_flip = −1 고정). 초기값이 거꾸로 선 조합은
        # 여기서 억지로 눕혀지고 맞춤 오차가 크게 남아 떨어진다.
        xax = Rb[:, :, 0].mean(axis=0)
        Q = dict(P, cam_r=km.rotvec(Rx).tolist(), cam_t=tx.ravel().tolist(), board_flip=-1.0,
                 board_yaw=float(np.arctan2(xax[1], xax[0])), board_t=[tb[0], tb[1], 0.0])
        Q = fit(Q, A, Dt, max_nfev=40)
        e = rms(reproj(Q, Dt)[0])
        if best is None or e < best[0]:
            best = (e, Q)
    return best or (1e9, P)


def solve(Dt, stage="auto", seed=0, quiet=False):
    import itertools
    say = (lambda *a: None) if quiet else print
    F = len(Dt["q"])
    say("표본 %d자세 · 모서리 %d개" % (F, len(Dt["uv"])))
    pnp = _pnp(Dt)
    cands = sorted((init_signs(Dt, pnp, sg) + (sg,) for sg in itertools.product((1.0, -1.0), repeat=5)), key=lambda t: t[0])
    cands = sorted(((rms(reproj(Q, Dt)[0]), Q, sg) for Q, sg in ((fit(Q, A, Dt), sg) for _, Q, sg in cands[:4])),
                   key=lambda t: t[0])                           # 앞의 넷만 끝까지 맞춰 다시 줄 세운다
    say("\n[부호] 서보가 느는 방향 ↔ URDF 축 (pan lift elbow wrist roll) — 32조합 중 앞의 넷, 손-눈+보드+영점 맞춤 RMS")
    for e, Q, sg in cands:
        say("  %s : %6.2f px  |t_cam| %4.0fmm  책상 z %+5.0fmm" % (" ".join("%+d" % s for s in sg), e,
                                                                 np.linalg.norm(Q["cam_t"]), Q["board_t"][2]))
    PA = cands[0][1]
    if cands[1][0] < cands[0][0] * 1.3:
        say("  ⚠ 1등과 2등이 30%% 안이다(%.2f vs %.2f) — 자세가 좁아 부호를 아직 못 가른다" % (cands[0][0], cands[1][0]))
    # 썩은 장만 뺀다(오검출 수준, 60px 넘게). ⚠ 11/12 처음엔 "중앙값의 4배(8px) 넘으면 이상치"로 뺐더니 **새로 간 넓은 자세가
    #   전부 버려졌다** — 모형이 못 맞히는 자세는 이상치가 아니라 모형이 틀렸다는 증거다. 그건 남겨서 풀이가 받아내게 한다.
    e0 = reproj(PA, Dt)[0]
    per0 = np.array([rms(e0[Dt["fidx"] == f]) for f in range(F)])
    cut = 60.0
    tau0 = km.lift_torque(PA, Dt["q"])
    good = np.flatnonzero((per0 <= cut) & (tau0 >= km.ROCK_TAU))
    if (tau0 < km.ROCK_TAU).any():
        say("  밑동이 넘어가는 자세 %d장 뺌(어깨 중력 토크 < %.0f kg·mm — kincal_model.ROCK_TAU)" % ((tau0 < km.ROCK_TAU).sum(), km.ROCK_TAU))
        PA = fit(PA, A, Dt, good)
    if (per0 > cut).any():
        say("  썩은 장 %d장 뺌(자세 RMS > %.0fpx): %s" % (F - len(good), cut, ", ".join(
            "%s#%d %.0fpx" % (os.path.basename(os.path.normpath(Dt["meta"][f][0])), Dt["meta"][f][1], per0[f])
            for f in np.flatnonzero(per0 > cut)[:8])))
    rng = np.random.default_rng(seed)
    test = np.sort(rng.choice(good, max(1, len(good) // 5), replace=False))
    train = np.setdiff1d(good, test)
    say("\n[단계] 맞춘 80%% / 남겨 둔 20%%(%d자세) 재투영 RMS px" % len(test))
    P, hold, chosen, table = PA, None, "A", {}
    for name in "ABCD":
        Q = fit(P if name != "A" else PA, STAGES[name], Dt, train)
        tr, te = rms(reproj(Q, Dt, train)[0]), rms(reproj(Q, Dt, test)[0])
        table[name] = (tr, te)
        take = hold is None or te < hold * 0.98
        say("  %s  맞춤 %6.2f  남김 %6.2f  %s" % (name, tr, te, "←" if take else "(안 받음 — 남긴 쪽이 안 준다)"))
        if stage == "auto":
            if not take:
                break
            P, hold, chosen = Q, te, name
        elif name == stage:
            P, hold, chosen = Q, te, name
            break
        else:
            P = Q
    P = fit(P, STAGES[chosen], Dt, good)                         # 고른 단계로 (이상치 뺀) 전부 써서 마지막 맞춤
    ea = reproj(P, Dt)[0]
    per = np.array([rms(ea[Dt["fidx"] == f]) for f in range(F)])
    e, zc = reproj(P, Dt, good)
    mm = float(np.sqrt((((e / Dt["intr"]["fx"]) * zc[:, None]) ** 2).sum(axis=1).mean()))
    ctr = km.target_base(P, "board", Dt["bp"]).mean(axis=0)
    pg = per[good]
    say("\n[결과] 단계 %s · 전체 재투영 %.2f px ≈ 표적 위 %.2f mm · 남겨 둔 자세 %.2f px · 자세별 중앙 %.1f / 90%% %.1f / 최대 %.1f px" % (
        chosen, rms(e), mm, hold, np.median(pg), np.percentile(pg, 90), pg.max()))
    rock_n = int((tau0 < km.ROCK_TAU).sum())
    if rock_n >= 8:                                             # 뺀 자세들로 "넘어가는 크기·회전 중심"만 따로 풀어 알려 준다(모형엔 안 넣는다)
        R = fit(dict(P, play_deg=-5.0, play_tau0=3.0), PLAY, Dt, np.flatnonzero(per0 <= cut))
        say("  밑동 넘어감(뺀 %d자세로 진단): %.1f° · 어깨 토크 %.1f kg·mm 아래에서 · 회전 중심 = 어깨 축에서 수평 %+.0f / 높이 %+.0f mm"
            " (높이가 −60~−120이면 발판이 까딱이는 것 · 0 근처면 어깨 체결 유격)" % (
                rock_n, R["play_deg"], R["play_tau0"], R["play_pr"], R["play_pz"]))
    for k in ("geo_lift", "geo_elbow", "geo_wrist", "geo_roll"):
        if k in STAGES[chosen]:
            g = P[k]
            say("  %-9s 옮김 %s mm · 축 기울기 %s°" % (k, np.round(g[:len(g) - 2], 1).tolist(), np.round(np.degrees(g[-2:]), 2).tolist()))
    say("  부호 %s · 손-눈 |t| %.1fmm t=%s (gripper_link 기준)" % (
        " ".join("%+d" % P[k] for k in km._SGN), np.linalg.norm(P["cam_t"]), np.round(P["cam_t"], 1).tolist()))
    say("  보드: 중심 base (%.0f, %.0f, %.1f)mm · 기울기 %.2f° — 책상이 밑동과 한 평면이면 z≈0·기울기≈0이어야 한다" % (
        ctr[0], ctr[1], ctr[2], np.degrees(np.hypot(*P["board_tilt"]))))
    say("  " + " · ".join("%s %+.3f" % (k, P[k] - km.DEFAULT[k]) for k in STAGES[chosen] if k in PRIOR and k != "board_tilt"))
    say("  자세별 px(이 값이 넓은 자세에서만 크면 모형 구조가 모자란 것): 중앙 %.1f" % np.median(pg))
    worst = np.argsort(-per)[:5]
    say("  가장 나쁜 자세: " + ", ".join("#%d %.1fpx" % (Dt["meta"][f][1], per[f]) for f in worst))
    return {"P": P, "intr": Dt["intr"], "stage": chosen, "servo": servo_fit(Dt, quiet) if "servo" in Dt else None,
            "rms_px": rms(e), "rms_mm": mm, "holdout_px": hold, "stages": table, "frames": F, "outliers": int((per0 > cut).sum()), "rocking": rock_n,
            "print_scale": Dt["ses"]["print_scale"], "charuco": Dt["ses"].get("charuco"), "dots_mm": Dt["ses"].get("dots_mm"),
            "tick_map": [x.tolist() for x in km.tick_map(Dt["ses"]["cal"], Dt["ses"]["cart"])] if "cal" in Dt["ses"] else None,
            "kinematics": "so101_new_calib.urdf", "signs_px": [[list(sg), c] for c, _, sg in cands]}


def servo_fit(Dt, quiet=False) -> dict:
    """**지령 → 실제 도착 각** (트윈의 가짜 팔이 쓰는 준정적 모형, twin_io.settle과 같은 식).

        읽음 − 지령 = bias − f·(접근 방향) − 중력토크/K        (관절마다 최소자승)

    f = 서보가 목표 앞에서 서는 폭(불감대+정지마찰, 도) — 다가온 쪽에 남는다. K = 중력에 버티는 강성(토크/도).
    손목 카메라 자료가 아니라 **엔코더 자료**라 표본이 많고 모형과 무관하다."""
    S = Dt["servo"]
    goal, got, dirs = S[:, 0], S[:, 1], S[:, 2]
    a1 = np.radians(got[:, 1]); a2 = a1 + np.radians(got[:, 2]); a3 = a2 + np.radians(got[:, 3])
    G = km._G
    tw = km.M_HAND * km.HAND_D * np.cos(a3)
    te = tw + (km.M_HAND + km.M2 / 2) * G.l2 * np.cos(a2)
    tl = te + (km.M_HAND + km.M2 + km.M_ELBOW + km.M1 / 2) * G.l1 * np.cos(a1)
    tau = {1: tl, 2: te, 3: tw}
    out = {"n": int(len(S)), "f": {}, "K": {}, "bias": {}, "resid_deg": {}, "raw_deg": {}}
    if not quiet:
        print("\n[서보] 지령 → 도착 (표본 %d)   f=접근 쪽에 남는 폭°  K=강성  잔차=모형 뒤 흩어짐°(앞)" % len(S))
    for k, j in enumerate(km.J):
        e = got[:, k] - goal[:, k]
        cols = [np.ones(len(e)), -dirs[:, k]] + ([-tau[k]] if k in tau else [])
        x, *_ = np.linalg.lstsq(np.stack(cols, axis=1), e, rcond=None)
        r = e - np.stack(cols, axis=1) @ x
        out["bias"][j], out["f"][j] = float(x[0]), float(x[1])
        out["K"][j] = float(1.0 / x[2]) if k in tau and abs(x[2]) > 1e-6 else None
        out["resid_deg"][j], out["raw_deg"][j] = float(r.std()), float(e.std())
        if not quiet:
            print("  %-13s f %+.2f  K %s  bias %+.2f  잔차 %.2f (%.2f)" % (
                j, x[1], "%6.0f" % out["K"][j] if out["K"][j] else "     -", x[0], r.std(), e.std()))
    return out


MARKER_ID, MARKER_MM = 49, 160.0     # 벽의 HOME 마커(docs/aruco/home_floor.pdf, DICT_4X4_50) — 인쇄 배율은 세션 값을 곱한다


def marker_fit(M: dict, dirs: list, quiet=False):
    """**벽의 ArUco 마커 = 독립 검증.** 팔·카메라 모형(M)은 책상 보드만으로 맞춘 그대로 두고, 벽 마커의 자세 6개만 푼다.
    그러고도 마커 네 모서리가 예측 자리에 찍히면, 모형이 보드에 끼워 맞춘 게 아니라 **다른 평면·다른 방향·다른 거리에서도 맞는다.**
    마커는 사진에서 다시 찾는다(세션 폴더에 img/가 있어야 한다, 결과는 aruco<id>.json에 적어 두고 다시 쓴다).
    깊이로 찾은 벽 평면(M["wallplane"])이 있으면 마커가 그 평면 위에 있는지도 본다 — 서로 다른 세 가지 측정이 한 벽을 가리키는가."""
    import cv2
    say = (lambda *a: None) if quiet else print
    P, it = M["P"], M["intr"]
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    q, dr, obs, tag = [], [], [], []
    scale = M["print_scale"]
    for d in dirs:
        ses = json.load(open(os.path.join(d, "session.json"), encoding="utf-8"))
        Am, Bm = km.tick_map(ses["cal"], ses["cart"])
        cache_path = os.path.join(d, "aruco%d.json" % MARKER_ID)
        cache = json.load(open(cache_path)) if os.path.exists(cache_path) else {}
        dirty = False
        for line in open(os.path.join(d, "samples.jsonl"), encoding="utf-8"):
            r = json.loads(line)
            key = str(r["i"])
            if key not in cache:
                img = cv2.imread(os.path.join(d, r["img"]))
                if img is None:
                    continue                                     # 사진을 아직 안 가져왔다 — 다음에 다시 본다
                mc, mi, _ = det.detectMarkers(img)
                cache[key] = None if mi is None else next(
                    (c.reshape(4, 2).tolist() for c, k in zip(mc, mi.ravel()) if int(k) == MARKER_ID), None)
                dirty = True
            if cache[key] is None or max(abs(r["read1"][j] - r["read2"][j]) for j in km.J) > MAX_READ_JITTER:
                continue
            q.append(Am * np.array(r["deg"]) + Bm)
            dr.append(r["dirs"])
            obs.append(cache[key])
            tag.append((d, r["i"]))
        if dirty:
            json.dump(cache, open(cache_path, "w"))
    if len(q) < 6:
        say("\n[벽 마커] ArUco %d가 보인 장이 %d장 — 아직 못 푼다(사진을 가져왔는가)" % (MARKER_ID, len(q)))
        return None
    q, dr, obs = np.array(q), np.array(dr), np.array(obs)
    keep = km.lift_torque(P, q) >= km.ROCK_TAU                  # 밑동이 넘어가는 자세는 여기서도 뺀다
    q, dr, obs, tag = q[keep], dr[keep], obs[keep], [t for t, k in zip(tag, keep) if k]
    if len(q) < 6:
        say("\n[벽 마커] 쓸 수 있는 장이 %d장 — 아직 못 푼다" % len(q))
        return None
    c = km.chain(P, q, dr)
    h = MARKER_MM * scale / 2.0
    L = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0.0]])     # OpenCV 마커 모서리 순서(좌상·우상·우하·좌하)
    K = np.array([[it["fx"] * P["f_scale"], 0, it["ppx"]], [0, it["fy"] * P["f_scale"], it["ppy"]], [0, 0, 1.0]])
    fi = np.repeat(np.arange(len(q)), 4)
    li = np.tile(np.arange(4), len(q))
    o = obs.reshape(-1, 2)

    def res(x):
        Q = dict(P, wall_r=x[:3].tolist(), wall_t=x[3:].tolist())
        return (km.project(Q, it, c["R_bc"], c["t_bc"], km.target_base(Q, "wall", L)[li], fi)[0] - o).ravel()

    best = None
    for f in np.linspace(0, len(q) - 1, min(len(q), 12)).astype(int):       # 몇 장에서 초기값을 세워 가장 잘 맞는 것
        ok, rv, tv = cv2.solvePnP(L, obs[f], K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        x0 = np.concatenate([km.rotvec(c["R_bc"][f] @ cv2.Rodrigues(rv)[0]), c["R_bc"][f] @ tv.ravel() + c["t_bc"][f]])
        r = least_squares(res, x0, loss="soft_l1", f_scale=3.0, x_scale=[1, 1, 1, 50, 50, 50])
        if best is None or r.cost < best.cost:
            best = r
    e = res(best.x).reshape(-1, 4, 2)
    per = np.sqrt((e ** 2).sum(axis=2).mean(axis=1))
    Q = dict(P, wall_r=best.x[:3].tolist(), wall_t=best.x[3:].tolist())
    pts = km.target_base(Q, "wall", L)
    zc = np.linalg.norm(pts.mean(axis=0) - c["t_bc"], axis=1)
    mm = float(np.sqrt(((per / it["fx"] * zc) ** 2).mean()))
    n = km.rodrigues(Q["wall_r"])[:, 2]
    out = {"wall_r": Q["wall_r"], "wall_t": Q["wall_t"], "wall_pts": pts.tolist(), "frames": int(len(q)),
           "px": float(np.sqrt((per ** 2).mean())), "px_median": float(np.median(per)), "mm": mm,
           "angle_to_table_deg": float(np.degrees(np.arccos(min(1.0, abs(n @ km.board_R(P)[:, 2])))))}
    say("\n[벽 마커] 독립 검증 — ArUco %d 자세 6개만 풀고 팔 모형은 그대로: %d장 · 모서리 RMS %.2f px(자세별 중앙 %.2f) ≈ 마커 위 %.1f mm · 거리 %d~%dmm" % (
        MARKER_ID, len(q), out["px"], out["px_median"], mm, zc.min(), zc.max()))
    msg = "  마커 중심 base (%.0f, %.0f, %.0f)mm · 책상과 이루는 각 %.1f° (벽이 수직이면 90)" % (*Q["wall_t"], out["angle_to_table_deg"])
    wp = M.get("wallplane")
    if wp:
        dist = np.array(wp["n"]) @ pts.T - wp["d"]
        out["to_depth_wall_mm"] = [float(dist.min()), float(dist.max())]
        out["angle_to_depth_wall_deg"] = float(np.degrees(np.arccos(min(1.0, abs(n @ np.array(wp["n"]))))))
        msg += " · 깊이로 찾은 벽 평면까지 %+.0f~%+.0fmm, 그 평면과 %.1f° (같은 벽이면 0mm·0°)" % (
            dist.min(), dist.max(), out["angle_to_depth_wall_deg"])
    say(msg)
    worst = np.argsort(-per)[:4]
    say("  가장 나쁜 장: " + ", ".join("%s#%d %.1fpx" % (os.path.basename(os.path.normpath(tag[f][0])), tag[f][1], per[f]) for f in worst))
    return out


def overlay(M: dict, Dt: dict, out: str, n: int = 9) -> None:
    """실제 사진 위에 검출(초록 +)과 모형 예측(빨강 ○)을 겹쳐 그린다 — 가장 나쁜 것·중간·가장 좋은 것 n/3장씩."""
    import cv2
    os.makedirs(out, exist_ok=True)
    e, _ = reproj(M["P"], Dt)
    F = len(Dt["q"])
    per = np.array([rms(e[Dt["fidx"] == f]) for f in range(F)])
    order = np.argsort(per)
    k = max(1, n // 3)
    pick = list(order[:k]) + list(order[F // 2 - k // 2:F // 2 - k // 2 + k]) + list(order[-k:])
    c = km.chain(M["P"], Dt["q"], Dt["dirs"])
    X = km.target_base(M["P"], "board", Dt["bp"])
    for f in pick:
        d, i, _ = Dt["meta"][f]
        img = cv2.imread(os.path.join(d, "img", "%06d.jpg" % i))
        if img is None:
            continue
        uv, zc = km.project(M["P"], Dt["intr"], c["R_bc"][f:f + 1], c["t_bc"][f:f + 1], X)
        for (u, v), z in zip(uv, zc):
            if z > 0 and -50 < u < 900 and -50 < v < 530:
                cv2.circle(img, (int(round(u)), int(round(v))), 7, (0, 0, 255), 2)
        for u, v in Dt["uv"][Dt["fidx"] == f]:
            cv2.drawMarker(img, (int(round(u)), int(round(v))), (0, 255, 0), cv2.MARKER_CROSS, 10, 2)
        cv2.putText(img, "#%d  %.1f px" % (i, per[f]), (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imwrite(os.path.join(out, "overlay_%s_%06d.jpg" % (os.path.basename(os.path.normpath(d)), i)), img)
    print("겹쳐 그린 사진 %d장 → %s" % (len(pick), out))


def _selftest() -> None:
    """모형으로 화소를 만들어(부호·영점을 일부러 틀고 0.3px 잡음) 다시 풀어 되찾는가."""
    rng = np.random.default_rng(3)
    intr = {"fx": 438.0, "fy": 437.0, "ppx": 422.7, "ppy": 229.3, "width": 848, "height": 480}
    bp = km.board_points({"nx": 5, "ny": 7, "square_mm": 35.0}, 0.945)
    T = dict(km.DEFAULT, sgn_pan=-1.0, sgn_elbow=-1.0, off_lift=4.0, off_elbow=-3.0, off_wrist=2.0,
             cam_r=[0.1, -0.2, 1.5], cam_t=[0.0, -60.0, -40.0], board_yaw=0.6, board_t=[180.0, -120.0, 1.0],
             board_tilt=[0.005, -0.004])
    # 카메라가 보드를 보는 자세를 고른다: 무작위 자세 중 모서리 8개 이상이 화면에 드는 것
    q, dirs, uv, cid, fidx = [], [], [], [], []
    for x in rng.uniform((-100, -90, -90, -95, -150), (100, 90, 90, 95, 150), size=(400000, 5)):
        c = km.chain(T, x)
        p, zc = km.project(T, intr, c["R_bc"], c["t_bc"], km.target_base(T, "board", bp))
        ok = (p[:, 0] > 10) & (p[:, 0] < 838) & (p[:, 1] > 10) & (p[:, 1] < 470) & (zc > 80) & (zc < 500)
        if ok.sum() < 8:
            continue
        for k in np.flatnonzero(ok):
            uv.append(p[k] + rng.normal(0, 0.3, 2))
            cid.append(k)
            fidx.append(len(q))
        q.append(x)
        dirs.append(rng.choice([-1.0, 1.0], 5))
        if len(q) >= 150:
            break
    assert len(q) >= 150, "합성 자세를 못 만든다 — 진짜 모형(T)의 카메라가 보드를 못 본다(%d)" % len(q)
    Dt = {"q": np.array(q), "dirs": np.array(dirs), "uv": np.array(uv), "cid": np.array(cid),
          "fidx": np.array(fidx), "meta": [("", i, "") for i in range(len(q))], "intr": intr, "bp": bp,
          "ses": {"print_scale": 0.945}}
    M = solve(Dt, quiet=True)
    P = M["P"]
    assert [P[k] for k in km._SGN] == [T[k] for k in km._SGN], [P[k] for k in km._SGN]
    assert M["rms_px"] < 0.6 and M["holdout_px"] < 0.7, M
    for k, tol in (("off_lift", 0.3), ("off_elbow", 0.3), ("off_wrist", 0.3)):
        assert abs(P[k] - T[k]) < tol, (k, P[k], T[k])
    assert np.linalg.norm(np.array(P["cam_t"]) - T["cam_t"]) < 2.0, P["cam_t"]
    assert np.linalg.norm(np.array(P["board_t"]) - T["board_t"]) < 2.0, P["board_t"]
    print("kincal_solve OK  단계 %s  %.2fpx  남김 %.2fpx" % (M["stage"], M["rms_px"], M["holdout_px"]))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="*")
    ap.add_argument("--out", default="")
    ap.add_argument("--stage", default="auto", choices=("auto", "A", "B", "C", "D"))
    ap.add_argument("--overlay", default="", help="사진 위에 예측을 겹쳐 그릴 폴더(세션 폴더에 img/가 있어야 한다)")
    ap.add_argument("--wallplane", default="", help="깊이 영상으로 찾은 벽 평면 json({n, d}) — 모형 파일에 실어 수집기 안전 검사가 쓴다")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        _selftest()
        return 0
    if not a.dirs:
        ap.error("세션 폴더를 달라")
    Dt = load(a.dirs)
    M = solve(Dt, a.stage)
    if a.wallplane:
        M["wallplane"] = json.load(open(a.wallplane))
    M["wall"] = marker_fit(M, a.dirs)
    if M["wall"]:
        M["P"]["wall_r"], M["P"]["wall_t"] = M["wall"]["wall_r"], M["wall"]["wall_t"]
    if a.overlay:
        overlay(M, Dt, a.overlay)
    if a.out:
        json.dump(M, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print("저장 " + a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
