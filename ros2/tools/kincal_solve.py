#!/usr/bin/env python3
"""**역보정 풀이** — kincal_collect가 모은 (관절 읽음, 사진 속 표적 화소) 쌍에서 팔·카메라 모형을 푼다 (PC, scipy).

    python ros2/tools/kincal_solve.py <세션 폴더>... [--out model.json] [--stage auto|A|B|C|D]
    python ros2/tools/kincal_solve.py --selftest

재는 것은 **재투영 오차(px)** 하나다: 모형이 "이 관절값이면 보드 모서리가 화면 여기"라고 한 자리와 실제 검출 자리의 차.
깊이를 안 쓰므로 D405 깊이 잡음·검은 칸 구멍과 무관하고, 미터 눈금은 팔 링크 길이와 보드 칸(인쇄 배율 포함)이 준다.

단계(뒤로 갈수록 미지수가 는다) — **남겨 둔 자세(20%)에서의 오차**가 줄 때만 다음 단계를 받아들인다.
최소자승은 미지수를 늘리면 맞춘 쪽 잔차는 언제나 준다. 그건 증거가 아니다.
  A 손-눈 + 보드 자세(12)     B + 관절 영점 3 · 링크 d0·l1·l2     C + 처짐 3 · 백래시 5     D + 초점·왜곡·roll 눈금
거울(sgn_pan × sgn_roll) 4조합은 A에서 다 풀어 가장 낮은 것을 고른다.
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

A = ["cam_r", "cam_t", "board_r", "board_t"]
B = A + ["off_lift", "off_elbow", "off_wrist", "d0", "l1", "l2"]
C = B + ["sag_lift", "sag_elbow", "sag_wrist", "bl_pan", "bl_lift", "bl_elbow", "bl_wrist", "bl_roll"]
D = C + ["f_scale", "k1", "roll_scale"]
STAGES = {"A": A, "B": B, "C": C, "D": D}
# 약한 사전값(σ) — 자료가 말해 주면 묻히고, 자료가 못 보는 방향(좁은 자세 범위)에서만 값을 붙든다.
PRIOR = {"off_lift": 6.0, "off_elbow": 6.0, "off_wrist": 6.0, "d0": 6.0, "l1": 3.0, "l2": 3.0,
         "sag_lift": 0.05, "sag_elbow": 0.05, "sag_wrist": 0.05,
         "bl_pan": 1.5, "bl_lift": 1.5, "bl_elbow": 1.5, "bl_wrist": 1.5, "bl_roll": 1.5,
         "f_scale": 0.02, "k1": 0.1, "roll_scale": 0.03}
LOSS_PX = 2.0
MAX_READ_JITTER = 0.4        # 사진 앞뒤 읽음이 이만큼(도) 다르면 팔이 아직 움직이던 것 — 버린다
MIN_CORNERS = 4


def load(dirs: list) -> dict:
    """세션 폴더들 → 관측 배열. 관절 도는 수집기가 적은 `deg`(read2를 그 세션 보정표로 바꾼 값)를 쓴다."""
    deg, dr, uv, cid, fidx, meta = [], [], [], [], [], []
    servo = []                                                  # (지령 도, 읽음 도, 접근 방향) — 보드가 안 보여도 쓴다
    ses = None
    for d in dirs:
        s = json.load(open(os.path.join(d, "session.json"), encoding="utf-8"))
        if ses is not None and (s["print_scale"] != ses["print_scale"] or s["cart"] != ses["cart"] or s["cal"] != ses["cal"]):
            raise SystemExit("세션끼리 보정표·인쇄 배율이 다르다 — 같이 못 푼다: " + d)
        ses = s
        for line in open(os.path.join(d, "samples.jsonl"), encoding="utf-8"):
            r = json.loads(line)
            if max(abs(r["read1"][j] - r["read2"][j]) for j in km.J) > MAX_READ_JITTER:
                continue
            servo.append((r["goal_deg"], r["deg"], r["dirs"]))
            if len(r["ch"]) < MIN_CORNERS:
                continue
            k = len(deg)
            deg.append(r["deg"])
            dr.append(r["dirs"])
            meta.append((d, r["i"], r["kind"]))
            for c, p in r["ch"].items():
                cid.append(int(c))
                uv.append(p)
                fidx.append(k)
    if ses is None or not deg:
        raise SystemExit("쓸 표본이 없다")
    return {"deg": np.array(deg), "dirs": np.array(dr), "uv": np.array(uv), "cid": np.array(cid),
            "fidx": np.array(fidx), "meta": meta, "intr": ses["intrinsics"], "ses": ses, "servo": np.array(servo),
            "bp": km.board_points(ses["charuco"], ses["print_scale"])}


def _pack(P, names):
    return np.concatenate([np.atleast_1d(np.asarray(P[n], float)) for n in names])


def _unpack(P0, names, x):
    P, k = dict(P0), 0
    for n in names:
        if isinstance(P0[n], (list, np.ndarray)):
            P[n] = x[k:k + 3].tolist()
            k += 3
        else:
            P[n] = float(x[k])
            k += 1
    return P


def reproj(P, Dt, frames=None):
    """모서리마다 (예측 − 검출) px, 카메라 z(mm). frames = 쓸 자세 번호(없으면 전부)."""
    m = np.ones(len(Dt["fidx"]), bool) if frames is None else np.isin(Dt["fidx"], frames)
    c = km.chain(P, Dt["deg"], Dt["dirs"])
    X = km.target_base(P, "board", Dt["bp"])[Dt["cid"][m]]
    uv, zc = km.project(P, Dt["intr"], c["R_bc"], c["t_bc"], X, Dt["fidx"][m])
    return uv - Dt["uv"][m], zc


def fit(P0, names, Dt, frames=None):
    pri = [n for n in names if n in PRIOR]

    def res(x):
        P = _unpack(P0, names, x)
        e, _ = reproj(P, Dt, frames)
        return np.concatenate([e.ravel() / LOSS_PX] + [[(P[n] - km.DEFAULT[n]) / PRIOR[n]] for n in pri])

    xs = np.concatenate([[1.0] * 3 if n.endswith("_r") else [50.0] * 3 if n.endswith("_t") else [PRIOR.get(n, 1.0)]
                         for n in names])
    r = least_squares(res, _pack(P0, names), loss="soft_l1", x_scale=xs, max_nfev=400)
    return _unpack(P0, names, r.x)


def rms(e) -> float:
    return float(np.sqrt((e ** 2).sum(axis=1).mean()))


def init_handeye(Dt, sp, sr):
    """PnP(자세마다 보드↔카메라) + OpenCV 손-눈 초기값들 → A단계로 다듬어 가장 좋은 것."""
    import cv2
    it = Dt["intr"]
    K = np.array([[it["fx"], 0, it["ppx"]], [0, it["fy"], it["ppy"]], [0, 0, 1.0]])
    P = dict(km.DEFAULT, sgn_pan=sp, sgn_roll=sr)
    c = km.chain(P, Dt["deg"])
    Rg, tg, Rt, tt = [], [], [], []
    for f in range(len(Dt["deg"])):
        m = Dt["fidx"] == f
        if m.sum() < 6:
            continue
        ok, rv, tv = cv2.solvePnP(Dt["bp"][Dt["cid"][m]], Dt["uv"][m], K, None, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            continue
        Rg.append(c["R_bt"][f]); tg.append(c["tcp"][f]); Rt.append(cv2.Rodrigues(rv)[0]); tt.append(tv.ravel())
    if len(Rg) < 5:
        raise SystemExit("모서리 6개 이상 보이는 자세가 5개 미만 — 초기값을 못 잡는다")
    best = None
    for method in (cv2.CALIB_HAND_EYE_TSAI, cv2.CALIB_HAND_EYE_PARK, cv2.CALIB_HAND_EYE_HORAUD,
                   cv2.CALIB_HAND_EYE_DANIILIDIS):
        try:
            Rx, tx = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=method)
        except cv2.error:
            continue
        if not np.all(np.isfinite(Rx)) or not np.all(np.isfinite(tx)):
            continue
        Rb = Rg[0] @ Rx @ Rt[0]                                  # 첫 자세로 보드 자세를 세운다
        tb = Rg[0] @ (Rx @ tt[0] + tx.ravel()) + tg[0]
        Q = fit(dict(P, cam_r=km.rotvec(Rx).tolist(), cam_t=tx.ravel().tolist(),
                     board_r=km.rotvec(Rb).tolist(), board_t=tb.tolist()), A, Dt)
        e = rms(reproj(Q, Dt)[0])
        if best is None or e < best[0]:
            best = (e, Q)
    if best is None:
        raise SystemExit("손-눈 초기값을 못 잡았다")
    return best


def solve(Dt, stage="auto", seed=0, quiet=False):
    say = (lambda *a: None) if quiet else print
    F = len(Dt["deg"])
    say("표본 %d자세 · 모서리 %d개" % (F, len(Dt["uv"])))
    say("\n[거울] sgn_pan × sgn_roll — A단계(손-눈 + 보드) 재투영 RMS")
    cands = []
    for sp in (1.0, -1.0):
        for sr in (1.0, -1.0):
            e, Q = init_handeye(Dt, sp, sr)
            cands.append((e, Q))
            say("  pan %+.0f roll %+.0f : %6.2f px  |t_cam| %5.0fmm" % (sp, sr, e, np.linalg.norm(Q["cam_t"])))
    cands.sort(key=lambda t: t[0])
    PA = cands[0][1]
    if cands[1][0] < cands[0][0] * 1.3:
        say("  ⚠ 1등과 2등이 30%% 안이다(%.2f vs %.2f) — 자세가 좁아 거울을 아직 못 가른다" % (cands[0][0], cands[1][0]))
    rng = np.random.default_rng(seed)
    test = np.sort(rng.choice(F, max(1, F // 5), replace=False))
    train = np.setdiff1d(np.arange(F), test)
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
    P = fit(P, STAGES[chosen], Dt)                               # 고른 단계로 전부 써서 마지막 맞춤
    e, zc = reproj(P, Dt)
    mm = float(np.sqrt((((e / Dt["intr"]["fx"]) * zc[:, None]) ** 2).sum(axis=1).mean()))
    per = np.array([rms(e[Dt["fidx"] == f]) for f in range(F)])
    Rb = km.rodrigues(P["board_r"])
    n = Rb[:, 2] * np.sign(Rb[2, 2] or 1.0)
    ctr = km.target_base(P, "board", Dt["bp"]).mean(axis=0)
    say("\n[결과] 단계 %s · 전체 재투영 %.2f px ≈ 표적 위 %.2f mm · 남겨 둔 자세 %.2f px" % (chosen, rms(e), mm, hold))
    say("  거울 pan %+.0f roll %+.0f · 손-눈 |t| %.1fmm t=%s" % (
        P["sgn_pan"], P["sgn_roll"], np.linalg.norm(P["cam_t"]), np.round(P["cam_t"], 1).tolist()))
    say("  보드: 중심 base (%.0f, %.0f, %.0f)mm · 법선이 base z와 %.1f° (책상이 수평이면 0에 가까워야 한다)" % (
        ctr[0], ctr[1], ctr[2], np.degrees(np.arccos(min(1.0, abs(n[2]))))))
    say("  " + " · ".join("%s %+.3f" % (k, P[k] - km.DEFAULT[k]) for k in STAGES[chosen] if k in PRIOR))
    worst = np.argsort(-per)[:5]
    say("  가장 나쁜 자세: " + ", ".join("#%d %.1fpx" % (Dt["meta"][f][1], per[f]) for f in worst))
    return {"P": P, "intr": Dt["intr"], "stage": chosen, "servo": servo_fit(Dt, quiet) if "servo" in Dt else None, "rms_px": rms(e), "rms_mm": mm, "holdout_px": hold,
            "stages": table, "frames": F, "print_scale": Dt["ses"]["print_scale"],
            "charuco": Dt["ses"].get("charuco"), "dots_mm": Dt["ses"].get("dots_mm"),
            "mirror_px": [[q["sgn_pan"], q["sgn_roll"], c] for c, q in cands]}


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


def _selftest() -> None:
    """모형으로 화소를 만들어(영점·링크·거울을 일부러 틀고 0.3px 잡음) 다시 풀어 되찾는가."""
    rng = np.random.default_rng(3)
    intr = {"fx": 438.0, "fy": 437.0, "ppx": 422.7, "ppy": 229.3, "width": 848, "height": 480}
    spec = {"nx": 5, "ny": 7, "square_mm": 35.0}
    bp = km.board_points(spec, 0.945)
    T = dict(km.DEFAULT, sgn_pan=-1.0, sgn_roll=1.0, off_lift=2.0, off_elbow=-1.5, off_wrist=1.0, l1=118.0, d0=-29.0,
             cam_r=km.rotvec(km.rodrigues([0, 1.4, 0]) @ km.rodrigues([0, 0, -1.5])).tolist(), cam_t=[-60.0, 5.0, 50.0],
             board_r=[0.0, 0.0, 0.4], board_t=[150.0, -160.0, -20.0])
    deg, dirs, uv, cid, fidx = [], [], [], [], []
    while len(deg) < 150:
        q = rng.uniform((-60, 20, 20, -90, -90), (60, 160, 150, 100, 90))
        c = km.chain(T, q)
        p, zc = km.project(T, intr, c["R_bc"], c["t_bc"], km.target_base(T, "board", bp))
        ok = (p[:, 0] > 10) & (p[:, 0] < 838) & (p[:, 1] > 10) & (p[:, 1] < 470) & (zc > 80) & (zc < 500)
        if ok.sum() < 8:
            continue
        for k in np.flatnonzero(ok):
            uv.append(p[k] + rng.normal(0, 0.3, 2)); cid.append(k); fidx.append(len(deg))
        deg.append(q); dirs.append(rng.choice([-1.0, 1.0], 5))
    Dt = {"deg": np.array(deg), "dirs": np.array(dirs), "uv": np.array(uv), "cid": np.array(cid),
          "fidx": np.array(fidx), "meta": [("", i, "") for i in range(len(deg))], "intr": intr, "bp": bp,
          "ses": {"print_scale": 0.945}}
    M = solve(Dt, quiet=True)
    P = M["P"]
    assert (P["sgn_pan"], P["sgn_roll"]) == (-1.0, 1.0), (P["sgn_pan"], P["sgn_roll"])
    assert M["rms_px"] < 0.6 and M["holdout_px"] < 0.7, M
    assert M["stage"] in "BCD", M["stage"]
    for k, tol in (("off_lift", 0.3), ("off_elbow", 0.3), ("off_wrist", 0.3), ("l1", 1.0), ("d0", 1.5)):
        assert abs(P[k] - T[k]) < tol, (k, P[k], T[k])
    assert np.linalg.norm(np.array(P["cam_t"]) - T["cam_t"]) < 2.0, P["cam_t"]
    assert np.linalg.norm(np.array(P["board_t"]) - T["board_t"]) < 2.0, P["board_t"]
    print("kincal_solve OK  단계 %s  %.2fpx  남김 %.2fpx" % (M["stage"], M["rms_px"], M["holdout_px"]))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="*")
    ap.add_argument("--out", default="")
    ap.add_argument("--stage", default="auto", choices=("auto", "A", "B", "C", "D"))
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        _selftest()
        return 0
    if not a.dirs:
        ap.error("세션 폴더를 달라")
    M = solve(load(a.dirs), a.stage)
    if a.out:
        json.dump(M, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print("저장 " + a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
