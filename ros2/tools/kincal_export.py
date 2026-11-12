#!/usr/bin/env python3
"""역보정 수집 자료(kincal) → **기존 손-눈 풀이가 먹는 표본 파일**(`handeye_resolve.py`의 handeye_samples.json 꼴).

    python ros2/tools/kincal_export.py data/kincal/s1 ... --out hs_new.json            # 고친 눈금(드라이버 값 = 도)
    python ros2/tools/kincal_export.py data/kincal/s1 ... --out hs_old.json --old      # 옛 눈금(span/200)
    python ros2/tools/handeye_resolve.py hs_new.json --expect-t-mm 60 160 --loo        # 기존 풀이 그대로

기존 스택(`eye.EyeConfig` → `tool_frame_joints` 평면 FK)은 그대로 두고 **표본만 새것으로** 바꿔 푸는 길이다 —
SO-101 URDF 모형(kincal_model)으로 소비자를 갈아 끼우기 전에, 지금 도는 도구들이 바로 쓸 수 있는 `~/arm_eye.json`을 낸다.
카메라계 3D 점은 깊이가 아니라 ChArUco PnP(인쇄 배율 포함)에서 온다. 밑동이 넘어가는 자세(ROCK_TAU)는 뺀다.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kincal_model as km  # noqa: E402

K_TRUE = 4095.0 / 4096.0        # lerobot DEGREES: 읽음값 1 = 360/4095 눈금 도 → 진짜 도로는 이 배


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--old", action="store_true", help="옛 눈금(범위폭/200)으로 도를 만든다 — 비교용")
    ap.add_argument("--model", default="", help="밑동 넘어감 판정에 쓸 kincal 모형(없으면 안 뺀다)")
    ap.add_argument("--n", type=int, default=80, help="고르게 추릴 표본 수")
    ap.add_argument("--min-corners", type=int, default=12)
    a = ap.parse_args()
    P = json.load(open(a.model))["P"] if a.model else None
    out, geom = [], km._G
    for d in a.dirs:
        ses = json.load(open(os.path.join(d, "session.json"), encoding="utf-8"))
        cal, cart, it = ses["cal"], ses["cart"], ses["intrinsics"]
        K = np.array([[it["fx"], 0, it["ppx"]], [0, it["fy"], it["ppy"]], [0, 0, 1.0]])
        bp = km.board_points(ses["charuco"], ses["print_scale"])
        Am, Bm = km.tick_map(cal, cart)
        for line in open(os.path.join(d, "samples.jsonl"), encoding="utf-8"):
            r = json.loads(line)
            if len(r["ch"]) < a.min_corners or max(abs(r["read1"][j] - r["read2"][j]) for j in km.J) > 0.4:
                continue
            if P is not None and km.lift_torque(P, Am * np.array(r["deg"]) + Bm)[0] < km.ROCK_TAU:
                continue
            ids = np.array([int(k) for k in r["ch"]])
            uv = np.array(list(r["ch"].values()))
            ok, rv, tv = cv2.solvePnP(bp[ids], uv, K, None, flags=cv2.SOLVEPNP_IPPE)
            if not ok:
                continue
            rv, tv = cv2.solvePnPRefineLM(bp[ids], uv, K, None, rv, tv)
            # 보이는 모서리가 장마다 달라 무게중심이 같은 점이 아니다 → **보드 전체 24점**을 그 장의 PnP 자세로 다 낸다
            Pc = bp @ cv2.Rodrigues(rv)[0].T + tv.ravel()
            deg = {}
            for j in km.J:
                ref, zero = float(cart["ref_deg"].get(j, 0.0)), float(cart["zero"].get(j, 0.0))
                over = (cart.get("deg_per_norm") or {}).get(j)
                span = (cal[j]["range_max"] - cal[j]["range_min"]) * 360.0 / 4096.0
                per = (abs(float(over)) if over else span / 200.0) if a.old else K_TRUE
                deg[j] = ref + (float(r["read2"][j]) - zero) * per
            out.append({"label": "%s#%d" % (os.path.basename(os.path.normpath(d)), r["i"]), "joints_deg": deg,
                        "dots_mm": {"c%d" % k: p.tolist() for k, p in enumerate(Pc)},
                        "dots_px": {"c%d" % int(k): p for k, p in r["ch"].items()},
                        "plane_mm": float(abs(tv.ravel()[2])), "tilt_deg": 0.0, "reconstructed": None})
    if len(out) > a.n:                                           # 고르게 추린다(풀이는 표본 수십 개를 전제로 짜였다)
        out = [out[i] for i in np.linspace(0, len(out) - 1, a.n).astype(int)]
    json.dump({"geom": {"z0": geom.z0, "d0": geom.d0, "l1": geom.l1, "l2": geom.l2, "l3": geom.l3}, "target": "charuco",
               "expect_mm": None, "charuco": None, "marker": "mid", "scale": "old" if a.old else "true_deg",
               "samples": out}, open(a.out, "w", encoding="utf-8"), ensure_ascii=False)
    print("표본 %d개 → %s (%s 눈금)" % (len(out), a.out, "옛" if a.old else "고친"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
