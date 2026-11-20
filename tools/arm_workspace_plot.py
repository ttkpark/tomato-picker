#!/usr/bin/env python3
"""SO-101 팔의 작업 범위 그림(옆면·윗면) — 역보정 모형 + 캘리브레이션 틱 범위에서 그린다.

    python tools/arm_workspace_plot.py [--wall] [out.png]

--wall = 밑동을 **벽(수직면)에** 단 그림. 팔에서 본 범위는 같고 중력·바닥만 달라진다.

기하 = docs/2026-11-09_손눈보정-역보정/kincal/final-2026-11-12.json(공식 URDF 사슬 + 이 팔의 어긋남), 관절 한계 =
deploy/calibration/tomato_follower.json 의 range_min/max. 점 = 집게가 무는 자리(TCP).
⚠ 자기 충돌·케이블은 안 본다(관절 한계만) — 몸통 근처 안쪽 경계는 실제보다 넓게 나온다.
"""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Wedge

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "ros2", "tools"))
sys.path.insert(0, os.path.join(ROOT, "src"))
import kincal_model as km  # noqa: E402
from tomato_picker import config  # noqa: E402

plt.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
BLUE, DIM, RED = "#2b8fd6", "#1f77b4", "#d62728"
WALL = "--wall" in sys.argv
ARGS = [a for a in sys.argv[1:] if a != "--wall"]

P = dict(km.DEFAULT, **json.load(open(os.path.join(ROOT, "docs", "kincal", "final-2026-11-12.json")))["P"])
cal = json.load(open(os.path.join(ROOT, "deploy", "calibration", "tomato_follower.json")))
lim = {j: [(cal[j][k] - 2048) * 360 / 4096 for k in ("range_min", "range_max")] for j in km.J}   # 서보 눈금 도


def plane(q):
    """자세들 → 팔 평면 좌표(부호 있는 수평거리 r, 높이 z)로 본 [어깨, 팔꿈치, 손목, TCP]. pan=0 기준."""
    c = km.chain(P, q)
    return [np.stack([(c[k][:, :2] - km.PAN_XY) @ U, c[k][:, 2]], 1) for k in ("shoulder", "elbow", "wrist", "tcp")]


# 팔 평면의 앞 방향 U: pan=0에서 TCP가 퍼지는 수평 방향(주성분), 0 자세의 TCP 쪽이 +
n = 130
g = np.array(np.meshgrid(*[np.linspace(*lim[j], n) for j in km.J[1:4]], indexing="ij")).reshape(3, -1).T
q = np.column_stack([np.zeros(len(g)), g, np.zeros(len(g))])
xy = km.chain(P, q)["tcp"][:, :2] - km.PAN_XY
U = np.linalg.svd(xy - xy.mean(0), full_matrices=False)[2][0]
home = np.zeros((1, 5))
if (km.chain(P, home)["tcp"][0, :2] - km.PAN_XY) @ U < 0:
    U = -U
tcp = plane(q)[3]
r, z = tcp[:, 0], tcp[:, 1]


def X(r_, z_):
    """팔 평면 (r, z) → 그림 좌표. 벽 장착이면 벽에서 나온 거리(z)가 가로, 벽을 따라간 거리(r)가 세로."""
    return (z_, r_) if WALL else (r_, z_)


fig, (a1, a2) = plt.subplots(1, 2, figsize=(17, 8.5), gridspec_kw={"width_ratios": [1.15, 1]})
SURF = "벽" if WALL else "책상"

# ── 팔 평면 단면 ──────────────────────────────────────────────────────
cell = 8.0
re, ze = np.arange(-560, 561, cell), np.arange(-360, 601, cell)
H = np.histogram2d(r, z, [re, ze])[0].T > 0                    # [z, r]
rc, zc = (re[:-1] + re[1:]) / 2, (ze[:-1] + ze[1:]) / 2
above = H & (zc[:, None] >= 0)
gx, gy, HH, AA = (zc, rc, H.T, above.T) if WALL else (rc, zc, H, above)
a1.contourf(gx, gy, HH, [0.5, 2], colors=[BLUE], alpha=0.12)   # 관절로 닿는 전부(마운트 평면 뒤 포함)
a1.contourf(gx, gy, AA, [0.5, 2], colors=[BLUE], alpha=0.30)   # 마운트 평면 앞
a1.contour(gx, gy, HH, [0.5], colors=[DIM], linewidths=1.0)
(a1.axvline if WALL else a1.axhline)(0, color="k", lw=1.6)
a1.text(*X(545 if WALL else -545, -8), f"마운트 평면({SURF})", va="top", ha="right" if WALL else "left",
        rotation=90 if WALL else 0, fontsize=9)
R_LOAD, Z_LOAD = config.ARM_LOAD_R_MAX, config.ARM_LOAD_Z_MAX
if not WALL:                                                   # 버티는 한계는 책상 장착에서 잰 것 — 벽에선 중력 방향이 달라 못 쓴다
    a1.plot([R_LOAD, R_LOAD], [0, Z_LOAD], "--", color=RED, lw=1.2)
    a1.plot([0, R_LOAD], [Z_LOAD, Z_LOAD], "--", color=RED, lw=1.2)
    a1.text(R_LOAD + 4, 30, f"버티는 한계 r≤{R_LOAD:.0f}\n(9/18 실측)", color=RED, fontsize=9)
    a1.text(8, Z_LOAD + 6, f"z≤{Z_LOAD:.0f}", color=RED, fontsize=9)

# 팔 뼈대: 앞으로 뻗은 대표 자세(사거리 최대)
best = q[np.argmax(np.where(np.abs(z - plane(home)[0][0, 1]) < 15, r, -1e9))]
pts = np.array([p[0] for p in plane(best[None])])
sk = np.vstack([[0, 0], [0, pts[0, 1]], pts])
a1.plot(*X(sk[:, 0], sk[:, 1]), "-o", color="#333", lw=5, ms=9, mfc="w", mew=2, solid_capstyle="round", zorder=5)
a1.plot(*X(*pts[3]), "o", color=RED, ms=7, zorder=6)
a1.annotate("무는 자리(TCP)", X(*pts[3]), X(pts[3, 0] - 60, pts[3, 1] + 45), color=RED, fontsize=10,
            arrowprops={"arrowstyle": "-", "color": RED})
for a, b, nm in ((0, 1, "상완"), (1, 2, "전완"), (2, 3, "손")):
    m = (pts[a] + pts[b]) / 2
    a1.text(*X(m[0], m[1] - 24), f"{nm} {np.linalg.norm(pts[b] - pts[a]):.0f}", ha="center", va="center",
            rotation=90 if WALL else 0, fontsize=9, color="#333")
a1.annotate("", X(-40, 0), X(-40, pts[0, 1]), arrowprops={"arrowstyle": "<->", "color": DIM})
a1.text(*X(-58, pts[0, 1] / 2), f"{pts[0, 1]:.0f}", ha="center", va="center", color=DIM, fontsize=10)
FWD, BACK = ("위", "아래") if WALL else ("앞", "뒤")
for val, lab in ((r[z >= 0].max(), FWD), (r[z >= 0].min(), BACK)):
    a1.plot(*X([val, val], [-70, 0]), color=DIM, lw=0.8)
    a1.text(*X(val, -110 if WALL else -75), f"{lab} {abs(val):.0f}", ha="center", va="center" if WALL else "top",
            color=DIM, fontsize=10)
zt = z.max()
a1.plot(*X([r[np.argmax(z)], 120], [zt, zt]), color=DIM, lw=0.8)
a1.text(*X(135, zt), f"{'벽에서' if WALL else '위'} {zt:.0f}", va="center", ha="center" if WALL else "left", color=DIM, fontsize=10)
rl, zl = (-560, 560), (-360, 600)
if WALL:
    a1.set(xlim=zl, ylim=rl, xlabel="벽에서 나온 거리 (mm, 벽 = 0)", ylabel="pan 축에서 벽을 따라간 거리 (mm, + = pan 0 방향)",
           title="SO-101 작업 범위 — 벽 장착, 옆에서 본 팔 평면 단면")
else:
    a1.set(xlim=rl, ylim=zl, xlabel="pan 축에서 수평거리 r (mm, +앞)", ylabel="높이 z (mm, 책상 = 0)",
           title="SO-101 작업 범위 — 옆면")
a1.set_aspect("equal")
a1.grid(alpha=0.3)
a1.text(0.02, 0.02, "관절 한계(서보 눈금 도): " + " · ".join(
    f"{n_} {lim[j][0]:+.0f}~{lim[j][1]:+.0f}°" for j, n_ in zip(km.J[1:4], ("어깨", "팔꿈치", "손목"))),
    transform=a1.transAxes, fontsize=9, bbox={"fc": "w", "ec": "0.8"})

# ── pan 축을 따라 본 그림 (책상: 윗면, 앞 = 위 / 벽: 벽을 마주 본 정면, pan 0 = 위) ─────────
up = z >= 0
rf, rb = r[up].max(), -r[up].min()
ang = []                                           # 서보 pan 한계 → 실제 방위(앞 기준, 반시계 +)
for qp in lim["shoulder_pan"]:
    v = km.chain(P, [[qp, *best[1:]]])["tcp"][0, :2] - km.PAN_XY
    f = np.array([U @ v, U[0] * v[1] - U[1] * v[0]])
    ang.append(np.degrees(np.arctan2(f[1], f[0])))
lo, hi = min(ang), max(ang)
for rad, off, al in ((rb, 180, 0.13), (rf, 0, 0.30)):
    a2.add_patch(Wedge((0, 0), rad, 90 + lo + off, 90 + hi + off, fc=BLUE, alpha=al, ec=DIM, lw=1))
if not WALL:
    a2.add_patch(Wedge((0, 0), R_LOAD, 90 + lo, 90 + hi, fill=False, ec=RED, ls="--", lw=1.2))
    a2.text(R_LOAD * 0.72, R_LOAD * 0.72, f"버티는 한계 {R_LOAD:.0f}", color=RED, fontsize=9)
for a_ in (lo, hi):
    t = np.radians(90 + a_)
    a2.plot([0, rf * np.cos(t)], [0, rf * np.sin(t)], color=DIM, lw=1)
    a2.text(1.07 * rf * np.cos(t), 1.07 * rf * np.sin(t), f"{a_:+.0f}°", ha="center", va="center", color=DIM, fontsize=11)
a2.plot([0, 0], [0, pts[3, 0]], "-", color="#333", lw=5, solid_capstyle="round")
a2.plot(0, 0, "o", color="#333", ms=12, mfc="w", mew=2)
a2.plot(0, pts[3, 0], "o", color=RED, ms=7)
a2.text(10, rf - 28, f"{FWD} {rf:.0f}", color=DIM, fontsize=10)
a2.text(10, -rb + 14, f"{BACK} {rb:.0f} (팔을 넘겼을 때)", color=DIM, fontsize=10)
a2.axhline(0, color=RED, lw=0.6)
a2.axvline(0, color=RED, lw=0.6)
m = max(rf, rb) + 60
if WALL:
    a2.set(xlabel="← 왼쪽   벽을 따라 가로 (mm)   오른쪽 →", ylabel="벽을 따라 세로 (mm, 위 = pan 0 방향)",
           title=f"SO-101 작업 범위 — 벽 장착, 정면 (pan {lo:+.0f}° ~ {hi:+.0f}°)")
else:
    a2.set(xlabel="← 왼쪽   y (mm)   오른쪽 →", ylabel="x (mm, 위 = 앞)",
           title=f"SO-101 작업 범위 — 윗면 (pan {lo:+.0f}° ~ {hi:+.0f}°)")
a2.set(xlim=(-m, m), ylim=(-m, m), aspect="equal")
a2.grid(alpha=0.3)

note = ("벽 장착: 팔에서 본 범위는 책상 장착과 같다(90° 눕힌 것) · pan 0을 위로 단 경우 · 중력이 달라 버티는 한계는 미실측"
        if WALL else "진한 파랑 = 책상 위")
fig.text(0.5, 0.012, f"기하 = 역보정 모형(2026-11-12, 공식 SO-101 URDF 사슬) · 한계 = 캘리브레이션 틱 범위 · {note} · "
         "자기 충돌은 안 봄", ha="center", fontsize=9, color="0.35")
fig.tight_layout(rect=(0, 0.03, 1, 1))
out = ARGS[0] if ARGS else os.path.join(ROOT, "docs", f"2026-11-18_so101-작업범위{'-벽장착' if WALL else ''}.png")
fig.savefig(out, dpi=130)
print(out, f"앞 {rf:.0f} 뒤 {rb:.0f} 위 {zt:.0f} pan {lo:+.1f}~{hi:+.1f}")
