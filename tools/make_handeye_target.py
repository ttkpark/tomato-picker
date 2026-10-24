#!/usr/bin/env python3
"""손-눈 보정 4점 표적지(A4, 실제 크기 PDF) — target_check.py의 EXPECT_W/H를 그대로 쓴다.

    python tools/make_handeye_target.py        → docs/print/handeye-target-A4.pdf

인쇄는 **실제 크기(100%, 페이지 맞춤 끔)**. 아래 100mm 눈금자를 자로 재서 맞는지 본다 —
배율이 틀리면 보정이 그만큼 통째로 틀린다(코드는 이 간격을 정답으로 쓴다).
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2", "tools"))
W_MM, H_MM = 210.0, 297.0
DOT_D = 15.0


def main() -> int:
    import ast
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2", "tools", "target_check.py"),
               encoding="utf-8").read()
    vals = {n.targets[0].id: ast.literal_eval(n.value) for n in ast.parse(src).body
            if isinstance(n, ast.Assign) and getattr(n.targets[0], "id", "") in ("EXPECT_W", "EXPECT_H")}
    ew, eh = vals["EXPECT_W"], vals["EXPECT_H"]       # cv2 없이 상수만 읽는다
    fig = plt.figure(figsize=(W_MM / 25.4, H_MM / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W_MM); ax.set_ylim(0, H_MM); ax.set_aspect("equal"); ax.axis("off")
    cx, cy = W_MM / 2, H_MM / 2 + 15
    for sx in (-1, 1):
        for sy in (-1, 1):
            ax.add_patch(Circle((cx + sx * ew / 2, cy + sy * eh / 2), DOT_D / 2, color="black"))
    # 배율 확인용 100mm 눈금자
    y0 = 28
    ax.add_patch(Rectangle((55, y0), 100, 1.2, color="black"))
    for i in range(11):
        h = 6 if i % 5 == 0 else 3
        ax.add_patch(Rectangle((55 + i * 10 - 0.3, y0), 0.6, h, color="black"))
    ax.text(105, y0 - 7, "이 눈금자가 정확히 100mm여야 한다 (실제 크기 100% 인쇄)", ha="center", fontsize=9,
            family=["Malgun Gothic", "AppleGothic", "NanumGothic", "DejaVu Sans"])
    ax.text(105, H_MM - 12, "손-눈 보정 표적 — 점 간격 가로 %.1f × 세로 %.1f mm · 세로로 길게 세운다" % (ew, eh),
            ha="center", fontsize=9, family=["Malgun Gothic", "AppleGothic", "NanumGothic", "DejaVu Sans"])
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "print", "handeye-target-A4.pdf")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, format="pdf")
    # 자체검사: 점 중심 간격이 상수와 같다
    cs = sorted(p.center for p in ax.patches if isinstance(p, Circle))
    assert abs((cs[2][0] - cs[0][0]) - ew) < 1e-9 and abs((cs[1][1] - cs[0][1]) - eh) < 1e-9, cs
    print(out, "· 점 %.1f × %.1f mm" % (ew, eh))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
