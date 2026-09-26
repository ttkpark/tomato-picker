#!/usr/bin/env python3
"""줄기 절단점 도출 자체검증 — 카메라도 실제 줄기도 없이 개발 PC에서 돈다.

    python ros2/tools/stem_cut_check.py

`tomato_perception/stem_cut.py`의 세선화·절단점 계산을 합성 마스크로 시험한다.
실기 검증(진짜 줄기 마스크에서 통하는지)은 이 자체검증의 범위 밖이다 —
2026-09-11 개발일지가 이미 "가는 표적은 시뮬레이션이 아니라 실측으로만 검증된다"
고 못박아 뒀다. 여기서 보는 것은 **수학이 스스로 모순되지 않는가**뿐이다.
"""

from __future__ import annotations

import os
import sys

import numpy as np

for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8")

HERE = os.path.dirname(os.path.abspath(__file__))
ROS2 = os.path.dirname(HERE)
SRC = os.path.join(ROS2, "src")
sys.path.insert(0, os.path.join(SRC, "tomato_perception"))

from tomato_perception.stem_cut import (  # noqa: E402
    find_cut_point, skeleton_points, zhang_suen_thin,
)

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


def _straight_stem(length=60, width=3, x0=50) -> np.ndarray:
    """수직 직선 줄기 마스크 — 위쪽(y=0)이 과실 쪽."""
    mask = np.zeros((length, 100), dtype=bool)
    mask[:, x0 - width // 2: x0 + width // 2 + 1] = True
    return mask


def _fruit_at_top(x0=50, r=15, length=60) -> np.ndarray:
    mask = np.zeros((length, 100), dtype=bool)
    yy, xx = np.ogrid[:length, :100]
    mask[((xx - x0) ** 2 + (yy - 0) ** 2) <= r * r] = True
    return mask


def test_thinning() -> None:
    print("\n[세선화] Zhang-Suen 기본 성질")
    thick = np.zeros((30, 30), dtype=bool)
    thick[10:20, 10:20] = True  # 10x10 사각형
    skel = zhang_suen_thin(thick)
    check("세선화 결과가 원본보다 화소가 훨씬 적다",
          0 < skel.sum() < thick.sum(), f"{skel.sum()}/{thick.sum()}")
    check("빈 마스크는 빈 스켈레톤", zhang_suen_thin(np.zeros((10, 10), bool)).sum() == 0)

    line = np.zeros((30, 30), dtype=bool)
    line[15, 5:25] = True  # 이미 1픽셀 두께
    skel_line = zhang_suen_thin(line)
    check("이미 얇은 선은 거의 그대로 유지된다(과도한 침식 없음)",
          skel_line.sum() >= line.sum() * 0.5, f"{skel_line.sum()}/{line.sum()}")


def test_skeleton_points() -> None:
    print("\n[좌표 추출] skeleton_points")
    empty = np.zeros((10, 10), dtype=bool)
    pts = skeleton_points(empty)
    check("빈 스켈레톤은 shape (0,2)", pts.shape == (0, 2), f"{pts.shape}")

    mask = np.zeros((10, 10), dtype=bool)
    mask[3, 4] = True
    pts2 = skeleton_points(mask)
    check("단일 화소 좌표가 (u=4, v=3)으로 나온다(x,y 순서 확인)",
          pts2.shape == (1, 2) and tuple(pts2[0]) == (4.0, 3.0), f"{pts2}")


def test_find_cut_point_straight() -> None:
    print("\n[절단점] 직선 줄기 — 꽃받침에서 정확히 offset만큼 떨어진 점")
    stem = _straight_stem()
    fruit = _fruit_at_top()
    px_per_mm = 1.0  # 계산 단순화를 위해 1px=1mm로 둔 시험 좌표계
    cut = find_cut_point(stem, fruit, px_per_mm=px_per_mm, cut_offset_mm=12.0)
    check("절단점을 찾는다", cut is not None, f"{cut}")
    if cut is not None:
        check("절단점이 꽃받침(y≈0)에서 약 12px 아래(y≈12)",
              abs(cut.v - 12.0) <= 2.0, f"v={cut.v:.1f}")
        check("접선이 줄기 방향(수직, |ty|가 |tx|보다 훨씬 큼)",
              abs(cut.tangent[1]) > abs(cut.tangent[0]) * 3,
              f"tangent={cut.tangent}")
        check("접선이 단위 벡터", abs(np.hypot(*cut.tangent) - 1.0) < 1e-6,
              f"{np.hypot(*cut.tangent):.4f}")


def test_find_cut_point_rejections() -> None:
    print("\n[절단점] 거절 조건 — 없는 것을 만들어 내지 않는가")
    empty = np.zeros((60, 100), dtype=bool)
    fruit = _fruit_at_top()
    check("줄기 마스크가 비어 있으면 None", find_cut_point(empty, fruit, 1.0) is None)

    stem = _straight_stem()
    check("과실 마스크가 비어 있으면 None", find_cut_point(stem, empty, 1.0) is None)

    # 과실과 줄기가 안 맞닿는 경우 (동떨어진 두 마스크)
    far_fruit = np.zeros((60, 100), dtype=bool)
    far_fruit[40:50, 10:20] = True
    check("과실과 줄기가 안 맞닿으면 None(분할 어긋남 방어)",
          find_cut_point(stem, far_fruit, 1.0) is None)

    short_stem = _straight_stem(length=5)  # offset(12mm)보다 훨씬 짧다
    short_fruit = np.zeros((5, 100), dtype=bool)
    yy, xx = np.ogrid[:5, :100]
    short_fruit[((xx - 50) ** 2 + yy ** 2) <= 15 ** 2] = True
    check("줄기가 offset보다 짧으면 None(억지로 만들지 않는다)",
          find_cut_point(short_stem, short_fruit, 1.0, cut_offset_mm=12.0) is None)

    check("px_per_mm <= 0이면 None(무효 축척 거절)",
          find_cut_point(stem, fruit, px_per_mm=0.0) is None and
          find_cut_point(stem, fruit, px_per_mm=-1.0) is None)
    check("cut_offset_mm <= 0이면 None(비물리적 오프셋 거절)",
          find_cut_point(stem, fruit, px_per_mm=1.0, cut_offset_mm=0.0) is None and
          find_cut_point(stem, fruit, px_per_mm=1.0, cut_offset_mm=-5.0) is None)


def test_offset_scales_with_px_per_mm() -> None:
    print("\n[단위] px_per_mm 환산이 실제로 거리에 반영된다")
    stem = _straight_stem(length=80)
    fruit = _fruit_at_top(length=80)
    cut_close = find_cut_point(stem, fruit, px_per_mm=0.5, cut_offset_mm=12.0)
    cut_far = find_cut_point(stem, fruit, px_per_mm=2.0, cut_offset_mm=12.0)
    check("px_per_mm이 클수록(카메라에 가까울수록) 절단점이 화면상 더 멀리(큰 v)",
          cut_close is not None and cut_far is not None and cut_far.v > cut_close.v,
          f"close.v={cut_close.v if cut_close else None} far.v={cut_far.v if cut_far else None}")


def main() -> int:
    test_thinning()
    test_skeleton_points()
    test_find_cut_point_straight()
    test_find_cut_point_rejections()
    test_offset_scales_with_px_per_mm()

    print(f"\n{'='*60}")
    if FAILED:
        print(f"❌ {len(FAILED)}개 실패 / {PASSED + len(FAILED)}개 중")
        for name in FAILED:
            print(f"   - {name}")
        return 1
    print(f"✅ 전부 통과 ({PASSED}개)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
