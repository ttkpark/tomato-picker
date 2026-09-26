#!/usr/bin/env python3
"""YOLO seg 어댑터 자체검증 — 모델도 카메라도 없이 개발 PC에서 돈다.

    python ros2/tools/yolo_seg_check.py

`tomato_perception/yolo_seg.py`의 `detections_to_blobs()`는 순수 변환이라
가짜 YoloDetection으로 전부 시험할 수 있다. 실제 torch/ultralytics 추론
(`run_yolo_seg()`)은 이 파일이 시험하지 않는다 — 모델과 GPU가 있는 곳
(젯슨 `.venvs/vision`)에서 정적 이미지로 별도 확인한다.
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
sys.path.insert(0, os.path.join(os.path.dirname(ROS2), "src"))
sys.path.insert(0, os.path.join(SRC, "tomato_perception"))

from tomato_perception.yolo_seg import (  # noqa: E402
    STEM_CLASS_NAMES, YoloDetection, detections_to_blobs, extract_stem_masks, mask_to_blob,
)
from tomato_perception.stem_cut import find_cut_point  # noqa: E402


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


def _disk_mask(size=100, cx=50, cy=50, r=20) -> np.ndarray:
    yy, xx = np.ogrid[:size, :size]
    return ((xx - cx) ** 2 + (yy - cy) ** 2) <= r * r


def test_mask_to_blob() -> None:
    print("\n[마스크→Blob] 단일 변환")
    mask = _disk_mask()
    blob = mask_to_blob(mask, confidence=0.9, ripe=True)
    check("중심이 원 중심과 일치", blob is not None and abs(blob.u - 50) < 1.0
          and abs(blob.v - 50) < 1.0, f"{blob}")
    check("반지름이 대략 20px", blob is not None and abs(blob.radius_px - 20) < 1.0,
          f"{blob.radius_px if blob else None}")
    check("confidence가 그대로 전달된다", blob is not None and blob.confidence == 0.9)

    empty = np.zeros((10, 10), dtype=bool)
    check("빈 마스크는 None(검출 아님)", mask_to_blob(empty, 0.9, True) is None)


def test_detections_to_blobs() -> None:
    print("\n[검출→Blob] 클래스 필터·최소화소·미익음 옵션")
    ripe_det = YoloDetection(mask=_disk_mask(cx=30, cy=30), confidence=0.8,
                             class_name="ripe")
    unripe_det = YoloDetection(mask=_disk_mask(cx=70, cy=70), confidence=0.7,
                               class_name="unripe")
    unknown_det = YoloDetection(mask=_disk_mask(cx=50, cy=10, r=5), confidence=0.5,
                                class_name="leaf")

    blobs, masks = detections_to_blobs([ripe_det, unripe_det, unknown_det],
                                       min_pixels=10, detect_unripe=False)
    check("미익음 기본 비활성 시 익은 것만 남는다", len(blobs) == 1 and blobs[0].ripe,
          f"{len(blobs)}개")
    check("모르는 클래스는 조용히 무시된다(열매로 오인 안 함)",
          all(b.ripe for b in blobs))

    blobs2, _ = detections_to_blobs([ripe_det, unripe_det], min_pixels=10,
                                     detect_unripe=True)
    check("detect_unripe=True면 둘 다 나온다",
          len(blobs2) == 2 and {b.ripe for b in blobs2} == {True, False},
          f"{[b.ripe for b in blobs2]}")

    tiny = YoloDetection(mask=_disk_mask(r=2), confidence=0.9, class_name="ripe")
    blobs3, _ = detections_to_blobs([tiny], min_pixels=400)
    check("min_pixels 미만은 버려진다(오검출 잡음 방어)", len(blobs3) == 0,
          f"{len(blobs3)}개 남음")

    blobs4, masks4 = detections_to_blobs([ripe_det], min_pixels=10)
    check("반환된 마스크가 검출 마스크와 같은 화소 수",
          len(masks4) == 1 and masks4[0].sum() == ripe_det.mask.sum())


def test_empty_input() -> None:
    print("\n[경계] 빈 입력")
    blobs, masks = detections_to_blobs([], min_pixels=10)
    check("검출 0개면 빈 리스트 반환(예외 아님)", blobs == [] and masks == [])
    stems = extract_stem_masks([], min_pixels=10)
    check("줄기 검출 0개면 빈 리스트 반환(예외 아님)", stems == [])


def test_extract_stem_masks_and_stem_cut() -> None:
    print("\n[줄기 세그멘테이션 및 stem_cut 연동] (T89)")
    check("STEM_CLASS_NAMES에 stem/peduncle/calyx 포함",
          {"stem", "peduncle"}.issubset(STEM_CLASS_NAMES), f"{STEM_CLASS_NAMES}")

    # 가짜 줄기 마스크(60x100 영역, 중심 x=50, 폭 3, y=0..59)
    stem_mask = np.zeros((60, 100), dtype=bool)
    stem_mask[:, 49:52] = True  # 60 * 3 = 180 px

    # 가짜 과실 마스크(y=0 기준 상단 원)
    fruit_mask = np.zeros((60, 100), dtype=bool)
    yy, xx = np.ogrid[:60, :100]
    fruit_mask[((xx - 50) ** 2 + (yy - 0) ** 2) <= 15 ** 2] = True

    stem_det = YoloDetection(mask=stem_mask, confidence=0.85, class_name="peduncle")
    fruit_det = YoloDetection(mask=fruit_mask, confidence=0.92, class_name="ripe")
    leaf_det = YoloDetection(mask=np.zeros((60, 100), dtype=bool), confidence=0.5, class_name="leaf")

    detections = [stem_det, fruit_det, leaf_det]

    stem_masks = extract_stem_masks(detections, min_pixels=50)
    check("줄기 마스크가 정확히 1개 추출된다", len(stem_masks) == 1, f"{len(stem_masks)}개")
    check("추출된 줄기 마스크 화소 수 일치", stem_masks[0].sum() == stem_mask.sum())

    # min_pixels 필터 검증
    tiny_stem = YoloDetection(mask=np.zeros((60, 100), dtype=bool), confidence=0.8, class_name="stem")
    tiny_stem.mask[10:15, 50] = True  # 5 px
    stems_filtered = extract_stem_masks([tiny_stem], min_pixels=50)
    check("min_pixels 미만 줄기 마스크는 버려진다", len(stems_filtered) == 0)

    # stem_cut.find_cut_point 연동 파이프라인 검증
    blobs, fruit_masks = detections_to_blobs(detections, min_pixels=50)
    check("과실 블롭 및 마스크가 1개 추출된다", len(fruit_masks) == 1 and len(blobs) == 1)

    cut_point = find_cut_point(
        stem_mask=stem_masks[0],
        fruit_mask=fruit_masks[0],
        px_per_mm=1.0,
        cut_offset_mm=12.0,
    )
    check("추출된 마스크로부터 stem_cut 절단점이 정상 산출된다",
          cut_point is not None and abs(cut_point.v - 12.0) <= 2.0,
          f"cut_point={cut_point}")


def main() -> int:
    test_mask_to_blob()
    test_detections_to_blobs()
    test_empty_input()
    test_extract_stem_masks_and_stem_cut()

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
