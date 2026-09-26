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
    YoloDetection, detections_to_blobs, mask_to_blob,
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


def main() -> int:
    test_mask_to_blob()
    test_detections_to_blobs()
    test_empty_input()

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
