"""YOLO 세그멘테이션 결과 → `Blob` + 마스크. rclpy도 cv2도 여기선 없다.

`detect_node.py`의 `_blobs()`가 지금은 HSV 마스킹을 부른다. YOLO로 갈아끼울 때
이 파일의 `to_blobs()`만 대신 부르면 나머지(3D 역투영·거절 로직)는 안 바뀐다 —
`detect_node.py` 헤더에 적힌 그 지점이다.

torch/ultralytics는 선택적 의존성이라 모듈 최상단에서 import하지 않는다
(자체검증은 이 파일의 순수 변환 로직만 보고, 모델이 없는 PC에서도 돈다).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .fruit3d import Blob

# ripe/unripe 클래스 이름 매핑 — 파인튜닝된 tomato-seg.pt가 이 이름을 쓴다고 가정.
# 학습 라벨이 다르면 여기 하나만 바꾼다.
RIPE_CLASS_NAMES = {"ripe", "red", "tomato_ripe"}
UNRIPE_CLASS_NAMES = {"unripe", "green", "tomato_unripe"}


@dataclass(frozen=True)
class YoloDetection:
    """YOLO 원시 출력 한 개 — 이미 화면 좌표계, 아직 3D 아님."""

    mask: np.ndarray       # bool, 프레임과 같은 shape
    confidence: float
    class_name: str


def mask_to_blob(mask: np.ndarray, confidence: float, ripe: bool) -> Blob | None:
    """이진 마스크 하나 → Blob. 화소가 없으면 None(빈 마스크는 검출이 아니다)."""
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None
    u = float(xs.mean())
    v = float(ys.mean())
    pixels = int(len(xs))
    # 등가 원 반지름 — fruit3d.read_blob()이 반지름*깊이로 물리 크기를 낸다.
    radius_px = float(np.sqrt(pixels / np.pi))
    return Blob(u=u, v=v, radius_px=radius_px, pixels=pixels,
                ripe=ripe, confidence=float(confidence))


def detections_to_blobs(
    detections: list[YoloDetection],
    min_pixels: int = 400,
    detect_unripe: bool = False,
) -> tuple[list[Blob], list[np.ndarray]]:
    """YOLO 검출 리스트 → (Blob 리스트, 마스크 리스트). detect_node._blobs()와 같은 반환형."""
    blobs: list[Blob] = []
    masks: list[np.ndarray] = []
    for det in detections:
        ripe = det.class_name in RIPE_CLASS_NAMES
        unripe = det.class_name in UNRIPE_CLASS_NAMES
        if not ripe and not unripe:
            continue  # 모르는 클래스는 조용히 버리지 않고 무시 — 열매가 아닌 클래스일 수 있다
        if unripe and not detect_unripe:
            continue
        pixels = int(det.mask.sum())
        if pixels < min_pixels:
            continue
        blob = mask_to_blob(det.mask, det.confidence, ripe)
        if blob is None:
            continue
        blobs.append(blob)
        masks.append(det.mask)
    return blobs, masks


def run_yolo_seg(model: Any, bgr: np.ndarray, conf: float = 0.6) -> list[YoloDetection]:
    """Ultralytics YOLO 모델로 실제 추론 — model은 `ultralytics.YOLO` 인스턴스.

    이 함수만 torch/ultralytics를 만진다. 자체검증은 `detections_to_blobs()`를
    가짜 `YoloDetection`으로 직접 시험하므로 이 함수를 부르지 않는다.
    """
    results = model.predict(source=bgr, conf=conf, verbose=False)
    out: list[YoloDetection] = []
    for r in results:
        if r.masks is None:
            continue
        h, w = bgr.shape[:2]
        names = r.names
        for mask_xy, cls_idx, box_conf in zip(
                r.masks.xy, r.boxes.cls.tolist(), r.boxes.conf.tolist()):
            import cv2  # 지연 import — 마스크 폴리곤을 래스터화할 때만 필요
            raster = np.zeros((h, w), dtype=np.uint8)
            pts = np.int32([mask_xy])
            cv2.fillPoly(raster, pts, 1)
            out.append(YoloDetection(
                mask=raster.astype(bool),
                confidence=float(box_conf),
                class_name=str(names[int(cls_idx)])))
    return out
