"""마스크 + 깊이 → **열매 하나의 3D 좌표**. numpy만 쓴다(cv2도 rclpy도 없다).

기존 스택은 열매를 픽셀 `(x, y)`와 면적으로 알았다. 면적은 "가까울수록 큼"이라는
**순서 정보**일 뿐이라 거리로 쓸 수 없다. D405가 화소마다 거리를 주므로 이제
좌표가 나온다 — 다만 그 거리를 **그대로 믿으면 안 된다.** 이 파일의 절반은 그
이야기다.

────────────────────────────────────────────────────────────────────────
깊이를 왜 그냥 못 쓰는가 (D405로 열매를 볼 때 실제로 일어나는 일)

① **구멍(0값)** — 빨갛고 매끈한 표면은 IR 패턴을 잘 안 돌려준다. 열매 한가운데가
   통째로 0인 경우가 흔하다. 0을 거리로 쓰면 좌표가 **카메라 원점**이 된다.
② **가장자리가 배경을 본다** — 마스크 경계의 화소는 열매와 배경에 걸쳐 있어
   깊이가 뒤쪽(잎·벽)으로 튄다. 평균을 내면 열매가 실제보다 **멀리** 잡힌다.
③ **잎이 앞을 가린다** — 마스크 안에 앞쪽 잎이 조금 섞이면 그 화소는 훨씬 가깝다.

그래서 여기서는 (a) 0을 버리고, (b) 중심 쪽 화소만 보고, (c) 평균이 아니라
**중앙값**을 쓰고, (d) 퍼짐이 크면 **그 열매를 버린다.**

⚠ 마지막 (d)가 이 파일에서 제일 중요하다. 이 로봇의 1번 병("지령은 나갔는데
   아무 일도 안 일어난다")의 인식판은 **"좌표는 나왔는데 거기 열매가 없다"** 이고,
   그건 못 믿을 깊이를 좌표로 바꿔 준 결과다. 못 믿겠으면 **안 준다.**
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 마스크 화소 중 유효 깊이가 이 비율보다 적으면 그 열매는 버린다.
MIN_DEPTH_RATIO = 0.25
# 중심에서 반지름의 이 비율 안쪽 화소만 본다 (가장자리는 배경을 본다).
CORE_RATIO = 0.6
# 유효 깊이의 퍼짐(MAD 기반 표준편차 환산)이 이보다 크면 잎·배경이 섞인 것.
MAX_SPREAD_MM = 25.0
# 중앙값 ±MAX_SPREAD_MM 안에 있어야 하는 화소의 최소 비율.
#
# ⚠ 이 검사가 왜 따로 필요한가 — **중앙값과 MAD만으로는 두 층을 못 본다.**
#    잎이 열매의 60%를 가리면 중앙값이 잎이 되고, 그러면 MAD는 0이 된다
#    (편차의 과반이 0이므로). 즉 "퍼짐 0mm, 아주 깨끗함"이라고 말하면서
#    **잎의 거리를 열매의 거리로 준다.** 팔은 그 좌표를 믿고 잎을 집는다.
#    비율로 보면 어느 층이 이기든 두 층이라는 사실이 드러난다.
MIN_INLIER_RATIO = 0.7
# 이보다 가깝거나 먼 값은 D405의 유효 범위 밖 — 구멍과 같이 버린다.
# (D405 권장 작동 범위 7cm ~ 50cm. 넉넉히 잡되 무한대는 안 받는다.)
MIN_DEPTH_MM = 60.0
MAX_DEPTH_MM = 900.0


@dataclass(frozen=True)
class Blob:
    """검출기가 찾은 마스크 한 덩이 — **아직 3D가 아니다.**"""

    u: float           # 중심 픽셀 x
    v: float           # 중심 픽셀 y
    radius_px: float   # 등가 원 반지름
    pixels: int        # 마스크 화소 수
    ripe: bool
    confidence: float = 1.0


@dataclass(frozen=True)
class Reading:
    """한 덩이를 3D로 읽은 결과 + **믿어도 되는지**.

    `ok=False`면 `point_mm`은 None이다 — 못 믿는 값을 채워 넣지 않는다.
    """

    blob: Blob
    ok: bool
    reason: str = ""
    point_mm: tuple[float, float, float] | None = None
    radius_mm: float = 0.0
    depth_mm: float = 0.0
    depth_pixels: int = 0
    spread_mm: float = 0.0


def core_mask(shape: tuple[int, int], blob: Blob,
              ratio: float = CORE_RATIO) -> np.ndarray:
    """덩이 중심에서 반지름×ratio 안쪽만 True. **가장자리를 버리는 장치다.**

    마스크 자체를 침식(erode)하지 않고 원으로 자르는 이유 — 침식은 cv2가 필요하고,
    가늘고 긴 오검출에서는 아무것도 안 남아 "구멍"과 구분이 안 된다. 원으로
    자르면 남는 화소 수가 예측 가능하다.
    """
    h, w = shape
    yy, xx = np.ogrid[:h, :w]
    r = max(1.0, blob.radius_px * ratio)
    return ((xx - blob.u) ** 2 + (yy - blob.v) ** 2) <= r * r


def read_blob(intr, depth_mm: np.ndarray, blob: Blob,
              mask: np.ndarray | None = None) -> Reading:
    """덩이 하나 → 카메라 좌표 3D(mm). intr은 handeye.Intrinsics.

    `mask`를 주면 그것과 중심 원의 교집합을 본다(색 마스크가 이미 있을 때).
    안 주면 중심 원만 본다.
    """
    core = core_mask(depth_mm.shape, blob)
    if mask is not None:
        core &= mask.astype(bool)

    candidates = int(core.sum())
    if candidates == 0:
        return Reading(blob, False, "중심 영역에 화소가 없다 (덩이가 너무 작다)")

    z = depth_mm[core].astype(float)
    valid = z[(z >= MIN_DEPTH_MM) & (z <= MAX_DEPTH_MM)]
    ratio = len(valid) / candidates

    if ratio < MIN_DEPTH_RATIO:
        return Reading(
            blob, False,
            f"유효 깊이가 {len(valid)}/{candidates}화소({ratio:.0%})뿐 — "
            f"기준 {MIN_DEPTH_RATIO:.0%}. 빨갛고 매끈한 표면은 IR을 안 돌려준다. "
            "조명을 낮추거나 각도를 바꿔라",
            depth_pixels=len(valid))

    median = float(np.median(valid))
    # MAD → 표준편차 환산(정규분포 가정에서 1.4826배). 평균·표준편차를 쓰면
    # 잎 하나가 통계를 통째로 끌고 간다.
    spread = float(np.median(np.abs(valid - median)) * 1.4826)
    inlier_ratio = float(np.mean(np.abs(valid - median) <= MAX_SPREAD_MM))

    if spread > MAX_SPREAD_MM:
        return Reading(
            blob, False,
            f"깊이 퍼짐 {spread:.0f}mm > {MAX_SPREAD_MM:.0f}mm — 마스크에 잎이나 "
            "배경이 섞였다. 이 좌표로 가면 열매가 없는 곳으로 간다",
            depth_mm=median, depth_pixels=len(valid), spread_mm=spread)

    if inlier_ratio < MIN_INLIER_RATIO:
        return Reading(
            blob, False,
            f"깊이가 한 층이 아니다 — 중앙값 {median:.0f}mm 근처에 "
            f"{inlier_ratio:.0%}뿐(기준 {MIN_INLIER_RATIO:.0%}). 앞에 잎이 걸쳤거나 "
            "열매 두 개가 겹쳐 보인다. 어느 쪽 거리인지 알 수 없으므로 안 준다",
            depth_mm=median, depth_pixels=len(valid), spread_mm=spread)

    point = intr.deproject(blob.u, blob.v, median)
    # 반지름은 **깊이가 정해진 뒤에야** 길이가 된다. 픽셀 반지름 하나만으로는
    # 크기를 알 수 없다(가까운 방울토마토와 먼 대추토마토가 같아 보인다).
    radius = blob.radius_px * median / max(1e-6, (intr.fx + intr.fy) / 2.0)

    return Reading(blob, True, "", point_mm=point, radius_mm=radius,
                   depth_mm=median, depth_pixels=len(valid), spread_mm=spread)


def read_all(intr, depth_mm: np.ndarray, blobs, masks=None) -> list[Reading]:
    """여러 덩이를 한 번에. **버려진 것도 돌려준다** — 왜 안 잡혔는지 화면에 띄우려고.

    "6개 중 2개만 잡힌다"가 조명 문제가 아니라 해상도 문제였던 적이 있다
    (2026-08-13). 버려진 이유가 안 보이면 그런 진단을 또 며칠 걸려서 한다.
    """
    masks = masks or [None] * len(blobs)
    return [read_blob(intr, depth_mm, b, m) for b, m in zip(blobs, masks)]
