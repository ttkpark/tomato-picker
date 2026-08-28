"""보정 결과를 파일로 남기고 TF로 되살린다. numpy만 쓴다(rclpy 없음).

────────────────────────────────────────────────────────────────────────
⚠ 이 파일에서 제일 중요한 함수는 `retarget()`이다. 왜 필요한지부터.

손-눈 보정이 푸는 것은 **`arm_base → 컬러 광학 프레임`** 이다(그 좌표계에서
점을 역투영했으니까). 그런데 그 변환을 그대로 static TF로 쏘면 안 된다 —
realsense2_camera 드라이버가 이미 `camera_link → camera_color_optical_frame`을
발행하고 있어서, 같은 프레임에 **부모가 둘**이 된다. tf2는 그 순간부터
"TF_OLD_DATA / multiple parents" 를 뿜으며 조회가 오락가락한다.

그래서 우리가 쏘는 것은 한 칸 위다:

    arm_base → camera_link  =  (arm_base → color_optical) ∘ (camera_link → color_optical)⁻¹

드라이버가 발행하는 `camera_link → color_optical`을 TF에서 읽어 합성한다.
**보정 결과 자체는 안 바뀐다** — 어느 마디에 붙일지만 바꾸는 것이다.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import time

import numpy as np

DEFAULT_PATH = "~/tomato_handeye.json"


# ----------------------------------------------------------------------
# 회전행렬 ↔ 쿼터니언
# ----------------------------------------------------------------------

def quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 회전행렬 → (x, y, z, w).

    trace가 음수일 때 가장 큰 대각 성분을 기준으로 갈라 푸는 표준 방법을 쓴다 —
    한 갈래로만 풀면 180° 근처에서 0으로 나눈다.
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    t = float(np.trace(R))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


def rotation(q: tuple[float, float, float, float]) -> np.ndarray:
    """(x, y, z, w) → 3x3. 크기가 1이 아니어도 정규화해서 받는다."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# ----------------------------------------------------------------------
# 붙일 마디 바꾸기
# ----------------------------------------------------------------------

def retarget(parent_to_optical, link_to_optical):
    """(부모→광학) 와 (카메라링크→광학) → (부모→카메라링크).

    둘 다 handeye.Rigid. 단위는 서로 같기만 하면 된다(여기서는 mm).
    """
    return parent_to_optical.compose(link_to_optical.inverse())


# ----------------------------------------------------------------------
# 파일
# ----------------------------------------------------------------------

def save(fit, mount: str, parent_frame: str, camera_frame: str,
         path: str = DEFAULT_PATH, note: str = "") -> str:
    """보정을 파일로. **잔차를 같이 적는다** — 나중에 "이 값 믿어도 되나"의 답이다.

    원자적으로 쓴다(임시파일 → replace). 저장 중에 전원이 나가도 반쪽짜리
    보정이 남지 않는다 — 반쪽 보정은 없는 보정보다 나쁘다.
    """
    full = os.path.expanduser(path)
    payload = {
        "version": 1,
        "mount": mount,
        "parent_frame": parent_frame,
        "camera_frame": camera_frame,
        "units": "mm",
        "transform": fit.transform.as_dict(),
        "rms_mm": round(float(fit.rms_mm), 3),
        "max_mm": round(float(fit.max_mm), 3),
        "samples": int(fit.samples),
        "marker_base_mm": list(fit.marker_base) if fit.marker_base else None,
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "note": note,
    }
    directory = os.path.dirname(full) or "."
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=directory, prefix=".handeye-", suffix=".json")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        os.replace(tmp, full)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    return full


def load(path: str = DEFAULT_PATH) -> dict | None:
    """없으면 None. **없는 것과 깨진 것을 구분하지 않는다** — 둘 다 "보정 안 됨"이고,
    그 상태에서 TF를 안 쏘는 것이 옳은 동작이다(조회가 실패해서 사실이 드러난다)."""
    try:
        with open(os.path.expanduser(path), encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) and data.get("transform") else None
