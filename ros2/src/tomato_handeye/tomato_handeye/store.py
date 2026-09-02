"""보정을 **한 벌로** 유지하고, 그것을 TF로 옮긴다. numpy만 쓴다(rclpy 없음).

────────────────────────────────────────────────────────────────────────
① 보정 파일은 하나다 — `~/arm_eye.json`

처음에는 ROS 쪽이 `~/tomato_handeye.json`을 따로 썼다. 그건 **같은 카메라의
보정이 두 벌**이라는 뜻이고, 이 저장소가 반복해서 비싸게 배운 실패 그대로다:
두 파일이 갈라지는 순간 "보정은 됐다는데 팔이 옆으로 간다"가 시작되고, 어느
쪽이 진짜인지 아무도 모른다.

그래서 저장·복원은 레거시 [`eye.EyeConfig`](../../../../src/tomato_picker/hardware/eye.py)를
**그대로 호출한다.** 형식을 여기서 다시 구현하지도 않는다 — 구현이 둘이면
결국 값도 둘이 된다. 파일이 정본이고, 두 계통이 같은 파일을 본다.

⚠ 그러면 레거시 대시보드에서 보정을 다시 잡아도 ROS가 그걸 그대로 쓴다.
   그게 의도다. 팔도 카메라도 하나뿐이므로 보정도 하나여야 한다.

② `retarget()` — 왜 광학 프레임에 직접 안 붙이는가

손-눈 보정이 푸는 것은 `arm_base → 컬러 광학 프레임`이다(그 좌표계에서 점을
역투영했으니까). 그런데 그대로 static TF로 쏘면 realsense2_camera가 이미
발행하는 `camera_link → camera_color_optical_frame`과 겹쳐 **같은 프레임에
부모가 둘**이 된다. tf2는 그때부터 조회가 오락가락한다.

    arm_base → camera_link = (arm_base → optical) ∘ (camera_link → optical)⁻¹

보정 결과 자체는 안 바뀐다. 어느 마디에 붙일지만 바꾸는 것이다.
"""

from __future__ import annotations

import math

import numpy as np


class CalibrationStoreUnavailable(RuntimeError):
    """보정 저장소(`eye.EyeConfig`)를 못 불러왔다. **대체 구현으로 넘어가지 않는다.**"""


def config(path: str = ""):
    """`eye.EyeConfig` 하나를 만들어 준다. 경로를 비우면 레거시 기본값을 그대로 쓴다.

    lazy import인 이유 — 이 모듈은 `ros_selfcheck.py`가 numpy만으로 읽을 수 있어야
    하고(TF 수학 검증), `eye.py`는 그보다 무거운 사슬(config·cartesian 상수)을 끈다.
    """
    try:
        from tomato_picker.hardware.eye import EyeConfig
    except ImportError as exc:  # pragma: no cover - 배치 실수를 즉시 드러낸다
        raise CalibrationStoreUnavailable(
            f"tomato_picker.hardware.eye를 import하지 못했다: {exc}\n"
            "보정 파일(~/arm_eye.json)의 정본은 그 모듈이다 — ROS 쪽에서 형식을 "
            "다시 구현하지 않는다(두 벌이 되는 순간 어느 쪽이 진짜인지 알 수 없다). "
            "저장소 src/를 PYTHONPATH에 넣어라."
        ) from exc
    return EyeConfig(path) if path else EyeConfig()


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
