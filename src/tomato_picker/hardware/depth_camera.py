"""발행된 깊이를 읽어 **픽셀 하나를 3D 점으로** 바꾼다.

카메라를 직접 열지 않는다 — 발행기(`tools/depth_cam.py` = D405,
`tools/astra_cam.py` = Astra)가 vision venv에서 잡고 있고 여기(팔·음성 venv)는
/dev/shm의 공유 파일만 읽는다. pyrealsense2·OpenNI2 의존이 이쪽에 없는 게 요점이다.

⚠ **카메라가 둘이다**(2026-09-01~). 이 클래스는 어느 쪽인지 이름으로만 안다 —
   `DepthView(camera="astra")`. 깊이 단위도(D405 0.1mm / Astra 1mm) 유효 거리도
   (7~50cm / 60~400cm) 다르지만, 그 값들은 전부 **meta.json이 말해준다.**
   그래서 이 파일에는 카메라별 분기가 없다. 상수를 여기 박는 순간, 카메라를
   하나 더 달거나 해상도를 바꾼 날 조용히 틀린 좌표가 나온다.

⚠ **깊이 한 픽셀을 그대로 믿으면 안 된다.** 스테레오 깊이는 픽셀 단위로
   구멍(0)과 튀는 값이 섞인다. 열매 가장자리처럼 대비가 급한 곳이 특히 심하다.
   그래서 항상 **주변 패치의 중앙값**을 쓰고, 유효 픽셀이 모자라면 값을
   돌려주는 대신 **거절한다.**

⚠ **거절이 이 모듈의 핵심 기능이다.** CLAUDE.md의 1번 병("지령은 나갔는데
   아무 일도 안 일어난다")의 비전판은 **"엉뚱한 깊이를 받아 팔이 허공을 집는
   것"**이다. 깊이가 없거나(0), 카메라가 믿을 수 있는 거리 밖이거나, 발행
   프로세스가 죽어 화면이 굳었으면 — 조용히 대충 된 값을 주는 것보다
   "못 하겠다"가 낫다. cartesian.py가 너무 작은 조그를 거절하는 것과 같은 규칙이다.
"""

from __future__ import annotations

import json
import os
import time

import numpy as np

from ..config import (
    D405_MAX_AGE_SEC,
    D405_MIN_VALID_FRAC,
    D405_PATCH_PX,
    DEPTH_CAMERA_DEFAULT,
    DEPTH_CAMERAS,
)
from .handeye import Intrinsics


class DepthError(RuntimeError):
    """이 픽셀의 3D 좌표를 **믿을 수 있게** 줄 수 없다."""


def camera_spec(name: str) -> dict:
    """카메라 이름 → 설정. 모르는 이름은 **거절한다.**

    오타를 기본값으로 삼키면(예: "astro" → d405) 화면에는 Astra라고 적혀 있고
    실제로는 D405를 읽는 상태가 된다. 그 상태를 사람이 알아채는 유일한 단서는
    "왜 이렇게 가깝게 나오지?" 뿐이다 — 차라리 지금 죽는 게 싸다.
    """
    try:
        return DEPTH_CAMERAS[name]
    except KeyError:
        raise ValueError(
            f"모르는 깊이 카메라: {name!r} — 있는 것은 {tuple(DEPTH_CAMERAS)}") from None


class DepthView:
    """발행된 최신 깊이 프레임에 대한 읽기 전용 창.

    상태를 캐시하지 않는다 — 물어볼 때마다 파일을 다시 읽는다. 깊이 프레임은
    814KB뿐이고, 굳은 값을 들고 있다가 그걸로 팔을 움직이는 사고가 훨씬 비싸다.
    """

    def __init__(
        self,
        meta_path: str | None = None,
        depth_path: str | None = None,
        color_path: str | None = None,
        max_age: float = D405_MAX_AGE_SEC,
        camera: str = DEPTH_CAMERA_DEFAULT,
    ) -> None:
        spec = camera_spec(camera)
        self.camera = camera
        self.label = spec["label"]
        self.service = spec["service"]
        self._spec = spec
        self._meta_path = meta_path or spec["meta"]
        self._depth_path = depth_path or spec["depth"]
        self._color_path = color_path or spec["color"]
        self._max_age = max_age

    # ------------------------------------------------------------------
    # 원자료
    # ------------------------------------------------------------------

    def meta(self) -> dict | None:
        try:
            with open(self._meta_path, "rb") as f:
                return json.loads(f.read().decode("utf-8"))
        except (OSError, ValueError):
            return None

    def age(self) -> float | None:
        m = self.meta()
        return None if not m else max(0.0, time.time() - float(m.get("ts", 0)))

    def available(self) -> bool:
        age = self.age()
        return age is not None and age <= self._max_age

    def intrinsics(self) -> Intrinsics:
        m = self._require_meta()
        return Intrinsics.from_dict(m["intrinsics"])

    # ------------------------------------------------------------------
    # 이 카메라가 스스로 말하는 한계
    #
    # ⚠ **meta가 config를 이긴다.** 발행기가 카메라에서 읽어 실어 보낸 값이라
    #   해상도를 바꾸거나 카메라를 바꿔 달아도 따라온다. config의 값은 발행기가
    #   아직 안 떴을 때 화면이 뭐라도 말하게 하는 초기값일 뿐이다.
    # ------------------------------------------------------------------

    def _limit(self, key: str, meta: dict | None = None) -> float:
        m = meta if meta is not None else (self.meta() or {})
        v = m.get(key)
        try:
            return float(v)
        except (TypeError, ValueError):
            return float(self._spec[key])

    @property
    def min_mm(self) -> float:
        return self._limit("min_mm")

    @property
    def max_mm(self) -> float:
        return self._limit("max_mm")

    @property
    def color_aligned(self) -> bool:
        """컬러 픽셀 좌표가 곧 깊이 격자 좌표인가.

        D405는 발행기가 깊이를 **컬러에 정렬**해 넘기므로 참이다. Astra Pro는
        RGB가 별개의 USB 장치이고 공장 D2C 보정값도 없어 거짓이다 — 그 경우
        컬러 화면의 클릭은 깊이 격자에서 **다른 자리**를 가리킨다.
        """
        m = self.meta() or {}
        v = m.get("color_aligned")
        return bool(self._spec["color_aligned"] if v is None else v)

    def require_color_aligned(self) -> None:
        """컬러 픽셀을 3D로 바꾸려는 쪽이 먼저 부른다. 아니면 **거절.**

        조용히 계산해 주면 좌표가 그럴듯하게 나오고 팔이 열매 옆 몇 cm를
        집는다 — 그리고 아무도 원인을 카메라 정렬에서 찾지 않는다.
        """
        if not self.color_aligned:
            raise DepthError(
                f"{self.label}는 컬러와 깊이가 정렬돼 있지 않다 — 컬러 화면의 "
                "픽셀은 깊이 격자의 다른 자리를 가리킨다. **깊이 화면**에서 "
                "클릭하라(그쪽은 정의상 정렬돼 있다).")

    def depth_raw(self) -> np.ndarray:
        """정렬된 uint16 깊이(원시 단위). mm로 바꾸려면 depth_scale_mm을 곱한다."""
        try:
            with open(self._depth_path, "rb") as f:
                return np.load(f)
        except (OSError, ValueError) as e:
            raise DepthError(f"깊이 프레임을 읽지 못했다: {e}") from e

    def color_jpeg(self) -> bytes:
        try:
            with open(self._color_path, "rb") as f:
                return f.read()
        except OSError as e:
            raise DepthError(f"컬러 프레임을 읽지 못했다: {e}") from e

    def color_bgr(self) -> np.ndarray:
        import cv2  # 색검출이 필요할 때만 — 이 모듈의 핵심 경로는 cv2가 없어도 돈다

        img = cv2.imdecode(np.frombuffer(self.color_jpeg(), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise DepthError("컬러 프레임을 디코드하지 못했다.")
        return img

    # ------------------------------------------------------------------
    # 픽셀 → 3D
    # ------------------------------------------------------------------

    def depth_mm_at(self, u: float, v: float, patch: int = D405_PATCH_PX) -> float:
        """(u,v) 주변 패치의 **중앙값** 깊이(mm). 못 믿으면 DepthError.

        중앙값을 쓰는 이유 — 평균은 구멍(0)과 튀는 값 하나에 통째로 끌려간다.
        중앙값은 절반이 성할 때까지 버틴다.
        """
        m = self._require_meta()
        depth = self.depth_raw()
        h, w = depth.shape
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            raise DepthError(f"픽셀 ({ui},{vi})이 프레임({w}x{h}) 밖이다.")

        r = max(0, int(patch) // 2)
        win = depth[max(0, vi - r): vi + r + 1, max(0, ui - r): ui + r + 1]
        valid = win[win > 0]
        frac = valid.size / max(1, win.size)
        lo, hi = self._limit("min_mm", m), self._limit("max_mm", m)
        if frac < D405_MIN_VALID_FRAC:
            raise DepthError(
                f"그 자리의 깊이가 비어 있다 — 주변 {win.size}픽셀 중 {valid.size}개만 "
                f"유효({frac:.0%}, 최소 {D405_MIN_VALID_FRAC:.0%}). 반사·그림자·"
                f"너무 가까움({lo:.0f}mm 미만) 중 하나다. 조금 다른 지점을 찍거나 "
                "조명을 보라.")

        mm = float(np.median(valid)) * float(m["depth_scale_mm"])
        if mm < lo:
            raise DepthError(
                f"{mm:.0f}mm는 너무 가깝다 — {self.label}는 {lo:.0f}mm 아래를 "
                "측정하지 못한다(그 값은 잡음이다).")
        if mm > hi:
            raise DepthError(
                f"{mm:.0f}mm는 너무 멀다 — {self.label}의 유효 거리는 "
                f"{lo:.0f}~{hi:.0f}mm다. 이 밖은 오차가 열매 지름을 넘어 "
                "집기에 못 쓴다(계산은 되지만 그래서 더 위험하다).")
        return mm

    def point_at(self, u: float, v: float,
                 patch: int = D405_PATCH_PX) -> tuple[float, float, float]:
        """(u,v) → 카메라 좌표 3D (mm). 못 믿으면 DepthError."""
        mm = self.depth_mm_at(u, v, patch)
        return self.intrinsics().deproject(float(u), float(v), mm)

    # ------------------------------------------------------------------
    # 화면용
    # ------------------------------------------------------------------

    def status(self) -> dict:
        """대시보드가 그대로 그릴 수 있는 상태. **왜 안 되는지**를 말해준다."""
        m = self.meta()
        if not m:
            return {"ok": False, "camera": self.camera, "label": self.label,
                    "why": f"{self.label} 깊이 발행기가 돌지 않는다 — "
                           f"`sudo systemctl start {self.service}` 후 다시 보라."}
        age = max(0.0, time.time() - float(m.get("ts", 0)))
        if age > self._max_age:
            return {"ok": False, "camera": self.camera, "label": self.label,
                    "age": round(age, 1),
                    "why": f"{self.label} 화면이 {age:.0f}초째 멈춰 있다 — 발행기가 "
                           f"죽었거나 USB가 빠졌다. `systemctl status {self.service}`."}
        near = float(m.get("near_frac", 0.0))
        lo, hi = self._limit("min_mm", m), self._limit("max_mm", m)
        out = {
            "ok": True,
            "camera": self.camera,
            "label": self.label,
            "age": round(age, 1),
            "serial": m.get("serial"),
            "size": f"{m.get('width')}x{m.get('height')}",
            "valid_frac": m.get("valid_frac"),
            "near_frac": near,
            "median_mm": m.get("median_mm"),
            "depth_scale_mm": m.get("depth_scale_mm"),
            "min_mm": lo,
            "max_mm": hi,
            "color_aligned": self.color_aligned,
        }
        band = f"{lo / 10:.0f}~{hi / 10:.0f}cm"
        # ⚠ 이 경고가 2026-08-28 D405 첫 연결에서 실제로 났던 상황이다 — 삼각대가
        #   장면에서 2m 떨어져 있어 유효범위 안이 0%였다. 2026-09-01 Astra는
        #   반대 이유로 같은 0%였다(벽에서 20cm — **너무 가까워서**). 그래서
        #   "붙여라"고 단정하지 않고 중앙값을 함께 보여준다.
        med = m.get("median_mm")
        if near < 0.02:
            out["warn"] = (
                f"보이는 것의 {near:.0%}만 {self.label} 유효범위({band}) 안이다 "
                f"(중앙값 {med}mm). 이대로는 보정을 해도 열매를 못 집는다 — "
                "카메라와 무대의 거리를 그 범위로 맞춰라.")
        elif near < 0.15:
            out["warn"] = (f"유효범위({band}) 안이 {near:.0%}뿐이다 — 거리를 맞추면 "
                           "정확도가 눈에 띄게 오른다.")
        if not out["color_aligned"]:
            out["note"] = ("컬러가 깊이와 정렬돼 있지 않다 — 클릭은 **깊이 화면**에서 하라.")
        return out

    # ------------------------------------------------------------------

    def _require_meta(self) -> dict:
        m = self.meta()
        if not m:
            raise DepthError(
                f"{self.label} 깊이 발행기가 돌지 않는다 — 젯슨에서 "
                f"`sudo systemctl start {self.service}`.")
        age = time.time() - float(m.get("ts", 0))
        if age > self._max_age:
            raise DepthError(
                f"깊이 프레임이 {age:.0f}초 전 것이다(최대 {self._max_age:.0f}초). "
                "발행기가 멈췄다 — 굳은 화면으로 팔을 움직이지 않겠다.")
        return m


def shm_present(camera: str = DEPTH_CAMERA_DEFAULT) -> bool:
    """파일만 보고 그 카메라를 쓸 수 있는지 — 서비스 기동 전 판단용."""
    return os.path.exists(camera_spec(camera)["meta"])


def available_cameras() -> list[str]:
    """지금 실제로 발행 중인 카메라들. 화면이 고를 수 있는 목록이다."""
    return [name for name in DEPTH_CAMERAS if shm_present(name)]
