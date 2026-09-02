"""**보정된 눈** — 카메라가 본 픽셀을 팔이 갈 수 있는 좌표로 바꾼다.

세 조각이 여기서 만난다:

    depth_camera.DepthView   픽셀 → 카메라 좌표 3D   (D405가 재 준 거리)
    handeye.Rigid            카메라 좌표 → 팔 좌표    (보정으로 푼 변환)
    cartesian.CartesianArm   팔 좌표 → 관절           (기구학)

그래서 화면의 토마토를 클릭하면 팔이 그 앞에 선다. 지금까지는 사람이
"5mm 앞으로"를 눌러가며 찾아야 했다.

────────────────────────────────────────────────────────────────────────
보정 절차 (fixed = 삼각대·차체 고정)

  전제: **기구학 영점이 먼저다.** 영점이 없으면 FK가 말하는 집게 좌표가
        아무 뜻이 없고, 그걸 짝으로 쓰는 보정도 통째로 무의미하다.

  ① 팔을 카메라가 잘 보는 자리로 옮긴다 (프리셋이든 조그든)
  ② 화면에서 **집게가 무는 지점(TCP)**을 클릭한다 → 표본 한 개
     (그때 카메라가 본 3D 점과, FK가 아는 집게 좌표가 한 쌍으로 묶인다)
  ③ 팔을 **멀찍이** 옮겨가며 ①②를 5번 이상 반복
     ⚠ 조금씩만 옮기면 표본이 한 뭉치에 몰려 회전이 안 정해진다.
       좌우·상하·앞뒤로 크게 흩어야 한다(코드가 한 직선이면 거절한다).
  ④ [보정 풀기] → 잔차(RMS)를 본다. 15mm를 넘으면 쓰지 마라.

on_arm(손목 장착)은 반대다 — 마커를 한 자리에 고정해 두고, 팔 자세를 크게
바꿔가며 **그 마커**를 8번 이상 클릭한다. 자세히는 docs/depth-camera.md.

⚠ **`fixed` 보정은 베이스가 주행하면 그 순간 거짓이 된다.** 삼각대는 지면에
   고정이지 로봇에 고정이 아니다. 코드는 로봇이 굴러간 걸 알 방법이 없고,
   팔은 아무 일 없다는 듯 엉뚱한 데로 간다. 주행 후에는 다시 잡아라.
   (이건 버그가 아니라 이 장착 방식의 성질이다. 손목/차체 장착으로 옮기면
    사라진다 — 그때는 mount를 바꾸고 다시 보정하면 된다.)
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import threading
import time

import numpy as np

from ..config import (
    ARM_CART_MAX_STEP_MM,
    ARM_EYE_FILE,
    DEPTH_CAMERA_DEFAULT,
    ARM_EYE_MIN_SAMPLES_FIXED,
    ARM_EYE_MIN_SAMPLES_ON_ARM,
    ARM_EYE_MOUNT,
    ARM_EYE_PICK_PITCH_DEG,
    ARM_EYE_STANDOFF_MM,
)
from .depth_camera import DepthError, DepthView
from .handeye import (
    GOOD_RMS_MM,
    CalibrationError,
    Fit,
    Intrinsics,
    Rigid,
    solve_fixed,
    solve_on_arm,
    tool_frame,
)

MOUNTS = ("fixed", "on_arm")
MOUNT_LABEL = {"fixed": "삼각대·차체 고정", "on_arm": "손목 장착"}


def _resolve(src):
    """CartesianArm을 **매번 다시 꺼낸다** — 호출가능이면 호출해서.

    왜 핸들을 붙들지 않나 — voice_mode는 USB가 빠졌다 돌아오면 팔 객체를
    통째로 새로 만들어 hardware["arm"]에 갈아끼운다(_retry_hardware_forever).
    여기서 처음 받은 객체를 들고 있으면, 재연결 뒤에도 **죽은 팔**에 대고
    좌표를 계산하게 된다. SequenceRunner가 hardware 딕셔너리를 통째로 받는
    것과 같은 이유다.
    """
    obj = src() if callable(src) else src
    if obj is None:
        raise RuntimeError("팔이 연결돼 있지 않습니다 — 팔 없이 보정할 수 없습니다.")
    return obj


# ----------------------------------------------------------------------
# 보정 결과 저장소
# ----------------------------------------------------------------------

class EyeConfig:
    """~/arm_eye.json — 이 로봇의 손-눈 보정 실측값.

    ARM_CART_FILE과 같은 규칙이다: 코드(config.py)는 공장 초기값, 파일은 이
    로봇에서 실제로 잰 값이고 **파일이 이긴다.**

    파일이 스스로를 설명하게 둔다 — 언제·몇 개 표본으로·잔차 얼마로 잡았는지,
    그리고 그때 카메라 내부파라미터가 무엇이었는지까지 적는다. 나중에 카메라
    해상도를 바꾸면 내부파라미터가 달라지는데, 그때 이 기록이 없으면 왜 갑자기
    빗나가는지 아무도 모른다.
    """

    def __init__(self, path: str = ARM_EYE_FILE,
                 camera: str = DEPTH_CAMERA_DEFAULT) -> None:
        self._path = os.path.expanduser(path)
        self._camera = camera
        self._lock = threading.RLock()
        self._data: dict = {}
        self.reload()

    @property
    def path(self) -> str:
        return self._path

    @property
    def camera(self) -> str:
        return self._camera

    def _scope(self, create: bool = False) -> dict:
        """이 카메라 몫의 칸. **카메라마다 보정이 따로다.**

        ⚠ 왜 한 파일에 나눠 담나 — 카메라가 둘이 되면서(2026-09-01) 보정도
          둘이 됐다. 같은 칸을 쓰면 Astra를 잡는 순간 D405 보정이 **조용히**
          덮여 사라지고, 다음에 D405로 팔을 보내면 엉뚱한 데로 간다.
          파일을 나누지 않은 건 "보정은 한 벌"이라는 규칙(CLAUDE.md)과
          ROS 쪽 저장소(tomato_handeye/store.py)를 지키기 위해서다.

        ⚠ 기본 카메라(d405)는 **파일 최상위**를 그대로 쓴다 — 옛 파일과
          ROS가 그 자리를 읽고 있다. 형식을 바꾸면 둘 다 조용히 못 읽는다.
        """
        if self._camera == DEPTH_CAMERA_DEFAULT:
            return self._data
        cams = self._data.setdefault("cameras", {}) if create else self._data.get("cameras", {})
        if create:
            return cams.setdefault(self._camera, {})
        return cams.get(self._camera, {})

    def reload(self) -> None:
        with self._lock:
            try:
                with open(self._path, encoding="utf-8") as f:
                    self._data = json.load(f)
            except (OSError, ValueError):
                self._data = {}

    def _merged(self, disk: dict) -> dict:
        """디스크에 있는 **남의 카메라 칸을 살린 채** 내 칸만 얹는다.

        ⚠ 이게 없으면 카메라 둘이 서로를 지운다. 두 EyeConfig 객체는 각자
          자기가 읽은 시점의 파일 사본을 들고 있어서, 나중에 저장하는 쪽이
          그 사본을 통째로 덮어쓴다 — Astra를 보정하는 순간 D405 보정이
          말없이 사라지고, 화면은 여전히 "보정됨"이라고 말한다.
          (자체검증 tools/eye_check.py가 실제로 이 버그를 잡아냈다.)
        """
        if self._camera == DEPTH_CAMERA_DEFAULT:
            out = dict(self._data)
            cams = disk.get("cameras")
            if isinstance(cams, dict):
                out["cameras"] = cams
            return out
        out = dict(disk)
        cams = dict(out.get("cameras") or {})
        cams[self._camera] = dict(self._scope(create=True))
        out["cameras"] = cams
        return out

    def _save(self) -> None:
        with self._lock:
            try:
                with open(self._path, encoding="utf-8") as f:
                    disk = json.load(f)
            except (OSError, ValueError):
                disk = {}
            self._data = self._merged(disk if isinstance(disk, dict) else {})
            d = os.path.dirname(self._path) or "."
            fd, tmp = tempfile.mkstemp(dir=d, prefix=".arm_eye.", suffix=".json")
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as f:
                    json.dump(self._data, f, ensure_ascii=False, indent=2)
                os.replace(tmp, self._path)
            except BaseException:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass
                raise

    # --- 읽기 ---

    @property
    def mount(self) -> str:
        m = self._scope().get("mount", ARM_EYE_MOUNT)
        return m if m in MOUNTS else ARM_EYE_MOUNT

    @property
    def has_calibration(self) -> bool:
        return isinstance(self._scope().get("transform"), dict)

    @property
    def transform(self) -> Rigid | None:
        t = self._scope().get("transform")
        return Rigid.from_dict(t) if isinstance(t, dict) else None

    @property
    def rms_mm(self) -> float | None:
        v = self._scope().get("rms_mm")
        return float(v) if v is not None else None

    @property
    def good(self) -> bool:
        r = self.rms_mm
        return r is not None and r <= GOOD_RMS_MM

    @property
    def intrinsics(self) -> Intrinsics | None:
        i = self._scope().get("intrinsics")
        return Intrinsics.from_dict(i) if isinstance(i, dict) else None

    # --- 쓰기 ---

    def set_mount(self, mount: str) -> None:
        if mount not in MOUNTS:
            raise ValueError(f"mount는 {MOUNTS} 중 하나여야 합니다 (받은 값: {mount})")
        with self._lock:
            scope = self._scope(create=True)
            if mount != self.mount and self.has_calibration:
                # 장착이 바뀌면 기존 변환은 뜻이 통째로 달라진다 — 남겨두면
                # "보정돼 있다"고 표시된 채 팔이 엉뚱한 데로 간다.
                scope.pop("transform", None)
                scope.pop("rms_mm", None)
                scope["note"] = "장착 방식이 바뀌어 기존 보정을 버렸습니다."
            scope["mount"] = mount
            self._save()

    def store(self, mount: str, fit: Fit, intrinsics: Intrinsics | None,
              note: str = "") -> None:
        with self._lock:
            self._scope(create=True).update({
                "mount": mount,
                "camera": self._camera,
                "transform": fit.transform.as_dict(),
                "rms_mm": round(fit.rms_mm, 3),
                "max_mm": round(fit.max_mm, 3),
                "samples": fit.samples,
                "scale_hint": round(fit.scale_hint, 5),
                "marker_base": list(fit.marker_base) if fit.marker_base else None,
                "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "intrinsics": intrinsics.as_dict() if intrinsics else None,
                "note": note,
            })
            self._save()

    def clear(self) -> None:
        with self._lock:
            scope = self._scope(create=True)
            scope.pop("transform", None)
            scope.pop("rms_mm", None)
            scope.pop("max_mm", None)
            scope.pop("marker_base", None)
            scope["note"] = "보정 해제됨"
            self._save()

    def snapshot(self) -> dict:
        with self._lock:
            d = dict(self._scope())
        d.pop("cameras", None)      # 최상위(d405)를 볼 때 남의 칸까지 실려 나가지 않게
        d["path"] = self._path
        d["camera"] = self._camera
        d["mount"] = self.mount
        d["mount_label"] = MOUNT_LABEL[self.mount]
        d["has_calibration"] = self.has_calibration
        d["good"] = self.good
        if self.has_calibration:
            t = self.transform
            roll, pitch, yaw = t.rpy_deg
            # 사람이 "그럴듯한가"를 눈으로 볼 수 있게 — 삼각대가 팔 앞 40cm에
            # 있다면 이 값이 그 정도여야 한다. 자릿수가 다르면 뭔가 틀린 것이다.
            #
            # ⚠ 변환은 **카메라 → 팔** 방향(p_base = R·p_cam + t)이라, 카메라
            #   원점(p_cam=0)이 팔 좌표로 어디인지는 t **그대로**다. 여기에
            #   inverse()를 쓰면 반대로 "팔 원점이 카메라에서 어디인지"가 나와,
            #   보정은 멀쩡한데 화면 숫자만 엉뚱해진다(실제로 한 번 그랬다).
            d["camera_at"] = [round(v, 1) for v in t.t.tolist()]
            d["camera_at_label"] = ("팔 base 기준 카메라 위치" if self.mount == "fixed"
                                    else "집게 기준 카메라 위치")
            d["rpy_deg"] = [round(roll, 1), round(pitch, 1), round(yaw, 1)]
        return d


# ----------------------------------------------------------------------
# 보정 진행 — 표본을 모으고 푼다
# ----------------------------------------------------------------------

class EyeCalibrator:
    """표본은 **메모리에만** 둔다 — 풀어서 저장된 변환만 파일에 남는다.

    왜 표본을 파일에 안 남기나 — 표본은 그 순간의 팔 자세와 짝지어진 값이라,
    무대나 삼각대를 건드린 뒤 남은 표본에 새 표본을 섞으면 조용히 망가진다.
    한 번에 다 찍고 풀고 끝내는 게 맞다. (그래서 서비스를 재시작하면 표본이
    사라진다 — harvest.py의 무대 학습이 그 반대 선택을 했다가 데였지만,
    이쪽은 사라지는 게 안전한 방향이다.)
    """

    def __init__(self, view: DepthView, cartesian, cfg: EyeConfig) -> None:
        self._view = view
        self._cart_src = cartesian
        self._cfg = cfg
        self._lock = threading.RLock()
        self._samples: list[dict] = []

    @property
    def _cart(self):
        return _resolve(self._cart_src)

    # --- 표본 ---

    def add_sample(self, u: float, v: float) -> str:
        """화면 픽셀 (u,v)를 지금 팔 자세와 묶어 표본 하나로 담는다.

        fixed  : (u,v)는 **집게가 무는 지점(TCP)** 이어야 한다.
        on_arm : (u,v)는 **고정해 둔 마커** 여야 한다.
        """
        if not self._cart.config.has_zero:
            raise RuntimeError(
                "기구학 영점이 없습니다 — /settings의 [3D 좌표 영점]을 먼저 잡으세요. "
                "영점이 없으면 FK가 말하는 집게 좌표에 뜻이 없어서, 그걸 짝으로 쓰는 "
                "이 보정도 통째로 무의미합니다.")

        p_cam = self._view.point_at(u, v)          # 못 믿으면 DepthError
        pose = self._cart.pose()
        mount = self._cfg.mount
        with self._lock:
            self._samples.append({
                "u": float(u), "v": float(v),
                "cam": [float(c) for c in p_cam],
                "pose": pose.as_dict(),
                "tcp": [pose.x, pose.y, pose.z],
                "at": time.strftime("%H:%M:%S"),
            })
            n = len(self._samples)
        need = self.needed()
        what = "집게 끝" if mount == "fixed" else "마커"
        return (f"표본 {n}개 — {what} @ 화면({int(u)},{int(v)}) "
                f"거리 {p_cam[2]:.0f}mm, 집게 좌표 "
                f"({pose.x:.0f}, {pose.y:.0f}, {pose.z:.0f})mm. "
                + (f"{need - n}개 더 필요합니다." if n < need
                   else "이제 [보정 풀기]를 누를 수 있습니다."))

    def drop_last(self) -> str:
        with self._lock:
            if not self._samples:
                return "지울 표본이 없습니다."
            s = self._samples.pop()
        return f"마지막 표본(화면 {int(s['u'])},{int(s['v'])})을 지웠습니다 — {len(self._samples)}개 남음."

    def drop(self, index: int) -> str:
        with self._lock:
            if not (0 <= index < len(self._samples)):
                return f"{index}번 표본이 없습니다."
            s = self._samples.pop(index)
        return f"{index}번 표본을 지웠습니다 — {len(self._samples)}개 남음."

    def clear(self) -> str:
        with self._lock:
            n = len(self._samples)
            self._samples.clear()
        return f"표본 {n}개를 모두 비웠습니다."

    def needed(self) -> int:
        return (ARM_EYE_MIN_SAMPLES_FIXED if self._cfg.mount == "fixed"
                else ARM_EYE_MIN_SAMPLES_ON_ARM)

    def samples(self) -> list[dict]:
        with self._lock:
            return [dict(s) for s in self._samples]

    # --- 풀기 ---

    def solve(self, save: bool = True) -> str:
        with self._lock:
            samples = [dict(s) for s in self._samples]
        need = self.needed()
        if len(samples) < need:
            raise CalibrationError(
                f"표본이 {len(samples)}개다 — {MOUNT_LABEL[self._cfg.mount]}은 "
                f"{need}개 이상이 필요하다. 팔을 **멀찍이** 옮겨가며 더 찍어라.")

        cam = np.array([s["cam"] for s in samples])
        mount = self._cfg.mount
        if mount == "fixed":
            base = np.array([s["tcp"] for s in samples])
            fit = solve_fixed(cam, base)
        else:
            from .kinematics import ToolPose

            frames = [tool_frame(ToolPose(**s["pose"])) for s in samples]
            fit = solve_on_arm(cam, frames)

        intr = None
        try:
            intr = self._view.intrinsics()
        except DepthError:
            pass

        if save:
            self._cfg.store(mount, fit, intr,
                            note=f"{MOUNT_LABEL[mount]} · 표본 {fit.samples}개")

        msg = fit.summary()
        worst = fit.worst_index()
        if worst >= 0 and fit.per_sample_mm[worst] > max(2.0 * fit.rms_mm, 8.0):
            msg += (f" · {worst}번 표본이 유독 어긋난다({fit.per_sample_mm[worst]:.0f}mm) "
                    "— 잘못 클릭했을 수 있으니 그것만 지우고 다시 풀어 보라")
        if not fit.good:
            msg += " · 저장은 했지만 이대로 팔을 보내지 마라"
        return msg


# ----------------------------------------------------------------------
# 보정된 눈 — 실제로 쓰는 쪽
# ----------------------------------------------------------------------

class Eye:
    """픽셀 → 팔 좌표. 보정이 없거나 나쁘면 **거절한다.**"""

    def __init__(self, view: DepthView, cartesian, cfg: EyeConfig | None = None) -> None:
        self._view = view
        self._cart_src = cartesian
        # ⚠ 보정 칸은 **이 카메라 것**을 쓴다. 넘겨받지 않았을 때 무심코
        #   기본값을 쓰면 Astra로 찍은 표본이 D405 보정을 덮어쓴다.
        self.config = cfg or EyeConfig(camera=view.camera)
        self.calibrator = EyeCalibrator(view, cartesian, self.config)

    @property
    def camera(self) -> str:
        return self._view.camera

    @property
    def label(self) -> str:
        return self._view.label

    @property
    def _cart(self):
        return _resolve(self._cart_src)

    # --- 변환 ---

    def cam_to_base(self, p_cam) -> tuple[float, float, float]:
        """카메라 좌표 3D → 팔 base 좌표 3D (mm)."""
        T = self.config.transform
        if T is None:
            raise RuntimeError(
                "손-눈 보정이 없습니다 — /settings의 [카메라 3D 보정]에서 먼저 잡으세요.")
        p = np.asarray(p_cam, dtype=float)
        if self.config.mount == "on_arm":
            # 카메라가 팔과 함께 움직인다 — 지금 손목이 어디 있는지가 매번 필요하다.
            p = tool_frame(self._cart.pose()).apply(T.apply(p))
        else:
            p = T.apply(p)
        return (float(p[0]), float(p[1]), float(p[2]))

    def pixel_to_base(self, u: float, v: float) -> tuple[float, float, float]:
        """화면 픽셀 → 팔 base 좌표 (mm). 깊이가 못 믿을 값이면 DepthError."""
        return self.cam_to_base(self._view.point_at(u, v))

    def probe(self, u: float, v: float) -> str:
        """그 픽셀이 **얼마나 먼지만** 알려준다 — 팔은 건드리지 않는다.

        보정 전에도 쓸 수 있어야 한다. 이게 있어야 "카메라가 무대를 볼 수 있는
        거리에 있는가"를 원격에서 확인할 수 있다(D405 유효범위 7~50cm).
        보정이 있으면 팔 좌표까지 덧붙인다.
        """
        p_cam = self._view.point_at(u, v)
        msg = (f"화면({int(u)},{int(v)}) 거리 {p_cam[2]:.0f}mm · 카메라 좌표 "
               f"({p_cam[0]:.0f}, {p_cam[1]:.0f}, {p_cam[2]:.0f})mm")
        if self.config.has_calibration:
            x, y, z = self.cam_to_base(p_cam)
            msg += f" → 팔 좌표 ({x:.0f}, {y:.0f}, {z:.0f})mm"
        else:
            msg += " (보정 전이라 팔 좌표는 아직 없음)"
        return msg

    # --- 열매 찾기 ---

    def fruits(self, ripe_only: bool = True) -> list[dict]:
        """컬러에서 열매를 찾아 **3D 좌표까지** 붙여 돌려준다.

        깊이가 안 나오는 열매는 버리지 않고 `why`를 달아 남긴다 — "화면에는
        보이는데 목록에 없다"가 가장 답답한 상황이라, 왜 없는지를 말해준다.

        ⚠ **컬러가 깊이에 정렬된 카메라에서만 된다.** 안 그러면 컬러에서 찾은
          (u,v)를 깊이 격자에 그대로 넣는 셈이라, 열매 옆 잎의 거리를 열매의
          거리라고 말하게 된다 — 그리고 아무 에러도 안 난다.
        """
        from ..vision.color_detect import detect_fruits

        self._view.require_color_aligned()

        frame = self._view.color_bgr()
        out = []
        for f in detect_fruits(frame):
            if ripe_only and not f.ripe:
                continue
            u, v = f.position
            item = {"u": int(u), "v": int(v), "area": int(f.area), "ripe": bool(f.ripe)}
            try:
                p_cam = self._view.point_at(u, v)
                item["dist_mm"] = round(float(p_cam[2]), 1)
                x, y, z = self.cam_to_base(p_cam)
                item.update({"x": round(x, 1), "y": round(y, 1), "z": round(z, 1)})
            except (DepthError, RuntimeError) as e:
                item["why"] = str(e)
            out.append(item)
        out.sort(key=lambda d: d.get("dist_mm", 1e9))
        return out

    # --- 팔을 보내기 ---

    def approach_pose(self, x: float, y: float, z: float,
                      pitch: float | None = None,
                      standoff: float | None = None) -> dict:
        """열매 좌표 → **그 앞에 서는** 집게 좌표. 실제로 가지는 않는다.

        목표점은 열매 **표면**이다(카메라는 앞면만 본다). 거기로 그대로 가면
        집게가 열매를 밀어 떨어뜨리므로, 접근축으로 standoff만큼 물러선다.
        """
        ph = math.radians(ARM_EYE_PICK_PITCH_DEG if pitch is None else pitch)
        off = ARM_EYE_STANDOFF_MM if standoff is None else float(standoff)
        th = math.atan2(y, x)
        # 접근 단위벡터 — kinematics.tool_axes의 approach와 같은 정의다.
        ax = (math.cos(ph) * math.cos(th), math.cos(ph) * math.sin(th), math.sin(ph))
        return {
            "x": x - ax[0] * off, "y": y - ax[1] * off, "z": z - ax[2] * off,
            "pitch": math.degrees(ph),
            "target": {"x": x, "y": y, "z": z},
            "standoff": off,
        }

    def aim(self, u: float, v: float, standoff: float | None = None,
            pitch: float | None = None) -> str:
        """화면의 그 점 **앞으로** 집게를 가져간다.

        ⚠ 한 번에 안 간다 — cartesian.move_to는 좌표 오타를 막으려고 한 번에
          ARM_CART_MAX_STEP_MM까지만 움직인다. 그 제한을 우회하지 않고 **여러
          번에 나눠** 간다. 제한은 여기서도 지켜야 할 안전장치지 장애물이 아니다.
        """
        if not self.config.has_calibration:
            raise RuntimeError(
                "손-눈 보정이 없습니다 — /settings의 [카메라 3D 보정]에서 먼저 잡으세요.")
        if not self.config.good:
            raise RuntimeError(
                f"보정 잔차가 {self.config.rms_mm:.0f}mm입니다(기준 {GOOD_RMS_MM:.0f}mm) — "
                "이 상태로 팔을 보내면 열매 옆을 집습니다. 표본을 더 흩어 다시 잡으세요.")

        x, y, z = self.pixel_to_base(u, v)
        plan = self.approach_pose(x, y, z, pitch=pitch, standoff=standoff)

        moved = 0
        last = ""
        for _ in range(12):
            now = self._cart.pose()
            dx, dy, dz = plan["x"] - now.x, plan["y"] - now.y, plan["z"] - now.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < 2.0:
                break
            f = min(1.0, (ARM_CART_MAX_STEP_MM * 0.9) / dist)
            last = self._cart.move_to(
                x=now.x + dx * f, y=now.y + dy * f, z=now.z + dz * f,
                pitch=plan["pitch"],
            )
            moved += 1
        else:
            return (f"목표까지 다 못 갔습니다({moved}번 이동) — 사거리 밖이거나 "
                    "관절 한계에 걸렸을 수 있습니다. " + last)

        return (f"열매 앞 {plan['standoff']:.0f}mm에 섰습니다 — 열매 "
                f"({x:.0f}, {y:.0f}, {z:.0f})mm, 집게 "
                f"({plan['x']:.0f}, {plan['y']:.0f}, {plan['z']:.0f})mm, "
                f"pitch {plan['pitch']:.0f}° ({moved}번에 나눠 이동). "
                "집으려면 [집게 닫기] 후 tool 프레임으로 전진하세요.")

    # --- 화면 ---

    def snapshot(self) -> dict:
        try:
            zeroed = bool(self._cart.config.has_zero)
        except (RuntimeError, AttributeError):
            zeroed = False       # 팔이 아직 안 붙었다 — 화면은 그래도 떠야 한다
        out = {
            "camera": self._view.status(),
            "camera_name": self._view.camera,
            "camera_label": self._view.label,
            "color_aligned": self._view.color_aligned,
            "calibration": self.config.snapshot(),
            "samples": self.calibrator.samples(),
            "needed": self.calibrator.needed(),
            "arm_zeroed": zeroed,
            "standoff_mm": ARM_EYE_STANDOFF_MM,
            "pick_pitch_deg": ARM_EYE_PICK_PITCH_DEG,
        }
        if not out["arm_zeroed"]:
            out["blocker"] = ("기구학 영점이 먼저입니다 — [3D 좌표 영점]을 잡기 전에는 "
                              "보정을 시작해도 의미가 없습니다.")
        elif not out["camera"].get("ok"):
            out["blocker"] = out["camera"].get("why")
        return out
