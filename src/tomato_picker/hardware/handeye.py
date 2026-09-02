"""손-눈 보정(hand-eye) — **카메라가 본 3D 점을 팔의 좌표로 옮긴다.**

D405는 픽셀마다 거리를 준다. 그런데 그 좌표는 **카메라가 자기 눈 기준으로**
말하는 값이라 팔에게는 아무 의미가 없다. 팔은 자기 어깨 기준(base frame)으로만
움직인다. 이 모듈이 그 사이의 **강체 변환 하나**를 실측에서 푼다.

    카메라가 본 점 p_cam ──(R, t)──> 팔이 아는 점 p_base

⚠ 이 값은 **추정하는 게 아니라 재는 것**이다. 자로 "카메라가 팔 앞 30cm,
   15° 아래" 같은 걸 재서 넣으면 각도 2°만 틀려도 50cm 앞에서 17mm 어긋난다.
   그래서 여기서는 **팔을 실제로 여러 자세로 움직이며 카메라로 보고**, 그
   대응관계에서 변환을 푼다. 사람이 각도를 재지 않는다.

────────────────────────────────────────────────────────────────────────
장착 방식이 두 가지고, **푸는 식이 다르다.**

① `fixed` — 카메라가 팔과 무관하게 고정 (삼각대·차체)
     미지수 = T_base_cam (6)
     집게(TCP) 자체를 마커로 삼는다. 팔을 N자세로 옮기며 그때마다
     (카메라가 본 집게 위치, FK로 아는 집게 위치)를 한 쌍씩 모은다.
     → 대응점 쌍이 그대로 있으므로 Kabsch(SVD) 한 방에 닫힌 해가 나온다. N≥3.

     ⚠⚠ **삼각대는 지면에 고정이지 로봇에 고정이 아니다.** 베이스가 굴러가면
        이 변환은 그 순간 거짓이 된다 — 코드는 그걸 알 방법이 없고, 팔은
        아무 일 없다는 듯 엉뚱한 데로 간다. 그래서 `fixed` 보정은 **주행하지
        않는 동안에만** 유효하다. 이 모듈은 그 사실을 파일에 적어 두고
        (`mount="fixed"`), 소비하는 쪽이 경고를 띄우게 한다.

② `on_arm` — 카메라가 손목에 붙어 팔과 함께 움직임 (eye-in-hand)
     미지수 = T_tool_cam (6) + 고정 마커의 위치 p_base (3) = 9
     마커를 한 자리에 두고, 팔을 N자세로 옮기며 매번 카메라로 그 마커를 본다.
     각 자세가 3개 식을 주므로 N≥5면 풀린다(권장 8 이상, 자세를 **많이 다르게**).

         p_base = R_i (R_x p_i + t_x) + b_i        (R_i, b_i = FK로 아는 T_base_tool)

     vec(AXB) = (Bᵀ ⊗ A) vec(X) 로 펴면 미지수 15개(R_x 9 + t_x 3 + p_base 3)의
     **선형** 최소자승이 된다. 풀고 나서 R_x를 SVD로 SO(3)에 투영하고,
     그 R_x를 고정한 채 나머지를 다시 풀어 다듬는다.

────────────────────────────────────────────────────────────────────────
여기에 하드웨어도 pyrealsense2도 cv2도 없다 — numpy뿐이다. 이유는
kinematics.py와 같다: **보정이 맞았는지는 숫자 문제**인데, 그걸 확인하려고
매번 젯슨에 올려 팔을 흔들 수는 없다. `tools/handeye_check.py`가 여기에
가짜 변환을 심어 놓고 되찾아오는지 PC에서 검증한다.

⚠ **잔차(residual)를 반드시 보라.** 이 계산은 입력이 쓰레기여도 답을 내놓는다
   (최소자승은 늘 뭔가를 돌려준다). 맞았는지 아닌지는 오직 잔차가 말한다.
   이 로봇의 1번 병("지령은 나갔는데 아무 일도 안 일어난다")의 보정판은
   **"보정은 됐다는데 팔이 옆으로 간다"**이고, 그건 잔차를 안 본 것이다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# 잔차가 이보다 크면 "보정됐다"고 말하면 안 된다(mm). 열매 지름이 30~40mm라
# 15mm를 넘으면 집게가 헛집는다.
GOOD_RMS_MM = 15.0
# 대응점이 한 직선에 몰리면 회전이 결정되지 않는다 — 이 비율 아래면 거절.
DEGENERATE_RATIO = 0.02


class CalibrationError(ValueError):
    """보정을 풀 수 없다 — 표본이 모자라거나, 한 직선/한 점에 몰려 있다.

    ⚠ 여기서 **가까운 답으로 몰래 대체하지 않는다.** 팔은 이 값을 믿고
       움직이므로, 못 푼 걸 푼 척하는 게 가장 위험하다.
    """


# ----------------------------------------------------------------------
# 카메라 내부 파라미터 — 픽셀 + 거리 → 카메라 좌표 3D
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class Intrinsics:
    """D405가 알려주는 렌즈 상수. **여기 단위는 mm다.**

    카메라에서 받아 그대로 저장해 두고, pyrealsense2가 없는 쪽(음성/팔 서비스는
    다른 venv다)에서도 픽셀을 3D로 풀 수 있게 한다. 이게 이 dataclass의 존재
    이유다 — 값을 파일로 옮겨 venv 경계를 넘긴다.

    model: "none" | "inverse_brown_conrady" | "brown_conrady"
      · D405의 **depth 스트림은 왜곡계수가 전부 0**(정류된 스트림)이라 순수 핀홀.
      · **color 스트림은 inverse_brown_conrady**에 계수가 실제로 있다.
        깊이를 컬러에 정렬(align to color)해 쓰므로 보통 이쪽을 쓴다.
    """

    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float
    model: str = "none"
    coeffs: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def deproject(self, u: float, v: float, z_mm: float) -> tuple[float, float, float]:
        """픽셀 (u,v)와 그 픽셀의 거리(mm) → 카메라 좌표 (x,y,z) mm.

        카메라 광학 좌표계 규약(librealsense 그대로): +x 오른쪽, +y **아래**,
        +z 앞(장면 쪽). y가 아래인 게 팔 좌표계(+z 위)와 달라 헷갈리는데,
        **여기서 굳이 뒤집지 않는다** — 손-눈 변환이 그 회전까지 통째로
        학습하므로, 중간에 사람이 축을 뒤집으면 오히려 두 번 뒤집힐 뿐이다.
        """
        x = (u - self.ppx) / self.fx
        y = (v - self.ppy) / self.fy
        c = list(self.coeffs) + [0.0] * (5 - len(self.coeffs))

        if self.model == "inverse_brown_conrady" and any(c):
            # librealsense의 deproject와 같은 식 — 이 모델은 **곧바로** 다항식을
            # 적용한다(반복 없음). 역방향이 아니라는 점이 이름과 헷갈린다.
            r2 = x * x + y * y
            f = 1.0 + c[0] * r2 + c[1] * r2 * r2 + c[4] * r2 * r2 * r2
            ux = x * f + 2.0 * c[2] * x * y + c[3] * (r2 + 2.0 * x * x)
            uy = y * f + 2.0 * c[3] * x * y + c[2] * (r2 + 2.0 * y * y)
            x, y = ux, uy
        elif self.model == "brown_conrady" and any(c):
            # 이쪽은 왜곡을 **풀어야** 한다 — 고정점 반복으로 충분히 수렴한다.
            x0, y0 = x, y
            for _ in range(10):
                r2 = x * x + y * y
                icd = 1.0 / (1.0 + c[0] * r2 + c[1] * r2 * r2 + c[4] * r2 * r2 * r2)
                dx = 2.0 * c[2] * x * y + c[3] * (r2 + 2.0 * x * x)
                dy = 2.0 * c[3] * x * y + c[2] * (r2 + 2.0 * y * y)
                x = (x0 - dx) * icd
                y = (y0 - dy) * icd

        return (x * z_mm, y * z_mm, z_mm)

    def as_dict(self) -> dict:
        return {"width": self.width, "height": self.height,
                "fx": self.fx, "fy": self.fy, "ppx": self.ppx, "ppy": self.ppy,
                "model": self.model, "coeffs": list(self.coeffs)}

    @staticmethod
    def from_dict(d: dict) -> "Intrinsics":
        return Intrinsics(
            width=int(d["width"]), height=int(d["height"]),
            fx=float(d["fx"]), fy=float(d["fy"]),
            ppx=float(d["ppx"]), ppy=float(d["ppy"]),
            model=str(d.get("model", "none")),
            coeffs=tuple(float(x) for x in d.get("coeffs", ())),
        )


# ----------------------------------------------------------------------
# 강체 변환
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class Rigid:
    """회전 + 평행이동. p' = R p + t (t는 mm)."""

    R: np.ndarray
    t: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "R", np.asarray(self.R, dtype=float).reshape(3, 3))
        object.__setattr__(self, "t", np.asarray(self.t, dtype=float).reshape(3))

    def apply(self, p) -> np.ndarray:
        """점 하나(3,) 또는 여러 개(N,3)를 옮긴다."""
        p = np.asarray(p, dtype=float)
        return p @ self.R.T + self.t if p.ndim == 2 else self.R @ p + self.t

    def inverse(self) -> "Rigid":
        Rt = self.R.T
        return Rigid(Rt, -Rt @ self.t)

    def compose(self, other: "Rigid") -> "Rigid":
        """self ∘ other — other로 옮긴 뒤 self로 옮기는 것과 같다."""
        return Rigid(self.R @ other.R, self.R @ other.t + self.t)

    @property
    def rpy_deg(self) -> tuple[float, float, float]:
        """사람이 읽을 용도의 ZYX 오일러각(도). **계산에는 쓰지 않는다** —
        짐벌락에서 값이 튀므로 저장·복원은 항상 행렬 그대로 한다."""
        sy = -self.R[2, 0]
        sy = max(-1.0, min(1.0, sy))
        pitch = math.asin(sy)
        if abs(sy) < 0.99999:
            roll = math.atan2(self.R[2, 1], self.R[2, 2])
            yaw = math.atan2(self.R[1, 0], self.R[0, 0])
        else:
            roll = math.atan2(-self.R[1, 2], self.R[1, 1])
            yaw = 0.0
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def as_dict(self) -> dict:
        return {"R": [round(v, 9) for v in self.R.flatten().tolist()],
                "t": [round(v, 4) for v in self.t.tolist()]}

    @staticmethod
    def from_dict(d: dict) -> "Rigid":
        return Rigid(np.asarray(d["R"], dtype=float).reshape(3, 3),
                     np.asarray(d["t"], dtype=float).reshape(3))

    @staticmethod
    def identity() -> "Rigid":
        return Rigid(np.eye(3), np.zeros(3))


@dataclass
class Fit:
    """보정 결과 + **믿어도 되는지에 대한 근거**.

    transform 하나만 돌려주지 않는 이유 — 최소자승은 입력이 무엇이든 답을
    내놓는다. 이 로봇에서 위험한 건 틀린 답이 아니라 **틀렸다는 걸 모르는 것**이다.
    """

    transform: Rigid
    rms_mm: float
    max_mm: float
    per_sample_mm: list[float] = field(default_factory=list)
    samples: int = 0
    # 진단용. 대응점에서 나온 최적 배율 — 1.0에서 크게 벗어나면 **단위나 링크
    # 길이가 틀린 것**이다(깊이를 m로 넣었거나, ARM_GEOM_L*이 실제와 다르거나).
    scale_hint: float = 1.0
    # on_arm에서만: 함께 풀린 마커의 base 좌표.
    marker_base: tuple[float, float, float] | None = None
    note: str = ""

    @property
    def good(self) -> bool:
        return self.rms_mm <= GOOD_RMS_MM

    def summary(self) -> str:
        head = "✅" if self.good else "⚠"
        s = (f"{head} 표본 {self.samples}개 · 잔차 RMS {self.rms_mm:.1f}mm "
             f"(최대 {self.max_mm:.1f}mm)")
        if abs(self.scale_hint - 1.0) > 0.05:
            s += (f" · ⚠ 배율 {self.scale_hint:.3f} — 깊이 단위나 링크 길이"
                  f"(ARM_GEOM_L*)를 의심하라")
        if not self.good:
            s += f" · {GOOD_RMS_MM:.0f}mm를 넘으면 집게가 헛집는다"
        return s

    def worst_index(self) -> int:
        """가장 어긋난 표본 번호 — 하나만 잘못 찍었을 때 그것만 지우면 된다."""
        if not self.per_sample_mm:
            return -1
        return int(np.argmax(self.per_sample_mm))


# ----------------------------------------------------------------------
# ① fixed — 카메라가 팔과 무관하게 고정
# ----------------------------------------------------------------------

def solve_fixed(cam_pts, base_pts) -> Fit:
    """대응점 쌍 → T_base_cam. Kabsch(SVD), 닫힌 해.

    cam_pts[i]  = 카메라가 본 집게 끝의 좌표 (mm, 카메라 좌표계)
    base_pts[i] = 그때 FK가 말하는 집게 끝의 좌표 (mm, 팔 base 좌표계)
    """
    P = _as_points(cam_pts, "카메라 점")
    Q = _as_points(base_pts, "팔 좌표 점")
    if len(P) != len(Q):
        raise CalibrationError(f"짝이 안 맞는다 — 카메라 {len(P)}개, 팔 {len(Q)}개.")
    if len(P) < 3:
        raise CalibrationError(
            f"표본이 {len(P)}개다 — 회전을 정하려면 최소 3개, 실제로는 5개 이상을 "
            "**서로 멀리 떨어진 자리**에서 찍어야 한다.")

    _reject_degenerate(P, "카메라")
    _reject_degenerate(Q, "팔")

    pc, qc = P.mean(axis=0), Q.mean(axis=0)
    Pc, Qc = P - pc, Q - qc

    U, S, Vt = np.linalg.svd(Pc.T @ Qc)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    # 반사(거울상)를 회전으로 착각하지 않게 마지막 축 부호를 눌러 준다.
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = qc - R @ pc
    fit = Rigid(R, t)

    # 배율 진단(적용하지 않는다) — 두 점구름의 퍼진 정도 비.
    denom = float((Pc ** 2).sum())
    scale = float(S.sum() * d / denom) if denom > 1e-9 else 1.0

    return _score(fit, P, Q, scale, "fixed")


# ----------------------------------------------------------------------
# ② on_arm — 카메라가 손목에 붙어 팔과 함께 움직임
# ----------------------------------------------------------------------

def solve_on_arm(cam_pts, tool_frames) -> Fit:
    """고정 마커를 여러 자세에서 본 기록 → T_tool_cam (+ 마커의 base 좌표).

    cam_pts[i]     = 그 자세에서 카메라가 본 마커 (mm, 카메라 좌표계)
    tool_frames[i] = 그 자세의 T_base_tool (Rigid) — FK에서 만든다
    """
    P = _as_points(cam_pts, "카메라 점")
    frames = list(tool_frames)
    if len(P) != len(frames):
        raise CalibrationError(f"짝이 안 맞는다 — 카메라 {len(P)}개, 자세 {len(frames)}개.")
    if len(P) < 5:
        raise CalibrationError(
            f"표본이 {len(P)}개다 — 손목 장착은 미지수가 15개라 최소 5자세, 실제로는 "
            "8자세 이상을 **자세를 크게 바꿔가며** 찍어야 한다(같은 자세를 조금씩 "
            "옮긴 표본은 아무리 많아도 회전을 못 정한다).")

    # A·[vec(R_x); t_x; p_base] = b,  블록마다 3행
    A = np.zeros((3 * len(P), 15))
    b = np.zeros(3 * len(P))
    for i, (p, T) in enumerate(zip(P, frames)):
        r = slice(3 * i, 3 * i + 3)
        A[r, 0:9] = np.kron(p.reshape(1, 3), T.R)   # vec는 열우선(order='F')
        A[r, 9:12] = T.R
        A[r, 12:15] = -np.eye(3)
        b[r] = -T.t

    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    sv = np.linalg.svd(A, compute_uv=False)
    if sv[-1] <= sv[0] * 1e-8:
        raise CalibrationError(
            "자세가 서로 충분히 다르지 않다 — 손목 회전을 크게 바꿔가며 다시 찍어라. "
            "(같은 방향에서 거리만 바꾼 표본은 회전을 결정하지 못한다.)")

    R_x = _nearest_rotation(sol[0:9].reshape(3, 3, order="F"))

    # R_x를 SO(3)에 투영했으니 나머지를 그 R_x에 맞춰 다시 푼다(다듬기).
    A2 = np.zeros((3 * len(P), 6))
    b2 = np.zeros(3 * len(P))
    for i, (p, T) in enumerate(zip(P, frames)):
        r = slice(3 * i, 3 * i + 3)
        A2[r, 0:3] = T.R
        A2[r, 3:6] = -np.eye(3)
        b2[r] = -T.t - T.R @ (R_x @ p)
    sol2, *_ = np.linalg.lstsq(A2, b2, rcond=None)
    t_x, p_base = sol2[0:3], sol2[3:6]

    X = Rigid(R_x, t_x)
    pred = np.array([T.apply(X.apply(p)) for p, T in zip(P, frames)])
    truth = np.tile(p_base, (len(P), 1))

    fit = _score(X, pred, truth, 1.0, "on_arm")
    fit.marker_base = (float(p_base[0]), float(p_base[1]), float(p_base[2]))
    fit.transform = X
    return fit


# ----------------------------------------------------------------------
# 팔 자세 → Rigid
# ----------------------------------------------------------------------

def tool_frame(pose) -> Rigid:
    """kinematics.ToolPose → T_base_tool.

    회전은 kinematics.tool_axes가 주는 (approach, lateral, up)을 **열로 세운
    것**이 그대로다. 즉 도구 좌표 (1,0,0)은 집게가 찔러 들어가는 방향이다.

    ⚠ roll은 여기 안 들어간다 — tool_axes는 pitch/방위각만 쓴다. 손목 장착
      보정에서 wrist_roll을 돌려가며 표본을 찍으면 그 회전이 모델에 없어서
      잔차로 남는다. **손목 장착 보정 중에는 roll을 고정하라.**
    """
    from .kinematics import tool_axes  # 순환 import 방지 — 여기서만 필요하다.

    approach, lateral, up = tool_axes(pose)
    R = np.array([approach, lateral, up], dtype=float).T
    return Rigid(R, np.array([pose.x, pose.y, pose.z], dtype=float))


# ----------------------------------------------------------------------
# 내부
# ----------------------------------------------------------------------

def _as_points(pts, what: str) -> np.ndarray:
    arr = np.asarray(pts, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise CalibrationError(f"{what}의 모양이 (N,3)이 아니다: {arr.shape}")
    if not np.isfinite(arr).all():
        raise CalibrationError(
            f"{what}에 NaN/inf가 있다 — 깊이가 0인 픽셀(구멍)을 그대로 담았을 "
            "가능성이 높다. 표본을 담을 때 유효 깊이만 받아야 한다.")
    return arr


def _reject_degenerate(P: np.ndarray, what: str) -> None:
    """한 점/한 직선에 몰린 표본을 거절한다.

    왜 막나 — 직선 위의 점들만으로는 그 직선을 축으로 한 회전이 **자유롭게
    남는다**. 최소자승은 그중 아무거나 하나를 돌려주고, 잔차도 작게 나온다.
    "잔차는 좋은데 팔이 엉뚱하게 돈다"가 정확히 이 경우다.
    """
    C = P - P.mean(axis=0)
    sv = np.linalg.svd(C, compute_uv=False)
    if sv[0] < 1e-6:
        raise CalibrationError(f"{what} 표본이 전부 같은 점이다 — 팔을 옮겨가며 찍어라.")
    if sv[1] < sv[0] * DEGENERATE_RATIO:
        raise CalibrationError(
            f"{what} 표본이 거의 한 직선 위에 있다 — 그 직선을 축으로 한 회전이 "
            "결정되지 않는다(잔차는 작게 나오지만 답은 틀린다). 좌우·상하·앞뒤로 "
            "**서로 멀리 떨어뜨려** 다시 찍어라.")


def _nearest_rotation(M: np.ndarray) -> np.ndarray:
    """가장 가까운 정규직교 회전행렬 — 최소자승이 준 9개 값은 회전이 아니다."""
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1.0
        R = U @ Vt
    return R


def _score(fit: Rigid, src: np.ndarray, dst: np.ndarray,
           scale: float, mode: str) -> Fit:
    """잔차를 재서 Fit으로 포장. src는 이미 변환된 점이거나 원본 점이다."""
    pred = fit.apply(src) if mode == "fixed" else src
    err = np.linalg.norm(pred - dst, axis=1)
    return Fit(
        transform=fit,
        rms_mm=float(np.sqrt((err ** 2).mean())),
        max_mm=float(err.max()),
        per_sample_mm=[float(e) for e in err],
        samples=len(err),
        scale_hint=float(scale),
    )
