#!/usr/bin/env python3
"""손-눈 보정 자체검증 — **카메라도 팔도 젯슨도 없이** 개발 PC에서 돈다.

    python tools/handeye_check.py

[`handeye.py`](../src/tomato_picker/hardware/handeye.py)가 약속한 그 검증이다.
가짜 변환을 하나 심어 놓고, 그것이 만들어 냈을 관측을 합성한 뒤, 코드가 그
변환을 **되찾아오는지** 본다.

무엇을 확인하나:
  ① `fixed` — 심어 둔 T_base_cam을 되찾는가 (잔차 ≈ 0)
  ② `on_arm` — T_tool_cam과 마커 위치를 함께 되찾는가
  ③ **잡음이 섞이면 잔차가 그만큼 커지는가** (잔차가 거짓말을 안 하는가)
  ④ 퇴화 표본(한 점·한 직선)을 **거절하는가** — 이게 제일 중요하다
  ⑤ Intrinsics 투영↔역투영 왕복 (왜곡 모델 포함)
  ⑥ Rigid의 합성·역·rpy가 서로 앞뒤가 맞는가

왜 이걸 만들었나 — 보정의 버그는 "보정은 됐다는데 팔이 옆으로 간다"로 나타나고,
그건 열매를 뭉갠 뒤에 안다. **최소자승은 입력이 쓰레기여도 답을 내놓는다**는 것이
이 모듈의 근본 위험이고, 여기서 그 위험 자체를 시험한다.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

# 윈도우 콘솔(cp949)에서도 한글/기호가 안 깨지게 — arm_cartesian_check.py와 같은 처리.
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8")

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware.handeye import (  # noqa: E402
    CalibrationError, Intrinsics, Rigid, solve_fixed, solve_on_arm, tool_frame,
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


def expect_error(name: str, fn, must_contain: str = "") -> None:
    try:
        fn()
    except CalibrationError as exc:
        text = str(exc)
        check(name, must_contain in text, f"거절함: {text[:60]}")
        return
    except Exception as exc:  # noqa: BLE001
        check(name, False, f"다른 예외: {type(exc).__name__}: {exc}")
        return
    check(name, False, "거절하지 않고 답을 냈다 — 이게 제일 위험하다")


# ----------------------------------------------------------------------
# 가짜 세계 만들기
# ----------------------------------------------------------------------

def rot(axis: str, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def truth_camera() -> Rigid:
    """"카메라는 팔 앞 320mm, 왼쪽 40mm, 위 250mm에서 25° 내려다본다" 를 심는다.

    광학 좌표계 규약(+x 오른쪽, +y 아래, +z 앞)에서 팔 좌표계(+x 앞, +y 왼쪽,
    +z 위)로 가려면 축을 통째로 갈아끼워야 한다 — 그 회전을 손으로 안 적고
    행렬 곱으로 만드는 이유는, 손으로 적은 축 부호가 이 프로젝트에서 이미
    여러 번 틀렸기 때문이다.
    """
    optical_to_base = np.array([[0.0, 0.0, 1.0],    # cam +z(앞)  → base +x
                                [-1.0, 0.0, 0.0],   # cam +x(오른)→ base -y
                                [0.0, -1.0, 0.0]])  # cam +y(아래)→ base -z
    return Rigid(rot("y", -25.0) @ optical_to_base, np.array([320.0, 40.0, 250.0]))


def sample_poses(n: int = 10) -> list[kin.ToolPose]:
    """서로 충분히 다른 팔 자세들 — 사방으로 흩어져야 회전이 결정된다."""
    geom = kin.ArmGeometry()
    poses = []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        poses.append(kin.ToolPose(
            x=220.0 + 40.0 * math.cos(a),
            y=60.0 * math.sin(a),
            z=90.0 + 70.0 * math.cos(a * 1.7),
            pitch=-20.0 + 25.0 * math.sin(a * 0.9),
        ))
    # 실제로 IK가 풀리는 자세인지 확인해 둔다 — 못 가는 자세로 검증하면
    # "코드는 맞는데 현장에서 못 찍는" 절차를 검증한 셈이 된다.
    reachable = []
    for p in poses:
        try:
            kin.inverse(p, geom)
        except kin.Unreachable:
            continue
        reachable.append(p)
    return reachable


# ----------------------------------------------------------------------
# ① fixed
# ----------------------------------------------------------------------

def test_fixed() -> None:
    print("\n[fixed] 카메라 고정 — 집게를 마커로 삼는다")
    truth = truth_camera()
    inv = truth.inverse()
    poses = sample_poses()
    base_pts = [np.array([p.x, p.y, p.z]) for p in poses]
    cam_pts = [inv.apply(p) for p in base_pts]

    fit = solve_fixed(cam_pts, base_pts)
    check("표본 수", fit.samples == len(poses), f"{fit.samples}개")
    check("잔차 ≈ 0", fit.rms_mm < 1e-6, f"RMS {fit.rms_mm:.2e}mm")
    check("good 판정", fit.good, fit.summary())
    check("평행이동 복원", np.allclose(fit.transform.t, truth.t, atol=1e-6),
          f"t={np.round(fit.transform.t, 3).tolist()}")
    check("회전 복원", np.allclose(fit.transform.R, truth.R, atol=1e-9),
          f"rpy={tuple(round(v, 2) for v in fit.transform.rpy_deg)}")
    check("배율 힌트 ≈ 1", abs(fit.scale_hint - 1.0) < 1e-6,
          f"{fit.scale_hint:.6f}")


def test_fixed_noise() -> None:
    print("\n[fixed] 잡음 — 잔차가 거짓말을 안 하는가")
    rng = np.random.default_rng(20260828)
    truth = truth_camera()
    inv = truth.inverse()
    poses = sample_poses(12)
    base_pts = [np.array([p.x, p.y, p.z]) for p in poses]

    for sigma, expect_good in ((2.0, True), (30.0, False)):
        cam = [inv.apply(p) + rng.normal(0.0, sigma, 3) for p in base_pts]
        fit = solve_fixed(cam, base_pts)
        # 잔차는 잡음 크기와 같은 정도여야 한다 — 훨씬 작으면 과적합(표본 부족),
        # 훨씬 크면 계산이 틀린 것이다.
        ratio = fit.rms_mm / sigma
        check(f"σ={sigma:.0f}mm 잔차가 잡음과 같은 자릿수", 0.3 < ratio < 3.0,
              f"RMS {fit.rms_mm:.1f}mm (비 {ratio:.2f})")
        check(f"σ={sigma:.0f}mm good={expect_good}", fit.good is expect_good,
              fit.summary())
        check(f"σ={sigma:.0f}mm worst_index가 실재", 0 <= fit.worst_index() < len(cam),
              f"{fit.worst_index()}번")


def test_scale_hint() -> None:
    print("\n[fixed] 단위를 틀렸을 때 — 배율이 그걸 말하는가")
    truth = truth_camera()
    inv = truth.inverse()
    poses = sample_poses()
    base_pts = [np.array([p.x, p.y, p.z]) for p in poses]
    # 깊이를 m로 넣은 셈 치고 카메라 점을 1/1000로 — 흔한 실수다.
    cam_pts = [inv.apply(p) / 1000.0 for p in base_pts]
    fit = solve_fixed(cam_pts, base_pts)
    check("배율 힌트가 1000배를 가리킨다", fit.scale_hint > 100.0,
          f"scale={fit.scale_hint:.1f}")
    check("요약에 경고가 뜬다", "배율" in fit.summary(), fit.summary()[:80])


# ----------------------------------------------------------------------
# ② on_arm
# ----------------------------------------------------------------------

def test_on_arm() -> None:
    print("\n[on_arm] 카메라가 손목에 — 고정 마커를 여러 자세에서 본다")
    # 손목에 붙은 카메라: 집게 앞쪽으로 30mm, 위로 45mm, 아래를 살짝 본다.
    optical_to_tool = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    truth = Rigid(rot("y", 15.0) @ optical_to_tool, np.array([30.0, 0.0, 45.0]))
    marker = np.array([260.0, -30.0, 40.0])

    frames = [tool_frame(p) for p in sample_poses(12)]
    # 카메라가 본 마커 = (T_base_tool ∘ T_tool_cam)⁻¹ · 마커
    cam_pts = [frame.compose(truth).inverse().apply(marker) for frame in frames]

    fit = solve_on_arm(cam_pts, frames)
    check("잔차 ≈ 0", fit.rms_mm < 1e-6, f"RMS {fit.rms_mm:.2e}mm")
    check("T_tool_cam 평행이동", np.allclose(fit.transform.t, truth.t, atol=1e-6),
          f"t={np.round(fit.transform.t, 3).tolist()}")
    check("T_tool_cam 회전", np.allclose(fit.transform.R, truth.R, atol=1e-9))
    check("마커 위치도 같이 풀렸다",
          fit.marker_base is not None
          and np.allclose(np.array(fit.marker_base), marker, atol=1e-6),
          f"marker={tuple(round(v, 2) for v in (fit.marker_base or ()))}")


# ----------------------------------------------------------------------
# ④ 퇴화 — 거절해야 한다
# ----------------------------------------------------------------------

def test_degenerate() -> None:
    print("\n[거절] 답을 내면 안 되는 입력들")
    truth = truth_camera()
    inv = truth.inverse()

    expect_error("표본 2개는 거절", lambda: solve_fixed(
        [inv.apply(np.array([200.0, 0.0, 100.0])),
         inv.apply(np.array([250.0, 0.0, 100.0]))],
        [[200.0, 0.0, 100.0], [250.0, 0.0, 100.0]]), "표본이")

    line = [np.array([180.0 + 12.0 * i, 0.0, 100.0]) for i in range(8)]
    expect_error("한 직선 위 표본은 거절",
                 lambda: solve_fixed([inv.apply(p) for p in line], line), "직선")

    same = [np.array([220.0, 0.0, 100.0])] * 6
    expect_error("같은 점만 있으면 거절",
                 lambda: solve_fixed([inv.apply(p) for p in same], same), "같은 점")

    poses = sample_poses()
    base_pts = [np.array([p.x, p.y, p.z]) for p in poses]
    expect_error("짝이 안 맞으면 거절",
                 lambda: solve_fixed([inv.apply(p) for p in base_pts[:-1]], base_pts),
                 "짝이 안 맞는다")

    bad = [inv.apply(p) for p in base_pts]
    bad[3] = np.array([float("nan"), 0.0, 0.0])
    expect_error("NaN(깊이 구멍)은 거절", lambda: solve_fixed(bad, base_pts), "NaN")

    frames = [tool_frame(p) for p in sample_poses(4)]
    expect_error("on_arm 표본 4개는 거절",
                 lambda: solve_on_arm([np.zeros(3)] * len(frames), frames), "표본이")


# ----------------------------------------------------------------------
# ⑤ Intrinsics
# ----------------------------------------------------------------------

def test_intrinsics() -> None:
    print("\n[Intrinsics] 픽셀 ↔ 3D 왕복")
    plain = Intrinsics(width=848, height=480, fx=430.0, fy=430.0,
                       ppx=424.0, ppy=240.0, model="none")
    for (u, v, z) in ((424.0, 240.0, 300.0), (600.0, 120.0, 250.0),
                      (100.0, 400.0, 180.0)):
        x, y, zz = plain.deproject(u, v, z)
        back_u = x / zz * plain.fx + plain.ppx
        back_v = y / zz * plain.fy + plain.ppy
        check(f"핀홀 왕복 ({u:.0f},{v:.0f})",
              abs(back_u - u) < 1e-9 and abs(back_v - v) < 1e-9 and abs(zz - z) < 1e-9)

    check("중심 화소는 광축 위", plain.deproject(424.0, 240.0, 300.0)[:2] == (0.0, 0.0))

    # brown_conrady는 왜곡을 **풀어야** 하므로 반복이 돈다 — 수렴하는지 본다.
    warped = Intrinsics(width=848, height=480, fx=430.0, fy=430.0, ppx=424.0,
                        ppy=240.0, model="brown_conrady",
                        coeffs=(0.08, -0.02, 0.001, 0.001, 0.0))
    x, y, z = warped.deproject(700.0, 380.0, 300.0)
    check("brown_conrady가 수렴한다", all(map(math.isfinite, (x, y, z))),
          f"({x:.1f}, {y:.1f}, {z:.1f})mm")
    plain_pt = plain.deproject(700.0, 380.0, 300.0)
    check("왜곡이 실제로 값을 바꾼다",
          abs(x - plain_pt[0]) > 1.0, f"차이 {abs(x - plain_pt[0]):.1f}mm")

    d = plain.as_dict()
    check("dict 왕복", Intrinsics.from_dict(d).as_dict() == d)


# ----------------------------------------------------------------------
# ⑥ Rigid
# ----------------------------------------------------------------------

def test_rigid() -> None:
    print("\n[Rigid] 합성·역·rpy")
    a = Rigid(rot("z", 30.0) @ rot("y", 12.0), np.array([10.0, -5.0, 3.0]))
    b = Rigid(rot("x", -40.0), np.array([-2.0, 7.0, 1.0]))
    p = np.array([100.0, 20.0, -30.0])

    check("합성 = 차례로 적용",
          np.allclose(a.compose(b).apply(p), a.apply(b.apply(p)), atol=1e-9))
    check("역변환이 제자리로",
          np.allclose(a.inverse().apply(a.apply(p)), p, atol=1e-9))
    check("역의 역", np.allclose(a.inverse().inverse().R, a.R, atol=1e-12))
    check("여러 점 한 번에",
          np.allclose(a.apply(np.array([p, p * 2])), np.array([a.apply(p), a.apply(p * 2)])))
    check("dict 왕복", np.allclose(Rigid.from_dict(a.as_dict()).R, a.R, atol=1e-9))

    roll, pitch, yaw = a.rpy_deg
    check("rpy가 원래 회전을 재현",
          np.allclose(rot("z", yaw) @ rot("y", pitch) @ rot("x", roll), a.R, atol=1e-9),
          f"rpy=({roll:.2f}, {pitch:.2f}, {yaw:.2f})°")

    # 짐벌락 근처(pitch≈90°)에서도 죽지 않아야 한다 — 값이 튀는 건 상관없지만
    # 예외가 나거나 NaN이 되면 진단 화면이 통째로 멈춘다.
    lock = Rigid(rot("y", 90.0), np.zeros(3))
    check("짐벌락에서 안 죽는다", all(map(math.isfinite, lock.rpy_deg)),
          f"rpy={tuple(round(v, 1) for v in lock.rpy_deg)}")


# ----------------------------------------------------------------------
# tool_frame — 기구학과 앞뒤가 맞는가
# ----------------------------------------------------------------------

def test_tool_frame() -> None:
    print("\n[tool_frame] 도구 좌표계가 기구학과 같은 것을 가리키는가")
    pose = kin.ToolPose(x=230.0, y=45.0, z=110.0, pitch=-30.0)
    T = tool_frame(pose)
    check("원점이 TCP", np.allclose(T.t, [pose.x, pose.y, pose.z]))
    check("회전이 정규직교", np.allclose(T.R.T @ T.R, np.eye(3), atol=1e-12))
    check("오른손 좌표계", abs(np.linalg.det(T.R) - 1.0) < 1e-12,
          f"det={np.linalg.det(T.R):.12f}")

    approach, lateral, up = kin.tool_axes(pose)
    check("+x = 찌르는 방향(approach)", np.allclose(T.R[:, 0], approach, atol=1e-12))
    check("+y = 왼쪽(lateral)", np.allclose(T.R[:, 1], lateral, atol=1e-12))
    check("+z = 집게 위(up)", np.allclose(T.R[:, 2], up, atol=1e-12))

    # 도구 좌표 이동과 base 좌표 이동이 같은 결과여야 한다.
    da, dl, du = 25.0, -10.0, 8.0
    check("도구 좌표 이동 = 행렬 곱",
          np.allclose(np.array(kin.offset_in_tool_frame(pose, da, dl, du)),
                      T.R @ np.array([da, dl, du]), atol=1e-9))


def main() -> int:
    truth = truth_camera()
    print("심어 둔 진짜 변환 T_base_cam:")
    print(f"  t = {truth.t.tolist()} mm")
    print(f"  rpy = {tuple(round(v, 2) for v in truth.rpy_deg)}°")

    test_fixed()
    test_fixed_noise()
    test_scale_hint()
    test_on_arm()
    test_degenerate()
    test_intrinsics()
    test_rigid()
    test_tool_frame()

    print()
    if FAILED:
        print(f"❌ {len(FAILED)}개 실패 / {PASSED + len(FAILED)}개 중")
        for name in FAILED:
            print(f"   - {name}")
        return 1
    print(f"✅ 전부 통과 ({PASSED}개)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
