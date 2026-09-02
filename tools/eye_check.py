#!/usr/bin/env python3
"""보정 → 통합 전 구간 자체검증 — **D405도 팔도 젯슨도 없이** 개발 PC에서 돈다.

    python tools/eye_check.py

handeye_check.py가 "수식이 맞는가"를 봤다면, 여기는 **배선이 맞는가**를 본다.
가짜 카메라를 실제 파일(/dev/shm 대신 임시 폴더)로 만들어 놓고, 진짜
DepthView·EyeCalibrator·Eye·CartesianArm을 그대로 통과시킨다.

무엇을 확인하나:
  ① 깊이 프레임이 없다/굳었다/구멍이다/너무 멀다 를 **거절**하는가
  ② 삼각대 보정 전 과정(표본 담기 → 풀기 → 파일 저장)이 도는가
  ③ 보정 후 클릭한 픽셀이 **실제 그 자리**의 팔 좌표로 나오는가
  ④ 보정이 없거나 나쁘면 팔을 **안 보내는가**
  ⑤ aim()이 열매 앞 standoff 지점에 실제로 서는가 (한 번에 못 가는 거리를
     여러 번에 나눠 가는지 포함)
  ⑥ 장착 방식을 바꾸면 옛 보정을 버리는가

왜 이걸 만들었나 — 이 경로의 버그는 "팔이 열매 옆 5cm를 집는다"로 나타나고,
그 원인 후보가 카메라·보정·기구학·영점 넷이라 현장에서 가르기가 아주 어렵다.
넷 중 셋을 여기서 미리 지운다.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import sys
import tempfile
import time

for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8")

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import numpy as np  # noqa: E402

from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware.cartesian import CartesianArm, SimJointIO  # noqa: E402
from tomato_picker.hardware.depth_camera import DepthError, DepthView  # noqa: E402
from tomato_picker.hardware.eye import Eye, EyeConfig  # noqa: E402
from tomato_picker.hardware.handeye import CalibrationError, Rigid  # noqa: E402

FAILED: list[str] = []
PASSED = 0
TMP = tempfile.mkdtemp(prefix="eye_check_")

# 젯슨 D405에서 실제로 읽은 값 — 해상도와 깊이 단위를 실물과 같게 둔다.
# (왜곡은 여기서 끄고 핀홀로 둔다. 왜곡 자체는 handeye_check가 따로 본다.)
W, H = 848, 480
FX = FY = 437.9
PPX, PPY = 422.7, 229.3
SCALE_MM = 0.1          # ⚠ D405는 0.1mm/단위다 — 흔한 1mm가 아니다


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
    except (DepthError, CalibrationError, RuntimeError, ValueError) as e:
        check(name, must_contain in str(e), f"메시지='{str(e)[:90]}'")
    else:
        check(name, False, "거절해야 하는데 그냥 통과했다")


# ----------------------------------------------------------------------
# 가짜 카메라 — 진짜 파일을 쓴다 (DepthView가 실제로 읽게)
# ----------------------------------------------------------------------

class FakeCam:
    """base 좌표의 점들을 '카메라가 본 것'으로 만들어 파일에 굽는다."""

    def __init__(self, position, look_at, tag: str = "") -> None:
        C = np.asarray(position, float)
        z = np.asarray(look_at, float) - C
        z /= np.linalg.norm(z)
        x = np.cross(z, [0.0, 0.0, 1.0])
        x /= np.linalg.norm(x)
        y = np.cross(z, x)          # x×y=z 인 오른손계 (+y는 화면 아래)
        self.T_base_cam = Rigid(np.column_stack([x, y, z]), C)
        self.T_cam_base = self.T_base_cam.inverse()
        self.meta = os.path.join(TMP, f"meta{tag}.json")
        self.npy = os.path.join(TMP, f"depth{tag}.npy")
        self.jpg = os.path.join(TMP, f"color{tag}.jpg")
        open(self.jpg, "wb").close()

    def project(self, base_pt) -> tuple[float, float, float]:
        """base 점 → (u, v, 거리mm). 핀홀 그대로."""
        p = self.T_cam_base.apply(np.asarray(base_pt, float))
        return (FX * p[0] / p[2] + PPX, FY * p[1] / p[2] + PPY, float(p[2]))

    def bake(self, base_pts, ts: float | None = None, blob: int = 4) -> list:
        """그 점들이 보이는 깊이 프레임을 굽고 픽셀 좌표를 돌려준다."""
        depth = np.zeros((H, W), dtype=np.uint16)
        pix = []
        for pt in base_pts:
            u, v, z = self.project(pt)
            ui, vi = int(round(u)), int(round(v))
            depth[max(0, vi - blob): vi + blob + 1,
                  max(0, ui - blob): ui + blob + 1] = int(round(z / SCALE_MM))
            pix.append((ui, vi, z))
        self.write(depth, ts)
        return pix

    def write(self, depth: np.ndarray, ts: float | None = None) -> None:
        with open(self.npy, "wb") as f:
            np.save(f, depth)
        meta = {
            "seq": 1, "ts": time.time() if ts is None else ts, "serial": "FAKE",
            "width": W, "height": H,
            "intrinsics": {"width": W, "height": H, "fx": FX, "fy": FY,
                           "ppx": PPX, "ppy": PPY, "model": "none",
                           "coeffs": [0, 0, 0, 0, 0]},
            "depth_scale_mm": SCALE_MM, "valid_frac": float((depth > 0).mean()),
            "near_frac": 0.5, "median_mm": 400.0,
        }
        with open(self.meta, "w", encoding="utf-8") as f:
            json.dump(meta, f)

    def write_unaligned(self) -> None:
        """Astra처럼 **컬러가 정렬 안 된** meta를 굽는다.

        min/max도 함께 실어, 읽는 쪽이 config가 아니라 meta를 보는지 확인한다
        (같은 깊이 프레임인데 카메라가 다르면 거절 결과가 달라져야 한다).
        """
        depth = np.full((H, W), int(round(400.0 / SCALE_MM)), dtype=np.uint16)
        self.write(depth)
        with open(self.meta, encoding="utf-8") as f:
            meta = json.load(f)
        meta.update({"camera": "astra", "color_aligned": False,
                     "min_mm": 600.0, "max_mm": 4000.0})
        with open(self.meta, "w", encoding="utf-8") as f:
            json.dump(meta, f)

    def view(self, camera: str = "d405") -> DepthView:
        return DepthView(self.meta, self.npy, self.jpg, camera=camera)


# ----------------------------------------------------------------------
# 가짜 팔
# ----------------------------------------------------------------------

def fresh_arm() -> tuple[CartesianArm, SimJointIO]:
    io_ = SimJointIO()
    path = os.path.join(TMP, f"cart_{time.time_ns()}.json")
    arm = CartesianArm(io_, path=path)
    arm.config.set_zero({j: 0.0 for j in kin.JOINTS})
    return arm, io_


def put(arm: CartesianArm, io_: SimJointIO, degrees: dict) -> None:
    io_.joints.update(arm.to_norms(degrees))


# 보정 표본을 찍을 자세들 — **서로 멀찍이** 흩어야 한다(한 뭉치면 거절당한다).
POSES = [
    {"shoulder_pan": 0, "shoulder_lift": 75, "elbow_flex": -85, "wrist_flex": -20},
    {"shoulder_pan": 30, "shoulder_lift": 60, "elbow_flex": -70, "wrist_flex": -10},
    {"shoulder_pan": -30, "shoulder_lift": 90, "elbow_flex": -100, "wrist_flex": -30},
    {"shoulder_pan": 15, "shoulder_lift": 95, "elbow_flex": -60, "wrist_flex": -40},
    {"shoulder_pan": -15, "shoulder_lift": 55, "elbow_flex": -95, "wrist_flex": 5},
    {"shoulder_pan": 40, "shoulder_lift": 80, "elbow_flex": -90, "wrist_flex": -25},
    {"shoulder_pan": -40, "shoulder_lift": 70, "elbow_flex": -75, "wrist_flex": -15},
]
CAM = FakeCam(position=(500.0, 0.0, 400.0), look_at=(200.0, 0.0, 100.0))
# Astra 대역(60~400cm)에 있는 카메라. D405 자리(위)에서는 Astra가 아무것도
# 못 본다 — 그게 두 카메라를 나눠 둔 이유고, 아래 테스트가 그걸 확인한다.
FAR = FakeCam(position=(1400.0, 0.0, 900.0), look_at=(200.0, 0.0, 100.0), tag="_far")


# ----------------------------------------------------------------------
# ① 깊이를 못 믿을 때 거절하는가
# ----------------------------------------------------------------------

def test_depth_guards() -> None:
    print("\n[깊이] 못 믿는 값을 거절하는가")
    view = CAM.view()
    CAM.bake([(200.0, 0.0, 100.0)])
    u, v, z = CAM.project((200.0, 0.0, 100.0))

    p = view.point_at(u, v)
    truth = CAM.T_cam_base.apply((200.0, 0.0, 100.0))
    err = float(np.linalg.norm(np.array(p) - truth))
    check("보이는 점은 정확히 역투영된다", err < 1.5, f"{err:.2f}mm (거리 {z:.0f}mm)")

    expect_error("구멍(깊이 0)은 거절", lambda: view.point_at(50, 50), "비어 있다")
    expect_error("프레임 밖은 거절", lambda: view.point_at(2000, 10), "밖이다")

    # 굳은 화면 — 발행기가 죽었는데 파일은 남아 있는 상황
    CAM.bake([(200.0, 0.0, 100.0)], ts=time.time() - 30)
    expect_error("굳은 프레임으로는 안 움직인다", lambda: view.point_at(u, v), "30초 전")
    check("status가 이유를 말한다", "멈춰" in (view.status().get("why") or ""),
          str(view.status().get("why"))[:60])

    # 너무 멀다 — 삼각대를 2m 뒤에 뒀을 때가 정확히 이 경우다
    far = np.zeros((H, W), dtype=np.uint16)
    far[100:120, 100:120] = int(2000 / SCALE_MM)
    CAM.write(far)
    expect_error("너무 먼 깊이는 거절", lambda: view.point_at(110, 110), "너무 멀다")

    near = np.zeros((H, W), dtype=np.uint16)
    near[100:120, 100:120] = int(30 / SCALE_MM)
    CAM.write(near)
    expect_error("너무 가까운 깊이는 거절", lambda: view.point_at(110, 110), "너무 가깝다")

    missing = DepthView(os.path.join(TMP, "nope.json"), CAM.npy, CAM.jpg)
    check("발행기가 없으면 status가 알려준다",
          missing.status().get("ok") is False, str(missing.status().get("why"))[:50])


# ----------------------------------------------------------------------
# ② ③ 삼각대 보정 전 과정
# ----------------------------------------------------------------------

def calibrate_fixed() -> tuple[Eye, CartesianArm, SimJointIO]:
    arm, io_ = fresh_arm()
    cfg = EyeConfig(path=os.path.join(TMP, f"eye_{time.time_ns()}.json"))
    cfg.set_mount("fixed")
    eye = Eye(CAM.view(), arm, cfg)
    for deg in POSES:
        put(arm, io_, deg)
        tcp = arm.pose()
        (u, v, _), = CAM.bake([(tcp.x, tcp.y, tcp.z)])
        eye.calibrator.add_sample(u, v)
    return eye, arm, io_


def test_fixed_flow() -> None:
    print("\n[보정] 삼각대 — 표본 담기 → 풀기 → 저장")
    arm, io_ = fresh_arm()
    cfg = EyeConfig(path=os.path.join(TMP, "eye_flow.json"))
    cfg.set_mount("fixed")
    eye = Eye(CAM.view(), arm, cfg)

    expect_error("표본이 모자라면 못 푼다", eye.calibrator.solve, "표본이 0개다")

    for i, deg in enumerate(POSES):
        put(arm, io_, deg)
        tcp = arm.pose()
        (u, v, _), = CAM.bake([(tcp.x, tcp.y, tcp.z)])
        msg = eye.calibrator.add_sample(u, v)
        if i == 0:
            check("표본 메시지가 거리와 좌표를 말한다",
                  "거리" in msg and "집게 좌표" in msg, msg[:80])
    check(f"표본 {len(POSES)}개 담김", len(eye.calibrator.samples()) == len(POSES))

    msg = eye.calibrator.solve()
    check("잔차가 충분히 작다", cfg.rms_mm is not None and cfg.rms_mm < 3.0, msg)
    check("보정이 저장됐다", os.path.exists(cfg.path) and cfg.has_calibration)

    # 심어둔 카메라 위치를 되찾았는가 — 사람이 화면에서 눈으로 보는 값이다
    got = np.array(cfg.snapshot()["camera_at"])
    err = float(np.linalg.norm(got - np.array([500.0, 0.0, 400.0])))
    check("카메라 위치 복원", err < 8.0, f"{list(got)} (오차 {err:.1f}mm)")

    # 파일에서 새로 읽어도 같은가 (재시작 후에도 살아 있는가)
    again = EyeConfig(path=cfg.path)
    check("재시작 후에도 보정이 남는다",
          again.has_calibration and abs((again.rms_mm or 9) - (cfg.rms_mm or 0)) < 1e-6)


def test_pixel_to_base() -> None:
    print("\n[통합] 클릭한 픽셀이 실제 그 자리로 나오는가")
    eye, arm, io_ = calibrate_fixed()
    eye.calibrator.solve()

    for truth in [(200.0, 0.0, 100.0), (230.0, 60.0, 150.0), (190.0, -70.0, 60.0)]:
        (u, v, z) = CAM.bake([truth])[0]
        got = eye.pixel_to_base(u, v)
        err = math.dist(got, truth)
        check(f"픽셀→팔좌표 {truth}", err < 3.0,
              f"오차 {err:.2f}mm (거리 {z:.0f}mm)")


def test_refusals() -> None:
    print("\n[통합] 믿을 수 없으면 팔을 안 보내는가")
    arm, io_ = fresh_arm()
    cfg = EyeConfig(path=os.path.join(TMP, "eye_none.json"))
    cfg.set_mount("fixed")
    eye = Eye(CAM.view(), arm, cfg)
    CAM.bake([(200.0, 0.0, 100.0)])
    u, v, _ = CAM.project((200.0, 0.0, 100.0))
    expect_error("보정 없이 aim은 거절", lambda: eye.aim(u, v), "보정이 없습니다")
    expect_error("보정 없이 변환도 거절", lambda: eye.pixel_to_base(u, v), "보정이 없습니다")

    # 영점이 없으면 표본을 담는 것부터 막는다
    io2 = SimJointIO()
    arm2 = CartesianArm(io2, path=os.path.join(TMP, "cart_nozero.json"))
    eye2 = Eye(CAM.view(), arm2, EyeConfig(path=os.path.join(TMP, "eye2.json")))
    expect_error("영점 없이 표본 담기는 거절",
                 lambda: eye2.calibrator.add_sample(u, v), "영점이 없습니다")

    # 잔차가 나쁜 보정은 저장은 되지만 aim은 막힌다
    eye3, arm3, io3 = calibrate_fixed()
    bad = eye3.calibrator.samples()
    eye3.calibrator.clear()
    for k, s in enumerate(bad):
        s2 = dict(s)
        if k % 2:                      # 절반을 크게 흔들어 잔차를 망가뜨린다
            s2["cam"] = [c + 40.0 for c in s["cam"]]
        eye3.calibrator._samples.append(s2)
    msg = eye3.calibrator.solve()
    check("나쁜 보정은 경고를 단다", "⚠" in msg, msg[:80])
    expect_error("나쁜 보정으로는 aim 거절", lambda: eye3.aim(u, v), "잔차가")


def test_aim() -> None:
    print("\n[통합] aim — 열매 앞 standoff 지점에 서는가")
    eye, arm, io_ = calibrate_fixed()
    eye.calibrator.solve()

    fruit = (200.0, 0.0, 100.0)
    (u, v, _), = CAM.bake([fruit])
    put(arm, io_, POSES[0])            # 멀리서 출발 — 한 번에 못 가는 거리
    start = arm.pose()
    msg = eye.aim(u, v)

    plan = eye.approach_pose(*fruit)
    now = arm.pose()
    err = math.dist((now.x, now.y, now.z), (plan["x"], plan["y"], plan["z"]))
    check("standoff 지점에 도착", err < 3.0, f"오차 {err:.2f}mm · {msg[:60]}")

    d_fruit = math.dist((now.x, now.y, now.z), fruit)
    check("열매를 밀지 않는다(앞에서 멈춘다)", 35.0 < d_fruit < 55.0,
          f"열매까지 {d_fruit:.0f}mm (standoff {plan['standoff']:.0f}mm)")
    check("집게가 열매 쪽을 본다", abs(now.pitch - plan["pitch"]) < 1.0,
          f"pitch {now.pitch:.1f}°")

    travel = math.dist((start.x, start.y, start.z), (now.x, now.y, now.z))
    check("한 번에 못 가는 거리를 나눠 갔다", "나눠 이동" in msg and travel > 0,
          f"이동 {travel:.0f}mm, 쓰기 {io_.writes}회")


def test_mount_switch() -> None:
    print("\n[보정] 장착을 바꾸면 옛 보정을 버리는가")
    eye, arm, io_ = calibrate_fixed()
    eye.calibrator.solve()
    check("바꾸기 전에는 보정이 있다", eye.config.has_calibration)
    eye.config.set_mount("on_arm")
    check("바꾸면 보정이 사라진다", not eye.config.has_calibration,
          eye.config.snapshot().get("note", ""))
    check("필요 표본 수도 손목 기준으로 바뀐다", eye.calibrator.needed() == 8,
          f"{eye.calibrator.needed()}개")
    expect_error("모르는 장착은 거절", lambda: eye.config.set_mount("헬멧"), "mount는")


def test_snapshot() -> None:
    print("\n[화면] 대시보드가 그릴 상태")
    arm, io_ = fresh_arm()
    eye = Eye(CAM.view(), arm, EyeConfig(path=os.path.join(TMP, "eye_snap.json")))
    CAM.bake([(200.0, 0.0, 100.0)])
    snap = eye.snapshot()
    check("영점이 있으면 blocker가 없다", "blocker" not in snap, str(snap.get("blocker")))
    check("카메라 상태가 들어 있다", snap["camera"]["ok"] is True)
    check("필요 표본 수를 알려준다", snap["needed"] == 5, str(snap["needed"]))

    io2 = SimJointIO()
    arm2 = CartesianArm(io2, path=os.path.join(TMP, "cart_snap2.json"))
    eye2 = Eye(CAM.view(), arm2, EyeConfig(path=os.path.join(TMP, "eye_snap2.json")))
    check("영점이 없으면 그걸 먼저 말한다",
          "영점" in (eye2.snapshot().get("blocker") or ""),
          str(eye2.snapshot().get("blocker"))[:50])


# ----------------------------------------------------------------------
# 카메라가 둘일 때 (2026-09-01, Astra Pro 추가)
#
# 여기서 지키려는 사고 하나: **한쪽을 보정하면 다른 쪽이 조용히 사라지는 것.**
# 파일이 하나(~/arm_eye.json)라 칸을 안 나누면 그렇게 된다. 그러면 화면은
# "보정됨"이라 말하는데 팔은 엉뚱한 데로 간다 — 이 저장소가 가장 싫어하는 모양.
# ----------------------------------------------------------------------

def test_two_cameras() -> None:
    print("\n[카메라 둘] Astra Pro가 붙으면서 보정도 둘이 됐다")
    from tomato_picker.config import DEPTH_CAMERAS
    from tomato_picker.hardware.depth_camera import available_cameras, camera_spec

    check("카메라 목록에 d405와 astra가 있다",
          {"d405", "astra"} <= set(DEPTH_CAMERAS), str(tuple(DEPTH_CAMERAS)))
    expect_error("모르는 카메라 이름은 거절한다 — 기본값으로 삼키지 않는다",
                 lambda: camera_spec("astro"), "모르는 깊이 카메라")
    check("발행 중인 카메라를 파일 존재로 판단한다",
          isinstance(available_cameras(), list))

    d405, astra = camera_spec("d405"), camera_spec("astra")
    check("두 카메라의 /dev/shm 경로가 안 겹친다",
          d405["meta"] != astra["meta"] and d405["depth"] != astra["depth"])
    # 겹치는 구간(60~90cm)은 있지만 쓰는 자리가 다르다 — 가까운 쪽/먼 쪽.
    check("가까운 쪽과 먼 쪽으로 갈린다 — 그래서 둘 다 필요하다",
          d405["min_mm"] < astra["min_mm"] and d405["max_mm"] < astra["max_mm"],
          f"d405 {d405['min_mm']:.0f}~{d405['max_mm']:.0f} · "
          f"astra {astra['min_mm']:.0f}~{astra['max_mm']:.0f}mm")
    check("D405는 Astra의 최소거리 아래를 본다(Astra가 못 보는 구간)",
          d405["min_mm"] < astra["min_mm"])
    check("Astra는 D405의 최대거리 너머를 본다(D405가 못 보는 구간)",
          astra["max_mm"] > d405["max_mm"])
    check("Astra 컬러는 정렬 안 됨으로 표시돼 있다",
          d405["color_aligned"] is True and astra["color_aligned"] is False)

    # --- 보정 칸이 카메라마다 따로인가 (이 테스트의 요점) ---
    path = os.path.join(TMP, "eye_two.json")
    arm, io_ = fresh_arm()
    eye_d = Eye(CAM.view("d405"), arm, EyeConfig(path=path, camera="d405"))
    eye_a = Eye(FAR.view("astra"), arm, EyeConfig(path=path, camera="astra"))
    check("Eye가 자기 카메라 이름을 안다",
          eye_d.camera == "d405" and eye_a.camera == "astra")
    near_as_astra = Eye(CAM.view("astra"), arm, EyeConfig(path=path, camera="astra"))

    def fill(eye, cam):
        for deg in POSES:
            put(arm, io_, deg)
            tcp = arm.pose()
            (u, v, _), = cam.bake([(tcp.x, tcp.y, tcp.z)])
            eye.calibrator.add_sample(u, v)
        return eye.calibrator.solve()

    # ⚠ 가까운 카메라 자리에서 Astra로 찍으면 **전부 거절**돼야 한다.
    #   같은 프레임인데 카메라가 다르면 결과가 달라지는 것 — 그게 요점이다.
    CAM.bake([(200.0, 0.0, 100.0)])
    u0, v0, _ = CAM.project((200.0, 0.0, 100.0))
    expect_error("D405 자리(35cm)의 표본을 Astra로는 못 담는다",
                 lambda: near_as_astra.calibrator.add_sample(u0, v0), "너무 가깝다")

    fill(eye_d, CAM)
    check("d405 보정이 저장됐다", eye_d.config.has_calibration)
    check("astra는 아직 보정이 없다 — 같은 파일이지만 칸이 다르다",
          not EyeConfig(path=path, camera="astra").has_calibration)

    fill(eye_a, FAR)
    reread = EyeConfig(path=path, camera="d405")
    check("astra를 잡아도 d405 보정이 그대로다",
          reread.has_calibration and reread.rms_mm is not None,
          f"d405 잔차 {reread.rms_mm}mm")
    EyeConfig(path=path, camera="astra").clear()
    check("astra 보정을 지워도 d405는 남는다",
          EyeConfig(path=path, camera="d405").has_calibration
          and not EyeConfig(path=path, camera="astra").has_calibration)
    check("스냅샷이 어느 카메라 것인지 말한다",
          EyeConfig(path=path, camera="astra").snapshot().get("camera") == "astra")

    # --- 옛 파일(칸 없는 평평한 형식)을 그대로 읽는가 ---
    #     ROS 쪽(tomato_handeye/store.py)도 이 자리를 읽는다. 형식을 옮기면
    #     둘 다 조용히 못 읽으므로, 기본 카메라는 계속 최상위를 쓴다.
    legacy = os.path.join(TMP, "eye_legacy.json")
    with open(legacy, "w", encoding="utf-8") as f:
        json.dump({"mount": "on_arm",
                   "transform": {"R": np.eye(3).tolist(), "t": [1.0, 2.0, 3.0]},
                   "rms_mm": 3.0}, f)
    old = EyeConfig(path=legacy)          # 이름을 안 대면 d405 = 최상위
    check("옛 형식 파일(최상위)을 기본 카메라가 그대로 읽는다",
          old.has_calibration and old.mount == "on_arm" and old.rms_mm == 3.0)

    # --- meta가 config를 이기는가 · 정렬 안 된 컬러를 거절하는가 ---
    CAM.write_unaligned()
    view = CAM.view("astra")
    check("meta의 color_aligned=false를 읽는다", view.color_aligned is False)
    check("meta의 min/max를 읽는다",
          view.min_mm == 600.0 and view.max_mm == 4000.0,
          f"{view.min_mm:.0f}~{view.max_mm:.0f}mm")
    expect_error("정렬 안 된 컬러에서 열매를 찾자고 하면 거절한다",
                 lambda: Eye(view, arm, EyeConfig(path=path, camera="astra")).fruits(),
                 "정렬")
    expect_error("같은 프레임이라도 Astra 기준으로는 너무 가깝다고 거절한다",
                 lambda: view.depth_mm_at(PPX, PPY), "너무 가깝다")

    # meta에 한계가 없으면 config가 대신 답한다 — **카메라마다 다르게.**
    # (같은 프레임, 같은 파일, 이름만 다른 두 창. 한쪽만 통과해야 한다.)
    CAM.write(np.full((H, W), int(round(400.0 / SCALE_MM)), dtype=np.uint16))
    check("meta에 한계가 없으면 config로 떨어진다 — D405는 40cm를 받는다",
          abs(CAM.view("d405").depth_mm_at(PPX, PPY) - 400.0) < 1.0)
    expect_error("같은 프레임을 Astra 이름으로 보면 거절한다",
                 lambda: CAM.view("astra").depth_mm_at(PPX, PPY), "너무 가깝다")

    st = CAM.view("astra").status()
    check("상태가 어느 카메라인지 말한다", st.get("label") == "Astra", str(st.get("label")))
    check("정렬 안 된 카메라는 상태에 그 사실을 단다", "정렬" in (st.get("note") or ""))
    check("상태에 그 카메라의 유효 거리가 실린다",
          st.get("min_mm") == 600.0 and st.get("max_mm") == 4000.0,
          f"{st.get('min_mm')}~{st.get('max_mm')}mm")


def main() -> int:
    print(f"보정→통합 자체검증 — 하드웨어 없이 (임시폴더 {TMP})")
    try:
        test_depth_guards()
        test_fixed_flow()
        test_pixel_to_base()
        test_refusals()
        test_aim()
        test_mount_switch()
        test_snapshot()
        test_two_cameras()
    finally:
        shutil.rmtree(TMP, ignore_errors=True)

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
