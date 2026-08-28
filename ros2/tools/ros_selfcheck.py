#!/usr/bin/env python3
"""ROS 계통 자체검증 — **ROS도 젯슨도 카메라도 없이** 개발 PC에서 돈다.

    python ros2/tools/ros_selfcheck.py

이 저장소의 규칙이다: 숫자가 맞는지를 확인하려고 매번 젯슨에 올려 팔을 흔들지
않는다([`kinematics.py`](../../src/tomato_picker/hardware/kinematics.py) ·
[`handeye.py`](../../src/tomato_picker/hardware/handeye.py)와 같은 이유).
그래서 각 패키지의 계산 부분은 `rclpy`를 import하지 않는 순수 모듈로 떼어 놨고,
여기서 그것들을 시험한다.

무엇을 확인하나:
  ① **rclpy 격리** — `*_node.py`가 아닌 파일에 rclpy가 섞이지 않았는가
     (섞이는 순간 이 검증 자체가 불가능해진다. 그래서 첫 번째다)
  ② **URDF ↔ 기구학 일치** — xacro의 관절 원점·축으로 계산한 TCP가
     kinematics.forward()와 같은 곳을 가리키는가. **부호 하나만 틀려도 여기서 걸린다.**
  ③ **링크 길이 일치** — so101_geometry.yaml과 ArmGeometry 기본값
  ④ **보드 계약** — 단위 변환·체크섬·정지마찰 feedforward·조용한 폴백 거절
  ⑤ **깊이 읽기** — 구멍·잎 섞임·가장자리를 실제로 거절하는가
  ⑥ **TF 수학** — 쿼터니언 왕복, camera_link 재타깃

⚠ 여기가 통과해도 로봇이 도는 건 아니다. 여기서 걸리는 종류의 실수(부호,
   라디안/도, mm/m, 좌표계 부모)를 **실물에서 배우지 않게** 하는 것이 전부다.
"""

from __future__ import annotations

import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8")

HERE = os.path.dirname(os.path.abspath(__file__))
ROS2 = os.path.dirname(HERE)
REPO = os.path.dirname(ROS2)
SRC = os.path.join(ROS2, "src")

sys.path.insert(0, os.path.join(REPO, "src"))
for pkg in ("tomato_bridge", "tomato_perception", "tomato_handeye"):
    sys.path.insert(0, os.path.join(SRC, pkg))

from tomato_picker.hardware import kinematics as kin  # noqa: E402
from tomato_picker.hardware.handeye import Intrinsics, Rigid  # noqa: E402

from tomato_bridge import board_contract as bc  # noqa: E402
from tomato_bridge.arm_source import EXTRA_JOINTS, JOINT_NAMES  # noqa: E402
from tomato_handeye import store  # noqa: E402
from tomato_perception.fruit3d import (  # noqa: E402
    MAX_SPREAD_MM, Blob, read_blob,
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


# ----------------------------------------------------------------------
# ① rclpy 격리
# ----------------------------------------------------------------------

def test_rclpy_isolation() -> None:
    print("\n[격리] rclpy는 *_node.py에만 있어야 한다")
    offenders = []
    scanned = 0
    for root, _dirs, files in os.walk(SRC):
        for f in files:
            if not f.endswith(".py") or f.endswith("_node.py"):
                continue
            if f in ("setup.py",) or "launch" in root:
                continue
            path = os.path.join(root, f)
            scanned += 1
            with open(path, encoding="utf-8") as fh:
                body = fh.read()
            if re.search(r"^\s*(import rclpy|from rclpy)", body, re.MULTILINE):
                offenders.append(os.path.relpath(path, REPO))
    check("순수 모듈에 rclpy가 없다", not offenders,
          f"{scanned}개 검사" if not offenders else "오염: " + ", ".join(offenders))


# ----------------------------------------------------------------------
# ②③ URDF
# ----------------------------------------------------------------------

def _geometry_yaml() -> dict:
    path = os.path.join(SRC, "tomato_description", "config", "so101_geometry.yaml")
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _xacro_joints() -> dict:
    """so101_arm.xacro에서 관절의 origin/axis를 읽는다.

    xacro를 실행하지 않는다(설치돼 있지 않아도 돌아야 하니까). 매크로 안의
    표현식이 `${l1}` 처럼 단순해서 직접 풀 수 있고, **단순하지 않으면 여기서
    터진다** — 그것도 신호다(URDF가 손으로 읽을 수 없을 만큼 복잡해졌다는).
    """
    path = os.path.join(SRC, "tomato_description", "urdf", "so101_arm.xacro")
    tree = ET.parse(path)
    ns = {"xacro": "http://www.ros.org/wiki/xacro"}
    macro = tree.getroot().find("xacro:macro", ns)
    out = {}
    for joint in macro.findall("joint"):
        name = joint.get("name", "").replace("${prefix}", "")
        origin = joint.find("origin")
        axis = joint.find("axis")
        out[name] = {
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link").replace("${prefix}", ""),
            "child": joint.find("child").get("link").replace("${prefix}", ""),
            "xyz": (origin.get("xyz") if origin is not None else "0 0 0"),
            "axis": (axis.get("xyz") if axis is not None else None),
        }
    return out


def _resolve(expr: str, values: dict) -> float:
    """`${l1}` / `0` / `${l3 - 0.03}` 같은 조각을 숫자로."""
    expr = expr.strip()
    m = re.fullmatch(r"\$\{([^}]*)\}", expr)
    if not m:
        return float(expr)
    body = m.group(1)
    for key, val in values.items():
        body = re.sub(rf"\b{key}\b", repr(val), body)
    return float(eval(body, {"__builtins__": {}}, {}))  # noqa: S307 - 우리가 쓴 식만 온다


def _rotation(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    """로드리게스 — URDF의 axis/angle을 회전행렬로."""
    a = np.array(axis, dtype=float)
    a = a / (np.linalg.norm(a) or 1.0)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def _urdf_fk(joints_deg: dict, geom: kin.ArmGeometry) -> np.ndarray:
    """xacro의 사슬을 그대로 따라가며 tool0 위치를 계산한다(m 단위 → mm로 돌려줌)."""
    spec = _xacro_joints()
    values = {"z0": geom.z0 / 1000.0, "d0": geom.d0 / 1000.0, "l1": geom.l1 / 1000.0,
              "l2": geom.l2 / 1000.0, "l3": geom.l3 / 1000.0}
    order = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
             "wrist_roll", "tool0_fixed"]

    R = np.eye(3)
    t = np.zeros(3)
    for name in order:
        j = spec[name]
        offset = np.array([_resolve(p, values) for p in j["xyz"].split()])
        t = t + R @ offset
        if j["type"] == "revolute":
            axis = tuple(float(v) for v in j["axis"].split())
            R = R @ _rotation(axis, math.radians(joints_deg.get(name, 0.0)))
    return t * 1000.0


def test_geometry_matches() -> None:
    print("\n[기하] yaml ↔ ArmGeometry")
    cfg = _geometry_yaml()["arm"]
    default = kin.ArmGeometry()
    for key in ("z0", "d0", "l1", "l2", "l3"):
        check(f"{key} 일치", abs(float(cfg[key]) - getattr(default, key)) < 1e-9,
              f"yaml {cfg[key]} vs 코드 {getattr(default, key)}")


def test_urdf_matches_kinematics() -> None:
    print("\n[URDF] xacro 사슬 ↔ kinematics.forward()")
    geom = kin.ArmGeometry()
    spec = _xacro_joints()

    # 사슬이 실제로 이어져 있는가 — 부모/자식이 어긋나면 TF 트리가 갈라진다.
    chain = ["arm_base", "pan_link", "upper_arm_link", "forearm_link",
             "wrist_link", "gripper_link", "tool0"]
    names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
             "wrist_roll", "tool0_fixed"]
    linked = all(spec[n]["parent"] == chain[i] and spec[n]["child"] == chain[i + 1]
                 for i, n in enumerate(names))
    check("사슬이 끊기지 않았다", linked, " → ".join(chain))

    check("관절 이름이 kinematics와 같다",
          tuple(names[:5]) == kin.JOINTS,
          f"URDF {names[:5]} vs kinematics {list(kin.JOINTS)}")
    check("브리지가 쓰는 이름도 같다", JOINT_NAMES == kin.JOINTS)
    check("URDF에만 있는 관절도 발행 목록에 있다",
          "gripper" in EXTRA_JOINTS, f"{EXTRA_JOINTS}")

    # 자세를 여러 개 넣어 본다. 기준 자세(전부 0)만 맞춰 놓고 축 부호가 틀린
    # 경우가 실제로 흔하다 — 그래서 **꺾인 자세**를 반드시 섞는다.
    cases = [
        ("전부 0 (앞으로 수평)", {}),
        ("어깨만 30°", {"shoulder_lift": 30.0}),
        ("팔꿈치만 -45°", {"elbow_flex": -45.0}),
        ("손목만 20°", {"wrist_flex": 20.0}),
        ("pan 40°", {"shoulder_pan": 40.0}),
        ("곧게 위로(교시 자세)", {"shoulder_lift": 90.0}),
        ("섞은 자세", {"shoulder_pan": -25.0, "shoulder_lift": 55.0,
                     "elbow_flex": -70.0, "wrist_flex": -35.0}),
    ]
    for label, joints in cases:
        expected = kin.forward(joints, geom)
        got = _urdf_fk(joints, geom)
        want = np.array([expected.x, expected.y, expected.z])
        err = float(np.linalg.norm(got - want))
        check(f"TCP 일치 — {label}", err < 1e-6,
              f"URDF {np.round(got, 2).tolist()} vs FK {np.round(want, 2).tolist()} "
              f"(차이 {err:.4f}mm)")

    # 축 부호를 일부러 뒤집으면 **걸려야** 한다. 검사가 검사를 하고 있는지 본다.
    flipped = dict(_xacro_joints())
    check("검사가 부호 오류를 실제로 잡는다",
          flipped["shoulder_lift"]["axis"] == "0 -1 0",
          "lift 축이 0 -1 0이 아니면 팔이 위가 아니라 아래로 든다")


# ----------------------------------------------------------------------
# ④ 보드 계약
# ----------------------------------------------------------------------

def test_board_contract() -> None:
    print("\n[보드계약] 단위·체크섬·정지마찰·거절")

    check("체크섬 (C 350 0 0)", bc.checksum("C 350 0 0") == "55",
          f"XOR={bc.checksum('C 350 0 0')}")
    check("프레이밍", bc.framed("S") == b"S*53\n", str(bc.framed("S")))

    check("m/s → mm/s", bc.to_physical(0.35, -0.2, 0.0)[:2] == (350, -200))
    # 90°/s = 1.5708 rad/s → 90000 mdeg/s
    check("rad/s → mdeg/s", bc.to_physical(0, 0, math.radians(90.0))[2] == 90000,
          f"{bc.to_physical(0, 0, math.radians(90.0))[2]}")

    stop = bc.plan(0, 0, 0)
    check("정지는 S (0을 보내는 것과 다르다)", stop.payload == "S" and stop.duty is None)

    nan = bc.plan(float("nan"), 0, 0)
    check("NaN 지령은 거절하고 세운다", nan.rejected and nan.payload == "S", nan.reason[:50])

    # 정지마찰: 아주 작은 지령도 문턱을 넘어야 한다 (안 그러면 물리적으로 0이다)
    calib = bc.DutyCalib()
    tiny = bc.plan(0.002, 0, 0, calib=calib)
    check("작은 지령도 문턱 위로 나간다", tiny.duty[0] >= calib.ks,
          f"2mm/s → duty {tiny.duty[0]} (문턱 {calib.ks})")
    check("0은 0이다", calib.duty_linear(0.0) == 0)
    check("부호가 보존된다", calib.duty_linear(-100.0) == -calib.duty_linear(100.0))
    check("duty는 천장을 안 넘는다", calib.duty_linear(99999.0) == calib.max_duty,
          f"{calib.duty_linear(99999.0)}")
    check("클수록 커진다(단조)",
          calib.duty_linear(50) < calib.duty_linear(150) < calib.duty_linear(400))

    check("실측 전에는 그렇다고 말한다",
          any("실측이 아니다" in n for n in bc.plan(0.3, 0, 0).notes))
    check("실측했다면 잔소리 안 한다",
          not any("실측이 아니다" in n
                  for n in bc.plan(0.1, 0, 0,
                                   calib=bc.DutyCalib(measured=True)).notes))

    v2 = bc.Caps.parse("cap proto=2 fw=3.0.0 board=stm32f411 id=A3F2C918 "
                       "units=1 closed_loop=1 calib=1 vmax=800 vymax=600 wmax=180000")
    check("cap 파싱", v2.units and v2.calib and v2.vmax_mms == 800, str(v2.board))
    check("cap 못 알아들으면 예외", _raises(lambda: bc.Caps.parse("hb 1234 rx=5")))
    check("모르는 필드는 무시", bc.Caps.parse("cap proto=2 quantum=42").proto == 2)

    physical = bc.plan(0.3, 0, 0, caps=v2)
    check("물리 단위 보드는 C를 받는다", physical.payload == "C 300 0 0", physical.payload)

    import dataclasses
    nocalib = bc.plan(0.3, 0, 0, caps=dataclasses.replace(v2, calib=False))
    check("calib=0이면 duty로 몰래 안 내려간다",
          nocalib.rejected and nocalib.duty is None, nocalib.reason[:60])

    clamped = bc.plan(2.0, 0, 0, caps=v2)
    check("보드가 말한 상한에서 자른다", clamped.physical[0] == 800, str(clamped.notes))
    check("상한을 모르면 안 자른다", bc.plan(2.0, 0, 0).physical[0] == 2000)

    estop = bc.plan(0.3, 0, 0, estop=True)
    check("비상정지 중에는 아무것도 안 나간다",
          estop.rejected and estop.payload == "S")

    signs = bc.plan(0.3, 0.2, 0, signs=bc.AxisSigns(vy=-1))
    check("축 부호가 먹는다", signs.physical[:2] == (300, -200), str(signs.physical))

    big = bc.plan(0.6, 0.6, 3.0)
    check("세 축이 포화되면 경고한다",
          any("포화" in n for n in big.notes), " | ".join(big.notes)[:80])


def _raises(fn) -> bool:
    try:
        fn()
    except Exception:  # noqa: BLE001
        return True
    return False


# ----------------------------------------------------------------------
# ⑤ 깊이 읽기
# ----------------------------------------------------------------------

def _scene(depth_value=300.0, size=200):
    """가짜 깊이 영상 하나 — 배경은 800mm, 열매는 300mm 원."""
    depth = np.full((size, size), 800.0)
    yy, xx = np.ogrid[:size, :size]
    disk = (xx - 100) ** 2 + (yy - 100) ** 2 <= 30 ** 2
    depth[disk] = depth_value
    return depth, disk


INTR = Intrinsics(width=200, height=200, fx=400.0, fy=400.0, ppx=100.0, ppy=100.0)


def test_fruit3d() -> None:
    print("\n[깊이] 못 믿을 깊이를 좌표로 바꿔 주지 않는가")
    blob = Blob(u=100.0, v=100.0, radius_px=30.0, pixels=2827, ripe=True)

    depth, disk = _scene()
    r = read_blob(INTR, depth, blob, disk)
    check("정상 열매를 읽는다", r.ok, r.reason)
    check("광축 위 열매는 (0,0,z)",
          abs(r.point_mm[0]) < 1e-9 and abs(r.point_mm[1]) < 1e-9
          and abs(r.point_mm[2] - 300.0) < 1e-9, str(np.round(r.point_mm, 3).tolist()))
    # 반지름 30px, 거리 300mm, f=400 → 30 * 300 / 400 = 22.5mm
    check("반지름이 길이가 된다", abs(r.radius_mm - 22.5) < 1e-6, f"{r.radius_mm:.2f}mm")

    holes = depth.copy()
    holes[disk] = 0.0
    check("구멍뿐이면 거절 (좌표가 카메라 원점이 되는 것을 막는다)",
          not read_blob(INTR, holes, blob, disk).ok,
          read_blob(INTR, holes, blob, disk).reason[:60])

    # 절반만 구멍이면? 남은 절반으로 읽을 수 있어야 한다(D405에서 흔한 상황).
    half = depth.copy()
    yy, xx = np.ogrid[:200, :200]
    half[disk & (xx > 100)] = 0.0
    r_half = read_blob(INTR, half, blob, disk)
    check("절반이 구멍이어도 남은 절반으로 읽는다", r_half.ok, r_half.reason[:60])

    # 잎이 앞을 가린 경우 — 마스크 안에 가까운 값이 섞인다
    leafy = depth.copy()
    leafy[disk & (xx > 95)] = 180.0
    check("깊이가 두 층이면 거절",
          not read_blob(INTR, leafy, blob, disk).ok,
          read_blob(INTR, leafy, blob, disk).reason[:50])

    # 가장자리가 배경을 보는 경우 — 중심만 보므로 **영향을 안 받아야** 한다.
    edgy = depth.copy()
    ring = disk & (((xx - 100) ** 2 + (yy - 100) ** 2) > 24 ** 2)
    edgy[ring] = 800.0
    r_edge = read_blob(INTR, edgy, blob, disk)
    check("가장자리가 배경을 봐도 중심으로 읽는다",
          r_edge.ok and abs(r_edge.depth_mm - 300.0) < 1e-9,
          f"{r_edge.depth_mm:.1f}mm")

    far = depth.copy()
    far[disk] = 1500.0
    check("D405 유효 범위 밖은 거절", not read_blob(INTR, far, blob, disk).ok)

    noisy = depth.copy()
    rng = np.random.default_rng(7)
    noisy[disk] = 300.0 + rng.normal(0, 4.0, int(disk.sum()))
    r_noise = read_blob(INTR, noisy, blob, disk)
    check("적당한 잡음은 통과", r_noise.ok and r_noise.spread_mm < MAX_SPREAD_MM,
          f"퍼짐 {r_noise.spread_mm:.1f}mm")


# ----------------------------------------------------------------------
# ⑥ TF 수학
# ----------------------------------------------------------------------

def _rot(axis: str, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def test_tf_math() -> None:
    print("\n[TF] 쿼터니언 · camera_link 재타깃")
    for label, R in (("항등", np.eye(3)),
                     ("z 90°", _rot("z", 90.0)),
                     ("180° 근처", _rot("x", 179.5)),
                     ("섞임", _rot("z", 33.0) @ _rot("y", -71.0) @ _rot("x", 12.0))):
        q = store.quaternion(R)
        back = store.rotation(q)
        check(f"쿼터니언 왕복 — {label}", np.allclose(back, R, atol=1e-9),
              f"|q|={np.linalg.norm(q):.9f}")

    # 재타깃: 광학 프레임에 직접 붙이면 부모가 둘이 된다. 한 칸 위(camera_link)로
    # 옮겨도 **광학 프레임의 최종 위치는 같아야** 한다 — 그게 이 함수의 정의다.
    base_to_optical = Rigid(_rot("y", -25.0) @ np.array([[0, 0, 1.0], [-1.0, 0, 0],
                                                         [0, -1.0, 0]]),
                            np.array([320.0, 40.0, 250.0]))
    link_to_optical = Rigid(np.array([[0, 0, 1.0], [-1.0, 0, 0], [0, -1.0, 0]]),
                            np.array([0.0, 15.0, 0.0]))
    base_to_link = store.retarget(base_to_optical, link_to_optical)
    recomposed = base_to_link.compose(link_to_optical)
    check("재타깃해도 광학 프레임 위치가 같다",
          np.allclose(recomposed.t, base_to_optical.t, atol=1e-9)
          and np.allclose(recomposed.R, base_to_optical.R, atol=1e-9),
          f"t={np.round(base_to_link.t, 2).tolist()}")

    p_cam = np.array([10.0, -20.0, 300.0])
    check("점을 옮긴 결과도 같다",
          np.allclose(base_to_optical.apply(p_cam),
                      base_to_link.apply(link_to_optical.apply(p_cam)), atol=1e-9))


def main() -> int:
    print(f"저장소: {REPO}")
    geom = kin.ArmGeometry()
    print(f"링크 길이: z0={geom.z0} d0={geom.d0} l1={geom.l1} l2={geom.l2} "
          f"l3={geom.l3} → 사거리 {geom.reach_max:.0f}mm")

    test_rclpy_isolation()
    test_geometry_matches()
    test_urdf_matches_kinematics()
    test_board_contract()
    test_fruit3d()
    test_tf_math()

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
