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
  ①' **레거시 경계** — 레거시에서 무엇을 끌어오는가. *계산은 공유하고 소유권은
     넘긴다*는 선을 지키는지. 웹 서비스에 붙는 경로가 기본값이 아닌지
  ② **URDF ↔ 기구학 일치** — xacro의 관절 원점·축으로 계산한 TCP가
     kinematics.forward()와 같은 곳을 가리키는가. **부호 하나만 틀려도 여기서 걸린다.**
  ③ **링크 길이 일치** — so101_geometry.yaml과 ArmGeometry 기본값
  ④ **보드 계약** — 단위 변환·체크섬·정지마찰 feedforward·조용한 폴백 거절
  ⑤ **깊이 읽기** — 구멍·잎 섞임·가장자리를 실제로 거절하는가
  ⑥ **TF 수학** — 쿼터니언 왕복, camera_link 재타깃
  ⑦ **손-눈 합격선** — 잔차 15mm 판정이 경고가 아니라 종료코드·저장차단으로
     이어지는가 (`handeye_resolve.gate`)
  ⑦' **줄끝** — 배포되는 파일이 작업트리에서도 LF인가 (CRLF는 컨테이너 bash와
     systemd 유닛을 *조용히* 깬다)
  ⑧ **의존성** — 빈 환경에서 4종이 *무엇이 없는지 말하고* 죽는가
     (`tools/selfcheck_deps.py`). 종료코드 2 = 환경이 없다, 1 = 검사 실패

⚠ 여기가 통과해도 로봇이 도는 건 아니다. 여기서 걸리는 종류의 실수(부호,
   라디안/도, mm/m, 좌표계 부모)를 **실물에서 배우지 않게** 하는 것이 전부다.
"""

from __future__ import annotations

import fnmatch
import io
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

# 서드파티보다 먼저 — pyyaml이 없어 여기서 죽는 것을 "통과"로 오독한 적이 있다
# (2026-09-17). tools/selfcheck_deps.py의 머리말을 보라.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))), "tools"))
from selfcheck_deps import require  # noqa: E402

require("numpy", "yaml")

import numpy as np  # noqa: E402
import yaml  # noqa: E402

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
from tomato_picker.hardware.handeye import (  # noqa: E402
    GOOD_RMS_MM as handeye_good, Intrinsics, Rigid,
)

from tomato_bridge import board_contract as bc  # noqa: E402
from tomato_bridge.arm_source import EXTRA_JOINTS, JOINT_NAMES  # noqa: E402
from tomato_handeye import store  # noqa: E402
from tomato_perception.fruit3d import (  # noqa: E402
    MAX_SPREAD_MM, Blob, read_blob,
)

FAILED: list[str] = []
PASSED = 0

# 줄끝 검사가 찾는 바이트. 소스에 날 CR을 적어 두면 편집기나 이 검사 자신이
# 그것부터 고쳐 버린다 — 그래서 숫자로 쓴다.
CR = bytes([13])


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

LEGACY_ALLOWED = {
    "tomato_picker.hardware.kinematics": "순수 계산 (import: math)",
    "tomato_picker.hardware.handeye": "순수 계산 (import: numpy)",
    "tomato_picker.hardware.cartesian": "이 팔의 영점·환산·안전 — 실측값은 한 벌",
    "tomato_picker.hardware.escape": "특이점 탈출 규칙 (순수 계산, 포트를 안 연다) — "
                                     "arm_extend·arm_source·move5_check가 같은 것을 써야 한다",
    "tomato_picker.hardware.eye": "보정 저장소 — ~/arm_eye.json 한 벌",
    "tomato_picker.hardware.ports": "포트 탐색 유틸",
    "tomato_picker.hardware.servo_probe": "버스 생존 확인 유틸",
    "tomato_picker.config": "상수(공장 초기값)",
}
# 지금은 쓰되 정해진 이정표에서 끊는다.
LEGACY_TRANSITIONAL = {
    "tomato_picker.hardware.motor_link": ("주행 보드 드라이버", "v2.0.0-ros.4 (ros2_control)"),
}
# 이유를 적어 둔 금지 목록 — 걸렸을 때 "왜 안 되는지"가 바로 나와야 한다.
LEGACY_FORBIDDEN = {
    "tomato_picker.hardware.arm": "LerobotArm은 프리셋·미러링·시퀀스까지 든 "
                                  "레거시 대시보드의 팔이다. 필요한 건 모터 버스뿐 "
                                  "(tomato_bridge.follower_io)",
    "tomato_picker.voice": "레거시 웹 서비스 — ROS가 거기 붙으면 새 계통이 아니다",
    "tomato_picker.harvest": "무대 학습(SceneModel)은 새 계통이 없애려는 그것이다",
    "tomato_picker.vision": "검출은 ROS 토픽으로 받는다 (tomato_perception)",
}


def _legacy_imports() -> dict[str, list[str]]:
    """ros2/src의 모든 파일에서 tomato_picker.* import를 모은다 → {모듈: [파일…]}."""
    found: dict[str, list[str]] = {}
    pattern = re.compile(
        r"^\s*(?:from\s+(tomato_picker[\w.]*)\s+import\s+(.+)|import\s+(tomato_picker[\w.]*))",
        re.MULTILINE)
    for root, _dirs, files in os.walk(SRC):
        if "__pycache__" in root:
            continue
        for f in files:
            if not f.endswith(".py"):
                continue
            path = os.path.join(root, f)
            with open(path, encoding="utf-8") as fh:
                body = fh.read()
            rel = os.path.relpath(path, REPO)
            for base, names, plain in pattern.findall(body):
                if plain:
                    found.setdefault(plain, []).append(rel)
                    continue
                # `from tomato_picker.hardware import kinematics as kin` 처럼
                # 패키지에서 모듈을 꺼내는 형태는 이름 쪽이 모듈이다.
                if base in ("tomato_picker", "tomato_picker.hardware"):
                    for chunk in names.replace("(", " ").replace(")", " ").split(","):
                        name = chunk.strip().split(" as ")[0].strip()
                        if name:
                            found.setdefault(f"{base}.{name}", []).append(rel)
                else:
                    found.setdefault(base, []).append(rel)
    return found


def test_legacy_boundary() -> None:
    """**계산은 공유하고, 소유권은 넘긴다** — 그 선을 코드가 지키는지 본다.

    이 검사가 있는 이유: 경계는 사람이 지키기로 한 약속이라 반드시 한 번은
    어긴다. 어긴 결과가 "새 계통이 옛 계통의 껍데기"인데, 그건 한참 뒤에야
    눈에 띈다. 여기서 즉시 걸리게 한다.
    """
    print("\n[경계] 레거시에서 무엇을 끌어오는가")
    imports = _legacy_imports()
    unknown, forbidden = [], []

    for module, users in sorted(imports.items()):
        where = ", ".join(sorted(set(os.path.basename(u) for u in users)))
        if module in LEGACY_ALLOWED:
            check(f"허용 — {module}", True, f"{LEGACY_ALLOWED[module]} · {where}")
        elif module in LEGACY_TRANSITIONAL:
            why, until = LEGACY_TRANSITIONAL[module]
            check(f"과도기 — {module}", True, f"{why}, {until}까지 · {where}")
        else:
            reason = next((r for pre, r in LEGACY_FORBIDDEN.items()
                           if module == pre or module.startswith(pre + ".")), None)
            (forbidden if reason else unknown).append((module, where, reason))

    for module, where, reason in forbidden:
        check(f"금지된 의존 — {module}", False, f"{reason} (쓰는 곳: {where})")
    for module, where, _ in unknown:
        check(f"표에 없는 의존 — {module}", False,
              f"허용/과도기/금지 어디에도 없다. docs/ros2-이행계획.md의 "
              f"'레거시 의존 졸업표'에 판정을 적고 이 목록에 넣어라 (쓰는 곳: {where})")

    # 웹 서비스는 import가 아니라 HTTP로 붙으므로 위 검사에 안 걸린다. 기본값을 본다.
    with open(os.path.join(SRC, "tomato_bringup", "config", "stage1.yaml"),
              encoding="utf-8") as f:
        params = yaml.safe_load(f)
    mode = params["tomato_arm"]["ros__parameters"]["arm_mode"]
    check("팔 기본 경로가 direct다 (웹 서비스에 안 붙는다)", mode == "direct",
          f"arm_mode={mode!r} — proxy가 기본이면 새 계통이 레거시 대시보드 없이는 "
          "못 돈다")

    launch = os.path.join(SRC, "tomato_bringup", "launch", "stage1.launch.py")
    with open(launch, encoding="utf-8") as f:
        body = f.read()
    check("런치 기본값도 direct다",
          '"arm_mode", default_value="direct"' in body,
          "런치 인자와 yaml이 다르면 어느 쪽이 이겼는지 아무도 모른다")

    eye = params["tomato_handeye"]["ros__parameters"]
    check("보정 파일이 한 벌이다 (레거시와 같은 ~/arm_eye.json)",
          eye["calib_path"] == "",
          f"calib_path={eye['calib_path']!r} — 따로 두면 같은 카메라의 보정이 두 벌이 된다")
    # 장착 방식은 "파일이 정한다"(빈 값)거나, 실물과 맞는 값이어야 한다.
    check("장착 방식이 fixed/on_arm/빈값 중 하나", eye["mount"] in ("", "fixed", "on_arm"),
          f"mount={eye['mount']!r}")
    # ⚠ on_arm인데 TF 부모가 arm_base면 카메라가 팔 밑동에 박혀 손목을 안 따라간다.
    #    정지 상태에서는 그럴듯해 보이고 팔이 움직이는 순간부터 전부 틀린다.
    node_src = os.path.join(SRC, "tomato_handeye", "tomato_handeye", "handeye_node.py")
    with open(node_src, encoding="utf-8") as f:
        handeye_body = f.read()
    check("TF 부모를 장착 방식이 정한다",
          "_tf_parent" in handeye_body and 'self._mount == "on_arm"' in handeye_body,
          "on_arm은 T_tool_cam을 푼다 — tool0에 매달아야 한다")

    # store.py가 자기 형식으로 파일을 쓰면 '한 벌'이 깨진다.
    with open(os.path.join(SRC, "tomato_handeye", "tomato_handeye", "store.py"),
              encoding="utf-8") as f:
        store_body = f.read()
    check("store.py가 보정 형식을 다시 구현하지 않는다",
          "json.dump" not in store_body,
          "저장 형식이 둘이면 값도 둘이 된다 — eye.EyeConfig를 호출하라")


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


def test_handeye_gate() -> None:
    """졸업 기준 2번(잔차 ≤15mm)을 **코드가** 강제하는가.

    이 검사가 있는 이유: 09-04에 17.8·20.2mm짜리 해가 나왔는데도 도구는
    경고 한 줄만 찍고 0으로 끝났다. 사람이 그 줄을 안 읽으면 그대로 다음 단계로
    간다. 여기서는 팔도 표본도 없이 **판정만** 시험한다 — 경계값과 종료코드.
    """
    print("\n[손-눈 합격선] 잔차 15mm를 코드가 강제하는가")
    sys.path.insert(0, os.path.join(ROS2, "tools"))
    import handeye_resolve as hr  # noqa: E402

    check("합격선이 레거시 handeye.py와 한 벌이다",
          hr.GOOD_RMS_MM == handeye_good, f"{hr.GOOD_RMS_MM} vs {handeye_good}")
    # 경계는 포함이다 — Fit.good(`<=`)과 다른 쪽으로 자르면 같은 해를 한 도구는
    # 통과시키고 다른 도구는 거절한다.
    check("14.99mm는 합격", hr.gate(14.99)[0] is True)
    check("정확히 15.00mm도 합격", hr.gate(15.0)[0] is True)
    check("15.01mm는 불합격", hr.gate(15.01)[0] is False)
    check("불합격 문구가 이유를 말한다", "헛집" in hr.gate(20.2)[1], hr.gate(20.2)[1])
    check("--max-rms로 합격선을 좁힐 수 있다", hr.gate(12.0, 10.0)[0] is False)

    # ⚠ 판정이 종료코드로 이어지는가 — 여기가 끊기면 위의 경계 검사는 장식이다.
    src = open(os.path.join(ROS2, "tools", "handeye_resolve.py"),
               encoding="utf-8").read()
    check("main()이 불합격을 비영 종료코드로 낸다",
          "return 0 if ok else 1" in src,
          "gate()가 False인데 0으로 끝나면 스크립트는 아무것도 안 멈춘다")
    check("불합격이면 --save를 막는다",
          "if args.save and not ok and not args.save_anyway:" in src,
          "경고만 하고 파일은 갱신하면, 다음 사람은 파일이 있다는 것만 보고 믿는다")


def _synthetic_handeye(t_x, R_x=None, target=None, noise_mm=0.0, seed=11,
                       geom=None, dot_scale=1.0):
    """정답을 아는 손-눈 표본을 만든다 — 팔도 카메라도 없이.

    ⚠ 왜 필요한가. 2026-09-04~17 내내 "잔차가 20mm고 |t_x|가 실측의 4~6배"라는
      같은 답이 반복됐는데, 그게 **하드웨어 탓인지 푸는 쪽 탓인지** 가릴 방법이
      없었다. 정답을 심은 표본에 같은 풀이를 돌려 보면 그 둘이 갈린다:
      되찾으면 풀이는 결백하고 표본이 상한 것이다(2026-09-17 T5가 그렇게 갈랐다).

    자세 집합은 09-04 실측 채집과 비슷한 폭(pan ±18°, elbow 30°, roll 120°)으로
    만든다 — 좁은 자세 폭 자체가 범인이라는 가설도 여기서 같이 죽는다.
    """
    import numpy as np
    from tomato_picker.hardware import kinematics as kin

    geom = geom or kin.ArmGeometry()
    t_x = np.asarray(t_x, dtype=float)
    if R_x is None:
        th = math.radians(20.0)
        R_x = np.array([[math.cos(th), 0.0, math.sin(th)],
                        [0.0, 1.0, 0.0],
                        [-math.sin(th), 0.0, math.cos(th)]])
    target = np.asarray(target if target is not None else [420.0, 10.0, 260.0], float)
    # 표적은 100 x 174.5mm 직사각형(실물과 같은 것) — 네 점 경로도 시험할 수 있게.
    corners = {"tl": (-50.0, 87.25), "tr": (50.0, 87.25),
               "bl": (-50.0, -87.25), "br": (50.0, -87.25)}

    # ⚠ 자세를 **골고루 쒸어야** 한다. 첫 판은 관절을 전부 같은 방향으로
    #   일정하게 쒸었는데, 그러면 관절끼리 완전히 상관돼 교대최소화가
    #   극소에 갇혔다(잡음 0인데도 t_x 오\李82mm). 이것도 이 저장소의 1번 병과
    #   같은 모양이다 — "도는 것처럼 보이는데 실제로 갈리는 것은 없다."
    rng = np.random.default_rng(seed)
    span = {"shoulder_pan": (-18.0, 18.0), "shoulder_lift": (11.0, 24.0),
            "elbow_flex": (32.0, 63.0), "wrist_flex": (89.0, 103.0),
            "wrist_roll": (-62.0, 59.0)}
    poses = [{j: float(rng.uniform(lo, hi)) for j, (lo, hi) in span.items()}
             for _ in range(11)]

    out = []
    for degs in poses:
        R_t, b = _hr().tool_frame(degs, geom, +1.0)
        dots = {}
        for k, (dy, dz) in corners.items():
            P = target + np.array([0.0, dy, dz])
            p_cam = R_x.T @ (R_t.T @ (P - b) - t_x)
            p_cam = p_cam * dot_scale + rng.normal(scale=noise_mm, size=3)
            dots[k] = [float(c) for c in p_cam]
        out.append({"label": "synth", "joints_deg": dict(degs), "dots_mm": dots})
    return out


def _hr():
    sys.path.insert(0, os.path.join(ROS2, "tools"))
    import handeye_resolve as hr  # noqa: E402
    return hr


def test_handeye_identifiability() -> None:
    """무엇이 이 표본으로 **원리적으로 갈리는가** — 죽은 손잡이를 못 박는다.

    2026-09-17(T5)에 숫자로 확인한 것들이다. 이 검사가 없으면 다음 사람이
    같은 벽에 다시 머리를 박는다:

      · `l3`와 `t_x`의 접근축 성분은 **완전히 같은 짓**을 한다. 그래서 `|t_x|`
        하나만 떼어 "실측의 3~5배"라고 말하는 것은 뜻이 없다 — 데이터가 정하는
        것은 `l3 + t_x[approach]`라는 **합** 하나뿐이다.
      · `z0`는 표적 위치(자유 미지수)가 통째로 흡수한다 — 잔차를 못 움직인다.
      · 반대로 풀이기 자체는 건강하다. 정답을 심으면 되찾는다.
    """
    import numpy as np
    from tomato_picker.hardware import kinematics as kin

    print("\n[손-눈 식별성] 이 식으로 무엇이 갈리고 무엇이 안 갈리는가")
    hr = _hr()
    geom = kin.ArmGeometry()

    # ── ① 도구 좌표계가 진짜 오른손 회전인가 ────────────────────────────
    # 한쪽이라도 왼손계면 R_x(회전)로는 절대 못 맞춘다 — 그때 오차는 갈 곳이
    # 없어 t_x로 몰린다(그게 09-04의 435mm처럼 보였다).
    dets, orth = [], []
    for s in _synthetic_handeye([-80.0, 0.0, 80.0]):
        R, _t = hr.tool_frame(s["joints_deg"], geom, +1.0)
        dets.append(float(np.linalg.det(R)))
        orth.append(float(np.abs(R.T @ R - np.eye(3)).max()))
    check("tool_frame이 오른손 회전이다 (det=+1)",
          all(abs(d - 1.0) < 1e-9 for d in dets), f"det {min(dets):.9f}~{max(dets):.9f}")
    check("tool_frame이 직교한다", max(orth) < 1e-9, f"최대 {max(orth):.1e}")

    # ── ② TCP는 wrist_roll 축 위에 있다 ────────────────────────────────
    # 이걸 어기면 roll을 돌릴 때마다 원점이 흔들려 t_x가 자세마다 달라진다.
    d0 = {"shoulder_pan": 10.0, "shoulder_lift": 20.0, "elbow_flex": 40.0,
          "wrist_flex": 95.0, "wrist_roll": 0.0}
    p0 = hr.tool_frame(d0, geom, +1.0)[1]
    p1 = hr.tool_frame(dict(d0, wrist_roll=40.0), geom, +1.0)[1]
    check("roll을 돌려도 TCP는 안 움직인다 (TCP가 롤축 위)",
          float(np.linalg.norm(p1 - p0)) < 1e-9, f"{np.linalg.norm(p1 - p0):.3e}mm")

    # ── ③ l3 ↔ t_x[approach] 는 완전 축퇴 ──────────────────
    # 잠리를 직접 계산해 보인다 — 푸는 쪽(국소해)을 거치면 같은 해로
    # 안 가서 축퇴가 가려진다. 식은 b_i(l3+Δ) = b_i(l3) + Δ·approach_i 이고
    # R_i·(… + t_x − Δ·e_approach) 가 그것을 그대로 상쇄한다.
    S = _synthetic_handeye([-80.0, 0.0, 80.0])
    Rx0 = np.eye(3)
    tx0 = np.array([-80.0, 0.0, 80.0])

    def _rms(g2, tx):
        frames = [hr.tool_frame(s["joints_deg"], g2, +1.0) for s in S]
        obs = [np.mean([s["dots_mm"][k] for k in hr.DOTS], axis=0) for s in S]
        pred = np.array([R @ (Rx0 @ o + tx) + b for (R, b), o in zip(frames, obs)])
        P = pred.mean(axis=0)
        return float(np.sqrt((np.linalg.norm(pred - P, axis=1) ** 2).mean()))

    r_a = _rms(geom, tx0)
    r_b = _rms(kin.ArmGeometry(l3=geom.l3 + 50.0), tx0 - np.array([50.0, 0.0, 0.0]))
    check("l3 +50mm 와 t_x[approach] −50mm 는 잔차가 **똑같다** (완전 축퇴)",
          abs(r_a - r_b) < 1e-9, f"{r_a:.9f} vs {r_b:.9f}mm")
    check("그래서 데이터가 정하는 것은 합 l3+t_a 하나다 — |t_x| 단독은 뜻이 없다",
          abs((geom.l3 + tx0[0]) - ((geom.l3 + 50.0) + (tx0[0] - 50.0))) < 1e-9)

    # ── ⑤ 풀이기는 결백하다 — 정답을 심으면 되찾는다 ────────────────────
    # 09-04의 "|t|가 4~6배"를 두고 자세 폭 부족을 의심했는데, 같은 폭의 자세로
    # 5mm 잡음을 줘도 되찾는다. 그러므로 범인은 자세 집합이 아니다.
    for noise, tol in ((0.0, 0.5), (5.0, 20.0)):
        syn = _synthetic_handeye([-80.0, 0.0, 80.0], noise_mm=noise)
        got = hr.solve(syn, geom, +1.0, ("mid",))[2]   # 기본값으로 — 수렴까지 돈다
        err = float(np.linalg.norm(got - np.array([-80.0, 0.0, 80.0])))
        check(f"잡음 {noise:.0f}mm에서 t_x를 {tol:.1f}mm 안으로 되찾는다",
              err < tol, f"오차 {err:.2f}mm · 되찾은 |t|={np.linalg.norm(got):.1f}mm "
                         f"(정답 113.1mm)")

    # ── ⑤-2 solve()는 회수가 아니라 **개선폭**으로 멎는다 (T16, 2026-09-18) ──
    # 옛 기본값 80회는 잡음 0인데도 t_x를 10.2mm 틀렸다 — 졸업 예산 15mm의
    # 2/3를 데이터가 아니라 멈추는 시점이 먹고 있었다. 그 사실을 남겨 두고
    # (아래 첫 검사), 기본값이 이제 정말 수렴까지 간다는 것을 못 박는다.
    slow = _synthetic_handeye([-80.0, 0.0, 80.0])
    truth = np.array([-80.0, 0.0, 80.0])
    r80 = hr.solve(slow, geom, +1.0, ("mid",), iters=80)
    rdef = hr.solve(slow, geom, +1.0, ("mid",))
    check("회수로 자르면(iters=80) 잔차가 1mm 이상 나쁘다 — 기본값을 바꾼 이유",
          r80[0] > rdef[0] + 1.0,
          f"80회 rms={r80[0]:.3f}mm · t_x 오차 "
          f"{np.linalg.norm(r80[2] - truth):.1f}mm → 기본값 rms={rdef[0]:.3f}mm · "
          f"{np.linalg.norm(rdef[2] - truth):.3f}mm")
    check("기본값은 잡음 0에서 t_x를 0.1mm 안으로 되찾는다 (수렴했다는 뜻)",
          float(np.linalg.norm(rdef[2] - truth)) < 0.1,
          f"오차 {np.linalg.norm(rdef[2] - truth):.4f}mm · rms {rdef[0]:.6f}mm")
    # 상한(iters)에 부딪혀 잘린 게 아니라 **개선폭**으로 멎었음을 보인다 —
    # 상한을 10배로 키워도 답이 그대로여야 진짜 멎은 것이다.
    r10x = hr.solve(slow, geom, +1.0, ("mid",), iters=200000)
    check("상한을 10배로 키워도 답이 안 바뀐다 (회수에 잘린 게 아니다)",
          abs(r10x[0] - rdef[0]) < 1e-9
          and float(np.linalg.norm(r10x[2] - rdef[2])) < 1e-6,
          f"rms {rdef[0]:.9f} vs {r10x[0]:.9f}mm · "
          f"t_x 차 {np.linalg.norm(r10x[2] - rdef[2]):.2e}mm")
    # tol이 진짜 손잡이인지 — 크게 주면 일찍 멎어 옛 병(잘린 답)이 재현된다.
    rcoarse = hr.solve(slow, geom, +1.0, ("mid",), tol=1.0)
    check("tol을 크게 주면 일찍 멎는다 — 멈춤 조건이 개선폭이라는 증거",
          rcoarse[0] > rdef[0] + 1.0,
          f"tol=1.0mm rms={rcoarse[0]:.3f}mm / 기본 tol rms={rdef[0]:.6f}mm")

    # ── ⑥ 거울(왼손계)은 구별된다 ──────────────────────────────────────
    # 카메라점의 한 축 부호가 뒤집히면 회전으로는 못 되돌린다. 그 증상이
    # "잔차도 크고 |t_x|가 수백 mm로 달아난다"이다 — 09-04 표본과 같은 모양.
    S0 = _synthetic_handeye([-80.0, 0.0, 80.0])
    Sm = [{"joints_deg": s["joints_deg"],
           "dots_mm": {k: [v[0], -v[1], v[2]] for k, v in s["dots_mm"].items()}}
          for s in S0]
    good = hr.solve(S0, geom, +1.0, ("mid",))
    mir = hr.solve(Sm, geom, +1.0, ("mid",))
    check("카메라점을 거울로 뒤집으면 잔차가 크게 나빠진다",
          mir[0] > good[0] + 20.0, f"{good[0]:.2f} → {mir[0]:.2f}mm")
    check("그리고 |t_x|가 수백 mm로 달아난다 (09-04 표본과 같은 증상)",
          float(np.linalg.norm(mir[2])) > 400.0,
          f"|t|={np.linalg.norm(mir[2]):.0f}mm (정답 113mm)")


# ----------------------------------------------------------------------
# ⑨ 특이점 탈출 (arm_extend)
# ----------------------------------------------------------------------

# 2026-09-18 젯슨 실측 — 프리셋 "대기"에서 읽은 정규화값. 이 자세가 문제의
# 출발점이다: shoulder_pan 98.81 · elbow_flex 103.16 이 **이미 범위 밖**이다.
ESCAPE_NORM_2026_09_18 = {
    "shoulder_pan": 98.81, "shoulder_lift": -90.86, "elbow_flex": 103.16,
    "wrist_flex": 5.54, "wrist_roll": 88.84,
}
ESCAPE_FRAME_2026_09_18 = {  # ~/arm_cartesian.json + 보정표 span에서 온 눈금
    "zero": {"shoulder_pan": -11.52, "shoulder_lift": -18.24,
             "elbow_flex": -60.35, "wrist_flex": -1.76, "wrist_roll": 0.0},
    "ref": {"shoulder_pan": 0.0, "shoulder_lift": 90.0, "elbow_flex": 0.0,
            "wrist_flex": 0.0, "wrist_roll": 0.0},
    "dpn": {"shoulder_pan": 1.2173, "shoulder_lift": 1.0288,
            "elbow_flex": 1.0332, "wrist_flex": 1.0389, "wrist_roll": 0.999},
}


def _escape_deg(norms: dict) -> dict:
    f = ESCAPE_FRAME_2026_09_18
    return {j: f["ref"][j] + (float(norms[j]) - f["zero"][j]) * f["dpn"][j]
            for j in norms}


def _escape_norm(degs: dict) -> dict:
    f = ESCAPE_FRAME_2026_09_18
    return {j: f["zero"][j] + (float(degs[j]) - f["ref"][j]) / f["dpn"][j]
            for j in degs}


def test_arm_extend_escape() -> None:
    """특이점에 갇힌 자세에서 **빠져나올 수 있는가**.

    2026-09-18(T26): `arm_extend.py --dry`가 21구간 전부를 막았다. 막은 이유는
    `shoulder_pan한계`인데 그 관절은 경로에서 **1도도 안 움직인다** — 지금 자리가
    이미 정규화 98.81이라 "범위 밖"에 걸린 것이다. 같은 줄에 걸린 elbow_flex는
    반대로 목표(-80° = 정규화 -137.8)가 범위 밖이었다. 둘 다 "못 간다"가 아니라
    **검사가 틀린 질문을 한 것**이다. 가둬 놓고 거절하는 병은 바닥 판정에서 한 번
    (MOUNT_Z_MM 주석), 여기서 두 번째다 — 그래서 검사로 박는다.
    """
    print("\n[탈출] arm_extend 관절한계 판정과 목표 자르기")
    sys.path.insert(0, os.path.join(REPO, "ros2", "tools"))
    import arm_extend as ax  # noqa: E402

    now = ESCAPE_NORM_2026_09_18

    check("이미 범위 밖인 관절이 그대로 있으면 막지 않는다",
          ax.limit_violations(now, dict(now)) == [],
          f"pan={now['shoulder_pan']} elbow={now['elbow_flex']}")
    check("더 밖으로 나가면 막는다",
          ax.limit_violations(now, {**now, "shoulder_pan": 99.5}) == ["shoulder_pan"])
    check("범위 밖에서 안쪽으로 돌아오는 걸음은 막지 않는다",
          ax.limit_violations(now, {**now, "elbow_flex": 99.0}) == [])
    check("범위 안에서 밖으로 나가면 막는다",
          ax.limit_violations(now, {**now, "wrist_flex": -99.9}) == ["wrist_flex"])
    check("범위 안 걸음은 통과한다",
          ax.limit_violations(now, {**now, "wrist_flex": 40.0}) == [])

    tgt_deg = {**_escape_deg(now), "shoulder_lift": 80.0,
               "elbow_flex": -80.0, "wrist_flex": 0.0}
    clamped, hit = ax.clamp_norm(_escape_norm(tgt_deg))
    check("기본 목표 elbow -80°는 이 보정표에서 범위 밖이라 잘린다",
          hit == ["shoulder_pan", "elbow_flex"] or set(hit) == {"shoulder_pan", "elbow_flex"},
          f"잘린 관절={hit} (elbow 정규화 {_escape_norm(tgt_deg)['elbow_flex']:.1f})")
    check("자른 값은 정확히 한계값이다",
          abs(clamped["elbow_flex"] + ax.LIMIT_NORM) < 1e-9
          and abs(clamped["shoulder_pan"] - ax.LIMIT_NORM) < 1e-9,
          f"elbow={clamped['elbow_flex']} pan={clamped['shoulder_pan']}")
    check("자를 것이 없으면 아무것도 안 자른다", ax.clamp_norm(
        {"elbow_flex": 12.0})[1] == [])

    # **이게 핵심 회귀 검사다** — 자르고 나서도 좌표 가드(90mm)를 넘어야
    # 이 도구가 제 일을 한 것이다. 목적은 pitch 0°가 아니라 탈출이다.
    r_now = kin.signed_radius(_escape_deg(now))
    r_goal = kin.signed_radius(_escape_deg(clamped))
    check("지금 자세는 좌표 가드 안쪽이다 (그래서 이 도구가 필요하다)",
          r_now < 90.0, f"signed_r={r_now:.1f}mm")
    check("잘린 목표는 가드를 넉넉히 넘는다",
          r_goal > 2.0 * 90.0, f"signed_r={r_goal:.1f}mm (가드 90mm)")
    check("잘린 목표가 사거리 안이다",
          abs(r_goal) <= kin.ArmGeometry().reach_max,
          f"{r_goal:.1f} ≤ {kin.ArmGeometry().reach_max:.1f}mm")


# ----------------------------------------------------------------------
# ⑩ 실패 단계 분류 (move5_check)
# ----------------------------------------------------------------------

# 2026-09-18 젯슨 실기에서 **실제로 기록된** 거절 문장들. 둘 다 stage=tf로
# 적혔고 둘 다 TF와 무관했다(docs/시험기록/move-to-point-2026-09-18.jsonl).
POSE_DETAIL_2026_09_18 = (
    "이동 실패 — 팔이 몸통 뒤로 넘어가 있습니다 — 수평거리 -136mm < 90mm. "
    "이 근처에서는 집게가 회전축 위에 있어 xyz 방향이 정해지지 않습니다. "
    "프리셋으로 앞으로 뻗은 자세를 먼저 만든 뒤 좌표 이동을 쓰세요."
)
STEP_DETAIL_2026_09_18 = (
    "이동 실패 — 한 번에 572mm는 너무 큽니다(상한 80mm). 나눠서 가세요 — "
    "큰 이동은 프리셋으로 대략 자세를 잡은 뒤 좌표로 다듬는 게 안전합니다."
)
# arm_node._to_arm_base_mm이 실패할 때 올리는 문장 형식(arm_node.py:142).
TF_DETAIL = ("목표를 arm_base 좌표로 못 옮겼다: lookup failed. "
             "손-눈 보정을 했는지, 그 static TF가 떠 있는지 확인하라.")

# 2026-09-18 사이클20 실기에서 **실제로 기록된** 경로 실패 문장 2종(T34).
# 둘 다 stage=ik로 적혔고 둘 다 목표는 멀쩡했다 — 막힌 것은 첫 걸음이다
# (docs/시험기록/move-to-point-2026-09-18.jsonl). ik는 "목표가 무리"라는 뜻이라
# 이 오분류는 다음 사이클을 표적 뽑기로 보낸다(고칠 곳은 경로다).
PATH_FLOOR_2026_09_18 = (
    "이동 실패 — 9걸음 중 1번째에서 멈췄습니다 — z=-19mm는 바닥 아래입니다"
    "(하한 15mm) — 무대를 긁습니다."
)
PATH_ELBOW_2026_09_18 = (
    "이동 실패 — 6걸음 중 1번째에서 멈췄습니다 — 관절 가동범위를 벗어납니다: "
    "elbow_flex -124(한계 ±98, -66°) — 그 방향으로는 더 못 갑니다."
)
# 감사 사이클40이 재현한 세 번째 얼굴 — 걸음 **도중**의 수평거리 하락이 자세
# 가드(pose)로 읽혔다. 같은 낱말("수평거리")을 쓰기 때문이다.
PATH_RADIUS_2026_09_18 = (
    "이동 실패 — 3걸음 중 2번째에서 멈췄습니다 — 도중에 수평거리가 45mm로 "
    "떨어진다(하한 90mm)"
)


def _escape_limits():
    """실측 프레임(2026-09-18)으로 만든 NormLimits — 팔도 파일도 안 건드린다."""
    from tomato_picker.hardware import cartesian as cart  # noqa: E402
    f = ESCAPE_FRAME_2026_09_18
    return cart.NormLimits(zero=f["zero"], ref=f["ref"],
                           signs={j: 1.0 for j in f["zero"]},
                           deg_per_norm=f["dpn"], source="ros_selfcheck 고정표")


# 경로 막힘을 **살아 있는 코드가 실제로 만드는 문장**으로 확인할 자리.
# 실측 프레임(위 ESCAPE_FRAME)에 가짜 팔을 물린다 — 파일도 장치도 안 건드린다.
PATH_START_DEG = {"shoulder_pan": 0.0, "shoulder_lift": 55.0, "elbow_flex": -70.0,
                  "wrist_flex": 10.0, "wrist_roll": 0.0}
# 이 자리는 **끝점 둘 다 갈 수 있는데 그 사이가 막히는** 목표다(사이클20이 실기
# 5/5로 겪은 것과 같은 모양). 좌표를 바꾸면 전제가 깨지니 아래 검사가 그 전제를
# 먼저 확인한다.
PATH_TARGET = (100.0, 0.0, 360.0, 0.0)


def _sim_escape_arm():
    """실측 프레임으로 영점을 잡은 **가짜 팔**. 기하·가드·경로가 전부 진짜다."""
    import tempfile  # noqa: E402  (여기서만 쓴다)
    from tomato_picker.hardware import cartesian as cart  # noqa: E402
    f = ESCAPE_FRAME_2026_09_18
    spans = {j: 200.0 * f["dpn"][j] for j in f["dpn"]}
    io_ = cart.SimJointIO(joints=_escape_norm(PATH_START_DEG), spans=spans)
    arm = cart.CartesianArm(io_, path=os.path.join(tempfile.mkdtemp(), "frame.json"))
    arm.config.set_zero(_escape_norm(f["ref"]), f["ref"])
    return arm


def test_prep_autoextend() -> None:
    """졸업기준3을 재는 도구가 **스스로 뻗는가** (T49, 2026-09-18).

    사이클35는 A자세에서 move5_check를 그대로 돌려 5/5 자세 가드 거절(=0/5)을
    받았고, 사람이 `arm_extend --hold`를 따로 돌린 뒤에야 1/5가 나왔다. 그
    수동 단계는 어디에도 안 적혀 있었다 — 잊으면 **거짓 0/5**가 기록에 남고,
    일지와 작업판은 다음 사이클이 사실로 믿는다. 그래서 두 방향을 못 박는다:
    가드 안이면 뻗고, **가드 밖이면 안 뻗는다**(쓸데없이 뻗으면 시작 자세를
    바꿔 시험 자체를 흔든다).
    """
    print("\n[뻗기] move5_check의 시험 전 자동 뻗기(prep)")
    sys.path.insert(0, os.path.join(REPO, "ros2", "tools"))
    import arm_extend as ax  # noqa: E402
    import move5_check as m5  # noqa: E402

    from tomato_picker.config import (ARM_CART_MAX_STEP_JOINT_DEG,  # noqa: E402
                                      ARM_CART_Z_MIN)
    from tomato_picker.hardware import escape as es  # noqa: E402

    def source_of(*parts):
        return open(os.path.join(*parts), encoding="utf-8").read()

    geom = kin.ArmGeometry()
    limits = _escape_limits()

    # ① 가드 안 자세 — 2026-09-18 실측(프리셋 "대기", pan·elbow가 이미 범위 밖)
    inside = _escape_deg(ESCAPE_NORM_2026_09_18)
    got = m5.prep_plan(inside, limits, geom)
    check("가드 안 자세면 뻗는다고 판정한다",
          got["needed"] is True and got["steps"],
          f"signed_r={got['signed_r']}mm 걸음={len(got['steps'])}")
    check("그 경로는 안전 검사를 통과한다(막힌 걸음 0)",
          got["blocked"] == [], f"막힌 걸음={got['blocked']}")
    # ⚠ 아래는 전부 `.get`으로 읽는다 — 뻗기가 꺼지는 회귀가 오면 **실패로**
    #   보여야 하고, KeyError로 검사 전체가 죽어 남은 검사를 못 돌면 안 된다.
    check("뻗은 뒤 목표는 가드를 넘는다",
          (got.get("signed_r_target") or -1.0) >= es.GUARD_MM,
          f"{got['signed_r']} → {got.get('signed_r_target')}mm (가드 {es.GUARD_MM:.0f}mm)")
    check("가동범위 밖 목표는 잘리고 **무엇이 잘렸는지 남는다**",
          got["clamped"] == ["elbow_flex", "shoulder_pan"],
          f"잘린 관절={got['clamped']}")
    hop = (max(abs(got["steps"][0]["degs"][j] - inside[j]) for j in kin.JOINTS)
           if got["steps"] else None)
    check(f"첫 걸음이 {es.STEP_DEG:.0f}° 안이다",
          hop is not None and hop <= es.STEP_DEG + 1e-6,
          f"최대 {hop}°")

    # ② 가드 밖 자세 — arm_extend가 세워 놓은 자리(뻗을 이유가 없다)
    outside_deg = es.TARGET_DEG.copy()
    outside = {**inside, **outside_deg}
    r_out = kin.signed_radius(outside, geom)
    out = m5.prep_plan(outside, limits, geom)
    check("가드 밖 자세는 뻗지 않는다",
          out["needed"] is False and out["steps"] == [],
          f"signed_r={r_out:.1f}mm 걸음={len(out['steps'])}")
    check("안 뻗는 경우에도 지금 자세를 숫자로 남긴다",
          abs(out["signed_r"] - r_out) < 0.1, f"기록값={out['signed_r']}mm")

    # ③ 바닥으로 내려가는 목표는 **시작 전에** 막힌다(절반 가서 멈추지 않는다)
    down = {**inside, "shoulder_lift": -95.0, "elbow_flex": 0.0, "wrist_flex": 0.0}
    steps = es.plan(inside, down, limits.norms(inside), limits.norms, geom)
    check("바닥 아래로 가는 걸음은 막힌 걸음으로 잡힌다",
          any("바닥아래" in " ".join(st["notes"]) for st in steps),
          f"막힌 걸음={es.blocked(steps)}/{len(steps)}")

    # ④ 규칙이 갈라지지 않는가 — 셋이 **같은 모듈**을 쓰는지 값으로 확인한다
    check("arm_extend와 move5_check가 같은 걸음 상한을 쓴다",
          ax.STEP_DEG == es.STEP_DEG == ARM_CART_MAX_STEP_JOINT_DEG,
          f"arm_extend={ax.STEP_DEG} escape={es.STEP_DEG} "
          f"config={ARM_CART_MAX_STEP_JOINT_DEG}")
    check("탈출 바닥이 yaml의 mount.z와 같다",
          abs(es.MOUNT_Z_MM - float(_geometry_yaml()["mount"]["z"])) < 1e-9,
          f"escape={es.MOUNT_Z_MM} yaml={_geometry_yaml()['mount']['z']}")
    check("좌표 유닛의 바닥(15mm)을 탈출에 쓰지 않는다",
          es.floor_z() < 0.0 < ARM_CART_Z_MIN,
          f"탈출 바닥={es.floor_z()}mm vs 좌표 바닥={ARM_CART_Z_MIN}mm "
          "(주저앉은 팔은 z≈-66mm다 — 15mm를 바닥으로 보면 탈출이 불가능하다)")

    # ⑤ 토픽 이름은 양쪽이 **글자 그대로** 같아야 한다(오타는 조용히 안 돈다)
    node_src = source_of(SRC, "tomato_bridge", "tomato_bridge", "arm_node.py")
    m5_src = source_of(REPO, "ros2", "tools", "move5_check.py")
    src_src = source_of(SRC, "tomato_bridge", "tomato_bridge", "arm_source.py")
    check("arm_node가 관절 지령 토픽을 구독한다",
          'JointState, "arm/joint_command"' in node_src)
    check("move5_check가 같은 이름으로 발행한다",
          '"/arm/joint_command"' in m5_src and '"/arm/joint_command_result"' in m5_src)
    check("prep를 끌 수 있다(기본은 켬)", '"--no-prep"' in m5_src)
    check("proxy 모드는 관절 지령을 **거절**한다(뻗은 척하지 않는다)",
          "def move_joints_deg" in src_src
          and "proxy 모드로는 관절 지령을 못 보낸다" in src_src)


def test_stage_classify() -> None:
    """거절 문장을 **어느 단계**로 적는가.

    이 검사가 있는 이유: 09-18에 기준3의 유일한 기록이 `stage=tf` 열 줄이었고
    **열 줄 다 TF가 아니었다**(자세 가드 5 + 한 걸음 상한 5). 옛 판정이
    `"좌표" in detail`이었고, 가드 안내문이 "좌표 이동을 쓰세요"로 끝나기
    때문이다. 기록은 다음 사이클이 사실로 믿는 물건이라 오분류 한 글자가
    하루를 엉뚱한 데로 끌고 간다 — 그래서 실제 문장으로 못 박는다.
    """
    print("\n[단계] move5_check 실패 단계 분류")
    sys.path.insert(0, os.path.join(REPO, "ros2", "tools"))
    import move5_check as m5  # noqa: E402
    from tomato_picker.hardware import cartesian as cart  # noqa: E402

    check("자세 가드 문구 → pose (실기 기록 그대로)",
          m5.classify_stage(POSE_DETAIL_2026_09_18) == "pose",
          m5.classify_stage(POSE_DETAIL_2026_09_18))
    check("한 걸음 상한 문구 → step (실기 기록 그대로)",
          m5.classify_stage(STEP_DETAIL_2026_09_18) == "step",
          m5.classify_stage(STEP_DETAIL_2026_09_18))
    check("TF 실패 문구 → tf", m5.classify_stage(TF_DETAIL) == "tf")
    check("응답 없음 → timeout (tf가 아니다)",
          m5.classify_stage(None) == "timeout"
          and m5.classify_stage("응답 없음(타임아웃)") == "timeout")
    check("IK 실패 → ik", m5.classify_stage("IK가 안 풀린다 — 사거리 밖") == "ik")

    # --- 경로 실패는 path다 (T34) — 목표가 아니라 **가는 길**이 막힌 것 ---
    for name, detail in (("z 바닥", PATH_FLOOR_2026_09_18),
                         ("elbow 한계", PATH_ELBOW_2026_09_18),
                         ("도중 수평거리", PATH_RADIUS_2026_09_18)):
        check(f"걸음 중 막힘({name}) → path (실기 기록 그대로)",
              m5.classify_stage(detail) == "path", m5.classify_stage(detail))
    check("걷기 전 막힘('막힌다')도 같은 표지로 읽는다",
          m5.classify_stage("5걸음 중 1번째에서 막힌다 — elbow_flex 한계") == "path")
    check("두 길이 다 막히면 path다 ('갈 길이 없습니다')",
          m5.classify_stage(
              "갈 길이 없습니다 — 직선 경로: 3걸음 중 1번째에서 막힌다 — a "
              "/ 관절공간 경로: 17걸음이 필요하다(상한 16걸음)") == "path")
    check("서보가 안 따라와 선 것도 path다 ('더 안 갑니다')",
          m5.classify_stage(
              "4걸음에서 더 안 갑니다 — 목표에서 111mm 떨어진 자리에 섰고 "
              "2걸음째 남은 길이 2.0mm도 안 줄었습니다. 도착 x=339") == "path")
    # ⚠ 문구가 겹치는 자리 — 한 걸음 상한은 걸음 **안에서** 나지만 고치는 법이
    #   다르다(쪼개라 → T30). 그래서 step이 path를 이긴다.
    check("걸음 안에서 난 한 걸음 상한은 그대로 step이다 (path가 안 먹는다)",
          m5.classify_stage(
              "이동 실패 — 3걸음 중 2번째에서 멈췄습니다 — 한 번에 110mm는 "
              "너무 큽니다(상한 80mm).") == "step")
    # ⚠ 성공 문장 끝의 " [직선 경로가 막혀 …]"는 **이미 버린 길**의 이야기다.
    #   안 떼면 성공 줄까지 path로 읽혀 기록이 또 거짓말을 한다.
    check("성공 문장에 달린 '직선이 막혔다' 해설은 실패 이유가 아니다",
          m5.classify_stage(
              "11걸음으로 이동 완료 → x=241 y=166 z=348 · 되먹임 2회 "
              "[직선 경로가 막혀 관절공간으로 돌아갔다(5걸음 중 1번째에서 "
              "막힌다 — elbow_flex 한계)]") != "path")

    # **살아 있는 코드가 만드는 문장**으로 확인한다 — 문구를 다듬다가 분류가
    # 조용히 틀어지는 것이 이 병의 발생 경로였다. self는 안 쓰이므로 None.
    step_msg = ""
    try:
        cart.CartesianArm._check_step(
            None, kin.ToolPose(x=200.0, y=0.0, z=100.0, pitch=0.0),
            kin.ToolPose(x=700.0, y=0.0, z=100.0, pitch=0.0))
    except RuntimeError as exc:
        step_msg = str(exc)
    check("지금 코드가 내는 한 걸음 상한 문장도 step이다",
          step_msg and m5.classify_stage(step_msg) == "step", step_msg[:60])

    reach_msg = ""
    try:
        cart.CartesianArm._check_workspace(
            None, kin.ToolPose(x=900.0, y=0.0, z=100.0, pitch=0.0),
            kin.ArmGeometry())
    except RuntimeError as exc:
        reach_msg = str(exc)
    # ⚠ 이 문장에도 "수평거리"가 들어 있다 — 자세 가드와 같은 낱말이다.
    # 가르는 것은 '목표'다(목표가 무리 ≠ 지금 자세가 무리).
    check("사거리 초과는 pose가 아니라 ik다 ('수평거리'가 겹쳐도)",
          reach_msg and m5.classify_stage(reach_msg) == "ik", reach_msg[:60])

    # **살아 있는 코드가 만드는 경로 문장** — 상수만 박아 두면 cartesian이 문구를
    # 다듬는 순간 분류가 조용히 ik로 돌아간다(이 병의 발생 경로가 그것이었다).
    arm = _sim_escape_arm()
    geom = arm.config.geometry()
    now = arm.pose()
    target = kin.ToolPose(x=PATH_TARGET[0], y=PATH_TARGET[1],
                          z=PATH_TARGET[2], pitch=PATH_TARGET[3])
    ends_ok = True
    try:
        arm._check_workspace(target, geom)                      # noqa: SLF001
        end_degs = kin.inverse(target, geom, elbow_up=cart.ARM_CART_ELBOW_UP,
                               seed_pan=0.0)
        arm._check_joint_limits(arm._to_norm(end_degs), end_degs)   # noqa: SLF001
    except Exception as exc:                                    # noqa: BLE001
        ends_ok, end_degs = False, str(exc)
    check("시험용 목표는 **목표로서는 멀쩡하다**(안 그러면 아래가 헛돈다)",
          ends_ok, str(end_degs)[:70])
    walk_msg = arm._walk(arm._io.read(), now,                   # noqa: SLF001
                         cart.plan_steps(now, target), geom, joint_space=False)
    check("지금 코드가 내는 경로 막힘 문장도 path다",
          bool(walk_msg) and m5.classify_stage(walk_msg) == "path",
          (walk_msg or "안 막혔다")[:70])

    src = open(os.path.join(REPO, "src", "tomato_picker", "hardware",
                            "cartesian.py"), encoding="utf-8").read()
    check("자세 가드가 기대하는 표지를 아직 쓰고 있다",
          all(w in src for w in ("몸통 뒤로", "거의 수직", "수평거리")),
          "문구를 바꾸면 이 검사가 먼저 터진다")
    check("경로 실패가 기대하는 표지를 아직 쓰고 있다",
          all(w in src for w in ("번째에서 막힌다", "번째에서 멈췄습니다",
                                 "걸음에서 더 안 갑니다", "걸음을 걷고도",
                                 "갈 길이 없습니다")),
          "문구를 바꾸면 경로 실패가 다시 ik로 샌다")

    # ⚠ 감사 T39 실측: 이 파일이 없으면 아래 두 check()가 아예 안 불려
    # FAIL 없이 통과 수만 줄어든다(10개→8개, 조용히 사라짐). 그 파일이
    # 저장소에 커밋돼 있으니(사라지면 그 자체가 사고) 존재를 먼저 못 박는다.
    # 날짜를 새 검사로 늘리지 않는 이유: "note가 있다"는 이 특정 사고
    # (09-18 오분류)의 기록이지 미래 시험 전부에 강제할 규칙이 아니다.
    record = os.path.join(REPO, "docs", "시험기록",
                          "move-to-point-2026-09-18.jsonl")
    check("09-18 시험기록 파일이 있다 (없으면 아래 두 검사가 소리없이 사라진다)",
          os.path.exists(record), record)
    if os.path.exists(record):
        rows = [json.loads(ln) for ln in open(record, encoding="utf-8")
                if ln.strip()]
        fails = [r for r in rows if r.get("detail") and not r.get("ok", True)]
        redo = [m5.classify_stage(r["detail"]) for r in fails]
        check("09-18 기록의 실패 줄은 지금 규칙으로 tf가 하나도 없다",
              redo and "tf" not in redo,
              f"{len(redo)}줄 → {sorted(set(redo))}")
        # ⚠ stage="joint"로 적힌 줄은 **이동은 성공했고 오차가 컸던** 줄이라
        #   detail이 성공 문장이다(분류기가 만든 값이 아니다). 여기서 빼지 않으면
        #   성공 문장을 재분류하며 엉뚱한 것을 지키게 된다.
        rejected = [r for r in fails if r.get("stage") != "joint"]
        moved = [r for r in rejected
                 if r.get("stage") == "ik" and m5.classify_stage(r["detail"]) == "path"]
        check("사이클20의 '걸음 중 N번째' 줄이 ik에서 path로 옮겨 갔다",
              len(moved) == 9, f"{len(moved)}줄 (기록에 9줄 있었다)")
        check("걸음 표지를 단 거절 줄은 이제 하나도 ik/pose가 아니다",
              not [r for r in rejected
                   if m5.PATH_STEP_RE.search(r["detail"])
                   and m5.classify_stage(r["detail"]) in ("ik", "pose")])
        check("그 기록에 정정 note가 남아 있다",
              any("note" in r for r in rows))


# ----------------------------------------------------------------------
# ⑪ 먼 좌표로 쪼개서 가기 (travel_to ↔ /arm/move_to_point)
# ----------------------------------------------------------------------

def test_travel_split() -> None:
    """`/arm/move_to_point`가 **상한보다 먼 목표**에 도달할 수 있는가.

    2026-09-18 T26 실기: 572~720mm 요청 5/5가 한 걸음 상한(80mm)에 거절됐다.
    상한은 서보가 한 번에 뛰면 위험해서 있는 값이라 키우지 않는다 — 걸음을
    늘려 푼다. 여기서 박는 것은 두 가지다: ① 쪼개기 계약(걸음 수·걸음 크기)
    ② **노드가 그 길을 쓰고 있는가**(계획만 맞고 배선이 옛 길이면 헛것이다).
    """
    print("\n[쪼개기] 상한보다 먼 목표를 여러 걸음으로")
    from tomato_picker.config import ARM_CART_MAX_STEP_MM as LIMIT  # noqa: E402
    from tomato_picker.hardware.cartesian import plan_steps  # noqa: E402

    now = kin.ToolPose(x=200.0, y=0.0, z=100.0, pitch=0.0)
    for dist in (572.0, 720.0):        # 09-18 실기에서 거절당한 거리 그대로
        steps = plan_steps(now, now.replace(x=now.x + dist))
        hops = [math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                for a, b in zip([now] + steps[:-1], steps)]
        check(f"{dist:.0f}mm는 ceil({dist:.0f}/{LIMIT:.0f})={math.ceil(dist / LIMIT)}걸음 이상",
              len(steps) >= math.ceil(dist / LIMIT), f"걸음={len(steps)}")
        check(f"{dist:.0f}mm의 모든 걸음이 상한 이하",
              max(hops) <= LIMIT + 1e-6, f"최대 {max(hops):.1f}mm")
    check("마지막 걸음은 목표 그 자체다 (근처가 아니다)",
          plan_steps(now, now.replace(x=500.0))[-1].x == 500.0)

    src = open(os.path.join(REPO, "ros2", "src", "tomato_bridge", "tomato_bridge",
                            "arm_source.py"), encoding="utf-8").read()
    body = src.split("class DirectArm")[1].split("class ProxyArm")[0]
    check("DirectArm.move_to가 travel_to를 부른다 (옛 move_to가 아니다)",
          "travel_to(" in body and "._unit().move_to(" not in body,
          "배선이 옛 길로 돌아가면 상한 거절이 그대로 재발한다")


# ----------------------------------------------------------------------
# ⑫ 표적 뽑기 (move5_check.sample_points ↔ 캘리브레이션 가동범위)
# ----------------------------------------------------------------------

def test_sample_within_limits() -> None:
    """뽑은 표적이 **이 팔이 실제로 갈 수 있는 자리**인가.

    2026-09-18(T30) 실기: `/arm/move_to_point` 5회가 5회 다 `elbow_flex`
    정규화 −121~−138(한계 ±98)로 거절됐다. 그 0/5는 팔이 만든 0이 아니라
    **도구가 만든 0**이다 — `sample_points`가 사거리(링크 길이)만 보고 뽑았고,
    가동범위(캘리브레이션)는 다른 것이기 때문이다. 기준3의 "임의의 자리"는
    갈 수 있는 자리라야 뜻이 있다. 그래서 여기서 못 박는다.
    """
    print("\n[표적] move5_check가 뽑는 표적이 가동범위 안인가")
    sys.path.insert(0, os.path.join(REPO, "ros2", "tools"))
    import move5_check as m5  # noqa: E402
    from tomato_picker.hardware import cartesian as cart  # noqa: E402

    f = ESCAPE_FRAME_2026_09_18          # 09-18 젯슨에서 읽은 진짜 보정표
    limits = cart.NormLimits(zero=f["zero"], ref=f["ref"],
                             signs={j: 1.0 for j in f["zero"]},
                             deg_per_norm=f["dpn"], source="테스트(09-18 실측표)")
    geom = kin.ArmGeometry()

    check("한계는 ±(100−여유)다 (cartesian과 같은 값)",
          abs(limits.limit - cart.NORM_LIMIT) < 1e-9, f"±{limits.limit}")

    # ① 한계를 모르면 09-18의 병이 그대로 재현된다 — 이 검사가 무엇을 막는지.
    blind = m5.sample_points(5, geom, 0)
    bad = [p for p in blind if m5.limit_violations(p, geom, limits)]
    check("한계를 모르면 못 가는 표적이 섞인다 (09-18 재현)",
          len(bad) > 0, f"{len(bad)}/5개가 가동범위 밖")

    # ② 한계를 주면 다섯 개가 전부 갈 수 있는 자리다.
    for seed in (0, 7):
        pts = m5.sample_points(5, geom, seed, limits=limits)
        off = {i: v for i, v in
               ((i, m5.limit_violations(p, geom, limits))
                for i, p in enumerate(pts, 1)) if v}
        check(f"seed={seed}: 뽑은 표적 5개의 IK 해가 전부 정규화 한계 안",
              not off, f"밖={off}")

    # ③ **스탠드오프까지 넣어 본다** — 팔이 가는 곳은 표적이 아니라 물러난 자리다
    #    (arm_node._on_move). 표적만 보면 30mm 차이로 통과시켰다 다시 거절당한다.
    pts = m5.sample_points(5, geom, 0, limits=limits)
    moved = []
    for p in pts:
        q = m5.standoff_pose(p)
        moved.append(math.dist((p["x"], p["y"], p["z"]), (q.x, q.y, q.z)))
    check("standoff_pose가 표적에서 스탠드오프만큼 물러난다",
          all(abs(d - m5.STANDOFF_MM) < 1e-6 for d in moved),
          f"이동량={[round(d, 2) for d in moved]}")
    check("한계 판정은 표적이 아니라 스탠드오프 자리를 본다",
          not any(limits.violations(kin.inverse(m5.standoff_pose(p), geom))
                  for p in pts))

    # ③b 작업영역 가드(바닥·몸통·사거리)도 **뽑는 쪽이 같은 숫자를 본다**.
    #    09-18 실기 5번째는 관절은 멀쩡했는데 표적 수평 76mm < 90mm로 거절됐다.
    from tomato_picker.config import (ARM_CART_R_MIN,  # noqa: E402
                                      ARM_CART_Z_MIN)
    for p in m5.sample_points(8, geom, 3, limits=limits):
        stand = m5.standoff_pose(p)
        check("뽑은 자리가 바닥·몸통·사거리 가드를 지난다",
              not m5.workspace_reject(stand, geom),
              f"{m5.workspace_reject(stand, geom)} (z최소 {ARM_CART_Z_MIN}, "
              f"r최소 {ARM_CART_R_MIN})")
    check("가드 숫자는 config에서 온다 (도구에 박지 않는다)",
          m5.ARM_CART_Z_MIN == ARM_CART_Z_MIN and m5.ARM_CART_R_MIN == ARM_CART_R_MIN)
    check("바닥 아래는 거절한다",
          "바닥" in m5.workspace_reject(
              kin.ToolPose(x=250.0, y=0.0, z=ARM_CART_Z_MIN - 1.0, pitch=0.0), geom))
    check("몸통 안쪽은 거절한다",
          "몸통" in m5.workspace_reject(
              kin.ToolPose(x=ARM_CART_R_MIN - 1.0, y=0.0, z=100.0, pitch=0.0), geom))

    # ④ 한계를 모르면 **모른다고 말한다** — 짐작으로 메우면 조용히 틀린다.
    nowhere = os.path.join(REPO, "ros2", "tools", "__없는파일__.json")
    none_limits, note = cart.load_norm_limits(path=nowhere, root=nowhere)
    check("영점이 없으면 NormLimits는 None이고 이유를 말한다",
          none_limits is None and "영점" in note, note)

    # ⑤ 정규화↔각도 식이 팔이 있을 때와 없을 때 **같은 줄**인가.
    signs = {j: 1.0 for j in f["zero"]}
    degs = cart.norms_to_degrees({"elbow_flex": -98.0}, zero=f["zero"], ref=f["ref"],
                                 signs=signs, deg_per_norm=f["dpn"])
    back = cart.degrees_to_norms(degs, zero=f["zero"], ref=f["ref"],
                                 signs=signs, deg_per_norm=f["dpn"])
    check("정규화 −98 = elbow_flex −38.9° (이 팔의 굽힘 끝)",
          abs(degs["elbow_flex"] + 38.9) < 0.1, f"{degs['elbow_flex']:.2f}°")
    check("각도→정규화가 왕복한다", abs(back["elbow_flex"] + 98.0) < 1e-9)

    # ⑤b **들 수 있는 자리인가** — 가동범위를 지나고도 서보가 못 버티는 자리가 있다.
    #     2026-09-18 실기(§24): r=331mm의 자세는 IK도 가동범위도 통과하는데 어깨가
    #     한계에 눌려 z가 73mm 처진 채 끝났다. 그런 자리를 뽑으면 기준3의 실패에
    #     **도구가 만든 실패**가 섞인다(T30·T31이 고친 것과 같은 병의 다음 겹).
    from tomato_picker.config import ARM_LOAD_R_MAX  # noqa: E402
    from tomato_picker.hardware import load_limits as ld  # noqa: E402

    def m5_source() -> str:
        return open(os.path.join(REPO, "ros2", "tools", "move5_check.py"),
                    encoding="utf-8").read()

    def stand_r(p: dict) -> float:
        q = m5.standoff_pose(p)
        return math.hypot(q.x, q.y)

    # 250mm를 고른 이유 — 이 seed에서 표적(스탠드오프 바깥)이 250을 넘는 자리가
    # 섞인다. 아래 "표적이 아니라 서는 자리를 본다"가 그 차이로 판정된다.
    tight = ld.LoadLimits(r_max=250.0, source="테스트")
    picked = m5.sample_points(8, geom, 0, limits=limits, load=tight)
    over = [round(stand_r(p), 1) for p in picked if stand_r(p) > tight.r_max + 1e-6]
    check("경계 밖 자리는 안 뽑힌다", not over, f"경계 {tight.r_max}mm 밖={over}")
    blind_r = [round(stand_r(p), 1) for p in m5.sample_points(8, geom, 0, limits=limits)]
    check("경계를 안 주면 밖이 섞인다 (이 검사가 무엇을 막는지)",
          any(r > tight.r_max for r in blind_r), f"stand r={blind_r}")
    # 판정은 **팔이 서는 자리**다 — 표적은 스탠드오프만큼 더 바깥이라 넘어도 된다.
    check("경계는 표적이 아니라 팔이 서는 자리를 본다",
          any(math.hypot(p["x"], p["y"]) > tight.r_max for p in picked),
          "표적까지 경계 안이면 30mm를 두 번 자르고 있는 것이다")

    # 숫자의 출처 — 도구에 박으면 다시 잰 값이 안 먹는다(§24는 아직 표본 9개다).
    got, note = ld.load_load_limits(path="/없는파일/arm_load_limits.json")
    check("파일이 없으면 config의 실측값을 쓴다",
          got is not None and got.r_max == ARM_LOAD_R_MAX, note)
    check("경계 숫자가 move5_check에 박혀 있지 않다",
          "310" not in m5_source(),
          "경계는 config 또는 ~/arm_load_limits.json에서만 온다")
    check("09-18 실측 경계는 못 든 자리(r=331)를 거절한다",
          bool(ld.LoadLimits(ARM_LOAD_R_MAX, "테스트").rejects(331.0, 0.0)),
          f"한계 {ARM_LOAD_R_MAX}mm")
    check("그러면서 든 자리(r=308)는 통과시킨다",
          not ld.LoadLimits(ARM_LOAD_R_MAX, "테스트").rejects(308.0, 0.0))

    # 모르면 **모른다고 말한다** — 깨진 파일을 보고 조용히 config로 돌아가면,
    # 넓히려고 쓴 파일이 무시된 줄 모른 채 옛 경계로 시험하게 된다.
    tmp = os.path.join(REPO, "ros2", "tools", "__t41_load_tmp.json")
    try:
        for body, why in (('{"r_max_mm": null}', "null"),
                          ('{"r_max_mm": 0}', "0 이하"),
                          ('{"note": "재는 중"}', "키가 없다"),
                          ("{깨진 json", "깨진 파일")):
            open(tmp, "w", encoding="utf-8").write(body)
            got, note = ld.load_load_limits(path=tmp)
            check(f"경계를 못 읽으면 None이고 이유를 말한다 ({why})",
                  got is None and len(note) > 0, note)
        open(tmp, "w", encoding="utf-8").write('{"r_max_mm": 321.5}')
        got, note = ld.load_load_limits(path=tmp)
        check("파일이 config를 이긴다 (이 팔에서 다시 잰 값)",
              got is not None and abs(got.r_max - 321.5) < 1e-9 and got.source == tmp,
              note)
    finally:
        os.path.exists(tmp) and os.remove(tmp)

    # 기록에 남는가 — 남지 않으면 그 0/5가 팔의 0인지 도구의 0인지 못 가린다.
    m5_body = m5_source()
    check("기록 줄에 load_limits가 들어간다",
          m5_body.count('"load_limits": load_tag') >= 2,
          "dry-run과 실기 둘 다 남겨야 한다")

    # ⑥ **노드가 붙으면서 팔을 놓지 않는가.** 2026-09-18: `arm_extend`로 z=+423mm
    #    까지 세워 둔 팔이 arm_node가 뜨자 z=−66mm로 주저앉았다 —
    #    `FollowerIO`의 기본값이 connect 직후 `disable_torque()`를 부르기 때문이다.
    #    젯슨 도구 스무 개가 전부 hold_torque=True인데 정작 팔의 주인만 아니었고,
    #    그래서 기준3을 잴 때마다 **시작 자세가 이미 무너져 있었다**.
    src = open(os.path.join(REPO, "ros2", "src", "tomato_bridge", "tomato_bridge",
                            "arm_source.py"), encoding="utf-8").read()
    body = src.split("class DirectArm")[1].split("class ProxyArm")[0]
    check("DirectArm이 팔을 hold_torque=True로 연다 (붙으면서 놓지 않는다)",
          "hold_torque=True" in body,
          "기본값은 connect 직후 토크를 끈다 — 팔이 주저앉는다")


def test_selfcheck_deps() -> None:
    """자체검증 4종이 **빈 환경에서 무엇이 없는지 말하고** 죽는가.

    이 검사가 있는 이유: 2026-09-17에 `.venv`에 pyyaml이 없어 이 스크립트가
    ModuleNotFoundError로 죽고 있었고, 그게 "통과"로 오독될 뻔했다. 졸업
    기준 4번은 넷이 **실제로 도는 것**을 전제한다 — 안 돌았으면 그 사실이
    한 줄로 보여야 한다. 여기서 지키는 것은 두 가지다: ① 필요한 것이
    requirements.txt에 적혀 있는가 ② 서드파티를 import하기 **전에** 물어보는가.
    """
    print("\n[의존성] 빈 환경에서 무엇이 없는지 말하고 죽는가")
    sys.path.insert(0, os.path.join(REPO, "tools"))
    import selfcheck_deps as sd  # noqa: E402

    req = open(os.path.join(REPO, "requirements.txt"), encoding="utf-8").read().lower()
    for mod in sd.SELFCHECK_MODULES:
        pip = sd.PIP_NAME[mod]
        check(f"requirements.txt가 {pip}를 적어 뒀다", pip.lower() in req,
              f"{pip}가 없다 — 새 환경에서 자체검증이 안 돈다")

    # 있는 것은 통과시키고, 없는 것은 잡는가. (find_spec이라 import 부작용 없음)
    check("있는 모듈은 통과시킨다", sd.missing("os", "sys") == [])
    check("없는 모듈은 잡아낸다",
          sd.missing("정말_없는_모듈_ㄱㄴㄷ") == ["정말_없는_모듈_ㄱㄴㄷ"])

    # ⚠ 종료코드 2 = 환경이 없다. 1(검사 실패)과 섞으면 "고칠 코드"와
    #   "깔 패키지"가 구분이 안 된다.
    # stderr를 삼킨다 — 안 그러면 이 **일부러 실패시키는** 호출이 뱉는
    # "❌ 자체검증을 돌릴 수 없다"가 통과 출력 한가운데 섞여 진짜 사고처럼 보인다.
    hushed, sys.stderr = sys.stderr, io.StringIO()
    try:
        sd.require("정말_없는_모듈_ㄱㄴㄷ")
        rc, said = 0, ""
    except SystemExit as e:
        rc, said = e.code, sys.stderr.getvalue()
    finally:
        sys.stderr = hushed
    check("없으면 종료코드 2로 죽는다", rc == 2, f"rc={rc}")
    check("죽으면서 설치 명령을 알려준다", "pip install" in said, said.strip()[:60])
    check("import 이름과 pip 이름이 다른 것을 표가 안다",
          sd.PIP_NAME["yaml"] == "PyYAML", "yaml을 `pip install yaml`로 안내하면 못 깐다")

    # 배선 — 스크립트마다 **무엇을 묻는지**가 실제로 필요한 것과 같아야 한다.
    # ⚠ 이 표는 파일 머리의 import를 훑어서 만든 게 아니라 **빈 venv에서 돌려
    #   보고** 적었다(2026-09-17). eye_check의 cv2가 그 차이다 — 늦은 import라
    #   머리에는 안 보이고 검사 뒤쪽에서 죽는다. 표를 고칠 일이 생기면
    #   빈 venv에서 한 번 돌려 보고 고쳐라.
    NEEDS = {
        "tools/handeye_check.py": {"numpy"},
        "tools/eye_check.py": {"numpy", "cv2"},
        "ros2/tools/ros_selfcheck.py": {"numpy", "yaml"},
        "tools/arm_cartesian_check.py": set(),   # 표준 라이브러리만 쓴다
    }
    for rel, needs in NEEDS.items():
        body = open(os.path.join(REPO, *rel.split("/")), encoding="utf-8").read()
        lines = body.splitlines()
        third = [i for i, ln in enumerate(lines)
                 if re.match(r"^\s*(import|from)\s+(numpy|yaml|cv2)\b", ln)]
        guard = [i for i, ln in enumerate(lines) if re.match(r"^\s*require\(", ln)]
        asked = set(re.findall(r'"([a-z0-9_]+)"',
                               " ".join(lines[i] for i in guard)))
        check(f"{rel} — 묻는 목록이 필요한 것과 같다", asked == needs,
              f"묻는 것={sorted(asked)} / 필요한 것={sorted(needs)}")
        if not needs:
            check(f"{rel} — 서드파티를 안 쓴다", not third,
                  "서드파티가 끼어들었다면 require()도 함께 넣어라")
            continue
        # 묻기 전에 import하면 묻는 의미가 없다(역추적이 먼저 뜬다).
        check(f"{rel} — 서드파티를 묻고 나서 import한다",
              not third or (bool(guard) and guard[0] < third[0]),
              f"require 줄={guard} / 서드파티 줄={third}")

    # 표에 적은 것이 전부 requirements.txt에 있는가 — 위 표만 늘리고 설치
    # 목록을 안 고치면 "묻기는 하는데 깔 방법은 안 적힌" 상태가 된다.
    all_needed = set().union(*NEEDS.values())
    check("스크립트가 묻는 것이 전부 SELFCHECK_MODULES에 있다",
          all_needed <= set(sd.SELFCHECK_MODULES),
          f"빠진 것={sorted(all_needed - set(sd.SELFCHECK_MODULES))}")


def test_mount_contract() -> None:
    """마운트 대조가 **실제로 실패할 수 있는가** — tf_check.py [마운트] 절의 계산.

    이 검사가 있는 이유: 그 절은 2026-09-18까지 출력 문구만 "yaml과 같은가"라
    말하고 실제로는 *0이 아니기만* 하면 통과시켰다(감사 T29). 종수는 하나 늘었지만
    잡는 것은 없는 검사였다. 그래서 여기서는 통과만 보지 않고 **틀린 값을 넣어
    실패하는지**까지 본다 — 검사는 실패할 수 있을 때만 뜻이 있다.

    ⚠ tf_check.py 자체는 rclpy를 물어 PC에서 못 돈다. 그래서 계산만
    mount_compare.py로 떼어 놨고 여기서 그 함수를 직접 시험한다.
    """
    print("\n[마운트] TF ↔ so101_geometry.yaml 대조가 실패할 수 있는가")
    sys.path.insert(0, HERE)
    import mount_compare as mc  # noqa: E402

    def qz(deg: float) -> tuple[float, float, float, float]:
        h = math.radians(deg) / 2.0
        return (0.0, 0.0, math.sin(h), math.cos(h))

    def qx(deg: float) -> tuple[float, float, float, float]:
        h = math.radians(deg) / 2.0
        return (math.sin(h), 0.0, 0.0, math.cos(h))

    def verdict(mount: dict, xyz, quat=(0.0, 0.0, 0.0, 1.0)) -> dict:
        return {name: ok for name, ok, _ in mc.compare(mount, xyz, quat)}

    GOOD = {"x": 60.0, "y": 0.0, "z": 76.5, "yaw_deg": 0.0}
    TRUTH = (60.0, 0.0, 76.5)

    check("맞는 값은 통과한다", all(verdict(GOOD, TRUTH).values()),
          f"{TRUTH} vs {GOOD}")

    # 축마다 따로 본다 — 한 축만 틀린 것을 다른 축이 덮으면 안 된다.
    for axis, idx in mc.AXES:
        far = list(TRUTH); far[idx] += 1.5     # 허용(1mm)보다 크게
        near = list(TRUTH); near[idx] += 0.5   # 허용 안쪽
        check(f"mount.{axis}가 1.5mm 틀리면 실패한다",
              not verdict(GOOD, tuple(far))[f"mount.{axis}"], f"TF {far}")
        check(f"mount.{axis}가 0.5mm 어긋난 것은 통과시킨다",
              verdict(GOOD, tuple(near))[f"mount.{axis}"], f"TF {near}")

    check("요가 0.6도 틀리면 실패한다",
          not verdict(GOOD, TRUTH, qz(0.6))["mount.yaw_deg"], "허용 0.5도")
    check("요가 0.3도 어긋난 것은 통과시킨다",
          verdict(GOOD, TRUTH, qz(0.3))["mount.yaw_deg"])
    # 180과 -180은 **같은 방향**이다. 접지 않으면 여기서 360도가 나와 헛실패한다.
    check("요 180도와 -180도를 같게 본다",
          verdict(dict(GOOD, yaw_deg=180.0), TRUTH, qz(-180.0))["mount.yaw_deg"],
          f"차이 {mc.angle_diff_deg(-180.0, 180.0):.1f}도")
    check("마운트가 2도 기울면 실패한다",
          not verdict(GOOD, TRUTH, qx(2.0))["마운트가 기울지 않았다 (roll=pitch=0)"],
          "xacro는 rpy=0 0 yaw로 박혀 있다 — 기울었으면 딴 경로가 끼어든 것이다")

    # 키가 없으면 런치가 조용히 건너뛰고 xacro default가 이긴다 → 통과가 아니다.
    for key, name in (("x", "mount.x"), ("y", "mount.y"), ("z", "mount.z"),
                      ("yaw_deg", "mount.yaw_deg")):
        missing = {k: v for k, v in GOOD.items() if k != key}
        check(f"yaml에 mount.{key}가 없으면 실패한다",
              not verdict(missing, TRUTH)[name],
              "런치가 없는 키를 건너뛰면 yaml이 아무것도 안 정한 상태가 된다")

    # 정본 yaml이 네 값을 실제로 갖고 있는가 + 자기 자신과는 당연히 맞는가.
    real, path = mc.load_mount(mc.geometry_paths(None, ws_src=SRC))
    check("so101_geometry.yaml에 mount 네 값이 다 있다",
          {"x", "y", "z", "yaw_deg"} <= set(real), f"{sorted(real)} · {os.path.basename(path)}")
    check("그 값을 그대로 넣으면 대조가 통과한다",
          all(verdict(real, (float(real["x"]), float(real["y"]), float(real["z"])),
                      qz(float(real["yaw_deg"]))).values()),
          f"x={real.get('x')} y={real.get('y')} z={real.get('z')} yaw={real.get('yaw_deg')}")

    # 배선 — yaml이 xacro까지 실제로 흘러가는가, 그리고 tf_check가 이 계산을 쓰는가.
    launch = open(os.path.join(SRC, "tomato_description", "launch",
                               "description.launch.py"), encoding="utf-8").read()
    check("런치가 mount 네 값을 xacro 인자로 넘긴다",
          all(a in launch for a in ("mount_x_mm", "mount_y_mm", "mount_z_mm",
                                    "mount_yaw_deg")),
          "안 넘기면 xacro default가 이겨서 yaml을 고쳐도 TF가 안 바뀐다")
    tf_body = open(os.path.join(ROS2, "tools", "tf_check.py"), encoding="utf-8").read()
    check("tf_check.py가 값을 비교한다 (0이 아닌지만 보지 않는다)",
          "mount_compare.compare" in tf_body and "1e-6 for v in mount" not in tf_body,
          "문구만 '같은가'이고 실제로는 존재만 보던 것이 감사 T29의 발견이다")


def _eol_patterns() -> tuple[list[str], set[str]]:
    """.gitattributes에서 **LF로 못 박은 패턴**과 바이너리 선언을 읽어 온다.

    패턴을 여기 박지 않는 이유: .gitattributes에 종류를 하나 더하면 검사 범위도
    같이 늘어야 한다. 두 곳에 적으면 한 곳만 늘어난다.
    """
    lf, binary = [], set()
    path = os.path.join(REPO, ".gitattributes")
    if not os.path.exists(path):
        return lf, binary
    for line in open(path, encoding="utf-8"):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        pat, *attrs = line.split()
        if "eol=lf" in attrs:
            lf.append(pat)
        if "binary" in attrs or "-text" in attrs:
            binary.add(pat)
    return lf, binary


# 검사 대상이 아닌 곳 — 가상환경·빌드산출물, 그리고 3D(바이너리만 있다).
EOL_SKIP_DIRS = {".git", ".venv", "__pycache__", ".work", "node_modules",
                 "build", "install", "log", "3D"}


def _worktree_files(patterns: list[str]) -> list[str]:
    """작업트리에서 패턴에 걸리는 파일. git 없이 돈다(컨테이너·빈 체크아웃에서도)."""
    hits = []
    for root, dirs, names in os.walk(REPO):
        dirs[:] = [d for d in dirs if d not in EOL_SKIP_DIRS]
        for name in names:
            if any(fnmatch.fnmatch(name, pat) for pat in patterns):
                hits.append(os.path.join(root, name))
    return hits


def test_record_timezone() -> None:
    """시험기록의 **날짜가 어느 시각대의 날짜인가** — 컨테이너 기본값은 UTC다.

    이 검사가 있는 이유: 기록 이름이 `move-to-point-<날짜>.jsonl`인데 도커
    컨테이너는 UTC로 돈다. 한국 자정~09시에 실기를 돌리면 **어제 파일**에
    줄이 붙고, 다음 사이클은 그것을 어제 시험으로 읽는다(실측 2026-09-18
    03:10 KST → 컨테이너 09-17 18:10 UTC). 고친 자리는 둘이다 —
    compose가 호스트의 시각대를 물리고, 실행이 스스로 시각대를 말한다.
    """
    print("\n[시각] 시험기록 날짜가 젯슨의 날짜인가")
    compose = open(os.path.join(ROS2, "docker", "docker-compose.yml"),
                   encoding="utf-8").read()
    # 존 이름을 적는 대신 호스트의 것을 물린다 — 젯슨을 옮기면 젯슨만 고친다.
    check("compose가 /etc/localtime을 읽기전용으로 물린다",
          "/etc/localtime:/etc/localtime:ro" in compose,
          "없으면 컨테이너가 UTC로 돌아 기록이 하루 전 이름으로 열린다")
    check("compose가 /etc/timezone도 물린다",
          "/etc/timezone:/etc/timezone:ro" in compose,
          "이름을 찍는 쪽(%Z)이 UTC라고 말한다")

    sys.path.insert(0, os.path.join(ROS2, "tools"))
    import move5_check as m5  # noqa: E402

    zone = m5.local_zone()
    check("local_zone()이 UTC 오프셋을 말한다", bool(re.search(r"UTC[+-]\d", zone)),
          zone)
    # 마운트를 빼고 `docker run`으로 띄우면 다시 UTC다 — 그때도 출력만 보고
    # 알 수 있어야 한다. 그래서 기록 경로를 찍는 그 줄에 붙여 둔다.
    body = open(os.path.join(ROS2, "tools", "move5_check.py"),
                encoding="utf-8").read()
    printed = [ln for ln in body.splitlines()
               if "기록: {out_path}" in ln or "기록: " in ln and "out_path" in ln]
    check("기록 경로를 찍는 줄에 시각대가 붙어 있다",
          bool(printed) and all("local_zone()" in ln for ln in printed),
          f"{printed}")


def test_line_endings() -> None:
    """작업트리가 **LF로 체크아웃되는가** — 이 저장소는 Windows에서 고쳐 젯슨으로 scp한다.

    이 검사가 있는 이유: 2026-09-18 사이클25에 컨테이너 bash가
    `/ws/tools/bringup_check.sh`를 통째로 못 읽었다(`set: pipefail: invalid
    option name`). 원인은 스크립트가 아니라 **줄끝**이었다 — core.autocrlf=true인
    Windows 작업트리가 CRLF로 체크아웃하고 scp는 바이트를 그대로 보낸다.
    졸업기준 1번을 재는 bringup_check가 그것 하나로 통째로 막혀 있었으니 이건
    한 번의 실수가 아니라 **배포 경로의 성질**이다. 그래서 여기서 못 박는다.

    ⚠ 저장소(index)는 원래도 LF였다. 깨지는 것은 늘 **작업트리**다 — 그러니
       검사도 작업트리 바이트를 본다(`git show`를 보면 항상 통과한다).
    """
    print("\n[줄끝] 배포되는 파일이 작업트리에서도 LF인가")
    lf_pats, bin_pats = _eol_patterns()

    # 셸·파이썬·systemd 유닛은 CRLF에서 *조용히* 깨진다(유닛은 ExecStart 끝에
    # CR이 붙어 실행 파일 이름이 틀린다). 이 셋은 규칙이 있어야 한다.
    for must in ("*.sh", "*.py", "*.service"):
        check(f".gitattributes가 {must}를 LF로 못 박았다", must in lf_pats,
              f"LF 패턴={lf_pats}")
    # 3D 모델·사진은 CR이 데이터다 — text로 잡으면 파일이 망가진다.
    for must in ("*.stl", "*.jpg"):
        check(f".gitattributes가 {must}를 바이너리로 뺐다", must in bin_pats,
              f"바이너리 패턴={sorted(bin_pats)}")

    files = _worktree_files(lf_pats) if lf_pats else []
    check("LF로 못 박은 종류의 파일을 실제로 찾았다", len(files) > 20, f"{len(files)}개")
    crlf = [os.path.relpath(f, REPO).replace(os.sep, "/")
            for f in files if CR in open(f, "rb").read()]
    check("그 파일들에 CR이 하나도 없다", not crlf,
          f"{len(crlf)}개가 CRLF다: {crlf[:5]} — 고치는 법: "
          "git add --renormalize . 뒤 그 파일을 지우고 git checkout -- <파일>")

    # 젯슨에서 실제로 도는 스크립트는 이름으로도 못 박는다 — 위 패턴이 지워져도
    # 이 셋은 남는다(막힌 것이 bringup_check였다).
    for rel in ("ros2/tools/bringup_check.sh", "ros2/docker/entrypoint.sh",
                "deploy/astra-install.sh"):
        raw = open(os.path.join(REPO, *rel.split("/")), "rb").read()
        first = raw.split(b"\n", 1)[0]
        check(f"{rel}가 LF다", CR not in raw,
              "컨테이너·젯슨 bash가 이 파일을 통째로 못 읽는다")
        # 셔뱅 줄만 CR이 붙어도 커널이 인터프리터 이름을 못 찾는다.
        check(f"{rel}의 셔뱅 줄에 CR이 없다",
              first.startswith(b"#!") and CR not in first,
              first[:40].decode("utf-8", "replace"))


# 한 장치·한 포트를 놓고 다투는 유닛 묶음. 이름만 적는다 — 무엇을 나눠 쓰는지는
# 사람이 읽을 사유이고, 검사가 보는 것은 "서로를 Conflicts로 적었는가"다.
EXCLUSIVE_UNITS = {
    "포트 8090 + 팔 포트(/dev/ttyACM0)":
        ("tomato-voice.service", "click-server.service"),
}


def _unit_text(name: str) -> str:
    return open(os.path.join(REPO, "deploy", name), encoding="utf-8").read()


def _unit_directive(body: str, key: str) -> list[str]:
    """유닛 본문에서 `Key=값`을 모은다. 주석(`#`)으로 시작하는 줄은 뺀다 —
    설명에 적어 둔 이름이 선언으로 세어지면 검사가 거짓말을 한다."""
    out = []
    for line in body.splitlines():
        t = line.strip()
        if t.startswith("#") or "=" not in t:
            continue
        k, _, v = t.partition("=")
        if k.strip() == key:
            out.extend(p for p in v.replace(",", " ").split() if p)
    return out


def test_service_exclusivity() -> None:
    """같은 것을 나눠 쓰는 유닛이 **systemd 수준에서** 서로를 밀어내는가.

    이 검사가 있는 이유: `tomato-voice`와 `click-server`는 포트 8090과 팔
    포트를 **둘 다** 놓고 다투는데 systemd에는 그 사실이 적혀 있지 않았다.
    둘 다 enabled라 부팅마다 진 쪽이 3초 간격으로 되살아났고, 재부팅 한 번에
    **2425번** bind에 실패했다(2026-09-18 젯슨 실측). 더 나쁜 것은 그 뒤로
    `systemctl start tomato-voice`가 *조용히* 실패했다는 것이다 — 실기 작업의
    "끝나면 되살린다"가 전부 거짓이 됐다.

    Conflicts를 적으면 systemd가 먼저 상대를 내리고 띄운다. 문서로는 09-03부터
    "꺼져 있어야 한다"고 적혀 있었지만 사람이 매번 지켜야 하는 규칙이었다.
    ⚠ 검사가 못 보는 것: 젯슨에서 무엇이 enabled인가. 그건 `systemctl
      is-enabled`로만 알 수 있다(운영 정책 = docs/인수인계-2026-09-03.md §1).
    """
    print("\n[서비스] 같은 장치를 다투는 유닛이 서로를 밀어내는가")
    for why, units in EXCLUSIVE_UNITS.items():
        for name in units:
            body = _unit_text(name)
            others = [u for u in units if u != name]
            declared = _unit_directive(body, "Conflicts")
            check(f"{name}가 Conflicts로 {others}를 적었다",
                  all(o in declared for o in others),
                  f"{why} — 선언된 것: {declared}")
            # Conflicts는 [Unit] 절의 지시어다. [Service]에 적으면 systemd가
            # 통째로 무시하고 유닛은 그대로 뜬다(조용한 실패).
            head = body.split("[Service]", 1)[0]
            check(f"{name}의 Conflicts가 [Unit] 절에 있다",
                  "Conflicts=" in head,
                  "[Service]에 적으면 systemd가 무시한다")

    # 못 고치는 이유로 죽는 것을 영원히 되풀이하지 않게 — 진 쪽에만 있으면 된다.
    voice = _unit_text("tomato-voice.service")
    burst = _unit_directive(voice, "StartLimitBurst")
    interval = _unit_directive(voice, "StartLimitIntervalSec")
    check("tomato-voice에 재시작 상한(StartLimitBurst)이 있다",
          bool(burst) and int(burst[0]) > 0, f"{burst}")
    check("tomato-voice에 그 상한을 재는 창(StartLimitIntervalSec)이 있다",
          bool(interval) and int(interval[0]) > 0, f"{interval}")
    # 상한은 [Unit] 절에서만 먹는다([Service]에 적으면 최신 systemd가 경고만
    # 내고 무시한다) — Restart=는 반대로 [Service]다. 둘을 바꿔 적기 쉽다.
    check("그 상한이 [Unit] 절에 있다",
          "StartLimitBurst=" in voice.split("[Service]", 1)[0],
          "[Service]에 적으면 먹지 않는다")

    # 운영 정책(어느 쪽이 부팅 자동실행인가)은 코드가 아니라 문서에 산다.
    # 문서가 사라지면 다음 사람이 둘 다 enable 해서 같은 병이 돌아온다.
    policy = open(os.path.join(REPO, "docs", "인수인계-2026-09-03.md"),
                  encoding="utf-8").read()
    check("운영 정책이 문서에 적혀 있다(어느 쪽이 부팅 자동실행인가)",
          "Conflicts" in policy and "disable tomato-voice" in policy,
          "docs/인수인계-2026-09-03.md §1")


def main() -> int:
    print(f"저장소: {REPO}")
    geom = kin.ArmGeometry()
    print(f"링크 길이: z0={geom.z0} d0={geom.d0} l1={geom.l1} l2={geom.l2} "
          f"l3={geom.l3} → 사거리 {geom.reach_max:.0f}mm")

    test_rclpy_isolation()
    test_legacy_boundary()
    test_geometry_matches()
    test_urdf_matches_kinematics()
    test_board_contract()
    test_fruit3d()
    test_tf_math()
    test_handeye_gate()
    test_handeye_identifiability()
    test_arm_extend_escape()
    test_prep_autoextend()
    test_stage_classify()
    test_travel_split()
    test_sample_within_limits()
    test_mount_contract()
    test_record_timezone()
    test_line_endings()
    test_service_exclusivity()
    test_selfcheck_deps()

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
