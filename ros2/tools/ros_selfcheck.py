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

import ast
import fnmatch
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap
import time
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
    MAX_SPREAD_MM, Blob, core_mask, read_all, read_blob,
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
    mount_cfg = _geometry_yaml().get("mount", {})
    default = kin.ArmGeometry()
    for key in ("z0", "d0", "l1", "l2", "l3"):
        check(f"{key} 일치", abs(float(cfg[key]) - getattr(default, key)) < 1e-9,
              f"yaml {cfg[key]} vs 코드 {getattr(default, key)}")

    # ⚠ 계측 검증 (단위·부호 규약)
    check("yaml 링크 단위가 mm다 (m 단위 오독 방지: l1/l2/l3/z0 > 10)",
          cfg["l1"] > 10.0 and cfg["l2"] > 10.0 and cfg["l3"] > 10.0 and cfg["z0"] > 10.0,
          f"l1={cfg['l1']}, l2={cfg['l2']}, l3={cfg['l3']}, z0={cfg['z0']}")
    check("d0는 음수다 (2번 lift 축이 1번 pan 축 뒤에 있다)",
          cfg["d0"] < -10.0,
          f"d0={cfg['d0']} — 양수면 팔이 앞으로 쏠려 모델링된다")
    check("mount.z 단위가 mm다 (z > 10)",
          float(mount_cfg.get("z", 0.0)) > 10.0,
          f"mount.z={mount_cfg.get('z')}")

    yraw = open(os.path.join(SRC, "tomato_description", "config", "so101_geometry.yaml"),
                encoding="utf-8").read()
    check("so101_geometry.yaml 링크 길이에 2026-08-31 실측 주석이 있다",
          "실측" in yraw and "2026-08-31" in yraw)
    check("mount.x는 아직 미실측(T12) 상태임이 명시돼 있다",
          "아직 안 쟀다" in yraw)

    rr_src = open(os.path.join(ROS2, "tools", "roll_rehome.py"), encoding="utf-8").read()
    check("roll_rehome이 grip_uv 및 arm_eye 무효화 가능성을 경고한다",
          "grip_uv" in rr_src and "arm_eye" in rr_src)

    sz_src = open(os.path.join(ROS2, "tools", "set_zero.py"), encoding="utf-8").read()
    check("set_zero가 grip_uv 및 arm_eye 무효화 가능성을 경고한다",
          "grip_uv" in sz_src and "arm_eye" in sz_src)
    check("set_zero.py에 미정의 변수 cart 참조가 없다 (FrameConfig 정합성)",
          "cart.get" not in sz_src and ("cfg._data" in sz_src or "cfg.deg_per_norm_override" in sz_src),
          "FrameConfig API로 런타임 NameError 방지")
    check("set_zero.py가 옛 영점 차이와 검증에 동일한 per_scale(deg_per_norm_override)을 사용한다",
          "def per_scale" in sz_src and "per_scale(j)" in sz_src,
          "옛 영점 편차 각도 왜곡 방지 및 서보 스케일 통일")

    # 교시 자세 및 영점 기구학 규약 검증
    from tomato_picker.config import ARM_CART_ZERO_POSE_DEG
    pose_teach = kin.forward(ARM_CART_ZERO_POSE_DEG, default)
    z_reach_up = default.z0 + default.l1 + default.l2 + default.l3
    check("교시 자세(ARM_CART_ZERO_POSE_DEG)의 TCP x가 d0(-31.5mm)와 일치한다",
          abs(pose_teach.x - default.d0) < 1e-9 and abs(pose_teach.y) < 1e-9,
          f"pose_teach.x={pose_teach.x:.2f}, d0={default.d0}")
    check("교시 자세의 TCP z가 수직 최대 도달 높이(542.0mm)와 일치한다",
          abs(pose_teach.z - z_reach_up) < 1e-9 and abs(pose_teach.pitch - 90.0) < 1e-9,
          f"pose_teach.z={pose_teach.z:.2f}, reach_up={z_reach_up}")

    pose_zero = kin.forward({j: 0.0 for j in kin.JOINTS}, default)
    check("관절 0도(수평 정면) 자세의 TCP x가 최대 수평 사거리(391.0mm)와 일치한다",
          abs(pose_zero.x - default.reach_max) < 1e-9 and abs(pose_zero.z - default.z0) < 1e-9,
          f"pose_zero.x={pose_zero.x:.2f}, reach_max={default.reach_max}")

    # 실측 프레임 deg_per_norm 전 관절 물리 서보 범위 검증 (0.5 ~ 2.0 deg/norm)
    dpn = ESCAPE_FRAME_2026_09_18["dpn"]
    dpn_ok = all(0.5 <= dpn[j] <= 2.0 for j in kin.JOINTS)
    check("실측 프레임(ESCAPE_FRAME) deg_per_norm 전 관절이 물리 서보 범위(0.5~2.0도/단위) 안이다",
          dpn_ok, f"dpn={dpn}")

    check("so101_geometry.yaml의 base.length에 가정치 및 T12 실측 주의 주석이 있다",
          "가정치" in yraw and "L" in yraw)

    t_t10 = np.array([-52.0, 0.0, 59.0])
    norm_t10 = float(np.linalg.norm(t_t10))
    check("T10 실측 |t|(78.6mm)가 집게 링크 l3(168mm) 기하 상한보다 작다",
          norm_t10 < float(cfg["l3"]), f"|t|={norm_t10:.2f}mm < l3={cfg['l3']}mm")
    norm_t10_alt = float(np.linalg.norm(np.array([-52.0, 18.0, 59.0])))
    check("D405 18mm 창 애매성 후보(|t|=80.7mm)도 물리 허용 범위(60~100mm) 안이다",
          60.0 <= norm_t10_alt <= 100.0, f"|t|={norm_t10_alt:.2f}mm")

    mx = float(mount_cfg.get("x", 0.0))
    my = float(mount_cfg.get("y", 0.0))
    mz = float(mount_cfg.get("z", 0.0))
    myaw = float(mount_cfg.get("yaw_deg", 0.0))
    check("mount.x가 물리 허용 범위(-10~80mm) 안이다 (T12 실측 대기 — 차체 길이는 아직 미실측)",
          -10.0 <= mx <= 80.0, f"x={mx}")
    check("mount.y가 예상 범위(-10~+10mm) 안이다", -10.0 <= my <= 10.0, f"y={my}")
    check("mount.z가 예상 범위(70~85mm) 안이다", 70.0 <= mz <= 85.0, f"z={mz}")
    check("mount.yaw_deg가 예상 범위(-5~+5deg) 안이다", -5.0 <= myaw <= 5.0, f"yaw={myaw}")
    check("so101_geometry.yaml에 mount.y 및 mount.yaw_deg 가정치 주석이 있다",
          "좌우 대칭 가정치" in yraw and "브래킷 정면 정렬 가정치" in yraw)

    # config.py ARM_GEOM_* 5종과 yaml / ArmGeometry 전수 일치 검증
    from tomato_picker.config import (
        ARM_GEOM_Z0, ARM_GEOM_D0, ARM_GEOM_L1, ARM_GEOM_L2, ARM_GEOM_L3,
    )
    geom_cfg_matches = (
        abs(ARM_GEOM_Z0 - float(cfg["z0"])) < 1e-9 and
        abs(ARM_GEOM_D0 - float(cfg["d0"])) < 1e-9 and
        abs(ARM_GEOM_L1 - float(cfg["l1"])) < 1e-9 and
        abs(ARM_GEOM_L2 - float(cfg["l2"])) < 1e-9 and
        abs(ARM_GEOM_L3 - float(cfg["l3"])) < 1e-9
    )
    check("config.py ARM_GEOM_* 5종이 so101_geometry.yaml과 전수 일치한다",
          geom_cfg_matches,
          f"z0={ARM_GEOM_Z0}, d0={ARM_GEOM_D0}, l1={ARM_GEOM_L1}, l2={ARM_GEOM_L2}, l3={ARM_GEOM_L3}")

    # so101_geometry.yaml 링크 길이 합산 reach_max(391.0mm) 기하 항등성
    yaml_reach = float(cfg["d0"]) + float(cfg["l1"]) + float(cfg["l2"]) + float(cfg["l3"])
    check("so101_geometry.yaml의 최대 수평 사거리(d0+l1+l2+l3)가 391.0mm와 일치한다",
          abs(yaml_reach - 391.0) < 1e-9, f"reach={yaml_reach}mm")

    # 2026-09-26 실측(전체 외관.pdf, 기준점은 사용자 확인): 케이스는 차체 외곽이 아니라
    # 50×30 체결 패턴 두 벌을 기준으로 한다. 136 = TOPLeft 뒷줄↔TOPRight 앞줄 나사,
    # 195.5 = TOPLeft 앞줄↔TOPRight 뒷줄 나사 — **195.5는 차체 길이가 아니다**
    # (사이클 595가 차체 길이로 박았다가 사용자 확인으로 정정. 차체 길이는 아직 미실측).
    scad_src = open(os.path.join(REPO, "hardware_design", "case", "tomato_case.scad"),
                    encoding="utf-8").read()

    def scad_num(name):
        m = re.search(r"^\s*" + name + r"\s*=\s*([0-9.]+)\s*;", scad_src, re.M)
        return float(m.group(1)) if m else float("nan")

    cw, mdy, mgap = scad_num("case_w"), scad_num("mount_dy"), scad_num("mount_gap")
    check("tomato_case.scad 폭이 상판 실측 151.9mm이다", abs(cw - 151.9) < 1e-6, f"case_w={cw}")
    pitch = mgap + mdy
    check("tomato_case.scad 체결패턴: 30변=길이, 대향나사 136 → 중심간격 166, 바깥나사 195.5와 0.5mm 이내",
          abs(mdy - 30.0) < 1e-6 and abs(mgap - 136.0) < 1e-6
          and re.search(r"mount_pitch\s*=\s*mount_dy\s*\+\s*mount_gap\s*;", scad_src) is not None
          and abs((pitch + mdy) - 195.5) <= 0.5,
          f"mount_dy={mdy}, mount_gap={mgap}, pitch={pitch}, outer={pitch + mdy}")
    check("tomato_case.scad 배터리 자리가 BOTTOM 관통 터널(43×57×134, 사용자 확인)과 같다",
          re.search(r"^\s*batt\s*=\s*\[\s*134\s*,\s*43\s*,\s*57\s*\]\s*;", scad_src, re.M) is not None)
    check("tomato_case.scad SO-101 받침 구멍 간격이 BOTTOM 가운데 두 나사 67.5mm다",
          re.search(r"so101_holes\s*=\s*\[\[\s*-67\.5\s*/\s*2\s*,", scad_src) is not None)
    check("tomato_case.scad에 차체 길이 195.5를 박아 두지 않았다 (195.5는 나사 간 거리)",
          re.search(r"(chassis_d|case_d)\s*=\s*195\.5", scad_src) is None)

    # T12 측정 카드 (§77): TOPRight/TOPLeft 3D 부품 단면 및 나사 스팬 기반 차체 전장 기하 하한(283.5mm) 검증
    import struct
    def _stl_z_bounds(stl_name: str) -> tuple[float, float]:
        path = os.path.join(REPO, "3D", stl_name)
        with open(path, "rb") as f:
            f.read(80)
            n = struct.unpack("<I", f.read(4))[0]
            zs = []
            for _ in range(n):
                f.read(12)
                p1 = struct.unpack("<3f", f.read(12))
                p2 = struct.unpack("<3f", f.read(12))
                p3 = struct.unpack("<3f", f.read(12))
                f.read(2)
                zs.extend([p1[2], p2[2], p3[2]])
            return min(zs), max(zs)

    tr_z_min, tr_z_max = _stl_z_bounds("TOPRight.stl")
    tl_z_min, tl_z_max = _stl_z_bounds("TOPLeft.stl")
    rear_overhang = 41.0 - tr_z_min   # 81.0mm (Z=41 후열 나사 ↔ 후단 Z=-40)
    front_overhang = tl_z_max - 71.0  # 7.0mm (Z=71 전열 나사 ↔ 전단 Z=78)
    outer_screws = 195.5              # 사용자 확인 실측 나사 스팬 (TOPLeft 최전열 ↔ TOPRight 최후열)
    min_parts_span = rear_overhang + outer_screws + front_overhang
    check("차체 탑재 3D 부품 전장 하한(TOPRight 오버행 81mm + 나사스팬 195.5mm + TOPLeft >= 283.5mm)이 성립한다 (T12 §77)",
          min_parts_span >= 283.5 and abs(rear_overhang - 81.0) < 1e-6,
          f"min_span={min_parts_span:.1f}mm, rear_overhang={rear_overhang:.1f}mm")
    check("T12 mount.x 예상 범위(+30~+75mm)가 부품 전장 기하 하한(L>=283.5mm)과 정합한다",
          30.0 <= mx <= 75.0,
          f"mount.x={mx}mm (기하 예상 범위 30~75mm)")

    # 파지 화면좌표(grip_uv) 기본값 및 검출 영역 검증
    cs_src = open(os.path.join(ROS2, "tools", "click_server.py"), encoding="utf-8").read()
    vs_src = open(os.path.join(ROS2, "tools", "visual_servo.py"), encoding="utf-8").read()
    sw_src = open(os.path.join(ROS2, "tools", "swab_trials.py"), encoding="utf-8").read()
    check("click_server·visual_servo 파지좌표 기본값 (471, 395)이 검출 유효영역(350..550, 300..450) 안이다",
          "GRIP_UV = (471, 395)" in cs_src and "TARGET_UV = (471.0, 395.0)" in vs_src,
          "기본값 (471, 395) px")
    check("swab_trials.py bite_uv 기본값 (407, 350)이 검출 유효영역(350..550, 300..450) 안이다",
          "default=(407, 350)" in sw_src,
          "실측 기본값 (407, 350) px")

    # ARM_CART_SIGNS 5관절 부호 및 kin.JOINTS 일치 검증
    from tomato_picker.config import ARM_CART_SIGNS
    check("config.ARM_CART_SIGNS 5개 관절 부호가 전부 +1이고 kin.JOINTS와 일치한다",
          set(ARM_CART_SIGNS.keys()) == set(kin.JOINTS) and all(v == 1 for v in ARM_CART_SIGNS.values()),
          str(ARM_CART_SIGNS))

    # click_server 3D 미리보기 roll 부호 ↔ handeye_collect ROLL_SIGN 일치 검증
    import handeye_collect as hc
    check("click_server.CAM_ROLL_SIGN(-1.0)이 handeye_collect.ROLL_SIGN(-1.0)과 일치한다",
          "var CAM_ROLL_SIGN = -1.0;" in cs_src and abs(hc.ROLL_SIGN - (-1.0)) < 1e-9,
          f"CAM_ROLL_SIGN=-1.0, hc.ROLL_SIGN={hc.ROLL_SIGN}")

    # arm_calib MOUNT_Z_MM 및 FLOOR_MARGIN_MM 일치 검증
    sys.path.insert(0, os.path.join(ROS2, "tools"))
    import arm_calib
    from tomato_picker.hardware import escape as es
    check("arm_calib.MOUNT_Z_MM(76.5) 및 FLOOR_MARGIN_MM(10.0)이 escape 및 yaml mount.z와 일치한다",
          abs(arm_calib.MOUNT_Z_MM - float(mount_cfg["z"])) < 1e-9 and
          abs(arm_calib.FLOOR_MARGIN_MM - es.FLOOR_MARGIN_MM) < 1e-9 and
          abs(arm_calib.MOUNT_Z_MM - es.MOUNT_Z_MM) < 1e-9,
          f"mount.z={arm_calib.MOUNT_Z_MM} margin={arm_calib.FLOOR_MARGIN_MM}")

    # 6대 하드웨어 측정 도구의 MOUNT_Z_MM(76.5) 및 FLOOR_MARGIN_MM(10.0) 하드코딩 일치 검증
    hw_tools = ["handeye_collect.py", "joint_axis_check.py", "repeat_check.py",
                "target_sweep.py", "visual_servo.py", "wall_find.py"]
    hw_tools_ok = all(
        "MOUNT_Z_MM = 76.5" in open(os.path.join(ROS2, "tools", t), encoding="utf-8").read() and
        "FLOOR_MARGIN_MM = 10.0" in open(os.path.join(ROS2, "tools", t), encoding="utf-8").read()
        for t in hw_tools
    )
    check("6대 하드웨어 측정 도구(handeye_collect 등)의 MOUNT_Z_MM(76.5) 및 FLOOR_MARGIN(10.0)이 일치한다",
          hw_tools_ok, f"6개 도구 전수 점검={len(hw_tools)}")

    # 비상 탈출 바닥 z(-66.5mm)와 무대 작업 z 하한(+15.0mm)의 기하학적 계층 분리 검증
    from tomato_picker.config import ARM_CART_Z_MIN
    check("비상탈출 바닥(-66.5mm)이 무대 작업 하한 ARM_CART_Z_MIN(+15.0mm)보다 엄격히 아래다 (지면 vs 무대 분리)",
          es.floor_z() == -66.5 and es.floor_z() < 0.0 < ARM_CART_Z_MIN,
          f"floor_z={es.floor_z()}mm < 0 < Z_MIN={ARM_CART_Z_MIN}mm")

    # tomato_robot.urdf.xacro xacro:arg 기본값 ↔ so101_geometry.yaml 일치 검증
    # launch 없이 xacro를 독립 실행할 때도 동일한 기하 및 관절 한계가 유지되도록 보증
    xacro_path = os.path.join(SRC, "tomato_description", "urdf", "tomato_robot.urdf.xacro")
    xtree = ET.parse(xacro_path)
    xns = {"xacro": "http://www.ros.org/wiki/xacro"}
    xargs = {elem.get("name"): float(elem.get("default"))
             for elem in xtree.getroot().findall("xacro:arg", xns)
             if elem.get("default") is not None}

    base_cfg = _geometry_yaml().get("base", {})
    lim_cfg = cfg.get("limits_deg", {})

    expected_xargs = {
        "z0_mm": float(cfg["z0"]), "d0_mm": float(cfg["d0"]),
        "l1_mm": float(cfg["l1"]), "l2_mm": float(cfg["l2"]), "l3_mm": float(cfg["l3"]),
        "mount_x_mm": mx, "mount_y_mm": my, "mount_z_mm": mz, "mount_yaw_deg": myaw,
        "base_length_mm": float(base_cfg.get("length", 0.0)),
        "base_width_mm": float(base_cfg.get("width", 0.0)),
        "base_height_mm": float(base_cfg.get("height", 0.0)),
        "wheel_radius_mm": float(base_cfg.get("wheel_radius", 0.0)),
        "pan_min_deg": float(lim_cfg["shoulder_pan"][0]), "pan_max_deg": float(lim_cfg["shoulder_pan"][1]),
        "lift_min_deg": float(lim_cfg["shoulder_lift"][0]), "lift_max_deg": float(lim_cfg["shoulder_lift"][1]),
        "elbow_min_deg": float(lim_cfg["elbow_flex"][0]), "elbow_max_deg": float(lim_cfg["elbow_flex"][1]),
        "wflex_min_deg": float(lim_cfg["wrist_flex"][0]), "wflex_max_deg": float(lim_cfg["wrist_flex"][1]),
        "wroll_min_deg": float(lim_cfg["wrist_roll"][0]), "wroll_max_deg": float(lim_cfg["wrist_roll"][1]),
        "grip_min_deg": float(lim_cfg["gripper"][0]), "grip_max_deg": float(lim_cfg["gripper"][1]),
    }

    xarg_diffs = [f"{k}: xacro={xargs.get(k)} vs yaml={v}"
                  for k, v in expected_xargs.items()
                  if k not in xargs or abs(xargs[k] - v) > 1e-6]
    check("tomato_robot.urdf.xacro 기본 인자 25종이 so101_geometry.yaml과 일치한다",
          not xarg_diffs, ", ".join(xarg_diffs) if xarg_diffs else "25종 전수 일치")

    all_joints = kin.JOINTS + ("gripper",)
    lim_keys_ok = all(j in lim_cfg and len(lim_cfg[j]) == 2 and float(lim_cfg[j][0]) < float(lim_cfg[j][1])
                      for j in all_joints)
    check("so101_geometry.yaml의 limits_deg가 6개 전 관절의 유효 범위(min < max)를 정의한다",
          lim_keys_ok, f"관절 {len(lim_cfg)}개 정의됨")


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

    inf_cmd = bc.plan(float("inf"), 0, 0)
    check("inf/-inf 지령은 거절하고 세운다 (math.isfinite 가드)",
          inf_cmd.rejected and inf_cmd.payload == "S", inf_cmd.reason[:50])

    w_tiny = bc.plan(0, 0, math.radians(0.02))
    check("문턱 미만 각속도(0.02°/s < EPS_MDEGS 500mdeg/s)는 S(정지)로 귀착된다",
          w_tiny.payload == "S" and w_tiny.duty is None, str(w_tiny.physical))

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
    check("DutyCalib wmax_degs 및 duty_angular가 정지마찰 ks_w를 반영한다",
          calib.wmax_degs == (255 - 90) / 1.1 and calib.duty_angular(90.0) == 189 and calib.duty_angular(-90.0) == -189)

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
    leg = bc.Caps.legacy()
    check("Caps.legacy()가 uno-moebius 및 units=False/calib=False 계약을 유지한다",
          leg.proto == 1 and leg.board == "uno-moebius" and not leg.units and not leg.calib)

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

    with open(os.path.join(SRC, "tomato_bringup", "config", "stage1.yaml"),
              encoding="utf-8") as f:
        stage1_cfg = yaml.safe_load(f)
    base_cfg = stage1_cfg.get("tomato_base", {}).get("ros__parameters", {})

    check("stage1.yaml의 tomato_base duty 환산 기본값이 DutyCalib과 일치한다",
          base_cfg.get("duty_ks") == calib.ks
          and base_cfg.get("duty_ks_w") == calib.ks_w
          and abs(base_cfg.get("duty_kv", 0) - calib.kv) < 1e-6
          and abs(base_cfg.get("duty_kv_w", 0) - calib.kv_w) < 1e-6
          and base_cfg.get("duty_max") == calib.max_duty
          and base_cfg.get("duty_measured") is False,
          f"ks={base_cfg.get('duty_ks')} kv={base_cfg.get('duty_kv')} measured={base_cfg.get('duty_measured')}")

    check("stage1.yaml의 tomato_base 데드맨 타임아웃(0.3s)이 보드계약 §9 4층 규약과 일치한다",
          base_cfg.get("cmd_timeout") == 0.3,
          f"cmd_timeout={base_cfg.get('cmd_timeout')}")

    check("stage1.yaml의 tomato_base 축부호(sign_vx=1, sign_vy=1, sign_w=1)가 AxisSigns 규약과 일치한다",
          base_cfg.get("sign_vx") == 1 and base_cfg.get("sign_vy") == 1 and base_cfg.get("sign_w") == 1,
          f"vx={base_cfg.get('sign_vx')} vy={base_cfg.get('sign_vy')} w={base_cfg.get('sign_w')}")

    # ⑤ [감사] 보드 계약 v2 5층 데드맨 안전 시한 크로스 레이어 실물 검증 (docs/보드-계약.md §9)
    # 1층 & 2층: 펌웨어(mecanum_stable.ino) 하드 1000ms / 소프트 300ms
    ino_path = os.path.join(REPO, "firmware", "mecanum_stable", "mecanum_stable.ino")
    ino_text = open(ino_path, encoding="utf-8").read()
    m_hard = re.search(r"HARD_TIMEOUT\s*=\s*(\d+)", ino_text)
    m_cmd = re.search(r"CMD_TIMEOUT\s*=\s*(\d+)", ino_text)
    hard_ms = int(m_hard.group(1)) if m_hard else 0
    cmd_ms = int(m_cmd.group(1)) if m_cmd else 0
    check("보드 1층 하드 데드맨(HARD_TIMEOUT=1000ms)이 펌웨어에 선언되어 있다",
          hard_ms == 1000, f"hard={hard_ms}ms")
    check("보드 2층 소프트 데드맨(CMD_TIMEOUT=300ms)이 펌웨어에 선언되어 있다",
          cmd_ms == 300, f"cmd={cmd_ms}ms")

    # 3층: 젯슨 MotorLink 재전송 스레드 (주기 20ms, stale 500ms)
    from tomato_picker.hardware.motor_link import MotorLink
    check("젯슨 3층 재전송 주기(SEND_INTERVAL_SEC=0.02s) 및 STALE_SEC(0.5s)가 선언되어 있다",
          MotorLink.SEND_INTERVAL_SEC == 0.02 and MotorLink.STALE_SEC == 0.5,
          f"send={MotorLink.SEND_INTERVAL_SEC}s stale={MotorLink.STALE_SEC}s")

    # 4층: ROS 2 cmd_vel_node.py의 데드맨 감시 주기(0.05s) 및 기본 cmd_timeout(0.3s)
    cmd_node_path = os.path.join(SRC, "tomato_bridge", "tomato_bridge", "cmd_vel_node.py")
    cmd_node_text = open(cmd_node_path, encoding="utf-8").read()
    check("ROS 4층 cmd_vel_node가 cmd_timeout 0.3s 기본값 및 0.05s 감시 타이머를 선언한다",
          'self.declare_parameter("cmd_timeout", 0.3)' in cmd_node_text
          and "self.create_timer(0.05, self._watch)" in cmd_node_text,
          "cmd_vel_node 데드맨 감시")

    # 5층: 하드웨어 비상정지 래치 시 거절 및 S 지령
    cmd_estop_real = bc.plan(0.35, 0.0, 0.0, estop=True)
    check("5층 비상정지(estop=True) 시 즉시 S 페이로드 반환 및 사유를 명시하여 거절한다",
          cmd_estop_real.rejected and cmd_estop_real.payload == "S" and "비상정지" in cmd_estop_real.reason,
          f"payload={cmd_estop_real.payload} reason={cmd_estop_real.reason}")

    # ⑥ [보드계약 v2] 능력선언(cap) 확장 필드 및 하트비트(hb) 상태 파싱 검증 (docs/보드-계약.md §6, §7)
    v2_full = bc.Caps.parse("cap proto=2 fw=3.0.0 board=stm32f411 id=A3F2C918 drive=mecanum wheels=4 "
                            "units=1 closed_loop=1 enc=4 calib=1 "
                            "vmax=800 vymax=600 wmax=180000 "
                            "vin=1 amp=1 estop_hw=1 pwm_hz=20000")
    check("보드계약 §6 cap 확장 필드(enc, vin, amp, pwm_hz)를 정확히 파싱한다",
          v2_full.enc == 4 and v2_full.vin and v2_full.amp and v2_full.pwm_hz == 20000,
          f"enc={v2_full.enc} vin={v2_full.vin} amp={v2_full.amp} pwm_hz={v2_full.pwm_hz}")

    hb_full = bc.Heartbeat.parse("hb 124500 rx=102 bad=0 i2c=0 wdt=0 st=0x29 "
                                 "tgt=300,0,0 act=295,-2,5 vin=12400 amp=850")
    check("보드계약 §7 hb 기본 필드 및 전원(vin, amp)을 정확히 파싱한다",
          hb_full.ms == 124500 and hb_full.rx == 102 and hb_full.vin_mv == 12400 and hb_full.amp_ma == 850,
          f"ms={hb_full.ms} vin={hb_full.vin_mv} amp={hb_full.amp_ma}")

    check("보드계약 §7 hb tgt/act 3축 속도 벡터를 정확히 파싱한다",
          hb_full.tgt == (300, 0, 0) and hb_full.act == (295, -2, 5),
          f"tgt={hb_full.tgt} act={hb_full.act}")

    # st=0x29 = 0b00101001: 비트 0(estop), 비트 3(calib_valid), 비트 5(output_saturated)
    check("보드계약 §7 st 상태 비트 플래그(estop, calib_valid, output_saturated)를 정확히 해석한다",
          hb_full.estop_latched and hb_full.calib_valid and hb_full.output_saturated
          and not hb_full.soft_deadman and not hb_full.hard_deadman and not hb_full.driver_fault,
          f"st=0x{hb_full.st:02X} estop={hb_full.estop_latched} calib={hb_full.calib_valid} sat={hb_full.output_saturated}")

    hb_soft = bc.Heartbeat.parse("hb 500 rx=1 bad=0 i2c=0 wdt=0 st=0x02")
    check("보드계약 §7 st 소프트 데드맨 감속 비트(비트 1)를 정확히 감지한다",
          hb_soft.soft_deadman and not hb_soft.hard_deadman,
          f"st=0x{hb_soft.st:02X} soft={hb_soft.soft_deadman}")

    # ⑦ [보드계약 v2] MobileBase 프로토콜 및 Telemetry 스냅샷 검증 (docs/보드-계약.md §11.1)
    telem = bc.Telemetry.from_heartbeat(hb_full)
    check("보드계약 §11.1 Telemetry가 Heartbeat로부터 무손실 복제 생성된다",
          telem.ms == hb_full.ms and telem.st == hb_full.st and telem.tgt == hb_full.tgt and telem.act == hb_full.act and telem.vin_mv == hb_full.vin_mv,
          f"ms={telem.ms} tgt={telem.tgt} act={telem.act}")

    class _ValidBase:
        def set_velocity(self, vx_mms: int, vy_mms: int, w_mdegs: int) -> None:
            pass
        def stop(self) -> None:
            pass
        def estop(self, on: bool) -> None:
            pass
        def caps(self) -> bc.Caps:
            return v2_full
        def telemetry(self) -> bc.Telemetry:
            return telem

    check("보드계약 §11.1 MobileBase 프로토콜을 만족하는 구현체가 isinstance 검사를 통과한다",
          isinstance(_ValidBase(), bc.MobileBase),
          "MobileBase 프로토콜 5대 메서드(set_velocity, stop, estop, caps, telemetry) 만족")

    class _IncompleteBase:
        def set_velocity(self, vx_mms: int, vy_mms: int, w_mdegs: int) -> None:
            pass

    check("보드계약 §11.1 누락된 메서드가 있는 클래스는 MobileBase 프로토콜을 통과하지 못한다",
          not isinstance(_IncompleteBase(), bc.MobileBase),
          "불완전 구현체 거절")

    check("board_contract.__all__에 MobileBase 및 Telemetry가 포함되어 있다",
          "MobileBase" in bc.__all__ and "Telemetry" in bc.__all__,
          f"__all__={bc.__all__}")

    from tomato_picker.hardware.base import MobileBase as LegacyBase
    check("레거시 hardware.base.MobileBase(ABC)와 새 tomato_bridge.board_contract.MobileBase(Protocol)의 역할이 분리되어 있다",
          LegacyBase is not bc.MobileBase and hasattr(LegacyBase, "drive_to") and not hasattr(bc.MobileBase, "drive_to"),
          "레거시 상위 스킬 ABC vs 보드계약 v2 물리 인터페이스 Protocol")

    # ⑧ [보드계약 v2] LegacyDutyControl 격리 및 베이스 구현체(MockBase, SimBase) 검증 (docs/보드-계약.md §11.1, §11.2)
    class _DutyImpl:
        def set_duty(self, dx: int, dy: int, dw: int) -> None:
            pass

    check("보드계약 §11.1 LegacyDutyControl 프로토콜 격리 및 isinstance 검사 통과",
          isinstance(_DutyImpl(), bc.LegacyDutyControl) and not isinstance(_ValidBase(), bc.LegacyDutyControl),
          "선택적 레거시 duty 제어 인터페이스 분리")

    mock = bc.MockBase()
    check("보드계약 §11.2 MockBase가 MobileBase 프로토콜을 충족한다",
          isinstance(mock, bc.MobileBase) and mock.caps().board == "uno-moebius",
          f"board={mock.caps().board}")

    sim = bc.SimBase(ks_mms=50, ks_w_mdegs=15000)
    check("보드계약 §11.2 SimBase가 MobileBase 프로토콜을 충족한다",
          isinstance(sim, bc.MobileBase) and sim.caps().board == "sim",
          f"board={sim.caps().board}")

    # SimBase 물리 모델: 정지마찰 문턱 미만이면 act 0 유지
    sim.set_velocity(30, 0, 0)
    sim.step(0.05)
    check("보드계약 §11.2 SimBase가 정지마찰 문턱(ks=50) 미만 지령 시 0으로 수렴한다",
          sim.telemetry().act[0] == 0,
          f"act={sim.telemetry().act}")

    # SimBase 정상 지령 시 1차 지연으로 추종 및 비상정지 래치 시 0 정지
    sim.set_velocity(300, 0, 0)
    sim.step(0.05)
    sim.step(0.05)
    act_vx = sim.telemetry().act[0]
    sim.estop(True)
    sim.step(0.05)
    check("보드계약 §11.2 SimBase가 문턱 초과 시 추종하고 estop 래치 시 즉시 0으로 차단된다",
          act_vx > 50 and sim.telemetry().act == (0, 0, 0) and sim.telemetry().st & 0x01 != 0,
          f"act_before_estop={act_vx} act_after={sim.telemetry().act} st=0x{sim.telemetry().st:02X}")

    # ⑨ [보드계약 v2] SimBase 물리 한계(포화·정지마찰·1차지연) 계측 감사 (docs/보드-계약.md §6, §7, §11.2)
    # SimBase가 caps.vmax_mms(800) 초과 지령 시 속도 상한으로 클램프하고 output_saturated(0x20) 반영
    sim_sat = bc.SimBase(ks_mms=50, ks_w_mdegs=15000)
    sim_sat.set_velocity(1500, 0, 0)
    for _ in range(20):
        sim_sat.step(0.05)
    check("보드계약 §11.2 SimBase가 caps.vmax_mms(800) 초과 지령 시 속도 상한으로 클램프한다",
          sim_sat.telemetry().act[0] == 800,
          f"act={sim_sat.telemetry().act} vmax={sim_sat.caps().vmax_mms}")

    check("보드계약 §11.2 SimBase가 상한 도달 시 telemetry st에 output_saturated(0x20) 플래그를 세운다",
          sim_sat.telemetry().output_saturated and (sim_sat.telemetry().st & 0x20 != 0),
          f"st=0x{sim_sat.telemetry().st:02X}")

    sim_sat.set_velocity(200, 0, 0)
    for _ in range(20):
        sim_sat.step(0.05)
    check("보드계약 §11.2 SimBase가 정상 속도로 복귀 시 output_saturated 플래그를 해제한다",
          not sim_sat.telemetry().output_saturated and (sim_sat.telemetry().st & 0x20 == 0) and sim_sat.telemetry().act[0] == 200,
          f"act={sim_sat.telemetry().act} st=0x{sim_sat.telemetry().st:02X}")

    # 음수 지령(-vmax 초과)에 대해서도 대칭적으로 속도 상한 클램프 및 포화 플래그 감지
    sim_sat.set_velocity(-1500, 0, 0)
    for _ in range(20):
        sim_sat.step(0.05)
    check("보드계약 §11.2 SimBase step이 음수 지령(-vmax 초과)에 대해서도 대칭적으로 속도 상한 및 포화를 감지한다",
          sim_sat.telemetry().act[0] == -800 and sim_sat.telemetry().output_saturated,
          f"act={sim_sat.telemetry().act} st=0x{sim_sat.telemetry().st:02X}")

    # 물리 기하학적 한계 대조 (DC 12V 330rpm 감속모터, r=40mm 무부하 상한 1382 mm/s)
    calib = bc.DutyCalib()
    check("보드계약 §11.2 SimBase와 DutyCalib의 물리 속도 상한 및 정지마찰 문턱이 물리 한계(vmax < 1382mm/s) 내에 안착한다",
          sim_sat.caps().vmax_mms < 1382 and calib.vmax_mms < 1382 and sim_sat._ks_mms > 0,
          f"sim_vmax={sim_sat.caps().vmax_mms} calib_vmax={calib.vmax_mms:.1f}")

    # ⑩ [보드계약 v2] UnoAdapterBase (Uno 개루프 + DutyCalib 어댑터) 검증 (docs/보드-계약.md §11.2, §13 단계 1)
    class _DummyMotorLink:
        def __init__(self) -> None:
            self.last_cmd = (0, 0, 0)
            self.stopped = False
        def set_velocity(self, vx: int = 0, vy: int = 0, w: int = 0) -> None:
            self.last_cmd = (vx, vy, w)
            self.stopped = False
        def stop(self) -> None:
            self.last_cmd = (0, 0, 0)
            self.stopped = True
        def stats(self) -> dict:
            return {"fw_ms": 15000, "fw_rx": 250, "fw_bad": 0, "i2c_err": 0, "wdt_near": 0}

    dummy_link = _DummyMotorLink()
    uno_base = bc.UnoAdapterBase(motor_link=dummy_link, calib=calib)
    check("보드계약 §11.2 UnoAdapterBase가 MobileBase 및 LegacyDutyControl 프로토콜을 모두 만족한다",
          isinstance(uno_base, bc.MobileBase) and isinstance(uno_base, bc.LegacyDutyControl),
          "MobileBase + LegacyDutyControl 다중 프로토콜 충족")

    uno_base.set_velocity(300, 0, 0)
    # 300 mm/s -> duty = Ks(90) + Kv(0.35)*300 = 195
    check("보드계약 §11.2 UnoAdapterBase가 물리 속도 지령(300mm/s)을 DutyCalib(duty 195)로 올바르게 변환하여 링크에 전달한다",
          dummy_link.last_cmd == (195, 0, 0) and uno_base._last_duty == (195, 0, 0),
          f"cmd={dummy_link.last_cmd}")

    uno_base.set_duty(120, -50, 80)
    check("보드계약 §11.2 UnoAdapterBase가 LegacyDutyControl.set_duty 직통 지령을 지원한다",
          dummy_link.last_cmd == (120, -50, 80),
          f"cmd={dummy_link.last_cmd}")

    uno_base.stop()
    check("보드계약 §11.2 UnoAdapterBase stop() 호출 시 링크 stop()이 수행되고 목표/duty가 0으로 정지된다",
          dummy_link.stopped and uno_base._last_duty == (0, 0, 0) and uno_base._tgt == (0, 0, 0),
          f"stopped={dummy_link.stopped} duty={uno_base._last_duty}")

    uno_base.estop(True)
    uno_base.set_velocity(200, 0, 0)
    telem_uno = uno_base.telemetry()
    check("보드계약 §11.2 UnoAdapterBase 비상정지 래치 시 지령이 차단되고 telemetry st에 estop 플래그가 반영된다",
          dummy_link.last_cmd == (0, 0, 0) and telem_uno.estop_latched and telem_uno.ms == 15000,
          f"last_cmd={dummy_link.last_cmd} st=0x{telem_uno.st:02X} ms={telem_uno.ms}")

    # ⑪ [감사] 보드 계약 v2 §12 계약 테스트 스위트 및 0속도 정지 안전 검증 (docs/보드-계약.md §11.2, §12)
    # 1. 0 속도 지령 안전 정지 (UnoAdapterBase): 300mm/s 주행 중 set_velocity(0,0,0) 수신 시 즉시 정지 및 duty 0
    uno_base.estop(False)
    uno_base.set_velocity(300, 0, 0)
    check("보드계약 §12 UnoAdapterBase가 300mm/s 주행 후 set_velocity(0,0,0) 수신 시 즉시 링크 stop을 호출하고 duty를 0으로 리셋한다",
          dummy_link.last_cmd == (195, 0, 0) and not dummy_link.stopped and (
              uno_base.set_velocity(0, 0, 0) is None
              and dummy_link.stopped
              and uno_base._last_duty == (0, 0, 0)
              and uno_base._tgt == (0, 0, 0)
          ),
          f"stopped={dummy_link.stopped} duty={uno_base._last_duty}")

    # 2. LegacyDutyControl set_duty(0,0,0) 수신 시 즉시 정지
    uno_base.set_duty(100, 50, -30)
    check("보드계약 §12 UnoAdapterBase가 set_duty(0,0,0) 수신 시 즉시 링크 stop을 호출하고 duty를 0으로 리셋한다",
          dummy_link.last_cmd == (100, 50, -30) and (
              uno_base.set_duty(0, 0, 0) is None
              and dummy_link.stopped
              and uno_base._last_duty == (0, 0, 0)
          ),
          f"stopped={dummy_link.stopped} duty={uno_base._last_duty}")

    # 3. MockBase 계약 테스트: estop 차단, estop 해제 복구, 0 지령 정지, telemetry 일관성
    mock_base = bc.MockBase()
    mock_base.set_velocity(100, 0, 0)
    t_m1 = mock_base.telemetry()
    mock_base.estop(True)
    mock_base.set_velocity(150, 0, 0)
    t_m2 = mock_base.telemetry()
    mock_base.estop(False)
    mock_base.set_velocity(150, 0, 0)
    t_m3 = mock_base.telemetry()
    mock_base.set_velocity(0, 0, 0)
    t_m4 = mock_base.telemetry()
    check("보드계약 §12 MockBase가 계약 테스트(정상 지령·estop 차단·해제 복구·0정지)를 충족한다",
          t_m1.tgt == (100, 0, 0) and not t_m1.estop_latched
          and t_m2.tgt == (0, 0, 0) and t_m2.estop_latched
          and t_m3.tgt == (150, 0, 0) and not t_m3.estop_latched
          and t_m4.tgt == (0, 0, 0),
          f"m1={t_m1.tgt} m2={t_m2.tgt} m3={t_m3.tgt} m4={t_m4.tgt}")

    # 4. SimBase 계약 테스트: estop 차단, estop 해제 복구, 0 지령 정지
    sim_base = bc.SimBase()
    sim_base.set_velocity(200, 0, 0)
    t_s1 = sim_base.telemetry()
    sim_base.estop(True)
    sim_base.set_velocity(200, 0, 0)
    t_s2 = sim_base.telemetry()
    sim_base.estop(False)
    sim_base.set_velocity(200, 0, 0)
    t_s3 = sim_base.telemetry()
    sim_base.set_velocity(0, 0, 0)
    t_s4 = sim_base.telemetry()
    check("보드계약 §12 SimBase가 계약 테스트(정상 지령·estop 차단·해제 복구·0정지)를 충족한다",
          t_s1.tgt == (200, 0, 0) and not t_s1.estop_latched
          and t_s2.tgt == (0, 0, 0) and t_s2.estop_latched
          and t_s3.tgt == (200, 0, 0) and not t_s3.estop_latched
          and t_s4.tgt == (0, 0, 0),
          f"s1={t_s1.tgt} s2={t_s2.tgt} s3={t_s3.tgt} s4={t_s4.tgt}")

    # 4b. SimBase 주행 중 set_velocity(0,0,0) 수신 시 tgt=(0,0,0) 즉시 반영 및 act 속도 0으로 감속 수렴
    sim_base.set_velocity(200, 0, 0)
    for _ in range(10):
        sim_base.step(0.05)
    act_running = sim_base.telemetry().act[0]
    sim_base.set_velocity(0, 0, 0)
    for _ in range(25):
        sim_base.step(0.05)
    check("보드계약 §12 안전: SimBase가 0 속도 지령(0,0,0) 수신 시 tgt=(0,0,0) 반영 및 act 속도가 0으로 감속 수렴한다",
          act_running > 50 and sim_base.telemetry().tgt == (0, 0, 0) and sim_base.telemetry().act == (0, 0, 0),
          f"before={act_running} tgt={sim_base.telemetry().tgt} act={sim_base.telemetry().act}")

    # 5. UnoAdapterBase 계약 테스트: estop 해제 후 주행 복구
    uno_base.estop(True)
    uno_base.set_velocity(200, 0, 0)
    t_u1 = uno_base.telemetry()
    cmd1 = dummy_link.last_cmd
    uno_base.estop(False)
    uno_base.set_velocity(200, 0, 0)
    t_u2 = uno_base.telemetry()
    check("보드계약 §12 UnoAdapterBase가 estop 해제 시 지령 수신을 정상 복구한다",
          t_u1.estop_latched and cmd1 == (0, 0, 0)
          and not t_u2.estop_latched and dummy_link.last_cmd == (160, 0, 0) and t_u2.tgt == (200, 0, 0),
          f"u1_cmd={cmd1} u2_tgt={t_u2.tgt}")

    # 6. 보드계약 §12 단위 계약: SimBase 폐루프 구현체 3축(vx=350, vy=-200, w=90000 mdeg/s) 지령 정상 상태 도달
    sim_unit = bc.SimBase()
    sim_unit.set_velocity(350, -200, 90000)
    for _ in range(40):
        sim_unit.step(0.05)
    t_unit = sim_unit.telemetry()
    check("보드계약 §12 SimBase가 물리 지령(vx=350, vy=-200, w=90000)을 정상 상태(±1 이내)로 추종한다",
          abs(t_unit.act[0] - 350) <= 1 and abs(t_unit.act[1] - (-200)) <= 1 and abs(t_unit.act[2] - 90000) <= 1,
          f"act={t_unit.act}")

    # 7. 보드계약 §12 안전 계약: 지령 단절 시 300ms 후 소프트 데드맨(감속) 진입 및 1000ms 후 하드 데드맨(완전 정지)
    sim_deadman = bc.SimBase(deadman_enabled=True)
    sim_deadman.set_velocity(400, 0, 0)
    sim_deadman.step(0.05)  # 50ms 주행
    t_dm0 = sim_deadman.telemetry()
    # 추가 지령 없이 300ms 초과(총 350ms) 시뮬레이션 진행
    for _ in range(6):
        sim_deadman.step(0.05)  # +300ms -> 총 350ms 경과
    t_dm_soft = sim_deadman.telemetry()
    # 총 1050ms 경과까지 추가 진행
    for _ in range(14):
        sim_deadman.step(0.05)  # +700ms -> 총 1050ms 경과
    t_dm_hard = sim_deadman.telemetry()
    check("보드계약 §12 SimBase가 지령 단절 300ms 후 소프트 데드맨 감속(st bit1) 및 1000ms 후 하드 데드맨 완전 정지(st bit2)를 수행한다",
          t_dm0.act[0] > 0 and (t_dm_soft.st & 0x02 != 0) and t_dm_hard.act == (0, 0, 0) and (t_dm_hard.st & 0x04 != 0),
          f"dm0_act={t_dm0.act} dm_soft_st=0x{t_dm_soft.st:02X} dm_hard_act={t_dm_hard.act} dm_hard_st=0x{t_dm_hard.st:02X}")

    # 8. 보드계약 §12 안전 계약: S(stop) 호출 시 슬루를 무시하고 즉시 0
    sim_stop = bc.SimBase()
    sim_stop.set_velocity(500, 300, -60000)
    for _ in range(10):
        sim_stop.step(0.05)
    sim_stop.stop()
    t_stop = sim_stop.telemetry()
    check("보드계약 §12 SimBase stop() 호출 시 슬루를 무시하고 act/tgt가 즉시 0으로 소멸한다",
          t_stop.act == (0, 0, 0) and t_stop.tgt == (0, 0, 0),
          f"act={t_stop.act} tgt={t_stop.tgt}")

    # 9. 보드계약 §12 물리 모델 계약: 정지마찰 문턱(ks_mms=50, ks_w=15000) 미만 지령은 물리적으로 0 수렴
    sim_fric = bc.SimBase(ks_mms=50, ks_w_mdegs=15000)
    sim_fric.set_velocity(40, -30, 10000)
    for _ in range(10):
        sim_fric.step(0.05)
    t_fric = sim_fric.telemetry()
    check("보드계약 §12 SimBase가 정지마찰 문턱 미만 지령에 대해 물리적으로 0(안 움직임)을 유지한다",
          t_fric.act == (0, 0, 0),
          f"fric_act={t_fric.act}")

    # 10. 보드계약 §12 교체 가능성 계약: MockBase, SimBase, UnoAdapterBase, Stm32Base가 동일 stop/estop 호출 인터페이스 및 0 수렴 일관성을 보장한다
    bases: list[bc.MobileBase] = [
        bc.MockBase(),
        bc.SimBase(),
        bc.UnoAdapterBase(motor_link=dummy_link, calib=calib),
        bc.Stm32Base(motor_link=dummy_link),
    ]
    stop_results = []
    for b in bases:
        b.set_velocity(200, 0, 0)
        b.stop()
        stop_results.append(b.telemetry().tgt == (0, 0, 0))
    check("보드계약 §12 다중 MobileBase 구현체(MockBase, SimBase, UnoAdapterBase, Stm32Base)가 일관된 stop/0수렴 계약을 보장한다",
          all(stop_results),
          f"stop_results={stop_results}")

    # ⑫ [보드계약 v2] cmd_vel_node MobileBase 인터페이스 및 UnoAdapterBase 주입 연동 검증 (docs/보드-계약.md §11.1, §13 단계 1)
    # AST 및 정적 검사: cmd_vel_node가 MobileBase 및 UnoAdapterBase를 import하고 의존 주입 지원
    import ast
    parsed_cmd_node = ast.parse(cmd_node_text)
    
    # 1. MobileBase 및 UnoAdapterBase import 확인
    imported_names = set()
    for node_item in ast.walk(parsed_cmd_node):
        if isinstance(node_item, ast.ImportFrom):
            for alias in node_item.names:
                imported_names.add(alias.name)
    check("보드계약 §11.1 cmd_vel_node가 MobileBase 및 UnoAdapterBase를 의존 선언한다",
          "MobileBase" in imported_names and "UnoAdapterBase" in imported_names,
          f"imported={imported_names}")

    # 2. CmdVelNode.__init__이 MobileBase 주입 인자를 수용하고 기본 UnoAdapterBase 생성 지원
    init_def = None
    on_cmd_def = None
    halt_def = None
    destroy_def = None
    for item in parsed_cmd_node.body:
        if isinstance(item, ast.ClassDef) and item.name == "CmdVelNode":
            for m in item.body:
                if isinstance(m, ast.FunctionDef):
                    if m.name == "__init__":
                        init_def = m
                    elif m.name == "_on_cmd":
                        on_cmd_def = m
                    elif m.name == "_halt":
                        halt_def = m
                    elif m.name == "destroy_node":
                        destroy_def = m

    init_args = [a.arg for a in init_def.args.args] if init_def else []
    check("보드계약 §11.1 CmdVelNode.__init__이 MobileBase 주입 파라미터(base)를 지원한다",
          "base" in init_args,
          f"init_args={init_args}")

    # 3. _on_cmd 콜백이 self._base.set_velocity()를 호출하는지 검증
    on_cmd_calls = []
    if on_cmd_def:
        for n in ast.walk(on_cmd_def):
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
                if isinstance(n.func.value, ast.Attribute) and n.func.value.attr == "_base":
                    on_cmd_calls.append(n.func.attr)
    check("보드계약 §11.1 cmd_vel_node _on_cmd 콜백이 MobileBase.set_velocity 인터페이스를 호출한다",
          "set_velocity" in on_cmd_calls,
          f"on_cmd_base_calls={on_cmd_calls}")

    # 4. _halt 메서드가 self._base.stop()을 호출하여 즉시 정지하는지 검증
    halt_calls = []
    if halt_def:
        for n in ast.walk(halt_def):
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
                if isinstance(n.func.value, ast.Attribute) and n.func.value.attr == "_base":
                    halt_calls.append(n.func.attr)
    check("보드계약 §11.1 cmd_vel_node _halt가 MobileBase.stop()을 호출하여 하위 베이스를 정지시킨다",
          "stop" in halt_calls,
          f"halt_base_calls={halt_calls}")

    # 5. CmdVelNode 기본 생성 시 UnoAdapterBase 인스턴스화 및 destroy_node 리소스 해제 보장
    check("보드계약 §13 단계 1 cmd_vel_node가 기본 UnoAdapterBase 주입 및 destroy_node 안전 정지/자원 해제를 구현한다",
          "UnoAdapterBase(motor_link=" in cmd_node_text
          and "self._base.close()" in cmd_node_text
          and "self._base.stop()" in cmd_node_text,
          "UnoAdapterBase 기본 주입 및 안전 종료 검증")

    # 6. CmdVelNode base_type 파라미터(uno/sim/stm32/mock) 선언 및 4대 베이스 팩토리 분기 검증 (보드계약 §11, §13)
    check("보드계약 §11 & §13 cmd_vel_node가 base_type 파라미터를 선언하고 4대 베이스(uno, sim, stm32, mock) 팩토리를 지원한다",
          'self.declare_parameter("base_type", "uno")' in cmd_node_text
          and 'base_type == "mock"' in cmd_node_text and "MockBase(" in cmd_node_text
          and 'base_type == "sim"' in cmd_node_text and "SimBase(" in cmd_node_text
          and 'base_type == "stm32"' in cmd_node_text and "Stm32Base(" in cmd_node_text
          and 'base_type == "uno"' in cmd_node_text and "UnoAdapterBase(" in cmd_node_text,
          "cmd_vel_node 4대 베이스 팩토리 분기 검증")

    # 7. stage1.yaml 및 stage1.launch.py에 base_type 파라미터/인자 선언 및 기본값 uno 일치 검증
    with open(os.path.join(SRC, "tomato_bringup", "config", "stage1.yaml"), encoding="utf-8") as f_yaml:
        s1_yaml = yaml.safe_load(f_yaml)
    base_yaml_cfg = s1_yaml.get("tomato_base", {}).get("ros__parameters", {})
    with open(os.path.join(SRC, "tomato_bringup", "launch", "stage1.launch.py"), encoding="utf-8") as f_launch:
        s1_launch_text = f_launch.read()
    check("보드계약 §11 & §13 stage1.yaml과 stage1.launch.py가 base_type 기본값('uno')을 선언하고 cmd_vel_node에 전달한다",
          base_yaml_cfg.get("base_type") == "uno"
          and 'DeclareLaunchArgument("base_type", default_value="uno"' in s1_launch_text
          and '"base_type": LaunchConfiguration("base_type")' in s1_launch_text,
          f"yaml_base_type={base_yaml_cfg.get('base_type')}")

    # 8. CmdVelNode 텔레메트리(~/telemetry) 퍼블리셔 및 주기 발행 타이머 검증 (보드계약 §11.1, §11.3)
    check("보드계약 §11.1 & §11.3 cmd_vel_node가 telemetry_hz(10.0Hz) 파라미터와 ~/telemetry 퍼블리셔를 선언한다",
          'self.declare_parameter("telemetry_hz", 10.0)' in cmd_node_text
          and 'self.create_publisher(String, "~/telemetry", 10)' in cmd_node_text
          and "_publish_telemetry" in cmd_node_text,
          "cmd_vel_node 텔레메트리 퍼블리셔 및 타이머 선언")

    check("stage1.yaml의 tomato_base telemetry_hz(10.0Hz) 설정이 일치한다",
          base_yaml_cfg.get("telemetry_hz") == 10.0,
          f"telemetry_hz={base_yaml_cfg.get('telemetry_hz')}")

    # 9. Telemetry.to_dict() 직렬화 무손실 검증 및 JSON 변환 호환성
    telem_dict = telem.to_dict()
    import json
    telem_json_str = json.dumps(telem_dict)
    check("보드계약 §11.1 Telemetry.to_dict()가 모든 필수 필드(ms, tgt, act, vin, amp, st 플래그)를 JSON 직렬화 가능하게 반환한다",
          isinstance(telem_dict, dict) and "ms" in telem_dict and "tgt" in telem_dict
          and "estop_latched" in telem_dict and len(telem_json_str) > 50,
          f"keys={list(telem_dict.keys())}")

    # 10. CmdVelNode estop(std_msgs/Bool) 구독 및 긴급정지 안전 연동 검증 (보드계약 §5.1, §9, §11.1, §12)
    check("보드계약 §11.1 & §12 cmd_vel_node가 estop(std_msgs/Bool) 구독과 _on_estop 핸들러를 선언한다",
          'self.create_subscription(Bool, "estop", self._on_estop, 10)' in cmd_node_text
          and "def _on_estop(" in cmd_node_text
          and "self._base.estop(self._estopped)" in cmd_node_text,
          "cmd_vel_node estop 구독 및 핸들러 선언")

    check("보드계약 §12 안전: cmd_vel_node가 estop 래치 중 수신된 /cmd_vel 지령을 즉시 차단하고 _halt를 수행한다",
          "if self._estopped:" in cmd_node_text
          and 'self._halt("비상정지 래치 중 — 지령 차단")' in cmd_node_text,
          "cmd_vel_node estop 래치 중 속도 지령 차단 안전 로직")

    # ⑬ [보드계약 v2] §12 프로토콜 계약 테스트 및 Response/ProtocolParser 검증 (docs/보드-계약.md §4, §5.4, §12)
    # 1. 체크섬 오류 시 nak crc 판정 및 카운트 누적
    parser = bc.ProtocolParser(expected_proto=2)
    r_bad_crc = parser.feed_line("C 350 0 0*99")
    check("보드계약 §12 프로토콜: 체크섬 불일치 수신 시 실행되지 않고 nak crc로 판정한다",
          r_bad_crc.is_nak and r_bad_crc.code == "crc" and parser.last_nak == "crc" and parser.nak_counts.get("crc") == 1,
          f"code={r_bad_crc.code} counts={parser.nak_counts}")

    # 2. 정상 체크섬 수신 후 strict 모드 전환 및 무체크섬 구동 명령 거부(nak nocrc)
    r_valid_cmd = parser.feed_line(f"ok S*{bc.checksum('ok S')}")
    check("보드계약 §12 프로토콜: 정상 체크섬 수신 시 strict_crc 상태로 승격된다",
          parser.strict_crc and r_valid_cmd.is_ok and r_valid_cmd.cmd == "S",
          f"strict={parser.strict_crc} cmd={r_valid_cmd.cmd}")

    r_nocrc = parser.feed_line("nak nocrc")
    check("보드계약 §12 프로토콜: strict 전환 후 무체크섬 구동 시 nak nocrc 응답을 파싱하고 추적한다",
          r_nocrc.is_nak and r_nocrc.code == "nocrc" and parser.last_nak == "nocrc",
          f"code={r_nocrc.code}")

    # 3. 지원하지 않는 명령 수신 시 nak unsupported 및 nak estop/nocalib 등 5대 nak 코드 파싱
    r_unsupported = parser.feed_line("nak unsupported")
    r_estop = parser.feed_line("nak estop")
    r_nocalib = parser.feed_line("nak nocalib")
    r_range = parser.feed_line("nak range")
    check("보드계약 §5.4 & §12 프로토콜: 모르는 명령 nak unsupported 및 5대 nak 규격이 정확히 식별된다",
          r_unsupported.code == "unsupported" and r_estop.code == "estop" and r_nocalib.code == "nocalib" and r_range.code == "range",
          f"unsupp={r_unsupported.code} estop={r_estop.code} nocalib={r_nocalib.code} range={r_range.code}")

    # 4. cap 파싱 및 proto 불일치 시 연결 거부 플래그 세우기
    r_cap_proto1 = parser.feed_line("cap proto=1 board=old-board")
    proto1_mismatch = parser.proto_mismatch
    r_cap_proto2 = parser.feed_line("cap proto=2 fw=3.0.0 board=stm32f411 id=A3F2C918 units=1 closed_loop=1 calib=1")
    check("보드계약 §12 프로토콜: proto 불일치(proto=1 vs expected=2) 시 거부 플래그를 세우고 정상 proto=2 수신 시 해제한다",
          proto1_mismatch and not parser.proto_mismatch and parser.caps is not None and parser.caps.board == "stm32f411",
          f"mismatch1={proto1_mismatch} mismatch2={parser.proto_mismatch} board={parser.caps.board if parser.caps else None}")

    # 5. ok X 1, ok X 0 비상정지 래치 응답 및 boot 리셋 원인 응답 파싱
    r_ok_x1 = parser.feed_line("ok X 1")
    r_boot = parser.feed_line("boot #3 cause=wdt last=S")
    check("보드계약 §5.1 & §12 프로토콜: ok X 비상정지 인자 응답 및 boot 리셋 원인 응답을 누락 없이 파싱한다",
          r_ok_x1.is_ok and r_ok_x1.cmd == "X" and r_ok_x1.args == ("1",)
          and r_boot.kind == "boot" and "cause=wdt" in r_boot.args,
          f"ok_x={r_ok_x1.cmd} args={r_ok_x1.args} boot={r_boot.kind} args={r_boot.args}")

    # 6. ResponseParser 클래스 별칭 및 parse_response 함수 인터페이스 일관성 검증
    check("보드계약 §12 프로토콜: ResponseParser 클래스 별칭이 ProtocolParser와 동일하게 유지된다",
          bc.ResponseParser is bc.ProtocolParser and hasattr(bc, "parse_response"),
          f"ResponseParser={bc.ResponseParser}")

    # ⑭ [보드계약 v2] Stm32Base 폐루프 구현체 및 §12 폐루프 계약 테스트 (docs/보드-계약.md §11.2, §12, §13 단계 4)
    class _DummyStmLink:
        def __init__(self) -> None:
            self.last_raw = ""
            self.stopped = False
        def send_raw(self, payload: str) -> bool:
            self.last_raw = payload
            self.stopped = (payload == "S")
            return True
        def stop(self) -> None:
            self.last_raw = "S"
            self.stopped = True

    # 1. Stm32Base가 MobileBase 프로토콜을 온전히 만족하며 기본 Caps가 폐루프(units=1, closed_loop=1, calib=1)이다
    stm_caps = bc.Caps(proto=2, board="stm32-closed-loop", fw="4.0.0", units=True, closed_loop=True, calib=True,
                       vmax_mms=800, vymax_mms=600, wmax_mdegs=180000)
    mock_link = _DummyStmLink()
    stm_base = bc.Stm32Base(motor_link=mock_link, caps=stm_caps)
    check("보드계약 §11.2 Stm32Base가 MobileBase 프로토콜을 충족하고 폐루프 능력을 선언한다",
          isinstance(stm_base, bc.MobileBase) and stm_base.caps().units and stm_base.caps().closed_loop and stm_base.caps().calib,
          f"caps={stm_base.caps()}")

    # 2. 물리 단위 지령(vx=350, vy=0, w=0) 전송 시 'C 350 0 0' 프레이밍 명령이 하위 링크에 전달된다
    stm_base.set_velocity(350, 0, 0)
    check("보드계약 §12 단위: Stm32Base가 물리 단위 지령 C vx vy w (350mm/s)를 링크에 전달한다",
          mock_link.last_raw == "C 350 0 0" and stm_base.telemetry().tgt == (350, 0, 0),
          f"last_raw={mock_link.last_raw} tgt={stm_base.telemetry().tgt}")

    # 3. 비상정지 래치(estop=True) 시 X 1 전송 및 후속 C 지령 차단(S 전송)
    stm_base.estop(True)
    check("보드계약 §12 안전: Stm32Base estop(True) 호출 시 X 1이 전송되고 텔레메트리 estop_latched(st bit0)가 설정된다",
          mock_link.last_raw == "X 1" and stm_base.telemetry().estop_latched and stm_base.telemetry().tgt == (0, 0, 0),
          f"last_raw={mock_link.last_raw} st=0x{stm_base.telemetry().st:02X}")
    stm_base.set_velocity(350, 0, 0)
    check("보드계약 §12 안전: Stm32Base estop 래치 중 속도 지령 수신 시 C를 차단하고 stop을 유지한다",
          mock_link.last_raw == "S" and stm_base.telemetry().tgt == (0, 0, 0),
          f"last_raw={mock_link.last_raw} tgt={stm_base.telemetry().tgt}")

    # 4. stop() 호출 시 슬루를 무시하고 즉시 S 전송 및 목표 0 리셋
    stm_base.estop(False)
    stm_base.set_velocity(200, -100, 45000)
    stm_base.stop()
    check("보드계약 §12 안전: Stm32Base stop() 호출 시 슬루를 무시하고 S를 전송하며 목표가 0으로 정지된다",
          mock_link.last_raw == "S" and stm_base.telemetry().tgt == (0, 0, 0),
          f"last_raw={mock_link.last_raw} tgt={stm_base.telemetry().tgt}")

    # 4b. 0 속도 지령 set_velocity(0,0,0) 인가 시 슬루를 무시하고 즉시 S 전송 및 목표 (0,0,0) 소멸
    stm_base.set_velocity(350, 0, 0)
    stm_base.set_velocity(0, 0, 0)
    check("보드계약 §12 안전: Stm32Base가 set_velocity(0,0,0) 수신 시 즉시 S 전송 및 telemetry().tgt를 (0,0,0)으로 리셋한다",
          mock_link.last_raw == "S" and stm_base.telemetry().tgt == (0, 0, 0),
          f"last_raw={mock_link.last_raw} tgt={stm_base.telemetry().tgt}")

    # 5. feed_line을 통한 폐루프 하트비트(hb ... act=... st=...) 수신 및 Telemetry 실측 속도 반영
    hb_frame = f"hb 125000 rx=100 bad=0 i2c=0 wdt=0 st=0x08 tgt=350,0,0 act=348,0,0 vin=12550 amp=850*{bc.checksum('hb 125000 rx=100 bad=0 i2c=0 wdt=0 st=0x08 tgt=350,0,0 act=348,0,0 vin=12550 amp=850')}"
    resp_hb = stm_base.feed_line(hb_frame)
    telem = stm_base.telemetry()
    check("보드계약 §12 계측: Stm32Base가 hb 실측 속도(act=348,0,0)와 텔레메트리(vin=12550mV, amp=850mA)를 온전히 반영한다",
          resp_hb.is_ok is False and resp_hb.kind == "hb" and telem.act == (348, 0, 0)
          and telem.tgt == (350, 0, 0) and telem.vin_mv == 12550 and telem.amp_ma == 850
          and telem.calib_valid,
          f"act={telem.act} vin={telem.vin_mv} amp={telem.amp_ma} st=0x{telem.st:02X}")

    # 6. 하트비트 수신 이후 stop() 호출 시 telemetry().tgt 즉시 0 반영 (하트비트 시차 지연/거짓 상태 노출 차단)
    stm_base.stop()
    check("보드계약 §12 안전: Stm32Base가 하트비트 수신 이후에도 stop() 즉시 telemetry().tgt를 (0, 0, 0)으로 소멸시킨다",
          stm_base.telemetry().tgt == (0, 0, 0) and mock_link.last_raw == "S",
          f"tgt={stm_base.telemetry().tgt} last_raw={mock_link.last_raw}")

    # 7. 하트비트 수신 이후 estop(True) 호출 시 telemetry().estop_latched 즉시 True 반영 및 해제(estop(False)) 시 복구
    stm_base.estop(True)
    latched_after_hb = stm_base.telemetry().estop_latched
    stm_base.estop(False)
    unlatched_after_hb = not stm_base.telemetry().estop_latched
    check("보드계약 §12 안전: Stm32Base가 하트비트 수신 이후에도 estop(True/False) 호출 즉시 텔레메트리 래치 상태를 동기화한다",
          latched_after_hb and unlatched_after_hb,
          f"latched={latched_after_hb} unlatched={unlatched_after_hb}")

    # 8. estop 래치 상태에서 set_velocity 호출 시 거절 플래그(rejected=True) 및 사유 보존
    stm_base.estop(True)
    stm_base.set_velocity(350, 0, 0)
    last_cmd = getattr(stm_base, "_last_cmd", None)
    check("보드계약 §12 안전: Stm32Base estop 래치 중 속도 지령 시 rejected=True 및 비상정지 사유를 보존한다",
          last_cmd is not None and last_cmd.rejected and "비상정지" in last_cmd.reason,
          f"last_cmd={last_cmd}")
    stm_base.estop(False)

    # ⑮ [계측 감사] MobileBase 축 부호(AxisSigns) 계약 통합 및 DutyCalib 물리 한계 검산 (docs/보드-계약.md §2, §8, §14)
    # 1. Stm32Base가 AxisSigns를 수용하고 C 물리 지령 발행 시 부호 변환을 정확히 반영한다
    mock_link_signs = _DummyStmLink()
    stm_signs_base = bc.Stm32Base(motor_link=mock_link_signs, caps=stm_caps, signs=bc.AxisSigns(vx=1, vy=-1, w=-1))
    stm_signs_base.set_velocity(350, 200, 30000)
    check("보드계약 §14.1 계측: Stm32Base가 AxisSigns(vy=-1, w=-1) 부호 변환을 C 물리 지령에 정확히 반영한다",
          mock_link_signs.last_raw == "C 350 -200 -30000",
          f"last_raw={mock_link_signs.last_raw}")

    # 2. SimBase가 AxisSigns를 수용하고 목표 속도(_tgt)에 부호 변환을 정확히 반영한다
    sim_signs_base = bc.SimBase(signs=bc.AxisSigns(vx=1, vy=-1, w=-1))
    sim_signs_base.set_velocity(350, 200, 30000)
    check("보드계약 §14.1 계측: SimBase가 AxisSigns(vy=-1, w=-1) 부호 변환을 목표 속도(_tgt)에 정확히 반영한다",
          sim_signs_base.telemetry().tgt == (350, -200, -30000),
          f"sim_tgt={sim_signs_base.telemetry().tgt}")

    for _ in range(25):
        sim_signs_base.step(0.05)
    check("보드계약 §14.1 계측: SimBase가 AxisSigns(vy=-1, w=-1) 적용 시 act 실측 속도를 tgt와 일치하게 정상 상태 추종한다",
          sim_signs_base.telemetry().act == (350, -200, -30000),
          f"sim_act={sim_signs_base.telemetry().act}")

    # 3. CmdVelNode 팩토리가 4대 베이스(uno, sim, stm32, mock) 생성 시 signs를 온전히 전달한다
    check("보드계약 §14.1 계측: cmd_vel_node가 SimBase 및 Stm32Base 생성 시 signs=self._signs를 온전히 전달한다",
          "SimBase(deadman_enabled=True, signs=self._signs)" in cmd_node_text
          and "Stm32Base(motor_link=self._link, signs=self._signs)" in cmd_node_text
          and "UnoAdapterBase(motor_link=self._link, calib=self._calib, signs=self._signs)" in cmd_node_text
          and "MockBase(signs=self._signs)" in cmd_node_text,
          "cmd_vel_node signs 전달 검증")

    # 4. DutyCalib 물리 한계 검산: ks >= 0, kv > 0 및 이론적 최고속도(vmax, wmax)가 물리 허용 범위 내 안착
    d_calib = bc.DutyCalib()
    vmax_ok = 350.0 <= d_calib.vmax_mms <= 600.0
    wmax_ok = 120.0 <= d_calib.wmax_degs <= 200.0
    check("보드계약 §8 계측: DutyCalib 환산 계수(ks=90, kv=0.35) 기반 최고 속도(vmax, wmax)가 물리 상한 범위에 안착한다",
          d_calib.ks == 90 and d_calib.ks_w == 90 and vmax_ok and wmax_ok and d_calib.measured is False,
          f"vmax={d_calib.vmax_mms:.1f} wmax={d_calib.wmax_degs:.1f} measured={d_calib.measured}")

    # ⑯ [조작대·주행] click_server 차체 넛지(nudge) 조작 및 base_nudge 연동 검증
    # 터미널에서만 되는 조작(base_nudge.py)을 남기지 않고 조작대 UI/API에서도 넛지가 가능하도록 연동
    sys.path.insert(0, os.path.join(ROS2, "tools"))
    import click_server as cs  # noqa: E402
    nudge_cmd = cs.build("nudge", {"vx": 130, "secs": 0.6})
    check("조작대: click_server가 nudge(전진 130 duty, 0.6s) 요청을 base_nudge 명령줄로 올바르게 변환한다",
          len(nudge_cmd) >= 5 and "base_nudge.py" in nudge_cmd[1] and "--vx" in nudge_cmd and "130" in nudge_cmd and "--secs" in nudge_cmd,
          f"cmd={' '.join(nudge_cmd[1:])}")

    nudge_zero_rejected = False
    try:
        cs.build("nudge", {"vx": 0, "vy": 0, "w": 0})
    except ValueError:
        nudge_zero_rejected = True
    check("조작대: click_server가 0 지령 넛지 요청을 조용히 무시하지 않고 명확히 거절(ValueError)한다",
          nudge_zero_rejected,
          f"zero_rejected={nudge_zero_rejected}")

    # 3. click_server nudge dither 불리언(True) 인자 기본 진폭 25 duty 자동 변환 검증 (감사)
    nudge_dither_cmd = cs.build("nudge", {"vx": 130, "dither": True})
    check("조작대: click_server가 nudge dither=True 불리언 입력을 기본 진폭 25 duty로 올바르게 변환한다",
          "--dither" in nudge_dither_cmd and "25" in nudge_dither_cmd,
          f"dither_cmd={' '.join(nudge_dither_cmd[1:])}")

    # 4. base_nudge.py의 중단/예외 발생 시 link.stop() 및 close() 보장 검증 (감사)
    base_nudge_text = open(os.path.join(ROS2, "tools", "base_nudge.py"), encoding="utf-8").read()
    check("조작대·주행: base_nudge.py가 중단/예외 시에도 안전 정지(link.stop 및 close)를 보장하는 try-finally 구문을 구비한다",
          "try:" in base_nudge_text and "finally:" in base_nudge_text and "link.stop()" in base_nudge_text and "link.close()" in base_nudge_text,
          "base_nudge 안전 정지 구문 검증")

    # 5. Stm32Base가 set_velocity 시점에 AxisSigns를 반영하여 SimBase 및 hb 수신 후와 일관된 telemetry().tgt를 유지한다 (감사)
    check("보드계약 §12·§14 감사: Stm32Base가 set_velocity 시점에 AxisSigns(vy=-1, w=-1)를 반영하여 SimBase 및 hb와 일관된 telemetry().tgt를 유지한다",
          stm_signs_base.telemetry().tgt == (350, -200, -30000) and stm_signs_base.telemetry().tgt == sim_signs_base.telemetry().tgt,
          f"stm_tgt={stm_signs_base.telemetry().tgt} sim_tgt={sim_signs_base.telemetry().tgt}")

    # 6. UnoAdapterBase가 set_velocity 시점에 AxisSigns(vy=-1, w=-1)를 반영하여 cmd.physical과 일치하는 telemetry().tgt를 유지한다 (T86)
    uno_signs_base = bc.UnoAdapterBase(signs=bc.AxisSigns(vx=1, vy=-1, w=-1))
    uno_signs_base.set_velocity(350, 200, 30000)
    check("보드계약 §11·§14 계측: UnoAdapterBase가 set_velocity 시점에 AxisSigns(vy=-1, w=-1)를 반영하여 cmd.physical과 일치하는 telemetry().tgt를 유지한다",
          uno_signs_base.telemetry().tgt == (350, -200, -30000) and uno_signs_base.telemetry().tgt == stm_signs_base.telemetry().tgt,
          f"uno_tgt={uno_signs_base.telemetry().tgt}")

    # 7. MockBase가 set_velocity 시점에 AxisSigns(vy=-1, w=-1)를 반영하여 4대 베이스와 일치하는 telemetry().tgt를 유지한다 (T86)
    mock_signs_base = bc.MockBase(signs=bc.AxisSigns(vx=1, vy=-1, w=-1))
    mock_signs_base.set_velocity(350, 200, 30000)
    check("보드계약 §11·§14 계측: MockBase가 set_velocity 시점에 AxisSigns(vy=-1, w=-1)를 반영하여 4대 베이스와 일치하는 telemetry().tgt를 유지한다",
          mock_signs_base.telemetry().tgt == (350, -200, -30000) and mock_signs_base.telemetry().tgt == stm_signs_base.telemetry().tgt,
          f"mock_tgt={mock_signs_base.telemetry().tgt}")

    # 8. UnoAdapterBase가 vy=-1 부호 반전 시 물리 duty에 음수 dy를 전달하고 w=1(반시계) 보존을 확인한다 (사이클 588 감사)
    uno_audit_link = type("AuditLink", (), {
        "last_cmd": None,
        "set_velocity": lambda self, *a: setattr(self, "last_cmd", a),
        "stop": lambda self: setattr(self, "last_cmd", (0, 0, 0)),
    })()
    uno_audit_base = bc.UnoAdapterBase(motor_link=uno_audit_link, calib=calib, signs=bc.AxisSigns(vx=1, vy=-1, w=1))
    uno_audit_base.set_velocity(0, 200, 30000)
    check("보드계약 §14.1 감사: UnoAdapterBase가 vy=-1 부호 반전 시 물리 duty에 음수 dy를 전달하고 w=1(반시계)을 보존한다",
        uno_audit_link.last_cmd == (0, -160, 123) and uno_audit_base.telemetry().tgt == (0, -200, 30000),
        f"last_cmd={uno_audit_link.last_cmd} tgt={uno_audit_base.telemetry().tgt}")

    # 9. UnoAdapterBase 기본 생성자 호출 시 signs 생략해도 AxisSigns(vx=1, vy=-1, w=1)가 기본 적용된다 (T87)
    uno_default_base = bc.UnoAdapterBase(motor_link=uno_audit_link, calib=calib)
    uno_default_base.set_velocity(0, 200, 30000)
    check("보드계약 §14.1 빌더: UnoAdapterBase 기본 생성 시 AxisSigns(vy=-1, w=1)가 기본 적용되어 vy=-1 반전과 w=1을 보장한다",
        uno_audit_link.last_cmd == (0, -160, 123) and uno_default_base.telemetry().tgt == (0, -200, 30000)
        and uno_default_base._signs == bc.AxisSigns(vx=1, vy=-1, w=1),
        f"last_cmd={uno_audit_link.last_cmd} tgt={uno_default_base.telemetry().tgt} signs={uno_default_base._signs}")

    # 10. UnoAdapterBase가 cmd_vel_node 기본 REP-103 AxisSigns(1, 1, 1) 주입 시에도 vy=-1 물리 반전을 적용한다 (사이클 598 감사)
    uno_node_base = bc.UnoAdapterBase(motor_link=uno_audit_link, calib=calib, signs=bc.AxisSigns(vx=1, vy=1, w=1))
    uno_node_base.set_velocity(0, 200, 30000)
    check("보드계약 §14.1 감사: UnoAdapterBase가 cmd_vel_node 기본 REP-103 AxisSigns(1, 1, 1) 주입 시에도 vy=-1 물리 반전을 보장한다",
        uno_audit_link.last_cmd == (0, -160, 123) and uno_node_base.telemetry().tgt == (0, -200, 30000)
        and uno_node_base._signs == bc.AxisSigns(vx=1, vy=-1, w=1),
        f"last_cmd={uno_audit_link.last_cmd} tgt={uno_node_base.telemetry().tgt} signs={uno_node_base._signs}")





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

    near = depth.copy()
    near[disk] = 40.0
    r_near = read_blob(INTR, near, blob, disk)
    check("D405 근거리 블라인드(하한 60mm 미만)는 거절한다 (사각지대 원점 오인 방어)",
          not r_near.ok and "유효 깊이" in r_near.reason,
          r_near.reason[:60])

    empty_mask = np.zeros((200, 200), dtype=bool)
    r_empty = read_blob(INTR, depth, blob, empty_mask)
    check("화소 없는 극소형 덩이(candidates==0)는 안전 거절한다",
          not r_empty.ok and "화소가 없다" in r_empty.reason,
          r_empty.reason[:60])

    # 복수 층 잠입 — 중앙값 300mm가 65%라 MAD=0(spread=0)이지만 inlier 비율 < 70%
    core = core_mask((200, 200), blob) & disk
    core_idx = np.where(core)
    split_layer = depth.copy()
    split_cut = int(len(core_idx[0]) * 0.65)
    split_layer[core_idx[0][split_cut:], core_idx[1][split_cut:]] = 340.0
    r_split = read_blob(INTR, split_layer, blob, disk)
    check("중앙값 편차(MAD) 0이어도 inlier 비율 70% 미만이면 거절 (복수 층 잠입 가드)",
          not r_split.ok and r_split.spread_mm == 0.0 and "깊이가 한 층이 아니다" in r_split.reason,
          r_split.reason[:60])

    readings = read_all(INTR, depth, [blob, blob], masks=[disk, empty_mask])
    check("read_all()이 정상 및 거절 열매를 누락 없이 순서대로 반환한다",
          len(readings) == 2 and readings[0].ok and not readings[1].ok,
          f"0={readings[0].ok}, 1={readings[1].ok}")

    expected_r_mm = 30.0 * 300.0 / ((INTR.fx + INTR.fy) / 2.0)
    check("열매 물리 반지름(radius_mm)이 깊이와 초점거리 비례식과 엄밀 일치한다",
          abs(r.radius_mm - expected_r_mm) < 1e-9 and r.point_mm == (0.0, 0.0, 300.0),
          f"calced={r.radius_mm} expected={expected_r_mm}")

    import ast
    from tomato_picker import config as tp_config

    with open(os.path.join(SRC, "tomato_perception", "tomato_perception", "detect_node.py"),
              encoding="utf-8") as f:
        detect_tree = ast.parse(f.read())
    dn_red, dn_green = None, None
    for stmt in detect_tree.body:
        if isinstance(stmt, ast.Assign) and len(stmt.targets) == 1 and isinstance(stmt.targets[0], ast.Name):
            if stmt.targets[0].id == "RED_RANGES":
                dn_red = ast.literal_eval(stmt.value)
            elif stmt.targets[0].id == "GREEN_RANGE":
                dn_green = ast.literal_eval(stmt.value)

    with open(os.path.join(SRC, "tomato_bringup", "config", "stage1.yaml"),
              encoding="utf-8") as f:
        stage1_cfg = yaml.safe_load(f)
    det_cfg = stage1_cfg.get("tomato_detect", {}).get("ros__parameters", {})
    eye_cfg = stage1_cfg.get("tomato_handeye", {}).get("ros__parameters", {})

    expected_red_ranges = [[lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]] for lo, hi in tp_config.RED_HSV_RANGES]
    expected_green_range = [tp_config.GREEN_HSV_RANGE[0][0], tp_config.GREEN_HSV_RANGE[0][1], tp_config.GREEN_HSV_RANGE[0][2],
                            tp_config.GREEN_HSV_RANGE[1][0], tp_config.GREEN_HSV_RANGE[1][1], tp_config.GREEN_HSV_RANGE[1][2]]
    expected_red_flat = [float(v) for pair in tp_config.RED_HSV_RANGES for bound in pair for v in bound]
    expected_green_flat = [float(v) for bound in tp_config.GREEN_HSV_RANGE for v in bound]

    check("detect_node.py 및 stage1.yaml의 HSV 색상 범위와 min_pixels가 config.py와 일치한다",
          dn_red == expected_red_ranges
          and dn_green == expected_green_range
          and det_cfg.get("red_ranges") == expected_red_flat
          and det_cfg.get("green_range") == expected_green_flat
          and det_cfg.get("min_pixels") == tp_config.MIN_FRUIT_AREA_PX,
          f"min_px={det_cfg.get('min_pixels')} red_matches={det_cfg.get('red_ranges') == expected_red_flat}")

    camera_topics = (
        "/camera/camera/color/image_raw",
        "/camera/camera/aligned_depth_to_color/image_raw",
        "/camera/camera/color/camera_info",
    )
    check("stage1.yaml의 tomato_detect/tomato_handeye 3대 카메라 토픽이 stage1.launch.py와 일치한다",
          (det_cfg.get("color_topic"), det_cfg.get("depth_topic"), det_cfg.get("info_topic")) == camera_topics
          and (eye_cfg.get("color_topic"), eye_cfg.get("depth_topic"), eye_cfg.get("info_topic")) == camera_topics,
          f"detect={det_cfg.get('color_topic')} eye={eye_cfg.get('color_topic')}")

    # T89: yolo_seg STEM_CLASS_NAMES 및 extract_stem_masks ↔ stem_cut 연동 계약
    from tomato_perception.yolo_seg import (
        STEM_CLASS_NAMES, YoloDetection, extract_stem_masks,
    )
    from tomato_perception.stem_cut import find_cut_point

    check("T89 계측: yolo_seg STEM_CLASS_NAMES에 stem/peduncle/calyx가 포함된다",
          {"stem", "peduncle", "calyx"}.issubset(STEM_CLASS_NAMES),
          f"{STEM_CLASS_NAMES}")

    # 가짜 줄기 및 과실 마스크
    s_mask = np.zeros((40, 40), dtype=bool)
    s_mask[5:35, 20] = True
    f_mask = np.zeros((40, 40), dtype=bool)
    f_mask[:6, 15:26] = True

    stem_dets = [
        YoloDetection(mask=s_mask, confidence=0.88, class_name="stem"),
        YoloDetection(mask=np.zeros((40, 40), dtype=bool), confidence=0.9, class_name="ripe"),
    ]
    extracted_stems = extract_stem_masks(stem_dets, min_pixels=20)
    check("T89 계측: yolo_seg.extract_stem_masks가 줄기 검출을 필터링하여 정확히 추출한다",
          len(extracted_stems) == 1 and extracted_stems[0].sum() == s_mask.sum(),
          f"count={len(extracted_stems)}")

    c_pt = find_cut_point(extracted_stems[0], f_mask, px_per_mm=1.0, cut_offset_mm=10.0)
    check("T89 계측: yolo_seg 추출 줄기 마스크가 stem_cut.find_cut_point에 정상 연동된다",
          c_pt is not None and abs(c_pt.v - 15.0) <= 2.0,
          f"c_pt={c_pt}")



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

    # ⚠ **--fix-t의 원점이 무엇인가** (2026-09-18 사람 지시). 사람이 자로 재는
    #   세 숫자는 "TCP(집게가 무는 지점) → 카메라 렌즈 중심"이지 "손목(wrist_roll)
    #   축 → 렌즈 중심"이 아니다. 두 도구의 t가 **둘 다 kin.forward()의 (x,y,z)**
    #   에서 오기 때문이다 — 손목축에서 재면 접근축 성분이 l3(=168mm)만큼 틀리고,
    #   그 한 번의 착각이 15mm 문턱을 열 배로 넘긴다. 문서만으로는 다음 사람이
    #   또 손목축을 잰다(작업판 T10의 제목이 실제로 그렇게 적혀 있었다).
    import numpy as np  # noqa: E402
    from tomato_picker.hardware import kinematics as kin  # noqa: E402
    sys.path.insert(0, os.path.join(ROS2, "tools"))
    import handeye_collect as hc  # noqa: E402

    probe = {"shoulder_pan": 12.0, "shoulder_lift": 47.0, "elbow_flex": -31.0,
             "wrist_flex": -22.0, "wrist_roll": 0.0}
    g = kin.ArmGeometry()
    tcp = kin.forward(probe, g)
    _, t_res = hr.tool_frame(probe, g, hc.ROLL_SIGN)
    t_col = hc.tool_frame_from_joints(probe, g).t
    check("handeye_resolve의 원점이 TCP다 (kin.forward)",
          float(np.max(np.abs(t_res - np.array([tcp.x, tcp.y, tcp.z])))) < 1e-9,
          f"{np.round(t_res, 3).tolist()} vs TCP({tcp.x:.1f},{tcp.y:.1f},{tcp.z:.1f})")
    check("handeye_collect의 원점도 같은 함수에서 온다",
          float(np.max(np.abs(np.asarray(t_col) - t_res))) < 1e-9,
          f"{np.round(np.asarray(t_col), 3).tolist()}")
    # 원점이 손목축이었다면 접근축으로 l3만큼 떨어져 있어야 한다 — 이 검사가
    # 무엇을 막는지. (l3=168mm이니 착각 한 번에 15mm 문턱을 열 배로 넘긴다)
    wrist = t_res - hr.tool_frame(probe, g, hc.ROLL_SIGN)[0][:, 0] * g.l3
    check("손목축 원점과 TCP 원점은 l3만큼 다르다",
          abs(float(np.linalg.norm(t_res - wrist)) - g.l3) < 1e-6,
          f"l3={g.l3}mm — 어느 쪽에서 쟀는지가 이만큼을 가른다")
    check("--fix-t 도움말이 원점을 TCP라고 말한다",
          "TCP에서 카메라 렌즈 중심까지" in src)

    # ⚠ lateral·up 축은 wrist_roll과 **함께 돈다** — 그래서 자로 재는 자세는
    #   반드시 roll=0이어야 하고, 사람이 그 자세를 만들 길이 **조작대에** 있어야
    #   한다(터미널에서만 되는 조작을 남기지 않는다는 이 저장소의 규칙).
    rolled = dict(probe, wrist_roll=90.0)
    R0 = hr.tool_frame(probe, g, hc.ROLL_SIGN)[0]
    R90 = hr.tool_frame(rolled, g, hc.ROLL_SIGN)[0]
    check("wrist_roll이 lateral·up 축을 돌린다 (재는 자세가 roll=0이라야 하는 이유)",
          float(np.max(np.abs(R0[:, 0] - R90[:, 0]))) < 1e-9
          and float(np.max(np.abs(R0[:, 1] - R90[:, 1]))) > 0.5,
          "approach는 그대로고 lateral/up만 돈다")
    cs = open(os.path.join(ROS2, "tools", "click_server.py"), encoding="utf-8").read()
    st = open(os.path.join(ROS2, "tools", "arm_stage.py"), encoding="utf-8").read()
    check("조작대에 손목 롤 0 경로가 있다",
          'if job == "roll0":' in cs and "run('roll0'" in cs,
          "버튼과 curl과 에이전트가 같은 경로를 쓴다")
    check("그 경로가 나머지 관절을 건드리지 않는다",
          '"--set", "wrist_roll=0"' in cs and '"--set"' not in st.split("def main")[0],
          "넷을 받아적으면 받아적는 사이에 처진 자리가 목표가 된다")
    check("arm_stage가 --set으로 관절 일부만 받는다",
          'dest="set_joints"' in st and "j not in kin.JOINTS" in st,
          "없는 관절 이름은 거절해야 한다 — 조용히 무시하면 팔이 안 움직인다")

    import handeye_from_axes as hfa  # noqa: E402
    hfa_axes, hfa_R, hfa_tcp = hfa.model_axes(probe, g)
    check("handeye_from_axes의 원점도 TCP다 (kin.forward)",
          float(np.linalg.norm(hfa_tcp - np.array([tcp.x, tcp.y, tcp.z]))) < 1e-9)
    check("handeye_from_axes의 도구 프레임 R도 handeye_resolve와 일치한다",
          float(np.linalg.norm(hfa_R - R0)) < 1e-9)

    import target_check as tc  # noqa: E402
    check("target_check 4점 표적 규격이 2026-08-31 실측치와 일치한다 (100x174.5mm)",
          tc.EXPECT_W == 100.0 and tc.EXPECT_H == 174.5 and abs(float(np.hypot(tc.EXPECT_W, tc.EXPECT_H)) - 201.1225) < 0.01)
    check("target_check 4점 표적 허용오차가 12mm다",
          tc.QUAD_TOL_MM == 12.0)


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

    # ── ⑦ solve_fixed_t는 자로 잰 t_x를 정확히 보존하고 회전을 푼다 (T10/T13/기준2) ──
    # 사람이 자로 잰 t_x(§28: -52, 0, 59 mm)를 상수로 박고 회전만 훑을 때:
    # (1) t_x가 입력값 그대로 고정되어 출력되는가
    # (2) 정답 t_x에서 rms가 수렴(<0.5mm)하고 회전행렬이 직교하는가
    # (3) 틀린 t_x(20mm 오프셋)를 넣으면 잔차가 증가해 잘못된 입력을 걸러내는가
    S_fix = _synthetic_handeye([-52.0, 0.0, 59.0])
    fixed_res = hr.solve_fixed_t(S_fix, geom, +1.0, ("mid",), [-52.0, 0.0, 59.0], tries=2000)
    check("solve_fixed_t가 입력 t_x를 그대로 보존한다",
          float(np.max(np.abs(fixed_res[2] - np.array([-52.0, 0.0, 59.0])))) < 1e-9,
          f"t_x: {fixed_res[2]}")
    check("정답 t_x에서 solve_fixed_t의 rms가 0.5mm 이하로 수렴한다",
          fixed_res[0] < 0.5, f"rms={fixed_res[0]:.4f}mm")
    R_fix = fixed_res[1]
    check("solve_fixed_t가 구한 회전행렬이 정규직교 오른손 회전이다",
          abs(float(np.linalg.det(R_fix)) - 1.0) < 1e-6
          and float(np.max(np.abs(R_fix.T @ R_fix - np.eye(3)))) < 1e-6,
          f"det={np.linalg.det(R_fix):.6f}")
    wrong_res = hr.solve_fixed_t(S_fix, geom, +1.0, ("mid",), [-72.0, 0.0, 59.0], tries=2000)
    check("t_x가 20mm 틀리면 solve_fixed_t 잔차가 확실히 증가한다 (거짓 통과 방지)",
          wrong_res[0] > fixed_res[0] + 1.0,
          f"정답 {fixed_res[0]:.2f}mm vs 틀림 {wrong_res[0]:.2f}mm")


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

    # ⚠ T51: 조작대에도 뻗기(arm_extend)를 붙였다 — ssh로 lerobot venv를 불러야만
    #   되던 조작이라 아침 5분에 ssh가 끼면 그 5분이 안 끝났다. 목표자세·걸음
    #   상한은 escape.py 하나가 갖고, 여기서 다시 정의하면 베껴서 갈라진다
    #   (arm_extend.py 상단 주석과 같은 이유) — 그래서 조작대는 값을 넘기지
    #   않고 arm_extend.py가 escape.TARGET_DEG/plan을 그대로 쓰게 둔다.
    sys.path.insert(0, os.path.join(ROS2, "tools"))
    import click_server as csrv  # noqa: E402
    cs_src = open(os.path.join(ROS2, "tools", "click_server.py"), encoding="utf-8").read()
    check("조작대에 특이점 탈출(extend) 경로가 있다",
          'if job == "extend":' in cs_src and "run('extend'" in cs_src,
          "버튼과 curl과 에이전트가 같은 경로를 쓴다")
    argv_default = csrv.build("extend", {})
    check("조작대의 extend가 arm_extend.py를 부른다",
          argv_default[-1].endswith("arm_extend.py"), " ".join(argv_default))
    check("target을 안 주면 --target을 안 붙인다 (escape.TARGET_DEG를 그대로 쓴다)",
          not any(a.startswith("--target") for a in argv_default), argv_default)
    check("dry=1이면 --dry가 붙는다",
          "--dry" in csrv.build("extend", {"dry": True}))
    check("target을 주면 그대로 넘긴다 (걸음 쪼개기는 arm_extend.py의 escape.plan 몫)",
          "--target=1,2,3,4,5" in csrv.build("extend", {"target": "1,2,3,4,5"}))
    check("조작대가 목표자세·걸음상한 상수를 따로 정의하지 않는다",
          "TARGET_DEG =" not in cs_src and "STEP_DEG =" not in cs_src,
          "베끼면 escape.py와 조용히 갈라진다")

    # ⚠ T70: arm_extend.py 종료 시 항상 토크를 켠 채 포트를 닫는다(hold_close).
    #   끄면 팔이 바닥 아래로 떨어진다(§26). 죽은 플래그 --hold는 제거됐다.
    ax_src = open(os.path.join(ROS2, "tools", "arm_extend.py"), encoding="utf-8").read()
    check("arm_extend.py에 죽은 --hold 인자가 없다 (T70)",
          '"--hold"' not in ax_src,
          "항상 hold하는 것이 안전 쪽 원칙이므로 불필요한 인자를 없앴다")
    check("arm_extend.py 종료 시 무조건 io.hold_close()를 호출한다 (T70)",
          "io.hold_close()" in ax_src and "io.close()" not in ax_src,
          "토크를 끄면 팔이 떨어지므로 예외 없이 hold_close()로 포트를 닫는다")


def test_arm_stage_park() -> None:
    """A자세에서 PARK 목표로 가는 경로가 막히지 않는다 (T60, 2026-09-18).

    사이클45·49 실기: park가 A자세뿐 아니라 아무 자세에서나 막힌다.
    tester 추가확인: wrist_flex+15.2° 남긴 근접 자세에서도 4번 전부 wrist_flex한계.
    원인: click_server PARK='60,65,0,-100,6'에서 wrist_flex=-100이 정규화 -100이고,
    arm_stage의 ±98 한계에 걸려 경유점 전부가 막혔다.
    고침: arm_stage가 목표 정규화값을 ±98로 자른 뒤 경로를 짠다(escape.clamp_norm
    과 같은 원칙 — 목적은 팔을 접는 것이지 정확히 -100°가 아니다).
    이 검사는 그 고침이 실제로 코드에 있는지와, 수치로 경로가 막히지 않는지를 확인한다.
    """
    print("\n[park] A자세에서 PARK 경로 막힘 수정 (T60)")
    sys.path.insert(0, os.path.join(REPO, "ros2", "tools"))
    import arm_stage as astage  # noqa: E402

    st_src = open(os.path.join(ROS2, "tools", "arm_stage.py"), encoding="utf-8").read()
    # 1) 클램핑 코드가 arm_stage.py에 있는가
    check("arm_stage가 목표 정규화값을 ±98로 자르는 코드를 갖는다 (T60)",
          "_target_norm" in st_src and "_clamped_norm" in st_src and "_clipped" in st_src,
          "wrist_flex=-100 목표가 경로 전체를 막던 원인을 해소한다")
    check("arm_stage의 클램핑이 escape.clamp_norm()과 같은 98 경계를 쓴다 (T60)",
          "max(-98.0, min(98.0" in st_src,
          "escape.py의 LIMIT_NORM = 100 - ARM_CART_NORM_MARGIN = 98")

    # 2) PARK wrist_flex=-100이 이 프레임에서 실제로 98을 넘는가 (막히던 원인 수치 확인)
    f = ESCAPE_FRAME_2026_09_18
    # PARK 목표각(도)
    park_deg = {"shoulder_pan": 60.0, "shoulder_lift": 65.0,
                "elbow_flex": 0.0, "wrist_flex": -100.0, "wrist_roll": 6.0}
    park_norm = {j: f["zero"][j] + (park_deg[j] - f["ref"][j]) / f["dpn"][j]
                 for j in kin.JOINTS}
    wf_norm = park_norm["wrist_flex"]
    check("PARK wrist_flex=-100°는 이 프레임에서 정규화 절댓값이 98을 넘는다 (막히던 근거)",
          abs(wf_norm) > 98.0,
          f"wrist_flex 정규화={wf_norm:.2f} (|{wf_norm:.2f}|>{98.0})")

    # 3) 클램핑 후 wrist_flex가 ±98 안에 들어오는가
    clamped_wf = max(-98.0, min(98.0, wf_norm))
    # 클램핑된 목표를 도(°)로 역변환
    clamped_deg_wf = f["ref"]["wrist_flex"] + (clamped_wf - f["zero"]["wrist_flex"]) * f["dpn"]["wrist_flex"]
    park_clamped_deg = {**park_deg, "wrist_flex": clamped_deg_wf}
    check("클램핑 후 wrist_flex 정규화가 ±98 이내다 (경로 통과)",
          abs(clamped_wf) <= 98.0 + 1e-9,
          f"클램핑 후 정규화={clamped_wf:.2f}, 해당 도={clamped_deg_wf:.1f}°")

    # 4) A자세(ESCAPE_NORM_2026_09_18)에서 클램핑된 PARK로 가는 경로가 안전한가
    #    arm_stage의 leg() 로직을 그대로 시뮬레이션: ±98 한계 + 바닥 검사
    geom = kin.ArmGeometry()
    a_deg = _escape_deg(ESCAPE_NORM_2026_09_18)  # A자세 도(°)
    a_norm = ESCAPE_NORM_2026_09_18
    z_floor = -astage.MOUNT_Z_MM + astage.FLOOR_MARGIN_MM

    def _stage_check(start_deg, tgt_deg, label):
        """arm_stage의 leg() 한 구간 시뮬레이션 — 막히는 구간이 없으면 True."""
        import math as _math
        blocked_joints = []
        for joint in kin.JOINTS:
            delta = abs(tgt_deg[joint] - start_deg[joint])
            if delta < 0.05:
                continue
            steps = max(1, int(_math.ceil(delta / astage.STEP_DEG)))
            for s in range(1, steps + 1):
                mid = {j: start_deg[j] + (tgt_deg[j] - start_deg[j]) * s / steps
                       for j in kin.JOINTS}
                p = kin.forward(mid, geom)
                if p.z < z_floor:
                    blocked_joints.append(f"{joint}:바닥아래@구간{s}")
                    continue
                # 정규화 계산 (가짜 프레임으로)
                mid_norm = _escape_norm(mid)
                for j in kin.JOINTS:
                    v = mid_norm[j]
                    now_abs = abs(a_norm.get(j, 0.0))
                    if abs(v) > max(98.0, now_abs) + 1e-6:
                        blocked_joints.append(f"{joint}:{j}한계@구간{s}")
        return blocked_joints

    # 클램핑된 PARK로의 경로
    blocked = _stage_check(a_deg, park_clamped_deg, "PARK(클램핑 후)")
    check("A자세에서 클램핑된 PARK로 가는 경로가 막히지 않는다 (T60 핵심)",
          len(blocked) == 0,
          f"막힌 구간: {blocked}" if blocked else "전 구간 통과")

    # 5) click_server의 park가 arm_stage를 부르는지
    cs_src = open(os.path.join(ROS2, "tools", "click_server.py"), encoding="utf-8").read()
    check("조작대 park 버튼이 arm_stage.py를 호른다 (실기는 tester 몫)",
          'if job == "park":' in cs_src and "arm_stage.py" in cs_src,
          "PC 검증 완료 — 실기 확인은 tester 사이클이 park 버튼으로 한다")


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
# ⚠ 2026-09-18 감사(T50): elbow_flex=-70.0이던 옛 값은 **이 프레임의 계산으로도
# 한계 밖**이었다(norm -128, 한계 ±98 — arm._to_norm(옛 PATH_START_DEG)로 직접
# 확인됨). 사이클40 감사와 사이클41 빌더가 둘 다 "시작 elbow -70은 범위 안"이라
# 적었는데, 그건 이 계산 없이 짐작한 것이었다 — **틀렸다**. -30.0으로 낮춰
# start·target 둘 다 한계 안임을 아래에서 확인한 뒤에도 여전히 1번째 걸음에서
# 막힌다(elbow norm -127) — 그러니 "직교 직선이 유효한 두 끝점 사이에서도
# 막힌다"는 결론 자체는 살아 있다, 다만 원인은 "시작이 이미 밖"이 아니라
# **직선이 관절공간에서 볼록하지 않다**는 기하 성질이다(자세한 근거 =
# docs/인수인계-2026-09-04.md "T50" 절).
PATH_START_DEG = {"shoulder_pan": 0.0, "shoulder_lift": 55.0, "elbow_flex": -30.0,
                  "wrist_flex": 10.0, "wrist_roll": 0.0}
# 이 자리는 **끝점 둘 다 갈 수 있는데 그 사이가 막히는** 목표다(사이클20이 실기
# 5/5로 겪은 것과 같은 모양). 좌표를 바꾸면 전제가 깨지니 아래 검사가 그 전제를
# 먼저 확인한다(시작·목표 둘 다 NormLimits 안인지를 명시적으로 검사한다).
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


class _SagArm:
    """중력 처짐을 흉내 내는 **가짜 팔** + 받는 쪽의 거절 규칙(arm_source와 같다).

    두 가지를 한 자리에서 재현한다:
      · 받는 쪽은 지령을 받으면 **지금 실제 자세에서 다시 plan** 해 두 걸음 이상이면
        거절한다(`arm_source.move_joints_deg`의 계약 그대로).
      · 팔은 지령보다 `sag`만큼 못 미친 자리에 선다(2026-09-18 실측 4.0~4.9°).
    이 둘이 겹치면 **낡은 걸음**은 12+4.3=16.3°가 되어 거절된다 — 사이클42가
    실기에서 겪은 그 숫자이고, T56이 고친 병이다. 팔도 파일도 안 건드린다.
    """

    def __init__(self, start, limits, geom, sag=4.3, frozen=False):
        from tomato_picker.hardware import escape as es  # noqa: E402
        self._es = es
        self.now = {j: float(start[j]) for j in kin.JOINTS}
        self.limits, self.geom, self.sag = limits, geom, float(sag)
        self.frozen = frozen                 # True면 지령을 받아도 안 움직인다
        self.sent, self.hops, self.rejected = 0, [], []

    def measure(self) -> dict:
        return dict(self.now)

    def send(self, degs) -> None:
        want = {**self.now, **{j: float(v) for j, v in degs.items() if j in kin.JOINTS}}
        big = max(abs(want[j] - self.now[j]) for j in kin.JOINTS)
        steps = self._es.plan(self.now, want, self.limits.norms(self.now),
                              self.limits.norms, self.geom, limit=self.limits.limit)
        if len(steps) > 1:
            self.rejected.append(round(big, 1))
            raise RuntimeError(f"한 번에 {big:.1f}°는 너무 크다"
                               f"(관절 상한 {self._es.STEP_DEG:.0f}°)")
        self.sent += 1
        self.hops.append(round(big, 2))
        if self.frozen:
            return
        moved = {}
        for j in kin.JOINTS:
            d = want[j] - self.now[j]
            # 걸음이 처짐보다 크면 그만큼 못 미치고, 작으면 그대로 닿는다.
            moved[j] = want[j] - math.copysign(self.sag, d) if abs(d) > self.sag else want[j]
        self.now = moved


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

    # ⑥ 걸음이 **낡는다** — 처짐이 얹히면 두 걸음째가 거절된다 (T56, 2026-09-18)
    # 사이클42 실기: A자세에서 2/18걸음이 16.3°(상한 12°)로 거절돼 5회가 전부
    # 자세 가드에 막히고 **거짓 0/5**가 기록에 남았다. 아래 셋이 그 병과 약을
    # 같은 가짜 팔 위에서 나란히 보여 준다 — 팔도 포트도 안 건드린다.
    planned_steps = m5.prep_plan(inside, limits, geom)["steps"]
    naive = _SagArm(inside, limits, geom)
    naive_fail = None
    for st in planned_steps:                    # 예전 방식: 짜 둔 걸음을 그대로
        try:
            naive.send(st["degs"])
        except RuntimeError as exc:
            naive_fail = (st["i"], str(exc))
            break
    check("낡은 걸음을 그대로 보내면 두 걸음째가 거절된다(병의 재현)",
          naive_fail is not None and naive_fail[0] == 2 and naive.rejected
          and naive.rejected[0] > es.STEP_DEG,
          f"거절 걸음={naive_fail and naive_fail[0]} 크기={naive.rejected}° "
          f"(상한 {es.STEP_DEG:.0f}°)")

    target = m5.prep_plan(inside, limits, geom)["target"]
    arm = _SagArm(inside, limits, geom)

    def _walk(fake):
        """가짜 팔 위에서 walk를 돌린다. 거절당하면 **예외 대신 실패로** 돌려준다.

        ⚠ 여기서 예외가 그대로 올라가면 남은 검사가 통째로 안 돈다 — 되읽기를
        빼먹는 회귀(T56이 고친 바로 그 병)가 오면 반드시 FAIL로 보여야 한다.
        """
        try:
            return es.walk(target, measure=fake.measure, send=fake.send,
                           to_norm=limits.norms, geom=geom, limit=limits.limit)
        except Exception as exc:  # noqa: BLE001 - 거절 문구를 그대로 보여 준다
            return {"reached": False, "sent": fake.sent, "gap_deg": None,
                    "last": fake.measure(), "planned": 0, "notes": [],
                    "detail": f"walk가 거절당해 멈췄다: {exc}"}

    walked = _walk(arm)
    check("walk는 같은 팔에서 **한 번도 거절당하지 않고** 완주한다",
          walked["reached"] is True and arm.rejected == [],
          f"{walked['detail']} · 보낸 걸음={arm.sent} 거절={arm.rejected}")
    # 걸음을 상한에 꽉 채우지 않는 이유 — 받는 쪽은 **자기가 따로 읽은** 자세로
    # 크기를 다시 잰다. 그 읽기가 한 박자 늦으면 12.0°짜리 걸음이 12.x°로 읽혀
    # 거절되고, 그 거절은 "팔이 못 간다"로 기록에 남는다. 여유는 0.5° 이상이어야
    # 한다(여기서 상수로 못 박는다 — SEND_MARGIN_DEG를 0으로 되돌리면 실패한다).
    check("walk가 보낸 걸음은 상한에 꽉 차지 않는다(여유 ≥0.5°)",
          bool(arm.hops) and max(arm.hops) <= es.STEP_DEG - 0.5
          and es.SEND_MARGIN_DEG >= 0.5,
          f"최대 걸음 {max(arm.hops) if arm.hops else None}° "
          f"(상한 {es.STEP_DEG:.0f}° · 여유 {es.SEND_MARGIN_DEG:.1f}°)")
    r_after = kin.signed_radius(walked["last"], geom)
    check("완주한 자리는 좌표 가드를 넘는다(뻗기의 목적)",
          r_after >= es.GUARD_MM,
          f"signed_r {kin.signed_radius(inside, geom):.0f} → {r_after:.0f}mm "
          f"(가드 {es.GUARD_MM:.0f}mm)")

    # 안 움직이는 팔 — 영원히 걷지 않고 **못 갔다고 말한다**(뻗은 척 금지)
    stuck = _SagArm(inside, limits, geom, frozen=True)
    stopped = _walk(stuck)
    check("안 움직이는 팔에서는 예산 안에서 멈추고 실패로 남는다",
          stopped["reached"] is False and stuck.sent <= len(planned_steps) * 3
          and "안 줄어든다" in stopped["detail"],
          f"보낸 걸음={stuck.sent} · {stopped['detail']}")
    check("prep가 그 되읽기 루프를 실제로 쓴다(낡은 걸음 루프로 되돌아오지 않았다)",
          "es.walk(" in m5_src and 'for step in planned["steps"]:' not in m5_src,
          "move5_check.run_prep → escape.walk")

    # ⑦ **prep 실패는 0/5가 아니다** (T59, 2026-09-18) — 사이클42는 prep이
    # 실패한 채로 5회를 그대로 돌려 자세 가드 거절 5줄을 남겼고, 그 0/5가
    # 기준3의 점수처럼 읽혔다. run_real은 rclpy 노드가 있어야 끝까지 돌길래
    # (서비스 호출·spin) 여기서는 arm_extend·prep_plan처럼 소스 자체를 본다 —
    # 이 저장소가 이미 ⑤에서 하는 것과 같은 방식이다.
    check("--force로만 prep 실패에도 옛 동작(5회 강행)을 쓸 수 있다",
          '"--force"' in m5_src, "기본은 강행하지 않는다")
    check("prep 실패면 기본은 5회를 돌리지 않는다(거짓 0/5 방지)",
          "prep_failed and not force" in m5_src and "return -2" in m5_src,
          "run_real이 팔을 움직이기 전에 멈춰야 한다")
    check("멈춘 판도 이유 한 줄은 기록에 남는다(invalid=True·stage=no-prep)",
          '"invalid": True' in m5_src and '"stage": "no-prep"' in m5_src,
          "다음 사이클이 '시험이 아예 안 됐다'를 줄만 보고 알아야 한다")
    check("main()이 '시험 못 함'을 0/5와 다른 말로 알린다",
          "ok == -2" in m5_src and "시험 못 함" in m5_src,
          "N/5 점수줄과 헷갈리지 않게")


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

    # **'더 안 갑니다'는 성공이 아니다** (T55, 2026-09-18).
    # 옛 코드는 _run_path가 이 메시지를 문자열로 return해 arm_node가 ok=True로
    # 응답했다 — 실기 기록에 목표에서 111~330mm 떨어진 자리가 ok=True로 남았다.
    # ArmStuck을 올리도록 고쳤으므로: (a) arm_node의 except Exception이 잡아
    # ok=False가 되고, (b) classify_stage가 path로 분류한다.
    check("ArmStuck은 RuntimeError의 서브클래스 — arm_node의 except Exception이 잡는다",
          issubclass(cart.ArmStuck, RuntimeError))
    stuck_msg = "4걸음에서 더 안 갑니다 — 목표에서 111mm 떨어진 자리에 섰고"
    check("ArmStuck 메시지도 stage=path로 분류된다",
          m5.classify_stage(stuck_msg) == "path", m5.classify_stage(stuck_msg))

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
    # T50(2026-09-18): **시작 자세도** 같은 잣대로 확인한다 — 옛 PATH_START_DEG는
    # 목표만 검사하는 사이 자기 자신이 이미 한계 밖이었다(elbow norm -128,
    # 아무도 안 봤다). "양 끝 다 갈 수 있는데 중간이 막힌다"는 이 검사의 전제이므로,
    # 시작이 몰래 밖으로 나가면 이 시험은 조용히 다른 것(그냥 못 가는 시작점)을
    # 재는 시험이 된다.
    start_over = arm._to_norm(PATH_START_DEG)                   # noqa: SLF001
    start_bad = {j: v for j, v in start_over.items() if abs(v) > cart.NORM_LIMIT}
    check("시험용 시작 자세도 한계 안이다(그래야 '중간만 막힌다'는 전제가 산다)",
          not start_bad, str({j: round(v, 1) for j, v in start_bad.items()}))
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

    # T72: settle 단계 판정 (한계에 눌림 / 되먹임 포화 표지)
    check("STAGES에 settle이 들어 있다 (T72)", "settle" in m5.STAGES)
    check("한계에 눌림 표지가 달린 문장은 settle로 분류된다 (T72)",
          m5.classify_stage("되먹임 3회 — 오차 12.09° → 1.16° (포화(더 안 줄어듦)) · 한계에 눌림 shoulder_lift,shoulder_pan") == "settle")
    check("되먹임 포화 문장은 settle로 분류된다 (T72)",
          m5.classify_stage("되먹임 3회 — 오차 4.83° (포화(더 안 줄어듦)) · 나빠져서 되돌림") == "settle")


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

    # ③c [계측사] 스탠드오프(standoff)·접근축(approach) 기하 규약 및 3대 구현 일치 검증 (§47)
    from tomato_picker.hardware import eye as eye_hw  # noqa: E402
    from tomato_picker.config import ARM_EYE_STANDOFF_MM  # noqa: E402
    test_p = {"x": 250.0, "y": 50.0, "z": 300.0, "pitch": -20.0}
    sp_test = m5.standoff_pose(test_p, 30.0)
    ap_test = eye_hw.Eye.approach_pose(None, 250.0, 50.0, 300.0, pitch=-20.0, standoff=30.0)
    tpose_test = kin.ToolPose(x=250.0, y=50.0, z=300.0, pitch=-20.0)
    dx_t, dy_t, dz_t = kin.offset_in_tool_frame(tpose_test, -30.0, 0.0, 0.0)
    check("standoff 3개 구현(eye·move5_check·kin.offset_in_tool_frame)의 접근축 이동이 완전 일치한다",
          abs(sp_test.x - ap_test["x"]) < 1e-9 and abs(sp_test.y - ap_test["y"]) < 1e-9
          and abs(sp_test.z - ap_test["z"]) < 1e-9
          and abs(sp_test.x - (tpose_test.x + dx_t)) < 1e-9
          and abs(sp_test.y - (tpose_test.y + dy_t)) < 1e-9
          and abs(sp_test.z - (tpose_test.z + dz_t)) < 1e-9,
          f"sp=({sp_test.x:.2f},{sp_test.y:.2f},{sp_test.z:.2f}) vs ap=({ap_test['x']:.2f},{ap_test['y']:.2f},{ap_test['z']:.2f})")
    p_back = m5.target_from_stand(sp_test, 30.0)
    check("move5_check의 target_from_stand와 standoff_pose가 왕복 가역 항등 변환이다",
          abs(test_p["x"] - p_back["x"]) <= 0.1 and abs(test_p["y"] - p_back["y"]) <= 0.1
          and abs(test_p["z"] - p_back["z"]) <= 0.1,
          f"orig={test_p} -> back={p_back}")
    arm_node_code = open(os.path.join(REPO, "ros2", "src", "tomato_bridge", "tomato_bridge",
                                      "arm_node.py"), encoding="utf-8").read()
    check("arm_node.py가 standoff_m(m)을 mm로 환산(x1000)하고 reached를 m(/1000)로 반환한다",
          "float(req.standoff_m) * 1000.0" in arm_node_code
          and "Point(x=pose.x / 1000.0" in arm_node_code,
          "MoveToPoint.srv 단위(m)와 kinematics(mm) 간 단위 일관성 보장")
    check("STANDOFF_MM(30.0) 및 ARM_EYE_STANDOFF_MM(45.0)이 물리 안전 범위(20..60mm) 안이다",
          20.0 <= m5.STANDOFF_MM <= 60.0 and 20.0 <= ARM_EYE_STANDOFF_MM <= 60.0,
          f"m5={m5.STANDOFF_MM} eye={ARM_EYE_STANDOFF_MM}")
    yaml_lim_arm = _geometry_yaml()["arm"]["limits_deg"]
    check("집게(gripper) 규약 분리: yaml limits_deg [0, 45]와 kin.JOINTS 분리가 유지된다",
          yaml_lim_arm.get("gripper") == [0.0, 45.0] and "gripper" not in kin.JOINTS,
          f"gripper_lim={yaml_lim_arm.get('gripper')} in_joints={'gripper' in kin.JOINTS}")

    # ③d [계측사] 프리셋(Preset) · 캘리브레이션(Calibration) 변환 및 높이 앵커(Height Anchor) 보간 규약 검증 (§49)
    import remap_presets as rp  # noqa: E402
    from tomato_picker.hardware import presets as arm_presets  # noqa: E402

    follower_cal = json.load(open(os.path.join(REPO, "deploy", "calibration", "tomato_follower.json"), encoding="utf-8"))
    norm_invertible = True
    for j in kin.JOINTS:
        for val in [-90.0, -45.0, 0.0, 45.0, 90.0]:
            r_raw = rp._norm_to_raw(val, follower_cal[j], False)
            b_norm = rp._raw_to_norm(r_raw, follower_cal[j], False)
            if abs(b_norm - val) > 1e-6:
                norm_invertible = False
    for val in [0.0, 25.0, 50.0, 75.0, 100.0]:
        r_raw = rp._norm_to_raw(val, follower_cal["gripper"], True)
        b_norm = rp._raw_to_norm(r_raw, follower_cal["gripper"], True)
        if abs(b_norm - val) > 1e-6:
            norm_invertible = False
    check("remap_presets의 _norm_to_raw 및 _raw_to_norm이 유효 범위 내에서 왕복 가역 항등 변환이다",
          norm_invertible, "정규화(-100..100, 0..100) <-> raw tick 왕복 오차 < 1e-6")

    # ③e [계측사] 서보 하드웨어 캘리브레이션 ↔ NormLimits 기하 한계 일치성 및 물리 가동구간 검증 (§74)
    cal_dpn_matches = all(
        abs((follower_cal[j]["range_max"] - follower_cal[j]["range_min"]) * 360.0 / 4096.0 / 200.0 - f["dpn"][j]) < 0.001
        for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
    )
    check("캘리브레이션 4대 관절 틱 스팬(range_max-min)이 ESCAPE_FRAME의 dpn(도/단위)과 1e-3 이내로 일치한다",
          cal_dpn_matches,
          ", ".join(f"{j}={f['dpn'][j]}" for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")))

    elbow_range = limits.degree_range("elbow_flex")
    check("팔 하드웨어 캘리브레이션 기하: elbow_flex 물리 하한이 -38.90도(>-40도)로 제한된다 (T31 원인 규명)",
          abs(elbow_range[0] - (-38.90)) <= 0.05,
          f"elbow_range=[{elbow_range[0]:.2f}°, {elbow_range[1]:.2f}°]")

    lift_range = limits.degree_range("shoulder_lift")
    check("팔 하드웨어 캘리브레이션 기하: shoulder_lift 물리 하한이 +7.94도(>0도)로 음수 각도가 물리 불가하다",
          lift_range[0] > 0.0 and abs(lift_range[0] - 7.94) <= 0.05,
          f"lift_range=[{lift_range[0]:.2f}°, {lift_range[1]:.2f}°]")

    roll_range = limits.degree_range("wrist_roll")
    check("팔 하드웨어 캘리브레이션 기하: wrist_roll 물리 가동 스팬이 200도 미만(+-97.9도)으로 케이블 감김을 차단한다",
          abs(roll_range[0] - (-97.90)) <= 0.05 and abs(roll_range[1] - 97.90) <= 0.05 and (roll_range[1] - roll_range[0]) < 200.0,
          f"roll_range=[{roll_range[0]:.2f}°, {roll_range[1]:.2f}°]")

    # ③f so101_geometry.yaml:limits_deg vs NormLimits 물리 기하 한계 정합성 검증 (T88, §75)
    yaml_lims = _geometry_yaml()["arm"]["limits_deg"]
    elbow_gap = abs(yaml_lims["elbow_flex"][0] - elbow_range[0])
    check("so101_geometry.yaml의 elbow_flex 하한(-38.90°)이 실제 하드웨어(-38.90°)와 0.05도 이내로 일치한다 (T88)",
          elbow_gap <= 0.05,
          f"yaml={yaml_lims['elbow_flex'][0]}° vs 실제={elbow_range[0]:.2f}° (차이 {elbow_gap:.2f}°)")

    lift_gap = abs(yaml_lims["shoulder_lift"][0] - lift_range[0])
    check("so101_geometry.yaml의 shoulder_lift 하한(+7.94°)이 실제 하드웨어(+7.94°)와 0.05도 이내로 일치한다 (T88)",
          lift_gap <= 0.05 and yaml_lims["shoulder_lift"][0] > 0.0,
          f"yaml={yaml_lims['shoulder_lift'][0]}° vs 실제={lift_range[0]:.2f}° (차이 {lift_gap:.2f}°)")

    sample_pose = {"shoulder_pan.pos": 10.0, "shoulder_lift.pos": 20.0, "elbow_flex.pos": -30.0,
                   "wrist_flex.pos": 15.0, "wrist_roll.pos": 0.0, "gripper.pos": 50.0}
    converted_pose, conv_warnings = rp.convert(sample_pose, follower_cal, follower_cal)
    check("remap_presets.convert()가 동일 캘리브레이션에 대해 무손실 항등 변환을 수행한다",
          all(abs(converted_pose[k] - sample_pose[k]) < 1e-6 for k in sample_pose) and len(conv_warnings) == 0,
          f"warnings={conv_warnings}")

    sample_degs = {"shoulder_pan": 15.0, "shoulder_lift": 45.0, "elbow_flex": -20.0,
                   "wrist_flex": 10.0, "wrist_roll": -30.0}
    c_zero = {j: 0.0 for j in kin.JOINTS}
    c_ref = cart.ARM_CART_ZERO_POSE_DEG
    c_signs = {j: 1.0 for j in kin.JOINTS}
    c_dpn = {j: 1.0 for j in kin.JOINTS}
    c_norms = cart.degrees_to_norms(sample_degs, zero=c_zero, ref=c_ref, signs=c_signs, deg_per_norm=c_dpn)
    c_degs_back = cart.norms_to_degrees(c_norms, zero=c_zero, ref=c_ref, signs=c_signs, deg_per_norm=c_dpn)
    check("cartesian의 norms_to_degrees와 degrees_to_norms가 5개 관절에 대해 왕복 가역 항등 변환이다",
          all(abs(c_degs_back[j] - sample_degs[j]) < 1e-6 for j in kin.JOINTS),
          f"degs_orig={sample_degs} -> back={c_degs_back}")

    with tempfile.NamedTemporaryFile("w", delete=False, suffix=".json") as ps_tmp:
        ps_tmp_path = ps_tmp.name
    try:
        store_test = arm_presets.PresetStore(ps_tmp_path)
        store_test.save(4, {"shoulder_lift.pos": 20.0, "elbow_flex.pos": -50.0}, name="상")
        store_test.set_anchor(4, "상", 200.0)
        store_test.save(6, {"shoulder_lift.pos": 40.0, "elbow_flex.pos": -20.0}, name="하")
        store_test.set_anchor(6, "하", 400.0)
        b_mid, _ = store_test.blend(300.0)
        b_lo, _ = store_test.blend(150.0)
        b_hi, _ = store_test.blend(450.0)
        blend_ok = (abs(b_mid["shoulder_lift.pos"] - 30.0) < 1e-6 and
                    abs(b_mid["elbow_flex.pos"] - (-35.0)) < 1e-6 and
                    abs(b_lo["shoulder_lift.pos"] - 20.0) < 1e-6 and
                    abs(b_hi["shoulder_lift.pos"] - 40.0) < 1e-6)
        check("PresetStore.blend()가 앵커 구간 외삽 방지 클램프 및 정확한 선형 보간을 수행한다",
              blend_ok, f"mid={b_mid} lo={b_lo} hi={b_hi}")
    finally:
        if os.path.exists(ps_tmp_path):
            os.unlink(ps_tmp_path)

    deg_per_tick = 360.0 / 4096.0
    spans_valid = True
    for j in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]:
        span_deg = (follower_cal[j]["range_max"] - follower_cal[j]["range_min"]) * deg_per_tick
        dpn = span_deg / 200.0
        if span_deg < 180.0 or not (0.9 <= dpn <= 1.5):
            spans_valid = False
    wroll_span = (follower_cal["wrist_roll"]["range_max"] - follower_cal["wrist_roll"]["range_min"]) * deg_per_tick
    if abs(wroll_span - 360.0) >= 1.0:
        spans_valid = False
    check("tomato_follower.json 서보 6종의 틱 스팬 및 deg_per_norm이 물리적 가동범위를 만족한다",
          spans_valid, f"wroll_span={wroll_span:.1f}deg, STS3215={deg_per_tick:.4f}deg/tick")

    # ③e [빌더] 보드 계약 v2(MobileBase) 및 5층 데드맨 안전 시한 검증 (§50)
    from tomato_bridge.board_contract import (Caps, DutyCalib, AxisSigns, Command,
                                              checksum, framed, plan, to_physical)
    # 1) 프레이밍 검증: <payload>*<XOR>\n
    chk_test = checksum("C 350 0 0")
    frame_test = framed("C 350 0 0")
    check("board_contract.framed() 및 checksum()이 보드계약 §4 프레이밍 규약을 준수한다",
          chk_test == "55" and frame_test == b"C 350 0 0*55\n",
          f"chk={chk_test} frame={frame_test}")

    # 2) to_physical 단위 변환: m/s -> mm/s, rad/s -> mdeg/s (정수)
    phys = to_physical(0.35, -0.2, math.radians(45.0), AxisSigns(1, 1, 1))
    check("board_contract.to_physical()이 m/s 및 rad/s를 mm/s 및 mdeg/s 정수로 정확히 변환한다",
          phys == (350, -200, 45000), f"phys={phys}")

    # 3) units=1 & calib=0 -> nocalib 거절 (조용한 실패 차단)
    cap_nocalib = Caps.parse("cap proto=2 units=1 calib=0 vmax=800 wmax=180000")
    cmd_nocalib = plan(0.3, 0.0, 0.0, caps=cap_nocalib)
    check("board_contract.plan()이 units=1이고 calib=0일 때 nocalib으로 거절한다",
          cmd_nocalib.rejected and "cap.calib=0" in cmd_nocalib.reason,
          f"rejected={cmd_nocalib.rejected} reason={cmd_nocalib.reason}")

    # 4) estop=True -> S 명령 및 거절 (비상정지 래치)
    cmd_estop = plan(0.3, 0.0, 0.0, estop=True)
    check("board_contract.plan()이 estop=True일 때 S 페이로드 및 거절을 반환한다",
          cmd_estop.rejected and cmd_estop.payload == "S" and "비상정지" in cmd_estop.reason,
          f"payload={cmd_estop.payload} reason={cmd_estop.reason}")

    # 5) units=1 & calib=1 -> 물리 C 지령 발행
    cap_v2 = Caps.parse("cap proto=2 units=1 calib=1 vmax=800 vymax=600 wmax=180000")
    cmd_v2 = plan(0.35, 0.0, math.radians(30.0), caps=cap_v2)
    check("board_contract.plan()이 정상 v2 보드에 대해 C 물리 지령 문자열을 생성한다",
          not cmd_v2.rejected and cmd_v2.payload == "C 350 0 30000" and cmd_v2.physical == (350, 0, 30000),
          f"cmd={cmd_v2}")

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
          bool(ld.LoadLimits(ARM_LOAD_R_MAX, source="테스트").rejects(331.0, 0.0)),
          f"한계 {ARM_LOAD_R_MAX}mm")
    check("그러면서 든 자리(r=308)는 통과시킨다",
          not ld.LoadLimits(ARM_LOAD_R_MAX, source="테스트").rejects(308.0, 0.0))

    # 모르면 **모른다고 말한다** — 깨진 파일을 보고 조용히 config로 돌아가면,
    # 넓히려고 쓴 파일이 무시된 줄 모른 채 옛 경계로 시험하게 된다.
    tmp = os.path.join(REPO, "ros2", "tools", "__t41_load_tmp.json")
    try:
        for body, why in (('{"r_max_mm": null}', "null"),
                          ('{"r_max_mm": 0}', "0 이하"),
                          ('{"z_max_mm": null}', "z_max null"),
                          ('{"z_max_mm": -10}', "z_max 0 이하"),
                          ('{"wflex_max_deg": null}', "wflex null"),
                          ('{"wflex_max_deg": "문자"}', "wflex 문자열"),
                          ('{"note": "재는 중"}', "키가 없다"),
                          ("{깨진 json", "깨진 파일")):
            open(tmp, "w", encoding="utf-8").write(body)
            got, note = ld.load_load_limits(path=tmp)
            check(f"경계를 못 읽으면 None이고 이유를 말한다 ({why})",
                  got is None and len(note) > 0, note)
        open(tmp, "w", encoding="utf-8").write('{"r_max_mm": 321.5, "z_max_mm": 430.0, "wflex_max_deg": -5.0}')
        got, note = ld.load_load_limits(path=tmp)
        check("파일이 config를 이긴다 (이 팔에서 다시 잰 값)",
              got is not None and abs(got.r_max - 321.5) < 1e-9 and abs(got.z_max - 430.0) < 1e-9 and abs(got.wflex_max_deg - (-5.0)) < 1e-9 and got.source == tmp,
              note)
    finally:
        os.path.exists(tmp) and os.remove(tmp)

    # T69 (T62 후속): z_max 검증 및 역사적 표적(c35 표적2 통과, c45 표적5 거절) 검증
    # z_max가 rejects()에 반영된다
    custom_limits = ld.LoadLimits(r_max=400.0, z_max=400.0, source="z_max테스트")
    check("z_max가 rejects()에 반영된다",
          bool(custom_limits.rejects(100.0, 100.0, z=450.0)) and not custom_limits.rejects(100.0, 100.0, z=350.0),
          "z가 z_max 초과 시 거절되고 이하 시 통과해야 한다")

    # T73: wflex_max_deg 검증 (T68 실측: wflex<=0.0 통과, wflex=+20.0 거절)
    wflex_limits = ld.LoadLimits(r_max=400.0, z_max=500.0, wflex_max_deg=0.0, source="wflex테스트")
    check("wflex_max_deg가 rejects()에 반영된다 (T73)",
          bool(wflex_limits.rejects(100.0, 100.0, z=400.0, wflex_deg=20.0))
          and not wflex_limits.rejects(100.0, 100.0, z=400.0, wflex_deg=-10.0),
          "wflex > wflex_max_deg 초과 시 거절되고 이하 시 통과해야 한다")

    # 사이클35 표적2(348.9, 261.6): stand r=348.9mm, z=261.6mm
    # 사이클45 표적5(215.2, 456.4): stand r=215.2mm, z=456.4mm
    p_c35_2 = {"x": 378.7, "y": 7.5, "z": 264.0, "pitch": 4.5}
    p_c45_5 = {"x": 219.1, "y": -75.4, "z": 481.4, "pitch": 56.6}
    stand_c35_2 = m5.standoff_pose(p_c35_2)
    stand_c45_5 = m5.standoff_pose(p_c45_5)
    load_filt = ld.LoadLimits(r_max=400.0, z_max=445.0, source="실측검증")
    check("사이클35 표적2(348.9,261.6)는 새 필터를 통과한다",
          not load_filt.rejects(stand_c35_2.x, stand_c35_2.y, stand_c35_2.z),
          f"stand r={math.hypot(stand_c35_2.x, stand_c35_2.y):.1f} z={stand_c35_2.z:.1f}")
    check("사이클45 표적5(215.2,456.4)는 걸러진다",
          bool(load_filt.rejects(stand_c45_5.x, stand_c45_5.y, stand_c45_5.z)),
          f"stand r={math.hypot(stand_c45_5.x, stand_c45_5.y):.1f} z={stand_c45_5.z:.1f}")

    # T73: T68 실측 자세 비교 (b) z=420,wflex=-10° 통과 vs (d) z=420,wflex=+20° 거절
    load_t68 = ld.LoadLimits(r_max=400.0, z_max=445.0, wflex_max_deg=0.0, source="T68검증")
    check("T68 자세(b) wflex=-10도는 통과한다 (T73)",
          not load_t68.rejects(243.0, 0.0, z=420.1, wflex_deg=-10.0))
    check("T68 자세(d) wflex=+20도는 걸러진다 (T73)",
          bool(load_t68.rejects(250.7, 0.0, z=420.1, wflex_deg=20.0)))

    # 계측 검증 (load_limits z_max 기준계 규약)
    ld_src = open(os.path.join(REPO, "src", "tomato_picker", "hardware", "load_limits.py"),
                  encoding="utf-8").read()
    check("load_limits의 z_max 규약이 arm_base 마운트 평면 기준이다 (base_link 오독 방지)",
          "arm_base(마운트 평면) 기준" in ld_src and "base_link 기준" not in ld_src,
          "kin.forward() 출력을 직접 검사하므로 arm_base 기준이어야 한다")
    z_max_reach = geom.z0 + geom.l1 + geom.l2 + geom.l3
    check("ARM_LOAD_Z_MAX가 마운트 높이 z0(119.5mm)보다 높고 수직 사거리 이내다",
          geom.z0 < ld.ARM_LOAD_Z_MAX < z_max_reach,
          f"z0={geom.z0} < z_max={ld.ARM_LOAD_Z_MAX} < reach={z_max_reach}")
    check("ARM_LOAD_R_MAX가 가드 최소거리(ARM_CART_R_MIN)보다 크고 최대 수평 사거리(geom.reach_max) 이내다",
          ARM_CART_R_MIN < ld.ARM_LOAD_R_MAX <= geom.reach_max,
          f"min={ARM_CART_R_MIN} < r_max={ld.ARM_LOAD_R_MAX} <= reach_max={geom.reach_max}")
    yaml_wflex = _geometry_yaml()["arm"]["limits_deg"]["wrist_flex"]
    check("ARM_LOAD_WFLEX_MAX_DEG가 so101_geometry.yaml의 wrist_flex 관절 범위 내에 있다",
          yaml_wflex[0] <= ld.ARM_LOAD_WFLEX_MAX_DEG <= yaml_wflex[1],
          f"wflex_range={yaml_wflex} 안 wflex_max={ld.ARM_LOAD_WFLEX_MAX_DEG}")
    check("LoadLimits가 z_max/r_max 등에 비숫자 주입 시 TypeError를 발생시킨다 (타입 가드)",
          _raises(lambda: ld.LoadLimits(310.0, "문자열주입")),
          "위치 인자 오지정으로 z_max에 source 문자열이 주입되는 버그 방어")

    # 기록에 남는가 — 남지 않으면 그 0/5가 팔의 0인지 도구의 0인지 못 가린다.
    m5_body = m5_source()
    check("기록 줄에 load_limits가 들어간다",
          m5_body.count('"load_limits": load_tag') >= 2,
          "dry-run과 실기 둘 다 남겨야 한다")

    # T71/T74: arm_load_limits.json 없이 실행되면 졸업 불가 경고 및 graduation_blocked 기록
    check("arm_load_limits.json 부재 시 졸업 불가 경고 문구가 move5_check에 존재한다",
          "경고: arm_load_limits.json 없음 — 이 판은 기준3 졸업 인정 불가" in m5_body,
          "T71 기준3 무효화 경고 문구가 있어야 한다")
    check("기록 줄에 graduation_blocked가 들어간다",
          m5_body.count('"graduation_blocked": graduation_blocked') >= 2,
          "dry-run과 실기 둘 다 graduation_blocked를 남겨야 한다")
    check("is_graduation_blocked() 함수가 존재한다 (T74)",
          hasattr(m5, "is_graduation_blocked"), "move5_check에 함수가 정의되어야 한다")
    check("load가 None이면 is_graduation_blocked는 True (T74)",
          m5.is_graduation_blocked(None, "none") is True)
    check("출처가 config면 is_graduation_blocked는 True (T74)",
          m5.is_graduation_blocked(ld.LoadLimits(310.0, source="config.ARM_LOAD_R_MAX"), "config.ARM_LOAD_R_MAX") is True)
    check("출처가 파일(~arm_load_limits.json)이면 is_graduation_blocked는 False (T74)",
          m5.is_graduation_blocked(ld.LoadLimits(310.0, source="/home/server/arm_load_limits.json"), "/home/server/arm_load_limits.json") is False)

    # ⑤b-2 기준3 합격선 강제 (15mm 도달 오차 상한 및 5/5 전회차 성공 강제)
    check("move5_check에 SUCCESS_ERR_MAX_MM(15.0mm) 상수가 정의되어 있다",
          hasattr(m5, "SUCCESS_ERR_MAX_MM") and abs(m5.SUCCESS_ERR_MAX_MM - 15.0) < 1e-9,
          f"SUCCESS_ERR_MAX_MM={getattr(m5, 'SUCCESS_ERR_MAX_MM', None)}")
    check("move5_check가 err <= SUCCESS_ERR_MAX_MM으로 합격을 판정한다",
          "err <= SUCCESS_ERR_MAX_MM" in m5_body,
          "하드코딩이나 임의 문턱이 아닌 15mm 규격 상수를 써야 한다")
    check("move5_check는 전회차 성공 시에만 0을 반환한다 (5/5 강제)",
          "return 0 if ok == len(points) else 1" in m5_body,
          "4/5 등 부분 성공은 종료코드 1이어야 한다")

    # ⑤c **뽑는 기하와 재는 기하가 같은 함수에서 오는가**(T53, 2026-09-18).
    #     T37은 재는 쪽(run_real)만 `_arm_node_geometry()`로 옮겼고 `main()`은
    #     `kin.ArmGeometry()` 기본값으로 표적을 뽑고 있었다. `~/arm_cartesian.json`이
    #     geometry를 채우는 날(링크를 다시 재면 그렇게 된다) **표적을 뽑은 팔과
    #     성공을 재는 팔이 다른 길이**가 되고, 기준3의 점수 자체가 못 믿을 것이 된다.
    #     말없이 갈라지는 종류라 소스와 동작 양쪽에서 못 박는다.
    geom_fn, geom_tag = m5._arm_node_geometry()
    check("_arm_node_geometry()가 기하와 출처를 함께 준다",
          isinstance(geom_fn, kin.ArmGeometry) and isinstance(geom_tag, str)
          and len(geom_tag) > 0, f"{geom_tag}")

    # 파일이 다른 길이를 말하면 **그 길이가 그대로 쓰여야** 한다. FrameConfig는
    # 경로가 기본인자로 굳어 있어(정의 시점 평가) 가짜 설정을 끼워 넣어 본다.
    class _FakeCfg:
        path = "/가짜/arm_cartesian.json"

        def geometry(self):
            return kin.ArmGeometry(l2=200.0)

    real_fc = m5.cart.FrameConfig
    try:
        m5.cart.FrameConfig = lambda *a, **k: _FakeCfg()
        got, tag = m5._arm_node_geometry()
        check("파일의 링크 길이가 코드 기본값을 이긴다",
              abs(got.l2 - 200.0) < 1e-9 and tag.startswith("file:"), f"{tag} l2={got.l2}")
        # 뽑는 쪽이 정말 그 길이를 쓰는지 — 사거리가 달라지면 표적도 달라진다.
        near = m5.sample_points(5, kin.ArmGeometry(), 0)
        far = m5.sample_points(5, got, 0)
        check("기하가 달라지면 뽑히는 표적도 달라진다",
              any(abs(a["x"] - b["x"]) + abs(a["z"] - b["z"]) > 1e-6
                  for a, b in zip(near, far)),
              "같으면 이 검사가 아무것도 안 지키고 있는 것이다")
    finally:
        m5.cart.FrameConfig = real_fc

    # 파일이 없거나 geometry 칸이 비면 arm_node와 **같은 폴백**이다.
    class _EmptyCfg(_FakeCfg):
        def geometry(self):
            return kin.ArmGeometry()

    try:
        m5.cart.FrameConfig = lambda *a, **k: _EmptyCfg()
        got, tag = m5._arm_node_geometry()
        check("geometry 칸이 비면 config 기본값이라고 말한다",
              got == kin.ArmGeometry() and tag == "config.ARM_GEOM_*", tag)
        m5.cart.FrameConfig = lambda *a, **k: (_ for _ in ()).throw(OSError("없다"))
        got, tag = m5._arm_node_geometry()
        check("파일을 못 읽어도 arm_node와 같은 기본값으로 떨어진다",
              got == kin.ArmGeometry() and tag == "config.ARM_GEOM_*", tag)
    finally:
        m5.cart.FrameConfig = real_fc

    # 소스에서도 막는다 — 기하를 만드는 자리가 둘이면 언젠가 또 갈린다.
    body_m5 = m5_source()
    head, _, rest = body_m5.partition("def _arm_node_geometry(")
    _, _, tail = rest.partition("def main(")
    check("main()이 _arm_node_geometry()로 표적을 뽑는다",
          "geom, geom_tag = _arm_node_geometry()" in tail
          and "geom = kin.ArmGeometry()" not in tail,
          "뽑는 쪽이 기본값을 따로 만들면 재는 쪽과 갈린다")
    check("기하를 만드는 자리는 _arm_node_geometry() 하나뿐이다",
          "kin.ArmGeometry()" not in head and "kin.ArmGeometry()" not in tail,
          "폴백 규칙이 두 군데면 한쪽만 고치고 끝난다")
    check("기록 줄에 geometry 출처가 들어간다",
          body_m5.count('"geometry": geom_tag') >= 2,
          "dry-run과 실기 둘 다 남겨야 한다 — 어떤 팔 길이로 쟀는지가 점수의 전제다")

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
    # ⚠ T66(2026-09-18): close()도 hold_torque=True로 연 것과 원칙이 같아야 한다.
    #   close()가 disable_torque() 경로(follower_io.close())로 가면 팔이 주저앉는다.
    #   hold=True(기본) → hold_close() / hold=False → close() 두 길을 명시적으로
    #   가진다 — 계약이 코드에 있어야 다음 사이클이 고치지 않는다.
    check("DirectArm.close(hold=True)가 hold_close()로 간다 (T66)",
          "hold_close()" in body and "if hold:" in body,
          "토크를 켠 채로 닫아야 열 때 hold_torque=True와 원칙이 같다")
    check("DirectArm.close(hold=False)가 self._io.close()로 간다 (T66)",
          "else:" in body and "self._io.close()" in body,
          "손으로 움직이거나 완전 종료 때만 토크를 끈다")
    check("close()의 hold 기본값이 True다 (T66)",
          "def close(self, hold: bool = True)" in body,
          "기본이 False면 hold_torque=True로 연 것과 원칙이 어긋난다")


def test_joint_record() -> None:
    """기록 줄에 지령·실제 관절이 남는가 (T64, 2026-09-18).

    사이클50 플래너가 성공/실패를 가르려고 기록의 TCP를 `kin.inverse`로
    역산해야 했다 — 역산은 유일해가 아니다(elbow_up 두 해 중 하나를 짐작으로
    골랐다). 관절을 뽑는 시점에 그대로 적어 두면 다음 사이클이 역산할 필요가
    없다. `--dry-run`은 서보가 없으니 지령=실제(이상적 실행)이고, 실기는
    `/joint_states`를 못 읽으면 null을 남긴다(load_limits.py와 같은 규칙 —
    짐작으로 메우지 않는다).
    """
    print("\n[관절기록] joints_cmd/joints_actual이 기록 줄에 남는가")
    sys.path.insert(0, os.path.join(REPO, "ros2", "tools"))
    import move5_check as m5  # noqa: E402

    geom = kin.ArmGeometry()

    # ① 도달 가능한 표적 — dry-run에서 joints_cmd/joints_actual이 둘 다 채워지고
    #    같은 값이다(서보가 없으니 지령이 곧 실행).
    pts = m5.sample_points(3, geom, 0)
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, "rec.jsonl")
        m5.run_dry(pts, geom, out)
        rows = [json.loads(line) for line in open(out, encoding="utf-8")]
    check("표적 개수만큼 줄이 남는다", len(rows) == len(pts), f"{len(rows)}줄")
    for row in rows:
        check("joints_cmd에 5관절이 다 있다",
              row.get("joints_cmd") is not None
              and set(row["joints_cmd"]) == set(kin.JOINTS),
              f"trial {row.get('trial')}: {row.get('joints_cmd')}")
        check("dry-run은 joints_actual == joints_cmd다 (서보가 없다)",
              row.get("joints_actual") == row.get("joints_cmd"),
              f"trial {row.get('trial')}: cmd={row.get('joints_cmd')} "
              f"actual={row.get('joints_actual')}")

    # ② IK가 안 풀리는 표적은 joints_cmd가 None이다 — 짐작으로 메우지 않는다.
    unreachable = {"x": 9999.0, "y": 0.0, "z": 300.0, "pitch": 0.0}
    check("사거리 밖 표적은 commanded_joints가 None을 준다",
          m5.commanded_joints(unreachable, geom) is None)
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, "rec.jsonl")
        m5.run_dry([unreachable], geom, out)
        row = json.loads(open(out, encoding="utf-8").readline())
    check("IK 실패 줄은 joints_cmd/joints_actual이 둘 다 null이다",
          row.get("joints_cmd") is None and row.get("joints_actual") is None, row)

    # ③ 소스에서도 막는다 — run_real(실기)이 두 칸을 채우는 자리가 있는가.
    #    ROS 없이는 run_real을 못 돌리므로(rclpy import) 텍스트로 지킨다.
    body = open(os.path.join(REPO, "ros2", "tools", "move5_check.py"),
               encoding="utf-8").read()
    check("run_real이 joints_cmd를 commanded_joints()로 채운다",
          "joints_cmd = commanded_joints(p, geom)" in body)
    check("실기 성공 줄은 되읽은 joints_actual을 남긴다",
          "joints_actual=joints_actual" in body)
    # 응답없음·거절·/joint_states 없음 — 셋 다 관절을 모른다, null이라고 말해야 한다.
    check("응답이 없거나 못 읽으면 joints_actual을 null로 남긴다",
          body.count("joints_actual=None") >= 3,
          "timeout/거절//joint_states 없음 세 갈래 모두 null이어야 한다")


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


# 검사 대상이 아닌 곳 — 가상환경·빌드산출물, 3D(바이너리), 그리고 호스트 러너(autopilot, 비배포).
EOL_SKIP_DIRS = {".git", ".venv", "__pycache__", ".work", "node_modules",
                 "build", "install", "log", "3D", "autopilot"}


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


def test_record_destination() -> None:
    """실기 기록이 **저장소로 돌아갈 길이 있는가** — 없으면 조용히 사라진다.

    이 검사가 있는 이유 (2026-09-18, T58): 사이클42의 실기 5회가 jsonl에 한 줄도
    안 남고 산문에만 남았다. 원인은 "컨테이너가 기록을 못 쓴다"가 아니라
    **젯슨 트리에 쓰고 저장소로 안 돌아온다**는 것이다 — 젯슨의
    `~/tomato-picker/`는 archive+scp로 배포된 부분복사본이라 `.git`이 없다
    (실측: `git rev-parse` → "not a git repository"). 커밋될 길이 없는 자리다.

    ⚠ 그리고 그냥 덮어쓰면 한쪽을 잃는다 — 같은 이름의 파일이 두 곳에서 따로
    자란다(2026-09-18 실측: 저장소 57줄 / 젯슨 20줄, 겹치는 것은 사람이 손으로
    옮긴 5줄뿐). 그래서 가져오는 도구는 **붙이기만** 한다.
    """
    print("\n[기록] 실기 기록이 저장소로 돌아갈 길이 있는가")
    sys.path.insert(0, os.path.join(ROS2, "tools"))
    import move5_check as m5  # noqa: E402
    import record_pull as rp  # noqa: E402

    tmp = tempfile.mkdtemp(prefix="rec-dest-")
    try:
        # 저장소 안 = 커밋된다
        kind, why = m5.record_home(os.path.join(REPO, "docs", "시험기록"))
        check("저장소 안의 기록 자리는 repo로 읽힌다", kind == "repo", f"{kind} {why}")
        # git 없는 트리 = 젯슨 트리의 성질. 여기서 갈리지 않으면 검사가 무의미하다.
        far = os.path.join(tmp, "tomato-picker", "docs", "시험기록")
        kind, why = m5.record_home(far)
        check("git 없는 트리(=젯슨)는 volatile로 읽힌다", kind == "volatile", f"{kind} {why}")
        # 못 쓰는 자리 = 기록이 진짜로 사라지는 유일한 경우
        blocker = os.path.join(tmp, "blocker")
        open(blocker, "w").write("x")  # 파일이라 그 아래로 못 판다
        kind, why = m5.record_home(os.path.join(blocker, "시험기록"))
        check("쓸 수 없는 자리는 unwritable로 읽힌다", kind == "unwritable", f"{kind} {why}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    body = open(os.path.join(ROS2, "tools", "move5_check.py"), encoding="utf-8").read()
    # 조용히 넘어가지 않는다 — 셋 다 출력에 나타나야 한다.
    check("못 쓰는 기록 자리면 팔을 움직이기 전에 멈춘다(rc≠0)",
          'if RECORD_HOME == "unwritable":' in body and "return 2" in body,
          "기록 없는 5회는 다음 사이클에 아무것도 아니다")
    check("volatile이면 끝에 가져오는 명령을 찍는다",
          "record_pull.py --date" in body,
          "scp가 아니다 — 덮어쓰면 한쪽을 잃는다")
    check("기록 경로를 찍는 줄이 어디에 떨어지는지도 말한다",
          "[{RECORD_HOME}]" in body)
    check("기록 줄마다 record_home이 붙는다",
          'row.setdefault("record_home", RECORD_HOME)' in body,
          "손으로 옮겨 온 줄인지 줄만 보고 알아야 한다")
    check("기록 줄마다 시각이 붙는다",
          'row.setdefault("t", ' in body,
          "시각이 없으면 같은 표적의 두 판을 구분할 길이 없다")

    # ⚠ 사람이 나중에 덧붙인 칸(cycle 따위)이 신원을 바꾸면 안 된다 — 2026-09-18
    #   실측으로 사이클35의 10줄이 저장소에만 `cycle: 35`를 달고 있었고, 글자
    #   해시로 보면 10줄이 통째로 겹쳐 붙었다(실제로 그렇게 나왔다).
    same = '{"trial": 1, "ok": true, "detail": "x"}'
    annotated = '{"trial": 1, "ok": true, "detail": "x", "cycle": 35}'
    check("사람이 덧붙인 cycle 칸은 같은 시험을 다른 줄로 만들지 않는다",
          rp.merge_lines([annotated], [same])[0] == [], "안 그러면 10줄이 겹쳐 붙는다")
    check("거절 문장이 다르면 다른 판으로 본다",
          len(rp.merge_lines([same], ['{"trial": 1, "ok": true, "detail": "y"}'])[0]) == 1,
          "같은 표적을 두 판 돌린 것을 한 판으로 접으면 기록이 준다")

    # 가져오는 도구는 **붙이기만** 한다 — 로컬 줄을 하나도 잃지 않는다.
    local = ['{"trial": 1, "ok": true}', '{"trial": 2, "ok": false}']
    remote = ['{"trial": 2, "ok": false}', '{"trial": 9, "ok": true}',
              '{"trial": 9, "ok": true}', "  "]
    fresh, dup = rp.merge_lines(local, remote)
    check("record_pull은 저장소에 없는 줄만 골라낸다",
          fresh == ['{"trial": 9, "ok": true}'], f"fresh={fresh}")
    check("이미 있는 줄은 다시 안 붙인다(원격 안의 중복도 한 번)",
          dup == ['{"trial": 2, "ok": false}', '{"trial": 9, "ok": true}'], f"dup={dup}")
    # 합친 결과가 **양쪽을 다 담는가** — 이것이 "덮어쓰면 한쪽을 잃는다"의 반대말이다.
    merged = local + fresh
    lost = [ln for ln in local + [r for r in remote if r.strip()] if ln not in merged]
    check("합친 결과가 저장소 줄과 젯슨 줄을 둘 다 담는다", not lost, f"잃은 줄={lost}")
    pull_body = open(os.path.join(ROS2, "tools", "record_pull.py"), encoding="utf-8").read()
    check("record_pull이 로컬 파일을 append로만 연다",
          'open(local_path, "a"' in pull_body and 'open(local_path, "w"' not in pull_body)

    # ── 연습(--dry-run)은 진짜 기록을 건드리지 않는다 (T43) ──────────────
    # 이 검사가 있는 이유: 사이클35의 빌더가 도구를 고친 뒤 스모크 테스트로
    # `--dry-run --n 2`를 한 번 돌렸더니 그날의 진짜 시험기록에 연습 줄 2개가
    # 섞였다. 기록은 다음 사이클이 사실로 믿는 물건이라 **연습과 기록은 같은
    # 파일이면 안 된다.** 갈림 자체(record_dir_for)와, 실제로 돌려서 진짜
    # 파일의 바이트가 그대로인지를 둘 다 본다 — 갈림만 보면 나중에 부르는 쪽이
    # 바뀌었을 때 조용히 돌아온다.
    dry_dir, _ = m5.record_dir_for(dry_run=True, record=False)
    real_dir, _ = m5.record_dir_for(dry_run=False, record=False)
    check("연습의 기본 기록 자리가 진짜 기록과 다르다", dry_dir != real_dir,
          f"연습={dry_dir} / 진짜={real_dir}")
    check("실기(run_real)의 기본 기록 자리는 그대로다", real_dir == m5.RECORD_DIR,
          "실기 판정은 늘 진짜 기록이다")
    check("--record를 주면 연습도 진짜 기록 자리로 간다",
          m5.record_dir_for(dry_run=True, record=True)[0] == m5.RECORD_DIR,
          "진짜 기록에 남기려면 말해야 한다")

    today = time.strftime("%Y-%m-%d")
    real_path = os.path.join(m5.RECORD_DIR, f"move-to-point-{today}.jsonl")
    before = open(real_path, "rb").read() if os.path.exists(real_path) else None
    tmp = tempfile.mkdtemp(prefix="dry-rec-")
    try:
        env = dict(os.environ)
        env.pop("TOMATO_RECORD_DIR", None)
        # 연습 자리는 임시 디렉터리에서 온다 — 여기를 옮기면 연습 줄이
        # 어디로 떨어지는지까지 같이 확인된다.
        for key in ("TMPDIR", "TMP", "TEMP"):
            env[key] = tmp
        proc = subprocess.run(
            [sys.executable, os.path.join(ROS2, "tools", "move5_check.py"),
             "--dry-run", "--n", "2"],
            cwd=REPO, env=env, capture_output=True, text=True, encoding="utf-8")
        after = open(real_path, "rb").read() if os.path.exists(real_path) else None
        check("--dry-run은 진짜 기록 파일을 건드리지 않는다",
              proc.returncode == 0 and after == before,
              f"rc={proc.returncode} {'바이트 그대로' if after == before else '바뀌었다'}")
        practice = os.path.join(tmp, "tomato-move5-dry",
                                f"move-to-point-{today}.jsonl")
        rows = [json.loads(ln) for ln in
                open(practice, encoding="utf-8").read().splitlines() if ln.strip()]             if os.path.exists(practice) else []
        check("연습 줄은 연습 자리에 떨어진다", len(rows) == 2, f"{practice} {len(rows)}줄")
        check("연습 줄은 record_home=practice로 표시된다",
              bool(rows) and all(r.get("record_home") == "practice" for r in rows),
              "젯슨의 volatile과 뜻이 다르다 — 회수할 것이 없다")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


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

    # T77/배치 검증: docker-compose가 카메라 및 시리얼 통신에 필요한 cgroup/마운트를 선언했는가
    dc_text = open(os.path.join(REPO, "ros2", "docker", "docker-compose.yml"),
                   encoding="utf-8").read()
    check("docker-compose가 video4linux와 USB major 규칙을 허용한다 (T77 장치 연결 요건)",
          "c 81:* rmw" in dc_text and "c 189:* rmw" in dc_text,
          "D405 스트림 및 USB 컨트롤 cgroup 권한")
    check("docker-compose에 /dev/serial 마운트가 포함되어 있다 (팔 포트 식별)",
          "/dev/serial:/dev/serial:ro" in dc_text)


# ----------------------------------------------------------------------
# ⑳ 붙는 순간 팔을 놓지 않는가 (연결)
# ----------------------------------------------------------------------

# 이 파일들의 `_connect`가 같은 순서를 쓰는가. (경로, 함수, 홀드 가드)
HOLD_CONNECT_SITES = [
    ("src/tomato_picker/hardware/arm.py", "_connect", "ARM_HOLD_ON_CONNECT"),
    ("ros2/src/tomato_bridge/tomato_bridge/follower_io.py", "_connect", "self._hold_torque"),
]
# 반드시 이 차례로 나와야 하는 호출. 하나라도 빠지거나 순서가 바뀌면 팔이 떨어진다.
HOLD_ORDER = ["connect", "Present_Position", "configure", "enable_torque", "Goal_Position"]


def _func_src(path: str, name: str) -> str:
    """파일에서 함수 하나의 원문을 뽑는다(AST의 줄번호로 자른다)."""
    body = io.open(os.path.join(REPO, path), encoding="utf-8").read()
    tree = ast.parse(body)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            lines = body.splitlines()
            return "\n".join(lines[node.lineno - 1:node.end_lineno])
    return ""


def _hold_branch(src: str, guard: str) -> str:
    """`if <guard>:` 가지의 본문만 — 힘 빼는 옛 가지(else)가 섞이면 검사가 거짓말한다."""
    tree = ast.parse(textwrap.dedent(src))
    for node in ast.walk(tree):
        if isinstance(node, ast.If) and ast.unparse(node.test).replace(" ", "") == guard.replace(" ", ""):
            return "\n".join(ast.unparse(st) for st in node.body)
        # follower_io는 `if not self._hold_torque:` 로 뒤집어 적었다 — else가 홀드 가지다.
        if isinstance(node, ast.If) and ast.unparse(node.test).replace(" ", "") == f"not{guard.replace(' ', '')}":
            return "\n".join(ast.unparse(st) for st in node.orelse)
    return ""


def _code_only(src: str) -> str:
    """주석을 떼고 코드만 남긴다 — ⚠ 이 검사가 **자기 주석에 속았다**: 두 파일의
    `_reconnect`가 "`old.disconnect()`가 아니다"라고 적어 둔 설명 줄을 선언으로
    세어 둘 다 FAIL이 났다. `_unit_directive`가 systemd 유닛에서 `#`을 떼는 것과
    같은 이유다 — 설명에 적어 둔 이름이 호출로 세어지면 검사가 거짓말을 한다."""
    return ast.unparse(ast.parse(textwrap.dedent(src)))


def _order_ok(text: str, want: list[str]) -> bool:
    at = -1
    for token in want:
        found = text.find(token, at + 1)
        if found < 0:
            return False
        at = found
    return True


def test_hold_on_connect() -> None:
    """팔에 **붙는 것만으로** 떨어뜨리지 않는가.

    이 검사가 있는 이유: 2026-09-18, 팔에 지령을 하나도 주지 않고
    `systemctl start tomato-voice` ↔ `systemctl start click-server`를 오갔더니
    shoulder_lift가 **77.0°·77.5°**(두 자세에서 재현) 떨어지고 TCP z가
    433.7mm → **-32.3mm**(바닥 아래)로 내려앉았다. 화분이 그 자리에 있었으면
    부딪혔다. 원인은 레거시 `arm.py._connect`가 붙자마자 `disable_torque()`로
    **아예 놓은 것**이고, 되켜지는 것은 다음 이동 때뿐이었다. click-server를
    다시 켜도 회복되지 않는다 — 다음 도구가 떨어진 자리를 붙들기 때문이다(래칫).
    실측표 = docs/인수인계-2026-09-04.md §26.

    고친 규칙은 두 계통이 **같아야 한다**: 한쪽만 고치면 그 서비스로 전환하는
    순간 같은 낙하가 돌아온다. 그래서 순서를 여기서 못 박는다 —
    버스 → 지금 자세 읽기 → configure(여기서 처진다) → 토크 → 읽어 둔 자세로 복귀.
    ⚠ 이 검사가 못 보는 것: 젯슨에서 실제로 몇 도 처지는가. 그건 실기로만 안다.
    """
    print("\n[연결] 붙는 것만으로 팔을 놓지 않는가")
    for path, func, guard in HOLD_CONNECT_SITES:
        src = _func_src(path, func)
        check(f"{os.path.basename(path)}에 {func}가 있다", bool(src), path)
        if not src:
            continue
        branch = _hold_branch(src, guard)
        check(f"{os.path.basename(path)}가 {guard}로 홀드 가지를 가른다",
              bool(branch), "가지를 못 찾았다")
        check(f"{os.path.basename(path)} 홀드 가지가 읽기→configure→토크→복귀 순서다",
              _order_ok(branch, HOLD_ORDER), f"필요: {' → '.join(HOLD_ORDER)}")
        # 홀드 가지 안에서 힘을 빼면 위 순서를 지켜도 팔은 떨어진다.
        check(f"{os.path.basename(path)} 홀드 가지가 disable_torque를 부르지 않는다",
              "disable_torque" not in branch, "붙자마자 놓는 그 호출이다")
        # 재연결도 같다 — 통신이 끊긴 것과 팔을 놓아도 되는 것은 별개다.
        recon = _code_only(_func_src(path, "_reconnect"))
        check(f"{os.path.basename(path)}의 _reconnect가 토크를 켠 채 닫는다",
              "disable_torque=False" in recon and ".disconnect()" not in recon,
              "lerobot의 disable_torque_on_disconnect 기본값이 True다")

    # 기본값이 False로 뒤집히면 위 가지가 통째로 안 돌고 병이 조용히 돌아온다.
    sys.path.insert(0, REPO)
    from src.tomato_picker.config import ARM_HOLD_ON_CONNECT  # noqa: PLC0415
    check("ARM_HOLD_ON_CONNECT 기본이 True다", ARM_HOLD_ON_CONNECT is True,
          f"지금 {ARM_HOLD_ON_CONNECT}")


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
    test_arm_stage_park()
    test_prep_autoextend()
    test_stage_classify()
    test_travel_split()
    test_sample_within_limits()
    test_joint_record()
    test_mount_contract()
    test_record_timezone()
    test_record_destination()
    test_line_endings()
    test_service_exclusivity()
    test_hold_on_connect()
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
