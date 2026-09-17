#!/usr/bin/env python3
"""마운트 대조 — base_link→arm_base TF가 so101_geometry.yaml과 같은가.

**왜 별도 파일인가.** 이 계산을 쓰는 곳(`tf_check.py`)은 `rclpy`를 import하므로
개발 PC에서 못 돈다. 그런데 "검사가 실제로 실패할 수 있는가"는 PC에서
확인해야 한다(2026-09-18 감사 T29: `tf_check.py`의 [마운트] 절이 문구는
"yaml과 같은가"라면서 실제로는 *0이 아니기만* 하면 통과시키고 있었고, 그
상태로 아무도 눈치채지 못했다). 그래서 **순수 계산만 여기 떼어 놓고**
`ros_selfcheck.py`가 이 함수를 직접 시험한다 — ros2/src 안의 노드와 순수
모듈을 가른 것과 같은 이유다.

허용오차는 **실측을 지키기 위한 값**이지 부동소수 오차가 아니다. 자로 재서
1mm를 고쳤는데 TF가 안 따라오면 그건 잡아야 한다.
"""

from __future__ import annotations

import math
import os

TOL_MM = 1.0    # 자로 재는 분해능. 이보다 큰 어긋남은 "누가 딴 값을 흘렸다"는 뜻
TOL_DEG = 0.5   # 요 0.5°는 300mm 앞에서 2.6mm — 위 1mm와 비슷한 크기다

AXES = (("x", 0), ("y", 1), ("z", 2))


def quat_to_rpy_deg(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """쿼터니언 → (roll, pitch, yaw) 도. URDF rpy와 같은 ZYX 순서."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_p = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_p)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))  # type: ignore[return-value]


def angle_diff_deg(a: float, b: float) -> float:
    """두 각의 차이를 (-180, 180]로 접어서 돌려준다 — 180과 -180은 같은 방향이다."""
    return (a - b + 180.0) % 360.0 - 180.0


def geometry_paths(share_dir: str | None = None, ws_src: str = "/ws/src") -> list[str]:
    """so101_geometry.yaml을 찾을 자리 — **런치가 읽은 것과 같은 파일이 먼저다.**

    런치(`description.launch.py`)는 설치된 share를 읽는다. 소스만 고치고
    빌드를 안 하면 TF는 옛 값을 쓰는데 소스는 새 값이라, 소스를 기준으로
    대조하면 *고쳤는데 안 고쳐진* 상태를 통과시킨다.
    """
    rel = os.path.join("tomato_description", "config", "so101_geometry.yaml")
    out = []
    if share_dir:
        out.append(os.path.join(share_dir, "config", "so101_geometry.yaml"))
    out.append(os.path.join(ws_src, rel))
    return out


def load_mount(paths: list[str]) -> tuple[dict, str]:
    """첫 번째로 실재하는 yaml의 mount 절을 읽는다. 없으면 예외."""
    import yaml  # 컨테이너·PC 모두 있다(ROS도 런치도 pyyaml을 쓴다)

    for path in paths:
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        return (cfg.get("mount") or {}), path
    raise FileNotFoundError("so101_geometry.yaml을 못 찾았다: " + ", ".join(paths))


def compare(mount: dict, xyz_mm: tuple[float, float, float],
            quat: tuple[float, float, float, float]) -> list[tuple[str, bool, str]]:
    """yaml의 mount와 TF 실측을 항목별로 대조한다 → [(이름, 통과, 설명)].

    `quat`은 (x, y, z, w) — geometry_msgs가 주는 순서 그대로.

    키가 없으면 **통과가 아니라 실패다.** 런치는 없는 키를 조용히 건너뛰고
    xacro의 default가 대신 이기는데, 그러면 "정본"이라 적힌 yaml이 실제로는
    아무것도 정하지 않은 상태가 된다.
    """
    out: list[tuple[str, bool, str]] = []

    for key, idx in AXES:
        if key not in mount:
            out.append((f"mount.{key}", False,
                        f"yaml에 mount.{key}가 없다 — xacro의 default가 조용히 이긴다"))
            continue
        want = float(mount[key])
        got = xyz_mm[idx]
        out.append((f"mount.{key}", abs(got - want) <= TOL_MM,
                    f"TF {got:.2f} vs yaml {want:.2f} mm · 차이 {abs(got - want):.2f}"
                    f" (허용 {TOL_MM}mm)"))

    roll, pitch, yaw = quat_to_rpy_deg(*quat)

    if "yaw_deg" not in mount:
        out.append(("mount.yaw_deg", False,
                    "yaml에 mount.yaw_deg가 없다 — xacro의 default가 조용히 이긴다"))
    else:
        want_yaw = float(mount["yaw_deg"])
        d = abs(angle_diff_deg(yaw, want_yaw))
        out.append(("mount.yaw_deg", d <= TOL_DEG,
                    f"TF {yaw:.3f} vs yaml {want_yaw:.3f} 도 · 차이 {d:.3f}"
                    f" (허용 {TOL_DEG}도)"))

    # xacro는 rpy="0 0 yaw"로 박아 놨다. roll/pitch가 0이 아니면 그 자리를
    # 누군가 다른 경로로 바꿨다는 뜻이고, 그러면 yaw만 봐서는 자세를 모른다.
    tilt = max(abs(roll), abs(pitch))
    out.append(("마운트가 기울지 않았다 (roll=pitch=0)", tilt <= TOL_DEG,
                f"roll {roll:.3f} pitch {pitch:.3f} 도 (허용 {TOL_DEG}도)"))

    return out
