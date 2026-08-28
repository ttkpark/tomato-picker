#!/usr/bin/env python3
"""TF 자체검증 — **robot_state_publisher가 실제로 만든 TF**를 읽어 대조한다.

    ros2 run ... 없이:  python3 /ws/tools/tf_check.py     (컨테이너 안, 런치가 떠 있는 상태)

`ros_selfcheck.py`와 무엇이 다른가 —

  · `ros_selfcheck`는 xacro **텍스트를 내가 파싱해** 관절 원점·축을 꺼내 계산했다.
    강하지만 구멍이 하나 있다: **내가 xacro를 잘못 읽었다면 검사도 같이 틀린다.**
  · 여기서는 xacro를 진짜 xacro가 펴고, 진짜 robot_state_publisher가 TF를 만들고,
    그것을 tf2로 조회해서 `kinematics.forward()`와 비교한다. 그 구멍이 막힌다.

즉 이 파일은 "URDF가 내 생각대로 생겼는가"가 아니라 **"ROS가 이해한 로봇이
기구학이 말하는 로봇과 같은가"** 를 묻는다. 둘이 다르면 rviz의 팔과 실제 팔이
다른 곳에 있고, 그 위에서 계산한 좌표는 전부 틀린다.

⚠ 팔도 카메라도 필요 없다. `/joint_states`를 **여기서 만들어 쏘기** 때문이다.
   그래서 영점을 잡기 전에도, 팔이 다른 프로세스에 물려 있어도 돌릴 수 있다.
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, "/repo/src")
from tomato_picker.hardware import kinematics as kin  # noqa: E402

JOINTS = list(kin.JOINTS)
EXTRA = ["gripper"]

# ros_selfcheck의 표와 **같은 자세들**을 쓴다 — 두 검사가 같은 것을 보고 있어야
# "한쪽만 통과"가 무슨 뜻인지 분명해진다.
CASES = [
    ("전부 0 (앞으로 수평)", {}),
    ("어깨만 30°", {"shoulder_lift": 30.0}),
    ("팔꿈치만 -45°", {"elbow_flex": -45.0}),
    ("손목만 20°", {"wrist_flex": 20.0}),
    ("pan 40°", {"shoulder_pan": 40.0}),
    ("곧게 위로(교시 자세)", {"shoulder_lift": 90.0}),
    ("섞은 자세", {"shoulder_pan": -25.0, "shoulder_lift": 55.0,
                 "elbow_flex": -70.0, "wrist_flex": -35.0}),
]

TOL_MM = 0.5   # 부동소수·쿼터니언 왕복 오차만 허용한다. 부호가 틀리면 수십 mm가 난다.


class TfCheck(Node):

    def __init__(self) -> None:
        super().__init__("tomato_tf_check")
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)
        self.failed: list[str] = []
        self.passed = 0

    def check(self, name: str, ok: bool, detail: str = "") -> None:
        if ok:
            self.passed += 1
            print(f"  ok   {name}" + (f"  ({detail})" if detail else ""))
        else:
            self.failed.append(name)
            print(f"  FAIL {name}  {detail}")

    def hold(self, joints: dict, secs: float = 1.2) -> None:
        """그 자세를 잠깐 유지해서 발행한다.

        한 번만 쏘면 robot_state_publisher가 아직 구독을 붙이기 전이라 놓친다
        (조용히 옛 TF가 조회돼 "값이 안 바뀐다"로 보인다). 그래서 유지한다.
        """
        end = time.monotonic() + secs
        while time.monotonic() < end:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINTS + EXTRA
            msg.position = [math.radians(joints.get(j, 0.0)) for j in JOINTS] + [0.0]
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def translation_mm(self, parent: str, child: str) -> tuple[float, float, float]:
        tf = self._buf.lookup_transform(parent, child, Time())
        t = tf.transform.translation
        return (t.x * 1000.0, t.y * 1000.0, t.z * 1000.0)


def main() -> int:
    rclpy.init()
    node = TfCheck()

    print("TF ↔ 기구학 대조 — robot_state_publisher가 만든 것을 읽는다\n")

    # TF 트리가 설 때까지 기다린다. 안 서면 런치가 안 떠 있는 것이다.
    node.hold({}, secs=2.5)
    try:
        node.translation_mm("arm_base", "tool0")
    except Exception as exc:  # noqa: BLE001
        print(f"  FAIL TF 트리가 없다 — {exc}")
        print("       description.launch.py(robot_state_publisher)가 떠 있는지 확인하라.")
        rclpy.shutdown()
        return 1

    print("[프레임] 트리가 이어져 있는가")
    for parent, child in (("base_link", "arm_base"), ("arm_base", "tool0"),
                          ("base_link", "tool0")):
        try:
            xyz = node.translation_mm(parent, child)
            node.check(f"{parent} → {child}", True,
                       " ".join(f"{v:.1f}" for v in xyz) + " mm")
        except Exception as exc:  # noqa: BLE001
            node.check(f"{parent} → {child}", False, str(exc)[:80])

    print("\n[대조] TF의 tool0 == kinematics.forward()")
    geom = kin.ArmGeometry()
    for label, joints in CASES:
        node.hold(joints)
        try:
            got = node.translation_mm("arm_base", "tool0")
        except Exception as exc:  # noqa: BLE001
            node.check(f"TCP — {label}", False, str(exc)[:80])
            continue
        want_pose = kin.forward(joints, geom)
        want = (want_pose.x, want_pose.y, want_pose.z)
        err = math.dist(got, want)
        node.check(f"TCP — {label}", err < TOL_MM,
                   f"TF ({got[0]:.1f}, {got[1]:.1f}, {got[2]:.1f}) vs FK "
                   f"({want[0]:.1f}, {want[1]:.1f}, {want[2]:.1f}) mm · 차이 {err:.3f}mm")

    # 마운트 — base_link에서 arm_base까지가 yaml과 같은가.
    print("\n[마운트] base_link → arm_base 가 so101_geometry.yaml과 같은가")
    node.hold({})
    try:
        mount = node.translation_mm("base_link", "arm_base")
        node.check("마운트 오프셋이 값을 가진다", any(abs(v) > 1e-6 for v in mount),
                   " ".join(f"{v:.1f}" for v in mount) + " mm "
                   "(⚠ 자로 잰 값인지 확인 — 기본값은 추정치다)")
    except Exception as exc:  # noqa: BLE001
        node.check("마운트 오프셋", False, str(exc)[:80])

    print()
    if node.failed:
        print(f"❌ {len(node.failed)}개 실패 / {node.passed + len(node.failed)}개 중")
        for name in node.failed:
            print(f"   - {name}")
        rclpy.shutdown()
        return 1
    print(f"✅ 전부 통과 ({node.passed}개)")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
