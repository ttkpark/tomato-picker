#!/usr/bin/env python3
"""3~4단 · 팔 검증 — **ROS가 직접 잡은 팔**이 기구학과 앞뒤가 맞는가.

    python3 /ws/tools/arm_check.py            (stage1 런치가 떠 있는 상태에서)

`tf_check.py`가 가짜 `/joint_states`로 URDF를 검증했다면, 여기는 **실물 팔이
발행한 관절값**으로 같은 사슬을 통과시킨다. 확인하는 것 셋:

  ③ `/joint_states`가 실물에서 온다 — arm_node가 포트를 직접 잡았다는 증거
  ③' TF의 tool0 == kinematics.forward(그 관절값)
     ⚠ 이건 URDF 검증이 아니라 **영점(~/arm_cartesian.json) 검증**에 가깝다.
       레거시 대시보드와 ROS가 같은 영점 파일을 읽으므로, 두 계통이 같은 팔을
       같은 자세로 믿고 있는지가 여기서 드러난다.
  ④ `/arm/move_to_point`가 좌표를 관절로 푼다 (dry_run — **팔은 안 움직인다**)

⚠ 팔을 실제로 움직이지 않는다. 실물 이동은 사람이 보는 앞에서만 해야 한다.
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from tomato_msgs.srv import MoveToPoint

sys.path.insert(0, "/repo/src")
from tomato_picker.hardware import kinematics as kin  # noqa: E402

TOL_MM = 1.0


class ArmCheck(Node):

    def __init__(self) -> None:
        super().__init__("tomato_arm_check")
        self.latest: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)
        self._cli = self.create_client(MoveToPoint, "/arm/move_to_point")
        self.failed: list[str] = []
        self.passed = 0

    def _on_js(self, msg: JointState) -> None:
        self.latest = msg

    def check(self, name: str, ok: bool, detail: str = "") -> None:
        if ok:
            self.passed += 1
            print(f"  ok   {name}" + (f"  ({detail})" if detail else ""))
        else:
            self.failed.append(name)
            print(f"  FAIL {name}  {detail}")

    def wait(self, secs: float) -> None:
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main() -> int:
    rclpy.init()
    node = ArmCheck()

    print("[3단] 실물 팔이 /joint_states를 내는가")
    node.wait(8.0)
    if node.latest is None:
        node.check("/joint_states 수신", False,
                   "8초 동안 한 번도 안 왔다 — arm_node가 팔을 못 열었을 것이다. "
                   "tomato-voice가 아직 살아 있는지 확인하라")
        print(f"\n❌ {len(node.failed)}개 실패")
        rclpy.shutdown()
        return 1

    names = list(node.latest.name)
    degs = {n: math.degrees(p) for n, p in zip(names, node.latest.position)}
    node.check("/joint_states 수신", True,
               " ".join(f"{n.split('_')[0]}={degs[n]:.1f}°" for n in kin.JOINTS))
    node.check("관절 이름이 kinematics와 같다",
               all(j in names for j in kin.JOINTS),
               f"{names}")
    node.check("전부 0이 아니다 (진짜 팔에서 온 값)",
               any(abs(degs.get(j, 0.0)) > 0.5 for j in kin.JOINTS),
               "전부 0이면 실물이 아니라 기본값을 보고 있는 것이다")

    print("\n[3'단] TF의 tool0 == kinematics.forward(그 관절값)")
    try:
        tf = node._buf.lookup_transform("arm_base", "tool0", Time())
        t = tf.transform.translation
        got = (t.x * 1000.0, t.y * 1000.0, t.z * 1000.0)
        want_pose = kin.forward(degs, kin.ArmGeometry())
        want = (want_pose.x, want_pose.y, want_pose.z)
        err = math.dist(got, want)
        node.check("TCP 일치", err < TOL_MM,
                   f"TF ({got[0]:.1f}, {got[1]:.1f}, {got[2]:.1f}) vs FK "
                   f"({want[0]:.1f}, {want[1]:.1f}, {want[2]:.1f}) mm · 차이 {err:.3f}mm")
        print(f"       (참고) 집게 자세 pitch={want_pose.pitch:.1f}° roll={want_pose.roll:.1f}°")
    except Exception as exc:  # noqa: BLE001
        node.check("TCP 일치", False, str(exc)[:100])

    print("\n[4단] /arm/move_to_point — dry_run (팔은 안 움직인다)")
    if not node._cli.wait_for_service(timeout_sec=5.0):
        node.check("서비스가 있다", False, "/arm/move_to_point 가 안 보인다")
    else:
        node.check("서비스가 있다", True)
        # 사거리 안쪽 한 점과 바깥 한 점 — 푸는 것과 **거절하는 것** 둘 다 본다.
        for label, (x, y, z), expect_ok in (
                ("닿는 곳 (220, 0, 100)mm", (0.220, 0.0, 0.100), True),
                ("사거리 밖 (900, 0, 100)mm", (0.900, 0.0, 0.100), False)):
            req = MoveToPoint.Request()
            req.target = PointStamped()
            req.target.header.frame_id = "arm_base"
            req.target.point.x, req.target.point.y, req.target.point.z = x, y, z
            req.approach_pitch_deg = -20.0
            req.standoff_m = 0.0
            req.dry_run = True
            fut = node._cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            res = fut.result()
            if res is None:
                node.check(f"{label}", False, "응답이 없다")
                continue
            node.check(f"{label} → ok={res.ok}", res.ok is expect_ok,
                       res.detail[:110])

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
