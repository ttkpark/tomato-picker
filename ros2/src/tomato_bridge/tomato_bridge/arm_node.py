"""팔 노드 — `/joint_states`를 발행하고 `/arm/move_to_point`를 받는다.

**팔의 유일한 주인이다.** 포트를 한 프로세스만 열 수 있으니 읽기와 쓰기가 한
노드 안에 있어야 한다(둘로 나누면 둘 중 하나는 절대 못 붙는다).

무엇을 하지 **않는가**도 중요하다 — IK도, 영점 환산도, 사거리 검사도 여기서
새로 짜지 않는다. 전부 `kinematics.py`/`cartesian.py`에 있고 이 노드는 그걸
호출할 뿐이다. 이 노드의 진짜 일은 셋이다:

  ① 각도(°) → 라디안, mm → m  (단위 경계)
  ② TF로 목표점을 **arm_base 좌표로** 옮기기
  ③ 실패를 문장으로 돌려주기

────────────────────────────────────────────────────────────────────────
`/joint_states`가 왜 중요한가 — 이게 있어야 robot_state_publisher가 TF를 만들고,
TF가 있어야 카메라가 본 열매를 팔 좌표로 옮길 수 있다. **팔 자세를 모르면
손-눈 보정도 무의미하다**(같은 카메라 점이 팔 자세에 따라 다른 곳을 가리킨다).

타임스탬프는 **관절을 읽은 시각**을 쓴다. 발행 시각을 쓰면 팔이 움직이는 중에
찍힌 프레임과 어긋나고, 그 어긋남은 조용히 좌표 오차가 된다.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState
from std_msgs.msg import String
# ⚠ import만 하고 안 쓰는 것처럼 보이지만 **반드시 있어야 한다** — 이 모듈이
#    PointStamped의 변환 함수를 tf2에 등록한다. 없으면 Buffer.transform()이
#    "타입을 모른다"며 실패하고, 그 에러 메시지는 원인을 안 가리킨다.
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformListener

from tomato_msgs.srv import MoveToPoint

from .arm_source import EXTRA_JOINTS, JOINT_NAMES, ArmUnavailable, make_source

try:  # 저장소의 src/가 PYTHONPATH에 있어야 한다 (docker/entrypoint.sh가 넣어 준다)
    from tomato_picker.hardware import kinematics as kin
except ImportError as exc:  # pragma: no cover - 컨테이너 설정 실수를 즉시 드러낸다
    raise SystemExit(
        f"tomato_picker.hardware.kinematics를 import하지 못했다: {exc}\n"
        "이 노드는 기존 기구학을 그대로 쓴다 — 저장소 src/를 PYTHONPATH에 넣어라."
    ) from exc


class ArmNode(Node):

    def __init__(self) -> None:
        super().__init__("tomato_arm")

        # ⚠ 기본은 direct — ROS가 팔의 주인이다. proxy는 브링업 전용이며
        #    관절값이 HTTP 폴링이라 TF 시각 정렬이 안 된다(arm_source.py 참고).
        self.declare_parameter("arm_mode", "direct")
        self.declare_parameter("dashboard_url", "http://127.0.0.1:8090")
        self.declare_parameter("arm_port", "")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("base_frame", "arm_base")
        # 못 읽을 때 얼마나 자주 불평할 것인가. 팔이 빠져 있으면 초당 10번 같은
        # 줄이 찍혀 로그가 못 쓰게 된다 — 그래서 상태가 **바뀔 때만** 찍는다.
        self.declare_parameter("gripper_deg", 0.0)

        mode = self.get_parameter("arm_mode").value
        self._base_frame = self.get_parameter("base_frame").value
        self._source = make_source(
            mode,
            url=self.get_parameter("dashboard_url").value,
            port=self.get_parameter("arm_port").value or None,
        )
        self.get_logger().info(f"팔 연결 방식: {self._source.describe()}")
        if self._source.warning():
            # 임시 경로로 돌고 있다는 사실은 **뜰 때마다** 말해야 한다.
            # 조용하면 브링업용 설정이 그대로 실측까지 따라간다.
            self.get_logger().warning(self._source.warning())

        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self._status = self.create_publisher(
            String, "~/status", QoSPresetProfiles.SYSTEM_DEFAULT.value)

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self._srv = self.create_service(MoveToPoint, "arm/move_to_point", self._on_move)

        hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self._timer = self.create_timer(1.0 / hz, self._tick)
        self._last_error: str | None = None

    # ------------------------------------------------------------------
    # 관절 발행
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        try:
            degs = self._source.joints_deg()
        except ArmUnavailable as exc:
            self._note(str(exc))
            return
        stamp = self.get_clock().now().to_msg()  # 읽은 직후 = 그 값의 시각
        self._note(None)

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(JOINT_NAMES) + list(EXTRA_JOINTS)
        # 집게는 지금 읽을 길이 없다(대시보드 status가 안 준다). 0으로 채우되
        # **모른다는 걸 파라미터 이름으로 남긴다** — 나중에 읽게 되면 여기만 고친다.
        gripper = math.radians(float(self.get_parameter("gripper_deg").value))
        # ⚠ 한 번에 대입한다. `msg.position += [...]`로 이어붙이면 죽는다 —
        #    rclpy의 float64[] 필드는 리스트가 아니라 array.array('d')라서
        #    `TypeError: can only extend array with array (not "list")`가 난다.
        #    (실측: 팔이 붙은 첫 프레임에서 노드가 통째로 죽었다.)
        msg.position = ([math.radians(degs.get(j, 0.0)) for j in JOINT_NAMES]
                        + [gripper] * len(EXTRA_JOINTS))
        self._pub.publish(msg)

    def _note(self, error: str | None) -> None:
        """상태가 **바뀔 때만** 로그와 토픽에 남긴다(초당 10줄 방지)."""
        if error == self._last_error:
            return
        self._last_error = error
        if error:
            self.get_logger().warning(f"팔을 못 읽는다 — {error}")
        else:
            self.get_logger().info("팔 관절 읽기 정상")
        self._status.publish(String(data=error or "ok"))

    # ------------------------------------------------------------------
    # 좌표로 보내기
    # ------------------------------------------------------------------

    def _on_move(self, req: MoveToPoint.Request,
                 res: MoveToPoint.Response) -> MoveToPoint.Response:
        try:
            x_mm, y_mm, z_mm = self._to_arm_base_mm(req.target)
        except Exception as exc:  # noqa: BLE001 - TF 실패 이유를 그대로 올린다
            res.ok = False
            res.detail = (f"목표를 {self._base_frame} 좌표로 못 옮겼다: {exc}. "
                          "손-눈 보정을 했는지, 그 static TF가 떠 있는지 확인하라.")
            return res

        pitch = float(req.approach_pitch_deg)
        pose = kin.ToolPose(x=x_mm, y=y_mm, z=z_mm, pitch=pitch)

        # 스탠드오프 — 접근축 **반대로** 물러난 지점까지만 간다. 열매에 바로
        # 꽂으면 오차가 곧 충돌이므로, 먼저 근처에 서고 마지막 몇 mm는 2단계
        # (시각 서보잉)가 닫는다. docs/ros2-이행계획.md 참고.
        standoff_mm = float(req.standoff_m) * 1000.0
        if standoff_mm:
            dx, dy, dz = kin.offset_in_tool_frame(pose, -standoff_mm, 0.0, 0.0)
            pose = pose.replace(x=pose.x + dx, y=pose.y + dy, z=pose.z + dz)

        try:
            joints = kin.inverse(pose, _geometry())
        except kin.Unreachable as exc:
            res.ok = False
            res.detail = f"IK가 안 풀린다 — {exc}"
            return res

        res.joints_deg = [float(joints[j]) for j in JOINT_NAMES]
        res.reached = Point(x=pose.x / 1000.0, y=pose.y / 1000.0, z=pose.z / 1000.0)

        if req.dry_run:
            res.ok = True
            res.detail = ("dry_run — 팔은 안 움직였다. IK는 풀린다: "
                          + ", ".join(f"{j}={joints[j]:.1f}°" for j in JOINT_NAMES))
            return res

        try:
            detail = self._source.move_to(pose.x, pose.y, pose.z, pitch, None)
        except Exception as exc:  # noqa: BLE001 - 사거리·관절한계·너무 작은 지령 등
            res.ok = False
            res.detail = f"이동 실패 — {exc}"
            return res

        res.ok = True
        res.detail = detail or "이동 완료"
        return res

    def _to_arm_base_mm(self, target) -> tuple[float, float, float]:
        """PointStamped → arm_base 기준 (mm, mm, mm).

        stamp가 0이면 "가장 최근"으로 조회한다. 검출 메시지가 주는 stamp를 그대로
        쓰면 **그 프레임을 찍은 순간의 팔 자세**로 변환되므로, 팔이 움직이는
        중에 온 목표도 안 틀린다. 이게 tf2를 쓰는 이유의 절반이다.
        """
        frame = target.header.frame_id or self._base_frame
        if frame == self._base_frame:
            p = target.point
        else:
            out = self._tf.transform(target, self._base_frame)
            p = out.point
        return (p.x * 1000.0, p.y * 1000.0, p.z * 1000.0)


def _geometry() -> "kin.ArmGeometry":
    """링크 길이는 **팔이 가진 파일**이 정본이다(`~/arm_cartesian.json`).

    URDF의 yaml과 이 파일이 어긋나면 rviz의 팔과 실제 팔이 다른 곳에 있다 —
    ros2/tools/ros_selfcheck.py가 그 어긋남을 검사한다.
    """
    try:
        from tomato_picker.hardware.cartesian import FrameConfig
        return FrameConfig().geometry()
    except Exception:  # noqa: BLE001 - 파일이 없으면 코드 기본값(공장 초기값)
        return kin.ArmGeometry()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._source.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
