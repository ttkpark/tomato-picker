"""`/cmd_vel` → 주행 보드. 단위 변환은 `board_contract.py`가 하고, 이 노드는
**전송과 데드맨**만 한다.

전송은 기존 [`motor_link.MotorLink`](../../../../src/tomato_picker/hardware/motor_link.py)를
그대로 쓴다. 새로 짜지 않는 이유는 그 안에 비싸게 배운 것들이 들어 있어서다 —
DTR을 내린 채 열기(Uno 리셋 방지), 전용 스레드의 20ms 재전송(젯슨이 바빠도
데드맨에 안 걸림), XOR 체크섬, 자동 재연결. ROS 노드가 이걸 다시 구현하면
그 교훈들을 처음부터 다시 배우게 된다.

────────────────────────────────────────────────────────────────────────
데드맨 (보드계약 §9의 4층)

  이 노드가 `/cmd_vel`을 `cmd_timeout`초 동안 못 받으면 **스스로 정지**한다.
  위(텔레옵·자율주행)가 죽어도 로봇은 선다. 아래 3층(젯슨 재전송 스레드,
  보드 소프트/하드 데드맨)은 그대로 살아 있다 — 층은 서로를 대신하지 않는다.

⚠ **포트는 한 프로세스만** 연다. `controller-drive.service`(게임패드)가 떠 있으면
   이 노드는 포트를 못 잡는다. 데모 중이라면 그쪽을 끄고 이쪽을 켜거나, 반대로.
"""

from __future__ import annotations


import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .board_contract import AxisSigns, Caps, DutyCalib, plan

try:
    from tomato_picker.hardware.motor_link import MotorLink
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        f"tomato_picker.hardware.motor_link를 import하지 못했다: {exc}\n"
        "저장소 src/를 PYTHONPATH에 넣고 pyserial이 있는지 확인하라."
    ) from exc


class CmdVelNode(Node):

    def __init__(self) -> None:
        super().__init__("tomato_base")

        self.declare_parameter("cmd_timeout", 0.3)   # 보드계약 §9 4층
        self.declare_parameter("serial_port", "")    # 비우면 motor_link가 찾는다
        # 축 부호 — 보드계약 §14.1이 아직 안 닫혔다. 실기에서 정하면 여기 기본값을
        # 바꾸고 계약 문서의 결정 항목을 닫아라(런타임 토글로 남기지 말 것).
        self.declare_parameter("sign_vx", 1)
        self.declare_parameter("sign_vy", 1)
        self.declare_parameter("sign_w", 1)
        # duty 환산. ⚠ measured=false인 동안은 m/s의 절대 크기를 못 믿는다.
        self.declare_parameter("duty_ks", 90)
        self.declare_parameter("duty_kv", 0.35)
        self.declare_parameter("duty_ks_w", 90)
        self.declare_parameter("duty_kv_w", 1.1)
        self.declare_parameter("duty_max", 255)
        self.declare_parameter("duty_measured", False)

        port = self.get_parameter("serial_port").value
        self._link = MotorLink(**({"port": port} if port else {}))
        self._caps = Caps.legacy()  # 지금 펌웨어는 `cap`을 안 뱉는다 (보드계약 §6)
        self._calib = DutyCalib(
            ks=int(self.get_parameter("duty_ks").value),
            kv=float(self.get_parameter("duty_kv").value),
            ks_w=int(self.get_parameter("duty_ks_w").value),
            kv_w=float(self.get_parameter("duty_kv_w").value),
            max_duty=int(self.get_parameter("duty_max").value),
            measured=bool(self.get_parameter("duty_measured").value),
        )
        self._signs = AxisSigns(
            vx=int(self.get_parameter("sign_vx").value),
            vy=int(self.get_parameter("sign_vy").value),
            w=int(self.get_parameter("sign_w").value),
        )
        if not self._calib.measured:
            self.get_logger().warning(
                "duty 환산이 실측이 아니다 — /cmd_vel의 m/s는 방향과 비율만 맞다. "
                "docs/ros2-이행계획.md의 'duty 곡선 재기'를 하고 duty_measured:=true로.")

        self._sub = self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self._status = self.create_publisher(String, "~/status", 10)
        self._last_cmd_ns = 0
        self._stopped = True
        self._last_notes: tuple[str, ...] = ()
        # 데드맨은 지령 주기와 무관하게 돌아야 한다 — 지령이 **안 오는 것**을
        # 감시하는 타이머라서, 지령 콜백 안에 두면 영영 안 돈다.
        self._timer = self.create_timer(0.05, self._watch)

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd_ns = self.get_clock().now().nanoseconds
        command = plan(msg.linear.x, msg.linear.y, msg.angular.z,
                       caps=self._caps, calib=self._calib, signs=self._signs)

        if command.notes != self._last_notes:
            self._last_notes = command.notes
            for note in command.notes:
                self.get_logger().warning(note)

        if command.rejected:
            self._halt(command.reason)
            return

        if command.duty is not None:
            self._link.set_velocity(*command.duty)
            self._stopped = not command.moving
            return

        if command.payload == "S":
            self._halt("정지 지령")
            return

        # 물리 단위 경로(`C`)는 계약대로 계산은 되지만 **보낼 길이 아직 없다** —
        # MotorLink의 재전송 스레드는 `V`를 되풀이하므로 `C`와 섞으면 마지막에
        # 온 것이 이겨서 서로를 지운다(보드계약 §5.1). 물리 단위 보드가 실제로
        # 생기면 계약 v2 전용 링크를 만들어 여기 물린다. 그때까지는 **거절**한다.
        self._halt(
            f"보드가 물리 단위를 지원한다고 나왔지만({command.payload}) 지금 전송 "
            "계층은 duty(`V`) 전용이다. 계약 v2 링크가 필요하다 — 조용히 duty로 "
            "바꾸지 않는다.")

    def _watch(self) -> None:
        """지령이 끊기면 선다 (보드계약 §9 4층)."""
        if self._stopped:
            return
        timeout_ns = float(self.get_parameter("cmd_timeout").value) * 1e9
        if self.get_clock().now().nanoseconds - self._last_cmd_ns > timeout_ns:
            self._halt(f"{float(self.get_parameter('cmd_timeout').value):.1f}초 동안 "
                       "/cmd_vel이 없었다 — 데드맨 정지")

    def _halt(self, why: str) -> None:
        self._link.stop()
        if not self._stopped:
            self.get_logger().info(f"정지: {why}")
            self._status.publish(String(data=why))
        self._stopped = True

    def destroy_node(self) -> bool:
        # 노드가 죽을 때 바퀴가 돌고 있으면 안 된다. 보드 데드맨이 1초 뒤 세우긴
        # 하지만, 그 1초는 부스에서 충분히 길다.
        try:
            self._link.stop()
            self._link.close()
        except Exception:  # noqa: BLE001 - 종료 경로
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
