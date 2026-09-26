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


import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .board_contract import (AxisSigns, Caps, DutyCalib, MockBase, MobileBase,
                             SimBase, Stm32Base, UnoAdapterBase, plan)

try:
    from tomato_picker.hardware.motor_link import MotorLink
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        f"tomato_picker.hardware.motor_link를 import하지 못했다: {exc}\n"
        "저장소 src/를 PYTHONPATH에 넣고 pyserial이 있는지 확인하라."
    ) from exc


class CmdVelNode(Node):

    def __init__(self, base: MobileBase | None = None) -> None:
        super().__init__("tomato_base")

        self.declare_parameter("base_type", "uno")  # uno | sim | stm32 | mock (보드계약 §11, §13)
        self.declare_parameter("cmd_timeout", 0.3)   # 보드계약 §9 4층
        self.declare_parameter("telemetry_hz", 10.0) # 보드계약 §11.1, §11.3 스냅샷 발행 주기
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
        base_type = str(self.get_parameter("base_type").value).lower()
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

        if base is not None:
            self._base: MobileBase = base
            self._link = getattr(base, "_link", None)
            self._caps = base.caps()
        elif base_type == "mock":
            self._link = None
            self._base = MockBase()
            self._caps = self._base.caps()
        elif base_type == "sim":
            self._link = None
            self._base = SimBase(deadman_enabled=True)
            self._caps = self._base.caps()
        elif base_type == "stm32":
            self._link = MotorLink(**({"port": port} if port else {}))
            self._base = Stm32Base(motor_link=self._link)
            self._caps = self._base.caps()
        elif base_type == "uno":
            self._link = MotorLink(**({"port": port} if port else {}))
            self._base = UnoAdapterBase(motor_link=self._link, calib=self._calib, signs=self._signs)
            self._caps = self._base.caps()
        else:
            raise ValueError(f"지원하지 않는 base_type이다: {base_type!r} (지원: 'uno', 'sim', 'stm32', 'mock')")

        if base_type == "uno" and not self._calib.measured:
            self.get_logger().warning(
                "duty 환산이 실측이 아니다 — /cmd_vel의 m/s는 방향과 비율만 맞다. "
                "docs/ros2-이행계획.md의 'duty 곡선 재기'를 하고 duty_measured:=true로.")

        self._sub = self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self._status = self.create_publisher(String, "~/status", 10)
        self._telem_pub = self.create_publisher(String, "~/telemetry", 10)
        self._last_cmd_ns = 0
        self._stopped = True
        self._last_notes: tuple[str, ...] = ()
        # 데드맨은 지령 주기와 무관하게 돌아야 한다 — 지령이 **안 오는 것**을
        # 감시하는 타이머라서, 지령 콜백 안에 두면 영영 안 돈다.
        self._timer = self.create_timer(0.05, self._watch)

        # 텔레메트리 주기 발행 (보드계약 §11.1, §11.3)
        telem_hz = float(self.get_parameter("telemetry_hz").value)
        if telem_hz > 0:
            self._telem_timer = self.create_timer(1.0 / telem_hz, self._publish_telemetry)
        else:
            self._telem_timer = None

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd_ns = self.get_clock().now().nanoseconds

        # 물리 단위(m/s -> mm/s, rad/s -> mdeg/s)로 변환하여 MobileBase 인터페이스 호출
        vx_mms = int(round(msg.linear.x * 1000.0))
        vy_mms = int(round(msg.linear.y * 1000.0))
        w_mdegs = int(round(math.degrees(msg.angular.z) * 1000.0))

        self._base.set_velocity(vx_mms, vy_mms, w_mdegs)

        # UnoAdapterBase 또는 구현체의 plan 결과 노트 로깅
        last_cmd = getattr(self._base, "_last_cmd", None)
        if last_cmd is not None:
            if last_cmd.notes != self._last_notes:
                self._last_notes = last_cmd.notes
                for note in last_cmd.notes:
                    self.get_logger().warning(note)

            if last_cmd.rejected:
                self._halt(last_cmd.reason)
                return

            if last_cmd.payload == "S":
                self._halt("정지 지령")
                return

            self._stopped = not last_cmd.moving
        else:
            self._stopped = (vx_mms == 0 and vy_mms == 0 and w_mdegs == 0)

    def _watch(self) -> None:
        """지령이 끊기면 선다 (보드계약 §9 4층)."""
        if self._stopped:
            return
        timeout_ns = float(self.get_parameter("cmd_timeout").value) * 1e9
        if self.get_clock().now().nanoseconds - self._last_cmd_ns > timeout_ns:
            self._halt(f"{float(self.get_parameter('cmd_timeout').value):.1f}초 동안 "
                       "/cmd_vel이 없었다 — 데드맨 정지")

    def _halt(self, why: str) -> None:
        self._base.stop()
        if not self._stopped:
            self.get_logger().info(f"정지: {why}")
            self._status.publish(String(data=why))
        self._stopped = True

    def _publish_telemetry(self) -> None:
        """하위 MobileBase 텔레메트리 스냅샷을 ~/telemetry JSON 토픽으로 발행 (보드계약 §11.1, §11.3)."""
        try:
            telem = self._base.telemetry()
            data = telem.to_dict() if hasattr(telem, "to_dict") else {}
            self._telem_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))
        except Exception as exc:  # noqa: BLE001 - 진단 토픽 발행 실패로 본체를 죽이지 않는다
            self.get_logger().debug(f"텔레메트리 발행 실패: {exc}")

    def destroy_node(self) -> bool:
        # 노드가 죽을 때 바퀴가 돌고 있으면 안 된다. 보드 데드맨이 1초 뒤 세우긴
        # 하지만, 그 1초는 부스에서 충분히 길다.
        try:
            if hasattr(self._base, "close"):
                self._base.close()
            else:
                self._base.stop()
                if self._link is not None and hasattr(self._link, "close"):
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
