"""팔을 어디서 읽고 어디로 지령하는가 — **두 가지 길, 같은 인터페이스**.

⚠ 팔 포트(`/dev/ttyACM0`)는 **한 프로세스만** 연다. 기존 대시보드
(`tomato-voice.service`)가 켜져 있으면 ROS는 그 포트를 못 잡는다. 이건 버그가
아니라 물리다. 그래서 길이 둘이다.

| | `direct` | `proxy` |
|---|---|---|
| 포트를 | 직접 잡는다 | 안 잡는다 |
| 기존 대시보드 | **꺼야 한다** | 켜 둔 채로 |
| 주기 | 팔이 허락하는 만큼(~30Hz) | HTTP 폴링(~10Hz) |
| 쓰는 때 | 실전 | 대시보드로 팔을 보면서 ROS를 붙여 볼 때, 데모 중 |

**둘 다 계산을 새로 하지 않는다.** 정규화값↔각도 환산, 영점, 사거리·관절한계
검사, "너무 작아서 물리적으로 0인 지령 거절"은 전부 이미
[`cartesian.py`](../../../../src/tomato_picker/hardware/cartesian.py)에 있다.
여기서 그걸 다시 구현하면 두 계통이 **같은 팔을 다르게 믿게** 된다 — 이 저장소가
이미 여러 번 비싸게 배운 실패다. `direct`는 그 객체를 그대로 쓰고, `proxy`는
그 객체를 가진 프로세스에게 물어본다.

rclpy가 여기 없다(stdlib만). `proxy`는 PC에서 가짜 서버로 검증할 수 있다.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Protocol

# kinematics.JOINTS와 같은 순서 — URDF 관절 이름도 이것과 같아야 한다.
JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll")
# URDF에는 있지만 kinematics가 안 쓰는 관절. 값을 모를 때 0으로 채운다
# (robot_state_publisher가 모든 비고정 관절값을 요구하므로 빠뜨릴 수 없다).
EXTRA_JOINTS = ("gripper",)


class ArmUnavailable(RuntimeError):
    """지금 팔을 읽거나 움직일 수 없다. **왜인지가 메시지에 있어야 한다.**"""


class ArmSource(Protocol):
    """노드가 팔에 요구하는 전부."""

    def joints_deg(self) -> dict[str, float]:
        """지금 관절 각(기구학 규약, 도). 못 읽으면 ArmUnavailable."""

    def move_to(self, x: float, y: float, z: float,
                pitch: float | None, roll: float | None) -> str:
        """집게를 좌표(mm, base)로. 사람이 읽을 결과 문장을 돌려준다.

        ⚠ 못 가면 **예외**다. 가까운 데까지 가고 성공했다고 하지 않는다.
        """

    def describe(self) -> str:
        """진단용 한 줄 — 어느 길로 붙어 있는지."""

    def close(self) -> None:
        ...


# ----------------------------------------------------------------------
# proxy — 기존 대시보드에게 물어본다 (포트를 안 잡는다)
# ----------------------------------------------------------------------

class ProxyArm:
    """`GET /status` · `POST /cmd` 로 기존 voice 대시보드를 통해 팔을 쓴다.

    대시보드가 이미 `arm.cartesian.snapshot()`을 status에 실어 보내므로 관절값은
    **이미 도(°) 단위**다 — 여기서 환산할 것이 없다. 그게 이 길의 장점이다.
    """

    def __init__(self, base_url: str = "http://127.0.0.1:8000",
                 timeout: float = 1.0) -> None:
        self._url = base_url.rstrip("/")
        self._timeout = timeout

    def describe(self) -> str:
        return f"proxy({self._url})"

    def _get(self, path: str) -> dict:
        try:
            with urllib.request.urlopen(f"{self._url}{path}", timeout=self._timeout) as r:
                return json.loads(r.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, ValueError) as exc:
            raise ArmUnavailable(
                f"대시보드({self._url})에 못 붙었다: {exc}. "
                "tomato-voice.service가 떠 있는지 확인하라 "
                "(proxy 모드는 그 프로세스를 통해 팔을 쓴다).") from exc

    def _post(self, body: dict) -> str:
        data = json.dumps(body).encode("utf-8")
        req = urllib.request.Request(f"{self._url}/cmd", data=data,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=max(self._timeout, 5.0)) as r:
                out = json.loads(r.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, ValueError) as exc:
            raise ArmUnavailable(f"대시보드에 지령을 못 보냈다: {exc}") from exc
        if not out.get("ok"):
            # 대시보드가 실패 이유를 문장으로 준다 — 그대로 올린다.
            raise ArmUnavailable(str(out.get("detail") or "이유 없이 실패"))
        return str(out.get("detail") or "")

    def joints_deg(self) -> dict[str, float]:
        cart = (self._get("/status").get("arm") or {}).get("cartesian") or {}
        joints = cart.get("joints")
        if not joints:
            # snapshot()은 팔 버스가 바쁘면 joints=None에 stale=True를 준다.
            # 그 사실을 삼키지 않는다 — 오래된 각도로 TF를 그리면 팔이 실제와
            # 다른 곳에 그려지고, 그 위에서 계산한 좌표가 전부 틀린다.
            raise ArmUnavailable(
                "대시보드가 관절값을 안 준다 — "
                + str(cart.get("error") or "팔이 바쁘거나(stale) 미연결"))
        return {k: float(v) for k, v in joints.items()}

    def move_to(self, x, y, z, pitch, roll) -> str:
        body = {"action": "arm_tool_move", "x": x, "y": y, "z": z}
        if pitch is not None:
            body["pitch"] = pitch
        if roll is not None:
            body["roll"] = roll
        return self._post(body)

    def close(self) -> None:
        pass


# ----------------------------------------------------------------------
# direct — 포트를 직접 잡는다
# ----------------------------------------------------------------------

class DirectArm:
    """`tomato_picker.hardware.arm.LerobotArm`을 이 프로세스에서 연다.

    lerobot을 **여기서 import하지 않는다**(모듈 최상단에 두면 lerobot이 없는
    PC에서 이 파일을 못 읽는다 — 셀프체크가 죽는다). 붙는 순간에만 끌어온다.
    """

    def __init__(self, port: str | None = None) -> None:
        self._port = port
        self._arm = None

    def describe(self) -> str:
        return f"direct({self._port or 'auto'})"

    def _connect(self):
        if self._arm is not None:
            return self._arm
        try:
            from tomato_picker.hardware.arm import LerobotArm
        except ImportError as exc:
            raise ArmUnavailable(
                f"tomato_picker/lerobot을 import하지 못했다: {exc}. "
                "PYTHONPATH에 저장소의 src/가 있는지, lerobot venv 안인지 확인하라."
            ) from exc
        try:
            self._arm = LerobotArm(**({"port": self._port} if self._port else {}))
        except Exception as exc:  # noqa: BLE001 — 포트 점유·전원 등 무엇이든 이유를 올린다
            raise ArmUnavailable(
                f"팔에 못 붙었다: {exc}. 포트를 다른 프로세스가 잡고 있으면 "
                "(tomato-voice / controller-drive) 그것부터 끄거나 proxy 모드를 써라."
            ) from exc
        return self._arm

    def joints_deg(self) -> dict[str, float]:
        return self._connect().cartesian.joints_deg()

    def move_to(self, x, y, z, pitch, roll) -> str:
        return self._connect().cartesian.move_to(x=x, y=y, z=z, pitch=pitch, roll=roll)

    def close(self) -> None:
        arm, self._arm = self._arm, None
        if arm is not None:
            try:
                arm.disconnect()
            except Exception:  # noqa: BLE001 — 종료 경로에서 예외를 올리지 않는다
                pass


def make_source(mode: str, *, url: str, port: str | None) -> ArmSource:
    """파라미터 문자열 → 구현. **모르는 모드는 거절한다**(기본값으로 안 넘어간다)."""
    if mode == "proxy":
        return ProxyArm(url)
    if mode == "direct":
        return DirectArm(port)
    raise ValueError(f"arm_mode는 'direct' 또는 'proxy'다 (받은 값: {mode!r})")
