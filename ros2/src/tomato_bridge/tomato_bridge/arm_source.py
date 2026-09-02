"""팔을 어디서 읽고 어디로 지령하는가 — **기본은 직결이다.**

| | `direct` (기본) | `proxy` (브링업 전용, ros.3에서 삭제) |
|---|---|---|
| 포트를 | ROS가 잡는다 | 레거시 대시보드가 잡고 있다 |
| 무엇을 통해 | lerobot `SOFollower` 직결 | `GET /status` · `POST /cmd` |
| 주기 | 팔이 허락하는 만큼 | HTTP 폴링 ~10Hz |
| TF 시각 정렬 | **된다** | **안 된다** |

────────────────────────────────────────────────────────────────────────
왜 proxy가 기본이면 안 되는가 — 새 계통의 존재 이유를 부정하기 때문이다.

이 계통을 만든 까닭 중 하나가 **시간**이었다. 깊이 프레임과 관절값은 다른
시각의 데이터라서, 그 프레임을 찍은 순간의 팔 자세로 풀어야 안 틀린다. 그런데
proxy는 남의 프로세스에게 10Hz로 물어본 값이라 "언제의 자세인가"가 흐릿하고,
대시보드가 바쁘면 stale까지 온다. 그 위에 TF를 그리면 팔이 실제와 다른 곳에
그려지고, 그 위에서 계산한 좌표가 전부 틀린다.

그래서 proxy는 **"레거시 대시보드를 켜 둔 채 배선을 확인하는"** 용도로만 남긴다.
포트가 안 열린다고 여기로 도망가지 말 것 — 답은 `systemctl stop tomato-voice`다.

────────────────────────────────────────────────────────────────────────
`direct`가 여전히 공유하는 것: `cartesian.CartesianArm` (영점·정규화 환산·사거리·
관절한계·"너무 작아 물리적으로 0인 지령 거절"). 이건 **이 팔의 실측값**이고
계산이라 공유가 맞다. 다시 구현하면 두 계통이 같은 팔을 다르게 믿게 된다.
장치를 누가 여느냐만 ROS로 넘어온 것이다 — [`follower_io.py`](follower_io.py).
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Protocol

from .follower_io import FollowerIO

# kinematics.JOINTS와 같은 순서 — URDF 관절 이름도 이것과 같아야 한다.
JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll")
# URDF에는 있지만 kinematics가 안 쓰는 관절. 값을 모를 때 0으로 채운다
# (robot_state_publisher가 모든 비고정 관절값을 요구하므로 빠뜨릴 수 없다).
EXTRA_JOINTS = ("gripper",)

MODES = ("direct", "proxy")
# proxy가 사라지는 이정표. docs/ros2-이행계획.md의 "레거시 의존 졸업표"와 같은 값.
PROXY_REMOVED_AT = "v2.0.0-ros.3"


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

    def warning(self) -> str:
        """이 경로를 쓰는 것 자체가 문제일 때의 경고. 없으면 빈 문자열."""

    def close(self) -> None:
        ...


# ----------------------------------------------------------------------
# direct — ROS가 팔의 주인이다 (기본)
# ----------------------------------------------------------------------

class DirectArm:
    """lerobot `SOFollower` 직결 + `CartesianArm`(영점·안전).

    레거시 `LerobotArm`을 거치지 않는다 — 그건 프리셋·미러링·시퀀스까지 든
    대시보드의 팔이다. 여기서 필요한 건 모터 버스 하나뿐이다.
    """

    def __init__(self, port: str | None = None) -> None:
        self._io = FollowerIO(port or "")
        self._cartesian = None

    def describe(self) -> str:
        return f"direct · {self._io.describe()}"

    def warning(self) -> str:
        return ""

    def _unit(self):
        """CartesianArm은 **처음 쓸 때** 만든다 — 그때 ~/arm_cartesian.json을 읽는다.

        노드가 뜨는 순간 읽으면, 영점을 잡고 나서 노드를 재시작해야 반영된다.
        """
        if self._cartesian is None:
            try:
                from tomato_picker.hardware.cartesian import CartesianArm
            except ImportError as exc:
                raise ArmUnavailable(
                    f"tomato_picker.hardware.cartesian을 import하지 못했다: {exc}. "
                    "PYTHONPATH에 저장소의 src/가 있는지 확인하라."
                ) from exc
            self._cartesian = CartesianArm(self._io)
        return self._cartesian

    def joints_deg(self) -> dict[str, float]:
        try:
            return self._unit().joints_deg()
        except ArmUnavailable:
            raise
        except Exception as exc:  # noqa: BLE001 - 포트 점유·전원 등 이유를 그대로 올린다
            raise ArmUnavailable(
                f"팔을 못 읽는다: {exc}. 포트를 다른 프로세스가 잡고 있으면 "
                "(tomato-voice / controller-drive) **그것부터 끄라** — "
                "proxy로 도망가면 TF 시각이 흐려진다."
            ) from exc

    def move_to(self, x, y, z, pitch, roll) -> str:
        return self._unit().move_to(x=x, y=y, z=z, pitch=pitch, roll=roll)

    def close(self) -> None:
        self._io.close()


# ----------------------------------------------------------------------
# proxy — 레거시 대시보드에게 물어본다 (브링업 전용, 곧 삭제)
# ----------------------------------------------------------------------

class ProxyArm:
    """`GET /status` · `POST /cmd` 로 레거시 voice 대시보드를 통해 팔을 쓴다.

    ⚠ **임시 경로다.** 포트를 안 잡는 대신 "언제의 자세인가"를 잃는다.
      배선 확인용으로만 쓰고, 실측·보정·수확에는 쓰지 마라.
    """

    def __init__(self, base_url: str = "http://127.0.0.1:8090",
                 timeout: float = 1.0) -> None:
        self._url = base_url.rstrip("/")
        self._timeout = timeout

    def describe(self) -> str:
        return f"proxy({self._url})"

    def warning(self) -> str:
        return (f"proxy 모드는 브링업 전용이다 — 관절값이 HTTP 폴링(~10Hz)이라 "
                f"TF 시각 정렬이 안 된다. 보정·수확에는 arm_mode:=direct 를 쓰고, "
                f"포트가 막히면 `systemctl stop tomato-voice`가 답이다. "
                f"({PROXY_REMOVED_AT}에서 삭제 예정)")

    def _get(self, path: str) -> dict:
        try:
            with urllib.request.urlopen(f"{self._url}{path}", timeout=self._timeout) as r:
                return json.loads(r.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, ValueError) as exc:
            raise ArmUnavailable(
                f"대시보드({self._url})에 못 붙었다: {exc}. "
                "tomato-voice.service가 떠 있는지, 포트가 맞는지 확인하라 "
                "(config.VOICE_LOG_HTTP_PORT = 8090)."
            ) from exc

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


def make_source(mode: str, *, url: str, port: str | None) -> ArmSource:
    """파라미터 문자열 → 구현. **모르는 모드는 거절한다**(기본값으로 안 넘어간다)."""
    if mode == "direct":
        return DirectArm(port)
    if mode == "proxy":
        return ProxyArm(url)
    raise ValueError(f"arm_mode는 {MODES} 중 하나다 (받은 값: {mode!r})")
