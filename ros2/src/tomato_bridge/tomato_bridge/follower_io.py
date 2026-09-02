"""SO-101 팔로워 **직결** — ROS가 팔의 주인이 되는 통로.

이전에는 이 자리에 `arm.LerobotArm`이 있었다. 그건 팔 하나가 아니라 **레거시
대시보드의 팔 전체**다 — 프리셋 20슬롯, 리더암 미러링, 시퀀스 재생, 범위
캘리브레이션 상태까지 들어 있다. ROS 노드가 그걸 통째로 물면 새 계통이 옛
계통의 껍데기가 된다. 그래서 여기서는 lerobot의 `SOFollower`(모터 버스)만
직접 잡는다.

────────────────────────────────────────────────────────────────────────
그러면 무엇을 **여전히 공유하는가** — 계산과 이 팔의 실측값이다.

    kinematics.py   FK/IK          (import: math)
    cartesian.py    영점·환산·안전  (이 팔의 ~/arm_cartesian.json)

이건 "레거시를 따라가는 것"이 아니라 라이브러리를 쓰는 것이다. 다시 구현하면
**같은 팔을 두 계통이 다르게 믿게** 된다 — 이 저장소가 반복해서 비싸게 배운
실패다. 나뉘는 선은 *계산이냐 소유냐*이지 *새 코드냐 옛 코드냐*가 아니다.

  · 계산·실측값 → 공유한다 (여기, cartesian, kinematics)
  · 장치 소유   → ROS가 가진다 (이 파일)
  · 웹 서비스   → 안 쓴다 (arm_source.ProxyArm은 브링업 전용, ros.3에서 삭제)

────────────────────────────────────────────────────────────────────────
⚠ 포트는 한 프로세스만 연다. `tomato-voice.service`가 살아 있으면 여기서 못 연다.
   그게 정상이고, 그때는 **그 서비스를 끄는 것**이 답이다(proxy로 도망가지 말 것).

lerobot은 `_connect()` 안에서만 import한다 — 모듈 최상단에 두면 lerobot이 없는
PC에서 이 파일을 못 읽고, 그러면 자체검증이 통째로 죽는다.
"""

from __future__ import annotations

import sys
import threading
import time

try:
    from tomato_picker.config import (
        ARM_FOLLOWER_SERIAL,
        ARM_ID,
        ARM_LEADER_SERIAL,
        ARM_MOVE_FPS,
        ARM_SERIAL_PORT,
    )
    from tomato_picker.hardware.cartesian import DEG_PER_TICK
    from tomato_picker.hardware.ports import resolve_arm_port
    from tomato_picker.hardware.servo_probe import require_live_bus
except ImportError:  # pragma: no cover - PC에서 상수만으로도 이 파일이 읽혀야 한다
    ARM_FOLLOWER_SERIAL = ARM_LEADER_SERIAL = ""
    ARM_ID = "tomato_follower"
    ARM_MOVE_FPS = 30
    ARM_SERIAL_PORT = "/dev/ttyACM0"
    DEG_PER_TICK = 360.0 / 4096.0
    resolve_arm_port = None
    require_live_bus = None


class FollowerIO:
    """`cartesian.JointIO` 구현 — 정규화값을 읽고 쓰는 통로 하나.

    CartesianArm이 요구하는 다섯 가지(read/write/spans_deg/busy_lock/before_move)
    만 있으면 되고, 그 이상은 일부러 두지 않는다. 프리셋도 미러링도 여기 없다 —
    그건 팔의 기능이 아니라 **레거시 대시보드의 기능**이다.
    """

    def __init__(self, port: str = "", arm_id: str = ARM_ID,
                 move_fps: int = ARM_MOVE_FPS, hold_torque: bool = False) -> None:
        self._port_fallback = port or ARM_SERIAL_PORT
        self._arm_id = arm_id
        # 연결할 때 힘을 뺄 것인가. 기본은 뺀다(미러링·교시가 그걸 전제한다).
        # **팔을 움직이는 도구는 True로 열어야 한다** — 아니면 붙기도 전에 처진다.
        self._hold_torque = bool(hold_torque)
        self._fps = max(5, int(move_fps))
        self._follower = None
        self._port: str | None = None
        # 관절 읽기(10Hz 타이머)와 이동(수 초 블로킹)이 같은 버스를 동시에 건드리면
        # scservo가 "Port is in use!"를 낸다. 여기서 직렬화한다.
        self._lock = threading.RLock()

    # ------------------------------------------------------------------
    # 연결
    # ------------------------------------------------------------------

    def describe(self) -> str:
        return f"follower({self._port or self._port_fallback})"

    @property
    def connected(self) -> bool:
        return self._follower is not None

    @staticmethod
    def _find_lerobot() -> str:
        """lerobot을 못 찾으면 **호스트 venv를 sys.path 끝에 붙인다.**

        왜 이 짓이 필요한가 — lerobot은 컨테이너에 없다. 젯슨의
        `~/lerobot/.venv`(2.0G)에 있고, 그 venv의 파이썬은 3.12.3으로
        컨테이너(우분투 24.04)와 **같은 버전**이라 그대로 읽힌다.

        ⚠ `PYTHONPATH`가 아니라 `sys.path.append`인 것이 중요하다. PYTHONPATH는
          표준 site-packages **앞**에 끼어들어서 venv의 numpy가 컨테이너의 numpy를
          가린다 — cv_bridge는 컨테이너 numpy에 맞춰 빌드돼 있어 그 순간 깨진다.
          끝에 붙이면 **컨테이너에 있는 것이 이기고, 없는 것만** venv에서 온다.

        ⚠ 그래도 대가가 있다: `SOFollower`를 import하면 **torch(607MB)가 딸려온다**
          (실측 3.5초). 서보 하나 열자고 딥러닝 프레임워크를 올리는 셈이다.
          이게 졸업표에서 이 의존을 과도기로 둔 이유다 — 결국은 `scservo_sdk`
          (이미 그 venv에 있는 순수 파이썬 SDK)로 직접 내려가야 한다. 다만 그때
          **정규화(-100..100) 규약을 lerobot과 똑같이 맞춰야** 한다. 안 그러면
          이미 잡아 둔 영점(~/arm_cartesian.json)의 뜻이 통째로 달라진다.
        """
        import glob
        import os

        # ⚠ `glob("<root>/**/site-packages", recursive=True)`로 찾으면 **못 찾는다.**
        #    venv 디렉터리가 `.venv`라 숨김이고, glob의 `**`는 `.`으로 시작하는
        #    디렉터리를 건너뛴다. 마운트도 환경변수도 멀쩡한데 결과가 빈 목록으로
        #    나와서, 증상은 "lerobot을 못 찾았다"로만 보인다(실측).
        #    그래서 venv 경로를 **명시적으로** 적는다.
        patterns = (
            os.path.join("{root}", ".venv", "lib", "python*", "site-packages"),
            os.path.join("{root}", "venv", "lib", "python*", "site-packages"),
            os.path.join("{root}", "lib", "python*", "site-packages"),
            "{root}",  # root 자체가 site-packages인 경우
        )
        for root in (os.environ.get("TOMATO_LEROBOT", ""),
                     os.path.expanduser("~/lerobot")):
            if not root:
                continue
            for pattern in patterns:
                for site in sorted(glob.glob(pattern.format(root=root))):
                    if os.path.isdir(os.path.join(site, "lerobot")):
                        if site not in sys.path:
                            sys.path.append(site)
                        return site
        return ""

    def _connect(self) -> None:
        """지금 실제로 있는 경로를 다시 찾아 연결한다(USB 재열거 대응).

        ⚠ **시리얼번호로 못 박은 팔로워만** 연다. 리더와 팔로워가 같은 CH343이라
          아무거나 집으면 리더암이 움직인다(레거시가 이미 겪은 사고).

        ⚠ 넘기기 전에 버스가 살아 있는지 먼저 묻는다. USB는 12V가 없어도 열거되므로
          "포트가 보인다"가 "서보가 있다"는 뜻이 아니다 — 그대로 lerobot에 넘기면
          응답을 기다리며 **무한 블로킹**되고, 노드가 통째로 선다.
        """
        try:
            from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
        except ImportError:
            site = self._find_lerobot()
            if not site:
                raise ImportError(
                    "lerobot을 못 찾았다. 컨테이너 안이라면 호스트 venv를 물려야 한다 — "
                    "compose가 ${HOME}/lerobot 을 /lerobot 로 마운트하고 "
                    "TOMATO_LEROBOT=/lerobot 을 넣는지 확인하라.") from None
            from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

        port = resolve_arm_port(
            self._port_fallback,
            follower_serial=ARM_FOLLOWER_SERIAL,
            leader_serial=ARM_LEADER_SERIAL,
        )
        require_live_bus(port)
        follower = SOFollower(SOFollowerRobotConfig(port=port, id=self._arm_id))
        follower.connect(calibrate=False)
        if not self._hold_torque:
            follower.bus.disable_torque()
        self._follower = follower
        self._port = port

    def _reconnect(self) -> None:
        old, self._follower = self._follower, None
        try:
            if old is not None:
                old.disconnect()
        except Exception:  # noqa: BLE001 - 이미 죽은 연결이라 실패가 정상
            pass
        self._connect()

    def _with_retry(self, fn):
        """시리얼 오류면 한 번 재연결 후 재시도.

        팔이 끊겼다 붙으면 장치 번호가 바뀐다(ttyACM0→ACM1). 노드를 재시작하지
        않고도 다음 명령이 살아나야 한다 — 부스에서 이게 없으면 매번 사람이 뛴다.
        """
        with self._lock:
            if self._follower is None:
                self._connect()
            try:
                return fn()
            except Exception as first:  # noqa: BLE001 - 어떤 시리얼 오류든 복구 시도
                try:
                    self._reconnect()
                except Exception:  # noqa: BLE001
                    raise first
                return fn()

    def close(self) -> None:
        with self._lock:
            follower, self._follower = self._follower, None
            if follower is None:
                return
            try:
                follower.bus.disable_torque()
                follower.disconnect()
            except Exception:  # noqa: BLE001 - 종료 경로에서 실패는 무시
                pass

    def hold_close(self) -> None:
        """토크를 **켠 채로** 포트를 닫는다 — 그 자리를 붙들고 있게.

        ⚠ `follower.disconnect()`로는 안 된다. lerobot의
          `config.disable_torque_on_disconnect`가 기본 True라 닫으면서 힘을 뺀다.

        2026-08-31: 도구들이 `disconnect()`를 부르며 "토크를 켠 채로 둔다"고 찍고
        있었다. 실제로는 매번 놓았다 — 다음 실행이 팔을 읽으니 elbow가 74° 무너져
        있었고, 나는 그 무너진 자세로 카메라 화면을 해석하다가 **기구학이 틀렸다고**
        결론 낼 뻔했다(FK는 "수평"인데 카메라는 바닥을 봤다). 이 저장소의 1번 병과
        같은 모양이다 — **증상이 원인을 안 가리킨다.**
        """
        with self._lock:
            follower, self._follower = self._follower, None
            if follower is None:
                return
            try:
                follower.bus.disconnect(disable_torque=False)
            except Exception:  # noqa: BLE001 - 종료 경로에서 실패는 무시
                pass

    def relax(self) -> None:
        """토크 해제 — 손으로 자세를 바꿔 영점을 잡을 때 쓴다."""
        self._with_retry(lambda: self._follower.bus.disable_torque())

    # ------------------------------------------------------------------
    # JointIO
    # ------------------------------------------------------------------

    def read(self) -> dict[str, float]:
        obs = self._with_retry(lambda: self._follower.get_observation())
        return {k[:-4]: float(v) for k, v in obs.items() if k.endswith(".pos")}

    def write(self, target: dict[str, float], secs: float) -> None:
        goal = {f"{name}.pos": float(v) for name, v in target.items()}
        self._with_retry(lambda: self._interpolate(goal, secs))

    def _interpolate(self, goal: dict[str, float], secs: float) -> None:
        """현재 자세에서 목표까지 나눠 보낸다.

        ⚠ 스텝마다 `sleep(secs/steps)`만 하면 send_action에 걸린 시간이 누적돼
          실제로는 목표보다 오래 걸리고 움직임이 뚝뚝 끊긴다. **절대 시각** 기준으로
          다음 스텝 시각을 맞추고 남은 시간만 잔다(늦었으면 안 잔다).
        """
        self._follower.bus.enable_torque()
        current = {k: float(v) for k, v in self._follower.get_observation().items()
                   if k.endswith(".pos")}
        steps = max(2, int(secs * self._fps))
        start = time.monotonic()
        interval = secs / steps
        for step in range(1, steps + 1):
            action = {
                k: current.get(k, goal[k])
                   + (goal[k] - current.get(k, goal[k])) * step / steps
                for k in goal
            }
            self._follower.send_action(action)
            rest = (start + interval * step) - time.monotonic()
            if rest > 0:
                time.sleep(rest)

    def spans_deg(self) -> dict[str, float]:
        """정규화 -100..100이 실제 몇 도인지 — **캘리브레이션에서 읽는다**.

        lerobot은 캘리브레이션된 raw 구간을 -100..100으로 편다. 그래서 관절마다
        1단위의 각도가 다르다(wrist_roll은 한 바퀴라 대략 두 배). 이걸 추측하면
        같은 회전 명령이 관절마다 다른 크기로 나간다 — 반드시 읽어야 한다.
        """
        follower = self._follower
        cal = getattr(follower, "calibration", None) or {}
        out: dict[str, float] = {}
        for name, c in cal.items():
            try:
                out[name] = abs(int(c.range_max) - int(c.range_min)) * DEG_PER_TICK
            except (AttributeError, TypeError, ValueError):
                continue
        return out

    def busy_lock(self):
        return self._lock

    def before_move(self) -> None:
        """이동 직전 훅. **여기서는 할 일이 없다** — 미러링이 없기 때문이다.

        레거시는 여기서 리더암 추종을 껐다(목표가 둘이면 서로 싸운다). ROS 계통은
        리더암을 안 쓰므로 목표가 하나뿐이다. 빈 채로 두는 것이 맞고, 그 사실을
        적어 두는 편이 다음 사람이 "빠뜨린 것 아닌가" 하고 뒤지는 것보다 낫다.
        """
