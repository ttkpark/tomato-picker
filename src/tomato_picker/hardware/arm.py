"""SO-101 Follower 팔 — 프리셋 포즈 보간 재생으로 구현한 실물 RobotArm.

담당 범위:
  · 프리셋 슬롯 0~9 재생/저장 (저장소는 presets.PresetStore)
  · **높이 앵커 보간 재생** — 비전이 준 토마토 y좌표로 상/중/하 사이 자세를 계산
  · **리더암 미러링** — 리더(SO-101 Leader)를 손으로 움직이면 팔로워가 따라오고,
    원하는 자세에서 슬롯에 저장한다. 예전에는 tools/mirror_toggle.py를 터미널에서
    돌려야 했지만(리더 필요 + SSH 필요), 이제 대시보드에서 켜고 끈다.

RobotArm.pick()의 position 인자는 아직 고정 시퀀스만 쓴다 — 좌표 기반 접근은
play_blended(y) 쪽이 담당한다(비전 y → 자세).
"""

from __future__ import annotations

import threading
import time

from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

from ..config import (
    ARM_FOLLOWER_SERIAL,
    ARM_HOME_PRESET,
    ARM_ID,
    ARM_LEADER_ID,
    ARM_LEADER_SERIAL,
    ARM_MIRROR_FPS,
    ARM_MOVE_FPS,
    ARM_MOVE_SECS,
    ARM_PICK_PRESETS,
    ARM_PLACE_PRESETS,
    ARM_PRESET_FILE,
    ARM_SERIAL_PORT,
)
from .base import RobotArm
from .ports import list_arm_ports, resolve_arm_port, resolve_leader_port
from .presets import PresetStore


def _pose_only(observation: dict) -> dict[str, float]:
    return {k: float(v) for k, v in observation.items() if k.endswith(".pos")}


class LerobotArm(RobotArm):
    """arm_presets.json의 저장 자세를 보간 재생하고, 리더암 미러링으로 저장하는 실물 팔."""

    def __init__(
        self,
        port: str = ARM_SERIAL_PORT,
        arm_id: str = ARM_ID,
        preset_file: str = ARM_PRESET_FILE,
    ) -> None:
        self._port_fallback = port
        self._arm_id = arm_id
        # 두 스레드(음성 인텐트 워커 / 대시보드 수동조작 / 미러링)가 같은 시리얼
        # 버스를 동시에 건드리면 scservo가 "Port is in use!"를 낸다 — 여기서 직렬화.
        self._lock = threading.RLock()
        self._follower: SOFollower | None = None
        self._port = port
        self._connect()
        self.presets = PresetStore(preset_file)
        # --- 리더암(미러링) 상태 ---
        self._leader = None
        self._leader_port: str | None = None
        self._mirroring = False
        self._mirror_thread: threading.Thread | None = None
        self._mirror_error: str | None = None

    # ------------------------------------------------------------------
    # 연결 관리
    # ------------------------------------------------------------------

    @property
    def preset_ids(self) -> list[int]:
        """저장된 프리셋 번호(오름차순) — 대시보드 버튼 생성용."""
        return self.presets.slot_ids()

    def _connect(self) -> None:
        """지금 실제로 존재하는 경로를 다시 찾아 연결한다(재열거 대응).

        ⚠ 포트는 **시리얼번호로 못 박은 팔로워**만 연다 — 리더/팔로워가 같은
        CH343이라 아무거나 집으면 리더암이 프리셋을 재생하게 된다(config 참고).
        """
        port = resolve_arm_port(
            self._port_fallback,
            follower_serial=ARM_FOLLOWER_SERIAL,
            leader_serial=ARM_LEADER_SERIAL,
        )
        follower = SOFollower(SOFollowerRobotConfig(port=port, id=self._arm_id))
        follower.connect(calibrate=False)
        follower.bus.disable_torque()
        self._follower = follower
        self._port = port

    def _reconnect(self) -> None:
        """죽은 연결을 버리고 새로 연다. USB가 빠졌다 다시 붙으면 장치 번호가
        바뀌므로(ttyACM0→ACM1) 경로 재탐색이 핵심."""
        old, self._follower = self._follower, None
        try:
            if old is not None:
                old.disconnect()
        except Exception:  # noqa: BLE001 - 이미 죽은 연결이라 실패가 정상
            pass
        self._connect()

    def _with_retry(self, fn):
        """시리얼 오류면 한 번 재연결 후 재시도. 팔이 끊겼다 붙었을 때
        서비스를 재시작하지 않고도 다음 명령이 살아나게 한다."""
        with self._lock:
            try:
                return fn()
            except Exception as first:  # noqa: BLE001 - 어떤 시리얼 오류든 복구 시도
                print(f"  [arm] 명령 실패({first}) — 재연결 후 1회 재시도")
                try:
                    self._reconnect()
                except Exception as exc:  # noqa: BLE001
                    raise RuntimeError(f"팔 재연결 실패: {exc}") from first
                return fn()

    # ------------------------------------------------------------------
    # RobotArm 인터페이스
    # ------------------------------------------------------------------

    def pick(self, position: tuple[float, float]) -> None:
        """position은 아직 미사용 — 고정 프리셋 시퀀스만 재생.
        좌표 기반 접근은 play_blended(y)를 쓸 것."""
        self._play_sequence(ARM_PICK_PRESETS)

    def place_in_basket(self) -> None:
        self._play_sequence(ARM_PLACE_PRESETS)

    def home(self) -> None:
        self.play_preset(ARM_HOME_PRESET)

    def demo_move(self) -> None:
        """음성 명령 트리거용 — 검증된 전체 시퀀스(1→2→3→4) 재생."""
        self._play_sequence(ARM_PICK_PRESETS + ARM_PLACE_PRESETS)

    def relax(self) -> None:
        """토크를 풀어 손으로 자세를 바꿀 수 있게 한다(수동조작 화면의 '힘 빼기')."""
        self.stop_mirror()
        self._with_retry(lambda: self._follower.bus.disable_torque())

    def close(self) -> None:
        self.stop_mirror()
        self.disconnect_leader()
        with self._lock:
            if self._follower is None:
                return
            try:
                self._follower.bus.disable_torque()
                self._follower.disconnect()
            except Exception:  # noqa: BLE001 - 종료 경로에서 실패는 무시
                pass

    # ------------------------------------------------------------------
    # 프리셋 — 재생
    # ------------------------------------------------------------------

    def play_preset(self, preset_id: int, secs: float = ARM_MOVE_SECS) -> None:
        """프리셋 하나 재생. 미러링 중이면 먼저 끈다(리더와 목표가 싸우지 않게)."""
        target = self.presets.get(preset_id)
        if not target:
            raise KeyError(f"프리셋 {preset_id}가 {self.presets.path}에 없습니다.")
        self.stop_mirror()
        self._with_retry(lambda: self._move_to(target, secs, ARM_MOVE_FPS))

    def play_sequence(self, preset_ids: list[int]) -> None:
        """여러 슬롯을 순서대로 재생 — 웹에서 '4,6,8' 같이 지정할 수 있다."""
        self._play_sequence(preset_ids)

    def play_blended(self, y: float, secs: float = ARM_MOVE_SECS) -> str:
        """토마토 높이(y, 화면 세로 픽셀)에 맞춰 앵커 사이 자세를 계산해 재생.

        반환값은 "'상'(슬롯 4) : '중'(슬롯 6) = 53% : 47%" 같은 설명 문자열 —
        대시보드가 무엇을 어떻게 섞었는지 그대로 보여준다.
        """
        pose, desc = self.presets.blend(y)
        self.stop_mirror()
        self._with_retry(lambda: self._move_to(pose, secs, ARM_MOVE_FPS))
        return desc

    # ------------------------------------------------------------------
    # 프리셋 — 저장/편집
    # ------------------------------------------------------------------

    def current_pose(self) -> dict[str, float]:
        """지금 팔로워의 관절값. 토크가 풀려 있어도 읽힌다(엔코더 값)."""
        return self._with_retry(lambda: _pose_only(self._follower.get_observation()))

    def save_preset(self, slot: int, name: str | None = None) -> dict[str, float]:
        """**지금 팔로워 자세**를 슬롯에 저장.

        미러링 ON이면 리더가 만든 자세가, OFF(힘 빼기)면 손으로 잡아둔 자세가
        그대로 저장된다 — 두 방식 다 같은 경로를 쓴다.
        """
        pose = self.current_pose()
        self.presets.save(slot, pose, name)
        return pose

    def delete_preset(self, slot: int) -> None:
        self.presets.delete(slot)

    def set_anchor(self, slot: int, label: str, y: float) -> None:
        self.presets.set_anchor(slot, label, y)

    def clear_anchor(self, slot: int) -> None:
        self.presets.clear_anchor(slot)

    # ------------------------------------------------------------------
    # 리더암 미러링
    # ------------------------------------------------------------------

    @property
    def leader_connected(self) -> bool:
        return self._leader is not None

    @property
    def mirroring(self) -> bool:
        return self._mirroring

    def leader_port_candidates(self) -> list[str]:
        """웹에서 리더 포트를 고를 수 있게, 팔로워가 쓰는 포트를 뺀 후보 목록."""
        follower_real = self._port
        return [p for p in list_arm_ports() if p != follower_real]

    def connect_leader(self, port: str | None = None) -> str:
        """리더암 연결. port 생략 시 팔로워가 안 쓰는 첫 ttyACM을 자동 선택."""
        from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

        if self._leader is not None:
            return self._leader_port or ""
        chosen = port or resolve_leader_port(self._port, ARM_LEADER_SERIAL)
        if not chosen:
            raise RuntimeError(
                "리더암 포트를 찾지 못했습니다 — 리더 USB가 꽂혀 있는지 확인하세요 "
                f"(팔로워={self._port})"
            )
        leader = SOLeader(SOLeaderTeleopConfig(port=chosen, id=ARM_LEADER_ID))
        leader.connect(calibrate=False)
        self._leader = leader
        self._leader_port = chosen
        self._mirror_error = None
        return chosen

    def disconnect_leader(self) -> None:
        self.stop_mirror()
        leader, self._leader = self._leader, None
        self._leader_port = None
        if leader is None:
            return
        try:
            leader.disconnect()
        except Exception:  # noqa: BLE001 - 정리 경로 실패는 무시
            pass

    def start_mirror(self) -> None:
        """리더 → 팔로워 실시간 추종 시작. 리더가 없으면 자동으로 연결을 시도한다."""
        if self._leader is None:
            self.connect_leader()
        if self._mirroring:
            return
        self._mirror_error = None
        self._mirroring = True
        self._mirror_thread = threading.Thread(
            target=self._mirror_loop, daemon=True, name="arm-mirror"
        )
        self._mirror_thread.start()

    def stop_mirror(self) -> None:
        """미러링 정지. 팔로워는 **토크를 유지**해 마지막 자세를 잡고 있는다
        (여기서 힘을 빼면 방금 잡아둔 자세가 중력에 무너져 저장할 수가 없다)."""
        if not self._mirroring:
            return
        self._mirroring = False
        thread, self._mirror_thread = self._mirror_thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    def _mirror_loop(self) -> None:
        period = 1.0 / max(1.0, ARM_MIRROR_FPS)
        try:
            with self._lock:
                self._follower.bus.enable_torque()
            while self._mirroring:
                start = time.monotonic()
                with self._lock:
                    if not self._mirroring:
                        break
                    action = self._leader.get_action()
                    self._follower.send_action(action)
                slack = period - (time.monotonic() - start)
                if slack > 0:
                    time.sleep(slack)
        except Exception as exc:  # noqa: BLE001 - 미러링이 죽어도 서비스는 살아야 함
            self._mirror_error = str(exc)
            self._mirroring = False
            print(f"  [arm] 미러링 중단: {exc}")

    # ------------------------------------------------------------------
    # 상태 스냅샷 (웹 UI)
    # ------------------------------------------------------------------

    def status(self) -> dict:
        return {
            "follower_port": self._port,
            "leader_connected": self.leader_connected,
            "leader_port": self._leader_port,
            "leader_candidates": self.leader_port_candidates(),
            "mirroring": self._mirroring,
            "mirror_error": self._mirror_error,
            "presets": self.presets.snapshot(),
        }

    # ------------------------------------------------------------------
    # 내부
    # ------------------------------------------------------------------

    def _play_sequence(self, preset_ids: list[int]) -> None:
        for preset_id in preset_ids:
            self.play_preset(preset_id)

    def _move_to(self, target: dict, secs: float, fps: int) -> None:
        self._follower.bus.enable_torque()
        current = _pose_only(self._follower.get_observation())
        steps = max(2, int(secs * fps))
        # 스텝마다 sleep(secs/steps)만 하면 send_action에 걸린 시간이 누적돼
        # 실제론 목표보다 오래 걸리고 움직임이 뚝뚝 끊긴다. 절대 시각 기준으로
        # 다음 스텝 시각을 맞춰 남은 시간만 잔다(늦었으면 안 잔다).
        start = time.monotonic()
        interval = secs / steps
        for step in range(1, steps + 1):
            action = {
                k: current.get(k, target[k]) + (target[k] - current.get(k, target[k])) * step / steps
                for k in target
            }
            self._follower.send_action(action)
            slack = (start + step * interval) - time.monotonic()
            if slack > 0:
                time.sleep(slack)
