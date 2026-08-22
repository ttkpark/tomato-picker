"""하드웨어 없이 PC에서 도는 Mock 구현.

동작을 콘솔에 출력하고, 카메라는 무대를 흉내 낸 합성 이미지를 만든다.
나무마다 빨강/초록 열매가 정해진 위치에 박혀 있어 색검출 파이프라인을
실제 영상 없이도 끝까지 검증할 수 있다.
"""

from __future__ import annotations

import time

import cv2
import numpy as np

from .base import Camera, MobileBase, RobotArm


def _log(msg: str) -> None:
    print(f"  [hw] {msg}")


class MockBase(MobileBase):
    def __init__(self) -> None:
        self.position = 0.0

    def drive_to(self, distance: float) -> None:
        _log(f"베이스 이동: {self.position:.0f} → {distance:.0f}")
        time.sleep(0.2)
        self.position = distance

    def drive_forward(self, seconds: float) -> None:
        _log(f"베이스: {seconds:.1f}초간 전진(음성 트리거)")
        time.sleep(min(seconds, 0.3))

    def drive_backward(self, seconds: float) -> None:
        _log(f"베이스: {seconds:.1f}초간 후진(음성 트리거)")
        time.sleep(min(seconds, 0.3))

    def drive(self, seconds: float, vx: int = 0, vy: int = 0, w: int = 0) -> None:
        """JetsonBase.drive와 같은 시그니처 — 수동조작 화면이 Mock에서도 돌게."""
        _log(f"베이스: vx={vx} vy={vy} w={w} 로 {seconds:.1f}초")
        time.sleep(min(seconds, 0.3))

    def hold(self, vx: int = 0, vy: int = 0, w: int = 0) -> None:
        """홀드 주행(키보드) — Mock에서는 로그만. 도배 방지로 값이 바뀔 때만 찍는다."""
        target = (vx, vy, w)
        if target != getattr(self, "_last_hold", None):
            self._last_hold = target
            _log(f"베이스 홀드: vx={vx} vy={vy} w={w}")

    def stop(self) -> None:
        _log("베이스: 정지")

    _tuning = {"hz": 700, "max_pwm": 2600, "accel": 6, "decel": 12}

    def link_stats(self) -> dict:
        """JetsonBase와 같은 모양 — 대시보드 링크/튜닝 패널이 Mock에서도 뜨게."""
        return {"connected": False, "port": None, "hb_age": None,
                "fw_rx": 0, "fw_bad": 0, "nak": 0, "error": "Mock(미연결)",
                "target": getattr(self, "_last_hold", (0, 0, 0)),
                "tuning": dict(self._tuning)}

    def tune(self, **values) -> dict:
        self._tuning = {**self._tuning,
                        **{k: int(v) for k, v in values.items() if v is not None}}
        _log(f"베이스 튜닝: {self._tuning}")
        return dict(self._tuning)


class MockArm(RobotArm):
    def pick(self, position: tuple[float, float]) -> None:
        _log(f"팔: ({position[0]:.0f}, {position[1]:.0f}) 열매 따기 시퀀스")
        time.sleep(0.3)

    def place_in_basket(self) -> None:
        _log("팔: 바구니에 담기")
        time.sleep(0.2)

    def home(self) -> None:
        _log("팔: 홈 포지션 복귀")
        time.sleep(0.2)

    def demo_move(self) -> None:
        _log("팔: 데모 동작(음성 트리거)")
        time.sleep(0.3)

    # --- 수동조작 화면이 LerobotArm과 같은 방식으로 부를 수 있게 맞춰둔다 ---
    #
    # 프리셋은 **읽기만** 실제 파일에서 한다. 팔이 없는 상태에서 저장하면
    # 진짜 관절값이 아닌 가짜 값이 ~/arm_presets.json을 덮어써 데모용 자세가
    # 통째로 날아가므로, 쓰기 계열은 전부 거부한다.
    # 좌표(xyz) 유닛은 **가짜 관절 위에** 그대로 얹는다 — 화면·안전검사·계산이
    # 팔 없이도 진짜와 같은 코드를 탄다. 다만 설정 파일은 실물과 갈라 쓴다
    # (Mock의 영점은 진짜 팔에서 아무 의미가 없는 숫자라, 섞이면 위험하다).
    MOCK_START_DEG = {"shoulder_pan": 0.0, "shoulder_lift": 75.0, "elbow_flex": -85.0,
                      "wrist_flex": -20.0, "wrist_roll": 0.0}

    def __init__(self) -> None:
        from ..config import ARM_CART_FILE, ARM_PRESET_FILE
        from .cartesian import CartesianArm, SimJointIO
        from .kinematics import JOINTS
        from .presets import PresetStore

        self.presets = PresetStore(ARM_PRESET_FILE)
        sim = SimJointIO()
        self.cartesian = CartesianArm(sim, path=ARM_CART_FILE + ".mock")
        self.cartesian.config.set_zero({j: 0.0 for j in JOINTS})
        sim.joints.update(self.cartesian.to_norms(self.MOCK_START_DEG))

    @property
    def preset_ids(self) -> list[int]:
        return self.presets.slot_ids()

    def play_preset(self, preset_id: int, secs: float | None = None) -> None:
        _log(f"팔: 프리셋 {preset_id} 재생")
        time.sleep(0.3)

    def play_sequence(self, preset_ids: list[int]) -> None:
        for preset_id in preset_ids:
            self.play_preset(preset_id)

    def play_blended(self, y: float, secs: float | None = None) -> str:
        _, desc = self.presets.blend(y)
        _log(f"팔: 높이 {y:.0f} 보간 재생 ({desc})")
        time.sleep(0.3)
        return desc

    def relax(self) -> None:
        _log("팔: 토크 해제(힘 빼기)")

    def _no_hardware(self, what: str):
        raise RuntimeError(f"팔이 연결되지 않아 {what}을(를) 할 수 없습니다(Mock)")

    def current_pose(self) -> dict[str, float]:
        self._no_hardware("자세 읽기")

    def save_preset(self, slot: int, name: str | None = None):
        self._no_hardware("프리셋 저장")

    def delete_preset(self, slot: int) -> None:
        self.presets.delete(slot)

    def set_anchor(self, slot: int, label: str, y: float) -> None:
        self.presets.set_anchor(slot, label, y)

    def clear_anchor(self, slot: int) -> None:
        self.presets.clear_anchor(slot)

    def connect_leader(self, port: str | None = None):
        self._no_hardware("리더암 연결")

    def disconnect_leader(self) -> None:
        pass

    def start_mirror(self) -> None:
        self._no_hardware("미러링")

    def stop_mirror(self) -> None:
        pass

    leader_connected = False
    mirroring = False

    def status(self) -> dict:
        return {
            "follower_port": None, "leader_connected": False, "leader_port": None,
            "leader_candidates": [], "mirroring": False,
            "mirror_error": "Mock(팔 미연결)", "presets": self.presets.snapshot(),
            "cartesian": self.cartesian.snapshot(),
        }


class MockCamera(Camera):
    """현재 베이스 위치에 따라 무대 한 나무를 합성해 보여준다."""

    def __init__(self, base: MockBase) -> None:
        self._base = base
        # 나무 거리 → (BGR 색, (x, y) 위치) 열매 목록.
        # OpenCV는 BGR이므로 빨강=(0,0,255), 초록=(0,200,0).
        self._stage: dict[float, list[tuple[tuple[int, int, int], tuple[int, int]]]] = {
            0.0: [((0, 0, 255), (160, 200)), ((0, 200, 0), (420, 260))],
            40.0: [((0, 0, 255), (200, 180)), ((0, 0, 255), (430, 300))],
            80.0: [((0, 200, 0), (240, 220))],
        }

    def capture(self) -> np.ndarray:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        for color, (x, y) in self._stage.get(self._base.position, []):
            cv2.circle(frame, (x, y), 35, color, thickness=-1)
        return frame
