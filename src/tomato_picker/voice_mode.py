"""음성 명령 진입점 — "팔 움직여"/"앞으로 가"/"토마토"를 들으면 팔/베이스를
동작시키고, 인식 결과를 실시간 로그 서버(SSE)로 내보낸다. camera는 필요
없어 팔+베이스만 조립한다.
"""

from __future__ import annotations

import time

from .config import (
    BASE_DRIVE_FORWARD_SECONDS,
    TOMATO_APPROACH_SECONDS,
    TOMATO_RETREAT_SECONDS,
    USE_REAL_ARM,
    USE_REAL_BASE,
    USE_REAL_CAMERA,
    USE_SHARED_VISION,
    VISION_SHM_COUNT,
    VISION_SHM_JPEG,
    VOICE_LOG_HTTP_PORT,
)
from .hardware.base import MobileBase, RobotArm
from .hardware.mock import MockArm, MockBase
from .voice.controller import VoiceController
from .voice.log_hub import LogHub
from .voice.server import start_log_server
from .voice.shared_vision import SharedFrameSource
from .voice.vision_stream import VisionStreamer


def _now() -> str:
    return time.strftime("%H:%M:%S")


def _build_arm() -> RobotArm:
    if USE_REAL_ARM:
        from .hardware.arm import LerobotArm

        return LerobotArm()
    return MockArm()


def _build_base() -> MobileBase:
    if USE_REAL_BASE:
        from .hardware.jetson import JetsonBase

        return JetsonBase()
    return MockBase()


def _build_vision(log_hub: LogHub):
    """대시보드 비전 소스를 만든다.

    - USE_SHARED_VISION: 별도 GPU YOLO 프로세스(/dev/shm)를 읽는 SharedFrameSource
      (카메라를 직접 안 열어 음성 서비스와 카메라 점유 충돌 없음). 우선.
    - USE_REAL_CAMERA: 같은 프로세스에서 색검출하는 VisionStreamer(폴백).
    카메라/소스 실패해도 음성·주행은 계속돼야 하므로 예외 시 None.
    """
    if USE_SHARED_VISION:
        src = SharedFrameSource(log_hub, VISION_SHM_JPEG, VISION_SHM_COUNT)
        src.start()
        return src
    if not USE_REAL_CAMERA:
        return None
    try:
        from .hardware.camera import JetsonCamera

        streamer = VisionStreamer(JetsonCamera(), log_hub)
        streamer.start()
        return streamer
    except Exception as exc:  # noqa: BLE001 - 카메라 없어도 음성/주행은 계속돼야 함
        print(f"[voice] 카메라 열기 실패: {exc} — 영상 없이 계속")
        return None


def run_voice() -> None:
    arm = _build_arm()
    base = _build_base()
    log_hub = LogHub()
    vision = _build_vision(log_hub)
    start_log_server(log_hub, port=VOICE_LOG_HTTP_PORT, vision=vision)
    print(f"[voice] 실시간 대시보드: http://<젯슨IP>:{VOICE_LOG_HTTP_PORT}  (Ctrl+C로 종료)")

    def on_intent(intent: str) -> None:
        if intent == "arm_move":
            print("[voice] '팔 움직여' 인식 → 데모 동작 재생")
            arm.demo_move()
        elif intent == "drive_forward":
            print(f"[voice] '앞으로 가' 인식 → {BASE_DRIVE_FORWARD_SECONDS:.1f}초 전진")
            base.drive_forward(BASE_DRIVE_FORWARD_SECONDS)
        elif intent == "tomato_pick":
            print("[voice] '토마토' 인식 → 전진→프리셋→후진 시퀀스 시작")
            log_hub.publish({"ts": _now(), "kind": "status", "text": f"토마토: {TOMATO_APPROACH_SECONDS:.1f}초 전진"})
            base.drive_forward(TOMATO_APPROACH_SECONDS)
            log_hub.publish({"ts": _now(), "kind": "status", "text": "토마토: 프리셋 재생"})
            arm.demo_move()
            log_hub.publish({"ts": _now(), "kind": "status", "text": f"토마토: {TOMATO_RETREAT_SECONDS:.1f}초 후진 복귀"})
            base.drive_backward(TOMATO_RETREAT_SECONDS)
            log_hub.publish({"ts": _now(), "kind": "status", "text": "토마토: 시퀀스 완료"})

    controller = VoiceController(log_hub=log_hub, on_intent=on_intent)
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\n[voice] 종료.")
