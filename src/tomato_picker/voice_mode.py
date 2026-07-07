"""음성 명령 진입점 — "팔 움직여"를 들으면 팔을 동작시키고, 인식 결과를
실시간 로그 서버(SSE)로 내보낸다. base/camera는 필요 없어 팔만 조립한다.
"""

from __future__ import annotations

from .config import USE_REAL_ARM, VOICE_LOG_HTTP_PORT
from .hardware.base import RobotArm
from .hardware.mock import MockArm
from .voice.controller import VoiceController
from .voice.log_hub import LogHub
from .voice.server import start_log_server


def _build_arm() -> RobotArm:
    if USE_REAL_ARM:
        from .hardware.arm import LerobotArm

        return LerobotArm()
    return MockArm()


def run_voice() -> None:
    arm = _build_arm()
    log_hub = LogHub()
    start_log_server(log_hub, port=VOICE_LOG_HTTP_PORT)
    print(f"[voice] 실시간 인식 로그: http://<젯슨IP>:{VOICE_LOG_HTTP_PORT}  (Ctrl+C로 종료)")

    def on_intent(intent: str) -> None:
        if intent == "arm_move":
            print("[voice] '팔 움직여' 인식 → 데모 동작 재생")
            arm.demo_move()

    controller = VoiceController(log_hub=log_hub, on_intent=on_intent)
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\n[voice] 종료.")
