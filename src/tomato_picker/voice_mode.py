"""음성 명령 진입점 — "팔 움직여"/"앞으로 가"/"토마토"를 들으면 팔/베이스를
동작시키고, 인식 결과를 실시간 로그 서버(SSE)로 내보낸다. camera는 필요
없어 팔+베이스만 조립한다.

**하드웨어가 하나도 안 붙어 있어도 대시보드는 뜬다.** 예전에는 팔 연결
실패(예: /dev/ttyACM0 없음)가 그대로 예외로 올라와 프로세스가 죽었고,
systemd가 3초마다 재시작만 반복해 화면 자체가 안 떴다(2026-08-05 실기).
부스 데모에서는 검은 화면보다 "장비 미연결"이라고 떠 있는 게 낫다. 그래서
① 로그 서버를 하드웨어보다 **먼저** 띄우고, ② 팔·베이스·비전·마이크는 각각
실패해도 Mock/None으로 대체하며 상태만 페이지에 표시하고, ③ 음성 루프가
죽어도 프로세스는 살아남아 페이지를 계속 서빙한다.
"""

from __future__ import annotations

import threading
import time

from .config import (
    BASE_DRIVE_FORWARD_SECONDS,
    TOMATO_APPROACH_SECONDS,
    TOMATO_RETREAT_SECONDS,
    USE_REAL_ARM,
    USE_REAL_BASE,
    USE_REAL_CAMERA,
    USE_SHARED_VISION,
    VISION_SHM_JPEG,
    VISION_SHM_STATUS,
    VOICE_LOG_HTTP_PORT,
    VOICE_LOG_HTTP_REDIRECT_PORT,
)
from .hardware.base import MobileBase, RobotArm
from .hardware.mock import MockArm, MockBase
from .voice.controller import VoiceController
from .voice.log_hub import LogHub
from .voice.server import start_log_server, start_redirect_server
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
        try:
            src = SharedFrameSource(log_hub, VISION_SHM_JPEG, VISION_SHM_STATUS)
            src.start()
            return src
        except Exception as exc:  # noqa: BLE001 - 비전 프로세스가 안 떠 있어도 계속
            print(f"[voice] 공유 비전 소스 열기 실패: {exc} — 영상 없이 계속")
            return None
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


def _safe_build(label: str, build, fallback, log_hub: LogHub, hw: dict[str, str]):
    """하드웨어 하나를 조립한다. 실패하면 Mock으로 대체하고 상태만 기록.

    부스 데모 중 장비 하나가 빠졌다고 전체가 죽으면 안 된다 — 무엇이 빠졌는지
    페이지에 보여주고 나머지는 그대로 돌린다.
    """
    try:
        obj = build()
        hw[label] = "ok"
        return obj
    except Exception as exc:  # noqa: BLE001 - 어떤 장비가 없어도 대시보드는 떠야 함
        hw[label] = "down"
        msg = f"{label} 연결 실패: {exc} — 해당 동작은 무시됩니다(대시보드는 정상)"
        print(f"[voice] {msg}")
        log_hub.publish({"ts": _now(), "kind": "error", "text": msg})
        return fallback()


HW_RETRY_SEC = 15.0


def _retry_hardware_forever(
    log_hub: LogHub, hw: dict[str, str], hardware: dict, publish_hw
) -> None:
    """실패한 장비를 주기적으로 다시 잡아본다.

    팔의 USB가 데모 도중 끊겼다 붙는 일이 잦은데(케이블·전원 문제), 그때마다
    서비스를 재시작하면 Whisper 로딩에 수십 초가 날아간다. 여기서 조용히
    재연결해 두면 사용자가 케이블만 다시 꽂아도 알아서 살아난다.
    """
    builders = {"로봇팔": ("arm", _build_arm), "바퀴": ("base", _build_base)}
    while True:
        time.sleep(HW_RETRY_SEC)
        for label, (key, build) in builders.items():
            if hw.get(label) == "ok":
                continue
            try:
                obj = build()
            except Exception:  # noqa: BLE001 - 아직 안 붙었을 뿐, 조용히 다음 주기에 재시도
                continue
            hardware[key] = obj
            hw[label] = "ok"
            msg = f"{label} 재연결 성공 — 다시 사용할 수 있습니다"
            print(f"[voice] {msg}")
            log_hub.publish({"ts": _now(), "kind": "status", "text": msg})
            publish_hw()


def run_voice() -> None:
    # 1) 로그 허브·HTTP 서버가 먼저다 — 하드웨어 조립에서 무슨 일이 나든
    #    브라우저에는 페이지가 떠 있고 실패 사유가 로그로 보인다.
    log_hub = LogHub()
    hw: dict[str, str] = {}
    # /control 수동 조작 화면이 참조하는 가변 핸들 — 아래에서 채운다.
    hardware: dict = {"arm": None, "base": None}
    vision = _build_vision(log_hub)
    hw["카메라"] = "ok" if vision is not None else "down"
    start_log_server(log_hub, port=VOICE_LOG_HTTP_PORT, vision=vision, hardware=hardware)
    start_redirect_server(VOICE_LOG_HTTP_REDIRECT_PORT, VOICE_LOG_HTTP_PORT)
    print(f"[voice] 실시간 대시보드: http://<젯슨IP>:{VOICE_LOG_HTTP_PORT}  (Ctrl+C로 종료)")
    print(f"[voice] 수동 조작:      http://<젯슨IP>:{VOICE_LOG_HTTP_PORT}/control")

    # 2) 하드웨어는 실패해도 Mock으로 대체 — 상태는 페이지 상단 배지로.
    hardware["arm"] = _safe_build("로봇팔", _build_arm, MockArm, log_hub, hw)
    hardware["base"] = _safe_build("바퀴", _build_base, MockBase, log_hub, hw)
    hw["마이크"] = "확인 중"

    def publish_hw() -> None:
        log_hub.publish({"ts": _now(), "kind": "hw", "items": dict(hw)}, latest_only=True)

    publish_hw()
    # 끊긴 장비를 백그라운드에서 계속 다시 잡는다 — 케이블만 다시 꽂으면 복구.
    threading.Thread(
        target=_retry_hardware_forever, args=(log_hub, hw, hardware, publish_hw), daemon=True
    ).start()

    # 인텐트 처리는 항상 hardware 딕셔너리에서 **지금** 핸들을 꺼낸다 —
    # 재연결로 객체가 교체돼도 옛 객체를 붙들고 있지 않게.
    def on_intent(intent: str) -> None:
        arm = hardware["arm"]
        base = hardware["base"]
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

    def on_mic_state(state: str) -> None:
        hw["마이크"] = state
        publish_hw()

    controller = VoiceController(log_hub=log_hub, on_intent=on_intent, on_mic_state=on_mic_state)
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\n[voice] 종료.")
        return
    except Exception as exc:  # noqa: BLE001 - 음성이 죽어도 대시보드는 남긴다
        # 여기 오는 건 마이크 재연결 루프로도 못 살린 경우(예: Whisper 로드 실패).
        # 프로세스를 죽이면 systemd가 재시작하며 페이지가 사라지므로, 사유를
        # 화면에 남기고 서버 스레드만 살려둔다.
        on_mic_state("down")
        msg = f"음성 인식 중단: {exc} — 대시보드는 계속 표시됩니다"
        print(f"[voice] {msg}")
        log_hub.publish({"ts": _now(), "kind": "error", "text": msg})

    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        print("\n[voice] 종료.")
