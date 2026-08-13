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

import itertools
import threading
import time

from .config import (
    BASE_DRIVE_FORWARD_SECONDS,
    FLOOR_CAM_JPEG,
    FLOOR_LINE_STATUS,
    FLOOR_VIEW_JPEG,
    LINE_STATION_LABELS,
    USE_FLOOR_CAM,
    USE_REAL_ARM,
    USE_REAL_BASE,
    USE_REAL_CAMERA,
    USE_SHARED_VISION,
    VISION_SHM_JPEG,
    VISION_SHM_STATUS,
    VOICE_LOG_HTTP_PORT,
    VOICE_LOG_HTTP_REDIRECT_PORT,
    VOICE_PICK_SEQUENCE_KEYS,
)
from .hardware.base import MobileBase, RobotArm
from .hardware.mock import MockArm, MockBase
from .voice.controller import VoiceController
from .harvest import HarvestRunner
from .voice.intents import Intent
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


# 장비 하나를 조립할 때 기다려주는 최대 시간. 이걸 넘기면 포기하고 넘어간다.
# 팔은 서보 응답을 기다리다 굳은 전력이 있어(2026-08-12) 반드시 상한이 필요하다.
BUILD_TIMEOUT_SEC = 12.0


def _safe_build(label: str, build, fallback, log_hub: LogHub, hw: dict[str, str],
                publish=None, timeout: float = BUILD_TIMEOUT_SEC):
    """하드웨어 하나를 조립한다. 실패·지연이면 Mock으로 대체하고 상태만 기록.

    부스 데모 중 장비 하나가 빠졌다고 전체가 죽으면 안 된다 — 무엇이 빠졌는지
    페이지에 보여주고 나머지는 그대로 돌린다.

    ⚠ **시간 상한이 핵심이다.** 예외는 원래도 흡수했지만, 예외 없이 그냥 안
    돌아오는 경우(시리얼 응답 대기)는 막지 못해 서비스 시작 전체가 섰다 —
    화면엔 "장비 상태 확인 중..."만 남고 바퀴·라인·마이크까지 못 떴다
    (2026-08-12: 팔 전원이 꺼진 채 USB만 꽂혀 있었고 select()에서 2분+ 대기).
    그래서 별도 스레드에서 조립하고 **timeout 안에 안 끝나면 두고 간다.**
    (그 스레드는 계속 매달려 있을 수 있지만 데몬이라 종료를 막지 않고,
     포트를 쥔 채면 다음 재시도가 실패로 끝나 상태에 그대로 드러난다.)
    """
    hw[label] = "연결 중"
    if publish:
        publish()          # 진행 상황을 먼저 보여준다 — 침묵보다 낫다
    box: dict = {}

    def work() -> None:
        try:
            box["obj"] = build()
        except Exception as exc:  # noqa: BLE001 - 어떤 장비가 없어도 대시보드는 떠야 함
            box["err"] = exc

    thread = threading.Thread(target=work, daemon=True, name=f"build-{label}")
    thread.start()
    thread.join(timeout)

    if "obj" in box:
        hw[label] = "ok"
        if publish:
            publish()
        return box["obj"]

    if thread.is_alive():
        reason = (f"{timeout:.0f}초 안에 응답이 없어 건너뜀 — 전원/케이블을 확인하세요. "
                  "고친 뒤에는 자동으로 다시 잡습니다")
    else:
        reason = str(box.get("err", "알 수 없는 오류"))
    hw[label] = "down"
    msg = f"{label} 연결 실패: {reason} — 해당 동작은 무시됩니다(대시보드는 정상)"
    print(f"[voice] {msg}")
    log_hub.publish({"ts": _now(), "kind": "error", "text": msg})
    if publish:
        publish()
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
            # 재시도도 스레드+상한으로 — 여기서 굳으면 이후 모든 장비의 재연결이
            # 영영 멈춘다(루프가 한 바퀴를 못 돈다).
            box: dict = {}

            def work(build=build, box=box) -> None:
                try:
                    box["obj"] = build()
                except Exception:  # noqa: BLE001 - 아직 안 붙었을 뿐
                    pass

            t = threading.Thread(target=work, daemon=True, name=f"retry-{label}")
            t.start()
            t.join(BUILD_TIMEOUT_SEC)
            obj = box.get("obj")
            if obj is None:
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
    hardware: dict = {"arm": None, "base": None, "line": None, "seq": None}
    vision = _build_vision(log_hub)
    hw["카메라"] = "ok" if vision is not None else "down"
    # 바닥 카메라(CSI) — line-cam.service가 /dev/shm에 쓰는 프레임을 읽기만 한다.
    # count 루프는 안 돌린다(start() 안 함 — 라인캠엔 개수 이벤트가 없다).
    floor = None
    if USE_FLOOR_CAM:
        floor = SharedFrameSource(
            log_hub, FLOOR_CAM_JPEG, FLOOR_LINE_STATUS, prefer_path=FLOOR_VIEW_JPEG
        )
        floor.start_line_publisher()
    start_log_server(
        log_hub, port=VOICE_LOG_HTTP_PORT, vision=vision, hardware=hardware, floor=floor
    )
    start_redirect_server(VOICE_LOG_HTTP_REDIRECT_PORT, VOICE_LOG_HTTP_PORT)
    print(f"[voice] 실시간 대시보드: http://<젯슨IP>:{VOICE_LOG_HTTP_PORT}  (Ctrl+C로 종료)")
    print(f"[voice] 수동 조작:      http://<젯슨IP>:{VOICE_LOG_HTTP_PORT}/control")

    # 2) 하드웨어는 실패해도 Mock으로 대체 — 상태는 페이지 상단 배지로.
    # ★ 배지 발행 함수를 **조립보다 먼저** 정의해 각 단계마다 갱신한다.
    #   예전엔 조립이 다 끝난 뒤에야 처음 발행해서, 한 장비가 늦으면 화면이
    #   "장비 상태 확인 중..."에 멈춘 채 아무 정보도 안 줬다(2026-08-12).
    def publish_hw() -> None:
        log_hub.publish({"ts": _now(), "kind": "hw", "items": dict(hw)}, latest_only=True)

    hw["로봇팔"] = hw["바퀴"] = "대기"
    hw["마이크"] = "대기"
    publish_hw()
    hardware["arm"] = _safe_build("로봇팔", _build_arm, MockArm, log_hub, hw, publish_hw)
    hardware["base"] = _safe_build("바퀴", _build_base, MockBase, log_hub, hw, publish_hw)
    # 3) 라인 주행 — 바닥 카메라 상태를 읽어 바퀴를 몬다. 모터보드 포트를 독점하는
    #    MotorLink가 이 프로세스에 있으므로 제어 루프도 여기 있어야 한다.
    if USE_FLOOR_CAM and hardware["base"] is not None:
        try:
            from .hardware.line_drive import LineDriver

            hardware["line"] = LineDriver(hardware["base"])
        except Exception as exc:  # noqa: BLE001 - 라인 주행이 없어도 나머지는 돈다
            print(f"[voice] 라인 주행 초기화 실패: {exc} — 수동 조작은 정상")
    # 4) 시퀀스 러너 — 팔 프리셋과 지점 이동을 섞은 대본을 돌린다. 가변
    #    hardware 딕셔너리를 그대로 넘겨서, 재연결로 팔/라인이 교체돼도 따라간다.
    from .hardware.sequence import SequenceRunner

    hardware["seq"] = SequenceRunner(hardware)
    # 5) 수확 계획 — 비전 좌표를 나무/높이로 읽어 다음 동작을 정하고, 자동 모드면
    #    스스로 실행한다. 비전이 없어도(None) "없음"으로 답할 뿐 죽지 않는다.
    hardware["harvest"] = HarvestRunner(hardware, vision, log_hub)

    hw["마이크"] = "확인 중"
    publish_hw()
    # 끊긴 장비를 백그라운드에서 계속 다시 잡는다 — 케이블만 다시 꽂으면 복구.
    threading.Thread(
        target=_retry_hardware_forever, args=(log_hub, hw, hardware, publish_hw), daemon=True
    ).start()

    # 인텐트 처리는 항상 hardware 딕셔너리에서 **지금** 핸들을 꺼낸다 —
    # 재연결로 객체가 교체돼도 옛 객체를 붙들고 있지 않게.
    def say(text: str) -> None:
        print(f"[voice] {text}")
        log_hub.publish({"ts": _now(), "kind": "status", "text": text})

    def on_intent(intent: Intent) -> None:
        """인텐트 하나를 실행한다. 이 함수는 인텐트 워커 스레드에서 **직렬로** 불린다.

        주행이 걸린 명령(지점 이동·수확)은 전부 SequenceRunner에 태운다 —
        도착까지 기다리기, 정지 버튼, 진행 표시를 다시 만들지 않고 얻고,
        "수확 도는 중에 음성으로 이동"이 러너의 잠금 하나로 막힌다.
        """
        arm = hardware["arm"]
        base = hardware["base"]
        seq = hardware["seq"]
        line = hardware["line"]

        if intent.name == "arm_move":
            say("'팔 움직여' 인식 → 데모 동작 재생")
            arm.demo_move()
        elif intent.name == "drive_forward":
            say(f"'앞으로 가' 인식 → {BASE_DRIVE_FORWARD_SECONDS:.1f}초 전진")
            base.drive_forward(BASE_DRIVE_FORWARD_SECONDS)
        elif intent.name == "station_move":
            index = int(intent.slots["station"])
            if line is None:
                say("지점 이동 불가 — 라인 주행이 비활성입니다(바닥 카메라 확인)")
                return
            say(f"{intent.label} → {LINE_STATION_LABELS[index]}")
            say(seq.run_text(intent.label, f"m{index}"))
        elif intent.name == "harvest_all":
            harvest = hardware["harvest"]
            plan = harvest.plan()
            say(f"{intent.label} — 지금 {plan['count']}개 보입니다 · 다음: {plan['next']}")
            say(harvest.start("모두 따기"))
        elif intent.name == "tomato_pick":
            key = VOICE_PICK_SEQUENCE_KEYS[intent.slots["height"]]
            if line is not None:
                say(f"{intent.label} → 시퀀스 '{key}'")
                say(seq.start(key))
                return
            # 라인 주행이 없으면 바구니까지 못 간다. 그렇다고 통째로 포기하지 않고
            # **첫 지점 이동 앞까지만** 돌린다 — 따서 들고 있는 데까지는 보여준다.
            # (m을 빼고 뒤까지 다 돌리면 허공에 놓기 동작을 해 토마토를 흘린다.)
            text = seq.status()["sequences"].get(key, {}).get("text") or ""
            head = list(itertools.takewhile(lambda t: not t.lower().startswith("m"), text.split()))
            if not head:
                say(f"{intent.label} 불가 — 라인 주행이 비활성이고 '{key}' 대본에 팔 동작이 없습니다")
                return
            say(f"{intent.label} — 라인 주행이 없어 바구니 이동은 건너뛰고 따기까지만 합니다")
            say(seq.run_text(intent.label, " ".join(head)))

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
