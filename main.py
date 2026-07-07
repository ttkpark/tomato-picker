"""토마토 따기 로봇 진입점.

  python main.py demo                      # 오프라인 자동 수확 시퀀스 (망/키 불필요)
  python main.py run "익은 토마토 다 따줘"   # Claude 오케스트레이션 (ANTHROPIC_API_KEY 필요)
  python main.py voice                     # 음성 명령("팔 움직여") + 실시간 인식 로그(젯슨 전용)

현재는 Mock 하드웨어로 동작한다. 실제 장비가 준비되면 build_robot()에서
하드웨어 구현만 교체하면 된다.
"""

from __future__ import annotations

import argparse
import sys

# Windows 콘솔(cp949)에서도 한글/기호가 깨지지 않도록 UTF-8로 출력한다.
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8")

from dotenv import load_dotenv

from src.tomato_picker.demo_mode import run_demo
from src.tomato_picker.hardware.mock import MockArm, MockBase, MockCamera
from src.tomato_picker.skills import TomatoPicker


def build_robot() -> TomatoPicker:
    """하드웨어를 조립한 로봇을 만든다. 실제 장비 교체 지점."""
    from src.tomato_picker.config import USE_REAL_ARM, USE_REAL_BASE, USE_REAL_CAMERA

    if USE_REAL_BASE:
        # 실물 메카넘 베이스(젯슨↔Arduino 시리얼). pyserial 필요.
        from src.tomato_picker.hardware.jetson import JetsonBase

        base = JetsonBase()
    else:
        base = MockBase()

    if USE_REAL_ARM:
        # 실물 SO-101 Follower(프리셋 재생). lerobot 필요, 젯슨 전용.
        from src.tomato_picker.hardware.arm import LerobotArm

        arm = LerobotArm()
    else:
        arm = MockArm()

    if USE_REAL_CAMERA:
        # 베이스 전면 고정 USB 카메라(MJPG). 젯슨 전용.
        from src.tomato_picker.hardware.camera import JetsonCamera

        camera = JetsonCamera()
    else:
        # MockCamera는 base.position만 읽으므로 Mock/Jetson 어느 베이스든 동작한다.
        camera = MockCamera(base)

    return TomatoPicker(base=base, arm=arm, camera=camera)


def main() -> int:
    load_dotenv()
    parser = argparse.ArgumentParser(description="토마토 따기 로봇")
    sub = parser.add_subparsers(dest="mode", required=True)
    sub.add_parser("demo", help="오프라인 자동 수확 시퀀스")
    run_p = sub.add_parser("run", help="Claude 자연어 명령 실행")
    run_p.add_argument("command", help='예: "익은 토마토 다 따줘"')
    sub.add_parser("voice", help='음성 명령("팔 움직여") + 실시간 인식 로그')
    args = parser.parse_args()

    if args.mode == "voice":
        from src.tomato_picker.voice_mode import run_voice

        run_voice()
        return 0

    robot = build_robot()

    if args.mode == "demo":
        run_demo(robot)
        return 0

    # online 경로: 실패 시 데모 모드로 폴백 (부스 와이파이 대비)
    try:
        from src.tomato_picker.orchestrator import run_command

        print(run_command(robot, args.command))
        return 0
    except Exception as exc:  # noqa: BLE001 - 폴백이 목적
        print(f"[경고] 온라인 실행 실패 ({exc}). 데모 모드로 폴백합니다.\n", file=sys.stderr)
        run_demo(robot)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
