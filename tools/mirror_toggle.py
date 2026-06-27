#!/usr/bin/env python3
"""
mirror_toggle.py — SO101 리더→팔로워 "미러링" + 모션 "프리셋"을 PS2 패드/키보드로 제어.

미러링 (L1):
  ON  : 팔로워가 리더 자세를 실시간 추종
  OFF : 팔로워 토크 해제(limp)

프리셋 (R1 + △○✕□ = 슬롯 1~4):
  미러링 ON 상태에서 누르면  -> 현재 팔로워 자세를 그 슬롯에 "저장"
  미러링 OFF 상태에서 누르면 -> 저장된 자세로 팔로워가 "이동(재생, 부드럽게 보간)"
  (저장 내용은 ~/arm_presets.json 에 보관되어 재실행해도 유지)

토글/프리셋 입력 소스:
  - 아두이노 시리얼(115200): "TOGGLE" / "PRESET <n>"
  - 키보드: 'l'=토글, '1'~'4'=프리셋, 'q'=종료 (아두이노 없이 테스트용)

실행 (젯슨, lerobot venv):
  ~/lerobot/.venv/bin/python mirror_toggle.py
  자가점검(움직임 없음): ~/lerobot/.venv/bin/python mirror_toggle.py --selftest
"""
import argparse
import glob
import json
import os
import queue
import sys
import threading
import time

from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

PRESET_FILE = os.path.expanduser("~/arm_presets.json")


def pose_only(d):
    """관측/액션 dict에서 관절 위치(.pos) 키만 남긴다."""
    return {k: float(v) for k, v in d.items() if k.endswith(".pos")}


def load_presets():
    try:
        with open(PRESET_FILE) as f:
            return json.load(f)
    except Exception:  # noqa: BLE001
        return {}


def save_presets(presets):
    with open(PRESET_FILE, "w") as f:
        json.dump(presets, f, indent=2)


def reader_serial(port, baud, q, stop):
    try:
        import serial
    except ImportError:
        print("[arduino] pyserial 미설치 — 시리얼 입력 비활성")
        return
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except Exception as e:  # noqa: BLE001
        print(f"[arduino] 포트 열기 실패 {port}: {e} (키보드만 사용)")
        return
    print(f"[arduino] {port} @ {baud} 열림 — TOGGLE / PRESET 수신 대기")
    while not stop.is_set():
        try:
            line = ser.readline()
        except Exception as e:  # noqa: BLE001
            print(f"[arduino] 읽기 오류: {e}")
            break
        if not line:
            continue
        s = line.decode(errors="ignore").strip().upper()
        if s == "TOGGLE" or s == "L":
            q.put(("toggle", None))
        elif s.startswith("PRESET"):
            try:
                n = int(s.split()[1])
                if 1 <= n <= 4:
                    q.put(("preset", n))
            except (IndexError, ValueError):
                pass
    try:
        ser.close()
    except Exception:  # noqa: BLE001
        pass


def reader_keyboard(q, stop):
    for line in sys.stdin:
        if stop.is_set():
            break
        c = line.strip().lower()
        if c == "l":
            q.put(("toggle", None))
        elif c in ("1", "2", "3", "4"):
            q.put(("preset", int(c)))
        elif c == "q":
            q.put(("quit", None))
            break


def auto_arduino_port(exclude):
    cands = sorted(set(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")) - set(exclude))
    return cands[0] if cands else None


def main():
    ap = argparse.ArgumentParser(description="SO101 미러링 + 모션 프리셋")
    ap.add_argument("--leader-port", default="/dev/ttyACM0")
    ap.add_argument("--leader-id", default="tomato_leader")
    ap.add_argument("--follower-port", default="/dev/ttyACM1")
    ap.add_argument("--follower-id", default="tomato_follower")
    ap.add_argument("--arduino-port", default=None, help="생략 시 자동탐색")
    ap.add_argument("--arduino-baud", type=int, default=115200)
    ap.add_argument("--fps", type=float, default=50.0)
    ap.add_argument("--play-secs", type=float, default=1.5, help="프리셋 재생 보간 시간(초)")
    ap.add_argument("--selftest", action="store_true",
                    help="연결/읽기만 확인하고 종료 (팔로워 움직임 없음)")
    args = ap.parse_args()

    leader = SOLeader(SOLeaderTeleopConfig(port=args.leader_port, id=args.leader_id))
    follower = SOFollower(SOFollowerRobotConfig(port=args.follower_port, id=args.follower_id))

    print("연결 중...")
    leader.connect(calibrate=False)
    follower.connect(calibrate=False)
    follower.bus.disable_torque()  # 항상 limp(OFF)로 시작
    print("연결됨. (팔로워 limp)")

    presets = load_presets()
    print(f"프리셋 로드: 슬롯 {sorted(int(k) for k in presets)} ({PRESET_FILE})")

    if args.selftest:
        try:
            print("리더 자세:", {k: round(v, 1) for k, v in pose_only(leader.get_action()).items()})
            print("팔로워 자세:", {k: round(v, 1) for k, v in pose_only(follower.get_observation()).items()})
            print("selftest OK — 움직임 없음.")
        finally:
            follower.bus.disable_torque()
            leader.disconnect()
            follower.disconnect()
        return

    state = {"mirroring": False}
    q = queue.Queue()
    stop = threading.Event()

    ard = args.arduino_port or auto_arduino_port([args.leader_port, args.follower_port])
    if ard:
        threading.Thread(target=reader_serial, args=(ard, args.arduino_baud, q, stop),
                         daemon=True).start()
    else:
        print("[arduino] 포트 못 찾음 — 키보드만 사용")
    threading.Thread(target=reader_keyboard, args=(q, stop), daemon=True).start()

    def set_mirror(on):
        if on == state["mirroring"]:
            return
        state["mirroring"] = on
        if on:
            follower.bus.enable_torque()
            print(">>> 미러링 ON  (리더 → 팔로워 추종)")
        else:
            follower.bus.disable_torque()
            print(">>> 미러링 OFF (팔로워 limp)")

    def save_preset(n):
        pose = pose_only(follower.get_observation())
        presets[str(n)] = pose
        save_presets(presets)
        print(f"[프리셋 {n}] 저장됨: " + ", ".join(f"{k.split('.')[0]}={v:.0f}" for k, v in pose.items()))

    def play_preset(n):
        target = presets.get(str(n))
        if not target:
            print(f"[프리셋 {n}] 비어 있음 — 먼저 미러링 ON에서 저장하세요.")
            return
        print(f"[프리셋 {n}] 재생 (부드럽게 {args.play_secs:.1f}s 이동)")
        follower.bus.enable_torque()
        cur = pose_only(follower.get_observation())
        steps = max(2, int(args.play_secs * args.fps))
        for s in range(1, steps + 1):
            a = {k: cur.get(k, target[k]) + (target[k] - cur.get(k, target[k])) * s / steps
                 for k in target}
            follower.send_action(a)
            time.sleep(args.play_secs / steps)
        print(f"[프리셋 {n}] 도착 — 자세 유지 중(토크 ON).")

    print("준비 완료. L1=미러링토글, R1+△○✕□=프리셋(ON에서 저장/OFF에서 재생). 'q' 종료.")
    print("  키보드: l=토글, 1~4=프리셋, q=종료")
    period = 1.0 / args.fps
    try:
        while True:
            t0 = time.perf_counter()
            try:
                while True:
                    kind, n = q.get_nowait()
                    if kind == "quit":
                        raise KeyboardInterrupt
                    if kind == "toggle":
                        set_mirror(not state["mirroring"])
                    elif kind == "preset":
                        if state["mirroring"]:
                            save_preset(n)
                        else:
                            play_preset(n)
            except queue.Empty:
                pass

            if state["mirroring"]:
                follower.send_action(leader.get_action())

            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        stop.set()
        try:
            follower.bus.disable_torque()
        except Exception:  # noqa: BLE001
            pass
        try:
            leader.disconnect()
        except Exception:  # noqa: BLE001
            pass
        try:
            follower.disconnect()
        except Exception:  # noqa: BLE001
            pass
        print("정리 완료.")


if __name__ == "__main__":
    main()
