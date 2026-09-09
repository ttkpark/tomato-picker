#!/usr/bin/env python3
"""베이스를 **한 번, 짧게** 움직인다 — 원격에서 바깥 웹캠으로 확인해 가며 조금씩.

    ~/lerobot/.venv/bin/python ros2/tools/base_nudge.py --vx 130 --secs 0.6     # 앞으로 톡
    ~/lerobot/.venv/bin/python ros2/tools/base_nudge.py --w -130 --secs 0.8     # 제자리 회전

⚠ `tomato-voice`·`controller-drive`는 내린 채로(주행 보드 포트는 한 프로세스).
⚠ 포트를 열면 Uno가 리셋된다(DTR) — 첫 하트비트를 기다린 뒤에만 지령을 낸다.
⚠ 정지마찰 문턱(실측 ≈90)보다 작은 지령은 물리적으로 0이다 — 게인 말고 크기부터.
⚠ 회전 부호: 2026-09-09 실측 **w=+가 반시계(왼쪽)**, w=−가 시계 — README("시계+")와 반대.
⚠ 실측 9/9: w=±160·0.8s는 0, ±255·1.0s는 ~40°이되 **가끔 0**(정지마찰·케이블 장력).
   0.6s는 안 움직인다(슬루 램프가 다 못 올라간다). 안 움직이면 --max-pwm 4095로 한 번.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))

from tomato_picker.hardware.motor_link import MotorLink  # noqa: E402


def _val(x):
    """property든 메서드든 값으로 — MotorLink는 둘을 섞어 쓴다."""
    return x() if callable(x) else x


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vx", type=int, default=0, help="전진+ (-255..255)")
    ap.add_argument("--vy", type=int, default=0, help="우평행+")
    ap.add_argument("--w", type=int, default=0, help="회전 (README: 시계+, 실기 확인 필요)")
    ap.add_argument("--secs", type=float, default=0.5)
    ap.add_argument("--boot-wait", type=float, default=8.0)
    ap.add_argument("--max-pwm", type=int, default=0,
                    help="이 펄스 동안만 듀티 상한(P n, ≤4095)을 올린다 — 하트비트 뒤에 보낸다")
    ap.add_argument("--dither", type=int, default=0,
                    help="정지마찰 깨기: 지령 동안 vy를 ±이 크기로 4Hz 흔든다(메모리 stiction-is-the-lever)")
    args = ap.parse_args()
    if not (args.vx or args.vy or args.w):
        print("지령이 전부 0이다 — 안 움직인다.")
        return 1
    if args.secs > 3.0:
        print("한 번에 3초 넘게는 안 간다 — 사진으로 확인하며 나눠서.")
        return 1

    link = MotorLink()
    t0 = time.monotonic()
    # ⚠ `pending_setup`은 on_connect 콜백이 있을 때만 서는 플래그라 여기선 늘 False다 —
    #   그걸 믿고 바로 지령을 내면 Uno 부트로더(~2초) 안에서 전부 버려진다(2026-09-09
    #   실측: 0.6초 전진 지령이 통째로 사라졌다). **첫 하트비트(hb_age)**를 기다린다.
    while True:
        st = link.stats()
        if _val(link.connected) and st.get("hb_age") is not None and time.monotonic() - t0 > 1.0:
            break
        if time.monotonic() - t0 > args.boot_wait:
            print(f"❌ {args.boot_wait:.0f}초 안에 보드 하트비트가 안 왔다 — {st}")
            link.close()
            return 1
        time.sleep(0.1)
    if args.max_pwm:
        n = max(500, min(4095, args.max_pwm))
        ok = link.send_raw(f"P {n}")
        print(f"듀티 상한 P {n} → {'보냄' if ok else '실패'}")
        time.sleep(0.15)
    print(f"보드 준비 {time.monotonic() - t0:.1f}s · 지령 vx={args.vx} vy={args.vy} w={args.w} · {args.secs:.2f}s")
    t1 = time.monotonic()
    while time.monotonic() - t1 < args.secs:
        vy = args.vy
        if args.dither:
            # 4Hz 사각파 — 직각축을 흔들어 정지마찰을 깬다(크기를 키우면 오버슈트로 실패)
            vy += args.dither if int((time.monotonic() - t1) * 8) % 2 == 0 else -args.dither
        link.set_velocity(args.vx, max(-255, min(255, vy)), args.w)   # STALE_SEC(0.5s) 안에 계속 갱신해야 간다
        time.sleep(0.05)
    link.stop()
    time.sleep(0.4)
    st = link.stats()
    link.close()
    print("정지. 보드:", {k: st.get(k) for k in ("connected", "board_resets", "boot_report") if k in st})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
