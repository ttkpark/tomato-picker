"""STM32 주행보드 ↔ 젯슨 링크 자체검사 (PC에서, 보드 없이).

    python tools/stm32_link_check.py

확인하는 것:
- MotorLink가 **하트비트 토큰(estop=)으로** STM32/Uno를 알아본다(STM32는 포트를 열어도 리셋되지 않아 배너가 안 온다).
- estop/rly/lowbat/vin/vm 상태와 이벤트 줄(estop·weld·rlyopen…)을 화면용으로 들고 있다.
- estop_reset()이 체크섬 붙은 `X 0`을 보내고 목표를 0으로 지운다(풀리는 순간 옛 목표로 튀지 않게).
- JetsonBase가 PWM 주파수를 **보드별 칸**에 둔다: Uno ≤1526Hz(PCA9685), STM32 기본 20kHz·≤40kHz.
"""
from __future__ import annotations

import os
import sys
import tempfile
import threading
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from tomato_picker.hardware import jetson                      # noqa: E402
from tomato_picker.hardware.motor_link import MotorLink, _framed  # noqa: E402

FAILS: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  ok  " if ok else "  FAIL") + f" {name}" + (f" — {detail}" if detail and not ok else ""))
    if not ok:
        FAILS.append(name)


class FakeSerial:
    def __init__(self, rx: bytes = b""):
        self.rx, self.tx = rx, b""

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def read(self, n: int) -> bytes:
        out, self.rx = self.rx[:n], self.rx[n:]
        return out

    def write(self, b: bytes) -> int:
        self.tx += b
        return len(b)

    def flush(self) -> None:
        pass


def make_link(rx: bytes) -> MotorLink:
    with mock.patch.object(threading.Thread, "start", lambda self: None):   # 재전송 스레드 없이
        link = MotorLink(port="TEST", on_connect=lambda l: None)
    link._ser, link._connected = FakeSerial(rx), True
    return link


STM_HB = (b"hb 1000 rx=5 bad=0 i2c=0 wdt=0 v=0,0,0 enc=0,0,0,0 vin=7800 vm=0 v5=5010 v33=3298 amp=0 ampw=0,0,0,0 "
          b"flt=0 fltn=0 spd=0,0,0,0 estop=2 lowbat=0 clf=0 rly=0\n")
UNO_HB = b"hb 1000 rx=5 bad=0 i2c=0 wdt=0 v=0,0,0\n"

# --- MotorLink: 보드 판별 · 상태 · 이벤트 ---
link = make_link(STM_HB + b"estop 0\n")
link._drain_input()
check("STM32 하트비트 → board=stm32", link.board == "stm32", str(link.board))
check("estop=2(풀렸지만 잠김) 보관", link.stats()["fw_state"].get("estop") == "2", str(link.stats()["fw_state"]))
fs = link.stats()["fw_state"]
check("vin/vm/rly 보관", fs.get("vin") == "7800" and fs.get("rly") == "0")
check("계통 전압 v5/v33 보관", fs.get("v5") == "5010" and fs.get("v33") == "3298", str(fs))
check("이벤트 줄 'estop 0' 기록", any(e.endswith("estop 0") for e in link.stats()["events"]), str(link.stats()["events"]))
check("하트비트가 연결로 인정", link.stats()["hb_age"] is not None)

link.set_velocity(120, 0, 0)
ok = link.estop_reset()
check("estop_reset → 체크섬 붙은 'X 0' 송신", ok and link._ser.tx.endswith(_framed("X 0")), repr(link._ser.tx))
check("estop_reset → 목표 0으로(풀리는 순간 튀지 않게)", link._target == (0, 0, 0), str(link._target))

uno = make_link(UNO_HB)
uno._drain_input()
check("Uno 하트비트 → board=uno", uno.board == "uno", str(uno.board))
check("Uno는 fw_state 비어 있음", uno.stats()["fw_state"] == {})

# --- Uno v2.3 거리 이동 M ---
mv = make_link(b"")
mid = mv.move(500, 120, 0, 0)
mv._tick()
check("move → 다음 송신에 체크섬 붙은 M", mv._ser.tx.endswith(_framed(f"M {mid} 500 120 0 0")), repr(mv._ser.tx))
mv._ser.tx = b""
mv._tick()
check("이동 중엔 V 대신 H(V는 보드가 abort로 받는다)", mv._ser.tx == _framed("H"), repr(mv._ser.tx))
mv._ser.rx = (f"ok M {mid} 500\n".encode() + UNO_HB.replace(b"\n", f" vcc=4980 vmin=4710 sag=1 mv={mid},120\n".encode()))
mv._drain_input()
st = mv.move_status()
check("ok M → run, hb mv= → got", st["state"] == "run" and st["got"] == 120, str(st))
check("hb vcc/vmin/sag 보관", mv.stats()["fw_state"].get("vmin") == "4710" and mv.stats()["fw_state"].get("sag") == "1")
mv.set_velocity(0, 0, 0)
check("set_velocity(0)은 이동을 안 끊는다", mv.move_status()["state"] == "run")
mv._ser.rx = f"mv done {mid} 503 500 910\n".encode()
mv._drain_input()
st = mv.move_status()
check("mv done → 보드 숫자가 최종(got=503)", st["state"] == "done" and st["got"] == 503, str(st))
mv._ser.tx = b""
mv._tick()
check("끝나면 다시 V", mv._ser.tx.startswith(b"V "), repr(mv._ser.tx))

mid = mv.move(500, 120, 0, 0); mv._tick()
mv.set_velocity(100, 0, 0)
check("0 아닌 set_velocity → abort", mv.move_status()["state"] == "abort")
mv._ser.tx = b""; mv._tick()
check("취소 뒤 V가 나간다", mv._ser.tx.startswith(b"V 100"), repr(mv._ser.tx))

mid = mv.move(500, 120, 0, 0); mv._tick()
mv._ser.rx = (f"ok M {mid} 500\n".encode() + b"boot #3 warm cause=EXT|BOD last=I2C i2cerr=0 near=0 mcusr=0x0 r2=0xAF\n"
              + f"mv lost {mid} 287 500\n".encode() + b"mecanum_stable v2.3 ready.\n")
mv._drain_input()
st = mv.move_status()
check("브라운아웃: mv lost → lost, got=287(리셋 직전 기록)", st["state"] == "lost" and st["got"] == 287, str(st))
check("재부팅 원인 EXT|BOD 누적", mv.stats()["boot_causes"].get("EXT|BOD") == 1, str(mv.stats()["boot_causes"]))

mv._fw_ms = 5000
mid = mv.move(500, 120, 0, 0); mv._tick()
mv._ser.rx = UNO_HB.replace(b"hb 1000", b"hb 900")   # 배너를 놓쳐도 uptime 되감기로 안다
mv._drain_input()
check("mv lost 없이 재부팅(cold) → lost", mv.move_status()["state"] == "lost", str(mv.move_status()))

mid = mv.move(500, 0, 0, 0); mv._tick()
mv._ser.rx = b"nak mv blind\n"; mv._drain_input()
check("nak mv → rejected", mv.move_status()["state"] == "rejected", str(mv.move_status()))
mid = mv.move(500, 120, 0, 0); mv._tick()
mv._mv["t"] -= 2.0; mv._tick()
check("ok M 없음 1초 → rejected(옛 펌웨어)", mv.move_status()["state"] == "rejected", str(mv.move_status()))

calls: list[tuple] = []
results = iter([{"state": "lost", "got": 300, "why": "reset"}, {"state": "done", "got": 205, "why": ""}])
mv._last_hb, mv._pending_on_connect = __import__("time").monotonic(), False
with mock.patch.object(mv, "move", lambda *a: calls.append(a) or 1), \
        mock.patch.object(mv, "_wait_move", lambda t: next(results)), mock.patch("time.sleep"):
    r = mv.drive_ticks(500, 200, 0, 0)
check("drive_ticks: 재부팅 뒤 남은 200만 다시", calls[1][0] == 200, str(calls))
check("다시 갈 땐 속도 70%(200→140)", calls[1][1] == 140, str(calls))
check("합계 got=505 · state=done · resumes=1", r["got"] == 505 and r["state"] == "done" and r["resumes"] == 1, str(r))
calls.clear()
results = iter([{"state": "stall", "got": 40, "why": "wheel=D"}])
with mock.patch.object(mv, "move", lambda *a: calls.append(a) or 1), \
        mock.patch.object(mv, "_wait_move", lambda t: next(results)), mock.patch("time.sleep"):
    r = mv.drive_ticks(500, 200, 0, 0)
check("stall은 다시 안 간다", len(calls) == 1 and r["state"] == "stall", str(r))

reopened = make_link(STM_HB)
reopened._drain_input()
reopened._board, reopened._fw_state = None, {}   # _open()이 하는 초기화와 같은 효과
check("재연결 전엔 보드 모름(None)", reopened.board is None)

# --- JetsonBase: 보드별 PWM 주파수 ---
with tempfile.TemporaryDirectory() as d, \
        mock.patch.object(jetson, "BASE_TUNING_FILE", os.path.join(d, "base_tuning.json")), \
        mock.patch.object(threading.Thread, "start", lambda self: None):
    base = jetson.JetsonBase(port="TEST")
    fake = FakeSerial()
    base._link._ser, base._link._connected = fake, True

    base._link._board = "stm32"
    check("STM32 기본 주파수 20kHz", base.tuning["hz"] == 20000, str(base.tuning))
    t = base.tune(hz=30000)
    check("STM32: hz → hz_stm32 칸(30000), Uno 칸 그대로", t["hz"] == 30000 and base._tuning["hz"] == jetson.BASE_PWM_HZ, str(base._tuning))
    base.tune(hz=99999)
    check("STM32 상한 40000", base.tuning["hz"] == 40000, str(base.tuning))
    fake.tx = b""
    base._push_tuning()
    check("STM32에 F 40000을 민다", _framed("F 40000") in fake.tx, repr(fake.tx))

    base._link._board = "uno"
    check("Uno 화면 주파수 = Uno 칸", base.tuning["hz"] == jetson.BASE_PWM_HZ, str(base.tuning))
    base.tune(hz=20000)
    check("Uno 상한 1526(PCA9685)", base.tuning["hz"] == 1526, str(base.tuning))
    fake.tx = b""
    base._push_tuning()
    check("Uno엔 F 1526을 민다(20000을 보내면 무시당한다)", _framed("F 1526") in fake.tx, repr(fake.tx))
    check("튜닝 파일은 임시 폴더에만", os.path.exists(os.path.join(d, "base_tuning.json")))

print(f"\n{'FAIL ' + str(len(FAILS)) if FAILS else '전부 통과'} ({len(FAILS)}건 실패)")
sys.exit(1 if FAILS else 0)
