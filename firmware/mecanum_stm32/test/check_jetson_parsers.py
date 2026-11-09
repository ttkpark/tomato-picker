"""펌웨어 출력(test_core dump)을 젯슨의 **실제 파서**에 넣어 본다 — motor_link(지금 쓰는 길)와 ROS board_contract.

    python firmware/mecanum_stm32/test/check_jetson_parsers.py firmware/mecanum_stm32/build/dump.txt
보드를 만들기 전에 '젯슨이 이 보드를 알아보고, 하트비트·이동·이벤트를 읽는가'를 확인한다.
"""
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path[:0] = [os.path.join(REPO, "src"), os.path.join(REPO, "ros2", "src", "tomato_bridge")]

fails = 0


def check(ok, msg):
    global fails
    print(("ok   " if ok else "FAIL ") + msg)
    fails += not ok


text = open(sys.argv[1], encoding="utf-8", errors="replace").read()
lines = [ln for ln in text.split("\n") if ln.strip()]

# ---- motor_link: 시리얼만 가짜로 ----
from tomato_picker.hardware import motor_link as ml  # noqa: E402


class FakeSer:
    def __init__(self, data: bytes):
        self.buf = data

    @property
    def in_waiting(self):
        return len(self.buf)

    def read(self, n):
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def write(self, b):
        return len(b)

    def flush(self):
        pass


link = ml.MotorLink.__new__(ml.MotorLink)        # 스레드·포트 없이 상태만
ml.MotorLink.__init__.__globals__  # noqa: B018
import threading  # noqa: E402

link._lock = threading.Lock()
for k, v in dict(_on_connect=None, _closing=True, _rx_buf=b"", _last_hb=0.0, _fw_ms=0, _rx_ok=0, _rx_bad=0, _nak=0,
                 _i2c_err=0, _wdt_near=0, _boot_report=None, _boot_causes={}, _acks=[], _board=None, _fw_state={},
                 _events=[], _pending_on_connect=False, _last_error=None, _generation=0, _mv=None, _mv_send=None,
                 _board_resets=0).items():
    setattr(link, k, v)
link._mv = {"id": 3, "target": 600, "got": 0, "state": "sent", "why": "", "t": 0.0}
link._ser = FakeSer(text.encode())
link._drain_input()
check(link._board == "stm32", f"motor_link가 보드를 stm32로 안다 ({link._board})")
check(link._boot_report and link._boot_report.startswith("boot #1 cold cause=POWER"), "boot 줄을 받았다")
check(link._fw_state.get("estop") in ("0", "2"), f"hb estop= 읽음 ({link._fw_state.get('estop')})")
check("vin" in link._fw_state and "vmin" in link._fw_state and "enc" in link._fw_state, "hb vin·vmin·enc 읽음")
check(link._mv and link._mv["state"] == "done" and link._mv["got"] >= 600,
      f"M → ok M → mv done (상태 {link._mv})")
check(any("estop 1" in e for e in link._events) and any("estop 0" in e for e in link._events), "이벤트 estop 1/0")
check("ok P 4095" in link._acks, f"ok 확인 응답 보관 ({link._acks})")
# 이 덤프는 부팅 배너부터 들어 있다 — 실기에선 STM32가 포트를 열어도 리셋되지 않아 첫 배너를 안 본다. 그 1번만 허용
check(link._board_resets == 1 and "nak" not in (link._last_error or ""), f"재부팅 오검출 없음 ({link._board_resets}, {link._last_error})")

# ---- ROS board_contract ----
from tomato_bridge import board_contract as bc  # noqa: E402

p = bc.ProtocolParser(expected_proto=2)
caps, hbs = [], []
for ln in lines:
    r = p.feed_line(ln)
    if r.kind == "cap":
        caps.append(r.cap)
    elif r.kind == "hb":
        hbs.append(r.hb)
check(len(caps) == 2 and not p.proto_mismatch, f"cap proto=2 두 번(부팅·Q) — {len(caps)}")
check(caps and caps[0].units and not caps[0].calib, "부팅 cap: units=1 calib=0")
check(caps and caps[-1].calib and caps[-1].vmax_mms > 0 and caps[-1].wmax_mdegs > 0 and caps[-1].board_id == "A3F2C918",
      f"캘리브레이션 뒤 cap: calib=1 vmax={caps[-1].vmax_mms if caps else '?'} wmax={caps[-1].wmax_mdegs if caps else '?'} id")
check(len(hbs) >= 5, f"hb {len(hbs)}개(덤프 ≈6.7초)")
hb_c = [h for h in hbs if h.act is not None and h.tgt == (250, 0, 0)]
check(hb_c and abs(hb_c[-1].act[0] - 250) < 25, f"C 250 → hb tgt=250 act≈250 ({hb_c[-1].act if hb_c else None})")
check(any(h.estop_latched for h in hbs) and hbs[-1].estop_latched is False, "st 비트0 = E-stop 래치(풀린 뒤 0)")
check(all(h.calib_valid for h in hbs[-3:]), "st 비트3 = 캘리브레이션 유효")
check(hbs[-1].vin_mv and 11500 < hbs[-1].vin_mv < 12500, f"hb vin_mv ({hbs[-1].vin_mv})")
check(p.nak_counts.get("crc", 0) == 0, "펌웨어 출력에 '*'가 없어 체크섬으로 오인되지 않는다")
print("통과" if not fails else f"실패 {fails}개")
sys.exit(1 if fails else 0)
