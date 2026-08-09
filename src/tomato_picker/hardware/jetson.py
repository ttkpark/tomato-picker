"""젯슨(Orin) ↔ Arduino Uno 시리얼로 메카넘 베이스를 구동하는 실물 구현.

프로토콜은 firmware/mecanum_stable/mecanum_stable.ino **v2**와 일치해야 한다.
실제 링크 관리(상시 연결·재전송 스레드·체크섬·재연결)는 전부 MotorLink가
맡고, 이 클래스는 그 위에 "몇 초 동안 굴러라" 같은 **행동 수준 API**만 얹는다.

  젯슨 → "V <vx> <vy> <w>*<CRC>"  속도 지령(-255..255). 50Hz로 재전송.
  젯슨 → "S*<CRC>"                즉시 정지
  Arduino → "hb <ms> rx= bad= v=" 1초 하트비트(링크 품질)

v2에서 달라진 점 — **가감속(소프트 스타트)은 펌웨어가 한다.** 예전에는 파이썬이
0.5초 램프를 만들어 보냈는데, 젯슨이 바쁘면 그 램프 자체가 지터를 타서 오히려
전류가 튀었다. 5ms 틱을 도는 AVR이 슬루를 걸면 매끄럽고, 젯슨 지령이 한두 번
밀려도 속도가 계단지지 않는다([[battery-power-system]]의 브라운아웃 완화).

⚠ 예전 주석의 "매 호출마다 새로 연결해야 한다"는 폐기됐다 — 그 방식의 진짜
문제(호출당 2초 DTR 리셋 대기, 포트 락 경합, 데드맨 초과로 인한 주행 끊김)가
소음·끊김의 큰 축이었다. 자세한 배경은 motor_link.py 모듈 docstring 참고.
"""

from __future__ import annotations

import json
import os
import time

from ..config import (
    BASE_DRIVE_SPEED,
    BASE_MAX_PWM,
    BASE_PWM_HZ,
    BASE_SERIAL_BAUD,
    BASE_SERIAL_PORT,
    BASE_SLEW_ACCEL,
    BASE_SLEW_DECEL,
    BASE_TUNING_FILE,
)
from .base import MobileBase
from .motor_link import MotorLink

# 펌웨어에 밀어넣는 튜닝 값과, 그걸 만드는 명령. 보드가 리셋되면 전부 기본값으로
# 돌아가므로 연결이 새로 열릴 때마다(MotorLink.generation 변화) 다시 보낸다.
TUNING_KEYS = ("hz", "max_pwm", "accel", "decel")


def _load_saved_tuning(path: str) -> dict:
    """지난번에 현장에서 고른 튜닝 값. 없거나 깨졌으면 빈 dict(=config 기본값 사용)."""
    try:
        with open(os.path.expanduser(path), encoding="utf-8") as f:
            saved = json.load(f)
    except (OSError, ValueError):
        return {}
    return {k: int(v) for k, v in saved.items() if k in TUNING_KEYS and isinstance(v, (int, float))}


class JetsonBase(MobileBase):
    """PCA9685 기반 메카넘 베이스 — mecanum_stable v2의 V/S 속도 프로토콜.

    생성자는 포트를 열지 않는다(백그라운드 스레드가 붙을 때까지 재시도) —
    하드웨어가 없어도 객체 생성은 성공하고, 케이블을 꽂으면 알아서 살아난다.
    """

    def __init__(self, port: str = BASE_SERIAL_PORT, baud: int = BASE_SERIAL_BAUD) -> None:
        self.position = 0.0  # MobileBase 인터페이스 호환용 — 실제 위치추적 없음(오픈루프).
        # ⚠ 튜닝 값이 링크보다 **먼저** 있어야 한다 — MotorLink는 생성자에서
        # 스레드를 띄우고, 포트가 붙는 즉시 on_connect(_push_tuning)를 부른다.
        self._tuning = {
            "hz": BASE_PWM_HZ,
            "max_pwm": BASE_MAX_PWM,
            "accel": BASE_SLEW_ACCEL,
            "decel": BASE_SLEW_DECEL,
        }
        # 현장에서 고른 값이 config 기본값을 이긴다. 이게 없던 시절엔 서비스가
        # 재시작될 때마다(팔 재연결·배포·크래시) 주파수가 조용히 기본값으로
        # 돌아가서, 사용자가 대시보드에서 매번 다시 눌러야 했다.
        self._tuning.update(_load_saved_tuning(BASE_TUNING_FILE))
        self._tuned_generation = -1
        # 링크가 열리는 즉시 튜닝을 밀어넣는다(주행 명령을 기다리지 않는다) —
        # 안 그러면 첫 주행만 컴파일 기본값 소리·속도로 나간다.
        self._link = MotorLink(port, baud, on_connect=self._push_tuning)

    # --- 상태 ---

    @property
    def link(self) -> MotorLink:
        return self._link

    def link_stats(self) -> dict:
        return {**self._link.stats(), "tuning": dict(self._tuning)}

    @property
    def tuning(self) -> dict:
        return dict(self._tuning)

    def tune(self, **values) -> dict:
        """PWM 주파수·듀티상한·가감속을 **런타임에** 바꾼다(재플래시 불필요).

        소음과 속도는 맞바꾸는 관계라 실기에서 귀로 찾는 수밖에 없다 —
        대시보드 "모터 튜닝" 패널이 이걸 부른다. 값은 인스턴스에 남아
        보드가 리셋돼 재연결돼도 자동으로 다시 밀어넣어진다.
        """
        for key in TUNING_KEYS:
            if values.get(key) is not None:
                self._tuning[key] = int(values[key])
        self._tuning["hz"] = max(24, min(1526, self._tuning["hz"]))
        self._tuning["max_pwm"] = max(200, min(4095, self._tuning["max_pwm"]))
        self._tuning["accel"] = max(1, min(255, self._tuning["accel"]))
        self._tuning["decel"] = max(1, min(255, self._tuning["decel"]))
        self._tuned_generation = -1   # 다음 지령에서 다시 밀어넣게
        self._save_tuning()
        self._ensure_tuning()
        return dict(self._tuning)

    def _save_tuning(self) -> None:
        """고른 값을 디스크에 남긴다 — 서비스가 재시작돼도 살아남게.

        저장 실패가 주행을 막으면 안 되므로 조용히 넘어간다(다음 재시작에서
        기본값으로 돌아갈 뿐, 지금 주행에는 영향이 없다).
        """
        path = os.path.expanduser(BASE_TUNING_FILE)
        try:
            tmp = f"{path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._tuning, f)
            os.replace(tmp, path)   # 원자적 — 반쯤 쓰인 파일을 다음 부팅이 읽지 않게
        except OSError as exc:
            print(f"  [base] 튜닝 저장 실패({exc}) — 재시작하면 기본값으로 돌아간다")

    def _ensure_tuning(self) -> None:
        """연결이 (다시) 열렸으면 튜닝 값을 펌웨어에 밀어넣는다.

        보드가 리셋되면 F/P/R이 전부 컴파일 기본값으로 돌아가므로, 연결
        세대가 바뀔 때마다 재적용해야 한다 — 안 그러면 USB가 한 번 흔들린
        뒤부터 조용히 "예전 소리, 예전 속도"로 돌아가 있다.
        """
        if not self._link.connected or self._link.generation == self._tuned_generation:
            return
        self._push_tuning()

    def _push_tuning(self, link: MotorLink | None = None) -> None:
        """F/P/R을 실제로 써 보낸다. MotorLink 스레드(연결 직후)와 tune()이 부른다.

        link를 인자로 받는 이유: 연결 직후 콜백은 `self._link = MotorLink(...)`
        대입이 끝나기 전에 불릴 수 있어 self._link가 아직 없다. 그래서 링크는
        **넘겨받은 것을 우선** 쓴다.

        ⚠ `F`는 펌웨어에서 PCA9685를 재시작하며 delay(100)을 태운다 — 워치독
        250ms 안이라 안전하지만, 주행 중에 바꾸면 그 100ms는 지령이 안 먹는다.
        """
        link = link or self._link
        t = self._tuning
        ok = link.send_raw(f"F {t['hz']}")
        ok = link.send_raw(f"P {t['max_pwm']}") and ok
        ok = link.send_raw(f"R {t['accel']} {t['decel']}") and ok
        if ok:
            self._tuned_generation = link.generation

    # --- 행동 API ---

    def drive_to(self, distance: float) -> None:
        raise NotImplementedError(
            "drive_to는 이 펌웨어(mecanum_stable, 오픈루프)에서 아직 캘리브레이션되지 "
            "않았다. drive_forward(seconds)를 대신 쓸 것."
        )

    def drive_forward(self, seconds: float, speed: int = BASE_DRIVE_SPEED) -> None:
        """seconds초 동안 전진한 뒤 정지(블로킹)."""
        self.drive(seconds, vx=speed)

    def drive_backward(self, seconds: float, speed: int = BASE_DRIVE_SPEED) -> None:
        """seconds초 동안 후진한 뒤 정지(블로킹)."""
        self.drive(seconds, vx=-speed)

    def drive(self, seconds: float, vx: int = 0, vy: int = 0, w: int = 0) -> None:
        """메카넘 3자유도 지령을 seconds초 동안 유지한 뒤 정지(블로킹).

        vx=전후, vy=좌우 게걸음, w=제자리 회전. 각각 -255..255.
        재전송은 MotorLink 스레드가 알아서 하므로 여기서는 목표만 갱신하며 기다린다
        (지령이 STALE_SEC보다 오래되면 자동 정지하므로 주기적으로 새로 찍어준다).
        """
        self._ensure_tuning()
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            self._link.set_velocity(vx, vy, w)
            time.sleep(0.1)
        self._link.stop()

    def hold(self, vx: int = 0, vy: int = 0, w: int = 0) -> None:
        """지금부터 이 속도로 계속 가라(논블로킹). (0,0,0)이면 정지.

        브라우저가 키를 누르고 있는 동안 100ms마다 부른다. 갱신이 끊기면
        MotorLink가 0.5초 뒤 스스로 세우고, 그마저 못 가면 펌웨어 데드맨이 세운다.
        """
        self._ensure_tuning()
        if vx or vy or w:
            self._link.set_velocity(vx, vy, w)
        else:
            self._link.stop()

    def stop(self) -> None:
        """비상 정지 — 즉시 반환(다음 20ms 송신에 S가 실린다)."""
        self._link.stop()

    def close(self) -> None:
        self._link.close()
