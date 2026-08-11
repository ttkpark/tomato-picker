"""모터 링크 실시간 계측 — 대시보드 `/diag` 화면의 데이터 소스.

**왜 만들었나.** 기존 `/status`는 1초 폴링이라 **80ms 펄스를 볼 수 없다.**
라인 주행이 "톡톡"이 아니라 들쭉날쭉해질 때, 원인이

  · 젯슨이 펄스를 고르게 못 내보내는 것인지 (지령 쪽)
  · 보드가 그 지령을 못 받는 것인지 (링크 쪽)
  · 받고도 슬루 때문에 속도가 안 붙는 것인지 (펌웨어 쪽)

를 가르려면 **최소 펄스보다 훨씬 촘촘히** 떠야 한다. 1Hz 스냅샷으로는
셋 다 똑같이 "이상함"으로 보인다.

**무엇을 재나**

1. **지령 파형** — MotorLink가 들고 있는 목표 `(vx,vy,w)`를 100Hz로 샘플링.
   화면에 그대로 그리면 펄스가 눈에 보인다.
2. **펄스 폭** — 그 파형의 0↔비0 엣지로 ON/OFF 지속시간을 **직접** 잰다.
   `LINE_PULSE_ON=0.08`을 넣었을 때 실제로 몇 ms가 나가는지가 숫자로 나온다.
   (LineDriver 20ms 루프 + MotorLink 20ms 송신이 각각 양자화를 얹으므로
   80ms 명령이 40~120ms로 벌어질 수 있다 — 그 편차가 여기 그대로 찍힌다.)
3. **수신율** — 펌웨어 하트비트의 `rx=` 카운터 증가율(/초). 20ms 재전송이
   제대로 돌면 ~50/s. 훨씬 낮으면 젯슨 송신 스레드가 밀리는 것이다.
4. **샘플러 자신의 지연** — 이 스레드가 100Hz 스케줄에서 얼마나 밀렸는지.
   MotorLink의 20ms 틱이 밀리는 것과 같은 원인(GIL·CPU 부하)을 보므로,
   "지령이 고르지 않다"의 책임 소재를 젯슨 쪽으로 좁힐 때 쓴다.

⚠ **관측 한계 두 가지.**
  · 여기서 재는 것은 **젯슨이 내보내려는 지령**이지 보드가 실제로 인가한
    전압이 아니다. 보드 쪽 실제 속도는 1Hz 하트비트의 `v=`가 전부다.
  · 100Hz 샘플링이라 펄스 폭은 ±10ms 양자화된다. 애초에 MotorLink가 20ms
    주기라 그 이하 해상도는 원래 존재하지 않는다 — 10ms 차이를 의미 있게
    읽지 말 것. 우리가 쫓는 것은 40ms vs 120ms 수준의 편차다.
"""

from __future__ import annotations

import threading
import time
from collections import deque


class LinkSampler:
    """모터 링크를 고속으로 떠서 파형·펄스폭·증가율을 만드는 백그라운드 샘플러."""

    # MotorLink 송신 주기(20ms)의 2배. 더 올려도 20ms 이하는 못 본다.
    SAMPLE_HZ = 100
    # 증가율을 평균낼 창. 짧으면 숫자가 튀고, 길면 변화를 늦게 본다.
    RATE_WINDOW = 2.0
    # 파형 보관 길이(초). 100Hz × 20s = 2000샘플, 메모리 부담 없음.
    HISTORY_SEC = 20.0
    # 펄스 기록 개수. 화면엔 최근 16개만 쓰지만 통계용으로 더 들고 있는다.
    MAX_PULSES = 40

    def __init__(self, get_base) -> None:
        # get_base: 지금의 base 객체를 돌려주는 콜러블. 서버가 하드웨어보다
        # 먼저 뜨는 게 원칙이라(voice_mode 참고) 생성 시점엔 아직 없을 수 있다.
        self._get_base = get_base
        self._lock = threading.Lock()
        self._wave: deque = deque(maxlen=int(self.SAMPLE_HZ * self.HISTORY_SEC))
        self._pulses: deque = deque(maxlen=self.MAX_PULSES)
        self._stats: dict = {}
        self._t0 = time.monotonic()
        # 펄스 엣지 추적 상태
        self._on_since: float | None = None
        self._off_since: float | None = None
        self._gap_ms: float | None = None
        self._peak = 0
        # 스케줄 지연 최댓값(스냅샷마다 리셋 — "최근 0.1초 중 최악")
        self._lag_max = 0.0
        threading.Thread(target=self._run, daemon=True, name="link-sampler").start()

    # ------------------------------------------------------------------

    def _run(self) -> None:
        period = 1.0 / self.SAMPLE_HZ
        next_t = time.monotonic()
        while True:
            now = time.monotonic()
            slack = next_t - now
            if slack > 0:
                time.sleep(slack)
                now = time.monotonic()
            elif -slack > self._lag_max:
                # 스케줄이 밀렸다 — 얼마나 밀렸는지가 곧 젯슨 부하 지표다.
                self._lag_max = -slack
            if -slack > 1.0:
                next_t = now      # 너무 오래 밀렸으면 따라잡기 포기(폭주 방지)
            next_t += period
            try:
                self._sample(now)
            except Exception as exc:  # noqa: BLE001 - 계측 스레드는 절대 죽으면 안 된다
                # 죽으면 /diag가 영영 빈 화면이 되고, 그걸 화면만 봐선 모른다
                # (파형이 멈춘 건지 로봇이 멈춘 건지 구분이 안 간다). 에러를
                # 상태에 실어 화면에 띄우고 계속 돈다.
                with self._lock:
                    self._stats = {"connected": False, "error": f"샘플러: {exc}"}

    def _sample(self, now: float) -> None:
        try:
            base = self._get_base()
            st = base.link_stats() if base is not None else None
        except Exception as exc:  # noqa: BLE001 - 링크 조회 실패도 하나의 상태다
            st = {"connected": False, "error": str(exc)}
        if not st:
            st = {"connected": False, "error": "바퀴 미연결"}
        target = st.get("target") or (0, 0, 0)
        vx, vy, w = (int(target[0]), int(target[1]), int(target[2]))
        with self._lock:
            self._stats = st
            self._wave.append((
                round(now - self._t0, 3), vx, vy, w,
                int(st.get("fw_rx") or 0), int(st.get("fw_bad") or 0), int(st.get("nak") or 0),
            ))
            self._track_pulse(now, vx, vy, w)

    def _track_pulse(self, now: float, vx: int, vy: int, w: int) -> None:
        """0 → 비0 → 0 전이를 잡아 ON 폭과 직전 OFF 폭을 잰다.

        "구동 중"의 정의는 **세 축 중 하나라도 0이 아님**이다. 보정만 나가는
        구간(진행 0, 게걸음만)도 모터는 도니까 펄스로 세는 게 맞다.
        락은 호출자(_sample)가 이미 잡고 있다.
        """
        driving = bool(vx or vy or w)
        peak = max(abs(vx), abs(vy), abs(w))
        if driving:
            if self._on_since is None:
                self._on_since = now
                self._peak = peak
                # 직전 OFF 구간 = 이 펄스의 앞쪽 쉼. 주기 = gap + on.
                self._gap_ms = None if self._off_since is None else (now - self._off_since) * 1000
            else:
                self._peak = max(self._peak, peak)
            self._off_since = None
            return
        if self._on_since is not None:
            self._pulses.append({
                "on_ms": round((now - self._on_since) * 1000, 1),
                "gap_ms": None if self._gap_ms is None else round(self._gap_ms, 1),
                "peak": self._peak,
            })
            self._on_since = None
        if self._off_since is None:
            self._off_since = now

    # ------------------------------------------------------------------

    def snapshot(self, since: float = -1.0) -> dict:
        """SSE 한 프레임. since 이후의 새 파형만 실어 보낸다(대역폭 절약).

        브라우저가 잠깐 멈췄다 돌아와도 since가 그대로라 빠진 구간을 한꺼번에
        받는다 — 화면의 파형에 구멍이 나지 않는다.
        """
        with self._lock:
            recent = list(self._wave)
            stats = dict(self._stats)
            pulses = list(self._pulses)
            lag_max, self._lag_max = self._lag_max, 0.0
        wave = [s for s in recent if s[0] > since]
        widths = [p["on_ms"] for p in pulses]
        return {
            "t": recent[-1][0] if recent else since,
            "clock": time.strftime("%H:%M:%S"),
            "stats": stats,
            "wave": wave,
            "rates": self._rates(recent),
            "pulses": pulses[-16:],
            "pulse_stat": ({
                "n": len(widths),
                "min": min(widths),
                "max": max(widths),
                "avg": round(sum(widths) / len(widths), 1),
            } if widths else None),
            "sample_lag_ms": round(lag_max * 1000, 1),
            "sample_hz": self.SAMPLE_HZ,
            "expect_rx": 50,   # MotorLink SEND_INTERVAL_SEC=0.02 → 초당 50프레임
        }

    @classmethod
    def _rates(cls, recent: list) -> dict:
        """최근 RATE_WINDOW 동안의 카운터 증가율(/초).

        보드가 리셋되면 카운터가 0으로 되감겨 음수가 나온다 — 0으로 눌러 둔다.
        리셋 자체는 stats의 board_resets가 따로 알려주므로 정보를 잃지 않는다.
        """
        if len(recent) < 2:
            return {}
        last = recent[-1]
        cutoff = last[0] - cls.RATE_WINDOW
        first = recent[0]
        for sample in recent:
            if sample[0] >= cutoff:
                first = sample
                break
        dt = last[0] - first[0]
        if dt <= 0:
            return {}
        return {
            "rx": round(max(0, last[4] - first[4]) / dt, 1),
            "bad": round(max(0, last[5] - first[5]) / dt, 1),
            "nak": round(max(0, last[6] - first[6]) / dt, 1),
            "window": round(dt, 1),
        }
