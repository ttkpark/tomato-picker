"""바닥 카메라 라인을 유지하며 무대 앞을 좌우로 오가는 주행 제어.

코스 기하는 tools/line_follow.py 참고 — 테이프가 **진행 방향과 나란한 가로 띠**라
제어 축이 통상적 라인트레이서와 다르다:

    띠의 세로 위치(dy) → **vy** (게걸음, 테이프까지의 거리 유지)
    띠의 기울기(yaw)   → w    (회전, 테이프와 평행 유지)
    진행 방향           → **vx** (전후, 여기가 실제 이동)

⚠ 이 대응은 2026-08-09 실기에서 확정됐다. 처음엔 반대로(거리→vx, 진행→vy) 잡았는데
**테이프까지의 거리는 로봇의 횡방향 오차**라 게걸음으로 잡아야 한다 — vx(앞뒤)로는
테이프 거리를 만들 수 없다. 부호도 둘 다 반대였다.

축·부호는 카메라 장착 방향에 따라 또 달라질 수 있어 **런타임에 뒤집을 수 있게**
했다(대시보드 버튼 → LINE_TUNING_FILE에 저장 → 재시작해도 유지). 실물을 손으로
밀어보지 않고는 확정할 수 없는 값이라, 코드에 박아두면 매번 배포가 필요해진다.

**왜 voice 서비스 안에 있나** — 모터보드 포트는 MotorLink가 독점하고, 그건
tomato-voice 프로세스가 들고 있다. 별도 서비스로 빼면 포트를 못 잡는다.

**변위(오도메트리)**: 바퀴 엔코더가 없어 "몇 초 굴렸다"로는 못 믿는다(메카넘
슬립 + 배터리 전압에 따라 속도가 변함). 그래서 두 가지를 겹쳐 쓴다.
  1. 바닥 카메라 시각 오도메트리(line_follow의 odom_x_px) — 연속 변위
  2. 양끝 **검은 테이프** — 절대 기준점. 만날 때마다 원점을 다시 잡아 누적 오차를 턴다.

안전 설계 (이 로봇은 사람 앞에서 굴러간다):
  · 테이프를 놓치면 즉시 정지 — 눈 감고 달리지 않는다.
  · dy가 한계를 넘으면 정지 — 보정 **부호가 반대**여도 폭주 대신 멈춘다.
  · 명령마다 최대 시간(타임아웃) — 마커를 영영 못 만나도 언젠가 선다.
  · 수동 조작(키보드/버튼)이 들어오면 자동주행은 취소된다(cancel()).
  · 이 루프가 죽으면 MotorLink의 STALE_SEC(0.5s)와 펌웨어 데드맨이 세운다.
"""

from __future__ import annotations

import json
import os
import threading
import time

from ..config import (
    LINE_DY_AXIS,
    LINE_DY_GAIN,
    LINE_DY_SIGN,
    LINE_HUE_END,
    LINE_HUE_MID,
    LINE_LOST_STOP_SEC,
    LINE_MARK_CLEAR_FRAC,
    LINE_MAX_CORRECTION,
    LINE_MAX_DY_NORM,
    LINE_PULSE_ON,
    LINE_PULSE_PERIOD,
    LINE_SPEED,
    LINE_STATION_LABELS,
    LINE_STATUS_PATH,
    LINE_TARGET_Y_FILE,
    LINE_TIMEOUT_SEC,
    LINE_TRAVEL_SIGN,
    LINE_TUNING_FILE,
    LINE_YAW_GAIN,
    LINE_YAW_GAIN_ON,
    LINE_YAW_SIGN,
)

# 런타임에 뒤집을 수 있는 축·부호. 실기에서만 확정되는 값이라 파일로 뺀다.
TUNING_KEYS = ("dy_axis", "dy_sign", "yaw_sign", "travel_sign", "yaw_gain",
               "speed", "pulse_on", "pulse_period")


def _load_saved(path: str) -> dict:
    try:
        with open(os.path.expanduser(path), encoding="utf-8") as f:
            saved = json.load(f)
    except (OSError, ValueError):
        return {}
    out = {}
    for key in TUNING_KEYS:
        if key in saved:
            out[key] = str(saved[key]) if key == "dy_axis" else float(saved[key])
    return out


class LineDriver:
    """라인 유지 주행 + 끝단 기준 변위 추정. 명령은 전부 논블로킹."""

    RATE = 0.05          # 20Hz 제어 주기
    MARK_TOL_FRAC = 0.12  # 마커가 화면 중앙 ±이 비율 안에 오면 "도착"

    def __init__(self, base, status_path: str = LINE_STATUS_PATH) -> None:
        self._base = base
        self._status_path = status_path
        self._lock = threading.Lock()
        # 명령 상태
        self._mode = "idle"          # idle | goto_end | travel | jog | goto_color
        self._goal: dict = {}
        self._deadline = 0.0
        self._detail = "대기"
        # 변위(오도메트리) 상태
        self._origin_px: float | None = None   # 왼쪽 끝에서 latch한 odom 원점
        self._span_px: float | None = None     # 양끝 사이 거리(첫 완주에서 측정)
        self._last_end: str | None = None      # 마지막으로 원점을 잡은 끝
        # 마지막으로 읽은 라인 상태 + 계산된 지령(부호 검증용 — 정지 중에도 계산)
        self._line: dict = {}
        self._would: tuple[int, int, int] = (0, 0, 0)
        self._lost_since: float | None = None
        # 지점(스테이션) 추적 — 주황을 지나야 인덱스가 확정된다(추측하지 않는다)
        self._station: int | None = None
        self._at_end = False
        self._mark_armed = True          # 다음 마커 통과를 받을 준비가 됐나
        self._last_marker: dict | None = None
        self._last_dir = 0               # 마지막 진행 방향(+1 오른쪽 / -1 왼쪽)
        # 축·부호 — config 기본값 위에 현장에서 뒤집은 값을 얹는다.
        self._tune = {
            "dy_axis": LINE_DY_AXIS,
            "dy_sign": float(LINE_DY_SIGN),
            "yaw_sign": float(LINE_YAW_SIGN),
            "travel_sign": float(LINE_TRAVEL_SIGN),
            "yaw_gain": float(LINE_YAW_GAIN),
            "speed": float(LINE_SPEED),
            "pulse_on": float(LINE_PULSE_ON),
            "pulse_period": float(LINE_PULSE_PERIOD),
        }
        self._tune.update(_load_saved(LINE_TUNING_FILE))
        self._thread = threading.Thread(target=self._run, daemon=True, name="line-drive")
        self._thread.start()

    # ------------------------------------------------------------------
    # 명령
    # ------------------------------------------------------------------

    def goto_end(self, side: str, speed: int | None = None) -> str:
        """끝단 검은 테이프까지 라인을 유지하며 저속 주행 후 정지."""
        if side not in ("left", "right"):
            raise ValueError("side는 left/right")
        self._start("goto_end", {"side": side, "speed": self._speed(speed)},
                    f"{'왼쪽' if side == 'left' else '오른쪽'} 끝으로 이동")
        return self._detail

    def travel(self, side: str, seconds: float, speed: int | None = None) -> str:
        """시간 기반 이동 — 토마토 사이를 옮길 때(스테이션 간격이 일정하므로)."""
        seconds = max(0.05, min(20.0, float(seconds)))
        self._start("travel", {"side": side, "seconds": seconds, "speed": self._speed(speed)},
                    f"{'왼쪽' if side == 'left' else '오른쪽'}으로 {seconds:.1f}초 이동")
        return self._detail

    def jog(self, side: str, seconds: float | None = None, speed: int | None = None) -> str:
        """버튼 한 번 = **펄스 한 번**. 톡 치고 바로 멈춘다(미세 정렬용).

        기본 시간을 펄스 ON과 같게 둬서, 연타하면 같은 크기의 걸음이 반복된다.
        """
        return self.travel(side, seconds or float(self._tune.get("pulse_on", LINE_PULSE_ON)), speed)

    def set_params(self, **values) -> str:
        """속도·펄스 설정을 런타임에 바꾸고 파일에 남긴다(재시작해도 유지)."""
        limits = {"speed": (30, 255), "pulse_on": (0.02, 3.0), "pulse_period": (0.02, 3.0)}
        with self._lock:
            for key, (lo, hi) in limits.items():
                if values.get(key) is not None:
                    self._tune[key] = float(max(lo, min(hi, float(values[key]))))
            # 주기가 ON보다 짧으면 의미가 없다 — 연속 주행으로 해석한다.
            if self._tune["pulse_period"] < self._tune["pulse_on"]:
                self._tune["pulse_period"] = self._tune["pulse_on"]
            snapshot = dict(self._tune)
        self._save_tuning(snapshot)
        mode = ("연속 주행" if snapshot["pulse_period"] <= snapshot["pulse_on"]
                else f"펄스 {snapshot['pulse_on']:.2f}s / {snapshot['pulse_period']:.2f}s 주기")
        return f"속도 {snapshot['speed']:.0f} · {mode}"

    def _save_tuning(self, snapshot: dict) -> None:
        try:
            with open(os.path.expanduser(LINE_TUNING_FILE), "w", encoding="utf-8") as f:
                json.dump(snapshot, f, ensure_ascii=False, indent=2)
        except OSError as exc:
            self._detail = f"설정 저장 실패: {exc}"

    # ------------------------------------------------------------------
    # 지점(스테이션) 이동 — 주황=양끝(절대 기준), 노랑=중간(세어서 구분)
    # ------------------------------------------------------------------

    def _marker_kind(self, marker: dict) -> str | None:
        """마커 색으로 종류 판정. 'end'(주황) | 'mid'(노랑) | None(그 외)."""
        hue = marker.get("hue")
        if hue is None:
            return None
        if LINE_HUE_END[0] <= hue <= LINE_HUE_END[1]:
            return "end"
        if LINE_HUE_MID[0] <= hue <= LINE_HUE_MID[1]:
            return "mid"
        return None

    def _observe_markers(self, line: dict) -> None:
        """마커가 화면 중앙을 지나가는 순간을 잡아 지점 인덱스를 갱신한다.

        같은 마커를 두 번 세지 않으려고, 한 번 인정한 뒤에는 중앙에서 충분히
        벗어나야(LINE_MARK_CLEAR_FRAC) 다음 통과를 받는다.
        """
        markers = line.get("markers") or []
        width = line.get("width") or 1280
        center = width / 2
        near = [m for m in markers if abs(m["x"] - center) < width * self.MARK_TOL_FRAC]
        if not near:
            # 중앙에서 충분히 벗어났으면 다음 통과를 받을 준비를 한다.
            if not markers or all(abs(m["x"] - center) > width * LINE_MARK_CLEAR_FRAC
                                  for m in markers):
                self._mark_armed = True
            return
        if not self._mark_armed:
            return
        self._mark_armed = False
        marker = min(near, key=lambda m: abs(m["x"] - center))
        kind = self._marker_kind(marker)
        direction = self._last_dir
        if kind == "end":
            # 주황 = 절대 기준점. 어느 끝인지는 **진행 방향**이 알려준다
            # (오른쪽으로 가다 만났으면 오른쪽 끝). 방향을 모르면 갱신하지 않는다.
            if direction > 0:
                self._station = len(LINE_STATION_LABELS) - 1
            elif direction < 0:
                self._station = 0
            self._at_end = True
        elif kind == "mid":
            self._at_end = False
            if self._station is not None and direction:
                self._station = max(0, min(len(LINE_STATION_LABELS) - 1,
                                           self._station + direction))
        self._last_marker = {**marker, "kind": kind}

    def goto_station(self, index: int, speed: int | None = None) -> str:
        """번호로 지점 이동. 인덱스를 모르면 먼저 끝지점을 지나야 한다."""
        if not 0 <= index < len(LINE_STATION_LABELS):
            raise ValueError(f"지점 번호는 0~{len(LINE_STATION_LABELS) - 1}")
        if self._station is None:
            raise RuntimeError(
                "지금 몇 번 지점인지 모릅니다 — 노랑(중간)은 서로 구분되지 않으니 "
                "먼저 [◀ 왼쪽 끝까지] 또는 [오른쪽 끝까지 ▶]로 주황 끝지점을 한 번 "
                "지나가면 그때부터 번호가 확정됩니다."
            )
        if self._station == index:
            return f"이미 {LINE_STATION_LABELS[index]}에 있습니다"
        side = "right" if index > self._station else "left"
        self._start("station", {"side": side, "target": index, "speed": self._speed(speed)},
                    f"{LINE_STATION_LABELS[index]}(으)로 이동")
        return self._detail

    def next_station(self, side: str, speed: int | None = None) -> str:
        """색과 무관하게 **다음 마커**에서 정지 — 인덱스를 몰라도 쓸 수 있다."""
        self._start("next_mark",
                    {"side": side, "speed": self._speed(speed), "since": self._last_marker},
                    f"{'왼쪽' if side == 'left' else '오른쪽'} 다음 지점까지")
        return self._detail

    def goto_color(self, side: str, name: str | None = None, speed: int | None = None) -> str:
        """색 마커가 화면 중앙에 올 때까지 이동 후 정지(스테이션 정지)."""
        self._start("goto_color", {"side": side, "name": name, "speed": self._speed(speed)},
                    f"{'왼쪽' if side == 'left' else '오른쪽'}으로 색 마커까지"
                    + (f"({name})" if name else ""))
        return self._detail

    def cancel(self, reason: str = "정지") -> None:
        """자동주행 취소 + 즉시 정지. 수동 조작이 들어와도 이걸 부른다."""
        with self._lock:
            self._mode = "idle"
            self._goal = {}
            self._detail = reason
        try:
            self._base.hold(0, 0, 0)
        except Exception:  # noqa: BLE001 - 정지 실패해도 데드맨이 세운다
            pass

    def reset_origin(self) -> str:
        """지금 위치를 변위 0으로 잡는다(끝단을 못 쓸 때 수동 기준)."""
        with self._lock:
            self._origin_px = self._odom()
            self._last_end = "manual"
        return "현재 위치를 변위 0으로 설정"

    def set_target_y(self) -> str:
        """지금 띠 위치를 '정위치'로 저장 — 코스를 새로 깔 때마다 한 번 누른다."""
        band_y = (self._line or {}).get("band_y")
        if band_y is None:
            raise RuntimeError("테이프가 안 보입니다 — 카메라가 띠를 보게 한 뒤 누르세요")
        with open(LINE_TARGET_Y_FILE, "w") as f:
            f.write(str(band_y))
        return f"정위치 기준을 band_y={band_y:.0f}로 저장(즉시 반영)"

    # ------------------------------------------------------------------
    # 상태
    # ------------------------------------------------------------------

    def status(self) -> dict:
        line = self._line or {}
        pos = self.position_px
        return {
            "mode": self._mode,
            "detail": self._detail,
            "found": bool(line.get("found")),
            "offset_y_px": line.get("offset_y_px"),
            "angle_deg": line.get("angle_deg"),
            "band_y": line.get("band_y"),
            "end_side": (line.get("end_marker") or {}).get("side"),
            "end_x": (line.get("end_marker") or {}).get("x"),
            "color_name": (line.get("color_marker") or {}).get("name"),
            "color_x": (line.get("color_marker") or {}).get("x"),
            "odom_conf": line.get("odom_conf"),
            "position_px": None if pos is None else round(pos, 1),
            "span_px": None if self._span_px is None else round(self._span_px, 1),
            "position_pct": self.position_pct,
            "last_end": self._last_end,
            # 정지 중에도 계속 계산한다 — 로봇을 손으로 밀어보며 **부호가 맞는지**
            # 움직이기 전에 확인할 수 있다(반대면 대시보드에서 바로 뒤집는다).
            "would_vx": self._would[0],
            "would_vy": self._would[1],
            "would_w": self._would[2],
            "dy_axis": self._tune["dy_axis"],
            "travel_axis": "vx" if self._tune["dy_axis"] == "vy" else "vy",
            "dy_sign": self._tune["dy_sign"],
            "yaw_sign": self._tune["yaw_sign"],
            "travel_sign": self._tune["travel_sign"],
            "yaw_gain": self._tune["yaw_gain"],
            "station": self._station,
            "station_label": (LINE_STATION_LABELS[self._station]
                              if self._station is not None else None),
            "station_labels": list(LINE_STATION_LABELS),
            "at_end": self._at_end,
            "markers": (self._line or {}).get("markers") or [],
            "last_marker": self._last_marker,
            "speed": self._tune["speed"],
            "pulse_on": self._tune["pulse_on"],
            "pulse_period": self._tune["pulse_period"],
            "pulsing": self._tune["pulse_on"] < self._tune["pulse_period"] - 1e-6,
        }

    @property
    def position_px(self) -> float | None:
        odom = self._odom()
        if odom is None or self._origin_px is None:
            return None
        return odom - self._origin_px

    @property
    def position_pct(self) -> float | None:
        """코스 전체를 0~100%로. 양끝을 한 번씩 만나야 계산된다."""
        pos = self.position_px
        if pos is None or not self._span_px:
            return None
        return round(max(0.0, min(100.0, 100.0 * pos / self._span_px)), 1)

    # ------------------------------------------------------------------
    # 내부
    # ------------------------------------------------------------------

    def _speed(self, speed: int | None) -> int:
        base = self._tune.get("speed", LINE_SPEED) if speed is None else speed
        return int(max(30, min(255, float(base))))

    def _travel_dir(self, goal: dict) -> int:
        """지금 이 순간 진행분을 실을지 정한다 — **펄스 구동**.

        저속으로 계속 미는 대신 짧고 빠른 펄스를 반복한다. 정지마찰 근처에서
        질질 끌리다 갑자기 미끄러지는 걸 막고, 한 번에 가는 거리가 일정해진다.
        ON == 주기면 항상 True가 되어 연속 주행과 같아진다(별도 모드가 아니다).
        """
        on = float(self._tune.get("pulse_on", LINE_PULSE_ON))
        period = float(self._tune.get("pulse_period", LINE_PULSE_PERIOD))
        base = 1 if goal.get("side") == "right" else -1
        if period <= on:
            return base
        phase = (time.monotonic() - goal["started"]) % period
        return base if phase < on else 0

    def _preflight(self) -> None:
        """출발 전 점검 — 여기서 막아야 "눌렀는데 바로 멈춘다"가 안 생긴다.

        기준선이 지금 위치와 너무 어긋나 있으면(코스를 새로 깔았거나 로봇을 옮긴
        뒤 기준을 다시 안 잡은 경우) 출발하자마자 안전정지에 걸린다. 그 상태를
        '실패'로 조용히 넘기지 말고, **무엇을 눌러야 하는지**까지 알려준다.
        """
        line = self._line or {}
        if not line.get("found"):
            raise RuntimeError("테이프가 안 보입니다 — 카메라가 띠를 보는지 확인하세요")
        dy = line.get("offset_y_norm")
        if dy is not None and abs(dy) > LINE_MAX_DY_NORM:
            raise RuntimeError(
                f"기준선에서 너무 멀어 출발할 수 없습니다(dy={line.get('offset_y_px'):.0f}px). "
                "지금 거리가 맞다면 [현재 위치를 기준선으로]를 누르고, "
                "아니면 로봇을 기준 거리로 옮긴 뒤 다시 시도하세요."
            )

    def _start(self, mode: str, goal: dict, detail: str) -> None:
        self._preflight()
        if goal.get("side") in ("left", "right"):
            # 주황(끝)을 만났을 때 "어느 끝인지"는 진행 방향으로 판정한다 —
            # 출발 즉시 만날 수도 있으니 루프가 돌기 전에 미리 정해 둔다.
            self._last_dir = 1 if goal["side"] == "right" else -1
        with self._lock:
            self._mode = mode
            self._goal = goal
            self._goal["started"] = time.monotonic()
            self._deadline = time.monotonic() + LINE_TIMEOUT_SEC
            self._detail = detail
            self._lost_since = None

    def _odom(self) -> float | None:
        return (self._line or {}).get("odom_x_px")

    def _read_line(self) -> dict:
        try:
            with open(self._status_path, encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def flip(self, what: str) -> str:
        """축/부호를 런타임에 뒤집고 파일에 남긴다(재시작해도 유지)."""
        with self._lock:
            if what == "dy_axis":
                self._tune["dy_axis"] = "vx" if self._tune["dy_axis"] == "vy" else "vy"
            elif what == "yaw_enable":
                # 회전 보정 켜기/끄기 — 발산하면 즉시 끌 수 있어야 한다.
                self._tune["yaw_gain"] = 0.0 if self._tune["yaw_gain"] else float(LINE_YAW_GAIN_ON)
            elif what in ("dy_sign", "yaw_sign", "travel_sign"):
                self._tune[what] = -self._tune[what]
            else:
                raise ValueError(f"알 수 없는 항목: {what}")
            snapshot = dict(self._tune)
        self._save_tuning(snapshot)
        return (f"보정축={snapshot['dy_axis']} dy부호={snapshot['dy_sign']:+.0f} "
                f"회전보정={'켜짐' if snapshot['yaw_gain'] else '꺼짐'}"
                f"(부호{snapshot['yaw_sign']:+.0f}) 진행부호={snapshot['travel_sign']:+.0f}")

    def _command(self, line: dict, travel_dir: int, speed: int) -> tuple[int, int, int]:
        """(vx, vy, w) 지령을 만든다.

        테이프까지의 거리 오차는 **횡방향**이라 보정축(기본 vy)에 싣고, 진행은
        나머지 축에 싣는다. 둘은 항상 직교해야 하므로 한쪽이 정해지면 다른 쪽도 정해진다.
        """
        corr = w = 0
        if line.get("found"):
            dy = line.get("offset_y_norm") or 0.0
            yaw = line.get("angle_deg") or 0.0
            corr = int(max(-LINE_MAX_CORRECTION, min(LINE_MAX_CORRECTION,
                                                     self._tune["dy_sign"] * LINE_DY_GAIN * dy)))
            w = int(max(-LINE_MAX_CORRECTION, min(LINE_MAX_CORRECTION,
                                                  self._tune["yaw_sign"] * self._tune["yaw_gain"] * yaw)))
        travel = int(self._tune["travel_sign"] * travel_dir * speed)
        if self._tune["dy_axis"] == "vy":
            return travel, corr, w      # 보정=게걸음, 진행=전후
        return corr, travel, w          # 보정=전후,   진행=게걸음

    def _latch_end(self, side: str) -> None:
        """끝단 마커에 도착 — 절대 기준점으로 원점/스팬을 갱신한다.

        누적 오도메트리는 조금씩 흐르므로, 코스 끝을 만날 때마다 여기서 턴다.
        """
        odom = self._odom()
        if odom is None:
            return
        if side == "left":
            self._origin_px = odom
            self._last_end = "left"
        else:
            if self._origin_px is not None:
                span = odom - self._origin_px
                if span > 50:                      # 최소한의 유효성(잡음 방지)
                    self._span_px = span
            else:
                self._origin_px = odom             # 오른쪽부터 만난 경우
            self._last_end = "right"

    def _goal_reached(self, line: dict, goal: dict, mode: str) -> str | None:
        """도착했으면 사유 문자열, 아니면 None."""
        now = time.monotonic()
        width = line.get("width") or 1280
        tol = width * self.MARK_TOL_FRAC

        if mode in ("travel", "jog"):
            if now - goal["started"] >= goal["seconds"]:
                return f"{goal['seconds']:.1f}초 이동 완료"
            return None

        if mode == "goto_end":
            end = line.get("end_marker")
            if end and end.get("side") == goal["side"] and abs(end["x"] - width / 2) < tol:
                self._latch_end(goal["side"])
                return f"{'왼쪽' if goal['side'] == 'left' else '오른쪽'} 끝 도착(변위 기준 갱신)"
            return None

        if mode == "station":
            # _observe_markers가 통과 때마다 인덱스를 갱신한다 — 목표에 닿으면 끝.
            if self._station == goal["target"]:
                return f"{LINE_STATION_LABELS[goal['target']]} 도착"
            return None

        if mode == "next_mark":
            # 색과 무관하게 "마커 하나 지나면" 끝. 시작 시점의 통과 카운트와 비교.
            if self._last_marker is not None and self._last_marker is not goal.get("since"):
                kind = self._last_marker.get("kind")
                name = {"end": "끝지점(주황)", "mid": "중간지점(노랑)"}.get(kind, "지점")
                return f"{name} 도착" + (
                    f" — {LINE_STATION_LABELS[self._station]}" if self._station is not None else ""
                )
            return None

        if mode == "goto_color":
            color = line.get("color_marker")
            if color and (goal.get("name") in (None, "", color.get("name"))):
                if abs(color["x"] - width / 2) < tol:
                    return f"'{color.get('name')}' 스테이션 도착"
            return None
        return None

    def _run(self) -> None:
        while True:
            line = self._read_line()
            if line:
                self._line = line
            with self._lock:
                mode, goal = self._mode, dict(self._goal)

            # 정지 중에도 보정값을 계산해 둔다(부호 검증용 — 진행분은 빼고 보정만)
            self._would = self._command(self._line, 0, 0)
            # 마커 통과는 **주행 중이 아니어도** 센다(손으로 밀어도 인덱스가 따라온다)
            self._observe_markers(self._line)

            # 끝단 마커는 **주행 중이 아니어도** 지나가면 기준을 잡는다.
            end = (self._line or {}).get("end_marker")
            if end and abs(end["x"] - (self._line.get("width") or 1280) / 2) < 60:
                if self._last_end != end["side"]:
                    self._latch_end(end["side"])

            if mode == "idle":
                time.sleep(self.RATE)
                continue

            stop_reason = self._safety_check(self._line, mode)
            if stop_reason is None:
                stop_reason = self._goal_reached(self._line, goal, mode)
            if stop_reason is None and time.monotonic() > self._deadline:
                stop_reason = "시간 초과 — 목표를 못 찾아 정지"

            if stop_reason:
                self.cancel(stop_reason)
                time.sleep(self.RATE)
                continue

            speed = goal.get("speed", LINE_SPEED)
            self._last_dir = 1 if goal.get("side") == "right" else -1
            vx, vy, w = self._command(self._line, self._travel_dir(goal), speed)
            try:
                self._base.hold(vx, vy, w)
            except Exception as exc:  # noqa: BLE001 - 링크가 죽어도 루프는 살아야 함
                self.cancel(f"주행 실패: {exc}")
            time.sleep(self.RATE)

    def _safety_check(self, line: dict, mode: str) -> str | None:
        """눈 감고 달리지 않기 — 테이프를 놓쳤거나 너무 벗어나면 정지."""
        now = time.monotonic()
        if not line.get("found"):
            if self._lost_since is None:
                self._lost_since = now
            elif now - self._lost_since > LINE_LOST_STOP_SEC:
                return "테이프를 놓쳐 정지 — 카메라가 띠를 보게 한 뒤 다시 시도"
            return None
        self._lost_since = None
        dy = abs(line.get("offset_y_norm") or 0.0)
        if dy > LINE_MAX_DY_NORM:
            return (f"기준선에서 너무 벗어나 정지(dy={line.get('offset_y_px')}px) — "
                    "보정 부호(LINE_DY_SIGN)가 반대일 수 있습니다")
        return None
