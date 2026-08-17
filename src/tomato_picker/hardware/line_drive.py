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
    LINE_ALIGN_DITHER,
    LINE_ALIGN_DIVERGE_LIMIT,
    LINE_ALIGN_DY_ABOVE_PX,
    LINE_ALIGN_DY_BELOW_PX,
    LINE_ALIGN_TIMEOUT_SEC,
    LINE_ALIGN_YAW_TOL,
    LINE_APPROACH_MIN,
    LINE_ARRIVE_ALIGN,
    LINE_CORR_MIN,
    LINE_DY_AXIS,
    LINE_DY_DEADBAND,
    LINE_DY_GAIN,
    LINE_DRIVE_SMOOTH,
    LINE_DY_SIGN,
    LINE_END_SIDE_MARGIN,
    LINE_HUE_END,
    LINE_HUE_MID,
    LINE_MARKER_STATION,
    LINE_LOST_STOP_SEC,
    LINE_MARK_ALIGN_TIMEOUT_SEC,
    LINE_MARK_ALIGN_TOL_PX,
    LINE_MARK_BREAK_CROSS,
    LINE_MARK_BREAK_MAG,
    LINE_MARK_BREAK_SEC,
    LINE_MARK_CLEAR_FRAC,
    LINE_MARK_REARM_MIN_PX,
    LINE_MARK_SETTLE_SEC,
    LINE_MARK_STALL_LIMIT,
    LINE_MARK_PULSE_CURVE,
    LINE_MARK_PULSE_MAX,
    LINE_MARK_PULSE_MIN,
    LINE_MARK_REARM_PX,
    LINE_MAX_CORRECTION,
    LINE_MAX_DETOURS,
    LINE_MAX_DY_NORM,
    LINE_NO_STRAFE,
    LINE_PULSE_ON,
    LINE_PULSE_PERIOD,
    LINE_SHAKE_ONE_WAY_PX,
    LINE_SMOOTH_CORR_MAX,
    LINE_SMOOTH_DY_GAIN,
    LINE_SMOOTH_SPEED,
    LINE_SMOOTH_YAW_GAIN,
    LINE_SPEED,
    LINE_STATION_LABELS,
    LINE_STATUS_PATH,
    LINE_STRAFE_FIX_AT,
    LINE_TARGET_Y_FILE,
    LINE_TIMEOUT_SEC,
    LINE_TRAVEL_KICK,
    LINE_TRAVEL_KICK_SEC,
    LINE_TRAVEL_SIGN,
    LINE_TRAVEL_WIGGLE,
    LINE_TRAVEL_WIGGLE_ON,
    LINE_TRAVEL_WIGGLE_PERIOD,
    LINE_TUNING_FILE,
    LINE_STOP_EACH_STATION,
    LINE_WIGGLE_SIGN,
    LINE_WIGGLE_YAW,
    LINE_YAW_DEADBAND,
    LINE_YAW_DEG_PER_UNIT,
    LINE_YAW_GAIN,
    LINE_YAW_GAIN_ON,
    LINE_YAW_PULSE_MAX,
    LINE_YAW_PULSE_MIN,
    LINE_YAW_SIGN,
    LINE_YAW_STICTION,
)

def _pulse_for_px(want_px: float) -> int:
    """"이만큼 가고 싶다" → 진행축 펄스 크기. 실측 곡선(LINE_MARK_PULSE_CURVE)의 역함수.

    곡선은 심하게 비선형이다(90→9px, 110→36px, 130→96px). 비례게인 하나로는
    아래쪽에서 문턱을 못 넘고 위쪽에서 두 배씩 지나친다 — 실제로 그렇게 망가졌다.
    구간별 선형보간으로 **목표 거리에 맞는 크기**를 고르고, 곡선 밖은 양끝으로 죈다.
    """
    pts = LINE_MARK_PULSE_CURVE
    if want_px <= pts[0][1]:
        return LINE_MARK_PULSE_MIN
    for (m0, d0), (m1, d1) in zip(pts, pts[1:]):
        if want_px <= d1:
            t = (want_px - d0) / max(1e-6, d1 - d0)
            return int(round(m0 + t * (m1 - m0)))
    return LINE_MARK_PULSE_MAX


# 런타임에 뒤집을 수 있는 축·부호. 실기에서만 확정되는 값이라 파일로 뺀다.
TUNING_KEYS = ("dy_axis", "dy_sign", "yaw_sign", "travel_sign", "yaw_gain",
               "speed", "pulse_on", "pulse_period", "no_strafe", "odom_sign",
               "align_dither", "smooth", "smooth_speed",
               "travel_kick", "travel_wiggle", "wiggle_sign", "wiggle_yaw",
               "stop_each",
               # 정렬 "완료" 목표 범위 — 현장에서 코스를 새로 깔 때마다 달라진다.
               "align_tol_x", "align_dy_below", "align_dy_above", "align_yaw_tol",
               # 게걸음 크기. corr_min=정렬(펄스) 전용 바닥값,
               # corr_max=정렬·주행 공통 상한, smooth_dy_gain=주행 중 비례이득.
               "corr_min", "corr_max", "smooth_dy_gain")

# 자동으로 정렬을 끼워 넣을 주행 모드. jog는 버튼 한 번 = 펄스 한 번이라
# 중간에 정렬이 끼어들면 "톡 쳤는데 로봇이 혼자 움직인다"가 되므로 뺀다.
DETOUR_MODES = ("goto_end", "travel", "station", "next_mark", "goto_color")
# 도착 뒤 지점 정렬(마커 중앙)로 마무리할 모드. travel/jog는 "시간만큼 가라"는
# 수동 미세조정이라 마커와 무관하다 — 끼어들면 오히려 방해다.
ARRIVE_ALIGN_MODES = ("goto_end", "station", "next_mark", "goto_color")


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

    # 50Hz. 20Hz(50ms)였을 때는 짧은 펄스를 제대로 못 그렸다 — ON 0.08초를 넣어도
    # 틱 경계에 걸려 0.05나 0.10초로 반올림되고, 어떤 주기에는 ON/OFF가 붙어
    # "톡톡"이 "주우웅"으로 들렸다(2026-08-10). 펄스를 짧게 쓰려면 루프가 더 촘촘해야 한다.
    RATE = 0.02
    MARK_TOL_FRAC = 0.12  # 마커가 화면 중앙 ±이 비율 안에 오면 "도착"
    # 방향 추정에 쓸 최소 변위(px). 이보다 작으면 잡음으로 보고 무시한다.
    DIR_ODOM_MIN_PX = 12.0

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
        self._found_ts = 0.0   # 마지막으로 테이프가 **보였던** 시각(출발 판정 히스테리시스)
        # 지점(스테이션) 추적 — 마커 **색**이 곧 번호다(LINE_MARKER_STATION).
        self._station: int | None = None
        self._at_end = False
        self._mark_armed = True          # 다음 마커 통과를 받을 준비가 됐나
        self._marked_odom: float | None = None   # 마지막으로 마커를 센 시점의 변위
        self._seek_odom0: float | None = None    # 명령 시작 시점의 변위(출발 킥 종료 판정)
        self._kick_done = False    # 출발 킥이 끝났나(굴렀으면 재점화하지 않는다)
        # 한 칸씩 가는 중이면 **최종** 목적지. 매 도착마다 다음 칸을 이어 시작한다.
        self._chain_target: int | None = None
        self._wiggle_on = False    # 좌우 톡 엄격 교대용(직전 틱이 ON이었나)
        self._wiggle_n = 0         # 실제로 낸 좌우 톡 수 — 홀짝으로 부호를 정한다
        self._last_marker: dict | None = None
        self._last_dir = 0               # 마지막 진행 방향(+1 오른쪽 / -1 왼쪽)
        self._dir_source = "미확인"      # 그 방향을 어디서 알았나(명령/오도메트리)
        self._odom_prev: float | None = None   # 방향 추정용 직전 오도메트리 표본
        self._between = False            # 경유(초록)를 지나 지점 사이에 있나
        self._last_way: dict | None = None
        self._align_worst: float | None = None   # 정렬 중 최소 오차(발산 감지용)
        self._align_diverge = 0
        self._align_gate = True    # 직전 틱의 펄스 게이트(발산 판정을 경계에서만 하려고)
        # 지점 정렬의 "굳음 감지 → 큰 충격" 상태
        self._mark_best: float | None = None   # 지금까지 가장 좋았던 오차
        self._mark_stall = 0                   # 연속으로 못 줄인 펄스 수
        self._mark_break_until = 0.0           # 이 시각까지는 충격 구간
        self._mark_break_axis = "travel"       # 어느 축이 막혔나
        self._mark_breaks = 0                  # 흔들어 푼 횟수(진단용)
        self._mark_settle_until = 0.0          # 정착(완전 정지) 대기 종료 시각
        self._mark_verifies = 0                # 정착 후 다시 잰 횟수(진단용)
        # 게걸음 금지 모드에서 "잠시 정렬하러 갔다 오는" 상태.
        # _resume = (원래 모드, 원래 goal, 남은 제한시간) — 정렬이 끝나면 되돌린다.
        self._resume: tuple[str, dict, float] | None = None
        self._detour_started = 0.0
        self._detours = 0
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
            # 게걸음 금지(1=금지). 주행은 전후+회전만 쓰고, 거리 오차는 정렬이 잡는다.
            "no_strafe": 1.0 if LINE_NO_STRAFE else 0.0,
            # 수동 이동 방향 판정 부호(오도메트리 +가 "오른쪽"인가).
            "odom_sign": 1.0,
            # 정렬 펄스에 얹는 전후 흔들기 세기(0=없음). 롤러 정지마찰을 깨려고.
            "align_dither": float(LINE_ALIGN_DITHER),
            # 주행 방식: 1=저속 연속(vy·w 비례보정), 0=펄스(톡톡).
            "smooth": 1.0 if LINE_DRIVE_SMOOTH else 0.0,
            "smooth_speed": float(LINE_SMOOTH_SPEED),
            # 출발 한 방 + 주행 중 좌우 톡 — 정지마찰을 깨는 두 수단.
            "travel_kick": float(LINE_TRAVEL_KICK),
            # 옆으로 흔들 때 **첫 반 주기**가 향하는 쪽(±1). 나무를 피한다.
            "wiggle_sign": float(LINE_WIGGLE_SIGN),
            # 흔들 때 같이 낼 회전 크기(0이면 회전 없이 게걸음만).
            "wiggle_yaw": float(LINE_WIGGLE_YAW),
            # 두 칸 이상 이동할 때 중간 지점을 다 들르나.
            "stop_each": 1.0 if LINE_STOP_EACH_STATION else 0.0,
            "travel_wiggle": float(LINE_TRAVEL_WIGGLE),
            # 정렬을 **끝낼** 목표 범위. 주행 보정 데드밴드와 다른 값이다
            # (그건 "보정을 낼지", 이건 "그만해도 되는지"). y는 위아래가 다르다.
            "align_tol_x": float(LINE_MARK_ALIGN_TOL_PX),
            "align_dy_below": float(LINE_ALIGN_DY_BELOW_PX),
            "align_dy_above": float(LINE_ALIGN_DY_ABOVE_PX),
            "align_yaw_tol": float(LINE_ALIGN_YAW_TOL),
            # 정렬 펄스 한 번의 크기. ⚠ 속도 슬라이더는 **진행축에만** 곱해진다 —
            # 게걸음(보정)이 큰 건 여기 때문이지 속도 때문이 아니다.
            "corr_min": float(LINE_CORR_MIN),
            "corr_max": float(LINE_MAX_CORRECTION),
            # 저속 연속 주행(smooth)에서 dy 오차 → vy로 바꾸는 비례이득.
            # 상한은 corr_max를 함께 쓴다 — 게걸음은 한 개념이어야 한다.
            "smooth_dy_gain": float(LINE_SMOOTH_DY_GAIN),
        }
        self._tune.update(_load_saved(LINE_TUNING_FILE))
        self._thread = threading.Thread(target=self._run, daemon=True, name="line-drive")
        self._thread.start()

    # ------------------------------------------------------------------
    # 명령
    # ------------------------------------------------------------------

    def _end_marker_centered(self, line: dict) -> bool:
        """**지점 마커**가 지금 화면 중앙에 와 있나.

        ⚠ 예전엔 주황(end)만 봤다. 2지점 코스에서는 노랑도 코스의 끝이라
        (노랑=지점0=좌측 끝) 지점 마커면 전부 끝점이다 — LINE_MARKER_STATION이
        코스의 정의이므로 거기 있는 색이면 받는다.
        """
        width = line.get("width") or 1280
        return self._centered_station_marker(line, width) is not None

    def _centered_station_marker(self, line: dict, width: float | None = None):
        """중앙에 와 있는 지점 마커 하나. 없으면 None."""
        width = width or line.get("width") or 1280
        near = [m for m in (line.get("markers") or [])
                if self._marker_kind(m) in LINE_MARKER_STATION
                and abs(m["x"] - width / 2) < width * self.MARK_TOL_FRAC]
        return min(near, key=lambda m: abs(m["x"] - width / 2)) if near else None

    def _station_of(self, marker: dict) -> int | None:
        """마커 **색 하나로** 지점 번호를 정한다 — 세지 않는다.

        코스가 2지점이 되면서 색이 곧 지점이다(노랑=0, 주황=1). 예전의 "주황을
        기준으로 잡고 마커를 지날 때마다 진행 방향으로 ±1" 방식은 진행 방향
        추정에 얹혀 있어서, 부호가 어긋나거나 사람이 로봇을 손으로 옮기면 번호가
        통째로 틀어졌다(2026-08-11: 제자리 정렬 중에 1→2→3으로 올라간 사고).
        관측 하나가 곧 답이면 그런 사고가 구조적으로 생기지 않는다.
        """
        return LINE_MARKER_STATION.get(self._marker_kind(marker) or "")

    def goto_end(self, side: str, speed: int | None = None) -> str:
        """끝점(주황 마커)까지 라인을 유지하며 저속 주행 후 정지."""
        if side not in ("left", "right"):
            raise ValueError("side는 left/right")
        # 이미 끝점 위면 출발하지 않는다 — 안 그러면 끝점을 지나쳐 반대편까지 밀고 간다.
        if self._end_marker_centered(self._line):
            self._latch_end(side)
            return "이미 끝점에 있습니다 (변위 기준만 갱신)"
        self._start("goto_end", {"side": side, "speed": self._speed(speed)},
                    f"{'왼쪽' if side == 'left' else '오른쪽'} 끝점으로 이동")
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
        limits = {"speed": (30, 255), "pulse_on": (0.02, 3.0), "pulse_period": (0.02, 3.0),
                  "align_dither": (0, 255), "smooth_speed": (30, 255),
                  "travel_kick": (0, 255), "travel_wiggle": (0, 255),
                  "wiggle_yaw": (0, 255),
                  # 정렬 완료 범위. 하한을 5px/0.5°로 둔다 — 0으로 두면 영원히
                  # 못 끝내는 정렬이 되고, 그건 설정으로 만들 수 있으면 안 된다.
                  "align_tol_x": (5, 400), "align_dy_below": (5, 400),
                  "align_dy_above": (5, 400), "align_yaw_tol": (0.5, 45.0),
                  # 정렬 펄스 크기(게걸음·회전). 30 밑은 정지마찰을 못 넘어
                  # "지령은 나가는데 안 움직인다"가 된다 — 거기서 막는다.
                  "corr_min": (30, 255), "corr_max": (30, 255),
                  "smooth_dy_gain": (20, 600)}
        with self._lock:
            for key, (lo, hi) in limits.items():
                if values.get(key) is not None:
                    self._tune[key] = float(max(lo, min(hi, float(values[key]))))
            # 주기가 ON보다 짧으면 의미가 없다 — 연속 주행으로 해석한다.
            if self._tune["pulse_period"] < self._tune["pulse_on"]:
                self._tune["pulse_period"] = self._tune["pulse_on"]
            # 상한이 하한보다 작으면 하한만 남는다(= 크기 고정). 뒤집힌 채로
            # 두면 슬라이더가 서로를 무시하는 것처럼 보인다.
            if self._tune["corr_max"] < self._tune["corr_min"]:
                self._tune["corr_max"] = self._tune["corr_min"]
            snapshot = dict(self._tune)
        self._save_tuning(snapshot)
        mode = ("연속 주행" if snapshot["pulse_period"] <= snapshot["pulse_on"]
                else f"펄스 {snapshot['pulse_on']:.2f}s / {snapshot['pulse_period']:.2f}s 주기")
        dither = snapshot.get("align_dither", 0)
        # 정렬 목표 범위도 같이 돌려준다 — 슬라이더를 만졌는데 어디에도 안 보이면
        # "먹었나?" 를 확인할 길이 없다.
        rng = (f" · 게걸음 정렬{snapshot.get('corr_min', 0):.0f}~{snapshot.get('corr_max', 0):.0f}"
               f"/주행이득{snapshot.get('smooth_dy_gain', 0):.0f}"
               f" · 정렬범위 x±{snapshot.get('align_tol_x', 0):.0f}px"
               f" · y 아래{snapshot.get('align_dy_below', 0):.0f}"
               f"/위{snapshot.get('align_dy_above', 0):.0f}px"
               f" · 회전 ±{snapshot.get('align_yaw_tol', 0):.1f}°")
        if snapshot.get("smooth"):
            return (f"저속 연속 주행 {snapshot.get('smooth_speed', 0):.0f} · "
                    f"정렬 흔들기 {dither:.0f}" + rng
                    + f" · (펄스로 바꾸면 속도 {snapshot['speed']:.0f} · {mode})")
        return (f"속도 {snapshot['speed']:.0f} · {mode} · "
                f"정렬 흔들기 {dither:.0f}" + (" (없음)" if dither <= 0 else "") + rng)

    def _save_tuning(self, snapshot: dict) -> None:
        try:
            with open(os.path.expanduser(LINE_TUNING_FILE), "w", encoding="utf-8") as f:
                json.dump(snapshot, f, ensure_ascii=False, indent=2)
        except OSError as exc:
            self._detail = f"설정 저장 실패: {exc}"

    # ------------------------------------------------------------------
    # 지점(스테이션) 이동 — **색 하나가 곧 지점**(노랑=0, 주황=1). 세지 않는다.
    # ------------------------------------------------------------------

    def _marker_kind(self, marker: dict) -> str | None:
        """마커의 색. 'end'(주황) | 'mid'(노랑) | 'way'(초록=경유) | None.

        ⚠ 'end'/'mid'는 이제 **색 이름일 뿐**이다 — 옛 "양끝/중간"이라는 뜻은
        2지점 코스에서 사라졌다(둘 다 코스의 끝이다). 색 → 지점은
        config.LINE_MARKER_STATION이 정한다.

        판정은 **검출기(line_follow)가 이미 했다** — 여기서 다시 하면 두 곳의
        hue 창이 어긋날 수 있다. 옛 상태 파일 호환용으로만 hue 폴백을 남긴다.
        """
        # ⚠ 검출기가 준 역할은 **항상** 우선한다. 예전엔 end/mid만 받아들여서
        #   'way'(초록 경유)가 폴백으로 넘어가 hue로 재판정됐고, 그 결과 초록이
        #   끝점으로 잡혀 지점 번호가 어긋났다(2026-08-10 테스트에서 발견).
        role = marker.get("role")
        if role in ("end", "mid", "way"):
            return role
        hue = marker.get("hue")
        if hue is None:
            return None
        if LINE_HUE_END[0] <= hue <= LINE_HUE_END[1]:
            return "end"
        if LINE_HUE_MID[0] <= hue <= LINE_HUE_MID[1]:
            return "mid"
        return None

    def _track_direction(self, line: dict) -> None:
        """**실제로 움직인 방향**으로 진행 방향을 갱신한다 (자동주행 중이 아닐 때).

        ⚠ 2026-08-11 실사고. 예전엔 _last_dir을 명령의 side("left"/"right")로만
        정했다. 그래서 수동 조작(키보드·게임패드·손으로 밀기)으로 옮기면 방향이
        **직전 명령 값에 멈춰** 있었고, 그 상태로 끝점(주황)을 지나면 반대쪽
        끝으로 기록됐다 — 오른쪽 끝에 서 있는데 화면은 "0. 좌측 끝"이라고 했다.
        한 번 어긋나면 이후 지점 번호가 전부 반대로 밀린다.

        바닥 카메라 시각 오도메트리는 **무엇이 밀었든** 실제 변위를 알려주므로
        수동 이동에는 그쪽이 맞다. 자동주행 중에는 기존대로 명령을 믿는다
        (그쪽이 의도를 알고, 오도메트리는 펄스 사이 잡음이 섞인다).

        화면 x축이 로봇의 좌우 중 어디인지는 카메라 장착에 달렸으므로 부호는
        런타임 토글(odom_sign)로 뒤집는다 — 코드에 박으면 매번 배포해야 한다.
        """
        odom = line.get("odom_x_px")
        if odom is None:
            return
        if self._odom_prev is None:
            self._odom_prev = odom
            return
        delta = odom - self._odom_prev
        if abs(delta) < self.DIR_ODOM_MIN_PX:
            return              # 아직 의미 있는 이동이 아니다 — 기준점을 유지
        self._odom_prev = odom
        self._last_dir = 1 if delta * float(self._tune.get("odom_sign", 1.0)) > 0 else -1
        self._dir_source = "오도메트리(수동 이동)"

    def _which_end(self, line: dict) -> str | None:
        """끝점 위에 섰을 때 **어느 쪽 끝인가** — 테이프가 이어지는 방향으로 안다.

        코스는 한쪽으로만 이어진다. 테이프가 오른쪽에만 있으면 여기가 왼쪽 끝이다.
        진행 방향을 몰라도, 손으로 옮겼어도 항상 맞는 **절대 관측**이다.
        (예전엔 _last_dir로 추측해서 반대로 찍혔다 — 2026-08-11 실사고.)

        애매하면(양쪽 다 있거나 다 없거나) None. 모를 때는 추측하지 않는다.
        """
        left = line.get("band_left_frac")
        right = line.get("band_right_frac")
        if left is None or right is None:
            return None            # 구 버전 검출기 — 판정 불가
        if right - left > LINE_END_SIDE_MARGIN:
            return "left"          # 오른쪽으로 이어진다 = 여기가 왼쪽 끝
        if left - right > LINE_END_SIDE_MARGIN:
            return "right"
        return None

    def _anchor_at_end(self, line: dict) -> None:
        """끝점 위에 서 있으면 **가만히 있어도** 지점 번호를 확정한다.

        주행 중이 아니어도 돈다 — 손으로 옮겨놨든, 번호가 어긋나 있든,
        끝점에 서 있기만 하면 스스로 바로잡힌다.
        """
        marker = self._centered_station_marker(line)
        if marker is None:
            return
        index = self._station_of(marker)
        if index is None:
            return
        if self._station != index:
            with self._lock:
                self._station = index
                self._at_end = True
                self._between = False
            print(f"  [line] 지점 확인 — {LINE_STATION_LABELS[index]} "
                  f"(마커 hue {marker.get('hue')})")
        # 원점 래치는 여전히 **테이프가 이어지는 방향**으로 한다(변위 기준).
        side = self._which_end(line)
        if side is not None and self._last_end != side:
            self._latch_end(side)

    def set_station(self, index: int) -> str:
        """지금 있는 곳이 몇 번 지점인지 **사람이 직접 알려준다**.

        마커를 세는 방식은 진행 방향에 의존하는데, 손으로 옮기거나 방향 부호가
        어긋나면 그 전제가 깨진다. 그때 번호를 되찾는 가장 확실한 길 —
        부호가 맞든 틀리든 이건 항상 통한다(데모 중 탈출구).
        """
        if not 0 <= index < len(LINE_STATION_LABELS):
            raise ValueError(f"지점 번호는 0~{len(LINE_STATION_LABELS) - 1}")
        with self._lock:
            self._station = index
            self._at_end = index in (0, len(LINE_STATION_LABELS) - 1)
            self._between = False
            # ⚠ mark_armed는 **False**여야 한다. True로 두면 지금 화면에 보이는
            #   마커를 다음 틱에 _observe_markers가 곧바로 다시 세면서, 낡은
            #   진행 방향으로 방금 지정한 번호를 덮어썼다 —
            #   "[0으로 지정]을 눌러도 3으로 설정된다"의 정체(2026-08-11).
            #   지금 보이는 마커는 이미 이 지정에 반영된 것이므로 다시 셀 이유가 없다.
            self._mark_armed = False
        if self._at_end:
            # 끝점이면 변위 기준까지 같이 잡는다 — 절대 기준점이므로.
            self._latch_end("left" if index == 0 else "right")
        return f"현재 위치를 '{LINE_STATION_LABELS[index]}'로 설정했습니다"

    def _observe_markers(self, line: dict) -> None:
        """마커가 화면 중앙을 지나가는 순간을 잡아 지점 인덱스를 갱신한다.

        같은 마커를 두 번 세지 않으려고, 한 번 인정한 뒤에는 중앙에서 충분히
        벗어나야(LINE_MARK_CLEAR_FRAC) 다음 통과를 받는다.
        """
        # ⚠ 정렬 중에는 지점 번호를 **절대 바꾸지 않는다.** 정렬 펄스로 마커가
        #   중앙 창(±154px)을 들락거릴 때마다 재무장→재계수되어, 제자리에서
        #   1→2→3으로 번호가 올라갔다(2026-08-11 실사고). 정렬은 "제자리 유지"
        #   동작이다 — 번호는 이동만이 바꾼다.
        #   띠가 안 보일 때도 세지 않는다: 띠 근접 필터가 못 돌아서 코스 밖
        #   색 덩어리가 걸러지지 않은 채 들어온다.
        if self._mode in ("align", "align_mark") or not line.get("found"):
            return
        markers = line.get("markers") or []
        width = line.get("width") or 1280
        center = width / 2
        # ★ 재무장은 **움직인 거리**로 판단한다. 화면 안 마커 배치로 판단하던
        #   예전 방식은 "지점 간격이 화면보다 넓다"를 가정했는데 실제는 반대라
        #   (간격 375px vs 화면 1280px) 조건이 영영 성립하지 않았다 —
        #   한 번 센 뒤로 다음 지점을 통째로 못 세고 끝까지 밀려갔다(2026-08-11).
        odom = self._odom()
        if not self._mark_armed and odom is not None and self._marked_odom is not None:
            if abs(odom - self._marked_odom) >= LINE_MARK_REARM_PX:
                self._mark_armed = True
        near = [m for m in markers if abs(m["x"] - center) < width * self.MARK_TOL_FRAC]
        if not near:
            # 중앙에서 충분히 벗어났으면 다음 통과를 받을 준비를 한다.
            # (오도메트리가 없을 때를 위한 보조 경로 — 넓은 간격에서는 이쪽이 먼저 선다.)
            if not markers or all(abs(m["x"] - center) > width * LINE_MARK_CLEAR_FRAC
                                  for m in markers):
                # ⚠ 경계 통과만으로는 부족하다 — **실제로 움직였어야** 한다.
                #   정렬 직후 정착 진동·검출 노이즈로 마커가 경계를 들락거리면
                #   제자리에서 번호가 1→2→3으로 올라갔다(2026-08-11 실사고).
                if (odom is None or self._marked_odom is None
                        or abs(odom - self._marked_odom) >= LINE_MARK_REARM_MIN_PX):
                    self._mark_armed = True
            return
        if not self._mark_armed:
            return
        self._mark_armed = False
        self._marked_odom = odom
        marker = min(near, key=lambda m: abs(m["x"] - center))
        kind = self._marker_kind(marker)
        if kind == "way":
            # 초록 = 경유 표식. **지점 번호를 바꾸지 않는다** — 지점 사이가 멀어
            # 카메라가 아무것도 못 보는 구간을 메우려고 붙인 것이라, 이걸 세면
            # 지점 번호가 어긋난다. "지점 사이를 지나는 중"이라는 것만 기록한다.
            self._between = True
            self._last_way = marker
            return
        # ★ 색 하나가 곧 지점이다 — 세지 않는다(LINE_MARKER_STATION).
        #   예전엔 주황을 기준으로 잡고 그 뒤로 진행 방향으로 ±1 했는데, 그 셈이
        #   방향 추정에 얹혀 있어서 부호가 어긋나거나 손으로 옮기면 번호가 통째로
        #   틀어졌다. 지금은 어느 방향으로 지나든, 손으로 밀어 넣었든 같은 답이다.
        index = self._station_of(marker)
        if index is not None:
            self._station = index
            # 2지점 코스에서는 지점이 곧 양끝이다.
            self._at_end = index in (0, len(LINE_STATION_LABELS) - 1)
            self._between = False
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
        # 두 칸 이상이면 **한 칸씩** 간다(토글). 칸마다 마커에 정렬하고 출발하므로
        # 오차가 쌓이지 않고, 중간에 어긋나도 거기서 드러난다. 끄면 한 번에 간다.
        if self._tune.get("stop_each", 1.0) and abs(index - self._station) >= 2:
            self._start_leg(index, speed)
        else:
            side = "right" if index > self._station else "left"
            self._start("station", {"side": side, "target": index, "speed": self._speed(speed)},
                        f"{LINE_STATION_LABELS[index]}(으)로 이동")
        return self._detail

    def _start_leg(self, final: int, speed: int | None = None) -> bool:
        """최종 목적지 final을 향해 **한 칸**만 간다. 더 갈 칸이 없으면 False.

        _start가 _chain_target을 지우므로(새 명령이면 연쇄도 끝나야 한다) 여기서
        **다시 건다** — 순서가 중요하다.
        """
        if self._station is None or self._station == final:
            return False
        step = 1 if final > self._station else -1
        leg = self._station + step
        left = abs(final - self._station)
        self._start("station",
                    {"side": "right" if step > 0 else "left", "target": leg,
                     "speed": self._speed(speed)},
                    f"{LINE_STATION_LABELS[final]}(으)로 — {LINE_STATION_LABELS[leg]} 경유"
                    f" ({left}칸 남음)")
        with self._lock:
            self._chain_target = final
        return True

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
            # 정지는 정지다 — 접어둔 주행도, 남은 칸도 되살아나면 안 된다.
            self._resume = None
            self._chain_target = None
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
            "found_reason": line.get("found_reason"),
            "offset_y_px": line.get("offset_y_px"),
            "angle_deg": line.get("angle_deg"),
            "band_y": line.get("band_y"),
            "end_side": (line.get("end_marker") or {}).get("side"),
            "end_x": (line.get("end_marker") or {}).get("x"),
            "color_name": (line.get("color_marker") or {}).get("name"),
            "color_role": (line.get("color_marker") or {}).get("role"),
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
            "between": self._between,
            "markers": (self._line or {}).get("markers") or [],
            "markers_dropped": (self._line or {}).get("markers_dropped") or 0,
            # 가장 가까운 지점 마커의 중앙 오차(px) — 지점 정렬이 잡는 값.
            "mark_dx": (lambda g: None if g is None else round(g[1], 1))(
                self._nearest_station_marker(self._line or {})),
            "last_marker": self._last_marker,
            "speed": self._tune["speed"],
            "pulse_on": self._tune["pulse_on"],
            "pulse_period": self._tune["pulse_period"],
            "pulsing": self._tune["pulse_on"] < self._tune["pulse_period"] - 1e-6,
            "smooth": self.smooth,
            "smooth_speed": self._tune.get("smooth_speed", LINE_SMOOTH_SPEED),
            "travel_kick": self._tune.get("travel_kick", LINE_TRAVEL_KICK),
            "travel_wiggle": self._tune.get("travel_wiggle", LINE_TRAVEL_WIGGLE),
            "wiggle_sign": self._tune.get("wiggle_sign", LINE_WIGGLE_SIGN),
            "wiggle_yaw": self._tune.get("wiggle_yaw", LINE_WIGGLE_YAW),
            "stop_each": bool(self._tune.get("stop_each", 1.0)),
            "chain_target": self._chain_target,
            "no_strafe": self.no_strafe,
            "align_dither": self._tune.get("align_dither", LINE_ALIGN_DITHER),
            # 정렬 완료 목표 범위(px·도) — 대시보드 슬라이더가 이 값으로 채워진다.
            "align_tol_x": self._tune.get("align_tol_x", LINE_MARK_ALIGN_TOL_PX),
            "align_dy_below": self._tune.get("align_dy_below", LINE_ALIGN_DY_BELOW_PX),
            "align_dy_above": self._tune.get("align_dy_above", LINE_ALIGN_DY_ABOVE_PX),
            "align_yaw_tol": self._tune.get("align_yaw_tol", LINE_ALIGN_YAW_TOL),
            "corr_min": self._tune.get("corr_min", LINE_CORR_MIN),
            "corr_max": self._tune.get("corr_max", LINE_MAX_CORRECTION),
            "smooth_dy_gain": self._tune.get("smooth_dy_gain", LINE_SMOOTH_DY_GAIN),
            "detours": self._detours,
            "resuming": self._resume is not None,
            "last_dir": self._last_dir,
            "dir_source": self._dir_source,
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
        if speed is not None:
            return int(max(30, min(255, float(speed))))
        # 저속 연속 주행은 펄스보다 느린 값을 쓴다 — 펄스는 순간적으로 세게 밀어
        # 정지마찰을 넘겨야 하지만, 연속 주행은 계속 굴러가므로 그럴 필요가 없다.
        key = "smooth_speed" if self.smooth else "speed"
        return int(max(30, min(255, float(self._tune.get(key, LINE_SPEED)))))

    def _corr_pulse(self, error: float, gain: float, sign: float, deadband: float) -> int:
        """오차 하나를 **펄스 한 번 분량**의 지령으로 바꾼다 (이산 뱅뱅).

        비례제어로 작은 값을 계속 흘리면 정지마찰을 못 넘어 아무 일도 안 일어난다.
        그래서 데드밴드 밖이면 **반드시 움직이는 크기**(LINE_CORR_MIN 이상)로 주고,
        안이면 0을 준다. 사람이 키를 톡 치고 결과를 보는 것과 같은 구조.
        """
        if error is None or abs(error) < deadband:
            return 0
        want = abs(sign * gain * error)
        # 크기는 /settings에서 조절한다. 하한은 "정지마찰을 넘는 최소",
        # 상한은 "한 걸음이 목표를 지나치지 않는 최대" — 둘 사이를 오차가 채운다.
        lo = float(self._tune.get("corr_min", LINE_CORR_MIN))
        hi = max(lo, float(self._tune.get("corr_max", LINE_MAX_CORRECTION)))
        mag = max(lo, min(hi, want))
        return int(mag if (sign * error) > 0 else -mag)

    def _pulse_gate(self, goal: dict, mode: str = "") -> bool:
        """지금이 펄스의 ON 구간인가. OFF 구간에는 **보정까지 전부 0**을 보내
        완전히 멈춘 상태에서 정착시킨 뒤 다시 측정한다.

        ⚠ 예전엔 여기서 '접근(coarse)/정밀(fine)' 두 국면을 갈라, 목표 마커가
        화면 중앙 근처에 없으면 ON 0.30/주기 0.34(≈88% 듀티)로 갔다. 지점이 3개일
        땐 간격이 판정 창보다 좁아 그 국면이 거의 안 나왔는데, 2지점(간격 955px)이
        되자 코스 한가운데가 통째로 그 국면이 됐다 — **톡톡으로 설정해 뒀는데도
        사실상 연속 주행으로 달려 발산했다.** 판정에 히스테리시스도 없어서 검출이
        한 프레임 깜빡이는 것만으로 300ms짜리 가속 펄스가 났다.
        지금은 국면이 하나다 — 설정한 펄스 값이 전 구간에 그대로 나간다.
        """
        # 저속 연속 주행에서는 게이팅 자체가 없다 — 계속 굴러가면서 보정한다.
        # (정렬은 제자리 동작이라 여전히 톡톡이다. 안 구르면 롤러가 안 미끄러진다.)
        if self.smooth and mode not in ("align", "align_mark"):
            return True
        # 정착 대기 중에는 **아무 지령도 내지 않는다** — 완전히 서야 관성이 죽고,
        # 그래야 그 다음 측정이 진짜 최종 위치다.
        if mode == "align_mark" and time.monotonic() < self._mark_settle_until:
            return False
        on = float(self._tune.get("pulse_on", LINE_PULSE_ON))
        period = float(self._tune.get("pulse_period", LINE_PULSE_PERIOD))
        if period <= on:
            return True
        return ((time.monotonic() - goal["started"]) % period) < on

    def align(self, speed: int | None = None) -> str:
        """제자리에서 **톡톡** 쳐가며 기준선·평행에 맞춘다(진행 없음).

        오차가 데드밴드 안에 들어올 때까지 펄스를 반복한다. 오차가 계속 커지면
        보정 부호가 반대라는 뜻이므로 스스로 멈추고 그렇게 알려준다.
        """
        self._start("align", {"speed": self._speed(speed)}, "정렬 중 — 톡톡 쳐서 맞춥니다")
        with self._lock:
            self._align_worst = None
            self._align_diverge = 0
            self._align_gate = True
        return self._detail

    def _nearest_station_marker(self, line: dict, ahead_dir: int = 0):
        """화면에서 중앙에 가장 가까운 **지점 마커**(주황/노랑)와 중앙까지의 dx.

        ahead_dir이 있으면 진행 방향 앞쪽(약간 지나친 것까지 포함) 마커만 본다 —
        접근 감속이 방금 지나온 마커에 걸려 엉뚱하게 느려지지 않게.
        화면 x+ = 코스 오른쪽(끝점 판정과 같은 전제)이므로 dx>0이면 오른쪽이다.
        """
        width = line.get("width") or 1280
        center = width / 2
        best = None
        for m in line.get("markers") or []:
            if self._marker_kind(m) not in ("end", "mid"):
                continue
            dx = m["x"] - center
            if ahead_dir and dx * ahead_dir < -width * 0.05:
                continue
            if best is None or abs(dx) < abs(best[1]):
                best = (m, dx)
        return best

    def align_mark(self, speed: int | None = None) -> str:
        """**지점 정렬** — 가장 가까운 지점 마커(주황/노랑)를 화면 중앙에 맞춘다.

        사람이 도착 후 손으로 하던 마무리([시간이동 0.1] 앞뒤 + [정렬톡톡] 조합 —
        2026-08-11 사진 2·6·8이 전부 이 수작업이다)를 그대로 자동화한 것.
        진행축은 마커의 dx를 향해 톡, 보정축은 dy/yaw를 함께 잡는다.
        """
        if self._nearest_station_marker(self._line) is None:
            raise RuntimeError(
                "화면에 지점 마커(주황/노랑)가 없습니다 — [톡]으로 마커가 보이는 "
                "곳까지 이동한 뒤 다시 누르세요")
        self._start("align_mark", {"speed": self._speed(speed)},
                    "지점 정렬 — 마커를 화면 중앙에")
        self._reset_mark_align()
        return self._detail

    def _reset_mark_align(self) -> None:
        """지점 정렬을 처음부터 다시 시작하는 상태로."""
        with self._lock:
            self._align_worst = None
            self._align_diverge = 0
            self._align_gate = True
            self._mark_best = None
            self._mark_stall = 0
            self._mark_break_until = 0.0
            self._mark_breaks = 0
            self._mark_settle_until = 0.0
            self._mark_verifies = 0
            # "될 때까지" 하되 영원히는 아니다 — 넉넉한 상한을 준다.
            self._deadline = time.monotonic() + LINE_MARK_ALIGN_TIMEOUT_SEC

    def _mark_pulse(self, line: dict) -> tuple[int, int]:
        """지점 정렬의 진행축 펄스: (방향, 크기). 중앙이면 (0, 0).

        크기는 dx에 비례하되 정지마찰을 확실히 넘는 바닥(LINE_CORR_MIN)을 깔아준다
        — 펄스 뱅뱅의 기본 규칙 그대로다.
        """
        got = self._nearest_station_marker(line)
        if got is None:
            return 0, 0
        _m, dx = got
        if abs(dx) <= self._tune.get("align_tol_x", LINE_MARK_ALIGN_TOL_PX):
            return 0, 0
        # 충격 구간이면 정지마찰을 확실히 넘는 크기로. 평소엔 상한 150으로 죈다
        # (LINE_MAX_CORRECTION=180은 여기엔 너무 세다 — 반대편으로 넘어가 왕복이 된다).
        if (time.monotonic() < self._mark_break_until
                and self._mark_break_axis == "travel"):
            # 굳음 해제 구간 — 크기는 톡톡 그대로다. 푸는 건 직각 흔들기가 한다.
            mag = LINE_MARK_BREAK_MAG
        else:
            # ⚠ 하한을 게걸음 슬라이더(corr_min, 현장 60)에서 가져왔었다. 그런데
            #   실측하면 60은 **한 펄스에 2.2px** — 아무 일도 안 한다. 그걸 "굳었다"로
            #   읽고 흔들다가 정렬이 발산했다(2026-08-17). 이제 **실측 곡선**으로
            #   "dx만큼 가려면 얼마를 줘야 하나"를 역산한다.
            mag = _pulse_for_px(abs(dx))
        return (1 if dx > 0 else -1), mag

    def _begin_arrive_align(self, reached: str) -> None:
        """지점 도착 → 곧바로 지점 정렬로 이어간다(멈춰서 사람 손을 기다리지 않고).

        과도하게 지나쳐 섰더라도 마커가 화면에 남아 있으면 여기서 되돌아온다 —
        "이전/다음 지점이 과도하게 넘어간다"의 마무리 처방이다(감속이 1차 처방).
        """
        now = time.monotonic()
        with self._lock:
            self._mode = "align_mark"
            self._goal = {"speed": self._speed(None), "started": now}
            self._detail = f"{reached} — 지점 정렬(톡톡)로 마무리"
        self._reset_mark_align()   # deadline도 여기서 넉넉하게 다시 잡는다

    def _align_error(self, line: dict) -> float:
        """정렬 오차 하나로 합친 값(허용 범위 대비 비율의 최댓값). 1 이하면 도착.

        ⚠ y는 **목표 범위를 px로 직접** 잰다(2026-08-13 요청). 주행 보정
        데드밴드(LINE_DY_DEADBAND)는 "보정을 낼지"의 기준이지 "끝낼지"의
        기준이 아니다 — 그걸 완료 판정에 쓰면 정착 후 재측정에서 조금만
        밀려도 정렬이 다시 시작돼 끝나지 않는다.

        위아래 허용치가 다르다: dy > 0 = 테이프가 기준선보다 아래.
        범위는 전부 /settings에서 바꾼다(self._tune).
        """
        dy_px = line.get("offset_y_px") or 0.0
        limit = (self._tune.get("align_dy_below", LINE_ALIGN_DY_BELOW_PX) if dy_px > 0
                 else self._tune.get("align_dy_above", LINE_ALIGN_DY_ABOVE_PX))
        dy = abs(dy_px) / max(1e-6, float(limit))
        yaw_tol = max(1e-6, float(self._tune.get("align_yaw_tol", LINE_ALIGN_YAW_TOL)))
        yaw = (abs(line.get("angle_deg") or 0.0) / yaw_tol
               if self._tune.get("yaw_gain") else 0.0)
        return max(dy, yaw)

    def _travel_dir(self, goal: dict) -> int:
        """지금 이 순간 진행분을 실을지 정한다 — **펄스 구동**.

        저속으로 계속 미는 대신 짧고 빠른 펄스를 반복한다. 정지마찰 근처에서
        질질 끌리다 갑자기 미끄러지는 걸 막고, 한 번에 가는 거리가 일정해진다.
        ON == 주기면 항상 True가 되어 연속 주행과 같아진다(별도 모드가 아니다).
        """
        base = 1 if goal.get("side") == "right" else -1
        return base if self._pulse_gate(goal, self._mode) else 0

    def _preflight(self, mode: str = "") -> None:
        """출발 전 점검 — 여기서 막아야 "눌렀는데 바로 멈춘다"가 안 생긴다.

        기준선이 지금 위치와 너무 어긋나 있으면(코스를 새로 깔았거나 로봇을 옮긴
        뒤 기준을 다시 안 잡은 경우) 출발하자마자 안전정지에 걸린다. 그 상태를
        '실패'로 조용히 넘기지 말고, **무엇을 눌러야 하는지**까지 알려준다.

        ⚠ 2026-08-11 실사고. 이 검사가 **정렬에도** 걸려 있었다. 그런데 정렬은
        "벗어난 걸 되돌리는" 동작이다 — 크게 벗어났을 때 유일한 복구 수단이
        바로 그 벗어남을 이유로 잠긴 것이다. dy=246px에서 주행도 정렬도 안 되어
        사람이 로봇을 손으로 들어 옮겨야 했다. 테이프가 **보이기만 하면** 정렬은
        언제나 시도할 수 있어야 한다(폭주 방지는 발산 감지가 따로 한다).
        """
        line = self._line or {}
        # ⚠ 순간적인 검출 실패로 버튼이 통째로 잠기면 안 된다(2026-08-11 실사고:
        #   "테이프 없음이 뜨고 다음 지점을 눌러도 반응이 없다"). 검출은 프레임
        #   단위로 깜빡일 수 있고 — 찢어진 JPEG, 코스 끝에서 띠가 화면 밖으로
        #   나가는 순간 등 — 그때마다 조작이 막히면 사람이 손쓸 방법이 없다.
        #   그래서 주행 중 정지 판정과 **같은 유예**(LINE_LOST_STOP_SEC)를 준다:
        #   "그 시간 안에 보였으면 출발해도 된다". 진짜 안 보이면 아래에서 막힌다.
        if not line.get("found") and time.monotonic() - self._found_ts > LINE_LOST_STOP_SEC:
            raise RuntimeError(
                "테이프가 안 보입니다 — 카메라가 띠를 보게 한 뒤 다시 눌러주세요. "
                "완전히 벗어났으면 아래 [바퀴 수동]으로 띠 위까지 옮기세요."
            )
        # 정렬은 언제나 시도할 수 있고, 정렬 우회가 가능한 모드도 막을 이유가 없다.
        # 이 검사의 목적은 "눌렀는데 바로 멈춘다"를 없애는 것인데, 우회가 있으면
        # 바로 멈추는 대신 **알아서 붙이고 출발**하기 때문이다. 사람에게 버튼을
        # 두 번 누르게 하지 않는다.
        if mode in ("align", "align_mark") or (self.no_strafe and mode in DETOUR_MODES):
            return
        dy = line.get("offset_y_norm")
        if dy is not None and abs(dy) > LINE_MAX_DY_NORM:
            raise RuntimeError(
                f"기준선에서 너무 멀어 출발할 수 없습니다(dy={line.get('offset_y_px'):.0f}px). "
                "먼저 [🎯 정렬(톡톡)]로 붙인 뒤 다시 시도하세요. "
                "지금 거리가 맞다면 [현재 위치를 기준선으로]를 누르세요."
            )

    def _start(self, mode: str, goal: dict, detail: str) -> None:
        self._preflight(mode)
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
            # 새 명령이니 "정렬 다녀오기" 상태도 초기화 — 이전 명령의 우회가
            # 남아 있으면 엉뚱한 주행으로 되돌아간다.
            self._resume = None
            self._detours = 0
            # 새 명령이 들어왔으면 한 칸씩 가던 연쇄도 끝이다. 이어달리기를
            # 계속할 _start_leg만 이 뒤에서 다시 건다.
            self._chain_target = None
            # 이 명령이 얼마나 굴러갔는지 재는 기준 — 출발 킥을 언제 끝낼지에 쓴다.
            self._seek_odom0 = self._odom()
            self._wiggle_on = False
            self._wiggle_n = 0
            self._kick_done = False
        # ★ **지금 서 있는 마커는 다시 세지 않는다.**
        #   2026-08-17 실기: [다음 지점]을 눌렀는데 0.7초 만에 "중간지점(노랑) 도착"이
        #   떴다 — 출발 지점의 마커가 아직 화면 중앙에 있는데 _observe_markers가 그걸
        #   세었고, next_mark의 도착 판정이 "_last_marker가 바뀌었나"라서 곧바로 걸렸다.
        #   그 뒤 엉뚱한 목표로 지점 정렬이 돌며 dy가 -7 → 327px로 발산해 테이프를
        #   놓쳤다. 재무장 거리(LINE_MARK_REARM_PX)를 **여기서부터** 다시 재게 한다.
        if mode in DETOUR_MODES and self._centered_station_marker(self._line) is not None:
            with self._lock:
                self._mark_armed = False
                self._marked_odom = self._odom()

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
            elif what == "smooth":
                self._tune["smooth"] = 0.0 if self.smooth else 1.0
            elif what == "no_strafe":
                self._tune["no_strafe"] = 0.0 if self.no_strafe else 1.0
            elif what == "odom_sign":
                # 수동 이동 시 "오른쪽으로 갔다"를 판정하는 부호. 화면 x축이
                # 로봇의 어느 쪽인지는 카메라 장착에 달렸다 — 실물로만 확정된다.
                self._tune["odom_sign"] = -float(self._tune.get("odom_sign", 1.0))
            elif what == "stop_each":
                # 두 칸 이상 이동할 때 중간 지점을 다 들를지.
                self._tune["stop_each"] = 0.0 if self._tune.get("stop_each", 1.0) else 1.0
            elif what == "wiggle_sign":
                # 지그재그가 **어느 쪽부터** 나가는가. 나무에 부딪히면 뒤집는다.
                self._tune["wiggle_sign"] = -float(
                    self._tune.get("wiggle_sign", LINE_WIGGLE_SIGN))
            elif what in ("dy_sign", "yaw_sign", "travel_sign"):
                self._tune[what] = -self._tune[what]
            else:
                raise ValueError(f"알 수 없는 항목: {what}")
            snapshot = dict(self._tune)
        self._save_tuning(snapshot)
        return (f"보정축={snapshot['dy_axis']} dy부호={snapshot['dy_sign']:+.0f} "
                f"회전보정={'켜짐' if snapshot['yaw_gain'] else '꺼짐'}"
                f"(부호{snapshot['yaw_sign']:+.0f}) 진행부호={snapshot['travel_sign']:+.0f} "
                f"게걸음={'금지(전후·회전만)' if snapshot.get('no_strafe') else '허용'} "
                f"주행={'저속연속(vy·w 비례보정)' if snapshot.get('smooth') else '펄스(톡톡)'} "
                f"흔들기 시작쪽={snapshot.get('wiggle_sign', 1.0):+.0f} "
                f"중간지점={'모두 들름' if snapshot.get('stop_each', 1.0) else '건너뜀'}")

    @property
    def no_strafe(self) -> bool:
        """게걸음(횡이동) 금지 모드인가. (저속 연속 주행에서는 의미 없음 — 아래 참고)"""
        return bool(self._tune.get("no_strafe", 1.0))

    @property
    def smooth(self) -> bool:
        """주행 방식이 '저속 연속'인가. 아니면 '펄스(톡톡)'."""
        return bool(self._tune.get("smooth", 1.0))

    def _corr_smooth(self, error, gain: float, sign: float, deadband: float,
                     cap: float | None = None) -> int:
        """굴러가는 중의 보정 — **비례제어**.

        멈춰 있을 때는 작은 값이 정지마찰을 못 넘어 뱅뱅(펄스)이 필요했다.
        하지만 바퀴가 이미 구르는 중이면 롤러가 미끄러질 수 있어 작은 값도
        그대로 먹는다 — 사람이 키를 눌러 몰 때 줄을 잘 따라가는 이유가 이것이다.
        그래서 여기서는 바닥값(corr_min) 없이 오차에 비례해 부드럽게 준다.

        ⚠ 2026-08-13: cap을 인자로 받는다. 예전엔 LINE_SMOOTH_CORR_MAX 고정이라
        **대시보드에서 게걸음을 줄여도 주행 중에는 아무 변화가 없었다** — 슬라이더가
        _corr_pulse만 건드렸기 때문이다(현장 보고: "게걸음을 설정해도 전혀 안 변한다").
        게걸음은 둘 중 어느 경로로 나가든 하나의 개념이어야 한다.
        """
        if error is None or abs(error) < deadband:
            return 0
        hi = float(LINE_SMOOTH_CORR_MAX if cap is None else cap)
        return int(max(-hi, min(hi, sign * gain * error)))

    def _travel_kick(self, goal: dict, speed: int) -> int:
        """출발 순간에는 세게 한 번 민다.

        정지마찰이 운동마찰보다 크므로 **처음만** 크면 된다. 순항 속도(105)로
        가만히 밀면 바퀴가 아예 안 돈다 — 파형은 평평한데 로봇은 제자리였다
        (2026-08-11). 한 번 구르기 시작하면 낮은 듀티로도 계속 굴러간다.
        """
        kick = float(self._tune.get("travel_kick", LINE_TRAVEL_KICK))
        if kick <= speed or self._kick_done:
            return speed
        if time.monotonic() - goal.get("started", 0.0) >= LINE_TRAVEL_KICK_SEC:
            self._kick_done = True
            return speed
        # ⚠ 킥은 **정지마찰용이지 가속용이 아니다.** 0.3초를 꽉 채우면 지점
        #   간격(≈375px)의 상당 부분을 킥 속도로 달려버려, 순항을 아무리 낮춰도
        #   "여전히 너무 빠르다"가 된다(2026-08-11 실사용 — 저속 88/77로 내려도
        #   체감이 안 변한 이유). 굴렀다는 증거(오도메트리 이동)가 보이는 즉시 끝낸다.
        odom = self._odom()
        if (odom is not None and self._seek_odom0 is not None
                and abs(odom - self._seek_odom0) > 25):
            self._kick_done = True
            return speed
        return int(kick)

    def _shake(self, step: float, amp: float) -> tuple[int, int]:
        """한 번의 흔들기가 낼 (게걸음, 회전). step은 ±1(교대 부호), amp는 게걸음 크기.

        **회전을 같이 쓰는 이유.** 게걸음은 메카넘 롤러가 옆으로 굴러야 생기는데,
        그게 바로 정지마찰에 가장 잘 걸리는 축이다. 반면 회전은 **네 바퀴가 모두
        제 축으로 도는** 동작이라 훨씬 쉽게 풀린다 — 먼저 풀린 바퀴가 구르기
        시작하면 게걸음도 따라 먹는다(2026-08-13 현장 요청).

        부호가 교대하므로 순회전은 상쇄된다 — 제자리에서 자세만 흔들릴 뿐
        방향이 누적해서 틀어지지 않는다. 게걸음의 상쇄와 같은 원리다.
        """
        first = float(self._tune.get("wiggle_sign", LINE_WIGGLE_SIGN))
        yaw = float(self._tune.get("wiggle_yaw", LINE_WIGGLE_YAW))
        return int(first * step * amp), int(first * step * yaw)

    def _travel_wiggle(self, goal: dict) -> tuple[int, int]:
        """주행 중 주기적으로 옆으로 톡 — 굴러가는 상태를 유지시킨다.

        사용자가 찾아낸 방법 그대로다: "중간중간 좌우 이동을 잠깐 섞어주면
        움직인다". 옆으로 치는 순간 바퀴가 풀린다.

        정렬의 전후 흔들기와 **같은 원리, 반대 축**이다. 펄스마다 부호를
        뒤집으므로 횡방향 변위는 서로 상쇄된다 — 라인에서 밀려나지 않는다.
        """
        amp = float(self._tune.get("travel_wiggle", LINE_TRAVEL_WIGGLE))
        if amp <= 0 and float(self._tune.get("wiggle_yaw", LINE_WIGGLE_YAW)) <= 0:
            return 0, 0
        t = time.monotonic() - goal.get("started", 0.0)
        on = (t % LINE_TRAVEL_WIGGLE_PERIOD) < LINE_TRAVEL_WIGGLE_ON
        # ⚠ 부호를 벽시계(주기 번호)로 정하면 틱이 밀리거나 감속으로 주기를
        #   건너뛸 때 같은 부호가 연달아 나온다 — 실측 +9틱/-5틱 불균형이 곧
        #   횡방향 순수 충격량이 되어 주행 중 dy가 표류했다(사진 3: 256px).
        #   **실제로 낸 펄스 수**로 교대해야 상쇄가 보장된다.
        if on and not self._wiggle_on:
            self._wiggle_n += 1
        self._wiggle_on = on
        if not on:
            return 0, 0
        # ⚠ 교대라 결국 상쇄되지만 **첫 반 주기는 한쪽으로 나간다** — 그쪽이 토마토
        #   나무면 매번 부딪힌다(2026-08-13 현장). 첫 펄스(_wiggle_n==1) 방향을
        #   wiggle_sign으로 정하고, 그 뒤로 교대한다.
        step = 1.0 if self._wiggle_n % 2 == 1 else -1.0
        return self._shake(step, amp)

    def _cross_shake(self) -> tuple[int, int]:
        """굳음 해제용 **직각 흔들기**. 첫 반 주기는 wiggle_sign 쪽으로 나간다.

        예전엔 부호를 벽시계로 정했다(`int(monotonic() * 6) % 2`). 교대는 됐지만
        **흔들기가 시작되는 순간의 부호가 매번 달랐다** — 운이 나쁘면 토마토 나무
        쪽으로 먼저 밀고 부딪혔다(2026-08-13 현장). 흔들기가 시작된 시점을
        _mark_break_until에서 역산해 기준으로 삼으면 항상 같은 쪽부터 나간다
        (새 상태를 들고 다닐 필요가 없다).
        """
        elapsed = max(0.0, LINE_MARK_BREAK_SEC - (self._mark_break_until - time.monotonic()))
        step = 1.0 if int(elapsed * 6) % 2 == 0 else -1.0
        return self._shake(step, LINE_MARK_BREAK_CROSS)

    def _yaw_pulse(self, line: dict) -> int:
        """각도 오차 → **한 번에 그만큼 도는** 회전 펄스. 데드밴드 안이면 0.

        ⚠ 예전엔 게걸음과 같은 _corr_pulse를 거쳤다. 그래서 크기가 corr_min~corr_max
        (현장 60~75)로 잘렸는데, **75는 로봇을 1도도 못 돌린다.** 2026-08-17 실측
        (제자리, 펄스 0.12s): w=75 → 0.00° · 100 → 0.45° · 120 → 1.1° · 140 → 1.95° ·
        160 → 3.20° · 180 → 4.35°. 정지마찰 문턱이 w≈90이다.
        회전 보정은 매 펄스 나가고 있었지만 **물리적으로 아무 일도 없었다** —
        "각도 부호를 뒤집든 말든 똑같다"가 그 뜻이었다(0에 부호를 붙여도 0).

        이제 크기는 실측 곡선의 **역함수**로 낸다: deg ≈ 0.043 × (w − 90)이므로
        오차만큼 돌리려면 w = 90 + 오차/0.043. 한 펄스로 오차를 덮는 게 목표다.
        게걸음 슬라이더(corr_min/corr_max)는 이제 이 축에 닿지 않는다 — 축이 다르면
        정지마찰도 다르다.
        """
        err = line.get("angle_deg")
        if err is None or not self._tune.get("yaw_gain"):
            return 0                      # 회전 보정 꺼짐(대시보드 토글)
        if abs(err) < LINE_YAW_DEADBAND:
            return 0
        want = LINE_YAW_STICTION + abs(err) / LINE_YAW_DEG_PER_UNIT
        mag = int(max(LINE_YAW_PULSE_MIN, min(LINE_YAW_PULSE_MAX, want)))
        # 부호 규칙은 예전과 같다(실측으로 확정된 것): yaw_sign×오차가 양수면 +.
        return mag if (self._tune["yaw_sign"] * err) > 0 else -mag

    def _shake_toward_line(self, cross: int, corr_now: int) -> int:
        """굳음 해제 흔들기에서 **기준선에서 멀어지는 반 주기**를 잘라낸다.

        흔들기는 부호를 교대해 순변위를 상쇄하는 게 원칙이다 — 정렬하는 동안
        위치가 밀려나지 않게 하려는 것이라 평소엔 그게 맞다. 그런데 **이미 크게
        벗어나 있으면 상쇄가 미덕이 아니다.** 멀어지는 쪽 반 주기가 오차를 그대로
        키운다(2026-08-17 현장: 지점 정렬 중 dy=-312px에서 양쪽으로 나갔다).

        그래서 |dy|가 LINE_SHAKE_ONE_WAY_PX를 넘으면 **가까워지는 쪽만** 낸다.
        방향은 같은 축의 보정 지령(corr_now) 부호로 판단한다 — 그게 "지금 어디로
        가야 기준선에 붙는가"의 답이고, 부호 설정([거리 부호 ±])도 거기 반영돼 있다.
        회전 흔들기는 자르지 않는다: Y를 안 움직이면서 정지마찰은 더 잘 푼다.
        """
        dy = (self._line or {}).get("offset_y_px")
        if dy is None or abs(dy) <= LINE_SHAKE_ONE_WAY_PX or not corr_now or not cross:
            return cross
        return cross if (cross > 0) == (corr_now > 0) else 0

    def _align_dither(self, goal: dict) -> int:
        """정렬 펄스에 얹을 **전후 흔들기**. 펄스마다 부호가 뒤집힌다.

        ⚠ 2026-08-11 실기: 게걸음 지령을 줘도 로봇이 **안 움직인다**. 메카넘의
        횡이동은 롤러가 옆으로 굴러야 생기는데, 정지 상태에서는 롤러의 정지마찰이
        그걸 막는다. 바퀴가 이미 굴러가고 있으면 마찰이 깨져 옆으로 밀린다
        (사용자 관찰: "살짝 전진 후진하면서 하면 좌우로 조금은 움직여지더라").

        펄스마다 앞·뒤를 번갈아 주므로 전후 변위는 서로 상쇄된다 — 정렬하는
        동안 진행 방향 위치가 밀려나지 않는다(끝나는 시점에 한 펄스분만 남는다).
        """
        amp = float(self._tune.get("align_dither", LINE_ALIGN_DITHER))
        if amp <= 0:
            return 0
        period = max(1e-3, float(self._tune.get("pulse_period", LINE_PULSE_PERIOD)))
        n = int((time.monotonic() - goal.get("started", 0.0)) / period)
        return int(amp) if n % 2 == 0 else -int(amp)

    def _command(self, line: dict, travel_dir: int, speed: int,
                 mode: str = "align", dither: int = 0) -> tuple[int, int, int]:
        """(vx, vy, w) 지령을 만든다.

        테이프까지의 거리 오차는 **횡방향**이라 보정축(기본 vy)에 싣고, 진행은
        나머지 축에 싣는다. 둘은 항상 직교해야 하므로 한쪽이 정해지면 다른 쪽도 정해진다.

        ⚠ mode의 기본값이 "align"인 이유: 이 함수는 상태표시용 `_would`를 만들 때도
        불리는데, 거기서는 **거리 보정이 어느 방향으로 나갈지**를 보여주는 게 목적이라
        (부호 검증) 게걸음 금지와 무관하게 항상 계산해야 한다.
        """
        corr = w = 0
        driving = mode not in ("align", "align_mark")
        if line.get("found"):
            if self.smooth and driving:
                # ★ 저속 연속 주행 — 바퀴가 구르는 중이라 롤러 정지마찰이 이미
                #   깨져 있다. 그래서 vy·w를 **비례제어**로 부드럽게 계속 준다
                #   (사람이 키를 눌러 몰 때와 같은 방식). 게걸음 금지는 여기서
                #   의미가 없다 — 어차피 전진하면서 옆으로 미는 것이라 정지
                #   상태의 순수 횡이동처럼 힘이 들지 않는다.
                # 게걸음(vy)의 세기·상한은 대시보드가 정한다. 회전(w)은 아래에서
                # 예전 상한을 그대로 쓴다 — "게걸음 최대"는 게걸음만 뜻해야 한다.
                corr = self._corr_smooth(line.get("offset_y_norm"),
                                         float(self._tune.get("smooth_dy_gain",
                                                              LINE_SMOOTH_DY_GAIN)),
                                         self._tune["dy_sign"], LINE_DY_DEADBAND,
                                         cap=self._tune.get("corr_max", LINE_MAX_CORRECTION))
                if self._tune["yaw_gain"]:
                    w = self._corr_smooth(line.get("angle_deg"), LINE_SMOOTH_YAW_GAIN,
                                          self._tune["yaw_sign"], LINE_YAW_DEADBAND)
            else:
                # ★ 펄스(톡톡) — 멈춰 있는 상태에서 쓰는 방식. 작은 값을 계속
                #   흘리면 정지마찰을 못 넘어 아무 일도 안 일어나므로, 확실히
                #   움직이는 크기로 짧게 주고 멈춰서 정착시킨 뒤 다시 잰다.
                #   게걸음 금지면 주행 중에는 거리 보정을 빼고, 그건 정렬이 잡는다.
                if not (self.no_strafe and driving):
                    corr = self._corr_pulse(line.get("offset_y_norm"), LINE_DY_GAIN,
                                            self._tune["dy_sign"], LINE_DY_DEADBAND)
                # 회전은 **전용 크기**로 낸다 — 게걸음 슬라이더에 잘리면 안 돈다.
                w = self._yaw_pulse(line)
        # 흔들기는 진행축에 그대로 얹는다(부호 뒤집기는 호출자가 이미 했다).
        travel = int(self._tune["travel_sign"] * travel_dir * speed) + dither
        travel = max(-255, min(255, travel))
        if self._tune["dy_axis"] == "vy":
            return travel, corr, w      # 보정=게걸음, 진행=전후
        return corr, travel, w          # 보정=전후,   진행=게걸음

    # ------------------------------------------------------------------
    # 게걸음 금지 모드의 "잠시 정렬하고 오기"
    # ------------------------------------------------------------------

    def _maybe_detour(self, line: dict, mode: str) -> bool:
        """거리 오차가 커졌으면 주행을 접고 정렬로 전환. 전환했으면 True.

        게걸음을 안 쓰기로 했으니 주행 중에는 거리 오차를 되돌릴 수단이 없다.
        사람이 [정렬]을 누를 때까지 기다리면 데모가 멈추므로 스스로 다녀온다.
        """
        # 저속 연속 주행은 vy 보정을 계속 넣으므로 우회할 이유가 없다.
        if self.smooth or not self.no_strafe or mode not in DETOUR_MODES:
            return False
        dy = line.get("offset_y_norm")
        if dy is None or abs(dy) < LINE_STRAFE_FIX_AT:
            return False
        if self._detours >= LINE_MAX_DETOURS:
            # 계속 벗어난다면 보정 부호나 코스 쪽 문제다 — 무한 왕복하지 않는다.
            self.cancel(f"거리 보정을 {LINE_MAX_DETOURS}번 했는데도 계속 벗어납니다 — "
                        "/settings에서 [거리 부호 ±]를 확인하세요")
            return True
        return self._fold_to_align(mode, "거리 보정 중(톡톡)")

    def _fold_to_align(self, mode: str, why: str) -> bool:
        """주행을 접어두고 **정렬(톡톡)**로 전환한다. 전환했으면 True.

        정렬이 성공하면 _resume_travel이 접어둔 주행을 그대로 이어간다(실패면
        재개하지 않는다 — 어긋난 채로 달리면 안 되므로). 우회 횟수를 다 썼거나
        우회할 수 없는 모드면 False를 돌려주고, 부르는 쪽이 멈춘다.
        """
        if mode not in DETOUR_MODES or self._detours >= LINE_MAX_DETOURS:
            return False
        now = time.monotonic()
        with self._lock:
            self._resume = (mode, dict(self._goal), self._deadline - now)
            self._detour_started = now
            self._detours += 1
            self._mode = "align"
            self._goal = {"speed": self._goal.get("speed", LINE_SPEED), "started": now}
            self._deadline = now + LINE_ALIGN_TIMEOUT_SEC
            self._detail = f"{why} #{self._detours} — 끝나면 주행을 계속합니다"
            self._align_worst = None
            self._align_diverge = 0
            self._align_gate = True
        return True

    #: 이 말이 들어간 사유는 "도착"이 아니다 — 연쇄를 이어가면 안 된다.
    #  (시퀀스 러너가 실패를 가려내는 낱말과 같은 목록이다 — 판단이 갈리면 안 된다.)
    FAIL_WORDS = ("정지", "실패", "초과", "취소", "중단")

    def _chain_continue(self, mode: str, reason: str) -> bool:
        """한 칸 도착했으니 다음 칸을 이어서 시작한다. 이어갔으면 True.

        **중간 지점에서는 정렬 전에** 불린다(정렬을 건너뛰고 곧장 다음 칸으로 —
        칸마다 정렬하면 너무 느리다). 최종 목적지에서만 도착 정렬이 돌고, 그 뒤
        여기 오면 남은 칸이 없어 그대로 멈춘다.

        실패로 끝났으면 이어가지 않는다: 어긋난 채로 계속 달리면 그 오차를
        그대로 안고 다음 칸까지 간다.
        """
        if self._chain_target is None or mode not in ("station", "align_mark"):
            return False
        if any(bad in reason for bad in self.FAIL_WORDS):
            with self._lock:
                self._chain_target = None
            return False
        final = self._chain_target
        if self._station is None or self._station == final:
            with self._lock:
                self._chain_target = None
            return False
        return self._start_leg(final)

    def _resume_travel(self) -> None:
        """정렬을 마쳤다 — 접어뒀던 주행을 이어서 재개."""
        now = time.monotonic()
        with self._lock:
            if self._resume is None:
                return
            mode, goal, remain = self._resume
            self._resume = None
            # ⚠ 시간 기반 주행(travel/jog)은 **실제로 굴러간 시간**만 세야 한다.
            #   정렬하느라 서 있던 시간이 이동 시간에 포함되면 "1.5초 이동"이
            #   재개하자마자 끝나버린다. 펄스 위상도 같이 밀어줘야 톡톡이 안 끊긴다.
            if "started" in goal:
                goal["started"] += now - self._detour_started
            self._mode = mode
            self._goal = goal
            self._deadline = now + max(1.0, remain)
            self._detail = f"거리 보정 완료 (#{self._detours}) — 주행 재개"

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
            # 끝점은 **주황 색 마커**다. 예전 코스(원목)에선 검은 테이프였고 그 검출은
            # 극성이 dark면 꺼지므로, 그것만 보면 영영 목표를 못 만나 타임아웃까지
            # 밀고 간다 — 실제로 "끝점인데 쭈욱 넘어간다"가 이것이었다(2026-08-10).
            if self._end_marker_centered(line):
                self._latch_end(goal["side"])
                return f"{'왼쪽' if goal['side'] == 'left' else '오른쪽'} 끝점 도착(변위 기준 갱신)"
            legacy = line.get("end_marker")   # 원목 코스(검은 테이프) 호환
            if legacy and legacy.get("side") == goal["side"] \
                    and abs(legacy["x"] - width / 2) < tol:
                self._latch_end(goal["side"])
                return f"{'왼쪽' if goal['side'] == 'left' else '오른쪽'} 끝 도착(변위 기준 갱신)"
            return None

        if mode == "align_mark":
            if not line.get("found"):
                return None      # 분실 중엔 판정 유보 — 길어지면 _safety_lost가 세운다
            got = self._nearest_station_marker(line)
            if got is None:
                return None      # 깜빡임일 수 있다 — 타임아웃(12s)이 안전망
            marker, dx = got
            centered = abs(dx) <= self._tune.get("align_tol_x", LINE_MARK_ALIGN_TOL_PX)
            level = self._align_error(line)          # dy·yaw 오차(데드밴드 배수)
            now = time.monotonic()

            # ★ 한 프레임의 "중앙"을 믿지 않는다. 조건을 만족하면 먼저 **완전히
            #   세우고** LINE_MARK_SETTLE_SEC만큼 기다린 뒤 **다시 잰다** —
            #   관성으로 더 나아가 결국 어긋나던 문제(2026-08-12 보고)의 처방이다.
            #   재서 벗어나 있으면 정렬을 이어간다(정렬이 "또 실행"되는 셈).
            if self._mark_settle_until:
                if now < self._mark_settle_until:
                    return None          # 정착 대기 중 — 지령은 _pulse_gate가 0으로
                self._mark_settle_until = 0.0
                self._mark_verifies += 1
                if not (centered and level <= 1.0):
                    # 관성으로 밀렸다 — 기준을 새로 잡고 계속 맞춘다.
                    self._mark_best, self._mark_stall = None, 0
                    self._detail = (f"정착 후 재측정 — {abs(dx):.0f}px 밀림, "
                                    f"다시 맞춥니다 (#{self._mark_verifies})")
                    return None
                kind = self._marker_kind(marker)
                name = {"end": "끝점(주황)", "mid": "중간지점(노랑)"}.get(kind, "지점")
                extra = f" · 흔들어 풀기 {self._mark_breaks}회" if self._mark_breaks else ""
                extra += f" · 정착확인 {self._mark_verifies}회"
                return (f"지점 정렬 완료 — {name} 중앙 ±{abs(dx):.0f}px · "
                        f"dy {line.get('offset_y_px') or 0:.0f}px{extra}")

            if centered and level <= 1.0:
                # 조건은 맞았다 — 그러나 지금은 굴러가는 중일 수 있다. 세우고 본다.
                self._mark_settle_until = now + LINE_MARK_SETTLE_SEC
                self._detail = f"정착 대기 {LINE_MARK_SETTLE_SEC:.1f}초 — 관성이 죽길 기다립니다"
                return None

            # ★ 여기서 **포기하지 않는다.** 예전엔 오차가 몇 번 안 줄면 끝냈고,
            #   그래서 사람이 [지점 정렬]을 3번 이상 눌러야 했다(2026-08-12 보고).
            #   못 줄이는 이유는 대개 "갈 수 없어서"가 아니라 **정지마찰에 걸려서**다.
            #   그러면 떼어내면 된다 — 굳음이 이어지면 큰 충격을 넣고 계속한다.
            gate = self._pulse_gate(goal, mode)
            edge = gate and not self._align_gate
            self._align_gate = gate
            if edge and time.monotonic() >= self._mark_break_until:
                # 지금 막힌 축이 어디인가 — 마커가 안 맞으면 진행축, 맞았으면 보정축.
                axis = "travel" if not centered else "corr"
                metric = abs(dx) if not centered else level
                # 축이 바뀌면(1단계 끝) 기준을 새로 잡는다 — 단위가 다르다.
                if axis != self._mark_break_axis:
                    self._mark_break_axis, self._mark_best, self._mark_stall = axis, None, 0
                improved = self._mark_best is None or metric < self._mark_best - (
                    8.0 if axis == "travel" else 0.05)
                if improved:
                    self._mark_best, self._mark_stall = metric, 0
                else:
                    self._mark_stall += 1
                    if self._mark_stall >= LINE_MARK_STALL_LIMIT:
                        self._mark_break_until = time.monotonic() + LINE_MARK_BREAK_SEC
                        self._mark_breaks += 1
                        self._mark_stall = 0
                        self._mark_best = None    # 충격 뒤엔 다시 재본다
                        self._detail = (
                            f"지점 정렬 — {'마커가' if axis == 'travel' else '기준선이'} "
                            f"굳어 흔들어 풀기 #{self._mark_breaks}")
                        print(f"  [line] 지점정렬 굳음({axis}) — 흔들어 풀기 #{self._mark_breaks}")
            return None

        if mode == "align":
            # ⚠ 테이프가 안 보이면 완료 선언 금지. _align_error는 None을 0으로
            #   보므로, 분실 순간에 "오차 0 = 정렬 완료"가 되고 그 메시지의
            #   f"{None:.0f}" 포맷이 **스레드를 죽였다**(2026-08-11 21:46 실사고
            #   — 이후 모든 버튼이 영원히 무반응, 화면은 낡은 스냅샷 유지).
            if not line.get("found"):
                return None      # 분실이 길면 _safety_lost가 세운다
            err = self._align_error(line)
            if err <= 1.0:
                return (f"정렬 완료 — dy {line.get('offset_y_px') or 0:.0f}px"
                        f" ∠{line.get('angle_deg') or 0:.1f}°")
            # ⚠ 2026-08-11 수정. 예전엔 "OFF 구간이면" 발산을 셌는데, 이 루프는
            #   50Hz라 OFF 한 번(0.12초)이 **6틱**이다. 정지 중엔 오차가 변할 리
            #   없으니 3틱 만에 한도(3)를 채워 **첫 펄스도 끝나기 전에 항상
            #   "정렬 실패, 부호가 반대일 수 있습니다"** 를 뱉었다. 멀쩡한 부호를
            #   뒤집게 만드는 가짜 경고였다.
            #   → 펄스 **경계에서 한 번만** 센다. 그것도 OFF→ON 전환 시점에:
            #     그때가 직전 펄스의 결과를 카메라가 따라잡은 뒤라 가장 정확하다
            #     (센싱 지연이 100~200ms라 OFF 진입 직후 값은 아직 옛날 것이다).
            gate = self._pulse_gate(goal, mode)
            edge = gate and not self._align_gate      # OFF → ON 전환
            self._align_gate = gate
            if edge:
                if self._align_worst is None or err < self._align_worst - 0.02:
                    self._align_worst, self._align_diverge = err, 0
                else:
                    self._align_diverge += 1
                    if self._align_diverge >= LINE_ALIGN_DIVERGE_LIMIT:
                        return (f"정렬 실패 — 펄스 {LINE_ALIGN_DIVERGE_LIMIT}번을 쳤는데 "
                                "오차가 안 줄었습니다. 보정 부호가 반대일 수 있습니다: "
                                "/settings에서 [거리 부호 ±] 또는 [회전 부호 ±]를 눌러보세요.")
            return None

        if mode == "station":
            # _observe_markers가 통과 때마다 인덱스를 갱신한다 — 목표에 닿으면 끝.
            if self._station == goal["target"]:
                return f"{LINE_STATION_LABELS[goal['target']]} 도착"
            return None

        if mode == "next_mark":
            # **지점**(주황/노랑)에서만 멈춘다 — 초록 경유 표식은 지점이 아니므로
            # 여기서 멈추면 지점 이동이 절반에서 끝나버린다.
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
            # ⚠ 이 스레드는 절대 죽으면 안 된다. 죽으면 라인 주행 전체가 침묵하는데
            #   상태 API는 마지막 스냅샷을 계속 내보내서 **화면만 봐서는 모른다**
            #   (2026-08-11: 포맷 예외 하나로 스레드가 죽고, 이후 40분간 "테이프
            #   없음(낡은 사유)" 고정 + 버튼 전부 무반응). 예외는 정지+기록으로
            #   바꾸고 루프는 계속 돈다 — linkmon 샘플러와 같은 원칙.
            try:
                self._tick_once()
            except Exception as exc:  # noqa: BLE001 - 제어 스레드의 마지막 방벽
                print(f"  [line] 루프 예외(복구됨): {exc!r}")
                try:
                    self.cancel(f"내부 오류로 정지: {exc}")
                except Exception:  # noqa: BLE001
                    pass
                time.sleep(self.RATE)

    def _tick_once(self) -> None:
            line = self._read_line()
            if line:
                self._line = line
                if line.get("found"):
                    self._found_ts = time.monotonic()
            with self._lock:
                mode, goal = self._mode, dict(self._goal)

            # 정지 중에도 보정값을 계산해 둔다(부호 검증용 — 진행분은 빼고 보정만)
            # 지금 방식이 실제로 낼 보정값을 보여준다(부호 검증 + 크기 감각).
            # 펄스 방식일 때는 "align"으로 계산해야 게걸음 금지에 가려지지 않는다.
            self._would = self._command(self._line, 0, 0,
                                        "travel" if self.smooth else "align")
            # 마커 통과는 **주행 중이 아니어도** 센다(손으로 밀어도 인덱스가 따라온다).
            # ⚠ 순서 중요: 방향을 **먼저** 갱신해야 한다. 마커를 지나는 그 순간의
            #   방향으로 "어느 끝인지"가 정해지므로, 옛 방향으로 세면 반대로 찍힌다.
            if self._mode == "idle":
                self._track_direction(self._line)
            else:
                self._odom_prev = None   # 자동주행 중엔 명령이 방향의 근거다
            self._observe_markers(self._line)
            # 끝점 위에 서 있기만 하면 번호가 스스로 확정된다(주행 중이 아니어도).
            self._anchor_at_end(self._line)

            # 끝단 마커는 **주행 중이 아니어도** 지나가면 기준을 잡는다.
            end = (self._line or {}).get("end_marker")
            if end and abs(end["x"] - (self._line.get("width") or 1280) / 2) < 60:
                if self._last_end != end["side"]:
                    self._latch_end(end["side"])

            if mode == "idle":
                time.sleep(self.RATE)
                return

            # 순서가 곧 정책이다:
            #   ① 테이프 분실 → 무조건 정지(눈 감고 달리지 않는다)
            #   ② 목표 도달 / 시간 초과
            #   ③ **벗어났으면 정렬로 붙여본다** ← 정지보다 먼저 시도한다
            #   ④ 그래도 안 되면(정렬 불가·횟수 소진) 그때 정지
            # ③을 ④보다 먼저 두는 게 핵심이다. 예전엔 ④가 먼저라 크게 벗어나면
            # 붙여볼 기회도 없이 멈췄고, 멈춘 뒤에는 정렬도 잠겨 있었다.
            stop_reason = self._safety_lost(self._line)
            reached = None
            if stop_reason is None:
                reached = self._goal_reached(self._line, goal, mode)
                stop_reason = reached
            if stop_reason is None and time.monotonic() > self._deadline:
                stop_reason = "시간 초과 — 목표를 못 찾아 정지"

            # 한 칸씩 가는 중 **중간** 지점이면 정렬을 건너뛰고 곧장 다음 칸으로.
            # 칸마다 정렬하면 한 번에 수 초씩 붙어 전체가 너무 느려진다(현장 요청) —
            # 정렬은 **최종 목적지에서 한 번**이면 된다. 중간은 마커를 지났다는
            # 사실만 있으면 되고, 그건 정렬 없이도 _observe_markers가 센다.
            if (reached and self._chain_target is not None
                    and self._station != self._chain_target):
                if self._chain_continue(mode, reached):
                    time.sleep(self.RATE)
                    return

            # 지점 도착이면 멈추지 않고 **지점 정렬(마커 중앙)**로 이어간다.
            # 안전정지·타임아웃(reached가 아님)은 그대로 선다 — 도착만 이어간다.
            if (reached and LINE_ARRIVE_ALIGN and mode in ARRIVE_ALIGN_MODES
                    and self._nearest_station_marker(self._line) is not None):
                self._begin_arrive_align(reached)
                time.sleep(self.RATE)
                return

            if stop_reason:
                # 접어뒀던 주행이 있으면, 정렬이 **성공했을 때만** 이어서 재개한다.
                # 실패(발산)나 시간 초과인데도 재개하면 벗어난 채로 계속 달린다.
                if mode == "align" and self._resume is not None:
                    if self._align_error(self._line) <= 1.0:
                        self._resume_travel()
                        time.sleep(self.RATE)
                        return
                    self._resume = None
                    stop_reason = f"{stop_reason} — 주행 재개를 취소했습니다"
                # 한 칸씩 가는 중이면 **여기서 다음 칸을 시작한다.** cancel로 가면
                # mode가 idle이 되고, 그걸 시퀀스 러너가 "도착"으로 읽어 팔이 나간다.
                if self._chain_continue(mode, stop_reason):
                    time.sleep(self.RATE)
                    return
                self.cancel(stop_reason)
                time.sleep(self.RATE)
                return

            # 게걸음 금지 모드: 거리가 벌어졌으면 주행을 접고 정렬하러 간다.
            if self._maybe_detour(self._line, mode):
                time.sleep(self.RATE)
                return

            # 너무 벗어났다 — 예전엔 여기서 멈추고 사람에게 "[정렬(톡톡)]을 누른 뒤
            # 다시 시도하세요"라고 떠넘겼다. 그 정렬을 **스스로 한다**(현장 요청):
            # 주행을 접고 톡톡으로 붙인 뒤 이어서 간다. 횟수를 다 썼을 때만 선다.
            stop_reason = self._safety_dy(self._line, mode)
            if stop_reason:
                if self._fold_to_align(mode, f"{stop_reason} → 스스로 정렬(톡톡)"):
                    time.sleep(self.RATE)
                    return
                # 여기 왔으면 스스로 붙이기를 다 해본 뒤다 — 사람에게 남길 말은
                # "정렬을 눌러라"가 아니라 "왜 안 붙는지 보라"여야 한다.
                self.cancel(
                    f"{stop_reason} — 스스로 정렬을 {self._detours}번 해봤지만 "
                    "못 붙였습니다. 보정 부호가 반대이거나(/settings [거리 부호 ±]) "
                    "기준선이 잘못 잡혔을 수 있습니다([현재 위치를 기준선으로])")
                time.sleep(self.RATE)
                return

            speed = goal.get("speed", LINE_SPEED)
            if goal.get("side") in ("left", "right"):
                self._last_dir = 1 if goal["side"] == "right" else -1
            # ★ 접근 감속 — 과도하게 넘어가는 것의 1차 처방(2026-08-11 사진 3·5·7).
            #   순항 속도 그대로 마커 중앙을 지나면 센싱 지연 + 펌웨어 감속 슬루
            #   동안 그대로 밀려 목표를 지나쳐 선다. 다음에 만날 마커(_mark_armed)가
            #   접근 창에 들어오면 거리에 비례해 늦춘다 — 도착 순간엔 거의 기어간다.
            # ⚠ 지점 간격(≈375px)이 이 창(640px)보다 좁아서, 실질적으로 **주행
            #   내내 감속 구간**이다 — 출발 속도가 곧장 하한(LINE_APPROACH_MIN)까지
            #   깎인다. 그래서 출발 킥이 더 중요해졌다(아래 _travel_kick 주석).
            if self.smooth and mode in DETOUR_MODES and self._mark_armed:
                got = self._nearest_station_marker(self._line, ahead_dir=self._last_dir)
                if got is not None:
                    width = (self._line or {}).get("width") or 1280
                    # 감속 시작 = 마커가 **보이는 순간**(반화면). 접근 창(30%)까지
                    # 기다리면 이미 늦다 — 순항에서 하한까지 내려올 거리가 모자라다
                    # ("테이프 보이는 순간 속도를 더 줄여서 미세 조정" — 사용자 제안).
                    zone = width * 0.5
                    adx = abs(got[1])
                    if adx < zone:
                        speed = int(max(LINE_APPROACH_MIN, speed * adx / zone))
            # OFF 구간엔 **보정까지 전부 0** — 완전히 멈춰 정착시킨 뒤 다시 잰다.
            if self._pulse_gate(goal, mode):
                phase1 = False
                if mode == "align":
                    travel, use = 0, speed
                elif mode == "align_mark":
                    # 진행축 펄스가 마커의 dx를 향한다 — 지나쳤으면 되돌아온다.
                    travel, use = self._mark_pulse(self._line)
                    phase1 = bool(travel)   # 1단계: 마커부터. 보정은 그 다음.
                else:
                    travel, use = self._travel_dir(goal), speed
                # 정렬은 제자리 동작이지만 **완전히 제자리면 롤러가 안 미끄러진다.**
                # 전후로 살짝 흔들어 정지마찰을 깨준다(펄스마다 부호 반전 → 상쇄).
                # 지점 정렬도 진행축 펄스가 없는 국면(마커는 중앙, dy만 남음)엔 같다.
                dither = (self._align_dither(goal)
                          if (mode == "align" or (mode == "align_mark" and not travel))
                          else 0)
                # 저속 연속 주행은 반대 문제를 갖는다 — vx만으로는 바퀴가 안 풀린다.
                # 출발엔 세게 한 방, 주행 중엔 주기적으로 옆으로 톡.
                wiggle = wiggle_w = 0
                if self.smooth and mode not in ("align", "align_mark"):
                    # ⚠ 예전엔 `if not braking:`으로 감속 중 킥을 막았다. 그 전제가
                    #   "감속 중이면 이미 구른다"였는데 **이 코스에선 거짓이다.**
                    #   감속은 마커가 반화면(640px) 안에 들어오면 시작되는데 지점
                    #   간격이 ≈375px이라, **출발하는 순간 이미 감속 구간**이다.
                    #   그래서 킥은 도달할 수 없는 코드였고, 슬라이더를 올려도
                    #   아무 일도 안 났다(2026-08-13). 심지어 출발 속도는 감속으로
                    #   하한(60)까지 깎이니, 정지마찰을 깰 수단이 하나도 없었다.
                    #   막을 필요도 없다 — _travel_kick은 오도메트리가 25px 움직인
                    #   순간(=실제로 굴렀다는 증거) 스스로 끝나고, 한 명령에 한 번만
                    #   난다. 즉 "구르는 중에 감속을 이기는" 상황은 원래 생기지 않는다.
                    use = self._travel_kick(goal, use)
                    wiggle, wiggle_w = self._travel_wiggle(goal)
                vx, vy, w = self._command(self._line, travel, use, mode, dither)
                if mode == "align_mark" and time.monotonic() < self._mark_break_until                         and self._mark_break_axis == "corr":
                    # ★ 보정축(기준선 거리)이 굳었다 — 그 축을 최대로 밀면서
                    #   **직각축을 크게 흔든다.** 구르는 바퀴라야 옆으로 미끄러진다
                    #   (정렬 dither·주행 wiggle과 같은 원리, 여기선 크기를 키운 것).
                    strong = LINE_MARK_BREAK_MAG
                    cross, cross_w = self._cross_shake()
                    corr_now = vy if self._tune["dy_axis"] == "vy" else vx
                    signed = strong if corr_now >= 0 else -strong
                    if self._tune["dy_axis"] == "vy":
                        vy, vx = signed, cross
                    else:
                        vx, vy = signed, cross
                    # 회전도 함께 흔든다 — 네 바퀴가 다 돌아 정지마찰이 가장 잘 풀린다.
                    # ⚠ 흔들기에 회전 성분이 없으면(wiggle_yaw=0, 현장 기본값)
                    #   **회전 보정을 지우지 않는다.** 예전엔 무조건 덮어써서 굳음 해제
                    #   구간 내내 각도가 방치됐다(실기: 그 구간에서 -2°→7°로 벌어짐).
                    if cross_w:
                        w = cross_w
                elif (mode == "align_mark" and phase1
                      and time.monotonic() < self._mark_break_until):
                    # 진행축이 굳었다 — 크기는 그대로 두고 **직각축을 흔들어**
                    #   바퀴를 굴린다. 구르기 시작하면 같은 톡톡으로도 나간다.
                    # ⚠ 이 직각축이 곧 **기준선까지의 거리축(Y)** 이다. 기준선에서
                    #   크게 벗어나 있으면 멀어지는 반 주기를 자른다 — 안 그러면
                    #   흔들다가 오차를 더 키운다(현장: dy -312px).
                    cross, cross_w = self._cross_shake()
                    if self._tune["dy_axis"] == "vy":
                        vy = self._shake_toward_line(cross, vy)
                    else:
                        vx = self._shake_toward_line(cross, vx)
                    if cross_w:          # 위와 같은 이유 — 회전 보정을 지우지 않는다
                        w = cross_w
                elif mode == "align_mark" and phase1:
                    # ★ 한 번에 한 축만. 진행 펄스에 dy/yaw 보정까지 얹으면
                    #   대각선으로 왔다갔다해 "너무 요란하게" 보인다(실기 보고).
                    #   마커를 먼저 중앙에 → 그 다음 dy/yaw(2단계, travel=0)로.
                    if self._tune["dy_axis"] == "vy":
                        vy = 0
                    else:
                        vx = 0
                    w = 0
                if wiggle:
                    # 좌우 톡은 **보정축**에 얹는다(진행축이 아니다).
                    if self._tune["dy_axis"] == "vy":
                        vy = max(-255, min(255, vy + wiggle))
                    else:
                        vx = max(-255, min(255, vx + wiggle))
                if wiggle_w:
                    # 회전 흔들기도 같이 — 부호가 교대라 순회전은 상쇄된다.
                    w = max(-255, min(255, w + wiggle_w))
            else:
                vx = vy = w = 0
            try:
                self._base.hold(vx, vy, w)
            except Exception as exc:  # noqa: BLE001 - 링크가 죽어도 루프는 살아야 함
                self.cancel(f"주행 실패: {exc}")
            time.sleep(self.RATE)

    def _safety_lost(self, line: dict) -> str | None:
        """눈 감고 달리지 않기 — 테이프를 놓치면 정지. 어떤 모드에서도 최우선."""
        now = time.monotonic()
        if not line.get("found"):
            if self._lost_since is None:
                self._lost_since = now
            elif now - self._lost_since > LINE_LOST_STOP_SEC:
                return "테이프를 놓쳐 정지 — 카메라가 띠를 보게 한 뒤 다시 시도"
            return None
        self._lost_since = None
        return None

    def _safety_dy(self, line: dict, mode: str) -> str | None:
        """너무 벗어났을 때의 마지막 방어선.

        ⚠ 정렬(align/align_mark)은 **면제**다. 정렬은 벗어남을 되돌리는 동작이라 여기에
        걸면 복구가 스스로 막힌다(2026-08-11 실사고). 정렬의 폭주 방지는
        발산 감지 — 펄스를 쳐도 오차가 안 줄면 스스로 포기한다.

        주행에서도 이 정지는 **정렬 우회를 먼저 시도한 뒤**에만 도달한다
        (_run 순서 참고). 즉 "붙일 수 있으면 붙여보고, 그래도 안 되면 선다".
        """
        if mode in ("align", "align_mark"):
            return None
        dy = abs(line.get("offset_y_norm") or 0.0)
        if dy > LINE_MAX_DY_NORM:
            # ⚠ 여기서는 **사실만** 돌려준다. 이 사유는 두 곳에 쓰인다:
            #   ① 스스로 정렬로 접어들 때의 설명  ② 끝내 못 붙여 멈출 때의 사유.
            #   ②의 해설("몇 번 해봤지만…")을 여기에 넣으면 ①에도 따라붙어,
            #   붙이는 중인데 "못 붙였습니다"라고 뜬다(실측으로 확인).
            return f"기준선에서 너무 벗어남(dy={line.get('offset_y_px')}px)"
        return None
