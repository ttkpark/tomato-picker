"""팔 프리셋과 **지점 이동**을 섞어 실행하는 시퀀스 러너.

왜 필요한가 — 수확 데모는 "팔만" 또는 "주행만"이 아니라 둘을 번갈아 한다:
지점으로 가서 → 따고 → 다음 지점으로 가서 → 또 따고 → 바구니로 간다.
기존 arm_sequence는 프리셋만 나열할 수 있어서, 그 사이의 이동을 사람이 손으로
끼워 넣어야 했다.

**표기법** (공백이나 쉼표로 구분, 대소문자 무관):
    3        팔 프리셋 3 재생
    m2       지점 2로 이동 (m = move)
    w1.5     1.5초 대기 (팔이 흔들림을 멈추길 기다릴 때)
  예) "m2 1 2 3 m0 8 9"  =  2번 지점(종착) → 따기(1,2,3) → 0번 지점(바구니) → 놓기(8,9)

**설계 원칙 두 가지**

1. **비동기 실행 + 언제든 중단.** 시퀀스는 수십 초가 걸리는데 HTTP 요청을
   붙잡고 있으면 대시보드가 멈춘 것처럼 보이고 정지 버튼도 못 누른다.
   그래서 스레드에서 돌리고 진행 상황을 status()로 노출한다.
2. **지점 이동은 "끝날 때까지 기다린다".** LineDriver는 논블로킹이라 명령만
   던지면 즉시 돌아온다 — 그대로 다음 단계로 가면 주행 중에 팔이 움직인다.
   mode가 idle로 돌아올 때까지 기다리고, 실패(테이프 분실·타임아웃)면 **거기서
   시퀀스를 멈춘다.** 실패한 위치에서 팔을 뻗으면 엉뚱한 곳을 집는다.
"""

from __future__ import annotations

import json
import os
import re
import threading
import time

from ..config import (
    ARM_POSE_GAP_SEC,
    SEQUENCE_FILE,
    SEQUENCE_PRESETS,
    SEQUENCE_STEP_TIMEOUT_SEC,
)

_TOKEN = re.compile(r"^(m\d+|w\d+(?:\.\d+)?|\d+)$", re.I)


def parse(text: str) -> list[tuple[str, float]]:
    """"m0 1 2 w0.5" → [("station",0), ("preset",1), ("preset",2), ("wait",0.5)].

    알아볼 수 없는 토큰은 **조용히 무시하지 않고** 에러다 — 오타 하나로 엉뚱한
    동작이 나가는 것보다, 저장할 때 걸러지는 편이 안전하다.
    """
    steps: list[tuple[str, float]] = []
    for raw in re.split(r"[\s,]+", (text or "").strip()):
        if not raw:
            continue
        if not _TOKEN.match(raw):
            raise ValueError(
                f"알 수 없는 단계 '{raw}' — 숫자=팔 프리셋, m숫자=지점 이동, "
                "w초=대기 (예: m2 1 2 3 m0 8 9)")
        low = raw.lower()
        if low.startswith("m"):
            steps.append(("station", float(low[1:])))
        elif low.startswith("w"):
            steps.append(("wait", float(low[1:])))
        else:
            steps.append(("preset", float(low)))
    if not steps:
        raise ValueError("시퀀스가 비어 있습니다")
    return steps


def describe(steps: list[tuple[str, float]]) -> str:
    """사람이 읽는 한 줄 — 버튼 툴팁과 진행 표시에 쓴다."""
    out = []
    for kind, value in steps:
        if kind == "station":
            out.append(f"지점{int(value)}")
        elif kind == "wait":
            out.append(f"{value:g}초")
        else:
            out.append(f"프리셋{int(value)}")
    return " → ".join(out)


class SequenceRunner:
    """저장된 시퀀스 2개(이상)를 관리하고 백그라운드에서 실행한다."""

    def __init__(self, hardware: dict, path: str = SEQUENCE_FILE) -> None:
        # hardware는 가변 딕셔너리 — 재연결로 팔/라인 객체가 교체돼도 **지금**
        # 핸들을 꺼내 쓴다(옛 객체를 붙들면 재연결 후 죽은 팔에 명령한다).
        self._hw = hardware
        self._path = os.path.expanduser(path)
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._running: str | None = None      # 실행 중인 시퀀스 이름
        self._detail = "대기"
        self._step = 0
        self._total = 0
        self._seqs: dict[str, str] = dict(SEQUENCE_PRESETS)
        self._load()

    # ---------------- 저장/조회 ----------------

    def _load(self) -> None:
        try:
            with open(self._path, encoding="utf-8") as f:
                saved = json.load(f)
        except (OSError, ValueError):
            return
        for key, text in saved.items():
            if isinstance(text, str):
                self._seqs[str(key)] = text

    def _save(self) -> None:
        try:
            with open(self._path, "w", encoding="utf-8") as f:
                json.dump(self._seqs, f, ensure_ascii=False, indent=2)
        except OSError as exc:
            print(f"  [seq] 저장 실패: {exc}")

    def set_sequence(self, key: str, text: str) -> str:
        steps = parse(text)          # 저장 전에 검증 — 잘못된 건 아예 안 들어간다
        with self._lock:
            self._seqs[str(key)] = text.strip()
        self._save()
        return f"시퀀스 {key} 저장: {describe(steps)}"

    def status(self) -> dict:
        with self._lock:
            seqs = dict(self._seqs)
            running, detail, step, total = self._running, self._detail, self._step, self._total
        out = {}
        for key, text in seqs.items():
            try:
                out[key] = {"text": text, "desc": describe(parse(text)), "error": None}
            except ValueError as exc:
                out[key] = {"text": text, "desc": None, "error": str(exc)}
        return {"sequences": out, "running": running, "detail": detail,
                "step": step, "total": total}

    # ---------------- 실행 ----------------

    def start(self, key: str) -> str:
        with self._lock:
            text = self._seqs.get(str(key))
        if text is None:
            raise KeyError(f"시퀀스 {key}가 없습니다")
        return self._launch(str(key), text)

    def run_text(self, label: str, text: str) -> str:
        """저장된 시퀀스가 아닌 **즉석 대본**을 돌린다.

        음성 "2번 이동"이 쓴다 — 지점 이동 하나짜리 대본("m1")을 러너에 태우면
        도착까지 기다리기·정지 버튼·진행 표시를 전부 공짜로 얻고, 수확 시퀀스가
        도는 중에 음성으로 주행이 끼어드는 것도 같은 잠금으로 막힌다.
        """
        return self._launch(label, text)

    def _launch(self, label: str, text: str) -> str:
        # 검증이 먼저다 — 대본이 잘못됐으면 러너를 점유하지 않고 그대로 실패한다.
        steps = parse(text)
        with self._lock:
            # 점유 확인과 점유를 같은 잠금 안에서 한다. 예전엔 둘이 나뉘어 있어
            # 동시에 들어온 두 요청이 모두 통과할 수 있었다(음성+버튼이 겹치면 실제로 난다).
            if self._running:
                raise RuntimeError(f"이미 '{self._running}' 실행 중입니다 — 먼저 정지하세요")
            self._running, self._step, self._total = label, 0, len(steps)
            self._detail = "시작"
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(label, steps),
                                        daemon=True, name=f"seq-{label}")
        self._thread.start()
        return f"시퀀스 {label} 실행: {describe(steps)}"

    def stop(self, reason: str = "사용자 정지") -> str:
        """플래그만 세운다 — 진행 중인 한 단계는 끝나고 그 다음에서 멈춘다.

        팔 동작 중간에 끊으면 어정쩡한 자세로 남으므로 단계 경계에서 멈춘다.
        주행은 즉시 세워야 하니 라인 쪽에 취소를 따로 건다.
        """
        self._stop.set()
        line = self._hw.get("line")
        if line is not None:
            try:
                line.cancel(reason)
            except Exception:  # noqa: BLE001
                pass
        with self._lock:
            self._detail = reason
        return "시퀀스 정지 요청 — 진행 중인 단계가 끝나면 멈춥니다"

    def _run(self, key: str, steps: list[tuple[str, float]]) -> None:
        try:
            for index, (kind, value) in enumerate(steps, start=1):
                if self._stop.is_set():
                    self._finish("중단됨")
                    return
                with self._lock:
                    self._step = index
                    self._detail = f"{index}/{len(steps)} {describe([(kind, value)])}"
                if kind == "wait":
                    # 대기도 중단에 반응해야 한다 — 통짜 sleep이면 정지가 안 먹는다.
                    self._stop.wait(value)
                elif kind == "preset":
                    arm = self._hw.get("arm")
                    if arm is None:
                        raise RuntimeError("팔이 연결되지 않았습니다")
                    arm.play_preset(int(value))
                    # 자세와 자세 **사이**에만 쉰다 — 다음 단계가 또 자세일 때만.
                    # 마지막 뒤나 지점 이동 앞에 넣으면 정착이 아니라 그냥 지연이다.
                    if index < len(steps) and steps[index][0] == "preset":
                        self._stop.wait(ARM_POSE_GAP_SEC)   # 정지 버튼에 반응해야 한다
                else:
                    self._goto_station(int(value))
            self._finish("완료")
        except Exception as exc:  # noqa: BLE001 - 실패해도 러너 스레드는 조용히 끝난다
            print(f"  [seq] {key} 실패: {exc}")
            self._finish(f"실패: {exc}")

    def _goto_station(self, index: int) -> None:
        """지점 이동을 걸고 **끝날 때까지** 기다린다.

        LineDriver는 논블로킹이라 명령만 던지면 즉시 돌아온다 — 그대로 다음
        단계로 가면 주행 중에 팔이 움직인다. mode가 idle로 돌아오는 걸 완료로
        보고, 실패 사유가 붙어 있으면 시퀀스를 거기서 끊는다(엉뚱한 위치에서
        팔을 뻗지 않게).
        """
        line = self._hw.get("line")
        if line is None:
            raise RuntimeError("라인 주행이 비활성입니다 — 지점 이동을 쓸 수 없습니다")
        detail = line.goto_station(index)
        if "이미" in detail:            # 이동 없이 끝난 경우
            return
        deadline = time.monotonic() + SEQUENCE_STEP_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if self._stop.is_set():
                raise RuntimeError("중단됨")
            state = line.status()
            if state.get("mode") == "idle":
                last = str(state.get("detail") or "")
                # 도착이 아니라 안전정지·실패로 끝난 경우를 가려낸다.
                if any(bad in last for bad in ("정지", "실패", "초과", "취소")):
                    raise RuntimeError(f"지점 {index} 이동 실패 — {last}")
                return
            time.sleep(0.1)
        line.cancel("시퀀스 단계 시간 초과")
        raise RuntimeError(f"지점 {index} 이동이 {SEQUENCE_STEP_TIMEOUT_SEC:.0f}초를 넘겨 중단")

    def _finish(self, detail: str) -> None:
        with self._lock:
            self._running = None
            self._detail = detail
