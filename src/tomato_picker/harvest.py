"""비전 좌표를 **나무별·높이별**로 정리하고, 다음에 무엇을 딸지 정한다.

**왜 따로 있나.** 지금까지 "위 토마토 따줘"는 사람이 어느 나무 앞인지 알고
말해야 했다. 화면에는 이미 열매 좌표가 다 있는데, 그걸 "어느 나무의 몇 층"으로
읽어주는 층이 없었을 뿐이다. 여기가 그 층이다 — 좌표 목록을 받아 표로 만들고,
지금 위치에서 가장 가까운 것부터 딸 순서를 짠다.

**판단 규칙**

· 나무 = 화면 x 경계로 가른다. 무대 카메라가 바닥에 고정이라 x가 곧 나무다.
  (검출된 것을 x로 정렬해 순서대로 배정하면, 한 그루가 안 잡힌 순간 나머지가
   통째로 밀려 엉뚱한 나무로 간다. 경계는 고정이라 그런 일이 없다.)
· 높이 = y 하나로 2층/1층. 위가 화면에서 더 작은 y다.
· 순서 = **가까운 나무부터**. 지금 서 있는 지점의 열매를 먼저 다 따고, 그 다음
  가장 가까운 나무로 옮긴다. 주행이 가장 적다. 한 나무 안에서는 위 → 아래.

**여기서 하드웨어를 만지지 않는다.** 순수하게 좌표 → 계획이라 하드웨어 없이
그대로 시험할 수 있다. 실행은 HarvestRunner가 시퀀스 러너에 태워서 한다.

디버깅: `python -m tomato_picker.harvest 247,347 718,324 248,234 ...`
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from .config import (
    HARVEST_ARM_SEC,
    HARVEST_MERGE_PX,
    HARVEST_REARM_EMPTY_SEC,
    HARVEST_TREE_NAMES,
    HARVEST_TREE_STATIONS,
    HARVEST_TREE_X_BOUNDS,
    HARVEST_UPPER_MAX_Y,
    LINE_STATION_LABELS,
    VOICE_PICK_SEQUENCE_KEYS,
)


@dataclass(frozen=True)
class Fruit:
    x: int
    y: int
    tree: int          # 0=왼쪽 … 나무 번호
    height: str        # "upper" | "lower"

    @property
    def station(self) -> int:
        return HARVEST_TREE_STATIONS[self.tree]

    @property
    def label(self) -> str:
        floor = "2층(위)" if self.height == "upper" else "1층(아래)"
        return f"{HARVEST_TREE_NAMES[self.tree]} {floor}"


def _tree_of(x: float) -> int:
    for i, bound in enumerate(HARVEST_TREE_X_BOUNDS):
        if x < bound:
            return i
    return len(HARVEST_TREE_X_BOUNDS)


def classify(positions) -> list[Fruit]:
    """[[x,y], …] → 나무·높이가 붙은 열매 목록(나무 순, 위 먼저).

    좌표가 프레임마다 몇 px씩 흔들려 같은 열매가 둘로 세어지는 일이 있어,
    같은 칸(나무×높이) 안에서 아주 가까운 것은 하나로 합친다.
    """
    fruits: list[Fruit] = []
    for pos in positions or []:
        try:
            x, y = int(pos[0]), int(pos[1])
        except (TypeError, ValueError, IndexError):
            continue          # 이상한 항목 하나가 계획 전체를 죽이면 안 된다
        tree = _tree_of(x)
        if tree >= len(HARVEST_TREE_STATIONS):
            continue          # 무대 밖(코스에 없는 나무) — 셈에서 뺀다
        height = "upper" if y < HARVEST_UPPER_MAX_Y else "lower"
        if any(f.tree == tree and f.height == height
               and abs(f.x - x) < HARVEST_MERGE_PX and abs(f.y - y) < HARVEST_MERGE_PX
               for f in fruits):
            continue
        fruits.append(Fruit(x, y, tree, height))
    # 나무 왼→오른, 같은 나무 안에서는 위 먼저.
    return sorted(fruits, key=lambda f: (f.tree, 0 if f.height == "upper" else 1))


def order(fruits: list[Fruit], at: int | None) -> list[Fruit]:
    """딸 순서 — **가까운 나무부터**. at은 지금 서 있는 지점(모르면 None).

    지금 지점의 나무를 0순위로 두고, 나머지는 거리순이다. 지점을 모르면
    왼쪽부터(=지점 순서) — 추측해서 엉뚱하게 도는 것보다 낫다.
    """
    if at is None:
        return list(fruits)
    return sorted(fruits, key=lambda f: (abs(f.station - at), f.station,
                                         0 if f.height == "upper" else 1))


def table(fruits: list[Fruit]) -> list[dict]:
    """대시보드가 그리는 나무별 표(빈 나무도 한 줄씩 — '없음'이 곧 정보다)."""
    out = []
    for tree, name in enumerate(HARVEST_TREE_NAMES):
        got = {f.height: f for f in fruits if f.tree == tree}
        out.append({
            "tree": tree, "name": name,
            "station": HARVEST_TREE_STATIONS[tree],
            "station_label": LINE_STATION_LABELS[HARVEST_TREE_STATIONS[tree]],
            "upper": ([got["upper"].x, got["upper"].y] if "upper" in got else None),
            "lower": ([got["lower"].x, got["lower"].y] if "lower" in got else None),
        })
    return out


def describe_next(fruit: Fruit | None, at: int | None) -> str:
    """다음 동작 한 줄 — 화면에 그대로 띄운다."""
    if fruit is None:
        return "딸 토마토가 없습니다"
    move = ("" if at == fruit.station
            else f"{LINE_STATION_LABELS[fruit.station]}(으)로 이동 → ")
    return (f"{move}{fruit.label} 따기 → 바구니에 놓기"
            f"   (화면 {fruit.x},{fruit.y})")


@dataclass
class _AutoState:
    on: bool = False
    seen_since: float = 0.0      # 토마토가 연속으로 보이기 시작한 시각
    empty_since: float = 0.0     # 비어 있기 시작한 시각
    armed: bool = True           # 다음 등장에 반응할 준비가 됐나(상승엣지)
    note: str = "꺼짐"


class HarvestRunner:
    """계획을 세우고, 시퀀스 러너에 태워 실행한다.

    한 열매를 따는 것 = "지점으로 이동 → 따기 프리셋 → 바구니로 → 놓기"인데,
    그 대본이 이미 SEQUENCE_PRESETS의 '위'/'아래'에 있다(바구니 이동·놓기 포함).
    그래서 여기서는 **어느 지점에서 어느 대본을 돌릴지**만 정하면 된다 — 주행
    대기·정지 버튼·진행 표시가 전부 러너 것이라 다시 만들지 않는다.
    """

    def __init__(self, hardware: dict, vision, log_hub=None) -> None:
        self._hw = hardware
        self._vision = vision
        self._log = log_hub
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._running = False
        self._detail = "대기"
        self._done = 0
        self._auto = _AutoState()
        threading.Thread(target=self._auto_loop, daemon=True, name="harvest-auto").start()

    # ---------------- 관찰 ----------------

    def _positions(self) -> list:
        if self._vision is None or not hasattr(self._vision, "latest_status"):
            return []
        try:
            return self._vision.latest_status().get("positions") or []
        except Exception:  # noqa: BLE001 - 비전이 죽어도 계획은 "없음"으로 끝난다
            return []

    def _station(self) -> int | None:
        line = self._hw.get("line")
        if line is None:
            return None
        try:
            return line.status().get("station")
        except Exception:  # noqa: BLE001
            return None

    def plan(self) -> dict:
        """지금 보이는 것으로 세운 계획. 대시보드가 1초마다 이걸 그린다."""
        fruits = classify(self._positions())
        at = self._station()
        queue = order(fruits, at)
        with self._lock:
            running, detail, done = self._running, self._detail, self._done
            auto_on, auto_note = self._auto.on, self._auto.note
        return {
            "count": len(fruits),
            "at": at,
            "at_label": (LINE_STATION_LABELS[at] if at is not None else None),
            "trees": table(fruits),
            "order": [{"label": f.label, "station": f.station,
                       "height": f.height, "x": f.x, "y": f.y} for f in queue],
            "next": describe_next(queue[0] if queue else None, at),
            "running": running, "detail": detail, "picked": done,
            "auto": auto_on, "auto_note": auto_note,
        }

    # ---------------- 실행 ----------------

    def _say(self, text: str) -> None:
        with self._lock:
            self._detail = text
        print(f"  [harvest] {text}")
        if self._log is not None:
            try:
                self._log.publish({"ts": time.strftime("%H:%M:%S"),
                                   "kind": "status", "text": f"[수확] {text}"})
            except Exception:  # noqa: BLE001
                pass

    def start(self, why: str = "모두 따기") -> str:
        with self._lock:
            if self._running:
                raise RuntimeError("이미 수확 중입니다 — 먼저 정지하세요")
            self._running, self._done = True, 0
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(why,), daemon=True,
                                        name="harvest")
        self._thread.start()
        return f"{why} 시작 — 보이는 토마토를 가까운 나무부터 딴다"

    def stop(self, reason: str = "사용자 정지") -> str:
        self._stop.set()
        seq = self._hw.get("seq")
        if seq is not None:
            try:
                seq.stop(reason)
            except Exception:  # noqa: BLE001
                pass
        return "수확 정지 요청 — 진행 중인 동작이 끝나면 멈춥니다"

    def set_auto(self, on: bool) -> str:
        with self._lock:
            self._auto.on = bool(on)
            self._auto.armed = True
            self._auto.seen_since = self._auto.empty_since = 0.0
            self._auto.note = ("대기 중 — 토마토가 보이면 시작합니다" if on else "꺼짐")
        return self._auto.note

    def _wait_sequence(self) -> bool:
        """시퀀스가 끝날 때까지 기다린다. 정상 종료면 True."""
        seq = self._hw.get("seq")
        if seq is None:
            return False
        while not self._stop.is_set():
            st = seq.status()
            if not st.get("running"):
                detail = str(st.get("detail") or "")
                return not any(bad in detail for bad in ("실패", "초과", "취소", "중단"))
            time.sleep(0.2)
        return False

    def _run(self, why: str) -> None:
        try:
            seq = self._hw.get("seq")
            if seq is None:
                raise RuntimeError("시퀀스 러너가 없습니다")
            while not self._stop.is_set():
                # 매번 **다시 본다** — 하나 따고 나면 화면이 바뀐다. 처음 목록을
                # 붙들고 돌면 이미 딴 것을 또 따러 간다.
                queue = order(classify(self._positions()), self._station())
                if not queue:
                    self._say(f"{why} 완료 — {self._done}개 땄습니다")
                    return
                fruit = queue[0]
                key = VOICE_PICK_SEQUENCE_KEYS[fruit.height]
                if self._station() != fruit.station:
                    self._say(f"{fruit.label} — {LINE_STATION_LABELS[fruit.station]}(으)로 이동")
                    seq.run_text(f"{fruit.label} 이동", f"m{fruit.station}")
                    if not self._wait_sequence():
                        self._say("이동이 끝나지 않아 중단합니다")
                        return
                    if self._stop.is_set():
                        break
                self._say(f"{fruit.label} 따기 → 바구니")
                seq.start(key)
                if not self._wait_sequence():
                    self._say("따기가 끝나지 않아 중단합니다")
                    return
                with self._lock:
                    self._done += 1
                # 딴 직후엔 비전이 아직 옛 좌표를 들고 있을 수 있다(TV_HOLD_SEC).
                # 잠깐 쉬어 화면이 따라잡게 한 뒤 다시 본다.
                self._stop.wait(1.5)
            self._say(f"중단됨 — {self._done}개까지 땄습니다")
        except Exception as exc:  # noqa: BLE001 - 실패해도 스레드는 조용히 끝난다
            self._say(f"실패: {exc}")
        finally:
            with self._lock:
                self._running = False

    def _badge(self) -> str:
        """대시보드 한 줄 — 지금 몇 개 보이고 다음에 뭘 할 건지."""
        with self._lock:
            running, detail, auto = self._running, self._detail, self._auto.on
        head = "🤖 자동" if auto else "🖐 수동"
        if running:
            return f"{head} · 수확 중 — {detail}"
        plan = self.plan()
        return f"{head} · 토마토 {plan['count']}개 · 다음: {plan['next']}"

    # ---------------- 자동 모드 ----------------

    def _auto_loop(self) -> None:
        """평소엔 대기, 토마토가 **연속으로** 보이면 수확을 건다.

        상승엣지로 만든다: 한 번 수확한 뒤에는 화면이 비워질 때까지 다시 걸지
        않는다. 못 딴 게 남아 있을 때 같은 동작을 무한 반복하지 않기 위해서다 —
        남았으면 로그로 알리고 사람이 판단한다.
        """
        last_badge = None
        while True:
            time.sleep(0.5)
            # 자동 모드가 꺼져 있어도 **계획은 늘 보여준다** — "지금 뭘 딸 차례인지"가
            # 곧 이 기능의 값이다. latest_only라 로그를 도배하지 않고 배지만 바뀐다.
            badge = self._badge()
            if badge != last_badge and self._log is not None:
                last_badge = badge
                try:
                    self._log.publish({"ts": time.strftime("%H:%M:%S"), "kind": "harvest",
                                       "text": badge}, latest_only=True)
                except Exception:  # noqa: BLE001
                    pass
            with self._lock:
                on, running = self._auto.on, self._running
            if not on or running:
                continue
            now = time.monotonic()
            n = len(classify(self._positions()))
            with self._lock:
                st = self._auto
                if n:
                    st.empty_since = 0.0
                    st.seen_since = st.seen_since or now
                    waited = now - st.seen_since
                    if not st.armed:
                        st.note = f"토마토 {n}개가 남아 있습니다 — 치우면 다시 대기합니다"
                        continue
                    st.note = f"토마토 {n}개 감지 — {max(0.0, HARVEST_ARM_SEC - waited):.1f}초 뒤 시작"
                    if waited < HARVEST_ARM_SEC:
                        continue
                    st.armed = False
                    st.seen_since = 0.0
                else:
                    st.seen_since = 0.0
                    st.empty_since = st.empty_since or now
                    if not st.armed and now - st.empty_since >= HARVEST_REARM_EMPTY_SEC:
                        st.armed = True
                    st.note = ("대기 중 — 토마토가 보이면 시작합니다" if st.armed
                               else "치우는 중… 비워지면 다시 대기합니다")
                    continue
            try:
                self.start("자동 수확")
            except RuntimeError:
                pass          # 그 사이 다른 경로로 시작됐다 — 그대로 둔다


if __name__ == "__main__":   # 디버깅: 좌표를 주면 표와 계획을 찍는다
    import sys

    pos = [[int(v) for v in a.split(",")] for a in sys.argv[1:]] or [
        [247, 347], [718, 324], [248, 234], [461, 224], [729, 217], [473, 344]]
    got = classify(pos)
    for row in table(got):
        print(f"  {row['name']:<16} 위={row['upper']}  아래={row['lower']}"
              f"  → 지점{row['station']}")
    for at in (None, 0, 2):
        seq = order(got, at)
        print(f"\n지금 지점={at} 일 때 순서:")
        for i, f in enumerate(seq, 1):
            print(f"  {i}. {f.label} (지점{f.station})")
        print(f"  다음 동작: {describe_next(seq[0] if seq else None, at)}")
