"""비전 좌표를 **나무별·높이별**로 정리하고, 다음에 무엇을 딸지 정한다.

**왜 따로 있나.** 지금까지 "위 토마토 따줘"는 사람이 어느 나무 앞인지 알고
말해야 했다. 화면에는 이미 열매 좌표가 다 있는데, 그걸 "어느 나무의 몇 층"으로
읽어주는 층이 없었을 뿐이다. 여기가 그 층이다 — 좌표 목록을 받아 표로 만들고,
지금 위치에서 가장 가까운 것부터 딸 순서를 짠다.

**고정 숫자를 쓰지 않는다.** 처음엔 화면 x를 정해진 경계로 갈랐는데, 그러면
무대나 카메라가 조금만 움직여도 나무 배정이 통째로 어긋난다. 지금은 **보이는
것에서 배운다**:

· 나무 = 열매들의 x를 **간격으로 묶는다**(군집). 같은 나무의 열매는 x가 붙어
  있고 나무 사이는 훌쩍 벌어지므로, 절대 위치가 아니라 **상대 간격**으로 갈린다.
  무대가 통째로 옮겨가도 간격 구조는 그대로라 그대로 먹는다.
· 묶음 → 지점 배정 = 세 그루가 다 보이는 순간(모호하지 않은 순간)에 각 나무의
  x 중심을 **배워 둔다**. 한 그루에 열매가 없어 두 묶음만 보일 때는 배워 둔
  중심에 가장 가까운 것으로 붙인다 — 그래서 왼쪽 나무를 다 따버려도 남은 두
  그루가 지점0·1로 밀려 내려가지 않는다.
· 높이 = 같은 나무 안에서 **서로 비교한다**. 위/아래는 원래 상대적인 말이라
  절대 y 경계가 필요 없다. 한 개만 남았을 때만 배워 둔 그 나무의 경계를 쓴다.
· 순서 = **가까운 나무부터**. 지금 서 있는 지점의 열매를 먼저 다 따고, 그 다음
  가장 가까운 나무로 옮긴다. 주행이 가장 적다. 한 나무 안에서는 위 → 아래.

배운 값은 화면에 그대로 보여주고, 무대를 옮겼으면 [무대 다시 배우기]로 지운다.
아무것도 못 배운 상태에서는 config의 초기 추정값으로 버틴다(그 사실도 표시).

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
    HARVEST_LEARN_EMA,
    HARVEST_MERGE_PX,
    HARVEST_MIN_TREE_GAP_PX,
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


def _cluster_x(xs: list[float]) -> list[list[int]]:
    """x를 **간격**으로 묶는다 — 절대 위치를 쓰지 않는다.

    같은 나무의 열매는 x가 붙어 있고 나무 사이는 훌쩍 벌어진다. 그래서 가장 큰
    틈에 비례한 문턱으로 자르면 무대가 어디로 옮겨가도 같은 구조가 나온다.
    문턱에 하한(HARVEST_MIN_TREE_GAP_PX)을 두는 이유: 한 그루에만 열매가 있으면
    "가장 큰 틈"도 몇 px라, 비례만 쓰면 같은 나무를 둘로 쪼갠다.
    """
    order_ = sorted(range(len(xs)), key=lambda i: xs[i])
    if not order_:
        return []
    gaps = [xs[order_[i + 1]] - xs[order_[i]] for i in range(len(order_) - 1)]
    thr = max(HARVEST_MIN_TREE_GAP_PX, 0.4 * max(gaps, default=0.0))
    groups, cur = [], [order_[0]]
    for i, gap in enumerate(gaps):
        if gap > thr:
            groups.append(cur)
            cur = []
        cur.append(order_[i + 1])
    groups.append(cur)
    return groups


@dataclass
class SceneModel:
    """무대에 대해 **배운 것**. 고정 상수 대신 이걸 쓴다.

    tree_x    나무별 x 중심(왼→오른). 지점 인덱스와 같은 순서다.
    y_split   나무별 위/아래 경계. 그 나무에서 위·아래를 둘 다 본 적이 있어야 생긴다.
    """

    tree_x: list[float] | None = None
    y_split: list[float | None] | None = None
    learned: int = 0

    def note(self) -> str:
        if self.tree_x is None:
            return (f"무대 학습 전 — 초기 추정값 사용(경계 {HARVEST_TREE_X_BOUNDS}). "
                    "나무 세 그루가 한 화면에 다 보이면 그때 배웁니다")
        xs = ", ".join(f"{x:.0f}" for x in self.tree_x)
        ys = ", ".join("—" if y is None else f"{y:.0f}" for y in (self.y_split or []))
        return f"학습됨 {self.learned}회 · 나무 x=[{xs}] · 위아래 경계 y=[{ys}]"

    def reset(self) -> str:
        self.tree_x, self.y_split, self.learned = None, None, 0
        return "무대 학습을 지웠습니다 — 세 그루가 다 보이면 다시 배웁니다"

    # --- 배우기 ---

    def _blend(self, old, new):
        return new if old is None else old * (1 - HARVEST_LEARN_EMA) + new * HARVEST_LEARN_EMA

    def learn_trees(self, centers: list[float]) -> None:
        """세 그루가 다 보이는 **모호하지 않은** 순간에만 부른다.

        평소엔 천천히 따라간다(EMA) — 한 프레임의 좌표 흔들림에 배운 값이
        휘둘리면 안 되므로. 하지만 **무대를 통째로 옮긴 것은 흔들림이 아니다.**
        모두가 같은 방향으로 크게 밀렸으면 그대로 갈아끼운다 — 안 그러면 몇 번
        더 볼 때까지 배운 값이 뒤처진 채로 남아 애매한 화면에서 오배정이 난다.
        """
        if self.tree_x is None or len(self.tree_x) != len(centers):
            self.tree_x = list(centers)
            self.y_split = [None] * len(centers)
        else:
            shift = sum(abs(o - n) for o, n in zip(self.tree_x, centers)) / len(centers)
            spacing = min((abs(a - b) for a, b in zip(centers, centers[1:])), default=1e9)
            if shift > 0.5 * spacing:      # 무대가 옮겨졌다 — 천천히 갈 이유가 없다
                self.tree_x = list(centers)
            else:
                self.tree_x = [self._blend(o, n) for o, n in zip(self.tree_x, centers)]
        self.learned += 1

    def learn_split(self, tree: int, split: float) -> None:
        if self.y_split is None or tree >= len(self.y_split):
            return
        self.y_split[tree] = self._blend(self.y_split[tree], split)

    # --- 쓰기 ---

    def assign(self, centers: list[float]) -> list[int] | None:
        """묶음들의 x 중심 → 나무 번호. 못 정하면 None.

        ⚠ "각자 가장 가까운 나무"로 붙이면 안 된다. 무대가 옮겨가 배운 값이 조금
        뒤처진 상태에서는 **두 묶음이 같은 나무를 고르거나 순서가 뒤집힌다**
        (실측: 남은 두 그루가 둘 다 오른쪽 나무로 붙었다). 나무도 묶음도 왼→오른
        순서가 정해져 있으므로, **순서를 지키는 조합 중 오차 합이 가장 작은 것**을
        고른다(작은 DP). 그러면 배운 값이 통째로 밀려 있어도 상대 배치로 맞춘다.
        """
        if self.tree_x is None or not centers or len(centers) > len(self.tree_x):
            return None
        n, k = len(self.tree_x), len(centers)
        INF = float("inf")
        # best[j][i] = 묶음 j까지를, 나무 i를 마지막으로 써서 붙였을 때의 최소 오차합
        best = [[INF] * n for _ in range(k)]
        back = [[-1] * n for _ in range(k)]
        for i in range(n):
            best[0][i] = abs(self.tree_x[i] - centers[0])
        for j in range(1, k):
            for i in range(j, n):
                for prev in range(j - 1, i):
                    cand = best[j - 1][prev]
                    if cand + abs(self.tree_x[i] - centers[j]) < best[j][i]:
                        best[j][i] = cand + abs(self.tree_x[i] - centers[j])
                        back[j][i] = prev
        end = min(range(n), key=lambda i: best[k - 1][i])
        if best[k - 1][end] == INF:
            return None
        out = [end]
        for j in range(k - 1, 0, -1):
            end = back[j][end]
            out.append(end)
        return list(reversed(out))

    def split_of(self, tree: int) -> float:
        """그 나무의 위/아래 경계. 못 배웠으면 다른 나무들의 평균, 그것도 없으면 초기값."""
        known = [y for y in (self.y_split or []) if y is not None]
        if self.y_split and tree < len(self.y_split) and self.y_split[tree] is not None:
            return float(self.y_split[tree])
        if known:
            return sum(known) / len(known)
        return float(HARVEST_UPPER_MAX_Y)


def _fallback_tree(x: float) -> int:
    """아무것도 못 배웠을 때의 초기 추정 — 예전 고정 경계."""
    for i, bound in enumerate(HARVEST_TREE_X_BOUNDS):
        if x < bound:
            return i
    return len(HARVEST_TREE_X_BOUNDS)


def classify(positions, model: SceneModel | None = None) -> list[Fruit]:
    """[[x,y], …] → 나무·높이가 붙은 열매 목록(나무 순, 위 먼저).

    model을 주면 **배운 값**으로 붙이고, 모호하지 않은 화면이면 그 자리에서
    배운다. 안 주면 초기 추정값만 쓴다(순수 함수로 시험할 때).
    """
    pts: list[tuple[int, int]] = []
    for pos in positions or []:
        try:
            x, y = int(pos[0]), int(pos[1])
        except (TypeError, ValueError, IndexError):
            continue          # 이상한 항목 하나가 계획 전체를 죽이면 안 된다
        if not any(abs(px - x) < HARVEST_MERGE_PX and abs(py - y) < HARVEST_MERGE_PX
                   for px, py in pts):
            pts.append((x, y))   # 좌표 지터로 같은 열매가 둘로 세어지는 것 방지
    if not pts:
        return []

    n_trees = len(HARVEST_TREE_STATIONS)
    groups = _cluster_x([p[0] for p in pts])
    centers = [sum(pts[i][0] for i in g) / len(g) for g in groups]

    if model is not None and len(groups) == n_trees:
        # 세 그루가 다 보인다 = 모호하지 않다. 이때만 배운다.
        model.learn_trees(centers)
        trees = list(range(n_trees))
    elif model is not None and (got := model.assign(centers)) is not None:
        trees = got
    else:
        # 못 배웠고 묶음도 모자란다 — 초기 추정 경계로 버틴다(그 사실은 note()에).
        trees = [_fallback_tree(c) for c in centers]

    fruits: list[Fruit] = []
    for gi, g in enumerate(groups):
        tree = trees[gi]
        if not 0 <= tree < n_trees:
            continue          # 무대 밖 덩어리 — 셈에서 뺀다
        ys = sorted((pts[i][1], pts[i][0]) for i in g)
        if len(ys) >= 2:
            # 같은 나무 안에서는 **서로 비교**한다 — 절대 경계가 필요 없다.
            # 가장 큰 y 틈에서 자른다(보통 2개라 그냥 위/아래).
            gaps = [(ys[k + 1][0] - ys[k][0], k) for k in range(len(ys) - 1)]
            _, cut = max(gaps)
            split = (ys[cut][0] + ys[cut + 1][0]) / 2
            if model is not None:
                model.learn_split(tree, split)
            for k, (y, x) in enumerate(ys):
                fruits.append(Fruit(x, y, tree, "upper" if k <= cut else "lower"))
        else:
            y, x = ys[0]
            split = model.split_of(tree) if model is not None else float(HARVEST_UPPER_MAX_Y)
            fruits.append(Fruit(x, y, tree, "upper" if y < split else "lower"))
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
        # 무대 모형은 러너가 들고 있다 — 관측할 때마다 조금씩 배운다.
        self._scene = SceneModel()
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
        fruits = classify(self._positions(), self._scene)
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
            "scene": self._scene.note(),
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

    def relearn(self) -> str:
        """무대를 옮겼을 때 — 배운 걸 지우고 다음 화면부터 다시 배운다."""
        with self._lock:
            return self._scene.reset()

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
                queue = order(classify(self._positions(), self._scene), self._station())
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
            n = len(classify(self._positions(), self._scene))
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
    scene = SceneModel()
    got = classify(pos, scene)
    print(f"  무대: {scene.note()}")
    for row in table(got):
        print(f"  {row['name']:<16} 위={row['upper']}  아래={row['lower']}"
              f"  → 지점{row['station']}")
    for at in (None, 0, 2):
        seq = order(got, at)
        print(f"\n지금 지점={at} 일 때 순서:")
        for i, f in enumerate(seq, 1):
            print(f"  {i}. {f.label} (지점{f.station})")
        print(f"  다음 동작: {describe_next(seq[0] if seq else None, at)}")
