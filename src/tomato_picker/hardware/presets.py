"""로봇팔 자세 프리셋 저장소 — 슬롯 0~19 + "높이 앵커" 보간.

두 가지 쓰임이 한 파일에 공존한다:

1. **시퀀스 슬롯 (0~19)** — 접근/집기/들기/놓기처럼 *순서대로 재생*하는 동작 단계.
   기존 ~/arm_presets.json의 1~4가 그대로 여기 들어온다([[tomato-pick-sequence]]).

2. **높이 앵커** — 그중 몇 개에 "상/중/하" 같은 라벨과 **기준 높이(y)** 를 달아두면,
   비전이 준 토마토 y좌표로 두 앵커 **사이 자세를 계산**해서 재생할 수 있다.
   ┌ 예: 슬롯4=상(y=250), 슬롯6=중(y=420), 슬롯8=하(y=600)
   └ 토마토 y=330 → 상:중 = 0.53:0.47 로 관절값을 선형 보간

   왜 앵커를 따로 두나 — 슬롯 "번호 순서"를 높이축으로 가정하면(4와 6 사이=5)
   시퀀스용 프리셋과 높이용 프리셋이 같은 번호공간을 쓰게 돼, '집기' 자세와
   '들기' 자세가 섞이는 사고가 난다. 높이축은 명시적으로 태그한 것만 참여시킨다.

보간이 물리적으로 말이 되는 이유: 앵커들은 같은 접근 방향에서 팔 높이만 다른
자세다. 그런 자세들 사이의 관절공간 선형보간은 실제로도 그 사이 높이를 가리킨다.
(임의의 두 자세를 섞으면 중간이 엉뚱한 곳을 향할 수 있으므로 앵커만 섞는다.)

파일 형식은 **기존 파일과 호환**된다. 옛 형식 {"1": {...}}을 그대로 읽고,
메타데이터는 숫자가 아닌 "__meta__" 키에 넣어 controller_drive.py처럼
`presets[str(n)]`만 보는 옛 코드가 계속 동작하게 했다.
"""

from __future__ import annotations

import json
import os
import tempfile
import threading

META_KEY = "__meta__"
# 슬롯 개수. 여기만 바꾸면 저장소·대시보드·게임패드가 다 따라온다 —
# 대시보드 격자는 auto-fill이고, 시퀀스 파서는 \d+라 두 자리도 그대로 먹는다.
# 10 → 20 (2026-08-17): 수확 대본 하나가 '접근·집기·들기·놓기'로 4~5칸을 먹는데
# 위/아래 두 벌에 바구니 동작까지 넣으면 10칸이 꽉 찼다. 남는 슬롯이 없으면
# 새 자세를 시험할 때 기존 걸 덮어써야 해서, 되돌리려면 다시 교시해야 했다.
SLOT_COUNT = 20
SLOT_IDS = [str(i) for i in range(SLOT_COUNT)]  # "0".."19"
DEFAULT_ANCHOR_LABELS = ["상", "중", "하"]


class PresetStore:
    """~/arm_presets.json을 읽고 쓰는 스레드 안전 저장소."""

    def __init__(self, path: str) -> None:
        self._path = os.path.expanduser(path)
        self._lock = threading.RLock()
        self._data: dict = {}
        self.reload()

    # --- 로드/세이브 ---

    def reload(self) -> None:
        with self._lock:
            try:
                with open(self._path, encoding="utf-8") as f:
                    raw = json.load(f)
            except (OSError, ValueError):
                raw = {}
            if not isinstance(raw, dict):
                raw = {}
            raw.setdefault(META_KEY, {})
            meta = raw[META_KEY]
            meta.setdefault("version", 2)
            meta.setdefault("names", {})    # {"1": "접근"}
            meta.setdefault("anchors", {})  # {"4": {"label": "상", "y": 250}}
            self._data = raw

    def _save(self) -> None:
        """원자적 저장 — 데모 중 전원이 나가도 프리셋 파일이 깨지지 않게."""
        directory = os.path.dirname(self._path) or "."
        os.makedirs(directory, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=directory, prefix=".presets-", suffix=".json")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(self._data, f, ensure_ascii=False, indent=2)
            os.replace(tmp, self._path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    # --- 슬롯 ---

    @property
    def path(self) -> str:
        return self._path

    def slot_ids(self) -> list[int]:
        """저장된 슬롯 번호(오름차순)."""
        with self._lock:
            return sorted(int(k) for k, v in self._data.items() if k.isdigit() and v)

    def get(self, slot: int) -> dict[str, float] | None:
        with self._lock:
            pose = self._data.get(str(slot))
            return dict(pose) if pose else None

    def save(self, slot: int, pose: dict[str, float], name: str | None = None) -> None:
        _check_slot(slot)
        with self._lock:
            self._data[str(slot)] = {k: float(v) for k, v in pose.items()}
            if name is not None:
                self._data[META_KEY]["names"][str(slot)] = name
            self._save()

    def delete(self, slot: int) -> None:
        with self._lock:
            self._data.pop(str(slot), None)
            self._data[META_KEY]["names"].pop(str(slot), None)
            self._data[META_KEY]["anchors"].pop(str(slot), None)
            self._save()

    def set_name(self, slot: int, name: str) -> None:
        _check_slot(slot)
        with self._lock:
            self._data[META_KEY]["names"][str(slot)] = name
            self._save()

    def name(self, slot: int) -> str:
        with self._lock:
            return self._data[META_KEY]["names"].get(str(slot), "")

    # --- 높이 앵커 ---

    def set_anchor(self, slot: int, label: str, y: float) -> None:
        """슬롯을 높이축에 올린다. y는 카메라 화면의 세로 픽셀(아래로 갈수록 큼)."""
        _check_slot(slot)
        with self._lock:
            if not self._data.get(str(slot)):
                raise KeyError(f"슬롯 {slot}이 비어 있어 앵커로 지정할 수 없습니다.")
            self._data[META_KEY]["anchors"][str(slot)] = {"label": label, "y": float(y)}
            self._save()

    def clear_anchor(self, slot: int) -> None:
        with self._lock:
            self._data[META_KEY]["anchors"].pop(str(slot), None)
            self._save()

    def anchors(self) -> list[dict]:
        """높이순(y 오름차순 = 위→아래) 앵커 목록. [{"slot":4,"label":"상","y":250}, ...]"""
        with self._lock:
            items = [
                {"slot": int(k), "label": v.get("label", ""), "y": float(v.get("y", 0))}
                for k, v in self._data[META_KEY]["anchors"].items()
                if k.isdigit() and self._data.get(k)
            ]
        return sorted(items, key=lambda a: a["y"])

    def blend(self, y: float) -> tuple[dict[str, float], str]:
        """높이 y에 해당하는 자세를 앵커 사이 선형보간으로 만든다.

        반환: (자세, 사람이 읽을 설명). 앵커가 하나뿐이면 그 자세를 그대로 쓰고,
        범위를 벗어나면 **바깥으로 외삽하지 않고** 끝 앵커로 고정한다
        (외삽은 팔이 사거리 밖으로 나가려 드는 위험한 동작이 되기 쉽다).
        """
        anchors = self.anchors()
        if not anchors:
            raise KeyError("높이 앵커가 하나도 없습니다 — 슬롯에 상/중/하를 지정하세요.")
        if len(anchors) == 1 or y <= anchors[0]["y"]:
            a = anchors[0]
            return self._pose_of(a), f"앵커 '{a['label']}'(슬롯 {a['slot']}) 그대로"
        if y >= anchors[-1]["y"]:
            a = anchors[-1]
            return self._pose_of(a), f"앵커 '{a['label']}'(슬롯 {a['slot']}) 그대로"

        lo, hi = anchors[0], anchors[-1]
        for first, second in zip(anchors, anchors[1:]):
            if first["y"] <= y <= second["y"]:
                lo, hi = first, second
                break
        span = hi["y"] - lo["y"]
        t = 0.0 if span == 0 else (y - lo["y"]) / span
        pose_lo, pose_hi = self._pose_of(lo), self._pose_of(hi)
        blended = {}
        for key in set(pose_lo) | set(pose_hi):
            if key in pose_lo and key in pose_hi:
                blended[key] = pose_lo[key] + (pose_hi[key] - pose_lo[key]) * t
            else:
                blended[key] = pose_lo.get(key, pose_hi.get(key, 0.0))
        desc = (f"'{lo['label']}'(슬롯 {lo['slot']}) : '{hi['label']}'(슬롯 {hi['slot']})"
                f" = {1 - t:.0%} : {t:.0%}")
        return blended, desc

    def _pose_of(self, anchor: dict) -> dict[str, float]:
        pose = self.get(anchor["slot"])
        if not pose:
            raise KeyError(f"앵커 슬롯 {anchor['slot']}이 비어 있습니다.")
        return pose

    # --- 웹 UI용 스냅샷 ---

    def snapshot(self) -> dict:
        with self._lock:
            names = dict(self._data[META_KEY]["names"])
            filled = {k for k in self._data if k.isdigit() and self._data[k]}
        anchor_by_slot = {str(a["slot"]): a for a in self.anchors()}
        return {
            "path": self._path,
            "slots": [
                {
                    "slot": int(sid),
                    "filled": sid in filled,
                    "name": names.get(sid, ""),
                    "anchor": anchor_by_slot.get(sid),
                }
                for sid in SLOT_IDS
            ],
            "anchors": self.anchors(),
        }


def _check_slot(slot: int) -> None:
    if str(slot) not in SLOT_IDS:
        raise ValueError(f"슬롯 번호는 0~{SLOT_COUNT - 1}만 됩니다 (받은 값: {slot})")
