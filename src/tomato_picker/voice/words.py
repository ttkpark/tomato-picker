"""음성 명령 낱말 사전 — 코드 기본값 위에 **현장에서 고친 값**을 얹는다.

축·부호(LINE_TUNING_FILE)·모터 튜닝(BASE_TUNING_FILE)·시퀀스(SEQUENCE_FILE)와
같은 원칙이다: 코드의 값은 출고 기본값이고, 대시보드에서 고른 값이 이긴다.
음성은 특히 그렇다 — 마이크·사람·주변 소음이 바뀌면 잘 걸리는 낱말도 바뀌는데,
그때마다 ssh로 config.py를 고치고 서비스를 재시작할 수는 없다.

**낱말을 많이 적을 필요는 없다.** 매칭은 korean.py가 발음을 접어서 하므로
"토마토" 하나면 "도마도·또마또·도마토"가 다 걸린다. 여기에 추가할 것은
접어도 안 같아지는 것들뿐이다(로그에 실제로 찍힌 오인식을 그대로 넣으면 된다).

**빈 칸도 허용한다** — 그 명령을 아예 끄고 싶을 때가 있다(예: 데모 중
"앞으로 가"가 잡음에 자꾸 걸리면 잠깐 비워 둔다). 대신 저장할 때 그렇게
알려준다. 되돌리기는 [기본값] 버튼.
"""

from __future__ import annotations

import json
import os
import threading

from ..config import (
    VOICE_BASKET_WORDS,
    VOICE_HEIGHT_WORDS,
    VOICE_INTENTS,
    VOICE_MOVE_WORDS,
    VOICE_PICK_WORDS,
    VOICE_STATION_WORDS,
    VOICE_WORDS_FILE,
)
from . import korean

# (키, 화면 라벨, 설명). 화면에 이 순서대로 나온다.
CATALOG: list[tuple[str, str, str]] = [
    ("pick", "수확",
     "이 말이 들리면 토마토를 딴다. 높이(위/아래)는 아래 두 칸에서 따로 읽는다"),
    ("height_upper", "위(2층)", "이 말이 같이 들리면 2층 프리셋으로 딴다"),
    ("height_lower", "아래(1층)", "이 말이 같이 들리면 1층 프리셋으로 딴다"),
    ("station_1", "1번 지점", "‘이동’ 낱말과 <b>함께</b> 들려야 인정한다"),
    ("station_2", "2번 지점", "‘이동’ 낱말과 <b>함께</b> 들려야 인정한다"),
    ("station_3", "3번 지점", "‘이동’ 낱말과 <b>함께</b> 들려야 인정한다"),
    ("basket", "바구니", "낱말이 특이해서 ‘이동’ 없이 단독으로도 인정한다"),
    ("move", "이동",
     "번호와 짝이 되는 말. ‘이번에는…’ 같은 일상 발화가 명령이 되지 않게 하는 안전장치라 "
     "너무 짧고 흔한 말은 넣지 말 것"),
    ("arm_move", "팔 움직여", "팔 데모 동작 재생"),
    ("drive_forward", "앞으로 가", "정해진 시간만큼 전진"),
]

_LABELS = {key: label for key, label, _hint in CATALOG}


def _defaults() -> dict[str, list[str]]:
    return {
        "pick": list(VOICE_PICK_WORDS),
        "height_upper": list(VOICE_HEIGHT_WORDS["upper"]),
        "height_lower": list(VOICE_HEIGHT_WORDS["lower"]),
        "station_1": list(VOICE_STATION_WORDS[1]),
        "station_2": list(VOICE_STATION_WORDS[2]),
        "station_3": list(VOICE_STATION_WORDS[3]),
        "basket": list(VOICE_BASKET_WORDS),
        "move": list(VOICE_MOVE_WORDS),
        "arm_move": list(VOICE_INTENTS["arm_move"]),
        "drive_forward": list(VOICE_INTENTS["drive_forward"]),
    }


def _parse(text: str) -> list[str]:
    """쉼표로 나눈 낱말 목록. 빈 칸·중복은 버리고 순서는 유지한다."""
    out: list[str] = []
    for raw in (text or "").replace("\n", ",").split(","):
        word = raw.strip()
        if word and word not in out:
            out.append(word)
    return out


class WordStore:
    """낱말 사전 하나. 인텐트 매칭이 매번 여기서 읽으므로 저장은 즉시 반영된다."""

    def __init__(self, path: str | None = VOICE_WORDS_FILE) -> None:
        # path=None이면 파일을 읽지도 쓰지도 않는다 — 회귀 검사(tools/check_intents.py)가
        # **출고 기본값**을 시험할 때 쓴다. 현장에서 고친 사전이 검사 결과를 바꾸면
        # 검사가 무엇을 보장하는지 알 수 없어진다.
        self._path = os.path.expanduser(path) if path else None
        self._lock = threading.Lock()
        self._words = _defaults()
        self._load()

    def _load(self) -> None:
        if self._path is None:
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                saved = json.load(f)
        except (OSError, ValueError):
            return
        for key, value in saved.items():
            # 모르는 키는 무시한다 — 옛 파일이 남아 있어도 서비스가 안 죽는다.
            if key in self._words and isinstance(value, list):
                self._words[key] = [str(w) for w in value if str(w).strip()]

    def _save(self) -> None:
        if self._path is None:
            return
        try:
            with open(self._path, "w", encoding="utf-8") as f:
                json.dump(self._words, f, ensure_ascii=False, indent=2)
        except OSError as exc:
            print(f"  [words] 저장 실패: {exc}")

    def get(self, key: str) -> list[str]:
        with self._lock:
            return list(self._words.get(key, ()))

    def set(self, key: str, text: str) -> str:
        if key not in _LABELS:
            raise KeyError(f"모르는 명령어 칸: {key}")
        parsed = _parse(text)
        with self._lock:
            self._words[key] = parsed
            self._save()
        label = _LABELS[key]
        if not parsed:
            return f"'{label}' 비움 — 이 명령은 이제 인식되지 않습니다"
        # 발음으로 접었을 때 아무것도 안 남는 낱말은 영원히 안 걸린다(영문·기호만 등).
        dead = [w for w in parsed if not korean.phonemes(w)]
        warn = f"  ⚠ 한글이 없어 매칭 불가: {', '.join(dead)}" if dead else ""
        return f"'{label}' 저장: {', '.join(parsed)}{warn}"

    def reset(self, key: str | None = None) -> str:
        base = _defaults()
        with self._lock:
            if key is None:
                self._words = base
                self._save()
                return "음성 명령어를 모두 기본값으로 되돌렸습니다"
            if key not in base:
                raise KeyError(f"모르는 명령어 칸: {key}")
            self._words[key] = base[key]
            self._save()
            return f"'{_LABELS[key]}' 기본값 복원: {', '.join(base[key])}"

    def catalog(self) -> list[dict]:
        """설정 화면이 그리는 목록. 기본값과 같은지도 알려준다(고친 칸 표시용)."""
        base = _defaults()
        with self._lock:
            words = {k: list(v) for k, v in self._words.items()}
        return [
            {"key": key, "label": label, "hint": hint,
             "text": ", ".join(words.get(key, ())),
             "changed": words.get(key) != base.get(key)}
            for key, label, hint in CATALOG
        ]


# 프로세스 하나에 사전 하나. intents.py가 매 인식마다 읽는다.
STORE = WordStore()
