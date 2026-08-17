"""인식된 텍스트 → 인텐트. 발음을 접어 비교하므로 오인식에 강하다.

**낱말 하나가 아니라 슬롯을 읽는다.** 예전에는 "명령 문장 전체"를 키워드로
나열했다("토마토 따줘", "토마토따줘", …). 그러면 조합이 늘어날 때마다 목록이
제곱으로 커지고("위 토마토 따줘"/"토마토 위에 따줘"/…), 안 적어 둔 어순은
그냥 무시된다. 여기서는 문장에서 **부품**을 따로 찾아 조립한다:

    "어 그럼 아래 토마토 좀 따줘"  →  [수확 어간]+[높이=아래]  →  아래 수확
    "2번으로 이동해"               →  [번호=2]+[이동]          →  지점1로 이동

부품 찾기는 korean.py의 근사 매칭이라 자음 세기·받침·띄어쓰기가 틀려도 걸린다.

**순서가 곧 우선순위다.** 지점 이동을 먼저 본다 — 지점 이름이 "토마토1"이라
"토마토 1번으로 이동"이 수확으로 새는 걸 막아야 하기 때문이다.

**안전 쪽으로 기운 설계.** 번호는 이동 낱말과 **같이** 나와야 인정한다.
"이번"은 일상 발화에서 흔한 말이고("이번에는…"), 이 명령은 실제로 바퀴를
굴린다. 반대로 높이는 못 알아들어도 기본값(위)으로 진행한다 — 팔만 움직이는
동작이라 위험이 낮고, 데모 중 침묵하는 것보다 낫다.

디버깅: `python -m tomato_picker.voice.intents "아래 토마토 따줘"`
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ..config import (
    VOICE_BASKET_STATION,
    VOICE_PICK_DEFAULT_HEIGHT,
    VOICE_SPOKEN_TO_STATION,
)
from . import korean, words


@dataclass(frozen=True)
class Intent:
    """무엇을 할지(name)와 그 인자(slots), 그리고 로그에 쓸 한 줄(label)."""

    name: str
    label: str
    slots: dict = field(default_factory=dict)


def _height(text: str) -> tuple[str, bool]:
    """(높이, 확실한가). 위/아래 양쪽 점수를 재고 **더 가까운 쪽**을 고른다.

    한쪽만 보고 "걸렸다"로 끝내면 안 된다 — "일층"과 "이층"은 발음을 접으면
    편집거리 1이라 서로의 후보에도 걸린다. 둘 다 재서 비교해야 가려진다.
    동점이거나 아무것도 안 걸리면 확실하지 않다고 알린다(부르는 쪽이
    기본값으로 처리하고 로그에 남긴다).
    """
    scored = {}
    for height in ("upper", "lower"):
        hit = korean.best(text, words.STORE.get(f"height_{height}"))
        if hit is not None:
            scored[height] = hit[1]
    if not scored:
        return VOICE_PICK_DEFAULT_HEIGHT, False
    best_score = min(scored.values())
    winners = [h for h, s in scored.items() if s == best_score]
    if len(winners) != 1:
        return VOICE_PICK_DEFAULT_HEIGHT, False
    return winners[0], True


def _spoken_number(text: str) -> tuple[int | None, bool]:
    """(말한 화분 번호, 없어진 번호였나). 가장 가까운 하나만 — 동점이면 안 고른다.

    "일번"과 "이번"도 편집거리 1이라 서로 걸린다. 애매하면 엉뚱한 지점으로
    가는 것보다 아무 데도 안 가는 게 낫다.
    """
    scored = {}
    for number in VOICE_SPOKEN_TO_STATION:
        hit = korean.best(text, words.STORE.get(f"station_{number}"))
        if hit is not None:
            scored[number] = hit[1]
    if not scored:
        return None, False
    best_score = min(scored.values())
    # ⚠ 코스에서 뺀 번호를 **경쟁자로 세운다.** 그냥 목록에서 지우기만 하면
    #   "삼번"이 "이번"에 편집거리 1로 걸려 지점1로 간다(2026-08-17 실측:
    #   "3번 이동" → station 1). 습관대로 옛 번호를 말했는데 로봇이 조용히 다른
    #   지점으로 가는 게 가장 나쁘다. 은퇴 번호가 더 가깝거나 비기면 안 고른다.
    retired = korean.best(text, words.STORE.get("station_retired"))
    if retired is not None and retired[1] <= best_score:
        return None, True
    winners = [n for n, s in scored.items() if s == best_score]
    return (winners[0] if len(winners) == 1 else None), False


def match_intent(text: str) -> Intent | None:
    """들린 문장에서 인텐트 하나를 뽑는다. 못 찾으면 None."""
    moving = korean.contains(text, words.STORE.get("move"))

    # 1) 지점 이동 — 수확보다 먼저 본다("토마토 1번으로 이동"이 수확으로 새지 않게).
    #    바구니는 그 자체로 특이한 낱말이라 이동 낱말 없이도 인정한다.
    if korean.contains(text, words.STORE.get("basket")):
        return Intent("station_move", "바구니로 이동",
                      {"station": VOICE_BASKET_STATION, "spoken": "바구니"})
    if moving:
        number, retired = _spoken_number(text)
        if number is not None:
            station = VOICE_SPOKEN_TO_STATION[number]
            return Intent("station_move", f"{number}번(토마토{number}) 지점으로 이동",
                          {"station": station, "spoken": str(number)})
        if retired:
            # ⚠ 없어진 번호를 말했다 — **여기서 끝낸다.** 아래로 흘려보내면 엉뚱한
            #   명령이 된다(2026-08-17 실측: "3번 지점으로 이동해"가 번호를 못 찾고
            #   흘러내려 **drive_forward**(전진)로 걸렸다). 어디로 갈지 모르면 아무
            #   데도 안 가는 게 이 로봇의 원칙이다.
            #   ⚠ 여기서 무조건 return None 하면 안 된다 — "토마토 따줘"·"앞으로 가"도
            #   'move' 낱말에 걸려 moving=True가 되므로 수확·전진이 통째로 죽는다.
            return None

    # 2) 수확. "모두"가 같이 들리면 **보이는 걸 전부** — 개별 수확보다 먼저 본다
    #    (안 그러면 "모두 토마토 따줘"가 그냥 한 개 따기로 새어 나간다).
    if korean.contains(text, words.STORE.get("pick")):
        # ⚠ 여기만 **정확히 일치**를 요구한다(거리 0). 다른 슬롯처럼 오인식을
        #   허용했더니 "상단 토마토 따"·"도마뚱"이 전체 수확으로 걸렸다 — 1글자
        #   차이였다. 잘못 걸리면 로봇이 코스를 길게 자율 주행한다. 애매하면
        #   작은 동작(한 개 따기)으로 떨어지는 게 안전하다.
        hit = korean.best(text, words.STORE.get("all"))
        if hit is not None and hit[1] == 0:
            return Intent("harvest_all", "보이는 토마토 모두 따기")
        height, sure = _height(text)
        korean_name = "위(2층)" if height == "upper" else "아래(1층)"
        label = f"{korean_name} 토마토 따기" + ("" if sure else " — 높이 미지정, 기본값")
        return Intent("tomato_pick", label, {"height": height, "explicit": sure})

    # 3) 슬롯 없는 단순 인텐트.
    for name in ("arm_move", "drive_forward"):
        if korean.contains(text, words.STORE.get(name)):
            return Intent(name, name)
    return None


if __name__ == "__main__":  # 현장 디버깅: 이 문장이 무슨 명령으로 읽히나
    import sys

    for arg in sys.argv[1:]:
        got = match_intent(arg)
        print(f"{arg!r:>32} → {got.label if got else '(매칭 없음)'}"
              + (f"  {got.slots}" if got and got.slots else ""))
