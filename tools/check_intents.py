"""음성 인텐트 매칭 회귀 검사 — 하드웨어·마이크 없이 표만 돌린다.

**왜 표로 묶어 두는가.** 매칭을 느슨하게 하면 잡담이 명령이 되고, 조이면
명령이 안 먹는다. 이 둘은 같은 손잡이의 양쪽이라 한쪽만 보고 고치면 반드시
반대쪽이 깨진다 — 실제로 "한번"을 1번 변형에 넣었더니 "한번 해볼까"가
지점 이동으로 걸렸다(2026-08-13). 그래서 **걸려야 할 말**과 **걸리면 안 되는
말**을 한 표에 같이 둔다. 낱말을 추가·수정할 때는 여기부터 돌릴 것.

    python tools/check_intents.py          # 프로젝트 루트에서
    python tools/check_intents.py "들린 말"  # 한 문장만 즉석 확인

종료 코드: 0=전부 통과, 1=실패 있음.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))

from tomato_picker.voice import korean  # noqa: E402
from tomato_picker.voice.intents import match_intent  # noqa: E402

# (발화, 기대 인텐트명, 확인할 슬롯). 기대가 None이면 **아무것도 걸리면 안 된다**.
CASES: list[tuple[str, str | None, dict]] = [
    # --- 수확: 위(2층) ---
    ("위 토마토 따줘", "tomato_pick", {"height": "upper", "explicit": True}),
    ("위쪽 토마토 수확해", "tomato_pick", {"height": "upper", "explicit": True}),
    ("2층 토마토 따줘", "tomato_pick", {"height": "upper", "explicit": True}),
    ("이층 토마토", "tomato_pick", {"height": "upper", "explicit": True}),
    ("상단 토마토 따", "tomato_pick", {"height": "upper", "explicit": True}),
    # --- 수확: 아래(1층) ---
    ("아래 토마토 따줘", "tomato_pick", {"height": "lower", "explicit": True}),
    ("아래쪽 토마토 수확해줘", "tomato_pick", {"height": "lower", "explicit": True}),
    ("1층 토마토 따줘", "tomato_pick", {"height": "lower", "explicit": True}),
    ("일층 토마토", "tomato_pick", {"height": "lower", "explicit": True}),
    ("밑에 토마토 따줘", "tomato_pick", {"height": "lower", "explicit": True}),
    ("하단 토마토 수확", "tomato_pick", {"height": "lower", "explicit": True}),
    # --- 수확: 높이를 못 들으면 기본값(위)으로 가되 explicit=False로 남긴다 ---
    ("토마토", "tomato_pick", {"height": "upper", "explicit": False}),
    ("토마토 따줘", "tomato_pick", {"height": "upper", "explicit": False}),
    ("수확해줘", "tomato_pick", {"height": "upper", "explicit": False}),
    # --- 수확: 오인식 (자음 세기·받침·ㅐ/ㅔ는 발음을 접으면 사라진다) ---
    ("도마도 따줘", "tomato_pick", {"height": "upper"}),
    ("또마또", "tomato_pick", {"height": "upper"}),
    ("도마토 따줘", "tomato_pick", {"height": "upper"}),
    ("아래 도마도 따줘", "tomato_pick", {"height": "lower"}),
    ("아레 토마토 따줘", "tomato_pick", {"height": "lower"}),
    ("알래 토마토 따줘", "tomato_pick", {"height": "lower"}),
    ("도루도", "tomato_pick", {"height": "upper"}),
    ("도마뚱", "tomato_pick", {"height": "upper"}),
    ("수학해줘", "tomato_pick", {"height": "upper"}),
    # --- 지점 이동: 말하는 번호는 화분 번호(1~3) → 지점 0~2 ---
    ("1번 이동", "station_move", {"station": 0}),
    ("2번 이동", "station_move", {"station": 1}),
    ("3번 이동", "station_move", {"station": 2}),
    ("일번 이동해", "station_move", {"station": 0}),
    ("이번 이동해줘", "station_move", {"station": 1}),
    ("삼번으로 이동", "station_move", {"station": 2}),
    ("2번으로 가줘", "station_move", {"station": 1}),
    ("1번으로 가", "station_move", {"station": 0}),
    ("3번 지점으로 이동해", "station_move", {"station": 2}),
    ("어 그럼 1번으로 이동해줘", "station_move", {"station": 0}),
    ("두번째로 이동", "station_move", {"station": 1}),
    ("일번 이돈", "station_move", {"station": 0}),
    ("이번 이똥해", "station_move", {"station": 1}),
    # --- 바구니(지점3): 낱말 자체가 특이해 이동 낱말 없이도 인정 ---
    ("바구니로 이동", "station_move", {"station": 3}),
    ("바구니", "station_move", {"station": 3}),
    ("빠구니로 가줘", "station_move", {"station": 3}),
    # --- 지점 이름이 "토마토1"이라 수확과 헷갈릴 수 있는 문장 ---
    ("토마토 1번으로 이동해", "station_move", {"station": 0}),
    # --- 기존 인텐트 ---
    ("팔 움직여", "arm_move", {}),
    ("팔움직여줘", "arm_move", {}),
    ("파를 움직여봐", "arm_move", {}),
    ("앞으로 가", "drive_forward", {}),
    ("전진해", "drive_forward", {}),
    # --- 걸리면 안 되는 말 (바퀴가 구르는 명령이라 여기가 더 중요하다) ---
    ("이번 주에 비가 온대", None, {}),
    ("그래서 이번에는 다르게 해보자", None, {}),
    ("한번 해볼까", None, {}),
    ("한번 해보자", None, {}),
    ("두 번 정도 해봤어", None, {}),
    ("안녕하세요 반갑습니다", None, {}),
    ("오늘 날씨 좋네요", None, {}),
    ("네 알겠습니다", None, {}),
    ("이 영상은", None, {}),          # Whisper가 무음에서 잘 내는 환각
    ("잠깐만요", None, {}),
]


def _describe(text: str) -> str:
    got = match_intent(text)
    if got is None:
        return "(매칭 없음)"
    return got.label + (f"  {got.slots}" if got.slots else "")


def main() -> int:
    if len(sys.argv) > 1:                     # 한 문장만 즉석 확인
        for arg in sys.argv[1:]:
            print(f"{arg!r} → {_describe(arg)}    [발음: {korean.phonemes(arg)}]")
        return 0

    fails = []
    for text, want_name, want_slots in CASES:
        got = match_intent(text)
        ok = (got.name if got else None) == want_name
        if ok and got is not None:
            ok = all(got.slots.get(k) == v for k, v in want_slots.items())
        print(f"{'OK  ' if ok else 'FAIL'} {text!r:<28} → {_describe(text)}")
        if not ok:
            fails.append((text, want_name, want_slots, got))

    print(f"\n{len(CASES) - len(fails)}/{len(CASES)} 통과")
    for text, want_name, want_slots, got in fails:
        print(f"  FAIL {text!r}: 기대={want_name}{want_slots} "
              f"실제={got.name if got else None}{got.slots if got else ''}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
