"""한국어 STT 오인식에 강한 근사 매칭 — 음절을 '거친 발음'으로 접어서 비교한다.

**왜 필요한가.** Whisper base는 짧은 명령어에서 자음의 세기를 자주 헷갈린다:
"토마토"가 "도마도/또마또/도마토"로, "팔 움직여"가 "파울 문지겨"로 나온다.
지금까지는 관찰된 오인식을 하나씩 손으로 목록에 넣어 왔는데(config의
VOICE_PICK_WORDS 주석 참고), 그 방식은 **새로 나타나는 오인식을 못 잡는다** —
현장에서 처음 보는 변형이 나오면 그냥 무시된다.

**어떻게 접는가.** 한국어 오인식은 무작위가 아니라 몇 갈래로 몰린다:

1. **파열음의 세기** — 예사/된/거센소리는 STT가 거의 구분하지 못한다.
   ㄷ·ㄸ·ㅌ → `t`, ㄱ·ㄲ·ㅋ → `k`, ㅂ·ㅃ·ㅍ → `p`, ㅈ·ㅉ·ㅊ → `c`, ㅅ·ㅆ → `s`.
2. **받침 끝소리 규칙** — 한국어는 받침을 7가지 소리로만 낸다(ㄷㅅㅆㅈㅊㅌㅎ은
   모두 [ㄷ]). 실제 발음이 같으니 표기 차이는 버린다.
3. **ㅐ/ㅔ 합류** — 현대 한국어 화자는 구분해 발음하지 않는다("아래"="아레").
4. **초성 ㅇ은 소리가 없다** — 버린다("이동"과 "동"의 거리를 실제 소리대로).

이렇게 접으면 "도마도·또마또·도마토"는 "토마토"와 **글자 그대로 같아진다**
(편집거리 0). 남는 변형만 짧은 편집거리로 흡수한다.

**왜 편집거리를 크게 안 주는가.** 명령이 실제로 바퀴를 굴리고 팔을 움직인다.
느슨하게 열면 잡담이 명령이 된다. 그래서 예산(`budget`)은 길이에 비례해
짜게 주고, 짧은 낱말("위" 같은)은 아예 정확히 일치할 때만 인정한다.
대신 여러 후보 중 **가장 가까운 하나**만 고르게 해서(`best`) 비슷한 낱말끼리
(일층/이층, 일번/이번) 서로를 잡아먹지 않게 한다.

디버깅: `python -m tomato_picker.voice.korean "위 토마토 따줘"`
"""

from __future__ import annotations

_CHO = "ㄱㄲㄴㄷㄸㄹㅁㅂㅃㅅㅆㅇㅈㅉㅊㅋㅌㅍㅎ"
_JUNG = "ㅏㅐㅑㅒㅓㅔㅕㅖㅗㅘㅙㅚㅛㅜㅝㅞㅟㅠㅡㅢㅣ"
_JONG = " ㄱㄲㄳㄴㄵㄶㄷㄹㄺㄻㄼㄽㄾㄿㅀㅁㅂㅄㅅㅆㅇㅈㅊㅋㅌㅍㅎ"

# 초성 → 거친 소리. ㅇ은 소리가 없으므로 빈 문자열.
_CHO_SOUND = {
    "ㄱ": "k", "ㄲ": "k", "ㅋ": "k",
    "ㄴ": "n",
    "ㄷ": "t", "ㄸ": "t", "ㅌ": "t",
    "ㄹ": "r",
    "ㅁ": "m",
    "ㅂ": "p", "ㅃ": "p", "ㅍ": "p",
    "ㅅ": "s", "ㅆ": "s",
    "ㅇ": "",
    "ㅈ": "c", "ㅉ": "c", "ㅊ": "c",
    "ㅎ": "h",
}

# 중성 → 거친 소리. ㅐ/ㅔ와 ㅙ/ㅚ/ㅞ는 실제로 구분되지 않아 합류시킨다.
# 이중모음은 두 글자로 풀어 써서, 단모음과의 거리가 1이 되게 한다.
_JUNG_SOUND = {
    "ㅏ": "a", "ㅐ": "E", "ㅑ": "ya", "ㅒ": "yE",
    "ㅓ": "v", "ㅔ": "E", "ㅕ": "yv", "ㅖ": "yE",
    "ㅗ": "o", "ㅘ": "wa", "ㅙ": "wE", "ㅚ": "wE", "ㅛ": "yo",
    "ㅜ": "u", "ㅝ": "wv", "ㅞ": "wE", "ㅟ": "wi", "ㅠ": "yu",
    "ㅡ": "z", "ㅢ": "zi", "ㅣ": "i",
}

# 받침 → 끝소리 규칙(7종성). 겹받침은 실제로 나는 쪽으로.
_JONG_SOUND = {
    " ": "",
    "ㄱ": "k", "ㄲ": "k", "ㅋ": "k", "ㄳ": "k", "ㄺ": "k",
    "ㄴ": "n", "ㄵ": "n", "ㄶ": "n",
    "ㄷ": "t", "ㅅ": "t", "ㅆ": "t", "ㅈ": "t", "ㅊ": "t", "ㅌ": "t", "ㅎ": "t",
    "ㄹ": "r", "ㄼ": "r", "ㄽ": "r", "ㄾ": "r", "ㅀ": "r",
    "ㅁ": "m", "ㄻ": "m",
    "ㅂ": "p", "ㅍ": "p", "ㅄ": "p", "ㄿ": "p",
    "ㅇ": "N",
}

# 숫자는 한자어 수사로 읽어 한글 표기와 하나로 만든다("1번"="일번", "2층"="이층").
_DIGIT_READING = {
    "0": "영", "1": "일", "2": "이", "3": "삼", "4": "사",
    "5": "오", "6": "육", "7": "칠", "8": "팔", "9": "구",
}


def phonemes(text: str) -> str:
    """텍스트를 거친 발음 문자열로 접는다. 한글·숫자가 아닌 건 모두 버린다.

    공백과 문장부호를 버리므로 띄어쓰기 차이("팔 움직여"/"팔움직여")는
    자동으로 흡수된다.
    """
    out: list[str] = []
    for ch in text:
        if ch in _DIGIT_READING:
            ch = _DIGIT_READING[ch]
        code = ord(ch) - 0xAC00
        if not 0 <= code < 11172:
            continue                      # 한글 음절이 아니면 버린다
        cho, rest = divmod(code, 21 * 28)
        jung, jong = divmod(rest, 28)
        out.append(_CHO_SOUND[_CHO[cho]])
        out.append(_JUNG_SOUND[_JUNG[jung]])
        out.append(_JONG_SOUND[_JONG[jong]])
    return "".join(out)


def budget(needle_len: int) -> int:
    """길이별로 허용할 편집 횟수.

    짧을수록 짜게 준다 — 2~3음소짜리 낱말에 1회만 허용해도 전혀 다른 말이
    걸린다("위"에 "이"·"의"·"우"가 다 붙는다). 긴 낱말은 오인식이 여러 군데
    나도 원말을 알아볼 수 있으므로 넉넉히 준다.
    """
    if needle_len < 4:
        return 0
    if needle_len < 7:
        return 1
    if needle_len < 10:
        return 2
    return 3


def distance(text_ph: str, needle_ph: str) -> int:
    """needle이 text 안에 **부분 문자열로** 나타나기까지의 최소 편집 횟수.

    Sellers 알고리즘 — 시작·끝 위치가 자유로운 편집거리다. 앞뒤에 무슨 말이
    붙어 있든("어 그럼 위 토마토 따줘") 가운데 낱말만 찾아낸다.
    """
    if not needle_ph:
        return len(text_ph)
    # prev[j] = needle 0글자를 text의 j번째까지 중 어딘가에 맞추는 비용 = 0
    prev = [0] * (len(text_ph) + 1)
    for i, nc in enumerate(needle_ph, start=1):
        cur = [i]                                    # text가 비면 needle을 다 지워야 한다
        for j, tc in enumerate(text_ph, start=1):
            cur.append(min(
                prev[j] + 1,                         # needle 글자 삭제
                cur[j - 1] + 1,                      # text 글자 삽입
                prev[j - 1] + (nc != tc),            # 치환(같으면 공짜)
            ))
        prev = cur
    return min(prev)


def best(text: str, variants: list[str]) -> tuple[str, int] | None:
    """변형 목록 중 **가장 가까운** 하나와 그 거리. 예산을 넘으면 None.

    가장 가까운 하나만 고르는 게 핵심이다 — "일층"과 "이층"은 서로 1회
    편집 거리라 둘 다 "걸릴" 수 있는데, 거리를 같이 돌려주면 부르는 쪽이
    더 가까운 후보를 골라 위/아래를 가릴 수 있다.
    """
    text_ph = phonemes(text)
    found: tuple[str, int] | None = None
    for variant in variants:
        needle = phonemes(variant)
        if not needle:
            continue
        dist = distance(text_ph, needle)
        if dist > budget(len(needle)):
            continue
        if found is None or dist < found[1]:
            found = (variant, dist)
    return found


def contains(text: str, variants: list[str]) -> bool:
    """변형 중 하나라도 근사 일치하면 True."""
    return best(text, variants) is not None


if __name__ == "__main__":  # 현장 디버깅: 들린 말이 어떻게 접히는지 눈으로 본다
    import sys

    for arg in sys.argv[1:] or ["토마토", "도마도", "또마또", "위", "아래"]:
        print(f"{arg!r:>20} → {phonemes(arg)}")
