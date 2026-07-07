"""인식된 텍스트를 인텐트로 매칭 — 키워드 부분 문자열, 순서 무관."""

from __future__ import annotations

from ..config import VOICE_INTENTS


def match_intent(text: str) -> str | None:
    compact = text.replace(" ", "")
    for intent, keywords in VOICE_INTENTS.items():
        for kw in keywords:
            if kw.replace(" ", "") in compact:
                return intent
    return None
