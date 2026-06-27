"""Claude 비전 기반 열매 검출 (대안 인식 경로).

color_detect는 단순 HSV 색검출(오프라인·빠름)이고, 이쪽은 카메라 이미지를
Claude에 직접 보내 '익었는지'를 판단시킨다. 색만으로 애매한 경우나 더 똑똑한
판단이 필요할 때 쓴다. 인터넷·API 키가 필요하므로 오프라인 폴백에선 color_detect를 쓴다.

반환 형태는 color_detect.detect_fruits와 동일한 list[Fruit]라 그대로 교체 가능.
"""

from __future__ import annotations

import base64
import json

import anthropic
import cv2
import numpy as np

from ..config import CLAUDE_MODEL
from .color_detect import Fruit

_PROMPT = (
    "이 사진은 토마토 나무다. 보이는 토마토를 모두 찾아라. "
    "각 토마토의 중심 픽셀 좌표(x, y)와 익음 여부를 판단하라. "
    "빨갛게 잘 익은 것은 ripe=true, 초록/덜 익은 것은 ripe=false."
)

# Claude가 이 스키마대로만 답하도록 강제(structured output).
_SCHEMA = {
    "type": "object",
    "properties": {
        "fruits": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "x": {"type": "integer", "description": "중심 x 픽셀"},
                    "y": {"type": "integer", "description": "중심 y 픽셀"},
                    "ripe": {"type": "boolean"},
                },
                "required": ["x", "y", "ripe"],
                "additionalProperties": False,
            },
        }
    },
    "required": ["fruits"],
    "additionalProperties": False,
}


def detect_fruits_llm(frame_bgr: np.ndarray) -> list[Fruit]:
    """BGR 프레임을 Claude에 보내 열매 목록(위치·익음)을 받는다.

    area는 LLM이 주지 않으므로 0으로 둔다. 익은 것(빨강) 우선 정렬.
    """
    ok, buf = cv2.imencode(".png", frame_bgr)
    if not ok:
        return []
    b64 = base64.standard_b64encode(buf.tobytes()).decode("ascii")

    client = anthropic.Anthropic()  # ANTHROPIC_API_KEY 환경변수 사용
    response = client.messages.create(
        model=CLAUDE_MODEL,
        max_tokens=2048,
        output_config={"format": {"type": "json_schema", "schema": _SCHEMA}},
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": "image/png",
                            "data": b64,
                        },
                    },
                    {"type": "text", "text": _PROMPT},
                ],
            }
        ],
    )
    text = next(b.text for b in response.content if b.type == "text")
    data = json.loads(text)
    fruits = [
        Fruit(position=(int(f["x"]), int(f["y"])), ripe=bool(f["ripe"]), area=0)
        for f in data.get("fruits", [])
    ]
    fruits.sort(key=lambda f: f.ripe, reverse=True)
    return fruits
