"""Claude 함수콜 오케스트레이션 — 자연어 명령 → 스킬 함수 계획·실행.

스킬 함수를 Claude에 tool로 노출하고, 수동 agentic 루프로 Claude가
요청하는 도구를 실행해 결과를 돌려준다. 망/ API가 필요하므로,
실패 시 호출 측(main)에서 demo_mode 폴백으로 떨어진다.
"""

from __future__ import annotations

import json

import anthropic

from .config import CLAUDE_MODEL, TREES
from .skills import TomatoPicker

SYSTEM_PROMPT = (
    "너는 토마토 따기 로봇의 컨트롤러다. 사용자의 자연어 명령을 "
    "제공된 도구(스킬 함수) 호출로 변환해 수행한다.\n"
    f"무대에는 나무 {sorted(TREES)}가 일렬로 있다.\n"
    "수확 규칙: 한 나무 앞으로 이동(drive_to_tree) → 스캔(scan_tree) → "
    "익은(ripe=true) 열매만 pick_fruit 후 place_in_basket. 초록 열매는 건너뛴다.\n"
    "여러 나무를 순회할 수 있다. 모든 작업이 끝나면 home으로 복귀한다."
)

# 스킬 함수 → Claude tool 스키마. 이름은 TomatoPicker 메서드명과 일치시킨다.
TOOLS = [
    {
        "name": "drive_to_tree",
        "description": "지정한 나무 앞으로 직선 이동한다.",
        "input_schema": {
            "type": "object",
            "properties": {"tree_id": {"type": "integer", "description": "나무 번호"}},
            "required": ["tree_id"],
        },
    },
    {
        "name": "scan_tree",
        "description": "현재 나무를 카메라로 스캔해 열매 목록(위치/익음 여부)을 반환한다.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "pick_fruit",
        "description": "직전 scan_tree 결과의 fruit id로 해당 열매를 딴다.",
        "input_schema": {
            "type": "object",
            "properties": {"fruit_id": {"type": "integer"}},
            "required": ["fruit_id"],
        },
    },
    {
        "name": "place_in_basket",
        "description": "그리퍼에 든 열매를 바구니에 담는다.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "home",
        "description": "로봇팔을 홈 포지션으로 복귀시킨다.",
        "input_schema": {"type": "object", "properties": {}},
    },
]


def run_command(robot: TomatoPicker, command: str, max_steps: int = 50) -> str:
    """자연어 명령 하나를 Claude로 계획·실행하고 최종 답변 텍스트를 반환."""
    client = anthropic.Anthropic()  # ANTHROPIC_API_KEY 환경변수 사용
    messages: list[dict] = [{"role": "user", "content": command}]

    for _ in range(max_steps):
        response = client.messages.create(
            model=CLAUDE_MODEL,
            max_tokens=4096,
            thinking={"type": "adaptive"},
            system=SYSTEM_PROMPT,
            tools=TOOLS,
            messages=messages,
        )
        messages.append({"role": "assistant", "content": response.content})

        if response.stop_reason != "tool_use":
            return "".join(b.text for b in response.content if b.type == "text")

        results = []
        for block in response.content:
            if block.type != "tool_use":
                continue
            method = getattr(robot, block.name)
            output = method(**block.input)
            results.append(
                {
                    "type": "tool_result",
                    "tool_use_id": block.id,
                    "content": json.dumps(output, ensure_ascii=False),
                }
            )
        messages.append({"role": "user", "content": results})

    return "최대 단계 수 초과 — 작업을 끝내지 못했습니다."
