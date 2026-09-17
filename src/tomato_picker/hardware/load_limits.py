"""팔이 **들 수 있는** 자리의 경계 — 가동범위와는 다른 종류의 한계다.

가동범위(`cartesian.NormLimits`)는 "그 관절각이 있는가"를 묻고, 여기는
"그 자세를 **서보가 버티는가**"를 묻는다. 둘은 갈린다: 2026-09-18 실기에서
관절각으로는 멀쩡히 풀리는 자리에 보냈는데 `shoulder_lift`·`shoulder_pan`이
한계에 눌려 목표보다 z가 73mm 처진 채 멈췄다(docs/인수인계-2026-09-04.md §24).

⚠ **경계를 코드에 박지 않는다.** 숫자는 `config.ARM_LOAD_R_MAX`(실측 당시 값)와
`~/arm_load_limits.json`(이 팔에서 다시 잰 값, 이쪽이 이긴다)에서만 온다.
파일이 깨졌거나 "모른다"고 말하면 **짐작으로 메우지 않고 None**을 준다 —
부르는 쪽이 기록에 `load_limits=none`을 남겨, 그 시험이 무엇을 못 봤는지가
남게 한다(`cartesian.load_norm_limits`와 같은 규칙).

⚠ 이 모듈은 **포트를 열지 않는다**(`escape.py`·`settle.py`와 같은 규칙).
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass

from ..config import ARM_LOAD_LIMITS_FILE, ARM_LOAD_R_MAX


@dataclass(frozen=True)
class LoadLimits:
    """이 팔이 버티는 자리의 경계. 지금은 수평 사거리 하나뿐이다.

    왜 r 하나인가 — §24의 9자세에서 **가른 것이 z가 아니라 r**이었다. 같은
    r에서 z를 138~415mm까지 흔들어도 결과가 안 바뀌었고, r만 넘기면 z와
    무관하게 눌렸다. 표본이 더 쌓여 r별 z 상한이 필요해지면 그때 여기에
    항목을 늘린다(읽는 쪽은 `rejects()` 하나만 부르므로 안 바뀐다).
    """

    r_max: float          # pan축에서 잰 수평 사거리 상한(mm)
    source: str           # 이 숫자가 어디서 왔나 — 기록에 그대로 들어간다
    note: str = ""        # 사람이 읽을 한 줄

    def rejects(self, x: float, y: float) -> str:
        """그 자리가 경계 밖이면 이유 한 줄, 안이면 "".

        ⚠ 넣는 자리는 **팔이 실제로 서는 곳**이다. 표적이 아니라 스탠드오프
        지점을 넣어야 한다(팔이 가는 곳은 거기다 — `move5_check.standoff_pose`).
        """
        r = math.hypot(x, y)
        if r > self.r_max:
            return f"수평 {r:.0f}mm > 들 수 있는 한계 {self.r_max:.0f}mm"
        return ""


def load_load_limits(path: str = ARM_LOAD_LIMITS_FILE,
                     default_r_max: float = ARM_LOAD_R_MAX,
                     ) -> tuple[LoadLimits | None, str]:
    """(limits, 한 줄 설명). 모르면 (None, 이유).

    ① 파일이 있으면 파일이 이긴다 — 이 팔에서 다시 잰 값이기 때문이다.
    ② 파일이 없으면 config의 실측 당시 값.
    ③ 파일이 깨졌거나 r_max_mm가 null·0 이하면 **None**. 깨진 파일을 보고
       조용히 config로 돌아가면, 넓히려고 쓴 파일이 무시된 줄도 모른 채
       옛 경계로 시험하게 된다(그 기록은 다음 사이클이 사실로 믿는다).
    """
    full = os.path.expanduser(path)
    if os.path.exists(full):
        try:
            data = json.load(open(full, encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001 - 깨진 파일은 '모른다'로 친다
            return None, f"{full}을 읽을 수 없다 ({exc}) — 경계를 모른다"
        if not isinstance(data, dict) or "r_max_mm" not in data:
            return None, f"{full}에 r_max_mm가 없다 — 경계를 모른다"
        raw = data["r_max_mm"]
        if raw is None:
            return None, f"{full}이 r_max_mm를 모른다고 말한다(null)"
        try:
            r_max = float(raw)
        except (TypeError, ValueError):
            return None, f"{full}의 r_max_mm가 숫자가 아니다 ({raw!r})"
        if r_max <= 0:
            return None, f"{full}의 r_max_mm가 {r_max} — 경계로 쓸 수 없다"
        note = str(data.get("note", "")) or f"{full}에서 읽음"
        return (LoadLimits(r_max=r_max, source=full, note=note),
                f"수평 한계 {r_max:.0f}mm ← {full}")
    src = "config.ARM_LOAD_R_MAX"
    return (LoadLimits(r_max=float(default_r_max), source=src,
                       note="2026-09-18 실기 9자세(§24) — 310~330mm 사이는 아직 모른다"),
            f"수평 한계 {default_r_max:.0f}mm ← {src} ({full} 없음)")
