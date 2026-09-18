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

from ..config import (
    ARM_LOAD_LIMITS_FILE,
    ARM_LOAD_R_MAX,
    ARM_LOAD_WFLEX_MAX_DEG,
    ARM_LOAD_Z_MAX,
)


@dataclass(frozen=True)
class LoadLimits:
    """이 팔이 버티는 자리의 경계. 수평 사거리(r), 높이(z), 손목 각도(wflex) 상한을 갖는다.

    §24의 실측과 §30(T62), §31(T68)의 실측으로 확정된 경계다:
    - 수평 사거리 r: pan축에서 잰 수평거리(mm).
    - 높이 z: base_link 기준 높이 상한(mm). z=440mm까지는 wflex<=0°에서
      수렴하지만(T68), z>=456mm는 서보 토크 및 모멘트 암 한계로 처짐이 91~103mm
      발생하며 실패한다(T57).
    - 손목 굽힘 wflex: wrist_flex 각도 상한(도). wflex>0°(위로 꺾임)는 모멘트 암을
      비틀어 되먹임 진동 및 발산(오차 48.1mm)을 유발하므로 거절한다(T68).
    """

    r_max: float          # pan축에서 잰 수평 사거리 상한(mm)
    z_max: float = ARM_LOAD_Z_MAX  # base_link 기준 높이 상한(mm)
    wflex_max_deg: float = ARM_LOAD_WFLEX_MAX_DEG  # wrist_flex 각도 상한(도)
    source: str = ""      # 이 숫자가 어디서 왔나 — 기록에 그대로 들어간다
    note: str = ""        # 사람이 읽을 한 줄

    def rejects(self, x: float, y: float, z: float | None = None,
                wflex_deg: float | None = None) -> str:
        """그 자리가 경계 밖이면 이유 한 줄, 안이면 "".

        ⚠ 넣는 자리는 **팔이 실제로 서는 곳**이다. 표적이 아니라 스탠드오프
        지점을 넣어야 한다(팔이 가는 곳은 거기다 — `move5_check.standoff_pose`).
        """
        r = math.hypot(x, y)
        if r > self.r_max:
            return f"수평 {r:.0f}mm > 들 수 있는 한계 {self.r_max:.0f}mm"
        if z is not None and z > self.z_max:
            return f"높이 {z:.0f}mm > 들 수 있는 한계 {self.z_max:.0f}mm"
        if wflex_deg is not None and wflex_deg > self.wflex_max_deg:
            return f"손목 {wflex_deg:+.1f}° > 들 수 있는 한계 {self.wflex_max_deg:+.1f}°"
        return ""


def load_load_limits(path: str = ARM_LOAD_LIMITS_FILE,
                     default_r_max: float = ARM_LOAD_R_MAX,
                     default_z_max: float = ARM_LOAD_Z_MAX,
                     default_wflex_max_deg: float = ARM_LOAD_WFLEX_MAX_DEG,
                     ) -> tuple[LoadLimits | None, str]:
    """(limits, 한 줄 설명). 모르면 (None, 이유).

    ① 파일이 있으면 파일이 이긴다 — 이 팔에서 다시 잰 값이기 때문이다.
    ② 파일이 없으면 config의 실측 당시 값.
    ③ 파일이 깨졌거나 r_max_mm/z_max_mm/wflex_max_deg가 null 또는 비정상 값이면 **None**.
       깨진 파일을 보고 조용히 config로 돌아가면, 넓히려고 쓴 파일이 무시된 줄도 모른 채
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
        raw_r = data["r_max_mm"]
        if raw_r is None:
            return None, f"{full}이 r_max_mm를 모른다고 말한다(null)"
        try:
            r_max = float(raw_r)
        except (TypeError, ValueError):
            return None, f"{full}의 r_max_mm가 숫자가 아니다 ({raw_r!r})"
        if r_max <= 0:
            return None, f"{full}의 r_max_mm가 {r_max} — 경계로 쓸 수 없다"

        # z_max_mm는 파일에 명시되어 있으면 파싱, 없으면 기본 config값
        if "z_max_mm" in data:
            raw_z = data["z_max_mm"]
            if raw_z is None:
                return None, f"{full}이 z_max_mm를 모른다고 말한다(null)"
            try:
                z_max = float(raw_z)
            except (TypeError, ValueError):
                return None, f"{full}의 z_max_mm가 숫자가 아니다 ({raw_z!r})"
            if z_max <= 0:
                return None, f"{full}의 z_max_mm가 {z_max} — 경계로 쓸 수 없다"
        else:
            z_max = float(default_z_max)

        # wflex_max_deg는 파일에 명시되어 있으면 파싱, 없으면 기본 config값
        if "wflex_max_deg" in data:
            raw_w = data["wflex_max_deg"]
            if raw_w is None:
                return None, f"{full}이 wflex_max_deg를 모른다고 말한다(null)"
            try:
                wflex_max = float(raw_w)
            except (TypeError, ValueError):
                return None, f"{full}의 wflex_max_deg가 숫자가 아니다 ({raw_w!r})"
        else:
            wflex_max = float(default_wflex_max_deg)

        note = str(data.get("note", "")) or f"{full}에서 읽음"
        return (LoadLimits(r_max=r_max, z_max=z_max, wflex_max_deg=wflex_max, source=full, note=note),
                f"한계 r={r_max:.0f}mm, z={z_max:.0f}mm, wflex={wflex_max:+.0f}° ← {full}")
    src = "config.ARM_LOAD_R_MAX+Z_MAX+WFLEX_MAX"
    return (LoadLimits(r_max=float(default_r_max), z_max=float(default_z_max),
                       wflex_max_deg=float(default_wflex_max_deg), source=src,
                       note="2026-09-18 실기 (§24, §30, §31 실측)"),
            f"한계 r={default_r_max:.0f}mm, z={default_z_max:.0f}mm, wflex={default_wflex_max_deg:+.0f}° ← {src} ({full} 없음)")
