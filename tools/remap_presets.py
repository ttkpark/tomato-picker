#!/usr/bin/env python3
"""캘리브레이션이 바뀌어도 **프리셋의 물리 자세를 유지**시키는 변환기.

왜 필요한가 — `arm_presets.json`에 저장되는 `.pos` 값은 각도가 아니라
**캘리브레이션에 상대적인 정규화값**이다. 그래서 팔의 range/homing을 다시 잡으면
같은 숫자가 **다른 자세**를 가리키게 되고, 힘들게 교시한 슬롯 0~9가 통째로 어긋난다
(리더암으로 다시 다 잡아야 함). 이 도구는 옛 캘리브레이션으로 숫자를 물리 위치로
되돌린 뒤 새 캘리브레이션 기준으로 다시 적어, 같은 자세를 그대로 유지시킨다.

lerobot 0.5.x `motors_bus.py`의 정규화 공식 그대로:

    RANGE_M100_100 (팔 관절 5개):  norm = ((raw-min)/(max-min))*200 - 100
    RANGE_0_100    (그리퍼):        norm = ((raw-min)/(max-min))*100

homing_offset은 서보 EEPROM에 들어가 raw 읽기값에 이미 반영되므로, 절대 엔코더
위치는 `raw - homing` 이다. 캘리브레이션이 바뀌면 homing도 바뀔 수 있어 그 차이까지
같이 보정한다.

사용법 (젯슨):
    # 1) 재캘리브레이션 **전에** 지금 캘리브레이션을 복사해 둔다
    cp ~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json \\
       ~/cal_before.json
    # 2) lerobot-calibrate 등으로 범위를 다시 잡는다
    # 3) 프리셋을 새 기준으로 변환한다
    python3 tools/remap_presets.py --old ~/cal_before.json \\
        --new ~/.cache/huggingface/lerobot/calibration/robots/so_follower/tomato_follower.json \\
        --presets ~/arm_presets.json
    #    (--apply 없이 돌리면 무엇이 어떻게 바뀌는지 보여주기만 한다)
"""

from __future__ import annotations

import argparse
import json
import os
import shutil

# 그리퍼만 0~100, 나머지 관절은 -100~100 (lerobot SO-101 정의와 동일)
RANGE_0_100_JOINTS = {"gripper"}


def _load(path: str) -> dict:
    with open(os.path.expanduser(path), encoding="utf-8") as f:
        return json.load(f)


def _norm_to_raw(norm: float, cal: dict, zero_to_100: bool) -> float:
    lo, hi = cal["range_min"], cal["range_max"]
    if cal.get("drive_mode"):
        norm = (100 - norm) if zero_to_100 else -norm
    if zero_to_100:
        return (min(100.0, max(0.0, norm)) / 100.0) * (hi - lo) + lo
    return ((min(100.0, max(-100.0, norm)) + 100) / 200.0) * (hi - lo) + lo


def _raw_to_norm(raw: float, cal: dict, zero_to_100: bool) -> float:
    lo, hi = cal["range_min"], cal["range_max"]
    raw = min(hi, max(lo, raw))          # 새 범위 밖이면 잘린다(아래에서 경고)
    if zero_to_100:
        norm = ((raw - lo) / (hi - lo)) * 100.0
        return (100 - norm) if cal.get("drive_mode") else norm
    norm = (((raw - lo) / (hi - lo)) * 200.0) - 100.0
    return -norm if cal.get("drive_mode") else norm


def convert(pose: dict, old: dict, new: dict) -> tuple[dict, list[str]]:
    """한 자세를 새 캘리브레이션 기준으로 변환. (새 자세, 경고목록)"""
    out, warnings = {}, []
    for key, value in pose.items():
        joint = key.split(".")[0]
        if joint not in old or joint not in new:
            out[key] = value
            warnings.append(f"{joint}: 캘리브레이션에 없어 그대로 둠")
            continue
        zero_100 = joint in RANGE_0_100_JOINTS
        raw_old = _norm_to_raw(float(value), old[joint], zero_100)
        # 절대 엔코더 위치로 환산 → 새 homing 기준으로 되돌리기
        absolute = raw_old - old[joint].get("homing_offset", 0)
        raw_new = absolute + new[joint].get("homing_offset", 0)
        lo, hi = new[joint]["range_min"], new[joint]["range_max"]
        if not (lo <= raw_new <= hi):
            warnings.append(
                f"{joint}: 새 범위({lo}~{hi}) 밖 {raw_new:.0f} — 끝값으로 잘림. "
                "그 자세는 새 범위로 도달할 수 없다는 뜻이다."
            )
        out[key] = round(_raw_to_norm(raw_new, new[joint], zero_100), 2)
    return out, warnings


def main() -> None:
    ap = argparse.ArgumentParser(description="캘리브레이션 변경에 맞춰 프리셋 재매핑")
    ap.add_argument("--old", required=True, help="변경 **전** 캘리브레이션 JSON")
    ap.add_argument("--new", required=True, help="변경 **후** 캘리브레이션 JSON")
    ap.add_argument("--presets", default="~/arm_presets.json")
    ap.add_argument("--apply", action="store_true",
                    help="실제로 덮어쓴다(생략하면 미리보기만). 원본은 .bak으로 남는다")
    args = ap.parse_args()

    old, new = _load(args.old), _load(args.new)
    presets_path = os.path.expanduser(args.presets)
    presets = _load(presets_path)

    print("=== 캘리브레이션 차이 ===")
    for joint in old:
        if joint not in new:
            continue
        o, n = old[joint], new[joint]
        if (o["range_min"], o["range_max"], o.get("homing_offset")) != \
           (n["range_min"], n["range_max"], n.get("homing_offset")):
            print(f"  {joint:15s} {o['range_min']}~{o['range_max']}(h{o.get('homing_offset')})"
                  f"  →  {n['range_min']}~{n['range_max']}(h{n.get('homing_offset')})")

    print("\n=== 프리셋 변환 ===")
    changed, all_warnings = {}, []
    for slot in sorted((k for k in presets if k.isdigit()), key=int):
        pose = presets[slot]
        if not isinstance(pose, dict) or not pose:
            continue
        new_pose, warnings = convert(pose, old, new)
        deltas = [f"{k.split('.')[0]} {pose[k]:+.1f}→{new_pose[k]:+.1f}"
                  for k in pose if abs(new_pose[k] - pose[k]) > 0.05]
        print(f"  슬롯{slot}: " + (", ".join(deltas) if deltas else "변화 없음"))
        for w in warnings:
            print(f"      ⚠ {w}")
            all_warnings.append(f"슬롯{slot} {w}")
        changed[slot] = new_pose

    if not args.apply:
        print("\n미리보기입니다. 실제로 적용하려면 --apply 를 붙이세요.")
        return

    backup = presets_path + ".bak"
    shutil.copy(presets_path, backup)
    for slot, pose in changed.items():
        presets[slot] = pose
    tmp = presets_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(presets, f, ensure_ascii=False, indent=2)
    os.replace(tmp, presets_path)
    print(f"\n적용 완료. 원본 백업: {backup}")
    if all_warnings:
        print("⚠ 범위 밖으로 잘린 관절이 있습니다 — 해당 슬롯은 재생해보고 확인하세요.")


if __name__ == "__main__":
    main()
